/* SPDX-License-Identifier: MIT */
#pragma once
#include <iqs915x_processing.h>
#include <stddef.h>
/* Fixed sensor-coordinate thresholds; independent of Harbour beta controls. */
struct iqs915x_iir_state {
    int64_t x_q16,y_q16;
    uint16_t previous_x,previous_y;
    uint32_t previous_ms;
    bool valid,pending,stopped,fast,quiet;
    uint32_t fast_ms,quiet_ms;
    uint16_t anchor_x,anchor_y;
    unsigned quiet_samples;
    int64_t offset_x,offset_y;
};
/* A uniform 5x5 tile, displacement in Q15 fractions of one key pitch.
 * X is odd about the key centre. Y is even in X and asymmetric in Y.
 * Opposite edges agree: normal displacement is zero, tangential displacement
 * repeats. The six bounded controls guarantee positive axis derivatives
 * (>=0.325 before quantization), hence no folded cells. */
static inline int32_t iqs915x_map_round(int64_t n,int64_t d) {
    return n<0?-(int32_t)((-n+d/2)/d):(int32_t)((n+d/2)/d);
}
static inline void iqs915x_map_generate(const struct iqs915x_processing_settings *p,int16_t map[5][5][2]) {
    int16_t a[5]={p->x_middle,p->coefficient_permille,p->x_middle,p->x_lower,p->x_middle};
    int32_t y[5]={0,-2*p->y_upper+p->y_center,2*p->y_center,2*p->y_lower+p->y_center,0};
    for(unsigned j=0;j<5;j++)for(unsigned i=0;i<5;i++) {
        map[j][i][0]=(i==1?-1:i==3?1:0)*iqs915x_map_round((int64_t)a[j]*32768,8000);
        map[j][i][1]=iqs915x_map_round((int64_t)y[j]*32768,16000);
    }
}
static inline void iqs915x_map_position(uint16_t raw,uint16_t resolution,unsigned blocks,unsigned *cell,unsigned *rem,unsigned *span) {
    unsigned block=(uint32_t)raw*blocks/resolution;if(block>=blocks)block=blocks-1;
    unsigned start=(uint32_t)resolution*block/blocks,end=(uint32_t)resolution*(block+1)/blocks;
    *span=end-start;unsigned q=(raw-start)*4;*cell=q/(*span);*rem=q%(*span);
    if(*cell>=4){*cell=3;*rem=*span;}
}
/* One final signed division, ties away from zero. Largest intermediate is
 * < 2^62 for uint16 resolutions and the bounded map. No floating point or I/O. */
static inline void iqs915x_map_apply(uint16_t *x,uint16_t *y,uint16_t rx,uint16_t ry,const int16_t map[5][5][2]) {
    if(rx<6||ry<4||*x>rx||*y>ry||*x==UINT16_MAX||*y==UINT16_MAX)return;
    unsigned i,j,u,v,w,h;iqs915x_map_position(*x,rx,6,&i,&u,&w);iqs915x_map_position(*y,ry,4,&j,&v,&h);
    if(!w||!h)return;
    uint16_t *axis[2]={x,y};unsigned pitch[2]={w,h};
    for(unsigned k=0;k<2;k++) {
        int64_t a=(int64_t)map[j][i][k]*(w-u)+(int64_t)map[j][i+1][k]*u;
        int64_t b=(int64_t)map[j+1][i][k]*(w-u)+(int64_t)map[j+1][i+1][k]*u;
        int32_t delta=iqs915x_map_round((a*(h-v)+b*v)*pitch[k],(int64_t)w*h*32768);
        *axis[k]=(uint16_t)((int32_t)*axis[k]+delta);
    }
}
static inline uint32_t iqs915x_isqrt(uint64_t n) {
    uint64_t root=0,bit=1ULL<<62;
    while(bit>n)bit>>=2;
    while(bit){if(n>=root+bit){n-=root+bit;root=(root>>1)+bit;}else root>>=1;bit>>=2;}
    return (uint32_t)root;
}
static inline unsigned iqs915x_dynamic_beta(const struct iqs915x_processing_settings *p,uint32_t speed) {
    if(speed<=p->bottom_speed)return p->bottom_beta;
    if(speed>=p->top_speed)return 256;
    return p->bottom_beta+(uint64_t)(256-p->bottom_beta)*(speed-p->bottom_speed)/(p->top_speed-p->bottom_speed);
}
static inline int64_t iqs915x_div_round(int64_t value,uint64_t denominator) {
    return value<0?-(int64_t)(((uint64_t)(-value)+denominator/2)/denominator):(int64_t)(((uint64_t)value+denominator/2)/denominator);
}
/* Fresh corrected coordinates only. Returns whether fresh tail samples are
 * still useful. The speed is raw-corrected displacement / actual elapsed time,
 * independent of both the IC report period and the previous filtered lag.
 * Variable-dt gain: beta*dt / (beta*dt+(256-beta)*15). All arithmetic is integer.
 * Fast-to-stationary motion discards lag after three samples spanning 30ms.
 * Offsets persist until contact reset; duplicate timestamps never advance state.
 * Long gaps clear velocity history, preserving stopped-contact continuity. */
static inline bool iqs915x_dynamic_iir(struct iqs915x_iir_state *s,
    const struct iqs915x_processing_settings *p,uint32_t now,uint16_t max_x,uint16_t max_y,uint16_t *x,uint16_t *y) {
    uint32_t dt=now-s->previous_ms;
    if(!s->valid || p->bottom_beta==256) {
        *s=(struct iqs915x_iir_state){.x_q16=(int64_t)*x*65536,.y_q16=(int64_t)*y*65536,
            .previous_x=*x,.previous_y=*y,.previous_ms=now,.valid=true};return false;
    }
    if(!dt){*x=(s->x_q16+32768)/65536;*y=(s->y_q16+32768)/65536;return s->pending;}
    int64_t dx=(int32_t)*x-s->previous_x,dy=(int32_t)*y-s->previous_y;
    uint32_t speed=(uint64_t)iqs915x_isqrt((uint64_t)(dx*dx+dy*dy)<<16)*1000/((uint64_t)dt*256);
    bool gap=dt>250;
    if(gap){s->fast=false;s->quiet=false;}
    bool held=s->stopped;
    s->previous_x=*x;s->previous_y=*y;s->previous_ms=now;
    if(p->stop_enabled && p->bottom_beta>0 && p->bottom_beta<256) {
        int64_t ax=(int32_t)*x-s->anchor_x,ay=(int32_t)*y-s->anchor_y;
        bool near=ax*ax+ay*ay<=(int64_t)p->stop_radius*p->stop_radius;
        if(s->stopped) {
            if(near){*x=(s->x_q16+32768)/65536;*y=(s->y_q16+32768)/65536;s->pending=false;return false;}
            s->stopped=false;
        }
        if(!gap && speed>=p->stop_fast_speed){s->fast=true;s->fast_ms=now;s->quiet=false;}
        else if(s->fast) {
            if(!s->quiet) {
                if((uint32_t)(now-s->fast_ms)>p->stop_arm_ms)s->fast=false;
                else {s->quiet=true;s->quiet_ms=now;s->quiet_samples=1;s->anchor_x=*x;s->anchor_y=*y;}
            } else if(!near) {
                s->quiet=false;
                if((uint32_t)(now-s->fast_ms)<=p->stop_arm_ms){s->quiet=true;s->quiet_ms=now;s->quiet_samples=1;s->anchor_x=*x;s->anchor_y=*y;}
                else s->fast=false;
            } else {
                if(s->quiet_samples<p->stop_samples)s->quiet_samples++;
                if(s->quiet_samples>=p->stop_samples && (uint32_t)(now-s->quiet_ms)>=p->stop_quiet_ms) {
                    uint16_t out_x=(s->x_q16+32768)/65536,out_y=(s->y_q16+32768)/65536;
                    s->x_q16=(int64_t)out_x*65536;s->y_q16=(int64_t)out_y*65536;
                    s->offset_x=(int64_t)*x*65536-s->x_q16;s->offset_y=(int64_t)*y*65536-s->y_q16;
                    s->anchor_x=*x;s->anchor_y=*y;s->stopped=true;s->fast=false;s->quiet=false;s->pending=false;
                    *x=out_x;*y=out_y;return false;
                }
            }
        }
    } else {s->fast=false;s->quiet=false;s->stopped=false;}
    unsigned beta=iqs915x_dynamic_beta(p,speed);
    if(gap && held)dt=15;
    int64_t target_x=(int64_t)*x*65536-s->offset_x,target_y=(int64_t)*y*65536-s->offset_y;
    int64_t bounded_x=target_x<0?0:target_x>(int64_t)max_x*65536?(int64_t)max_x*65536:target_x;
    int64_t bounded_y=target_y<0?0:target_y>(int64_t)max_y*65536?(int64_t)max_y*65536:target_y;
    s->offset_x+=target_x-bounded_x;s->offset_y+=target_y-bounded_y;
    target_x=bounded_x;target_y=bounded_y;
    if(gap && !held){s->x_q16=target_x;s->y_q16=target_y;}
    uint64_t numerator=(uint64_t)beta*dt,denominator=numerator+(256-beta)*15;
    if(numerator) {
        s->x_q16+=iqs915x_div_round((target_x-s->x_q16)*(int64_t)numerator,denominator);
        s->y_q16+=iqs915x_div_round((target_y-s->y_q16)*(int64_t)numerator,denominator);
    }
    int64_t ex=target_x-s->x_q16,ey=target_y-s->y_q16;
    bool pending=beta>0 && (ex>=32768||ex<=-32768||ey>=32768||ey<=-32768);
    if(!pending&&beta>0){s->x_q16=target_x;s->y_q16=target_y;}
    *x=(s->x_q16+32768)/65536;*y=(s->y_q16+32768)/65536;s->pending=pending;return pending;
}
