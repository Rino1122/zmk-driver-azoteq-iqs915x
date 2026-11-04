/* SPDX-License-Identifier: MIT */
#pragma once
#include <iqs915x_processing.h>
#include <stddef.h>
struct iqs915x_iir_state {
    int64_t x_q16,y_q16;
    uint16_t previous_x,previous_y;
    uint32_t previous_ms;
    bool valid,pending;
};
static inline void iqs915x_pitch_lut(int16_t coefficient,uint16_t lut[17]) {
    for(unsigned i=0;i<17;i++) {
        int32_t u=i*2048;
        lut[i]=u+(int64_t)coefficient*u*(32768-u)/(1000LL*32768);
    }
}
static inline uint16_t iqs915x_pitch_axis(uint16_t raw,uint16_t resolution,unsigned blocks,const uint16_t lut[17]) {
    if(raw==UINT16_MAX || (resolution && raw>resolution))return UINT16_MAX;
    if(!resolution || !blocks)return raw;
    unsigned block=(uint32_t)raw*blocks/resolution;
    if(block>=blocks)block=blocks-1;
    unsigned start=(uint32_t)resolution*block/blocks,end=(uint32_t)resolution*(block+1)/blocks,center=(start+end)/2;
    if(raw==start||raw==center||raw==end)return raw;
    unsigned half=raw<center?center-start:end-center,distance=raw<center?center-raw:raw-center;
    if(!half)return raw;
    unsigned pos=distance*16,idx=pos/half,rem=pos%half;
    if(idx>=16)return raw<center?start:end;
    unsigned q15=lut[idx]+((uint32_t)(lut[idx+1]-lut[idx])*rem)/half;
    unsigned corrected=(half*q15+16384)/32768;
    return raw<center?center-corrected:center+corrected;
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
 * First touch and gaps >250ms seed immediately; duplicate timestamps are ignored. */
static inline bool iqs915x_dynamic_iir(struct iqs915x_iir_state *s,
    const struct iqs915x_processing_settings *p,uint32_t now,uint16_t *x,uint16_t *y) {
    uint32_t dt=now-s->previous_ms;
    if(!s->valid || dt>250) {
        *s=(struct iqs915x_iir_state){.x_q16=(int64_t)*x*65536,.y_q16=(int64_t)*y*65536,
            .previous_x=*x,.previous_y=*y,.previous_ms=now,.valid=true};return false;
    }
    if(!dt){*x=(s->x_q16+32768)/65536;*y=(s->y_q16+32768)/65536;return s->pending;}
    int64_t dx=(int32_t)*x-s->previous_x,dy=(int32_t)*y-s->previous_y;
    uint32_t speed=(uint64_t)iqs915x_isqrt((uint64_t)(dx*dx+dy*dy)<<16)*1000/((uint64_t)dt*256);
    unsigned beta=iqs915x_dynamic_beta(p,speed);
    s->previous_x=*x;s->previous_y=*y;s->previous_ms=now;
    uint64_t numerator=(uint64_t)beta*dt,denominator=numerator+(256-beta)*15;
    int64_t target_x=(int64_t)*x*65536,target_y=(int64_t)*y*65536;
    if(numerator) {
        s->x_q16+=iqs915x_div_round((target_x-s->x_q16)*(int64_t)numerator,denominator);
        s->y_q16+=iqs915x_div_round((target_y-s->y_q16)*(int64_t)numerator,denominator);
    }
    int64_t ex=target_x-s->x_q16,ey=target_y-s->y_q16;
    bool pending=beta>0 && (ex>=32768||ex<=-32768||ey>=32768||ey<=-32768);
    if(!pending&&beta>0){s->x_q16=target_x;s->y_q16=target_y;}
    *x=(s->x_q16+32768)/65536;*y=(s->y_q16+32768)/65536;s->pending=pending;return pending;
}
