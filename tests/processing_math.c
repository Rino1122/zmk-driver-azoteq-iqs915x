/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdio.h>
#include <iqs915x_processing_math.h>
int main(void) {
    uint16_t lut[17];struct iqs915x_processing_settings p=IQS915X_PROCESSING_DEFAULTS;
    assert(iqs915x_processing_valid(&p));
    iqs915x_pitch_lut(500,lut);
    for(unsigned i=0;i<17;i++){unsigned u=i*2048;assert(lut[i]==u+(uint64_t)u*(32768-u)/65536);}
    assert(iqs915x_pitch_axis(250,6000,6,lut)==187);
    assert(iqs915x_pitch_axis(750,6000,6,lut)==813);
    for(int a=-900;a<=900;a+=100)for(unsigned res=5999;res<=6001;res++) {
        iqs915x_pitch_lut(a,lut);unsigned previous=0;
        for(unsigned x=0;x<=res;x++) {unsigned y=iqs915x_pitch_axis(x,res,6,lut);assert(y>=previous&&y<=res);previous=y;}
        assert(iqs915x_pitch_axis(UINT16_MAX,res,6,lut)==UINT16_MAX);
    }
    iqs915x_pitch_lut(0,lut);
    for(unsigned x=0;x<=6000;x++)assert(iqs915x_pitch_axis(x,6000,6,lut)==x);
    assert(iqs915x_isqrt(0)==0&&iqs915x_isqrt(25)==5&&iqs915x_isqrt(24)==4);
    assert(iqs915x_isqrt(2ULL*65535*65535)==92680);
    p.bottom_speed=100;p.top_speed=1000;p.bottom_beta=64;
    assert(iqs915x_dynamic_beta(&p,0)==64&&iqs915x_dynamic_beta(&p,100)==64);
    assert(iqs915x_dynamic_beta(&p,550)==160&&iqs915x_dynamic_beta(&p,1000)==256);
    assert(iqs915x_dynamic_beta(&p,1000000)==256);
    struct iqs915x_iir_state s={0};uint16_t x=1000,y=2000;
    assert(!iqs915x_dynamic_iir(&s,&p,100,&x,&y)&&x==1000&&y==2000);
    p.bottom_speed=100000;p.top_speed=200000;
    x=1100;y=2000;assert(iqs915x_dynamic_iir(&s,&p,115,&x,&y));assert(x==1025&&y==2000);
    x=1100;y=2000;for(unsigned t=130;t<2000;t+=15){x=1100;y=2000;if(!iqs915x_dynamic_iir(&s,&p,t,&x,&y))break;}
    assert(x==1100&&y==2000);
    // Timestamps wrap correctly and a long gap resets without a stale tail.
    s=(struct iqs915x_iir_state){0};x=5;y=10;iqs915x_dynamic_iir(&s,&p,UINT32_MAX-10,&x,&y);
    x=105;y=10;iqs915x_dynamic_iir(&s,&p,4,&x,&y);assert(x==30);
    x=60000;y=60000;assert(!iqs915x_dynamic_iir(&s,&p,1000,&x,&y)&&x==60000);
    // Gain 256 is exact bypass; zero gain holds and never requests tail polling.
    p.bottom_beta=256;x=61000;y=61000;assert(!iqs915x_dynamic_iir(&s,&p,1015,&x,&y)&&x==61000);
    p.bottom_beta=0;x=61001;y=61000;assert(!iqs915x_dynamic_iir(&s,&p,1030,&x,&y)&&x==61000);
    // Equal speeds at different sample intervals select the same beta;
    // the gain then compensates for elapsed time rather than pixel/cycle.
    p=(struct iqs915x_processing_settings){0,64,100,1000};
    for(unsigned dt=10;dt<=20;dt+=10) {
        struct iqs915x_iir_state state={0};x=1000;y=1000;iqs915x_dynamic_iir(&state,&p,0,&x,&y);
        x=1000+dt/2;y=1000;iqs915x_dynamic_iir(&state,&p,dt,&x,&y);
        unsigned beta=iqs915x_dynamic_beta(&p,500);
        int64_t expected=1000LL*65536+iqs915x_div_round((int64_t)(dt/2)*65536*beta*dt,beta*dt+(256-beta)*15);
        assert(state.x_q16==expected);
    }
    p.top_speed=p.bottom_speed;assert(!iqs915x_processing_valid(&p));
    puts("Processing math: periodic correction, beta thresholds, convergence, timestamps and integer limits PASS");
}
