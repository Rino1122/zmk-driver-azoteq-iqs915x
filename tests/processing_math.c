/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdio.h>
#include <iqs915x_processing_math.h>
int main(void) {
    struct iqs915x_processing_settings p=IQS915X_PROCESSING_DEFAULTS;
    assert(iqs915x_processing_valid(&p));int16_t map[5][5][2];
    iqs915x_map_generate(&p,map);
    uint16_t xx=250,yy=250;iqs915x_map_apply(&xx,&yy,6000,4000,map);assert(xx==187&&yy==187);
    // All extrema combinations: quantized cell derivatives stay positive.
    for(unsigned mask=0;mask<64;mask++) {
        p.coefficient_permille=mask&1?900:-900;p.x_middle=mask&2?900:-900;p.x_lower=mask&4?900:-900;
        p.y_upper=mask&8?900:-900;p.y_lower=mask&16?900:-900;p.y_center=mask&32?500:-500;
        iqs915x_map_generate(&p,map);
        for(unsigned j=0;j<5;j++)for(unsigned i=0;i<5;i++) {
            assert(map[j][i][0]==-map[j][4-i][0]);assert(map[j][i][1]==map[j][4-i][1]);
            assert(map[0][i][0]==map[4][i][0]);assert(map[0][i][1]==0&&map[4][i][1]==0);
            if(i<4)assert(8192+map[j][i+1][0]-map[j][i][0]>0);
            if(j<4)assert(8192+map[j+1][i][1]-map[j][i][1]>0);
        }
        for(unsigned res=5999;res<=6001;res++)for(unsigned y=0;y<=4000;y+=125) {
            unsigned previous=0;
            for(unsigned x=0;x<=res;x++) {uint16_t a=x,b=y;iqs915x_map_apply(&a,&b,res,4000,map);assert(a>=previous&&a<=res&&b<=4000);previous=a;}
        }
    }
    p=(struct iqs915x_processing_settings){.stop_enabled=1,.stop_fast_speed=5333,.stop_arm_ms=120,.stop_quiet_ms=30,.stop_radius=2,.stop_samples=3,.bottom_beta=256,.top_speed=1000};iqs915x_map_generate(&p,map);
    for(unsigned x=0;x<65535;x++){uint16_t a=x,b=65534-x;iqs915x_map_apply(&a,&b,65534,65534,map);assert(a==x&&b==65534-x);}
    xx=UINT16_MAX;yy=123;iqs915x_map_apply(&xx,&yy,6000,4000,map);assert(xx==UINT16_MAX&&yy==123);
    xx=3;yy=3;iqs915x_map_apply(&xx,&yy,3,3,map);assert(xx==3&&yy==3);
    assert(iqs915x_isqrt(0)==0&&iqs915x_isqrt(25)==5&&iqs915x_isqrt(24)==4);
    assert(iqs915x_isqrt(2ULL*65535*65535)==92680);
    p.bottom_speed=100;p.top_speed=1000;p.bottom_beta=64;
    assert(iqs915x_dynamic_beta(&p,0)==64&&iqs915x_dynamic_beta(&p,100)==64);
    assert(iqs915x_dynamic_beta(&p,550)==160&&iqs915x_dynamic_beta(&p,1000)==256);
    assert(iqs915x_dynamic_beta(&p,1000000)==256);
    struct iqs915x_iir_state s={0};uint16_t x=1000,y=2000;
    assert(!iqs915x_dynamic_iir(&s,&p,100,65534,65534,&x,&y)&&x==1000&&y==2000);
    p.bottom_speed=100000;p.top_speed=200000;
    x=1070;y=2000;assert(iqs915x_dynamic_iir(&s,&p,115,65534,65534,&x,&y));assert(x==1018&&y==2000);
    x=1070;y=2000;for(unsigned t=130;t<2000;t+=15){x=1070;y=2000;if(!iqs915x_dynamic_iir(&s,&p,t,65534,65534,&x,&y))break;}
    assert(x==1070&&y==2000);
    // Timestamps wrap correctly and a long gap resets without a stale tail.
    s=(struct iqs915x_iir_state){0};x=5;y=10;iqs915x_dynamic_iir(&s,&p,UINT32_MAX-10,65534,65534,&x,&y);
    x=105;y=10;iqs915x_dynamic_iir(&s,&p,4,65534,65534,&x,&y);assert(x==30);
    x=60000;y=60000;assert(!iqs915x_dynamic_iir(&s,&p,1000,65534,65534,&x,&y)&&x==60000);
    // Gain 256 is exact bypass; zero gain holds and never requests tail polling.
    p.bottom_beta=256;x=61000;y=61000;assert(!iqs915x_dynamic_iir(&s,&p,1015,65534,65534,&x,&y)&&x==61000);
    p.bottom_beta=0;x=61001;y=61000;assert(!iqs915x_dynamic_iir(&s,&p,1030,65534,65534,&x,&y)&&x==61000);
    // Equal speeds at different sample intervals select the same beta;
    // the gain then compensates for elapsed time rather than pixel/cycle.
    p=(struct iqs915x_processing_settings){.stop_enabled=1,.stop_fast_speed=5333,.stop_arm_ms=120,.stop_quiet_ms=30,.stop_radius=2,.stop_samples=3,.bottom_beta=64,.bottom_speed=100,.top_speed=1000};
    for(unsigned dt=10;dt<=20;dt+=10) {
        struct iqs915x_iir_state state={0};x=1000;y=1000;iqs915x_dynamic_iir(&state,&p,0,65534,65534,&x,&y);
        x=1000+dt/2;y=1000;iqs915x_dynamic_iir(&state,&p,dt,65534,65534,&x,&y);
        unsigned beta=iqs915x_dynamic_beta(&p,500);
        int64_t expected=1000LL*65536+iqs915x_div_round((int64_t)(dt/2)*65536*beta*dt,beta*dt+(256-beta)*15);
        assert(state.x_q16==expected);
    }
    // A fast contact stops without replaying its discarded lag, even after idle.
    p=(struct iqs915x_processing_settings){.stop_enabled=1,.stop_fast_speed=5333,.stop_arm_ms=120,.stop_quiet_ms=30,.stop_radius=2,.stop_samples=3,.bottom_beta=60,.bottom_speed=5333,.top_speed=233333};
    s=(struct iqs915x_iir_state){0};x=1000;y=1000;iqs915x_dynamic_iir(&s,&p,0,6000,4000,&x,&y);
    x=1200;y=1000;iqs915x_dynamic_iir(&s,&p,15,6000,4000,&x,&y);
    for(unsigned t=30;t<=60;t+=15){x=1200;y=1000;iqs915x_dynamic_iir(&s,&p,t,6000,4000,&x,&y);}
    assert(s.stopped&&!s.pending);uint16_t frozen=x;
    x=1200;y=1000;assert(!iqs915x_dynamic_iir(&s,&p,1000,6000,4000,&x,&y)&&x==frozen);
    x=1203;y=1000;iqs915x_dynamic_iir(&s,&p,2000,6000,4000,&x,&y);
    assert(!s.stopped&&x>=frozen&&x<=frozen+3);
    p.top_speed=p.bottom_speed;assert(!iqs915x_processing_valid(&p));
    puts("Processing math: periodic correction, beta thresholds, convergence, timestamps and integer limits PASS");
}
