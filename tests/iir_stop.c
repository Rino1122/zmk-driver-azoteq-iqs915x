/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdio.h>
#include <iqs915x_processing_math.h>
static const struct iqs915x_processing_settings p={.bottom_beta=60,.bottom_speed=5333,.top_speed=233333};
static uint16_t feed(struct iqs915x_iir_state *s,uint32_t t,uint16_t x,uint16_t y) {
    iqs915x_dynamic_iir(s,&p,t,6000,4000,&x,&y);assert(x<=6000&&y<=4000);return x;
}
static struct iqs915x_iir_state stop(uint32_t start) {
    struct iqs915x_iir_state s={0};feed(&s,start,1000,1000);feed(&s,start+15,1200,1000);
    feed(&s,start+30,1200,1000);feed(&s,start+45,1200,1000);
    uint16_t before=(s.x_q16+32768)/65536;
    assert(feed(&s,start+60,1200,1000)==before&&s.stopped&&!s.pending);
    return s;
}
int main(void) {
    struct iqs915x_iir_state s=stop(0),other=stop(UINT32_MAX-40);
    int64_t frozen=s.x_q16,offset=s.offset_x;
    feed(&s,60,1400,1000);assert(s.stopped&&s.x_q16==frozen); /* duplicate */
    for(unsigned t=75;t<1000;t+=15){feed(&s,t,1200+(t%2),1000);assert(s.stopped&&s.x_q16==frozen);}
    feed(&s,2000,1202,1000);assert(s.stopped);
    feed(&s,3000,1203,1000);assert(!s.stopped&&s.offset_x==offset&&s.x_q16>frozen&&s.x_q16<frozen+3*65536);
    assert(other.stopped); /* independent state */
    for(unsigned t=3015;t<=4005;t+=15)feed(&s,t,1203,1000);
    assert(!s.stopped&&s.x_q16==(int64_t)1203*65536-offset);
    feed(&s,4020,1403,1000);feed(&s,4035,1403,1000);feed(&s,4050,1403,1000);feed(&s,4065,1403,1000);assert(s.stopped);
    /* Slow travel, including after a fast stroke, never stays in the radius. */
    s=(struct iqs915x_iir_state){0};feed(&s,0,1000,1000);feed(&s,15,1200,1000);
    for(unsigned t=30;t<=600;t+=15){feed(&s,t,1200+t/5,1000);assert(!s.stopped);}
    /* Saturation absorbs overflow so reversing at an edge has no dead zone. */
    s=(struct iqs915x_iir_state){.valid=true,.x_q16=6000LL*65536,.y_q16=1000LL*65536,.previous_x=5900,.previous_y=1000,.offset_x=-200LL*65536};
    feed(&s,15,5900,1000);assert(s.offset_x==-100LL*65536);
    feed(&s,30,5897,1000);assert(s.x_q16<6000LL*65536);
    /* Bypass and zero beta never arm stopping. */
    for(unsigned beta=0;beta<=256;beta+=256){
        struct iqs915x_processing_settings q=p;q.bottom_beta=beta;s=(struct iqs915x_iir_state){0};
        for(unsigned t=0;t<150;t+=15){uint16_t x=t?1200:1000,y=1000;iqs915x_dynamic_iir(&s,&q,t,6000,4000,&x,&y);assert(!s.stopped);}
    }
    puts("IIR stop: freeze, jitter, resume, repeated stops, slow travel, edges, wrap and bypass PASS");
}
