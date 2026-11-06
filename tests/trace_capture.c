/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <iqs915x_processing_math.h>
#include <string.h>
#define K_FOREVER 0
#define K_NO_WAIT 0
#define K_MSEC(x) (x)
#define BIT(n) (1U<<(n))
#define IQS915X_SHOW_RESET BIT(7)
#define IQS915X_READY 2
#define IQS915X_POWER_ACTIVE 0
#define atomic_get(p) (*(p))
#define atomic_set(p,v) (*(p)=(v))
#define atomic_clear(p) (*(p)=0)
#define k_mutex_lock(p,t) ((void)(p),(void)(t))
#define k_mutex_unlock(p) ((void)(p))
#define CONTAINER_OF(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
struct k_work {int unused;};struct k_work_delayable {struct k_work work;};
#define k_work_delayable_from_work(p) CONTAINER_OF(p,struct k_work_delayable,work)
static int64_t now;static int64_t k_uptime_get(void){return now;}
static void k_work_reschedule(struct k_work_delayable *w,int64_t delay){(void)w;(void)delay;}
struct device {void *data;};
static bool device_is_ready(const struct device *d){return d!=NULL;}
struct iqs915x_trace_sample {uint32_t sequence,time;uint16_t x[4],y[4],id[4],flags;uint8_t mask,confidence;};
struct iqs915x_data {
 const struct device *dev;int processing_lock,trace_active,output_enabled,processing_reset,processing_tail;
 struct k_work_delayable trace_work;int64_t trace_deadline;
 int trace_saved_mode,requested_mode,pm_suspended,channel_running,channel_pending,ati_error,lifecycle;
 bool trace_restoring;uint32_t trace_session,trace_sequence,trace_dropped,comm_completed_ms;
 uint16_t swipe_resolution_x,swipe_resolution_y;
 struct iqs915x_trace_sample trace_ring[128];
 bool trace_filtered;uint8_t trace_mask;uint16_t trace_ids[4];uint32_t trace_last_ms;
 struct iqs915x_processing_settings processing,trace_settings;struct iqs915x_iir_state trace_iir[4];
};
struct iqs915x_stream_data {uint16_t abs_x,abs_y,finger2_x,finger2_y,finger3_x,finger3_y,finger4_x,finger4_y,trackpad_flags,info_flags;};
static int power_error;
static int iqs915x_set_mode(const struct device *dev,int mode){if(power_error)return power_error;((struct iqs915x_data*)dev->data)->requested_mode=mode;return 0;}
static int iqs915x_restore_capture_mode(const struct device *dev,int mode){return iqs915x_set_mode(dev,mode);}
#include "iqs915x_trace.inc"
static struct iqs915x_data d={.lifecycle=IQS915X_READY,.requested_mode=2,.swipe_resolution_x=6000,.swipe_resolution_y=4000};
static const struct device dev={&d};static uint8_t reply[96];static unsigned length;
static int command(unsigned op,unsigned session,unsigned after){uint8_t q[10]={op};trace_put32(q+1,session);trace_put32(q+5,after);return iqs915x_trace_command(&dev,q,op==2?10:9,reply,&length);}
int main(void){
 d.dev=&dev;
 assert(!command(2,42,0)&&reply[1]&&d.trace_saved_mode==2&&d.requested_mode==0);
 assert(command(2,43,0)==-EBUSY);assert(command(5,43,0)==-ESTALE);
 struct iqs915x_stream_data s={.abs_x=123,.abs_y=456,.finger2_x=UINT16_MAX,.finger2_y=UINT16_MAX,.finger3_x=UINT16_MAX,.finger3_y=UINT16_MAX,.finger4_x=UINT16_MAX,.finger4_y=UINT16_MAX,.trackpad_flags=1|BIT(8)};
 for(unsigned i=0;i<140;i++){d.comm_completed_ms=i*30;trace_sample(&d,&s);}
 assert(!command(5,42,0)&&length==92&&reply[18]==2&&trace_u32(reply+10)==12&&trace_u32(reply+20)==13);
 assert(!command(2,42,0)&&d.trace_sequence==140); // duplicate start never clears the ring
 trace_sample(&d,NULL);assert(d.trace_ring[140%128].flags&0x2000);
 now=3000;assert(!command(3,42,0));now=4000;trace_lease(&d.trace_work.work);assert(d.trace_active);
 now=7000;power_error=-EBUSY;trace_lease(&d.trace_work.work);assert(!d.trace_active&&d.trace_restoring&&!d.output_enabled);
 power_error=0;trace_lease(&d.trace_work.work);assert(!d.trace_restoring&&d.requested_mode==2);
 assert(command(3,42,0)==-ETIME);assert(!command(4,42,0));
 assert(!command(2,50,0)&&d.trace_sequence==0);assert(!command(4,50,0)&&!d.trace_active);trace_lease(&d.trace_work.work);assert(d.requested_mode==2);
 // All four slots survive; slot reuse gets a fresh identity.
 assert(!command(2,51,0));
 s.finger2_x=700;s.finger2_y=800;s.finger3_x=900;s.finger3_y=1000;s.finger4_x=1100;s.finger4_y=1200;s.trackpad_flags=4|0xf00;
 d.comm_completed_ms+=30;trace_sample(&d,&s);assert(d.trace_ring[0].mask==15&&d.trace_ring[0].x[3]==1100);
 s.finger2_x=s.finger2_y=UINT16_MAX;s.trackpad_flags=3|0xd00;d.comm_completed_ms+=15;trace_sample(&d,&s);
 s.finger2_x=710;s.finger2_y=810;s.trackpad_flags=4|0xf00;d.comm_completed_ms+=15;trace_sample(&d,&s);assert(d.trace_ids[1]==2);
 // Dedicated monitor IIR must not change the normal settings or history.
 assert(!command(4,51,0));trace_lease(&d.trace_work.work);
 d.processing=(struct iqs915x_processing_settings)IQS915X_PROCESSING_DEFAULTS;
 d.processing.bottom_speed=100000;d.processing.top_speed=200000;
 uint8_t request[10]={2};trace_put32(request+1,60);request[9]=1;
 assert(!iqs915x_trace_command(&dev,request,10,reply,&length)&&reply[19]);
 s.trackpad_flags=1|BIT(8);s.finger2_x=s.finger2_y=s.finger3_x=s.finger3_y=s.finger4_x=s.finger4_y=UINT16_MAX;
 s.abs_x=100;d.comm_completed_ms+=30;trace_sample(&d,&s);
 s.abs_x=200;d.comm_completed_ms+=30;trace_sample(&d,&s);
 assert(d.trace_ring[1].x[0]>100&&d.trace_ring[1].x[0]<200&&d.processing.bottom_beta==60);
 puts("Trace capture: independent raw ring, pagination/drop visibility, session guards, duplicate safety, lease renewal/expiry and power restore PASS");
}
