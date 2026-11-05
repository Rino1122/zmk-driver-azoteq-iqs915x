/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
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
struct iqs915x_circle_sample {uint32_t sequence,time;uint16_t x,y,flags;};
struct iqs915x_data {
 const struct device *dev;int processing_lock,circle_active,output_enabled,processing_reset,processing_tail;
 struct k_work_delayable circle_work;int64_t circle_deadline;
 int circle_saved_mode,requested_mode,pm_suspended,channel_running,channel_pending,ati_error,lifecycle;
 bool circle_restoring;uint32_t circle_session,circle_sequence,circle_dropped,comm_completed_ms;
 uint16_t swipe_resolution_x,swipe_resolution_y;
 struct iqs915x_circle_sample circle_ring[128];
};
struct iqs915x_stream_data {uint16_t abs_x,abs_y,finger2_x,finger2_y,finger3_x,finger3_y,finger4_x,finger4_y,trackpad_flags,info_flags;};
static int power_error;
static int iqs915x_set_mode(const struct device *dev,int mode){if(power_error)return power_error;((struct iqs915x_data*)dev->data)->requested_mode=mode;return 0;}
#include "iqs915x_circle_capture.inc"
static struct iqs915x_data d={.lifecycle=IQS915X_READY,.requested_mode=2,.swipe_resolution_x=6000,.swipe_resolution_y=4000};
static const struct device dev={&d};static uint8_t reply[96];static unsigned length;
static int command(unsigned op,unsigned session,unsigned after){uint8_t q[9]={op};circle_put32(q+1,session);circle_put32(q+5,after);return iqs915x_circle_command(&dev,q,9,reply,&length);}
int main(void){
 d.dev=&dev;
 assert(!command(2,42,0)&&reply[1]&&d.circle_saved_mode==2&&d.requested_mode==0);
 assert(command(2,43,0)==-EBUSY);assert(command(5,43,0)==-ESTALE);
 struct iqs915x_stream_data s={.abs_x=123,.abs_y=456,.finger2_x=UINT16_MAX,.finger2_y=UINT16_MAX,.finger3_x=UINT16_MAX,.finger3_y=UINT16_MAX,.finger4_x=UINT16_MAX,.finger4_y=UINT16_MAX,.trackpad_flags=1|BIT(8)};
 for(unsigned i=0;i<140;i++){d.comm_completed_ms=i*15;circle_sample(&d,&s);}
 assert(!command(5,42,0)&&length==90&&reply[18]==5&&circle_u32(reply+10)==12&&circle_u32(reply+20)==13);
 assert(!command(2,42,0)&&d.circle_sequence==140); // duplicate start never clears the ring
 circle_sample(&d,NULL);assert(d.circle_ring[140%128].flags&0x2000);
 now=3000;assert(!command(3,42,0));now=4000;circle_lease(&d.circle_work.work);assert(d.circle_active);
 now=7000;power_error=-EBUSY;circle_lease(&d.circle_work.work);assert(!d.circle_active&&d.circle_restoring&&!d.output_enabled);
 power_error=0;circle_lease(&d.circle_work.work);assert(!d.circle_restoring&&d.requested_mode==2);
 assert(command(3,42,0)==-ETIME);assert(!command(4,42,0));
 assert(!command(2,50,0)&&d.circle_sequence==0);assert(!command(4,50,0)&&!d.circle_active);circle_lease(&d.circle_work.work);assert(d.requested_mode==2);
 puts("Circle capture: independent raw ring, pagination/drop visibility, session guards, duplicate safety, lease renewal/expiry and power restore PASS");
}
