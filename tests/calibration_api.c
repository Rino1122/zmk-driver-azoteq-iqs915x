/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <iqs915x_calibration.h>
#include <iqs915x_calibration_math.h>
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define K_FOREVER 0
#define K_NO_WAIT 0
#define K_MSEC(x) (x)
#define IQS915X_NUM_FINGERS_MASK 7
#define IQS915X_SHOW_RESET 128
#define IQS915X_CHANNEL_PROFILE_BRETAGNE_LEFT 2
#define IQS915X_READY 2
#define IQS915X_POWER_ACTIVE 0
#define atomic_get(p) (*(p))
#define atomic_set(p,v) (*(p)=(v))
#define atomic_clear(p) (*(p)=0)
#define CONTAINER_OF(p,t,m) ((t*)((char*)(p)-offsetof(t,m)))
struct k_work {int unused;};
struct k_work_delayable {struct k_work work;};
static struct k_work_delayable *k_work_delayable_from_work(struct k_work *w) {return (void*)w;}
static int k_mutex_lock(int *p,int t){(void)p;(void)t;return 0;}
#define k_mutex_unlock(p) ((void)(p))
#define k_sem_give(p) ((void)(p))
#define k_work_reschedule(p,t) ((void)(p),(void)(t))
#define k_work_cancel_delayable(p) ((void)(p))
struct device {void *data;const void *config;};
static bool device_is_ready(const struct device *dev){return dev!=NULL;}
struct iqs915x_config {unsigned channel_profile;};
struct iqs915x_stream_data {uint16_t abs_x,abs_y,finger2_x,finger2_y,finger3_x,finger3_y,finger4_x,finger4_y,trackpad_flags,info_flags;};
struct iqs915x_data {
 const struct device *dev;int actual_mode,mode_transitioning,settings_lock,calibration_lock,rdy_sem,lifecycle,ati_error,channel_lock,diagnostic_job,channel_pending,channel_running;
 struct k_work_delayable calibration_timeout;
 struct iqs915x_calibration_grid calibration_active,calibration_pending;
 bool calibration_enabled,calibration_pending_enabled;
 int calibration_dirty,calibration_capturing,contact_restart_requested;
 uint32_t calibration_crc,calibration_pending_crc;
 uint8_t calibration_upload[900];unsigned calibration_uploaded;
 uint32_t calibration_upload_token;bool calibration_upload_valid;
 uint32_t calibration_session,calibration_sequence,calibration_dropped;
 uint8_t calibration_ring[128][14];
 uint16_t swipe_resolution_x,swipe_resolution_y,last_abs_x,last_abs_y;
 uint32_t comm_completed_ms;
};
#include "iqs915x_calibration.inc"
static struct iqs915x_data d;
static const struct iqs915x_config cfg={.channel_profile=2};
static const struct device dev={.data=&d,.config=&cfg};
static uint8_t out[96];static unsigned length;
static int cmd(unsigned op,unsigned token,unsigned off,const uint8_t *data,unsigned size) {
 uint8_t q[96]={op};cal_put32(q+1,token);cal_put32(q+5,off);if(size)memcpy(q+9,data,size);
 return iqs915x_calibration_command(&dev,q,9+size,out,&length);
}
static int upload(uint8_t *map,unsigned token) {
 assert(!cmd(5,token,0,NULL,0));
 for(unsigned at=0;at<900;at+=80)assert(!cmd(6,token,at,map+at,MIN(80,900-at)));
 return cmd(7,token,0,NULL,0);
}
int main(void) {
 d.dev=&dev;d.lifecycle=2;d.swipe_resolution_x=1024;d.swipe_resolution_y=768;
 assert(!cmd(0,0,0,NULL,0)&&length==24);
 assert(cmd(1,0,0,NULL,0)==-EAGAIN);
 assert(!cmd(1,123,0,NULL,0));assert(cmd(2,124,0,NULL,0)==-ESTALE);
 struct iqs915x_stream_data s={.abs_x=500,.abs_y=300,.trackpad_flags=257};
 for(unsigned i=0;i<130;i++)iqs915x_calibration_sample(&dev,&s);
 assert(cmd(4,123,0,NULL,0)==-EOVERFLOW);
 assert(!cmd(4,123,2,NULL,0)&&out[4]==6&&cal_get32(out+5)==3);
 assert(!cmd(3,123,0,NULL,0)&&!d.calibration_capturing);
 uint8_t map[900]={ 'B','R','C','1',1,0,2,0 };
 cal_put16(map+8,1024);cal_put16(map+10,768);
 for(unsigned i=0;i<221;i++)if(i%17>0&&i%17<16)cal_put16(map+16+i*2,3);
 cal_put32(map+12,cal_crc(map+16,884));
 assert(!upload(map,42)&&d.calibration_dirty&&!d.calibration_enabled);
 // Invalid upload must not damage an already staged valid candidate.
 map[20]^=1;assert(upload(map,43)==-EINVAL);assert(d.calibration_dirty);
 s.trackpad_flags=0;iqs915x_calibration_sample(&dev,&s);
 assert(d.calibration_enabled&&!d.calibration_dirty);
 s.abs_x=500;s.abs_y=300;s.trackpad_flags=257;iqs915x_calibration_sample(&dev,&s);
 assert(s.abs_x==503&&s.abs_y==300);
 assert(!cmd(8,0,0,NULL,0)&&d.calibration_enabled);
 s.trackpad_flags=0;iqs915x_calibration_sample(&dev,&s);assert(!d.calibration_enabled);
 assert(!cmd(1,124,0,NULL,0));iqs915x_calibration_expire(&d.calibration_timeout.work);assert(!d.calibration_capturing);
 map[20]^=1;map[5]=1;assert(upload(map,44)==-EINVAL);map[5]=0;
 cal_put16(map+8,2048);assert(upload(map,45)==-EINVAL);cal_put16(map+8,1024);
 // Excessive displacement and cell fold are rejected despite valid CRC.
 cal_put16(map+16,30000);cal_put32(map+12,cal_crc(map+16,884));assert(upload(map,46)==-ERANGE);
 puts("Calibration API: capture lease, overwrite detection, atomic uploads, compatibility, all-up switch: PASS");
}
