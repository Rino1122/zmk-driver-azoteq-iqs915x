/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <iqs915x_processing_math.h>
#define K_FOREVER 0
#define BIT(n) (1U<<(n))
#define IQS915X_NUM_FINGERS_MASK 7U
#define IQS915X_TP_MOVEMENT BIT(7)
#define IQS915X_SHOW_RESET BIT(7)
#define IQS915X_POWER_ACTIVE 0
#define atomic_get(p) (*(p))
#define atomic_set(p,v) (*(p)=(v))
#define atomic_clear(p) (*(p)=0)
static bool atomic_cas(int *p,int old,int value){if(*p!=old)return false;*p=value;return true;}
#define k_mutex_lock(p,t) ((void)(p),(void)(t))
#define k_mutex_unlock(p) ((void)(p))
struct device {void *data;const void *config;};
static bool device_is_ready(const struct device *d){return d!=NULL;}
struct iqs915x_config {unsigned channel_profile;};
struct iqs915x_stream_data {uint16_t abs_x,abs_y,finger2_x,finger2_y,finger3_x,finger3_y,finger4_x,finger4_y,trackpad_flags,info_flags;};
struct iqs915x_data {
 struct iqs915x_processing_settings processing,processing_pending;
 struct iqs915x_iir_state processing_iir[4];int16_t processing_map[5][5][2];
 int processing_lock,actual_mode,lifecycle,processing_tail,processing_reset;
 bool processing_touch,processing_dirty;uint32_t processing_generation,comm_completed_ms;
 uint16_t swipe_resolution_x,swipe_resolution_y;
};
static bool output_enabled=true;
static bool iqs915x_output_is_enabled(const struct iqs915x_data *d){(void)d;return output_enabled;}
#define iqs915x_circle_command(...) (-ENOTSUP)
#include "iqs915x_processing.inc"
static struct iqs915x_data d;
static const struct iqs915x_config cfg={2};static const struct device dev={&d,&cfg};
static uint8_t out[96];static unsigned length;
static int request(unsigned op,const struct iqs915x_processing_settings *p){
 uint8_t q[31]={op};if(op)processing_encode(q+9,p);
 return iqs915x_processing_command(&dev,q,op?31:9,out,&length);
}
static struct iqs915x_stream_data sample(unsigned fingers,uint16_t x,uint16_t y){
 struct iqs915x_stream_data s={.abs_x=x,.abs_y=y,.finger2_x=UINT16_MAX,.finger2_y=UINT16_MAX,.finger3_x=UINT16_MAX,.finger3_y=UINT16_MAX,.finger4_x=UINT16_MAX,.finger4_y=UINT16_MAX,.trackpad_flags=fingers|(fingers?BIT(8):0)};
 d.comm_completed_ms+=15;iqs915x_processing_sample(&dev,&s);return s;
}
int main(void){
 d.processing=(struct iqs915x_processing_settings)IQS915X_PROCESSING_DEFAULTS;
 d.processing_pending=d.processing;d.swipe_resolution_x=6000;d.swipe_resolution_y=4000;
 iqs915x_map_generate(&d.processing,d.processing_map);
 assert(!request(0,NULL)&&length==34&&out[0]==2&&!out[1]);
 struct iqs915x_processing_settings p={.bottom_beta=256,.bottom_speed=100,.top_speed=1000};assert(!request(1,&p));
 struct iqs915x_stream_data s=sample(1,750,300);assert(s.abs_x==750);
 p.coefficient_permille=500;assert(!request(1,&p)&&out[1]);assert(d.processing.coefficient_permille==0);
 sample(0,750,300);assert(!d.processing_dirty&&d.processing.coefficient_permille==500);
 s=sample(1,750,300);assert(s.abs_x==800&&!d.processing_tail);
 sample(0,750,300);p.bottom_beta=60;p.bottom_speed=100000;p.top_speed=200000;assert(!request(1,&p));
 s=sample(1,250,300);assert(s.abs_x==200);s=sample(1,300,300);assert(d.processing_tail&&s.abs_x<250);
 unsigned reads=0;while(d.processing_tail&&reads++<100)s=sample(1,300,300);assert(reads<100&&!d.processing_tail);
 uint16_t expected_x=300,expected_y=300;iqs915x_map_apply(&expected_x,&expected_y,6000,4000,d.processing_map);assert(s.abs_x==expected_x);
 // Low confidence, a lost sample and power/output transitions cannot retain a tail.
 d.processing_reset=1;s=sample(1,1000,300);assert(s.abs_x==1000);
 output_enabled=false;s=sample(1,1300,300);assert(!d.processing_tail&&!d.processing_iir[0].valid);
 output_enabled=true;s=sample(1,1500,300);assert(s.abs_x==1500);
 s=sample(1,UINT16_MAX,300);assert(s.abs_x==UINT16_MAX&&!d.processing_tail&&!d.processing_iir[0].valid);
 p.top_speed=p.bottom_speed;assert(request(1,&p)==-ERANGE);assert(d.processing.top_speed==200000);
 uint8_t bad[9]={1};assert(iqs915x_processing_command(&dev,bad,9,out,&length)==-EINVAL);
 puts("Processing API: pending/all-up settings, per-slot filtering, bounded tail, output/reset quarantine and validation PASS");
}
