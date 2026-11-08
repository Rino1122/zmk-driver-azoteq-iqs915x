/* SPDX-License-Identifier: MIT */
/* Requests only enqueue work. All I2C/reinitialization runs on the sensor thread. */
static uint8_t sensor_patch_byte(const struct device *dev,uint16_t addr,uint8_t original) {
 const struct iqs915x_data *d=dev->data;const uint8_t *p=d->sensor_initializing;
 if(addr==0x11F1)return p[0];
 if(addr==0x11EE)return p[1];
 if(addr>=0x11EA&&addr<=0x11ED) {
  unsigned i=addr-0x11EA;uint16_t v=iqs915x_sensor_cycle(iqs915x_sensor_u32(p+(i<2?4:8)),d->active_sampling_period_ms);
  return v>>((i%2)*8);
 }
 return original;
}
/* Each helper honors the existing RDY/Force Comms gate. No runtime polling. */
static int sensor_check_registers(const struct device *dev) {
 struct iqs915x_data *d=dev->data;uint8_t b[8],settings;uint16_t period;
 int rc=iqs915x_read_block(dev,0x11EA,b,8);if(rc)return rc;
 for(unsigned i=0;i<8;i++)if((i<5||i==7)&&b[i]!=sensor_patch_byte(dev,0x11EA+i,b[i]))return 1;
 rc=iqs915x_read_block(dev,IQS915X_TRACKPAD_SETTINGS,&settings,1);if(rc)return rc;
 if((settings&(BIT(3)|BIT(4)))!=(d->sensor_initializing[2]?BIT(3):0))return 1;
 rc=iqs915x_read_reg16(dev,IQS915X_ACTIVE_MODE_REPORT_RATE,&period);if(rc)return rc;
 return period!=d->active_sampling_period_ms;
}
static int iqs915x_sensor_command(const struct device *dev,const uint8_t *q,unsigned n,uint8_t *out,unsigned *length) {
 if(!q||!out||!length||(q[0]!=6&&q[0]!=7)||n!=(q[0]==7?21U:9U))return -EINVAL;
 for(unsigned i=1;i<9;i++)if(q[i])return -EINVAL;
 struct iqs915x_data *d=dev->data;int rc=0;
 k_mutex_lock(&d->processing_lock,K_FOREVER);
 if(q[0]==7) {
  if(atomic_get(&d->lifecycle)>=4){rc=-ENODEV;goto done;}
  if(!iqs915x_sensor_valid(q+9,d->active_sampling_period_ms)){rc=-ERANGE;goto done;}
  if(memcmp(d->sensor_desired,q+9,12)) {
   if(atomic_get(&d->lifecycle)!=IQS915X_READY||atomic_get(&d->pm_suspended)||atomic_get(&d->trace_active)||atomic_get(&d->channel_running)||atomic_get(&d->channel_pending)){rc=-EBUSY;goto done;}
   memcpy(d->sensor_desired,q+9,12);atomic_set(&d->sensor_request,1);k_sem_give(&d->rdy_sem);
  }
 }
 memset(out,0,34);out[0]=1;out[1]=atomic_get(&d->lifecycle)!=IQS915X_READY||!d->sensor_verified||atomic_get(&d->sensor_request)||memcmp(d->sensor_desired,d->sensor_applied,12)!=0;
 out[2]=atomic_get(&d->lifecycle);out[3]=d->sensor_verified;
 out[4]=d->active_sampling_period_ms;out[5]=d->active_sampling_period_ms>>8;
 memcpy(out+6,d->sensor_desired,12);memcpy(out+18,d->sensor_applied,12);
 uint16_t b=iqs915x_sensor_cycle(iqs915x_sensor_u32(d->sensor_applied+4),d->active_sampling_period_ms);
 uint16_t t=iqs915x_sensor_cycle(iqs915x_sensor_u32(d->sensor_applied+8),d->active_sampling_period_ms);
 out[30]=b;out[31]=b>>8;out[32]=t;out[33]=t>>8;*length=34;
 done:k_mutex_unlock(&d->processing_lock);return rc;
}
