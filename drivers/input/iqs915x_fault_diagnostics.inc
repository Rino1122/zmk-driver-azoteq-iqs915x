/* SPDX-License-Identifier: MIT */
int iqs915x_fault_get(const struct device *dev,struct iqs915x_fault_snapshot *out) {
  if(!dev || !device_is_ready(dev) || !out)return -ENODEV;
  if(k_is_in_isr())return -EWOULDBLOCK;
  struct iqs915x_data *d=dev->data;
  if(k_mutex_lock(&d->channel_lock,K_NO_WAIT))return -EBUSY;
  int ret=d->fault_snapshot_valid?0:-ENODATA;
  if(!ret)*out=d->fault_snapshot;
  k_mutex_unlock(&d->channel_lock);return ret;
}
static void iqs915x_fault_capture(struct iqs915x_data *d,int error,bool absent) {
  const struct iqs915x_config *cfg=d->dev->config;
  struct iqs915x_fault_snapshot f={.error=error,.stop_status=-EINPROGRESS,
    .captured_ms=k_uptime_get(),.wait_count=d->wait_count,.init_step=d->init_step,
    .restart_count=d->init_restart_count,.lifecycle=atomic_get(&d->lifecycle),
    .rdy_active=gpio_pin_get_dt(&cfg->rdy_gpio)>0,
    .read_status={-ENODATA,-ENODATA,-ENODATA}};
  if(!absent) {
    uint8_t bytes[2];
    f.read_status[0]=iqs915x_read_block(d->dev,IQS915X_INFO_FLAGS,bytes,2);
    if(!f.read_status[0]) {
      f.info=iqs_channel_u16(bytes);f.busy=f.info==0xEEEE;
      if(!f.busy) {
        f.valid_mask|=1;
        f.read_status[1]=iqs915x_read_block(d->dev,IQS915X_CONFIG_SETTINGS,bytes,2);
        if(!f.read_status[1]) {
          f.config=iqs_channel_u16(bytes);
          if(f.config==0xEEEE)f.busy=true;else f.valid_mask|=2;
        }
        if(!f.busy) {
          f.read_status[2]=iqs915x_read_block(d->dev,IQS915X_SYSTEM_CONTROL,bytes,2);
          if(!f.read_status[2]) {
            f.control=iqs_channel_u16(bytes);
            if(f.control==0xEEEE)f.busy=true;else f.valid_mask|=4;
          }
        }
      }
    }
  }
  k_mutex_lock(&d->channel_lock,K_FOREVER);
  d->fault_snapshot=f;d->fault_snapshot_valid=true;
  k_mutex_unlock(&d->channel_lock);
  LOG_ERR("terminal evidence info_rc=%d info=0x%04x busy=%u cfg_rc=%d cfg=0x%04x ctrl_rc=%d ctrl=0x%04x valid=0x%02x",
          f.read_status[0],f.info,f.busy,f.read_status[1],f.config,
          f.read_status[2],f.control,f.valid_mask);
}
