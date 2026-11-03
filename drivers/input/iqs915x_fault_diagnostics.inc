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

int iqs915x_mismatch_get(const struct device *dev,struct iqs915x_mismatch_snapshot *out) {
  if(!dev || !device_is_ready(dev) || !out)return -ENODEV;
  if(k_is_in_isr())return -EWOULDBLOCK;
  struct iqs915x_data *d=dev->data;
  if(k_mutex_lock(&d->channel_lock,K_NO_WAIT))return -EBUSY;
  int ret=d->mismatch_snapshot_valid?0:-ENODATA;
  if(!ret)*out=d->mismatch_snapshot;
  k_mutex_unlock(&d->channel_lock);return ret;
}
static void iqs915x_mismatch_capture(struct iqs915x_data *d,uint16_t expected,uint16_t observed) {
  /* Only this sensor thread writes the flag. Do not add I2C to every retry. */
  if(d->mismatch_snapshot_valid)return;
  const struct iqs915x_config *cfg=d->dev->config;
  struct iqs915x_mismatch_snapshot m={.expected=expected,.observed=observed};
  struct iqs915x_fault_snapshot *f=&m.evidence;
  *f=(struct iqs915x_fault_snapshot){.error=-EIO,.stop_status=-ENODATA,
      .captured_ms=k_uptime_get(),.wait_count=d->wait_count,.init_step=d->init_step,
      .restart_count=d->init_restart_count,.lifecycle=atomic_get(&d->lifecycle),
      .rdy_active=gpio_pin_get_dt(&cfg->rdy_gpio)>0,
      .read_status={-ENODATA,-ENODATA,-ENODATA}};
  /* Control/Config/Other/ALP bytes from a single communication window. */
  m.raw_status=iqs915x_read_block(d->dev,IQS915X_SYSTEM_CONTROL,m.raw,sizeof(m.raw));
  f->read_status[1]=f->read_status[2]=m.raw_status;
  if(!m.raw_status) {
    f->control=iqs_channel_u16(m.raw);f->config=iqs_channel_u16(m.raw+2);
    if(f->control==0xEEEE)f->busy=true;else f->valid_mask|=4;
    if(f->config==0xEEEE)f->busy=true;else f->valid_mask|=2;
  }
  uint8_t bytes[2];
  f->read_status[0]=iqs915x_read_block(d->dev,IQS915X_INFO_FLAGS,bytes,2);
  if(!f->read_status[0]) {
    f->info=iqs_channel_u16(bytes);
    if(f->info==0xEEEE)f->busy=true;else f->valid_mask|=1;
  }
  k_mutex_lock(&d->channel_lock,K_FOREVER);
  d->mismatch_snapshot=m;d->mismatch_snapshot_valid=true;
  k_mutex_unlock(&d->channel_lock);
  LOG_ERR("first mismatch expected=0x%04x observed=0x%04x raw_rc=%d info_rc=%d info=0x%04x valid=0x%02x busy=%u",
          expected,observed,m.raw_status,f->read_status[0],f->info,f->valid_mask,f->busy);
  if(!m.raw_status) {
    LOG_ERR("first mismatch raw 11bc: %02x %02x %02x %02x %02x %02x %02x %02x",
      m.raw[0],m.raw[1],m.raw[2],m.raw[3],m.raw[4],m.raw[5],m.raw[6],m.raw[7]);
    LOG_ERR("first mismatch raw 11c4: %02x %02x %02x %02x %02x %02x %02x %02x",
      m.raw[8],m.raw[9],m.raw[10],m.raw[11],m.raw[12],m.raw[13],m.raw[14],m.raw[15]);
  }
}

int iqs915x_ati_timeout_get(const struct device *dev,struct iqs915x_fault_snapshot *out) {
  if(!dev || !device_is_ready(dev) || !out)return -ENODEV;
  if(k_is_in_isr())return -EWOULDBLOCK;
  struct iqs915x_data *d=dev->data;
  if(k_mutex_lock(&d->channel_lock,K_NO_WAIT))return -EBUSY;
  int ret=d->ati_timeout_snapshot_valid?0:-ENODATA;
  if(!ret)*out=d->ati_timeout_snapshot;
  k_mutex_unlock(&d->channel_lock);return ret;
}
static void iqs915x_ati_timeout_capture(struct iqs915x_data *d) {
  if(d->ati_timeout_snapshot_valid)return;
  const struct iqs915x_config *cfg=d->dev->config;
  struct iqs915x_fault_snapshot f={.error=-ETIMEDOUT,.stop_status=-ENODATA,
    .captured_ms=k_uptime_get(),.wait_count=d->wait_count,.init_step=d->init_step,
    .restart_count=d->init_restart_count,.lifecycle=atomic_get(&d->lifecycle),
    .rdy_active=gpio_pin_get_dt(&cfg->rdy_gpio)>0,
    .read_status={-ENODATA,-ENODATA,-ENODATA}};
  {
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
  d->ati_timeout_snapshot=f;d->ati_timeout_snapshot_valid=true;
  k_mutex_unlock(&d->channel_lock);
  LOG_ERR("ATI timeout evidence info_rc=%d info=0x%04x busy=%u cfg_rc=%d cfg=0x%04x ctrl_rc=%d ctrl=0x%04x valid=0x%02x",
          f.read_status[0],f.info,f.busy,f.read_status[1],f.config,
          f.read_status[2],f.control,f.valid_mask);
}

/* Retired forensic trace opcode: preserve wire/API compatibility. */
int iqs915x_ati_trace_get(const struct device *dev,struct iqs915x_ati_trace *out) {
  if(!dev || !out)return -ENODEV;
  return -ENOTSUP;
}
static void iqs915x_trace_begin(struct iqs915x_data *d,uint16_t expected,int status) {}
static void iqs915x_trace_config(struct iqs915x_data *d,uint16_t value,int status) {}
static void iqs915x_trace_request(struct iqs915x_data *d,int status) {}
static void iqs915x_trace_sample(struct iqs915x_data *d,uint16_t info,int status) {}
static void iqs915x_trace_wait_timeout(struct iqs915x_data *d) {}
static void iqs915x_trace_finish(struct iqs915x_data *d,unsigned outcome) {}
