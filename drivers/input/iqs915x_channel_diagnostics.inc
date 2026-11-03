/* SPDX-License-Identifier: MIT */
int iqs915x_channels_start(const struct device *dev, uint32_t *id) {
  if(!dev || !device_is_ready(dev) || !id)return -EINVAL;
  if(k_is_in_isr())return -EWOULDBLOCK;
  struct iqs915x_data *d=dev->data;
  if(atomic_get(&d->lifecycle)!=IQS915X_READY)return -ENODEV;
  if(atomic_get(&d->requested_mode)!=IQS915X_POWER_ACTIVE ||
     atomic_get(&d->actual_mode)!=IQS915X_POWER_ACTIVE ||
     atomic_get(&d->mode_transitioning) || atomic_get(&d->pm_suspended))return -EACCES;
  if(k_mutex_lock(&d->channel_lock,K_NO_WAIT))return -EBUSY;
  int ret=0;
  if(atomic_get(&d->diagnostic_job) || d->channel_snapshot.status==-EINPROGRESS)ret=-EBUSY;
  else {
    uint32_t next=d->channel_snapshot.id+1;if(!next)next=1;
    d->channel_snapshot=(struct iqs915x_channel_snapshot){.id=next,.status=-EINPROGRESS};
    *id=next;if(atomic_get(&d->ati_error))atomic_set(&d->channel_auto_attempted,1);
    d->channel_requested_generation=iqs915x_request_generation(d);
    atomic_set(&d->channel_pending,1);k_sem_give(&d->rdy_sem);
  }
  k_mutex_unlock(&d->channel_lock);return ret;
}
int iqs915x_channels_page(const struct device *dev,uint32_t id,unsigned offset,
                          struct iqs915x_channel_page *out) {
  if(!dev || !device_is_ready(dev) || !out || offset>=IQS915X_CHANNEL_COUNT)return -EINVAL;
  struct iqs915x_data *d=dev->data;
  if(k_mutex_lock(&d->channel_lock,K_NO_WAIT))return -EBUSY;
  const struct iqs915x_channel_snapshot *s=&d->channel_snapshot;
  int ret=!s->id ? -ENODATA : id && s->id!=id ? -ESTALE : 0;
  if(!ret) {
    *out=(struct iqs915x_channel_page){.id=s->id,.status=s->status,.offset=offset};
    if(atomic_get(&d->diagnostic_job))out->status=-EINPROGRESS;
    else if(s->status!=-EINPROGRESS) {
      out->ati_status=s->ati_status;out->restore_status=s->restore_status;
      out->started_ms=s->started_ms;out->ended_ms=s->ended_ms;
      out->info_before=s->info_before;out->info_after=s->info_after;
      out->config_before=s->config_before;out->mapping_valid=s->mapping_valid;
      if(s->status==0) {
        out->count=MIN(IQS915X_CHANNEL_PAGE,IQS915X_CHANNEL_COUNT-offset);
        for(unsigned i=0;i<out->count;i++) {
          out->values[i]=s->compensation[offset+i];
          if(iqs915x_channel_disabled(s,offset+i))out->disabled_mask|=BIT(i);
        }
      }
    }
  }
  k_mutex_unlock(&d->channel_lock);return ret;
}
struct iqs_channel_context {struct iqs915x_data *data;uint32_t generation;};
static int channel_read(void *ctx,uint16_t reg,uint8_t *buf,unsigned n) {
  struct iqs_channel_context *c=ctx;return iqs915x_read_block(c->data->dev,reg,buf,n);
}
static int channel_write(void *ctx,uint16_t reg,uint16_t v) {
  struct iqs_channel_context *c=ctx;return iqs915x_write_reg16(c->data->dev,reg,v);
}
static uint32_t channel_now(void *ctx) {(void)ctx;return (uint32_t)k_uptime_get();}
static bool channel_valid(void *ctx) {
  struct iqs_channel_context *c=ctx;struct iqs915x_data *d=c->data;
  return c->generation==iqs915x_request_generation(d) &&
    atomic_get(&d->lifecycle)==IQS915X_READY && !atomic_get(&d->pm_suspended) &&
    (atomic_get(&d->diagnostic_job) || atomic_get(&d->requested_mode)==IQS915X_POWER_ACTIVE);
}
static void iqs915x_channels_capture_pending(struct iqs915x_data *d) {
  struct iqs_channel_context ctx={d,d->channel_requested_generation};
  atomic_set(&d->channel_running,1);
  iqs915x_reset_input_session(d);
  struct iqs915x_channel_snapshot *s=&d->channel_staging;
  memset(s,0,sizeof(*s));
  k_mutex_lock(&d->channel_lock,K_FOREVER);s->id=d->channel_snapshot.id;
  k_mutex_unlock(&d->channel_lock);
  struct iqs915x_channel_io io={&ctx,channel_read,channel_write,channel_valid,channel_now};
  iqs915x_channel_capture(&io,s);
  if(!s->status) {
    const struct iqs915x_config *cfg=d->dev->config;
    unsigned at=0x1218-IQS915X_INIT_DATA_BASE_ADDR;
    s->mapping_valid=cfg->init_data_len>=at+30 &&
      !memcmp(s->mapping,cfg->init_data+at,30);
    if(!s->mapping_valid)s->status=-ENOTSUP;
  }
  if(!s->status) {
    iqs915x_runtime_info(d,s->info_after);
    unsigned saturated=0,disabled=0;
    for(unsigned i=0;i<IQS915X_CHANNEL_COUNT;i++) {
      if(iqs915x_channel_disabled(s,i)){disabled++;continue;}
      if(s->compensation[i]==0 || s->compensation[i]==1023) {
        saturated++;
        LOG_WRN("channel diagnostic profile=%u capture=%u row=%u col=%u key_row=%u key_col=%u compensation=%u (ATI limit, check candidate)",
                ((const struct iqs915x_config *)d->dev->config)->channel_profile,s->id,
                i/18,i%18,i/18/3,i%18/3,s->compensation[i]);
      }
    }
    (void)saturated;(void)disabled;
    LOG_INF("channel diagnostic capture=%u ati_limits=%u disabled=%u info=0x%04x duration_ms=%u",
            s->id,saturated,disabled,s->info_after,s->ended_ms-s->started_ms);
  } else LOG_WRN("channel diagnostic capture=%u failed=%d restore_failed=%u",
                 s->id,s->status,s->restore_failed);
  k_mutex_lock(&d->channel_lock,K_FOREVER);d->channel_snapshot=*s;
  k_mutex_unlock(&d->channel_lock);
  atomic_clear(&d->channel_running);
  bool reset=(s->info_before!=0xEEEE && (s->info_before & IQS915X_SHOW_RESET)) ||
             (s->info_after!=0xEEEE && (s->info_after & IQS915X_SHOW_RESET));
  if(s->restore_failed || reset)iqs915x_runtime_reset(d,"diagnostic restore/reset boundary");
}

/* One bounded automatic attempt per ATI-error episode, not per Re-ATI event.
 * Failed/cancelled attempts remain cached; explicit start can retry. */
static bool iqs915x_channels_auto_request(struct iqs915x_data *d) {
  if(!atomic_get(&d->ati_error)){atomic_clear(&d->channel_auto_attempted);return false;}
  if(atomic_get(&d->channel_auto_attempted) || atomic_get(&d->channel_running) ||
     atomic_get(&d->channel_pending) || atomic_get(&d->lifecycle)!=IQS915X_READY ||
     atomic_get(&d->requested_mode)!=IQS915X_POWER_ACTIVE ||
     atomic_get(&d->actual_mode)!=IQS915X_POWER_ACTIVE ||
     atomic_get(&d->mode_transitioning) || atomic_get(&d->pm_suspended))return false;
  uint32_t id;
  int rc=iqs915x_channels_start(d->dev,&id);
  if(rc)return false;
  atomic_set(&d->channel_auto_attempted,1);
  LOG_INF("ATI compensation auto capture=%u",id);
  return true;
}

/* One bounded sensor-thread transaction. Never changes persisted side/layer intent. */
int iqs915x_diagnostic_start(const struct device *dev,bool run_ati,uint32_t *id) {
  if(!dev || !id || !device_is_ready(dev))return -EINVAL;
  struct iqs915x_data *d=dev->data;
  if(k_is_in_isr())return -EWOULDBLOCK;
  if(atomic_get(&d->lifecycle)!=IQS915X_READY)return -ENODEV;
  if(atomic_get(&d->pm_suspended))return -EACCES;
  if(k_mutex_lock(&d->channel_lock,K_NO_WAIT))return -EBUSY;
  int rc=0;
  if(atomic_get(&d->diagnostic_job) || d->channel_snapshot.status==-EINPROGRESS)rc=-EBUSY;
  else {
    uint32_t next=d->channel_snapshot.id+1;if(!next)next=1;
    d->channel_snapshot=(struct iqs915x_channel_snapshot){.id=next,.status=-EINPROGRESS};
    d->channel_requested_generation=iqs915x_request_generation(d);
    if(atomic_get(&d->ati_error))atomic_set(&d->channel_auto_attempted,1);
    atomic_set(&d->diagnostic_job,run_ati?2:1);atomic_set(&d->channel_pending,1);
    *id=next;k_sem_give(&d->rdy_sem);
  }
  k_mutex_unlock(&d->channel_lock);return rc;
}
static int diagnostic_mode_finish(struct iqs915x_data *d,int64_t deadline) {
  while(d->work_state!=WORK_READ_DATA) {
    if(k_uptime_get()>=deadline || atomic_get(&d->lifecycle)!=IQS915X_READY)return -ETIMEDOUT;
    iqs915x_mode_step(d);
  }
  return atomic_get(&d->lifecycle)==IQS915X_READY?0:-EIO;
}
static void iqs915x_diagnostic_execute(struct iqs915x_data *d) {
  int kind=atomic_get(&d->diagnostic_job),rc=0;
  uint32_t generation=d->channel_requested_generation;
  int64_t deadline=k_uptime_get()+8000;
  atomic_set(&d->channel_running,1);atomic_clear(&d->output_enabled);d->enabled=false;
  iqs915x_reset_input_session(d);
  if(atomic_get(&d->pm_suspended) || generation!=iqs915x_request_generation(d))rc=-ECANCELED;
  if(!rc) {
    iqs915x_begin_mode(d,IQS915X_MODE_ACTIVE,false);
    rc=diagnostic_mode_finish(d,deadline);
    if(!rc)d->ic_suspended=false;
  }
  int ati_status=-ENODATA,restore_status=0;
  uint16_t info=0;
  if(!rc && kind==2) {
    uint16_t cfg=iqs915x_config_settings_without_event_mode(d->confirmed_config_settings);
    rc=iqs915x_write_reg16(d->dev,IQS915X_CONFIG_SETTINGS,cfg);
    uint16_t observed=0;
    if(!rc)rc=iqs915x_read_reg16(d->dev,IQS915X_CONFIG_SETTINGS,&observed);
    if(!rc && observed!=cfg)rc=-EIO;
    /* Consume any previous Occurred before issuing this request. */
    if(!rc)rc=iqs915x_read_reg16(d->dev,IQS915X_INFO_FLAGS,&observed);
    if(!rc && (observed==0xEEEE || (observed&IQS915X_SHOW_RESET)))rc=-EIO;
    if(!rc)rc=iqs915x_write_reg16(d->dev,IQS915X_SYSTEM_CONTROL,IQS915X_MODE_ACTIVE|IQS915X_REATI_TP);
    int64_t ati_end=MIN(deadline,k_uptime_get()+5000);
    bool occurred=false;
    while(!rc && k_uptime_get()<ati_end) {
      if(atomic_get(&d->pm_suspended) || generation!=iqs915x_request_generation(d)){rc=-ECANCELED;break;}
      const struct iqs915x_config *cfg=d->dev->config;
      if(gpio_pin_get_dt(&cfg->rdy_gpio)>0) {
        rc=iqs915x_read_reg16(d->dev,IQS915X_INFO_FLAGS,&info);
        if(!rc && info!=0xEEEE) {
          if(info&IQS915X_SHOW_RESET){rc=-EIO;break;}
          iqs915x_runtime_info(d,info);
          if(info&IQS915X_REATI_OCCURRED){occurred=true;break;}
        }
      }
      k_sleep(K_MSEC(5));
    }
    if(!rc && !occurred)rc=-ETIMEDOUT;
    ati_status=rc;
  }
  if(!rc && (atomic_get(&d->pm_suspended) || generation!=iqs915x_request_generation(d)))rc=-ECANCELED;
  if(!rc && (kind==1 || (info&IQS915X_ATI_ERROR))) {
    iqs915x_channels_capture_pending(d);
    rc=d->channel_snapshot.status;
  } else {
    k_mutex_lock(&d->channel_lock,K_FOREVER);
    d->channel_snapshot.status=rc;d->channel_snapshot.info_after=info;
    d->channel_snapshot.ended_ms=k_uptime_get();
    k_mutex_unlock(&d->channel_lock);
  }
  /* Reconcile the newest intent, including Suspend or a changed layer. */
  if(atomic_get(&d->lifecycle)==IQS915X_READY) {
    d->applied_generation=iqs915x_request_generation(d)-1;
    iqs915x_apply_pending_power_request(d);
    int restore=diagnostic_mode_finish(d,k_uptime_get()+3000);
    restore_status=restore;
    if(restore)rc=restore;
  } else if(!rc)rc=-EIO;
  k_mutex_lock(&d->channel_lock,K_FOREVER);
  d->channel_snapshot.status=rc;
  d->channel_snapshot.ati_status=ati_status;d->channel_snapshot.restore_status=restore_status;
  atomic_clear(&d->diagnostic_job);atomic_clear(&d->channel_running);
  k_mutex_unlock(&d->channel_lock);
  d->ati_assist_state=0;d->ati_assist_due=0;
}
