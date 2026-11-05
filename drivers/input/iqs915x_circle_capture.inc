/* SPDX-License-Identifier: MIT */
/* Independent raw branch; never replace the contact subscriber or mutate saved processing settings. */
static uint32_t circle_u32(const uint8_t *b){return (uint32_t)b[0]|(uint32_t)b[1]<<8|(uint32_t)b[2]<<16|(uint32_t)b[3]<<24;}
static void circle_put16(uint8_t *b,uint16_t v){b[0]=v;b[1]=v>>8;}
static void circle_put32(uint8_t *b,uint32_t v){for(unsigned i=0;i<4;i++)b[i]=v>>(i*8);}
static void circle_restore(struct iqs915x_data *d) {
    /* Close the live gate before removing capture's output suppression. */
    atomic_clear(&d->output_enabled);atomic_clear(&d->circle_active);
    atomic_set(&d->processing_reset,1);atomic_clear(&d->processing_tail);
    d->circle_restoring=true;
}
static void circle_lease(struct k_work *work) {
    struct iqs915x_data *d=CONTAINER_OF(k_work_delayable_from_work(work),struct iqs915x_data,circle_work);
    k_mutex_lock(&d->processing_lock,K_FOREVER);
    int64_t remaining=d->circle_deadline-k_uptime_get();
    if(atomic_get(&d->circle_active)&&remaining>0){k_work_reschedule(&d->circle_work,K_MSEC(remaining));k_mutex_unlock(&d->processing_lock);return;}
    if(atomic_get(&d->circle_active))circle_restore(d);
    if(d->circle_restoring){
        int rc=iqs915x_set_mode(d->dev,d->circle_saved_mode);
        if(rc==-EBUSY)k_work_reschedule(&d->circle_work,K_MSEC(100));
        else d->circle_restoring=false;
    }
    k_mutex_unlock(&d->processing_lock);
}
static void circle_sample(struct iqs915x_data *d,const struct iqs915x_stream_data *s) {
    k_mutex_lock(&d->processing_lock,K_FOREVER);
    if(atomic_get(&d->circle_active)) {
        uint32_t sequence=++d->circle_sequence;
        struct iqs915x_circle_sample *p=&d->circle_ring[(sequence-1)%128];
        *p=(struct iqs915x_circle_sample){.sequence=sequence,.time=d->comm_completed_ms,.x=UINT16_MAX,.y=UINT16_MAX,.flags=0x2000};
        if(s){
            p->flags=s->trackpad_flags&0xfff;
            if(s->info_flags&IQS915X_SHOW_RESET)p->flags|=0x8000;
            unsigned valid=0;uint16_t xs[]={s->abs_x,s->finger2_x,s->finger3_x,s->finger4_x};
            uint16_t ys[]={s->abs_y,s->finger2_y,s->finger3_y,s->finger4_y};
            for(unsigned i=0;i<4;i++)if((s->trackpad_flags&15)&&xs[i]!=UINT16_MAX&&ys[i]!=UINT16_MAX){valid++;p->x=xs[i];p->y=ys[i];if(!(s->trackpad_flags&BIT(8+i)))p->flags|=0x2000;}
            if(((s->trackpad_flags&15)&&valid!=(s->trackpad_flags&15))||(s->trackpad_flags&15)>1)p->flags|=0x2000;
        }
        if(sequence>128)d->circle_dropped=sequence-128;
    }
    k_mutex_unlock(&d->processing_lock);
}
/* q: operation 2=start,3=renew,4=stop,5=page; session:u32, after:u32.
 * Reply v1: active:u8,session:u32,last:u32,overwritten:u32,maxX/Y:u16,count:u8,reserved:u8,
 * then up to 5 records {sequence:u32,time_ms:u32,x/y:u16,flags:u16}. 20..90 bytes.
 * bit13=bad sample/read failure, bit15=reset. Other flags retain raw contact bits.
 * A session has a 3500ms MCU lease, renewed by the central's shorter 3000ms lease. */
static int iqs915x_circle_command(const struct device *dev,const uint8_t *q,unsigned n,uint8_t *out,unsigned *length) {
    if(!dev||!device_is_ready(dev)||!q||!out||!length||n!=9||q[0]<2||q[0]>5)return -EINVAL;
    struct iqs915x_data *d=dev->data;uint32_t session=circle_u32(q+1),after=circle_u32(q+5);
    if(!session)return -EINVAL;
    k_mutex_lock(&d->processing_lock,K_FOREVER);int rc=0;
    if(atomic_get(&d->circle_active)&&k_uptime_get()>=d->circle_deadline){circle_restore(d);k_work_reschedule(&d->circle_work,K_NO_WAIT);}
    if(q[0]==2){
        if(after){rc=-EINVAL;goto done;}
        if(atomic_get(&d->circle_active)){if(session!=d->circle_session){rc=-EBUSY;goto done;}}
        else {
            if(d->circle_restoring||atomic_get(&d->pm_suspended)||atomic_get(&d->channel_running)||atomic_get(&d->channel_pending)||atomic_get(&d->ati_error)||atomic_get(&d->lifecycle)!=IQS915X_READY){rc=-EBUSY;goto done;}
            d->circle_saved_mode=atomic_get(&d->requested_mode);
            rc=iqs915x_set_mode(dev,IQS915X_POWER_ACTIVE);if(rc)goto done;
            d->circle_session=session;d->circle_sequence=d->circle_dropped=0;
            atomic_set(&d->circle_active,1);atomic_set(&d->processing_reset,1);atomic_clear(&d->processing_tail);
        }
    }else if(session!=d->circle_session){rc=-ESTALE;goto done;}
    else if(q[0]==3&&!atomic_get(&d->circle_active)){rc=-ETIME;goto done;}
    if(q[0]==2||q[0]==3){d->circle_deadline=k_uptime_get()+3500;k_work_reschedule(&d->circle_work,K_MSEC(3500));}
    if(q[0]==4&&atomic_get(&d->circle_active)){circle_restore(d);k_work_reschedule(&d->circle_work,K_NO_WAIT);}
    if(after>d->circle_sequence){rc=-ERANGE;goto done;}
    memset(out,0,20);out[0]=1;out[1]=atomic_get(&d->circle_active);circle_put32(out+2,session);
    circle_put32(out+6,d->circle_sequence);circle_put32(out+10,d->circle_dropped);
    circle_put16(out+14,d->swipe_resolution_x);circle_put16(out+16,d->swipe_resolution_y);
    if(q[0]==5){
        uint32_t first=d->circle_sequence>128?d->circle_sequence-127:1;
        if(after+1<first)after=first-1;
        while(after<d->circle_sequence&&out[18]<5){const struct iqs915x_circle_sample *p=&d->circle_ring[after%128];uint8_t *b=out+20+14*out[18]++;
            circle_put32(b,p->sequence);circle_put32(b+4,p->time);circle_put16(b+8,p->x);circle_put16(b+10,p->y);circle_put16(b+12,p->flags);after++;}
    }
    *length=20+14*out[18];
done:k_mutex_unlock(&d->processing_lock);return rc;
}
