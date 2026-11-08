/* SPDX-License-Identifier: MIT */
/* Independent raw branch; never replace the contact subscriber or mutate saved processing settings. */
static uint32_t trace_u32(const uint8_t *b){return (uint32_t)b[0]|(uint32_t)b[1]<<8|(uint32_t)b[2]<<16|(uint32_t)b[3]<<24;}
static void trace_put16(uint8_t *b,uint16_t v){b[0]=v;b[1]=v>>8;}
static void trace_put32(uint8_t *b,uint32_t v){for(unsigned i=0;i<4;i++)b[i]=v>>(i*8);}
static void trace_restore(struct iqs915x_data *d) {
    /* Close the live gate before removing capture's output suppression. */
    atomic_clear(&d->output_enabled);atomic_clear(&d->trace_active);
    atomic_set(&d->processing_reset,1);atomic_clear(&d->processing_tail);
    d->trace_restoring=true;
}
static void trace_lease(struct k_work *work) {
    struct iqs915x_data *d=CONTAINER_OF(k_work_delayable_from_work(work),struct iqs915x_data,trace_work);
    k_mutex_lock(&d->processing_lock,K_FOREVER);
    int64_t remaining=d->trace_deadline-k_uptime_get();
    if(atomic_get(&d->trace_active)&&remaining>0){k_work_reschedule(&d->trace_work,K_MSEC(remaining));k_mutex_unlock(&d->processing_lock);return;}
    if(atomic_get(&d->trace_active))trace_restore(d);
    if(d->trace_restoring){
        int rc=iqs915x_restore_capture_mode(d->dev,d->trace_saved_mode);
        if(rc==-EBUSY)k_work_reschedule(&d->trace_work,K_MSEC(100));
        else d->trace_restoring=false;
    }
    k_mutex_unlock(&d->processing_lock);
}
/* A session reports at about 33Hz, plus all contact/quality boundaries.
 * Existing sensor reads are reused; normal operation takes the atomic fast path. */
static void trace_sample(struct iqs915x_data *d,const struct iqs915x_stream_data *s) {
    if(!atomic_get(&d->trace_active))return;
    k_mutex_lock(&d->processing_lock,K_FOREVER);
    if(atomic_get(&d->trace_active)) {
        struct iqs915x_trace_sample next={.time=d->comm_completed_ms,.flags=0x2000};
        for(unsigned i=0;i<4;i++){next.x[i]=next.y[i]=UINT16_MAX;next.id[i]=d->trace_ids[i];}
        if(s){
            next.flags=s->trackpad_flags&0xfff;
            if(s->info_flags&IQS915X_SHOW_RESET)next.flags|=0x8000;
            unsigned valid=0,count=s->trackpad_flags&15;
            uint16_t xs[]={s->abs_x,s->finger2_x,s->finger3_x,s->finger4_x},ys[]={s->abs_y,s->finger2_y,s->finger3_y,s->finger4_y};
            for(unsigned i=0;i<4;i++)if(count&&xs[i]!=UINT16_MAX&&ys[i]!=UINT16_MAX){valid++;next.mask|=BIT(i);next.x[i]=xs[i];next.y[i]=ys[i];}
            if((count&&valid!=count)||count>4)next.flags|=0x2000;
            next.confidence=(s->trackpad_flags>>8)&next.mask;
        }
        if(next.flags&0xa000)next.mask=next.confidence=0;
        for(unsigned i=0;i<4;i++) {
            if(!(next.mask&BIT(i))){d->trace_iir[i].valid=false;continue;}
            if(!(d->trace_mask&BIT(i))) {
                if(!++d->trace_ids[i])++d->trace_ids[i];
                d->trace_iir[i].valid=false;
            }
            next.id[i]=d->trace_ids[i];
            if(!(next.confidence&BIT(i))){d->trace_iir[i].valid=false;continue;}
            if(d->trace_filtered)iqs915x_dynamic_iir(&d->trace_iir[i],&d->trace_settings,next.time,&next.x[i],&next.y[i]);
        }
        bool boundary=next.mask!=d->trace_mask || (next.flags&0xa000);
        if(d->trace_sequence){const struct iqs915x_trace_sample *last=&d->trace_ring[(d->trace_sequence-1)%128];boundary|=last->confidence!=next.confidence || ((last->flags^next.flags)&0xa000);}
        d->trace_mask=next.mask;
        if(!d->trace_sequence||boundary||(uint32_t)(next.time-d->trace_last_ms)>=30) {
            next.sequence=++d->trace_sequence;d->trace_ring[(next.sequence-1)%128]=next;d->trace_last_ms=next.time;
            if(next.sequence>128)d->trace_dropped=next.sequence-128;
        }
    }
    k_mutex_unlock(&d->processing_lock);
}
/* START is 10 bytes (op,session:u32,after=0:u32,IIR:u8). Other ops are 9.
 * Reply v2: 20-byte header, byte19=IIR flag, then <=2 36-byte frames:
 * sequence:u32,time:u32,flags:u16,mask:u8,confidence:u8,
 * four {x:u16,y:u16,contact_id:u16}. Masks refer to physical sensor slots.
 * Low confidence remains visible; errors/reset close all tracks. */
static int iqs915x_trace_command(const struct device *dev,const uint8_t *q,unsigned n,uint8_t *out,unsigned *length) {
    if(!dev||!device_is_ready(dev)||!q||!out||!length||q[0]<2||q[0]>5||n!=(q[0]==2?10U:9U)||(q[0]==2&&q[9]>1))return -EINVAL;
    struct iqs915x_data *d=dev->data;uint32_t session=trace_u32(q+1),after=trace_u32(q+5);
    if(!session)return -EINVAL;
    k_mutex_lock(&d->processing_lock,K_FOREVER);int rc=0;
    if(atomic_get(&d->trace_active)&&k_uptime_get()>=d->trace_deadline){trace_restore(d);k_work_reschedule(&d->trace_work,K_NO_WAIT);}
    if(q[0]==2){
        if(after){rc=-EINVAL;goto done;}
        if(atomic_get(&d->trace_active)){if(session!=d->trace_session||d->trace_filtered!=(q[9]!=0)){rc=-EBUSY;goto done;}}
        else {
            if(atomic_get(&d->sensor_request)||!d->sensor_verified||d->trace_restoring||atomic_get(&d->pm_suspended)||atomic_get(&d->channel_running)||atomic_get(&d->channel_pending)||atomic_get(&d->ati_error)||atomic_get(&d->lifecycle)!=IQS915X_READY){rc=-EBUSY;goto done;}
            d->trace_saved_mode=atomic_get(&d->requested_mode);
            rc=iqs915x_set_mode(dev,IQS915X_POWER_ACTIVE);if(rc)goto done;
            d->trace_session=session;d->trace_sequence=d->trace_dropped=0;
            d->trace_mask=0;d->trace_last_ms=0;d->trace_filtered=q[9]!=0;d->trace_settings=d->processing;
            memset(d->trace_ids,0,sizeof(d->trace_ids));memset(d->trace_iir,0,sizeof(d->trace_iir));
            atomic_set(&d->trace_active,1);atomic_set(&d->processing_reset,1);atomic_clear(&d->processing_tail);
        }
    }else if(session!=d->trace_session){rc=-ESTALE;goto done;}
    else if(q[0]==3&&!atomic_get(&d->trace_active)){rc=-ETIME;goto done;}
    if(q[0]==2||q[0]==3){d->trace_deadline=k_uptime_get()+3500;k_work_reschedule(&d->trace_work,K_MSEC(3500));}
    if(q[0]==4&&atomic_get(&d->trace_active)){trace_restore(d);k_work_reschedule(&d->trace_work,K_NO_WAIT);}
    if(after>d->trace_sequence){rc=-ERANGE;goto done;}
    memset(out,0,20);out[0]=2;out[1]=atomic_get(&d->trace_active);trace_put32(out+2,session);
    trace_put32(out+6,d->trace_sequence);trace_put32(out+10,d->trace_dropped);
    trace_put16(out+14,d->swipe_resolution_x);trace_put16(out+16,d->swipe_resolution_y);
    out[19]=d->trace_filtered;
    if(q[0]==5){
        uint32_t first=d->trace_sequence>128?d->trace_sequence-127:1;
        if(after+1<first)after=first-1;
        while(after<d->trace_sequence&&out[18]<2){
            const struct iqs915x_trace_sample *p=&d->trace_ring[after%128];uint8_t *b=out+20+36*out[18]++;
            trace_put32(b,p->sequence);trace_put32(b+4,p->time);trace_put16(b+8,p->flags);b[10]=p->mask;b[11]=p->confidence;
            for(unsigned i=0;i<4;i++){trace_put16(b+12+6*i,p->x[i]);trace_put16(b+14+6*i,p->y[i]);trace_put16(b+16+6*i,p->id[i]);}
            after++;
        }
    }
    *length=20+36*out[18];
done:k_mutex_unlock(&d->processing_lock);return rc;
}
