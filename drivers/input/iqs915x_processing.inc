/* SPDX-License-Identifier: MIT */
static uint16_t processing_u16(const uint8_t *b){return b[0]|(uint16_t)b[1]<<8;}
static uint32_t processing_u32(const uint8_t *b){return (uint32_t)b[0]|(uint32_t)b[1]<<8|(uint32_t)b[2]<<16|(uint32_t)b[3]<<24;}
static void processing_put16(uint8_t *b,uint16_t v){b[0]=v;b[1]=v>>8;}
static void processing_put32(uint8_t *b,uint32_t v){for(unsigned i=0;i<4;i++)b[i]=v>>(8*i);}
static void processing_encode(uint8_t *b,const struct iqs915x_processing_settings *p){
    processing_put16(b,p->coefficient_permille);processing_put16(b+2,p->bottom_beta);
    processing_put32(b+4,p->bottom_speed);processing_put32(b+8,p->top_speed);
}
static void processing_commit(struct iqs915x_data *d) {
    d->processing=d->processing_pending;d->processing_generation++;
    iqs915x_pitch_lut(d->processing.coefficient_permille,d->processing_lut);
    memset(d->processing_iir,0,sizeof(d->processing_iir));
    atomic_clear(&d->processing_tail);d->processing_dirty=false;
}
static void iqs915x_processing_sample(const struct device *dev,struct iqs915x_stream_data *s) {
    struct iqs915x_data *d=dev->data;
    uint16_t *xs[]={&s->abs_x,&s->finger2_x,&s->finger3_x,&s->finger4_x};
    uint16_t *ys[]={&s->abs_y,&s->finger2_y,&s->finger3_y,&s->finger4_y};
    unsigned fingers=s->trackpad_flags&IQS915X_NUM_FINGERS_MASK,valid=0;
    k_mutex_lock(&d->processing_lock,K_FOREVER);
    d->processing_touch=fingers!=0;
    if(!fingers && d->processing_dirty)processing_commit(d);
    if(atomic_cas(&d->processing_reset,1,0)||!fingers||(s->info_flags&IQS915X_SHOW_RESET))
        memset(d->processing_iir,0,sizeof(d->processing_iir));
    for(unsigned i=0;i<4;i++)if(*xs[i]!=UINT16_MAX&&*ys[i]!=UINT16_MAX)valid++;
    bool tail=false;
    for(unsigned i=0;i<4;i++) {
        *xs[i]=iqs915x_pitch_axis(*xs[i],d->swipe_resolution_x,6,d->processing_lut);
        *ys[i]=iqs915x_pitch_axis(*ys[i],d->swipe_resolution_y,4,d->processing_lut);
        if(iqs915x_output_is_enabled(d) && fingers && fingers<=4 && valid==fingers && !(s->info_flags&IQS915X_SHOW_RESET) &&
           *xs[i]!=UINT16_MAX && *ys[i]!=UINT16_MAX && (s->trackpad_flags&BIT(8+i))) {
            uint16_t previous_x=(d->processing_iir[i].x_q16+32768)/65536;
            uint16_t previous_y=(d->processing_iir[i].y_q16+32768)/65536;
            bool was_valid=d->processing_iir[i].valid;
            tail|=iqs915x_dynamic_iir(&d->processing_iir[i],&d->processing,d->comm_completed_ms,xs[i],ys[i]);
            if(was_valid&&(*xs[i]!=previous_x||*ys[i]!=previous_y))s->trackpad_flags|=IQS915X_TP_MOVEMENT;
        } else d->processing_iir[i].valid=false;
    }
    atomic_set(&d->processing_tail,tail);
    k_mutex_unlock(&d->processing_lock);
}
int iqs915x_processing_command(const struct device *dev,const uint8_t *q,unsigned n,uint8_t *out,unsigned *length) {
    if(!dev||!device_is_ready(dev))return -ENODEV;
    if(!q||!out||!length||n<9||q[0]>1||processing_u32(q+1)||processing_u32(q+5)||n!=(q[0]?21U:9U))return -EINVAL;
    struct iqs915x_data *d=dev->data;
    struct iqs915x_processing_settings p;
    if(q[0]) {
        p=(struct iqs915x_processing_settings){.coefficient_permille=(int16_t)processing_u16(q+9),
            .bottom_beta=processing_u16(q+11),.bottom_speed=processing_u32(q+13),.top_speed=processing_u32(q+17)};
        if(!iqs915x_processing_valid(&p))return -ERANGE;
    }
    k_mutex_lock(&d->processing_lock,K_FOREVER);
    if(q[0]) {
        d->processing_pending=p;d->processing_dirty=true;
        if(!d->processing_touch||atomic_get(&d->actual_mode)!=IQS915X_POWER_ACTIVE)processing_commit(d);
    }
    out[0]=1;out[1]=d->processing_dirty;out[2]=((const struct iqs915x_config *)dev->config)->channel_profile;
    out[3]=atomic_get(&d->lifecycle);processing_put16(out+4,d->swipe_resolution_x);processing_put16(out+6,d->swipe_resolution_y);
    processing_encode(out+8,&d->processing);processing_put32(out+20,d->processing_generation);*length=24;
    k_mutex_unlock(&d->processing_lock);return 0;
}
