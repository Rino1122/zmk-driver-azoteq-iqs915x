/* SPDX-License-Identifier: MIT */
static uint32_t cal_get32(const uint8_t *p) { return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24; }
static uint16_t cal_get16(const uint8_t *p) { return p[0]|(uint16_t)p[1]<<8; }
static void cal_put32(uint8_t *p,uint32_t n) { for(unsigned i=0;i<4;i++)p[i]=n>>(8*i); }
static void cal_put16(uint8_t *p,uint16_t n) { p[0]=n;p[1]=n>>8; }
static uint32_t cal_crc(const uint8_t *p,unsigned n) {
    uint32_t c=~0U;
    while(n--){c^=*p++;for(unsigned i=0;i<8;i++)c=(c>>1)^((0U-(c&1U))&0xedb88320U);}
    return ~c;
}
static void iqs915x_calibration_expire(struct k_work *work) {
    struct iqs915x_data *d=CONTAINER_OF(k_work_delayable_from_work(work),struct iqs915x_data,calibration_timeout);
    atomic_clear(&d->calibration_capturing);
    k_sem_give(&d->rdy_sem);
}
/* Called only for fresh successful reads, before any coordinate transform. */
static void iqs915x_calibration_sample(const struct device *dev,struct iqs915x_stream_data *s) {
    struct iqs915x_data *d=dev->data;
    unsigned fingers=s->trackpad_flags & IQS915X_NUM_FINGERS_MASK;
    k_mutex_lock(&d->calibration_lock,K_FOREVER);
    if(atomic_get(&d->calibration_capturing)) {
        uint32_t seq=++d->calibration_sequence;
        if(seq>128)d->calibration_dropped++;
        uint8_t *b=d->calibration_ring[(seq-1)%128];
        cal_put32(b,seq);cal_put32(b+4,d->comm_completed_ms);
        cal_put16(b+8,s->abs_x);cal_put16(b+10,s->abs_y);
        b[12]=fingers;b[13]=(s->trackpad_flags>>8)&15;
    }
    if(!fingers && atomic_get(&d->calibration_dirty)) {
        d->calibration_active=d->calibration_pending;
        d->calibration_enabled=d->calibration_pending_enabled;
        d->calibration_crc=d->calibration_pending_crc;
        atomic_clear(&d->calibration_dirty);
        d->last_abs_x=d->last_abs_y=0;
        atomic_set(&d->contact_restart_requested,1);
    }
    if(d->calibration_enabled && !atomic_get(&d->calibration_capturing)) {
        uint16_t *xs[]={&s->abs_x,&s->finger2_x,&s->finger3_x,&s->finger4_x};
        uint16_t *ys[]={&s->abs_y,&s->finger2_y,&s->finger3_y,&s->finger4_y};
        for(unsigned i=0;i<4;i++)iqs915x_cal_apply(&d->calibration_active,
            d->swipe_resolution_x,d->swipe_resolution_y,xs[i],ys[i]);
    }
    k_mutex_unlock(&d->calibration_lock);
}
static int cal_validate(struct iqs915x_data *d,const uint8_t *b,struct iqs915x_calibration_grid *g) {
    const struct iqs915x_config *c=d->dev->config;
    unsigned side=c->channel_profile==IQS915X_CHANNEL_PROFILE_BRETAGNE_LEFT?0:1;
    if(memcmp(b,"BRC1",4)||b[4]!=1||b[5]!=side||b[6]!=c->channel_profile||b[7]||
       cal_get16(b+8)!=d->swipe_resolution_x||cal_get16(b+10)!=d->swipe_resolution_y||
       !d->swipe_resolution_x||!d->swipe_resolution_y||cal_crc(b+16,884)!=cal_get32(b+12))return -EINVAL;
    for(unsigned i=0;i<IQS915X_CAL_NODES;i++) {
        g->dx[i]=(int16_t)cal_get16(b+16+i*2);
        g->dy[i]=(int16_t)cal_get16(b+16+442+i*2);
        int64_t x=(int64_t)g->dx[i]*97500/d->swipe_resolution_x;
        int64_t y=(int64_t)g->dy[i]*64000/d->swipe_resolution_y;
        if(x*x+y*y>3200LL*3200)return -ERANGE;
    }
    /* A bilinear cell's Jacobian is affine; check every corner in physical units.
     * Gershgorin bounds on J^T J conservatively enforce singular values .5..2. */
    for(unsigned row=0;row<12;row++)for(unsigned col=0;col<16;col++) {
        unsigned i=row*17+col;
        for(unsigned corner=0;corner<4;corner++) {
            unsigned h=i+((corner&2)?17:0),v=i+((corner&1)?1:0);
            int64_t a=65536+(int64_t)(g->dx[h+1]-g->dx[h])*16*65536/d->swipe_resolution_x;
            int64_t b=(int64_t)(g->dx[v+17]-g->dx[v])*12*97500*65536/d->swipe_resolution_x/64000;
            int64_t c1=(int64_t)(g->dy[h+1]-g->dy[h])*16*64000*65536/d->swipe_resolution_y/97500;
            int64_t e=65536+(int64_t)(g->dy[v+17]-g->dy[v])*12*65536/d->swipe_resolution_y;
            int64_t aa=(a*a+c1*c1)>>16,bb=(b*b+e*e)>>16,ab=llabs((a*b+c1*e)>>16);
            if(a*e-b*c1<=0||MIN(aa,bb)-ab<16384||MAX(aa,bb)+ab>262144)return -ERANGE;
        }
    }
    return 0;
}
int iqs915x_calibration_command(const struct device *dev,const uint8_t *q,
    unsigned n,uint8_t *out,unsigned *length) {
    if(!dev||!device_is_ready(dev))return -ENODEV;
    if(!q||n<9||n>96||!out||!length)return -EINVAL;
    struct iqs915x_data *d=dev->data;
    uint32_t token=cal_get32(q+1),off=cal_get32(q+5);
    int rc=0;*length=0;
    k_mutex_lock(&d->calibration_lock,K_FOREVER);
    switch(q[0]) {
    case 0:
        cal_put16(out,d->swipe_resolution_x);cal_put16(out+2,d->swipe_resolution_y);
        out[4]=((const struct iqs915x_config *)dev->config)->channel_profile;
        out[5]=d->calibration_enabled;out[6]=atomic_get(&d->calibration_dirty)!=0;
        out[7]=atomic_get(&d->calibration_capturing)!=0;
        cal_put32(out+8,d->calibration_crc);cal_put32(out+12,d->calibration_session);
        cal_put32(out+16,d->calibration_sequence);cal_put32(out+20,d->calibration_dropped);*length=24;break;
    case 1:
        if(!token||atomic_get(&d->lifecycle)!=IQS915X_READY||atomic_get(&d->ati_error)){rc=-EAGAIN;break;}
        if(atomic_get(&d->calibration_capturing)&&d->calibration_session!=token){rc=-EBUSY;break;}
        if(!atomic_get(&d->calibration_capturing)) {
            d->calibration_session=token;d->calibration_sequence=d->calibration_dropped=0;
        }
        atomic_set(&d->calibration_capturing,1);
        k_work_reschedule(&d->calibration_timeout,K_MSEC(3000));k_sem_give(&d->rdy_sem);break;
    case 2:
        if(token!=d->calibration_session||!atomic_get(&d->calibration_capturing)){rc=-ESTALE;break;}
        k_work_reschedule(&d->calibration_timeout,K_MSEC(3000));break;
    case 3:
        if(token!=d->calibration_session){rc=-ESTALE;break;}
        atomic_clear(&d->calibration_capturing);k_work_cancel_delayable(&d->calibration_timeout);break;
    case 4: {
        if(token!=d->calibration_session){rc=-ESTALE;break;}
        uint32_t first=d->calibration_sequence>128?d->calibration_sequence-127:1;
        if(off+1<first){rc=-EOVERFLOW;break;}
        cal_put32(out,d->calibration_dropped);unsigned count=0;
        while(off<d->calibration_sequence&&count<6) {
            off++;memcpy(out+5+count*14,d->calibration_ring[(off-1)%128],14);count++;
        }
        out[4]=count;*length=5+count*14;break;
    }
    case 5:
        if(!token){rc=-EINVAL;break;}
        d->calibration_uploaded=0;d->calibration_upload_token=token;d->calibration_upload_valid=true;break;
    case 6:
        if(!d->calibration_upload_valid||token!=d->calibration_upload_token||off!=d->calibration_uploaded||
           n==9||off+n-9>900){rc=-EINVAL;break;}
        memcpy(d->calibration_upload+off,q+9,n-9);d->calibration_uploaded+=n-9;break;
    case 7:
        if(!d->calibration_upload_valid||token!=d->calibration_upload_token||d->calibration_uploaded!=900){rc=-EINVAL;break;}
        rc=cal_validate(d,d->calibration_upload,&d->calibration_pending);
        if(!rc){d->calibration_pending_enabled=true;d->calibration_pending_crc=cal_get32(d->calibration_upload+12);atomic_set(&d->calibration_dirty,1);d->calibration_upload_valid=false;}
        break;
    case 8:d->calibration_pending_enabled=false;d->calibration_pending_crc=0;atomic_set(&d->calibration_dirty,1);break;
    default:rc=-EINVAL;
    }
    k_mutex_unlock(&d->calibration_lock);return rc;
}
