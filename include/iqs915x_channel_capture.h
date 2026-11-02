/* SPDX-License-Identifier: MIT */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#define IQS915X_CHANNEL_COLUMNS 18U
#define IQS915X_CHANNEL_ROWS 12U
#define IQS915X_CHANNEL_COUNT 216U
#define IQS915X_CHANNEL_PAGE 8U
struct iqs915x_channel_snapshot {
    uint32_t id, started_ms, ended_ms;
    int status;
    uint16_t info_before, info_after, config_before;
    uint16_t reference[IQS915X_CHANNEL_COUNT];
    uint8_t disabled[48], mapping[30];
    bool mapping_valid, restore_failed;
};
struct iqs915x_channel_io {
    void *ctx;
    int (*read)(void *, uint16_t, uint8_t *, unsigned);
    int (*write)(void *, uint16_t, uint16_t);
    bool (*valid)(void *);
    uint32_t (*now)(void *);
};
static inline uint16_t iqs_channel_u16(const uint8_t *p) {return p[0]|((uint16_t)p[1]<<8);}
/* Every callback performs exactly one I2C transaction/STOP. Explicit capture
 * temporarily suppresses auto-ATI; no engineering/streaming selector is needed
 * for the direct 0xB000 reference window (datasheet A.27, packed Total Rxs).
 * A capture is unavailable, never partial-success, after reset/busy/cancellation.
 */
static inline int iqs915x_channel_capture(const struct iqs915x_channel_io *io,
                                         struct iqs915x_channel_snapshot *s) {
    uint8_t b[16];int ret=0,restore=0;bool written=false;
    s->restore_failed=false;s->mapping_valid=false;s->started_ms=io->now(io->ctx);
#define CAP_READ(reg,len) do { \
    if(!io->valid(io->ctx)){ret=-ECANCELED;goto done;} \
    if((uint32_t)(io->now(io->ctx)-s->started_ms)>=2000){ret=-ETIMEDOUT;goto done;} \
    ret=io->read(io->ctx,(reg),b,(len));if(ret<0)goto done; \
}while(0)
    CAP_READ(0x1020,2);s->info_before=iqs_channel_u16(b);
    if(s->info_before==0xEEEE || (s->info_before&0x87)){ret=-EAGAIN;goto done;}
    CAP_READ(0x11E3,2);if(b[0]!=18 || b[1]!=12){ret=-ENOTSUP;goto done;}
    CAP_READ(0x11BE,2);s->config_before=iqs_channel_u16(b);
    written=true; /* Restore even if an ambiguous write reports an error. */
    ret=io->write(io->ctx,0x11BE,s->config_before & ~(0x0004U|0x0008U));
    if(ret<0)goto done;
    CAP_READ(0x11BE,2);
    if(iqs_channel_u16(b)!=(s->config_before & ~(0x0004U|0x0008U))){ret=-EIO;goto done;}
    CAP_READ(0x1020,2);s->info_before=iqs_channel_u16(b);
    if(s->info_before==0xEEEE || (s->info_before&0x87)){ret=-EAGAIN;goto done;}
    for(unsigned i=0;i<30;i+=16) {
        unsigned n=30-i<16?30-i:16;CAP_READ(0x1218+i,n);memcpy(s->mapping+i,b,n);
    }
    for(unsigned i=0;i<48;i+=16){CAP_READ(0x1246+i,16);memcpy(s->disabled+i,b,16);}
    for(unsigned i=0;i<IQS915X_CHANNEL_COUNT;i+=8) {
        CAP_READ(0xB000+2*i,16);
        for(unsigned j=0;j<8;j++)s->reference[i+j]=iqs_channel_u16(b+2*j);
    }
    CAP_READ(0x1020,2);s->info_after=iqs_channel_u16(b);
    if(s->info_after==0xEEEE || (s->info_after&0x97) ||
       ((s->info_before^s->info_after)&8)){ret=-EAGAIN;goto done;}
done:
    if(written) {
        restore=io->write(io->ctx,0x11BE,s->config_before);
        if(!restore) {
            int rc=io->read(io->ctx,0x11BE,b,2);
            if(rc<0)restore=rc;else if(iqs_channel_u16(b)!=s->config_before)restore=-EIO;
        }
    }
    s->restore_failed=restore<0;
    if(restore<0)ret=restore;
    s->ended_ms=io->now(io->ctx);s->status=ret;
#undef CAP_READ
    return restore<0 ? restore : ret;
}
static inline bool iqs915x_channel_disabled(const struct iqs915x_channel_snapshot *s,unsigned i) {
    unsigned row=i/18,col=i%18;return !!(s->disabled[row*4+col/8]&(1u<<(col%8)));
}
