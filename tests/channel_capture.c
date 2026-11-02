#include <assert.h>
#include <stdio.h>
#include <iqs915x_channel_capture.h>
struct fake {uint8_t memory[65536];unsigned calls;uint32_t now;int fail,abort_at,reati_at,restore_fail;unsigned step;};
static int read_reg(void *ctx,uint16_t reg,uint8_t *out,unsigned n) {
 struct fake *f=ctx;assert(n<=16);f->calls++;f->now+=f->step?f->step:15;
 if(f->fail && f->calls==(unsigned)f->fail)return -EIO;
 if(f->reati_at && f->calls>=(unsigned)f->reati_at)f->memory[0x1020]|=0x10;
 memcpy(out,f->memory+reg,n);return 0;
}
static int write_reg(void *ctx,uint16_t reg,uint16_t v) {
 struct fake *f=ctx;assert(reg==0x11BE);f->calls++;f->now+=f->step?f->step:15;
 if(f->restore_fail && (v&4))return -EIO;
 f->memory[reg]=v;f->memory[reg+1]=v>>8;return 0;
}
static bool valid(void *ctx){struct fake *f=ctx;return !f->abort_at || f->calls<(unsigned)f->abort_at;}
static uint32_t now(void *ctx){return ((struct fake *)ctx)->now;}
static void setup(struct fake *f) {
 memset(f,0,sizeof(*f));f->memory[0x11E3]=18;f->memory[0x11E4]=12;
 f->memory[0x11BE]=0x86;f->memory[0x11BF]=0x0D;f->memory[0x1020]=8;
 for(unsigned i=0;i<216;i++){f->memory[0xB000+i*2]=(i+1)&255;f->memory[0xB001+i*2]=(i+1)>>8;}
 /* Bottom-right six by three disabled; one disabled zero and one enabled zero. */
 for(unsigned row=9;row<12;row++){f->memory[0x1246+row*4+1]=0xF0;f->memory[0x1246+row*4+2]=3;}
 f->memory[0xB000+2*215]=f->memory[0xB001+2*215]=0;
 f->memory[0xB000+2*20]=f->memory[0xB001+2*20]=0;
}
int main(void) {
 struct fake f;struct iqs915x_channel_snapshot s;
 struct iqs915x_channel_io io={&f,read_reg,write_reg,valid,now};
 setup(&f);assert(!iqs915x_channel_capture(&io,&s));
 assert(s.reference[0]==1 && s.reference[19]==20 && s.reference[20]==0 && s.reference[214]==215);
 assert(s.info_before==8 && s.info_after==8 && f.memory[0x11BE]==0x86 && !s.restore_failed);
 unsigned excluded=0,zero=0;for(unsigned i=0;i<216;i++) {
  if(iqs915x_channel_disabled(&s,i))excluded++;else if(!s.reference[i])zero++;
 }
 assert(excluded==18 && zero==1 && iqs915x_channel_disabled(&s,215));
 /* Every failure after freeze still restores config; no partial success. */
 for(unsigned call=5;call<=39;call++) {
  setup(&f);f.fail=call;assert(iqs915x_channel_capture(&io,&s)==-EIO);
  assert(f.memory[0x11BE]==0x86 && !s.restore_failed);
 }
 setup(&f);f.abort_at=12;assert(iqs915x_channel_capture(&io,&s)==-ECANCELED && f.memory[0x11BE]==0x86);
 setup(&f);f.reati_at=10;assert(iqs915x_channel_capture(&io,&s)==-EAGAIN && f.memory[0x11BE]==0x86);
 setup(&f);f.restore_fail=1;assert(iqs915x_channel_capture(&io,&s)==-EIO && s.restore_failed);
 setup(&f);f.step=200;assert(iqs915x_channel_capture(&io,&s)==-ETIMEDOUT && f.memory[0x11BE]==0x86);
 setup(&f);f.memory[0x11E3]=19;assert(iqs915x_channel_capture(&io,&s)==-ENOTSUP);
 setup(&f);f.memory[0x1020]=0x88;assert(iqs915x_channel_capture(&io,&s)==-EAGAIN);
 puts("Channel capture: packed reference, disabled masks, cancellation, ATI/reset fencing and restore failures PASS");
}
