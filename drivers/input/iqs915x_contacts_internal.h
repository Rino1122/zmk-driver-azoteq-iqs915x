/* SPDX-License-Identifier: MIT */
#pragma once
#include <string.h>
#include <iqs915x_contacts.h>
/* Pure tracking core, independent of Zephyr, host protocol and mouse gestures. */
struct iqs915x_contacts_state {
  uint32_t generation, sequence;
  uint8_t active, next_id[4];
  bool blocked, probe_pending;
  struct iqs915x_contact last[4];
};
static inline void iqs915x_contacts_init(struct iqs915x_contacts_state *s) {
  memset(s, 0, sizeof(*s)); s->generation=1;
  for (unsigned i=0;i<4;i++) s->next_id[i]=i;
}
static inline void iqs915x_contacts_header(struct iqs915x_contacts_state *s,
    struct iqs915x_contact_frame *f, uint64_t time) {
  memset(f,0,sizeof(*f));f->sample_time_us=time;f->generation=s->generation;f->sequence=++s->sequence;
}
static inline void iqs915x_contacts_end(struct iqs915x_contacts_state *s,
    struct iqs915x_contact_frame *f, uint64_t time, uint8_t flags) {
  iqs915x_contacts_header(s,f,time);f->flags=flags|IQS915X_FRAME_SESSION_END;
  for(unsigned i=0;i<4;i++) if(s->active&(1u<<i)) {
    f->contacts[f->count]=s->last[i];f->contacts[f->count++].tip=false;
  }
  s->active=0;s->blocked=true;s->probe_pending=true;if(!++s->generation)s->generation=1;
}
/* false means no frame while quarantined. count=0 is the sensor's all-up
 * indication; lingering XY values are ignored. Confidence never selects slots. */
static inline bool iqs915x_contacts_sample(struct iqs915x_contacts_state *s,
    uint8_t count, const uint16_t x[4], const uint16_t y[4], uint8_t confidence,
    uint64_t time, struct iqs915x_contact_frame *f) {
  s->probe_pending=false; /* One fresh snapshot suffices; held fingers still wait for lift. */
  uint8_t mask=0,n=0;
  for(unsigned i=0;i<4;i++) if(x[i]!=UINT16_MAX && y[i]!=UINT16_MAX) {mask|=1u<<i;n++;}
  if(!count) {mask=0;n=0;}
  if(s->blocked) {if(!count)s->blocked=false;return false;}
  if(count>4 || n!=count) {
    iqs915x_contacts_end(s,f,time,IQS915X_FRAME_TRACKING_LOST);return true;
  }
  if(!mask && !s->active)return false; /* No idle periodic frames. */
  iqs915x_contacts_header(s,f,time);
  for(unsigned i=0;i<4;i++) {
    if(mask&(1u<<i)) {
      if(!(s->active&(1u<<i))) {s->last[i].id=s->next_id[i];s->next_id[i]+=4;}
      s->last[i].slot=i;s->last[i].x=x[i];s->last[i].y=y[i];
      s->last[i].tip=true;s->last[i].confidence=(confidence&(1u<<i))!=0;
      f->contacts[f->count++]=s->last[i];
    } else if(s->active&(1u<<i)) {
      f->contacts[f->count]=s->last[i];f->contacts[f->count++].tip=false;
    }
  }
  s->active=mask;return true;
}
