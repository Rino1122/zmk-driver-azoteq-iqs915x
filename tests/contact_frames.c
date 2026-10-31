/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdio.h>
#include "iqs915x_contacts_internal.h"
int main(void) {
 struct iqs915x_contacts_state s;struct iqs915x_contact_frame f;
 uint16_t x[4]={10,UINT16_MAX,30,UINT16_MAX},y[4]={20,UINT16_MAX,40,UINT16_MAX};
 iqs915x_contacts_init(&s);
 assert(iqs915x_contacts_sample(&s,2,x,y,1,100,&f));
 assert(f.count==2 && f.contacts[0].slot==0 && f.contacts[1].slot==2);
 assert(f.contacts[0].confidence && !f.contacts[1].confidence && f.contacts[1].tip);
 uint8_t id=f.contacts[0].id;uint32_t gen=f.generation,seq=f.sequence;
 x[0]=11;assert(iqs915x_contacts_sample(&s,2,x,y,0,200,&f));
 assert(f.contacts[0].id==id && !f.contacts[0].confidence && f.contacts[0].tip);
 assert(f.sample_time_us==200 && f.generation==gen && f.sequence==seq+1);
 x[0]=y[0]=UINT16_MAX;
 assert(iqs915x_contacts_sample(&s,1,x,y,4,300,&f));
 assert(f.count==2 && !f.contacts[0].tip && f.contacts[0].x==11 && f.contacts[0].id==id);
 x[0]=12;y[0]=22;assert(iqs915x_contacts_sample(&s,2,x,y,5,400,&f));
 assert(f.contacts[0].id!=id && f.contacts[0].id!=f.contacts[1].id);
 assert(iqs915x_contacts_sample(&s,0,x,y,0,500,&f)); /* Ignore stale XY on all-up. */
 assert(f.count==2 && !f.contacts[0].tip && !f.contacts[1].tip);
 assert(!iqs915x_contacts_sample(&s,0,x,y,0,600,&f));
 assert(iqs915x_contacts_sample(&s,2,x,y,5,700,&f));
 assert(iqs915x_contacts_sample(&s,5,x,y,15,800,&f));
 assert(f.flags==(IQS915X_FRAME_SESSION_END|IQS915X_FRAME_TRACKING_LOST));
 assert(f.count==2 && !f.contacts[0].tip && f.generation==gen);
 assert(!iqs915x_contacts_sample(&s,1,x,y,15,900,&f));
 assert(!iqs915x_contacts_sample(&s,0,x,y,0,1000,&f));
 assert(iqs915x_contacts_sample(&s,2,x,y,5,1100,&f) && f.generation!=gen);
 x[2]=UINT16_MAX;assert(iqs915x_contacts_sample(&s,2,x,y,5,1200,&f));
 assert(f.flags&IQS915X_FRAME_TRACKING_LOST); /* Missing coordinate is not a phantom lift. */
 assert(!iqs915x_contacts_sample(&s,2,x,y,5,1300,&f));
 iqs915x_contacts_end(&s,&f,1400,0);assert(f.flags==IQS915X_FRAME_SESSION_END && f.count==0);
 assert(!iqs915x_contacts_sample(&s,0,x,y,0,1500,&f));
 x[1]=20;y[1]=30;x[2]=y[2]=30;x[3]=40;y[3]=50;
 assert(iqs915x_contacts_sample(&s,4,x,y,15,1600,&f) && f.count==4);
 iqs915x_contacts_end(&s,&f,1700,0);assert(f.count==4);
 for(unsigned i=0;i<4;i++)assert(!f.contacts[i].tip);
 puts("Contact frames: holes, stable IDs/reuse, confidence, UP coordinates, overflow, invalid XY, reset quarantine: PASS");
}
