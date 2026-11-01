#!/usr/bin/env python3
"""Exercise production subscription, gating and boundary callbacks, no hardware."""
from pathlib import Path
import subprocess, tempfile,sys
sys.dont_write_bytecode=True
from runtime_power import STUB
from init_event_mode import block
root=Path(__file__).resolve().parents[1]
source=(root/'drivers/input/iqs915x.c').read_text()
gestures=(root/'drivers/input/iqs915x_gestures.c').read_text()
functions='\n'.join(block(source,m) for m in (
 'static uint32_t iqs915x_request_generation(', 'static bool iqs915x_output_is_enabled(',
 'int iqs915x_subscribe_contacts(', 'int iqs915x_set_output_mode(', 'int iqs915x_get_output_mode(',
 'int iqs915x_get_contact_resolution(', 'static void iqs915x_end_contact_session(',
 'static bool iqs915x_contact_probe_required('))
functions+='\n'+block(gestures,'bool iqs915x_get_finger_coordinates(')
functions+='\n'+block(source,'static bool iqs915x_deliver_contacts(')
harness=r'''
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include "iqs915x_internal.h"
static int64_t k_uptime_get(void) {return 123;}
static bool k_is_in_isr(void) {return false;}
FUNCTIONS
static struct iqs915x_contact_frame received;
static int frames;
static void copy_frame(const struct device *dev,const struct iqs915x_contact_frame *f,void *user) {
 assert(user==&frames);received=*f;frames++;
}
int main(void) {
 struct iqs915x_data d={0};struct device dev={.data=&d};d.dev=&dev;
 iqs915x_contacts_init(&d.contacts);d.output_enabled=1;
 assert(iqs915x_set_output_mode(&dev,IQS915X_OUTPUT_CONTACTS)==-ENOTCONN);
 assert(!iqs915x_subscribe_contacts(&dev,copy_frame,&frames));
 assert(iqs915x_subscribe_contacts(&dev,copy_frame,&frames)==-EALREADY);
 assert(!iqs915x_set_output_mode(&dev,IQS915X_OUTPUT_CONTACTS));
 assert(!d.output_enabled && d.request_generation==1);
 assert(iqs915x_subscribe_contacts(&dev,NULL,NULL)==-EBUSY);
 struct iqs915x_stream_data s={.trackpad_flags=1|IQS915X_FINGER1_CONFIDENCE,
 .abs_x=100,.abs_y=200,.finger2_x=UINT16_MAX,.finger2_y=UINT16_MAX,
 .finger3_x=UINT16_MAX,.finger3_y=UINT16_MAX,.finger4_x=UINT16_MAX,.finger4_y=UINT16_MAX};
 assert(iqs915x_deliver_contacts(&d,&s,0,1000) && !frames); /* stale/closed gate */
 iqs915x_end_contact_session(&d);assert(frames==1 && (received.flags&IQS915X_FRAME_SESSION_END));
 assert(!iqs915x_contact_probe_required(&d)); /* closed transition cannot read */
 d.applied_generation=d.request_generation;d.output_enabled=1;
 assert(iqs915x_contact_probe_required(&d));
 assert(iqs915x_deliver_contacts(&d,&s,1,2000) && frames==1); /* held during switch */
 assert(!iqs915x_contact_probe_required(&d)); /* held snapshot is not polled forever */
 s.trackpad_flags=0;assert(iqs915x_deliver_contacts(&d,&s,1,3000) && frames==1);
 s.trackpad_flags=1;assert(iqs915x_deliver_contacts(&d,&s,1,4000) && frames==2);
 assert(received.count==1 && received.contacts[0].tip && !received.contacts[0].confidence);
 assert(received.sample_time_us==4000);
 /* Simulate reactivation while no fingers are down: forced snapshot consumes
  * the all-up barrier, so the very next DOWN must be reported. */
 iqs915x_end_contact_session(&d);assert(iqs915x_contact_probe_required(&d));
 s.trackpad_flags=0;int before_idle=frames;
 assert(iqs915x_deliver_contacts(&d,&s,1,4100) && frames==before_idle);
 assert(!iqs915x_contact_probe_required(&d));
 s.trackpad_flags=1;assert(iqs915x_deliver_contacts(&d,&s,1,4200) && frames==before_idle+1);
 s.trackpad_flags=IQS915X_TOO_MANY_FINGERS; /* raw count=0 is not all-up here */
 assert(iqs915x_deliver_contacts(&d,&s,1,5000));
 assert(received.flags&IQS915X_FRAME_TRACKING_LOST);assert(!received.contacts[0].tip);
 s.trackpad_flags=1;int before=frames;iqs915x_deliver_contacts(&d,&s,1,6000);assert(frames==before);
 assert(!iqs915x_set_output_mode(&dev,IQS915X_OUTPUT_MOUSE));assert(!d.output_enabled);
 assert(iqs915x_subscribe_contacts(&dev,NULL,NULL)==-EBUSY);
 iqs915x_end_contact_session(&d);d.applied_generation=d.request_generation;d.output_enabled=1;
 assert(iqs915x_contact_probe_required(&d));
 assert(iqs915x_deliver_contacts(&d,&s,2,7000)); /* mouse also waits for all-up */
 assert(!iqs915x_contact_probe_required(&d)); /* Held finger is not polled repeatedly. */
 s.trackpad_flags=0;assert(iqs915x_deliver_contacts(&d,&s,2,8000));
 s.trackpad_flags=1;assert(!iqs915x_deliver_contacts(&d,&s,2,9000));
 /* A fresh all-up snapshot on the Mouse boundary permits the first new touch. */
 d.mouse_wait_clear=true;iqs915x_end_contact_session(&d);
 assert(iqs915x_contact_probe_required(&d));
 s.trackpad_flags=0;assert(iqs915x_deliver_contacts(&d,&s,2,9100));
 assert(!d.mouse_wait_clear && !iqs915x_contact_probe_required(&d));
 s.trackpad_flags=1;assert(!iqs915x_deliver_contacts(&d,&s,2,9200));
 iqs915x_end_contact_session(&d);assert(!iqs915x_contact_probe_required(&d)); /* Ordinary Mouse power boundary. */
 assert(!iqs915x_subscribe_contacts(&dev,NULL,NULL));before=frames;
 iqs915x_end_contact_session(&d);assert(frames==before);
 d.pm_suspended=1;assert(iqs915x_set_output_mode(&dev,IQS915X_OUTPUT_MOUSE)==-EBUSY);
 d.pm_suspended=0;d.lifecycle=IQS915X_ABSENT;
 assert(iqs915x_set_output_mode(&dev,IQS915X_OUTPUT_CONTACTS)==-ENODEV);
 puts("Contact API: subscription detach, generation gate, end markers, all-up barrier, overflow flag, mouse fallback: PASS");
}
'''.replace('FUNCTIONS',functions)
with tempfile.TemporaryDirectory(prefix='iqs-contact-api-') as tmp:
 p=Path(tmp);(p/'stub.h').write_text(STUB)
 for n in ('device.h','drivers/gpio.h','drivers/i2c.h','kernel.h','sys/atomic.h','input/input.h'):
  f=p/'zephyr'/n;f.parent.mkdir(parents=True,exist_ok=True);f.write_text('#include "stub.h"\n')
 (p/'test.c').write_text(harness)
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-parameter',
 '-fsanitize=address,undefined','-I'+tmp,'-I'+str(root/'include'),'-I'+str(root/'drivers/input'),str(p/'test.c'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
