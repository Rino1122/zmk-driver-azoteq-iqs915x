# ZMK driver for Azoteq IQS9150/IQS9151 trackpads

## Compatibility

This driver is designed for the IQS9150/IQS9151 series trackpad controllers. It has been developed based on the IQS9150/IQS9151 datasheet (Revision v1.1) and is intended for use with the following modules:

- PXM0091 (IQS9150 module)

## Supported features

- Trackpad movement from calibrated absolute finger coordinates, emitted as relative input.
- Optional firmware-side pointer acceleration for single-finger cursor motion.
- Driver-side single finger tap: Reported as a left click.
- Driver-side two finger tap: Reported as a right click.
- Tap-and-hold: A single tap is held briefly; a retouch starts left-button drag, otherwise a click is reported.
- Driver-side vertical scroll from absolute finger coordinates.
- Driver-side horizontal scroll from absolute finger coordinates.
- Scroll inertia.

## Usage

- Specify a node with the "azoteq,iqs915x" compatible inside an i2c node in your keyboard overlay.
- Reference it from an input listener:

```
/ {
    trackpad_input: trackpad_input {
        compatible = "zmk,input-listener";
        device = <&trackpad>;
    };
};

&i2c0 {
    status = "okay";
    trackpad: iqs915x@56 {
        status = "okay";
        compatible = "azoteq,iqs915x";
        reg = <0x56>;

        reset-gpios = <&gpio0 14 GPIO_ACTIVE_LOW>;
        rdy-gpios = <&gpio0 15 GPIO_ACTIVE_LOW>;

        /*
         * See: dts/bindings/input/azoteq,iqs915x-common.yaml for a full list.
         */
        one-finger-tap;
        tap-and-hold;
        tap-and-hold-release-timeout-ms = <250>;
        two-finger-tap;

        scroll;

        /* Scroll inertia settings */
        scroll-inertia;
        scroll-inertia-initial-velocity-percent = <100>; /* 100 = measured speed, 200 = 2x */
        trigger-ms = <35>;                    /* Wait before inertia starts */
        scroll-decay-factor-int = <85>;       /* Retention percent per tick (0-100) */
        scroll-report-interval-ms = <65>;     /* Time between inertia updates */
        scroll-threshold-start = <2>;         /* Arm inertia above this velocity */
        scroll-threshold-stop = <0>;          /* Stop inertia at/below this velocity */


        /* Optional firmware-side pointer acceleration; disabled if omitted */
        pointer-accel;
        pointer-sensitivity-percent = <90>;
        pointer-accel-threshold = <8>;
        pointer-accel-saturation = <96>;
        pointer-accel-max-percent = <180>;

        /* 3/4 finger swipe as one-shot gesture input events */
        three-finger-swipe;
        four-finger-swipe;
        /* 0 = auto threshold from X/Y resolution with ratio */
        swipe-step = <0>;
        swipe-threshold-numerator = <1>;
        swipe-threshold-denominator = <5>;
        swipe-direction-settle-frames = <2>;
        swipe-direction-lock-numerator = <3>;
        swipe-direction-lock-denominator = <2>;

    };
};
```

The driver uses IQS9150 absolute finger coordinates. Processing now runs in
this order on each MCU: validated sensor coordinates -> periodic key-pitch
correction -> MCU dynamic IIR -> contact/Mouse output. The IC's dynamic XY IIR
is enabled in init-data (0x11E2 bit3 set, bit4 clear), upstream of MCU processing.
Retained warm-start settings are checked against the compiled configuration;
mismatches force reinitialization. Jitter/area/MAV settings are preserved.

The periodic correction is a uniform 5x5 displacement tile, repeated over
6 X / 4 Y keys. Six permille controls generate its Q15 nodes: X upper,
middle, lower; Y upper, lower, centre shift. X is mirrored about the key
centre; Y can be asymmetric. Normal edge displacement is zero and tangential
edge displacement agrees across tiles. For normalized nodes (i/4,j/4), X
row strengths are [middle,upper,middle,lower,middle], with displacement
[0,-a/8000,0,a/8000,0]. Y rows are
[0,(-2*upper+centre)/16000,centre/8000,(2*lower+centre)/16000,0],
constant across X. Signed Q15 node quantization and final bilinear displacement
round ties away from zero. Settings rebuild the 100-byte tile once; sampling
uses integer multiply/divide with signed 64-bit intermediates (<2^62).
All-zero controls are exact identity. Allowed strengths +/-900 and centre
+/-500 guarantee positive axis derivatives (at least 0.325 before quantization);
Y has no X dependence, so the bilinear Jacobian cannot fold. These are bounded
manual controls, not an automatically estimated per-position calibration.

MCU dynamic IIR uses Bottom Beta (0..256), Bottom Speed and Top Speed, following
the beta-vs-speed shape in Azoteq's datasheet section 7.8.1:
https://www.azoteq.com/images/stories/pdf/IQS9150_IQS9151_datasheet.pdf
Beta is constant below Bottom Speed, linearly interpolated to 256 at Top Speed,
and 256 above it. Speed is the Euclidean corrected-input displacement divided
by measured milliseconds, in sensor pixels/second, not pixels/cycle. An integer
square root retains Q8 subpixel distance precision. The beta gain at the 15ms
reference interval is beta/256. For variable dt, gain is
`beta*dt / (beta*dt+(256-beta)*15)`. This is an MCU implementation inspired by
the documented dynamic filter, not a claim to reproduce undocumented IC internals.

All arithmetic is integer. Filter states retain Q16 fractions independently for
four physical slots. First touch, tracking loss, low confidence, read errors,
reset and power/output boundaries reset the state; gaps over 250ms reseed.
Beta 256 bypasses filtering (and speed calculation), Beta 0 holds slow positions
without tail polling. In event-only Mouse mode, fresh 15ms samples are requested
only while filter output needs convergence; polling stops at half-pixel residual
or lift. There is no steady idle timer, fabricated timestamp, per-sample Flash
access, or additional BLE stream. PTP keeps its normal fresh-sample policy.

`iqs915x_processing_command()` exposes GET/SET of a 22-byte LE settings record:
X upper/middle/lower, Y upper/lower/centre: six i16, then beta:u16,
bottom_speed:u32, top_speed:u32. Speeds obey 0 <= bottom < top <= 1,000,000.
Defaults: five strengths 500, centre 0, beta 60, speeds 5333/233333.
Driver wire version 2 returns 34 bytes. Old version-1 SET lengths are rejected.
Updates publish at all-up or with no live output and reset filter histories.
Firmware owns persistence and split synchronization; each MCU has its own
filter histories even when both use common settings. See the public header.
Old calibration map commands and their buffers are removed.

For coordinate calibration, enable `CONFIG_INPUT_AZOTEQ_IQS915X_COORD_LOG=y`.
The driver emits raw touched stream samples before calibration as INFO logs in
the form `coord,t=...,f=...,x1=...,y1=...`. Keep this disabled for normal use
because it logs every touched sample.

For scroll diagnostics, enable `CONFIG_LOG=y` and
`CONFIG_INPUT_AZOTEQ_IQS915X_LOG_LEVEL=4` on the trackpad peripheral. Each
nonzero axis delta passed to scroll normalization produces a DEBUG line:

```text
scroll_output,t=50000,source=manual,axis=wheel,delta=-60,wheel=-1,status=sent,rc=0,acc_before=0,acc_added=-10260,acc_after=-2260,denom=8000
```

- `t`: uptime in milliseconds at logging time, after the submission attempt.
- `source`: `manual` or `inertia`.
- `axis`: vertical `wheel` or horizontal `hwheel`.
- `delta`: calibrated coordinate delta after the cross-axis filter.
- `wheel`: integer value submitted or attempted; zero when still accumulating.
- `status`: `buffered` (below the output threshold), `sent` (input API accepted
  the event), `failed` (input API returned an error), or `disabled` (output gate
  prevented submission).
- `rc`: input API return code for `sent`/`failed`; zero for `buffered`, and
  synthetic `-EACCES` for `disabled`, where the API was not called.
- `acc_before`, `acc_added`, `acc_after`: signed accumulator before adding the
  delta, after adding it, and after submission. Successful submission retains
  only the remainder; failed or disabled submission keeps the accumulated value.
- `denom`: resolution times scroll divisor. Accumulator values use coordinate
  units multiplied by 171; a successful output consumes `wheel * denom`.

At release, `scroll_inertia_release,t=...,window_ms=...,vx=...,vy=...,threshold=...,initial_velocity_percent=...,start=...`
reports the estimated velocity in coordinate units per 10 ms, the elapsed
estimation window (at most 100 ms), and whether inertia was armed. `t` is the
first zero-finger report time, excluding the 20 ms confirmation delay.

Sum `wheel` only for `status=sent`, separately for each source and axis, to
compare driver output. Acceptance by the input API does not confirm delivery
over the split transport or to the host. Pair these logs with raw coordinate
logs when needed. DEBUG logging adds traffic and can affect report timing;
return to the normal log level after collecting diagnostics.

In Active Event Mode, the driver enables `TP_EVENT` as the only event source and
disables both IQS915x hardware gesture events and `TP_TOUCH_EVENT`.
`TP_TOUCH_EVENT` reports diamond-pattern channel state changes, not high-level
finger up/down transitions. Because `GLOBAL_TP_TOUCH` can miss transitions on
some IQS9150 devices, touch down/up boundaries are recognized from the
`NUM_FINGERS` field only. One-finger tap, two-finger tap, two-finger
scroll, and 3/4-finger swipes are recognized in the driver from finger count,
touch duration, and calibrated absolute finger coordinates. Tap classification
uses the IQS9150-style tap profile from init-data: `TAP_TOUCH_TIME` (`0x11FA`),
`TAP_WAIT_TIME` / air time (`0x11FC`), and `TAP_DISTANCE` (`0x11FE`).
Single tap is reported after air time elapses. If another touch-down occurs
within that air time, the pending single tap is canceled and the second contact
is classified as double click or tap-and-drag.

When `three-finger-swipe` or `four-finger-swipe` is enabled, the driver tracks
the centroid of active fingers and emits one-shot private gesture input events
based on the dominant swipe direction. The direction is locked only after the
stable-finger baseline has settled and the dominant axis is sufficiently larger
than the other axis. One gesture emits only one press/release pair until fingers
are released.

By default (`swipe-step = <0>`), swipe thresholds are computed from init-data
X/Y resolutions (registers `0x11E6`/`0x11E8`) using the smaller resolution and
`swipe-threshold-numerator` / `swipe-threshold-denominator`.
Default `1/5` means a gesture triggers at about 20% travel of the smaller axis,
using the same raw-coordinate threshold for horizontal and vertical swipes. If
you set `swipe-step` to a value > 0, that fixed threshold overrides the
ratio-based calculation. `swipe-direction-settle-frames` defaults to 2, and the
default direction-lock ratio is 3/2, so the dominant axis must be about 1.5x the
other axis before a 3/4-finger swipe direction is emitted.

The driver reports these gestures using `IQS915X_INPUT_EV_GESTURE` with
direction-specific codes from `<dt-bindings/input/iqs915x_gestures.h>`.
Firmware can map those events to keymap positions, behaviors, or other local
actions without consuming normal keyboard codes such as F13..F20.

Gesture code mapping:

- 3-finger left/up/down/right -> `IQS915X_GESTURE_3F_LEFT/UP/DOWN/RIGHT`
- 4-finger left/up/down/right -> `IQS915X_GESTURE_4F_LEFT/UP/DOWN/RIGHT`

See [docs/gesture_virtual_keys_ja.md](docs/gesture_virtual_keys_ja.md) for a
firmware-side setup guide.

Important: this driver emits gesture input events, not ZMK keymap position
events. To expose gesture controls in Studio, add 8 gesture slots on the
firmware side and map the gesture events to those keymap positions with an input
processor.

Two-finger scrolling uses centroid deltas from absolute coordinates. Movement
before the tap-distance threshold is retained and emitted once scrolling is
recognized, so the output reflects the full gesture displacement. Raw deltas
are normalized by the init-data X/Y resolutions before being reported as wheel
events. `scroll-divisor` is an extra coarse divisor applied after that
normalization. The fixed scale is 171 wheel units per full axis at divisor 1
(approximately one third of the original 512); this applies to both manual
scrolling and inertia. Fractional output is retained in the accumulator.

Finger-count changes during contact are debounced for 20 ms; initial contact
is accepted immediately. Touch boundaries, tap/drag recognition and scroll
release use the confirmed count. Unused (`0xffff`) and out-of-range XY pairs
are rejected before coordinate correction. Valid coordinates identify occupied
finger slots; confidence bits are checked separately for single-finger input
and are not used as occupancy flags. The driver reads four coordinate slots;
frames with more fingers or an ambiguous number of valid slots do not produce
movement from an assumed slot assignment.

Transient count changes retain the scroll session and fractions. During a
missing-coordinate interval, movement reports pause and the last valid centroid
is retained. If the same two slots return less than 20 ms after the first
invalid report, and neither finger has a discontinuous coordinate change, the
full centroid delta is accumulated once. Recovered movement is excluded from
inertia velocity and clears the previous inertia candidate. Longer gaps, slot
changes and discontinuities rebaseline instead. Slot reuse after a brief loss
cannot be distinguished from continuous physical contact; the time and
coordinate guards limit recovery to plausible continuity.

Once two or more fingers are confirmed during a contact sequence, cursor
movement and single-finger tap/hold/drag remain disabled until a zero-finger
count is confirmed. Pending single-finger gestures are canceled, and an active
tap-and-drag releases its left button when multiple fingers are confirmed.
Single-finger input resumes on the next contact after release, with a fresh
pointer baseline. An established scroll session retains its fractions if two
fingers return before release, but movement with one remaining finger is not
added to scrolling. Swipe centroids also use the valid slots and rebaseline on
slot changes. Event Mode count confirmation uses a timed thread wake and the
latest snapshot, without an extra I2C read. The original scroll cross-axis
filter remains applied to manual and inertial deltas.

When scroll inertia is enabled, the driver starts it only after a zero-finger
count is confirmed. `trigger-ms` is measured from the first zero-finger report;
inertia cannot start before the 20 ms confirmation completes. Raw contact
cancels pending or running inertia immediately. Tap duration and motion freshness
also use the first zero-finger report time, excluding the confirmation delay.
A release-time velocity estimate replaces the last-frame start check and EMA.
It uses signed centroid displacement divided by actual elapsed time over the
latest 100 ms, ending at the first zero-finger report. For shorter gestures,
the available interval since the valid two-finger baseline is used. Motion
before scroll recognition is included, but a tap still cannot start inertia.
Zero-motion reports and the stationary time between the last report and release
are included in the elapsed time; a pause of 100 ms removes all prior motion.
An interval crossing the window boundary contributes only its overlapping
fraction, assuming uniform movement within that sensor interval. Coordinate
recovery, slot changes and rebaselining clear the history and establish a new
baseline, excluding recovered displacement from velocity.

The average is converted to coordinate units per 10 ms. Its larger absolute
axis is compared with `scroll-threshold-start` before amplification. The signed
velocity is multiplied by `scroll-inertia-initial-velocity-percent` (100–1000,
default 100) and converted to coordinate units per millisecond. At 100%, inertia
starts at the measured average speed; at 200%, it starts at twice that speed.
Manual scroll displacement and inertia start detection are unaffected by this
multiplier. This normalization is independent of
sensor report rate; settings previously tuned at rates other than 10 ms may
need adjustment. `trigger-ms` controls start delay only, not motion freshness.
Motion and inertia use separate fractional accumulators so stopping or
cancelling inertia does not discard manual-scroll remainders.

Inertia integrates the decaying velocity over the actual time since the last
output. The decay percentage specifies retention over the boot-time DTS
`scroll-report-interval-ms`, which remains the reference even if the runtime
report interval changes. Reporting more often therefore produces smaller
increments with the same trajectory, subject to integer rounding and stop
conditions. Fixed-point velocity and displacement remainders preserve fractions.
No-output stopping uses elapsed time (three reference periods), and
`scroll-threshold-stop` is measured in coordinate units per 10 ms. The duration
limit clips integrated motion when a worker runs late.

This fixes the former mismatch between 10 ms velocity units and displacement
per inertia tick. Consequently 100% can produce more inertia travel than the
old implementation when its tick interval was longer than 10 ms.

Harbour integration uses runtime settings API v2:
`scroll_inertia.initial_velocity_percent`, with limits exposed as
`inertia_initial_velocity_percent` (100–1000). Reads return version 2; source
callers using version 1 are accepted with a 100% multiplier. The firmware RPC
must relay and persist this value and use a driver that supports API v2.

The velocity estimator has host-side regression tests that do not require a
Zephyr workspace:

```sh
cc -std=c11 -Wall -Wextra -Werror -Idrivers/input tests/scroll_motion.c -o /tmp/iqs915x-scroll-motion-test
/tmp/iqs915x-scroll-motion-test
cc -std=c11 -Wall -Wextra -Werror -Idrivers/input tests/scroll_inertia.c -o /tmp/iqs915x-scroll-inertia-test
/tmp/iqs915x-scroll-inertia-test
```

See [docs/scroll_parameters_ja.md](docs/scroll_parameters_ja.md) for a
practical Japanese guide to each scroll parameter and tuning workflow.

## Initialization data (IQS9150/IQS9151)

The IQS9150/IQS9151 does **not** have NVM, so all register settings must be
written via I2C after an IC reset. A host reboot or soft-off wake can retain the
powered IC's settings.
The driver compiles `drivers/input/IQS9150_init.h` as the sole source of the
1174 initialization bytes (`0x115C..0x15EB` + `0x2000..0x2005`). CMake runs
`scripts/generate_init_array.py` automatically to generate an array of references
to the export's C macros. The generated header includes the original export;
there is no manually copied array or DTS conversion. Header or generator changes
regenerate the array before compilation. Invalid byte counts, section addresses,
duplicate definitions and out-of-range values fail the build.

`drivers/input/iqs915x_sensor_config.h` owns initialization overrides: Active sampling
period 15 ms, LP2 sampling period 500 ms, Re-ATI retry 1 second and axis
orientation. Firmware DTS supplies only the I2C/GPIO wiring and input behavior
settings, such as tap, scroll, acceleration and enable state. The former
`profile`, `report-rate-ms`, `switch-xy`, `flip-x` and `flip-y` properties have
been removed; migrate firmware overlays by removing them and the profile node.

### Updating initialization settings

1. Replace `drivers/input/IQS9150_init.h` with the **Azoteq GUI** export.
2. Adjust driver overrides/calibration in `iqs915x_sensor_config.h` as needed.
3. Build the firmware with the updated driver revision. No conversion or copying
   into firmware DTS is required.

### Priority

The Azoteq header supplies the base bytes. Before each transfer, the driver
patches a temporary buffer with its sensor settings and mandatory protocol bits.
Read-back verification uses the same patched bytes. The export itself is never
modified. At driver startup, INFO_FLAGS selects the path from SHOW_RESET:

- SHOW_RESET set: transfer init-data, acknowledge reset, verify Streaming,
  request TP Re-ATI and verify the Event Mode policy.
- SHOW_RESET clear: check retained resolution and channel mask. If they match,
  retain init-data and calibration and verify Event Mode without requesting ATI.
  A mismatched retained profile starts the software-reset initialization path.

Startup accesses prefer RDY with bounded Force Comms fallback for quiet retained
states; see "Startup communication windows" below. Streaming Re-ATI waits and
init-data transfer require RDY.
The optional reset GPIO is configured inactive at startup without a reset pulse.
Changes to the initialization header take effect on the next IC reset; a host
reboot with SHOW_RESET clear keeps the previous sensor settings.

An IC reset detected at runtime takes the full initialization path. Existing
bounded software-reset recovery remains available for initialization failures.

The driver applies these communication and ATI settings while pre-patching init-data:

- ALP Setup, including ALP Enable (bit 31), retains the Azoteq header values.
  An ALP-enabled export therefore uses ALP sensing in LP2.
- LP2 sampling period is fixed at 500 ms; LP1/LP2 Auto-Prox is disabled.
- Automatic TP Re-ATI and its event are enabled. ALP Re-ATI/events and
  `TP_TOUCH_EVENT` are disabled. Re-ATI retry time is fixed at 1 second.
- ATI target, reference drift limit, and both negative and positive delta
  Re-ATI thresholds retain the Azoteq header values. Tune these values in
  `IQS9150_init.h`. Positive delta recovery is intended for snap domes; for a trackpad
  that does not use it, supply a sufficiently high threshold in the header.

Both Active and LP2 use Event Mode, including disabled-by-default startup.
Init-data transfer and TP Re-ATI completion still run in Streaming Mode; the
final initialization steps explicitly enable and verify Event Mode regardless
of the requested output state. ALP, TP Touch, Gesture, Switch and Snap events
are explicitly disabled in both modes. TP Event is enabled only in Active;
LP2 disables it because ALP sensing
does not refresh trackpad coordinates. Re-ATI Event remains enabled in LP2.

Enable/disable and Device PM transitions go directly to the power-mode write.
Every control step uses clock-stretch Force Comms without waiting for an event
or RDY: mode write, mode read-back, Event policy write, and Event policy read-back.
Each step performs one I2C transaction and ends its communication window with
STOP. Disable/PM suspend completes only after both LP2 and Event Mode are
confirmed. The output gate stays closed until both Active and Event Mode are
confirmed on enable, with the existing two-frame pointer baseline guard afterward.
A pointer frame checks the output gate and request generation once before
submission. If disable arrives after X has been queued, the frame still sends
Y with sync, so an orphaned X cannot accumulate into a later report. Subsequent
frames are suppressed.
The driver initiates control I2C immediately; clock stretching can still delay
each transaction until the IC is ready. Actual transition latency needs hardware
validation.

Normal Active/LP2 reads wait for event RDY. With no event, the driver waits
without periodic Force Comms polling. API and timer wakes do not count as RDY
samples. The three-sampling-period watchdog applies only when Streaming is
expected, not during normal Event Mode operation.

In LP2, the driver reads only fresh `INFO_FLAGS`; coordinates and `NUM_FINGERS`
are not assumed to update. It remains in LP2 until an enable or PM request
changes its state. There is no periodic coordinate probe, reseed timer, or
stationary-finger detection in either LP2 or Active.

The driver requests TP ATI only during initialization, including reinitialization
after an IC reset. It never requests TP/ALP Reseed at runtime. Reseed copies the
latest measured counts into the reference; it does not perform ATI sensitivity
adjustment (datasheet section 5.5.3). A real stationary contact therefore no
longer causes the driver to overwrite the reference.

Runtime recovery relies on the IC's automatic TP Re-ATI. Negative delta exceeding
the header threshold for 15 consecutive sensing cycles triggers Re-ATI. Its
counter may restart on a power-mode change. The automatic Re-ATI enable is
shared with reference drift and positive delta recovery; enabling it does not
select negative delta alone. Reference drift remains subject to the header
limit. Positive delta is for snap-dome recovery and can be kept inactive with
a sufficiently high header threshold (datasheet section 5.8.2).

ATI Error retries are handled by the IC with the configured one-second suppression
time. Runtime Re-ATI releases held buttons, cancels pending tap/drag/inertia work,
and resumes input with the existing two-frame pointer baseline guard. Device PM
suspend resets input state and stops communication after entering LP2; resume
restores the saved enabled state. INFO logs cover the ATI policy, `reati`, and
`communication` state changes. WARN logs identify communication fallback, failed
verification and ATI errors.

Host checks (no local west build is required):

```sh
python3 tests/init_event_mode.py
python3 tests/runtime_power.py
python3 tests/pointer_frame.py
```

The tests compile actual initialization/runtime control flow with mocked Zephyr
and IC services, and check that long LP2 waits and stationary single/multiple
contacts do not initiate reseed or coordinate probes. They also check idle LP2
Event Mode, Force Comms enable/disable without RDY, Streaming watchdog timing,
request reversals, Re-ATI/ATI Error, ALP/header tuning preservation and PM handling.
Hardware validation should include startup, tap/drag, scroll/inertia, 3/4-finger
swipe, runtime enable/disable, stationary single/multiple fingers, negative delta
recovery, communication fallback and reset.

## Key differences from IQS5xx driver

This driver is forked from the [zmk-driver-azoteq-iqs5xx](https://github.com/user/zmk-driver-azoteq-iqs5xx) driver with the following major changes:

- **Byte order**: IQS9150 uses little-endian (IQS5xx uses big-endian).
- **Register addresses**: Completely new register map starting from `0x1000`.
- **I2C address**: Default `0x56` (IQS5xx uses `0x74`).
- **RDY line**: Active-low by default (IQS5xx is active-high).
- **Gesture support**: Extended with double tap, triple tap, swipe-and-hold, and more.
- **Max touches**: Up to 7 fingers (IQS5xx supports up to 5).

### Harbour activation policy regression checks

Layer selection and persistence belong to the LandsFall central firmware. The existing settings API v2 and asynchronous `iqs915x_set_enabled()` are reused; its successful return confirms request acceptance, not completed Active-mode transition.

Run `python3 tests/runtime_session_reset.py` for the production request/reset paths: immediate output gating, button/hold release, inertia cancellation and stale work rejection after re-enable. Also run `tests/runtime_power.py`, `tests/pointer_frame.py`, `tests/init_header.py` and `tests/init_data.py`. These host tests complement firmware builds and do not establish physical USB/BLE behavior.

### Device PM failure recovery

Suspend must run before the parent I2C controller is suspended, and resume
must run after the controller is available. Suspend waits for LP2 and Event
Mode confirmation. If it fails, Zephyr keeps the device ACTIVE: the driver
restores the previous enable request asynchronously with a new generation
to supersede late LP2 work. This restores intent, not proof of hardware recovery
on a broken bus. Runtime enable requests return `-EBUSY` during a PM transition
or while suspended, and must be retried by the caller if still needed.

Run `python3 tests/pm_recovery.py` for PM rollback and request exclusion.


## Lifecycle and IC Suspend

The current driver offers `iqs915x_set_mode()` for Active, LP2 and the IC's
retentive Suspend mode, plus `iqs915x_get_status()` for cached lifecycle and
completion state. Unselected sensors start with output closed and enter Suspend
after initialization. Suspend entry finishes with the write's STOP; no confirming
read wakes the sensor again. Suspend exit clears the bit using Force Comms and
allows the IC's automatic reseed. Device PM uses Suspend and restores the previous
requested mode.

Startup can fall back to Force Comms for quiet retained states. Three failed probe transactions latch
ABSENT; the using firmware must configure a finite I2C transfer timeout (Bretagne:
1000 ms). Initialization has a 15-second deadline and otherwise latches FAULT.
Terminal states disable RDY and stop all retries until this MCU boots again;
reading status never re-probes the sensor. A known sensor's SHOW_RESET starts
recovery and reapplies the latest requested mode. The initialization header stays
owned by this driver. When the MCU is powered off it cannot service a sensor reset.

Tests: `init_header.py`, `init_data.py`, `init_event_mode.py`, `runtime_power.py`,
`runtime_session_reset.py`, and `pointer_frame.py` in `tests/`. Runtime power tests
compile the production boot loop, mode state machine and PM/API paths, including
three-attempt absence latching, busy initialization timeout, Suspend entry/resume,
and cached access without I2C. A successful build is not hardware power validation.

## Contact-frame output API

`iqs915x_subscribe_contacts()` installs one synchronous subscriber per sensor.
The default output remains `IQS915X_OUTPUT_MOUSE`. Select
`IQS915X_OUTPUT_CONTACTS` with `iqs915x_set_output_mode()` to receive validated,
coordinate-corrected sensor contacts instead of relative mouse/gesture events.
No Windows/HID protocol code is part of this API. The source is the existing
44-byte read through finger 4; no extra I2C transaction is introduced per window.

The callback must copy the complete frame into bounded consumer storage and
return promptly. It must not retain the pointer, block, perform I2C or call
back into driver APIs. It runs in the sensor thread under a subscription mutex.
`count` includes final `tip=false` records. `slot` identifies the sensor memory
slot; `id` remains stable during a contact and changes when that slot is reused.
IDs of concurrent slots cannot collide. `generation` identifies a contact session,
independent of power-request generation; `sequence` increments across sessions.
`sample_time_us` is MCU read-completion time with millisecond resolution, not a
sensor hardware timestamp or a promise of 125 Hz sampling. Use
`iqs915x_get_contact_resolution()` for cached coordinate maxima before mapping
coordinates to another protocol.

The memory slot tracks the same finger between samples according to the Azoteq
[IQS9150/IQS9151 datasheet](https://www.azoteq.com/images/stories/pdf/IQS9150_IQS9151_datasheet.pdf),
section 7.2.6. Occupancy comes from finger count and valid XY, not Confidence.
A Confidence decrease is reported while the finger remains present. Slot holes
are supported. Invalid XY, count/slot disagreement, more than four contacts or
the Trackpad Flags Too Many Fingers bit end the current session with releases
and quarantine tracking until a fresh all-up sample. The bit is in Trackpad
Flags bit 5 (appendix A.5), despite section 7.3 referring to Info Flags.

Reset, re-ATI, power transitions, output changes and contact-path I2C read
failures also end the contact session. A frame with `SESSION_END` closes its
`generation`, including when its contact count is zero; consumers must release
that generation and reject delayed frames belonging to it. Strict restarts wait
for all fingers to lift; ordinary power transitions follow the policy below.
Output switching immediately closes the
input gate, then the sensor thread cancels delayed mouse/tap/inertia work,
releases buttons and reapplies the independently selected power mode using the
existing state machine. Mode selection is asynchronous; `get_output_mode()`
reports the requested selection. Power/request status is separately available
through `iqs915x_get_status()`.

To detach, request MOUSE and retry the NULL subscription until the transition
has completed (`-EBUSY` while pending). A successful detach guarantees that the
old callback has finished and will not run again. A terminal device permits
detach after its session-end notification. Replacing a subscriber without detach
returns `-EALREADY`. API requests are thread-context only. A registered subscriber
may receive session-end notifications during power changes even in mouse mode.

Contact output bypasses mouse finger-count debounce, pointer acceleration,
tap/drag generation, scroll/inertia and swipe conversion. Driver initialization,
coordinate validation/correction and power ownership remain unchanged. Switching
back to mouse waits for all-up once; ordinary mouse power transitions retain
the existing behavior. Higher layers must handle queue loss, host negotiation
and mapping; enabling this API alone does not connect sensor contacts to PTP.

Host-side tests (no device writes):

```sh
python3 tests/contact_frames.py
python3 tests/contact_api.py
python3 tests/runtime_power.py
python3 tests/runtime_session_reset.py
python3 tests/pm_recovery.py
```


When Contacts output resumes in Active Event Mode, the driver requests one fresh
stream snapshot in its own communication window. This observes an already idle
surface without waiting for the first touch event, so the first new touch is not
consumed by an unresolved all-up barrier. When such a strict barrier exists, a
finger present in that snapshot must still lift; ordinary power resume reports
it immediately. Cached data and LP2 status never clear a strict barrier. After
that snapshot, normal event waiting resumes; no periodic idle polling is introduced.


PTP/contact power policy: ordinary Active/LP2/Suspend requests close the old
contact session and resume from the first valid fresh sample, including fingers
already on the surface. They do not create an all-up barrier. An existing barrier
from tracking loss, reset, re-ATI, I2C failure, or an explicit output-mode request
survives power changes. Re-requesting CONTACTS explicitly renews its session and
requires all-up, even when CONTACTS is already selected. Use a power request for
ordinary activation, and an output request for transport/session recovery. Mouse
output and its existing state machine are unchanged. Active sampling remains
15 ms; this policy does not change report rates.

The one-shot fresh Active snapshot also covers an explicit Contacts-to-Mouse
output switch while its all-up gate is pending. Ordinary Mouse power transitions
do not add this read or a new gate. A finger present at the snapshot still must
lift; an idle snapshot permits the first new touch without discarding it.

### Stationary contact refresh in raw-contact mode

Raw-contact output refreshes held, accepted contacts with a real sensor read at
its configured Active sampling period (currently 15 ms). Event-mode RDY updates
still trigger reads earlier; a deadline initiates a fresh I2C transaction when
stationary coordinates do not generate an event. Cached coordinates are never
retimestamped as new measurements. Each read remains one transaction/window.
Idle/no-contact, quarantined, disabled and legacy Mouse states retain event
waiting. This addresses a reporting-cadence gap; whether it resolves Windows
stationary repeated-tap behavior still requires hardware verification.

Raw contact consumers may call `iqs915x_set_contact_periodic(dev, false)` to
retain event-only acquisition while stationary. The default remains periodic
held-contact acquisition for existing PTP clients. Power gates still dominate
both policies. `iqs915x_get_gesture_profile()` exports the immutable DTS/init-data
thresholds and geometry without sensor I/O (also usable with an absent local IC).
The pure motion-history helper is exported as `iqs915x_motion.h` for consumers;
this does not run or enable the driver's legacy gesture state machine.

### Per-device channel-disable profiles

`channel-profile` defaults to `export`, preserving all Azoteq export mask bytes.
`bretagne-right` and `bretagne-left` require the exported 18 Rx x 12 Tx geometry.
Right disables Rx0..5 in Tx9..11; left disables Rx12..17 in Tx9..11. Each profile
contains exactly 18 disabled channels. Profile bytes are owned by
`iqs915x_sensor_config.h`, applied by the same init-chunk preparation function
for writing and verification. XY flips/correction do not alter this mask.

Warm startup verifies all 88 retained mask bytes with bounded reads before
Re-ATI. A mismatch requests software reset and the full verified init transfer;
I2C errors return through the existing init retry path. This prevents an older
right-side mask surviving a firmware update on the left IC. No runtime polling
or per-contact processing is added by channel profiles.

## Startup and calibration recovery

Cold startup writes the profile in Streaming/Manual Active, clears Show Reset,
and requests TP Re-ATI once. Wait for Occurred for 5000 ms using elapsed time,
including one final status transaction if RDY is missing. A missing notification
permits one software reset/reinitialization; total startup/recovery is bounded
by 15000 ms. Final failure attempts Suspend once and latches until MCU reboot.
Software-reset status checks are forced, not dependent on RDY edge counts.

Warm MCU startup checks retained geometry/channel masks, wakes the IC, and restores
Event Mode without requesting ATI or reseed. Mismatched profiles still reinitialize.
Occurred may coexist with ATI Error: configuration completes and normal requested
Active/LP2/Suspend policy resumes. TP automatic Re-ATI and its event remain enabled.
ATI Error gates both Mouse and raw-contact output, ends the session on error/recovery,
and never pins Active, polls a held contact, or issues periodic host ATI requests.
`iqs915x_status.ati_error` is cached MCU state for a local indicator, not an I2C read.
Terminal set_mode records UI intent only; it cannot restart a latched sensor.

## ATI Compensation channel diagnostics

`iqs915x_channels_start()` queues a capture on the sensor thread; it requires
READY and requested/actual Active and never wakes an unselected/LP2/Suspend IC.
`iqs915x_channels_page()` reads cached 8-value pages fenced by capture ID.
`-EINPROGRESS`, cancellation, I/O, timeout, mapping mismatch and stale IDs are
separate results; partial captures are never returned as successful pages.

Capture temporarily closes input and disables TP/ALP automatic Re-ATI, verifies
the setting, reads the packed 18x12 ATI Compensation words from 0xD000 in 16-byte transactions,
reads actual channel disables and RxTx mapping, checks status and restores/verifies
the original configuration on every exit. Request generation changes cancel it.
A failed restore routes into bounded normal recovery. The capture deadline is
2000 ms (plus bounded I2C/restore time). On ATI Error, the first stable selected
Active opportunity requests one capture per error episode. A failed/cancelled
attempt is not retried automatically; a cleared error rearms it. No capture in
LP2/Suspend/absence and no periodic capture. Manual start can refresh the cache.

The snapshot supports only the verified 18x12 mapping in the configured init-data.
Rows/columns correspond to physical grid positions; both halves use the same
orientation. Only bits 9:0 are ATI Compensation (divider bits 14:10 are excluded).
Enabled channels at 0 or 1023 are logged as ATI-limit candidates,
with zero-based 6x4 key positions computed from each 3x3 group. Disabled channels
are excluded. ATI Error or low Confidence alone does not identify a faulty channel.
A non-limit value does not exclude the Count-outside-range ATI error condition.
Source: IQS9150/IQS9151 Datasheet v1.1, sections 5.2.1, 5.8.3 and A.26,
https://www.azoteq.com/images/stories/pdf/IQS9150_IQS9151_datasheet.pdf .

### Terminal-failure evidence

Terminal lifecycle retains the original `last_error`; successful or failed cleanup
I2C cannot erase it. Before best-effort Suspend, a one-time read-only probe captures
Info Flags and, only when meaningful, Config Settings and System Control. Busy
0xEEEE responses are raw evidence, never ordinary flags or configuration; the
valid mask marks interpretable values. Absent devices are not probed.
`iqs915x_fault_get()` returns this cached evidence without waking/retrying the IC.
Probe read statuses, the original cause, init phase/retries and Suspend status are
separate. There are at most three probe reads, no retry loops; normal I2C transaction
timeouts still apply in addition to the init/recovery deadline. This is not a
channel-reference capture and does not declare a faulty channel.

### First initialization configuration mismatch

The first non-busy CONFIG_SETTINGS mismatch is saved before rewriting or resetting
the IC. `iqs915x_mismatch_get()` reads this independent MCU cache without I2C,
including after the retry succeeds or terminal failure occurs. A 16-byte read from
0x11BC saves Control/Config/Other/ALP bytes in one window, then a separate Info Flags
read is attempted. No retries are added; later mismatches do not recapture.
Expected/observed values are the original failed comparison, not later probes.
Per-read results and valid masks separate unavailable/busy responses from values.
Config/Control evidence is derived from the raw block, with the block's I/O status.
The terminal evidence remains separate and is never overwritten by this getter.

The first initial Re-ATI timeout is retained separately with
`iqs915x_ati_timeout_get()`. It captures Info/Config/Control before restarting the
IC on an elapsed deadline (missing Occurred, busy or read failure). At most three
read windows are used; busy or failed Info skips subsequent reads, and busy Config
skips Control. No write or retry is added. Validity/read status must be checked;
the reads are sequential, not atomic. The getter performs no I2C and survives IC
retries/terminal failure until MCU reboot. stop_status is ENODATA because this
capture precedes the restart, not a terminal Suspend.

Initial Re-ATI trace (`iqs915x_ati_trace_get`) retains the first attempt per MCU
boot. One new diagnostic Config read window is inserted after Streaming write
and before Control request. The latest readback is recorded. Busy waits for a new bounded opportunity;
read errors use existing backoff, and a valid mismatch rewrites the intended
Streaming configuration up to three checks. ATI cannot be queued before a
matching non-busy readback. The 15-second initialization deadline remains. Config write status refers to the
first attempted Streaming write; request status is the latest request write in
that attempt. Waiting uses the existing stream reads: valid/busy/error counts,
first busy time, first/last valid Info and RDY semaphore timeout count add no I2C.
Outcome 1=Occurred,2=ATI deadline,3=reset/restart,4=terminal,0=ongoing. A successful
attempt is frozen too. Subsequent retries do not overwrite it. Counts distinguish
zero timestamps/no sample; valid Info means non-EEEE, not successful calibration.
The trace freezes before post-timeout diagnostic reads, avoiding their inclusion.

### Startup communication windows

Startup register, block and stream transactions now wait for asserted RDY at the
low-level boundary, including warm-start checks and first-mismatch evidence.
A conservative 1 ms host guard after the preceding transaction avoids immediate
reuse of the preceding window; it is not a datasheet timing guarantee. A stale
semaphore token alone never authorizes access. Each wait is bounded by 150 ms and
the existing 15 s initialization deadline. Initial probing, software-reset
recovery and Event Mode preparation/confirmation may fall back to Force Comms
method 0 after that wait, because retained Event Mode/Suspend can be quiet.
Init-data/ACK reset and Streaming preparation, verification and Re-ATI wait do
not force access when RDY is absent. A missing window does not classify the IC as
absent. Existing bounded reset/retry limits and verified configuration gate remain.

`Init comms: rdy=... force=... no_window=...` reports startup access decisions
(cumulative per MCU boot, including initialization retries). Terminal evidence
and best-effort Suspend are outside these counters and remain bounded diagnostic
accesses. `0xEEEE` is rejected as unusable state/configuration data; the legacy
`busy` diagnostic name does not imply a documented IC busy-response code.
Runtime sampling, Event Mode, LP2/Suspend and ATI-error auto-retry policy are
unchanged. Hardware validation is required; this does not establish that any
observed corruption originated in firmware rather than bus/power integrity.

ATI-error assistance runs only in confirmed Active with output requested. After
2 s without an ATI occurred notification (1 s retry plus 1 s margin), it requests
TP ATI once and waits up to 5 s. A missing completion or ambiguous write never
causes repeated requests; disable/re-enable the layer to retry. Power generation
changes, disabled output and channel diagnostics discard elapsed time. Every ATI
occurred notification rearms the 2 s interval if error persists. Normal sensing
without ATI error has no additional timer polling.


Temporary diagnostic jobs
-------------------------
`iqs915x_diagnostic_start(dev, run_ati, id)` queues one bounded sensor-thread job
from READY, including LP2/Suspend, without changing requested mode or settings.
Input closes before temporary Active. With run_ati, Streaming is selected and a
TP ATI request waits up to 5 s for RDY/Occurred. If the ATI error clears, no
Compensation capture is necessary (successful result with info_after error bit
clear). Otherwise the existing bounded Compensation capture runs. The latest
requested power mode is restored before the result becomes available. Result
ati_status and restore_status distinguish ATI and restoration failures. A changed
power generation or PM request cancels sampling; restore reconciles newest intent.
Host disconnect does not prolong a job: sensor-thread deadlines finish restoration
without host participation. No persistent policy is changed.

### Multi-contact trace monitor

The circle-calibration acquisition has been replaced by a four-contact diagnostic
monitor. START/RENEW/STOP/PAGE still use processing operations 2/3/4/5. START
requires 10 bytes: op:u8, session:u32LE, after=0:u32LE, monitor_IIR:u8 (0/1).
Other requests remain 9 bytes. Old 9-byte START is rejected. The 3500ms MCU
lease and pre-session power restoration remain; PM restoration does not wake
a suspended sensor. Firmware owns its shorter lease and normal-policy restoration.

Wire v2 has a 20-byte header: version:u8=2, active:u8, session:u32,
last_sequence:u32, overwritten:u32, maxX:u16, maxY:u16, count:u8 (0..2), IIR:u8.
Each 36-byte frame has sequence:u32, time_ms:u32, flags:u16, mask:u8,
confidence:u8, then four {x:u16,y:u16,id:u16}. Total reply is 20..92 bytes.
All values are little endian. IDs change on physical-slot reuse. Low confidence
is reported separately; read failure, inconsistent contact count and reset
close tracks. Consumers break trails on sequence gaps. Ring size is 128 frames.

Positions are pre-map sensor coordinates. Optional dynamic IIR runs with a
snapshot of applied settings and independent per-slot monitor histories.
The normal settings, map, filter history and NVS are never modified by monitoring.
Steady frames are decimated to 30ms, while contact/confidence/error boundaries
are retained immediately. IIR consumes every acquired sample. There are no
extra reads/timers/communication outside monitoring; the inactive raw branch
returns after an atomic check. The contact subscriber is unchanged.
See tests/trace_capture.c for production-code state/transport/IIR tests.

## Fixed IC filtering and finger split

IC runtime initialization overrides are retired. The compiled init-data uses
Finger Split Factor5, dynamic IIR enabled, Bottom Beta100, and bottom/top
15,000/16,500 pixel/cycle. At the fixed Active period15ms these correspond to
1,000,000/1,100,000 pixel/s. MCU processing/IIR and raw trace APIs remain.
Warm initialization verifies these registers against compiled init-data and
resets/reloads retained settings when different. No late saved-profile reset.

### Cached status notifications

`iqs915x_subscribe_status()` registers up to two lifetime consumers. Each receives
an initial notification and notifications when lifecycle, power mode, transition,
calibration health or request generation changes. Callbacks run on the sensor
thread outside driver locks and must only schedule work. Read the latest state
with `iqs915x_get_status()` from that work; notifications coalesce intermediate
states. Coordinate updates alone do not notify, and registration/status reads
never add sensor I/O or wake an IC in Suspend. Terminal devices remain latched
until MCU restart.

## Retired calibration interface

The obsolete `coordinate-correction` devicetree property and log-derived LUT
generator have been removed. Current processing settings and four-contact
monitoring remain the supported calibration interfaces.
