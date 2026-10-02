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
        /* Required board profile: init-data and coordinate calibration. */
        profile = <&board_iqs915x_profile>;

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
        trigger-ms = <35>;                    /* Wait before inertia starts */
        scroll-decay-factor-int = <85>;       /* Retention percent per tick (0-100) */
        scroll-report-interval-ms = <65>;     /* Time between inertia updates */
        scroll-threshold-start = <2>;         /* Arm inertia above this velocity */
        scroll-threshold-stop = <0>;          /* Stop inertia at/below this velocity */

        /* Optional absolute coordinate correction; disabled if omitted */
        coordinate-correction;

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

        switch-xy;
    };
};
```

The driver uses IQS9150 finger coordinates from registers `0x1024` and
`0x1026` as the internal pointer source. If `coordinate-correction` is set, raw
coordinates are calibrated with per-axis LUTs generated from `docs/logs/*.txt`.
If the property is omitted, raw IQS9150 absolute coordinates are used directly.
The driver converts consecutive-sample deltas to `INPUT_REL_X`/`INPUT_REL_Y` for
the host. The first sample after touch-down is used as a baseline (no cursor
move), then relative movement is reported while `TP Movement` is asserted.

If `pointer-accel` is enabled, the driver applies a lightweight integer-only
scale curve to single-finger `INPUT_REL_X`/`INPUT_REL_Y` reports immediately
before emission. Gesture recognition still uses the calibrated absolute
coordinates before acceleration, so tap, scroll, and swipe thresholds are not
changed. `pointer-sensitivity-percent` is the base scale; speeds at or below
`pointer-accel-threshold` use that base scale, speeds at or above
`pointer-accel-saturation` use `pointer-accel-max-percent`, and speeds between
them are linearly interpolated. Speeds are normalized to a 10 ms report interval
using `report-rate-ms` when configured. When `pointer-accel` is omitted and
`pointer-sensitivity-percent` is left at `100`, pointer output is unchanged from
the raw absolute-coordinate delta path.

Coordinate calibration assumes 6 X blocks and 4 Y blocks. Each block boundary
and block center is treated as a fixed point, and the same axis-specific
half-block LUT is mirrored across every block half, including corner blocks.
The LUT generator learns the curve from fully covered inner blocks, then applies
that one block-local curve to all blocks. To regenerate the LUTs, collect three
X-only and three Y-only raw coordinate logs under `docs/logs/` and run
`python3 scripts/generate_coord_lut.py`.

For coordinate calibration, enable `CONFIG_INPUT_AZOTEQ_IQS915X_COORD_LOG=y`.
The driver emits raw touched stream samples before calibration as INFO logs in
the form `coord,t=...,f=...,x1=...,y1=...`. Keep this disabled for normal use
because it logs every touched sample.

For scroll diagnostics, enable `CONFIG_LOG=y` and
`CONFIG_INPUT_AZOTEQ_IQS915X_LOG_LEVEL=4` on the trackpad peripheral. Each
nonzero axis delta passed to scroll normalization produces a DEBUG line:

```text
scroll_output,t=50000,source=manual,axis=wheel,delta=-20,wheel=-1,status=sent,rc=0,acc_before=0,acc_added=-10240,acc_after=-2240,denom=8000
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
  units multiplied by 512; a successful output consumes `wheel * denom`.

At release, `scroll_inertia_release,t=...,window_ms=...,vx=...,vy=...,threshold=...,start=...`
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
normalization.

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

A confirmed change from multiple fingers to one allows cursor movement using
the remaining valid slot without requiring a zero-finger interval. A change of
pointer slot establishes a new baseline instead of emitting the position jump.
An established scroll session retains its fractions if two fingers return
before release, but movement during the confirmed one-finger interval is not
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
axis is compared with `scroll-threshold-start`, and the same signed velocity
seeds the existing inertia decay flow. This normalization is independent of
sensor report rate; settings previously tuned at rates other than 10 ms may
need adjustment. `trigger-ms` controls start delay only, not motion freshness.
Motion and inertia use separate fractional accumulators so stopping or
cancelling inertia does not discard manual-scroll remainders. Inertia follows a
Q8 fixed-point decay flow with remainder preservation and stops when the
decayed motion no longer reaches HID output for several ticks.

The velocity estimator has host-side regression tests that do not require a
Zephyr workspace:

```sh
cc -std=c11 -Wall -Wextra -Werror -Idrivers/input tests/scroll_motion.c -o /tmp/iqs915x-scroll-motion-test
/tmp/iqs915x-scroll-motion-test
```

See [docs/scroll_parameters_ja.md](docs/scroll_parameters_ja.md) for a
practical Japanese guide to each scroll parameter and tuning workflow.

## Initialization data (IQS9150/IQS9151)

The IQS9150/IQS9151 does **not** have NVM, so all register settings must be written via I2C at every boot.
The board supplies an `azoteq,iqs915x-profile` phandle containing the complete
initialization byte stream and coordinate calibration tables. This keeps the
driver reusable across boards while preserving the existing 1174-byte export.

- `init-data`: 1174 bytes (`0x115C..0x15EB` + `0x2000..0x2005`)
- `x/y-coordinate-lut-q15`: per-axis half-block correction curves
- `x/y-coordinate-blocks`: profile-specific block counts

The original `drivers/input/IQS9150_init.h` is retained as the Azoteq export
source for byte-count validation and future profile regeneration. It is not
compiled into the driver.

### Updating a board profile

1. Export a new `IQS9150_init.h` with the **Azoteq GUI**.
2. Convert its values into the board's `azoteq,iqs915x-profile` DTS node.
3. Verify the profile contains exactly 1174 bytes and keep DTS tuning
   properties (`report-rate-ms`, scroll settings, etc.) in the sensor node.

### Priority

The driver writes the profile-provided init-data first, then applies individual DTS properties (e.g. `report-rate-ms`) as register overrides. The conversion script emits the profile DTS array by default; use `--c-header` only for legacy tooling.
This priority is determined by the driver's initialization sequence in C code, not by DTS property order.

The driver overrides the profile's LP2 sampling period (`0x11AA`) to 150 ms
during initialization to shorten the wait when returning from LP2. This is a
fixed driver setting and has no DTS override.

Runtime enable/disable mode writes and Event Mode relatching use clock-stretch
Force Comms without waiting for a finger-triggered RDY. Each state-machine step
performs one I2C transaction, ending its communication window with STOP. The IC
can still stretch the clock until communication is available, so transition
latency depends on its sampling period. Normal input reads remain RDY-driven.
LP2 relatching disables `TP_EVENT` so retained trackpad touch state does not
request repeated communication windows. Active relatching enables it again;
temporary Idle scans for periodic reseeding also retain `TP_EVENT`.

## Key differences from IQS5xx driver

This driver is forked from the [zmk-driver-azoteq-iqs5xx](https://github.com/user/zmk-driver-azoteq-iqs5xx) driver with the following major changes:

- **Byte order**: IQS9150 uses little-endian (IQS5xx uses big-endian).
- **Register addresses**: Completely new register map starting from `0x1000`.
- **I2C address**: Default `0x56` (IQS5xx uses `0x74`).
- **RDY line**: Active-low by default (IQS5xx is active-high).
- **Gesture support**: Extended with double tap, triple tap, swipe-and-hold, and more.
- **Max touches**: Up to 7 fingers (IQS5xx supports up to 5).
