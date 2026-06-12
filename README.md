# ZMK driver for Azoteq IQS9150/IQS9151 trackpads

## Compatibility

This driver is designed for the IQS9150/IQS9151 series trackpad controllers. It has been developed based on the IQS9150/IQS9151 datasheet (Revision v1.1) and is intended for use with the following modules:

- PXM0091 (IQS9150 module)

## Supported features

- Trackpad movement from calibrated absolute finger coordinates, emitted as relative input.
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
        trigger-ms = <35>;                    /* Wait before inertia starts */
        scroll-decay-factor-int = <85>;       /* Retention percent per tick (0-100) */
        scroll-report-interval-ms = <65>;     /* Time between inertia updates */
        scroll-threshold-start = <2>;         /* Arm inertia above this velocity */
        scroll-threshold-stop = <0>;          /* Stop inertia at/below this velocity */

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
`0x1026` as the internal pointer source. Raw coordinates are calibrated with
empirical per-axis tables generated from diagonal motion logs, then converted
to consecutive-sample deltas and emitted as `INPUT_REL_X`/`INPUT_REL_Y` to the
host. The first sample after touch-down is used as a baseline (no cursor move),
then relative movement is reported while `TP Movement` is asserted. Coordinates
outside the logged active range pass through unchanged so future corner reports
are still handled.

For coordinate calibration, enable `CONFIG_INPUT_AZOTEQ_IQS915X_COORD_LOG=y`.
The driver emits raw touched stream samples before calibration as INFO logs in
the form `coord,t=...,f=...,x1=...,y1=...`. Keep this disabled for normal use
because it logs every touched sample.

In Event Mode, the driver enables `TP_EVENT` as the only event source and
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

The current scroll inertia model uses two-finger centroid deltas from absolute
coordinates, then follows a Q8 fixed-point decay flow with remainder
preservation and round-to-nearest output.

Legacy properties (`scroll-inertia-decay`, `scroll-inertia-interval-ms`,
`scroll-inertia-stale-gap-ms`, `scroll-inertia-min-avg-speed`) are still
accepted as fallback for compatibility, but new configurations should use
`trigger-ms` and `scroll-*` properties shown above.

See [docs/scroll_parameters_ja.md](docs/scroll_parameters_ja.md) for a
practical Japanese guide to each scroll parameter and tuning workflow.

## Initialization data (IQS9150/IQS9151)

The IQS9150/IQS9151 does **not** have NVM, so all register settings must be written via I2C at every boot.
This driver now uses a built-in init-data profile and no longer accepts an `azoteq,init-data` DTS property.

- Base profile source: `drivers/input/IQS9150_init.h` (Azoteq export format)
- Generated array: build directory (`drivers/input` target binary dir under `generated/`)
- Total length: 1174 bytes (`0x115C..0x15EB` + `0x2000..0x2005`)

### Updating the built-in init-data profile

1. Use the **Azoteq GUI** to configure your trackpad and export a new `IQS9150_init.h`.
2. Replace `drivers/input/IQS9150_init.h` with the exported header as-is.
3. Build firmware. The array header is generated automatically before compiling `iqs915x.c`.
4. Keep DTS tuning properties (`report-rate-ms`, scroll settings, etc.) in your overlay as needed.

The generated array header is a build artifact and does not need to be committed.

### Priority

The driver writes built-in init-data first, then applies individual DTS properties (e.g. `report-rate-ms`) as register overrides.
This priority is determined by the driver's initialization sequence in C code, not by DTS property order.

## Key differences from IQS5xx driver

This driver is forked from the [zmk-driver-azoteq-iqs5xx](https://github.com/user/zmk-driver-azoteq-iqs5xx) driver with the following major changes:

- **Byte order**: IQS9150 uses little-endian (IQS5xx uses big-endian).
- **Register addresses**: Completely new register map starting from `0x1000`.
- **I2C address**: Default `0x56` (IQS5xx uses `0x74`).
- **RDY line**: Active-low by default (IQS5xx is active-high).
- **Gesture support**: Extended with double tap, triple tap, swipe-and-hold, and more.
- **Max touches**: Up to 7 fingers (IQS5xx supports up to 5).
