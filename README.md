# ZMK driver for Azoteq IQS9150/IQS9151 trackpads

## Compatibility

This driver is designed for the IQS9150/IQS9151 series trackpad controllers. It has been developed based on the IQS9150/IQS9151 datasheet (Revision v1.1) and is intended for use with the following modules:

- PXM0091 (IQS9150 module)

## Supported features

- Trackpad movement (relative coordinates).
- Optional absolute X/Y reporting from FINGER1_X/Y.
- Single finger tap: Reported as a left click.
- Two finger tap: Reported as a right click.
- Press and hold: Reported as a continuous left click (allows click and drag).
- Vertical scroll.
- Horizontal scroll.
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
        press-and-hold;
        press-and-hold-time = <250>;
        two-finger-tap;

        scroll;

        /* Scroll inertia settings */
        scroll-inertia;
        scroll-inertia-decay = <850>;         /* x/1000 velocity retention per tick */
        scroll-inertia-interval-ms = <15>;    /* Time between inertia updates */
        scroll-inertia-recent-window-ms = <60>;
        scroll-inertia-stale-gap-ms = <35>;
        scroll-inertia-min-samples = <1>;
        scroll-inertia-min-avg-speed = <4>;

        /* Use FINGER1_X/Y as internal source and emit REL_X/Y deltas */
        report-absolute;

        /* 3/4 finger swipe as one-shot virtual key events */
        three-finger-swipe;
        four-finger-swipe;
        /* 0 = auto threshold from X/Y resolution with ratio */
        swipe-step = <0>;
        swipe-threshold-numerator = <1>;
        swipe-threshold-denominator = <5>;

        /* Optional: override emitted keycodes (defaults: F13..F20) */
        three-finger-swipe-up-key = <INPUT_KEY_F13>;
        three-finger-swipe-down-key = <INPUT_KEY_F14>;
        three-finger-swipe-left-key = <INPUT_KEY_F15>;
        three-finger-swipe-right-key = <INPUT_KEY_F16>;
        four-finger-swipe-up-key = <INPUT_KEY_F17>;
        four-finger-swipe-down-key = <INPUT_KEY_F18>;
        four-finger-swipe-left-key = <INPUT_KEY_F19>;
        four-finger-swipe-right-key = <INPUT_KEY_F20>;

        switch-xy;
    };
};
```

When `report-absolute` is enabled, the driver uses raw IQS9150 finger 1
coordinates from registers `0x1024` and `0x1026` as the internal pointer
source, computes deltas between consecutive samples, and emits
`INPUT_REL_X`/`INPUT_REL_Y` to the host. The first sample after touch-down is
used as a baseline (no cursor move), then relative movement is reported while
`TP Movement` is asserted. If the property is omitted, the driver uses
`REL_X`/`REL_Y` registers directly.

When `three-finger-swipe` or `four-finger-swipe` is enabled, the driver tracks
the centroid of active fingers and emits one-shot virtual key tap events based
on the dominant swipe direction. One gesture emits only one key event until
fingers are released.

By default (`swipe-step = <0>`), swipe thresholds are computed from init-data
X/Y resolutions (registers `0x11E6`/`0x11E8`) using
`swipe-threshold-numerator` / `swipe-threshold-denominator`.
Default `1/5` means a gesture triggers at about 20% centroid travel on each
axis. If you set `swipe-step` to a value > 0, that fixed threshold overrides
the ratio-based calculation.

You can override each emitted key code using DTS properties
(`three-finger-swipe-up-key`, etc.). If omitted, defaults are F13..F20.

Virtual key mapping:

- 3-finger up/down/left/right -> F13/F14/F15/F16
- 4-finger up/down/left/right -> F17/F18/F19/F20

See [docs/gesture_virtual_keys_ja.md](docs/gesture_virtual_keys_ja.md) for a
firmware-side setup guide.

Important: this driver emits key events, but ZMK Studio only shows assignable
controls that exist as positions in your keyboard keymap/layout. To expose
gesture controls in Studio, add 8 gesture slots on the firmware side and map
them to the key codes emitted by this driver.

The scroll inertia settings replace the older kinetic-scroll settings.
If you previously used a friction value such as `85`, the direct replacement
is `scroll-inertia-decay = <850>;`. The old kinetic interval maps directly to
`scroll-inertia-interval-ms`.

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
4. Keep DTS tuning properties (`tap-time`, `report-rate-ms`, etc.) in your overlay as needed.

The generated array header is a build artifact and does not need to be committed.

### Priority

The driver writes built-in init-data first, then applies individual DTS properties (e.g. `tap-time`, `report-rate-ms`) as register overrides.
This priority is determined by the driver's initialization sequence in C code, not by DTS property order.

## Key differences from IQS5xx driver

This driver is forked from the [zmk-driver-azoteq-iqs5xx](https://github.com/user/zmk-driver-azoteq-iqs5xx) driver with the following major changes:

- **Byte order**: IQS9150 uses little-endian (IQS5xx uses big-endian).
- **Register addresses**: Completely new register map starting from `0x1000`.
- **I2C address**: Default `0x56` (IQS5xx uses `0x74`).
- **RDY line**: Active-low by default (IQS5xx is active-high).
- **Gesture support**: Extended with double tap, triple tap, swipe-and-hold, and more.
- **Max touches**: Up to 7 fingers (IQS5xx supports up to 5).
