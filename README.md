# ZMK driver for Azoteq IQS9150/IQS9151 trackpads

## Compatibility

This driver is designed for the IQS9150/IQS9151 series trackpad controllers. It has been developed based on the IQS9150/IQS9151 datasheet (Revision v1.1) and is intended for use with the following modules:

- PXM0091 (IQS9150 module)

## Supported features

- Trackpad movement (relative coordinates).
- Single finger tap: Reported as a left click.
- Two finger tap: Reported as a right click.
- Press and hold: Reported as a continuous left click (allows click and drag).
- Vertical scroll.
- Horizontal scroll.

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
        natural-scroll-y;
        natural-scroll-x;

        switch-xy;
    };
};
```

## Key differences from IQS5xx driver

This driver is forked from the [zmk-driver-azoteq-iqs5xx](https://github.com/user/zmk-driver-azoteq-iqs5xx) driver with the following major changes:

- **Byte order**: IQS9150 uses little-endian (IQS5xx uses big-endian).
- **Register addresses**: Completely new register map starting from `0x1000`.
- **I2C address**: Default `0x56` (IQS5xx uses `0x74`).
- **RDY line**: Active-low by default (IQS5xx is active-high).
- **Gesture support**: Extended with double tap, triple tap, swipe-and-hold, and more.
- **Max touches**: Up to 7 fingers (IQS5xx supports up to 5).
