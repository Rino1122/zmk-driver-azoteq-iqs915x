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
- Kinetic (inertial) scroll.

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

        /* Kinetic (inertial) scroll settings */
        kinetic-scroll;
        kinetic-friction = <85>;     /* 0-100% velocity retention per tick */
        kinetic-interval-ms = <15>;  /* Time between kinetic updates */

        switch-xy;

        /* Init data for IQS9150 (no NVM) - see section below */
        azoteq,init-data = [ ... ];
    };
};
```

## Initialization data (IQS9150/IQS9151)

The IQS9150/IQS9151 does **not** have NVM, so all register settings must be written via I2C at every boot. The `azoteq,init-data` property provides this data.

### How to generate init-data

1. Use the **Azoteq GUI** to configure your trackpad and export `IQS9150_init.h`.
2. Run the conversion script:
   ```bash
   python3 scripts/convert_init_header.py IQS9150_init.h
   ```
3. The script outputs a DTS snippet like this (abbreviated):
   ```
   azoteq,init-data = [
       /* ALP ATI Compensation (0x115C, 26 bytes) */
       00 00 00 00 00 00 00 00 ...
       /* Rx/Tx Mapping (0x1218, 46 bytes) */
       19 0C 18 0B 17 0A 16 09 ...
       ...
   ];
   ```
4. Copy the entire `azoteq,init-data = [ ... ];` block and paste it into your shield overlay inside the iqs915x node:

   ```dts
   /* your_shield.overlay */
   &i2c0 {
       trackpad: iqs915x@56 {
           compatible = "azoteq,iqs915x";
           reg = <0x56>;
           rdy-gpios = <&gpio0 15 GPIO_ACTIVE_LOW>;

           /* Paste the conversion script output here */
           azoteq,init-data = [
               /* ALP ATI Compensation (0x115C, 26 bytes) */
               00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
               00 00 00 00 00 00 00 00 00 00
               /* I2C Slave Address (0x1176, 2 bytes) */
               FF FF
               /* ... (remaining sections) ... */
               /* Eng Settings (0x2000, 6 bytes) */
               01 00 03 03 14 00
           ];

           /* These DTS properties override init-data values */
           one-finger-tap;
           scroll;
           tap-time = <150>;
           report-rate-ms = <10>;
       };
   };
   ```

### Priority

When both `azoteq,init-data` and individual DTS properties (e.g. `tap-time`, `report-rate-ms`) are specified, `init-data` is written first, then individual properties override the corresponding registers. This allows you to use the GUI for board-level settings (Rx/Tx mapping, ATI) while fine-tuning gesture behavior in the overlay.

This priority is determined by the driver's initialization sequence in C code, **not** by the order of properties in the DTS file. You can place `azoteq,init-data` and other properties in any order within the node.

## Key differences from IQS5xx driver

This driver is forked from the [zmk-driver-azoteq-iqs5xx](https://github.com/user/zmk-driver-azoteq-iqs5xx) driver with the following major changes:

- **Byte order**: IQS9150 uses little-endian (IQS5xx uses big-endian).
- **Register addresses**: Completely new register map starting from `0x1000`.
- **I2C address**: Default `0x56` (IQS5xx uses `0x74`).
- **RDY line**: Active-low by default (IQS5xx is active-high).
- **Gesture support**: Extended with double tap, triple tap, swipe-and-hold, and more.
- **Max touches**: Up to 7 fingers (IQS5xx supports up to 5).
