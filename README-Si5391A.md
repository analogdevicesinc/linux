# Skyworks Si5391A Linux Driver

## Overview

This is a Linux kernel driver for the Skyworks Si5391A clock generator chip. The Si5391A is a high-performance, low-jitter clock generator that can produce up to 10 differential or single-ended output clocks from a single input reference.

The driver is based on the existing Si5341 driver architecture and provides full control over the device through the Linux Common Clock Framework (CCF).

## Features

- Support for up to 10 independent clock outputs
- Runtime frequency configuration for each output
- Support for multiple input clock sources (XTAL, IN0, IN1, IN2)
- Configurable output formats (LVDS, LVPECL, HCSL, LVCMOS)
- Device Tree integration
- Power management support
- I2C interface with page-based register access

## Driver Architecture

The driver is structured as follows:

```
clk-si5391a.c
├── Register Access Layer
│   ├── si5391a_reg_read()
│   ├── si5391a_reg_write()
│   ├── si5391a_bulk_read()
│   └── si5391a_bulk_write()
├── Clock Framework Integration
│   ├── PLL Operations (si5391a_pll_ops)
│   └── Output Clock Operations (si5391a_output_clk_ops)
├── Device Initialization
│   ├── si5391a_initialize_chip()
│   └── si5391a_dt_parse_dt()
└── I2C Driver Interface
    ├── si5391a_probe()
    └── si5391a_remove()
```

## Building the Driver

### Kernel Configuration

Add the following to your kernel configuration:

```
CONFIG_COMMON_CLK_SI5391A=y  # Built-in
# or
CONFIG_COMMON_CLK_SI5391A=m  # Module
```

### Makefile Integration

The driver should be added to `drivers/clk/Makefile`:

```makefile
obj-$(CONFIG_COMMON_CLK_SI5391A) += clk-si5391a.o
```

### Kconfig Entry

Add to `drivers/clk/Kconfig`:

```kconfig
config COMMON_CLK_SI5391A
    tristate "Clock driver for Skyworks Si5391A"
    depends on I2C
    depends on OF
    select REGMAP_I2C
    help
      This driver supports the Skyworks Si5391A clock generator.
```

## Device Tree Configuration

### Basic Configuration

```dts
&i2c1 {
    si5391a: clock-generator@74 {
        compatible = "skyworks,si5391a";
        reg = <0x74>;
        #clock-cells = <1>;

        /* Input clock - 48MHz crystal */
        clocks = <&ref48>;
        clock-names = "xtal";

        /* PLL configuration for 14GHz VCO */
        skyworks,pll-m-num = <14000>;
        skyworks,pll-m-den = <48>;
    };
};
```

### Advanced Configuration with Output Settings

```dts
&i2c1 {
    si5391a: clock-generator@74 {
        compatible = "skyworks,si5391a";
        reg = <0x74>;
        #clock-cells = <1>;
        #address-cells = <1>;
        #size-cells = <0>;

        clocks = <&ref48>;
        clock-names = "xtal";

        skyworks,pll-m-num = <14000>;
        skyworks,pll-m-den = <48>;
        skyworks,iovdd-33;  /* 3.3V I2C interface */

        /* Configure OUT8 for 200MHz */
        out@8 {
            reg = <8>;
            skyworks,format = <1>;      /* LVDS */
            skyworks,common-mode = <3>;
            skyworks,amplitude = <3>;
            always-on;
        };

        /* Configure OUT9 for 100MHz */
        out@9 {
            reg = <9>;
            skyworks,format = <1>;      /* LVDS */
            skyworks,common-mode = <3>;
            skyworks,amplitude = <3>;
            always-on;
        };
    };
};
```

## Using the Clock Outputs

### In Device Tree

Reference a specific output clock in your device node:

```dts
some-device {
    clocks = <&si5391a 8>;  /* Use output 8 */
    clock-names = "refclk";

    /* Set output 8 to 200MHz */
    assigned-clocks = <&si5391a 8>;
    assigned-clock-rates = <200000000>;
};
```

### From Kernel Code

```c
struct clk *clk;

/* Get clock output 8 */
clk = clk_get(&pdev->dev, "refclk");
if (IS_ERR(clk))
    return PTR_ERR(clk);

/* Set to 200MHz */
clk_set_rate(clk, 200000000);

/* Enable the clock */
clk_prepare_enable(clk);
```

### From User Space

Using the debugfs interface (if enabled):

```bash
# View all clocks
cat /sys/kernel/debug/clk/clk_summary

# View specific output
cat /sys/kernel/debug/clk/si5391a.8/clk_rate

# Set rate (if debugfs write is enabled)
echo 200000000 > /sys/kernel/debug/clk/si5391a.8/clk_rate
```

## Register Map Overview

Key register addresses used by the driver:

| Register | Address | Description |
|----------|---------|-------------|
| PAGE | 0x0001 | Page selection register |
| PN_BASE | 0x0002 | Part number base |
| DEVICE_REV | 0x0005 | Device revision |
| STATUS | 0x000C | Device status |
| SOFT_RST | 0x001C | Soft reset control |
| PLL_M_NUM | 0x0235 | PLL M divider numerator |
| PLL_M_DEN | 0x023B | PLL M divider denominator |
| OUT_EN | 0x0A03 | Output enable control |

Output-specific registers (N = 0-9):
- Config: 0x0108 + (N * 5)
- Format: Config + 1
- Common Mode: Config + 2
- R Divider: 0x024A + (N * 3)

## Troubleshooting

### Common Issues

1. **Device not detected**
   - Check I2C address (default 0x74)
   - Verify I2C bus speed (max 400kHz)
   - Check power supplies (VDD, VDDA, VDDS)

2. **Clock output not working**
   - Verify PLL lock status in STATUS register
   - Check output enable bits
   - Verify R-divider settings

3. **Wrong frequency**
   - Check PLL M divider settings
   - Verify input clock frequency
   - Check output R-divider value

### Debug Messages

Enable dynamic debug for detailed driver messages:

```bash
echo "file clk-si5391a.c +p" > /sys/kernel/debug/dynamic_debug/control
```

View kernel log:
```bash
dmesg | grep si5391a
```

## Technical Specifications

### Electrical Characteristics
- Input voltage: 1.8V or 3.3V (configurable)
- I2C interface: Standard (100kHz) or Fast (400kHz)
- Output formats: LVDS, LVPECL, HCSL, LVCMOS
- Jitter: < 0.3ps RMS (12kHz to 20MHz)

### Frequency Ranges
- Input: 8MHz to 750MHz
- VCO: 13.5GHz to 14.256GHz
- Output: 100Hz to 712.5MHz

## Related Documentation

- [Si5391A Datasheet](https://www.skyworks.com/en/products/timing/clocks-and-synthesizers/si5391a)
- [Si5391A Reference Manual](https://www.skyworks.com/en/products/timing/clocks-and-synthesizers/si5391a)
- [Linux Clock Framework Documentation](https://www.kernel.org/doc/html/latest/driver-api/clk.html)
- [Device Tree Clock Bindings](https://www.kernel.org/doc/Documentation/devicetree/bindings/clock/clock-bindings.txt)

## License

This driver is licensed under GPL v2. See the source file header for details.

## Author

Based on the Si5341 driver by Mike Looijmans <mike.looijmans@topic.nl>
Adapted for Si5391A by Linux kernel developers.

## Version History

- v1.0 - Initial release based on Si5341 driver structure
  - Basic clock output control
  - Device Tree support
  - I2C register access
  - PLL configuration