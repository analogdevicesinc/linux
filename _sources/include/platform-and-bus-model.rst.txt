:orphan:

Platform Data
=============

.. start-platform-data

For compile time configuration, it's common Linux practice to keep board- and
application-specific configuration out of the main driver file, instead putting
it into the board support file.

For devices on custom boards, as typical of embedded and SoC-(system-on-chip)
based hardware, Linux uses ``platform_data`` to point to board-specific structures
describing devices and how they are connected to the SoC. This can include
available ports, chip variants, preferred modes, default initialization,
additional pin roles, and so on. This shrinks the board-support packages (BSPs)
and minimizes board and application specific #ifdefs in drivers.

.. end-platform-data

Platform Data Example
=====================

The following example shows a set of these configuration options. These variables
are fully documented in adxl34x header file (:git+linux:`main:include/linux/input/adxl34x.h`).

.. code-block:: c

   #include <linux/input/adxl34x.h>
   static const struct adxl34x_platform_data adxl34x_info = {
       .x_axis_offset = 0,
       .y_axis_offset = 0,
       .z_axis_offset = 0,
       .tap_threshold = 0x31,
       .tap_duration = 0x10,
       .tap_latency = 0x60,
       .tap_window = 0xF0,
       .tap_axis_control = ADXL_TAP_X_EN | ADXL_TAP_Y_EN | ADXL_TAP_Z_EN,
       .act_axis_control = 0xFF,
       .activity_threshold = 5,
       .inactivity_threshold = 3,
       .inactivity_time = 4,
       .free_fall_threshold = 0x7,
       .free_fall_time = 0x20,
       .data_rate = 0x8,
       .data_range = ADXL_FULL_RES,

       .ev_type = EV_ABS,
       .ev_code_x = ABS_X,     /* EV_REL */
       .ev_code_y = ABS_Y,     /* EV_REL */
       .ev_code_z = ABS_Z,     /* EV_REL */

       .ev_code_tap = {BTN_TOUCH, BTN_TOUCH, BTN_TOUCH}, /* EV_KEY x,y,z */

       .ev_code_ff = KEY_F,                /* EV_KEY */
       .ev_code_act_inactivity = KEY_A,    /* EV_KEY */
       .power_mode = ADXL_AUTO_SLEEP | ADXL_LINK,
       .fifo_mode = ADXL_FIFO_STREAM,
   };

Platform and Bus Model
======================

To attach devices to drivers, the platform and bus model eliminates the need
for device drivers to contain hard coded physical addresses or bus IDs of the
devices they control. The platform and bus model also prevents resource
conflicts, greatly improves portability, and cleanly interfaces with the
kernel's power-management features.

With the platform and bus model, device drivers know how to control a device
once informed of its physical location and interrupt lines. This information is
provided as a data structure passed to the driver during probing.

Declaring I2C devices
=====================

.. start-declaring-i2c-devices

Unlike PCI or USB devices, I2C devices are not enumerated at the hardware level.
Instead, the software must know which devices are connected on each I2C bus
segment, and what address these devices are using. For this reason, the kernel
code must instantiate I2C devices explicitly. There are different ways to
achieve this, depending on the context and requirements. However the most
common method is to declare the I2C devices by bus number.

This method is appropriate when the I2C bus is a system bus, as in many
embedded systems, wherein each I2C bus has a number which is known in advance.
It is thus possible to pre-declare the I2C devices that inhabit this bus. This
is done with an array of ``struct i2c_board_info``, which is registered by
calling ``i2c_register_board_info()``.

So, to enable such a driver one need only edit the board support file by adding
an appropriate entry to ``i2c_board_info``.

For more information see: :external+upstream:doc:`i2c/instantiating-devices`

.. end-declaring-i2c-devices

Example Declaring I2C Devices
=============================

.. code-block:: c

   static struct i2c_board_info __initdata bfin_i2c_board_info[] = {
   #if defined(CONFIG_TOUCHSCREEN_AD7879_I2C) || defined(CONFIG_TOUCHSCREEN_AD7879_I2C_MODULE)
       {
           I2C_BOARD_INFO("ad7879", 0x2F),
           .irq = IRQ_PG5,
           .platform_data = (void *)&bfin_ad7879_ts_info,
       },
   #endif
   #if defined(CONFIG_KEYBOARD_ADP5588) || defined(CONFIG_KEYBOARD_ADP5588_MODULE)
       {
           I2C_BOARD_INFO("adp5588-keys", 0x34),
           .irq = IRQ_PG0,
           .platform_data = (void *)&adp5588_kpad_data,
       },
   #endif
   #if defined(CONFIG_PMIC_ADP5520) || defined(CONFIG_PMIC_ADP5520_MODULE)
       {
           I2C_BOARD_INFO("pmic-adp5520", 0x32),
           .irq = IRQ_PG0,
           .platform_data = (void *)&adp5520_pdev_data,
       },
   #endif
   #if defined(CONFIG_INPUT_ADXL34X_I2C) || defined(CONFIG_INPUT_ADXL34X_I2C_MODULE)
       {
           I2C_BOARD_INFO("adxl34x", 0x53),
           .irq = IRQ_PG0,
           .platform_data = (void *)&adxl34x_info,
       },
   #endif
   };

.. code-block:: c

   static void __init blackfin_init(void)
   {
       (...)
       i2c_register_board_info(0, bfin_i2c_board_info, ARRAY_SIZE(bfin_i2c_board_info));
       (...)
   }

Declaring SPI slave devices
===========================

.. start-declaring-spi-slave-devices

Unlike PCI or USB devices, SPI devices are not enumerated at the hardware level.
Instead, the software must know which devices are connected on each SPI bus
segment, and what slave selects these devices are using. For this reason, the
kernel code must instantiate SPI devices explicitly. The most common method is
to declare the SPI devices by bus number.

This method is appropriate when the SPI bus is a system bus, as in many
embedded systems, wherein each SPI bus has a number which is known in advance.
It is thus possible to pre-declare the SPI devices that inhabit this bus. This
is done with an array of ``struct spi_board_info``, which is registered by
calling ``spi_register_board_info()``.

For more information see: :external+upstream:doc:`spi/spi-summary`

.. end-declaring-spi-slave-devices

Example Declaring SPI Slave Devices
===================================

So, to enable such a driver one need only edit the board support file by adding
an appropriate entry to ``spi_board_info``.

.. code-block:: c

   static struct spi_board_info bfin_spi_board_info[] __initdata = {
   #if defined(CONFIG_TOUCHSCREEN_AD7877) || defined(CONFIG_TOUCHSCREEN_AD7877_MODULE)
       {
           .modalias           = "ad7877",
           .platform_data      = &bfin_ad7877_ts_info,
           .irq                = IRQ_PF6,
           .max_speed_hz       = 12500000,     /* max spi clock (SCK) speed in HZ */
           .bus_num            = 0,
           .chip_select        = 1,
           .controller_data    = &spi_ad7877_chip_info,
       },
   #endif
   #if defined(CONFIG_TOUCHSCREEN_AD7879_SPI) || defined(CONFIG_TOUCHSCREEN_AD7879_SPI_MODULE)
       {
           .modalias = "ad7879",
           .platform_data = &bfin_ad7879_ts_info,
           .irq = IRQ_PF7,
           .max_speed_hz = 5000000,     /* max spi clock (SCK) speed in HZ */
           .bus_num = 0,
           .chip_select = 1,
           .controller_data = &spi_ad7879_chip_info,
           .mode = SPI_CPHA | SPI_CPOL,
       },
   #endif
   #if defined(CONFIG_INPUT_ADXL34X_SPI) || defined(CONFIG_INPUT_ADXL34X_SPI_MODULE)
       {
           .modalias       = "adxl34x",
           .platform_data  = &adxl34x_info,
           .irq            = IRQ_PF6,
           .max_speed_hz   = 5000000,    /* max spi clock (SCK) speed in HZ */
           .bus_num        = 0,
           .chip_select    = 2,
           .controller_data = &spi_adxl34x_chip_info,
           .mode = SPI_MODE_3,
       },
   #endif
   };

.. code-block:: c

   static void __init blackfin_init(void)
   {
       (...)
       spi_register_board_info(bfin_spi_board_info, ARRAY_SIZE(bfin_spi_board_info));
       (...)
   }
