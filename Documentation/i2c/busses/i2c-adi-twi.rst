       Analog Devices TWI adapter driver

Copyright (C) 2014 - 2018 Analog Devices Inc.

This document describes the driver for the on-chip TWI (I2C) controllers (Analog Devices IP blocks).

1) Kernel Configuration
The kernel configuration option is CONFIG_I2C_ADI_TWI:
 Device Drivers ---> I2C support ---> I2C Hardware Bus support ---> ADI TWI I2C support

CONFIG_I2C_ADI_TWI_CLK_KHZ: is to set serial clock frequencies (SCL), which can vary from 21kHz to 400kHz.

In situations where a user wants simple means to send and receive I2C messages, the i2c-dev driver can be used. i2c-dev
provides a userspace accessible means to communicate with the I2C interface. To enable the i2c-dev driver (the kernel
configuration option is CONFIG_I2C_CHARDEV):
 Device Drivers ---> I2C support ---> I2C device interface

2) Driver parameters list
 This driver does not take in any parameters.

3) Command line options
 This driver does not take in any command-line options.

4) Driver information and notes

4.1) I2C introduction.
I2C is a bidirectional low-speed serial bus that provides a simple, efficient method of data exchange, minimizing the
interconnection between devices. Multiple slave devices may be accessed over the same bus, using a unique 7-bit
address for each slave. Communication on the bus is half-duplex, and slaves do not transmit any data unless a master
has addressed it first.

From the Linux OS point of view the I2C driver has two parts:
o Bus driver - low-level interface that is used to communicate with the I2C bus
o Chip driver - the interface between other device drivers and the I2C bus driver
The I2C bus driver is a low-level interface that is used to interface with the I2C bus. This driver is invoked by the
I2C chip driver, and it is not exposed to the user space. The Linux kernel contains a core I2C module that is used by
the chip driver to access the bus driver to transfer data over the I2C bus. 

This document focuses on the bus driver provided by Analog Devices, product developer needs to provide an implementation
of the chip driver to connect a specific I2C slave device to applications running under Linux.

4.2) I2C bus driver overview.
The I2C bus driver is invoked only by the chip driver and is not exposed to the user space. The Linux kernel contains
a core I2C module that is used by the chip driver to access the I2C bus driver to transfer data over the I2C bus. The
chip driver uses a standard kernel space API that is provided in the Linux kernel to access the core I2C module. The
standard I2C kernel functions are documented in the files available under Documentation/i2c in the kernel source tree.

4.3) Driver Features 
The I2C driver supports the following features: 
o Compatibility with the I2C bus standard
o Bit rates from 21Kbps to 400 Kbps
o Power management features by suspending and resuming I2C. 
o 7-bit addressing (note: 10-bit address mode is not supported due to some patent issue)
o I2C master mode of operation (note: driver does not support the slave mode)
o Interrupt-driven data transfer

4.4) I2C bus driver software operation
The I2C bus driver is described by a structure called i2c_adapter. The most important field in this structure is 
struct i2c_algorithm 
*algo . This field is a pointer to the i2c_algorithm structure that describes how data is transferred over the I2C
bus.*
The algorithm structure contains a pointer to a function that is called whenever the I2C chip driver wants to
communicate with an I2C device. During startup, the I2C bus adapter is registered with the I2C core when the driver is
loaded. Certain architectures have more than one I2C module. If so, the driver registers separate i2c_adapter
structures for each I2C module with the I2C core. These adapters are unregistered (removed) when the driver is
unloaded. During normal communication, it times out and returns an error when the transfer has some error condition,
such as NACK is detected. When an error condition occurs, the I2C driver should stop current transfer.

4.5) Device-tree support.
Please see the following document:
	Documentation/devicetree/bindings/i2c/adi,twi.yaml
