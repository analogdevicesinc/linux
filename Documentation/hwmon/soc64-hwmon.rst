.. SPDX-License-Identifier: GPL-2.0
Kernel driver soc64-hwmon
=========================

Supported chips:

 * Intel N5X

Author: Kris Chaplin <kris.chaplin@intel.com>

Description
-----------

This driver supports hardware monitoring for 64-Bit SoC FPGA and eASIC devices
based around the Secure Device Manager and Stratix 10 Service layer.

The following sensor types are supported

  * temperature
  * voltage


Usage Notes
-----------

The driver relies on a device tree node to enumerate support present on the
specific device. See Documentation/devicetree/bindings/hwmon/soc64-hwmon.txt
for details of the device-tree node.
