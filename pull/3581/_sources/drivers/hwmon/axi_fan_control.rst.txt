.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/hwmon/axi_fan_control

.. _axi_fan_control:

AXI Fan Control
===============

AXI Fan Control HDL CORE Linux driver.

Supported Devices
-----------------

:external+hdl:ref:`axi_fan_control`

Supported boards
~~~~~~~~~~~~~~~~

:dokuwiki:`ADRV9009-ZU11EG RF System-on-Module </resources/eval/user-guides/adrv9009-zu11eg?s[]=zu11eg>`

Description
^^^^^^^^^^^

This is a Hardware Monitor subsystem driver. The Hwmon subsystem provides a
unified framework for drivers doing hardware monitoring. It exposes a set of
sysfs files that should follow the
`sysfs-interface <https://www.kernel.org/doc/Documentation/hwmon/sysfs-interface>`__.

Source Code
-----------

Status
~~~~~~

.. list-table::
   :header-rows: 1

   * - Source
     - Mainlined?
     -
     -
   * - :git+linux:`main:drivers/hwmon/axi_fan_control.c`
     - `WIP <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/hwmon/axi_fan_control.c>`__
     -
     -

Files
~~~~~

.. list-table::
   :header-rows: 1

   * - Function
     - File
     -
   * - driver
     - :git+linux:`main:drivers/hwmon/axi_fan_control.c`
     -

Example platform device initialization
--------------------------------------

The AXI FAN CONTROL driver is a platform driver and can currently only be
instantiated via device tree.

Required devicetree properties:

- **compatible**: Should always be one of these:

  - adi,axi-fan-control-1.00.a

- **reg**: Base address and register area size. This parameter expects a
  register range.
- **clocks**: Reference clock.
- **interrupts**: Interrupt line.
- **adi,pulses-per-revolution**: Number of pulses per revolution

::

   &fpga_axi {
       axi_fan_control: axi-fan-control@80000000 {
           compatible = "adi,axi-fan-control-1.00.a";
           reg = <0x0 0x80000000 0x10000>;
           clocks = <&clk 71>;
           interrupts = <0 110 0>;
           adi,pulses-per-revolution = <2>;
   };

Enabling Linux driver support
-----------------------------

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

Adding Linux driver support
---------------------------

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

::

   Linux Kernel Configuration
       Device Drivers  --->
           <*>     Hardware Monitoring support --->
               --- Hardware Monitoring support
           [--snip--]
               <*> Analog Devices FAN Control HDL Core driver
           [--snip--]

Hardware configuration
----------------------

Driver testing
--------------

HWMON device files
~~~~~~~~~~~~~~~~~~

Each and every HWMON device has a device folder under /sys/class/hwmon/hwmonX,
where X is the hwmon device index. Under every of these directory folders there
are a set of files, depending on what kind of hardware monitoring is supported
(temperature, voltage). In order to determine which hwmon device corresponds to
which hardware chip, the user can read the name file under
/sys/class/hwmon/hwmonX/name.

::

   root:/> cd /sys/class/hwmon/
   root:/sys/class/hwmon> ls
   hwmon0  hwmon1
   root:/sys/class/hwmon> cd hwmon0
   root:/sys/class/hwmon/hwmon0> ls -l
   total 0
   drwxr-xr-x 3 root root    0 Nov 16 11:17 .
   drwxr-xr-x 3 root root    0 Nov 16 11:17 ..
   lrwxrwxrwx 1 root root    0 Nov 16 11:17 device -> ../../../axi-fan-control@80000000
   -r--r--r-- 1 root root 4096 Nov 16 11:17 fan1_fault
   -r--r--r-- 1 root root 4096 Nov 16 11:17 fan1_input
   -r--r--r-- 1 root root 4096 Nov 16 11:17 fan1_label
   -r--r--r-- 1 root root 4096 Nov 16 11:17 name
   lrwxrwxrwx 1 root root    0 Nov 16 11:17 of_node -> ../../../../../firmware/devicetree/base/axi-fan-control@80000000
   drwxr-xr-x 2 root root    0 Nov 16 11:17 power
   -rw-r--r-- 1 root root 4096 Nov 16 11:17 pwm1
   lrwxrwxrwx 1 root root    0 Nov 16 11:17 subsystem -> ../../../../../class/hwmon
   -r--r--r-- 1 root root 4096 Nov 16 11:17 temp1_input
   -r--r--r-- 1 root root 4096 Nov 16 11:17 temp1_label
   -rw-r--r-- 1 root root 4096 Nov 16 11:17 uevent
   root:/sys/class/hwmon/hwmon0>

For more information on the files format and accepted values, take a look to
`sysfs-interface <https://www.kernel.org/doc/Documentation/hwmon/sysfs-interface>`__.

Show device name
^^^^^^^^^^^^^^^^

::

   root:/sys/class/hwmon/hwmon0> cat name
   axi_fan_control

Show FAN input (in RPM)
^^^^^^^^^^^^^^^^^^^^^^^

::

   root:/sys/class/hwmon/hwmon0> cat fan1_input
   3600

Show FAN label
^^^^^^^^^^^^^^

::

   root:/sys/class/hwmon/hwmon0> cat fan1_label
   SOM FAN

Show PWM input
^^^^^^^^^^^^^^

::

   root:/sys/class/hwmon/hwmon0> cat pwm1
   200

Show Temperature input (in milliC)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

   root:/sys/class/hwmon/hwmon0> cat temp1_input
   32000

Show Temperature label
^^^^^^^^^^^^^^^^^^^^^^

::

   root:/sys/class/hwmon/hwmon0> cat temp1_label
   SYSMON4

Show fault
^^^^^^^^^^

When reading **fan1_fault** returns 1, that means that either the FAN is
physically not working/broken or the tacho signal fails to stay within its
designated frequency interval.

::

   root:/sys/class/hwmon/hwmon0> cat fan1_fault
   0

Set PWM value
^^^^^^^^^^^^^

::

   root:/sys/class/hwmon/hwmon0> echo 250 > pwm1

More Information
----------------

HWMON pointers
--------------

- HWMON mailing list: linux-hwmon@vger.kernel.org
- `HWMON Linux Kernel Documentation <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/Documentation/hwmon>`__
- `HWMON sysfs-interface <https://www.kernel.org/doc/Documentation/hwmon/sysfs-interface>`__

.. include:: /include/need-help.rst
   :start-after: .. start-need-help
   :end-before: .. end-need-help
