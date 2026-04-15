.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/drm/hdl-axi-hdmi

.. _hdl-axi-hdmi:

AXI HDMI HDL
============

AXI HDMI HDL Linux Driver.

Supported Devices
-----------------

This driver supports the :external+hdl:ref:`adv7511`

Description
-----------

The AXI HDMI HDL driver is the driver for the HDL graphics core which is used on
various FPGA designs interfacing to the :ref:`adv7511`. The driver is
implemented as a DRM KMS driver.

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
   * - :git+linux:`drivers/gpu/drm/analog/analog_drm_drv.c <main:drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_drv.c>`
     - `WIP <https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/gpu/drm/adi/axi_hdmi_drv.c>`__
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
     - :git+linux:`drivers/gpu/drm/adi/axi_hdmi_drv.c <main:drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_drv.c>`
     -
   * - driver
     - :git+linux:`drivers/gpu/drm/adi/axi_hdmi_drv.h <main:drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_drv.h>`
     -
   * - driver
     - :git+linux:`drivers/gpu/drm/adi/axi_hdmi_crtc.c <main:drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_crtc.c>`
     -
   * - driver
     - :git+linux:`drivers/gpu/drm/adi/axi_hdmi_encoder.c <main:drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_encoder.c>`
     -

Example platform device initialization
--------------------------------------

The AXI HDMI driver is a platform driver and can currently only be instantiated
via device tree.

Required devicetree properties:

- compatible: Should always be ``adi,cf-adv7x11-core-1.00.a``
- reg: Base address and register area size. This parameter expects two register
  ranges. The first for the axi-hdmi core’s registers and the second for the
  axi-clockgen core’s registers.
- slave_adapter: Phandle to the I2C device on which the ADV7511 can be found
- dma-request: Phandle to the Xilinx VDMA device

Example:

::

   axi_iic_0: i2c@41600000 {
       compatible = "xlnx,axi-iic-1.01.b", "xlnx,xps-iic-2.00.a";
       interrupt-parent = <&gic>;
       interrupts = <0 56 0x4>;
       reg = <0x41600000 0x10000>;

       #size-cells = <0>;
       #address-cells = <1>;
   };

   axi_vdma_0: axivdma@43000000 {
       #address-cells = <1>;
       #size-cells = <1>;
       #dma-cells = <1>;
       compatible = "xlnx,axi-vdma";
       reg = <0x43000000 0x1000>;
       xlnx,include-sg = <0x0>;
       xlnx,num-fstores = <0x3>;
       dma-channel@7e200000 {
           compatible = "xlnx,axi-vdma-mm2s-channel";
           interrupts = <0 59 0x4>;
           xlnx,datawidth = <0x40>;
           xlnx,genlock-mode = <0x0>;
           xlnx,include-dre = <0x0>;
       };
   };

   axi-hdmi@6c000000 {
       compatible = "adi,cf-axi-hdmi-1.00.a";
       reg = <0x6c000000 0x10000
              0x66000000 0x10000>;
       slave_adapter = <&axi_iic_0>;
       dma-request = <&axi_vdma_0 0>;
   };

Enabling Linux driver support
-----------------------------

Configure kernel with ``make menuconfig`` (alternatively use ``make xconfig`` or
``make qconfig``)

.. note::

   The DRM AXI HDMI driver depends on **CONFIG_DRM**

::

   Linux Kernel Configuration
       Device Drivers  --->
           Graphics support  --->
               <*> Direct Rendering Manager (XFree86 4.1.0 and higher DRI support)  --->
               ...
               <*> DRM Support for Analog FPGA platforms

Driver testing
--------------

Using the framebuffer console
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The driver can be tested using the
`Linux framebuffer console <http://www.mjmwired.net/kernel/Documentation/fb/fbcon.txt>`__.

Enable framebuffer support in the kernel config:

::

   Linux Kernel Configuration
       Device Drivers  --->
           Graphics support  --->
               ...
                           Console display driver support  --->
                           <*> Framebuffer Console support
                           ...

Using Xorg
~~~~~~~~~~

The Xorg graphical environment can be used with driver either using
`xf86-video-fbdev <http://cgit.freedesktop.org/xorg/driver/xf86-video-fbdev/>`__
or the
`xf86-video-modesetting <http://cgit.freedesktop.org/xorg/driver/xf86-video-modesetting/>`__
Xorg module.

xf86-video-fbdev
^^^^^^^^^^^^^^^^

The fbdev driver will bind to the device by default, if no other driver has been
bound to the device. This usually means that running a Xorg server without any
additional configuration changes will come up with the fbdev driver.

xf86-video-modesetting
^^^^^^^^^^^^^^^^^^^^^^

The xf86-video-modesetting driver is a driver which has been written to take
advantage of the new Kernel Mode Setting (KMS) API of the DRM layer. This allows
to switch between different screen resolutions at runtime (using the Xservers
xrandr interface) and adds plug-and-play support for monitors.

To enable the modesetting driver the following section has to be added to the
Xorg config (/etc/X11/xorg.org).

::

   Section "Device"
     Identifier "ADV7511 HDMI"
     Driver "modesetting"
   EndSection

Others
~~~~~~

Since the driver registers a
`Linux framebuffer device <https://en.wikipedia.org/wiki/Linux_framebuffer>`__
it is possible to use any application or toolkit which has support for it to
display graphics on the HDMI output. This for example includes
`DirectFB <http://directfb.org/>`__, `mplayer <http://www.mplayerhq.hu/>`__ and
`SDL <http://www.libsdl.org/>`__.

More information
----------------

-
  :dokuwiki+deprecated:`AD-FMCOMMS1-EBZ Reference Design <resources/fpga/xilinx/fmc/ad-fmcomms1-ebz>`
- :ref:`adv7511`
- :external+documentation:ref:`linux-kernel zynq`

.. include:: /include/need-help.rst
   :start-after: .. start-need-help
   :end-before: .. end-need-help
