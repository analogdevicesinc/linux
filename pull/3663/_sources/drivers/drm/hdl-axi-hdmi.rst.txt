.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/drm/hdl-axi-hdmi


.. _hdl-axi-hdmi:

AXI HDMI HDL
============

AXI HDMI HDL Linux Driver.

Supported IP Cores
------------------

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
   * - :git+linux:`drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_drv.c <main:drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_drv.c>`
     - No
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
     - :git+linux:`drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_drv.c <main:drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_drv.c>`
     -
   * - driver
     - :git+linux:`drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_drv.h <main:drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_drv.h>`
     -
   * - driver
     - :git+linux:`drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_crtc.c <main:drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_crtc.c>`
     -
   * - driver
     - :git+linux:`drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_encoder.c <main:drivers/gpu/drm/adi_axi_hdmi/axi_hdmi_encoder.c>`
     -

Adding Linux driver support
---------------------------

Kernel configuration
~~~~~~~~~~~~~~~~~~~~

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
               <*> DRM Support for Analog Devices HDMI FPGA platforms

Devicetree configuration
~~~~~~~~~~~~~~~~~~~~~~~~

Example platform device initialization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The AXI HDMI driver is a platform driver and can currently only be instantiated
via device tree.

Required device-tree configuration:

- ``compatible = "adi,axi-hdmi-tx-1.00.a"``.
- ``reg``: One AXI HDMI register range. The clock generator is a separate
  clock provider, not a second register range.
- ``clocks``: The pixel-clock provider.
- ``dmas`` and ``dma-names = "video"``: The display DMA channel. Its controller
  must support interleaved transfers.
- An output graph endpoint connected to the ADV7511 input endpoint, or the
  ``encoder-slave`` phandle pointing to the ADV7511 I2C device node,
  not to its I2C bus.

The optional ``adi,is-rgb`` property selects RGB handling. Supply addresses,
clock and DMA references, interrupts and graph wiring from the actual board.

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

More Information
----------------

-
  :dokuwiki+deprecated:`AD-FMCOMMS1-EBZ Reference Design <resources/fpga/xilinx/fmc/ad-fmcomms1-ebz>`
- :ref:`adv7511`
- :external+system-level:ref:`linux-kernel zynq`

.. include:: /include/need-help.rst
   :start-after: .. start-need-help
   :end-before: .. end-need-help
