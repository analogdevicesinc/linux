.. _ad9088 gpio:

GPIO usage
""""""""""

This document describes the General Purpose Input Output (GPIO) architecture
for AD9084/AD9088 Apollo devices, including driver interactions, signals,
and flows.

The device contains 30 general-purpose GPIOs that can be used to trigger
various changes.

.. important::

   GPIOs serve multiple roles, and mapping them properly can be tricky.
   You can inspect the GPIO consumers with:

   .. shell::
      :no-path:

      $ gpioinfo -c /dev/gpiochip0
        [ ... ]
        line  30:       unnamed                 output consumer=reset
        line  31:       unnamed                 input
        line  32:       unnamed                 input consumer=reset-done
        line  33:       unnamed                 input consumer=reset-done
        line  34:       unnamed                 output
        line  35:       unnamed                 input
        line  36:       unnamed                 output consumer=pll-datapath-reset
        line  37:       unnamed                 output consumer=pll-datapath-reset
        line  38:       unnamed                 output consumer=datapath-reset
        line  39:       unnamed                 output consumer=datapath-reset
        [ ... ]

Each reference design has distinct routes that need to be considered when
interfacing with the device GPIOs. This section uses the
:git+hdl:`AD9084-EBZ/VCK190 <projects/ad9084_ebz/vck190>` reference
design as an example.

.. _ad9088 gpio ffh:

GPIO Fast Frequency Hopping
===========================

The relevant devicetree options for FFH with GPIOs are:

* ``adi,gpio-hop-terminal``: sets the pins for ``profile_tx_rxn[1:0]``
* ``adi,gpio-hop-slice``: sets the pins for ``profile_txrx_slice[3:0]``
* ``adi,gpio-hop-side``: sets the pin for ``profile_txrx_BA``
* ``adi,gpio-hop-block``: sets the pins for ``profile_fcn_sel[3:0]``
* ``adi,gpio-hop-profile``: sets the pins for ``profile[4:0]``

These override the quick configuration profile pin assignments.

In particular, for Profile 1, ``profile_fcn_sel[3]`` lands on GPIO31.
However, the AD9084-EBZ/VCK190 only routes AD9084 GPIO15..GPIO30,
so the hop block selector cannot be fully driven with the default
Profile 1. Instead, modify ``profile_fcn_sel[3]`` to GPIO 15:

.. code:: devicetree

   trx0_ad9084: ad9084@0 {
      // ...
      adi,gpio-quick-config = <ADI_APOLLO_QUICK_CFG_PROFILE_1>;

      /* GPIO frequency hopping configuration (axi_gpio offset +15) */
      adi,gpio-hop-terminal = <17 18>;         // profile_tx_rxn[1:0]
      adi,gpio-hop-slice = <19 20 21>;         // profile_txrx_slice[2:0]
      adi,gpio-hop-side = <22>;                // profile_txrx_BA
      adi,gpio-hop-block = <23 24 25 15>;      // profile_fcn_sel[3:0]
      adi,gpio-hop-profile = <26 27 28 29 30>; // profile[4:0]
   }

With this devicetree configuration, and for TX side A, slice 0, FNCO block, and
profile selected by GPIOs, the feature can be tested as described below:

Set ``ADI_APOLLO_NCO_CHAN_SEL_DIRECT_GPIO``:

.. shell::

   /sys/bus/iio/devices/iio:device8
   $ echo 3 > out_voltage0_i_ffh_fnco_mode

Set the profile frequencies:

.. shell::

   /sys/bus/iio/devices/iio:device8
   $ echo 0 > out_voltage0_i_ffh_fnco_index
   $ echo $((16#20000000)) > out_voltage0_i_ffh_fnco_frequency
   $ echo 1 > out_voltage0_i_ffh_fnco_index
   $ echo $((16#25000000)) > out_voltage0_i_ffh_fnco_frequency
   $ echo 2 > out_voltage0_i_ffh_fnco_index
   $ echo $((16#30000000)) > out_voltage0_i_ffh_fnco_frequency

Set the GPIO outputs to select the profile:

.. shell::
   :no-path:

   $ gpiodetect
     gpiochip0 [a4000000.gpio] (64 lines)
     gpiochip1 [versal_gpio] (58 lines)
     gpiochip2 [pmc_gpio] (116 lines)

   # AD9084 offset -15 (profile[4] 26 -> 11)
   $ ADDR="0=0 2=0 3=0 4=0 5=0 6=0 7=0 8=0 9=0 10=0"
   # Profile 0
   $ gpioset -c /dev/gpiochip0 $ADDR 11=0 12=0 13=0 14=0 15=0

   # Profile 1
   $ gpioset -c /dev/gpiochip0 $ADDR 11=1 12=0 13=0 14=0 15=0

   # Profile 2
   $ gpioset -c /dev/gpiochip0 $ADDR 11=0 12=1 13=0 14=0 15=0

The mapping is:

===========  ======  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==
AD9084 GPIO    15    16  17  18  19  20  21  22  23  24  25  26  27  28  29  30
===========  ======  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==
GPIOCHIP0       0     1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
-----------  ------  --  --  --  --  --  --  --  --  --  --  --  --  --  --  --
Function     **f3**   u  t0  t1  s0  s1  s2  BA  f0  f1  f2  p0  p1  p2  p3  p4
===========  ======  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==  ==

Where ``f=profile_fcn_sel``, ``u=unused``, ``t=profile_tx_rxn``,
``s=profile_txrx_slice``, ``BA=profile_txrx_BA``, and
``p=profile``.

For the RX side, replace the ``out_`` attributes with ``in_``.

To configure and use the CNCO instead of FNCO, replace ``_fnco_`` with
``_cnco_`` in the properties and set ``8=1`` for
``profile_fcn_sel[3:0]=ADI_APOLLO_GPIO_BLOCK_CNCO``.

GPIO chip
=========

The GPIOs can be exported into a :external+upstream:c:struct:`gpio_chip`
using the ``adi,gpio-exports`` devicetree property:

.. code:: dts

   trx0_ad9084: ad9084@0 {
        adi,gpio-exports = /bits/ 8 <15 16 17 18>;
   };

Some of the device GPIOs are mapped in the HDL design to an axi_gpio IP core, and
the Apollo and axi_gpio GPIOs can be attached to other drivers in the device tree:

.. tip::

   ``0`` is the index of the item in ``adi,gpio-exports`` and, in the previous
   example, is equivalent to the device's GPIO 15. Also check the schematics and
   HDL design for the routes between Apollo GPIOs, axi_gpio GPIOs, and other
   input options.

.. code:: dts

   my_driver {
        /* ... */
        provider-gpios = <&axi_gpio 0 GPIO_ACTIVE_HIGH>;
        consumer-gpios = <&tx0_ad9084 0 GPIO_ACTIVE_HIGH>;
   }

Or from user space through `gpiod <https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/>`__:

.. shell::

   $ gpiodetect
     gpiochip0 [a4000000.gpio] (64 lines)
     gpiochip1 [versal_gpio] (58 lines)
     gpiochip2 [pmc_gpio] (116 lines)
     gpiochip3 [ad9088] (4 lines)
   $ gpioget -c /dev/gpiochip3 3
     0
   $ gpioset -c /dev/gpiochip0 3=1
   $ gpioget -c /dev/gpiochip3 3
     1
