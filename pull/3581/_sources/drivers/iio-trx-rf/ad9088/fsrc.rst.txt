.. _ad9088 fsrc:

Fractional Sample Rate Converter
""""""""""""""""""""""""""""""""

This document describes the Fractional Sample Rate Converter (FSRC) support
for AD9084/AD9088 Apollo devices, including the FPGA AXI FSRC sequencer,
the device tree configuration, and the dynamic reconfiguration flows.

Overview
========

The FSRC changes the ratio between the JESD204 link sample rate and the
converter data path sample rate by a fractional factor N/M, without changing
the clocks. The JESD204 link keeps running at the profile lane rate, and the
rate change is carried by invalid samples (holes):

* **TX**: the FPGA inserts invalid samples in the stream it sends to Apollo,
  so only M of every N link samples are valid. The Apollo JRx rate match FIFO
  discards the invalid samples, and the Apollo TX FSRC interpolates the
  valid samples back to the full data path rate.
* **RX**: the Apollo RX FSRC decimates the data path rate by N/M and fills
  the link with invalid samples, sent as -FS. The FPGA removes them, so the
  DMA only receives valid samples.

For a ratio N/M, a fraction (N-M)/N of the link samples are invalid. For
example, 15625/12288 gives 21.36% invalid samples.

The supported ratios are 1 < N/M < 2. N = M selects the FSRC 1x mode, where
the FSRC passes the samples unchanged.

With the Apollo FSRC active, -FS data samples are mapped to -FS+1 before the
invalid sample insertion.

The FSRC is reconfigured at runtime, with the JESD204 links up, either over
SPI or with a SYSREF timed trigger driven by the FPGA sequencer, see
:ref:`ad9088 fsrc reconfig`.

System Architecture
===================

Data Paths
----------

::

    TX:
    +-----+    +-----------+  JESD204   +-------------+    +----------+    +-----+
    | DMA |--->| FPGA TX   |----------->| Apollo JRx  |--->| Apollo   |--->| DAC |
    |     |    | FSRC      | M valid of | rate match  |    | TX FSRC  |    |     |
    +-----+    | (holes)   | N samples  | FIFO (drops |    | (interp. |    +-----+
               +-----------+            | invalids)   |    |  M -> N) |
                                        +-------------+    +----------+

    RX:
    +-----+    +----------+    +-----------+  JESD204   +-----------+    +-----+
    | ADC |--->| Apollo   |--->| Apollo    |----------->| FPGA RX   |--->| DMA |
    |     |    | RX FSRC  |    | JTx       | -FS holes  | FSRC      |    |     |
    +-----+    | (N -> M) |    |           |            | (removes  |    +-----+
               +----------+    +-----------+            |  -FS)     |
                                                        +-----------+

The Apollo JRx rate match FIFO takes each converter clock worth of samples,
NS of them, as all valid or all invalid. The FPGA therefore inserts the
invalid samples in groups of NS samples, NS being the JRx samples per
converter clock (``ns_minus1 + 1`` of the JRx link).

HDL
---

The FPGA side is the ``axi_fsrc`` IP from the HDL ``library/axi_fsrc``:

* ``axi_fsrc_tx``: inserts the invalid samples in the TX stream, with a
  programmable N/M accumulator.
* ``axi_fsrc_rx``: removes the -FS invalid samples from the RX stream.
* ``axi_fsrc_sequencer``: counts SYSREF periods and drives the Apollo
  trigger pins and the TX data start, for SYSREF timed reconfigurations.

In the :git+hdl:`AD9084-EBZ <projects/ad9084_ebz>` reference design, the
FSRC is built with the ``FSRC_ENABLE=1`` project parameter. The sequencer
then drives the Apollo trigger pins (``trig_a``/``trig_b``) instead of the
AION trigger IP.

Linux Drivers
-------------

* :git+linux:`ad9088_fsrc.c <main:drivers/iio/trx-rf/ad9088/ad9088_fsrc.c>` -
  Apollo FSRC configuration and reconfiguration sequences.
* :git+linux:`axi_fsrc_sequencer.c <main:drivers/iio/jesd204/axi_fsrc_sequencer.c>` -
  FPGA AXI FSRC TX, RX and sequencer driver (``CONFIG_AXI_FSRC_SEQUENCER``).

The ``axi_fsrc_sequencer`` driver registers an IIO device, ``axi_fsrc``,
which the Apollo driver consumes as the ``fsrc`` IIO channel.

Device Tree
===========

AXI FSRC Sequencer
------------------

The sequencer node holds the sequencer registers, and links the TX and RX
FSRC cores through ``fsrc-topology``. Each core node sets its role with one
of ``adi,fsrc_tx``, ``adi,fsrc_rx``, ``adi,fsrc_tx_a`` or ``adi,fsrc_rx_a``.

.. code:: dts

   axi_fsrc_sequencer: axi-fsrc-sequencer@a4540000 {
       compatible = "adi,axi-fsrc-sequencer";
       reg = <0xa4540000 0x10000>;

       fsrc-topology = <&axi_fsrc_tx &axi_fsrc_rx>;
       #io-channel-cells = <1>;
   };

   axi_fsrc_tx: axi-fsrc-sequencer@a4510000 {
       reg = <0xa4510000 0x10000>;
       adi,fsrc_tx;
   };

   axi_fsrc_rx: axi-fsrc-sequencer@a4500000 {
       reg = <0xa4500000 0x10000>;
       adi,fsrc_rx;
   };

* ``fsrc-topology``: phandles to the FSRC cores. Each role can only be
  assigned once.

Apollo
------

The ``fsrc`` IIO channel enables the FSRC support in the Apollo driver:

.. code:: dts

   trx0_ad9084: ad9084@0 {
       compatible = "adi,ad9084";
       /* ... other properties ... */

       /* SYSREF timed reconfiguration, see below */
       adi,fsrc-gpio-trig;

       io-channels = <&adf4030 5>, <&adf4382 0>, <&axi_fsrc_sequencer 0>;
       io-channel-names = "bsync", "clk", "fsrc";
   };

* ``io-channels``/``io-channel-names``: with an ``fsrc`` channel, the driver
  keeps the Apollo FSRC in the data path. Without it, the profile is used
  as is, and the FSRC configure and reconfig debugfs attributes return
  -ENODEV.
* ``adi,fsrc-gpio-trig``: optional, asserts that the FPGA sequencer trigger
  output is routed to the Apollo trigger pin A0, and selects the SYSREF timed
  TX reconfiguration. Without it, the reconfigurations are done over SPI.

With the ``fsrc`` channel present, the driver modifies the device profile
before loading it:

* RX and TX FSRC not bypassed, ``enable0`` and ``enable1`` set, in 1x mode.
* ``split_4t4r`` set on 4T4R profiles, so the second stream of each side
  (FSRC0_1) is enabled.
* FSRC gain reduction at ``0xFFF``, profiles without FSRC may set it at 0.
* RX rate match FIFO invalid samples enabled, sample repeat and DDC dither
  disabled.

So the device boots at 1:1 with the FSRC in the path, ready for a runtime
reconfiguration.

On the VCK190 and VCU118 device trees, the FSRC nodes and the ``fsrc``
channel are guarded by the ``WITH_FSRC`` macro. Since the sequencer drives
the trigger pins, the VCU118 device tree also leaves out the AION trigger
with ``WITH_FSRC``, and the VCK190 device tree builds without it
(``WITH_AXI_AION_TRIG`` at 0).

AXI FSRC IIO Device
===================

The ``axi_fsrc`` device exposes one output channel with the FPGA FSRC
controls:

.. shell::

   /sys/bus/iio/devices/iio:device4
   $ cat name
    axi_fsrc
   $ ls
    name                          out_altvoltage0_tx_active
    of_node                       out_altvoltage0_tx_enable
    out_altvoltage0_rx_enable     out_altvoltage0_tx_ratio_set
    out_altvoltage0_seq_start     ...

- ``out_altvoltage0_rx_enable``: Enable the FPGA RX FSRC, which removes the
  -FS invalid samples.

  | ``0``: Disable.
  | ``1``: Enable.

- ``out_altvoltage0_tx_enable``: Enable the FPGA TX FSRC. Disabling it also
  clears ``tx_active``.

  | ``0``: Disable.
  | ``1``: Enable.

- ``out_altvoltage0_tx_active``: Start or stop the TX data. When stopped, the
  FPGA sends only invalid samples. Requires ``tx_enable``, else -EINVAL.

  | ``0``: Stop, send only invalid samples.
  | ``1``: Start sending valid samples at the configured ratio.

- ``out_altvoltage0_tx_ratio_set``: Program the TX hole pattern as
  ``<N> <M> [NS]``, with NS the invalid sample group size (default 1). Reads
  back the last values.

  | Range: ``M <= N < 2M``
  | NS: a divisor of the core samples per beat, or a multiple of it spanning
    at most 256 beats.

  The new hole pattern takes effect at once, while the FPGA TX is active.

  .. shell::
     :no-path:

     $ cat out_altvoltage0_tx_ratio_set
      1 1 4

- ``out_altvoltage0_seq_start``: Write ``1`` to arm the TX FSRC on the
  sequencer data start, reseed the hole pattern, and start the sequencer.

The Apollo driver drives these attributes during the reconfiguration
sequences; they are only needed directly to test the FPGA side.

Sequencer Timing
----------------

All sequencer counts are in SYSREF periods from the sequencer start, 4 bits
each:

* Apollo trigger: at count 2.
* TX data start: at count 4, the FPGA TX FSRC starts sending valid samples.

Register Access
---------------

The ``direct_reg_access`` debugfs attribute reads and writes the core
registers, with the core selected by bits [23:16] of the address: ``0``
sequencer, ``1`` RX, ``2`` TX, ``3`` RX A, ``4`` TX A. A core that is not in
the topology returns -ENODEV, an index past the last core -EINVAL.

.. shell::

   /sys/kernel/debug/iio/iio:device4
   # Read the TX FSRC enable register
   $ echo 0x20010 > direct_reg_access
   $ cat direct_reg_access
    0x1

Apollo Debugfs Attributes
=========================

The FSRC is configured through the Apollo device debugfs:

.. shell::

   /sys/kernel/debug/iio/iio:device8
   $ ls | grep fsrc
    fsrc_inspect
    fsrc_rx_configure
    fsrc_rx_reconfig
    fsrc_tx_configure
    fsrc_tx_reconfig

- ``fsrc_tx_configure``: Set the TX ratio as ``<N> <M>``. Programs the FPGA
  TX hole pattern, with NS from the JRx link, which takes effect at once, the
  JRx invalid sample handling, and the Apollo TX FSRC rate, or the 1x mode
  for N = M, which takes effect at the next ``fsrc_tx_reconfig``.
- ``fsrc_rx_configure``: Set the RX ratio as ``<N> <M>``. Programs the
  Apollo RX FSRC rate, or the 1x mode for N = M, which takes effect at the
  next ``fsrc_rx_reconfig``.
- ``fsrc_tx_reconfig``: Write ``1`` to apply the TX configuration, see
  :ref:`ad9088 fsrc reconfig`.
- ``fsrc_rx_reconfig``: Write ``1`` to apply the RX configuration.
- ``fsrc_inspect``: Read the configuration of all RX and TX FSRC blocks.

The configure attributes require two arguments, else -EINVAL. The configure
and reconfig attributes require the ``fsrc`` IIO channel, else -ENODEV. The
Apollo API rejects ratios outside 1 < N/M < 2.

The FSRC rate is ``fsrc_rate_int + fsrc_rate_frac_a / fsrc_rate_frac_b``
= 2^48 M/N, and the gain reduction is floor(4096 M/N):

.. shell::
   :no-path:

   $ echo "15625 12288" > fsrc_tx_configure
   $ echo 1 > fsrc_tx_reconfig
   $ cat fsrc_inspect
    Rx FSRC:
      A0:
        enable0:          1
    ...
    Tx FSRC:
      A0:
        enable0:          1
        enable1:          0
        mode_1x:          0
        split_4t4r:       1
        fsrc_bypass:      0
        fsrc_rate_int:    0xc9539b888722
        fsrc_rate_frac_a: 0x25ce
        fsrc_rate_frac_b: 0x3d09
        gain_reduction:   0xc95
        fsrc_delay:       0x0
    ...
      FSRC ratio (N/M) = 2^48 / (fsrc_rate_int + fsrc_rate_frac_a/fsrc_rate_frac_b)

At 1:1, the blocks report ``mode_1x: 1`` and ``gain_reduction: 0xfff``.

.. _ad9088 fsrc reconfig:

Dynamic Reconfiguration
=======================

A reconfiguration is set with the configure attributes, then applied with
the reconfig attributes, TX and RX independently. Write the reconfig right
after the configure: the FPGA TX hole pattern changes at the configure, the
Apollo TX FSRC only at the reconfig, and the TX data is not valid in
between.

.. shell::
   :no-path:

   # TX and RX at 15625/12288
   $ echo "15625 12288" > fsrc_tx_configure
   $ echo "15625 12288" > fsrc_rx_configure
   $ echo 1 > fsrc_tx_reconfig
   $ echo 1 > fsrc_rx_reconfig

   # Back to 1:1
   $ echo "1 1" > fsrc_tx_configure
   $ echo "1 1" > fsrc_rx_configure
   $ echo 1 > fsrc_tx_reconfig
   $ echo 1 > fsrc_rx_reconfig

The DMA sample rate follows the ratio: with the TX at N/M, a DMA tone of
frequency f leaves the DAC at f M/N relative to the DAC NCO; with the RX at
N/M, the captured tone scales by N/M.

TX over SPI
-----------

Without ``adi,fsrc-gpio-trig``, ``fsrc_tx_reconfig``:

#. Enables the FPGA TX FSRC and waits 10 ms.
#. Enables the Apollo DSP reset on trigger, and runs the manual dynamic
   reconfiguration sync, which applies the new configuration.
#. Disarms the trigger resets.
#. Starts the FPGA TX data at the new ratio.
#. After 100 us, resets the JRx rate match FIFOs of links A0 and B0.

The new rate is applied at an SPI write, so the FPGA hole pattern and the
Apollo FSRC start at an arbitrary time relative to each other and to SYSREF.

TX with the SYSREF Timed Trigger
--------------------------------

With ``adi,fsrc-gpio-trig``, ``fsrc_tx_reconfig``:

#. Enables the FPGA TX FSRC and stops the TX data: the FPGA sends only
   invalid samples.
#. Forces invalid samples on the Apollo JTx links, and waits 1 s for the
   invalid samples to flow through the system.
#. Resets the JRx rate match FIFOs of links A0 and B0, while only invalid
   samples arrive.
#. Maps the trigger pin A0 to all RX and TX blocks, enables the DSP reset on
   trigger, and arms the trigger sync.
#. Starts the FPGA sequencer: at the trigger count it triggers the Apollo
   reconfiguration, and at the data start count the FPGA TX starts sending
   valid samples.
#. Clears the trigger sync and disarms the trigger resets.

The JRx rate match FIFOs start empty, and the Apollo reconfiguration and
the FPGA data start happen at fixed SYSREF counts from the sequencer start,
so the timing repeats from one reconfiguration to the next. There is no
rate match FIFO reset after the trigger, which would be at an arbitrary
time. The JTx forced invalid samples clear when the reconfiguration
completes. The sequence takes about 1.1 s.

RX
--

``fsrc_rx_reconfig`` always reconfigures over SPI:

#. Enables the FPGA RX FSRC, which removes the -FS invalid samples.
#. Enables the Apollo DSP reset on trigger, and runs the manual dynamic
   reconfiguration sync.
#. Disarms the trigger resets.

The RX needs no alignment with the FPGA, which deletes the invalid samples
wherever they are. Running the sequencer here would also restart the TX hole
pattern. The JRx rate match FIFOs are on the TX path, so the RX
reconfiguration does not reset them.
