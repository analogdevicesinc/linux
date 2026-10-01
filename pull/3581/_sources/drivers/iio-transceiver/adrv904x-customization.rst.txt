.. imported from: https://wiki.analog.com/resources/tools-software/linux-drivers/iio-transceiver/adrv904x-customization

.. _adrv904x-customization:

ADRV904x Customization
======================

ADRV904x Device Driver Customization.

.. note::

   There are configuration options that must be set properly. Some others allow
   you to set defaults, but can be changed anytime later using the driver API.
   But most of these options don’t need to be changed at all.

   If unsure please see the manual or don’t change!

Clock Settings
--------------

:ref:`ad9528`

A sample configuration of the AD9528:

::

   clk0_ad9528: ad9528-1@1 {
       compatible = "adi,ad9528";
       reg = <1>;

       #address-cells = <1>;
       #size-cells = <0>;

       spi-max-frequency = <10000000>;
       //adi,spi-3wire-enable;

       clock-output-names = "ad9528-1_out0", "ad9528-1_out1", "ad9528-1_out2",
           "ad9528-1_out3", "ad9528-1_out4", "ad9528-1_out5", "ad9528-1_out6",
           "ad9528-1_out7", "ad9528-1_out8", "ad9528-1_out9", "ad9528-1_out10",
           "ad9528-1_out11", "ad9528-1_out12", "ad9528-1_out13";
       #clock-cells = <1>;

       jesd204-device;
       #jesd204-cells = <2>;
       jesd204-sysref-provider;

       adi,vcxo-freq = <122880000>;

       adi,refa-enable;

       adi,refa-diff-rcv-enable;
       adi,refa-r-div = <1>;

       /* PLL1 config */
       adi,pll1-feedback-div = <4>;
       adi,pll1-charge-pump-current-nA = <5000>;

       /* PLL2 config */ /* 983040000 Hz */
       adi,pll2-vco-div-m1 = <4>;
       adi,pll2-n2-div = <4>; /* N / M1 */
       adi,pll2-r1-div = <1>;
       adi,pll2-charge-pump-current-nA = <815000>;
       adi,pll2-freq-doubler-enable;

       /* SYSREF config */
       adi,sysref-src = <SYSREF_SRC_INTERNAL>;
       adi,sysref-pattern-mode = <SYSREF_PATTERN_NSHOT>;
       adi,sysref-k-div = <512>;
       adi,sysref-nshot-mode = <SYSREF_NSHOT_8_PULSES>;
       adi,sysref-request-trigger-mode = <SYSREF_LEVEL_HIGH>;
       adi,jesd204-desired-sysref-frequency-hz = <3840000>;

       adi,rpole2 = <RPOLE2_900_OHM>;
       adi,rzero = <RZERO_1850_OHM>;
       adi,cpole1 = <CPOLE1_16_PF>;

       adi,status-mon-pin0-function-select = <9>; /* PLL1 in holdover */
       adi,status-mon-pin1-function-select = <3>; /* PLL2 locked */

       ad9528_0_c0: channel@0 {
           reg = <0>;
           adi,extended-name = "DEV_SYSREF";
           adi,driver-mode = <DRIVER_MODE_LVDS>;
           adi,divider-phase = <0>;
           adi,channel-divider = <4>;
           adi,signal-source = <SOURCE_SYSREF_VCO>;
           //adi,output-dis;
       };

       ad9528_0_c1: channel@1 {
           reg = <1>;
           adi,extended-name = "DEV_CLK";
           adi,driver-mode = <DRIVER_MODE_LVDS>;
           adi,divider-phase = <0>;
           adi,channel-divider = <4>;
           adi,signal-source = <SOURCE_VCO>;
           //adi,output-dis;
       };

       ad9528_0_c3: channel@3 {
           reg = <3>;
           adi,extended-name = "CORE_CLK";
           adi,driver-mode = <DRIVER_MODE_LVDS>;
           adi,divider-phase = <0>;
           adi,channel-divider = <4>;
           adi,signal-source = <SOURCE_VCO>;
           //adi,output-dis;
       };

       ad9528_0_c11: channel@11 {
           reg = <11>;
           adi,extended-name = "REF_CLK1";
           adi,driver-mode = <DRIVER_MODE_LVDS>;
           adi,divider-phase = <0>;
           adi,channel-divider = <2>;
           adi,signal-source = <SOURCE_SYSREF_VCO>;
           //adi,output-dis;
       };

       ad9528_0_c12: channel@12 {
           reg = <12>;
           adi,extended-name = "FPGA_SYSREF";
           adi,driver-mode = <DRIVER_MODE_LVDS>;
           adi,divider-phase = <0>;
           adi,channel-divider = <4>;
           adi,signal-source = <SOURCE_SYSREF_VCO>;
           //adi,output-dis;
       };

       ad9528_0_c13: channel@13 {
           reg = <13>;
           adi,extended-name = "REF_CLK0";
           adi,driver-mode = <DRIVER_MODE_LVDS>;
           adi,divider-phase = <0>;
           adi,channel-divider = <2>;
           adi,signal-source = <SOURCE_VCO>;
           adi,output-dis;
       };
   };

ADRV904x PHY
------------

::

   trx0_adrv904x: adrv904x-phy@0 {
           compatible = "adrv9040";
           reg = <0>;

           #address-cells = <1>;
           #size-cells = <0>;

           /* SPI Setup */
           spi-max-frequency = <25000000>;

           interrupt-parent = <&gpio>;
           interrupts = <134 IRQ_TYPE_EDGE_RISING>; /* adrv904x_gpint1 - CHECK */

           /* Clocks */
           clocks = <&clk0_ad9528 1>;

           clock-names = "dev_clk";

           clock-output-names = "rx_sampl_clk", "tx_sampl_clk";
           #clock-cells = <1>;

           jesd204-device;
           #jesd204-cells = <2>;
           jesd204-top-device = <0>; /* This is the TOP device */
           jesd204-link-ids = <DEFRAMER0_LINK_TX FRAMER0_LINK_RX>;

           jesd204-inputs =
               <&axi_adrv904x_rx_jesd 0 FRAMER0_LINK_RX>,
               <&axi_adrv904x_core_tx 0 DEFRAMER0_LINK_TX>;

           adi,device-config-name = "DeviceProfileTest.bin";
           adi,arm-firmware-name = "ADRV9040_FW.bin";
           adi,arm-dfe-firmware-name = "ADRV9040_DFE_CALS_FW.bin";
           adi,stream-firmware-name = "stream_image.bin";
           adi,rx-gaintable-names = "ADRV9040_RxGainTable.csv";
           adi,rx-gaintable-channel-masks = <0xFF>;
           //adi,tx-attntable-names = "ADRV904X_TxAttenTable.csv";
           //adi,tx-attntable-channel-masks = <0x0F>;
       };

.. list-table::
   :header-rows: 1

   * - Devicetree property
     - Description
   * - adi,device-config-name
     - Device profile file (may also contain init data structures)
   * - adi,arm-firmware-name
     - Firmware file for the ARM processor
   * - adi,arm-dfe-firmware-name
     - Firmware containing DFE CPU File Settings
   * - adi,stream-firmware-name
     - Firmware file for the stream processor
   * - adi,rx-gaintable-names
     - RX gain table
   * - adi,rx-gaintable-channel-masks
     - Channel mask for RX gain values
   * - adi,tx-attntable-names
     - TX attenuation table
   * - adi,tx-attntable-channel-masks
     - Channel mask for TX attenuation values

AXI FPGA
--------

::

   fpga_axi: fpga-axi@0 {
           interrupt-parent = <&gic>;
           compatible = "simple-bus";
           #address-cells = <0x1>;
           #size-cells = <0x1>;
           ranges = <0 0 0 0xffffffff>;

           rx_dma: dma@9c400000 {
               compatible = "adi,axi-dmac-1.00.a";
               reg = <0x9c400000 0x1000>;
               #dma-cells = <1>;
               #clock-cells = <0>;
               interrupts = <0 110 IRQ_TYPE_LEVEL_HIGH>;
               clocks = <&zynqmp_clk 73>;
           };

           tx_dma: dma@9c420000  {
               compatible = "adi,axi-dmac-1.00.a";
               reg = <0x9c420000 0x1000>;
               #dma-cells = <1>;
               #clock-cells = <0>;
               interrupts = <0 109 IRQ_TYPE_LEVEL_HIGH>;
               clocks = <&zynqmp_clk 73>;
           };

           axi_adrv904x_core_rx: axi-adrv904x-rx-hpc@84a00000 {
               compatible = "adi,axi-adc-10.0.a";
               reg = <0x84a00000 0x6000>;
               dmas = <&rx_dma 0>;
               dma-names = "rx";
               spibus-connected = <&trx0_adrv904x>;
           };

           axi_adrv904x_core_tx: axi-adrv904x-tx-hpc@84a04000 {
               compatible = "adi,axi-adrv904x-tx-1.0";
               reg = <0x84a04000 0x2000>;
               dmas = <&tx_dma 0>;
               dma-names = "tx";
               clocks = <&trx0_adrv904x 1>;
               clock-names = "sampl_clk";
               //adi,axi-pl-fifo-enable;

               jesd204-device;
               #jesd204-cells = <2>;
               jesd204-inputs = <&axi_adrv904x_tx_jesd 0 DEFRAMER0_LINK_TX>;
           };

           axi_adrv904x_rx_jesd: axi-jesd204-rx@84aa0000 {
               compatible = "adi,axi-jesd204-rx-1.0";
               reg = <0x84aa0000 0x4000>;

               interrupts = <0 107 IRQ_TYPE_LEVEL_HIGH>;

               clocks = <&zynqmp_clk 71>, <&clk0_ad9528 3>, <&axi_adrv904x_adxcvr_rx 0>;
               clock-names = "s_axi_aclk", "device_clk", "lane_clk";

               #clock-cells = <0>;
               clock-output-names = "jesd_rx_lane_clk";

               adi,octets-per-frame = <4>;
               adi,frames-per-multiframe = <32>;

               jesd204-device;
               #jesd204-cells = <2>;
               jesd204-inputs = <&axi_adrv904x_adxcvr_rx 0 FRAMER0_LINK_RX>;
           };

           axi_adrv904x_tx_jesd: axi-jesd204-tx@84a90000 {
               compatible = "adi,axi-jesd204-tx-1.0";
               reg = <0x84a90000 0x4000>;

               interrupts = <0 106 IRQ_TYPE_LEVEL_HIGH>;

               clocks = <&zynqmp_clk 71>, <&clk0_ad9528 3>, <&axi_adrv904x_adxcvr_tx 0>;
               clock-names = "s_axi_aclk", "device_clk", "lane_clk";

               #clock-cells = <0>;
               clock-output-names = "jesd_tx_lane_clk";

               jesd204-device;
               #jesd204-cells = <2>;
               jesd204-inputs = <&axi_adrv904x_adxcvr_tx 0 DEFRAMER0_LINK_TX>;

           };

           axi_adrv904x_adxcvr_rx: axi-adxcvr-rx@84a60000 {
               #address-cells = <1>;
               #size-cells = <0>;
               compatible = "adi,axi-adxcvr-1.0";
               reg = <0x84a60000 0x10000>;

               clocks = <&clk0_ad9528 13>;
               clock-names = "conv";

               #clock-cells = <1>;
               clock-output-names = "rx_gt_clk", "rx_out_clk";

               adi,sys-clk-select = <XCVR_QPLL>;
               adi,out-clk-select = <XCVR_REFCLK>;

               jesd204-device;
               #jesd204-cells = <2>;
               jesd204-inputs =  <&clk0_ad9528 0 FRAMER0_LINK_RX>;

           };

           axi_adrv904x_adxcvr_tx: axi-adxcvr-tx@84a80000 {
               #address-cells = <1>;
               #size-cells = <0>;
               compatible = "adi,axi-adxcvr-1.0";
               reg = <0x84a80000 0x1000>;

               clocks = <&clk0_ad9528 13>;
               clock-names = "conv";

               #clock-cells = <1>;
               clock-output-names = "tx_gt_clk", "tx_out_clk";

               adi,sys-clk-select = <XCVR_QPLL>;
               adi,out-clk-select = <XCVR_REFCLK>;

               jesd204-device;
               #jesd204-cells = <2>;
               jesd204-inputs =  <&clk0_ad9528 0 DEFRAMER0_LINK_TX>;

           };

           axi_sysid_0: axi-sysid-0@85000000 {
               compatible = "adi,axi-sysid-1.00.a";
               reg = <0x85000000 0x10000>;
           };
       };

GPIO Settings
-------------

The GPIO configurations can be done by changing a node’s properties, as the
following example shows:

::

   &trx0_adrv904x {
       reset-gpios = <&gpio 134 0>;
   };

   &clk0_ad9528 {
       reset-gpios = <&gpio 147 0>;
   };

.. warning::

   When configuring GPIOs manually, make sure to not collide to any pin selected
   in another node configuration as adi,increment-pin, adi,decrement-pin,
   adi,agc-power-feedback-high-thres-exceeded, etc…
