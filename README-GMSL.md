# Gigabit Multimedia Serial Link™ (GMSL) drivers for Raspberry PI platform
## Table of contents
1. [Description](#description)
2. [How to build](#how-to-build)
3. [Source Code](#source-code)
4. [Device tree](#device-tree)
5. [Device tree bindings](#device-tree-bindings)
6. [GMSL hardware configuration](#hardware-configuration)
7. [Testing](#testing)
8. [Debugging](#debugging)

## Description

The Linux kernel in this repository is the [Raspberrypi linux kernel tree](https://github.com/raspberrypi/linux) together with drivers & patches applied by Analog Devices.

The current branch is based on [rpi-6.13.y](https://github.com/raspberrypi/linux/tree/rpi-6.13.y).

## How to build

### Getting the ADI kernel
First clone the ADI Linux repo, branch `gmsl/rpi-6.13.y` with the following command:

`git clone --branch gmsl/rpi-6.13.y https://github.com/analogdevicesinc/linux`

Second, configure the kernel build with *bcm2711_adi_gmsl_defconfig* for **RPI4** or

*bcm2712_adi_gmsl_defconfig* for **RPI5**

`make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- bcm2712_adi_gmsl_defconfig`

Last, build the kernel image, modules and devicetree blobs

`make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image modules dtbs`

For more detailed instructions please check the [RPI wiki](https://www.raspberrypi.com/documentation/computers/linux_kernel.html#cross-compiling-the-kernel).

## Source code

| Release Tag     | File                          	|
|----------------	| ------------------------------	|
| gmsl/rpi-6.13.y 	| [MAX9295A/MAX96717[F]/MAX96793][source-0] 	|
| gmsl/rpi-6.13.y 	| [MAX96712/MAX96724[F/R]][source-1]          	|
| gmsl/rpi-6.13.y 	| [MAX9296A/MAX96714[F/R]/MAX96716A/MAX96792A][source-2]          	|
| gmsl/rpi-6.13.y 	| [Serializer][source-3]        	|
| gmsl/rpi-6.13.y 	| [Deserializer][source-4]      	|

[source-0]: https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.13.y/drivers/media/i2c/maxim-serdes/max96717.c
[source-1]: https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.13.y/drivers/media/i2c/maxim-serdes/max96724.c
[source-2]: https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.13.y/drivers/media/i2c/maxim-serdes/max9296a.c
[source-3]: https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.13.y/drivers/media/i2c/maxim-serdes/max_ser.c
[source-4]: https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.13.y/drivers/media/i2c/maxim-serdes/max_des.c

## Device tree

| Release Tag   	| File           	|
|---------------	| ---------------	|
| gmsl/rpi-6.13.y 	| [gen_gmsl_dts][dtss-0] 	|

[dtss-0]: https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.13.y/arch/arm/boot/dts/overlays/gen_gmsl_dts/README.md

## Device tree bindings

| Release Tag   	| File                       	|
|---------------	| ---------------------------	|
| gmsl/rpi-6.13.y 	| [MAX9295A/MAX96717[F]/MAX96793][doc-0] 	|
| gmsl/rpi-6.13.y 	| [MAX96712/MAX96724[F/R]][doc-1]          	|
| gmsl/rpi-6.13.y 	| [MAX9296A/MAX96714[F/R]/MAX96716A/MAX96792A][doc-2]          	|
| gmsl/rpi-6.13.y 	| [Serializer][doc-3]        	|
| gmsl/rpi-6.13.y 	| [Deserializer][doc-4]      	|

[doc-0]: https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.1.y/Documentation/devicetree/bindings/media/i2c/maxim%2Cmax96717.yaml
[doc-1]: https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.1.y/Documentation/devicetree/bindings/media/i2c/maxim%2Cmax96724.yaml
[doc-2]: https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.1.y/Documentation/devicetree/bindings/media/i2c/maxim%2Cmax9296a.yaml
[doc-3]: https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.1.y/Documentation/devicetree/bindings/media/i2c/maxim-serializer.yaml
[doc-4]: https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.1.y/Documentation/devicetree/bindings/media/i2c/maxim-deserializer.yaml


## Hardware configuration

### CFG Pin Levels

EVKITs need to have their CFG Pin Levels configured using the [GMSL SerDes GUI Software][gui-0].

To do this, open the software, navigate to the `Tools` tab, and then press on the `Set CFG Pin Levels` entry under the `Other Config` section.

Use the `Serializer` tab to configure serializers, and the `Deserializer` tab to configure deserializers.

You'll have to connect your serializers one by one to configure them since the GUI can't switch between multiple connected serializers.

[gui-0]: https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download.html?swpart=SFW0019760F

#### MAX96724

Configure it to use device address 0x4e (0x27), COAX, GMSL2 and 6Gbps.

This should mean `CFG0` at pin level `0` and `CFG1` at pin level `1`.

#### MAX9295A, MAX96717, MAX96793

Configure it to use device address 0x80 (0x40), COAX, and either Tunnel or Pixel mode. The default device tree configuration will switch it to pixel mode if more than one serializer is connected to a deserializer.

This should mean `CFG0` at pin level `0` and `CFG1` at pin level `5` or `7`.

### CSI I2C

When using the MAX96724 deserializer, to accept I2C communcation over the CSI bus, you will have to flip the `SW5` switches to the `ON` position.

## Testing
### QV4L2

To test the cameras using qv4l2, you need to install the `qv4l2` package using the following command.

`sudo apt install qv4l2`

To open the camera `0` in the `qv4l2` app, run the following command.

`qv4l2 -d 0`

The same applies for other cameras.

## Debugging

### Log status

GMSL drivers implement the standard log status controls. When log status is called for one GMSL V4L subdev it will prin a lot of useful debugging information related to routuing, MIPI stream datat types, important registers values like video_lock, csi_pkt_cnt I2C translates, etc.

#### Serializer
```
~$: v4l2-ctl --log-status -d /dev/v4l-subdev3

max96717 15-0040: =================  START STATUS  =================
max96717 15-0040: mode: pixel
max96717 15-0040: tpg: disabled
max96717 15-0040: tun_pkt_cnt: 0
max96717 15-0040: i2c_xlates:
max96717 15-0040:     en: 1, src: 0x50 dst: 0x10
max96717 15-0040:     en: 0, src: 0x00 dst: 0x00
max96717 15-0040:
max96717 15-0040:
max96717 15-0040: pipe: 0
max96717 15-0040:     enabled: 0
max96717 15-0040:
max96717 15-0040: phy: 0
max96717 15-0040:     enabled: 1
max96717 15-0040:     active: 0
max96717 15-0040:     num_data_lanes: 2
max96717 15-0040:     clock_lane: 0
max96717 15-0040:     noncontinuous_clock: 1
max96717 15-0040:     phy_pkt_cnt: 0
max96717 15-0040:     csi_pkt_cnt: 0
max96717 15-0040:     phy_clk_cnt: 0
max96717 15-0040:
max96717 15-0040: ==================  END STATUS  ==================
```

#### Deserializer

```
~$: v4l2-ctl --log-status -d /dev/v4l-subdev2

max9296a 10-002a: =================  START STATUS  =================
max9296a 10-002a: active: 0
max9296a 10-002a: mode: pixel
max9296a 10-002a: tpg: disabled
max9296a 10-002a:
max9296a 10-002a: link: 0
max9296a 10-002a:     enabled: 1
max9296a 10-002a:     version: GMSL2 6Gbps
max9296a 10-002a:     ser_xlate: en: 1, src: 0x40 dst: 0x40
max9296a 10-002a:
max9296a 10-002a: link: 1
max9296a 10-002a:     enabled: 0
max9296a 10-002a:
max9296a 10-002a: pipe: 0
max9296a 10-002a:     enabled: 0
max9296a 10-002a:     phy_id: invalid
max9296a 10-002a:     link_id: 0
max9296a 10-002a:     stream_id: 0
max9296a 10-002a:     dbl8: 0
max9296a 10-002a:     dbl8mode: 0
max9296a 10-002a:     dbl10: 0
max9296a 10-002a:     dbl10mode: 0
max9296a 10-002a:     dbl12: 0
max9296a 10-002a:     remaps: 0
max9296a 10-002a:     video_lock: 0
max9296a 10-002a:
max9296a 10-002a: pipe: 1
max9296a 10-002a:     enabled: 0
max9296a 10-002a:     phy_id: invalid
max9296a 10-002a:     link_id: 1
max9296a 10-002a:     stream_id: 0
max9296a 10-002a:     dbl8: 0
max9296a 10-002a:     dbl8mode: 0
max9296a 10-002a:     dbl10: 0
max9296a 10-002a:     dbl10mode: 0
max9296a 10-002a:     dbl12: 0
max9296a 10-002a:     remaps: 0
max9296a 10-002a:     video_lock: 0
max9296a 10-002a:
max9296a 10-002a: phy: 0
max9296a 10-002a:     enabled: 0
max9296a 10-002a:
max9296a 10-002a: phy: 1
max9296a 10-002a:     enabled: 1
max9296a 10-002a:     link_frequency: 750000000
max9296a 10-002a:     num_data_lanes: 4
max9296a 10-002a:     clock_lane: 0
max9296a 10-002a:     alt_mem_map8: 0
max9296a 10-002a:     alt2_mem_map8: 0
max9296a 10-002a:     alt_mem_map10: 0
max9296a 10-002a:     alt_mem_map12: 0
max9296a 10-002a:     csi2_pkt_cnt: 0
max9296a 10-002a:     phy_pkt_cnt: 0
max9296a 10-002a:
max9296a 10-002a: ==================  END STATUS  ==================
```

### Register read/write

To read and write registers, you can interact with the devices through the V4L2 API.

The following commands will read and write register `0x8d3` from subdev `12`, which is the MAX96724 in the example above.

`v4l2-dbg -d 0 -c subdev12 -g 0x8d3`

`v4l2-dbg -d 0 -c subdev12 -s 0x8d3 0x00`

These commands can only be run as root.
