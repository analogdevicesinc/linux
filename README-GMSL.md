# Gigabit Multimedia Serial Link™ (GMSL) drivers for Xilinx platforms
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

The Linux kernel in this repository is the [Linux kernel from Xilinx](https://github.com/Xilinx/linux-xlnx) together with drivers & patches applied from Analog Devices.

The current branch is based on [xilinx_v6.1_LTS](https://github.com/Xilinx/linux-xlnx/tree/xlnx_rebase_v6.1_LTS).

## How to build

For build instructions [check the wiki](https://wiki.analog.com/resources/tools-software/linux-drivers-all#building_the_adi_linux_kernel).

## Source code

| Release Tag   	      | File                          	|
|---------------------	| ------------------------------	|
| gmsl/xilinx_v6.1_LTS 	| [MAX96717/MAX9295A][source-0] 	|
| gmsl/xilinx_v6.1_LTS 	| [MAX96724][source-1]          	|
| gmsl/xilinx_v6.1_LTS 	| [MAX9296A][source-2]          	|
| gmsl/xilinx_v6.1_LTS 	| [Serializer][source-3]        	|
| gmsl/xilinx_v6.1_LTS 	| [Deserializer][source-4]      	|
| gmsl/xilinx_v6.1_LTS 	| [Aggregator][source-5]        	|

[source-0]: https://github.com/analogdevicesinc/linux/blob/gmsl/xilinx_v6.1_LTS/drivers/media/i2c/maxim-serdes/max96717.c
[source-1]: https://github.com/analogdevicesinc/linux/blob/gmsl/xilinx_v6.1_LTS/drivers/media/i2c/maxim-serdes/max96724.c
[source-2]: https://github.com/analogdevicesinc/linux/blob/gmsl/xilinx_v6.1_LTS/drivers/media/i2c/maxim-serdes/max9296a.c
[source-3]: https://github.com/analogdevicesinc/linux/blob/gmsl/xilinx_v6.1_LTS/drivers/media/i2c/maxim-serdes/max_ser.c
[source-4]: https://github.com/analogdevicesinc/linux/blob/gmsl/xilinx_v6.1_LTS/drivers/media/i2c/maxim-serdes/max_des.c
[source-5]: https://github.com/analogdevicesinc/linux/blob/gmsl/xilinx_v6.1_LTS/drivers/media/i2c/maxim-serdes/max_aggregator.c

## Device tree

| Release Tag       	| File              	|
|-------------------	| ------------------	|
| gmsl/xilinx_v6.1_LTS 	| [gmsl.dts][dtss-0] 	|

[dtss-0]: https://github.com/analogdevicesinc/linux/blob/gmsl/xilinx_v6.1_LTS/arch/arm64/boot/dts/xilinx/gmsl.dts

## Device tree bindings

| Release Tag       	| File                       	|
|-------------------	| ---------------------------	|
| gmsl/xilinx_v6.1_LTS 	| [MAX96717/MAX9295A][doc-0] 	|
| gmsl/xilinx_v6.1_LTS 	| [MAX96724][doc-1]          	|
| gmsl/xilinx_v6.1_LTS 	| [MAX9296A][doc-2]          	|
| gmsl/xilinx_v6.1_LTS 	| [Serializer][doc-3]        	|
| gmsl/xilinx_v6.1_LTS 	| [Deserializer][doc-4]      	|

[doc-0]: https://github.com/analogdevicesinc/nvidia/blob/gmsl/jetson_35.3.1/kernel_kernel-5.10/Documentation/devicetree/bindings/media/i2c/maxim%2Cmax96717.yaml
[doc-1]: https://github.com/analogdevicesinc/nvidia/blob/gmsl/jetson_35.3.1/kernel_kernel-5.10/Documentation/devicetree/bindings/media/i2c/maxim%2Cmax96724.yaml
[doc-2]: https://github.com/analogdevicesinc/nvidia/blob/gmsl/jetson_35.3.1/kernel_kernel-5.10/Documentation/devicetree/bindings/media/i2c/maxim%2Cmax9296a.yaml
[doc-3]: https://github.com/analogdevicesinc/nvidia/blob/gmsl/jetson_35.3.1/kernel_kernel-5.10/Documentation/devicetree/bindings/media/i2c/maxim-serializer.yaml
[doc-4]: https://github.com/analogdevicesinc/nvidia/blob/gmsl/jetson_35.3.1/kernel_kernel-5.10/Documentation/devicetree/bindings/media/i2c/maxim-deserializer.yaml


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

#### MAX9295A, MAX96717

Configure it to use device address 0x80 (0x40), COAX, and either Tunnel or Pixel mode. The default device tree configuration will switch it to pixel mode.

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

### Debugging

To read and write registers, you can interact with the devices through the V4L2 API.

The following commands will read and write register `0x8d3` from subdev `12`, which is the MAX96724 in the example above.

`v4l2-dbg -d 0 -c subdev12 -g 0x8d3`

`v4l2-dbg -d 0 -c subdev12 -s 0x8d3 0x00`

These commands can only be run as root.

Below are registers that might help you when trying to figure out where the problem is.

#### MAX96724

```
MIPI_PHY25 (0x8D0)
MIPI_PHY26 (0x8D1)
MIPI_PHY27 (0x8D2)
MIPI_PHY28 (0x8D3)
VPRBS (0x1DC, 0x1FC, 0x21C, 0x23C)
```

#### MAX96717

```
VIDEO_TX2 (0x112)
EXT21 (0x38D)
EXT22 (0x38E)
EXT23 (0x38F)
EXT24 (0x390)
```

#### MAX9295A

```
VIDEO_TX2 (0x102)
```
