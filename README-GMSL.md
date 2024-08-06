# Gigabit Multimedia Serial Link™ (GMSL) drivers for Xilinx platforms
## Table of contents
1. [Description](#description)
2. [How to build](#how-to-build)
3. [Source Code](#source-code)
4. [Device tree](#device-tree)
5. [Device tree bindings](#device-tree-bindings)
6. [Testing](#testing)
7. [Debugging](#debugging)

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

[source-0]: https://github.com/analogdevicesinc/linux/blob/gmsl_k26/xilinx_v6.1_LTS/drivers/media/i2c/maxim-serdes/max96717.c
[source-1]: https://github.com/analogdevicesinc/linux/blob/gmsl_k26/xilinx_v6.1_LTS/drivers/media/i2c/maxim-serdes/max96724.c
[source-2]: https://github.com/analogdevicesinc/linux/blob/gmsl_k26/xilinx_v6.1_LTS/drivers/media/i2c/maxim-serdes/max9296a.c
[source-3]: https://github.com/analogdevicesinc/linux/blob/gmsl_k26/xilinx_v6.1_LTS/drivers/media/i2c/maxim-serdes/max_ser.c
[source-4]: https://github.com/analogdevicesinc/linux/blob/gmsl_k26/xilinx_v6.1_LTS/drivers/media/i2c/maxim-serdes/max_des.c
[source-5]: https://github.com/analogdevicesinc/linux/blob/gmsl_k26/xilinx_v6.1_LTS/drivers/media/i2c/maxim-serdes/max_aggregator.c

## Device tree

| Release Tag       	      | File              	|
|-------------------------	| ------------------	|
| gmsl_k26/xilinx_v6.1_LTS 	| [gmsl.dts][dtss-0] 	|

[dtss-0]: https://github.com/analogdevicesinc/linux/blob/gmsl_k26/xilinx_v6.1_LTS/arch/arm64/boot/dts/xilinx/gmsl-k26.dts

## Device tree bindings

| Release Tag       	      | File                       	|
|-------------------------	| ---------------------------	|
| gmsl_k26/xilinx_v6.1_LTS 	| [MAX96717/MAX9295A][doc-0] 	|
| gmsl_k26/xilinx_v6.1_LTS 	| [MAX96724][doc-1]          	|
| gmsl_k26/xilinx_v6.1_LTS 	| [MAX9296A][doc-2]          	|
| gmsl_k26/xilinx_v6.1_LTS 	| [Serializer][doc-3]        	|
| gmsl_k26/xilinx_v6.1_LTS 	| [Deserializer][doc-4]      	|

[doc-0]: https://github.com/analogdevicesinc/linux/blob/gmsl_k26/xilinx_v6.1_LTS/Documentation/devicetree/bindings/media/i2c/maxim%2Cmax96717.yaml
[doc-1]: https://github.com/analogdevicesinc/linux/blob/gmsl_k26/xilinx_v6.1_LTS/Documentation/devicetree/bindings/media/i2c/maxim%2Cmax96724.yaml
[doc-2]: https://github.com/analogdevicesinc/linux/blob/gmsl_k26/xilinx_v6.1_LTS/Documentation/devicetree/bindings/media/i2c/maxim%2Cmax9296a.yaml
[doc-3]: https://github.com/analogdevicesinc/linux/blob/gmsl_k26/xilinx_v6.1_LTS/Documentation/devicetree/bindings/media/i2c/maxim-serializer.yaml
[doc-4]: https://github.com/analogdevicesinc/linux/blob/gmsl_k26/xilinx_v6.1_LTS/Documentation/devicetree/bindings/media/i2c/maxim-deserializer.yaml

## Testing
Please follow the instructions [from](https://wiki.analog.com/resources/eval/user-guides/ad-gmsl2eth-sl-guide)

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
