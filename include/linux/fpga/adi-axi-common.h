// SPDX-License-Identifier: GPL-2.0
/*
 * ADI AXI-COMMON Module
 *
 * Copyright 2019 Analog Devices Inc.
 *
 * https://wiki.analog.com/resources/fpga/docs/axi_ip
 */

#ifndef ADI_AXI_COMMON_H_
#define ADI_AXI_COMMON_H_

#define AXI_PCORE_VER(major, minor, letter)	((major << 16) | (minor << 8) | letter)
#define AXI_PCORE_VER_MAJOR(version)	(version >> 16)
#define AXI_PCORE_VER_MINOR(version)	((version >> 8) & 0xff)
#define AXI_PCORE_VER_LETTER(version)	(version & 0xff)

#define	AXI_REG_VERSION			0x0000
#define AXI_VERSION(x)			(((x) & 0xffffffff) << 0)
#define AXI_VERSION_IS(x, y, z)		((x) << 16 | (y) << 8 | (z))
#define AXI_VERSION_MAJOR(x)		((x) >> 16)

#define AXI_REG_FPGA_INFO		0x001C
#define AXI_REG_FPGA_VOLTAGE		0x0140

#define AXI_INFO_FPGA_TECH(info)        ((info) >> 24)
#define AXI_INFO_FPGA_FAMILY(info)      (((info) >> 16) & 0xff)
#define AXI_INFO_FPGA_SPEED_GRADE(info)	(((info) >> 8) & 0xff)
#define AXI_INFO_FPGA_DEV_PACKAGE(info)	((info) & 0xff)
#define AXI_INFO_FPGA_VOLTAGE(val)      ((val) & 0xffff)

#define AXI_REG_ID			0x0004
#define AXI_REG_SCRATCH			0x0008
#define AXI_REG_CONFIG			0x000c

enum axi_fgpa_technology {
	AXI_FPGA_TECH_UNKNOWN = 0,
	AXI_FPGA_TECH_SERIES7,
	AXI_FPGA_TECH_ULTRASCALE,
	AXI_FPGA_TECH_ULTRASCALE_PLUS,
};

enum axi_fpga_family {
	AXI_FPGA_FAMILY_UNKNOWN = 0,
	AXI_FPGA_FAMILY_ARTIX,
	AXI_FPGA_FAMILY_KINTEX,
	AXI_FPGA_FAMILY_VIRTEX,
	AXI_FPGA_FAMILY_ZYNQ,
};

enum axi_fpga_speed_grade {
	AXI_FPGA_SPEED_UNKNOWN	= 0,
	AXI_FPGA_SPEED_1	= 10,
	AXI_FPGA_SPEED_1L	= 11,
	AXI_FPGA_SPEED_1H	= 12,
	AXI_FPGA_SPEED_1HV	= 13,
	AXI_FPGA_SPEED_1LV	= 14,
	AXI_FPGA_SPEED_2	= 20,
	AXI_FPGA_SPEED_2L	= 21,
	AXI_FPGA_SPEED_2LV	= 22,
	AXI_FPGA_SPEED_3	= 30,
};

enum axi_fpga_dev_pack {
	AXI_FPGA_DEV_UNKNOWN = 0,
	AXI_FPGA_DEV_RF,
	AXI_FPGA_DEV_FL,
	AXI_FPGA_DEV_FF,
	AXI_FPGA_DEV_FB,
	AXI_FPGA_DEV_HC,
	AXI_FPGA_DEV_FH,
	AXI_FPGA_DEV_CS,
	AXI_FPGA_DEV_CP,
	AXI_FPGA_DEV_FT,
	AXI_FPGA_DEV_FG,
	AXI_FPGA_DEV_SB,
	AXI_FPGA_DEV_RB,
	AXI_FPGA_DEV_RS,
	AXI_FPGA_DEV_CL,
	AXI_FPGA_DEV_SF,
	AXI_FPGA_DEV_BA,
	AXI_FPGA_DEV_FA,
};

#endif /* ADI_AXI_COMMON_H_ */
