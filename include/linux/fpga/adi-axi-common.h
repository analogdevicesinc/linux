/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Analog Devices AXI common registers & definitions
 *
 * Copyright 2019 Analog Devices Inc.
 *
 * https://wiki.analog.com/resources/fpga/docs/axi_ip
 * https://wiki.analog.com/resources/fpga/docs/hdl/regmap
 */

#ifndef ADI_AXI_COMMON_H_
#define ADI_AXI_COMMON_H_

#define	ADI_AXI_REG_VERSION			0x0000
#define ADI_AXI_REG_ID				0x0004
#define ADI_AXI_REG_SCRATCH			0x0008
#define ADI_AXI_REG_CONFIG			0x000c

#define ADI_AXI_REG_FPGA_INFO			0x001C
#define ADI_AXI_REG_FPGA_VOLTAGE		0x0140

#define ADI_AXI_PCORE_VER(major, minor, patch)	\
	(((major) << 16) | ((minor) << 8) | (patch))

#define ADI_AXI_PCORE_VER_MAJOR(version)	(((version) >> 16) & 0xff)
#define ADI_AXI_PCORE_VER_MINOR(version)	(((version) >> 8) & 0xff)
#define ADI_AXI_PCORE_VER_PATCH(version)	((version) & 0xff)

#define ADI_AXI_INFO_FPGA_TECH(info)		(((info) >> 24) & 0xff)
#define ADI_AXI_INFO_FPGA_FAMILY(info)		(((info) >> 16) & 0xff)
#define ADI_AXI_INFO_FPGA_SPEED_GRADE(info)	(((info) >> 8) & 0xff)
#define ADI_AXI_INFO_FPGA_DEV_PACKAGE(info)	((info) & 0xff)
#define ADI_AXI_INFO_FPGA_VOLTAGE(val)		((val) & 0xffff)

enum adi_axi_fgpa_technology {
	ADI_AXI_FPGA_TECH_UNKNOWN = 0,
	ADI_AXI_FPGA_TECH_SERIES7,
	ADI_AXI_FPGA_TECH_ULTRASCALE,
	ADI_AXI_FPGA_TECH_ULTRASCALE_PLUS,
};

enum adi_axi_fpga_family {
	ADI_AXI_FPGA_FAMILY_UNKNOWN = 0,
	ADI_AXI_FPGA_FAMILY_ARTIX,
	ADI_AXI_FPGA_FAMILY_KINTEX,
	ADI_AXI_FPGA_FAMILY_VIRTEX,
	ADI_AXI_FPGA_FAMILY_ZYNQ,
};

enum adi_axi_fpga_speed_grade {
	ADI_AXI_FPGA_SPEED_UNKNOWN	= 0,
	ADI_AXI_FPGA_SPEED_1	= 10,
	ADI_AXI_FPGA_SPEED_1L	= 11,
	ADI_AXI_FPGA_SPEED_1H	= 12,
	ADI_AXI_FPGA_SPEED_1HV	= 13,
	ADI_AXI_FPGA_SPEED_1LV	= 14,
	ADI_AXI_FPGA_SPEED_2	= 20,
	ADI_AXI_FPGA_SPEED_2L	= 21,
	ADI_AXI_FPGA_SPEED_2LV	= 22,
	ADI_AXI_FPGA_SPEED_3	= 30,
};

enum adi_axi_fpga_dev_pack {
	ADI_AXI_FPGA_DEV_UNKNOWN = 0,
	ADI_AXI_FPGA_DEV_RF,
	ADI_AXI_FPGA_DEV_FL,
	ADI_AXI_FPGA_DEV_FF,
	ADI_AXI_FPGA_DEV_FB,
	ADI_AXI_FPGA_DEV_HC,
	ADI_AXI_FPGA_DEV_FH,
	ADI_AXI_FPGA_DEV_CS,
	ADI_AXI_FPGA_DEV_CP,
	ADI_AXI_FPGA_DEV_FT,
	ADI_AXI_FPGA_DEV_FG,
	ADI_AXI_FPGA_DEV_SB,
	ADI_AXI_FPGA_DEV_RB,
	ADI_AXI_FPGA_DEV_RS,
	ADI_AXI_FPGA_DEV_CL,
	ADI_AXI_FPGA_DEV_SF,
	ADI_AXI_FPGA_DEV_BA,
	ADI_AXI_FPGA_DEV_FA,
};

#endif /* ADI_AXI_COMMON_H_ */
