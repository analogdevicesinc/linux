/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Analog Devices ADSP family SRU control driver.
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Author: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef SRU_CTRL_ADSP_H
#define SRU_CTRL_ADSP_H

#include <linux/clk-provider.h>

#define REG_DAI_EXTD_CLK0  0x000	// DAI0 Extended Clock Routing Control Register 0
#define REG_DAI_EXTD_CLK1  0x004	// DAI0 Extended Clock Routing Control Register 1
#define REG_DAI_EXTD_CLK2  0x008	// DAI0 Extended Clock Routing Control Register 2
#define REG_DAI_EXTD_CLK3  0x00C	// DAI0 Extended Clock Routing Control Register 3
#define REG_DAI_EXTD_CLK4  0x010	// DAI0 Extended Clock Routing Control Register 4
#define REG_DAI_EXTD_CLK5  0x014	// DAI0 Extended Clock Routing Control Register 5
#define REG_DAI_EXTD_DAT0  0x018	// DAI0 Extended Serial Data Routing Control Register 0
#define REG_DAI_EXTD_DAT1  0x01C	// DAI0 Extended Serial Data Routing Control Register 1
#define REG_DAI_EXTD_DAT2  0x020	// DAI0 Extended Serial Data Routing Control Register 2
#define REG_DAI_EXTD_DAT3  0x024	// DAI0 Extended Serial Data Routing Control Register 3
#define REG_DAI_EXTD_DAT4  0x028	// DAI0 Extended Serial Data Routing Control Register 4
#define REG_DAI_EXTD_DAT5  0x02C	// DAI0 Extended Serial Data Routing Control Register 5
#define REG_DAI_EXTD_DAT6  0x030	// DAI0 Extended Serial Data Routing Control Register 6
#define REG_DAI_EXTD_FS0   0x034	// DAI0 Extended Frame Sync Routing Control Register 0
#define REG_DAI_EXTD_FS1   0x038	// DAI0 Extended Frame Sync Routing Control Register 1
#define REG_DAI_EXTD_FS2   0x03C	// DAI0 Extended Frame Sync Routing Control Register 2
#define REG_DAI_EXTD_FS4   0x044	// DAI0 Extended Frame Sync Routing Control Register 4
#define REG_DAI_EXTD_PIN0  0x048	// DAI0 Extended Pin Buffer Assignment Register 0
#define REG_DAI_EXTD_PIN1  0x04C	// DAI0 Extended Pin Buffer Assignment Register 1
#define REG_DAI_EXTD_PIN2  0x050	// DAI0 Extended Pin Buffer Assignment Register 2
#define REG_DAI_EXTD_PIN3  0x054	// DAI0 Extended Pin Buffer Assignment Register 3
#define REG_DAI_EXTD_PIN4  0x058	// DAI0 Extended Pin Buffer Assignment Register 4
#define REG_DAI_EXTD_MISC0 0x05C	// DAI0 Extended Miscellaneous Control Register 0
#define REG_DAI_EXTD_MISC1 0x060	// DAI0 Extended Miscellaneous Control Register 1
#define REG_DAI_EXTD_MISC2 0x064	// DAI0 Extended Miscellaneous Control Register 2
#define REG_DAI_EXTD_PBEN0 0x068	// DAI0 Extended Pin Buffer Enable Register 0
#define REG_DAI_EXTD_PBEN1 0x06C	// DAI0 Extended Pin Buffer Enable Register 1
#define REG_DAI_EXTD_PBEN2 0x070	// DAI0 Extended Pin Buffer Enable Register 2
#define REG_DAI_EXTD_PBEN3 0x074	// DAI0 Extended Pin Buffer Enable Register 3
#define REG_DAI_CLK0       0x0C0	// DAI0 Clock Routing Control Register 0
#define REG_DAI_CLK1       0x0C4	// DAI0 Clock Routing Control Register 1
#define REG_DAI_CLK2       0x0C8	// DAI0 Clock Routing Control Register 2
#define REG_DAI_CLK3       0x0CC	// DAI0 Clock Routing Control Register 3
#define REG_DAI_CLK4       0x0D0	// DAI0 Clock Routing Control Register 4
#define REG_DAI_CLK5       0x0D4	// DAI0 Clock Routing Control Register 5
#define REG_DAI_DAT0       0x100	// DAI0 Serial Data Routing Control Register 0
#define REG_DAI_DAT1       0x104	// DAI0 Serial Data Routing Control Register 1
#define REG_DAI_DAT2       0x108	// DAI0 Serial Data Routing Control Register 2
#define REG_DAI_DAT3       0x10C	// DAI0 Serial Data Routing Control Register 3
#define REG_DAI_DAT4       0x110	// DAI0 Serial Data Routing Control Register 4
#define REG_DAI_DAT5       0x114	// DAI0 Serial Data Routing Control Register 5
#define REG_DAI_DAT6       0x118	// DAI0 Serial Data Routing Control Register 6
#define REG_DAI_FS0        0x140	// DAI0 Frame Sync Routing Control Register 0
#define REG_DAI_FS1        0x144	// DAI0 Frame Sync Routing Control Register 1
#define REG_DAI_FS2        0x148	// DAI0 Frame Sync Routing Control Register 2
#define REG_DAI_FS4        0x150	// DAI0 Frame Sync Routing Control Register 4
#define REG_DAI_PIN0       0x180	// DAI0 Pin Buffer Assignment Register 0
#define REG_DAI_PIN1       0x184	// DAI0 Pin Buffer Assignment Register 1
#define REG_DAI_PIN2       0x188	// DAI0 Pin Buffer Assignment Register 2
#define REG_DAI_PIN3       0x18C	// DAI0 Pin Buffer Assignment Register 3
#define REG_DAI_PIN4       0x190	// DAI0 Pin Buffer Assignment Register 4
#define REG_DAI_MISC0      0x1C0	// DAI0 Miscellaneous Control Register 0
#define REG_DAI_MISC1      0x1C4	// DAI0 Miscellaneous Control Register 1
#define REG_DAI_MISC2      0x1C8	// DAI0 Miscellaneous Control Register 1
#define REG_DAI_PBEN0      0x1E0	// DAI0 Pin Buffer Enable Register 0
#define REG_DAI_PBEN1      0x1E4	// DAI0 Pin Buffer Enable Register 1
#define REG_DAI_PBEN2      0x1E8	// DAI0 Pin Buffer Enable Register 2
#define REG_DAI_PBEN3      0x1EC	// DAI0 Pin Buffer Enable Register 3

#define NUM_GROUPS 6
#define GROUP_A (1 << 0)
#define GROUP_B (1 << 1)
#define GROUP_C (1 << 2)
#define GROUP_D (1 << 3)
#define GROUP_E (1 << 4)
#define GROUP_F (1 << 5)

#define DAI0_DESTINATION_COUNT ARRAY_SIZE(dai0_destinations)
#define DAI0_SOURCE_COUNT ARRAY_SIZE(dai0_sources)

#define DAI1_DESTINATION_COUNT ARRAY_SIZE(dai1_destinations)
#define DAI1_SOURCE_COUNT ARRAY_SIZE(dai1_sources)

//These numbers must be adjusted to match the device tree binding (if changed).
//They're used to map the device tree integers to the desired list+offset
#define DAI0_DST_DTS_BINDING      0
#define DAI1_DST_DTS_BINDING      140
#define DAI0_SRC_DTS_BINDING      280
#define DAI1_SRC_DTS_BINDING      481
#define DAI1_SRC_DTS_BINDING_END  682

#define NUM_GROUP_COMBINATIONS 16	//adjust as needed

struct dai_destination {
	char *dest_signal;
	u32 width;
	u32 input;
	u32 offset;
	u32 offset_ext;
	u32 group;
};

struct dai_source {
	char *source_signal;
	u32 selection_code;
	u32 group;
};

struct group_combination {
	u32 associated_groups;
	u32 destination_count;
	const char **dest_names;
};

struct adsp_sru_ctrl {
	struct device *dev;
	struct pinctrl_dev *pin_dev;
	void __iomem *regs;
	const char **group_names;
	unsigned int *pins;

	u8 dai;
	u8 has_extended;

	const struct dai_destination *dest;
	u32 dest_count;

	struct group_combination grp_combs[NUM_GROUP_COMBINATIONS];
	u32 grp_combs_cnt;
};

#endif
