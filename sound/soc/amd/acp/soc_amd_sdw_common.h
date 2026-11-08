/* SPDX-License-Identifier: GPL-2.0-only
 *  Copyright (c) 2024 Advanced Micro Devices, Inc. All rights reserved
 */

/*
 *  soc_amd_sdw_common.h - prototypes for common helpers
 */

#ifndef SOC_AMD_SDW_COMMON_H
#define SOC_AMD_SDW_COMMON_H

#include <linux/bits.h>
#include <linux/types.h>
#include <sound/soc.h>
#include <sound/soc_sdw_utils.h>
#include "acp-mach-common.h"

#define ACP63_SDW_MAX_LINKS		2
#define ACP7X_SDW_MAX_LINKS		4

#define AMD_SDW_MAX_GROUPS		9
#define ACP63_PCI_REV			0x63
#define ACP70_PCI_REV			0x70
#define ACP71_PCI_REV			0x71
#define ACP72_PCI_REV			0x72
#define ACP7B_PCI_REV			0x7b
#define ACP7F_PCI_REV			0x7f

#define SOC_JACK_JDSRC(quirk)		((quirk) & GENMASK(3, 0))
#define ASOC_SDW_FOUR_SPK		BIT(4)
/*
 * ASOC_SDW_ACP_DMIC is a generic DMIC-enable flag used across all ACP
 * platforms. On ACP7.B/7.F it additionally identifies the PDM0 controller;
 * ASOC_SDW_ACP_DMIC1 selects PDM1 on ACP7.B/7.F instead. Both ACP7.B/7.F
 * controllers support a 2-channel stereo capture path, and BIOS wires exactly
 * one per platform.
 */
#define ASOC_SDW_ACP_DMIC		BIT(5)	/* enable DMIC; PDM0 on ACP7.B/7.F */
#define ASOC_SDW_ACP_DMIC1		BIT(6)	/* PDM1 / DMIC1, ACP7.B/7.F only */
#define ASOC_SDW_CODEC_SPKR		BIT(15)

#define AMD_SDW0	0
#define AMD_SDW1	1
#define AMD_SDW2	2
#define AMD_SDW3	3
#define ACP63_SW0_AUDIO0_TX	0
#define ACP63_SW0_AUDIO1_TX	1
#define ACP63_SW0_AUDIO2_TX	2

#define ACP63_SW0_AUDIO0_RX	3
#define ACP63_SW0_AUDIO1_RX	4
#define ACP63_SW0_AUDIO2_RX	5

#define ACP63_SW1_AUDIO0_TX	0
#define ACP63_SW1_AUDIO0_RX	1

#define ACP_DMIC_BE_ID		4

#define ACP70_SW_AUDIO0_TX	0
#define ACP70_SW_AUDIO1_TX	1
#define ACP70_SW_AUDIO2_TX	2

#define ACP70_SW_AUDIO0_RX	3
#define ACP70_SW_AUDIO1_RX	4
#define ACP70_SW_AUDIO2_RX	5

/*
 * ACP7.B/7.F SoundWire CPU pin IDs.
 *
 * ACP7x hardware exposes 11 TX and 11 RX data ports per SoundWire
 * instance, each with a dedicated DMA channel hardwired in hardware.
 * The CPU pin IDs use a flat linear numbering shared across all four
 * SoundWire links (SW0-SW3): TX ports are numbered 0-10 and RX ports
 * are numbered 11-21. All port IDs are defined here to represent the
 * ACP7x hardware data port layout. The ID numbering differs from
 * ACP70 due to hardware differences between ACP70 and ACP7x.
 */
#define ACP7X_SW_AUDIO0_TX	0
#define ACP7X_SW_AUDIO1_TX	1
#define ACP7X_SW_AUDIO2_TX	2
#define ACP7X_SW_AUDIO3_TX	3
#define ACP7X_SW_AUDIO4_TX	4
#define ACP7X_SW_AUDIO5_TX	5
#define ACP7X_SW_AUDIO6_TX	6
#define ACP7X_SW_AUDIO7_TX	7
#define ACP7X_SW_AUDIO8_TX	8
#define ACP7X_SW_AUDIO9_TX	9
#define ACP7X_SW_AUDIO10_TX	10

#define ACP7X_SW_AUDIO0_RX	11
#define ACP7X_SW_AUDIO1_RX	12
#define ACP7X_SW_AUDIO2_RX	13
#define ACP7X_SW_AUDIO3_RX	14
#define ACP7X_SW_AUDIO4_RX	15
#define ACP7X_SW_AUDIO5_RX	16
#define ACP7X_SW_AUDIO6_RX	17
#define ACP7X_SW_AUDIO7_RX	18
#define ACP7X_SW_AUDIO8_RX	19
#define ACP7X_SW_AUDIO9_RX	20
#define ACP7X_SW_AUDIO10_RX	21

struct amd_mc_ctx {
	unsigned int acp_rev;
	unsigned int max_sdw_links;
	/* PDM controller selection, copied from amd_pdm_pdata at probe time */
	unsigned int pdm_sel;
};

int get_acp63_cpu_pin_id(u32 sdw_link_id, int be_id, int *cpu_pin_id, struct device *dev);
int get_acp70_cpu_pin_id(u32 sdw_link_id, int be_id, int *cpu_pin_id, struct device *dev);
int get_acp7x_cpu_pin_id(u32 sdw_link_id, int be_id, int *cpu_pin_id, struct device *dev);

#endif
