/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef SOC_ADI_SEC_H
#define SOC_ADI_SEC_H

/* Global Registers */
#define ADI_SEC_REG_GCTL			0x000
#define ADI_SEC_REG_GSTAT			0x004
#define ADI_SEC_REG_RAISE			0x008
#define ADI_SEC_REG_END				0x00c

/* Fault management interface (SFI) registers */
#define ADI_SEC_REG_FCTL			0x010
#define ADI_SEC_REG_FSTAT			0x014
#define ADI_SEC_REG_FSID			0x018
#define ADI_SEC_REG_FEND			0x01c
#define ADI_SEC_REG_FDLY			0x020
#define ADI_SEC_REG_FDLY_CUR			0x024
#define ADI_SEC_REG_FSRDLY			0x028
#define ADI_SEC_REG_FSRDLY_CUR			0x02c
#define ADI_SEC_REG_FCOPP			0x030
#define ADI_SEC_REG_FCOPP_CUR			0x034

/* Start of CCTL registers */
#define ADI_SEC_REG_CCTL_BASE			0x400
#define ADI_SEC_CCTL_SIZE			0x040
#define ADI_SEC_REG_CCTL1			0x440
#define ADI_SEC_REG_CCTL2			0x480

/* Start of SCTL registesr */
#define ADI_SEC_REG_SCTL_BASE			0x800

/* Register bits */
#define ADI_SEC_CCTL_EN				0x00000001	/* SCI Enable */
#define ADI_SEC_SCTL_SRC_EN			0x00000004	/* SEN: Enable */
#define ADI_SEC_SCTL_FAULT_EN       		0x00000002	/* FEN: Enable */
#define ADI_SEC_SCTL_INT_EN         		0x00000001	/* IEN: Enable */
#define ADI_SEC_SCTL_CTG			0x0F000000	/* Core Target Select */

struct adi_sec;

void sec_raise_irq(struct adi_sec *sec, unsigned int irq);
void sec_enable_sci(struct adi_sec *sec, unsigned int coreid);
void sec_enable_ssi(struct adi_sec *sec, unsigned int sid, bool fault,
		    bool source);
void sec_set_ssi_coreid(struct adi_sec *sec, unsigned int sid,
			unsigned int coreid);
struct adi_sec *get_adi_sec_from_node(struct device *dev);
void put_adi_sec(struct adi_sec *sec);
void adi_sec_writel(u32 val, struct adi_sec *rcu, int offset);
u32 adi_sec_readl(struct adi_sec *rcu, int offset);

#endif
