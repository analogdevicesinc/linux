// SPDX-License-Identifier: GPL-2.0+
/*
 * ADRV906X DDR ECC Driver
 * This driver is based on synopsys_edac.c drivers
 *
 * Copyright (c) 2026, Analog Devices Incorporated, All Rights Reserved
 */
#include <linux/arm-smccc.h>
#include <linux/edac.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/sizes.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/wait.h>
#include <linux/kthread.h>

#include "edac_module.h"

/* definition must be in sync with tf-a */
#define PLAT_SIP_SVC_DDR_EDAC   0xC2000004

/* enum definition must be in sync with tf-a */
enum {
	GET_CORRECTABLE_ERROR_INFO = 0,
	GET_UNCORRECTABLE_ERROR_INFO,
	GET_SYNDROME_MASK,
	GET_DTYPE,
	GET_MTYPE,
	GET_ECC_STATE,
	GET_ADDRESS_MAP_OFFSET,
	GET_POISON_CONFIG,
	SET_POISON_CONFIG,
	SET_INJECT_POISON,
	GET_INJECT_POISON,
	SET_PERF_MON_CORE_ENABLED,
	GET_PERF_MON_CORE_ENABLED,
	START_PERF_MON,
	STOP_PERF_MON,
	SET_PERF_MON_CORE_DATA_SEL,
	GET_PERF_MON_CORE_DATA_SEL,
};

#define EDAC_NONE 0
#define EDAC_CE 1
#define EDAC_UE 2
/* Number of cs_rows needed per memory controller */
#define ADRV906X_EDAC_NR_CSROWS         1

/* Number of channels per memory controller */
#define ADRV906X_EDAC_NR_CHANS          1

/* Granularity of reported error in bytes */
#define ADRV906X_EDAC_ERR_GRAIN         1

#define ADRV906X_EDAC_MSG_SIZE          256

#define ADRV906X_EDAC_MOD_STRING        "adrv906x_edac"
#define ADRV906X_EDAC_MOD_VER           "1"

/* Synopsys DDR memory controller registers that are relevant to ECC */
#define CTRL_OFST                       0x0
#define T_ZQ_OFST                       0xA4

/* ECC control register */
#define ECC_CTRL_OFST                   0xC4
/* ECC log register */
#define CE_LOG_OFST                     0xC8
/* ECC address register */
#define CE_ADDR_OFST                    0xCC
/* ECC data[31:0] register */
#define CE_DATA_31_0_OFST               0xD0

/* Uncorrectable error info registers */
#define UE_LOG_OFST                     0xDC
#define UE_ADDR_OFST                    0xE0
#define UE_DATA_31_0_OFST               0xE4

#define STAT_OFST                       0xF0
#define SCRUB_OFST                      0xF4

/* Control register bit field definitions */
#define CTRL_BW_MASK                    0xC
#define CTRL_BW_SHIFT                   2

#define DDRCTL_WDTH_16                  1
#define DDRCTL_WDTH_32                  0

/* ZQ register bit field definitions */
#define T_ZQ_DDRMODE_MASK               0x2

/* ECC control register bit field definitions */
#define ECC_CTRL_CLR_CE_ERR             0x2
#define ECC_CTRL_CLR_UE_ERR             0x1

/* ECC correctable/uncorrectable error log register definitions */
#define LOG_VALID                       0x1
#define CE_LOG_BITPOS_MASK              0xFE
#define CE_LOG_BITPOS_SHIFT             1

/* ECC correctable/uncorrectable error address register definitions */
#define ADDR_COL_MASK                   0xFFF
#define ADDR_ROW_MASK                   0xFFFF000
#define ADDR_ROW_SHIFT                  12
#define ADDR_BANK_MASK                  0x70000000
#define ADDR_BANK_SHIFT                 28

/* ECC statistic register definitions */
#define STAT_UECNT_MASK                 0xFF
#define STAT_CECNT_MASK                 0xFF00
#define STAT_CECNT_SHIFT                8

/* ECC scrub register definitions */
#define SCRUB_MODE_MASK                 0x7
#define SCRUB_MODE_SECDED               0x4

/* DDR ECC Quirks */
#define DDR_ECC_INTR_SUPPORT            BIT(0)
#define DDR_ECC_DATA_POISON_SUPPORT     BIT(1)
#define DDR_ECC_INTR_SELF_CLEAR         BIT(2)

/* ZynqMP Enhanced DDR memory controller registers that are relevant to ECC */
/* ECC Configuration Registers */
#define ECC_CFG0_OFST                   0x70
#define ECC_CFG1_OFST                   0x74

/* ECC Status Register */
#define ECC_STAT_OFST                   0x78

/* ECC Clear Register */
#define ECC_CLR_OFST                    0x7C

/* ECC Error count Register */
#define ECC_ERRCNT_OFST                 0x80

/* ECC Corrected Error Address Register */
#define ECC_CEADDR0_OFST                0x84
#define ECC_CEADDR1_OFST                0x88

/* ECC Syndrome Registers */
#define ECC_CSYND0_OFST                 0x8C
#define ECC_CSYND1_OFST                 0x90
#define ECC_CSYND2_OFST                 0x94

/* ECC Bit Mask0 Address Register */
#define ECC_BITMASK0_OFST               0x98
#define ECC_BITMASK1_OFST               0x9C
#define ECC_BITMASK2_OFST               0xA0

/* ECC UnCorrected Error Address Register */
#define ECC_UEADDR0_OFST                0xA4
#define ECC_UEADDR1_OFST                0xA8

/* ECC Syndrome Registers */
#define ECC_UESYND0_OFST                0xAC
#define ECC_UESYND1_OFST                0xB0
#define ECC_UESYND2_OFST                0xB4

/* ECC Poison Address Reg */
#define ECC_POISON0_OFST                0xB8
#define ECC_POISON1_OFST                0xBC

#define ECC_ADDRMAP0_OFFSET             0x200

/* Control register bitfield definitions */
#define ECC_CTRL_BUSWIDTH_MASK          0x3000
#define ECC_CTRL_BUSWIDTH_SHIFT         12
#define ECC_CTRL_CLR_CE_ERRCNT          BIT(2)
#define ECC_CTRL_CLR_UE_ERRCNT          BIT(3)

/* DDR Control Register width definitions  */
#define QUARTER_DQ_MODE              2
#define HALF_DQ_MODE                 1
#define FULL_DQ_MODE                 0

/* ECC status register definitions */
#define ECC_STAT_UECNT_MASK             0xF0000
#define ECC_STAT_UECNT_SHIFT            16
#define ECC_STAT_CECNT_MASK             0xF00
#define ECC_STAT_CECNT_SHIFT            8
#define ECC_STAT_BITNUM_MASK            0x7F

/* ECC error count register definitions */
#define ECC_ERRCNT_UECNT_MASK           0xFFFF0000
#define ECC_ERRCNT_UECNT_SHIFT          16
#define ECC_ERRCNT_CECNT_MASK           0xFFFF

/* DDR QOS Interrupt register definitions */
#define DDR_QOS_IRQ_STAT_OFST           0x20200
#define DDR_QOSUE_MASK                  0x4
#define DDR_QOSCE_MASK                  0x2
#define ECC_CE_UE_INTR_MASK             0x6
#define DDR_QOS_IRQ_EN_OFST             0x20208
#define DDR_QOS_IRQ_DB_OFST             0x2020C

/* DDR QOS Interrupt register definitions */
#define DDR_UE_MASK                     BIT(9)
#define DDR_CE_MASK                     BIT(8)

/* ECC Corrected Error Register Mask and Shifts*/
#define ECC_CEADDR0_RW_MASK             0x3FFFF
#define ECC_CEADDR0_RNK_MASK            BIT(24)
#define ECC_CEADDR1_BNKGRP_MASK         0x3000000
#define ECC_CEADDR1_BNKNR_MASK          0x70000
#define ECC_CEADDR1_BLKNR_MASK          0xFFF
#define ECC_CEADDR1_BNKGRP_SHIFT        24
#define ECC_CEADDR1_BNKNR_SHIFT         16

/* ECC Poison register shifts */
#define ECC_POISON0_RANK_SHIFT          24
#define ECC_POISON0_RANK_MASK           BIT(24)
#define ECC_POISON0_COLUMN_SHIFT        0
#define ECC_POISON0_COLUMN_MASK         0xFFF
#define ECC_POISON1_BG_SHIFT            28
#define ECC_POISON1_BG_MASK             0x30000000
#define ECC_POISON1_BANKNR_SHIFT        24
#define ECC_POISON1_BANKNR_MASK         0x7000000
#define ECC_POISON1_ROW_SHIFT           0
#define ECC_POISON1_ROW_MASK            0x3FFFF

/* DDR Memory type defines */
#define MEM_TYPE_DDR3                   0x1
#define MEM_TYPE_LPDDR3                 0x8
#define MEM_TYPE_DDR2                   0x4
#define MEM_TYPE_DDR4                   0x10
#define MEM_TYPE_LPDDR4                 0x20

/* DDRC Software control register */
#define DDRC_SWCTL                      0x320

/* DDRC ECC CE & UE poison mask */
#define ECC_CEPOISON_MASK               0x3
#define ECC_UEPOISON_MASK               0x1

/* DDRC Device config masks */
#define DDRC_MSTR_CFG_MASK              0xC0000000
#define DDRC_MSTR_CFG_SHIFT             30
#define DDRC_MSTR_CFG_X4_MASK           0x0
#define DDRC_MSTR_CFG_X8_MASK           0x1
#define DDRC_MSTR_CFG_X16_MASK          0x2
#define DDRC_MSTR_CFG_X32_MASK          0x3

#define DDR_MAX_ROW_SHIFT               18
#define DDR_MAX_COL_SHIFT               14
#define DDR_MAX_BANK_SHIFT              3
#define DDR_MAX_BANKGRP_SHIFT           2

#define ROW_MAX_VAL_MASK                0xF
#define COL_MAX_VAL_MASK                0x1F
#define BANK_MAX_VAL_MASK               0x1F
#define BANKGRP_MAX_VAL_MASK            0x1F
#define RANK_MAX_VAL_MASK               0x1F

#define ROW_B0_BASE                     6
#define ROW_B1_BASE                     7
#define ROW_B2_BASE                     8
#define ROW_B3_BASE                     9
#define ROW_B4_BASE                     10
#define ROW_B5_BASE                     11
#define ROW_B6_BASE                     12
#define ROW_B7_BASE                     13
#define ROW_B8_BASE                     14
#define ROW_B9_BASE                     15
#define ROW_B10_BASE                    16
#define ROW_B11_BASE                    17
#define ROW_B12_BASE                    18
#define ROW_B13_BASE                    19
#define ROW_B14_BASE                    20
#define ROW_B15_BASE                    21
#define ROW_B16_BASE                    22
#define ROW_B17_BASE                    23

#define COL_B2_BASE                     2
#define COL_B3_BASE                     3
#define COL_B4_BASE                     4
#define COL_B5_BASE                     5
#define COL_B6_BASE                     6
#define COL_B7_BASE                     7
#define COL_B8_BASE                     8
#define COL_B9_BASE                     9

#define BANK_B0_BASE                    2
#define BANK_B1_BASE                    3
#define BANK_B2_BASE                    4

#define BANKGRP_B0_BASE                 2
#define BANKGRP_B1_BASE                 3

#define RANK_B0_BASE                    6

#define PERF_MAX_NUM_SIGNAL             6
/**
 * struct ecc_error_info - ECC error log information.
 * @row:	Row number.
 * @col:	Column number.
 * @bank:	Bank number.
 * @bitpos:	Bit position.
 * @data:	Data causing the error.
 * @bankgrpnr:	Bank group number.
 * @blknr:	Block number.
 */
struct ecc_error_info {
	u32 row;
	u32 col;
	u32 bank;
	u32 bitpos;
	u32 data;
	u32 bankgrpnr;
	u32 blknr;
};

/**
 * struct adrv906x_ecc_status - ECC status information to report.
 * @ce_cnt:	Correctable error count.
 * @ue_cnt:	Uncorrectable error count.
 * @ceinfo:	Correctable error log information.
 * @ueinfo:	Uncorrectable error log information.
 */
struct adrv906x_ecc_status {
	u32 ce_cnt;
	u32 ue_cnt;
	struct ecc_error_info ceinfo;
	struct ecc_error_info ueinfo;
};

/**
 * struct adrv906x_edac_priv - DDR memory controller private instance data.
 * @baseaddr:		Base address of the DDR controller.
 * @reglock:		Concurrent CSRs access lock.
 * @message:		Buffer for framing the event specific info.
 * @stat:		ECC status information.
 * @p_data:		Platform data.
 * @ce_cnt:		Correctable Error count.
 * @ue_cnt:		Uncorrectable Error count.
 * @poison_addr:	Data poison address.
 * @row_shift:		Bit shifts for row bit.
 * @col_shift:		Bit shifts for column bit.
 * @bank_shift:		Bit shifts for bank bit.
 * @bankgrp_shift:	Bit shifts for bank group bit.
 * @rank_shift:		Bit shifts for rank bit.
 */
struct adrv906x_edac_priv {
	void __iomem *baseaddr;
	spinlock_t reglock; /* hw access control */
	char message[ADRV906X_EDAC_MSG_SIZE];
	struct adrv906x_ecc_status stat;
	const struct adrv906x_platform_data *p_data;
	u32 ce_cnt;
	u32 ue_cnt;
	wait_queue_head_t edac_wq;
	u16 edac_cmd_flag;
	struct task_struct *edac_task;
	spinlock_t cmd_flag_lock; /* wait queue cmd access control */
#ifdef CONFIG_EDAC_DEBUG
	ulong poison_addr;
	u32 row_shift[18];
	u32 col_shift[14];
	u32 bank_shift[3];
	u32 bankgrp_shift[2];
	u32 rank_shift[1];
#endif
};

/**
 * struct adrv906x_platform_data -  adrv906x platform data structure.
 * @get_error_info:	Get EDAC error info.
 * @get_mtype:		Get mtype.
 * @get_dtype:		Get dtype.
 * @get_ecc_state:	Get ECC state.
 * @get_mem_info:	Get EDAC memory info
 * @quirks:		To differentiate IPs.
 */
struct adrv906x_platform_data {
	int (*get_error_info)(struct adrv906x_edac_priv *priv);
	enum mem_type (*get_mtype)(const void __iomem *base);
	enum dev_type (*get_dtype)(const void __iomem *base);
	bool (*get_ecc_state)(void __iomem *base);
#ifdef CONFIG_EDAC_DEBUG
	u64 (*get_mem_info)(struct adrv906x_edac_priv *priv);
#endif
	int quirks;
};

/**
 * adrv906x_smc_call - wrapper for smc call
 * check for error, place to add debug info for development
 */
static void adrv906x_smc_call(struct arm_smccc_1_2_regs *args, struct arm_smccc_1_2_regs *res)
{
	arm_smccc_1_2_smc(args, res);

	if (res->a0 != SMCCC_RET_SUCCESS)
		edac_printk(KERN_ERR, EDAC_MC, "smc call failed. cmd 0x%lx, error 0x%lx\n",
			    args->a1, res->a0);
}

/**
 * adrv906x_get_error_info - Get the current ECC error info.
 * @priv:	DDR memory controller private instance data.
 *
 * Return: one if there is no error otherwise returns zero.
 */
static int adrv906x_get_error_info(struct adrv906x_edac_priv *priv)
{
	struct arm_smccc_1_2_regs args, res;
	struct adrv906x_ecc_status *p;

	p = &priv->stat;

	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = GET_CORRECTABLE_ERROR_INFO;
	adrv906x_smc_call(&args, &res);

	p->ce_cnt = res.a1;
	if (!p->ce_cnt)
		goto ue_err;

	p->ceinfo.row = FIELD_GET(GENMASK(17, 0), res.a2);
	p->ceinfo.col = FIELD_GET(GENMASK(27, 18), res.a2);
	p->ceinfo.bank = FIELD_GET(GENMASK(29, 28), res.a2);
	p->ceinfo.bankgrpnr = FIELD_GET(GENMASK(31, 30), res.a2);
	p->ceinfo.bitpos = res.a3;
	p->ceinfo.data = res.a4;
	p->ceinfo.blknr = p->ceinfo.col;

ue_err:
	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = GET_UNCORRECTABLE_ERROR_INFO;
	adrv906x_smc_call(&args, &res);
	p->ue_cnt = res.a1;

	if (!p->ue_cnt)
		goto out;

	p->ceinfo.row = FIELD_GET(GENMASK(17, 0), res.a2);
	p->ceinfo.col = FIELD_GET(GENMASK(27, 18), res.a2);
	p->ceinfo.bank = FIELD_GET(GENMASK(29, 28), res.a2);
	p->ceinfo.bankgrpnr = FIELD_GET(GENMASK(31, 30), res.a2);
	p->ceinfo.bitpos = res.a3;
	p->ceinfo.data = res.a4;
	p->ceinfo.blknr = p->ceinfo.col;

out:
	/* get syndrome mask */
	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = GET_SYNDROME_MASK;
	adrv906x_smc_call(&args, &res);
	edac_printk(KERN_ERR, EDAC_MC, "Syndrome #0 0x%lx\n", res.a1);
	edac_printk(KERN_ERR, EDAC_MC, "Syndrome #1 0x%lx\n", res.a2);
	edac_printk(KERN_ERR, EDAC_MC, "Syndrome #2 0x%lx\n", res.a3);
	return 0;
}

/**
 * handle_error - Handle Correctable and Uncorrectable errors.
 * @mci:	EDAC memory controller instance.
 * @p:		Synopsys ECC status structure.
 *
 * Handles ECC correctable and uncorrectable errors.
 */
static void handle_error(struct mem_ctl_info *mci, struct adrv906x_ecc_status *p)
{
	struct adrv906x_edac_priv *priv = mci->pvt_info;
	struct ecc_error_info *pinf;

	if (p->ce_cnt) {
		pinf = &p->ceinfo;
		if (priv->p_data->quirks & DDR_ECC_INTR_SUPPORT) {
			snprintf(priv->message, ADRV906X_EDAC_MSG_SIZE,
				 "DDR ECC error type:%s Row %d Bank %d BankGroup Number %d Block Number %d Bit Position: %d Data: 0x%08x",
				 "CE", pinf->row, pinf->bank,
				 pinf->bankgrpnr, pinf->blknr,
				 pinf->bitpos, pinf->data);
		} else {
			snprintf(priv->message, ADRV906X_EDAC_MSG_SIZE,
				 "DDR ECC error type:%s Row %d Bank %d Col %d Bit Position: %d Data: 0x%08x",
				 "CE", pinf->row, pinf->bank, pinf->col,
				 pinf->bitpos, pinf->data);
		}

		edac_mc_handle_error(HW_EVENT_ERR_CORRECTED, mci,
				     p->ce_cnt, 0, 0, 0, 0, 0, -1,
				     priv->message, "");
	}

	if (p->ue_cnt) {
		pinf = &p->ueinfo;
		if (priv->p_data->quirks & DDR_ECC_INTR_SUPPORT) {
			snprintf(priv->message, ADRV906X_EDAC_MSG_SIZE,
				 "DDR ECC error type :%s Row %d Bank %d BankGroup Number %d Block Number %d",
				 "UE", pinf->row, pinf->bank,
				 pinf->bankgrpnr, pinf->blknr);
		} else {
			snprintf(priv->message, ADRV906X_EDAC_MSG_SIZE,
				 "DDR ECC error type :%s Row %d Bank %d Col %d ",
				 "UE", pinf->row, pinf->bank, pinf->col);
		}

		edac_mc_handle_error(HW_EVENT_ERR_UNCORRECTED, mci,
				     p->ue_cnt, 0, 0, 0, 0, 0, -1,
				     priv->message, "");
	}

	memset(p, 0, sizeof(*p));
}

/**
 * intr_handler_* - Interrupt Handler for ECC interrupts.
 * @irq:        IRQ number.
 * @dev_id:     Device ID.
 *
 * Return: IRQ_NONE, if interrupt not set or IRQ_HANDLED otherwise.
 */
static irqreturn_t intr_handler_ce(int irq, void *dev_id)
{
	struct mem_ctl_info *mci = dev_id;
	struct adrv906x_edac_priv *priv;
	unsigned long flags;

	priv = mci->pvt_info;
	spin_lock_irqsave(&priv->cmd_flag_lock, flags);
	priv->edac_cmd_flag |= EDAC_CE;
	spin_unlock_irqrestore(&priv->cmd_flag_lock, flags);
	wake_up_interruptible(&priv->edac_wq);

	return IRQ_HANDLED;
}

static irqreturn_t intr_handler_ue(int irq, void *dev_id)
{
	struct mem_ctl_info *mci = dev_id;
	struct adrv906x_edac_priv *priv;
	unsigned long flags;

	priv = mci->pvt_info;
	spin_lock_irqsave(&priv->cmd_flag_lock, flags);
	priv->edac_cmd_flag |= EDAC_UE;
	spin_unlock_irqrestore(&priv->cmd_flag_lock, flags);
	wake_up_interruptible(&priv->edac_wq);

	return IRQ_HANDLED;
}

/**
 * check_errors - Check controller for ECC errors.
 * @mci:	EDAC memory controller instance.
 *
 * Check and post ECC errors. Called by the polling thread.
 */
static void check_errors(struct mem_ctl_info *mci)
{
	const struct adrv906x_platform_data *p_data;
	struct adrv906x_edac_priv *priv;
	int status;

	priv = mci->pvt_info;
	p_data = priv->p_data;

	status = p_data->get_error_info(priv);
	if (status)
		return;

	priv->ce_cnt += priv->stat.ce_cnt;
	priv->ue_cnt += priv->stat.ue_cnt;
	handle_error(mci, &priv->stat);

	edac_dbg(3, "Total error count CE %d UE %d\n",
		 priv->ce_cnt, priv->ue_cnt);
}

/**
 * adrv906x_get_dtype - Return the controller memory width.
 * @base:	DDR memory controller base address.
 *
 * Get the EDAC device type width appropriate for the current controller
 * configuration.
 *
 * Return: a device type width enumeration.
 */
static enum dev_type adrv906x_get_dtype(const void __iomem *base)
{
	struct arm_smccc_1_2_regs args, res;

	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = GET_DTYPE;
	adrv906x_smc_call(&args, &res);

	return res.a1;
}

/**
 * adrv906x_get_ecc_state - Return the controller ECC enable/disable status.
 * @base:	DDR memory controller base address.
 *
 * Get the ECC enable/disable status for the controller.
 *
 * Return: a ECC status boolean i.e true/false - enabled/disabled.
 */
static bool adrv906x_get_ecc_state(void __iomem *base)
{
	struct arm_smccc_1_2_regs args, res;

	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = GET_ECC_STATE;
	adrv906x_smc_call(&args, &res);

	return res.a1;
}

/**
 * get_memsize - Read the size of the attached memory device.
 *
 * Return: the memory size in bytes.
 */
static u32 get_memsize(void)
{
	struct sysinfo inf;

	si_meminfo(&inf);

	return inf.totalram * inf.mem_unit;
}

/**
 * adrv906x_get_mtype - Returns controller memory type.
 * @base:	Synopsys ECC status structure.
 *
 * Get the EDAC memory type appropriate for the current controller
 * configuration.
 *
 * Return: a memory type enumeration.
 */
static enum mem_type adrv906x_get_mtype(const void __iomem *base)
{
	struct arm_smccc_1_2_regs args, res;

	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = GET_MTYPE;
	adrv906x_smc_call(&args, &res);

	return res.a1;
}

/**
 * init_csrows - Initialize the csrow data.
 * @mci:	EDAC memory controller instance.
 *
 * Initialize the chip select rows associated with the EDAC memory
 * controller instance.
 */
static void init_csrows(struct mem_ctl_info *mci)
{
	struct adrv906x_edac_priv *priv = mci->pvt_info;
	const struct adrv906x_platform_data *p_data;
	struct csrow_info *csi;
	struct dimm_info *dimm;
	u32 size, row;
	int j;

	p_data = priv->p_data;

	for (row = 0; row < mci->nr_csrows; row++) {
		csi = mci->csrows[row];
		size = get_memsize();

		for (j = 0; j < csi->nr_channels; j++) {
			dimm = csi->channels[j]->dimm;
			dimm->edac_mode = EDAC_SECDED;
			dimm->mtype = p_data->get_mtype(priv->baseaddr);
			dimm->nr_pages = (size >> PAGE_SHIFT) / csi->nr_channels;
			dimm->grain = ADRV906X_EDAC_ERR_GRAIN;
			dimm->dtype = p_data->get_dtype(priv->baseaddr);
		}
	}
}

/**
 * mc_init - Initialize one driver instance.
 * @mci:	EDAC memory controller instance.
 * @pdev:	platform device.
 *
 * Perform initialization of the EDAC memory controller instance and
 * related driver-private data associated with the memory controller the
 * instance is bound to.
 */
static void mc_init(struct mem_ctl_info *mci, struct platform_device *pdev)
{
	struct adrv906x_edac_priv *priv;

	mci->pdev = &pdev->dev;
	priv = mci->pvt_info;
	platform_set_drvdata(pdev, mci);

	/* Initialize controller capabilities and configuration */
	mci->mtype_cap = MEM_FLAG_DDR3 | MEM_FLAG_DDR2;
	mci->edac_ctl_cap = EDAC_FLAG_NONE | EDAC_FLAG_SECDED;
	mci->scrub_cap = SCRUB_HW_SRC;
	mci->scrub_mode = SCRUB_NONE;

	mci->edac_cap = EDAC_FLAG_SECDED;
	mci->ctl_name = "adrv906x_ddr_controller";
	mci->dev_name = ADRV906X_EDAC_MOD_STRING;
	mci->mod_name = ADRV906X_EDAC_MOD_VER;

	if (priv->p_data->quirks & DDR_ECC_INTR_SUPPORT) {
		edac_op_state = EDAC_OPSTATE_INT;
	} else {
		edac_op_state = EDAC_OPSTATE_POLL;
		mci->edac_check = check_errors;
	}

	mci->ctl_page_to_phys = NULL;

	init_csrows(mci);
}

static int setup_irq(struct mem_ctl_info *mci,
		     struct platform_device *pdev)
{
	int ret, irq;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		edac_printk(KERN_ERR, EDAC_MC,
			    "No IRQ CE %d in DT\n", irq);
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, intr_handler_ce,
			       0, dev_name(&pdev->dev), mci);
	if (ret < 0) {
		edac_printk(KERN_ERR, EDAC_MC, "Failed to request IRQ%d\n", irq);
		return ret;
	}

	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		edac_printk(KERN_ERR, EDAC_MC,
			    "No IRQ UE %d in DT\n", irq);
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, intr_handler_ue,
			       0, dev_name(&pdev->dev), mci);
	if (ret < 0) {
		edac_printk(KERN_ERR, EDAC_MC, "Failed to request IRQ%d\n", irq);
		return ret;
	}

	return 0;
}

static const struct adrv906x_platform_data adrv906x_edac_def = {
	.get_error_info = adrv906x_get_error_info,
	.get_mtype	= adrv906x_get_mtype,
	.get_dtype	= adrv906x_get_dtype,
	.get_ecc_state	= adrv906x_get_ecc_state,
	.quirks		= (DDR_ECC_INTR_SUPPORT | DDR_ECC_INTR_SELF_CLEAR
#ifdef CONFIG_EDAC_DEBUG
			   | DDR_ECC_DATA_POISON_SUPPORT
#endif
			   ),
};

static const struct of_device_id adrv906x_edac_match[] = {
	{
		.compatible = "adi,adrv906x-ddrc",
		.data = (void *)&adrv906x_edac_def
	},
	{
		/* end of table */
	}
};

MODULE_DEVICE_TABLE(of, adrv906x_edac_match);

#ifdef CONFIG_EDAC_DEBUG
#define to_mci(k) container_of(k, struct mem_ctl_info, dev)

/**
 * ddr_poison_setup -	Update poison registers.
 * @priv:		DDR memory controller private instance data.
 *
 * Update poison registers as per DDR mapping.
 * Return: none.
 */
static int ddr_poison_setup(struct adrv906x_edac_priv *priv)
{
	int col = 0, row = 0, bank = 0, bankgrp = 0, rank = 0;
	const struct adrv906x_platform_data *p_data;
	struct arm_smccc_1_2_regs args, res;
	int index;
	ulong hif_addr = 0;

	p_data = priv->p_data;

	if (priv->poison_addr >= 0x40000000)
		hif_addr = (priv->poison_addr - 0x40000000) >> 1;
	else
		return -EINVAL;

	for (index = 0; index < DDR_MAX_ROW_SHIFT; index++) {
		if (priv->row_shift[index])
			row |= (((hif_addr >> priv->row_shift[index]) &
				 BIT(0)) << index);
		else
			break;
	}

	for (index = 0; index < DDR_MAX_COL_SHIFT; index++) {
		if (priv->col_shift[index] || index < 3)
			col |= (((hif_addr >> priv->col_shift[index]) &
				 BIT(0)) << index);
		else
			break;
	}

	for (index = 0; index < DDR_MAX_BANK_SHIFT; index++) {
		if (priv->bank_shift[index])
			bank |= (((hif_addr >> priv->bank_shift[index]) &
				  BIT(0)) << index);
		else
			break;
	}

	for (index = 0; index < DDR_MAX_BANKGRP_SHIFT; index++) {
		if (priv->bankgrp_shift[index])
			bankgrp |= (((hif_addr >> priv->bankgrp_shift[index])
				     & BIT(0)) << index);
		else
			break;
	}

	if (priv->rank_shift[0])
		rank = (hif_addr >> priv->rank_shift[0]) & BIT(0);

	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = SET_INJECT_POISON;
	args.a2 = (rank << 16) | col;
	args.a3 = (bankgrp << 16) | bank;
	args.a4 = row;

	adrv906x_smc_call(&args, &res);

	return 0;
}

static ssize_t inject_data_error_show(struct device *dev,
				      struct device_attribute *mattr,
				      char *data)
{
	struct mem_ctl_info *mci = to_mci(dev);
	struct adrv906x_edac_priv *priv = mci->pvt_info;
	struct arm_smccc_1_2_regs args, res;

	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = GET_INJECT_POISON;

	adrv906x_smc_call(&args, &res);

	return sprintf(data, "Poison0 Addr: 0x%lx\n\rPoison1 Addr: 0x%lx\n\r"
		       "Error injection Address: 0x%lx\n\r", res.a1, res.a2,
		       priv->poison_addr);
}

static ssize_t inject_data_error_store(struct device *dev,
				       struct device_attribute *mattr,
				       const char *data, size_t count)
{
	struct mem_ctl_info *mci = to_mci(dev);
	struct adrv906x_edac_priv *priv = mci->pvt_info;

	if (kstrtoul(data, 0, &priv->poison_addr))
		return -EINVAL;

	if (ddr_poison_setup(priv))
		return -EINVAL;

	return count;
}

static ssize_t inject_data_poison_show(struct device *dev,
				       struct device_attribute *mattr,
				       char *data)
{
	struct arm_smccc_1_2_regs args, res;

	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = GET_POISON_CONFIG;
	adrv906x_smc_call(&args, &res);

	/* a1 is poison enabled, we don't use it */
	return sprintf(data, "Data Poisoning: %s\n\r",
		       res.a2 ? ("Correctable Error") : ("UnCorrectable Error"));
}

static ssize_t inject_data_poison_store(struct device *dev,
					struct device_attribute *mattr,
					const char *data, size_t count)
{
	struct arm_smccc_1_2_regs args, res;

	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = SET_POISON_CONFIG;

	args.a2 = true;
	if (strncmp(data, "UE", 2) == 0)
		args.a3 = 1;
	else
		args.a3 = 0;

	adrv906x_smc_call(&args, &res);

	return count;
}

static int get_attr_cmd_tokens(char *inpstr, char *tokens[], u8 tokens_needed)
{
	char *token, *cmdstr;
	int i;

	cmdstr = inpstr;

	for (i = 0 ; i < tokens_needed ; i++) {
		do {
			token = strsep(&cmdstr, " ");
			if (!token)
				return -EINVAL;
		} while (!strlen(token) && cmdstr[0] != '\0');

		if (!strlen(token))
			return -EINVAL;

		tokens[i] = token;
	}

	return 0;
}

static ssize_t perf_mon_core_enable_show(struct device *dev,
					 struct device_attribute *mattr,
					 char *data)
{
	struct arm_smccc_1_2_regs args, res;
	int i;
	char tmpstr[128], finalstr[512];

	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = GET_PERF_MON_CORE_ENABLED;
	for (i = 0; i < PERF_MAX_NUM_SIGNAL; i++) {
		args.a2 = i;
		adrv906x_smc_call(&args, &res);
		sprintf(tmpstr, "signal %d: enable flag: %d\n", i, (int)res.a1);
		strcat(finalstr, tmpstr);
	}

	return sprintf(data, "%s", finalstr);
}

static ssize_t perf_mon_core_enable_store(struct device *dev,
					  struct device_attribute *mattr,
					  const char *data, size_t count)
{
	struct arm_smccc_1_2_regs args, res;
	char *cmdstr, *orig;
	char *tokens[2];
	u16 signal, enable;
	int ret;

	cmdstr = kmalloc(count + 1, GFP_KERNEL);
	if (!cmdstr)
		return -ENOMEM;
	orig = cmdstr;
	strscpy(cmdstr, data, count + 1);

	ret = get_attr_cmd_tokens(cmdstr, tokens, 2);
	if (ret)
		goto free_m;

	ret = kstrtou16(tokens[0], 10, &signal);
	if (ret)
		goto free_m;

	if (signal > PERF_MAX_NUM_SIGNAL) {
		ret = -EINVAL;
		goto free_m;
	}

	ret = kstrtou16(tokens[1], 10, &enable);
	if (ret)
		goto free_m;

	ret = count;

	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = SET_PERF_MON_CORE_ENABLED;
	args.a2 = signal;
	args.a3 = enable;
	adrv906x_smc_call(&args, &res);

free_m:
	kfree(orig);

	return ret;
}

static ssize_t perf_mon_core_data_sel_show(struct device *dev,
					   struct device_attribute *mattr,
					   char *data)
{
	struct arm_smccc_1_2_regs args, res;
	int i;
	char tmpstr[128], finalstr[512];

	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = GET_PERF_MON_CORE_DATA_SEL;
	for (i = 0; i < PERF_MAX_NUM_SIGNAL; i++) {
		args.a2 = i;
		adrv906x_smc_call(&args, &res);
		sprintf(tmpstr, "signal %d: data select: 0x%x\n", i, (int)res.a1);
		strcat(finalstr, tmpstr);
	}

	return sprintf(data, "%s", finalstr);
}

static ssize_t perf_mon_core_data_sel_store(struct device *dev,
					    struct device_attribute *mattr,
					    const char *data, size_t count)
{
	struct arm_smccc_1_2_regs args, res;
	char *cmdstr, *orig;
	char *tokens[2];
	u32 signal, select;
	int ret;

	cmdstr = kmalloc(count + 1, GFP_KERNEL);
	if (!cmdstr)
		return -ENOMEM;
	orig = cmdstr;
	strscpy(cmdstr, data, count + 1);

	ret = get_attr_cmd_tokens(cmdstr, tokens, 2);
	if (ret)
		goto free_m;

	ret = kstrtou32(tokens[0], 10, &signal);
	if (ret)
		goto free_m;

	if (signal > PERF_MAX_NUM_SIGNAL) {
		ret = -EINVAL;
		goto free_m;
	}

	ret = kstrtou32(tokens[1], 10, &select);
	if (ret)
		goto free_m;

	ret = count;

	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = SET_PERF_MON_CORE_DATA_SEL;
	args.a2 = signal;
	args.a3 = select;
	adrv906x_smc_call(&args, &res);

free_m:
	kfree(orig);

	return ret;
}

static ssize_t perf_monitor_data_show(struct device *dev,
				      struct device_attribute *mattr,
				      char *data)
{
	struct arm_smccc_1_2_regs args, res;
	char tmpstr[128], finalstr[512];

	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = STOP_PERF_MON;
	adrv906x_smc_call(&args, &res);
	sprintf(tmpstr, "signal 0: event counter: 0x%lx\n", res.a1);
	strcat(finalstr, tmpstr);
	sprintf(tmpstr, "signal 1: event counter: 0x%lx\n", res.a2);
	strcat(finalstr, tmpstr);
	sprintf(tmpstr, "signal 2: event counter: 0x%lx\n", res.a3);
	strcat(finalstr, tmpstr);
	sprintf(tmpstr, "signal 3: event counter: 0x%lx\n", res.a4);
	strcat(finalstr, tmpstr);
	sprintf(tmpstr, "signal 4: event counter: 0x%lx\n", res.a5);
	strcat(finalstr, tmpstr);
	sprintf(tmpstr, "signal 5: event counter: 0x%lx\n", res.a6);
	strcat(finalstr, tmpstr);

	return sprintf(data, "%s", finalstr);
}

static ssize_t perf_monitor_data_store(struct device *dev,
				       struct device_attribute *mattr,
				       const char *data, size_t count)
{
	struct arm_smccc_1_2_regs args, res;

	args.a0 = PLAT_SIP_SVC_DDR_EDAC;
	args.a1 = START_PERF_MON;
	adrv906x_smc_call(&args, &res);

	return count;
}

static DEVICE_ATTR_RW(inject_data_error);
static DEVICE_ATTR_RW(inject_data_poison);
static DEVICE_ATTR_RW(perf_mon_core_enable);
static DEVICE_ATTR_RW(perf_mon_core_data_sel);
static DEVICE_ATTR_RW(perf_monitor_data);

static int edac_create_sysfs_attributes(struct mem_ctl_info *mci)
{
	int rc;

	rc = device_create_file(&mci->dev, &dev_attr_inject_data_error);
	if (rc < 0)
		return rc;
	rc = device_create_file(&mci->dev, &dev_attr_inject_data_poison);
	if (rc < 0)
		return rc;
	rc = device_create_file(&mci->dev, &dev_attr_perf_mon_core_enable);
	if (rc < 0)
		return rc;
	rc = device_create_file(&mci->dev, &dev_attr_perf_mon_core_data_sel);
	if (rc < 0)
		return rc;
	rc = device_create_file(&mci->dev, &dev_attr_perf_monitor_data);
	if (rc < 0)
		return rc;
	return 0;
}

static void edac_remove_sysfs_attributes(struct mem_ctl_info *mci)
{
	device_remove_file(&mci->dev, &dev_attr_inject_data_error);
	device_remove_file(&mci->dev, &dev_attr_inject_data_poison);
	device_remove_file(&mci->dev, &dev_attr_perf_mon_core_enable);
	device_remove_file(&mci->dev, &dev_attr_perf_mon_core_data_sel);
	device_remove_file(&mci->dev, &dev_attr_perf_monitor_data);
}

static void setup_row_address_map(struct adrv906x_edac_priv *priv, u32 *addrmap)
{
	u32 addrmap_row_b2_10;
	int index;

	priv->row_shift[0] = (addrmap[5] & ROW_MAX_VAL_MASK) + ROW_B0_BASE;
	priv->row_shift[1] = ((addrmap[5] >> 8) &
			      ROW_MAX_VAL_MASK) + ROW_B1_BASE;

	addrmap_row_b2_10 = (addrmap[5] >> 16) & ROW_MAX_VAL_MASK;
	if (addrmap_row_b2_10 != ROW_MAX_VAL_MASK) {
		for (index = 2; index < 11; index++)
			priv->row_shift[index] = addrmap_row_b2_10 +
						 index + ROW_B0_BASE;
	} else {
		priv->row_shift[2] = (addrmap[9] &
				      ROW_MAX_VAL_MASK) + ROW_B2_BASE;
		priv->row_shift[3] = ((addrmap[9] >> 8) &
				      ROW_MAX_VAL_MASK) + ROW_B3_BASE;
		priv->row_shift[4] = ((addrmap[9] >> 16) &
				      ROW_MAX_VAL_MASK) + ROW_B4_BASE;
		priv->row_shift[5] = ((addrmap[9] >> 24) &
				      ROW_MAX_VAL_MASK) + ROW_B5_BASE;
		priv->row_shift[6] = (addrmap[10] &
				      ROW_MAX_VAL_MASK) + ROW_B6_BASE;
		priv->row_shift[7] = ((addrmap[10] >> 8) &
				      ROW_MAX_VAL_MASK) + ROW_B7_BASE;
		priv->row_shift[8] = ((addrmap[10] >> 16) &
				      ROW_MAX_VAL_MASK) + ROW_B8_BASE;
		priv->row_shift[9] = ((addrmap[10] >> 24) &
				      ROW_MAX_VAL_MASK) + ROW_B9_BASE;
		priv->row_shift[10] = (addrmap[11] &
				       ROW_MAX_VAL_MASK) + ROW_B10_BASE;
	}

	priv->row_shift[11] = (((addrmap[5] >> 24) & ROW_MAX_VAL_MASK) ==
			       ROW_MAX_VAL_MASK) ? 0 : (((addrmap[5] >> 24) &
							 ROW_MAX_VAL_MASK) + ROW_B11_BASE);
	priv->row_shift[12] = ((addrmap[6] & ROW_MAX_VAL_MASK) ==
			       ROW_MAX_VAL_MASK) ? 0 : ((addrmap[6] &
							 ROW_MAX_VAL_MASK) + ROW_B12_BASE);
	priv->row_shift[13] = (((addrmap[6] >> 8) & ROW_MAX_VAL_MASK) ==
			       ROW_MAX_VAL_MASK) ? 0 : (((addrmap[6] >> 8) &
							 ROW_MAX_VAL_MASK) + ROW_B13_BASE);
	priv->row_shift[14] = (((addrmap[6] >> 16) & ROW_MAX_VAL_MASK) ==
			       ROW_MAX_VAL_MASK) ? 0 : (((addrmap[6] >> 16) &
							 ROW_MAX_VAL_MASK) + ROW_B14_BASE);
	priv->row_shift[15] = (((addrmap[6] >> 24) & ROW_MAX_VAL_MASK) ==
			       ROW_MAX_VAL_MASK) ? 0 : (((addrmap[6] >> 24) &
							 ROW_MAX_VAL_MASK) + ROW_B15_BASE);
	priv->row_shift[16] = ((addrmap[7] & ROW_MAX_VAL_MASK) ==
			       ROW_MAX_VAL_MASK) ? 0 : ((addrmap[7] &
							 ROW_MAX_VAL_MASK) + ROW_B16_BASE);
	priv->row_shift[17] = (((addrmap[7] >> 8) & ROW_MAX_VAL_MASK) ==
			       ROW_MAX_VAL_MASK) ? 0 : (((addrmap[7] >> 8) &
							 ROW_MAX_VAL_MASK) + ROW_B17_BASE);
}

static void setup_column_address_map(struct adrv906x_edac_priv *priv, u32 *addrmap)
{
	priv->col_shift[0] = 0;
	priv->col_shift[1] = 1;
	priv->col_shift[2] = (addrmap[2] & COL_MAX_VAL_MASK) + COL_B2_BASE;
	priv->col_shift[3] = ((addrmap[2] >> 8) &
			      COL_MAX_VAL_MASK) + COL_B3_BASE;
	priv->col_shift[4] = (((addrmap[2] >> 16) & COL_MAX_VAL_MASK) ==
			      COL_MAX_VAL_MASK) ? 0 : (((addrmap[2] >> 16) &
							COL_MAX_VAL_MASK) + COL_B4_BASE);
	priv->col_shift[5] = (((addrmap[2] >> 24) & COL_MAX_VAL_MASK) ==
			      COL_MAX_VAL_MASK) ? 0 : (((addrmap[2] >> 24) &
							COL_MAX_VAL_MASK) + COL_B5_BASE);
	priv->col_shift[6] = ((addrmap[3] & COL_MAX_VAL_MASK) ==
			      COL_MAX_VAL_MASK) ? 0 : ((addrmap[3] &
							COL_MAX_VAL_MASK) + COL_B6_BASE);
	priv->col_shift[7] = (((addrmap[3] >> 8) & COL_MAX_VAL_MASK) ==
			      COL_MAX_VAL_MASK) ? 0 : (((addrmap[3] >> 8) &
							COL_MAX_VAL_MASK) + COL_B7_BASE);
	priv->col_shift[8] = (((addrmap[3] >> 16) & COL_MAX_VAL_MASK) ==
			      COL_MAX_VAL_MASK) ? 0 : (((addrmap[3] >> 16) &
							COL_MAX_VAL_MASK) + COL_B8_BASE);
	priv->col_shift[9] = (((addrmap[3] >> 24) & COL_MAX_VAL_MASK) ==
			      COL_MAX_VAL_MASK) ? 0 : (((addrmap[3] >> 24) &
							COL_MAX_VAL_MASK) + COL_B9_BASE);
}

static void setup_bank_address_map(struct adrv906x_edac_priv *priv, u32 *addrmap)
{
	priv->bank_shift[0] = (addrmap[1] & BANK_MAX_VAL_MASK) + BANK_B0_BASE;
	priv->bank_shift[1] = ((addrmap[1] >> 8) &
			       BANK_MAX_VAL_MASK) + BANK_B1_BASE;
	priv->bank_shift[2] = (((addrmap[1] >> 16) &
				BANK_MAX_VAL_MASK) == BANK_MAX_VAL_MASK) ? 0 :
			      (((addrmap[1] >> 16) & BANK_MAX_VAL_MASK) +
			       BANK_B2_BASE);
}

static void setup_bg_address_map(struct adrv906x_edac_priv *priv, u32 *addrmap)
{
	priv->bankgrp_shift[0] = (addrmap[8] &
				  BANKGRP_MAX_VAL_MASK) + BANKGRP_B0_BASE;
	priv->bankgrp_shift[1] = (((addrmap[8] >> 8) & BANKGRP_MAX_VAL_MASK) ==
				  BANKGRP_MAX_VAL_MASK) ? 0 : (((addrmap[8] >> 8)
						& BANKGRP_MAX_VAL_MASK) + BANKGRP_B1_BASE);
}

static void setup_rank_address_map(struct adrv906x_edac_priv *priv, u32 *addrmap)
{
	priv->rank_shift[0] = ((addrmap[0] & RANK_MAX_VAL_MASK) ==
			       RANK_MAX_VAL_MASK) ? 0 : ((addrmap[0] &
							  RANK_MAX_VAL_MASK) + RANK_B0_BASE);
}

/*
 * setup_address_map -	Set Address Map by querying ADDRMAP registers.
 * @priv:		DDR memory controller private instance data.
 *
 * Set Address Map by querying ADDRMAP registers.
 *
 * Return: none.
 */
static void setup_address_map(struct adrv906x_edac_priv *priv)
{
	struct arm_smccc_1_2_regs args, res;
	u32 addrmap[12];
	int index;

	for (index = 0; index < 12; index++) {
		args.a0 = PLAT_SIP_SVC_DDR_EDAC;
		args.a1 = GET_ADDRESS_MAP_OFFSET;
		args.a2 = index;
		adrv906x_smc_call(&args, &res);
		addrmap[index] = res.a1;
	}

	setup_row_address_map(priv, addrmap);

	setup_column_address_map(priv, addrmap);

	setup_bank_address_map(priv, addrmap);

	setup_bg_address_map(priv, addrmap);

	setup_rank_address_map(priv, addrmap);
}
#endif /* CONFIG_EDAC_DEBUG */

static int adrv906x_edac_thread(void *data)
{
	struct mem_ctl_info *mci = (struct mem_ctl_info *)data;
	struct adrv906x_edac_priv *priv = mci->pvt_info;
	const struct adrv906x_platform_data *p_data;
	unsigned long flags;
	int status;

	p_data = priv->p_data;

	while (!kthread_should_stop()) {
		wait_event_interruptible(priv->edac_wq, priv->edac_cmd_flag != EDAC_NONE);

		spin_lock_irqsave(&priv->cmd_flag_lock, flags);
		priv->edac_cmd_flag = EDAC_NONE;
		spin_unlock_irqrestore(&priv->cmd_flag_lock, flags);
		status = p_data->get_error_info(priv);
		if (status)
			edac_printk(KERN_ERR, EDAC_MC, "get_error_info failed %d\n", status);

		priv->ce_cnt += priv->stat.ce_cnt;
		priv->ue_cnt += priv->stat.ue_cnt;
		handle_error(mci, &priv->stat);
	}

	return 0;
}

/**
 * mc_probe - Check controller and bind driver.
 * @pdev:	platform device.
 *
 * Probe a specific controller instance for binding with the driver.
 *
 * Return: 0 if the controller instance was successfully bound to the
 * driver; otherwise, < 0 on error.
 */
static int mc_probe(struct platform_device *pdev)
{
	const struct adrv906x_platform_data *p_data;
	struct edac_mc_layer layers[2];
	struct adrv906x_edac_priv *priv;
	struct mem_ctl_info *mci;
	void __iomem *baseaddr = 0;
	int rc;

	p_data = of_device_get_match_data(&pdev->dev);
	if (!p_data)
		return -ENODEV;

	if (!p_data->get_ecc_state(baseaddr)) {
		edac_printk(KERN_INFO, EDAC_MC, "ECC not enabled\n");
		return -ENXIO;
	}

	layers[0].type = EDAC_MC_LAYER_CHIP_SELECT;
	layers[0].size = ADRV906X_EDAC_NR_CSROWS;
	layers[0].is_virt_csrow = true;
	layers[1].type = EDAC_MC_LAYER_CHANNEL;
	layers[1].size = ADRV906X_EDAC_NR_CHANS;
	layers[1].is_virt_csrow = false;

	mci = edac_mc_alloc(0, ARRAY_SIZE(layers), layers,
			    sizeof(struct adrv906x_edac_priv));
	if (!mci) {
		edac_printk(KERN_ERR, EDAC_MC,
			    "Failed memory allocation for mc instance\n");
		return -ENOMEM;
	}

	priv = mci->pvt_info;
	priv->baseaddr = baseaddr;
	priv->p_data = p_data;
	spin_lock_init(&priv->reglock);

	mc_init(mci, pdev);

	if (priv->p_data->quirks & DDR_ECC_INTR_SUPPORT) {
		rc = setup_irq(mci, pdev);
		if (rc)
			goto free_edac_mc;
	}

	rc = edac_mc_add_mc(mci);
	if (rc) {
		edac_printk(KERN_ERR, EDAC_MC,
			    "Failed to register with EDAC core\n");
		goto free_edac_mc;
	}

#ifdef CONFIG_EDAC_DEBUG
	rc = edac_create_sysfs_attributes(mci);
	if (rc) {
		edac_printk(KERN_ERR, EDAC_MC,
			    "Failed to create sysfs entries\n");
		goto free_edac_mc;
	}

	if (priv->p_data->quirks & DDR_ECC_INTR_SUPPORT)
		setup_address_map(priv);
#endif
	spin_lock_init(&priv->cmd_flag_lock);
	init_waitqueue_head(&priv->edac_wq);
	priv->edac_cmd_flag = EDAC_NONE;
	priv->edac_task = kthread_create(adrv906x_edac_thread, (void *)mci,
					 "adrv906x_edac_thread");
	if (priv->edac_task) {
		edac_printk(KERN_INFO, EDAC_MC, "edac work thread created\n");
		wake_up_process(priv->edac_task);
	} else {
		edac_printk(KERN_ERR, EDAC_MC, "edac work thread creation failed\n");
	}

	return rc;

free_edac_mc:
	edac_mc_free(mci);

	return rc;
}

/**
 * mc_remove - Unbind driver from controller.
 * @pdev:	Platform device.
 *
 * Return: Unconditionally 0
 */
static void mc_remove(struct platform_device *pdev)
{
	struct mem_ctl_info *mci = platform_get_drvdata(pdev);
	struct adrv906x_edac_priv *priv = mci->pvt_info;

	kthread_stop(priv->edac_task);
#ifdef CONFIG_EDAC_DEBUG
	if (priv->p_data->quirks & DDR_ECC_DATA_POISON_SUPPORT)
		edac_remove_sysfs_attributes(mci);
#endif

	edac_mc_del_mc(&pdev->dev);
	edac_mc_free(mci);
}

static struct platform_driver adrv906x_edac_mc_driver = {
	.driver			= {
		.name		= "adrv906x-edac",
		.of_match_table = adrv906x_edac_match,
	},
	.probe			= mc_probe,
	.remove_new		= mc_remove,
};

module_platform_driver(adrv906x_edac_mc_driver);

MODULE_AUTHOR("Analog Devices Inc");
MODULE_DESCRIPTION("ADRV906X DDR ECC driver");
MODULE_LICENSE("GPL");
