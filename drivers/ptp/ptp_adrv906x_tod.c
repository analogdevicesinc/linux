// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/printk.h>
#include <linux/module.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/timekeeping.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/init.h>

#include "ptp_private.h"
#include "ptp_adrv906x_tod.h"

MODULE_DESCRIPTION("Driver for time-of-day module in adrv906x-based devices");
MODULE_AUTHOR("Landau Zhang <landau.zhang@analog.com>");
MODULE_AUTHOR("Kim Holdt <kim.holdt@analog.com>");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");

static struct adrv906x_tod *adrv906x_tod;

#define TOD_FRAC_NANO_NUM                               0x10000
#define TOD_MAX_DELAY_COUNT                             10
#define TOD_PPSX_PULSE_WIDTH                            (10 * NSEC_PER_MSEC)

#define ADRV906X_TOD_VERSION                            (0x4U)
#define   ADRV906X_TOD_VERSION_MINOR                    GENMASK(15, 0)
#define   ADRV906X_TOD_VERSION_MAJOR                    GENMASK(31, 16)
#define ADRV906X_TOD_CFG_INCR                           (0x10U)
#define   ADRV906X_TOD_CFG_INCR_FRAC_NS_PER_CLK_MASK    GENMASK(15, 0)
#define   ADRV906X_TOD_CFG_INCR_NS_PER_CLK_MASK         GENMASK(19, 16)
#define   ADRV906X_TOD_CFG_INCR_CNT_CTRL_MASK           GENMASK(26, 20)
#define   ADRV906X_TOD_CFG_INCR_CFG_TOD_CNT_EN_MASK     BIT(28)

#define ADRV906X_TOD_IRQ_EVENT                          (0x14U)

#define ADRV906X_TOD_IRQ_MASK                           (0x18U)
#define   ADRV906X_TOD_IRQ_MASK_INTERNAL_0              BIT(0)
#define   ADRV906X_TOD_IRQ_MASK_INTERNAL_1              BIT(1)
#define   ADRV906X_TOD_IRQ_MASK_INTERNAL_GNSS           BIT(2)
#define   ADRV906X_TOD_IRQ_MASK_EXTERNAL_PPS            BIT(3)
#define   ADRV906X_TOD_IRQ_MASK_MASK                    GENMASK(3, 0)

#define ADRV906X_TOD_IRQ_STATUS                         (0x1CU)

#define ADRV906X_TOD_CFG_TOD_OP                         (0x20U)
#define   ADRV906X_TOD_CFG_TOD_OP_WR_TOD_MASK           GENMASK(2, 0)
#define    ADRV906X_TOD_CFG_TOD_OP_WR_TOD_SHIFT         (0)
#define   ADRV906X_TOD_CFG_TOD_OP_RD_TOD_MASK           GENMASK(6, 4)
#define    ADRV906X_TOD_CFG_TOD_OP_RD_TOD_SHIFT         (4)
#define   ADRV906X_TOD_CFG_TOD_OP_WR_TOD_PPS_MASK       GENMASK(10, 8)
#define    ADRV906X_TOD_CFG_TOD_OP_WR_TOD_PPS_SHIFT     (8)
#define   ADRV906X_TOD_CFG_TOD_OP_RD_TOD_PPS_MASK       GENMASK(14, 12)
#define    ADRV906X_TOD_CFG_TOD_OP_RD_TOD_PPS_SHIFT     (12)

#define ADRV906X_TOD_CFG_TV_NSEC                        (0x24U)
#define   ADRV906X_TOD_CFG_TV_NSEC_FRAC_NSEC_MASK       GENMASK(15, 0)
#define   ADRV906X_TOD_CFG_TV_NSEC_NSEC_MASK            GENMASK(31, 16)

#define ADRV906X_TOD_CFG_TV_SEC_0                       (0x28U)
#define   ADRV906X_TOD_CFG_TV_SEC_0_NSEC_MASK           GENMASK(15, 0)
#define   ADRV906X_TOD_CFG_TV_SEC_0_SEC_MASK            GENMASK(31, 16)

#define ADRV906X_TOD_CFG_TV_SEC_1                       (0x2CU)
#define ADRV906X_TOD_CFG_OP_GC_VAL_0                    (0x30U)
#define ADRV906X_TOD_CFG_OP_GC_VAL_1                    (0x34U)

#define ADRV906X_TOD_CFG_OP_GC                          (0x38U)
#define   ADRV906X_TOD_CFG_OP_GC_RD_GC_MASK             BIT(0)

#define ADRV906X_TOD_CFG_IO_SOURCE                      (0x3CU)
#define   ADRV906X_TOD_CFG_IO_PPS_OUT_SRC_MASK          GENMASK(3, 0)
#define   ADRV906X_TOD_CFG_IO_PPS_OUT_SRC_SEL(x)                     \
	FIELD_PREP(ADRV906X_TOD_CFG_IO_PPS_OUT_SRC_MASK, x)
#define   ADRV906X_TOD_CFG_IO_TOD_OUT_SRC_MASK          GENMASK(11, 8)
#define   ADRV906X_TOD_CFG_IO_TOD_OUT_SRC_SEL(x)                     \
	FIELD_PREP(ADRV906X_TOD_CFG_IO_TOD_OUT_SRC_MASK, x)
#define   ADRV906X_TOD_CFG_IO_WR_OUTPUT_CFG_MASK        BIT(16)

#define ADRV906X_TOD_CFG_IO_CTRL                        (0x40U)
#define   ADRV906X_TOD_CFG_IO_CTRL_PPS_OUT_EN_MASK      BIT(0)
#define   ADRV906X_TOD_CFG_IO_CTRL_TOD_OUT_EN_MASK      BIT(4)
#define   ADRV906X_TOD_CFG_IO_CTRL_TOD_STAT_SEL_MASK    GENMASK(30, 28)
#define   ADRV906X_TOD_CFG_IO_CTRL_TOD_STAT_SEL(x)                   \
	FIELD_PREP(ADRV906X_TOD_CFG_IO_CTRL_TOD_STAT_SEL_MASK, x)

#define ADRV906X_TOD_CFG_PPSX_START                     (0x44U)
#define ADRV906X_TOD_CFG_PPSX_STOP                      (0x48U)

#define ADRV906X_TOD_CFG_TEST_OUT_SRC                   (0x4CU)
#define   ADRV906X_TOD_CFG_TEST_OUT_SRC_PPSX_SRC_MASK   GENMASK(31, 28)
#define   ADRV906X_TOD_CFG_TEST_OUT_SRC_PPSX_SRC(x)                  \
	FIELD_PREP(ADRV906X_TOD_CFG_TEST_OUT_SRC_PPSX_SRC_MASK, x)

#define ADRV906X_TOD_CFG_TSU_TOD                        (0x50U)
#define   ADRV906X_TOD_CFG_TSU_TOD_MASK                 GENMASK(1, 0)

#define ADRV906X_TOD_STAT_GC_0                          (0x70U)
#define ADRV906X_TOD_STAT_GC_1                          (0x74U)

#define ADRV906X_TOD_STAT_TV_NSEC                       (0x78U)
#define   ADRV906X_TOD_STAT_TV_FRAC_NSEC_MASK           GENMASK(15, 0)
#define   ADRV906X_TOD_STAT_TV_NSEC_NSEC_MASK           GENMASK(31, 16)

#define ADRV906X_TOD_STAT_TV_SEC_0                      (0x7CU)
#define   ADRV906X_TOD_STAT_TV_SEC_0_NSEC_MASK          GENMASK(15, 0)
#define   ADRV906X_TOD_STAT_TV_SEC_0_SEC_MASK           GENMASK(31, 16)

#define ADRV906X_TOD_STAT_TV_SEC_1                      (0x80U)

#define ADRV906X_TOD_STAT_TOD_OP                        (0x90U)
#define   ADRV906X_TOD_STAT_TOD_OP_WR_TOD_MASK          GENMASK(2, 0)
#define    ADRV906X_TOD_STAT_TOD_OP_WR_TOD_SHIFT        (0)
#define   ADRV906X_TOD_STAT_TOD_OP_RD_TOD_MASK          GENMASK(6, 4)
#define    ADRV906X_TOD_STAT_TOD_OP_RD_TOD_SHIFT        (4)
#define   ADRV906X_TOD_STAT_TOD_OP_WR_TOD_PPS_MASK      GENMASK(10, 8)
#define    ADRV906X_TOD_STAT_TOD_OP_WR_TOD_PPS_SHIFT    (8)
#define   ADRV906X_TOD_STAT_TOD_OP_RD_TOD_PPS_MASK      GENMASK(22, 20)
#define    ADRV906X_TOD_STAT_TOD_OP_RD_TOD_PPS_SHIFT    (12)

#define ADRV906X_TOD_CFG_CDC_DELAY                      (0x100U)
#define   ADRV906X_TOD_CFG_CDC_DELAY_CDC_MASK           GENMASK(4, 0)

int adrv906x_phc_index = -1;
EXPORT_SYMBOL(adrv906x_phc_index);

int adrv906x_tod_cfg_cdc_delay = -1;
EXPORT_SYMBOL(adrv906x_tod_cfg_cdc_delay);

struct adrv906x_tod_reg {
	u16 bitshift;
	u16 regaddr;
	u32 regmask;
};

static struct adrv906x_tod_reg adrv906x_tod_reg_op_trig[HW_TOD_TRIG_OP_CNT][HW_TOD_TRIG_MODE_CNT] = {
	[HW_TOD_TRIG_OP_WR] =		 {
		[HW_TOD_TRIG_MODE_GC] =	 {
			.regaddr	= ADRV906X_TOD_CFG_TOD_OP,
			.regmask	= ADRV906X_TOD_CFG_TOD_OP_WR_TOD_MASK,
			.bitshift	= ADRV906X_TOD_CFG_TOD_OP_WR_TOD_SHIFT
		},
		[HW_TOD_TRIG_MODE_PPS] = {
			.regaddr	= ADRV906X_TOD_CFG_TOD_OP,
			.regmask	= ADRV906X_TOD_CFG_TOD_OP_WR_TOD_PPS_MASK,
			.bitshift	= ADRV906X_TOD_CFG_TOD_OP_WR_TOD_PPS_SHIFT
		}
	},
	[HW_TOD_TRIG_OP_RD] =		 {
		[HW_TOD_TRIG_MODE_GC] =	 {
			.regaddr	= ADRV906X_TOD_CFG_TOD_OP,
			.regmask	= ADRV906X_TOD_CFG_TOD_OP_RD_TOD_MASK,
			.bitshift	= ADRV906X_TOD_CFG_TOD_OP_RD_TOD_SHIFT
		},
		[HW_TOD_TRIG_MODE_PPS] = {
			.regaddr	= ADRV906X_TOD_CFG_TOD_OP,
			.regmask	= ADRV906X_TOD_CFG_TOD_OP_RD_TOD_PPS_MASK,
			.bitshift	= ADRV906X_TOD_CFG_TOD_OP_RD_TOD_PPS_SHIFT
		}
	}
};

static struct adrv906x_tod_reg adrv906x_tod_reg_op_poll[HW_TOD_TRIG_OP_CNT][HW_TOD_TRIG_MODE_CNT] = {
	[HW_TOD_TRIG_OP_WR] =		 {
		[HW_TOD_TRIG_MODE_GC] =	 {
			.regaddr	= ADRV906X_TOD_STAT_TOD_OP,
			.regmask	= ADRV906X_TOD_STAT_TOD_OP_WR_TOD_MASK,
			.bitshift	= ADRV906X_TOD_STAT_TOD_OP_WR_TOD_SHIFT
		},
		[HW_TOD_TRIG_MODE_PPS] = {
			.regaddr	= ADRV906X_TOD_STAT_TOD_OP,
			.regmask	= ADRV906X_TOD_STAT_TOD_OP_WR_TOD_PPS_MASK,
			.bitshift	= ADRV906X_TOD_STAT_TOD_OP_WR_TOD_PPS_SHIFT
		}
	},
	[HW_TOD_TRIG_OP_RD] =		 {
		[HW_TOD_TRIG_MODE_GC] =	 {
			.regaddr	= ADRV906X_TOD_STAT_TOD_OP,
			.regmask	= ADRV906X_TOD_STAT_TOD_OP_RD_TOD_MASK,
			.bitshift	= ADRV906X_TOD_STAT_TOD_OP_RD_TOD_SHIFT
		},
		[HW_TOD_TRIG_MODE_PPS] = {
			.regaddr	= ADRV906X_TOD_STAT_TOD_OP,
			.regmask	= ADRV906X_TOD_STAT_TOD_OP_RD_TOD_PPS_MASK,
			.bitshift	= ADRV906X_TOD_STAT_TOD_OP_RD_TOD_PPS_SHIFT
		}
	}
};


struct adrv906x_tod_lc_clk_cfg adrv906x_lc_clk_cfg[HW_TOD_LC_CLK_FREQ_CNT] = {
	[HW_TOD_LC_100_P_000_M] = { 100000, 10, 0x0000, 0x00 },
	[HW_TOD_LC_122_P_880_M] = { 122880, 8,	0x2355, 0x04 },
	[HW_TOD_LC_125_P_000_M] = { 125000, 8,	0x0000, 0x00 },
	[HW_TOD_LC_156_P_250_M] = { 156250, 6,	0x6666, 0x01 },
	[HW_TOD_LC_245_P_760_M] = { 245760, 4,	0x11AA, 0x02 },
	[HW_TOD_LC_250_P_000_M] = { 250000, 4,	0x0000, 0x00 },
	[HW_TOD_LC_312_P_500_M] = { 312500, 3,	0x3333, 0x08 },
	[HW_TOD_LC_322_P_265_M] = { 322265, 3,	0x1A60, 0x20 },
	[HW_TOD_LC_390_P_625_M] = { 390625, 2,	0x8F5C, 0x10 },
	[HW_TOD_LC_491_P_520_M] = { 491520, 2,	0x08D5, 0x04 },
	[HW_TOD_LC_500_P_000_M] = { 500000, 2,	0x0000, 0x00 },
	[HW_TOD_LC_983_P_040_M] = { 983040, 1,	0x046A, 0x02 }
};

static int adrv906x_tod_cfg_lc_clk(struct adrv906x_tod_counter *counter)
{
	struct adrv906x_tod *tod = counter->parent;
	int err = -EINVAL;
	u32 wr_val;
	int lp;

	for (lp = 0; lp < HW_TOD_LC_CLK_FREQ_CNT; lp++) {
		if (tod->lc_freq_khz == adrv906x_lc_clk_cfg[lp].freq_khz) {
			wr_val = FIELD_PREP(ADRV906X_TOD_CFG_INCR_FRAC_NS_PER_CLK_MASK,
					    adrv906x_lc_clk_cfg[lp].frac_ns_per_clk) |
				 FIELD_PREP(ADRV906X_TOD_CFG_INCR_NS_PER_CLK_MASK,
					    adrv906x_lc_clk_cfg[lp].ns_per_clk) |
				 FIELD_PREP(ADRV906X_TOD_CFG_INCR_CNT_CTRL_MASK,
					    adrv906x_lc_clk_cfg[lp].cnt_ctrl);
			iowrite32(wr_val, tod->regs + ADRV906X_TOD_CFG_INCR);
			err = 0;
			break;
		}
	}

	return err;
}

static inline void timespec_to_tstamp(struct adrv906x_tod_tstamp *tstamp, const struct timespec64 *ts)
{
	tstamp->nanoseconds = ts->tv_nsec;
	tstamp->frac_nanoseconds = 0;
	tstamp->seconds = ts->tv_sec;
}

static inline void tstamp_to_timespec(struct timespec64 *ts, const struct adrv906x_tod_tstamp *tstamp)
{
	ts->tv_sec = tstamp->seconds;

	if (tstamp->frac_nanoseconds < (TOD_FRAC_NANO_NUM / 2))
		ts->tv_nsec = tstamp->nanoseconds;
	else
		ts->tv_nsec = tstamp->nanoseconds + 1;
}

static int adrv906x_tod_hw_gc_get_cnt(struct adrv906x_tod_counter *counter, u64 *p_cnt)
{
	struct adrv906x_tod *tod = counter->parent;
	u32 gc_reg_cnt[2] = { 0, 0 };
	u64 gc_cnt;
	u32 gc_rd = 1;

	/* Write the OP_GC:RD_GC_MASK to latch the GC counter register */
	iowrite32(gc_rd, tod->regs + ADRV906X_TOD_CFG_OP_GC);
	iowrite32(0, tod->regs + ADRV906X_TOD_CFG_OP_GC);

	/* Read back the Golden Counter */
	gc_reg_cnt[0] = ioread32(tod->regs + ADRV906X_TOD_STAT_GC_0);
	gc_reg_cnt[1] = ioread32(tod->regs + ADRV906X_TOD_STAT_GC_1);

	gc_cnt = gc_reg_cnt[0] | ((u64)(gc_reg_cnt[1] & 0xFFFF) << 32);
	*p_cnt = gc_cnt;

	return 0;
}

static int adrv906x_tod_hw_gc_set_cnt(struct adrv906x_tod_counter *counter, u64 cnt)
{
	struct adrv906x_tod *tod = counter->parent;
	u32 gc_reg_cnt[2] = { 0, 0 };

	gc_reg_cnt[0] = cnt & 0xFFFFFFFF;
	gc_reg_cnt[1] = (cnt >> 32) & 0xFFFF;

	/* Write the GC value */
	iowrite32(gc_reg_cnt[0], tod->regs + ADRV906X_TOD_CFG_OP_GC_VAL_0);
	iowrite32(gc_reg_cnt[1], tod->regs + ADRV906X_TOD_CFG_OP_GC_VAL_1);

	return 0;
}

static void adrv906x_tod_clear_soft_pps(struct work_struct *work)
{
	struct adrv906x_tod *tod = container_of(work, struct adrv906x_tod, pps_work.work);

	tod->pps_high = false;

	wake_up_all(&tod->pps_queue);
}

static void adrv906x_tod_hw_op_trig(struct adrv906x_tod_counter *counter, u8 op_flag, u8 set_flag)
{
	struct adrv906x_tod *tod = counter->parent;
	u8 trig_mode = counter->trigger_mode;
	u16 bitshift;
	u16 regaddr;
	u8 tod_idx;
	u32 val;

	tod_idx = counter->id;
	regaddr = adrv906x_tod_reg_op_trig[op_flag][trig_mode].regaddr;
	bitshift = adrv906x_tod_reg_op_trig[op_flag][trig_mode].bitshift;

	val = ioread32(tod->regs + regaddr);
	if (set_flag == HW_TOD_TRIG_SET_FLAG_TRIG)
		val |= (BIT(tod_idx) << bitshift);
	else
		val &= ~(BIT(tod_idx) << bitshift);
	iowrite32(val, tod->regs + regaddr);
}

static void adrv906x_tod_hw_op_trig_set(struct adrv906x_tod_counter *counter, u8 op_flag)
{
	struct adrv906x_tod *tod = counter->parent;

	/* In PPS mode and HW version 2.2 and below, only trigger when PPS signal is low. */
	if (counter->trigger_mode == HW_TOD_TRIG_MODE_PPS &&
	    tod->ver_major == 2 && tod->ver_minor <= 2) {
		dev_info(tod->dev, "trigger waiting for interrupt");
		wait_event(tod->pps_queue, !tod->pps_high);
	}

	adrv906x_tod_hw_op_trig(counter, op_flag, HW_TOD_TRIG_SET_FLAG_TRIG);
}

static void adrv906x_tod_hw_op_trig_clear(struct adrv906x_tod_counter *counter, u8 op_flag)
{
	adrv906x_tod_hw_op_trig(counter, op_flag, HW_TOD_TRIG_SET_FLAG_CLEAR);
}

static int adrv906x_tod_hw_op_poll_reg(struct adrv906x_tod_counter *counter, u32 regaddr,
				       u32 bit_mask, const struct adrv906x_tod_trig_delay *p_delay, bool done_high)
{
	u32 delay_cnt = TOD_MAX_DELAY_COUNT;
	u8 done = 0;
	int err = 0;
	u32 val;

	while (!done && (delay_cnt != 0)) {
		ndelay(p_delay->ns);
		val = ioread32(counter->parent->regs + regaddr);

		if (!done_high)
			val = ~val;

		done = (val & bit_mask) == bit_mask;
		delay_cnt--;
	}

	if (!done) {
		dev_err(counter->parent->dev,
			"trigger operation on reg 0x%x bit(s) 0x%x missed, delay configured: %llu us",
			regaddr, bit_mask, p_delay->ns / NSEC_PER_USEC);
		err = -EAGAIN;
	}

	return err;
}

static int adrv906x_tod_hw_op_poll(struct adrv906x_tod_counter *counter, u8 op_flag,
				   const struct adrv906x_tod_trig_delay *p_delay)
{
	u8 trig_mode = counter->trigger_mode;
	u8 tod_idx = counter->id;
	u32 bit_mask, regaddr;
	int err;

	regaddr = adrv906x_tod_reg_op_poll[op_flag][trig_mode].regaddr;
	bit_mask = BIT(adrv906x_tod_reg_op_poll[op_flag][trig_mode].bitshift + tod_idx);
	err = adrv906x_tod_hw_op_poll_reg(counter, regaddr, bit_mask, p_delay, true);

	return err;
}

static int adrv906x_tod_compensate_tstamp(struct adrv906x_tod_counter *counter,
					  struct adrv906x_tod_tstamp *tstamp,
					  const struct adrv906x_tod_trig_delay *trig_delay)
{
	struct adrv906x_tod *tod = counter->parent;
	struct adrv906x_tod_tstamp old_tstamp;
	u32 frac_ns_tstamp;
	u32 seconds;
	u32 ns;

	/*
	 * Update the ToD vector value:
	 * new_tstamp_value = old_tstamp + trigger_delay
	 */
	memcpy(&old_tstamp, tstamp, sizeof(struct adrv906x_tod_tstamp));

	/*
	 * Fraction part of the nanosecond stored as a 16bit value in the ToD tstamp:
	 * frac_ns_tstamp = (trig_delay.rem_ns / gc_clk_frequency) * 2^16
	 */
	frac_ns_tstamp = (u32)div_u64(trig_delay->rem_ns * TOD_FRAC_NANO_NUM, tod->gc_clk_freq_khz);

	/* Update the fraction part of nanosecond and the nanosecond part in the tstamp */
	if ((old_tstamp.frac_nanoseconds + frac_ns_tstamp) < TOD_FRAC_NANO_NUM) {
		tstamp->frac_nanoseconds = (u16)(old_tstamp.frac_nanoseconds + frac_ns_tstamp);
		tstamp->nanoseconds = old_tstamp.nanoseconds + trig_delay->ns;
	} else {
		tstamp->frac_nanoseconds = (u16)((old_tstamp.frac_nanoseconds + frac_ns_tstamp) - TOD_FRAC_NANO_NUM);
		tstamp->nanoseconds = old_tstamp.nanoseconds + trig_delay->ns + 1;
	}

	/* Update the second part in the tstamp */
	if (tstamp->nanoseconds >= NSEC_PER_SEC) {
		seconds = div_u64_rem(tstamp->nanoseconds, NSEC_PER_SEC, &ns);
		tstamp->nanoseconds = ns;
		tstamp->seconds = old_tstamp.seconds + seconds;
	} else {
		tstamp->seconds = old_tstamp.seconds;
	}

	return 0;
}

static void adrv906x_tod_hw_settstamp_to_reg(struct adrv906x_tod_counter *counter,
					     const struct adrv906x_tod_tstamp *tstamp)
{
	struct adrv906x_tod *tod = counter->parent;
	u32 reg_tstamp[3] = { 0 };

	reg_tstamp[0] = (tstamp->frac_nanoseconds & 0xFFFF) | ((tstamp->nanoseconds & 0xFFFF) << 16);
	reg_tstamp[1] = ((tstamp->nanoseconds & 0xFFFF0000) >> 16) | ((tstamp->seconds & 0xFFFF) << 16);
	reg_tstamp[2] = ((tstamp->seconds & 0xFFFFFFFF0000) >> 16);

	iowrite32(reg_tstamp[0], tod->regs + ADRV906X_TOD_CFG_TV_NSEC);
	iowrite32(reg_tstamp[1], tod->regs + ADRV906X_TOD_CFG_TV_SEC_0);
	iowrite32(reg_tstamp[2], tod->regs + ADRV906X_TOD_CFG_TV_SEC_1);
}

static int adrv906x_tod_hw_gettstamp_from_reg(struct adrv906x_tod_counter *counter,
					      struct adrv906x_tod_tstamp *tstamp)
{
	struct adrv906x_tod *tod = counter->parent;
	u32 reg_tstamp[3] = { 0 };
	u8 tod_idx;
	u32 val;

	tod_idx = counter->id;

	val = ioread32(tod->regs + ADRV906X_TOD_CFG_IO_CTRL);
	val &= ~ADRV906X_TOD_CFG_IO_CTRL_TOD_STAT_SEL_MASK;
	val |= ADRV906X_TOD_CFG_IO_CTRL_TOD_STAT_SEL(BIT(tod_idx));
	iowrite32(val, tod->regs + ADRV906X_TOD_CFG_IO_CTRL);

	reg_tstamp[0] = ioread32(tod->regs + ADRV906X_TOD_STAT_TV_NSEC);
	reg_tstamp[1] = ioread32(tod->regs + ADRV906X_TOD_STAT_TV_SEC_0);
	reg_tstamp[2] = ioread32(tod->regs + ADRV906X_TOD_STAT_TV_SEC_1);

	tstamp->frac_nanoseconds = reg_tstamp[0] & 0xFFFF;
	tstamp->nanoseconds = ((reg_tstamp[0] >> 16) & 0xFFFF) | ((reg_tstamp[1] & 0xFFFF) << 16);
	tstamp->seconds = ((reg_tstamp[1] >> 16) & 0xFFFF) | (reg_tstamp[2] << 16);

	return 0;
}

static void adrv906x_tod_hw_set_trigger_delay(struct adrv906x_tod_counter *counter)
{
	u64 gc_cnt = 0;

	/* Set the trigger delay to GC value register */
	adrv906x_tod_hw_gc_get_cnt(counter, &gc_cnt);
	gc_cnt += counter->trig_delay_tick;
	adrv906x_tod_hw_gc_set_cnt(counter, gc_cnt);
}

static void adrv906x_tod_get_trigger_delay(struct adrv906x_tod_counter *counter,
					   struct adrv906x_tod_trig_delay *trig_delay)
{
	struct adrv906x_tod *tod = counter->parent;

	/**
	 * The trigger delay value depends on the counter->trig_delay_tick.
	 * adrv906x_tod_trig_delay.ns = counter->trig_delay_tick * 1e6 / tod->gc_clk_freq_khz
	 * adrv906x_tod_trig_delay.frac_ns = counter->trig_delay_tick * 1e6 % tod->gc_clk_freq_khz
	 * 1e6 is used to calculate the nano-second of the trigger tick so that the
	 * "counter->trig_delay_tick * 1e6" will not overflow unless counter->trig_delay_tick beyond
	 * the value "2^44".
	 */
	trig_delay->ns = div_u64_rem(counter->trig_delay_tick * USEC_PER_SEC,
				     tod->gc_clk_freq_khz,
				     &(trig_delay->rem_ns));
}

static int adrv906x_tod_hw_settstamp(struct adrv906x_tod_counter *counter,
				     const struct adrv906x_tod_tstamp *vector)
{
	struct adrv906x_tod_trig_delay trig_delay = { 0, 0 };
	struct adrv906x_tod_tstamp tstamp = { 0, 0, 0 };
	int err;

	memcpy(&tstamp, vector, sizeof(struct adrv906x_tod_tstamp));

	if (counter->trigger_mode == HW_TOD_TRIG_MODE_GC) {
		adrv906x_tod_get_trigger_delay(counter, &trig_delay);
		adrv906x_tod_compensate_tstamp(counter, &tstamp, &trig_delay);
		adrv906x_tod_hw_set_trigger_delay(counter);
	} else {
		/* We set the delay to time out polling after a second. */
		trig_delay.ns = 100 * NSEC_PER_MSEC;
	}

	adrv906x_tod_hw_settstamp_to_reg(counter, &tstamp);

	adrv906x_tod_hw_op_trig_set(counter, HW_TOD_TRIG_OP_WR);
	err = adrv906x_tod_hw_op_poll(counter, HW_TOD_TRIG_OP_WR, &trig_delay);

	if (!err)
		adrv906x_tod_hw_op_trig_clear(counter, HW_TOD_TRIG_OP_WR);

	return err;
}

static int adrv906x_tod_get_tstamp(struct adrv906x_tod_counter *counter,
				   struct adrv906x_tod_tstamp *tstamp)
{
	struct adrv906x_tod_trig_delay trig_delay = { 0, 0 };
	int err;

	adrv906x_tod_get_trigger_delay(counter, &trig_delay);

	if (counter->trigger_mode == HW_TOD_TRIG_MODE_GC)
		adrv906x_tod_hw_set_trigger_delay(counter);
	else
		/* We set the delay to time out polling after a second. */
		trig_delay.ns = 100 * NSEC_PER_MSEC;

	adrv906x_tod_hw_op_trig_set(counter, HW_TOD_TRIG_OP_RD);
	err = adrv906x_tod_hw_op_poll(counter, HW_TOD_TRIG_OP_RD, &trig_delay);

	if (!err)
		adrv906x_tod_hw_gettstamp_from_reg(counter, tstamp);

	adrv906x_tod_hw_op_trig_clear(counter, HW_TOD_TRIG_OP_RD);

	return err;
}

static int adrv906x_tod_adjust_time(struct adrv906x_tod_counter *counter, s64 delta)
{
	struct adrv906x_tod_tstamp tstamp = { 0 };
	s64 seconds;
	int err;
	s32 ns;

	err = adrv906x_tod_get_tstamp(counter, &tstamp);

	seconds = div_s64_rem(delta, NSEC_PER_SEC, &ns);
	if (!err) {
		if ((ns < 0) && (abs(ns) > tstamp.nanoseconds)) {
			tstamp.nanoseconds = NSEC_PER_SEC + ns + tstamp.nanoseconds;
			tstamp.seconds -= 1;
		} else {
			tstamp.nanoseconds += ns;
		}

		if (tstamp.nanoseconds < NSEC_PER_SEC) {
			tstamp.seconds += seconds;
		} else {
			tstamp.nanoseconds -= NSEC_PER_SEC;
			tstamp.seconds += seconds + 1;
		}

		err = adrv906x_tod_hw_settstamp(counter, &tstamp);
	}

	return err;
}

static int adrv906x_tod_hw_cdc_output_enable(struct adrv906x_tod_counter *counter, u8 enable)
{
	struct adrv906x_tod *tod = counter->parent;
	u8 tod_idx;
	u32 val = 0;
	int i;

	if ((counter->en) && (counter->id != TOD_INTERNAL_GNSS))
		tod_idx = counter->id;
	else
		return -ENODEV;

	if (enable)
		val |= BIT(tod_idx);

	for (i = 0; i < ADRV906X_HW_TOD_CDC_DOMAIN_CNT; i++)
		iowrite32(val, tod->regs + ADRV906X_TOD_CFG_TSU_TOD + i * 0x4);

	return 0;
}

static int adrv906x_tod_hw_extts_enable(struct adrv906x_tod_counter *counter, u8 enable)
{
	struct adrv906x_tod_trig_delay trig_delay = { 0, 0 };
	struct adrv906x_tod *tod = counter->parent;
	u8 tod_idx = counter->id;
	u32 val;
	int ret;

	if (!counter->en) {
		dev_err(tod->dev, "tod %d is disabled, cannot enable output", tod_idx);
		return -EOPNOTSUPP;
	}

	adrv906x_tod_get_trigger_delay(counter, &trig_delay);
	adrv906x_tod_hw_set_trigger_delay(counter);

	val = ioread32(tod->regs + ADRV906X_TOD_CFG_IO_SOURCE);
	val &= ~ADRV906X_TOD_CFG_IO_TOD_OUT_SRC_MASK;
	val |= ADRV906X_TOD_CFG_IO_WR_OUTPUT_CFG_MASK;

	if (enable)
		val |= ADRV906X_TOD_CFG_IO_TOD_OUT_SRC_SEL(BIT(tod_idx));
	else
		val &= ~ADRV906X_TOD_CFG_IO_TOD_OUT_SRC_SEL(BIT(tod_idx));
	iowrite32(val, tod->regs + ADRV906X_TOD_CFG_IO_SOURCE);

	ret = adrv906x_tod_hw_op_poll_reg(counter, ADRV906X_TOD_CFG_IO_SOURCE,
					  ADRV906X_TOD_CFG_IO_WR_OUTPUT_CFG_MASK, &trig_delay, false);

	return ret;
}

static int adrv906x_tod_pps_irq_enable(struct adrv906x_tod_counter *counter, u8 enable)
{
	struct adrv906x_tod *tod = counter->parent;
	int tod_idx = counter->id;
	int val;

	if (!counter->en)
		return -ENODEV;

	val = ioread32(tod->regs + ADRV906X_TOD_IRQ_MASK);

	if (enable)
		val &= ~BIT(tod_idx);
	else
		val |= BIT(tod_idx);

	iowrite32(val, tod->regs + ADRV906X_TOD_IRQ_MASK);

	return 0;
}

static void adrv906x_tod_hw_pps_irq_external_enable(struct adrv906x_tod *tod)
{
	u32 val = 0;

	val |= ~ADRV906X_TOD_IRQ_MASK_EXTERNAL_PPS;
	iowrite32(val, tod->regs + ADRV906X_TOD_IRQ_MASK);
}

static int adrv906x_tod_hw_pps_enable(struct adrv906x_tod_counter *counter, u8 enable)
{
	struct adrv906x_tod_trig_delay trig_delay = { 0, 0 };
	struct adrv906x_tod *tod = counter->parent;
	u32 val;
	int ret;

	adrv906x_tod_get_trigger_delay(counter, &trig_delay);
	adrv906x_tod_hw_set_trigger_delay(counter);

	val = ioread32(tod->regs + ADRV906X_TOD_CFG_IO_SOURCE);
	val &= ~ADRV906X_TOD_CFG_IO_PPS_OUT_SRC_MASK;
	val |= ADRV906X_TOD_CFG_IO_WR_OUTPUT_CFG_MASK;

	/* Do not change source when external pps is enabled */
	if (enable) {
		if (tod->external_pps) {
			val |= ADRV906X_TOD_CFG_IO_PPS_OUT_SRC_SEL(BIT(TOD_EXTERNAL));
			dev_info(tod->dev, "using external pps");
		} else {
			val |= ADRV906X_TOD_CFG_IO_PPS_OUT_SRC_SEL(BIT(counter->id));
		}
	}
	iowrite32(val, tod->regs + ADRV906X_TOD_CFG_IO_SOURCE);

	ret = adrv906x_tod_hw_op_poll_reg(counter, ADRV906X_TOD_CFG_IO_SOURCE,
					  ADRV906X_TOD_CFG_IO_WR_OUTPUT_CFG_MASK, &trig_delay, false);

	return ret;
}

static int adrv906x_tod_pps_enable(struct adrv906x_tod_counter *counter, u8 on)
{
	struct adrv906x_tod *tod = counter->parent;

	mutex_lock(&tod->reg_lock);
	adrv906x_tod_hw_pps_enable(counter, on);
	mutex_unlock(&tod->reg_lock);

	return 0;
}

static irqreturn_t adrv906x_tod_pps_isr(int irq, void *dev_id)
{
	struct adrv906x_tod *tod = dev_id;
	struct adrv906x_tod_counter *counter;
	struct ptp_clock_event event;
	u32 irq_val;
	u8 i;

	irq_val = ioread32(tod->regs + ADRV906X_TOD_IRQ_STATUS);
	iowrite32(irq_val, tod->regs + ADRV906X_TOD_IRQ_EVENT);

	for (i = 0; i < ADRV906X_HW_TOD_COUNTER_CNT; i++) {
		if (irq_val & BIT(i) || irq_val == ADRV906X_TOD_IRQ_MASK_EXTERNAL_PPS) {
			counter = &tod->counter[i];
			if (counter->en) {
				event.type = PTP_CLOCK_PPS;
				ptp_clock_event(counter->ptp_clk, &event);
			}
		}
	}

	if (tod->ver_major == 2 && tod->ver_minor <= 2) {
		tod->pps_high = true;
		schedule_delayed_work(&tod->pps_work, msecs_to_jiffies(tod->pps_in_pulse_width_ms));
	}

	return IRQ_HANDLED;
}

static void adrv906x_tod_hw_cfg_ppsx(struct adrv906x_tod_counter *counter,
				     struct ptp_perout_request *rq)
{
	struct adrv906x_tod *tod = counter->parent;
	u32 stop, val;

	val = ioread32(tod->regs + ADRV906X_TOD_CFG_TEST_OUT_SRC);
	val &= ~ADRV906X_TOD_CFG_TEST_OUT_SRC_PPSX_SRC_MASK;

	if (!(rq->period.sec == 0 && rq->period.nsec == 0)) {
		iowrite32(rq->start.nsec, tod->regs + ADRV906X_TOD_CFG_PPSX_START);
		stop = (rq->start.nsec + tod->ppsx_pulse_width_ns) & 0xFFFFFFFF;
		iowrite32(stop, tod->regs + ADRV906X_TOD_CFG_PPSX_STOP);

		val |= ADRV906X_TOD_CFG_TEST_OUT_SRC_PPSX_SRC(BIT(counter->id));
	}

	iowrite32(val, tod->regs + ADRV906X_TOD_CFG_TEST_OUT_SRC);
}

static int adrv906x_tod_perout_enable(struct adrv906x_tod_counter *counter,
				      struct ptp_perout_request *rq)
{
	struct adrv906x_tod *tod = counter->parent;

	mutex_lock(&tod->reg_lock);
	adrv906x_tod_hw_cfg_ppsx(counter, rq);
	mutex_unlock(&tod->reg_lock);

	return 0;
}

static int adrv906x_tod_cfg_cdc_delay_set(struct adrv906x_tod_counter *counter)
{
	struct adrv906x_tod *tod = counter->parent;
	u32 i;

	for (i = 0; i < ADRV906X_HW_TOD_CDC_DOMAIN_CNT; i++)
		iowrite32(tod->cdc.delay_cnt[i],
			  tod->regs + ADRV906X_TOD_CFG_CDC_DELAY + i * sizeof(u32));

	/* According to the user manual, all CFG_CDC_DELAY:CDC register fields
	 * shall have the same value. So we just pick the first one.
	 */
	adrv906x_tod_cfg_cdc_delay = tod->cdc.delay_cnt[0];

	return 0;
}

static int adrv906x_tod_module_init(struct adrv906x_tod_counter *counter)
{
	struct adrv906x_tod *tod = counter->parent;
	u32 val;
	int ret;

	/* Update the ns and frac_ns part to the CFG_INCR */
	ret = adrv906x_tod_cfg_lc_clk(counter);
	/* Enable the ToD counter */
	if (!ret) {
		val = ioread32(tod->regs + ADRV906X_TOD_CFG_INCR);
		val |= ADRV906X_TOD_CFG_INCR_CFG_TOD_CNT_EN_MASK;
		iowrite32(val, tod->regs + ADRV906X_TOD_CFG_INCR);
	}

	return ret;
}

static int adrv906x_tod_dt_parse(struct adrv906x_tod_counter *counter, struct device_node *np)
{
	struct adrv906x_tod *tod = counter->parent;
	struct device *dev = tod->dev;
	int ret;
	u32 val;

	if (!np) {
		dev_err(dev, "platform tod data missing!");
		return -ENODEV;
	}

	counter->trigger_mode = of_property_read_bool(np, "adi,pps-mode");
	dev_info(dev, "tod trigger mode: %s", counter->trigger_mode ==
		 HW_TOD_TRIG_MODE_GC ? "gc mode" : "pps mode");

	ret = of_property_read_u32(np, "adi,trigger-delay-tick", &val);
	if (ret) {
		/* Use a default of 500 us based on the provided clock speed */
		val = tod->gc_clk_freq_khz * 500 / 1000;
		dev_info(dev, "'adi,trigger-delay-tick' not set, using '%u'", val);
	}
	counter->trig_delay_tick = val;

	return 0;
}

static int adrv906x_tod_extts_enable(struct adrv906x_tod_counter *counter, u8 enable)
{
	struct adrv906x_tod *tod = counter->parent;
	int ret;

	if (!counter->en)
		return -ENODEV;

	mutex_lock(&tod->reg_lock);
	if (counter->id != TOD_INTERNAL_GNSS) {
		ret = adrv906x_tod_hw_cdc_output_enable(counter, enable);
		if (ret)
			goto exit;

		adrv906x_phc_index = ptp_clock_index(counter->ptp_clk);
	}
	ret = adrv906x_tod_hw_extts_enable(counter, enable);

exit:
	mutex_unlock(&tod->reg_lock);

	return ret;
}

static int adrv906x_tod_enable(struct adrv906x_tod_counter *counter,
			       struct ptp_clock_request *rq, int enable)
{
	struct adrv906x_tod *tod = counter->parent;
	int ret;

	switch (rq->type) {
	case PTP_CLK_REQ_EXTTS:
		/* Reject requests with unsupported flags */
		if (rq->extts.flags & ~(PTP_ENABLE_FEATURE |
					PTP_RISING_EDGE |
					PTP_FALLING_EDGE |
					PTP_STRICT_FLAGS))
			return -EOPNOTSUPP;

		/* Reject requests to enable time stamping on falling edges */
		if ((rq->extts.flags & PTP_STRICT_FLAGS) &&
		    (rq->extts.flags & PTP_ENABLE_FEATURE) &&
		    (rq->extts.flags & PTP_FALLING_EDGE))
			return -EOPNOTSUPP;

		ret = adrv906x_tod_extts_enable(counter, enable);
		break;
	case PTP_CLK_REQ_PEROUT:
		/* Reject requests with unsupported flags */
		/* Add PTP_PEROUT_PHASE & PTP_PEROUT_DUTY_CYCLE when supported */
		if (rq->perout.flags & PTP_PEROUT_ONE_SHOT)
			return -EOPNOTSUPP;

		/* Reject requests that are not pps */
		if (enable && (rq->perout.period.sec != 1 ||
			       rq->perout.period.nsec != 0))
			return -EINVAL;

		if (enable && (rq->perout.start.nsec + tod->ppsx_pulse_width_ns)
		    >= NSEC_PER_SEC) {
			dev_err(tod->dev, "periodic pulse crosses 1 second boundary");
			return -EINVAL;
		}

		/* Enable ppsx for periodic output for given tod counter */
		ret = adrv906x_tod_perout_enable(counter, &rq->perout);
		break;
	case PTP_CLK_REQ_PPS:
		/* Enable internal pps output for given tod counter */
		ret = adrv906x_tod_pps_enable(counter, enable);
		break;
	default:
		ret = -EOPNOTSUPP;
	}

	return ret;
}

static int adrv906x_tod_settime(struct adrv906x_tod_counter *counter, const struct timespec64 *ts)
{
	struct adrv906x_tod *tod = counter->parent;
	struct adrv906x_tod_tstamp tstamp;
	int err;

	timespec_to_tstamp(&tstamp, ts);
	mutex_lock(&tod->reg_lock);
	err = adrv906x_tod_hw_settstamp(counter, &tstamp);
	mutex_unlock(&tod->reg_lock);

	return err;
}

static int adrv906x_tod_adjtime(struct adrv906x_tod_counter *counter, s64 delta)
{
	struct adrv906x_tod *tod = counter->parent;
	int err;

	mutex_lock(&tod->reg_lock);
	err = adrv906x_tod_adjust_time(counter, delta);
	mutex_unlock(&tod->reg_lock);

	return err;
}

static int adrv906x_tod_gettimex(struct adrv906x_tod_counter *counter,
				 struct timespec64 *ts,
				 struct ptp_system_timestamp *sts)
{
	struct adrv906x_tod *tod = counter->parent;
	struct adrv906x_tod_tstamp tstamp;
	int err;

	mutex_lock(&tod->reg_lock);
	ptp_read_system_prets(sts);
	err = adrv906x_tod_get_tstamp(counter, &tstamp);
	ptp_read_system_postts(sts);
	tstamp_to_timespec(ts, &tstamp);
	mutex_unlock(&tod->reg_lock);

	return err;
}

static int adrv906x_tod_cfg_cdc_delay_all(struct adrv906x_tod *tod)
{
	u8 i;

	for (i = 0; i < ADRV906X_HW_TOD_COUNTER_CNT; i++) {
		if (tod->counter[i].en) {
			adrv906x_tod_cfg_cdc_delay_set(&tod->counter[i]);
			break;
		}
	}

	return 0;
}

static void adrv906x_tod_hw_external_pps_override(struct adrv906x_tod *tod)
{
	adrv906x_tod_hw_pps_irq_external_enable(tod);
}

static int adrv906x_phc_enable(struct ptp_clock_info *ptp,
			       struct ptp_clock_request *rq, int enable)
{
	struct adrv906x_tod_counter *counter = container_of(ptp, struct adrv906x_tod_counter, caps);

	return adrv906x_tod_enable(counter, rq, enable);
}

static int adrv906x_phc_settime(struct ptp_clock_info *ptp, const struct timespec64 *ts)
{
	struct adrv906x_tod_counter *counter = container_of(ptp, struct adrv906x_tod_counter, caps);

	return adrv906x_tod_settime(counter, ts);
}

static int adrv906x_phc_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct adrv906x_tod_counter *counter = container_of(ptp, struct adrv906x_tod_counter, caps);

	return adrv906x_tod_adjtime(counter, delta);
}

static int adrv906x_phc_adjfreq(struct ptp_clock_info *ptp, s32 delta)
{
	return -EOPNOTSUPP;
}

static int adrv906x_phc_gettimex(struct ptp_clock_info *ptp,
				 struct timespec64 *ts,
				 struct ptp_system_timestamp *sts)
{
	struct adrv906x_tod_counter *counter = container_of(ptp, struct adrv906x_tod_counter, caps);

	return adrv906x_tod_gettimex(counter, ts, sts);
}

static struct ptp_clock_info adrv906x_tod_caps = {
	.owner		= THIS_MODULE,
	.n_per_out	= 1,
	.n_ext_ts	= 1,
	.pps		= 1,
	.adjfine	= NULL,
	.adjfreq	= &adrv906x_phc_adjfreq,
	.adjtime	= &adrv906x_phc_adjtime,
	.gettimex64	= &adrv906x_phc_gettimex,
	.getcrosststamp = NULL,
	.settime64	= &adrv906x_phc_settime,
	.enable		= &adrv906x_phc_enable,
	.do_aux_work	= NULL,  /* Use the aux */
};

static int adrv906x_tod_add_counter(struct adrv906x_tod *tod, struct device_node *np)
{
	struct adrv906x_tod_counter *counter;
	int ret;
	u32 val;

	/* Get the tod index */
	ret = of_property_read_u32(np, "reg", &val);
	if (ret) {
		dev_err(tod->dev, "dt: tod 'reg' property missing");
		return ret;
	}

	if (val >= ADRV906X_HW_TOD_COUNTER_CNT)
		return -EINVAL;

	counter = &tod->counter[val];

	if (counter->en) {
		dev_err(tod->dev, "dt: 'reg' value of '%d' used more than once", val);
		return -EINVAL;
	}

	counter->en = true;
	counter->id = val;
	counter->parent = tod;

	ret = adrv906x_tod_dt_parse(counter, np);
	if (ret) {
		dev_err(tod->dev, "dt: tod counter dt parse failed");
		counter->en = false;
		return ret;
	}

	ret = adrv906x_tod_module_init(counter);
	if (ret) {
		counter->en = false;
		return ret;
	}

	counter->caps = adrv906x_tod_caps;
	snprintf(counter->caps.name, 16, "adrv906x-ptp-tod%d", counter->id);
	counter->ptp_clk = ptp_clock_register(&counter->caps, tod->dev);
	if (IS_ERR(counter->ptp_clk)) {
		ret = PTR_ERR(counter->ptp_clk);
		counter->en = false;
		return ret;
	}

	adrv906x_tod_pps_irq_enable(counter, ADRV906X_HW_TOD_PPS_IRQ_ON);
	dev_info(tod->dev, "added counter %d as /dev/ptp%d",
		 counter->id, ptp_clock_index(counter->ptp_clk));

	return 0;
}

int adrv906x_tod_register_pll(struct ptp_clock_info *pll_caps)
{
	int i;

	if (!adrv906x_tod)
		return -ENODEV;

	for (i = 0; i < ADRV906X_HW_TOD_COUNTER_CNT; i++) {
		adrv906x_tod->counter[i].caps.adjfine = pll_caps->adjfine;
		adrv906x_tod->counter[i].caps.adjfreq = pll_caps->adjfreq;
		memcpy(adrv906x_tod->counter[i].caps.name, pll_caps->name, sizeof(adrv906x_tod->counter[i].caps.name));
	}

	return 0;
}
EXPORT_SYMBOL(adrv906x_tod_register_pll);

void adrv906x_tod_hw_disable_all(struct adrv906x_tod *tod)
{
	/* Disable debug outputs */
	iowrite32(0, tod->regs + ADRV906X_TOD_CFG_IO_CTRL);
	/* Disable all IRQs */
	iowrite32(ADRV906X_TOD_IRQ_MASK_MASK, tod->regs + ADRV906X_TOD_IRQ_MASK);
}

static void adrv906x_tod_get_version(struct adrv906x_tod *tod)
{
	u32 val;

	val = ioread32(tod->regs + ADRV906X_TOD_VERSION);
	tod->ver_major = FIELD_GET(ADRV906X_TOD_VERSION_MAJOR, val);
	tod->ver_minor = FIELD_GET(ADRV906X_TOD_VERSION_MINOR, val);
}

int adrv906x_tod_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct device_node *tod_np = NULL;
	struct adrv906x_tod_counter *counter;
	struct device_node *child;
	struct clk *lc_clk;
	struct clk *gc_clk;
	unsigned long rate;
	void __iomem *regs;
	u32 val;
	int ret;
	int i;

	adrv906x_tod = devm_kzalloc(dev, sizeof(*adrv906x_tod), GFP_KERNEL);
	if (!adrv906x_tod)
		return -ENOMEM;
	adrv906x_tod->dev = dev;

	if (!np) {
		dev_err(dev, "platform device data missing!");
		return -ENODEV;
	}

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs)) {
		ret = PTR_ERR(regs);
		return ret;
	}

	adrv906x_tod->regs = regs;

	adrv906x_tod_get_version(adrv906x_tod);
	dev_info(dev, "tod version %d.%d", adrv906x_tod->ver_major, adrv906x_tod->ver_minor);

	lc_clk = devm_clk_get(dev, "lc_clk");
	if (IS_ERR(lc_clk)) {
		dev_err(dev, "can not get 'lc_clk'");
		ret = PTR_ERR(lc_clk);
		goto err_out;
	}

	gc_clk = devm_clk_get(dev, "gc_clk");
	if (IS_ERR(gc_clk)) {
		dev_err(dev, "can not get 'gc_clk'");
		ret = PTR_ERR(gc_clk);
		goto err_out;
	}

	adrv906x_tod->gc_clk = gc_clk;
	adrv906x_tod->lc_clk = lc_clk;
	/* get the gc and local clock frequency from the clock */
	rate = clk_get_rate(adrv906x_tod->gc_clk);
	adrv906x_tod->gc_clk_freq_khz = (u32)div_u64((u64)rate, 1000);

	rate = clk_get_rate(adrv906x_tod->lc_clk);
	adrv906x_tod->lc_freq_khz = (u32)div_u64((u64)rate, 1000);

	ret = of_property_read_u32(np, "adi,pps-in-pulse-width-ms", &val);
	if (ret) {
		dev_info(dev, "'adi,pps-in-pulse-width-ms' not set, using 40ms");
		val = 40 * USEC_PER_MSEC;
	} else if (val >= 1000) {
		dev_err(dev, "'adi,pps-in-pulse-width-ms' out of range, using 40ms");
		val = 40 * USEC_PER_MSEC;
	}
	adrv906x_tod->pps_in_pulse_width_ms = val;

	adrv906x_tod->irq = platform_get_irq_byname(pdev, "pps");
	if (adrv906x_tod->irq < 0) {
		dev_err(dev, "dt: irq node missing");
		ret = -ENOENT;
		goto err_out;
	}

	ret = devm_request_irq(&pdev->dev, adrv906x_tod->irq,
			       adrv906x_tod_pps_isr, 0, pdev->name,
			       adrv906x_tod);
	if (ret) {
		dev_err(dev, "irq %d unavailable", adrv906x_tod->irq);
		ret = -ENOENT;
		goto err_out;
	}

	tod_np = of_get_child_by_name(np, "adrv906x-tod");
	if (!tod_np)
		goto err_out;

	mutex_init(&adrv906x_tod->reg_lock);

	adrv906x_tod_hw_disable_all(adrv906x_tod);

	child = NULL;
	for_each_child_of_node(tod_np, child) {
		ret = adrv906x_tod_add_counter(adrv906x_tod, child);
		if (ret) {
			dev_warn(dev, "cannot add tod counter: %s", child->full_name);
			continue;
		}
	}

	for (i = 0; i < ADRV906X_HW_TOD_CDC_DOMAIN_CNT; i++) {
		ret = of_property_read_u32_index(tod_np, "adi,cdc-delay-value", i, &val);
		if (ret)
			val = 0;
		adrv906x_tod->cdc.delay_cnt[i] = val;
	}

	adrv906x_tod_cfg_cdc_delay_all(adrv906x_tod);
	counter = &adrv906x_tod->counter[adrv906x_tod->tod_counter_src];
	if (counter->en) {
		ret = adrv906x_tod_extts_enable(counter, 1);
		if (ret) {
			dev_err(dev, "default tod counter enable failed");
			goto err_out_unreg;
		}
	}

	ret = of_property_read_u32(tod_np, "adi,default-tod-counter", &val);
	if (ret) {
		dev_warn(dev, "adi,default-tod-counter not set, using default %d", 0);
		adrv906x_tod->tod_counter_src = 0;
	} else if (adrv906x_tod->counter[val].en) {
		adrv906x_tod->tod_counter_src = val;
	} else {
		dev_err(dev, "selected default tod not enabled - exiting");
		goto err_out_unreg;
	}

	ret = of_property_read_u32(np, "adi,ppsx-pulse-width-ns", &val);
	if (ret) {
		dev_err(dev, "'adi,ppsx-pulse-width-ns' not set, using 10ms");
		val = TOD_PPSX_PULSE_WIDTH;
	}
	if (val >= (1000 * NSEC_PER_MSEC)) {
		dev_err(dev, "'adi,ppsx-pulse-width-ns' out of range, using 10ms");
		val = TOD_PPSX_PULSE_WIDTH;
	}
	adrv906x_tod->ppsx_pulse_width_ns = val;

	adrv906x_tod->external_pps = of_property_read_bool(np, "adi,external-pps");
	if (adrv906x_tod->external_pps) {
		dev_info(dev, "using external pps");
		adrv906x_tod_hw_external_pps_override(adrv906x_tod);
	}

	if (adrv906x_tod->ver_major == 2 && adrv906x_tod->ver_minor <= 2) {
		init_waitqueue_head(&adrv906x_tod->pps_queue);
		INIT_DELAYED_WORK(&adrv906x_tod->pps_work, adrv906x_tod_clear_soft_pps);
	}

	dev_info(dev, "adrv906x tod probe ok");

	return 0;

err_out_unreg:
	for (i = 0; i < ADRV906X_HW_TOD_COUNTER_CNT; i++)
		if (adrv906x_tod->counter[i].ptp_clk)
			ptp_clock_unregister(adrv906x_tod->counter[i].ptp_clk);

err_out:
	adrv906x_tod_hw_disable_all(adrv906x_tod);
	return ret;
}
EXPORT_SYMBOL(adrv906x_tod_probe);

int adrv906x_tod_remove(struct platform_device *pdev)
{
	u8 i;

	if (!adrv906x_tod)
		return -ENODEV;

	adrv906x_tod_hw_disable_all(adrv906x_tod);

	for (i = 0; i < ADRV906X_HW_TOD_COUNTER_CNT; i++) {
		if (adrv906x_tod->counter[i].en)
			adrv906x_tod_extts_enable(&adrv906x_tod->counter[i],
						  ADRV906X_HW_TOD_DISABLE);
		if (adrv906x_tod->counter[i].ptp_clk)
			ptp_clock_unregister(adrv906x_tod->counter[i].ptp_clk);
	}

	if (adrv906x_tod->ver_major == 2 && adrv906x_tod->ver_minor <= 2)
		cancel_delayed_work_sync(&adrv906x_tod->pps_work);

	mutex_destroy(&adrv906x_tod->reg_lock);

	return 0;
}
EXPORT_SYMBOL(adrv906x_tod_remove);

static const struct of_device_id ptp_adrv906x_tod_of_match[] = {
	{ .compatible = "adi,adrv906x-tod", },
	{},
};

MODULE_DEVICE_TABLE(of, ptp_adrv906x_tod_of_match);

static struct platform_driver ptp_adrv906x_tod_driver = {
	.driver			= {
		.name		= "adrv906x-tod",
		.of_match_table = ptp_adrv906x_tod_of_match,
	},
	.probe			= adrv906x_tod_probe,
	.remove			= adrv906x_tod_remove,
};

module_platform_driver(ptp_adrv906x_tod_driver);
