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
#include <linux/string.h>
#include <linux/io.h>
#include <linux/of.h>
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
#define   ADRV906X_TOD_CFG_TOD_OP_RD_TOD_MASK           GENMASK(6, 4)
#define   ADRV906X_TOD_CFG_TOD_OP_WR_TOD_PPS_MASK       GENMASK(10, 8)
#define   ADRV906X_TOD_CFG_TOD_OP_RD_TOD_PPS_MASK       GENMASK(14, 12)

#define ADRV906X_TOD_CFG_TV_NSEC                        (0x24U)
#define   ADRV906X_TOD_CFG_TV_NSEC_FRAC_NSEC_MASK       GENMASK(15, 0)
#define   ADRV906X_TOD_CFG_TV_NSEC_NSEC_MASK            GENMASK(31, 16)

#define ADRV906X_TOD_CFG_TV_SEC_0                       (0x28U)
#define   ADRV906X_TOD_CFG_TV_SEC_0_NSEC_MASK           GENMASK(15, 0)
#define   ADRV906X_TOD_CFG_TV_SEC_0_SEC_MASK            GENMASK(31, 16)

#define ADRV906X_TOD_CFG_TV_SEC_1                       (0x2CU)
#define   ADRV906X_TOD_CFG_TV_SEC_1_SEC_MASK            GENMASK(31, 0)

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

struct adrv906x_tod_lc_clk_cfg adrv906x_lc_clk_cfg[HW_TOD_LC_CLK_FREQ_CNT] = {
	[HW_TOD_LC_100_P_000_M] = { 100000U, 10U, 0x0000U, 0x00U },
	[HW_TOD_LC_122_P_880_M] = { 122880U, 8U,  0x2355U, 0x04U },
	[HW_TOD_LC_125_P_000_M] = { 125000U, 8U,  0x0000U, 0x00U },
	[HW_TOD_LC_156_P_250_M] = { 156250U, 6U,  0x6666U, 0x01U },
	[HW_TOD_LC_245_P_760_M] = { 245760U, 4U,  0x11AAU, 0x02U },
	[HW_TOD_LC_250_P_000_M] = { 250000U, 4U,  0x0000U, 0x00U },
	[HW_TOD_LC_312_P_500_M] = { 312500U, 3U,  0x3333U, 0x08U },
	[HW_TOD_LC_322_P_265_M] = { 322265U, 3U,  0x1A60U, 0x20U },
	[HW_TOD_LC_390_P_625_M] = { 390625U, 2U,  0x8F5CU, 0x10U },
	[HW_TOD_LC_491_P_520_M] = { 491520U, 2U,  0x08D5U, 0x04U },
	[HW_TOD_LC_500_P_000_M] = { 500000U, 2U,  0x0000U, 0x00U },
	[HW_TOD_LC_983_P_040_M] = { 983040U, 1U,  0x046AU, 0x02U }
};

/**
 * @brief Configure the ToD IP for the system clock frequency
 * @param counter Context struct
 * @return 0 Success
 * @return -EINVAL The chosen clock frequency isn't supported.
 */
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
			ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_INCR, wr_val);
			err = 0;
			break;
		}
	}

	return err;
}

/**
 * @brief Convert a kernel tstamp to HW format.
 * @param tstamp HW tstamp container
 * @param ts Kernel tstamp container
 */
static inline void timespec_to_tstamp(struct adrv906x_tod_tstamp *tstamp,
				      const struct timespec64 *ts)
{
	tstamp->nanoseconds = ts->tv_nsec;
	tstamp->frac_nanoseconds = 0U;
	tstamp->seconds = ts->tv_sec;
}

/**
 * @brief Convert a HW tstamp to kernel format.
 * @param tstamp HW tstamp container
 * @param ts Kernel tstamp container
 */
static inline void tstamp_to_timespec(struct timespec64 *ts,
				      const struct adrv906x_tod_tstamp *tstamp)
{
	ts->tv_sec = tstamp->seconds;

	if (tstamp->frac_nanoseconds < (TOD_FRAC_NANO_NUM / 2))
		ts->tv_nsec = tstamp->nanoseconds;
	else
		ts->tv_nsec = tstamp->nanoseconds + 1U;
}

/**
 * @brief Generate a command mask for the HW register
 * @param op_flag HW operation
 * @param is_pps Operational mode
 * @param tod_idx Index of the ToD the operation should be for
 * @return Returns the mask to use in the HW register
 */
static inline u32 adrv906x_tod_op_to_mask(u8 op_flag, bool is_pps, u32 tod_idx)
{
	u32 mask = 0U;

	if (op_flag == HW_TOD_TRIG_OP_WR) {
		if (is_pps)
			mask = FIELD_PREP(ADRV906X_TOD_CFG_TOD_OP_WR_TOD_PPS_MASK, tod_idx);
		else
			mask = FIELD_PREP(ADRV906X_TOD_CFG_TOD_OP_WR_TOD_MASK, tod_idx);
	} else {
		if (is_pps)
			mask = FIELD_PREP(ADRV906X_TOD_CFG_TOD_OP_RD_TOD_PPS_MASK, tod_idx);
		else
			mask = FIELD_PREP(ADRV906X_TOD_CFG_TOD_OP_RD_TOD_MASK, tod_idx);
	}

	return mask;
}

/**
 * @brief  Obtain a Golden Count from HW for the given ToD
 * @param counter Context struct for ToD counter
 * @return Golden Count for given ToD counter
 */
static u64 adrv906x_tod_hw_gc_get_cnt(struct adrv906x_tod_counter *counter)
{
	struct adrv906x_tod *tod = counter->parent;
	u32 gc_reg_cnt[2] = { 0U, 0U };
	u64 gc_cnt;
	u32 gc_rd = 1U;

	/* Write the OP_GC:RD_GC_MASK to latch the GC counter register */
	ADRV906X_REG_WRITE(tod, ADRV906X_TOD_CFG_OP_GC, gc_rd);
	ADRV906X_REG_WRITE(tod, ADRV906X_TOD_CFG_OP_GC, 0);

	/* Read back the Golden Counter */
	gc_reg_cnt[0] = ADRV906X_REG_READ(tod, ADRV906X_TOD_STAT_GC_0);
	gc_reg_cnt[1] = ADRV906X_REG_READ(tod, ADRV906X_TOD_STAT_GC_1);

	gc_cnt = gc_reg_cnt[0] | ((u64)(gc_reg_cnt[1] & 0xFFFFU) << 32);

	return gc_cnt;
}

/**
 * @brief  Write a Golden Count to the HW for the given ToD
 * @param counter Context struct for ToD counter
 * @param gc_cnt The Golden Count to write to HW
 */
static void adrv906x_tod_hw_gc_set_cnt(struct adrv906x_tod_counter *counter, u64 gc_cnt)
{
	struct adrv906x_tod *tod = counter->parent;
	u32 gc_reg_cnt[2] = { 0U, 0U };

	gc_reg_cnt[0] = gc_cnt & 0xFFFFFFFFU;
	gc_reg_cnt[1] = (gc_cnt >> 32) & 0xFFFFU;

	/* Write the GC value */
	ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_OP_GC_VAL_0, gc_reg_cnt[0]);
	ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_OP_GC_VAL_1, gc_reg_cnt[1]);
}

/**
 * @brief Helper function to clear the 'soft PPS' and trigger pending operations
 * @param work Context struct
 */
static void adrv906x_tod_clear_soft_pps(struct work_struct *work)
{
	struct adrv906x_tod *tod = container_of(work, struct adrv906x_tod, pps_work.work);

	atomic_set(&tod->pps_state, 0);

	wake_up_all(&tod->pps_queue);
}

/**
 * @brief Trigger or clear the flag for an operation
 * @note All the registers required for or provided by an operation must be written or read in
 * advance of calling this function
 * @param counter Context struct
 * @param op_flag Choice of operation
 * @param is_pps Non-zero if a PPS operation
 * @param set_flag Non-zero to trigger operation, zero to clear it
 */
static void adrv906x_tod_hw_op_trig(struct adrv906x_tod_counter *counter, u8 op_flag, bool is_pps,
				    bool set_flag)
{
	struct adrv906x_tod *tod = counter->parent;
	u8 tod_idx = BIT(counter->id);
	u32 mask, val;

	mask = adrv906x_tod_op_to_mask(op_flag, is_pps, tod_idx);

	val = ADRV906X_REG_READ(tod, ADRV906X_TOD_CFG_TOD_OP);
	if (set_flag)
		val |= mask;
	else
		val &= ~mask;
	ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_TOD_OP, val);
}

/**
 * @brief Convenience function to set a trigger
 * @param counter Context struct
 * @param op_flag Operation selection
 */
static void adrv906x_tod_hw_op_trig_set(struct adrv906x_tod_counter *counter, u8 op_flag)
{
	adrv906x_tod_hw_op_trig(counter, op_flag, counter->trigger_mode, true);
}

/**
 * @brief Convenience function to clear a trigger
 * @param counter Context struct
 * @param op_flag Operation selection
 */
static void adrv906x_tod_hw_op_trig_clear(struct adrv906x_tod_counter *counter, u8 op_flag)
{
	adrv906x_tod_hw_op_trig(counter, op_flag, counter->trigger_mode, false);
}

/**
 * @brief Polls the provided register for the provided bit mask
 * @param counter Context struct
 * @param regaddr Address (offset from base) to poll
 * @param bit_mask Bit mask to poll for
 * @param p_delay Timespan to wait between each poll
 * @param done_high If non-zero, the function matches for '1's at the bit mask's indices
 * @return 0 Operation completed as expected
 * @return -EAGAIN The trigger timed out
 */
static int adrv906x_tod_hw_op_poll_reg(struct adrv906x_tod_counter *counter, u32 regaddr,
				       u32 bit_mask, const struct adrv906x_tod_trig_delay *p_delay,
				       bool done_high)
{
	u32 delay_cnt = TOD_MAX_DELAY_COUNT;
	u8 done = 0U;
	int err = 0;
	u32 val;

	while (!done && (delay_cnt != 0U)) {
		ndelay(p_delay->ns);
		val = ADRV906X_REG_READ(counter->parent, regaddr);

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

/**
 * @brief Convenience function to poll an operation completion
 * @param counter Context struct
 * @param op_flag Selected operation to poll for
 * @param p_delay Time between each poll
 * @return See adrv906x_tod_hw_op_poll_reg()
 */
static int adrv906x_tod_hw_op_poll(struct adrv906x_tod_counter *counter, u8 op_flag,
				   const struct adrv906x_tod_trig_delay *p_delay)
{
	u8 tod_idx = BIT(counter->id);
	u32 mask;

	mask = adrv906x_tod_op_to_mask(op_flag, counter->trigger_mode, tod_idx);
	return adrv906x_tod_hw_op_poll_reg(counter, ADRV906X_TOD_STAT_TOD_OP, mask, p_delay, true);
}

/**
 * @brief Compensate tstamps to write to HW register before a set operation
 * @param counter Context struct
 * @param tstamp Tstamp to compensate
 * @param trig_delay Timespan to compensate
 */
static void adrv906x_tod_compensate_tstamp(struct adrv906x_tod_counter *counter,
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
		tstamp->frac_nanoseconds =
			(u16)((old_tstamp.frac_nanoseconds + frac_ns_tstamp) - TOD_FRAC_NANO_NUM);
		tstamp->nanoseconds = old_tstamp.nanoseconds + trig_delay->ns + 1U;
	}

	/* Update the second part in the tstamp */
	if (tstamp->nanoseconds >= NSEC_PER_SEC) {
		seconds = div_u64_rem(tstamp->nanoseconds, NSEC_PER_SEC, &ns);
		tstamp->nanoseconds = ns;
		tstamp->seconds = old_tstamp.seconds + seconds;
	} else {
		tstamp->seconds = old_tstamp.seconds;
	}
}

/**
 * @brief Write tstamp to HW
 * @param counter Context struct
 * @param tstamp Tstamp to write
 */
static void adrv906x_tod_hw_settstamp_to_reg(struct adrv906x_tod_counter *counter,
					     const struct adrv906x_tod_tstamp *tstamp)
{
	struct adrv906x_tod *tod = counter->parent;
	u32 reg_tstamp[3] = { 0U, 0U, 0U };

	reg_tstamp[0] |= FIELD_PREP(ADRV906X_TOD_CFG_TV_NSEC_FRAC_NSEC_MASK,
				    tstamp->frac_nanoseconds);
	reg_tstamp[0] |= FIELD_PREP(ADRV906X_TOD_CFG_TV_NSEC_NSEC_MASK,
				    tstamp->nanoseconds & 0xFFFF);
	reg_tstamp[1] |= FIELD_PREP(ADRV906X_TOD_CFG_TV_SEC_0_NSEC_MASK,
				    (tstamp->nanoseconds & 0xFFFF0000) >> 16);
	reg_tstamp[1] |= FIELD_PREP(ADRV906X_TOD_CFG_TV_SEC_0_SEC_MASK,
				    (tstamp->seconds & 0xFFFF));
	reg_tstamp[2] |= FIELD_PREP(ADRV906X_TOD_CFG_TV_SEC_1_SEC_MASK,
				    (tstamp->seconds & 0xFFFFFFFF0000) >> 16);

	ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_TV_NSEC, reg_tstamp[0]);
	ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_TV_SEC_0, reg_tstamp[1]);
	ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_TV_SEC_1, reg_tstamp[2]);
}

/**
 * @brief Read tstamp from HW
 * @param counter Context struct
 * @param tstamp Return struct for tstamp
 */
static void adrv906x_tod_hw_gettstamp_from_reg(struct adrv906x_tod_counter *counter,
					       struct adrv906x_tod_tstamp *tstamp)
{
	struct adrv906x_tod *tod = counter->parent;
	u32 reg_tstamp[3] = { 0U };
	u8 tod_idx;
	u32 val;

	tod_idx = counter->id;

	val = ADRV906X_REG_READ(tod, ADRV906X_TOD_CFG_IO_CTRL);
	val &= ~ADRV906X_TOD_CFG_IO_CTRL_TOD_STAT_SEL_MASK;
	val |= ADRV906X_TOD_CFG_IO_CTRL_TOD_STAT_SEL(BIT(tod_idx));
	ADRV906X_REG_WRITE(tod, ADRV906X_TOD_CFG_IO_CTRL, val);

	reg_tstamp[0] = ADRV906X_REG_READ(tod, ADRV906X_TOD_STAT_TV_NSEC);
	reg_tstamp[1] = ADRV906X_REG_READ(tod, ADRV906X_TOD_STAT_TV_SEC_0);
	reg_tstamp[2] = ADRV906X_REG_READ(tod, ADRV906X_TOD_STAT_TV_SEC_1);

	tstamp->frac_nanoseconds = reg_tstamp[0] & 0xFFFFU;
	tstamp->nanoseconds = ((reg_tstamp[0] >> 16) & 0xFFFFU) | ((reg_tstamp[1] & 0xFFFFU) << 16);
	tstamp->seconds = ((reg_tstamp[1] >> 16) & 0xFFFFU) | (reg_tstamp[2] << 16);
}

/**
 * @brief Prepare the HW for an operation
 * @param counter Context struct
 */
static void adrv906x_tod_hw_set_trigger_delay(struct adrv906x_tod_counter *counter)
{
	u64 gc_cnt = 0U;

	/* Set the trigger delay to GC value register */
	gc_cnt = adrv906x_tod_hw_gc_get_cnt(counter);
	gc_cnt += counter->trig_delay_tick;
	adrv906x_tod_hw_gc_set_cnt(counter, gc_cnt);
}

/**
 * @brief Get the trigger delay for the referenced counter
 * @param counter Context struct
 * @param trig_delay Return struct for trigger delay
 */
static void adrv906x_tod_get_trigger_delay(struct adrv906x_tod_counter *counter,
					   struct adrv906x_tod_trig_delay *trig_delay)
{
	struct adrv906x_tod *tod = counter->parent;

	/*
	 * The trigger delay value depends on the counter->trig_delay_tick.
	 * adrv906x_tod_trig_delay.ns = counter->trig_delay_tick * 1e6 / tod->gc_clk_freq_khz
	 * adrv906x_tod_trig_delay.frac_ns = counter->trig_delay_tick * 1e6 % tod->gc_clk_freq_khz
	 * 1e6 is used to calculate the nano-second of the trigger tick so that the
	 * "counter->trig_delay_tick * 1e6" will not overflow unless counter->trig_delay_tick beyond
	 * the value "2^44".
	 */
	trig_delay->ns = div_u64_rem(counter->trig_delay_tick * USEC_PER_SEC,
				     tod->gc_clk_freq_khz,
				     &trig_delay->rem_ns);
}

/**
 * @brief Make the referenced counter to continue counting from the provided tstamp
 * @param counter Context struct
 * @param vector Tstamp to set
 * @return See adrv906x_tod_hw_op_poll()
 */
static int adrv906x_tod_hw_settstamp(struct adrv906x_tod_counter *counter,
				     const struct adrv906x_tod_tstamp *vector)
{
	struct adrv906x_tod_trig_delay trig_delay = { 0U, 0U };
	struct adrv906x_tod_tstamp tstamp = { 0U, 0U, 0U };
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

/**
 * @brief Get the current tstamp from the reference counter
 * @param counter Context structure
 * @param tstamp Return structure for tstamp
 * @return See adrv906x_tod_hw_op_poll()
 */
static int adrv906x_tod_hw_get_tstamp(struct adrv906x_tod_counter *counter,
				      struct adrv906x_tod_tstamp *tstamp)
{
	struct adrv906x_tod_trig_delay trig_delay = { 0U, 0U };
	struct adrv906x_tod *tod = counter->parent;
	int err;

	adrv906x_tod_get_trigger_delay(counter, &trig_delay);

	if (counter->trigger_mode == HW_TOD_TRIG_MODE_GC)
		adrv906x_tod_hw_set_trigger_delay(counter);
	else
		/* We set the delay to time out polling after a second. */
		trig_delay.ns = 100 * NSEC_PER_MSEC;

	/* In PPS mode and HW version 2.2 and below, only trigger reads when PPS signal is low. */
	if (counter->trigger_mode == HW_TOD_TRIG_MODE_PPS &&
	    tod->ver_major == 2 && tod->ver_minor <= 2)
		wait_event(tod->pps_queue, atomic_read(&tod->pps_state) == 0);

	adrv906x_tod_hw_op_trig_set(counter, HW_TOD_TRIG_OP_RD);
	err = adrv906x_tod_hw_op_poll(counter, HW_TOD_TRIG_OP_RD, &trig_delay);

	if (!err)
		adrv906x_tod_hw_gettstamp_from_reg(counter, tstamp);

	adrv906x_tod_hw_op_trig_clear(counter, HW_TOD_TRIG_OP_RD);

	return err;
}

/**
 * @brief Adjust the counter by the input time, where a negative value is backwards in time
 * @param counter Context struct
 * @param delta The adjustment in signed ns
 * @return See adrv906x_tod_hw_op_poll_reg()
 */
static int adrv906x_tod_hw_adjust_time(struct adrv906x_tod_counter *counter, s64 delta)
{
	struct adrv906x_tod_trig_delay trig_delay = { 0U, 0U };
	struct adrv906x_tod_tstamp ts0 = { 0U }, ts1 = { 0U };
	u64 gc0, gc1, gc2;
	struct timespec64 tmp;
	ktime_t kt0, kt1;
	u32 op_mask;
	int err;

	adrv906x_tod_get_trigger_delay(counter, &trig_delay);

	/*
	 * The time adjustment will need to know the tstamp for a specific GC value
	 * to be able to adjust the time correctly.
	 *     gc0   gc1   gc2
	 * |---*-----*-----*-----|
	 *           ts0   ts1
	 */

	gc0 = adrv906x_tod_hw_gc_get_cnt(counter);

	gc1 = gc0 + counter->trig_delay_tick;
	adrv906x_tod_hw_gc_set_cnt(counter, gc1);

	adrv906x_tod_hw_op_trig(counter, HW_TOD_TRIG_OP_RD, false, true);
	op_mask = FIELD_PREP(ADRV906X_TOD_CFG_TOD_OP_RD_TOD_MASK, BIT(counter->id));
	err = adrv906x_tod_hw_op_poll_reg(counter, ADRV906X_TOD_STAT_TOD_OP, op_mask, &trig_delay,
					  true);
	adrv906x_tod_hw_gettstamp_from_reg(counter, &ts0);
	adrv906x_tod_hw_op_trig(counter, HW_TOD_TRIG_OP_RD, false, false);
	if (err)
		return err;

	/*
	 * We leverage the 'delta' variable to do the adjustment calculation before
	 * converting to the ADRV906X format.
	 */
	kt0 = ktime_set(ts0.seconds, ts0.nanoseconds);
	kt1 = ktime_add_ns(kt0, delta);
	tmp = ktime_to_timespec64(kt1);
	timespec_to_tstamp(&ts1, &tmp);
	ts1.frac_nanoseconds = ts0.frac_nanoseconds;

	adrv906x_tod_compensate_tstamp(counter, &ts1, &trig_delay);

	gc2 = gc1 + counter->trig_delay_tick;
	adrv906x_tod_hw_gc_set_cnt(counter, gc2);
	adrv906x_tod_hw_settstamp_to_reg(counter, &ts1);

	adrv906x_tod_hw_op_trig(counter, HW_TOD_TRIG_OP_WR, false, true);
	op_mask = FIELD_PREP(ADRV906X_TOD_CFG_TOD_OP_WR_TOD_MASK, BIT(counter->id));
	err = adrv906x_tod_hw_op_poll_reg(counter, ADRV906X_TOD_STAT_TOD_OP, op_mask, &trig_delay,
					  true);

	adrv906x_tod_hw_op_trig(counter, HW_TOD_TRIG_OP_WR, false, false);

	return err;
}

/**
 * @brief Enable the ToD output in the CDC domain
 * @param counter Context struct
 * @param enable Enable flag, non-zero to enable
 * @return 0 Success
 * @return -ENODEV Requesting to enable output for a disabled counter
 */
static int adrv906x_tod_hw_cdc_output_enable(struct adrv906x_tod_counter *counter, u8 enable)
{
	struct adrv906x_tod *tod = counter->parent;
	u8 tod_idx;
	u32 val = 0U;
	int i;

	if (counter->en && counter->id != TOD_INTERNAL_GNSS)
		tod_idx = counter->id;
	else
		return -ENODEV;

	if (enable)
		val |= BIT(tod_idx);

	for (i = 0; i < ADRV906X_HW_TOD_CDC_DOMAIN_CNT; i++)
		ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_TSU_TOD + i * 0x4, val);

	return 0;
}

/**
 * @brief Instruct HW to enable the ToD output
 * @param counter Context struct
 * @param enable Enable flag, non-zero to enable
 * @return See adrv906x_tod_hw_op_poll_reg()
 */
static int adrv906x_tod_hw_extts_enable(struct adrv906x_tod_counter *counter, u8 enable)
{
	struct adrv906x_tod_trig_delay trig_delay = { 0U, 0U };
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

	val = ADRV906X_REG_READ(tod, ADRV906X_TOD_CFG_IO_SOURCE);
	val &= ~ADRV906X_TOD_CFG_IO_TOD_OUT_SRC_MASK;
	val |= ADRV906X_TOD_CFG_IO_WR_OUTPUT_CFG_MASK;

	if (enable)
		val |= ADRV906X_TOD_CFG_IO_TOD_OUT_SRC_SEL(BIT(tod_idx));
	else
		val &= ~ADRV906X_TOD_CFG_IO_TOD_OUT_SRC_SEL(BIT(tod_idx));
	ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_IO_SOURCE, val);

	ret = adrv906x_tod_hw_op_poll_reg(counter, ADRV906X_TOD_CFG_IO_SOURCE,
					  ADRV906X_TOD_CFG_IO_WR_OUTPUT_CFG_MASK, &trig_delay,
					  false);

	return ret;
}

/**
 * @brief Enable the interrupt lines for the referenced counter
 * @param counter Context struct
 * @param enable Enable flag, non-zero to enable
 * @return 0 Success
 * @return -ENODEV Referred counter not active
 */
static int adrv906x_tod_pps_irq_enable(struct adrv906x_tod_counter *counter, u8 enable)
{
	struct adrv906x_tod *tod = counter->parent;
	int tod_idx = counter->id;
	u32 val;

	if (!counter->en)
		return -ENODEV;

	val = ADRV906X_REG_READ(tod, ADRV906X_TOD_IRQ_MASK);

	if (enable)
		val &= ~BIT(tod_idx);
	else
		val |= BIT(tod_idx);

	ADRV906X_REG_WRITE(tod, ADRV906X_TOD_IRQ_MASK, val);

	return 0;
}

/**
 * @brief Configure interrupt line to be driven by the external PPS input
 * @param tod Context struct
 */
static void adrv906x_tod_hw_pps_irq_external_enable(struct adrv906x_tod *tod)
{
	u32 val = 0U;

	val = ADRV906X_REG_READ(tod, ADRV906X_TOD_IRQ_MASK);
	val &= ~ADRV906X_TOD_IRQ_MASK_EXTERNAL_PPS;
	ADRV906X_REG_WRITE(tod, ADRV906X_TOD_IRQ_MASK, val);
}

/**
 * @brief Disable all interrupts
 * @param tod Context struct
 */
static void adrv906x_tod_hw_pps_irq_disable_all(struct adrv906x_tod *tod)
{
	u32 val = ADRV906X_TOD_IRQ_MASK_EXTERNAL_PPS |
		  ADRV906X_TOD_IRQ_MASK_INTERNAL_0 |
		  ADRV906X_TOD_IRQ_MASK_INTERNAL_1 |
		  ADRV906X_TOD_IRQ_MASK_INTERNAL_GNSS;

	ADRV906X_REG_WRITE(tod, ADRV906X_TOD_IRQ_MASK, val);
}

/**
 * @brief Select the referenced counter as the PPS source and en-/disable the PPS output
 * @note This function doesn't change the output if the external PPS is enabled
 * @param counter Context struct
 * @param enable Enable flag, non-zero to enable
 * @return See adrv906x_tod_hw_op_poll_reg()
 */
static int adrv906x_tod_hw_pps_enable(struct adrv906x_tod_counter *counter, u8 enable)
{
	struct adrv906x_tod_trig_delay trig_delay = { 0U, 0U };
	struct adrv906x_tod *tod = counter->parent;
	u32 val;
	int ret;

	adrv906x_tod_get_trigger_delay(counter, &trig_delay);
	adrv906x_tod_hw_set_trigger_delay(counter);

	val = ADRV906X_REG_READ(tod, ADRV906X_TOD_CFG_IO_SOURCE);
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
	ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_IO_SOURCE, val);

	ret = adrv906x_tod_hw_op_poll_reg(counter, ADRV906X_TOD_CFG_IO_SOURCE,
					  ADRV906X_TOD_CFG_IO_WR_OUTPUT_CFG_MASK, &trig_delay,
					  false);

	return ret;
}

/**
 * @brief Enable or disable the PPS output for the referenced counter
 * @param counter Context struct
 * @param enable Flag to enable or disable the PPS
 * @return See adrv906x_tod_hw_pps_enable()
 */
static int adrv906x_tod_pps_enable(struct adrv906x_tod_counter *counter, u8 enable)
{
	struct adrv906x_tod *tod = counter->parent;
	int err;

	mutex_lock(&tod->reg_lock);
	err = adrv906x_tod_hw_pps_enable(counter, enable);
	mutex_unlock(&tod->reg_lock);

	return err;
}

static irqreturn_t adrv906x_tod_pps_isr(int irq, void *dev_id)
{
	struct adrv906x_tod *tod = dev_id;
	struct adrv906x_tod_counter *counter;
	struct ptp_clock_event event;
	u32 irq_val;
	u8 i;

	irq_val = ADRV906X_REG_READ(tod, ADRV906X_TOD_IRQ_STATUS);
	ADRV906X_REG_WRITE(tod, ADRV906X_TOD_IRQ_EVENT, irq_val);

	for (i = 0U; i < ADRV906X_HW_TOD_COUNTER_CNT; i++) {
		if (irq_val & BIT(i) || (tod->external_pps &&
					 irq_val == ADRV906X_TOD_IRQ_MASK_EXTERNAL_PPS)) {
			counter = &tod->counter[i];
			if (counter->en) {
				event.type = PTP_CLOCK_PPS;
				ptp_clock_event(counter->ptp_clk, &event);
			}
		}
	}

	if (irq_val & ADRV906X_TOD_IRQ_MASK_EXTERNAL_PPS &&
	    tod->ver_major == 2 && tod->ver_minor <= 2) {
		atomic_set(&tod->pps_state, 1);
		schedule_delayed_work(&tod->pps_work, msecs_to_jiffies(tod->pps_in_pulse_width_ms));
	}

	return IRQ_HANDLED;
}

/**
 * @brief Configure the PPSX output
 * @note The HW is restricted to a PPS pulse of configurable width
 * @param counter Context struct
 * @param rq Request struct
 */
static void adrv906x_tod_hw_cfg_ppsx(struct adrv906x_tod_counter *counter,
				     struct ptp_perout_request *rq)
{
	struct adrv906x_tod *tod = counter->parent;
	u32 stop, val;

	val = ADRV906X_REG_READ(tod, ADRV906X_TOD_CFG_TEST_OUT_SRC);
	val &= ~ADRV906X_TOD_CFG_TEST_OUT_SRC_PPSX_SRC_MASK;

	if (!(rq->period.sec == 0 && rq->period.nsec == 0U)) {
		ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_PPSX_START, rq->start.nsec);
		stop = (rq->start.nsec + tod->ppsx_pulse_width_ns) & 0xFFFFFFFFU;
		ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_PPSX_STOP, stop);

		val |= ADRV906X_TOD_CFG_TEST_OUT_SRC_PPSX_SRC(BIT(counter->id));
	}

	ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_TEST_OUT_SRC, val);
}

/**
 * @brief Configure the periodic output for the referenced counter
 * @param counter Counter struct
 * @param rq Request struct
 */
static void adrv906x_tod_perout_enable(struct adrv906x_tod_counter *counter,
				       struct ptp_perout_request *rq)
{
	struct adrv906x_tod *tod = counter->parent;

	mutex_lock(&tod->reg_lock);
	adrv906x_tod_hw_cfg_ppsx(counter, rq);
	mutex_unlock(&tod->reg_lock);
}

/**
 * @brief Configure the CDC parameters for the referenced counter
 * @param counter Context struct
 */
static void adrv906x_tod_cfg_cdc_delay_set(struct adrv906x_tod_counter *counter)
{
	struct adrv906x_tod *tod = counter->parent;
	u32 i;

	for (i = 0U; i < ADRV906X_HW_TOD_CDC_DOMAIN_CNT; i++)
		ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_CDC_DELAY + i * sizeof(u32),
					tod->cdc.delay_cnt[i]);

	/* According to the user manual, all CFG_CDC_DELAY:CDC register fields
	 * shall have the same value. So we just pick the first one.
	 */
	adrv906x_tod_cfg_cdc_delay = tod->cdc.delay_cnt[0];
}

/**
 * @brief Initialize the counter HW
 * @param counter Context struct
 * @return See adrv906x_tod_cfg_lc_clk()
 */
static int adrv906x_tod_module_init(struct adrv906x_tod_counter *counter)
{
	struct adrv906x_tod *tod = counter->parent;
	u32 val;
	int ret;

	/* Update the ns and frac_ns part to the CFG_INCR */
	ret = adrv906x_tod_cfg_lc_clk(counter);
	/* Enable the ToD counter */
	if (!ret) {
		val = ADRV906X_REG_READ(tod, ADRV906X_TOD_CFG_INCR);
		val |= ADRV906X_TOD_CFG_INCR_CFG_TOD_CNT_EN_MASK;
		ADRV906X_REG_WRITE_DUAL(tod, ADRV906X_TOD_CFG_INCR, val);
	}

	return ret;
}

/**
 * @brief Parse the ToD config from the device tree
 * @param counter Context struct
 * @param np Pointer to device tree node
 */
static void adrv906x_tod_dt_parse(struct adrv906x_tod_counter *counter, struct device_node *np)
{
	struct adrv906x_tod *tod = counter->parent;
	struct device *dev = tod->dev;
	int ret;
	u32 val;

	counter->trigger_mode = of_property_read_bool(np, "adi,pps-mode");
	dev_info(dev, "tod trigger mode: %s", counter->trigger_mode ==
		 HW_TOD_TRIG_MODE_GC ? "gc mode" : "pps mode");

	ret = of_property_read_u32(np, "adi,trigger-delay-tick", &val);
	if (ret) {
		/* Use a default of 500 us based on the provided clock speed */
		val = tod->gc_clk_freq_khz * 500U / 1000U;
		dev_info(dev, "'adi,trigger-delay-tick' not set, using '%u'", val);
	}
	counter->trig_delay_tick = val;
}

/**
 * @brief Configure CDC and enable tstamp output for the referenced counter
 * @param counter Context struct
 * @param enable Enable flag, non-zero to enable
 * @return See adrv906x_tod_hw_extts_enable() or adrv906x_tod_hw_cdc_output_enable()
 */
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

/**
 * @brief Configure the tstamp output
 * @param counter Context struct
 * @param rq Request struct
 * @param enable Enable flag, non-zero to enable
 * @return 0 Success
 * @return -EOPNOTSUPP Unsupported request
 * @return -EINVAL Only aligned PPS pulses are supported
 */
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
			       rq->perout.period.nsec != 0U))
			return -EINVAL;

		if (enable && (rq->perout.start.nsec + tod->ppsx_pulse_width_ns)
		    >= NSEC_PER_SEC) {
			dev_err(tod->dev, "periodic pulse crosses 1 second boundary");
			return -EINVAL;
		}

		/* Enable ppsx for periodic output for given tod counter */
		adrv906x_tod_perout_enable(counter, &rq->perout);
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

/**
 * @brief Set tstamp for the referenced counter
 * @param counter Context struct
 * @param ts tstamp to set
 * @return See adrv906x_tod_hw_settstamp()
 */
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

/**
 * @brief Shift time for the referenced counter
 * @param counter Context struct
 * @param delta Time to shift counter
 * @return See adrv906x_tod_hw_adjust_time()
 */
static int adrv906x_tod_adjtime(struct adrv906x_tod_counter *counter, s64 delta)
{
	struct adrv906x_tod *tod = counter->parent;
	int err;

	mutex_lock(&tod->reg_lock);
	err = adrv906x_tod_hw_adjust_time(counter, delta);
	mutex_unlock(&tod->reg_lock);

	return err;
}

/**
 * @brief Get the tstamp for the referenced counter
 * @param counter Context struct
 * @param ts Time to shift counter
 * @param sts System tstamp
 * @return See adrv906x_tod_hw_get_tstamp()
 */
static int adrv906x_tod_gettimex(struct adrv906x_tod_counter *counter,
				 struct timespec64 *ts,
				 struct ptp_system_timestamp *sts)
{
	struct adrv906x_tod *tod = counter->parent;
	struct adrv906x_tod_tstamp tstamp;
	int err;

	mutex_lock(&tod->reg_lock);
	ptp_read_system_prets(sts);
	err = adrv906x_tod_hw_get_tstamp(counter, &tstamp);
	ptp_read_system_postts(sts);
	tstamp_to_timespec(ts, &tstamp);
	mutex_unlock(&tod->reg_lock);

	return err;
}

/**
 * @brief Configure the CDC delay for all active counters
 * @param tod Context struct
 */
static void adrv906x_tod_cfg_cdc_delay_all(struct adrv906x_tod *tod)
{
	int i;

	for (i = 0; i < ADRV906X_HW_TOD_COUNTER_CNT; i++) {
		if (tod->counter[i].en) {
			adrv906x_tod_cfg_cdc_delay_set(&tod->counter[i]);
			break;
		}
	}
}

/**
 * @brief Enable the external PPS override
 * @param tod Context struct
 */
static void adrv906x_tod_hw_external_pps_override(struct adrv906x_tod *tod)
{
	adrv906x_tod_hw_pps_irq_disable_all(tod);
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
	.adjtime	= &adrv906x_phc_adjtime,
	.gettimex64	= &adrv906x_phc_gettimex,
	.getcrosststamp = NULL,
	.settime64	= &adrv906x_phc_settime,
	.enable		= &adrv906x_phc_enable,
	.do_aux_work	= NULL,  /* Use the aux */
};

/**
 * @brief  Obtain and apply configuration for the referenced counter
 * @param tod Context struct
 * @param np Pointer to device tree node
 * @return 0 Success
 * @return -EINVAL Invalid counter selected or invalid DT node. See kernel log.
 */
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

	adrv906x_tod_dt_parse(counter, np);
	ret = adrv906x_tod_module_init(counter);
	if (ret) {
		counter->en = false;
		return ret;
	}

	counter->caps = adrv906x_tod_caps;
	snprintf(counter->caps.name, 16U, "adrv906x-ptp-tod%d", counter->id);
	counter->ptp_clk = ptp_clock_register(&counter->caps, tod->dev);
	if (IS_ERR(counter->ptp_clk)) {
		ret = PTR_ERR(counter->ptp_clk);
		counter->en = false;
		return ret;
	}

	adrv906x_tod_pps_irq_enable(counter, ADRV906X_ENABLE);
	dev_info(tod->dev, "added counter %d as /dev/ptp%d",
		 counter->id, ptp_clock_index(counter->ptp_clk));

	return 0;
}

/**
 * @brief Register capabilities to adjust a PLL supplying clock to the counters
 * @param pll_caps PLL capabilities struct
 * @return 0 Success
 * @return -ENODEV No ToD device instantiated
 */
int adrv906x_tod_register_pll(struct ptp_clock_info *pll_caps)
{
	int i;

	if (!adrv906x_tod)
		return -ENODEV;

	for (i = 0; i < ADRV906X_HW_TOD_COUNTER_CNT; i++) {
		adrv906x_tod->counter[i].caps.adjfine = pll_caps->adjfine;
		memcpy(adrv906x_tod->counter[i].caps.name, pll_caps->name,
		       sizeof(adrv906x_tod->counter[i].caps.name));
	}

	return 0;
}
EXPORT_SYMBOL(adrv906x_tod_register_pll);

/**
 * @brief Disable all counter outputs
 * @param tod Context struct
 */
static void adrv906x_tod_hw_disable_all(struct adrv906x_tod *tod)
{
	/* Disable debug outputs */
	ADRV906X_REG_WRITE(tod, ADRV906X_TOD_CFG_IO_CTRL, 0);
	/* Disable all IRQs */
	adrv906x_tod_hw_pps_irq_disable_all(tod);
}

/**
 * @brief Read and report HW version
 * @param tod Context struct
 * @return 0 Success
 * @return -ERANGE Unsupported HW version
 */
static int adrv906x_tod_get_version(struct adrv906x_tod *tod)
{
	u32 val;

	val = ADRV906X_REG_READ(tod, ADRV906X_TOD_VERSION);
	tod->ver_major = FIELD_GET(ADRV906X_TOD_VERSION_MAJOR, val);
	tod->ver_minor = FIELD_GET(ADRV906X_TOD_VERSION_MINOR, val);

	if (tod->sec_regs) {
		val = ADRV906X_REG_READ_SEC(tod, ADRV906X_TOD_VERSION);
		tod->sec_ver_major = FIELD_GET(ADRV906X_TOD_VERSION_MAJOR, val);
		tod->sec_ver_minor = FIELD_GET(ADRV906X_TOD_VERSION_MINOR, val);

		if (tod->ver_major != tod->sec_ver_major || tod->ver_minor != tod->sec_ver_minor) {
			dev_err(tod->dev, "tod versions don't match: %d.%d != %d.%d",
				tod->ver_major, tod->ver_minor,
				tod->sec_ver_major, tod->sec_ver_minor);
			return -ERANGE;
		}
	}

	return 0;
}

static unsigned int platform_get_num_of_resources(struct platform_device *dev,
						  unsigned int type)
{
	unsigned int num = 0;
	u32 i;

	for (i = 0; i < dev->num_resources; i++) {
		struct resource *r = &dev->resource[i];
		if (type == resource_type(r))
			num++;
	}

	return num;
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
	unsigned int num;
	u32 val;
	int ret;
	int i;

	adrv906x_tod = devm_kzalloc(dev, sizeof(*adrv906x_tod), GFP_KERNEL);
	if (!adrv906x_tod)
		return -ENOMEM;
	adrv906x_tod->dev = dev;

	if (!np) {
		dev_err(dev, "platform device data missing");
		return -EINVAL;
	}

	num = platform_get_num_of_resources(pdev, IORESOURCE_MEM);
	if (num != 1 && num != 2) {
		dev_err(dev, "invalid number of resources");
		return -EINVAL;
	}

	regs = devm_platform_ioremap_resource(pdev, 0U);
	if (IS_ERR(regs)) {
		ret = PTR_ERR(regs);
		return ret;
	}
	adrv906x_tod->regs = regs;
	adrv906x_tod->sec_regs = NULL;

	if (num == 2) {
		dev_info(dev, "operating in dual-tile mode");
		regs = devm_platform_ioremap_resource(pdev, 1U);
		if (IS_ERR(regs)) {
			ret = PTR_ERR(regs);
			return ret;
		}
		adrv906x_tod->sec_regs = regs;
	}

	ret = adrv906x_tod_get_version(adrv906x_tod);
	if (ret)
		goto err_out;
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
	adrv906x_tod->gc_clk_freq_khz = (u32)div_u64((u64)rate, 1000U);

	rate = clk_get_rate(adrv906x_tod->lc_clk);
	adrv906x_tod->lc_freq_khz = (u32)div_u64((u64)rate, 1000U);

	ret = of_property_read_u32(np, "adi,pps-in-pulse-width-ms", &val);
	if (ret) {
		dev_info(dev, "'adi,pps-in-pulse-width-ms' not set, using 40ms");
		val = 40 * USEC_PER_MSEC;
	} else if (val >= 1000U) {
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
			       adrv906x_tod_pps_isr, 0U, pdev->name,
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
			val = 0U;
		adrv906x_tod->cdc.delay_cnt[i] = val;
	}

	adrv906x_tod_cfg_cdc_delay_all(adrv906x_tod);
	counter = &adrv906x_tod->counter[adrv906x_tod->tod_counter_src];
	if (counter->en) {
		ret = adrv906x_tod_extts_enable(counter, 1U);
		if (ret) {
			dev_err(dev, "default tod counter enable failed");
			goto err_out_unreg;
		}
	}

	ret = of_property_read_u32(tod_np, "adi,default-tod-counter", &val);
	if (ret) {
		dev_warn(dev, "adi,default-tod-counter not set, using default %d", 0);
		adrv906x_tod->tod_counter_src = 0U;
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
		atomic_set(&adrv906x_tod->pps_state, 0);
		adrv906x_tod_hw_pps_irq_external_enable(adrv906x_tod);
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

/**
 * @brief Stop and remove the driver
 * @param pdev Context struct
 * @return See kernel log for error descriptions
 */
void adrv906x_tod_remove(struct platform_device *pdev)
{
	struct adrv906x_tod_counter *counter;
	int i;

	if (!adrv906x_tod)
		return;

	adrv906x_tod_hw_disable_all(adrv906x_tod);

	for (i = 0; i < ADRV906X_HW_TOD_COUNTER_CNT; i++) {
		counter = &adrv906x_tod->counter[i];
		if (counter->en) {
			adrv906x_tod_extts_enable(counter, ADRV906X_DISABLE);
			adrv906x_tod_pps_enable(counter, ADRV906X_DISABLE);
		}
		if (adrv906x_tod->counter[i].ptp_clk)
			ptp_clock_unregister(adrv906x_tod->counter[i].ptp_clk);
	}

	if (adrv906x_tod->ver_major == 2 && adrv906x_tod->ver_minor <= 2)
		cancel_delayed_work_sync(&adrv906x_tod->pps_work);

	mutex_destroy(&adrv906x_tod->reg_lock);
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
