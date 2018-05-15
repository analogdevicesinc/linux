/*
 * NXP i.MX8QXP ADC driver
 *
 * Copyright (C) 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/iio/sysfs.h>

/* ADC register */
#define IMX8QXP_REG_ADC_VERID		0x00
#define IMX8QXP_REG_ADC_PARAM		0x04
#define IMX8QXP_REG_ADC_CTRL		0x10
#define IMX8QXP_REG_ADC_STAT		0x14
#define IMX8QXP_REG_ADC_IE		0x18
#define IMX8QXP_REG_ADC_DE		0x1c
#define IMX8QXP_REG_ADC_CFG		0x20
#define IMX8QXP_REG_ADC_PAUSE		0x24
#define IMX8QXP_REG_ADC_FCTRL		0x30
#define IMX8QXP_REG_ADC_SWTRIG		0x34
#define IMX8QXP_REG_ADC_TCTRL0		0xc0
#define IMX8QXP_REG_ADC_TCTRL1		0xc4
#define IMX8QXP_REG_ADC_TCTRL2		0xc8
#define IMX8QXP_REG_ADC_TCTRL3		0xcc
#define IMX8QXP_REG_ADC_TCTRL4		0xd0
#define IMX8QXP_REG_ADC_TCTRL5		0xd4
#define IMX8QXP_REG_ADC_TCTRL6		0xd8
#define IMX8QXP_REG_ADC_TCTRL7		0xdc
#define IMX8QXP_REG_ADC_CMDL1		0x100
#define IMX8QXP_REG_ADC_CMDH1		0x104
#define IMX8QXP_REG_ADC_CMDL2		0x108
#define IMX8QXP_REG_ADC_CMDH2		0x10c
#define IMX8QXP_REG_ADC_CMDL3		0x110
#define IMX8QXP_REG_ADC_CMDH3		0x114
#define IMX8QXP_REG_ADC_CMDL4		0x118
#define IMX8QXP_REG_ADC_CMDH4		0x11c
#define IMX8QXP_REG_ADC_CMDL5		0x120
#define IMX8QXP_REG_ADC_CMDH5		0x124
#define IMX8QXP_REG_ADC_CMDL6		0x128
#define IMX8QXP_REG_ADC_CMDH6		0x12c
#define IMX8QXP_REG_ADC_CMDL7		0x130
#define IMX8QXP_REG_ADC_CMDH7		0x134
#define IMX8QXP_REG_ADC_CMDL8		0x138
#define IMX8QXP_REG_ADC_CMDH8		0x13c
#define IMX8QXP_REG_ADC_CV1		0x200
#define IMX8QXP_REG_ADC_CV2		0x204
#define IMX8QXP_REG_ADC_CV3		0x208
#define IMX8QXP_REG_ADC_CV4		0x20c
#define IMX8QXP_REG_ADC_RESFIFO		0x300
#define IMX8QXP_REG_ADC_TST		0xffc

/* ADC IE bit shift */
#define IMX8QXP_REG_ADC_IE_FOFIE_SHIFT		(1)
#define IMX8QXP_REG_ADC_IE_FWMIE_SHIFT		(0)
/* ADC CTRL bit shift*/
#define IMX8QXP_REG_ADC_CTRL_FIFO_RESET_SHIFT		(8)
#define IMX8QXP_REG_ADC_CTRL_SOFTWARE_RESET_SHIFT	(1)
#define IMX8QXP_REG_ADC_CTRL_ADC_ENABLE			(0)
/* ADC TCTRL bit shift*/
#define IMX8QXP_REG_ADC_TCTRL_TCMD_SHIFT	(24)
#define IMX8QXP_REG_ADC_TCTRL_TDLY_SHIFT	(16)
#define IMX8QXP_REG_ADC_TCTRL_TPRI_SHIFT	(8)
#define IMX8QXP_REG_ADC_TCTRL_HTEN_SHIFT	(0)
/* ADC CMDL bit shift*/
#define IMX8QXP_REG_ADC_CMDL_CSCALE_SHIFT	(8)
#define IMX8QXP_REG_ADC_CMDL_MODE_SHIFT		(7)
#define IMX8QXP_REG_ADC_CMDL_DIFF_SHIFT		(6)
#define IMX8QXP_REG_ADC_CMDL_ABSEL_SHIFT	(5)
#define IMX8QXP_REG_ADC_CMDL_ADCH_SHIFT		(0)
/* ADC CMDH bit shift*/
#define IMX8QXP_REG_ADC_CMDH_NEXT_SHIFT		(24)
#define IMX8QXP_REG_ADC_CMDH_LOOP_SHIFT		(16)
#define IMX8QXP_REG_ADC_CMDH_AVGS_SHIFT		(12)
#define IMX8QXP_REG_ADC_CMDH_STS_SHIFT		(8)
#define IMX8QXP_REG_ADC_CMDH_LWI_SHIFT		(7)
#define IMX8QXP_REG_ADC_CMDH_CMPEN_SHIFT	(0)
/* ADC CFG bit shift*/
#define IMX8QXP_REG_ADC_CFG_PWREN_SHIFT		(28)
#define IMX8QXP_REG_ADC_CFG_PUDLY_SHIFT		(16)
#define IMX8QXP_REG_ADC_CFG_REFSEL_SHIFT	(6)
#define IMX8QXP_REG_ADC_CFG_PWRSEL_SHIFT	(4)
#define IMX8QXP_REG_ADC_CFG_TPRICTRL_SHIFT	(0)
/* ADC FCTRL bit shift*/
#define IMX8QXP_ADC_FCTRL_FWMARK_SHIFT		(16)
#define IMX8QXP_ADC_FCTRL_FWMARK_MASK		(0x1f << 16)
#define IMX8QXP_ADC_FCTRL_FCOUNT_MASK		(0x1f)
/* ADC STAT bit shift*/
#define IMX8QXP_REG_ADC_STAT_CMDACT_SHIFT	(24)
#define IMX8QXP_REG_ADC_STAT_TRGACT_SHIFT	(16)
#define IMX8QXP_REG_ADC_STAT_FOF_SHIFT		(1)
#define IMX8QXP_REG_ADC_STAT_RDY_SHIFT		(0)

/* ADC CMD PARAMETER*/
#define IMX8QXP_REG_ADC_CMDL_CNANNEL_SCALE_DOWN		(0)
#define IMX8QXP_REG_ADC_CMDL_CHANNEL_SCALE_FULL		(0x3f)
#define IMX8QXP_REG_ADC_CMDL_SEL_A_A_B_CHANNEL		(0)
#define IMX8QXP_REG_ADC_CMDL_SEL_B_B_A_CHANNEL		(1)
#define IMX8QXP_REG_ADC_CMDL_STANDARD_RESOLUTION	(0)
#define IMX8QXP_REG_ADC_CMDL_HIGH_RESOLUTION		(1)
#define IMX8QXP_REG_ADC_CMDL_MODE_SINGLE		(0)
#define IMX8QXP_REG_ADC_CMDL_MODE_DIFF			(1)

#define IMX8QXP_REG_ADC_CMDH_LWI_INCREMENT_ENABLE	(1)
#define IMX8QXP_REG_ADC_CMDH_LWI_INCREMENT_DISABLE	(0)
#define IMX8QXP_REG_ADC_CMDH_CMPEN_DISABLE		(0)
#define IMX8QXP_REG_ADC_CMDH_CMPEN_ENABLE_TRUE		(2)
#define IMX8QXP_REG_ADC_CMDH_CMPEN_ENABLE_UNTILE_TRUE	(0x3)

/* ADC PAUSE PARAMETER*/
#define IMX8QXP_REG_ADC_PAUSE_ENABLE		(0x80000000)
/* ADC TRIGGER PARAMETER*/
#define IMX8QXP_REG_ADC_TCTRL_TPRI_PRIORITY_HIGH	(0)
#define IMX8QXP_REG_ADC_TCTRL_TPRI_PRIORITY_LOW	(1)

#define IMX8QXP_REG_ADC_TCTRL_HTEN_HARDWARE_TIRGGER_DISABLE	(0)
#define IMX8QXP_REG_ADC_TCTRC_HTEN_HARDWARE_TIRGGER_ENABLE	(1)

#define MAX_CMD (8)
#define MAX_TRIG (8)

#define IMX8QXP_ADC_TIMEOUT		msecs_to_jiffies(100)

struct imx8qxp_adc_trigger_ctrl {
	u32 tcmd;
	u32 tdly;
	u32 tpri;
	u32 hten;
};

struct imx8qxp_adc_cmd_l {
	u32 scale;
	u32 mode;
	u32 diff;
	u32 absel;
	u32 adch;
};

struct imx8qxp_adc_cmd_h {
	u32 next;
	u32 loop;
	u32 avgs;
	u32 sts;
	u32 lwi;
	u32 cmpen;
};

struct imx8qxp_adc_cfg {
	u32 pwren;
	u32 pudly;
	u32 refsel;
	u32 pwrsel;
	u32 tprictrl;
};

struct imx8qxp_adc {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	struct clk *ipg_clk;

	u32 vref_uv;
	u32 value;
	u32 channel_id;
	u32 trigger_id;
	u32 cmd_id;

	struct regulator *vref;
	struct imx8qxp_adc_cmd_l adc_cmd_l[MAX_CMD + 1];
	struct imx8qxp_adc_cmd_h adc_cmd_h[MAX_CMD + 1];
	struct imx8qxp_adc_trigger_ctrl adc_trigger_ctrl[MAX_TRIG + 1];
	struct imx8qxp_adc_cfg adc_cfg;
	struct completion completion;
};

#define IMX8QXP_ADC_CHAN(_idx) {					\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
}

static const struct iio_chan_spec imx8qxp_adc_iio_channels[] = {
	IMX8QXP_ADC_CHAN(0),
	IMX8QXP_ADC_CHAN(1),
	IMX8QXP_ADC_CHAN(2),
	IMX8QXP_ADC_CHAN(3),
};

static void imx8qxp_adc_feature_prepare(struct imx8qxp_adc *adc)
{
	u32 i;

	adc->trigger_id = 0;
	adc->cmd_id = 0;
	adc->channel_id = 0;

	for (i = 0; i < MAX_CMD + 1; i++) {
		adc->adc_cmd_l[i].scale = 1;
		adc->adc_cmd_l[i].mode = 0;
		adc->adc_cmd_l[i].diff = 0;
		adc->adc_cmd_l[i].absel = 0;
		adc->adc_cmd_l[i].adch = 0;
		adc->adc_cmd_h[i].next = 0;
		adc->adc_cmd_h[i].loop = 0;
		adc->adc_cmd_h[i].avgs = 0;
		adc->adc_cmd_h[i].sts = 0;
		adc->adc_cmd_h[i].lwi = 0;
		adc->adc_cmd_h[i].cmpen = 0;
	}

	for (i = 0; i < MAX_TRIG; i++) {
		adc->adc_trigger_ctrl[i].tcmd = 0;
		adc->adc_trigger_ctrl[i].tdly = 0;
		adc->adc_trigger_ctrl[i].tpri = 0;
		adc->adc_trigger_ctrl[i].hten = 0;
	}

	adc->adc_cfg.pwren = 0;
	adc->adc_cfg.pudly = 0;
	adc->adc_cfg.refsel = 0;
	adc->adc_cfg.pwrsel = 0;
	adc->adc_cfg.tprictrl = 0;
}

static void imx8qxp_adc_reset(struct imx8qxp_adc *adc)
{
	u32 ctrl;

	/*software reset, need to clear the set bit*/
	ctrl = readl(adc->regs + IMX8QXP_REG_ADC_CTRL);
	ctrl |= 1 << IMX8QXP_REG_ADC_CTRL_SOFTWARE_RESET_SHIFT;
	writel(ctrl, adc->regs + IMX8QXP_REG_ADC_CTRL);
	udelay(10);
	ctrl &= ~(1 << IMX8QXP_REG_ADC_CTRL_SOFTWARE_RESET_SHIFT);
	writel(ctrl, adc->regs + IMX8QXP_REG_ADC_CTRL);

	/* reset the fifo */
	ctrl |= 1 << IMX8QXP_REG_ADC_CTRL_FIFO_RESET_SHIFT;
	writel(ctrl, adc->regs + IMX8QXP_REG_ADC_CTRL);
}

static void imx8qxp_adc_reg_config(struct imx8qxp_adc *adc)
{
	u32 adc_cfg, adc_tctrl, adc_cmdl, adc_cmdh;
	u32 t_id, c_id;

	adc_cfg = readl(adc->regs + IMX8QXP_REG_ADC_CFG);
	adc_cfg |= adc->adc_cfg.pwren << IMX8QXP_REG_ADC_CFG_PWREN_SHIFT
			| adc->adc_cfg.pudly << IMX8QXP_REG_ADC_CFG_PUDLY_SHIFT
			| adc->adc_cfg.refsel << IMX8QXP_REG_ADC_CFG_REFSEL_SHIFT
			| adc->adc_cfg.pwrsel << IMX8QXP_REG_ADC_CFG_PWRSEL_SHIFT
			| adc->adc_cfg.tprictrl << IMX8QXP_REG_ADC_CFG_TPRICTRL_SHIFT;
	writel(adc_cfg, adc->regs + IMX8QXP_REG_ADC_CFG);

	t_id = adc->trigger_id;
	adc_tctrl = readl(adc->regs + IMX8QXP_REG_ADC_TCTRL0 + t_id * 4);
	adc_tctrl |= adc->adc_trigger_ctrl[t_id].tcmd << IMX8QXP_REG_ADC_TCTRL_TCMD_SHIFT
			| adc->adc_trigger_ctrl[t_id].tdly << IMX8QXP_REG_ADC_TCTRL_TDLY_SHIFT
			| adc->adc_trigger_ctrl[t_id].tpri << IMX8QXP_REG_ADC_TCTRL_TPRI_SHIFT
			| adc->adc_trigger_ctrl[t_id].hten << IMX8QXP_REG_ADC_TCTRL_HTEN_SHIFT;
	writel(adc_tctrl, adc->regs + IMX8QXP_REG_ADC_TCTRL0 + t_id * 4);

	c_id = adc->cmd_id - 1;
	adc_cmdl = readl(adc->regs + IMX8QXP_REG_ADC_CMDL1 + c_id * 8);
	adc_cmdl |= adc->adc_cmd_l[c_id].scale << IMX8QXP_REG_ADC_CMDL_CSCALE_SHIFT
			| adc->adc_cmd_l[c_id].mode << IMX8QXP_REG_ADC_CMDL_MODE_SHIFT
			| adc->adc_cmd_l[c_id].diff << IMX8QXP_REG_ADC_CMDL_DIFF_SHIFT
			| adc->adc_cmd_l[c_id].absel << IMX8QXP_REG_ADC_CMDL_ABSEL_SHIFT
			| adc->adc_cmd_l[c_id].adch << IMX8QXP_REG_ADC_CMDL_ADCH_SHIFT;
	writel(adc_cmdl, adc->regs + IMX8QXP_REG_ADC_CMDL1 + c_id * 8);

	adc_cmdh = readl(adc->regs + IMX8QXP_REG_ADC_CMDH1 + c_id * 8);
	adc_cmdh |= adc->adc_cmd_h[c_id].next << IMX8QXP_REG_ADC_CMDH_NEXT_SHIFT
			| adc->adc_cmd_h[c_id].loop << IMX8QXP_REG_ADC_CMDH_LOOP_SHIFT
			| adc->adc_cmd_h[c_id].avgs << IMX8QXP_REG_ADC_CMDH_AVGS_SHIFT
			| adc->adc_cmd_h[c_id].sts << IMX8QXP_REG_ADC_CMDH_STS_SHIFT
			| adc->adc_cmd_h[c_id].lwi << IMX8QXP_REG_ADC_CMDH_LWI_SHIFT
			| adc->adc_cmd_h[c_id].cmpen << IMX8QXP_REG_ADC_CMDH_CMPEN_SHIFT;
	writel(adc_cmdh, adc->regs + IMX8QXP_REG_ADC_CMDH1 + c_id * 8);
}

static void imx8qxp_adc_mode_config(struct imx8qxp_adc *adc)
{
	u32 cmd_id, trigger_id, channel_id;

	channel_id = adc->channel_id;
	cmd_id = adc->cmd_id - 1;
	trigger_id = adc->trigger_id;

	/* config the cmd */
	adc->adc_cmd_l[cmd_id].scale = IMX8QXP_REG_ADC_CMDL_CHANNEL_SCALE_FULL;
	adc->adc_cmd_l[cmd_id].mode = IMX8QXP_REG_ADC_CMDL_STANDARD_RESOLUTION;
	adc->adc_cmd_l[cmd_id].diff = IMX8QXP_REG_ADC_CMDL_MODE_SINGLE;
	adc->adc_cmd_l[cmd_id].absel = IMX8QXP_REG_ADC_CMDL_SEL_A_A_B_CHANNEL;
	adc->adc_cmd_l[cmd_id].adch = channel_id;

	adc->adc_cmd_h[cmd_id].next = 0;
	adc->adc_cmd_h[cmd_id].loop = 0;
	adc->adc_cmd_h[cmd_id].avgs = 7;  // 128 times conversion
	adc->adc_cmd_h[cmd_id].sts = 0;
	adc->adc_cmd_h[cmd_id].lwi = IMX8QXP_REG_ADC_CMDH_LWI_INCREMENT_DISABLE;
	adc->adc_cmd_h[cmd_id].cmpen = IMX8QXP_REG_ADC_CMDH_CMPEN_DISABLE;

	/* config the trigger control */
	adc->adc_trigger_ctrl[trigger_id].tcmd = adc->cmd_id;  //point to cmd1
	adc->adc_trigger_ctrl[trigger_id].tdly = 0;
	adc->adc_trigger_ctrl[trigger_id].tpri = IMX8QXP_REG_ADC_TCTRL_TPRI_PRIORITY_HIGH;
	adc->adc_trigger_ctrl[trigger_id].hten = IMX8QXP_REG_ADC_TCTRL_HTEN_HARDWARE_TIRGGER_DISABLE;

	/* ADC configuration */
	adc->adc_cfg.pwren = 1;
	adc->adc_cfg.pudly = 0x80;
	adc->adc_cfg.refsel = 0;
	adc->adc_cfg.pwrsel = 3;
	adc->adc_cfg.tprictrl = 0;

	imx8qxp_adc_reg_config(adc);
}

static void imx8qxp_adc_fifo_config(struct imx8qxp_adc *adc)
{
	u32 fifo_ctrl, interrupt_en;

	fifo_ctrl = readl(adc->regs + IMX8QXP_REG_ADC_FCTRL);
	fifo_ctrl &= ~IMX8QXP_ADC_FCTRL_FWMARK_MASK;
	/* set the watermark level to 1 */
	fifo_ctrl |= 0 << IMX8QXP_ADC_FCTRL_FWMARK_SHIFT;
	writel(fifo_ctrl, adc->regs + IMX8QXP_REG_ADC_FCTRL);

	/* FIFO Watermark Interrupt Enable */
	interrupt_en = readl(adc->regs + IMX8QXP_REG_ADC_IE);
	interrupt_en |= 1 << IMX8QXP_REG_ADC_IE_FWMIE_SHIFT;
	writel(interrupt_en, adc->regs + IMX8QXP_REG_ADC_IE);
}

static void imx8qxp_adc_disable(struct imx8qxp_adc *adc)
{
	u32 ctrl;

	ctrl = readl(adc->regs + IMX8QXP_REG_ADC_CTRL);
	ctrl &=  ~(1 << IMX8QXP_REG_ADC_CTRL_ADC_ENABLE);
	writel(ctrl, adc->regs + IMX8QXP_REG_ADC_CTRL);
}

static void imx8qxp_adc_enable(struct imx8qxp_adc *adc)
{
	u32 ctrl;

	ctrl = readl(adc->regs + IMX8QXP_REG_ADC_CTRL);
	ctrl |= 1 << IMX8QXP_REG_ADC_CTRL_ADC_ENABLE;
	writel(ctrl, adc->regs + IMX8QXP_REG_ADC_CTRL);

}

static void imx8qxp_adc_start_trigger(struct imx8qxp_adc *adc)
{
	writel(1 << adc->trigger_id, adc->regs + IMX8QXP_REG_ADC_SWTRIG);
}

static u32 imx8qxp_adc_get_sample_rate(struct imx8qxp_adc *adc)
{

	u32 input_clk;

	input_clk = clk_get_rate(adc->clk);

	return input_clk / 3;
}

static int imx8qxp_adc_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long mask)
{

	struct imx8qxp_adc *adc = iio_priv(indio_dev);

	u32 channel;
	long ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		reinit_completion(&adc->completion);

		channel = chan->channel & 0x03;
		adc->channel_id = channel;
		adc->cmd_id = 1;
		adc->trigger_id = 0;
		imx8qxp_adc_mode_config(adc);

		imx8qxp_adc_fifo_config(adc);

		imx8qxp_adc_enable(adc);

		imx8qxp_adc_start_trigger(adc);

		ret = wait_for_completion_interruptible_timeout
				(&adc->completion, IMX8QXP_ADC_TIMEOUT);
		if (ret == 0) {
			mutex_unlock(&indio_dev->mlock);
			return -ETIMEDOUT;
		}
		if (ret < 0) {
			mutex_unlock(&indio_dev->mlock);
			return ret;
		}

		*val = adc->value;
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		adc->vref_uv = regulator_get_voltage(adc->vref);
		*val = adc->vref_uv / 1000;
		*val2 = 12;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = imx8qxp_adc_get_sample_rate(adc);
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int imx8qxp_adc_read_data(struct imx8qxp_adc *adc)
{
	u32 value;

	value = readl(adc->regs + IMX8QXP_REG_ADC_RESFIFO);
	value &= 0xffff;
	return (value >> 3);
}

static irqreturn_t imx8qxp_adc_isr(int irq, void *dev_id)
{
	struct imx8qxp_adc *adc = (struct imx8qxp_adc *)dev_id;

	u32 fifo_count;

	fifo_count = readl(adc->regs + IMX8QXP_REG_ADC_FCTRL)
			& IMX8QXP_ADC_FCTRL_FCOUNT_MASK;

	if (fifo_count) {
		adc->value = imx8qxp_adc_read_data(adc);
		complete(&adc->completion);
	}

	return IRQ_HANDLED;
}

static int imx8qxp_adc_reg_access(struct iio_dev *indio_dev,
			unsigned int reg, unsigned int writeval,
			unsigned int *readval)
{
	struct imx8qxp_adc *adc = iio_priv(indio_dev);

	if (!readval || reg % 4 || reg > IMX8QXP_REG_ADC_TST)
		return -EINVAL;

	*readval = readl(adc->regs + reg);

	return 0;
}

static const struct iio_info imx8qxp_adc_iio_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &imx8qxp_adc_read_raw,
	.debugfs_reg_access = &imx8qxp_adc_reg_access,
};

static const struct of_device_id imx8qxp_adc_match[] = {
	{ .compatible = "fsl,imx8qxp-adc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx8qxp_adc_match);

static int imx8qxp_adc_probe(struct platform_device *pdev)
{
	struct imx8qxp_adc *adc;
	struct iio_dev *indio_dev;
	struct resource *mem;
	int irq;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*adc));
	if (!indio_dev) {
		dev_err(&pdev->dev, "Failed allocating iio device\n");
		return -ENOMEM;
	}

	adc = iio_priv(indio_dev);
	adc->dev = &pdev->dev;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	adc->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(adc->regs)) {
		ret = PTR_ERR(adc->regs);
		dev_err(&pdev->dev,
			"Failed to remap adc memory, err = %d\n", ret);
		return ret;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "No irq resource?\n");
		return irq;
	}

	adc->clk = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(adc->clk)) {
		ret = PTR_ERR(adc->clk);
		dev_err(&pdev->dev, "Failed getting clock, err = %d\n", ret);
		return ret;
	}

	adc->ipg_clk = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(adc->ipg_clk)) {
		ret = PTR_ERR(adc->ipg_clk);
		dev_err(&pdev->dev, "Failed getting clock, err = %d\n", ret);
		return ret;
	}

	adc->vref = devm_regulator_get(&pdev->dev, "vref");
	if (IS_ERR(adc->vref)) {
		ret = PTR_ERR(adc->vref);
		dev_err(&pdev->dev,
			"Failed getting reference voltage, err = %d\n", ret);
		return ret;
	}

	ret = regulator_enable(adc->vref);
	if (ret) {
		dev_err(&pdev->dev,
			"Can't enable adc reference top voltage, err = %d\n",
			ret);
		return ret;
	}

	platform_set_drvdata(pdev, indio_dev);

	init_completion(&adc->completion);

	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &imx8qxp_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = imx8qxp_adc_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(imx8qxp_adc_iio_channels);

	ret = clk_prepare_enable(adc->clk);
	if (ret) {
		dev_err(&pdev->dev,
			"Could not prepare or enable the clock.\n");
		goto error_adc_clk_enable;
	}

	ret = clk_prepare_enable(adc->ipg_clk);
	if (ret) {
		dev_err(&pdev->dev,
			"Could not prepare or enable the clock.\n");
		goto error_adc_ipg_clk_enable;
	}

	ret = devm_request_irq(adc->dev, irq,
				imx8qxp_adc_isr, 0,
				dev_name(&pdev->dev), adc);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed requesting irq, irq = %d\n", irq);
		goto error_iio_device_register;
	}

	imx8qxp_adc_feature_prepare(adc);
	imx8qxp_adc_reset(adc);

	ret = iio_device_register(indio_dev);
	if (ret) {
		imx8qxp_adc_disable(adc);
		dev_err(&pdev->dev, "Couldn't register the device.\n");
		goto error_iio_device_register;
	}

	return 0;

error_iio_device_register:
	clk_disable_unprepare(adc->ipg_clk);
error_adc_ipg_clk_enable:
	clk_disable_unprepare(adc->clk);
error_adc_clk_enable:
	regulator_disable(adc->vref);

	return ret;
}

static int imx8qxp_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct imx8qxp_adc *adc = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	imx8qxp_adc_disable(adc);

	clk_disable_unprepare(adc->clk);
	clk_disable_unprepare(adc->ipg_clk);
	regulator_disable(adc->vref);

	return 0;
}

static int __maybe_unused imx8qxp_adc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct imx8qxp_adc *adc = iio_priv(indio_dev);

	imx8qxp_adc_disable(adc);

	clk_disable_unprepare(adc->clk);
	clk_disable_unprepare(adc->ipg_clk);
	regulator_disable(adc->vref);

	return 0;
}

static int __maybe_unused imx8qxp_adc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct imx8qxp_adc *adc = iio_priv(indio_dev);
	int ret;

	ret = regulator_enable(adc->vref);
	if (ret) {
		dev_err(adc->dev,
			"Can't enable adc reference top voltage, err = %d\n",
			ret);
		return ret;
	}

	ret = clk_prepare_enable(adc->clk);
	if (ret) {
		dev_err(adc->dev,
			"Could not prepare or enable clock.\n");
		regulator_disable(adc->vref);
		return ret;
	}

	ret = clk_prepare_enable(adc->ipg_clk);
	if (ret) {
		dev_err(adc->dev,
			"Could not prepare or enable clock.\n");
		clk_disable_unprepare(adc->clk);
		regulator_disable(adc->vref);
		return ret;
	}
	imx8qxp_adc_reset(adc);

	return 0;
}

static SIMPLE_DEV_PM_OPS(imx8qxp_adc_pm_ops, imx8qxp_adc_suspend, imx8qxp_adc_resume);

static struct platform_driver imx8qxp_adc_driver = {
	.probe		= imx8qxp_adc_probe,
	.remove		= imx8qxp_adc_remove,
	.driver		= {
		.name	= "imx8qxp_adc",
		.of_match_table = imx8qxp_adc_match,
		.pm	= &imx8qxp_adc_pm_ops,
	},
};

module_platform_driver(imx8qxp_adc_driver);

MODULE_AUTHOR("Haibo Chen <haibo.chen@nxp.com>");
MODULE_DESCRIPTION("NXP IMX8QXP ADC driver");
MODULE_LICENSE("GPL v2");
