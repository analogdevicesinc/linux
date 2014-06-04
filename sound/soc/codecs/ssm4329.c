/*
 * SSM4329 driver
 *
 * Copyright 2014 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
//#include <linux/platform_data/ssm4329.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/gcd.h>
#include <linux/workqueue.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "ssm4329.h"
#include "tdmc.h"
#include "sigmadsp.h"

/* ADI Vendor ID */
#define SSM4329_REG_VENDOR_ID		0x4000
/* SSM4329 Device ID */
#define SSM4329_REG_DEVICE_ID1		0x4001
/* SSM4329 Device ID */
#define SSM4329_REG_DEVICE_ID2		0x4002
/* Revision Code */
#define SSM4329_REG_REVISION		0x4003
/* Analog blocks power control */
#define SSM4329_REG_ANA_PWR		0x4004
/* Digital blocks power control */
#define SSM4329_REG_DIG_PWR1		0x4005
/* Digital blocks power control */
#define SSM4329_REG_DIG_PWR2		0x4006
/* Master power control */
#define SSM4329_REG_CHIP_PWR		0x4007
/* Clock Control */
#define SSM4329_REG_CLK_CTRL1		0x4008
/* PLL Input Divider */
#define SSM4329_REG_CLK_CTRL2		0x4009
/* PLL Feedback Integer Divider (LSBs) */
#define SSM4329_REG_CLK_CTRL3		0x400A
/* PLL Feedback Integer Divider (MSBs) */
#define SSM4329_REG_CLK_CTRL4		0x400B
/* PLL Fractional numerator value (LSBs) */
#define SSM4329_REG_CLK_CTRL5		0x400C
/* PLL Fractional numerator value (MSBs) */
#define SSM4329_REG_CLK_CTRL6		0x400D
/* PLL Fractional denominator (LSBs) */
#define SSM4329_REG_CLK_CTRL7		0x400E
/* PLL Fractional denominator (MSBs) */
#define SSM4329_REG_CLK_CTRL8		0x400F
/* PLL Update */
#define SSM4329_REG_CLK_CTRL9		0x4010
/* Serial Port 1 Output Routing */
#define SSM4329_REG_ROUTE_SP1_1		0x4011
/* Serial Port 1 Output Routing */
#define SSM4329_REG_ROUTE_SP1_2		0x4012
/* Serial Port 2 Output Routing */
#define SSM4329_REG_ROUTE_SP2_1		0x4013
/* Serial Port 2 Output Routing */
#define SSM4329_REG_ROUTE_SP2_2		0x4014
/* DAC and Interpolation path Output Routing */
#define SSM4329_REG_ROUTE_DAC_INT	0x4015
/* Input SRC Routing */
#define SSM4329_REG_ROUTE_SRC_IN	0x4016
/* Output SRC Routing */
#define SSM4329_REG_ROUTE_SRC_OUT	0x4017
/* Serial Port 1 base */
#define SSM4329_REG_SPT1_BASE		0x401a
/* serial Port 2 base */
#define SSM4329_REG_SPT2_BASE		0x4023
/* Serial Port X Control */
#define SSM4329_REG_SPT_CTRL1		0x0
/* Serial Port X Contro */
#define SSM4329_REG_SPT_CTRL2		0x1
/* Serial Port X Clock ontrol */
#define SSM4329_REG_SPT_CLOCKING	0x2
/* Serial Port X Input lot to Channel Mapping */
#define SSM4329_REG_SPT_INPUTS1		0x3
/* Serial Port X Input lot to Channel Mapping */
#define SSM4329_REG_SPT_INPUTS2		0x4
/* Serial Port X OutputChannel to Slot Mapping */
#define SSM4329_REG_SPT_OUTPUTS1	0x5
/* Serial Port X OutputChannel to Slot Mapping */
#define SSM4329_REG_SPT_OUTPUTS2	0x6
/* Serial Port X OutputChannel to Slot Mapping */
#define SSM4329_REG_SPT_OUTPUTS3	0x7
/* Serial Port X OutputChannel to Slot Mapping */
#define SSM4329_REG_SPT_OUTPUTS4	0x8
/* Class-D Amp and Output Sense Control */
#define SSM4329_REG_AMP_SNS_CTRL	0x402C
/* DAC Control */
#define SSM4329_REG_DAC_CTRL		0x402D
/* DAC Volume Control */
#define SSM4329_REG_DAC_VOLUME		0x402E
/* DAC High Rate Clip Point */
#define SSM4329_REG_DAC_CLIP		0x402F
/* DAC and Interp path Control */
#define SSM4329_REG_DAC_INTERP_CTRL	0x4030
/* Interp path Volume Control */
#define SSM4329_REG_INTERP_VOLUME	0x4031
/* Interp High Rate Clip Point */
#define SSM4329_REG_INTERP_CLIP		0x4032
/* ADC Control */
#define SSM4329_REG_ADC_CTRL		0x4033
/* ADC Volume Control */
#define SSM4329_REG_ADC_VOLUME		0x4034
/* SRC rate control */
#define SSM4329_REG_SRC_CTRL		0x4035
/* SigmaDSP Program Initialization */
#define SSM4329_REG_SDSP_CTRL1		0x4036
/* SimgaDSP Go Control */
#define SSM4329_REG_SDSP_CTRL2		0x4037
/* SigmaDSP Run Control */
#define SSM4329_REG_SDSP_CTRL3		0x4038
/* SigmaDSP Watchdog Control */
#define SSM4329_REG_SDSP_CTRL4		0x4039
/* SigmaDSP Watchdog Value */
#define SSM4329_REG_SDSP_CTRL5		0x403A
/* SigmaDSP Watchdog Value */
#define SSM4329_REG_SDSP_CTRL6		0x403B
/* SigmaDSP Watchdog Value */
#define SSM4329_REG_SDSP_CTRL7		0x403C
/* SigmaDSP Modulo Data Memory Start Location */
#define SSM4329_REG_SDSP_CTRL8		0x403D
/* SigmaDSP Modulo Data Memory Start Location */
#define SSM4329_REG_SDSP_CTRL9		0x403E
/* SigmaDSP Go divide rate. */
#define SSM4329_REG_SDSP_CTRL10		0x403F
/* SigmaDSP Go divide rate. */
#define SSM4329_REG_SDSP_CTRL11		0x4040
/* GPIO, IRQ, MCLKO pin control */
#define SSM4329_REG_PIN_FUNC		0x4041
/* GPIO output setting */
#define SSM4329_REG_GPIO_OUT_SETTING	0x4042
/* Mask causes of IRQ1 */
#define SSM4329_REG_IRQ1_MASK1		0x4043
/* Mask causes of IRQ1 */
#define SSM4329_REG_IRQ1_MASK2		0x4044
/* Mask causes of IRQ2 */
#define SSM4329_REG_IRQ2_MASK1		0x4045
/* Mask causes of IRQ2 */
#define SSM4329_REG_IRQ2_MASK2		0x4046
/* Clear Interrupts */
#define SSM4329_REG_IRQ_CLEAR		0x4047
/* MCLK Output Control */
#define SSM4329_REG_MCLKO_CTRL		0x4048
/* External Amplifier Control Enable */
#define SSM4329_REG_EAC_EN		0x404A
/* External Amplifier directly mapped registers */
#define SSM4329_REG_EAC_DIRECT(x)		(0x404B + (x))
/* External Amp Control Address */
#define SSM4329_REG_EAC_ADDR		0x404E
/* External Amp Control Write Data */
#define SSM4329_REG_EAC_WR_DATA		0x404F
/* External Amp Read/Write Control */
#define SSM4329_REG_EAC_RW_CTRL		0x4050
/* Pad Control 1 */
#define SSM4329_REG_PAD_CTRL1		0x4051
/* Pad Control 2 */
#define SSM4329_REG_PAD_CTRL2		0x4052
/* Fault Auto Recovery */
#define SSM4329_REG_FAULT_RECOV		0x4053
/* VBAT Warning Level */
#define SSM4329_REG_VBAT_WARN_LEVEL	0x4054
/* Boost DC/DC converter control registers */
#define SSM4329_REG_BST_CTRL		0x4055
/* Software reset, not including control registers */
#define SSM4329_REG_SOFT_RESET		0x4056
/* Software reset of entire IC. */
#define SSM4329_REG_SOFT_FULL_RESET	0x4057
/* Read 8-bit VBAT sense value */
#define SSM4329_REG_VBAT		0x4058
/* Fault Status */
#define SSM4329_REG_FAULT_STATUS	0x4059
/* SigmaDSP and PLL Lock Status */
#define SSM4329_REG_PLL_SDSP_STATUS	0x405A
/* External Amplifier Control Status */
#define SSM4329_REG_EAC_STATUS		0x405B
/* External Amplifier Control Read Data */
#define SSM4329_REG_EAC_RD_DATA		0x405C
/* IRQ1 Status */
#define SSM4329_REG_IRQ1_STATUS1	0x405D
/* IRQ1 Status */
#define SSM4329_REG_IRQ1_STATUS2	0x405E
/* IRQ2 Status */
#define SSM4329_REG_IRQ2_STATUS1	0x405F
/* IRQ2 Status */
#define SSM4329_REG_IRQ2_STATUS2	0x4060
/* GPIO input reading */
#define SSM4329_REG_GPIO_IN_READING	0x4061

#define SSM4329_REG_CLK_CTRL9_PLL_UPDATE BIT(0)

#define SSM4329_REG_CHIP_PWR_CHIP_PWDN BIT(0)

#define SSM4329_SPT_CTRL1_DELAY_MASK		(0x7 << 4)
#define SSM4329_SPT_CTRL1_DELAY1		(0x0 << 4)
#define SSM4329_SPT_CTRL1_DELAY0		(0x1 << 4)
#define SSM4329_SPT_CTRL1_DELAY8		(0x2 << 4)
#define SSM4329_SPT_CTRL1_DELAY12		(0x3 << 4)
#define SSM4329_SPT_CTRL1_DELAY16		(0x4 << 4)
#define SSM4329_SPT_CTRL1_SLOT_WIDTH_MASK	(0x3 << 2)
#define SSM4329_SPT_CTRL1_SLOT_WIDTH_32		(0x0 << 2)
#define SSM4329_SPT_CTRL1_SLOT_WIDTH_16		(0x1 << 2)
#define SSM4329_SPT_CTRL1_SLOT_WIDTH_24		(0x2 << 2)
#define SSM4329_SPT_CTRL1_MODE_MASK		(0x3 << 0)
#define SSM4329_SPT_CTRL1_MODE_STEREO		(0x0 << 0)
#define SSM4329_SPT_CTRL1_MODE_TDM		(0x1 << 0)
#define SSM4329_SPT_CTRL1_MODE_MONO		(0x2 << 0)

#define SSM4329_SPT_CTRL2_TRI_STATE		BIT(4)

#define SSM4329_SPT_CLOCKING_LRCLK_POL		BIT(7)
#define SSM4329_SPT_CLOCKING_LRCLK_SRC_MASK	(0x7 << 4)
#define SSM4329_SPT_CLOCKING_LRCLK_SRC_EXT	(0x0 << 4)
#define SSM4329_SPT_CLOCKING_LRCLK_SRC_48	(0x1 << 4)
#define SSM4329_SPT_CLOCKING_LRCLK_SRC_96	(0x2 << 4)
#define SSM4329_SPT_CLOCKING_LRCLK_SRC_192	(0x3 << 4)
#define SSM4329_SPT_CLOCKING_LRCLK_SRC_12	(0x4 << 4)
#define SSM4329_SPT_CLOCKING_LRCLK_SRC_24	(0x5 << 4)
#define SSM4329_SPT_CLOCKING_LRCLK_SRC_8	(0x6 << 4)
#define SSM4329_SPT_CLOCKING_LRCLK_SRC_16	(0x7 << 4)
#define SSM4329_SPT_CLOCKING_BCLK_POL		BIT(3)
#define SSM4329_SPT_CLOCKING_BCLK_SRC_MASK	(0x7 << 0)
#define SSM4329_SPT_CLOCKING_BCLK_SRC_EXT	(0x0 << 0)
#define SSM4329_SPT_CLOCKING_BCLK_SRC_64	(0x1 << 0)
#define SSM4329_SPT_CLOCKING_BCLK_SRC_128	(0x2 << 0)
#define SSM4329_SPT_CLOCKING_BCLK_SRC_256	(0x3 << 0)
#define SSM4329_SPT_CLOCKING_BCLK_SRC_512	(0x4 << 0)

struct ssm4329_dai_cfg {
	unsigned int fmt;
	unsigned int slot_width;
	unsigned int slots;

	unsigned int out_active_channels;
};

struct ssm4329 {
	struct regmap *regmap;
	unsigned int sysclk;

	struct gpio_desc *pwdn_gpio;
	struct regulator_bulk_data supplies[2];

	struct ssm4329_dai_cfg dais[2];

	struct snd_soc_dapm_context *dapm;

	struct sigmadsp sigmadsp;

#ifdef CONFIG_SND_SOC_SSM4329_TDMC_MASTER
	struct snd_soc_dai *tdmc_dai;
	bool tdmc_active;
	unsigned int sp2_out_data_active:1;
	unsigned int sp2_out_ctrl_active:1;
	struct delayed_work sp2_disable_work;
	struct mutex lock;
#endif
};

static int ssm4329_update_dai_cfg(struct ssm4329 *ssm4329, unsigned int id,
	struct ssm4329_dai_cfg *dai_cfg);

#ifdef CONFIG_SND_SOC_SSM4329_TDMC_MASTER

static void ssm4329_lock(struct ssm4329 *ssm4329) __acquires(ssm4329->lock)
{
	mutex_lock(&ssm4329->lock);
}

static void ssm4329_unlock(struct ssm4329 *ssm4329) __releases(ssm4329->lock)
{
	mutex_unlock(&ssm4329->lock);
}

static void ssm4329_sp2_disable_work(struct work_struct *work)
{
    struct ssm4329 *ssm4329 = container_of(work, struct ssm4329,
	    sp2_disable_work.work);

    regmap_update_bits(ssm4329->regmap, SSM4329_REG_DIG_PWR1, BIT(4), BIT(4));
}

static void ssm4329_check_sp2_power(struct ssm4329 *ssm4329)
{
	mutex_lock(&ssm4329->lock);

	if (ssm4329->sp2_out_data_active || ssm4329->sp2_out_ctrl_active) {
		cancel_delayed_work_sync(&ssm4329->sp2_disable_work);
		regmap_update_bits(ssm4329->regmap, SSM4329_REG_DIG_PWR1,
			BIT(4), 0);
	} else {
		/* Writes will happen in bursts so keep it active for a while */
		if (ssm4329->tdmc_active)
			schedule_delayed_work(&ssm4329->sp2_disable_work, HZ);
		else
			regmap_update_bits(ssm4329->regmap,
				SSM4329_REG_DIG_PWR1, BIT(4), BIT(4));
	}

	mutex_unlock(&ssm4329->lock);
}

static int ssm4329_sp2_out_event(struct snd_soc_dapm_widget *w,
    struct snd_kcontrol *ctrl, int event)
{
	struct ssm4329 *ssm4329 = snd_soc_codec_get_drvdata(w->codec);

	ssm4329->sp2_out_data_active = SND_SOC_DAPM_EVENT_ON(event);
	ssm4329_check_sp2_power(ssm4329);

	return 0;
}

#define SSM4329_EAC_CTRL_WRITE	(0 << 4)
#define SSM4329_EAC_CTRL_READ	(1 << 4)
#define SSM4329_EAC_CTRL_UPDATE	BIT(0)

static int ssm4329_tdmc_write(struct tdmc_master *master,
	struct tdmc_slave *slave, unsigned int reg, unsigned int val)
{
	struct ssm4329 *ssm4329 = tdmc_master_get_drvdata(master);

	ssm4329->sp2_out_ctrl_active = true;
	ssm4329_check_sp2_power(ssm4329);

	switch (reg) {
	case 0x00:
	case 0x01:
	case 0x02:
		regmap_write(ssm4329->regmap, SSM4329_REG_EAC_DIRECT(reg), val);
		break;
	default:
		regmap_write(ssm4329->regmap, SSM4329_REG_EAC_ADDR, reg);
		regmap_write(ssm4329->regmap, SSM4329_REG_EAC_WR_DATA, val);
		regmap_write(ssm4329->regmap, SSM4329_REG_EAC_RW_CTRL,
			SSM4329_EAC_CTRL_WRITE | SSM4329_EAC_CTRL_UPDATE);
		break;
	}

	ssm4329->sp2_out_ctrl_active = false;
	ssm4329_check_sp2_power(ssm4329);

	return 0;
}

static int ssm4329_tdmc_attach_slave(struct tdmc_master *master,
	struct tdmc_slave *slave)
{
	struct ssm4329 *ssm4329 = tdmc_master_get_drvdata(master);
	struct ssm4329_dai_cfg dai_cfg;

	if (slave->port != 1)
		return -EINVAL;

	ssm4329->tdmc_active = true;
	regmap_write(ssm4329->regmap, SSM4329_REG_EAC_EN, 1);

	/*
	 * Automatically switch to the only valid config, otherwise register
	 * access will not work.
	 */
	dai_cfg.slot_width = 64;
	dai_cfg.slots = 1;
	dai_cfg.fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM;
	dai_cfg.out_active_channels = 0x2;

	return ssm4329_update_dai_cfg(ssm4329, 1, &dai_cfg);
}

static void ssm4329_tdmc_detach_slave(struct tdmc_master *master,
	struct tdmc_slave *slave)
{
	struct ssm4329 *ssm4329 = tdmc_master_get_drvdata(master);

	regmap_write(ssm4329->regmap, SSM4329_REG_EAC_EN, 0);
	ssm4329->tdmc_active = false;
}

static const struct tdmc_ops ssm4329_tdmc_ops = {
	.write = ssm4329_tdmc_write,
	.attach_slave = ssm4329_tdmc_attach_slave,
	.detach_slave = ssm4329_tdmc_detach_slave,
};

int ssm4329_tdmc_master_register(struct device *dev, struct ssm4329 *ssm4329)
{
	mutex_init(&ssm4329->lock);
	INIT_DELAYED_WORK(&ssm4329->sp2_disable_work, ssm4329_sp2_disable_work);
	return devm_tdmc_master_register(dev, &ssm4329_tdmc_ops, ssm4329);
}

bool ssm4329_tdmc_active(struct ssm4329 *ssm4329)
{
	return ssm4329->tdmc_active;
}

#define SSM4329_SP2_OUT_WIDGET() SND_SOC_DAPM_AIF_OUT_E("SP2 OUT", NULL, 0, \
	SND_SOC_NOPM, 0, 0, ssm4329_sp2_out_event, \
	SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD)

#else

static void ssm4329_lock(struct ssm4329 *ssm4329) {}
static void ssm4329_unlock(struct ssm4329 *ssm4329) {}

int ssm4329_tdmc_master_register(struct device *dev, struct ssm4329 *ssm4329)
{
	return 0;
}

bool ssm4329_tdmc_active(struct ssm4329 *ssm4329)
{
	return false;
}

#define SSM4329_SP2_OUT_WIDGET() SND_SOC_DAPM_AIF_OUT("SP2 OUT", NULL, 0, \
	SSM4329_REG_DIG_PWR1, 4, 1)

#endif

static const struct reg_default ssm4329_reg_defaults[] = {
	{ SSM4329_REG_ANA_PWR, 0x04 },
	{ SSM4329_REG_DIG_PWR1, 0xf8 },
	{ SSM4329_REG_DIG_PWR2, 0x0f },
	{ SSM4329_REG_CHIP_PWR, 0x01 },
	{ SSM4329_REG_CLK_CTRL1, 0x01 },
	{ SSM4329_REG_CLK_CTRL2, 0x00 },
	{ SSM4329_REG_CLK_CTRL3, 0x00 },
	{ SSM4329_REG_CLK_CTRL4, 0x08 },
	{ SSM4329_REG_CLK_CTRL5, 0x00 },
	{ SSM4329_REG_CLK_CTRL6, 0x00 },
	{ SSM4329_REG_CLK_CTRL7, 0x00 },
	{ SSM4329_REG_CLK_CTRL8, 0x00 },
	{ SSM4329_REG_CLK_CTRL9, 0x00 },
	{ SSM4329_REG_ROUTE_SP1_1, 0x11 },
	{ SSM4329_REG_ROUTE_SP1_2, 0x00 },
	{ SSM4329_REG_ROUTE_SP2_1, 0x02 },
	{ SSM4329_REG_ROUTE_SP2_2, 0x00 },
	{ SSM4329_REG_ROUTE_DAC_INT, 0x00 },
	{ SSM4329_REG_ROUTE_SRC_IN, 0x00 },
	{ SSM4329_REG_ROUTE_SRC_OUT, 0x00 },
	{ SSM4329_REG_SPT1_BASE + SSM4329_REG_SPT_CTRL1, 0x00 },
	{ SSM4329_REG_SPT1_BASE + SSM4329_REG_SPT_CTRL2, 0x00 },
	{ SSM4329_REG_SPT1_BASE + SSM4329_REG_SPT_CLOCKING, 0x00 },
	{ SSM4329_REG_SPT1_BASE + SSM4329_REG_SPT_INPUTS1, 0x10 },
	{ SSM4329_REG_SPT1_BASE + SSM4329_REG_SPT_INPUTS2, 0x32 },
	{ SSM4329_REG_SPT1_BASE + SSM4329_REG_SPT_OUTPUTS1, 0x00 },
	{ SSM4329_REG_SPT1_BASE + SSM4329_REG_SPT_OUTPUTS2, 0x01 },
	{ SSM4329_REG_SPT1_BASE + SSM4329_REG_SPT_OUTPUTS3, 0x02 },
	{ SSM4329_REG_SPT1_BASE + SSM4329_REG_SPT_OUTPUTS4, 0x03 },
	{ SSM4329_REG_SPT2_BASE + SSM4329_REG_SPT_CTRL1, 0x00 },
	{ SSM4329_REG_SPT2_BASE + SSM4329_REG_SPT_CTRL2, 0x00 },
	{ SSM4329_REG_SPT2_BASE + SSM4329_REG_SPT_CLOCKING, 0x00 },
	{ SSM4329_REG_SPT2_BASE + SSM4329_REG_SPT_INPUTS1, 0x10 },
	{ SSM4329_REG_SPT2_BASE + SSM4329_REG_SPT_INPUTS2, 0x32 },
	{ SSM4329_REG_SPT2_BASE + SSM4329_REG_SPT_OUTPUTS1, 0x00 },
	{ SSM4329_REG_SPT2_BASE + SSM4329_REG_SPT_OUTPUTS2, 0x01 },
	{ SSM4329_REG_SPT2_BASE + SSM4329_REG_SPT_OUTPUTS3, 0x02 },
	{ SSM4329_REG_SPT2_BASE + SSM4329_REG_SPT_OUTPUTS4, 0x03 },
	{ SSM4329_REG_AMP_SNS_CTRL, 0x41 },
	{ SSM4329_REG_DAC_CTRL, 0x62 },
	{ SSM4329_REG_DAC_VOLUME, 0x40 },
	{ SSM4329_REG_DAC_CLIP, 0xff },
	{ SSM4329_REG_DAC_INTERP_CTRL, 0x00 },
	{ SSM4329_REG_INTERP_VOLUME, 0x40 },
	{ SSM4329_REG_INTERP_CLIP, 0xff },
	{ SSM4329_REG_ADC_CTRL, 0x04 },
	{ SSM4329_REG_ADC_VOLUME, 0x40 },
	{ SSM4329_REG_SRC_CTRL, 0x00 },
	{ SSM4329_REG_SDSP_CTRL1, 0x00 },
	{ SSM4329_REG_SDSP_CTRL2, 0x00 },
	{ SSM4329_REG_SDSP_CTRL3, 0x00 },
	{ SSM4329_REG_SDSP_CTRL4, 0x00 },
	{ SSM4329_REG_SDSP_CTRL5, 0x00 },
	{ SSM4329_REG_SDSP_CTRL6, 0x00 },
	{ SSM4329_REG_SDSP_CTRL7, 0x00 },
	{ SSM4329_REG_SDSP_CTRL8, 0x07 },
	{ SSM4329_REG_SDSP_CTRL9, 0xf4 },
	{ SSM4329_REG_SDSP_CTRL10, 0x08 },
	{ SSM4329_REG_SDSP_CTRL11, 0x00 },
	{ SSM4329_REG_PIN_FUNC, 0x00 },
	{ SSM4329_REG_GPIO_OUT_SETTING, 0x00 },
	{ SSM4329_REG_IRQ1_MASK1, 0x7f },
	{ SSM4329_REG_IRQ1_MASK2, 0x08 },
	{ SSM4329_REG_IRQ2_MASK1, 0x7f },
	{ SSM4329_REG_IRQ2_MASK2, 0x08 },
	{ SSM4329_REG_IRQ_CLEAR, 0x00 },
	{ SSM4329_REG_MCLKO_CTRL, 0x00 },
	{ SSM4329_REG_EAC_EN, 0x00 },
	{ SSM4329_REG_EAC_DIRECT(0), 0x81 },
	{ SSM4329_REG_EAC_DIRECT(1), 0x09 },
	{ SSM4329_REG_EAC_DIRECT(2), 0x32 },
	{ SSM4329_REG_EAC_ADDR, 0x00 },
	{ SSM4329_REG_EAC_WR_DATA, 0x00 },
	{ SSM4329_REG_EAC_RW_CTRL, 0x00 },
	{ SSM4329_REG_PAD_CTRL1, 0x00 },
	{ SSM4329_REG_PAD_CTRL2, 0x00 },
	{ SSM4329_REG_FAULT_RECOV, 0x00 },
	{ SSM4329_REG_VBAT_WARN_LEVEL, 0x00 },
	{ SSM4329_REG_BST_CTRL, 0x50 },
};

static const DECLARE_TLV_DB_MINMAX_MUTE(ssm4329_volume_tlv, -7125, 2400);
static const DECLARE_TLV_DB_LINEAR(ssm4329_clip_tlv, -4816, 0);

static const struct snd_kcontrol_new ssm4329_amplifier_boost_control =
	SOC_DAPM_SINGLE("Switch", SSM4329_REG_AMP_SNS_CTRL, 2, 1, 1);

static const char const *ssm4329_dac_mux_text[] = {
	"Serial Port 1",
	"Serial Port 2",
	"DSP",
	"ADC",
};

static SOC_ENUM_SINGLE_DECL(ssm4329_dac_mux_enum, SSM4329_REG_ROUTE_DAC_INT,
	0, ssm4329_dac_mux_text);
static SOC_ENUM_SINGLE_DECL(ssm4329_interp_mux_enum, SSM4329_REG_ROUTE_DAC_INT,
	4, ssm4329_dac_mux_text);

static const struct snd_kcontrol_new ssm4329_dac_mux_ctrl = SOC_DAPM_ENUM(
	"DAC Output Mux", ssm4329_dac_mux_enum);
static const struct snd_kcontrol_new ssm4329_interp_mux_ctrl = SOC_DAPM_ENUM(
	"Interpolator Output Mux", ssm4329_interp_mux_enum);

static const char const *ssm4329_sp_enum_text[] = {
	"ADC",
	"AEC",
	"DSP",
	"SRC",
	"Voltage Sense",
	"Current Sense",
	"VBAT",
	"Voltage Sense/VBAT",
	"Current Sense/VBAT",
	"Interpolator",
};

#define SSM4329_SERIAL_PORT_OUT_ENUM_DECL(_port, _channel, _reg, _shift) \
	SOC_ENUM_SINGLE_DECL(ssm4329_sp ## _port ## _ch ## _channel ## _enum, \
		_reg, _shift, ssm4329_sp_enum_text)

#define SSM4329_SERIAL_PORT_OUT_CTRL_DECL(_port, _channel) \
	const struct snd_kcontrol_new \
		ssm4329_sp ## _port ## _ch ## _channel ## _ctrl = SOC_DAPM_ENUM( \
		"Serial Port " #_port " Channel " #_channel " Output Mux", \
		ssm4329_sp ## _port ## _ch ## _channel ## _enum)

#define SSM4329_SERIAL_PORT_OUT_MUX(_port, _channel) \
	SND_SOC_DAPM_MUX("Serial Port " #_port " Channel " #_channel " Output Mux", \
		SND_SOC_NOPM, 0, 0, \
		&ssm4329_sp ## _port ## _ch ## _channel ## _ctrl)

static SSM4329_SERIAL_PORT_OUT_ENUM_DECL(1, 1, SSM4329_REG_ROUTE_SP1_1, 0);
static SSM4329_SERIAL_PORT_OUT_ENUM_DECL(1, 2, SSM4329_REG_ROUTE_SP1_1, 4);
static SSM4329_SERIAL_PORT_OUT_ENUM_DECL(1, 3, SSM4329_REG_ROUTE_SP1_2, 0);
static SSM4329_SERIAL_PORT_OUT_ENUM_DECL(1, 4, SSM4329_REG_ROUTE_SP1_2, 4);
static SSM4329_SERIAL_PORT_OUT_ENUM_DECL(2, 1, SSM4329_REG_ROUTE_SP2_1, 0);
static SSM4329_SERIAL_PORT_OUT_ENUM_DECL(2, 2, SSM4329_REG_ROUTE_SP2_1, 4);
static SSM4329_SERIAL_PORT_OUT_ENUM_DECL(2, 3, SSM4329_REG_ROUTE_SP2_2, 0);
static SSM4329_SERIAL_PORT_OUT_ENUM_DECL(2, 4, SSM4329_REG_ROUTE_SP2_2, 4);
static SSM4329_SERIAL_PORT_OUT_CTRL_DECL(1, 1);
static SSM4329_SERIAL_PORT_OUT_CTRL_DECL(1, 2);
static SSM4329_SERIAL_PORT_OUT_CTRL_DECL(1, 3);
static SSM4329_SERIAL_PORT_OUT_CTRL_DECL(1, 4);
static SSM4329_SERIAL_PORT_OUT_CTRL_DECL(2, 1);
static SSM4329_SERIAL_PORT_OUT_CTRL_DECL(2, 2);
static SSM4329_SERIAL_PORT_OUT_CTRL_DECL(2, 3);
static SSM4329_SERIAL_PORT_OUT_CTRL_DECL(2, 4);

static const char *ssm4329_in_src_ch1_mux_text[] = {
    "Serial Port 1 Channel 1",
    "Serial Port 1 Channel 3",
    "Serial Port 2 Channel 1",
    "Serial Port 2 Channel 3",
    "ADC",
    "DSP",
};

static const char *ssm4329_in_src_ch2_mux_text[] = {
    "Serial Port 1 Channel 2",
    "Serial Port 1 Channel 4",
    "Serial Port 2 Channel 2",
    "Serial Port 2 Channel 4",
    "ADC",
    "DSP",
};

static SOC_ENUM_SINGLE_DECL(ssm4329_in_src_ch1_mux_enum,
	SSM4329_REG_ROUTE_SRC_IN, 0, ssm4329_in_src_ch1_mux_text);
static SOC_ENUM_SINGLE_DECL(ssm4329_in_src_ch2_mux_enum,
	SSM4329_REG_ROUTE_SRC_IN, 4, ssm4329_in_src_ch2_mux_text);

static const struct snd_kcontrol_new ssm4329_in_src_ch1_mux_ctrl = SOC_DAPM_ENUM(
	"Input SRC Channel 1 Mux", ssm4329_in_src_ch1_mux_enum);

static const struct snd_kcontrol_new ssm4329_in_src_ch2_mux_ctrl = SOC_DAPM_ENUM(
	"Input SRC Channel 2 Mux", ssm4329_in_src_ch2_mux_enum);

static const char *ssm4329_out_src_mux_text[] = {
    "DSP",
    "AEC",
    "Input SRC",
    "ADC",
};

static SOC_ENUM_SINGLE_DECL(ssm4329_out_src_ch1_mux_enum,
	SSM4329_REG_ROUTE_SRC_OUT, 0, ssm4329_out_src_mux_text);
static SOC_ENUM_SINGLE_DECL(ssm4329_out_src_ch2_mux_enum,
	SSM4329_REG_ROUTE_SRC_OUT, 4, ssm4329_out_src_mux_text);

static const struct snd_kcontrol_new ssm4329_out_src_ch1_mux_ctrl = SOC_DAPM_ENUM(
	"Output SRC Channel 1 Mux", ssm4329_out_src_ch1_mux_enum);

static const struct snd_kcontrol_new ssm4329_out_src_ch2_mux_ctrl = SOC_DAPM_ENUM(
	"Output SRC Channel 2 Mux", ssm4329_out_src_ch2_mux_enum);

static const struct snd_soc_dapm_widget ssm4329_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("SP1 IN", NULL, 0, SSM4329_REG_DIG_PWR1, 1, 1),
	SND_SOC_DAPM_AIF_OUT("SP1 OUT", NULL, 0, SSM4329_REG_DIG_PWR1, 2, 1),
	SND_SOC_DAPM_AIF_IN("SP2 IN", NULL, 0, SSM4329_REG_DIG_PWR1, 3, 1),
	SSM4329_SP2_OUT_WIDGET(),
	SND_SOC_DAPM_PGA("Interpolator", SSM4329_REG_DIG_PWR1, 5, 1, NULL, 0),
	SND_SOC_DAPM_PGA("DAC AEC", SSM4329_REG_DIG_PWR1, 6, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Interpolator AEC", SSM4329_REG_DIG_PWR1, 7, 1, NULL, 0),

	SND_SOC_DAPM_ADC("ADC", NULL, SSM4329_REG_ANA_PWR, 2, 1),

	SND_SOC_DAPM_SUPPLY("Boost Converter enable", SSM4329_REG_ANA_PWR, 1, 1,
		NULL, 0),
	SND_SOC_DAPM_DAC("DAC", NULL, SSM4329_REG_ANA_PWR, 0, 1),
	SND_SOC_DAPM_SWITCH("Amplifier Boost", SND_SOC_NOPM, 0, 0,
		&ssm4329_amplifier_boost_control),

	SND_SOC_DAPM_SIGGEN("Voltage Sense"),
	SND_SOC_DAPM_SIGGEN("Current Sense"),
	SND_SOC_DAPM_SIGGEN("VBAT"),
	SND_SOC_DAPM_SIGGEN("DSP Siggen"),
	SND_SOC_DAPM_PGA("DSP Processor", SSM4329_REG_DIG_PWR1, 0, 1, NULL, 0),

	SSM4329_SERIAL_PORT_OUT_MUX(1, 1),
	SSM4329_SERIAL_PORT_OUT_MUX(1, 2),
	SSM4329_SERIAL_PORT_OUT_MUX(1, 3),
	SSM4329_SERIAL_PORT_OUT_MUX(1, 4),
	SSM4329_SERIAL_PORT_OUT_MUX(2, 1),
	SSM4329_SERIAL_PORT_OUT_MUX(2, 2),
	SSM4329_SERIAL_PORT_OUT_MUX(2, 3),
	SSM4329_SERIAL_PORT_OUT_MUX(2, 4),

	SND_SOC_DAPM_MUX("DAC Playback Mux", SND_SOC_NOPM, 0, 0,
		&ssm4329_dac_mux_ctrl),
	SND_SOC_DAPM_MUX("Interpolator Playback Mux", SND_SOC_NOPM, 0, 0,
		&ssm4329_interp_mux_ctrl),

	SND_SOC_DAPM_MUX("Input SRC Channel 1 Mux", SSM4329_REG_DIG_PWR1, 0, 1,
		&ssm4329_in_src_ch1_mux_ctrl),
	SND_SOC_DAPM_MUX("Input SRC Channel 2 Mux", SSM4329_REG_DIG_PWR1, 1, 1,
		&ssm4329_in_src_ch2_mux_ctrl),
	SND_SOC_DAPM_MUX("Output SRC Channel 1 Mux", SSM4329_REG_DIG_PWR1, 2, 1,
		&ssm4329_out_src_ch1_mux_ctrl),
	SND_SOC_DAPM_MUX("Output SRC Channel 2 Mux", SSM4329_REG_DIG_PWR1, 3, 1,
		&ssm4329_out_src_ch2_mux_ctrl),

	SND_SOC_DAPM_OUTPUT("OUT"),
	SND_SOC_DAPM_INPUT("AIN"),
};

#define SSM4329_SP_ROUTE(_sp, _ch, _switch, _source) \
	{ "Serial Port " #_sp " Channel " #_ch " Output Mux", _switch, _source }

#define SSM4329_SP_ROUTES(_sp, _ch) \
	SSM4329_SP_ROUTE(_sp, _ch, "ADC", "ADC"), \
	SSM4329_SP_ROUTE(_sp, _ch, "DSP", "DSP"), \
	SSM4329_SP_ROUTE(_sp, _ch, "AEC", \
		(_ch % 2 == 0) ? "DAC AEC" : "Interpolator AEC"), \
	SSM4329_SP_ROUTE(_sp, _ch, "SRC", \
		(_ch % 2 == 0) ? "Output SRC Channel 1 Mux" : \
		    "Output SRC Channel 2 Mux"), \
	SSM4329_SP_ROUTE(_sp, _ch, "Voltage Sense", "Voltage Sense"), \
	SSM4329_SP_ROUTE(_sp, _ch, "Current Sense", "Current Sense"), \
	SSM4329_SP_ROUTE(_sp, _ch, "VBAT", "VBAT"), \
	SSM4329_SP_ROUTE(_sp, _ch, "Voltage Sense/VBAT", "Voltage Sense"), \
	SSM4329_SP_ROUTE(_sp, _ch, "Voltage Sense/VBAT", "VBAT"), \
	SSM4329_SP_ROUTE(_sp, _ch, "Current Sense/VBAT", "Current Sense"), \
	SSM4329_SP_ROUTE(_sp, _ch, "Current Sense/VBAT", "VBAT"), \
	SSM4329_SP_ROUTE(_sp, _ch, "Interpolator", "Interpolator")

static const struct snd_soc_dapm_route ssm4329_dapm_routes[] = {
	{ "ADC", NULL, "AIN" },
	{ "OUT", NULL, "DAC" },
	{ "OUT", NULL, "Amplifier Boost" },

	{ "DAC", NULL, "DAC Playback Mux" },
	{ "DAC Playback Mux", "Serial Port 1", "SP1 IN" },
	{ "DAC Playback Mux", "Serial Port 2", "SP2 IN" },
	{ "DAC Playback Mux", "ADC", "ADC" },
	{ "DAC Playback Mux", "DSP", "DSP Siggen" },

	{ "Interpolator", NULL, "Interpolator Playback Mux" },
	{ "Interpolator Playback Mux", "Serial Port 1", "SP1 IN" },
	{ "Interpolator Playback Mux", "Serial Port 2", "SP2 IN" },
	{ "Interpolator Playback Mux", "ADC", "ADC" },
	{ "Interpolator Playback Mux", "DSP", "DSP Processor" },

	{ "DSP Processor", NULL, "Voltage Sense" },
	{ "DSP Processor", NULL, "Current Sense" },
	{ "DSP Processor", NULL, "VBAT" },
	{ "DSP Processor", NULL, "SP1 IN" },
	{ "DSP Processor", NULL, "SP2 IN" },
	{ "DSP Processor", NULL, "ADC" },
	{ "DSP Processor", NULL, "Input SRC Channel 1 Mux" },
	{ "DSP Processor", NULL, "Input SRC Channel 2 Mux" },
	{ "DSP Processor", NULL, "Output SRC Channel 1 Mux" },
	{ "DSP Processor", NULL, "Output SRC Channel 2 Mux" },

	{ "DSP Processor", NULL, "DSP Siggen" },

	{ "Amplifier Boost", "Switch", "DAC" },
	{ "Amplifier Boost", NULL, "Boost Converter enable" },

	SSM4329_SP_ROUTES(1, 1),
	SSM4329_SP_ROUTES(1, 2),
	SSM4329_SP_ROUTES(1, 3),
	SSM4329_SP_ROUTES(1, 4),
	SSM4329_SP_ROUTES(2, 1),
	SSM4329_SP_ROUTES(2, 2),
	SSM4329_SP_ROUTES(2, 3),
	SSM4329_SP_ROUTES(2, 4),

	{ "Input SRC Channel 1 Mux", "Serial Port 1 Channel 1", "SP1 IN" },
	{ "Input SRC Channel 1 Mux", "Serial Port 1 Channel 1", "SP2 IN" },
	{ "Input SRC Channel 1 Mux", "Serial Port 2 Channel 3", "SP1 IN" },
	{ "Input SRC Channel 1 Mux", "Serial Port 2 Channel 3", "SP2 IN" },
	{ "Input SRC Channel 1 Mux", "ADC", "ADC" },
	{ "Input SRC Channel 1 Mux", "DSP", "DSP Processor" },

	{ "Input SRC Channel 2 Mux", "Serial Port 1 Channel 2", "SP1 IN" },
	{ "Input SRC Channel 2 Mux", "Serial Port 1 Channel 2", "SP2 IN" },
	{ "Input SRC Channel 2 Mux", "Serial Port 2 Channel 4", "SP1 IN" },
	{ "Input SRC Channel 2 Mux", "Serial Port 2 Channel 4", "SP2 IN" },
	{ "Input SRC Channel 2 Mux", "ADC", "ADC" },
	{ "Input SRC Channel 2 Mux", "DSP", "DSP Processor" },

	{ "Output SRC Channel 1 Mux", "DSP", "DSP Processor" },
	{ "Output SRC Channel 1 Mux", "AEC", "DAC AEC" },
	{ "Output SRC Channel 1 Mux", "Input SRC", "Input SRC Channel 1 Mux" },
	{ "Output SRC Channel 1 Mux", "ADC", "ADC" },

	{ "Output SRC Channel 2 Mux", "DSP", "DSP Processor" },
	{ "Output SRC Channel 2 Mux", "AEC", "Interpolator AEC" },
	{ "Output SRC Channel 2 Mux", "Input SRC", "Input SRC Channel 2 Mux" },
	{ "Output SRC Channel 2 Mux", "ADC", "ADC" },

	{ "SP1 Capture", NULL, "SP1 OUT" },
	{ "SP2 Capture", NULL, "SP2 OUT" },

	{ "SP1 IN", NULL, "SP1 Playback" },
	{ "SP2 IN", NULL, "SP2 Playback"  },
};

static const struct snd_soc_dapm_route ssm4329_sp1_out_routes[] = {
	{ "SP1 OUT", NULL, "Serial Port 1 Channel 1 Output Mux" },
	{ "SP1 OUT", NULL, "Serial Port 1 Channel 2 Output Mux" },
	{ "SP1 OUT", NULL, "Serial Port 1 Channel 3 Output Mux" },
	{ "SP1 OUT", NULL, "Serial Port 1 Channel 4 Output Mux" },
};


static const struct snd_soc_dapm_route ssm4329_sp2_out_routes[] = {
	{ "SP2 OUT", NULL, "Serial Port 2 Channel 1 Output Mux" },
	{ "SP2 OUT", NULL, "Serial Port 2 Channel 2 Output Mux" },
	{ "SP2 OUT", NULL, "Serial Port 2 Channel 3 Output Mux" },
	{ "SP2 OUT", NULL, "Serial Port 2 Channel 4 Output Mux" },
};

static const struct snd_kcontrol_new ssm4329_snd_controls[] = {
	SOC_SINGLE_TLV("DAC Playback Volume", SSM4329_REG_DAC_VOLUME,
		0, 0xff, 1, ssm4329_volume_tlv),
	SOC_SINGLE_TLV("DAC Clip Point Playback Volume", SSM4329_REG_DAC_CLIP,
		0, 0xff, 0, ssm4329_clip_tlv),
	SOC_SINGLE_TLV("Interpolator Playback Volume",
		SSM4329_REG_INTERP_VOLUME, 0, 0xff, 1, ssm4329_volume_tlv),
	SOC_SINGLE_TLV("Interpolator Clip Point Playback Volume",
		SSM4329_REG_INTERP_CLIP, 0, 0xff, 0, ssm4329_clip_tlv),

	SOC_SINGLE_TLV("ADC Capture Volume", SSM4329_REG_ADC_VOLUME,
		0, 0xff, 1, ssm4329_volume_tlv),

	SOC_SINGLE("DAC High Pass Filter Playback Switch", SSM4329_REG_DAC_CTRL,
		5, 1, 0),
	SOC_SINGLE("DAC Low-power Playback Switch", SSM4329_REG_DAC_CTRL, 4, 1, 0),

	SOC_SINGLE("ADC High Pass Filter Capture Switch", SSM4329_REG_ADC_CTRL,
		5, 1, 0),
	SOC_SINGLE("ADC Low-power Capture Switch", SSM4329_REG_ADC_CTRL, 4, 1, 0),

	SOC_SINGLE("DAC Playback Switch", SSM4329_REG_DAC_CTRL, 6, 1, 1),
	SOC_SINGLE("ADC Capture Switch", SSM4329_REG_ADC_CTRL, 6, 1, 1),
	SOC_SINGLE("Interpolator Playback Switch", SSM4329_REG_DAC_INTERP_CTRL,
		0, 1, 1),

	SOC_SINGLE("Low-EMI Mode Playback Switch",  SSM4329_REG_AMP_SNS_CTRL, 2, 1,
		0),
};

static int ssm4329_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct ssm4329 *ssm4329 = snd_soc_dai_get_drvdata(dai);
	unsigned int reg_base;
	unsigned int ctrl1_val = 0, ctrl1_mask = 0;
	unsigned int clock_val = 0;
	unsigned int slot_width;
	bool lrclk_master;
	bool bclk_master;
	int ret;

	if (dai->id == 0)
		reg_base = SSM4329_REG_SPT1_BASE;
	else
		reg_base = SSM4329_REG_SPT2_BASE;

	switch (ssm4329->dais[dai->id].fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		bclk_master = false;
		lrclk_master = false;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		bclk_master = true;
		lrclk_master = false;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		bclk_master = false;
		lrclk_master = true;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		bclk_master = true;
		lrclk_master = true;
		break;
	default:
		return -EINVAL;
	}

	slot_width = ssm4329->dais[dai->id].slot_width;
	if (slot_width == 0) {
		if (bclk_master) {
			slot_width = 32;
		} else {
			/*
			 * If the slot width was not explcitly configured we assume that
			 * it is equal to params_width(). The might not always be true, but
			 * it is the best we can do at the moment.
			 */
			slot_width = params_width(params);
		}

		switch (slot_width) {
		case 16:
			ctrl1_val |= SSM4329_SPT_CTRL1_SLOT_WIDTH_16;
			break;
		case 24:
			ctrl1_val |= SSM4329_SPT_CTRL1_SLOT_WIDTH_24;
			break;
		case 32:
			ctrl1_val |= SSM4329_SPT_CTRL1_SLOT_WIDTH_32;
			break;
		default:
			return -EINVAL;
		}
		ctrl1_mask |= SSM4329_SPT_CTRL1_SLOT_WIDTH_MASK;
	}

	if ((ssm4329->dais[dai->id].fmt & SND_SOC_DAIFMT_FORMAT_MASK) ==
		SND_SOC_DAIFMT_RIGHT_J) {
		switch (slot_width - params_width(params)) {
		case 0:
			ctrl1_val |= SSM4329_SPT_CTRL1_DELAY0;
			break;
		case 8:
			ctrl1_val |= SSM4329_SPT_CTRL1_DELAY8;
			break;
		case 12:
			ctrl1_val |= SSM4329_SPT_CTRL1_DELAY12;
			break;
		case 16:
			ctrl1_val |= SSM4329_SPT_CTRL1_DELAY16;
			break;
		default:
			return -EINVAL;
		}
		ctrl1_mask |= SSM4329_SPT_CTRL1_DELAY_MASK;
	}

	if (lrclk_master) {
		switch (params_rate(params)) {
		case 8000:
			clock_val |= SSM4329_SPT_CLOCKING_LRCLK_SRC_8;
			break;
		case 12000:
			clock_val |= SSM4329_SPT_CLOCKING_LRCLK_SRC_12;
			break;
		case 16000:
			clock_val |= SSM4329_SPT_CLOCKING_LRCLK_SRC_16;
			break;
		case 24000:
			clock_val |= SSM4329_SPT_CLOCKING_LRCLK_SRC_24;
			break;
		case 48000:
			clock_val |= SSM4329_SPT_CLOCKING_LRCLK_SRC_48;
			break;
		case 96000:
			clock_val |= SSM4329_SPT_CLOCKING_LRCLK_SRC_96;
			break;
		case 192000:
			clock_val |= SSM4329_SPT_CLOCKING_LRCLK_SRC_192;
			break;
		default:
			return -EINVAL;
		}

		regmap_update_bits(ssm4329->regmap,
			reg_base + SSM4329_REG_SPT_CLOCKING,
			SSM4329_SPT_CLOCKING_LRCLK_SRC_MASK, clock_val);
	}

	if (ctrl1_mask != 0)
		regmap_update_bits(ssm4329->regmap,
			reg_base + SSM4329_REG_SPT_CTRL1, ctrl1_mask, ctrl1_val);


	ret = sigmadsp_setup(&ssm4329->sigmadsp, params_rate(params));
	if (ret)
		return ret;

	regmap_write(ssm4329->regmap, SSM4329_REG_SDSP_CTRL2, 6);
	regmap_write(ssm4329->regmap, SSM4329_REG_SDSP_CTRL3, 1);


	return 0;
}

static int ssm4329_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	struct ssm4329 *ssm4329 = snd_soc_codec_get_drvdata(codec);

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
		regmap_update_bits(ssm4329->regmap, SSM4329_REG_CHIP_PWR,
			SSM4329_REG_CHIP_PWR_CHIP_PWDN, 0x00);
		break;
	case SND_SOC_BIAS_OFF:
		regmap_update_bits(ssm4329->regmap, SSM4329_REG_CHIP_PWR,
			SSM4329_REG_CHIP_PWR_CHIP_PWDN,
			SSM4329_REG_CHIP_PWR_CHIP_PWDN);
		break;
	default:
		break;
	}

	codec->dapm.bias_level = level;

	return 0;
}

static int ssm4329_check_dai_cfg(struct snd_soc_dai *dai,
	struct ssm4329_dai_cfg *dai_cfg)
{
	struct ssm4329 *ssm4329 = snd_soc_dai_get_drvdata(dai);
	bool bclk_master;
	bool tdmc_mode;

	if (dai->id == 1 && ssm4329_tdmc_active(ssm4329))
		tdmc_mode = true;
	else
		tdmc_mode = false;

	switch (dai_cfg->fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
	case SND_SOC_DAIFMT_CBS_CFM:
		if (tdmc_mode)
			return -EINVAL;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
	case SND_SOC_DAIFMT_CBM_CFS:
		bclk_master = true;
		break;
	default:
		return -EINVAL;
	}

	switch (dai_cfg->fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_NF:
	case SND_SOC_DAIFMT_NB_IF:
	case SND_SOC_DAIFMT_IB_IF:
		if (tdmc_mode)
			return -EINVAL;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}

	switch (dai_cfg->fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_DSP_B:
		if (tdmc_mode)
			return -EINVAL;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		break;
	default:
		return -EINVAL;
	}

	switch (dai_cfg->slot_width) {
	case 16:
	case 24:
	case 0:
	case 32:
		if (tdmc_mode)
			return -EINVAL;
		break;
	case 64:
		if (!tdmc_mode)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	if (tdmc_mode && dai_cfg->slots != 1)
		return -EINVAL;

	if (bclk_master) {
		if (dai_cfg->slot_width == 0) {
			if (dai_cfg->slots != 2)
				return -EINVAL;
		} else {
			switch (dai_cfg->slots * dai_cfg->slot_width) {
			case 64:
			case 128:
			case 256:
			case 512:
				break;
			default:
				return -EINVAL;
			}
		}
	}

	return 0;
}

static void ssm4329_update_dai_routes(struct ssm4329 *ssm4329,
	unsigned int dai_id, unsigned int old, unsigned int new)
{
	const struct snd_soc_dapm_route *routes;
	unsigned int old_state, new_state;
	unsigned int i;

	if (!ssm4329->dapm)
		return;

	if (old == new)
		return;

	if (dai_id == 0)
	    routes = ssm4329_sp1_out_routes;
	else
	    routes = ssm4329_sp2_out_routes;

    for (i = 0; i < 4; i++) {
		old_state = old & BIT(i);
		new_state = new & BIT(i);
		if (old_state != new_state) {
		    if (new_state)
				snd_soc_dapm_add_routes(ssm4329->dapm, &routes[i], 1);
		    else
				snd_soc_dapm_del_routes(ssm4329->dapm, &routes[i], 1);
		}
    }
    snd_soc_dapm_sync(ssm4329->dapm);
}

static int ssm4329_update_dai_cfg(struct ssm4329 *ssm4329, unsigned int id,
	struct ssm4329_dai_cfg *dai_cfg)
{
	struct ssm4329_dai_cfg *old_cfg = &ssm4329->dais[id];
	unsigned int ctrl1_val = 0, clock_val = 0;
	unsigned int reg_base;
	unsigned int dai_fmt;
	bool invert_lrclk;
	bool bclk_master;
	bool lrclk_master;

	/*
	 * Can't use dai->driver->base as this also needs to be called from the tdmc
	 * code even before the CODEC has probed.
	 */
	if (id == 0)
		reg_base = SSM4329_REG_SPT1_BASE;
	else
		reg_base = SSM4329_REG_SPT2_BASE;

	switch (dai_cfg->fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		bclk_master = false;
		lrclk_master = false;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		bclk_master = true;
		lrclk_master = false;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		bclk_master = false;
		lrclk_master = true;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		bclk_master = true;
		lrclk_master = true;
		break;
	default:
		return -EINVAL;
	}

	dai_fmt = dai_cfg->fmt & SND_SOC_DAIFMT_FORMAT_MASK;

	switch (dai_fmt) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_DSP_A:
		ctrl1_val |= SSM4329_SPT_CTRL1_DELAY1;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_DSP_B:
		/* For RJ this gets adjusted in hw_params() */
		ctrl1_val |= SSM4329_SPT_CTRL1_DELAY0;
		break;
	default:
		return -EINVAL;
	}

	switch (dai_cfg->fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		invert_lrclk = false;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		ctrl1_val |= SSM4329_SPT_CLOCKING_BCLK_POL;
		invert_lrclk = false;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		invert_lrclk = true;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		ctrl1_val |= SSM4329_SPT_CLOCKING_BCLK_POL;
		invert_lrclk = true;
		break;
	default:
		return -EINVAL;
	}

	switch (dai_fmt) {
	/* I2S is falling edge = leading edge */
	case SND_SOC_DAIFMT_I2S:
		break;
	/* Everything else is rising edge = leading edge */
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
	case SND_SOC_DAIFMT_LEFT_J:
	case SND_SOC_DAIFMT_RIGHT_J:
		invert_lrclk = !invert_lrclk;
		break;
	default:
		return -EINVAL;
	}

	/*
	 * In 'TDM' mode the DAI syncs the first channel on the leading
	 * frame-clock edge and advances one channel every slot-width bit-block
	 * cycles. In 'stereo' mode it syncs the first channel on the leading
	 * edge and the second channel on the trailing edge of the frame clock.
	 */
	if (dai_fmt == SND_SOC_DAIFMT_DSP_A ||
	    dai_fmt == SND_SOC_DAIFMT_DSP_B || dai_cfg->slot_width != 0) {
		invert_lrclk = !invert_lrclk;
		ctrl1_val |= SSM4329_SPT_CTRL1_MODE_TDM;
	} else {
		ctrl1_val |= SSM4329_SPT_CTRL1_MODE_STEREO;
	}

	switch (dai_cfg->slot_width) {
	case 16:
		ctrl1_val |= SSM4329_SPT_CTRL1_SLOT_WIDTH_16;
		break;
	case 24:
		ctrl1_val |= SSM4329_SPT_CTRL1_SLOT_WIDTH_24;
		break;
	case 0: /* Will be overwritten in hw_params */
	case 32:
	case 64: /* Special case for tdmc mode */
		ctrl1_val |= SSM4329_SPT_CTRL1_SLOT_WIDTH_32;
		break;
	default:
		return -EINVAL;
	}

	if (bclk_master) {
	    switch (dai_cfg->slots * dai_cfg->slot_width) {
		case 0:
		case 64:
		    clock_val |= SSM4329_SPT_CLOCKING_BCLK_SRC_64;
		    break;
		case 128:
		    clock_val |= SSM4329_SPT_CLOCKING_BCLK_SRC_128;
		    break;
		case 256:
		    clock_val |= SSM4329_SPT_CLOCKING_BCLK_SRC_256;
		    break;
		case 512:
		    clock_val |= SSM4329_SPT_CLOCKING_BCLK_SRC_512;
		    break;
		default:
		    return -EINVAL;
		}
	} else {
		clock_val |= SSM4329_SPT_CLOCKING_BCLK_SRC_EXT;
	}

	if (invert_lrclk)
	    clock_val |= SSM4329_SPT_CLOCKING_LRCLK_POL;
	
	if (!lrclk_master)
		clock_val |= SSM4329_SPT_CLOCKING_LRCLK_SRC_EXT;
	else /* Will be overwritten in hw_params */
		clock_val |= SSM4329_SPT_CLOCKING_LRCLK_SRC_48;

	ssm4329_lock(ssm4329);

	regmap_write(ssm4329->regmap, reg_base + SSM4329_REG_SPT_CTRL1,
		ctrl1_val);
	regmap_write(ssm4329->regmap, reg_base + SSM4329_REG_SPT_CLOCKING,
		clock_val);

	ssm4329_update_dai_routes(ssm4329, id, old_cfg->out_active_channels,
		dai_cfg->out_active_channels);

	*old_cfg = *dai_cfg;

	ssm4329_unlock(ssm4329);

	return 0;
}

static int ssm4329_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
	unsigned int rx_mask, int slots, int width)
{
	struct ssm4329 *ssm4329 = snd_soc_dai_get_drvdata(dai);
	struct ssm4329_dai_cfg cfg = ssm4329->dais[dai->id];
	unsigned int reg_base;
	unsigned int rx_slot[4];
	unsigned int tx_slot[4];
	unsigned int i;
	int ret;

	if (dai->id == 0)
		reg_base = SSM4329_REG_SPT1_BASE;
	else
		reg_base = SSM4329_REG_SPT2_BASE;

	/* Stereo mode */
	if (slots == 0) {
		slots = 2;
		width = 0;
	}

	cfg.slot_width = width;
	cfg.slots = slots;

	ret = ssm4329_check_dai_cfg(dai, &cfg);
	if (ret)
	    return ret;

	for (i = 0; i < 4; i++) {
		if (rx_mask) {
			rx_slot[i] = __ffs(rx_mask);
			rx_mask &= ~(1 << rx_slot[i]);
			if (rx_slot[i] >= slots || rx_slot[i] >= 16)
				return -EINVAL;
		} else {
			rx_slot[i] = i;
		}
	}

	if (rx_mask != 0)
		return -EINVAL;

	cfg.out_active_channels = 0;

	for (i = 0; i < 4; i++) {
		if (tx_mask) {
			tx_slot[i] = __ffs(tx_mask);
			tx_mask &= ~(1 << tx_slot[i]);
			if (tx_slot[i] >= slots || tx_slot[i] >= 16)
				return -EINVAL;
			cfg.out_active_channels |= BIT(i);
		} else {
			tx_slot[i] = 16;
		}
	}

	regmap_write(ssm4329->regmap, reg_base + SSM4329_REG_SPT_INPUTS1,
		(rx_slot[1] << 4) | rx_slot[0]);
	regmap_write(ssm4329->regmap, reg_base + SSM4329_REG_SPT_INPUTS2,
		(rx_slot[3] << 4) | rx_slot[2]);
	regmap_write(ssm4329->regmap, reg_base + SSM4329_REG_SPT_OUTPUTS1,
		tx_slot[0]);
	regmap_write(ssm4329->regmap, reg_base + SSM4329_REG_SPT_OUTPUTS2,
		tx_slot[1]);
	regmap_write(ssm4329->regmap, reg_base + SSM4329_REG_SPT_OUTPUTS3,
		tx_slot[2]);
	regmap_write(ssm4329->regmap, reg_base + SSM4329_REG_SPT_OUTPUTS4,
		tx_slot[3]);

	return ssm4329_update_dai_cfg(ssm4329, dai->id, &cfg);
}

static int ssm4329_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct ssm4329 *ssm4329 = snd_soc_dai_get_drvdata(dai);
	struct ssm4329_dai_cfg cfg = ssm4329->dais[dai->id];
	int ret;

	cfg.fmt = fmt;
	ret = ssm4329_check_dai_cfg(dai, &cfg);
	if (ret)
	    return ret;

	return ssm4329_update_dai_cfg(ssm4329, dai->id, &cfg);
}

static int ssm4329_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	return 0;
}

static int ssm4329_set_tristate(struct snd_soc_dai *dai, int tristate)
{
	struct ssm4329 *ssm4329 = snd_soc_dai_get_drvdata(dai);
	unsigned int reg_base;
	unsigned int val;

	if (dai->id == 0)
		reg_base = SSM4329_REG_SPT1_BASE;
	else
		reg_base = SSM4329_REG_SPT2_BASE;

	if (tristate)
		val = SSM4329_SPT_CTRL2_TRI_STATE;
	else
		val = 0;

	return regmap_update_bits(ssm4329->regmap,
		reg_base + SSM4329_REG_SPT_CTRL2,
		SSM4329_SPT_CTRL2_TRI_STATE, val);
}

static int ssm4329_set_pll(struct snd_soc_codec *codec, int pll_id,
	int source, unsigned int freq_in, unsigned int freq_out)
{
	struct ssm4329 *ssm4329 = snd_soc_codec_get_drvdata(codec);
	unsigned int div;
	unsigned int r, n, m, i, j;

	if (pll_id != SSM4329_PLL)
	    return -EINVAL;
	
	switch (source) {
	case SSM4329_PLL_SRC_MCLKIN:
	case SSM4329_PLL_SRC_FSYNC1:
	case SSM4329_PLL_SRC_BCLK1:
	case SSM4329_PLL_SRC_FSYNC2:
	case SSM4329_PLL_SRC_BCLK2:
		break;
	default:
		return -EINVAL;
	}

	if (freq_in < 8000 || freq_in > 27000000)
		return -EINVAL;

	if (!freq_out) {
		r = 0;
		n = 0;
		m = 0;
		div = 0;
	} else {
		div = DIV_ROUND_UP(freq_in, 13500000);
		freq_in /= div;

		if (freq_out % freq_in != 0) {
			r = freq_out / freq_in;
			i = freq_out % freq_in;
			j = gcd(i, freq_in);
			n = i / j;
			m = freq_in / j;
		} else {
			r = freq_out / freq_in;
			n = 0;
			m = 0;
		}

		if (n > 0xffff || m > 0xffff || div == 0 || div > 7 || r > 0x3fff)
			return -EINVAL;
	}

	if (m != 0)
		source |= 1 << 4;

	regmap_write(ssm4329->regmap, SSM4329_REG_CLK_CTRL1, source);
	regmap_write(ssm4329->regmap, SSM4329_REG_CLK_CTRL2, div);
	regmap_write(ssm4329->regmap, SSM4329_REG_CLK_CTRL3, r & 0xff);
	regmap_write(ssm4329->regmap, SSM4329_REG_CLK_CTRL4, (r >> 8) & 0xff);
	regmap_write(ssm4329->regmap, SSM4329_REG_CLK_CTRL5, n & 0xff);
	regmap_write(ssm4329->regmap, SSM4329_REG_CLK_CTRL6, (n >> 8) & 0xff);
	regmap_write(ssm4329->regmap, SSM4329_REG_CLK_CTRL7, m & 0xff);
	regmap_write(ssm4329->regmap, SSM4329_REG_CLK_CTRL8, (m >> 8) & 0xff);

	/* Trigger PLL update */
	regmap_write(ssm4329->regmap, SSM4329_REG_CLK_CTRL9,
		SSM4329_REG_CLK_CTRL9_PLL_UPDATE);
	regmap_write(ssm4329->regmap, SSM4329_REG_CLK_CTRL9, 0);

	return 0;
}

static int ssm4329_codec_probe(struct snd_soc_codec *codec)
{
	struct ssm4329 *ssm4329 = snd_soc_codec_get_drvdata(codec);
	int ret;

	codec->control_data = ssm4329->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 0, 0, SND_SOC_REGMAP);
	if (ret < 0)
		return ret;

	ssm4329_lock(ssm4329);
	ssm4329->dapm = &codec->dapm;

	ssm4329_update_dai_routes(ssm4329, 0, 0,
		ssm4329->dais[0].out_active_channels);
	ssm4329_update_dai_routes(ssm4329, 1, 0,
		ssm4329->dais[1].out_active_channels);

	ssm4329_unlock(ssm4329);

	return 0;
}

static const struct snd_soc_dai_ops ssm4329_dai_ops = {
	.startup	= ssm4329_startup,
	.hw_params	= ssm4329_hw_params,
	.set_fmt	= ssm4329_set_dai_fmt,
	.set_tdm_slot	= ssm4329_set_tdm_slot,
	.set_tristate	= ssm4329_set_tristate,
};

static struct snd_soc_dai_driver ssm4329_dais[] = {
	{
		.name = "ssm4329-sp1",
		.id = 0,
		.playback = {
			.stream_name = "SP1 Playback",
			.channels_min = 1,
			.channels_max = 4,
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S32_LE,
			.sig_bits = 24,
		},
		.capture = {
			.stream_name = "SP1 Capture",
			.channels_min = 1,
			.channels_max = 4,
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S32_LE,
			.sig_bits = 24,
		},
		.ops = &ssm4329_dai_ops,
	}, {
		.name = "ssm4329-sp2",
		.id = 1,
		.playback = {
			.stream_name = "SP2 Playback",
			.channels_min = 1,
			.channels_max = 4,
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S32_LE,
			.sig_bits = 24,
		},
		.capture = {
			.stream_name = "SP2 Capture",
			.channels_min = 1,
			.channels_max = 4,
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S32_LE,
			.sig_bits = 24,
		},
		.ops = &ssm4329_dai_ops,
	},
};

static const struct snd_soc_codec_driver ssm4329_codec_driver = {
	.probe = ssm4329_codec_probe,
	.set_bias_level = ssm4329_set_bias_level,
	.set_pll = ssm4329_set_pll,
	.idle_bias_off = true,

	.controls = ssm4329_snd_controls,
	.num_controls = ARRAY_SIZE(ssm4329_snd_controls),
	.dapm_widgets = ssm4329_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(ssm4329_dapm_widgets),
	.dapm_routes = ssm4329_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(ssm4329_dapm_routes),
};

int ssm4329_probe(struct device *dev, struct regmap *regmap)
{
	unsigned int a, b, c, d;
	struct ssm4329 *ssm4329;
	unsigned int i;
	int ret;

	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	regmap_read(regmap, SSM4329_REG_VENDOR_ID, &a);
	regmap_read(regmap, SSM4329_REG_DEVICE_ID1, &b);
	regmap_read(regmap, SSM4329_REG_DEVICE_ID2, &c);
	regmap_read(regmap, SSM4329_REG_REVISION, &d);
	dev_info(dev, "ssm4329 version: %c-%.2x%.2x.%.2x", a, b, c, d);

	ssm4329 = devm_kzalloc(dev, sizeof(*ssm4329), GFP_KERNEL);
	if (ssm4329 == NULL)
		return -ENOMEM;

	dev_set_drvdata(dev, ssm4329);

	ssm4329->regmap = regmap;

	sigmadsp_init_regmap(&ssm4329->sigmadsp, NULL, regmap);

	/* Power-on default is stero left-justified. */
	for (i = 0; i < ARRAY_SIZE(ssm4329->dais); i++) {
		ssm4329->dais[i].slot_width = 0;
		ssm4329->dais[i].slots = 2;
		ssm4329->dais[i].fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS;
		ssm4329->dais[i].out_active_channels = 0x3;
	}

	ssm4329->supplies[0].supply = "AVDD";
	ssm4329->supplies[1].supply = "IOVDD";

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ssm4329->supplies),
		ssm4329->supplies);
	if (ret)
		return ret;

	ret = ssm4329_tdmc_master_register(dev, ssm4329);
	if (ret)
		return ret;

	return snd_soc_register_codec(dev, &ssm4329_codec_driver,
			ssm4329_dais, ARRAY_SIZE(ssm4329_dais));
}
EXPORT_SYMBOL_GPL(ssm4329_probe);

static bool ssm4329_register_readable(struct device *dev, unsigned int reg)
{
	if (reg < 0x4000)
		return false;
	return true;
}

static bool ssm4329_register_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SSM4329_REG_VENDOR_ID:
	case SSM4329_REG_DEVICE_ID1:
	case SSM4329_REG_DEVICE_ID2:
	case SSM4329_REG_REVISION:
	case SSM4329_REG_EAC_RW_CTRL:
	case SSM4329_REG_VBAT:
	case SSM4329_REG_FAULT_STATUS:
	case SSM4329_REG_PLL_SDSP_STATUS:
	case SSM4329_REG_EAC_STATUS:
	case SSM4329_REG_EAC_RD_DATA:
	case SSM4329_REG_IRQ1_STATUS1:
	case SSM4329_REG_IRQ1_STATUS2:
	case SSM4329_REG_IRQ2_STATUS1:
	case SSM4329_REG_IRQ2_STATUS2:
	case SSM4329_REG_GPIO_IN_READING:
	    return true;
	}

	return false;
}

const struct regmap_config ssm4329_regmap_config = {
	.max_register = SSM4329_REG_GPIO_IN_READING,
	.volatile_reg = ssm4329_register_volatile,
	.readable_reg = ssm4329_register_readable,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = ssm4329_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ssm4329_reg_defaults),
};
EXPORT_SYMBOL_GPL(ssm4329_regmap_config);

MODULE_DESCRIPTION("ASoC SSM4329 driver");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
