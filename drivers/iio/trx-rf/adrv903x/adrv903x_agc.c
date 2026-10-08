// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV903X RF Transceiver - AGC debugfs support
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/debugfs.h>
#include <linux/cleanup.h>
#include <linux/seq_file.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>

#include "adrv903x.h"

#define adrv903x_agc_seq_printf(seq, ptr, member) \
	seq_printf(seq, "%s: %u\n", #member, (ptr)->member)

struct adrv903x_agc_priv {
	struct adrv903x_rf_phy *phy;
	u8 idx;
};

static int adrv903x_agc_config_show(struct seq_file *s, void *ignored)
{
	struct adrv903x_agc_priv *priv = s->private;
	struct adrv903x_rf_phy *phy = priv->phy;
	adi_adrv903x_AgcCfg_t *agc = &phy->rx_agc[priv->idx];
	int ret;

	scoped_guard(mutex, &phy->lock) {
		ret = adrv903x_api_call(phy, adi_adrv903x_AgcCfgGet,
					ADI_ADRV903X_RX0 << priv->idx, agc);
		if (ret)
			return ret;
	}

	adrv903x_agc_seq_printf(s, agc, agcRxMaxGainIndex);
	adrv903x_agc_seq_printf(s, agc, agcRxMinGainIndex);
	adrv903x_agc_seq_printf(s, agc, agcGainUpdateCounter);
	adrv903x_agc_seq_printf(s, agc, agcChangeGainIfThreshHigh);
	adrv903x_agc_seq_printf(s, agc, agcSlowLoopSettlingDelay);
	adrv903x_agc_seq_printf(s, agc, agcEnableFastRecoveryLoop);
	adrv903x_agc_seq_printf(s, agc, agcPeakThreshGainControlMode);
	adrv903x_agc_seq_printf(s, agc, agcLowThreshPreventGainInc);
	adrv903x_agc_seq_printf(s, agc, agcResetOnRxon);
	adrv903x_agc_seq_printf(s, agc, agcRxAttackDelay);
	adrv903x_agc_seq_printf(s, agc, agcAdcResetGainStep);
	adrv903x_agc_seq_printf(s, agc, agcPeakWaitTime);

	/* Power parameters */
	adrv903x_agc_seq_printf(s, &agc->agcPower, powerEnableMeasurement);
	adrv903x_agc_seq_printf(s, &agc->agcPower, powerInputSelect);
	adrv903x_agc_seq_printf(s, &agc->agcPower, powerMeasurementDuration);
	adrv903x_agc_seq_printf(s, &agc->agcPower, underRangeHighPowerThresh);
	adrv903x_agc_seq_printf(s, &agc->agcPower, underRangeHighPowerGainStepRecovery);
	adrv903x_agc_seq_printf(s, &agc->agcPower, underRangeLowPowerThresh);
	adrv903x_agc_seq_printf(s, &agc->agcPower, underRangeLowPowerGainStepRecovery);
	adrv903x_agc_seq_printf(s, &agc->agcPower, overRangeLowPowerThresh);
	adrv903x_agc_seq_printf(s, &agc->agcPower, overRangeLowPowerGainStepAttack);
	adrv903x_agc_seq_printf(s, &agc->agcPower, overRangeHighPowerThresh);
	adrv903x_agc_seq_printf(s, &agc->agcPower, overRangeHighPowerGainStepAttack);

	/* Peak parameters */
	adrv903x_agc_seq_printf(s, &agc->agcPeak, enableHb2Overload);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2OverloadPowerMode);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2OverloadSignalSelection);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2OverloadDurationCnt);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2OverloadThreshCnt);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2HighThresh);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2UpperThreshPeakExceededCnt);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2GainStepAttack);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2UnderRangeHighThresh);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2UnderRangeHighThreshExceededCnt);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2GainStepHighRecovery);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2UnderRangeMidThresh);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2UnderRangeMidThreshExceededCnt);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2GainStepMidRecovery);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2UnderRangeLowThresh);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2UnderRangeLowThreshExceededCnt);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2GainStepLowRecovery);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2UnderRangeLowInterval);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2UnderRangeMidInterval);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, hb2UnderRangeHighInterval);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, adcOvldUpperThreshPeakExceededCnt);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, adcOvldGainStepAttack);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, adcOvldLowThresh);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, adcOvldLowerThreshPeakExceededCnt);
	adrv903x_agc_seq_printf(s, &agc->agcPeak, adcOvldGainStepRecovery);

	return 0;
}

static ssize_t adrv903x_agc_config_write(struct file *file,
					  const char __user *userbuf,
					  size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct adrv903x_agc_priv *priv = s->private;
	struct adrv903x_rf_phy *phy = priv->phy;
	adi_adrv903x_AgcCfg_t *agc = &phy->rx_agc[priv->idx];
	int ret;

	agc->rxChannelMask = ADI_ADRV903X_RX0 << priv->idx;

	/* Firmware returns defaults outside documented valid ranges — clamp before commit */
	agc->agcPeak.adcOvldLowThresh =
		clamp_t(u8, agc->agcPeak.adcOvldLowThresh, 0, 8);
	agc->agcPeak.adcOvldLowerThreshPeakExceededCnt =
		clamp_t(u8, agc->agcPeak.adcOvldLowerThreshPeakExceededCnt, 2, 255);
	agc->agcPeak.adcOvldUpperThreshPeakExceededCnt =
		clamp_t(u8, agc->agcPeak.adcOvldUpperThreshPeakExceededCnt, 2, 255);
	agc->agcPeakWaitTime =
		clamp_t(u8, agc->agcPeakWaitTime, 0, 3);

	guard(mutex)(&phy->lock);
	ret = adrv903x_api_call(phy, adi_adrv903x_AgcCfgSet, agc, 1);

	return ret ? ret : count;
}

static int adrv903x_agc_config_open(struct inode *inode, struct file *file)
{
	return single_open(file, adrv903x_agc_config_show, inode->i_private);
}

static const struct file_operations adrv903x_agc_config_fops = {
	.owner		= THIS_MODULE,
	.open		= adrv903x_agc_config_open,
	.read		= seq_read,
	.write		= adrv903x_agc_config_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void adrv903x_agc_debugfs_create_chan(struct adrv903x_rf_phy *phy,
					     int nr, struct dentry *d)
{
	struct device *dev = &phy->spi->dev;
	adi_adrv903x_AgcCfg_t *agc = &phy->rx_agc[nr];

#define agc_attr(member) \
	devm_kasprintf(dev, GFP_KERNEL, "rx%d_agc_" #member, nr)

#define agc_add_u8(member) \
	debugfs_create_u8(agc_attr(member), 0600, d, &agc->member)

#define agc_add_u16(member) \
	debugfs_create_u16(agc_attr(member), 0600, d, &agc->member)

#define agc_add_u32(member) \
	debugfs_create_u32(agc_attr(member), 0600, d, &agc->member)

	/* Top-level AGC params */
	agc_add_u8(agcRxMaxGainIndex);
	agc_add_u8(agcRxMinGainIndex);
	agc_add_u32(agcGainUpdateCounter);
	agc_add_u8(agcChangeGainIfThreshHigh);
	agc_add_u8(agcSlowLoopSettlingDelay);
	agc_add_u8(agcEnableFastRecoveryLoop);
	agc_add_u8(agcPeakThreshGainControlMode);
	agc_add_u8(agcLowThreshPreventGainInc);
	agc_add_u8(agcResetOnRxon);
	agc_add_u8(agcRxAttackDelay);
	agc_add_u8(agcAdcResetGainStep);
	agc_add_u8(agcPeakWaitTime);

	/* Power parameters */
	agc_add_u8(agcPower.powerEnableMeasurement);
	agc_add_u8(agcPower.powerInputSelect);
	agc_add_u8(agcPower.powerMeasurementDuration);
	agc_add_u8(agcPower.underRangeHighPowerThresh);
	agc_add_u8(agcPower.underRangeHighPowerGainStepRecovery);
	agc_add_u8(agcPower.underRangeLowPowerThresh);
	agc_add_u8(agcPower.underRangeLowPowerGainStepRecovery);
	agc_add_u8(agcPower.overRangeLowPowerThresh);
	agc_add_u8(agcPower.overRangeLowPowerGainStepAttack);
	agc_add_u8(agcPower.overRangeHighPowerThresh);
	agc_add_u8(agcPower.overRangeHighPowerGainStepAttack);

	/* Peak: ADC overload */
	agc_add_u8(agcPeak.adcOvldUpperThreshPeakExceededCnt);
	agc_add_u8(agcPeak.adcOvldGainStepAttack);
	agc_add_u8(agcPeak.adcOvldLowThresh);
	agc_add_u8(agcPeak.adcOvldLowerThreshPeakExceededCnt);
	agc_add_u8(agcPeak.adcOvldGainStepRecovery);

	/* Peak: HB2 overload */
	agc_add_u8(agcPeak.enableHb2Overload);
	agc_add_u8(agcPeak.hb2OverloadPowerMode);
	agc_add_u8(agcPeak.hb2OverloadSignalSelection);
	agc_add_u8(agcPeak.hb2OverloadDurationCnt);
	agc_add_u8(agcPeak.hb2OverloadThreshCnt);
	agc_add_u16(agcPeak.hb2HighThresh);
	agc_add_u8(agcPeak.hb2UpperThreshPeakExceededCnt);
	agc_add_u8(agcPeak.hb2GainStepAttack);
	agc_add_u16(agcPeak.hb2UnderRangeHighThresh);
	agc_add_u8(agcPeak.hb2UnderRangeHighThreshExceededCnt);
	agc_add_u8(agcPeak.hb2GainStepHighRecovery);
	agc_add_u16(agcPeak.hb2UnderRangeMidThresh);
	agc_add_u8(agcPeak.hb2UnderRangeMidThreshExceededCnt);
	agc_add_u8(agcPeak.hb2GainStepMidRecovery);
	agc_add_u16(agcPeak.hb2UnderRangeLowThresh);
	agc_add_u8(agcPeak.hb2UnderRangeLowThreshExceededCnt);
	agc_add_u8(agcPeak.hb2GainStepLowRecovery);
	agc_add_u16(agcPeak.hb2UnderRangeLowInterval);
	agc_add_u8(agcPeak.hb2UnderRangeMidInterval);
	agc_add_u8(agcPeak.hb2UnderRangeHighInterval);

#undef agc_add_u32
#undef agc_add_u16
#undef agc_add_u8
#undef agc_attr
}

void adrv903x_agc_debugfs_register(struct iio_dev *indio_dev)
{
	struct adrv903x_rf_phy *phy = iio_priv(indio_dev);
	struct dentry *d = iio_get_debugfs_dentry(indio_dev);
	struct adrv903x_agc_priv *priv;
	int i;

	if (!d)
		return;

	for (i = 0; i < ADI_ADRV903X_MAX_RX_ONLY; i++) {
		if (phy->chip_info->is_adrv9032r && i != 0 && i != 4)
			continue;

		priv = devm_kzalloc(&phy->spi->dev, sizeof(*priv), GFP_KERNEL);
		if (!priv)
			return;

		priv->phy = phy;
		priv->idx = i;

		debugfs_create_file(devm_kasprintf(&phy->spi->dev, GFP_KERNEL,
						   "rx%d_agc_config", i),
				    0600, d, priv,
				    &adrv903x_agc_config_fops);

		adrv903x_agc_debugfs_create_chan(phy, i, d);
	}
}
