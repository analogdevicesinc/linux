// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9104 debugfs interface
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include "linux/array_size.h"
#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>

#include <linux/iio/iio.h>
#include <linux/iio/backend.h>

#include "adrv9104.h"
#include "adi_adrv910x.h"
#include "adi_adrv910x_rx.h"
#include "adi_adrv910x_ssi.h"
#include "adi_adrv910x_stream.h"

#define adrv9104_seq_printf(seq, ptr, member) \
	seq_printf(seq, "%s: %u\n", #member, (ptr)->member)

static int adrv9104_init_set(void *arg, u64 val)
{
	struct adrv9104_rf_phy *phy = arg;

	if (!val)
		return -EINVAL;

	return adr9104_init(phy);
}
DEFINE_DEBUGFS_ATTRIBUTE(adrv9104_init_fops,
			 NULL, adrv9104_init_set, "%llu");

static int adrv9104_rx_agc_config_show(struct seq_file *s, void *ignored)
{
	struct adrv9104_rx *rx = s->private;
	struct adrv9104_rf_phy *phy = adrv9104_rx_to_phy(rx, rx->channel.idx);
	struct adi_adrv910x_GainControlCfg agc = {0};
	int ret;

	scoped_guard(mutex, &phy->lock) {
		if (!rx->channel.enabled)
			return -ENODEV;

		ret = adrv9104_api_call(phy, adi_adrv910x_Rx_GainControl_Inspect,
					rx->channel.number, &agc);
		if (ret)
			return ret;
	}

#define adrv9104_agc_seq_printf(member) \
	adrv9104_seq_printf(s, &agc, member)

	adrv9104_agc_seq_printf(peakWaitTime);
	adrv9104_agc_seq_printf(maxGainIndex);
	adrv9104_agc_seq_printf(minGainIndex);
	adrv9104_agc_seq_printf(gainUpdateCounter);
	adrv9104_agc_seq_printf(attackDelay_us);
	adrv9104_agc_seq_printf(slowLoopSettlingDelay);
	adrv9104_agc_seq_printf(lowThreshPreventGainInc);
	adrv9104_agc_seq_printf(changeGainIfThreshHigh);
	adrv9104_agc_seq_printf(agcMode);
	adrv9104_agc_seq_printf(resetOnRxon);
	adrv9104_agc_seq_printf(resetOnRxonGainIndex);
	adrv9104_agc_seq_printf(enableSyncPulseForGainCounter);
	adrv9104_agc_seq_printf(enableFastRecoveryLoop);
	/* power parameters */
	adrv9104_agc_seq_printf(power.powerEnableMeasurement);
	adrv9104_agc_seq_printf(power.underRangeHighPowerThresh);
	adrv9104_agc_seq_printf(power.underRangeLowPowerThresh);
	adrv9104_agc_seq_printf(power.underRangeHighPowerGainStepRecovery);
	adrv9104_agc_seq_printf(power.underRangeLowPowerGainStepRecovery);
	adrv9104_agc_seq_printf(power.powerMeasurementDuration);
	adrv9104_agc_seq_printf(power.powerMeasurementDelay);
	adrv9104_agc_seq_printf(power.rxTddPowerMeasDuration);
	adrv9104_agc_seq_printf(power.rxTddPowerMeasDelay);
	adrv9104_agc_seq_printf(power.overRangeHighPowerThresh);
	adrv9104_agc_seq_printf(power.overRangeLowPowerThresh);
	adrv9104_agc_seq_printf(power.overRangeHighPowerGainStepAttack);
	adrv9104_agc_seq_printf(power.overRangeLowPowerGainStepAttack);
	adrv9104_agc_seq_printf(power.feedback_inner_high_inner_low);
	adrv9104_agc_seq_printf(power.feedback_apd_high_apd_low);
	/* peak parameters */
	adrv9104_agc_seq_printf(peak.agcUnderRangeLowInterval);
	adrv9104_agc_seq_printf(peak.agcUnderRangeMidInterval);
	adrv9104_agc_seq_printf(peak.agcUnderRangeHighInterval);
	adrv9104_agc_seq_printf(peak.apdHighThresh);
	adrv9104_agc_seq_printf(peak.apdLowThresh);
	adrv9104_agc_seq_printf(peak.apdUpperThreshPeakExceededCount);
	adrv9104_agc_seq_printf(peak.apdLowerThreshPeakExceededCount);
	adrv9104_agc_seq_printf(peak.apdGainStepAttack);
	adrv9104_agc_seq_printf(peak.apdGainStepRecovery);
	adrv9104_agc_seq_printf(peak.enableHbOverload);
	adrv9104_agc_seq_printf(peak.hbOverloadDurationCount);
	adrv9104_agc_seq_printf(peak.hbOverloadThreshCount);
	adrv9104_agc_seq_printf(peak.hbHighThresh);
	adrv9104_agc_seq_printf(peak.hbUnderRangeLowThresh);
	adrv9104_agc_seq_printf(peak.hbUnderRangeMidThresh);
	adrv9104_agc_seq_printf(peak.hbUnderRangeHighThresh);
	adrv9104_agc_seq_printf(peak.hbUpperThreshPeakExceededCount);
	adrv9104_agc_seq_printf(peak.hbUnderRangeHighThreshExceededCount);
	adrv9104_agc_seq_printf(peak.hbGainStepHighRecovery);
	adrv9104_agc_seq_printf(peak.hbGainStepLowRecovery);
	adrv9104_agc_seq_printf(peak.hbGainStepMidRecovery);
	adrv9104_agc_seq_printf(peak.hbGainStepAttack);
	adrv9104_agc_seq_printf(peak.hbOverloadPowerMode);
	adrv9104_agc_seq_printf(peak.hbUnderRangeMidThreshExceededCount);
	adrv9104_agc_seq_printf(peak.hbUnderRangeLowThreshExceededCount);
	adrv9104_agc_seq_printf(peak.feedback_apd_low_hb_low);
	adrv9104_agc_seq_printf(peak.feedback_apd_high_hb_high);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adrv9104_rx_agc_config);

static int adrv9104_api_version_show(struct seq_file *s, void *ignored)
{
	struct adi_common_ApiVersion api_version;
	struct adrv9104_rf_phy *phy = s->private;
	int ret;

	ret = adrv9104_api_call(phy, adi_adrv910x_apiVersion_get, &api_version);
	if (ret)
		return ret;

	seq_printf(s, "%u.%u.%u\n", api_version.major, api_version.minor,
		   api_version.patch);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adrv9104_api_version);

static int adrv9104_stream_version_show(struct seq_file *s, void *ignored)
{
	struct adi_adrv910x_StreamVersion stream_version;
	struct adrv9104_rf_phy *phy = s->private;
	int ret;

	scoped_guard(mutex, &phy->lock) {
		ret = adrv9104_api_call(phy, adi_adrv910x_Stream_Version, &stream_version);
		if (ret)
			return ret;
	}

	seq_printf(s, "%u.%u.%u.%u\n", stream_version.majorVer, stream_version.minorVer,
		   stream_version.maintVer, stream_version.buildVer);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adrv9104_stream_version);

static ssize_t adrv9104_ssi_delays_write(struct file *file, const char __user *userbuf,
					 size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct adrv9104_rf_phy *phy = s->private;
	int ret;

	guard(mutex)(&phy->lock);
	ret = adrv9104_api_call(phy, adi_adrv910x_Ssi_Delay_Configure, phy->ssi_type,
				&phy->ssi_delays);
	if (ret)
		return ret;

	return count;
}

static int adrv9104_ssi_delays_show(struct seq_file *s, void *ignored)
{
	struct adi_adrv910x_SsiCalibrationCfg delays = {0};
	struct adrv9104_rf_phy *phy = s->private;
	int ret, rx;

	scoped_guard(mutex, &phy->lock) {
		ret = adrv9104_api_call(phy, adi_adrv910x_Ssi_Delay_Inspect, phy->ssi_type,
					&delays);
		if (ret)
			return ret;
	}

	for (rx = 0; rx < ARRAY_SIZE(phy->rx_channels); rx++) {
		seq_printf(s, "rx%d_ClkDelay: %u\n", rx, delays.rxClkDelay[rx]);
		seq_printf(s, "rx%d_StrobeDelay: %u\n", rx, delays.rxStrobeDelay[rx]);
		seq_printf(s, "rx%d_rxIDataDelay: %u\n", rx, delays.rxIDataDelay[rx]);
		seq_printf(s, "rx%d_rxQDataDelay: %u\n", rx, delays.rxQDataDelay[rx]);
	}

	seq_printf(s, "tx_ClkDelay: %u\n", delays.txClkDelay);
	seq_printf(s, "tx_RefClkDelay: %u\n", delays.txRefClkDelay);
	seq_printf(s, "tx_StrobeDelay: %u\n", delays.txStrobeDelay);
	seq_printf(s, "tx_rxIDataDelay: %u\n", delays.txIDataDelay);
	seq_printf(s, "tx_rxQDataDelay: %u\n", delays.txQDataDelay);

	return 0;
}
DEFINE_SHOW_STORE_ATTRIBUTE(adrv9104_ssi_delays);

void adrv9104_debugfs_create(struct adrv9104_rf_phy *phy, struct iio_dev *indio_dev)
{
	struct dentry *d = iio_get_debugfs_dentry(indio_dev);
	struct adrv9104_rx *rx;
	char attr[64];

	if (!d)
		return;

	debugfs_create_file_unsafe("initialize", 0600, d, phy, &adrv9104_init_fops);
	debugfs_create_file("api_version", 0400, d, phy, &adrv9104_api_version_fops);
	debugfs_create_file("stream_version", 0400, d, phy, &adrv9104_stream_version_fops);
	debugfs_create_file("ssi_delays", 0600, d, phy, &adrv9104_ssi_delays_fops);
	debugfs_create_bool("rerun_calls_on_lo_retune", 0600, d, &phy->rerun_calls);

	adrv9104_for_each_rx(phy, rx) {
		sprintf(attr, "rx%d_agc_config", rx->channel.idx);
		debugfs_create_file(attr, 0400, d, rx, &adrv9104_rx_agc_config_fops);
		sprintf(attr, "rx%d_carrier_hz", rx->channel.idx);
		debugfs_create_u64(attr, 0600, d, &rx->channel.carrier_hz);

		/* ssi delays */
		sprintf(attr, "rx%d_ssi_clk_delay", rx->channel.idx);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.rxClkDelay[rx->channel.idx]);
		sprintf(attr, "rx%d_ssi_strobe_delay", rx->channel.idx);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.rxStrobeDelay[rx->channel.idx]);
		sprintf(attr, "rx%d_ssi_i_data_delay", rx->channel.idx);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.rxIDataDelay[rx->channel.idx]);
		sprintf(attr, "rx%d_ssi_q_data_delay", rx->channel.idx);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.rxQDataDelay[rx->channel.idx]);

		iio_backend_debugfs_add(rx->channel.back, indio_dev);
	}

	if (!phy->tx_channel.channel.enabled)
		return;

	sprintf(attr, "tx%d_carrier_hz", phy->tx_channel.channel.idx);
	debugfs_create_u64(attr, 0600, d, &phy->tx_channel.channel.carrier_hz);

	/* ssi delays */
	debugfs_create_u8("tx_ssi_clk_delay", 0600, d, &phy->ssi_delays.txClkDelay);
	debugfs_create_u8("tx_ssi_refclk_delay", 0600, d, &phy->ssi_delays.txRefClkDelay);
	debugfs_create_u8("tx_ssi_strobe_delay", 0600, d, &phy->ssi_delays.txStrobeDelay);
	debugfs_create_u8("tx_ssi_i_data_delay", 0600, d, &phy->ssi_delays.txIDataDelay);
	debugfs_create_u8("tx_ssi_q_data_delay", 0600, d, &phy->ssi_delays.txQDataDelay);

	iio_backend_debugfs_add(phy->tx_channel.channel.back, indio_dev);
}
