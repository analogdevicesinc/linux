/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ADRV9002 debugfs interface
 *
 * Copyright 20202 Analog Devices Inc.
 */
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/string.h>

#include "adrv9002.h"
#include "adi_adrv9001_radio.h"
#include "adi_adrv9001_rx.h"
#include "adi_adrv9001_rxSettings_types.h"
#include "adi_adrv9001_rx_gaincontrol.h"
#include "adi_adrv9001_rx_gaincontrol_types.h"
#include "adi_adrv9001_ssi.h"
#include "adi_adrv9001_ssi_types.h"
#include "adi_adrv9001_tx.h"
#include "adi_adrv9001_tx_types.h"

#define rx_to_phy(rx, nr)	\
	container_of(rx, struct adrv9002_rf_phy, rx_channels[nr])

#define tx_to_phy(tx, nr)	\
	container_of(tx, struct adrv9002_rf_phy, tx_channels[nr])

#define chan_to_tx(c)		\
	container_of(c, struct adrv9002_tx_chan, channel)

#define chan_to_rx(c)		\
	container_of(c, struct adrv9002_rx_chan, channel)

#define chan_to_phy(c) ({						\
	struct adrv9002_chan *__c = (c);				\
	struct adrv9002_rf_phy *__phy;					\
									\
	if (__c->port == ADI_RX)					\
		__phy = rx_to_phy(chan_to_rx(__c), __c->idx);	\
	else								\
		__phy = tx_to_phy(chan_to_tx(__c), __c->idx);	\
									\
	__phy;								\
})

static ssize_t adrv9002_rx_adc_type_get(struct file *file, char __user *userbuf,
					size_t count, loff_t *ppos)
{
	struct adrv9002_rx_chan	*rx = file->private_data;
	struct adrv9002_rf_phy *phy = rx_to_phy(rx, rx->channel.idx);
	char buf[8];
	adi_adrv9001_AdcType_e adc_type;
	int ret, len;

	if (!rx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Rx_AdcType_Get(phy->adrv9001, rx->channel.number,
					  &adc_type);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	len = snprintf(buf, sizeof(buf), "%s\n",
		       adc_type == ADI_ADRV9001_ADC_HP ? "HP" : "LP");

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static const struct file_operations adrv9002_channel_adc_type_fops = {
	.open = simple_open,
	.read = adrv9002_rx_adc_type_get,
	.llseek = default_llseek,
};

#define adrv9002_seq_printf(seq, ptr, member) \
	seq_printf(seq, "%s: %u\n", #member, (ptr)->member)

static int adrv9002_rx_gain_control_pin_mode_show(struct seq_file *s,
						  void *ignored)
{
	struct adrv9002_rx_chan	*rx = s->private;
	struct adrv9002_rf_phy *phy = rx_to_phy(rx, rx->channel.idx);
	int ret;
	struct adi_adrv9001_RxGainControlPinCfg cfg = {0};

	if (!rx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Rx_GainControl_PinMode_Inspect(phy->adrv9001,
							  rx->channel.number,
							  &cfg);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	seq_printf(s, "min_gain_index: %u\n", cfg.minGainIndex);
	seq_printf(s, "max_gain_index: %u\n", cfg.maxGainIndex);
	seq_printf(s, "increment_step_size: %u\n", cfg.incrementStepSize);
	seq_printf(s, "decrement_step_size: %u\n", cfg.decrementStepSize);
	seq_printf(s, "increment_pin: dgpio%d\n", cfg.incrementPin - 1);
	seq_printf(s, "decrement_pin: dgpio%d\n", cfg.decrementPin - 1);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adrv9002_rx_gain_control_pin_mode);

static int adrv9002_rx_agc_config_show(struct seq_file *s, void *ignored)
{
	struct adrv9002_rx_chan	*rx = s->private;
	struct adrv9002_rf_phy *phy = rx_to_phy(rx, rx->channel.idx);
	struct adi_adrv9001_GainControlCfg agc = {0};
	int ret;

	if (!rx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Rx_GainControl_Inspect(phy->adrv9001,
						  rx->channel.number, &agc);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

#define adrv9002_agc_seq_printf(member) \
	adrv9002_seq_printf(s, &agc, member)

	adrv9002_agc_seq_printf(peakWaitTime);
	adrv9002_agc_seq_printf(maxGainIndex);
	adrv9002_agc_seq_printf(minGainIndex);
	adrv9002_agc_seq_printf(gainUpdateCounter);
	adrv9002_agc_seq_printf(attackDelay_us);
	adrv9002_agc_seq_printf(slowLoopSettlingDelay);
	adrv9002_agc_seq_printf(lowThreshPreventGainInc);
	adrv9002_agc_seq_printf(changeGainIfThreshHigh);
	adrv9002_agc_seq_printf(agcMode);
	adrv9002_agc_seq_printf(resetOnRxon);
	adrv9002_agc_seq_printf(resetOnRxonGainIndex);
	adrv9002_agc_seq_printf(enableSyncPulseForGainCounter);
	adrv9002_agc_seq_printf(enableFastRecoveryLoop);
	/* power parameters */
	adrv9002_agc_seq_printf(power.powerEnableMeasurement);
	adrv9002_agc_seq_printf(power.underRangeHighPowerThresh);
	adrv9002_agc_seq_printf(power.underRangeLowPowerThresh);
	adrv9002_agc_seq_printf(power.underRangeHighPowerGainStepRecovery);
	adrv9002_agc_seq_printf(power.underRangeLowPowerGainStepRecovery);
	adrv9002_agc_seq_printf(power.powerMeasurementDuration);
	adrv9002_agc_seq_printf(power.powerMeasurementDelay);
	adrv9002_agc_seq_printf(power.rxTddPowerMeasDuration);
	adrv9002_agc_seq_printf(power.rxTddPowerMeasDelay);
	adrv9002_agc_seq_printf(power.overRangeHighPowerThresh);
	adrv9002_agc_seq_printf(power.overRangeLowPowerThresh);
	adrv9002_agc_seq_printf(power.overRangeHighPowerGainStepAttack);
	adrv9002_agc_seq_printf(power.overRangeLowPowerGainStepAttack);
	adrv9002_agc_seq_printf(power.feedback_inner_high_inner_low);
	adrv9002_agc_seq_printf(power.feedback_apd_high_apd_low);
	/* peak parameters */
	adrv9002_agc_seq_printf(peak.agcUnderRangeLowInterval);
	adrv9002_agc_seq_printf(peak.agcUnderRangeMidInterval);
	adrv9002_agc_seq_printf(peak.agcUnderRangeHighInterval);
	adrv9002_agc_seq_printf(peak.apdHighThresh);
	adrv9002_agc_seq_printf(peak.apdLowThresh);
	adrv9002_agc_seq_printf(peak.apdUpperThreshPeakExceededCount);
	adrv9002_agc_seq_printf(peak.apdLowerThreshPeakExceededCount);
	adrv9002_agc_seq_printf(peak.apdGainStepAttack);
	adrv9002_agc_seq_printf(peak.apdGainStepRecovery);
	adrv9002_agc_seq_printf(peak.enableHbOverload);
	adrv9002_agc_seq_printf(peak.hbOverloadDurationCount);
	adrv9002_agc_seq_printf(peak.hbOverloadThreshCount);
	adrv9002_agc_seq_printf(peak.hbHighThresh);
	adrv9002_agc_seq_printf(peak.hbUnderRangeLowThresh);
	adrv9002_agc_seq_printf(peak.hbUnderRangeMidThresh);
	adrv9002_agc_seq_printf(peak.hbUnderRangeHighThresh);
	adrv9002_agc_seq_printf(peak.hbUpperThreshPeakExceededCount);
	adrv9002_agc_seq_printf(peak.hbUnderRangeHighThreshExceededCount);
	adrv9002_agc_seq_printf(peak.hbGainStepHighRecovery);
	adrv9002_agc_seq_printf(peak.hbGainStepLowRecovery);
	adrv9002_agc_seq_printf(peak.hbGainStepMidRecovery);
	adrv9002_agc_seq_printf(peak.hbGainStepMidRecovery);
	adrv9002_agc_seq_printf(peak.hbOverloadPowerMode);
	adrv9002_agc_seq_printf(peak.hbUnderRangeMidThreshExceededCount);
	adrv9002_agc_seq_printf(peak.hbUnderRangeLowThreshExceededCount);
	adrv9002_agc_seq_printf(peak.feedback_apd_low_hb_low);
	adrv9002_agc_seq_printf(peak.feedback_apd_high_hb_high);

	return 0;
}

static ssize_t adrv9002_rx_agc_config_write(struct file *file, const char __user *userbuf,
					    size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct adrv9002_rx_chan	*rx = s->private;
	struct adrv9002_rf_phy *phy = rx_to_phy(rx, rx->channel.idx);
	int ret;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Rx_GainControl_Configure(phy->adrv9001, rx->channel.number,
						    &rx->agc);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	return count;
}

static int adrv9002_rx_agc_config_open(struct inode *inode, struct file *file)
{
	return single_open(file, adrv9002_rx_agc_config_show, inode->i_private);
}

static const struct file_operations adrv9002_rx_agc_config_fops = {
	.owner		= THIS_MODULE,
	.open		= adrv9002_rx_agc_config_open,
	.read		= seq_read,
	.write		= adrv9002_rx_agc_config_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void adrv9002_debugfs_agc_config_create(struct adrv9002_rx_chan *rx, struct dentry *d)
{
#define adrv9002_agc_get_attr(nr, member) ((nr) == ADI_CHANNEL_1 ? \
					"rx0_agc_" #member : "rx1_agc_" #member)

#define adrv9002_agc_add_file_u8(member) { \
	const char *attr = adrv9002_agc_get_attr(rx->channel.number, member); \
	debugfs_create_u8(attr, 0600, d, (u8 *)&rx->agc.member); \
}

#define adrv9002_agc_add_file_u16(member) { \
	const char *attr = adrv9002_agc_get_attr(rx->channel.number, member); \
	debugfs_create_u16(attr, 0600, d, &rx->agc.member); \
}

#define adrv9002_agc_add_file_u32(member) { \
	const char *attr = adrv9002_agc_get_attr(rx->channel.number, member); \
	debugfs_create_u32(attr, 0600, d, &rx->agc.member); \
}

#define adrv9002_agc_add_file_bool(member) { \
	const char *attr = adrv9002_agc_get_attr(rx->channel.number, member); \
	debugfs_create_bool(attr, 0600, d, &rx->agc.member); \
}
	adrv9002_agc_add_file_u8(peakWaitTime);
	adrv9002_agc_add_file_u8(maxGainIndex);
	adrv9002_agc_add_file_u8(minGainIndex);
	adrv9002_agc_add_file_u32(gainUpdateCounter);
	adrv9002_agc_add_file_u8(attackDelay_us);
	adrv9002_agc_add_file_u8(slowLoopSettlingDelay);
	adrv9002_agc_add_file_bool(lowThreshPreventGainInc);
	adrv9002_agc_add_file_u8(changeGainIfThreshHigh);
	adrv9002_agc_add_file_u8(agcMode);
	adrv9002_agc_add_file_bool(resetOnRxon);
	adrv9002_agc_add_file_u8(resetOnRxonGainIndex);
	adrv9002_agc_add_file_bool(enableSyncPulseForGainCounter);
	adrv9002_agc_add_file_bool(enableFastRecoveryLoop);
	/* power parameters */
	adrv9002_agc_add_file_bool(power.powerEnableMeasurement);
	adrv9002_agc_add_file_u8(power.underRangeHighPowerThresh);
	adrv9002_agc_add_file_u8(power.underRangeLowPowerThresh);
	adrv9002_agc_add_file_u8(power.underRangeHighPowerGainStepRecovery);
	adrv9002_agc_add_file_u8(power.underRangeLowPowerGainStepRecovery);
	adrv9002_agc_add_file_u8(power.powerMeasurementDuration);
	adrv9002_agc_add_file_u8(power.powerMeasurementDelay);
	adrv9002_agc_add_file_u16(power.rxTddPowerMeasDuration);
	adrv9002_agc_add_file_u16(power.rxTddPowerMeasDelay);
	adrv9002_agc_add_file_u8(power.overRangeHighPowerThresh);
	adrv9002_agc_add_file_u8(power.overRangeLowPowerThresh);
	adrv9002_agc_add_file_u8(power.overRangeHighPowerGainStepAttack);
	adrv9002_agc_add_file_u8(power.overRangeLowPowerGainStepAttack);
	adrv9002_agc_add_file_u8(power.feedback_inner_high_inner_low);
	adrv9002_agc_add_file_u8(power.feedback_apd_high_apd_low);
	/* peak parameters */
	adrv9002_agc_add_file_u16(peak.agcUnderRangeLowInterval);
	adrv9002_agc_add_file_u8(peak.agcUnderRangeMidInterval);
	adrv9002_agc_add_file_u8(peak.agcUnderRangeHighInterval);
	adrv9002_agc_add_file_u8(peak.apdHighThresh);
	adrv9002_agc_add_file_u8(peak.apdLowThresh);
	adrv9002_agc_add_file_u8(peak.apdUpperThreshPeakExceededCount);
	adrv9002_agc_add_file_u8(peak.apdLowerThreshPeakExceededCount);
	adrv9002_agc_add_file_u8(peak.apdGainStepAttack);
	adrv9002_agc_add_file_u8(peak.apdGainStepRecovery);
	adrv9002_agc_add_file_bool(peak.enableHbOverload);
	adrv9002_agc_add_file_u8(peak.hbOverloadDurationCount);
	adrv9002_agc_add_file_u8(peak.hbOverloadThreshCount);
	adrv9002_agc_add_file_u16(peak.hbHighThresh);
	adrv9002_agc_add_file_u16(peak.hbUnderRangeLowThresh);
	adrv9002_agc_add_file_u16(peak.hbUnderRangeMidThresh);
	adrv9002_agc_add_file_u16(peak.hbUnderRangeHighThresh);
	adrv9002_agc_add_file_u8(peak.hbUpperThreshPeakExceededCount);
	adrv9002_agc_add_file_u8(peak.hbUnderRangeHighThreshExceededCount);
	adrv9002_agc_add_file_u8(peak.hbGainStepHighRecovery);
	adrv9002_agc_add_file_u8(peak.hbGainStepLowRecovery);
	adrv9002_agc_add_file_u8(peak.hbGainStepMidRecovery);
	adrv9002_agc_add_file_u8(peak.hbGainStepMidRecovery);
	adrv9002_agc_add_file_u8(peak.hbOverloadPowerMode);
	adrv9002_agc_add_file_u8(peak.hbUnderRangeMidThreshExceededCount);
	adrv9002_agc_add_file_u8(peak.hbUnderRangeLowThreshExceededCount);
	adrv9002_agc_add_file_u8(peak.feedback_apd_low_hb_low);
	adrv9002_agc_add_file_u8(peak.feedback_apd_high_hb_high);
}

static int adrv9002_tx_dac_full_scale_get(void *arg, u64 *val)
{
	struct adrv9002_tx_chan	*tx = arg;
	struct adrv9002_rf_phy *phy = tx_to_phy(tx, tx->channel.idx);
	int ret;
	bool enable;

	if (!tx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Tx_OutputPowerBoost_Get(phy->adrv9001,
						   tx->channel.number, &enable);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	*val = enable;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_tx_dac_full_scale_fops,
			 adrv9002_tx_dac_full_scale_get,
			 NULL, "%lld\n");

static int adrv9002_tx_pin_atten_control_show(struct seq_file *s, void *ignored)
{
	struct adrv9002_tx_chan	*tx = s->private;
	struct adrv9002_rf_phy *phy = tx_to_phy(tx, tx->channel.idx);
	struct adi_adrv9001_TxAttenuationPinControlCfg cfg = {0};
	int ret;

	if (!tx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Tx_Attenuation_PinControl_Inspect(phy->adrv9001,
							     tx->channel.number,
							     &cfg);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	seq_printf(s, "step_size_mdB: %u\n", cfg.stepSize_mdB);
	seq_printf(s, "increment_pin: dgpio%d\n", cfg.incrementPin - 1);
	seq_printf(s, "decrement_pin: dgpio%d\n", cfg.decrementPin - 1);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adrv9002_tx_pin_atten_control);

static int adrv9002_pll_status_show(struct seq_file *s, void *ignored)
{
	struct adrv9002_rf_phy *phy = s->private;
	int ret;
	bool lo1, lo2, aux, clk, clk_lp;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Radio_PllStatus_Get(phy->adrv9001,
					       ADI_ADRV9001_PLL_LO1, &lo1);

	ret = adi_adrv9001_Radio_PllStatus_Get(phy->adrv9001,
	                                       ADI_ADRV9001_PLL_LO2, &lo2);
	if (ret)
		goto error;

	ret = adi_adrv9001_Radio_PllStatus_Get(phy->adrv9001,
	                                       ADI_ADRV9001_PLL_AUX, &aux);
	if (ret)
		goto error;

	ret = adi_adrv9001_Radio_PllStatus_Get(phy->adrv9001,
	                                       ADI_ADRV9001_PLL_CLK, &clk);
	if (ret)
		goto error;

	ret = adi_adrv9001_Radio_PllStatus_Get(phy->adrv9001,
	                                       ADI_ADRV9001_PLL_CLK_LP,
	                                       &clk_lp);
	if (ret)
		goto error;
	mutex_unlock(&phy->lock);

	seq_printf(s, "Clock: %s\n", clk ? "Locked" : "Unlocked");
	seq_printf(s, "Clock LP: %s\n", clk_lp ? "Locked" : "Unlocked");
	seq_printf(s, "LO1: %s\n", lo1 ? "Locked" : "Unlocked");
	seq_printf(s, "LO2: %s\n", lo2 ? "Locked" : "Unlocked");
	seq_printf(s, "AUX: %s\n", aux ? "Locked" : "Unlocked");

	return 0;
error:
	mutex_unlock(&phy->lock);
	return adrv9002_dev_err(phy);
}
DEFINE_SHOW_ATTRIBUTE(adrv9002_pll_status);

static const char *const adrv9002_ssi_test_mode_data_avail[] = {
	"TESTMODE_DATA_NORMAL",
	"TESTMODE_DATA_FIXED_PATTERN",
	"TESTMODE_DATA_RAMP_NIBBLE",
	"TESTMODE_DATA_RAMP_16_BIT",
	"TESTMODE_DATA_PRBS15",
	"TESTMODE_DATA_PRBS7",
};

#define ADRV9002_RX_SSI_TEST_DATA_MASK_SIZE	6
#define ADRV9002_RX_SSI_TEST_DATA_LVDS_MASK	0x3b
#define ADRV9002_RX_SSI_TEST_DATA_CMOS_MASK	GENMASK(3, 0)
#define ADRV9002_TX_SSI_TEST_DATA_LVDS_MASK	0x33
#define ADRV9002_TX_SSI_TEST_DATA_CMOS_MASK	GENMASK(3, 0)

static unsigned long rx_ssi_avail_mask;
static unsigned long tx_ssi_avail_mask;

static ssize_t adrv9002_ssi_test_mode_data_show(char __user *userbuf,
						size_t count, loff_t *ppos,
						const char *item)
{
	char buf[32];
	int len;

	len = scnprintf(buf, sizeof(buf), "%s\n", item);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static int adrv9002_ssi_test_mode_data_set(const char __user *userbuf,
					   size_t count, loff_t *ppos,
					   const unsigned long mask)
{
	char buf[32] = {0};
	int bit = 0, ret;

	ret = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, userbuf,
				     count);
	if (ret < 0)
		return ret;

	for_each_set_bit(bit, &mask, ADRV9002_RX_SSI_TEST_DATA_MASK_SIZE) {
		if (sysfs_streq(buf, adrv9002_ssi_test_mode_data_avail[bit]))
			break;
	}

	if (bit == ADRV9002_RX_SSI_TEST_DATA_MASK_SIZE)
		return -EINVAL;

	return bit;
}

static int adrv9002_ssi_mode_avail_show(struct seq_file *s, void *ignored)
{
	int bit = 0;
	const unsigned long *mask = s->private;

	for_each_set_bit(bit, mask, ADRV9002_RX_SSI_TEST_DATA_MASK_SIZE) {
		seq_printf(s, "%s\n", adrv9002_ssi_test_mode_data_avail[bit]);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adrv9002_ssi_mode_avail);

static ssize_t adrv9002_rx_ssi_test_mode_data_show(struct file *file,
						   char __user *userbuf,
						   size_t count, loff_t *ppos)
{
	struct adrv9002_rx_chan	*rx = file->private_data;
	int idx = rx->ssi_test.testData;
	const char *data = adrv9002_ssi_test_mode_data_avail[idx];

	return adrv9002_ssi_test_mode_data_show(userbuf, count, ppos, data);
}

static ssize_t adrv9002_rx_ssi_test_mode_data_set(struct file *file,
						  const char __user *userbuf,
						  size_t count, loff_t *ppos)
{
	struct adrv9002_rx_chan	*rx = file->private_data;
	int ret;

	ret = adrv9002_ssi_test_mode_data_set(userbuf, count, ppos, rx_ssi_avail_mask);
	if (ret < 0)
		return ret;

	rx->ssi_test.testData = ret;

	return count;
}

static const struct file_operations adrv9002_rx_ssi_test_mode_data_fops = {
	.open = simple_open,
	.read = adrv9002_rx_ssi_test_mode_data_show,
	.write = adrv9002_rx_ssi_test_mode_data_set,
	.llseek = default_llseek,
};

static int adrv9002_rx_ssi_test_mode_fixed_pattern_get(void *arg, u64 *val)
{
	struct adrv9002_rx_chan	*rx = arg;

	*val = rx->ssi_test.fixedDataPatternToTransmit;

	return 0;
};

static int adrv9002_rx_ssi_test_mode_fixed_pattern_set(void *arg, const u64 val)
{
	struct adrv9002_rx_chan	*rx = arg;
	struct adrv9002_rf_phy *phy = rx_to_phy(rx, rx->channel.idx);
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);
	int val_max = ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS ? 0xf : U16_MAX;
	int __val;

	__val = clamp_val(val, 0, val_max);
	rx->ssi_test.fixedDataPatternToTransmit = __val;

	return 0;
};

DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_rx_ssi_test_mode_fixed_pattern_fops,
			 adrv9002_rx_ssi_test_mode_fixed_pattern_get,
			 adrv9002_rx_ssi_test_mode_fixed_pattern_set,
			 "%llu\n");

static int adrv9002_ssi_rx_test_mode_set(void *arg, const u64 val)
{
	struct adrv9002_rx_chan	*rx = arg;
	struct adrv9002_rf_phy *phy = rx_to_phy(rx, rx->channel.idx);
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);
	int ret;

	if (!rx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Ssi_Rx_TestMode_Configure(phy->adrv9001, rx->channel.number, ssi_type,
						     ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						     &rx->ssi_test);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
};
DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_ssi_rx_test_mode_config_fops,
			 NULL, adrv9002_ssi_rx_test_mode_set, "%llu");

static ssize_t adrv9002_tx_ssi_test_mode_data_show(struct file *file, char __user *userbuf,
						   size_t count, loff_t *ppos)
{
	struct adrv9002_tx_chan	*tx = file->private_data;
	int idx = tx->ssi_test.testData;
	const char *data = adrv9002_ssi_test_mode_data_avail[idx];

	return adrv9002_ssi_test_mode_data_show(userbuf, count, ppos, data);
}

static ssize_t adrv9002_tx_ssi_test_mode_data_set(struct file *file, const char __user *userbuf,
						  size_t count, loff_t *ppos)
{
	struct adrv9002_tx_chan	*tx = file->private_data;
	int ret;

	ret = adrv9002_ssi_test_mode_data_set(userbuf, count, ppos, tx_ssi_avail_mask);
	if (ret < 0)
		return ret;

	tx->ssi_test.testData = ret;

	return count;
}

static const struct file_operations adrv9002_tx_ssi_test_mode_data_fops = {
	.open = simple_open,
	.read = adrv9002_tx_ssi_test_mode_data_show,
	.write = adrv9002_tx_ssi_test_mode_data_set,
	.llseek = default_llseek,
};

static int adrv9002_tx_ssi_test_mode_fixed_pattern_get(void *arg, u64 *val)
{
	struct adrv9002_tx_chan	*tx = arg;

	*val = tx->ssi_test.fixedDataPatternToCheck;

	return 0;
};

static int adrv9002_tx_ssi_test_mode_fixed_pattern_set(void *arg, const u64 val)
{
	struct adrv9002_tx_chan	*tx = arg;
	u32 __val;

	__val = clamp_val(val, 0, U16_MAX);
	tx->ssi_test.fixedDataPatternToCheck = __val;

	return 0;
};
DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_tx_ssi_test_mode_fixed_pattern_fops,
			 adrv9002_tx_ssi_test_mode_fixed_pattern_get,
			 adrv9002_tx_ssi_test_mode_fixed_pattern_set,
			 "%llu\n");

static int adrv9002_init_set(void *arg, const u64 val)
{
	struct adrv9002_rf_phy *phy = arg;

	if (!val)
		return -EINVAL;

	return adrv9002_clean_setup(phy);
}
DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_init_fops,
			 NULL, adrv9002_init_set, "%llu");

static int adrv9002_tx_ssi_test_mode_loopback_get(void *arg, u64 *val)
{
	struct adrv9002_tx_chan	*tx = arg;

	*val = tx->loopback;

	return 0;
};

static int adrv9002_tx_ssi_test_mode_loopback_set(void *arg, const u64 val)
{
	struct adrv9002_tx_chan	*tx = arg;
	struct adrv9002_rf_phy *phy = tx_to_phy(tx, tx->channel.idx);
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);
	bool enable = !!val;
	int ret;

	if (enable == tx->loopback)
		return 0;

	tx->loopback = enable;
	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Ssi_Loopback_Set(phy->adrv9001, tx->channel.number, ssi_type, enable);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
};

DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_tx_ssi_test_mode_loopback_fops,
			 adrv9002_tx_ssi_test_mode_loopback_get,
			 adrv9002_tx_ssi_test_mode_loopback_set,
			 "%llu\n");

static int adrv9002_ssi_tx_test_mode_set(void *arg, const u64 val)
{
	struct adrv9002_tx_chan	*tx = arg;
	struct adrv9002_rf_phy *phy = tx_to_phy(tx, tx->channel.idx);
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);
	int ret;

	if (!tx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adrv9002_axi_tx_test_pattern_cfg(phy, tx->channel.idx, tx->ssi_test.testData);
	if (ret)
		goto unlock;

	ret = adi_adrv9001_Ssi_Tx_TestMode_Configure(phy->adrv9001, tx->channel.number,
						     ssi_type,
						     ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
						     &tx->ssi_test);
	if (ret)
		ret = adrv9002_dev_err(phy);

unlock:
	mutex_unlock(&phy->lock);
	return ret;
};
DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_ssi_tx_test_mode_config_fops,
			 NULL, adrv9002_ssi_tx_test_mode_set, "%llu");

static int adrv9002_ssi_tx_test_mode_status_show(struct seq_file *s,
						 void *ignored)
{
	struct adrv9002_tx_chan	*tx = s->private;
	struct adrv9002_rf_phy *phy = tx_to_phy(tx, tx->channel.idx);
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);
	adi_adrv9001_TxSsiTestModeStatus_t ssi_status = {0};
	int ret;

	if (!tx->channel.enabled)
		return -ENODEV;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Ssi_Tx_TestMode_Status_Inspect(phy->adrv9001,
							  tx->channel.number,
							  ssi_type,
							  ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA,
							  &tx->ssi_test,
							  &ssi_status);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	seq_printf(s, "dataError: %u\n", ssi_status.dataError);
	seq_printf(s, "fifoFull: %u\n", ssi_status.fifoFull);
	seq_printf(s, "fifoEmpty: %u\n", ssi_status.fifoEmpty);
	seq_printf(s, "strobeAlignError: %u\n", ssi_status.strobeAlignError);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adrv9002_ssi_tx_test_mode_status);

static int adrv9002_ssi_delays_show(struct seq_file *s, void *ignored)
{
	struct adrv9002_rf_phy *phy = s->private;
	int ret, i;
	struct adi_adrv9001_SsiCalibrationCfg delays = {0};
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Ssi_Delay_Inspect(phy->adrv9001, ssi_type, &delays);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	for (i = 0; i < ADRV9002_CHANN_MAX; i++) {
		seq_printf(s, "rx%d_ClkDelay: %u\n", i, delays.rxClkDelay[i]);
		seq_printf(s, "rx%d_StrobeDelay: %u\n", i, delays.rxStrobeDelay[i]);
		seq_printf(s, "rx%d_rxIDataDelay: %u\n", i, delays.rxIDataDelay[i]);
		seq_printf(s, "rx%d_rxQDataDelay: %u\n", i, delays.rxQDataDelay[i]);
		seq_printf(s, "tx%d_ClkDelay: %u\n", i, delays.txClkDelay[i]);
		seq_printf(s, "tx%d_RefClkDelay: %u\n", i, delays.txRefClkDelay[i]);
		seq_printf(s, "tx%d_StrobeDelay: %u\n", i, delays.txStrobeDelay[i]);
		seq_printf(s, "tx%d_rxIDataDelay: %u\n", i, delays.txIDataDelay[i]);
		seq_printf(s, "tx%d_rxQDataDelay: %u\n", i, delays.txQDataDelay[i]);
	}

	return 0;
}

static ssize_t adrv9002_ssi_delays_write(struct file *file, const char __user *userbuf,
					 size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct adrv9002_rf_phy *phy = s->private;
	int ret;
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Ssi_Delay_Configure(phy->adrv9001, ssi_type, &phy->ssi_delays);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	return count;
}

static int adrv9002_ssi_delays_open(struct inode *inode, struct file *file)
{
	return single_open(file, adrv9002_ssi_delays_show, inode->i_private);
}

static const struct file_operations adrv9002_ssi_delays_fops = {
	.owner		= THIS_MODULE,
	.open		= adrv9002_ssi_delays_open,
	.read		= seq_read,
	.write		= adrv9002_ssi_delays_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int adrv9002_enablement_delays_show(struct seq_file *s, void *ignored)
{
	struct adrv9002_chan *chan = s->private;
	struct adrv9002_rf_phy *phy = chan_to_phy(chan);
	struct adi_adrv9001_ChannelEnablementDelays en_delays = {0}, en_delays_ns;
	int ret;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Radio_ChannelEnablementDelays_Inspect(phy->adrv9001, chan->port,
								 chan->number, &en_delays);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	adrv9002_en_delays_arm_to_ns(phy, &en_delays, &en_delays_ns);

	seq_printf(s, "fall_to_off_delay: %u\n", en_delays_ns.fallToOffDelay);
	seq_printf(s, "guard_delay: %u\n", en_delays_ns.guardDelay);
	seq_printf(s, "hold_delay: %u\n", en_delays_ns.holdDelay);
	seq_printf(s, "rise_to_analog_on_delay: %u\n", en_delays_ns.riseToAnalogOnDelay);
	seq_printf(s, "rise_to_on_delay: %u\n", en_delays_ns.riseToOnDelay);

	return 0;
}

static ssize_t adrv9002_enablement_delays_write(struct file *file, const char __user *userbuf,
						size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct adrv9002_chan *chan = s->private;
	struct adrv9002_rf_phy *phy = chan_to_phy(chan);
	struct adi_adrv9001_ChannelEnablementDelays en_delays;
	int ret;

	adrv9002_en_delays_ns_to_arm(phy, &chan->en_delays_ns, &en_delays);

	mutex_lock(&phy->lock);
	ret = adrv9002_channel_to_state(phy, chan, ADI_ADRV9001_CHANNEL_CALIBRATED, true);
	if (ret)
		goto unlock;

	ret = adi_adrv9001_Radio_ChannelEnablementDelays_Configure(phy->adrv9001, chan->port,
								   chan->number, &en_delays);
	if (ret) {
		ret = adrv9002_dev_err(phy);
		goto unlock;
	}

	ret = adrv9002_channel_to_state(phy, chan, chan->cached_state, false);
unlock:
	mutex_unlock(&phy->lock);

	return ret ? ret : count;
}

static int adrv9002_enablement_delays_open(struct inode *inode, struct file *file)
{
	return single_open(file, adrv9002_enablement_delays_show, inode->i_private);
}

static const struct file_operations adrv9002_enablement_delays_fops = {
	.owner		= THIS_MODULE,
	.open		= adrv9002_enablement_delays_open,
	.read		= seq_read,
	.write		= adrv9002_enablement_delays_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void adrv9002_debugfs_create(struct adrv9002_rf_phy *phy, struct dentry *d)
{
	int chan;
	char attr[64];
	adi_adrv9001_SsiType_e ssi_type = adrv9002_axi_ssi_type_get(phy);

	if (!d)
		return;

	if (ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS) {
		rx_ssi_avail_mask = ADRV9002_RX_SSI_TEST_DATA_CMOS_MASK;
		tx_ssi_avail_mask = ADRV9002_TX_SSI_TEST_DATA_CMOS_MASK;
	} else {
		rx_ssi_avail_mask = ADRV9002_RX_SSI_TEST_DATA_LVDS_MASK;
		tx_ssi_avail_mask = ADRV9002_TX_SSI_TEST_DATA_LVDS_MASK;
	}

	debugfs_create_file_unsafe("initialize", 0600, d, phy,
				   &adrv9002_init_fops);

	debugfs_create_file("pll_status", 0400, d, phy,
			    &adrv9002_pll_status_fops);

	debugfs_create_file("rx_ssi_test_mode_data_available", 0400, d,
			    &rx_ssi_avail_mask, &adrv9002_ssi_mode_avail_fops);

	debugfs_create_file("tx_ssi_test_mode_data_available", 0400, d,
			    &tx_ssi_avail_mask, &adrv9002_ssi_mode_avail_fops);

	debugfs_create_file("ssi_delays", 0600, d, phy, &adrv9002_ssi_delays_fops);

	for (chan = 0; chan < ARRAY_SIZE(phy->tx_channels); chan++) {
		struct adrv9002_tx_chan *tx = &phy->tx_channels[chan];

		sprintf(attr, "tx%d_attenuation_pin_control", chan);
		debugfs_create_file(attr, 0400, d, tx, &adrv9002_tx_pin_atten_control_fops);
		sprintf(attr, "tx%d_dac_boost_en", chan);
		debugfs_create_file_unsafe(attr, 0400, d, tx, &adrv9002_tx_dac_full_scale_fops);
		sprintf(attr, "tx%d_ssi_test_mode_data", chan);
		debugfs_create_file(attr, 0600, d, &phy->tx_channels[chan],
				    &adrv9002_tx_ssi_test_mode_data_fops);
		sprintf(attr, "tx%d_ssi_test_mode_fixed_pattern", chan);
		debugfs_create_file_unsafe(attr, 0600, d, tx,
					   &adrv9002_tx_ssi_test_mode_fixed_pattern_fops);
		sprintf(attr, "tx%d_ssi_test_mode_configure", chan);
		debugfs_create_file(attr, 0200, d, tx, &adrv9002_ssi_tx_test_mode_config_fops);
		sprintf(attr, "tx%d_ssi_test_mode_status", chan);
		debugfs_create_file(attr, 0400, d, tx, &adrv9002_ssi_tx_test_mode_status_fops);
		sprintf(attr, "tx%d_ssi_test_mode_loopback_en", chan);
		debugfs_create_file_unsafe(attr, 0600, d, tx,
					   &adrv9002_tx_ssi_test_mode_loopback_fops);
		/* ssi delays */
		sprintf(attr, "tx%d_ssi_clk_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.txClkDelay[chan]);
		sprintf(attr, "tx%d_ssi_refclk_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.txRefClkDelay[chan]);
		sprintf(attr, "tx%d_ssi_strobe_delay_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.txStrobeDelay[chan]);
		sprintf(attr, "tx%d_ssi_i_data_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.txIDataDelay[chan]);
		sprintf(attr, "tx%d_ssi_q_data_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.txQDataDelay[chan]);
		/* enablement delays */
		sprintf(attr, "tx%d_fall_to_off_delay_ns", chan);
		debugfs_create_u32(attr, 0600, d, &tx->channel.en_delays_ns.fallToOffDelay);
		sprintf(attr, "tx%d_guard_delay_ns", chan);
		debugfs_create_u32(attr, 0600, d, &tx->channel.en_delays_ns.guardDelay);
		sprintf(attr, "tx%d_hold_delay_ns", chan);
		debugfs_create_u32(attr, 0600, d, &tx->channel.en_delays_ns.holdDelay);
		sprintf(attr, "tx%d_rise_to_analog_delay_ns", chan);
		debugfs_create_u32(attr, 0600, d, &tx->channel.en_delays_ns.riseToAnalogOnDelay);
		sprintf(attr, "tx%d_rise_to_on_delay_ns", chan);
		debugfs_create_u32(attr, 0600, d, &tx->channel.en_delays_ns.riseToOnDelay);
		sprintf(attr, "tx%d_enablement_delays", chan);
		debugfs_create_file(attr, 0600, d, &tx->channel, &adrv9002_enablement_delays_fops);
	}

	for (chan = 0; chan < ARRAY_SIZE(phy->rx_channels); chan++) {
		struct adrv9002_rx_chan *rx = &phy->rx_channels[chan];

		sprintf(attr, "rx%d_adc_type", chan);
		debugfs_create_file(attr, 0400, d, rx, &adrv9002_channel_adc_type_fops);
		sprintf(attr, "rx%d_gain_control_pin_mode", chan);
		debugfs_create_file(attr, 0400, d, rx, &adrv9002_rx_gain_control_pin_mode_fops);
		sprintf(attr, "rx%d_agc_config", chan);
		debugfs_create_file(attr, 0600, d, rx, &adrv9002_rx_agc_config_fops);
		sprintf(attr, "rx%d_ssi_test_mode_data", chan);
		debugfs_create_file(attr, 0600, d, rx, &adrv9002_rx_ssi_test_mode_data_fops);
		sprintf(attr, "rx%d_ssi_test_mode_fixed_pattern", chan);
		debugfs_create_file_unsafe(attr, 0600, d, rx,
					   &adrv9002_rx_ssi_test_mode_fixed_pattern_fops);
		sprintf(attr, "rx%d_ssi_test_mode_configure", chan);
		debugfs_create_file(attr, 0200, d, rx, &adrv9002_ssi_rx_test_mode_config_fops);
		adrv9002_debugfs_agc_config_create(rx, d);
		/* ssi delays */
		sprintf(attr, "rx%d_ssi_clk_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.rxClkDelay[chan]);
		sprintf(attr, "rx%d_ssi_strobe_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.rxStrobeDelay[chan]);
		sprintf(attr, "rx%d_ssi_i_data_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.rxIDataDelay[chan]);
		sprintf(attr, "rx%d_ssi_q_data_delay", chan);
		debugfs_create_u8(attr, 0600, d, &phy->ssi_delays.rxQDataDelay[chan]);
		/* enablement delays */
		sprintf(attr, "rx%d_fall_to_off_delay_ns", chan);
		debugfs_create_u32(attr, 0600, d, &rx->channel.en_delays_ns.fallToOffDelay);
		sprintf(attr, "rx%d_guard_delay_ns", chan);
		debugfs_create_u32(attr, 0600, d, &rx->channel.en_delays_ns.guardDelay);
		sprintf(attr, "rx%d_hold_delay_ns", chan);
		debugfs_create_u32(attr, 0600, d, &rx->channel.en_delays_ns.holdDelay);
		sprintf(attr, "rx%d_rise_to_analog_delay_ns", chan);
		debugfs_create_u32(attr, 0600, d, &rx->channel.en_delays_ns.riseToAnalogOnDelay);
		sprintf(attr, "rx%d_rise_to_on_delay_ns", chan);
		debugfs_create_u32(attr, 0600, d, &rx->channel.en_delays_ns.riseToOnDelay);
		sprintf(attr, "rx%d_enablement_delays", chan);
		debugfs_create_file(attr, 0600, d, &rx->channel, &adrv9002_enablement_delays_fops);
	}
}