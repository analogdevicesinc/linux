// SPDX-License-Identifier: GPL-2.0
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
#include <linux/stringify.h>
#include "linux/spi/spi.h"

#include "adrv9002.h"
#include "adi_adrv9001_fh.h"
#include "adi_adrv9001_fh_types.h"
#include "adi_adrv9001_gpio.h"
#include "adi_adrv9001_gpio_types.h"
#include "adi_adrv9001_radio.h"
#include "adi_adrv9001_rx.h"
#include "adi_adrv9001_rxSettings_types.h"
#include "adi_adrv9001_rx_gaincontrol.h"
#include "adi_adrv9001_rx_gaincontrol_types.h"
#include "adi_adrv9001_ssi.h"
#include "adi_adrv9001_ssi_types.h"
#include "adi_adrv9001_tx.h"
#include "adi_adrv9001_tx_types.h"
#include "adi_adrv9001_version.h"

static ssize_t adrv9002_rx_adc_type_get(struct file *file, char __user *userbuf,
					size_t count, loff_t *ppos)
{
	struct adrv9002_rx_chan	*rx = file->private_data;
	struct adrv9002_rf_phy *phy = rx_to_phy(rx, rx->channel.idx);
	char buf[8];
	adi_adrv9001_AdcType_e adc_type;
	int ret, len;

	mutex_lock(&phy->lock);
	if (!rx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

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

	mutex_lock(&phy->lock);
	if (!rx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

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

	mutex_lock(&phy->lock);
	if (!rx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

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
	adrv9002_agc_seq_printf(peak.hbGainStepAttack);
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
	if (!rx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

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
	adrv9002_agc_add_file_u8(peak.hbGainStepAttack);
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

	mutex_lock(&phy->lock);
	if (!tx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

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

	mutex_lock(&phy->lock);
	if (!tx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

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
	int val_max = phy->ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS ? 0xf : U16_MAX;
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
	int ret;

	mutex_lock(&phy->lock);
	if (!rx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	ret = adi_adrv9001_Ssi_Rx_TestMode_Configure(phy->adrv9001, rx->channel.number,
						     phy->ssi_type,
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
	int ret;

	if (!val)
		return -EINVAL;

	mutex_lock(&phy->lock);
	ret = adrv9002_init(phy, phy->curr_profile);
	mutex_unlock(&phy->lock);

	return ret;
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
	bool enable = !!val;
	int ret;

	if (enable == tx->loopback)
		return 0;

	mutex_lock(&phy->lock);
	if (!tx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	tx->loopback = enable;
	ret = adi_adrv9001_Ssi_Loopback_Set(phy->adrv9001, tx->channel.number, phy->ssi_type,
					    enable);
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
	int ret;

	mutex_lock(&phy->lock);
	if (!tx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	ret = adrv9002_axi_tx_test_pattern_cfg(phy, tx->channel.idx, tx->ssi_test.testData);
	if (ret)
		goto unlock;

	ret = adi_adrv9001_Ssi_Tx_TestMode_Configure(phy->adrv9001, tx->channel.number,
						     phy->ssi_type,
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
	adi_adrv9001_TxSsiTestModeStatus_t ssi_status = {0};
	int ret;

	mutex_lock(&phy->lock);
	if (!tx->channel.enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

	ret = adi_adrv9001_Ssi_Tx_TestMode_Status_Inspect(phy->adrv9001,
							  tx->channel.number,
							  phy->ssi_type,
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

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Ssi_Delay_Inspect(phy->adrv9001, phy->ssi_type, &delays);
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

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_Ssi_Delay_Configure(phy->adrv9001, phy->ssi_type, &phy->ssi_delays);
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
	if (!chan->enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

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
	if (!chan->enabled) {
		mutex_unlock(&phy->lock);
		return -ENODEV;
	}

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

static int adrv9002_rx_near_end_loopback_set(void *arg, const u64 val)
{
	struct adrv9002_rx_chan	*rx = arg;
	struct adrv9002_rf_phy *phy = rx_to_phy(rx, rx->channel.idx);

	adrv9002_axi_hdl_loopback(phy, rx->channel.idx, !!val);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_rx_near_end_loopback_set_fops, NULL,
			 adrv9002_rx_near_end_loopback_set, "%llu\n");

static ssize_t adrv9002_api_version_get(struct file *file, char __user *userbuf,
					size_t count, loff_t *off)
{
	return simple_read_from_buffer(userbuf, count, off, ADI_ADRV9001_CURRENT_VERSION "\n",
				       strlen(ADI_ADRV9001_CURRENT_VERSION) + 1);
}

static const struct file_operations adrv9002_api_version_get_fops = {
	.open = simple_open,
	.read = adrv9002_api_version_get,
	.llseek = default_llseek,
};

static const char * const dgpio_str[] = {
	"Unassigned", "dgpio0", "dgpio1", "dgpio2", "dgpio3", "dgpio4", "dgpio5",
	"dgpio6", "dgpio7", "dgpio8", "dgpio9", "dgpio10", "dgpio11", "dgpio12", "dgpio13",
	"dgpio14", "dgpio15"
};

static void adrv9002_fh_gain_config_dump_show(struct seq_file *s, const adi_adrv9001_FhCfg_t *cfg,
					      int chan)
{
	const adi_adrv9001_FhGainSetupByPinCfg_t *gain = &cfg->gainSetupByPinConfig[chan];
	int e;

	seq_printf(s, "Gain Pin Config channel(%d):\n", chan);
	adrv9002_seq_printf(s, &cfg->gainSetupByPinConfig[chan], numRxGainTableEntries);
	seq_puts(s, "RX Gain Table: ");
	for (e = 0; e < gain->numRxGainTableEntries; e++)
		seq_printf(s, "%u ", gain->rxGainTable[e]);
	seq_puts(s, "\n");

	adrv9002_seq_printf(s, &cfg->gainSetupByPinConfig[chan], numTxAttenTableEntries);
	seq_puts(s, "TX Atten Table: ");
	for (e = 0; e < gain->numTxAttenTableEntries; e++)
		seq_printf(s, "%u ", gain->txAttenTable[e]);
	seq_puts(s, "\n");

	adrv9002_seq_printf(s, &cfg->gainSetupByPinConfig[chan], numGainCtrlPins);
	seq_puts(s, "Gain Select Pins: ");
	for (e = 0; e < gain->numGainCtrlPins; e++)
		seq_printf(s, "%s ", dgpio_str[gain->gainSelectGpioConfig[e].pin]);
	seq_puts(s, "\n");
}

static int adrv9002_fh_config_dump_show(struct seq_file *s, void *ignored)
{
	struct adrv9002_rf_phy *phy = s->private;
	adi_adrv9001_FhCfg_t cfg = {0};
	int ret, p;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_fh_Configuration_Inspect(phy->adrv9001, &cfg);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	adrv9002_seq_printf(s, &cfg, mode);
	seq_printf(s, "RX1 Hop Signal: %d\n", cfg.rxPortHopSignals[0]);
	seq_printf(s, "RX2 Hop Signal: %d\n", cfg.rxPortHopSignals[1]);
	seq_printf(s, "TX1 Hop Signal: %d\n", cfg.txPortHopSignals[0]);
	seq_printf(s, "TX2 Hop Signal: %d\n", cfg.txPortHopSignals[1]);
	adrv9002_seq_printf(s, &cfg, rxZeroIfEnable);
	seq_printf(s, "Hop Signal 1 pin: %s\n", dgpio_str[cfg.hopSignalGpioConfig[0].pin]);
	seq_printf(s, "Hop Signal 2 pin: %s\n", dgpio_str[cfg.hopSignalGpioConfig[1].pin]);
	adrv9002_seq_printf(s, &cfg.hopTableSelectConfig, hopTableSelectMode);
	seq_printf(s, "Hop Signal 1 table select pin: %s\n",
		   dgpio_str[cfg.hopTableSelectConfig.hopTableSelectGpioConfig[0].pin]);
	seq_printf(s, "Hop Signal 2 table select pin: %s\n",
		   dgpio_str[cfg.hopTableSelectConfig.hopTableSelectGpioConfig[1].pin]);
	adrv9002_seq_printf(s, &cfg, minRxGainIndex);
	adrv9002_seq_printf(s, &cfg, maxRxGainIndex);
	adrv9002_seq_printf(s, &cfg, minTxAtten_mdB);
	adrv9002_seq_printf(s, &cfg, maxTxAtten_mdB);
	seq_printf(s, "minOperatingFrequency_Hz: %llu\n", cfg.minOperatingFrequency_Hz);
	seq_printf(s, "maxOperatingFrequency_Hz: %llu\n", cfg.maxOperatingFrequency_Hz);
	adrv9002_seq_printf(s, &cfg, minFrameDuration_us);
	adrv9002_seq_printf(s, &cfg, txAnalogPowerOnFrameDelay);
	adrv9002_seq_printf(s, &cfg, tableIndexCtrl);
	adrv9002_seq_printf(s, &cfg, numTableIndexPins);
	if (cfg.numTableIndexPins)
		seq_puts(s, "Table Index Control Pins: ");
	for (p = 0; p < cfg.numTableIndexPins; p++)
		seq_printf(s, "%s ", dgpio_str[cfg.tableIndexGpioConfig[p].pin]);
	if (cfg.numTableIndexPins)
		seq_puts(s, "\n");
	adrv9002_seq_printf(s, &cfg, gainSetupByPin);
	if (cfg.gainSetupByPin) {
		int c;

		for (c = 0; c < ADRV9002_CHANN_MAX; c++)
			adrv9002_fh_gain_config_dump_show(s, &cfg, c);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(adrv9002_fh_config_dump);

static int adrv9002_hop_table_dump_show(struct seq_file *s, int hop, int tbl_idx)
{
	struct adrv9002_rf_phy *phy = s->private;
	/* static as it would be to big to have it on the stack */
	static adi_adrv9001_FhHopFrame_t tbl[ADI_ADRV9001_FH_MAX_HOP_TABLE_SIZE];
	u32 read_back = 0, e;
	int ret;

	mutex_lock(&phy->lock);
	memset(&tbl, 0, sizeof(tbl));
	ret = adi_adrv9001_fh_HopTable_Inspect(phy->adrv9001, hop, tbl_idx, tbl,
					       ARRAY_SIZE(tbl), &read_back);
	if (ret) {
		mutex_unlock(&phy->lock);
		return adrv9002_dev_err(phy);
	}

	if (read_back)
		seq_puts(s, "hop_freq_hz rx1_off_freq_hz rx2_off_freq_hz rx1_gain tx1_atten rx2_gain tx2_atten\n");
	for (e = 0; e < read_back; e++) {
		seq_printf(s, "%-11llu %-15d %-15d %-8u %-9u %-8u %u\n", tbl[e].hopFrequencyHz,
			   tbl[e].rx1OffsetFrequencyHz, tbl[e].rx2OffsetFrequencyHz,
			   tbl[e].rx1GainIndex, tbl[e].tx1Attenuation_fifthdB,
			   tbl[e].rx2GainIndex, tbl[e].tx2Attenuation_fifthdB);
	}

	mutex_unlock(&phy->lock);
	return 0;
}

#define ADRV9002_FH_TABLE_ATTR_SHOW(h, t, hop, table)						\
static int adrv9002_hop##h##_table_##t##_dump_show(struct seq_file *s, void *ignored)		\
{												\
	return adrv9002_hop_table_dump_show(s, hop, table);					\
}												\
DEFINE_SHOW_ATTRIBUTE(adrv9002_hop##h##_table_##t##_dump)

ADRV9002_FH_TABLE_ATTR_SHOW(1, a, ADI_ADRV9001_FH_HOP_SIGNAL_1, ADI_ADRV9001_FHHOPTABLE_A);
ADRV9002_FH_TABLE_ATTR_SHOW(1, b, ADI_ADRV9001_FH_HOP_SIGNAL_1, ADI_ADRV9001_FHHOPTABLE_B);
ADRV9002_FH_TABLE_ATTR_SHOW(2, a, ADI_ADRV9001_FH_HOP_SIGNAL_2, ADI_ADRV9001_FHHOPTABLE_A);
ADRV9002_FH_TABLE_ATTR_SHOW(2, b, ADI_ADRV9001_FH_HOP_SIGNAL_2, ADI_ADRV9001_FHHOPTABLE_B);

static void adrv9002_debugfs_fh_config_create(struct adrv9002_rf_phy *phy, struct dentry *d)
{
	int hop, p;
	char attr[64];
	const struct {
		const struct file_operations *fops_a;
		const struct file_operations *fops_b;
	} hop_tbl_helper[ADRV9002_FH_HOP_SIGNALS_NR] = {
		{ &adrv9002_hop1_table_a_dump_fops, &adrv9002_hop1_table_b_dump_fops },
		{ &adrv9002_hop2_table_a_dump_fops, &adrv9002_hop2_table_b_dump_fops }
	};

	debugfs_create_u8("fh_min_rx_gain", 0600, d, &phy->fh.minRxGainIndex);
	debugfs_create_u8("fh_max_rx_gain", 0600, d, &phy->fh.maxRxGainIndex);
	debugfs_create_u8("fh_tx_analog_power_on_frame_delay", 0600, d,
			  &phy->fh.txAnalogPowerOnFrameDelay);
	debugfs_create_bool("fh_rx_zero_if_en", 0600, d, &phy->fh.rxZeroIfEnable);
	debugfs_create_u16("fh_min_tx_atten_mdb", 0600, d, &phy->fh.minTxAtten_mdB);
	debugfs_create_u16("fh_max_tx_atten_mdb", 0600, d, &phy->fh.maxTxAtten_mdB);
	debugfs_create_u32("fh_mode", 0600, d, &phy->fh.mode);
	debugfs_create_u32("fh_min_frame_duration_us", 0600, d, &phy->fh.minFrameDuration_us);
	debugfs_create_u64("fh_min_lo_freq_hz", 0600, d, &phy->fh.minOperatingFrequency_Hz);
	debugfs_create_u64("fh_max_lo_freq_hz", 0600, d, &phy->fh.maxOperatingFrequency_Hz);
	/* table index control attrs */
	debugfs_create_u32("fh_table_index_control_mode", 0600, d, &phy->fh.tableIndexCtrl);
	debugfs_create_u8("fh_table_index_control_npins", 0600, d, &phy->fh.numTableIndexPins);
	for (p = 0; p < ARRAY_SIZE(phy->fh.tableIndexGpioConfig); p++) {
		sprintf(attr, "fh_table_index_control_pin%d", p + 1);
		debugfs_create_u32(attr, 0600, d, &phy->fh.tableIndexGpioConfig[p].pin);
	}
	/* hop signals attrs */
	debugfs_create_u32("fh_hop_table_mode_select", 0600, d,
			   &phy->fh.hopTableSelectConfig.hopTableSelectMode);
	for (hop = 0; hop < ADRV9002_FH_HOP_SIGNALS_NR; hop++) {
		adi_adrv9001_FhhopTableSelectCfg_t *tbl = &phy->fh.hopTableSelectConfig;

		sprintf(attr, "fh_hop%d_pin_set", hop + 1);
		debugfs_create_u32(attr, 0600, d, &phy->fh.hopSignalGpioConfig[hop].pin);
		sprintf(attr, "fh_hop%d_table_select_pin_set", hop + 1);
		debugfs_create_u32(attr, 0600, d, &tbl->hopTableSelectGpioConfig[hop].pin);
		sprintf(attr, "fh_hop%d_table_a_dump", hop + 1);
		debugfs_create_file(attr, 0400, d, phy, hop_tbl_helper[hop].fops_a);
		sprintf(attr, "fh_hop%d_table_b_dump", hop + 1);
		debugfs_create_file(attr, 0400, d, phy, hop_tbl_helper[hop].fops_b);
	}

	for (p = 0; p < ADI_ADRV9001_NUM_CHANNELS; p++) {
		sprintf(attr, "fh_rx%d_port_hop_signal", p);
		debugfs_create_u32(attr, 0600, d, &phy->fh.rxPortHopSignals[p]);
		sprintf(attr, "fh_tx%d_port_hop_signal", p);
		debugfs_create_u32(attr, 0600, d, &phy->fh.txPortHopSignals[p]);
	}

	debugfs_create_file("fh_config_dump", 0400, d, phy, &adrv9002_fh_config_dump_fops);
}

/* does not take into account that idx 0 is ADI_ADRV9001_GPIO_UNASSIGNED */
#define ADRV9002_GPIO_MAX	ADI_ADRV9001_GPIO_ANALOG_11

static int adrv9002_gpio_val_get(void *arg, u64 *val, int gpio)
{
	struct adrv9002_rf_phy *phy = arg;
	adi_adrv9001_GpioPinDirection_e dir;
	adi_adrv9001_GpioPinLevel_e level;
	int ret;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_gpio_PinDirection_Get(phy->adrv9001, gpio, &dir);
	if (ret)
		goto unlock;

	if (dir)
		ret = adi_adrv9001_gpio_OutputPinLevel_Get(phy->adrv9001, gpio, &level);
	else
		ret = adi_adrv9001_gpio_InputPinLevel_Get(phy->adrv9001, gpio, &level);
	if (ret)
		goto unlock;

	*val = level;
unlock:
	mutex_unlock(&phy->lock);
	return ret ? adrv9002_dev_err(phy) : 0;
}

static int adrv9002_gpio_val_set(void *arg, const u64 val, int gpio)
{
	struct adrv9002_rf_phy *phy = arg;
	adi_adrv9001_GpioPinDirection_e dir;
	int ret;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_gpio_PinDirection_Get(phy->adrv9001, gpio, &dir);
	if (ret) {
		mutex_unlock(&phy->lock);
		return adrv9002_dev_err(phy);
	}

	if (dir == ADI_ADRV9001_GPIO_PIN_DIRECTION_INPUT) {
		mutex_unlock(&phy->lock);
		return -EPERM;
	}

	ret = adi_adrv9001_gpio_OutputPinLevel_Set(phy->adrv9001, gpio, !!val);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

static int adrv9002_gpio_dir_get(void *arg, u64 *val, int gpio)
{
	struct adrv9002_rf_phy *phy = arg;
	adi_adrv9001_GpioPinDirection_e dir;
	int ret;

	mutex_lock(&phy->lock);
	ret = adi_adrv9001_gpio_PinDirection_Get(phy->adrv9001, gpio, &dir);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	*val = dir;

	return 0;
}

static int (*dir_set[4])(adi_adrv9001_Device_t *adrv9001, u32 pin) = {
	adi_adrv9001_gpio_ManualInput_Configure,
	adi_adrv9001_gpio_ManualAnalogInput_Configure,
	adi_adrv9001_gpio_ManualOutput_Configure,
	adi_adrv9001_gpio_ManualAnalogOutput_Configure,
};

static int adrv9002_gpio_dir_set(void *arg, const u64 val, int gpio)
{
	struct adrv9002_rf_phy *phy = arg;
	bool agpio = (gpio > ADI_ADRV9001_GPIO_DIGITAL_15) ? true : false;
	int ret;

	if (val && agpio)
		/* the nibbles go 4 by 4 and 0 means unassigned. hence the +1 */
		gpio = (gpio - ADI_ADRV9001_GPIO_ANALOG_00) / 4 + 1;
	else if (val && !agpio)
		/* for dgpio's, the crumbs go 2 by 2 */
		gpio = (gpio - ADI_ADRV9001_GPIO_DIGITAL_00) / 2 + 1;

	mutex_lock(&phy->lock);
	ret = dir_set[2 * !!val + agpio](phy->adrv9001, gpio);
	mutex_unlock(&phy->lock);
	if (ret)
		return adrv9002_dev_err(phy);

	return 0;
}

#define gpio_idx(t, i)	\
	(strcmp(__stringify(t), "a") ? (i) + ADI_ADRV9001_GPIO_DIGITAL_00 : \
				(i) + ADI_ADRV9001_GPIO_ANALOG_00)

#define gpio_fops(t, i, v)		(&adrv9002_##t##gpio##i##_##v##_fops)

#define ADRV9002_GPIO_ATTR(t, i)						\
static int adrv9002_##t##gpio##i##_val_get(void *arg, u64 *val)			\
{										\
	return adrv9002_gpio_val_get(arg, val, gpio_idx(t, i));			\
}										\
										\
static int adrv9002_##t##gpio##i##_val_set(void *arg, const u64 val)		\
{										\
	return adrv9002_gpio_val_set(arg, val, gpio_idx(t, i));			\
}										\
DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_##t##gpio##i##_val_fops,			\
			 adrv9002_##t##gpio##i##_val_get,			\
			 adrv9002_##t##gpio##i##_val_set, "%llu\n");		\
										\
										\
static int adrv9002_##t##gpio##i##_dir_get(void *arg, u64 *val)			\
{										\
	return adrv9002_gpio_dir_get(arg, val, gpio_idx(t, i));			\
}										\
										\
static int adrv9002_##t##gpio##i##_dir_set(void *arg, const u64 val)		\
{										\
	return adrv9002_gpio_dir_set(arg, val, gpio_idx(t, i));			\
}										\
DEFINE_DEBUGFS_ATTRIBUTE(adrv9002_##t##gpio##i##_dir_fops,			\
			 adrv9002_##t##gpio##i##_dir_get,			\
			 adrv9002_##t##gpio##i##_dir_set, "%llu\n")
/* agpio's */
ADRV9002_GPIO_ATTR(a, 0);
ADRV9002_GPIO_ATTR(a, 1);
ADRV9002_GPIO_ATTR(a, 2);
ADRV9002_GPIO_ATTR(a, 3);
ADRV9002_GPIO_ATTR(a, 4);
ADRV9002_GPIO_ATTR(a, 5);
ADRV9002_GPIO_ATTR(a, 6);
ADRV9002_GPIO_ATTR(a, 7);
ADRV9002_GPIO_ATTR(a, 8);
ADRV9002_GPIO_ATTR(a, 9);
ADRV9002_GPIO_ATTR(a, 10);
ADRV9002_GPIO_ATTR(a, 11);
/* dgpio's */
ADRV9002_GPIO_ATTR(d, 0);
ADRV9002_GPIO_ATTR(d, 1);
ADRV9002_GPIO_ATTR(d, 2);
ADRV9002_GPIO_ATTR(d, 3);
ADRV9002_GPIO_ATTR(d, 4);
ADRV9002_GPIO_ATTR(d, 5);
ADRV9002_GPIO_ATTR(d, 6);
ADRV9002_GPIO_ATTR(d, 7);
ADRV9002_GPIO_ATTR(d, 8);
ADRV9002_GPIO_ATTR(d, 9);
ADRV9002_GPIO_ATTR(d, 10);
ADRV9002_GPIO_ATTR(d, 11);
ADRV9002_GPIO_ATTR(d, 12);
ADRV9002_GPIO_ATTR(d, 13);
ADRV9002_GPIO_ATTR(d, 14);
ADRV9002_GPIO_ATTR(d, 15);

static const struct {
	const char *v;
	const char *d;
	const struct file_operations *fops_l;
	const struct file_operations *fops_d;
} gpio_helper[ADRV9002_GPIO_MAX] = {
	{ "dgpio0_value", "dgpio0_direction", gpio_fops(d, 0, val), gpio_fops(d, 0, dir) },
	{ "dgpio1_value", "dgpio1_direction", gpio_fops(d, 1, val), gpio_fops(d, 1, dir) },
	{ "dgpio2_value", "dgpio2_direction", gpio_fops(d, 2, val), gpio_fops(d, 2, dir) },
	{ "dgpio3_value", "dgpio3_direction", gpio_fops(d, 3, val), gpio_fops(d, 3, dir) },
	{ "dgpio4_value", "dgpio4_direction", gpio_fops(d, 4, val), gpio_fops(d, 4, dir) },
	{ "dgpio5_value", "dgpio5_direction", gpio_fops(d, 5, val), gpio_fops(d, 5, dir) },
	{ "dgpio6_value", "dgpio6_direction", gpio_fops(d, 6, val), gpio_fops(d, 6, dir) },
	{ "dgpio7_value", "dgpio7_direction", gpio_fops(d, 7, val), gpio_fops(d, 7, dir) },
	{ "dgpio8_value", "dgpio8_direction", gpio_fops(d, 8, val), gpio_fops(d, 8, dir) },
	{ "dgpio9_value", "dgpio9_direction", gpio_fops(d, 9, val), gpio_fops(d, 9, dir) },
	{ "dgpio10_value", "dgpio10_direction", gpio_fops(d, 10, val), gpio_fops(d, 10, dir) },
	{ "dgpio11_value", "dgpio11_direction", gpio_fops(d, 11, val), gpio_fops(d, 11, dir) },
	{ "dgpio12_value", "dgpio12_direction", gpio_fops(d, 12, val), gpio_fops(d, 12, dir) },
	{ "dgpio13_value", "dgpio13_direction", gpio_fops(d, 13, val), gpio_fops(d, 13, dir) },
	{ "dgpio14_value", "dgpio14_direction", gpio_fops(d, 14, val), gpio_fops(d, 14, dir) },
	{ "dgpio15_value", "dgpio15_direction", gpio_fops(d, 15, val), gpio_fops(d, 15, dir) },
	{ "agpio0_value", "agpio0_direction", gpio_fops(a, 0, val), gpio_fops(a, 0, dir) },
	{ "agpio1_value", "agpio1_direction", gpio_fops(a, 1, val), gpio_fops(a, 1, dir) },
	{ "agpio2_value", "agpio2_direction", gpio_fops(a, 2, val), gpio_fops(a, 2, dir) },
	{ "agpio3_value", "agpio3_direction", gpio_fops(a, 3, val), gpio_fops(a, 3, dir) },
	{ "agpio4_value", "agpio4_direction", gpio_fops(a, 4, val), gpio_fops(a, 4, dir) },
	{ "agpio5_value", "agpio5_direction", gpio_fops(a, 5, val), gpio_fops(a, 5, dir) },
	{ "agpio6_value", "agpio6_direction", gpio_fops(a, 6, val), gpio_fops(a, 6, dir) },
	{ "agpio7_value", "agpio7_direction", gpio_fops(a, 7, val), gpio_fops(a, 7, dir) },
	{ "agpio8_value", "agpio8_direction", gpio_fops(a, 8, val), gpio_fops(a, 8, dir) },
	{ "agpio9_value", "agpio9_direction", gpio_fops(a, 9, val), gpio_fops(a, 9, dir) },
	{ "agpio10_value", "agpio10_direction", gpio_fops(a, 10, val), gpio_fops(a, 10, dir) },
	{ "agpio11_value", "agpio11_direction", gpio_fops(a, 11, val), gpio_fops(a, 11, dir) },
};

static void adrv9002_debugfs_gpio_config_create(struct adrv9002_rf_phy *phy, struct dentry *d)
{
	int g;

	for (g = 0; g < ARRAY_SIZE(gpio_helper); g++) {
		debugfs_create_file_unsafe(gpio_helper[g].v, 0600, d, phy, gpio_helper[g].fops_l);
		debugfs_create_file_unsafe(gpio_helper[g].d, 0600, d, phy, gpio_helper[g].fops_d);
	}
}

void adrv9002_debugfs_create(struct adrv9002_rf_phy *phy, struct dentry *d)
{
	int chan;
	char attr[64];

	if (!d)
		return;

	if (phy->ssi_type == ADI_ADRV9001_SSI_TYPE_CMOS) {
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

	debugfs_create_file("api_version", 0400, d, NULL, &adrv9002_api_version_get_fops);

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
		sprintf(attr, "tx%d_ssi_strobe_delay", chan);
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
		/* near end loopback enable */
		sprintf(attr, "rx%d_near_end_loopback", chan);
		debugfs_create_file_unsafe(attr, 0200, d, rx,
					   &adrv9002_rx_near_end_loopback_set_fops);
	}

	adrv9002_debugfs_fh_config_create(phy, d);
	adrv9002_debugfs_gpio_config_create(phy, d);
}
