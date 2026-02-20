// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV903X RF Transceiver - debugfs support
 *
 * Copyright 2020-2026 Analog Devices Inc.
 */

#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>

#include "adrv903x.h"

static ssize_t adrv903x_tr_cal_status_read(struct adrv903x_rf_phy *phy,
					   adi_adrv903x_TrackingCalibrationMask_e mask,
					   adi_adrv903x_Channels_e channel,
					   char *buf)
{
	adi_adrv903x_CalStatus_t calStatus = { 0 };
	int ret;

	scoped_guard(mutex, &phy->lock)
		ret = adrv903x_api_call(phy, adi_adrv903x_TrackingCalStatusGet, mask,
					channel, &calStatus);
	if (ret)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "err %d %% %d perf %d iter cnt %d update cnt %d\n",
			 calStatus.errorCode, calStatus.percentComplete,
			 calStatus.performanceMetric, calStatus.iterCount,
			 calStatus.updateCount);
}

static ssize_t adrv903x_debugfs_read(struct file *file, char __user *userbuf,
				     size_t count, loff_t *ppos)
{
	struct adrv903x_debugfs_entry *entry = file->private_data;
	struct adrv903x_rf_phy *phy = entry->phy;
	ssize_t len = 0;
	char buf[700];
	u64 val = 0;
	u8 chan;
	int ret;

	if (entry->out_value) {
		switch (entry->size) {
		case 1:
			val = *(u8 *)entry->out_value;
			break;
		case 2:
			val = *(u16 *)entry->out_value;
			break;
		case 4:
			val = *(u32 *)entry->out_value;
			break;
		case 5:
			val = *(bool *)entry->out_value;
			break;
		case 8:
			val = *(u64 *)entry->out_value;
			break;
		default:
			return -EINVAL;
		}
	} else if (entry->cmd) {
		switch (entry->cmd) {
		case DBGFS_TX0_QEC_STATUS:
		case DBGFS_TX1_QEC_STATUS:
		case DBGFS_TX2_QEC_STATUS:
		case DBGFS_TX3_QEC_STATUS:
		case DBGFS_TX4_QEC_STATUS:
		case DBGFS_TX5_QEC_STATUS:
		case DBGFS_TX6_QEC_STATUS:
		case DBGFS_TX7_QEC_STATUS:
			chan = ADI_ADRV903X_TX0 << (entry->cmd - DBGFS_TX0_QEC_STATUS);
			ret = adrv903x_tr_cal_status_read(phy, ADI_ADRV903X_TC_TX_QEC_MASK, chan, buf);
			if (ret < 0)
				return ret;
			len = ret;
			break;
		case DBGFS_TX0_LOL_STATUS:
		case DBGFS_TX1_LOL_STATUS:
		case DBGFS_TX2_LOL_STATUS:
		case DBGFS_TX3_LOL_STATUS:
		case DBGFS_TX4_LOL_STATUS:
		case DBGFS_TX5_LOL_STATUS:
		case DBGFS_TX6_LOL_STATUS:
		case DBGFS_TX7_LOL_STATUS:
			chan = ADI_ADRV903X_TX0 << (entry->cmd - DBGFS_TX0_LOL_STATUS);
			ret = adrv903x_tr_cal_status_read(phy, ADI_ADRV903X_TC_TX_LOL_MASK, chan, buf);
			if (ret < 0)
				return ret;
			len = ret;
			break;
		case DBGFS_RX0_QEC_STATUS:
		case DBGFS_RX1_QEC_STATUS:
		case DBGFS_RX2_QEC_STATUS:
		case DBGFS_RX3_QEC_STATUS:
		case DBGFS_RX4_QEC_STATUS:
		case DBGFS_RX5_QEC_STATUS:
		case DBGFS_RX6_QEC_STATUS:
		case DBGFS_RX7_QEC_STATUS:
			chan = ADI_ADRV903X_RX0 << (entry->cmd - DBGFS_RX0_QEC_STATUS);
			ret = adrv903x_tr_cal_status_read(phy, ADI_ADRV903X_TC_RX_QEC_MASK, chan, buf);
			if (ret < 0)
				return ret;
			len = ret;
			break;
		case DBGFS_RX0_ADC_STATUS:
		case DBGFS_RX1_ADC_STATUS:
		case DBGFS_RX2_ADC_STATUS:
		case DBGFS_RX3_ADC_STATUS:
		case DBGFS_RX4_ADC_STATUS:
		case DBGFS_RX5_ADC_STATUS:
		case DBGFS_RX6_ADC_STATUS:
		case DBGFS_RX7_ADC_STATUS:
		case DBGFS_ORX0_ADC_STATUS:
		case DBGFS_ORX1_ADC_STATUS:
			chan = ADI_ADRV903X_RX0 << (entry->cmd - DBGFS_RX0_ADC_STATUS);
			if (chan < ADI_ADRV903X_RX7) {
				ret = adrv903x_tr_cal_status_read(phy, ADI_ADRV903X_TC_RX_ADC_MASK, chan, buf);
			} else {
				chan = ADI_ADRV903X_ORX0 << (entry->cmd - DBGFS_ORX0_ADC_STATUS);
				ret = adrv903x_tr_cal_status_read(phy, ADI_ADRV903X_TC_ORX_ADC_MASK, chan, buf);
			}
			if (ret < 0)
				return ret;
			len = ret;
			break;
		default:
			val = entry->val;
		}
	} else {
		return -EFAULT;
	}

	if (!len)
		len = snprintf(buf, sizeof(buf), "%llu\n", val);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t adrv903x_debugfs_write(struct file *file,
				      const char __user *userbuf, size_t count,
				      loff_t *ppos)
{
	struct adrv903x_debugfs_entry *entry = file->private_data;
	struct adrv903x_rf_phy *phy = entry->phy;
	adi_adrv903x_FrmTestDataCfg_t frm_test_data;
	adi_adrv903x_TxTestNcoConfig_t txNcoConfig = { 0 };
	u32 val2, val3, val4;
	s64 val;
	char buf[80];
	int ret;

	ret = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, userbuf, count);
	if (ret < 0)
		return ret;
	buf[ret] = '\0';

	ret = sscanf(buf, "%lli %i %i %i", &val, &val2, &val3, &val4);
	if (ret < 1)
		return -EINVAL;

	guard(mutex)(&phy->lock);

	switch (entry->cmd) {
	case DBGFS_BIST_FRAMER_0_PRBS:
		frm_test_data.injectPoint = ADI_ADRV903X_FTD_FRAMERINPUT;
		frm_test_data.testDataSource = val;
		frm_test_data.framerSelMask = ADI_ADRV903X_FRAMER_0;

		ret = adrv903x_api_call(phy, adi_adrv903x_FramerTestDataSet,
					&frm_test_data);
		if (ret)
			return ret;

		entry->val = val;
		return count;

	case DBGFS_BIST_FRAMER_1_PRBS:
		frm_test_data.injectPoint = ADI_ADRV903X_FTD_FRAMERINPUT;
		frm_test_data.testDataSource = val;
		frm_test_data.framerSelMask = ADI_ADRV903X_FRAMER_1;

		ret = adrv903x_api_call(phy, adi_adrv903x_FramerTestDataSet,
					&frm_test_data);
		if (ret)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_BIST_FRAMER_0_LOOPBACK:
		ret = adi_adrv903x_FramerLoopbackSet(phy->palauDevice, ADI_ADRV903X_FRAMER_0);
		if (ret)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_BIST_FRAMER_1_LOOPBACK:
		ret = adi_adrv903x_FramerLoopbackSet(phy->palauDevice, ADI_ADRV903X_FRAMER_1);
		if (ret)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_BIST_TONE:
		if (ret > 4)
			return -EINVAL;

		/* Channel select: val = 0-7 for TX0-TX7 */
		if (val > 7)
			val = 7;
		txNcoConfig.chanSelect = ADI_ADRV903X_TX0 << val;

		txNcoConfig.enable = val2;
		txNcoConfig.ncoSelect = ADI_ADRV903X_TX_TEST_NCO_0;
		txNcoConfig.frequencyKhz = val3;
		if (ret >= 4) {
			if (val4 > 8)
				val4  = 8;
			txNcoConfig.attenCtrl = val4;
		} else {
			txNcoConfig.attenCtrl = ADI_ADRV903X_TX_TEST_NCO_ATTEN_0DB;
		}

		ret = adrv903x_api_call(phy, adi_adrv903x_TxTestToneSet, &txNcoConfig);
		if (ret)
			return ret;

		entry->val = val;
		return count;

	default:
		break;
	}

	if (entry->out_value) {
		switch (entry->size) {
		case 1:
			*(u8 *)entry->out_value = val;
			break;
		case 2:
			*(u16 *)entry->out_value = val;
			break;
		case 4:
			*(u32 *)entry->out_value = val;
			break;
		case 5:
			*(bool *)entry->out_value = val;
			break;
		case 8:
			*(u64 *)entry->out_value = val;
			break;
		default:
			return -EINVAL;
		}
	}

	return count;
}

static const struct file_operations adrv903x_debugfs_reg_fops = {
	.open = simple_open,
	.read = adrv903x_debugfs_read,
	.write = adrv903x_debugfs_write,
};

static void adrv903x_add_debugfs_entry(struct adrv903x_rf_phy *phy,
				       const char *propname, unsigned int cmd)
{
	unsigned int i = phy->adrv903x_debugfs_entry_index;

	if (i >= ARRAY_SIZE(phy->debugfs_entry))
		return;

	phy->debugfs_entry[i].phy = phy;
	phy->debugfs_entry[i].propname = propname;
	phy->debugfs_entry[i].cmd = cmd;

	phy->adrv903x_debugfs_entry_index++;
}

void adrv903x_register_debugfs(struct iio_dev *indio_dev)
{
	struct adrv903x_rf_phy *phy = iio_priv(indio_dev);
	umode_t mode = 0644;
	int i;

	if (!iio_get_debugfs_dentry(indio_dev))
		return;

	adrv903x_add_debugfs_entry(phy, "bist_framer_0_prbs",
				   DBGFS_BIST_FRAMER_0_PRBS);
	adrv903x_add_debugfs_entry(phy, "bist_framer_1_prbs",
				   DBGFS_BIST_FRAMER_1_PRBS);
	adrv903x_add_debugfs_entry(phy, "bist_framer_0_loopback",
				   DBGFS_BIST_FRAMER_0_LOOPBACK);
	adrv903x_add_debugfs_entry(phy, "bist_framer_1_loopback",
				   DBGFS_BIST_FRAMER_1_LOOPBACK);
	adrv903x_add_debugfs_entry(phy, "bist_tone", DBGFS_BIST_TONE);
	for (i = CHAN_TX1; i < ADI_ADRV903X_MAX_TXCHANNELS; i++) {
		if (i == CHAN_TX1 || i == CHAN_TX5 || !phy->chip_info->is_adrv9032r) {
			adrv903x_add_debugfs_entry(phy,
						   devm_kasprintf(&phy->spi->dev, GFP_KERNEL, "tx%d_qec_status", i),
						   DBGFS_TX0_QEC_STATUS + i);
			adrv903x_add_debugfs_entry(phy,
						   devm_kasprintf(&phy->spi->dev, GFP_KERNEL, "tx%d_lol_status", i),
						   DBGFS_TX0_LOL_STATUS + i);
			adrv903x_add_debugfs_entry(phy,
						   devm_kasprintf(&phy->spi->dev, GFP_KERNEL, "rx%d_qec_status", i),
						   DBGFS_RX0_QEC_STATUS + i);
			adrv903x_add_debugfs_entry(phy,
						   devm_kasprintf(&phy->spi->dev, GFP_KERNEL, "rx%d_adc_status", i),
						   DBGFS_RX0_ADC_STATUS + i);
		}
	}
	adrv903x_add_debugfs_entry(phy, "orx0_adc_status", DBGFS_ORX0_ADC_STATUS);
	adrv903x_add_debugfs_entry(phy, "orx1_adc_status", DBGFS_ORX1_ADC_STATUS);

	for (i = 0; i < phy->adrv903x_debugfs_entry_index; i++) {
		if (phy->adrv903x_debugfs_entry_index > DBGFS_BIST_TONE)
			mode = 0400;
		debugfs_create_file(phy->debugfs_entry[i].propname, mode,
				    iio_get_debugfs_dentry(indio_dev),
				    &phy->debugfs_entry[i],
				    &adrv903x_debugfs_reg_fops);
	}
}
