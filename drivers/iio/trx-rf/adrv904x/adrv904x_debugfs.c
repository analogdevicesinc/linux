// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV904X RF Transceiver - debugfs support
 *
 * Copyright 2020-2026 Analog Devices Inc.
 */

#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>

#include "adrv904x.h"
#include "adi_adrv904x_datainterface.h"

static ssize_t adrv904x_debugfs_read(struct file *file, char __user *userbuf,
				     size_t count, loff_t *ppos)
{
	struct adrv904x_debugfs_entry *entry = file->private_data;
	char buf[700];
	u64 val = 0;
	ssize_t len = 0;
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
		case DBGFS_TX_TO_ORX_MAPPING: {
			struct adrv904x_rf_phy *phy = entry->phy;
			adi_adrv904x_TxChannels_e tx_orx0 = ADI_ADRV904X_TXOFF;
			adi_adrv904x_TxChannels_e tx_orx1 = ADI_ADRV904X_TXOFF;

			guard(mutex)(&phy->lock);

			ret = adrv904x_api_call(phy, adi_adrv904x_TxToOrxMappingGet,
						ADI_ADRV904X_ORX0, &tx_orx0);
			if (ret)
				return ret;

			ret = adrv904x_api_call(phy, adi_adrv904x_TxToOrxMappingGet,
						ADI_ADRV904X_ORX1, &tx_orx1);
			if (ret)
				return ret;

			val = (ffs(tx_orx1) ? (ffs(tx_orx1) - 1) << 4 : 0) |
			      (ffs(tx_orx0) ? (ffs(tx_orx0) - 1) : 0);
			break;
		}
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

static ssize_t adrv904x_debugfs_write(struct file *file,
				      const char __user *userbuf, size_t count,
				      loff_t *ppos)
{
	struct adrv904x_debugfs_entry *entry = file->private_data;
	struct adrv904x_rf_phy *phy = entry->phy;
	adi_adrv904x_FrmTestDataCfg_t frm_test_data;
	adi_adrv904x_TxTestNcoConfig_t txNcoConfig;
	u32 val2, val3;
	s64 val;
	char buf[80];
	char *p;
	int ret, nargs;

	ret = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, userbuf, count);
	if (ret < 0)
		return ret;
	buf[ret] = '\0';

	ret = kstrtoll(strim(buf), 10, &val);
	if (ret)
		return ret;

	guard(mutex)(&phy->lock);

	switch (entry->cmd) {
	case DBGFS_BIST_FRAMER_0_PRBS:
		frm_test_data.injectPoint = ADI_ADRV904X_FTD_FRAMERINPUT;
		frm_test_data.testDataSource = val;
		frm_test_data.framerSelMask = ADI_ADRV904X_FRAMER_0;

		ret = adrv904x_api_call(phy, adi_adrv904x_FramerTestDataSet,
				       &frm_test_data);
		if (ret)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_BIST_FRAMER_LOOPBACK:
		ret = adrv904x_api_call(phy, adi_adrv904x_FramerLoopbackSet,
				       ADI_ADRV904X_FRAMER_0);
		if (ret)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_TX_TO_ORX_MAPPING:
		if (val < 0 || val > 0xff)
			return -EINVAL;

		ret = adrv904x_api_call(phy, adi_adrv904x_TxToOrxMappingSet,
					(u8)val);
		if (ret)
			return ret;

		return count;
	case DBGFS_BIST_TONE:
		/* Parse additional values: enable freq_khz [atten] */
		p = strchr(buf, ' ');
		if (!p)
			return -EINVAL;
		nargs = sscanf(p, "%u %u", &val2, &val3);
		if (nargs < 1)
			return -EINVAL;

		txNcoConfig.chanSelect = ADI_ADRV904X_TX7;
		txNcoConfig.bandSelect = 0;
		txNcoConfig.enable = val;
		txNcoConfig.ncoSelect = ADI_ADRV904X_TX_TEST_NCO_0;
		txNcoConfig.frequencyKhz = val2;
		if (nargs == 2) {
			if (val3 > 8)
				val3 = 8;
			txNcoConfig.attenCtrl = val3;
		} else {
			txNcoConfig.attenCtrl = ADI_ADRV904X_TX_TEST_NCO_ATTEN_0DB;
		}

		ret = adrv904x_api_call(phy, adi_adrv904x_TxTestToneSet,
				       &txNcoConfig);
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

static const struct file_operations adrv904x_debugfs_reg_fops = {
	.open = simple_open,
	.read = adrv904x_debugfs_read,
	.write = adrv904x_debugfs_write,
};

static void adrv904x_add_debugfs_entry(struct adrv904x_rf_phy *phy,
				       const char *propname, unsigned int cmd)
{
	unsigned int i = phy->adrv904x_debugfs_entry_index;

	if (i >= ARRAY_SIZE(phy->debugfs_entry))
		return;

	phy->debugfs_entry[i].phy = phy;
	phy->debugfs_entry[i].propname = propname;
	phy->debugfs_entry[i].cmd = cmd;

	phy->adrv904x_debugfs_entry_index++;
}

void adrv904x_register_debugfs(struct iio_dev *indio_dev)
{
	struct adrv904x_rf_phy *phy = iio_priv(indio_dev);
	int i;

	if (!iio_get_debugfs_dentry(indio_dev))
		return;

	adrv904x_add_debugfs_entry(phy, "bist_framer_0_prbs",
				   DBGFS_BIST_FRAMER_0_PRBS);
	adrv904x_add_debugfs_entry(phy, "bist_framer_loopback",
				   DBGFS_BIST_FRAMER_LOOPBACK);
	adrv904x_add_debugfs_entry(phy, "bist_tone", DBGFS_BIST_TONE);
	adrv904x_add_debugfs_entry(phy, "tx_to_orx_mapping",
				   DBGFS_TX_TO_ORX_MAPPING);

	for (i = 0; i < phy->adrv904x_debugfs_entry_index; i++)
		debugfs_create_file(phy->debugfs_entry[i].propname, 0644,
				    iio_get_debugfs_dentry(indio_dev),
				    &phy->debugfs_entry[i],
				    &adrv904x_debugfs_reg_fops);
}
