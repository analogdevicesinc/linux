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
#include "adi_adrv904x_datainterface_types.h"

static ssize_t adrv904x_debugfs_read_rx_capture(struct adrv904x_rf_phy *phy,
						char __user *userbuf,
						size_t count, loff_t *ppos)
{
	char *out_buf __free(kvfree) = NULL;
	size_t buf_size, pos = 0;
	u32 i;

	guard(mutex)(&phy->lock);

	if (!phy->rx_capture_data || !phy->rx_capture_len)
		return -ENODATA;

	/* Allocate buffer: up to 12 chars per value (including newline) */
	buf_size = phy->rx_capture_len * 12 + 1;
	out_buf = kvzalloc(buf_size, GFP_KERNEL);
	if (!out_buf)
		return -ENOMEM;

	for (i = 0; i < phy->rx_capture_len && pos < buf_size - 12; i++)
		pos += scnprintf(out_buf + pos, buf_size - pos, "%u\n",
				 phy->rx_capture_data[i]);

	return simple_read_from_buffer(userbuf, count, ppos, out_buf, pos);
}

static ssize_t adrv904x_debugfs_read(struct file *file, char __user *userbuf,
				     size_t count, loff_t *ppos)
{
	struct adrv904x_debugfs_entry *entry = file->private_data;
	struct adrv904x_rf_phy *phy = entry->phy;
	char buf[700];
	u64 val = 0;
	ssize_t len = 0;

	if (entry->cmd == DBGFS_RX_DATA_CAPTURE)
		return adrv904x_debugfs_read_rx_capture(phy, userbuf, count, ppos);

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
		val = entry->val;
	} else {
		return -EFAULT;
	}

	if (!len)
		len = snprintf(buf, sizeof(buf), "%llu\n", val);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

/*
 * Parse and validate RX capture parameters. Returns 0 on success.
 * Does NOT allocate memory or call hardware - that's done by caller.
 */
static int adrv904x_parse_rx_capture_params(const char *buf, s64 *channel,
					    u32 *length,
					    adi_adrv904x_RxChannels_e *chan_sel)
{
	if (sscanf(buf, "%lld %u", channel, length) < 2)
		return -EINVAL;

	/* Channel: 0-7 for RX0-RX7, 8-9 for ORX0-ORX1 */
	if (*channel < 0 || *channel > 9)
		return -EINVAL;

	/* Validate capture length against hardware-supported sizes */
	switch (*length) {
	case ADI_ADRV904X_CAPTURE_SIZE_32:
	case ADI_ADRV904X_CAPTURE_SIZE_64:
	case ADI_ADRV904X_CAPTURE_SIZE_128:
	case ADI_ADRV904X_CAPTURE_SIZE_256:
	case ADI_ADRV904X_CAPTURE_SIZE_512:
	case ADI_ADRV904X_CAPTURE_SIZE_1K:
	case ADI_ADRV904X_CAPTURE_SIZE_2K:
	case ADI_ADRV904X_CAPTURE_SIZE_4K:
	case ADI_ADRV904X_CAPTURE_SIZE_8K:
	case ADI_ADRV904X_CAPTURE_SIZE_12K:
		break;
	case ADI_ADRV904X_CAPTURE_SIZE_16K:
	case ADI_ADRV904X_CAPTURE_SIZE_32K:
		/* ORX channels limited to 12K max */
		if (*channel >= 8)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	if (*channel < 8)
		*chan_sel = ADI_ADRV904X_RX0 << *channel;
	else
		*chan_sel = ADI_ADRV904X_ORX0 << (*channel - 8);

	return 0;
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

	/*
	 * Handle RX_DATA_CAPTURE before taking the lock since it needs
	 * to allocate memory with GFP_KERNEL which can sleep.
	 */
	if (entry->cmd == DBGFS_RX_DATA_CAPTURE) {
		adi_adrv904x_RxChannels_e chan_sel;
		u32 *capture_buf __free(kfree) = NULL;

		ret = adrv904x_parse_rx_capture_params(buf, &val, &val2,
						       &chan_sel);
		if (ret)
			return ret;

		capture_buf = kcalloc(val2, sizeof(u32), GFP_KERNEL);
		if (!capture_buf)
			return -ENOMEM;

		guard(mutex)(&phy->lock);

		ret = adi_adrv904x_RxOrxDataCaptureStart(phy->kororDevice,
							 chan_sel,
							 ADI_ADRV904X_CAPTURE_LOC_DDC0,
							 capture_buf, val2,
							 0, 1000000);
		if (ret)
			return __adrv904x_dev_err(phy, __func__, __LINE__);

		kfree(phy->rx_capture_data);
		phy->rx_capture_data = no_free_ptr(capture_buf);
		phy->rx_capture_len = val2;
		entry->val = val;

		return count;
	}

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
	adrv904x_add_debugfs_entry(phy, "rx_data_capture", DBGFS_RX_DATA_CAPTURE);

	for (i = 0; i < phy->adrv904x_debugfs_entry_index; i++)
		debugfs_create_file(phy->debugfs_entry[i].propname, 0644,
				    iio_get_debugfs_dentry(indio_dev),
				    &phy->debugfs_entry[i],
				    &adrv904x_debugfs_reg_fops);
}
