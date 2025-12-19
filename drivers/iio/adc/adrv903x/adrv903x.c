// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9025/6 RF Transceiver
 *
 * Copyright 2020-2023 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>

#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/of.h>

// #include <asm/unaligned.h>

#include <linux/iio/sysfs.h>
#include <linux/iio/iio.h>

#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/clk.h>

#include <linux/jesd204/jesd204.h>

#include <dt-bindings/iio/adc/adi,adrv903x.h>

#include "adrv903x.h"
#include "initdata.h"

#define ADI_FILE ADI_COMMON_FILE_HAL

enum adrv903x_iio_dev_attr {
	ADRV903X_INIT_CAL,
	ADRV903X_JESD204_FSM_ERROR,
	ADRV903X_JESD204_FSM_PAUSED,
	ADRV903X_JESD204_FSM_STATE,
	ADRV903X_JESD204_FSM_RESUME,
	ADRV903X_JESD204_FSM_CTRL,
};

static int __adrv903x_dev_err(struct adrv903x_rf_phy *phy, const char *function,
			      const int line)
{
	int ret = 0;

	dev_err(&phy->spi->dev, "%s, %d: failed with %s (%d)\n", function, line,
		phy->palauDevice->common.errPtr->errDebugInfo.errCause,
		phy->palauDevice->common.errPtr->errDebugInfo.highestPriorityAction);

	switch (phy->palauDevice->common.errPtr->errDebugInfo.highestPriorityAction) {
	case ADI_COMMON_ERR_ACT_CHECK_PARAM:
		ret = -EINVAL;
		break;
	case ADI_COMMON_ERR_ACT_RESET_FEATURE:
		ret = -EFAULT;
		break;
	case ADI_COMMON_ERR_ACT_CHECK_INTERFACE:
		ret = -EIO;
		break;
	case ADI_COMMON_ERR_ACT_NONE:
		ret = 0;
		break;
	default:
		ret = -EFAULT;
	}

	printk("ERROR\n");

	adi_common_ErrClear(phy->palauDevice->common.errPtr);

	return ret;
}

#define adrv903x_dev_err(phy) __adrv903x_dev_err(phy, __func__, __LINE__)

int adrv903x_spi_read(struct spi_device *spi, unsigned int reg)
{
	unsigned char buf[3];
	int ret;

	buf[0] = 0x80 | (reg >> 8);
	buf[1] = reg & 0xFF;
	ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 1);

	dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n", __func__, reg,
		buf[2], ret);

	if (ret < 0) {
		dev_err(&spi->dev, "%s: failed (%d)\n", __func__, ret);
		return ret;
	}

	return buf[2];
}

int adrv903x_spi_write(struct spi_device *spi, unsigned int reg, unsigned int val)
{
	unsigned char buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val;

	ret = spi_write_then_read(spi, buf, 3, NULL, 0);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: failed (%d)\n", __func__, ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n", __func__, reg, val,
		ret);

	return 0;
}

/* Helper function to find the sampling rate for a specific deframer
 *
 * Returns recovery action required.
 */
int adrv903x_TxLinkSamplingRateFind(adi_adrv903x_Device_t *device,
				    adi_adrv903x_DeframerSel_e deframerSel,
				    u32 *iqRate_kHz)
{
	int recoveryAction = ADI_COMMON_ERR_ACT_NONE;
	u32 deframerIndex = 0;
	u32 rate;

	/* Check device pointer is not null */
	ADI_NULL_DEVICE_PTR_RETURN(device);
	ADI_NULL_PTR_RETURN(iqRate_kHz);

	ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_HAL_LOG_API);

	switch (deframerSel) {
	case ADI_ADRV903X_DEFRAMER_0:
		deframerIndex = 0;
		break;
	case ADI_ADRV903X_DEFRAMER_1:
		deframerIndex = 1;
		break;
	default:
		ADI_ERROR_REPORT(
			&device->common, ADI_COMMON_ERRSRC_API,
			ADI_COMMON_ERRCODE_INVALID_PARAM,
			ADI_COMMON_ERR_ACT_CHECK_PARAM, deframerSel,
			"Only one deframer can be selected at a time.");
		if (device->common.errPtr->errDebugInfo.highestPriorityAction)
			return device->common.errPtr->errDebugInfo.highestPriorityAction;
		break;
	}

	if (device->initExtract.jesdSetting.deframerSetting[deframerIndex].jesdM < 1) {
		*iqRate_kHz = 0;
		return recoveryAction;
	}

	rate = device->initExtract.jesdSetting.deframerSetting[deframerIndex].iqRate_kHz;

	*iqRate_kHz = rate;

	return recoveryAction;
}

/* Helper function for shutdown procedure
 *
 */
static void adrv903x_shutdown(struct adrv903x_rf_phy *phy)
{
	/***********************************************
	 * Shutdown Procedure *
	 * **********************************************/
	/* Function to turn radio off, Disables transmitters and receivers */

	adi_adrv903x_Shutdown(phy->palauDevice);
	adi_adrv903x_HwClose(phy->palauDevice);

	memset(&phy->adi_adrv903x_device.devStateInfo, 0,
	       sizeof(phy->adi_adrv903x_device.devStateInfo));
}

static ssize_t adrv903x_phy_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adrv903x_rf_phy *phy = iio_priv(indio_dev);
	bool enable;
	int ret = 0;
	u64 val;

	mutex_lock(&phy->lock);

	switch ((u32)this_attr->address & 0xFF) {
	case ADRV903X_INIT_CAL:
		ret = kstrtobool(buf, &enable);
		if (ret)
			break;

		val = (u64)this_attr->address >> 8;

		if (val) {
			if (enable)
				phy->cal_mask.calMask |= val;
			else
				phy->cal_mask.calMask &= ~val;
		} else if (enable) {
			static const u32 INIT_CALS_TIMEOUT_MS =
				60000; /*60 seconds timeout*/

			phy->cal_mask.orxChannelMask =
				phy->adrv903xPostMcsInitInst.initCals.orxChannelMask;
			phy->cal_mask.rxChannelMask =
				phy->adrv903xPostMcsInitInst.initCals.rxChannelMask;
			phy->cal_mask.txChannelMask =
				phy->adrv903xPostMcsInitInst.initCals.txChannelMask;

			/* Run Init Cals */
			ret = adi_adrv903x_InitCalsRun(phy->palauDevice,
						       &phy->cal_mask);
			if (ret) {
				ret = adrv903x_dev_err(phy);
				break;
			}

			ret = adi_adrv903x_InitCalsWait(phy->palauDevice,
							INIT_CALS_TIMEOUT_MS);
			if (ret)
				ret = adrv903x_dev_err(phy);
		}
		break;
	case ADRV903X_JESD204_FSM_RESUME:
		if (!phy->jdev) {
			ret = -ENOTSUPP;
			break;
		}

		ret = jesd204_fsm_resume(phy->jdev, JESD204_LINKS_ALL);
		break;
	case ADRV903X_JESD204_FSM_CTRL:
		if (!phy->jdev) {
			ret = -ENOTSUPP;
			break;
		}

		ret = kstrtobool(buf, &enable);
		if (ret)
			break;

		if (enable) {
			jesd204_fsm_stop(phy->jdev, JESD204_LINKS_ALL);
			jesd204_fsm_clear_errors(phy->jdev, JESD204_LINKS_ALL);
			ret = jesd204_fsm_start(phy->jdev, JESD204_LINKS_ALL);
		} else {
			jesd204_fsm_stop(phy->jdev, JESD204_LINKS_ALL);
			jesd204_fsm_clear_errors(phy->jdev, JESD204_LINKS_ALL);
			ret = 0;
		}

		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	return ret ? ret : len;
}

static ssize_t adrv903x_phy_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adrv903x_rf_phy *phy = iio_priv(indio_dev);
	struct jesd204_dev *jdev = phy->jdev;
	struct jesd204_link *links[5];
	int ret = 0, i, err, num_links;
	bool paused;
	u64 val;

	mutex_lock(&phy->lock);
	switch ((u32)this_attr->address & 0xFF) {
	case ADRV903X_INIT_CAL:
		val = this_attr->address >> 8;

		if (val)
			ret = sprintf(buf, "%d\n",
				      !!(phy->cal_mask.calMask & val));
		break;
	case ADRV903X_JESD204_FSM_ERROR:
		if (!phy->jdev) {
			ret = -ENOTSUPP;
			break;
		}

		num_links = jesd204_get_active_links_num(jdev);
		if (num_links < 0) {
			ret = num_links;
			break;
		}

		ret = jesd204_get_links_data(jdev, links, num_links);
		if (ret)
			break;

		err = 0;
		for (i = 0; i < num_links; i++) {
			if (links[i]->error) {
				err = links[i]->error;
				break;
			}
		}
		ret = sprintf(buf, "%d\n", err);
		break;
	case ADRV903X_JESD204_FSM_PAUSED:
		if (!phy->jdev) {
			ret = -ENOTSUPP;
			break;
		}

		num_links = jesd204_get_active_links_num(jdev);
		if (num_links < 0) {
			ret = num_links;
			break;
		}

		ret = jesd204_get_links_data(jdev, links, num_links);
		if (ret)
			break;
		/*
		 * Take the slowest link; if there are N links and one is paused, all are paused.
		 * Not sure if this can happen yet, but best design it like this here.
		 */
		paused = false;
		for (i = 0; i < num_links; i++) {
			if (jesd204_link_get_paused(links[i])) {
				paused = true;
				break;
			}
		}
		ret = sprintf(buf, "%d\n", paused);
		break;
	case ADRV903X_JESD204_FSM_STATE:
		if (!phy->jdev) {
			ret = -ENOTSUPP;
			break;
		}

		num_links = jesd204_get_active_links_num(jdev);
		if (num_links < 0) {
			ret = num_links;
			break;
		}

		ret = jesd204_get_links_data(jdev, links, num_links);
		if (ret)
			break;
		/*
		 * just get the first link state; we're assuming that all 3 are in sync
		 * and that ADRV903X_JESD204_FSM_PAUSED was called before
		 */
		ret = sprintf(buf, "%s\n", jesd204_link_get_state_str(links[0]));
		break;
	case ADRV903X_JESD204_FSM_CTRL:
		if (!phy->jdev) {
			ret = -ENOTSUPP;
			break;
		}

		ret = sprintf(buf, "%d\n", phy->is_initialized);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&phy->lock);

	return ret;
}

static IIO_DEVICE_ATTR(calibrate, 0644, adrv903x_phy_show,
		       adrv903x_phy_store, ADRV903X_INIT_CAL);

static IIO_DEVICE_ATTR(calibrate_rx_dc_offset_en, 0644,
		       adrv903x_phy_show, adrv903x_phy_store,
		       ADRV903X_INIT_CAL | (ADI_ADRV903X_IC_RX_DC_OFFSET << 8));

static IIO_DEVICE_ATTR(calibrate_rx_adc_en, 0644,
		       adrv903x_phy_show, adrv903x_phy_store,
		       ADRV903X_INIT_CAL | (ADI_ADRV903X_IC_ADC_RX << 8));

static IIO_DEVICE_ATTR(calibrate_orx_adc_en, 0644,
		       adrv903x_phy_show, adrv903x_phy_store,
		       ADRV903X_INIT_CAL | (ADI_ADRV903X_IC_ADC_ORX << 8));

static IIO_DEVICE_ATTR(calibrate_tx_loopback_en, 0644,
		       adrv903x_phy_show, adrv903x_phy_store,
		       ADRV903X_INIT_CAL | (ADI_ADRV903X_IC_ADC_TXLB << 8));

static IIO_DEVICE_ATTR(calibrate_tx_dac_en, 0644,
		       adrv903x_phy_show, adrv903x_phy_store,
		       ADRV903X_INIT_CAL | (ADI_ADRV903X_IC_TXDAC << 8));

static IIO_DEVICE_ATTR(calibrate_tx_bb_filter_en, 0644,
		       adrv903x_phy_show, adrv903x_phy_store,
		       ADRV903X_INIT_CAL | (ADI_ADRV903X_IC_TXBBF << 8));

static IIO_DEVICE_ATTR(calibrate_tx_loopback_filter_en, 0644,
		       adrv903x_phy_show, adrv903x_phy_store,
		       ADRV903X_INIT_CAL | (ADI_ADRV903X_IC_TXLB_FILTER << 8));

static IIO_DEVICE_ATTR(calibrate_tx_loopback_path_delay_en, 0644,
		       adrv903x_phy_show, adrv903x_phy_store,
		       ADRV903X_INIT_CAL | (ADI_ADRV903X_IC_TXLB_PATH_DLY << 8));

// static IIO_DEVICE_ATTR(calibrate_tx_atten_en, 0644,
// 		       adrv903x_phy_show, adrv903x_phy_store,
// 		       ADRV903X_INIT_CAL | (ADI_ADRV903X_IC_TX_ATTEN_CAL << 8));

static IIO_DEVICE_ATTR(calibrate_tx_hrm_en, 0644,
		       adrv903x_phy_show, adrv903x_phy_store,
		       ADRV903X_INIT_CAL | (ADI_ADRV903X_IC_HRM << 8));

static IIO_DEVICE_ATTR(calibrate_tx_qec_en, 0644,
		       adrv903x_phy_show, adrv903x_phy_store,
		       ADRV903X_INIT_CAL | (ADI_ADRV903X_IC_TXQEC << 8));

static IIO_DEVICE_ATTR(calibrate_tx_lol_en, 0644,
		       adrv903x_phy_show, adrv903x_phy_store,
		       ADRV903X_INIT_CAL |
			       (ADI_ADRV903X_IC_TXLOL << 8));

static IIO_DEVICE_ATTR(jesd204_fsm_error, 0444,
		       adrv903x_phy_show,
		       NULL,
		       ADRV903X_JESD204_FSM_ERROR);

static IIO_DEVICE_ATTR(jesd204_fsm_paused, 0444,
		       adrv903x_phy_show,
		       NULL,
		       ADRV903X_JESD204_FSM_PAUSED);

static IIO_DEVICE_ATTR(jesd204_fsm_state, 0444,
		       adrv903x_phy_show,
		       NULL,
		       ADRV903X_JESD204_FSM_STATE);

static IIO_DEVICE_ATTR(jesd204_fsm_resume, 0200,
		       NULL,
		       adrv903x_phy_store,
		       ADRV903X_JESD204_FSM_RESUME);

static IIO_DEVICE_ATTR(jesd204_fsm_ctrl, 0644,
		       adrv903x_phy_show,
		       adrv903x_phy_store,
		       ADRV903X_JESD204_FSM_CTRL);

static struct attribute *adrv903x_phy_attributes[] = {
	&iio_dev_attr_calibrate.dev_attr.attr,
	&iio_dev_attr_calibrate_rx_dc_offset_en.dev_attr.attr,
	&iio_dev_attr_calibrate_rx_adc_en.dev_attr.attr,
	&iio_dev_attr_calibrate_orx_adc_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_loopback_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_dac_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_bb_filter_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_loopback_filter_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_loopback_path_delay_en.dev_attr.attr,
	// &iio_dev_attr_calibrate_tx_atten_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_hrm_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_qec_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_lol_en.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_error.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_state.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_paused.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_resume.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_ctrl.dev_attr.attr,
	NULL,
};

static const struct attribute_group adrv903x_phy_attribute_group = {
	.attrs = adrv903x_phy_attributes,
};

static int adrv903x_phy_reg_access(struct iio_dev *indio_dev, u32 reg,
				   u32 writeval, u32 *readval)
{
	struct adrv903x_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&phy->lock);
	if (readval == NULL)
		ret = adrv903x_spi_write(phy->spi, reg, writeval);
	else {
		*readval = adrv903x_spi_read(phy->spi, reg);
		ret = 0;
	}
	mutex_unlock(&phy->lock);

	return ret;
}

enum lo_ext_info {
	LOEXT_FREQ,
};

static ssize_t adrv903x_phy_lo_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adrv903x_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv903x_LoConfig_t loConfig = { 0 };
	int ret = 0;
	u64 readin;

	switch (private) {
	case LOEXT_FREQ:

		ret = kstrtoull(buf, 10, &readin);
		if (ret)
			return ret;

		loConfig.loFrequency_Hz = readin;
		loConfig.loName = ADI_ADRV903X_LO0 + chan->channel;

		mutex_lock(&phy->lock);

		ret = adi_adrv903x_LoFrequencySet(
			phy->palauDevice, &loConfig);
		if (ret)
			ret = adrv903x_dev_err(phy);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&phy->lock);

	return ret ? ret : len;
}

static ssize_t adrv903x_phy_lo_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct adrv903x_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv903x_LoConfigReadback_t loConfig = { 0 };
	int ret;

	mutex_lock(&phy->lock);
	switch (private) {
	case LOEXT_FREQ:
		loConfig.loName = ADI_ADRV903X_LO0 + chan->channel;
		ret = adi_adrv903x_LoFrequencyGet(
			phy->palauDevice, &loConfig);
		if (ret)
			ret = adrv903x_dev_err(phy);
		break;
	default:
		ret = 0;
	}
	mutex_unlock(&phy->lock);

	return ret ? ret : sprintf(buf, "%llu\n", loConfig.loFrequency_Hz);
}

#define _ADRV903X_EXT_LO_INFO(_name, _ident)                                   \
	{                                                                      \
		.name = _name, .read = adrv903x_phy_lo_read,                   \
		.write = adrv903x_phy_lo_write, .private = _ident,             \
	}

static const struct iio_chan_spec_ext_info adrv903x_phy_ext_lo_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADRV903X_EXT_LO_INFO("frequency", LOEXT_FREQ),
	{},
};

static ssize_t adrv903x_phy_rx_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adrv903x_rf_phy *phy = iio_priv(indio_dev);

	adi_adrv903x_TrackingCalibrationMask_t calMask = 0;
	bool enable;
	int ret = 0;
	u32 mask;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&phy->lock);

	switch (private) {
	case RX_QEC:
		if (chan->channel <= CHAN_RX8) {
			mask = ADI_ADRV903X_RX0 << chan->channel;
			calMask = ADI_ADRV903X_TC_RX_QEC_MASK;

			ret = adi_adrv903x_TrackingCalsEnableSet(phy->palauDevice, calMask,
								mask,
								enable ? ADI_ADRV903X_TRACKING_CAL_ENABLE :
									ADI_ADRV903X_TRACKING_CAL_DISABLE);
			if (ret)
				ret = adrv903x_dev_err(phy);
		}
		break;
	case RX_DIG_DC:
		if (chan->channel <= CHAN_RX8) {
			mask = ADI_ADRV903X_RX0 << chan->channel;

			ret = adi_adrv903x_DigDcOffsetEnableSet(phy->palauDevice, mask,
								enable ? 0xFF : 0x00);
			if (ret)
				ret = adrv903x_dev_err(phy);
		}
		break;
	case RX_ADC:
		if (chan->channel <= CHAN_RX8) {
			mask = ADI_ADRV903X_RX0 << chan->channel;
			calMask = ADI_ADRV903X_TC_RX_ADC_MASK;

			ret = adi_adrv903x_TrackingCalsEnableSet(phy->palauDevice, calMask,
								mask,
								enable ? ADI_ADRV903X_TRACKING_CAL_ENABLE :
									ADI_ADRV903X_TRACKING_CAL_DISABLE);
			if (ret)
				ret = adrv903x_dev_err(phy);
		} else {
			mask = chan->channel - CHAN_RX8;
			calMask = ADI_ADRV903X_TC_ORX_ADC_MASK;

			ret = adi_adrv903x_TrackingCalsEnableSet(phy->palauDevice, calMask,
								mask,
								enable ? ADI_ADRV903X_TRACKING_CAL_ENABLE :
									ADI_ADRV903X_TRACKING_CAL_DISABLE);
			if (ret)
				ret = adrv903x_dev_err(phy);
		}
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	return ret ? ret : len;
}

static ssize_t adrv903x_phy_rx_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct adrv903x_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv903x_TrackingCalEnableMasks_t enableMasks;
	adi_adrv903x_TrackingCalibrationMask_t chan_cals;
	uint8_t isEnabled;
	int ret = 0;
	u32 mask;

	mutex_lock(&phy->lock);

	switch (private) {
	case RX_QEC:
		if (chan->channel <= CHAN_RX8) {
			ret = adi_adrv903x_TrackingCalsEnableGet(phy->palauDevice, &enableMasks);
			if (ret == 0) {
				chan_cals = enableMasks.enableMask[chan->channel];
				ret = sprintf(buf, "%d\n", !!(chan_cals & ADI_ADRV903X_TC_RX_QEC_MASK));
			}
		}
		break;
	case RX_DIG_DC:
		mask = ADI_ADRV903X_RX0 << chan->channel;
		ret = adi_adrv903x_DigDcOffsetEnableGet(phy->palauDevice, mask, &isEnabled);

		if (ret == 0)
			ret = sprintf(buf, "%d\n", isEnabled);
		else
			ret = adrv903x_dev_err(phy);
		break;
	case RX_RF_BANDWIDTH:
		if (chan->channel <= CHAN_RX8)
			ret = sprintf(buf, "%u\n",
				phy->adi_adrv903x_device.initExtract.rx.rxChannelCfg[chan->channel].rfBandwidth_kHz *
				1000);
		else
			ret = sprintf(buf, "%u\n",
				phy->adi_adrv903x_device.initExtract.orx.orxChannelCfg[chan->channel - CHAN_OBS_RX1].rfBandwidth_kHz *
				1000);
		break;
	case RX_ADC:
		if (chan->channel <= CHAN_RX8) {
			ret = adi_adrv903x_TrackingCalsEnableGet(phy->palauDevice, &enableMasks);
			if (ret == 0) {
				chan_cals = enableMasks.enableMask[chan->channel];
				ret = sprintf(buf, "%d\n", !!(chan_cals & ADI_ADRV903X_TC_RX_ADC_MASK));
			}
		} else {
			ret = adi_adrv903x_TrackingCalsEnableGet(phy->palauDevice, &enableMasks);
			if (ret == 0) {
				chan_cals = enableMasks.enableMask[chan->channel - CHAN_OBS_RX1];
				ret = sprintf(buf, "%d\n", !!(chan_cals & ADI_ADRV903X_TC_ORX_ADC_MASK));
			}
		}
		break;
	case RX_QEC_STATUS:
		if (chan->channel <= CHAN_RX8) {
			adi_adrv903x_CalStatus_t calStatus = { 0 };
			mask = ADI_ADRV903X_RX0 << chan->channel;

			ret = adi_adrv903x_TrackingCalStatusGet(phy->palauDevice, ADI_ADRV903X_TC_RX_QEC_MASK, mask, &calStatus);
			if (ret == 0) {
				ret = sprintf(buf, "err %d %% %d perf %d iter cnt %d update cnt %d\n", calStatus.errorCode, calStatus.percentComplete,
					calStatus.performanceMetric, calStatus.iterCount, calStatus.updateCount);
			}
		}
		break;
	case RX_ADC_STATUS:
		if (chan->channel <= CHAN_RX8) {
			adi_adrv903x_CalStatus_t calStatus = { 0 };
			mask = ADI_ADRV903X_RX0 << chan->channel;

			ret = adi_adrv903x_TrackingCalStatusGet(phy->palauDevice, ADI_ADRV903X_TC_RX_ADC_MASK, mask, &calStatus);
			if (ret == 0) {
				ret = sprintf(buf, "err %d %% %d perf %d iter cnt %d update cnt %d\n", calStatus.errorCode, calStatus.percentComplete,
					calStatus.performanceMetric, calStatus.iterCount, calStatus.updateCount);
			}
		}
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	return ret;
}

#define _ADRV903X_EXT_RX_INFO(_name, _ident)                                   \
	{                                                                      \
		.name = _name, .read = adrv903x_phy_rx_read,                   \
		.write = adrv903x_phy_rx_write, .private = _ident,             \
	}

static ssize_t adrv903x_phy_tx_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct adrv903x_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv903x_TrackingCalEnableMasks_t enableMasks[ADI_ADRV903X_NUM_TRACKING_CAL_CHANNELS] = { 0 };
	adi_adrv903x_TrackingCalibrationMask_t chan_cals;
	adi_adrv903x_CalStatus_t calStatus = { 0 };
	int val, ret = 0, ret_s = 0;

	if (chan->channel > CHAN_TX8)
		return -EINVAL;

	mutex_lock(&phy->lock);
	switch (private) {
	case TX_QEC:
		ret = adi_adrv903x_TrackingCalsEnableGet(phy->palauDevice, enableMasks);
		if (ret == 0) {
			chan_cals = enableMasks->enableMask[chan->channel];
			val = !!(chan_cals & ADI_ADRV903X_TC_TX_QEC_MASK);
		}
		break;
	case TX_LOL:
		ret = adi_adrv903x_TrackingCalsEnableGet(phy->palauDevice, enableMasks);
		if (ret == 0) {
			chan_cals = enableMasks->enableMask[chan->channel];
			val = !!(chan_cals & ADI_ADRV903X_TC_TX_LOL_MASK);
		}
		break;
	case TX_RF_BANDWIDTH:
		val = phy->adi_adrv903x_device.initExtract.tx.txChannelCfg[chan->channel]
			      .rfBandwidth_kHz * 1000;
		break;
	case TX_LB_ADC:
		ret = adi_adrv903x_TrackingCalsEnableGet(phy->palauDevice, enableMasks);
		if (ret == 0) {
			chan_cals = enableMasks->enableMask[chan->channel];
			val = !!(chan_cals & ADI_ADRV903X_TC_TX_LB_ADC_MASK);
		}
		break;
	case TX_QEC_STATUS:
		ret = adi_adrv903x_TrackingCalStatusGet(phy->palauDevice, ADI_ADRV903X_TC_TX_QEC_MASK, ADI_ADRV903X_TX0 << chan->channel, &calStatus);
		if (ret == 0)
			ret_s = sprintf(buf, "err %d %% %d perf %d iter cnt %d update cnt %d\n", ret, calStatus.percentComplete,
				calStatus.performanceMetric, calStatus.iterCount, calStatus.updateCount);
		break;
	case TX_LOL_STATUS:
		ret = adi_adrv903x_TrackingCalStatusGet(phy->palauDevice, ADI_ADRV903X_TC_TX_LOL_MASK, ADI_ADRV903X_TX0 << chan->channel, &calStatus);
		if (ret == 0)
			ret_s = sprintf(buf, "err %d %% %d perf %d iter cnt %d update cnt %d\n", ret, calStatus.percentComplete,
				calStatus.performanceMetric, calStatus.iterCount, calStatus.updateCount);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	if (ret == 0)
		if (private != TX_QEC_STATUS && private != TX_LOL_STATUS)
			ret = sprintf(buf, "%d\n", val);

		else
			ret = ret_s;
	else
		return adrv903x_dev_err(phy);

	return ret;
}

static ssize_t adrv903x_phy_tx_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	adi_adrv903x_TrackingCalibrationMask_t calMask = 0;
	struct adrv903x_rf_phy *phy = iio_priv(indio_dev);
	bool enable;
	int ret = 0;
	u64 mask;

	if (chan->channel > CHAN_TX8)
		return -EINVAL;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&phy->lock);

	switch (private) {
	case TX_QEC:
		mask = ADI_ADRV903X_TX0 << chan->channel;
		calMask = ADI_ADRV903X_TC_TX_QEC_MASK;

		ret = adi_adrv903x_TrackingCalsEnableSet(phy->palauDevice, calMask,
							mask,
                                                        enable ? ADI_ADRV903X_TRACKING_CAL_ENABLE :
								 ADI_ADRV903X_TRACKING_CAL_DISABLE);
		if (ret)
			ret = adrv903x_dev_err(phy);
		break;
	case TX_LOL:
		mask = ADI_ADRV903X_TX0 << chan->channel;
		calMask = ADI_ADRV903X_TC_TX_LOL_MASK;

		adi_adrv903x_ChannelTrackingCals_t channelMask = { 0 };
		channelMask.txChannel = mask;

		ret = adi_adrv903x_TxToOrxMappingSet(phy->palauDevice, 0x50);
		if (ret)
			ret = adrv903x_dev_err(phy);

		ret = adi_adrv903x_TrackingCalsEnableSet(phy->palauDevice, calMask,
							mask,
                                                        enable ? ADI_ADRV903X_TRACKING_CAL_ENABLE :
								 ADI_ADRV903X_TRACKING_CAL_DISABLE);

		if (ret)
			ret = adrv903x_dev_err(phy);
		break;
	case TX_LB_ADC:
		mask = ADI_ADRV903X_TX0 << chan->channel;
		calMask = ADI_ADRV903X_TC_TX_LB_ADC_MASK;

		ret = adi_adrv903x_TrackingCalsEnableSet(phy->palauDevice, calMask,
							mask,
                                                        enable ? ADI_ADRV903X_TRACKING_CAL_ENABLE :
								 ADI_ADRV903X_TRACKING_CAL_DISABLE);
		if (ret)
			ret = adrv903x_dev_err(phy);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	return ret ? ret : len;
}

#define _ADRV903X_EXT_TX_INFO(_name, _ident)                                   \
	{                                                                      \
		.name = _name, .read = adrv903x_phy_tx_read,                   \
		.write = adrv903x_phy_tx_write, .private = _ident,             \
	}

static const struct iio_chan_spec_ext_info adrv903x_phy_rx_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADRV903X_EXT_RX_INFO("quadrature_tracking_en", RX_QEC),
	_ADRV903X_EXT_RX_INFO("bb_dc_offset_tracking_en", RX_DIG_DC),
	_ADRV903X_EXT_RX_INFO("rf_bandwidth", RX_RF_BANDWIDTH),
	_ADRV903X_EXT_RX_INFO("adc_tracking_en", RX_ADC),
	_ADRV903X_EXT_RX_INFO("quadrature_tracking_status", RX_QEC_STATUS),
	_ADRV903X_EXT_RX_INFO("adc_tracking_status", RX_ADC_STATUS),
	{},
};

static const struct iio_chan_spec_ext_info adrv903x_phy_obs_rx_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADRV903X_EXT_RX_INFO("rf_bandwidth", RX_RF_BANDWIDTH),
	_ADRV903X_EXT_RX_INFO("adc_tracking_en", RX_ADC),
	{},
};

static struct iio_chan_spec_ext_info adrv903x_phy_tx_ext_info[] = {
	_ADRV903X_EXT_TX_INFO("quadrature_tracking_en", TX_QEC),
	_ADRV903X_EXT_TX_INFO("lo_leakage_tracking_en", TX_LOL),
	_ADRV903X_EXT_TX_INFO("rf_bandwidth", TX_RF_BANDWIDTH),
	_ADRV903X_EXT_TX_INFO("loopback_adc_tracking_en", TX_LB_ADC),
	_ADRV903X_EXT_TX_INFO("quadrature_tracking_status", TX_QEC_STATUS),
	_ADRV903X_EXT_TX_INFO("lo_leakage_tracking_status", TX_LOL_STATUS),
	{},
};

static int adrv903x_gainindex_to_gain(struct adrv903x_rf_phy *phy, int channel,
				      unsigned int index, int *val, int *val2)
{
	int code;

	code = MAX_RX_GAIN_mdB - (255 - index) * RX_GAIN_STEP_mdB;

	*val = code / 1000;
	*val2 = (code % 1000) * 1000;
	if (!*val)
		*val2 *= -1;

	return 0;
}

static int adrv903x_gain_to_gainindex(struct adrv903x_rf_phy *phy, int channel,
				      int val, int val2, unsigned int *index)
{
	int gain = ((abs(val) * 1000) + (abs(val2) / 1000));

	gain = clamp(gain, MIN_GAIN_mdB, MAX_RX_GAIN_mdB);
	*index = (gain - MAX_RX_GAIN_mdB) / RX_GAIN_STEP_mdB + 255;

	return 0;
}

static int adrv903x_phy_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, int *val,
				 int *val2, long m)
{
	adi_adrv903x_DevTempSensorMask_e avg_mask = ADI_ADRV903X_DEVTEMP_MASK_TX0;
	struct adrv903x_rf_phy *phy = iio_priv(indio_dev);
	u32 orxchan = 0, rxchan = 0, txchan = 0;
	adi_adrv903x_DevTempData_t temp = { 0 };
	int ret;

	mutex_lock(&phy->lock);
	switch (m) {
	case IIO_CHAN_INFO_ENABLE:

		ret = adi_adrv903x_RxTxEnableGet(phy->palauDevice, &orxchan, &rxchan,
						 &txchan);
		if (ret) {
			ret = adrv903x_dev_err(phy);
			break;
		}

		if (chan->output)
			*val = !!(txchan & (ADI_ADRV903X_TX0 << chan->channel));
		else if (chan->channel <= CHAN_RX8)
			*val = !!(rxchan & (ADI_ADRV903X_RX0 << chan->channel));
		else
			*val = !!(orxchan & (ADI_ADRV903X_RX0 << chan->channel));

		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			adi_adrv903x_TxAtten_t txAtten;

			ret = adi_adrv903x_TxAttenGet(
				phy->palauDevice, 1 << chan->channel, &txAtten);
			if (ret) {
				ret = adrv903x_dev_err(phy);
				break;
			}

			*val = -1 * (txAtten.txAttenuation_mdB / 1000);
			*val2 = (txAtten.txAttenuation_mdB % 1000) * 1000;
			if (!*val)
				*val2 *= -1;

		} else {
			if (chan->channel <= CHAN_RX8) {
				adi_adrv903x_RxGain_t rxGain;

				ret = adi_adrv903x_RxGainGet(
					phy->palauDevice, 1 << chan->channel, &rxGain);
				if (ret) {
					ret = adrv903x_dev_err(phy);
					break;
				}

				ret = adrv903x_gainindex_to_gain(phy, chan->channel,
								rxGain.gainIndex, val,
								val2);
			} else {
				uint8_t attenDb = 0;
				ret = adi_adrv903x_OrxAttenGet(phy->palauDevice, chan->channel - CHAN_OBS_RX1, &attenDb);
				if (ret) {
					ret = adrv903x_dev_err(phy);
					break;
				}
				*val = attenDb;
			}
		}
		ret = IIO_VAL_INT_PLUS_MICRO_DB;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (chan->channel) {
			case CHAN_RX1:
			case CHAN_RX2:
			case CHAN_RX3:
			case CHAN_RX4:
			case CHAN_RX5:
			case CHAN_RX6:
			case CHAN_RX7:
			case CHAN_RX8:
				*val = clk_get_rate(phy->clks[RX_SAMPL_CLK]);
				break;
			case CHAN_OBS_RX1:
			case CHAN_OBS_RX2:
				*val = clk_get_rate(phy->clks[OBS_SAMPL_CLK]);
				break;
			}

		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_PROCESSED:
		// ToDo variable avg_mask
		adi_adrv903x_TemperatureGet(phy->palauDevice, avg_mask, &temp);
		*val = (temp.tempDegreesCelsius[avg_mask]* 1000);
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	return ret;
};

static int adrv903x_phy_write_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan, int val,
				  int val2, long mask)
{
	u32 orxchan_msk = 0, rxchan_msk = 0, txchan_msk = 0;
	struct adrv903x_rf_phy *phy = iio_priv(indio_dev);
	u32 orxchan_en = 0, rxchan_en = 0, txchan_en = 0;
		int ret = 0;
	u32 code;

	mutex_lock(&phy->lock);
	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		ret = adi_adrv903x_RxTxEnableGet(phy->palauDevice, &orxchan_msk, &rxchan_msk,
						 &txchan_msk);
		if (ret) {
			ret = adrv903x_dev_err(phy);
			goto out;
		}

		txchan_msk = 0;
		rxchan_msk = 0;
		orxchan_msk = 0;
		if (chan->output) {
			txchan_msk = (1 << chan->channel);
			if (val)
				txchan_en |= (ADI_ADRV903X_TX0 << chan->channel);
			else
				txchan_en &= ~(ADI_ADRV903X_TX0 << chan->channel);
		} else {
			if (chan->channel <= CHAN_RX8) {
				rxchan_msk = (1 << chan->channel);
				if (val)
					rxchan_en |= (ADI_ADRV903X_RX0 << chan->channel);
				else
					rxchan_en &= ~(ADI_ADRV903X_RX0 << chan->channel);
			} else {
				orxchan_msk = (1 << chan->channel);
				if (val)
					orxchan_en |= (ADI_ADRV903X_RX0 << chan->channel);
				else
					orxchan_en &= ~(ADI_ADRV903X_RX0 << chan->channel);
			}
		}
		ret = adi_adrv903x_RxTxEnableSet(phy->palauDevice, orxchan_msk, orxchan_en, rxchan_msk, rxchan_en,
						 txchan_msk, txchan_en);
		if (ret)
			ret = adrv903x_dev_err(phy);
		break;

	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			adi_adrv903x_TxAtten_t txAtten;

			if (val > 0 || (val == 0 && val2 > 0)) {
				ret = -EINVAL;
				goto out;
			}

			code = ((abs(val) * 1000) + (abs(val2) / 1000));

			txAtten.txChannelMask = 1 << chan->channel;
			txAtten.txAttenuation_mdB =
				code; /* Back off Tx output power by 30dB */
			ret = adi_adrv903x_TxAttenSet(phy->palauDevice, &txAtten,
						      1);
			if (ret)
				adrv903x_dev_err(phy);

		} else {
			if (chan->channel <= CHAN_RX8) {
				adi_adrv903x_RxGain_t rxGain;

				ret = adrv903x_gain_to_gainindex(phy, chan->channel,
								val, val2, &code);
				if (ret < 0)
					break;

				if (chan->channel <= CHAN_RX8) {
					rxGain.gainIndex = code;
					rxGain.rxChannelMask = 1 << chan->channel;

					ret = adi_adrv903x_RxGainSet(phy->palauDevice, &rxGain,
								1);
					if (ret)
						adrv903x_dev_err(phy);
				}
			} else {
				uint8_t attenDb = val;
				ret = adi_adrv903x_OrxAttenSet(phy->palauDevice, chan->channel - CHAN_RX8, attenDb);
				if (ret)
					adrv903x_dev_err(phy);
			}
		}
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		break;
	default:
		ret = -EINVAL;
	}
out:
	mutex_unlock(&phy->lock);

	return ret;
}

static const struct iio_chan_spec adrv903x_phy_chan[] = {
	{
		/* LO1 */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.extend_name = "LO1",
		.ext_info = adrv903x_phy_ext_lo_info,
	},
	{
		/* LO2 */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.extend_name = "LO2",
		.ext_info = adrv903x_phy_ext_lo_info,
	},
	{
		/* TX1 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_TX1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_tx_ext_info,
	},
	{
		/* RX1 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_rx_ext_info,
	},
	{
		/* TX2 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_TX2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_tx_ext_info,
	},
	{
		/* RX2 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_rx_ext_info,
	},
	{
		/* TX3 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_TX3,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_tx_ext_info,
	},
	{
		/* RX3 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX3,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_rx_ext_info,
	},
	{
		/* TX4 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_TX4,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_tx_ext_info,
	},
	{
		/* RX4 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX4,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_rx_ext_info,
	},
	{
		/* TX5 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_TX5,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_tx_ext_info,
	},
	{
		/* RX5 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX5,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_rx_ext_info,
	},
	{
		/* TX6 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_TX6,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_tx_ext_info,
	},
	{
		/* RX6 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX6,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_rx_ext_info,
	},
	{
		/* TX7 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_TX7,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_tx_ext_info,
	},
	{
		/* RX7 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX7,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_rx_ext_info,
	},
	{
		/* TX8 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_TX8,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_tx_ext_info,
	},
	{
		/* RX8 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX8,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_rx_ext_info,
	},
	{
		/* RX Sniffer/Observation */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_OBS_RX1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_obs_rx_ext_info,
	},

	{
		/* RX Sniffer/Observation */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_OBS_RX2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv903x_phy_obs_rx_ext_info,
	},
	{
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static const struct iio_info adrv903x_phy_info = {
	.read_raw = &adrv903x_phy_read_raw,
	.write_raw = &adrv903x_phy_write_raw,
	.debugfs_reg_access = &adrv903x_phy_reg_access,
	.attrs = &adrv903x_phy_attribute_group,
};

static ssize_t adrv903x_debugfs_read(struct file *file, char __user *userbuf,
				     size_t count, loff_t *ppos)
{
	struct adrv903x_debugfs_entry *entry = file->private_data;
	struct adrv903x_rf_phy *phy = entry->phy;
	adi_adrv903x_TxTestNcoConfigReadbackResp_t txRbConfig = { 0 };
	char buf[700];
	u64 val = 0;
	ssize_t len = 0;
	int ret;

	if (entry->cmd == DBGFS_BIST_TONE_GET) {
		u32 orxchan = 0, rxchan = 0, txchan = 0;
		int i;
		int tx_enabled;

		len = snprintf(buf, sizeof(buf), "BIST Tone Configuration:\n\n");

		mutex_lock(&phy->lock);

		/* Get TX channel enable status */
		ret = adi_adrv903x_RxTxEnableGet(phy->palauDevice, &orxchan, &rxchan, &txchan);
		if (ret) {
			mutex_unlock(&phy->lock);
			dev_err(&phy->spi->dev, "Failed to get RxTx enable status: %d\n", ret);
			return -EIO;
		}

		/* Loop through all channels 0-7 */
		for (i = 0; i < 8; i++) {
			/* Set which channel to query */
			txRbConfig.chanSelect = ADI_ADRV903X_TX0 << i;

			/* Query this channel's configuration */
			ret = adi_adrv903x_TxTestToneGet(phy->palauDevice, &txRbConfig);
			if (ret) {
				mutex_unlock(&phy->lock);
				dev_err(&phy->spi->dev, "Failed to get BIST tone config for channel %d: %d\n", i, ret);
				return -EIO;
			}

			/* Check if this TX channel is enabled */
			tx_enabled = !!(txchan & (ADI_ADRV903X_TX0 << i));

			/* Append this channel's info to buffer */
			len += snprintf(buf + len, sizeof(buf) - len,
				"Ch%d 0x%02x TxEn:%d\n"
				" En %u, NCO %u, %d kHz, %u deg, %u (%d dB)\n\n",
				i,
				txRbConfig.chanSelect,
				tx_enabled,
				txRbConfig.enabled,
				txRbConfig.ncoSelect,
				txRbConfig.frequencyKhz,
				txRbConfig.phase,
				txRbConfig.attenCtrl,
				txRbConfig.attenCtrl * 6);

			/* Check if buffer is getting full */
			if (len >= sizeof(buf) - 100) {
				len += snprintf(buf + len, sizeof(buf) - len,
					"... (buffer full, remaining channels truncated)\n");
				break;
			}
		}

		mutex_unlock(&phy->lock);
		return simple_read_from_buffer(userbuf, count, ppos, buf, len);
	}

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
			ret = -EINVAL;
		}

	} else if (entry->cmd)
		val = entry->val;
	else
		return -EFAULT;

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

	count = min_t(size_t, count, (sizeof(buf) - 1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;

	ret = sscanf(buf, "%lli %i %i %i", &val, &val2, &val3, &val4);
	if (ret < 1)
		return -EINVAL;

	switch (entry->cmd) {
	case DBGFS_BIST_FRAMER_0_PRBS:
		mutex_lock(&phy->lock);

		frm_test_data.injectPoint = ADI_ADRV903X_FTD_FRAMERINPUT;
		frm_test_data.testDataSource = val;
		frm_test_data.framerSelMask = ADI_ADRV903X_FRAMER_0;

		ret = adi_adrv903x_FramerTestDataSet(phy->palauDevice,
						     &frm_test_data);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv903x_dev_err(phy);

		entry->val = val;
		return count;

	case DBGFS_BIST_FRAMER_1_PRBS:
		mutex_lock(&phy->lock);

		frm_test_data.injectPoint = ADI_ADRV903X_FTD_FRAMERINPUT;
		frm_test_data.testDataSource = val;
		frm_test_data.framerSelMask = ADI_ADRV903X_FRAMER_1;

		ret = adi_adrv903x_FramerTestDataSet(phy->palauDevice,
						     &frm_test_data);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv903x_dev_err(phy);

		entry->val = val;
		return count;
	case DBGFS_BIST_FRAMER_0_LOOPBACK:
		mutex_lock(&phy->lock);
		ret = adi_adrv903x_FramerLoopbackSet(phy->palauDevice, ADI_ADRV903X_FRAMER_0);
		mutex_unlock(&phy->lock);
		if (ret)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_BIST_FRAMER_1_LOOPBACK:
		mutex_lock(&phy->lock);
		ret = adi_adrv903x_FramerLoopbackSet(phy->palauDevice, ADI_ADRV903X_FRAMER_1);
		mutex_unlock(&phy->lock);
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

		mutex_lock(&phy->lock);
		ret = adi_adrv903x_TxTestToneSet(phy->palauDevice, &txNcoConfig);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv903x_dev_err(phy);

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
			ret = -EINVAL;
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

	if (WARN_ON(i >= ARRAY_SIZE(phy->debugfs_entry)))
		return;

	phy->debugfs_entry[i].phy = phy;
	phy->debugfs_entry[i].propname = propname;
	phy->debugfs_entry[i].cmd = cmd;

	phy->adrv903x_debugfs_entry_index++;
}

static int adrv903x_register_debugfs(struct iio_dev *indio_dev)
{
	struct adrv903x_rf_phy *phy = iio_priv(indio_dev);
	struct dentry *d;
	int i;

	if (!iio_get_debugfs_dentry(indio_dev))
		return -ENODEV;

	adrv903x_add_debugfs_entry(phy, "bist_framer_0_prbs",
				   DBGFS_BIST_FRAMER_0_PRBS);
	adrv903x_add_debugfs_entry(phy, "bist_framer_1_prbs",
				   DBGFS_BIST_FRAMER_1_PRBS);
	adrv903x_add_debugfs_entry(phy, "bist_framer_0_loopback",
				   DBGFS_BIST_FRAMER_0_LOOPBACK);
	adrv903x_add_debugfs_entry(phy, "bist_framer_1_loopback",
				   DBGFS_BIST_FRAMER_1_LOOPBACK);
	adrv903x_add_debugfs_entry(phy, "bist_tone", DBGFS_BIST_TONE);
	adrv903x_add_debugfs_entry(phy, "bist_tone_get", DBGFS_BIST_TONE_GET);

	for (i = 0; i < phy->adrv903x_debugfs_entry_index; i++) {
		/* bist_tone_get is read-only */
		if (phy->debugfs_entry[i].cmd == DBGFS_BIST_TONE_GET)
			d = debugfs_create_file(phy->debugfs_entry[i].propname, 0444,
						iio_get_debugfs_dentry(indio_dev),
						&phy->debugfs_entry[i],
						&adrv903x_debugfs_reg_fops);
		else
			d = debugfs_create_file(phy->debugfs_entry[i].propname, 0644,
						iio_get_debugfs_dentry(indio_dev),
						&phy->debugfs_entry[i],
						&adrv903x_debugfs_reg_fops);
	}
	return 0;
}

#define ADRV903X_MAX_CLK_NAME 79

static char *adrv903x_clk_set_dev_name(struct adrv903x_rf_phy *phy, char *dest,
				       const char *name)
{
	size_t len = 0;

	if (name == NULL)
		return NULL;

	if (*name == '-')
		len = strscpy(dest, dev_name(&phy->spi->dev),
			      ADRV903X_MAX_CLK_NAME);
	else
		*dest = '\0';

	return strncat(dest, name, ADRV903X_MAX_CLK_NAME - len);
}

static unsigned long adrv903x_bb_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct adrv903x_clock *clk_priv = to_clk_priv(hw);

	return clk_priv->rate;
}

static int adrv903x_bb_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct adrv903x_clock *clk_priv = to_clk_priv(hw);

	clk_priv->rate = rate;

	return 0;
}

static long adrv903x_bb_round_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long *prate)
{
	struct adrv903x_clock *clk_priv = to_clk_priv(hw);

	dev_dbg(&clk_priv->spi->dev, "%s: Rate %lu Hz", __func__, rate);

	return rate;
}

static const struct clk_ops bb_clk_ops = {
	.round_rate = adrv903x_bb_round_rate,
	.set_rate = adrv903x_bb_set_rate,
	.recalc_rate = adrv903x_bb_recalc_rate,
};

static int adrv903x_clk_register(struct adrv903x_rf_phy *phy, const char *name,
				 const char *parent_name,
				 const char *parent_name2, unsigned long flags,
				 u32 source)
{
	struct adrv903x_clock *clk_priv = &phy->clk_priv[source];
	struct clk_init_data init;
	struct clk *clk;
	char c_name[ADRV903X_MAX_CLK_NAME + 1],
		p_name[2][ADRV903X_MAX_CLK_NAME + 1];
	const char *_parent_name[2];
	u32 rate;

	/* struct adrv903x_clock assignments */
	clk_priv->source = source;
	clk_priv->hw.init = &init;
	clk_priv->spi = phy->spi;
	clk_priv->phy = phy;

	_parent_name[0] =
		adrv903x_clk_set_dev_name(phy, p_name[0], parent_name);
	_parent_name[1] =
		adrv903x_clk_set_dev_name(phy, p_name[1], parent_name2);

	init.name = adrv903x_clk_set_dev_name(phy, c_name, name);
	init.flags = flags;
	init.parent_names = &_parent_name[0];
	init.num_parents = _parent_name[1] ? 2 : _parent_name[0] ? 1 : 0;


	switch (source) {
	case RX_SAMPL_CLK:
		phy->rx_iqRate_kHz = phy->palauDevice->initExtract.rx.rxChannelCfg[0].rxDdc0OutputRate_kHz;
		init.ops = &bb_clk_ops;
		clk_priv->rate = phy->rx_iqRate_kHz;
		break;
	case OBS_SAMPL_CLK:
		phy->orx_iqRate_kHz = phy->palauDevice->initExtract.jesdSetting.framerSetting[1].iqRate_kHz;
		init.ops = &bb_clk_ops;
		clk_priv->rate = phy->orx_iqRate_kHz;;
		break;
	case TX_SAMPL_CLK:
		adrv903x_TxLinkSamplingRateFind(phy->palauDevice,
						ADI_ADRV903X_DEFRAMER_0,
						&rate);
		init.ops = &bb_clk_ops;
		clk_priv->rate = rate;
		break;
	default:
		return -EINVAL;
	}

	clk_priv->rate *= 1000;

	clk = devm_clk_register(&phy->spi->dev, &clk_priv->hw);
	phy->clks[source] = clk;

	return 0;
}

static void adrv903x_info(struct adrv903x_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	adi_adrv903x_Version_t apiVersion;
	uint8_t siRevision = 0xbb;

	adi_adrv903x_ApiVersionGet(phy->palauDevice, &apiVersion);
	adi_adrv903x_DeviceRevGet(phy->palauDevice, &siRevision);

	dev_info(&spi->dev, "\n%s Rev %d, API version: %u.%u.%u.%u successfully initialized%s",
		spi_get_device_id(spi)->name,
		phy->palauDevice->devStateInfo.deviceSiRev,
		apiVersion.majorVer, apiVersion.minorVer,
		apiVersion.maintenanceVer, apiVersion.buildVer,
		phy->jdev ? " via jesd204-fsm" : "");
}

struct adrv903x_jesd204_link {
	unsigned int source_id;
	bool is_framer;
};

struct adrv903x_jesd204_priv {
	struct adrv903x_rf_phy *phy;
	struct adrv903x_jesd204_link link[5];
};

static int adrv903x_jesd204_device_init(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason)
{
	adi_adrv903x_ErrAction_e recoveryAction = ADI_ADRV903X_ERR_ACT_NONE;
	struct adrv903x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv903x_rf_phy *phy = priv->phy;

	dev_dbg(dev, "%s:%d device init %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	ADI_LIBRARY_MEMSET(&phy->adi_adrv903x_device.devStateInfo, 0,
	       sizeof(phy->adi_adrv903x_device.devStateInfo));

	recoveryAction = adi_adrv903x_HwOpen(phy->palauDevice, &phy->spiSettings);
	if (recoveryAction != ADI_ADRV903X_ERR_ACT_NONE) {
		pr_err("ERROR adi_adrv903x_HwOpen failed in %s at line %d.\n", __func__,
		       __LINE__);
		return JESD204_STATE_CHANGE_ERROR;
	}

	adi_adrv903x_LogLevelSet(&phy->palauDevice->common,
			       ADI_HAL_LOG_ERR | ADI_HAL_LOG_WARN);

	recoveryAction = adi_adrv903x_HwReset(phy->palauDevice);

	recoveryAction = adi_adrv903x_PreMcsInit(phy->palauDevice, &deviceInitStruct,
			 &phy->trxBinaryInfoPtr);
	if (recoveryAction != ADI_ADRV903X_ERR_ACT_NONE) {
		pr_err("ERROR adi_adrv903x_PreMcsInit failed in %s at line %d.\n", __func__,
		       __LINE__);
		return JESD204_STATE_CHANGE_ERROR;
	}

	recoveryAction = adi_adrv903x_PreMcsInit_NonBroadcast(phy->palauDevice,
			 &deviceInitStruct);
	if (recoveryAction != ADI_ADRV903X_ERR_ACT_NONE) {
		pr_err("ERROR adi_adrv903x_PreMcsInit_NonBroadcast failed in %s at line %d.\n",
		       __func__, __LINE__);
		return JESD204_STATE_CHANGE_ERROR;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv903x_jesd204_link_init(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	adi_adrv903x_ErrAction_e recoveryAction = ADI_ADRV903X_ERR_ACT_NONE;
	struct adrv903x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv903x_rf_phy *phy = priv->phy;
	uint8_t source_id;
	u32 rate;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		 lnk->link_id, jesd204_state_op_reason_str(reason));

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	switch (lnk->link_id) {
	case DEFRAMER0_LINK_TX:
	case DEFRAMER1_LINK_TX:
		adi_adrv903x_DeframerCfg_t deframerCfg = { 0 };

		if (lnk->link_id == DEFRAMER0_LINK_TX)
			source_id = ADI_ADRV903X_DEFRAMER_0;
		else
			source_id = ADI_ADRV903X_DEFRAMER_1;

		recoveryAction = adi_adrv903x_DeframerCfgGet(phy->palauDevice, source_id, &deframerCfg);
		if (recoveryAction != ADI_ADRV903X_ERR_ACT_NONE) {
			pr_err("ERROR adi_adrv903x_DeframerCfgGet failed in %s at line %d.\n", __func__,
				   __LINE__);
			return JESD204_STATE_CHANGE_ERROR;
		}

		priv->link[lnk->link_id].source_id = source_id;
		phy->tx_iqRate_kHz  =
			phy->palauDevice->initExtract.jesdSetting.deframerSetting[source_id - 1].iqRate_kHz;
		rate = phy->palauDevice->initExtract.jesdSetting.deframerSetting[source_id - 1].iqRate_kHz;
		lnk->num_lanes = hweight8(
					 phy->palauDevice->initExtract.jesdSetting.deframerSetting[source_id - 1].deserialLaneEnabled);
		lnk->num_converters =
			phy->palauDevice->initExtract.jesdSetting.deframerSetting[source_id - 1].jesdM;
		lnk->bits_per_sample =
			phy->palauDevice->initExtract.jesdSetting.deframerSetting[source_id - 1].jesdNp;

		lnk->octets_per_frame = deframerCfg.jesd204F;
		lnk->frames_per_multiframe = deframerCfg.jesd204K;
		lnk->device_id = deframerCfg.deviceId;
		lnk->bank_id = deframerCfg.bankId;
		lnk->scrambling = deframerCfg.decrambling;
		lnk->converter_resolution = deframerCfg.jesd204Np;
		lnk->ctrl_bits_per_sample = 0; // ToDo - from where
		lnk->jesd_version = deframerCfg.enableJesd204C ? JESD204_VERSION_C :
						    JESD204_VERSION_B;
		lnk->subclass = JESD204_SUBCLASS_1; // ToDo - from where
		lnk->is_transmit = true;
		lnk->jesd_encoder = JESD204_ENCODER_64B66B;
		break;
	case FRAMER0_LINK_RX:
	case FRAMER1_LINK_RX:
	case FRAMER2_LINK_RX:
		adi_adrv903x_FramerCfg_t framerCfg = { 0 };

		if (lnk->link_id == FRAMER0_LINK_RX)
			source_id = ADI_ADRV903X_FRAMER_0;
		else if (lnk->link_id == FRAMER1_LINK_RX)
			source_id = ADI_ADRV903X_FRAMER_1;
		else
			source_id = ADI_ADRV903X_FRAMER_2;

		recoveryAction = adi_adrv903x_FramerCfgGet(phy->palauDevice, source_id, &framerCfg);
		if (recoveryAction != ADI_ADRV903X_ERR_ACT_NONE) {
			pr_err("ERROR adi_adrv903x_FramerCfgGet failed in %s at line %d.\n", __func__,
				   __LINE__);
			return JESD204_STATE_CHANGE_ERROR;
		}

		priv->link[lnk->link_id].source_id = source_id;
		priv->link[lnk->link_id].is_framer = true;
		// phy->rx_iqRate_kHz  =
		// 	phy->palauDevice->initExtract.jesdSetting.framerSetting[source_id - 1].iqRate_kHz;
		rate = phy->palauDevice->initExtract.jesdSetting.framerSetting[source_id - 1].iqRate_kHz;
		if (lnk->link_id == FRAMER0_LINK_RX)
			phy->rx_iqRate_kHz = rate;
		if (lnk->link_id == FRAMER1_LINK_RX)
			phy->orx_iqRate_kHz = rate;
		lnk->num_lanes = hweight8(
					 phy->palauDevice->initExtract.jesdSetting.framerSetting[source_id - 1].serialLaneEnabled);
		lnk->num_converters =
			phy->palauDevice->initExtract.jesdSetting.framerSetting[source_id - 1].jesdM;
		lnk->bits_per_sample =
			phy->palauDevice->initExtract.jesdSetting.framerSetting[source_id - 1].jesdNp;

		lnk->octets_per_frame = framerCfg.jesd204F;
		lnk->frames_per_multiframe = framerCfg.jesd204K;
		lnk->device_id = framerCfg.deviceId;
		lnk->bank_id = framerCfg.bankId;
		lnk->scrambling = framerCfg.scramble;
		lnk->converter_resolution = framerCfg.jesd204Np;
		lnk->ctrl_bits_per_sample = 0;
		lnk->jesd_version = framerCfg.enableJesd204C ? JESD204_VERSION_C :
			    JESD204_VERSION_B;
		lnk->subclass = JESD204_SUBCLASS_1;
		lnk->is_transmit = false;
		lnk->jesd_encoder = JESD204_ENCODER_64B66B;

		break;
	default:
		return -EINVAL;
	}

	lnk->sample_rate = rate * 1000;

	return JESD204_STATE_CHANGE_DONE;
}

int adrv903x_jesd204_link_pre_setup(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason)
{
	struct adrv903x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv903x_rf_phy *phy = priv->phy;
	uint32_t deviceClockScaled_kHz = 0;
	long dev_clk;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	deviceClockScaled_kHz =
		phy->palauDevice->initExtract.clocks.deviceClockScaled_kHz;

	dev_clk = clk_round_rate(phy->dev_clk,
				 deviceClockScaled_kHz * 1000);

	if (dev_clk > 0 && ((dev_clk / 1000) ==
		deviceClockScaled_kHz)) {
		clk_set_rate(phy->dev_clk, (unsigned long) dev_clk);
	} else {
		dev_err(&phy->spi->dev,
			"Requesting device clock %u failed got %ld",
			deviceClockScaled_kHz * 1000, dev_clk);
		return -EINVAL;
	}

	return JESD204_STATE_CHANGE_DONE;
}

int adrv903x_jesd204_link_setup(struct jesd204_dev *jdev,
				enum jesd204_state_op_reason reason)
{
	struct adrv903x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv903x_rf_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	if (reason == JESD204_STATE_OP_REASON_UNINIT) {
		phy->is_initialized = 0;

		adi_adrv903x_Shutdown(phy->palauDevice);
		adi_adrv903x_HwClose(phy->palauDevice);

		memset(&phy->adi_adrv903x_device.devStateInfo, 0,
			sizeof(phy->adi_adrv903x_device.devStateInfo));

		return JESD204_STATE_CHANGE_DONE;
	}

	ret = adi_adrv903x_MultichipSyncSet_v2(phy->palauDevice, ADI_ENABLE);
	if (ret)
		return adrv903x_dev_err(phy);

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv903x_jesd204_setup_stage1(struct jesd204_dev *jdev,
					 enum jesd204_state_op_reason reason)
{
	struct adrv903x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv903x_rf_phy *phy = priv->phy;
	int ret, i;
	u32 mcsStatus;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	/* This loop will send SysRef pulses up to 255 times unless MCS status achieved before. */
	for (i = 0; i < 255; i++) {
		ret = adi_adrv903x_MultichipSyncStatusGet(phy->palauDevice,
							  &mcsStatus);
		if (ret)
			return adrv903x_dev_err(phy);

		if ((mcsStatus & 0x01) == 0x01)
			break;

		jesd204_sysref_async_force(phy->jdev);
	}

	if (mcsStatus != 0x01) {
		dev_err(&phy->spi->dev,
			"%s:%d Unexpected MCS sync status (0x%X)",
			__func__, __LINE__, mcsStatus);

		return adrv903x_dev_err(phy);
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv903x_jesd204_setup_stage2(struct jesd204_dev *jdev,
					 enum jesd204_state_op_reason reason)
{
	struct adrv903x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv903x_rf_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	/* MCS end sequence*/
	ret = adi_adrv903x_MultichipSyncSet_v2(phy->palauDevice, ADI_DISABLE);
	if (ret)
		return adrv903x_dev_err(phy);

	/* Post MCS */
	ret = adi_adrv903x_PostMcsInit(phy->palauDevice,
				       &utilityInit);
	if (ret)
		return adrv903x_dev_err(phy);

	ret = adi_adrv903x_SerializerReset(phy->palauDevice);
	if (ret)
		return adrv903x_dev_err(phy);

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv903x_jesd204_clks_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct adrv903x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv903x_rf_phy *phy = priv->phy;
	int ret;

	dev_err(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));


	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	if (!lnk->num_converters)
		return JESD204_STATE_CHANGE_DONE;

	if (priv->link[lnk->link_id].is_framer) {
		if (phy->palauDevice->devStateInfo.linkSharingEnabled == 1) {
			ret = adi_adrv903x_FramerSysrefCtrlSet(phy->palauDevice,
				ADI_ADRV903X_FRAMER_1, ADI_DISABLE);
			if (ret)
				return adrv903x_dev_err(phy);

			ret = adi_adrv903x_FramerLinkStateSet(phy->palauDevice,
				ADI_ADRV903X_FRAMER_1, ADI_DISABLE);
			if (ret)
				return adrv903x_dev_err(phy);

			ret = adi_adrv903x_FramerLinkStateSet(phy->palauDevice,
				ADI_ADRV903X_FRAMER_1, ADI_ENABLE);
			if (ret)
				return adrv903x_dev_err(phy);

			dev_dbg(&phy->spi->dev,
				"%s:%d Link %d Framer enabled", __func__, __LINE__,
				ADI_ADRV903X_FRAMER_1);

			/*************************************************/
			/***** Enable SYSREF to Koror JESD204B Framer ****/
			/*************************************************/
			/*** < User: Make sure SYSREF is stopped/disabled > ***/
			ret = adi_adrv903x_FramerSysrefCtrlSet(phy->palauDevice,
				ADI_ADRV903X_FRAMER_1, ADI_ENABLE);
			if (ret)
				return adrv903x_dev_err(phy);

			jesd204_sysref_async_force(phy->jdev);

			ret = adi_adrv903x_FramerLinkStateSet(phy->palauDevice,
				ADI_ADRV903X_FRAMER_1, ADI_DISABLE);
			if (ret)
				return adrv903x_dev_err(phy);

			ret = adi_adrv903x_FramerSysrefCtrlSet(phy->palauDevice,
				ADI_ADRV903X_FRAMER_1, ADI_DISABLE);
			if (ret)
				return adrv903x_dev_err(phy);

		}

		ret = adi_adrv903x_FramerSysrefCtrlSet(phy->palauDevice,
			priv->link[lnk->link_id].source_id, ADI_ENABLE);
		if (ret)
			return adrv903x_dev_err(phy);

		ret = adi_adrv903x_FramerLinkStateSet(phy->palauDevice,
			priv->link[lnk->link_id].source_id, ADI_DISABLE);
		if (ret)
			return adrv903x_dev_err(phy);


		ret = adi_adrv903x_FramerLinkStateSet(phy->palauDevice,
			priv->link[lnk->link_id].source_id, ADI_ENABLE);
		if (ret)
			return adrv903x_dev_err(phy);

		dev_dbg(&phy->spi->dev,
			"%s:%d Link %d Framer enabled", __func__, __LINE__,
			priv->link[lnk->link_id].source_id);

		ret = adi_adrv903x_FramerSysrefCtrlSet(phy->palauDevice,
			priv->link[lnk->link_id].source_id, ADI_DISABLE);
		if (ret)
			return adrv903x_dev_err(phy);

	} else {
		ret = adi_adrv903x_DeframerSysrefCtrlSet(phy->palauDevice,
			(uint8_t) ADI_ADRV903X_ALL_DEFRAMER, ADI_DISABLE);
		if (ret)
			return adrv903x_dev_err(phy);


		ret = adi_adrv903x_DeframerLinkStateSet(phy->palauDevice,
			(uint8_t) ADI_ADRV903X_ALL_DEFRAMER, ADI_DISABLE);
		if (ret)
			return adrv903x_dev_err(phy);

	};

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv903x_jesd204_link_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct adrv903x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv903x_rf_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	if (!lnk->num_converters)
		return JESD204_STATE_CHANGE_DONE;

	if (!priv->link[lnk->link_id].is_framer) { /* DEFRAMER */
		adi_adrv903x_InitCals_t serdesCal = {
			.calMask = ADI_ADRV903X_IC_SERDES,
			.orxChannelMask = 0x00U,
			.rxChannelMask = 0xFFU,
			.txChannelMask = 0x00U,
			.warmBoot = 0,
		};

		ret = adi_adrv903x_DeframerLinkStateSet(phy->palauDevice,
			priv->link[lnk->link_id].source_id, ADI_ENABLE);
		if (ret)
			return adrv903x_dev_err(phy);

		/* Notify ARM to run SERDES Calbriation if necessary */
		ret = adi_adrv903x_InitCalsRun(phy->palauDevice, &serdesCal);
		if (ret)
			return adrv903x_dev_err(phy);

		/* Wait up to 60 seconds for ARM */
		ret = adi_adrv903x_InitCalsWait(phy->palauDevice, 60000);
		if (ret) {
			dev_err(&phy->spi->dev, "Error: InitCalsWait\n");
			return adrv903x_dev_err(phy);
		}

		/***********************************************************/
		/**** Enable SYSREF to Koror JESD204B/JESD204C Deframer ****/
		/***********************************************************/
		ret = adi_adrv903x_DeframerSysrefCtrlSet(phy->palauDevice,
			priv->link[lnk->link_id].source_id, ADI_DISABLE);
		if (ret)
			return adrv903x_dev_err(phy);

		ret = adi_adrv903x_DeframerLinkStateSet(phy->palauDevice,
			priv->link[lnk->link_id].source_id, ADI_ENABLE);
		if (ret)
			return adrv903x_dev_err(phy);

		ret = adi_adrv903x_DeframerSysrefCtrlSet(phy->palauDevice,
				priv->link[lnk->link_id].source_id, ADI_ENABLE);
		if (ret)
			return adrv903x_dev_err(phy);
	};

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv903x_jesd204_link_running(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct adrv903x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv903x_rf_phy *phy = priv->phy;
	int ret;

	adi_adrv903x_DeframerStatus_v2_t deframerStatus;
 	adi_adrv903x_FramerStatus_t framerStatus;
 	uint8_t deframerLinkCondition = 0;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	if (!lnk->num_converters)
		return JESD204_STATE_CHANGE_DONE;

	if (priv->link[lnk->link_id].is_framer) {
		ret = adi_adrv903x_FramerStatusGet(phy->palauDevice,
 			priv->link[lnk->link_id].source_id, &framerStatus);
		if (ret)
			return adrv903x_dev_err(phy);

		if (framerStatus.status != 0x82)
			dev_warn(&phy->spi->dev,
				"Link%u framerStatus 0x%X",
				lnk->link_id, framerStatus.status);
	} else {
 		ret = adi_adrv903x_DeframerStatusGet_v2(phy->palauDevice,
 			priv->link[lnk->link_id].source_id, &deframerStatus);
		if (ret)
			return adrv903x_dev_err(phy);

		ret  = adi_adrv903x_DfrmLinkConditionGet(
			phy->palauDevice,
			priv->link[lnk->link_id].source_id,
			&deframerLinkCondition);
		if (ret)
			return adrv903x_dev_err(phy);

		for(int i = 0; i < lnk->num_lanes; i++) {
			dev_warn(&phy->spi->dev,
				"Link%u deframerStatus lane %d 0x%X",
				lnk->link_id, i, deframerStatus.laneStatus[i]);
		}
	};

	return JESD204_STATE_CHANGE_DONE;
}

/* Helper function to convert between tracking cals to init cals
 *
 * Returns init cal.
 */
 ADI_API adi_adrv903x_InitCalibrations_e drv_cals_TrackingCalConvert(const adi_adrv903x_TrackingCalibrationMask_e trackingCal)
 {
     adi_adrv903x_InitCalibrations_e initCal = (adi_adrv903x_InitCalibrations_e) 0U;

     switch (trackingCal)
     {
         case ADI_ADRV903X_TC_TX_LOL_MASK:
             initCal = ADI_ADRV903X_IC_TXLOL;
             break;

         case ADI_ADRV903X_TC_TX_QEC_MASK:
             initCal = ADI_ADRV903X_IC_TXQEC;
             break;

         case ADI_ADRV903X_TC_TX_SERDES_MASK:
             initCal = ADI_ADRV903X_IC_SERDES;
             break;

         case ADI_ADRV903X_TC_RX_ADC_MASK:
             initCal = ADI_ADRV903X_IC_ADC_RX;
             break;

         case ADI_ADRV903X_TC_TX_LB_ADC_MASK:
             initCal = ADI_ADRV903X_IC_ADC_TXLB;
             break;

         case ADI_ADRV903X_TC_ORX_ADC_MASK:
             initCal = ADI_ADRV903X_IC_ADC_ORX;
             break;

         case ADI_ADRV903X_TC_RX_QEC_MASK:
             /* Fall Through */

         default:
             initCal = (adi_adrv903x_InitCalibrations_e) 0U;
             break;
     }

     return initCal;
 }

static int adrv903x_jesd204_post_running_stage(struct jesd204_dev *jdev,
	enum jesd204_state_op_reason reason)
{
	adi_adrv903x_TrackingCalibrationMask_e trackingCal = (adi_adrv903x_TrackingCalibrationMask_e) 0U;
	adi_adrv903x_InitCalibrations_e currentInitCalMask = (adi_adrv903x_InitCalibrations_e) 0U;
	struct adrv903x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	static adi_adrv903x_InitCalStatus_t initCalStatus;
	struct device *dev = jesd204_dev_to_device(jdev);
	const uint32_t ALL_CHANNELS_MASK = 0xFFU;
	struct adrv903x_rf_phy *phy = priv->phy;
	adi_adrv903x_TxAtten_t txAttenuation[1];
	const uint32_t NUM_TRACKING_CALS = 7U;
	uint8_t i, j;
	int ret;

	const uint32_t trackingCalMask = (uint32_t)(ADI_ADRV903X_TC_RX_ADC_MASK     |
						ADI_ADRV903X_TC_ORX_ADC_MASK    |
						ADI_ADRV903X_TC_TX_LB_ADC_MASK  |
						ADI_ADRV903X_TC_TX_SERDES_MASK);

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		phy->is_initialized = 0;
		return JESD204_STATE_CHANGE_DONE;
	}

	clk_set_rate(phy->clks[RX_SAMPL_CLK], phy->rx_iqRate_kHz * 1000);
	clk_set_rate(phy->clks[OBS_SAMPL_CLK], phy->orx_iqRate_kHz * 1000);
	clk_set_rate(phy->clks[TX_SAMPL_CLK], phy->tx_iqRate_kHz * 1000);

	// Tx enabled only if EN toggles
	ret = adi_adrv903x_RxTxEnableSet(phy->palauDevice, 0x00, 0x00,
		ADI_ADRV903X_RX_MASK_ALL, ADI_ADRV903X_RX_MASK_ALL,
		ADI_ADRV903X_TXALL, ADI_ADRV903X_TXALL);
	if (ret)
		return adrv903x_dev_err(phy);

	ret = adi_adrv903x_RxTxEnableSet(phy->palauDevice, 0x00, 0x00,
		0x00, 0x00,
		ADI_ADRV903X_TXALL, 0x00);
	if (ret)
		return adrv903x_dev_err(phy);

	ret = adi_adrv903x_RxTxEnableSet(phy->palauDevice, 0x00, 0x00,
		ADI_ADRV903X_RX_MASK_ALL, ADI_ADRV903X_RX_MASK_ALL,
		ADI_ADRV903X_TXALL, ADI_ADRV903X_TXALL);
	if (ret)
		return adrv903x_dev_err(phy);

	txAttenuation[0].txChannelMask = ADI_ADRV903X_TXALL;
	txAttenuation[0].txAttenuation_mdB = 6000;
	ret = adi_adrv903x_TxAttenSet(phy->palauDevice, txAttenuation, 1);
	if (ret)
		return adrv903x_dev_err(phy);

	memset(&initCalStatus, 0, sizeof(adi_adrv903x_InitCalStatus_t));

	ret = adi_adrv903x_InitCalsDetailedStatusGet(phy->palauDevice, &initCalStatus);
	if (ret)
		return adrv903x_dev_err(phy);

	for (i = 0U; i < NUM_TRACKING_CALS; ++i)
	{
		trackingCal = (adi_adrv903x_TrackingCalibrationMask_e) (1U << i);

		if (((uint32_t)trackingCal & trackingCalMask) == 0U)
		{
			/* Tracking Cal not configured to run */
			continue;
		}

		/* Check if the Current Tracking Cal was initially run */
		currentInitCalMask = drv_cals_TrackingCalConvert(trackingCal);

		for (j = 0U; j < ADI_ADRV903X_MAX_CHANNELS; ++j)
		{
			if ((initCalStatus.calsSincePowerUp[j] & currentInitCalMask) == 0U)
			{
				/* Tracking Cal was already run */
				continue;
			}
		}

		ret = adi_adrv903x_TrackingCalsEnableSet(phy->palauDevice,
							trackingCal,
							ALL_CHANNELS_MASK,
							ADI_ADRV903X_TRACKING_CAL_ENABLE);
		if (ret)
			return adrv903x_dev_err(phy);
	}

	phy->is_initialized = 1;
	adrv903x_info(phy);

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_adrv903x_init = {
	.state_ops = {
		[JESD204_OP_DEVICE_INIT] = {
			.per_device = adrv903x_jesd204_device_init,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_LINK_INIT] = {
			.per_link = adrv903x_jesd204_link_init,
		},
		[JESD204_OP_LINK_PRE_SETUP] = {
			.per_device = adrv903x_jesd204_link_pre_setup,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_LINK_SETUP] = {
			.per_device = adrv903x_jesd204_link_setup,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
			.post_state_sysref = true,
		},
		[JESD204_OP_OPT_SETUP_STAGE1] = {
			.per_device = adrv903x_jesd204_setup_stage1,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
			.post_state_sysref = true,
		},
		[JESD204_OP_OPT_SETUP_STAGE2] = {
			.per_device = adrv903x_jesd204_setup_stage2,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
			.post_state_sysref = true,
		},
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = adrv903x_jesd204_clks_enable,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = adrv903x_jesd204_link_enable,
			.post_state_sysref = true,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = adrv903x_jesd204_link_running,
		},
		[JESD204_OP_OPT_POST_RUNNING_STAGE] = {
			.per_device = adrv903x_jesd204_post_running_stage,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
	},

	.max_num_links = 10,
	.sizeof_priv = sizeof(struct adrv903x_jesd204_priv),
};

static int adrv903x_probe(struct spi_device *spi)
{
	adi_adrv903x_TrackingCalibrationMask_e trackingCal = (adi_adrv903x_TrackingCalibrationMask_e) 0U;
	adi_adrv903x_InitCalibrations_e currentInitCalMask = (adi_adrv903x_InitCalibrations_e) 0U;
	adi_adrv903x_ExtractInitDataOutput_e checkExtractInitData =
		ADI_ADRV903X_EXTRACT_INIT_DATA_NOT_POPULATED;
	static adi_adrv903x_InitCalStatus_t initCalStatus;
	struct device_node *np = spi->dev.of_node;
	const uint32_t ALL_CHANNELS_MASK = 0xFFU;
	adi_adrv903x_TxAtten_t txAttenuation[1];
	const uint32_t NUM_TRACKING_CALS = 7U;
	adi_common_ErrData_t* errData = NULL;
	struct adrv903x_jesd204_priv *priv;
	adi_adrv903x_Version_t apiVersion;
	struct adrv903x_rf_phy *phy;
	struct iio_dev *indio_dev;
	struct jesd204_dev *jdev;
	struct clk *clk = NULL;
	const char *name;
	int ret, i, j;
	u32 val;

	const uint32_t trackingCalMask = (uint32_t)(ADI_ADRV903X_TC_RX_ADC_MASK     |
						ADI_ADRV903X_TC_ORX_ADC_MASK    |
						ADI_ADRV903X_TC_TX_LB_ADC_MASK  |
						ADI_ADRV903X_TC_TX_SERDES_MASK);

	int id = spi_get_device_id(spi)->driver_data;

	jdev = devm_jesd204_dev_register(&spi->dev, &jesd204_adrv903x_init);
	if (IS_ERR(jdev)) {
		return PTR_ERR(jdev);
	}

	clk = devm_clk_get(&spi->dev, "dev_clk");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*phy));
	if (indio_dev == NULL)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->indio_dev = indio_dev;
	phy->spi = spi;
	phy->spi_device_id = id;
	phy->dev_clk = clk;
	phy->jdev = jdev;
	mutex_init(&phy->lock);

	priv = jesd204_dev_priv(jdev);
	priv->phy = phy;

	phy->palauDevice = &phy->adi_adrv903x_device;
	phy->linux_hal.spi = spi;
	phy->linux_hal.logCfg.logMask = ADI_HAL_LOG_ERR | ADI_HAL_LOG_WARN;
	phy->palauDevice->common.devHalInfo = &phy->linux_hal;
	phy->palauDevice->common.type = "RF";

	if (!of_property_read_string(np, "adi,device-config-name", &name)) {
		strncpy(phy->trxBinaryInfoPtr.cpuProfile.filePath, name, sizeof(phy->trxBinaryInfoPtr.cpuProfile.filePath));
	} else {
		dev_err(&spi->dev, "error missing dt property: adi,device-config-name\n");
		return -EINVAL;
	}

	if (!of_property_read_string(np, "adi,arm-firmware-name", &name)) {
		strncpy(phy->trxBinaryInfoPtr.cpu.filePath, name, sizeof(phy->trxBinaryInfoPtr.cpu.filePath));
	} else {
		dev_err(&spi->dev, "error missing dt property: adi,arm-firmware-name\n");
		return -EINVAL;
	}

	if (!of_property_read_string(np, "adi,stream-firmware-name", &name)) {
		strncpy(phy->trxBinaryInfoPtr.stream.filePath, name, sizeof(phy->trxBinaryInfoPtr.stream.filePath));
	} else {
		dev_err(&spi->dev, "error missing dt property: adi,stream-firmware-name\n");
		return -EINVAL;
	}

	for (i = 0; i < ADI_ADRV903X_RX_GAIN_TABLE_ARR_MAX; i++) {
		ret = of_property_read_string_index(np, "adi,rx-gaintable-names", i, &name);

		if (!ret && !of_property_read_u32_index(np, "adi,rx-gaintable-channel-masks", i, &val)) {
			strncpy(phy->trxBinaryInfoPtr.rxGainTable[i].filePath, name,
				sizeof(phy->trxBinaryInfoPtr.rxGainTable[i].filePath));
			phy->trxBinaryInfoPtr.rxGainTable[i].channelMask = val;
		}
	}

	if (adi_adrv903x_hal_PlatformSetup(ADI_LINUX) != ADI_HAL_ERR_OK)
	{
		dev_err(&spi->dev, "error HAL function(s) not implemented.\n");
		return -EINVAL;
	}

	phy->linux_hal.reset_gpio =
		devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_LOW);

	ret = clk_prepare_enable(phy->dev_clk);
	if (ret)
		return ret;

	phy->spiSettings.msbFirst = 1;
	phy->spiOptions.allowSpiStreaming = 0;
	phy->spiOptions.allowAhbAutoIncrement = 1;
	phy->spiOptions.allowAhbSpiFifoMode = 0;
	phy->spiSettings.fourWireMode = 1;
	phy->spiSettings.cmosPadDrvStrength = ADI_ADRV903X_CMOSPAD_DRV_STRONG;
	phy->linux_hal.spiCfg.interfaceEnabled = 1;

	adi_adrv903x_LogLevelSet(&phy->palauDevice->common, ADI_HAL_LOG_ALL);

	/* Register errData to use for API calls */
	errData = ADI_CREATE_ERROR_MEMORY();
	if (errData == NULL)
	{
		return -1;
	}

	ret = adrv903x_TlsSet(HAL_TLS_ERR, errData);
	if (ret != ADI_HAL_ERR_OK)
	{
		(void) ADI_DESTROY_ERROR_MEMORY(errData);
		return -1;
	}

	phy->palauDevice->common.deviceInfo.id = id;
	phy->palauDevice->common.deviceInfo.name = spi_get_device_id(spi)->name;
	phy->palauDevice->common.deviceInfo.type = 0x00;

	ret = adi_adrv903x_HwOpen(phy->palauDevice, &phy->spiSettings);
	if (ret)
		return adrv903x_dev_err(phy);

	ret = adi_adrv903x_SpiVerify(phy->palauDevice);
	if (ret)
		return adrv903x_dev_err(phy);

	ret = adi_adrv903x_ApiVersionGet(phy->palauDevice, &apiVersion);
	if (ret)
		return adrv903x_dev_err(phy);

	dev_info(&spi->dev,
		 "%s Rev %d, API version: %u.%u.%u.%u found",
		 spi_get_device_id(spi)->name,
		 phy->palauDevice->devStateInfo.deviceSiRev,
		 apiVersion.majorVer, apiVersion.minorVer,
		 apiVersion.maintenanceVer, apiVersion.buildVer);

	if (apiVersion.majorVer > 1U) {
		ret = adi_adrv903x_InitDataExtract(phy->palauDevice,
						   &phy->trxBinaryInfoPtr.cpuProfile,
                                                   &initStructApiVersion,
                                                   &initStructArmVersion,
                                                   &initStructStreamVersion,
                                                   &deviceInitStruct,
                                                   &utilityInit,
                                        	   &checkExtractInitData);

		switch (checkExtractInitData) {
		case ADI_ADRV903X_EXTRACT_INIT_DATA_LEGACY_PROFILE_BIN:
			printk("\n\tUsing the Default Init and PostMcsInit Structures\n");
			break;

		case ADI_ADRV903X_EXTRACT_INIT_DATA_POPULATED:
			printk("\n\tUsing the Profile Init and PostMcsInit Structures\n");
			break;

		case ADI_ADRV903X_EXTRACT_INIT_DATA_NOT_POPULATED:
		/* Fall Through */

		default:
			ADI_APP_ERROR_REPORT(ADI_COMMON_ERRCODE_INVALID_PARAM,
					     ret,
					     &deviceInitStruct,
					     "PreMcsInit and/or PostMcsInit Data Structures Not Populated");
			return -EINVAL;
			break;
		}
	}

	/* Copy the data structure that holds Utility Init structures to the device */
	memcpy(&phy->adrv903xPostMcsInitInst, &utilityInit, sizeof(adi_adrv903x_PostMcsInit_t));

	/* Extract Info from CPU Profile Binary */
	/* Required for Link init */
	ret = adi_adrv903x_DeviceInfoExtract(phy->palauDevice,
			 &phy->trxBinaryInfoPtr.cpuProfile);
	if (ret != ADI_ADRV903X_ERR_ACT_NONE) {
		ADI_API_ERROR_REPORT(&phy->palauDevice->common, ret,
				     "Issue during CPU Profile Binary Image Extract");
		return -1;
	}

	adrv903x_clk_register(phy, "-rx_sampl_clk", __clk_get_name(phy->dev_clk), NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			      RX_SAMPL_CLK);
	adrv903x_clk_register(phy, "-obs_sampl_clk", __clk_get_name(phy->dev_clk), NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			      OBS_SAMPL_CLK);
	adrv903x_clk_register(phy, "-tx_sampl_clk", __clk_get_name(phy->dev_clk), NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			      TX_SAMPL_CLK);

	phy->clk_data.clks = phy->clks;
	phy->clk_data.clk_num = NUM_ADRV903X_CLKS;

	ret = of_clk_add_provider(np, of_clk_src_onecell_get,
				  &phy->clk_data);
	if (ret)
		goto out_disable_clocks;

	indio_dev->dev.parent = &spi->dev;

	if (np)
		indio_dev->name = np->name;
	else
		indio_dev->name = "adrv903x-phy";

	indio_dev->modes = INDIO_DIRECT_MODE;

	switch (id) {
	case ID_ADRV9032:
		indio_dev->info = &adrv903x_phy_info;
		indio_dev->channels = adrv903x_phy_chan;
		indio_dev->num_channels = ARRAY_SIZE(adrv903x_phy_chan);
		break;
	default:
		ret = -EINVAL;
		goto out_clk_del_provider;
	}

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto out_clk_del_provider;

	ret = adrv903x_register_axi_converter(phy);
	if (ret < 0)
		goto out_iio_device_unregister;

	ret = adrv903x_register_debugfs(indio_dev);
	if (ret < 0)
		dev_warn(&spi->dev, "%s: failed to register debugfs", __func__);

	adi_adrv903x_ApiVersionGet(phy->palauDevice, &apiVersion);

	dev_info(&spi->dev,
		 "%s Rev %d, API version: %u.%u.%u.%u found",
		 spi_get_device_id(spi)->name,
		 phy->palauDevice->devStateInfo.deviceSiRev,
		 apiVersion.majorVer, apiVersion.minorVer,
		 apiVersion.maintenanceVer, apiVersion.buildVer);

	ret = adi_adrv903x_HwReset(phy->palauDevice);

	ret = adi_adrv903x_PreMcsInit(phy->palauDevice, &deviceInitStruct,
			 &phy->trxBinaryInfoPtr);
	if (ret != ADI_ADRV903X_ERR_ACT_NONE) {
		pr_err("ERROR adi_adrv903x_PreMcsInit failed in %s at line %d.\n", __func__,
		       __LINE__);
	}

	ret = adi_adrv903x_PreMcsInit_NonBroadcast(phy->palauDevice,
			 &deviceInitStruct);
	if (ret != ADI_ADRV903X_ERR_ACT_NONE) {
		pr_err("ERROR adi_adrv903x_PreMcsInit_NonBroadcast failed in %s at line %d.\n",
		       __func__, __LINE__);
	}

		// Tx enabled only if EN toggles
	ret = adi_adrv903x_RxTxEnableSet(phy->palauDevice, 0x00, 0x00,
		ADI_ADRV903X_RX_MASK_ALL, ADI_ADRV903X_RX_MASK_ALL,
		ADI_ADRV903X_TXALL, ADI_ADRV903X_TXALL);
	if (ret)
		return adrv903x_dev_err(phy);

	ret = adi_adrv903x_RxTxEnableSet(phy->palauDevice, 0x00, 0x00,
		0x00, 0x00,
		ADI_ADRV903X_TXALL, 0x00);
	if (ret)
		return adrv903x_dev_err(phy);

	ret = adi_adrv903x_RxTxEnableSet(phy->palauDevice, 0x00, 0x00,
		ADI_ADRV903X_RX_MASK_ALL, ADI_ADRV903X_RX_MASK_ALL,
		ADI_ADRV903X_TXALL, ADI_ADRV903X_TXALL);
	if (ret)
		return adrv903x_dev_err(phy);

		txAttenuation[0].txChannelMask = ADI_ADRV903X_TXALL;
	txAttenuation[0].txAttenuation_mdB = 6000;
	ret = adi_adrv903x_TxAttenSet(phy->palauDevice, txAttenuation, 1);
	if (ret)
		return adrv903x_dev_err(phy);

	memset(&initCalStatus, 0, sizeof(adi_adrv903x_InitCalStatus_t));

	ret = adi_adrv903x_InitCalsDetailedStatusGet(phy->palauDevice, &initCalStatus);
	if (ret)
		return adrv903x_dev_err(phy);

	for (i = 0U; i < NUM_TRACKING_CALS; ++i)
	{
		trackingCal = (adi_adrv903x_TrackingCalibrationMask_e) (1U << i);

		if (((uint32_t)trackingCal & trackingCalMask) == 0U)
		{
			/* Tracking Cal not configured to run */
			continue;
		}

		/* Check if the Current Tracking Cal was initially run */
		currentInitCalMask = drv_cals_TrackingCalConvert(trackingCal);

		for (j = 0U; j < ADI_ADRV903X_MAX_CHANNELS; ++j)
		{
			if ((initCalStatus.calsSincePowerUp[j] & currentInitCalMask) == 0U)
			{
				/* Tracking Cal was already run */
				continue;
			}
		}

		ret = adi_adrv903x_TrackingCalsEnableSet(phy->palauDevice,
							trackingCal,
							ALL_CHANNELS_MASK,
							ADI_ADRV903X_TRACKING_CAL_ENABLE);
		if (ret)
			return adrv903x_dev_err(phy);
	}

	phy->is_initialized = 1;
	adrv903x_info(phy);

	pr_err("TESTING ADRV903X successfully probed\n");

	return 0;

out_iio_device_unregister:
	iio_device_unregister(indio_dev);
out_clk_del_provider:
	of_clk_del_provider(np);
out_disable_clocks:
	clk_disable_unprepare(phy->dev_clk);

	return ret;
}

static void adrv903x_remove(struct spi_device *spi)
{
	struct adrv903x_rf_phy *phy = adrv903x_spi_to_phy(spi);

	iio_device_unregister(phy->indio_dev);
	of_clk_del_provider(spi->dev.of_node);
	clk_disable_unprepare(phy->dev_clk);

	adrv903x_shutdown(phy);
	return;
}

static const struct spi_device_id adrv903x_id[] = {
	{ "adrv9032", ID_ADRV9032 },
	{}
};
MODULE_DEVICE_TABLE(spi, adrv903x_id);

static const struct of_device_id adrv903x_of_match[] = {
	{ .compatible = "adi,adrv9032" },
	{},
};
MODULE_DEVICE_TABLE(of, adrv903x_of_match);

static struct spi_driver adrv903x_driver = {
	.driver = {
			.name = "adrv903x",
			.of_match_table = of_match_ptr(adrv903x_of_match),
		},
	.probe = adrv903x_probe,
	.remove = adrv903x_remove,
	.id_table = adrv903x_id,
};
module_spi_driver(adrv903x_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADRV903X TRX");
MODULE_LICENSE("GPL v2");
