// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV904X RF Transceiver
 *
 * Copyright 2020-2024 Analog Devices Inc.
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

#include <linux/unaligned.h>

#include <linux/iio/sysfs.h>
#include <linux/iio/iio.h>

#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/clk.h>

#include <linux/jesd204/jesd204.h>

#include <dt-bindings/iio/adc/adi,adrv904x.h>

#include "adrv904x.h"
#include "initdata.h"

#define ADI_FILE ADI_COMMON_FILE_HAL

enum adrv904x_iio_dev_attr {
	ADRV904X_INIT_CAL,
	ADRV904X_JESD204_FSM_ERROR,
	ADRV904X_JESD204_FSM_PAUSED,
	ADRV904X_JESD204_FSM_STATE,
	ADRV904X_JESD204_FSM_RESUME,
	ADRV904X_JESD204_FSM_CTRL,
};

static int __adrv904x_dev_err(struct adrv904x_rf_phy *phy, const char *function,
			      const int line)
{
	int ret = 0;

	dev_err(&phy->spi->dev, "%s, %d: failed with %s (%d)\n", function, line,
		phy->kororDevice->common.errPtr->errDebugInfo.errCause,
		phy->kororDevice->common.errPtr->errDebugInfo.highestPriorityAction);

	switch (phy->kororDevice->common.errPtr->errDebugInfo.highestPriorityAction) {
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

	pr_err("ERROR\n");

	adi_common_ErrClear(phy->kororDevice->common.errPtr);

	return ret;
}

#define adrv904x_dev_err(phy) __adrv904x_dev_err(phy, __func__, __LINE__)

int adrv904x_spi_read(struct spi_device *spi, unsigned int reg)
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

int adrv904x_spi_write(struct spi_device *spi, unsigned int reg, unsigned int val)
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
int adrv904x_TxLinkSamplingRateFind(adi_adrv904x_Device_t *device,
				    adi_adrv904x_DeframerSel_e deframerSel,
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
	case ADI_ADRV904X_DEFRAMER_0:
		deframerIndex = 0;
		break;
	case ADI_ADRV904X_DEFRAMER_1:
		deframerIndex = 1;
		break;
	default:
		ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API,
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
static void adrv904x_shutdown(struct adrv904x_rf_phy *phy)
{
	/***********************************************
	 * Shutdown Procedure *
	 * **********************************************/
	/* Function to turn radio off, Disables transmitters and receivers */

	adi_adrv904x_Shutdown(phy->kororDevice);
	adi_adrv904x_HwClose(phy->kororDevice);

	memset(&phy->adi_adrv904x_device.devStateInfo, 0,
	       sizeof(phy->adi_adrv904x_device.devStateInfo));
}

static ssize_t adrv904x_phy_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adrv904x_rf_phy *phy = iio_priv(indio_dev);
	bool enable;
	int ret = 0;
	u64 val;

	mutex_lock(&phy->lock);

	switch ((u32)this_attr->address & 0xFF) {
	case ADRV904X_INIT_CAL:
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
				phy->adrv904xPostMcsInitInst.initCals.orxChannelMask;
			phy->cal_mask.rxChannelMask =
				phy->adrv904xPostMcsInitInst.initCals.rxChannelMask;
			phy->cal_mask.txChannelMask =
				phy->adrv904xPostMcsInitInst.initCals.txChannelMask;

			/* Run Init Cals */
			ret = adi_adrv904x_InitCalsRun(phy->kororDevice,
						       &phy->cal_mask);
			if (ret) {
				ret = adrv904x_dev_err(phy);
				break;
			}

			ret = adi_adrv904x_InitCalsWait(phy->kororDevice,
							INIT_CALS_TIMEOUT_MS);
			if (ret)
				ret = adrv904x_dev_err(phy);
		}
		break;
	case ADRV904X_JESD204_FSM_RESUME:
		if (!phy->jdev) {
			ret = -EOPNOTSUPP;
			break;
		}

		ret = jesd204_fsm_resume(phy->jdev, JESD204_LINKS_ALL);
		break;
	case ADRV904X_JESD204_FSM_CTRL:
		if (!phy->jdev) {
			ret = -EOPNOTSUPP;
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

static ssize_t adrv904x_phy_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adrv904x_rf_phy *phy = iio_priv(indio_dev);
	struct jesd204_dev *jdev = phy->jdev;
	struct jesd204_link *links[5];
	int ret = 0, i, err, num_links;
	bool paused;
	u64 val;

	mutex_lock(&phy->lock);
	switch ((u32)this_attr->address & 0xFF) {
	case ADRV904X_INIT_CAL:
		val = this_attr->address >> 8;

		if (val)
			ret = sprintf(buf, "%d\n",
				      !!(phy->cal_mask.calMask & val));
		break;
	case ADRV904X_JESD204_FSM_ERROR:
		if (!phy->jdev) {
			ret = -EOPNOTSUPP;
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
	case ADRV904X_JESD204_FSM_PAUSED:
		if (!phy->jdev) {
			ret = -EOPNOTSUPP;
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
	case ADRV904X_JESD204_FSM_STATE:
		if (!phy->jdev) {
			ret = -EOPNOTSUPP;
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
		 * and that ADRV904X_JESD204_FSM_PAUSED was called before
		 */
		ret = sprintf(buf, "%s\n", jesd204_link_get_state_str(links[0]));
		break;
	case ADRV904X_JESD204_FSM_CTRL:
		if (!phy->jdev) {
			ret = -EOPNOTSUPP;
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

static IIO_DEVICE_ATTR(calibrate, 0644, adrv904x_phy_show,
		       adrv904x_phy_store, ADRV904X_INIT_CAL);

static IIO_DEVICE_ATTR(calibrate_rx_dc_offset_en, 0644,
		       adrv904x_phy_show, adrv904x_phy_store,
		       ADRV904X_INIT_CAL | (ADI_ADRV904X_IC_RX_DC_OFFSET << 8));

static IIO_DEVICE_ATTR(calibrate_rx_adc_en, 0644,
		       adrv904x_phy_show, adrv904x_phy_store,
		       ADRV904X_INIT_CAL | (ADI_ADRV904X_IC_ADC_RX << 8));

static IIO_DEVICE_ATTR(calibrate_orx_adc_en, 0644,
		       adrv904x_phy_show, adrv904x_phy_store,
		       ADRV904X_INIT_CAL | (ADI_ADRV904X_IC_ADC_ORX << 8));

static IIO_DEVICE_ATTR(calibrate_tx_loopback_en, 0644,
		       adrv904x_phy_show, adrv904x_phy_store,
		       ADRV904X_INIT_CAL | (ADI_ADRV904X_IC_ADC_TXLB << 8));

static IIO_DEVICE_ATTR(calibrate_tx_dac_en, 0644,
		       adrv904x_phy_show, adrv904x_phy_store,
		       ADRV904X_INIT_CAL | (ADI_ADRV904X_IC_TXDAC << 8));

static IIO_DEVICE_ATTR(calibrate_tx_bb_filter_en, 0644,
		       adrv904x_phy_show, adrv904x_phy_store,
		       ADRV904X_INIT_CAL | (ADI_ADRV904X_IC_TXBBF << 8));

static IIO_DEVICE_ATTR(calibrate_tx_loopback_filter_en, 0644,
		       adrv904x_phy_show, adrv904x_phy_store,
		       ADRV904X_INIT_CAL | (ADI_ADRV904X_IC_TXLB_FILTER << 8));

static IIO_DEVICE_ATTR(calibrate_tx_loopback_path_delay_en, 0644,
		       adrv904x_phy_show, adrv904x_phy_store,
		       ADRV904X_INIT_CAL | (ADI_ADRV904X_IC_TXLB_PATH_DLY << 8));

static IIO_DEVICE_ATTR(calibrate_tx_hrm_en, 0644,
		       adrv904x_phy_show, adrv904x_phy_store,
		       ADRV904X_INIT_CAL | (ADI_ADRV904X_IC_HRM << 8));

static IIO_DEVICE_ATTR(calibrate_tx_qec_en, 0644,
		       adrv904x_phy_show, adrv904x_phy_store,
		       ADRV904X_INIT_CAL | (ADI_ADRV904X_IC_TXQEC << 8));

static IIO_DEVICE_ATTR(calibrate_tx_lol_en, 0644,
		       adrv904x_phy_show, adrv904x_phy_store,
		       ADRV904X_INIT_CAL |
			       (ADI_ADRV904X_IC_TXLOL << 8));

static IIO_DEVICE_ATTR(jesd204_fsm_error, 0444,
		       adrv904x_phy_show,
		       NULL,
		       ADRV904X_JESD204_FSM_ERROR);

static IIO_DEVICE_ATTR(jesd204_fsm_paused, 0444,
		       adrv904x_phy_show,
		       NULL,
		       ADRV904X_JESD204_FSM_PAUSED);

static IIO_DEVICE_ATTR(jesd204_fsm_state, 0444,
		       adrv904x_phy_show,
		       NULL,
		       ADRV904X_JESD204_FSM_STATE);

static IIO_DEVICE_ATTR(jesd204_fsm_resume, 0200,
		       NULL,
		       adrv904x_phy_store,
		       ADRV904X_JESD204_FSM_RESUME);

static IIO_DEVICE_ATTR(jesd204_fsm_ctrl, 0644,
		       adrv904x_phy_show,
		       adrv904x_phy_store,
		       ADRV904X_JESD204_FSM_CTRL);

static struct attribute *adrv904x_phy_attributes[] = {
	&iio_dev_attr_calibrate.dev_attr.attr,
	&iio_dev_attr_calibrate_rx_dc_offset_en.dev_attr.attr,
	&iio_dev_attr_calibrate_rx_adc_en.dev_attr.attr,
	&iio_dev_attr_calibrate_orx_adc_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_loopback_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_dac_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_bb_filter_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_loopback_filter_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_loopback_path_delay_en.dev_attr.attr,
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

static const struct attribute_group adrv904x_phy_attribute_group = {
	.attrs = adrv904x_phy_attributes,
};

static int adrv904x_phy_reg_access(struct iio_dev *indio_dev, u32 reg,
				   u32 writeval, u32 *readval)
{
	struct adrv904x_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&phy->lock);
	if (!readval) {
		ret = adrv904x_spi_write(phy->spi, reg, writeval);
	} else {
		*readval = adrv904x_spi_read(phy->spi, reg);
		ret = 0;
	}
	mutex_unlock(&phy->lock);

	return ret;
}

enum lo_ext_info {
	LOEXT_FREQ,
};

static ssize_t adrv904x_phy_lo_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adrv904x_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv904x_LoConfig_t loConfig = { 0 };
	int ret = 0;
	u64 readin;

	switch (private) {
	case LOEXT_FREQ:

		ret = kstrtoull(buf, 10, &readin);
		if (ret)
			return ret;

		loConfig.loFrequency_Hz = readin;
		loConfig.loName = ADI_ADRV904X_LO0 + chan->channel;

		mutex_lock(&phy->lock);

		ret = adi_adrv904x_LoFrequencySet(phy->kororDevice, &loConfig);
		if (ret)
			ret = adrv904x_dev_err(phy);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&phy->lock);

	return ret ? ret : len;
}

static ssize_t adrv904x_phy_lo_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct adrv904x_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv904x_LoConfigReadback_t loConfig = { 0 };
	int ret;

	mutex_lock(&phy->lock);
	switch (private) {
	case LOEXT_FREQ:
		loConfig.loName = ADI_ADRV904X_LO0 + chan->channel;
		ret = adi_adrv904x_LoFrequencyGet(phy->kororDevice, &loConfig);
		if (ret)
			ret = adrv904x_dev_err(phy);
		break;
	default:
		ret = 0;
	}
	mutex_unlock(&phy->lock);

	return ret ? ret : sprintf(buf, "%llu\n", loConfig.loFrequency_Hz);
}

#define _ADRV904X_EXT_LO_INFO(_name, _ident)                                   \
	{                                                                      \
		.name = _name, .read = adrv904x_phy_lo_read,                   \
		.write = adrv904x_phy_lo_write, .private = _ident,             \
	}

static const struct iio_chan_spec_ext_info adrv904x_phy_ext_lo_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADRV904X_EXT_LO_INFO("frequency", LOEXT_FREQ),
	{},
};

static ssize_t adrv904x_phy_rx_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adrv904x_rf_phy *phy = iio_priv(indio_dev);

	adi_adrv904x_TrackingCalibrationMask_t calMask = 0;
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
			mask = ADI_ADRV904X_RX0 << chan->channel;
			calMask = ADI_ADRV904X_TC_RX_QEC_MASK;

			ret = adi_adrv904x_TrackingCalsEnableSet(phy->kororDevice, calMask,
								 mask,
								 enable ? ADI_ADRV904X_TRACKING_CAL_ENABLE :
									ADI_ADRV904X_TRACKING_CAL_DISABLE);
			if (ret)
				ret = adrv904x_dev_err(phy);
		}
		break;
	case RX_DIG_DC:
		if (chan->channel <= CHAN_RX8) {
			mask = ADI_ADRV904X_RX0 << chan->channel;

			ret = adi_adrv904x_DigDcOffsetEnableSet(phy->kororDevice, mask,
								enable ? 0xFF : 0x00);
			if (ret)
				ret = adrv904x_dev_err(phy);
		}
		break;
	case RX_ADC:
		if (chan->channel <= CHAN_RX8) {
			mask = ADI_ADRV904X_RX0 << chan->channel;
			calMask = ADI_ADRV904X_TC_RX_ADC_MASK;

			ret = adi_adrv904x_TrackingCalsEnableSet(phy->kororDevice, calMask,
								 mask,
								 enable ? ADI_ADRV904X_TRACKING_CAL_ENABLE :
									ADI_ADRV904X_TRACKING_CAL_DISABLE);
			if (ret)
				ret = adrv904x_dev_err(phy);
		} else {
			mask = chan->channel - CHAN_RX8;
			calMask = ADI_ADRV904X_TC_ORX_ADC_MASK;

			ret = adi_adrv904x_TrackingCalsEnableSet(phy->kororDevice, calMask,
								 mask,
								 enable ? ADI_ADRV904X_TRACKING_CAL_ENABLE :
									ADI_ADRV904X_TRACKING_CAL_DISABLE);
			if (ret)
				ret = adrv904x_dev_err(phy);
		}
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	return ret ? ret : len;
}

static ssize_t adrv904x_phy_rx_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct adrv904x_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv904x_TrackingCalEnableMasks_t enableMasks;
	adi_adrv904x_TrackingCalibrationMask_t chan_cals;
	u8 isEnabled;
	int ret = 0;
	u32 mask;

	mutex_lock(&phy->lock);

	switch (private) {
	case RX_QEC:
		if (chan->channel <= CHAN_RX8) {
			ret = adi_adrv904x_TrackingCalsEnableGet(phy->kororDevice, &enableMasks);
			if (ret == 0) {
				chan_cals = enableMasks.enableMask[chan->channel];
				ret = sprintf(buf, "%d\n", !!(chan_cals & ADI_ADRV904X_TC_RX_QEC_MASK));
			}
		}
		break;
	case RX_DIG_DC:
		mask = ADI_ADRV904X_RX0 << chan->channel;
		ret = adi_adrv904x_DigDcOffsetEnableGet(phy->kororDevice, mask, &isEnabled);

		if (ret == 0)
			ret = sprintf(buf, "%d\n", isEnabled);
		else
			ret = adrv904x_dev_err(phy);
		break;
	case RX_RF_BANDWIDTH:
		ret = sprintf(buf, "%u\n",
			      phy->adi_adrv904x_device.initExtract.rx.rxChannelCfg[chan->channel].rfBandwidth_kHz *
			      1000);
		break;
	case RX_ADC:
		if (chan->channel <= CHAN_RX8) {
			ret = adi_adrv904x_TrackingCalsEnableGet(phy->kororDevice, &enableMasks);
			if (ret == 0) {
				chan_cals = enableMasks.enableMask[chan->channel];
				ret = sprintf(buf, "%d\n", !!(chan_cals & ADI_ADRV904X_TC_RX_ADC_MASK));
			}
		} else {
			ret = adi_adrv904x_TrackingCalsEnableGet(phy->kororDevice, &enableMasks);
			if (ret == 0) {
				chan_cals = enableMasks.enableMask[chan->channel - CHAN_OBS_RX1];
				ret = sprintf(buf, "%d\n", !!(chan_cals & ADI_ADRV904X_TC_ORX_ADC_MASK));
			}
		}
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	return ret;
}

#define _ADRV904X_EXT_RX_INFO(_name, _ident)                                   \
	{                                                                      \
		.name = _name, .read = adrv904x_phy_rx_read,                   \
		.write = adrv904x_phy_rx_write, .private = _ident,             \
	}

static ssize_t adrv904x_phy_tx_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct adrv904x_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv904x_TrackingCalEnableMasks_t enableMasks[ADI_ADRV904X_NUM_TRACKING_CAL_CHANNELS] = { 0 };
	adi_adrv904x_TrackingCalibrationMask_t chan_cals;
	int val, ret = 0;

	if (chan->channel > CHAN_TX8)
		return -EINVAL;

	mutex_lock(&phy->lock);
	switch (private) {
	case TX_QEC:
		ret = adi_adrv904x_TrackingCalsEnableGet(phy->kororDevice, enableMasks);
		if (ret == 0) {
			chan_cals = enableMasks->enableMask[chan->channel];
			val = !!(chan_cals & ADI_ADRV904X_TC_TX_QEC_MASK);
		}
		break;
	case TX_LOL:
		ret = adi_adrv904x_TrackingCalsEnableGet(phy->kororDevice, enableMasks);
		if (ret == 0) {
			chan_cals = enableMasks->enableMask[chan->channel];
			val = !!(chan_cals & ADI_ADRV904X_TC_TX_LOL_MASK);
		}
		break;
	case TX_RF_BANDWIDTH:
		val = phy->adi_adrv904x_device.initExtract.tx.txChannelCfg[chan->channel]
			      .rfBandwidth_kHz * 1000;
		break;
	case TX_LB_ADC:
		ret = adi_adrv904x_TrackingCalsEnableGet(phy->kororDevice, enableMasks);
		if (ret == 0) {
			chan_cals = enableMasks->enableMask[chan->channel];
			val = !!(chan_cals & ADI_ADRV904X_TC_TX_LB_ADC_MASK);
		}
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	if (ret == 0)
		ret = sprintf(buf, "%d\n", val);
	else
		return adrv904x_dev_err(phy);

	return ret;
}

static ssize_t adrv904x_phy_tx_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	adi_adrv904x_TrackingCalibrationMask_t calMask = 0;
	struct adrv904x_rf_phy *phy = iio_priv(indio_dev);
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
		mask = ADI_ADRV904X_TX0 << chan->channel;
		calMask = ADI_ADRV904X_TC_TX_QEC_MASK;

		ret = adi_adrv904x_TrackingCalsEnableSet(phy->kororDevice, calMask,
							 mask,
							 enable ? ADI_ADRV904X_TRACKING_CAL_ENABLE :
								 ADI_ADRV904X_TRACKING_CAL_DISABLE);
		if (ret)
			ret = adrv904x_dev_err(phy);
		break;
	case TX_LOL:
		mask = ADI_ADRV904X_TX0 << chan->channel;
		calMask = ADI_ADRV904X_TC_TX_LOL_MASK;

		adi_adrv904x_ChannelTrackingCals_t channelMask = { 0 };

		channelMask.txChannel = mask;

		ret = adi_adrv904x_TrackingCalsEnableSet(phy->kororDevice, calMask,
							 mask,
							 enable ? ADI_ADRV904X_TRACKING_CAL_ENABLE :
								 ADI_ADRV904X_TRACKING_CAL_DISABLE);

		if (ret)
			ret = adrv904x_dev_err(phy);
		break;
	case TX_LB_ADC:
		mask = ADI_ADRV904X_TX0 << chan->channel;
		calMask = ADI_ADRV904X_TC_TX_LB_ADC_MASK;

		ret = adi_adrv904x_TrackingCalsEnableSet(phy->kororDevice, calMask,
							 mask,
							 enable ? ADI_ADRV904X_TRACKING_CAL_ENABLE :
								 ADI_ADRV904X_TRACKING_CAL_DISABLE);
		if (ret)
			ret = adrv904x_dev_err(phy);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	return ret ? ret : len;
}

#define _ADRV904X_EXT_TX_INFO(_name, _ident)                                   \
	{                                                                      \
		.name = _name, .read = adrv904x_phy_tx_read,                   \
		.write = adrv904x_phy_tx_write, .private = _ident,             \
	}

static const struct iio_chan_spec_ext_info adrv904x_phy_rx_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADRV904X_EXT_RX_INFO("quadrature_tracking_en", RX_QEC),
	_ADRV904X_EXT_RX_INFO("bb_dc_offset_tracking_en", RX_DIG_DC),
	_ADRV904X_EXT_RX_INFO("rf_bandwidth", RX_RF_BANDWIDTH),
	_ADRV904X_EXT_RX_INFO("adc_tracking_en", RX_ADC),
	{},
};

static const struct iio_chan_spec_ext_info adrv904x_phy_obs_rx_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADRV904X_EXT_RX_INFO("rf_bandwidth", RX_RF_BANDWIDTH),
	_ADRV904X_EXT_RX_INFO("adc_tracking_en", RX_ADC),
	{},
};

static struct iio_chan_spec_ext_info adrv904x_phy_tx_ext_info[] = {
	_ADRV904X_EXT_TX_INFO("quadrature_tracking_en", TX_QEC),
	_ADRV904X_EXT_TX_INFO("lo_leakage_tracking_en", TX_LOL),
	_ADRV904X_EXT_TX_INFO("rf_bandwidth", TX_RF_BANDWIDTH),
	_ADRV904X_EXT_TX_INFO("loopback_adc_tracking_en", TX_LB_ADC),
	{},
};

static int adrv904x_gainindex_to_gain(struct adrv904x_rf_phy *phy, int channel,
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

static int adrv904x_gain_to_gainindex(struct adrv904x_rf_phy *phy, int channel,
				      int val, int val2, unsigned int *index)
{
	int gain = ((abs(val) * 1000) + (abs(val2) / 1000));

	gain = clamp(gain, MIN_GAIN_mdB, MAX_RX_GAIN_mdB);
	*index = (gain - MAX_RX_GAIN_mdB) / RX_GAIN_STEP_mdB + 255;

	return 0;
}

static int adrv904x_phy_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, int *val,
				 int *val2, long m)
{
	adi_adrv904x_DevTempSensorMask_e avg_mask = ADI_ADRV904X_DEVTEMP_MASK_TX0;
	struct adrv904x_rf_phy *phy = iio_priv(indio_dev);
	u32 orxchan = 0, rxchan = 0, txchan = 0;
	adi_adrv904x_DevTempData_t temp = { 0 };
	int ret;

	mutex_lock(&phy->lock);
	switch (m) {
	case IIO_CHAN_INFO_ENABLE:

		ret = adi_adrv904x_RxTxEnableGet(phy->kororDevice, &orxchan, &rxchan,
						 &txchan);
		if (ret) {
			ret = adrv904x_dev_err(phy);
			break;
		}

		if (chan->output)
			*val = !!(txchan & (ADI_ADRV904X_TX0 << chan->channel));
		else if (chan->channel <= CHAN_RX8)
			*val = !!(rxchan & (ADI_ADRV904X_RX0 << chan->channel));
		else
			*val = !!(orxchan & (ADI_ADRV904X_RX0 << chan->channel));

		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			adi_adrv904x_TxAtten_t txAtten;

			ret = adi_adrv904x_TxAttenGet(phy->kororDevice,
						      1 << chan->channel,
						      &txAtten);
			if (ret) {
				ret = adrv904x_dev_err(phy);
				break;
			}

			*val = -1 * (txAtten.txAttenuation_mdB / 1000);
			*val2 = (txAtten.txAttenuation_mdB % 1000) * 1000;
			if (!*val)
				*val2 *= -1;

		} else {
			if (chan->channel <= CHAN_RX8) {
				adi_adrv904x_RxGain_t rxGain;

				ret = adi_adrv904x_RxGainGet(phy->kororDevice,
							     1 << chan->channel,
							     &rxGain);
				if (ret) {
					ret = adrv904x_dev_err(phy);
					break;
				}

				ret = adrv904x_gainindex_to_gain(phy, chan->channel,
								 rxGain.gainIndex, val,
								 val2);
			}
		}
		ret = IIO_VAL_INT_PLUS_MICRO_DB;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (chan->output)
			*val = clk_get_rate(phy->clks[TX_SAMPL_CLK]);
		else
			*val = clk_get_rate(phy->clks[RX_SAMPL_CLK]);

		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_PROCESSED:
		// ToDo variable avg_mask
		adi_adrv904x_TemperatureGet(phy->kororDevice, avg_mask, &temp);
		*val = (temp.tempDegreesCelsius[avg_mask] * 1000);
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	return ret;
};

static int adrv904x_phy_write_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan, int val,
				  int val2, long mask)
{
	u32 orxchan_msk = 0, rxchan_msk = 0, txchan_msk = 0;
	struct adrv904x_rf_phy *phy = iio_priv(indio_dev);
	u32 orxchan_en = 0, rxchan_en = 0, txchan_en = 0;
		int ret = 0;
	u32 code;

	mutex_lock(&phy->lock);
	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		ret = adi_adrv904x_RxTxEnableGet(phy->kororDevice, &orxchan_msk, &rxchan_msk,
						 &txchan_msk);
		if (ret) {
			ret = adrv904x_dev_err(phy);
			goto out;
		}

		txchan_msk = 0;
		rxchan_msk = 0;
		orxchan_msk = 0;
		if (chan->output) {
			txchan_msk = (1 << chan->channel);
			if (val)
				txchan_en |= (ADI_ADRV904X_TX0 << chan->channel);
			else
				txchan_en &= ~(ADI_ADRV904X_TX0 << chan->channel);
		} else {
			if (chan->channel <= CHAN_RX8) {
				rxchan_msk = (1 << chan->channel);
				if (val)
					rxchan_en |= (ADI_ADRV904X_RX0 << chan->channel);
				else
					rxchan_en &= ~(ADI_ADRV904X_RX0 << chan->channel);
			} else {
				orxchan_msk = (1 << chan->channel);
				if (val)
					orxchan_en |= (ADI_ADRV904X_RX0 << chan->channel);
				else
					orxchan_en &= ~(ADI_ADRV904X_RX0 << chan->channel);
			}
		}
		ret = adi_adrv904x_RxTxEnableSet(phy->kororDevice, orxchan_msk, orxchan_en, rxchan_msk, rxchan_en,
						 txchan_msk, txchan_en);
		if (ret)
			ret = adrv904x_dev_err(phy);
		break;

	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			adi_adrv904x_TxAtten_t txAtten;

			if (val > 0 || (val == 0 && val2 > 0)) {
				ret = -EINVAL;
				goto out;
			}

			code = ((abs(val) * 1000) + (abs(val2) / 1000));

			txAtten.txChannelMask = 1 << chan->channel;
			txAtten.txAttenuation_mdB =
				code; /* Back off Tx output power by 30dB */
			ret = adi_adrv904x_TxAttenSet(phy->kororDevice, &txAtten,
						      1);
			if (ret)
				adrv904x_dev_err(phy);

		} else {
			adi_adrv904x_RxGain_t rxGain;

			ret = adrv904x_gain_to_gainindex(phy, chan->channel,
							 val, val2, &code);
			if (ret < 0)
				break;

			if (chan->channel <= CHAN_RX8) {
				rxGain.gainIndex = code;
				rxGain.rxChannelMask = 1 << chan->channel;

				ret = adi_adrv904x_RxGainSet(phy->kororDevice, &rxGain,
							     1);
				if (ret)
					adrv904x_dev_err(phy);
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

static const struct iio_chan_spec adrv904x_phy_chan[] = {
	{
		/* LO1 */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.extend_name = "LO1",
		.ext_info = adrv904x_phy_ext_lo_info,
	},
	{
		/* LO2 */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.extend_name = "LO2",
		.ext_info = adrv904x_phy_ext_lo_info,
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
		.ext_info = adrv904x_phy_tx_ext_info,
	},
	{
		/* RX1 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv904x_phy_rx_ext_info,
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
		.ext_info = adrv904x_phy_tx_ext_info,
	},
	{
		/* RX2 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv904x_phy_rx_ext_info,
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
		.ext_info = adrv904x_phy_tx_ext_info,
	},
	{
		/* RX3 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX3,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv904x_phy_rx_ext_info,
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
		.ext_info = adrv904x_phy_tx_ext_info,
	},
	{
		/* RX4 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX4,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv904x_phy_rx_ext_info,
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
		.ext_info = adrv904x_phy_tx_ext_info,
	},
	{
		/* RX5 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX5,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv904x_phy_rx_ext_info,
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
		.ext_info = adrv904x_phy_tx_ext_info,
	},
	{
		/* RX6 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX6,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv904x_phy_rx_ext_info,
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
		.ext_info = adrv904x_phy_tx_ext_info,
	},
	{
		/* RX7 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX7,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv904x_phy_rx_ext_info,
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
		.ext_info = adrv904x_phy_tx_ext_info,
	},
	{
		/* RX8 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX8,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv904x_phy_rx_ext_info,
	},
	{
		/* RX Sniffer/Observation */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_OBS_RX1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv904x_phy_obs_rx_ext_info,
	},
	{
		/* RX Sniffer/Observation */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_OBS_RX2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv904x_phy_obs_rx_ext_info,
	},
	{
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static const struct iio_info adrv904x_phy_info = {
	.read_raw = &adrv904x_phy_read_raw,
	.write_raw = &adrv904x_phy_write_raw,
	.debugfs_reg_access = &adrv904x_phy_reg_access,
	.attrs = &adrv904x_phy_attribute_group,
};

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
			ret = -EINVAL;
		}

	} else {
		if (entry->cmd)
			val = entry->val;
		else
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

		frm_test_data.injectPoint = ADI_ADRV904X_FTD_FRAMERINPUT;
		frm_test_data.testDataSource = val;
		frm_test_data.framerSelMask = ADI_ADRV904X_FRAMER_0;

		ret = adi_adrv904x_FramerTestDataSet(phy->kororDevice,
						     &frm_test_data);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv904x_dev_err(phy);

		entry->val = val;
		return count;
	case DBGFS_BIST_FRAMER_LOOPBACK:
		mutex_lock(&phy->lock);
		ret = adi_adrv904x_FramerLoopbackSet(phy->kororDevice, ADI_ADRV904X_FRAMER_0);
		mutex_unlock(&phy->lock);
		if (ret)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_BIST_TONE:
		if (ret > 3)
			return -EINVAL;

		txNcoConfig.chanSelect = ADI_ADRV904X_TX7;
		txNcoConfig.bandSelect = 0;
		txNcoConfig.enable = val;
		txNcoConfig.ncoSelect = ADI_ADRV904X_TX_TEST_NCO_0;
		txNcoConfig.frequencyKhz = val2;
		if (ret == 3) {
			if (val3 > 8)
				val3  = 8;
			txNcoConfig.attenCtrl = val3;
		} else {
			txNcoConfig.attenCtrl = ADI_ADRV904X_TX_TEST_NCO_ATTEN_0DB;
		}

		mutex_lock(&phy->lock);
		ret = adi_adrv904x_TxTestToneSet(phy->kororDevice, &txNcoConfig);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv904x_dev_err(phy);

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

static const struct file_operations adrv904x_debugfs_reg_fops = {
	.open = simple_open,
	.read = adrv904x_debugfs_read,
	.write = adrv904x_debugfs_write,
};

static void adrv904x_add_debugfs_entry(struct adrv904x_rf_phy *phy,
				       const char *propname, unsigned int cmd)
{
	unsigned int i = phy->adrv904x_debugfs_entry_index;

	if (WARN_ON(i >= ARRAY_SIZE(phy->debugfs_entry)))
		return;

	phy->debugfs_entry[i].phy = phy;
	phy->debugfs_entry[i].propname = propname;
	phy->debugfs_entry[i].cmd = cmd;

	phy->adrv904x_debugfs_entry_index++;
}

static int adrv904x_register_debugfs(struct iio_dev *indio_dev)
{
	struct adrv904x_rf_phy *phy = iio_priv(indio_dev);
	struct dentry *d;
	int i;

	if (!iio_get_debugfs_dentry(indio_dev))
		return -ENODEV;

	adrv904x_add_debugfs_entry(phy, "bist_framer_0_prbs",
				   DBGFS_BIST_FRAMER_0_PRBS);
	adrv904x_add_debugfs_entry(phy, "bist_framer_loopback",
				   DBGFS_BIST_FRAMER_LOOPBACK);
	adrv904x_add_debugfs_entry(phy, "bist_tone", DBGFS_BIST_TONE);

	for (i = 0; i < phy->adrv904x_debugfs_entry_index; i++)
		d = debugfs_create_file(phy->debugfs_entry[i].propname, 0644,
					iio_get_debugfs_dentry(indio_dev),
					&phy->debugfs_entry[i],
					&adrv904x_debugfs_reg_fops);
	return 0;
}

#define ADRV904X_MAX_CLK_NAME 79

static char *adrv904x_clk_set_dev_name(struct adrv904x_rf_phy *phy, char *dest,
				       const char *name)
{
	size_t len = 0;

	if (!name)
		return NULL;

	if (*name == '-')
		len = strscpy(dest, dev_name(&phy->spi->dev),
			      ADRV904X_MAX_CLK_NAME);
	else
		*dest = '\0';

	return strncat(dest, name, ADRV904X_MAX_CLK_NAME - len);
}

static unsigned long adrv904x_bb_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct adrv904x_clock *clk_priv = to_clk_priv(hw);

	return clk_priv->rate;
}

static int adrv904x_bb_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct adrv904x_clock *clk_priv = to_clk_priv(hw);

	clk_priv->rate = rate;

	return 0;
}

static long adrv904x_bb_round_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long *prate)
{
	struct adrv904x_clock *clk_priv = to_clk_priv(hw);

	dev_dbg(&clk_priv->spi->dev, "%s: Rate %lu Hz", __func__, rate);

	return rate;
}

static const struct clk_ops bb_clk_ops = {
	.round_rate = adrv904x_bb_round_rate,
	.set_rate = adrv904x_bb_set_rate,
	.recalc_rate = adrv904x_bb_recalc_rate,
};

static int adrv904x_clk_register(struct adrv904x_rf_phy *phy, const char *name,
				 const char *parent_name,
				 const char *parent_name2, unsigned long flags,
				 u32 source)
{
	struct adrv904x_clock *clk_priv = &phy->clk_priv[source];
	struct clk_init_data init;
	struct clk *clk;
	char c_name[ADRV904X_MAX_CLK_NAME + 1],
		p_name[2][ADRV904X_MAX_CLK_NAME + 1];
	const char *_parent_name[2];
	u32 rate;

	/* struct adrv904x_clock assignments */
	clk_priv->source = source;
	clk_priv->hw.init = &init;
	clk_priv->spi = phy->spi;
	clk_priv->phy = phy;

	_parent_name[0] =
		adrv904x_clk_set_dev_name(phy, p_name[0], parent_name);
	_parent_name[1] =
		adrv904x_clk_set_dev_name(phy, p_name[1], parent_name2);

	init.name = adrv904x_clk_set_dev_name(phy, c_name, name);
	init.flags = flags;
	init.parent_names = &_parent_name[0];
	init.num_parents = _parent_name[1] ? 2 : _parent_name[0] ? 1 : 0;

	switch (source) {
	case RX_SAMPL_CLK:
		init.ops = &bb_clk_ops;
		clk_priv->rate = phy->rx_iqRate_kHz;
		break;
	case TX_SAMPL_CLK:
			adrv904x_TxLinkSamplingRateFind(phy->kororDevice,
							ADI_ADRV904X_DEFRAMER_0,
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

static void adrv904x_info(struct adrv904x_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	adi_adrv904x_Version_t apiVersion;
	u8 siRevision = 0xbb;

	adi_adrv904x_ApiVersionGet(phy->kororDevice, &apiVersion);
	adi_adrv904x_DeviceRevGet(phy->kororDevice, &siRevision);

	dev_info(&spi->dev, "\n%s Rev %d, API version: %u.%u.%u.%u successfully initialized%s",
		 spi_get_device_id(spi)->name,
		 phy->kororDevice->devStateInfo.deviceSiRev,
		 apiVersion.majorVer, apiVersion.minorVer,
		 apiVersion.maintenanceVer, apiVersion.buildVer,
		 phy->jdev ? " via jesd204-fsm" : "");
}

struct adrv904x_jesd204_link {
	unsigned int source_id;
	bool is_framer;
};

struct adrv904x_jesd204_priv {
	struct adrv904x_rf_phy *phy;
	struct adrv904x_jesd204_link link[5];
};

int adrv904x_jesd204_link_pre_setup(struct jesd204_dev *jdev,
				    enum jesd204_state_op_reason reason)
{
	struct adrv904x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv904x_rf_phy *phy = priv->phy;
	u32 deviceClockScaled_kHz = 0;
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
		phy->kororDevice->initExtract.clocks.deviceClockScaled_kHz;

	dev_clk = clk_round_rate(phy->dev_clk,
				 deviceClockScaled_kHz * 1000);

	if (dev_clk > 0 && ((dev_clk / 1000) ==
		deviceClockScaled_kHz)) {
		clk_set_rate(phy->dev_clk, (unsigned long)dev_clk);
	} else {
		dev_err(&phy->spi->dev,
			"Requesting device clock %u failed got %ld",
			deviceClockScaled_kHz * 1000, dev_clk);
		return -EINVAL;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv904x_jesd204_device_init(struct jesd204_dev *jdev,
					enum jesd204_state_op_reason reason)
{
	adi_adrv904x_ErrAction_e recoveryAction = ADI_ADRV904X_ERR_ACT_NONE;
	struct adrv904x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv904x_rf_phy *phy = priv->phy;

	dev_dbg(dev, "%s:%d device init %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	ADI_LIBRARY_MEMSET(&phy->adi_adrv904x_device.devStateInfo, 0,
			   sizeof(phy->adi_adrv904x_device.devStateInfo));

	recoveryAction = adi_adrv904x_HwOpen(phy->kororDevice, &phy->spiSettings);
	if (recoveryAction != ADI_ADRV904X_ERR_ACT_NONE) {
		pr_err("ERROR adi_adrv904x_HwOpen failed in %s at line %d.\n", __func__,
		       __LINE__);
		return JESD204_STATE_CHANGE_ERROR;
	}

	adi_adrv904x_LogLevelSet(&phy->kororDevice->common,
				 ADI_HAL_LOG_ERR | ADI_HAL_LOG_WARN);

	recoveryAction = adi_adrv904x_HwReset(phy->kororDevice);

	recoveryAction = adi_adrv904x_PreMcsInit(phy->kororDevice, &deviceInitStruct,
						 &phy->trxBinaryInfoPtr);
	if (recoveryAction != ADI_ADRV904X_ERR_ACT_NONE) {
		pr_err("ERROR adi_adrv904x_PreMcsInit failed in %s at line %d.\n", __func__,
		       __LINE__);
		return JESD204_STATE_CHANGE_ERROR;
	}

	recoveryAction = adi_adrv904x_PreMcsInit_NonBroadcast(phy->kororDevice,
							      &deviceInitStruct);
	if (recoveryAction != ADI_ADRV904X_ERR_ACT_NONE) {
		pr_err("ERROR adi_adrv904x_PreMcsInit_NonBroadcast failed in %s at line %d.\n",
		       __func__, __LINE__);
		return JESD204_STATE_CHANGE_ERROR;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv904x_jesd204_link_init(struct jesd204_dev *jdev,
				      enum jesd204_state_op_reason reason,
				      struct jesd204_link *lnk)
{
	adi_adrv904x_ErrAction_e recoveryAction = ADI_ADRV904X_ERR_ACT_NONE;
	struct adrv904x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv904x_rf_phy *phy = priv->phy;
	u8 source_id;
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
		adi_adrv904x_DeframerCfg_t deframerCfg = { 0 };

		if (lnk->link_id == DEFRAMER0_LINK_TX)
			source_id = ADI_ADRV904X_DEFRAMER_0;
		else
			source_id = ADI_ADRV904X_DEFRAMER_1;

		recoveryAction = adi_adrv904x_DeframerCfgGet(phy->kororDevice, source_id, &deframerCfg);
		if (recoveryAction != ADI_ADRV904X_ERR_ACT_NONE) {
			pr_err("ERROR adi_adrv904x_DeframerCfgGet failed in %s at line %d.\n", __func__,
			       __LINE__);
			return JESD204_STATE_CHANGE_ERROR;
		}

		priv->link[lnk->link_id].source_id = source_id;
		phy->tx_iqRate_kHz  =
			phy->kororDevice->initExtract.jesdSetting.deframerSetting[source_id - 1].iqRate_kHz;
		rate = phy->kororDevice->initExtract.jesdSetting.deframerSetting[source_id - 1].iqRate_kHz;
		lnk->num_lanes =
			hweight8(phy->kororDevice->initExtract.jesdSetting.deframerSetting[source_id - 1].deserialLaneEnabled);
		lnk->num_converters =
			phy->kororDevice->initExtract.jesdSetting.deframerSetting[source_id - 1].jesdM;
		lnk->bits_per_sample =
			phy->kororDevice->initExtract.jesdSetting.deframerSetting[source_id - 1].jesdNp;

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
		adi_adrv904x_FramerCfg_t framerCfg = { 0 };

		if (lnk->link_id == FRAMER0_LINK_RX)
			source_id = ADI_ADRV904X_FRAMER_0;
		else if (lnk->link_id == FRAMER1_LINK_RX)
			source_id = ADI_ADRV904X_FRAMER_1;
		else
			source_id = ADI_ADRV904X_FRAMER_2;

		recoveryAction = adi_adrv904x_FramerCfgGet(phy->kororDevice, source_id, &framerCfg);
		if (recoveryAction != ADI_ADRV904X_ERR_ACT_NONE) {
			pr_err("ERROR adi_adrv904x_FramerCfgGet failed in %s at line %d.\n", __func__,
			       __LINE__);
			return JESD204_STATE_CHANGE_ERROR;
		}

		priv->link[lnk->link_id].source_id = source_id;
		priv->link[lnk->link_id].is_framer = true;
		phy->rx_iqRate_kHz  =
			phy->kororDevice->initExtract.jesdSetting.framerSetting[source_id - 1].iqRate_kHz;
		rate = phy->kororDevice->initExtract.jesdSetting.framerSetting[source_id - 1].iqRate_kHz;
		lnk->num_lanes =
			hweight8(phy->kororDevice->initExtract.jesdSetting.framerSetting[source_id - 1].serialLaneEnabled);
		lnk->num_converters =
			phy->kororDevice->initExtract.jesdSetting.framerSetting[source_id - 1].jesdM;
		lnk->bits_per_sample =
			phy->kororDevice->initExtract.jesdSetting.framerSetting[source_id - 1].jesdNp;

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

int adrv904x_jesd204_link_setup(struct jesd204_dev *jdev,
				enum jesd204_state_op_reason reason)
{
	struct adrv904x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv904x_rf_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	if (reason == JESD204_STATE_OP_REASON_UNINIT) {
		phy->is_initialized = 0;

		adi_adrv904x_Shutdown(phy->kororDevice);
		adi_adrv904x_HwClose(phy->kororDevice);

		memset(&phy->adi_adrv904x_device.devStateInfo, 0,
		       sizeof(phy->adi_adrv904x_device.devStateInfo));

		return JESD204_STATE_CHANGE_DONE;
	}

	ret = adi_adrv904x_MultichipSyncSet(phy->kororDevice, ADI_ENABLE);
	if (ret)
		return adrv904x_dev_err(phy);

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv904x_jesd204_setup_stage1(struct jesd204_dev *jdev,
					 enum jesd204_state_op_reason reason)
{
	struct adrv904x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv904x_rf_phy *phy = priv->phy;
	int ret, i;
	u32 mcsStatus;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	/* This loop will send SysRef pulses up to 255 times unless MCS status achieved before. */
	for (i = 0; i < 255; i++) {
		ret = adi_adrv904x_MultichipSyncStatusGet(phy->kororDevice,
							  &mcsStatus);
		if (ret)
			return adrv904x_dev_err(phy);

		if ((mcsStatus & 0x01) == 0x01)
			break;

		jesd204_sysref_async_force(phy->jdev);
	}

	if (mcsStatus != 0x01) {
		dev_err(&phy->spi->dev,
			"%s:%d Unexpected MCS sync status (0x%X)",
			__func__, __LINE__, mcsStatus);

		return adrv904x_dev_err(phy);
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv904x_jesd204_setup_stage2(struct jesd204_dev *jdev,
					 enum jesd204_state_op_reason reason)
{
	struct adrv904x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv904x_rf_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	/* MCS end sequence*/
	ret = adi_adrv904x_MultichipSyncSet(phy->kororDevice, ADI_DISABLE);
	if (ret)
		return adrv904x_dev_err(phy);

	/* Post MCS */
	ret = adi_adrv904x_PostMcsInit(phy->kororDevice,
				       &utilityInit);
	if (ret)
		return adrv904x_dev_err(phy);

	ret = adi_adrv904x_SerializerReset(phy->kororDevice);
	if (ret)
		return adrv904x_dev_err(phy);

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv904x_jesd204_clks_enable(struct jesd204_dev *jdev,
					enum jesd204_state_op_reason reason,
					struct jesd204_link *lnk)
{
	struct adrv904x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv904x_rf_phy *phy = priv->phy;
	int ret;

	dev_err(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	if (!lnk->num_converters)
		return JESD204_STATE_CHANGE_DONE;

	if (priv->link[lnk->link_id].is_framer) {
		if (phy->kororDevice->devStateInfo.linkSharingEnabled == 1) {
			ret = adi_adrv904x_FramerSysrefCtrlSet(phy->kororDevice,
							       ADI_ADRV904X_FRAMER_1,
							       ADI_DISABLE);
			if (ret)
				return adrv904x_dev_err(phy);

			ret = adi_adrv904x_FramerLinkStateSet(phy->kororDevice,
							      ADI_ADRV904X_FRAMER_1,
							      ADI_DISABLE);
			if (ret)
				return adrv904x_dev_err(phy);

			ret = adi_adrv904x_FramerLinkStateSet(phy->kororDevice,
							      ADI_ADRV904X_FRAMER_1,
							      ADI_ENABLE);
			if (ret)
				return adrv904x_dev_err(phy);

			dev_dbg(&phy->spi->dev,
				"%s:%d Link %d Framer enabled", __func__, __LINE__,
				ADI_ADRV904X_FRAMER_1);

			/*************************************************/
			/***** Enable SYSREF to Koror JESD204B Framer ****/
			/*************************************************/
			/*** < User: Make sure SYSREF is stopped/disabled > ***/
			ret = adi_adrv904x_FramerSysrefCtrlSet(phy->kororDevice,
							       ADI_ADRV904X_FRAMER_1,
							       ADI_ENABLE);
			if (ret)
				return adrv904x_dev_err(phy);

			jesd204_sysref_async_force(phy->jdev);

			ret = adi_adrv904x_FramerLinkStateSet(phy->kororDevice,
							      ADI_ADRV904X_FRAMER_1,
							      ADI_DISABLE);
			if (ret)
				return adrv904x_dev_err(phy);

			ret = adi_adrv904x_FramerSysrefCtrlSet(phy->kororDevice,
							       ADI_ADRV904X_FRAMER_1,
							       ADI_DISABLE);
			if (ret)
				return adrv904x_dev_err(phy);
		}

		ret = adi_adrv904x_FramerSysrefCtrlSet(phy->kororDevice,
						       priv->link[lnk->link_id].source_id,
						       ADI_ENABLE);
		if (ret)
			return adrv904x_dev_err(phy);

		ret = adi_adrv904x_FramerLinkStateSet(phy->kororDevice,
						      priv->link[lnk->link_id].source_id,
						      ADI_DISABLE);
		if (ret)
			return adrv904x_dev_err(phy);

		ret = adi_adrv904x_FramerLinkStateSet(phy->kororDevice,
						      priv->link[lnk->link_id].source_id,
						      ADI_ENABLE);
		if (ret)
			return adrv904x_dev_err(phy);

		dev_dbg(&phy->spi->dev,
			"%s:%d Link %d Framer enabled", __func__, __LINE__,
			priv->link[lnk->link_id].source_id);

		ret = adi_adrv904x_FramerSysrefCtrlSet(phy->kororDevice,
						       priv->link[lnk->link_id].source_id,
						       ADI_DISABLE);
		if (ret)
			return adrv904x_dev_err(phy);

	} else {
		ret = adi_adrv904x_DeframerSysrefCtrlSet(phy->kororDevice,
							 (uint8_t)ADI_ADRV904X_ALL_DEFRAMER,
							 ADI_DISABLE);
		if (ret)
			return adrv904x_dev_err(phy);

		ret = adi_adrv904x_DeframerLinkStateSet(phy->kororDevice,
							(uint8_t)ADI_ADRV904X_ALL_DEFRAMER,
							ADI_DISABLE);
		if (ret)
			return adrv904x_dev_err(phy);
	};

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv904x_jesd204_link_enable(struct jesd204_dev *jdev,
					enum jesd204_state_op_reason reason,
					struct jesd204_link *lnk)
{
	struct adrv904x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv904x_rf_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	if (!lnk->num_converters)
		return JESD204_STATE_CHANGE_DONE;

	if (!priv->link[lnk->link_id].is_framer) { /* DEFRAMER */
		adi_adrv904x_InitCals_t serdesCal = {
			.calMask = ADI_ADRV904X_IC_SERDES,
			.orxChannelMask = 0x00U,
			.rxChannelMask = 0xFFU,
			.txChannelMask = 0x00U,
			.warmBoot = 0,
		};

		ret = adi_adrv904x_DeframerLinkStateSet(phy->kororDevice,
							priv->link[lnk->link_id].source_id,
							ADI_ENABLE);
		if (ret)
			return adrv904x_dev_err(phy);

		/* Notify ARM to run SERDES Calbriation if necessary */
		ret = adi_adrv904x_InitCalsRun(phy->kororDevice, &serdesCal);
		if (ret)
			return adrv904x_dev_err(phy);

		/* Wait up to 60 seconds for ARM */
		ret = adi_adrv904x_InitCalsWait(phy->kororDevice, 60000);
		if (ret) {
			dev_err(&phy->spi->dev, "Error: InitCalsWait\n");
			return adrv904x_dev_err(phy);
		}

		/***********************************************************/
		/**** Enable SYSREF to Koror JESD204B/JESD204C Deframer ****/
		/***********************************************************/
		ret = adi_adrv904x_DeframerSysrefCtrlSet(phy->kororDevice,
							 priv->link[lnk->link_id].source_id,
							 ADI_DISABLE);
		if (ret)
			return adrv904x_dev_err(phy);

		ret = adi_adrv904x_DeframerLinkStateSet(phy->kororDevice,
							priv->link[lnk->link_id].source_id,
							ADI_ENABLE);
		if (ret)
			return adrv904x_dev_err(phy);

		ret = adi_adrv904x_DeframerSysrefCtrlSet(phy->kororDevice,
							 priv->link[lnk->link_id].source_id,
							 ADI_ENABLE);
		if (ret)
			return adrv904x_dev_err(phy);
	};

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv904x_jesd204_link_running(struct jesd204_dev *jdev,
					 enum jesd204_state_op_reason reason,
					 struct jesd204_link *lnk)
{
	struct adrv904x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv904x_rf_phy *phy = priv->phy;
	int ret;

	adi_adrv904x_DeframerStatus_v2_t deframerStatus;
	adi_adrv904x_FramerStatus_t framerStatus;
	u8 deframerLinkCondition = 0;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	if (!lnk->num_converters)
		return JESD204_STATE_CHANGE_DONE;

	if (priv->link[lnk->link_id].is_framer) {
		ret = adi_adrv904x_FramerStatusGet(phy->kororDevice,
						   priv->link[lnk->link_id].source_id,
						   &framerStatus);
		if (ret)
			return adrv904x_dev_err(phy);

		if (framerStatus.status != 0x82)
			dev_warn(&phy->spi->dev,
				 "Link%u framerStatus 0x%X",
				 lnk->link_id, framerStatus.status);
	} else {
		ret = adi_adrv904x_DeframerStatusGet_v2(phy->kororDevice,
							priv->link[lnk->link_id].source_id,
							&deframerStatus);
		if (ret)
			return adrv904x_dev_err(phy);

		ret  = adi_adrv904x_DfrmLinkConditionGet(phy->kororDevice,
							 priv->link[lnk->link_id].source_id,
							 &deframerLinkCondition);
		if (ret)
			return adrv904x_dev_err(phy);

		for (int i = 0; i < lnk->num_lanes; i++) {
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
ADI_API adi_adrv904x_InitCalibrations_e drv_cals_TrackingCalConvert(const adi_adrv904x_TrackingCalibrationMask_e trackingCal)
{
	adi_adrv904x_InitCalibrations_e initCal = (adi_adrv904x_InitCalibrations_e)0U;

	switch (trackingCal) {
	case ADI_ADRV904X_TC_TX_LOL_MASK:
		initCal = ADI_ADRV904X_IC_TXLOL;
		break;

	case ADI_ADRV904X_TC_TX_QEC_MASK:
		initCal = ADI_ADRV904X_IC_TXQEC;
		break;

	case ADI_ADRV904X_TC_TX_SERDES_MASK:
		initCal = ADI_ADRV904X_IC_SERDES;
		break;

	case ADI_ADRV904X_TC_RX_ADC_MASK:
		initCal = ADI_ADRV904X_IC_ADC_RX;
		break;

	case ADI_ADRV904X_TC_TX_LB_ADC_MASK:
		initCal = ADI_ADRV904X_IC_ADC_TXLB;
		break;

	case ADI_ADRV904X_TC_ORX_ADC_MASK:
		initCal = ADI_ADRV904X_IC_ADC_ORX;
		break;

	case ADI_ADRV904X_TC_RX_QEC_MASK:
		fallthrough;

	default:
		initCal = (adi_adrv904x_InitCalibrations_e) 0U;
		break;
	}

	return initCal;
}

static int adrv904x_jesd204_post_running_stage(struct jesd204_dev *jdev,
					       enum jesd204_state_op_reason reason)
{
	adi_adrv904x_TrackingCalibrationMask_e trackingCal = (adi_adrv904x_TrackingCalibrationMask_e) 0U;
	adi_adrv904x_InitCalibrations_e currentInitCalMask = (adi_adrv904x_InitCalibrations_e) 0U;
	struct adrv904x_jesd204_priv *priv = jesd204_dev_priv(jdev);
	static adi_adrv904x_InitCalStatus_t initCalStatus;
	struct device *dev = jesd204_dev_to_device(jdev);
	const u32 ALL_CHANNELS_MASK = 0xFFU;
	struct adrv904x_rf_phy *phy = priv->phy;
	adi_adrv904x_TxAtten_t txAttenuation[1];
	const u32 NUM_TRACKING_CALS = 7U;
	u8 i, j;
	int ret;

	const u32 trackingCalMask = (u32)(ADI_ADRV904X_TC_RX_ADC_MASK     |
						ADI_ADRV904X_TC_ORX_ADC_MASK    |
						ADI_ADRV904X_TC_TX_LB_ADC_MASK  |
						ADI_ADRV904X_TC_TX_SERDES_MASK);

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		phy->is_initialized = 0;
		return JESD204_STATE_CHANGE_DONE;
	}

	clk_set_rate(phy->clks[RX_SAMPL_CLK], phy->rx_iqRate_kHz * 1000);
	clk_set_rate(phy->clks[TX_SAMPL_CLK], phy->tx_iqRate_kHz * 1000);

	// Tx enabled only if EN toggles
	ret = adi_adrv904x_RxTxEnableSet(phy->kororDevice, 0x00, 0x00,
					 ADI_ADRV904X_RX_MASK_ALL,
					 ADI_ADRV904X_RX_MASK_ALL,
					 ADI_ADRV904X_TXALL, ADI_ADRV904X_TXALL);
	if (ret)
		return adrv904x_dev_err(phy);

	ret = adi_adrv904x_RxTxEnableSet(phy->kororDevice, 0x00, 0x00, 0x00,
					 0x00, ADI_ADRV904X_TXALL, 0x00);
	if (ret)
		return adrv904x_dev_err(phy);

	ret = adi_adrv904x_RxTxEnableSet(phy->kororDevice, 0x00, 0x00,
					 ADI_ADRV904X_RX_MASK_ALL,
					 ADI_ADRV904X_RX_MASK_ALL,
					 ADI_ADRV904X_TXALL, ADI_ADRV904X_TXALL);
	if (ret)
		return adrv904x_dev_err(phy);

	txAttenuation[0].txChannelMask = ADI_ADRV904X_TXALL;
	txAttenuation[0].txAttenuation_mdB = 6000;
	ret = adi_adrv904x_TxAttenSet(phy->kororDevice, txAttenuation, 1);
	if (ret)
		return adrv904x_dev_err(phy);

	memset(&initCalStatus, 0, sizeof(adi_adrv904x_InitCalStatus_t));

	ret = adi_adrv904x_InitCalsDetailedStatusGet(phy->kororDevice, &initCalStatus);
	if (ret)
		return adrv904x_dev_err(phy);

	for (i = 0U; i < NUM_TRACKING_CALS; ++i) {
		trackingCal = (adi_adrv904x_TrackingCalibrationMask_e)(1U << i);

		if (((uint32_t)trackingCal & trackingCalMask) == 0U) {
			/* Tracking Cal not configured to run */
			continue;
		}

		/* Check if the Current Tracking Cal was initially run */
		currentInitCalMask = drv_cals_TrackingCalConvert(trackingCal);

		for (j = 0U; j < ADI_ADRV904X_MAX_CHANNELS; ++j) {
			if ((initCalStatus.calsSincePowerUp[j] & currentInitCalMask) == 0U) {
				/* Tracking Cal was already run */
				continue;
			}
		}

		ret = adi_adrv904x_TrackingCalsEnableSet(phy->kororDevice,
							 trackingCal,
							 ALL_CHANNELS_MASK,
							 ADI_ADRV904X_TRACKING_CAL_ENABLE);
		if (ret)
			return adrv904x_dev_err(phy);
	}

	phy->is_initialized = 1;
	adrv904x_info(phy);

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_adrv904x_init = {
	.state_ops = {
		[JESD204_OP_DEVICE_INIT] = {
			.per_device = adrv904x_jesd204_device_init,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_LINK_INIT] = {
			.per_link = adrv904x_jesd204_link_init,
		},
		[JESD204_OP_LINK_PRE_SETUP] = {
			.per_device = adrv904x_jesd204_link_pre_setup,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_LINK_SETUP] = {
			.per_device = adrv904x_jesd204_link_setup,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
			.post_state_sysref = true,
		},
		[JESD204_OP_OPT_SETUP_STAGE1] = {
			.per_device = adrv904x_jesd204_setup_stage1,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
			.post_state_sysref = true,
		},
		[JESD204_OP_OPT_SETUP_STAGE2] = {
			.per_device = adrv904x_jesd204_setup_stage2,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
			.post_state_sysref = true,
		},
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = adrv904x_jesd204_clks_enable,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = adrv904x_jesd204_link_enable,
			.post_state_sysref = true,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = adrv904x_jesd204_link_running,
		},
		[JESD204_OP_OPT_POST_RUNNING_STAGE] = {
			.per_device = adrv904x_jesd204_post_running_stage,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
	},

	.max_num_links = 10,
	.sizeof_priv = sizeof(struct adrv904x_jesd204_priv),
};

static int adrv904x_probe(struct spi_device *spi)
{
	adi_adrv904x_ExtractInitDataOutput_e checkExtractInitData =
		ADI_ADRV904X_EXTRACT_INIT_DATA_NOT_POPULATED;
	struct device_node *np = spi->dev.of_node;
	adi_common_ErrData_t *errData = NULL;
	struct adrv904x_jesd204_priv *priv;
	adi_adrv904x_Version_t apiVersion;
	struct adrv904x_rf_phy *phy;
	struct iio_dev *indio_dev;
	struct jesd204_dev *jdev;
	struct clk *clk = NULL;
	const char *name;
	int ret, i;
	u32 val;

	int id = spi_get_device_id(spi)->driver_data;

	jdev = devm_jesd204_dev_register(&spi->dev, &jesd204_adrv904x_init);
	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

	clk = devm_clk_get(&spi->dev, "dev_clk");
	if (IS_ERR(clk))
		ret = PTR_ERR(clk);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*phy));
	if (!indio_dev)
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

	phy->kororDevice = &phy->adi_adrv904x_device;
	phy->linux_hal.spi = spi;
	phy->linux_hal.logCfg.logMask = ADI_HAL_LOG_ERR | ADI_HAL_LOG_WARN;
	phy->kororDevice->common.devHalInfo = &phy->linux_hal;
	phy->kororDevice->common.type = "RF";

	if (!of_property_read_string(np, "adi,device-config-name", &name)) {
		strscpy(phy->trxBinaryInfoPtr.cpuProfile.filePath, name, sizeof(phy->trxBinaryInfoPtr.cpuProfile.filePath));
	} else {
		dev_err(&spi->dev, "error missing dt property: adi,device-config-name\n");
		return -EINVAL;
	}

	if (!of_property_read_string(np, "adi,arm-firmware-name", &name)) {
		strscpy(phy->trxBinaryInfoPtr.cpu.filePath, name, sizeof(phy->trxBinaryInfoPtr.cpu.filePath));
	} else {
		dev_err(&spi->dev, "error missing dt property: adi,arm-firmware-name\n");
		return -EINVAL;
	}

	if (!of_property_read_string(np, "adi,arm-dfe-firmware-name", &name)) {
		strscpy(phy->trxBinaryInfoPtr.dfeCpu.filePath, name, sizeof(phy->trxBinaryInfoPtr.dfeCpu.filePath));
	} else {
		dev_err(&spi->dev, "error missing dt property: adi,arm-dfe-firmware-name\n");
		return -EINVAL;
	}

	if (!of_property_read_string(np, "adi,stream-firmware-name", &name)) {
		strscpy(phy->trxBinaryInfoPtr.stream.filePath, name, sizeof(phy->trxBinaryInfoPtr.stream.filePath));
	} else {
		dev_err(&spi->dev, "error missing dt property: adi,stream-firmware-name\n");
		return -EINVAL;
	}

	for (i = 0; i < ADI_ADRV904X_RX_GAIN_TABLE_ARR_MAX; i++) {
		ret = of_property_read_string_index(np, "adi,rx-gaintable-names", i, &name);

		if (!ret && !of_property_read_u32_index(np, "adi,rx-gaintable-channel-masks", i, &val)) {
			strscpy(phy->trxBinaryInfoPtr.rxGainTable[i].filePath, name,
				sizeof(phy->trxBinaryInfoPtr.rxGainTable[i].filePath));
			phy->trxBinaryInfoPtr.rxGainTable[i].channelMask = val;
		}
	}

	if (adrv904x_hal_PlatformSetup(ADI_LINUX) != ADI_HAL_ERR_OK) {
		dev_err(&spi->dev, "error HAL function(s) not implemented\n");
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
	phy->spiSettings.cmosPadDrvStrength = ADI_ADRV904X_CMOSPAD_DRV_STRONG;
	phy->linux_hal.spiCfg.interfaceEnabled = 1;

	adi_adrv904x_LogLevelSet(&phy->kororDevice->common, ADI_HAL_LOG_ALL);

	/* Register errData to use for API calls */
	errData = ADI_CREATE_ERROR_MEMORY();
	if (!errData)
		return -1;

	ret = adi_hal_TlsSet(HAL_TLS_ERR, errData);
	if (ret != ADI_HAL_ERR_OK) {
		(void)ADI_DESTROY_ERROR_MEMORY(errData);
		return -1;
	}

	phy->kororDevice->common.deviceInfo.id = id;
	phy->kororDevice->common.deviceInfo.name = spi_get_device_id(spi)->name;
	phy->kororDevice->common.deviceInfo.type = 0x00;

	ret = adi_adrv904x_HwOpen(phy->kororDevice, &phy->spiSettings);
	if (ret)
		return adrv904x_dev_err(phy);

	ret = adi_adrv904x_SpiVerify(phy->kororDevice);
	if (ret)
		return adrv904x_dev_err(phy);

	ret = adi_adrv904x_ApiVersionGet(phy->kororDevice, &apiVersion);
	if (ret)
		return adrv904x_dev_err(phy);

	dev_info(&spi->dev,
		 "%s Rev %d, API version: %u.%u.%u.%u found",
		 spi_get_device_id(spi)->name,
		 phy->kororDevice->devStateInfo.deviceSiRev,
		 apiVersion.majorVer, apiVersion.minorVer,
		 apiVersion.maintenanceVer, apiVersion.buildVer);

	if (apiVersion.majorVer > 1U) {
		ret = adi_adrv904x_InitDataExtract(phy->kororDevice,
						   &phy->trxBinaryInfoPtr.cpuProfile,
						   &initStructApiVersion,
						   &initStructArmVersion,
						   &initStructStreamVersion,
						   &deviceInitStruct,
						   &utilityInit,
						   &checkExtractInitData);

		switch (checkExtractInitData) {
		case ADI_ADRV904X_EXTRACT_INIT_DATA_LEGACY_PROFILE_BIN:
			pr_err("\n\tUsing the Default Init and PostMcsInit Structures\n");
			break;

		case ADI_ADRV904X_EXTRACT_INIT_DATA_POPULATED:
			pr_err("\n\tUsing the Profile Init and PostMcsInit Structures\n");
			break;

		case ADI_ADRV904X_EXTRACT_INIT_DATA_NOT_POPULATED:
			fallthrough;

		default:
			ADI_APP_ERROR_REPORT(ADI_COMMON_ERRCODE_INVALID_PARAM,
					     ret,
					     &deviceInitStruct,
					     "PreMcsInit and/or PostMcsInit Data Structures Not Populated");
			return -EINVAL;
		}
	}

	/* Copy the data structure that holds Utility Init structures to the device */
	memcpy(&phy->adrv904xPostMcsInitInst, &utilityInit, sizeof(adi_adrv904x_PostMcsInit_t));

	/* Extract Info from CPU Profile Binary */
	/* Required for Link init */
	ret = adi_adrv904x_DeviceInfoExtract(phy->kororDevice,
					     &phy->trxBinaryInfoPtr.cpuProfile);
	if (ret != ADI_ADRV904X_ERR_ACT_NONE) {
		ADI_API_ERROR_REPORT(&phy->kororDevice->common, ret,
				     "Issue during CPU Profile Binary Image Extract");
		return -1;
	}

	adrv904x_clk_register(phy, "-rx_sampl_clk", __clk_get_name(phy->dev_clk), NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			      RX_SAMPL_CLK);

	adrv904x_clk_register(phy, "-tx_sampl_clk", __clk_get_name(phy->dev_clk), NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			      TX_SAMPL_CLK);

	phy->clk_data.clks = phy->clks;
	phy->clk_data.clk_num = NUM_ADRV904X_CLKS;

	ret = of_clk_add_provider(np, of_clk_src_onecell_get,
				  &phy->clk_data);
	if (ret)
		goto out_disable_clocks;

	indio_dev->dev.parent = &spi->dev;

	if (np)
		indio_dev->name = np->name;
	else
		indio_dev->name = "adrv904x-phy";

	indio_dev->modes = INDIO_DIRECT_MODE;

	switch (id) {
	case ID_ADRV9040:
		indio_dev->info = &adrv904x_phy_info;
		indio_dev->channels = adrv904x_phy_chan;
		indio_dev->num_channels = ARRAY_SIZE(adrv904x_phy_chan);
		break;
	default:
		ret = -EINVAL;
		goto out_clk_del_provider;
	}

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto out_clk_del_provider;

	ret = adrv904x_register_axi_converter(phy);
	if (ret < 0)
		goto out_iio_device_unregister;

	ret = adrv904x_register_debugfs(indio_dev);
	if (ret < 0)
		dev_warn(&spi->dev, "%s: failed to register debugfs", __func__);

	adi_adrv904x_ApiVersionGet(phy->kororDevice, &apiVersion);
	adi_adrv904x_HwClose(phy->kororDevice);

	dev_info(&spi->dev,
		 "%s Rev %d, API version: %u.%u.%u.%u found",
		 spi_get_device_id(spi)->name,
		 phy->kororDevice->devStateInfo.deviceSiRev,
		 apiVersion.majorVer, apiVersion.minorVer,
		 apiVersion.maintenanceVer, apiVersion.buildVer);

	ret = jesd204_fsm_start(phy->jdev, JESD204_LINKS_ALL);
	if (ret)
		goto out_iio_device_unregister;

	return 0;

out_iio_device_unregister:
	iio_device_unregister(indio_dev);
out_clk_del_provider:
	of_clk_del_provider(np);
out_disable_clocks:
	clk_disable_unprepare(phy->dev_clk);

	return ret;
}

static void adrv904x_remove(struct spi_device *spi)
{
	struct adrv904x_rf_phy *phy = adrv904x_spi_to_phy(spi);

	iio_device_unregister(phy->indio_dev);
	of_clk_del_provider(spi->dev.of_node);
	clk_disable_unprepare(phy->dev_clk);

	adrv904x_shutdown(phy);
}

static const struct spi_device_id adrv904x_id[] = {
	{ "adrv9040", ID_ADRV9040 },
	{}
};
MODULE_DEVICE_TABLE(spi, adrv904x_id);

static const struct of_device_id adrv904x_of_match[] = {
	{ .compatible = "adi,adrv9040" },
	{},
};
MODULE_DEVICE_TABLE(of, adrv904x_of_match);

static struct spi_driver adrv904x_driver = {
	.driver = {
			.name = "adrv9040",
			.of_match_table = of_match_ptr(adrv904x_of_match),
		},
	.probe = adrv904x_probe,
	.remove = adrv904x_remove,
	.id_table = adrv904x_id,
};
module_spi_driver(adrv904x_driver);

MODULE_AUTHOR("George Mois <george.mois@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADRV904X TRX");
MODULE_LICENSE("GPL v2");
