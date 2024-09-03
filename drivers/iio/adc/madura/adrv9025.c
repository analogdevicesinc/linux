// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9025/6 RF Transceiver
 *
 * Copyright 2020-2023 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/types.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>

#include <asm/unaligned.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include <linux/jesd204/jesd204.h>

#include <dt-bindings/iio/adc/adi,adrv9025.h>

#include "adrv9025.h"

enum adrv9025_iio_dev_attr {
	ADRV9025_INIT_CAL,
	adrv9025_JESD204_FSM_ERROR,
	adrv9025_JESD204_FSM_PAUSED,
	adrv9025_JESD204_FSM_STATE,
	adrv9025_JESD204_FSM_RESUME,
	adrv9025_JESD204_FSM_CTRL,
};

static int __adrv9025_dev_err(struct adrv9025_rf_phy *phy, const char *function,
			      const int line)
{
	int ret;

	dev_err(&phy->spi->dev, "%s, %d: failed with %s (%d)\n", function, line,
		phy->madDevice->common.error.errormessage ?
			phy->madDevice->common.error.errormessage :
			"",
		phy->madDevice->common.error.errCode);

	switch (phy->madDevice->common.error.errCode) {
	case ADI_COMMON_ERR_INV_PARAM:
	case ADI_COMMON_ERR_NULL_PARAM:
		ret = -EINVAL;
		break;
	case ADI_COMMON_ERR_API_FAIL:
		ret = -EFAULT;
		break;
	case ADI_COMMON_ERR_SPI_FAIL:
		ret = -EIO;
		break;
	case ADI_COMMON_ERR_OK:
		ret = 0;
		break;
	default:
		ret = -EFAULT;
	}

	adrv9025_ErrorClear(&phy->madDevice->common);

	return ret;
}

#define adrv9025_dev_err(phy) __adrv9025_dev_err(phy, __func__, __LINE__)

int adrv9025_spi_read(struct spi_device *spi, unsigned int reg)
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

int adrv9025_spi_write(struct spi_device *spi, unsigned int reg, unsigned int val)
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

int adrv9025_RxLinkSamplingRateFind(adi_adrv9025_Device_t *device,
				    adi_adrv9025_Init_t *adrv9025Init,
				    adi_adrv9025_FramerSel_e framerSel,
				    u32 *iqRate_kHz)
{
	int recoveryAction = ADI_COMMON_ACT_NO_ACTION;
	adi_adrv9025_AdcSampleXbarSel_e conv = ADI_ADRV9025_ADC_RX1_Q;
	u32 framerIndex = 0;

	/* Check device pointer is not null */
	ADI_NULL_DEVICE_PTR_RETURN(device);
	ADI_NULL_PTR_RETURN(&device->common, iqRate_kHz);

	ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_API);

	switch (framerSel) {
	case ADI_ADRV9025_FRAMER_0:
		framerIndex = 0;
		break;
	case ADI_ADRV9025_FRAMER_1:
		framerIndex = 1;
		break;
	case ADI_ADRV9025_FRAMER_2:
		framerIndex = 2;
		break;
	default:
		ADI_ERROR_REPORT(&device->common, ADI_COMMON_ERRSRC_API,
				 ADI_COMMON_ERR_INV_PARAM,
				 ADI_COMMON_ACT_ERR_CHECK_PARAM, framerSel,
				 "Only one framer can be selected at a time.");
		ADI_ERROR_RETURN(device->common.error.newAction);
		break;
	}

	if (adrv9025Init->dataInterface.framer[framerIndex].jesd204M < 1) {
		*iqRate_kHz = 0;
		return recoveryAction;
	}

	conv = adrv9025Init->dataInterface.framer[framerIndex]
		       .adcCrossbar.conv0;

	switch (conv) {
	case ADI_ADRV9025_ADC_RX1_I: /* fall through */
	case ADI_ADRV9025_ADC_RX1_Q: /* fall through */
	case ADI_ADRV9025_ADC_DUALBAND_RX1_BAND_B_I: /* fall through */
	case ADI_ADRV9025_ADC_DUALBAND_RX1_BAND_B_Q: /* fall through */
		*iqRate_kHz = adrv9025Init->rx.rxChannelCfg[0]
				      .profile.rxOutputRate_kHz;
		break;
	case ADI_ADRV9025_ADC_RX2_I: /* fall through */
	case ADI_ADRV9025_ADC_RX2_Q: /* fall through */
	case ADI_ADRV9025_ADC_DUALBAND_RX2_BAND_B_I: /* fall through */
	case ADI_ADRV9025_ADC_DUALBAND_RX2_BAND_B_Q: /* fall through */
		*iqRate_kHz = adrv9025Init->rx.rxChannelCfg[1]
				      .profile.rxOutputRate_kHz;
		break;
	case ADI_ADRV9025_ADC_RX3_I: /* fall through */
	case ADI_ADRV9025_ADC_RX3_Q: /* fall through */
	case ADI_ADRV9025_ADC_DUALBAND_RX3_BAND_B_I: /* fall through */
	case ADI_ADRV9025_ADC_DUALBAND_RX3_BAND_B_Q: /* fall through */
		*iqRate_kHz = adrv9025Init->rx.rxChannelCfg[2]
				      .profile.rxOutputRate_kHz;
		break;
	case ADI_ADRV9025_ADC_RX4_I: /* fall through */
	case ADI_ADRV9025_ADC_RX4_Q: /* fall through */
	case ADI_ADRV9025_ADC_DUALBAND_RX4_BAND_B_I: /* fall through */
	case ADI_ADRV9025_ADC_DUALBAND_RX4_BAND_B_Q: /* fall through */
		*iqRate_kHz = adrv9025Init->rx.rxChannelCfg[3]
				      .profile.rxOutputRate_kHz;
		break;
		// NOTE: ORx2/4 profiles never referenced, ORx2IQ enum below refers to digital channel not RF input
		// RF ORx1/2 share digital ORX1, and RF ORX3/4 share digital ORX2
	case ADI_ADRV9025_ADC_ORX1_I: /* fall through */
	case ADI_ADRV9025_ADC_ORX1_Q: /* fall through */
		*iqRate_kHz = adrv9025Init->rx.rxChannelCfg[4]
				      .profile.rxOutputRate_kHz;
		break;
		// NOTE: ORx2/4 profiles never referenced, ORx2IQ enum below refers to digital channel not RF input
		// RF ORx1/2 share digital ORX1, and RF ORX3/4 share digital ORX2
	case ADI_ADRV9025_ADC_ORX2_I: /* fall through */
	case ADI_ADRV9025_ADC_ORX2_Q: /* fall through */
		*iqRate_kHz = adrv9025Init->rx.rxChannelCfg[6]
				      .profile.rxOutputRate_kHz;
		break;
	default:
		*iqRate_kHz = 0;
		ADI_ERROR_REPORT(
			&device->common, ADI_COMMON_ERRSRC_API,
			ADI_COMMON_ERR_INV_PARAM,
			ADI_COMMON_ACT_ERR_CHECK_PARAM, adcCrossbar.conv0,
			"Invalid ADC crossbar used to read iqRate_kHz");
		ADI_ERROR_RETURN(device->common.error.newAction);
	}

	return recoveryAction;
}

int adrv9025_TxLinkSamplingRateFind(adi_adrv9025_Device_t *device,
				    adi_adrv9025_Init_t *adrv9025Init,
				    adi_adrv9025_DeframerSel_e deframerSel,
				    u32 *iqRate_kHz)
{
	int recoveryAction = ADI_COMMON_ACT_NO_ACTION;
	u32 deframerIndex = 0;

	/* Check device pointer is not null */
	ADI_NULL_DEVICE_PTR_RETURN(device);
	ADI_NULL_PTR_RETURN(&device->common, iqRate_kHz);

	ADI_FUNCTION_ENTRY_LOG(&device->common, ADI_COMMON_LOG_API);

	switch (deframerSel) {
	case ADI_ADRV9025_DEFRAMER_0:
		deframerIndex = 0;
		break;
	case ADI_ADRV9025_DEFRAMER_1:
		deframerIndex = 1;
		break;
	default:
		ADI_ERROR_REPORT(
			&device->common, ADI_COMMON_ERRSRC_API,
			ADI_COMMON_ERR_INV_PARAM,
			ADI_COMMON_ACT_ERR_CHECK_PARAM, deframerSel,
			"Only one deframer can be selected at a time.");
		ADI_ERROR_RETURN(device->common.error.newAction);
		break;
	}

	if (adrv9025Init->dataInterface.deframer[deframerIndex].jesd204M < 1) {
		*iqRate_kHz = 0;
		return recoveryAction;
	}

	//Use samplerate of DAC set to use deframer output 0.
	if ((adrv9025Init->dataInterface.deframer[deframerIndex]
		     .dacCrossbar.tx1DacChanI == ADI_ADRV9025_DEFRAMER_OUT0) ||
	    (adrv9025Init->dataInterface.deframer[deframerIndex]
		     .dacCrossbar.tx1DacChanQ == ADI_ADRV9025_DEFRAMER_OUT0)) {
		*iqRate_kHz = adrv9025Init->tx.txChannelCfg[0]
				      .profile.txInputRate_kHz;
	} else if ((adrv9025Init->dataInterface.deframer[deframerIndex]
			    .dacCrossbar.tx2DacChanI ==
		    ADI_ADRV9025_DEFRAMER_OUT0) ||
		   (adrv9025Init->dataInterface.deframer[deframerIndex]
			    .dacCrossbar.tx2DacChanQ ==
		    ADI_ADRV9025_DEFRAMER_OUT0)) {
		*iqRate_kHz = adrv9025Init->tx.txChannelCfg[1]
				      .profile.txInputRate_kHz;
	} else if ((adrv9025Init->dataInterface.deframer[deframerIndex]
			    .dacCrossbar.tx3DacChanI ==
		    ADI_ADRV9025_DEFRAMER_OUT0) ||
		   (adrv9025Init->dataInterface.deframer[deframerIndex]
			    .dacCrossbar.tx3DacChanQ ==
		    ADI_ADRV9025_DEFRAMER_OUT0)) {
		*iqRate_kHz = adrv9025Init->tx.txChannelCfg[2]
				      .profile.txInputRate_kHz;
	} else if ((adrv9025Init->dataInterface.deframer[deframerIndex]
			    .dacCrossbar.tx4DacChanI ==
		    ADI_ADRV9025_DEFRAMER_OUT0) ||
		   (adrv9025Init->dataInterface.deframer[deframerIndex]
			    .dacCrossbar.tx4DacChanQ ==
		    ADI_ADRV9025_DEFRAMER_OUT0)) {
		*iqRate_kHz = adrv9025Init->tx.txChannelCfg[3]
				      .profile.txInputRate_kHz;
	}

	return recoveryAction;
}

static void adrv9025_shutdown(struct adrv9025_rf_phy *phy)
{
	/***********************************************
	 * Shutdown Procedure *
	 * **********************************************/
	/* Function to turn radio on, Disables transmitters and receivers */

	adi_adrv9025_Shutdown(phy->madDevice);
	adi_adrv9025_HwClose(phy->madDevice);

	memset(&phy->adi_adrv9025_device.devStateInfo, 0,
	       sizeof(phy->adi_adrv9025_device.devStateInfo));
}

static ssize_t adrv9025_phy_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);
	bool enable;
	int ret = 0;
	u64 val;

	mutex_lock(&phy->lock);

	switch ((u32)this_attr->address & 0xFF) {
	case ADRV9025_INIT_CAL:
		ret = strtobool(buf, &enable);
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
			u8 initCalsError = 0;

			phy->cal_mask.channelMask =
				phy->adrv9025PostMcsInitInst.initCals.channelMask;

			/* Run Init Cals */
			ret = adi_adrv9025_InitCalsRun(phy->madDevice,
						       &phy->cal_mask);
			if (ret) {
				ret = adrv9025_dev_err(phy);
				break;
			}

			ret = adi_adrv9025_InitCalsWait(phy->madDevice,
							INIT_CALS_TIMEOUT_MS,
							&initCalsError);
			if (ret)
				ret = adrv9025_dev_err(phy);
		}
		break;
	case adrv9025_JESD204_FSM_RESUME:
		if (!phy->jdev) {
			ret = -ENOTSUPP;
			break;
		}

		ret = jesd204_fsm_resume(phy->jdev, JESD204_LINKS_ALL);
		break;
	case adrv9025_JESD204_FSM_CTRL:
		if (!phy->jdev) {
			ret = -ENOTSUPP;
			break;
		}

		ret = strtobool(buf, &enable);
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

static ssize_t adrv9025_phy_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);
	struct jesd204_dev *jdev = phy->jdev;
	struct jesd204_link *links[5];
	int ret = 0, i, err, num_links;
	bool paused;
	u64 val;

	mutex_lock(&phy->lock);
	switch ((u32)this_attr->address & 0xFF) {
	case ADRV9025_INIT_CAL:
		val = this_attr->address >> 8;

		if (val)
			ret = sprintf(buf, "%d\n",
				      !!(phy->cal_mask.calMask & val));
		break;
	case adrv9025_JESD204_FSM_ERROR:
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
	case adrv9025_JESD204_FSM_PAUSED:
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
	case adrv9025_JESD204_FSM_STATE:
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
		 * and that adrv9025_JESD204_FSM_PAUSED was called before
		 */
		ret = sprintf(buf, "%s\n", jesd204_link_get_state_str(links[0]));
		break;
	case adrv9025_JESD204_FSM_CTRL:
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

static IIO_DEVICE_ATTR(calibrate, 0644, adrv9025_phy_show,
		       adrv9025_phy_store, ADRV9025_INIT_CAL);

static IIO_DEVICE_ATTR(calibrate_rx_qec_en, 0644,
		       adrv9025_phy_show, adrv9025_phy_store,
		       ADRV9025_INIT_CAL | (ADI_ADRV9025_RX_QEC_INIT << 8));

static IIO_DEVICE_ATTR(calibrate_tx_qec_en, 0644,
		       adrv9025_phy_show, adrv9025_phy_store,
		       ADRV9025_INIT_CAL | (ADI_ADRV9025_TX_QEC_INIT << 8));

static IIO_DEVICE_ATTR(calibrate_tx_lol_en, 0644,
		       adrv9025_phy_show, adrv9025_phy_store,
		       ADRV9025_INIT_CAL |
			       (ADI_ADRV9025_TX_LO_LEAKAGE_INTERNAL << 8));

static IIO_DEVICE_ATTR(calibrate_tx_lol_ext_en, 0644,
		       adrv9025_phy_show, adrv9025_phy_store,
		       ADRV9025_INIT_CAL |
			       (ADI_ADRV9025_TX_LO_LEAKAGE_EXTERNAL << 8));

static IIO_DEVICE_ATTR(jesd204_fsm_error, 0444,
		       adrv9025_phy_show,
		       NULL,
		       adrv9025_JESD204_FSM_ERROR);

static IIO_DEVICE_ATTR(jesd204_fsm_paused, 0444,
		       adrv9025_phy_show,
		       NULL,
		       adrv9025_JESD204_FSM_PAUSED);

static IIO_DEVICE_ATTR(jesd204_fsm_state, 0444,
		       adrv9025_phy_show,
		       NULL,
		       adrv9025_JESD204_FSM_STATE);

static IIO_DEVICE_ATTR(jesd204_fsm_resume, 0200,
		       NULL,
		       adrv9025_phy_store,
		       adrv9025_JESD204_FSM_RESUME);

static IIO_DEVICE_ATTR(jesd204_fsm_ctrl, 0644,
		       adrv9025_phy_show,
		       adrv9025_phy_store,
		       adrv9025_JESD204_FSM_CTRL);

static struct attribute *adrv9025_phy_attributes[] = {
	&iio_dev_attr_calibrate.dev_attr.attr,
	&iio_dev_attr_calibrate_rx_qec_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_qec_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_lol_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_lol_ext_en.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_error.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_state.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_paused.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_resume.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_ctrl.dev_attr.attr,
	NULL,
};

static const struct attribute_group adrv9025_phy_attribute_group = {
	.attrs = adrv9025_phy_attributes,
};

static int adrv9025_phy_reg_access(struct iio_dev *indio_dev, u32 reg,
				   u32 writeval, u32 *readval)
{
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&phy->lock);
	if (readval == NULL)
		ret = adrv9025_spi_write(phy->spi, reg, writeval);
	else {
		*readval = adrv9025_spi_read(phy->spi, reg);
		ret = 0;
	}
	mutex_unlock(&phy->lock);

	return ret;
}

enum lo_ext_info {
	LOEXT_FREQ,
};

static ssize_t adrv9025_phy_lo_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);
	u64 readin;
	int ret = 0;

	switch (private) {
	case LOEXT_FREQ:

		ret = kstrtoull(buf, 10, &readin);
		if (ret)
			return ret;

		mutex_lock(&phy->lock);

		ret = adi_adrv9025_PllFrequencySet(
			phy->madDevice, ADI_ADRV9025_LO1_PLL + chan->channel,
			readin);
		if (ret)
			ret = adrv9025_dev_err(phy);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&phy->lock);

	return ret ? ret : len;
}

static ssize_t adrv9025_phy_lo_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);

	u64 val;
	int ret;

	mutex_lock(&phy->lock);
	switch (private) {
	case LOEXT_FREQ:
		ret = adi_adrv9025_PllFrequencyGet(
			phy->madDevice, ADI_ADRV9025_LO1_PLL + chan->channel,
			&val);
		if (ret)
			ret = adrv9025_dev_err(phy);
		break;
	default:
		ret = 0;
	}
	mutex_unlock(&phy->lock);

	return ret ? ret : sprintf(buf, "%llu\n", val);
}

#define _ADRV9025_EXT_LO_INFO(_name, _ident)                                   \
	{                                                                      \
		.name = _name, .read = adrv9025_phy_lo_read,                   \
		.write = adrv9025_phy_lo_write, .private = _ident,             \
	}

static const struct iio_chan_spec_ext_info adrv9025_phy_ext_lo_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADRV9025_EXT_LO_INFO("frequency", LOEXT_FREQ),
	{},
};

static const struct iio_chan_spec_ext_info adrv9025_phy_ext_auxlo_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADRV9025_EXT_LO_INFO("frequency", LOEXT_FREQ),
	{},
};

static int adrv9025_set_agc_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan, u32 mode)
{
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv9025_RxAgcMode_t gainMode;
	int ret;

	switch (mode) {
	case 0:
		gainMode.agcMode = ADI_ADRV9025_MGC;
		break;
	case 1:
		gainMode.agcMode = ADI_ADRV9025_AGCFAST;
		break;
	case 2:
		gainMode.agcMode = ADI_ADRV9025_AGCSLOW;
		break;
	case 3:
		gainMode.agcMode = ADI_ADRV9025_HYBRID;
		break;
	default:
		return -EINVAL;
	}

	gainMode.rxChannelMask = ADI_ADRV9025_RX1 << chan->channel;

	ret = adi_adrv9025_RxGainCtrlModeSet(phy->madDevice, &gainMode, 1);
	if (ret)
		return adrv9025_dev_err(phy);

	return 0;
}

static int adrv9025_get_agc_mode(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv9025_RxAgcMode_t gainMode;
	int ret;

	ret = adi_adrv9025_RxGainCtrlModeGet(
		phy->madDevice, ADI_ADRV9025_RX1 << chan->channel, &gainMode);
	if (ret)
		return adrv9025_dev_err(phy);

	return gainMode.agcMode;
}

static const char *const adrv9025_agc_modes[] = { "manual", "fast_attack",
						  "slow_loop", "hybrid" };

static const struct iio_enum adrv9025_agc_modes_available = {
	.items = adrv9025_agc_modes,
	.num_items = ARRAY_SIZE(adrv9025_agc_modes),
	.get = adrv9025_get_agc_mode,
	.set = adrv9025_set_agc_mode,

};

static ssize_t adrv9025_phy_rx_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);

	bool enable;
	int ret = 0;
	u64 mask;
	u16 mask16;

	ret = strtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&phy->lock);

	switch (private) {
	case RSSI:
		break;
	case RX_QEC:
		mask = ADI_ADRV9025_TRACK_RX1_QEC << chan->channel;

		ret = adi_adrv9025_TrackingCalsEnableSet(
			phy->madDevice, mask,
			enable ? ADI_ADRV9025_TRACKING_CAL_ENABLE :
				 ADI_ADRV9025_TRACKING_CAL_DISABLE);
		if (ret)
			ret = adrv9025_dev_err(phy);
		break;
	case RX_HD2:
		mask = ADI_ADRV9025_TRACK_RX1_HD2 << chan->channel;

		ret = adi_adrv9025_TrackingCalsEnableSet(
			phy->madDevice, mask,
			enable ? ADI_ADRV9025_TRACKING_CAL_ENABLE :
				 ADI_ADRV9025_TRACKING_CAL_DISABLE);
		if (ret)
			ret = adrv9025_dev_err(phy);
		break;
	case RX_DIG_DC:
		ret = adi_adrv9025_DigDcOffsetEnableGet(phy->madDevice, &mask16);
		if (ret) {
			ret = adrv9025_dev_err(phy);
			goto out;
		}

		if (enable)
			mask16 |= (ADI_ADRV9025_MSHIFT_DC_OFFSET_RX_CH0 << chan->channel);
		else
			mask16 &= ~(ADI_ADRV9025_MSHIFT_DC_OFFSET_RX_CH0 << chan->channel);

		ret = adi_adrv9025_DigDcOffsetEnableSet(phy->madDevice, mask16);
		if (ret)
			ret = adrv9025_dev_err(phy);
		break;
	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&phy->lock);

	return ret ? ret : len;
}

static ssize_t adrv9025_phy_rx_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);
	int ret = 0;
	u16 dec_pwr_mdb, mask16;
	u64 tmask, mask;
	u32 rxchan = 0;

	mutex_lock(&phy->lock);

	switch (private) {
	case RSSI:
		rxchan = ADI_ADRV9025_RX1 << chan->channel;

		ret = adi_adrv9025_RxDecPowerGet(phy->madDevice, rxchan,
						 &dec_pwr_mdb);
		if (ret == 0)
			ret = sprintf(buf, "%u.%02u dB\n", dec_pwr_mdb / 1000,
				      dec_pwr_mdb % 1000);
		else
			ret = adrv9025_dev_err(phy);

		break;
	case RX_QEC:
		tmask = ADI_ADRV9025_TRACK_RX1_QEC << chan->channel;
		ret = adi_adrv9025_TrackingCalsEnableGet(phy->madDevice, &mask);
		if (ret == 0)
			ret = sprintf(buf, "%d\n", !!(tmask & mask));

		break;
	case RX_HD2:
		tmask = ADI_ADRV9025_TRACK_RX1_HD2 << chan->channel;
		ret = adi_adrv9025_TrackingCalsEnableGet(phy->madDevice, &mask);
		if (ret == 0)
			ret = sprintf(buf, "%d\n", !!(tmask & mask));

		break;
	case RX_DIG_DC:
		ret = adi_adrv9025_DigDcOffsetEnableGet(phy->madDevice, &mask16);

		if (ret == 0)
			ret = sprintf(buf, "%d\n",
				!!((ADI_ADRV9025_MSHIFT_DC_OFFSET_RX_CH0 << chan->channel) & mask16));
		else
			ret = adrv9025_dev_err(phy);
		break;
	case RX_RF_BANDWIDTH:
		ret = sprintf(buf, "%u\n",
			phy->deviceInitStruct.rx.rxChannelCfg[chan->channel].profile.rfBandwidth_kHz *
			1000);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	return ret;
}

#define _ADRV9025_EXT_RX_INFO(_name, _ident)                                   \
	{                                                                      \
		.name = _name, .read = adrv9025_phy_rx_read,                   \
		.write = adrv9025_phy_rx_write, .private = _ident,             \
	}

static ssize_t adrv9025_phy_tx_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);
	u64 tmask, mask;
	int val, ret = 0;

	if (chan->channel > CHAN_TX4)
		return -EINVAL;

	mutex_lock(&phy->lock);
	switch (private) {
	case TX_QEC:
		tmask = ADI_ADRV9025_TRACK_TX1_QEC << chan->channel;
		ret = adi_adrv9025_TrackingCalsEnableGet(phy->madDevice, &mask);
		val = !!(tmask & mask);
		break;
	case TX_LOL:
		tmask = ADI_ADRV9025_TRACK_TX1_LOL << chan->channel;
		ret = adi_adrv9025_TrackingCalsEnableGet(phy->madDevice, &mask);
		val = !!(tmask & mask);
		break;
	case TX_RF_BANDWIDTH:
		val = phy->deviceInitStruct.tx.txChannelCfg[chan->channel]
			      .profile.rfBandwidth_kHz *
		      1000;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	if (ret == 0)
		ret = sprintf(buf, "%d\n", val);
	else
		return adrv9025_dev_err(phy);

	return ret;
}

static ssize_t adrv9025_phy_tx_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);
	u64 mask;
	bool enable;
	int ret = 0;

	if (chan->channel > CHAN_TX4)
		return -EINVAL;

	ret = strtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&phy->lock);

	switch (private) {
	case TX_QEC:
		mask = ADI_ADRV9025_TRACK_TX1_QEC << chan->channel;

		ret = adi_adrv9025_TrackingCalsEnableSet(
			phy->madDevice, mask,
			enable ? ADI_ADRV9025_TRACKING_CAL_ENABLE :
				 ADI_ADRV9025_TRACKING_CAL_DISABLE);
		if (ret)
			ret = adrv9025_dev_err(phy);

		break;
	case TX_LOL:
		mask = ADI_ADRV9025_TRACK_TX1_LOL << chan->channel;

		ret = adi_adrv9025_TrackingCalsEnableSet(
			phy->madDevice, mask,
			enable ? ADI_ADRV9025_TRACKING_CAL_ENABLE :
				 ADI_ADRV9025_TRACKING_CAL_DISABLE);
		if (ret)
			ret = adrv9025_dev_err(phy);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	return ret ? ret : len;
}

#define _ADRV9025_EXT_TX_INFO(_name, _ident)                                   \
	{                                                                      \
		.name = _name, .read = adrv9025_phy_tx_read,                   \
		.write = adrv9025_phy_tx_write, .private = _ident,             \
	}

static const struct iio_chan_spec_ext_info adrv9025_phy_rx_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	IIO_ENUM_AVAILABLE("gain_control_mode", IIO_SEPARATE,
				  &adrv9025_agc_modes_available),
	IIO_ENUM("gain_control_mode", false, &adrv9025_agc_modes_available),
	_ADRV9025_EXT_RX_INFO("rssi", RSSI),
	_ADRV9025_EXT_RX_INFO("quadrature_tracking_en", RX_QEC),
	_ADRV9025_EXT_RX_INFO("hd2_tracking_en",
			      RX_HD2), /* 2nd Harmonic Distortion */
	_ADRV9025_EXT_RX_INFO("bb_dc_offset_tracking_en", RX_DIG_DC),
	_ADRV9025_EXT_RX_INFO("rf_bandwidth", RX_RF_BANDWIDTH),
	{},
};

static const struct iio_chan_spec_ext_info adrv9025_phy_obs_rx_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADRV9025_EXT_RX_INFO("quadrature_tracking_en", RX_QEC),
	_ADRV9025_EXT_RX_INFO("rf_bandwidth", RX_RF_BANDWIDTH),
	_ADRV9025_EXT_RX_INFO("bb_dc_offset_tracking_en", RX_DIG_DC),
	{},
};

static struct iio_chan_spec_ext_info adrv9025_phy_tx_ext_info[] = {
	_ADRV9025_EXT_TX_INFO("quadrature_tracking_en", TX_QEC),
	_ADRV9025_EXT_TX_INFO("lo_leakage_tracking_en", TX_LOL),
	_ADRV9025_EXT_TX_INFO("rf_bandwidth", TX_RF_BANDWIDTH),
	{},
};
static int adrv9025_gainindex_to_gain(struct adrv9025_rf_phy *phy, int channel,
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

static int adrv9025_gain_to_gainindex(struct adrv9025_rf_phy *phy, int channel,
				      int val, int val2, unsigned int *index)
{
	int gain = ((abs(val) * 1000) + (abs(val2) / 1000));

	gain = clamp(gain, MIN_GAIN_mdB, MAX_RX_GAIN_mdB);
	*index = (gain - MAX_RX_GAIN_mdB) / RX_GAIN_STEP_mdB + 255;

	return 0;
}

static int adrv9025_phy_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, int *val,
				 int *val2, long m)
{
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);
	u32 rxchan = 0, txchan = 0;
	u16 temp;
	int ret;

	mutex_lock(&phy->lock);
	switch (m) {
	case IIO_CHAN_INFO_ENABLE:

		ret = adi_adrv9025_RxTxEnableGet(phy->madDevice, &rxchan,
						 &txchan);
		if (ret) {
			ret = adrv9025_dev_err(phy);
			break;
		}

		if (chan->output)
			*val = !!(txchan & (ADI_ADRV9025_TX1 << chan->channel));
		else
			*val = !!(rxchan & (ADI_ADRV9025_RX1 << chan->channel));

		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			adi_adrv9025_TxAtten_t txAtten;

			ret = adi_adrv9025_TxAttenGet(
				phy->madDevice, 1 << chan->channel, &txAtten);
			if (ret) {
				ret = adrv9025_dev_err(phy);
				break;
			}

			*val = -1 * (txAtten.txAttenuation_mdB / 1000);
			*val2 = (txAtten.txAttenuation_mdB % 1000) * 1000;
			if (!*val)
				*val2 *= -1;

		} else {
			adi_adrv9025_RxGain_t rxGain;

			ret = adi_adrv9025_RxGainGet(
				phy->madDevice, 1 << chan->channel, &rxGain);
			if (ret) {
				ret = adrv9025_dev_err(phy);
				break;
			}

			ret = adrv9025_gainindex_to_gain(phy, chan->channel,
							 rxGain.gainIndex, val,
							 val2);
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
		adi_adrv9025_TemperatureGet(phy->madDevice, &temp);
		*val = temp * 1000;
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&phy->lock);

	return ret;
};

static int adrv9025_phy_write_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan, int val,
				  int val2, long mask)
{
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);
	u32 rxchan = 0, txchan = 0;
	u32 code;
	int ret = 0;

	mutex_lock(&phy->lock);
	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:

		ret = adi_adrv9025_RxTxEnableGet(phy->madDevice, &rxchan,
						 &txchan);
		if (ret) {
			ret = adrv9025_dev_err(phy);
			goto out;
		}

		if (chan->output) {
			if (val)
				txchan |= (ADI_ADRV9025_TX1 << chan->channel);
			else
				txchan &= ~(ADI_ADRV9025_TX1 << chan->channel);
		} else {
			if (val)
				rxchan |= (ADI_ADRV9025_RX1 << chan->channel);
			else
				rxchan &= ~(ADI_ADRV9025_RX1 << chan->channel);
		}
		ret = adi_adrv9025_RxTxEnableSet(phy->madDevice, rxchan,
						 txchan);
		if (ret)
			ret = adrv9025_dev_err(phy);
		break;

	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			adi_adrv9025_TxAtten_t txAtten;

			if (val > 0 || (val == 0 && val2 > 0)) {
				ret = -EINVAL;
				goto out;
			}

			code = ((abs(val) * 1000) + (abs(val2) / 1000));

			txAtten.txChannelMask = 1 << chan->channel;
			txAtten.txAttenuation_mdB =
				code; /* Back off Tx output power by 30dB */
			ret = adi_adrv9025_TxAttenSet(phy->madDevice, &txAtten,
						      1);
			if (ret)
				adrv9025_dev_err(phy);

		} else {
			adi_adrv9025_RxGain_t rxGain;

			ret = adrv9025_gain_to_gainindex(phy, chan->channel,
							 val, val2, &code);
			if (ret < 0)
				break;

			rxGain.gainIndex = code;
			rxGain.rxChannelMask = 1 << chan->channel;

			ret = adi_adrv9025_RxGainSet(phy->madDevice, &rxGain,
						     1);
			if (ret)
				adrv9025_dev_err(phy);
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

static const struct iio_chan_spec adrv9025_phy_chan[] = {
	{
		/* LO1 */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.extend_name = "LO1",
		.ext_info = adrv9025_phy_ext_lo_info,
	},
	{
		/* LO2 */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.extend_name = "LO2",
		.ext_info = adrv9025_phy_ext_lo_info,
	},
	{
		/* LO2 */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 2,
		.extend_name = "AUX_LO",
		.ext_info = adrv9025_phy_ext_auxlo_info,
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
		.ext_info = adrv9025_phy_tx_ext_info,
	},
	{
		/* RX1 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv9025_phy_rx_ext_info,
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
		.ext_info = adrv9025_phy_tx_ext_info,
	},
	{
		/* RX2 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv9025_phy_rx_ext_info,
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
		.ext_info = adrv9025_phy_tx_ext_info,
	},
	{
		/* RX3 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX3,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv9025_phy_rx_ext_info,
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
		.ext_info = adrv9025_phy_tx_ext_info,
	},
	{
		/* RX4 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX4,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv9025_phy_rx_ext_info,
	},
	{
		/* RX Sniffer/Observation */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_OBS_RX1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv9025_phy_obs_rx_ext_info,
	},
	{
		/* RX Sniffer/Observation */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_OBS_RX2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = adrv9025_phy_obs_rx_ext_info,
	},
	{
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static const struct iio_info adrv9025_phy_info = {
	.read_raw = &adrv9025_phy_read_raw,
	.write_raw = &adrv9025_phy_write_raw,
	.debugfs_reg_access = &adrv9025_phy_reg_access,
	.attrs = &adrv9025_phy_attribute_group,
};

static ssize_t adrv9025_debugfs_read(struct file *file, char __user *userbuf,
				     size_t count, loff_t *ppos)
{
	struct adrv9025_debugfs_entry *entry = file->private_data;
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

	} else if (entry->cmd)
		val = entry->val;
	else
		return -EFAULT;

	if (!len)
		len = snprintf(buf, sizeof(buf), "%llu\n", val);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t adrv9025_debugfs_write(struct file *file,
				      const char __user *userbuf, size_t count,
				      loff_t *ppos)
{
	struct adrv9025_debugfs_entry *entry = file->private_data;
	struct adrv9025_rf_phy *phy = entry->phy;
	adi_adrv9025_FrmTestDataCfg_t frm_test_data;
	adi_adrv9025_TxTestToneCfg_t toneCfg;
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

		frm_test_data.injectPoint = ADI_ADRV9025_FTD_FRAMERINPUT;
		frm_test_data.testDataSource = val;
		frm_test_data.framerSelMask = ADI_ADRV9025_FRAMER_0;

		ret = adi_adrv9025_FramerTestDataSet(phy->madDevice,
						     &frm_test_data);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9025_dev_err(phy);

		entry->val = val;
		return count;
	case DBGFS_BIST_FRAMER_LOOPBACK:
		mutex_lock(&phy->lock);
		ret = adi_adrv9025_SpiFieldWrite(phy->madDevice, 0x6689,
						 val ? 0x7 : 0, 0xE0, 5);
		mutex_unlock(&phy->lock);
		if (ret)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_BIST_TONE:
		if (ret != 2)
			return -EINVAL;

		toneCfg.enable = val;
		toneCfg.txChannelMask = ADI_ADRV9025_TXALL;
		toneCfg.txToneFreq_Hz = val2;
		if (ret == 3)
			toneCfg.txToneGain = ADI_ADRV9025_TX_NCO_0_DB & val3;
		else
			toneCfg.txToneGain = ADI_ADRV9025_TX_NCO_0_DB;

		mutex_lock(&phy->lock);
		ret = adi_adrv9025_TxTestToneSet(phy->madDevice, &toneCfg, 1);
		mutex_unlock(&phy->lock);
		if (ret)
			return adrv9025_dev_err(phy);

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

static const struct file_operations adrv9025_debugfs_reg_fops = {
	.open = simple_open,
	.read = adrv9025_debugfs_read,
	.write = adrv9025_debugfs_write,
};

static void adrv9025_add_debugfs_entry(struct adrv9025_rf_phy *phy,
				       const char *propname, unsigned int cmd)
{
	unsigned int i = phy->adrv9025_debugfs_entry_index;

	if (WARN_ON(i >= ARRAY_SIZE(phy->debugfs_entry)))
		return;

	phy->debugfs_entry[i].phy = phy;
	phy->debugfs_entry[i].propname = propname;
	phy->debugfs_entry[i].cmd = cmd;

	phy->adrv9025_debugfs_entry_index++;
}

static int adrv9025_register_debugfs(struct iio_dev *indio_dev)
{
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);
	struct dentry *d;
	int i;

	if (!iio_get_debugfs_dentry(indio_dev))
		return -ENODEV;

	adrv9025_add_debugfs_entry(phy, "bist_framer_0_prbs",
				   DBGFS_BIST_FRAMER_0_PRBS);
	adrv9025_add_debugfs_entry(phy, "bist_framer_loopback",
				   DBGFS_BIST_FRAMER_LOOPBACK);
	adrv9025_add_debugfs_entry(phy, "bist_tone", DBGFS_BIST_TONE);

	for (i = 0; i < phy->adrv9025_debugfs_entry_index; i++)
		d = debugfs_create_file(phy->debugfs_entry[i].propname, 0644,
					iio_get_debugfs_dentry(indio_dev),
					&phy->debugfs_entry[i],
					&adrv9025_debugfs_reg_fops);
	return 0;
}

#define ADRV9025_MAX_CLK_NAME 79

static char *adrv9025_clk_set_dev_name(struct adrv9025_rf_phy *phy, char *dest,
				       const char *name)
{
	size_t len = 0;

	if (name == NULL)
		return NULL;

	if (*name == '-')
		len = strscpy(dest, dev_name(&phy->spi->dev),
			      ADRV9025_MAX_CLK_NAME);
	else
		*dest = '\0';

	return strncat(dest, name, ADRV9025_MAX_CLK_NAME - len);
}

static unsigned long adrv9025_bb_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct adrv9025_clock *clk_priv = to_clk_priv(hw);

	return clk_priv->rate;
}

static int adrv9025_bb_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct adrv9025_clock *clk_priv = to_clk_priv(hw);

	clk_priv->rate = rate;

	return 0;
}

static long adrv9025_bb_round_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long *prate)
{
	struct adrv9025_clock *clk_priv = to_clk_priv(hw);

	dev_dbg(&clk_priv->spi->dev, "%s: Rate %lu Hz", __func__, rate);

	return rate;
}

static const struct clk_ops bb_clk_ops = {
	.round_rate = adrv9025_bb_round_rate,
	.set_rate = adrv9025_bb_set_rate,
	.recalc_rate = adrv9025_bb_recalc_rate,
};

static int adrv9025_clk_register(struct adrv9025_rf_phy *phy, const char *name,
				 const char *parent_name,
				 const char *parent_name2, unsigned long flags,
				 u32 source)
{
	struct adrv9025_clock *clk_priv = &phy->clk_priv[source];
	struct clk_init_data init;
	struct clk *clk;
	char c_name[ADRV9025_MAX_CLK_NAME + 1],
		p_name[2][ADRV9025_MAX_CLK_NAME + 1];
	const char *_parent_name[2];
	u32 rate;

	/* struct adrv9025_clock assignments */
	clk_priv->source = source;
	clk_priv->hw.init = &init;
	clk_priv->spi = phy->spi;
	clk_priv->phy = phy;

	_parent_name[0] =
		adrv9025_clk_set_dev_name(phy, p_name[0], parent_name);
	_parent_name[1] =
		adrv9025_clk_set_dev_name(phy, p_name[1], parent_name2);

	init.name = adrv9025_clk_set_dev_name(phy, c_name, name);
	init.flags = flags;
	init.parent_names = &_parent_name[0];
	init.num_parents = _parent_name[1] ? 2 : _parent_name[0] ? 1 : 0;


	switch (source) {
	case RX_SAMPL_CLK:
		init.ops = &bb_clk_ops;
		clk_priv->rate = phy->rx_iqRate_kHz;
		break;
	case TX_SAMPL_CLK:
			adrv9025_TxLinkSamplingRateFind(phy->madDevice, &phy->deviceInitStruct,
							ADI_ADRV9025_DEFRAMER_0,
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

static irqreturn_t adrv9025_irq_handler(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct adrv9025_rf_phy *phy = iio_priv(indio_dev);
	adi_adrv9025_gpIntStatus_t gpInt0Status;
	int ret;

	ret = adi_adrv9025_GpInt0Handler(phy->madDevice, &gpInt0Status);
	if (ret)
		return adrv9025_dev_err(phy);

	dev_warn(&phy->spi->dev, "GP0 Interrupt Status 0x%llX: %s\n",
		 gpInt0Status.gp_Interrupt_Status,
		 gpInt0Status.gp_Int_Error_Message);

	return IRQ_HANDLED;
}

static void adrv9025_info(struct adrv9025_rf_phy *phy)
{
	struct spi_device *spi = phy->spi;
	adi_adrv9025_ApiVersion_t apiVersion;
	adi_adrv9025_ArmVersion_t armVersion;
	adi_adrv9025_StreamVersion_t streamVersion;

	adi_adrv9025_ApiVersionGet(phy->madDevice, &apiVersion);
	adi_adrv9025_ArmVersionGet(phy->madDevice, &armVersion);
	adi_adrv9025_StreamVersionGet(phy->madDevice, &streamVersion);

	dev_info(&spi->dev,
		 "%s Rev %d, Firmware %u.%u.%u.%u API version: %u.%u.%u.%u Stream version: %u.%u.%u.%u successfully initialized%s",
		 spi_get_device_id(spi)->name,
		 phy->madDevice->devStateInfo.deviceSiRev, armVersion.majorVer,
		 armVersion.minorVer, armVersion.maintVer, armVersion.rcVer,
		 apiVersion.majorVer, apiVersion.minorVer,
		 apiVersion.maintenanceVer, apiVersion.buildVer,
		 streamVersion.majorVer, streamVersion.minorVer,
		 streamVersion.mainVer, streamVersion.buildVer,
		 phy->jdev ? " via jesd204-fsm" : "");
}

struct adrv9025_jesd204_link {
	unsigned int source_id;
	bool is_framer;
};

struct adrv9025_jesd204_priv {
	struct adrv9025_rf_phy *phy;
	struct adrv9025_jesd204_link link[5];
};

int adrv9025_jesd204_link_pre_setup(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv9025_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct adrv9025_rf_phy *phy = priv->phy;
	long dev_clk;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	dev_clk = clk_round_rate(phy->dev_clk,
				 phy->deviceInitStruct.clocks.deviceClock_kHz * 1000);

	if (dev_clk > 0 && ((dev_clk / 1000) ==
		phy->deviceInitStruct.clocks.deviceClock_kHz)) {
		clk_set_rate(phy->dev_clk, (unsigned long) dev_clk);
	} else {
		dev_err(&phy->spi->dev,
			"Requesting device clock %u failed got %ld",
			phy->deviceInitStruct.clocks.deviceClock_kHz * 1000, dev_clk);
		return -EINVAL;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv9025_jesd204_link_init(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv9025_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct adrv9025_rf_phy *phy = priv->phy;
	adi_adrv9025_FrmCfg_t *framer = NULL;
	adi_adrv9025_DfrmCfg_t *deframer = NULL;
	u32 rate;
	int ret;

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
		deframer = &phy->deviceInitStruct.dataInterface.deframer[0];
		priv->link[lnk->link_id].source_id = ADI_ADRV9025_DEFRAMER_0;
		ret = adrv9025_TxLinkSamplingRateFind(phy->madDevice, &phy->deviceInitStruct,
							ADI_ADRV9025_DEFRAMER_0,
							&rate);
		phy->tx_iqRate_kHz  = rate;
		break;
	case DEFRAMER1_LINK_TX:
		deframer = &phy->deviceInitStruct.dataInterface.deframer[1];
		priv->link[lnk->link_id].source_id = ADI_ADRV9025_DEFRAMER_1;
		ret = adrv9025_TxLinkSamplingRateFind(phy->madDevice, &phy->deviceInitStruct,
							ADI_ADRV9025_DEFRAMER_1,
							&rate);
		break;
	case FRAMER0_LINK_RX:
		framer = &phy->deviceInitStruct.dataInterface.framer[0];
		priv->link[lnk->link_id].source_id = ADI_ADRV9025_FRAMER_0;
		priv->link[lnk->link_id].is_framer = true;
		ret = adrv9025_RxLinkSamplingRateFind(phy->madDevice, &phy->deviceInitStruct,
							ADI_ADRV9025_FRAMER_0,
							&rate);
		phy->rx_iqRate_kHz = rate;
		break;
	case FRAMER1_LINK_RX:
		framer = &phy->deviceInitStruct.dataInterface.framer[1];
		priv->link[lnk->link_id].source_id = ADI_ADRV9025_FRAMER_1;
		priv->link[lnk->link_id].is_framer = true;
		ret = adrv9025_RxLinkSamplingRateFind(phy->madDevice, &phy->deviceInitStruct,
							ADI_ADRV9025_FRAMER_1,
							&rate);
		break;
	case FRAMER2_LINK_RX:
		framer = &phy->deviceInitStruct.dataInterface.framer[2];
		priv->link[lnk->link_id].source_id = ADI_ADRV9025_FRAMER_2;
		priv->link[lnk->link_id].is_framer = true;
		ret = adrv9025_RxLinkSamplingRateFind(phy->madDevice, &phy->deviceInitStruct,
							ADI_ADRV9025_FRAMER_2,
							&rate);
		break;
	default:
		return -EINVAL;
	}

	if (ret)
		return adrv9025_dev_err(phy);

	lnk->sample_rate = rate * 1000;

	if (framer) {
		lnk->num_converters = framer->jesd204M;
		lnk->num_lanes = hweight8(framer->serializerLanesEnabled);
		lnk->octets_per_frame = framer->jesd204F;
		lnk->frames_per_multiframe = framer->jesd204K;
		lnk->device_id = framer->deviceId;
		lnk->bank_id = framer->bankId;
		lnk->scrambling = framer->scramble;
		lnk->bits_per_sample = framer->jesd204Np;
		lnk->converter_resolution = framer->jesd204Np;
		lnk->ctrl_bits_per_sample = 0;
		lnk->jesd_version = framer->enableJesd204C ? JESD204_VERSION_C : JESD204_VERSION_B;
		lnk->subclass = JESD204_SUBCLASS_1;
		lnk->is_transmit = false;
	} else if (deframer) {
		lnk->num_converters = deframer->jesd204M;
		lnk->num_lanes = hweight8(deframer->deserializerLanesEnabled);
		lnk->octets_per_frame = deframer->jesd204F;
		lnk->frames_per_multiframe = deframer->jesd204K;
		lnk->device_id = deframer->deviceId;
		lnk->bank_id = deframer->bankId;
		lnk->scrambling = deframer->scramble;
		lnk->bits_per_sample = deframer->jesd204Np;
		lnk->converter_resolution = deframer->jesd204Np;
		lnk->ctrl_bits_per_sample = 0;
		lnk->jesd_version = deframer->enableJesd204C ? JESD204_VERSION_C : JESD204_VERSION_B;
		lnk->subclass = JESD204_SUBCLASS_1;
		lnk->is_transmit = true;
	};

	return JESD204_STATE_CHANGE_DONE;
}

int adrv9025_jesd204_link_setup(struct jesd204_dev *jdev,
				enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv9025_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct adrv9025_rf_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));


	if (reason == JESD204_STATE_OP_REASON_UNINIT) {
		phy->is_initialized = 0;

		adi_adrv9025_Shutdown(phy->madDevice);
		adi_adrv9025_HwClose(phy->madDevice);

		memset(&phy->adi_adrv9025_device.devStateInfo, 0,
			sizeof(phy->adi_adrv9025_device.devStateInfo));

		return JESD204_STATE_CHANGE_DONE;
	}

	memset(&phy->adi_adrv9025_device.devStateInfo, 0,
		sizeof(phy->adi_adrv9025_device.devStateInfo));

	ret = adi_adrv9025_HwOpen(phy->madDevice, &phy->spiSettings);
	if (ret)
		return adrv9025_dev_err(phy);

	adi_common_LogLevelSet(&phy->madDevice->common,
			       ADI_HAL_LOG_ERR | ADI_HAL_LOG_WARN);

	/* Pre MCS - Broadcastable */
	ret = adi_adrv9025_PreMcsInit_v2(phy->madDevice, &phy->deviceInitStruct,
					 phy->platformFiles.armImageFile,
					 phy->platformFiles.streamImageFile,
					 phy->platformFiles.rxGainTableFileArr,
					 phy->platformFiles.rxGainTableFileArrSize,
					 phy->platformFiles.txAttenTableFileArr,
					 phy->platformFiles.txAttenTableFileArrSize);
	if (ret)
		return adrv9025_dev_err(phy);

	/* Pre MCS - Non-Broadcastable */
	ret = adi_adrv9025_PreMcsInit_NonBroadCast(phy->madDevice,
						   &phy->deviceInitStruct);
	if (ret)
		return adrv9025_dev_err(phy);

	/* MCS start sequence*/
	ret = adi_adrv9025_MultichipSyncSet(phy->madDevice, ADI_ENABLE);
	if (ret)
		return adrv9025_dev_err(phy);

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv9025_jesd204_setup_stage1(struct jesd204_dev *jdev,
					 enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv9025_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct adrv9025_rf_phy *phy = priv->phy;
	int ret, i;
	u32 mcsStatus;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	/* This loop will send SysRef pulses up to 255 times unless MCS status achieved before. */
	for (i = 0; i < 255; i++) {
		ret = adi_adrv9025_MultichipSyncStatusGet(phy->madDevice,
							  &mcsStatus);
		if (ret)
			return adrv9025_dev_err(phy);

		if ((mcsStatus & 0x17) == 0x17)
			break;

		jesd204_sysref_async_force(phy->jdev);
	}

	if (mcsStatus != 0x17) {
		dev_err(&phy->spi->dev,
			"%s:%d Unexpected MCS sync status (0x%X)",
			__func__, __LINE__, mcsStatus);

		return adrv9025_dev_err(phy);
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv9025_jesd204_setup_stage2(struct jesd204_dev *jdev,
					 enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv9025_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct adrv9025_rf_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	/* MCS end sequence*/
	ret = adi_adrv9025_MultichipSyncSet(phy->madDevice, ADI_DISABLE);
	if (ret)
		return adrv9025_dev_err(phy);

	/* Post MCS */
	ret = adi_adrv9025_PostMcsInit(phy->madDevice,
				       &phy->adrv9025PostMcsInitInst);
	if (ret)
		return adrv9025_dev_err(phy);

	ret = adi_adrv9025_SerializerReset(
		phy->madDevice, phy->deviceInitStruct.clocks.serdesPllVcoFreq_kHz);
	if (ret)
		return adrv9025_dev_err(phy);

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv9025_jesd204_clks_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv9025_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct adrv9025_rf_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	if (!lnk->num_converters)
		return JESD204_STATE_CHANGE_DONE;

	if (priv->link[lnk->link_id].is_framer) {

		if (phy->madDevice->devStateInfo.linkSharingEnabled == 1) {
			ret = adi_adrv9025_FramerSysrefCtrlSet(phy->madDevice,
				ADI_ADRV9025_FRAMER_1, 0);
			if (ret)
				return adrv9025_dev_err(phy);

			ret = adi_adrv9025_FramerLinkStateSet(phy->madDevice,
				ADI_ADRV9025_FRAMER_1, 0);
			if (ret)
				return adrv9025_dev_err(phy);

			ret = adi_adrv9025_FramerLinkStateSet(phy->madDevice,
				ADI_ADRV9025_FRAMER_1, 1);
			if (ret)
				return adrv9025_dev_err(phy);

			dev_dbg(&phy->spi->dev,
				"%s:%d Link %d Framer enabled", __func__, __LINE__,
				ADI_ADRV9025_FRAMER_1);

			/*************************************************/
			/**** Enable SYSREF to Talise JESD204B Framer ***/
			/*************************************************/
			/*** < User: Make sure SYSREF is stopped/disabled > ***/
			ret = adi_adrv9025_FramerSysrefCtrlSet(phy->madDevice,
				ADI_ADRV9025_FRAMER_1, 1);
			if (ret)
				return adrv9025_dev_err(phy);

			jesd204_sysref_async_force(phy->jdev);

			ret = adi_adrv9025_FramerLinkStateSet(phy->madDevice,
				ADI_ADRV9025_FRAMER_1, 0);
			if (ret)
				return adrv9025_dev_err(phy);

			ret = adi_adrv9025_FramerSysrefCtrlSet(phy->madDevice,
				ADI_ADRV9025_FRAMER_1, 0);
			if (ret)
				return adrv9025_dev_err(phy);

		}

		ret = adi_adrv9025_FramerSysrefCtrlSet(phy->madDevice,
			priv->link[lnk->link_id].source_id, 0);
		if (ret)
			return adrv9025_dev_err(phy);

		ret = adi_adrv9025_FramerLinkStateSet(phy->madDevice,
			priv->link[lnk->link_id].source_id, 0);
		if (ret)
			return adrv9025_dev_err(phy);


		ret = adi_adrv9025_FramerLinkStateSet(phy->madDevice,
			priv->link[lnk->link_id].source_id, 1);
		if (ret)
			return adrv9025_dev_err(phy);


		dev_dbg(&phy->spi->dev,
			"%s:%d Link %d Framer enabled", __func__, __LINE__,
			priv->link[lnk->link_id].source_id);

		/*************************************************/
		/**** Enable SYSREF to Talise JESD204B Framer ***/
		/*************************************************/
		/*** < User: Make sure SYSREF is stopped/disabled > ***/
		ret = adi_adrv9025_FramerSysrefCtrlSet(phy->madDevice,
			priv->link[lnk->link_id].source_id, 1);
		if (ret)
			return adrv9025_dev_err(phy);

	} else {
		ret = adi_adrv9025_DeframerSysrefCtrlSet(phy->madDevice,
			priv->link[lnk->link_id].source_id, 0);
		if (ret)
			return adrv9025_dev_err(phy);


		ret = adi_adrv9025_DfrmLinkStateSet(phy->madDevice,
			priv->link[lnk->link_id].source_id, 0);
		if (ret)
			return adrv9025_dev_err(phy);

	};

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv9025_jesd204_link_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv9025_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct adrv9025_rf_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	if (!lnk->num_converters)
		return JESD204_STATE_CHANGE_DONE;

	if (!priv->link[lnk->link_id].is_framer) { /* DEFRAMER */
		u8 errFlags = 0;
		adi_adrv9025_InitCals_t serdesCal = {
			.calMask = ADI_ADRV9025_SERDES_INIT,
			.channelMask = 0xF, /* CAL_ALL_CHANNELS */
			.warmBoot = 0,
		};

		ret = adi_adrv9025_DfrmLinkStateSet(phy->madDevice,
			priv->link[lnk->link_id].source_id, 1);
		if (ret)
			return adrv9025_dev_err(phy);

		/* Notify ARM to run SERDES Calbriation if necessary */
		ret = adi_adrv9025_InitCalsRun(phy->madDevice, &serdesCal);
		if (ret)
			return adrv9025_dev_err(phy);

		/* Wait up to 60 seconds for ARM */
		ret = adi_adrv9025_InitCalsWait(phy->madDevice, 60000, &errFlags);
		if (ret) {
			dev_err(&phy->spi->dev, "Error: InitCalsWait 0x%X\n", errFlags);
			return adrv9025_dev_err(phy);
		}

		/***************************************************/
		/**** Enable SYSREF to Talise JESD204B Deframer ***/
		/***************************************************/
		ret = adi_adrv9025_DeframerSysrefCtrlSet(phy->madDevice,
			priv->link[lnk->link_id].source_id, 1);
		if (ret)
			return adrv9025_dev_err(phy);

	};

	return JESD204_STATE_CHANGE_DONE;
}

static int adrv9025_jesd204_link_running(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv9025_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct adrv9025_rf_phy *phy = priv->phy;
	int ret;

	adi_adrv9025_FramerStatus_t framerStatus;
	adi_adrv9025_DeframerStatus_t deframerStatus;
	u8 deframerLinkCondition = 0;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	if (!lnk->num_converters)
		return JESD204_STATE_CHANGE_DONE;

	if (priv->link[lnk->link_id].is_framer) {
		ret = adi_adrv9025_FramerStatusGet(phy->madDevice,
			priv->link[lnk->link_id].source_id, &framerStatus);
		if (ret)
			return adrv9025_dev_err(phy);


		if ((framerStatus.status & 0x0F) != 0x0A)
			dev_warn(&phy->spi->dev,
				"Link%u framerStatus 0x%X",
				lnk->link_id, framerStatus.status);
	} else {
		ret = adi_adrv9025_DeframerStatusGet(phy->madDevice,
			priv->link[lnk->link_id].source_id, &deframerStatus);
		if (ret)
			return adrv9025_dev_err(phy);

		ret  = adi_adrv9025_DfrmLinkConditionGet(
			phy->madDevice,
			priv->link[lnk->link_id].source_id,
			&deframerLinkCondition);

		if ((deframerStatus.status & 0x7F) != 0x7) /* Ignore Valid ILAS checksum */
			dev_warn(&phy->spi->dev,
				"Link%u deframerStatus 0x%X",
				lnk->link_id, deframerStatus.status);

		/* Kick off SERDES tracking cal if lanes are up */
		ret = adi_adrv9025_TrackingCalsEnableSet(
			phy->madDevice, ADI_ADRV9025_TRACK_DESERIALIZER,
			ADI_ADRV9025_TRACKING_CAL_ENABLE);
		if (ret)
			return adrv9025_dev_err(phy);
	};


	return JESD204_STATE_CHANGE_DONE;
}


static int adrv9025_jesd204_post_running_stage(struct jesd204_dev *jdev,
	enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct adrv9025_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct adrv9025_rf_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__,
		jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		phy->is_initialized = 0;
		return JESD204_STATE_CHANGE_DONE;
	}

	/* Initialize Tx Ramp down functionality */
	ret = adi_adrv9025_TxRampDownInit(phy->madDevice, &phy->deviceInitStruct);
	if (ret)
		return adrv9025_dev_err(phy);

	/* Setup GP Interrupts from init structure */
	ret = adi_adrv9025_GpIntInit(phy->madDevice,
				     &phy->deviceInitStruct.gpInterrupts);
	if (ret)
		return adrv9025_dev_err(phy);

	clk_set_rate(phy->clks[RX_SAMPL_CLK], phy->rx_iqRate_kHz * 1000);
	clk_set_rate(phy->clks[TX_SAMPL_CLK], phy->tx_iqRate_kHz * 1000);

	ret = adi_adrv9025_AgcCfgSet(phy->madDevice, phy->agcConfig, 1);
	if (ret)
		return adrv9025_dev_err(phy);

	ret = adi_adrv9025_RxTxEnableSet(phy->madDevice, 0xF, ADI_ADRV9025_TXALL);
	if (ret)
		return adrv9025_dev_err(phy);

	phy->is_initialized = 1;
	//enable_irq(phy->spi->irq);
	adrv9025_info(phy);

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_adrv9025_init = {
	.state_ops = {
		[JESD204_OP_LINK_INIT] = {
			.per_link = adrv9025_jesd204_link_init,
		},
		[JESD204_OP_LINK_PRE_SETUP] = {
			.per_device = adrv9025_jesd204_link_pre_setup,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_OPT_SETUP_STAGE1] = {
			.per_device = adrv9025_jesd204_setup_stage1,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
			.post_state_sysref = true,
		},
		[JESD204_OP_OPT_SETUP_STAGE2] = {
			.per_device = adrv9025_jesd204_setup_stage2,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
			.post_state_sysref = true,
		},
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = adrv9025_jesd204_clks_enable,
		},
		[JESD204_OP_LINK_SETUP] = {
			.per_device = adrv9025_jesd204_link_setup,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
			.post_state_sysref = true,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = adrv9025_jesd204_link_enable,
			.post_state_sysref = true,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = adrv9025_jesd204_link_running,
		},
		[JESD204_OP_OPT_POST_RUNNING_STAGE] = {
			.per_device = adrv9025_jesd204_post_running_stage,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
	},

	.max_num_links = 5,
	.sizeof_priv = sizeof(struct adrv9025_jesd204_priv),
};

static int __adrv9025_of_get_u32(struct device *dev, struct device_node *np, const char *propname,
				 u32 defval, void *out_value, u32 size, u32 min, u32 max)
{
	u32 tmp;
	int ret;

	ret = of_property_read_u32(np, propname, &tmp);
	if (ret) {
		tmp = defval;
		ret = 0;
	} else if (tmp < min || tmp > max) {
		dev_err(dev, "%s dt property out of range (actual value %d, min %d, max %d)\n", propname, tmp, min, max);
		return -EINVAL;
	}

	if (out_value) {
		switch (size) {
		case 1:
			*(u8 *)out_value = tmp;
			break;
		case 2:
			*(u16 *)out_value = tmp;
			break;
		case 4:
			*(u32 *)out_value = tmp;
			break;
		}
	}

	return 0;
}

#define ADRV9025_OF_PROP(_dt_name, _member_, _default, min, max) \
	__adrv9025_of_get_u32(dev, np, _dt_name, _default, _member_, sizeof(*_member_), min, max)

static int adrv9025_phy_parse_agc_dt(struct iio_dev *iodev, struct device *dev)
{
	struct adrv9025_rf_phy *phy = iio_priv(iodev);
	struct device_node *np = dev->of_node;
	int ret;

	ret = ADRV9025_OF_PROP("adi,rxagc-peak-agc-under-range-low-interval",
			       &phy->agcConfig->agcPeak.agcUnderRangeLowInterval, 1229, 0, 65535);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-agc-under-range-mid-interval",
			       &phy->agcConfig->agcPeak.agcUnderRangeMidInterval, 4, 0, 63);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-agc-under-range-high-interval",
			       &phy->agcConfig->agcPeak.agcUnderRangeHighInterval, 4, 0, 63);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-apd-high-thresh",
			       &phy->agcConfig->agcPeak.apdHighThresh, 42, 0, 63);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-apd-low-gain-mode-high-thresh",
			       &phy->agcConfig->agcPeak.apdLowGainModeHighThresh, 0, 0, 63);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-apd-low-thresh",
			       &phy->agcConfig->agcPeak.apdLowThresh, 30, 0, 63);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-apd-low-gain-mode-low-thresh",
			       &phy->agcConfig->agcPeak.apdLowGainModeLowThresh, 0, 0, 63);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-apd-upper-thresh-peak-exceeded-cnt",
			       &phy->agcConfig->agcPeak.apdUpperThreshPeakExceededCnt, 3, 0, 255);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-apd-lower-thresh-peak-exceeded-cnt",
			       &phy->agcConfig->agcPeak.apdLowerThreshPeakExceededCnt, 3, 0, 255);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-apd-gain-step-attack",
			       &phy->agcConfig->agcPeak.apdGainStepAttack, 4, 0, 31);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-apd-gain-step-recovery",
			       &phy->agcConfig->agcPeak.apdGainStepRecovery, 0, 0, 31);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-enable-hb2-overload",
			       &phy->agcConfig->agcPeak.enableHb2Overload, 1, 0, 1);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-overload-duration-cnt",
			       &phy->agcConfig->agcPeak.hb2OverloadDurationCnt, 2, 0, 6);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-overload-thresh-cnt",
			       &phy->agcConfig->agcPeak.hb2OverloadThreshCnt, 1, 1, 15);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-high-thresh",
			       &phy->agcConfig->agcPeak.hb2HighThresh, 10388, 0, 16383);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-under-range-low-thresh",
			       &phy->agcConfig->agcPeak.hb2UnderRangeLowThresh, 64, 0, 16383);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-under-range-mid-thresh",
			       &phy->agcConfig->agcPeak.hb2UnderRangeMidThresh, 102, 0, 16383);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-under-range-high-thresh",
			       &phy->agcConfig->agcPeak.hb2UnderRangeHighThresh, 128, 0, 16383);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-upper-thresh-peak-exceeded-cnt",
			       &phy->agcConfig->agcPeak.hb2UpperThreshPeakExceededCnt, 3, 0, 255);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-under-range-high-thresh-exceeded-cnt",
			       &phy->agcConfig->agcPeak.hb2UnderRangeHighThreshExceededCnt, 3, 0, 255);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-gain-step-high-recovery",
			       &phy->agcConfig->agcPeak.hb2GainStepHighRecovery, 2, 0, 31);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-gain-step-low-recovery",
			       &phy->agcConfig->agcPeak.hb2GainStepLowRecovery, 8, 0, 31);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-gain-step-mid-recovery",
			       &phy->agcConfig->agcPeak.hb2GainStepMidRecovery, 4, 0, 31);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-gain-step-attack",
			       &phy->agcConfig->agcPeak.hb2GainStepAttack, 4, 0, 31);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-overload-power-mode",
			       &phy->agcConfig->agcPeak.hb2OverloadPowerMode, 1, 0, 1);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-thresh-config",
			       &phy->agcConfig->agcPeak.hb2ThreshConfig, 3, 3, 3);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-under-range-mid-thresh-exceeded-cnt",
			       &phy->agcConfig->agcPeak.hb2UnderRangeMidThreshExceededCnt, 3, 0, 255);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-peak-hb2-under-range-low-thresh-exceeded-cnt",
			       &phy->agcConfig->agcPeak.hb2UnderRangeLowThreshExceededCnt, 3, 0, 255);
	if (ret)
		return ret;

	ret = ADRV9025_OF_PROP("adi,rxagc-power-power-enable-measurement",
			       &phy->agcConfig->agcPower.powerEnableMeasurement, 1, 0, 1);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-power-power-input-select",
			       &phy->agcConfig->agcPower.powerInputSelect, 2, 0, 3);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-power-under-range-high-power-thresh",
			       &phy->agcConfig->agcPower.underRangeHighPowerThresh, 13, 0, 127);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-power-under-range-low-power-thresh",
			       &phy->agcConfig->agcPower.underRangeLowPowerThresh, 4, 0, 31);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-power-under-range-high-power-gain-step-recovery",
			       &phy->agcConfig->agcPower.underRangeHighPowerGainStepRecovery, 4, 0, 31);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-power-under-range-low-power-gain-step-recovery",
			       &phy->agcConfig->agcPower.underRangeLowPowerGainStepRecovery, 4, 0, 31);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-power-power-measurement-duration",
			       &phy->agcConfig->agcPower.powerMeasurementDuration, 5, 0, 31);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-power-rx-tdd-power-meas-duration",
			       &phy->agcConfig->agcPower.rxTddPowerMeasDuration, 31661, 0, 65535);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-power-rx-tdd-power-meas-delay",
			       &phy->agcConfig->agcPower.rxTddPowerMeasDelay, 54098, 0, 65535);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-power-over-range-high-power-thresh",
			       &phy->agcConfig->agcPower.overRangeHighPowerThresh, 10, 0, 127);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-power-over-range-low-power-thresh",
			       &phy->agcConfig->agcPower.overRangeLowPowerThresh, 2, 0, 15);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-power-power-log-shift",
			       &phy->agcConfig->agcPower.powerLogShift, 1, 0, 1);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-power-over-range-high-power-gain-step-attack",
			       &phy->agcConfig->agcPower.overRangeHighPowerGainStepAttack, 4, 0, 31);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-power-over-range-low-power-gain-step-attack",
			       &phy->agcConfig->agcPower.overRangeLowPowerGainStepAttack, 4, 0, 31);
	if (ret)
		return ret;

	ret = ADRV9025_OF_PROP("adi,rxagc-rx-channel-mask",
			       &phy->agcConfig->rxChannelMask, 0x0F, 0, 15);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-agc-peak-wait-time",
			       &phy->agcConfig->agcPeakWaitTime, 4, 0, 31);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-agc-rx-max-gain-index",
			       &phy->agcConfig->agcRxMaxGainIndex, 255, 0, 255);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-agc-rx-min-gain-index",
			       &phy->agcConfig->agcRxMinGainIndex, 183, 0, 255);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-agc-gain-update-counter",
			       &phy->agcConfig->agcGainUpdateCounter, 245760, 0, 4194303);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-agc-rx-attack-delay",
			       &phy->agcConfig->agcRxAttackDelay, 10, 0, 63);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-agc-slow-loop-settling-delay",
			       &phy->agcConfig->agcSlowLoopSettlingDelay, 16, 0, 127);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-agc-low-thresh-prevent-gain-inc",
			       &phy->agcConfig->agcLowThreshPreventGainInc, 1, 0, 1);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-agc-change-gain-if-thresh-high",
			       &phy->agcConfig->agcChangeGainIfThreshHigh, 3, 0, 3);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-agc-peak-thresh-gain-control-mode",
			       &phy->agcConfig->agcPeakThreshGainControlMode, 1, 0, 1);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-agc-reset-on-rxon",
			       &phy->agcConfig->agcResetOnRxon, 0, 0, 1);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-agc-enable-sync-pulse-for-gain-counter",
			       &phy->agcConfig->agcEnableSyncPulseForGainCounter, 0, 0, 1);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-agc-enable-fast-recovery-loop",
			       &phy->agcConfig->agcEnableFastRecoveryLoop, 0, 0, 1);
	if (ret)
		return ret;
	ret = ADRV9025_OF_PROP("adi,rxagc-agc-adc-reset-gain-step",
			       &phy->agcConfig->agcAdcResetGainStep, 0, 0, 31);
	if (ret)
		return ret;
	return ADRV9025_OF_PROP("adi,rxagc-agc-slow-loop-fast-gain-change-block-enable",
				&phy->agcConfig->agcSlowloopFastGainChangeBlockEnable, 0, 0, 1);
}

static int adrv9025_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adrv9025_rf_phy *phy;
	struct jesd204_dev *jdev;
	struct adrv9025_jesd204_priv *priv;
	struct device_node *np = spi->dev.of_node;
	adi_adrv9025_ApiVersion_t apiVersion;
	struct clk *clk = NULL;

	int ret, i;
	const char *name;
	u32 val;

	int id = spi_get_device_id(spi)->driver_data;

	jdev = devm_jesd204_dev_register(&spi->dev, &jesd204_adrv9025_init);
	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

	clk = devm_clk_get(&spi->dev, "dev_clk");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*phy));
	if (indio_dev == NULL)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->indio_dev = indio_dev;
	phy->spi = spi;
	phy->spi_device_id = id;
	phy->dev_clk = clk;
	phy->jdev = jdev;
	phy->agcConfig = kzalloc(sizeof(adi_adrv9025_AgcCfg_t), GFP_KERNEL);
	if (!(phy->agcConfig))
		return -ENOMEM;
	mutex_init(&phy->lock);

	ret = adrv9025_phy_parse_agc_dt(indio_dev, &spi->dev);
	if (ret)
		return ret;

	priv = jesd204_dev_priv(jdev);
	priv->phy = phy;

	phy->madDevice = &phy->adi_adrv9025_device;
	phy->linux_hal.spi = spi;
	phy->linux_hal.logCfg.logLevel = ADI_HAL_LOG_ERR | ADI_HAL_LOG_WARN;
	phy->madDevice->common.devHalInfo = &phy->linux_hal;

	if (!of_property_read_string(np, "adi,arm-firmware-name", &name)) {
		strncpy(phy->platformFiles.armImageFile, name, sizeof(phy->platformFiles.armImageFile));
	} else {
		dev_err(&spi->dev, "error missing dt property: adi,arm-firmware-name\n");
		return -EINVAL;
	}

	if (!of_property_read_string(np, "adi,stream-firmware-name", &name)) {
		strncpy(phy->platformFiles.streamImageFile, name, sizeof(phy->platformFiles.streamImageFile));
	} else {
		dev_err(&spi->dev, "error missing dt property: adi,stream-firmware-name\n");
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(phy->platformFiles.rxGainTableFileArr); i++) {
		ret = of_property_read_string_index(np, "adi,rx-gaintable-names", i, &name);

		if (!ret && !of_property_read_u32_index(np, "adi,rx-gaintable-channel-masks", i, &val)) {
			strncpy(phy->platformFiles.rxGainTableFileArr[i].rxGainTableCsvFileName, name,
				sizeof(phy->platformFiles.rxGainTableFileArr[0].rxGainTableCsvFileName));
			phy->platformFiles.rxGainTableFileArr[i].rxChannelMask = val;
			phy->platformFiles.rxGainTableFileArrSize++;
		}
	}

	for (i = 0; i < ARRAY_SIZE(phy->platformFiles.txAttenTableFileArr); i++) {
		ret = of_property_read_string_index(np, "adi,tx-attntable-names", i, &name);

		if (!ret && !of_property_read_u32_index(np, "adi,tx-attntable-channel-masks", i, &val)) {
			strncpy(phy->platformFiles.txAttenTableFileArr[i].txAttenTableCsvFileName, name,
				sizeof(phy->platformFiles.txAttenTableFileArr[0].txAttenTableCsvFileName));
			phy->platformFiles.txAttenTableFileArr[i].txChannelMask = val;
			phy->platformFiles.txAttenTableFileArrSize++;
		}
	}

	adi_hal_PlatformSetup(phy->madDevice->common.devHalInfo, 0);

	phy->linux_hal.reset_gpio =
		devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_LOW);

	ret = clk_prepare_enable(phy->dev_clk);
	if (ret)
		return ret;

	phy->spiSettings.msbFirst = 1;
	phy->spiSettings.enSpiStreaming = 0;
	phy->spiSettings.autoIncAddrUp = 1;
	phy->spiSettings.fourWireMode = 1;
	phy->spiSettings.cmosPadDrvStrength = ADI_ADRV9025_CMOSPAD_DRV_STRONG;

	ret = adi_adrv9025_HwOpen(phy->madDevice, &phy->spiSettings);
	if (ret)
		return adrv9025_dev_err(phy);

	adi_common_LogLevelSet(&phy->madDevice->common,
			       ADI_HAL_LOG_ERR | ADI_HAL_LOG_WARN);


	ret = of_property_read_string(np, "adi,device-profile-name", &name);
	if (ret) {
		dev_err(&spi->dev, "error missing dt property: adi,device-profile-name\n");
		return -EINVAL;
	}

	ret = adi_adrv9025_ConfigFileLoad(phy->madDevice, name, &phy->deviceInitStruct);
	if (ret)
		return adrv9025_dev_err(phy);

	ret = of_property_read_string(np, "adi,init-profile-name", &name);
	if (ret) {
		dev_err(&spi->dev, "error missing dt property: adi,init-profile-name\n");
		return -EINVAL;
	}

	ret = adi_adrv9025_UtilityInitFileLoad(phy->madDevice, name, &phy->adrv9025PostMcsInitInst);
	if (ret)
		return adrv9025_dev_err(phy);

	adrv9025_clk_register(phy, "-rx_sampl_clk", __clk_get_name(phy->dev_clk), NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			      RX_SAMPL_CLK);

	adrv9025_clk_register(phy, "-tx_sampl_clk", __clk_get_name(phy->dev_clk), NULL,
			      CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			      TX_SAMPL_CLK);

	phy->clk_data.clks = phy->clks;
	phy->clk_data.clk_num = NUM_ADRV9025_CLKS;

	ret = of_clk_add_provider(np, of_clk_src_onecell_get,
				  &phy->clk_data);
	if (ret)
		goto out_disable_clocks;

	indio_dev->dev.parent = &spi->dev;

	if (np)
		indio_dev->name = np->name;
	else
		indio_dev->name = "adrv9025-phy";

	indio_dev->modes = INDIO_DIRECT_MODE;

	switch (id) {
	case ID_ADRV9025:
	case ID_ADRV9026:
	case ID_ADRV9029:
		indio_dev->info = &adrv9025_phy_info;
		indio_dev->channels = adrv9025_phy_chan;
		indio_dev->num_channels = ARRAY_SIZE(adrv9025_phy_chan);
		break;
	default:
		ret = -EINVAL;
		goto out_clk_del_provider;
	}

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto out_clk_del_provider;

	ret = adrv9025_register_axi_converter(phy);
	if (ret < 0)
		goto out_iio_device_unregister;

	ret = adrv9025_register_debugfs(indio_dev);
	if (ret < 0)
		dev_warn(&spi->dev, "%s: failed to register debugfs", __func__);

	if (spi->irq) {
		ret = devm_request_threaded_irq(
			&spi->dev, spi->irq, NULL, adrv9025_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, indio_dev->name,
			indio_dev);

		if (ret) {
			dev_err(&spi->dev, "request_irq() failed with %d\n",
				ret);
			goto out_iio_device_unregister;
		}
	}

	adi_adrv9025_ApiVersionGet(phy->madDevice, &apiVersion);
	adi_adrv9025_Shutdown(phy->madDevice);
	adi_adrv9025_HwClose(phy->madDevice);

	dev_info(&spi->dev,
		 "%s Rev %d, API version: %u.%u.%u.%u found",
		 spi_get_device_id(spi)->name,
		 phy->madDevice->devStateInfo.deviceSiRev,
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

static void adrv9025_remove(struct spi_device *spi)
{
	struct adrv9025_rf_phy *phy = adrv9025_spi_to_phy(spi);

	iio_device_unregister(phy->indio_dev);
	of_clk_del_provider(spi->dev.of_node);
	clk_disable_unprepare(phy->dev_clk);

	adrv9025_shutdown(phy);
}

static const struct spi_device_id adrv9025_id[] = {
	{ "adrv9025", ID_ADRV9025 },
	{ "adrv9026", ID_ADRV9026 },
	{ "adrv9029", ID_ADRV9029 },
	{}
};
MODULE_DEVICE_TABLE(spi, adrv9025_id);

static const struct of_device_id adrv9025_of_match[] = {
	{ .compatible = "adi,adrv9025" },
	{ .compatible = "adi,adrv9026" },
	{ .compatible = "adi,adrv9029" },
	{},
};
MODULE_DEVICE_TABLE(of, adrv9025_of_match);

static struct spi_driver adrv9025_driver = {
	.driver = {
			.name = "adrv9025",
			.of_match_table = of_match_ptr(adrv9025_of_match),
		},
	.probe = adrv9025_probe,
	.remove = adrv9025_remove,
	.id_table = adrv9025_id,
};
module_spi_driver(adrv9025_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADRV9025 TRX");
MODULE_LICENSE("GPL v2");
