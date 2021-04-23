// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AD9083
 *
 * Copyright 2021 Analog Devices Inc.
 */


#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>

#define JESD204_OF_PREFIX	"adi,"
#include <linux/jesd204/jesd204.h>
#include <linux/jesd204/jesd204-of.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/iio/sysfs.h>
#include "ad9083/adi_ad9083.h"
#include "ad9083/adi_ad9083_hal.h"
#include "cf_axi_adc.h"

#include <dt-bindings/iio/adc/adi,ad9083.h>

#define IN_OUT_BUFF_SZ		3
#define MAX_REG_ADDR		0x1000
#define CHIPID_AD9083		0x00EA
#define CHIPID_MASK		0xFFFF
#define AD9083_ADC_TERM_RES_200 1	/*!< 200 Ohm */
#define AD9083_ADC_TERM_RES_100 2	/*!< 100 Ohm */

/** struct ad9083_jesd204_priv - Private data to be returned to the driver
 * @phy:			ad9083 phy
 */
struct ad9083_jesd204_priv {
	struct ad9083_phy *phy;
};

/**
 * struct ad9083_phy - Physical layer
 * @adi_ad9083:			Device structure
 * @chip_info:			Chip info and channels descritions
 * @jdev:			JESD204 device
 * @jesd204_link:		JESD204 link configuration settings
 * @lock:			Mutex protecting against concurrent accesses
 * @jesd_param:			Defines JESD Parameters
 * @vmax_micro:			Full scale voltage
 * @fc_hz:				Cut-off frequency of low-pass filter
 * @rterm_ohms:			Termination resistor: 100Ohm, 200Ohm, open
 * @hp_en:			Enable/disable high performance
 * @backoff:			The backoff in terms of noiseterms of noise, 100 * dB
 * @finmax_hz:			Max input
 * @nco_freq_hz:		NCO frequency
 * @decimation:			Decimation config
 * @nco0_datapath_mode:		NCO data path
 * @adc_frequency_hz:		ADC frequency of the device
 * @is_initialized:		Flag used to indicate operational jesd204 links
 * @total_dcm			Total decimation set by nco0_datapath_mode and decimation config
 */
struct ad9083_phy {
	adi_ad9083_device_t	adi_ad9083;
	struct axiadc_chip_info	chip_info;
	struct jesd204_dev	*jdev;
	struct jesd204_link	jesd204_link;
	/* Protect against concurrent accesses to the device */
	struct mutex		lock;
	adi_cms_jesd_param_t	jesd_param;
	u32 vmax_micro;
	u64 fc_hz;
	u32 rterm_ohms;
	bool hp_en;
	u32 backoff;
	u64 finmax_hz;
	u64 nco_freq_hz[3];
	u8 decimation[4];
	u8 nco0_datapath_mode;
	u64 adc_frequency_hz;
	bool is_initialized;
	u16 total_dcm;
};

/**
 * Possible JESD parameters values
 */
static const u8 ad9083_jesd_l[] = {1, 2, 3, 4};
static const u8 ad9083_jesd_f[] = {6, 8, 12, 16, 32, 48, 64};
static const u8 ad9083_jesd_k[] = {16, 32};
static const u8 ad9083_jesd_n[] = {12, 16};
static const u8 ad9083_jesd_np[] = {12, 16};
static const u8 ad9083_jesd_m[] = {8, 16, 32, 64, 96};

enum ad9083_iio_dev_attr {
	AD9083_JESD204_FSM_ERROR,
	AD9083_JESD204_FSM_PAUSED,
	AD9083_JESD204_FSM_STATE,
	AD9083_JESD204_FSM_RESUME,
	AD9083_JESD204_FSM_CTRL,
};

static int ad9083_udelay(void *user_data, unsigned int us)
{
	usleep_range(us, (us * 110) / 100);

	return 0;
}

static int ad9083_log_write(void *user_data, s32 log_type, const char *message,
			    va_list argp)
{
	struct axiadc_converter *conv = user_data;
	char logMessage[160];

	vsnprintf(logMessage, sizeof(logMessage), message, argp);

	switch (log_type) {
	case ADI_CMS_LOG_NONE:
		break;
	case ADI_CMS_LOG_MSG:
		dev_dbg(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_WARN:
		dev_dbg(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_ERR:
		dev_dbg(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_SPI:
		break;
	case ADI_CMS_LOG_API:
		dev_dbg(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_ALL:
		dev_dbg(&conv->spi->dev, "%s", logMessage);
		break;
	}

	return 0;
}

static int ad9083_reset_pin_ctrl(void *user_data, uint8_t enable)
{
	struct axiadc_converter *conv = user_data;

	if (conv->reset_gpio)
		return gpiod_direction_output(conv->reset_gpio, enable);

	return 0;
}

static int ad9083_spi_xfer(void *user_data, u8 *wbuf,
			   u8 *rbuf, u32 len)
{
	struct axiadc_converter *conv = user_data;
	int ret;
	struct spi_transfer t = {
		.tx_buf = wbuf,
		.rx_buf = rbuf,
		.len = len,
	};

	ret = spi_sync_transfer(conv->spi, &t, 1);

	dev_dbg(&conv->spi->dev, "%s: reg=0x%X, val=0x%X",
		(wbuf[0] & 0x80) ? "rd" : "wr",
		(wbuf[0] & 0x7F) << 8 | wbuf[1],
		(wbuf[0] & 0x80) ? rbuf[2] : wbuf[2]);

	return ret;
}

static int ad9083_register_write(adi_ad9083_device_t *h,
				 const u16 address, const u8 data)
{
	u8 inData[IN_OUT_BUFF_SZ];
	u8 outData[IN_OUT_BUFF_SZ];

	if (address >= MAX_REG_ADDR)
		return -EINVAL;

	inData[0] = address >> 8;
	inData[1] = address;
	inData[2] = data;

	return ad9083_spi_xfer(h->hal_info.user_data, inData, outData, IN_OUT_BUFF_SZ);
}

static int ad9083_register_read(adi_ad9083_device_t *h,
				const u16 address, u8 *data)
{
	int ret;
	u8 inData[IN_OUT_BUFF_SZ];
	u8 outData[IN_OUT_BUFF_SZ];

	if (address >= MAX_REG_ADDR)
		return -EINVAL;

	inData[0] = (address >> 8) | 0x80;
	inData[1] = address;
	ret = ad9083_spi_xfer(h->hal_info.user_data, inData, outData, IN_OUT_BUFF_SZ);
	if (ret != 0)
		return -EINVAL;

	*data = outData[2];

	return 0;
}

static int ad9083_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9083_phy *phy = conv->phy;
	int ret;
	u8 val;

	if (!readval)
		return ad9083_register_write(&phy->adi_ad9083, reg, writeval);

	ret = ad9083_register_read(&phy->adi_ad9083, reg, &val);
	if (ret < 0)
		return ret;

	*readval = val;

	return 0;
}

static const char *const ad9083_jtx_qbf_states[] = {
	"CGS", "ILA_M0R", "ILA_M0", "ILA_M1R", "ILA_M1C1", "ILA_M1C2",
	"ILA_M1C3", "ILA_M1", "ILA_M2R", "ILA_M2", "ILA_M3R", "ILA_M3",
	"ILA_BP", "DATA"
};

static int ad9083_jesd_rx_link_status_print(struct ad9083_phy *phy)
{
	u16 stat, retry = 3;
	struct device *dev = jesd204_dev_to_device(phy->jdev);
	s32 ret;

	do {
		ret = adi_ad9083_jesd_tx_link_status_get(&phy->adi_ad9083, &stat);
		if (ret)
			return -EFAULT;

		if ((stat & 0xFF) == 0x7D)
			ret = 0;
		else
			ret = -EIO;

		if (ret == 0 || retry == 0)
			dev_info(dev, "JESD RX (JTX) state_204b %s, SYNC %s, PLL %s, PHASE %s, MODE %s\n",
				 ad9083_jtx_qbf_states[stat & 0xF],
				 stat & BIT(4) ? "deasserted" : "asserted",
				 stat & BIT(5) ? "locked" : "unlocked",
				 stat & BIT(6) ? "established" : "lost",
				 stat & BIT(7) ? "invalid" : "valid");
		else
			ad9083_udelay(NULL, 20000);

	} while (ret && retry--);

	return 0;
}

static int ad9083_jesd204_link_init(struct jesd204_dev *jdev,
				    enum jesd204_state_op_reason reason,
				    struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9083_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9083_phy *phy = priv->phy;
	struct jesd204_link *link;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		__LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	link = &phy->jesd204_link;

	jesd204_copy_link_params(lnk, link);

	lnk->jesd_encoder = JESD204_ENCODER_8B10B;
	lnk->sample_rate = phy->adc_frequency_hz;
	lnk->sample_rate_div = phy->total_dcm;
	lnk->link_id = 0;

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9083_jesd204_clks_enable(struct jesd204_dev *jdev,
				      enum jesd204_state_op_reason reason,
				      struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9083_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9083_phy *phy = priv->phy;
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		__LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	if (lnk->subclass == JESD204_SUBCLASS_1) {
		adi_ad9083_hal_bf_set(&phy->adi_ad9083,
			BF_JTX_TPL_SYSREF_CLR_PHASE_ERR_INFO, 1);
		adi_ad9083_hal_bf_set(&phy->adi_ad9083,
			BF_JTX_TPL_SYSREF_CLR_PHASE_ERR_INFO, 0);
	}

	ret = adi_ad9083_jesd_tx_link_digital_reset(&phy->adi_ad9083, 1);
	if (ret)
		return JESD204_STATE_CHANGE_ERROR;
	mdelay(1);
	ret = adi_ad9083_jesd_tx_link_digital_reset(&phy->adi_ad9083, 0);
	if (ret)
		return JESD204_STATE_CHANGE_ERROR;
	mdelay(1);

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9083_jesd204_link_enable(struct jesd204_dev *jdev,
				      enum jesd204_state_op_reason reason,
				      struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		__LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9083_jesd204_link_running(struct jesd204_dev *jdev,
				       enum jesd204_state_op_reason reason,
				       struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9083_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9083_phy *phy = priv->phy;
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		phy->is_initialized = false;
		return JESD204_STATE_CHANGE_DONE;
	}

	if (lnk->subclass == JESD204_SUBCLASS_1) {
		u8 stat;

		ret = ad9083_register_read(&phy->adi_ad9083,
			REG_JTX_TPL_CONFIG1_ADDR, &stat);
		if (ret)
			return JESD204_STATE_CHANGE_ERROR;

		if ((stat & 0xF) != (BIT(1) | BIT(3)))
			dev_err(dev,
				"JTX TPL ERROR link_num %u: CONFIG: %s, SYSREF: %s %s, LMFC PHASE: %s\n",
				lnk->link_id,
				stat & BIT(0) ? "Invalid" : "Valid",
				stat & BIT(1) ? "Received" : "Waiting",
				stat & BIT(2) ? "(Unaligned)" : "",
				stat & BIT(3) ? "Established" : "Lost");
	}

	ret = ad9083_jesd_rx_link_status_print(phy);
	if (ret < 0)
		return JESD204_STATE_CHANGE_ERROR;

	phy->is_initialized = true;

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_ad9083_init = {
	.state_ops = {
		[JESD204_OP_LINK_INIT] = {
			.per_link = ad9083_jesd204_link_init,
		},
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = ad9083_jesd204_clks_enable,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = ad9083_jesd204_link_enable,
			.post_state_sysref = true,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = ad9083_jesd204_link_running,
		},
	},

	.max_num_links = 1,
	.num_retries = 3,
	.sizeof_priv = sizeof(struct ad9083_jesd204_priv),
};

static void ad9083_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int ad9083_request_clks(struct axiadc_converter *conv)
{
	int ret;

	conv->clk = devm_clk_get(&conv->spi->dev, "adc_ref_clk");
	if (IS_ERR(conv->clk))
		return PTR_ERR(conv->clk);

	ret = clk_prepare_enable(conv->clk);
	if (ret)
		return ret;

	return devm_add_action_or_reset(&conv->spi->dev, ad9083_clk_disable, conv->clk);
}

static ssize_t ad9083_phy_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9083_phy *phy = conv->phy;
	bool enable;
	int ret = 0;

	mutex_lock(&phy->lock);

	switch ((u32)this_attr->address & 0xFF) {
	case AD9083_JESD204_FSM_RESUME:
		if (!phy->jdev) {
			ret = -ENOTSUPP;
			break;
		}

		ret = jesd204_fsm_resume(phy->jdev, JESD204_LINKS_ALL);
		break;
	case AD9083_JESD204_FSM_CTRL:
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

static ssize_t ad9083_phy_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9083_phy *phy = conv->phy;
	struct jesd204_dev *jdev = phy->jdev;
	struct jesd204_link *links[3];
	int ret = 0;
	int i, err, num_links;
	bool paused;

	mutex_lock(&phy->lock);

	switch ((u32)this_attr->address & 0xFF) {
	case AD9083_JESD204_FSM_ERROR:
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
	case AD9083_JESD204_FSM_PAUSED:
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
	case AD9083_JESD204_FSM_STATE:
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
		 * and that AD9083_JESD204_FSM_PAUSED was called before
		 */
		ret = sprintf(buf, "%s\n", jesd204_link_get_state_str(links[0]));
		break;
	case AD9083_JESD204_FSM_CTRL:
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

static IIO_DEVICE_ATTR(jesd204_fsm_error, S_IRUGO,
		       ad9083_phy_show,
		       NULL,
		       AD9083_JESD204_FSM_ERROR);

static IIO_DEVICE_ATTR(jesd204_fsm_paused, S_IRUGO,
		       ad9083_phy_show,
		       NULL,
		       AD9083_JESD204_FSM_PAUSED);

static IIO_DEVICE_ATTR(jesd204_fsm_state, S_IRUGO,
		       ad9083_phy_show,
		       NULL,
		       AD9083_JESD204_FSM_STATE);

static IIO_DEVICE_ATTR(jesd204_fsm_resume, S_IWUSR,
		       NULL,
		       ad9083_phy_store,
		       AD9083_JESD204_FSM_RESUME);

static IIO_DEVICE_ATTR(jesd204_fsm_ctrl, S_IWUSR | S_IRUGO,
		       ad9083_phy_show,
		       ad9083_phy_store,
		       AD9083_JESD204_FSM_CTRL);

static struct attribute *ad9083_phy_attributes[] = {
	&iio_dev_attr_jesd204_fsm_error.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_state.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_paused.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_resume.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_ctrl.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9083_phy_attribute_group = {
	.attrs = ad9083_phy_attributes,
};

static int ad9083_setup(struct axiadc_converter *conv)
{
	struct ad9083_phy *phy = conv->phy;
	struct device *dev = &conv->spi->dev;
	adi_cms_chip_id_t chip_id;
	u8 api_rev[3];
	int ret;

	ret = ad9083_request_clks(conv);
	if (ret) {
		dev_err(dev, "ad9083_request_clks failed (%d)\n", ret);
		return ret;
	}

	ret = adi_ad9083_device_reset(&phy->adi_ad9083,
		conv->reset_gpio ? AD9083_HARD_RESET : AD9083_SOFT_RESET);
	if (ret < 0) {
		dev_err(dev, "adi_ad9083_device_reset failed (%d)\n", ret);
		return ret;
	}

	ret = adi_ad9083_device_chip_id_get(&phy->adi_ad9083, &chip_id);
	if (ret < 0) {
		dev_err(dev, "adi_ad9083_device_chip_id_get failed (%d)\n", ret);
		return ret;
	}

	if ((chip_id.prod_id & CHIPID_MASK) != CHIPID_AD9083) {
		dev_err(dev, "Chip id check failed (%d)\n", chip_id.prod_id);
		return -ENOENT;
	}

	ret = adi_ad9083_device_init(&phy->adi_ad9083);
	if (ret < 0) {
		dev_err(dev, "adi_ad9083_device_init failed (%d)\n", ret);
		return ret;
	}

	ret = adi_ad9083_device_clock_config_set(&phy->adi_ad9083,
		phy->adc_frequency_hz, clk_get_rate(conv->clk));
	if (ret < 0) {
		dev_err(dev, "adi_ad9083_device_clock_config_set failed (%d)\n", ret);
		return ret;
	}

	ret = adi_ad9083_rx_adc_config_set(&phy->adi_ad9083, phy->vmax_micro, phy->fc_hz,
					   phy->rterm_ohms, phy->hp_en, phy->backoff,
					   phy->finmax_hz);
	if (ret < 0) {
		dev_err(dev, "adi_ad9083_rx_adc_config_set failed (%d)\n", ret);
		return ret;
	}

	ret = adi_ad9083_rx_datapath_config_set(&phy->adi_ad9083,
						phy->nco0_datapath_mode,
						phy->decimation, phy->nco_freq_hz);
	if (ret < 0) {
		dev_err(dev, "adi_ad9083_rx_datapath_config_set failed (%d)\n", ret);
		return ret;
	}

	if (phy->jesd_param.jesd_subclass == JESD204_SUBCLASS_1) {
		ret = adi_ad9083_hal_bf_set(&phy->adi_ad9083, BF_SYSREF_RESYNC_MODE_INFO, 1);
		if (ret)
			return ret;
		ret = adi_ad9083_hal_bf_set(&phy->adi_ad9083, BF_JTX_SYSREF_FOR_STARTUP_INFO, 1);
		if (ret)
			return ret;
		ret = adi_ad9083_hal_bf_set(&phy->adi_ad9083, BF_JTX_SYSREF_FOR_RELINK_INFO, 1);
		if (ret)
			return ret;
		ret = adi_ad9083_hal_bf_set(&phy->adi_ad9083, BF_SYSREF_RESYNC_MODE_INFO, 1);
		if (ret)
			return ret;
		ret = adi_ad9083_hal_bf_set(&phy->adi_ad9083, 0x00000D40, 0x00000101, 0);
		if (ret)
			return ret;
	}

	ret = adi_ad9083_jtx_startup(&phy->adi_ad9083, &phy->jesd_param);
	if (ret < 0)
		dev_err(dev, "adi_ad9083_jtx_startup failed (%d)\n", ret);

	adi_ad9083_device_api_revision_get(&phy->adi_ad9083, &api_rev[0],
					   &api_rev[1], &api_rev[2]);

	ret = adi_ad9083_rx_datapath_total_dec_get(&phy->adi_ad9083, &phy->total_dcm);
	if (ret < 0) {
		dev_err(dev, "adi_ad9083_rx_datapath_total_dec_get failed (%d)\n", ret);
		return ret;
	}

	dev_info(dev, "%s Rev. %u Grade %u (API %u.%u.%u) probed\n",
		 conv->chip_info->name, chip_id.dev_revision,
		 chip_id.prod_grade, api_rev[0], api_rev[1], api_rev[2]);

	return ret;
}

static int ad9083_validate_parameter(u8 par_val, const u8 *vals, u8 size)
{
	int i;

	for (i = 0; i < size; i++)
		if (par_val == vals[i])
			return 0;

	return -EINVAL;
}

static int ad9083_parse_dt(struct ad9083_phy *phy, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;

	/* AD9083 Config */
	ret = of_property_read_u64(np, "adi,adc-frequency-hz",
				   &phy->adc_frequency_hz);
	if (ret) {
		dev_err(dev, "Missing adi,adc-frequency-hz property\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "adi,vmax-microvolt", &phy->vmax_micro);
	if (ret)
		phy->vmax_micro = 1800;

	ret = of_property_read_u64(np, "adi,fc-hz", &phy->fc_hz);
	if (ret)
		phy->fc_hz = 800000000;

	ret = of_property_read_u32(np, "adi,rterm-ohms", &phy->rterm_ohms);
	if (ret)
		phy->rterm_ohms = AD9083_ADC_TERM_RES_OPEN;
	else if (phy->rterm_ohms == 100)
		phy->rterm_ohms = AD9083_ADC_TERM_RES_100;
	else if (phy->rterm_ohms == 200)
		phy->rterm_ohms = AD9083_ADC_TERM_RES_200;
	else
		phy->rterm_ohms = AD9083_ADC_TERM_RES_OPEN;

	phy->hp_en = of_property_read_bool(np, "adi,hp-en");

	of_property_read_u32(np, "adi,backoff", &phy->backoff);

	ret = of_property_read_u64(np, "adi,finmax-hz", &phy->finmax_hz);
	if (ret)
		phy->finmax_hz = 100000000;

	of_property_read_u64(np, "adi,nco0_freq-hz", &phy->nco_freq_hz[0]);

	of_property_read_u64(np, "adi,nco1_freq-hz", &phy->nco_freq_hz[1]);

	of_property_read_u64(np, "adi,nco2_freq-hz", &phy->nco_freq_hz[2]);

	ret = of_property_read_u8(np, "adi,cic_decimation", &phy->decimation[0]);
	if (ret)
		phy->decimation[0] = AD9083_CIC_DEC_4;

	ret = of_property_read_u8(np, "adi,j_decimation", &phy->decimation[1]);
	if (ret)
		phy->decimation[1] = AD9083_J_DEC_4;

	of_property_read_u8(np, "adi,g_decimation", &phy->decimation[2]);

	of_property_read_u8(np, "adi,h_decimation", &phy->decimation[3]);

	ret = of_property_read_u8(np, "adi,nco0_datapath_mode", &phy->nco0_datapath_mode);
	if (ret)
		phy->nco0_datapath_mode = AD9083_DATAPATH_ADC_CIC_J;

	/* JESD Link Config */

	phy->jesd_param.jesd_s = 1;
	phy->jesd_param.jesd_hd = 1;
	phy->jesd_param.jesd_scr = 1;

	JESD204_LNK_READ_NUM_LANES(dev, np, &phy->jesd204_link,
				   &phy->jesd_param.jesd_l, 4);
	if (ad9083_validate_parameter(phy->jesd_param.jesd_l,
				      ad9083_jesd_l,
				      ARRAY_SIZE(ad9083_jesd_l))) {
		dev_err(dev, "Invalid JESD number of lanes: %d\n", phy->jesd_param.jesd_l);
		return -EINVAL;
	}

	JESD204_LNK_READ_OCTETS_PER_FRAME(dev, np, &phy->jesd204_link,
					  &phy->jesd_param.jesd_f, 8);
	if (ad9083_validate_parameter(phy->jesd_param.jesd_f,
				      ad9083_jesd_f,
				      ARRAY_SIZE(ad9083_jesd_f))) {
		dev_err(dev, "Invalid JESD octets per frame: %d\n", phy->jesd_param.jesd_f);
		return -EINVAL;
	}

	JESD204_LNK_READ_FRAMES_PER_MULTIFRAME(dev, np, &phy->jesd204_link,
					       &phy->jesd_param.jesd_k, 32);
	if (ad9083_validate_parameter(phy->jesd_param.jesd_k,
				      ad9083_jesd_k,
				      ARRAY_SIZE(ad9083_jesd_k))) {
		dev_err(dev, "Invalid JESD frames per multiframe: %d\n", phy->jesd_param.jesd_k);
		return -EINVAL;
	}

	JESD204_LNK_READ_CONVERTER_RESOLUTION(dev, np, &phy->jesd204_link,
					      &phy->jesd_param.jesd_n, 16);
	if (ad9083_validate_parameter(phy->jesd_param.jesd_n,
				      ad9083_jesd_n,
				      ARRAY_SIZE(ad9083_jesd_n))) {
		dev_err(dev, "Invalid JESD converter resolution: %d\n", phy->jesd_param.jesd_n);
		return -EINVAL;
	}

	JESD204_LNK_READ_BITS_PER_SAMPLE(dev, np, &phy->jesd204_link,
					 &phy->jesd_param.jesd_np, 16);
	if (ad9083_validate_parameter(phy->jesd_param.jesd_np,
				      ad9083_jesd_np,
				      ARRAY_SIZE(ad9083_jesd_np))) {
		dev_err(dev, "Invalid JESD bits per sample: %d\n", phy->jesd_param.jesd_np);
		return -EINVAL;
	}

	JESD204_LNK_READ_NUM_CONVERTERS(dev, np, &phy->jesd204_link,
					&phy->jesd_param.jesd_m, 16);
	if (ad9083_validate_parameter(phy->jesd_param.jesd_m,
				      ad9083_jesd_m,
				      ARRAY_SIZE(ad9083_jesd_m))) {
		dev_err(dev, "Invalid JESD converters number: %d\n", phy->jesd_param.jesd_m);
		return -EINVAL;
	}

	JESD204_LNK_READ_SUBCLASS(dev, np, &phy->jesd204_link,
				  &phy->jesd_param.jesd_subclass, JESD_SUBCLASS_0);


	if (phy->jesd_param.jesd_subclass >= JESD_SUBCLASS_INVALID) {
		dev_err(dev, "Invalid JESD subclass value: %d\n", phy->jesd_param.jesd_subclass);
		return -EINVAL;
	}

	return 0;
}

#define AIM_CHAN(_chan, _mod, _si, _bits, _sign)			\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.modified = 1,						\
		.channel = _chan,					\
		.channel2 = _mod,					\
		.scan_index = _si,					\
		.scan_type = {						\
			.sign = _sign,					\
			.realbits = _bits,				\
			.storagebits = 16,				\
			.shift = 0,					\
		},							\
	}

static void ad9083_setup_chip_info_tbl(struct ad9083_phy *phy)
{
	bool complex;
	int i;

	if (phy->nco0_datapath_mode == AD9083_DATAPATH_ADC_CIC_NCO_J ||
	    phy->nco0_datapath_mode == AD9083_DATAPATH_ADC_CIC_NCO_G ||
	    phy->nco0_datapath_mode == AD9083_DATAPATH_ADC_CIC_NCO_G_H)
		complex = true;
	else
		complex = false;

	for (i = 0; i < phy->jesd_param.jesd_m; i++) {
		phy->chip_info.channel[i].type = IIO_VOLTAGE;
		phy->chip_info.channel[i].indexed = 1;
		phy->chip_info.channel[i].channel = complex ? i / 2 : i;
		phy->chip_info.channel[i].modified = complex ? 1 : 0;
		phy->chip_info.channel[i].channel2 = (i & 1) ? IIO_MOD_Q : IIO_MOD_I;
		phy->chip_info.channel[i].scan_index = i;
		phy->chip_info.channel[i].scan_type.sign = 'S';
		phy->chip_info.channel[i].scan_type.realbits = phy->jesd_param.jesd_n;
		phy->chip_info.channel[i].scan_type.storagebits = 16;
		phy->chip_info.channel[i].scan_type.shift = 0;
	}

	phy->chip_info.name = "AD9083";
	phy->chip_info.max_rate = 2000000000;
	phy->chip_info.num_channels = phy->jesd_param.jesd_m;
}

static int ad9083_probe(struct spi_device *spi)
{
	struct axiadc_converter *conv;
	struct ad9083_phy *phy;
	struct jesd204_dev *jdev;
	struct ad9083_jesd204_priv *priv;
	int ret;

	jdev = devm_jesd204_dev_register(&spi->dev, &jesd204_ad9083_init);
	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

	phy = devm_kzalloc(&spi->dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	spi_set_drvdata(spi, conv);

	conv->reset_gpio =
		devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(conv->reset_gpio))
		return PTR_ERR(conv->reset_gpio);

	conv->adc_clkscale.mult = 1;
	conv->adc_clkscale.div = 1;
	conv->spi = spi;
	conv->phy = phy;
	conv->chip_info = &phy->chip_info;
	conv->reg_access = ad9083_reg_access;
	conv->attrs = &ad9083_phy_attribute_group;

	if (jdev) {
		phy->jdev = jdev;
		priv = jesd204_dev_priv(jdev);
		priv->phy = phy;
	}

	phy->adi_ad9083.hal_info.user_data = conv;
	phy->adi_ad9083.hal_info.spi_xfer = ad9083_spi_xfer;
	phy->adi_ad9083.hal_info.delay_us = ad9083_udelay;
	phy->adi_ad9083.hal_info.sdo = SPI_SDIO;
	phy->adi_ad9083.hal_info.msb = SPI_MSB_FIRST;
	phy->adi_ad9083.hal_info.addr_inc = SPI_ADDR_INC_AUTO;
	phy->adi_ad9083.hal_info.log_write = ad9083_log_write;
	phy->adi_ad9083.hal_info.reset_pin_ctrl = ad9083_reset_pin_ctrl;

	ret = ad9083_parse_dt(phy, &spi->dev);
	if (ret < 0)
		return -ENODEV;

	ad9083_setup_chip_info_tbl(phy);

	ret = ad9083_setup(conv);
	if (ret < 0)
		return -ENODEV;

	return jesd204_fsm_start(jdev, JESD204_LINKS_ALL);
}

static const struct spi_device_id ad9083_id[] = {
	{ "ad9083", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad9083_id);

static const struct of_device_id ad9083_of_match[] = {
	{ .compatible = "adi,ad9083" },
	{}
};
MODULE_DEVICE_TABLE(of, ad9083_of_match);

static struct spi_driver ad9083_driver = {
	.driver = {
			.name = "ad9083",
			.of_match_table = ad9083_of_match,
		},
	.probe = ad9083_probe,
	.id_table = ad9083_id,
};
module_spi_driver(ad9083_driver);

MODULE_AUTHOR("Cristian Pop <cristian.pop@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9083 ADC");
MODULE_LICENSE("GPL v2");
