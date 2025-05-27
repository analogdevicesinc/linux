// SPDX-License-Identifier: GPL-2.0+
/*
 * AD9213 ADC driver
 *
 * Copyright 2021-2025 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/backend.h>

#define JESD204_OF_PREFIX	"adi,"
#include <linux/jesd204/jesd204.h>
#include <linux/jesd204/jesd204-of.h>

#define AD9213_REG_SPI_CONFIG_A			0x0000
#define AD9213_SOFT_RESET_1			BIT(7)
#define AD9213_LSB_FIRST_1			BIT(6)
#define AD9213_ADDR_ASCENSION_1			BIT(5)
#define AD9213_ADDR_ASCENSION_0			BIT(2)
#define AD9213_LSB_FIRST_0			BIT(1)
#define AD9213_SOFT_RESET_0			BIT(0)

#define AD9213_REG_GEN_CTRL			0x0026
#define AD9213_CLK_SWITCH			BIT(3)

#define AD9213_REG_PLL_STATUS			0x0501
#define AD9213_JTX_PLL_LOCKED			BIT(7)

#define AD9213_REG_JTX_LINK_CTRL2		0x0504
#define AD9213_JTX_SYNC_PIN_MODE(x)		((x) & 0x3)
#define AD9213_JTX_SYNC_PIN_INV			BIT(5)
#define AD9213_JTX_SYNC_PIN_TYPE		BIT(4)
#define AD9213_JTX_8B10B_BYPASS			BIT(2)
#define AD9213_JTX_10B_INV			BIT(1)
#define AD9213_JTX_10B_MIRROR			BIT(0)

#define AD9213_REG_JTX_SYNC_CTRL		0x0508
#define AD9213_SPI_CMOS_EN_RC			BIT(5)

#define AD9213_REG_JTX_LMFC_OFFSET		0x050A
#define AD9213_JTX_LMFC_OFFSET(x)		((x) & 0x1f)

#define AD9213_REG_JTX_SCR_L_CFG		0x0520
#define AD9213_JTX_SCR_CFG			BIT(7)
#define AD9213_JTX_L_CFG(x)			((x) & 0x1f)

#define AD9213_REG_JTX_F_CFG			0x0521
#define AD9213_JTX_F_CFG(x)			((x) & 0xff)

#define AD9213_REG_JTX_K_CFG			0x0522
#define AD9213_JTX_K_CFG(x)			((x) & 0x1f)

#define AD9213_REG_JTX_M_CFG			0x0523
#define AD9213_JTX_M_CFG(x)			((x) & 0xff)

#define AD9213_REG_JTX_CS_N_CFG			0x0524
#define AD9213_JTX_CS_CFG(x)			(((x) & 0x3) << 6)
#define AD9213_JTX_N_CFG(x)			((x) & 0x1f)

#define AD9213_REG_JTX_SCV_NP_CFG		0x0525
#define AD9213_JTX_SUBCLASSV_CFG(x)		(((x) & 0x7) << 5)
#define AD9213_JTX_NP_CFG(x)			((x) & 0x1f)

#define AD9213_REG_PLL_ENABLE_CTRL		0x0570
#define AD9213_LOLSTICKYCLEAR_FORCE_LCPLL_RC	BIT(4)
#define AD9213_LDSYNTH_FORCE_LCPLL_ADC		BIT(2)
#define AD9213_PWRUP_LCPLL			BIT(0)

#define AD9213_REG_CHIP_DP_MODE			0x0606
#define AD9213_CHIP_I_ONLY			BIT(5)
#define AD9213_DDC_0_ONLY			BIT(1)

#define AD9213_REG_CHIP_DEC_RATIO		0x0607
#define AD9213_CHIP_DEC_RATIO(x)		((x) & 0xf)

#define AD9213_REG_OUT_RES			0x0626
#define AD9213_DFORMAT_FBW_DITHER_EN		BIT(4)
#define AD9213_DFORMAT_RES(x)			((x) & 0xf)

#define AD9213_REG_DDC_CTRL			0x0630
#define AD9213_DDC0_C2R_EN			BIT(4)
#define AD9213_DDC0_IF_MODE(x)			(((x) & 0x3) << 2)
#define AD9213_DDC0_GAIN			BIT(1)

#define AD9213_REG_DDC_DEC_CTRL			0x0631
#define AD9213_DDC0_DEC_SEL(x)			((x) & 0xf)

#define AD9213_REG_JTX_CLK			0x0681
#define AD9213_JTX_CLK_EN			BIT(4)

#define AD9213_WRITE				(0 << 15)
#define AD9213_READ				BIT(15)
#define AD9213_ADDR(x)				((x) & 0x7fff)

struct ad9213 {
	struct spi_device	*spi;
	struct gpio_desc	*reset_gpio;
	struct clk		*jesd_clk;
	bool			syncinb_cmos_en;
	u32			lmfc_offset;
	bool			is_initialized;
	int			spi_device_id;
	struct jesd204_dev	*jdev;
	struct jesd204_link	jesd204_link;
	/* protect against device accesses */
	struct mutex		lock;
	u64			adc_frequency_hz;
	struct iio_backend	*back;
};

enum {
	AD9213_JESD204_FSM_ERROR,
	AD9213_JESD204_FSM_PAUSED,
	AD9213_JESD204_FSM_STATE,
	AD9213_JESD204_FSM_RESUME,
	AD9213_JESD204_FSM_CTRL,
};

struct ad9213_jesd204_priv {
	struct ad9213 *adc;
};

enum ad9213_device_id {
	ID_AD9213,
};

static int ad9213_write(struct ad9213 *adc,
			unsigned int reg,
			unsigned int val)
{
	unsigned char buf[3];
	u16 cmd;

	cmd = AD9213_WRITE | AD9213_ADDR(reg);
	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xff;
	buf[2] = val;

	return spi_write(adc->spi, buf, ARRAY_SIZE(buf));
}

static int ad9213_read(struct ad9213 *adc,
		       unsigned int reg,
		       unsigned int *val)
{
	unsigned char buf[3];
	u16 cmd;
	int ret;

	cmd = AD9213_READ | AD9213_ADDR(reg);
	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xFF;

	ret = spi_write_then_read(adc->spi, &buf[0], 2, val, 1);

	return ret;
}

static int ad9213_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int write_val,
			     unsigned int *read_val)
{
	struct ad9213 *adc = iio_priv(indio_dev);
	int ret;

	mutex_lock(&adc->lock);
	if (read_val)
		ret = ad9213_read(adc, reg, read_val);
	else
		ret = ad9213_write(adc, reg, write_val);
	mutex_unlock(&adc->lock);

	return ret;
}

static ssize_t ad9213_phy_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9213 *adc = iio_priv(indio_dev);
	bool enable;
	int ret = 0;

	mutex_lock(&adc->lock);
	dev_info(&adc->spi->dev, "is_initialized in phy_store: %d", adc->is_initialized);
	switch ((u32)this_attr->address & 0xFF) {
	case AD9213_JESD204_FSM_RESUME:
		if (!adc->jdev) {
			ret = -EOPNOTSUPP;
			break;
		}
		ret = jesd204_fsm_resume(adc->jdev, JESD204_LINKS_ALL);
		break;

	case AD9213_JESD204_FSM_CTRL:
		if (!adc->jdev) {
			ret = -EOPNOTSUPP;
			break;
		}
		ret = strtobool(buf, &enable);
		if (ret)
			break;
		if (jesd204_dev_is_top(adc->jdev)) {
			if (enable) {
				jesd204_fsm_stop(adc->jdev, JESD204_LINKS_ALL);
				jesd204_fsm_clear_errors(adc->jdev, JESD204_LINKS_ALL);
				ret = jesd204_fsm_start(adc->jdev, JESD204_LINKS_ALL);
			} else {
				jesd204_fsm_stop(adc->jdev, JESD204_LINKS_ALL);
				jesd204_fsm_clear_errors(adc->jdev, JESD204_LINKS_ALL);
				ret = 0;
			}
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&adc->lock);

	return ret ? ret : len;
}

static ssize_t ad9213_phy_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9213 *adc = iio_priv(indio_dev);
	int ret = 0;
	int i, err, num_links;
	bool paused;
	struct jesd204_link *links[8];

	switch ((u32)this_attr->address & 0xFF) {
	case AD9213_JESD204_FSM_ERROR:
		if (!adc->jdev) {
			ret = -EOPNOTSUPP;
			break;
		}
		num_links = jesd204_get_active_links_num(adc->jdev);
		if (num_links < 0) {
			ret = num_links;
			break;
		}

		ret = jesd204_get_links_data(adc->jdev, links, num_links);
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

	case AD9213_JESD204_FSM_PAUSED:
		if (!adc->jdev) {
			ret = -EOPNOTSUPP;
			break;
		}
		num_links = jesd204_get_active_links_num(adc->jdev);
		if (num_links < 0) {
			ret = num_links;
			break;
		}
		ret = jesd204_get_links_data(adc->jdev, links, num_links);
		if (ret)
			break;
		/*
		 * Take the slowest link; if there are N links and one is
		 * paused, all are paused. Not sure if this can happen yet,
		 * but best design it like this here.
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

	case AD9213_JESD204_FSM_STATE:
		if (!adc->jdev)
			ret = -EOPNOTSUPP;
		num_links = jesd204_get_active_links_num(adc->jdev);
		if (num_links < 0) {
			ret = num_links;
			break;
		}
		ret = jesd204_get_links_data(adc->jdev, links, num_links);
		if (ret)
			break;
		/*
		 * just get the first link state; we're assuming that all 3
		 * are in sync and that AD9081_JESD204_FSM_PAUSED
		 * was called before
		 */
		ret = sprintf(buf, "%s\n",
			      jesd204_link_get_state_str(links[0]));
		break;

	case AD9213_JESD204_FSM_CTRL:
		if (!adc->jdev) {
			ret = -EOPNOTSUPP;
			break;
		}

		ret = sprintf(buf, "%d\n", adc->is_initialized);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&adc->lock);
	return ret;
}

static int ad9213_setup(struct ad9213 *adc)
{
	/* Power up the AD9213. */
	/* Assert a Pin Reset. */
	gpiod_set_value_cansleep(adc->reset_gpio, 0);
	mdelay(1);
	gpiod_set_value_cansleep(adc->reset_gpio, 1);
	mdelay(1);

	/* Write Register 0x0 = 0x24 to set the SPI address ascensionmode to increment */
	ad9213_write(adc, AD9213_REG_SPI_CONFIG_A,
		     AD9213_ADDR_ASCENSION_1 | AD9213_ADDR_ASCENSION_0);
	/* Wait 100 ms before performing the next step. */
	mdelay(100);

	/* Write Register 0x26 = 0x08 to configure the AD9213 to receive sample clock signal.
	 * Sample clock must be applied before this step is performed.
	 */
	ad9213_write(adc, AD9213_REG_GEN_CTRL, AD9213_CLK_SWITCH);
	/* 6. Wait 100 ms before performing the next step. */
	mdelay(100);

	/* Perform the user specific configuration of DDC, NCO, and JESD, which varies depending
	 * on the required application.
	 */

	/* Example: 16 lane, M = 2, complex decimate by 8, N = N' = 16, no NCO */

	/* Write Register 0x504,  Bit 4 = 1. Set SYNC pin logic type: 0 = CMOS, 1 = LVDS. */
	if (adc->syncinb_cmos_en) {
		ad9213_write(adc, AD9213_REG_JTX_LINK_CTRL2, 0x00);
		ad9213_write(adc, AD9213_REG_JTX_SYNC_CTRL,
			     AD9213_SPI_CMOS_EN_RC);
	} else {
		ad9213_write(adc, AD9213_REG_JTX_LINK_CTRL2,
			     AD9213_JTX_SYNC_PIN_TYPE);
	}

	/* Write Register 0x520 = 0x8F. Scrambler on, L = 16 lanes. */
	ad9213_write(adc, AD9213_REG_JTX_SCR_L_CFG,
		     AD9213_JTX_SCR_CFG | AD9213_JTX_L_CFG(adc->jesd204_link.num_lanes - 1));

	/* Write Register 0x521 = 0x01. Set F = 2. */
	ad9213_write(adc, AD9213_REG_JTX_F_CFG,
		     AD9213_JTX_F_CFG(adc->jesd204_link.octets_per_frame - 1));

	/* Write Register 0x522 = 0x1F. K = 32 (default). */
	ad9213_write(adc, AD9213_REG_JTX_K_CFG,
		     AD9213_JTX_K_CFG(adc->jesd204_link.frames_per_multiframe - 1));

	/* Write Register 0x523 = 0x01. M = 2 (2 virtual converters, I/Q). */
	ad9213_write(adc, AD9213_REG_JTX_M_CFG,
		     AD9213_JTX_M_CFG(adc->jesd204_link.num_converters - 1));

	/* Write Register 0x524 = 0xCF.  Set N = 16. */
	ad9213_write(adc, AD9213_REG_JTX_CS_N_CFG,
		     AD9213_JTX_CS_CFG(0x3) |
		     AD9213_JTX_N_CFG(adc->jesd204_link.converter_resolution - 1));

	/* DFORMAT_RES should be set the same as jesd_N */
	ad9213_write(adc, AD9213_REG_OUT_RES,
		     AD9213_DFORMAT_RES(16 - adc->jesd204_link.converter_resolution));

	/* Write Register 0x525 = 0x0F. N' = 16, Subclass 0 operation. */
	ad9213_write(adc, AD9213_REG_JTX_SCV_NP_CFG,
		     AD9213_JTX_SUBCLASSV_CFG(adc->jesd204_link.subclass) |
		     AD9213_JTX_N_CFG(adc->jesd204_link.bits_per_sample - 1));

	ad9213_write(adc, AD9213_REG_JTX_LMFC_OFFSET,
		     AD9213_JTX_LMFC_OFFSET(adc->lmfc_offset));

	/* Write Register 0x606 = 0x01. Complex decimation enabled. */
	//ad9213_write(adc, AD9213_REG_CHIP_DP_MODE, AD9213_DDC_0_ONLY);
	/* full bandwidth mode */
	ad9213_write(adc, AD9213_REG_CHIP_DP_MODE, 0x00);

	/* Write Register 0x607 = 0x03. Chip decimate by 8 */
	//ad9213_write(adc, AD9213_REG_CHIP_DEC_RATIO, AD9213_CHIP_DEC_RATIO(0x3));
	/* full bandwidth mode */
	ad9213_write(adc, AD9213_REG_CHIP_DEC_RATIO, 0x00);

	/* Write Register 0x681 = 0x10. Enables DDC and JTX clocks. */
	ad9213_write(adc, AD9213_REG_JTX_CLK, AD9213_JTX_CLK_EN);

	/* Write Register 0x570, Bit 0 = 0, powers down JESD204B PLL. */
	ad9213_write(adc, AD9213_REG_PLL_ENABLE_CTRL, 0x00);

	/* Write Register 0x570, Bit 0 = 1, powers up JESD204B PLL. */
	ad9213_write(adc, AD9213_REG_PLL_ENABLE_CTRL, AD9213_PWRUP_LCPLL);

	return 0;
}

static int ad9213_jesd204_link_init(struct jesd204_dev *jdev,
				    enum jesd204_state_op_reason reason,
				    struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9213_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9213 *adc = priv->adc;
	struct jesd204_link *link;

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	ad9213_setup(adc);

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		__LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	link = &adc->jesd204_link;

	jesd204_copy_link_params(lnk, link);

	lnk->sample_rate = adc->adc_frequency_hz;
	lnk->jesd_encoder = JESD204_ENCODER_8B10B;

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9213_jesd204_clks_sync2(struct jesd204_dev *jdev,
				     enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9213_jesd204_link_running(struct jesd204_dev *jdev,
				       enum jesd204_state_op_reason reason,
				       struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9213_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9213 *adc = priv->adc;
	unsigned int status;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		adc->is_initialized = false;
		return JESD204_STATE_CHANGE_DONE;
	}

	/* Read Register 0x501. Confirm if JESD204B PLL is locked, Bit 7 = 1 indicates lock. */
	ad9213_read(adc, AD9213_REG_PLL_STATUS, &status);
	dev_info(&adc->spi->dev, "AD9213 PLL %s\n",
		 status & AD9213_JTX_PLL_LOCKED ? "LOCKED" : "UNLOCKED");

	adc->is_initialized = true;

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_ad9213_init = {
	.state_ops = {
		[JESD204_OP_LINK_INIT] = {
			.per_link = ad9213_jesd204_link_init,
		},
		[JESD204_OP_CLK_SYNC_STAGE2] = {
			.per_device = ad9213_jesd204_clks_sync2,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = ad9213_jesd204_link_running,
		},
	},

	.max_num_links = 1,
	.num_retries = 3,
	.sizeof_priv = sizeof(struct ad9213_jesd204_priv),
};

static IIO_DEVICE_ATTR(jesd204_fsm_error, 0444,
		       ad9213_phy_show,
		       NULL,
		       AD9213_JESD204_FSM_ERROR);

static IIO_DEVICE_ATTR(jesd204_fsm_paused, 0444,
		       ad9213_phy_show,
		       NULL,
		       AD9213_JESD204_FSM_PAUSED);

static IIO_DEVICE_ATTR(jesd204_fsm_state, 0444,
		       ad9213_phy_show,
		       NULL,
		       AD9213_JESD204_FSM_STATE);

static IIO_DEVICE_ATTR(jesd204_fsm_resume, 0200,
		       NULL,
		       ad9213_phy_store,
		       AD9213_JESD204_FSM_RESUME);

static IIO_DEVICE_ATTR(jesd204_fsm_ctrl, 0644,
		       ad9213_phy_show,
		       ad9213_phy_store,
		       AD9213_JESD204_FSM_CTRL);

static struct attribute *ad9213_phy_attributes[] = {
	&iio_dev_attr_jesd204_fsm_error.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_state.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_paused.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_resume.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_ctrl.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9213_phy_attribute_group = {
	.attrs = ad9213_phy_attributes,
};

static int ad9213_phy_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val,
			       int *val2,
			       long m)
{
	struct ad9213 *adc = iio_priv(indio_dev);
	unsigned long long llval = adc->adc_frequency_hz;

	switch (m) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = lower_32_bits(llval);
		*val2 = upper_32_bits(llval);

		return IIO_VAL_INT_64;
	default:
		return 0;
	}

	return -EINVAL;
}

static const struct iio_chan_spec ad9213_adc_chan[] = {
	{
		/* RX */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 0,
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 12,
			.storagebits = 16,
			.shift = 4,
		},
	},
};

static int ad9213_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct ad9213 *adc = iio_priv(indio_dev);
	int ret;

	if (test_bit(0, scan_mask))
		ret = iio_backend_chan_enable(adc->back, 0);
	else
		ret = iio_backend_chan_disable(adc->back, 0);
	if (ret)
		return ret;

	return 0;
}

static const struct iio_info ad9213_iio_info = {
	.read_raw = &ad9213_phy_read_raw,
	.debugfs_reg_access = &ad9213_reg_access,
	.attrs = &ad9213_phy_attribute_group,
	.update_scan_mode = &ad9213_update_scan_mode,
};

static int ad9213_probe(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node;
	struct ad9213_jesd204_priv *priv;
	struct iio_dev *indio_dev;
	struct jesd204_dev *jdev;
	unsigned int status;
	struct ad9213 *adc;
	int ret;

	int id = spi_get_device_id(spi)->driver_data;

	jdev = devm_jesd204_dev_register(&spi->dev, &jesd204_ad9213_init);
	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	spi_set_drvdata(spi, indio_dev);

	adc = iio_priv(indio_dev);

	adc->spi = spi;
	adc->is_initialized = false;
	adc->spi_device_id = id;

	adc->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset",
						  GPIOD_OUT_HIGH);
	if (IS_ERR(adc->reset_gpio))
		return PTR_ERR(adc->reset_gpio);

	if (jdev) {
		adc->jdev = jdev;
		priv = jesd204_dev_priv(jdev);
		priv->adc = adc;
	}

	adc->syncinb_cmos_en =
		of_property_read_bool(np, "adi,syncinb-cmos-enable");

	adc->lmfc_offset = 0;
	of_property_read_u32(np, "adi,lmfc-offset", &adc->lmfc_offset);

	adc->adc_frequency_hz = 10000000000;
	ret = of_property_read_u64(np, "adi,adc-frequency-hz",
				   &adc->adc_frequency_hz);
	if (ret)
		return ret;

	JESD204_LNK_READ_NUM_LANES(&spi->dev, np,
				   &adc->jesd204_link,
				   &adc->jesd204_link.num_lanes, 16);

	JESD204_LNK_READ_NUM_CONVERTERS(&spi->dev, np,
					&adc->jesd204_link,
					&adc->jesd204_link.num_converters, 2);

	JESD204_LNK_READ_OCTETS_PER_FRAME(&spi->dev, np,
					  &adc->jesd204_link,
					  &adc->jesd204_link.octets_per_frame, 2);

	JESD204_LNK_READ_SAMPLES_PER_CONVERTER_PER_FRAME(&spi->dev, np,
							 &adc->jesd204_link,
							 &adc->jesd204_link.samples_per_conv_frame,
							 16);

	JESD204_LNK_READ_HIGH_DENSITY(&spi->dev, np,
				      &adc->jesd204_link,
				      &adc->jesd204_link.high_density, 0);

	JESD204_LNK_READ_CONVERTER_RESOLUTION(&spi->dev, np,
					      &adc->jesd204_link,
					      &adc->jesd204_link.converter_resolution,
					      16);

	JESD204_LNK_READ_BITS_PER_SAMPLE(&spi->dev, np,
					 &adc->jesd204_link,
					 &adc->jesd204_link.bits_per_sample,
					 16);

	JESD204_LNK_READ_CTRL_BITS_PER_SAMPLE(&spi->dev, np,
					      &adc->jesd204_link,
					      &adc->jesd204_link.ctrl_bits_per_sample, 0);

	JESD204_LNK_READ_FRAMES_PER_MULTIFRAME(&spi->dev, np,
					       &adc->jesd204_link,
					       &adc->jesd204_link.frames_per_multiframe, 32);

	JESD204_LNK_READ_SUBCLASS(&spi->dev, np,
				  &adc->jesd204_link, &adc->jesd204_link.subclass, 0);

	mutex_init(&adc->lock);

	if (!adc->jdev) {
		ret = ad9213_setup(adc);
		if (ret)
			return ret;

		/* 11. Read Register 0x501. Confirm if JESD204B PLL is locked, Bit 7 = 1 indicates lock. */
		ad9213_read(adc, AD9213_REG_PLL_STATUS, &status);
		dev_info(&adc->spi->dev, "AD9213 PLL %s\n",
			 status & AD9213_JTX_PLL_LOCKED ? "LOCKED" : "UNLOCKED");
	}

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi->dev.of_node->name;
	indio_dev->info = &ad9213_iio_info;

	indio_dev->channels = ad9213_adc_chan;
	indio_dev->num_channels = 1;

	adc->back = devm_iio_backend_get(&spi->dev, NULL);
	if (IS_ERR(adc->back))
		return PTR_ERR(adc->back);

	ret = devm_iio_backend_request_buffer(&spi->dev, adc->back, indio_dev);
	if (ret)
		return ret;

	ret = devm_iio_backend_enable(&spi->dev, adc->back);
	if (ret)
		return ret;

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret)
		return ret;

	dev_info(&adc->spi->dev, "%s Probed\n", indio_dev->name);

	return jesd204_fsm_start(jdev, JESD204_LINKS_ALL);
}

static void ad9213_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);
}

static const struct spi_device_id ad9213_id[] = {
	{ "ad9213", ID_AD9213 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad9213_id);

static const struct of_device_id ad9213_of_match[] = {
	{ .compatible = "adi,ad9213" },
	{},
};
MODULE_DEVICE_TABLE(of, ad9213_of_match);

static struct spi_driver ad9213_driver = {
	.driver = {
			.name = "ad9213",
			.of_match_table = of_match_ptr(ad9213_of_match),
		},
	.probe = ad9213_probe,
	.remove = ad9213_remove,
	.id_table = ad9213_id,
};
module_spi_driver(ad9213_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9213 ADC");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_BACKEND);
