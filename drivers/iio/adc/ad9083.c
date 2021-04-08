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
#include "ad9083/adi_ad9083.h"
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
 * @sampling_frequency_hz:	Sampling frequency of the device per channel
 */
struct ad9083_phy {
	adi_ad9083_device_t	adi_ad9083;
	struct axiadc_chip_info	chip_info;
	struct jesd204_dev	*jdev;
	struct jesd204_link	jesd204_link;
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
	u64 sampling_frequency_hz;
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
			dev_info(dev, "JESD RX (JTX) state_204b %x, SYNC %s, PLL %s, PHASE %s, MODE %s\n",
				 stat & 0x0f,
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
	lnk->sample_rate = phy->sampling_frequency_hz;
	lnk->sample_rate_div = 1;
	lnk->link_id = 0;

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9083_jesd204_clks_enable(struct jesd204_dev *jdev,
				      enum jesd204_state_op_reason reason,
				      struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		__LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

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
	struct ad9083_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9083_phy *phy = priv->phy;
	int ret;

	ret = ad9083_jesd_rx_link_status_print(phy);
	if (ret < 0)
		return JESD204_STATE_CHANGE_ERROR;

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

static int ad9083_setup(struct axiadc_converter *conv)
{
	struct ad9083_phy *phy = conv->phy;
	struct device *dev = &conv->spi->dev;
	adi_cms_chip_id_t chip_id;
	u64 temp;
	int ret;

	ret = ad9083_request_clks(conv);
	if (ret) {
		dev_err(dev, "ad9083_request_clks failed (%d)\n", ret);
		return ret;
	}

	/* software reset, resistor is not mounted */
	ret = adi_ad9083_device_reset(&phy->adi_ad9083, AD9083_SOFT_RESET);
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

	temp = phy->sampling_frequency_hz * phy->jesd204_link.num_converters;
	ret = adi_ad9083_device_clock_config_set(&phy->adi_ad9083, temp, clk_get_rate(conv->clk));
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

	ret = adi_ad9083_jtx_startup(&phy->adi_ad9083, &phy->jesd_param);
	if (ret < 0)
		dev_err(dev, "adi_ad9083_jtx_startup failed (%d)\n", ret);

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
	ret = of_property_read_u64(np, "adi,sampling-frequency-hz",
				   &phy->sampling_frequency_hz);
	if (ret)
		phy->sampling_frequency_hz = 125000000;

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

static struct axiadc_chip_info axiadc_chip_info_tbl = {
	.name = "AD9083",
	.max_rate = 2000000000,
	.num_channels = 16,
	.channel[0] = AIM_CHAN(0, IIO_MOD_I, 0, 16, 'S'),
	.channel[1] = AIM_CHAN(0, IIO_MOD_Q, 1, 16, 'S'),
	.channel[2] = AIM_CHAN(1, IIO_MOD_I, 2, 16, 'S'),
	.channel[3] = AIM_CHAN(1, IIO_MOD_Q, 3, 16, 'S'),
	.channel[4] = AIM_CHAN(2, IIO_MOD_I, 4, 16, 'S'),
	.channel[5] = AIM_CHAN(2, IIO_MOD_Q, 5, 16, 'S'),
	.channel[6] = AIM_CHAN(3, IIO_MOD_I, 6, 16, 'S'),
	.channel[7] = AIM_CHAN(3, IIO_MOD_Q, 7, 16, 'S'),
	.channel[8] = AIM_CHAN(4, IIO_MOD_I, 8, 16, 'S'),
	.channel[9] = AIM_CHAN(4, IIO_MOD_Q, 9, 16, 'S'),
	.channel[10] = AIM_CHAN(5, IIO_MOD_I, 10, 16, 'S'),
	.channel[11] = AIM_CHAN(5, IIO_MOD_Q, 11, 16, 'S'),
	.channel[12] = AIM_CHAN(6, IIO_MOD_I, 12, 16, 'S'),
	.channel[13] = AIM_CHAN(6, IIO_MOD_Q, 13, 16, 'S'),
	.channel[14] = AIM_CHAN(7, IIO_MOD_I, 14, 16, 'S'),
	.channel[15] = AIM_CHAN(7, IIO_MOD_Q, 15, 16, 'S'),
};

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

	conv->adc_clkscale.mult = 1;
	conv->adc_clkscale.div = 1;
	conv->spi = spi;
	conv->phy = phy;
	conv->chip_info = &axiadc_chip_info_tbl;
	conv->reg_access = ad9083_reg_access;

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

	ret = ad9083_parse_dt(phy, &spi->dev);
	if (ret < 0)
		return -ENODEV;

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
