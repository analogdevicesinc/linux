// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AD9081 and similar mixed signal front end (MxFE®)
 *
 * Copyright 2019-2021 Analog Devices Inc.
 */
//#define DEBUG
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/clkdev.h>

#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>

#define JESD204_OF_PREFIX	"adi,"
#include <linux/jesd204/jesd204.h>
#include <linux/jesd204/jesd204-of.h>

#include "ad9081/adi_ad9081.h"
#include "ad9081/adi_ad9081_hal.h"
#include "cf_axi_adc.h"

#include <dt-bindings/iio/adc/adi,ad9081.h>

#define CHIPID_AD9081 0x9081
#define CHIPID_AD9082 0x9082
#define CHIPID_AD9988 0x9988
#define CHIPID_AD9986 0x9986
#define CHIPID_AD9177 0x9177
#define CHIPID_AD9207 0x9207
#define CHIPID_AD9209 0x9209

#define CHIPID_MASK 0xFFFF
#define ID_DUAL BIT(31)

#define MAX_NUM_MAIN_DATAPATHS 4
#define MAX_NUM_CHANNELIZER 8
#define MAX_NUM_RX_NCO_CHAN_REGS 16
#define MAX_NUM_TX_NCO_CHAN_REGS 31

#define for_each_cddc(bit, mask) \
	for ((bit) = 0; (bit) < MAX_NUM_MAIN_DATAPATHS; (bit)++) \
		if ((mask) & BIT(bit))

#define for_each_fddc(bit, mask) \
	for ((bit) = 0; (bit) < MAX_NUM_CHANNELIZER; (bit)++) \
		if ((mask) & BIT(bit))

enum {	CDDC_NCO_FREQ,
	FDDC_NCO_FREQ,
	CDDC_NCO_FREQ_AVAIL,
	FDDC_NCO_FREQ_AVAIL,
	CDDC_NCO_PHASE,
	FDDC_NCO_PHASE,
	FDDC_NCO_GAIN,
	CDDC_6DB_GAIN,
	FDDC_6DB_GAIN,
	DAC_MAIN_TEST_TONE_EN,
	DAC_CHAN_TEST_TONE_EN,
	DAC_MAIN_TEST_TONE_OFFSET,
	DAC_CHAN_TEST_TONE_OFFSET,
	TRX_CONVERTER_RATE,
	TRX_ENABLE,
	CDDC_FFH_HOPF_SET,
	ADC_CDDC_FFH_TRIG_HOP_EN,
	ADC_FFH_GPIO_MODE_SET,
	CDDC_FFH_INDEX_SET,
	DAC_FFH_GPIO_MODE_SET,
	DAC_FFH_FREQ_SET,
};

enum {
	AD9081_LOOPBACK_MODE,
	AD9081_ADC_CLK_PWDN,
	AD9081_MCS,
	AD9081_JESD204_FSM_ERROR,
	AD9081_JESD204_FSM_PAUSED,
	AD9081_JESD204_FSM_STATE,
	AD9081_JESD204_FSM_RESUME,
	AD9081_JESD204_FSM_CTRL,
	AD9081_POWER_DOWN,
};

struct ad9081_jesd204_priv {
	struct ad9081_phy *phy;
};
struct ad9081_jesd_link {
	adi_cms_jesd_param_t jesd_param;
	struct jesd204_link jesd204_link;
	u32 jrx_tpl_phase_adjust;
	u8 logiclane_mapping[8];
	u8 link_converter_select[16];
	unsigned long lane_rate_kbps;
	unsigned long lane_cal_rate_kbps;
};

enum ad9081_clocks {
	RX_SAMPL_CLK,
	TX_SAMPL_CLK,
	RX_SAMPL_CLK_LINK2, /* Dual Link */
	NUM_AD9081_CLKS,
};

struct ad9081_clock {
	struct clk_hw hw;
	struct spi_device *spi;
	struct ad9081_phy *phy;
	unsigned long rate;
	enum ad9081_clocks source;
};

#define to_clk_priv(_hw) container_of(_hw, struct ad9081_clock, hw)

struct dac_settings_cache {
	u16 chan_gain[MAX_NUM_CHANNELIZER];
	u16 main_test_tone_offset[MAX_NUM_CHANNELIZER];
	u16 chan_test_tone_offset[MAX_NUM_CHANNELIZER];
	s32 main_phase[MAX_NUM_CHANNELIZER];
	s32 chan_phase[MAX_NUM_CHANNELIZER];
	u8 main_test_tone_en[MAX_NUM_CHANNELIZER];
	u8 chan_test_tone_en[MAX_NUM_CHANNELIZER];
	u8 enable[MAX_NUM_CHANNELIZER];
};

struct device_settings_cache {
	u8 loopback_mode;
	u8 adc_clk_pwdn;
};

enum ad9081_debugfs_cmd {
	DBGFS_NONE,
	DBGFS_BIST_PRBS_JRX,
	DBGFS_BIST_PRBS_JRX_ERR,
	DBGFS_BIST_JRX_SPO_SET,
	DBGFS_BIST_JRX_SPO_SWEEP,
	DBGFS_BIST_JRX_2D_EYE,
	DBGFS_BIST_PRBS_JTX,
	DBGFS_DEV_API_INFO,
	DBGFS_DEV_CHIP_INFO,
	DBGFS_ENTRY_MAX,
};

struct ad9081_debugfs_entry {
	struct iio_dev *indio_dev;
	const char *propname;
	void *out_value;
	u32 val;
	u8 size;
	u8 cmd;
};

struct ad9081_phy {
	struct spi_device *spi;
	struct jesd204_dev *jdev;
	adi_ad9081_device_t ad9081;
	struct axiadc_chip_info chip_info;
	struct clk *dev_clk;
	struct bin_attribute 	bin;

	struct gpio_desc *rx1_en_gpio;
	struct gpio_desc *rx2_en_gpio;
	struct gpio_desc *tx1_en_gpio;
	struct gpio_desc *tx2_en_gpio;
	struct gpio_desc *ms_sync_en_gpio;
	struct regulator *supply_reg;

	struct clk *clks[NUM_AD9081_CLKS];
	struct clock_scale clkscale[NUM_AD9081_CLKS];
	struct ad9081_clock clk_priv[NUM_AD9081_CLKS];
	struct clk_onecell_data clk_data;

	struct delayed_work dwork;

	const char **rx_labels;
	const char **tx_labels;

	u32 mcs_cached_val;

	u32 multidevice_instance_count;
	bool dual_link_use_own_tpl_en;
	u32 lmfc_delay;
	u32 nco_sync_ms_extra_lmfc_num;
	bool nco_sync_direct_sysref_mode_en;
	u32 sysref_average_cnt_exp;
	bool sysref_continuous_dis;
	bool sysref_coupling_ac_en;
	bool sysref_cmos_input_en;
	u8 sysref_cmos_single_end_term_pos;
	u8 sysref_cmos_single_end_term_neg;

	bool config_sync_01_swapped;
	bool config_sync_0a_cmos_en;
	bool jrx_link_watchdog_en;
	bool is_initialized;
	bool tx_disable;
	bool rx_disable;
	bool standalone;

	struct device_settings_cache device_cache;

	u64 dac_frequency_hz;
	s64 tx_main_shift[MAX_NUM_MAIN_DATAPATHS];
	s64 tx_chan_shift[MAX_NUM_CHANNELIZER];
	u32 tx_dac_fsc[MAX_NUM_MAIN_DATAPATHS];
	u32 tx_main_interp;
	u32 tx_chan_interp;
	u8 tx_dac_chan_xbar[MAX_NUM_MAIN_DATAPATHS];
	u8 tx_dac_chan_xbar_1x_non1x[MAX_NUM_MAIN_DATAPATHS];
	u8 tx_main_ffh_select[MAX_NUM_MAIN_DATAPATHS];

	u8 tx_ffh_hopf_index[MAX_NUM_MAIN_DATAPATHS];
	u8 tx_ffh_hopf_mode[MAX_NUM_MAIN_DATAPATHS];
	s64 tx_ffh_hopf_vals[MAX_NUM_TX_NCO_CHAN_REGS][MAX_NUM_MAIN_DATAPATHS];
	bool tx_ffh_hopf_via_gpio_en;

	struct dac_settings_cache dac_cache;
	struct ad9081_jesd_link jrx_link_tx[2];

	u32 adc_main_decimation[MAX_NUM_MAIN_DATAPATHS];
	u32 adc_chan_decimation[MAX_NUM_CHANNELIZER];
	u32 adc_dcm[2];
	bool adc_invert_en[MAX_NUM_MAIN_DATAPATHS];
	u64 adc_frequency_hz;
	s64 rx_fddc_shift[MAX_NUM_CHANNELIZER];
	s64 rx_cddc_shift[MAX_NUM_RX_NCO_CHAN_REGS][MAX_NUM_MAIN_DATAPATHS];
	s32 rx_fddc_phase[MAX_NUM_CHANNELIZER];
	s32 rx_cddc_phase[MAX_NUM_RX_NCO_CHAN_REGS][MAX_NUM_MAIN_DATAPATHS];

	u32 rx_nyquist_zone[MAX_NUM_MAIN_DATAPATHS];
	u8 rx_cddc_c2r[MAX_NUM_MAIN_DATAPATHS];
	u8 rx_cddc_gain_6db_en[MAX_NUM_MAIN_DATAPATHS];
	u8 rx_fddc_gain_6db_en[MAX_NUM_CHANNELIZER];
	u8 rx_fddc_c2r[MAX_NUM_CHANNELIZER];
	u8 rx_fddc_dcm[MAX_NUM_CHANNELIZER];
	u8 rx_cddc_dcm[MAX_NUM_MAIN_DATAPATHS];
	u8 rx_fddc_mxr_if[MAX_NUM_CHANNELIZER];
	u8 rx_fddc_select;
	u8 rx_cddc_select;
	u8 rx_main_ffh_select[MAX_NUM_MAIN_DATAPATHS];
	u8 rx_main_ffh_index[MAX_NUM_MAIN_DATAPATHS];
	u8 rx_main_ffh_mode[MAX_NUM_MAIN_DATAPATHS];
	u8 rx_cddc_nco_channel_select_mode[MAX_NUM_MAIN_DATAPATHS];
	bool rx_main_ffh_trig_en[MAX_NUM_MAIN_DATAPATHS];
	bool rx_main_ffh_gpio_en[MAX_NUM_MAIN_DATAPATHS];

	adi_cms_chip_id_t chip_id;

	struct ad9081_jesd_link jtx_link_rx[2];
	short coeffs_i[196];
	short coeffs_q[196];

	char rx_chan_labels[MAX_NUM_CHANNELIZER][32];
	char tx_chan_labels[MAX_NUM_CHANNELIZER][32];

	struct ad9081_debugfs_entry debugfs_entry[DBGFS_ENTRY_MAX];
	u32 ad9081_debugfs_entry_index;
	u8 direct_lb_map;
	u8 rx_ffh_gpio_mux_sel[6];
	u8 sync_ms_gpio_num;
	char dbuf[1024];
};

static int adi_ad9081_adc_nco_sync(adi_ad9081_device_t *device,
				   u8 trigger_src,
				   u8 extra_lmfc_num)
{
	int err;

	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();

	err = adi_ad9081_hal_bf_set(device, REG_MAIN_AUTO_CLK_GATING_ADDR,
				    0x00000400, 7);
	AD9081_ERROR_RETURN(err);

	err = adi_ad9081_adc_ddc_coarse_sync_enable_set(device,
							AD9081_ADC_CDDC_ALL, 1);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_adc_ddc_coarse_sync_next_set(device,
						      AD9081_ADC_CDDC_ALL, 1);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_adc_ddc_coarse_trig_nco_reset_enable_set(
		device, AD9081_ADC_CDDC_ALL, 0);
	AD9081_ERROR_RETURN(err);

	err = adi_ad9081_adc_ddc_fine_sync_enable_set(device,
						      AD9081_ADC_FDDC_ALL, 1);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_adc_ddc_fine_sync_next_set(device, AD9081_ADC_FDDC_ALL,
						    1);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_adc_ddc_fine_trig_nco_reset_enable_set(
		device, AD9081_ADC_FDDC_ALL, 0);
	AD9081_ERROR_RETURN(err);

	err = adi_ad9081_device_nco_sync_mode_set(device, 0);
	AD9081_ERROR_RETURN(err);

	err = adi_ad9081_device_nco_sync_sysref_mode_set(device, trigger_src);
	AD9081_ERROR_RETURN(err);

	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_device_nco_sync_extra_lmfc_num_set(device,
							    extra_lmfc_num);
	AD9081_ERROR_RETURN(err);

	err = adi_ad9081_adc_ddc_coarse_sync_next_set(device,
						      AD9081_ADC_CDDC_ALL, 0);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_adc_ddc_fine_sync_next_set(device, AD9081_ADC_FDDC_ALL,
						    0);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_adc_ddc_coarse_sync_next_set(device,
						      AD9081_ADC_CDDC_ALL, 1);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_adc_ddc_fine_sync_next_set(device, AD9081_ADC_FDDC_ALL,
						    1);
	AD9081_ERROR_RETURN(err);

	err = adi_ad9081_device_nco_sync_reset_via_sysref_set(device, 0);
	AD9081_ERROR_RETURN(err);
	err = adi_ad9081_device_nco_sync_reset_via_sysref_set(device, 1);
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

int adi_ad9081_device_gpio_set_highz(adi_ad9081_device_t *device, u8 gpio_index)
{
	int err;

	AD9081_NULL_POINTER_RETURN(device);
	AD9081_LOG_FUNC();
	AD9081_INVALID_PARAM_RETURN(gpio_index > 5);

	if ((gpio_index & 1) == 0) {
		err = adi_ad9081_hal_bf_set(device,
			REG_GPIO_CFG0_ADDR + (gpio_index >> 1), 0x0400, 0);
		AD9081_ERROR_RETURN(err);
	} else {
		err = adi_ad9081_hal_bf_set(device,
			REG_GPIO_CFG0_ADDR + (gpio_index >> 1), 0x0404, 0);
		AD9081_ERROR_RETURN(err);
	}

	return API_CMS_ERROR_OK;
}

static int ad9081_nco_sync(struct ad9081_phy *phy, bool master)
{
	int ret;

	ret = adi_ad9081_device_nco_sync_pre(&phy->ad9081);
	if (ret != 0)
		return ret;

	/* trigger_src  0: sysref, 1: lmfc rising edge, 2: lmfc falling edge */

	if (phy->nco_sync_direct_sysref_mode_en)
		return adi_ad9081_adc_nco_sync(&phy->ad9081,
			0, phy->nco_sync_ms_extra_lmfc_num);
	else
		return adi_ad9081_adc_nco_master_slave_sync(&phy->ad9081,
					     master,
					     1, /* trigger_src */
					     phy->sync_ms_gpio_num, /* gpio_index */
					     phy->nco_sync_ms_extra_lmfc_num);
}

int32_t ad9081_jesd_tx_link_dig_reset(adi_ad9081_device_t *device,
				      uint8_t reset)
{
	int32_t err;

	AD9081_NULL_POINTER_RETURN(device);
	AD9081_INVALID_PARAM_RETURN(reset > 1);

	err = adi_ad9081_hal_bf_set(device, REG_FORCE_LINK_RESET_REG_ADDR,
				    BF_FORCE_LINK_DIGITAL_RESET_INFO,
				    reset); /* not paged */
	AD9081_ERROR_RETURN(err);

	return API_CMS_ERROR_OK;
}

static bool ad9081_link_is_dual(struct ad9081_jesd_link *link)
{
	return !!link[0].jesd_param.jesd_duallink;
}

static adi_ad9081_jesd_link_select_e
ad9081_link_sel(struct ad9081_jesd_link *link)
{
	return ad9081_link_is_dual(link) ? AD9081_LINK_ALL : AD9081_LINK_0;
}

int32_t ad9081_log_write(void *user_data, int32_t log_type, const char *message,
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
		dev_warn(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_ERR:
		dev_err(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_SPI:
		break;
	case ADI_CMS_LOG_API:
		dev_dbg(&conv->spi->dev, "%s", logMessage);
		break;
	case ADI_CMS_LOG_ALL:
		printk(logMessage);
		break;
	}

	return 0;
}

static int ad9081_udelay(void *user_data, unsigned int us)
{
	usleep_range(us, (us * 110) / 100);
	return 0;
}

static int ad9081_spi_xfer(void *user_data, uint8_t *wbuf, uint8_t *rbuf,
			   uint32_t len)
{
	struct axiadc_converter *conv = user_data;
	struct spi_transfer t = {
		.tx_buf = wbuf,
		.rx_buf = rbuf,
		.len = len & 0xFFFF,
	};

	if (conv->spi->mode & SPI_LSB_FIRST) {
		int ret, i;
		u8 tx[64], rx[64];

		struct spi_transfer t = {
			.tx_buf = tx,
			.rx_buf = rx,
			.len = len & 0xFFFF,
		};

		len &= 0xFFFF;

		if (len > sizeof(tx))
			return -EIO;

		tx[0] = wbuf[1];
		tx[1] = wbuf[0];

		for (i = 2; i < len; i++)
			tx[i] =  wbuf[len - i + 1];

		ret = spi_sync_transfer(conv->spi, &t, 1);

		for (i = 2; i < len; i++)
			rbuf[i] =  rx[len - i + 1];

		return ret;
	}

	return spi_sync_transfer(conv->spi, &t, 1);
}

static int ad9081_reset_pin_ctrl(void *user_data, u8 enable)
{
	struct axiadc_converter *conv = user_data;

	return gpiod_direction_output(conv->reset_gpio, enable);
}

static int ad9081_sysref_ctrl(void *clk_src)
{
	struct ad9081_phy *phy = clk_src;

	if (phy->jdev)
		return jesd204_sysref_async_force(phy->jdev);

	return 0;
}

static int ad9081_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	u8 val;
	int ret;

	if (reg & 0x40000000) { /* CBUS Access */
		if (readval == NULL)
			return adi_ad9081_device_cbusjrx_register_set(&phy->ad9081,
				reg & 0xFF, writeval, (reg >> 8) & 0x7);

		ret = adi_ad9081_device_cbusjrx_register_get(&phy->ad9081,
			reg & 0xFF, &val, (reg >> 8) & 0x7);
		if (ret < 0)
			return ret;
	} else {
		if (readval == NULL)
			return adi_ad9081_hal_reg_set(&phy->ad9081, reg & 0x3FFF, writeval);

		ret = adi_ad9081_hal_reg_get(&phy->ad9081, reg & 0x3FFF, &val);
		if (ret < 0)
			return ret;
	}
	*readval = val;

	return 0;
}

#define AD9081_MAX_CLK_NAME 79

static char *ad9081_clk_set_dev_name(struct ad9081_phy *phy, char *dest,
				     const char *name)
{
	size_t len = 0;

	if (name == NULL)
		return NULL;

	if (*name == '-')
		len = strlcpy(dest, dev_name(&phy->spi->dev),
			      AD9081_MAX_CLK_NAME);
	else
		*dest = '\0';

	return strncat(dest, name, AD9081_MAX_CLK_NAME - len);
}

static unsigned long ad9081_bb_recalc_rate(struct clk_hw *hw,
					   unsigned long parent_rate)
{
	struct ad9081_clock *clk_priv = to_clk_priv(hw);

	return clk_priv->rate;
}

static int ad9081_bb_set_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long parent_rate)
{
	struct ad9081_clock *clk_priv = to_clk_priv(hw);

	clk_priv->rate = rate;

	return 0;
}

static long ad9081_bb_round_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long *prate)
{
	struct ad9081_clock *clk_priv = to_clk_priv(hw);

	dev_dbg(&clk_priv->spi->dev, "%s: Rate %lu Hz", __func__, rate);

	return rate;
}

static int ad9081_bb_determine_rate(struct clk_hw *hw,
				      struct clk_rate_request *req)
{
	return 0;
}

static const struct clk_ops bb_clk_ops = {
	.round_rate = ad9081_bb_round_rate,
	.determine_rate = ad9081_bb_determine_rate,
	.set_rate = ad9081_bb_set_rate,
	.recalc_rate = ad9081_bb_recalc_rate,
};

static int ad9081_clk_register(struct ad9081_phy *phy, const char *name,
			       const char *parent_name,
			       const char *parent_name2, unsigned long flags,
			       u32 source)
{
	struct ad9081_clock *clk_priv = &phy->clk_priv[source];
	struct clk_init_data init;
	struct clk *clk;
	char c_name[AD9081_MAX_CLK_NAME + 1],
		p_name[2][AD9081_MAX_CLK_NAME + 1];
	const char *_parent_name[2];

	/* struct ad9081_clock assignments */
	clk_priv->source = source;
	clk_priv->hw.init = &init;
	clk_priv->spi = phy->spi;
	clk_priv->phy = phy;

	_parent_name[0] = ad9081_clk_set_dev_name(phy, p_name[0], parent_name);
	_parent_name[1] = ad9081_clk_set_dev_name(phy, p_name[1], parent_name2);

	init.name = ad9081_clk_set_dev_name(phy, c_name, name);
	init.flags = flags;
	init.parent_names = &_parent_name[0];
	init.num_parents = _parent_name[1] ? 2 : _parent_name[0] ? 1 : 0;

	switch (source) {
	case RX_SAMPL_CLK:
	case RX_SAMPL_CLK_LINK2:
		init.ops = &bb_clk_ops;
		break;
	case TX_SAMPL_CLK:
		init.ops = &bb_clk_ops;
		break;
	default:
		return -EINVAL;
	}

	of_clk_get_scale(phy->spi->dev.of_node, &name[1],
				 &phy->clkscale[source]);

	clk = devm_clk_register(&phy->spi->dev, &clk_priv->hw);
	phy->clks[source] = clk;

	return 0;
}

#if 0
static unsigned int ad9081_pnsel_to_testmode(enum adc_pn_sel sel)
{
	switch (sel) {
	case ADC_PN9:
		return AD9081_TESTMODE_PN9_SEQ;
	case ADC_PN23A:
		return AD9081_TESTMODE_PN23_SEQ;
	default:
		return AD9081_TESTMODE_OFF;
	}
}

static int ad9081_testmode_set(struct iio_dev *indio_dev, unsigned int chan,
	unsigned int mode)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	int ret;

	ad9081_adc_set_channel_select(&phy->ad9081, BIT(chan & 1));
	/* FIXME: Add support for DDC testmodes */
	ret = ad9081_spi_write(conv->spi, AD9081_REG_TEST_MODE, mode);
	conv->testmode[chan] = mode;
	ad9081_adc_set_channel_select(&phy->ad9081, AD9081_ADC_CH_ALL);

	return ret;
}

static int ad9081_set_pnsel(struct iio_dev *indio_dev, unsigned int chan,
	enum adc_pn_sel sel)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned int mode = ad9081_pnsel_to_testmode(sel);
	unsigned int output_mode;
	int ret;

	output_mode = conv->adc_output_mode;
	if (mode != AD9081_TESTMODE_OFF)
		output_mode &= ~AD9081_OUTPUT_MODE_TWOS_COMPLEMENT;

	ret = ad9081_spi_write(conv->spi, AD9081_REG_OUTPUT_MODE, output_mode);
	if (ret < 0)
		return ret;

	return ad9081_testmode_set(indio_dev, chan, mode);
}

static int ad9081_read_thresh(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int *val,
	int *val2)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	u16 low, high;

	mutex_lock(&conv->lock);
	low = (ad9081_spi_read(spi, AD9081_FD_LT_MSB_REG) << 8) |
		ad9081_spi_read(spi, AD9081_FD_LT_LSB_REG);
	high = (ad9081_spi_read(spi, AD9081_FD_UT_MSB_REG) << 8) |
		ad9081_spi_read(spi, AD9081_FD_UT_LSB_REG);
	mutex_unlock(&conv->lock);

	switch (info) {
	case IIO_EV_INFO_HYSTERESIS:
		*val = high - low;
		break;
	case IIO_EV_INFO_VALUE:
		*val = high;
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static int ad9081_read_thresh_en(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret;

	ret = ad9081_spi_read(spi, AD9081_CHIP_PIN_CTRL1_REG);
	if (ret < 0)
		return ret;
	else
		return !(ret & AD9081_CHIP_PIN_CTRL_MASK(chan->channel));
}

static int ad9081_write_thresh(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int val,
	int val2)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret = 0;
	int low, high;

	mutex_lock(&conv->lock);
	high = (ad9081_spi_read(spi, AD9081_FD_UT_MSB_REG) << 8) |
		ad9081_spi_read(spi, AD9081_FD_UT_LSB_REG);

	switch (info) {
	case IIO_EV_INFO_HYSTERESIS:
		if (val < 0) {
			ret = -EINVAL;
			goto unlock;
		}

		low = high - val;
		break;

	case IIO_EV_INFO_VALUE:
		if (val > 0x7FF) {
			ret = -EINVAL;
			goto unlock;
		}

		ad9081_spi_write(spi, AD9081_FD_UT_MSB_REG, val >> 8);
		ad9081_spi_write(spi, AD9081_FD_UT_LSB_REG, val & 0xFF);

		/* Calculate the new lower threshold limit */
		low = (ad9081_spi_read(spi, AD9081_FD_LT_MSB_REG) << 8) |
			ad9081_spi_read(spi, AD9081_FD_LT_LSB_REG);
		low = val - high + low;
		break;

	default:
		ret = -EINVAL;
		goto unlock;
	}

	if (low < 0)
		low = 0;

	ad9081_spi_write(spi, AD9081_FD_LT_MSB_REG, low >> 8);
	ad9081_spi_write(spi, AD9081_FD_LT_LSB_REG, low & 0xFF);

unlock:
	mutex_unlock(&conv->lock);
	return ret;
}

static int ad9081_write_thresh_en(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, int state)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct spi_device *spi = conv->spi;
	int ret;

	mutex_lock(&conv->lock);

	ret = ad9081_spi_read(spi, AD9081_CHIP_PIN_CTRL1_REG);
	if (ret < 0)
		goto err_unlock;

	if (state)
		ret &= ~AD9081_CHIP_PIN_CTRL_MASK(chan->channel);
	else
		ret |= AD9081_CHIP_PIN_CTRL_MASK(chan->channel);

	ret = ad9081_spi_write(spi, AD9081_CHIP_PIN_CTRL1_REG, ret);
err_unlock:
	mutex_unlock(&conv->lock);
	return ret;
}
#endif

static irqreturn_t ad9081_event_handler(struct axiadc_converter *conv,
	unsigned int chn)
{
	u64 event = IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, chn,
			IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING);
	s64 timestamp = iio_get_time_ns(conv->indio_dev);

	if (conv->indio_dev)
		iio_push_event(conv->indio_dev, event, timestamp);

	return IRQ_HANDLED;
}

static irqreturn_t ad9081_fdA_handler(int irq, void *private)
{
	return ad9081_event_handler(private, 0);
}

static irqreturn_t ad9081_fdB_handler(int irq, void *private)
{
	return ad9081_event_handler(private, 1);
}

static int ad9081_testmode_read(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	return conv->testmode[chan->channel];
}

static int ad9081_testmode_write(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 unsigned int item)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	int ret;

	mutex_lock(&conv->lock);
	ret = adi_ad9081_adc_test_mode_config_set(&phy->ad9081, item, item,
						  AD9081_LINK_ALL);
	if (!ret)
		conv->testmode[chan->channel] = item;
	mutex_unlock(&conv->lock);

	return ret;
}

static const char *const ad9081_adc_testmodes[] = {
	[AD9081_TMODE_OFF] = "off",
	[AD9081_TMODE_MIDSCALE] = "midscale_short",
	[AD9081_TMODE_POS_FULL] = "pos_fullscale",
	[AD9081_TMODE_NEG_FULL] = "neg_fullscale",
	[AD9081_TMODE_ALT_CHECKER] = "checkerboard",
	[AD9081_TMODE_PN9] = "pn9",
	[AD9081_TMODE_PN23] = "pn23",
	[AD9081_TMODE_1_0_TOGG] = "one_zero_toggle",
	[AD9081_TMODE_USER_PAT] = "user",
	[AD9081_TMODE_PN7] = "pn7",
	[AD9081_TMODE_PN15] = "pn15",
	[AD9081_TMODE_PN31] = "pn31",
	[AD9081_TMODE_RAMP] = "ramp",
};

static const char *const ad9081_jesd_testmodes[] = {
	[AD9081_JESD_TX_TEST_MODE_DISABLED] = "off",
	[AD9081_JESD_TX_TEST_MODE_CHECKER_BOARD] = "checkerboard",
	[AD9081_JESD_TX_TEST_MODE_WORD_TOGGLE] = "word_toggle",
	[AD9081_JESD_TX_TEST_MODE_PN31] = "pn31",
	[AD9081_JESD_TX_TEST_MODE_PN15] = "pn15",
	[AD9081_JESD_TX_TEST_MODE_PN7] = "pn7",
	[AD9081_JESD_TX_TEST_MODE_RAMP] = "ramp",
	[AD9081_JESD_TX_TEST_MODE_USER_REPEAT] = "user_repeat",
	[AD9081_JESD_TX_TEST_MODE_USER_SINGLE] = "user_single",
};

static const struct iio_enum ad9081_testmode_enum = {
	.items = ad9081_adc_testmodes,
	.num_items = ARRAY_SIZE(ad9081_adc_testmodes),
	.set = ad9081_testmode_write,
	.get = ad9081_testmode_read,
};

int ad9081_iio_val_to_str(char *buf, u32 max, int val)
{
	int vals[2];

	vals[0] = val;
	vals[1] = max;

	return iio_format_value(buf, IIO_VAL_FRACTIONAL, 2, vals);
}

int ad9081_iio_str_to_val(const char *str, int min, int max, int *val)
{
	int ret, integer, fract;

	ret = iio_str_to_fixpoint(str, 100000, &integer, &fract);

	*val = DIV_ROUND_CLOSEST(
		max * (integer * 1000 + DIV_ROUND_CLOSEST(fract, 1000)), 1000);

	*val = clamp(*val, min, max);

	return ret;
}

static void ad9081_iiochan_to_fddc_cddc(struct ad9081_phy *phy,
	const struct iio_chan_spec *chan, u8 *fddc_num, u8 *fddc_mask,
	u8 *cddc_num, u8 *cddc_mask)
{
	unsigned int l = 0, m = 0, c;

	if (chan->output) {
		u8 mask = 0;
		int i;

		c = chan->channel;

		if (ad9081_link_is_dual(phy->jrx_link_tx)) {
			m = phy->jrx_link_tx[0].jesd_param.jesd_m;

			if (c >= (m / 2))
				c = c - (m / 2) + 4;
		}

		*fddc_num = c;
		*fddc_mask = BIT(c) & AD9081_DAC_CH_ALL;

		for (i = 0; i < ARRAY_SIZE(phy->tx_dac_chan_xbar); i++)
			if (phy->tx_dac_chan_xbar[i] & BIT(c)) {
				mask |= BIT(i);
				*cddc_num = i;

			}

		*cddc_mask = mask;
	} else {
		if (ad9081_link_is_dual(phy->jtx_link_rx)) {
			m = phy->jtx_link_rx[0].jesd_param.jesd_m;

			if (chan->address >= m)
				l = 1;
			else
				m = 0;
		}

		*fddc_num = phy->jtx_link_rx[l].link_converter_select[chan->address - m] / 2;
		*fddc_mask = BIT(*fddc_num) & AD9081_ADC_FDDC_ALL;

		adi_ad9081_adc_xbar_find_cddc(&phy->ad9081, *fddc_mask , cddc_mask);
		*cddc_num = ilog2(*cddc_mask & AD9081_ADC_CDDC_ALL);
	}

	dev_dbg(&phy->spi->dev,
		"%s_voltage%d: link=%d fddc_num=%d fddc_mask=%X cddc_num=%d cddc_mask=%X",
		chan->output ? "out" : "in", chan->channel, l,
		*fddc_num, *fddc_mask, *cddc_num, *cddc_mask);
}

static int ad9081_nyquist_zone_read(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	u8 cddc_num, cddc_mask, fddc_num, fddc_mask;

	ad9081_iiochan_to_fddc_cddc(phy, chan, &fddc_num,
		&fddc_mask, &cddc_num, &cddc_mask);

	return phy->rx_nyquist_zone[cddc_num];
}

static int ad9081_nyquist_zone_write(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 unsigned int item)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	u8 cddc_num, cddc_mask, fddc_num, fddc_mask;
	int ret;

	ad9081_iiochan_to_fddc_cddc(phy, chan, &fddc_num,
		&fddc_mask, &cddc_num, &cddc_mask);

	mutex_lock(&conv->lock);
	ret = adi_ad9081_adc_nyquist_zone_set(&phy->ad9081, cddc_mask, item);
	if (!ret)
		phy->rx_nyquist_zone[cddc_num] = item;
	mutex_unlock(&conv->lock);

	return ret;
}

static const char *const ad9081_adc_nyquist_zones[] = {
	[AD9081_ADC_NYQUIST_ZONE_ODD] = "odd",
	[AD9081_ADC_NYQUIST_ZONE_EVEN] = "even",
};

static const struct iio_enum ad9081_nyquist_zone_enum = {
	.items = ad9081_adc_nyquist_zones,
	.num_items = ARRAY_SIZE(ad9081_adc_nyquist_zones),
	.set = ad9081_nyquist_zone_write,
	.get = ad9081_nyquist_zone_read,
};

static int ad9081_main_ffh_mode_read(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	u8 cddc_num, cddc_mask, fddc_num, fddc_mask;

	ad9081_iiochan_to_fddc_cddc(phy, chan, &fddc_num,
		&fddc_mask, &cddc_num, &cddc_mask);

	if (chan->output)
		return phy->tx_ffh_hopf_mode[cddc_num];

	return phy->rx_main_ffh_mode[cddc_num];
}

static int ad9081_main_ffh_mode_write(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 unsigned int item)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	u8 cddc_num, cddc_mask, fddc_num, fddc_mask;
	int ret, i;

	ad9081_iiochan_to_fddc_cddc(phy, chan, &fddc_num,
		&fddc_mask, &cddc_num, &cddc_mask);

	mutex_lock(&conv->lock);
	if (chan->output) {
		ret = adi_ad9081_dac_duc_main_nco_hopf_mode_set(&phy->ad9081,
						  cddc_mask, item);
		if (!ret)
			for_each_cddc(i, cddc_mask)
				phy->tx_ffh_hopf_mode[i] = item;

	} else {
		ret = adi_ad9081_adc_ddc_coarse_nco_channel_update_mode_set(&phy->ad9081,
			cddc_mask, item > 0);

		if (item > 0)
			ret = adi_ad9081_adc_ddc_coarse_gpio_chip_xfer_mode_set(&phy->ad9081,
				cddc_mask, item - 1);

		if (!ret)
			phy->rx_main_ffh_mode[cddc_num] = item;
	}
	mutex_unlock(&conv->lock);

	return ret;
}

static const char *const ad9081_dac_main_ffh_modes[] = {
	"phase_continuous",
	"phase_incontinuous",
	"phase_coherent",
};

static const struct iio_enum ad9081_dac_main_ffh_mode_enum = {
	.items = ad9081_dac_main_ffh_modes,
	.num_items = ARRAY_SIZE(ad9081_dac_main_ffh_modes),
	.set = ad9081_main_ffh_mode_write,
	.get = ad9081_main_ffh_mode_read,
};

static const char *const ad9081_adc_main_ffh_modes[] = {
	"instantaneous_update",
	"synchronous_update_by_transfer_bit",
	"synchronous_update_by_gpio",
};

static const struct iio_enum ad9081_adc_main_ffh_mode_enum = {
	.items = ad9081_adc_main_ffh_modes,
	.num_items = ARRAY_SIZE(ad9081_adc_main_ffh_modes),
	.set = ad9081_main_ffh_mode_write,
	.get = ad9081_main_ffh_mode_read,
};

static ssize_t ad9081_ext_info_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	long long val;
	u64 range;
	u8 cddc_num, cddc_mask, fddc_num, fddc_mask;
	int i, ret = -EINVAL;

	mutex_lock(&conv->lock);

	ad9081_iiochan_to_fddc_cddc(phy, chan, &fddc_num,
		&fddc_mask, &cddc_num, &cddc_mask);

	switch (private) {
	case CDDC_NCO_FREQ:
		if (chan->output) {
			for_each_cddc(i, cddc_mask) {
				val = phy->tx_main_shift[i];
				ret = 0;
				break;
			}
		} else {
			val = phy->rx_cddc_shift[phy->rx_main_ffh_index[cddc_num]][cddc_num];
			ret = 0;
		}
		break;
	case FDDC_NCO_FREQ:
		if (chan->output) {
			val = phy->tx_chan_shift[fddc_num];
			ret = 0;
		} else {
			val = phy->rx_fddc_shift[fddc_num];
			ret = 0;
		}
		break;
	case CDDC_NCO_FREQ_AVAIL:
		if (chan->output) {
			if (phy->tx_main_interp == 1)
				range = 0; /* full bw mode */
			else
				range = DIV_ROUND_CLOSEST_ULL(phy->dac_frequency_hz, 2);

		} else {
			if (phy->adc_dcm[0] == 1)
				range = 0; /* full bw mode */
			else
				range = DIV_ROUND_CLOSEST_ULL(phy->adc_frequency_hz, 2);

		}

		mutex_unlock(&conv->lock);
		return sprintf(buf, "[%lld 1 %lld]\n", -1 * range, range);
	case FDDC_NCO_FREQ_AVAIL:
		if (chan->output) {
			if (phy->tx_chan_interp == 1)
				range = 0; /* full bw mode */
			else
				range = DIV_ROUND_CLOSEST_ULL(phy->dac_frequency_hz,
					phy->tx_main_interp * 2);

		} else {
			if (phy->adc_dcm[0] == 1 || phy->adc_chan_decimation[fddc_num] == 1)
				range = 0; /* full bw mode */
			else
				range = DIV_ROUND_CLOSEST_ULL(phy->adc_frequency_hz,
					phy->adc_main_decimation[cddc_num] * 2);

		}

		mutex_unlock(&conv->lock);
		return sprintf(buf, "[%lld 1 %lld]\n", -1 * range, range);
	case CDDC_NCO_PHASE:
		if (chan->output) {
			for_each_cddc(i, cddc_mask) {
				val = phy->dac_cache.main_phase[i];
				ret = 0;
				break;
			}
		} else {
			val = phy->rx_cddc_phase[phy->rx_main_ffh_index[cddc_num]][cddc_num];
			ret = 0;
		}
		break;
	case FDDC_NCO_PHASE:
		if (chan->output) {
			val = phy->dac_cache.chan_phase[fddc_num];
			ret = 0;
		} else {
			val = phy->rx_fddc_phase[fddc_num];
			ret = 0;
		}
		break;
	case FDDC_NCO_GAIN:
		val = phy->dac_cache.chan_gain[fddc_num];
		mutex_unlock(&conv->lock);
		return ad9081_iio_val_to_str(buf, 0xFFF, val);
	case CDDC_6DB_GAIN:
		val = phy->rx_cddc_gain_6db_en[cddc_num];
		ret = 0;
		break;
	case FDDC_6DB_GAIN:
		val = phy->rx_fddc_gain_6db_en[fddc_num];
		ret = 0;
		break;
	case DAC_MAIN_TEST_TONE_EN:
		val = phy->dac_cache.main_test_tone_en[fddc_num];
		ret = 0;
		break;
	case DAC_CHAN_TEST_TONE_EN:
		val = phy->dac_cache.chan_test_tone_en[fddc_num];
		ret = 0;
		break;
	case DAC_MAIN_TEST_TONE_OFFSET:
		val = phy->dac_cache.main_test_tone_offset[fddc_num];
		mutex_unlock(&conv->lock);
		return ad9081_iio_val_to_str(buf, 0x7FFF, val);
	case DAC_CHAN_TEST_TONE_OFFSET:
		val = phy->dac_cache.chan_test_tone_offset[fddc_num];
		mutex_unlock(&conv->lock);
		return ad9081_iio_val_to_str(buf, 0x7FFF, val);
	case TRX_CONVERTER_RATE:
		if (chan->output)
			val = phy->ad9081.dev_info.dac_freq_hz;
		else
			val = phy->ad9081.dev_info.adc_freq_hz;

		ret = 0;
		break;
	case CDDC_FFH_HOPF_SET:
		if (conv->id == CHIPID_AD9988 || conv->id == CHIPID_AD9986) {
			ret = -EOPNOTSUPP;
			break;
		}

		if (chan->output) {
			for_each_cddc(i, cddc_mask) {
				val = phy->tx_main_ffh_select[i];
				ret = 0;
				goto out_unlock;
			}
		} else {
			val = phy->rx_main_ffh_select[cddc_num];
			ret = 0;
		}
		break;
	case CDDC_FFH_INDEX_SET:
		if (conv->id == CHIPID_AD9988 || conv->id == CHIPID_AD9986) {
			ret = -EOPNOTSUPP;
			break;
		}

		if (chan->output)
			val = phy->tx_ffh_hopf_index[cddc_num];
		else
			val = phy->rx_main_ffh_index[cddc_num];

		ret = 0;
		break;
	case ADC_CDDC_FFH_TRIG_HOP_EN:
	case ADC_FFH_GPIO_MODE_SET:
	case DAC_FFH_GPIO_MODE_SET:
		if (conv->id == CHIPID_AD9988 || conv->id == CHIPID_AD9986) {
			ret = -EOPNOTSUPP;
			break;
		}

		if (chan->output) {
			val = phy->tx_ffh_hopf_via_gpio_en;
		} else {
			if (private == ADC_FFH_GPIO_MODE_SET)
				val = phy->rx_main_ffh_gpio_en[cddc_num];
			else
				val = phy->rx_main_ffh_trig_en[cddc_num];
		}
		ret = 0;
		break;
	case DAC_FFH_FREQ_SET:
		if (conv->id == CHIPID_AD9988 || conv->id == CHIPID_AD9986) {
			ret = -EOPNOTSUPP;
			break;
		}

		if (phy->tx_disable) {
			ret = -ENODEV;
			break;
		}
		ret = sysfs_emit(buf, "%lld\n",
			phy->tx_ffh_hopf_vals[phy->tx_ffh_hopf_index[cddc_num]][cddc_num]);
		break;
	default:
		ret = -EINVAL;
	}

out_unlock:
	mutex_unlock(&conv->lock);

	if (ret == 0)
		ret = sprintf(buf, "%lld\n", val);

	return ret;
}

static ssize_t ad9081_ext_info_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	long long readin;
	bool enable;
	int i, ret, readin_32;
	u8 cddc_num, cddc_mask, fddc_num, fddc_mask;
	s16 val16;
	s64 val64;
	u64 ftw;

	mutex_lock(&conv->lock);

	ad9081_iiochan_to_fddc_cddc(phy, chan, &fddc_num,
		&fddc_mask, &cddc_num, &cddc_mask);

	switch (private) {
	case CDDC_NCO_FREQ:
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			goto out;

		if (chan->output) {
			/* set main nco */
			for_each_cddc(i, cddc_mask) {
				ret = adi_ad9081_dac_duc_nco_set(
					&phy->ad9081, BIT(i),
					AD9081_DAC_CH_NONE, readin);
				if (!ret)
					phy->tx_main_shift[i] = readin;
				else
					ret = -EFAULT;
			}
		} else {
			ret = adi_ad9081_adc_ddc_coarse_nco_set(&phy->ad9081,
								cddc_mask, readin);
			if (!ret)
				phy->rx_cddc_shift[phy->rx_main_ffh_index[cddc_num]][cddc_num] = readin;
			else
				ret = -EFAULT;
		}
		break;
	case FDDC_NCO_FREQ:
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			goto out;

		if (chan->output) {
			ret = adi_ad9081_dac_duc_nco_set(&phy->ad9081,
							 AD9081_DAC_NONE,
							 fddc_mask,
							 readin);
			if (!ret)
				phy->tx_chan_shift[fddc_num] = readin;
			else
				ret = -EFAULT;

		} else {
			ret = adi_ad9081_adc_ddc_fine_nco_set(
				&phy->ad9081, fddc_mask, readin);
			if (!ret)
				phy->rx_fddc_shift[fddc_num] = readin;
			else
				ret = -EFAULT;
		}
		break;
	case CDDC_NCO_PHASE:
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			goto out;

		readin = clamp_t(long long, readin, -180000, 180000);

		if (chan->output) {
			val16 = div_s64(readin * 32768, 180000LL);

			/* set main nco */
			for_each_cddc(i, cddc_mask) {
				ret = adi_ad9081_dac_duc_nco_phase_offset_set(
					&phy->ad9081, BIT(i), val16,
					AD9081_DAC_CH_NONE, 0);
				if (!ret)
					phy->dac_cache.main_phase[i] =
						readin;
				else
					ret = -EFAULT;
			}
		} else {
			val64 = div_s64(readin * 14073748835533, 18000LL);

			ret =  adi_ad9081_adc_ddc_coarse_nco_phase_offset_set(
				&phy->ad9081, cddc_mask, val64);
			if (!ret)
				phy->rx_cddc_phase[phy->rx_main_ffh_index[cddc_num]][cddc_num] = readin;
			else
				ret = -EFAULT;
		}
		break;
	case FDDC_NCO_PHASE:
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			goto out;

		readin = clamp_t(long long, readin, -180000, 180000);

		if (chan->output) {
			val16 = div_s64(readin * 32768, 180000LL);
			ret = adi_ad9081_dac_duc_nco_phase_offset_set(
					&phy->ad9081,
					AD9081_DAC_NONE, 0,
					fddc_mask,
					val16);
			if (!ret)
				phy->dac_cache.chan_phase[fddc_num] =
					readin;
			else
				ret = -EFAULT;
		} else {
			val64 = div_s64(readin * 14073748835533, 18000LL);

			ret = adi_ad9081_adc_ddc_fine_nco_phase_offset_set(
				&phy->ad9081, fddc_mask, val64);
			if (!ret)
				phy->rx_fddc_phase[fddc_num] = readin;
			else
				ret = -EFAULT;
		}
		break;
	case FDDC_NCO_GAIN:
		ret = ad9081_iio_str_to_val(buf, 0, 0xFFF, &readin_32);
		if (ret)
			goto out;
		ret = adi_ad9081_dac_duc_nco_gain_set(
			&phy->ad9081, fddc_mask, readin_32);
		if (!ret)
			phy->dac_cache.chan_gain[fddc_num] = readin_32;
		break;
	case CDDC_6DB_GAIN:
		ret = strtobool(buf, &enable);
		if (ret)
			goto out;

		ret = adi_ad9081_adc_ddc_coarse_gain_set(
			&phy->ad9081, cddc_mask, enable);
		if (ret)
			goto out;
		phy->rx_cddc_gain_6db_en[cddc_num] = enable;
		ret = 0;
		break;
	case FDDC_6DB_GAIN:
		ret = strtobool(buf, &enable);
		if (ret)
			goto out;
		ret = adi_ad9081_adc_ddc_fine_gain_set(
			&phy->ad9081, fddc_mask, enable);
		if (ret)
			goto out;
		phy->rx_fddc_gain_6db_en[fddc_num] = enable;
		ret = 0;
		break;
	case DAC_MAIN_TEST_TONE_EN:
		ret = strtobool(buf, &enable);
		if (ret)
			goto out;

		ret = adi_ad9081_dac_duc_main_dc_test_tone_en_set(&phy->ad9081,
								  cddc_mask, enable);
		if (!ret)
			phy->dac_cache.main_test_tone_en[fddc_num] = enable;
		break;
	case DAC_CHAN_TEST_TONE_EN:
		ret = strtobool(buf, &enable);
		if (ret)
			goto out;

		ret = adi_ad9081_dac_dc_test_tone_en_set(
			&phy->ad9081, fddc_mask, enable);
		if (!ret)
			phy->dac_cache.chan_test_tone_en[fddc_num] = enable;
		break;
	case DAC_MAIN_TEST_TONE_OFFSET:
		ret = ad9081_iio_str_to_val(buf, 0, 0x7FFF, &readin_32);
		if (ret)
			goto out;

		ret = adi_ad9081_dac_duc_main_dc_test_tone_offset_set(
			&phy->ad9081, cddc_mask, readin_32);
		if (!ret)
			phy->dac_cache.main_test_tone_offset[fddc_num] = readin_32;

		break;
	case DAC_CHAN_TEST_TONE_OFFSET:
		ret = ad9081_iio_str_to_val(buf, 0, 0x7FFF, &readin_32);
		if (ret)
			goto out;

		ret = adi_ad9081_dac_dc_test_tone_offset_set(
			&phy->ad9081, fddc_mask, readin_32);
		if (!ret)
			phy->dac_cache.chan_test_tone_offset[fddc_num] = readin_32;
		break;

	case CDDC_FFH_HOPF_SET:
		if (conv->id == CHIPID_AD9988 || conv->id == CHIPID_AD9986) {
			ret = -EOPNOTSUPP;
			break;
		}
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			goto out;

		if (chan->output) {
			if (readin < 0 || readin >= MAX_NUM_TX_NCO_CHAN_REGS) {
				ret = -EINVAL;
				goto out;
			}
			/* set main nco */
			for_each_cddc(i, cddc_mask) {
				ret = adi_ad9081_dac_duc_main_nco_hopf_select_set(&phy->ad9081,
					BIT(i), readin + 1);
				if (!ret)
					phy->tx_main_ffh_select[i] = readin;
				else
					ret = -EFAULT;
			}
		} else {
			if (readin < 0 || readin >= MAX_NUM_RX_NCO_CHAN_REGS) {
				ret = -EINVAL;
				goto out;
			}
			ret = adi_ad9081_adc_ddc_coarse_nco_channel_selection_set(&phy->ad9081,
				cddc_mask, readin);
			if (!ret)
				phy->rx_main_ffh_select[cddc_num] = readin;
			else
				ret = -EFAULT;
		}
		break;
	case CDDC_FFH_INDEX_SET:
		if (conv->id == CHIPID_AD9988 || conv->id == CHIPID_AD9986) {
			ret = -EOPNOTSUPP;
			break;
		}
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			goto out;

		if (chan->output) {
			readin = clamp_t(long long, readin, 0, 30);
			if (readin < 0 || readin >= MAX_NUM_TX_NCO_CHAN_REGS) {
				ret = -EINVAL;
				goto out;
			}
			for_each_cddc(i, cddc_mask)
				phy->tx_ffh_hopf_index[i] = readin;
		} else {
			if (readin < 0 || readin >= MAX_NUM_RX_NCO_CHAN_REGS) {
				ret = -EINVAL;
				goto out;
			}
			ret = adi_ad9081_adc_ddc_coarse_nco_channel_update_index_set(&phy->ad9081,
				cddc_mask, readin);
			if (!ret)
				phy->rx_main_ffh_index[cddc_num] = readin;
			else
				ret = -EFAULT;
		}
		break;
	case ADC_CDDC_FFH_TRIG_HOP_EN:
	case ADC_FFH_GPIO_MODE_SET:
	case DAC_FFH_GPIO_MODE_SET:
		if (conv->id == CHIPID_AD9988 || conv->id == CHIPID_AD9986) {
			ret = -EOPNOTSUPP;
			break;
		}
		ret = strtobool(buf, &enable);
		if (ret)
			goto out;

		if (chan->output) {
			ret = adi_ad9081_dac_duc_main_nco_hopf_gpio_as_hop_en_set(&phy->ad9081,
				enable);
			if (!ret)
				phy->tx_ffh_hopf_via_gpio_en = enable;

			adi_ad9081_jesd_rx_syncb_mode_set(&phy->ad9081, 0);
			adi_ad9081_jesd_rx_syncb_driver_powerdown_set(&phy->ad9081,
				!enable);
		} else {
			if (private == ADC_FFH_GPIO_MODE_SET) {
				ret = adi_ad9081_adc_ddc_coarse_nco_channel_select_via_gpio_set(&phy->ad9081,
					cddc_mask, enable ? phy->rx_cddc_nco_channel_select_mode[cddc_num] : 0);
				if (!ret)
					phy->rx_main_ffh_gpio_en[cddc_num] = enable;
			} else {
				ret = adi_ad9081_adc_ddc_coarse_trig_hop_en_set(&phy->ad9081,
					cddc_mask, enable);
				if (!ret)
					phy->rx_main_ffh_trig_en[cddc_num] = enable;
			}
		}
		break;
	case DAC_FFH_FREQ_SET:
		if (conv->id == CHIPID_AD9988 || conv->id == CHIPID_AD9986) {
			ret = -EOPNOTSUPP;
			break;
		}

		if (phy->tx_disable) {
			ret = -ENODEV;
			break;
		}

		ret = kstrtoll(buf, 10, &readin);
		if (ret) {
			ret = -EINVAL;
			break;
		}
		ret = adi_ad9081_hal_calc_tx_nco_ftw32(&phy->ad9081,
				phy->ad9081.dev_info.dac_freq_hz, readin,
				&ftw);
		if (ret) {
			ret = -EINVAL;
			break;
		}

		ret = adi_ad9081_dac_duc_main_nco_hopf_ftw_set(&phy->ad9081,
						 cddc_mask,
						 phy->tx_ffh_hopf_index[cddc_num] + 1,
						 ftw);
		if (ret) {
			ret = -EINVAL;
			break;
		}
		for_each_cddc(i, cddc_mask)
			phy->tx_ffh_hopf_vals[phy->tx_ffh_hopf_index[cddc_num]][i] = readin;
		break;
	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&conv->lock);

	return ret ? ret : len;
}

static struct iio_chan_spec_ext_info rxadc_ext_info[] = {
	IIO_ENUM("test_mode", IIO_SHARED_BY_TYPE, &ad9081_testmode_enum),
	IIO_ENUM_AVAILABLE("test_mode", IIO_SHARED_BY_TYPE, &ad9081_testmode_enum),
	IIO_ENUM("nyquist_zone", IIO_SEPARATE, &ad9081_nyquist_zone_enum),
	IIO_ENUM_AVAILABLE("nyquist_zone", IIO_SHARED_BY_TYPE, &ad9081_nyquist_zone_enum),
	IIO_ENUM("main_ffh_mode", IIO_SEPARATE, &ad9081_adc_main_ffh_mode_enum),
	IIO_ENUM_AVAILABLE("main_ffh_mode", IIO_SHARED_BY_TYPE, &ad9081_adc_main_ffh_mode_enum),
	{
		.name = "main_nco_frequency",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = CDDC_NCO_FREQ,
	},
	{
		.name = "main_nco_frequency_available",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = true,
		.private = CDDC_NCO_FREQ_AVAIL,
	},
	{
		.name = "channel_nco_frequency",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = FDDC_NCO_FREQ,
	},
	{
		.name = "channel_nco_frequency_available",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = FDDC_NCO_FREQ_AVAIL,
	},
	{
		.name = "main_nco_phase",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = CDDC_NCO_PHASE,
	},
	{
		.name = "channel_nco_phase",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = FDDC_NCO_PHASE,
	},
	{
		.name = "main_6db_digital_gain_en",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = CDDC_6DB_GAIN,
	},
	{
		.name = "channel_6db_digital_gain_en",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = FDDC_6DB_GAIN,
	},
	{
		.name = "adc_frequency",
		.read = ad9081_ext_info_read,
		.shared = true,
		.private = TRX_CONVERTER_RATE,
	},
	{
		.name = "main_nco_ffh_select",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = CDDC_FFH_HOPF_SET,
	},
	{
		.name = "main_nco_ffh_index",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = CDDC_FFH_INDEX_SET,
	},
	{
		.name = "main_ffh_trig_hop_en",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = ADC_CDDC_FFH_TRIG_HOP_EN,
	},
	{
		.name = "main_ffh_gpio_mode_en",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = ADC_FFH_GPIO_MODE_SET,
	},
	{},
};

static struct iio_chan_spec_ext_info txdac_ext_info[] = {
	IIO_ENUM("main_ffh_mode", IIO_SEPARATE, &ad9081_dac_main_ffh_mode_enum),
	IIO_ENUM_AVAILABLE("main_ffh_mode", IIO_SHARED_BY_TYPE, &ad9081_dac_main_ffh_mode_enum),
	{
		.name = "main_nco_frequency",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = CDDC_NCO_FREQ,
	},
	{
		.name = "main_nco_frequency_available",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = true,
		.private = CDDC_NCO_FREQ_AVAIL,
	},
	{
		.name = "channel_nco_frequency",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = FDDC_NCO_FREQ,
	},
	{
		.name = "channel_nco_frequency_available",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = true,
		.private = FDDC_NCO_FREQ_AVAIL,
	},
	{
		.name = "main_nco_phase",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = CDDC_NCO_PHASE,
	},
	{
		.name = "channel_nco_phase",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = FDDC_NCO_PHASE,
	},
	{
		.name = "channel_nco_gain_scale",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = FDDC_NCO_GAIN,
	},
	{
		.name = "main_nco_test_tone_en",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = DAC_MAIN_TEST_TONE_EN,
	},
	{
		.name = "channel_nco_test_tone_en",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = DAC_CHAN_TEST_TONE_EN,
	},
	{
		.name = "main_nco_test_tone_scale",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = DAC_MAIN_TEST_TONE_OFFSET,
	},
	{
		.name = "channel_nco_test_tone_scale",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = DAC_CHAN_TEST_TONE_OFFSET,
	},
	{
		.name = "dac_frequency",
		.read = ad9081_ext_info_read,
		.shared = true,
		.private = TRX_CONVERTER_RATE,
	},
	{
		.name = "main_nco_ffh_select",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = CDDC_FFH_HOPF_SET,
	},
	{
		.name = "main_nco_ffh_index",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = CDDC_FFH_INDEX_SET,
	},
	{
		.name = "main_ffh_gpio_mode_en",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = true,
		.private = DAC_FFH_GPIO_MODE_SET,
	},
	{
		.name = "main_nco_ffh_frequency",
		.read = ad9081_ext_info_read,
		.write = ad9081_ext_info_write,
		.shared = false,
		.private = DAC_FFH_FREQ_SET,
	},
	{},
};

static int ad9081_set_sample_rate(struct axiadc_converter *conv,
				  unsigned int sample_rate)
{
	return -ENOTSUPP;
}

static int ad9081_request_clks(struct axiadc_converter *conv)
{
	struct ad9081_phy *phy = conv->phy;
	int ret;

	phy->dev_clk = devm_clk_get(&conv->spi->dev, "dev_clk");
	if (IS_ERR(phy->dev_clk))
		return PTR_ERR(phy->dev_clk);

	ret = clk_prepare_enable(phy->dev_clk);
	if (ret)
		return ret;

	return 0;
}

static int ad9081_main_decimation_to_val(u32 decim)
{
	switch (decim) {
	case 1:
		return AD9081_CDDC_DCM_1;
	case 2:
		return AD9081_CDDC_DCM_2;
	case 3:
		return AD9081_CDDC_DCM_3;
	case 4:
		return AD9081_CDDC_DCM_4;
	case 6:
		return AD9081_CDDC_DCM_6;
	case 8:
		return AD9081_CDDC_DCM_8;
	case 9:
		return AD9081_CDDC_DCM_9;
	case 12:
		return AD9081_CDDC_DCM_12;
	case 16:
		return AD9081_CDDC_DCM_16;
	case 18:
		return AD9081_CDDC_DCM_18;
	case 24:
		return AD9081_CDDC_DCM_24;
	case 36:
		return AD9081_CDDC_DCM_36;
	default:
		return -EINVAL;
	}
}

static int ad9081_chan_decimation_to_val(u32 decim)
{
	switch (decim) {
	case 1:
		return AD9081_FDDC_DCM_1;
	case 2:
		return AD9081_FDDC_DCM_2;
	case 3:
		return AD9081_FDDC_DCM_3;
	case 4:
		return AD9081_FDDC_DCM_4;
	case 6:
		return AD9081_FDDC_DCM_6;
	case 8:
		return AD9081_FDDC_DCM_8;
	case 12:
		return AD9081_FDDC_DCM_12;
	case 16:
		return AD9081_FDDC_DCM_16;
	case 24:
		return AD9081_FDDC_DCM_24;
	default:
		return -EINVAL;
	}
}

static const char *const ad9081_jtx_qbf_states[] = {
	"CGS", "ILA_M0R", "ILA_M0", "ILA_M1R", "ILA_M1C1", "ILA_M1C2",
	"ILA_M1C3", "ILA_M1", "ILA_M2R", "ILA_M2", "ILA_M3R", "ILA_M3",
	"ILA_BP", "DATA"
};

int ad9081_jesd_tx_link_status_print(struct ad9081_phy *phy,
				     struct jesd204_link *lnk, int retry)
{
	int ret, l;
	u16 stat;

	switch (lnk->link_id) {
	case FRAMER_LINK0_RX:
		l = AD9081_LINK_0;
		break;
	case FRAMER_LINK1_RX:
		l = AD9081_LINK_1;
		break;
	default:
		return -EINVAL;
	}

	do {
		ret = adi_ad9081_jesd_tx_link_status_get(
			&phy->ad9081, l, &stat);
		if (ret)
			return -EFAULT;

		if (lnk->jesd_version == JESD204_VERSION_C) {
			if ((stat & 0x60) == 0x60)
				ret = 0;
			else
				ret = -EIO;

			if (ret == 0 || retry == 0)
				dev_info(&phy->spi->dev,
					"JESD RX (JTX) Link%d PLL %s, PHASE %s, MODE %s\n",
					lnk->link_id,
					stat & BIT(5) ? "locked" : "unlocked",
					stat & BIT(6) ? "established" : "lost",
					stat & BIT(7) ? "invalid" : "valid");
			else
				msleep(20);
		} else {
			if ((stat & 0xFF) == 0x7D)
				ret = 0;
			else
				ret = -EIO;

			if (ret == 0 || retry == 0)
				dev_info(&phy->spi->dev,
					"JESD RX (JTX) Link%d in %s, SYNC %s, PLL %s, PHASE %s, MODE %s\n",
					lnk->link_id, ad9081_jtx_qbf_states[stat & 0xF],
					stat & BIT(4) ? "deasserted" : "asserted",
					stat & BIT(5) ? "locked" : "unlocked",
					stat & BIT(6) ? "established" : "lost",
					stat & BIT(7) ? "invalid" : "valid");
			else
				msleep(20);
		}
	} while (ret && retry--);

	return ret;
}

static const char *const ad9081_jrx_204c_states[] = {
	"Reset", "Undef", "Sync header alignment done",
	"Extended multiblock sync complete",
	"Extended multiblock alignment complete",
	"Undef", "Link is good", "Undef",
};

static int ad9081_jesd_rx_link_status_print(struct ad9081_phy *phy,
				     struct jesd204_link *lnk, int retry)
{
	int ret, l;
	u16 stat, mask;

	switch (lnk->link_id) {
	case DEFRAMER_LINK0_TX:
		l = AD9081_LINK_0;
		break;
	case DEFRAMER_LINK1_TX:
		l = AD9081_LINK_1;
		break;
	default:
		return -EINVAL;
	}

	do {
		ret = adi_ad9081_jesd_rx_link_status_get(&phy->ad9081, l, &stat);
		if (ret)
			return -EFAULT;

		if (lnk->jesd_version == JESD204_VERSION_C) {
			stat >>= 8;
			if (stat == 6)
				ret = 0;
			else
				ret = -EIO;

			if (ret == 0 || retry == 0)
				dev_info(&phy->spi->dev,
					"JESD TX (JRX) Link%d 204C status: %s (%d)\n",
					lnk->link_id, ad9081_jrx_204c_states[stat & 0x7],
					stat);
			else
				msleep(20);
		} else {
			mask = (1 << lnk->num_lanes) - 1;

			stat = mask & stat;

			if (stat == mask)
				ret = 0;
			else
				ret = -EIO;

			if (ret == 0 || retry == 0)
				dev_info(&phy->spi->dev,
					"JESD TX (JRX) Link%d 0x%X lanes in DATA\n",
					lnk->link_id, stat);
			else
				msleep(20);
		}
	} while (ret && retry--);

	return ret;
}

static void ad9081_convert_link_converter_select(
	adi_ad9081_jtx_conv_sel_t *jesd_conv_sel, u8 *vals)
{
	jesd_conv_sel->virtual_converter0_index = *vals++;
	jesd_conv_sel->virtual_converter1_index = *vals++;
	jesd_conv_sel->virtual_converter2_index = *vals++;
	jesd_conv_sel->virtual_converter3_index = *vals++;
	jesd_conv_sel->virtual_converter4_index = *vals++;
	jesd_conv_sel->virtual_converter5_index = *vals++;
	jesd_conv_sel->virtual_converter6_index = *vals++;
	jesd_conv_sel->virtual_converter7_index = *vals++;
	jesd_conv_sel->virtual_converter8_index = *vals++;
	jesd_conv_sel->virtual_converter9_index = *vals++;
	jesd_conv_sel->virtual_convertera_index = *vals++;
	jesd_conv_sel->virtual_converterb_index = *vals++;
	jesd_conv_sel->virtual_converterc_index = *vals++;
	jesd_conv_sel->virtual_converterd_index = *vals++;
	jesd_conv_sel->virtual_convertere_index = *vals++;
	jesd_conv_sel->virtual_converterf_index = *vals++;
}

static int ad9081_setup_tx(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct ad9081_phy *phy = conv->phy;
	u64 sample_rate, status64;
	int ret, i;

	if (phy->tx_disable) {
		/* Disable DAC core clock domain and reduce power consumption */
		adi_ad9081_dac_tx_enable_set(&phy->ad9081, AD9081_DAC_ALL, 0);
		adi_ad9081_hal_reg_set(&phy->ad9081,
			REG_ENABLE_TIMING_CTRL_DAC0_ADDR, 0x0);
		adi_ad9081_hal_reg_set(&phy->ad9081,
			REG_ENABLE_TIMING_CTRL_DAC1_ADDR, 0x0);

		return 0;
	}

	memcpy(phy->ad9081.serdes_info.des_settings.lane_mapping[0],
		phy->jrx_link_tx[0].logiclane_mapping,
		sizeof(phy->jrx_link_tx[0].logiclane_mapping));

	memcpy(phy->ad9081.serdes_info.des_settings.lane_mapping[1],
		phy->jrx_link_tx[1].logiclane_mapping,
		sizeof(phy->jrx_link_tx[1].logiclane_mapping));

	/* start txfe tx */
	ret = adi_ad9081_device_startup_tx(
		&phy->ad9081, phy->tx_main_interp, phy->tx_chan_interp,
		phy->tx_chan_interp == 1 ? phy->tx_dac_chan_xbar_1x_non1x : phy->tx_dac_chan_xbar,
		phy->tx_main_shift, phy->tx_chan_shift,
		&phy->jrx_link_tx[0].jesd_param);

	if (ret != 0)
		return ret;

	/* setup txfe dac channel gain */
	ret = adi_ad9081_dac_duc_nco_gains_set(&phy->ad9081,
					       phy->dac_cache.chan_gain);
	if (ret != 0)
		return ret;

	adi_ad9081_jesd_rx_lmfc_delay_set(&phy->ad9081, AD9081_LINK_0,
		phy->jrx_link_tx[0].jrx_tpl_phase_adjust);

	adi_ad9081_jesd_rx_lmfc_delay_set(&phy->ad9081, AD9081_LINK_1,
		phy->jrx_link_tx[1].jrx_tpl_phase_adjust);

	if (phy->jrx_link_tx[0].jesd_param.jesd_jesdv == 2  &&
		phy->ad9081.dev_info.dev_rev < 3) {
		ret = adi_ad9081_hal_bf_set(&phy->ad9081, REG_JRX_TPL_1_ADDR,
			BF_JRX_TPL_BUF_PROTECT_EN_INFO,
			0);
		if (ret != 0)
			return ret;
	}

	adi_ad9081_dac_irqs_status_get(&phy->ad9081, &status64);
	dev_dbg(&spi->dev, "DAC IRQ status 0x%llX\n", status64);

	sample_rate = DIV_ROUND_CLOSEST_ULL(phy->dac_frequency_hz,
			phy->tx_main_interp * phy->tx_chan_interp);
	clk_set_rate_scaled(phy->clks[TX_SAMPL_CLK], sample_rate,
		&phy->clkscale[TX_SAMPL_CLK]);

	for (i = 0; i < ARRAY_SIZE(phy->tx_dac_fsc); i++) {
		if (phy->tx_dac_fsc[i]) {
			ret = adi_ad9081_dac_fsc_set(&phy->ad9081, BIT(i), phy->tx_dac_fsc[i], 1);
			if (ret != 0)
				return ret;
		}
	}

	if (phy->tx_ffh_hopf_via_gpio_en) {
		adi_ad9081_jesd_rx_syncb_mode_set(&phy->ad9081, 0);
		adi_ad9081_dac_duc_main_nco_hopf_gpio_as_hop_en_set(&phy->ad9081, 1);
		adi_ad9081_jesd_rx_syncb_driver_powerdown_set(&phy->ad9081, 0);
	}

	return 0;
}

static int ad9081_setup_rx(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct ad9081_phy *phy = conv->phy;
	u64 sample_rate;
	adi_cms_jesd_param_t jesd_param[2];
	adi_ad9081_jtx_conv_sel_t jesd_conv_sel[2];
	u8 dcm;
	int ret, i;

	if (phy->rx_disable) {
		adi_ad9081_adc_clk_enable_set(&phy->ad9081, 0);

		return 0;
	}

	memcpy(phy->ad9081.serdes_info.ser_settings.lane_mapping[0],
		phy->jtx_link_rx[0].logiclane_mapping,
		sizeof(phy->jtx_link_rx[0].logiclane_mapping));

	if (ad9081_link_is_dual(phy->jtx_link_rx))
		memcpy(phy->ad9081.serdes_info.ser_settings.lane_mapping[1],
			phy->jtx_link_rx[1].logiclane_mapping,
			sizeof(phy->jtx_link_rx[1].logiclane_mapping));

	for (i = 0; i < ARRAY_SIZE(phy->adc_main_decimation); i++) {
		ret = ad9081_main_decimation_to_val(
			phy->adc_main_decimation[i]);
		if (ret >= 0)
			phy->rx_cddc_dcm[i] = ret;
	}

	for (i = 0; i < ARRAY_SIZE(phy->adc_chan_decimation); i++) {
		ret = ad9081_chan_decimation_to_val(
			phy->adc_chan_decimation[i]);
		if (ret >= 0)
			phy->rx_fddc_dcm[i] = ret;
	}

	/* FIXME - the API should change here */
	ad9081_convert_link_converter_select(&jesd_conv_sel[0],
		phy->jtx_link_rx[0].link_converter_select);
	ad9081_convert_link_converter_select(&jesd_conv_sel[1],
		phy->jtx_link_rx[1].link_converter_select);

	jesd_param[0] = phy->jtx_link_rx[0].jesd_param;
	jesd_param[1] = phy->jtx_link_rx[1].jesd_param;

	/* start txfe rx and set other settings for normal use cases */
	/* start txfe rx */
	ret = adi_ad9081_device_startup_rx(&phy->ad9081, phy->rx_cddc_select,
					   phy->rx_fddc_select,
					   phy->rx_cddc_shift[0],
					   phy->rx_fddc_shift, phy->rx_cddc_dcm,
					   phy->rx_fddc_dcm, phy->rx_cddc_c2r,
					   phy->rx_fddc_c2r, jesd_param,
					   jesd_conv_sel);
	if (ret != 0)
		return ret;

	if (conv->id == CHIPID_AD9081 || conv->id == CHIPID_AD9988) {
		/* Fix: 4x4 Crossbar Mux0 Mappings for AD9081 */
		ret  = adi_ad9081_adc_pfir_din_select_set(&phy->ad9081,
			AD9081_ADC_PFIR_ADC_PAIR0, 0, 1);
		if (ret != 0)
			return ret;

		ret  = adi_ad9081_adc_pfir_din_select_set(&phy->ad9081,
			AD9081_ADC_PFIR_ADC_PAIR1, 3, 0);
		if (ret != 0)
			return ret;
	}

	for_each_cddc(i, phy->rx_cddc_select) {
		ret = adi_ad9081_adc_data_inversion_dc_coupling_set(&phy->ad9081,
			BIT(i), phy->adc_invert_en[i]);
		if (ret != 0)
			return ret;
		ret = adi_ad9081_adc_ddc_coarse_gain_set(
			&phy->ad9081, BIT(i), phy->rx_cddc_gain_6db_en[i]);
		if (ret != 0)
			return ret;

		ret = adi_ad9081_adc_nyquist_zone_set(&phy->ad9081, BIT(i),
			phy->rx_nyquist_zone[i]);
		if (ret != 0)
			return ret;

		ret = adi_ad9081_adc_ddc_coarse_nco_channel_select_via_gpio_set(&phy->ad9081,
			BIT(i), phy->rx_cddc_nco_channel_select_mode[i]);
		if (ret != 0)
			return ret;
	}

	for_each_fddc(i, phy->rx_fddc_select) {
		ret = adi_ad9081_adc_ddc_fine_nco_mode_set(
				&phy->ad9081, BIT(i), phy->rx_fddc_mxr_if[i]);
		if (ret != 0)
			return ret;

		ret = adi_ad9081_adc_ddc_fine_gain_set(
			&phy->ad9081, BIT(i), phy->rx_fddc_gain_6db_en[i]);
		if (ret != 0)
			return ret;
	}

	/* setup txfe jtx converter mapping */
	for (i = 0; i < ARRAY_SIZE(phy->jtx_link_rx[0].link_converter_select);
	     i++) {
		ret = adi_ad9081_jesd_tx_conv_sel_set(&phy->ad9081,
			AD9081_LINK_0, i, phy->jtx_link_rx[0].link_converter_select[i]);
		if (ret != 0)
			return ret;
	}
	if (ad9081_link_is_dual(phy->jtx_link_rx)) {
		for (i = 0;
		     i < ARRAY_SIZE(phy->jtx_link_rx[1].link_converter_select);
		     i++) {
			ret = adi_ad9081_jesd_tx_conv_sel_set(
				&phy->ad9081, AD9081_LINK_1, i,
				phy->jtx_link_rx[1].link_converter_select[i]);
			if (ret != 0)
				return ret;
		}
	}

	ret = adi_ad9081_adc_chip_dcm_ratio_get(&phy->ad9081,
						AD9081_LINK_0, &dcm);
	if (ret != 0 || !dcm)
		return -EINVAL;

	phy->adc_dcm[0] = dcm;

	if (ad9081_link_is_dual(phy->jtx_link_rx)) {
		ret = adi_ad9081_adc_chip_dcm_ratio_get(&phy->ad9081,
							AD9081_LINK_1, &dcm);
		if (ret != 0 || !dcm)
			return -EINVAL;

		phy->adc_dcm[1] = dcm;
	}

	sample_rate = DIV_ROUND_CLOSEST_ULL(phy->adc_frequency_hz, phy->adc_dcm[0]);
	clk_set_rate_scaled(phy->clks[RX_SAMPL_CLK], sample_rate,
		&phy->clkscale[RX_SAMPL_CLK]);


	if (ad9081_link_is_dual(phy->jtx_link_rx)) {
		sample_rate = DIV_ROUND_CLOSEST_ULL(phy->adc_frequency_hz, phy->adc_dcm[1]);
		clk_set_rate_scaled(phy->clks[RX_SAMPL_CLK_LINK2], sample_rate,
			&phy->clkscale[RX_SAMPL_CLK_LINK2]);
	}

	for (i = 0; i < ARRAY_SIZE(phy->rx_ffh_gpio_mux_sel); i++)
		if (phy->rx_ffh_gpio_mux_sel[i] == AD9081_PERI_SEL_SYNCINB1_N ||
			phy->rx_ffh_gpio_mux_sel[i] == AD9081_PERI_SEL_SYNCINB1_P)
			adi_ad9081_hal_bf_set(&phy->ad9081, REG_SYNCB_CTRL_ADDR,
				BF_PD_SYNCB_RX_RC_INFO, 0);

	return adi_ad9081_adc_ddc_ffh_sel_to_gpio_mapping_set(&phy->ad9081,
		phy->rx_ffh_gpio_mux_sel);
}

static int ad9081_setup(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct ad9081_phy *phy = conv->phy;
	struct clock_scale devclk_clkscale;
	u64 dev_frequency_hz;
	u8 txfe_pll_stat;
	int ret;

	of_clk_get_scale(spi->dev.of_node, "dev_clk", &devclk_clkscale);
	dev_frequency_hz = clk_get_rate_scaled(phy->dev_clk, &devclk_clkscale);

	ret = adi_ad9081_hal_bf_set(&phy->ad9081, REG_SYNC_LMFC_DELAY_ADDR,
		BF_SYNC_LMFC_DELAY_SET_INFO,
		BF_SYNC_LMFC_DELAY_SET(phy->lmfc_delay));
	if (ret != 0)
		return ret;

	/* Configure SYSREF */
	ret = adi_ad9081_sync_sysref_input_config_set(&phy->ad9081,
		phy->sysref_coupling_ac_en ? COUPLING_AC : COUPLING_DC,
		phy->sysref_cmos_input_en ? SIGNAL_CMOS : SIGNAL_LVDS,
		phy->sysref_cmos_single_end_term_pos,
		phy->sysref_cmos_single_end_term_neg);
	if (ret != 0)
		return ret;

	ret = adi_ad9081_hal_bf_set(&phy->ad9081, REG_SYNC_DEBUG0_ADDR,
		BF_AVRG_FLOW_EN_INFO, 1);
	if (ret != 0)
		return ret;

	ret = adi_ad9081_hal_bf_set(&phy->ad9081, REG_SYSREF_AVERAGE_ADDR,
		BF_SYSREF_AVERAGE_INFO,
		BF_SYSREF_AVERAGE(phy->sysref_average_cnt_exp));
	if (ret != 0)
		return ret;

	ret = adi_ad9081_device_clk_config_set(&phy->ad9081,
		phy->dac_frequency_hz, phy->adc_frequency_hz, dev_frequency_hz);
	if (ret != 0)
		return ret;

	if (dev_frequency_hz != phy->dac_frequency_hz) {
		ret = adi_ad9081_device_clk_pll_lock_status_get(&phy->ad9081,
								&txfe_pll_stat);
		if (ret != 0)
			return ret;

		if (txfe_pll_stat != 3) {
			dev_err(&spi->dev, "CLK PLL Failed to Lock (Status: %d)",
				txfe_pll_stat);
			return -EFAULT;
		}
	}

	ret = ad9081_setup_tx(spi);
	if (ret)
		return ret;

	ret = ad9081_setup_rx(spi);
	if (ret)
		return ret;

	if (phy->config_sync_0a_cmos_en) {
		adi_ad9081_jesd_rx_synca_mode_set(&phy->ad9081, 0);
		adi_ad9081_hal_reg_set(&phy->ad9081, REG_SYNCA_CTRL_ADDR, 0x0);
	}

	if (phy->config_sync_01_swapped) {
		adi_ad9081_jesd_rx_syncb_driver_powerdown_set(&phy->ad9081, 0);
		adi_ad9081_hal_reg_set(&phy->ad9081,
			REG_GENERAL_JRX_CTRL_ADDR, 0x80);
		/* Differential mode */
		adi_ad9081_dac_gpio_as_sync1_out_set(&phy->ad9081, 1);
		adi_ad9081_jesd_tx_sync_mode_set(&phy->ad9081,
			AD9081_LINK_0, 1);

		adi_ad9081_hal_2bf_set(&phy->ad9081, REG_SYNCA_CTRL_ADDR,
					BF_PD_SYNCB_RX_RC_INFO, 1,
					BF_SYNCB_RX_MODE_RC_INFO, 1);

		adi_ad9081_hal_2bf_set(&phy->ad9081, REG_SYNCB_CTRL_ADDR,
					BF_PD_SYNCB_RX_RC_INFO, 0,
					BF_SYNCB_RX_MODE_RC_INFO, 1);
	}

	return 0;
}

static int ad9081_multichip_sync(struct ad9081_phy *phy, int step)
{
	dev_dbg(&phy->spi->dev, "%s:%d\n", __func__, step);

	switch (step & 0xFF) {
	case 8:
		jesd204_fsm_stop(phy->jdev, JESD204_LINKS_ALL);
		jesd204_fsm_clear_errors(phy->jdev, JESD204_LINKS_ALL);
		break;
	case 9:
		return jesd204_fsm_start(phy->jdev, JESD204_LINKS_ALL);
	case 10:
		jesd204_fsm_stop(phy->jdev, JESD204_LINKS_ALL);
		jesd204_fsm_clear_errors(phy->jdev, JESD204_LINKS_ALL);
		return jesd204_fsm_start(phy->jdev, JESD204_LINKS_ALL);
	case 20:
		return adi_ad9081_jesd_rx_calibrate_204c(&phy->ad9081, 1, 0, 1);
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9081_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan, int *val,
			   int *val2, long info)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	u8 msb, lsb, dir;
	u8 cddc_num, cddc_mask, fddc_num, fddc_mask;
	u64 freq;

	ad9081_iiochan_to_fddc_cddc(phy, chan, &fddc_num,
		&fddc_mask, &cddc_num, &cddc_mask);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		dir = chan->output ? TX_SAMPL_CLK : RX_SAMPL_CLK;

		if (!phy->clks[dir])
			return -ENODEV;

		freq = clk_get_rate_scaled(phy->clks[dir], &phy->clkscale[dir]);

		*val = lower_32_bits(freq);
		*val2 = upper_32_bits(freq);
		return IIO_VAL_INT_64;
	case IIO_CHAN_INFO_ENABLE:
		if (chan->output) {
			*val = phy->dac_cache.enable[fddc_num];
			return IIO_VAL_INT;
		}
		break;
	case IIO_CHAN_INFO_PROCESSED:
		adi_ad9081_hal_reg_get(&phy->ad9081, 0x2108, &msb);
		adi_ad9081_hal_reg_get(&phy->ad9081, 0x2107, &lsb);

		*val = ((s16)(msb << 8 | lsb) * 1000) / 128;
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int ad9081_write_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan, int val, int val2,
			    long info)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	unsigned long r_clk;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		if (chan->extend_name)
			return -ENODEV;

		if (conv->sample_rate_read_only)
			return -EPERM;

		return ad9081_set_sample_rate(conv, val);

		r_clk = clk_round_rate(conv->clk, val);
		if (r_clk < 0 || r_clk > conv->chip_info->max_rate) {
			dev_warn(&conv->spi->dev,
				 "Error setting ADC sample rate %ld", r_clk);
			return -EINVAL;
		}

		ret = clk_set_rate(conv->clk, r_clk);
		if (ret < 0)
			return ret;
		break;
	case IIO_CHAN_INFO_ENABLE:
		if (chan->output) {
			u8 cddc_num, cddc_mask, fddc_num, fddc_mask;

			ad9081_iiochan_to_fddc_cddc(phy, chan, &fddc_num,
				&fddc_mask, &cddc_num, &cddc_mask);

			if (cddc_mask) {
				ret = adi_ad9081_dac_tx_enable_set(&phy->ad9081,
								   cddc_mask, !!val);
				if (!ret)
					phy->dac_cache.enable[fddc_num] = !!val;
			}
		} else {
			/* FIXME: NO API? */

		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
static int ad9081_read_label(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan, char *label)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;

	return sprintf(label, "%s\n",
		chan->output ? phy->tx_labels[chan->channel] :
		phy->rx_labels[chan->channel]);
}

static const char* const ffh_modes[] = {
	"phase_continuous", "phase_incontinuous", "phase_coherent"
};

static int ad9081_set_power_state(struct ad9081_phy *phy, bool on)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	int ret;

	if (on) {
		if (phy->is_initialized)
			return 0;

		if (phy->supply_reg) {
			ret = regulator_enable(phy->supply_reg);
			if (ret) {
				dev_err(&phy->spi->dev,
					"%s: Failed to enable vdd supply voltage!\n",
					__func__);
				return ret;
			}
		}

		ret = adi_ad9081_device_reset(&phy->ad9081,
			conv->reset_gpio ? AD9081_HARD_RESET_AND_INIT :
			AD9081_SOFT_RESET_AND_INIT);
		if (ret < 0) {
			dev_err(&phy->spi->dev, "%s: reset/init failed (%d)\n",
				__func__, ret);
			return ret;
		}

		phy->jrx_link_tx[0].lane_cal_rate_kbps = 0;

		ret = ad9081_setup(phy->spi);
		if (ret < 0) {
			dev_err(&phy->spi->dev, "%s: setup failed (%d)\n",
				__func__, ret);
			return ret;
		}

		jesd204_fsm_stop(phy->jdev, JESD204_LINKS_ALL);
		jesd204_fsm_clear_errors(phy->jdev, JESD204_LINKS_ALL);
		ret = jesd204_fsm_start(phy->jdev, JESD204_LINKS_ALL);
	} else {
		if (!phy->is_initialized)
			return 0;

		jesd204_fsm_stop(phy->jdev, JESD204_LINKS_ALL);

		phy->jrx_link_tx[0].lane_cal_rate_kbps = 0;

		ret = adi_ad9081_device_reset(&phy->ad9081,
			conv->reset_gpio ? AD9081_HARD_RESET_AND_INIT :
			AD9081_SOFT_RESET_AND_INIT);
		if (ret < 0) {
			dev_err(&phy->spi->dev, "%s: reset/init failed (%d)\n",
				__func__, ret);
			return ret;
		}

		if (phy->supply_reg) {
			ret = regulator_disable(phy->supply_reg);
			if (ret) {
				dev_err(&phy->spi->dev,
					"%s: Failed to disable vdd supply voltage!\n",
					__func__);
				return ret;
			}
		}
	}

	return ret;
}

static ssize_t ad9081_phy_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	unsigned long res;
	bool bres;
	bool enable;
	int ret = 0;

	mutex_lock(&conv->lock);

	switch ((u32)this_attr->address & 0xFF) {
	case AD9081_LOOPBACK_MODE:
		if (conv->id == CHIPID_AD9988 || conv->id == CHIPID_AD9986) {
			ret = -ENOTSUPP;
			break;
		}

		if (phy->tx_disable || phy->rx_disable) {
			ret = -ENODEV;
			break;
		}

		ret = kstrtoul(buf, 0, &res);

		/* setup mxfe loopback mode */
		switch (res) {
		case 0:
			adi_ad9081_jesd_loopback_mode_set(&phy->ad9081, 0);
			adi_ad9081_device_direct_loopback_set(&phy->ad9081,
				0, phy->direct_lb_map);
			break;
		case 1:
			adi_ad9081_device_direct_loopback_set(&phy->ad9081,
				0, phy->direct_lb_map);
			adi_ad9081_jesd_loopback_mode_set(&phy->ad9081, 1);
			break;
		case 2:
		case 3:
			if (phy->adc_frequency_hz != phy->dac_frequency_hz) {
				dev_err(&phy->spi->dev,
					"ADC and DAC frequency doesn't match!\n");
				ret = -EOPNOTSUPP;
				break;
			}

			adi_ad9081_jesd_loopback_mode_set(&phy->ad9081, 0);
			adi_ad9081_device_direct_loopback_set(&phy->ad9081,
				(res == 2) ? 1 : 3, phy->direct_lb_map);
			break;
		default:
			ret = -EINVAL;
		}

		if (!ret)
			phy->device_cache.loopback_mode = res;
		break;
	case AD9081_ADC_CLK_PWDN:
		if (phy->rx_disable) {
			ret = -ENODEV;
			break;
		}

		ret = strtobool(buf, &bres);
		if (ret < 0) {
			ret = -EINVAL;
			break;
		}

		adi_ad9081_adc_clk_enable_set(&phy->ad9081, !bres);
		phy->device_cache.adc_clk_pwdn = bres;
		break;
	case AD9081_MCS:
		ret = kstrtoul(buf, 0, &res);
		if (ret) {
			ret = -EINVAL;
			break;
		}
		/* setup txfe loopback mode */
		ret = ad9081_multichip_sync(phy, res);
		if (!ret)
			phy->mcs_cached_val = res;

		break;

	case AD9081_JESD204_FSM_RESUME:
		if (!phy->jdev) {
			ret = -ENOTSUPP;
			break;
		}

		ret = jesd204_fsm_resume(phy->jdev, JESD204_LINKS_ALL);
		break;
	case AD9081_JESD204_FSM_CTRL:
		if (!phy->jdev) {
			ret = -ENOTSUPP;
			break;
		}

		ret = strtobool(buf, &enable);
		if (ret)
			break;

		if (enable) {
			if (!phy->is_initialized)
				phy->jrx_link_tx[0].lane_cal_rate_kbps = 0;
			jesd204_fsm_stop(phy->jdev, JESD204_LINKS_ALL);
			jesd204_fsm_clear_errors(phy->jdev, JESD204_LINKS_ALL);
			ret = jesd204_fsm_start(phy->jdev, JESD204_LINKS_ALL);
		} else {
			jesd204_fsm_stop(phy->jdev, JESD204_LINKS_ALL);
			jesd204_fsm_clear_errors(phy->jdev, JESD204_LINKS_ALL);
			ret = 0;
		}

		break;
	case AD9081_POWER_DOWN:
		ret = strtobool(buf, &enable);
		if (ret)
			break;
		ret = ad9081_set_power_state(phy, !enable);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&conv->lock);

	return ret ? ret : len;
}

static ssize_t ad9081_phy_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	struct jesd204_dev *jdev = phy->jdev;
	struct jesd204_link *links[4];
	int i, err, num_links;
	bool paused;
	int ret = 0;

	mutex_lock(&conv->lock);
	switch ((u32)this_attr->address & 0xFF) {
	case AD9081_LOOPBACK_MODE:
		if (phy->tx_disable || phy->rx_disable) {
			ret = -ENODEV;
			break;
		}
		ret = sprintf(buf, "%u\n", phy->device_cache.loopback_mode);
		break;
	case AD9081_ADC_CLK_PWDN:
		if (phy->rx_disable) {
			ret = -ENODEV;
			break;
		}
		ret = sprintf(buf, "%u\n", phy->device_cache.adc_clk_pwdn);
		break;
	case AD9081_MCS:
		ret = sprintf(buf, "%u\n", phy->mcs_cached_val);
		break;
	case AD9081_JESD204_FSM_ERROR:
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
	case AD9081_JESD204_FSM_PAUSED:
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
	case AD9081_JESD204_FSM_STATE:
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
		 * just get the first link state; we're assuming that all 3
		 * are in sync and that AD9081_JESD204_FSM_PAUSED
		 * was called before
		 */
		ret = sprintf(buf, "%s\n",
			jesd204_link_get_state_str(links[0]));
		break;
	case AD9081_JESD204_FSM_CTRL:
		if (!phy->jdev) {
			ret = -ENOTSUPP;
			break;
		}

		ret = sprintf(buf, "%d\n", phy->is_initialized);
		break;
	case AD9081_POWER_DOWN:
		ret = sprintf(buf, "%d\n", !phy->is_initialized);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&conv->lock);

	return ret;
}

static IIO_DEVICE_ATTR(loopback_mode, S_IRUGO | S_IWUSR,
		ad9081_phy_show,
		ad9081_phy_store,
		AD9081_LOOPBACK_MODE);

static IIO_CONST_ATTR(loopback_mode_available, "0 1 2 3");

static IIO_DEVICE_ATTR(adc_clk_powerdown, S_IRUGO | S_IWUSR,
		ad9081_phy_show,
		ad9081_phy_store,
		AD9081_ADC_CLK_PWDN);

static IIO_DEVICE_ATTR(multichip_sync, S_IRUGO | S_IWUSR,
		ad9081_phy_show,
		ad9081_phy_store,
		AD9081_MCS);

static IIO_DEVICE_ATTR(jesd204_fsm_error, 0444,
		       ad9081_phy_show,
		       NULL,
		       AD9081_JESD204_FSM_ERROR);

static IIO_DEVICE_ATTR(jesd204_fsm_paused, 0444,
		       ad9081_phy_show,
		       NULL,
		       AD9081_JESD204_FSM_PAUSED);

static IIO_DEVICE_ATTR(jesd204_fsm_state, 0444,
		       ad9081_phy_show,
		       NULL,
		       AD9081_JESD204_FSM_STATE);

static IIO_DEVICE_ATTR(jesd204_fsm_resume, 0200,
		       NULL,
		       ad9081_phy_store,
		       AD9081_JESD204_FSM_RESUME);

static IIO_DEVICE_ATTR(jesd204_fsm_ctrl, 0644,
		       ad9081_phy_show,
		       ad9081_phy_store,
		       AD9081_JESD204_FSM_CTRL);

static IIO_DEVICE_ATTR(powerdown, 0644,
		       ad9081_phy_show,
		       ad9081_phy_store,
		       AD9081_POWER_DOWN);

static struct attribute *ad9081_phy_attributes[] = {
	&iio_dev_attr_loopback_mode.dev_attr.attr,
	&iio_const_attr_loopback_mode_available.dev_attr.attr,
	&iio_dev_attr_adc_clk_powerdown.dev_attr.attr,
	&iio_dev_attr_multichip_sync.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_error.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_state.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_paused.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_resume.dev_attr.attr,
	&iio_dev_attr_jesd204_fsm_ctrl.dev_attr.attr,
	&iio_dev_attr_powerdown.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9081_phy_attribute_group = {
	.attrs = ad9081_phy_attributes,
};

static int ad9081_request_fd_irqs(struct axiadc_converter *conv)
{
	struct device *dev = &conv->spi->dev;
	struct gpio_desc *gpio;

	gpio = devm_gpiod_get(dev, "fastdetect-a", GPIOD_IN);
	if (!IS_ERR(gpio)) {
		int ret, irq = gpiod_to_irq(gpio);

		if (irq < 0)
			return irq;

		ret = devm_request_threaded_irq(dev,
				irq, NULL, ad9081_fdA_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"fastdetect-a", conv);
		if (ret < 0)
			return ret;
	}

	gpio = devm_gpiod_get(dev, "fastdetect-b", GPIOD_IN);
	if (!IS_ERR(gpio)) {
		int ret, irq = gpiod_to_irq(gpio);

		if (irq < 0)
			return irq;

		ret = devm_request_threaded_irq(dev,
				irq, NULL, ad9081_fdB_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"fastdetect-b", conv);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad9081_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int i;

	for (i = 0; i < conv->chip_info->num_channels; i++) {
		axiadc_write(st, ADI_REG_CHAN_CNTRL_2(i),
			     (i & 1) ? 0x00004000 : 0x40000000);
		axiadc_write(st, ADI_REG_CHAN_CNTRL(i),
			     ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE |
				     ADI_IQCOR_ENB | ADI_ENABLE);
	}

	return 0;
}
static int ad9081_status_show(struct seq_file *file, void *offset)
{
	struct axiadc_converter *conv = spi_get_drvdata(file->private);
	struct ad9081_phy *phy = conv->phy;
	int ret, l;
	u16 stat;
	u8 vals[3];

	for (l = AD9081_LINK_0; !phy->tx_disable && (l < (ad9081_link_is_dual(phy->jrx_link_tx) ?
		AD9081_LINK_ALL : AD9081_LINK_1)); l++) {

		ret = adi_ad9081_jesd_rx_link_status_get(
			&phy->ad9081, l, &stat);
		if (ret)
			return -EFAULT;

		adi_ad9081_hal_reg_get(&phy->ad9081, REG_JRX_TPL_3_ADDR, &vals[0]);
		adi_ad9081_hal_reg_get(&phy->ad9081, REG_JRX_TPL_4_ADDR, &vals[1]);
		adi_ad9081_hal_reg_get(&phy->ad9081, REG_JRX_TPL_5_ADDR, &vals[2]);

		if (phy->jrx_link_tx[l - 1].jesd_param.jesd_jesdv == JESD204_VERSION_C) {
			stat >>= 8;
			seq_printf(file,
				"JESD TX (JRX) Link%d 204C status %s (%d), TPL Phase Difference Read %u, Set %u\n",
				l - 1, ad9081_jrx_204c_states[stat & 0x7], stat,
				vals[2], vals[1] << 8 | vals[0]);
		} else {
			seq_printf(file,
				"JESD TX (JRX) Link%d 204B 0x%X lanes in DATA, TPL Phase Difference Read %u, Set %u\n",
				l - 1, stat & 0xF, vals[2], vals[1] << 8 | vals[0]);
		}
	}

	for (l = AD9081_LINK_0; !phy->rx_disable && (l < (ad9081_link_is_dual(phy->jtx_link_rx) ?
		AD9081_LINK_ALL : AD9081_LINK_1)); l++) {

		ret = adi_ad9081_jesd_tx_link_status_get(
			&phy->ad9081, l, &stat);
		if (ret)
			return -EFAULT;

		if (phy->jtx_link_rx[0].jesd_param.jesd_jesdv == JESD204_VERSION_C) {
			seq_printf(file,
				"JESD RX (JTX) Link%d 204C PLL %s, PHASE %s, MODE %s\n",
				l - 1,
				stat & BIT(5) ? "locked" : "unlocked",
				stat & BIT(6) ? "established" : "lost",
				stat & BIT(7) ? "invalid" : "valid");
		} else {
			seq_printf(file,
				"JESD RX (JTX) Link%d 204B in %s, SYNC %s, PLL %s, PHASE %s, MODE %s\n",
				l - 1, ad9081_jtx_qbf_states[stat & 0xf],
				stat & BIT(4) ? "deasserted" : "asserted",
				stat & BIT(5) ? "locked" : "unlocked",
				stat & BIT(6) ? "established" : "lost",
				stat & BIT(7) ? "invalid" : "valid");
		}
	}

	return 0;
}

static void ad9081_work_func(struct work_struct *work)
{
	u8 status;
	int ret;
	struct ad9081_phy *phy =
		container_of(work, struct ad9081_phy, dwork.work);

	ret = adi_ad9081_hal_reg_get(&phy->ad9081, REG_IRQ_STATUS0_ADDR,
				     &status);

	if (!(status & BIT(6))) {
		dev_err(&phy->spi->dev, "Link Reset IRQ_STATUS0: 0x%X\n", status);

		phy->jrx_link_tx[0].lane_cal_rate_kbps = 0;
		jesd204_fsm_stop(phy->jdev, JESD204_LINKS_ALL);
		jesd204_fsm_clear_errors(phy->jdev, JESD204_LINKS_ALL);
		jesd204_fsm_start(phy->jdev, JESD204_LINKS_ALL);

		return;
	}

	schedule_delayed_work(&phy->dwork, msecs_to_jiffies(1000));
}

static int ad9081_fsc_set(void *arg, const u64 val)
{
	struct iio_dev *indio_dev = arg;
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	int ret;

	if (!val)
		return -EINVAL;

	mutex_lock(&conv->lock);
	ret = adi_ad9081_dac_fsc_set(&phy->ad9081, AD9081_DAC_ALL, val, 1);
	mutex_unlock(&conv->lock);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(ad9081_fsc_fops, NULL, ad9081_fsc_set, "%llu");

static const char* const pfir_filter_modes[] = {
	"disabled", "real_n4", "real_n2", "undef",
	"matrix", "complex_full", "complex_half", "real_n"
};

static unsigned char pfir_mode_tap_lut[] = {0, 48, 96, 0, 48, 128, 96, 192};

static const char* const pfir_filter_pairs[] = {
	"adc_pair_0", "adc_pair_1", "adc_pair_all"
};

static const char* const pfir_filter_pages[] = {
	"page_0", "page_1", "page_2", "page_3", "page_all"
};

enum coeff_load_sel_types {
	REAL_I_LOAD = 0,
	REAL_Q_LOAD = 1,
	REAL_CROSS_I_LOAD = 2,
	REAL_CROSS_Q_LOAD = 3,
	COMPLEX_LOAD = 4,
};

static const char* const pfir_filter_load_type[] = {
	"real_i", "real_q", "real_cross_i", "real_cross_q", "complex"
};

static u32 ad9081_pfir_gain_enc(int val)
{
	switch (val) {
	case -12:
		return AD9081_ADC_PFIR_GAIN_N12DB;
	case -6:
		return AD9081_ADC_PFIR_GAIN_N6DB;
	case 6:
		return AD9081_ADC_PFIR_GAIN_P6DB;
	case 12:
		return AD9081_ADC_PFIR_GAIN_P12DB;
	default:
		return AD9081_ADC_PFIR_GAIN_0DB;
	}
};

static int ad9081_adc_pfir_prog(struct ad9081_phy *phy,
	adi_ad9081_adc_pfir_ctl_page_e ctl_pages,
	adi_ad9081_adc_pfir_coeff_page_e coeff_pages,
	adi_ad9081_adc_pfir_i_mode_e mode,
	uint8_t coeff_load_sel,
	int16_t *coeffs, uint8_t coeffs_num)
{
	int ret, i;

	if (coeffs_num != pfir_mode_tap_lut[mode])
		dev_err(&phy->spi->dev, "Expected %d coefficients got %d",
			pfir_mode_tap_lut[mode], coeffs_num);

	ret = adi_ad9081_adc_pfir_coeff_validate(&phy->ad9081, coeffs_num, coeffs);
	if (ret)
		dev_err(&phy->spi->dev, "Validate %s:%s coefficients failed (%d)",
			pfir_filter_modes[mode],
			pfir_filter_load_type[coeff_load_sel], ret);

	ret = adi_ad9081_adc_pfir_coeff_load_sel_set(&phy->ad9081, ctl_pages,
			1 << coeff_load_sel);
	if (ret)
		goto out;

	for (i = 0; i < coeffs_num; i++) {
		ret = adi_ad9081_adc_pfir_coeff_set(&phy->ad9081, coeff_pages, i,
						    coeffs[i]);
		if (ret)
			goto out;
	}

	ret = adi_ad9081_adc_pfir_coeff_load_sel_set(&phy->ad9081, ctl_pages, 0);
	if (ret)
		goto out;

	ret = adi_ad9081_adc_pfir_coeff_xfer_set(&phy->ad9081, ctl_pages, 1);
	if (ret)
		goto out;

	ret =  adi_ad9081_adc_pfir_coeff_xfer_set(&phy->ad9081, ctl_pages, 0);
out:
	dev_printk(ret ? KERN_ERR : KERN_INFO, &phy->spi->dev,
		"Filter loading ADC<0x%X> on pages <0x%X> using <%s> with <%d> tabs {%d, %d, %d, ..., %d, %d, %d} load type %s\n",
		ctl_pages, coeff_pages, pfir_filter_modes[mode], coeffs_num,
		coeffs[0], coeffs[1], coeffs[2],
		coeffs[coeffs_num - 3], coeffs[coeffs_num - 2], coeffs[coeffs_num - 1],
		pfir_filter_load_type[coeff_load_sel]);

	return ret;
}

#if 0
int adi_ad9081_adc_pfir_coeffs_get(adi_ad9081_device_t *device,
			       adi_ad9081_adc_pfir_coeff_page_e coeff_pages)
{
	int32_t err, j;
	u8 val1, val2;


	err = adi_ad9081_adc_pfir_coeff_page_set(device, coeff_pages);
	AD9081_ERROR_RETURN(err);
	for (j = 0; j < 192; j++) {
		err = adi_ad9081_hal_reg_get(device, 0x1900 + j * 2,
					     &val2);

		err = adi_ad9081_hal_reg_get(device, 0x1901 + j * 2,
					     &val1);

		pr_err("%d:0x%X: \t%d\t(0x%X)", j, 0x1900 + j*2 ,
			(short)(val1 << 8 | val2), (val1 << 8 | val2));
	}

	return API_CMS_ERROR_OK;
}
#endif

static int ad9081_parse_fir(struct ad9081_phy *phy,
				 char *data, u32 size)
{
	char *line;
	int i = 0, q = 0, ret;
	char *ptr = data;

	adi_ad9081_adc_pfir_ctl_page_e ctl_pages = AD9081_ADC_PFIR_ADC_PAIR_ALL;
	adi_ad9081_adc_pfir_coeff_page_e coeff_pages = AD9081_ADC_PFIR_COEFF_PAGE_ALL;
	adi_ad9081_adc_pfir_i_mode_e i_mode = AD9081_ADC_PFIR_I_MODE_DISABLE;
	adi_ad9081_adc_pfir_q_mode_e q_mode = AD9081_ADC_PFIR_Q_MODE_DISABLE;
	adi_ad9081_adc_pfir_gain_e ix_gain = 0, iy_gain = 0, qx_gain = 0, qy_gain = 0;
	u8 half_complex_delay = 0, image_cancel_delay = 0;
	u8 read_mask = 0;

	while ((line = strsep(&ptr, "\n"))) {
		if (line >= data + size) {
			break;
		}

		if (line[0] == '#')
			continue;

		if (~read_mask & BIT(0)) {
			char imode[16], qmode[16];

			ret = sscanf(line, "mode: %15s %15s",
				     imode, qmode);

			if (ret == 2)
				q_mode = sysfs_match_string(pfir_filter_modes, qmode);

			if (ret == 1 || ret == 2) {
				i_mode = sysfs_match_string(pfir_filter_modes, imode);
				read_mask |= BIT(0);
				continue;
			}
		}

		if (~read_mask & BIT(1)) {
			int ix = 0, iy = 0, qx = 0, qy = 0;

			ret = sscanf(line, "gain: %d %d %d %d",
				     &ix, &iy, &qx, &qy);

			if (ret == 4) {
				ix_gain = ad9081_pfir_gain_enc(ix);
				iy_gain = ad9081_pfir_gain_enc(iy);
				qx_gain = ad9081_pfir_gain_enc(qx);
				qy_gain = ad9081_pfir_gain_enc(qy);

				read_mask |= BIT(1);
				continue;
			}
		}

		if (~read_mask & BIT(2)) {
			char a[16], b[16];

			ret = sscanf(line, "dest: %15s %15s", a, b);

			if (ret == 2) {
				ret = sysfs_match_string(pfir_filter_pages, b);
				if (ret < 0)
					goto out;
				coeff_pages = 1 << ret;
				if (ret == 4)
					coeff_pages = AD9081_ADC_PFIR_COEFF_PAGE_ALL;

				ret = sysfs_match_string(pfir_filter_pairs, a);
				if (ret < 0)
					goto out;
				ctl_pages = 1 << ret;
				if (ret == 2)
					ctl_pages = AD9081_ADC_PFIR_ADC_PAIR_ALL;


				read_mask |= BIT(2);
				continue;
			}
		}

		if (~read_mask & BIT(3)) {
			ret = sscanf(line, "delay: %hhu %hhu",
				     &half_complex_delay, &image_cancel_delay);
			if (ret == 2) {
				read_mask |= BIT(3);
				continue;
			}
		}

		if (i_mode == AD9081_ADC_PFIR_I_MODE_MATRIX) {
			int val1, val2, val3, val4;

			ret = sscanf(line, "%d %d %d %d", &val1, &val2, &val3, &val4);
			if (ret == 4) {
				if (i >= ARRAY_SIZE(phy->coeffs_i) / 2)
					return -EINVAL;

				phy->coeffs_i[i] = (short)val1;
				phy->coeffs_i[i + 96] = (short)val2;
				i++;


				if (q >= ARRAY_SIZE(phy->coeffs_q) / 2)
					return -EINVAL;

				phy->coeffs_q[q] = (short)val3;
				phy->coeffs_q[q + 96] = (short)val4;
				q++;

				continue;
			}
		} else {
			int val1, val2;

			ret = sscanf(line, "%d %d", &val1, &val2);
			if (ret == 1 || ret == 2) {
				if (i >= ARRAY_SIZE(phy->coeffs_i))
					return -EINVAL;

				if (i_mode == AD9081_ADC_PFIR_I_MODE_COMPLEX_FULL) {
					phy->coeffs_i[i++] = (short)val1;
					phy->coeffs_i[i++] = (short)val2;

				} else {
					phy->coeffs_i[i++] = (short)val1;

					if (ret == 1) /* single real_n */
						val2 = val1;

					phy->coeffs_q[q++] = (short)val2;
				}

				continue;
			}
		}
	}

	ret = adi_ad9081_adc_pfir_hc_prog_delay_set(&phy->ad9081,
		coeff_pages, image_cancel_delay);
	if (ret)
		return -EIO;
	ret = adi_ad9081_adc_pfir_half_complex_delay_set(&phy->ad9081,
		ctl_pages, half_complex_delay);
	if (ret)
		return -EIO;
	ret = adi_ad9081_adc_pfir_quad_mode_set(&phy->ad9081, ctl_pages,
		(phy->chip_id.prod_id == CHIPID_AD9081 ||
		phy->chip_id.prod_id == CHIPID_AD9988) ? 1 : 0);
	if (ret)
		return -EIO;
	ret = adi_ad9081_adc_pfir_coeff_page_set(&phy->ad9081, coeff_pages);
	if (ret)
		return -EIO;
	ret = adi_ad9081_adc_pfir_coeff_clear_set(&phy->ad9081, ctl_pages, 1);
	if (ret)
		return -EIO;
	ret = adi_ad9081_adc_pfir_coeff_clear_set(&phy->ad9081, ctl_pages, 0);
	if (ret)
		return -EIO;
	ret = adi_ad9081_adc_pfir_i_mode_set(&phy->ad9081, ctl_pages, i_mode);
	if (ret)
		return -EIO;
	ret = adi_ad9081_adc_pfir_q_mode_set(&phy->ad9081, ctl_pages, q_mode);
	if (ret)
		return -EIO;
	ret = adi_ad9081_adc_pfir_i_gain_set(&phy->ad9081, ctl_pages, ix_gain,
					     iy_gain);
	if (ret)
		return -EIO;
	ret = adi_ad9081_adc_pfir_q_gain_set(&phy->ad9081, ctl_pages, qx_gain,
					     qy_gain);
	if (ret)
		return -EIO;

	if (i_mode == AD9081_ADC_PFIR_I_MODE_COMPLEX_FULL) {
		ret = ad9081_adc_pfir_prog(phy, ctl_pages, coeff_pages,
			i_mode, COMPLEX_LOAD, phy->coeffs_i, i);
	} else {
		if (i_mode && i) {
			ret = ad9081_adc_pfir_prog(phy, ctl_pages, coeff_pages,
				i_mode, REAL_I_LOAD, phy->coeffs_i, i);

			if (ret)
				goto out;
		}
		if (q_mode && q) {
			ret = ad9081_adc_pfir_prog(phy, ctl_pages, coeff_pages,
				(adi_ad9081_adc_pfir_i_mode_e)q_mode, REAL_Q_LOAD,
				phy->coeffs_q, q);

			if (ret)
				goto out;
		}

		if (i_mode == AD9081_ADC_PFIR_I_MODE_MATRIX) {
			ret = ad9081_adc_pfir_prog(phy, ctl_pages, coeff_pages,
				i_mode, REAL_CROSS_I_LOAD, &phy->coeffs_i[96], i);
			if (ret)
				goto out;
			ret = ad9081_adc_pfir_prog(phy, ctl_pages, coeff_pages,
				i_mode, REAL_CROSS_Q_LOAD, &phy->coeffs_q[96], q);
			if (ret)
				goto out;
		}
	}

	//adi_ad9081_adc_pfir_coeffs_get(&phy->ad9081, coeff_pages);

	if (ret != API_CMS_ERROR_OK)
		dev_err(&phy->spi->dev, "Prgramming filter failed (%d)", ret);

	return size;

out:
	dev_err(&phy->spi->dev, "malformed pFir filter file detected\n");

	return -EINVAL;
}

static ssize_t
ad9081_fir_bin_write(struct file *filp, struct kobject *kobj,
		       struct bin_attribute *bin_attr,
		       char *buf, loff_t off, size_t count)
{

	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;

	return ad9081_parse_fir(phy, buf, count);
}

static adi_ad9081_deser_mode_e
ad9081_deserializer_mode_get(struct ad9081_jesd_link *txlink)
{
	return (txlink->jesd_param.jesd_jesdv == 1) ?
	((txlink->lane_rate_kbps > (AD9081_DESER_MODE_204B_BR_TRESH / 1000)) ?
		AD9081_HALF_RATE : AD9081_FULL_RATE) :
		((txlink->lane_rate_kbps <
		(AD9081_DESER_MODE_204C_BR_TRESH / 1000)) ?
		AD9081_HALF_RATE : AD9081_QUART_RATE);
}

static int ad9081_val_to_prbs(int val)
{
	switch (val) {
	case 0:
		return PRBS_NONE;
	case 7:
		return PRBS7;
	case 9:
		return PRBS9;
	case 15:
		return PRBS15;
	case 23:
		return PRBS23;
	case 31:
		return PRBS31;
	default:
		return -EINVAL;
	}
};

static ssize_t ad9081_debugfs_read(struct file *file, char __user *userbuf,
	size_t count, loff_t *ppos)
{
	struct ad9081_debugfs_entry *entry = file->private_data;
	struct iio_dev *indio_dev = entry->indio_dev;
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	adi_ad9081_deser_mode_e dmode;
	adi_cms_jesd_prbs_pattern_e prbs;
	s16 eye_data[192];
	u64 val = 0;
	ssize_t len = 0;
	int ret, i, j, spo_steps;
	u16 duration;
	u8 api_rev[3], lane;

	if (*ppos)
		return 0;

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

	} else if (entry->cmd) {
		switch (entry->cmd) {
		case DBGFS_BIST_PRBS_JRX_ERR:
			mutex_lock(&conv->lock);
			for (i = 0; i < phy->jrx_link_tx[0].jesd_param.jesd_l; i++) {
				adi_ad9081_prbs_test_t prbs_rx_result;

				for (j = 0; j < 8; j++)
					if (phy->jrx_link_tx[0].logiclane_mapping[j] == i) {
						ret = adi_ad9081_jesd_rx_phy_prbs_test_result_get(
							&phy->ad9081, j, &prbs_rx_result);

						len += snprintf(phy->dbuf + len, sizeof(phy->dbuf), "%u/%u ",
							prbs_rx_result.phy_prbs_err_cnt,
							prbs_rx_result.phy_prbs_pass);
					}
			}

			if (ad9081_link_is_dual(phy->jrx_link_tx)) {
				len += snprintf(phy->dbuf + len, sizeof(phy->dbuf), ": ");
				for (i = 0; i < phy->jrx_link_tx[1].jesd_param.jesd_l; i++) {
					adi_ad9081_prbs_test_t prbs_rx_result;

					for (j = 0; j < 8; j++)
						if (phy->jrx_link_tx[1].logiclane_mapping[j] == i) {
							ret = adi_ad9081_jesd_rx_phy_prbs_test_result_get(&phy->ad9081,
								j, &prbs_rx_result);

							len += snprintf(phy->dbuf + len, sizeof(phy->dbuf), "%u/%u ",
								prbs_rx_result.phy_prbs_err_cnt,
								prbs_rx_result.phy_prbs_pass);
						}
				}
			}

			mutex_unlock(&conv->lock);
			len += snprintf(phy->dbuf + len, sizeof(phy->dbuf), "\n");
			break;
		case DBGFS_BIST_JRX_SPO_SWEEP:
			len = snprintf(phy->dbuf, sizeof(phy->dbuf), "l:%u r:%u\n",
				entry->val >> 16, entry->val & 0xFFFF);
			break;
		case DBGFS_BIST_JRX_SPO_SET:
			len = snprintf(phy->dbuf, sizeof(phy->dbuf), "%d\n", (int)entry->val);
			break;
		case DBGFS_BIST_JRX_2D_EYE:
			if (!entry->val)
				return -EINVAL;

			dmode = ad9081_deserializer_mode_get(&phy->jrx_link_tx[0]);
			lane = (entry->val & 0xFF) - 1;
			prbs = (entry->val >> 8) & 0xFF;
			duration = (entry->val >> 16) & 0xFFFF;

			mutex_lock(&conv->lock);
			entry->val = 0;

			switch (dmode) {
			case AD9081_QUART_RATE:
				spo_steps = 32;

				ret = adi_ad9081_jesd_cal_bg_cal_pause(&phy->ad9081);
				if (ret)
					break;
				ret = adi_ad9081_jesd_rx_qr_two_dim_eye_scan(&phy->ad9081,
					lane, eye_data);
				if (ret)
					break;
				ret = adi_ad9081_jesd_cal_bg_cal_start(&phy->ad9081);
				break;
			case AD9081_HALF_RATE:
				spo_steps = 64;
				ret = adi_ad9081_jesd_rx_hr_two_dim_eye_scan(&phy->ad9081,
					lane, prbs, duration, eye_data);
				break;
			default:
				ret = -EOPNOTSUPP;
			}

			if (ret < 0) {
				dev_err(&phy->spi->dev,
					"JRX eye_scan lane%u failed (%d)", lane, ret);
				mutex_unlock(&conv->lock);
				return ret;
			}

			len = snprintf(phy->dbuf, sizeof(phy->dbuf),
				"# lane %u spo_steps %u rate %lu\n",
				lane, spo_steps, phy->jrx_link_tx[0].lane_rate_kbps);

			for (i = 0; i < (spo_steps * 3); i += 3)
				if (eye_data[i])
					len += snprintf(phy->dbuf + len,
						sizeof(phy->dbuf),
						"%d,%d,%d\n", eye_data[i],
						eye_data[i + 1], eye_data[i + 2]);

			mutex_unlock(&conv->lock);
			break;
		case DBGFS_DEV_API_INFO:
			adi_ad9081_device_api_revision_get(&phy->ad9081,
				&api_rev[0], &api_rev[1], &api_rev[2]);

			len = snprintf(phy->dbuf, sizeof(phy->dbuf), "%u.%u.%u\n",
				api_rev[0], api_rev[1], api_rev[2]);
			break;
		case DBGFS_DEV_CHIP_INFO:
			adi_ad9081_device_api_revision_get(&phy->ad9081, &api_rev[0],
				&api_rev[1], &api_rev[2]);

			len = snprintf(phy->dbuf, sizeof(phy->dbuf), "AD%X Rev. %u Grade %u\n",
				conv->id, phy->chip_id.dev_revision, phy->chip_id.prod_grade);
			break;
		default:
			val = entry->val;
		}
	} else {
		return -EFAULT;
	}
	if (!len)
		len = snprintf(phy->dbuf, sizeof(phy->dbuf), "%llu\n", val);

	return simple_read_from_buffer(userbuf, count, ppos, phy->dbuf, len);
}

static ssize_t ad9081_debugfs_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct ad9081_debugfs_entry *entry = file->private_data;
	struct iio_dev *indio_dev = entry->indio_dev;
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	int val2, val3, val4, ret;
	s64 val;
	char buf[80];
	u8 val_u8;

	u8 lv, rv;

	count = min_t(size_t, count, (sizeof(buf) - 1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;

	ret = sscanf(buf, "%lli %i %i %i", &val, &val2, &val3, &val4);
	if (ret < 1)
		return -EINVAL;

	switch (entry->cmd) {
	case DBGFS_BIST_PRBS_JRX:
		if (ret < 1)
			return -EINVAL;
		if (ret == 1)
			val2 = 1; /* 1 second */

		mutex_lock(&conv->lock);
		if (val == 0)
			adi_ad9081_jesd_rx_phy_prbs_test_disable_set(&phy->ad9081);
		else
			adi_ad9081_jesd_rx_phy_prbs_test(&phy->ad9081,
				ad9081_val_to_prbs(val), val2);
		entry->val = val;
		mutex_unlock(&conv->lock);

		return count;
	case DBGFS_BIST_PRBS_JTX:
		if (ret < 1)
			return -EINVAL;

		mutex_lock(&conv->lock);
		if (val == 0)
			adi_ad9081_jesd_tx_gen_test(&phy->ad9081,
				ad9081_link_sel(phy->jtx_link_rx),
				AD9081_JESD_TX_TEST_DATA_SAMPLE,
				AD9081_JESD_TX_TEST_MODE_DISABLED);
		else
			adi_ad9081_jesd_tx_phy_prbs_test(&phy->ad9081,
				ad9081_link_sel(phy->jtx_link_rx), ad9081_val_to_prbs(val));
		entry->val = val;
		mutex_unlock(&conv->lock);

		return count;
	case DBGFS_BIST_JRX_SPO_SET:
		if (ret < 2)
			return -EINVAL;

		ret = adi_ad9081_jesd_rx_gen_2s_comp(&phy->ad9081, val2, 7,
						     &val_u8);
		if (ret)
			return ret;
		mutex_lock(&conv->lock);
		ret = adi_ad9081_jesd_rx_spo_set(&phy->ad9081, val & 0x7, val_u8);
		if (ret) {
			mutex_unlock(&conv->lock);
			return ret;
		}

		entry->val = val2;
		mutex_unlock(&conv->lock);

		return count;
	case DBGFS_BIST_JRX_2D_EYE:
		if (ret < 1)
			return -EINVAL;

		if (ret < 2)
			val2 = 7; /* PRBS7 */

		if (ret < 3)
			val3 = 10; /* 10 ms */

		if (val > 7)
			return -EINVAL;

		if (phy->jrx_link_tx[0].logiclane_mapping[val] >=
			phy->jrx_link_tx[0].jesd_param.jesd_l)
			ret = -EINVAL;
		else
			ret = 0;

		if (ret && ad9081_link_is_dual(phy->jrx_link_tx)) {
			if (phy->jrx_link_tx[1].logiclane_mapping[val] >=
				phy->jrx_link_tx[1].jesd_param.jesd_l)
				ret = -EINVAL;
			else
				ret = 0;
		}

		if (ret)
			return ret;

		val = val + 1;

		ret = ad9081_val_to_prbs(val2);
		if (ret < 0)
			return ret;

		val2 = ret;
		mutex_lock(&conv->lock);
		/*           Time          PRBS                 Lane */
		entry->val = val3 << 16 | (val2 & 0xFF) << 8 | (val & 0xFF);
		mutex_unlock(&conv->lock);

		return count;
	case DBGFS_BIST_JRX_SPO_SWEEP:
		if (ret < 2)
			return -EINVAL;
		if (ret == 2)
			val3 = 1; /* 1 second */

		mutex_lock(&conv->lock);
		ret = adi_ad9081_jesd_cal_bg_cal_pause(&phy->ad9081);
		if (ret) {
			mutex_unlock(&conv->lock);
			return ret;
		}
		ret = adi_ad9081_jesd_rx_spo_sweep(&phy->ad9081, val & 0x7,
				ad9081_val_to_prbs(val2),
				ad9081_deserializer_mode_get(&phy->jrx_link_tx[0]),
				val3, &lv, &rv);

		adi_ad9081_jesd_cal_bg_cal_start(&phy->ad9081);
		if (ret) {
			mutex_unlock(&conv->lock);
			return ret;
		}

		entry->val = lv << 16 | rv;
		mutex_unlock(&conv->lock);

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

static const struct file_operations ad9081_debugfs_reg_fops = {
	.open = simple_open,
	.read = ad9081_debugfs_read,
	.write = ad9081_debugfs_write,
};

static void ad9081_add_debugfs_entry(struct iio_dev *indio_dev,
				       const char *propname, unsigned int cmd)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	unsigned int i = phy->ad9081_debugfs_entry_index;

	if (WARN_ON(i >= ARRAY_SIZE(phy->debugfs_entry)))
		return;

	phy->debugfs_entry[i].indio_dev = indio_dev;
	phy->debugfs_entry[i].propname = propname;
	phy->debugfs_entry[i].cmd = cmd;

	phy->ad9081_debugfs_entry_index++;
}

static int ad9081_post_iio_register(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9081_phy *phy = conv->phy;
	int i;

	phy->ad9081_debugfs_entry_index = 0;

	if (iio_get_debugfs_dentry(indio_dev)) {
		debugfs_create_devm_seqfile(&conv->spi->dev, "status",
					    iio_get_debugfs_dentry(indio_dev),
					    ad9081_status_show);

		ad9081_add_debugfs_entry(indio_dev,
			"bist_prbs_select_jrx", DBGFS_BIST_PRBS_JRX);
		ad9081_add_debugfs_entry(indio_dev,
			"bist_prbs_select_jtx", DBGFS_BIST_PRBS_JTX);
		ad9081_add_debugfs_entry(indio_dev,
			"bist_prbs_error_counters_jrx", DBGFS_BIST_PRBS_JRX_ERR);
		ad9081_add_debugfs_entry(indio_dev,
			"bist_spo_set_jrx", DBGFS_BIST_JRX_SPO_SET);
		ad9081_add_debugfs_entry(indio_dev,
			"bist_spo_sweep_jrx", DBGFS_BIST_JRX_SPO_SWEEP);
		ad9081_add_debugfs_entry(indio_dev,
			"bist_2d_eyescan_jrx", DBGFS_BIST_JRX_2D_EYE);
		ad9081_add_debugfs_entry(indio_dev,
			"api_version", DBGFS_DEV_API_INFO);
		ad9081_add_debugfs_entry(indio_dev,
			"chip_version", DBGFS_DEV_CHIP_INFO);

		for (i = 0; i < phy->ad9081_debugfs_entry_index; i++)
			debugfs_create_file( phy->debugfs_entry[i].propname, 0644,
					    iio_get_debugfs_dentry(indio_dev),
					    &phy->debugfs_entry[i],
					    &ad9081_debugfs_reg_fops);

		debugfs_create_file_unsafe("dac-full-scale-current-ua", 0200,
			iio_get_debugfs_dentry(indio_dev), indio_dev,
			&ad9081_fsc_fops);

		debugfs_create_u8("adi,direct-loopback-mode-dac-adc-mapping",
			0644, iio_get_debugfs_dentry(indio_dev),
			&phy->direct_lb_map);
	}

	sysfs_bin_attr_init(&phy->bin);
	phy->bin.attr.name = "filter_fir_config";
	phy->bin.attr.mode = S_IWUSR;
	phy->bin.write = ad9081_fir_bin_write;
	phy->bin.size = 4096;

	return sysfs_create_bin_file(&indio_dev->dev.kobj, &phy->bin);
}

static int ad9081_get_jesd_converter_selection(struct ad9081_phy *phy,
					       const struct device_node *np,
					       const char *phandle_name,
					       struct ad9081_jesd_link *link)
{
	struct of_phandle_args args;
	int ret, i;
	u32 reg;

	for (i = 0; i < ARRAY_SIZE(link->link_converter_select); i++) {
		ret = of_parse_phandle_with_fixed_args(np, phandle_name, 1, i,
						       &args);
		if (ret)
			break;

		ret = of_property_read_u32(args.np, "reg", &reg);
		if (ret || reg > 7 || args.args[0] > 1) {
			dev_err(&phy->spi->dev,
				"Invalid converter selection (%d, %d, %d)\n",
				ret, reg, args.args[0]);
			return -EINVAL;
		}

		link->link_converter_select[i] = 2 * reg + args.args[0];

		dev_dbg(&phy->spi->dev, "%s: %d = 0x%X\n", __func__, i,
		       link->link_converter_select[i]);

		of_node_put(args.np);
	}

	return i;
}

static int ad9081_parse_jesd_link_dt(struct ad9081_phy *phy,
				     struct device_node *np,
				     struct ad9081_jesd_link *link, bool jtx)
{
	struct device *dev = &phy->spi->dev;
	u32 tmp;
	int ret;

	link->jesd204_link.is_transmit = !jtx;
	link->jesd204_link.scrambling = 1; /* Force scambling on */
	link->jesd_param.jesd_scr = 1; /* Force scambling on */

	tmp = 0;

	JESD204_LNK_READ_DEVICE_ID(dev, np, &link->jesd204_link,
				   &link->jesd_param.jesd_did, 0);

	JESD204_LNK_READ_OCTETS_PER_FRAME(dev, np, &link->jesd204_link,
					  &link->jesd_param.jesd_f, -1);

	JESD204_LNK_READ_FRAMES_PER_MULTIFRAME(dev, np, &link->jesd204_link,
					       &link->jesd_param.jesd_k, -1);

	JESD204_LNK_READ_SAMPLES_PER_CONVERTER_PER_FRAME(dev, np,
							 &link->jesd204_link,
							 &link->jesd_param.jesd_s, -1);

	JESD204_LNK_READ_HIGH_DENSITY(dev, np, &link->jesd204_link,
				      &link->jesd_param.jesd_hd, -1);

	JESD204_LNK_READ_CONVERTER_RESOLUTION(dev, np, &link->jesd204_link,
					      &link->jesd_param.jesd_n, -1);

	JESD204_LNK_READ_BITS_PER_SAMPLE(dev, np, &link->jesd204_link,
					 &link->jesd_param.jesd_np, -1);

	JESD204_LNK_READ_NUM_CONVERTERS(dev, np, &link->jesd204_link,
					&link->jesd_param.jesd_m, -1);

	JESD204_LNK_READ_CTRL_BITS_PER_SAMPLE(dev, np, &link->jesd204_link,
					      &link->jesd_param.jesd_cs, -1);

	JESD204_LNK_READ_NUM_LANES(dev, np, &link->jesd204_link,
				   &link->jesd_param.jesd_l, -1);

	JESD204_LNK_READ_VERSION(dev, np, &link->jesd204_link,
				 &link->jesd_param.jesd_jesdv, -1);

	JESD204_LNK_READ_SUBCLASS(dev, np, &link->jesd204_link,
				  &link->jesd_param.jesd_subclass, -1);

	ret = of_property_read_u32(np, "adi,link-mode", &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_mode_id = tmp;

	ret = of_property_read_u32(np, "adi,dual-link", &tmp);
	if (ret) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}
	link->jesd_param.jesd_duallink = tmp;

	ret = of_property_read_variable_u8_array(
		np, "adi,logical-lane-mapping", link->logiclane_mapping, 1,
		ARRAY_SIZE(link->logiclane_mapping));

	if (ret < 0) {
		dev_err(&phy->spi->dev,
			"Missing device tree property @ line %d ", __LINE__);
		return -EINVAL;
	}

	if (jtx) { /* JTX - for RX ADC path */
		ret = ad9081_get_jesd_converter_selection(
			phy, np, "adi,converter-select", link);

		if (ret < 0 || ret != link->jesd_param.jesd_m) {
			dev_err(&phy->spi->dev,
				"converter number mismatch (%d != %d)\n", ret,
				link->jesd_param.jesd_m);
			return -EINVAL;
		}
	} else { /* JRX */
		link->jrx_tpl_phase_adjust = 12;
		of_property_read_u32(np, "adi,tpl-phase-adjust",
				&link->jrx_tpl_phase_adjust);
	}

	return 0;
}

static int ad9081_reg_from_phandle(struct ad9081_phy *phy,
				   const struct device_node *np,
				   const char *phandle_name, int index,
				   u32 *reg)
{
	int ret;
	struct device_node *ph = of_parse_phandle(np, phandle_name, index);

	if (np == NULL)
		return -EINVAL;

	ret = of_property_read_u32(ph, "reg", reg);

	of_node_put(ph);

	return ret;
}

static void ad9081_dt_err(struct ad9081_phy *phy, const char *prop)
{
	dev_err(&phy->spi->dev, "Missing required dt property: '%s'\n", prop);
}

static int ad9081_parse_dt_tx(struct ad9081_phy *phy, struct device_node *np)
{
	struct device_node *of_channels, *of_chan;
	struct device_node *of_trx_path;
	u32 reg, index, tmp;
	int i, ret;

	of_trx_path = of_get_child_by_name(np, "adi,tx-dacs");
	if (of_trx_path == NULL)
		return -ENODEV;

	ret = of_property_read_u64(of_trx_path, "adi,dac-frequency-hz",
			     &phy->dac_frequency_hz);
	if (ret) {
		ad9081_dt_err(phy, "adi,dac-frequency-hz");
		of_node_put(of_trx_path);

		return ret;
	}

	phy->tx_ffh_hopf_via_gpio_en =
		of_property_read_bool(of_trx_path, "adi,ffh-hopf-via-gpio-enable");

	/* The 4 DAC Main Datapaths */

	of_channels = of_get_child_by_name(of_trx_path, "adi,main-data-paths");
	if (of_channels == NULL) {
		of_node_put(of_trx_path);
		return -ENODEV;
	}

	ret = of_property_read_u32(of_channels, "adi,interpolation",
			     &phy->tx_main_interp);
	if (ret) {
		ad9081_dt_err(phy, "adi,interpolation");
		of_node_put(of_channels);
		of_node_put(of_trx_path);

		return ret;
	}

	for_each_child_of_node(of_channels, of_chan) {
		ret = of_property_read_u32(of_chan, "reg", &reg);
		if (!ret && (reg < ARRAY_SIZE(phy->tx_main_shift))) {
			of_property_read_u64(of_chan,
					     "adi,nco-frequency-shift-hz",
					     &phy->tx_main_shift[reg]);
			of_property_read_u32(of_chan,
					     "adi,full-scale-current-ua",
					     &phy->tx_dac_fsc[reg]);

			ret = of_property_read_u32(of_chan,
					     "adi,maindp-dac-1x-non1x-crossbar-select",
					     &tmp);

			if (ret == 0 && tmp < 4)
				phy->tx_dac_chan_xbar_1x_non1x[reg] = 1 << tmp;
			else
				phy->tx_dac_chan_xbar_1x_non1x[reg] = 1 << reg;

			for (i = 0; i < ARRAY_SIZE(phy->tx_dac_chan_xbar);
				i++) {
				ret = ad9081_reg_from_phandle(phy, of_chan,
					"adi,crossbar-select", i, &index);
				if (ret)
					break;

				if (index >= MAX_NUM_CHANNELIZER) {
					dev_err(&phy->spi->dev,
						"invalid device tree configuration: index (%d > %d)\n",
						index, MAX_NUM_CHANNELIZER);
					of_node_put(of_channels);
					of_node_put(of_trx_path);

					return -EINVAL;
				}

				phy->tx_dac_chan_xbar[reg] |= BIT(index);
			}
		}
	}

	of_node_put(of_channels);

	/* The 8 DAC Channelizers */

	of_channels =
		of_get_child_by_name(of_trx_path, "adi,channelizer-paths");
	if (of_channels == NULL) {
		of_node_put(of_trx_path);
		return -ENODEV;
	}

	ret = of_property_read_u32(of_channels, "adi,interpolation",
			     &phy->tx_chan_interp);
	if (ret) {
		ad9081_dt_err(phy, "adi,interpolation");
		of_node_put(of_channels);
		of_node_put(of_trx_path);

		return ret;
	}

	for_each_child_of_node(of_channels, of_chan) {
		ret = of_property_read_u32(of_chan, "reg", &reg);
		if (!ret && (reg < ARRAY_SIZE(phy->tx_chan_shift))) {
			u32 val;

			of_property_read_u64(of_chan,
					     "adi,nco-frequency-shift-hz",
					     &phy->tx_chan_shift[reg]);
			ret = of_property_read_u32(of_chan, "adi,gain", &val);
			if (!ret)
				phy->dac_cache.chan_gain[reg] = val;

			phy->dac_cache.enable[reg] = 1; /* powers up enabled */
		}
	}

	of_node_put(of_channels);

	of_channels = of_get_child_by_name(of_trx_path, "adi,jesd-links");
	if (of_channels == NULL) {
		of_node_put(of_trx_path);
		return -ENODEV;
	}

	ret = of_property_read_u32(of_channels, "adi,lane-invert-mask", &tmp);
	if (!ret)
		phy->ad9081.serdes_info.des_settings.invert_mask = tmp;

	ret = of_property_read_u32(of_channels, "adi,lane-boost-mask", &tmp);
	if (!ret)
		phy->ad9081.serdes_info.des_settings.boost_mask = tmp;

	of_property_read_variable_u8_array(
		of_channels, "adi,ctle-filter-settings",
		phy->ad9081.serdes_info.des_settings.ctle_filter, 1,
		ARRAY_SIZE(phy->ad9081.serdes_info.des_settings.ctle_filter));


	phy->jrx_link_watchdog_en = of_property_read_bool(of_channels,
		"adi,jrx-link-watchdog-enable");

	for_each_child_of_node(of_channels, of_chan) {
		ret = of_property_read_u32(of_chan, "reg", &reg);
		if (!ret && (reg < ARRAY_SIZE(phy->jrx_link_tx))) {
			ad9081_parse_jesd_link_dt(phy, of_chan,
						  &phy->jrx_link_tx[reg], false);
		} else {
			dev_err(&phy->spi->dev,
				"Missing or invalid reg property in tx jesd-links node (%d)\n",
				reg);
			of_node_put(of_chan);
			break;
		}
	}

	of_node_put(of_channels);
	of_node_put(of_trx_path);

	return ret;
}

static int ad9081_parse_dt_rx(struct ad9081_phy *phy, struct device_node *np)
{
	struct device_node *of_channels, *of_chan;
	struct device_node *of_trx_path;
	u32 reg, tmp, nz;
	int ret, i;
	u8 lane_cfg[8];

	/* The 4 ADC Main Datapaths */

	of_trx_path = of_get_child_by_name(np, "adi,rx-adcs");
	if (of_trx_path == NULL)
		return -ENODEV;

	ret = of_property_read_u64(of_trx_path, "adi,adc-frequency-hz",
			     &phy->adc_frequency_hz);
	if (ret) {
		ad9081_dt_err(phy, "adi,adc-frequency-hz");
		of_node_put(of_trx_path);

		return ret;
	}

	nz = AD9081_ADC_NYQUIST_ZONE_ODD;
	of_property_read_u32(of_trx_path, "adi,nyquist-zone", &nz);

	of_property_read_variable_u8_array(of_trx_path,
		"adi,ffh-gpio-mux-sel", phy->rx_ffh_gpio_mux_sel,
		ARRAY_SIZE(phy->rx_ffh_gpio_mux_sel),
		ARRAY_SIZE(phy->rx_ffh_gpio_mux_sel));

	/* The 4 DAC Main Datapaths */

	of_channels = of_get_child_by_name(of_trx_path, "adi,main-data-paths");
	if (of_channels == NULL) {
		of_node_put(of_trx_path);
		return -ENODEV;
	}

	for_each_child_of_node(of_channels, of_chan) {
		ret = of_property_read_u32(of_chan, "reg", &reg);
		if (!ret && (reg < ARRAY_SIZE(phy->tx_main_shift))) {
			of_property_read_u64(of_chan,
					     "adi,nco-frequency-shift-hz",
					     &phy->rx_cddc_shift[0][reg]);
			of_property_read_u8(of_chan,
					     "adi,nco-channel-select-mode",
					     &phy->rx_cddc_nco_channel_select_mode[reg]);
			ret = of_property_read_u32(of_chan, "adi,decimation",
					     &phy->adc_main_decimation[reg]);
			phy->adc_invert_en[reg] = of_property_read_bool(of_chan,
				"adi,adc-invert-en");
			if (ret) {
				ad9081_dt_err(phy, "adi,decimation");
				of_node_put(of_channels);
				of_node_put(of_trx_path);

				return ret;
			}

			phy->rx_cddc_c2r[reg] = of_property_read_bool(
				of_chan, "adi,complex-to-real-enable");
			phy->rx_cddc_gain_6db_en[reg] = of_property_read_bool(
				of_chan, "adi,digital-gain-6db-enable");
			phy->rx_cddc_select |= BIT(reg);
			ret = of_property_read_u32(of_trx_path,
				"adi,nyquist-zone", &tmp);
			if (!ret)
				phy->rx_nyquist_zone[reg] = tmp;
			else
				phy->rx_nyquist_zone[reg] = nz;
		}
	}

	of_node_put(of_channels);

	/* The 8 ADC Channelizers */

	of_channels =
		of_get_child_by_name(of_trx_path, "adi,channelizer-paths");
	if (of_channels == NULL) {
		of_node_put(of_trx_path);
		return -ENODEV;
	}

	for_each_child_of_node(of_channels, of_chan) {
		ret = of_property_read_u32(of_chan, "reg", &reg);
		if (!ret && (reg < ARRAY_SIZE(phy->rx_fddc_shift))) {
			u32 mode;

			ret = of_property_read_u32(of_chan, "adi,decimation",
					     &phy->adc_chan_decimation[reg]);
			if (ret) {
				ad9081_dt_err(phy, "adi,decimation");
				of_node_put(of_channels);
				of_node_put(of_trx_path);

				return ret;
			}

			of_property_read_u64(of_chan,
					     "adi,nco-frequency-shift-hz",
					     &phy->rx_fddc_shift[reg]);
			phy->rx_fddc_c2r[reg] = of_property_read_bool(
				of_chan, "adi,complex-to-real-enable");
			mode = AD9081_ADC_NCO_VIF;
			of_property_read_u32(of_chan, "adi,nco-mixer-mode",
					     &mode);
			phy->rx_fddc_mxr_if[reg] = mode;
			phy->rx_fddc_gain_6db_en[reg] = of_property_read_bool(
				of_chan, "adi,digital-gain-6db-enable");
			phy->rx_fddc_select |= BIT(reg);
		}
	}

	of_node_put(of_channels);

	of_channels = of_get_child_by_name(of_trx_path, "adi,jesd-links");
	if (of_channels == NULL) {
		of_node_put(of_trx_path);
		return -ENODEV;
	}

	ret = of_property_read_u32(of_channels, "adi,lane-invert-mask", &tmp);
	if (!ret)
		phy->ad9081.serdes_info.ser_settings.invert_mask = tmp;

	ret = of_property_read_variable_u8_array(of_channels,
		"adi,lane-swing-settings", lane_cfg, ARRAY_SIZE(lane_cfg),
		ARRAY_SIZE(lane_cfg));

	if (ret > 0)
		for (i = 0; i < ARRAY_SIZE(lane_cfg); i++)
			phy->ad9081.serdes_info.ser_settings.lane_settings[i].swing_setting =
				lane_cfg[i];

	ret = of_property_read_variable_u8_array(of_channels,
		"adi,lane-pre-emp-settings", lane_cfg, ARRAY_SIZE(lane_cfg),
		ARRAY_SIZE(lane_cfg));

	if (ret > 0)
		for (i = 0; i < ARRAY_SIZE(lane_cfg); i++)
			phy->ad9081.serdes_info.ser_settings.lane_settings[i].pre_emp_setting =
				lane_cfg[i];

	ret = of_property_read_variable_u8_array(of_channels,
		"adi,lane-post-emp-settings", lane_cfg, ARRAY_SIZE(lane_cfg),
		ARRAY_SIZE(lane_cfg));

	if (ret > 0)
		for (i = 0; i < ARRAY_SIZE(lane_cfg); i++)
			phy->ad9081.serdes_info.ser_settings.lane_settings[i].post_emp_setting =
				lane_cfg[i];

	for_each_child_of_node(of_channels, of_chan) {
		ret = of_property_read_u32(of_chan, "reg", &reg);
		if (!ret && (reg < ARRAY_SIZE(phy->jtx_link_rx))) {
			ad9081_parse_jesd_link_dt(phy, of_chan,
						  &phy->jtx_link_rx[reg], true);
		} else {
			dev_err(&phy->spi->dev,
				"Missing or invalid reg property in rx jesd-links node (%d)\n",
				reg);
			of_node_put(of_chan);
			break;
		}
	}

	of_node_put(of_channels);
	of_node_put(of_trx_path);

	return ret;
}

static int ad9081_parse_dt(struct ad9081_phy *phy, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;

	phy->standalone = of_property_read_bool(np, "adi,standalone-enable");

	phy->multidevice_instance_count = 1;
	of_property_read_u32(np, "adi,multidevice-instance-count",
			     &phy->multidevice_instance_count);

	phy->dual_link_use_own_tpl_en =
		of_property_read_bool(np, "adi,dual-link-use-separate-tpl-enable");

	phy->config_sync_01_swapped =
		of_property_read_bool(np, "adi,jesd-sync-pins-01-swap-enable");

	phy->config_sync_0a_cmos_en =
		of_property_read_bool(np, "adi,jesd-sync-pin-0a-cmos-enable");

	phy->lmfc_delay = 0;
	of_property_read_u32(np, "adi,lmfc-delay-dac-clk-cycles",
			&phy->lmfc_delay);

	phy->nco_sync_ms_extra_lmfc_num = 0;
	of_property_read_u32(np, "adi,nco-sync-ms-extra-lmfc-num",
			&phy->nco_sync_ms_extra_lmfc_num);

	phy->nco_sync_direct_sysref_mode_en =
		of_property_read_bool(np,
			"adi,nco-sync-direct-sysref-mode-enable");

	phy->sysref_average_cnt_exp = 7;
	of_property_read_u32(np, "adi,sysref-average-cnt-exp",
			&phy->sysref_average_cnt_exp);

	phy->sysref_coupling_ac_en = of_property_read_bool(np,
		"adi,sysref-ac-coupling-enable");

	phy->sysref_cmos_input_en = of_property_read_bool(np,
		"adi,sysref-cmos-input-enable");

	phy->sysref_cmos_single_end_term_pos = 1; /* 6.3k */
	of_property_read_u8(np, "adi,sysref-single-end-pos-termination",
			&phy->sysref_cmos_single_end_term_pos);

	phy->sysref_cmos_single_end_term_neg = 15; /* 6.4k */
	of_property_read_u8(np, "adi,sysref-single-end-pos-termination",
			&phy->sysref_cmos_single_end_term_neg);

	phy->sysref_continuous_dis =
		of_property_read_bool(np,
			"adi,continuous-sysref-mode-disable");

	phy->direct_lb_map = 0x44;
	of_property_read_u8(np, "adi,direct-loopback-mode-dac-adc-mapping",
			&phy->direct_lb_map);

	of_property_read_u8(np, "adi,master-slave-sync-gpio-num",
			&phy->sync_ms_gpio_num);

	ret = ad9081_parse_dt_tx(phy, np);
	if (ret == -ENODEV && phy->dac_frequency_hz) {
		phy->tx_disable = true;
		dev_info(&phy->spi->dev, "Disabling TX side\n");
	} else if (ret < 0) {
		dev_err(&phy->spi->dev, "failed to parse devicetree");
		return ret;
	}

	ret = ad9081_parse_dt_rx(phy, np);
	if (ret == -ENODEV && phy->adc_frequency_hz) {
		phy->rx_disable = true;
		dev_info(&phy->spi->dev, "Disabling RX side\n");
	} else if (ret < 0) {
		dev_err(&phy->spi->dev, "failed to parse devicetree");
		return ret;
	}

	/* More here */

	return 0;
}

static char* ad9081_lable_writer(struct ad9081_phy *phy, const struct iio_chan_spec *chan)
{
	struct axiadc_converter *conv = spi_get_drvdata(phy->spi);
	u8 adc_num, cddc_num, cddc_mask, fddc_num, fddc_mask;

	ad9081_iiochan_to_fddc_cddc(phy, chan, &fddc_num, &fddc_mask, &cddc_num, &cddc_mask);

	if (chan->output) {
		snprintf(phy->tx_chan_labels[fddc_num], sizeof(phy->tx_chan_labels[0]),
			"FDUC%u->CDUC%u->DAC%u", fddc_num, cddc_num, cddc_num);

		return phy->tx_chan_labels[fddc_num];

	}

	switch (conv->id) {
	case CHIPID_AD9082:
	case CHIPID_AD9986:
	case CHIPID_AD9207:
		if (cddc_num == 0 || cddc_num == 2)
			adc_num = 0;
		else
			adc_num = 1;
		break;
	default:
		adc_num = cddc_num;
	}

	snprintf(phy->rx_chan_labels[fddc_num], sizeof(phy->rx_chan_labels[0]),
		"FDDC%u->CDDC%u->ADC%u", fddc_num, cddc_num, adc_num);

	return phy->rx_chan_labels[fddc_num];
}

static int ad9081_setup_chip_info_tbl(struct ad9081_phy *phy,
				      bool complex_rx, bool complex_tx, bool buffer_capable)
{
	int i, c, m;

	m = phy->jtx_link_rx[0].jesd_param.jesd_m +
	    phy->jtx_link_rx[1].jesd_param.jesd_m;

	switch (m) {
	case 0:
	case 2:
	case 4:
	case 6:
	case 8:
	case 12:
	case 16:
		break;
	default:
		return -EINVAL;
	}

	phy->rx_labels = devm_kcalloc(&phy->spi->dev,
				   m * phy->multidevice_instance_count,
				   sizeof(*phy->rx_labels), GFP_KERNEL);
	if (!phy->rx_labels)
		return -ENOMEM;

	for (c = 0, i = 0; i < (m * phy->multidevice_instance_count);
		 i++, c++) {
		phy->chip_info.channel[c].type = IIO_VOLTAGE;
		phy->chip_info.channel[c].output = 0;
		phy->chip_info.channel[c].indexed = 1;
		phy->chip_info.channel[c].modified = complex_rx ? 1 : 0;
		phy->chip_info.channel[c].channel = complex_rx ? i / 2 : i;
		phy->chip_info.channel[c].channel2 =
			(i & 1) ? IIO_MOD_Q : IIO_MOD_I;

		phy->chip_info.channel[c].scan_index = buffer_capable ? i : -1;

		if (phy->dual_link_use_own_tpl_en &&
			(i >= (phy->jtx_link_rx[0].jesd_param.jesd_m *
			phy->multidevice_instance_count)))
			phy->chip_info.channel[c].scan_index = -1;

		phy->chip_info.channel[c].address = i,
		phy->chip_info.channel[c].info_mask_shared_by_type =
			BIT(IIO_CHAN_INFO_SAMP_FREQ);

		phy->chip_info.channel[c].scan_type.realbits =
			phy->jtx_link_rx[0].jesd_param.jesd_n;
		phy->chip_info.channel[c].scan_type.storagebits =
			(phy->jtx_link_rx[0].jesd_param.jesd_np > 8) ? 16 : 8;
		phy->chip_info.channel[c].scan_type.sign = 's';

		if (i < m) {
			phy->chip_info.channel[c].ext_info = rxadc_ext_info;
			phy->rx_labels[phy->chip_info.channel[c].channel] =
				ad9081_lable_writer(phy, &phy->chip_info.channel[c]);
		} else {
			phy->rx_labels[phy->chip_info.channel[c].channel] = "buffer_only";
		}
	}

	m = phy->jrx_link_tx[0].jesd_param.jesd_m *
		(ad9081_link_is_dual(phy->jrx_link_tx) ? 2 : 1);

	phy->tx_labels = devm_kcalloc(&phy->spi->dev, m,
		sizeof(*phy->tx_labels), GFP_KERNEL);
	if (!phy->tx_labels)
		return -ENOMEM;

	for (i = 0; i < m; i++, c++) {
		phy->chip_info.channel[c].type = IIO_VOLTAGE;
		phy->chip_info.channel[c].output = 1;
		phy->chip_info.channel[c].indexed = 1;
		phy->chip_info.channel[c].modified = complex_tx ? 1 : 0;
		phy->chip_info.channel[c].channel = complex_tx ? i / 2 : i;
		phy->chip_info.channel[c].channel2 =
			(i & 1) ? IIO_MOD_Q : IIO_MOD_I;
		phy->chip_info.channel[c].scan_index = -1;
		phy->chip_info.channel[c].info_mask_shared_by_type =
			BIT(IIO_CHAN_INFO_SAMP_FREQ);

		phy->chip_info.channel[c].info_mask_separate =
			BIT(IIO_CHAN_INFO_ENABLE);

		phy->chip_info.channel[c].ext_info = txdac_ext_info;
		phy->tx_labels[phy->chip_info.channel[c].channel] =
			ad9081_lable_writer(phy, &phy->chip_info.channel[c]);
	}

	phy->chip_info.channel[c].type = IIO_TEMP;
	phy->chip_info.channel[c].indexed = 1;
	phy->chip_info.channel[c].info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED);
	phy->chip_info.channel[c].scan_index = -1;
	c++;

	phy->chip_info.num_channels = c;
	phy->chip_info.name = "AD9081";
	phy->chip_info.max_rate = 3000000000UL;

	return 0;
}

static const struct iio_info ad9081_iio_info = {
	.read_raw = &ad9081_read_raw,
	.write_raw = &ad9081_write_raw,
	.read_label = &ad9081_read_label,
	.debugfs_reg_access = &ad9081_reg_access,
	.attrs = &ad9081_phy_attribute_group,
};

static int ad9081_register_iiodev(struct axiadc_converter *conv)
{
	struct iio_dev *indio_dev;
	struct spi_device *spi = conv->spi;
	struct ad9081_phy *phy = conv->phy;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, 0);
	if (!indio_dev)
		return -ENOMEM;

	iio_device_set_drvdata(indio_dev, conv);

	indio_dev->dev.parent = &spi->dev;

	if (spi->dev.of_node)
		indio_dev->name = spi->dev.of_node->name;
	else
		indio_dev->name = spi_get_device_id(spi)->name;

	mutex_init(&conv->lock);
	indio_dev->info = &ad9081_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = phy->chip_info.channel;
	indio_dev->num_channels = phy->chip_info.num_channels;

	ret = iio_device_register(indio_dev);
	ad9081_post_iio_register(indio_dev);

	conv->indio_dev = indio_dev;

	return ret;
}

static int ad9081_jesd204_link_init(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9081_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9081_phy *phy = priv->phy;
	struct ad9081_jesd_link *link;
	int ret;

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	switch (lnk->link_id) {
	case DEFRAMER_LINK0_TX:
	case DEFRAMER_LINK1_TX:
		if (phy->tx_disable)
			return -ENODEV;
		link = &phy->jrx_link_tx[0];
		lnk->sample_rate = phy->dac_frequency_hz;
		lnk->sample_rate_div = phy->tx_main_interp * phy->tx_chan_interp;
		break;
	case FRAMER_LINK0_RX:
	case FRAMER_LINK1_RX:
		if (phy->rx_disable)
			return -ENODEV;
		link = &phy->jtx_link_rx[lnk->link_id - FRAMER_LINK0_RX];
		lnk->sample_rate = phy->adc_frequency_hz;
		lnk->sample_rate_div = phy->adc_dcm[lnk->link_id - FRAMER_LINK0_RX];
		break;
	default:
		return -EINVAL;
	}

	jesd204_copy_link_params(lnk, &link->jesd204_link);

	if (lnk->jesd_version == JESD204_VERSION_C)
		lnk->jesd_encoder = JESD204_ENCODER_64B66B;
	else
		lnk->jesd_encoder = JESD204_ENCODER_8B10B;

	ret = jesd204_link_get_rate_khz(lnk, &link->lane_rate_kbps);
	if (ret)
		return ret;

	if (phy->sysref_continuous_dis) {
		lnk->sysref.mode = JESD204_SYSREF_ONESHOT;
		phy->ad9081.clk_info.sysref_mode = SYSREF_ONESHOT;
	} else {
		lnk->sysref.mode = JESD204_SYSREF_CONTINUOUS;
		phy->ad9081.clk_info.sysref_mode = SYSREF_CONT;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9081_jesd204_clks_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9081_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9081_phy *phy = priv->phy;
	u8 jesd_pll_status;
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));


	if (reason == JESD204_STATE_OP_REASON_INIT) {
		ret = adi_ad9081_jesd_pll_lock_status_get(&phy->ad9081,
			&jesd_pll_status);
		if (ret != 0)
			return ret;

		if (!jesd_pll_status) {
			dev_err(dev, "JESD PLL Not Locked!\n");
			return -EFAULT;
		}
	}

	if (lnk->is_transmit && (reason == JESD204_STATE_OP_REASON_INIT) &&
		(lnk->jesd_version == JESD204_VERSION_C)) {

		if ((phy->jrx_link_tx[0].lane_rate_kbps >
			(AD9081_JESDRX_204C_CAL_THRESH / 1000)) &&
			phy->jrx_link_tx[0].lane_cal_rate_kbps !=
			phy->jrx_link_tx[0].lane_rate_kbps) {

			ret = adi_ad9081_jesd_rx_link_enable_set(&phy->ad9081,
				ad9081_link_sel(phy->jrx_link_tx), 1);
			if (ret != 0)
				return ret;

			dev_info(dev, "running jesd_rx_calibrate_204c, LR %lu kbps",
				phy->jrx_link_tx[0].lane_rate_kbps);

			ret = adi_ad9081_jesd_rx_calibrate_204c(&phy->ad9081, 1, 0, 1);
			if (ret < 0)
				return ret;

			ret = adi_ad9081_jesd_rx_link_enable_set(&phy->ad9081,
				ad9081_link_sel(phy->jrx_link_tx), 0);
			if (ret != 0)
				return ret;
		}

		phy->jrx_link_tx[0].lane_cal_rate_kbps = phy->jrx_link_tx[0].lane_rate_kbps;
	}

	if (!lnk->is_transmit) {
		/* txfe RX (JTX) link digital reset */
		ret = ad9081_jesd_tx_link_dig_reset(&phy->ad9081,
			reason != JESD204_STATE_OP_REASON_INIT);
		if (ret != 0)
			return ret;

		mdelay(4);
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9081_jesd204_link_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9081_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9081_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	if (lnk->is_transmit) {
		/* txfe TX (JRX) link */
		ret = adi_ad9081_jesd_rx_link_enable_set(&phy->ad9081,
			ad9081_link_sel(phy->jrx_link_tx),
			reason == JESD204_STATE_OP_REASON_INIT);
		if (ret != 0)
			return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9081_jesd204_link_running(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9081_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9081_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		if (lnk->is_transmit && phy->jrx_link_watchdog_en)
			cancel_delayed_work(&phy->dwork);

		phy->is_initialized = false;

		return JESD204_STATE_CHANGE_DONE;
	}

	if (lnk->is_transmit) {
		ret = ad9081_jesd_rx_link_status_print(phy, lnk, 3);
		if (ret < 0)
			return JESD204_STATE_CHANGE_ERROR;
	} else {
		ret = ad9081_jesd_tx_link_status_print(phy, lnk, 3);
		if (ret < 0)
			return JESD204_STATE_CHANGE_ERROR;
	}

	if (lnk->is_transmit && phy->jrx_link_watchdog_en)
		schedule_delayed_work(&phy->dwork, msecs_to_jiffies(1000));

	phy->is_initialized = true;

	/* Need to redo this since GPIOx might have been clobbered by master/slave sync */
	adi_ad9081_dac_duc_main_nco_hopf_gpio_as_hop_en_set(&phy->ad9081,
		phy->tx_ffh_hopf_via_gpio_en);

	if (phy->ms_sync_en_gpio)
		gpiod_set_value(phy->ms_sync_en_gpio, 0);

	return JESD204_STATE_CHANGE_DONE;
}

int ad9081_jesd204_uninit(struct jesd204_dev *jdev,
			    enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	if (reason != JESD204_STATE_OP_REASON_UNINIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9081_jesd204_setup_stage1(struct jesd204_dev *jdev,
					 enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9081_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9081_phy *phy = priv->phy;
	adi_cms_jesd_subclass_e subclass = JESD_SUBCLASS_0;
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		if (phy->ms_sync_en_gpio)
			gpiod_set_value(phy->ms_sync_en_gpio, 0);

		return JESD204_STATE_CHANGE_DONE;
	}

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	ret = ad9081_jesd_tx_link_dig_reset(&phy->ad9081, 0);
	if (ret != 0)
		return ret;

	mdelay(4);

	/* JESD OneShot Sync */
	ret = adi_ad9081_hal_bf_set(&phy->ad9081, REG_SYNC_DEBUG0_ADDR,
		BF_AVRG_FLOW_EN_INFO, 1);
	if (ret != 0)
		return ret;

	ret = adi_ad9081_hal_bf_set(&phy->ad9081, REG_SYSREF_AVERAGE_ADDR,
		BF_SYSREF_AVERAGE_INFO,
		BF_SYSREF_AVERAGE(phy->sysref_average_cnt_exp));
	if (ret != 0)
		return ret;

	if (phy->jrx_link_tx[0].jesd_param.jesd_subclass ||
		phy->jtx_link_rx[0].jesd_param.jesd_subclass)
		subclass = JESD_SUBCLASS_1;

	ret = adi_ad9081_jesd_oneshot_sync(&phy->ad9081, subclass);
	if (ret != 0)
		return ret;

	ret = adi_ad9081_hal_bf_set(&phy->ad9081, REG_SYNC_DEBUG0_ADDR,
		BF_AVRG_FLOW_EN_INFO, 0);
	if (ret != 0)
		return ret;

	ret = adi_ad9081_hal_bf_set(&phy->ad9081, REG_SYSREF_AVERAGE_ADDR,
		BF_SYSREF_AVERAGE_INFO,
		BF_SYSREF_AVERAGE(0));
	if (ret != 0)
		return ret;

	if (phy->ms_sync_en_gpio)
		gpiod_set_value(phy->ms_sync_en_gpio, 1);

	if (jesd204_dev_is_top(jdev)) {
		/* We need to make sure the master-slave master GPIO is enabled before we move on */
		ret = adi_ad9081_device_nco_sync_gpio_set(&phy->ad9081, phy->sync_ms_gpio_num, 1);
		if (ret != 0)
			return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9081_jesd204_setup_stage2(struct jesd204_dev *jdev,
					 enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9081_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9081_phy *phy = priv->phy;
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	/* NCO Sync */

	ret = ad9081_nco_sync(phy, jesd204_dev_is_top(jdev));
	if (ret != 0)
		return ret;

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9081_jesd204_setup_stage3(struct jesd204_dev *jdev,
					 enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9081_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9081_phy *phy = priv->phy;
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	ret = adi_ad9081_device_nco_sync_post(&phy->ad9081);
	if (ret != 0)
		return ret;

	ret = adi_ad9081_device_gpio_set_highz(&phy->ad9081, phy->sync_ms_gpio_num);
	if (ret != 0)
		return ret;

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_ad9081_init = {
	.state_ops = {
		[JESD204_OP_DEVICE_INIT] = {
			.per_device = ad9081_jesd204_uninit,
		},
		[JESD204_OP_LINK_INIT] = {
			.per_link = ad9081_jesd204_link_init,
		},
		[JESD204_OP_OPT_SETUP_STAGE1] = {
			.per_device = ad9081_jesd204_setup_stage1,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_OPT_SETUP_STAGE2] = {
			.per_device = ad9081_jesd204_setup_stage2,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
			.post_state_sysref = true,
		},
		[JESD204_OP_OPT_SETUP_STAGE3] = {
			.per_device = ad9081_jesd204_setup_stage3,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = ad9081_jesd204_clks_enable,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = ad9081_jesd204_link_enable,
			.post_state_sysref = true,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = ad9081_jesd204_link_running,
		},
	},

	.max_num_links = 4,
	.num_retries = 3,
	.sizeof_priv = sizeof(struct ad9081_jesd204_priv),
};

static irqreturn_t ad9081_irq_handler(int irq, void *p)
{
	struct axiadc_converter *conv = p;
	struct ad9081_phy *phy = conv->phy;
	u64 status64;

	adi_ad9081_dac_irqs_status_get(&phy->ad9081, &status64);
	dev_err(&phy->spi->dev, "DAC IRQ status 0x%llX\n", status64);

	return IRQ_HANDLED;
}

static void ad9081_reg_disable(void *data)
{
	regulator_disable(data);
}

static int ad9081_probe(struct spi_device *spi)
{
	struct axiadc_converter *conv;
	struct ad9081_phy *phy;
	struct jesd204_dev *jdev;
	struct ad9081_jesd204_priv *priv;
	struct gpio_desc *gpio;
	u8 api_rev[3];
	u32 spi_id;
	int ret;

	jdev = devm_jesd204_dev_register(&spi->dev, &jesd204_ad9081_init);
	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

	if (!jdev) {
		dev_err(&spi->dev, "Failed to register jesd204-fsm device");
		return -ENODEV;
	}

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	phy = devm_kzalloc(&spi->dev, sizeof(*phy), GFP_KERNEL);
	if (phy == NULL)
		return -ENOMEM;

	conv->adc_clkscale.mult = 1;
	conv->adc_clkscale.div = 1;

	spi_set_drvdata(spi, conv);
	conv->spi = spi;
	conv->phy = phy;
	phy->spi = spi;
	phy->jdev = jdev;
	priv = jesd204_dev_priv(jdev);
	priv->phy = phy;

	ret = ad9081_request_clks(conv);
	if (ret)
		return ret;

	phy->ad9081.hal_info.sdo = (spi->mode & SPI_3WIRE) ? SPI_SDIO : SPI_SDO;
	phy->ad9081.hal_info.msb =
		(spi->mode & SPI_LSB_FIRST) ? SPI_MSB_LAST : SPI_MSB_FIRST;
	phy->ad9081.hal_info.addr_inc = SPI_ADDR_INC_AUTO;
	phy->ad9081.hal_info.delay_us = ad9081_udelay;
	phy->ad9081.hal_info.spi_xfer = ad9081_spi_xfer;
	phy->ad9081.hal_info.reset_pin_ctrl = ad9081_reset_pin_ctrl;
	phy->ad9081.hal_info.user_data = conv;
	phy->ad9081.hal_info.log_write = ad9081_log_write;

	phy->ad9081.clk_info.sysref_ctrl = ad9081_sysref_ctrl;
	phy->ad9081.clk_info.sysref_clk = phy;

	phy->ad9081.serdes_info = (adi_ad9081_serdes_settings_t) {
		.ser_settings = { /* txfe jtx */
			.lane_settings = {
				{
					.swing_setting = AD9081_SER_SWING_850,
					.pre_emp_setting = AD9081_SER_PRE_EMP_0DB,
					.post_emp_setting = AD9081_SER_POST_EMP_0DB
				}, {
					.swing_setting = AD9081_SER_SWING_850,
					.pre_emp_setting = AD9081_SER_PRE_EMP_0DB,
					.post_emp_setting = AD9081_SER_POST_EMP_0DB
				}, {
					.swing_setting = AD9081_SER_SWING_850,
					.pre_emp_setting = AD9081_SER_PRE_EMP_0DB,
					.post_emp_setting = AD9081_SER_POST_EMP_0DB
				}, {
					.swing_setting = AD9081_SER_SWING_850,
					.pre_emp_setting = AD9081_SER_PRE_EMP_0DB,
					.post_emp_setting = AD9081_SER_POST_EMP_0DB
				}, {
					.swing_setting = AD9081_SER_SWING_850,
					.pre_emp_setting = AD9081_SER_PRE_EMP_0DB,
					.post_emp_setting = AD9081_SER_POST_EMP_0DB
				}, {
					.swing_setting = AD9081_SER_SWING_850,
					.pre_emp_setting = AD9081_SER_PRE_EMP_0DB,
					.post_emp_setting = AD9081_SER_POST_EMP_0DB
				}, {
					.swing_setting = AD9081_SER_SWING_850,
					.pre_emp_setting = AD9081_SER_PRE_EMP_0DB,
					.post_emp_setting = AD9081_SER_POST_EMP_0DB
				}, {
					.swing_setting = AD9081_SER_SWING_850,
					.pre_emp_setting = AD9081_SER_PRE_EMP_0DB,
					.post_emp_setting = AD9081_SER_POST_EMP_0DB
				},
			},
			.invert_mask = 0x00,
			.lane_mapping = {
				{ 0, 1, 2, 3, 4, 5, 6, 7 },
				{ 7, 7, 7, 7, 7, 7, 7, 7 }
			}, /* link0, link1 */
		},
		.des_settings = { /* txfe jrx */
			.boost_mask = 0xff,
			.invert_mask = 0x00,
			.ctle_filter = { 2, 2, 2, 2, 2, 2, 2, 2 },
			.lane_mapping =  {
				{ 0, 1, 2, 3, 4, 5, 6, 7 },
				{ 0, 1, 2, 3, 4, 5, 6, 7 }
			}, /* link0, link1 */
		}
	};

	conv->reset_gpio =
		devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(conv->reset_gpio))
		return PTR_ERR(conv->reset_gpio);

	phy->rx1_en_gpio =
		devm_gpiod_get_optional(&spi->dev, "rx1-enable", GPIOD_OUT_LOW);
	if (IS_ERR(phy->rx1_en_gpio))
		return PTR_ERR(phy->rx1_en_gpio);

	phy->rx2_en_gpio =
		devm_gpiod_get_optional(&spi->dev, "rx2-enable", GPIOD_OUT_LOW);
	if (IS_ERR(phy->rx2_en_gpio))
		return PTR_ERR(phy->rx2_en_gpio);

	phy->tx1_en_gpio =
		devm_gpiod_get_optional(&spi->dev, "tx1-enable", GPIOD_OUT_LOW);
	if (IS_ERR(phy->tx1_en_gpio))
		return PTR_ERR(phy->tx1_en_gpio);

	phy->tx2_en_gpio =
		devm_gpiod_get_optional(&spi->dev, "tx2-enable", GPIOD_OUT_LOW);
	if (IS_ERR(phy->tx2_en_gpio))
		return PTR_ERR(phy->tx2_en_gpio);

	phy->ms_sync_en_gpio =
		devm_gpiod_get_optional(&spi->dev, "ms-sync-enable", GPIOD_OUT_LOW);
	if (IS_ERR(phy->ms_sync_en_gpio))
		return PTR_ERR(phy->ms_sync_en_gpio);

	ret = ad9081_parse_dt(phy, &spi->dev);
	if (ret < 0) {
		dev_err(&spi->dev, "Parsing devicetree failed (%d)\n", ret);
		return -ENODEV;
	}

	phy->supply_reg = devm_regulator_get(&spi->dev, "vdd");
	if (IS_ERR(phy->supply_reg))
		return dev_err_probe(&spi->dev, PTR_ERR(phy->supply_reg),
				     "failed to get the vdd supply regulator\n");

	if (phy->supply_reg) {
		ret = regulator_enable(phy->supply_reg);
		if (ret) {
			dev_err(&spi->dev, "Failed to enable vdd supply voltage!\n");
			return ret;
		}

		ret = devm_add_action_or_reset(&spi->dev, ad9081_reg_disable, phy->supply_reg);
		if (ret)
			return ret;
	}

	ret = adi_ad9081_device_reset(&phy->ad9081,
		conv->reset_gpio ? AD9081_HARD_RESET_AND_INIT :
		AD9081_SOFT_RESET_AND_INIT);
	if (ret < 0) {
		dev_err(&spi->dev, "reset/init failed (%d)\n", ret);
		return -ENODEV;
	}

	ret = adi_ad9081_device_chip_id_get(&phy->ad9081, &phy->chip_id);
	if (ret < 0) {
		dev_err(&spi->dev, "chip_id failed (%d)\n", ret);
		return -ENODEV;
	}

	conv->id = phy->chip_id.prod_id;

	if (!phy->rx_disable)
		ad9081_clk_register(phy, "-rx_sampl_clk",
			__clk_get_name(phy->dev_clk), NULL,
			CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			RX_SAMPL_CLK);

	if (!phy->tx_disable)
		ad9081_clk_register(phy, "-tx_sampl_clk",
			__clk_get_name(phy->dev_clk), NULL,
			CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
			TX_SAMPL_CLK);

	if (!phy->rx_disable && ad9081_link_is_dual(phy->jtx_link_rx))
		ad9081_clk_register(phy, "-rx_sampl_clk_link2",
				__clk_get_name(phy->dev_clk), NULL,
				CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED,
				RX_SAMPL_CLK_LINK2);

	phy->clk_data.clks = phy->clks;
	phy->clk_data.clk_num = NUM_AD9081_CLKS;

	ret = of_clk_add_provider(spi->dev.of_node, of_clk_src_onecell_get,
				  &phy->clk_data);

	INIT_DELAYED_WORK(&phy->dwork, ad9081_work_func);

	switch (conv->id) {
	case CHIPID_AD9081:
	case CHIPID_AD9082:
	case CHIPID_AD9988:
	case CHIPID_AD9986:
	case CHIPID_AD9177:
	case CHIPID_AD9207:
	case CHIPID_AD9209:
		spi_id = spi_get_device_id(spi)->driver_data & CHIPID_MASK;

		if (conv->id != spi_id)
			dev_warn(&spi->dev, "Expected AD%X found AD%X\n",
				spi_id, conv->id);

		ret = ad9081_setup(spi);
		if (ret)
			break;
		conv->chip_info = &phy->chip_info;
		ret = ad9081_setup_chip_info_tbl(phy, true, true,
			// (phy->adc_dcm[0] == 1) ? false : true,
			// (phy->tx_main_interp == 1) ? false : true,
			jesd204_dev_is_top(jdev));
		if (ret)
			break;
		break;
	default:
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
		return -ENODEV;
	}

	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&spi->dev, "Failed to initialize: %d\n", ret);
		goto out_clk_del_provider;
	}

	conv->clk = phy->clks[RX_SAMPL_CLK];
	conv->reg_access = ad9081_reg_access;
	conv->write_raw = ad9081_write_raw;
	conv->read_raw = ad9081_read_raw;
	conv->read_label = ad9081_read_label;
#if 0
	conv->read_event_value = ad9081_read_thresh,
	conv->write_event_value = ad9081_write_thresh,
	conv->read_event_config = ad9081_read_thresh_en,
	conv->write_event_config = ad9081_write_thresh_en,
	conv->set_pnsel = ad9081_set_pnsel;
#endif
	conv->post_setup = ad9081_post_setup;
	conv->post_iio_register = ad9081_post_iio_register;

	conv->attrs = &ad9081_phy_attribute_group;

	if (phy->standalone || !jesd204_dev_is_top(jdev)) {
		ret = ad9081_register_iiodev(conv);
		if (ret)
			goto out_clk_del_provider;
	}

	ret = ad9081_request_fd_irqs(conv);
	if (ret < 0)
		dev_warn(&spi->dev,
			 "Failed to request FastDetect IRQs (%d)", ret);

	gpio = devm_gpiod_get(&spi->dev, "irqb0", GPIOD_IN);
	if (0 && !IS_ERR(gpio)) { /* REVIST: Not yet used */
		int irq = gpiod_to_irq(gpio);

		if (irq >= 0) {
			ret = devm_request_threaded_irq(&spi->dev, irq, NULL,
				ad9081_irq_handler,
				IRQF_TRIGGER_FALLING  | IRQF_ONESHOT,
				spi->dev.of_node ? spi->dev.of_node->name :
				spi_get_device_id(spi)->name, conv);

			if (ret) {
				dev_err(&spi->dev,
					"request_irq() failed with %d\n", ret);
				goto out_clk_del_provider;
			}
		}
	}

	adi_ad9081_device_api_revision_get(&phy->ad9081, &api_rev[0],
					   &api_rev[1], &api_rev[2]);

	dev_info(&spi->dev, "AD%X Rev. %u Grade %u (API %u.%u.%u) probed\n",
		 conv->id, phy->chip_id.dev_revision,
		 phy->chip_id.prod_grade, api_rev[0], api_rev[1], api_rev[2]);

	ret = jesd204_fsm_start(jdev, JESD204_LINKS_ALL);
	if (ret)
		goto out_clk_del_provider;

	return 0;

out_clk_del_provider:
	of_clk_del_provider(spi->dev.of_node);

	return ret;
}

static void ad9081_remove(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct ad9081_phy *phy = conv->phy;

	jesd204_fsm_stop(phy->jdev, JESD204_LINKS_ALL);

	cancel_delayed_work_sync(&phy->dwork);

	if (phy->standalone || !jesd204_dev_is_top(phy->jdev))
		iio_device_unregister(conv->indio_dev);

	clk_disable_unprepare(phy->dev_clk);
	of_clk_del_provider(spi->dev.of_node);
	adi_ad9081_device_deinit(&phy->ad9081);
}

static const struct spi_device_id ad9081_id[] = {
	{ "ad9081", CHIPID_AD9081 },
	{ "ad9082", CHIPID_AD9082 },
	{ "ad9988", CHIPID_AD9988 },
	{ "ad9986", CHIPID_AD9986 },
	{ "ad9177", CHIPID_AD9177 },
	{ "ad9207", CHIPID_AD9207 },
	{ "ad9209", CHIPID_AD9209 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad9081_id);

static const struct of_device_id ad9081_of_match[] = {
	{ .compatible = "adi,ad9081" },
	{ .compatible = "adi,ad9082" },
	{ .compatible = "adi,ad9988" },
	{ .compatible = "adi,ad9986" },
	{ .compatible = "adi,ad9177" },
	{ .compatible = "adi,ad9207" },
	{ .compatible = "adi,ad9209" },
	{},
};
MODULE_DEVICE_TABLE(of, ad9081_of_match);

static struct spi_driver ad9081_driver = {
	.driver = {
			.name = "ad9081",
			.of_match_table = of_match_ptr(ad9081_of_match),
		},
	.probe = ad9081_probe,
	.remove = ad9081_remove,
	.id_table = ad9081_id,
};
module_spi_driver(ad9081_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9081 ADC");
MODULE_LICENSE("GPL v2");
