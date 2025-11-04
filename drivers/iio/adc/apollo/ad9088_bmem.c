// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AD9088 BMEM (Buffer Memory) interface
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include "ad9088.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/buffer_impl.h>

#include "public/inc/adi_apollo_bmem.h"
#include "public/inc/adi_apollo_bmem_types.h"
#include "adi_apollo_bf_rx_bmem.h"

#define AD9088_BMEM_SRAM_SIZE_BYTES	(128 * 1024)	/* 128KB SRAM */
#define AD9088_BMEM_SRAM_SIZE_WORDS	(32 * 1024)	/* 32K 32-bit words */
#define AD9088_BMEM_SAMPLE_SIZE_16BIT	(16)
#define AD9088_BMEM_SAMPLE_SIZE_32BIT	(32)

#define TIMEOUT_US 100000
#define POLL_DELAY_US 10

#define MAX_NUM_CHANNELS 4

struct ad9088_bmem_state {
	struct device		*dev;
	struct mutex		lock;
	struct completion	complete;
	struct ad9088_phy	*phy;
	struct delayed_work	capture_work;
	struct iio_dev		*indio_dev;

	u32			bmem_sel;
	u32			bmem_loc;
	u32			sample_size;
	u16			start_addr;
	u16			end_addr;

	u32			sample_count;
	u32			delay_sample[MAX_NUM_CHANNELS];

	/* Delay sample and hop configurations per channel */
	adi_apollo_bmem_delay_sample_t delay_sample_config[MAX_NUM_CHANNELS];
	adi_apollo_bmem_delay_hop_t delay_hop_config[MAX_NUM_CHANNELS];

	adi_apollo_bmem_capture_t capture_config;

	/* Per-channel buffers for captured data */
	__le32 *channel_buffers[4];  /* One buffer per channel */

	/* Scan buffer for pushing to IIO (interleaved samples + timestamp) */
	u8 scan_data[ALIGN(MAX_NUM_CHANNELS * sizeof(u16), sizeof(s64)) + sizeof(s64)] __aligned(IIO_DMA_MINALIGN);

	/* Track which channels are enabled for capture */
	unsigned long		active_channels;
};

static adi_apollo_bmem_sel_e ad9088_channel_to_bmem(int channel)
{
	switch (channel) {
	case 0:
		return ADI_APOLLO_BMEM_A0;
	case 1:
		return ADI_APOLLO_BMEM_A1;
	case 2:
		return ADI_APOLLO_BMEM_B0;
	case 3:
		return ADI_APOLLO_BMEM_B1;
	default:
		return ADI_APOLLO_BMEM_A0;
	}
}

static uint32_t calc_bmem_sram_base(int32_t bmem_index)
{
	static uint32_t bmem_regmap[ADI_APOLLO_BMEM_NUM] = {
		0x60240000, 0x60440000,
		0x60240000, 0x60440000,
		0x60A40000, 0x60C40000,
		0x60A40000, 0x60C40000
	};

	return bmem_regmap[bmem_index];
}

static uint32_t calc_bmem_hsdin_base(int32_t bmem_index)
{
	static uint32_t bmem_regmap[ADI_APOLLO_BMEM_NUM] = {
		RX_BMEM0_REG_RX_SLICE_0_RX_DIGITAL0, RX_BMEM0_REG_RX_SLICE_1_RX_DIGITAL0,
		RX_BMEM0_REG_RX_SLICE_0_RX_DIGITAL0, RX_BMEM0_REG_RX_SLICE_1_RX_DIGITAL0,
		RX_BMEM0_REG_RX_SLICE_0_RX_DIGITAL1, RX_BMEM0_REG_RX_SLICE_1_RX_DIGITAL1,
		RX_BMEM0_REG_RX_SLICE_0_RX_DIGITAL1, RX_BMEM0_REG_RX_SLICE_1_RX_DIGITAL1
	};

	return bmem_regmap[bmem_index];
}

static int32_t rx_mux1_config(adi_apollo_device_t *device)
{
	int32_t err = API_CMS_ERROR_OK;

	adi_apollo_rx_mux0_sel_e cbout_to_adc_4t4r_a[] = {ADI_APOLLO_4T4R_CB_OUT_0_FROM_ADC0, ADI_APOLLO_4T4R_CB_OUT_1_FROM_ADC1}; // Ave Pair: ADCA0->CB0 + ADCA1->CB1 (Side A)
	adi_apollo_rx_mux0_sel_e cbout_to_adc_4t4r_b[] = {ADI_APOLLO_4T4R_CB_OUT_0_FROM_ADC0, ADI_APOLLO_4T4R_CB_OUT_1_FROM_ADC1}; // Ave Pair: ADCB0->CB0 + ADCB1->CB1 (Side B)
	adi_apollo_rx_mux0_sel_e cbout_to_adc_8t8r_a[] = {ADI_APOLLO_8T8R_CB_OUT_FROM_ADC0, ADI_APOLLO_8T8R_CB_OUT_FROM_ADC1,      // Ave Pair: ADCA0->CB0 + ADCA1->CB1
							  ADI_APOLLO_8T8R_CB_OUT_FROM_ADC2, ADI_APOLLO_8T8R_CB_OUT_FROM_ADC3
							 };     // Ave Pair: ADCA2->CB2 + ADCA3->CB3 (Side A)
	adi_apollo_rx_mux0_sel_e cbout_to_adc_8t8r_b[] = {ADI_APOLLO_8T8R_CB_OUT_FROM_ADC0, ADI_APOLLO_8T8R_CB_OUT_FROM_ADC1,      // Ave Pair: ADCB0->CB0 + ADCB1->CB1
							  ADI_APOLLO_8T8R_CB_OUT_FROM_ADC2, ADI_APOLLO_8T8R_CB_OUT_FROM_ADC3
							 };     // Ave Pair: ADCB2->CB2 + ADCB3->CB3 (Side B)

	if (device->dev_info.is_8t8r) {
		err = adi_apollo_rxmux_xbar1_set(device, ADI_APOLLO_SIDE_A, cbout_to_adc_8t8r_a, ADI_APOLLO_RX_MUX0_NUM_8T8R);
		ADI_CMS_ERROR_RETURN(err);
		err = adi_apollo_rxmux_xbar1_set(device, ADI_APOLLO_SIDE_B, cbout_to_adc_8t8r_b, ADI_APOLLO_RX_MUX0_NUM_8T8R);
		ADI_CMS_ERROR_RETURN(err);
	} else {
		err = adi_apollo_rxmux_xbar1_set(device, ADI_APOLLO_SIDE_A, cbout_to_adc_4t4r_a, ADI_APOLLO_RX_MUX0_NUM_4T4R);
		ADI_CMS_ERROR_RETURN(err);
		err = adi_apollo_rxmux_xbar1_set(device, ADI_APOLLO_SIDE_B, cbout_to_adc_4t4r_b, ADI_APOLLO_RX_MUX0_NUM_4T4R);
		ADI_CMS_ERROR_RETURN(err);
	}

	return err;
}

static int ad9088_hsdin_bmem_capture_run(adi_apollo_device_t *device, adi_apollo_blk_sel_t bmems)
{
	int32_t err;
	uint8_t i;
	uint32_t regmap_base_addr = 0;

	ADI_CMS_NULL_PTR_CHECK(device);
	ADI_APOLLO_LOG_FUNC();
	ADI_APOLLO_BMEM_BLK_SEL_MASK(bmems);

	for (i = 0; i < ADI_APOLLO_BMEM_NUM; i++) {
		if ((bmems & (ADI_APOLLO_BMEM_A0 << i)) > 0) {
			regmap_base_addr = calc_bmem_hsdin_base(i);

			// Issue BMEM Reset before every capture
			err = adi_apollo_hal_bf_set(device, BF_BMEM_RESET_INFO(regmap_base_addr), 1);
			ADI_CMS_ERROR_RETURN(err);

			err = adi_apollo_hal_bf_wait_to_clear(device, BF_BMEM_RESET_INFO(regmap_base_addr), TIMEOUT_US, POLL_DELAY_US);
			ADI_CMS_ERROR_RETURN(err);

			// Enable triggers
			err = adi_apollo_hal_bf_set(device, BF_TRIG_MODE_INFO(regmap_base_addr), 1);
			ADI_CMS_ERROR_RETURN(err);

			// Set trigger mode self clear
			err = adi_apollo_hal_bf_set(device, BF_TRIG_MODE_SCLR_EN_INFO(regmap_base_addr), 1);
			ADI_CMS_ERROR_RETURN(err);


			// Set SRAM access to converter - Fast
			err = adi_apollo_hal_bf_set(device, BF_FAST_NSLOW_MODE_INFO(regmap_base_addr), 1);
			ADI_CMS_ERROR_RETURN(err);

			// Wait at least 4 i_HCLK and 4 (i_refclk/i_clk_div) when fast_nslow_mode switched
			err = adi_apollo_hal_delay_us(device, 10000);
			ADI_CMS_ERROR_RETURN(err);
		}
	}

	adi_apollo_trigts_trig_now(device, ADI_APOLLO_RX, ADI_APOLLO_SIDE_ALL);

	for (i = 0; i < ADI_APOLLO_BMEM_NUM; i++) {
		if ((bmems & (ADI_APOLLO_BMEM_A0 << i)) > 0) {
			regmap_base_addr = calc_bmem_hsdin_base(i);

			// Wait for Capture to complete and RAM to fill
			err = adi_apollo_hal_bf_wait_to_set(device, BF_FULL_IRQ_INFO(regmap_base_addr), TIMEOUT_US, POLL_DELAY_US);
			ADI_CMS_ERROR_RETURN(err);

			// Set SRAM access to uP/SPI/HSCI - Slow
			err = adi_apollo_hal_bf_set(device, BF_FAST_NSLOW_MODE_INFO(regmap_base_addr), 0);
			ADI_CMS_ERROR_RETURN(err);

			// Delay after switching fast_nslow_mode. Before reading the memory
			err = adi_apollo_hal_delay_us(device, 10000);
			ADI_CMS_ERROR_RETURN(err);
		}
	}

	return API_CMS_ERROR_OK;
}

static int ad9088_bmem_configure_capture(struct ad9088_bmem_state *st)
{
	adi_apollo_device_t *device = &st->phy->ad9088;
	int ret;

	/* Configure capture parameters */
	st->capture_config.sample_size = (st->sample_size == 32) ? 1 : 0;
	st->capture_config.ramclk_ph_dis = 0;
	st->capture_config.st_addr_cpt = st->start_addr;
	st->capture_config.end_addr_cpt = st->end_addr;
	st->capture_config.parity_check_en = 1;

	/* Configure BMEM capture */
	ret = adi_apollo_bmem_hsdin_capture_config(device, st->bmem_sel, &st->capture_config);

	//adi_apollo_clk_mcs_oneshot_sync(device);
	adi_apollo_clk_mcs_dyn_sync_rxtxlinks_sequence_run(device);

	return ad9088_check_apollo_error(st->dev, ret, "adi_apollo_bmem_hsdin_capture_config");
}

static int ad9088_bmem_start_capture(struct ad9088_bmem_state *st)
{
	adi_apollo_device_t *device = &st->phy->ad9088;
	int ret;

	/* Start capture operation */
	ret = ad9088_hsdin_bmem_capture_run(device, st->bmem_sel);
	return ad9088_check_apollo_error(st->dev, ret, "adi_apollo_bmem_hsdin_capture_run");
}

static int ad9088_bmem_read_samples(struct ad9088_bmem_state *st)
{
	adi_apollo_device_t *device = &st->phy->ad9088;
	int ret, ch, num_enabled = 0;
	u32 i, sample_count, word_count, scan_offset;
	u16 *scan_u16;
	u32 words_read;

	/* Calculate number of 32-bit words to read */
	word_count = st->end_addr - st->start_addr + 1;
	sample_count = st->sample_count;

	dev_dbg(st->dev, "%s: start_addr=%u, end_addr=%u, sample_size=%u sample_count=%u\n", __func__,
		st->start_addr, st->end_addr, st->sample_size, sample_count);

	/* Step 1: Read from each enabled channel/BMEM into separate buffers */
	for_each_set_bit(ch, &st->active_channels, st->indio_dev->num_channels) {
		adi_apollo_blk_sel_t bmem = ad9088_channel_to_bmem(ch);

		dev_dbg(st->dev, "Reading channel %d (BMEM 0x%x)\n", ch, bmem);
		num_enabled++;

		words_read = 0;

		do {
			u32 chunk = min_t(u32, 4096, word_count - words_read);

			ret = adi_apollo_hal_stream_reg32_get(device,
							      calc_bmem_sram_base(ilog2(bmem)) + (words_read * 4),
							      &st->channel_buffers[ch][words_read], chunk, 0);
			ret = ad9088_check_apollo_error(st->dev, ret, "adi_apollo_bmem_hsdin_capture_get");
			if (ret) {
				dev_err(st->dev, "Failed to read channel %d: %d\n", ch, ret);
				return ret;
			}

			words_read += chunk;
		} while (words_read < word_count);
	}

	if (num_enabled == 0) {
		dev_err(st->dev, "No channels were successfully read\n");
		return -EIO;
	}

	/* Step 2: Demultiplex - build scan records and push to IIO buffer */
	scan_u16 = (u16 *)st->scan_data;

	for (i = 0; i < word_count; i++) {
		if (sample_count--) {
			scan_offset = 0;
			for_each_set_bit(ch, &st->active_channels, st->indio_dev->num_channels) {
				scan_u16[scan_offset++] = lower_16_bits(st->channel_buffers[ch][i]);
			}

			iio_push_to_buffers(st->indio_dev, st->scan_data);
		}

		if (sample_count--) {
			scan_offset = 0;
			for_each_set_bit(ch, &st->active_channels, st->indio_dev->num_channels) {
				scan_u16[scan_offset++] = upper_16_bits(st->channel_buffers[ch][i]);
			}

			iio_push_to_buffers(st->indio_dev, st->scan_data);
		}
	}

	return 0;
}

static void ad9088_bmem_capture_work_func(struct work_struct *work)
{
	struct ad9088_bmem_state *st =
		container_of(work, struct ad9088_bmem_state, capture_work.work);
	int ret;

	guard(mutex)(&st->phy->lock);

	ret = ad9088_bmem_read_samples(st);
	if (ret)
		dev_err(st->dev, "Error reading BMEM samples: %d\n", ret);

	/* Start capture on all selected BMEMs simultaneously */
	ret = ad9088_bmem_start_capture(st);
	if (ret)
		dev_err(st->dev, "Error starting BMEM capture: %d\n", ret);

	/* Schedule work to read samples after a delay */
	reinit_completion(&st->complete);
	queue_delayed_work(system_freezable_wq, &st->capture_work, msecs_to_jiffies(10));
}

static int ad9088_bmem_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct ad9088_bmem_state *st = iio_priv(indio_dev);
	int ret = 0;
	u64 freq;

	guard(mutex)(&st->phy->lock);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		freq = st->phy->profile.adc_config[chan->channel > 1].adc_sampling_rate_Hz;
		*val = lower_32_bits(freq);
		*val2 = upper_32_bits(freq);
		ret = IIO_VAL_INT_64;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static ssize_t ad9088_bmem_ext_info_read(struct iio_dev *indio_dev,
					 uintptr_t private,
					 const struct iio_chan_spec *chan,
					 char *buf)
{
	struct ad9088_bmem_state *st = iio_priv(indio_dev);
	int ret = 0, i, ch = chan->channel;
	ssize_t len = 0;

	guard(mutex)(&st->phy->lock);

	switch (private) {
	case 0: /* delay_sample - simple set function */
		ret = sprintf(buf, "%u\n", st->delay_sample[ch]);
		break;
	case 1: /* delay_sample_config_value - from delay_sample_config structure */
		ret = sprintf(buf, "%u\n", st->delay_sample_config[ch].sample_delay);
		break;
	case 2: /* delay_sample_parity_check_en */
		ret = sprintf(buf, "%u\n", st->delay_sample_config[ch].parity_check_en);
		break;
	case 3: /* delay_hop_array - 4 values */
		for (i = 0; i < ADI_APOLLO_BMEM_HOP_PROFILES; i++) {
			len += sprintf(buf + len, "%u%s",
				      st->delay_hop_config[ch].hop_delay[i],
				      (i < ADI_APOLLO_BMEM_HOP_PROFILES - 1) ? " " : "\n");
		}
		ret = len;
		break;
	case 4: /* delay_hop_sel_mode */
		ret = sprintf(buf, "%u\n", st->delay_hop_config[ch].hop_delay_sel_mode);
		break;
	case 5: /* delay_hop_trig_sclr_en */
		ret = sprintf(buf, "%u\n", st->delay_hop_config[ch].trig_mode_sclr_en);
		break;
	case 6: /* delay_hop_parity_check_en */
		ret = sprintf(buf, "%u\n", st->delay_hop_config[ch].parity_check_en);
		break;
	case 7: /* delay_start - write-only, returns 0 on read */
		ret = sprintf(buf, "0\n");
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t ad9088_bmem_ext_info_write(struct iio_dev *indio_dev,
					  uintptr_t private,
					  const struct iio_chan_spec *chan,
					  const char *buf, size_t len)
{
	struct ad9088_bmem_state *st = iio_priv(indio_dev);
	long long readin;
	int ret, ch = chan->channel;
	u32 bmem_sel = ad9088_channel_to_bmem(ch);
	char *token, *buf_copy;
	int count = 0;

	guard(mutex)(&st->phy->lock);

	switch (private) {
	case 0: /* delay_sample - simple set function */
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			return ret;
		ret = adi_apollo_bmem_hsdin_delay_sample_set(&st->phy->ad9088, bmem_sel, readin);
		ret = ad9088_check_apollo_error(st->dev, ret, "adi_apollo_bmem_hsdin_delay_sample_set");
		if (ret)
			return ret;
		st->delay_sample[ch] = readin;
		break;
	case 1: /* delay_sample_config_value - updates structure and applies full config */
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			return ret;
		st->delay_sample_config[ch].sample_delay = readin;
		ret = adi_apollo_bmem_hsdin_delay_sample_config(&st->phy->ad9088, bmem_sel,
								&st->delay_sample_config[ch]);
		ret = ad9088_check_apollo_error(st->dev, ret, "adi_apollo_bmem_hsdin_delay_sample_config");
		if (ret)
			return ret;
		st->delay_sample[ch] = readin;
		break;
	case 2: /* delay_sample_parity_check_en */
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			return ret;
		st->delay_sample_config[ch].parity_check_en = readin ? 1 : 0;
		ret = adi_apollo_bmem_hsdin_delay_sample_config(&st->phy->ad9088, bmem_sel,
								&st->delay_sample_config[ch]);
		ret = ad9088_check_apollo_error(st->dev, ret, "adi_apollo_bmem_hsdin_delay_sample_config");
		if (ret)
			return ret;
		break;
	case 3: /* delay_hop_array - expects exactly 4 space-separated values */
		buf_copy = kstrdup(buf, GFP_KERNEL);
		if (!buf_copy)
			return -ENOMEM;

		/* Parse space-separated list of exactly 4 delay values */
		{
			char *ptr = buf_copy;
			while ((token = strsep(&ptr, " \n")) != NULL && count < ADI_APOLLO_BMEM_HOP_PROFILES) {
				u16 val;
				if (*token == '\0')
					continue;
				ret = kstrtou16(token, 10, &val);
				if (ret) {
					kfree(buf_copy);
					return ret;
				}
				st->delay_hop_config[ch].hop_delay[count++] = val;
			}
		}
		kfree(buf_copy);

		if (count != ADI_APOLLO_BMEM_HOP_PROFILES) {
			dev_err(st->dev, "delay_hop_array requires exactly 4 values, got %d\n", count);
			return -EINVAL;
		}

		/* Apply to hardware */
		ret = adi_apollo_bmem_hsdin_delay_hop_set(&st->phy->ad9088, bmem_sel,
							  st->delay_hop_config[ch].hop_delay,
							  ADI_APOLLO_BMEM_HOP_PROFILES);
		ret = ad9088_check_apollo_error(st->dev, ret, "adi_apollo_bmem_hsdin_delay_hop_set");
		if (ret)
			return ret;
		break;
	case 4: /* delay_hop_sel_mode */
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			return ret;
		st->delay_hop_config[ch].hop_delay_sel_mode = readin;
		ret = adi_apollo_bmem_hsdin_delay_hop_config(&st->phy->ad9088, bmem_sel,
							     &st->delay_hop_config[ch]);
		ret = ad9088_check_apollo_error(st->dev, ret, "adi_apollo_bmem_hsdin_delay_hop_config");
		if (ret)
			return ret;
		break;
	case 5: /* delay_hop_trig_sclr_en */
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			return ret;
		st->delay_hop_config[ch].trig_mode_sclr_en = readin ? 1 : 0;
		ret = adi_apollo_bmem_hsdin_delay_hop_config(&st->phy->ad9088, bmem_sel,
							     &st->delay_hop_config[ch]);
		ret = ad9088_check_apollo_error(st->dev, ret, "adi_apollo_bmem_hsdin_delay_hop_config");
		if (ret)
			return ret;
		break;
	case 6: /* delay_hop_parity_check_en */
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			return ret;
		st->delay_hop_config[ch].parity_check_en = readin ? 1 : 0;
		ret = adi_apollo_bmem_hsdin_delay_hop_config(&st->phy->ad9088, bmem_sel,
							     &st->delay_hop_config[ch]);
		ret = ad9088_check_apollo_error(st->dev, ret, "adi_apollo_bmem_hsdin_delay_hop_config");
		if (ret)
			return ret;
		break;
	case 7: /* delay_start - trigger delay start for this channel's BMEM */
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			return ret;
		if (readin) {
			ret = adi_apollo_bmem_hsdin_delay_start(&st->phy->ad9088, bmem_sel);
			ret = ad9088_check_apollo_error(st->dev, ret, "adi_apollo_bmem_hsdin_delay_start");
			if (ret)
				return ret;
		}
		break;
	default:
		ret = -EINVAL;
	}

	return ret ? ret : len;
}

static struct iio_chan_spec_ext_info ad9088_bmem_ext_info[] = {
	{
		.name = "delay_sample",
		.read = ad9088_bmem_ext_info_read,
		.write = ad9088_bmem_ext_info_write,
		.shared = IIO_SEPARATE,
		.private = 0,
	},
	{
		.name = "delay_sample_config_value",
		.read = ad9088_bmem_ext_info_read,
		.write = ad9088_bmem_ext_info_write,
		.shared = IIO_SEPARATE,
		.private = 1,
	},
	{
		.name = "delay_sample_parity_check_en",
		.read = ad9088_bmem_ext_info_read,
		.write = ad9088_bmem_ext_info_write,
		.shared = IIO_SEPARATE,
		.private = 2,
	},
	{
		.name = "delay_hop_array",
		.read = ad9088_bmem_ext_info_read,
		.write = ad9088_bmem_ext_info_write,
		.shared = IIO_SEPARATE,
		.private = 3,
	},
	{
		.name = "delay_hop_sel_mode",
		.read = ad9088_bmem_ext_info_read,
		.write = ad9088_bmem_ext_info_write,
		.shared = IIO_SEPARATE,
		.private = 4,
	},
	{
		.name = "delay_hop_trig_sclr_en",
		.read = ad9088_bmem_ext_info_read,
		.write = ad9088_bmem_ext_info_write,
		.shared = IIO_SEPARATE,
		.private = 5,
	},
	{
		.name = "delay_hop_parity_check_en",
		.read = ad9088_bmem_ext_info_read,
		.write = ad9088_bmem_ext_info_write,
		.shared = IIO_SEPARATE,
		.private = 6,
	},
	{
		.name = "delay_start",
		.read = ad9088_bmem_ext_info_read,
		.write = ad9088_bmem_ext_info_write,
		.shared = IIO_SEPARATE,
		.private = 7,
	},
	{},
};

static const struct iio_chan_spec ad9088_bmem_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9088_bmem_ext_info,
		.indexed = 1,
		.channel = 0,
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9088_bmem_ext_info,
		.indexed = 1,
		.channel = 1,
		.scan_index = 1,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9088_bmem_ext_info,
		.indexed = 1,
		.channel = 2,
		.scan_index = 2,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9088_bmem_ext_info,
		.indexed = 1,
		.channel = 3,
		.scan_index = 3,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	},
};

static int ad9088_bmem_debugfs_reg_access(struct iio_dev *indio_dev,
					  unsigned int reg, unsigned int writeval,
					  unsigned int *readval)
{
	struct ad9088_bmem_state *st = iio_priv(indio_dev);
	struct ad9088_phy *phy = st->phy;
	u8 val;
	int ret;

	guard(mutex)(&st->phy->lock);

	if (!readval) {
		ret = adi_apollo_hal_reg_set(&phy->ad9088, reg, writeval);
		return ad9088_check_apollo_error(st->dev, ret, "adi_apollo_hal_reg_set");
	}

	ret = adi_apollo_hal_reg_get(&phy->ad9088, reg, &val);
	ret = ad9088_check_apollo_error(st->dev, ret, "adi_apollo_hal_reg_get");
	if (ret)
		return ret;

	*readval = val;
	return 0;
}

static const struct iio_info ad9088_bmem_info = {
	.read_raw = &ad9088_bmem_read_raw,
	.debugfs_reg_access = &ad9088_bmem_debugfs_reg_access,
};

static int ad9088_bmem_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad9088_bmem_state *st = iio_priv(indio_dev);
	struct ad9088_phy *phy = st->phy;
	adi_apollo_blk_sel_t bmem_mask = 0;
	int ret, i;

	guard(mutex)(&st->phy->lock);

	dev_dbg(st->dev, "%s: buffer length: %d\n", __func__, indio_dev->buffer->length);

	st->end_addr = st->start_addr + DIV_ROUND_UP(indio_dev->buffer->length,
						     2 * indio_dev->num_channels ) - 1;

	st->sample_count = indio_dev->buffer->length / indio_dev->num_channels ;

	rx_mux1_config(&phy->ad9088);

	/* Save which channels are active and build BMEM bitmask */
	st->active_channels = *indio_dev->active_scan_mask;

	for_each_set_bit(i, &st->active_channels, indio_dev->num_channels) {
		bmem_mask |= ad9088_channel_to_bmem(i);
	}

	if (bmem_mask == 0) {
		dev_err(st->dev, "No channels enabled\n");
		return -EINVAL;
	}

	dev_dbg(st->dev, "Configuring BMEMs with mask 0x%x\n", bmem_mask);

	/* Store the mask for later use */
	st->bmem_sel = bmem_mask;

	ret  = adi_apollo_trigts_bmem_trig_sel_mux_set(&phy->ad9088, ADI_APOLLO_RX, bmem_mask, ADI_APOLLO_TRIG_SPI);
	if (ret)
		return ret;
	/* Configure capture parameters for all selected BMEMs */
	ret = ad9088_bmem_configure_capture(st);
	if (ret)
		return ret;

	/* Start capture on all selected BMEMs simultaneously */
	ret = ad9088_bmem_start_capture(st);
	if (ret)
		return ret;

	/* Schedule work to read samples after a delay */
	reinit_completion(&st->complete);
	queue_delayed_work(system_freezable_wq, &st->capture_work, msecs_to_jiffies(100));

	return 0;
}

static int ad9088_bmem_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad9088_bmem_state *st = iio_priv(indio_dev);

	dev_dbg(st->dev, "%s:%d\n", __func__, __LINE__);

	complete_all(&st->complete);
	cancel_delayed_work_sync(&st->capture_work);

	return 0;
}

static const struct iio_buffer_setup_ops ad9088_bmem_buffer_ops = {
	.postenable = ad9088_bmem_buffer_postenable,
	.predisable = ad9088_bmem_buffer_predisable,
};

static int ad9088_bmem_debugfs_init(struct iio_dev *indio_dev)
{
	struct ad9088_bmem_state *st = iio_priv(indio_dev);
	struct dentry *d;

	d = iio_get_debugfs_dentry(indio_dev);
	if (d) {
		debugfs_create_u32("bmem_sel", 0644, d, &st->bmem_sel);
		debugfs_create_u32("bmem_loc", 0644, d, &st->bmem_loc);
		debugfs_create_u16("start_addr", 0644, d, &st->start_addr);
		debugfs_create_u16("end_addr", 0644, d, &st->end_addr);
		debugfs_create_u32("sample_size", 0644, d, &st->sample_size);
	}

	return 0;
}

int ad9088_bmem_probe(struct ad9088_phy *phy)
{
	struct iio_dev *indio_dev;
	struct ad9088_bmem_state *st;
	struct device *dev = &phy->spi->dev;
	int ret, i;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev) {
		dev_err(dev, "Can't allocate iio device\n");
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);
	st->dev = dev;
	st->phy = phy;
	st->indio_dev = indio_dev;

	/* Initialize default configuration */
	st->bmem_sel = ADI_APOLLO_BMEM_A0; /* Default to A0, can be changed via debugfs or channel selection */
	st->bmem_loc = ADI_APOLLO_BMEM_HSDIN;
	st->sample_size = 16;
	st->start_addr = 0;

	/* Initialize delay configurations for each channel */
	for (i = 0; i < MAX_NUM_CHANNELS; i++) {
		st->delay_sample[i] = 0;

		/* Initialize delay_sample_config */
		st->delay_sample_config[i].sample_size = 0; /* 0: 16-bit */
		st->delay_sample_config[i].ramclk_ph_dis = 0;
		st->delay_sample_config[i].sample_delay = 0;
		st->delay_sample_config[i].parity_check_en = 1;

		/* Initialize delay_hop_config */
		st->delay_hop_config[i].sample_size = 0; /* 0: 16-bit */
		st->delay_hop_config[i].ramclk_ph_dis = 0;
		st->delay_hop_config[i].hop_delay[0] = 0;
		st->delay_hop_config[i].hop_delay[1] = 0;
		st->delay_hop_config[i].hop_delay[2] = 0;
		st->delay_hop_config[i].hop_delay[3] = 0;
		st->delay_hop_config[i].hop_delay_sel_mode = 0; /* 0: cycle through profiles */
		st->delay_hop_config[i].trig_mode_sclr_en = 1;
		st->delay_hop_config[i].parity_check_en = 1;
	}

	mutex_init(&st->lock);
	init_completion(&st->complete);
	INIT_DELAYED_WORK(&st->capture_work, ad9088_bmem_capture_work_func);

	/* Allocate per-channel buffers for captured samples */
	for (i = 0; i < 4; i++) {
		st->channel_buffers[i] = devm_kzalloc(dev, AD9088_BMEM_SRAM_SIZE_BYTES, GFP_KERNEL);
		if (!st->channel_buffers[i])
			return -ENOMEM;
	}

	indio_dev->info = &ad9088_bmem_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad9088_bmem_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad9088_bmem_channels);
	indio_dev->name = "ad9088-bmem";

	ret = devm_iio_kfifo_buffer_setup_ext(dev, indio_dev, &ad9088_bmem_buffer_ops, NULL);
	if (ret)
		return ret;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;

	ad9088_bmem_debugfs_init(indio_dev);

	return 0;
}
EXPORT_SYMBOL_GPL(ad9088_bmem_probe);