// SPDX-License-Identifier: GPL-2.0
/*
 * Common extended library for ADIS devices
 *
 * Copyright 2023 Analog Devices Inc.
 *   Author: Ramona Bolboaca <ramona.bolboaca@analog.com>
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/imu/adis.h>
#include <linux/iio/imu/adis_extd.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/irq.h>
#include <linux/lcm.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#define ADIS_EXTD_CHECKSUM_SIZE 2 /* in bytes */
#define ADIS_EXTD_16_BIT_BURST_SIZE 0
#define ADIS_EXTD_MSG_SIZE_16_BIT_BURST 20 /* in bytes */
#define ADIS_EXTD_MSG_SIZE_32_BIT_BURST 32 /* in bytes */
#define ADIS_EXTD_LSB_DEC_MASK		BIT(0)
#define ADIS_EXTD_LSB_FIR_MASK		BIT(1)

/**
 * @brief Read field to u32 value.
 * @param adis_extd - The adis extended device.
 * @param field     - The field structure to be read.
 * @param field_val - The read field value.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_read_field_u32(struct adis_extd *adis_extd,
			     struct adis_extd_field field,
			     u32 *field_val)
{
	int ret;
	unsigned int reg_val;

	ret = adis_read_reg(&adis_extd->adis, field.reg_addr, &reg_val,
			    field.reg_size);
	if (ret)
		return ret;

	*field_val = adis_extd_field_get(reg_val, field.field_mask);

	return 0;
}

/**
 * @brief Write field to u32 value.
 * @param adis_extd - The adis extended device.
 * @param field     - The field structure to be written.
 * @param field_val - The field value to be written.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_write_field_u32(struct adis_extd *adis_extd,
			      struct adis_extd_field field,
			      u32 field_val)
{
	if (field_val > adis_extd_field_get(field.field_mask, field.field_mask))
		return -EINVAL;

	return adis_update_bits_base(&adis_extd->adis, field.reg_addr, field.field_mask,
				     adis_extd_field_prep(field_val, field.field_mask),
				     field.reg_size);
}

/**
 * @brief Update device diagnosis flags according to the received parameter.
 * @param adis_extd - The adis extended device.
 * @param diag_stat - Diagnosis flags.
 */
void adis_extd_update_diag_flags(struct adis_extd *adis_extd,
				 u16 diag_stat)
{
	const struct adis_extd_data_field_map_def *field_map =
			adis_extd->info->field_map;
	adis_extd->diag_flags.snsr_init_failure =
		diag_stat & field_map->diag_snsr_init_failure_mask ? 1 : 0;
	adis_extd->diag_flags.data_path_overrun =
		diag_stat & field_map->diag_data_path_overrun_mask ? 1 : 0;
	adis_extd->diag_flags.fls_mem_update_failure =
		diag_stat & field_map->diag_fls_mem_update_failure_mask ? 1 : 0;
	adis_extd->diag_flags.spi_comm_err =
		diag_stat & field_map->diag_spi_comm_err_mask ? 1 : 0;
	adis_extd->diag_flags.standby_mode =
		diag_stat & field_map->diag_standby_mode_mask ? 1 : 0;
	adis_extd->diag_flags.snsr_failure =
		diag_stat & field_map->diag_snsr_failure_mask ? 1 : 0;
	adis_extd->diag_flags.mem_failure =
		diag_stat & field_map->diag_mem_failure_mask ? 1 : 0;
	adis_extd->diag_flags.clk_err =
		diag_stat & field_map->diag_clk_err_mask ? 1 : 0;
	adis_extd->diag_flags.gyro1_failure =
		diag_stat & field_map->diag_gyro1_failure_mask ? 1 : 0;
	adis_extd->diag_flags.gyro2_failure =
		diag_stat & field_map->diag_gyro2_failure_mask ? 1 : 0;
	adis_extd->diag_flags.accl_failure =
		diag_stat & field_map->diag_accl_failure_mask ? 1 : 0;
	adis_extd->diag_flags.x_axis_gyro_failure =
		diag_stat & field_map->diag_x_axis_gyro_failure_mask ? 1 : 0;
	adis_extd->diag_flags.y_axis_gyro_failure =
		diag_stat & field_map->diag_y_axis_gyro_failure_mask ? 1 : 0;
	adis_extd->diag_flags.z_axis_gyro_failure =
		diag_stat & field_map->diag_z_axis_gyro_failure_mask ? 1 : 0;
	adis_extd->diag_flags.x_axis_accl_failure =
		diag_stat & field_map->diag_x_axis_accl_failure_mask ? 1 : 0;
	adis_extd->diag_flags.y_axis_accl_failure =
		diag_stat & field_map->diag_y_axis_accl_failure_mask ? 1 : 0;
	adis_extd->diag_flags.z_axis_accl_failure =
		diag_stat & field_map->diag_z_axis_accl_failure_mask ? 1 : 0;
	adis_extd->diag_flags.aduc_mcu_fault =
		diag_stat & field_map->diag_aduc_mcu_fault_mask ? 1 : 0;
}

/**
 * @brief Write FIFO enable bit value.
 * @param arg     - The adis extended device.
 * @param fifo_en - The FIFO enable bit value to write.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_write_fifo_en(void *arg, u64 fifo_en)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_write_field_u32(adis_extd, adis_extd->info->field_map->fifo_en,
					fifo_en);
}

/**
 * @brief Write FIFO overflow bit value.
 * @param arg           - The adis extended device.
 * @param fifo_overflow - The FIFO overflow bit value to write.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_write_fifo_overflow(void *arg, u64 fifo_overflow)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_write_field_u32(adis_extd,
					 adis_extd->info->field_map->fifo_overflow,
					 fifo_overflow);
}

/**
 * @brief Write FIFO watermark interrupt enable bit value.
 * @param arg            - The adis extended device.
 * @param fifo_wm_int_en - The FIFO watermark interrupt enable bit value to write.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_write_fifo_wm_int_en(void *arg, u64 fifo_wm_int_en)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_write_field_u32(adis_extd,
					 adis_extd->info->field_map->fifo_wm_int_en,
					 fifo_wm_int_en);
}

/**
 * @brief Write FIFO watermark interrupt polarity bit value.
 * @param arg             - The adis extended device.
 * @param fifo_wm_int_pol - The FIFO watermark interrupt polarity bit value to write.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_write_fifo_wm_int_pol(void *arg, u64 fifo_wm_int_pol)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_write_field_u32(adis_extd,
					 adis_extd->info->field_map->fifo_wm_int_pol,
					 fifo_wm_int_pol);
}

/**
 * @brief Write FIFO watermark threshold level value.
 * @param arg         - The adis extended device.
 * @param fifo_wm_lvl - The FIFO watermark threshold level value to write.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_write_fifo_wm_lvl(void *arg, u64 fifo_wm_lvl)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_write_field_u32(adis_extd,
					 adis_extd->info->field_map->fifo_wm_lvl,
					 fifo_wm_lvl);
}

/**
 * @brief Read filter size variable B value.
 * @param arg             - The adis extended device.
 * @param filt_size_var_b - The filter size variable B read value.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_read_filt_size_var_b(void *arg, u64 *filt_size_var_b)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(adis_extd,
				       adis_extd->info->field_map->filt_size_var_b,
				       &field_val);
	if (ret)
		return ret;

	*filt_size_var_b = field_val;

	return 0;
}

/**
 * @brief Write burst32 enable bit value.
 * @param arg     - The adis extended device.
 * @param burst32 - The burst32 enable bit value to write.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_write_burst32(void *arg, u64 burst32)
{
	struct adis_extd *adis_extd = arg;
	struct adis *adis = &adis_extd->adis;
	int ret;

	ret = adis_extd_write_field_u32(
		      adis_extd, adis_extd->info->field_map->burst32, burst32);
	if (ret)
		return ret;

	adis_extd->burst32 = burst32;

	fsleep(adis_extd->info->timeouts->msc_reg_update_us);

	if (burst32)
		adis->burst_extra_len = adis_extd->info->burst32_extra_bytes;
	else
		adis->burst_extra_len = 0;

	return 0;
}

/**
 * @brief Write filter size variable B value.
 * @param arg             - The adis extended device.
 * @param filt_size_var_b - The filter size variable B value to write.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_write_filt_size_var_b(void *arg, u64 filt_size_var_b)
{
	struct adis_extd *adis_extd = arg;
	int ret;

	if (filt_size_var_b > adis_extd->info->filt_size_var_b_max)
		return -EINVAL;

	ret = adis_extd_write_field_u32(adis_extd,
					adis_extd->info->field_map->filt_size_var_b,
					filt_size_var_b);
	if (ret)
		return ret;

	fsleep(adis_extd->info->timeouts->filt_size_var_b_update_us);

	assign_bit(ADIS_EXTD_LSB_FIR_MASK, &adis_extd->lsb_flag, filt_size_var_b);

	return 0;
}

/**
 * @brief Write data ready polarity encoded value.
 * @param arg         - The adis extended device.
 * @param dr_polarity - The data ready polarity encoded value to write.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_write_dr_polarity(void *arg, u64 dr_polarity)
{
	struct adis_extd *adis_extd = arg;
	int ret;

	ret = adis_extd_write_field_u32(adis_extd,
					adis_extd->info->field_map->dr_polarity,
					dr_polarity);
	if (ret)
		return ret;

	fsleep(adis_extd->info->timeouts->msc_reg_update_us);

	return 0;
}

/**
 * @brief Read synchronization mode encoded value.
 * @param arg       - The adis extended device.
 * @param sync_mode - The synchronization mode encoded value.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_read_sync_mode(void *arg, u64 *sync_mode)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(adis_extd, adis_extd->info->field_map->sync_mode,
				       &field_val);
	if (ret)
		return ret;

	*sync_mode = field_val;

	return 0;
}

/**
 * @brief Read external clock scale factor value.
 * @param arg      - The adis extended device.
 * @param up_scale - The external clock scale factor read value.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_read_up_scale(void *arg, u64 *up_scale)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(adis_extd, adis_extd->info->field_map->up_scale,
				       &field_val);
	if (ret)
		return ret;

	*up_scale = field_val;

	return 0;
}

/**
 * @brief Read decimation rate value.
 * @param arg      - The adis extended device.
 * @param dec_rate - The decimation rate read value.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_read_dec_rate(void *arg, u64 *dec_rate)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(adis_extd, adis_extd->info->field_map->dec_rate,
				       &field_val);
	if (ret)
		return ret;

	*dec_rate = field_val;

	return 0;
}

/**
 * @brief Read adis synchronization clock frequency value in Hertz.
 * @param adis_extd - The adis extended device.
 * @param clk_freq  - The current adis synchronization clock frequency.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_get_sync_clk_freq(struct adis_extd *adis_extd,
				       u32 *clk_freq)
{
	int ret;
	u64 sync_mode;

	ret = adis_extd_read_sync_mode(adis_extd, &sync_mode);
	if (ret)
		return ret;

	if (sync_mode == ADIS_EXTD_SYNC_DEFAULT ||
	    sync_mode == ADIS_EXTD_SYNC_OUTPUT)
		*clk_freq = adis_extd->int_clk;
	else
		*clk_freq = adis_extd->ext_clk;

	return 0;
}

/**
 * @brief Read sampling frequency in Hertz.
 * @param adis_extd - The adis extended device.
 * @param freq      - The current sampling frequency of the device.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_read_sampling_freq(struct adis_extd *adis_extd, u32 *freq)
{
	int ret;
	u64 dec_rate;
	u64 sync_mode;
	u64 up_scale;
	u32 sample_rate;
	u32 adis_sync_clk_freq;

	ret = adis_extd_get_sync_clk_freq(adis_extd, &adis_sync_clk_freq);
	if (ret)
		return ret;

	sample_rate = adis_sync_clk_freq;

	ret = adis_extd_read_sync_mode(adis_extd, &sync_mode);
	if (ret)
		return ret;

	if (sync_mode == ADIS_EXTD_SYNC_SCALED) {
		ret = adis_extd_read_up_scale(adis_extd, &up_scale);
		if (ret)
			return ret;

		sample_rate = adis_sync_clk_freq * up_scale;
	}

	ret = adis_extd_read_dec_rate(adis_extd, &dec_rate);
	if (ret)
		return ret;

	*freq = DIV_ROUND_CLOSEST(sample_rate, (u32)dec_rate + 1);

	return 0;
}

/**
 * @brief Write external clock scale factor value.
 * @param arg      - The adis extended device.
 * @param up_scale - The external clock scale factor value to write.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_write_up_scale(void *arg, u64 up_scale)
{
	struct adis_extd *adis_extd = arg;
	int ret;
	u64 sync_mode;

	ret = adis_extd_read_sync_mode(adis_extd, &sync_mode);
	if (ret)
		return ret;

	/*
	 * Allow for any value to be written unless the device is in SYNC_SCALED
	 * synchronization mode.
	 * If the device is in SYNC_SCALED syncronization mode, make sure the
	 * result for clk_freq * up_scale is between sampling clock limits,
	 * otherwise return -EINVAL.
	 */
	if (sync_mode == ADIS_EXTD_SYNC_SCALED &&
	    (adis_extd->ext_clk * up_scale >
	     adis_extd->info->sampling_clk_limits.max_freq ||
	     adis_extd->ext_clk * up_scale <
	     adis_extd->info->sampling_clk_limits.min_freq))
		return -EINVAL;

	ret = adis_extd_write_field_u32(adis_extd, adis_extd->info->field_map->up_scale,
					up_scale);
	if (ret)
		return ret;

	return adis_extd_read_sampling_freq(adis_extd,
					    &adis_extd->sampling_frequency);
}

/**
 * @brief Write decimation rate value.
 * @param arg      - The adis extended device.
 * @param dec_rate - The decimation rate value to write.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_write_dec_rate(void *arg, u64 dec_rate)
{
	struct adis_extd *adis_extd = arg;
	int ret;

	if (dec_rate > adis_extd->info->dec_rate_max)
		return -EINVAL;

	ret = adis_extd_write_field_u32(adis_extd, adis_extd->info->field_map->dec_rate,
					dec_rate);
	if (ret)
		return ret;

	fsleep(adis_extd->info->timeouts->dec_rate_update_us);

	ret = adis_extd_read_sampling_freq(adis_extd,
					   &adis_extd->sampling_frequency);
	if(ret)
		return ret;

	assign_bit(ADIS_EXTD_LSB_DEC_MASK, &adis_extd->lsb_flag, dec_rate);

	return 0;
}

/**
 * @brief Update synchronization mode.
 * @param adis_extd - The adis device.
 * @param sync_mode - The synchronization mode encoded value to update.
 * @param ext_clk   - The external clock frequency to update, will be ignored
 * if sync_mode is different from ADIS_SYNC_SCALED and ADIS_SYNC_DIRECT.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_update_sync_mode(struct adis_extd *adis_extd,
			       unsigned int sync_mode,
			       unsigned int ext_clk)
{
	int ret;

	if (sync_mode > adis_extd->info->sync_mode_max)
		return -EINVAL;

	if (sync_mode == ADIS_EXTD_SYNC_DIRECT ||
	    sync_mode == ADIS_EXTD_SYNC_SCALED) {
		/* Sync pulse is external */
		if (ext_clk < adis_extd->info->sync_clk_freq_limits[sync_mode]
		    .min_freq ||
		    ext_clk > adis_extd->info->sync_clk_freq_limits[sync_mode]
		    .max_freq)
			return -EINVAL;

		adis_extd->ext_clk = ext_clk;

		if (sync_mode == ADIS_EXTD_SYNC_SCALED) {
			/*
			 * In sync scaled mode, the IMU sample rate is the
			 * clk_freq * sync_scale.
			 * Hence, default the IMU sample rate to the highest
			 * multiple of the input clock lower than the IMU max
			 * sample rate.
			 */
			ret = adis_extd_write_up_scale(adis_extd,
						       adis_extd->info->sampling_clk_limits.max_freq / ext_clk);
			if (ret)
				return ret;
		}
	}

	ret = adis_extd_write_field_u32(adis_extd,
					adis_extd->info->field_map->sync_mode, sync_mode);
	if (ret)
		return ret;

	return adis_extd_read_sampling_freq(adis_extd,
					    &adis_extd->sampling_frequency);
}

/**
 * @brief Sets sync scale and decimation rate based on the desired sampling
 *        frequency in Hz.
 * @param adis_extd - The adis extended device structure.
 * @param freq      - The desired sampling frequency of the device.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_sampling_freq(struct adis_extd *adis_extd,
		const u32 freq)
{
	int ret;
	u32 adis_sync_clk_freq;
	u32 sample_rate;
	u16 dec_rate;
	int up_scale;
	u64 sync_mode;
	unsigned long scaled_rate;
	u32 max_sampling_clk_freq =
		adis_extd->info->sampling_clk_limits.max_freq;

	if (!freq)
		return -EINVAL;

	ret = adis_extd_get_sync_clk_freq(adis_extd, &adis_sync_clk_freq);
	if (ret)
		return ret;

	sample_rate = adis_sync_clk_freq;

	ret = adis_extd_read_sync_mode(adis_extd, &sync_mode);
	if (ret)
		return ret;

	adis_enable_irq(&adis_extd->adis, false);

	if (sync_mode == ADIS_EXTD_SYNC_SCALED) {
		scaled_rate = lcm(adis_extd->ext_clk, freq);

		/*
		 * If lcm is bigger than the IMU maximum sampling rate there's
		 * no perfect solution. In this case, we get the highest
		 * multiple of the input clock lower than the IMU max sample
		 * rate.
		 */
		if (scaled_rate > max_sampling_clk_freq)
			scaled_rate = max_sampling_clk_freq /
				      adis_extd->ext_clk * adis_extd->ext_clk;
		else
			scaled_rate = max_sampling_clk_freq / scaled_rate *
				      scaled_rate;

		up_scale = scaled_rate / adis_extd->ext_clk;
		ret = adis_extd_write_up_scale(adis_extd, up_scale);
		if (ret)
			goto enable_irq;

		sample_rate = scaled_rate;
	}

	dec_rate = DIV_ROUND_CLOSEST(sample_rate, freq);

	if (dec_rate)
		dec_rate--;

	if (dec_rate > adis_extd->info->dec_rate_max)
		dec_rate = adis_extd->info->dec_rate_max;

	ret = adis_extd_write_dec_rate(adis_extd, dec_rate);

enable_irq:
	adis_enable_irq(&adis_extd->adis, true);
	return ret;
}

/**
 * @brief Command: fifo flush
 * @param arg - The adis extended device.
 * @param val - Value to write, any value will trigger the command.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_cmd_fifo_flush(void *arg, u64 val)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_field field = adis_extd->info->field_map->fifo_flush;

	return adis_write_reg(&adis_extd->adis, field.reg_addr,
			      field.field_mask, field.reg_size);
}

/* The values are approximated. */
static const u32 adis_extd_3db_freqs[] = {
	720, /* Filter disabled, full BW (~720Hz) */
	360, 164, 80, 40, 20, 10,
};

/**
 * @brief Read adis_extd low pass filter value.
 * @param adis_extd - The adis extended device.
 * @param filter   - The read filter value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_lpf(struct adis_extd *adis_extd, u32 *filter)
{
	u64 filter_sz;
	int ret;

	ret = adis_extd_read_filt_size_var_b(adis_extd, &filter_sz);
	if (ret)
		return ret;

	*filter = adis_extd_3db_freqs[filter_sz];

	return 0;
}

/**
 * @brief Write adis_extd low pass filter value.
 * @param adis_extd - The adis extended device.
 * @param filter   - The filter value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_filter(struct adis_extd *adis_extd, u32 filter)
{
	int i = ARRAY_SIZE(adis_extd_3db_freqs);

	while (--i) {
		if (adis_extd_3db_freqs[i] >= filter)
			break;
	}

	if (i < 0)
		return -EINVAL;

	return adis_extd_write_filt_size_var_b(adis_extd, i);
}

/**
 * @brief Read adis_extd raw attributes.
 */
static int adis_extd_read_raw(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan, int *val,
			      int *val2, long info)
{
	struct adis_extd *adis_extd = iio_priv(indio_dev);
	int ret;
	u16 temp_out;
	u32 tmp;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if(chan->type== IIO_TEMP) {
			ret = adis_read_reg_16(&adis_extd->adis,
					       adis_extd->info->field_map->temp_out.reg_addr, &temp_out);
			if (ret)
				return ret;
			tmp = temp_out;
			*val = sign_extend32(tmp, 15);
			return IIO_VAL_INT;
		}

		return adis_single_conversion(indio_dev, chan, 0, val);
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			*val = adis_extd->info->gyro_scale.dividend;
			*val2 = adis_extd->info->gyro_scale.divisor;
			return IIO_VAL_FRACTIONAL;
		case IIO_ACCEL:
			*val = adis_extd->info->accl_scale.dividend;
			*val2 = adis_extd->info->accl_scale.divisor;
			return IIO_VAL_FRACTIONAL;
		case IIO_TEMP:
			*val = adis_extd->info->temp_scale.dividend;
			*val2 = adis_extd->info->temp_scale.divisor;
			return IIO_VAL_FRACTIONAL;
		case IIO_ROT:
			*val = adis_extd->info->rot_scale.dividend;
			*val2 = adis_extd->info->rot_scale.power;
			return IIO_VAL_FRACTIONAL_LOG2;
		case IIO_VELOCITY:
			*val = adis_extd->info->vel_scale.dividend;
			*val2 = adis_extd->info->vel_scale.power;
			return IIO_VAL_FRACTIONAL_LOG2;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_CALIBBIAS:
		ret = adis_read_reg_32(&adis_extd->adis,
				       adis_extd->info->field_map->xg_bias.reg_addr + 4 * chan->scan_index, val);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		ret = adis_extd_read_lpf(adis_extd, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = adis_extd_read_sampling_freq(adis_extd, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

/**
 * @brief Write adis_extd raw attributes.
 */
static int adis_extd_write_raw(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan, int val,
			       int val2, long info)
{
	struct adis_extd *adis_extd = iio_priv(indio_dev);
	u32 tmp;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		tmp = val;
		return adis_extd_write_sampling_freq(adis_extd, tmp);
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		return adis_extd_write_filter(adis_extd, val);
	case IIO_CHAN_INFO_CALIBBIAS:
		tmp = val;
		ret = adis_write_reg_16(&adis_extd->adis,
					adis_extd->info->field_map->xg_bias.reg_addr +
					4 * chan->scan_index + 2,
					tmp >> 16);
		if(ret)
			return ret;

		return adis_write_reg_16(&adis_extd->adis,
					 adis_extd->info->field_map->xg_bias.reg_addr +
					 4 * chan->scan_index,
					 tmp);
	default:
		return -EINVAL;
	}
}

/**
 * @brief Adis extended buffer preenable callback handler.
 * @param indio_dev - The iio device.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_buff_preenable(struct iio_dev *indio_dev)
{
	struct adis_extd *adis_extd = iio_priv(indio_dev);
	int ret;

	adis_extd->samples_lost = 0;
	adis_extd->data_cntr = 0;
	adis_extd->buffer_enabled = true;

	/* If the device has a fifo, use it for data continuity */
	if (adis_extd->info->has_fifo) {
		/* Flush the fifo to make sure the FIFO is empty. */
		ret = adis_extd_cmd_fifo_flush(adis_extd, 1);
		if (ret)
			return ret;
		fsleep(500);
		/* Set FIFO overflow behavior: older data will be rewritten with newer data if an overflow occurs. */
		ret = adis_extd_write_fifo_overflow(adis_extd, 1);
		if (ret)
			return ret;
		ret = adis_extd_write_fifo_wm_lvl(adis_extd, 511);
		if (ret)
			return ret;
		return adis_extd_write_fifo_en(adis_extd, 1);

	}

	return 0;
}

/**
 * @brief Adis extended buffer postdisable callback handler.
 * @param indio_dev - The iio device.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_buff_postdisable(struct iio_dev *indio_dev)
{
	struct adis_extd *adis_extd = iio_priv(indio_dev);
	int ret;

	if (adis_extd->info->has_fifo) {
		ret = adis_extd_write_fifo_en(adis_extd, 0);
		if (ret)
			return ret;
		ret = adis_extd_cmd_fifo_flush(adis_extd, 1);
		if (ret)
			return ret;
	}

	adis_extd->buffer_enabled = false;

	return 0;
}

const struct iio_buffer_setup_ops adis_extd_buff_ops = {
	.preenable = adis_extd_buff_preenable,
	.postdisable = adis_extd_buff_postdisable,
};

/**
 * @brief Adis extended scan update for burst readings.
 * @param indio_dev - The iio device.
 * @param scan_mask - The changed scan mask (unused).
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_update_scan_mode_burst(struct iio_dev *indio_dev,
		const unsigned long *scan_mask)
{
	struct adis *adis = iio_device_get_drvdata(indio_dev);
	struct adis_extd *adis_extd = iio_priv(indio_dev);
	unsigned int burst_length, burst_max_length;
	u8 *tx;

	burst_length = adis->data->burst_len + adis->burst_extra_len;

	burst_max_length = adis->data->burst_len + adis_extd->info->burst32_extra_bytes;

	kfree(adis->xfer);
	kfree(adis->buffer);
	kfree(adis_extd->xfer);
	kfree(adis_extd->buffer);

	adis->xfer = kcalloc(2, sizeof(*adis->xfer), GFP_KERNEL);
	if (!adis->xfer)
		return -ENOMEM;

	adis->buffer = kzalloc(burst_max_length + sizeof(u16), GFP_KERNEL);
	if (!adis->buffer) {
		kfree(adis->xfer);
		adis->xfer = NULL;
		return -ENOMEM;
	}

	adis_extd->xfer = kcalloc(2, sizeof(*adis_extd->xfer), GFP_KERNEL);
	if (!adis_extd->xfer)
		return -ENOMEM;

	adis_extd->buffer = kzalloc(burst_max_length + sizeof(u16), GFP_KERNEL);
	if (!adis_extd->buffer) {
		kfree(adis_extd->xfer);
		adis_extd->xfer = NULL;
		return -ENOMEM;
	}

	tx = adis->buffer + burst_max_length;
	tx[0] = ADIS_READ_REG(adis->data->burst_reg_cmd);
	tx[1] = 0;

	adis->xfer[0].tx_buf = tx;
	adis->xfer[0].bits_per_word = 8;
	adis->xfer[0].len = 2;
	if (adis->data->burst_max_speed_hz)
		adis->xfer[0].speed_hz = adis->data->burst_max_speed_hz;
	adis->xfer[1].rx_buf = adis->buffer;
	adis->xfer[1].bits_per_word = 8;
	adis->xfer[1].len = burst_length;
	if (adis->data->burst_max_speed_hz)
		adis->xfer[1].speed_hz = adis->data->burst_max_speed_hz;

	spi_message_init(&adis->msg);
	spi_message_add_tail(&adis->xfer[0], &adis->msg);
	spi_message_add_tail(&adis->xfer[1], &adis->msg);

	tx = adis_extd->buffer + burst_max_length;
	tx[0] = 0;
	tx[1] = 0;

	adis_extd->xfer[0].tx_buf = tx;
	adis_extd->xfer[0].bits_per_word = 8;
	adis_extd->xfer[0].len = 2;
	if (adis->data->burst_max_speed_hz)
		adis_extd->xfer[0].speed_hz = adis->data->burst_max_speed_hz;
	adis_extd->xfer[1].rx_buf = adis_extd->buffer;
	adis_extd->xfer[1].bits_per_word = 8;
	adis_extd->xfer[1].len = burst_length;
	if (adis->data->burst_max_speed_hz)
		adis_extd->xfer[1].speed_hz = adis->data->burst_max_speed_hz;

	spi_message_init(&adis_extd->msg);
	spi_message_add_tail(&adis_extd->xfer[0], &adis_extd->msg);
	spi_message_add_tail(&adis_extd->xfer[1], &adis_extd->msg);

	return 0;
}

static const struct iio_info adis_extd_info = {
	.read_raw = &adis_extd_read_raw,
	.write_raw = &adis_extd_write_raw,
	.update_scan_mode = adis_extd_update_scan_mode_burst,
	.debugfs_reg_access = adis_debugfs_reg_access,
};

/**
 * @brief Performs checksum validation for burst data.
 * @param buffer - The buffer containing the burst data.
 * @param size   - The size of buffer.
 * @return true in case the checksum is correct, false otherwise.
 */
static bool adis_extd_validate_checksum(const u8 *buffer, u16 size, u16 idx)
{
	u16 i;
	u16 checksum =
		get_unaligned_be16(&buffer[size - ADIS_EXTD_CHECKSUM_SIZE]);
	for (i = idx; i < size - ADIS_EXTD_CHECKSUM_SIZE; i++)
		checksum -= buffer[i];
	return (checksum == 0);
}

/**
 * @brief Read a single burst data set and push it to buffer.
 * @param adis_extd - The adis extended device structure.
 * @param buffer    - The buffer to be filled with burst data.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_burst_data(struct adis_extd *adis_extd,
				     __be16 **buffer, bool pop, bool burst_request)
{
	bool valid;
	struct adis *adis = &adis_extd->adis;
	int ret;
	u16 idx = 0;
	unsigned int burst_length;

	if (pop)
		ret = spi_sync(adis->spi, &adis->msg);
	else
		ret = spi_sync(adis->spi, &adis_extd->msg);
	if (ret)
		return ret;

	if (burst_request && adis_extd->info->has_fifo)
		return -EAGAIN;

	burst_length = adis->data->burst_len + adis->burst_extra_len;

	if(adis_extd->info->has_fifo)
		idx = 2;

	if (pop)
		valid = adis_extd_validate_checksum(adis->buffer, burst_length, idx);
	else
		valid = adis_extd_validate_checksum(adis_extd->buffer, burst_length, idx);

	if (!valid) {

		adis_extd->diag_flags.checksum_err = true;

		dev_err(&adis->spi->dev, "Invalid crc\n");
		return -EIO;
	}

	if (pop)
		*buffer = (__be16 *)adis->buffer;
	else
		*buffer = (__be16 *)adis_extd->buffer;

	adis_extd->diag_flags.checksum_err = false;

	if (pop)
		adis_extd_update_diag_flags(adis_extd,
				    get_unaligned_be16(adis->buffer));
	else
		adis_extd_update_diag_flags(adis_extd,
				    get_unaligned_be16(adis_extd->buffer));

	return 0;
}

static void adis_extd_burst32_check(struct adis_extd *adis_extd)
{
	struct adis *adis = &adis_extd->adis;
	int ret;
	if (adis_extd->lsb_flag && !adis_extd->burst32) {
		ret = adis_extd_write_burst32(adis_extd, 1);
		if (ret)
			return;
		adis->xfer[1].len += adis_extd->info->burst32_extra_bytes;
		adis_extd->xfer[1].len += adis_extd->info->burst32_extra_bytes;
		return;
	}

	if (!adis_extd->lsb_flag && adis_extd->burst32) {
		ret = adis_extd_write_burst32(adis_extd, 0);
		if (ret)
			return;
		adis->xfer[1].len -= adis_extd->info->burst32_extra_bytes;
		adis_extd->xfer[1].len -= adis_extd->info->burst32_extra_bytes;
	}
}

/**
 * @brief Read a single burst data set and push it to buffer.
 * @param indio_dev - The iio device.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_trigger_push_single_sample(struct iio_dev *indio_dev,
		struct iio_poll_func *pf, bool pop, bool burst_request)
{
	int ret, bit, i = 0;
	u8 buff_idx;
	u32 res1;
	u32 res2;
	u16 current_data_cntr;
	__be16 *buffer;
	struct adis_extd *adis_extd = iio_priv(indio_dev);

	const u8 temp_offset = adis_extd->burst32 ? 13 : 7;
	const u8 data_cntr_offset = adis_extd->burst32 ? 14 : 8;

	ret = adis_extd_read_burst_data(adis_extd, &buffer, pop, burst_request);
	if (ret)
		return ret;

	current_data_cntr = get_unaligned_be16(&buffer[data_cntr_offset]);

	if (adis_extd->data_cntr) {
		if (current_data_cntr == adis_extd->data_cntr)
			return -EAGAIN;
		if (current_data_cntr > adis_extd->data_cntr) {
			if (adis_extd->sync_mode != ADIS_EXTD_SYNC_SCALED) {
				adis_extd->samples_lost +=
					current_data_cntr -
					adis_extd->data_cntr - 1;
			} else {
				res1 = (current_data_cntr -
					adis_extd->data_cntr) *
				       49;
				res2 = DIV_ROUND_CLOSEST(1000000, adis_extd->sampling_frequency);

				if (res1 > res2) {
					adis_extd->samples_lost += res1 / res2;
					if (res1 % res2 < res2 / 2)
						adis_extd->samples_lost--;
				}
			}
		} else {
			if (adis_extd->sync_mode != ADIS_EXTD_SYNC_SCALED)
				adis_extd->samples_lost +=
					0xFFFFU - adis_extd->data_cntr +
					current_data_cntr;
		}
	}

	adis_extd->data_cntr = current_data_cntr;

	for_each_set_bit(bit, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		switch (bit) {
		case ADIS_EXTD_TEMP:
			/* upper not used */
			adis_extd->data[i++] = 0;
			adis_extd->data[i++] = buffer[temp_offset];
			break;
		case ADIS_EXTD_GYRO_X ... ADIS_EXTD_ACCEL_Z:
			/*
			 * The first 2 bytes on the received data are the
			 * DIAG_STAT reg, hence the +1 offset here...
			 */
			if (adis_extd->burst_sel) {
				adis_extd->data[i++] = 0;
				adis_extd->data[i++] = 0;
			} else {
				if (adis_extd->burst32) {
					/* upper 16 */
					adis_extd->data[i++] =
						buffer[bit * 2 + 2];
					/* lower 16 */
					adis_extd->data[i++] =
						buffer[bit * 2 + 1];
				} else {
					adis_extd->data[i++] = buffer[bit + 1];
					/* lower not used */
					adis_extd->data[i++] = 0;
				}
			}
			break;

		case ADIS_EXTD_DELTA_ANGL_X ... ADIS_EXTD_DELTA_VEL_Z:
			if (!adis_extd->burst_sel) {
				adis_extd->data[i++] = 0;
				adis_extd->data[i++] = 0;
			} else {
				buff_idx = bit - ADIS_EXTD_DELTA_ANGL_X;
				if (adis_extd->burst32) {
					/* upper 16 */
					adis_extd->data[i++] =
						buffer[buff_idx * 2 + 2];
					/* lower 16 */
					adis_extd->data[i++] =
						buffer[buff_idx * 2 + 1];
				} else {
					adis_extd->data[i++] =
						buffer[buff_idx + 1];
					/* lower not used */
					adis_extd->data[i++] = 0;
				}
			}
			break;
		}
	}

	return iio_push_to_buffers_with_timestamp(indio_dev, adis_extd->data,
			pf->timestamp);
}

/**
 * @brief Adis extended trigger handler with data ready interrupt.
 */
static irqreturn_t adis_extd_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adis_extd *adis_extd = iio_priv(indio_dev);

	adis_extd_burst32_check(adis_extd);

	adis_dev_lock(&adis_extd->adis);
	adis_extd_trigger_push_single_sample(indio_dev, pf, true, false);
	adis_dev_unlock(&adis_extd->adis);

	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

/**
 * @brief Adis extended trigger handler with fifo interrupt.
 */
static irqreturn_t adis_extd_trigger_handler_with_fifo(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adis_extd *adis_extd = iio_priv(indio_dev);
	u64 fifo_cnt;
	int i;

	adis_extd_burst32_check(adis_extd);

	adis_extd_read_fifo_cnt(adis_extd, &fifo_cnt);
	fsleep(10);
	adis_dev_lock(&adis_extd->adis);
	if (fifo_cnt > 2) {
		/* burst request */
		adis_extd_trigger_push_single_sample(indio_dev, pf, true, true);
		fsleep(10);
		for (i = 0; i < fifo_cnt - 1; i++) {
			adis_extd_trigger_push_single_sample(indio_dev, pf, true, false);
			fsleep(10);
		}
		/* read without fifo pop */
		adis_extd_trigger_push_single_sample(indio_dev, pf, false, false);
	}
	adis_dev_unlock(&adis_extd->adis);
	fsleep(10);

	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

/**
 * @brief Disable clock for adis extended device
 * @param data - Clock data.
 */
static void adis_extd_disable_clk(void *data)
{
	clk_disable_unprepare((struct clk *)data);
}

/**
 * @brief Write 4khz internal sync enable bit value.
 * @param arg       - The adis extended device.
 * @param sync_4khz - The 4khz internal sync enable bit value (1/0 - enabled/disabled).
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_write_sync_4khz(void *arg, u64 sync_4khz)
{
	struct adis_extd *adis_extd = arg;
	int ret;

	ret = adis_extd_write_field_u32(
		      adis_extd, adis_extd->info->field_map->sync_4khz, sync_4khz);
	if (ret)
		return ret;

	adis_extd->int_clk = sync_4khz ? 4000 : 2000;

	fsleep(adis_extd->info->timeouts->msc_reg_update_us);

	return adis_extd_read_sampling_freq(adis_extd,
					    &adis_extd->sampling_frequency);
}

/**
 * @brief Sync mode configuration for adis extended device.
 * @param adis_extd - Adis extended device structure.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_config_sync_mode(struct adis_extd *adis_extd)
{
	int ret;
	struct device *dev = &adis_extd->adis.spi->dev;
	u32 sync_mode;
	u32 ext_clk_freq = 0;

	if(adis_extd->info->int_clk == 4000) {
		/* Enable 4KHz sync signal by default */
		ret = adis_extd_write_sync_4khz(adis_extd, 1);
		if(ret)
			return ret;
	}

	adis_extd->int_clk = adis_extd->info->int_clk;

	ret = device_property_read_u32(dev, "adi,sync-mode", &sync_mode);
	if (ret)
		return 0;

	if (sync_mode > adis_extd->info->sync_mode_max) {
		dev_err(dev, "Invalid sync mode: %u for %s\n", sync_mode,
			adis_extd->info->name);
		return -EINVAL;
	}

	/* All the other modes require external input signal */
	if (sync_mode == ADIS_EXTD_SYNC_DIRECT ||
	    sync_mode == ADIS_EXTD_SYNC_SCALED) {
		struct clk *clk = devm_clk_get(dev, NULL);

		if (IS_ERR(clk))
			return PTR_ERR(clk);

		ret = clk_prepare_enable(clk);
		if (ret)
			return ret;

		ret = devm_add_action_or_reset(dev, adis_extd_disable_clk, clk);
		if (ret)
			return ret;

		ext_clk_freq = clk_get_rate(clk);
	}

	return adis_extd_update_sync_mode(adis_extd, sync_mode, ext_clk_freq);
}

/**
 * @brief IRQ configuration for adis extended device.
 * @param adis_extd - The adis extended device structure.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_config_irq_pin(struct adis_extd *adis_extd)
{
	int ret;
	struct irq_data *desc;
	u32 irq_type;
	u8 polarity;
	struct spi_device *spi = adis_extd->adis.spi;

	desc = irq_get_irq_data(spi->irq);
	if (!desc) {
		dev_err(&spi->dev, "Could not find IRQ %d\n", spi->irq);
		return -EINVAL;
	}

	irq_type = irqd_get_trigger_type(desc);
	if (irq_type == IRQ_TYPE_EDGE_RISING) {
		polarity = 1;
		adis_extd->adis.irq_flag = IRQF_TRIGGER_RISING;
	} else if (irq_type == IRQ_TYPE_EDGE_FALLING) {
		polarity = 0;
		adis_extd->adis.irq_flag = IRQF_TRIGGER_FALLING;
	} else {
		dev_err(&spi->dev, "Invalid interrupt type 0x%x specified\n",
			irq_type);
		return -EINVAL;
	}

	adis_extd->adis.irq_flag |= IRQF_ONESHOT;

	if (adis_extd->info->has_fifo) {
		return adis_extd_write_fifo_wm_int_pol(adis_extd, polarity);
		if (ret)
			return ret;
	}

	return adis_extd_write_dr_polarity(adis_extd, polarity);
}

int adis_extd_init_values(struct adis_extd *adis_extd)
{
	int ret;
	ret = adis_extd_write_burst32(adis_extd, 0);
	if (ret)
		return ret;

	adis_extd->lsb_flag = 0;

	return 0;
}

/**
 * @brief Initialize adis extended device.
 * @param spi      - SPI device.
 * @param attr_arr - Debug attributes array to be initialized.
 * @param arr_size - Size of attr_arr.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_init(struct spi_device *spi,
		   const enum adis_extd_debug_attr *attr_arr, u16 arr_size)
{
	struct iio_dev *indio_dev;
	struct adis_extd *adis_extd;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adis_extd));
	if (!indio_dev)
		return -ENOMEM;

	adis_extd = iio_priv(indio_dev);

	adis_extd->info = device_get_match_data(&spi->dev);
	if (!adis_extd->info)
		return -EINVAL;

	ret = adis_init(&adis_extd->adis, indio_dev, spi,
			&adis_extd->info->adis_data);
	if (ret)
		return ret;

	indio_dev->name = adis_extd->info->name;
	indio_dev->channels = adis_extd->info->channels;
	indio_dev->num_channels = adis_extd->info->num_channels;
	indio_dev->info = &adis_extd_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = __adis_initial_startup(&adis_extd->adis);
	if (ret)
		return ret;

	ret = adis_extd_config_irq_pin(adis_extd);
	if (ret)
		return ret;

	ret = adis_extd_config_sync_mode(adis_extd);
	if (ret)
		return ret;

	ret = adis_extd_write_burst32(adis_extd, 0);
	if (ret)
		return ret;

	if (adis_extd->info->has_fifo)
		ret = devm_adis_setup_buffer_and_trigger(&adis_extd->adis, indio_dev,
				adis_extd_trigger_handler_with_fifo,
				&adis_extd_buff_ops);
	else
		ret = devm_adis_setup_buffer_and_trigger(&adis_extd->adis, indio_dev,
				adis_extd_trigger_handler,
				&adis_extd_buff_ops);
	if (ret)
		return ret;

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret)
		return ret;

	adis_extd_debugfs_init(indio_dev, attr_arr, arr_size);

	return 0;
}
EXPORT_SYMBOL_NS_GPL(adis_extd_init, IIO_ADIS_EXTDLIB);

MODULE_AUTHOR("Ramona Bolboaca <ramona.bolboaca@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADIS extended IMU driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_ADISLIB);
