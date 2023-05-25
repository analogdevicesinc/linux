
// SPDX-License-Identifier: GPL-2.0
/*
 * ADIS extended debugfs APIs
 *
 * Copyright 2023 Analog Devices Inc.
 */
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/imu/adis.h>
#include <linux/iio/imu/adis_extd.h>
#include <linux/spi/spi.h>

/**
 * @brief Read diag status register and update device diag flags.
 * @param adis_extd  - The adis extended device.
 * @param diag_flags - The read diag flags.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_stat(struct adis_extd *adis_extd,
				    struct adis_extd_diag_flags *diag_flags)
{
	struct adis_extd_field field = adis_extd->info->field_map->diag_stat;
	unsigned int field_val;
	int ret;

	/* Diag flags will be updated in buffer readings */
	if (adis_extd->buffer_enabled) {
		*diag_flags = adis_extd->diag_flags;
		return 0;
	}

	ret = adis_read_reg(&adis_extd->adis, field.reg_addr, &field_val,
			    field.reg_size);
	if (ret)
		return ret;

	adis_extd_update_diag_flags(adis_extd, field_val);
	*diag_flags = adis_extd->diag_flags;

	return 0;
}

/**
 * @brief Diagnosis: read sensor initialization failure flag value.
 * @param arg               - The adis extended device.
 * @param snsr_init_failure - Sensor initialization failure flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_snsr_init_failure(void *arg,
						 u64 *snsr_init_failure)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*snsr_init_failure = diag_flags.snsr_init_failure;

	return 0;
}

/**
 * @brief Diagnosis: read data path overrun flag value.
 * @param arg                   - The adis extended device.
 * @param data_path_overrun_err - Data path overrun flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_data_path_overrun(void *arg,
						 u64 *data_path_overrun_err)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*data_path_overrun_err = diag_flags.data_path_overrun;

	return 0;
}

/**
 * @brief Diagnosis: read flash memory update error flag value.
 * @param arg                    - The adis extended device.
 * @param fls_mem_update_failure - Flash memory update error flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int
adis_extd_read_diag_fls_mem_update_failure(void *arg,
					   u64 *fls_mem_update_failure)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*fls_mem_update_failure = diag_flags.fls_mem_update_failure;

	return 0;
}

/**
 * @brief Diagnosis: read spi communication error flag value.
 * @param arg          - The adis extended device.
 * @param spi_comm_err - Spi communication error flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_spi_comm_err(void *arg, u64 *spi_comm_err)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*spi_comm_err = diag_flags.spi_comm_err;

	return 0;
}

/**
 * @brief Diagnosis: read standby mode flag value.
 * @param arg          - The adis extended device.
 * @param standby_mode - Standby mode flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_standby_mode(void *arg, u64 *standby_mode)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*standby_mode = diag_flags.standby_mode;

	return 0;
}

/**
 * @brief Diagnosis: read sensor self test error flag value.
 * @param arg          - The adis extended device.
 * @param snsr_failure - Sensor self test error flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_snsr_failure(void *arg, u64 *snsr_failure)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*snsr_failure = diag_flags.snsr_failure;

	return 0;
}

/**
 * @brief Diagnosis: read flash memory test error flag value.
 * @param arg         - The adis extended device.
 * @param mem_failure - Flash memory test error flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_mem_failure(void *arg, u64 *mem_failure)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*mem_failure = diag_flags.mem_failure;

	return 0;
}

/**
 * @brief Diagnosis: read clock error flag value.
 * @param arg     - The adis extended device.
 * @param clk_err - Clock error flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_clk_err(void *arg, u64 *clk_err)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*clk_err = diag_flags.clk_err;

	return 0;
}

/**
 * @brief Diagnosis: read gyroscope1 self test error flag value.
 * @param arg           - The adis extended device.
 * @param gyro1_failure - Gyroscope1 self test error flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_gyro1_failure(void *arg, u64 *gyro1_failure)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*gyro1_failure = diag_flags.gyro1_failure;

	return 0;
}

/**
 * @brief Diagnosis: read gyroscope2 self test error flag value.
 * @param arg           - The adis extended device.
 * @param gyro2_failure - Gyroscope2 self test error flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_gyro2_failure(void *arg, u64 *gyro2_failure)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*gyro2_failure = diag_flags.gyro2_failure;

	return 0;
}

/**
 * @brief Diagnosis: read accelerometer self test error flag value.
 * @param arg          - The adis extended device.
 * @param accl_failure - Accelerometer self test error flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_accl_failure(void *arg, u64 *accl_failure)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*accl_failure = diag_flags.accl_failure;

	return 0;
}

/**
 * @brief Diagnosis: read X-Axis Gyroscope failure flag value.
 * @param arg                 - The adis extended device.
 * @param x_axis_gyro_failure - X-Axis Gyroscope failure flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_x_axis_gyro_failure(void *arg,
						   u64 *x_axis_gyro_failure)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*x_axis_gyro_failure = diag_flags.x_axis_gyro_failure;

	return 0;
}

/**
 * @brief Diagnosis: read Y-Axis Gyroscope failure flag value.
 * @param arg                 - The adis extended device.
 * @param y_axis_gyro_failure - Y-Axis Gyroscope failure flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_y_axis_gyro_failure(void *arg,
						   u64 *y_axis_gyro_failure)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*y_axis_gyro_failure = diag_flags.y_axis_gyro_failure;

	return 0;
}

/**
 * @brief Diagnosis: read Z-Axis Gyroscope failure flag value.
 * @param arg                 - The adis extended device.
 * @param z_axis_gyro_failure - Z-Axis Gyroscope failure flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_z_axis_gyro_failure(void *arg,
						   u64 *z_axis_gyro_failure)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*z_axis_gyro_failure = diag_flags.z_axis_gyro_failure;

	return 0;
}

/**
 * @brief Diagnosis: read X-Axis Accelerometer failure flag value.
 * @param arg                 - The adis extended device.
 * @param x_axis_accl_failure - X-Axis Accelerometer failure flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_x_axis_accl_failure(void *arg,
						   u64 *x_axis_accl_failure)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*x_axis_accl_failure = diag_flags.x_axis_accl_failure;

	return 0;
}

/**
 * @brief Diagnosis: read Y-Axis Accelerometer failure flag value.
 * @param arg                 - The adis extended device.
 * @param y_axis_accl_failure - Y-Axis Accelerometer failure flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_y_axis_accl_failure(void *arg,
						   u64 *y_axis_accl_failure)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*y_axis_accl_failure = diag_flags.y_axis_accl_failure;

	return 0;
}

/**
 * @brief Diagnosis: read Z-Axis Accelerometer failure flag value.
 * @param arg                 - The adis extended device.
 * @param z_axis_accl_failure - Z-Axis Accelerometer failure flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_z_axis_accl_failure(void *arg,
						   u64 *z_axis_accl_failure)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*z_axis_accl_failure = diag_flags.z_axis_accl_failure;

	return 0;
}

/**
 * @brief Diagnosis: read ADuC microcontroller fault flag value.
 * @param arg            - The adis extended device.
 * @param aduc_mcu_fault - ADuC microcontroller fault flag value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_diag_aduc_mcu_fault(void *arg, u64 *aduc_mcu_fault)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_diag_flags diag_flags;
	int ret;

	ret = adis_extd_read_diag_stat(adis_extd, &diag_flags);
	if (ret)
		return ret;

	*aduc_mcu_fault = diag_flags.aduc_mcu_fault;

	return 0;
}

/**
 * @brief Diagnosis: read checksum error flag value.
 * @param arg          - The adis extended device.
 * @param checksum_err - Checksum error flag value.
 */
static int adis_extd_read_diag_checksum_err(void *arg, u64 *checksum_err)
{
	struct adis_extd *adis_extd = arg;

	*checksum_err = adis_extd->diag_flags.checksum_err;

	return 0;
}

/**
 * @brief Diagnosis: read flash memory write counts exceeded flag value.
 * @param arg                   - The adis extended device.
 * @param fls_mem_wr_cnt_exceed - Flash memory write counts exceeded flag value.
 */
static int adis_extd_read_diag_fls_mem_wr_cnt_exceed(void *arg,
						     u64 *fls_mem_wr_cnt_exceed)
{
	struct adis_extd *adis_extd = arg;

	*fls_mem_wr_cnt_exceed = adis_extd->diag_flags.fls_mem_wr_cnt_exceed;

	return 0;
}

/**
 * @brief Read raw time stamp data.
 * @param arg        - The adis extended device.
 * @param time_stamp - The raw read value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_time_stamp(void *arg, u64 *time_stamp)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->time_stamp, &field_val);
	if (ret)
		return ret;

	*time_stamp = field_val;

	return 0;
}

/**
 * @brief Read data counter value.
 * @param arg       - The adis extended device.
 * @param data_cntr - The read value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_data_cntr(void *arg, u64 *data_cntr)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->data_cntr, &field_val);
	if (ret)
		return ret;

	*data_cntr = field_val;

	return 0;
}

/**
 * @brief Read output FIFO sample count.
 * @param arg      - The adis extended device.
 * @param fifo_cnt - The raw read value.
 * @return 0 in case of success, error code otherwise.
 */
int adis_extd_read_fifo_cnt(void *arg, u64 *fifo_cnt)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->fifo_cnt, &field_val);
	if (ret)
		return ret;

	*fifo_cnt = field_val;

	return 0;
}

/**
 * @brief Read current sample SPI transaction checksum.
 * @param arg      - The adis extended device.
 * @param checksum - The raw read value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_spi_chksum(void *arg, u64 *spi_chksum)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->spi_chksum, &field_val);
	if (ret)
		return ret;

	*spi_chksum = field_val;

	return 0;
}

/*
 * @brief Read FIFO enable bit value.
 * @param arg     - The adis extended device.
 * @param fifo_en - The FIFO enable bit read value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_fifo_en(void *arg, u64 *fifo_en)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->fifo_en, &field_val);
	if (ret)
		return ret;

	*fifo_en = field_val;

	return 0;
}

/**
 * @brief Read FIFO overflow bit value.
 * @param arg           - The adis extended device.
 * @param fifo_overflow - The FIFO overflow bit read value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_fifo_overflow(void *arg, u64 *fifo_overflow)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->fifo_overflow,
		&field_val);
	if (ret)
		return ret;

	*fifo_overflow = field_val;

	return 0;
}

/**
 * @brief Read FIFO watermark interrupt enable bit value.
 * @param arg            - The adis extended device.
 * @param fifo_wm_int_en - The FIFO watermark interrupt enable bit read value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_fifo_wm_int_en(void *arg, u64 *fifo_wm_int_en)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->fifo_wm_int_en,
		&field_val);
	if (ret)
		return ret;

	*fifo_wm_int_en = field_val;

	return 0;
}

/**
 * @brief Read FIFO watermark interrupt polarity bit value.
 * @param arg             - The adis extended device.
 * @param fifo_wm_int_pol - The FIFO watermark interrupt polarity bit read value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_fifo_wm_int_pol(void *arg, u64 *fifo_wm_int_pol)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->fifo_wm_int_pol,
		&field_val);
	if (ret)
		return ret;

	*fifo_wm_int_pol = field_val;

	return 0;
}

/**
 * @brief Read FIFO watermark threshold level value.
 * @param arg         - The adis extended device.
 * @param fifo_wm_lvl - The FIFO watermark threshold level read value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_fifo_wm_lvl(void *arg, u64 *fifo_wm_lvl)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->fifo_wm_lvl, &field_val);
	if (ret)
		return ret;

	*fifo_wm_lvl = field_val;

	return 0;
}

#define MAX_TXT_SIZE 50
static const char *const gyro_meas_range_txt[] = {
	"+/-125_degrees_per_sec",
	"+/-500_degrees_per_sec",
	"",
	"+/-2000_degrees_per_sec",
};

/**
 * @brief Read gyroscope measurement range.
 */
static ssize_t adis_extd_read_gyro_meas_range(struct file *file,
					      char __user *userbuf,
					      size_t count, loff_t *ppos)
{
	struct adis_extd *adis_extd = file->private_data;
	size_t len;
	int ret;
	struct adis_extd_field field =
		adis_extd->info->field_map->gyro_meas_range;
	unsigned int field_val;
	char buf[MAX_TXT_SIZE];

	ret = adis_read_reg(&adis_extd->adis, field.reg_addr, &field_val,
			    field.reg_size);
	if (ret)
		return ret;

	field_val = adis_extd_field_get(field_val, field.field_mask);

	len = snprintf(buf, strlen(gyro_meas_range_txt[field_val]) + 1, "%s\n",
		       gyro_meas_range_txt[field_val]);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

/**
 * @brief Read data ready polarity encoded value.
 * @param arg         - The adis extended device.
 * @param dr_polarity - The data ready polarity encoded value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_dr_polarity(void *arg, u64 *dr_polarity)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->dr_polarity, &field_val);
	if (ret)
		return ret;

	*dr_polarity = field_val;

	return 0;
}

/**
 * @brief Read sync polarity encoded value.
 * @param arg           - The adis extended device.
 * @param sync_polarity - The sync polarity encoded value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_sync_polarity(void *arg, u64 *sync_polarity)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->sync_polarity,
		&field_val);
	if (ret)
		return ret;

	*sync_polarity = field_val;

	return 0;
}

/**
 * @brief Write sync polarity encoded value.
 * @param arg           - The adis extended device.
 * @param sync_polarity - The sync polarity encoded value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_sync_polarity(void *arg, u64 sync_polarity)
{
	struct adis_extd *adis_extd = arg;
	int ret;

	ret = adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->sync_polarity,
		sync_polarity);
	if (ret)
		return ret;

	fsleep(adis_extd->info->timeouts->msc_reg_update_us);

	return 0;
}

/**
 * @brief Read internal sensor bandwidth encoded value.
 * @param arg     - The adis extended device.
 * @param sens_bw - The internal sensor bandwidth encoded value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_sens_bw(void *arg, u64 *sens_bw)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->sens_bw, &field_val);
	if (ret)
		return ret;

	*sens_bw = field_val;

	return 0;
}

/**
 * @brief Write internal sensor bandwidth encoded value.
 * @param arg     - The adis extended device.
 * @param sens_bw - The internal sensor bandwidth encoded value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_sens_bw(void *arg, u64 sens_bw)
{
	struct adis_extd *adis_extd = arg;
	int ret;

	ret = adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->sens_bw, sens_bw);
	if (ret)
		return ret;

	msleep(adis_extd->info->timeouts->sens_bw_update_ms);

	return 0;
}

/**
 * @brief Read point of percussion alignment enable bit value.
 * @param arg               - The adis extended device.
 * @param pt_of_perc_algnmt - The point of percussion alignment enable bit
 *                            value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_pt_of_perc_algnmt(void *arg, u64 *pt_of_perc_algnmt)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->pt_of_perc_algnmt,
		&field_val);
	if (ret)
		return ret;

	*pt_of_perc_algnmt = field_val;

	return 0;
}

/**
 * @brief Write point of percussion alignment enable bit value.
 * @param arg               - The adis extended device.
 * @param pt_of_perc_algnmt - The point of percussion alignment enable bit
 *			      value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_pt_of_perc_algnmt(void *arg, u64 pt_of_perc_algnmt)
{
	struct adis_extd *adis_extd = arg;
	int ret;

	ret = adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->pt_of_perc_algnmt,
		pt_of_perc_algnmt);
	if (ret)
		return ret;

	fsleep(adis_extd->info->timeouts->msc_reg_update_us);

	return 0;
}

/**
 * @brief Read linear acceleration compensation enable bit value.
 * @param arg              - The adis extended device.
 * @param linear_accl_comp - The linear acceleration compensation enable bit
 *			     value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_linear_accl_comp(void *arg, u64 *linear_accl_comp)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->linear_accl_comp,
		&field_val);
	if (ret)
		return ret;

	*linear_accl_comp = field_val;

	return 0;
}

/**
 * @brief Write linear acceleration compensation enable bit value.
 * @param arg              - The adis extended device.
 * @param linear_accl_comp - The linear acceleration compensation enable bit
 *			     value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_linear_accl_comp(void *arg, u64 linear_accl_comp)
{
	struct adis_extd *adis_extd = arg;
	int ret;

	ret = adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->linear_accl_comp,
		linear_accl_comp);
	if (ret)
		return ret;

	fsleep(adis_extd->info->timeouts->msc_reg_update_us);

	return 0;
}

/**
 * @brief Read burst selection encoded value.
 * @param arg       - The adis extended device.
 * @param burst_sel - The burst selection encoded value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_burst_sel(void *arg, u64 *burst_sel)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->burst_sel, &field_val);
	if (ret)
		return ret;

	*burst_sel = field_val;

	return 0;
}

/**
 * @brief Write burst selection encoded value.
 * @param arg       - The adis extended device.
 * @param burst_sel - The burst selection encoded value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_burst_sel(void *arg, u64 burst_sel)
{
	struct adis_extd *adis_extd = arg;
	int ret;

	ret = adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->burst_sel, burst_sel);
	if (ret)
		return ret;

	adis_extd->burst_sel = burst_sel;

	fsleep(adis_extd->info->timeouts->msc_reg_update_us);

	return 0;
}

/**
 * @brief Read burst32 enable bit value.
 * @param arg     - The adis extended device.
 * @param burst32 - The burst32 enable bit value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_burst32(void *arg, u64 *burst32)
{
	struct adis_extd *adis_extd = arg;
	struct adis *adis = &adis_extd->adis;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->burst32, &field_val);
	if (ret)
		return ret;

	*burst32 = field_val;

	if (*burst32)
		adis->burst_extra_len = adis_extd->info->burst32_extra_bytes;
	else
		adis->burst_extra_len = 0;

	return 0;
}

/**
 * @brief Read timestamp32 enable bit value.
 * @param arg         - The adis extended device.
 * @param timestamp32 - The timestamp32 enable bit value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_timestamp32(void *arg, u64 *timestamp32)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->timestamp32, &field_val);
	if (ret)
		return ret;

	*timestamp32 = field_val;

	return 0;
}

/**
 * @brief Write timestamp32 enable bit value.
 * @param arg         - The adis extended device.
 * @param timestamp32 - The timestamp32 enable bit value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_timestamp32(void *arg, u64 timestamp32)
{
	struct adis_extd *adis_extd = arg;
	int ret;

	ret = adis_extd_write_field_u32(adis_extd,
					adis_extd->info->field_map->timestamp32,
					timestamp32);
	if (ret)
		return ret;

	fsleep(adis_extd->info->timeouts->msc_reg_update_us);

	return 0;
}

/**
 * @brief Read 4khz internal sync enable bit value.
 * @param arg       - The adis extended device.
 * @param sync_4khz - The 4khz internal sync enable bit value (1/0 - enabled/disabled).
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_sync_4khz(void *arg, u64 *sync_4khz)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->sync_4khz, &field_val);
	if (ret)
		return ret;

	*sync_4khz = field_val;

	return 0;
}

/**
 * @brief Write synchronization mode encoded value.
 * @param arg       - The adis extended device.
 * @param sync_mode - The synchronization mode encoded value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_sync_mode(void *arg, u64 sync_mode)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_update_sync_mode(adis_extd, sync_mode,
					  adis_extd->ext_clk);
}

/**
 * @brief Read time base control value.
 * @param arg           - The adis extended device.
 * @param bias_corr_tbc - The time base control read value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_bias_corr_tbc(void *arg, u64 *bias_corr_tbc)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->bias_corr_tbc,
		&field_val);
	if (ret)
		return ret;

	*bias_corr_tbc = field_val;

	return 0;
}

/**
 * @brief Write time base control value.
 * @param arg           - The adis extended device.
 * @param bias_corr_tbc - The time base control value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_bias_corr_tbc(void *arg, u64 bias_corr_tbc)
{
	struct adis_extd *adis_extd = arg;

	if (bias_corr_tbc > adis_extd->info->bias_corr_tbc_max)
		return -EINVAL;

	return adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->bias_corr_tbc,
		bias_corr_tbc);
}

/**
 * @brief Read x axis gyroscope bias correction enable bit value.
 * @param arg             - The adis extended device.
 * @param bias_corr_en_xg - The x axis gyroscope bias correction enable bit read
 *                          value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_bias_corr_en_xg(void *arg, u64 *bias_corr_en_xg)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->bias_corr_en_xg,
		&field_val);
	if (ret)
		return ret;

	*bias_corr_en_xg = field_val;

	return 0;
}

/**
 * @brief Write x axis gyroscope bias correction enable bit value.
 * @param arg             - The adis extended device.
 * @param bias_corr_en_xg - The x axis gyroscope bias correction enable bit
 *                          value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_bias_corr_en_xg(void *arg, u64 bias_corr_en_xg)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->bias_corr_en_xg,
		bias_corr_en_xg);
}

/**
 * @brief Read y axis gyroscope bias correction enable bit value.
 * @param arg             - The adis extended device.
 * @param bias_corr_en_yg - The y axis gyroscope bias correction enable bit read
 *                          value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_bias_corr_en_yg(void *arg, u64 *bias_corr_en_yg)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->bias_corr_en_yg,
		&field_val);
	if (ret)
		return ret;

	*bias_corr_en_yg = field_val;

	return 0;
}

/**
 * @brief Write y axis gyroscope bias correction enable bit value.
 * @param arg             - The adis extended device.
 * @param bias_corr_en_xg - The y axis gyroscope bias correction enable bit
 *                          value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_bias_corr_en_yg(void *arg, u64 bias_corr_en_yg)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->bias_corr_en_yg,
		bias_corr_en_yg);
}

/**
 * @brief Read z axis gyroscope bias correction enable bit value.
 * @param arg             - The adis extended device.
 * @param bias_corr_en_zg - The z axis gyroscope bias correction enable bit read
 *                          value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_bias_corr_en_zg(void *arg, u64 *bias_corr_en_zg)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->bias_corr_en_zg,
		&field_val);
	if (ret)
		return ret;

	*bias_corr_en_zg = field_val;

	return 0;
}

/**
 * @brief Write z axis gyroscope bias correction enable bit value.
 * @param arg             - The adis extended device.
 * @param bias_corr_en_zg - The z axis gyroscope bias correction enable bit
 *                          value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_bias_corr_en_zg(void *arg, u64 bias_corr_en_zg)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->bias_corr_en_zg,
		bias_corr_en_zg);
}

/**
 * @brief Read x axis accelerometer bias correction enable bit value.
 * @param arg             - The adis extended device.
 * @param bias_corr_en_xa - The x axis accelerometer bias correction enable bit read
 *                          value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_bias_corr_en_xa(void *arg, u64 *bias_corr_en_xa)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->bias_corr_en_xa,
		&field_val);
	if (ret)
		return ret;

	*bias_corr_en_xa = field_val;

	return 0;
}

/**
 * @brief Write x axis accelerometer bias correction enable bit value.
 * @param arg             - The adis extended device.
 * @param bias_corr_en_xa - The x axis accelerometer bias correction enable bit
 *                          value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_bias_corr_en_xa(void *arg, u64 bias_corr_en_xa)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->bias_corr_en_xa,
		bias_corr_en_xa);
}

/**
 * @brief Read y axis accelerometer bias correction enable bit value.
 * @param arg             - The adis extended device.
 * @param bias_corr_en_ya - The y axis accelerometer bias correction enable bit read
 *                          value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_bias_corr_en_ya(void *arg, u64 *bias_corr_en_ya)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->bias_corr_en_ya,
		&field_val);
	if (ret)
		return ret;

	*bias_corr_en_ya = field_val;

	return 0;
}

/**
 * @brief Write y axis accelerometer bias correction enable bit value.
 * @param arg             - The adis extended device.
 * @param bias_corr_en_ya - The y axis accelerometer bias correction enable bit
 *                          value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_bias_corr_en_ya(void *arg, u64 bias_corr_en_ya)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->bias_corr_en_ya,
		bias_corr_en_ya);
}

/**
 * @brief Read z axis accelerometer bias correction enable bit value.
 * @param arg             - The adis extended device.
 * @param bias_corr_en_za - The z axis accelerometer bias correction enable bit read
 *                          value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_bias_corr_en_za(void *arg, u64 *bias_corr_en_za)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->bias_corr_en_za,
		&field_val);
	if (ret)
		return ret;

	*bias_corr_en_za = field_val;

	return 0;
}

/**
 * @brief Write z axis accelerometer bias correction enable bit value.
 * @param arg             - The adis extended device.
 * @param bias_corr_en_za - The z axis accelerometer bias correction enable bit
 *                          value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_bias_corr_en_za(void *arg, u64 bias_corr_en_za)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->bias_corr_en_za,
		bias_corr_en_za);
}

/**
 * @brief Command: bias correction update
 * @param arg - The adis extended device.
 * @param val - Value to write, any value will trigger the command.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_cmd_bias_corr_update(void *arg, u64 val)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_field field =
		adis_extd->info->field_map->bias_corr_update;

	return adis_write_reg(&adis_extd->adis, field.reg_addr,
			      field.field_mask, field.reg_size);
}

/**
 * @brief Command: factory calibration restore
 * @param arg - The adis extended device.
 * @param val - Value to write, any value will trigger the command.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_cmd_fact_calib_restore(void *arg, u64 val)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_field field =
		adis_extd->info->field_map->fact_calib_restore;
	int ret;

	ret = adis_write_reg(&adis_extd->adis, field.reg_addr, field.field_mask,
			     field.reg_size);
	if (ret)
		return ret;

	msleep(adis_extd->info->timeouts->fact_calib_restore_ms);

	ret = adis_extd_read_sampling_freq(adis_extd,
					   &adis_extd->sampling_frequency);

	if (ret)
		return ret;

	return adis_extd_init_values(adis_extd);
}

/**
 * @brief Command: sensor self test
 * @param arg - The adis extended device.
 * @param val - Value to write, any value will trigger the command.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_cmd_snsr_self_test(void *arg, u64 val)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_field field =
		adis_extd->info->field_map->snsr_self_test;
	int ret;

	ret = adis_write_reg(&adis_extd->adis, field.reg_addr, field.field_mask,
			     field.reg_size);
	if (ret)
		return ret;

	msleep(adis_extd->info->timeouts->self_test_ms);

	return 0;
}

/**
 * @brief Read flash memory write cycle counter value.
 * @param arg             - The adis extended device.
 * @param fls_mem_wr_cntr - The flash memory write cycle counter value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_fls_mem_wr_cntr(void *arg, u64 *fls_mem_wr_cntr)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->fls_mem_wr_cntr,
		&field_val);
	if (ret != 0 && ret != -EBUSY)
		return ret;

	if (ret == -EBUSY) {
		/* return stored value */
		*fls_mem_wr_cntr = adis_extd->flash_count;
		return 0;
	}

	adis_extd->flash_count = field_val;
	*fls_mem_wr_cntr = field_val;

	if (*fls_mem_wr_cntr > adis_extd->info->fls_mem_wr_cntr_max)
		adis_extd->diag_flags.fls_mem_wr_cnt_exceed = true;

	return 0;
}

/**
 * @brief Command: flash memory update
 * @param arg - The adis extended device.
 * @param val - Value to write, any value will trigger the command.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_cmd_fls_mem_update(void *arg, u64 val)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_field field =
		adis_extd->info->field_map->fls_mem_update;
	int ret;
	u64 fls_mem_wr_cntr;

	ret = adis_write_reg(&adis_extd->adis, field.reg_addr, field.field_mask,
			     field.reg_size);
	if (ret)
		return ret;

	msleep(adis_extd->info->timeouts->fls_mem_update_ms);

	return adis_extd_read_fls_mem_wr_cntr(adis_extd, &fls_mem_wr_cntr);
}

/**
 * @brief Command: flash memory test
 * @param arg - The adis extended device.
 * @param val - Value to write, any value will trigger the command.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_cmd_fls_mem_test(void *arg, u64 val)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_field field = adis_extd->info->field_map->fls_mem_test;
	int ret;

	ret = adis_write_reg(&adis_extd->adis, field.reg_addr, field.field_mask,
			     field.reg_size);
	if (ret)
		return ret;

	msleep(adis_extd->info->timeouts->fls_mem_test_ms);

	return 0;
}

/**
 * @brief Command: software reset.
 * @param arg - The adis extended device.
 * @param val - Value to write, any value will trigger the command.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_cmd_sw_res(void *arg, u64 val)
{
	struct adis_extd *adis_extd = arg;
	struct adis_extd_field field = adis_extd->info->field_map->sw_res;
	int ret;

	ret = adis_write_reg(&adis_extd->adis, field.reg_addr, field.field_mask,
			     field.reg_size);
	if (ret)
		return ret;

	msleep(adis_extd->info->timeouts->sw_reset_ms);

	return adis_extd_init_values(adis_extd);
}

/**
 * @brief Read firmware revision value.
 */
static ssize_t adis_extd_read_firm_rev(struct file *file, char __user *userbuf,
				       size_t count, loff_t *ppos)
{
	struct adis_extd *adis_extd = file->private_data;
	u32 field_val;
	char buf[7];
	size_t len;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->firm_rev, &field_val);
	if (ret)
		return ret;

	len = scnprintf(buf, sizeof(buf), "%x.%x\n", field_val >> 8,
			field_val & 0xff);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

/**
 * @brief Read firmware date.
 */
static ssize_t adis_extd_read_fw_date(struct file *file, char __user *userbuf,
				      size_t count, loff_t *ppos)
{
	struct adis_extd *adis_extd = file->private_data;
	u32 field_val;

	u16 year, month, day;
	char buf[12];
	size_t len;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->firm_y, &field_val);
	if (ret)
		return ret;
	year = field_val;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->firm_m, &field_val);
	if (ret)
		return ret;
	month = field_val;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->firm_d, &field_val);
	if (ret)
		return ret;
	day = field_val;

	len = snprintf(buf, sizeof(buf), "%.2x-%.2x-%.4x\n", month, day, year);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

/**
 * @brief Read product id value.
 * @param arg     - The adis extended device.
 * @param prod_id - The product id value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_prod_id(void *arg, u64 *prod_id)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->prod_id, &field_val);
	if (ret)
		return ret;

	*prod_id = field_val;

	return 0;
}

/**
 * @brief Read serial number value.
 * @param arg        - The adis extended device.
 * @param serial_num - The serial number value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_serial_num(void *arg, u64 *serial_num)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->serial_num, &field_val);
	if (ret)
		return ret;

	*serial_num = field_val;

	return 0;
}

/**
 * @brief Read user scratch register 1 value.
 * @param arg       - The adis extended device.
 * @param usr_scr_1 - The user scratch register 1 value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_usr_scr_1(void *arg, u64 *usr_scr_1)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->usr_scr_1, &field_val);
	if (ret)
		return ret;

	*usr_scr_1 = field_val;

	return 0;
}

/**
 * @brief Write user scratch register 1 value.
 * @param arg       - The adis extended device.
 * @param usr_scr_1 - The user scratch register 1 value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_usr_scr_1(void *arg, u64 usr_scr_1)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->usr_scr_1, usr_scr_1);
}

/**
 * @brief Read user scratch register 2 value.
 * @param arg       - The adis extended device.
 * @param usr_scr_2 - The user scratch register 2 value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_usr_scr_2(void *arg, u64 *usr_scr_2)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->usr_scr_2, &field_val);
	if (ret)
		return ret;

	*usr_scr_2 = field_val;

	return 0;
}

/**
 * @brief Write user scratch register 2 value.
 * @param arg       - The adis extended device.
 * @param usr_scr_2 - The user scratch register 2 value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_usr_scr_2(void *arg, u64 usr_scr_2)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->usr_scr_2, usr_scr_2);
}

/**
 * @brief Read user scratch register 3 value.
 * @param arg       - The adis extended device.
 * @param usr_scr_3 - The user scratch register 3 value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_usr_scr_3(void *arg, u64 *usr_scr_3)
{
	struct adis_extd *adis_extd = arg;
	u32 field_val;
	int ret;

	ret = adis_extd_read_field_u32(
		adis_extd, adis_extd->info->field_map->usr_scr_3, &field_val);
	if (ret)
		return ret;

	*usr_scr_3 = field_val;

	return 0;
}

/**
 * @brief Write user scratch register 3 value.
 * @param arg       - The adis extended device.
 * @param usr_scr_3 - The user scratch register 3 value to write.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_write_usr_scr_3(void *arg, u64 usr_scr_3)
{
	struct adis_extd *adis_extd = arg;

	return adis_extd_write_field_u32(
		adis_extd, adis_extd->info->field_map->usr_scr_3, usr_scr_3);
}

/**
 * @brief Write user scratch register 3 value.
 * @param arg          - The adis extended device.
 * @param samples_lost - The samples lost value.
 * @return 0 in case of success, error code otherwise.
 */
static int adis_extd_read_lost_samples_count(void *arg, u64 *samples_lost)
{
	struct adis_extd *adis_extd = arg;

	*samples_lost = adis_extd->samples_lost;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_snsr_init_failure_fops,
			 adis_extd_read_diag_snsr_init_failure, NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_data_path_overrun_fops,
			 adis_extd_read_diag_data_path_overrun, NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_fls_mem_update_failure_fops,
			 adis_extd_read_diag_fls_mem_update_failure, NULL,
			 "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_spi_comm_err_fops,
			 adis_extd_read_diag_spi_comm_err, NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_standby_mode_fops,
			 adis_extd_read_diag_standby_mode, NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_snsr_failure_fops,
			 adis_extd_read_diag_snsr_failure, NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_mem_failure_fops,
			 adis_extd_read_diag_mem_failure, NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_diag_clk_err_fops,
			 adis_extd_read_diag_clk_err, NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_gyro1_failure_fops,
			 adis_extd_read_diag_gyro1_failure, NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_gyro2_failure_fops,
			 adis_extd_read_diag_gyro2_failure, NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_accl_failure_fops,
			 adis_extd_read_diag_accl_failure, NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_x_axis_gyro_failure_fops,
			 adis_extd_read_diag_x_axis_gyro_failure, NULL,
			 "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_y_axis_gyro_failure_fops,
			 adis_extd_read_diag_y_axis_gyro_failure, NULL,
			 "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_z_axis_gyro_failure_fops,
			 adis_extd_read_diag_z_axis_gyro_failure, NULL,
			 "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_x_axis_accl_failure_fops,
			 adis_extd_read_diag_x_axis_accl_failure, NULL,
			 "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_y_axis_accl_failure_fops,
			 adis_extd_read_diag_y_axis_accl_failure, NULL,
			 "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_z_axis_accl_failure_fops,
			 adis_extd_read_diag_z_axis_accl_failure, NULL,
			 "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_diag_aduc_mcu_fault_fops,
			 adis_extd_read_diag_aduc_mcu_fault, NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_read_diag_checksum_err_fops,
			 adis_extd_read_diag_checksum_err, NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_read_diag_fls_mem_wr_cnt_exceed_fops,
			 adis_extd_read_diag_fls_mem_wr_cnt_exceed, NULL,
			 "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_time_stamp_fops, adis_extd_read_time_stamp,
			 NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_data_cntr_fops, adis_extd_read_data_cntr,
			 NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_fifo_cnt_fops, adis_extd_read_fifo_cnt, NULL,
			 "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_spi_chksum_fops, adis_extd_read_spi_chksum,
			 NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_fifo_en_fops, adis_extd_read_fifo_en,
			 adis_extd_write_fifo_en, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_fifo_overflow_fops,
			 adis_extd_read_fifo_overflow,
			 adis_extd_write_fifo_overflow, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_fifo_wm_int_en_fops,
			 adis_extd_read_fifo_wm_int_en,
			 adis_extd_write_fifo_wm_int_en, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_fifo_wm_int_pol_fops,
			 adis_extd_read_fifo_wm_int_pol,
			 adis_extd_write_fifo_wm_int_pol, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_fifo_wm_lvl_fops, adis_extd_read_fifo_wm_lvl,
			 adis_extd_write_fifo_wm_lvl, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_filt_size_var_b_fops,
			 adis_extd_read_filt_size_var_b,
			 adis_extd_write_filt_size_var_b, "%llu\n");

static const struct file_operations adis_extd_gyro_meas_range_fops = {
	.open = simple_open,
	.read = adis_extd_read_gyro_meas_range,
	.llseek = default_llseek,
	.owner = THIS_MODULE,
};

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_dr_polarity_fops, adis_extd_read_dr_polarity,
			 adis_extd_write_dr_polarity, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_sync_polarity_fops,
			 adis_extd_read_sync_polarity,
			 adis_extd_write_sync_polarity, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_sync_mode_fops, adis_extd_read_sync_mode,
			 adis_extd_write_sync_mode, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_sens_bw_fops, adis_extd_read_sens_bw,
			 adis_extd_write_sens_bw, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_pt_of_perc_algnmt_fops,
			 adis_extd_read_pt_of_perc_algnmt,
			 adis_extd_write_pt_of_perc_algnmt, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_linear_accl_comp_fops,
			 adis_extd_read_linear_accl_comp,
			 adis_extd_write_linear_accl_comp, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_burst_sel_fops, adis_extd_read_burst_sel,
			 adis_extd_write_burst_sel, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_burst32_fops, adis_extd_read_burst32,
			 adis_extd_write_burst32, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_timestamp32_fops, adis_extd_read_timestamp32,
			 adis_extd_write_timestamp32, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_sync_4khz_fops, adis_extd_read_sync_4khz,
			 adis_extd_write_sync_4khz, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_up_scale_fops, adis_extd_read_up_scale,
			 adis_extd_write_up_scale, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_dec_rate_fops, adis_extd_read_dec_rate,
			 adis_extd_write_dec_rate, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_bias_corr_tbc_fops,
			 adis_extd_read_bias_corr_tbc,
			 adis_extd_write_bias_corr_tbc, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_bias_corr_en_xg_fops,
			 adis_extd_read_bias_corr_en_xg,
			 adis_extd_write_bias_corr_en_xg, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_bias_corr_en_yg_fops,
			 adis_extd_read_bias_corr_en_yg,
			 adis_extd_write_bias_corr_en_yg, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_bias_corr_en_zg_fops,
			 adis_extd_read_bias_corr_en_zg,
			 adis_extd_write_bias_corr_en_zg, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_bias_corr_en_xa_fops,
			 adis_extd_read_bias_corr_en_xa,
			 adis_extd_write_bias_corr_en_xa, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_bias_corr_en_ya_fops,
			 adis_extd_read_bias_corr_en_ya,
			 adis_extd_write_bias_corr_en_ya, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_bias_corr_en_za_fops,
			 adis_extd_read_bias_corr_en_za,
			 adis_extd_write_bias_corr_en_za, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_bias_corr_update_fops, NULL,
			 adis_extd_cmd_bias_corr_update, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_fact_calib_restore_fops, NULL,
			 adis_extd_cmd_fact_calib_restore, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_cmd_snsr_self_test_fops, NULL,
			 adis_extd_cmd_snsr_self_test, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_cmd_fls_mem_update_fops, NULL,
			 adis_extd_cmd_fls_mem_update, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_cmd_fls_mem_test_fops, NULL,
			 adis_extd_cmd_fls_mem_test, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_cmd_fifo_flush_fops, NULL,
			 adis_extd_cmd_fifo_flush, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_cmd_sw_res_fops, NULL, adis_extd_cmd_sw_res,
			 "%llu\n");

static const struct file_operations adis_extd_fw_rev_fops = {
	.open = simple_open,
	.read = adis_extd_read_firm_rev,
	.llseek = default_llseek,
	.owner = THIS_MODULE,
};

static const struct file_operations adis_extd_fw_date_fops = {
	.open = simple_open,
	.read = adis_extd_read_fw_date,
	.llseek = default_llseek,
	.owner = THIS_MODULE,
};

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_prod_id_fops, adis_extd_read_prod_id, NULL,
			 "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_serial_num_fops, adis_extd_read_serial_num,
			 NULL, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_usr_scr_1_fops, adis_extd_read_usr_scr_1,
			 adis_extd_write_usr_scr_1, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_usr_scr_2_fops, adis_extd_read_usr_scr_2,
			 adis_extd_write_usr_scr_2, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_usr_scr_3_fops, adis_extd_read_usr_scr_3,
			 adis_extd_write_usr_scr_3, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_fls_mem_we_cntr_fops,
			 adis_extd_read_fls_mem_wr_cntr, NULL, "%lld\n");

DEFINE_DEBUGFS_ATTRIBUTE(adis_extd_read_lost_samples_count_fops,
			 adis_extd_read_lost_samples_count, NULL, "%llu\n");

struct adis_extd_debugfs_attr_data {
	const char *name;
	umode_t mode;
	const struct file_operations *fops;
};

static const struct adis_extd_debugfs_attr_data debug_attr_list[] = {
	[ADIS_EXTD_DIAG_SNSR_INIT_FAILURE] = { "diag_sensor_initialization_failure",
					       0664,
					       &adis_extd_diag_snsr_init_failure_fops },
	[ADIS_EXTD_DIAG_DATA_PATH_OVERRUN] = { "diag_data_path_overrun", 0664,
					       &adis_extd_diag_data_path_overrun_fops },
	[ADIS_EXTD_DIAG_FLS_MEM_UPDATE_FAILURE] = { "diag_flash_memory_update_error",
						    0664,
						    &adis_extd_diag_fls_mem_update_failure_fops },
	[ADIS_EXTD_DIAG_SPI_COMM_ERR] = { "diag_spi_communication_error", 0664,
					  &adis_extd_diag_spi_comm_err_fops },
	[ADIS_EXTD_DIAG_STANDBY_MODE] = { "diag_standby_mode", 0664,
					  &adis_extd_diag_standby_mode_fops },
	[ADIS_EXTD_DIAG_SNSR_FAILURE] = { "diag_sensor_self_test_error", 0664,
					  &adis_extd_diag_snsr_failure_fops },
	[ADIS_EXTD_DIAG_MEM_FAILURE] = { "diag_flash_memory_test_error", 0664,
					 &adis_extd_diag_mem_failure_fops },
	[ADIS_EXTD_DIAG_CLK_ERR] = { "diag_clk_error", 0664,
				     &adis_extd_diag_diag_clk_err_fops },
	[ADIS_EXTD_DIAG_GYRO1_FAILURE] = { "diag_gyroscope1_self_test_error",
					   0664,
					   &adis_extd_diag_gyro1_failure_fops },
	[ADIS_EXTD_DIAG_GYRO2_FAILURE] = { "diag_gyroscope2_self_test_error",
					   0664,
					   &adis_extd_diag_gyro2_failure_fops },
	[ADIS_EXTD_DIAG_ACCL_FAILURE] = { "diag_acceleration_self_test_error",
					  0664,
					  &adis_extd_diag_accl_failure_fops },
	[ADIS_EXTD_DIAG_X_AXIS_GYRO_FAILURE] = { "diag_x_axis_gyroscope_failure",
						 0664,
						 &adis_extd_diag_x_axis_accl_failure_fops },
	[ADIS_EXTD_DIAG_Y_AXIS_GYRO_FAILURE] = { "diag_y_axis_gyroscope_failure",
						 0664,
						 &adis_extd_diag_y_axis_accl_failure_fops },
	[ADIS_EXTD_DIAG_Z_AXIS_GYRO_FAILURE] = { "diag_z_axis_gyroscope_failure",
						 0664,
						 &adis_extd_diag_z_axis_accl_failure_fops },
	[ADIS_EXTD_DIAG_X_AXIS_ACCL_FAILURE] = { "diag_x_axis_accelerometer_failure",
						 0664,
						 &adis_extd_diag_x_axis_gyro_failure_fops },
	[ADIS_EXTD_DIAG_Y_AXIS_ACCL_FAILURE] = { "diag_y_axis_accelerometer_failure",
						 0664,
						 &adis_extd_diag_y_axis_gyro_failure_fops },
	[ADIS_EXTD_DIAG_Z_AXIS_ACCL_FAILURE] = { "diag_z_axis_accelerometer_failure",
						 0664,
						 &adis_extd_diag_z_axis_gyro_failure_fops },
	[ADIS_EXTD_DIAG_ADUC_MCU_FAULT] = { "diag_aduc_mcu_fault", 0664,
					    &adis_extd_diag_aduc_mcu_fault_fops },
	[ADIS_EXTD_DIAG_CHECKSUM_ERR] = { "diag_checksum_error_flag", 0664,
					  &adis_extd_read_diag_checksum_err_fops },
	[ADIS_EXTD_DIAG_FLS_MEM_WR_CNT_EXCEED] = { "diag_flash_memory_write_count_exceeded_error",
						   0664,
						   &adis_extd_read_diag_fls_mem_wr_cnt_exceed_fops },
	[ADIS_EXTD_DIAG_LOST_SAMPLES_COUNT] = { "lost_samples_count", 0664,
						&adis_extd_read_lost_samples_count_fops },
	[ADIS_EXTD_TIME_STAMP] = { "time_stamp", 0664,
				   &adis_extd_time_stamp_fops },
	[ADIS_EXTD_DATA_CNTR] = { "data_counter", 0664,
				  &adis_extd_data_cntr_fops },
	[ADIS_EXTD_FIFO_CNT] = { "fifo_sample_count", 0664,
				 &adis_extd_fifo_cnt_fops },
	[ADIS_EXTD_SPI_CHKSUM] = { "spi_checksum", 0664,
				   &adis_extd_spi_chksum_fops },
	[ADIS_EXTD_FIFO_EN] = { "fifo_enable", 0664, &adis_extd_fifo_en_fops },
	[ADIS_EXTD_FIFO_OVERFLOW] = { "fifo_overflow_behavior", 0664,
				      &adis_extd_fifo_overflow_fops },
	[ADIS_EXTD_FIFO_WM_INT_EN] = { "fifo_watermark_interrupt_enable", 0664,
				       &adis_extd_fifo_wm_int_en_fops },
	[ADIS_EXTD_FIFO_WM_INT_POL] = { "fifo_watermark_interrupt_polarity",
					0664, &adis_extd_fifo_wm_int_pol_fops },
	[ADIS_EXTD_FIFO_WM_LVL] = { "fifo_watermark_threshold_level", 0664,
				    &adis_extd_fifo_wm_lvl_fops },
	[ADIS_EXTD_FILT_SIZE_VAR_B] = { "filter_size", 0664,
					&adis_extd_filt_size_var_b_fops },
	[ADIS_EXTD_GYRO_MEAS_RANGE] = { "gyroscope_measurement_range", 0664,
					&adis_extd_gyro_meas_range_fops },
	[ADIS_EXTD_DR_POLARITY] = { "data_ready_polarity", 0664,
				    &adis_extd_dr_polarity_fops },
	[ADIS_EXTD_SYNC_POLARITY] = { "sync_polarity", 0664,
				      &adis_extd_sync_polarity_fops },
	[ADIS_EXTD_SYNC_MODE] = { "sync_mode_select", 0664,
				  &adis_extd_sync_mode_fops },
	[ADIS_EXTD_SENS_BW] = { "internal_sensor_bandwidth", 0664,
				&adis_extd_sens_bw_fops },
	[ADIS_EXTD_PT_OF_PERC_ALGNMT] = { "point_of_percussion_alignment", 0664,
					  &adis_extd_pt_of_perc_algnmt_fops },
	[ADIS_EXTD_LINEAR_ACCL_COMP] = { "linear_acceleration_compensation",
					 0664,
					 &adis_extd_linear_accl_comp_fops },
	[ADIS_EXTD_BURST_SEL] = { "burst_data_selection", 0664,
				  &adis_extd_burst_sel_fops },
	[ADIS_EXTD_BURST32] = { "burst_size_selection", 0664,
				&adis_extd_burst32_fops },
	[ADIS_EXTD_TIMESTAMP32] = { "timestamp32", 0664,
				    &adis_extd_timestamp32_fops },
	[ADIS_EXTD_SYNC_4KHZ] = { "internal_sync_enable_4khz", 0664,
				  &adis_extd_sync_4khz_fops },
	[ADIS_EXTD_UP_SCALE] = { "sync_signal_scale", 0664,
				 &adis_extd_up_scale_fops },
	[ADIS_EXTD_DEC_RATE] = { "decimation_filter", 0664,
				 &adis_extd_dec_rate_fops },
	[ADIS_EXTD_BIAS_CORR_TBC] = { "bias_correction_time_base_control", 0664,
				      &adis_extd_bias_corr_tbc_fops },
	[ADIS_EXTD_BIAS_CORR_EN_XG] = { "x_axis_gyroscope_bias_correction_enable",
					0664, &adis_extd_bias_corr_en_xg_fops },
	[ADIS_EXTD_BIAS_CORR_EN_YG] = { "y_axis_gyroscope_bias_correction_enable",
					0664, &adis_extd_bias_corr_en_yg_fops },
	[ADIS_EXTD_BIAS_CORR_EN_ZG] = { "z_axis_gyroscope_bias_correction_enable",
					0664, &adis_extd_bias_corr_en_zg_fops },
	[ADIS_EXTD_BIAS_CORR_EN_XA] = { "x_axis_accelerometer_bias_correction_enable",
					0664, &adis_extd_bias_corr_en_xa_fops },
	[ADIS_EXTD_BIAS_CORR_EN_YA] = { "y_axis_accelerometer_bias_correction_enable",
					0664, &adis_extd_bias_corr_en_ya_fops },
	[ADIS_EXTD_BIAS_CORR_EN_ZA] = { "z_axis_accelerometer_bias_correction_enable",
					0664, &adis_extd_bias_corr_en_za_fops },
	[ADIS_EXTD_CMD_BIAS_CORR_UPDATE] = { "bias_correction_update", 0664,
					     &adis_extd_bias_corr_update_fops },
	[ADIS_EXTD_CMD_FACT_CALIB_RESTORE] = { "factory_calibration_restore",
					       0664,
					       &adis_extd_fact_calib_restore_fops },
	[ADIS_EXTD_CMD_SNSR_SELF_TEST] = { "sensor_self_test", 0664,
					   &adis_extd_cmd_snsr_self_test_fops },
	[ADIS_EXTD_CMD_FLS_MEM_UPDATE] = { "flash_memory_update", 0664,
					   &adis_extd_cmd_fls_mem_update_fops },
	[ADIS_EXTD_CMD_FLS_MEM_TEST] = { "flash_memory_test", 0664,
					 &adis_extd_cmd_fls_mem_test_fops },
	[ADIS_EXTD_CMD_FIFO_FLUSH] = { "fifo_flush", 0664,
				       &adis_extd_cmd_fifo_flush_fops },
	[ADIS_EXTD_CMD_SW_RES] = { "software_reset", 0664,
				   &adis_extd_cmd_sw_res_fops },
	[ADIS_EXTD_FIRM_REV] = { "firmware_revision", 0664,
				 &adis_extd_fw_rev_fops },
	[ADIS_EXTD_FIRM_DATE] = { "firmware_date", 0664,
				  &adis_extd_fw_date_fops },
	[ADIS_EXTD_PROD_ID] = { "product_id", 0664, &adis_extd_prod_id_fops },
	[ADIS_EXTD_SERIAL_NUM] = { "serial_number", 0664,
				   &adis_extd_serial_num_fops },
	[ADIS_EXTD_USR_SCR_1] = { "scratch_pad_register1", 0664,
				  &adis_extd_usr_scr_1_fops },
	[ADIS_EXTD_USR_SCR_2] = { "scratch_pad_register2", 0664,
				  &adis_extd_usr_scr_2_fops },
	[ADIS_EXTD_USR_SCR_3] = { "scratch_pad_register3", 0664,
				  &adis_extd_usr_scr_3_fops },
	[ADIS_EXTD_FLS_MEM_WR_CNTR] = { "flash_counter", 0664,
					&adis_extd_fls_mem_we_cntr_fops },
};

/**
 * @brief Initialize debug attributes for adis_extd device.
 * @param indio_dev - The iio device.
 * @param attr_arr  - The debug attributes array.
 * @param arr_size  - The size of attr_arr.
 * @return 0 in case of success, error code otherwise.
 */
void adis_extd_debugfs_init(struct iio_dev *indio_dev,
			    const enum adis_extd_debug_attr *attr_arr,
			    u16 arr_size)
{
	struct adis_extd *adis_extd = iio_priv(indio_dev);
	struct dentry *d = iio_get_debugfs_dentry(indio_dev);
	u16 i = 0;

	for (i = 0; i < arr_size; i++)
		debugfs_create_file_unsafe(debug_attr_list[attr_arr[i]].name,
					   debug_attr_list[attr_arr[i]].mode, d,
					   adis_extd,
					   debug_attr_list[attr_arr[i]].fops);
}
