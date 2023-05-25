/* SPDX-License-Identifier: GPL-2.0
 *
 * Common extended library for ADIS devices
 *
 * Copyright 2023 Analog Devices Inc.
 */

#ifndef __IIO_ADIS_EXTD_H__
#define __IIO_ADIS_EXTD_H__

#define ADIS_EXTD_BURST_MAX_DATA 20 /* in bytes */
#define ADIS_EXTD_MAX_SCAN_DATA 32 /* in bytes */

/** @struct adis_extd_sync_modes
 *  @brief ADIS_EXTD sync modes
 */
enum adis_extd_sync_modes {
	ADIS_EXTD_SYNC_DEFAULT = 0,
	ADIS_EXTD_SYNC_DIRECT,
	ADIS_EXTD_SYNC_SCALED,
	ADIS_EXTD_SYNC_OUTPUT
};

/** @struct adis_extd_chan_type
 *  @brief ADIS_EXTD channels enumeration
 */
enum adis_extd_chan_type {
	ADIS_EXTD_GYRO_X,
	ADIS_EXTD_GYRO_Y,
	ADIS_EXTD_GYRO_Z,
	ADIS_EXTD_ACCEL_X,
	ADIS_EXTD_ACCEL_Y,
	ADIS_EXTD_ACCEL_Z,
	ADIS_EXTD_TEMP,
	ADIS_EXTD_DELTA_ANGL_X,
	ADIS_EXTD_DELTA_ANGL_Y,
	ADIS_EXTD_DELTA_ANGL_Z,
	ADIS_EXTD_DELTA_VEL_X,
	ADIS_EXTD_DELTA_VEL_Y,
	ADIS_EXTD_DELTA_VEL_Z,
};

/** @struct adis_extd_debug_attr
 *  @brief ADIS_EXTD supported debug attributes
 */
enum adis_extd_debug_attr {
	ADIS_EXTD_DIAG_SNSR_INIT_FAILURE,
	ADIS_EXTD_DIAG_DATA_PATH_OVERRUN,
	ADIS_EXTD_DIAG_FLS_MEM_UPDATE_FAILURE,
	ADIS_EXTD_DIAG_SPI_COMM_ERR,
	ADIS_EXTD_DIAG_STANDBY_MODE,
	ADIS_EXTD_DIAG_SNSR_FAILURE,
	ADIS_EXTD_DIAG_MEM_FAILURE,
	ADIS_EXTD_DIAG_CLK_ERR,
	ADIS_EXTD_DIAG_GYRO1_FAILURE,
	ADIS_EXTD_DIAG_GYRO2_FAILURE,
	ADIS_EXTD_DIAG_ACCL_FAILURE,
	ADIS_EXTD_DIAG_X_AXIS_GYRO_FAILURE,
	ADIS_EXTD_DIAG_Y_AXIS_GYRO_FAILURE,
	ADIS_EXTD_DIAG_Z_AXIS_GYRO_FAILURE,
	ADIS_EXTD_DIAG_X_AXIS_ACCL_FAILURE,
	ADIS_EXTD_DIAG_Y_AXIS_ACCL_FAILURE,
	ADIS_EXTD_DIAG_Z_AXIS_ACCL_FAILURE,
	ADIS_EXTD_DIAG_ADUC_MCU_FAULT,
	ADIS_EXTD_DIAG_CHECKSUM_ERR,
	ADIS_EXTD_DIAG_FLS_MEM_WR_CNT_EXCEED,
	ADIS_EXTD_DIAG_LOST_SAMPLES_COUNT,

	ADIS_EXTD_TIME_STAMP,
	ADIS_EXTD_DATA_CNTR,

	ADIS_EXTD_FIFO_CNT,
	ADIS_EXTD_SPI_CHKSUM,

	ADIS_EXTD_FIFO_EN,
	ADIS_EXTD_FIFO_OVERFLOW,
	ADIS_EXTD_FIFO_WM_INT_EN,
	ADIS_EXTD_FIFO_WM_INT_POL,
	ADIS_EXTD_FIFO_WM_LVL,

	ADIS_EXTD_FILT_SIZE_VAR_B,
	ADIS_EXTD_GYRO_MEAS_RANGE,

	ADIS_EXTD_DR_POLARITY,
	ADIS_EXTD_SYNC_POLARITY,
	ADIS_EXTD_SYNC_MODE,
	ADIS_EXTD_SENS_BW,
	ADIS_EXTD_PT_OF_PERC_ALGNMT,
	ADIS_EXTD_LINEAR_ACCL_COMP,
	ADIS_EXTD_BURST_SEL,
	ADIS_EXTD_BURST32,
	ADIS_EXTD_TIMESTAMP32,
	ADIS_EXTD_SYNC_4KHZ,

	ADIS_EXTD_UP_SCALE,
	ADIS_EXTD_DEC_RATE,
	ADIS_EXTD_BIAS_CORR_TBC,
	ADIS_EXTD_BIAS_CORR_EN_XG,
	ADIS_EXTD_BIAS_CORR_EN_YG,
	ADIS_EXTD_BIAS_CORR_EN_ZG,
	ADIS_EXTD_BIAS_CORR_EN_XA,
	ADIS_EXTD_BIAS_CORR_EN_YA,
	ADIS_EXTD_BIAS_CORR_EN_ZA,

	ADIS_EXTD_CMD_BIAS_CORR_UPDATE,
	ADIS_EXTD_CMD_FACT_CALIB_RESTORE,
	ADIS_EXTD_CMD_SNSR_SELF_TEST,
	ADIS_EXTD_CMD_FLS_MEM_UPDATE,
	ADIS_EXTD_CMD_FLS_MEM_TEST,
	ADIS_EXTD_CMD_FIFO_FLUSH,
	ADIS_EXTD_CMD_SW_RES,

	ADIS_EXTD_FIRM_REV,
	ADIS_EXTD_FIRM_DATE,
	ADIS_EXTD_PROD_ID,
	ADIS_EXTD_SERIAL_NUM,
	ADIS_EXTD_USR_SCR_1,
	ADIS_EXTD_USR_SCR_2,
	ADIS_EXTD_USR_SCR_3,
	ADIS_EXTD_FLS_MEM_WR_CNTR,
};

/** @struct adis_extd_diag_flags
 *  @brief Bitfield struct which maps on the diagnosis register
 */
struct adis_extd_diag_flags {
	/** Sensor initialization failure. */
	u8 snsr_init_failure : 1;
	/** Data path overrun. */
	u8 data_path_overrun : 1;
	/** Flash memory update failure. */
	u8 fls_mem_update_failure : 1;
	/** SPI communication error. */
	u8 spi_comm_err : 1;
	/** Standby mode. */
	u8 standby_mode : 1;
	/** Sensor failure. */
	u8 snsr_failure : 1;
	/** Memory failure. */
	u8 mem_failure : 1;
	/** Clock error. */
	u8 clk_err : 1;
	/** Gyroscope 1 failure. */
	u8 gyro1_failure : 1;
	/** Gyroscope 2 failure. */
	u8 gyro2_failure : 1;
	/** Accelerometer failure. */
	u8 accl_failure : 1;
	/** X-Axis gyroscope failure. */
	u8 x_axis_gyro_failure : 1;
	/** Y-Axis gyroscope failure. */
	u8 y_axis_gyro_failure : 1;
	/** Z-Axis gyroscope failure. */
	u8 z_axis_gyro_failure : 1;
	/** X-Axis accelerometer failure. */
	u8 x_axis_accl_failure : 1;
	/** Y-Axis accelerometer failure. */
	u8 y_axis_accl_failure : 1;
	/** Z-Axis accelerometer failure. */
	u8 z_axis_accl_failure : 1;
	/** ADuC microcontroller fault. */
	u8 aduc_mcu_fault : 1;
	/** Checksum error.  */
	u8 checksum_err : 1;
	/** Flash memory write count exceeded. */
	u8 fls_mem_wr_cnt_exceed : 1;
};

/** @struct adis_extd_timeout
 *  @brief ADIS_EXTD chip timeouts
 */
struct adis_extd_timeout {
	/** Wait time in milliseconds needed after a hardware reset. */
	u16 reset_ms;
	/** Wait time in milliseconds needed after a factory calibration restore
	 *  command is issued.
	 */
	u16 fact_calib_restore_ms;
	/** Wait time in milliseconds needed after a self test
	 * command is issued.
	 */
	u16 self_test_ms;
	/** Wait time in milliseconds needed after a flash memory update command
	 *  is issued.
	 */
	u16 fls_mem_update_ms;
	/** Wait time in milliseconds needed after a flash memory test command
	 *  is issued.
	 */
	u16 fls_mem_test_ms;
	/** Wait time in milliseconds needed after a software reset command is
	 * issued.
	 */
	u16 sw_reset_ms;
	/** Wait time in microseconds needed after a write command is issued for
	 *  the decimation rate field.
	 */
	u16 dec_rate_update_us;
	/** Wait time in microseconds needed after a write command is issued for
	 *  the filter size variable b field.
	 */
	u16 filt_size_var_b_update_us;
	/** Wait time in microseconds needed after a write command
	 *  is issued in the miscellaneous control register.
	 */
	u16 msc_reg_update_us;
	/** Wait time in milliseconds needed after a write command is issued to
	 *  change the internal sensor bandwidth field in the miscellaneous
	 *  control register.
	 */
	u16 sens_bw_update_ms;
};

/** @struct adis_extd_field
 *  @brief ADIS_EXTD device field structure
 */
struct adis_extd_field {
	/** Address of the register in which the field is found. */
	u8 reg_addr;
	/** Size of te register in which the field is found. */
	u8 reg_size;
	/** The mask of the field in a register. */
	u32 field_mask;
};

/** @struct adis_extd_data_field_map_def
 *  @brief ADIS_EXTD filed map definition structure
 */
struct adis_extd_data_field_map_def {
	/** Status/Error Flag Indicators register. */
	struct adis_extd_field diag_stat;
	/** Sensor initialization failure bit mask. */
	u16 diag_snsr_init_failure_mask;
	/** Data path overrun. */
	u16 diag_data_path_overrun_mask;
	/** Flash memory update failure. */
	u16 diag_fls_mem_update_failure_mask;
	/** SPI communication error.  */
	u16 diag_spi_comm_err_mask;
	/** Standby mode.  */
	u16 diag_standby_mode_mask;
	/** Sensor failure.  */
	u16 diag_snsr_failure_mask;
	/** Memory failure.  */
	u16 diag_mem_failure_mask;
	/** Clock error.  */
	u16 diag_clk_err_mask;
	/** Gyroscope 1 failure.  */
	u16 diag_gyro1_failure_mask;
	/** Gyroscope 2 failure.  */
	u16 diag_gyro2_failure_mask;
	/** Accelerometer failure.  */
	u16 diag_accl_failure_mask;
	/** X-Axis gyroscope failure bit mask. */
	u16 diag_x_axis_gyro_failure_mask;
	/** Y-Axis gyroscope failure bit mask. */
	u16 diag_y_axis_gyro_failure_mask;
	/** Z-Axis gyroscope failure bit mask. */
	u16 diag_z_axis_gyro_failure_mask;
	/** X-Axis accelerometer failure bit mask. */
	u16 diag_x_axis_accl_failure_mask;
	/** Y-Axis accelerometer failure bit mask. */
	u16 diag_y_axis_accl_failure_mask;
	/** Z-Axis accelerometer failure bit mask. */
	u16 diag_z_axis_accl_failure_mask;
	/** ADuC microcontroller fault bit mask. */
	u16 diag_aduc_mcu_fault_mask;

	/** X-axis gyroscope raw data. */
	struct adis_extd_field x_gyro;
	/** Y-axis gyroscope raw data. */
	struct adis_extd_field y_gyro;
	/** Z-axis gyroscope raw data. */
	struct adis_extd_field z_gyro;
	/** X-axis accelerometer raw data. */
	struct adis_extd_field x_accl;
	/** Y-axis accelerometer raw data. */
	struct adis_extd_field y_accl;
	/** Z-axis accelerometer raw data. */
	struct adis_extd_field z_accl;
	/** Temperature raw data. */
	struct adis_extd_field temp_out;

	/** Time stamp raw data. */
	struct adis_extd_field time_stamp;
	/** Data update counter. */
	struct adis_extd_field data_cntr;

	/** X-axis delta angle raw data. */
	struct adis_extd_field x_deltang;
	/** Y-axis delta angle raw data. */
	struct adis_extd_field y_deltang;
	/** Z-axis delta angle raw data. */
	struct adis_extd_field z_deltang;
	/** X-axis delta velocity raw data. */
	struct adis_extd_field x_deltvel;
	/** Y-axis delta velocity raw data. */
	struct adis_extd_field y_deltvel;
	/** Z-axis delta velocity raw data. */
	struct adis_extd_field z_deltvel;

	/** FIFO sample count.  */
	struct adis_extd_field fifo_cnt;
	/** SPI dynamic checksum.  */
	struct adis_extd_field spi_chksum;

	/** X-axis gyroscope offset correction. */
	struct adis_extd_field xg_bias;
	/** Y-axis gyroscope offset correction. */
	struct adis_extd_field yg_bias;
	/** Z-axis gyroscope offset correction. */
	struct adis_extd_field zg_bias;
	/** X-axis accelerometer offset correction. */
	struct adis_extd_field xa_bias;
	/** Y-axis accelerometer offset correction. */
	struct adis_extd_field ya_bias;
	/** Z-axis accelerometer offset correction. */
	struct adis_extd_field za_bias;

	/** FIFO mode enable. */
	struct adis_extd_field fifo_en;
	/** FIFO overflow behavior. */
	struct adis_extd_field fifo_overflow;
	/** FIFO watermark interrupt enable. */
	struct adis_extd_field fifo_wm_int_en;
	/** FIFO watermark interrupt polarity. */
	struct adis_extd_field fifo_wm_int_pol;
	/** FIFO watermark threshold level. */
	struct adis_extd_field fifo_wm_lvl;

	/** Filter size variable B. */
	struct adis_extd_field filt_size_var_b;

	/** Gyroscope measurement range. */
	struct adis_extd_field gyro_meas_range;

	struct adis_extd_field msc_ctrl;
	/** Data ready polarity. */
	struct adis_extd_field dr_polarity;
	/** SYNC signal polarity. */
	struct adis_extd_field sync_polarity;
	/** SYNC mode select. */
	struct adis_extd_field sync_mode;
	/** Internal sensor bandwidth. */
	struct adis_extd_field sens_bw;
	/** Point of percussion alignment enable bit. */
	struct adis_extd_field pt_of_perc_algnmt;
	/** Linear acceleration compensation enable bit. */
	struct adis_extd_field linear_accl_comp;
	/** Burst read output array selection. */
	struct adis_extd_field burst_sel;
	/** 32-bit burst enable bit. */
	struct adis_extd_field burst32;
	/** 32-bit timestamp enable bit. */
	struct adis_extd_field timestamp32;
	/** 4khz internal sync enable bit. */
	struct adis_extd_field sync_4khz;

	/** External clock scale factor. */
	struct adis_extd_field up_scale;
	/** Decimation rate. */
	struct adis_extd_field dec_rate;

	/** Bias correction time base control. */
	struct adis_extd_field bias_corr_tbc;
	/** X-axis gyroscope bias correction enable. */
	struct adis_extd_field bias_corr_en_xg;
	/** Y-axis gyroscope bias correction enable. */
	struct adis_extd_field bias_corr_en_yg;
	/** Z-axis gyroscope bias correction enable. */
	struct adis_extd_field bias_corr_en_zg;
	/** X-axis accelerometer bias correction enable. */
	struct adis_extd_field bias_corr_en_xa;
	/** Y-axis accelerometer bias correction enable. */
	struct adis_extd_field bias_corr_en_ya;
	/** Z-axis accelerometer bias correction enable. */
	struct adis_extd_field bias_corr_en_za;

	struct adis_extd_field glob_cmd;
	/** Bias correction update command bit. */
	struct adis_extd_field bias_corr_update;
	/** Factory calibration restore command bit. */
	struct adis_extd_field fact_calib_restore;
	/** Sensor self test command bit. */
	struct adis_extd_field snsr_self_test;
	/** Flash memory update command bit. */
	struct adis_extd_field fls_mem_update;
	/** Flash memory test command bit. */
	struct adis_extd_field fls_mem_test;
	/** FIFO flush command bit. */
	struct adis_extd_field fifo_flush;
	/** Software reset command bit. */
	struct adis_extd_field sw_res;

	/** Firmware revision. */
	struct adis_extd_field firm_rev;
	/** Factory configuration day. */
	struct adis_extd_field firm_d;
	/** Factory configuration month. */
	struct adis_extd_field firm_m;
	/** Factory configuration year. */
	struct adis_extd_field firm_y;
	/** Product identification. */
	struct adis_extd_field prod_id;
	/** Serial number. */
	struct adis_extd_field serial_num;

	/** User Scratch Pad Register 1. */
	struct adis_extd_field usr_scr_1;
	/** User Scratch Pad Register 2. */
	struct adis_extd_field usr_scr_2;
	/** User Scratch Pad Register 3. */
	struct adis_extd_field usr_scr_3;

	/** Flash memory endurance counter  */
	struct adis_extd_field fls_mem_wr_cntr;
};

/** @struct adis_extd_clk_freq_limit
 *  @brief ADIS_EXTD frequency limit for input synchronization clock
 */
struct adis_extd_clk_freq_limit {
	/** Minimum allowed frequency for adis clocks in Hertz. */
	u32 min_freq;
	/** Maximum allowed frequency for adis clocks in Hertz. */
	u32 max_freq;
};

/** @struct adis_extd_scale_fractional
 *  @brief ADIS_EXTD fractional scale format structure; scale = dividend/divisor
 */
struct adis_extd_scale_fractional {
	/** Scale dividend. */
	u32 dividend;
	/** Scale divisor. */
	u32 divisor;
};

/** @struct adis_extd_scale_fractional_log2
 *  @brief ADIS_EXTD IIO fractional log2 scale format structure; scale = dividend/2^power
 */
struct adis_extd_scale_fractional_log2 {
	/** Scale dividend. */
	u32 dividend;
	/** Scale 2's power. */
	u32 power;
};

/** @struct adis_extd_chip_info
 *  @brief ADIS_EXTD specific chip information structure
 */
struct adis_extd_chip_info {
	/** Chip specific field map configuration. */
	const struct adis_extd_data_field_map_def *field_map;
	/** Chip specific synchronization clock frequency limits. */
	const struct adis_extd_clk_freq_limit *sync_clk_freq_limits;
	/** Chip specific sampling clock frequency limits. */
	const struct adis_extd_clk_freq_limit sampling_clk_limits;
	/** Chip specific timeouts. */
	const struct adis_extd_timeout *timeouts;
	/** Chip specific iio channels. */
	const struct iio_chan_spec *channels;
	/** Chip number of channels  */
	u32 num_channels;
	/** Chip name.  */
	const char *name;
	/** Chip specific data needed for generic adis driver.  */
	const struct adis_data adis_data;
	/** Gyroscope fractional scale. */
	struct adis_extd_scale_fractional gyro_scale;
	/** Accelerometer fractional scale. */
	struct adis_extd_scale_fractional accl_scale;
	/** Rotation angle fractional log2 scale. */
	struct adis_extd_scale_fractional_log2 rot_scale;
	/** Linear velocity fractional log2 scale. */
	struct adis_extd_scale_fractional_log2 vel_scale;
	/** Temperature fractional scale. */
	struct adis_extd_scale_fractional temp_scale;
	/** Chip specific flash memory write counter maximum allowed value. */
	u32 fls_mem_wr_cntr_max;
	/** Chip specific internal clock frequency in Hertz. */
	u32 int_clk;
	/** Chip specific filter size variable B field maximum allowed value. */
	u16 filt_size_var_b_max;
	/** Chip specific decimation rate field maximum allowed	value. */
	u16 dec_rate_max;
	/** Chip specific sync mode select field maximum allowed value. */
	u8 sync_mode_max;
	/** Chip specific bias correction time base control maximum allowed
	 *  encoded value.
	 */
	u8 bias_corr_tbc_max;
	/** Extra needed bytes number for burst readings when burst32 is enabled */
	u8 burst32_extra_bytes;
	/** True if iio device offers FIFO support for buffer reading. */
	bool has_fifo;
};

/** @struct adis_extd
 *  @brief ADIS_EXTD specific device structure
 */
struct adis_extd {
	/** Chip specific information. */
	const struct adis_extd_chip_info *info;
	/** Generic adis device structure. */
	struct adis adis;
	/** Extra SPI message used for reading fifo data without popping  */
	struct spi_message msg;
	/** Extra SPI transfer used for reading fifo data without popping  */
	struct spi_transfer	*xfer;
	/** Extra SPI buffer used for reading fifo data without popping  */
	void			*buffer;
	/** Extra SPI tx buffer used for reading fifo data without popping  */
	u8			tx[10] ____cacheline_aligned;
	/** Extra SPI rx buffer used for reading fifo data without popping  */
	u8			rx[4];
	/** Data buffer to store one sample-set. */
	__be16 data[ADIS_EXTD_MAX_SCAN_DATA];
	/** Current diag flags values. */
	struct adis_extd_diag_flags diag_flags;
	/** Current configured sync mode. */
	u16 sync_mode;
	/** Current configured burst32 value. */
	u8 burst32;
	/** Current setting for adis burst data selection. */
	u8 burst_sel;
	/** External clock frequency in Hertz. */
	u32 ext_clk;
	/** Internal clock frequency in Hertz. */
	u32 int_clk;
	/** Current data counter for the current buffer reading.*/
	u16 data_cntr;
	/** Number of lost samples for the current buffer reading. */
	u32 samples_lost;
	/** Number of flash writes. */
	u32 flash_count;
	/** Current device sampling frequency. */
	u32 sampling_frequency;
	/** Set to true if device fifo is enabled. */
	bool fifo_enabled;
	/** Set to true if buffer is enabled. */
	bool buffer_enabled;
	/** Different from 0 is LSB data is needed in burst reads  */
	unsigned long lsb_flag;
};

#define ADIS_EXTD_DATA(_msc_ctrl_addr, _glob_cmd_addr, _diag_stat_addr,        \
		       _prod_id_addr, _self_test_addr, _burst_reg_addr,        \
		       _prod_id, _timeouts, _burst_max_speed,                  \
		       _cs_change_delay, _read_delay, _write_delay)            \
	{                                                                      \
		.msc_ctrl_reg = _msc_ctrl_addr,                                \
		.glob_cmd_reg = _glob_cmd_addr,                                \
		.diag_stat_reg = _diag_stat_addr,                              \
		.prod_id_reg = _prod_id_addr,                                  \
		.prod_id = (_prod_id),                                         \
		.self_test_mask = BIT(2),                                      \
		.self_test_reg = _self_test_addr,                              \
		.cs_change_delay = _cs_change_delay,                           \
		.read_delay = _read_delay,                                     \
		.write_delay = _write_delay,                                   \
		.unmasked_drdy = true,                                         \
		.timeouts = (_timeouts),                                       \
		.burst_reg_cmd = _burst_reg_addr,                              \
		.burst_len = ADIS_EXTD_BURST_MAX_DATA,                         \
		.burst_max_speed_hz = _burst_max_speed                         \
	}

#define ADIS_EXTD_MOD_CHAN(_type, _mod, _address, _si, _r_bits, _s_bits)       \
	{                                                                      \
		.type = (_type), .modified = 1, .channel2 = (_mod),            \
		.info_mask_separate =                                          \
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_CALIBBIAS), \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),          \
		.info_mask_shared_by_all =                                     \
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |                         \
			BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),      \
		.address = (_address), .scan_index = (_si),                    \
		.scan_type = {                                                 \
			.sign = 's',                                           \
			.realbits = (_r_bits),                                 \
			.storagebits = (_s_bits),                              \
			.endianness = IIO_BE,                                  \
		},                                                             \
	}

#define ADIS_EXTD_MOD_CHAN_DELTA(_type, _mod, _address, _si, _r_bits, _s_bits) \
	{                                                                      \
		.type = (_type), .modified = 1, .channel2 = (_mod),            \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                  \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),          \
		.info_mask_shared_by_all =                                     \
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |                         \
			BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),      \
		.address = (_address), .scan_index = (_si),                    \
		.scan_type = {                                                 \
			.sign = 's',                                           \
			.realbits = (_r_bits),                                 \
			.storagebits = (_s_bits),                              \
			.endianness = IIO_BE,                                  \
		},                                                             \
	}

#define ADIS_EXTD_GYRO_CHAN(_addr, _mod, _r_bits, _s_bits)                     \
	ADIS_EXTD_MOD_CHAN(IIO_ANGL_VEL, IIO_MOD_##_mod, _addr,                \
			   ADIS_EXTD_GYRO_##_mod, _r_bits, _s_bits)

#define ADIS_EXTD_ACCEL_CHAN(_addr, _mod, _r_bits, _s_bits)                    \
	ADIS_EXTD_MOD_CHAN(IIO_ACCEL, IIO_MOD_##_mod, _addr,                   \
			   ADIS_EXTD_ACCEL_##_mod, _r_bits, _s_bits)

#define ADIS_EXTD_DELTA_ANGL_CHAN(_addr, _mod, _r_bits, _s_bits)               \
	ADIS_EXTD_MOD_CHAN_DELTA(IIO_ROT, IIO_MOD_##_mod, _addr,               \
				 ADIS_EXTD_DELTA_ANGL_##_mod, _r_bits,         \
				 _s_bits)

#define ADIS_EXTD_DELTA_VEL_CHAN(_addr, _mod, _r_bits, _s_bits)                \
	ADIS_EXTD_MOD_CHAN_DELTA(IIO_VELOCITY, IIO_MOD_##_mod, _addr,          \
				 ADIS_EXTD_DELTA_VEL_##_mod, _r_bits, _s_bits)

#define ADIS_EXTD_TEMP_CHAN(_regdef, _r_bits, _s_bits)                         \
	{                                                                      \
		.type = IIO_TEMP,                                              \
		.info_mask_separate =                                          \
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),     \
		.info_mask_shared_by_all =                                     \
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |                         \
			BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),      \
		.address = _regdef.temp_out.reg_addr,                          \
		.scan_index = ADIS_EXTD_TEMP,                                  \
		.scan_type = {                                                 \
			.sign = 's',                                           \
			.realbits = _r_bits,                                   \
			.storagebits = _s_bits,                                \
			.endianness = IIO_BE,                                  \
		},                                                             \
	}

int adis_extd_init(struct spi_device *spi,
		   const enum adis_extd_debug_attr *attr_arr, u16 arr_size);

/* DebugFS API's*/
#ifdef CONFIG_DEBUG_FS
void adis_extd_debugfs_init(struct iio_dev *indio_dev,
			      const enum adis_extd_debug_attr *attr_arr,
			      u16 arr_size);
#else
static inline void adis_extd_debugfs_init(struct iio_dev *indio_dev,
		const enum adis_extd_debug_attr *attr_arr,
		u16 arr_size) {}
#endif

void adis_extd_update_diag_flags(struct adis_extd *adis_extd,
					u16 diag_stat);

int adis_extd_read_field_u32(struct adis_extd *adis_extd,
				    struct adis_extd_field field,
				    u32 *field_val);

static inline u32 adis_extd_field_get(u32 reg, u32 mask)
{
	return (reg & mask) >> __ffs(mask);
}

static inline u32 adis_extd_field_prep(u32 reg, u32 mask)
{
	return (reg << __ffs(mask)) & mask;
}

int adis_extd_write_field_u32(struct adis_extd *adis_extd,
				     struct adis_extd_field field,
				     u32 field_val);
int adis_extd_read_sampling_freq(struct adis_extd *adis_extd, u32 *freq);

int adis_extd_update_sync_mode(struct adis_extd *adis_extd,
				      unsigned int sync_mode,
				      unsigned int ext_clk);
int adis_extd_write_fifo_en(void *arg, u64 fifo_en);
int adis_extd_write_fifo_overflow(void *arg, u64 fifo_overflow);
int adis_extd_write_fifo_wm_int_en(void *arg, u64 fifo_wm_int_en);
int adis_extd_write_fifo_wm_int_pol(void *arg, u64 fifo_wm_int_pol);
int adis_extd_write_fifo_wm_lvl(void *arg, u64 fifo_wm_lvl);
int adis_extd_read_filt_size_var_b(void *arg, u64 *filt_size_var_b);
int adis_extd_write_filt_size_var_b(void *arg, u64 filt_size_var_b);
int adis_extd_write_dr_polarity(void *arg, u64 dr_polarity);
int adis_extd_read_sync_mode(void *arg, u64 *sync_mode);
int adis_extd_read_up_scale(void *arg, u64 *up_scale);
int adis_extd_write_up_scale(void *arg, u64 up_scale);
int adis_extd_read_dec_rate(void *arg, u64 *dec_rate);
int adis_extd_write_dec_rate(void *arg, u64 dec_rate);
int adis_extd_cmd_fifo_flush(void *arg, u64 val);
int adis_extd_write_sync_4khz(void *arg, u64 sync_4khz);
int adis_extd_write_burst32(void *arg, u64 burst32);
int adis_extd_read_fifo_cnt(void *arg, u64 *fifo_cnt);
int adis_extd_init_values(struct adis_extd *adis_extd);
#endif
