/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2026 InvenSense, Inc.
 */

#ifndef INV_ICM42607_H_
#define INV_ICM42607_H_

#include <linux/bits.h>
#include <linux/iio/iio.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/time.h>
#include <linux/types.h>

#include <asm/byteorder.h>

/*
 * Serial bus slew rates. Rates are expressed as range between the two
 * values with the midpoint as the typical rate. For the final value of
 * 2ns, 2ns is considered the max value with no expressed minimum or
 * typical value.
 */
enum inv_icm42607_slew_rate {
	INV_ICM42607_SLEW_RATE_20_60NS = 0,
	INV_ICM42607_SLEW_RATE_12_36NS = 1,
	INV_ICM42607_SLEW_RATE_6_19NS = 2,
	INV_ICM42607_SLEW_RATE_4_14NS = 3,
	INV_ICM42607_SLEW_RATE_2_6NS = 4,
	INV_ICM42607_SLEW_RATE_2NS = 5,
	INV_ICM42607_SLEW_RATE_NB
};

enum inv_icm42607_sensor_mode {
	INV_ICM42607_SENSOR_MODE_OFF = 0,
	INV_ICM42607_SENSOR_MODE_STANDBY = 1,
	INV_ICM42607_SENSOR_MODE_LOW_POWER = 2,
	INV_ICM42607_SENSOR_MODE_LOW_NOISE = 3,
	INV_ICM42607_SENSOR_MODE_NB
};

/* gyroscope fullscale values */
enum inv_icm42607_gyro_fs {
	INV_ICM42607_GYRO_FS_2000DPS = 0,
	INV_ICM42607_GYRO_FS_1000DPS = 1,
	INV_ICM42607_GYRO_FS_500DPS = 2,
	INV_ICM42607_GYRO_FS_250DPS = 3,
	INV_ICM42607_GYRO_FS_NB
};

/* accelerometer fullscale values */
enum inv_icm42607_accel_fs {
	INV_ICM42607_ACCEL_FS_16G = 0,
	INV_ICM42607_ACCEL_FS_8G = 1,
	INV_ICM42607_ACCEL_FS_4G = 2,
	INV_ICM42607_ACCEL_FS_2G = 3,
	INV_ICM42607_ACCEL_FS_NB
};

/* ODR values  - Note Gyro does not support ODR less than 12.5Hz */
enum inv_icm42607_odr {
	INV_ICM42607_ODR_1600HZ = 5,
	INV_ICM42607_ODR_800HZ = 6,
	INV_ICM42607_ODR_400HZ = 7,
	INV_ICM42607_ODR_200HZ = 8,
	INV_ICM42607_ODR_100HZ = 9,
	INV_ICM42607_ODR_50HZ = 10,
	INV_ICM42607_ODR_25HZ = 11,
	INV_ICM42607_ODR_12_5HZ = 12,
	INV_ICM42607_ODR_6_25HZ_LP = 13,
	INV_ICM42607_ODR_3_125HZ_LP = 14,
	INV_ICM42607_ODR_1_5625HZ_LP = 15,
	INV_ICM42607_ODR_NB
};

/* Low-Noise mode sensor data filter (bandwidth) */
enum inv_icm42607_filter_bw {
	INV_ICM42607_FILTER_BYPASS = 0,
	INV_ICM42607_FILTER_BW_180HZ = 1,
	INV_ICM42607_FILTER_BW_121HZ = 2,
	INV_ICM42607_FILTER_BW_73HZ = 3,
	INV_ICM42607_FILTER_BW_53HZ = 4,
	INV_ICM42607_FILTER_BW_34HZ = 5,
	INV_ICM42607_FILTER_BW_25HZ = 6,
	INV_ICM42607_FILTER_BW_16HZ = 7,
	INV_ICM42607_FILTER_BW_NB
};

/* Low-Power mode sensor data filter (averaging) */
enum inv_icm42607_filter_avg {
	INV_ICM42607_FILTER_AVG_2X = 0,
	INV_ICM42607_FILTER_AVG_4X = 1,
	INV_ICM42607_FILTER_AVG_8X = 2,
	INV_ICM42607_FILTER_AVG_16X = 3,
	INV_ICM42607_FILTER_AVG_32X = 4,
	INV_ICM42607_FILTER_AVG_64X = 5,
	/* values 6 and 7 also correspond to 64x. */
};

/* Temperature sensor data filter (bandwidth) */
enum inv_icm42607_temp_filter_bw {
	INV_ICM42607_TEMP_FILTER_BYPASS = 0,
	INV_ICM42607_TEMP_FILTER_BW_180HZ = 1,
	INV_ICM42607_TEMP_FILTER_BW_72HZ = 2,
	INV_ICM42607_TEMP_FILTER_BW_34HZ = 3,
	INV_ICM42607_TEMP_FILTER_BW_16HZ = 4,
	INV_ICM42607_TEMP_FILTER_BW_8HZ = 5,
	INV_ICM42607_TEMP_FILTER_BW_4HZ = 6,
	/* value 7 also corresponds to 4Hz */
};

/* Signed so that negative values can signify an invalid condition. */
struct inv_icm42607_sensor_conf {
	int mode;
	int fs;
	int odr;
	int filter;
};
#define INV_ICM42607_SENSOR_CONF_INIT		{ -1, -1, -1, -1 }

struct inv_icm42607_conf {
	struct inv_icm42607_sensor_conf gyro;
	struct inv_icm42607_sensor_conf accel;
	ktime_t gyro_stop; /* earliest time to stop the gyro */
};

struct inv_icm42607_hw {
	const char *name;
	const struct inv_icm42607_conf *conf;
	u8 whoami;
	bool has_gyro;
};

/**
 *  struct inv_icm42607_state - driver state variables
 *  @hw:		Hardware specific data.
 *  @lock:		lock for serializing multiple registers access.
 *  @map:		regmap pointer.
 *  @indio_accel:	accelerometer IIO device.
 *  @indio_gyro:	gyroscope IIO device.
 *  @vddio_supply:	I/O voltage regulator for the chip.
 *  @vddio_en:		I/O voltage status for runtime PM.
 *  @conf:		chip sensors configurations.
 *  @orientation:	sensor chip orientation relative to main hardware.
 */
struct inv_icm42607_state {
	const struct inv_icm42607_hw *hw;
	struct mutex lock;
	struct regmap *map;
	struct iio_dev *indio_accel;
	struct iio_dev *indio_gyro;
	struct regulator *vddio_supply;
	bool vddio_en;
	struct inv_icm42607_conf conf;
	struct iio_mount_matrix orientation;
};

/**
 * struct inv_icm42607_sensor_state - sensor state variables
 * @power_mode:		sensor requested power mode (for common frequencies)
 * @filter:		sensor filter.
 */
struct inv_icm42607_sensor_state {
	enum inv_icm42607_sensor_mode power_mode;
	int filter;
};

/*
 * Virtual register addresses: bank id in the upper byte, register address in
 * the lower byte. Bank 0 is directly addressable on the bus. MREG banks are
 * reached through the BLK_SEL/MADDR/M indirect access window, one byte per
 * transaction (datasheet section 13, no burst support).
 *
 * MREG1 programs BLK_SEL = 0x00, which collides with the direct bank, so it
 * gets the distinct virtual id 0x01. MREG2 and MREG3 virtual ids match their
 * BLK_SEL values.
 */
#define INV_ICM42607_REG_BANK_MASK			GENMASK(15, 8)
#define INV_ICM42607_REG_ADDR_MASK			GENMASK(7, 0)

#define INV_ICM42607_BANK0					0x00
#define INV_ICM42607_MREG1					0x01
#define INV_ICM42607_MREG2					0x28
#define INV_ICM42607_MREG3					0x50

/* BLK_SEL value written to the device for each virtual bank id. */
#define INV_ICM42607_MREG1_BLK_SEL			0x00
#define INV_ICM42607_MREG2_BLK_SEL			0x28
#define INV_ICM42607_MREG3_BLK_SEL			0x50

/*
 * Datasheet section 13, Accessing MREGx Registers: no bus access for 10us
 * after programming the indirect access window.
 */
#define INV_ICM42607_MREG_ACCESS_DELAY_US		10

/* Register Map for User Bank 0 */
#define INV_ICM42607_REG_MCLK_RDY			0x0000
#define INV_ICM42607_MCLK_RDY_BIT			BIT(3)

#define INV_ICM42607_REG_DEVICE_CONFIG			0x0001
#define INV_ICM42607_DEVICE_CONFIG_SPI_AP_4WIRE		BIT(2)
#define INV_ICM42607_DEVICE_CONFIG_SPI_MODE		BIT(0)

#define INV_ICM42607_REG_SIGNAL_PATH_RESET		0x0002
#define INV_ICM42607_SIGNAL_PATH_RESET_SOFT_RESET	BIT(4)
#define INV_ICM42607_SIGNAL_PATH_RESET_FIFO_FLUSH	BIT(2)

#define INV_ICM42607_REG_DRIVE_CONFIG1			0x0003
#define INV_ICM42607_DRIVE_CONFIG1_I3C_DDR_MASK		GENMASK(5, 3)
#define INV_ICM42607_DRIVE_CONFIG1_I3C_SDR_MASK		GENMASK(2, 0)

#define INV_ICM42607_REG_DRIVE_CONFIG2			0x0004
#define INV_ICM42607_DRIVE_CONFIG2_I2C_MASK		GENMASK(5, 3)
#define INV_ICM42607_DRIVE_CONFIG2_ALL_MASK		GENMASK(2, 0)

#define INV_ICM42607_REG_DRIVE_CONFIG3			0x0005
#define INV_ICM42607_DRIVE_CONFIG3_SPI_MASK		GENMASK(2, 0)

#define INV_ICM42607_REG_INT_CONFIG			0x0006
#define INV_ICM42607_INT_CONFIG_INT2_LATCHED		BIT(5)
#define INV_ICM42607_INT_CONFIG_INT2_PUSH_PULL		BIT(4)
#define INV_ICM42607_INT_CONFIG_INT2_ACTIVE_HIGH	BIT(3)
#define INV_ICM42607_INT_CONFIG_INT2_ACTIVE_LOW		0x00
#define INV_ICM42607_INT_CONFIG_INT1_LATCHED		BIT(2)
#define INV_ICM42607_INT_CONFIG_INT1_PUSH_PULL		BIT(1)
#define INV_ICM42607_INT_CONFIG_INT1_ACTIVE_HIGH	BIT(0)
#define INV_ICM42607_INT_CONFIG_INT1_ACTIVE_LOW		0x00

/* all sensor data are 16 bits (2 registers wide) in big-endian */
#define INV_ICM42607_REG_TEMP_DATA1			0x0009
#define INV_ICM42607_REG_TEMP_DATA0			0x000A
#define INV_ICM42607_REG_ACCEL_DATA_X1			0x000B
#define INV_ICM42607_REG_ACCEL_DATA_X0			0x000C
#define INV_ICM42607_REG_ACCEL_DATA_Y1			0x000D
#define INV_ICM42607_REG_ACCEL_DATA_Y0			0x000E
#define INV_ICM42607_REG_ACCEL_DATA_Z1			0x000F
#define INV_ICM42607_REG_ACCEL_DATA_Z0			0x0010
#define INV_ICM42607_REG_GYRO_DATA_X1			0x0011
#define INV_ICM42607_REG_GYRO_DATA_X0			0x0012
#define INV_ICM42607_REG_GYRO_DATA_Y1			0x0013
#define INV_ICM42607_REG_GYRO_DATA_Y0			0x0014
#define INV_ICM42607_REG_GYRO_DATA_Z1			0x0015
#define INV_ICM42607_REG_GYRO_DATA_Z0			0x0016
#define INV_ICM42607_DATA_INVALID			-32768

#define INV_ICM42607_REG_TMST_FSYNCH			0x0017
#define INV_ICM42607_REG_TMST_FSYNCL			0x0018

/* APEX Data Registers */
#define INV_ICM42607_REG_APEX_DATA0			0x0031
#define INV_ICM42607_REG_APEX_DATA1			0x0032
#define INV_ICM42607_REG_APEX_DATA2			0x0033
#define INV_ICM42607_REG_APEX_DATA3			0x0034
#define INV_ICM42607_REG_APEX_DATA4			0x001D
#define INV_ICM42607_REG_APEX_DATA5			0x001E

#define INV_ICM42607_REG_PWR_MGMT0			0x001F
#define INV_ICM42607_PWR_MGMT0_ACCEL_LP_CLK_SEL		BIT(7)
#define INV_ICM42607_PWR_MGMT0_IDLE			BIT(4)
#define INV_ICM42607_PWR_MGMT0_GYRO_MODE_MASK		GENMASK(3, 2)
#define INV_ICM42607_PWR_MGMT0_ACCEL_MODE_MASK		GENMASK(1, 0)

#define INV_ICM42607_REG_GYRO_CONFIG0			0x0020
#define INV_ICM42607_GYRO_CONFIG0_FS_SEL_MASK		GENMASK(6, 5)
#define INV_ICM42607_GYRO_CONFIG0_ODR_MASK		GENMASK(3, 0)

#define INV_ICM42607_REG_ACCEL_CONFIG0			0x0021
#define INV_ICM42607_ACCEL_CONFIG0_FS_SEL_MASK		GENMASK(6, 5)
#define INV_ICM42607_ACCEL_CONFIG0_ODR_MASK		GENMASK(3, 0)

#define INV_ICM42607_REG_TEMP_CONFIG0			0x0022
#define INV_ICM42607_TEMP_CONFIG0_FILTER_MASK		GENMASK(6, 4)

#define INV_ICM42607_REG_GYRO_CONFIG1			0x0023
#define INV_ICM42607_GYRO_CONFIG1_FILTER_MASK		GENMASK(2, 0)

#define INV_ICM42607_REG_ACCEL_CONFIG1			0x0024
#define INV_ICM42607_ACCEL_CONFIG1_AVG_MASK		GENMASK(6, 4)
#define INV_ICM42607_ACCEL_CONFIG1_FILTER_MASK		GENMASK(2, 0)

#define INV_ICM42607_REG_APEX_CONFIG0			0x0025
#define INV_ICM42607_APEX_CONFIG0_DMP_POWER_SAVE_EN	BIT(3)
#define INV_ICM42607_APEX_CONFIG0_DMP_INIT_EN		BIT(2)
#define INV_ICM42607_APEX_CONFIG0_DMP_MEM_RESET_EN	BIT(0)

#define INV_ICM42607_REG_APEX_CONFIG1			0x0026
#define INV_ICM42607_APEX_CONFIG1_SMD_ENABLE		BIT(6)
#define INV_ICM42607_APEX_CONFIG1_FF_ENABLE		BIT(5)
#define INV_ICM42607_APEX_CONFIG1_TILT_ENABLE		BIT(4)
#define INV_ICM42607_APEX_CONFIG1_PED_ENABLE		BIT(3)
#define INV_ICM42607_APEX_CONFIG1_DMP_ODR_MASK		GENMASK(1, 0)

#define INV_ICM42607_REG_WOM_CONFIG			0x0027
#define INV_ICM42607_WOM_CONFIG_INT_DUR_MASK		GENMASK(4, 3)
#define INV_ICM42607_WOM_CONFIG_INT_MODE		BIT(2)
#define INV_ICM42607_WOM_CONFIG_MODE			BIT(1)
#define INV_ICM42607_WOM_CONFIG_EN			BIT(0)

#define INV_ICM42607_REG_FIFO_CONFIG1			0x0028
#define INV_ICM42607_FIFO_CONFIG1_MODE			BIT(1)
#define INV_ICM42607_FIFO_CONFIG1_BYPASS		BIT(0)

#define INV_ICM42607_REG_FIFO_CONFIG2			0x0029
#define INV_ICM42607_REG_FIFO_CONFIG3			0x002A
#define INV_ICM42607_FIFO_WATERMARK_VAL(_wm)		\
		cpu_to_le16((_wm) & GENMASK(11, 0))
/* FIFO is 2048 bytes, let 12 samples for reading latency */
#define INV_ICM42607_FIFO_WATERMARK_MAX			(2048 - 12 * 16)
#define INV_ICM42607_FIFO_1SENSOR_PACKET_SIZE		8
#define INV_ICM42607_FIFO_2SENSORS_PACKET_SIZE		16

#define INV_ICM42607_REG_INT_SOURCE0			0x002B
#define INV_ICM42607_INT_SOURCE0_ST_INT1_EN		BIT(7)
#define INV_ICM42607_INT_SOURCE0_FSYNC_INT1_EN		BIT(6)
#define INV_ICM42607_INT_SOURCE0_PLL_RDY_INT1_EN	BIT(5)
#define INV_ICM42607_INT_SOURCE0_RESET_DONE_INT1_EN	BIT(4)
#define INV_ICM42607_INT_SOURCE0_DRDY_INT1_EN		BIT(3)
#define INV_ICM42607_INT_SOURCE0_FIFO_THS_INT1_EN	BIT(2)
#define INV_ICM42607_INT_SOURCE0_FIFO_FULL_INT1_EN	BIT(1)
#define INV_ICM42607_INT_SOURCE0_AGC_RDY_INT1_EN	BIT(0)

#define INV_ICM42607_REG_INT_SOURCE1			0x002C
#define INV_ICM42607_INT_SOURCE1_I3C_ERROR_INT1_EN	BIT(6)
#define INV_ICM42607_INT_SOURCE1_SMD_INT1_EN		BIT(3)
#define INV_ICM42607_INT_SOURCE1_WOM_INT1_EN		GENMASK(2, 0)

#define INV_ICM42607_REG_INT_SOURCE3			0x002D
#define INV_ICM42607_INT_SOURCE3_ST_INT2_EN		BIT(7)
#define INV_ICM42607_INT_SOURCE3_FSYNC_INT2_EN		BIT(6)
#define INV_ICM42607_INT_SOURCE3_PLL_RDY_INT2_EN	BIT(5)
#define INV_ICM42607_INT_SOURCE3_RESET_DONE_INT2_EN	BIT(4)
#define INV_ICM42607_INT_SOURCE3_DRDY_INT2_EN		BIT(3)
#define INV_ICM42607_INT_SOURCE3_FIFO_THS_INT2_EN	BIT(2)
#define INV_ICM42607_INT_SOURCE3_FIFO_FULL_INT2_EN	BIT(1)
#define INV_ICM42607_INT_SOURCE3_AGC_RDY_INT2_EN	BIT(0)

#define INV_ICM42607_REG_INT_SOURCE4			0x002E
#define INV_ICM42607_INT_SOURCE4_I3C_ERROR_INT2_EN	BIT(6)
#define INV_ICM42607_INT_SOURCE4_SMD_INT2_EN		BIT(3)
#define INV_ICM42607_INT_SOURCE4_WOM_Z_INT2_EN		BIT(2)
#define INV_ICM42607_INT_SOURCE4_WOM_Y_INT2_EN		BIT(1)
#define INV_ICM42607_INT_SOURCE4_WOM_X_INT2_EN		BIT(0)

#define INV_ICM42607_REG_FIFO_LOST_PKT0			0x002F
#define INV_ICM42607_REG_FIFO_LOST_PKT1			0x0030

#define INV_ICM42607_REG_INTF_CONFIG0			0x0035
#define INV_ICM42607_INTF_CONFIG0_FIFO_COUNT_FORMAT	BIT(6)
#define INV_ICM42607_INTF_CONFIG0_FIFO_COUNT_ENDIAN	BIT(5)
#define INV_ICM42607_INTF_CONFIG0_SENSOR_DATA_ENDIAN	BIT(4)
#define INV_ICM42607_INTF_CONFIG0_UI_SIFS_CFG_MASK	GENMASK(1, 0)
#define INV_ICM42607_INTF_CONFIG0_UI_SIFS_CFG_SPI_DIS	2
#define INV_ICM42607_INTF_CONFIG0_UI_SIFS_CFG_I2C_DIS	3

#define INV_ICM42607_REG_INTF_CONFIG1			0x0036
#define INV_ICM42607_INTF_CONFIG1_I3C_SDR_EN		BIT(3)
#define INV_ICM42607_INTF_CONFIG1_I3C_DDR_EN		BIT(2)
#define INV_ICM42607_INTF_CONFIG1_CLKSEL_MASK		GENMASK(1, 0)
#define INV_ICM42607_INTF_CONFIG1_CLKSEL_INT		0
#define INV_ICM42607_INTF_CONFIG1_CLKSEL_PLL		1
#define INV_ICM42607_INTF_CONFIG1_CLKSEL_OFF		2

#define INV_ICM42607_REG_INT_STATUS_DRDY		0x0039
#define INV_ICM42607_INT_STATUS_DRDY_DATA_RDY		BIT(0)

#define INV_ICM42607_REG_INT_STATUS			0x003A
#define INV_ICM42607_INT_STATUS_ST			BIT(7)
#define INV_ICM42607_INT_STATUS_FSYNC			BIT(6)
#define INV_ICM42607_INT_STATUS_PLL_RDY			BIT(5)
#define INV_ICM42607_INT_STATUS_RESET_DONE		BIT(4)
#define INV_ICM42607_INT_STATUS_FIFO_THS		BIT(2)
#define INV_ICM42607_INT_STATUS_FIFO_FULL		BIT(1)
#define INV_ICM42607_INT_STATUS_AGC_RDY			BIT(0)

#define INV_ICM42607_REG_INT_STATUS2			0x003B
#define INV_ICM42607_INT_STATUS2_SMD			BIT(3)
#define INV_ICM42607_INT_STATUS2_WOM_INT		GENMASK(2, 0)

#define INV_ICM42607_REG_INT_STATUS3			0x003C
#define INV_ICM42607_INT_STATUS3_STEP_DET		BIT(5)
#define INV_ICM42607_INT_STATUS3_STEP_CNT_OVF		BIT(4)
#define INV_ICM42607_INT_STATUS3_TILT_DET		BIT(3)
#define INV_ICM42607_INT_STATUS3_FF_DET			BIT(2)

/*
 * FIFO access registers
 * FIFO count is 16 bits (2 registers) big-endian
 * FIFO data is a continuous read register to read FIFO content
 */
#define INV_ICM42607_REG_FIFO_COUNTH			0x003D
#define INV_ICM42607_REG_FIFO_COUNTL			0x003E
#define INV_ICM42607_REG_FIFO_DATA			0x003F

#define INV_ICM42607_REG_WHOAMI				0x0075
#define INV_ICM42370P_WHOAMI				0x0D
#define INV_ICM42607P_WHOAMI				0x60
#define INV_ICM42607_WHOAMI				0x67

#define INV_ICM42607_REG_BLK_SEL_W			0x0079
#define INV_ICM42607_REG_MADDR_W			0x007A
#define INV_ICM42607_REG_M_W				0x007B
#define INV_ICM42607_REG_BLK_SEL_R			0x007C
#define INV_ICM42607_REG_MADDR_R			0x007D
#define INV_ICM42607_REG_M_R				0x007E

/* User Bank MREG 1 registers */
#define INV_ICM42607_REG_OFFSET_USER0			0x014E
#define INV_ICM42607_REG_OFFSET_USER1			0x014F
#define INV_ICM42607_REG_OFFSET_USER2			0x0150
#define INV_ICM42607_REG_OFFSET_USER3			0x0151
#define INV_ICM42607_REG_OFFSET_USER4			0x0152
#define INV_ICM42607_REG_OFFSET_USER5			0x0153
#define INV_ICM42607_REG_OFFSET_USER6			0x0154
#define INV_ICM42607_REG_OFFSET_USER7			0x0155
#define INV_ICM42607_REG_OFFSET_USER8			0x0156

/* User Bank MREG 3 registers */
#define INV_ICM42607_REG_ZG_ST_DATA			0x5005
/*
 * Timings as listed in section 3 of datasheet, all values listed in datasheet
 * in ms except temp startup time... setting all values in us and using
 * USEC_PER_MSEC to convert from values displayed in datasheet.
 */
#define INV_ICM42607_POWER_UP_TIME_US			(100 * USEC_PER_MSEC)
#define INV_ICM42607_RESET_TIME_US			(1 * USEC_PER_MSEC)
#define INV_ICM42607_ACCEL_STARTUP_TIME_US		(10 * USEC_PER_MSEC)
#define INV_ICM42607_GYRO_STARTUP_TIME_US		(30 * USEC_PER_MSEC)
#define INV_ICM42607_GYRO_STOP_TIME_US			(45 * USEC_PER_MSEC)
#define INV_ICM42607_TEMP_STARTUP_TIME_US		77

/*
 * Suspend delay assumed from other icm42600 series device, not
 * documented in datasheet.
 */
#define INV_ICM42607_SUSPEND_DELAY_MS			(2 * MSEC_PER_SEC)

typedef int (*inv_icm42607_bus_setup)(struct inv_icm42607_state *);

extern const struct regmap_config inv_icm42607_regmap_config;
extern const struct inv_icm42607_hw inv_icm42370p_hw_data;
extern const struct inv_icm42607_hw inv_icm42607_hw_data;
extern const struct inv_icm42607_hw inv_icm42607p_hw_data;
extern const struct dev_pm_ops inv_icm42607_pm_ops;

const struct iio_mount_matrix *
inv_icm42607_get_mount_matrix(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan);

int inv_icm42607_get_pwr_mgmt0(struct inv_icm42607_state *st,
			       enum inv_icm42607_sensor_mode *gyro,
			       enum inv_icm42607_sensor_mode *accel);

int inv_icm42607_set_sensor_conf(struct inv_icm42607_state *st,
				 struct inv_icm42607_sensor_conf *conf,
				 enum iio_chan_type chan_type);

int inv_icm42607_read_sensor(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     s16 *val);

int inv_icm42607_core_probe(struct regmap *regmap,
			    const struct inv_icm42607_hw *hw,
			    inv_icm42607_bus_setup bus_setup);

struct iio_dev *inv_icm42607_gyro_init(struct inv_icm42607_state *st);

struct iio_dev *inv_icm42607_accel_init(struct inv_icm42607_state *st);

#endif
