// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Analog Devices Inc.
 *
 * Driver for the LTC4296 SPoE PSE.
 *
 * Author: Antoniu Miclaus <antoniu.miclaus@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include <asm/unaligned.h>

#define LTC4296_REG_GFLTEV        	0x02
#define LTC4296_REG_GFLTMSK       	0x03
#define LTC4296_REG_GCAP          	0x06
#define LTC4296_REG_GIOST         	0x07
#define LTC4296_REG_GCMD          	0x08
#define LTC4296_REG_GCFG          	0x09
#define LTC4296_REG_GADCCFG       	0x0A
#define LTC4296_REG_GADCDAT       	0x0B
#define LTC4296_REG_P0EV          	0x10
#define LTC4296_REG_P0ST          	0x12
#define LTC4296_REG_P0CFG0        	0x13
#define LTC4296_REG_P0CFG1        	0x14
#define LTC4296_REG_P0ADCCFG      	0x15
#define LTC4296_REG_P0ADCDAT      	0x16
#define LTC4296_REG_P0SELFTEST    	0x17
#define LTC4296_REG_P1EV          	0x20
#define LTC4296_REG_P1ST          	0x22
#define LTC4296_REG_P1CFG0        	0x23
#define LTC4296_REG_P1CFG1        	0x24
#define LTC4296_REG_P1ADCCFG      	0x25
#define LTC4296_REG_P1ADCDAT      	0x26
#define LTC4296_REG_P1SELFTEST    	0x27
#define LTC4296_REG_P2EV          	0x30
#define LTC4296_REG_P2ST          	0x32
#define LTC4296_REG_P2CFG0        	0x33
#define LTC4296_REG_P2CFG1       	0x34
#define LTC4296_REG_P2ADCCFG      	0x35
#define LTC4296_REG_P2ADCDAT      	0x36
#define LTC4296_REG_P2SELFTEST   	0x37
#define LTC4296_REG_P3EV          	0x40
#define LTC4296_REG_P3ST          	0x42
#define LTC4296_REG_P3CFG0        	0x43
#define LTC4296_REG_P3CFG1        	0x44
#define LTC4296_REG_P3ADCCFG      	0x45
#define LTC4296_REG_P3ADCDAT     	0x46
#define LTC4296_REG_P3SELFTEST    	0x47
#define LTC4296_REG_P4EV          	0x50
#define LTC4296_REG_P4ST          	0x52
#define LTC4296_REG_P4CFG0        	0x53
#define LTC4296_REG_P4CFG1      	0x54
#define LTC4296_REG_P4ADCCFG      	0x55
#define LTC4296_REG_P4ADCDAT     	0x56
#define LTC4296_REG_P4SELFTEST    	0x57

/* LTC4296_REG_GFLTEV */
#define LTC4296_UVLO_DIGITAL_MSK	BIT(4)
#define LTC4296_COMMAND_FAULT_MSK	BIT(3)
#define LTC4296_PEC_FAULT_MSK		BIT(2)
#define LTC4296_MEMORY_FAULT_MSK	BIT(1)
#define LTC4296_LOW_CKT_BRK_FAULT_MSK	BIT(0)

/* LTC4296_REG_GCAP */
#define LTC4296_SCCP_SUPPORT_MSK             BIT(6)
#define LTC4296_WAKE_FWD_SUPPORT_MSK         BIT(5)
#define LTC4296_NUMPORTS_MSK                 GENMASK(4, 0)

/* LTC4296_REG_GIOST */
#define LTC4296_PG_OUT4_MSK			BIT(8)
#define LTC4296_PG_OUT3_MSK			BIT(7)
#define LTC4296_PG_OUT2_MSK			BIT(6)
#define LTC4296_PG_OUT1_MSK			BIT(5)
#define LTC4296_PG_OUT0_MSK			BIT(4)
#define LTC4296_PAD_AUTO_MSK			BIT(3)
#define LTC4296_PAD_WAKEUP_MSK			BIT(2)
#define LTC4296_PAD_WAKEUP_DRIVE_MSK		BIT(1)

/* LTC4296_REG_GCMD */
#define LTC4296_SW_RESET_MSK			GENMASK(15, 8)
#define LTC4296_WRITE_PROTECT_MSK		GENMASK(7, 0)

/* LTC4296_REG_GCFG */
#define LTC4296_MASK_LOWFAULT_MSK		BIT(5)
#define LTC4296_TLIM_DISABLE_MSK		BIT(4)
#define LTC4296_TLIM_TIMER_SLEEP_MSK		GENMASK(3, 2)
#define LTC4296_REFRESH_MSK			BIT(1)
#define LTC4296_SW_VIN_PGOOD_MSK		BIT(0)

/* LTC4296_REG_GADCCFG */
#define LTC4296_GADC_SAMPLE_MODE_MSK		GENMASK(6, 5)
#define LTC4296_GADC_SEL_MSK			GENMASK(4, 0)

/* LTC4296_REG_GADCDAT */
#define LTC4296_GADC_MISSED_MSK			BIT(13)
#define LTC4296_GADC_NEW_MSK			BIT(12)
#define LTC4296_GADC_MSK			GENMASK(11, 0)

/* LTC4296_REG_PXEV */
#define LTC4296_VALID_SIGNATURE_MSK		BIT(9)
#define LTC4296_INVALID_SIGNATURE_MSK		BIT(8)
#define LTC4296_TOFF_TIMER_DONE_MSK		BIT(7)
#define LTC4296_OVERLOAD_DETECTED_ISLEEP_MSK	BIT(6)
#define LTC4296_OVERLOAD_DETECTED_IPOWERED_MSK	BIT(5)
#define LTC4296_MFVS_TIMEOUT_MSK		BIT(4)
#define LTC4296_TINRUSH_TIMER_DONE_MSK		BIT(3)
#define LTC4296_PD_WAKEUP_MSK			BIT(2)
#define LTC4296_LSNS_FORWARD_FAULT_MSK		BIT(1)
#define LTC4296_LSNS_REVERSE_FAULT_MSK		BIT(0)

/* LTC4296_REG_PXST */
#define LTC4296_DET_VHIGH_MSK			BIT(13)
#define LTC4296_DET_VLOW_MSK			BIT(12)
#define LTC4296_POWER_STABLE_HI_MSK		BIT(11)
#define LTC4296_POWER_STABLE_LO_MSK		BIT(10)
#define LTC4296_POWER_STABLE_MSK		BIT(9)
#define LTC4296_OVERLOAD_HELD_MSK		BIT(8)
#define LTC4296_PI_SLEEPING_MSK			BIT(7)
#define LTC4296_PI_PREBIASED_MSK		BIT(6)
#define LTC4296_PI_DETECTING_MSK		BIT(5)
#define LTC4296_PI_POWERED_MSK			BIT(4)
#define LTC4296_PI_DISCHARGE_EN_MSK		BIT(3)
#define LTC4296_PSE_STATUS_MSK			GENMASK(2, 0)

/* LTC4296_REG_PXCFG0 */
#define LTC4296_SW_INRUSH_MSK			BIT(15)
#define LTC4296_END_CLASSIFICATION_MSK		BIT(14)
#define LTC4296_SET_CLASSIFICATION_MODE_MSK	BIT(13)
#define LTC4296_DISABLE_DETECTION_PULLUP_MSK	BIT(12)
#define LTC4296_TDET_DISABLE_MSK		BIT(11)
#define LTC4296_FOLDBACK_DISABLE_MSK		BIT(10)
#define LTC4296_SOFT_START_DISABLE_MSK		BIT(9)
#define LTC4296_TOFF_TIMER_DISABLE_MSK		BIT(8)
#define LTC4296_TMFVDO_TIMER_DISABLE_MSK	BIT(7)
#define LTC4296_SW_PSE_READY_MSK		BIT(6)
#define LTC4296_SW_POWER_AVAILABLE_MSK		BIT(5)
#define LTC4296_UPSTREAM_WAKEUP_DISABLE_MSK	BIT(4)
#define LTC4296_DOWNSTREAM_WAKEUP_DISABLE_MSK	BIT(3)
#define LTC4296_SW_PSE_WAKEUP_MSK		BIT(2)
#define LTC4296_HW_EN_MASK_MSK			BIT(1)
#define LTC4296_SW_EN_MSK			BIT(0)

/* LTC4296_REG_PXCFG1 */
#define LTC4296_PREBIAS_OVERRIDE_GOOD_MSK	BIT(8)
#define LTC4296_TLIM_TIMER_TOP_MSK		GENMASK(7, 6)
#define LTC4296_TOD_TRESTART_TIMER_MSK		GENMASK(5, 4)
#define LTC4296_TINRUSH_TIMER_MSK		GENMASK(3, 2)
#define LTC4296_SIG_OVERRIDE_BAD_MSK		BIT(1)
#define LTC4296_SIG_OVERRIDE_GOOD_MSK		BIT(0)

/* LTC4296_REG_PXADCCFG */
#define LTC4296_MFVS_THRESHOLD_MSK		GENMASK(7, 0)

/* LTC4296_REG_PXADCDAT */
#define LTC4296_MISSED_MSK			BIT(13)
#define LTC4296_NEW_MSK				BIT(12)
#define LTC4296_SOURCE_CURRENT_MSK		GENMASK(11, 0)

/* Miscellaneous Definitions*/
#define LTC4296_RESET_CODE			0x73
#define LTC4296_UNLOCK_KEY			0x05
#define LTC4296_LOCK_KEY			0xA0

#define LTC4296_VGAIN				35230 / 1000
#define LTC4296_IGAIN				1 / 10

#define LTC4296_VMAX         			1
#define LTC4296_VMIN        			0

#define LTC4296_MAX_PORTS			5

#define RTESTLOAD              			200 /*(ohm)*/


enum ltc4296_state_dev {
	LTC_UNLOCKED = 0,
	LTC_LOCKED
};

enum ltc4296_port {
	LTC_PORT0 = 0,
	LTC_PORT1,
	LTC_PORT2,
	LTC_PORT3,
	LTC_PORT4,
	LTC_NO_PORT
};

enum ltc4296_port_status {
	LTC_PORT_DISABLED = 0,
	LTC_PORT_ENABLED
};

enum ltc4296_pse_status {
	LTC_PSE_STATUS_DISABLED = 0,      /*  000b Port is disabled                */
	LTC_PSE_STATUS_SLEEPING,          /*  001b Port is in sleeping             */
	LTC_PSE_STATUS_DELIVERING,        /*  010b Port is delivering power        */
	LTC_PSE_STATUS_SEARCHING,         /*  011b Port is searching               */
	LTC_PSE_STATUS_ERROR,             /*  100b Port is in error                */
	LTC_PSE_STATUS_IDLE,              /*  101b Port is idle                    */
	LTC_PSE_STATUS_PREPDET,           /*  110b Port is preparing for detection */
	LTC_PSE_STATUS_UNKNOWN            /*  111b Port is in an unknown state     */
};

enum ltc4296_board_class {
	SPOE_CLASS10 = 0,
	SPOE_CLASS11,
	SPOE_CLASS12,
	SPOE_CLASS13,
	SPOE_CLASS14,
	SPOE_CLASS15,
	APL_CLASSA,
	APL_CLASSA_NOAUTONEG,
	APL_CLASSC,
	APL_CLASS3,
	PRODUCTION_POWER_TEST,
	APL_CLASSA_OLD_DEMO,
	SPOE_OFF,
	PRODUCTION_DATA_TEST,
	RESERVED,
	DEBUGMODE
};

enum ltc4296_config {
	LTC_CFG_SCCP_MODE = 0,
	LTC_CFG_APL_MODE,
	LTC_CFG_RESET
};

enum ltc4296_port_reg_offset_e {
	LTC_PORT_EVENTS   = 0,
	LTC_PORT_STATUS   = 2,
	LTC_PORT_CFG0     = 3,
	LTC_PORT_CFG1     = 4,
	LTC_PORT_ADCCFG   = 5,
	LTC_PORT_ADCDAT   = 6,
	LTC_PORT_SELFTEST = 7
};

enum adi_ltc_result
{
	ADI_LTC_SUCCESS = 0,                  /*!< Success                                                    */
	ADI_LTC_DISCONTINUE_SCCP,             /*!< Discontinue the SCCP configuration cycle.                  */
	ADI_LTC_SCCP_COMPLETE,                /*!< Complete SCCP configuration cycle.                         */
	ADI_LTC_SCCP_PD_DETECTION_FAILED,     /*!< PD Detection failed                                        */
	ADI_LTC_SCCP_PD_NOT_PRESENT,          /*!< SCCP  PD not present                                       */
	ADI_LTC_SCCP_PD_RES_INVALID,          /*!< PD Response is invalid                                     */
	ADI_LTC_SCCP_PD_PRESENT,              /*!< PD is present.                                             */
	ADI_LTC_SCCP_PD_CLASS_COMPATIBLE,     /*!< PD Class is compatible                                     */
	ADI_LTC_SCCP_PD_CLASS_NOT_SUPPORTED,  /*!< PD Class is out of range                                   */
	ADI_LTC_SCCP_PD_CLASS_NOT_COMPATIBLE, /*!< PD Class is not compatible                                 */
	ADI_LTC_SCCP_PD_LINE_NOT_HIGH,        /*!< PD line has not gone HIGH                                  */
	ADI_LTC_SCCP_PD_LINE_NOT_LOW,         /*!< PD line has not gone LOW                                   */
	ADI_LTC_SCCP_PD_CRC_FAILED,           /*!< CRC received from PD is incorrect                          */
	ADI_LTC_APL_COMPLETE,                 /*!< Complete APL configuration cycle.                          */
	ADI_LTC_DISCONTINUE_APL,              /*!< Discontinue the APL configuration cycle.                   */
	ADI_LTC_INVALID_ADC_VOLTAGE,          /*!< Invalid ADC Accumulation result.                           */
	ADI_LTC_INVALID_ADC_PORT_CURRENT,     /*!< Invalid ADC Port Current                                   */
	ADI_LTC_TEST_COMPLETE,                /*!< LTC Test complete.                                         */
	ADI_LTC_DISCONTINUE_TEST,             /*!< LTC Discontinue Test.                                      */
	ADI_LTC_TEST_FAILED,                  /*!< LTC Test Failed.                                           */
	ADI_LTC_INVALID_VIN                   /*!< VIN is invalid                                             */
};

struct ltc4296_vi
{
	int ltc4296_vin;
	int ltc4296_vout;
	int ltc4296_iout;
	bool  ltc4296_print_vin;
};

struct ltc4296_state {
	struct spi_device *spi;
	struct ltc4296_vi ltc4296_vi;
	u8 data[5];
};

static u8 ltc4296_get_pec_byte(u8 data, u8 seed)
{
	u8 pec = seed;
	u8 din, in0, in1, in2;
	int bit;

	for(bit = 7; bit >= 0; bit--) {
		din = (data >> bit) & 0x01;
		in0 = din ^ ( (pec >> 7) & 0x01 );
		in1 = in0 ^ ( pec & 0x01);
		in2 = in0 ^ ( (pec >> 1) & 0x01 );
		pec = (pec << 1);
		pec &= ~(0x07);
		pec = pec | in0 | (in1 << 1) | (in2 << 2);
	}

	return pec;
}

static int ltc4296_spi_write(struct ltc4296_state *st, unsigned int reg, u16 val)
{
	st->data[0] = reg << 1 | 0x0;
	st->data[1] = ltc4296_get_pec_byte(st->data[0], 0x41);
	st->data[2] = val >> 8;
	st->data[3] = val & 0xFF;
	st->data[4] = ltc4296_get_pec_byte(st->data[3], ltc4296_get_pec_byte(st->data[2], 0x41));

	return spi_write(st->spi, &st->data[0], 5);
}

static int ltc4296_spi_read(struct ltc4296_state *st, unsigned int reg,
			     u16 *val)
{
	int ret;
	struct spi_transfer t = {0};

	t.tx_buf = &st->data[0];
	t.rx_buf = &st->data[0];
	t.len = 5;

    	st->data[0] = reg << 1 | 0x1;
    	st->data[1] = ltc4296_get_pec_byte(st->data[0], 0x41);


	ret = spi_sync_transfer(st->spi, &t, 1);
	if (ret)
		return ret;

    	*val = get_unaligned_be16(&st->data[2]);

	return 0;
}

static umode_t ltc4296_is_visible(const void *data, enum hwmon_sensor_types type,
				   u32 attr, int channel)
{
	return 0;
}

static int ltc4296_write(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	return 0;
}

static int ltc4296_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{
	return 0;
}

static const struct hwmon_channel_info *ltc4296_info[] = {
	HWMON_CHANNEL_INFO(chip,
			   HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_ENABLE | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_ENABLE | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_ENABLE | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_ENABLE | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL),
	HWMON_CHANNEL_INFO(curr,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL,
			   HWMON_C_INPUT | HWMON_C_LABEL),
	NULL
};

static const struct hwmon_ops ltc4296_hwmon_ops = {
	.is_visible = ltc4296_is_visible,
	.write = ltc4296_write,
	.read = ltc4296_read,
};

static const struct hwmon_chip_info ltc4296_chip_info = {
	.ops = &ltc4296_hwmon_ops,
	.info = ltc4296_info,
};

int ltc4296_reset(struct ltc4296_state *st)
{
	int ret;

	ret = ltc4296_spi_write(st, LTC4296_REG_GCMD, LTC4296_RESET_CODE);
	if (ret != 0) {
		return ret;
	}

	usleep_range(10000, 10500);

	return 0;
}

static int ltc4296_probe(struct spi_device *spi)
{
	struct ltc4296_state *st;
	struct device *hwmon_dev;
	u16 value;
	int ret;

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->spi = spi;

	ret = ltc4296_reset(st);
	if (ret)
		return ret;

	ret = ltc4296_spi_write(st, LTC4296_REG_GCMD, LTC4296_UNLOCK_KEY);
	if (ret)
		return ret;

	ret = ltc4296_spi_read(st, LTC4296_REG_GCMD, &value);
	if (ret)
		return ret;

	if (value != LTC4296_UNLOCK_KEY) {
		dev_err_probe(&spi->dev, -EINVAL, "Device locked. Write Access is disabled");
		return -EINVAL;
	}

	hwmon_dev = devm_hwmon_device_register_with_info(&spi->dev, spi->dev.driver->name,
							 st, &ltc4296_chip_info,
							 NULL);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct spi_device_id ltc4296_id[] = {
	{ "ltc4296", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ltc4296_id);

static const struct of_device_id ltc4296_of_match[] = {
	{ .compatible = "adi,ltc4296", },
	{ },
};
MODULE_DEVICE_TABLE(of, ltc4296_of_match);

static struct spi_driver ltc4296_driver = {
	.probe = ltc4296_probe,
	.driver = {
		.name = "ltc4296",
		.of_match_table	= of_match_ptr(ltc4296_of_match),
	},
	.id_table = ltc4296_id,
};
module_spi_driver(ltc4296_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_DESCRIPTION("LTC4296 SPoE PSE");
