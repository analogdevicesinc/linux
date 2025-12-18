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
#include <linux/of.h>
#include <linux/pse-pd/pse.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>

#include <dt-bindings/net/pse-pd/adi,ltc4296.h>

#define LTC4296_REG_GFLTEV			0x02
#define LTC4296_REG_GFLTMSK			0x03
#define LTC4296_REG_GCAP			0x06
#define LTC4296_REG_GIOST			0x07
#define LTC4296_REG_GCMD			0x08
#define LTC4296_REG_GCFG			0x09
#define LTC4296_REG_GADCCFG			0x0A
#define LTC4296_REG_GADCDAT			0x0B
#define LTC4296_REG_P0EV			0x10
#define LTC4296_REG_P0ST			0x12
#define LTC4296_REG_P0CFG0			0x13
#define LTC4296_REG_P0CFG1			0x14
#define LTC4296_REG_P0ADCCFG			0x15
#define LTC4296_REG_P0ADCDAT			0x16
#define LTC4296_REG_P0SELFTEST			0x17
#define LTC4296_REG_P1EV			0x20
#define LTC4296_REG_P1ST			0x22
#define LTC4296_REG_P1CFG0			0x23
#define LTC4296_REG_P1CFG1			0x24
#define LTC4296_REG_P1ADCCFG			0x25
#define LTC4296_REG_P1ADCDAT			0x26
#define LTC4296_REG_P1SELFTEST			0x27
#define LTC4296_REG_P2EV			0x30
#define LTC4296_REG_P2ST			0x32
#define LTC4296_REG_P2CFG0			0x33
#define LTC4296_REG_P2CFG1			0x34
#define LTC4296_REG_P2ADCCFG			0x35
#define LTC4296_REG_P2ADCDAT			0x36
#define LTC4296_REG_P2SELFTEST			0x37
#define LTC4296_REG_P3EV			0x40
#define LTC4296_REG_P3ST			0x42
#define LTC4296_REG_P3CFG0			0x43
#define LTC4296_REG_P3CFG1			0x44
#define LTC4296_REG_P3ADCCFG			0x45
#define LTC4296_REG_P3ADCDAT			0x46
#define LTC4296_REG_P3SELFTEST			0x47
#define LTC4296_REG_P4EV			0x50
#define LTC4296_REG_P4ST			0x52
#define LTC4296_REG_P4CFG0			0x53
#define LTC4296_REG_P4CFG1			0x54
#define LTC4296_REG_P4ADCCFG			0x55
#define LTC4296_REG_P4ADCDAT			0x56
#define LTC4296_REG_P4SELFTEST			0x57

/* LTC4296_REG_GFLTEV */
#define LTC4296_UVLO_DIGITAL_MSK		BIT(4)
#define LTC4296_COMMAND_FAULT_MSK		BIT(3)
#define LTC4296_PEC_FAULT_MSK			BIT(2)
#define LTC4296_MEMORY_FAULT_MSK		BIT(1)
#define LTC4296_LOW_CKT_BRK_FAULT_MSK		BIT(0)

/* LTC4296_REG_GCAP */
#define LTC4296_SCCP_SUPPORT_MSK		BIT(6)
#define LTC4296_WAKE_FWD_SUPPORT_MSK		BIT(5)
#define LTC4296_NUMPORTS_MSK			GENMASK(4, 0)

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
#define LTC4296_GADC_SAMPLE_MODE_DISABLED	0x0
#define LTC4296_GADC_SAMPLE_MODE_SINGLE		0x1
#define LTC4296_GADC_SAMPLE_MODE_CONT_LG	0x2
#define LTC4296_GADC_SAMPLE_MODE_CONT_HG	0x3
#define LTC4296_GADC_SEL_GND			0x0
#define LTC4296_GADC_SEL_VIN			0x1
#define LTC4296_GADC_SEL_TEMP			0x2
#define LTC4296_GADC_SEL_OUT_VOLTAGE(port)	(0x4 + (port) * 2)
#define LTC4296_GADC_SEL_RETURN_VOLTAGE(port)	(LTC4296_GADC_SEL_OUT_VOLTAGE(port) + 1)
#define LTC4296_GADC_SEL_VREF			0x10

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

#define LTC4296_ADC_OFFSET			2049

#define LTC4296_VGAIN				(35230 / 1000)
#define LTC4296_IGAIN				(1 / 10)

#define LTC4296_VMAX				1
#define LTC4296_VMIN				0

#define LTC4296_MAX_PORTS			5

#define T_RSTL_NOM		9000
#define T_MSP			2000

#define SCCP_TYPE_MASK         0xF000
#define SCCP_CLASS_TYPE_MASK   0x0FFF

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

enum adi_ltc_result {
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

/* LTC4296_1_IGAIN / Rsense[port_no] */
/* {(0.1/3), (0.1/1.5), (0.1/0.68), (0.1/0.25), (0.1/0.1)} */
static const int ltc4296_spoe_rsense[LTC4296_MAX_PORTS] = {333, 666, 1470, 4000, 10000};

/* Value of RSense resistor for each port in mOhm */
static const int ltc4296_spoe_sense_resistor[LTC4296_MAX_PORTS] = {3000, 1500, 680, 250, 100};

static int ltc4296_spoe_vol_range_mv[12][2] = {
	{20000, 30000},  /* SPoE Class 10         */
	{20000, 30000},  /* SPoE Class 11         */
	{20000, 30000},  /* SPoE Class 12         */
	{50000, 58000},  /* SPoE Class 13         */
	{50000, 58000},  /* SPoE Class 14         */
	{50000, 58000},  /* SPoE Class 15         */
	{9600, 15000},   /* APL Class A           */
	{9600, 15000},   /* APL Class A noAutoNeg */
	{9600, 15000},   /* APL Class C           */
	{46000, 50000},  /* APL Class 3           */
	{9600, 15000},   /* Production Power Test */
	{9600, 15000}    /* APL Class A oldAPL    */
};

/* 
 * Row - PSE power class (starting from 10)
 * Column - PD power class (starting from 10)
 */
const u8 ltc4296_class_compatibility[6][6] = {
	{1, 1, 1, 0, 0, 0},
	{0, 1, 1, 0, 0, 0},
	{0, 0, 1, 0, 0, 0},
	{0, 0, 0, 1, 1, 1},
	{0, 0, 0, 0, 1, 1},
	{0, 0, 0, 0, 0, 1}
};

/* Top 4 bits give the SCCP Types
 *                              A,   B,   C,   D,   E
*/
const u8 sccp_types[5] = { 0xE, 0xD, 0xB, 0x7, 0xC };

/* Bottom 12 bits give the SCCP class*/
#define SCCP_CLASS_SIZE 16
u16 sccp_classes[SCCP_CLASS_SIZE] = {
    0x3FE, // class 0
    0x3FD, // class 1
    0x3FB, // class 2
    0x3F7, // class 3
    0x3F7, // class 4
    0x3EF, // class 5
    0x3DF, // class 6
    0x3BF, // class 7
    0x37F, // class 8
    0x2FF, // class 9
    0x001, // class 10
    0x002, // class 11
    0x003, // class 12
    0x004, // class 13
    0x005, // class 14
    0x006  // class 15
};

static u8 set_port_vout[LTC4296_MAX_PORTS] = {0x04, 0x06, 0x08, 0x0A, 0x0C};

struct ltc4296_vi {
	int ltc4296_vin;
	int ltc4296_vout;
	int ltc4296_iout;
	bool ltc4296_print_vin;
};

struct ltc4296_port_data {
	struct gpio_desc *sccpo;
	struct gpio_desc *sccpi;
	u32 hs_sense_resistor;
	u32 port_type;
	u8 port_num;
};

struct ltc4296_state {
	struct spi_device *spi;
	struct ltc4296_vi ltc4296_vi;
	struct ltc4296_port_data port_data[LTC4296_MAX_PORTS];
	struct pse_controller_dev pse_dev;
	struct task_struct *pd_polling;
	struct device *hwmon_dev;
	struct gpio_desc *wakeup;
	u8 data[5];

	struct mutex lock;
};

static u8 get_CRC(u8* buf)
{
	u8 byte, bit;
	u8 x3,x4,x7, in;

	u8 crc = 0;
	for(byte=0; byte<2; byte++)
	{
		for(bit=0; bit<8; bit++)
		{
			/* save some bits before shifting register */
			x3 = (crc>>3) & 0x01;
			x4 = (crc>>4) & 0x01;
			x7 = (crc>>7) & 0x01;
			in = (buf[byte] >> bit) & 0x01;
			in ^= x7;
			/* shift the register */
			crc  = (crc<<1) | in;

			/* clear bits 4 & 5 */
			crc &= ~(0x30);

			/* replace bits with xor of 'in' and prev bit */
			u8 temp = x3 ^ in;
			crc |= (temp<<4);
			temp = x4 ^ in;
			crc |= (temp<<5);
		}
	}
	return crc;
}

static void write_bit(struct ltc4296_port_data *port, u8 bit)
{
	gpiod_set_value(port->sccpo, 1);

	if (bit){
		fsleep(300);
		gpiod_set_value(port->sccpo, 0);
		fsleep(2150);//T_WRITESLOT-T_REC-T_W1L = 2.15
	} else {
		fsleep(2450); // TW0L + 0.45 = 2.45
		gpiod_set_value(port->sccpo, 0);
	}

	/*Recovery time after every bit transmit */
	fsleep(320);//0.32

	return;
}

static void transmit_byte(struct ltc4296_port_data *port, u8 tx_byte)
{
	u8 bit_pos = 0;
	while (bit_pos < 8)
	{
		u8 bit = (tx_byte>>bit_pos) & 0x01;
		write_bit(port, bit);
		bit_pos++;
	}
	return;
}

static u8 read_bit(struct ltc4296_port_data *port)
{
	u8 bit;

	gpiod_set_value(port->sccpo, 1);
	fsleep(300); //T_W1L =0.3

	gpiod_set_value(port->sccpo, 0);

	fsleep(700); //T_MSR-T_W1L = 1.225-0.3 = 700

	bit = gpiod_get_value(port->sccpi);

	fsleep(2000); //T_READSLOT-T_MSR = 3-1 =2

	fsleep(320); //T_REC
	return bit;
}

static void receive_response(struct ltc4296_port_data *port, u8* buf)
{
	volatile u8 rx_byte = 0;
	volatile u8 bytes_rxd=0;
	volatile u8 bit_pos = 0;
    	u8 sccp_buf[3] = {0, 0, 0};

	while (bytes_rxd < 3)
	{
		rx_byte = 0;
		bit_pos = 0;
		while(bit_pos < 8)
		{
			u8 bit = read_bit(port);
			rx_byte |= (bit<<bit_pos);
			bit_pos++;
		}
		sccp_buf[bytes_rxd] = rx_byte;
		bytes_rxd++;
		fsleep(5000);
	}

	buf[0] = sccp_buf[0];
	buf[1] = sccp_buf[1];
	buf[2] = sccp_buf[2];

	return;
}

static enum adi_ltc_result sccp_reset_pulse(struct ltc4296_port_data *port)
{
	enum adi_ltc_result ret= ADI_LTC_SCCP_PD_PRESENT;
	uint8_t sccpi_val;

	sccpi_val = gpiod_get_value(port->sccpi);

	/* check if the line is high before reset pulse */
	if(!sccpi_val){
		pr_err("Port %d: SCCPI line is not high before reset pulse\n", port->port_num);
		return ADI_LTC_SCCP_PD_LINE_NOT_HIGH;
	}

	gpiod_set_value(port->sccpo, 1);

	/* check to make sure line is actually getting pulled down (protect pull down fet) */
	fsleep(3000);

	sccpi_val = gpiod_get_value(port->sccpi);

	if(sccpi_val){
		pr_err("Port %d: SCCPI line is not low\n", port->port_num);

		/* release because fet must be pulling down against stronger source than a classification v source */
		gpiod_set_value(port->sccpo, 0);
		return ADI_LTC_SCCP_PD_LINE_NOT_LOW;
	}

	fsleep(T_RSTL_NOM - 3000);

	gpiod_set_value(port->sccpo, 0);

	fsleep(T_MSP);

	/* look for presence pulse */
	sccpi_val = gpiod_get_value(port->sccpi);

	if(sccpi_val)
		ret = ADI_LTC_SCCP_PD_NOT_PRESENT;
	else
		ret = ADI_LTC_SCCP_PD_PRESENT;

	return ret;
}

static enum adi_ltc_result sccp_read_write_pd(struct ltc4296_port_data *port, u8 addr, u8 cmd, u8* buf)
{
	enum adi_ltc_result ret;

	ret = sccp_reset_pulse(port);
	if(ret == ADI_LTC_SCCP_PD_NOT_PRESENT)
		return ADI_LTC_SCCP_PD_NOT_PRESENT; //PD is not present
	else if(ret == ADI_LTC_SCCP_PD_LINE_NOT_LOW)
		return ADI_LTC_SCCP_PD_LINE_NOT_LOW;
	else if(ret == ADI_LTC_SCCP_PD_LINE_NOT_HIGH)
		return ADI_LTC_SCCP_PD_LINE_NOT_HIGH;

	fsleep(5000);
	transmit_byte(port, addr);
	fsleep(5000);
	transmit_byte(port, cmd);
	fsleep(5000);
	receive_response(port, buf);

	/* Check if the received data from PD is valid */
	if (get_CRC(buf) != buf[2] )
	{
		pr_err("PD CRC Error, CRC is 0x%x \n\r", buf[2]);
		return ADI_LTC_SCCP_PD_CRC_FAILED; /* Wrong CRC*/
	}
	return ADI_LTC_SCCP_PD_PRESENT;
}

static u8 ltc4296_get_pec_byte(u8 data, u8 seed)
{
	u8 pec = seed;
	u8 din, in0, in1, in2;
	int bit;

	for (bit = 7; bit >= 0; bit--) {
		din = (data >> bit) & 0x01;
		in0 = din ^ ((pec >> 7) & 0x01);
		in1 = in0 ^ (pec & 0x01);
		in2 = in0 ^ ((pec >> 1) & 0x01);
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

static int ltc4296_reset(struct ltc4296_state *st)
{
	int ret;

	ret = ltc4296_spi_write(st, LTC4296_REG_GCMD, FIELD_PREP(GENMASK(15, 8), LTC4296_RESET_CODE));
	if (ret)
		return ret;

	fsleep(100000);

	return 0;
}

static int ltc4296_get_port_addr(enum ltc4296_port port_no,
				 enum ltc4296_port_reg_offset_e port_offset,
				 u8 *port_addr)
{
	if (!port_addr)
		return -EINVAL;

	*port_addr = (((port_no + 1) << 4) + port_offset);

	return 0;
}

static int ltc4296_clear_global_faults(struct ltc4296_state *st)
{
	/* GFLTEV = clear all faults */
	/* LTC4296_LOW_CKT_BRK_FAULT | LTC4296_MEMORY_FAULT | */
	/* LTC4296_PEC_FAULT | LTC4296_COMMAND_FAULT | LTC4296_UVLO_DIGITAL */
	return ltc4296_spi_write(st, LTC4296_REG_GFLTEV, 0x001F);
}

static int ltc4296_clear_ckt_breaker(struct ltc4296_state *st)
{
	int ret;
	u16 val = 0;

	if (!st)
		return -EINVAL;

	ret = ltc4296_spi_read(st, LTC4296_REG_GFLTEV, &val);
	if (ret)
		return ret;

	val = val | LTC4296_LOW_CKT_BRK_FAULT_MSK;

	/* GFLTEV = clear LTC4296_LOW_CKT_BRK_FAULT by writing 1 to the bit */
	return ltc4296_spi_write(st, LTC4296_REG_GFLTEV, val);
}

static int ltc4296_read_global_faults(struct ltc4296_state *st, u16 *g_events)
{
	if (!st)
		return -EINVAL;

	/* GFLTEV = clear all faults */
	/* LTC4296_LOW_CKT_BRK_FAULT | LTC4296_MEMORY_FAULT | */
	/* LTC4296_PEC_FAULT | LTC4296_COMMAND_FAULT | LTC4296_UVLO_DIGITAL */
	return ltc4296_spi_read(st, LTC4296_REG_GFLTEV, g_events);
}

static int ltc4296_unlock(struct ltc4296_state *st)
{
	if (!st)
		return -EINVAL;

	return ltc4296_spi_write(st, LTC4296_REG_GCMD, LTC4296_UNLOCK_KEY);
}

static int ltc4296_is_locked(struct ltc4296_state *st, enum ltc4296_state_dev *state)
{
	u16 val16;
	int ret;

	if (!st || !state)
		return -EINVAL;

	ret = ltc4296_spi_read(st, LTC4296_REG_GCMD, &val16);
	if (ret)
		return ret;

	if ((val16 & LTC4296_UNLOCK_KEY) == LTC4296_UNLOCK_KEY)
		*state = LTC_UNLOCKED;
	else
		*state = LTC_LOCKED;

	return 0;
}

static int ltc4296_read_gadc(struct ltc4296_state *st, u32 *port_voltage_mv)
{
	int ret;
	u16 val16;

	if (!st || !port_voltage_mv)
		return -EINVAL;

	ret = ltc4296_spi_read(st, LTC4296_REG_GADCDAT, &val16);
	if (ret)
		return ret;
	if ((val16 & LTC4296_GADC_NEW_MSK) != LTC4296_GADC_NEW_MSK)
		return ADI_LTC_INVALID_ADC_VOLTAGE;

	/* A new ADC value is available */
	*port_voltage_mv = (((val16 & LTC4296_GADC_MSK) - LTC4296_ADC_OFFSET) *
			    LTC4296_VGAIN);

	return 0;
}

static int ltc4296_read_gadc_raw(struct ltc4296_state *st, u16 *val)
{
	int ret;

	if (!st || !val)
		return -EINVAL;

	ret = ltc4296_spi_read(st, LTC4296_REG_GADCDAT, val);
	if (ret)
		return ret;

	*val = FIELD_GET(LTC4296_GADC_MSK, *val);

	return 0;
}

static int ltc4296_set_gadc_mode(struct ltc4296_state *st,
				 u32 channel, u32 mode)
{
	int ret;

	ret = ltc4296_spi_write(st, LTC4296_REG_GADCCFG,
			 FIELD_PREP(LTC4296_GADC_SAMPLE_MODE_MSK, mode) |
			 FIELD_PREP(LTC4296_GADC_SEL_MSK, channel));
	if (ret)
		return ret;

	fsleep(4500);

	return 0;
}

static int ltc4296_set_gadc_vin(struct ltc4296_state *st)
{
	int ret;

	if (!st)
		return -EINVAL;

	/* LTC4296-1 Set global ADC to measure Vin GADCCFG = ContModeLowGain | Vin */
	ret = ltc4296_spi_write(st, LTC4296_REG_GADCCFG, 0x0041);
	if (ret)
		return ret;

	fsleep(4000); /* Delay of 4ms */

	return 0;
}

static int ltc4296_is_vin_valid(struct ltc4296_state *st, int port_vin_mv,
				enum ltc4296_board_class ltcboard_class, bool *vin_valid)
{
	if (!st || !vin_valid)
		return -EINVAL;

	if (port_vin_mv <= ltc4296_spoe_vol_range_mv[ltcboard_class][LTC4296_VMAX] &&
	    port_vin_mv >= ltc4296_spoe_vol_range_mv[ltcboard_class][LTC4296_VMIN])
		*vin_valid = true;
	else
		/* Voltage is out of range of the MIN/MAX value */
		*vin_valid = false;

	return 0;
}

static int ltc4296_is_vout_valid(struct ltc4296_state *st, int port_vout_mv,
				 enum ltc4296_board_class ltcboard_class, bool *vout_valid)
{
	if (!st || !vout_valid)
		return -EINVAL;

	if (port_vout_mv <= ltc4296_spoe_vol_range_mv[ltcboard_class][LTC4296_VMAX] &&
	    port_vout_mv >= ltc4296_spoe_vol_range_mv[ltcboard_class][LTC4296_VMIN])
		*vout_valid =  true;
	else
		/* Voltage is out of range of the MIN/MAX value */
		*vout_valid = false;

	return 0;
}

static int ltc4296_disable_gadc(struct ltc4296_state *st)
{
	int ret;

	if (!st)
		return -EINVAL;

	ret = ltc4296_spi_write(st, LTC4296_REG_GADCCFG, 0x0000);
	if (ret)
		return ret;

	fsleep(4000); /* Delay of 4ms */

	return 0;
}

static int ltc4296_read_port_events(struct ltc4296_state *st, enum ltc4296_port port_no,
				    u16 *port_events)
{
	int ret;
	u8 port_addr = 0;

	if (!st || !port_events)
		return -EINVAL;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_EVENTS, &port_addr);
	if (ret)
		return ret;

	return ltc4296_spi_read(st, port_addr, port_events);
}

static int ltc4296_clear_port_events(struct ltc4296_state *st,
				     enum ltc4296_port port_no)
{
	int ret;
	u8 port_addr = 0;

	if (!st)
		return -EINVAL;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_EVENTS, &port_addr);
	if (ret)
		return ret;

	return ltc4296_spi_write(st, port_addr, 0x3FF);
}

static int ltc4296_read_port_status(struct ltc4296_state *st, enum ltc4296_port port_no,
				    u16 *port_status)
{
	int ret;
	u8 port_addr = 0;

	if (!st || !port_status)
		return -EINVAL;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_STATUS, &port_addr);
	if (ret)
		return ret;

	return ltc4296_spi_read(st, port_addr, port_status);
}

static int ltc4296_is_port_disabled(struct ltc4296_state *st, enum ltc4296_port port_no,
				    enum ltc4296_port_status *port_chk)
{
	int ret;
	u16 port_status = 0;
	u8 port_addr = 0;

	if (!st || !port_chk)
		return -EINVAL;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_STATUS, &port_addr);
	if (ret)
		return ret;

	ret = ltc4296_spi_read(st, port_addr, &port_status);
	if (ret)
		return ret;

	if ((port_status & LTC4296_PSE_STATUS_MSK) == LTC_PSE_STATUS_DISABLED)
		*port_chk = LTC_PORT_DISABLED;
	else
		*port_chk = LTC_PORT_ENABLED;

	return 0;
}

static int ltc4296_port_disable(struct ltc4296_state *st, enum ltc4296_port port_no)
{
	int ret;
	u8 port_addr = 0;

	if (!st)
		return -EINVAL;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_CFG0, &port_addr);
	if (ret)
		return ret;
	/* Write 0 to disable port */
	return ltc4296_spi_write(st, port_addr, 0x0000);
}

static int ltc4296_is_port_deliver_pwr(struct ltc4296_state *st,
				       enum ltc4296_port port_no,
				       enum ltc4296_pse_status *pwr_status)
{
	int ret;
	u16 port_status;
	u8 port_addr = 0;

	if (!st || !pwr_status)
		return -EINVAL;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_STATUS, &port_addr);
	if (ret)
		return ret;

	ret = ltc4296_spi_read(st, port_addr, &port_status);
	if (ret)
		return ret;

	if ((port_status & LTC4296_PSE_STATUS_MSK) == LTC_PSE_STATUS_DELIVERING)
		*pwr_status = LTC_PSE_STATUS_DELIVERING;
	else
		*pwr_status = LTC_PSE_STATUS_UNKNOWN;

	return 0;
}

static int ltc4296_is_port_pwr_stable(struct ltc4296_state *st,
				      enum ltc4296_port port_no, bool *pwr_status)
{
	int ret;
	u16 port_status;
	u8 port_addr = 0;

	if (!st || !pwr_status)
		return -EINVAL;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_STATUS, &port_addr);
	if (ret)
		return ret;

	ret = ltc4296_spi_read(st, port_addr, &port_status);
	if (ret)
		return ret;

	if ((port_status & LTC4296_POWER_STABLE_MSK) == LTC4296_POWER_STABLE_MSK)
		*pwr_status =  true;
	else
		*pwr_status = false;

	return 0;
}

static int ltc4296_read_port_adc(struct ltc4296_state *st, enum ltc4296_port port_no,
				 u16 *port_voltage_mv)
{
	int ret;
	u8 port_addr = 0;

	if (!st || !port_voltage_mv)
		return -EINVAL;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_ADCDAT, &port_addr);
	if (ret)
		return ret;

	return ltc4296_spi_read(st, port_addr, port_voltage_mv);

	// if ((val16 & LTC4296_NEW_MSK) == LTC4296_NEW_MSK){
	// 	/* A new ADC value is available */
	// 	// *port_i_out_ma = (((val16 & 0x0FFF) - 2048) * 1000 /
	// 	// 		  (10 * st->port_data[port_no].hs_sense_resistor));
	// } else {
	// 	return ADI_LTC_INVALID_ADC_PORT_CURRENT;
	// }
}

static int ltc4296_port_prebias(struct ltc4296_state *st, enum ltc4296_port port_no,
				enum ltc4296_config mode)
{
	int ret;
	u8 port_addr = 0;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_CFG1, &port_addr);
	if (ret)
		return ret;

	if (mode == LTC_CFG_SCCP_MODE)
		return ltc4296_spi_write(st, port_addr, 0x0108);
	else if (mode == LTC_CFG_APL_MODE)
		return ltc4296_spi_write(st, port_addr, 0x0109);
	else
		return -EINVAL;
}

static int ltc4296_port_en(struct ltc4296_state *st, enum ltc4296_port port_no)
{
	int ret;
	u8 port_addr = 0;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_CFG0, &port_addr);
	if (ret)
		return ret;

	return ltc4296_spi_write(st, port_addr, (LTC4296_SW_EN_MSK |
				 LTC4296_SW_POWER_AVAILABLE_MSK |
				 LTC4296_SW_PSE_READY_MSK |
				 LTC4296_TMFVDO_TIMER_DISABLE_MSK));
}

static int ltc4296_port_dis(struct ltc4296_state *st, enum ltc4296_port port_no)
{
	int ret;
	u8 port_addr = 0;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_CFG0, &port_addr);
	if (ret)
		return ret;

	return ltc4296_spi_write(st, port_addr, 0);
}

static int ltc4296_port_en_and_classification(struct ltc4296_state *st,
					      enum ltc4296_port port_no)
{
	int ret;
	u8 port_addr = 0;

	if (!st)
		return -EINVAL;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_CFG0, &port_addr);
	if (ret)
		return ret;

	return ltc4296_spi_write(st, port_addr,
				 (LTC4296_SW_EN_MSK | LTC4296_SW_PSE_READY_MSK |
				 LTC4296_SET_CLASSIFICATION_MODE_MSK));
}

static int ltc4296_set_port_mfvs(struct ltc4296_state *st, enum ltc4296_port port_no)
{
	int ret;
	u8 port_addr = 0;
	u16 val16;
	int mfvs_threshold, val;

	if (!st)
		return -EINVAL;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_ADCCFG, &port_addr);
	if (ret)
		return ret;

	/* LTC4296-1 Set Port ADC MFVS Threshold Value */
	val = st->port_data[port_no].hs_sense_resistor;
	mfvs_threshold = (625 * val / 10);
	/* Round of to the nearest integer */
	val16 = DIV_ROUND_CLOSEST(mfvs_threshold, 1000);

	return ltc4296_spi_write(st, port_addr, val16);
}

static int ltc4296_set_port_pwr(struct ltc4296_state *st, enum ltc4296_port port_no)
{
	int ret;
	u8 port_addr = 0;

	if (!st)
		return -EINVAL;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_CFG0, &port_addr);
	if (ret)
		return ret;

	return ltc4296_spi_write(st, port_addr,
				 (LTC4296_SW_EN_MSK | LTC4296_SW_POWER_AVAILABLE_MSK |
				  LTC4296_SW_PSE_READY_MSK | LTC4296_SET_CLASSIFICATION_MODE_MSK |
				  LTC4296_END_CLASSIFICATION_MSK));
}

static int ltc4296_force_port_pwr(struct ltc4296_state *st, enum ltc4296_port port_no)
{
	int ret;
	u8 port_addr = 0;

	if (!st)
		return -EINVAL;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_CFG0, &port_addr);
	if (ret)
		return ret;

	return ltc4296_spi_write(st, port_addr,
				 (LTC4296_SW_EN_MSK | LTC4296_SW_POWER_AVAILABLE_MSK |
				  LTC4296_SW_PSE_READY_MSK | LTC4296_TMFVDO_TIMER_DISABLE_MSK));
}

static int ltc4296_port_pwr_available(struct ltc4296_state *st,
				      enum ltc4296_port port_no)
{
	int ret;
	u8 port_addr = 0;

	if (!st)
		return -EINVAL;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_CFG0, &port_addr);
	if (ret)
		return ret;
	/* LTC4296-1 Port Clear Classification & PSE Ready, Set Power Available */
	/* PxCFG0=en | power_available */
	return ltc4296_spi_write(st, port_addr,
				 (LTC4296_SW_EN_MSK | LTC4296_SW_POWER_AVAILABLE_MSK));
}

static int ltc4296_set_gadc_vout(struct ltc4296_state *st, enum ltc4296_port port_no)
{
	u16 val16;

	if (!st)
		return -EINVAL;

	/* LTC4296-1 Set global ADC to measure Port Vout GADCCFG=ContModeLowGain|VoutPort2 */

	val16 = (0x001F & set_port_vout[port_no]);
	/* Set Continuous mode with LOW GAIN bit */
	val16 = (val16 | 0x40);

	return ltc4296_spi_write(st, LTC4296_REG_GADCCFG, val16);
}

static int ltc4296_print_global_faults(struct ltc4296_state *st, u16 g_events)
{
	if ((g_events & LTC4296_LOW_CKT_BRK_FAULT_MSK) == LTC4296_LOW_CKT_BRK_FAULT_MSK)
		dev_info(&st->spi->dev, "LTC4296-1 low side fault, too high current or negative voltage on output\n");
	else if ((g_events & LTC4296_MEMORY_FAULT_MSK) == LTC4296_MEMORY_FAULT_MSK)
		dev_info(&st->spi->dev, "LTC4296-1 memory fault\n");
	else if ((g_events & LTC4296_PEC_FAULT_MSK) == LTC4296_PEC_FAULT_MSK)
		dev_info(&st->spi->dev, "LTC4296-1 pec fault\n");
	else if ((g_events & LTC4296_COMMAND_FAULT_MSK) == LTC4296_COMMAND_FAULT_MSK)
		dev_info(&st->spi->dev, "LTC4296-1 command fault\n");
	else if ((g_events & LTC4296_UVLO_DIGITAL_MSK) == LTC4296_UVLO_DIGITAL_MSK)
		dev_info(&st->spi->dev, "LTC4296-1  UVLO, too low input voltage fault\n");

	return 0;
}

static int ltc4296_print_port_events(struct ltc4296_state *st, enum ltc4296_port port_no,
				     u16 port_events)
{
	if ((port_events & LTC4296_LSNS_REVERSE_FAULT_MSK) == LTC4296_LSNS_REVERSE_FAULT_MSK)
		dev_info(&st->spi->dev, "LTC4296-1 port%d LSNS_REVERSE_FAULT, negative low side current\n",
			 port_no);

	if ((port_events & LTC4296_LSNS_FORWARD_FAULT_MSK) == LTC4296_LSNS_FORWARD_FAULT_MSK)
		dev_info(&st->spi->dev, "LTC4296-1 port%d LSNS_FORWARD_FAULT, too high low side current\n",
			 port_no);

	if ((port_events & LTC4296_PD_WAKEUP_MSK) == LTC4296_PD_WAKEUP_MSK)
		dev_info(&st->spi->dev, "LTC4296-1 port%d PD_WAKEUP, wake up requested by PD\n",
			 port_no);

	if ((port_events & LTC4296_TINRUSH_TIMER_DONE_MSK) == LTC4296_TINRUSH_TIMER_DONE_MSK)
		dev_info(&st->spi->dev, "LTC4296-1 port%d TINRUSH_TIMER_DONE, too long time to power on\n",
			 port_no);

	if ((port_events & LTC4296_MFVS_TIMEOUT_MSK) == LTC4296_MFVS_TIMEOUT_MSK)
		dev_info(&st->spi->dev, "LTC4296-1 port%d MFVS_TIMEOUT, PD disconnected\n",
			 port_no);

	if ((port_events & LTC4296_OVERLOAD_DETECTED_IPOWERED_MSK) == LTC4296_OVERLOAD_DETECTED_IPOWERED_MSK)
		dev_info(&st->spi->dev, "LTC4296-1 port%d OVERLOAD_DETECTED_IPOWERED, too high output current\n",
			 port_no);

	if ((port_events & LTC4296_OVERLOAD_DETECTED_ISLEEP_MSK) == LTC4296_OVERLOAD_DETECTED_ISLEEP_MSK)
		dev_info(&st->spi->dev, "LTC4296-1 port%d OVERLOAD_DETECTED_ISLEEP, un-powered port overloaded\n",
			 port_no);

	if ((port_events & LTC4296_TOFF_TIMER_DONE_MSK) == LTC4296_TOFF_TIMER_DONE_MSK)
		dev_info(&st->spi->dev, "LTC4296-1 port%d TOFF_TIMER_DONE, too long time to power off\n",
			 port_no);

	return 0;
}

static int ltc4296_chk_global_events(struct ltc4296_state *st)
{
	int ret;
	enum ltc4296_state_dev state;
	u16 global_events = 0;

	if (!st)
		return -EINVAL;

	/* Check if LTC4296-1 is locked */
	ret = ltc4296_is_locked(st, &state);
	if (ret)
		return ret;

	if (state == LTC_LOCKED) {
		/* Unlock the LTC4296_1 */
		dev_info(&st->spi->dev, "PSE initiated ...\n");
		ret = ltc4296_reset(st);
		if (ret)
			return ret;

		ret = ltc4296_unlock(st);
		if (ret)
			return ret;

		ret = ltc4296_clear_global_faults(st);
		if (ret)
			return ret;

		return ADI_LTC_DISCONTINUE_SCCP;
	} else if (state == LTC_UNLOCKED) {
		ret = ltc4296_read_global_faults(st, &global_events);
		if (ret)
			return ret;

		ret = ltc4296_print_global_faults(st, global_events);
		if (ret)
			return ret;

		if ((global_events & LTC4296_LOW_CKT_BRK_FAULT_MSK) ==
		    LTC4296_LOW_CKT_BRK_FAULT_MSK) {
			ret = ltc4296_clear_ckt_breaker(st);
			if (ret)
				return ret;

			return ADI_LTC_DISCONTINUE_SCCP;
		} else if (((global_events & LTC4296_MEMORY_FAULT_MSK) ==
			    LTC4296_MEMORY_FAULT_MSK)   ||
			   ((global_events & LTC4296_PEC_FAULT_MSK) == LTC4296_PEC_FAULT_MSK) ||
			   ((global_events & LTC4296_COMMAND_FAULT_MSK) ==
			     LTC4296_COMMAND_FAULT_MSK) ||
			   ((global_events & LTC4296_UVLO_DIGITAL_MSK) ==
			     LTC4296_UVLO_DIGITAL_MSK)) {
			/* Global events other than circuit breaker ?*/
			ret = ltc4296_reset(st);
			if (ret)
				return ret;

			ret = ltc4296_unlock(st);
			if (ret)
				return ret;

			ret = ltc4296_clear_global_faults(st);
			if (ret)
				return ret;

			return ADI_LTC_DISCONTINUE_SCCP;
		}
	}

	return 0;
}

static int ltc4296_chk_port_events(struct ltc4296_state *st, enum ltc4296_port ltc4296_port)
{
	int ret;
	u16 port_events = 0;

	if (!st)
		return -EINVAL;

	/* Read the Port Events */
	ret = ltc4296_read_port_events(st, ltc4296_port, &port_events);
	if (ret)
		return ret;

	/* Report only in case of port event other than signature */
	ret = ltc4296_print_port_events(st, ltc4296_port, port_events);
	if (ret)
		return ret;

	return ltc4296_clear_port_events(st, ltc4296_port);
}

static int ltc4296_init(struct ltc4296_state *st)
{
	int ret;
	u16 value;

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
		dev_err_probe(&st->spi->dev, -EINVAL, "Device locked. Write Access is disabled (%d)\n", value);
		return -EINVAL;
	}

	return 0;
}

static umode_t ltc4296_is_visible(const void *data, enum hwmon_sensor_types type,
				  u32 attr, int channel)
{
	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
			return 0444;
		default:
			return 0;
		}
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			return 0444;
		case hwmon_in_enable:
			return 0200;
		default:
			return 0;
		}
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
			return 0444;
		default:
			return 0;
		}
	default:
		return 0;
	}
}

static int ltc4296_write(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long val)
{
	struct ltc4296_state *st = dev_get_drvdata(dev);
	int ret;

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_enable:
			if (val == 0) {
				ret = ltc4296_port_dis(st, channel);
			} else if (val == 1) {
				ret = ltc4296_port_prebias(st, channel, LTC_CFG_APL_MODE);
				if (ret)
					return ret;

				ret = ltc4296_port_en(st, channel);
			} else {
				ret = -EINVAL;
			}
			break;
		default:
			return -EOPNOTSUPP;
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	return ret;
}

static int ltc4296_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long *val)
{
	struct ltc4296_state *st = dev_get_drvdata(dev);
	u16 adc_val;
	int ret;

	switch (type) {
	case hwmon_temp:
		switch (attr){
		case hwmon_temp_input:
			guard(mutex)(&st->lock);

			ret = ltc4296_set_gadc_mode(st, LTC4296_GADC_SEL_TEMP,
							LTC4296_GADC_SAMPLE_MODE_CONT_LG);
			if (ret)
				return ret;

			ret = ltc4296_read_gadc_raw(st, &adc_val);
			if (ret)
				return ret;

			*val = (adc_val - 2048) / 4 - 273;

			break;
		}

		break;
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			guard(mutex)(&st->lock);

			ret = ltc4296_set_gadc_mode(st, LTC4296_GADC_SEL_OUT_VOLTAGE(channel),
						    LTC4296_GADC_SAMPLE_MODE_CONT_LG);
			if (ret)
				return ret;

			ret = ltc4296_read_gadc_raw(st, &adc_val);
			if (ret)
				return ret;

			*val = (adc_val - LTC4296_ADC_OFFSET) * LTC4296_VGAIN;

			break;
		default:
			return -EOPNOTSUPP;
		}
		break;
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
			guard(mutex)(&st->lock);

			ret = ltc4296_read_port_adc(st, channel, &adc_val);

			*val = ((u32)((adc_val & 0x0FFF) - 2048) * 35200) / 1000;
			break;
		default:
			return -EOPNOTSUPP;
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	return ret;
}

static int ltc4296_sccp_res_pd(struct ltc4296_port_data *port, u16 *res_data,
			       u8 broadcast_addr, u8 read_scratchpad)
{
	uint8_t sccp_buf[3] = {0, 0, 0};
	int ret;

	ret = sccp_read_write_pd(port, broadcast_addr, read_scratchpad, sccp_buf);
	if (ret != ADI_LTC_SCCP_PD_PRESENT)
		return ret;

	*res_data = get_unaligned_le16(&sccp_buf[0]);

	return ADI_LTC_SCCP_PD_PRESENT;
}

static bool ltc4296_is_pd_compatible(u32 pse_class, u32 pd_class)
{
	if (ltc4296_class_compatibility[pse_class][pd_class])
		return true;

	return false;
}

static int ltc4296_do_sccp(struct ltc4296_state *st, uint32_t port)
{
	bool class_compatible;
	u32 pse_power_class;
	u16 raw_port_status;
	bool valid_adc_read;
	u32 pd_power_class;
	u16 pd_resp_data;
	u32 port_status;
	u32 adc_val;
	int ret;

	ret = ltc4296_is_port_deliver_pwr(st, port, &port_status);
	if (ret)
		goto err;

	ret = ltc4296_is_port_disabled(st, port, &port_status);
	if (ret)
		goto err;

	if (port_status == LTC_PORT_ENABLED){
		ret = ltc4296_is_port_deliver_pwr(st, port, &port_status);
		if (ret)
			goto err;

		if (port_status != LTC_PSE_STATUS_DELIVERING){
			ret = ltc4296_port_disable(st, port);
			if (ret)
				goto err;
		}
	} else if (port_status == LTC_PORT_DISABLED) {
		ret = ltc4296_spi_write(st, LTC4296_REG_GADCCFG, 0x0);
		if (ret){
			pr_err("Global ADC config error %d %d\n", ret, __LINE__);
			goto err;
		}

		fsleep(4000);

		ret = ltc4296_set_gadc_vin(st);
		if (ret){
			pr_err("Global ADC Vin set error %d %d\n", ret, __LINE__);
			goto err;
		}

		fsleep(4000);

		ret = ltc4296_read_gadc(st, &adc_val);
		if (ret){
			pr_err("Global ADC read error %d %d\n", ret, __LINE__);
			goto err;
		}

		pse_power_class = st->port_data[port].port_type - LTC4296_PORT_SCCP_CLASS_10;

		// pr_info("Port %d ADC read (%d) - power class %d\n", port, adc_val, pse_power_class);
		// pr_info("Range %d - %d\n", ltc4296_spoe_vol_range_mv[pse_power_class][LTC4296_VMIN],
		//        ltc4296_spoe_vol_range_mv[pse_power_class][LTC4296_VMAX]);

		ret = ltc4296_is_vin_valid(st, adc_val, pse_power_class, &valid_adc_read);
		if (ret){
			pr_err("ltc4296_is_vin_valid() error %d %d\n", ret, __LINE__);
			goto err;
		}

		if (!valid_adc_read){
			pr_err("Invalid Vin read for port %d expecting (%d - %d), has %d\n", port,
			       ltc4296_spoe_vol_range_mv[pse_power_class][LTC4296_VMIN],
			       ltc4296_spoe_vol_range_mv[pse_power_class][LTC4296_VMAX], adc_val);

			ret = -EINVAL;
			goto err;
		}

		ret = ltc4296_port_prebias(st, port, LTC_CFG_SCCP_MODE);
		if (ret){
			pr_err("ltc4296_port_prebias() error %d %d\n", ret, __LINE__);
			goto err;
		}

		ret = ltc4296_port_en_and_classification(st, port);
		if (ret){
			pr_err("ltc4296_port_en_and_classification() error %d %d\n", ret, __LINE__);
			goto err;
		}

		fsleep(4000);

		ret = ltc4296_read_port_status(st, port, &raw_port_status);
		if (ret){
			pr_err("ltc4296_read_port_status() error %d %d\n", ret, __LINE__);
			goto err;
		}

		if ((raw_port_status & LTC4296_PSE_STATUS_MSK) == LTC_PSE_STATUS_SEARCHING){
			ret = ltc4296_sccp_res_pd(&st->port_data[port], &pd_resp_data, 0xCC, 0xAA);
			if (ret == ADI_LTC_SCCP_PD_PRESENT){
				pd_power_class = FIELD_GET(GENMASK(11, 0), pd_resp_data) - 1;
				class_compatible = ltc4296_is_pd_compatible(pse_power_class, pd_power_class);

				pr_info("PD power class %d PSE power class %d\n", pd_power_class, pse_power_class);

				if (!class_compatible){
					pr_err("Incompatible power class: PSE has %d, PD has %d\n",
					       pse_power_class, pd_power_class);

					goto err;
				}

				ret = ltc4296_set_port_mfvs(st, port);
				if (ret){
					pr_err("ltc4296_set_port_mfvs() error %d %d\n", ret, __LINE__);
					goto err;
				}

				ret = ltc4296_set_port_pwr(st, port);
				if (ret){
					pr_err("ltc4296_set_port_pwr() error %d %d\n", ret, __LINE__);
					goto err;
				}

				fsleep(5000);

				ret = ltc4296_port_pwr_available(st, port);
				if (ret){
					pr_err("ltc4296_port_pwr_available() error %d %d\n", ret, __LINE__);
					goto err;
				}

				ret = ltc4296_set_gadc_vout(st, port);
				if (ret){
					pr_err("ltc4296_set_gadc_vout() error %d %d\n", ret, __LINE__);
					goto err;
				}

				pr_info("SCCP complete on port %d\n", port);
			} else {
				// pr_info("PD not present on port %d\n", port);

				ret = ltc4296_port_disable(st, port);
				if (ret)
					return ret;
			}
		}
	}

	return 0;

err:
	ltc4296_port_disable(st, port);

	return ret;
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
	HWMON_CHANNEL_INFO(temp,
			HWMON_T_INPUT | HWMON_T_LABEL),
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

static int ltc4296_parse_port_cfg(struct device *dev, struct ltc4296_state *st)
{
	struct fwnode_handle *sub_port_node;
	struct fwnode_handle *port_node;
	u32 reg;
	int ret;

	fwnode_for_each_available_child_node(dev_fwnode(dev), port_node){
		fwnode_for_each_available_child_node(port_node, sub_port_node){

			ret = fwnode_property_read_u32(sub_port_node, "reg", &reg);
			if (ret){
				pr_err("Error reading reg property\n");
				return ret;
			}

			st->port_data[reg].port_num = reg;
			st->port_data[reg].sccpi = devm_fwnode_gpiod_get(dev,
									 sub_port_node,
									 "adi,sccpi",
									 GPIOD_IN,
									 "adi,sccpi");

			if (IS_ERR(st->port_data[reg].sccpi)){
				pr_err("Error getting sccpi\n");
				return PTR_ERR(st->port_data[reg].sccpi);
			}

			st->port_data[reg].sccpo = devm_fwnode_gpiod_get(dev,
									 sub_port_node,
									 "adi,sccpo",
									 GPIOD_OUT_LOW,
									 "adi,sccpo");
			if (IS_ERR(st->port_data[reg].sccpo)){
				pr_err("Error getting sccpo\n");
				return PTR_ERR(st->port_data[reg].sccpo);
			}

			fwnode_property_read_u32(sub_port_node,
						 "adi,hs-resistor",
						 &st->port_data[reg].hs_sense_resistor);
			
			fwnode_property_read_u32(sub_port_node,
						 "adi,port-type",
						 &st->port_data[reg].port_type);
		}
	}

	return 0;
}

static int ltc4296_get_port_status(struct ltc4296_state *st, enum ltc4296_port port_no,
				   u16 *port_status)
{
	int ret;
	u8 reg_addr = 0;

	if (!st || !port_status)
		return -EINVAL;

	ret = ltc4296_get_port_addr(port_no, LTC_PORT_STATUS, &reg_addr);
	if (ret)
		return ret;

	ret = ltc4296_spi_read(st, reg_addr, port_status);
	if (ret)
		return ret;

	*port_status = FIELD_GET(LTC4296_PSE_STATUS_MSK, *port_status);

	return 0;
}

static int ltc4296_get_status(struct pse_controller_dev *pcdev, int id,
			      struct pse_pw_status *pw_status)
{
	struct ltc4296_state *st = container_of(pcdev, struct ltc4296_state, pse_dev);
	u16 port_status;
	u8 reg_addr;
	int ret;

	ltc4296_get_port_addr(id, LTC_PORT_STATUS, &reg_addr);

	ret = ltc4296_spi_read(st, reg_addr, &port_status);
	if (ret)
		return ret;

	switch (port_status & LTC4296_PSE_STATUS_MSK) {
	case 0:
		pw_status->podl_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_DISABLED;
		pw_status->c33_pw_status = ETHTOOL_C33_PSE_PW_D_STATUS_DISABLED;
		break;
	case 1:
		pw_status->podl_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_SLEEP;
		pw_status->c33_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_SLEEP;
		break;
	case 2:
		pw_status->podl_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_DELIVERING;
		pw_status->c33_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_DELIVERING;
		break;
	case 3:
		pw_status->podl_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_SEARCHING;
		pw_status->c33_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_SEARCHING;
		break;
	case 4:
		pw_status->podl_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_ERROR;
		pw_status->c33_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_ERROR;
		break;
	case 5:
		pw_status->podl_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_IDLE;
		pw_status->c33_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_IDLE;
		break;
	case 6:
		pw_status->podl_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_SEARCHING;
		pw_status->c33_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_SEARCHING;
		break;
	case 7:
		pw_status->podl_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_UNKNOWN;
		pw_status->c33_pw_status = ETHTOOL_PODL_PSE_PW_D_STATUS_UNKNOWN;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ltc4296_get_admin_state(struct pse_controller_dev *pcdev, int id,
				   struct pse_admin_state *admin_state)
{
	admin_state->podl_admin_state = ETHTOOL_PODL_PSE_ADMIN_STATE_DISABLED;
	admin_state->c33_admin_state = ETHTOOL_C33_PSE_ADMIN_STATE_DISABLED;

	return 0;
}

static int ltc4296_pi_enable(struct pse_controller_dev *pcdev, int id)
{
	struct ltc4296_state *st = container_of(pcdev, struct ltc4296_state, pse_dev);
	int ret;

	pr_info("Port %d enabled\n", id);

	// ret = ltc4296_port_prebias(st, id, LTC_CFG_APL_MODE);
	// if (ret)
	// 	return ret;

	// ret = ltc4296_port_en(st, id);
	// if (ret)
	// 	return ret;

	return 0;
}

static int ltc4296_pi_disable(struct pse_controller_dev *pcdev, int id)
{
	struct ltc4296_state *st = container_of(pcdev, struct ltc4296_state, pse_dev);
	int ret;

	pr_info("Port %d disabled\n", id);

	return 0;
}

static const struct pse_controller_ops ltc4296_pse_ops = {
	.pi_get_pw_status = ltc4296_get_status,
	.pi_get_admin_state = ltc4296_get_admin_state,
	.pi_enable = ltc4296_pi_enable,
	.pi_disable = ltc4296_pi_disable,
};

static void ltc4296_disable_ports(void *data)
{
	struct ltc4296_state *st = data;

	kthread_stop(st->pd_polling);

	for (int i = 0; i < LTC4296_MAX_PORTS; i++)
		ltc4296_port_dis(st, i);
}

static int ltc4296_pd_polling_thread(void *data)
{
	struct ltc4296_state *st = data;
	// u16 port_status;
	int ret;

	while(likely(!kthread_should_stop())){

		for (int i = 0; i < LTC4296_MAX_PORTS; i++) {
			switch(st->port_data[i].port_type){
			case LTC4296_PORT_SCCP_CLASS_10:
			case LTC4296_PORT_SCCP_CLASS_11:
			case LTC4296_PORT_SCCP_CLASS_12:
			case LTC4296_PORT_SCCP_CLASS_13:
			case LTC4296_PORT_SCCP_CLASS_14:
			case LTC4296_PORT_SCCP_CLASS_15:
				mutex_lock(&st->lock);
				ret = ltc4296_do_sccp(st, i);
				mutex_unlock(&st->lock);
				if (ret){
					pr_warn("Error doing SCCP on port %d\n", i);
					continue;
				}
			default:
				continue;
			}
		}

		schedule_timeout_interruptible(msecs_to_jiffies(3000));
	}

	return 0;
}

static int ltc4296_probe(struct spi_device *spi)
{
	enum ltc4296_port_status port_status;
	struct ltc4296_state *st;
	struct device *hwmon_dev;
	int ret;

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->spi = spi;

	ret = ltc4296_parse_port_cfg(&st->spi->dev, st);
	if (ret){
		pr_err("Error parsing port cfg\n");
		return ret;
	}

	ret = ltc4296_init(st);
	if (ret){
		pr_err("ltc4296 init return value: %d", ret);
		return ret;
	}

	ret = devm_mutex_init(&st->spi->dev, &st->lock);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "Failed to init mutex\n");

	ret = devm_add_action_or_reset(&spi->dev, ltc4296_disable_ports, st);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "Failed to add disable ports action\n");

	st->pse_dev.owner = THIS_MODULE;
	st->pse_dev.ops = &ltc4296_pse_ops;
	st->pse_dev.dev = &spi->dev;
	st->pse_dev.types = ETHTOOL_PSE_C33;
	st->pse_dev.nr_lines = LTC4296_MAX_PORTS;

	ret = devm_pse_controller_register(&spi->dev, &st->pse_dev);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "Failed to register PSE controller\n");

	for (int i = 0; i < LTC4296_MAX_PORTS; i++) {
		switch (st->port_data[i].port_type) {
		case LTC4296_PORT_DISABLED:
			continue;
		case LTC4296_PORT_APL:
			ret = ltc4296_port_prebias(st, i, LTC_CFG_APL_MODE);
			if (ret)
				return dev_err_probe(&spi->dev, ret,
						"Failed to enable APL mode prebias for port %d\n", i);

			ret = ltc4296_port_en(st, i);
			if (ret)
				return dev_err_probe(&spi->dev, ret,
						"Failed to enable APL mode for port %d\n", i);
			break;
		case LTC4296_PORT_SCCP_CLASS_10:
		case LTC4296_PORT_SCCP_CLASS_11:
		case LTC4296_PORT_SCCP_CLASS_12:
		case LTC4296_PORT_SCCP_CLASS_13:
		case LTC4296_PORT_SCCP_CLASS_14:
		case LTC4296_PORT_SCCP_CLASS_15:
			if (!st->port_data[i].sccpi || !st->port_data[i].sccpo)
				return dev_err_probe(&spi->dev, -EINVAL,
						"SCCP port %d missing SCCPI or SCCPO GPIO\n", i);
			mutex_lock(&st->lock);
			ret = ltc4296_do_sccp(st, i);
			mutex_unlock(&st->lock);
			if (ret)
				dev_warn(&spi->dev, "Error doing SCCP on port %d (%d)\n", i, ret);

			break;
		default:
			return dev_err_probe(&spi->dev, ret, "Invalid port type\n");
		}
	}

	st->pd_polling = kthread_run(ltc4296_pd_polling_thread, st, "ltc4296_pd_polling");
	if (IS_ERR(st->pd_polling))
		return dev_err_probe(&spi->dev, ret, "Failed to start thread");

	sched_set_fifo(st->pd_polling);

	st->hwmon_dev = devm_hwmon_device_register_with_info(&spi->dev, spi->dev.driver->name,
							 st, &ltc4296_chip_info,
							 NULL);
	if (IS_ERR(st->hwmon_dev))
		return dev_err_probe(&spi->dev, PTR_ERR(st->hwmon_dev),
				"Failed to register hwmon device\n");

	return 0;
}

static void ltc4296_unregister(struct spi_device *spi)
{

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
	.remove = ltc4296_unregister,
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
