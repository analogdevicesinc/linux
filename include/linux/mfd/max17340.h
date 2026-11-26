/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Analog Devices MAX17340 MFD driver header
 *
 * Copyright (C) 2024-2026 Analog Devices, Inc.
 */

#ifndef __LINUX_MFD_MAX17340_H
#define __LINUX_MFD_MAX17340_H

#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>

/* I2C Addresses */
#define MAX17340_ADDR_FG_RAM		0x36
#define MAX17340_ADDR_FG_NVM		0x0B
#define MAX17340_ADDR_FUNC		0x37

/* ========== FUNC Registers (8-bit, address 0x37) ========== */

/* GPIO registers */
#define MAX17340_GPIO_DATA		0x90
#define MAX17340_GPIO_DBNC_SEL		0x91
#define MAX17340_GPIO_MODE		0x92
#define MAX17340_GPIO_STS		0x93

/* Write protection register (FUNC address 0x37) */
#define MAX17340_WRITE_PROT		0xA0

/* WRITE_PROT bits (temporary unlock, nLOCKCtrl has priority) */
#define MAX17340_WP_NV			BIT(0)	/* NV Shadowed (0x1AX) */
#define MAX17340_WP_NVM_PROT		BIT(1)	/* Protection NVM (0x14X) */
#define MAX17340_WP_NVM_CHG		BIT(2)	/* Charger NVM (0x12X, 0x13X) */
#define MAX17340_WP_NVM_GAUGE		BIT(3)	/* Gauge NVM */
#define MAX17340_WP_USR_DATA		BIT(4)	/* User Memory (0x1EX-0x1FX) */
#define MAX17340_WP_FUNC_REGS		BIT(5)	/* FUNC registers */
#define MAX17340_WP_FG_RAM		BIT(6)	/* FG RAM (0x01X-0x0FX) */
#define MAX17340_WP_GLOBAL		BIT(7)	/* Global write protect */

/* Mailbox registers (FUNC address 0x37) */
/* AP → RISCV command registers (AP writes here) */
#define MAX17340_MBOX_AP_REQ_LEN	0x20	/* AP_DATAOUT_M1: Command length byte */
#define MAX17340_MBOX_AP_REQ_OPCODE	0x21	/* AP_DATAOUT0: Command opcode */
#define MAX17340_MBOX_AP_REQ_DATA	0x22	/* AP_DATAOUT1~32: Command data */

/* RISCV → AP response registers (AP reads here) */
#define MAX17340_MBOX_RISCV_RESP_LEN	0x50	/* AP_DATAIN_M1: Response length byte */
#define MAX17340_MBOX_RISCV_RESP_OPCODE	0x51	/* AP_DATAIN0: Response result */
#define MAX17340_MBOX_RISCV_RESP_DATA	0x52	/* AP_DATAIN1~32: Response data */

/* RISCV response opcode values */
#define MAX17340_RESP_SUCCESS		0x01	/* Command completed successfully */
#define MAX17340_RESP_NOT_IMPLEMENTED	0x02	/* Command not implemented */
#define MAX17340_RESP_FAILURE		0x03	/* Command failed */
#define MAX17340_RESP_ECDSA_SUCCESS	0xAA	/* ECDSA command completed successfully */

/* Mailbox interrupt registers */
#define MAX17340_MBOX_INT_0		0x04	/* AP_INT_01-07 */
#define MAX17340_MBOX_INT_1		0x05	/* AP_INT_08-15 */
#define MAX17340_MBOX_INT_2		0x06	/* AP_INT_16-23 */
#define MAX17340_MBOX_INT_3		0x07	/* AP_INT_24-31 */
#define MAX17340_MBOX_INT_4		0x08	/* AP_INT_32-39 (BATTVIMON) */
#define MAX17340_MBOX_INT_M_0		0x0F	/* Mask for INT_0 */
#define MAX17340_MBOX_INT_M_1		0x10	/* Mask for INT_1 */
#define MAX17340_MBOX_INT_M_2		0x11	/* Mask for INT_2 */
#define MAX17340_MBOX_INT_M_3		0x12	/* Mask for INT_3 */
#define MAX17340_MBOX_INT_M_4		0x13	/* Mask for INT_4 */

/* Device info registers */
#define MAX17340_MBOX_DEVICE_ID		0x00
#define MAX17340_MBOX_DEVICE_REV	0x01
#define MAX17340_MBOX_FW_REV		0x02
#define MAX17340_MBOX_FW_SUB_REV	0x03

/* MBOX_INT_0 bits (AP_INT_01-07, bit 0 reserved) */
#define MAX17340_MBOX_INT0_CMD_DONE	BIT(1)	/* AP_INT_01: Generic command done */
#define MAX17340_MBOX_INT0_ECDSA	BIT(2)	/* AP_INT_02: ECDSA done */
#define MAX17340_MBOX_INT0_RSVD_03	BIT(3)	/* AP_INT_03: Reserved */
#define MAX17340_MBOX_INT0_PONR		BIT(4)	/* AP_INT_04: Power-on reset */
#define MAX17340_MBOX_INT0_IMN		BIT(5)	/* AP_INT_05: Current min */
#define MAX17340_MBOX_INT0_BST		BIT(6)	/* AP_INT_06: Battery status */
#define MAX17340_MBOX_INT0_RSVD_07	BIT(7)	/* AP_INT_07: Reserved */

/* MBOX_INT_1 bits (AP_INT_08-15) */
#define MAX17340_MBOX_INT1_THMHOT	BIT(0)	/* AP_INT_08: Thermistor hot */
#define MAX17340_MBOX_INT1_IMX		BIT(1)	/* AP_INT_09: Current max */
#define MAX17340_MBOX_INT1_DSOCI	BIT(2)	/* AP_INT_10: dSOCi */
#define MAX17340_MBOX_INT1_VMN		BIT(3)	/* AP_INT_11: Voltage min */
#define MAX17340_MBOX_INT1_TMN		BIT(4)	/* AP_INT_12: Temp min */
#define MAX17340_MBOX_INT1_SMN		BIT(5)	/* AP_INT_13: SOC min */
#define MAX17340_MBOX_INT1_RSVD_14	BIT(6)	/* AP_INT_14: Reserved */
#define MAX17340_MBOX_INT1_VMX		BIT(7)	/* AP_INT_15: Voltage max */

/* MBOX_INT_2 bits (AP_INT_16-23) */
#define MAX17340_MBOX_INT2_TMX		BIT(0)	/* AP_INT_16: Temp max */
#define MAX17340_MBOX_INT2_SMX		BIT(1)	/* AP_INT_17: SOC max */
#define MAX17340_MBOX_INT2_RSVD_18	BIT(2)	/* AP_INT_18: Reserved */
#define MAX17340_MBOX_INT2_STARTUP	BIT(3)	/* AP_INT_19: Startup set status */
#define MAX17340_MBOX_INT2_ODCP		BIT(4)	/* AP_INT_20: Over-discharge current */
#define MAX17340_MBOX_INT2_UVP		BIT(5)	/* AP_INT_21: Under-voltage protection */
#define MAX17340_MBOX_INT2_TOOHOT_D	BIT(6)	/* AP_INT_22: Too hot discharge */
#define MAX17340_MBOX_INT2_DIEHOT	BIT(7)	/* AP_INT_23: Die overtemp */

/* MBOX_INT_3 bits (AP_INT_24-31) */
#define MAX17340_MBOX_INT3_TEMPREGCHG	BIT(0)	/* AP_INT_24: Temp region change */
#define MAX17340_MBOX_INT3_QOVERFLOW	BIT(1)	/* AP_INT_25: Charge overflow */
#define MAX17340_MBOX_INT3_OVP		BIT(2)	/* AP_INT_26: Over-voltage protection */
#define MAX17340_MBOX_INT3_CHGWDT	BIT(3)	/* AP_INT_27: Charger watchdog */
#define MAX17340_MBOX_INT3_OCCP		BIT(4)	/* AP_INT_28: Over-charge current */
#define MAX17340_MBOX_INT3_TOOHOT_C	BIT(5)	/* AP_INT_29: Too hot charge */
#define MAX17340_MBOX_INT3_TOOCOLD_D	BIT(6)	/* AP_INT_30: Too cold discharge */
#define MAX17340_MBOX_INT3_TOOCOLD_C	BIT(7)	/* AP_INT_31: Too cold charge */

/* MBOX_INT_4 bits (AP_INT_32-39, BATTVIMON) */
#define MAX17340_MBOX_INT4_CP		BIT(0)	/* AP_INT_32: Charge pump */
#define MAX17340_MBOX_INT4_CT		BIT(1)	/* AP_INT_33: Charge termination */
#define MAX17340_MBOX_INT4_DROPOUT	BIT(2)	/* AP_INT_34: Dropout */
#define MAX17340_MBOX_INT4_FULL		BIT(3)	/* AP_INT_35: Full */
#define MAX17340_MBOX_INT4_BVIM_TRIG	BIT(4)	/* AP_INT_36: BATTVIMON triggered */
#define MAX17340_MBOX_INT4_BVIM_STOP	BIT(5)	/* AP_INT_37: BATTVIMON stopped */
#define MAX17340_MBOX_INT4_BVIM_RDY	BIT(6)	/* AP_INT_38: Samples ready */
#define MAX17340_MBOX_INT4_RSVD_39	BIT(7)	/* AP_INT_39: Reserved */

/* Mailbox command opcodes - System */
#define MAX17340_MBOX_CMD_TRIMMING_TEST		0x34
#define MAX17340_MBOX_CMD_HW_REG_INIT		0x40
#define MAX17340_MBOX_CMD_FORCE_LOW_POWER	0x41
#define MAX17340_MBOX_CMD_RISCV_REG_READ	0x4E
#define MAX17340_MBOX_CMD_RISCV_REG_WRITE	0x4F
#define MAX17340_MBOX_CMD_TE_LITE_CLK_EN	0x5B
#define MAX17340_MBOX_CMD_FET_OVRD_EN		0x5E
#define MAX17340_MBOX_CMD_FET_OVRD_DIS		0x5F

/* Mailbox command opcodes - FuelGauge */
#define MAX17340_MBOX_CMD_SOC_HOLD_DIS		0x61
#define MAX17340_MBOX_CMD_WAKE_FROM_SHIP	0x62
#define MAX17340_MBOX_CMD_SYSTEM_REBOOT		0x63
#define MAX17340_MBOX_CMD_GET_FG_STATUS		0x64
#define MAX17340_MBOX_CMD_CLEAR_FG_STATUS	0x65
#define MAX17340_MBOX_CMD_SET_FG_MASK		0x66
#define MAX17340_MBOX_CMD_GET_FG_MASK		0x67

/* Mailbox command opcodes - NVM */
#define MAX17340_MBOX_CMD_OTP_SLP_BURN		0x70
#define MAX17340_MBOX_CMD_NV_RELOAD		0x71
#define MAX17340_MBOX_CMD_NV_COPY_PAGE		0x7F
#define MAX17340_MBOX_CMD_NV_COPY_BLOCK		0x80
#define MAX17340_MBOX_CMD_NV_COPY_USER_MEM	0x81
#define MAX17340_MBOX_CMD_LOCK			0x8F
#define MAX17340_MBOX_CMD_RECALL_FTP_WRITES	0x90
#define MAX17340_MBOX_CMD_GET_PAGE_WR_CNT	0x91
#define MAX17340_MBOX_CMD_WRITE_DATA		0x96
#define MAX17340_MBOX_CMD_RECALL_LIFE_LOG	0x9F

/* Mailbox command opcodes - ECDSA/Secure (uses AP_INT_02) */
#define MAX17340_MBOX_CMD_READ_DATA		0x44
#define MAX17340_MBOX_CMD_RAM_CODE_EXEC		0xA0
#define MAX17340_MBOX_CMD_COMPUTE_SIGNATURE	0xA5
#define MAX17340_MBOX_CMD_GEN_DEVICE_KEY_PAIR	0xCB
#define MAX17340_MBOX_CMD_SET_RMA_STATE		0xCF

/* Mailbox command opcodes - BATTVIMON */
#define MAX17340_MBOX_CMD_BATTVIMON_ENABLE	0xB0
#define MAX17340_MBOX_CMD_BATTVIMON_DISABLE	0xB1
#define MAX17340_MBOX_CMD_BATTVIMON_READ_SAMPLES 0xB2
#define MAX17340_MBOX_CMD_BATTVIMON_READ_LATEST	0xB3
#define MAX17340_MBOX_CMD_BATTVIMON_READ_OLDEST	0xB4

/* Check if opcode uses ECDSA completion (AP_INT_02) instead of CMD_DONE (AP_INT_01) */
#define MAX17340_MBOX_CMD_IS_ECDSA(op) ({		\
	typeof(op) __op = (op);				\
	__op == MAX17340_MBOX_CMD_READ_DATA ||		\
	__op == MAX17340_MBOX_CMD_WRITE_DATA ||		\
	__op == MAX17340_MBOX_CMD_COMPUTE_SIGNATURE ||	\
	__op == MAX17340_MBOX_CMD_GEN_DEVICE_KEY_PAIR ||\
	__op == MAX17340_MBOX_CMD_SET_RMA_STATE; })

/* RAM_CODE_EXEC (0xA0) has no completion interrupt (INT = 0x00) */
#define MAX17340_MBOX_CMD_NO_INT(op) ({			\
	typeof(op) __op = (op);				\
	__op == MAX17340_MBOX_CMD_RAM_CODE_EXEC; })

/* GPIO_DATA register bits */
#define MAX17340_GPIO_DATA_1		BIT(0)	/* AGPIO1 */
#define MAX17340_GPIO_DATA_2		BIT(1)	/* AGPIO2 */
#define MAX17340_GPIO_DATA_3		BIT(2)	/* AGPIO3 */
#define MAX17340_GPIO_DATA_4		BIT(3)	/* AGPIO4 */

/* GPIO_MODE register - 2 bits per GPIO */
#define MAX17340_GPIO_MODE_MASK		0x03
#define MAX17340_GPIO_MODE_DEDICATED	0x00	/* TH/PB/ALRT dedicated function */
#define MAX17340_GPIO_MODE_INPUT	0x01	/* GPIO input */
#define MAX17340_GPIO_MODE_PUSHPULL	0x02	/* GPIO push-pull output */
#define MAX17340_GPIO_MODE_OPENDRAIN	0x03	/* GPIO open-drain output */

/* Number of GPIOs */
#define MAX17340_NUM_GPIOS		4

/* ========== FG_RAM Registers (16-bit, address 0x36) ========== */

/* Status and Configuration */
#define MAX17340_STATUS			0x00
#define MAX17340_CONFIG			0x0B
#define MAX17340_PROTSTATUS		0x2A
#define MAX17340_STATUS2		0x2C
#define MAX17340_FSTAT			0x3D
#define MAX17340_CONFIG2		0xAB
#define MAX17340_PROTALRT		0xAF

/* Alert Thresholds */
#define MAX17340_VALRTTH		0x01
#define MAX17340_TALRTTH		0x02
#define MAX17340_SALRTTH		0x03
#define MAX17340_IALRTTH		0x04

/* Fuel Gauge */
#define MAX17340_REPCAP			0x05
#define MAX17340_REPSOC			0x06
#define MAX17340_AGE			0x07
#define MAX17340_MAXMINVOLT		0x08
#define MAX17340_MAXMINTEMP		0x09
#define MAX17340_MAXMINCURR		0x0A
#define MAX17340_FULLCAPREP		0x10
#define MAX17340_TTE			0x11
#define MAX17340_QRTABLE00		0x12
#define MAX17340_AVGTA			0x16
#define MAX17340_CYCLES			0x17
#define MAX17340_DESIGNCAP		0x18
#define MAX17340_AVGVCELL		0x19
#define MAX17340_VCELL			0x1A
#define MAX17340_TEMP			0x1B
#define MAX17340_CURRENT		0x1C
#define MAX17340_AVGCURRENT		0x1D
#define MAX17340_AVCAP			0x1F
#define MAX17340_TTF			0x20
#define MAX17340_DEVNAME		0x21
#define MAX17340_FULLCAPNOM		0x23
#define MAX17340_MIXCAP			0x2B
#define MAX17340_DIETEMP		0x34
#define MAX17340_FULLCAP		0x35
#define MAX17340_REPSOH			0x70

/* Charger Runtime */
#define MAX17340_ICHGTERM		0x1E
#define MAX17340_CHARGINGCURRENT	0x24
#define MAX17340_CHARGINGVOLTAGE	0x25
#define MAX17340_CHGSTAT		0x3C

/* BATTVIMON Registers (FG_RAM) */
#define MAX17340_BVIM_CFG		0xE0
#define MAX17340_SMPL_MATH		0xE1
#define MAX17340_BVIM_TRIG		0xE2
#define MAX17340_BVIMTR			0xE3
#define MAX17340_BVIM_VTR_M_TH		0xE4
#define MAX17340_BVIM_ITR_M_TH		0xE5
#define MAX17340_BVIM_VTR_SMIN_TH	0xE6
#define MAX17340_BVIM_VTR_SMAX_TH	0xE7
#define MAX17340_BVIM_VTR_SAVG_TH	0xE8
#define MAX17340_BVIM_ITR_SMIN_TH	0xE9
#define MAX17340_BVIM_ITR_SMAX_TH	0xEA
#define MAX17340_BVIM_ITR_SAVG_TH	0xEB
#define MAX17340_BVIM_STS		0xEC
#define MAX17340_BVIMRS			0xED
#define MAX17340_BVIM_RLAP		0xEE
#define MAX17340_BVIM_RFAP		0xEF

/* ========== FG_NVM Registers (16-bit, address 0x0B) ========== */

/* Protection Thresholds */
#define MAX17340_NVM_UVPRTTH		0x40
#define MAX17340_NVM_OVPRTTH		0x41
#define MAX17340_NVM_TPRTTH1		0x42
#define MAX17340_NVM_TPRTTH2		0x43
#define MAX17340_NVM_IPRTTH1		0x44
#define MAX17340_NVM_PROTCFG		0x4E

/* Charge Configuration */
#define MAX17340_NVM_CHGCFG0		0x33
#define MAX17340_NVM_CHGCFG1		0x34
#define MAX17340_NVM_VCHGCFG1		0x36
#define MAX17340_NVM_ICHGCFG1		0x38
#define MAX17340_NVM_STEPVOLT		0x3A
#define MAX17340_NVM_STEPCURR		0x3B
#define MAX17340_NVM_ICHGTERM		0x9C

/* FG Model Configuration */
#define MAX17340_NVM_MODELCFG		0x9D
#define MAX17340_NVM_VEMPTY		0x9E
#define MAX17340_NVM_CONFIG		0xB0
#define MAX17340_NVM_DESIGNCAP		0xB3
#define MAX17340_NVM_DESIGNVOLTAGE	0xB4
#define MAX17340_NVM_RELAXCFG		0xB5
#define MAX17340_NVM_FILTERCFG		0xB9
#define MAX17340_NVM_PACKCFG		0xC1
#define MAX17340_NVM_RSENSE		0xCC
#define MAX17340_NVM_FULLCAPNOM		0xD5
#define MAX17340_NVM_RCOMP0		0xD6
#define MAX17340_NVM_TEMPCO		0xD7

/* ========== STATUS register bits ========== */
#define MAX17340_STATUS_PONR		BIT(1)
#define MAX17340_STATUS_BST		BIT(3)
#define MAX17340_STATUS_ALLOWCHGB	BIT(5)
#define MAX17340_STATUS_DSOCI		BIT(7)
#define MAX17340_STATUS_VMN		BIT(8)
#define MAX17340_STATUS_TMN		BIT(9)
#define MAX17340_STATUS_SMN		BIT(10)
#define MAX17340_STATUS_CA		BIT(11)
#define MAX17340_STATUS_VMX		BIT(12)
#define MAX17340_STATUS_TMX		BIT(13)
#define MAX17340_STATUS_SMX		BIT(14)
#define MAX17340_STATUS_PA		BIT(15)

/* ========== CHGSTAT register bits ========== */
#define MAX17340_CHGSTAT_CV		BIT(0)
#define MAX17340_CHGSTAT_CC		BIT(1)
#define MAX17340_CHGSTAT_CT		BIT(2)
#define MAX17340_CHGSTAT_CP		BIT(3)
#define MAX17340_CHGSTAT_STEPID_MASK	GENMASK(6, 4)
#define MAX17340_CHGSTAT_PREQ		BIT(7)
#define MAX17340_CHGSTAT_TEMPID_MASK	GENMASK(10, 8)
#define MAX17340_CHGSTAT_DROPOUT	BIT(15)

/* ========== PROTSTATUS register bits ========== */
#define MAX17340_PROT_SHDN		BIT(0)
#define MAX17340_PROT_BLOCKDIS		BIT(1)
#define MAX17340_PROT_ODCP		BIT(2)
#define MAX17340_PROT_UVP		BIT(3)
#define MAX17340_PROT_TOOHOTD		BIT(4)
#define MAX17340_PROT_DIEHOT		BIT(5)
#define MAX17340_PROT_BLOCKCHG		BIT(7)
#define MAX17340_PROT_PREQF		BIT(8)
#define MAX17340_PROT_QOVFLW		BIT(9)
#define MAX17340_PROT_OCCP		BIT(10)
#define MAX17340_PROT_OVP		BIT(11)
#define MAX17340_PROT_TOOCOLDC		BIT(12)
#define MAX17340_PROT_FULL		BIT(13)
#define MAX17340_PROT_TOOHOTC		BIT(14)
#define MAX17340_PROT_CHGWDT		BIT(15)

/* ========== Other register bits ========== */
#define MAX17340_CONFIG_AEN		BIT(2)
#define MAX17340_CONFIG_ETHRM		BIT(4)
#define MAX17340_CONFIG_TEN		BIT(9)
#define MAX17340_CONFIG_SS		BIT(10)

#define MAX17340_CONFIG2_POR_CMD	BIT(14)
#define MAX17340_CONFIG2_LDMDL		BIT(15)

#define MAX17340_FSTAT_DNR		BIT(0)
#define MAX17340_FSTAT_LDMDL		BIT(1)
#define MAX17340_FSTAT_FQ		BIT(7)

#define MAX17340_PROTALRT_TEMPREGCHG	BIT(6)

/* STATUS alert mask - bits that trigger power_supply_changed() */
#define MAX17340_STATUS_ALERT_MASK	(MAX17340_STATUS_PA | \
					 MAX17340_STATUS_VMX | \
					 MAX17340_STATUS_VMN | \
					 MAX17340_STATUS_TMX | \
					 MAX17340_STATUS_TMN | \
					 MAX17340_STATUS_SMX | \
					 MAX17340_STATUS_SMN | \
					 MAX17340_STATUS_CA)

/* ========== BVIM_CFG register bits ========== */
#define MAX17340_BVIM_CFG_SMPL_N_MASK	GENMASK(7, 0)
#define MAX17340_BVIM_CFG_CNT_RUN	BIT(10)
#define MAX17340_BVIM_CFG_UVLO_STOP	BIT(11)
#define MAX17340_BVIM_CFG_OVP_STOP	BIT(12)

/* ========== BVIM_TRIG register bits ========== */
#define MAX17340_BVIM_TRIG_NOW		BIT(0)
#define MAX17340_BVIM_TRIG_UVLO		BIT(1)
#define MAX17340_BVIM_TRIG_OVP		BIT(2)
#define MAX17340_BVIM_TRIG_VBATT	BIT(3)
#define MAX17340_BVIM_TRIG_IBATT	BIT(4)

/* ========== BVIM_STS register bits ========== */
#define MAX17340_BVIM_STS_OSC_MASK	GENMASK(8, 0)
#define MAX17340_BVIM_STS_TR_STS_MASK	GENMASK(10, 9)
#define MAX17340_BVIM_STS_ENABLED	BIT(11)
#define MAX17340_BVIM_STS_STOPPED	BIT(12)
#define MAX17340_BVIM_STS_STOP_REASON_MASK GENMASK(15, 13)

/* BVIM trigger complete status */
#define MAX17340_BVIM_STS_TRIGGER	BIT(9)

/* ========== Unit conversion constants ========== */
#define MAX17340_VOLTAGE_LSB_UV		78125	/* 78.125 uV/LSB */
#define MAX17340_CURRENT_LSB_NV		1562500	/* 1.5625 uV/LSB in nV */
#define MAX17340_CAPACITY_LSB_NVH	5000000	/* 5.0 uVh/LSB in nVh */
#define MAX17340_TEMP_LSB_DECIDEGC	10	/* For deci-degrees conversion */
#define MAX17340_TIME_LSB_MS		5625	/* 5.625 sec in ms */

#define MAX17340_DEFAULT_RSENSE_UOHM	10000	/* 10 mOhm default */

/* ========== Timing constants ========== */
#define MAX17340_POR_DELAY_MS		500
#define MAX17340_MODEL_DELAY_MS		350

/**
 * struct max17340_dev - MAX17340 MFD device structure
 * @dev:		Parent device pointer
 * @regmap:		Regmap for FG_RAM (address 0x36, 16-bit)
 * @regmap_nvm:		Regmap for FG_NVM (address 0x0B, 16-bit)
 * @regmap_func:	Regmap for FUNC (address 0x37, 8-bit)
 * @rsense:		Sense resistor value in micro-ohms
 * @irq:		Alert IRQ number
 * @battery:		Battery power supply (for IRQ notification)
 * @charger:		Charger power supply (for IRQ notification)
 * @mailbox_done:	Mailbox command completion
 * @mailbox_lock:	Serializes mailbox transactions and response reads
 * @pending_opcode:	Last issued mailbox opcode (selects the response length)
 */
struct max17340_dev {
	struct device *dev;
	struct regmap *regmap;
	struct regmap *regmap_nvm;
	struct regmap *regmap_func;
	u32 rsense;
	int irq;

	/* Power supply references for IRQ notification */
	struct power_supply *battery;
	struct power_supply *charger;

#if IS_ENABLED(CONFIG_MFD_MAX17340_MAILBOX)
	/* Mailbox support */
	struct completion mailbox_done;
	struct mutex mailbox_lock;	/* serializes mailbox transactions */
	u8 pending_opcode;
#endif
};

/* Write protection control */
int max17340_unlock_write(struct max17340_dev *mfd);
int max17340_lock_write(struct max17340_dev *mfd);

#if IS_ENABLED(CONFIG_MFD_MAX17340_MAILBOX)
int max17340_mailbox_cmd(struct max17340_dev *mfd, u8 opcode,
			 const u8 *args, int num_args,
			 u8 *resp, int num_resp);
int max17340_mailbox_read_fw_version(struct max17340_dev *mfd,
				     u8 *major, u8 *minor);
#endif

/* Unit conversion helper functions */
static inline int max17340_raw_to_uv(unsigned int raw)
{
	return (raw * MAX17340_VOLTAGE_LSB_UV) / 1000;
}

static inline int max17340_raw_to_ua(u32 rsense, s16 raw)
{
	s64 val = (s64)raw * MAX17340_CURRENT_LSB_NV;

	return div_s64(val, rsense);
}

static inline int max17340_raw_to_uah(u32 rsense, unsigned int raw)
{
	u64 val = (u64)raw * MAX17340_CAPACITY_LSB_NVH;

	return div_u64(val, rsense);
}

static inline int max17340_raw_to_decidegc(s16 raw)
{
	/* Units of LSB = 1/256 degree Celsius, convert to deci-degrees */
	return (int)raw * MAX17340_TEMP_LSB_DECIDEGC / 256;
}

static inline int max17340_raw_to_sec(unsigned int raw)
{
	return (raw * MAX17340_TIME_LSB_MS) / 1000;
}

#endif /* __LINUX_MFD_MAX17340_H */
