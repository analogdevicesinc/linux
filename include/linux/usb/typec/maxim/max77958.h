/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * MAX77958 Core Header
 *
 * Copyright (c) 2024 Analog Devices, Inc.
 */

#ifndef __MAX77958_H__
#define __MAX77958_H__

#undef  __CONST_FFS
#define __CONST_FFS(x) ({ \
		typeof(x) (_x) = (x); \
		((_x) & 0x0F ? \
		 ((_x) & 0x03 ? ((_x) & 0x01 ? 0 : 1) : ((_x) & 0x04 ? 2 : 3)) : \
		 ((_x) & 0x30 ? ((_x) & 0x10 ? 4 : 5) : ((_x) & 0x40 ? 6 : 7))); })

#undef FFS
#define FFS(_x) ({ \
		typeof(_x) (x) = (_x); \
		((x) ? (__CONST_FFS(x)) : 0); })

#undef  BIT_RSVD
#define BIT_RSVD  0

#undef  BITS
#define BITS(end, _start) ({ \
		typeof(end) (_end) = (end); \
		((BIT(_end) - BIT(_start)) + BIT(_end)); })

#undef  __BITS_GET
#define __BITS_GET(_word, _mask, _shift) \
	(((_word) & (_mask)) >> (_shift))

#undef  BITS_GET
#define BITS_GET(_word, bit) ({ \
		typeof(bit) (_bit) = (bit); \
		__BITS_GET(_word, _bit, FFS(_bit)); })

#undef  __BITS_SET
#define __BITS_SET(_word, mask, _shift, _val) ({ \
		typeof(mask) (_mask) = (mask); \
		(((_word) & ~(_mask)) | (((_val) << (_shift)) & (_mask))); })

#undef  BITS_SET
#define BITS_SET(_word, bit, _val) ({ \
		typeof(bit) (_bit) = (bit); \
		__BITS_SET(_word, _bit, FFS(_bit), _val); })

#undef  BITS_MATCH
#define BITS_MATCH(_word, bit) ({ \
		typeof(bit) (_bit) = (bit); \
		(((_word) & (_bit)) == (_bit)); })

/*
 * Register address
 */
#define	REG_UIC_HW_REV			0x00
#define	REG_UIC_DEVICE_ID		0x01
#define	REG_UIC_FW_REV			0x02
#define	REG_UIC_FW_SUBREV		0x03
#define	REG_UIC_INT				0x04
#define	REG_CC_INT				0x05
#define	REG_PD_INT				0x06

#define	REG_ACTION_INT			0x07
#define	REG_USBC_STATUS1		0x08
#define	REG_USBC_STATUS2		0x09
#define	REG_BC_STATUS			0x0A
#define REG_DP_STATUS			0x0B
#define	REG_CC_STATUS0			0x0C
#define	REG_CC_STATUS1			0x0D

#define	REG_PD_STATUS0			0x0E
#define	REG_PD_STATUS1			0x0F

#define	REG_UIC_INT_M			0x10
#define	REG_CC_INT_M			0x11
#define	REG_PD_INT_M			0x12
#define	REG_ACTION_INT_M		0x13
#define	REG_VBUS_VOLTAGE_L		0x14
#define	REG_VBUS_VOLTAGE_H		0x15

#define REG_OPCODE				0x21
#define REG_OPCODE_DATA			0x22
#define REG_END_DATA			0x41
#define REG_OPCDE_RES			0x51

#define REG_RESET				0x80

/*
 * REG_UIC_INT Interrupts
 */
#define BIT_APCmdResI			BIT(7)
#define BIT_SYSMsgI				BIT(6)
#define BIT_VBUSDetI			BIT(5)
#define BIT_VbADCI				BIT(4)
#define BIT_DCDTmoI				BIT(3)
#define BIT_FakeVBUSI			BIT(2)
#define BIT_CHGTypI				BIT(1)
#define BIT_Action4I			BIT(0)

/*
 * REG_CC_INT Interrupts
 */
#define BIT_VCONNOCPI			BIT(7)
#define BIT_VSAFE0VI			BIT(6)
#define BIT_AttachSrcErrI		BIT(5)
#define BIT_WTRI				BIT(4)
#define BIT_CCPinStatI			BIT(3)
#define BIT_CCIStatI			BIT(2)
#define BIT_CCVcnStatI			BIT(1)
#define BIT_CCStatI				BIT(0)

/*
 * REG_PD_INT Interrupts
 */
#define BIT_PDMsgI				BIT(7)
#define BIT_PSRdyI				BIT(6)
#define BIT_DataRoleI			BIT(5)
#define BIT_DisplayPortI		BIT(2)

/*
 * REG_ACTION_INT Interrupts
 */
#define BIT_Action_EnterI		BIT(3)
#define BIT_Action2I			BIT(2)
#define BIT_Action1I			BIT(1)
#define BIT_Action0I			BIT(0)

/*
 * REG_USBC_STATUS1
 */
#define BIT_VBADC				BITS(7, 3)
#define BIT_FakeVbus			BIT(3)

/*
 * REG_USBC_STATUS2
 */
#define BIT_SYSMsg				BITS(7, 0)

/*
 * REG_BC_STATUS
 */
#define BIT_VBUSDet				BIT(7)
#define BIT_PrChgTyp			BITS(5, 3)
#define BIT_DCDTmo				BIT(2)
#define BIT_ChgTyp				BITS(1, 0)

/*
 * REG_DP_STATUS
 */
#define BIT_DP_ExitMode			BIT(7)
#define BIT_DP_Attention		BIT(6)
#define BIT_DP_Configure		BIT(5)
#define BIT_DP_Status			BIT(4)
#define BIT_DP_EnterMode		BIT(3)
#define BIT_DP_DiscoverMode		BIT(2)
#define BIT_DP_DiscoverSVID		BIT(1)
#define BIT_DP_DiscoverIdentity BIT(0)

/*
 * REG_CC_STATUS0
 */
#define BIT_CCPinStat			BITS(7, 6)
#define BIT_CCIStat				BITS(5, 4)
#define BIT_CCVcnStat			BIT(3)
#define BIT_CCStat				BITS(2, 0)

/*
 * REG_CC_STATUS1
 */
#define BIT_VCONNOCP			BIT(5)
#define BIT_VCONNSC				BIT(4)
#define BIT_VSAFE0V				BIT(3)
#define BIT_AttachSrcErr		BIT(2)
#define BIT_Wtr					BIT(1)

/*
 * REG_PD_STATUS0
 */
#define BIT_PDMsg				BITS(7, 0)

/*
 * REG_PD_STATUS1
 */
#define BIT_DataRole			BIT(7)
#define BIT_PowerRole			BIT(6)
#define BIT_VCONNS				BIT(5)
#define BIT_PSRDY				BIT(4)
#define BIT_FCT_ID				BITS(3, 0)

/*
 * F/W update
 */

struct max77958_fw_header {
	u32	magic;	/* magic number */
	u8	major;	/* major version */
	u8	minor;	/* minor version */
	u8	id;		/* id */
	u8	rev;	/* rev */
};

#define FW_CMD_READ			0x3
#define FW_CMD_READ_SIZE	6	/* cmd(1) + len(1) + data(4) */

#define FW_CMD_WRITE		0x1
#define FW_CMD_WRITE_SIZE	36	/* cmd(1) + len(1) + data(34) */

#define FW_CMD_END			0x0

#define FW_HEADER_SIZE		8
#define FW_VERIFY_DATA_SIZE 3

#define FW_VERIFY_TRY_COUNT 10

#define FW_WAIT_TIMEOUT			(1000 * 5) /* 5 sec */
#define I2C_SMBUS_BLOCK_HALF	(I2C_SMBUS_BLOCK_MAX / 2)

enum {
	MAX77958_DEVREV_UNKNOWN = 0xFF,
	MAX77958_DEVREV_PASS2 = 0x00,
	MAX77958_DEVREV_PASS3 = 0x01,
};
#define MAX77958_DEVREV_STR(devrev)	((devrev == MAX77958_DEVREV_PASS3) ? "PASS3" : (devrev == MAX77958_DEVREV_PASS2) ? "PASS2" : "UNKNOWN")

enum {
	FW_UPDATE_START = 0x00,
	FW_UPDATE_WAIT_RESP_START,
	FW_UPDATE_WAIT_RESP_STOP,
	FW_UPDATE_DOING,
	FW_UPDATE_END,
};

enum {
	FW_UPDATE_FAIL = 0xF0,
	FW_UPDATE_I2C_FAIL,
	FW_UPDATE_TIMEOUT_FAIL,
	FW_UPDATE_VERIFY_FAIL,
	FW_UPDATE_CMD_FAIL,
	FW_UPDATE_MAX_LENGTH_FAIL,
};

enum {
	NO_DETERMINATION = 0,
	CC1_ACTIVE,
	CC2_ACTVIE,
};

/*
 * All type of Interrupts
 */

enum max77958_ccistat_type {
	NOT_IN_UFP_MODE,
	CCI_500mA,
	CCI_1_5A,
	CCI_3_0A,
};

enum max77958_chg_type {
	CHGTYP_NOTHING = 0,
	CHGTYP_USB_SDP,
	CHGTYP_CDP,
	CHGTYP_DCP,
};

enum max77958_pr_chg_type {
	UNKNOWN = 0,
	SAMSUNG_2A,
	APPLE_05A,
	APPLE_1A,
	APPLE_2A,
	APPLE_12W,
	DCP_3A,
	RFU_CHG,
};

enum max77958_ccpd_device {
	DEV_NONE = 0,
	DEV_OTG,

	DEV_USB,
	DEV_CDP,
	DEV_DCP,

	DEV_APPLE500MA,
	DEV_APPLE1A,
	DEV_APPLE2A,
	DEV_APPLE12W,
	DEV_DCP3A,
	DEV_HVDCP,
	DEV_QC,

	DEV_FCT_0,
	DEV_FCT_1K,
	DEV_FCT_255K,
	DEV_FCT_301K,
	DEV_FCT_523K,
	DEV_FCT_619K,
	DEV_FCT_OPEN,

	DEV_PD_TA,
	DEV_PD_AMA,

	DEV_UNKNOWN,
};

enum max77958_fctid {
	FCT_GND = 1,
	FCT_1KOHM,
	FCT_255KOHM,
	FCT_301KOHM,
	FCT_523KOHM,
	FCT_619KOHM,
	FCT_OPEN,
};

enum max77958_cc_pin_state {
	cc_No_Connection = 0,
	cc_SINK,
	cc_SOURCE,
	cc_Audio_Accessory,
	cc_Debug_Accessory,
	cc_Error,
	cc_Disabled,
	cc_Debug_Sink,
};

enum max77958_usbc_SYSMsg {
	SYSMSG_NONE							= 0x00,
	SYSMSG_BOOT_WDT						= 0x03,
	SYSMSG_BOOT_SWRSTREQ				= 0x04,
	SYSMSG_BOOT_POR						= 0x05,
	SYSMSG_AFC_DONE						= 0x20,
	SYSMSG_APCMD_UNKNOWN				= 0x31,
	SYSMSG_APCMD_INPROGRESS				= 0x32,
	SYSMSG_APCMD_FAIL					= 0x33,
	SYSMSG_CCx_5V_SHORT					= 0x61,
	SYSMSG_SBUx_GND_SHORT				= 0x62,
	SYSMSG_SBUx_5V_SHORT				= 0x63,

	SYSMSG_UNKNOWN_RID_TWICE			= 0x70,
	SYSMSG_USER_PD_COLLISION			= 0x80,
	SYSMSG_MTP_WRITEFAIL				= 0xC0,
	SYSMSG_MTP_READFAIL					= 0xC1,
	SYSMSG_MTP_ERASEFAIL				= 0xC2,
	SYSMSG_MTP_CUSTMINFONOTSET			= 0xC3,
	SYSMSG_MTP_OVERACTIONBLKSIZE		= 0xC4,
	SYSMSG_FIRMWAREERROR				= 0xE0,
	SYSMSG_ACTION_UNKNOWN				= 0xE1,
	SYSMSG_ACTION_OFFSET				= 0xE2,
	SYSMSG_ACTION_UNKNOWN_CMD			= 0xE3,
	SYSMSG_EXECUTE_I2C_WRITEBURST_FAIL	= 0xEA,
	SYSMSG_EXECUTE_I2C_READBURST_FAIL	= 0xEB,
	SYSMSG_EXECUTE_I2C_READ_FAIL		= 0xEC,
	SYSMSG_EXECUTE_I2C_WRITE_FAIL		= 0xED,
};

enum max77958_pdmsg {
	PDMSG_NOTHING_HAPPENED                      = 0x00,
	PDMSG_SNK_PSRDY_RECEIVED                    = 0x01,
	PDMSG_SNK_ERROR_RECOVERY                    = 0x02,
	PDMSG_SNK_SENDERRESPONSETIMER_TIMEOUT       = 0x03,
	PDMSG_SRC_PSRDY_SENT                        = 0x04,
	PDMSG_SRC_ERROR_RECOVERY                    = 0x05,
	PDMSG_SRC_SENDERRESPONSETIMER_TIMEOUT       = 0x06,
	PDMSG_DR_SWAP_REQ_RECEIVED                  = 0x07,
	PDMSG_PR_SWAP_REQ_RECEIVED                  = 0x08,
	PDMSG_VCONN_SWAP_REQ_RECEIVED               = 0x09,
	PDMSG_VDM_ATTENTION_MSG_RECEIVED            = 0x11,
	PDMSG_REJECT_RECEIVED                       = 0x12,
	PDMSG_PRSWAP_SNKTOSRC_SENT					= 0x14,
	PDMSG_PRSWAP_SRCTOSNK_SENT					= 0x15,
	PDMSG_HARDRESET_RECEIVED					= 0x16,
	PDMSG_HARDRESET_SENT						= 0x19,
	PDMSG_PRSWAP_SRCTOSWAP						= 0x1A,
	PDMSG_PRSWAP_SWAPTOSNK						= 0x1B,
	PDMSG_PRSWAP_SNKTOSWAP						= 0x1C,
	PDMSG_PRSWAP_SWAPTOSRC						= 0x1D,
	PDMSG_SNK_DISABLED                          = 0x20,
	PDMSG_SRC_DISABLED                          = 0x21,
	PDMSG_VDM_NAK_RECEIVED                      = 0x40,
	PDMSG_VDM_BUSY_RECEIVED                     = 0x41,
	PDMSG_VDM_ACK_RECEIVED                      = 0x42,
	PDMSG_VDM_REQ_RECEIVED                      = 0x43,
	PDMSG_VDM_DISCOVERMODE_RECEIVED             = 0x63,
	PDMSG_VDM_DP_STATUS_RECEIVED                = 0x65,
};

enum max77958_connstat {
	DRY = 0x00,
	WATER = 0x01,
};

/*
 * External type definition
 */
#define OPCODE_WAIT_TIMEOUT (3000) /* 3000ms */

#define OPCODE_WRITE_COMMAND	0x21
#define OPCODE_READ_COMMAND		0x51
#define OPCODE_SIZE				1
#define OPCODE_HEADER_SIZE      1
#define OPCODE_DATA_LENGTH		32
#define OPCODE_MAX_LENGTH		(OPCODE_DATA_LENGTH + OPCODE_SIZE)
#define OPCODE_WRITE			0x21
#define OPCODE_DATA				0x22
#define OPCODE_WRITE_END		0x41
#define OPCODE_READ				0x51
#define OPCODE_WRITE_SEQ		0x1
#define OPCODE_READ_SEQ			0x2
#define OPCODE_RW_SEQ			0x3
#define OPCODE_PUSH_SEQ			0x4
#define OPCODE_UPDATE_SEQ		0x5

enum {
	OPCODE_BC_CTRL1_R			 = 0x01,
	OPCODE_BC_CTRL1_W			 = 0x02,
	OPCODE_BC_CTRL2_R			 = 0x03,
	OPCODE_BC_CTRL2_W			 = 0x04,
	OPCODE_CONTROL1_R			 = 0x05,
	OPCODE_CONTROL1_W			 = 0x06,
	OPCODE_CCCONTROL1_R			 = 0x0B,
	OPCODE_CCCONTROL1_W			 = 0x0C,
	OPCODE_CCCONTROL2_R			 = 0x0D,
	OPCODE_CCCONTROL2_W			 = 0x0E,
	OPCODE_CCCONTROL3_R			 = 0x0F,
	OPCODE_CCCONTROL3_W			 = 0x10,
	OPCODE_CCCONTROL4_R			 = 0x11,
	OPCODE_CCCONTROL4_W			 = 0x12,
	OPCODE_VCONN_ILIM_R			 = 0x13,
	OPCODE_VCONN_ILIM_W			 = 0x14,
	OPCODE_CCCONTROL5_R			 = 0x15,
	OPCODE_CCCONTROL5_W			 = 0x16,
	OPCODE_CCCONTROL6_R			 = 0x17,
	OPCODE_CCCONTROL6_W			 = 0x18,
	OPCODE_GET_SINK_CAP			 = 0x2F,
	OPCODE_CURRENT_SRC_CAP		 = 0x30,
	OPCODE_GET_SOURCE_CAP		 = 0x31,
	OPCODE_SRCCAP_REQ			 = 0x32,
	OPCODE_SET_SOURCE_CAP		 = 0x33,
	OPCODE_SEND_GET_REQ			 = 0x34,
	OPCODE_READ_GET_REQ_RESP	 = 0x35,
	OPCODE_SEND_GET_RESP		 = 0x36,
	OPCODE_SWAP_REQ				 = 0x37,
	OPCODE_SWAP_RESP			 = 0x38,
	OPCODE_VDM_DISCOVER_ID_RESP	 = 0x40,
	OPCODE_VDM_DISCOVER_ID_REQ	 = 0x41,
	OPCODE_VDM_DISCOVER_SVID_RESP = 0x42,
	OPCODE_VDM_DISCOVER_SVID_REQ  = 0x43,
	OPCODE_VDM_DISCOVER_MODE_RESP = 0x44,
	OPCODE_VDM_DISCOVER_MODE_REQ  = 0x45,
	OPCODE_VDM_ENTER_MODE_REQ	 = 0x46,
	OPCODE_VDM_EXIT_MODE_REQ	= 0x47,
	OPCODE_SEND_VDM				 = 0x48,
	OPCODE_GET_PD_MSG			 = 0x4A,
	OPCODE_GET_VDM_RESP			 = 0x4B,
	OPCODE_SET_CSTM_INFORMATION_R  = 0x55, /* Customer VIF Information */
	OPCODE_SET_CSTM_INFORMATION_W  = 0x56, /* Customer VIF Information */
	/* Device Config Information */
	OPCODE_SET_DEVCONFG_INFORMATION_R  = 0x57,
	/* Device Config Information */
	OPCODE_SET_DEVCONFG_INFORMATION_W  = 0x58,
	OPCODE_ACTION_BLOCK_MTP_UPDATE  = 0x60, /* Action Block update */
	OPCODE_FW_OPCODE_CLEAR		= 0x70,
	OPCODE_MASTER_I2C_READ  = 0x85, /* I2C_READ */
	OPCODE_MASTER_I2C_WRITE  = 0x86, /* I2C_WRITE */
	OPCODE_NONE = 0xFF,
};

/*
 * REG_INT_M Initial values
 */
#define REG_UIC_INT_M_INIT		0x05
#define REG_CC_INT_M_INIT		0x20
#define REG_PD_INT_M_INIT		0x00
#define REG_ACTION_INT_M_INIT   0xFF

#define OPCODE_CMD_REQUEST	0x1

#define MFD_DEV_NAME "max77958"

/* MAX77958 shared i2c API function */
int max77958_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest);
int max77958_write_reg(struct i2c_client *i2c, u8 reg, u8 value);
int max77958_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask);
int max77958_i2c_bulk(const char *caller, u8 type, struct i2c_client *i2c, u8 reg, int count, u8 *buf);

enum i2c_bulk_type_e {
	MAX77958_I2C_BULK_READ,
	MAX77958_I2C_BULK_WRITE,
	MAX77958_I2C_BULK_VERIFY,
	MAX77958_I2C_OPCODE_READ,
	MAX77958_I2C_OPCODE_WRITE,
	MAX77958_I2C_BULK_MAX,
};
#define max77958_bulk_read(i2c, reg, count, buf)	max77958_i2c_bulk(__func__, MAX77958_I2C_BULK_READ, i2c, reg, count, buf)
#define max77958_bulk_write(i2c, reg, count, buf)	max77958_i2c_bulk(__func__, MAX77958_I2C_BULK_WRITE, i2c, reg, count, buf)
#define max77958_opcode_read(i2c, count, buf)		max77958_i2c_bulk(__func__, MAX77958_I2C_OPCODE_READ, i2c, OPCODE_READ, count, buf)
#define max77958_opcode_write(i2c, count, buf)		max77958_i2c_bulk(__func__, MAX77958_I2C_OPCODE_WRITE, i2c, OPCODE_WRITE, count, buf)

#endif /* __MAX77958_H__ */

