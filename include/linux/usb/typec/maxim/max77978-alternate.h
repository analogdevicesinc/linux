/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * MAX77978 DisplayPort Alternate Mode Header
 *
 * Copyright (c) 2024 Analog Devices, Inc.
 */

#ifndef __MAX77978_ALTERNATE_H__
#define __MAX77978_ALTERNATE_H__

typedef union {
	uint8_t			DATA;
	struct {
		uint8_t 	BDATA[1];
	} BYTES;
	struct {
		uint8_t		Num_Of_VDO:3,
					Cmd_Type:2,
					Reserved:3;
	} BITS;
} SEND_VDM_BYTE_DATA;

typedef union {
	uint16_t		DATA;
	struct {
		uint8_t		BDATA[2];
	} BYTES;
	struct {
		uint16_t	MSG_Type:4,
					Reserved1:1,
					Port_Data_Role:1,
					Specification_Rev:2,
					Port_Power_Role:1,
					MSG_ID:3,
					Number_of_obj:3,
					reserved2:1;
	} BITS;
} UND_DATA_MSG_HEADER_Type;

typedef union {
	uint32_t		DATA;
	struct {
		uint8_t		BDATA[4];
	} BYTES;
	struct {
		uint32_t	VDM_command:5,
					Rsvd2_VDM_header:1,
					VDM_command_type:2,
					Object_Position:3,
					Rsvd_VDM_header:2,
					Structured_VDM_Version:2,
					VDM_Type:1,
					Standard_Vendor_ID:16;
	} BITS;
} UND_DATA_MSG_VDM_HEADER_Type;

typedef union {
	uint32_t		DATA;
	struct {
		uint8_t		BDATA[4];
	} BYTES;
	struct {
		uint32_t	USB_Vendor_ID:16,
					Rsvd_ID_header:10,
					Modal_Operation_Supported:1,
					Product_Type:3,
					Data_Capable_USB_Device:1,
					Data_Capable_USB_Host:1;
	} BITS;
} UND_DATA_MSG_ID_HEADER_Type;

typedef union {
	uint32_t		DATA;
	struct {
		uint8_t		BDATA[4];
	} BYTES;
	struct {
		uint32_t	Cert_TID:20,
					Rsvd_cert_VDOer:12;
	} BITS;
} UND_CERT_STAT_VDO_Type;

typedef union {
	uint32_t		DATA;
	struct {
		uint8_t		BDATA[4];
	} BYTES;
	struct {
		uint32_t	Device_Version:16,
					Product_ID:16;
	} BITS;
} UND_PRODUCT_VDO_Type;

typedef union {
	uint32_t		DATA;
	struct {
		uint8_t		BDATA[4];
	} BYTES;
	struct {
		uint32_t	USB_Superspeed_Signaling_Support:3,
					SOP_contoller_present:1,
					Vbus_through_cable:1,
					Vbus_Current_Handling_Capability:2,
					SSRX2_Directionality_Support:1,
					SSRX1_Directionality_Support:1,
					SSTX2_Directionality_Support:1,
					SSTX1_Directionality_Support:1,
					Cable_Termination_Type:2,
					Cable_Latency:4,
					TypeC_to_Plug_Receptacle:1,
					TypeC_to_ABC:2,
					Rsvd_CABLE_VDO:4,
					Cable_Firmware_Version:4,
					Cable_HW_Version:4;
	} BITS;
} UND_CABLE_VDO_Type;

typedef union {
	uint32_t		DATA;
	struct {
		uint8_t		BDATA[4];
	} BYTES;
	struct {
		uint32_t	SVID_1:16,
					SVID_0:16;
	} BITS;
} UND_VDO1_Type;

typedef union {
	uint32_t	DATA;
	struct {
		uint8_t	BDATA[4];
	} BYTES;
	struct {
		uint32_t	VENDOR_DEFINED_MESSAGE:15,
					VDM_TYPE:1,
					USB_Vendor_ID:16;
	} BITS;
} UND_UNSTRUCTURED_VDM_HEADER_Type;

typedef union {
	uint32_t		DATA;
	struct {
		uint8_t		BDATA[4];
	} BYTES;
	struct {
		uint32_t	DATA:8,
					TOTAL_NUMBER_OF_UVDM_SET:4,
					RESERVED:1,
					COMMAND_TYPE:2,
					DATA_TYPE:1,
					PID:16;
	} BITS;
} UND_SEC_UNSTRUCTURED_VDM_HEADER_Type;

typedef union {
	uint32_t	DATA;
	struct {
		uint8_t	BDATA[4];
	} BYTES;
	struct {
		uint32_t	VENDOR_DEFINED_MESSAGE:15,
					VDM_TYPE:1,
					USB_Vendor_ID:16;
	} BITS;
} UND_SEC_DATA_HEADER_Type;

typedef union {
	uint32_t	DATA;
	struct {
		uint8_t	BDATA[4];
	} BYTES;
	struct {
		uint32_t	VENDOR_DEFINED_MESSAGE:15,
					VDM_TYPE:1,
					USB_Vendor_ID:16;
	} BITS;
} UND_SEC_DATA_TAILER_Type;

typedef union {
	uint32_t		DATA;
	struct {
		uint8_t		BDATA[4];
	} BYTES;
	struct {
		uint32_t	ORDER_OF_CURRENT_UVDM_SET:4,
					RESERVED:9,
					COMMAND_TYPE:2,
					DATA_TYPE:1,
					PID:16;
	} BITS;
} UND_SEC_UNSTRUCTURED_VDM_RESPONSE_HEADER_Type;

typedef union {
	uint32_t		DATA;
	struct {
		uint8_t		BDATA[4];
	} BYTES;
	struct {
		uint32_t	VENDOR_DEFINED_MESSAGE:15,
					VDM_TYPE:1,
					USB_Vendor_ID:16;
	} BITS;
} UND_SEC_DATA_RESPONSE_HEADER_Type;

typedef struct {
	uint32_t	VDO[7];
} VDO_MESSAGE_Type;

/* For DP */
#define TypeC_POWER_SINK_INPUT		0
#define TypeC_POWER_SOURCE_OUTPUT	1
#define TypeC_DP_SUPPORT			(0xFF01)

/* For DP VDM Modes VDO Port_Capability */
typedef enum {
	num_Reserved_Capable		= 0,
	num_UFP_D_Capable			= 1,
	num_DFP_D_Capable			= 2,
	num_DFP_D_and_UFP_D_Capable	= 3
} Num_DP_Port_Capability_Type;

/* For DP VDM Modes VDO Receptacle_Indication */
typedef enum {
	num_USB_TYPE_C_PLUG			= 0,
	num_USB_TYPE_C_Receptacle	= 1
} Num_DP_Receptacle_Indication_Type;


/* For DP_Status_Update Port_Connected */
typedef enum {
	num_Adaptor_Disable			= 0,
	num_Connect_DFP_D			= 1,
	num_Connect_UFP_D			= 2,
	num_Connect_DFP_D_and_UFP_D	= 3
} Num_DP_Port_Connected_Type;

/* For DP_Configure Select_Configuration */
typedef enum {
	num_Cfg_for_USB				= 0,
	num_Cfg_UFP_U_as_DFP_D		= 1,
	num_Cfg_UFP_U_as_UFP_D		= 2,
	num_Cfg_Reserved			= 3
} Num_DP_Sel_Configuration_Type;

typedef enum { /* There is another Macro definitions which are similiar to this */
	REQ			= 0,
	ACK			= 1,
	NAK 		= 2,
	BUSY 		= 3
} VDM_CMD_TYPE;

typedef enum {
	Reserved			= 0,
	Discover_Identity	= 1,
	Discover_SVIDs 		= 2,
	Discover_Modes	 	= 3,
	Enter_Mode 			= 4,
	Exit_Mode			= 5,
	Attention			= 6,
	Configure			= 17
} VDM_HEADER_COMMAND;

typedef enum {
	Version_1_0			= 0,
	Version_2_0			= 1,
	Reserved1			= 2,
	Reserved2			= 3
} STRUCTURED_VDM_VERSION;
typedef union {
	uint32_t		DATA;
	struct {
		uint8_t		BDATA[4];
	} BYTES;
	struct {
		uint32_t	Port_Capability:2,
					Signalling_DP:4,
					Receptacle_Indication:1,
					USB_2p0_Not_Used:1,
					DFP_D_Pin_Assignments:8,
					UFP_D_Pin_Assignments:8,
					DP_MODE_VDO_Reserved:8;
	} BITS;
} UND_VDO_MODE_DP_CAPABILITY_Type;

typedef union {
	uint32_t		DATA;
	struct {
		uint8_t		BDATA[4];
	} BYTES;
	struct {
		uint32_t	Port_Connected:2,
					Power_Low:1,
					Enabled:1,
					Multi_Function_Preference:1,
					USB_Configuration_Req:1,
					Exit_DP_Mode_Req:1,
					HPD_State:1,
					HPD_Interrupt:1,
					Reserved:23;
	} BITS;
} UND_VDO_DP_STATUS_UPDATES_Type;

typedef union {
	uint32_t		DATA;
	struct {
		uint8_t		BDATA[4];
	} BYTES;
	struct {
		uint32_t	SEL_Configuration:2,
					Select_DP_V1p3:1,
					Select_USB_Gen2:1,
					Select_Reserved_1:2,
					Select_Reserved_2:2,
					DFP_D_PIN_Assign_A:1,
					DFP_D_PIN_Assign_B:1,
					DFP_D_PIN_Assign_C:1,
					DFP_D_PIN_Assign_D:1,
					DFP_D_PIN_Assign_E:1,
					DFP_D_PIN_Assign_F:1,
					DFP_D_PIN_Reserved:2,
					UFP_D_PIN_Assign_A:1,
					UFP_D_PIN_Assign_B:1,
					UFP_D_PIN_Assign_C:1,
					UFP_D_PIN_Assign_D:1,
					UFP_D_PIN_Assign_E:1,
					UFP_D_PIN_Assign_F:1,
					UFP_D_PIN_Reserved:2,
					DP_MODE_Reserved:8;
	} BITS;
} UND_DP_CONFIG_UPDATE_Type;

#pragma pack(push, 1)
typedef struct {
	UND_DATA_MSG_HEADER_Type			MSG_HEADER;
	UND_DATA_MSG_VDM_HEADER_Type		DATA_MSG_VDM_HEADER;
	UND_VDO_MODE_DP_CAPABILITY_Type		DATA_MSG_MODE_VDO_DP;
} DIS_MODE_DP_CAPA_Type;

typedef struct {
	UND_DATA_MSG_HEADER_Type			MSG_HEADER;
	UND_DATA_MSG_VDM_HEADER_Type		DATA_MSG_VDM_HEADER;
	UND_VDO_DP_STATUS_UPDATES_Type		DATA_DP_STATUS_UPDATE;
} DP_STATUS_UPDATE_Type;

typedef struct {
	UND_DATA_MSG_HEADER_Type			MSG_HEADER;
	UND_DATA_MSG_VDM_HEADER_Type		DATA_MSG_VDM_HEADER;
	UND_VDO_DP_STATUS_UPDATES_Type		DATA_MSG_DP_STATUS;
} DIS_ATTENTION_MESSAGE_DP_STATUS_Type;
#pragma pack(pop)

enum VDM_MSG_IRQ_State {
	VDM_DISCOVER_ID			=	(1 << 0),
	VDM_DISCOVER_SVIDS		=	(1 << 1),
	VDM_DISCOVER_MODES		=	(1 << 2),
	VDM_ENTER_MODE			=	(1 << 3),
	VDM_DP_STATUS_UPDATE	=	(1 << 4),
	VDM_DP_CONFIGURE		=	(1 << 5),
	VDM_ATTENTION			=	(1 << 6),
	VDM_EXIT_MODE			=	(1 << 7),
};

#define ALTERNATE_MODE_NOT_READY	BIT(0)
#define ALTERNATE_MODE_READY		BIT(1)
#define ALTERNATE_MODE_STOP			BIT(2)
#define ALTERNATE_MODE_START		BIT(3)
#define ALTERNATE_MODE_RESET		BIT(4)

#define MODE_INT_CLEAR			0x01
#define PD_NEXT_STATE			0x02
#define MODE_INTERFACE			0x03
#define SVID_SELECT				0x07
#define REQ_PR_SWAP				0x10
#define REQ_DR_SWAP				0x11
#define SEL_SSM_MSG_REQ			0x20
#define DP_ALT_MODE_REQ			0x30
#define SEC_UVDM_SHORT_DATA		0x0
#define SEC_UVDM_LONG_DATA		0x1
#define SEC_UVDM_ININIATOR		0x0
#define SEC_UVDM_RESPONDER_ACK	0x1
#define SEC_UVDM_RESPONDER_NAK	0x2
#define SEC_UVDM_RESPONDER_BUSY	0x3
#define UNSTRUCTURED_VDM		0x0
#define STRUCTURED_VDM			1

/*For DP Pin Assignment */
#define DP_PIN_ASSIGNMENT_NODE	0x00000000
#define DP_PIN_ASSIGNMENT_A		0x00000001	/* ( 1 << 0 ) */
#define DP_PIN_ASSIGNMENT_B		0x00000002	/* ( 1 << 1 ) */
#define DP_PIN_ASSIGNMENT_C		0x00000004	/* ( 1 << 2 ) */
#define DP_PIN_ASSIGNMENT_D		0x00000008	/* ( 1 << 3 ) */
#define DP_PIN_ASSIGNMENT_E		0x00000010	/* ( 1 << 4 ) */
#define DP_PIN_ASSIGNMENT_F		0x00000020	/* ( 1 << 5 ) */

#define DISCOVER_IDENTITY_RESPONSE_SIZE		31
#define DISCOVER_SVIDS_RESPONSE_SIZE		31
#define DISCOVER_MODES_RESPONSE_SIZE		11
#define ENTER_MODE_RESPONSE_SIZE			7
#define ATTENTION_RESPONSE_SIZE				11
#define DP_STATUS_RESPONSE_SIZE				11
#define DP_CONFIGURE_RESPONSE_SIZE			11

#define CCIC_NOTIFY_LOW     0x0
#define CCIC_NOTIFY_HIGH    0x1
#define CCIC_NOTIFY_IRQ     0x2

typedef union sec_uvdm_header {
	uint32_t		data;
	struct {
		uint8_t		bdata[4];
	} BYTES;
	struct {
		uint32_t	data:8,
					total_number_of_uvdm_set:4,
					direction:1,
					command_type:2,
					data_type:1,
					pid:16;
	} BITS;
} U_SEC_UVDM_HEADER;

typedef U_SEC_UVDM_HEADER U_SEC_UVDM_RESPONSE_HEADER;

typedef union sec_tx_data_header {
	uint32_t		data;
	struct {
		uint8_t		bdata[4];
	} BYTES;
	struct {
		uint32_t	data_size_of_current_set:8,
					total_data_size:8,
					reserved:12,
					order_of_current_uvdm_set:4;
	} BITS;
} U_SEC_TX_DATA_HEADER;

typedef union sec_data_tx_tailer {
	uint32_t		data;
	struct {
		uint8_t		bdata[4];
	} BYTES;
	struct {
		uint32_t	checksum:16,
					reserved:16;
	} BITS;
} U_SEC_TX_DATA_TAILER;

#define SEC_UVDM_UNSTRUCTURED_VDM	0x0
#define SEC_UVDM_LONGPACKET_WAIT_MS (6000)
#define SEC_UVDM_RX_HEADER_ACK		0x0
#define SEC_UVDM_RX_HEADER_NAK		0x1

#define MAXIM_ENABLE_ALTERNATE_SRCCAP	0x1
#define MAXIM_ENABLE_ALTERNATE_VDM		0x2
#define MAXIM_ENABLE_ALTERNATE_SRC_VDM	0x3
#define MAXIM_DISABLE_ALTERNATE_SRC_VDM	0x0

typedef union {
	uint32_t DATA;
	uint8_t  BYTES[4];
	struct {
		uint32_t	Vdm_Flag_Reserve_b0:1,		/* b0 */
					Vdm_Flag_Discover_ID:1,		/* b1 */
					Vdm_Flag_Discover_SVIDs:1,	/* b2 */
					Vdm_Flag_Discover_MODEs:1,	/* b3 */
					Vdm_Flag_Enter_Mode:1,		/* b4 */
					Vdm_Flag_Exit_Mode:1,		/* b5 */
					Vdm_Flag_Attention:1,		/* b6 */
					Vdm_Flag_Reserved:9,		/* b7 - b15 */
					Vdm_Flag_DP_Status_Update:1,	/* b16 */
					Vdm_Flag_DP_Configure:1,	/* b17 */
					Vdm_Flag_Reserved2:14;		/* b18 - b31 */
	} BITS;
} MAX77978_VDM_MSG_IRQ_STATUS_Type;

struct DP_DP_DISCOVER_IDENTITY { /* struct type definition */
	SEND_VDM_BYTE_DATA byte_data;
	UND_DATA_MSG_VDM_HEADER_Type vdm_header;

} __packed;

struct DP_DP_DISCOVER_ENTER_MODE { /* struct type definition */
	SEND_VDM_BYTE_DATA byte_data;
	UND_DATA_MSG_VDM_HEADER_Type vdm_header;

} __packed;

struct DP_DP_STATUS { /* struct type definition */
	SEND_VDM_BYTE_DATA byte_data;
	UND_DATA_MSG_VDM_HEADER_Type vdm_header;
	UND_VDO_DP_STATUS_UPDATES_Type vdo_status;
} __packed;

struct DP_DP_CONFIGURE { /* struct type definition */
	SEND_VDM_BYTE_DATA byte_data;
	UND_DATA_MSG_VDM_HEADER_Type vdm_header;
	UND_DP_CONFIG_UPDATE_Type vdo_config;
} __packed;

void max77978_receive_alternate_message(struct max77978_usbc_platform_data *data);
void max77978_vdm_message_handler(struct max77978_usbc_platform_data *usbpd_data, char *opcode_data, int len);
void max77978_set_discover_identity(void *data);
void max77978_set_dp_configure(void *data, uint8_t w_data);

#endif /* __MAX77978_ALTERNATE_H__ */
