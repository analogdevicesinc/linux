// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAX77978 DisplayPort Alternate Mode Driver
 *
 * Copyright (c) 2024 Analog Devices, Inc.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/usb/typec/maxim/max77978-private.h>
#include <linux/completion.h>
#include <linux/usb/typec/maxim/max77978-usbc.h>
#include <linux/usb/typec/maxim/max77978-alternate.h>

static const struct DP_DP_DISCOVER_IDENTITY DP_DISCOVER_IDENTITY = {
	{
		.BITS.Num_Of_VDO = 1,
		.BITS.Cmd_Type = ACK,
		.BITS.Reserved = 0
	},

	{
		.BITS.VDM_command = Discover_Identity,
		.BITS.Rsvd2_VDM_header = 0,
		.BITS.VDM_command_type = REQ,
		.BITS.Object_Position = 0,
		.BITS.Rsvd_VDM_header = 0,
		.BITS.Structured_VDM_Version = Version_1_0,
		.BITS.VDM_Type = STRUCTURED_VDM,
		.BITS.Standard_Vendor_ID = 0xFF00
	}

};
static const struct DP_DP_DISCOVER_ENTER_MODE DP_DISCOVER_ENTER_MODE = {
	{
		.BITS.Num_Of_VDO = 1,
		.BITS.Cmd_Type = ACK,
		.BITS.Reserved = 0
	},
	{
		.BITS.VDM_command = Enter_Mode,
		.BITS.Rsvd2_VDM_header = 0,
		.BITS.VDM_command_type = REQ,
		.BITS.Object_Position = 1,
		.BITS.Rsvd_VDM_header = 0,
		.BITS.Structured_VDM_Version = Version_1_0,
		.BITS.VDM_Type = STRUCTURED_VDM,
		.BITS.Standard_Vendor_ID = 0xFF01
	}
};

static struct DP_DP_CONFIGURE DP_CONFIGURE = {
	{
		.BITS.Num_Of_VDO = 2,
		.BITS.Cmd_Type = ACK,
		.BITS.Reserved = 0
	},
	{
		.BITS.VDM_command = 17, /* SVID Specific Command */
		.BITS.Rsvd2_VDM_header = 0,
		.BITS.VDM_command_type = REQ,
		.BITS.Object_Position = 1,
		.BITS.Rsvd_VDM_header = 0,
		.BITS.Structured_VDM_Version = Version_1_0,
		.BITS.VDM_Type = STRUCTURED_VDM,
		.BITS.Standard_Vendor_ID = 0xFF01
	},
	{
		.BITS.SEL_Configuration = num_Cfg_UFP_U_as_UFP_D,
		.BITS.Select_DP_V1p3 = 1,
		.BITS.Select_USB_Gen2 = 0,
		.BITS.Select_Reserved_1 = 0,
		.BITS.Select_Reserved_2 = 0,
		.BITS.DFP_D_PIN_Assign_A = 0,
		.BITS.DFP_D_PIN_Assign_B = 0,
		.BITS.DFP_D_PIN_Assign_C = 0,
		.BITS.DFP_D_PIN_Assign_D = 1,
		.BITS.DFP_D_PIN_Assign_E = 0,
		.BITS.DFP_D_PIN_Assign_F = 0,
		.BITS.DFP_D_PIN_Reserved = 0,
		.BITS.UFP_D_PIN_Assign_A = 0,
		.BITS.UFP_D_PIN_Assign_B = 0,
		.BITS.UFP_D_PIN_Assign_C = 0,
		.BITS.UFP_D_PIN_Assign_D = 0,
		.BITS.UFP_D_PIN_Assign_E = 0,
		.BITS.UFP_D_PIN_Assign_F = 0,
		.BITS.UFP_D_PIN_Reserved = 0,
		.BITS.DP_MODE_Reserved = 0
	}
};

static char vdm_msg_irq_state_print[9][40] = {
	{"bFLAG_Vdm_Reserve_b0"},
	{"bFLAG_Vdm_Discover_ID"},
	{"bFLAG_Vdm_Discover_SVIDs"},
	{"bFLAG_Vdm_Discover_MODEs"},
	{"bFLAG_Vdm_Enter_Mode"},
	{"bFLAG_Vdm_DP_Status_Update"},
	{"bFLAG_Vdm_DP_Configure"},
	{"bFLAG_Vdm_Attention"},
	{"bFLAG_Vdm_Exit_Mode"},
};
static char dp_pin_assignment_print[7][40] = {
	{"DP_Pin_Assignment_None"},
	{"DP_Pin_Assignment_A"},
	{"DP_Pin_Assignment_B"},
	{"DP_Pin_Assignment_C"},
	{"DP_Pin_Assignment_D"},
	{"DP_Pin_Assignment_E"},
	{"DP_Pin_Assignment_F"},
};

static uint8_t dp_pin_assignment_data[7] = {
	DP_PIN_ASSIGNMENT_NODE,
	DP_PIN_ASSIGNMENT_A,
	DP_PIN_ASSIGNMENT_B,
	DP_PIN_ASSIGNMENT_C,
	DP_PIN_ASSIGNMENT_D,
	DP_PIN_ASSIGNMENT_E,
	DP_PIN_ASSIGNMENT_F,
};

#define DEBUG_VDM_PRINT
static void max77978_vdm_process_printf(char *type, char *vdm_data, int len)
{
#ifdef DEBUG_VDM_PRINT
	print_hex_dump(KERN_INFO, type, DUMP_PREFIX_NONE, 8, 1, (const void *)&vdm_data[2], len-2, false);
#endif
}

struct vdm_info {
	void *buf;
	int len;
	int vdo0_num;
	uint8_t w_data;
};

#define null2term(cond)	\
	do {									\
		if (!(cond)) {						\
			msg_err("%s is null", #cond);	\
			return;							\
		}									\
	} while (0)

#define null2return(cond, retval)			\
	do {									\
		if (!(cond)) {						\
			msg_err("%s is null", #cond);	\
			return retval;					\
		}									\
	} while (0)

static void max77978_request_vdm(struct max77978_usbc_platform_data *usbpd_data, struct vdm_info *vdm, bool in_lock)
{
	/* send the opcode */
	usbc_cmd_data_t write_data;
	int len = vdm->len;
	int vdm_header_num = sizeof(UND_DATA_MSG_VDM_HEADER_Type);

	null2term(usbpd_data);

	init_usbc_cmd_data(&write_data);
	write_data.opcode = OPCODE_SEND_VDM;
	memcpy(write_data.write_data, vdm->buf, len);
	write_data.write_length = len;
	if (vdm->w_data)
		write_data.write_data[6] = vdm->w_data;
	write_data.read_length = OPCODE_SIZE + OPCODE_HEADER_SIZE + vdm_header_num + (vdm->vdo0_num * 4);
	if (in_lock)
		max77978_usbc_opcode_push(usbpd_data, &write_data);
	else
		max77978_usbc_opcode_write(usbpd_data, &write_data);
}

void max77978_set_discover_identity(void *data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	struct vdm_info vdm;

	null2term(usbpd_data);

	vdm.buf = (void *)&DP_DISCOVER_IDENTITY;
	vdm.len = sizeof(DP_DISCOVER_IDENTITY);
	vdm.vdo0_num = DP_DISCOVER_IDENTITY.byte_data.BITS.Num_Of_VDO;
	vdm.w_data = 0;

	max77978_request_vdm(usbpd_data, &vdm, false);
}

static void max77978_set_dp_enter_mode(void *data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	struct vdm_info vdm;

	null2term(usbpd_data);

	vdm.buf = (void *)&DP_DISCOVER_ENTER_MODE;
	vdm.len = sizeof(DP_DISCOVER_ENTER_MODE);
	vdm.vdo0_num = DP_DISCOVER_ENTER_MODE.byte_data.BITS.Num_Of_VDO;
	vdm.w_data = 0;

	max77978_request_vdm(usbpd_data, &vdm, true);
}

void max77978_set_dp_configure(void *data, uint8_t w_data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	struct vdm_info vdm;

	null2term(usbpd_data);

	vdm.buf = (void *)&DP_CONFIGURE;
	vdm.len = sizeof(DP_CONFIGURE);
	vdm.vdo0_num = DP_CONFIGURE.byte_data.BITS.Num_Of_VDO;
	vdm.w_data = w_data;

	max77978_request_vdm(usbpd_data, &vdm, true);
}

static void max77978_vdm_process_discover_identity(void *data, char *vdm_data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	UND_DATA_MSG_ID_HEADER_Type *DATA_MSG_ID = NULL;
	UND_PRODUCT_VDO_Type *DATA_MSG_PRODUCT = NULL;

	null2term(usbpd_data);

	/* Message Type Definition */
	DATA_MSG_ID = (UND_DATA_MSG_ID_HEADER_Type *)&vdm_data[8];
	DATA_MSG_PRODUCT = (UND_PRODUCT_VDO_Type *)&vdm_data[16];
	usbpd_data->is_sent_pin_configuration = 0;
	usbpd_data->vendor_id = DATA_MSG_ID->BITS.USB_Vendor_ID;
	usbpd_data->product_id = DATA_MSG_PRODUCT->BITS.Product_ID;
	usbpd_data->device_version = DATA_MSG_PRODUCT->BITS.Device_Version;
	msg_info("Vendor_ID : 0x%X, Product_ID : 0x%X Device Version 0x%X",
		usbpd_data->vendor_id, usbpd_data->product_id, usbpd_data->device_version);
}

static void max77978_vdm_process_discover_svids(void *data, char *vdm_data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	int i = 0, cnt = 0;
	uint16_t svid = 0;

	DIS_MODE_DP_CAPA_Type *pDP_DIS_MODE = (DIS_MODE_DP_CAPA_Type *)&vdm_data[2];
	/* Number_of_obj has msg_header & vdm_header, each vdo has 2 svids
	 * This logic can work until Max VDOs 12 */
	int num_of_vdos = pDP_DIS_MODE->MSG_HEADER.BITS.Number_of_obj - 1;
	UND_VDO1_Type  *DATA_MSG_VDO1 = (UND_VDO1_Type  *)&vdm_data[8];

	null2term(usbpd_data);

	usbpd_data->svid_dp = 0;
	usbpd_data->svid_0 = DATA_MSG_VDO1->BITS.SVID_0;
	usbpd_data->svid_1 = DATA_MSG_VDO1->BITS.SVID_1;

	for (i = 0; i < num_of_vdos; i++) {
		int j;

		for (j = 1; j >= 0; j--) {
			int pos = 8 + ((i + j) * 2);

			memcpy(&svid, &vdm_data[pos], 2);
			if (svid == TypeC_DP_SUPPORT) {
				msg_info("svid_%d : 0x%X", cnt, svid);
				usbpd_data->svid_dp = svid;
				break;
			}
			cnt++;
		}
	}

	if (usbpd_data->svid_dp == TypeC_DP_SUPPORT) {
		usbpd_data->dp_is_connect = 1;
		/* If you want to support USB SuperSpeed when you connect
		 * Display port dongle, You should change dp_hs_connect depend
		 * on Pin assignment.If DP use 4lane(Pin Assignment C,E,A),
		 * dp_hs_connect is 1. USB can support HS.If DP use 2lane(Pin Assigment B,D,F), dp_hs_connect is 0. USB
		 * can support SS
		 */
		usbpd_data->dp_hs_connect = 1;
	}
	msg_info("SVID_0 : 0x%X, SVID_1 : 0x%X",
			usbpd_data->svid_0, usbpd_data->svid_1);
}

static u8 max77978_vdm_choose_pin_assignment(DIS_MODE_DP_CAPA_Type *pDP_DIS_MODE)
{
	u8 pin_assignment = DP_PIN_ASSIGNMENT_NODE;
	u8 port_capability = pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.Port_Capability;
	u8 receptacle_indication = pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.Receptacle_Indication;

	/* this setting is based on vesa spec (DP_Alt_Mode_on_USB_Type-C)
	* A USB Type-C Receptacle that supports DFP_D functionality (e.g., the receptacle can behave
	* as a DisplayPort Source device or as a DFP_D on a DisplayPort Branch device) shall support
	* one or more DFP_D pin assignments. Likewise, a USB Type-C Receptacle that supports UFP_D
	* (e.g., the receptacle can behave as a DisplayPort Sink device or as the UFP_D on a DisplayPort
	* Branch device) shall support one or more UFP_D pin assignments.
	*
	* A USB Type-C Plug that is intended to plug directly into a receptacle-based DFP_D (e.g., the
	* plug can behave as a DisplayPort Sink device or as the UFP_D on a DisplayPort Branch device)
	* shall support one or more DFP_D pin assignments. Likewise, a USB Type-C Plug that is intended
	* to plug directly into a receptacle-based UFP_D (e.g., the plug can behave as a DisplayPort Source
	* device or as the DFP_D on a DisplayPort Branch device) shall support one or more UFP_D
	* pin assignments.
	*/

	if (port_capability == num_UFP_D_Capable || port_capability == num_DFP_D_and_UFP_D_Capable) {
		if (receptacle_indication == num_USB_TYPE_C_Receptacle) {
			pin_assignment = pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.UFP_D_Pin_Assignments;
			msg_info("1. support UFP_D 0x%08x", pin_assignment);
		} else {
			pin_assignment = pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS.DFP_D_Pin_Assignments;
			msg_info("2. support DFP_D 0x%08x", pin_assignment);
		}
	} else {
		pin_assignment = DP_PIN_ASSIGNMENT_NODE;
		msg_info("do not support Port_Capability %x", port_capability);
	}

	return pin_assignment;
}

static void max77978_vdm_process_discover_mode(void *data, char *vdm_data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	DIS_MODE_DP_CAPA_Type *pDP_DIS_MODE = (DIS_MODE_DP_CAPA_Type *)&vdm_data[2];
	UND_DATA_MSG_VDM_HEADER_Type *DATA_MSG_VDM = (UND_DATA_MSG_VDM_HEADER_Type *)&vdm_data[4];

	null2term(usbpd_data);

	msg_info("vendor_id = 0x%04x , svid_1 = 0x%04x", DATA_MSG_VDM->BITS.Standard_Vendor_ID, usbpd_data->svid_1);
	if (DATA_MSG_VDM->BITS.Standard_Vendor_ID == TypeC_DP_SUPPORT && usbpd_data->svid_dp == TypeC_DP_SUPPORT) {
		/*  pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.BITS. */
		msg_info("pDP_DIS_MODE->MSG_HEADER.DATA = 0x%08X", pDP_DIS_MODE->MSG_HEADER.DATA);
		msg_info("pDP_DIS_MODE->DATA_MSG_VDM_HEADER.DATA = 0x%08X", pDP_DIS_MODE->DATA_MSG_VDM_HEADER.DATA);
		msg_info("pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.DATA = 0x%08X", pDP_DIS_MODE->DATA_MSG_MODE_VDO_DP.DATA);

		if (pDP_DIS_MODE->MSG_HEADER.BITS.Number_of_obj > 1)
			usbpd_data->pin_assignment = max77978_vdm_choose_pin_assignment(pDP_DIS_MODE);
	}

	max77978_set_dp_enter_mode(usbpd_data);
}

static void max77978_vdm_process_enter_mode(void *data, char *vdm_data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	UND_DATA_MSG_VDM_HEADER_Type *DATA_MSG_VDM = (UND_DATA_MSG_VDM_HEADER_Type *)&vdm_data[4];

	null2term(usbpd_data);

	if (DATA_MSG_VDM->BITS.VDM_command_type == 1) {
		msg_info("EnterMode ACK.");
	} else {
		msg_info("EnterMode NAK.");
	}
}

static int max77978_vdm_dp_select_pin(void *data, int multi_function_preference)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	uint32_t dp_pin = 0;
	int pin_sel = 0;

	null2return(usbpd_data, pin_sel);

	msg_info("multi=%d, pin_assignment=0x%X", multi_function_preference, usbpd_data->pin_assignment);
	if (multi_function_preference) {
		if (usbpd_data->pin_assignment & DP_PIN_ASSIGNMENT_D)
			dp_pin = DP_PIN_ASSIGNMENT_D;
		else if (usbpd_data->pin_assignment & DP_PIN_ASSIGNMENT_B)
			dp_pin = DP_PIN_ASSIGNMENT_B;
		else if (usbpd_data->pin_assignment & DP_PIN_ASSIGNMENT_F)
			dp_pin = DP_PIN_ASSIGNMENT_F;
		else if (usbpd_data->pin_assignment & DP_PIN_ASSIGNMENT_C)
			dp_pin = DP_PIN_ASSIGNMENT_C;
		else if (usbpd_data->pin_assignment & DP_PIN_ASSIGNMENT_E)
			dp_pin = DP_PIN_ASSIGNMENT_E;
		else if (usbpd_data->pin_assignment & DP_PIN_ASSIGNMENT_A)
			dp_pin = DP_PIN_ASSIGNMENT_A;
		else
			msg_err("wrong pin assignment (0x%08X)", usbpd_data->pin_assignment);
	} else {
		if (usbpd_data->pin_assignment & DP_PIN_ASSIGNMENT_C)
			dp_pin = DP_PIN_ASSIGNMENT_C;
		else if (usbpd_data->pin_assignment & DP_PIN_ASSIGNMENT_E)
			dp_pin = DP_PIN_ASSIGNMENT_E;
		else if (usbpd_data->pin_assignment & DP_PIN_ASSIGNMENT_A)
			dp_pin = DP_PIN_ASSIGNMENT_A;
		else if (usbpd_data->pin_assignment & DP_PIN_ASSIGNMENT_D)
			dp_pin = DP_PIN_ASSIGNMENT_D;
		else if (usbpd_data->pin_assignment & DP_PIN_ASSIGNMENT_B)
			dp_pin = DP_PIN_ASSIGNMENT_B;
		else if (usbpd_data->pin_assignment & DP_PIN_ASSIGNMENT_F)
			dp_pin = DP_PIN_ASSIGNMENT_F;
		else
			msg_err("wrong pin assignment (0x%08X)", usbpd_data->pin_assignment);
	}

	while (dp_pin > 0) {
		dp_pin = dp_pin >> 1;
		pin_sel++;
	}

	msg_info("pin_sel=%d", pin_sel);
	return pin_sel;
}

static void max77978_vdm_prepare_dp_configure(struct max77978_usbc_platform_data *usbpd_data, DP_STATUS_UPDATE_Type *DP_STATUS)
{
	uint8_t W_DATA = 0;
	uint8_t multi_func = 0;
	int pin_sel = 0;

	null2term(usbpd_data);

	msg_info("DP_STATUS_UPDATE = 0x%08X", DP_STATUS->DATA_DP_STATUS_UPDATE.DATA);

	if (DP_STATUS->DATA_DP_STATUS_UPDATE.BITS.Port_Connected == 0x00) {
		msg_info("port disconnected!");
		return;
	}

	if (usbpd_data->is_sent_pin_configuration == 0) {
		multi_func = DP_STATUS->DATA_DP_STATUS_UPDATE.BITS.Multi_Function_Preference;
		pin_sel = max77978_vdm_dp_select_pin(usbpd_data, multi_func);
		usbpd_data->dp_selected_pin = pin_sel;
		W_DATA = dp_pin_assignment_data[pin_sel];

		msg_info("multi_func_preference=%d, pin_sel=%s(%d), W_DATA : %d",
			multi_func, dp_pin_assignment_print[pin_sel], pin_sel, W_DATA);

		max77978_set_dp_configure(usbpd_data, W_DATA);

		usbpd_data->is_sent_pin_configuration = 1;
	} else {
		msg_info("pin configuration is already sent as %s!",
			dp_pin_assignment_print[usbpd_data->dp_selected_pin]);
	}

	return;
}

static void max77978_vdm_announce_hpd(struct max77978_usbc_platform_data *usbpd_data, DP_STATUS_UPDATE_Type *DP_STATUS)
{
	null2term(usbpd_data);

	/*
	 * TODO: notify HPD state/interrupt changes to downstream consumers.
	 * HPD_State: 1 = high (connected), 0 = low (disconnected)
	 * HPD_Interrupt: 1 = IRQ pulse detected
	 */
	dev_dbg(usbpd_data->dev, "HPD_State=%d HPD_Interrupt=%d\n",
			DP_STATUS->DATA_DP_STATUS_UPDATE.BITS.HPD_State,
			DP_STATUS->DATA_DP_STATUS_UPDATE.BITS.HPD_Interrupt);
}

static void max77978_vdm_process_dp_status_update(void *data, char *vdm_data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	VDO_MESSAGE_Type *VDO_MSG;
	DP_STATUS_UPDATE_Type *DP_STATUS;
	int i;

	null2term(usbpd_data);

	msg_info("svid_dp=0x%04X", usbpd_data->svid_dp);
	if (usbpd_data->svid_dp == TypeC_DP_SUPPORT) {
		DP_STATUS = (DP_STATUS_UPDATE_Type *)&vdm_data[2];
		max77978_vdm_prepare_dp_configure(usbpd_data, DP_STATUS);
		max77978_vdm_announce_hpd(usbpd_data, DP_STATUS);
	} else {
		/* need to check F/W code */
		VDO_MSG = (VDO_MESSAGE_Type *)&vdm_data[8];
		for (i = 0; i < 6; i++)
			msg_info("VDO_%d : %d", i+1, VDO_MSG->VDO[i]);
	}
}

static void max77978_vdm_process_dp_attention(void *data, char *vdm_data)
{
	max77978_vdm_process_dp_status_update(data, vdm_data);
}

static void max77978_vdm_process_dp_configure(void *data, char *vdm_data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	UND_DATA_MSG_VDM_HEADER_Type *DATA_MSG_VDM = (UND_DATA_MSG_VDM_HEADER_Type *)&vdm_data[4];

	null2term(usbpd_data);

	msg_info("vendor_id = 0x%04x , svid_1 = 0x%04x", DATA_MSG_VDM->BITS.Standard_Vendor_ID, usbpd_data->svid_1);
}

void max77978_vdm_message_handler(struct max77978_usbc_platform_data *usbpd_data, char *opcode_data, int len)
{
	unsigned char vdm_data[OPCODE_DATA_LENGTH] = {0,};
	UND_DATA_MSG_VDM_HEADER_Type vdm_header;

	null2term(usbpd_data);

	memset(&vdm_header, 0, sizeof(UND_DATA_MSG_VDM_HEADER_Type));
	memcpy(vdm_data, opcode_data, len);
	memcpy(&vdm_header, &vdm_data[4], sizeof(vdm_header));

	switch (vdm_data[1]) {
	case OPCODE_ID_VDM_DISCOVER_IDENTITY:
		max77978_vdm_process_printf("VDM_DISCOVER_IDENTITY::", vdm_data, len);
		max77978_vdm_process_discover_identity(usbpd_data, vdm_data);
		break;
	case OPCODE_ID_VDM_DISCOVER_SVIDS:
		max77978_vdm_process_printf("VDM_DISCOVER_SVIDS::", vdm_data, len);
		max77978_vdm_process_discover_svids(usbpd_data, vdm_data);
		break;
	case OPCODE_ID_VDM_DISCOVER_MODES:
		max77978_vdm_process_printf("VDM_DISCOVER_MODES::", vdm_data, len);
		vdm_data[0] = vdm_data[2];
		vdm_data[1] = vdm_data[3];
		max77978_vdm_process_discover_mode(usbpd_data, vdm_data);
		break;
	case OPCODE_ID_VDM_ENTER_MODE:
		max77978_vdm_process_printf("VDM_ENTER_MODE::", vdm_data, len);
		max77978_vdm_process_enter_mode(usbpd_data, vdm_data);
		break;
	case OPCODE_ID_VDM_SVID_DP_STATUS:
		max77978_vdm_process_printf("VDM_SVID_DP_STATUS::", vdm_data, len);
		vdm_data[0] = vdm_data[2];
		vdm_data[1] = vdm_data[3];
		max77978_vdm_process_dp_status_update(usbpd_data, vdm_data);
		break;
	case OPCODE_ID_VDM_SVID_DP_CONFIGURE:
		max77978_vdm_process_printf("VDM_SVID_DP_CONFIGURE::", vdm_data, len);
		max77978_vdm_process_dp_configure(usbpd_data, vdm_data);
		break;
	case OPCODE_ID_VDM_ATTENTION:
		max77978_vdm_process_printf("VDM_ATTENTION::", vdm_data, len);
		vdm_data[0] = vdm_data[2];
		vdm_data[1] = vdm_data[3];
		max77978_vdm_process_dp_attention(usbpd_data, vdm_data);
		break;
	case OPCODE_ID_VDM_EXIT_MODE:
		max77978_vdm_process_printf("VDM_EXIT_MODE::", vdm_data, len);
		break;
	default:
		break;
	}
}

struct vdm_response {
	u8 data;
	int write_length;
	int read_length;
};

static int max77978_get_vdm_response(struct max77978_usbc_platform_data *usbpd_data, struct vdm_response *vdm)
{
	/* send the opcode */
	usbc_cmd_data_t write_data;
	int ret = 0;

	null2return(usbpd_data, -ENOENT);

	init_usbc_cmd_data(&write_data);
	write_data.opcode = OPCODE_GET_VDM_RESP;
	write_data.write_data[0] = vdm->data;
	write_data.write_length = vdm->write_length;
	write_data.read_length = vdm->read_length;
	ret = max77978_usbc_opcode_write(usbpd_data, &write_data);
	if (ret)
		msg_info("%s error. ret=%d", __func__, ret);

	return ret;
}

static int max77978_get_discover_identity(void *data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	struct vdm_response vdm;

	if (!usbpd_data) {
		msg_info("%s usbpd_data is null.", __func__);
		return -ENOENT;
	}

	vdm.data = OPCODE_ID_VDM_DISCOVER_IDENTITY;
	vdm.write_length = 1;
	vdm.read_length = DISCOVER_IDENTITY_RESPONSE_SIZE;
	return max77978_get_vdm_response(usbpd_data, &vdm);
}

static int max77978_get_discover_svids(void *data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	struct vdm_response vdm;

	vdm.data = OPCODE_ID_VDM_DISCOVER_SVIDS;
	vdm.write_length = 1;
	vdm.read_length = DISCOVER_SVIDS_RESPONSE_SIZE;
	return max77978_get_vdm_response(usbpd_data, &vdm);
}

static int max77978_get_discover_modes(void *data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	struct vdm_response vdm;

	vdm.data = OPCODE_ID_VDM_DISCOVER_MODES;
	vdm.write_length = 1;
	vdm.read_length = DISCOVER_MODES_RESPONSE_SIZE;
	return max77978_get_vdm_response(usbpd_data, &vdm);
}

static int max77978_get_enter_mode(void *data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	struct vdm_response vdm;

	vdm.data = OPCODE_ID_VDM_ENTER_MODE;
	vdm.write_length = 1;
	vdm.read_length = ENTER_MODE_RESPONSE_SIZE;
	return max77978_get_vdm_response(usbpd_data, &vdm);
}

static int max77978_get_attention(void *data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	struct vdm_response vdm;

	vdm.data = OPCODE_ID_VDM_ATTENTION;
	vdm.write_length = 1;
	vdm.read_length = ATTENTION_RESPONSE_SIZE;
	return max77978_get_vdm_response(usbpd_data, &vdm);
}

static int max77978_get_dp_status_update(void *data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	struct vdm_response vdm;

	vdm.data = OPCODE_ID_VDM_SVID_DP_STATUS;
	vdm.write_length = 1;
	vdm.read_length = DP_STATUS_RESPONSE_SIZE;
	return max77978_get_vdm_response(usbpd_data, &vdm);
}

static int max77978_get_dp_configure(void *data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	struct vdm_response vdm;

	vdm.data = OPCODE_ID_VDM_SVID_DP_CONFIGURE;
	vdm.write_length = 1;
	vdm.read_length = DP_CONFIGURE_RESPONSE_SIZE;
	return max77978_get_vdm_response(usbpd_data, &vdm);
}

static void max77978_process_alternate_mode(void *data)
{
	struct max77978_usbc_platform_data *usbpd_data = data;
	uint32_t mode;

	if (!usbpd_data) {
		msg_info("%s usbpd_data is null.", __func__);
		return;
	}

	mode = usbpd_data->alternate_state;

	if (mode) {
		msg_info("mode : 0x%x", mode);

		if (mode & VDM_DISCOVER_ID)
			if (max77978_get_discover_identity(usbpd_data))
				goto process_error;
		if (mode & VDM_DISCOVER_SVIDS)
			if (max77978_get_discover_svids(usbpd_data))
				goto process_error;
		if (mode & VDM_DISCOVER_MODES)
			if (max77978_get_discover_modes(usbpd_data))
				goto process_error;
		if (mode & VDM_ENTER_MODE)
			if (max77978_get_enter_mode(usbpd_data))
				goto process_error;
		if (mode & VDM_DP_STATUS_UPDATE)
			if (max77978_get_dp_status_update(usbpd_data))
				goto process_error;
		if (mode & VDM_DP_CONFIGURE)
			if (max77978_get_dp_configure(usbpd_data))
				goto process_error;
		if (mode & VDM_ATTENTION)
			if (max77978_get_attention(usbpd_data))
				goto process_error;
process_error:
		usbpd_data->alternate_state = 0;
	}
}

void max77978_receive_alternate_message(struct max77978_usbc_platform_data *data)
{
	u8 dp_status, i;
	struct max77978_usbc_platform_data *usbpd_data = data;

	null2term(usbpd_data);

	for (i = 0; i < 8; i++) {
		dp_status = (1 << i);
		if (usbpd_data->max77978->dp_status & dp_status) {
			msg_info("%s", vdm_msg_irq_state_print[i + 1]);
			usbpd_data->alternate_state |= dp_status;
		}
	}

	max77978_process_alternate_mode(usbpd_data);
}

MODULE_DESCRIPTION("max77978 Alternate driver");
MODULE_AUTHOR("Analog Device Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.2.1");
