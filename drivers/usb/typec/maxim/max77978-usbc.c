// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAX77978 USBC Driver
 *
 * Copyright (c) 2024 Analog Devices, Inc.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/mod_devicetable.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/file.h>
#include <linux/sort.h>
#include <linux/usb/typec/maxim/max77978.h>
#include <linux/usb/typec/maxim/max77978-private.h>
#include <linux/usb/typec/maxim/max77978-usbc.h>
#if defined(CONFIG_MAX77978_DEBUG)
#include <linux/usb/typec/maxim/max77978-debug.h>
#endif
#include <linux/usb/typec/maxim/max77978-alternate.h>

/* Soft reset register and value */
#define MAX77978_SOFT_RESET_VAL		0x0F

struct max77978_usbc_platform_data *g_usbc_data;

static void max77978_get_version_info(struct max77978_usbc_platform_data *usbc_data);
int max77978_i2c_opcode_write(struct max77978_usbc_platform_data *usbc_data, u8 opcode, u8 length, u8 *values);
int max77978_i2c_opcode_read(struct max77978_usbc_platform_data *usbc_data, u8 opcode, u8 length, u8 *values);

static int max77978_current_pr_state(struct max77978_usbc_platform_data *usbc_data)
{
	return usbc_data->cc_data->current_port_type;
}

static void vbus_control_hard_reset(struct work_struct *work)
{
	struct max77978_usbc_platform_data *usbpd_data =
		container_of(work, struct max77978_usbc_platform_data,
			     vbus_hard_reset_work.work);
	int current_pr = max77978_current_pr_state(usbpd_data);

	msg_info("current_pr=%d", current_pr);

	if (current_pr == TYPEC_PORT_SRC)
		max77978_vbus_turn_on_ctrl(usbpd_data, VBUS_ON);
}

static void max77978_send_role_swap_message(struct max77978_usbc_platform_data *usbpd_data, u8 mode)
{
	usbc_cmd_data_t cmd_data;

	usbpd_data->role_swap_type = mode;
	max77978_usbc_clear_queue(usbpd_data);
	init_usbc_cmd_data(&cmd_data);
	cmd_data.opcode = OPCODE_SEND_SWAP_REQUEST;
	/* 0x1 : DR_SWAP, 0x2 : PR_SWAP, 0x4: Manual Role Swap */
	cmd_data.write_data[0] = mode;
	cmd_data.write_length = 0x1;
	cmd_data.read_length = 0x1;
	max77978_usbc_opcode_write(usbpd_data, &cmd_data);
}

static void max77978_send_role_swap_push(struct max77978_usbc_platform_data *usbpd_data, u8 mode)
{
	usbc_cmd_data_t cmd_data;

	init_usbc_cmd_data(&cmd_data);
	cmd_data.opcode = OPCODE_SEND_SWAP_REQUEST;
	/* 0x1 : DR_SWAP, 0x2 : PR_SWAP, 0x4: Manual Role Swap */
	cmd_data.write_data[0] = mode;
	cmd_data.write_length = 0x1;
	cmd_data.read_length = 0x1;
	max77978_usbc_opcode_push(usbpd_data, &cmd_data);
}

static int max77978_dr_set(struct typec_port *port, enum typec_data_role role)
{
	struct max77978_usbc_platform_data *usbpd_data = typec_get_drvdata(port);

	if (!usbpd_data)
		return -EINVAL;

	msg_info("typec_power_role=%d(%s), typec_data_role=%d(%s), data_role=%d(%s)",
			usbpd_data->typec_power_role, usbpd_data->typec_power_role == TYPEC_SINK ? "TYPEC_SINK" : usbpd_data->typec_power_role == TYPEC_SOURCE ? "TYPEC_SOURCE" : "UNKNOWN",
			usbpd_data->typec_data_role, usbpd_data->typec_data_role == TYPEC_DEVICE ? "TYPEC_DEVICE" : usbpd_data->typec_data_role == TYPEC_HOST ? "TYPEC_HOST" : "UNKNOWN",
			role, role == TYPEC_DEVICE ? "TYPEC_DEVICE" : role == TYPEC_HOST ? "TYPEC_HOST" : "UNKNOWN");

	if (usbpd_data->typec_data_role != TYPEC_DEVICE &&
			usbpd_data->typec_data_role != TYPEC_HOST)
		return -EPERM;
	else if (usbpd_data->typec_data_role == role)
		return -EPERM;

	reinit_completion(&usbpd_data->typec_reverse_completion);
	if (role == TYPEC_DEVICE) {
		msg_info("try reversing, from DFP to UFP");
		usbpd_data->typec_try_state_change = TRY_ROLE_SWAP_DR;
		max77978_send_role_swap_message(usbpd_data, DATA_ROLE_SWAP); // for attaching UFP role
	} else if (role == TYPEC_HOST) {
		msg_info("try reversing, from UFP to DFP");
		usbpd_data->typec_try_state_change = TRY_ROLE_SWAP_DR;
		max77978_send_role_swap_message(usbpd_data, DATA_ROLE_SWAP); // for attaching DFP role
	} else {
		msg_info("invalid typec_role");
		return -EIO;
	}
	if (!wait_for_completion_timeout(&usbpd_data->typec_reverse_completion,
				msecs_to_jiffies(TRY_ROLE_SWAP_WAIT_MS))) {
		usbpd_data->typec_try_state_change = TRY_ROLE_SWAP_NONE;
		return -ETIMEDOUT;
	}

	return 0;
}

static int max77978_pr_set(struct typec_port *port, enum typec_role role)
{
	struct max77978_usbc_platform_data *usbpd_data = typec_get_drvdata(port);

	if (!usbpd_data)
		return -EINVAL;

	msg_info("typec_power_role=%d(%s), typec_data_role=%d(%s), power_role=%d(%s)",
			usbpd_data->typec_power_role, usbpd_data->typec_power_role == TYPEC_SINK ? "TYPEC_SINK" : usbpd_data->typec_power_role == TYPEC_SOURCE ? "TYPEC_SOURCE" : "UNKNOWN",
			usbpd_data->typec_data_role, usbpd_data->typec_data_role == TYPEC_DEVICE ? "TYPEC_DEVICE" : usbpd_data->typec_data_role == TYPEC_HOST ? "TYPEC_HOST" : "UNKNOWN",
			role, role == TYPEC_SINK ? "TYPEC_SINK" : role == TYPEC_SOURCE ? "TYPEC_SOURCE" : "UNKNOWN");

	if (usbpd_data->typec_power_role != TYPEC_SINK
	    && usbpd_data->typec_power_role != TYPEC_SOURCE)
		return -EPERM;
	else if (usbpd_data->typec_power_role == role)
		return -EPERM;

	reinit_completion(&usbpd_data->typec_reverse_completion);
	if (role == TYPEC_SINK) {
		msg_info("try reversing, from Source to Sink");
		usbpd_data->typec_try_state_change = TRY_ROLE_SWAP_PR;
		max77978_send_role_swap_message(usbpd_data, POWER_ROLE_SWAP); // for attaching SNK role
	} else if (role == TYPEC_SOURCE) {
		msg_info("try reversing, from Sink to Source");
		usbpd_data->typec_try_state_change = TRY_ROLE_SWAP_PR;
		max77978_send_role_swap_message(usbpd_data, POWER_ROLE_SWAP); // for attaching SRC role
	} else {
		msg_info("invalid typec_role");
		return -EIO;
	}
	if (!wait_for_completion_timeout(&usbpd_data->typec_reverse_completion,
				msecs_to_jiffies(TRY_ROLE_SWAP_WAIT_MS))) {
		usbpd_data->typec_try_state_change = TRY_ROLE_SWAP_NONE;
		if (usbpd_data->typec_power_role != role)
			return -ETIMEDOUT;
	}

	return 0;
}

static const struct typec_operations max77978_ops = {
	.dr_set = max77978_dr_set,
	.pr_set = max77978_pr_set,
};

int max77978_get_pd_support(struct max77978_usbc_platform_data *usbc_data)
{
	msg_info("usbc_data->pd_support is %d, usbc_data->pwr_opmode is %d", usbc_data->pd_support, usbc_data->pwr_opmode);
	if (usbc_data->pd_support)
		return TYPEC_PWR_MODE_PD;

	return usbc_data->pwr_opmode;
}

static void max77978_get_version_info(struct max77978_usbc_platform_data *usbc_data)
{
	u8 chip_info[4] = {0, };

	max77978_read_reg(usbc_data->i2c, REG_UIC_DEVICE_ID, &chip_info[0]);
	max77978_read_reg(usbc_data->i2c, REG_UIC_DEVICE_REV, &chip_info[1]);
	max77978_read_reg(usbc_data->i2c, REG_UIC_FW_REV, &chip_info[2]);
	max77978_read_reg(usbc_data->i2c, REG_UIC_FW_SUBREV, &chip_info[3]);

	usbc_data->device_id = chip_info[0];
	usbc_data->device_revision = chip_info[1];
	usbc_data->fw_revision = chip_info[2];
	usbc_data->fw_minor_revision = chip_info[3];

	msg_info("DevID=%02Xh DevRev=%02Xh FWRev=%02X.%02X",
			usbc_data->device_id,
			usbc_data->device_revision,
			usbc_data->fw_revision,
			usbc_data->fw_minor_revision);
}

void init_usbc_cmd_data(usbc_cmd_data_t *cmd_data)
{
	cmd_data->opcode = OPCODE_NONE;
	cmd_data->prev_opcode = OPCODE_NONE;
	cmd_data->response = OPCODE_NONE;
	cmd_data->val = REG_NONE;
	cmd_data->mask = REG_NONE;
	cmd_data->reg = REG_NONE;
	cmd_data->noti_cmd = OPCODE_NOTI_NONE;
	cmd_data->write_length = 0;
	cmd_data->read_length = 0;
	cmd_data->seq = 0;
	cmd_data->is_uvdm = 0;
	memset(cmd_data->write_data, REG_NONE, OPCODE_DATA_LENGTH);
	memset(cmd_data->read_data, REG_NONE, OPCODE_DATA_LENGTH);
}

static void init_usbc_cmd_node(usbc_cmd_node_t *cmd_node)
{
	usbc_cmd_data_t *cmd_data = &(cmd_node->cmd_data);

	msg_debug("IN");

	cmd_node->next = NULL;
	init_usbc_cmd_data(cmd_data);
}

static void copy_usbc_cmd_data(usbc_cmd_data_t *from, usbc_cmd_data_t *to)
{
	if (from != NULL && to != NULL)
		memcpy(to, from, sizeof(*to));
}

bool is_empty_usbc_cmd_queue(usbc_cmd_queue_t *cmd_queue)
{
	if (cmd_queue->front == NULL) {
		msg_info("cmd_queue Empty");
		return true;
	}
	msg_debug("cmd_queue NOT Empty");
	return false;
}

static void enqueue_usbc_cmd_data(usbc_cmd_queue_t *cmd_queue, usbc_cmd_data_t *cmd_data)
{
	usbc_cmd_node_t *temp_node = kzalloc(sizeof(usbc_cmd_node_t), GFP_KERNEL);

	if (!temp_node) {
		msg_info("failed to allocate usbc command queue");
		return;
	}

	init_usbc_cmd_node(temp_node);

	copy_usbc_cmd_data(cmd_data, &(temp_node->cmd_data));

	if (is_empty_usbc_cmd_queue(cmd_queue)) {
		cmd_queue->front = temp_node;
		cmd_queue->rear = temp_node;
	} else {
		cmd_queue->rear->next = temp_node;
		cmd_queue->rear = temp_node;
	}
}

static void dequeue_usbc_cmd_data(usbc_cmd_queue_t *cmd_queue, usbc_cmd_data_t *cmd_data)
{
	usbc_cmd_node_t *temp_node;

	if (is_empty_usbc_cmd_queue(cmd_queue)) {
		msg_info("Queue, Empty!");
		return;
	}

	temp_node = cmd_queue->front;
	copy_usbc_cmd_data(&(temp_node->cmd_data), cmd_data);

	msg_info("Opcode(0x%02x) Response(0x%02x)", cmd_data->opcode, cmd_data->response);

	if (cmd_queue->front->next == NULL) {
		//msg_info("front->next = NULL");
		cmd_queue->front = NULL;
	} else
		cmd_queue->front = cmd_queue->front->next;

	if (is_empty_usbc_cmd_queue(cmd_queue))
		cmd_queue->rear = NULL;

	kfree(temp_node);
}

static bool front_usbc_cmd(usbc_cmd_queue_t *cmd_queue, usbc_cmd_data_t *cmd_data)
{
	if (is_empty_usbc_cmd_queue(cmd_queue)) {
		msg_info("Queue, Empty!");
		return false;
	}

	copy_usbc_cmd_data(&(cmd_queue->front->cmd_data), cmd_data);
	msg_info("Opcode(0x%02x)", cmd_data->opcode);
	return true;
}

static void max77978_irq_execute(struct max77978_usbc_platform_data *usbc_data, const struct max77978_usbc_command_data *cmd_data)
{
	int len = cmd_data->read_length;
	unsigned char data[OPCODE_DATA_LENGTH + OPCODE_SIZE] = {0,};
	u8 response = 0xff;
	u8 vdm_opcode_header = 0x0;
	UND_DATA_MSG_VDM_HEADER_Type vdm_header;
	u8 vdm_command = 0x0;
	u8 vdm_type = 0x0;
	u8 vdm_response = 0x0;
	u8 reqd_vdm_command = 0;
	u8 swap_request_result = 0x0;
	uint8_t W_DATA = 0x0;

	memset(&vdm_header, 0, sizeof(UND_DATA_MSG_VDM_HEADER_Type));
	/* Read opcode response from fixed OPCODE_READ register (0x51) and following */
	max77978_i2c_opcode_read(usbc_data, OPCODE_READ, len, data);

	/* opcode identifying the message type. (0x51)*/
	response = data[0];

	if (response != cmd_data->response)
		msg_info("Response [0x%02x] != [0x%02x]", response, cmd_data->response);

	/* to do(read switch case) */
	switch (response) {
	case OPCODE_VCONN_SWITCH_OVERCURRENT_R:
	case OPCODE_VCONN_SWITCH_OVERCURRENT_W:
	case OPCODE_VBUS_ALARM_R:
	case OPCODE_VBUS_ALARM_W:
	case OPCODE_ERROR_RECOVERY:
	case OPCODE_GPIO_CONTROL_R:
	case OPCODE_GPIO_CONTROL_W:
	case OPCODE_GPIO0_GPIO1_ADC:
	case OPCODE_GET_SNKCAP:
	case OPCODE_SET_SRCCAP:
	case OPCODE_SEND_GET_RESPONSE:
	case OPCODE_SEND_SWAP_RESPONSE:
	case OPCODE_SRCCAP_APDO_REQUEST:
	case OPCODE_SET_PPS_MODE:
	case OPCODE_SNK_PDO_REQUEST:
	case OPCODE_SNK_PDO_SET:
	case OPCODE_GET_PD_MESSAGE:
	case OPCODE_VENDOR_BC_CTRL2_BC_CTRL1_R:
	case OPCODE_VENDOR_BC_CTRL2_BC_CTRL1_W:
	case OPCODE_VENDOR_USBSW_CTRL_R:
	case OPCODE_VENDOR_USBSW_CTRL_W:
	case OPCODE_VENDOR_SBUSW_CTRL_R:
	case OPCODE_VENDOR_SBUSW_CTRL_W:
	case OPCODE_EUSB_REVISION:
	case OPCODE_SRC_PDO_REQUEST:
	case OPCODE_SRC_PDO_SET:
	case OPCODE_EUSB_TUNE1_R:
	case OPCODE_EUSB_TUNE1_W:
	case OPCODE_EUSB_TUNE2_R:
	case OPCODE_EUSB_TUNE2_W:
	case OPCODE_EUSB_TUNE3_R:
	case OPCODE_EUSB_TUNE3_W:
	case OPCODE_EUSB_LDO_CTRL_R:
	case OPCODE_EUSB_LDO_CTRL_W:
	case OPCODE_EUSB_LDO_STATUS_R:
	case OPCODE_EUSB_CTL_R:
	case OPCODE_EUSB_CTL_W:
	case OPCODE_SET_CSTM_INFORMATION_R:
	case OPCODE_SET_CSTM_INFORMATION_W:
	case OPCODE_ACTION_BLOCK_MTP_UPDATE:
	case OPCODE_GET_SRCCAP:
	case OPCODE_SEND_GET_REQUEST:
	case OPCODE_READ_GET_REQUEST:
		break;
	case OPCODE_CURRENT_SRCCAP:
		max77978_current_pdo(usbc_data, data);
		break;
	case OPCODE_SRCCAP_REQUEST:
		if (data[1] == 0xfe || data[1] == 0xff) {
			usbc_data->srccap_request_retry = true;
			msg_info("srccap_request_retry is set");
		}
		break;
	case OPCODE_MASTER_I2C_READ:
	case OPCODE_MASTER_I2C_WRITE:
		break;

	case OPCODE_GET_VDM_RESP:
		max77978_vdm_message_handler(usbc_data, data, len + OPCODE_SIZE);
		break;
	case OPCODE_SEND_VDM:
		vdm_opcode_header = data[1];
		switch (vdm_opcode_header) {
		case 0xFF:
			msg_info("This isn't invalid response(OPCODE : 0x48, HEADER : 0xFF)");
			break;
		default:
			memcpy(&vdm_header, &data[2], sizeof(vdm_header));
			vdm_type = vdm_header.BITS.VDM_Type;
			vdm_command = vdm_header.BITS.VDM_command;
			vdm_response = vdm_header.BITS.VDM_command_type;
			msg_info("vdm_type[%x], vdm_command[%x], vdm_response[%x]",
				vdm_type, vdm_command, vdm_response);
			switch (vdm_type) {
			case STRUCTURED_VDM:
				if (vdm_response == SEC_UVDM_RESPONDER_ACK) {
					switch (vdm_command) {
					case Discover_Identity:
						msg_info("ignore Discover_Identity");
						break;
					case Discover_SVIDs:
						msg_info("ignore Discover_SVIDs");
						break;
					case Discover_Modes:
						msg_info("ignore Discover_Modes");
						break;
					case Enter_Mode:
						msg_info("ignore Enter_Mode");
						break;
					case Exit_Mode:
						msg_info("ignore Exit_Mode");
						break;
					case Attention:
						msg_info("ignore Attention");
						break;
					case Configure:
						break;
					default:
						msg_info("vdm_command isn't valid[%x]", vdm_command);
						break;
					}
				} else if (vdm_response == SEC_UVDM_ININIATOR) {
					switch (vdm_command) {
					case Attention:
						/* Attention message is not able to be received via 0x48 OPCode */
						/* Check requested vdm command and responded vdm command */
						{
							/* Read requested vdm command */
							max77978_read_reg(usbc_data->i2c, 0x23, &reqd_vdm_command);
							reqd_vdm_command &= 0x1F; /* Command bit, b4...0 */

							if (reqd_vdm_command == Configure) {
								W_DATA = 1 << (usbc_data->dp_selected_pin - 1);
								/* Retry Configure message */
								msg_info("Retry Configure message, W_DATA = %x, dp_selected_pin = %d",
										W_DATA, usbc_data->dp_selected_pin);
								max77978_set_dp_configure(usbc_data, W_DATA);
							}
						}
						break;
					case Discover_Identity:
					case Discover_SVIDs:
					case Discover_Modes:
					case Enter_Mode:
					case Configure:
					default:
						/* Nothing */
						break;
					}
				} else
					msg_info("vdm_response is error value[%x]", vdm_response);
				break;
			case SEC_UVDM_UNSTRUCTURED_VDM:
				msg_info("SEC_UVDM_UNSTRUCTURED_VDM");
				break;
			default:
				msg_info("vdm_type isn't valid error");
				break;
			}
			break;
		}
		break;
	case OPCODE_SEND_SWAP_REQUEST:
		swap_request_result = data[1];
		switch (swap_request_result) {
		case ROLE_ACCEPT:
			usbc_data->role_swap_type = 0;
			break;
		case ROLE_BUSY:
			if (usbc_data->role_swap_type == DATA_ROLE_SWAP) {
				usbc_data->role_swap_type = DATA_ROLE_SWAP_RETRY;
				msg_info("Retry DATA_ROLE_SWAP");
				msleep(100);
				max77978_send_role_swap_push(usbc_data, DATA_ROLE_SWAP);
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void max77978_usbc_dequeue_queue(struct max77978_usbc_platform_data *usbc_data)
{
	usbc_cmd_data_t cmd_data;
	usbc_cmd_queue_t *cmd_queue = NULL;

	cmd_queue = &(usbc_data->cmd_queue);

	init_usbc_cmd_data(&cmd_data);

	if (is_empty_usbc_cmd_queue(cmd_queue)) {
		msg_info("Queue, Empty");
		return;
	}

	dequeue_usbc_cmd_data(cmd_queue, &cmd_data);
	msg_info("!! Dequeue queue : opcode : %x, 1st data : %x. 2st data : %x",
		cmd_data.write_data[0],
		cmd_data.read_data[0],
		cmd_data.val);
}

static void max77978_usbc_clear_fw_queue(struct max77978_usbc_platform_data *usbc_data)
{
	usbc_cmd_data_t write_data;

	msg_info("called");

	init_usbc_cmd_data(&write_data);
	write_data.opcode = OPCODE_FW_OPCODE_CLEAR;
	max77978_usbc_opcode_write(usbc_data, &write_data);
}

void max77978_usbc_clear_queue(struct max77978_usbc_platform_data *usbc_data)
{
	usbc_cmd_data_t cmd_data;
	usbc_cmd_queue_t *cmd_queue = &usbc_data->cmd_queue;

	msg_info("IN");
	mutex_lock(&usbc_data->op_lock);

	while (!is_empty_usbc_cmd_queue(cmd_queue)) {
		init_usbc_cmd_data(&cmd_data);
		dequeue_usbc_cmd_data(cmd_queue, &cmd_data);
	}
	usbc_data->opcode_stamp = 0;
	mutex_unlock(&usbc_data->op_lock);
	/* also clear fw opcode queue to sync with driver */
	max77978_usbc_clear_fw_queue(usbc_data);
}

static void max77978_usbc_cmd_run(struct max77978_usbc_platform_data *usbc_data)
{
	usbc_cmd_queue_t *cmd_queue = NULL;
	usbc_cmd_data_t cmd_data;
	int ret = 0;

	cmd_queue = &(usbc_data->cmd_queue);

	init_usbc_cmd_data(&cmd_data);

	if (is_empty_usbc_cmd_queue(cmd_queue)) {
		msg_info("Queue, Empty");
		return;
	}

	dequeue_usbc_cmd_data(cmd_queue, &cmd_data);

	if (cmd_data.opcode == OPCODE_NONE) {/* Apcmdres isr */
		msg_info("Apcmdres ISR !!!");
		max77978_irq_execute(usbc_data, &cmd_data);
		usbc_data->opcode_stamp = 0;
		max77978_usbc_cmd_run(usbc_data);
	} else { /* No ISR */
		msg_info("No ISR");
		copy_usbc_cmd_data(&cmd_data, &(usbc_data->last_opcode));
		ret = max77978_i2c_opcode_write(usbc_data, cmd_data.opcode, cmd_data.write_length, cmd_data.write_data);
		if (ret < 0) {
			msg_info("i2c write fail. dequeue opcode");
			max77978_usbc_dequeue_queue(usbc_data);
		}
	}
}

int max77978_i2c_opcode_write(struct max77978_usbc_platform_data *usbc_data, u8 opcode, u8 length, u8 *values)
{
	u8 write_values[OPCODE_MAX_LENGTH] = { 0, };
	int ret = 0;

	if (length > OPCODE_DATA_LENGTH)
		return -EMSGSIZE;

	write_values[0] = opcode;
	if (length)
		memcpy(&write_values[1], values, length);

	/* Write opcode and data */
	ret = max77978_opcode_write(usbc_data->i2c, OPCODE_MAX_LENGTH, write_values);
	if (ret == 0)
		usbc_data->opcode_stamp = jiffies;

	return ret;
}

int max77978_i2c_opcode_read(struct max77978_usbc_platform_data *usbc_data, u8 opcode, u8 length, u8 *values)
{
	if (length > OPCODE_DATA_LENGTH)
		return -EMSGSIZE;

	/* Read opcode data */
	return max77978_opcode_read(usbc_data->i2c, length + OPCODE_SIZE, values);
}

int max77978_usbc_opcode_write(struct max77978_usbc_platform_data *usbc_data, usbc_cmd_data_t *write_op)
{
	usbc_cmd_queue_t *cmd_queue = &(usbc_data->cmd_queue);
	usbc_cmd_data_t execute_cmd_data;
	usbc_cmd_data_t current_cmd_data;

	mutex_lock(&usbc_data->op_lock);
	init_usbc_cmd_data(&current_cmd_data);

	/* the messages sent to USBC. */
	init_usbc_cmd_data(&execute_cmd_data);
	execute_cmd_data.opcode = write_op->opcode;
	execute_cmd_data.write_length = write_op->write_length;
	execute_cmd_data.is_uvdm = write_op->is_uvdm;
	memcpy(execute_cmd_data.write_data, write_op->write_data, OPCODE_DATA_LENGTH);
	execute_cmd_data.seq = OPCODE_WRITE_SEQ;
	enqueue_usbc_cmd_data(cmd_queue, &execute_cmd_data);

	/* the messages recevied From USBC. */
	init_usbc_cmd_data(&execute_cmd_data);
	execute_cmd_data.response = write_op->opcode;
	execute_cmd_data.read_length = write_op->read_length;
	execute_cmd_data.is_uvdm = write_op->is_uvdm;
	execute_cmd_data.seq = OPCODE_WRITE_SEQ;
	enqueue_usbc_cmd_data(cmd_queue, &execute_cmd_data);

	msg_info("W->W opcode[0x%02x] write_length[%d] read_length[%d]",
		write_op->opcode, write_op->write_length, write_op->read_length);

	front_usbc_cmd(cmd_queue, &current_cmd_data);
	if (current_cmd_data.opcode == write_op->opcode)
		max77978_usbc_cmd_run(usbc_data);
	else {
		msg_info("!!!current_cmd_data.opcode [0x%02x][0x%02x], read_op->opcode[0x%02x]",
			current_cmd_data.opcode, current_cmd_data.response, write_op->opcode);
		if (usbc_data->opcode_stamp != 0 && current_cmd_data.opcode == OPCODE_NONE) {
			if (time_after(jiffies,
					usbc_data->opcode_stamp + MAX77978_MAX_APDCMD_TIME)) {
				usbc_data->opcode_stamp = 0;
				msg_info("error. we will dequeue response data");
				max77978_usbc_dequeue_queue(usbc_data);
				max77978_usbc_cmd_run(usbc_data);
			}
		}
	}

	mutex_unlock(&usbc_data->op_lock);

	return 0;
}

int max77978_usbc_opcode_read(struct max77978_usbc_platform_data *usbc_data, usbc_cmd_data_t *read_op)
{
	usbc_cmd_queue_t *cmd_queue = &(usbc_data->cmd_queue);
	usbc_cmd_data_t execute_cmd_data;
	usbc_cmd_data_t current_cmd_data;

	mutex_lock(&usbc_data->op_lock);
	init_usbc_cmd_data(&current_cmd_data);

	/* the messages sent to USBC. */
	init_usbc_cmd_data(&execute_cmd_data);
	execute_cmd_data.opcode = read_op->opcode;
	execute_cmd_data.write_length = read_op->write_length;
	execute_cmd_data.is_uvdm = read_op->is_uvdm;
	memcpy(execute_cmd_data.write_data, read_op->write_data, read_op->write_length);
	execute_cmd_data.seq = OPCODE_READ_SEQ;
	enqueue_usbc_cmd_data(cmd_queue, &execute_cmd_data);

	/* the messages recevied From USBC. */
	init_usbc_cmd_data(&execute_cmd_data);
	execute_cmd_data.response = read_op->opcode;
	execute_cmd_data.read_length = read_op->read_length;
	execute_cmd_data.is_uvdm = read_op->is_uvdm;
	execute_cmd_data.seq = OPCODE_READ_SEQ;
	enqueue_usbc_cmd_data(cmd_queue, &execute_cmd_data);

	msg_info("R->R opcode[0x%02x] write_length[%d] read_length[%d]",
		read_op->opcode, read_op->write_length, read_op->read_length);

	front_usbc_cmd(cmd_queue, &current_cmd_data);
	if (current_cmd_data.opcode == read_op->opcode)
		max77978_usbc_cmd_run(usbc_data);
	else {
		msg_info("!!!current_cmd_data.opcode [0x%02x][0x%02x], read_op->opcode[0x%02x]",
			current_cmd_data.opcode, current_cmd_data.response, read_op->opcode);
		if (usbc_data->opcode_stamp != 0 && current_cmd_data.opcode == OPCODE_NONE) {
			if (time_after(jiffies,
					usbc_data->opcode_stamp + MAX77978_MAX_APDCMD_TIME)) {
				usbc_data->opcode_stamp = 0;
				msg_info("error. we will dequeue response data");
				max77978_usbc_dequeue_queue(usbc_data);
				max77978_usbc_cmd_run(usbc_data);
			}
		}
	}

	mutex_unlock(&usbc_data->op_lock);

	return 0;
}

int max77978_usbc_opcode_push(struct max77978_usbc_platform_data *usbc_data, usbc_cmd_data_t *read_op)
{
	usbc_cmd_queue_t *cmd_queue = &(usbc_data->cmd_queue);
	usbc_cmd_data_t execute_cmd_data;
	usbc_cmd_data_t current_cmd_data;

	mutex_lock(&usbc_data->op_lock);

	init_usbc_cmd_data(&current_cmd_data);

	/* the messages sent to USBC. */
	init_usbc_cmd_data(&execute_cmd_data);
	execute_cmd_data.opcode = read_op->opcode;
	execute_cmd_data.write_length = read_op->write_length;
	execute_cmd_data.is_uvdm = read_op->is_uvdm;
	memcpy(execute_cmd_data.write_data, read_op->write_data, read_op->write_length);
	execute_cmd_data.seq = OPCODE_PUSH_SEQ;
	enqueue_usbc_cmd_data(cmd_queue, &execute_cmd_data);

	/* the messages recevied From USBC. */
	init_usbc_cmd_data(&execute_cmd_data);
	execute_cmd_data.response = read_op->opcode;
	execute_cmd_data.read_length = read_op->read_length;
	execute_cmd_data.is_uvdm = read_op->is_uvdm;
	execute_cmd_data.seq = OPCODE_PUSH_SEQ;
	enqueue_usbc_cmd_data(cmd_queue, &execute_cmd_data);

	mutex_unlock(&usbc_data->op_lock);

	msg_info("P->P opcode[0x%02x] write_length[%d] read_length[%d]",
		read_op->opcode, read_op->write_length, read_op->read_length);

	return 0;
}

int max77978_usbc_opcode_rw(struct max77978_usbc_platform_data *usbc_data, usbc_cmd_data_t *read_op, usbc_cmd_data_t *write_op)
{
	usbc_cmd_queue_t *cmd_queue = &(usbc_data->cmd_queue);
	usbc_cmd_data_t execute_cmd_data;
	usbc_cmd_data_t current_cmd_data;

	mutex_lock(&usbc_data->op_lock);
	init_usbc_cmd_data(&current_cmd_data);

	/* the messages sent to USBC. */
	init_usbc_cmd_data(&execute_cmd_data);
	execute_cmd_data.opcode = read_op->opcode;
	execute_cmd_data.write_length = read_op->write_length;
	execute_cmd_data.is_uvdm = read_op->is_uvdm;
	memcpy(execute_cmd_data.write_data, read_op->write_data, read_op->write_length);
	execute_cmd_data.seq = OPCODE_RW_SEQ;
	enqueue_usbc_cmd_data(cmd_queue, &execute_cmd_data);

	/* the messages recevied From USBC. */
	init_usbc_cmd_data(&execute_cmd_data);
	execute_cmd_data.response = read_op->opcode;
	execute_cmd_data.read_length = read_op->read_length;
	execute_cmd_data.is_uvdm = read_op->is_uvdm;
	execute_cmd_data.seq = OPCODE_RW_SEQ;
	enqueue_usbc_cmd_data(cmd_queue, &execute_cmd_data);

	/* the messages sent to USBC. */
	init_usbc_cmd_data(&execute_cmd_data);
	execute_cmd_data.opcode = write_op->opcode;
	execute_cmd_data.write_length = write_op->write_length;
	execute_cmd_data.is_uvdm = write_op->is_uvdm;
	memcpy(execute_cmd_data.write_data, write_op->write_data, OPCODE_DATA_LENGTH);
	execute_cmd_data.seq = OPCODE_RW_SEQ;
	enqueue_usbc_cmd_data(cmd_queue, &execute_cmd_data);

	/* the messages recevied From USBC. */
	init_usbc_cmd_data(&execute_cmd_data);
	execute_cmd_data.response = write_op->opcode;
	execute_cmd_data.read_length = write_op->read_length;
	execute_cmd_data.is_uvdm = write_op->is_uvdm;
	execute_cmd_data.seq = OPCODE_RW_SEQ;
	enqueue_usbc_cmd_data(cmd_queue, &execute_cmd_data);

	msg_info("RW->R opcode[0x%02x] write_length[%d] read_length[%d]",
		read_op->opcode, read_op->write_length, read_op->read_length);
	msg_info("RW->W opcode[0x%02x] write_length[%d] read_length[%d]",
		write_op->opcode, write_op->write_length, write_op->read_length);

	front_usbc_cmd(cmd_queue, &current_cmd_data);
	if (current_cmd_data.opcode == read_op->opcode)
		max77978_usbc_cmd_run(usbc_data);
	else {
		msg_info("!!! current_cmd_data.opcode [0x%02x], read_op->opcode[0x%02x]",
			current_cmd_data.opcode, read_op->opcode);
		if (usbc_data->opcode_stamp != 0 && current_cmd_data.opcode == OPCODE_NONE) {
			if (time_after(jiffies,
					usbc_data->opcode_stamp + MAX77978_MAX_APDCMD_TIME)) {
				usbc_data->opcode_stamp = 0;
				msg_info("error. we will dequeue response data");
				max77978_usbc_dequeue_queue(usbc_data);
				max77978_usbc_cmd_run(usbc_data);
			}
		}
	}

	mutex_unlock(&usbc_data->op_lock);

	return 0;
}

static void max77978_reset_ic(struct max77978_usbc_platform_data *usbc_data)
{
	msg_info("IN");
	max77978_write_reg(usbc_data->i2c, REG_RESET,
			   MAX77978_SOFT_RESET_VAL);
	msleep(100); /* need 100ms delay */
	msg_info("OUT");
}

static void max77978_usbc_check_sysmsg(struct max77978_usbc_platform_data *usbc_data, u8 sysmsg)
{
	usbc_cmd_queue_t *cmd_queue = &(usbc_data->cmd_queue);
	bool is_empty_queue = is_empty_usbc_cmd_queue(cmd_queue);
	usbc_cmd_data_t cmd_data;
	usbc_cmd_data_t next_cmd_data;
	u8 next_opcode = 0xFF;
	u8 interrupt = 0;

	if (usbc_data->shut_down) {
		msg_info("ignore the sysmsg during shutdown mode.");
		return;
	}

	switch (sysmsg) {
	case SYSMSG_NONE:
		msg_info("[0x%02X] SYSMSG_NONE", sysmsg);
		break;
	case SYSMSG_BOOT_WDT:
		usbc_data->watchdog_count++;
		msg_info("[0x%02X] SYSMSG_BOOT_WDT (watchdog_count=%d)", sysmsg, usbc_data->watchdog_count);
		max77978_write_reg(usbc_data->i2c, REG_UIC_INT_M, REG_UIC_INT_M_INIT);
		max77978_write_reg(usbc_data->i2c, REG_CC_INT_M, REG_CC_INT_M_INIT);
		max77978_write_reg(usbc_data->i2c, REG_PD_INT_M, REG_PD_INT_M_INIT);
		max77978_write_reg(usbc_data->i2c, REG_ACTION_INT_M, REG_ACTION_INT_M_INIT);
		/* clear UIC_INT to prevent infinite sysmsg irq*/
		max77978_read_reg(usbc_data->i2c, REG_UIC_INT, &interrupt);
		msg_info("UIC_INT:0x%02x", interrupt);
		max77978_usbc_clear_queue(usbc_data);
		break;
	case SYSMSG_BOOT_POR:
		msg_info("[0x%02X] SYSMSG_BOOT_POR", sysmsg);
		if (usbc_data->max77978->boot_complete) {
			usbc_data->por_count++;
			msg_info("[0x%02X] SYSMSG_BOOT_POR(watchdog_count=%d)", sysmsg, usbc_data->por_count);
			max77978_reset_ic(usbc_data);
			max77978_write_reg(usbc_data->i2c, REG_UIC_INT_M, REG_UIC_INT_M_INIT);
			max77978_write_reg(usbc_data->i2c, REG_CC_INT_M, REG_CC_INT_M_INIT);
			max77978_write_reg(usbc_data->i2c, REG_PD_INT_M, REG_PD_INT_M_INIT);
			max77978_write_reg(usbc_data->i2c, REG_ACTION_INT_M, REG_ACTION_INT_M_INIT);
			/* clear UIC_INT to prevent infinite sysmsg irq */
			max77978_read_reg(usbc_data->i2c, REG_UIC_INT, &interrupt);
			msg_info("UIC_INT:0x%02x", interrupt);
			max77978_usbc_clear_queue(usbc_data);
		}
		break;
	case SYSMSG_APCMD_UNKNOWN:
		msg_info("[0x%02X] SYSMSG_APCMD_UNKNOWN", sysmsg);
		break;
	case SYSMSG_APCMD_INPROGRESS:
		msg_info("[0x%02X] SYSMSG_APCMD_INPROGRESS", sysmsg);
		break;
	case SYSMSG_APCMD_FAIL:
		msg_info("[0x%02X] SYSMSG_APCMD_FAIL", sysmsg);
		init_usbc_cmd_data(&cmd_data);
		init_usbc_cmd_data(&next_cmd_data);

		if (front_usbc_cmd(cmd_queue, &next_cmd_data))
			next_opcode = next_cmd_data.response;

		if (!is_empty_queue) {
			copy_usbc_cmd_data(&(usbc_data->last_opcode), &cmd_data);

			if (next_opcode == OPCODE_SEND_VDM) {
				usbc_data->opcode_stamp = 0;
				max77978_usbc_dequeue_queue(usbc_data);
				cmd_data.opcode = OPCODE_NONE;
			}

			if ((cmd_data.opcode != OPCODE_NONE) && (cmd_data.opcode == next_opcode)) {
				if (next_opcode != OPCODE_SEND_VDM) {
					int ret = max77978_i2c_opcode_write(usbc_data, cmd_data.opcode, cmd_data.write_length, cmd_data.write_data);
					if (ret) {
						msg_err("i2c write fail. dequeue opcode");
						max77978_usbc_dequeue_queue(usbc_data);
					} else
						msg_info("RETRY SUCCESS : %x, %x", cmd_data.opcode, next_opcode);
				} else
					msg_info("IGNORE COMMAND : %x, %x", cmd_data.opcode, next_opcode);
			} else {
				msg_info("RETRY FAILED : %x, %x", cmd_data.opcode, next_opcode);
			}

		}
		break;
	default:
		msg_info("[0x%02X] SYSMSG not supported", sysmsg);
		break;
	}
}

static irqreturn_t max77978_apcmd_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	u8 sysmsg = 0;

	msg_irq_handle_start(irq);

	max77978_read_reg(usbc_data->i2c, REG_USBC_STATUS2, &usbc_data->usbc_status2);
	sysmsg = usbc_data->usbc_status2;
	msg_info("[IN] sysmsg : %d", sysmsg);
	mutex_lock(&usbc_data->op_lock);
	max77978_usbc_cmd_run(usbc_data);
	mutex_unlock(&usbc_data->op_lock);

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

static irqreturn_t max77978_sysmsg_irq(int irq, void *data)
{
	struct max77978_usbc_platform_data *usbc_data = data;
	u8 sysmsg = 0;
	u8 i = 0;
	u8 raw_data[3] = {0, };
	u8 usbc_status2 = 0;

	msg_irq_handle_start(irq);

	for (i = 0; i < 3; i++) {
		usbc_status2 = 0;
		max77978_read_reg(usbc_data->i2c, REG_USBC_STATUS2, &usbc_status2);
		raw_data[i] = usbc_status2;
	}
	if ((raw_data[0] == raw_data[1]) && (raw_data[0] == raw_data[2])) {
		sysmsg = raw_data[0];
	} else {
		msg_info("[W]sysmsg was changed suddenly, [0]0x%02X, [1]0x%02X, [2]0x%02X", raw_data[0], raw_data[1], raw_data[2]);
		sysmsg = raw_data[2];
	}
	max77978_usbc_check_sysmsg(usbc_data, sysmsg);
	usbc_data->sysmsg = sysmsg;

	msg_irq_handle_complete(irq);
	return IRQ_HANDLED;
}

static int max77978_init_irq_handler(struct max77978_usbc_platform_data *usbc_data)
{
	int ret = 0;

	usbc_data->irq_apcmd = usbc_data->irq_base + MAX77978_UIC_IRQ_APC_INT;
	if (usbc_data->irq_apcmd) {
		ret = request_threaded_irq(usbc_data->irq_apcmd,
			   NULL, max77978_apcmd_irq,
			   IRQF_ONESHOT, "usbc-apcmd-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_UIC_IRQ_APC_INT, ret);
			return ret;
		}
	}

	usbc_data->irq_sysmsg = usbc_data->irq_base + MAX77978_UIC_IRQ_SYSM_INT;
	if (usbc_data->irq_sysmsg) {
		ret = request_threaded_irq(usbc_data->irq_sysmsg,
				NULL, max77978_sysmsg_irq,
				IRQF_ONESHOT, "usbc-sysmsg-irq", usbc_data);
		if (ret) {
			msg_irq_request_failed(MAX77978_UIC_IRQ_SYSM_INT, ret);
			free_irq(usbc_data->irq_apcmd, usbc_data);
			return ret;
		}
	}

	return ret;
}

static void max77978_usbc_free_irq(struct max77978_usbc_platform_data *usbc_data)
{
	free_irq(usbc_data->irq_apcmd, usbc_data);
	free_irq(usbc_data->irq_sysmsg, usbc_data);
	free_irq(usbc_data->pd_data->irq_pdmsg, usbc_data);
	free_irq(usbc_data->pd_data->irq_psrdy, usbc_data);
	free_irq(usbc_data->pd_data->irq_datarole, usbc_data);
	free_irq(usbc_data->pd_data->irq_vdm, usbc_data);
	free_irq(usbc_data->cc_data->irq_vconnocp, usbc_data);
	free_irq(usbc_data->cc_data->irq_vsafe0v, usbc_data);
	free_irq(usbc_data->cc_data->irq_vconnsc, usbc_data);
	free_irq(usbc_data->cc_data->irq_ccpinstat, usbc_data);
	free_irq(usbc_data->cc_data->irq_ccistat, usbc_data);
	free_irq(usbc_data->cc_data->irq_ccvcnstat, usbc_data);
	free_irq(usbc_data->cc_data->irq_ccstat, usbc_data);
	free_irq(usbc_data->bc12_data->irq_vbusdet, usbc_data);
	free_irq(usbc_data->bc12_data->irq_dcdtmo, usbc_data);
	free_irq(usbc_data->bc12_data->irq_chgtype, usbc_data);
#ifdef USE_UIC_IRQ_VBADC
	free_irq(usbc_data->bc12_data->irq_vbadc, usbc_data);
#else
	free_irq(usbc_data->bc12_data->irq_vbadch, usbc_data);
	free_irq(usbc_data->bc12_data->irq_vbadcl, usbc_data);
#endif
}

int max77978_usbc_init(struct max77978_dev *max77978)
{
	struct max77978_platform_data *pdata = dev_get_platdata(max77978->dev);
	struct max77978_usbc_platform_data *usbc_data = NULL;
	int ret = 0;

	msg_info("init start with irq %d", max77978->irq);
	usbc_data = kzalloc(sizeof(struct max77978_usbc_platform_data), GFP_KERNEL);
	if (!usbc_data) {
		msg_err("usbc_data is null");
		return -ENOMEM;
	}

	g_usbc_data = usbc_data;
	usbc_data->dev = max77978->dev;
	usbc_data->max77978 = max77978;
	usbc_data->i2c = max77978->i2c;
	usbc_data->max77978_data = pdata;
	usbc_data->irq_base = pdata->irq_base;

	usbc_data->pd_data = devm_kzalloc(max77978->dev, sizeof(*usbc_data->pd_data), GFP_KERNEL);
	if (!usbc_data->pd_data) {
		msg_err("pd_data is null");
		ret = -ENOMEM;
		goto err_nomem;
	}

	usbc_data->cc_data = devm_kzalloc(max77978->dev, sizeof(*usbc_data->cc_data), GFP_KERNEL);
	if (!usbc_data->cc_data) {
		msg_err("cc_data is null");
		ret = -ENOMEM;
		goto err_nomem;
	}

	usbc_data->bc12_data = devm_kzalloc(max77978->dev, sizeof(*usbc_data->bc12_data), GFP_KERNEL);
	if (!usbc_data->bc12_data) {
		msg_err("bc12_data is null");
		ret = -ENOMEM;
		goto err_nomem;
	}


#if defined(CONFIG_MAX77978_DEBUG)
	mxim_debug_init();
	mxim_debug_set_i2c_client(usbc_data->i2c);
#endif

	usbc_data->device_id = 0x0;
	usbc_data->fw_revision = 0x0;
	usbc_data->fw_revision = 0x0;
	usbc_data->plug_attach_done = 0x0;
	usbc_data->cc_data->current_port_type = 0xFF;
	usbc_data->pd_data->current_port_data = 0xFF;
	usbc_data->cc_data->current_vcon = 0xFF;
	usbc_data->current_connstat = 0xFF;
	usbc_data->prev_connstat = 0xFF;
	usbc_data->cmd_queue.front = NULL;
	usbc_data->cmd_queue.rear = NULL;
	usbc_data->opcode_stamp = 0;
	mutex_init(&usbc_data->op_lock);

	usbc_data->typec_cap.revision = USB_TYPEC_REV_1_2;
	usbc_data->typec_cap.pd_revision = 0x300;
	usbc_data->typec_cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;

	usbc_data->typec_cap.driver_data = usbc_data;
	usbc_data->typec_cap.ops = &max77978_ops;

	usbc_data->typec_cap.type = TYPEC_PORT_DRP;
	usbc_data->typec_cap.data = TYPEC_PORT_DRD;

	usbc_data->typec_power_role = TYPEC_SINK;
	usbc_data->typec_try_state_change = TRY_ROLE_SWAP_NONE;

	usbc_data->port = typec_register_port(usbc_data->dev, &usbc_data->typec_cap);
	if (IS_ERR(usbc_data->port)) {
		msg_err("unable to register typec_register_port");
		ret = PTR_ERR(usbc_data->port);
		usbc_data->port = NULL;
		goto err_port;
	}
	msg_info("success typec_register_port port=%pK", usbc_data->port);
	init_completion(&usbc_data->typec_reverse_completion);

	usbc_data->por_count = 0;
	usbc_data->pd_support = false;
	usbc_data->alternate_state = 0;
	usbc_data->dp_is_connect = 0;
	usbc_data->dp_hs_connect = 0;
	usbc_data->is_sent_pin_configuration = 0;

	INIT_DELAYED_WORK(&usbc_data->vbus_hard_reset_work, vbus_control_hard_reset);

	max77978_get_version_info(usbc_data);
	max77978_init_irq_handler(usbc_data);

	max77978_cc_init(usbc_data);
	max77978_bc12_init(usbc_data);
	max77978_pd_init(usbc_data);
	enable_irq(max77978->irq);
	max77978->boot_complete = 1;
	msg_info("done (irq_apcmd=%d irq_sysmsg=%d)", usbc_data->irq_apcmd, usbc_data->irq_sysmsg);

	return 0;

err_port:
	mutex_destroy(&usbc_data->op_lock);
err_nomem:
	/* pd_data, cc_data, bc12_data, chg_data are allocated with devm_kzalloc,
	 * they will be automatically freed when the device is removed.
	 * Only usbc_data needs to be freed here.
	 */
	kfree(usbc_data);
	g_usbc_data = NULL;

	return ret;
}

void max77978_usbc_deinit(struct max77978_usbc_platform_data *usbc_data)
{
	if (!usbc_data)
		return;

	/* Clear any pending commands in the queue */
	max77978_usbc_clear_queue(usbc_data);

	max77978_usbc_free_irq(usbc_data);
	if (usbc_data->port)
		typec_unregister_port(usbc_data->port);
	mutex_destroy(&usbc_data->op_lock);

#if defined(CONFIG_MAX77978_DEBUG)
	mxim_debug_exit();
#endif

	kfree(usbc_data);
	g_usbc_data = NULL;
}

static void max77978_usbc_disable_irq(struct max77978_usbc_platform_data *usbc_data)
{
	disable_irq(usbc_data->irq_apcmd);
	disable_irq(usbc_data->irq_sysmsg);
	disable_irq(usbc_data->pd_data->irq_pdmsg);
	disable_irq(usbc_data->pd_data->irq_psrdy);
	disable_irq(usbc_data->pd_data->irq_datarole);
	disable_irq(usbc_data->pd_data->irq_vdm);
	disable_irq(usbc_data->cc_data->irq_vconnocp);
	disable_irq(usbc_data->cc_data->irq_vsafe0v);
	disable_irq(usbc_data->cc_data->irq_vconnsc);
	disable_irq(usbc_data->cc_data->irq_ccpinstat);
	disable_irq(usbc_data->cc_data->irq_ccistat);
	disable_irq(usbc_data->cc_data->irq_ccvcnstat);
	disable_irq(usbc_data->cc_data->irq_ccstat);
	disable_irq(usbc_data->bc12_data->irq_vbusdet);
	disable_irq(usbc_data->bc12_data->irq_dcdtmo);
#ifdef USE_UIC_IRQ_VBADC
	disable_irq(usbc_data->bc12_data->irq_vbadc);
#else
	disable_irq(usbc_data->bc12_data->irq_vbadch);
	disable_irq(usbc_data->bc12_data->irq_vbadcl);
#endif
}

void max77978_usbc_shutdown(struct max77978_usbc_platform_data *usbc_data)
{
	if (!usbc_data)
		return;

	msg_info("shutdown start");
	if (!usbc_data->i2c) {
		msg_err("no max77978 i2c client");
		return;
	}

	/* mask all interrupts */
	max77978_write_reg(usbc_data->i2c, REG_PD_INT_M, 0xFF);
	max77978_write_reg(usbc_data->i2c, REG_CC_INT_M, 0xFF);
	max77978_write_reg(usbc_data->i2c, REG_UIC_INT_M, 0xFF);
	max77978_write_reg(usbc_data->i2c, REG_ACTION_INT_M, 0xFF);

	max77978_usbc_disable_irq(usbc_data);
	max77978_usbc_free_irq(usbc_data);
	usbc_data->shut_down = 1;

	/* send the reset command */
	if (usbc_data->current_connstat == WATER)
		msg_info("Skip the max77978_reset_ic function");
	else {
		max77978_reset_ic(usbc_data);
		max77978_write_reg(usbc_data->i2c, REG_PD_INT_M, 0xFF);
		max77978_write_reg(usbc_data->i2c, REG_CC_INT_M, 0xFF);
		max77978_write_reg(usbc_data->i2c, REG_UIC_INT_M, 0xFF);
		max77978_write_reg(usbc_data->i2c, REG_ACTION_INT_M, 0xFF);
	}
	msg_info("shutdown done");
}

MODULE_DESCRIPTION("max77978 USBC driver");
MODULE_AUTHOR("Analog Device Inc.");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.2.1");
