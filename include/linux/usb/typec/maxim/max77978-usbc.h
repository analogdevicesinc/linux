/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * MAX77978 USB Type-C Controller Header
 *
 * Copyright (c) 2024 Analog Devices, Inc.
 */

#ifndef __MAX77978_USBC_H__
#define __MAX77978_USBC_H__
#include <linux/usb/typec.h>

#include <linux/usb/typec/maxim/max77978.h>
#include <linux/usb/typec/maxim/max77978-pd.h>
#include <linux/usb/typec/maxim/max77978-cc.h>
#include <linux/usb/typec/maxim/max77978-bc12.h>
#define MAX77978_MAX_APDCMD_TIME (7*HZ)

struct max77978_opcode {
	unsigned char opcode;
	unsigned char data[OPCODE_DATA_LENGTH];
	int read_length;
	int write_length;
};

#define REG_NONE			0xff

#define DISABLED 0
#define ENABLED 1

#define TRYNONE	0
#define TRYSNK	1
#define TRYSRC	2

#define MOISTUREDETECTIONDISABLE 0
#define MOISTUREDETECTIONENABLE 1

#define MEMORYUPDATERAM 0
#define MEMORYUPDATEMTP 1

#define PD_VID 0x0B6A
#define PD_PID 0x6860

#define I2C_SID0 0x69
#define I2C_SID1 0x62
#define I2C_SID2 0x64
#define I2C_SID3 0x28

#define FAST_ROLESWAP 0
#define RPVALUE 0x0

union max77978_custm_info_pass1 {
	uint8_t CustmInfoArry[10];

	struct {
		uint8_t debugSRC : 1;
		uint8_t debugSNK : 1;
		uint8_t trysnksrc : 2;
		uint8_t typecstate : 2;
		uint8_t memoryupdate : 1;
		uint8_t moisturedetection : 1;

		uint16_t VID : 16;
		uint16_t PID : 16;

		uint32_t i2cSID1: 8;
		uint32_t i2cSID2: 8;
		uint32_t i2cSID3: 8;
		uint32_t i2cSID4: 8;

		uint8_t fastroleswap : 1;
		uint8_t rpvalue : 3;
		uint8_t reserved1 : 4;
	} custm_bits;
} __packed;

typedef struct max77978_usbc_command_data {
	u8	opcode;
	u8  prev_opcode;
	u8	response;
	u8	read_data[OPCODE_DATA_LENGTH];
	u8	write_data[OPCODE_DATA_LENGTH];
	int read_length;
	int write_length;
	u8	reg;
	u8	val;
	u8	mask;
	u8  seq;
	int noti_cmd;
	u8	is_uvdm;
} usbc_cmd_data_t;

typedef struct max77978_usbc_command_node {
	struct max77978_usbc_command_data cmd_data;
	struct max77978_usbc_command_node *next;
} usbc_cmd_node_t;

typedef struct max77978_usbc_command_queue {
	struct mutex	command_mutex;
	usbc_cmd_node_t	*front;
	usbc_cmd_node_t	*rear;
} usbc_cmd_queue_t;

struct max77978_usbc_platform_data {
	struct max77978_dev *max77978;
	struct device *dev;
	struct i2c_client *i2c;

	int irq_base;

	/* interrupt pin */
	int irq_apcmd;
	int irq_sysmsg;

	/* register information */
	u8 usbc_status1;
	u8 usbc_status2;
	u8 bc_status;
	u8 cc_status0;
	u8 cc_status1;
	u8 pd_status0;
	u8 pd_status1;

	int watchdog_count;
	int por_count;

	u8 opcode_res;
	/* USBC System message interrupt */
	u8 sysmsg;
	u8 pd_msg;

	/* F/W state */
	u8 device_id;
	u8 device_revision;
	u8 fw_revision;
	u8 fw_minor_revision;
	u8 plug_attach_done;

	enum max77978_connstat prev_connstat;
	enum max77978_connstat current_connstat;

	struct max77978_usbc_command_queue apcmd_queue;
	usbc_cmd_data_t last_opcode;
	unsigned long opcode_stamp;
	struct mutex op_lock;

	usbc_cmd_queue_t cmd_queue;

	uint32_t alternate_state;
	uint32_t acc_type;
	uint32_t vendor_id;
	uint32_t product_id;
	uint32_t device_version;
	uint32_t svid_0;
	uint32_t svid_1;
	uint32_t svid_dp;
	uint32_t dp_is_connect;
	uint32_t dp_hs_connect;
	uint32_t dp_selected_pin;
	u8 pin_assignment;
	uint32_t is_sent_pin_configuration;
	int role_swap_type;

	struct max77978_bc12_data *bc12_data;
	struct max77978_pd_data *pd_data;
	struct max77978_cc_data *cc_data;
	struct max77978_platform_data *max77978_data;

	int is_host;
	int is_client;

	struct typec_port *port;
	struct typec_partner *partner;

	struct typec_capability typec_cap;
	struct completion typec_reverse_completion;
	enum typec_role typec_power_role;
	enum typec_data_role typec_data_role;
	int typec_try_state_change;
	int pwr_opmode;
	bool pd_support;
	int pd_pr_swap;
	int shut_down;
	struct delayed_work vbus_hard_reset_work;
	uint8_t ReadMSG[32];

	bool srccap_request_retry;

};

typedef enum {
	OPCODE_NOTI_START	= 0x00,
	OPCODE_NOTI_NONE	= 0xFF,
} max77978_opcode_noti_cmd_e;

typedef enum {
	OPCODE_ID_VDM_DISCOVER_IDENTITY = 0x1,
	OPCODE_ID_VDM_DISCOVER_SVIDS = 0x2,
	OPCODE_ID_VDM_DISCOVER_MODES = 0x3,
	OPCODE_ID_VDM_ENTER_MODE = 0x4,
	OPCODE_ID_VDM_EXIT_MODE = 0x5,
	OPCODE_ID_VDM_ATTENTION = 0x6,
	OPCODE_ID_VDM_SVID_DP_STATUS = 0x10,
	OPCODE_ID_VDM_SVID_DP_CONFIGURE = 0x11,
} max77978_vdm_list;

#define TRY_ROLE_SWAP_WAIT_MS   (5000)
enum {
	TRY_ROLE_SWAP_NONE = 0,
	TRY_ROLE_SWAP_PR = 1,	/* pr_swap */
	TRY_ROLE_SWAP_DR = 2,	/* dr_swap */
	TRY_ROLE_SWAP_TYPE = 3, /* type */
	TRY_ROLE_SWAP_VC = 4, /* vconn swap */
};
union VDM_HEADER_Type {
	uint32_t        DATA;
	struct {
		uint8_t     BDATA[4];
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
};

#define VDM_RESPONDER_ACK	0x1
#define VDM_RESPONDER_NAK	0x2
#define VDM_RESPONDER_BUSY	0x3

#define DATA_ROLE_SWAP			1
#define POWER_ROLE_SWAP			2
#define VCONN_ROLE_SWAP			3
#define MANUAL_ROLE_SWAP		4
#define DATA_ROLE_SWAP_RETRY	5
#define ROLE_ACCEPT				0x1
#define ROLE_REJECT				0x2
#define ROLE_BUSY				0x3

void init_usbc_cmd_data(struct max77978_usbc_command_data *cmd_data);
bool is_empty_usbc_cmd_queue(usbc_cmd_queue_t *cmd_queue);

void max77978_usbc_clear_queue(struct max77978_usbc_platform_data *usbc_data);

int max77978_usbc_opcode_rw(struct max77978_usbc_platform_data *usbc_data, usbc_cmd_data_t *opcode_r, usbc_cmd_data_t *opcode_w);
int max77978_usbc_opcode_write(struct max77978_usbc_platform_data *usbc_data, usbc_cmd_data_t *write_op);
int max77978_usbc_opcode_read(struct max77978_usbc_platform_data *usbc_data, usbc_cmd_data_t *read_op);
int max77978_usbc_opcode_push(struct max77978_usbc_platform_data *usbc_data, usbc_cmd_data_t *read_op);

#define VBUS_OFF	(0)
#define VBUS_ON		(1)
void max77978_vbus_turn_on_ctrl(struct max77978_usbc_platform_data *usbc_data, bool enable);
void max77978_notify_dr_status(struct max77978_usbc_platform_data *usbpd_data, uint8_t attach);

int max77978_cc_init(struct max77978_usbc_platform_data *usbc_data);
int max77978_bc12_init(struct max77978_usbc_platform_data *usbc_data);
int max77978_pd_init(struct max77978_usbc_platform_data *usbc_data);
void max77978_detach_pd(struct max77978_usbc_platform_data *usbc_data);
void max77978_current_pdo(struct max77978_usbc_platform_data *usbc_data, unsigned char *data);
void __maybe_unused max77978_select_pdo(void *data, int num);

#define msg_info(format, args...)	pr_info("%s: " format "\n", __func__, ## args)
#define msg_err(format, args...)	pr_err("%s: " format "\n", __func__, ## args)
#define msg_debug(format, args...)	pr_debug("%s: " format "\n", __func__, ## args)

//#define msg_irq_handle_start(irq, format, args...)	pr_info("%s : IRQ(%d)_start" format "\n", __func__, irq, ## args)
//#define msg_irq_handle_done(irq, format, args...)	pr_info("%s : IRQ(%d)_complete" format "\n", __func__, irq, ## args)
#define msg_irq_handle_start(irq)			pr_info("%s: irq(%d) +++\n", __func__, irq)
#define msg_irq_handle_complete(irq)		pr_info("%s: irq(%d) ---\n", __func__, irq)
#define msg_irq_request_failed(irq, err)	pr_err("%s: irq(%s)_request_failed with err=%d\n", __func__, #irq, err)

int max77978_get_pd_support(struct max77978_usbc_platform_data *usbc_data);

/* Driver init/deinit functions */
struct max77978_dev;
int max77978_usbc_init(struct max77978_dev *max77978);
void max77978_usbc_deinit(struct max77978_usbc_platform_data *usbc_data);
void max77978_usbc_shutdown(struct max77978_usbc_platform_data *usbc_data);

/* Global USBC data pointer */
extern struct max77978_usbc_platform_data *g_usbc_data;

#endif /* __MAX77978_USBC_H__ */
