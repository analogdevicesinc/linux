/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024 NXP
 */

#ifndef SE_MU_H
#define SE_MU_H

#include <linux/miscdevice.h>
#include <linux/semaphore.h>
#include <linux/mailbox_client.h>
#include <uapi/linux/se_ioctl.h>

#include "se_msg_sqfl_ctrl.h"

#define MAX_FW_LOAD_RETRIES		50

#define RES_STATUS(x)			FIELD_GET(0x000000ff, x)
#define MAX_DATA_SIZE_PER_USER		(65 * 1024)
#define MAX_NVM_MSG_LEN			(256)
#define MESSAGING_VERSION_2		0x2
#define MESSAGING_VERSION_6		0x6
#define MESSAGING_VERSION_7		0x7
#define NODE_NAME			"secure-enclave"

#define GET_ASCII_TO_U8(diff, tens_chr, ones_chr) \
		((diff > 2) ? (((tens_chr - '0') * 10) + (ones_chr - '0')) :\
		(tens_chr - '0'))

#define GET_IDX_FROM_DEV_NODE_NAME(dev_of_node) \
		((strlen(dev_of_node->full_name) > strlen(NODE_NAME)) ?\
		GET_ASCII_TO_U8((strlen(dev_of_node->full_name) - strlen(NODE_NAME)),\
				dev_of_node->full_name[strlen(NODE_NAME) + 1], \
				dev_of_node->full_name[strlen(NODE_NAME) + 2]) : 0)

struct se_clbk_handle {
	struct completion done;
	bool signal_rcvd;
	struct se_if_device_ctx *dev_ctx;
	u32 rx_msg_sz;
	/* Assignment of the rx_msg buffer to held till the
	 * received content as part callback function, is copied.
	 */
	struct se_api_msg *rx_msg;
};

struct se_imem_buf {
	u8 *buf;
	phys_addr_t phyaddr;
	u32 size;
	u32 state;
};

struct se_buf_desc {
	u8 *shared_buf_ptr;
	void __user *usr_buf_ptr;
	u32 size;
	struct list_head link;
};

struct se_shared_mem {
	dma_addr_t dma_addr;
	u32 size;
	u32 pos;
	u8 *ptr;
};

struct se_shared_mem_mgmt_info {
	struct list_head pending_in;
	struct list_head pending_out;

	struct se_shared_mem secure_mem;
	struct se_shared_mem non_secure_mem;
};

/* Private struct for each char device instance. */
struct se_if_device_ctx {
	struct se_if_priv *priv;
	struct miscdevice *miscdev;
	const char *devname;

	struct mutex fops_lock;

	struct se_shared_mem_mgmt_info se_shared_mem_mgmt;
	struct list_head link;
};

/* Header of the messages exchange with the EdgeLock Enclave */
struct se_msg_hdr {
	u8 ver;
	u8 size;
	u8 command;
	u8 tag;
}  __packed;

#define SE_MU_HDR_SZ	4

struct se_api_msg {
	struct se_msg_hdr header;
	u32 data[];
};

struct se_if_defines {
	const u8 se_if_type;
	const u8 se_instance_id;
	u8 cmd_tag;
	u8 rsp_tag;
	u8 success_tag;
	u8 base_api_ver;
	u8 fw_api_ver;
};

struct se_lg_fl_info {
	loff_t offset;
	struct file *lg_file;
	struct path root;
};

struct se_if_priv {
	struct list_head priv_data;
	struct device *dev;

	struct se_clbk_handle cmd_receiver_clbk_hdl;
	/* Update to the waiting_rsp_dev, to be protected
	 * under se_if_cmd_lock.
	 */
	struct se_clbk_handle waiting_rsp_clbk_hdl;
	/*
	 * prevent new command to be sent on the se interface while previous
	 * command is still processing. (response is awaited)
	 */
	struct mutex se_if_cmd_lock;
	struct se_msg_seq_ctrl se_msg_sq_ctl;

	struct mbox_client se_mb_cl;
	struct mbox_chan *tx_chan, *rx_chan;

	uint32_t flags;
	struct se_shared_mem mu_mem;
	struct gen_pool *mem_pool;
	const struct se_if_defines *if_defs;
	struct se_lg_fl_info lg_fl_info;

	struct imx_sc_ipc *ipc_scu;
	u8 part_owner;
	struct se_if_device_ctx *priv_dev_ctx;
	struct list_head dev_ctx_list;
	u32 active_devctx_count;
	u32 dev_ctx_mono_count;

	struct se_time_frame time_frame;
};

#define SE_DUMP_IOCTL_BUFS	0
#define SE_DUMP_MU_SND_BUFS	1
#define SE_DUMP_MU_RCV_BUFS	2
#define SE_DUMP_KDEBUG_BUFS	3

char *get_se_if_name(u8 se_if_id);
uint32_t get_se_soc_id(struct se_if_priv *priv);
int se_dump_to_logfl(struct se_if_device_ctx *dev_ctx,
		     u8 caller_type, int buf_size,
		     const char *buf, ...);
#endif
