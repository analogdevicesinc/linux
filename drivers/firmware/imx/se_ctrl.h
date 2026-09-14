/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2026 NXP
 */

#ifndef SE_CTRL_H
#define SE_CTRL_H

#include <linux/bitfield.h>
#include <linux/miscdevice.h>
#include <linux/mailbox_client.h>
#include <linux/semaphore.h>

#define MAX_FW_LOAD_RETRIES		50
#define SE_MSG_WORD_SZ			0x4

#define RES_STATUS(x)			FIELD_GET(0x000000ff, x)
#define MAX_NVM_MSG_LEN			(256)
#define MESSAGING_VERSION_6		0x6
#define MESSAGING_VERSION_7		0x7

struct se_clbk_handle {
	struct se_if_device_ctx *dev_ctx;
	struct completion done;
	bool signal_rcvd;
	u32 rx_msg_sz;
	/*
	 * Assignment of the rx_msg buffer to held till the
	 * received content as part callback function, is copied.
	 */
	struct se_api_msg *rx_msg;
	/*
	 * Serialise the timeout path in ele_msg_rcv() against
	 * se_if_rx_callback() so that the callback can never
	 * memcpy into a buffer that the timeout path has already
	 * freed.
	 */
	spinlock_t clbk_rx_lock;
};

struct se_imem_buf {
	u8 *buf;
	dma_addr_t daddr;
	u32 size;
	u32 state;
};

/* Private struct for each char device instance. */
struct se_if_device_ctx {
	struct se_if_priv *priv;
	const char *devname;
};

/* Header of the messages exchange with the EdgeLock Enclave */
struct se_msg_hdr {
	u8 ver;
	u8 size;
	u8 command;
	u8 tag;
}  __packed;

#define SE_MU_HDR_SZ		4
#define SE_MU_HDR_WORD_SZ	1

struct se_api_msg {
	struct se_msg_hdr header;
	u32 data[];
};

struct se_if_defines {
	const u8 se_if_type;
	u8 cmd_tag;
	u8 rsp_tag;
	u8 success_tag;
	u8 base_api_ver;
	u8 fw_api_ver;
};

struct se_fw_img_name {
	const char *prim_fw_nm_in_rfs;
	const char *seco_fw_nm_in_rfs;
};

struct se_fw_load_info {
	const struct se_fw_img_name *se_fw_img_nm;
	bool is_fw_tobe_loaded;
	bool imem_mgmt;
	struct se_imem_buf imem;
	/* to serialize the fw load state */
	struct mutex load_fw_lock;
};

struct se_if_priv {
	struct device *dev;

	struct se_clbk_handle cmd_receiver_clbk_hdl;
	/*
	 * Update to the waiting_rsp_dev, to be protected
	 * under se_if_cmd_lock.
	 */
	struct se_clbk_handle waiting_rsp_clbk_hdl;
	/*
	 * prevent new command to be sent on the se interface while previous
	 * command is still processing. (response is awaited)
	 */
	struct mutex se_if_cmd_lock;

	struct mbox_client se_mb_cl;
	struct mbox_chan *tx_chan, *rx_chan;

	struct gen_pool *mem_pool;
	const struct se_if_defines *if_defs;
	struct se_fw_load_info load_fw;

	atomic_t fw_busy;

	struct se_if_device_ctx *priv_dev_ctx;
};

char *get_se_if_name(u8 se_if_id);
#endif
