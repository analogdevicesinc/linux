/* SPDX-License-Identifier: (GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause*/
/*
 * Copyright 2024 NXP
 */

#ifndef SE_IOCTL_H
#define SE_IOCTL_H

#ifdef __KERNEL__
#include <linux/time64.h>
#else
#include <linux/time_types.h>
#endif
#include <linux/types.h>

#define SE_TYPE_STR_DBG			"dbg"
#define SE_TYPE_STR_HSM			"hsm"
#define SE_TYPE_STR_SHE			"she"
#define SE_TYPE_STR_V2X_SV		"v2x_sv"
#define SE_TYPE_STR_V2X_SG		"v2x_sg"
#define SE_TYPE_STR_V2X_SHE		"v2x_she"
#define SE_TYPE_STR_V2X_DBG		"v2x_dbg"

#define SE_TYPE_ID_UNKWN		0x0
#define SE_TYPE_ID_DBG			0x1
#define SE_TYPE_ID_HSM			0x2
#define SE_TYPE_ID_SHE			0x3
#define SE_TYPE_ID_V2X_DBG		0x4
#define SE_TYPE_ID_V2X_SHE		0x5
#define SE_TYPE_ID_V2X_SV		0x6
#define SE_TYPE_ID_V2X_SG		0x7

/* IOCTL definitions. */

struct se_ioctl_setup_iobuf {
	void __user *user_buf;
	__u32 length;
	__u32 flags;
	__u64 ele_addr;
};

struct se_ioctl_shared_mem_cfg {
	__u32 base_offset;
	__u32 size;
};

struct se_ioctl_get_if_info {
	__u8 se_if_id;
	__u8 interrupt_idx;
	__u8 tz;
	__u8 did;
	__u8 cmd_tag;
	__u8 rsp_tag;
	__u8 success_tag;
	__u8 base_api_ver;
	__u8 fw_api_ver;
};

struct se_ioctl_cmd_snd_rcv_rsp_info {
	__u32 __user *tx_buf;
	int tx_buf_sz;
	__u32 __user *rx_buf;
	int rx_buf_sz;
};

struct se_ioctl_get_soc_info {
	__u16 soc_id;
	__u16 soc_rev;
	__u8  board_type;
};

struct se_ioctl_signed_message {
	__u8 *message;
	__u32 msg_size;
	__u32 error_code;
};

struct se_time_frame {
#ifdef __KERNEL__
	struct timespec64 t_start;
	struct timespec64 t_end;
#else
	struct __kernel_timespec t_start;
	struct __kernel_timespec t_end;
#endif
};

/* IO Buffer Flags */
#define SE_IO_BUF_FLAGS_IS_OUTPUT	(0x00u)
#define SE_IO_BUF_FLAGS_IS_INPUT	(0x01u)
#define SE_IO_BUF_FLAGS_USE_SEC_MEM	(0x02u)
#define SE_IO_BUF_FLAGS_USE_SHORT_ADDR	(0x04u)
#define SE_IO_BUF_FLAGS_USE_MU_BUF	(0x08u)
#define SE_IO_BUF_FLAGS_IS_IN_OUT	(0x10u)

/* IOCTLS */
#define SE_IOCTL			0x0A /* like MISC_MAJOR. */

/*
 * ioctl to designated the current fd as logical-reciever.
 * This is ioctl is send when the nvm-daemon, a slave to the
 * firmware is started by the user.
 */
#define SE_IOCTL_ENABLE_CMD_RCV	_IO(SE_IOCTL, 0x01)

/*
 * ioctl to get configure the SCU shared buffer.
 */
#define SE_IOCTL_SHARED_BUF_CFG	_IOW(SE_IOCTL, 0x02, \
				struct se_ioctl_shared_mem_cfg)
/*
 * ioctl to get the buffer allocated from the memory, which is shared
 * between kernel and FW.
 * Post allocation, the kernel tagged the allocated memory with:
 *  Output
 *  Input
 *  Input-Output
 *  Short address
 *  Secure-memory
 */
#define SE_IOCTL_SETUP_IOBUF	_IOWR(SE_IOCTL, 0x03, \
					struct se_ioctl_setup_iobuf)

/*
 * ioctl to get the mu information, that is used to exchange message
 * with FW, from user-spaced.
 */
#define SE_IOCTL_GET_MU_INFO	_IOR(SE_IOCTL, 0x04, \
					struct se_ioctl_get_if_info)

/*
 * ioctl to send signed message to SE.
 */
#define SE_IOCTL_SIGNED_MESSAGE     _IOR(SE_IOCTL, 0x05, \
					struct se_ioctl_signed_message)

/*
 * ioctl to get SoC Info from user-space.
 */
#define SE_IOCTL_GET_SOC_INFO      _IOR(SE_IOCTL, 0x06, \
					struct se_ioctl_get_soc_info)

/*
 * ioctl to send command and receive response from user-space.
 */
#define SE_IOCTL_CMD_SEND_RCV_RSP _IOWR(SE_IOCTL, 0x07, \
					struct se_ioctl_cmd_snd_rcv_rsp_info)

/*
 * ioctl to capture the timestamp at the request to FW and response from FW
 * for a crypto operation
 */
#define SE_IOCTL_GET_TIMER	_IOR(SE_IOCTL, 0x08, struct se_time_frame)

#endif
