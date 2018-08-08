/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 ******************************************************************************
 * communication stack of audio with rpmsg
 ******************************************************************************
 * Packet structure:
 *   A SRTM message consists of a 10 bytes header followed by 0~N bytes of data
 *
 *   +---------------+-------------------------------+
 *   |               |            Content            |
 *   +---------------+-------------------------------+
 *   |  Byte Offset  | 7   6   5   4   3   2   1   0 |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       0       |           Category            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |     1 ~ 2     |           Version             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       3       |             Type              |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       4       |           Command             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       5       |           Reserved0           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       6       |           Reserved1           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       7       |           Reserved2           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       8       |           Reserved3           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       9       |           Reserved4           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       10      |            DATA 0             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   :   :   :   :   :   :   :   :   :   :   :   :   :
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |   N + 10 - 1  |            DATA N-1           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *
 *   +----------+------------+------------------------------------------------+
 *   |  Field   |    Byte    |                                                |
 *   +----------+------------+------------------------------------------------+
 *   | Category |     0      | The destination category.                      |
 *   +----------+------------+------------------------------------------------+
 *   | Version  |   1 ~ 2    | The category version of the sender of the      |
 *   |          |            | packet.                                        |
 *   |          |            | The first byte represent the major version of  |
 *   |          |            | the packet.The second byte represent the minor |
 *   |          |            | version of the packet.                         |
 *   +----------+------------+------------------------------------------------+
 *   |  Type    |     3      | The message type of current message packet.    |
 *   +----------+------------+------------------------------------------------+
 *   | Command  |     4      | The command byte sent to remote processor/SoC. |
 *   +----------+------------+------------------------------------------------+
 *   | Reserved |   5 ~ 9    | Reserved field for future extension.           |
 *   +----------+------------+------------------------------------------------+
 *   | Data     |     N      | The data payload of the message packet.        |
 *   +----------+------------+------------------------------------------------+
 *
 * Audio control:
 *   SRTM Audio Control Category Request Command Table:
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |  Category  | Version | Type | Command | Data                          | Function                                       |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x00   | Data[0]: Audio Device Index   | Open an Audio TX Instance.                     |
 *       |            |         |      |         | Data[1]:     format           |                                                |
 *       |            |         |      |         | Data[2]:     channels         |                                                |
 *       |            |         |      |         | Data[3-6]:   samplerate       |                                                |
 *       |            |         |      |         | Data[7-10]:  buffer_addr      |                                                |
 *       |            |         |      |         | Data[11-14]: buffer_size      |                                                |
 *       |            |         |      |         | Data[15-18]: period_size      |                                                |
 *       |            |         |      |         | Data[19-22]: buffer_tail      |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x01   | Data[0]: Audio Device Index   | Start an Audio TX Instance.                    |
 *       |            |         |      |         | Same as above command         |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x02   | Data[0]: Audio Device Index   | Pause an Audio TX Instance.                    |
 *       |            |         |      |         | Same as above command         |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x03   | Data[0]: Audio Device Index   | Resume an Audio TX Instance.                   |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x04   | Data[0]: Audio Device Index   | Terminate an Audio TX Instance.                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x05   | Data[0]: Audio Device Index   | Close an Audio TX Instance.                    |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x06   | Data[0]: Audio Device Index   | Set Parameters for an Audio TX Instance.       |
 *       |            |         |      |         | Data[1]:     format           |                                                |
 *       |            |         |      |         | Data[2]:     channels         |                                                |
 *       |            |         |      |         | Data[3-6]:   samplerate       |                                                |
 *       |            |         |      |         | Data[7-22]:  reserved         |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x07   | Data[0]: Audio Device Index   | Set Audio TX Buffer.                           |
 *       |            |         |      |         | Data[1-6]:   reserved         |                                                |
 *       |            |         |      |         | Data[7-10]:  buffer_addr      |                                                |
 *       |            |         |      |         | Data[11-14]: buffer_size      |                                                |
 *       |            |         |      |         | Data[15-18]: period_size      |                                                |
 *       |            |         |      |         | Data[19-22]: buffer_tail      |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x08   | Data[0]: Audio Device Index   | Suspend an Audio TX Instance.                  |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x09   | Data[0]: Audio Device Index   | Resume an Audio TX Instance.                   |
 *       |            |         |      |         | Data[1]:     format           |                                                |
 *       |            |         |      |         | Data[2]:     channels         |                                                |
 *       |            |         |      |         | Data[3-6]:   samplerate       |                                                |
 *       |            |         |      |         | Data[7-10]:  buffer_addr      |                                                |
 *       |            |         |      |         | Data[11-14]: buffer_size      |                                                |
 *       |            |         |      |         | Data[15-18]: period_size      |                                                |
 *       |            |         |      |         | Data[19-22]: buffer_tail      |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x0A   | Data[0]: Audio Device Index   | Open an Audio RX Instance.                     |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x0B   | Data[0]: Audio Device Index   | Start an Audio RX Instance.                    |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x0C   | Data[0]: Audio Device Index   | Pause an Audio RX Instance.                    |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x0D   | Data[0]: Audio Device Index   | Resume an Audio RX Instance.                   |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x0E   | Data[0]: Audio Device Index   | Terminate an Audio RX Instance.                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x0F   | Data[0]: Audio Device Index   | Close an Audio RX Instance.                    |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x10   | Data[0]: Audio Device Index   | Set Parameters for an Audio RX Instance.       |
 *       |            |         |      |         | Data[1]:     format           |                                                |
 *       |            |         |      |         | Data[2]:     channels         |                                                |
 *       |            |         |      |         | Data[3-6]:   samplerate       |                                                |
 *       |            |         |      |         | Data[7-22]:  reserved         |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x11   | Data[0]: Audio Device Index   | Set Audio RX Buffer.                           |
 *       |            |         |      |         | Data[1-6]:   reserved         |                                                |
 *       |            |         |      |         | Data[7-10]:  buffer_addr      |                                                |
 *       |            |         |      |         | Data[11-14]: buffer_size      |                                                |
 *       |            |         |      |         | Data[15-18]: period_size      |                                                |
 *       |            |         |      |         | Data[19-22]: buffer_tail      |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x12   | Data[0]: Audio Device Index   | Suspend an Audio RX Instance.                  |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x13   | Data[0]: Audio Device Index   | Resume an Audio RX Instance.                   |
 *       |            |         |      |         | Data[1]:     format           |                                                |
 *       |            |         |      |         | Data[2]:     channels         |                                                |
 *       |            |         |      |         | Data[3-6]:   samplerate       |                                                |
 *       |            |         |      |         | Data[7-10]:  buffer_addr      |                                                |
 *       |            |         |      |         | Data[11-14]: buffer_size      |                                                |
 *       |            |         |      |         | Data[15-18]: period_size      |                                                |
 *       |            |         |      |         | Data[19-22]: buffer_tail      |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x14   | Data[0]: Audio Device Index   | Set register value to codec.                   |
 *       |            |         |      |         | Data[1-6]:   reserved         |                                                |
 *       |            |         |      |         | Data[7-10]:  register         |                                                |
 *       |            |         |      |         | Data[11-14]: value            |                                                |
 *       |            |         |      |         | Data[15-22]: reserved         |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x00 |  0x15   | Data[0]: Audio Device Index   | Get register value from codec.                 |
 *       |            |         |      |         | Data[1-6]:   reserved         |                                                |
 *       |            |         |      |         | Data[7-10]:  register         |                                                |
 *       |            |         |      |         | Data[11-22]: reserved         |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       Note 1: See <List of Sample Format> for available value of
 *               Sample Format;
 *       Note 2: See <List of Audio Channels> for available value of Channels;
 *       Note 3: Sample Rate of Set Parameters for an Audio TX Instance
 *               Command and Set Parameters for an Audio RX Instance Command is
 *               in little-endian format.
 *
 *       SRTM Audio Control Category Response Command Table:
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |  Category  | Version | Type | Command | Data                          | Function                                       |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x00   | Data[0]: Audio Device Index   | Reply for Open an Audio TX Instance.           |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x01   | Data[0]: Audio Device Index   | Reply for Start an Audio TX Instance.          |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x02   | Data[0]: Audio Device Index   | Reply for Pause an Audio TX Instance.          |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x03   | Data[0]: Audio Device Index   | Reply for Resume an Audio TX Instance.         |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x04   | Data[0]: Audio Device Index   | Reply for Terminate an Audio TX Instance.      |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x05   | Data[0]: Audio Device Index   | Reply for Close an Audio TX Instance.          |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x06   | Data[0]: Audio Device Index   | Reply for Set Parameters for an Audio          |
 *       |            |         |      |         | Data[1]: Return code          | TX Instance.                                   |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x07   | Data[0]: Audio Device Index   | Reply for Set Audio TX Buffer.                 |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x08   | Data[0]: Audio Device Index   | Reply for Suspend an Audio TX Instance.        |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x09   | Data[0]: Audio Device Index   | Reply for Resume an Audio TX Instance.         |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x0A   | Data[0]: Audio Device Index   | Reply for Open an Audio RX Instance.           |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x0B   | Data[0]: Audio Device Index   | Reply for Start an Audio RX Instance.          |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x0C   | Data[0]: Audio Device Index   | Reply for Pause an Audio RX Instance.          |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x0D   | Data[0]: Audio Device Index   | Reply for Resume an Audio RX Instance.         |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x0E   | Data[0]: Audio Device Index   | Reply for Terminate an Audio RX Instance.      |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x0F   | Data[0]: Audio Device Index   | Reply for Close an Audio RX Instance.          |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x10   | Data[0]: Audio Device Index   | Reply for Set Parameters for an Audio          |
 *       |            |         |      |         | Data[1]: Return code          | RX Instance.                                   |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x11   | Data[0]: Audio Device Index   | Reply for Set Audio RX Buffer.                 |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x12   | Data[0]: Audio Device Index   | Reply for Supend an Audio RX Instance.         |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x13   | Data[0]: Audio Device Index   | Reply for Resume an Audio RX Instance.         |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x14   | Data[0]: Audio Device Index   | Reply for Set codec register value.            |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x01 |  0x15   | Data[0]: Audio Device Index   | Reply for Get codec register value.            |
 *       |            |         |      |         | Data[1]: Return code          |                                                |
 *       |            |         |      |         | Data[2-6]:   reserved         |                                                |
 *       |            |         |      |         | Data[7-10]:  register         |                                                |
 *       |            |         |      |         | Data[11-14]: value            |                                                |
 *       |            |         |      |         | Data[15-22]: reserved         |                                                |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *
 *       SRTM Audio Control Category Notification Command Table:
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |  Category  | Version | Type | Command | Data                          | Function                                       |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x02 |  0x00   | Data[0]: Audio Device Index   | Notify one TX period is finished.              |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *       |    0x03    | 0x0100  | 0x02 |  0x01   | Data[0]: Audio Device Index   | Notify one RX period is finished.              |
 *       +------------+---------+------+---------+-------------------------------+------------------------------------------------+
 *
 *       List of Sample Format:
 *       +--------------------+----------------------------------------------+
 *       |   Sample Format    |             Description                      |
 *       +--------------------+----------------------------------------------+
 *       |         0x0        | S16_LE                                       |
 *       +--------------------+----------------------------------------------+
 *       |         0x1        | S24_LE                                       |
 *       +--------------------+----------------------------------------------+
 *
 *       List of Audio Channels
 *       +--------------------+----------------------------------------------+
 *       |    Audio Channel   |             Description                      |
 *       +--------------------+----------------------------------------------+
 *       |         0x0        | Left Channel                                 |
 *       +--------------------+----------------------------------------------+
 *       |         0x1        | Right Channel                                |
 *       +--------------------+----------------------------------------------+
 *       |         0x2        | Left & Right Channel                         |
 *       +--------------------+----------------------------------------------+
 *
 */

#ifndef __FSL_RPMSG_I2S_H
#define __FSL_RPMSG_I2S_H

#include <linux/pm_qos.h>
#include <linux/imx_rpmsg.h>
#include <linux/interrupt.h>
#include <sound/dmaengine_pcm.h>

#define RPMSG_TIMEOUT 1000

#define		I2S_TX_OPEN		0x0
#define		I2S_TX_START		0x1
#define		I2S_TX_PAUSE		0x2
#define		I2S_TX_RESTART		0x3
#define		I2S_TX_TERMINATE	0x4
#define		I2S_TX_CLOSE		0x5
#define		I2S_TX_HW_PARAM		0x6
#define		I2S_TX_BUFFER		0x7
#define		I2S_TX_SUSPEND		0x8
#define		I2S_TX_RESUME		0x9

#define		I2S_RX_OPEN		0xA
#define		I2S_RX_START		0xB
#define		I2S_RX_PAUSE		0xC
#define		I2S_RX_RESTART		0xD
#define		I2S_RX_TERMINATE	0xE
#define		I2S_RX_CLOSE		0xF
#define		I2S_RX_HW_PARAM		0x10
#define		I2S_RX_BUFFER		0x11
#define		I2S_RX_SUSPEND		0x12
#define		I2S_RX_RESUME		0x13
#define         SET_CODEC_VALUE         0x14
#define         GET_CODEC_VALUE         0x15
#define		I2S_TX_POINTER		0x16
#define		I2S_RX_POINTER		0x17

#define         I2S_TYPE_A_NUM          0x18

#define         WORK_MAX_NUM		0x18

#define		I2S_TX_PERIOD_DONE	0x0
#define		I2S_RX_PERIOD_DONE	0x1

#define         I2S_TYPE_C_NUM          0x2

#define         I2S_CMD_MAX_NUM	        (I2S_TYPE_A_NUM + I2S_TYPE_C_NUM)

#define         I2S_TYPE_A		0x0
#define         I2S_TYPE_B		0x1
#define         I2S_TYPE_C		0x2

#define		I2S_RESP_NONE		0x0
#define		I2S_RESP_NOT_ALLOWED	0x1
#define		I2S_RESP_SUCCESS	0x2
#define		I2S_RESP_FAILED		0x3

#define		RPMSG_S16_LE		0x0
#define		RPMSG_S24_LE		0x1
#define		RPMSG_S32_LE		0x2

#define		RPMSG_CH_LEFT		0x0
#define		RPMSG_CH_RIGHT		0x1
#define		RPMSG_CH_STEREO		0x2

struct i2s_param_s {
	unsigned char audioindex;
	unsigned char format;
	unsigned char channels;
	unsigned int  rate;
	unsigned int  buffer_addr;  /* Register for SET_CODEC_VALUE*/
	unsigned int  buffer_size;  /* register value for SET_CODEC_VALUE*/
	unsigned int  period_size;
	unsigned int  buffer_tail;
} __packed;

struct i2s_param_r {
	unsigned char audioindex;
	unsigned char resp;
	unsigned char reserved1[1];
	unsigned int  buffer_offset;	/* the consumed offset of buffer*/
	unsigned int  reg_addr;
	unsigned int  reg_data;
	unsigned char reserved2[4];
	unsigned int  buffer_tail;
} __packed;

/* struct of send message */
struct i2s_rpmsg_s {
	struct imx_rpmsg_head header;
	struct i2s_param_s    param;
};

/* struct of received message */
struct i2s_rpmsg_r {
	struct imx_rpmsg_head header;
	struct i2s_param_r    param;
};

struct i2s_rpmsg {
	struct i2s_rpmsg_s       send_msg;
	struct i2s_rpmsg_r       recv_msg;
};

struct work_of_rpmsg {
	struct i2s_info		*i2s_info;
	/* sent msg for each work */
	struct i2s_rpmsg         msg;
	struct work_struct       work;
};

typedef void (*dma_callback)(void *arg);
struct i2s_info {
	struct rpmsg_device     *rpdev;
	struct device            *dev;
	struct completion        cmd_complete;

	/* received msg (global) */
	struct i2s_rpmsg_r       recv_msg;

	struct i2s_rpmsg         rpmsg[I2S_CMD_MAX_NUM];

	struct workqueue_struct  *rpmsg_wq;
	struct work_of_rpmsg	 work_list[WORK_MAX_NUM];
	int                      work_index;
	int                      num_period[2];
	void                     *callback_param[2];
	int (*send_message)(struct i2s_rpmsg *msg, struct i2s_info *info);
	dma_callback             callback[2];
	spinlock_t               lock[2];
	struct mutex             tx_lock;
	struct mutex             i2c_lock;
	struct timer_list        stream_timer[2];
	int                      prealloc_buffer_size;
};

struct fsl_rpmsg_i2s {
	struct platform_device *pdev;
	struct i2s_info        i2s_info;
	struct pm_qos_request pm_qos_req;
	int codec_wm8960;
	int codec_cs42888;
	int force_lpa;
	int version;
	int rates;
	u64 formats;
	int enable_lpa;
};

#define RPMSG_CODEC_DRV_NAME_WM8960 "rpmsg-audio-codec-wm8960"
#define RPMSG_CODEC_DRV_NAME_CS42888 "rpmsg-audio-codec-cs42888"

struct fsl_rpmsg_codec {
	int audioindex;

	/*property for wm8960*/
	bool capless;
	bool shared_lrclk;

	/*property for cs42xx8*/

	char name[32];
	int num_adcs;
};

#endif /* __FSL_RPMSG_I2S_H */
