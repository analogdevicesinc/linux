/******************************************************************************
 * i2cif.h
 *
 * I2C device I/O interface for Xen guest OSes.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Copyright 2018 NXP
 *
 * Authors: Peng Fan <peng.fan@nxp.com>
 */

#ifndef __XEN_PUBLIC_IO_I2CIF_H__
#define __XEN_PUBLIC_IO_I2CIF_H__

#include <xen/interface/io/ring.h>
#include <xen/interface/grant_table.h>

#define I2CIF_BUF_LEN	32
#define I2CIF_MAX_MSG	2

#define I2CIF_M_RD		0x0001	/* read data, from slave to master */
					/* I2C_M_RD is guaranteed to be 0x0001! */
#define I2CIF_M_TEN		0x0010	/* this is a ten bit chip address */
#define I2CIF_M_RECV_LEN	0x0400	/* length will be first received byte */
#define I2CIF_M_NO_RD_ACK	0x0800	/* if I2CIF_FUNC_PROTOCOL_MANGLING */
#define I2CIF_M_IGNORE_NAK	0x1000	/* if I2CIF_FUNC_PROTOCOL_MANGLING */
#define I2CIF_M_REV_DIR_ADDR	0x2000	/* if I2CIF_FUNC_PROTOCOL_MANGLING */
#define I2CIF_M_NOSTART		0x4000	/* if I2CIF_FUNC_NOSTART */
#define I2CIF_M_STOP		0x8000	/* if I2CIF_FUNC_PROTOCOL_MANGLING */

#define I2CIF_FUNC_I2C				0x00000001
#define I2CIF_FUNC_10BIT_ADDR			0x00000002
#define I2CIF_FUNC_PROTOCOL_MANGLING		0x00000004 /* I2C_M_IGNORE_NAK etc. */
#define I2CIF_FUNC_SMBUS_PEC			0x00000008
#define I2CIF_FUNC_NOSTART			0x00000010 /* I2C_M_NOSTART */
#define I2CIF_FUNC_SLAVE			0x00000020
#define I2CIF_FUNC_SMBUS_BLOCK_PROC_CALL	0x00008000 /* SMBus 2.0 */
#define I2CIF_FUNC_SMBUS_QUICK			0x00010000
#define I2CIF_FUNC_SMBUS_READ_BYTE		0x00020000
#define I2CIF_FUNC_SMBUS_WRITE_BYTE		0x00040000
#define I2CIF_FUNC_SMBUS_READ_BYTE_DATA		0x00080000
#define I2CIF_FUNC_SMBUS_WRITE_BYTE_DATA	0x00100000
#define I2CIF_FUNC_SMBUS_READ_WORD_DATA		0x00200000
#define I2CIF_FUNC_SMBUS_WRITE_WORD_DATA	0x00400000
#define I2CIF_FUNC_SMBUS_PROC_CALL		0x00800000
#define I2CIF_FUNC_SMBUS_READ_BLOCK_DATA	0x01000000
#define I2CIF_FUNC_SMBUS_WRITE_BLOCK_DATA	0x02000000
#define I2CIF_FUNC_SMBUS_READ_I2C_BLOCK		0x04000000 /* I2C-like block xfer  */
#define I2CIF_FUNC_SMBUS_WRITE_I2C_BLOCK	0x08000000 /* w/ 1-byte reg. addr. */
#define I2CIF_FUNC_SMBUS_HOST_NOTIFY		0x10000000

#define I2CIF_ADAPTER_NAME_LEN	32

struct i2cif_request {
	struct {
		__u16 addr;	/* slave address */
		__u16 flags;	/* msg flags */
		__u16 len;	/* msg length */
	} msg[I2CIF_MAX_MSG];
	int num_msg;
	__u8 write_buf[I2CIF_BUF_LEN];
};

struct i2cif_response {
	int result;
	__u8 read_buf[I2CIF_BUF_LEN];
};

DEFINE_RING_TYPES(i2cif, struct i2cif_request, struct i2cif_response);

#endif
