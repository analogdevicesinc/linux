/**
 * gadget.h - Cadence USB3 device Controller Core file
 *
 * Copyright (C) 2016 Cadence Design Systems - http://www.cadence.com
 * Copyright 2017 NXP
 *
 * Authors: Pawel Jez <pjez@cadence.com>,
 *          Konrad Kociolek <konrad@cadence.com>,
 *          Peter Chen <peter.chen@nxp.com>
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __DRIVERS_CDNS3_GADGET
#define __DRIVERS_CDNS3_GADGET

#include "dev-regs-map.h"

#if IS_ENABLED(CONFIG_USB_CDNS_MISC)
#include "cdns_misc.h"
#endif

#define gadget_to_usb_ss(g)  \
	(container_of(g, struct usb_ss_dev, gadget))

#define to_usb_ss_ep(ep) \
	(container_of(ep, struct usb_ss_endpoint, endpoint))

#define ep_to_usb_ss_ep(ep) \
	(container_of(ep, struct usb_ss_endpoint, endpoint))

/*-------------------------------------------------------------------------*/
/* TRB macros */

/* Common TRB fields */
#define TRB_SET_CYCLE_BIT		1uL
#define TRB_SET_CHAIN_BIT		0x10

/* offset 0 */
#define TRB_DATA_BUFFER_POINTER_MASK	0xFFFFFFFF
#define TRB_SET_DATA_BUFFER_POINTER(p)	(p & TRB_DATA_BUFFER_POINTER_MASK)

/* offset 4 */
#define TRB_TRANSFER_LENGTH_MASK	0x1FFFF
#define TRB_SET_TRANSFER_LENGTH(l)	(l & TRB_TRANSFER_LENGTH_MASK)

#define TRB_BURST_LENGTH_MASK		0xFF
#define TRB_SET_BURST_LENGTH(l)		((l & TRB_BURST_LENGTH_MASK) << 24)

/* offset 8 */
#define TRB_SET_INT_ON_SHORT_PACKET	0x04
#define TRB_SET_FIFO_MODE		0x08
#define TRB_SET_INT_ON_COMPLETION	0x20

#define TRB_TYPE_NORMAL			0x400

#define TRB_STREAM_ID_MASK		0xFFFF
#define TRB_SET_STREAM_ID(sid)		((sid & TRB_STREAM_ID_MASK) << 16)

/*-------------------------------------------------------------------------*/
/* Driver numeric constants */


#define DEVICE_ADDRESS_MAX		127

/* Endpoint init values */
#define ENDPOINT_MAX_PACKET_LIMIT	1024
#define ENDPOINT_MAX_STREAMS		15

#define ENDPOINT0_MAX_PACKET_LIMIT	512

/* All endpoints except EP0 */
#define USB_SS_ENDPOINTS_MAX_COUNT	30

#define USB_SS_TRBS_NUM			32

/* Standby mode */
#define STB_CLK_SWITCH_DONE_MASK	0x200
#define STB_CLK_SWITCH_EN_MASK		0x100
#define STB_CLK_SWITCH_EN_SHIFT		8

#define ENDPOINT_MAX_PACKET_SIZE_0	0
#define ENDPOINT_MAX_PACKET_SIZE_8	8
#define ENDPOINT_MAX_PACKET_SIZE_64	64
#define ENDPOINT_MAX_PACKET_SIZE_512	512
#define ENDPOINT_MAX_PACKET_SIZE_1023	1023
#define ENDPOINT_MAX_PACKET_SIZE_1024	1024

#define SS_LINK_STATE_U3		3
#define FSHS_LPM_STATE_L2		2

#define ADDR_MODULO_8			8

#define INTERRUPT_MASK			0xFFFFFFFF

#define ACTUAL_TRANSFERRED_BYTES_MASK	0x1FFFF

#define ENDPOINT_DIR_MASK		0x80

#define ENDPOINT_ZLP_BUF_SIZE		1024
/*-------------------------------------------------------------------------*/

/**
 * IS_REG_REQUIRING_ACTIVE_REF_CLOCK - Macro checks if desired
 * register requires active clock, it involves such registers as:
 * EP_CFG, EP_TR_ADDR, EP_CMD, EP_SEL, USB_CONF
 * @usb_ss: extended gadget object
 * @reg: register address
 */
#define IS_REG_REQUIRING_ACTIVE_REF_CLOCK(usb_ss, reg)	(!reg || \
	(reg >= &usb_ss->regs->ep_sel && reg <= &usb_ss->regs->ep_cmd))

/**
 * CAST_EP_REG_POS_TO_INDEX - Macro converts bit position of ep_ists register to
 * index of endpoint object in usb_ss_dev.eps[] container
 * @i: bit position of endpoint for which endpoint object is required
 *
 * Remember that endpoint container doesn't contain default endpoint
 */
#define CAST_EP_REG_POS_TO_INDEX(i) (((i) / 16) + ((((i) % 16) - 2) * 2))

/**
 * CAST_EP_ADDR_TO_INDEX - Macro converts endpoint address to
 * index of endpoint object in usb_ss_dev.eps[] container
 * @ep_addr: endpoint address for which endpoint object is required
 *
 * Remember that endpoint container doesn't contain default endpoint
 */
#define CAST_EP_ADDR_TO_INDEX(ep_addr) \
	(((ep_addr & 0x7F) - 1) + ((ep_addr & 0x80) ? 1 : 0))

/**
 * CAST_EP_ADDR_TO_BIT_POS - Macro converts endpoint address to
 * bit position in ep_ists register
 * @ep_addr: endpoint address for which bit position is required
 *
 * Remember that endpoint container doesn't contain default endpoint
 */
#define CAST_EP_ADDR_TO_BIT_POS(ep_addr) \
	(((uint32_t)1 << (ep_addr & 0x7F))  << ((ep_addr & 0x80) ? 16 : 0))


#define CAST_INDEX_TO_EP_ADDR(index) \
	((index / 2 + 1) | ((index % 2) ? 0x80 : 0x00))

/* 18KB is the total size, and 2KB is used for EP0 and configuration */
#define CDNS3_ONCHIP_BUF_SIZE	16	/* KB */
#define CDNS3_EP_BUF_SIZE	2	/* KB */
#define CDNS3_UNALIGNED_BUF_SIZE	16384 /* Bytes */
/*-------------------------------------------------------------------------*/
/* Used structs */

struct usb_ss_trb {
	u32 offset0;
	u32 offset4;
	u32 offset8;
};

struct usb_ss_dev;

struct usb_ss_endpoint {
	struct usb_ep endpoint;
	struct list_head request_list;
	struct list_head ep_match_pending_list;

	struct usb_ss_trb *trb_pool;
	dma_addr_t trb_pool_dma;

	struct usb_ss_dev *usb_ss;
	char name[20];
	int hw_pending_flag;
	int stalled_flag;
	int wedge_flag;
	void *cpu_addr;
	dma_addr_t dma_addr;
	u8					dir;
	u8					num;
	u8					type;
	bool					used;
};

struct usb_ss_dev {
	struct device dev;
	struct usbss_dev_register_block_type __iomem *regs;

	struct usb_gadget gadget;
	struct usb_gadget_driver *gadget_driver;

	dma_addr_t setup_dma;
	dma_addr_t trb_ep0_dma;
	u32 *trb_ep0;
	u8 *setup;
	void *zlp_buf;

	struct usb_ss_endpoint *eps[USB_SS_ENDPOINTS_MAX_COUNT];
	int ep_nums;
	struct usb_request *actual_ep0_request;
	int ep0_data_dir;
	int hw_configured_flag;
	int wake_up_flag;
	u16 isoch_delay;
	spinlock_t lock;

	unsigned is_connected:1;
	unsigned in_standby_mode:1;

	u32 usb_ien;
	u32 ep_ien;
	int setup_pending;
	struct device *sysdev;
	bool start_gadget; /* The device mode is enabled */
	struct list_head ep_match_list;
	int onchip_mem_allocated_size; /* KB */
	/* Memory is allocated for OUT */
	int out_mem_is_allocated:1;
};

#endif /* __DRIVERS_CDNS3_GADGET */
