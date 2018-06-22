/**
 * gadget.c - Cadence USB3 Device Core file
 *
 * Copyright (C) 2016 Cadence Design Systems - http://www.cadence.com
 * Copyright 2017 NXP
 *
 * Authors: Pawel Jez <pjez@cadence.com>,
 *          Konrad Kociolek <konrad@cadence.com>
 *	    Peter Chen <peter.chen@nxp.com>
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/usb/composite.h>
#include <linux/of_platform.h>
#include <linux/usb/gadget.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/byteorder/generic.h>
#include <linux/ctype.h>

#include "core.h"
#include "gadget-export.h"
#include "gadget.h"
#include "io.h"

/*-------------------------------------------------------------------------*/
/* Function declarations */

static void select_ep(struct usb_ss_dev *usb_ss, u32 ep);
static int usb_ss_allocate_trb_pool(struct usb_ss_endpoint *usb_ss_ep);
static void cdns_ep_stall_flush(struct usb_ss_endpoint *usb_ss_ep);
static void cdns_ep0_config(struct usb_ss_dev *usb_ss);
static void cdns_gadget_unconfig(struct usb_ss_dev *usb_ss);
static void cdns_ep0_run_transfer(struct usb_ss_dev *usb_ss,
	dma_addr_t dma_addr, unsigned int length, int erdy);
static int cdns_ep_run_transfer(struct usb_ss_endpoint *usb_ss_ep);
static int cdns_get_setup_ret(struct usb_ss_dev *usb_ss,
	struct usb_ctrlrequest *ctrl_req);
static int cdns_req_ep0_set_address(struct usb_ss_dev *usb_ss,
	struct usb_ctrlrequest *ctrl_req);
static int cdns_req_ep0_get_status(struct usb_ss_dev *usb_ss,
	struct usb_ctrlrequest *ctrl_req);
static int cdns_req_ep0_handle_feature(struct usb_ss_dev *usb_ss,
	struct usb_ctrlrequest *ctrl_req, int set);
static int cdns_req_ep0_set_sel(struct usb_ss_dev *usb_ss,
	struct usb_ctrlrequest *ctrl_req);
static int cdns_req_ep0_set_isoch_delay(struct usb_ss_dev *usb_ss,
	struct usb_ctrlrequest *ctrl_req);
static int cdns_req_ep0_set_configuration(struct usb_ss_dev *usb_ss,
	struct usb_ctrlrequest *ctrl_req);
static int cdns_ep0_standard_request(struct usb_ss_dev *usb_ss,
	struct usb_ctrlrequest *ctrl_req);
static void cdns_ep0_setup_phase(struct usb_ss_dev *usb_ss);
static int cdns_check_ep_interrupt_proceed(struct usb_ss_endpoint *usb_ss_ep);
static void cdns_check_ep0_interrupt_proceed(struct usb_ss_dev *usb_ss,
	int dir);
static void cdns_check_usb_interrupt_proceed(struct usb_ss_dev *usb_ss,
	u32 usb_ists);
static int usb_ss_gadget_ep0_enable(struct usb_ep *ep,
	const struct usb_endpoint_descriptor *desc);
static int usb_ss_gadget_ep0_disable(struct usb_ep *ep);
static int usb_ss_gadget_ep0_set_halt(struct usb_ep *ep, int value);
static int usb_ss_gadget_ep0_queue(struct usb_ep *ep,
	struct usb_request *request, gfp_t gfp_flags);
static int usb_ss_gadget_ep_enable(struct usb_ep *ep,
	const struct usb_endpoint_descriptor *desc);
static int usb_ss_gadget_ep_disable(struct usb_ep *ep);
static struct usb_request *usb_ss_gadget_ep_alloc_request(struct usb_ep *ep,
	gfp_t gfp_flags);
static void usb_ss_gadget_ep_free_request(struct usb_ep *ep,
	struct usb_request *request);
static int usb_ss_gadget_ep_queue(struct usb_ep *ep,
	struct usb_request *request, gfp_t gfp_flags);
static int usb_ss_gadget_ep_dequeue(struct usb_ep *ep,
	struct usb_request *request);
static int usb_ss_gadget_ep_set_halt(struct usb_ep *ep, int value);
static int usb_ss_gadget_ep_set_wedge(struct usb_ep *ep);
static int usb_ss_gadget_get_frame(struct usb_gadget *gadget);
static int usb_ss_gadget_wakeup(struct usb_gadget *gadget);
static int usb_ss_gadget_set_selfpowered(struct usb_gadget *gadget,
	int is_selfpowered);
static int usb_ss_gadget_pullup(struct usb_gadget *gadget, int is_on);
static int usb_ss_gadget_udc_start(struct usb_gadget *gadget,
	struct usb_gadget_driver *driver);
static int usb_ss_gadget_udc_stop(struct usb_gadget *gadget);
static int usb_ss_init_ep(struct usb_ss_dev *usb_ss);
static int usb_ss_init_ep0(struct usb_ss_dev *usb_ss);
static void __cdns3_gadget_start(struct usb_ss_dev *usb_ss);
static void cdns_prepare_setup_packet(struct usb_ss_dev *usb_ss);
static void cdns_ep_config(struct usb_ss_endpoint *usb_ss_ep);
static void cdns_enable_l1(struct usb_ss_dev *usb_ss, int enable);

static struct usb_endpoint_descriptor cdns3_gadget_ep0_desc = {
	.bLength	= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes	= USB_ENDPOINT_XFER_CONTROL,
};

static u32 gadget_readl(struct usb_ss_dev *usb_ss, uint32_t __iomem *reg)
{
	return cdns_readl(reg);
}

static void gadget_writel(struct usb_ss_dev *usb_ss,
		uint32_t __iomem *reg, u32 value)
{
	cdns_writel(reg, value);
}

/**
 * next_request - returns next request from list
 * @list: list containing requests
 *
 * Returns request or NULL if no requests in list
 */
static struct usb_request *next_request(struct list_head *list)
{
	if (list_empty(list))
		return NULL;
	return list_first_entry(list, struct usb_request, list);
}

/**
 * wait_reg_bit - Read reg and compare until equal to specific value
 * @reg: the register address to read
 * @value: the value to compare
 * @wait_value: 0 or 1
 * @timeout_ms: timeout value in milliseconds, must be larger than 1
 *
 * Returns -ETIMEDOUT if timeout occurs
 */
static int wait_reg_bit(struct usb_ss_dev *usb_ss, u32 __iomem *reg,
		u32 value, int wait_value, int timeout_ms)
{
	u32 temp;

	WARN_ON(timeout_ms <= 0);
	timeout_ms *= 100;
	temp = cdns_readl(reg);
	while (timeout_ms-- > 0) {
		if (!!(temp & value) == wait_value)
			return 0;
		temp = cdns_readl(reg);
		udelay(10);
	}

	dev_err(&usb_ss->dev, "wait register timeout %s\n", __func__);
	return -ETIMEDOUT;
}

static int wait_reg_bit_set(struct usb_ss_dev *usb_ss, u32 __iomem *reg,
		u32 value, int timeout_ms)
{
	return wait_reg_bit(usb_ss, reg, value, 1, timeout_ms);
}

static int wait_reg_bit_clear(struct usb_ss_dev *usb_ss, u32 __iomem *reg,
		u32 value, int timeout_ms)
{
	return wait_reg_bit(usb_ss, reg, value, 0, timeout_ms);
}

/**
 * select_ep - selects endpoint
 * @usb_ss: extended gadget object
 * @ep: endpoint address
 */
static void select_ep(struct usb_ss_dev *usb_ss, u32 ep)
{
	if (!usb_ss || !usb_ss->regs) {
		dev_err(&usb_ss->dev, "Failed to select endpoint!\n");
		return;
	}

	gadget_writel(usb_ss, &usb_ss->regs->ep_sel, ep);
}

/**
 * usb_ss_allocate_trb_pool - Allocates TRB's pool for selected endpoint
 * @usb_ss_ep: extended endpoint object
 *
 * Function will return 0 on success or -ENOMEM on allocation error
 */
static int usb_ss_allocate_trb_pool(struct usb_ss_endpoint *usb_ss_ep)
{
	struct usb_ss_dev *usb_ss = usb_ss_ep->usb_ss;

	if (usb_ss_ep->trb_pool)
		return 0;

	usb_ss_ep->trb_pool = dma_zalloc_coherent(usb_ss->sysdev,
			sizeof(struct usb_ss_trb) * USB_SS_TRBS_NUM,
		&usb_ss_ep->trb_pool_dma, GFP_DMA);

	if (!usb_ss_ep->trb_pool) {
		dev_err(&usb_ss->dev,
				"Failed to allocate TRB pool for endpoint %s\n",
				usb_ss_ep->name);
		return -ENOMEM;
	}

	return 0;
}

/**
 * cdns_data_flush - do flush data at onchip buffer
 * @usb_ss_ep: extended endpoint object
 *
 * Endpoint must be selected before call to this function
 *
 * Returns zero on success or negative value on failure
 */
static int cdns_data_flush(struct usb_ss_endpoint *usb_ss_ep)
{
	struct usb_ss_dev *usb_ss = usb_ss_ep->usb_ss;

	gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
		EP_CMD__DFLUSH__MASK);
	/* wait for DFLUSH cleared */
	return wait_reg_bit_clear(usb_ss, &usb_ss->regs->ep_cmd,
		EP_CMD__DFLUSH__MASK, 100);
}

/**
 * cdns_ep_stall_flush - Stalls and flushes selected endpoint
 * @usb_ss_ep: extended endpoint object
 *
 * Endpoint must be selected before call to this function
 */
static void cdns_ep_stall_flush(struct usb_ss_endpoint *usb_ss_ep)
{
	struct usb_ss_dev *usb_ss = usb_ss_ep->usb_ss;

	gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
		EP_CMD__DFLUSH__MASK | EP_CMD__ERDY__MASK |
		EP_CMD__SSTALL__MASK);

	/* wait for DFLUSH cleared */
	wait_reg_bit_clear(usb_ss, &usb_ss->regs->ep_cmd,
		EP_CMD__DFLUSH__MASK, 100);
	usb_ss_ep->stalled_flag = 1;
}

/**
 * cdns_ep0_config - Configures default endpoint
 * @usb_ss: extended gadget object
 *
 * Functions sets parameters: maximal packet size and enables interrupts
 */
static void cdns_ep0_config(struct usb_ss_dev *usb_ss)
{
	u32 reg, max_packet_size = 0;

	switch (usb_ss->gadget.speed) {
	case USB_SPEED_UNKNOWN:
		max_packet_size = ENDPOINT_MAX_PACKET_SIZE_0;
		usb_ss->gadget.ep0->maxpacket = ENDPOINT_MAX_PACKET_SIZE_0;
		cdns3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(0);
		break;

	case USB_SPEED_LOW:
		max_packet_size = ENDPOINT_MAX_PACKET_SIZE_8;
		usb_ss->gadget.ep0->maxpacket = ENDPOINT_MAX_PACKET_SIZE_8;
		cdns3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(8);
		break;

	case USB_SPEED_FULL:
		max_packet_size = ENDPOINT_MAX_PACKET_SIZE_64;
		usb_ss->gadget.ep0->maxpacket = ENDPOINT_MAX_PACKET_SIZE_64;
		cdns3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		break;

	case USB_SPEED_HIGH:
		max_packet_size = ENDPOINT_MAX_PACKET_SIZE_64;
		usb_ss->gadget.ep0->maxpacket = ENDPOINT_MAX_PACKET_SIZE_64;
		cdns3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		break;

	case USB_SPEED_WIRELESS:
		max_packet_size = ENDPOINT_MAX_PACKET_SIZE_64;
		usb_ss->gadget.ep0->maxpacket = ENDPOINT_MAX_PACKET_SIZE_64;
		cdns3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		break;

	case USB_SPEED_SUPER:
		max_packet_size = ENDPOINT_MAX_PACKET_SIZE_512;
		usb_ss->gadget.ep0->maxpacket = ENDPOINT_MAX_PACKET_SIZE_512;
		cdns3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);
		break;

	case USB_SPEED_SUPER_PLUS:
		dev_warn(&usb_ss->dev, "USB 3.1 is not supported\n");
		usb_ss->gadget.ep0->maxpacket = ENDPOINT_MAX_PACKET_SIZE_512;
		cdns3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);
		max_packet_size = ENDPOINT_MAX_PACKET_SIZE_512;
		break;
	}

	/* init ep out */
	select_ep(usb_ss, USB_DIR_OUT);

	gadget_writel(usb_ss, &usb_ss->regs->ep_cfg,
		EP_CFG__ENABLE__MASK |
		EP_CFG__MAXPKTSIZE__WRITE(max_packet_size));
	gadget_writel(usb_ss, &usb_ss->regs->ep_sts_en,
		EP_STS_EN__SETUPEN__MASK |
		EP_STS_EN__DESCMISEN__MASK |
		EP_STS_EN__TRBERREN__MASK);

	/* init ep in */
	select_ep(usb_ss, USB_DIR_IN);

	gadget_writel(usb_ss, &usb_ss->regs->ep_cfg,
		EP_CFG__ENABLE__MASK |
		EP_CFG__MAXPKTSIZE__WRITE(max_packet_size));
	gadget_writel(usb_ss, &usb_ss->regs->ep_sts_en,
		EP_STS_EN__SETUPEN__MASK |
		EP_STS_EN__TRBERREN__MASK);

	reg = gadget_readl(usb_ss, &usb_ss->regs->usb_conf);
	reg |= USB_CONF__U1DS__MASK | USB_CONF__U2DS__MASK;
	gadget_writel(usb_ss, &usb_ss->regs->usb_conf, reg);

	cdns_prepare_setup_packet(usb_ss);
}

/**
 * cdns_gadget_unconfig - Unconfigures device controller
 * @usb_ss: extended gadget object
 */
static void cdns_gadget_unconfig(struct usb_ss_dev *usb_ss)
{
	/* RESET CONFIGURATION */
	gadget_writel(usb_ss, &usb_ss->regs->usb_conf,
		USB_CONF__CFGRST__MASK);

	cdns_enable_l1(usb_ss, 0);
	usb_ss->hw_configured_flag = 0;
	usb_ss->onchip_mem_allocated_size = 0;
	usb_ss->out_mem_is_allocated = 0;
}

/**
 * cdns_ep0_run_transfer - Do transfer on default endpoint hardware
 * @usb_ss: extended gadget object
 * @dma_addr: physical address where data is/will be stored
 * @length: data length
 * @erdy: set it to 1 when ERDY packet should be sent -
 *        exit from flow control state
 */
static void cdns_ep0_run_transfer(struct usb_ss_dev *usb_ss,
		dma_addr_t dma_addr, unsigned int length, int erdy)
{
	usb_ss->trb_ep0[0] = TRB_SET_DATA_BUFFER_POINTER(dma_addr);
	usb_ss->trb_ep0[1] = TRB_SET_TRANSFER_LENGTH((u32)length);
	usb_ss->trb_ep0[2] = TRB_SET_CYCLE_BIT |
		TRB_SET_INT_ON_COMPLETION | TRB_TYPE_NORMAL;

	dev_dbg(&usb_ss->dev, "DRBL(%02X)\n",
		usb_ss->ep0_data_dir ? USB_DIR_IN : USB_DIR_OUT);

	select_ep(usb_ss, usb_ss->ep0_data_dir
		? USB_DIR_IN : USB_DIR_OUT);

	gadget_writel(usb_ss, &usb_ss->regs->ep_traddr,
			EP_TRADDR__TRADDR__WRITE(usb_ss->trb_ep0_dma));
	gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
		EP_CMD__DRDY__MASK); /* drbl */

	if (erdy)
		gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
			EP_CMD__ERDY__MASK);
}

/**
 * cdns_ep_run_transfer - Do transfer on no-default endpoint hardware
 * @usb_ss_ep: extended endpoint object
 *
 * Returns zero on success or negative value on failure
 */
static int cdns_ep_run_transfer(struct usb_ss_endpoint *usb_ss_ep)
{
	dma_addr_t trb_dma;
	struct usb_request *request = next_request(&usb_ss_ep->request_list);
	struct usb_ss_dev *usb_ss = usb_ss_ep->usb_ss;
	int sg_iter = 0;
	struct usb_ss_trb *trb;

	if (request == NULL)
		return -EINVAL;

	if (request->num_sgs > USB_SS_TRBS_NUM)
		return -EINVAL;

	dev_dbg(&usb_ss->dev, "DRBL(%02X)\n",
		usb_ss_ep->endpoint.desc->bEndpointAddress);

	usb_ss_ep->hw_pending_flag = 1;
	trb_dma = request->dma;

	/* must allocate buffer aligned to 8 */
	if ((request->dma % ADDR_MODULO_8)) {
		if (request->length <= CDNS3_UNALIGNED_BUF_SIZE) {
			memcpy(usb_ss_ep->cpu_addr, request->buf,
				request->length);
			trb_dma = usb_ss_ep->dma_addr;
		} else {
			return -ENOMEM;
		}
	}

	trb = usb_ss_ep->trb_pool;

	do {
	/* fill TRB */
		trb->offset0 = TRB_SET_DATA_BUFFER_POINTER(request->num_sgs == 0
				? trb_dma : request->sg[sg_iter].dma_address);

		trb->offset4 = TRB_SET_BURST_LENGTH(16) |
			TRB_SET_TRANSFER_LENGTH(request->num_sgs == 0 ?
				request->length : request->sg[sg_iter].length);

		trb->offset8 = TRB_SET_CYCLE_BIT
			| TRB_SET_INT_ON_COMPLETION
			| TRB_SET_INT_ON_SHORT_PACKET
			| TRB_TYPE_NORMAL;

		++sg_iter;
		++trb;

	} while (sg_iter < request->num_sgs);

	/* arm transfer on selected endpoint */
	select_ep(usb_ss_ep->usb_ss,
			usb_ss_ep->endpoint.desc->bEndpointAddress);
	gadget_writel(usb_ss, &usb_ss->regs->ep_traddr,
			EP_TRADDR__TRADDR__WRITE(usb_ss_ep->trb_pool_dma));
	gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
		EP_CMD__DRDY__MASK); /* DRDY */
	return 0;
}

/**
 * cdns_get_setup_ret - Returns status of handling setup packet
 * Setup is handled by gadget driver
 * @usb_ss: extended gadget object
 * @ctrl_req: pointer to received setup packet
 *
 * Returns zero on success or negative value on failure
 */
static int cdns_get_setup_ret(struct usb_ss_dev *usb_ss,
		struct usb_ctrlrequest *ctrl_req)
{
	int ret;

	spin_unlock(&usb_ss->lock);
	usb_ss->setup_pending = 1;
	ret = usb_ss->gadget_driver->setup(&usb_ss->gadget, ctrl_req);
	usb_ss->setup_pending = 0;
	spin_lock(&usb_ss->lock);
	return ret;
}

static void cdns_prepare_setup_packet(struct usb_ss_dev *usb_ss)
{
	usb_ss->ep0_data_dir = 0;
	cdns_ep0_run_transfer(usb_ss, usb_ss->setup_dma, 8, 0);
}

/**
 * cdns_req_ep0_set_address - Handling of SET_ADDRESS standard USB request
 * @usb_ss: extended gadget object
 * @ctrl_req: pointer to received setup packet
 *
 * Returns 0 if success, error code on error
 */
static int cdns_req_ep0_set_address(struct usb_ss_dev *usb_ss,
		struct usb_ctrlrequest *ctrl_req)
{
	enum usb_device_state device_state = usb_ss->gadget.state;
	u32 reg;
	u32 addr;

	addr = le16_to_cpu(ctrl_req->wValue);

	if (addr > DEVICE_ADDRESS_MAX) {
		dev_err(&usb_ss->dev,
			"Device address (%d) cannot be greater than %d\n",
				addr, DEVICE_ADDRESS_MAX);
		return -EINVAL;
	}

	if (device_state == USB_STATE_CONFIGURED) {
		dev_err(&usb_ss->dev, "USB device already configured\n");
		return -EINVAL;
	}

	reg = gadget_readl(usb_ss, &usb_ss->regs->usb_cmd);

	gadget_writel(usb_ss, &usb_ss->regs->usb_cmd, reg
			| USB_CMD__FADDR__WRITE(addr)
			| USB_CMD__SET_ADDR__MASK);

	usb_gadget_set_state(&usb_ss->gadget,
		(addr ? USB_STATE_ADDRESS : USB_STATE_DEFAULT));

	cdns_prepare_setup_packet(usb_ss);

	gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
		EP_CMD__ERDY__MASK | EP_CMD__REQ_CMPL__MASK);
	return 0;
}

/**
 * cdns_req_ep0_get_status - Handling of GET_STATUS standard USB request
 * @usb_ss: extended gadget object
 * @ctrl_req: pointer to received setup packet
 *
 * Returns 0 if success, error code on error
 */
static int cdns_req_ep0_get_status(struct usb_ss_dev *usb_ss,
		struct usb_ctrlrequest *ctrl_req)
{
	u16 usb_status = 0;
	unsigned int length = 2;
	u32 recip = ctrl_req->bRequestType & USB_RECIP_MASK;
	u32 reg;

	switch (recip) {

	case USB_RECIP_DEVICE:
		/* handling otg features */
		if (ctrl_req->wIndex == OTG_STS_SELECTOR) {
			length = 1;
			usb_status = usb_ss->gadget.host_request_flag;
		} else {

			reg = gadget_readl(usb_ss, &usb_ss->regs->usb_sts);

			if (reg & USB_STS__U1ENS__MASK)
				usb_status |= 1uL << USB_DEV_STAT_U1_ENABLED;

			if (reg & USB_STS__U2ENS__MASK)
				usb_status |= 1uL << USB_DEV_STAT_U2_ENABLED;

			if (usb_ss->wake_up_flag)
				usb_status |= 1uL << USB_DEVICE_REMOTE_WAKEUP;

			/* self powered */
			usb_status |= usb_ss->gadget.is_selfpowered;
		}
		break;

	case USB_RECIP_INTERFACE:
		return cdns_get_setup_ret(usb_ss, ctrl_req);

	case USB_RECIP_ENDPOINT:
		/* check if endpoint is stalled */
		select_ep(usb_ss, ctrl_req->wIndex);
		if (gadget_readl(usb_ss, &usb_ss->regs->ep_sts)
			& EP_STS__STALL__MASK)
			usb_status = 1;
		break;

	default:
		return -EINVAL;
	}

	*(u16 *)usb_ss->setup = cpu_to_le16(usb_status);

	usb_ss->actual_ep0_request = NULL;
	cdns_ep0_run_transfer(usb_ss, usb_ss->setup_dma, length, 1);
	return 0;
}

/**
 * cdns_req_ep0_handle_feature -
 * Handling of GET/SET_FEATURE standard USB request
 *
 * @usb_ss: extended gadget object
 * @ctrl_req: pointer to received setup packet
 * @set: must be set to 1 for SET_FEATURE request
 *
 * Returns 0 if success, error code on error
 */
static int cdns_req_ep0_handle_feature(struct usb_ss_dev *usb_ss,
		struct usb_ctrlrequest *ctrl_req, int set)
{
	u32 recip = ctrl_req->bRequestType & USB_RECIP_MASK;
	struct usb_ss_endpoint *usb_ss_ep;
	u32 reg;
	u8 tmode = 0;
	int ret = 0;

	switch (recip) {
	case USB_RECIP_DEVICE:
		switch (ctrl_req->wValue) {
		case USB_DEVICE_U1_ENABLE:
			if (usb_ss->gadget.state != USB_STATE_CONFIGURED)
				return -EINVAL;
			if (usb_ss->gadget.speed != USB_SPEED_SUPER)
				return -EINVAL;

			reg = gadget_readl(usb_ss, &usb_ss->regs->usb_conf);
			if (set)
				/* set U1EN */
				reg |= USB_CONF__U1EN__MASK;
			else
				/* set U1 disable */
				reg |= USB_CONF__U1DS__MASK;
			gadget_writel(usb_ss, &usb_ss->regs->usb_conf, reg);
			break;
		case USB_DEVICE_U2_ENABLE:
			if (usb_ss->gadget.state != USB_STATE_CONFIGURED)
				return -EINVAL;
			if (usb_ss->gadget.speed != USB_SPEED_SUPER)
				return -EINVAL;

			reg = gadget_readl(usb_ss, &usb_ss->regs->usb_conf);
			if (set)
				/* set U2EN */
				reg |= USB_CONF__U2EN__MASK;
			else
				/* set U2 disable */
				reg |= USB_CONF__U2DS__MASK;
			gadget_writel(usb_ss, &usb_ss->regs->usb_conf, reg);
			break;
		case USB_DEVICE_A_ALT_HNP_SUPPORT:
			break;
		case USB_DEVICE_A_HNP_SUPPORT:
			break;
		case USB_DEVICE_B_HNP_ENABLE:
			if (!usb_ss->gadget.b_hnp_enable && set)
				usb_ss->gadget.b_hnp_enable = 1;
			break;
		case USB_DEVICE_REMOTE_WAKEUP:
			usb_ss->wake_up_flag = !!set;
			break;
		case USB_DEVICE_TEST_MODE:
			if (usb_ss->gadget.state != USB_STATE_CONFIGURED)
				return -EINVAL;
			if (usb_ss->gadget.speed != USB_SPEED_HIGH &&
				usb_ss->gadget.speed !=	USB_SPEED_FULL)
				return -EINVAL;
			if (ctrl_req->wLength != 0 ||
				ctrl_req->bRequestType & USB_DIR_IN) {
				dev_err(&usb_ss->dev, "req is error\n");
				return -EINVAL;
			}
			tmode = le16_to_cpu(ctrl_req->wIndex) >> 8;
			switch (tmode) {
			case TEST_J:
			case TEST_K:
			case TEST_SE0_NAK:
			case TEST_PACKET:
				reg = gadget_readl(usb_ss,
					&usb_ss->regs->usb_cmd);
				tmode -= 1;
				reg |= USB_CMD__STMODE |
					USB_CMD__TMODE_SEL(tmode);
				gadget_writel(usb_ss, &usb_ss->regs->usb_cmd,
						reg);
				dev_info(&usb_ss->dev,
					"set test mode, val=0x%x", reg);
				break;
			default:
				return -EINVAL;
			}
			break;

		default:
			return -EINVAL;

		}
		break;
	case USB_RECIP_INTERFACE:
		return cdns_get_setup_ret(usb_ss, ctrl_req);
	case USB_RECIP_ENDPOINT:
		select_ep(usb_ss, ctrl_req->wIndex);
		if (set) {
			/* set stall */
			gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
			EP_CMD__SSTALL__MASK);

			/* handle non zero endpoint software endpoint */
			if (ctrl_req->wIndex & 0x7F) {
				usb_ss_ep = usb_ss->eps[CAST_EP_ADDR_TO_INDEX(
						ctrl_req->wIndex)];
				usb_ss_ep->stalled_flag = 1;
			}
		} else {
			struct usb_request *request;

			if (ctrl_req->wIndex & 0x7F) {
				if (usb_ss->eps[CAST_EP_ADDR_TO_INDEX(
					ctrl_req->wIndex)]->wedge_flag)
					goto jmp_wedge;
			}

			/* clear stall */
			gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
			EP_CMD__CSTALL__MASK | EP_CMD__EPRST__MASK);
			/* wait for EPRST cleared */
			ret = wait_reg_bit_clear(usb_ss, &usb_ss->regs->ep_cmd,
				EP_CMD__EPRST__MASK, 100);

			/* handle non zero endpoint software endpoint */
			if (ctrl_req->wIndex & 0x7F) {
				usb_ss_ep = usb_ss->eps[CAST_EP_ADDR_TO_INDEX(
						ctrl_req->wIndex)];
				usb_ss_ep->stalled_flag = 0;

				request = next_request(
						&usb_ss_ep->request_list);
				if (request)
					cdns_ep_run_transfer(usb_ss_ep);
			}
		}
jmp_wedge:
		select_ep(usb_ss, 0x00);
		break;

	default:
		return -EINVAL;
	}

	gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
		EP_CMD__ERDY__MASK | EP_CMD__REQ_CMPL__MASK);
	return ret;
}

/**
 * cdns_req_ep0_set_sel - Handling of SET_SEL standard USB request
 * @usb_ss: extended gadget object
 * @ctrl_req: pointer to received setup packet
 *
 * Returns 0 if success, error code on error
 */
static int cdns_req_ep0_set_sel(struct usb_ss_dev *usb_ss,
		struct usb_ctrlrequest *ctrl_req)
{
	if (usb_ss->gadget.state < USB_STATE_ADDRESS)
		return -EINVAL;

	if (ctrl_req->wLength != 6) {
		dev_err(&usb_ss->dev, "Set SEL should be 6 bytes, got %d\n",
				ctrl_req->wLength);
		return -EINVAL;
	}

	usb_ss->ep0_data_dir = 0;
	usb_ss->actual_ep0_request = NULL;
	cdns_ep0_run_transfer(usb_ss, usb_ss->setup_dma, 6, 1);
	return 0;
}

/**
 * cdns_req_ep0_set_isoch_delay -
 * Handling of GET_ISOCH_DELAY standard USB request
 * @usb_ss: extended gadget object
 * @ctrl_req: pointer to received setup packet
 *
 * Returns 0 if success, error code on error
 */
static int cdns_req_ep0_set_isoch_delay(struct usb_ss_dev *usb_ss,
		struct usb_ctrlrequest *ctrl_req)
{
	if (ctrl_req->wIndex || ctrl_req->wLength)
		return -EINVAL;

	usb_ss->isoch_delay = ctrl_req->wValue;
	gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
	EP_CMD__ERDY__MASK | EP_CMD__REQ_CMPL__MASK);
	return 0;
}

static void cdns_enable_l1(struct usb_ss_dev *usb_ss, int enable)
{
	if (enable)
		gadget_writel(usb_ss, &usb_ss->regs->usb_conf,
				USB_CONF__L1EN__MASK);
	else
		gadget_writel(usb_ss, &usb_ss->regs->usb_conf,
				USB_CONF__L1DS__MASK);
}

/**
 * cdns_req_ep0_set_configuration - Handling of SET_CONFIG standard USB request
 * @usb_ss: extended gadget object
 * @ctrl_req: pointer to received setup packet
 *
 * Returns 0 if success, 0x7FFF on deferred status stage, error code on error
 */
static int cdns_req_ep0_set_configuration(struct usb_ss_dev *usb_ss,
		struct usb_ctrlrequest *ctrl_req)
{
	enum usb_device_state device_state = usb_ss->gadget.state;
	u32 config = le16_to_cpu(ctrl_req->wValue);
	struct usb_ep *ep;
	struct usb_ss_endpoint *usb_ss_ep, *temp_ss_ep;
	int i, result = 0;

	switch (device_state) {
	case USB_STATE_ADDRESS:
		/* Configure non-control EPs */
		list_for_each_entry_safe(usb_ss_ep, temp_ss_ep,
			&usb_ss->ep_match_list, ep_match_pending_list)
			cdns_ep_config(usb_ss_ep);

		result = cdns_get_setup_ret(usb_ss, ctrl_req);

		if (result != 0)
			return result;

		if (config) {
			if (!usb_ss->hw_configured_flag) {
				/* SET CONFIGURATION */
				gadget_writel(usb_ss, &usb_ss->regs->usb_conf,
					USB_CONF__CFGSET__MASK);
				gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
					EP_CMD__ERDY__MASK |
					EP_CMD__REQ_CMPL__MASK);
				/* wait until configuration set */
				result = wait_reg_bit_set(usb_ss,
					&usb_ss->regs->usb_sts,
					USB_STS__CFGSTS__MASK, 100);
				usb_ss->hw_configured_flag = 1;
				cdns_enable_l1(usb_ss, 1);

				list_for_each_entry(ep,
					&usb_ss->gadget.ep_list,
					ep_list) {
					if (ep->enabled)
						cdns_ep_run_transfer(
							to_usb_ss_ep(ep));
				}
			}
		} else {
			cdns_gadget_unconfig(usb_ss);
			for (i = 0; i < usb_ss->ep_nums; i++)
				usb_ss->eps[i]->endpoint.enabled = 0;
			usb_gadget_set_state(&usb_ss->gadget,
				USB_STATE_ADDRESS);
		}
		break;
	case USB_STATE_CONFIGURED:
		result = cdns_get_setup_ret(usb_ss, ctrl_req);
		if (!config && !result) {
			cdns_gadget_unconfig(usb_ss);
			for (i = 0; i < usb_ss->ep_nums; i++)
				usb_ss->eps[i]->endpoint.enabled = 0;
			usb_gadget_set_state(&usb_ss->gadget,
				USB_STATE_ADDRESS);
		}
		break;
	default:
		result = -EINVAL;
	}

	return result;
}

/**
 * cdns_ep0_standard_request - Handling standard USB requests
 * @usb_ss: extended gadget object
 * @ctrl_req: pointer to received setup packet
 *
 * Returns 0 if success, error code on error
 */
static int cdns_ep0_standard_request(struct usb_ss_dev *usb_ss,
		struct usb_ctrlrequest *ctrl_req)
{
	switch (ctrl_req->bRequest) {
	case USB_REQ_SET_ADDRESS:
		return cdns_req_ep0_set_address(usb_ss, ctrl_req);
	case USB_REQ_SET_CONFIGURATION:
		return cdns_req_ep0_set_configuration(usb_ss, ctrl_req);
	case USB_REQ_GET_STATUS:
		return cdns_req_ep0_get_status(usb_ss, ctrl_req);
	case USB_REQ_CLEAR_FEATURE:
		return cdns_req_ep0_handle_feature(usb_ss, ctrl_req, 0);
	case USB_REQ_SET_FEATURE:
		return cdns_req_ep0_handle_feature(usb_ss, ctrl_req, 1);
	case USB_REQ_SET_SEL:
		return cdns_req_ep0_set_sel(usb_ss, ctrl_req);
	case USB_REQ_SET_ISOCH_DELAY:
		return cdns_req_ep0_set_isoch_delay(usb_ss, ctrl_req);
	default:
		return cdns_get_setup_ret(usb_ss, ctrl_req);
	}
}

/**
 * cdns_ep0_setup_phase - Handling setup USB requests
 * @usb_ss: extended gadget object
 */
static void cdns_ep0_setup_phase(struct usb_ss_dev *usb_ss)
{
	int result;
	struct usb_ctrlrequest *ctrl_req =
			(struct usb_ctrlrequest *)usb_ss->setup;

	if ((ctrl_req->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
		result = cdns_ep0_standard_request(usb_ss, ctrl_req);
	else
		result = cdns_get_setup_ret(usb_ss, ctrl_req);

	if (result != 0 && result != USB_GADGET_DELAYED_STATUS) {
		dev_dbg(&usb_ss->dev, "STALL(00) %d\n", result);
		/* set_stall on ep0 */
		select_ep(usb_ss, 0x00);
		gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
			EP_CMD__SSTALL__MASK);
		gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
			EP_CMD__ERDY__MASK | EP_CMD__REQ_CMPL__MASK);
	}
}

/**
 * cdns_check_ep_interrupt_proceed - Processes interrupt related to endpoint
 * @usb_ss_ep: extended endpoint object
 *
 * Returns 0
 */
static int cdns_check_ep_interrupt_proceed(struct usb_ss_endpoint *usb_ss_ep)
{
	struct usb_ss_dev *usb_ss = usb_ss_ep->usb_ss;
	struct usb_request *request;
	u32 ep_sts_reg;

	select_ep(usb_ss, usb_ss_ep->endpoint.address);
	ep_sts_reg = gadget_readl(usb_ss, &usb_ss->regs->ep_sts);

	dev_dbg(&usb_ss->dev, "EP_STS: %08X\n", ep_sts_reg);

	if (ep_sts_reg & EP_STS__TRBERR__MASK) {
		gadget_writel(usb_ss,
			&usb_ss->regs->ep_sts, EP_STS__TRBERR__MASK);

		dev_dbg(&usb_ss->dev, "TRBERR(%02X)\n",
			usb_ss_ep->endpoint.desc->bEndpointAddress);
	}

	if (ep_sts_reg & EP_STS__ISOERR__MASK) {
		gadget_writel(usb_ss,
			&usb_ss->regs->ep_sts, EP_STS__ISOERR__MASK);
		dev_dbg(&usb_ss->dev, "ISOERR(%02X)\n",
			usb_ss_ep->endpoint.desc->bEndpointAddress);
	}

	if (ep_sts_reg & EP_STS__OUTSMM__MASK) {
		gadget_writel(usb_ss, &usb_ss->regs->ep_sts,
			EP_STS__OUTSMM__MASK);
		dev_dbg(&usb_ss->dev, "OUTSMM(%02X)\n",
			usb_ss_ep->endpoint.desc->bEndpointAddress);
	}

	if (ep_sts_reg & EP_STS__NRDY__MASK) {
		gadget_writel(usb_ss,
			&usb_ss->regs->ep_sts, EP_STS__NRDY__MASK);
		dev_dbg(&usb_ss->dev, "NRDY(%02X)\n",
			usb_ss_ep->endpoint.desc->bEndpointAddress);
	}

	if ((ep_sts_reg & EP_STS__IOC__MASK)
			|| (ep_sts_reg & EP_STS__ISP__MASK)) {
		gadget_writel(usb_ss, &usb_ss->regs->ep_sts,
		EP_STS__IOC__MASK | EP_STS__ISP__MASK);

		/* get just completed request */
		request = next_request(&usb_ss_ep->request_list);
		if (!request)
			return 0;

		if ((request->dma % ADDR_MODULO_8) &&
				(usb_ss_ep->dir == USB_DIR_OUT))
			memcpy(request->buf, usb_ss_ep->cpu_addr,
					request->length);

		usb_gadget_unmap_request_by_dev(usb_ss->sysdev, request,
			usb_ss_ep->endpoint.desc->bEndpointAddress
			& ENDPOINT_DIR_MASK);

		request->status = 0;
		request->actual =
			le32_to_cpu(((u32 *) usb_ss_ep->trb_pool)[1])
			& ACTUAL_TRANSFERRED_BYTES_MASK;

		dev_dbg(&usb_ss->dev, "IOC(%02X) %d\n",
			usb_ss_ep->endpoint.desc->bEndpointAddress,
			request->actual);

		list_del(&request->list);

		usb_ss_ep->hw_pending_flag = 0;
		if (request->complete) {
			spin_unlock(&usb_ss->lock);
			usb_gadget_giveback_request(&usb_ss_ep->endpoint,
				request);
			spin_lock(&usb_ss->lock);
		}

		if (request->buf == usb_ss->zlp_buf)
			kfree(request);

		/* handle deferred STALL */
		if (usb_ss_ep->stalled_flag) {
			cdns_ep_stall_flush(usb_ss_ep);
			return 0;
		}

		/* exit if hardware transfer already started */
		if (usb_ss_ep->hw_pending_flag)
			return 0;

		/* if any request queued run it! */
		if (!list_empty(&usb_ss_ep->request_list))
			cdns_ep_run_transfer(usb_ss_ep);
	}

	if (ep_sts_reg & EP_STS__DESCMIS__MASK) {
		gadget_writel(usb_ss,
			&usb_ss->regs->ep_sts, EP_STS__DESCMIS__MASK);
		dev_dbg(&usb_ss->dev, "DESCMIS(%02X)\n",
			usb_ss_ep->endpoint.desc->bEndpointAddress);
	}

	return 0;
}

/**
 * cdns_check_ep0_interrupt_proceed - Processes interrupt related to endpoint 0
 * @usb_ss: extended gadget object
 * @dir: 1 for IN direction, 0 for OUT direction
 */
static void cdns_check_ep0_interrupt_proceed(struct usb_ss_dev *usb_ss, int dir)
{
	u32 ep_sts_reg;
	int i;

	select_ep(usb_ss, 0 | (dir ? USB_DIR_IN : USB_DIR_OUT));
	ep_sts_reg = gadget_readl(usb_ss, &usb_ss->regs->ep_sts);

	dev_dbg(&usb_ss->dev, "EP_STS: %08X\n", ep_sts_reg);

	if ((ep_sts_reg & EP_STS__SETUP__MASK) && (dir == 0)) {
		dev_dbg(&usb_ss->dev, "SETUP(%02X)\n", 0x00);

		gadget_writel(usb_ss, &usb_ss->regs->ep_sts,
			EP_STS__SETUP__MASK |
			EP_STS__IOC__MASK | EP_STS__ISP__MASK);

		dev_dbg(&usb_ss->dev, "SETUP: ");
		for (i = 0; i < 8; i++)
			dev_dbg(&usb_ss->dev, "%02X ", usb_ss->setup[i]);
		dev_dbg(&usb_ss->dev, "\nSTATE: %d\n", usb_ss->gadget.state);
		usb_ss->ep0_data_dir = usb_ss->setup[0] & USB_DIR_IN;
		cdns_ep0_setup_phase(usb_ss);
		ep_sts_reg &= ~(EP_STS__SETUP__MASK |
			EP_STS__IOC__MASK |
			EP_STS__ISP__MASK);
	}

	if (ep_sts_reg & EP_STS__TRBERR__MASK) {
		gadget_writel(usb_ss,
			&usb_ss->regs->ep_sts, EP_STS__TRBERR__MASK);
		dev_dbg(&usb_ss->dev, "TRBERR(%02X)\n",
			dir ? USB_DIR_IN : USB_DIR_OUT);
	}

	if (ep_sts_reg & EP_STS__DESCMIS__MASK) {
		gadget_writel(usb_ss,
			&usb_ss->regs->ep_sts, EP_STS__DESCMIS__MASK);

		dev_dbg(&usb_ss->dev, "DESCMIS(%02X)\n",
			dir ? USB_DIR_IN : USB_DIR_OUT);

		if (dir == 0 && !usb_ss->setup_pending) {
			usb_ss->ep0_data_dir = 0;
			cdns_ep0_run_transfer(usb_ss,
				usb_ss->setup_dma, 8, 0);
		}
	}

	if ((ep_sts_reg & EP_STS__IOC__MASK)
			|| (ep_sts_reg & EP_STS__ISP__MASK)) {
		gadget_writel(usb_ss,
			&usb_ss->regs->ep_sts, EP_STS__IOC__MASK);
		if (usb_ss->actual_ep0_request) {
			usb_gadget_unmap_request_by_dev(usb_ss->sysdev,
					usb_ss->actual_ep0_request,
					usb_ss->ep0_data_dir);

			usb_ss->actual_ep0_request->actual =
				le32_to_cpu((usb_ss->trb_ep0)[1])
				& ACTUAL_TRANSFERRED_BYTES_MASK;

			dev_dbg(&usb_ss->dev, "IOC(%02X) %d\n",
				dir ? USB_DIR_IN : USB_DIR_OUT,
				usb_ss->actual_ep0_request->actual);
		}

		if (usb_ss->actual_ep0_request
				&& usb_ss->actual_ep0_request->complete) {
			spin_unlock(&usb_ss->lock);
			usb_ss->actual_ep0_request->complete(usb_ss->gadget.ep0,
					usb_ss->actual_ep0_request);
			spin_lock(&usb_ss->lock);
		}
		cdns_prepare_setup_packet(usb_ss);
		gadget_writel(usb_ss,
			&usb_ss->regs->ep_cmd, EP_CMD__REQ_CMPL__MASK);
	}
}

/**
 * cdns_check_usb_interrupt_proceed - Processes interrupt related to device
 * @usb_ss: extended gadget object
 * @usb_ists: bitmap representation of device's reported interrupts
 * (usb_ists register value)
 */
static void cdns_check_usb_interrupt_proceed(struct usb_ss_dev *usb_ss,
		u32 usb_ists)
{
	int interrupt_bit = ffs(usb_ists) - 1;
	int speed;
	u32 val;

	dev_dbg(&usb_ss->dev, "USB interrupt detected\n");

	switch (interrupt_bit) {
	case USB_ISTS__CON2I__SHIFT:
		/* FS/HS Connection detected */
		dev_dbg(&usb_ss->dev,
			"[Interrupt] FS/HS Connection detected\n");
		val = gadget_readl(usb_ss, &usb_ss->regs->usb_sts);
		speed = USB_STS__USBSPEED__READ(val);
		if (speed == USB_SPEED_WIRELESS)
			speed = USB_SPEED_SUPER;
		dev_dbg(&usb_ss->dev, "Speed value: %s (%d), usbsts:0x%x\n",
			usb_speed_string(speed), speed, val);
		usb_ss->gadget.speed = speed;
		usb_ss->is_connected = 1;
		usb_gadget_set_state(&usb_ss->gadget, USB_STATE_POWERED);
		cdns_ep0_config(usb_ss);
		break;
	case USB_ISTS__CONI__SHIFT:
		/* SS Connection detected */
		dev_dbg(&usb_ss->dev, "[Interrupt] SS Connection detected\n");
		val = gadget_readl(usb_ss, &usb_ss->regs->usb_sts);
		speed = USB_STS__USBSPEED__READ(val);
		if (speed == USB_SPEED_WIRELESS)
			speed = USB_SPEED_SUPER;
		dev_dbg(&usb_ss->dev, "Speed value: %s (%d), usbsts:0x%x\n",
			usb_speed_string(speed), speed, val);
		usb_ss->gadget.speed = speed;
		usb_ss->is_connected = 1;
		usb_gadget_set_state(&usb_ss->gadget, USB_STATE_POWERED);
		cdns_ep0_config(usb_ss);
		break;
	case USB_ISTS__DIS2I__SHIFT:
	case USB_ISTS__DISI__SHIFT:
		/* SS Disconnection detected */
		val = gadget_readl(usb_ss, &usb_ss->regs->usb_sts);
		dev_dbg(&usb_ss->dev,
			"[Interrupt] Disconnection detected: usbsts:0x%x\n",
			val);
		if (usb_ss->gadget_driver
			&& usb_ss->gadget_driver->disconnect) {

			spin_unlock(&usb_ss->lock);
			usb_ss->gadget_driver->disconnect(&usb_ss->gadget);
			spin_lock(&usb_ss->lock);
		}
		usb_ss->gadget.speed = USB_SPEED_UNKNOWN;
		usb_gadget_set_state(&usb_ss->gadget, USB_STATE_NOTATTACHED);
		usb_ss->is_connected = 0;
		cdns_gadget_unconfig(usb_ss);
		break;
	case USB_ISTS__L2ENTI__SHIFT:
		dev_dbg(&usb_ss->dev,
			 "[Interrupt] Device suspended\n");
		break;
	case USB_ISTS__L2EXTI__SHIFT:
		dev_dbg(&usb_ss->dev, "[Interrupt] L2 exit detected\n");
		/*
		 * Exit from standby mode
		 * on L2 exit (Suspend in HS/FS or SS)
		 */
		break;
	case USB_ISTS__U3EXTI__SHIFT:
		/*
		 * Exit from standby mode
		 * on U3 exit (Suspend in HS/FS or SS)
		 */
		dev_dbg(&usb_ss->dev, "[Interrupt] U3 exit detected\n");
		break;

		/* resets cases */
	case USB_ISTS__UWRESI__SHIFT:
	case USB_ISTS__UHRESI__SHIFT:
	case USB_ISTS__U2RESI__SHIFT:
		dev_dbg(&usb_ss->dev, "[Interrupt] Reset detected\n");
		speed = USB_STS__USBSPEED__READ(
				gadget_readl(usb_ss, &usb_ss->regs->usb_sts));
		if (speed == USB_SPEED_WIRELESS)
			speed = USB_SPEED_SUPER;
		usb_gadget_set_state(&usb_ss->gadget, USB_STATE_DEFAULT);
		usb_ss->gadget.speed = speed;
		cdns_gadget_unconfig(usb_ss);
		cdns_ep0_config(usb_ss);
		break;
	default:
		break;
	}

	/* Clear interrupt bit */
	gadget_writel(usb_ss, &usb_ss->regs->usb_ists, (1uL << interrupt_bit));
}

/**
 * cdns_irq_handler - irq line interrupt handler
 * @cdns: cdns3 instance
 *
 * Returns IRQ_HANDLED when interrupt raised by USBSS_DEV,
 * IRQ_NONE when interrupt raised by other device connected
 * to the irq line
 */
static irqreturn_t cdns_irq_handler_thread(struct cdns3 *cdns)
{
	struct usb_ss_dev *usb_ss =
		container_of(cdns->gadget_dev, struct usb_ss_dev, dev);
	u32 reg;
	enum irqreturn ret = IRQ_NONE;
	unsigned long flags;

	spin_lock_irqsave(&usb_ss->lock, flags);

	/* check USB device interrupt */
	reg = gadget_readl(usb_ss, &usb_ss->regs->usb_ists);
	if (reg) {
		dev_dbg(&usb_ss->dev, "usb_ists: %08X\n", reg);
		cdns_check_usb_interrupt_proceed(usb_ss, reg);
		ret = IRQ_HANDLED;
	}

	/* check endpoint interrupt */
	reg = gadget_readl(usb_ss, &usb_ss->regs->ep_ists);
	if (reg != 0) {
		dev_dbg(&usb_ss->dev, "ep_ists: %08X\n", reg);
	} else {
		if (gadget_readl(usb_ss, &usb_ss->regs->usb_sts) &
				USB_STS__CFGSTS__MASK)
			ret = IRQ_HANDLED;
		goto irqend;
	}

	/* handle default endpoint OUT */
	if (reg & EP_ISTS__EOUT0__MASK) {
		cdns_check_ep0_interrupt_proceed(usb_ss, 0);
		ret = IRQ_HANDLED;
	}

	/* handle default endpoint IN */
	if (reg & EP_ISTS__EIN0__MASK) {
		cdns_check_ep0_interrupt_proceed(usb_ss, 1);
		ret = IRQ_HANDLED;
	}

	/* check if interrupt from non default endpoint, if no exit */
	reg &= ~(EP_ISTS__EOUT0__MASK | EP_ISTS__EIN0__MASK);
	if (!reg)
		goto irqend;

	do {
		unsigned int bit_pos = ffs(reg);
		u32 bit_mask = 1 << (bit_pos - 1);

		dev_dbg(&usb_ss->dev, "Interrupt on index: %d bitmask %08X\n",
				CAST_EP_REG_POS_TO_INDEX(bit_pos), bit_mask);
		cdns_check_ep_interrupt_proceed(
				usb_ss->eps[CAST_EP_REG_POS_TO_INDEX(bit_pos)]);
		reg &= ~bit_mask;
		ret = IRQ_HANDLED;
	} while (reg);

irqend:
	spin_unlock_irqrestore(&usb_ss->lock, flags);
	return ret;
}

/**
 * usb_ss_gadget_ep0_enable
 * Function shouldn't be called by gadget driver,
 * endpoint 0 is allways active
 */
static int usb_ss_gadget_ep0_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	return -EINVAL;
}

/**
 * usb_ss_gadget_ep0_disable
 * Function shouldn't be called by gadget driver,
 * endpoint 0 is allways active
 */
static int usb_ss_gadget_ep0_disable(struct usb_ep *ep)
{
	return -EINVAL;
}

/**
 * usb_ss_gadget_ep0_set_halt
 * @ep: pointer to endpoint zero object
 * @value: 1 for set stall, 0 for clear stall
 *
 * Returns 0
 */
static int usb_ss_gadget_ep0_set_halt(struct usb_ep *ep, int value)
{
	/* TODO */
	return 0;
}

/**
 * usb_ss_gadget_ep0_queue Transfer data on endpoint zero
 * @ep: pointer to endpoint zero object
 * @request: pointer to request object
 * @gfp_flags: gfp flags
 *
 * Returns 0 on success, error code elsewhere
 */
static int usb_ss_gadget_ep0_queue(struct usb_ep *ep,
		struct usb_request *request, gfp_t gfp_flags)
{
	int ret = 0;
	unsigned long flags;
	int erdy_sent = 0;
	/* get extended endpoint */
	struct usb_ss_endpoint *usb_ss_ep =
		to_usb_ss_ep(ep);
	struct usb_ss_dev *usb_ss = usb_ss_ep->usb_ss;

	dev_dbg(&usb_ss->dev, "QUEUE(%02X) %d\n",
		usb_ss->ep0_data_dir ? USB_DIR_IN : USB_DIR_OUT,
		request->length);

	/* send STATUS stage */
	if (request->length == 0 && request->zero == 0) {
		spin_lock_irqsave(&usb_ss->lock, flags);
		select_ep(usb_ss, 0x00);
		if (!usb_ss->hw_configured_flag) {
			gadget_writel(usb_ss, &usb_ss->regs->usb_conf,
			USB_CONF__CFGSET__MASK); /* SET CONFIGURATION */
			cdns_prepare_setup_packet(usb_ss);
			gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
				EP_CMD__ERDY__MASK | EP_CMD__REQ_CMPL__MASK);
			/* wait until configuration set */
			ret = wait_reg_bit_set(usb_ss, &usb_ss->regs->usb_sts,
				USB_STS__CFGSTS__MASK, 100);
			erdy_sent = 1;
			usb_ss->hw_configured_flag = 1;
			cdns_enable_l1(usb_ss, 1);

			list_for_each_entry(ep,
				&usb_ss->gadget.ep_list,
				ep_list) {

				if (ep->enabled)
					cdns_ep_run_transfer(
						to_usb_ss_ep(ep));
			}
		}
		if (!erdy_sent)
			gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
			EP_CMD__ERDY__MASK | EP_CMD__REQ_CMPL__MASK);
		if (request->complete)
			request->complete(usb_ss->gadget.ep0, request);
		spin_unlock_irqrestore(&usb_ss->lock, flags);
		return 0;
	}

	spin_lock_irqsave(&usb_ss->lock, flags);
	ret = usb_gadget_map_request_by_dev(usb_ss->sysdev, request,
			usb_ss->ep0_data_dir);
	if (ret) {
		spin_unlock_irqrestore(&usb_ss->lock, flags);
		dev_err(&usb_ss->dev, "failed to map request\n");
		return -EINVAL;
	}

	usb_ss->actual_ep0_request = request;
	cdns_ep0_run_transfer(usb_ss, request->dma, request->length, 1);
	spin_unlock_irqrestore(&usb_ss->lock, flags);
	return ret;
}
/**
 * ep_onchip_buffer_alloc - Try to allocate onchip buf for EP
 *
 * The real allocation will occur during write to EP_CFG register,
 * this function is used to check if the 'size' allocation is allowed.
 *
 * @usb_ss: extended gadget object
 * @size: the size (KB) for EP would like to allocate
 * @is_in: the direction for EP
 *
 * Return 0 if the later allocation is allowed or negative value on failure
 */

static int ep_onchip_buffer_alloc(struct usb_ss_dev *usb_ss,
		int size, int is_in)
{
	if (is_in) {
		usb_ss->onchip_mem_allocated_size += size;
	} else if (!usb_ss->out_mem_is_allocated) {
		 /* ALL OUT EPs are shared the same chunk onchip memory */
		usb_ss->onchip_mem_allocated_size += size;
		usb_ss->out_mem_is_allocated = 1;
	}

	if (usb_ss->onchip_mem_allocated_size > CDNS3_ONCHIP_BUF_SIZE) {
		usb_ss->onchip_mem_allocated_size -= size;
		return -EPERM;
	} else {
		return 0;
	}
}

/**
 * cdns_ep_config Configure hardware endpoint
 * @usb_ss_ep: extended endpoint object
 */
static void cdns_ep_config(struct usb_ss_endpoint *usb_ss_ep)
{
	struct usb_ss_dev *usb_ss = usb_ss_ep->usb_ss;
	u32 ep_cfg = 0;
	u32 max_packet_size = 0;
	u32 bEndpointAddress = usb_ss_ep->num | usb_ss_ep->dir;
	u32 interrupt_mask = 0;
	int is_in = !!usb_ss_ep->dir;
	bool is_iso_ep = (usb_ss_ep->type == USB_ENDPOINT_XFER_ISOC);
	int default_buf_size = CDNS3_EP_BUF_SIZE;

	dev_dbg(&usb_ss->dev, "%s: %s addr=0x%x\n", __func__,
			usb_ss_ep->name, bEndpointAddress);

	if (is_iso_ep) {
		ep_cfg = EP_CFG__EPTYPE__WRITE(USB_ENDPOINT_XFER_ISOC);
		interrupt_mask = INTERRUPT_MASK;
	} else {
		ep_cfg = EP_CFG__EPTYPE__WRITE(USB_ENDPOINT_XFER_BULK);
	}

	switch (usb_ss->gadget.speed) {
	case USB_SPEED_UNKNOWN:
		max_packet_size = ENDPOINT_MAX_PACKET_SIZE_0;
		break;
	case USB_SPEED_LOW:
		max_packet_size = ENDPOINT_MAX_PACKET_SIZE_8;
		break;
	case USB_SPEED_FULL:
		max_packet_size = (is_iso_ep ?
			ENDPOINT_MAX_PACKET_SIZE_1023 :
			ENDPOINT_MAX_PACKET_SIZE_64);
		break;
	case USB_SPEED_HIGH:
		max_packet_size = (is_iso_ep ?
			ENDPOINT_MAX_PACKET_SIZE_1024 :
			ENDPOINT_MAX_PACKET_SIZE_512);
		break;
	case USB_SPEED_WIRELESS:
		max_packet_size = ENDPOINT_MAX_PACKET_SIZE_512;
		break;
	case USB_SPEED_SUPER:
		max_packet_size = ENDPOINT_MAX_PACKET_SIZE_1024;
		break;
	case USB_SPEED_SUPER_PLUS:
		dev_warn(&usb_ss->dev, "USB 3.1 is not supported\n");
		max_packet_size = ENDPOINT_MAX_PACKET_SIZE_1024;
		break;
	}

	if (ep_onchip_buffer_alloc(usb_ss, default_buf_size, is_in)) {
		dev_err(&usb_ss->dev, "onchip mem is full, ep is invalid\n");
		return;
	}

	ep_cfg |= EP_CFG__MAXPKTSIZE__WRITE(max_packet_size) |
		EP_CFG__BUFFERING__WRITE(default_buf_size - 1) |
		EP_CFG__MAXBURST__WRITE(usb_ss_ep->endpoint.maxburst);

	select_ep(usb_ss, bEndpointAddress);
	gadget_writel(usb_ss, &usb_ss->regs->ep_cfg, ep_cfg);
	gadget_writel(usb_ss, &usb_ss->regs->ep_sts_en,
		EP_STS_EN__TRBERREN__MASK | interrupt_mask);

	/* enable interrupt for selected endpoint */
	ep_cfg = gadget_readl(usb_ss, &usb_ss->regs->ep_ien);
	ep_cfg |= CAST_EP_ADDR_TO_BIT_POS(bEndpointAddress);
	gadget_writel(usb_ss, &usb_ss->regs->ep_ien, ep_cfg);
}

/**
 * usb_ss_gadget_ep_enable Enable endpoint
 * @ep: endpoint object
 * @desc: endpoint descriptor
 *
 * Returns 0 on success, error code elsewhere
 */
static int usb_ss_gadget_ep_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct usb_ss_endpoint *usb_ss_ep;
	struct usb_ss_dev *usb_ss;
	unsigned long flags;
	int ret;
	u32 ep_cfg;

	usb_ss_ep = to_usb_ss_ep(ep);
	usb_ss = usb_ss_ep->usb_ss;

	if (!ep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT) {
		dev_err(&usb_ss->dev, "usb-ss: invalid parameters\n");
		return -EINVAL;
	}

	if (!desc->wMaxPacketSize) {
		dev_err(&usb_ss->dev, "usb-ss: missing wMaxPacketSize\n");
		return -EINVAL;
	}

	ret = usb_ss_allocate_trb_pool(usb_ss_ep);
	if (ret)
		return ret;

	if (!usb_ss_ep->cpu_addr) {
		usb_ss_ep->cpu_addr = dma_alloc_coherent(usb_ss->sysdev,
				CDNS3_UNALIGNED_BUF_SIZE,
				&usb_ss_ep->dma_addr, GFP_DMA);
		if (!usb_ss_ep->cpu_addr)
			return -ENOMEM;
	}

	dev_dbg(&usb_ss->dev, "Enabling endpoint: %s, addr=0x%x\n",
		ep->name, desc->bEndpointAddress);
	spin_lock_irqsave(&usb_ss->lock, flags);
	select_ep(usb_ss, desc->bEndpointAddress);
	gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
		EP_CMD__EPRST__MASK);
	ret = wait_reg_bit_clear(usb_ss, &usb_ss->regs->ep_cmd,
		EP_CMD__EPRST__MASK, 100);
	ep_cfg = gadget_readl(usb_ss, &usb_ss->regs->ep_cfg);
	ep_cfg |= EP_CFG__ENABLE__MASK;
	gadget_writel(usb_ss, &usb_ss->regs->ep_cfg, ep_cfg);

	ep->enabled = 1;
	ep->desc = desc;
	usb_ss_ep->hw_pending_flag = 0;
	usb_ss_ep->stalled_flag = 0;
	spin_unlock_irqrestore(&usb_ss->lock, flags);
	return 0;
}

/* Find correct direction for HW endpoint according to description */
static int ep_dir_is_correct(struct usb_endpoint_descriptor *desc,
	struct usb_ss_endpoint *usb_ss_ep)
{
	return (usb_ss_ep->endpoint.caps.dir_in &&
			!!(desc->bEndpointAddress & USB_DIR_IN))
			|| (usb_ss_ep->endpoint.caps.dir_out
			&& ((desc->bEndpointAddress & 0x80) == USB_DIR_OUT));
}

static struct usb_ss_endpoint *find_available_ss_ep(
		struct usb_ss_dev *usb_ss,
		struct usb_endpoint_descriptor *desc)
{
	struct usb_ep *ep;
	struct usb_ss_endpoint *usb_ss_ep;

	list_for_each_entry(ep, &usb_ss->gadget.ep_list, ep_list) {
		unsigned long num;
		int ret;
		/* ep name pattern likes epXin or epXout */
		char c[2] = {ep->name[2], '\0'};

		ret = kstrtoul(c, 10, &num);
		if (ret)
			return ERR_PTR(ret);

		usb_ss_ep = to_usb_ss_ep(ep);
		if (ep_dir_is_correct(desc, usb_ss_ep)) {
			if (!usb_ss_ep->used) {
				usb_ss_ep->num  = num;
				usb_ss_ep->used = true;
				return usb_ss_ep;
			}
		}
	}
	return ERR_PTR(-ENOENT);
}

static struct usb_ep *usb_ss_gadget_match_ep(struct usb_gadget *gadget,
		struct usb_endpoint_descriptor *desc,
		struct usb_ss_ep_comp_descriptor *comp_desc)
{
	struct usb_ss_dev *usb_ss = gadget_to_usb_ss(gadget);
	struct usb_ss_endpoint *usb_ss_ep;
	unsigned long flags;

	usb_ss_ep = find_available_ss_ep(usb_ss, desc);
	if (IS_ERR(usb_ss_ep)) {
		dev_err(&usb_ss->dev, "no available ep\n");
		return NULL;
	}

	dev_dbg(&usb_ss->dev, "match endpoint: %s\n", usb_ss_ep->name);
	spin_lock_irqsave(&usb_ss->lock, flags);
	usb_ss_ep->endpoint.desc = desc;
	usb_ss_ep->dir  = usb_endpoint_dir_in(desc) ? USB_DIR_IN : USB_DIR_OUT;
	usb_ss_ep->type = usb_endpoint_type(desc);

	list_add_tail(&usb_ss_ep->ep_match_pending_list,
			&usb_ss->ep_match_list);
	spin_unlock_irqrestore(&usb_ss->lock, flags);
	return &usb_ss_ep->endpoint;
}

static void usb_ss_free_trb_pool(struct usb_ss_endpoint *usb_ss_ep)
{
	struct usb_ss_dev *usb_ss = usb_ss_ep->usb_ss;

	if (usb_ss_ep->trb_pool) {
		dma_free_coherent(usb_ss->sysdev,
			sizeof(struct usb_ss_trb) * USB_SS_TRBS_NUM,
			usb_ss_ep->trb_pool, usb_ss_ep->trb_pool_dma);
		usb_ss_ep->trb_pool = NULL;
	}

	if (usb_ss_ep->cpu_addr) {
		dma_free_coherent(usb_ss->sysdev, CDNS3_UNALIGNED_BUF_SIZE,
			usb_ss_ep->cpu_addr, usb_ss_ep->dma_addr);
		usb_ss_ep->cpu_addr = NULL;
	}
}

/**
 * usb_ss_gadget_ep_disable Disable endpoint
 * @ep: endpoint object
 *
 * Returns 0 on success, error code elsewhere
 */
static int usb_ss_gadget_ep_disable(struct usb_ep *ep)
{
	struct usb_ss_endpoint *usb_ss_ep;
	struct usb_ss_dev *usb_ss;
	unsigned long flags;
	int ret = 0;
	struct usb_request *request;
	u32 ep_cfg;

	if (!ep) {
		pr_debug("usb-ss: invalid parameters\n");
		return -EINVAL;
	}

	usb_ss_ep = to_usb_ss_ep(ep);
	usb_ss = usb_ss_ep->usb_ss;

	spin_lock_irqsave(&usb_ss->lock, flags);
	if (!usb_ss->start_gadget) {
		dev_dbg(&usb_ss->dev,
			"Disabling endpoint at disconnection: %s\n", ep->name);
		spin_unlock_irqrestore(&usb_ss->lock, flags);
		return 0;
	}

	dev_dbg(&usb_ss->dev,
		"Disabling endpoint: %s\n", ep->name);

	select_ep(usb_ss, ep->desc->bEndpointAddress);
	ret = cdns_data_flush(usb_ss_ep);
	while (!list_empty(&usb_ss_ep->request_list)) {

		request = next_request(&usb_ss_ep->request_list);
		usb_gadget_unmap_request_by_dev(usb_ss->sysdev, request,
				ep->desc->bEndpointAddress & USB_DIR_IN);
		request->status = -ESHUTDOWN;
		list_del(&request->list);
		spin_unlock(&usb_ss->lock);
		usb_gadget_giveback_request(ep, request);
		spin_lock(&usb_ss->lock);
	}

	ep_cfg = gadget_readl(usb_ss, &usb_ss->regs->ep_cfg);
	ep_cfg &= ~EP_CFG__ENABLE__MASK;
	gadget_writel(usb_ss, &usb_ss->regs->ep_cfg, ep_cfg);
	ep->desc = NULL;
	ep->enabled = 0;

	spin_unlock_irqrestore(&usb_ss->lock, flags);

	return ret;
}

/**
 * usb_ss_gadget_ep_alloc_request Allocates request
 * @ep: endpoint object associated with request
 * @gfp_flags: gfp flags
 *
 * Returns allocated request address, NULL on allocation error
 */
static struct usb_request *usb_ss_gadget_ep_alloc_request(struct usb_ep *ep,
		gfp_t gfp_flags)
{
	struct usb_request *request;

	request = kzalloc(sizeof(struct usb_request), gfp_flags);
	if (!request)
		return NULL;

	return request;
}

/**
 * usb_ss_gadget_ep_free_request Free memory occupied by request
 * @ep: endpoint object associated with request
 * @request: request to free memory
 */
static void usb_ss_gadget_ep_free_request(struct usb_ep *ep,
		struct usb_request *request)
{
	kfree(request);
}

/**
 * usb_ss_gadget_ep_queue Transfer data on endpoint
 * @ep: endpoint object
 * @request: request object
 * @gfp_flags: gfp flags
 *
 * Returns 0 on success, error code elsewhere
 */
static int __usb_ss_gadget_ep_queue(struct usb_ep *ep,
		struct usb_request *request, gfp_t gfp_flags)
{
	struct usb_ss_endpoint *usb_ss_ep =
		to_usb_ss_ep(ep);
	struct usb_ss_dev *usb_ss = usb_ss_ep->usb_ss;
	int ret = 0;
	int empty_list = 0;

	request->actual = 0;
	request->status = -EINPROGRESS;

	dev_dbg(&usb_ss->dev,
		"Queuing endpoint: %s\n", usb_ss_ep->name);

	dev_dbg(&usb_ss->dev, "QUEUE(%02X) %d\n",
		ep->desc->bEndpointAddress, request->length);

	ret = usb_gadget_map_request_by_dev(usb_ss->sysdev, request,
			ep->desc->bEndpointAddress & USB_DIR_IN);

	if (ret)
		return ret;

	empty_list = list_empty(&usb_ss_ep->request_list);
	list_add_tail(&request->list, &usb_ss_ep->request_list);

	if (!usb_ss->hw_configured_flag)
		return 0;

	if (empty_list) {
		if (!usb_ss_ep->stalled_flag)
			ret = cdns_ep_run_transfer(usb_ss_ep);
	}

	return ret;
}

static int usb_ss_gadget_ep_queue(struct usb_ep *ep,
		struct usb_request *request, gfp_t gfp_flags)
{
	struct usb_ss_endpoint *usb_ss_ep = to_usb_ss_ep(ep);
	struct usb_ss_dev *usb_ss = usb_ss_ep->usb_ss;
	struct usb_request *zlp_request;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&usb_ss->lock, flags);

	ret = __usb_ss_gadget_ep_queue(ep, request, gfp_flags);
	if (ret == 0 && request->zero && request->length &&
			(request->length % ep->maxpacket == 0)) {
		zlp_request = usb_ss_gadget_ep_alloc_request(ep, GFP_ATOMIC);
		zlp_request->length = 0;
		zlp_request->buf = usb_ss->zlp_buf;

		dev_dbg(&usb_ss->dev, "Queuing ZLP for endpoint: %s\n",
			usb_ss_ep->name);
		ret = __usb_ss_gadget_ep_queue(ep, zlp_request, gfp_flags);
	}

	spin_unlock_irqrestore(&usb_ss->lock, flags);
	return ret;
}

/**
 * usb_ss_gadget_ep_dequeue Remove request from transfer queue
 * @ep: endpoint object associated with request
 * @request: request object
 *
 * Returns 0 on success, error code elsewhere
 */
static int usb_ss_gadget_ep_dequeue(struct usb_ep *ep,
		struct usb_request *request)
{
	struct usb_ss_endpoint *usb_ss_ep =
		to_usb_ss_ep(ep);
	struct usb_ss_dev *usb_ss = usb_ss_ep->usb_ss;
	unsigned long flags;
	struct usb_request *req, *req_temp;
	int ret = 0;

	if (ep == NULL || request == NULL || ep->desc == NULL)
		return -EINVAL;

	spin_lock_irqsave(&usb_ss->lock, flags);
	dev_dbg(&usb_ss->dev, "DEQUEUE(%02X) %d\n",
		ep->address, request->length);
	usb_gadget_unmap_request_by_dev(usb_ss->sysdev, request,
		ep->address & USB_DIR_IN);
	request->status = -ECONNRESET;

	select_ep(usb_ss, ep->desc->bEndpointAddress);
	ret = cdns_data_flush(usb_ss_ep);
	if (ep->address) {
		list_for_each_entry_safe(req, req_temp,
			&usb_ss_ep->request_list, list) {
			if (request == req) {
				list_del_init(&request->list);
				if (request->complete) {
					spin_unlock(&usb_ss->lock);
					usb_gadget_giveback_request
						(&usb_ss_ep->endpoint, request);
					spin_lock(&usb_ss->lock);
				}
				break;
			}
		}
	}

	spin_unlock_irqrestore(&usb_ss->lock, flags);
	return ret;
}

/**
 * usb_ss_gadget_ep_set_halt Sets/clears stall on selected endpoint
 * @ep: endpoint object to set/clear stall on
 * @value: 1 for set stall, 0 for clear stall
 *
 * Returns 0 on success, error code elsewhere
 */
static int usb_ss_gadget_ep_set_halt(struct usb_ep *ep, int value)
{
	struct usb_ss_endpoint *usb_ss_ep =
		to_usb_ss_ep(ep);
	struct usb_ss_dev *usb_ss = usb_ss_ep->usb_ss;
	unsigned long flags;
	int ret = 0;

	/* return error when endpoint disabled */
	if (!ep->enabled)
		return -EPERM;

	/* if actual transfer is pending defer setting stall on this endpoint */
	if (usb_ss_ep->hw_pending_flag && value) {
		usb_ss_ep->stalled_flag = 1;
		return 0;
	}

	dev_dbg(&usb_ss->dev, "HALT(%02X) %d\n", ep->address, value);

	spin_lock_irqsave(&usb_ss->lock, flags);

	select_ep(usb_ss, ep->desc->bEndpointAddress);
	if (value) {
		cdns_ep_stall_flush(usb_ss_ep);
	} else {
		/*
		 * TODO:
		 * epp->wedgeFlag = 0;
		 */
		usb_ss_ep->wedge_flag = 0;
		gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
		EP_CMD__CSTALL__MASK | EP_CMD__EPRST__MASK);
		/* wait for EPRST cleared */
		ret = wait_reg_bit_clear(usb_ss, &usb_ss->regs->ep_cmd,
			EP_CMD__EPRST__MASK, 100);
		usb_ss_ep->stalled_flag = 0;
	}
	usb_ss_ep->hw_pending_flag = 0;
	spin_unlock_irqrestore(&usb_ss->lock, flags);

	return ret;
}

/**
 * usb_ss_gadget_ep_set_wedge Set wedge on selected endpoint
 * @ep: endpoint object
 *
 * Returns 0
 */
static int usb_ss_gadget_ep_set_wedge(struct usb_ep *ep)
{
	struct usb_ss_endpoint *usb_ss_ep = to_usb_ss_ep(ep);
	struct usb_ss_dev *usb_ss = usb_ss_ep->usb_ss;

	dev_dbg(&usb_ss->dev, "WEDGE(%02X)\n", ep->address);
	usb_ss_gadget_ep_set_halt(ep, 1);
	usb_ss_ep->wedge_flag = 1;
	return 0;
}

static const struct usb_ep_ops usb_ss_gadget_ep0_ops = {
	.enable = usb_ss_gadget_ep0_enable,
	.disable = usb_ss_gadget_ep0_disable,
	.alloc_request = usb_ss_gadget_ep_alloc_request,
	.free_request = usb_ss_gadget_ep_free_request,
	.queue = usb_ss_gadget_ep0_queue,
	.dequeue = usb_ss_gadget_ep_dequeue,
	.set_halt = usb_ss_gadget_ep0_set_halt,
	.set_wedge = usb_ss_gadget_ep_set_wedge,
};

static const struct usb_ep_ops usb_ss_gadget_ep_ops = {
	.enable = usb_ss_gadget_ep_enable,
	.disable = usb_ss_gadget_ep_disable,
	.alloc_request = usb_ss_gadget_ep_alloc_request,
	.free_request = usb_ss_gadget_ep_free_request,
	.queue = usb_ss_gadget_ep_queue,
	.dequeue = usb_ss_gadget_ep_dequeue,
	.set_halt = usb_ss_gadget_ep_set_halt,
	.set_wedge = usb_ss_gadget_ep_set_wedge,
};

/**
 * usb_ss_gadget_get_frame Returns number of actual ITP frame
 * @gadget: gadget object
 *
 * Returns number of actual ITP frame
 */
static int usb_ss_gadget_get_frame(struct usb_gadget *gadget)
{
	struct usb_ss_dev *usb_ss = gadget_to_usb_ss(gadget);

	dev_dbg(&usb_ss->dev, "usb_ss_gadget_get_frame\n");
	return gadget_readl(usb_ss, &usb_ss->regs->usb_iptn);
}

static int usb_ss_gadget_wakeup(struct usb_gadget *gadget)
{
	struct usb_ss_dev *usb_ss = gadget_to_usb_ss(gadget);

	dev_dbg(&usb_ss->dev, "usb_ss_gadget_wakeup\n");
	return 0;
}

static int usb_ss_gadget_set_selfpowered(struct usb_gadget *gadget,
		int is_selfpowered)
{
	struct usb_ss_dev *usb_ss = gadget_to_usb_ss(gadget);
	unsigned long flags;

	dev_dbg(&usb_ss->dev, "usb_ss_gadget_set_selfpowered: %d\n",
		is_selfpowered);

	spin_lock_irqsave(&usb_ss->lock, flags);
	gadget->is_selfpowered = !!is_selfpowered;
	spin_unlock_irqrestore(&usb_ss->lock, flags);
	return 0;
}

static int usb_ss_gadget_pullup(struct usb_gadget *gadget, int is_on)
{
	struct usb_ss_dev *usb_ss = gadget_to_usb_ss(gadget);

	if (!usb_ss->start_gadget)
		return 0;

	dev_dbg(&usb_ss->dev, "usb_ss_gadget_pullup: %d\n", is_on);

	if (is_on)
		gadget_writel(usb_ss, &usb_ss->regs->usb_conf,
				USB_CONF__DEVEN__MASK);
	else
		gadget_writel(usb_ss, &usb_ss->regs->usb_conf,
				USB_CONF__DEVDS__MASK);

	return 0;
}

/**
 * usb_ss_gadget_udc_start Gadget start
 * @gadget: gadget object
 * @driver: driver which operates on this gadget
 *
 * Returns 0 on success, error code elsewhere
 */
static int usb_ss_gadget_udc_start(struct usb_gadget *gadget,
		struct usb_gadget_driver *driver)
{
	struct usb_ss_dev *usb_ss = gadget_to_usb_ss(gadget);
	unsigned long flags;

	if (usb_ss->gadget_driver) {
		dev_err(&usb_ss->dev, "%s is already bound to %s\n",
				usb_ss->gadget.name,
				usb_ss->gadget_driver->driver.name);
		return -EBUSY;
	}

	dev_dbg(&usb_ss->dev, "%s begins\n", __func__);

	spin_lock_irqsave(&usb_ss->lock, flags);
	usb_ss->gadget_driver = driver;
	if (!usb_ss->start_gadget) {
		spin_unlock_irqrestore(&usb_ss->lock, flags);
		return 0;
	}

	__cdns3_gadget_start(usb_ss);
	spin_unlock_irqrestore(&usb_ss->lock, flags);
	dev_dbg(&usb_ss->dev, "%s ends\n", __func__);
	return 0;
}

/**
 * usb_ss_gadget_udc_stop Stops gadget
 * @gadget: gadget object
 *
 * Returns 0
 */
static int usb_ss_gadget_udc_stop(struct usb_gadget *gadget)
{
	struct usb_ss_dev *usb_ss = gadget_to_usb_ss(gadget);
	struct usb_ep *ep;
	struct usb_ss_endpoint *usb_ss_ep, *temp_ss_ep;
	int i;
	u32 bEndpointAddress;
	int ret = 0;

	usb_ss->gadget_driver = NULL;
	list_for_each_entry_safe(usb_ss_ep, temp_ss_ep,
		&usb_ss->ep_match_list, ep_match_pending_list) {
		list_del(&usb_ss_ep->ep_match_pending_list);
		usb_ss_ep->used = false;
	}

	usb_ss->onchip_mem_allocated_size = 0;
	usb_ss->out_mem_is_allocated = 0;
	if (!usb_ss->start_gadget)
		return 0;

	list_for_each_entry(ep, &usb_ss->gadget.ep_list, ep_list) {
		usb_ss_ep = to_usb_ss_ep(ep);
		bEndpointAddress = usb_ss_ep->num | usb_ss_ep->dir;
		select_ep(usb_ss, bEndpointAddress);
		gadget_writel(usb_ss, &usb_ss->regs->ep_cmd,
			EP_CMD__EPRST__MASK);
		ret = wait_reg_bit_clear(usb_ss, &usb_ss->regs->ep_cmd,
			EP_CMD__EPRST__MASK, 100);
	}

	/* disable interrupt for device */
	gadget_writel(usb_ss, &usb_ss->regs->usb_ien, 0);
	gadget_writel(usb_ss, &usb_ss->regs->usb_conf, USB_CONF__DEVDS__MASK);

	for (i = 0; i < usb_ss->ep_nums ; i++)
		usb_ss_free_trb_pool(usb_ss->eps[i]);

	return ret;
}

static const struct usb_gadget_ops usb_ss_gadget_ops = {
	.get_frame = usb_ss_gadget_get_frame,
	.wakeup = usb_ss_gadget_wakeup,
	.set_selfpowered = usb_ss_gadget_set_selfpowered,
	.pullup = usb_ss_gadget_pullup,
	.udc_start = usb_ss_gadget_udc_start,
	.udc_stop = usb_ss_gadget_udc_stop,
	.match_ep = usb_ss_gadget_match_ep,
};

/**
 * usb_ss_init_ep Initializes software endpoints of gadget
 * @usb_ss: extended gadget object
 *
 * Returns 0 on success, error code elsewhere
 */
static int usb_ss_init_ep(struct usb_ss_dev *usb_ss)
{
	struct usb_ss_endpoint *usb_ss_ep;
	u32 ep_enabled_reg, iso_ep_reg, bulk_ep_reg;
	int i;
	int ep_reg_pos, ep_dir, ep_number;
	int found_endpoints = 0;

	/* Read it from USB_CAP3 to USB_CAP5 */
	ep_enabled_reg = 0x00ff00ff;
	iso_ep_reg = 0x00fe00fe;
	bulk_ep_reg = 0x00fe00fe;

	dev_dbg(&usb_ss->dev, "Initializing non-zero endpoints\n");

	for (i = 0; i < USB_SS_ENDPOINTS_MAX_COUNT; i++) {
		ep_number = (i / 2) + 1;
		ep_dir = i % 2;
		ep_reg_pos = (16 * ep_dir) + ep_number;

		if (!(ep_enabled_reg & (1uL << ep_reg_pos)))
			continue;

		/* create empty endpoint object */
		usb_ss_ep = devm_kzalloc(&usb_ss->dev, sizeof(*usb_ss_ep),
			GFP_KERNEL);
		if (!usb_ss_ep)
			return -ENOMEM;

		/* set parent of endpoint object */
		usb_ss_ep->usb_ss = usb_ss;

		/* set index of endpoint in endpoints container */
		usb_ss->eps[found_endpoints++] = usb_ss_ep;

		/* set name of endpoint */
		snprintf(usb_ss_ep->name, sizeof(usb_ss_ep->name), "ep%d%s",
				ep_number, !!ep_dir ? "in" : "out");
		usb_ss_ep->endpoint.name = usb_ss_ep->name;
		dev_dbg(&usb_ss->dev, "Initializing endpoint: %s\n",
				usb_ss_ep->name);

		usb_ep_set_maxpacket_limit(&usb_ss_ep->endpoint,
		ENDPOINT_MAX_PACKET_LIMIT);
		usb_ss_ep->endpoint.max_streams = ENDPOINT_MAX_STREAMS;
		usb_ss_ep->endpoint.ops = &usb_ss_gadget_ep_ops;
		if (ep_dir)
			usb_ss_ep->endpoint.caps.dir_in = 1;
		else
			usb_ss_ep->endpoint.caps.dir_out = 1;

		/* check endpoint type */
		if (iso_ep_reg & (1uL << ep_reg_pos))
			usb_ss_ep->endpoint.caps.type_iso = 1;

		if (bulk_ep_reg & (1uL << ep_reg_pos)) {
			usb_ss_ep->endpoint.caps.type_bulk = 1;
			usb_ss_ep->endpoint.caps.type_int = 1;
			usb_ss_ep->endpoint.maxburst = CDNS3_EP_BUF_SIZE - 1;
		}

		list_add_tail(&usb_ss_ep->endpoint.ep_list,
				&usb_ss->gadget.ep_list);
		INIT_LIST_HEAD(&usb_ss_ep->request_list);
		INIT_LIST_HEAD(&usb_ss_ep->ep_match_pending_list);
	}

	usb_ss->ep_nums = found_endpoints;
	return 0;
}

/**
 * usb_ss_init_ep0 Initializes software endpoint 0 of gadget
 * @usb_ss: extended gadget object
 *
 * Returns 0 on success, error code elsewhere
 */
static int usb_ss_init_ep0(struct usb_ss_dev *usb_ss)
{
	struct usb_ss_endpoint *ep0;

	dev_dbg(&usb_ss->dev, "Initializing EP0\n");
	ep0 = devm_kzalloc(&usb_ss->dev, sizeof(struct usb_ss_endpoint),
		GFP_KERNEL);

	if (!ep0)
		return -ENOMEM;

	/* fill CDNS fields */
	ep0->usb_ss = usb_ss;
	sprintf(ep0->name, "ep0");

	/* fill linux fields */
	ep0->endpoint.ops = &usb_ss_gadget_ep0_ops;
	ep0->endpoint.maxburst = 1;
	usb_ep_set_maxpacket_limit(&ep0->endpoint, ENDPOINT0_MAX_PACKET_LIMIT);
	ep0->endpoint.address = 0;
	ep0->endpoint.enabled = 1;
	ep0->endpoint.caps.type_control = 1;
	ep0->endpoint.caps.dir_in = 1;
	ep0->endpoint.caps.dir_out = 1;
	ep0->endpoint.name = ep0->name;
	ep0->endpoint.desc = &cdns3_gadget_ep0_desc;
	usb_ss->gadget.ep0 = &ep0->endpoint;

	return 0;
}

static void cdns3_gadget_release(struct device *dev)
{
	struct usb_ss_dev *usb_ss = container_of(dev, struct usb_ss_dev, dev);

	dev_dbg(dev, "releasing '%s'\n", dev_name(dev));
	kfree(usb_ss);
}

static int __cdns3_gadget_init(struct cdns3 *cdns)
{
	struct usb_ss_dev *usb_ss;
	int ret;
	struct device *dev;

	usb_ss = kzalloc(sizeof(*usb_ss), GFP_KERNEL);
	if (!usb_ss)
		return -ENOMEM;

	dev = &usb_ss->dev;
	dev->release = cdns3_gadget_release;
	dev->parent = cdns->dev;
	dev_set_name(dev, "gadget-cdns3");
	cdns->gadget_dev = dev;
	usb_ss->sysdev = cdns->dev;
	ret = device_register(dev);
	if (ret)
		goto err1;

	usb_ss->regs = cdns->dev_regs;

	/* fill gadget fields */
	usb_ss->gadget.ops = &usb_ss_gadget_ops;
	usb_ss->gadget.max_speed = USB_SPEED_SUPER;
	usb_ss->gadget.speed = USB_SPEED_UNKNOWN;
	usb_ss->gadget.name = "usb-ss-gadget";
	usb_ss->gadget.sg_supported = 1;
	usb_ss->is_connected = 0;
	spin_lock_init(&usb_ss->lock);

	usb_ss->in_standby_mode = 1;

	/* initialize endpoint container */
	INIT_LIST_HEAD(&usb_ss->gadget.ep_list);
	INIT_LIST_HEAD(&usb_ss->ep_match_list);
	ret = usb_ss_init_ep0(usb_ss);
	if (ret) {
		dev_err(dev, "Failed to create endpoint 0\n");
		ret = -ENOMEM;
		goto err2;
	}

	ret = usb_ss_init_ep(usb_ss);
	if (ret) {
		dev_err(dev, "Failed to create non zero endpoints\n");
		ret = -ENOMEM;
		goto err2;
	}

	/* allocate memory for default endpoint TRB */
	usb_ss->trb_ep0 = (u32 *)dma_alloc_coherent(usb_ss->sysdev, 20,
			&usb_ss->trb_ep0_dma, GFP_DMA);
	if (!usb_ss->trb_ep0) {
		dev_err(dev, "Failed to allocate memory for ep0 TRB\n");
		ret = -ENOMEM;
		goto err2;
	}

	/* allocate memory for setup packet buffer */
	usb_ss->setup = (u8 *)dma_alloc_coherent(usb_ss->sysdev, 8,
			&usb_ss->setup_dma,
			GFP_DMA);
	if (!usb_ss->setup) {
		dev_err(dev, "Failed to allocate memory for SETUP buffer\n");
		ret = -ENOMEM;
		goto err3;
	}

	/* add USB gadget device */
	ret = usb_add_gadget_udc(&usb_ss->dev, &usb_ss->gadget);
	if (ret < 0) {
		dev_err(dev, "Failed to register USB device controller\n");
		goto err4;
	}

	usb_ss->zlp_buf = kzalloc(ENDPOINT_ZLP_BUF_SIZE, GFP_KERNEL);
	if (!usb_ss->zlp_buf) {
		ret = -ENOMEM;
		goto err4;
	}

	return 0;
err4:
	dma_free_coherent(usb_ss->sysdev, 8, usb_ss->setup,
			usb_ss->setup_dma);
err3:
	dma_free_coherent(usb_ss->sysdev, 20, usb_ss->trb_ep0,
			usb_ss->trb_ep0_dma);
err2:
	device_del(dev);
err1:
	put_device(dev);
	cdns->gadget_dev = NULL;
	return ret;
}

/**
 * cdns3_gadget_remove: parent must call this to remove UDC
 *
 * cdns: cdns3 instance
 *
 */
void cdns3_gadget_remove(struct cdns3 *cdns)
{
	struct usb_ss_dev *usb_ss;

	if (!cdns->roles[CDNS3_ROLE_GADGET])
		return;

	usb_ss = container_of(cdns->gadget_dev, struct usb_ss_dev, dev);
	usb_del_gadget_udc(&usb_ss->gadget);
	dma_free_coherent(usb_ss->sysdev, 8, usb_ss->setup, usb_ss->setup_dma);
	dma_free_coherent(usb_ss->sysdev, 20, usb_ss->trb_ep0,
			usb_ss->trb_ep0_dma);
	device_unregister(cdns->gadget_dev);
	cdns->gadget_dev = NULL;
	kfree(usb_ss->zlp_buf);
}

static void __cdns3_gadget_start(struct usb_ss_dev *usb_ss)
{

	/* configure endpoint 0 hardware */
	cdns_ep0_config(usb_ss);

	/* enable interrupts for endpoint 0 (in and out) */
	gadget_writel(usb_ss, &usb_ss->regs->ep_ien,
		EP_IEN__EOUTEN0__MASK | EP_IEN__EINEN0__MASK);

	/* enable interrupt for device */
	gadget_writel(usb_ss, &usb_ss->regs->usb_ien,
			USB_IEN__U2RESIEN__MASK
			| USB_ISTS__DIS2I__MASK
			| USB_IEN__CON2IEN__MASK
			| USB_IEN__UHRESIEN__MASK
			| USB_IEN__UWRESIEN__MASK
			| USB_IEN__DISIEN__MASK
			| USB_IEN__CONIEN__MASK
			| USB_IEN__U3EXTIEN__MASK
			| USB_IEN__L2ENTIEN__MASK
			| USB_IEN__L2EXTIEN__MASK);

	gadget_writel(usb_ss, &usb_ss->regs->usb_conf,
		    USB_CONF__CLK2OFFDS__MASK
	/*	    | USB_CONF__USB3DIS__MASK */
		    | USB_CONF__L1DS__MASK);

	gadget_writel(usb_ss, &usb_ss->regs->usb_conf,
			USB_CONF__U1DS__MASK
			| USB_CONF__U2DS__MASK
			);

	gadget_writel(usb_ss, &usb_ss->regs->usb_conf, USB_CONF__DEVEN__MASK);

	gadget_writel(usb_ss, &usb_ss->regs->dbg_link1,
		DBG_LINK1__LFPS_MIN_GEN_U1_EXIT_SET__MASK |
		DBG_LINK1__LFPS_MIN_GEN_U1_EXIT__WRITE(0x3C));
}

static int cdns3_gadget_start(struct cdns3 *cdns)
{
	struct usb_ss_dev *usb_ss = container_of(cdns->gadget_dev,
			struct usb_ss_dev, dev);
	unsigned long flags;

	dev_dbg(&usb_ss->dev, "%s begins\n", __func__);

	pm_runtime_get_sync(cdns->dev);
	spin_lock_irqsave(&usb_ss->lock, flags);
	usb_ss->start_gadget = 1;
	if (!usb_ss->gadget_driver) {
		spin_unlock_irqrestore(&usb_ss->lock, flags);
		return 0;
	}

	__cdns3_gadget_start(usb_ss);
	usb_ss->in_standby_mode = 0;
	spin_unlock_irqrestore(&usb_ss->lock, flags);
	dev_dbg(&usb_ss->dev, "%s ends\n", __func__);
	return 0;
}

static void __cdns3_gadget_stop(struct cdns3 *cdns)
{
	struct usb_ss_dev *usb_ss;
	unsigned long flags;

	usb_ss = container_of(cdns->gadget_dev, struct usb_ss_dev, dev);
	if (usb_ss->gadget_driver)
		usb_ss->gadget_driver->disconnect(&usb_ss->gadget);
	usb_gadget_disconnect(&usb_ss->gadget);
	spin_lock_irqsave(&usb_ss->lock, flags);
	/* disable interrupt for device */
	gadget_writel(usb_ss, &usb_ss->regs->usb_ien, 0);
	gadget_writel(usb_ss, &usb_ss->regs->usb_conf, USB_CONF__DEVDS__MASK);
	usb_ss->start_gadget = 0;
	spin_unlock_irqrestore(&usb_ss->lock, flags);
}

static void cdns3_gadget_stop(struct cdns3 *cdns)
{
	if (cdns->role == CDNS3_ROLE_GADGET)
		__cdns3_gadget_stop(cdns);
	pm_runtime_mark_last_busy(cdns->dev);
	pm_runtime_put_autosuspend(cdns->dev);
}

static int cdns3_gadget_suspend(struct cdns3 *cdns, bool do_wakeup)
{
	__cdns3_gadget_stop(cdns);
	return 0;
}

static int cdns3_gadget_resume(struct cdns3 *cdns, bool hibernated)
{
	struct usb_ss_dev *usb_ss = container_of(cdns->gadget_dev,
			struct usb_ss_dev, dev);
	unsigned long flags;

	spin_lock_irqsave(&usb_ss->lock, flags);
	usb_ss->start_gadget = 1;
	if (!usb_ss->gadget_driver) {
		spin_unlock_irqrestore(&usb_ss->lock, flags);
		return 0;
	}

	__cdns3_gadget_start(usb_ss);
	usb_ss->in_standby_mode = 0;
	spin_unlock_irqrestore(&usb_ss->lock, flags);
	return 0;
}

/**
 * cdns3_gadget_init - initialize device structure
 *
 * cdns: cdns3 instance
 *
 * This function initializes the gadget.
 */
int cdns3_gadget_init(struct cdns3 *cdns)
{
	struct cdns3_role_driver *rdrv;

	rdrv = devm_kzalloc(cdns->dev, sizeof(*rdrv), GFP_KERNEL);
	if (!rdrv)
		return -ENOMEM;

	rdrv->start	= cdns3_gadget_start;
	rdrv->stop	= cdns3_gadget_stop;
	rdrv->suspend	= cdns3_gadget_suspend;
	rdrv->resume	= cdns3_gadget_resume;
	rdrv->irq	= cdns_irq_handler_thread;
	rdrv->name	= "gadget";
	cdns->roles[CDNS3_ROLE_GADGET] = rdrv;
	return __cdns3_gadget_init(cdns);
}
