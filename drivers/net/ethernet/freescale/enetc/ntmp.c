// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NETC NTMP (NETC Table Management Protocol) 2.0 driver
 * Copyright 2023 NXP
 * Copyright (c) 2023 Wei Fang <wei.fang@nxp.com>
 */
#include <linux/iopoll.h>

#include "ntmp_formats.h"

/* Define NTMP Table ID */
#define NTMP_MAFT_ID			1
#define NTMP_VAFT_ID			2
#define NTMP_RSST_ID			3
#define NTMP_TGST_ID			5
#define NTMP_RPT_ID			10
#define NTMP_IPFT_ID			13
#define NTMP_ISIT_ID			30
#define NTMP_IST_ID			31
#define NTMP_ISFT_ID			32
#define NTMP_SGIT_ID			36
#define NTMP_SGCLT_ID			37
#define NTMP_ISCT_ID			38

/* Define NTMP Access Method */
#define NTMP_AM_ENTRY_ID		0
#define NTMP_AM_EXACT_KEY		1
#define NTMP_AM_SEARCH			2
#define NTMP_AM_TERNARY_KEY		3

/* Define NTMP Header Version */
#define NTMP_HEADER_VERSION2		2

/* Define NTMP Protocol Format */
#define NTMP_REQ_HDR_NPF		BIT(15)

#define NTMP_RESP_LEN_MASK		GENMASK(19, 0)
#define NTMP_REQ_LEN_MASK		GENMASK(31, 20)

#define ENETC_STREAM_GATE_STATE_CLOSE	0
#define ENETC_STREAM_GATE_STATE_OPEN	1

#define ENETC_NTMP_ENTRY_ID_SIZE	4

#define ENETC_RSS_TABLE_ENTRY_NUM	64
#define ENETC_RSS_CFGEU			BIT(0)
#define ENETC_RSS_STSEU			BIT(1)
#define ENETC_RSS_STSE_DATA_SIZE(n)	((n) * 8)
#define ENETC_RSS_CFGE_DATA_SIZE(n)	(n)

#define NTMP_REQ_RESP_LEN(req, resp)	(((req) << 20 & NTMP_REQ_LEN_MASK) | \
					 ((resp) & NTMP_RESP_LEN_MASK))

static inline u32 netc_cbdr_read(void __iomem *reg)
{
	return ioread32(reg);
}

static inline void netc_cbdr_write(void __iomem *reg, u32 val)
{
	iowrite32(val, reg);
}

int netc_setup_cbdr(struct device *dev, int cbd_num,
		    struct netc_cbdr_regs *regs,
		    struct netc_cbdr *cbdr)
{
	int size;

	size = cbd_num * sizeof(union netc_cbd) +
	       NETC_CBDR_BASE_ADDR_ALIGN;

	cbdr->addr_base = dma_alloc_coherent(dev, size, &cbdr->dma_base,
					     GFP_KERNEL);
	if (!cbdr->addr_base)
		return -ENOMEM;

	cbdr->dma_size = size;
	cbdr->bd_num = cbd_num;
	cbdr->regs = *regs;
	cbdr->dma_dev = dev;

	/* The base address of the Control BD Ring must be 128 bytes aligned */
	cbdr->dma_base_align =  ALIGN(cbdr->dma_base,
				      NETC_CBDR_BASE_ADDR_ALIGN);
	cbdr->addr_base_align = PTR_ALIGN(cbdr->addr_base,
					  NETC_CBDR_BASE_ADDR_ALIGN);

	cbdr->next_to_clean = 0;
	cbdr->next_to_use = 0;
	spin_lock_init(&cbdr->ring_lock);

	/* Step 1: Configure the base address of the Control BD Ring */
	netc_cbdr_write(cbdr->regs.bar0, lower_32_bits(cbdr->dma_base_align));
	netc_cbdr_write(cbdr->regs.bar1, upper_32_bits(cbdr->dma_base_align));

	/* Step 2: Configure the producer index register */
	netc_cbdr_write(cbdr->regs.pir, cbdr->next_to_clean);

	/* Step 3: Configure the consumer index register */
	netc_cbdr_write(cbdr->regs.cir, cbdr->next_to_use);

	/* Step4: Configure the number of BDs of the Control BD Ring */
	netc_cbdr_write(cbdr->regs.lenr, cbdr->bd_num);

	/* Step 5: Enable the Control BD Ring */
	netc_cbdr_write(cbdr->regs.mr, NETC_CBDRMR_EN);

	return 0;
}
EXPORT_SYMBOL_GPL(netc_setup_cbdr);

void netc_free_cbdr(struct netc_cbdr *cbdr)
{
	/* Disable the Control BD Ring */
	netc_cbdr_write(cbdr->regs.mr, 0);

	dma_free_coherent(cbdr->dma_dev, cbdr->dma_size,
			  cbdr->addr_base, cbdr->dma_base);

	memset(cbdr, 0, sizeof(*cbdr));
}
EXPORT_SYMBOL_GPL(netc_free_cbdr);

static inline int netc_get_free_cbd_num(struct netc_cbdr *cbdr)
{
	return (cbdr->next_to_clean - cbdr->next_to_use - 1 + cbdr->bd_num) %
		cbdr->bd_num;
}

static inline union netc_cbd *netc_get_cbd(struct netc_cbdr *cbdr, int index)
{
	return &((union netc_cbd *)(cbdr->addr_base_align))[index];
}

static void netc_clean_cbdr(struct netc_cbdr *cbdr)
{
	union netc_cbd *cbd;
	int i;

	i = cbdr->next_to_clean;
	while (netc_cbdr_read(cbdr->regs.cir) != i) {
		cbd = netc_get_cbd(cbdr, i);
		memset(cbd, 0, sizeof(*cbd));
		i = (i + 1) % cbdr->bd_num;
	}

	cbdr->next_to_clean = i;
}

static int netc_xmit_ntmp_cmd(struct netc_cbdr *cbdr, union netc_cbd *cbd)
{
	union netc_cbd *ring_cbd;
	unsigned long flags;
	int i, err;
	u16 status;
	u32 val;

	if (unlikely(!cbdr->addr_base))
		return -EFAULT;

	spin_lock_irqsave(&cbdr->ring_lock, flags);

	if (unlikely(!netc_get_free_cbd_num(cbdr)))
		netc_clean_cbdr(cbdr);

	i = cbdr->next_to_use;
	ring_cbd = netc_get_cbd(cbdr, i);

	/* Copy command BD to the ring */
	*ring_cbd = *cbd;
	/* Update producer index of both software and hardware */
	i = (i + 1) % cbdr->bd_num;
	cbdr->next_to_use = i;
	netc_cbdr_write(cbdr->regs.pir, i);

	err = read_poll_timeout_atomic(netc_cbdr_read, val, val == i,
				       10, NETC_CBDR_TIMEOUT, true,
				       cbdr->regs.cir);
	if (unlikely(err)) {
		err = -EBUSY;
		goto err_unlock;
	}

	/* Check the writeback error status */
	status = le16_to_cpu(ring_cbd->ntmp_resp_hdr.error_rr) & NTMP_RESP_HDR_ERR;
	if (unlikely(status)) {
		dev_err(cbdr->dma_dev, "Command BD error: 0x%04x\n", status);
		err = -EIO;
	}

	netc_clean_cbdr(cbdr);

err_unlock:
	spin_unlock_irqrestore(&cbdr->ring_lock, flags);

	return err;
}

static void *ntmp_alloc_data_mem(struct netc_cbdr *cbdr,
				 int size, dma_addr_t *dma,
				 void **data_align)
{
	void *data;

	data = dma_alloc_coherent(cbdr->dma_dev,
				  size + NETC_CBD_DATA_ADDR_ALIGN,
				  dma, GFP_ATOMIC);
	if (!data) {
		dev_err(cbdr->dma_dev, "NTMP alloc data memory failed!\n");
		return NULL;
	}
	*data_align = PTR_ALIGN(data, NETC_CBD_DATA_ADDR_ALIGN);

	return data;
}

static void ntmp_free_data_mem(struct netc_cbdr *cbdr, int size,
			       void *data, dma_addr_t dma)
{
	dma_free_coherent(cbdr->dma_dev, size + NETC_CBD_DATA_ADDR_ALIGN,
			  data, dma);
}

static inline void ntmp_fill_request_headr(union netc_cbd *cbd, dma_addr_t dma,
					   int len, int table_id, int cmd,
					   int access_method)
{
	dma_addr_t dma_align;

	memset(cbd, 0, sizeof(*cbd));
	dma_align = ALIGN(dma, NETC_CBD_DATA_ADDR_ALIGN);
	cbd->ntmp_req_hdr.addr = cpu_to_le64(dma_align);
	cbd->ntmp_req_hdr.len = cpu_to_le32(len);
	cbd->ntmp_req_hdr.cmd = cmd;
	cbd->ntmp_req_hdr.access_method = access_method;
	cbd->ntmp_req_hdr.table_id = table_id;
	cbd->ntmp_req_hdr.hdr_ver = NTMP_HEADER_VERSION2;
	cbd->ntmp_req_hdr.cci = 0;
	cbd->ntmp_req_hdr.rr = 0;	//Must be set to 0 by SW.
	/* For NTMP version 2.0 or later version */
	cbd->ntmp_req_hdr.npf = cpu_to_le32(NTMP_REQ_HDR_NPF);
}

int ntmp_maft_add_entry(struct netc_cbdr *cbdr, u32 entry_id,
			const char *mac_addr, int si_bitmap)
{
	struct maft_req_add *req;
	union netc_cbd cbd;
	u32 len, data_size;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Set mac address filter table request data buffer */
	req->entry_id = cpu_to_le32(entry_id);
	ether_addr_copy(req->keye.mac_addr, mac_addr);
	req->cfge.si_bitmap = cpu_to_le16(si_bitmap);

	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_MAFT_ID,
				NTMP_CMD_ADD, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev, "Add MAC filter table entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_maft_add_entry);

int ntmp_maft_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			  struct ntmp_mfe *entry)
{
	struct maft_resp_query *resp;
	struct maft_req_qd *req;
	union netc_cbd cbd;
	u32 len, data_size;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!entry || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data_size = sizeof(*resp);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Set mac address filter table request data buffer */
	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(sizeof(*req), data_size);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_MAFT_ID,
				NTMP_CMD_QUERY, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev, "Query MAC filter table entry failed (%d)!", err);
		goto end;
	}

	resp = (struct maft_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(cbdr->dma_dev,
			"Entry ID doesn't match, query ID:0x%0x, response ID:0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	ether_addr_copy(entry->mac, resp->keye.mac_addr);
	entry->si_bitmap = le16_to_cpu(resp->cfge.si_bitmap);

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_maft_query_entry);

int ntmp_maft_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	struct maft_req_qd *req;
	union netc_cbd cbd;
	u32 len, data_size;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Set mac address filter table request data buffer */
	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_MAFT_ID,
				NTMP_CMD_DELETE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Delete MAC filter table entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_maft_delete_entry);

int ntmp_vaft_add_entry(struct netc_cbdr *cbdr, u32 entry_id,
			struct ntmp_vfe *vfe)
{
	struct vaft_req_add *req;
	union netc_cbd cbd;
	u32 len, data_size;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Set VLAN address filter table request data buffer */
	req->entry_id = cpu_to_le32(entry_id);
	req->keye.vlan_id = cpu_to_le16(vfe->vid);
	req->keye.tpid = vfe->tpid;
	req->cfge.si_bitmap = cpu_to_le16(vfe->si_bitmap);

	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_VAFT_ID,
				NTMP_CMD_ADD, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev, "Add VLAN filter table entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_vaft_add_entry);

int ntmp_vaft_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			  struct ntmp_vfe *entry)
{
	struct vaft_resp_query *resp;
	struct vaft_req_qd *req;
	union netc_cbd cbd;
	u32 len, data_size;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!entry || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data_size = sizeof(*resp);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Set VLAN address filter table request data buffer */
	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(sizeof(*req), data_size);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_VAFT_ID,
				NTMP_CMD_QUERY, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev, "Query VLAN filter table entry failed (%d)!", err);
		goto end;
	}

	resp = (struct vaft_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(cbdr->dma_dev,
			"Entry ID doesn't match, query ID:0x%0x, response ID:0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	entry->vid = le16_to_cpu(resp->keye.vlan_id);
	entry->tpid = resp->keye.tpid;
	entry->si_bitmap = le16_to_cpu(resp->cfge.si_bitmap);

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_vaft_query_entry);

int ntmp_vaft_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	struct vaft_req_qd *req;
	union netc_cbd cbd;
	u32 len, data_size;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Set VLAN address filter table request data buffer */
	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_VAFT_ID,
				NTMP_CMD_DELETE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Delete VLAN filter table entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_vaft_delete_entry);

int ntmp_rsst_query_or_update_entry(struct netc_cbdr *cbdr, u32 *table,
				    int count, bool query)
{
	struct rsst_req_update *requ;
	struct rsst_req_query *req;
	union netc_cbd cbd;
	u32 len, data_size;
	dma_addr_t dma;
	int err, i;
	void *tmp;

	if (count != ENETC_RSS_TABLE_ENTRY_NUM)
		/* HW only takes in a full 64 entry table */
		return -EINVAL;

	if (query)
		data_size = ENETC_NTMP_ENTRY_ID_SIZE + ENETC_RSS_STSE_DATA_SIZE(count) +
			    ENETC_RSS_CFGE_DATA_SIZE(count);
	else
		data_size = struct_size(requ, groups, count);

	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Set the request data buffer */
	if (query) {
		len = NTMP_REQ_RESP_LEN(sizeof(*req), data_size);
		ntmp_fill_request_headr(&cbd, dma, len, NTMP_RSST_ID,
					NTMP_CMD_QUERY, NTMP_AM_ENTRY_ID);
	} else {
		requ = (struct rsst_req_update *)req;
		requ->crd.update_act = cpu_to_le16(ENETC_RSS_CFGEU | ENETC_RSS_STSEU);
		for (i = 0; i < count; i++)
			requ->groups[i] = (u8)(table[i]);

		len = NTMP_REQ_RESP_LEN(data_size, 0);
		ntmp_fill_request_headr(&cbd, dma, len, NTMP_RSST_ID,
					NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);
	}

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev, "%s RSS table entry failed (%d)!",
			query ? "Query" : "Update", err);
		goto end;
	}

	if (query) {
		u8 *group = (u8 *)req;

		group += ENETC_NTMP_ENTRY_ID_SIZE + ENETC_RSS_STSE_DATA_SIZE(count);
		for (i = 0; i < count; i++)
			table[i] = group[i];
	}

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_rsst_query_or_update_entry);

/* Test codes for Time gate scheduling table */
int ntmp_tgst_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			  struct ntmp_tgst_info *info)
{
	struct tgst_resp_query *resp;
	struct tgst_cfge_data *cfge;
	struct tgst_olse_data *olse;
	struct tgst_req_query *req;
	union netc_cbd cbd;
	u32 len, data_size;
	dma_addr_t dma;
	u16 list_len;
	int i, err;
	void *tmp;

	if (!cbdr || !info)
		return -EINVAL;

	data_size = sizeof(*resp) + struct_size(cfge, ge, NTMP_TGST_MAX_ENTRY_NUM) +
		    struct_size(olse, ge, NTMP_TGST_MAX_ENTRY_NUM);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(sizeof(*req), data_size);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_TGST_ID,
				NTMP_CMD_QUERY, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev,
			"Query time gate scheduling table entry failed (%d)!", err);
		goto end;
	}

	resp = (struct tgst_resp_query *)req;
	cfge = (struct tgst_cfge_data *)resp->data;
	list_len = le16_to_cpu(cfge->admin_cl_len);

	info->status = le64_to_cpu(resp->status.cfg_ct);
	info->entry_id = le32_to_cpu(resp->entry_id);
	info->admin_bt = le64_to_cpu(cfge->admin_bt);
	info->admin_ct = le32_to_cpu(cfge->admin_ct);
	info->admin_ct_ext = le32_to_cpu(cfge->admin_ct_ext);
	info->admin_cl_len = list_len;
	for (i = 0; i < list_len; i++) {
		info->admin[i].interval = le32_to_cpu(cfge->ge[i].interval);
		info->admin[i].tc_gates = cfge->ge[i].tc_state;
		info->admin[i].oper_type = le16_to_cpu(cfge->ge[i].hr_cb) &
					   NTMP_TGST_HR_CB_GE;
	}

	olse = (struct tgst_olse_data *)&cfge->ge[i];
	list_len = le16_to_cpu(olse->oper_cl_len);

	info->cfg_ct = le64_to_cpu(olse->cfg_ct);
	info->cfg_ce = le64_to_cpu(olse->cfg_ce);
	info->oper_bt = le64_to_cpu(olse->oper_bt);
	info->oper_ct = le32_to_cpu(olse->oper_ct);
	info->oper_ct_ext = le32_to_cpu(olse->oper_ct_ext);
	info->oper_cl_len = list_len;
	for (i = 0; i < list_len; i++) {
		info->oper[i].interval = le32_to_cpu(olse->ge[i].interval);
		info->oper[i].tc_gates = olse->ge[i].tc_state;
		info->oper[i].oper_type = le16_to_cpu(olse->ge[i].hr_cb) &
					  NTMP_TGST_HR_CB_GE;
	}

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_tgst_query_entry);

int ntmp_tgst_delete_admin_gate_list(struct netc_cbdr *cbdr, u32 entry_id)
{
	struct tgst_req_update *req;
	struct tgst_cfge_data *cfge;
	union netc_cbd cbd;
	u32 len, data_size;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	cfge = &req->cfge;

	/* Set the request data buffer and set the admin control list len
	 * to zero to delete the existing admin control list.
	 */
	req->crd.update_act = cpu_to_le16(1);
	req->crd.tbl_ver = 0;
	req->entry_id = cpu_to_le32(entry_id);
	cfge->admin_cl_len = 0;

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, sizeof(struct tgst_resp_status));
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_TGST_ID,
				NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Delete time gate scheduling table entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_tgst_delete_admin_gate_list);

static u64 ntmp_adjust_base_time(struct netc_cbdr *cbdr, u64 base_time, u32 cycle_time)
{
	u64 current_time, delta, n;
	u32 time_high, time_low;

	time_low = netc_cbdr_read(cbdr->regs.sictr0);
	time_high = netc_cbdr_read(cbdr->regs.sictr1);
	current_time = (u64)time_high << 32 | time_low;
	if (base_time >= current_time)
		return base_time;

	delta = current_time - base_time;
	n = DIV_ROUND_UP_ULL(delta, cycle_time);

	return base_time + (n * (u64)cycle_time);
}

int ntmp_tgst_update_admin_gate_list(struct netc_cbdr *cbdr, u32 entry_id,
				     struct ntmp_tgst_cfg *cfg)
{
	struct tgst_req_update *req;
	struct tgst_cfge_data *cfge;
	union netc_cbd cbd;
	struct tgst_ge *ge;
	u32 len, data_size;
	dma_addr_t dma;
	u64 base_time;
	int i, err;
	void *tmp;

	/* Calculate the size of request data buffer */
	data_size = struct_size(req, cfge.ge, cfg->num_entries);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	cfge = &req->cfge;
	ge = cfge->ge;

	/* Set the request data buffer */
	req->crd.update_act = cpu_to_le16(1);
	req->crd.tbl_ver = 0;
	req->entry_id = cpu_to_le32(entry_id);
	base_time = ntmp_adjust_base_time(cbdr, cfg->base_time, cfg->cycle_time);
	cfge->admin_bt = cpu_to_le64(base_time);
	cfge->admin_ct = cpu_to_le32(cfg->cycle_time);
	cfge->admin_ct_ext = cpu_to_le32(cfg->cycle_time_extension);
	cfge->admin_cl_len = cpu_to_le16(cfg->num_entries);

	for (i = 0; i < cfg->num_entries; i++) {
		struct ntmp_tgst_ge *temp_entry = &cfg->entries[i];
		struct tgst_ge *temp_ge = ge + i;

		temp_ge->tc_state = (u8)temp_entry->tc_gates;
		temp_ge->interval = cpu_to_le32(temp_entry->interval);
		temp_ge->hr_cb = cpu_to_le16(temp_entry->oper_type);
	}

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, sizeof(struct tgst_resp_status));
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_TGST_ID,
				NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Update time gate scheduling table entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_tgst_update_admin_gate_list);

int ntmp_rpt_add_or_update_entry(struct netc_cbdr *cbdr, struct ntmp_rpt_cfg *cfg)
{
	struct rpt_req_ua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.update_act = cpu_to_le16(0xf);
	req->entry_id = cpu_to_le32(cfg->entry_id);
	req->cfge.cbs = cpu_to_le32(cfg->cbs);
	req->cfge.cir = cpu_to_le32(cfg->cir);
	req->cfge.ebs = cpu_to_le32(cfg->ebs);
	req->cfge.eir = cpu_to_le32(cfg->eir);
	req->fee.fen = 1;	//always enable the function

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_RPT_ID,
				NTMP_CMD_AU, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Add/Update rate policer table entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_rpt_add_or_update_entry);

int ntmp_rpt_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			 struct ntmp_rpt_info *info)
{
	struct rpt_resp_query *resp;
	struct rpt_req_nua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!info || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data_size = sizeof(*resp);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Request data */
	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(sizeof(*req), data_size);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_RPT_ID,
				NTMP_CMD_QUERY, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev,
			"Query rate policer table entry failed (%d)!", err);
		goto end;
	}

	resp = (struct rpt_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(cbdr->dma_dev,
			"Entry ID doesn't match, query ID:0x%0x, response ID:0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	info->sts.byte_cnt = le64_to_cpu(resp->stse.byte_count);
	info->sts.drop_frames = le32_to_cpu(resp->stse.drop_frames);
	info->sts.dr0_grn_frames = le32_to_cpu(resp->stse.dr0_grn_frames);
	info->sts.dr1_grn_frames = le32_to_cpu(resp->stse.dr1_grn_frames);
	info->sts.dr2_ylw_frames = le32_to_cpu(resp->stse.dr2_ylw_frames);
	info->sts.dr3_red_frames = le32_to_cpu(resp->stse.dr3_red_frames);
	info->sts.remark_ylw_frames = le32_to_cpu(resp->stse.remark_ylw_frames);
	info->sts.remark_red_frames = le32_to_cpu(resp->stse.remark_red_frames);
	info->cfg.cir = le32_to_cpu(resp->cfge.cir);
	info->cfg.cbs = le32_to_cpu(resp->cfge.cbs);
	info->cfg.eir = le32_to_cpu(resp->cfge.eir);
	info->cfg.ebs = le32_to_cpu(resp->cfge.ebs);
	info->cfg.mren = resp->cfge.mren;
	info->cfg.doy = resp->cfge.doy;
	info->cfg.cm = resp->cfge.cm;
	info->cfg.cf = resp->cfge.cf;
	info->cfg.ndor = resp->cfge.ndor;
	info->cfg.sdu_type = resp->cfge.sdu_type;
	info->fen = !!resp->fee.fen;
	info->mr = resp->pse.mr;

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_rpt_query_entry);

int ntmp_rpt_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	struct rpt_req_nua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_RPT_ID,
				NTMP_CMD_DELETE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Delete rate policer table entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_rpt_delete_entry);

int ntmp_isit_add_or_update_entry(struct netc_cbdr *cbdr, struct ntmp_isit_cfg *cfg,
				  bool add)
{
	struct isit_resp_query *resp;
	struct isit_req_ua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	u16 v_pbit_vid = 0;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = add ? sizeof(*resp) : sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	if (!add)
		req->crd.update_act = cpu_to_le16(1);
	else
		/* Query ENTRY_ID only */
		req->crd.query_act = 1;
	req->ak.key_type = cpu_to_le32(cfg->key_type);
	ether_addr_copy(req->ak.fk.mac, cfg->mac);

	if (cfg->tagged)
		v_pbit_vid = cfg->vid | BIT(15);

	req->ak.fk.vlan_h = (v_pbit_vid >> 8) & 0xff;
	req->ak.fk.vlan_l = v_pbit_vid & 0xff;
	req->is_eid = cpu_to_le32(cfg->is_eid);

	/* Request header */
	if (add) {
		len = NTMP_REQ_RESP_LEN(sizeof(*req), sizeof(*resp));
		/* Must be EXACT MATCH and the command must be
		 * add, followed by a query. So that we can get
		 * the entry id from HW.
		 */
		ntmp_fill_request_headr(&cbd, dma, len, NTMP_ISIT_ID,
					NTMP_CMD_AQ, NTMP_AM_EXACT_KEY);
	} else {
		len = NTMP_REQ_RESP_LEN(sizeof(*req), sizeof(struct isit_resp_nq));
		ntmp_fill_request_headr(&cbd, dma, len, NTMP_ISIT_ID,
					NTMP_CMD_UPDATE, NTMP_AM_EXACT_KEY);
	}

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev,
			"%s ingress stream identification table entry failed (%d)!",
			add ? "Add" : "Update", err);
		goto end;
	}

	if (add) {
		resp = (struct isit_resp_query *)req;
		cfg->entry_id = le32_to_cpu(resp->entry_id);
	}

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isit_add_or_update_entry);

int ntmp_isit_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			  struct ntmp_isit_info *info)
{
	struct isit_resp_query *resp;
	struct isit_req_nua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!info || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data_size = sizeof(*resp);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->ak.entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(sizeof(*req), data_size);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_ISIT_ID,
				NTMP_CMD_QUERY, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev,
			"Query ingress stream identification table entry failed (%d)!",
			err);
		goto end;
	}

	resp = (struct isit_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(cbdr->dma_dev,
			"Entry ID doesn't match, query ID:0x%0x, response ID:0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	info->key_type = le32_to_cpu(resp->key.key_type) & NTMP_ISIT_KEY_TYPE;
	memcpy(info->key, resp->key.frame_key, NTMP_ISIT_FRAME_KEY_LEN);
	info->is_eid = le32_to_cpu(resp->is_eid);

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isit_query_entry);

int ntmp_isit_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	struct isit_req_nua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->ak.entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, sizeof(struct isit_resp_nq));
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_ISIT_ID,
				NTMP_CMD_DELETE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Delete ingress stream identification table entry failed (%d)!",
			err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isit_delete_entry);

int ntmp_ist_add_or_update_entry(struct netc_cbdr *cbdr, struct ntmp_ist_cfg *cfg)
{
	struct ist_req_ua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Fill up NTMP request data buffer */
	req->crd.update_act = cpu_to_le16(1);
	req->entry_id = cpu_to_le32(cfg->entry_id);
	req->cfge.sfe = cfg->sfe;
	req->cfge.fa = cfg->fa;
	req->cfge.msdu = cpu_to_le16(cfg->msdu);
	req->cfge.rp_eid = cpu_to_le32(cfg->rp_eid);
	req->cfge.sgi_eid = cpu_to_le32(cfg->sgi_eid);
	req->cfge.isc_eid = cpu_to_le32(cfg->isc_eid);
	req->cfge.si_bitmap = cpu_to_le16(cfg->si_bitmap);
	req->cfge.orp = cfg->orp;
	req->cfge.osgi = cfg->osgi;

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_IST_ID,
				NTMP_CMD_AU, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Add/Update ingress stream table entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ist_add_or_update_entry);

int ntmp_ist_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			 struct ntmp_ist_info *info)
{
	struct ist_resp_query *resp;
	struct ist_req_nua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!info || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Fill up NTMP request data buffer */
	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, sizeof(*resp));
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_IST_ID,
				NTMP_CMD_QUERY, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev,
			"Query ingress stream table entry failed (%d)!", err);
		goto end;
	}

	resp = (struct ist_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(cbdr->dma_dev,
			"Entry ID doesn't match, query ID:0x%0x, response ID:0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	info->sfe = resp->cfge.sfe;
	info->ipv = resp->cfge.ipv;
	info->oipv = resp->cfge.oipv;
	info->dr = resp->cfge.dr;
	info->odr = resp->cfge.odr;
	info->orp = resp->cfge.orp;
	info->osgi = resp->cfge.osgi;
	info->fa = resp->cfge.fa;
	info->sdu_type = resp->cfge.sdu_type;
	info->msdu = le16_to_cpu(resp->cfge.msdu);
	info->si_bitmap = le16_to_cpu(resp->cfge.si_bitmap);
	info->rp_eid = le32_to_cpu(resp->cfge.rp_eid);
	info->sgi_eid = le32_to_cpu(resp->cfge.sgi_eid);
	info->isc_eid = le32_to_cpu(resp->cfge.isc_eid);

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ist_query_entry);

int ntmp_ist_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	struct ist_req_nua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Fill up NTMP request data buffer */
	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_IST_ID,
				NTMP_CMD_DELETE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Delete ingress stream table entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ist_delete_entry);

int ntmp_isft_add_or_update_entry(struct netc_cbdr *cbdr, struct ntmp_isft_cfg *cfg,
				  bool add)
{
	struct isft_resp_query *resp;
	struct isft_req_ua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = add ? sizeof(*resp) : sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.update_act = cpu_to_le16(1);
	if (add)
		req->crd.query_act = 1;
	req->ak.is_eid = cpu_to_le32(cfg->is_eid);
	req->ak.pcp = cfg->priority;

	/* Override flags need to be set */
	if (cfg->or_flags & NTMP_ISFT_FLAG_ORP) {
		req->cfge.orp = 1;
		req->cfge.rp_eid = cpu_to_le32(cfg->rp_eid);
	}

	if (cfg->or_flags & NTMP_ISFT_FLAG_OSGI) {
		req->cfge.osgi = 1;
		req->cfge.sgi_eid = cpu_to_le32(cfg->sgi_eid);
	}

	req->cfge.isc_eid = cpu_to_le32(cfg->isc_eid);
	req->cfge.msdu = cpu_to_le16(cfg->msdu);

	/* Request header */
	if (add) {
		len = NTMP_REQ_RESP_LEN(sizeof(*req), sizeof(*resp));
		/* Must be exact match, and command must be add,
		 * followed by a query. So that we can get entry
		 * ID from hardware.
		 */
		ntmp_fill_request_headr(&cbd, dma, len, NTMP_ISFT_ID,
					NTMP_CMD_AQ, NTMP_AM_EXACT_KEY);
	} else {
		len = NTMP_REQ_RESP_LEN(sizeof(*req), sizeof(struct isft_resp_nq));
		ntmp_fill_request_headr(&cbd, dma, len, NTMP_ISFT_ID,
					NTMP_CMD_UPDATE, NTMP_AM_EXACT_KEY);
	}

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev,
			"%s ingress stream filter table entry failed (%d)!",
			add ? "Add" : "Update", err);
		goto end;
	}

	if (add) {
		resp = (struct isft_resp_query *)req;
		cfg->entry_id = le32_to_cpu(resp->entry_id);
	}

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isft_add_or_update_entry);

int ntmp_isft_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			  struct ntmp_isft_info *info)
{
	struct isft_resp_query *resp;
	struct isft_req_nua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!info || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data_size = sizeof(*resp);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->ak.entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(sizeof(*req), data_size);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_ISFT_ID,
				NTMP_CMD_QUERY, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev,
			"Query ingress stream filter table entry failed (%d)!", err);
		goto end;
	}

	resp = (struct isft_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(cbdr->dma_dev,
			"Entry ID doesn't match, query ID:0x%0x, response ID:0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	info->is_eid = le32_to_cpu(resp->keye.is_eid);
	info->pcp = resp->keye.pcp;
	info->ipv = resp->cfge.ipv;
	info->oipv = resp->cfge.oipv;
	info->dr = resp->cfge.dr;
	info->odr = resp->cfge.odr;
	info->osgi = resp->cfge.osgi;
	info->orp = resp->cfge.orp;
	info->sdu_type = resp->cfge.sdu_type;
	info->msdu = le16_to_cpu(resp->cfge.msdu);
	info->rp_eid = le32_to_cpu(resp->cfge.rp_eid);
	info->sgi_eid = le32_to_cpu(resp->cfge.sgi_eid);
	info->isc_eid = le32_to_cpu(resp->cfge.isc_eid);

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isft_query_entry);

int ntmp_isft_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	struct isft_req_nua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Fill up NTMP request data buffer */
	req->ak.entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, sizeof(struct isft_resp_nq));
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_ISFT_ID,
				NTMP_CMD_DELETE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Delete ingress stream filter table entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isft_delete_entry);

int ntmp_sgclt_add_entry(struct netc_cbdr *cbdr, struct ntmp_sgclt_cfg *cfg)
{
	struct sgclt_req_add *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	int i, err;
	void *tmp;

	data_size = struct_size(req, cfge.ge, cfg->num_gates);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Fill up NTMP request data buffer */
	req->entry_id = cpu_to_le32(cfg->entry_id);
	req->cfge.ct = cpu_to_le32(cfg->ct);
	req->cfge.ext_gtst = 1;
	if (cfg->init_ipv >= 0) {
		req->cfge.ext_oipv = 1;
		req->cfge.ext_ipv = cfg->init_ipv & 0x7;
	}

	req->cfge.list_len = cfg->num_gates - 1;
	for (i = 0; i < cfg->num_gates; i++) {
		struct action_gate_entry *from = &cfg->entries[i];
		struct sgclt_ge *to = &req->cfge.ge[i];

		if (from->gate_state)
			to->gtst = ENETC_STREAM_GATE_STATE_OPEN;

		if (from->ipv >= 0) {
			to->oipv = 1;
			to->ipv = from->ipv & 0x7;
		}

		if (from->maxoctets >= 0) {
			to->iomen |= 0x01;
			to->iom[0] = from->maxoctets & 0xFF;
			to->iom[1] = (from->maxoctets >> 8) & 0xFF;
			to->iom[2] = (from->maxoctets >> 16) & 0xFF;
		}

		to->interval = from->interval;
	}

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_SGCLT_ID,
				NTMP_CMD_ADD, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Add stream gate control list entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_sgclt_add_entry);

int ntmp_sgclt_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			   struct ntmp_sgclt_info *info)
{
	struct sgclt_resp_query *resp;
	struct sgclt_req_qd *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	int i, err;
	void *tmp;

	if (!info || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data_size = struct_size(resp, cfge.ge, NTMP_SGCLT_MAX_GE_NUM);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Fill up NTMP request data buffer */
	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(sizeof(*req), data_size);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_SGCLT_ID,
				NTMP_CMD_QUERY, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev,
			"Query stream gate control list entry failed (%d)!", err);
		goto end;
	}

	resp = (struct sgclt_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(cbdr->dma_dev,
			"Entry ID doesn't match, query ID:0x%0x, response ID:0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	info->cycle_time = le32_to_cpu(resp->cfge.ct);
	info->ref_count = resp->ref_count;
	info->list_len = resp->cfge.list_len + 1;
	info->ext_gtst = resp->cfge.ext_gtst;
	info->ext_ipv = resp->cfge.ext_ipv;
	info->ext_oipv = resp->cfge.ext_oipv;
	for (i = 0; i < info->list_len; i++) {
		info->ge[i].interval = le32_to_cpu(resp->cfge.ge[i].interval);
		info->ge[i].iom = resp->cfge.ge[i].iom[2] << 16 | resp->cfge.ge[i].iom[1] << 8 |
				  resp->cfge.ge[i].iom[0];
		info->ge[i].ipv = resp->cfge.ge[i].ipv;
		info->ge[i].oipv = resp->cfge.ge[i].oipv;
		info->ge[i].iomen = resp->cfge.ge[i].iomen;
		info->ge[i].gtst = resp->cfge.ge[i].gtst;
	}

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_sgclt_query_entry);

int ntmp_sgclt_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	struct sgclt_req_qd *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Fill up NTMP request data buffer */
	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_SGCLT_ID,
				NTMP_CMD_DELETE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Delete stream gate control list entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_sgclt_delete_entry);

int ntmp_sgit_add_or_update_entry(struct netc_cbdr *cbdr, struct ntmp_sgit_cfg *cfg)
{
	struct sgit_req_ua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	u64 base_time;
	void *tmp;
	int err;

	base_time = cfg->admin_bt;
	if (cfg->admin_sgcl_eid != NTMP_NULL_ENTRY_ID)
		base_time = ntmp_adjust_base_time(cbdr, cfg->admin_bt, cfg->sgcl_ct);

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.update_act = cpu_to_le16(7);
	req->entry_id = cpu_to_le32(cfg->entry_id);
	req->acfge.admin_sgcl_eid = cpu_to_le32(cfg->admin_sgcl_eid);
	req->acfge.admin_bt = cpu_to_le64(base_time);
	req->acfge.admin_ct_ext = cpu_to_le32(cfg->admin_ct_ext);
	/* Specify the gate state to be open before the admin stream
	 * control list takes effect.
	 */
	req->icfge.gst = 1;
	if (cfg->init_ipv >= 0) {
		req->icfge.oipv = 1;
		req->icfge.ipv = cfg->init_ipv & 0x7;
	}

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_SGIT_ID,
				NTMP_CMD_AU, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Add/Update stream gate instance table entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_sgit_add_or_update_entry);

int ntmp_sgit_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			  struct ntmp_sgit_info *info)
{
	struct sgit_resp_query *resp;
	struct sgit_req_nua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!info || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data_size = sizeof(*resp);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(sizeof(*req), data_size);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_SGIT_ID,
				NTMP_CMD_QUERY, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev,
			"Query stream gate instance table entry failed (%d)!", err);
		goto end;
	}

	resp = (struct sgit_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(cbdr->dma_dev,
			"Entry ID doesn't match, query ID:0x%0x, response ID:0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	info->cfg_ct = le64_to_cpu(resp->sgise.cfg_ct);
	info->oper_bt = le64_to_cpu(resp->sgise.oper_bt);
	info->admin_bt = le64_to_cpu(resp->acfge.admin_bt);
	info->oper_ct_ext = le32_to_cpu(resp->sgise.oper_ct_ext);
	info->admin_ct_ext = le32_to_cpu(resp->acfge.admin_ct_ext);
	info->oper_sgcl_eid = le32_to_cpu(resp->sgise.oper_sgcl_eid);
	info->admin_sgcl_eid = le32_to_cpu(resp->acfge.admin_sgcl_eid);
	info->sdu_type = resp->cfge.sdu_type;
	info->state = resp->sgise.state;
	info->oex = resp->sgise.oex;
	info->oexen = resp->cfge.oexen;
	info->irx = resp->sgise.irx;
	info->irxen = resp->cfge.irxen;
	info->ipv = resp->icfge.ipv;
	info->oipv = resp->icfge.oipv;
	info->gst = resp->icfge.gst;

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_sgit_query_entry);

int ntmp_sgit_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	struct sgit_req_nua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Fill up NTMP request data buffer */
	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_SGIT_ID,
				NTMP_CMD_DELETE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Delete stream gate instance table entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_sgit_delete_entry);

int ntmp_isct_operate_entry(struct netc_cbdr *cbdr, u32 entry_id, int cmd,
			    struct ntmp_isct_info *info)
{
	struct isct_resp_query *resp;
	struct isct_req_data *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	bool query;
	void *tmp;
	int err;

	/* Check the command. */
	switch (cmd) {
	case NTMP_CMD_QUERY:
		if (!info)
			return -EINVAL;
		fallthrough;
	case NTMP_CMD_DELETE:
	case NTMP_CMD_UPDATE:
	case NTMP_CMD_QD:
	case NTMP_CMD_QU:
	case NTMP_CMD_ADD:
	break;
	default:
		return -EINVAL;
	}

	query = !!(cmd & NTMP_CMD_QUERY);
	data_size = query ? sizeof(*resp) : sizeof(*req);

	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	if (cmd & NTMP_CMD_UPDATE)
		req->crd.update_act = cpu_to_le16(1);
	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(sizeof(*req), query ? sizeof(*resp) : 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_ISCT_ID,
				cmd, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev,
			"Operate stream gate instance table entry (%d) failed (%d)!",
			cmd, err);
		goto end;
	}

	if (query) {
		resp = (struct isct_resp_query *)req;
		if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
			dev_err(cbdr->dma_dev,
				"Entry ID doesn't match, query ID:0x%0x, response ID:0x%x\n",
				entry_id, le32_to_cpu(resp->entry_id));
			err = -EIO;
			goto end;
		}

		info->rx_count = le32_to_cpu(resp->rx_count);
		info->msdu_drop_count = le32_to_cpu(resp->msdu_drop_count);
		info->policer_drop_count = le32_to_cpu(resp->policer_drop_count);
		info->sg_drop_count = le32_to_cpu(resp->sg_drop_count);
	}

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isct_operate_entry);

int ntmp_ipft_add_entry(struct netc_cbdr *cbdr, struct ntmp_ipft_key *key,
			struct ntmp_ipft_cfg *cfg, u32 *entry_id)
{
	struct ipft_resp_query *resp;
	struct ipft_req_add *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	int i, err;
	void *tmp;

	if (!key || !cfg || !entry_id)
		return -EINVAL;

	data_size = sizeof(*resp);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Fill up NTMP request data buffer */
	req->crd.update_act = cpu_to_le16(3);
	req->crd.query_act = 1;
	req->keye.precedence = cpu_to_le16(key->precedence);
	req->keye.frm_attr_flags = cpu_to_le16(key->frm_attr_flags);
	req->keye.frm_attr_flags_mask = cpu_to_le16(key->frm_attr_flags_mask);
	req->keye.dscp = cpu_to_le16(key->dscp);
	req->keye.src_port = cpu_to_le16(key->src_port);
	req->keye.outer_vlan_tci = key->outer_vlan_tci;
	req->keye.outer_vlan_tci_mask = key->outer_vlan_tci_mask;
	ether_addr_copy(req->keye.dmac, key->dmac);
	ether_addr_copy(req->keye.dmac_mask, key->dmac_mask);
	ether_addr_copy(req->keye.smac, key->smac);
	ether_addr_copy(req->keye.smac_mask, key->smac_mask);
	req->keye.inner_vlan_tci = key->inner_vlan_tci;
	req->keye.inner_vlan_tci_mask = key->inner_vlan_tci_mask;
	req->keye.ethertype = key->ethertype;
	req->keye.ethertype_mask = key->ethertype_mask;
	req->keye.ip_protocol = key->ip_protocol;
	req->keye.ip_protocol_mask = key->ip_protocol_mask;
	memcpy(req->keye.ip_src, key->ip_src, sizeof(req->keye.ip_src));
	memcpy(req->keye.ip_src_mask, key->ip_src_mask, sizeof(req->keye.ip_src_mask));
	req->keye.l4_src_port = key->l4_src_port;
	req->keye.l4_src_port_mask = key->l4_src_port_mask;
	memcpy(req->keye.ip_dst, key->ip_dst, sizeof(req->keye.ip_dst));
	memcpy(req->keye.ip_dst_mask, key->ip_dst_mask, sizeof(req->keye.ip_dst_mask));
	req->keye.l4_dst_port = key->l4_dst_port;
	req->keye.l4_dst_port_mask = key->l4_dst_port_mask;
	for (i = 0; i < NTMP_IPFT_MAX_PLD_LEN; i++) {
		req->keye.byte[i].data = key->byte[i].data;
		req->keye.byte[i].mask = key->byte[i].mask;
	}

	req->cfge.ipv = cfg->ipv;
	req->cfge.oipv = cfg->oipv;
	req->cfge.dr = cfg->dr;
	req->cfge.odr = cfg->odr;
	req->cfge.filter = cpu_to_le16(cfg->filter);
	req->cfge.flta_tgt = cpu_to_le32(cfg->flta_tgt);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(sizeof(*req), data_size);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_IPFT_ID,
				NTMP_CMD_AQ, NTMP_AM_TERNARY_KEY);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev,
			"Add ingress port filter table entry failed (%d)!", err);
		goto end;
	}

	resp = (struct ipft_resp_query *)req;
	*entry_id = le32_to_cpu(resp->entry_id);

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ipft_add_entry);

int ntmp_ipft_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			  struct ntmp_ipft_info *info)
{
	struct ipft_resp_query *resp;
	struct ipft_req_qd *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	int i, err;
	void *tmp;

	if (!info || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	data_size = sizeof(*resp);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Full Query */
	req->crd.query_act = 0;
	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(sizeof(*req), data_size);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_IPFT_ID,
				NTMP_CMD_QUERY, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err) {
		dev_err(cbdr->dma_dev,
			"Query ingress port filter table entry failed (%d)!", err);
		goto end;
	}

	resp = (struct ipft_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(cbdr->dma_dev,
			"Entry ID doesn't match, query ID:0x%0x, response ID:0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	info->key.precedence = le16_to_cpu(resp->keye.precedence);
	info->key.frm_attr_flags = le16_to_cpu(resp->keye.frm_attr_flags);
	info->key.frm_attr_flags_mask = le16_to_cpu(resp->keye.frm_attr_flags_mask);
	info->key.dscp = le16_to_cpu(resp->keye.dscp);
	info->key.src_port = le16_to_cpu(resp->keye.src_port);
	info->key.outer_vlan_tci = resp->keye.outer_vlan_tci;
	info->key.outer_vlan_tci_mask = resp->keye.outer_vlan_tci_mask;
	ether_addr_copy(info->key.dmac, resp->keye.dmac);
	ether_addr_copy(info->key.dmac_mask, resp->keye.dmac_mask);
	ether_addr_copy(info->key.smac, resp->keye.smac);
	ether_addr_copy(info->key.smac_mask, resp->keye.smac_mask);
	info->key.inner_vlan_tci = resp->keye.inner_vlan_tci;
	info->key.inner_vlan_tci_mask = resp->keye.inner_vlan_tci_mask;
	info->key.ethertype = resp->keye.ethertype;
	info->key.ethertype_mask = resp->keye.ethertype_mask;
	info->key.ip_protocol = resp->keye.ip_protocol;
	info->key.ip_protocol_mask = resp->keye.ip_protocol_mask;
	memcpy(info->key.ip_src, resp->keye.ip_src, sizeof(info->key.ip_src));
	memcpy(info->key.ip_src_mask, resp->keye.ip_src_mask, sizeof(info->key.ip_src_mask));
	info->key.l4_src_port = resp->keye.l4_src_port;
	info->key.l4_src_port_mask = resp->keye.l4_src_port_mask;
	memcpy(info->key.ip_dst, resp->keye.ip_dst, sizeof(info->key.ip_dst));
	memcpy(info->key.ip_dst_mask, resp->keye.ip_dst_mask, sizeof(info->key.ip_dst_mask));
	info->key.l4_dst_port = resp->keye.l4_dst_port;
	info->key.l4_dst_port_mask = resp->keye.l4_dst_port_mask;
	for (i = 0; i < NTMP_IPFT_MAX_PLD_LEN; i++) {
		info->key.byte[i].data = resp->keye.byte[i].data;
		info->key.byte[i].mask = resp->keye.byte[i].mask;
	}
	info->match_count = le64_to_cpu(resp->stse.match_count);

	info->cfg.ipv = resp->cfge.ipv;
	info->cfg.oipv = resp->cfge.oipv;
	info->cfg.dr = resp->cfge.dr;
	info->cfg.odr = resp->cfge.odr;
	info->cfg.filter = le16_to_cpu(resp->cfge.filter);
	info->cfg.flta_tgt = le32_to_cpu(resp->cfge.flta_tgt);

end:
	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ipft_query_entry);

int ntmp_ipft_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	struct ipft_resp_nq *resp;
	struct ipft_req_qd *req;
	union netc_cbd cbd;
	u32 len, data_size;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(cbdr, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Set mac address filter table request data buffer */
	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, sizeof(*resp));
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_IPFT_ID,
				NTMP_CMD_DELETE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdr, &cbd);
	if (err)
		dev_err(cbdr->dma_dev,
			"Delete ingress port filter table entry failed (%d)!", err);

	ntmp_free_data_mem(cbdr, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ipft_delete_entry);

MODULE_AUTHOR("Wei Fang <wei.fang@nxp.com>");
MODULE_DESCRIPTION("NXP NETC Table Management Protocol");
MODULE_LICENSE("Dual BSD/GPL");
