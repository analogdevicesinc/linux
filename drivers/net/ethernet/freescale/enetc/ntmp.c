// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * NETC NTMP (NETC Table Management Protocol) 2.0 driver
 * Copyright 2023 NXP
 * Copyright (c) 2023 Wei Fang <wei.fang@nxp.com>
 */
#include <linux/iopoll.h>
#include <linux/fsl/netc_global.h>
#include <linux/fsl/netc_lib.h>

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

#define NTMP_QUERY_ACT_ENTRY_ID		1

/* Generic Update Actions for most tables */
#define NTMP_GEN_UA_CFGEU		BIT(0)
#define NTMP_GEN_UA_STSEU		BIT(1)

/* Update Actions for specific tables */
#define NTMP_SGIT_UA_ACFGEU		BIT(0)
#define NTMP_SGIT_UA_CFGEU		BIT(1)
#define NTMP_SGIT_UA_SGISEU		BIT(2)
#define NTMP_RPT_UA_FEEU		BIT(1)
#define NTMP_RPT_UA_PSEU		BIT(2)
#define NTMP_RPT_UA_STSEU		BIT(3)

#define ENETC_RSS_TABLE_ENTRY_NUM	64
#define ENETC_RSS_STSE_DATA_SIZE(n)	((n) * 8)
#define ENETC_RSS_CFGE_DATA_SIZE(n)	(n)

#define NTMP_REQ_RESP_LEN(req, resp)	(((req) << 20 & NTMP_REQ_LEN_MASK) | \
					 ((resp) & NTMP_RESP_LEN_MASK))

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

	/* The base address of the Control BD Ring must be 128 bytes aligned */
	cbdr->dma_base_align =  ALIGN(cbdr->dma_base,
				      NETC_CBDR_BASE_ADDR_ALIGN);
	cbdr->addr_base_align = PTR_ALIGN(cbdr->addr_base,
					  NETC_CBDR_BASE_ADDR_ALIGN);

	cbdr->next_to_clean = 0;
	cbdr->next_to_use = 0;
	spin_lock_init(&cbdr->ring_lock);

	/* Step 1: Configure the base address of the Control BD Ring */
	netc_write(cbdr->regs.bar0, lower_32_bits(cbdr->dma_base_align));
	netc_write(cbdr->regs.bar1, upper_32_bits(cbdr->dma_base_align));

	/* Step 2: Configure the producer index register */
	netc_write(cbdr->regs.pir, cbdr->next_to_clean);

	/* Step 3: Configure the consumer index register */
	netc_write(cbdr->regs.cir, cbdr->next_to_use);

	/* Step4: Configure the number of BDs of the Control BD Ring */
	netc_write(cbdr->regs.lenr, cbdr->bd_num);

	/* Step 5: Enable the Control BD Ring */
	netc_write(cbdr->regs.mr, NETC_CBDRMR_EN);

	return 0;
}
EXPORT_SYMBOL_GPL(netc_setup_cbdr);

void netc_teardown_cbdr(struct device *dev, struct netc_cbdr *cbdr)
{
	/* Disable the Control BD Ring */
	netc_write(cbdr->regs.mr, 0);

	dma_free_coherent(dev, cbdr->dma_size, cbdr->addr_base, cbdr->dma_base);

	memset(cbdr, 0, sizeof(*cbdr));
}
EXPORT_SYMBOL_GPL(netc_teardown_cbdr);

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
	while (netc_read(cbdr->regs.cir) != i) {
		cbd = netc_get_cbd(cbdr, i);
		memset(cbd, 0, sizeof(*cbd));
		i = (i + 1) % cbdr->bd_num;
	}

	cbdr->next_to_clean = i;
}

static struct netc_cbdr *netc_select_cbdr(struct netc_cbdrs *cbdrs)
{
	int cpu, i;

	for (i = 0; i < cbdrs->cbdr_num; i++) {
		if (spin_is_locked(&cbdrs->ring[i].ring_lock))
			continue;

		return &cbdrs->ring[i];
	}

	/* If all the command BDRs are busy now, we select
	 * one of them, but need to wait for a while to use.
	 */
	cpu = smp_processor_id();

	return &cbdrs->ring[cpu % cbdrs->cbdr_num];
}

static int netc_xmit_ntmp_cmd(struct netc_cbdrs *cbdrs, union netc_cbd *cbd)
{
	union netc_cbd *ring_cbd;
	struct netc_cbdr *cbdr;
	int i, err;
	u16 status;
	u32 val;

	if (cbdrs->cbdr_num == 1)
		cbdr = cbdrs->ring;
	else
		cbdr = netc_select_cbdr(cbdrs);

	if (unlikely(!cbdr->addr_base))
		return -EFAULT;

	spin_lock_bh(&cbdr->ring_lock);

	if (unlikely(!netc_get_free_cbd_num(cbdr)))
		netc_clean_cbdr(cbdr);

	i = cbdr->next_to_use;
	ring_cbd = netc_get_cbd(cbdr, i);

	/* Copy command BD to the ring */
	*ring_cbd = *cbd;
	/* Update producer index of both software and hardware */
	i = (i + 1) % cbdr->bd_num;
	cbdr->next_to_use = i;
	dma_wmb();
	netc_write(cbdr->regs.pir, i);

	err = read_poll_timeout_atomic(netc_read, val, val == i,
				       10, NETC_CBDR_TIMEOUT, true,
				       cbdr->regs.cir);
	if (unlikely(err)) {
		err = -EBUSY;
		goto err_unlock;
	}

	/* The caller may need to check other fields in the response header */
	*cbd = *ring_cbd;
	/* Check the writeback error status */
	dma_rmb();
	/* Check the writeback error status */
	status = le16_to_cpu(ring_cbd->ntmp_resp_hdr.error_rr) & NTMP_RESP_HDR_ERR;
	if (unlikely(status)) {
		dev_err(cbdrs->dma_dev, "Command BD error: 0x%04x\n", status);
		err = -EIO;
	}

	netc_clean_cbdr(cbdr);
	dma_wmb();

err_unlock:
	spin_unlock_bh(&cbdr->ring_lock);

	return err;
}

static void *ntmp_alloc_data_mem(struct device *dev, int size,
				 dma_addr_t *dma, void **data_align)
{
	void *data;

	data = dma_alloc_coherent(dev, size + NETC_CBD_DATA_ADDR_ALIGN,
				  dma, GFP_ATOMIC);
	if (!data) {
		dev_err(dev, "NTMP alloc data memory failed!\n");
		return NULL;
	}

	*data_align = PTR_ALIGN(data, NETC_CBD_DATA_ADDR_ALIGN);

	return data;
}

static void ntmp_free_data_mem(struct device *dev, int size,
			       void *data, dma_addr_t dma)
{
	dma_free_coherent(dev, size + NETC_CBD_DATA_ADDR_ALIGN, data, dma);
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

u32 ntmp_lookup_free_eid(unsigned long *bitmap, u32 bitmap_size)
{
	u32 entry_id;

	if (!bitmap)
		return NTMP_NULL_ENTRY_ID;

	entry_id = find_first_zero_bit(bitmap, bitmap_size);
	if (entry_id == bitmap_size)
		return NTMP_NULL_ENTRY_ID;

	/* Set the bit once we found it */
	set_bit(entry_id, bitmap);

	return entry_id;
}
EXPORT_SYMBOL_GPL(ntmp_lookup_free_eid);

void ntmp_clear_eid_bitmap(unsigned long *bitmap, u32 entry_id)
{
	if (!bitmap || entry_id == NTMP_NULL_ENTRY_ID)
		return;

	clear_bit(entry_id, bitmap);
}
EXPORT_SYMBOL_GPL(ntmp_clear_eid_bitmap);

u32 ntmp_lookup_free_words(unsigned long *bitmap, u32 bitmap_size,
			   u32 num_words)
{
	u32 entry_id, next_eid, size;

	if (!bitmap)
		return NTMP_NULL_ENTRY_ID;

	do {
		entry_id = find_first_zero_bit(bitmap, bitmap_size);
		if (entry_id == bitmap_size)
			return NTMP_NULL_ENTRY_ID;

		next_eid = find_next_bit(bitmap, bitmap_size, entry_id + 1);
		size = next_eid - entry_id;
	} while (size < num_words && next_eid != bitmap_size);

	if (size < num_words)
		return NTMP_NULL_ENTRY_ID;

	bitmap_set(bitmap, entry_id, num_words);

	return entry_id;
}
EXPORT_SYMBOL_GPL(ntmp_lookup_free_words);

void ntmp_clear_words_bitmap(unsigned long *bitmap, u32 entry_id,
			     u32 num_words)
{
	if (!bitmap || entry_id == NTMP_NULL_ENTRY_ID)
		return;

	bitmap_clear(bitmap, entry_id, num_words);
}
EXPORT_SYMBOL_GPL(ntmp_clear_words_bitmap);

static int ntmp_delete_entry_by_id(struct netc_cbdrs *cbdrs, int tbl_id, u8 tbl_ver,
				   u32 entry_id, u32 req_len, u32 resp_len)
{
	struct device *dev = cbdrs->dma_dev;
	struct ntmp_qd_by_eid *req;
	union netc_cbd cbd;
	u32 len, dma_len;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (entry_id == NTMP_NULL_ENTRY_ID)
		return 0;

	/* If the req_len is 0, indicates the requested length it the
	 * standard length.
	 */
	if (!req_len)
		req_len = sizeof(*req);

	dma_len = req_len >= resp_len ? req_len : resp_len;
	tmp = ntmp_alloc_data_mem(dev, dma_len, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.tbl_ver = tbl_ver;
	req->entry_id = cpu_to_le32(entry_id);
	len = NTMP_REQ_RESP_LEN(req_len, resp_len);
	ntmp_fill_request_headr(&cbd, dma, len, tbl_id,
				NTMP_CMD_DELETE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(dev, "Delete table (id: %d) entry failed: %d!",
			tbl_id, err);

	ntmp_free_data_mem(dev, dma_len, tmp, dma);

	return err;
}

static int ntmp_query_entry_by_id(struct netc_cbdrs *cbdrs, int tbl_id,
				  u32 len, struct ntmp_qd_by_eid *req,
				  dma_addr_t *dma, bool compare_eid)
{
	struct device *dev = cbdrs->dma_dev;
	struct common_resp_query *resp;
	int cmd = NTMP_CMD_QUERY;
	union netc_cbd cbd;
	u32 entry_id;
	int err;

	entry_id = le32_to_cpu(req->entry_id);
	if (le16_to_cpu(req->crd.update_act))
		cmd = NTMP_CMD_QU;

	/* Request header */
	ntmp_fill_request_headr(&cbd, *dma, len, tbl_id,
				cmd, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err) {
		dev_err(dev, "Query table (id: %d) entry failed: %d\n",
			tbl_id, err);
		return err;
	}

	/* For a few tables, the first field of its response data
	 * is not entry_id or not the entry_id of current table.
	 * So return directly here.
	 */
	if (!compare_eid)
		return 0;

	resp = (struct common_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(dev, "Table (id: %d) query EID:0x%0x, response EID:0x%x\n",
			tbl_id, entry_id, le32_to_cpu(resp->entry_id));
		return -EIO;
	}

	return 0;
}

int ntmp_maft_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			struct maft_entry_data *data)
{
	struct device *dev = cbdrs->dma_dev;
	struct maft_req_add *req;
	union netc_cbd cbd;
	u32 len, req_len;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!data)
		return -EINVAL;

	req_len = sizeof(*req);
	tmp = ntmp_alloc_data_mem(dev, req_len, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Set mac address filter table request data buffer */
	req->crd.tbl_ver = cbdrs->tbl.maft_ver;
	req->entry_id = cpu_to_le32(entry_id);
	req->keye = data->keye;
	req->cfge = data->cfge;

	len = NTMP_REQ_RESP_LEN(req_len, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_MAFT_ID,
				NTMP_CMD_ADD, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(dev, "Add MAFT entry failed (%d)!", err);

	ntmp_free_data_mem(dev, req_len, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_maft_add_entry);

int ntmp_maft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct maft_entry_data *data)
{
	struct device *dev = cbdrs->dma_dev;
	struct maft_resp_query *resp;
	u32 resp_len = sizeof(*resp);
	struct ntmp_qd_by_eid *req;
	u32 req_len = sizeof(*req);
	void *tmp = NULL;
	dma_addr_t dma;
	u32 dma_len;
	int err;

	if (!data || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	dma_len = max_t(u32, req_len, resp_len);
	tmp = ntmp_alloc_data_mem(dev, dma_len, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.tbl_ver = cbdrs->tbl.maft_ver;
	req->entry_id = cpu_to_le32(entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_MAFT_ID,
				     NTMP_REQ_RESP_LEN(req_len, resp_len),
				     req, &dma, true);
	if (err)
		goto end;

	resp = (struct maft_resp_query *)req;
	data->keye = resp->keye;
	data->cfge = resp->cfge;

end:
	ntmp_free_data_mem(dev, dma_len, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_maft_query_entry);

int ntmp_maft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return ntmp_delete_entry_by_id(cbdrs, NTMP_MAFT_ID, cbdrs->tbl.maft_ver,
				       entry_id, 0, 0);
}
EXPORT_SYMBOL_GPL(ntmp_maft_delete_entry);

int ntmp_vaft_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			struct vaft_entry_data *data)
{
	struct device *dev = cbdrs->dma_dev;
	struct vaft_req_add *req;
	union netc_cbd cbd;
	u32 len, data_size;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!data)
		return -EINVAL;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(dev, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Set VLAN address filter table request data buffer */
	req->crd.tbl_ver = cbdrs->tbl.vaft_ver;
	req->entry_id = cpu_to_le32(entry_id);
	req->keye = data->keye;
	req->cfge = data->cfge;

	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_VAFT_ID,
				NTMP_CMD_ADD, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(dev, "Add VAFT entry failed (%d)!", err);

	ntmp_free_data_mem(dev, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_vaft_add_entry);

int ntmp_vaft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct vaft_entry_data *data)
{
	struct device *dev = cbdrs->dma_dev;
	struct vaft_resp_query *resp;
	u32 resp_len = sizeof(*resp);
	struct ntmp_qd_by_eid *req;
	u32 req_len = sizeof(*req);
	void *tmp = NULL;
	dma_addr_t dma;
	u32 dma_len;
	int err;

	if (!data || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	dma_len = max_t(u32, req_len, resp_len);
	tmp = ntmp_alloc_data_mem(dev, dma_len, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.tbl_ver = cbdrs->tbl.vaft_ver;
	req->entry_id = cpu_to_le32(entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_VAFT_ID,
				     NTMP_REQ_RESP_LEN(req_len, resp_len),
				     req, &dma, true);
	if (err)
		goto end;

	resp = (struct vaft_resp_query *)req;
	data->keye = resp->keye;
	data->cfge = resp->cfge;

end:
	ntmp_free_data_mem(dev, dma_len, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_vaft_query_entry);

int ntmp_vaft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return ntmp_delete_entry_by_id(cbdrs, NTMP_VAFT_ID, cbdrs->tbl.maft_ver,
				       entry_id, 0, 0);
}
EXPORT_SYMBOL_GPL(ntmp_vaft_delete_entry);

int ntmp_rsst_query_or_update_entry(struct netc_cbdrs *cbdrs, u32 *table,
				    int count, bool query)
{
	struct device *dev = cbdrs->dma_dev;
	struct rsst_req_update *requ;
	struct ntmp_qd_by_eid *req;
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

	tmp = ntmp_alloc_data_mem(dev, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Set the request data buffer */
	req->crd.tbl_ver = cbdrs->tbl.rsst_ver;
	if (query) {
		len = NTMP_REQ_RESP_LEN(sizeof(*req), data_size);
		ntmp_fill_request_headr(&cbd, dma, len, NTMP_RSST_ID,
					NTMP_CMD_QUERY, NTMP_AM_ENTRY_ID);
	} else {
		requ = (struct rsst_req_update *)req;
		requ->crd.update_act = cpu_to_le16(NTMP_GEN_UA_CFGEU |
						   NTMP_GEN_UA_STSEU);
		for (i = 0; i < count; i++)
			requ->groups[i] = (u8)(table[i]);

		len = NTMP_REQ_RESP_LEN(data_size, 0);
		ntmp_fill_request_headr(&cbd, dma, len, NTMP_RSST_ID,
					NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);
	}

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err) {
		dev_err(dev, "%s RSS table entry failed (%d)!",
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
	ntmp_free_data_mem(dev, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_rsst_query_or_update_entry);

/* Test codes for Time gate scheduling table */
int ntmp_tgst_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct tgst_query_data *data)
{
	struct device *dev = cbdrs->dma_dev;
	struct tgst_resp_query *resp;
	u32 resp_len = sizeof(*resp);
	struct tgst_cfge_data *cfge;
	struct tgst_olse_data *olse;
	struct ntmp_qd_by_eid *req;
	u32 req_len = sizeof(*req);
	void *tmp = NULL;
	dma_addr_t dma;
	u32 dma_len;
	int i, err;

	if (!data || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	resp_len += struct_size(cfge, ge, NTMP_TGST_MAX_ENTRY_NUM) +
		    struct_size(olse, ge, NTMP_TGST_MAX_ENTRY_NUM);
	dma_len = max_t(u32, req_len, resp_len);
	tmp = ntmp_alloc_data_mem(dev, dma_len, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.tbl_ver = cbdrs->tbl.tgst_ver;
	req->entry_id = cpu_to_le32(entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_TGST_ID,
				     NTMP_REQ_RESP_LEN(req_len, resp_len),
				     req, &dma, false);
	if (err)
		goto end;

	resp = (struct tgst_resp_query *)req;
	cfge = (struct tgst_cfge_data *)resp->data;

	data->config_change_time = resp->status.cfg_ct;
	data->admin_bt = cfge->admin_bt;
	data->admin_ct = cfge->admin_ct;
	data->admin_ct_ext = cfge->admin_ct_ext;
	data->admin_cl_len = cfge->admin_cl_len;
	for (i = 0; i < le16_to_cpu(cfge->admin_cl_len); i++) {
		data->cfge_ge[i].interval = cfge->ge[i].interval;
		data->cfge_ge[i].tc_state = cfge->ge[i].tc_state;
		data->cfge_ge[i].hr_cb = cfge->ge[i].hr_cb;
	}

	olse = (struct tgst_olse_data *)&cfge->ge[i];
	data->oper_cfg_ct = olse->oper_cfg_ct;
	data->oper_cfg_ce = olse->oper_cfg_ce;
	data->oper_bt = olse->oper_bt;
	data->oper_ct = olse->oper_ct;
	data->oper_ct_ext = olse->oper_ct_ext;
	for (i = 0; i < le16_to_cpu(olse->oper_cl_len); i++) {
		data->olse_ge[i].interval = olse->ge[i].interval;
		data->olse_ge[i].tc_state = olse->ge[i].tc_state;
		data->olse_ge[i].hr_cb = olse->ge[i].hr_cb;
	}

end:
	ntmp_free_data_mem(dev, dma_len, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_tgst_query_entry);

int ntmp_tgst_delete_admin_gate_list(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	struct device *dev = cbdrs->dma_dev;
	struct tgst_req_update *req;
	struct tgst_cfge_data *cfge;
	union netc_cbd cbd;
	u32 len, data_size;
	dma_addr_t dma;
	void *tmp;
	int err;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(dev, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	cfge = &req->cfge;

	/* Set the request data buffer and set the admin control list len
	 * to zero to delete the existing admin control list.
	 */
	req->crd.update_act = cpu_to_le16(NTMP_GEN_UA_CFGEU);
	req->crd.tbl_ver = cbdrs->tbl.tgst_ver;
	req->entry_id = cpu_to_le32(entry_id);
	cfge->admin_cl_len = 0;

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, sizeof(struct tgst_resp_status));
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_TGST_ID,
				NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(dev, "Delete TGST entry failed (%d)!", err);

	ntmp_free_data_mem(dev, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_tgst_delete_admin_gate_list);

int ntmp_tgst_update_admin_gate_list(struct netc_cbdrs *cbdrs, u32 entry_id,
				     struct tgst_cfge_data *cfge)
{
	struct device *dev = cbdrs->dma_dev;
	struct tgst_req_update *req;
	u32 len, req_len, cfge_len;
	union netc_cbd cbd;
	dma_addr_t dma;
	u16 list_len;
	void *tmp;
	int err;

	if (!cfge)
		return -EINVAL;

	list_len = le16_to_cpu(cfge->admin_cl_len);
	cfge_len = struct_size(cfge, ge, list_len);

	/* Calculate the size of request data buffer */
	req_len = struct_size(req, cfge.ge, list_len);
	tmp = ntmp_alloc_data_mem(dev, req_len, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Set the request data buffer */
	req->crd.update_act = cpu_to_le16(NTMP_GEN_UA_CFGEU);
	req->crd.tbl_ver = cbdrs->tbl.tgst_ver;
	req->entry_id = cpu_to_le32(entry_id);
	memcpy(&req->cfge, cfge, cfge_len);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(req_len, sizeof(struct tgst_resp_status));
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_TGST_ID,
				NTMP_CMD_UPDATE, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(dev, "Update TGST entry failed (%d)!", err);

	ntmp_free_data_mem(dev, req_len, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_tgst_update_admin_gate_list);

int ntmp_rpt_add_or_update_entry(struct netc_cbdrs *cbdrs,
				 struct ntmp_rpt_entry *entry)
{
	struct device *dev = cbdrs->dma_dev;
	struct rpt_req_ua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!entry)
		return -EINVAL;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(dev, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.update_act = cpu_to_le16(NTMP_GEN_UA_CFGEU | NTMP_RPT_UA_FEEU |
					  NTMP_RPT_UA_PSEU | NTMP_RPT_UA_STSEU);
	req->crd.tbl_ver = cbdrs->tbl.rpt_ver;
	req->entry_id = cpu_to_le32(entry->entry_id);
	req->cfge = entry->cfge;
	req->fee = entry->fee;

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_RPT_ID,
				NTMP_CMD_AU, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(dev, "Add/Update RPT entry failed (%d)!", err);

	ntmp_free_data_mem(dev, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_rpt_add_or_update_entry);

int ntmp_rpt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			 struct ntmp_rpt_entry *entry)
{
	struct device *dev = cbdrs->dma_dev;
	struct rpt_resp_query *resp;
	u32 resp_len = sizeof(*resp);
	struct ntmp_qd_by_eid *req;
	u32 req_len = sizeof(*req);
	void *tmp = NULL;
	dma_addr_t dma;
	u32 dma_len;
	int err;

	if (!entry || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	dma_len = max_t(u32, req_len, resp_len);
	tmp = ntmp_alloc_data_mem(dev, dma_len, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.tbl_ver = cbdrs->tbl.rpt_ver;
	req->entry_id = cpu_to_le32(entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_RPT_ID,
				     NTMP_REQ_RESP_LEN(req_len, resp_len),
				     req, &dma, true);
	if (err)
		goto end;

	resp = (struct rpt_resp_query *)req;
	entry->stse = resp->stse;
	entry->cfge = resp->cfge;
	entry->fee = resp->fee;
	entry->pse = resp->pse;

end:
	ntmp_free_data_mem(dev, dma_len, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_rpt_query_entry);

int ntmp_rpt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return ntmp_delete_entry_by_id(cbdrs, NTMP_RPT_ID, cbdrs->tbl.rpt_ver,
				       entry_id, 0, 0);
}
EXPORT_SYMBOL_GPL(ntmp_rpt_delete_entry);

int ntmp_isit_add_or_update_entry(struct netc_cbdrs *cbdrs, bool add,
				  struct ntmp_isit_entry *entry)
{
	struct device *dev = cbdrs->dma_dev;
	struct isit_resp_query *resp;
	struct isit_req_ua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!entry)
		return -EINVAL;

	data_size = add ? sizeof(*resp) : sizeof(*req);
	tmp = ntmp_alloc_data_mem(dev, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	if (!add)
		req->crd.update_act = cpu_to_le16(NTMP_GEN_UA_CFGEU);
	else
		/* Query ENTRY_ID only */
		req->crd.query_act = NTMP_QUERY_ACT_ENTRY_ID;

	req->crd.tbl_ver = cbdrs->tbl.isit_ver;
	req->ak.keye = entry->keye;
	req->is_eid = entry->is_eid;

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
		len = NTMP_REQ_RESP_LEN(sizeof(*req), sizeof(struct common_resp_nq));
		ntmp_fill_request_headr(&cbd, dma, len, NTMP_ISIT_ID,
					NTMP_CMD_UPDATE, NTMP_AM_EXACT_KEY);
	}

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err) {
		dev_err(dev, "%s ISIT entry failed (%d)!",
			add ? "Add" : "Update", err);
		goto end;
	}

	if (add) {
		resp = (struct isit_resp_query *)req;
		entry->entry_id = le32_to_cpu(resp->entry_id);
	}

end:
	ntmp_free_data_mem(dev, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isit_add_or_update_entry);

int ntmp_isit_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct ntmp_isit_entry *entry)
{
	struct device *dev = cbdrs->dma_dev;
	struct isit_resp_query *resp;
	u32 resp_len = sizeof(*resp);
	struct isit_req_qd *req;
	u32 req_len, dma_len;
	void *tmp = NULL;
	dma_addr_t dma;
	int err;

	if (!entry || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	req_len = sizeof(*req);
	dma_len = max_t(u32, req_len, resp_len);
	tmp = ntmp_alloc_data_mem(dev, dma_len, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.tbl_ver = cbdrs->tbl.isit_ver;
	req->ak.eid.entry_id = cpu_to_le32(entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_ISIT_ID,
				     NTMP_REQ_RESP_LEN(req_len, resp_len),
				     (struct ntmp_qd_by_eid *)req, &dma, false);
	if (err)
		goto end;

	resp = (struct isit_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(dev, "ISIT Query EID:0x%0x, Response EID:0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	entry->keye = resp->keye;
	entry->is_eid = resp->is_eid;

end:
	ntmp_free_data_mem(dev, dma_len, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isit_query_entry);

int ntmp_isit_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	u32 resp_len = sizeof(struct common_resp_nq);
	u32 req_len = sizeof(struct isit_req_qd);

	return ntmp_delete_entry_by_id(cbdrs, NTMP_ISIT_ID, cbdrs->tbl.isit_ver,
				       entry_id, req_len, resp_len);
}
EXPORT_SYMBOL_GPL(ntmp_isit_delete_entry);

int ntmp_ist_add_or_update_entry(struct netc_cbdrs *cbdrs,
				 struct ntmp_ist_entry *entry)
{
	struct device *dev = cbdrs->dma_dev;
	struct ist_req_ua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!entry)
		return -EINVAL;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(dev, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Fill up NTMP request data buffer */
	req->crd.update_act = cpu_to_le16(NTMP_GEN_UA_CFGEU);
	req->crd.tbl_ver = cbdrs->tbl.ist_ver;
	req->entry_id = cpu_to_le32(entry->entry_id);
	req->cfge = entry->cfge;

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_IST_ID,
				NTMP_CMD_AU, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(dev, "Add/Update IST entry failed (%d)!", err);

	ntmp_free_data_mem(dev, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ist_add_or_update_entry);

int ntmp_ist_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			 struct ist_cfge_data *cfge)
{
	struct device *dev = cbdrs->dma_dev;
	struct ist_resp_query *resp;
	struct ntmp_qd_by_eid *req;
	u32 req_len = sizeof(*req);
	u32 resp_len, dma_len;
	void *tmp = NULL;
	dma_addr_t dma;
	int err;

	if (!cfge || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	resp_len = sizeof(*resp);
	dma_len = max_t(u32, req_len, resp_len);
	tmp = ntmp_alloc_data_mem(dev, dma_len, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.tbl_ver = cbdrs->tbl.ist_ver;
	req->entry_id = cpu_to_le32(entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_IST_ID,
				     NTMP_REQ_RESP_LEN(req_len, resp_len),
				     req, &dma, true);
	if (err)
		goto end;

	resp = (struct ist_resp_query *)req;
	*cfge = resp->cfge;

end:
	ntmp_free_data_mem(dev, dma_len, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ist_query_entry);

int ntmp_ist_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return ntmp_delete_entry_by_id(cbdrs, NTMP_IST_ID, cbdrs->tbl.ist_ver,
				       entry_id, 0, 0);
}
EXPORT_SYMBOL_GPL(ntmp_ist_delete_entry);

int ntmp_isft_add_or_update_entry(struct netc_cbdrs *cbdrs, bool add,
				  struct ntmp_isft_entry *entry)
{
	struct device *dev = cbdrs->dma_dev;
	struct isft_resp_query *resp;
	struct isft_req_ua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!entry)
		return -EINVAL;

	data_size = add ? sizeof(*resp) : sizeof(*req);
	tmp = ntmp_alloc_data_mem(dev, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.update_act = cpu_to_le16(NTMP_GEN_UA_CFGEU);
	req->crd.tbl_ver = cbdrs->tbl.isft_ver;
	if (add)
		req->crd.query_act = NTMP_QUERY_ACT_ENTRY_ID;

	req->ak.keye = entry->keye;
	req->cfge = entry->cfge;

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
		len = NTMP_REQ_RESP_LEN(sizeof(*req), sizeof(struct common_resp_nq));
		ntmp_fill_request_headr(&cbd, dma, len, NTMP_ISFT_ID,
					NTMP_CMD_UPDATE, NTMP_AM_EXACT_KEY);
	}

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err) {
		dev_err(dev, "%s ISFT entry failed (%d)!",
			add ? "Add" : "Update", err);
		goto end;
	}

	if (add) {
		resp = (struct isft_resp_query *)req;
		entry->entry_id = le32_to_cpu(resp->entry_id);
	}

end:
	ntmp_free_data_mem(dev, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isft_add_or_update_entry);

int ntmp_isft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct ntmp_isft_entry *entry)
{
	struct device *dev = cbdrs->dma_dev;
	struct isft_resp_query *resp;
	u32 resp_len = sizeof(*resp);
	struct isft_req_qd *req;
	u32 req_len, dma_len;
	void *tmp = NULL;
	dma_addr_t dma;
	int err;

	if (!entry || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	req_len = sizeof(*req);
	dma_len = max_t(u32, req_len, resp_len);
	tmp = ntmp_alloc_data_mem(dev, dma_len, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.tbl_ver = cbdrs->tbl.isft_ver;
	req->ak.eid.entry_id = cpu_to_le32(entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_ISFT_ID,
				     NTMP_REQ_RESP_LEN(req_len, resp_len),
				     (struct ntmp_qd_by_eid *)req, &dma, false);
	if (err)
		goto end;

	resp = (struct isft_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(dev, "ISFT Query EID:0x%0x, Response EID:0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	entry->keye = resp->keye;
	entry->cfge = resp->cfge;

end:
	ntmp_free_data_mem(dev, dma_len, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isft_query_entry);

int ntmp_isft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	u32 resp_len = sizeof(struct common_resp_nq);
	u32 req_len = sizeof(struct isft_req_qd);

	return ntmp_delete_entry_by_id(cbdrs, NTMP_ISFT_ID, cbdrs->tbl.isft_ver,
				       entry_id, req_len, resp_len);
}
EXPORT_SYMBOL_GPL(ntmp_isft_delete_entry);

int ntmp_sgclt_add_entry(struct netc_cbdrs *cbdrs,
			 struct ntmp_sgclt_entry *entry)
{
	struct device *dev = cbdrs->dma_dev;
	struct sgclt_req_add *req;
	u32 num_gates, cfge_len;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!entry)
		return -EINVAL;

	num_gates = entry->cfge.list_length + 1;
	data_size = struct_size(req, cfge.ge, num_gates);
	tmp = ntmp_alloc_data_mem(dev, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Fill up NTMP request data buffer */
	req->crd.tbl_ver = cbdrs->tbl.sgclt_ver;
	req->entry_id = cpu_to_le32(entry->entry_id);
	cfge_len = struct_size_t(struct sgclt_cfge_data, ge, num_gates);
	memcpy(&req->cfge, &entry->cfge, cfge_len);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_SGCLT_ID,
				NTMP_CMD_ADD, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(dev, "Add SGCLT entry failed (%d)!", err);

	ntmp_free_data_mem(dev, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_sgclt_add_entry);

int ntmp_sgclt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			   struct ntmp_sgclt_entry *entry, u32 cfge_size)
{
	struct device *dev = cbdrs->dma_dev;
	struct sgclt_resp_query *resp;
	struct ntmp_qd_by_eid *req;
	u32 req_len = sizeof(*req);
	u32 num_gates, cfge_len;
	u32 resp_len, dma_len;
	void *tmp = NULL;
	dma_addr_t dma;
	int err;

	if (!entry || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	resp_len = struct_size(resp, cfge.ge, NTMP_SGCLT_MAX_GE_NUM);
	dma_len = max_t(u32, req_len, resp_len);
	tmp = ntmp_alloc_data_mem(dev, dma_len, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.tbl_ver = cbdrs->tbl.sgclt_ver;
	req->entry_id = cpu_to_le32(entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_SGCLT_ID,
				     NTMP_REQ_RESP_LEN(req_len, resp_len),
				     req, &dma, true);
	if (err)
		goto end;

	resp = (struct sgclt_resp_query *)req;
	entry->ref_count = resp->ref_count;
	num_gates = resp->cfge.list_length + 1;
	cfge_len = struct_size_t(struct sgclt_cfge_data, ge, num_gates);
	if (cfge_len > cfge_size) {
		err = -ENOMEM;
		dev_err(dev, "SGCLT_CFGE buffer size is %u, larger than %u\n",
			cfge_size, cfge_len);

		goto end;
	}

	memcpy(&entry->cfge, &resp->cfge, cfge_len);

end:
	ntmp_free_data_mem(dev, dma_len, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_sgclt_query_entry);

int ntmp_sgclt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return ntmp_delete_entry_by_id(cbdrs, NTMP_SGCLT_ID, cbdrs->tbl.sgclt_ver,
				       entry_id, 0, 0);
}
EXPORT_SYMBOL_GPL(ntmp_sgclt_delete_entry);

int ntmp_sgit_add_or_update_entry(struct netc_cbdrs *cbdrs,
				  struct ntmp_sgit_entry *entry)
{
	struct device *dev = cbdrs->dma_dev;
	struct sgit_req_ua *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!entry)
		return -EINVAL;

	data_size = sizeof(*req);
	tmp = ntmp_alloc_data_mem(dev, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.update_act = cpu_to_le16(NTMP_SGIT_UA_ACFGEU | NTMP_SGIT_UA_CFGEU |
					  NTMP_SGIT_UA_SGISEU);
	req->crd.tbl_ver = cbdrs->tbl.sgit_ver;
	req->entry_id = cpu_to_le32(entry->entry_id);
	req->acfge = entry->acfge;
	req->cfge = entry->cfge;
	req->icfge = entry->icfge;

	/* Request header */
	len = NTMP_REQ_RESP_LEN(data_size, 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_SGIT_ID,
				NTMP_CMD_AU, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err)
		dev_err(dev, "Add/Update SGIT entry failed (%d)!", err);

	ntmp_free_data_mem(dev, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_sgit_add_or_update_entry);

int ntmp_sgit_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct ntmp_sgit_entry *entry)
{
	struct device *dev = cbdrs->dma_dev;
	struct sgit_resp_query *resp;
	u32 resp_len = sizeof(*resp);
	struct ntmp_qd_by_eid *req;
	u32 req_len = sizeof(*req);
	void *tmp = NULL;
	dma_addr_t dma;
	u32 dma_len;
	int err;

	if (!entry || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	dma_len = max_t(u32, req_len, resp_len);
	tmp = ntmp_alloc_data_mem(dev, dma_len, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	req->crd.tbl_ver = cbdrs->tbl.sgit_ver;
	req->entry_id = cpu_to_le32(entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_SGIT_ID,
				     NTMP_REQ_RESP_LEN(req_len, resp_len),
				     req, &dma, true);
	if (err)
		goto end;

	resp = (struct sgit_resp_query *)req;
	entry->sgise = resp->sgise;
	entry->cfge = resp->cfge;
	entry->icfge = resp->icfge;
	entry->acfge = resp->acfge;

end:
	ntmp_free_data_mem(dev, dma_len, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_sgit_query_entry);

int ntmp_sgit_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return ntmp_delete_entry_by_id(cbdrs, NTMP_SGIT_ID, cbdrs->tbl.sgit_ver,
				       entry_id, 0, 0);
}
EXPORT_SYMBOL_GPL(ntmp_sgit_delete_entry);

int ntmp_isct_operate_entry(struct netc_cbdrs *cbdrs, u32 entry_id, int cmd,
			    struct isct_stse_data *stse)
{
	struct device *dev = cbdrs->dma_dev;
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
	case NTMP_CMD_QD:
	case NTMP_CMD_QU:
		if (!stse)
			return -EINVAL;
		fallthrough;
	case NTMP_CMD_DELETE:
	case NTMP_CMD_UPDATE:
	case NTMP_CMD_ADD:
	break;
	default:
		return -EINVAL;
	}

	query = !!(cmd & NTMP_CMD_QUERY);
	data_size = query ? sizeof(*resp) : sizeof(*req);

	tmp = ntmp_alloc_data_mem(dev, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	if (cmd & NTMP_CMD_UPDATE)
		req->crd.update_act = cpu_to_le16(NTMP_GEN_UA_CFGEU);

	req->crd.tbl_ver = cbdrs->tbl.isct_ver;
	req->entry_id = cpu_to_le32(entry_id);

	/* Request header */
	len = NTMP_REQ_RESP_LEN(sizeof(*req), query ? sizeof(*resp) : 0);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_ISCT_ID,
				cmd, NTMP_AM_ENTRY_ID);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err) {
		dev_err(dev, "Operate SGIT entry (%d) failed (%d)!",
			cmd, err);
		goto end;
	}

	if (query) {
		resp = (struct isct_resp_query *)req;
		if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
			dev_err(dev, "ISCT Query EID:0x%0x, Response EID:0x%x\n",
				entry_id, le32_to_cpu(resp->entry_id));
			err = -EIO;
			goto end;
		}

		*stse = resp->stse;
	}

end:
	ntmp_free_data_mem(dev, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_isct_operate_entry);

int ntmp_ipft_add_entry(struct netc_cbdrs *cbdrs, u32 *entry_id,
			struct ntmp_ipft_entry *entry)
{
	struct device *dev = cbdrs->dma_dev;
	struct ipft_resp_query *resp;
	struct ipft_req_add *req;
	union netc_cbd cbd;
	u32 data_size, len;
	dma_addr_t dma;
	void *tmp;
	int err;

	if (!entry)
		return -EINVAL;

	data_size = sizeof(*resp);
	tmp = ntmp_alloc_data_mem(dev, data_size, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	/* Fill up NTMP request data buffer */
	req->crd.update_act = cpu_to_le16(NTMP_GEN_UA_CFGEU | NTMP_GEN_UA_STSEU);
	req->crd.query_act = NTMP_QUERY_ACT_ENTRY_ID;
	req->crd.tbl_ver = cbdrs->tbl.ipft_ver;
	req->keye = entry->keye;
	req->cfge = entry->cfge;

	/* Request header */
	len = NTMP_REQ_RESP_LEN(sizeof(*req), data_size);
	ntmp_fill_request_headr(&cbd, dma, len, NTMP_IPFT_ID,
				NTMP_CMD_AQ, NTMP_AM_TERNARY_KEY);

	err = netc_xmit_ntmp_cmd(cbdrs, &cbd);
	if (err) {
		dev_err(dev, "Add IPFT entry failed (%d)!", err);
		goto end;
	}

	resp = (struct ipft_resp_query *)req;
	if (entry_id)
		*entry_id = le32_to_cpu(resp->entry_id);

end:
	ntmp_free_data_mem(dev, data_size, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ipft_add_entry);

int ntmp_ipft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  bool update, struct ntmp_ipft_entry *entry)
{
	struct device *dev = cbdrs->dma_dev;
	struct ipft_resp_query *resp;
	u32 resp_len = sizeof(*resp);
	struct ipft_req_qd *req;
	u32 req_len, dma_len;
	void *tmp = NULL;
	dma_addr_t dma;
	int err;

	if (!entry || entry_id == NTMP_NULL_ENTRY_ID)
		return -EINVAL;

	req_len = sizeof(*req);
	/* CFGE_DATA is present when performing an update command,
	 * but we don't need to set this filed because only STSEU
	 * is updated here.
	 */
	if (update)
		req_len += sizeof(struct ipft_cfge_data);

	dma_len = max_t(u32, req_len, resp_len);
	tmp = ntmp_alloc_data_mem(dev, dma_len, &dma, (void **)&req);
	if (!tmp)
		return -ENOMEM;

	if (update)
		req->crd.update_act = cpu_to_le16(NTMP_GEN_UA_STSEU);

	req->crd.tbl_ver = cbdrs->tbl.ipft_ver;
	req->entry_id = cpu_to_le32(entry_id);
	err = ntmp_query_entry_by_id(cbdrs, NTMP_IPFT_ID,
				     NTMP_REQ_RESP_LEN(req_len, resp_len),
				     (struct ntmp_qd_by_eid *)req, &dma, false);

	resp = (struct ipft_resp_query *)req;
	if (unlikely(le32_to_cpu(resp->entry_id) != entry_id)) {
		dev_err(dev, "IPFT Query EID:0x%0x, Response EID:0x%x\n",
			entry_id, le32_to_cpu(resp->entry_id));
		err = -EIO;
		goto end;
	}

	entry->keye = resp->keye;
	entry->match_count = resp->match_count;
	entry->cfge = resp->cfge;

end:
	ntmp_free_data_mem(dev, dma_len, tmp, dma);

	return err;
}
EXPORT_SYMBOL_GPL(ntmp_ipft_query_entry);

int ntmp_ipft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	u32 resp_len = sizeof(struct common_resp_nq);
	u32 req_len = sizeof(struct ipft_req_qd);

	return ntmp_delete_entry_by_id(cbdrs, NTMP_IPFT_ID, cbdrs->tbl.ipft_ver,
				       entry_id, req_len, resp_len);
}
EXPORT_SYMBOL_GPL(ntmp_ipft_delete_entry);

MODULE_DESCRIPTION("NXP NETC Library");
MODULE_LICENSE("Dual BSD/GPL");
