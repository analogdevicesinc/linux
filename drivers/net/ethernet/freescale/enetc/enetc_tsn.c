// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2017-2023 NXP */

#include "enetc.h"

#include <net/tsn.h>

static int alloc_cbdr(struct enetc_si *si, struct enetc_cbd **curr_cbd)
{
	struct enetc_cbdr *ring = &si->cbd_ring;
	int i;

	i = ring->next_to_use;
	*curr_cbd = ENETC_CBD(*ring, i);

	memset(*curr_cbd, 0, sizeof(struct enetc_cbd));
	return i;
}

/* Transmit the BD control ring by writing the pir register.
 * Update the counters maintained by software.
 */
static int xmit_cbdr(struct enetc_si *si, int i)
{
	struct enetc_cbdr *ring = &si->cbd_ring;
	struct enetc_cbd *dest_cbd;
	int nc, timeout;

	i = (i + 1) % ring->bd_count;

	ring->next_to_use = i;
	/* let H/W know BD ring has been updated */
	enetc_wr_reg(ring->pir, i);

	timeout = ENETC_CBDR_TIMEOUT;

	do {
		if (enetc_rd_reg(ring->cir) == i)
			break;
		usleep_range(10, 20);
		timeout -= 10;
	} while (timeout);

	if (!timeout)
		return -EBUSY;

	nc = ring->next_to_clean;

	while (enetc_rd_reg(ring->cir) != nc) {
		dest_cbd = ENETC_CBD(*ring, nc);
		if (dest_cbd->status_flags & ENETC_CBD_STATUS_MASK)
			WARN_ON(1);

		nc = (nc + 1) % ring->bd_count;
	}

	ring->next_to_clean = nc;

	return 0;
}

static inline u64 get_current_time(struct enetc_si *si)
{
	u64 tmp = 0;

	tmp = (u64)enetc_rd(&si->hw, ENETC_SICTR0);
	return ((u64)enetc_rd(&si->hw, ENETC_SICTR1) << 32) + tmp;
}

static u16 enetc_get_max_gcl_len(struct enetc_hw *hw)
{
	return enetc_rd(hw, ENETC_PTGCAPR) & ENETC_PTGCAPR_MAX_GCL_LEN_MASK;
}

static int enetc_qbv_set(struct net_device *ndev,
			 struct tsn_qbv_conf *admin_conf)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	return tsn_qbv_tc_taprio_compat_set(ndev, admin_conf,
					    priv->active_offloads & ENETC_F_QBV);
}

static int enetc_qbv_get(struct net_device *ndev,
			 struct tsn_qbv_conf *admin_conf)
{
	struct tsn_qbv_basic *admin_basic = &admin_conf->admin;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_hw *hw = &priv->si->hw;
	u16 data_size, admin_len, maxlen;
	struct tgs_gcl_query *gcl_query;
	struct tgs_gcl_resp *gcl_data;
	struct enetc_cbd *cbdr;
	__le16 dma_size;
	struct gce *gce;
	int curr_cbd, i;
	dma_addr_t dma;
	u64 temp;

	if (enetc_rd(hw, ENETC_PTGCR) & ENETC_PTGCR_TGE) {
		admin_conf->gate_enabled = true;
	} else {
		admin_conf->gate_enabled = false;
		return 0;
	}

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	gcl_query =  &cbdr->gcl_query;

	maxlen = enetc_get_max_gcl_len(hw);

	data_size = sizeof(struct tgs_gcl_resp) +
		    sizeof(struct gce) * 2 * maxlen;

	gcl_data = kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (!gcl_data)
		return -ENOMEM;

	gce = (struct gce *)(gcl_data + 1);

	gcl_query->acl_len = cpu_to_le16(maxlen);

	dma_size = cpu_to_le16(data_size);
	cbdr->length = dma_size;
	cbdr->status_flags = 0;

	dma = dma_map_single(&priv->si->pdev->dev, gcl_data,
			     data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(priv->si->ndev, "DMA mapping failed!\n");
		kfree(gcl_data);
		return -ENOMEM;
	}

	cbdr->addr[0] = cpu_to_le32(lower_32_bits(dma));
	cbdr->addr[1] = cpu_to_le32(upper_32_bits(dma));
	cbdr->cmd = 1;
	cbdr->cls = BDCR_CMD_PORT_GCL;
	xmit_cbdr(priv->si, curr_cbd);
	dma_unmap_single(&priv->si->pdev->dev, dma, data_size, DMA_FROM_DEVICE);

	/* since cbdr already passed to free, below could be get wrong */
	admin_len = le16_to_cpu(gcl_query->admin_list_len);

	admin_basic->control_list_length = admin_len;

	temp = ((u64)le32_to_cpu(gcl_data->abth)) << 32;
	admin_basic->base_time = le32_to_cpu(gcl_data->abtl) + temp;

	admin_basic->cycle_time = le32_to_cpu(gcl_data->act);
	admin_basic->cycle_time_extension = le32_to_cpu(gcl_data->acte);

	admin_basic->control_list = kcalloc(admin_len,
					    sizeof(*admin_basic->control_list),
					    GFP_KERNEL);
	if (!admin_basic->control_list) {
		memset(cbdr, 0, sizeof(*cbdr));
		kfree(gcl_data);
		return -ENOMEM;
	}

	for (i = 0; i < admin_len; i++) {
		struct gce *temp_gce = gce + i;
		struct tsn_qbv_entry *temp_entry;

		temp_entry = admin_basic->control_list + i;

		temp_entry->gate_state = temp_gce->gate;
		temp_entry->time_interval = le32_to_cpu(temp_gce->period);
	}

	/* Updated by ENETC on completion of the configuration
	 * command. A zero value indicates success.
	 */
	admin_conf->config_change = true;

	memset(cbdr, 0, sizeof(*cbdr));
	kfree(gcl_data);

	return 0;
}

static int enetc_qbv_get_status(struct net_device *ndev,
				struct tsn_qbv_status *status)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_hw *hw = &priv->si->hw;
	struct tsn_qbv_basic *oper_basic;
	u16 data_size, oper_len, maxlen;
	struct tgs_gcl_query *gcl_query;
	struct tgs_gcl_resp *gcl_data;
	struct enetc_cbd *cbdr;
	struct gce *gce;
	int curr_cbd, i;
	__le16 dma_size;
	dma_addr_t dma;
	u64 temp;

	if (!status)
		return -EINVAL;

	oper_basic = &status->oper;

	if (!(enetc_rd(hw, ENETC_PTGCR) & ENETC_PTGCR_TGE))
		return -EINVAL;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	gcl_query = &cbdr->gcl_query;

	maxlen = enetc_get_max_gcl_len(hw);

	data_size = sizeof(struct tgs_gcl_resp) +
		    sizeof(struct gce) * 2 * maxlen;

	gcl_data = kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (!gcl_data)
		return -ENOMEM;

	gce = (struct gce *)(gcl_data + 1);

	gcl_query->acl_len = cpu_to_le16(maxlen);
	gcl_query->ocl_len = cpu_to_le16(maxlen);

	dma_size = cpu_to_le16(data_size);
	cbdr->length = dma_size;
	cbdr->status_flags = 0; /* long format command no ie */

	dma = dma_map_single(&priv->si->pdev->dev, gcl_data,
			     data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(ndev, "DMA mapping failed!\n");
		kfree(gcl_data);
		return -ENOMEM;
	}

	cbdr->addr[0] = cpu_to_le32(lower_32_bits(dma));
	cbdr->addr[1] = cpu_to_le32(upper_32_bits(dma));
	cbdr->cmd = 1;
	cbdr->cls = BDCR_CMD_PORT_GCL;
	xmit_cbdr(priv->si, curr_cbd);
	dma_unmap_single(&priv->si->pdev->dev, dma, data_size, DMA_FROM_DEVICE);

	/* since cbdr already passed to free, below could be get wrong */
	oper_len = le16_to_cpu(gcl_query->oper_list_len);

	if (enetc_rd(hw, ENETC_QBV_PTGAGLSR_OFFSET) & ENETC_QBV_CFG_PEND_MASK) {
		status->config_pending = true;
		goto exit;
	}

	/* The Oper and Admin timing fields exist in the response buffer even
	 * if no valid corresponding lists exists. These fields are considered
	 * invalid if the corresponding list does not exist.
	 */
	status->config_pending = false;
	temp = ((u64)le32_to_cpu(gcl_data->ccth)) << 32;
	status->config_change_time = le32_to_cpu(gcl_data->cctl) + temp;

	temp = ((u64)le32_to_cpu(gcl_data->cceh)) << 32;
	status->config_change_error = le32_to_cpu(gcl_data->ccel) + temp;

	/* changed to SITGTGR */
	status->tick_granularity = enetc_rd(&priv->si->hw, ENETC_SITGTGR);

	/* current time */
	status->current_time = get_current_time(priv->si);

	status->supported_list_max = maxlen;

	/* status->oper.gate_states , no init oper/admin gate state */
	status->oper.control_list_length = oper_len;
	temp = ((u64)le32_to_cpu(gcl_data->obth)) << 32;
	status->oper.base_time = le32_to_cpu(gcl_data->obtl) + temp;
	status->oper.cycle_time = le32_to_cpu(gcl_data->oct);
	status->oper.cycle_time_extension = le32_to_cpu(gcl_data->octe);

	oper_basic->control_list =
		kcalloc(oper_len, sizeof(*oper_basic->control_list), GFP_KERNEL);
	if (!oper_basic->control_list) {
		memset(cbdr, 0, sizeof(*cbdr));
		kfree(gcl_data);
		return -ENOMEM;
	}

	for (i = 0; i < oper_len; i++) {
		struct gce *temp_gce = gce + maxlen + i;
		struct tsn_qbv_entry *temp_entry = oper_basic->control_list + i;

		temp_entry->gate_state = temp_gce->gate;
		temp_entry->time_interval = le32_to_cpu(temp_gce->period);
	}

exit:
	memset(cbdr, 0, sizeof(*cbdr));
	kfree(gcl_data);
	return 0;
}

/* CBD Class 7: Stream Identity Entry Set Descriptor - Long Format */
static int enetc_cb_streamid_set(struct net_device *ndev, u32 index,
				 bool en, struct tsn_cb_streamid *streamid)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct null_streamid_data *si_data1;
	struct smac_streamid_data *si_data2;
	struct streamid_conf *si_conf;
	struct enetc_cbd *cbdr;
	__le16 dma_size;
	dma_addr_t dma;
	void *si_data;
	u16 data_size;
	int curr_cbd;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16((u16)index);
	cbdr->cmd = 0;
	cbdr->cls = BDCR_CMD_STREAM_IDENTIFY;
	cbdr->status_flags = 0;

	data_size = sizeof(struct null_streamid_data);
	si_data = kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	cbdr->length = cpu_to_le16(data_size);

	dma = dma_map_single(&priv->si->pdev->dev, si_data,
			     data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(ndev, "DMA mapping failed!\n");
		kfree(si_data);
		return -ENOMEM;
	}

	cbdr->addr[0] = cpu_to_le32(lower_32_bits(dma));
	cbdr->addr[1] = cpu_to_le32(upper_32_bits(dma));
	si_data1 = (struct null_streamid_data *)si_data;
	si_data1->dmac[0] = 0xFF;
	si_data1->dmac[1] = 0xFF;
	si_data1->dmac[2] = 0xFF;
	si_data1->dmac[3] = 0xFF;
	si_data1->dmac[4] = 0xFF;
	si_data1->dmac[5] = 0xFF;
	si_data1->vid_vidm_tg =
		cpu_to_le16(ENETC_CBDR_SID_VID_MASK
			    + ((0x3 << 14) | ENETC_CBDR_SID_VIDM));

	si_conf = &cbdr->sid_set;
	/* Only one port supported for one entry, set itself */
	si_conf->iports = cpu_to_le32(1 << (priv->si->pdev->devfn & 0x7));
	si_conf->id_type = 1;
	si_conf->oui[2] = 0x0;
	si_conf->oui[1] = 0x80;
	si_conf->oui[0] = 0xC2;

	xmit_cbdr(priv->si, curr_cbd);

	memset(cbdr, 0, sizeof(*cbdr));
	kfree(si_data);

	if (!en)
		return 0;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16((u16)index);
	cbdr->cmd = 0;
	cbdr->cls = BDCR_CMD_STREAM_IDENTIFY;
	cbdr->status_flags = 0;

	si_conf = &cbdr->sid_set;
	si_conf->en = 0x80;
	si_conf->stream_handle = cpu_to_le32(streamid->handle);
	si_conf->iports = cpu_to_le32(1 << (priv->si->pdev->devfn & 0x7));
	si_conf->id_type = streamid->type;
	si_conf->oui[2] = 0x0;
	si_conf->oui[1] = 0x80;
	si_conf->oui[0] = 0xC2;

	if (si_conf->id_type == 1) {
		data_size = sizeof(struct null_streamid_data);
		si_data = kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	} else if (si_conf->id_type == 2) {
		data_size = sizeof(struct smac_streamid_data);
		si_data = kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	} else {
		return -EINVAL;
	}

	if (!si_data)
		return -ENOMEM;

	dma_size = cpu_to_le16(data_size);
	cbdr->length = dma_size;
	cbdr->status_flags = 0;

	dma = dma_map_single(&priv->si->pdev->dev, si_data,
			     data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(ndev, "DMA mapping failed!\n");
		memset(cbdr, 0, sizeof(*cbdr));
		kfree(si_data);
		return -ENOMEM;
	}
	cbdr->addr[0] = cpu_to_le32(lower_32_bits(dma));
	cbdr->addr[1] = cpu_to_le32(upper_32_bits(dma));

	/* VIDM default to be 1.
	 * VID Match. If set (b1) then the VID must match, otherwise
	 * any VID is considered a match. VIDM setting is only used
	 * when TG is set to b01.
	 */
	if (si_conf->id_type == 1) {
		si_data1 = (struct null_streamid_data *)si_data;
		si_data1->dmac[0] = streamid->para.nid.dmac & 0xFF;
		si_data1->dmac[1] = (streamid->para.nid.dmac >> 8) & 0xFF;
		si_data1->dmac[2] = (streamid->para.nid.dmac >> 16) & 0xFF;
		si_data1->dmac[3] = (streamid->para.nid.dmac >> 24) & 0xFF;
		si_data1->dmac[4] = (streamid->para.nid.dmac >> 32) & 0xFF;
		si_data1->dmac[5] = (streamid->para.nid.dmac >> 40) & 0xFF;
		si_data1->vid_vidm_tg =
		cpu_to_le16((streamid->para.nid.vid & ENETC_CBDR_SID_VID_MASK) +
			    ((((u16)(streamid->para.nid.tagged) & 0x3) << 14)
			     | ENETC_CBDR_SID_VIDM));
	} else if (si_conf->id_type == 2) {
		si_data2 = (struct smac_streamid_data *)si_data;
		si_data2->smac[0] = streamid->para.sid.smac & 0xFF;
		si_data2->smac[1] = (streamid->para.sid.smac >> 8) & 0xFF;
		si_data2->smac[2] = (streamid->para.sid.smac >> 16) & 0xFF;
		si_data2->smac[3] = (streamid->para.sid.smac >> 24) & 0xFF;
		si_data2->smac[4] = (streamid->para.sid.smac >> 32) & 0xFF;
		si_data2->smac[5] = (streamid->para.sid.smac >> 40) & 0xFF;
		si_data2->vid_vidm_tg =
		cpu_to_le16((streamid->para.sid.vid & ENETC_CBDR_SID_VID_MASK) +
			    ((((u16)(streamid->para.sid.tagged) & 0x3) << 14)
			     | ENETC_CBDR_SID_VIDM));
	}

	xmit_cbdr(priv->si, curr_cbd);

	memset(cbdr, 0, sizeof(*cbdr));
	kfree(si_data);

	return 0;
}

/* CBD Class 7: Stream Identity Entry Query Descriptor - Long Format */
static int enetc_cb_streamid_get(struct net_device *ndev, u32 index,
				 struct tsn_cb_streamid *streamid)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct streamid_query_resp *si_data;
	struct enetc_cbd *cbdr;
	int curr_cbd, valid;
	__le16 dma_size;
	dma_addr_t dma;
	u16 data_size;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 1;
	cbdr->cls = BDCR_CMD_STREAM_IDENTIFY;
	cbdr->status_flags = 0;

	data_size = sizeof(struct streamid_query_resp);
	si_data = kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (!si_data)
		return -ENOMEM;

	dma_size = cpu_to_le16(data_size);
	cbdr->length = dma_size;
	cbdr->status_flags = 0; /* long format command no ie */

	dma = dma_map_single(&priv->si->pdev->dev, si_data,
			     data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(ndev, "DMA mapping failed!\n");
		kfree(si_data);
		return -ENOMEM;
	}
	cbdr->addr[0] = cpu_to_le32(lower_32_bits(dma));
	cbdr->addr[1] = cpu_to_le32(upper_32_bits(dma));

	xmit_cbdr(priv->si, curr_cbd);

	streamid->type = si_data->id_type;

	if (streamid->type == 1) {
		streamid->para.nid.dmac = si_data->mac[0]
			+ ((u64)si_data->mac[1] << 8)
			+ ((u64)si_data->mac[2] << 16)
			+ ((u64)si_data->mac[3] << 24)
			+ ((u64)si_data->mac[4] << 32)
			+ ((u64)si_data->mac[5] << 40);
		/* VID Match. If set (b1) then the VID must match, otherwise
		 * any VID is considered a match.
		 */
		streamid->para.nid.vid = le16_to_cpu(si_data->vid_vidm_tg) &
					 ENETC_CBDR_SID_VID_MASK;
		streamid->para.nid.tagged =
				le16_to_cpu(si_data->vid_vidm_tg) >> 14 & 0x3;
	} else if (streamid->type == 2) {
		streamid->para.sid.smac = si_data->mac[0]
			+ ((u64)si_data->mac[1] << 8)
			+ ((u64)si_data->mac[2] << 16)
			+ ((u64)si_data->mac[3] << 24)
			+ ((u64)si_data->mac[4] << 32)
			+ ((u64)si_data->mac[5] << 40);
		/* VID Match. If set (b1) then the VID must match, otherwise
		 * any VID is considered a match.
		 */
		streamid->para.sid.vid = le16_to_cpu(si_data->vid_vidm_tg) &
					 ENETC_CBDR_SID_VID_MASK;
		streamid->para.sid.tagged =
			le16_to_cpu(si_data->vid_vidm_tg) >> 14 & 0x3;
	}

	streamid->handle = le32_to_cpu(si_data->stream_handle);
	streamid->ifac_iport = le32_to_cpu(si_data->input_ports);
	valid = si_data->en ? 1 : 0;

	memset(cbdr, 0, sizeof(*cbdr));
	kfree(si_data);

	return valid;
}

/*  CBD Class 7: Stream Identity Statistics Query Descriptor - Long Format */
static int enetc_cb_streamid_counters_get(struct net_device *ndev, u32 index,
					  struct tsn_cb_streamid_counters *counters)
{
	return 0;
}

static void enetc_qci_enable(struct enetc_hw *hw)
{
	enetc_wr(hw, ENETC_PPSFPMR, enetc_rd(hw, ENETC_PPSFPMR)
		 | ENETC_PPSFPMR_PSFPEN | ENETC_PPSFPMR_VS
		 | ENETC_PPSFPMR_PVC | ENETC_PPSFPMR_PVZC);
}

static void enetc_qci_disable(struct enetc_hw *hw)
{
	enetc_wr(hw, ENETC_PPSFPMR, enetc_rd(hw, ENETC_PPSFPMR)
		 & ~ENETC_PPSFPMR_PSFPEN & ~ENETC_PPSFPMR_VS
		 & ~ENETC_PPSFPMR_PVC & ~ENETC_PPSFPMR_PVZC);
}

/* CBD Class 8: Stream Filter Instance Set Descriptor - Short Format */
static int enetc_qci_sfi_set(struct net_device *ndev, u32 index, bool en,
			     struct tsn_qci_psfp_sfi_conf *tsn_qci_sfi)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct sfi_conf *sfi_config;
	struct enetc_cbd *cbdr;
	int curr_cbd;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 0;
	cbdr->cls = BDCR_CMD_STREAM_FILTER;
	cbdr->status_flags = 0x80;
	cbdr->length = cpu_to_le16(1);

	sfi_config = &cbdr->sfi_conf;
	if (en)
		sfi_config->en = 0x80;

	if (tsn_qci_sfi->stream_handle_spec >= 0) {
		sfi_config->stream_handle =
			cpu_to_le32(tsn_qci_sfi->stream_handle_spec);
		sfi_config->sthm |= 0x80;
	}

	sfi_config->sg_inst_table_index =
		cpu_to_le16(tsn_qci_sfi->stream_gate_instance_id);
	sfi_config->input_ports =
		cpu_to_le32(1 << (priv->si->pdev->devfn & 0x7));

	/* The priority value which may be matched against the
	 * frame’s priority value to determine a match for this entry.
	 */
	if (tsn_qci_sfi->priority_spec >= 0)
		sfi_config->multi |= (tsn_qci_sfi->priority_spec & 0x7) | 0x8;

	/* Filter Type. Identifies the contents of the MSDU/FM_INST_INDEX
	 * field as being either an MSDU value or an index into the Flow
	 * Meter Instance table.
	 */
	if (tsn_qci_sfi->stream_filter.maximum_sdu_size != 0) {
		sfi_config->msdu =
		cpu_to_le16(tsn_qci_sfi->stream_filter.maximum_sdu_size);
		sfi_config->multi |= 0x40;
	}

	if (tsn_qci_sfi->stream_filter.flow_meter_instance_id >= 0) {
		sfi_config->fm_inst_table_index =
		cpu_to_le16(tsn_qci_sfi->stream_filter.flow_meter_instance_id);
		sfi_config->multi |= 0x80;
	}

	/* Stream blocked due to oversized frame enable. TRUE or FALSE */
	if (tsn_qci_sfi->block_oversize_enable)
		sfi_config->multi |= 0x20;

	/* Stream blocked due to oversized frame. TRUE or FALSE */
	if (tsn_qci_sfi->block_oversize)
		sfi_config->multi |= 0x10;

	xmit_cbdr(priv->si, curr_cbd);

	memset(cbdr, 0, sizeof(*cbdr));
	return 0;
}

/* CBD Class 8: Stream Filter Instance Query Descriptor - Short Format */
static int enetc_qci_sfi_get(struct net_device *ndev, u32 index,
			     struct tsn_qci_psfp_sfi_conf *tsn_qci_sfi)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct sfi_conf *sfi_config;
	struct enetc_cbd *cbdr;
	int curr_cbd;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 1;
	cbdr->cls = BDCR_CMD_STREAM_FILTER;
	cbdr->status_flags = 0x80;

	xmit_cbdr(priv->si, curr_cbd);

	sfi_config = &cbdr->sfi_conf;
	if (sfi_config->sthm & 0x80)
		tsn_qci_sfi->stream_handle_spec =
			le32_to_cpu(sfi_config->stream_handle);
	else
		tsn_qci_sfi->stream_handle_spec = -1;

	tsn_qci_sfi->stream_gate_instance_id =
		le16_to_cpu(sfi_config->sg_inst_table_index);

	if (sfi_config->multi & 0x8)
		tsn_qci_sfi->priority_spec = sfi_config->multi & 0x7;
	else
		tsn_qci_sfi->priority_spec = -1;

	/* Filter Type. Identifies the contents of the MSDU/FM_INST_INDEX
	 * field as being either an MSDU value or an index into the Flow
	 * Meter Instance table.
	 */
	if (sfi_config->multi & 0x80)
		tsn_qci_sfi->stream_filter.flow_meter_instance_id =
			le16_to_cpu(sfi_config->fm_inst_table_index);
	else
		tsn_qci_sfi->stream_filter.flow_meter_instance_id = -1;

	if (sfi_config->multi & 0x40)
		tsn_qci_sfi->stream_filter.maximum_sdu_size =
			le16_to_cpu(sfi_config->msdu);

	/* Stream blocked due to oversized frame enable. TRUE or FALSE */
	if (sfi_config->multi & 0x20)
		tsn_qci_sfi->block_oversize_enable = true;
	/* Stream blocked due to oversized frame. TRUE or FALSE */
	if (sfi_config->multi & 0x10)
		tsn_qci_sfi->block_oversize = true;

	if (sfi_config->en & 0x80) {
		memset(cbdr, 0, sizeof(*cbdr));
		return 1;
	}

	memset(cbdr, 0, sizeof(*cbdr));
	return 0;
}

/* CBD Class 8: Stream Filter Instance Query Statistics
 * Descriptor - Long Format
 */
static int enetc_qci_sfi_counters_get(struct net_device *ndev, u32 index,
				      struct tsn_qci_psfp_sfi_counters *counters)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct sfi_counter_data *sfi_counter_data;
	struct enetc_cbd *cbdr;
	__le16 dma_size;
	dma_addr_t dma;
	u16 data_size;
	int curr_cbd;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16((u16)index);
	cbdr->cmd = 2;
	cbdr->cls = BDCR_CMD_STREAM_FILTER;
	cbdr->status_flags = 0;

	data_size = sizeof(struct sfi_counter_data);
	sfi_counter_data = kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (!sfi_counter_data)
		return -ENOMEM;

	dma = dma_map_single(&priv->si->pdev->dev, sfi_counter_data,
			     data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(ndev, "DMA mapping failed!\n");
		kfree(sfi_counter_data);
		return -ENOMEM;
	}
	cbdr->addr[0] = cpu_to_le32(lower_32_bits(dma));
	cbdr->addr[1] = cpu_to_le32(upper_32_bits(dma));

	dma_size = cpu_to_le16(data_size);
	cbdr->length = dma_size;

	xmit_cbdr(priv->si, curr_cbd);

	counters->matching_frames_count =
		((u64)le32_to_cpu(sfi_counter_data->matchh) << 32) +
		le32_to_cpu(sfi_counter_data->matchl);

	counters->not_passing_sdu_count =
		((u64)le32_to_cpu(sfi_counter_data->msdu_droph) << 32) +
		le32_to_cpu(sfi_counter_data->msdu_dropl);

	counters->passing_sdu_count = counters->matching_frames_count -
				      counters->not_passing_sdu_count;

	counters->not_passing_frames_count =
		((u64)le32_to_cpu(sfi_counter_data->stream_gate_droph) << 32)
		+ le32_to_cpu(sfi_counter_data->stream_gate_dropl);

	counters->passing_frames_count = counters->matching_frames_count -
					 counters->not_passing_sdu_count -
					 counters->not_passing_frames_count;

	counters->red_frames_count =
		((u64)le32_to_cpu(sfi_counter_data->flow_meter_droph) << 32) +
		le32_to_cpu(sfi_counter_data->flow_meter_dropl);

	memset(cbdr, 0, sizeof(*cbdr));
	return 0;
}

/* CBD Class 9: Stream Gate Instance Table Entry Set
 * Descriptor - Short Format
 */
static int enetc_qci_sgi_set(struct net_device *ndev, u32 index,
			     struct tsn_qci_psfp_sgi_conf *tsn_qci_sgi)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_cbd *cbdr, *cbdr_sgcl;
	struct sgcl_conf *sgcl_config;
	struct sgi_table *sgi_config;
	struct sgcl_data *sgcl_data;
	struct sgce *sgce;
	__le16 dma_size;
	int curr_cbd, i;
	dma_addr_t dma;
	u16 data_size;

	/* disable first */
	curr_cbd = alloc_cbdr(priv->si, &cbdr);
	memset(cbdr, 0, sizeof(*cbdr));

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 0;
	cbdr->cls = BDCR_CMD_STREAM_GCL;
	cbdr->status_flags = 0x80;

	xmit_cbdr(priv->si, curr_cbd);

	if (!tsn_qci_sgi->gate_enabled) {
		memset(cbdr, 0, sizeof(*cbdr));
		return 0;
	}

	/* Re-enable */
	curr_cbd = alloc_cbdr(priv->si, &cbdr);
	memset(cbdr, 0, sizeof(*cbdr));

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 0;
	cbdr->cls = BDCR_CMD_STREAM_GCL;
	cbdr->status_flags = 0x80;

	sgi_config = &cbdr->sgi_table;

	sgi_config->ocgtst = tsn_qci_sgi->admin.control_list_length ?
			0x80 : (tsn_qci_sgi->admin.gate_states ? 0x80 : 0x0);

	sgi_config->oipv =
		tsn_qci_sgi->admin.control_list_length ?
		0x0 : ((tsn_qci_sgi->admin.init_ipv < 0) ?
		       0x0 : ((tsn_qci_sgi->admin.init_ipv & 0x7) | 0x8));

	sgi_config->en = 0x80;

	if (tsn_qci_sgi->block_invalid_rx_enable)
		sgi_config->gset |= 0x80;
	if (tsn_qci_sgi->block_invalid_rx)
		sgi_config->gset |= 0x40;
	if (tsn_qci_sgi->block_octets_exceeded)
		sgi_config->gset |= 0x10;
	if (tsn_qci_sgi->block_octets_exceeded_enable)
		sgi_config->gset |= 0x20;

	xmit_cbdr(priv->si, curr_cbd);

	if (tsn_qci_sgi->admin.control_list_length == 0)
		goto exit;

	curr_cbd = alloc_cbdr(priv->si, &cbdr_sgcl);
	memset(cbdr, 0, sizeof(*cbdr));

	cbdr_sgcl->index = cpu_to_le16(index);
	cbdr_sgcl->cmd = 1;
	cbdr_sgcl->cls = BDCR_CMD_STREAM_GCL;
	cbdr_sgcl->status_flags = 0;

	sgcl_config = &cbdr_sgcl->sgcl_conf;

	/* tsn_qci_sgi->admin.control_list_length is not zero now */
	if (tsn_qci_sgi->admin.control_list_length > 4)
		return -EINVAL;

	sgcl_config->acl_len =
		(tsn_qci_sgi->admin.control_list_length - 1) & 0x3;

	data_size = struct_size(sgcl_data, sgcl,
				tsn_qci_sgi->admin.control_list_length);

	sgcl_data = kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (!sgcl_data)
		return -ENOMEM;

	dma_size = cpu_to_le16(data_size);
	cbdr_sgcl->length = dma_size;

	dma = dma_map_single(&priv->si->pdev->dev,
			     sgcl_data, data_size,
			     DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(ndev, "DMA mapping failed!\n");
		memset(cbdr, 0, sizeof(*cbdr));
		memset(cbdr_sgcl, 0, sizeof(*cbdr_sgcl));
		kfree(sgcl_data);
		return -ENOMEM;
	}
	cbdr_sgcl->addr[0] = cpu_to_le32(lower_32_bits(dma));
	cbdr_sgcl->addr[1] = cpu_to_le32(upper_32_bits(dma));

	sgce = (struct sgce *)(sgcl_data + 1);

	if (tsn_qci_sgi->admin.gate_states)
		sgcl_config->agtst = 0x80;

	sgcl_data->ct = cpu_to_le32(tsn_qci_sgi->admin.cycle_time);
	sgcl_data->cte = cpu_to_le32(tsn_qci_sgi->admin.cycle_time_extension);

	if (tsn_qci_sgi->admin.init_ipv >= 0)
		sgcl_config->aipv = (tsn_qci_sgi->admin.init_ipv & 0x7) | 0x8;

	for (i = 0; i < tsn_qci_sgi->admin.control_list_length; i++) {
		struct tsn_qci_psfp_gcl *temp_sgcl = tsn_qci_sgi->admin.gcl + i;
		struct sgce *temp_entry = (struct sgce *)(sgce + i);

		if (temp_sgcl->gate_state)
			temp_entry->multi |= 0x10;

		if (temp_sgcl->ipv >= 0)
			temp_entry->multi |= ((temp_sgcl->ipv & 0x7) << 5)
						| 0x08;

		if (temp_sgcl->octet_max)
			temp_entry->multi |= 0x01;

		temp_entry->interval = cpu_to_le32(temp_sgcl->time_interval);
		temp_entry->msdu[0] = temp_sgcl->octet_max & 0xFF;
		temp_entry->msdu[1] = (temp_sgcl->octet_max >> 8) & 0xFF;
		temp_entry->msdu[2] = (temp_sgcl->octet_max >> 16) & 0xFF;
	}

	if (!tsn_qci_sgi->admin.base_time) {
		sgcl_data->btl =
			cpu_to_le32(enetc_rd(&priv->si->hw, ENETC_SICTR0));
		sgcl_data->bth =
			cpu_to_le32(enetc_rd(&priv->si->hw, ENETC_SICTR1));
	} else {
		u32 tempu, templ;

		tempu = upper_32_bits(tsn_qci_sgi->admin.base_time);
		templ = lower_32_bits(tsn_qci_sgi->admin.base_time);
		sgcl_data->bth = cpu_to_le32(tempu);
		sgcl_data->btl = cpu_to_le32(templ);
	}

	xmit_cbdr(priv->si, curr_cbd);

	memset(cbdr_sgcl, 0, sizeof(*cbdr_sgcl));
	kfree(sgcl_data);

exit:
	memset(cbdr, 0, sizeof(*cbdr));
	return 0;
}

/* CBD Class 9: Stream Gate Instance Table Entry Query
 * Descriptor - Short Format
 */
static int enetc_qci_sgi_get(struct net_device *ndev, u32 index,
			     struct tsn_qci_psfp_sgi_conf *tsn_qci_sgi)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_cbd *cbdr, *cbdr_sgcl;
	struct sgcl_query_resp *sgcl_data;
	u16 data_size, gcl_data_stat = 0;
	struct sgcl_query *sgcl_query;
	struct sgi_table *sgi_config;
	struct sgce *sgce;
	u8 admin_len = 0;
	__le16 dma_size;
	int curr_cbd, i;
	dma_addr_t dma;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 2;
	cbdr->cls = BDCR_CMD_STREAM_GCL;
	cbdr->status_flags = 0x80;

	xmit_cbdr(priv->si, curr_cbd);

	sgi_config = &cbdr->sgi_table;

	tsn_qci_sgi->admin.gate_states = (sgi_config->ocgtst & 0x80) ?
						true : false;
	if (sgi_config->oipv & 0x08)
		tsn_qci_sgi->admin.init_ipv = sgi_config->oipv & 0x7;
	else
		tsn_qci_sgi->admin.init_ipv = -1;

	if (sgi_config->en & 0x80)
		tsn_qci_sgi->gate_enabled = true;
	if (sgi_config->gset & 0x80)
		tsn_qci_sgi->block_invalid_rx_enable = true;
	if (sgi_config->gset & 0x40)
		tsn_qci_sgi->block_invalid_rx = true;
	if (sgi_config->gset & 0x20)
		tsn_qci_sgi->block_octets_exceeded_enable = true;
	if (sgi_config->gset & 0x10)
		tsn_qci_sgi->block_octets_exceeded = true;

	/* Check gate list length is zero? */
	if (!(sgi_config->oacl_len & 0x30)) {
		tsn_qci_sgi->admin.control_list_length = 0;
		goto exit;
	}

	curr_cbd = alloc_cbdr(priv->si, &cbdr_sgcl);

	cbdr_sgcl->index = cpu_to_le16(index);
	cbdr_sgcl->cmd = 3;
	cbdr_sgcl->cls = BDCR_CMD_STREAM_GCL;
	cbdr_sgcl->status_flags = 0;

	data_size = sizeof(struct sgcl_query_resp) + 4 * sizeof(struct sgce);

	sgcl_data = kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (!sgcl_data)
		return -ENOMEM;

	dma_size = cpu_to_le16(data_size);
	cbdr_sgcl->length = dma_size;
	cbdr_sgcl->status_flags = 0;

	sgcl_query = &cbdr_sgcl->sgcl_query;

	sgcl_query->oacl_len = 0x10;

	dma = dma_map_single(&priv->si->pdev->dev, sgcl_data,
			     data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(ndev, "DMA mapping failed!\n");
		memset(cbdr, 0, sizeof(*cbdr));
		memset(cbdr_sgcl, 0, sizeof(*cbdr_sgcl));
		kfree(sgcl_data);
		return -ENOMEM;
	}
	cbdr_sgcl->addr[0] = cpu_to_le32(lower_32_bits(dma));
	cbdr_sgcl->addr[1] = cpu_to_le32(upper_32_bits(dma));

	xmit_cbdr(priv->si, curr_cbd);

	sgce = (struct sgce *)(sgcl_data + 1);

	gcl_data_stat = le16_to_cpu(sgcl_data->stat);
	if (gcl_data_stat & 0x10)
		tsn_qci_sgi->admin.gate_states = true;

	if (gcl_data_stat & 0x80)
		tsn_qci_sgi->admin.init_ipv = gcl_data_stat & 0x7;
	else
		tsn_qci_sgi->admin.init_ipv = -1;

	/* admin_len can also get from gcl_data_stat bit 5,6
	 * OR sgi_config->oacl_len
	 */
	admin_len = (sgcl_query->oacl_len & 0x3) + 1;
	tsn_qci_sgi->admin.control_list_length = admin_len;
	tsn_qci_sgi->admin.cycle_time = le32_to_cpu(sgcl_data->act);
	tsn_qci_sgi->admin.cycle_time_extension = le32_to_cpu(sgcl_data->acte);
	tsn_qci_sgi->admin.base_time = ((u64)(le32_to_cpu(sgcl_data->abth))
					      << 32)
					+ le32_to_cpu(sgcl_data->abtl);

	tsn_qci_sgi->admin.gcl = kcalloc(admin_len,
					 sizeof(struct tsn_qci_psfp_gcl),
					 GFP_KERNEL);
	if (!tsn_qci_sgi->admin.gcl) {
		kfree(sgcl_data);
		return -ENOMEM;
	}

	for (i = 0; i < admin_len; i++) {
		struct tsn_qci_psfp_gcl *temp_sgcl = tsn_qci_sgi->admin.gcl + i;
		struct sgce *temp_entry = (struct sgce *)(sgce + i);

		if (temp_entry->multi & 0x10)
			temp_sgcl->gate_state = true;

		if (temp_entry->multi & 0x08)
			temp_sgcl->ipv = temp_entry->multi >> 5;
		else
			temp_sgcl->ipv = -1;

		temp_sgcl->time_interval = le32_to_cpu(temp_entry->interval);

		if (temp_entry->multi & 0x01)
			temp_sgcl->octet_max = (temp_entry->msdu[0] & 0xff)
				| (((u32)temp_entry->msdu[1] << 8) & 0xff00)
				| (((u32)temp_entry->msdu[1] << 16) & 0xff0000);
		else
			temp_sgcl->octet_max = 0;
	}

	memset(cbdr_sgcl, 0, sizeof(*cbdr_sgcl));
	kfree(sgcl_data);

exit:
	memset(cbdr, 0, sizeof(*cbdr));
	return 0;
}

/* CBD Class 9: Stream Gate Instance Table Entry Query Descriptor
 * CBD Class 9: Stream Gate Control List Query Descriptor
 */
static int enetc_qci_sgi_status_get(struct net_device *ndev, u16 index,
				    struct tsn_psfp_sgi_status *status)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_cbd *cbdr_sgi, *cbdr_sgcl;
	struct sgcl_query_resp *sgcl_data;
	u16 data_size, gcl_data_stat = 0;
	struct sgcl_query *sgcl_query;
	struct sgi_table *sgi_config;
	struct sgce *sgce;
	__le16 dma_size;
	u8 oper_len = 0;
	int curr_cbd, i;
	dma_addr_t dma;

	curr_cbd = alloc_cbdr(priv->si, &cbdr_sgi);

	cbdr_sgi->index = cpu_to_le16(index);
	cbdr_sgi->cmd = 2;
	cbdr_sgi->cls = BDCR_CMD_STREAM_GCL;
	cbdr_sgi->status_flags = 0x80;

	sgi_config = &cbdr_sgi->sgi_table;

	if (sgi_config->gset & 0x4)
		status->config_pending = true;

	status->oper.gate_states = ((sgi_config->ocgtst & 0x80) ? true : false);

	/* Check gate list length is zero */
	if (!(sgi_config->oacl_len & 0x30)) {
		status->oper.control_list_length = 0;
		goto cmd2quit;
	}

	xmit_cbdr(priv->si, curr_cbd);

	curr_cbd = alloc_cbdr(priv->si, &cbdr_sgcl);

	cbdr_sgcl->index = cpu_to_le16(index);
	cbdr_sgcl->cmd = 3;
	cbdr_sgcl->cls = BDCR_CMD_STREAM_GCL;
	cbdr_sgcl->status_flags = 0;

	/* Max size */
	data_size = sizeof(struct sgcl_query_resp) + 4 * sizeof(struct sgce);

	sgcl_data = kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (!sgcl_data)
		return -ENOMEM;

	dma_size = cpu_to_le16(data_size);
	cbdr_sgcl->length = dma_size;
	cbdr_sgcl->status_flags = 0;

	sgcl_query = &cbdr_sgcl->sgcl_query;

	sgcl_query->oacl_len = 0x20;

	dma = dma_map_single(&priv->si->pdev->dev, sgcl_data,
			     data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(ndev, "DMA mapping failed!\n");
		memset(cbdr_sgi, 0, sizeof(*cbdr_sgi));
		memset(cbdr_sgcl, 0, sizeof(*cbdr_sgcl));
		kfree(sgcl_data);
		return -ENOMEM;
	}
	cbdr_sgcl->addr[0] = cpu_to_le32(lower_32_bits(dma));
	cbdr_sgcl->addr[1] = cpu_to_le32(upper_32_bits(dma));

	xmit_cbdr(priv->si, curr_cbd);

	sgce = (struct sgce *)(sgcl_data + 1);

	/* oper_len can also get from gcl_data_stat bit 5,6
	 * OR sgi_config->oacl_len
	 */
	oper_len = ((sgcl_query->oacl_len & 0x0c) >> 2) + 1;

	/* Get Stream Gate Control List */
	status->oper.cycle_time = le32_to_cpu(sgcl_data->oct);
	status->oper.cycle_time_extension = le32_to_cpu(sgcl_data->octe);
	status->oper.base_time = le32_to_cpu(sgcl_data->obtl)
				+ ((u64)le32_to_cpu(sgcl_data->obth) << 32);
	status->oper.control_list_length = oper_len;

	gcl_data_stat = le16_to_cpu(sgcl_data->stat);
	if (gcl_data_stat & 0x400)
		status->oper.init_ipv = gcl_data_stat & 0x38 >> 7;
	else
		status->oper.init_ipv = -1;

	if (gcl_data_stat & 0x800)
		status->oper.gate_states = true;

	status->oper.gcl = kcalloc(oper_len,
				   sizeof(struct tsn_qci_psfp_gcl),
				   GFP_KERNEL);
	if (!status->oper.gcl) {
		memset(cbdr_sgi, 0, sizeof(*cbdr_sgi));
		memset(cbdr_sgcl, 0, sizeof(*cbdr_sgcl));
		kfree(sgcl_data);
		return -ENOMEM;
	}

	for (i = 0; i < oper_len; i++) {
		struct tsn_qci_psfp_gcl *temp_sgcl = status->oper.gcl + i;
		struct sgce *temp_entry = (struct sgce *)(sgce + i);

		if (temp_entry->multi & 0x10)
			temp_sgcl->gate_state = true;

		if (temp_entry->multi & 0x08)
			temp_sgcl->ipv = temp_entry->multi >> 5;
		else
			temp_sgcl->ipv = -1;

		temp_sgcl->time_interval = le32_to_cpu(temp_entry->interval);

		if (temp_entry->multi & 0x01)
			temp_sgcl->octet_max = temp_entry->msdu[0]
					| ((((u32)temp_entry->msdu[1]) << 8)
					   & 0xff00)
					| ((((u32)temp_entry->msdu[2]) << 16)
					   & 0xff0000);
		else
			temp_sgcl->octet_max = 0;
	}

	status->config_change_time = le32_to_cpu(sgcl_data->cctl)
				+ ((u64)le32_to_cpu(sgcl_data->ccth) << 32);

	memset(cbdr_sgcl, 0, sizeof(*cbdr_sgcl));
	kfree(sgcl_data);

cmd2quit:
	/* changed to SITGTGR */
	status->tick_granularity = enetc_rd(&priv->si->hw, ENETC_SITGTGR);

	/* current time */
	status->current_time = get_current_time(priv->si);

	memset(cbdr_sgi, 0, sizeof(*cbdr_sgi));

	return 0;
}

/* CBD Class 10: Flow Meter Instance Set Descriptor - Short Format */
static int enetc_qci_fmi_set(struct net_device *ndev, u32 index, bool enable,
			     struct tsn_qci_psfp_fmi *tsn_qci_fmi)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct fmi_conf *fmi_config;
	struct enetc_cbd *cbdr;
	int curr_cbd;
	u64 temp = 0;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16((u16)index);
	cbdr->cmd = 0;
	cbdr->cls = BDCR_CMD_FLOW_METER;
	cbdr->status_flags = 0x80;

	xmit_cbdr(priv->si, curr_cbd);

	if (!enable) {
		memset(cbdr, 0, sizeof(*cbdr));
		return 0;
	}

	/* Re-enable */
	curr_cbd = alloc_cbdr(priv->si, &cbdr);
	memset(cbdr, 0, sizeof(*cbdr));
	cbdr->index = cpu_to_le16((u16)index);
	cbdr->cmd = 0;
	cbdr->cls = BDCR_CMD_FLOW_METER;
	cbdr->status_flags = 0x80;

	fmi_config = &cbdr->fmi_conf;
	fmi_config->en = 0x80;
	if (tsn_qci_fmi->cir) {
		temp = (u64)1000000 * tsn_qci_fmi->cir;
		temp = temp / 3725;
	}
	fmi_config->cir = cpu_to_le32((u32)temp);
	fmi_config->cbs = cpu_to_le32(tsn_qci_fmi->cbs);
	temp = 0;
	if (tsn_qci_fmi->eir) {
		temp = (u64)1000000 * tsn_qci_fmi->eir;
		temp = temp / 3725;
	}
	fmi_config->eir = cpu_to_le32((u32)temp);
	fmi_config->ebs = cpu_to_le32(tsn_qci_fmi->ebs);

	if (tsn_qci_fmi->mark_red)
		fmi_config->conf |= 0x1;

	if (tsn_qci_fmi->mark_red_enable)
		fmi_config->conf |= 0x2;

	if (tsn_qci_fmi->drop_on_yellow)
		fmi_config->conf |= 0x4;

	if (tsn_qci_fmi->cm)
		fmi_config->conf |= 0x8;

	if (tsn_qci_fmi->cf)
		fmi_config->conf |= 0x10;

	xmit_cbdr(priv->si, curr_cbd);

	memset(cbdr, 0, sizeof(*cbdr));
	return 0;
}

/* CBD Class 10: Flow Meter Instance Query Descriptor - Short Format */
static int enetc_qci_fmi_get(struct net_device *ndev, u32 index,
			     struct tsn_qci_psfp_fmi *tsn_qci_fmi,
			     struct tsn_qci_psfp_fmi_counters *counters)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct fmi_query_stat_resp *fmi_counter_data;
	struct fmi_conf *fmi_config;
	struct enetc_cbd *cbdr;
	__le16 dma_size;
	dma_addr_t dma;
	u16 data_size;
	int curr_cbd;
	u64 temp = 0;

	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 1;
	cbdr->cls = BDCR_CMD_FLOW_METER;
	cbdr->status_flags = 0x80;

	xmit_cbdr(priv->si, curr_cbd);

	fmi_config = &cbdr->fmi_conf;
	if (fmi_config->cir) {
		temp = (u64)3725 * le32_to_cpu(fmi_config->cir);
		temp = temp / 1000;
	}
	tsn_qci_fmi->cir = temp;
	tsn_qci_fmi->cbs = le32_to_cpu(fmi_config->cbs);
	temp = 0;
	if (fmi_config->eir) {
		temp = (u64)3725 * le32_to_cpu(fmi_config->eir);
		temp = temp / 1000;
	}
	tsn_qci_fmi->eir = temp;
	tsn_qci_fmi->ebs = le32_to_cpu(fmi_config->ebs);

	if (fmi_config->conf & 0x1)
		tsn_qci_fmi->mark_red = true;

	if (fmi_config->conf & 0x2)
		tsn_qci_fmi->mark_red_enable = true;

	if (fmi_config->conf & 0x4)
		tsn_qci_fmi->drop_on_yellow = true;

	if (fmi_config->conf & 0x8)
		tsn_qci_fmi->cm = true;

	if (fmi_config->conf & 0x10)
		tsn_qci_fmi->cf = true;

	memset(cbdr, 0, sizeof(*cbdr));

	/* Get counters */
	curr_cbd = alloc_cbdr(priv->si, &cbdr);

	cbdr->index = cpu_to_le16(index);
	cbdr->cmd = 2;
	cbdr->cls = BDCR_CMD_FLOW_METER;
	cbdr->status_flags = 0x0;

	data_size = sizeof(struct fmi_query_stat_resp);
	fmi_counter_data = kzalloc(data_size, __GFP_DMA | GFP_KERNEL);
	if (!fmi_counter_data)
		return -ENOMEM;

	dma = dma_map_single(&priv->si->pdev->dev, fmi_counter_data,
			     data_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(&priv->si->pdev->dev, dma)) {
		netdev_err(ndev, "DMA mapping failed!\n");
		kfree(fmi_counter_data);
		return -ENOMEM;
	}
	cbdr->addr[0] = cpu_to_le32(lower_32_bits(dma));
	cbdr->addr[1] = cpu_to_le32(upper_32_bits(dma));

	dma_size = cpu_to_le16(data_size);
	cbdr->length = dma_size;

	xmit_cbdr(priv->si, curr_cbd);

	memcpy(counters, fmi_counter_data, sizeof(*counters));

	return 0;
}

static int enetc_qbu_set(struct net_device *ndev, u8 preemptible_tcs)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_hw *hw = &priv->si->hw;
	u32 val;
	int err;

	err = tsn_qbu_ethtool_mm_compat_set(ndev, preemptible_tcs);
	if (err)
		return err;

	val = enetc_rd(hw, ENETC_PTGCR);
	if (val & ENETC_PTGCR_TGE)
		enetc_wr(hw, ENETC_PTGCR, val & ~ENETC_PTGCR_TGPE);

	enetc_change_preemptible_tcs(priv, preemptible_tcs);

	return 0;
}

static int enetc_qbu_get(struct net_device *ndev,
			 struct tsn_preempt_status *preemptstat)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int err;
	u32 val;

	err = tsn_qbu_ethtool_mm_compat_get(ndev, preemptstat);
	if (err)
		return err;

	preemptstat->admin_state = priv->preemptible_tcs;

	val = enetc_rd(&priv->si->hw, ENETC_PTGCR);
	preemptstat->hold_request = !!(val & ENETC_PTGCR_TGE) ? 1 : 2;
	preemptstat->hold_advance = val & 0xFFFF;
	preemptstat->release_advance = preemptstat->hold_advance;

	return 0;
}

static u32 __enetc_tsn_get_cap(struct enetc_si *si)
{
	u32 cap = TSN_CAP_CBS | TSN_CAP_TBS;

	if (si->hw_features & ENETC_SI_F_PSFP)
		cap |= TSN_CAP_QCI;

	if (si->hw_features & ENETC_SI_F_QBV)
		cap |= TSN_CAP_QBV;

	if (si->hw_features & ENETC_SI_F_QBU)
		cap |= TSN_CAP_QBU;

	return cap;
}

static u32 enetc_tsn_get_capability(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	return __enetc_tsn_get_cap(priv->si);
}

static int  __enetc_get_max_cap(struct enetc_si *si,
				struct tsn_qci_psfp_stream_param *stream_para)
{
	u32 reg;

	/* Port stream filter capability */
	reg = enetc_port_rd(&si->hw, ENETC_PSFCAPR);
	stream_para->max_sf_instance = reg & ENETC_PSFCAPR_MSK;
	/* Port stream filter capability */
	reg = enetc_port_rd(&si->hw, ENETC_PSGCAPR);
	stream_para->max_sg_instance = (reg & ENETC_PSGCAPR_SGIT_MSK);
	stream_para->supported_list_max = (reg & ENETC_PSGCAPR_GCL_MSK) >> 16;
	/* Port flow meter capability */
	reg = enetc_port_rd(&si->hw, ENETC_PFMCAPR);
	stream_para->max_fm_instance = reg & ENETC_PFMCAPR_MSK;

	return 0;
}

static int enetc_get_max_capa(struct net_device *ndev,
			      struct tsn_qci_psfp_stream_param *stream_para)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	return __enetc_get_max_cap(priv->si, stream_para);
}

static int enetc_set_cbs(struct net_device *ndev, u8 tc, u8 bw)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	struct enetc_cbs *ecbs = si->ecbs;
	struct enetc_cbs_tc_cfg *tc_cfg;
	u32 send_slope, hi_credit;
	u32 max_interference_size;
	u32 port_frame_max_size;
	u32 port_transmit_rate;
	int bw_sum = 0;
	u8 tc_nums;
	int i;

	if (!ecbs)
		return -ENOMEM;

	port_transmit_rate = priv->speed;
	if (port_transmit_rate != ecbs->port_transmit_rate)
		ecbs->port_transmit_rate = port_transmit_rate;
	port_frame_max_size = ecbs->port_max_size_frame;
	tc_nums = ecbs->tc_nums;
	tc_cfg = ecbs->tc_cfg;

	if (tc >= tc_nums) {
		dev_err(&ndev->dev, "Make sure the TC less than %d\n", tc_nums);
		return -EINVAL;
	}

	if (!bw) {
		if (tc_cfg[tc].enable) {
			/* Make sure the other TC that are numerically
			 * lower than this TC have been disabled.
			 */
			for (i = 0; i < tc; i++) {
				if (tc_cfg[i].enable)
					break;
			}
			if (i < tc) {
				dev_err(&ndev->dev,
					"TC%d has been disabled first\n", i);
				return -EINVAL;
			}
			memset(&tc_cfg[tc], 0, sizeof(*tc_cfg));
			tc_cfg[tc].enable = false;
			enetc_port_wr(&si->hw, ENETC_PTCCBSR1(tc), 0);
			enetc_port_wr(&si->hw, ENETC_PTCCBSR0(tc), 0);
		}
		return 0;
	}

	/* Make sure the other TC that are numerically
	 * higher than this TC have been enabled.
	 */
	for (i = tc_nums - 1; i > tc; i--) {
		if (!tc_cfg[i].enable) {
			dev_err(&ndev->dev,
				"TC%d has been enabled first\n", i);
			return -EINVAL;
		}
		bw_sum += tc_cfg[i].bw;
	}

	if (bw_sum + bw >= 100) {
		dev_err(&ndev->dev,
			"The sum of all CBS Bandwidth cann't exceed 100\n");
		return -EINVAL;
	}

	tc_cfg[tc].bw = bw;
	tc_cfg[tc].tc_max_sized_frame = enetc_port_rd(&si->hw, ENETC_PTCMSDUR(tc));
	tc_cfg[tc].idle_slope = port_transmit_rate / 100 * bw;
	tc_cfg[tc].send_slope = port_transmit_rate - tc_cfg[tc].idle_slope;

	/* For TC7, the max_interference_size is ENETC_MAC_MAXFRM_SIZE.
	 * For TC6, the max_interference_size is calculated as below:
	 *
	 *      max_interference_size = (M0 + Ma + Ra * M0 / (R0 - Ra))
	 *
	 * For other traffic class, for example SR class Q:
	 *
	 *                            R0 * (M0 + Ma + ... + Mp)
	 *      max_interference_size =  ------------------------------
	 *                            (R0 - Ra) + ... + (R0 - Rp)
	 *
	 */

	if (tc == tc_nums - 1) {
		tc_cfg[tc].max_interference_size = port_frame_max_size * 8;

	} else if (tc == tc_nums - 2) {
		if (!tc_cfg[tc + 1].send_slope)
			return -1;

		tc_cfg[tc].max_interference_size = (port_frame_max_size
				+ tc_cfg[tc + 1].tc_max_sized_frame
				+ port_frame_max_size * (tc_cfg[tc + 1].idle_slope
				/ tc_cfg[tc + 1].send_slope)) * 8;
	} else {
		max_interference_size = port_frame_max_size;
		send_slope = 0;
		for (i = tc + 1; i < tc_nums; i++) {
			send_slope += tc_cfg[i].send_slope;
			max_interference_size += tc_cfg[i].tc_max_sized_frame;
		}
		if (!send_slope)
			return -1;

		max_interference_size = ((u64)port_transmit_rate * max_interference_size) /
					send_slope;
		tc_cfg[tc].max_interference_size = max_interference_size * 8;
	}

	if (!port_transmit_rate)
		return -1;

	tc_cfg[tc].hi_credit = tc_cfg[tc].max_interference_size * tc_cfg[tc].bw / 100;
	tc_cfg[tc].lo_credit = tc_cfg[tc].tc_max_sized_frame *
			    (tc_cfg[tc].send_slope / port_transmit_rate);
	tc_cfg[tc].tc = tc;

	hi_credit = (ENETC_CLK * 100L) * (u64)tc_cfg[tc].hi_credit / port_transmit_rate;
	enetc_port_wr(&si->hw, ENETC_PTCCBSR1(tc), hi_credit);

	/* Set bw register and enable this traffic class */
	enetc_port_wr(&si->hw, ENETC_PTCCBSR0(tc),
		      (tc_cfg[tc].bw & 0x7F) | (1 << 31));
	tc_cfg[tc].enable = true;

	return 0;
}

static int enetc_get_cbs(struct net_device *ndev, u8 tc)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	struct enetc_cbs *ecbs = si->ecbs;
	struct enetc_cbs_tc_cfg *tc_cfg;

	if (!ecbs)
		return -ENOMEM;

	tc_cfg = ecbs->tc_cfg;
	if (tc >= ecbs->tc_nums) {
		dev_err(&ndev->dev, "The maximum of TC is %d\n", ecbs->tc_nums);
		return -EINVAL;
	}

	return tc_cfg[tc].bw;
}

static int enetc_set_tsd(struct net_device *ndev, struct tsn_tsd *ttsd)
{
	return -EOPNOTSUPP;
}

static int enetc_get_tsd(struct net_device *ndev, struct tsn_tsd_status *tts)
{
	return -EOPNOTSUPP;
}

static void enetc_cbs_init(struct enetc_si *si)
{
	struct enetc_ndev_priv *priv = netdev_priv(si->ndev);
	u8 tc_nums;

	tc_nums = priv->num_tx_rings;
	si->ecbs = kzalloc(sizeof(*si->ecbs) +
			   sizeof(struct enetc_cbs_tc_cfg) * tc_nums,
			   GFP_KERNEL);
	if (!si->ecbs)
		return;

	si->ecbs->port_max_size_frame = si->ndev->mtu + ETH_HLEN
						+ VLAN_HLEN + ETH_FCS_LEN;
	si->ecbs->tc_nums = tc_nums;
	si->ecbs->port_transmit_rate = priv->speed;

	/*This trick is used only for CFP*/
	if (!si->ecbs->port_transmit_rate)
		si->ecbs->port_transmit_rate = 1000000000;

	if (!si->ecbs->port_transmit_rate) {
		dev_err(&si->pdev->dev, "Failure to get port speed for CBS\n");
		kfree(si->ecbs);
		si->ecbs = NULL;
	}
}

static void enetc_qbv_init(struct enetc_hw *hw)
{
	/* Set PSPEED to be 1Gbps */
	enetc_port_wr(hw, ENETC_PMR,
		      (enetc_port_rd(hw, ENETC_PMR)
		      & (~ENETC_PMR_PSPEED_MASK))
		      | ENETC_PMR_PSPEED_1000M);
}

static void enetc_tsn_init(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	u32 capability;

	capability = __enetc_tsn_get_cap(si);

	if (capability & TSN_CAP_CBS)
		enetc_cbs_init(si);

	if (capability & TSN_CAP_QBV)
		enetc_qbv_init(&si->hw);

	if (capability & TSN_CAP_QCI)
		enetc_qci_enable(&si->hw);

	netdev_info(ndev, "TSN capabilities: 0x%x\n", capability);
}

static void enetc_tsn_deinit(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	u32 capability;

	capability = __enetc_tsn_get_cap(si);

	if (capability & TSN_CAP_QCI)
		enetc_qci_disable(&si->hw);

	dev_info(&si->pdev->dev, "%s: release\n", __func__);
}

static const struct tsn_ops enetc_tsn_ops_full = {
	.device_init = enetc_tsn_init,
	.device_deinit = enetc_tsn_deinit,
	.get_capability = enetc_tsn_get_capability,
	.qbv_set = enetc_qbv_set,
	.qbv_get = enetc_qbv_get,
	.qbv_get_status = enetc_qbv_get_status,
	.cb_streamid_set = enetc_cb_streamid_set,
	.cb_streamid_get = enetc_cb_streamid_get,
	.cb_streamid_counters_get = enetc_cb_streamid_counters_get,
	.qci_get_maxcap = enetc_get_max_capa,
	.qci_sfi_set = enetc_qci_sfi_set,
	.qci_sfi_get = enetc_qci_sfi_get,
	.qci_sfi_counters_get = enetc_qci_sfi_counters_get,
	.qci_sgi_set = enetc_qci_sgi_set,
	.qci_sgi_get = enetc_qci_sgi_get,
	.qci_sgi_status_get = enetc_qci_sgi_status_get,
	.qci_fmi_set = enetc_qci_fmi_set,
	.qci_fmi_get = enetc_qci_fmi_get,
	.qbu_set = enetc_qbu_set,
	.qbu_get = enetc_qbu_get,
	.cbs_set = enetc_set_cbs,
	.cbs_get = enetc_get_cbs,
	.tsd_set = enetc_set_tsd,
	.tsd_get = enetc_get_tsd,
};

static const struct tsn_ops enetc_tsn_ops_part = {
	.device_init = enetc_tsn_init,
	.device_deinit = enetc_tsn_deinit,
	.get_capability = enetc_tsn_get_capability,
	.cb_streamid_set = enetc_cb_streamid_set,
	.cb_streamid_get = enetc_cb_streamid_get,
	.cb_streamid_counters_get = enetc_cb_streamid_counters_get,
	.qci_get_maxcap = enetc_get_max_capa,
	.qci_sfi_set = enetc_qci_sfi_set,
	.qci_sfi_get = enetc_qci_sfi_get,
	.qci_sfi_counters_get = enetc_qci_sfi_counters_get,
	.qci_sgi_set = enetc_qci_sgi_set,
	.qci_sgi_get = enetc_qci_sgi_get,
	.qci_sgi_status_get = enetc_qci_sgi_status_get,
	.qci_fmi_set = enetc_qci_fmi_set,
	.qci_fmi_get = enetc_qci_fmi_get,
};

void enetc_tsn_pf_init(struct net_device *netdev, struct pci_dev *pdev)
{
	int port = pdev->devfn & 0x7;

	if (port == 1 || port == 3)
		tsn_port_register(netdev, &enetc_tsn_ops_part,
				  (u16)pdev->bus->number);
	else
		tsn_port_register(netdev, &enetc_tsn_ops_full,
				  (u16)pdev->bus->number);
}

void enetc_tsn_pf_deinit(struct net_device *netdev)
{
	tsn_port_unregister(netdev);
}
