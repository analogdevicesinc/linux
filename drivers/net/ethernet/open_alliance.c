// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/**
 * Open Alliance SPI protocol driver for 10BASE-T1x Ethernet MAC-PHY
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/module.h>
#include <linux/open_alliance.h>
#include <linux/property.h>
#include <linux/interrupt.h>

#define OPEN_ALLIANCE_FEC_LEN		4
#define OPEN_ALLIANCE_MIN_FRAME_SIZE	64

static int open_alliance_chunk_sizes_map[][4] = {
	{ 8, 0x3 },
	{ 16, 0x4 },
	{ 32, 0x5 },
	{ 64, 0x6 },
};

static u8 open_alliance_crc1(u8 *buf, u32 nr_bits)
{
	u8 p = 1;
	int i;

	for (i = 0; i < nr_bits; i++)
		p ^= (buf[i / 8] >> (i % 8)) & 0x1;

	return p;
}

static int open_alliance_ctrl_tx_chunk_to_buffer(struct open_alliance *oa,
						 struct open_alliance_ctrl_tx_chunk *ctrl)
{
	u8 header_buf[OPEN_ALLIANCE_HEADER_SIZE];
	u32 transfer_size;
	__be32 __header;
	u32 data_size;
	u32 header = 0;
	u8 parity;

	header |= FIELD_PREP(OPEN_ALLIANCE_WNR, ctrl->wnr);
	header |= FIELD_PREP(OPEN_ALLIANCE_AID, ctrl->aid);
	header |= FIELD_PREP(OPEN_ALLIANCE_MMS, ctrl->mms);
	header |= FIELD_PREP(OPEN_ALLIANCE_ADDR, ctrl->addr);
	header |= FIELD_PREP(OPEN_ALLIANCE_LEN, ctrl->len - 1);

	__header = cpu_to_be32(header);
	memcpy(header_buf, &__header, OPEN_ALLIANCE_HEADER_SIZE);
	parity = open_alliance_crc1(header_buf, OPEN_ALLIANCE_HEADER_SIZE * 8);

	header |= FIELD_PREP(OPEN_ALLIANCE_P, parity);

	transfer_size = OPEN_ALLIANCE_HEADER_SIZE + ctrl->len * OPEN_ALLIANCE_MAX_REG_SIZE;
	if (oa->protected)
		transfer_size += ctrl->len * OPEN_ALLIANCE_MAX_REG_SIZE;

	if (transfer_size > OPEN_ALLIANCE_MAX_BUFF)
		return -ENOMEM;

	__header = cpu_to_be32(header);
	memcpy(oa->tx_data, &__header, OPEN_ALLIANCE_HEADER_SIZE);

	if (oa->protected)
		data_size = ctrl->len * OPEN_ALLIANCE_MAX_REG_SIZE * 2;
	else
		data_size = ctrl->len * OPEN_ALLIANCE_MAX_REG_SIZE;

	memcpy(oa->tx_data + OPEN_ALLIANCE_HEADER_SIZE, ctrl->data, data_size);

	return 0;
}

static int __open_alliance_write_reg(struct open_alliance *oa, u8 mms, u16 addr, u32 val)
{
	struct open_alliance_ctrl_tx_chunk ctrl = {0};
	struct spi_transfer t = {0};
	u32 echoed_write_val;
	u32 transfer_size;
	__be32 __val;
	int ret;

	transfer_size = 2 * OPEN_ALLIANCE_HEADER_SIZE + OPEN_ALLIANCE_MAX_REG_SIZE;
	if (oa->protected)
		transfer_size += OPEN_ALLIANCE_MAX_REG_SIZE;

	ctrl.wnr = 1;
	ctrl.mms = mms;
	ctrl.addr = addr;
	ctrl.len = 1;

	__val = cpu_to_be32(val);
	memcpy(ctrl.data, &__val, OPEN_ALLIANCE_MAX_REG_SIZE);
	if (oa->protected) {
		__val = ~__val;
		memcpy(ctrl.data + OPEN_ALLIANCE_MAX_REG_SIZE, &__val, OPEN_ALLIANCE_MAX_REG_SIZE);
	}

	ret = open_alliance_ctrl_tx_chunk_to_buffer(oa, &ctrl);
	if (ret < 0)
		return ret;

	t.rx_buf = oa->rx_data;
	t.tx_buf = oa->tx_data;
	t.len = transfer_size;

	ret = spi_sync_transfer(oa->spidev, &t, 1);
	if (ret < 0)
		return ret;

	echoed_write_val = get_unaligned_be32(t.rx_buf + OPEN_ALLIANCE_HEADER_SIZE * 2);
	if (echoed_write_val != val)
		return -EIO;

	return 0;
}

int open_alliance_write_reg(struct open_alliance *oa, u8 mms, u16 addr, u32 val)
{
	int ret;

	mutex_lock(&oa->spi_lock);
	ret = __open_alliance_write_reg(oa, mms, addr, val);
	mutex_unlock(&oa->spi_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(open_alliance_write_reg);

static int __open_alliance_read_reg(struct open_alliance *oa, u8 mms, u16 addr, u32 *val)
{
	struct open_alliance_ctrl_tx_chunk ctrl = {0};
	struct spi_transfer t = {0};
	u32 transfer_size;
	u32 read_val;
	int ret;

	transfer_size = 2 * OPEN_ALLIANCE_HEADER_SIZE + OPEN_ALLIANCE_MAX_REG_SIZE;
	if (oa->protected)
		transfer_size += OPEN_ALLIANCE_MAX_REG_SIZE;

	ctrl.mms = mms;
	ctrl.addr = addr;
	ctrl.len = 1;

	ret = open_alliance_ctrl_tx_chunk_to_buffer(oa, &ctrl);
	if (ret < 0)
		return ret;

	t.rx_buf = oa->rx_data;
	t.tx_buf = oa->tx_data;
	t.len = transfer_size;

	ret = spi_sync_transfer(oa->spidev, &t, 1);
	if (ret < 0)
		return ret;

	read_val = get_unaligned_be32(t.rx_buf + OPEN_ALLIANCE_HEADER_SIZE * 2);
	if (oa->protected) {
		u32 offset = OPEN_ALLIANCE_HEADER_SIZE * 2 + OPEN_ALLIANCE_MAX_REG_SIZE;
		u32 complement_val = get_unaligned_be32(t.rx_buf + offset);

		if (~complement_val != read_val) {
			dev_err(&oa->spidev->dev, "Open Alliance Protection Error.\n");
			return -EIO;
		}
	}

	*val = read_val;

	return 0;
}

int open_alliance_read_reg(struct open_alliance *oa, u8 mms, u16 addr, u32 *val)
{
	int ret;

	mutex_lock(&oa->spi_lock);
	ret = __open_alliance_read_reg(oa, mms, addr, val);
	mutex_unlock(&oa->spi_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(open_alliance_read_reg);

static int open_alliance_frame_to_chunks(struct open_alliance *oa,
					 struct sk_buff *txb, u8 vs)
{
	u8 frame_buf[OPEN_ALLIANCE_MIN_FRAME_SIZE];
	struct open_alliance_data_tx_chunk chunk = {0};
	unsigned int chunks_nr;
	u32 tx_len;
	u8 *tx_buf;
	int ret;
	int i;

	/* Pad frames lower than MAC limit */
	if (txb->len + OPEN_ALLIANCE_FEC_LEN < OPEN_ALLIANCE_MIN_FRAME_SIZE) {
		memset(frame_buf, 0, OPEN_ALLIANCE_MIN_FRAME_SIZE);
		memcpy(frame_buf, txb->data, txb->len);
		tx_buf = &frame_buf[0];
		tx_len = OPEN_ALLIANCE_MIN_FRAME_SIZE - OPEN_ALLIANCE_FEC_LEN;
	} else {
		tx_buf = txb->data;
		tx_len = txb->len;
	}

	chunks_nr = DIV_ROUND_UP(tx_len, oa->chunk_size);
	for (i = 0; i < chunks_nr; i++) {
		memset(&chunk, 0, sizeof(struct open_alliance_data_tx_chunk));

		chunk.dnc = 1;
		chunk.dv = 1;
		chunk.sv = !i;				/* Mark first chunk of a frame */
		chunk.ev = (i == (chunks_nr - 1));	/* Mark last chunk of a frame */
		chunk.vs = vs;

		/* Add the End Byte Offset to the last chunk of the frame */
		if (i == chunks_nr - 1)
			chunk.ebo = tx_len - 1;

		if (tx_len >= oa->chunk_size)
			memcpy(chunk.data, tx_buf + (i * oa->chunk_size), oa->chunk_size);
		else
			memcpy(chunk.data, tx_buf + (i * oa->chunk_size), tx_len);

		ret = kfifo_in_spinlocked(&oa->tx_chunks_kfifo, &chunk, 1, &oa->tx_lock);
		if (!ret)
			return -ENOMEM;

		tx_len -= oa->chunk_size;
	}

	return 0;
}

static int open_alliance_data_tx_chunks_to_buffer(struct open_alliance *oa,
						  struct open_alliance_data_tx_chunk *data,
						  u8 *buf)
{
	u8 header_buf[OPEN_ALLIANCE_HEADER_SIZE];
	u32 transfer_size;
	__be32 __header;
	u32 header = 0;
	u8 parity;

	header |= FIELD_PREP(OPEN_ALLIANCE_DNC,  data->dnc);
	header |= FIELD_PREP(OPEN_ALLIANCE_NORX, data->norx);
	header |= FIELD_PREP(OPEN_ALLIANCE_VS,   data->vs);
	header |= FIELD_PREP(OPEN_ALLIANCE_DV,   data->dv);
	header |= FIELD_PREP(OPEN_ALLIANCE_SV,   data->sv);
	header |= FIELD_PREP(OPEN_ALLIANCE_SWO,  data->swo);
	header |= FIELD_PREP(OPEN_ALLIANCE_EV,   data->ev);
	header |= FIELD_PREP(OPEN_ALLIANCE_EBO,  data->ebo);
	header |= FIELD_PREP(OPEN_ALLIANCE_TSC,  data->tsc);

	__header = cpu_to_be32(header);
	memcpy(header_buf, &__header, OPEN_ALLIANCE_HEADER_SIZE);
	parity = open_alliance_crc1(header_buf, OPEN_ALLIANCE_HEADER_SIZE * 8);

	header |= FIELD_PREP(OPEN_ALLIANCE_P, parity);

	transfer_size = OPEN_ALLIANCE_HEADER_SIZE + oa->chunk_size;

	if (transfer_size > OPEN_ALLIANCE_MAX_BUFF)
		return -ENOMEM;

	__header = cpu_to_be32(header);
	memcpy(buf, &__header, OPEN_ALLIANCE_HEADER_SIZE);
	memcpy(buf + OPEN_ALLIANCE_HEADER_SIZE, data->data, oa->chunk_size);

	return 0;
}

static int open_alliance_rx_buf_to_rx_chunks(struct open_alliance *oa,
					     struct open_alliance_data_rx_chunk *data,
					     u8 *buf)
{
	__be32 read_val;
	u8 parity_flag;
	u8 parity;

	/* chunk footer lies at the end of RX data */
	read_val = get_unaligned_be32(buf + oa->chunk_size);

	/* parity bit received needs to be zeroed before CRC */
	buf[oa->chunk_size + 3] &= ~OPEN_ALLIANCE_P;
	parity = open_alliance_crc1(buf + oa->chunk_size, OPEN_ALLIANCE_HEADER_SIZE * 8);
	parity_flag = FIELD_GET(OPEN_ALLIANCE_P, read_val);
	if (parity != parity_flag)
		return -EBADMSG;

	data->exst = FIELD_GET(OPEN_ALLIANCE_EXST, read_val);
	data->hdrb = FIELD_GET(OPEN_ALLIANCE_HDRB, read_val);
	data->sync = FIELD_GET(OPEN_ALLIANCE_SYNC, read_val);
	data->rca  = FIELD_GET(OPEN_ALLIANCE_RCA,  read_val);
	data->vs   = FIELD_GET(OPEN_ALLIANCE_VS,   read_val);
	data->dv   = FIELD_GET(OPEN_ALLIANCE_DV,   read_val);
	data->sv   = FIELD_GET(OPEN_ALLIANCE_SV,   read_val);
	data->swo  = FIELD_GET(OPEN_ALLIANCE_SWO,  read_val);
	data->fd   = FIELD_GET(OPEN_ALLIANCE_FD,   read_val);
	data->ev   = FIELD_GET(OPEN_ALLIANCE_EV,   read_val);
	data->ebo  = FIELD_GET(OPEN_ALLIANCE_EBO,  read_val);
	data->rtsa = FIELD_GET(OPEN_ALLIANCE_RTSA, read_val);
	data->txc  = FIELD_GET(OPEN_ALLIANCE_TXC,  read_val);

	/* copy data only if Data Valid Bit is set by the device */
	if (data->dv)
		memcpy(data->data, buf, oa->chunk_size);

	return 0;
}

/* Some devices will opt to use the Vendor Specific bits in the data
 * receive/transmit chunks as port source/destination.
 */
static int open_alliance_get_port_nr(struct open_alliance *oa,
				     struct open_alliance_data_rx_chunk *rx_data)
{
	u32 port_nr;

	if (oa->nr_ports > 1)
		port_nr = oa->vs_mask & rx_data->vs;
	else
		port_nr = 0;

	if (port_nr >= oa->nr_ports)
		return -EIO;

	return port_nr;
}

static int open_alliance_add_rx_chunk(struct open_alliance *oa,
				      struct open_alliance_data_rx_chunk *rx_data,
				      u32 port_nr)
{
	if (kfifo_is_full(&oa->rx_chunks_fifos[port_nr]))
		return -ENOMEM;

	return kfifo_in_spinlocked(&oa->rx_chunks_fifos[port_nr], rx_data, 1, &oa->rx_lock);
}

static bool open_alliance_rx_fifos_full(struct open_alliance *oa)
{
	int i;

	for (i = 0; i < oa->nr_ports; i++)
		if (kfifo_is_full(&oa->rx_chunks_fifos[i]))
			return true;

	return false;
}

/* Dequeue frames from FIFO until an End Valid Flag is found,
 * it is necessary to start assembling the frame with a
 * start valid chunk. Loose all chunks until a start valid
 * is found in the RX FIFO.
 */
static int open_alliance_rx_chunks_to_frame(struct open_alliance *oa)
{
	struct open_alliance_data_rx_chunk rx_data;
	struct open_alliance_rx_frame rx_frame;
	u32 rx_chunks_to_frames = 0;
	bool sv_found = false;
	bool ev_found = false;
	int chunk_data_size;
	struct sk_buff *rxb;
	u32 frame_size = 0;
	u32 start_byte;
	u32 end_byte;
	int port_nr;
	u8 *rxpkt;
	int ret;

	if (kfifo_is_empty(&oa->rx_ports_fifo))
		return 0;

	ret = kfifo_out_spinlocked(&oa->rx_ports_fifo, &rx_frame, 1, &oa->rx_lock);
	if (ret < 0)
		return ret;

	port_nr = rx_frame.port;
	if (port_nr >= oa->nr_ports)
		return -EINVAL;

	if (kfifo_is_empty(&oa->rx_chunks_fifos[port_nr]))
		return 0;

	while (!kfifo_is_empty(&oa->rx_chunks_fifos[port_nr])) {
		start_byte = 0;
		end_byte = oa->chunk_size - 1;

		ret = kfifo_out_spinlocked(&oa->rx_chunks_fifos[port_nr], &rx_data,
					   1, &oa->rx_lock);
		if (ret < 0)
			return ret;

		if (!rx_data.dv || rx_data.rtsa)
			continue;

		if (rx_data.sv) {
			/* Frames start at any 32-bit aligned word
			 * within the TX data chunk payload.
			 */
			sv_found = true;
			start_byte = rx_data.swo * 4;
		}

		if (!sv_found)
			continue;

		rx_chunks_to_frames++;
		if (rx_data.ev) {
			end_byte = rx_data.ebo;

			if (rx_data.fd)
				return 0;
		}

		chunk_data_size = end_byte - start_byte + 1;

		/* Mismatched start chunk and end chunk can lead to the following condition: */
		if (chunk_data_size > oa->chunk_size || chunk_data_size < 1)
			continue;

		memcpy(oa->frame_data + frame_size, rx_data.data + start_byte, chunk_data_size);
		frame_size += chunk_data_size;

		if (rx_data.ev) {
			ev_found = true;
			break;
		}
	}

	if (!ev_found)
		return -ENODATA;

	rxb = netdev_alloc_skb(oa->netdevs[port_nr], frame_size);
	if (!rxb)
		return -ENOMEM;

	/* point dev to the right port netdev */
	oa->statistics[OA_RX_CHUNKS_TO_FRAMES] += rx_chunks_to_frames;

	rxpkt = skb_put(rxb, frame_size);
	memcpy(rxpkt, oa->frame_data, frame_size);

	/* FIXME: after forwarding problems are solved, the bellow assignment needs to be removed */
	if (oa->nr_ports > 1)
		rxb->offload_fwd_mark = 1;

	rxb->protocol = eth_type_trans(rxb, oa->netdevs[port_nr]);

	return netif_receive_skb(rxb);
}

static void open_alliance_irq_rx_work(struct work_struct *work)
{
	struct open_alliance *oa = container_of(work, struct open_alliance, irq_rx_work);

	oa->set_rx_irq(oa->priv, oa->rx_irq_masked);
}

static void open_alliance_rx_irq_masked(struct open_alliance *oa, bool mask)
{
	oa->rx_irq_masked = mask;
	schedule_work(&oa->irq_rx_work);
}

static int open_alliance_transfer(struct open_alliance *oa)
{
	struct open_alliance_data_rx_chunk rx_data;
	struct open_alliance_data_tx_chunk tx_data;
	struct spi_transfer t = {0};
	u32 chunks_to_receive;
	u32 transfer_size;
	int port_nr = 0;
	u32 offset;
	int ret;
	int i;

	if (kfifo_len(&oa->tx_chunks_kfifo) > oa->rca)
		chunks_to_receive = kfifo_len(&oa->tx_chunks_kfifo);
	else
		chunks_to_receive = oa->rca;

	transfer_size = chunks_to_receive * (oa->chunk_size + OPEN_ALLIANCE_HEADER_SIZE);
	if (transfer_size >= OPEN_ALLIANCE_MAX_BUFF) {
		chunks_to_receive = OPEN_ALLIANCE_MAX_BUFF;
		chunks_to_receive /= (oa->chunk_size + OPEN_ALLIANCE_HEADER_SIZE);
		transfer_size = (oa->chunk_size + OPEN_ALLIANCE_HEADER_SIZE);
		transfer_size *= chunks_to_receive;
	}

	for (i = 0; i < chunks_to_receive; i++) {
		offset = (oa->chunk_size + OPEN_ALLIANCE_HEADER_SIZE) * i;

		/* To talk to device, host needs some data TX chunks or empty.
		 * Send data to the device only if TX credits are available.
		 */
		if (!kfifo_is_empty(&oa->tx_chunks_kfifo) && oa->txc) {
			ret = kfifo_out_spinlocked(&oa->tx_chunks_kfifo, &tx_data, 1,
						   &oa->tx_lock);
			if (ret < 0)
				return ret;

			oa->statistics[OA_TX_CHUNKS_SENT]++;
			oa->txc--;
		} else {
			memset(&tx_data, 0, sizeof(struct open_alliance_data_tx_chunk));
			tx_data.dnc = 1;
		}

		/* Tell device to not send RX data chunks yet to
		 * the host if there is no room in one of the receive FIFOs.
		 */
		if (open_alliance_rx_fifos_full(oa))
			tx_data.norx = 1;

		ret = open_alliance_data_tx_chunks_to_buffer(oa, &tx_data,
							     oa->tx_data + offset);
		if (ret < 0)
			return ret;
	}

	t.rx_buf = oa->rx_data;
	t.tx_buf = oa->tx_data;
	t.len = transfer_size;

	ret = spi_sync_transfer(oa->spidev, &t, 1);
	if (ret < 0)
		return ret;

	oa->statistics[OA_CHUNKS_TRANSFERED] += chunks_to_receive;

	for (i = 0; i < chunks_to_receive; i++) {
		offset = (oa->chunk_size + OPEN_ALLIANCE_HEADER_SIZE) * i;

		/* rx_data might be populated with possible data chunks. */
		ret = open_alliance_rx_buf_to_rx_chunks(oa, &rx_data, oa->rx_data + offset);
		if (ret < 0)
			return ret;

		oa->txc = rx_data.txc;
		oa->rca = rx_data.rca;

		if (rx_data.dv) {
			if (rx_data.sv) {
				/* FIXME: Circumvent a bug here where only the SV chunk will
				 * have the port address properly set. Rest of chunks that
				 * follow (bellonging to the same frame) will report port
				 * 0 no matter what.
				 */

				port_nr = open_alliance_get_port_nr(oa, &rx_data);
				if (port_nr < 0)
					return ret;
			}

			oa->statistics[OA_RX_CHUNKS_RECEIVED]++;

			/* If one RX FIFO becomes full here, it means
			 * that host failed to receive multiple End Valid
			 * flagged chunks from the device.
			 * Flush all stored chunks.
			 */
			if (open_alliance_rx_fifos_full(oa)) {
				int j;

				spin_lock(&oa->rx_lock);
				kfifo_reset(&oa->rx_ports_fifo);
				for (j = 0; j < oa->nr_ports; j++)
					kfifo_reset(&oa->rx_chunks_fifos[j]);
				spin_unlock(&oa->rx_lock);
			}

			ret = open_alliance_add_rx_chunk(oa, &rx_data, port_nr);
			if (ret < 0)
				return ret;

			/* When receiving an End Valid Flag, mark coresponding port. */
			if (rx_data.ev) {
				struct open_alliance_rx_frame rx_frame;

				oa->statistics[OA_RX_END_VALID_RECEIVED]++;
				rx_frame.port = port_nr;
				kfifo_in_spinlocked(&oa->rx_ports_fifo, &rx_frame, 1,
						    &oa->rx_lock);

				/* Full frame has been stored, notify NAPI */
				napi_schedule(&oa->napi);
			}
		}
	}

	return 0;
}

static int open_alliance_spi_transfer(void *pv)
{
	struct open_alliance *oa = pv;
	int ret;
	u32 val;

	while (!kthread_should_stop()) {

		schedule_timeout_interruptible(msecs_to_jiffies(5000) + 1);
		/* Check chip register for available chunks only if
		 * we did receive all of them in the last run.
		 */
		if (!oa->rca) {
			ret = open_alliance_read_reg(oa, OPEN_ALLIANCE_CTRL_ST,
						     OPEN_ALLIANCE_BUFSTS, &val);
			if (ret < 0)
				return ret;

			oa->rca = FIELD_GET(OPEN_ALLIANCE_BUFSTS_RCA, val);
			oa->txc = FIELD_GET(OPEN_ALLIANCE_BUFSTS_TXC, val);
		}

		mutex_lock(&oa->spi_lock);
		while (oa->rca || kfifo_len(&oa->tx_chunks_kfifo)) {

			open_alliance_transfer(oa);

			if (kthread_should_stop())
				break;
		}
		mutex_unlock(&oa->spi_lock);
	}

	return 0;
}

static int open_alliance_napi_poll(struct napi_struct *napi, int budget) {
	struct open_alliance *oa = netdev_get_ml_priv(napi->dev, ML_PRIV_OPEN_ALLIANCE);
	int frames_sent = 0;
	int ret;

	while (!kfifo_is_empty(&oa->rx_ports_fifo) && (budget > 0)) {
		ret = open_alliance_rx_chunks_to_frame(oa);
		if (ret >= 0)
			frames_sent++;

		budget--;
	}

	if (napi_complete_done(&oa->napi, frames_sent) && budget > 0)
		open_alliance_rx_irq_masked(oa, true);

	return frames_sent;
}

int open_alliance_read_rxb(struct open_alliance *oa)
{
	open_alliance_rx_irq_masked(oa, false);

	wake_up_process(oa->spi_thread);

	return 0;
}
EXPORT_SYMBOL_GPL(open_alliance_read_rxb);

int open_alliance_write_txb(struct open_alliance *oa, struct sk_buff *txb, u8 vs)
{
	int ret;

	/* Split the whole transfer into:
	 * 1. Storing the frame into chunks in the TX fifo
	 * 2. Doing the SPI transfers.
	 * The reason for this is because the former step can
	 * also be executed when reading frames from device.
	 */
	ret = open_alliance_frame_to_chunks(oa, txb, vs);
	if (ret < 0)
		return ret;

	wake_up_process(oa->spi_thread);

	return (OPEN_ALLIANCE_MAX_CHUNKS_NR - kfifo_len(&oa->tx_chunks_kfifo)) * oa->chunk_size;
}
EXPORT_SYMBOL_GPL(open_alliance_write_txb);

int open_alliance_tx_space(struct open_alliance *oa)
{
	return (OPEN_ALLIANCE_MAX_CHUNKS_NR - kfifo_len(&oa->tx_chunks_kfifo)) * oa->chunk_size;
}
EXPORT_SYMBOL_GPL(open_alliance_tx_space);

void open_alliance_ethtool_get_strings(struct open_alliance *oa, u32 sset, u8 *data)
{
	if (sset == ETH_SS_STATS) {
		int i;

		for (i = 0; i < ARRAY_SIZE(open_alliance_statistics); i++)
			memcpy(data + i * ETH_GSTRING_LEN,
			       open_alliance_statistics[i].name, ETH_GSTRING_LEN);
	}
}
EXPORT_SYMBOL_GPL(open_alliance_ethtool_get_strings);

int open_alliance_ethtool_get_sset_count(struct open_alliance *oa, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(open_alliance_statistics);
	default:
		return -EOPNOTSUPP;
	}
}
EXPORT_SYMBOL_GPL(open_alliance_ethtool_get_sset_count);

void open_alliance_ethtool_get_stats(struct open_alliance *oa, struct ethtool_stats *stats,
				     u64 *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(open_alliance_statistics); i++)
		*data++ = oa->statistics[i];
}
EXPORT_SYMBOL_GPL(open_alliance_ethtool_get_stats);

int open_alliance_net_open(struct open_alliance *oa)
{
	int ret;

	if (!oa || !kfifo_initialized(&oa->rx_ports_fifo))
		return -EINVAL;

	mutex_lock(&oa->spi_lock);

	if (!oa->enabled) {
		napi_enable(&oa->napi);
		oa->enabled = true;
	}

	mutex_unlock(&oa->spi_lock);

	ret = wake_up_process(oa->spi_thread);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(open_alliance_net_open);

int open_alliance_net_stop(struct open_alliance *oa)
{
	int ret;

	if (!oa || !kfifo_initialized(&oa->rx_ports_fifo))
		return -EINVAL;

	ret = kthread_stop(oa->spi_thread);
	if (ret < 0)
		return ret;

	mutex_lock(&oa->spi_lock);

	if (oa->enabled) {
		napi_disable(&oa->napi);
		oa->enabled = false;
	}

	mutex_unlock(&oa->spi_lock);

	flush_work(&oa->irq_rx_work);

	return 0;
}
EXPORT_SYMBOL_GPL(open_alliance_net_stop);

struct open_alliance *open_alliance_init(struct spi_device *spidev,
					 struct net_device *netdevs[OPEN_ALLIANCE_MAX_PORTS],
					 u32 nr_ports,
					 u32 vs_mask,
					 int (*set_rx_irq)(void *, bool),
					 void *p)
{
	struct fwnode_handle *fwnode;
	struct open_alliance *oa;
	u32 chunk_size;
	int ret;
	u32 val;
	int i;

	if (!spidev || nr_ports > OPEN_ALLIANCE_MAX_PORTS)
		return ERR_PTR(-EINVAL);

	oa = kzalloc(sizeof(*oa), GFP_KERNEL);
	if (!oa)
		return ERR_PTR(-ENOMEM);

	oa->spidev = spidev;
	oa->set_rx_irq = set_rx_irq;
	oa->priv = p;
	oa->nr_ports = nr_ports;
	INIT_WORK(&oa->irq_rx_work, open_alliance_irq_rx_work);

	oa->spi_thread = kthread_create(open_alliance_spi_transfer, oa, "open-alliance-spi-thread");
	if (IS_ERR(oa->spi_thread))
		return ERR_CAST(oa->spi_thread);

	for (i = 0; i < oa->nr_ports ; i++) {
		if (!netdevs[i])
			return ERR_PTR(-EINVAL);

		oa->netdevs[i] = netdevs[i];
		netdev_set_ml_priv(oa->netdevs[i], oa, ML_PRIV_OPEN_ALLIANCE);
	}

	fwnode = dev_fwnode(&spidev->dev);
	if (fwnode_property_present(fwnode, "open-alliance-chunk-size")) {
		ret = fwnode_property_read_u32_array(fwnode, "open-alliance-chunk-size",
						     &chunk_size, 1);
		if (ret < 0)
			return ERR_PTR(ret);
	} else {
		chunk_size = 64;
	}

	for (i = 0; i < ARRAY_SIZE(open_alliance_chunk_sizes_map); i++) {
		if (open_alliance_chunk_sizes_map[i][0] == chunk_size) {
			oa->chunk_size = chunk_size;
			val = open_alliance_chunk_sizes_map[i][1];
			break;
		}
	}

	if (!oa->chunk_size) {
		kfree(oa);
		return ERR_PTR(-EINVAL);
	}

	val = FIELD_PREP(GENMASK(2, 0), val);
	val |= OPEN_ALLIANCE_CONFIG0_ZARFE;
	ret = __open_alliance_write_reg(oa, OPEN_ALLIANCE_MMS_MAC, OPEN_ALLIANCE_CONFIG0, val);
	if (ret < 0) {
		kfifo_free(&oa->tx_chunks_kfifo);
		kfree(oa);
		return ERR_PTR(ret);
	}

	if (fwnode_property_present(fwnode, "open-alliance-protected"))
		oa->protected = true;

	ret = kfifo_alloc(&oa->tx_chunks_kfifo, OPEN_ALLIANCE_MAX_CHUNKS_NR,
			  GFP_KERNEL);
	if (ret < 0) {
		kfree(oa);
		return ERR_PTR(ret);
	}

	oa->vs_mask = vs_mask;
	ret = kfifo_alloc(&oa->rx_ports_fifo, OPEN_ALLIANCE_MAX_CHUNKS_NR, GFP_KERNEL);
	if (ret < 0) {
		kfifo_free(&oa->tx_chunks_kfifo);
		kfree(oa);
		return ERR_PTR(ret);
	}

	for (i = 0; i < oa->nr_ports; i++) {
		ret = kfifo_alloc(&oa->rx_chunks_fifos[i],
				  OPEN_ALLIANCE_MAX_CHUNKS_NR, GFP_KERNEL);
		if (ret < 0) {
			int j;

			for (j = --i; j >= 0; j--)
				kfifo_free(&oa->rx_chunks_fifos[j]);

			kfifo_free(&oa->tx_chunks_kfifo);
			kfifo_free(&oa->rx_ports_fifo);
			kfree(oa);

			return ERR_PTR(ret);
		}
	}

	mutex_init(&oa->spi_lock);
	spin_lock_init(&oa->tx_lock);
	spin_lock_init(&oa->rx_lock);

	netif_napi_add(oa->netdevs[0], &oa->napi, open_alliance_napi_poll, NAPI_POLL_WEIGHT);

	return oa;
}
EXPORT_SYMBOL_GPL(open_alliance_init);

int open_alliance_remove(struct open_alliance *oa)
{
	int i;

	netif_napi_del(&oa->napi);

	for (i = 0; i < oa->nr_ports; i++)
		kfifo_free(&oa->rx_chunks_fifos[i]);

	kfifo_free(&oa->tx_chunks_kfifo);
	kfifo_free(&oa->rx_ports_fifo);
	kfree(oa);

	return 0;
}
EXPORT_SYMBOL_GPL(open_alliance_remove);
