// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Packet engine driver for Analog Devices Incorporated
 *
 * Currently tested on SC598 processor
 *
 * Copyright (c) 2023 - Timesys Corporation
 *   Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 */

#include <linux/sched.h>
#include <linux/crypto.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/minmax.h>
#include <linux/timekeeping.h>
#include <linux/platform_device.h>

#include <crypto/engine.h>
#include <crypto/scatterwalk.h>
#include <linux/dma-mapping.h>

/*Hashing*/
#include <crypto/hash.h>
#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <crypto/internal/hash.h>

/*Symmetric Crytography*/
#include <crypto/aes.h>
#include <crypto/internal/des.h>
#include <crypto/internal/aead.h>
#include <crypto/internal/skcipher.h>

#include "adi-pkte.h"
#include "adi-pkte-hash.h"

static int adi_one_request(struct crypto_engine *engine, void *areq);
static int adi_prepare_req(struct crypto_engine *engine, void *areq);

static int align_packet_16byte(struct adi_dev *pkte_dev, u32 packet_size,
			       u32 *data_offset)
{
	u32 i, n_pad_len = 0;
	u32 n_total_size;

	if (packet_size % 16 == 0)
		return 0;

	n_total_size = ((packet_size / 16) + 1) * 16;
	n_pad_len = n_total_size - packet_size;
	for (i = 0; i < n_pad_len; i++) {
		writeb(0x00, pkte_dev->io_base + *data_offset);
		*data_offset += 1;
	}

	return n_pad_len;
}

void adi_write_packet(struct adi_dev *pkte_dev, u32 *source)
{
	u32 i, w_iter, n_length, packet_size, n_pad_len = 0;
	u32 *source_offset;
	u32 data_offset;
	u8 ibuf_increment;
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;
	source_offset = source;
	data_offset =
	    adi_read(pkte_dev, BUF_PTR_OFFSET) & BITM_PKTE_BUF_PTR_INBUF;
	data_offset = data_offset >> BITP_PKTE_BUF_PTR_INBUF;
	data_offset += DATAIO_BUF_OFFSET;

	n_length = pkte_dev->src_bytes_available;
	packet_size = min(n_length, 128);
	w_iter = (!!(packet_size % 4) + packet_size / 4);
	for (i = 0; i < w_iter; i++) {
		adi_write(pkte_dev, data_offset, *source_offset);
		data_offset += 4;
		source_offset++;
	}

	dev_dbg(pkte_dev->dev, "%s: wrote %x bytes\n", __func__, packet_size);

	//Pad to 16 byte alignment/boundaries
	if (pkte->pPkteList.pCommand.opcode != opcode_hash)
		n_pad_len =
		    align_packet_16byte(pkte_dev, packet_size, &data_offset);

	pkte->pPkteList.pSource += packet_size / 4;
	ibuf_increment = packet_size + n_pad_len;
	if (ibuf_increment % 4)
		ibuf_increment += (4 - ibuf_increment % 4);

	pkte_dev->src_bytes_available -=
	    min(ibuf_increment, pkte_dev->src_bytes_available);
	adi_write(pkte_dev, INBUF_INCR_OFFSET, ibuf_increment);
}

static const struct adi_hash_cmd_params hash_params[] = {
	{
	 .hash_cmd = hash_md5,
	 .packet_size = 4 * 4,
	  },
	{
	 .hash_cmd = hash_sha1,
	 .packet_size = 5 * 4,
	  },
	{
	 .hash_cmd = hash_sha224,
	 .packet_size = 7 * 4,
	  },
	{
	 .hash_cmd = hash_sha256,
	 .packet_size = 8 * 4,
	  },

};

static int adi_get_hash_packet_size(int hash_type)
{
	if ((hash_type < ARRAY_SIZE(hash_params)) && (hash_type > 0))
		return hash_params[hash_type].packet_size;

	return -EINVAL;
}

static int adi_read_hash_to_dest(struct adi_dev *pkte_dev, u32 packet_size,
				 u32 *destination, u32 data_buf_offset)
{
	u32 read_iter, dest_offset_incr, hash_packet_size, final_packet_size;
	struct ADI_PKTE_DEVICE *pkte;
	u32 *dest_data_buf;
	u32 i;

	pkte = pkte_dev->pkte_device;
	dest_data_buf = destination;
	hash_packet_size =
	    adi_get_hash_packet_size(pkte->pPkteList.pCommand.hash);

	if (hash_packet_size < 0)
		return hash_packet_size;

	switch (pkte->pPkteList.pCommand.opcode) {
	case opcode_hash:
		final_packet_size = hash_packet_size;
		read_iter = final_packet_size / 4;
		dest_offset_incr = 4;
		break;
	case opcode_encrypt_hash:
		final_packet_size = packet_size + hash_packet_size;
		read_iter = final_packet_size / 4;
		dest_offset_incr = 1;
		break;
	default:
		final_packet_size = packet_size;
		read_iter =
		    (final_packet_size / 16 + final_packet_size % 16) * 16;
		dest_offset_incr = 1;
		break;
	}

	/* Check if buffer is about to overflow */
	if (read_iter > PKTE_BUFLEN) {
		dev_err(pkte_dev->dev,
			"Attempted to overwrite destination buffer!\n");
		return -ENOMEM;
	}

	for (i = 0; i < read_iter; i++) {
		*dest_data_buf = adi_read(pkte_dev, data_buf_offset);
		data_buf_offset += dest_offset_incr;
		dest_data_buf++;
	}

	return final_packet_size;
}

static int adi_read_packet(struct adi_dev *pkte_dev, u32 *destination)
{
	u32 data_buf_offset, imsk_stat_offset;
	u32 n_length, packet_size;
	u8 pos, ibuf_increment;
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;
	if (pkte_dev->flags & PKTE_AUTONOMOUS_MODE)
		pos = pkte_dev->ring_pos_consume;
	else
		pos = 0;

	imsk_stat_offset = adi_read(pkte_dev, IMSK_STAT_OFFSET);
	imsk_stat_offset &= BITM_PKTE_IMSK_EN_OPDN;
	if (imsk_stat_offset == BITM_PKTE_IMSK_EN_OPDN)
		return 0;

	data_buf_offset = DATAIO_BUF_OFFSET;
	n_length =
	    (pkte->pPkteDescriptor.CmdDescriptor[pos].PE_DEST_ADDR +
	     pkte->pPkteList.nSrcSize - adi_physical_address(pkte_dev, (void *)
							     pkte->pPkteList.pDestination));
	packet_size = min(n_length, 128);
	packet_size = adi_read_hash_to_dest(pkte_dev, packet_size,
					    destination, data_buf_offset);
	if (packet_size < 0)
		return -EINVAL;

	ibuf_increment = packet_size;
	if (ibuf_increment % 4)
		ibuf_increment = ((ibuf_increment / 4) + 1) * 4;

	adi_write(pkte_dev, OUTBUF_DECR_OFFSET, ibuf_increment);

	return 0;
}

static void adi_append_sg(struct adi_request_ctx *rctx,
			  struct adi_dev *pkte_dev)
{
	u32 *pkte_buf, pkte_buf_index;
	size_t count;
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;

	while ((rctx->bufcnt < rctx->buflen) && rctx->total) {
		count = min(rctx->sg->length - rctx->offset, rctx->total);
		/* Check if the page has data, if not, skip to next
		 * (if this is not the last entry)
		 */
		if (count <= 0)
			goto next_sg_entry;

		/* Since buflen > bufcnt for the loop to be true, above can only be
		 * possible if the page does not contain any data - i.e,
		 * sg->length = 0
		 */
		count = min(count, rctx->buflen - rctx->bufcnt);
		pkte_buf_index = pkte_dev->ring_pos_produce;
		pkte_buf = &pkte->source[pkte_buf_index][0] + rctx->bufcnt;
		scatterwalk_map_and_copy(pkte_buf, rctx->sg, rctx->offset,
					 count, 0);

		rctx->bufcnt += count;
		rctx->offset += count;
		rctx->total -= count;
		if (pkte_dev->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE)) {
			if (rctx->bufcnt >= PKTE_BUFLEN) {
				pkte_dev->ring_pos_produce++;
				if (pkte_dev->ring_pos_produce >=
				    PKTE_RING_BUFFERS)
					pkte_dev->ring_pos_produce = 0;
			}
		}

next_sg_entry:
		if (rctx->offset == rctx->sg->length) {
			rctx->sg = sg_next(rctx->sg);
			if (rctx->sg)
				rctx->offset = 0;
			else
				rctx->total = 0;
		}
	}
}

static void adi_prep_engine(struct adi_dev *pkte_dev, u32 hash_mode)
{
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;
	pkte_dev->src_count_set = 0;
	pkte_dev->src_bytes_available = 0;
	pkte_dev->ring_pos_produce = 0;
	pkte_dev->ring_pos_consume = 0;
	ready = 0;
	processing = 0;

	pkte->pPkteList.pCommand.opcode = opcode_hash;
	pkte->pPkteList.pCommand.direction = dir_outbound;
	pkte->pPkteList.pCommand.cipher = cipher_null;
	pkte->pPkteList.pCommand.cipher_mode = cipher_mode_ecb;
	pkte->pPkteList.pCommand.hash_mode = hash_mode;
	pkte->pPkteList.pCommand.aes_key_length = aes_key_length_other;
	pkte->pPkteList.pCommand.aes_des_key = aes_key;
	pkte->pPkteList.pCommand.hash_source = hash_source_no_load;

	pkte_dev->flags &= ~PKTE_FLAGS_STARTED;

	adi_start_engine(pkte_dev);
}

static void set_packet_final(struct adi_dev *pkte_dev, int *dev_final_hash)
{
	struct ADI_PKTE_COMMAND *pCmd;

	pCmd = &(pkte_dev->pkte_device->pPkteList.pCommand);
	pkte_dev->flags |= PKTE_FLAGS_FINAL;
	dev_dbg(pkte_dev->dev, "%s final hash condition set\n", __func__);
	if (pkte_dev->flags & PKTE_AUTONOMOUS_MODE) {
		pCmd->final_hash_condition = final_hash;
	} else {
		pCmd->final_hash_condition = final_hash;
		*dev_final_hash =
		    0x1 | (final_hash << BITP_PKTE_CTL_STAT_HASHFINAL);
		adi_write(pkte_dev, CTL_STAT_OFFSET, *dev_final_hash);
	}
}

static void pkte_continue_op(struct adi_dev *pkte_dev, int *dev_final_hash)
{
	u32 pos;
	struct SA *sa_record;

	dev_dbg(pkte_dev->dev, "%s hash_source_state set\n", __func__);
	if (pkte_dev->flags & PKTE_AUTONOMOUS_MODE) {
		pos = pkte_dev->ring_pos_consume;
		sa_record =
		    &pkte_dev->pkte_device->pPkteDescriptor.SARecord[pos];
		*dev_final_hash = sa_record->SA_Para.SA_CMD0;
		*dev_final_hash &= ~BITM_PKTE_SA_CMD0_HASHSRC;
		*dev_final_hash |=
		    hash_source_state << BITP_PKTE_SA_CMD0_HASHSRC;
		sa_record->SA_Para.SA_CMD0 = *dev_final_hash;
		return;
	}

	/* Set HASHSRC to hash_source_state */
	*dev_final_hash = adi_read(pkte_dev, SA_CMD_OFFSET(0));
	*dev_final_hash &= ~BITM_PKTE_SA_CMD0_HASHSRC;
	*dev_final_hash |= hash_source_state << BITP_PKTE_SA_CMD0_HASHSRC;
	pos = pkte_dev->ring_pos_consume;
	sa_record = &pkte_dev->pkte_device->pPkteDescriptor.SARecord[pos];
	sa_record->SA_Para.SA_CMD0 = *dev_final_hash;
	adi_write(pkte_dev, SA_CMD_OFFSET(0), *dev_final_hash);
}

#ifdef DEBUG_PKTE
static void print_packet_debug(struct adi_dev *pkte_dev)
{
	struct ADI_PKTE_DEVICE pkte;
	char debug_print[8192];
	int i, j;

	pkte = pkte_dev->pkte_device;
	for (i = 0, j = 0; i < length; i++)
		j += sprintf(&debug_print[j], "%02x ",
			     *(((u8 *) pkte->pPkteList.pSource) + i));
	debug_print[j] = 0;
	dev_info(pkte_dev->dev, "%s source data: %s\n", __func__, debug_print);
}
#endif

static int adi_process_packet(struct adi_dev *pkte_dev,
			      size_t length, int final)
{
	struct ADI_PKTE_DEVICE *pkte;
	u32 dev_final_hash;
	u32 len32, err;

	pkte = pkte_dev->pkte_device;
	len32 = DIV_ROUND_UP(length, sizeof(u32));
	dev_dbg(pkte_dev->dev, "%s: length: %zd, final: %x len32 %i\n",
		__func__, length, final, len32);
	if (final) {
		set_packet_final(pkte_dev, &dev_final_hash);
		if (length == 0) {
			ready = 1;
			wake_up_interruptible(&wq_ready);
			return 0;
		}
	}

	if (pkte_dev->flags & (PKTE_TCM_MODE | PKTE_HOST_MODE)) {
		wait_event_interruptible(wq_processing, processing == 0);
		processing = 1;
	}

	err = adi_wait_for_bit(pkte_dev, STAT_OFFSET, BITM_PKTE_STAT_OUTPTDN);
	if (err)
		return err;

	err =
	    adi_wait_for_bit(pkte_dev, CTL_STAT_OFFSET,
			     BITM_PKTE_CTL_STAT_PERDY);
	if (err)
		return err;

	pkte->pPkteList.pSource = &pkte->source[pkte_dev->ring_pos_consume][0];
#ifdef DEBUG_PKTE
	print_packet_debug(pkte_dev);
#endif
	if (pkte_dev->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE))
		adi_configure_cdr(pkte_dev);

	pkte->pPkteList.pDestination = &pkte->destination[0];
	adi_source_data(pkte_dev, length);
	pkte_dev->src_bytes_available = length;
	if (pkte_dev->flags & PKTE_TCM_MODE)
		adi_config_sa_para(pkte_dev);

	/* Continue with previous operation if pkte
	 * had already started, else start now
	 */
	if (pkte_dev->flags & PKTE_FLAGS_STARTED)
		pkte_continue_op(pkte_dev, &dev_final_hash);
	else
		pkte_dev->flags |= PKTE_FLAGS_STARTED;

	if (pkte_dev->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE)) {
		pkte_dev->ring_pos_consume++;
		if (adi_read(pkte_dev, RDSC_CNT_OFFSET))
			adi_write(pkte_dev, RDSC_DECR_OFFSET, 0x1);

		if (pkte_dev->ring_pos_consume >= PKTE_RING_BUFFERS)
			pkte_dev->ring_pos_consume = 0;

	}

	if (pkte_dev->flags & PKTE_HOST_MODE) {
		struct STATE *pkte_state;
		u32 pkte_state_addr;

		pkte_state = &pkte->pPkteDescriptor.State;
		pkte_state_addr = adi_physical_address(pkte_dev, (void *)pkte_state);
		adi_write(pkte_dev, CDSC_CNT_OFFSET, 1);
		adi_write(pkte_dev, SA_RDY_OFFSET, pkte_state_addr);
		adi_write(pkte_dev, BUF_THRESH_OFFSET,
			  128 << BITP_PKTE_BUF_THRESH_INBUF |
			  128 << BITP_PKTE_BUF_THRESH_OUTBUF);

		adi_write(pkte_dev, INT_EN_OFFSET,
			  BITM_PKTE_IMSK_EN_OPDN | BITM_PKTE_IMSK_EN_IBUFTHRSH);

	} else if (pkte_dev->flags & PKTE_TCM_MODE) {
		adi_init_spe(pkte_dev);
		adi_init_ring(pkte_dev);
		adi_write(pkte_dev, CDSC_CNT_OFFSET, 1);
		adi_write(pkte_dev, INT_EN_OFFSET, BITM_PKTE_IMSK_EN_RDRTHRSH);
	} else if (pkte_dev->flags & PKTE_AUTONOMOUS_MODE) {
		adi_write(pkte_dev, CDSC_CNT_OFFSET, 1);
		adi_write(pkte_dev, BUF_THRESH_OFFSET,
			  128 << BITP_PKTE_BUF_THRESH_INBUF |
			  128 << BITP_PKTE_BUF_THRESH_OUTBUF);
		if (final)
			adi_write(pkte_dev, INT_EN_OFFSET,
				  BITM_PKTE_IMSK_EN_RDRTHRSH);

	}

	return 0;
}

static int adi_prepare_secret_key(struct adi_dev *pkte_dev,
				  struct adi_request_ctx *rctx)
{
	struct ADI_PKTE_DEVICE *pkte;
	u8 *source_bytewise;
	u32 i, err;

	pkte = pkte_dev->pkte_device;
	/* Compute the inner hash of the HMAC (Result).  From RFC2104:
	 * HASH(Key XOR opad, HASH(Key XOR ipad, text))
	 */
	adi_prep_engine(pkte_dev, hash_mode_standard);
	memset(&pkte->source[pkte_dev->ring_pos_consume][0], 0,
	       INNER_OUTER_KEY_SIZE);

	memcpy(&pkte->source[pkte_dev->ring_pos_consume][0],
	       pkte_dev->secret_key, pkte_dev->secret_keylen);

	source_bytewise = (u8 *) &pkte->source[pkte_dev->ring_pos_consume][0];
	/* XOR padded key with ipad */
	for (i = 0; i < INNER_OUTER_KEY_SIZE; i++)
		*(source_bytewise + i) ^= 0x36;

	/* Hash XOR result */
	err = adi_process_packet(pkte_dev, INNER_OUTER_KEY_SIZE, 0);
	if (err)
		return err;

	if (!(pkte_dev->flags & PKTE_HOST_MODE)) {
		err =
		    adi_wait_for_bit(pkte_dev, STAT_OFFSET,
				     BITM_PKTE_STAT_OUTPTDN);
		if (err)
			return err;

	}

	err =
	    adi_wait_for_bit(pkte_dev, CTL_STAT_OFFSET,
			     BITM_PKTE_CTL_STAT_PERDY);
	if (err)
		return err;

	pkte_dev->ring_pos_produce++;
	pkte_dev->flags |= PKTE_FLAGS_HMAC_KEY_PREPARED;
	return 0;
}

#ifdef DEBUG_PKTE
static void adi_printkey(const u8 *key, unsigned int keylen,
			 struct adi_dev *pkte_dev)
{
	char dev_final_hash[256];
	int i, j;

	for (i = 0, j = 0; i < keylen; i++)
		j += sprintf(&dev_final_hash[j], "%c ", key[i]);
	dev_final_hash[j] = 0;
	dev_dbg(pkte_dev->dev, "%s HMAC key: %s\n", __func__, dev_final_hash);
}

#endif

static int adi_setkey(struct crypto_ahash *tfm,
		      const u8 *key, unsigned int keylen)
{
	struct adi_ctx *ctx = crypto_ahash_ctx(tfm);
	struct adi_dev *pkte_dev = adi_find_dev(ctx);

	if (keylen <= PKTE_MAX_KEY_SIZE) {

#ifdef DEBUG_PKTE
		adi_printkey(key, keylen, pkte_dev);
#endif

		if (keylen > INNER_OUTER_KEY_SIZE) {
			dev_err(pkte_dev->dev,
				"Key lengths > %d currently unsupported\n",
				INNER_OUTER_KEY_SIZE);
			return -ENODEV;
		}

		memcpy(pkte_dev->secret_key, key, keylen);
		pkte_dev->secret_keylen = keylen;
		return 0;

	}

	dev_err(pkte_dev->dev, "Key length exceeds limit!\n");
	return -ENOMEM;
}

static int adi_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct adi_ctx *ctx = crypto_ahash_ctx(tfm);
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *pkte_dev = adi_find_dev(ctx);
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;
	rctx->pkte_dev = pkte_dev;
	rctx->digcnt = crypto_ahash_digestsize(tfm);

	dev_dbg(pkte_dev->dev, "%s STAT=0x%08x CTL_STAT=0x%08x\n",
		__func__,
		adi_read(pkte_dev, STAT_OFFSET),
		adi_read(pkte_dev, CTL_STAT_OFFSET));

	adi_prep_engine(pkte_dev, opcode_hash);
	pkte->pPkteList.pCommand.final_hash_condition = not_final_hash;
	switch (rctx->digcnt) {
	case MD5_DIGEST_SIZE:
		dev_dbg(pkte_dev->dev, "%s, selected MD5 hashing\n", __func__);
		pkte->pPkteList.pCommand.hash = hash_md5;
		pkte->pPkteList.pCommand.digest_length = digest_length4;
		break;
	case SHA1_DIGEST_SIZE:
		dev_dbg(pkte_dev->dev, "%s, selected SHA1 hashing\n", __func__);
		pkte->pPkteList.pCommand.hash = hash_sha1;
		pkte->pPkteList.pCommand.digest_length = digest_length5;
		break;
	case SHA224_DIGEST_SIZE:
		dev_dbg(pkte_dev->dev, "%s, selected SHA224 hashing\n",
			__func__);
		pkte->pPkteList.pCommand.hash = hash_sha224;
		pkte->pPkteList.pCommand.digest_length = digest_length7;
		break;
	case SHA256_DIGEST_SIZE:
		dev_dbg(pkte_dev->dev, "%s, selected SHA256 hashing\n",
			__func__);
		pkte->pPkteList.pCommand.hash = hash_sha256;
		pkte->pPkteList.pCommand.digest_length = digest_length8;
		break;
	default:
		dev_dbg(pkte_dev->dev, "%s, unknown hashing request\n",
			__func__);
		return -EINVAL;
	}

	rctx->bufcnt = 0;
	rctx->total = 0;
	rctx->offset = 0;
	rctx->buflen = PKTE_BUFLEN;

	pkte_dev->ring_pos_produce = 0;
	pkte_dev->ring_pos_consume = 0;
	processing = 0;

	return 0;
}

static int adi_update_req(struct adi_dev *pkte_dev)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(pkte_dev->req);
	int bufcnt, err, final;

	dev_dbg(pkte_dev->dev, "%s flags %lx\n", __func__, pkte_dev->flags);
	final = (pkte_dev->flags & PKTE_FLAGS_FINUP);

	err = 0;
	if (rctx->total > rctx->buflen) {
		while (rctx->total > rctx->buflen) {
			adi_append_sg(rctx, pkte_dev);
			bufcnt = rctx->bufcnt;
			rctx->bufcnt = 0;
			err = adi_process_packet(pkte_dev, bufcnt, 0);
			if (err)
				return err;
		}

	} else {
		if ((rctx->total == (rctx->buflen)) ||
		    (rctx->bufcnt + rctx->total >= (rctx->buflen))) {
			adi_append_sg(rctx, pkte_dev);
			bufcnt = rctx->bufcnt;
			rctx->bufcnt = 0;
			err = adi_process_packet(pkte_dev, bufcnt, 0);
			if (err)
				return err;
		}

	}

	adi_append_sg(rctx, pkte_dev);
	if (final) {
		bufcnt = rctx->bufcnt;
		rctx->bufcnt = 0;
		err = adi_process_packet(pkte_dev, bufcnt, 1);
		if (err)
			return err;
	}

	return err;
}

static int adi_final_req(struct adi_dev *pkte_dev)
{
	struct ahash_request *req = pkte_dev->req;
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	int buflen;

	buflen = rctx->bufcnt;
	rctx->bufcnt = 0;
	return adi_process_packet(pkte_dev, buflen, 1);
}

static void adi_copy_hash(struct ahash_request *req)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	u32 *hash = (u32 *) rctx->digest;
	u32 i;
	struct ADI_PKTE_DEVICE *pkte;

	pkte = rctx->pkte_dev->pkte_device;

	rctx->digcnt = pkte->pPkteList.pCommand.digest_length * 4;

	//Endian Swap!
	if (pkte->pPkteList.pCommand.hash == hash_md5) {
		memcpy(hash, pkte->pPkteList.pDestination, rctx->digcnt);
		return;
	}

	for (i = 0; i < rctx->digcnt / 4; i++)
		hash[i] = __builtin_bswap32(pkte->pPkteList.pDestination[i]);

}

static int adi_finish(struct ahash_request *req)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *pkte_dev = rctx->pkte_dev;

	adi_reset_state(pkte_dev);

	if (!req->result) {
		dev_err(pkte_dev->dev, "%s: error\n", __func__);
		return -EINVAL;
	}

	memcpy(req->result, rctx->digest, rctx->digcnt);

	return 0;
}

static void adi_finish_req(struct ahash_request *req, int err)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *pkte_dev = rctx->pkte_dev;
	u32 i, digestLength;
	u8 *source_bytewise;
	struct ADI_PKTE_DEVICE *pkte;
	u32 pkte_ctl_stat;

	dev_dbg(pkte_dev->dev, "%s flags %lx\n", __func__, pkte_dev->flags);
	pkte = pkte_dev->pkte_device;

	if (err)
		goto finish_req_err;

	if (!(PKTE_FLAGS_FINAL & pkte_dev->flags))
		goto finish_req;

	wait_event_interruptible(wq_ready, ready == 1);
	err =
	    adi_wait_for_bit(pkte_dev, CTL_STAT_OFFSET,
			     BITM_PKTE_CTL_STAT_PERDY);
	if (err)
		goto finish_req_err;

	err = adi_read_packet(pkte_dev, &pkte->pPkteList.pDestination[0]);
	if (err)
		goto finish_req_err;

	if (!(PKTE_FLAGS_HMAC & pkte_dev->flags))
		goto finish_req;

	/* Compute the outer hash of the HMAC (Result).  From RFC2104:
	 * HASH(Key XOR opad, HASH(Key XOR ipad, text))
	 */
	dev_dbg(pkte_dev->dev,
		"%s processing req for hmac %lx\n", __func__, pkte_dev->flags);
	adi_prep_engine(pkte_dev, hash_mode_standard);

	memset(&pkte->source[pkte_dev->ring_pos_consume][0], 0,
	       INNER_OUTER_KEY_SIZE);
	/* Copy secret key */
	memcpy(&pkte->source[pkte_dev->ring_pos_consume][0],
	       pkte_dev->secret_key, pkte_dev->secret_keylen);
	source_bytewise = (u8 *) &pkte->source[pkte_dev->ring_pos_consume][0];
	/* XOR with Opad */
	for (i = 0; i < INNER_OUTER_KEY_SIZE; i++)
		*(source_bytewise + i) ^= 0x5c;
	digestLength = pkte->pPkteList.pCommand.digest_length;

	/* Endian Swap! */
	if (pkte->pPkteList.pCommand.hash == hash_md5) {
		for (i = 0; i < digestLength; i++)
			*((u32 *) (source_bytewise +
				   INNER_OUTER_KEY_SIZE +
				   i * 4)) = pkte->pPkteList.pDestination[i];
	} else {
		for (i = 0; i < digestLength; i++)
			*((u32 *) (source_bytewise +
				   INNER_OUTER_KEY_SIZE +
				   i * 4)) =
			    __builtin_bswap32(pkte->pPkteList.pDestination[i]);
	}

	err = adi_process_packet(pkte_dev,
				 INNER_OUTER_KEY_SIZE + (digestLength * 4), 1);
	if (err)
		goto finish_req_err;

	if (!(pkte_dev->flags & PKTE_HOST_MODE)) {
		pkte_ctl_stat =
		    adi_read(pkte_dev,
			     CTL_STAT_OFFSET) & BITM_PKTE_STAT_OUTPTDN;
		while (!pkte_ctl_stat) {
			pkte_ctl_stat =
			    adi_read(pkte_dev,
				     CTL_STAT_OFFSET) & BITM_PKTE_STAT_OUTPTDN;
			cond_resched();
		}

	}

	pkte_ctl_stat =
	    adi_read(pkte_dev, CTL_STAT_OFFSET) & BITM_PKTE_CTL_STAT_PERDY;
	while (!pkte_ctl_stat) {
		pkte_ctl_stat =
		    adi_read(pkte_dev,
			     CTL_STAT_OFFSET) & BITM_PKTE_CTL_STAT_PERDY;
		cond_resched();
	}

	err = adi_read_packet(pkte_dev, &pkte->pPkteList.pDestination[0]);
	if (err)
		goto finish_req_err;

finish_req:
	adi_copy_hash(req);
	err = adi_finish(req);

finish_req_err:
	crypto_finalize_hash_request(pkte_dev->engine, req, err);
}

static int adi_handle_queue(struct adi_dev *pkte_dev, struct ahash_request *req)
{
	return crypto_transfer_hash_request_to_engine(pkte_dev->engine, req);
}

static int adi_prepare_req(struct crypto_engine *engine, void *areq)
{
	struct ahash_request *req = container_of(areq, struct ahash_request,
						 base);
	struct adi_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct adi_dev *pkte_dev = adi_find_dev(ctx);
	struct adi_request_ctx *rctx;

	if (!pkte_dev)
		return -ENODEV;

	pkte_dev->req = req;

	rctx = ahash_request_ctx(req);

	dev_dbg(pkte_dev->dev, "processing new req, op: %lu, nbytes %d\n",
		rctx->op, req->nbytes);

	return adi_hw_init(pkte_dev);
}

static int adi_one_request(struct crypto_engine *engine, void *areq)
{
	struct ahash_request *req = container_of(areq, struct ahash_request,
						 base);
	struct adi_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct adi_dev *pkte_dev = adi_find_dev(ctx);
	struct adi_request_ctx *rctx;
	int err = 0;

	if (!pkte_dev)
		return -ENODEV;

	err = adi_prepare_req(engine, areq);
	if (err)
		return err;

	pkte_dev->req = req;

	rctx = ahash_request_ctx(req);

	switch (rctx->op) {
	case PKTE_OP_UPDATE:
		err = adi_update_req(pkte_dev);
		break;
	case PKTE_OP_FINAL:
		err = adi_final_req(pkte_dev);
		break;
	}

	if (err != -EINPROGRESS)
		adi_finish_req(req, err);

	return err;
}

static int adi_enqueue(struct ahash_request *req, unsigned int op)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *pkte_dev = rctx->pkte_dev;

	rctx->op = op;

	return adi_handle_queue(pkte_dev, req);
}

static int adi_update(struct ahash_request *req)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *pkte_dev = rctx->pkte_dev;
	int err;

	if (!req->nbytes)
		return 0;

	rctx->total = req->nbytes;
	rctx->sg = req->src;
	rctx->offset = 0;

	if (unlikely(pkte_dev->flags & PKTE_FLAGS_HMAC)) {
		if (!(pkte_dev->flags & PKTE_FLAGS_HMAC_KEY_PREPARED)) {
			dev_dbg(pkte_dev->dev, "preparing secret key\n");
			err = adi_prepare_secret_key(pkte_dev, rctx);
			if (err)
				return err;
		}
	}

	if ((rctx->bufcnt + rctx->total <= rctx->buflen)) {
		adi_append_sg(rctx, pkte_dev);
		return 0;
	}

	return adi_enqueue(req, PKTE_OP_UPDATE);
}

static int adi_final(struct ahash_request *req)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *pkte_dev = rctx->pkte_dev;

	pkte_dev->flags |= PKTE_FLAGS_FINUP;
	return adi_enqueue(req, PKTE_OP_FINAL);
}

static int adi_finup(struct ahash_request *req)
{
	struct adi_request_ctx *rctx = ahash_request_ctx(req);
	struct adi_dev *pkte_dev = rctx->pkte_dev;

	dev_dbg(pkte_dev->dev, "%s not yet implemented\n", __func__);

	return 0;
}

static int adi_digest(struct ahash_request *req)
{
	return adi_init(req) ? : adi_finup(req);
}

static int adi_export(struct ahash_request *req, void *out)
{
	struct adi_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct adi_dev *pkte_dev = adi_find_dev(ctx);

	dev_dbg(pkte_dev->dev, "%s not yet implemented\n", __func__);
	return 0;
}

static int adi_import(struct ahash_request *req, const void *in)
{
	struct adi_ctx *ctx = crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct adi_dev *pkte_dev = adi_find_dev(ctx);

	dev_dbg(pkte_dev->dev, "%s not yet implemented\n", __func__);
	return 0;
}

static int adi_cra_init_algs(struct crypto_tfm *tfm, const char *algs_hmac_name)
{
	struct adi_ctx *ctx = crypto_tfm_ctx(tfm);
	struct adi_dev *pkte_dev = adi_find_dev(ctx);

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct adi_request_ctx));

	pkte_dev->secret_keylen = 0;
	if (algs_hmac_name) {
		dev_dbg(pkte_dev->dev, "registering HMAC: %s\n",
			algs_hmac_name);
		pkte_dev->flags |= PKTE_FLAGS_HMAC;
	}

	ctx->enginectx.do_one_request = adi_one_request;
	return 0;
}

static int adi_cra_init(struct crypto_tfm *tfm)
{
	return adi_cra_init_algs(tfm, NULL);
}

static int adi_cra_md5_init(struct crypto_tfm *tfm)
{
	return adi_cra_init_algs(tfm, "md5");
}

static int adi_cra_sha1_init(struct crypto_tfm *tfm)
{
	return adi_cra_init_algs(tfm, "sha1");
}

static int adi_cra_sha224_init(struct crypto_tfm *tfm)
{
	return adi_cra_init_algs(tfm, "sha224");
}

static int adi_cra_sha256_init(struct crypto_tfm *tfm)
{
	return adi_cra_init_algs(tfm, "sha256");
}

static struct ahash_engine_alg algs_md5_sha1[] = {
	{
	 .base.init = adi_init,
	 .base.update = adi_update,
	 .base.final = adi_final,
	 .base.finup = adi_finup,
	 .base.digest = adi_digest,
	 .base.export = adi_export,
	 .base.import = adi_import,
	 .base.halg = {
		       .digestsize = MD5_DIGEST_SIZE,
		       .statesize = sizeof(struct adi_request_ctx),
		       .base = {
				.cra_name = "md5",
				.cra_driver_name = "adi-md5",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = MD5_HMAC_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_init = adi_cra_init,
				.cra_module = THIS_MODULE,
				}
		       },
	 .op.do_one_request = adi_one_request,
	  },
	{
	 .base.init = adi_init,
	 .base.update = adi_update,
	 .base.final = adi_final,
	 .base.finup = adi_finup,
	 .base.digest = adi_digest,
	 .base.export = adi_export,
	 .base.import = adi_import,
	 .base.setkey = adi_setkey,
	 .base.halg = {
		       .digestsize = MD5_DIGEST_SIZE,
		       .statesize = sizeof(struct adi_request_ctx),
		       .base = {
				.cra_name = "hmac(md5)",
				.cra_driver_name = "adi-hmac-md5",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = MD5_HMAC_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_init = adi_cra_md5_init,
				.cra_module = THIS_MODULE,
				}
		       },
	 .op.do_one_request = adi_one_request,
	  },
	{
	 .base.init = adi_init,
	 .base.update = adi_update,
	 .base.final = adi_final,
	 .base.finup = adi_finup,
	 .base.digest = adi_digest,
	 .base.export = adi_export,
	 .base.import = adi_import,
	 .base.halg = {
		       .digestsize = MD5_DIGEST_SIZE,
		       .digestsize = SHA1_DIGEST_SIZE,
		       .statesize = sizeof(struct adi_request_ctx),
		       .base = {
				.cra_name = "sha1",
				.cra_driver_name = "adi-sha1",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = SHA1_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_init = adi_cra_init,
				.cra_module = THIS_MODULE,
				}
		       },
	 .op.do_one_request = adi_one_request,
	  },
	{
	 .base.init = adi_init,
	 .base.update = adi_update,
	 .base.final = adi_final,
	 .base.finup = adi_finup,
	 .base.digest = adi_digest,
	 .base.export = adi_export,
	 .base.import = adi_import,
	 .base.setkey = adi_setkey,
	 .base.halg = {
		       .digestsize = SHA1_DIGEST_SIZE,
		       .statesize = sizeof(struct adi_request_ctx),
		       .base = {
				.cra_name = "hmac(sha1)",
				.cra_driver_name = "adi-hmac-sha1",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = SHA1_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_init = adi_cra_sha1_init,
				.cra_module = THIS_MODULE,
				}
		       },
	 .op.do_one_request = adi_one_request,
	  },
};

static struct ahash_engine_alg algs_sha224_sha256[] = {
	{
	 .base.init = adi_init,
	 .base.update = adi_update,
	 .base.final = adi_final,
	 .base.finup = adi_finup,
	 .base.digest = adi_digest,
	 .base.export = adi_export,
	 .base.import = adi_import,
	 .base.halg = {
		       .digestsize = SHA224_DIGEST_SIZE,
		       .statesize = sizeof(struct adi_request_ctx),
		       .base = {
				.cra_name = "sha224",
				.cra_driver_name = "adi-sha224",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = SHA224_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_init = adi_cra_init,
				.cra_module = THIS_MODULE,
				}
		       },
	 .op.do_one_request = adi_one_request,
	  },
	{
	 .base.init = adi_init,
	 .base.update = adi_update,
	 .base.final = adi_final,
	 .base.finup = adi_finup,
	 .base.digest = adi_digest,
	 .base.export = adi_export,
	 .base.import = adi_import,
	 .base.setkey = adi_setkey,
	 .base.halg = {
		       .digestsize = SHA224_DIGEST_SIZE,
		       .statesize = sizeof(struct adi_request_ctx),
		       .base = {
				.cra_name = "hmac(sha224)",
				.cra_driver_name = "adi-hmac-sha224",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = SHA224_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_init = adi_cra_sha224_init,
				.cra_module = THIS_MODULE,
				}
		       },
	 .op.do_one_request = adi_one_request,
	  },
	{
	 .base.init = adi_init,
	 .base.update = adi_update,
	 .base.final = adi_final,
	 .base.finup = adi_finup,
	 .base.digest = adi_digest,
	 .base.export = adi_export,
	 .base.import = adi_import,
	 .base.halg = {
		       .digestsize = SHA256_DIGEST_SIZE,
		       .statesize = sizeof(struct adi_request_ctx),
		       .base = {
				.cra_name = "sha256",
				.cra_driver_name = "adi-sha256",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = SHA256_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_init = adi_cra_init,
				.cra_module = THIS_MODULE,
				}
		       },
	 .op.do_one_request = adi_one_request,
	  },
	{
	 .base.init = adi_init,
	 .base.update = adi_update,
	 .base.final = adi_final,
	 .base.finup = adi_finup,
	 .base.digest = adi_digest,
	 .base.export = adi_export,
	 .base.import = adi_import,
	 .base.setkey = adi_setkey,
	 .base.halg = {
		       .digestsize = SHA256_DIGEST_SIZE,
		       .statesize = sizeof(struct adi_request_ctx),
		       .base = {
				.cra_name = "hmac(sha256)",
				.cra_driver_name = "adi-hmac-sha256",
				.cra_priority = 1000,
				.cra_flags = CRYPTO_ALG_ASYNC |
				CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize = SHA256_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct adi_ctx),
				.cra_init = adi_cra_sha256_init,
				.cra_module = THIS_MODULE,
				}
		       },
	 .op.do_one_request = adi_one_request,
	  },
};

struct adi_algs_info adi_algs_info_adi[NUM_HASH_CATEGORIES] = {
	{
	 .algs_list = algs_md5_sha1,
	 .size = ARRAY_SIZE(algs_md5_sha1),
	  },
	{
	 .algs_list = algs_sha224_sha256,
	 .size = ARRAY_SIZE(algs_sha224_sha256),
	  },
};
