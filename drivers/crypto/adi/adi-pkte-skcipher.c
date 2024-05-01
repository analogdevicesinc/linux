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

#include <linux/cacheflush.h>
#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

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
#include "adi-pkte-skcipher.h"

#define BITM_CIPHER_TYPE	0x38	/* bitmask for isolating cipher used */

enum CIPHER_MODE {
	UNUSED = 0 << 0,	/* not a bitmask */
	CBC = 1 << 1,
	ECB = 1 << 2,
	AES = 1 << 3,
	DES = 1 << 4,
	TDES = 1 << 5,
	ENCRYPT = 1 << 6,
	DECRYPT = 1 << 7,
};

u32 aes_key_length_lut[] = {
	aes_key_length256,
	aes_key_length192,
	aes_key_length128,
};

struct adi_crypt_reqctx {
	enum CIPHER_MODE mode;
};

static inline int aes_get_key_length(u32 keylen)
{
	u32 aes_index;

	aes_index = (keylen >> 6) - 1;
	if ((aes_index >= ARRAY_SIZE(aes_key_length_lut)) ||
			(aes_index < 0))
		return -EINVAL;
	return aes_key_length_lut[aes_index];
}

static int adi_crypt_configure_pkte(struct adi_dev *pkte_dev,
		struct skcipher_request *req)
{
	struct adi_crypt_reqctx *rctx;
	struct ADI_PKTE_DEVICE *pkte;
	struct adi_ctx *ctx;
	u32 i;

	rctx = skcipher_request_ctx(req);
	ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	pkte = pkte_dev->pkte_device;
	if (!(ctx->flags_skcipher & PKTE_FLAGS_STARTED)) {
		pkte_dev->src_count_set = 0;
		pkte_dev->src_bytes_available = 0;
		pkte_dev->ring_pos_produce = 0;
		pkte_dev->ring_pos_consume = 0;
		ready = 0;
		processing = 0;

		for (i = 0; i < 4; i++)
			pkte_dev->IV[i] = 0;

		if (rctx->mode & ENCRYPT) {
			pkte->pPkteList.pCommand.opcode = opcode_encrypt;
			pkte->pPkteList.pCommand.direction = dir_outbound;
		} else if (rctx->mode & DECRYPT) {
			pkte->pPkteList.pCommand.opcode = opcode_decrypt;
			pkte->pPkteList.pCommand.direction = dir_inbound;
		} else {
			dev_err(pkte_dev->dev, "%s: Unsupported Operation\n",
				__func__);
		}

		switch (rctx->mode & BITM_CIPHER_TYPE) {
		case AES:
			pkte->pPkteList.pCommand.aes_key_length =
				aes_get_key_length(ctx->keylen);
			if (pkte->pPkteList.pCommand.aes_key_length < 0) {
				dev_err(pkte_dev->dev, "Unsupported AES key length\n");
				return -EINVAL;
			}

			pkte->pPkteList.pCommand.cipher = cipher_aes;
			pkte->pPkteList.pCommand.aes_des_key = aes_key;
			break;
		case DES:
			pkte->pPkteList.pCommand.aes_key_length =
			    aes_key_length_other;
			pkte->pPkteList.pCommand.cipher = cipher_des;
			pkte->pPkteList.pCommand.aes_des_key = des_key;
			break;
		case TDES:
			pkte->pPkteList.pCommand.aes_key_length =
			    aes_key_length_other;
			pkte->pPkteList.pCommand.cipher = cipher_tdes;
			pkte->pPkteList.pCommand.aes_des_key = des_key;
			break;
		default:
			dev_err(pkte_dev->dev, "%s: Unsupported Mode\n", __func__);
			return -EINVAL;
		}

		if (rctx->mode & CBC)
			pkte->pPkteList.pCommand.cipher_mode = cipher_mode_cbc;
		else
			pkte->pPkteList.pCommand.cipher_mode = cipher_mode_ecb;

		pkte->pPkteList.pCommand.hash_mode = hash_mode_standard;
		pkte->pPkteList.pCommand.hash = hash_null;
		pkte->pPkteList.pCommand.digest_length = digest_length0;
		pkte->pPkteList.pCommand.hash_source = hash_source_no_load;
		pkte->pPkteList.pCommand.final_hash_condition = final_hash;
		adi_reset_state(pkte_dev);
		adi_start_engine(pkte_dev);
	}

	pkte->pPkteList.pSource = &pkte->source[pkte_dev->ring_pos_consume][0];
	if (pkte_dev->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE))
		adi_configure_cdr(pkte_dev);

	pkte->pPkteList.pDestination = &pkte->destination[0];

	return 0;
}

static void adi_crypt_prepare_IV(struct adi_dev *pkte_dev,
		struct adi_crypt_reqctx *rctx, u32 cryptlen)
{

	u8 *pkte_src, cpy_len, pos = 0;
	struct ADI_PKTE_DEVICE *pkte;

	pkte = pkte_dev->pkte_device;
	if ((rctx->mode & DES) || (rctx->mode & TDES)) {
		pos = (cryptlen - 8) / 4;
		cpy_len = 8;
		pkte_dev->IV[2] = 0;
		pkte_dev->IV[3] = 0;
	} else {
		pos = (cryptlen - 16) / 4;
		cpy_len = 16;
	}

	pkte_src = (u8 *) &pkte->source[pkte_dev->ring_pos_consume][pos];
	memcpy((u8 *) &pkte_dev->IV, pkte_src, cpy_len);
}

#ifdef DEBUG_PKTE
static void adi_crypt_debug_print_consume(struct adi_dev *pkte_dev,
		struct skcipher_request req);
{
	struct ADI_PKTE_DEVICE pkte;
	u32 i, read_iter;
	char temp[256];

	read_iter = req->cryptlen / 4;
	for (i = 0; i < read_iter; i++)
		dev_info(pkte_dev->dev, "SRC %x\n",
			pkte->source[pkte_dev->ring_pos_consume][i]);
}
#endif

static int adi_crypt_prepare_req(struct skcipher_request *req)
{
	struct adi_crypt_reqctx *rctx;
	struct ADI_PKTE_DEVICE *pkte;
	struct scatter_walk in;
	struct adi_dev *pkte_dev;
	struct adi_ctx *ctx;
	u32 ivsize;


	ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	ivsize = crypto_skcipher_ivsize(crypto_skcipher_reqtfm(req));
	pkte_dev = ctx->pkte_dev;
	pkte = pkte_dev->pkte_device;

	if (!req)
		return -EINVAL;

	if (!pkte_dev)
		return -ENODEV;

	if (pkte_dev->flags & PKTE_TCM_MODE) {
		dev_err(pkte_dev->dev, "%s: PKTE_TCM_MODE Currently untested",
			__func__);
		return -EINVAL;
	}

	if (pkte_dev->flags & PKTE_HOST_MODE) {
		dev_err(pkte_dev->dev, "%s: PKTE_HOST_MODE Currently untested",
			__func__);
		return -EINVAL;
	}

	adi_crypt_configure_pkte(pkte_dev, req);
	dev_dbg(pkte_dev->dev, "Setting key...\n");
	adi_config_sa_key(pkte_dev, ctx->key);
	if (rctx->mode & CBC) {
		if ((ctx->flags_skcipher & PKTE_FLAGS_STARTED)) {
			if ((rctx->mode & ENCRYPT)) {
				adi_crypt_prepare_IV(pkte_dev, rctx,
						pkte_dev->src_bytes_available);
				/* Feed previous result into IV */
				adi_config_state(pkte_dev, pkte_dev->IV);
			}

		} else {
			adi_config_state(pkte_dev, (u32 *) req->iv);
		}
	}

	adi_source_data(pkte_dev, req->cryptlen);
	pkte_dev->src_bytes_available = req->cryptlen;

	dev_dbg(pkte_dev->dev, "Started writing (via scatterwalk) %d bytes...\n",
		req->cryptlen);

	scatterwalk_start(&in, req->src);
	scatterwalk_copychunks(&pkte->source[pkte_dev->ring_pos_consume][0],
			       &in, req->cryptlen, 0);

	if (rctx->mode & (CBC | DECRYPT))
		adi_crypt_prepare_IV(pkte_dev, rctx, req->cryptlen);

#ifdef DEBUG_PKTE
	adi_crypt_debug_print_consume(pkte_dev, req);
#endif

	adi_write(pkte_dev, CDSC_CNT_OFFSET, 1);
	adi_write(pkte_dev, BUF_THRESH_OFFSET,
		  (u32) 128 << BITP_PKTE_BUF_THRESH_INBUF | (u32) 128 <<
		  BITP_PKTE_BUF_THRESH_OUTBUF);

	adi_write(pkte_dev, INT_EN_OFFSET, BITM_PKTE_IMSK_EN_RDRTHRSH);
	ctx->flags_skcipher |= PKTE_FLAGS_STARTED;
	return 0;
}

#ifdef DEBUG_PKTE
static void adi_crypt_debug_print_consume(struct adi_dev *pkte_dev,
		struct skcipher_request req);
{
	u32 i read_iter;

	read_iter = req->cryptlen / 4;
	for (i = 0; i < read_iter; i++)
		dev_dbg(pkte_dev->dev, "DST %x\n",
			pkte_dev->pkte_device->destination[i]);

}
#endif

static int adi_crypt_cipher_one_req(struct crypto_engine *engine, void *areq)
{
	struct skcipher_request *req = container_of(areq,
						    struct skcipher_request,
						    base);
	struct adi_ctx *ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct adi_dev *pkte_dev = ctx->pkte_dev;
	int err = 0;
	struct scatter_walk out;

	if (!pkte_dev)
		return -ENODEV;

	err = adi_crypt_prepare_cipher_req(engine, areq);
	if (err)
		return err;

	if (pkte_dev->flags & (PKTE_TCM_MODE | PKTE_HOST_MODE)) {
		wait_event_interruptible(wq_processing, processing == 0);
		processing = 1;
	}


	err = adi_wait_for_bit(pkte_dev, STAT_OFFSET, BITM_PKTE_STAT_OUTPTDN);
	if (err)
		return err;

	err = adi_wait_for_bit(pkte_dev, CTL_STAT_OFFSET, BITM_PKTE_CTL_STAT_PERDY);
	if (err)
		return err;

	scatterwalk_start(&out, req->dst);
	scatterwalk_copychunks(pkte_dev->pkte_device->destination, &out,
			       req->cryptlen, 1);

#ifdef DEBUG_PKTE
	adi_crypt_debug_print_dest(pkte_dev, req);
#endif
	crypto_finalize_skcipher_request(pkte_dev->engine, req, err);

	if (pkte_dev->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE)) {
		pkte_dev->ring_pos_consume++;
		if (adi_read(pkte_dev, RDSC_CNT_OFFSET))
			adi_write(pkte_dev, RDSC_DECR_OFFSET, 0x1);
		if (pkte_dev->ring_pos_consume >= PKTE_RING_BUFFERS)
			pkte_dev->ring_pos_consume = 0;
	}

	return 0;
}

int adi_crypt_prepare_cipher_req(struct crypto_engine *engine, void *areq)
{
	struct skcipher_request *req = container_of(areq,
						    struct skcipher_request,
						    base);

	return adi_crypt_prepare_req(req);
}

static int adi_init_skcipher(struct crypto_skcipher *tfm)
{
	struct adi_ctx *ctx = crypto_skcipher_ctx(tfm);

	crypto_skcipher_set_reqsize(tfm, sizeof(struct adi_crypt_reqctx));

	ctx->enginectx.do_one_request = adi_crypt_cipher_one_req;

	ctx->flags_skcipher &= ~PKTE_FLAGS_STARTED;

	return 0;
}

#ifdef DEBUG_PKTE
static void adi_print_key(const u8 *key, unsigned int keylen)
{
	int i, j;
	char temp[256];

	for (i = 0, j = 0; i < keylen; i++)
		j += sprintf(&temp[j], "%x ", key[i]);
	temp[j] = 0;
	pr_debug("%s Crypto key: %s\n", __func__, temp);

}
#endif

static int adi_crypt_setkey(struct crypto_skcipher *tfm, const u8 *key,
			    unsigned int keylen)
{
	struct adi_ctx *ctx = crypto_skcipher_ctx(tfm);

	if (keylen <= PKTE_MAX_KEY_SIZE) {
#ifdef DEBUG_PKTE
		adi_print_key(key, keylen);
#endif
		memcpy(ctx->key, key, keylen);
		ctx->keylen = keylen;
	}

	return 0;
}

static int adi_crypt_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
				unsigned int keylen)
{
	pr_debug("%s, keylen %d (%d %d %d)\n", __func__,
		 keylen, AES_KEYSIZE_128, AES_KEYSIZE_192, AES_KEYSIZE_256);

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 &&
	    keylen != AES_KEYSIZE_256)
		return -EINVAL;
	else
		return adi_crypt_setkey(tfm, key, keylen);
}

static int adi_crypt_des_setkey(struct crypto_skcipher *tfm, const u8 *key,
				unsigned int keylen)
{
	pr_debug("%s, keylen %d\n", __func__, keylen);

	return verify_skcipher_des_key(tfm, key) ? :
	    adi_crypt_setkey(tfm, key, keylen);
}

static int adi_crypt_tdes_setkey(struct crypto_skcipher *tfm, const u8 *key,
				 unsigned int keylen)
{
	pr_debug("%s, keylen %d\n", __func__, keylen);

	return verify_skcipher_des3_key(tfm, key) ? :
	    adi_crypt_setkey(tfm, key, keylen);
}

static int adi_crypt(struct skcipher_request *req, unsigned long mode)
{
	struct adi_ctx *ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct adi_crypt_reqctx *rctx = skcipher_request_ctx(req);
	struct adi_dev *pkte_dev = adi_find_dev(ctx);

	pr_debug("%s len=%d\n", __func__, req->cryptlen);

	if (!pkte_dev)
		return -ENODEV;

	rctx->mode = mode;

	return crypto_transfer_skcipher_request_to_engine(pkte_dev->engine, req);
}

static int adi_crypt_aes_cbc_decrypt(struct skcipher_request *req)
{

	if (req->cryptlen % AES_BLOCK_SIZE)
		return -EINVAL;

	if (req->cryptlen == 0)
		return 0;

	return adi_crypt(req, CBC | AES | DECRYPT);
}

static int adi_crypt_aes_cbc_encrypt(struct skcipher_request *req)
{

	if (req->cryptlen % AES_BLOCK_SIZE)
		return -EINVAL;

	if (req->cryptlen == 0)
		return 0;

	return adi_crypt(req, CBC | AES | ENCRYPT);
}

static int adi_crypt_des_cbc_decrypt(struct skcipher_request *req)
{

	if (req->cryptlen % DES_BLOCK_SIZE)
		return -EINVAL;

	if (req->cryptlen == 0)
		return 0;

	return adi_crypt(req, CBC | DES | DECRYPT);
}

static int adi_crypt_des_cbc_encrypt(struct skcipher_request *req)
{

	if (req->cryptlen % DES_BLOCK_SIZE)
		return -EINVAL;

	if (req->cryptlen == 0)
		return 0;

	return adi_crypt(req, CBC | DES | ENCRYPT);
}

static int adi_crypt_tdes_cbc_decrypt(struct skcipher_request *req)
{

	if (req->cryptlen % DES_BLOCK_SIZE)
		return -EINVAL;

	if (req->cryptlen == 0)
		return 0;

	return adi_crypt(req, CBC | TDES | DECRYPT);
}

static int adi_crypt_tdes_cbc_encrypt(struct skcipher_request *req)
{

	if (req->cryptlen % DES_BLOCK_SIZE)
		return -EINVAL;

	if (req->cryptlen == 0)
		return 0;

	return adi_crypt(req, CBC | TDES | ENCRYPT);
}

static int adi_crypt_aes_ecb_decrypt(struct skcipher_request *req)
{

	if (req->cryptlen % AES_BLOCK_SIZE)
		return -EINVAL;

	if (req->cryptlen == 0)
		return 0;

	return adi_crypt(req, ECB | AES | DECRYPT);
}

static int adi_crypt_aes_ecb_encrypt(struct skcipher_request *req)
{

	if (req->cryptlen % AES_BLOCK_SIZE)
		return -EINVAL;

	if (req->cryptlen == 0)
		return 0;

	return adi_crypt(req, ECB | AES | ENCRYPT);
}

static int adi_crypt_des_ecb_decrypt(struct skcipher_request *req)
{

	if (req->cryptlen % DES_BLOCK_SIZE)
		return -EINVAL;

	if (req->cryptlen == 0)
		return 0;

	return adi_crypt(req, ECB | DES | DECRYPT);
}

static int adi_crypt_des_ecb_encrypt(struct skcipher_request *req)
{

	if (req->cryptlen % DES_BLOCK_SIZE)
		return -EINVAL;

	if (req->cryptlen == 0)
		return 0;

	return adi_crypt(req, ECB | DES | ENCRYPT);
}

static int adi_crypt_tdes_ecb_decrypt(struct skcipher_request *req)
{

	if (req->cryptlen % DES_BLOCK_SIZE)
		return -EINVAL;

	if (req->cryptlen == 0)
		return 0;

	return adi_crypt(req, ECB | TDES | DECRYPT);
}

static int adi_crypt_tdes_ecb_encrypt(struct skcipher_request *req)
{

	if (req->cryptlen % DES_BLOCK_SIZE)
		return -EINVAL;

	if (req->cryptlen == 0)
		return 0;

	return adi_crypt(req, ECB | TDES | ENCRYPT);
}

struct skcipher_alg crypto_algs[NUM_CRYPTO_ALGS] = {
	{
	 .base.cra_name = "cbc(aes)",
	 .base.cra_driver_name = "adi-cbc-aes",
	 .base.cra_priority = 1000,
	 .base.cra_flags = CRYPTO_ALG_ASYNC,
	 .base.cra_blocksize = AES_BLOCK_SIZE,
	 .base.cra_ctxsize = sizeof(struct adi_ctx),
	 .base.cra_alignmask = 0,
	 .base.cra_module = THIS_MODULE,

	 .init = adi_init_skcipher,
	 .min_keysize = AES_MIN_KEY_SIZE,
	 .max_keysize = AES_MAX_KEY_SIZE,
	 .ivsize = AES_BLOCK_SIZE,
	 .setkey = adi_crypt_aes_setkey,
	 .encrypt = adi_crypt_aes_cbc_encrypt,
	 .decrypt = adi_crypt_aes_cbc_decrypt,
	  },
	{
	 .base.cra_name = "cbc(des)",
	 .base.cra_driver_name = "adi-cbc-des",
	 .base.cra_priority = 1000,
	 .base.cra_flags = CRYPTO_ALG_ASYNC,
	 .base.cra_blocksize = DES_BLOCK_SIZE,
	 .base.cra_ctxsize = sizeof(struct adi_ctx),
	 .base.cra_alignmask = 0,
	 .base.cra_module = THIS_MODULE,

	 .init = adi_init_skcipher,
	 .min_keysize = DES_BLOCK_SIZE,
	 .max_keysize = DES_BLOCK_SIZE,
	 .ivsize = DES_BLOCK_SIZE,
	 .setkey = adi_crypt_des_setkey,
	 .encrypt = adi_crypt_des_cbc_encrypt,
	 .decrypt = adi_crypt_des_cbc_decrypt,
	  },
	{
	 .base.cra_name = "cbc(des3_ede)",
	 .base.cra_driver_name = "adi-cbc-des3",
	 .base.cra_priority = 1000,
	 .base.cra_flags = CRYPTO_ALG_ASYNC,
	 .base.cra_blocksize = DES_BLOCK_SIZE,
	 .base.cra_ctxsize = sizeof(struct adi_ctx),
	 .base.cra_alignmask = 0,
	 .base.cra_module = THIS_MODULE,

	 .init = adi_init_skcipher,
	 .min_keysize = 3 * DES_BLOCK_SIZE,
	 .max_keysize = 3 * DES_BLOCK_SIZE,
	 .ivsize = DES_BLOCK_SIZE,
	 .setkey = adi_crypt_tdes_setkey,
	 .encrypt = adi_crypt_tdes_cbc_encrypt,
	 .decrypt = adi_crypt_tdes_cbc_decrypt,
	  },
	{
	 .base.cra_name = "ecb(aes)",
	 .base.cra_driver_name = "adi-ecb-aes",
	 .base.cra_priority = 1000,
	 .base.cra_flags = CRYPTO_ALG_ASYNC,
	 .base.cra_blocksize = AES_BLOCK_SIZE,
	 .base.cra_ctxsize = sizeof(struct adi_ctx),
	 .base.cra_alignmask = 0,
	 .base.cra_module = THIS_MODULE,

	 .init = adi_init_skcipher,
	 .min_keysize = AES_MIN_KEY_SIZE,
	 .max_keysize = AES_MAX_KEY_SIZE,
	 .setkey = adi_crypt_aes_setkey,
	 .encrypt = adi_crypt_aes_ecb_encrypt,
	 .decrypt = adi_crypt_aes_ecb_decrypt,
	  },
	{
	 .base.cra_name = "ecb(des)",
	 .base.cra_driver_name = "adi-ecb-des",
	 .base.cra_priority = 1000,
	 .base.cra_flags = CRYPTO_ALG_ASYNC,
	 .base.cra_blocksize = DES_BLOCK_SIZE,
	 .base.cra_ctxsize = sizeof(struct adi_ctx),
	 .base.cra_alignmask = 0,
	 .base.cra_module = THIS_MODULE,

	 .init = adi_init_skcipher,
	 .min_keysize = DES_BLOCK_SIZE,
	 .max_keysize = DES_BLOCK_SIZE,
	 .setkey = adi_crypt_des_setkey,
	 .encrypt = adi_crypt_des_ecb_encrypt,
	 .decrypt = adi_crypt_des_ecb_decrypt,
	  },
	{
	 .base.cra_name = "ecb(des3_ede)",
	 .base.cra_driver_name = "adi-ecb-des3",
	 .base.cra_priority = 1000,
	 .base.cra_flags = CRYPTO_ALG_ASYNC,
	 .base.cra_blocksize = DES_BLOCK_SIZE,
	 .base.cra_ctxsize = sizeof(struct adi_ctx),
	 .base.cra_alignmask = 0,
	 .base.cra_module = THIS_MODULE,

	 .init = adi_init_skcipher,
	 .min_keysize = 3 * DES_BLOCK_SIZE,
	 .max_keysize = 3 * DES_BLOCK_SIZE,
	 .setkey = adi_crypt_tdes_setkey,
	 .encrypt = adi_crypt_tdes_ecb_encrypt,
	 .decrypt = adi_crypt_tdes_ecb_decrypt,
	  },
};
