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

enum CIPHER_MODE {
	UNUSED  = 0 << 0, /* not a bitmask */
	CBC     = 1 << 1,
	ECB     = 1 << 2,
	AES     = 1 << 3,
	DES     = 1 << 4,
	TDES    = 1 << 5,
	ENCRYPT = 1 << 6,
	DECRYPT = 1 << 7,
};

struct adi_crypt_reqctx {
	enum CIPHER_MODE mode;
};

static int adi_crypt_prepare_req(struct skcipher_request *req)
{
	struct adi_ctx *ctx;
	struct adi_dev *crypt;
	struct adi_crypt_reqctx *rctx;
	int ret;
	int i, j, ivsize, pos;
	struct ADI_PKTE_DEVICE *pkte;

#ifdef DEBUG_PKTE
	char temp[256];
#endif

	ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	ivsize = crypto_skcipher_ivsize(crypto_skcipher_reqtfm(req));
	crypt = ctx->pkte_dev;
	pkte = crypt->pkte_device;

	if (!req)
		return -EINVAL;

	if (!crypt)
		return -ENODEV;


	if (crypt->flags & PKTE_TCM_MODE) {
		dev_err(crypt->dev, "%s: PKTE_TCM_MODE Currently untested", __func__);
		return -EINVAL;
	}

	if (crypt->flags & PKTE_HOST_MODE) {
		dev_err(crypt->dev, "%s: PKTE_HOST_MODE Currently untested", __func__);
		return -EINVAL;
	}

	rctx = skcipher_request_ctx(req);

	if (!(ctx->flags_skcipher & PKTE_FLAGS_STARTED)) {

		crypt->src_count_set = 0;
		crypt->src_bytes_available = 0;
		crypt->ring_pos_produce = 0;
		crypt->ring_pos_consume = 0;
		adi_pkte_ready = 0;
		adi_pkte_processing = 0;

		for (i = 0; i < 4; i++)
			adi_pkte_iv[i] = 0;

		if (rctx->mode & ENCRYPT) {
			pkte->pPkteList.pCommand.opcode = opcode_encrypt;
			pkte->pPkteList.pCommand.direction = dir_outbound;
		} else if (rctx->mode & DECRYPT) {
			pkte->pPkteList.pCommand.opcode = opcode_decrypt;
			pkte->pPkteList.pCommand.direction = dir_inbound;
		} else {
			dev_err(crypt->dev, "%s: Unsupported Operation\n", __func__);
		}

		if (rctx->mode & AES) {
			switch ((ctx->keylen*8)) {
			case 256:
				pkte->pPkteList.pCommand.aes_key_length = aes_key_length256;
				break;
			case 192:
				pkte->pPkteList.pCommand.aes_key_length = aes_key_length192;
				break;
			case 128:
				pkte->pPkteList.pCommand.aes_key_length = aes_key_length128;
				break;
			default:
				dev_err(crypt->dev, "%s: Keylen %d unsupported\n", __func__,
					ctx->keylen);
			}
		} else {
			pkte->pPkteList.pCommand.aes_key_length = aes_key_length_other;
		}

		if (rctx->mode & AES) {
			pkte->pPkteList.pCommand.cipher = cipher_aes;
			pkte->pPkteList.pCommand.aes_des_key = aes_key;
		} else if (rctx->mode & DES) {
			pkte->pPkteList.pCommand.cipher = cipher_des;
			pkte->pPkteList.pCommand.aes_des_key = des_key;
		} else if (rctx->mode & TDES) {
			pkte->pPkteList.pCommand.cipher = cipher_tdes;
			pkte->pPkteList.pCommand.aes_des_key = des_key;
		} else {
			dev_err(crypt->dev, "%s: Unsupported Mode\n", __func__);
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

		adi_reset_state(crypt);
		adi_start_engine(crypt);
	}

	pkte->pPkteList.pSource = &pkte->source[crypt->ring_pos_consume][0];

	if (crypt->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE))
		adi_configure_cdr(crypt);

	pkte->pPkteList.pDestination = &pkte->destination[0];

	dev_dbg(crypt->dev, "Setting key...\n");
	adi_config_sa_key(crypt, ctx->key);

	if (rctx->mode & CBC) {
		if ((ctx->flags_skcipher & PKTE_FLAGS_STARTED)) {
			if ((rctx->mode & ENCRYPT)) {

				if ((rctx->mode & DES) || (rctx->mode & TDES)) {
					pos = (crypt->src_bytes_available-8)/4;
					memcpy((u8 *)&adi_pkte_iv[0], &pkte->destination[pos], 8);
					adi_pkte_iv[2] = 0;
					adi_pkte_iv[3] = 0;
				} else {
					pos = (crypt->src_bytes_available-16)/4;
					memcpy((u8 *)&adi_pkte_iv[0], &pkte->destination[pos], 16);
				}

				//Feed previous result into IV
				adi_config_state(crypt, adi_pkte_iv);
			} else {
				//Feed previous result into IV
				adi_config_state(crypt, adi_pkte_iv);
			}
		} else {
			adi_config_state(crypt, (u32 *)req->iv);
		}
	}

	adi_source_data(crypt, req->cryptlen);
	crypt->src_bytes_available = req->cryptlen;

	dev_dbg(crypt->dev, "Started writing (via scatterwalk) %d bytes...\n", req->cryptlen);

	memcpy_from_sglist(&pkte->source[crypt->ring_pos_consume][0],
			   req->src, 0, req->cryptlen);

	if (rctx->mode & CBC) {
		if (rctx->mode & DECRYPT) {
			if ((rctx->mode & DES) || (rctx->mode & TDES)) {
				pos = (req->cryptlen-8)/4;
				memcpy((u8 *)&adi_pkte_iv[0],
					&pkte->source[crypt->ring_pos_consume][pos], 8);
				adi_pkte_iv[2] = 0;
				adi_pkte_iv[3] = 0;
			} else {
				pos = (req->cryptlen-16)/4;
				memcpy((u8 *)&adi_pkte_iv[0],
					&pkte->source[crypt->ring_pos_consume][pos], 16);
			}
		}
	}

#ifdef DEBUG_PKTE
	for (i = 0; i < req->cryptlen/4; i++)
		dev_dbg(crypt->dev, "SRC %x\n",
			pkte->source[crypt->ring_pos_consume][i]);
#endif

	adi_write(crypt, CDSC_CNT_OFFSET, 1);

	adi_write(crypt, BUF_THRESH_OFFSET, (u32)128<<BITP_PKTE_BUF_THRESH_INBUF |
					    (u32)128<<BITP_PKTE_BUF_THRESH_OUTBUF);

	adi_write(crypt, INT_EN_OFFSET, BITM_PKTE_IMSK_EN_RDRTHRSH);

	ctx->flags_skcipher |= PKTE_FLAGS_STARTED;

	return 0;
}

static int adi_crypt_cipher_one_req(struct crypto_engine *engine, void *areq)
{
	struct skcipher_request *req = container_of(areq,
						    struct skcipher_request,
						    base);
	struct adi_ctx *ctx = crypto_skcipher_ctx(
			crypto_skcipher_reqtfm(req));
	struct adi_dev *crypt = ctx->pkte_dev;
	int err;
	unsigned int i;

	err = adi_crypt_prepare_req(req);
	if (err)
		return err;


	if (crypt->flags & (PKTE_TCM_MODE | PKTE_HOST_MODE)) {
		wait_event_interruptible(adi_pkte_wq_processing, adi_pkte_processing == 0);
		adi_pkte_processing = 1;
	}

	while (!(adi_read(crypt, STAT_OFFSET) & BITM_PKTE_STAT_OUTPTDN))
		;

	while (!(adi_read(crypt, CTL_STAT_OFFSET) & BITM_PKTE_CTL_STAT_PERDY))
		;

	memcpy_to_sglist(req->dst, 0, crypt->pkte_device->destination, req->cryptlen);

#ifdef DEBUG_PKTE
	for (i = 0; i < req->cryptlen/4; i++)
		dev_dbg(crypt->dev, "DST %x\n", crypt->pkte_device->destination[i]);
#endif
	crypto_finalize_skcipher_request(crypt->engine, req,
					   err);

	if (crypt->flags & (PKTE_TCM_MODE | PKTE_AUTONOMOUS_MODE)) {
		crypt->ring_pos_consume++;
		if (adi_read(crypt, RDSC_CNT_OFFSET))
			adi_write(crypt, RDSC_DECR_OFFSET, 0x1);
		if (crypt->ring_pos_consume >= PKTE_RING_BUFFERS)
			crypt->ring_pos_consume = 0;
	}

	return 0;
}

static int adi_init_skcipher(struct crypto_skcipher *tfm)
{
	struct adi_ctx *ctx = crypto_skcipher_ctx(tfm);

	crypto_skcipher_set_reqsize(tfm, sizeof(struct adi_crypt_reqctx));

	ctx->flags_skcipher &= ~PKTE_FLAGS_STARTED;

	return 0;
}

static int adi_crypt_setkey(struct crypto_skcipher *tfm, const u8 *key,
			     unsigned int keylen)
{
	struct adi_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct adi_dev *crypt = ctx->pkte_dev;
	int i, j;
	char temp[256];

	if (keylen <= PKTE_MAX_KEY_SIZE) {
#ifdef DEBUG_PKTE
		for (i = 0, j = 0; i < keylen; i++)
			j += sprintf(&temp[j], "%x ", key[i]);
		temp[j] = 0;
		pr_debug("%s Crypto key: %s\n", __func__, temp);
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

	return verify_skcipher_des_key(tfm, key) ?:
	       adi_crypt_setkey(tfm, key, keylen);
}

static int adi_crypt_tdes_setkey(struct crypto_skcipher *tfm, const u8 *key,
				 unsigned int keylen)
{
	pr_debug("%s, keylen %d\n", __func__, keylen);

	return verify_skcipher_des3_key(tfm, key) ?:
	       adi_crypt_setkey(tfm, key, keylen);
}


static int adi_crypt(struct skcipher_request *req, unsigned long mode)
{
	struct adi_ctx *ctx = crypto_skcipher_ctx(
			crypto_skcipher_reqtfm(req));
	struct adi_crypt_reqctx *rctx = skcipher_request_ctx(req);
	struct adi_dev *crypt = adi_find_dev(ctx);

	pr_debug("%s len=%d\n", __func__, req->cryptlen);

	if (!crypt)
		return -ENODEV;

	rctx->mode = mode;

	return crypto_transfer_skcipher_request_to_engine(crypt->engine, req);
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

struct skcipher_engine_alg adi_pkte_skcipher_algs[NUM_CRYPTO_ALGS] = {
	{
		.base.base.cra_name		= "cbc(aes)",
		.base.base.cra_driver_name	= "adi-cbc-aes",
		.base.base.cra_priority	= 1000,
		.base.base.cra_flags		= CRYPTO_ALG_ASYNC,
		.base.base.cra_blocksize	= AES_BLOCK_SIZE,
		.base.base.cra_ctxsize	= sizeof(struct adi_ctx),
		.base.base.cra_alignmask	= 0,
		.base.base.cra_module	= THIS_MODULE,

		.base.init			= adi_init_skcipher,
		.op.do_one_request	= adi_crypt_cipher_one_req,
		.base.min_keysize		= AES_MIN_KEY_SIZE,
		.base.max_keysize		= AES_MAX_KEY_SIZE,
		.base.ivsize			= AES_BLOCK_SIZE,
		.base.setkey			= adi_crypt_aes_setkey,
		.base.encrypt		= adi_crypt_aes_cbc_encrypt,
		.base.decrypt		= adi_crypt_aes_cbc_decrypt,
	},
	{
		.base.base.cra_name		= "cbc(des)",
		.base.base.cra_driver_name	= "adi-cbc-des",
		.base.base.cra_priority	= 1000,
		.base.base.cra_flags		= CRYPTO_ALG_ASYNC,
		.base.base.cra_blocksize	= DES_BLOCK_SIZE,
		.base.base.cra_ctxsize	= sizeof(struct adi_ctx),
		.base.base.cra_alignmask	= 0,
		.base.base.cra_module	= THIS_MODULE,

		.base.init			= adi_init_skcipher,
		.op.do_one_request	= adi_crypt_cipher_one_req,
		.base.min_keysize		= DES_BLOCK_SIZE,
		.base.max_keysize		= DES_BLOCK_SIZE,
		.base.ivsize			= DES_BLOCK_SIZE,
		.base.setkey			= adi_crypt_des_setkey,
		.base.encrypt		= adi_crypt_des_cbc_encrypt,
		.base.decrypt		= adi_crypt_des_cbc_decrypt,
	},
	{
		.base.base.cra_name		= "cbc(des3_ede)",
		.base.base.cra_driver_name	= "adi-cbc-des3",
		.base.base.cra_priority	= 1000,
		.base.base.cra_flags		= CRYPTO_ALG_ASYNC,
		.base.base.cra_blocksize	= DES_BLOCK_SIZE,
		.base.base.cra_ctxsize	= sizeof(struct adi_ctx),
		.base.base.cra_alignmask	= 0,
		.base.base.cra_module	= THIS_MODULE,

		.base.init			= adi_init_skcipher,
		.op.do_one_request	= adi_crypt_cipher_one_req,
		.base.min_keysize		= 3 * DES_BLOCK_SIZE,
		.base.max_keysize		= 3 * DES_BLOCK_SIZE,
		.base.ivsize			= DES_BLOCK_SIZE,
		.base.setkey			= adi_crypt_tdes_setkey,
		.base.encrypt		= adi_crypt_tdes_cbc_encrypt,
		.base.decrypt		= adi_crypt_tdes_cbc_decrypt,
	},
	{
		.base.base.cra_name		= "ecb(aes)",
		.base.base.cra_driver_name	= "adi-ecb-aes",
		.base.base.cra_priority	= 1000,
		.base.base.cra_flags		= CRYPTO_ALG_ASYNC,
		.base.base.cra_blocksize	= AES_BLOCK_SIZE,
		.base.base.cra_ctxsize	= sizeof(struct adi_ctx),
		.base.base.cra_alignmask	= 0,
		.base.base.cra_module	= THIS_MODULE,

		.base.init			= adi_init_skcipher,
		.op.do_one_request	= adi_crypt_cipher_one_req,
		.base.min_keysize		= AES_MIN_KEY_SIZE,
		.base.max_keysize		= AES_MAX_KEY_SIZE,
		.base.setkey			= adi_crypt_aes_setkey,
		.base.encrypt		= adi_crypt_aes_ecb_encrypt,
		.base.decrypt		= adi_crypt_aes_ecb_decrypt,
	},
	{
		.base.base.cra_name		= "ecb(des)",
		.base.base.cra_driver_name	= "adi-ecb-des",
		.base.base.cra_priority	= 1000,
		.base.base.cra_flags		= CRYPTO_ALG_ASYNC,
		.base.base.cra_blocksize	= DES_BLOCK_SIZE,
		.base.base.cra_ctxsize	= sizeof(struct adi_ctx),
		.base.base.cra_alignmask	= 0,
		.base.base.cra_module	= THIS_MODULE,

		.base.init			= adi_init_skcipher,
		.op.do_one_request	= adi_crypt_cipher_one_req,
		.base.min_keysize		= DES_BLOCK_SIZE,
		.base.max_keysize		= DES_BLOCK_SIZE,
		.base.setkey			= adi_crypt_des_setkey,
		.base.encrypt		= adi_crypt_des_ecb_encrypt,
		.base.decrypt		= adi_crypt_des_ecb_decrypt,
	},
	{
		.base.base.cra_name		= "ecb(des3_ede)",
		.base.base.cra_driver_name	= "adi-ecb-des3",
		.base.base.cra_priority	= 1000,
		.base.base.cra_flags		= CRYPTO_ALG_ASYNC,
		.base.base.cra_blocksize	= DES_BLOCK_SIZE,
		.base.base.cra_ctxsize	= sizeof(struct adi_ctx),
		.base.base.cra_alignmask	= 0,
		.base.base.cra_module	= THIS_MODULE,

		.base.init			= adi_init_skcipher,
		.op.do_one_request	= adi_crypt_cipher_one_req,
		.base.min_keysize		= 3 * DES_BLOCK_SIZE,
		.base.max_keysize		= 3 * DES_BLOCK_SIZE,
		.base.setkey			= adi_crypt_tdes_setkey,
		.base.encrypt		= adi_crypt_tdes_ecb_encrypt,
		.base.decrypt		= adi_crypt_tdes_ecb_decrypt,
	},
};
