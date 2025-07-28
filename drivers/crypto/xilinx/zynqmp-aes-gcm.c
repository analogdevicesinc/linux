// SPDX-License-Identifier: GPL-2.0
/*
 * Xilinx ZynqMP AES Driver.
 * Copyright (C) 2020 - 2022 Xilinx Inc.
 * Copyright (C) 2022 - 2023, Advanced Micro Devices, Inc.
 */

#include <crypto/aes.h>
#include <crypto/engine.h>
#include <crypto/gcm.h>
#include <crypto/internal/aead.h>
#include <crypto/scatterwalk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/firmware/xlnx-zynqmp.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/string.h>

#define ZYNQMP_DMA_BIT_MASK		32U
#define VERSAL_DMA_BIT_MASK		64U
#define ZYNQMP_AES_KEY_SIZE		AES_KEYSIZE_256
#define ZYNQMP_AES_AUTH_SIZE		16U
#define ZYNQMP_AES_BLK_SIZE		1U
#define ZYNQMP_AES_MIN_INPUT_BLK_SIZE	4U
#define ZYNQMP_AES_WORD_LEN		4U
#define VERSAL_AES_QWORD_LEN		16U
#define ZYNQMP_AES_GCM_TAG_MISMATCH_ERR	0x01
#define ZYNQMP_AES_WRONG_KEY_SRC_ERR	0x13
#define ZYNQMP_AES_PUF_NOT_PROGRAMMED	0xE300
#define XILINX_KEY_MAGIC		0x3EA0

enum zynqmp_aead_op {
	ZYNQMP_AES_DECRYPT = 0,
	ZYNQMP_AES_ENCRYPT
};

enum zynqmp_aead_keysrc {
	ZYNQMP_AES_KUP_KEY = 0,
	ZYNQMP_AES_DEV_KEY,
	ZYNQMP_AES_PUF_KEY
};

enum versal_aead_keysrc {
	VERSAL_AES_BBRAM_KEY = 0,
	VERSAL_AES_BBRAM_RED_KEY,
	VERSAL_AES_BH_KEY,
	VERSAL_AES_BH_RED_KEY,
	VERSAL_AES_EFUSE_KEY,
	VERSAL_AES_EFUSE_RED_KEY,
	VERSAL_AES_EFUSE_USER_KEY_0,
	VERSAL_AES_EFUSE_USER_KEY_1,
	VERSAL_AES_EFUSE_USER_RED_KEY_0,
	VERSAL_AES_EFUSE_USER_RED_KEY_1,
	VERSAL_AES_KUP_KEY,
	VERSAL_AES_PUF_KEY,
	VERSAL_AES_USER_KEY_0,
	VERSAL_AES_USER_KEY_1,
	VERSAL_AES_USER_KEY_2,
	VERSAL_AES_USER_KEY_3,
	VERSAL_AES_USER_KEY_4,
	VERSAL_AES_USER_KEY_5,
	VERSAL_AES_USER_KEY_6,
	VERSAL_AES_USER_KEY_7,
	VERSAL_AES_EXPANDED_KEYS,
	VERSAL_AES_ALL_KEYS,
};

enum versal_aead_op {
	VERSAL_AES_ENCRYPT = 0,
	VERSAL_AES_DECRYPT
};

enum versal_aes_keysize {
	AES_KEY_SIZE_128 = 0,
	AES_KEY_SIZE_256 = 2,
};

struct zynqmp_aead_tfm_ctx {
	struct device *dev;
	dma_addr_t key_dma_addr;
	u8 *key;
	u32 keylen;
	u32 authsize;
	u8 keysrc;
	struct crypto_aead *fbk_cipher;
};

struct xilinx_aead_drv_ctx {
	struct aead_engine_alg aead;
	struct device *dev;
	struct crypto_engine *engine;
	u8 keysrc;
	u8 dma_bit_mask;
	int (*aes_aead_cipher)(struct aead_request *areq);
};

struct xilinx_hwkey_info {
	u16 magic;
	u16 type;
} __packed;

struct zynqmp_aead_hw_req {
	u64 src;
	u64 iv;
	u64 key;
	u64 dst;
	u64 size;
	u64 op;
	u64 keysrc;
};

struct zynqmp_aead_req_ctx {
	enum zynqmp_aead_op op;
};

struct versal_init_ops {
	u64 iv;
	u32 op;
	u32 keysrc;
	u32 size;
};

struct versal_in_params {
	u64 in_data_addr;
	u32 size;
	u32 is_last;
};

static int zynqmp_aes_aead_cipher(struct aead_request *req)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct zynqmp_aead_tfm_ctx *tfm_ctx = crypto_aead_ctx(aead);
	struct zynqmp_aead_req_ctx *rq_ctx = aead_request_ctx(req);
	dma_addr_t dma_addr_data, dma_addr_hw_req;
	struct device *dev = tfm_ctx->dev;
	struct zynqmp_aead_hw_req *hwreq;
	unsigned int data_size;
	unsigned int status;
	int ret;
	size_t dma_size;
	void *dmabuf;
	char *kbuf;

	dma_size = req->cryptlen + ZYNQMP_AES_AUTH_SIZE;
	kbuf = kmalloc(dma_size, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	dmabuf = kmalloc(sizeof(*hwreq) + GCM_AES_IV_SIZE, GFP_KERNEL);
	if (!dmabuf) {
		kfree(kbuf);
		return -ENOMEM;
	}
	hwreq = dmabuf;
	data_size = req->cryptlen;
	scatterwalk_map_and_copy(kbuf, req->src, 0, req->cryptlen, 0);
	memcpy(dmabuf + sizeof(struct zynqmp_aead_hw_req), req->iv, GCM_AES_IV_SIZE);
	dma_addr_data = dma_map_single(dev, kbuf, dma_size, DMA_BIDIRECTIONAL);
	if (unlikely(dma_mapping_error(dev, dma_addr_data))) {
		ret = -ENOMEM;
		goto freemem;
	}

	hwreq->src = dma_addr_data;
	hwreq->dst = dma_addr_data;
	hwreq->keysrc = tfm_ctx->keysrc;
	hwreq->op = rq_ctx->op;

	if (hwreq->op == ZYNQMP_AES_ENCRYPT)
		hwreq->size = data_size;
	else
		hwreq->size = data_size - ZYNQMP_AES_AUTH_SIZE;

	if (hwreq->keysrc == ZYNQMP_AES_KUP_KEY)
		hwreq->key = tfm_ctx->key_dma_addr;
	else
		hwreq->key = 0;

	dma_addr_hw_req = dma_map_single(dev, dmabuf, sizeof(struct zynqmp_aead_hw_req) +
					 GCM_AES_IV_SIZE,
					 DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev, dma_addr_hw_req))) {
		ret = -ENOMEM;
		dma_unmap_single(dev, dma_addr_data, dma_size, DMA_BIDIRECTIONAL);
		goto freemem;
	}
	hwreq->iv = dma_addr_hw_req + sizeof(struct zynqmp_aead_hw_req);
	dma_sync_single_for_device(dev, dma_addr_hw_req, sizeof(struct zynqmp_aead_hw_req) +
				   GCM_AES_IV_SIZE, DMA_TO_DEVICE);
	ret = zynqmp_pm_aes_engine(dma_addr_hw_req, &status);
	dma_unmap_single(dev, dma_addr_hw_req, sizeof(struct zynqmp_aead_hw_req) + GCM_AES_IV_SIZE,
			 DMA_TO_DEVICE);
	dma_unmap_single(dev, dma_addr_data, dma_size, DMA_BIDIRECTIONAL);
	if (ret) {
		dev_err(dev, "ERROR: AES PM API failed\n");
	} else if (status) {
		switch (status) {
		case ZYNQMP_AES_GCM_TAG_MISMATCH_ERR:
			ret = -EBADMSG;
			break;
		case ZYNQMP_AES_WRONG_KEY_SRC_ERR:
			ret = -EINVAL;
			dev_err(dev, "ERROR: Wrong KeySrc, enable secure mode\n");
			break;
		case ZYNQMP_AES_PUF_NOT_PROGRAMMED:
			ret = -EINVAL;
			dev_err(dev, "ERROR: PUF is not registered\n");
			break;
		default:
			ret = -EINVAL;
			break;
		}
	} else {
		if (hwreq->op == ZYNQMP_AES_ENCRYPT)
			data_size = data_size + crypto_aead_authsize(aead);
		else
			data_size = data_size - ZYNQMP_AES_AUTH_SIZE;

		sg_copy_from_buffer(req->dst, sg_nents(req->dst),
				    kbuf, data_size);
		ret = 0;
	}

freemem:
	memzero_explicit(kbuf, dma_size);
	kfree(kbuf);
	memzero_explicit(dmabuf, sizeof(struct zynqmp_aead_hw_req) + GCM_AES_IV_SIZE);
	kfree(dmabuf);

	return ret;
}

static int versal_aes_aead_cipher(struct aead_request *req)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct zynqmp_aead_tfm_ctx *tfm_ctx = crypto_aead_ctx(aead);
	struct zynqmp_aead_req_ctx *rq_ctx = aead_request_ctx(req);
	dma_addr_t dma_addr_data, dma_addr_hw_req, dma_addr_in;
	u32 total_len = req->assoclen + req->cryptlen;
	struct device *dev = tfm_ctx->dev;
	struct versal_init_ops *hwreq;
	struct versal_in_params *in;
	u32 gcm_offset, out_len;
	size_t dmabuf_size;
	size_t kbuf_size;
	void *dmabuf;
	char *kbuf;
	int ret;

	kbuf_size = total_len + ZYNQMP_AES_AUTH_SIZE;
	kbuf = kmalloc(kbuf_size, GFP_KERNEL);
	if (unlikely(!kbuf)) {
		ret = -ENOMEM;
		goto err;
	}
	dmabuf_size = sizeof(struct versal_init_ops) +
		      sizeof(struct versal_in_params) +
		      GCM_AES_IV_SIZE;
	dmabuf = kmalloc(dmabuf_size, GFP_KERNEL);
	if (unlikely(!dmabuf)) {
		ret = -ENOMEM;
		goto buf1_free;
	}

	dma_addr_hw_req = dma_map_single(dev, dmabuf, dmabuf_size, DMA_BIDIRECTIONAL);
	if (unlikely(dma_mapping_error(dev, dma_addr_hw_req))) {
		ret = -ENOMEM;
		goto buf2_free;
	}
	scatterwalk_map_and_copy(kbuf, req->src, 0, total_len, 0);
	dma_addr_data = dma_map_single(dev, kbuf, kbuf_size, DMA_BIDIRECTIONAL);
	if (unlikely(dma_mapping_error(dev, dma_addr_data))) {
		dma_unmap_single(dev, dma_addr_hw_req, dmabuf_size, DMA_BIDIRECTIONAL);
		ret = -ENOMEM;
		goto buf2_free;
	}
	hwreq = dmabuf;
	in = dmabuf + sizeof(struct versal_init_ops);
	memcpy(dmabuf + sizeof(struct versal_init_ops) +
	       sizeof(struct versal_in_params), req->iv, GCM_AES_IV_SIZE);
	hwreq->iv = dma_addr_hw_req + sizeof(struct versal_init_ops) +
		    sizeof(struct versal_in_params);
	hwreq->keysrc = tfm_ctx->keysrc;
	dma_addr_in = dma_addr_hw_req + sizeof(struct versal_init_ops);
	if (rq_ctx->op == ZYNQMP_AES_ENCRYPT) {
		hwreq->op = VERSAL_AES_ENCRYPT;
		out_len = total_len + crypto_aead_authsize(aead);
		in->size = req->cryptlen;
	} else {
		hwreq->op = VERSAL_AES_DECRYPT;
		out_len = total_len - ZYNQMP_AES_AUTH_SIZE;
		in->size = req->cryptlen - ZYNQMP_AES_AUTH_SIZE;
	}

	if (tfm_ctx->keylen == XSECURE_AES_KEY_SIZE_128)
		hwreq->size = AES_KEY_SIZE_128;
	else
		hwreq->size = AES_KEY_SIZE_256;

	/* Request aes key write for volatile user keys */
	if (hwreq->keysrc >= VERSAL_AES_USER_KEY_0 && hwreq->keysrc <= VERSAL_AES_USER_KEY_7) {
		ret = versal_pm_aes_key_write(hwreq->size, hwreq->keysrc,
					      tfm_ctx->key_dma_addr);
		if (ret)
			goto unmap;
	}

	in->in_data_addr = dma_addr_data + req->assoclen;
	in->is_last = 1;
	gcm_offset = req->assoclen + in->size;
	dma_sync_single_for_device(dev, dma_addr_hw_req, dmabuf_size, DMA_BIDIRECTIONAL);
	ret = versal_pm_aes_op_init(dma_addr_hw_req);
	if (ret)
		goto clearkey;

	if (req->assoclen > 0) {
		/* Currently GMAC is OFF by default */
		ret = versal_pm_aes_update_aad(dma_addr_data, req->assoclen);
		if (ret)
			goto clearkey;
	}
	if (rq_ctx->op == ZYNQMP_AES_ENCRYPT) {
		ret = versal_pm_aes_enc_update(dma_addr_in,
					       dma_addr_data + req->assoclen);
		if (ret)
			goto clearkey;

		ret = versal_pm_aes_enc_final(dma_addr_data + gcm_offset);
		if (ret)
			goto clearkey;
	} else {
		ret = versal_pm_aes_dec_update(dma_addr_in,
					       dma_addr_data + req->assoclen);
		if (ret)
			goto clearkey;

		ret = versal_pm_aes_dec_final(dma_addr_data + gcm_offset);
		if (ret) {
			ret = -EBADMSG;
			goto clearkey;
		}
	}
	dma_unmap_single(dev, dma_addr_data, kbuf_size, DMA_BIDIRECTIONAL);
	dma_unmap_single(dev, dma_addr_hw_req, dmabuf_size, DMA_BIDIRECTIONAL);
	sg_copy_from_buffer(req->dst, sg_nents(req->dst),
			    kbuf, out_len);
	dma_addr_data = 0;
	dma_addr_hw_req = 0;

clearkey:
	if (hwreq->keysrc >= VERSAL_AES_USER_KEY_0 && hwreq->keysrc <= VERSAL_AES_USER_KEY_7)
		versal_pm_aes_key_zero(hwreq->keysrc);
unmap:
	if (unlikely(dma_addr_data))
		dma_unmap_single(dev, dma_addr_data, kbuf_size, DMA_BIDIRECTIONAL);
	if (unlikely(dma_addr_hw_req))
		dma_unmap_single(dev, dma_addr_hw_req, dmabuf_size, DMA_BIDIRECTIONAL);
buf2_free:
	memzero_explicit(dmabuf, dmabuf_size);
	kfree(dmabuf);
buf1_free:
	memzero_explicit(kbuf, kbuf_size);
	kfree(kbuf);
err:
	return ret;
}

static int zynqmp_fallback_check(struct zynqmp_aead_tfm_ctx *tfm_ctx,
				 struct aead_request *req)
{
	struct zynqmp_aead_req_ctx *rq_ctx = aead_request_ctx(req);

	if (tfm_ctx->authsize != ZYNQMP_AES_AUTH_SIZE && rq_ctx->op == ZYNQMP_AES_DECRYPT)
		return 1;

	if (req->assoclen != 0 ||
	    req->cryptlen < ZYNQMP_AES_MIN_INPUT_BLK_SIZE)
		return 1;
	if (tfm_ctx->keylen == AES_KEYSIZE_128 ||
	    tfm_ctx->keylen == AES_KEYSIZE_192)
		return 1;

	if ((req->cryptlen % ZYNQMP_AES_WORD_LEN) != 0)
		return 1;

	if (rq_ctx->op == ZYNQMP_AES_DECRYPT &&
	    req->cryptlen <= ZYNQMP_AES_AUTH_SIZE)
		return 1;

	return 0;
}

static int versal_fallback_check(struct zynqmp_aead_tfm_ctx *tfm_ctx,
				 struct aead_request *req)
{
	struct zynqmp_aead_req_ctx *rq_ctx = aead_request_ctx(req);

	if (tfm_ctx->authsize != ZYNQMP_AES_AUTH_SIZE && rq_ctx->op == ZYNQMP_AES_DECRYPT)
		return 1;

	if (tfm_ctx->keylen == AES_KEYSIZE_192)
		return 1;

	if (req->cryptlen < ZYNQMP_AES_MIN_INPUT_BLK_SIZE ||
	    req->cryptlen % ZYNQMP_AES_WORD_LEN ||
	    req->assoclen % VERSAL_AES_QWORD_LEN)
		return 1;

	if (rq_ctx->op == ZYNQMP_AES_DECRYPT &&
	    req->cryptlen <= ZYNQMP_AES_AUTH_SIZE)
		return 1;

	return 0;
}

static int handle_aes_req(struct crypto_engine *engine, void *req)
{
	struct aead_request *areq =
				container_of(req, struct aead_request, base);
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct aead_alg *alg = crypto_aead_alg(aead);
	struct xilinx_aead_drv_ctx *drv_ctx;
	int err;

	drv_ctx = container_of(alg, struct xilinx_aead_drv_ctx, aead.base);
	err = drv_ctx->aes_aead_cipher(areq);
	local_bh_disable();
	crypto_finalize_aead_request(engine, areq, err);
	local_bh_enable();

	return 0;
}

static int zynqmp_aes_aead_setkey(struct crypto_aead *aead, const u8 *key,
				  unsigned int keylen)
{
	struct crypto_tfm *tfm = crypto_aead_tfm(aead);
	struct zynqmp_aead_tfm_ctx *tfm_ctx = crypto_tfm_ctx(tfm);
	struct xilinx_hwkey_info hwkey;
	unsigned char keysrc;
	int err;

	if (keylen == sizeof(struct xilinx_hwkey_info)) {
		memcpy(&hwkey, key, sizeof(struct xilinx_hwkey_info));
		if (hwkey.magic != XILINX_KEY_MAGIC)
			return -EINVAL;
		keysrc = hwkey.type;
		if (keysrc == ZYNQMP_AES_KUP_KEY ||
		    keysrc == ZYNQMP_AES_DEV_KEY ||
		    keysrc == ZYNQMP_AES_PUF_KEY) {
			tfm_ctx->keysrc = keysrc;
			tfm_ctx->keylen = sizeof(struct xilinx_hwkey_info);
			return 0;
		}
		return -EINVAL;
	}

	if (keylen == ZYNQMP_AES_KEY_SIZE && tfm_ctx->keysrc == ZYNQMP_AES_KUP_KEY) {
		tfm_ctx->keylen = keylen;
		memcpy(tfm_ctx->key, key, keylen);
		dma_sync_single_for_device(tfm_ctx->dev, tfm_ctx->key_dma_addr,
					   ZYNQMP_AES_KEY_SIZE,
					   DMA_TO_DEVICE);
	} else if (tfm_ctx->keysrc != ZYNQMP_AES_KUP_KEY) {
		return -EINVAL;
	}

	tfm_ctx->fbk_cipher->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
	tfm_ctx->fbk_cipher->base.crt_flags |= (aead->base.crt_flags &
					CRYPTO_TFM_REQ_MASK);

	err = crypto_aead_setkey(tfm_ctx->fbk_cipher, key, keylen);
	if (!err)
		tfm_ctx->keylen = keylen;

	return err;
}

static int versal_aes_aead_setkey(struct crypto_aead *aead, const u8 *key,
				  unsigned int keylen)
{
	struct crypto_tfm *tfm = crypto_aead_tfm(aead);
	struct zynqmp_aead_tfm_ctx *tfm_ctx = crypto_tfm_ctx(tfm);
	struct xilinx_hwkey_info hwkey;
	unsigned char keysrc;
	int err;

	if (keylen == sizeof(struct xilinx_hwkey_info)) {
		memcpy(&hwkey, key, sizeof(struct xilinx_hwkey_info));
		if (hwkey.magic != XILINX_KEY_MAGIC)
			return -EINVAL;

		keysrc = hwkey.type;
		if ((keysrc >= VERSAL_AES_EFUSE_USER_KEY_0 &&
		     keysrc  <= VERSAL_AES_USER_KEY_7) &&
		     keysrc != VERSAL_AES_KUP_KEY) {
			tfm_ctx->keysrc = keysrc;
			tfm_ctx->keylen = sizeof(struct xilinx_hwkey_info);
			return 0;
		}
		return -EINVAL;
	}
	if (tfm_ctx->keysrc < VERSAL_AES_USER_KEY_0 || tfm_ctx->keysrc > VERSAL_AES_USER_KEY_7)
		return -EINVAL;
	if (keylen == XSECURE_AES_KEY_SIZE_256 || keylen == XSECURE_AES_KEY_SIZE_128) {
		tfm_ctx->keylen = keylen;
		memcpy(tfm_ctx->key, key, keylen);
		dma_sync_single_for_device(tfm_ctx->dev, tfm_ctx->key_dma_addr,
					   ZYNQMP_AES_KEY_SIZE,
					   DMA_TO_DEVICE);
	}

	tfm_ctx->fbk_cipher->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
	tfm_ctx->fbk_cipher->base.crt_flags |= (aead->base.crt_flags &
						CRYPTO_TFM_REQ_MASK);
	err = crypto_aead_setkey(tfm_ctx->fbk_cipher, key, keylen);
	if (!err)
		tfm_ctx->keylen = keylen;

	return err;
}

static int zynqmp_aes_aead_setauthsize(struct crypto_aead *aead,
				       unsigned int authsize)
{
	struct crypto_tfm *tfm = crypto_aead_tfm(aead);
	struct zynqmp_aead_tfm_ctx *tfm_ctx = crypto_tfm_ctx(tfm);

	tfm_ctx->authsize = authsize;
	return crypto_aead_setauthsize(tfm_ctx->fbk_cipher, authsize);
}

static int zynqmp_aes_aead_encrypt(struct aead_request *req)
{
	struct zynqmp_aead_req_ctx *rq_ctx = aead_request_ctx(req);
	struct aead_request *subreq = aead_request_ctx(req);
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct zynqmp_aead_tfm_ctx *tfm_ctx = crypto_aead_ctx(aead);
	struct aead_alg *alg = crypto_aead_alg(aead);
	struct xilinx_aead_drv_ctx *drv_ctx;
	int err;

	drv_ctx = container_of(alg, struct xilinx_aead_drv_ctx, aead.base);
	if (tfm_ctx->keysrc == ZYNQMP_AES_KUP_KEY  &&
	    tfm_ctx->keylen == sizeof(struct xilinx_hwkey_info))
		return -EINVAL;

	rq_ctx->op = ZYNQMP_AES_ENCRYPT;
	err = zynqmp_fallback_check(tfm_ctx, req);
	if (err && tfm_ctx->keysrc != ZYNQMP_AES_KUP_KEY)
		return -EOPNOTSUPP;

	if (err) {
		aead_request_set_tfm(subreq, tfm_ctx->fbk_cipher);
		aead_request_set_callback(subreq, req->base.flags,
					  NULL, NULL);
		aead_request_set_crypt(subreq, req->src, req->dst,
				       req->cryptlen, req->iv);
		aead_request_set_ad(subreq, req->assoclen);
		if (rq_ctx->op == ZYNQMP_AES_ENCRYPT)
			err = crypto_aead_encrypt(subreq);
		else
			err = crypto_aead_decrypt(subreq);
		return err;
	}

	return crypto_transfer_aead_request_to_engine(drv_ctx->engine, req);
}

static int versal_aes_aead_encrypt(struct aead_request *req)
{
	struct zynqmp_aead_req_ctx *rq_ctx = aead_request_ctx(req);
	struct aead_request *subreq = aead_request_ctx(req);
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct zynqmp_aead_tfm_ctx *tfm_ctx = crypto_aead_ctx(aead);
	struct aead_alg *alg = crypto_aead_alg(aead);
	struct xilinx_aead_drv_ctx *drv_ctx;
	int err;

	drv_ctx = container_of(alg, struct xilinx_aead_drv_ctx, aead.base);
	rq_ctx->op = ZYNQMP_AES_ENCRYPT;
	if (tfm_ctx->keysrc >= VERSAL_AES_USER_KEY_0 &&
	    tfm_ctx->keysrc <= VERSAL_AES_USER_KEY_7 &&
	    tfm_ctx->keylen == sizeof(struct xilinx_hwkey_info))
		return -EINVAL;
	err = versal_fallback_check(tfm_ctx, req);
	if (err && (tfm_ctx->keysrc < VERSAL_AES_USER_KEY_0 ||
		    tfm_ctx->keysrc > VERSAL_AES_USER_KEY_7))
		return -EOPNOTSUPP;
	if (err) {
		aead_request_set_tfm(subreq, tfm_ctx->fbk_cipher);
		aead_request_set_callback(subreq, req->base.flags,
					  NULL, NULL);
		aead_request_set_crypt(subreq, req->src, req->dst,
				       req->cryptlen, req->iv);
		aead_request_set_ad(subreq, req->assoclen);
		if (rq_ctx->op == ZYNQMP_AES_ENCRYPT)
			err = crypto_aead_encrypt(subreq);
		else
			err = crypto_aead_decrypt(subreq);
		return err;
	}

	return crypto_transfer_aead_request_to_engine(drv_ctx->engine, req);
}

static int zynqmp_aes_aead_decrypt(struct aead_request *req)
{
	struct zynqmp_aead_req_ctx *rq_ctx = aead_request_ctx(req);
	struct aead_request *subreq = aead_request_ctx(req);
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct zynqmp_aead_tfm_ctx *tfm_ctx = crypto_aead_ctx(aead);
	struct aead_alg *alg = crypto_aead_alg(aead);
	struct xilinx_aead_drv_ctx *drv_ctx;
	int err;

	drv_ctx = container_of(alg, struct xilinx_aead_drv_ctx, aead.base);
	rq_ctx->op = ZYNQMP_AES_DECRYPT;
	if (tfm_ctx->keysrc == ZYNQMP_AES_KUP_KEY  &&
	    tfm_ctx->keylen == sizeof(struct xilinx_hwkey_info))
		return -EINVAL;
	err = zynqmp_fallback_check(tfm_ctx, req);
	if (err && tfm_ctx->keysrc != ZYNQMP_AES_KUP_KEY)
		return -EOPNOTSUPP;

	if (err) {
		aead_request_set_tfm(subreq, tfm_ctx->fbk_cipher);
		aead_request_set_callback(subreq, req->base.flags,
					  NULL, NULL);
		aead_request_set_crypt(subreq, req->src, req->dst,
				       req->cryptlen, req->iv);
		aead_request_set_ad(subreq, req->assoclen);
		if (rq_ctx->op == ZYNQMP_AES_ENCRYPT)
			err = crypto_aead_encrypt(subreq);
		else
			err = crypto_aead_decrypt(subreq);
		return err;
	}

	return crypto_transfer_aead_request_to_engine(drv_ctx->engine, req);
}

static int versal_aes_aead_decrypt(struct aead_request *req)
{
	struct zynqmp_aead_req_ctx *rq_ctx = aead_request_ctx(req);
	struct aead_request *subreq = aead_request_ctx(req);
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct zynqmp_aead_tfm_ctx *tfm_ctx = crypto_aead_ctx(aead);
	struct aead_alg *alg = crypto_aead_alg(aead);
	struct xilinx_aead_drv_ctx *drv_ctx;
	int err;

	drv_ctx = container_of(alg, struct xilinx_aead_drv_ctx, aead.base);
	rq_ctx->op = ZYNQMP_AES_DECRYPT;
	if (tfm_ctx->keysrc >= VERSAL_AES_USER_KEY_0 &&
	    tfm_ctx->keysrc <= VERSAL_AES_USER_KEY_7 &&
	    tfm_ctx->keylen == sizeof(struct xilinx_hwkey_info))
		return -EINVAL;

	err = versal_fallback_check(tfm_ctx, req);
	if (err &&
	    tfm_ctx->keysrc < VERSAL_AES_USER_KEY_0 &&
	    tfm_ctx->keysrc > VERSAL_AES_USER_KEY_7)
		return -EOPNOTSUPP;
	if (err) {
		aead_request_set_tfm(subreq, tfm_ctx->fbk_cipher);
		aead_request_set_callback(subreq, req->base.flags,
					  NULL, NULL);
		aead_request_set_crypt(subreq, req->src, req->dst,
				       req->cryptlen, req->iv);
		aead_request_set_ad(subreq, req->assoclen);
		if (rq_ctx->op == ZYNQMP_AES_ENCRYPT)
			err = crypto_aead_encrypt(subreq);
		else
			err = crypto_aead_decrypt(subreq);
		return err;
	}

	return crypto_transfer_aead_request_to_engine(drv_ctx->engine, req);
}

static int aes_aead_init(struct crypto_aead *aead)
{
	struct crypto_tfm *tfm = crypto_aead_tfm(aead);
	struct zynqmp_aead_tfm_ctx *tfm_ctx = crypto_tfm_ctx(tfm);
	struct xilinx_aead_drv_ctx *drv_ctx;
	struct aead_alg *alg = crypto_aead_alg(aead);

	drv_ctx = container_of(alg, struct xilinx_aead_drv_ctx, aead.base);
	tfm_ctx->dev = drv_ctx->dev;
	tfm_ctx->keylen = 0;
	tfm_ctx->keysrc = drv_ctx->keysrc;

	tfm_ctx->fbk_cipher = crypto_alloc_aead(drv_ctx->aead.base.base.cra_name,
						0,
						CRYPTO_ALG_NEED_FALLBACK);

	if (IS_ERR(tfm_ctx->fbk_cipher)) {
		pr_err("%s() Error: failed to allocate fallback for %s\n",
		       __func__, drv_ctx->aead.base.base.cra_name);
		return PTR_ERR(tfm_ctx->fbk_cipher);
	}
	tfm_ctx->key = kmalloc(ZYNQMP_AES_KEY_SIZE, GFP_KERNEL);
	if (!tfm_ctx->key) {
		crypto_free_aead(tfm_ctx->fbk_cipher);
		return -ENOMEM;
	}
	tfm_ctx->key_dma_addr = dma_map_single(tfm_ctx->dev, tfm_ctx->key,
					       ZYNQMP_AES_KEY_SIZE,
					       DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(tfm_ctx->dev, tfm_ctx->key_dma_addr))) {
		kfree(tfm_ctx->key);
		crypto_free_aead(tfm_ctx->fbk_cipher);
		tfm_ctx->fbk_cipher = NULL;
		return -ENOMEM;
	}
	crypto_aead_set_reqsize(aead,
				max(sizeof(struct zynqmp_aead_req_ctx),
				    sizeof(struct aead_request) +
				    crypto_aead_reqsize(tfm_ctx->fbk_cipher)));
	return 0;
}

static void zynqmp_aes_aead_exit(struct crypto_aead *aead)
{
	struct crypto_tfm *tfm = crypto_aead_tfm(aead);
	struct zynqmp_aead_tfm_ctx *tfm_ctx = crypto_tfm_ctx(tfm);

	dma_unmap_single(tfm_ctx->dev, tfm_ctx->key_dma_addr, ZYNQMP_AES_KEY_SIZE, DMA_TO_DEVICE);
	kfree(tfm_ctx->key);
	if (tfm_ctx->fbk_cipher) {
		crypto_free_aead(tfm_ctx->fbk_cipher);
		tfm_ctx->fbk_cipher = NULL;
	}
	memzero_explicit(tfm_ctx, sizeof(struct zynqmp_aead_tfm_ctx));
}

static struct xilinx_aead_drv_ctx zynqmp_aes_drv_ctx = {
	.aes_aead_cipher = zynqmp_aes_aead_cipher,
	.keysrc = ZYNQMP_AES_KUP_KEY,
	.aead.base = {
		.setkey		= zynqmp_aes_aead_setkey,
		.setauthsize	= zynqmp_aes_aead_setauthsize,
		.encrypt	= zynqmp_aes_aead_encrypt,
		.decrypt	= zynqmp_aes_aead_decrypt,
		.init		= aes_aead_init,
		.exit		= zynqmp_aes_aead_exit,
		.ivsize		= GCM_AES_IV_SIZE,
		.maxauthsize	= ZYNQMP_AES_AUTH_SIZE,
		.base = {
		.cra_name		= "gcm(aes)",
		.cra_driver_name	= "zynqmp-aes-gcm",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_AEAD |
					  CRYPTO_ALG_ASYNC |
					  CRYPTO_ALG_ALLOCATES_MEMORY |
					  CRYPTO_ALG_KERN_DRIVER_ONLY |
					  CRYPTO_ALG_NEED_FALLBACK,
		.cra_blocksize		= ZYNQMP_AES_BLK_SIZE,
		.cra_ctxsize		= sizeof(struct zynqmp_aead_tfm_ctx),
		.cra_module		= THIS_MODULE,
		}
	},
	.aead.op = {
		.do_one_request = handle_aes_req,
	},
	.dma_bit_mask = ZYNQMP_DMA_BIT_MASK,
};

static struct xilinx_aead_drv_ctx versal_aes_drv_ctx = {
	.aes_aead_cipher	= versal_aes_aead_cipher,
	.keysrc = VERSAL_AES_USER_KEY_0,
	.aead.base = {
		.setkey		= versal_aes_aead_setkey,
		.setauthsize	= zynqmp_aes_aead_setauthsize,
		.encrypt	= versal_aes_aead_encrypt,
		.decrypt	= versal_aes_aead_decrypt,
		.init		= aes_aead_init,
		.exit		= zynqmp_aes_aead_exit,
		.ivsize		= GCM_AES_IV_SIZE,
		.maxauthsize	= ZYNQMP_AES_AUTH_SIZE,
		.base = {
		.cra_name		= "gcm(aes)",
		.cra_driver_name	= "versal-aes-gcm",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_AEAD |
					  CRYPTO_ALG_ASYNC |
					  CRYPTO_ALG_ALLOCATES_MEMORY |
					  CRYPTO_ALG_KERN_DRIVER_ONLY |
					  CRYPTO_ALG_NEED_FALLBACK,
		.cra_blocksize		= ZYNQMP_AES_BLK_SIZE,
		.cra_ctxsize		= sizeof(struct zynqmp_aead_tfm_ctx),
		.cra_module		= THIS_MODULE,
		}
	},
	.aead.op = {
		.do_one_request = handle_aes_req,
	},
	.dma_bit_mask = VERSAL_DMA_BIT_MASK,
};

static struct xlnx_feature aes_feature_map[] = {
	{
		.family = ZYNQMP_FAMILY_CODE,
		.subfamily = ALL_SUB_FAMILY_CODE,
		.feature_id = PM_SECURE_AES,
		.data = &zynqmp_aes_drv_ctx,
	},
	{
		.family = VERSAL_FAMILY_CODE,
		.subfamily = VERSAL_SUB_FAMILY_CODE,
		.feature_id = XSECURE_API_AES_OP_INIT,
		.data = &versal_aes_drv_ctx,
	},
	{ /* sentinel */ }
};

static int zynqmp_aes_aead_probe(struct platform_device *pdev)
{
	struct xilinx_aead_drv_ctx *aes_drv_ctx;
	struct device *dev = &pdev->dev;
	int err;

	/* Verify the hardware is present */
	aes_drv_ctx = xlnx_get_crypto_dev_data(aes_feature_map);
	if (IS_ERR(aes_drv_ctx)) {
		dev_err(dev, "AES is not supported on the platform\n");
		return PTR_ERR(aes_drv_ctx);
	}

	/* ZynqMP AES driver supports only one instance */
	if (!aes_drv_ctx->dev)
		aes_drv_ctx->dev = dev;
	else
		return -ENODEV;

	platform_set_drvdata(pdev, aes_drv_ctx);

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(aes_drv_ctx->dma_bit_mask));
	if (err < 0) {
		dev_err(dev, "No usable DMA configuration\n");
		return err;
	}

	aes_drv_ctx->engine = crypto_engine_alloc_init(dev, 1);
	if (!aes_drv_ctx->engine) {
		dev_err(dev, "Cannot alloc AES engine\n");
		err = -ENOMEM;
		goto err_engine;
	}

	err = crypto_engine_start(aes_drv_ctx->engine);
	if (err) {
		dev_err(dev, "Cannot start AES engine\n");
		goto err_engine;
	}

	err = crypto_engine_register_aead(&aes_drv_ctx->aead);
	if (err < 0) {
		dev_err(dev, "Failed to register AEAD alg.\n");
		goto err_engine;
	}
	return 0;

err_engine:
	if (aes_drv_ctx->engine)
		crypto_engine_exit(aes_drv_ctx->engine);

	return err;
}

static void zynqmp_aes_aead_remove(struct platform_device *pdev)
{
	struct xilinx_aead_drv_ctx *aes_drv_ctx;

	aes_drv_ctx = platform_get_drvdata(pdev);

	crypto_engine_exit(aes_drv_ctx->engine);
	crypto_engine_unregister_aead(&aes_drv_ctx->aead);
}

static struct platform_driver zynqmp_aes_driver = {
	.probe	= zynqmp_aes_aead_probe,
	.remove_new = zynqmp_aes_aead_remove,
	.driver = {
		.name		= "zynqmp-aes",
	},
};

static struct platform_device *platform_dev;

static int __init aes_driver_init(void)
{
	int ret;

	ret = platform_driver_register(&zynqmp_aes_driver);
	if (ret)
		return ret;

	platform_dev = platform_device_register_simple(zynqmp_aes_driver.driver.name,
						       0, NULL, 0);
	if (IS_ERR(platform_dev)) {
		ret = PTR_ERR(platform_dev);
		platform_driver_unregister(&zynqmp_aes_driver);
	}

	return ret;
}

static void __exit aes_driver_exit(void)
{
	platform_device_unregister(platform_dev);
	platform_driver_unregister(&zynqmp_aes_driver);
}

module_init(aes_driver_init);
module_exit(aes_driver_exit);

MODULE_DESCRIPTION("Xilinx ZynqMP AES Driver");
MODULE_LICENSE("GPL");
