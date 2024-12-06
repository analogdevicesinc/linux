/*
 * Copyright 2013 Freescale
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 */

#include <crypto/internal/aead.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/skcipher.h>
#include <crypto/authenc.h>
#include <crypto/null.h>
#include <crypto/scatterwalk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/rtnetlink.h>

struct tls_instance_ctx {
	struct crypto_ahash_spawn auth;
	struct crypto_skcipher_spawn enc;
};

struct crypto_tls_ctx {
	unsigned int reqoff;
	struct crypto_ahash *auth;
	struct crypto_skcipher *enc;
	struct crypto_sync_skcipher *null;
};

struct tls_request_ctx {
	/*
	 * cryptlen holds the payload length in the case of encryption or
	 * payload_len + icv_len + padding_len in case of decryption
	 */
	unsigned int cryptlen;
	/* working space for partial results */
	struct scatterlist tmp[2];
	struct scatterlist cipher[2];
	struct scatterlist dst[2];
	char tail[];
};

struct async_op {
	struct completion completion;
	int err;
};

static void tls_async_op_done(struct crypto_async_request *req, int err)
{
	struct async_op *areq = req->data;

	if (err == -EINPROGRESS)
		return;

	areq->err = err;
	complete(&areq->completion);
}

static int crypto_tls_setkey(struct crypto_aead *tls, const u8 *key,
			     unsigned int keylen)
{
	struct crypto_tls_ctx *ctx = crypto_aead_ctx(tls);
	struct crypto_ahash *auth = ctx->auth;
	struct crypto_skcipher *enc = ctx->enc;
	struct crypto_authenc_keys keys;
	int err = -EINVAL;

	if (crypto_authenc_extractkeys(&keys, key, keylen) != 0)
		goto out;

	crypto_ahash_clear_flags(auth, CRYPTO_TFM_REQ_MASK);
	crypto_ahash_set_flags(auth, crypto_aead_get_flags(tls) &
				    CRYPTO_TFM_REQ_MASK);
	err = crypto_ahash_setkey(auth, keys.authkey, keys.authkeylen);
	if (err)
		goto out;

	crypto_skcipher_clear_flags(enc, CRYPTO_TFM_REQ_MASK);
	crypto_skcipher_set_flags(enc, crypto_aead_get_flags(tls) &
					 CRYPTO_TFM_REQ_MASK);
	err = crypto_skcipher_setkey(enc, keys.enckey, keys.enckeylen);

out:
	memzero_explicit(&keys, sizeof(keys));
	return err;
}

/**
 * crypto_tls_genicv - Calculate hmac digest for a TLS record
 * @hash:	(output) buffer to save the digest into
 * @src:	(input) scatterlist with the assoc and payload data
 * @srclen:	(input) size of the source buffer (assoclen + cryptlen)
 * @req:	(input) aead request
 **/
static int crypto_tls_genicv(u8 *hash, struct scatterlist *src,
			     unsigned int srclen, struct aead_request *req)
{
	struct crypto_aead *tls = crypto_aead_reqtfm(req);
	struct crypto_tls_ctx *ctx = crypto_aead_ctx(tls);
	struct tls_request_ctx *treq_ctx = aead_request_ctx(req);
	struct async_op ahash_op;
	struct ahash_request *ahreq = (void *)(treq_ctx->tail + ctx->reqoff);
	unsigned int flags = CRYPTO_TFM_REQ_MAY_SLEEP;
	int err = -EBADMSG;

	 /* Bail out if the request assoc len is 0 */
	if (!req->assoclen)
		return err;

	init_completion(&ahash_op.completion);

	/* the hash transform to be executed comes from the original request */
	ahash_request_set_tfm(ahreq, ctx->auth);
	/* prepare the hash request with input data and result pointer */
	ahash_request_set_crypt(ahreq, src, hash, srclen);
	/* set the notifier for when the async hash function returns */
	ahash_request_set_callback(ahreq, aead_request_flags(req) & flags,
				   tls_async_op_done, &ahash_op);

	/* Calculate the digest on the given data. The result is put in hash */
	err = crypto_ahash_digest(ahreq);
	if (err == -EINPROGRESS) {
		err = wait_for_completion_interruptible(&ahash_op.completion);
		if (!err)
			err = ahash_op.err;
	}

	return err;
}

/**
 * crypto_tls_gen_padicv - Calculate and pad hmac digest for a TLS record
 * @hash:	(output) buffer to save the digest and padding into
 * @phashlen:	(output) the size of digest + padding
 * @req:	(input) aead request
 **/
static int crypto_tls_gen_padicv(u8 *hash, unsigned int *phashlen,
				 struct aead_request *req)
{
	struct crypto_aead *tls = crypto_aead_reqtfm(req);
	unsigned int hash_size = crypto_aead_authsize(tls);
	unsigned int block_size = crypto_aead_blocksize(tls);
	unsigned int srclen = req->cryptlen + hash_size;
	unsigned int icvlen = req->cryptlen + req->assoclen;
	unsigned int padlen;
	int err;

	err = crypto_tls_genicv(hash, req->src, icvlen, req);
	if (err)
		goto out;

	/* add padding after digest */
	padlen = block_size - (srclen % block_size);
	memset(hash + hash_size, padlen - 1, padlen);

	*phashlen = hash_size + padlen;
out:
	return err;
}

static int crypto_tls_copy_data(struct aead_request *req,
				struct scatterlist *src,
				struct scatterlist *dst,
				unsigned int len)
{
	struct crypto_aead *tls = crypto_aead_reqtfm(req);
	struct crypto_tls_ctx *ctx = crypto_aead_ctx(tls);
	SYNC_SKCIPHER_REQUEST_ON_STACK(skreq, ctx->null);

	skcipher_request_set_sync_tfm(skreq, ctx->null);
	skcipher_request_set_callback(skreq, aead_request_flags(req),
				      NULL, NULL);
	skcipher_request_set_crypt(skreq, src, dst, len, NULL);

	return crypto_skcipher_encrypt(skreq);
}

static int crypto_tls_encrypt(struct aead_request *req)
{
	struct crypto_aead *tls = crypto_aead_reqtfm(req);
	struct crypto_tls_ctx *ctx = crypto_aead_ctx(tls);
	struct tls_request_ctx *treq_ctx = aead_request_ctx(req);
	struct skcipher_request *skreq;
	struct scatterlist *cipher = treq_ctx->cipher;
	struct scatterlist *tmp = treq_ctx->tmp;
	struct scatterlist *sg, *src, *dst;
	unsigned int cryptlen, phashlen;
	u8 *hash = treq_ctx->tail;
	int err;

	/*
	 * The hash result is saved at the beginning of the tls request ctx
	 * and is aligned as required by the hash transform. Enough space was
	 * allocated in crypto_tls_init_tfm to accommodate the difference. The
	 * requests themselves start later at treq_ctx->tail + ctx->reqoff so
	 * the result is not overwritten by the second (cipher) request.
	 */
	hash = (u8 *)ALIGN((unsigned long)hash +
			   crypto_ahash_alignmask(ctx->auth),
			   crypto_ahash_alignmask(ctx->auth) + 1);

	/*
	 * STEP 1: create ICV together with necessary padding
	 */
	err = crypto_tls_gen_padicv(hash, &phashlen, req);
	if (err)
		return err;

	/*
	 * STEP 2: Hash and padding are combined with the payload
	 * depending on the form it arrives. Scatter tables must have at least
	 * one page of data before chaining with another table and can't have
	 * an empty data page. The following code addresses these requirements.
	 *
	 * If the payload is empty, only the hash is encrypted, otherwise the
	 * payload scatterlist is merged with the hash. A special merging case
	 * is when the payload has only one page of data. In that case the
	 * payload page is moved to another scatterlist and prepared there for
	 * encryption.
	 */
	if (req->cryptlen) {
		src = scatterwalk_ffwd(tmp, req->src, req->assoclen);

		sg_init_table(cipher, 2);
		sg_set_buf(cipher + 1, hash, phashlen);

		if (sg_is_last(src)) {
			sg_set_page(cipher, sg_page(src), req->cryptlen,
				    src->offset);
			src = cipher;
		} else {
			unsigned int rem_len = req->cryptlen;

			for (sg = src; rem_len > sg->length; sg = sg_next(sg))
				rem_len -= min(rem_len, sg->length);

			sg_set_page(cipher, sg_page(sg), rem_len, sg->offset);
			sg_chain(sg, 1, cipher);
		}
	} else {
		sg_init_one(cipher, hash, phashlen);
		src = cipher;
	}

	/**
	 * If src != dst copy the associated data from source to destination.
	 * In both cases fast-forward passed the associated data in the dest.
	 */
	if (req->src != req->dst) {
		err = crypto_tls_copy_data(req, req->src, req->dst,
					   req->assoclen);
		if (err)
			return err;
	}
	dst = scatterwalk_ffwd(treq_ctx->dst, req->dst, req->assoclen);

	/*
	 * STEP 3: encrypt the frame and return the result
	 */
	cryptlen = req->cryptlen + phashlen;

	/*
	 * The hash and the cipher are applied at different times and their
	 * requests can use the same memory space without interference
	 */
	skreq = (void *)(treq_ctx->tail + ctx->reqoff);
	skcipher_request_set_tfm(skreq, ctx->enc);
	skcipher_request_set_crypt(skreq, src, dst, cryptlen, req->iv);
	skcipher_request_set_callback(skreq, aead_request_flags(req),
				      req->base.complete, req->base.data);
	/*
	 * Apply the cipher transform. The result will be in req->dst when the
	 * asynchronuous call terminates
	 */
	err = crypto_skcipher_encrypt(skreq);

	return err;
}

static int crypto_tls_decrypt(struct aead_request *req)
{
	struct crypto_aead *tls = crypto_aead_reqtfm(req);
	struct crypto_tls_ctx *ctx = crypto_aead_ctx(tls);
	struct tls_request_ctx *treq_ctx = aead_request_ctx(req);
	unsigned int cryptlen = req->cryptlen;
	unsigned int hash_size = crypto_aead_authsize(tls);
	unsigned int block_size = crypto_aead_blocksize(tls);
	struct skcipher_request *skreq = (void *)(treq_ctx->tail + ctx->reqoff);
	struct scatterlist *tmp = treq_ctx->tmp;
	struct scatterlist *src, *dst;

	u8 padding[255]; /* padding can be 0-255 bytes */
	u8 pad_size;
	u16 *len_field;
	u8 *ihash, *hash = treq_ctx->tail;

	int paderr = 0;
	int err = -EINVAL;
	int i;
	struct async_op ciph_op;

	/*
	 * Rule out bad packets. The input packet length must be at least one
	 * byte more than the hash_size
	 */
	if (cryptlen <= hash_size || cryptlen % block_size)
		goto out;

	/*
	 * Step 1 - Decrypt the source. Fast-forward past the associated data
	 * to the encrypted data. The result will be overwritten in place so
	 * that the decrypted data will be adjacent to the associated data. The
	 * last step (computing the hash) will have it's input data already
	 * prepared and ready to be accessed at req->src.
	 */
	src = scatterwalk_ffwd(tmp, req->src, req->assoclen);
	dst = src;

	init_completion(&ciph_op.completion);
	skcipher_request_set_tfm(skreq, ctx->enc);
	skcipher_request_set_callback(skreq, aead_request_flags(req),
				      tls_async_op_done, &ciph_op);
	skcipher_request_set_crypt(skreq, src, dst, cryptlen, req->iv);
	err = crypto_skcipher_decrypt(skreq);
	if (err == -EINPROGRESS) {
		err = wait_for_completion_interruptible(&ciph_op.completion);
		if (!err)
			err = ciph_op.err;
	}
	if (err)
		goto out;

	/*
	 * Step 2 - Verify padding
	 * Retrieve the last byte of the payload; this is the padding size.
	 */
	cryptlen -= 1;
	scatterwalk_map_and_copy(&pad_size, dst, cryptlen, 1, 0);

	/* RFC recommendation for invalid padding size. */
	if (cryptlen < pad_size + hash_size) {
		pad_size = 0;
		paderr = -EBADMSG;
	}
	cryptlen -= pad_size;
	scatterwalk_map_and_copy(padding, dst, cryptlen, pad_size, 0);

	/* Padding content must be equal with pad_size. We verify it all */
	for (i = 0; i < pad_size; i++)
		if (padding[i] != pad_size)
			paderr = -EBADMSG;

	/*
	 * Step 3 - Verify hash
	 * Align the digest result as required by the hash transform. Enough
	 * space was allocated in crypto_tls_init_tfm
	 */
	hash = (u8 *)ALIGN((unsigned long)hash +
			   crypto_ahash_alignmask(ctx->auth),
			   crypto_ahash_alignmask(ctx->auth) + 1);
	/*
	 * Two bytes at the end of the associated data make the length field.
	 * It must be updated with the length of the cleartext message before
	 * the hash is calculated.
	 */
	len_field = sg_virt(req->src) + req->assoclen - 2;
	cryptlen -= hash_size;
	*len_field = htons(cryptlen);

	/* This is the hash from the decrypted packet. Save it for later */
	ihash = hash + hash_size;
	scatterwalk_map_and_copy(ihash, dst, cryptlen, hash_size, 0);

	/* Now compute and compare our ICV with the one from the packet */
	err = crypto_tls_genicv(hash, req->src, cryptlen + req->assoclen, req);
	if (!err)
		err = memcmp(hash, ihash, hash_size) ? -EBADMSG : 0;

	if (req->src != req->dst) {
		err = crypto_tls_copy_data(req, req->src, req->dst, cryptlen +
					   req->assoclen);
		if (err)
			goto out;
	}

	/* return the first found error */
	if (paderr)
		err = paderr;

out:
	aead_request_complete(req, err);
	return err;
}

static int crypto_tls_init_tfm(struct crypto_aead *tfm)
{
	struct aead_instance *inst = aead_alg_instance(tfm);
	struct tls_instance_ctx *ictx = aead_instance_ctx(inst);
	struct crypto_tls_ctx *ctx = crypto_aead_ctx(tfm);
	struct crypto_ahash *auth;
	struct crypto_skcipher *enc;
	struct crypto_sync_skcipher *null;
	int err;

	auth = crypto_spawn_ahash(&ictx->auth);
	if (IS_ERR(auth))
		return PTR_ERR(auth);

	enc = crypto_spawn_skcipher(&ictx->enc);
	err = PTR_ERR(enc);
	if (IS_ERR(enc))
		goto err_free_ahash;

	null = crypto_get_default_null_skcipher();
	err = PTR_ERR(null);
	if (IS_ERR(null))
		goto err_free_skcipher;

	ctx->auth = auth;
	ctx->enc = enc;
	ctx->null = null;

	/*
	 * Allow enough space for two digests. The two digests will be compared
	 * during the decryption phase. One will come from the decrypted packet
	 * and the other will be calculated. For encryption, one digest is
	 * padded (up to a cipher blocksize) and chained with the payload
	 */
	ctx->reqoff = ALIGN(crypto_ahash_digestsize(auth) +
			    crypto_ahash_alignmask(auth),
			    crypto_ahash_alignmask(auth) + 1) +
			    max(crypto_ahash_digestsize(auth),
				crypto_skcipher_blocksize(enc));

	crypto_aead_set_reqsize(tfm,
				sizeof(struct tls_request_ctx) +
				ctx->reqoff +
				max_t(unsigned int,
				      crypto_ahash_reqsize(auth) +
				      sizeof(struct ahash_request),
				      crypto_skcipher_reqsize(enc) +
				      sizeof(struct skcipher_request)));

	return 0;

err_free_skcipher:
	crypto_free_skcipher(enc);
err_free_ahash:
	crypto_free_ahash(auth);
	return err;
}

static void crypto_tls_exit_tfm(struct crypto_aead *tfm)
{
	struct crypto_tls_ctx *ctx = crypto_aead_ctx(tfm);

	crypto_free_ahash(ctx->auth);
	crypto_free_skcipher(ctx->enc);
	crypto_put_default_null_skcipher();
}

static void crypto_tls_free(struct aead_instance *inst)
{
	struct tls_instance_ctx *ctx = aead_instance_ctx(inst);

	crypto_drop_skcipher(&ctx->enc);
	crypto_drop_ahash(&ctx->auth);
	kfree(inst);
}

static int crypto_tls_create(struct crypto_template *tmpl, struct rtattr **tb)
{
	struct crypto_attr_type *algt;
	struct aead_instance *inst;
	struct hash_alg_common *auth;
	struct crypto_alg *auth_base;
	struct skcipher_alg *enc;
	struct tls_instance_ctx *ctx;
	u32 mask;
	int err;

	algt = crypto_get_attr_type(tb);
	if (IS_ERR(algt))
		return PTR_ERR(algt);

	if ((algt->type ^ CRYPTO_ALG_TYPE_AEAD) & algt->mask)
		return -EINVAL;

	err = crypto_check_attr_type(tb, CRYPTO_ALG_TYPE_AEAD, &mask);
	if (err)
		return err;

	inst = kzalloc(sizeof(*inst) + sizeof(*ctx), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;
	ctx = aead_instance_ctx(inst);

	err = crypto_grab_ahash(&ctx->auth, aead_crypto_instance(inst),
				crypto_attr_alg_name(tb[1]), 0, mask);
	if (err)
		goto err_free_inst;
	auth = crypto_spawn_ahash_alg(&ctx->auth);
	auth_base = &auth->base;

	err = crypto_grab_skcipher(&ctx->enc, aead_crypto_instance(inst),
				   crypto_attr_alg_name(tb[2]), 0, mask);
	if (err)
		goto err_free_inst;
	enc = crypto_spawn_skcipher_alg(&ctx->enc);

	err = -ENAMETOOLONG;
	if (snprintf(inst->alg.base.cra_name, CRYPTO_MAX_ALG_NAME,
		     "tls10(%s,%s)", auth_base->cra_name,
		     enc->base.cra_name) >= CRYPTO_MAX_ALG_NAME)
		goto err_free_inst;

	if (snprintf(inst->alg.base.cra_driver_name, CRYPTO_MAX_ALG_NAME,
		     "tls10(%s,%s)", auth_base->cra_driver_name,
		     enc->base.cra_driver_name) >= CRYPTO_MAX_ALG_NAME)
		goto err_free_inst;

	inst->alg.base.cra_flags = (auth_base->cra_flags |
					enc->base.cra_flags) & CRYPTO_ALG_ASYNC;
	inst->alg.base.cra_priority = enc->base.cra_priority * 10 +
					auth_base->cra_priority;
	inst->alg.base.cra_blocksize = enc->base.cra_blocksize;
	inst->alg.base.cra_alignmask = auth_base->cra_alignmask |
					enc->base.cra_alignmask;
	inst->alg.base.cra_ctxsize = sizeof(struct crypto_tls_ctx);

	inst->alg.ivsize = crypto_skcipher_alg_ivsize(enc);
	inst->alg.chunksize = crypto_skcipher_alg_chunksize(enc);
	inst->alg.maxauthsize = auth->digestsize;

	inst->alg.init = crypto_tls_init_tfm;
	inst->alg.exit = crypto_tls_exit_tfm;

	inst->alg.setkey = crypto_tls_setkey;
	inst->alg.encrypt = crypto_tls_encrypt;
	inst->alg.decrypt = crypto_tls_decrypt;

	inst->free = crypto_tls_free;

	err = aead_register_instance(tmpl, inst);
	if (err) {
err_free_inst:
		crypto_tls_free(inst);
	}

	return err;
}

static struct crypto_template crypto_tls_tmpl = {
	.name = "tls10",
	.create = crypto_tls_create,
	.module = THIS_MODULE,
};

static int __init crypto_tls_module_init(void)
{
	return crypto_register_template(&crypto_tls_tmpl);
}

static void __exit crypto_tls_module_exit(void)
{
	crypto_unregister_template(&crypto_tls_tmpl);
}

module_init(crypto_tls_module_init);
module_exit(crypto_tls_module_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TLS 1.0 record encryption");
MODULE_ALIAS_CRYPTO("tls10");
