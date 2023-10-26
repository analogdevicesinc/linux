// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 NXP
 */

#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/tee_drv.h>
#include <linux/uuid.h>

#include <crypto/aes.h>
#include <crypto/engine.h>
#include <crypto/scatterwalk.h>
#include <crypto/sha2.h>
#include <crypto/skcipher.h>
#include <crypto/internal/skcipher.h>
#include <crypto/xts.h>

#include "tee_skcipher.h"

static DEFINE_MUTEX(algs_lock);
static unsigned int active_devs;
static atomic_long_t key_counter;
static DEFINE_MUTEX(tc_prv_lock);

struct tee_crypt_priv_data {
	uuid_t uuid;
	struct tee_context *ctx;
	u32 session_id;
	uint64_t shm_pool_paddr;
	u8 *shm_pool;
};

static struct tee_crypt_priv_data *tc_prv;

struct tee_crypt_alg_entry {
	bool key_on_ddr;
	u8 enc_cmd_id;
	u8 dec_cmd_id;
};

struct tee_crypt_skcipher_alg {
	struct skcipher_alg skcipher;
	struct tee_crypt_alg_entry tc_alg_entry;
	bool registered;
};

/*
 * per-session context
 */
struct tc_aes_ctx {
	uint32_t key_id_1;
	uint32_t key_id_2;
	uint32_t iv_size;
};

static int skcipher_alloc_shm(struct tee_crypt_priv_data *tc_prv_ctx,
			      uint64_t length,
			      uint64_t *pa)
{
	int ret;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	if (!tc_prv_ctx || !length || !pa) {
		pr_err("%s: Invalid Input params.\n", __func__);
		return -EINVAL;
	}

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	inv_arg.func = PTA_SHM_ALLOCATE;
	inv_arg.session = tc_prv_ctx->session_id;
	inv_arg.num_params = 4;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = (length >> 32);
	param[0].u.value.b = (length & 0xFFFFFFFF);
	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;

	ret = tee_client_invoke_func(tc_prv_ctx->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		pr_err("PTA_SHM_ALLOCATE invoke err: %x\n", inv_arg.ret);
		ret = -EFAULT;
	} else {
		*pa = param[1].u.value.a;
		*pa = (*pa << 32) | param[1].u.value.b;
	}

	return ret;
}

static int skcipher_free_shm(struct tee_crypt_priv_data *tc_prv_ctx,
			     uint64_t pa)
{
	int ret;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	if (!tc_prv_ctx || !pa) {
		pr_err("%s: Invalid Input params.\n", __func__);
		return -EINVAL;
	}

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	inv_arg.func = PTA_SHM_FREE;
	inv_arg.session = tc_prv_ctx->session_id;
	inv_arg.num_params = 4;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = pa >> 32;
	param[0].u.value.b = pa & 0xFFFFFFFF;

	ret = tee_client_invoke_func(tc_prv_ctx->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		pr_err("PTA_SHM_FREE invoke err: %x\n", inv_arg.ret);
		ret = -EFAULT;
	}

	return ret;
}

static int skcipher_setkey(struct crypto_skcipher *skcipher,
			   const u8 *key,
			   unsigned int key_len,
			   bool xts_algo)
{
	struct tc_aes_ctx *ctx = crypto_skcipher_ctx(skcipher);
	struct tee_crypt_skcipher_alg *alg =
		container_of(crypto_skcipher_alg(skcipher), typeof(*alg),
			     skcipher);
	struct tee_crypt_priv_data *tc_prv_ctx = NULL;
	int ret;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];
	struct tee_shm *reg_shm_key = NULL;

	mutex_lock(&tc_prv_lock);
	tc_prv_ctx = tc_prv;

	/* Key Counter should be atomically incremented. */
	ctx->key_id_1 = atomic_long_inc_return(&key_counter);
	if (xts_algo)
		ctx->key_id_2 = atomic_long_inc_return(&key_counter);
	ctx->iv_size = crypto_skcipher_ivsize(skcipher);

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	reg_shm_key = tee_shm_register_kernel_buf(tc_prv_ctx->ctx,
						  (void *)key,
						  key_len);
	if (IS_ERR(reg_shm_key)) {
		pr_err("key buffer, tee-shm register failed.\n");
		ret = PTR_ERR(reg_shm_key);
		goto exit;
	}

	inv_arg.func = xts_algo ? PTA_SET_XTS_KEY : PTA_SET_CBC_KEY;
	inv_arg.session = tc_prv_ctx->session_id;
	inv_arg.num_params = 4;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_MEMREF_INPUT;
	param[0].u.memref.shm = reg_shm_key;
	param[0].u.memref.size = key_len;
	param[0].u.memref.shm_offs = 0;

	param[1].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[1].u.value.a = ctx->key_id_1;
	if (xts_algo)
		param[1].u.value.b = ctx->key_id_2;

	ret = tee_client_invoke_func(tc_prv_ctx->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		pr_err("PTA_SET_KEY invoke err: %x\n", inv_arg.ret);
		ret = -EFAULT;
	}

exit:
	mutex_unlock(&tc_prv_lock);

	if (reg_shm_key)
		tee_shm_free(reg_shm_key);

	return ret;
}

static int cbc_skcipher_setkey(struct crypto_skcipher *skcipher,
			       const u8 *key, unsigned int keylen)
{
	int ret;

	if (!skcipher || !key) {
		pr_err("%s: Invalid Input params.\n", __func__);
		return -EINVAL;
	}

	ret = aes_check_keylen(keylen);
	if (ret) {
		pr_err("%s: key size mismatch.\n", __func__);
		return ret;
	}

	return skcipher_setkey(skcipher, key, keylen, false);
}

static int xts_skcipher_setkey(struct crypto_skcipher *skcipher, const u8 *key,
			       unsigned int keylen)
{
	int ret;

	if (!skcipher || !key) {
		pr_err("%s: Invalid Input params.\n", __func__);
		return -EINVAL;
	}

	ret = xts_verify_key(skcipher, key, keylen);
	if (ret) {
		pr_err("%s: key size mismatch.\n", __func__);
		return ret;
	}

	return skcipher_setkey(skcipher, key, keylen, true);
}

static inline int tc_sk_crypt(struct tc_aes_ctx *ctx,
			      struct tee_crypt_priv_data *tc_prv_ctx,
			      u32 out_len,
			      u32 in_len,
			      u32 iv_len,
			      u8 cmd_id)
{
	struct arm_smccc_res res;
	phys_addr_t iv;
	phys_addr_t srcdata;
	phys_addr_t dstdata;
	uint64_t key_param;
	unsigned long smc_cmd;

	if (!tc_prv_ctx || !ctx || !out_len || !in_len || !iv_len)
		return -EINVAL;

	switch (cmd_id) {
	case PTA_ENCRYPT_CBC:
		smc_cmd = IMX_SMC_ENCRYPT_CBC;
		key_param = ctx->key_id_1;
		break;
	case PTA_DECRYPT_CBC:
		smc_cmd = IMX_SMC_DECRYPT_CBC;
		key_param = ctx->key_id_1;
		break;
	case PTA_ENCRYPT_XTS:
		smc_cmd = IMX_SMC_ENCRYPT_XTS;
		key_param = ctx->key_id_2;
		key_param = (key_param << 32) | ctx->key_id_1;
		break;
	case PTA_DECRYPT_XTS:
		smc_cmd = IMX_SMC_DECRYPT_XTS;
		key_param = ctx->key_id_2;
		key_param = (key_param << 32) | ctx->key_id_1;
		break;
	default:
		pr_err("Invalid command id: 0x%x\n", cmd_id);
		return -EINVAL;
	}

	iv = tc_prv_ctx->shm_pool_paddr + in_len + out_len;
	srcdata = tc_prv_ctx->shm_pool_paddr;
	dstdata = tc_prv_ctx->shm_pool_paddr + in_len;
	arm_smccc_smc(smc_cmd, key_param, iv, srcdata, in_len, dstdata,
		      out_len, 0, &res);

	return res.a0;
}

static int aes_crypt(struct skcipher_request *req, u8 ta_cmd_id)
{
	int ret = 0;
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct tc_aes_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct tee_crypt_priv_data *tc_prv_ctx = NULL;
	u32 crypt_len = req->cryptlen;
	u32 pad_len = (crypt_len &=  AES_BLOCK_SIZE - 1)
		      ? (AES_BLOCK_SIZE - crypt_len) : 0;
	u32 out_len = req->cryptlen + pad_len;
	u32 in_len = req->cryptlen + pad_len;
	u32 iv_len = ctx->iv_size;
	u8 *out_buf = NULL;
	u8 *in_buf = NULL;
	u8 *iv_buf = NULL;
	u32 src_nents;
	u32 dst_nents;

	src_nents = sg_nents_for_len(req->src, req->cryptlen);

	mutex_lock(&tc_prv_lock);
	tc_prv_ctx = tc_prv;

	in_buf = tc_prv_ctx->shm_pool;
	out_buf = in_buf + in_len;
	iv_buf = out_buf + out_len;

	memcpy(iv_buf, req->iv, iv_len);

	sg_copy_to_buffer(req->src, src_nents, in_buf, in_len);

	ret = tc_sk_crypt(ctx, tc_prv_ctx,
			  out_len, in_len, iv_len,
			  ta_cmd_id);

	if (ret) {
		pr_err("TEE-Crypt: SK-Cipher Ops Failed[0x%x].", ret);
		goto out;
	}

	dst_nents = sg_nents_for_len(req->dst, req->cryptlen);

	sg_copy_from_buffer(req->dst, dst_nents, out_buf, out_len);

out:
	mutex_unlock(&tc_prv_lock);

	return ret;
}

static int skcipher_encrypt(struct skcipher_request *req)
{
	struct crypto_skcipher *skcipher = NULL;
	struct tee_crypt_skcipher_alg *alg = NULL;

	if (!req)
		return -EINVAL;

	skcipher = crypto_skcipher_reqtfm(req);
	if (!skcipher)
		return -EINVAL;

	alg = container_of(crypto_skcipher_alg(skcipher), typeof(*alg),
			   skcipher);

	return aes_crypt(req, alg->tc_alg_entry.enc_cmd_id);
}

static int skcipher_decrypt(struct skcipher_request *req)
{
	struct crypto_skcipher *skcipher = NULL;
	struct tee_crypt_skcipher_alg *alg = NULL;

	if (!req)
		return -EINVAL;

	skcipher = crypto_skcipher_reqtfm(req);
	if (!skcipher)
		return -EINVAL;

	alg = container_of(crypto_skcipher_alg(skcipher), typeof(*alg),
			   skcipher);

	return aes_crypt(req, alg->tc_alg_entry.dec_cmd_id);
}

static struct tee_crypt_skcipher_alg driver_algs[] = {
	{
		.skcipher = {
			.base = {
				.cra_name = "cbc(aes)",
				.cra_driver_name = "cbc-aes-tee",
				.cra_blocksize = AES_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct tc_aes_ctx)
			},
			.setkey = cbc_skcipher_setkey,
			.encrypt = skcipher_encrypt,
			.decrypt = skcipher_decrypt,
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.ivsize = AES_BLOCK_SIZE,
		},
		.tc_alg_entry = {
			.key_on_ddr = false,
			.enc_cmd_id = PTA_ENCRYPT_CBC,
			.dec_cmd_id = PTA_DECRYPT_CBC,
		},
	},
	{
		.skcipher = {
			.base = {
				.cra_name = "xts(aes)",
				.cra_driver_name = "xts-aes-tee",
				.cra_blocksize = AES_BLOCK_SIZE,
				.cra_ctxsize = sizeof(struct tc_aes_ctx)
			},
			.setkey = xts_skcipher_setkey,
			.encrypt = skcipher_encrypt,
			.decrypt = skcipher_decrypt,
			.min_keysize = 2 * AES_MIN_KEY_SIZE,
			.max_keysize = 2 * AES_MAX_KEY_SIZE,
			.ivsize = AES_BLOCK_SIZE,
		},
		.tc_alg_entry = {
			.key_on_ddr = false,
			.enc_cmd_id = PTA_ENCRYPT_XTS,
			.dec_cmd_id = PTA_DECRYPT_XTS,
		},
	},
};

static int tee_crypt_cra_init(struct crypto_skcipher *tfm)
{
	return 0;
}

static void tee_crypt_cra_exit(struct crypto_skcipher *skcipher)
{
	struct tc_aes_ctx *ctx = crypto_skcipher_ctx(skcipher);
	struct tee_crypt_priv_data *tc_prv_ctx = NULL;
	int ret;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[4];

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	mutex_lock(&tc_prv_lock);
	tc_prv_ctx = tc_prv;

	inv_arg.func = PTA_REMOVE_KEY;
	inv_arg.session = tc_prv_ctx->session_id;
	inv_arg.num_params = 4;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = ctx->key_id_1;
	param[0].u.value.b = ctx->key_id_2;

	ret = tee_client_invoke_func(tc_prv_ctx->ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0))
		pr_err("PTA_REMOVE_KEY invoke err: %x\n", inv_arg.ret);

	mutex_unlock(&tc_prv_lock);
}

static void tee_crypt_skcipher_alg_init(struct tee_crypt_skcipher_alg *t_alg)
{
	struct skcipher_alg *alg = NULL;

	if (!t_alg) {
		pr_err("%s: Invalid Input params.\n", __func__);
		return;
	}

	alg = &t_alg->skcipher;
	alg->base.cra_module = THIS_MODULE;
	alg->base.cra_priority = TEE_CRYPTO_CRA_PRIORITY;
	alg->base.cra_ctxsize = sizeof(struct tc_aes_ctx);
	alg->base.cra_flags |= (CRYPTO_ALG_ALLOCATES_MEMORY |
				CRYPTO_ALG_KERN_DRIVER_ONLY);

	alg->init = tee_crypt_cra_init;
	alg->exit = tee_crypt_cra_exit;
}

int tee_crypt_algapi_init(void)
{
	int i = 0, ret = 0;
	bool registered = false;

	for (i = 0; i < ARRAY_SIZE(driver_algs); i++) {
		struct tee_crypt_skcipher_alg *t_alg = driver_algs + i;

		tee_crypt_skcipher_alg_init(t_alg);

		ret = crypto_register_skcipher(&t_alg->skcipher);
		if (ret) {
			pr_warn("%s alg registration failed\n",
				t_alg->skcipher.base.cra_driver_name);
			continue;
		}

		t_alg->registered = true;
		registered = true;
	}

	if (registered)
		pr_info("tee_crypt algorithms registered in /proc/crypto\n");

	return ret;
}

void tee_crypt_algapi_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(driver_algs); i++) {
		struct tee_crypt_skcipher_alg *t_alg = driver_algs + i;

		if (t_alg->registered)
			crypto_unregister_skcipher(&t_alg->skcipher);
	}
}

static void register_algs(void)
{
	mutex_lock(&algs_lock);

	if (++active_devs != 1)
		goto algs_unlock;

	tee_crypt_algapi_init();

algs_unlock:
	mutex_unlock(&algs_lock);
}

static void unregister_algs(void)
{
	mutex_lock(&algs_lock);

	if (--active_devs != 0)
		goto algs_unlock;

	tee_crypt_algapi_exit();

algs_unlock:
	mutex_unlock(&algs_lock);
}

static int optee_ctx_match(struct tee_ioctl_version_data *ver, const void *data)
{
	if (!ver)
		return false;

	return (ver->impl_id == TEE_IMPL_ID_OPTEE);
}

static int tee_crypt_close_session(struct tee_crypt_priv_data *tc_prv_ctx)
{
	if (!tc_prv_ctx)
		return -EFAULT;

	if (tc_prv_ctx->shm_pool)
		memunmap(tc_prv_ctx->shm_pool);
	if (tc_prv_ctx->shm_pool_paddr)
		skcipher_free_shm(tc_prv_ctx, tc_prv_ctx->shm_pool_paddr);
	if (tc_prv_ctx->session_id)
		tee_client_close_session(tc_prv_ctx->ctx,
					 tc_prv_ctx->session_id);
	if (tc_prv_ctx->ctx)
		tee_client_close_context(tc_prv_ctx->ctx);

	return 0;
}

static int tee_crypt_open_session(struct tee_crypt_priv_data *tc_prv_ctx)
{
	int ret;
	struct tee_ioctl_open_session_arg sess_arg = { };

	if (!tc_prv_ctx)
		return -EFAULT;

	tc_prv_ctx->ctx = tee_client_open_context(NULL, optee_ctx_match, NULL,
						  NULL);
	if (IS_ERR(tc_prv_ctx->ctx))
		return -ENODEV;

	memcpy(sess_arg.uuid, tc_prv_ctx->uuid.b, TEE_IOCTL_UUID_LEN);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_REE_KERNEL;
	sess_arg.num_params = 0;

	ret = tee_client_open_session(tc_prv_ctx->ctx, &sess_arg, NULL);
	if ((ret < 0) || (sess_arg.ret != 0)) {
		pr_err("tee_client_open_session failed, err: %x\n",
		       sess_arg.ret);
		ret = -EINVAL;
		goto out_ctx;
	}
	tc_prv_ctx->session_id = sess_arg.session;

	ret = skcipher_alloc_shm(tc_prv_ctx,
				 AES_BLOCK_SIZE + 2 * SZ_4K,
				 &tc_prv_ctx->shm_pool_paddr);
	if (ret) {
		ret = -ENOMEM;
		goto out_ctx;
	}
	tc_prv_ctx->shm_pool = memremap(tc_prv_ctx->shm_pool_paddr,
				    AES_BLOCK_SIZE + 2 * SZ_4K, MEMREMAP_WB);
	if (!tc_prv_ctx->shm_pool) {
		ret = -EINVAL;
		goto out_ctx;
	}

	return 0;

out_ctx:
	tee_crypt_close_session(tc_prv_ctx);

	return ret;
}

#define VERSION "1.0"
static int __init init_tee_crypt(void)
{
	int ret = -EPERM;

	tc_prv = kcalloc(1, sizeof(struct tee_crypt_priv_data),
			 GFP_KERNEL);

	tc_prv->uuid = UUID_INIT(0x560c5231, 0x71bc, 0x476d,
			0x8c, 0x2e, 0x4b, 0xa1, 0x07, 0x99,
			0x1e, 0x72);

	ret = tee_crypt_open_session(tc_prv);
	if (ret) {
		pr_info("TEE-Crypto: Init failed[0x%x].\n",
			ret);
		kfree(tc_prv);
		return ret;
	}

	atomic_long_set(&key_counter, 0);

	register_algs();
	pr_info("driver %s loaded.\n", VERSION);

	return ret;
}

static void __exit exit_tee_crypt(void)
{
	int ret;

	ret = tee_crypt_close_session(tc_prv);
	if (ret) {
		pr_info("TEE-Crypto: Session closed failed[0x%x].\n",
			ret);
		return;
	}

	kfree(tc_prv);
	unregister_algs();

	pr_info("driver unloaded.\n");
}

late_initcall(init_tee_crypt);
module_exit(exit_tee_crypt);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("TEE Crypto Driver");
MODULE_LICENSE("GPL");
MODULE_SOFTDEP("platform:optee");

int verbosity;
module_param(verbosity, int, 0644);
MODULE_PARM_DESC(verbosity, "0: normal, 1: verbose, 2: debug");
