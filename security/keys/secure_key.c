// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2018, 2021 NXP
 * Secure key is generated using NXP CAAM hardware block. CAAM generates the
 * random number (used as a key) and creates its blob for the user.
 */

#include <linux/slab.h>
#include <linux/parser.h>
#include <linux/string.h>
#include <linux/key-type.h>
#include <linux/rcupdate.h>
#include <keys/secure-type.h>
#include <linux/completion.h>

#include "securekey_desc.h"

static const char hmac_alg[] = "hmac(sha1)";
static const char hash_alg[] = "sha1";

static struct crypto_shash *hashalg;
static struct crypto_shash *hmacalg;

enum {
	error = -1,
	new_key,
	load_blob,
};

static const match_table_t key_tokens = {
	{new_key, "new"},
	{load_blob, "load"},
	{error, NULL}
};

static struct secure_key_payload *secure_payload_alloc(struct key *key)
{
	struct secure_key_payload *sec_key = NULL;
	int ret = 0;

	ret = key_payload_reserve(key, sizeof(*sec_key));
	if (ret < 0)
		goto out;

	sec_key = kzalloc(sizeof(*sec_key), GFP_KERNEL);
	if (!sec_key)
		goto out;

out:
	return sec_key;
}

/*
 * parse_inputdata - parse the keyctl input data and fill in the
 *		     payload structure for key or its blob.
 * param[in]: data pointer to the data to be parsed for creating key.
 * param[in]: p pointer to secure key payload structure to fill parsed data
 * On success returns 0, otherwise -EINVAL.
 */
static int parse_inputdata(char *data, struct secure_key_payload *p)
{
	substring_t args[MAX_OPT_ARGS];
	long keylen = 0;
	int ret = -EINVAL;
	int key_cmd = -EINVAL;
	char *c = NULL;

	c = strsep(&data, " \t");
	if (!c) {
		ret = -EINVAL;
		goto out;
	}

	/* Get the keyctl command i.e. new_key or load_blob etc */
	key_cmd = match_token(c, key_tokens, args);

	switch (key_cmd) {
	case new_key:
		/* first argument is key size */
		c = strsep(&data, " \t");
		if (!c) {
			ret = -EINVAL;
			goto out;
		}

		ret = kstrtol(c, 10, &keylen);
		if (ret < 0 || keylen < MIN_KEY_SIZE ||
						keylen > MAX_KEY_SIZE) {
			ret = -EINVAL;
			goto out;
		}

		p->key_len = keylen;
		ret = new_key;

		break;
	case load_blob:
		/* first argument is blob data for CAAM*/
		c = strsep(&data, " \t");
		if (!c) {
			ret = -EINVAL;
			goto out;
		}

		/* Blob_len = No of characters in blob/2 */
		p->blob_len = strlen(c) / 2;
		if (p->blob_len > MAX_BLOB_SIZE) {
			ret = -EINVAL;
			goto out;
		}

		ret = hex2bin(p->blob, c, p->blob_len);
		if (ret < 0) {
			ret = -EINVAL;
			goto out;
		}
		ret = load_blob;

		break;
	case error:
		ret = -EINVAL;
		break;
	}

out:
	return ret;
}

/*
 * secure_instantiate - create a new secure type key.
 * Supports the operation to generate a new key. A random number
 * is generated from CAAM as key data and the corresponding red blob
 * is formed and stored as key_blob.
 * Also supports the operation to load the blob and key is derived using
 * that blob from CAAM.
 * On success, return 0. Otherwise return errno.
 */
static int secure_instantiate(struct key *key,
		struct key_preparsed_payload *prep)
{
	struct secure_key_payload *payload = NULL;
	size_t datalen = prep->datalen;
	char *data = NULL;
	int key_cmd = 0;
	int ret = 0;
	enum sk_req_type sk_op_type;
	struct device *dev = NULL;

	if (datalen <= 0 || datalen > 32767 || !prep->data) {
		ret = -EINVAL;
		goto out;
	}

	data = kmalloc(datalen + 1, GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto out;
	}

	memcpy(data, prep->data, datalen);
	data[datalen] = '\0';

	payload = secure_payload_alloc(key);
	if (!payload) {
		ret = -ENOMEM;
		goto out;
	}

	/* Allocate caam job ring for operation to be performed from CAAM */
	dev = caam_jr_alloc();
	if (!dev) {
		pr_info("caam_jr_alloc failed\n");
		ret = -ENODEV;
		goto out;
	}

	key_cmd = parse_inputdata(data, payload);
	if (key_cmd < 0) {
		ret = key_cmd;
		goto out;
	}

	switch (key_cmd) {
	case load_blob:
		/*
		 * Red blob decryption to be done for load operation
		 * to derive the key.
		 */
		sk_op_type = sk_red_blob_dec;
		ret = key_deblob(payload, sk_op_type, dev);
		if (ret != 0) {
			pr_info("secure_key: key_blob decap fail (%d)\n", ret);
			goto out;
		}
		break;
	case new_key:
		/* Get Random number from caam of the specified length */
		sk_op_type = sk_get_random;
		ret = caam_get_random(payload, sk_op_type, dev);
		if (ret != 0) {
			pr_info("secure_key: get_random fail (%d)\n", ret);
			goto out;
		}

		/* Generate red blob of key random bytes with CAAM */
		sk_op_type = sk_red_blob_enc;
		ret = key_blob(payload, sk_op_type, dev);
		if (ret != 0) {
			pr_info("secure_key: key_blob encap fail (%d)\n", ret);
			goto out;
		}
		break;
	default:
		ret = -EINVAL;
		goto out;
	}
out:
	if (data)
		kfree_sensitive(data);
	if (dev)
		caam_jr_free(dev);

	if (!ret)
		rcu_assign_keypointer(key, payload);
	else
		kfree_sensitive(payload);

	return ret;
}

/*
 * secure_read - copy the blob data to internal buffer in hex.
 * param[in]: key pointer to key struct
 * param[in]: buffer pointer to user data for creating key
 * param[in]: buflen is the length of the buffer
 * On success, return to kernel space the secure key data size.
 */
static long secure_read(const struct key *key, char *buffer,
			 size_t buflen)
{
	const struct secure_key_payload *p = NULL;
	char *ascii_buf;
	char *bufp;
	int i;

	p = dereference_key_locked(key);
	if (!p)
		return -EINVAL;

	if (buffer && buflen >= 2 * p->blob_len) {
		ascii_buf = kmalloc(2 * p->blob_len, GFP_KERNEL);
		if (!ascii_buf)
			return -ENOMEM;

		bufp = ascii_buf;
		for (i = 0; i < p->blob_len; i++)
			bufp = hex_byte_pack(bufp, p->blob[i]);
		memcpy(buffer, ascii_buf, 2 * p->blob_len);
		kfree_sensitive(ascii_buf);
	}
	return 2 * p->blob_len;
}

/*
 * secure_destroy - clear and free the key's payload
 */
static void secure_destroy(struct key *key)
{
	kfree_sensitive(key->payload.data[0]);
}

struct key_type key_type_secure = {
	.name = "secure",
	.instantiate = secure_instantiate,
	.destroy = secure_destroy,
	.read = secure_read,
};
EXPORT_SYMBOL_GPL(key_type_secure);

static void secure_shash_release(void)
{
	if (hashalg)
		crypto_free_shash(hashalg);
	if (hmacalg)
		crypto_free_shash(hmacalg);
}

static int __init secure_shash_alloc(void)
{
	int ret;

	hmacalg = crypto_alloc_shash(hmac_alg, 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(hmacalg)) {
		pr_info("secure_key: could not allocate crypto %s\n",
				hmac_alg);
		return PTR_ERR(hmacalg);
	}

	hashalg = crypto_alloc_shash(hash_alg, 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(hashalg)) {
		pr_info("secure_key: could not allocate crypto %s\n",
				hash_alg);
		ret = PTR_ERR(hashalg);
		goto hashalg_fail;
	}

	return 0;

hashalg_fail:
	crypto_free_shash(hmacalg);
	return ret;
}

static int __init init_secure_key(void)
{
	int ret;

	ret = secure_shash_alloc();
	if (ret < 0)
		return ret;

	ret = register_key_type(&key_type_secure);
	if (ret < 0)
		secure_shash_release();
	return ret;
}

static void __exit cleanup_secure_key(void)
{
	secure_shash_release();
	unregister_key_type(&key_type_secure);
}

late_initcall(init_secure_key);
module_exit(cleanup_secure_key);

MODULE_LICENSE("GPL");
