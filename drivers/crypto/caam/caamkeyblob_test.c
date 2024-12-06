// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Copyright 2018-2020 NXP
 *
 * Test of black key generation from a plaintext (of different size)
 * and from random.
 * The random black key is encapsulated into a blob.
 * Next, the blob is decapsulated and the new obtained black key is
 * compared to the random black key.
 */
#include <linux/module.h>
#include "caamkeyblob.h"

#define MAX_INPUT_SIZE 64
#define KEY_ENCRYPTION_ECB 1
#define KEY_ENCRYPTION_CCM 9

static char input[MAX_INPUT_SIZE];

static int create_black_key(struct device *dev, struct keyblob_info *info)
{
	int ret;

	ret = generate_black_key(dev, info);
	if (ret)
		dev_err(dev, "black key of size: %zd, type: %d returned %d",
			info->key_len, info->type, ret);

	return (ret) ? 1 : 0;
}

static int black_key_test(void)
{
	struct device *dev;
	struct keyblob_info *info;
	int i, ret = 0, nb_errors = 0;
	unsigned char tmp_black_key[MAX_BLACK_KEY_SIZE];
	size_t tmp_black_key_len = 0;

	dev = caam_jr_alloc();
	if (!dev)
		return -ENOMEM;

	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->key = input;

	for (i = 1; i <= MAX_INPUT_SIZE; i++) {
		info->key_len = i;

		/* Create a black key encrypted with AES-ECB */
		info->type = KEY_ENCRYPTION_ECB;
		info->black_key_len = MAX_INPUT_SIZE;
		nb_errors += create_black_key(dev, info);

		/* Create a black key encrypted with AES-CCM */
		info->type = KEY_ENCRYPTION_CCM;
		info->black_key_len = MAX_INPUT_SIZE + CCM_OVERHEAD;
		nb_errors += create_black_key(dev, info);
	}

	/* Create, from random, a black key encrypted with AES-ECB */
	info->key = NULL;
	info->type = KEY_ENCRYPTION_ECB;
	info->black_key_len = MAX_INPUT_SIZE;
	ret = create_black_key(dev, info);
	nb_errors += (ret) ? 1 : 0;

	/* Save it for later to compare it */
	tmp_black_key_len = info->black_key_len;
	memcpy(tmp_black_key, info->black_key, tmp_black_key_len);

	/* Encapsulate the random black key into a black blob */
	info->key = NULL;
	/* Set key modifier, used as revision number, for blob */
	info->key_mod = caam_key_modifier;
	info->key_mod_len = ARRAY_SIZE(caam_key_modifier);
	/* Black key, encrypted with ECB-AES, in General Memory */
	info->type = KEY_ENCRYPTION_ECB;
	info->key_len = MAX_INPUT_SIZE;
	info->blob_len = MAX_BLOB_SIZE;
	ret = caam_blob_encap(dev, info);
	nb_errors += (ret) ? 1 : 0;

	/* Decapsulate the black key from the above black blob */
	ret = caam_blob_decap(dev, info);
	nb_errors += (ret) ? 1 : 0;

	/*
	 * Compare the generated black key with
	 * the one decapsulated from the blob
	 */
	if (info->black_key_len == tmp_black_key_len)
		ret = memcmp(tmp_black_key, info->black_key, tmp_black_key_len);
	else
		ret = 1;
	nb_errors += ret;

	/*
	 * Check number of errors.
	 * If nb_errors > 0, at least one operation failed, success otherwise
	 */
	pr_info("Nb errors: %d\n", nb_errors);

	caam_jr_free(dev);
	kfree(info);

	return nb_errors;
}

static int black_key_test_init(void)
{
	return black_key_test();
}

static void black_key_test_exit(void)
{
}

module_init(black_key_test_init);
module_exit(black_key_test_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("NXP CAAM Black Key and Blob Test");
MODULE_AUTHOR("NXP Semiconductors");
