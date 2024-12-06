// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 NXP.
 *
 */

#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/err.h>
#include <keys/secure-type.h>
#include <keys/encrypted-type.h>
#include "encrypted.h"

/*
 * request_secure_key - request the secure key
 *
 * Secure keys and their blobs are derived from CAAM hardware.
 * Userspace manages secure  key-type data, but key data is not
 * visible in plain form. It is presented as blobs.
 */
struct key *request_secure_key(const char *secure_desc,
				const u8 **master_key, size_t *master_keylen)
{
	struct secure_key_payload *spayload;
	struct key *skey;

	skey = request_key(&key_type_secure, secure_desc, NULL);
	if (IS_ERR(skey))
		goto error;

	down_read(&skey->sem);
	spayload = skey->payload.data[0];
	*master_key = spayload->key;
	*master_keylen = spayload->key_len;
error:
	return skey;
}
