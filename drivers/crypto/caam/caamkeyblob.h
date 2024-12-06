/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Black key generation and blob encapsulation/decapsualtion for CAAM
 *
 * Copyright 2018-2020 NXP
 */

#ifndef _CAAMKEYBLOB_H_
#define _CAAMKEYBLOB_H_

#include <linux/device.h>
#include "caamkeyblob_desc.h"

/*
 * Minimum key size to be used is 16 bytes and maximum key size fixed
 * is 64 bytes.
 * Blob size to be kept is Maximum key size + tag object header added by CAAM.
 */

#define MIN_KEY_SIZE			16
#define MAX_KEY_SIZE			64

#define MAX_BLACK_KEY_SIZE		(MAX_KEY_SIZE + CCM_OVERHEAD +\
					TAG_OVERHEAD_SIZE)

/*
 * For blobs a randomly-generated, 256-bit blob key is used to
 * encrypt the data using the AES-CCM cryptographic algorithm.
 * Therefore, blob size is max key size, CCM_OVERHEAD, blob header
 * added by CAAM and the tagged object header size.
 */
#define MAX_BLOB_SIZE			(MAX_KEY_SIZE + CCM_OVERHEAD +\
					BLOB_OVERHEAD + TAG_OVERHEAD_SIZE)

/* Key modifier for CAAM blobs, used as a revision number */
static const char caam_key_modifier[KEYMOD_SIZE_GM] = {
		'C', 'A', 'A', 'M', '_', 'K', 'E', 'Y',
		'_', 'T', 'Y', 'P', 'E', '_', 'V', '1',
};

/**
 * struct keyblob_info - Structure that contains all the data necessary
 *                       to generate a black key and encapsulate it into a blob
 *
 * @key                : The plaintext used as input key
 *                       for black key generation
 * @key_len            : Size of plaintext or size of key in case of
 *                       black key generated from random
 * @type               : The type of data contained (e.g. black key, blob, etc.)
 * @black_key_len      : Length of the generated black key
 * @black_key          : Black key data obtained from CAAM
 * @blob_len           : Length of the blob that encapsulates the black key
 * @blob               : Blob data obtained from CAAM
 * @key_modifier_len   : 8-byte or 16-byte Key_Modifier based on general or
 *                       secure memory blob type
 * @key_modifier       : can be either a secret value, or used as a revision
 *                       number, revision date or nonce
 *                       In this case is used as a revision number.
 */
struct keyblob_info {
	char *key;
	size_t key_len;

	u32 type;

	size_t black_key_len;
	unsigned char black_key[MAX_BLACK_KEY_SIZE];

	size_t blob_len;
	unsigned char blob[MAX_BLOB_SIZE];

	size_t key_mod_len;
	const void *key_mod;
};

int generate_black_key(struct device *dev, struct keyblob_info *info);

int caam_blob_encap(struct device *dev, struct keyblob_info *info);

int caam_blob_decap(struct device *dev, struct keyblob_info *info);

#endif /* _CAAMKEYBLOB_H_ */
