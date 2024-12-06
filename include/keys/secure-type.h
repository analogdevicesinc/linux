/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 NXP.
 *
 */

#ifndef _KEYS_SECURE_TYPE_H
#define _KEYS_SECURE_TYPE_H

#include <linux/key.h>
#include <linux/rcupdate.h>

/* Minimum key size to be used is 32 bytes and maximum key size fixed
 * is 128 bytes.
 * Blob size to be kept is Maximum key size + blob header added by CAAM.
 */

#define MIN_KEY_SIZE                    32
#define MAX_KEY_SIZE                    128
#define BLOB_HEADER_SIZE		48

#define MAX_BLOB_SIZE                   (MAX_KEY_SIZE + BLOB_HEADER_SIZE)

struct secure_key_payload {
	struct rcu_head rcu;
	unsigned int key_len;
	unsigned int blob_len;
	unsigned char key[MAX_KEY_SIZE + 1];
	unsigned char blob[MAX_BLOB_SIZE];
};

extern struct key_type key_type_secure;
#endif
