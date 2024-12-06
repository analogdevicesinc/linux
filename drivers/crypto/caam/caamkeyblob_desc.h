/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Shared descriptors for CAAM black key and blob
 *
 * Copyright 2018-2020 NXP
 */

#ifndef _CAAMKEYBLOB_DESC_H_
#define _CAAMKEYBLOB_DESC_H_

#include <linux/types.h>

#include "jr.h"
#include "regs.h"
#include "desc.h"

#include "compat.h"
#include "tag_object.h"
#include "desc_constr.h"

/* Defines for secure memory and general memory blobs */
#define DATA_GENMEM 0
#define DATA_SECMEM 1

/* Encrypted key */
#define BLACK_KEY 1

/* Define key encryption/covering options */
#define KEY_COVER_ECB 0	/* cover key in AES-ECB */
#define KEY_COVER_CCM 1 /* cover key with AES-CCM */

/* Define the trust in the key, to select either JDKEK or TDKEK */
#define UNTRUSTED_KEY 0
#define TRUSTED_KEY 1

/* Define space required for BKEK + MAC tag storage in any blob */
#define BLOB_OVERHEAD (32 + 16)

#define PAD_16_BYTE(_key_size) (roundup(_key_size, 16))
#define PAD_8_BYTE(_key_size) (roundup(_key_size, 8))

/*
 * ECB-Black Key will be padded with zeros to make it a
 * multiple of 16 bytes long before it is encrypted,
 * and the resulting Black Key will be this length.
 */
#define ECB_BLACK_KEY_SIZE(_key_size) (PAD_16_BYTE(_key_size))

/*
 * CCM-Black Key will always be at least 12 bytes longer,
 * since the encapsulation uses a 6-byte nonce and adds
 * a 6-byte ICV. But first, the key is padded as necessary so
 * that CCM-Black Key is a multiple of 8 bytes long.
 */
#define NONCE_SIZE 6
#define ICV_SIZE 6
#define CCM_OVERHEAD (NONCE_SIZE + ICV_SIZE)
#define CCM_BLACK_KEY_SIZE(_key_size) (PAD_8_BYTE(_key_size) \
							+ CCM_OVERHEAD)

static inline int secret_size_in_ccm_black_key(int key_size)
{
	return ((key_size >= CCM_OVERHEAD) ? key_size - CCM_OVERHEAD : 0);
}

#define SECRET_SIZE_IN_CCM_BLACK_KEY(_key_size) \
	secret_size_in_ccm_black_key(_key_size)

/* A red key is not encrypted so its size is the same */
#define RED_KEY_SIZE(_key_size) (_key_size)

/*
 * Based on memory type, the key modifier length
 * can be either 8-byte or 16-byte.
 */
#define KEYMOD_SIZE_SM 8
#define KEYMOD_SIZE_GM 16

/* Create job descriptor to cover key */
int cnstr_desc_black_key(u32 **desc, char *key, size_t key_len,
			 dma_addr_t black_key, size_t black_key_len,
			 u8 key_enc, u8 trusted_key);

/* Create job descriptor to generate a random key and cover it */
int cnstr_desc_random_black_key(u32 **desc, size_t key_len,
				dma_addr_t black_key, size_t black_key_len,
				u8 key_enc, u8 trusted_key);

/* Encapsulate data in a blob */
int cnstr_desc_blob_encap(u32 **desc, dma_addr_t black_key,
			  size_t black_key_len, u8 color, u8 key_enc,
			  u8 trusted_key, u8 mem_type, const void *key_mod,
			  size_t key_mod_len, dma_addr_t blob, size_t blob_len);

/* Decapsulate data from a blob */
int cnstr_desc_blob_decap(u32 **desc, dma_addr_t blob, size_t blob_len,
			  const void *key_mod, size_t key_mod_len,
			  dma_addr_t black_key, size_t black_key_len,
			  u8 keycolor, u8 key_enc, u8 trusted_key, u8 mem_type);

#endif /* _CAAMKEYBLOB_DESC_H_ */
