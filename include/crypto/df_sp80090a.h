/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright Stephan Mueller <smueller@chronox.de>, 2014
 */

#ifndef _CRYPTO_DF80090A_H
#define _CRYPTO_DF80090A_H

#include <crypto/internal/cipher.h>
#include <crypto/aes.h>
#include <linux/list.h>

/*
 * Concatenation Helper and string operation helper
 *
 * SP800-90A requires the concatenation of different data. To avoid copying
 * buffers around or allocate additional memory, the following data structure
 * is used to point to the original memory with its size. In addition, it
 * is used to build a linked list. The linked list defines the concatenation
 * of individual buffers. The order of memory block referenced in that
 * linked list determines the order of concatenation.
 */
struct drbg_string {
	const unsigned char *buf;
	size_t len;
	struct list_head list;
};

static inline void drbg_string_fill(struct drbg_string *string,
				    const unsigned char *buf, size_t len)
{
	string->buf = buf;
	string->len = len;
	INIT_LIST_HEAD(&string->list);
}

static inline int crypto_drbg_ctr_df_datalen(u8 statelen, u8 blocklen)
{
	return statelen +       /* df_data */
		blocklen +      /* pad */
		blocklen +      /* iv */
		statelen + blocklen;  /* temp */
}

int crypto_drbg_ctr_df(struct aes_enckey *aes,
		       unsigned char *df_data,
		       size_t bytes_to_return,
		       struct list_head *seedlist,
		       u8 blocklen_bytes,
		       u8 statelen);

#endif /* _CRYPTO_DF80090A_H */
