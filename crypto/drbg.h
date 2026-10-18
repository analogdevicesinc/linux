/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2026 Google LLC
 */
#ifndef _CRYPTO_DRBG_H
#define _CRYPTO_DRBG_H

#include <linux/types.h>

struct drbg_state;

struct drbg_state *crypto_drbg_alloc(void);
void crypto_drbg_set_entropy(struct drbg_state *drbg, const u8 *data,
			     size_t len);
int crypto_drbg_seed(struct drbg_state *drbg, const u8 *pers, size_t pers_len);
int crypto_drbg_get_bytes(struct drbg_state *drbg, u8 *out, size_t out_len,
			  const u8 *addtl, size_t addtl_len);
void crypto_drbg_free(struct drbg_state *drbg);

#endif /* _CRYPTO_DRBG_H */
