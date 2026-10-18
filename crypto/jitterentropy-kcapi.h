/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2026 Google LLC
 */
#ifndef _CRYPTO_JITTERENTROPY_KCAPI_H
#define _CRYPTO_JITTERENTROPY_KCAPI_H

#include <linux/types.h>

struct jitterentropy;

struct jitterentropy *crypto_jent_alloc(void);
int crypto_jent_get_bytes(struct jitterentropy *rng, u8 *out,
			  unsigned int out_len);
void crypto_jent_free(struct jitterentropy *rng);

#endif /* _CRYPTO_JITTERENTROPY_KCAPI_H */
