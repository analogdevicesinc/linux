/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 *   Copyright (C) 2019 Samsung Electronics Co., Ltd.
 */

#ifndef __CRYPTO_CTX_H__
#define __CRYPTO_CTX_H__

#include <crypto/aead.h>

enum {
	CRYPTO_AEAD_AES_GCM = 16,
	CRYPTO_AEAD_AES_CCM,
	CRYPTO_AEAD_MAX,
};

struct ksmbd_crypto_ctx {
	struct list_head		list;

	struct crypto_aead		*ccmaes[CRYPTO_AEAD_MAX];
};

#define CRYPTO_GCM(c)		((c)->ccmaes[CRYPTO_AEAD_AES_GCM])
#define CRYPTO_CCM(c)		((c)->ccmaes[CRYPTO_AEAD_AES_CCM])

void ksmbd_release_crypto_ctx(struct ksmbd_crypto_ctx *ctx);
struct ksmbd_crypto_ctx *ksmbd_crypto_ctx_find_gcm(void);
struct ksmbd_crypto_ctx *ksmbd_crypto_ctx_find_ccm(void);
void ksmbd_crypto_destroy(void);
int ksmbd_crypto_create(void);

#endif /* __CRYPTO_CTX_H__ */
