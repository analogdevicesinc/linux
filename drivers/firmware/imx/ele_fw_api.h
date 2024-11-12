/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024 NXP
 */

#ifndef ELE_FW_API_H
#define ELE_FW_API_H

#include <linux/hw_random.h>

#define MESSAGING_VERSION_7		0x7

#define ELE_INIT_FW_REQ                 0x17
#define ELE_INIT_FW_REQ_SZ              0x04
#define ELE_INIT_FW_RSP_SZ              0x08

#define ELE_GET_RANDOM_REQ		0xCD
#define ELE_GET_RANDOM_REQ_SZ		0x10
#define ELE_GET_RANDOM_RSP_SZ		0x08
#define ELE_RNG_MAX_SIZE		16

int ele_init_fw(struct se_if_priv *priv);
int ele_get_random(struct se_if_priv *priv, void *data, size_t len);
int ele_get_hwrng(struct hwrng *rng, void *data, size_t len, bool wait);

#endif /* ELE_FW_API_H */
