/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Packet engine driver for Analog Devices Incorporated
 *
 * Currently tested on SC598 processor
 *
 * Copyright (c) 2023 - Timesys Corporation
 *   Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 */

#ifndef ADI_PKTE_SKCIPHER
#define ADI_PKTE_SKCIPHER

#define NUM_CRYPTO_ALGS 6


int adi_crypt_prepare_cipher_req(struct crypto_engine *engine, void *areq);
extern struct skcipher_alg crypto_algs[NUM_CRYPTO_ALGS];

#endif
