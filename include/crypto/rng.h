/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * RNG: Random Number Generator  algorithms under the crypto API
 *
 * Copyright (c) 2008 Neil Horman <nhorman@tuxdriver.com>
 * Copyright (c) 2015 Herbert Xu <herbert@gondor.apana.org.au>
 */

#ifndef _CRYPTO_RNG_H
#define _CRYPTO_RNG_H

#include <linux/fips.h>
#include <linux/random.h>

int __crypto_stdrng_get_bytes(void *buf, unsigned int len);

/**
 * crypto_stdrng_get_bytes() - get cryptographically secure random bytes
 * @buf: output buffer holding the random numbers
 * @len: length of the output buffer
 *
 * This function fills the given buffer with random numbers using the normal
 * Linux RNG if fips_enabled=0, or a NIST SP800-90A DRBG if fips_enabled=1.
 *
 * Context: May sleep
 * Return: 0 function was successful; < 0 if an error occurred
 */
static inline int crypto_stdrng_get_bytes(void *buf, unsigned int len)
{
	might_sleep();
	if (fips_enabled)
		return __crypto_stdrng_get_bytes(buf, len);
	return get_random_bytes_wait(buf, len);
}

#ifdef CONFIG_CRYPTO_FIPS
int crypto_del_default_rng(void);
#else
static inline int crypto_del_default_rng(void)
{
	return 0;
}
#endif

#endif
