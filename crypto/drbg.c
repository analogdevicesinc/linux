/*
 * DRBG: Deterministic Random Bits Generator
 *       Implementation of the HMAC SHA-512 DRBG from NIST SP800-90A,
 *       both with and without prediction resistance
 *
 * Copyright Stephan Mueller <smueller@chronox.de>, 2014
 * Copyright 2026 Google LLC
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, and the entire permission notice in its entirety,
 *    including the disclaimer of warranties.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * ALTERNATIVELY, this product may be distributed under the terms of
 * the GNU General Public License, in which case the provisions of the GPL are
 * required INSTEAD OF the above restrictions.  (This clause is
 * necessary due to a potential bad interaction between the GPL and
 * the restrictions contained in a BSD-style copyright.)
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ALL OF
 * WHICH ARE HEREBY DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF NOT ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * DRBG Usage
 * ==========
 * The SP 800-90A DRBG allows the user to specify a personalization string
 * for initialization as well as an additional information string for each
 * random number request. The following code fragments show how a caller
 * uses the kernel crypto API to use the full functionality of the DRBG.
 *
 * Usage without any additional data
 * ---------------------------------
 * struct crypto_rng *drng;
 * int err;
 * char data[DATALEN];
 *
 * drng = crypto_alloc_rng(drng_name, 0, 0);
 * err = crypto_rng_get_bytes(drng, data, DATALEN);
 * crypto_free_rng(drng);
 *
 *
 * Usage with personalization string during initialization
 * -------------------------------------------------------
 * struct crypto_rng *drng;
 * int err;
 * char data[DATALEN];
 * char personalization[11] = "some-string";
 *
 * drng = crypto_alloc_rng(drng_name, 0, 0);
 * // The reset completely re-initializes the DRBG with the provided
 * // personalization string
 * err = crypto_rng_reset(drng, personalization, strlen(personalization));
 * err = crypto_rng_get_bytes(drng, data, DATALEN);
 * crypto_free_rng(drng);
 *
 *
 * Usage with additional information string during random number request
 * ---------------------------------------------------------------------
 * struct crypto_rng *drng;
 * int err;
 * char data[DATALEN];
 * char addtl_string[11] = "some-string";
 *
 * drng = crypto_alloc_rng(drng_name, 0, 0);
 * err = crypto_rng_generate(drng, addtl_string, strlen(addtl_string),
			     data, DATALEN);
 * crypto_free_rng(drng);
 *
 *
 * Usage with personalization and additional information strings
 * -------------------------------------------------------------
 * Just mix both scenarios above.
 */

#include <crypto/internal/drbg.h>
#include <crypto/internal/rng.h>
#include <crypto/sha2.h>
#include <linux/fips.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/string_choices.h>
#include <linux/unaligned.h>

enum drbg_seed_state {
	DRBG_SEED_STATE_UNSEEDED,
	DRBG_SEED_STATE_PARTIAL, /* Seeded with !rng_is_initialized() */
	DRBG_SEED_STATE_FULL,
};

/* State length in bytes */
#define DRBG_STATE_LEN		SHA512_DIGEST_SIZE

/* Security strength in bytes */
#define DRBG_SEC_STRENGTH	(SHA512_DIGEST_SIZE / 2)

/*
 * Maximum number of requests before reseeding is forced.
 * SP800-90A allows this to be up to 2**48.  We use a lower value.
 */
#define DRBG_MAX_REQUESTS	(1 << 20)

/*
 * Maximum number of random bytes that can be requested at once.
 * SP800-90A allows up to 2**19 bits, which is 2**16 bytes.
 */
#define DRBG_MAX_REQUEST_BYTES	(1 << 16)

/*
 * Maximum length of additional info and personalization strings, in bytes.
 * SP800-90A allows up to 2**35 bits, i.e. 2**32 bytes.  We use 2**32 - 2 bytes
 * so that the value never quite completely fills the range of a size_t,
 * allowing the health check to verify that larger values are rejected.
 */
#define DRBG_MAX_ADDTL		(U32_MAX - 1)

struct drbg_state {
	struct mutex drbg_mutex;	/* lock around DRBG */
	u8 V[DRBG_STATE_LEN];		/* internal state -- 10.1.2.1 1a */
	struct hmac_sha512_key key;	/* current key -- 10.1.2.1 1b */
	u8 C[DRBG_STATE_LEN];		/* current key -- 10.1.2.1 1b */
	/* Number of RNG requests since last reseed -- 10.1.2.1 1c */
	size_t reseed_ctr;
	size_t reseed_threshold;
	enum drbg_seed_state seeded;		/* DRBG fully seeded? */
	unsigned long last_seed_time;
	bool instantiated;
	bool pr;		/* Prediction resistance enabled? */
	struct crypto_rng *jent;
	struct drbg_string test_data;
};

enum drbg_prefixes {
	DRBG_PREFIX0 = 0x00,
	DRBG_PREFIX1,
};

static int drbg_uninstantiate(struct drbg_state *drbg);

/******************************************************************
 * HMAC DRBG functions
 ******************************************************************/

/* update function of HMAC DRBG as defined in 10.1.2.2 */
static void drbg_hmac_update(struct drbg_state *drbg, struct list_head *seed,
			     int reseed)
{
	int i = 0;
	struct hmac_sha512_ctx hmac_ctx;

	if (!reseed) {
		/* 10.1.2.3 step 2 -- memset(0) of C is implicit with kzalloc */
		memset(drbg->V, 1, DRBG_STATE_LEN);
		hmac_sha512_preparekey(&drbg->key, drbg->C, DRBG_STATE_LEN);
	}

	for (i = 2; 0 < i; i--) {
		/* first round uses 0x0, second 0x1 */
		unsigned char prefix = DRBG_PREFIX0;
		if (1 == i)
			prefix = DRBG_PREFIX1;
		/* 10.1.2.2 step 1 and 4 -- concatenation and HMAC for key */
		hmac_sha512_init(&hmac_ctx, &drbg->key);
		hmac_sha512_update(&hmac_ctx, drbg->V, DRBG_STATE_LEN);
		hmac_sha512_update(&hmac_ctx, &prefix, 1);
		if (seed) {
			struct drbg_string *input;

			list_for_each_entry(input, seed, list)
				hmac_sha512_update(&hmac_ctx, input->buf,
						   input->len);
		}
		hmac_sha512_final(&hmac_ctx, drbg->C);
		hmac_sha512_preparekey(&drbg->key, drbg->C, DRBG_STATE_LEN);

		/* 10.1.2.2 step 2 and 5 -- HMAC for V */
		hmac_sha512(&drbg->key, drbg->V, DRBG_STATE_LEN, drbg->V);

		/* 10.1.2.2 step 3 */
		if (!seed)
			break;
	}
}

/* generate function of HMAC DRBG as defined in 10.1.2.5 */
static void drbg_hmac_generate(struct drbg_state *drbg,
			       unsigned char *buf,
			       unsigned int buflen,
			       struct list_head *addtl)
{
	int len = 0;

	/* 10.1.2.5 step 2 */
	if (addtl && !list_empty(addtl))
		drbg_hmac_update(drbg, addtl, 1);

	while (len < buflen) {
		unsigned int outlen = 0;

		/* 10.1.2.5 step 4.1 */
		hmac_sha512(&drbg->key, drbg->V, DRBG_STATE_LEN, drbg->V);
		outlen = (DRBG_STATE_LEN < (buflen - len)) ?
			  DRBG_STATE_LEN : (buflen - len);

		/* 10.1.2.5 step 4.2 */
		memcpy(buf + len, drbg->V, outlen);
		len += outlen;
	}

	/* 10.1.2.5 step 6 */
	if (addtl && !list_empty(addtl))
		drbg_hmac_update(drbg, addtl, 1);
	else
		drbg_hmac_update(drbg, NULL, 1);
}

static inline void __drbg_seed(struct drbg_state *drbg, struct list_head *seed,
			       int reseed, enum drbg_seed_state new_seed_state)
{
	drbg_hmac_update(drbg, seed, reseed);

	drbg->seeded = new_seed_state;
	drbg->last_seed_time = jiffies;
	drbg->reseed_ctr = 1;

	switch (drbg->seeded) {
	case DRBG_SEED_STATE_UNSEEDED:
		/* Impossible, but handle it to silence compiler warnings. */
		fallthrough;
	case DRBG_SEED_STATE_PARTIAL:
		/*
		 * Require frequent reseeds until the seed source is
		 * fully initialized.
		 */
		drbg->reseed_threshold = 50;
		break;

	case DRBG_SEED_STATE_FULL:
		/*
		 * Seed source has become fully initialized, frequent
		 * reseeds no longer required.
		 */
		drbg->reseed_threshold = DRBG_MAX_REQUESTS;
		break;
	}
}

static void drbg_seed_from_random(struct drbg_state *drbg)
	__must_hold(&drbg->drbg_mutex)
{
	struct drbg_string data;
	LIST_HEAD(seedlist);
	unsigned char entropy[DRBG_SEC_STRENGTH];

	drbg_string_fill(&data, entropy, DRBG_SEC_STRENGTH);
	list_add_tail(&data.list, &seedlist);

	get_random_bytes(entropy, DRBG_SEC_STRENGTH);

	__drbg_seed(drbg, &seedlist, true, DRBG_SEED_STATE_FULL);

	memzero_explicit(entropy, DRBG_SEC_STRENGTH);
}

static bool drbg_nopr_reseed_interval_elapsed(struct drbg_state *drbg)
{
	unsigned long next_reseed;

	/* Don't ever reseed from get_random_bytes() in test mode. */
	if (list_empty(&drbg->test_data.list))
		return false;

	/*
	 * Obtain fresh entropy for the nopr DRBGs after 300s have
	 * elapsed in order to still achieve sort of partial
	 * prediction resistance over the time domain at least. Note
	 * that the period of 300s has been chosen to match the
	 * CRNG_RESEED_INTERVAL of the get_random_bytes()' chacha
	 * rngs.
	 */
	next_reseed = drbg->last_seed_time + 300 * HZ;
	return time_after(jiffies, next_reseed);
}

/*
 * Seeding or reseeding of the DRBG
 *
 * @drbg: DRBG state struct
 * @pers: personalization / additional information buffer
 * @reseed: 0 for initial seed process, 1 for reseeding
 *
 * return:
 *	0 on success
 *	error value otherwise
 */
static int drbg_seed(struct drbg_state *drbg, struct drbg_string *pers,
		     bool reseed)
	__must_hold(&drbg->drbg_mutex)
{
	int ret;
	unsigned char entropy[((32 + 16) * 2)];
	unsigned int entropylen;
	struct drbg_string data1;
	LIST_HEAD(seedlist);
	enum drbg_seed_state new_seed_state = DRBG_SEED_STATE_FULL;

	/* 9.1 / 9.2 / 9.3.1 step 3 */
	if (pers && pers->len > DRBG_MAX_ADDTL) {
		pr_devel("DRBG: personalization string too long %zu\n",
			 pers->len);
		return -EINVAL;
	}

	if (list_empty(&drbg->test_data.list)) {
		drbg_string_fill(&data1, drbg->test_data.buf,
				 drbg->test_data.len);
		pr_devel("DRBG: using test entropy\n");
	} else {
		/*
		 * Gather entropy equal to the security strength of the DRBG.
		 * With a derivation function, a nonce is required in addition
		 * to the entropy. A nonce must be at least 1/2 of the security
		 * strength of the DRBG in size. Thus, entropy + nonce is 3/2
		 * of the strength. The consideration of a nonce is only
		 * applicable during initial seeding.
		 */
		if (!reseed)
			entropylen = ((DRBG_SEC_STRENGTH + 1) / 2) * 3;
		else
			entropylen = DRBG_SEC_STRENGTH;
		BUG_ON((entropylen * 2) > sizeof(entropy));

		/* Get seed from in-kernel /dev/urandom */
		if (!rng_is_initialized())
			new_seed_state = DRBG_SEED_STATE_PARTIAL;

		get_random_bytes(entropy, entropylen);

		if (!drbg->jent) {
			drbg_string_fill(&data1, entropy, entropylen);
			pr_devel("DRBG: (re)seeding with %u bytes of entropy\n",
				 entropylen);
		} else {
			/*
			 * Get seed from Jitter RNG, failures are
			 * fatal only in FIPS mode.
			 */
			ret = crypto_rng_get_bytes(drbg->jent,
						   entropy + entropylen,
						   entropylen);
			if (fips_enabled && ret) {
				pr_devel("DRBG: jent failed with %d\n", ret);

				/*
				 * Do not treat the transient failure of the
				 * Jitter RNG as an error that needs to be
				 * reported. The combined number of the
				 * maximum reseed threshold times the maximum
				 * number of Jitter RNG transient errors is
				 * less than the reseed threshold required by
				 * SP800-90A allowing us to treat the
				 * transient errors as such.
				 *
				 * However, we mandate that at least the first
				 * seeding operation must succeed with the
				 * Jitter RNG.
				 */
				if (!reseed || ret != -EAGAIN)
					goto out;
			}

			drbg_string_fill(&data1, entropy, entropylen * 2);
			pr_devel("DRBG: (re)seeding with %u bytes of entropy\n",
				 entropylen * 2);
		}
	}
	list_add_tail(&data1.list, &seedlist);

	/*
	 * concatenation of entropy with personalization str / addtl input)
	 * the variable pers is directly handed in by the caller, so check its
	 * contents whether it is appropriate
	 */
	if (pers && pers->buf && 0 < pers->len) {
		list_add_tail(&pers->list, &seedlist);
		pr_devel("DRBG: using personalization string\n");
	}

	if (!reseed) {
		memset(drbg->V, 0, DRBG_STATE_LEN);
		memset(drbg->C, 0, DRBG_STATE_LEN);
	}

	__drbg_seed(drbg, &seedlist, reseed, new_seed_state);
	ret = 0;
out:
	memzero_explicit(entropy, sizeof(entropy));

	return ret;
}

/* Free all substructures in a DRBG state without the DRBG state structure */
static inline void drbg_dealloc_state(struct drbg_state *drbg)
{
	if (!drbg)
		return;
	memzero_explicit(&drbg->key, sizeof(drbg->key));
	memzero_explicit(drbg->V, sizeof(drbg->V));
	memzero_explicit(drbg->C, sizeof(drbg->C));
	drbg->reseed_ctr = 0;
	drbg->instantiated = false;
}

/*
 * DRBG generate function as required by SP800-90A - this function
 * generates random numbers
 *
 * @drbg DRBG state handle
 * @buf Buffer where to store the random numbers -- the buffer must already
 *      be pre-allocated by caller
 * @buflen Length of output buffer - this value defines the number of random
 *	   bytes pulled from DRBG
 * @addtl Additional input that is mixed into state, may be NULL -- note
 *	  the entropy is pulled by the DRBG internally unconditionally
 *	  as defined in SP800-90A. The additional input is mixed into
 *	  the state in addition to the pulled entropy.
 *
 * return: 0 when all bytes are generated; < 0 in case of an error
 */
static int drbg_generate(struct drbg_state *drbg,
			 unsigned char *buf, unsigned int buflen,
			 struct drbg_string *addtl)
	__must_hold(&drbg->drbg_mutex)
{
	int len = 0;
	LIST_HEAD(addtllist);

	if (!drbg->instantiated) {
		pr_devel("DRBG: not yet instantiated\n");
		return -EINVAL;
	}
	if (0 == buflen || !buf) {
		pr_devel("DRBG: no output buffer provided\n");
		return -EINVAL;
	}
	if (addtl && NULL == addtl->buf && 0 < addtl->len) {
		pr_devel("DRBG: wrong format of additional information\n");
		return -EINVAL;
	}

	/* 9.3.1 step 2 */
	if (buflen > DRBG_MAX_REQUEST_BYTES) {
		pr_devel("DRBG: requested random numbers too large %u\n",
			 buflen);
		return -EINVAL;
	}

	/* 9.3.1 step 3 is implicit with the chosen DRBG */

	/* 9.3.1 step 4 */
	if (addtl && addtl->len > DRBG_MAX_ADDTL) {
		pr_devel("DRBG: additional information string too long %zu\n",
			 addtl->len);
		return -EINVAL;
	}
	/* 9.3.1 step 5 is implicit with the chosen DRBG */

	/*
	 * 9.3.1 step 6 and 9 supplemented by 9.3.2 step c is implemented
	 * here. The spec is a bit convoluted here, we make it simpler.
	 */
	if (drbg->reseed_threshold < drbg->reseed_ctr)
		drbg->seeded = DRBG_SEED_STATE_UNSEEDED;

	if (drbg->pr || drbg->seeded == DRBG_SEED_STATE_UNSEEDED) {
		pr_devel("DRBG: reseeding before generation (prediction "
			 "resistance: %s, state %s)\n",
			 str_true_false(drbg->pr),
			 (drbg->seeded ==  DRBG_SEED_STATE_FULL ?
			  "seeded" : "unseeded"));
		/* 9.3.1 steps 7.1 through 7.3 */
		len = drbg_seed(drbg, addtl, true);
		if (len)
			goto err;
		/* 9.3.1 step 7.4 */
		addtl = NULL;
	} else if (rng_is_initialized() &&
		   (drbg->seeded == DRBG_SEED_STATE_PARTIAL ||
		    drbg_nopr_reseed_interval_elapsed(drbg))) {
		drbg_seed_from_random(drbg);
	}

	if (addtl && 0 < addtl->len)
		list_add_tail(&addtl->list, &addtllist);
	/* 9.3.1 step 8 and 10 */
	drbg_hmac_generate(drbg, buf, buflen, &addtllist);

	/* 10.1.2.5 step 7 */
	drbg->reseed_ctr++;

	/*
	 * Section 11.3.3 requires to re-perform self tests after some
	 * generated random numbers. The chosen value after which self
	 * test is performed is arbitrary, but it should be reasonable.
	 * However, we do not perform the self tests because of the following
	 * reasons: it is mathematically impossible that the initial self tests
	 * were successfully and the following are not. If the initial would
	 * pass and the following would not, the kernel integrity is violated.
	 * In this case, the entire kernel operation is questionable and it
	 * is unlikely that the integrity violation only affects the
	 * correct operation of the DRBG.
	 */

	/*
	 * All operations were successful, return 0 as mandated by
	 * the kernel crypto API interface.
	 */
	len = 0;
err:
	return len;
}

/*
 * Wrapper around drbg_generate which can pull arbitrary long strings
 * from the DRBG without hitting the maximum request limitation.
 *
 * Parameters: see drbg_generate
 * Return codes: see drbg_generate -- if one drbg_generate request fails,
 *		 the entire drbg_generate_long request fails
 */
static int drbg_generate_long(struct drbg_state *drbg,
			      unsigned char *buf, unsigned int buflen,
			      struct drbg_string *addtl)
{
	unsigned int len = 0;
	unsigned int slice = 0;
	do {
		int err = 0;
		unsigned int chunk = 0;
		slice = (buflen - len) / DRBG_MAX_REQUEST_BYTES;
		chunk = slice ? DRBG_MAX_REQUEST_BYTES : (buflen - len);
		mutex_lock(&drbg->drbg_mutex);
		err = drbg_generate(drbg, buf + len, chunk, addtl);
		mutex_unlock(&drbg->drbg_mutex);
		if (0 > err)
			return err;
		len += chunk;
	} while (slice > 0 && (len < buflen));
	return 0;
}

static int drbg_prepare_hrng(struct drbg_state *drbg)
{
	/* We do not need an HRNG in test mode. */
	if (list_empty(&drbg->test_data.list))
		return 0;

	drbg->jent = crypto_alloc_rng("jitterentropy_rng", 0, 0);
	if (IS_ERR(drbg->jent)) {
		const int err = PTR_ERR(drbg->jent);

		drbg->jent = NULL;
		if (fips_enabled)
			return err;
		pr_info("DRBG: Continuing without Jitter RNG\n");
	}

	return 0;
}

/*
 * DRBG instantiation function as required by SP800-90A - this function
 * sets up the DRBG handle, performs the initial seeding and all sanity
 * checks required by SP800-90A
 *
 * @drbg memory of state -- if NULL, new memory is allocated
 * @pers Personalization string that is mixed into state, may be NULL -- note
 *	 the entropy is pulled by the DRBG internally unconditionally
 *	 as defined in SP800-90A. The additional input is mixed into
 *	 the state in addition to the pulled entropy.
 * @pr prediction resistance enabled
 *
 * return
 *	0 on success
 *	error value otherwise
 */
static int drbg_instantiate(struct drbg_state *drbg, struct drbg_string *pers,
			    bool pr)
{
	int ret;
	bool reseed = true;

	pr_devel("DRBG: Initializing DRBG with prediction resistance %s\n",
		 str_enabled_disabled(pr));
	mutex_lock(&drbg->drbg_mutex);

	/* 9.1 step 1 is implicit with the selected DRBG type */

	/*
	 * 9.1 step 2 is implicit as caller can select prediction resistance
	 * all DRBG types support prediction resistance
	 */

	/* 9.1 step 4 is implicit in DRBG_SEC_STRENGTH */

	if (!drbg->instantiated) {
		drbg->instantiated = true;
		drbg->pr = pr;
		drbg->seeded = DRBG_SEED_STATE_UNSEEDED;
		drbg->last_seed_time = 0;
		drbg->reseed_threshold = DRBG_MAX_REQUESTS;

		ret = drbg_prepare_hrng(drbg);
		if (ret)
			goto free_everything;

		reseed = false;
	}

	ret = drbg_seed(drbg, pers, reseed);

	if (ret && !reseed)
		goto free_everything;

	mutex_unlock(&drbg->drbg_mutex);
	return ret;

free_everything:
	mutex_unlock(&drbg->drbg_mutex);
	drbg_uninstantiate(drbg);
	return ret;
}

/*
 * DRBG uninstantiate function as required by SP800-90A - this function
 * frees all buffers and the DRBG handle
 *
 * @drbg DRBG state handle
 *
 * return
 *	0 on success
 */
static int drbg_uninstantiate(struct drbg_state *drbg)
{
	if (!IS_ERR_OR_NULL(drbg->jent))
		crypto_free_rng(drbg->jent);
	drbg->jent = NULL;

	drbg_dealloc_state(drbg);
	/* no scrubbing of test_data -- this shall survive an uninstantiate */
	return 0;
}

/*
 * Helper function for setting the test data in the DRBG
 *
 * @drbg DRBG state handle
 * @data test data
 * @len test data length
 */
static void drbg_kcapi_set_entropy(struct crypto_rng *tfm,
				   const u8 *data, unsigned int len)
{
	struct drbg_state *drbg = crypto_rng_ctx(tfm);

	mutex_lock(&drbg->drbg_mutex);
	drbg_string_fill(&drbg->test_data, data, len);
	mutex_unlock(&drbg->drbg_mutex);
}

/***************************************************************
 * Kernel crypto API interface to register DRBG
 ***************************************************************/

static int drbg_kcapi_init(struct crypto_tfm *tfm)
{
	struct drbg_state *drbg = crypto_tfm_ctx(tfm);

	mutex_init(&drbg->drbg_mutex);

	return 0;
}

static void drbg_kcapi_cleanup(struct crypto_tfm *tfm)
{
	drbg_uninstantiate(crypto_tfm_ctx(tfm));
}

/*
 * Generate random numbers invoked by the kernel crypto API:
 * The API of the kernel crypto API is extended as follows:
 *
 * src is additional input supplied to the RNG.
 * slen is the length of src.
 * dst is the output buffer where random data is to be stored.
 * dlen is the length of dst.
 */
static int drbg_kcapi_random(struct crypto_rng *tfm,
			     const u8 *src, unsigned int slen,
			     u8 *dst, unsigned int dlen)
{
	struct drbg_state *drbg = crypto_rng_ctx(tfm);
	struct drbg_string *addtl = NULL;
	struct drbg_string string;

	if (slen) {
		/* linked list variable is now local to allow modification */
		drbg_string_fill(&string, src, slen);
		addtl = &string;
	}

	return drbg_generate_long(drbg, dst, dlen, addtl);
}

/* Seed (i.e. instantiate) or re-seed the DRBG. */
static int drbg_kcapi_seed(struct crypto_rng *tfm,
			   const u8 *seed, unsigned int slen, bool pr)
{
	struct drbg_state *drbg = crypto_rng_ctx(tfm);
	struct drbg_string string;
	struct drbg_string *seed_string = NULL;

	if (0 < slen) {
		drbg_string_fill(&string, seed, slen);
		seed_string = &string;
	}

	return drbg_instantiate(drbg, seed_string, pr);
}

static int drbg_kcapi_seed_pr(struct crypto_rng *tfm,
			      const u8 *seed, unsigned int slen)
{
	return drbg_kcapi_seed(tfm, seed, slen, /* pr= */ true);
}

static int drbg_kcapi_seed_nopr(struct crypto_rng *tfm,
				const u8 *seed, unsigned int slen)
{
	return drbg_kcapi_seed(tfm, seed, slen, /* pr= */ false);
}

/***************************************************************
 * Kernel module: code to load the module
 ***************************************************************/

/*
 * Tests as defined in 11.3.2 in addition to the cipher tests: testing
 * of the error handling.
 *
 * Note: testing of failing seed source as defined in 11.3.2 is not applicable
 * as seed source of get_random_bytes does not fail.
 *
 * Note 2: There is no sensible way of testing the reseed counter
 * enforcement, so skip it.
 */
static inline int __init drbg_healthcheck_sanity(void)
{
#define OUTBUFLEN 16
	unsigned char buf[OUTBUFLEN];
	struct drbg_state *drbg = NULL;
	int ret;
	int rc = -EFAULT;
	struct drbg_string addtl;

	/* only perform test in FIPS mode */
	if (!fips_enabled)
		return 0;

	drbg = kzalloc_obj(struct drbg_state);
	if (!drbg)
		return -ENOMEM;

	guard(mutex_init)(&drbg->drbg_mutex);
	drbg->instantiated = true;
	drbg->reseed_threshold = DRBG_MAX_REQUESTS;

	/*
	 * if the following tests fail, it is likely that there is a buffer
	 * overflow as buf is much smaller than the requested or provided
	 * string lengths -- in case the error handling does not succeed
	 * we may get an OOPS. And we want to get an OOPS as this is a
	 * grave bug.
	 */

	drbg_string_fill(&addtl, buf, DRBG_MAX_ADDTL + 1);
	/* overflow addtllen with additional info string */
	ret = drbg_generate(drbg, buf, OUTBUFLEN, &addtl);
	BUG_ON(ret == 0);
	/* overflow max_bits */
	ret = drbg_generate(drbg, buf, DRBG_MAX_REQUEST_BYTES + 1, NULL);
	BUG_ON(ret == 0);

	/* overflow max addtllen with personalization string */
	ret = drbg_seed(drbg, &addtl, false);
	BUG_ON(0 == ret);
	/* all tests passed */
	rc = 0;

	pr_devel("DRBG: Sanity tests for failure code paths successfully "
		 "completed\n");

	kfree(drbg);
	return rc;
}

static struct rng_alg drbg_algs[] = {
	{
		.base.cra_name		= "stdrng",
		.base.cra_driver_name	= "drbg_pr_hmac_sha512",
		.base.cra_priority	= 200,
		.base.cra_ctxsize	= sizeof(struct drbg_state),
		.base.cra_module	= THIS_MODULE,
		.base.cra_init		= drbg_kcapi_init,
		.set_ent		= drbg_kcapi_set_entropy,
		.seed			= drbg_kcapi_seed_pr,
		.generate		= drbg_kcapi_random,
		.base.cra_exit		= drbg_kcapi_cleanup,
	},
	{
		.base.cra_name		= "stdrng",
		.base.cra_driver_name	= "drbg_nopr_hmac_sha512",
		.base.cra_priority	= 201,
		.base.cra_ctxsize	= sizeof(struct drbg_state),
		.base.cra_module	= THIS_MODULE,
		.base.cra_init		= drbg_kcapi_init,
		.set_ent		= drbg_kcapi_set_entropy,
		.seed			= drbg_kcapi_seed_nopr,
		.generate		= drbg_kcapi_random,
		.base.cra_exit		= drbg_kcapi_cleanup,
	},
};

static int __init drbg_init(void)
{
	int ret;

	ret = drbg_healthcheck_sanity();
	if (ret)
		return ret;

	/*
	 * In FIPS mode, boost the algorithm priorities to ensure that when
	 * users request "stdrng", they really get an algorithm from here.
	 */
	if (fips_enabled) {
		for (size_t i = 0; i < ARRAY_SIZE(drbg_algs); i++)
			drbg_algs[i].base.cra_priority += 2000;
	}

	return crypto_register_rngs(drbg_algs, ARRAY_SIZE(drbg_algs));
}

static void __exit drbg_exit(void)
{
	crypto_unregister_rngs(drbg_algs, ARRAY_SIZE(drbg_algs));
}

module_init(drbg_init);
module_exit(drbg_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stephan Mueller <smueller@chronox.de>");
MODULE_DESCRIPTION("NIST SP800-90A Deterministic Random Bit Generator (DRBG)");
MODULE_ALIAS_CRYPTO("stdrng");
MODULE_ALIAS_CRYPTO("drbg_pr_hmac_sha512");
MODULE_ALIAS_CRYPTO("drbg_nopr_hmac_sha512");
