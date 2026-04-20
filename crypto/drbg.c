/*
 * DRBG: Deterministic Random Bits Generator
 *       Implementation of the HMAC SHA-512 DRBG from NIST SP800-90A,
 *       both with and without prediction resistance
 *
 * Copyright Stephan Mueller <smueller@chronox.de>, 2014
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
#include <crypto/hash.h>
#include <linux/fips.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/string_choices.h>
#include <linux/unaligned.h>

struct drbg_state;
typedef uint32_t drbg_flag_t;

struct drbg_core {
	drbg_flag_t flags;	/* flags for the cipher */
	__u8 statelen;		/* maximum state length */
	__u8 blocklen_bytes;	/* block size of output in bytes */
	char cra_name[CRYPTO_MAX_ALG_NAME]; /* mapping to kernel crypto API */
	 /* kernel crypto API backend cipher name */
	char backend_cra_name[CRYPTO_MAX_ALG_NAME];
};

struct drbg_state_ops {
	int (*update)(struct drbg_state *drbg, struct list_head *seed,
		      int reseed);
	int (*generate)(struct drbg_state *drbg,
			unsigned char *buf, unsigned int buflen,
			struct list_head *addtl);
	int (*crypto_init)(struct drbg_state *drbg);
	int (*crypto_fini)(struct drbg_state *drbg);

};

enum drbg_seed_state {
	DRBG_SEED_STATE_UNSEEDED,
	DRBG_SEED_STATE_PARTIAL, /* Seeded with !rng_is_initialized() */
	DRBG_SEED_STATE_FULL,
};

struct drbg_state {
	struct mutex drbg_mutex;	/* lock around DRBG */
	unsigned char *V;	/* internal state -- 10.1.2.1 1a */
	unsigned char *Vbuf;
	unsigned char *C;	/* current key -- 10.1.2.1 1b */
	unsigned char *Cbuf;
	/* Number of RNG requests since last reseed -- 10.1.2.1 1c */
	size_t reseed_ctr;
	size_t reseed_threshold;
	void *priv_data;	/* Cipher handle */

	enum drbg_seed_state seeded;		/* DRBG fully seeded? */
	unsigned long last_seed_time;
	bool pr;		/* Prediction resistance enabled? */
	struct crypto_rng *jent;
	const struct drbg_state_ops *d_ops;
	const struct drbg_core *core;
	struct drbg_string test_data;
};

static inline __u8 drbg_statelen(struct drbg_state *drbg)
{
	if (drbg && drbg->core)
		return drbg->core->statelen;
	return 0;
}

static inline __u8 drbg_blocklen(struct drbg_state *drbg)
{
	if (drbg && drbg->core)
		return drbg->core->blocklen_bytes;
	return 0;
}

static inline size_t drbg_max_request_bytes(struct drbg_state *drbg)
{
	/* SP800-90A requires the limit 2**19 bits, but we return bytes */
	return (1 << 16);
}

/*
 * SP800-90A allows implementations to support additional info / personalization
 * strings of up to 2**35 bits.  Implementations can have a smaller maximum.  We
 * use 2**35 - 16 bits == U32_MAX - 1 bytes so that the max + 1 always fits in a
 * size_t, allowing drbg_healthcheck_sanity() to verify its enforcement.
 */
static inline size_t drbg_max_addtl(struct drbg_state *drbg)
{
	return U32_MAX - 1;
}

static inline size_t drbg_max_requests(struct drbg_state *drbg)
{
	/* SP800-90A requires 2**48 maximum requests before reseeding */
	return (1<<20);
}

/* DRBG type flags */
#define DRBG_HMAC	((drbg_flag_t)1<<1)
#define DRBG_TYPE_MASK	DRBG_HMAC
/* DRBG strength flags */
#define DRBG_STRENGTH128	((drbg_flag_t)1<<3)
#define DRBG_STRENGTH192	((drbg_flag_t)1<<4)
#define DRBG_STRENGTH256	((drbg_flag_t)1<<5)
#define DRBG_STRENGTH_MASK	(DRBG_STRENGTH128 | DRBG_STRENGTH192 | \
				 DRBG_STRENGTH256)

enum drbg_prefixes {
	DRBG_PREFIX0 = 0x00,
	DRBG_PREFIX1,
};

/***************************************************************
 * Backend cipher definitions available to DRBG
 ***************************************************************/

/*
 * The order of the DRBG definitions here matter: every DRBG is registered
 * as stdrng. Each DRBG receives an increasing cra_priority values the later
 * they are defined in this array (see drbg_fill_array).
 *
 * Thus, the favored DRBGs are the latest entries in this array.
 */
static const struct drbg_core drbg_cores[] = {
	{
		.flags = DRBG_HMAC | DRBG_STRENGTH256,
		.statelen = 64, /* block length of cipher */
		.blocklen_bytes = 64,
		.cra_name = "hmac_sha512",
		.backend_cra_name = "hmac(sha512)",
	},
};

static int drbg_uninstantiate(struct drbg_state *drbg);

/******************************************************************
 * Generic helper functions
 ******************************************************************/

/*
 * Return strength of DRBG according to SP800-90A section 8.4
 *
 * @flags DRBG flags reference
 *
 * Return: normalized strength in *bytes* value or 32 as default
 *	   to counter programming errors
 */
static inline unsigned short drbg_sec_strength(drbg_flag_t flags)
{
	switch (flags & DRBG_STRENGTH_MASK) {
	case DRBG_STRENGTH128:
		return 16;
	case DRBG_STRENGTH192:
		return 24;
	case DRBG_STRENGTH256:
		return 32;
	default:
		return 32;
	}
}

/******************************************************************
 * HMAC DRBG callback functions
 ******************************************************************/

static int drbg_kcapi_hash(struct drbg_state *drbg, unsigned char *outval,
			   const struct list_head *in);
static void drbg_kcapi_hmacsetkey(struct drbg_state *drbg,
				  const unsigned char *key);
static int drbg_init_hash_kernel(struct drbg_state *drbg);
static int drbg_fini_hash_kernel(struct drbg_state *drbg);

MODULE_ALIAS_CRYPTO("drbg_pr_hmac_sha512");
MODULE_ALIAS_CRYPTO("drbg_nopr_hmac_sha512");

/* update function of HMAC DRBG as defined in 10.1.2.2 */
static int drbg_hmac_update(struct drbg_state *drbg, struct list_head *seed,
			    int reseed)
{
	int ret = -EFAULT;
	int i = 0;
	struct drbg_string seed1, seed2, vdata;
	LIST_HEAD(seedlist);
	LIST_HEAD(vdatalist);

	if (!reseed) {
		/* 10.1.2.3 step 2 -- memset(0) of C is implicit with kzalloc */
		memset(drbg->V, 1, drbg_statelen(drbg));
		drbg_kcapi_hmacsetkey(drbg, drbg->C);
	}

	drbg_string_fill(&seed1, drbg->V, drbg_statelen(drbg));
	list_add_tail(&seed1.list, &seedlist);
	/* buffer of seed2 will be filled in for loop below with one byte */
	drbg_string_fill(&seed2, NULL, 1);
	list_add_tail(&seed2.list, &seedlist);
	/* input data of seed is allowed to be NULL at this point */
	if (seed)
		list_splice_tail(seed, &seedlist);

	drbg_string_fill(&vdata, drbg->V, drbg_statelen(drbg));
	list_add_tail(&vdata.list, &vdatalist);
	for (i = 2; 0 < i; i--) {
		/* first round uses 0x0, second 0x1 */
		unsigned char prefix = DRBG_PREFIX0;
		if (1 == i)
			prefix = DRBG_PREFIX1;
		/* 10.1.2.2 step 1 and 4 -- concatenation and HMAC for key */
		seed2.buf = &prefix;
		ret = drbg_kcapi_hash(drbg, drbg->C, &seedlist);
		if (ret)
			return ret;
		drbg_kcapi_hmacsetkey(drbg, drbg->C);

		/* 10.1.2.2 step 2 and 5 -- HMAC for V */
		ret = drbg_kcapi_hash(drbg, drbg->V, &vdatalist);
		if (ret)
			return ret;

		/* 10.1.2.2 step 3 */
		if (!seed)
			return ret;
	}

	return 0;
}

/* generate function of HMAC DRBG as defined in 10.1.2.5 */
static int drbg_hmac_generate(struct drbg_state *drbg,
			      unsigned char *buf,
			      unsigned int buflen,
			      struct list_head *addtl)
{
	int len = 0;
	int ret = 0;
	struct drbg_string data;
	LIST_HEAD(datalist);

	/* 10.1.2.5 step 2 */
	if (addtl && !list_empty(addtl)) {
		ret = drbg_hmac_update(drbg, addtl, 1);
		if (ret)
			return ret;
	}

	drbg_string_fill(&data, drbg->V, drbg_statelen(drbg));
	list_add_tail(&data.list, &datalist);
	while (len < buflen) {
		unsigned int outlen = 0;
		/* 10.1.2.5 step 4.1 */
		ret = drbg_kcapi_hash(drbg, drbg->V, &datalist);
		if (ret)
			return ret;
		outlen = (drbg_blocklen(drbg) < (buflen - len)) ?
			  drbg_blocklen(drbg) : (buflen - len);

		/* 10.1.2.5 step 4.2 */
		memcpy(buf + len, drbg->V, outlen);
		len += outlen;
	}

	/* 10.1.2.5 step 6 */
	if (addtl && !list_empty(addtl))
		ret = drbg_hmac_update(drbg, addtl, 1);
	else
		ret = drbg_hmac_update(drbg, NULL, 1);
	if (ret)
		return ret;

	return len;
}

static const struct drbg_state_ops drbg_hmac_ops = {
	.update		= drbg_hmac_update,
	.generate	= drbg_hmac_generate,
	.crypto_init	= drbg_init_hash_kernel,
	.crypto_fini	= drbg_fini_hash_kernel,
};

/******************************************************************
 * Functions common for DRBG implementations
 ******************************************************************/

static inline int __drbg_seed(struct drbg_state *drbg, struct list_head *seed,
			      int reseed, enum drbg_seed_state new_seed_state)
{
	int ret = drbg->d_ops->update(drbg, seed, reseed);

	if (ret)
		return ret;

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
		drbg->reseed_threshold = drbg_max_requests(drbg);
		break;
	}

	return ret;
}

static int drbg_seed_from_random(struct drbg_state *drbg)
	__must_hold(&drbg->drbg_mutex)
{
	struct drbg_string data;
	LIST_HEAD(seedlist);
	unsigned int entropylen = drbg_sec_strength(drbg->core->flags);
	unsigned char entropy[32];
	int ret;

	BUG_ON(!entropylen);
	BUG_ON(entropylen > sizeof(entropy));

	drbg_string_fill(&data, entropy, entropylen);
	list_add_tail(&data.list, &seedlist);

	get_random_bytes(entropy, entropylen);

	ret = __drbg_seed(drbg, &seedlist, true, DRBG_SEED_STATE_FULL);

	memzero_explicit(entropy, entropylen);
	return ret;
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
	unsigned int entropylen = drbg_sec_strength(drbg->core->flags);
	struct drbg_string data1;
	LIST_HEAD(seedlist);
	enum drbg_seed_state new_seed_state = DRBG_SEED_STATE_FULL;

	/* 9.1 / 9.2 / 9.3.1 step 3 */
	if (pers && pers->len > (drbg_max_addtl(drbg))) {
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
		BUG_ON(!entropylen);
		if (!reseed)
			entropylen = ((entropylen + 1) / 2) * 3;
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
		memset(drbg->V, 0, drbg_statelen(drbg));
		memset(drbg->C, 0, drbg_statelen(drbg));
	}

	ret = __drbg_seed(drbg, &seedlist, reseed, new_seed_state);

out:
	memzero_explicit(entropy, entropylen * 2);

	return ret;
}

/* Free all substructures in a DRBG state without the DRBG state structure */
static inline void drbg_dealloc_state(struct drbg_state *drbg)
{
	if (!drbg)
		return;
	kfree_sensitive(drbg->Vbuf);
	drbg->Vbuf = NULL;
	drbg->V = NULL;
	kfree_sensitive(drbg->Cbuf);
	drbg->Cbuf = NULL;
	drbg->C = NULL;
	drbg->reseed_ctr = 0;
	drbg->d_ops = NULL;
	drbg->core = NULL;
}

/*
 * Allocate all sub-structures for a DRBG state.
 * The DRBG state structure must already be allocated.
 */
static inline int drbg_alloc_state(struct drbg_state *drbg)
{
	int ret = -ENOMEM;

	switch (drbg->core->flags & DRBG_TYPE_MASK) {
	case DRBG_HMAC:
		drbg->d_ops = &drbg_hmac_ops;
		break;
	default:
		ret = -EOPNOTSUPP;
		goto err;
	}

	ret = drbg->d_ops->crypto_init(drbg);
	if (ret < 0)
		goto err;

	drbg->Vbuf = kmalloc(drbg_statelen(drbg) + ret, GFP_KERNEL);
	if (!drbg->Vbuf) {
		ret = -ENOMEM;
		goto fini;
	}
	drbg->V = PTR_ALIGN(drbg->Vbuf, ret + 1);
	drbg->Cbuf = kmalloc(drbg_statelen(drbg) + ret, GFP_KERNEL);
	if (!drbg->Cbuf) {
		ret = -ENOMEM;
		goto fini;
	}
	drbg->C = PTR_ALIGN(drbg->Cbuf, ret + 1);

	return 0;

fini:
	drbg->d_ops->crypto_fini(drbg);
err:
	drbg_dealloc_state(drbg);
	return ret;
}

/*************************************************************************
 * DRBG interface functions
 *************************************************************************/

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

	if (!drbg->core) {
		pr_devel("DRBG: not yet seeded\n");
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
	len = -EINVAL;
	if (buflen > (drbg_max_request_bytes(drbg))) {
		pr_devel("DRBG: requested random numbers too large %u\n",
			 buflen);
		goto err;
	}

	/* 9.3.1 step 3 is implicit with the chosen DRBG */

	/* 9.3.1 step 4 */
	if (addtl && addtl->len > (drbg_max_addtl(drbg))) {
		pr_devel("DRBG: additional information string too long %zu\n",
			 addtl->len);
		goto err;
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
		len = drbg_seed_from_random(drbg);
		if (len)
			goto err;
	}

	if (addtl && 0 < addtl->len)
		list_add_tail(&addtl->list, &addtllist);
	/* 9.3.1 step 8 and 10 */
	len = drbg->d_ops->generate(drbg, buf, buflen, &addtllist);

	/* 10.1.2.5 step 7 */
	drbg->reseed_ctr++;
	if (0 >= len)
		goto err;

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
		slice = ((buflen - len) / drbg_max_request_bytes(drbg));
		chunk = slice ? drbg_max_request_bytes(drbg) : (buflen - len);
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
 * @coreref reference to core
 * @pr prediction resistance enabled
 *
 * return
 *	0 on success
 *	error value otherwise
 */
static int drbg_instantiate(struct drbg_state *drbg, struct drbg_string *pers,
			    int coreref, bool pr)
{
	int ret;
	bool reseed = true;

	pr_devel("DRBG: Initializing DRBG core %d with prediction resistance "
		 "%s\n", coreref, str_enabled_disabled(pr));
	mutex_lock(&drbg->drbg_mutex);

	/* 9.1 step 1 is implicit with the selected DRBG type */

	/*
	 * 9.1 step 2 is implicit as caller can select prediction resistance
	 * and the flag is copied into drbg->flags --
	 * all DRBG types support prediction resistance
	 */

	/* 9.1 step 4 is implicit in  drbg_sec_strength */

	if (!drbg->core) {
		drbg->core = &drbg_cores[coreref];
		drbg->pr = pr;
		drbg->seeded = DRBG_SEED_STATE_UNSEEDED;
		drbg->last_seed_time = 0;
		drbg->reseed_threshold = drbg_max_requests(drbg);

		ret = drbg_alloc_state(drbg);
		if (ret)
			goto unlock;

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

unlock:
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

	if (drbg->d_ops)
		drbg->d_ops->crypto_fini(drbg);
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
 * Kernel crypto API cipher invocations requested by DRBG
 ***************************************************************/

struct sdesc {
	struct shash_desc shash;
};

static int drbg_init_hash_kernel(struct drbg_state *drbg)
{
	struct sdesc *sdesc;
	struct crypto_shash *tfm;

	tfm = crypto_alloc_shash(drbg->core->backend_cra_name, 0, 0);
	if (IS_ERR(tfm)) {
		pr_info("DRBG: could not allocate digest TFM handle: %s\n",
				drbg->core->backend_cra_name);
		return PTR_ERR(tfm);
	}
	BUG_ON(drbg_blocklen(drbg) != crypto_shash_digestsize(tfm));
	sdesc = kzalloc(sizeof(struct shash_desc) + crypto_shash_descsize(tfm),
			GFP_KERNEL);
	if (!sdesc) {
		crypto_free_shash(tfm);
		return -ENOMEM;
	}

	sdesc->shash.tfm = tfm;
	drbg->priv_data = sdesc;

	return 0;
}

static int drbg_fini_hash_kernel(struct drbg_state *drbg)
{
	struct sdesc *sdesc = drbg->priv_data;
	if (sdesc) {
		crypto_free_shash(sdesc->shash.tfm);
		kfree_sensitive(sdesc);
	}
	drbg->priv_data = NULL;
	return 0;
}

static void drbg_kcapi_hmacsetkey(struct drbg_state *drbg,
				  const unsigned char *key)
{
	struct sdesc *sdesc = drbg->priv_data;

	crypto_shash_setkey(sdesc->shash.tfm, key, drbg_statelen(drbg));
}

static int drbg_kcapi_hash(struct drbg_state *drbg, unsigned char *outval,
			   const struct list_head *in)
{
	struct sdesc *sdesc = drbg->priv_data;
	struct drbg_string *input = NULL;

	crypto_shash_init(&sdesc->shash);
	list_for_each_entry(input, in, list)
		crypto_shash_update(&sdesc->shash, input->buf, input->len);
	return crypto_shash_final(&sdesc->shash, outval);
}

/***************************************************************
 * Kernel crypto API interface to register DRBG
 ***************************************************************/

/*
 * Look up the DRBG flags by given kernel crypto API cra_name
 * The code uses the drbg_cores definition to do this
 *
 * @cra_name kernel crypto API cra_name
 * @coreref reference to integer which is filled with the pointer to
 *  the applicable core
 * @pr reference for setting prediction resistance
 *
 * return: flags
 */
static inline void drbg_convert_tfm_core(const char *cra_driver_name,
					 int *coreref, bool *pr)
{
	int i = 0;
	size_t start = 0;
	int len = 0;

	*pr = true;
	/* disassemble the names */
	if (!memcmp(cra_driver_name, "drbg_nopr_", 10)) {
		start = 10;
		*pr = false;
	} else if (!memcmp(cra_driver_name, "drbg_pr_", 8)) {
		start = 8;
	} else {
		return;
	}

	/* remove the first part */
	len = strlen(cra_driver_name) - start;
	for (i = 0; ARRAY_SIZE(drbg_cores) > i; i++) {
		if (!memcmp(cra_driver_name + start, drbg_cores[i].cra_name,
			    len)) {
			*coreref = i;
			return;
		}
	}
}

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

/*
 * Seed the DRBG invoked by the kernel crypto API
 */
static int drbg_kcapi_seed(struct crypto_rng *tfm,
			   const u8 *seed, unsigned int slen)
{
	struct drbg_state *drbg = crypto_rng_ctx(tfm);
	struct crypto_tfm *tfm_base = crypto_rng_tfm(tfm);
	bool pr = false;
	struct drbg_string string;
	struct drbg_string *seed_string = NULL;
	int coreref = 0;

	drbg_convert_tfm_core(crypto_tfm_alg_driver_name(tfm_base), &coreref,
			      &pr);
	if (0 < slen) {
		drbg_string_fill(&string, seed, slen);
		seed_string = &string;
	}

	return drbg_instantiate(drbg, seed_string, coreref, pr);
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
	bool pr = false;
	int coreref = 0;
	struct drbg_string addtl;
	size_t max_addtllen, max_request_bytes;

	/* only perform test in FIPS mode */
	if (!fips_enabled)
		return 0;

	drbg_convert_tfm_core("drbg_nopr_hmac_sha512", &coreref, &pr);

	drbg = kzalloc_obj(struct drbg_state);
	if (!drbg)
		return -ENOMEM;

	guard(mutex_init)(&drbg->drbg_mutex);
	drbg->core = &drbg_cores[coreref];
	drbg->reseed_threshold = drbg_max_requests(drbg);

	/*
	 * if the following tests fail, it is likely that there is a buffer
	 * overflow as buf is much smaller than the requested or provided
	 * string lengths -- in case the error handling does not succeed
	 * we may get an OOPS. And we want to get an OOPS as this is a
	 * grave bug.
	 */

	max_addtllen = drbg_max_addtl(drbg);
	max_request_bytes = drbg_max_request_bytes(drbg);
	drbg_string_fill(&addtl, buf, max_addtllen + 1);
	/* overflow addtllen with additional info string */
	ret = drbg_generate(drbg, buf, OUTBUFLEN, &addtl);
	BUG_ON(ret == 0);
	/* overflow max_bits */
	ret = drbg_generate(drbg, buf, max_request_bytes + 1, NULL);
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
		.seed			= drbg_kcapi_seed,
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
		.seed			= drbg_kcapi_seed,
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
