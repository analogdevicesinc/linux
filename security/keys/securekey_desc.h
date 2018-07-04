/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2018 NXP
 *
 */
#ifndef _SECUREKEY_DESC_H_
#define _SECUREKEY_DESC_H_

#include "compat.h"
#include "regs.h"
#include "intern.h"
#include "desc.h"
#include "desc_constr.h"
#include "jr.h"
#include "error.h"
#include "pdb.h"

#define SK_BLOB_KEY_SZ		32	/* Blob key size. */
#define SK_BLOB_MAC_SZ		16	/* Blob MAC size. */

/*
 * brief defines different kinds of operations supported by this module.
 */
enum sk_req_type {
	sk_get_random,
	sk_red_blob_enc,
	sk_red_blob_dec,
};


/*
 * struct random_des
 * param[out] rnd_data output buffer for random data.
 */
struct random_desc {
	dma_addr_t rnd_data;
};

/* struct redblob_encap_desc
 * details Structure containing dma address for redblob encapsulation.
 * param[in] in_data input data to redblob encap descriptor.
 * param[out] redblob output buffer for redblob.
 */
struct redblob_encap_desc {
	dma_addr_t in_data;
	dma_addr_t redblob;
};

/* struct redblob_decap_desc
 * details Structure containing dma address for redblob decapsulation.
 * param[in] redblob input buffer to redblob decap descriptor.
 * param[out] out_data output data from redblob decap descriptor.
 */
struct redblob_decap_desc {
	dma_addr_t redblob;
	dma_addr_t out_data;
};

/* struct sk_desc
 * details Structure for securekey descriptor creation.
 * param[in] req_type operation supported.
 * param[in] dma_u union of struct for supported operation.
 */
struct sk_desc {
	u32 req_type;
	union {
		struct redblob_encap_desc redblob_encapdesc;
		struct redblob_decap_desc redblob_decapdesc;
		struct random_desc random_descp;
	} dma_u;
};

/* struct sk_fetch_rnd_data
 * decriptor structure containing key length.
 */
struct sk_fetch_rnd_data {
	void *data;
	size_t key_len;
};

/* struct sk_red_blob_encap
 * details Structure containing buffer pointers for redblob encapsulation.
 * param[in] data Input data.
 * param[in] data_sz size of Input data.
 * param[out] redblob output buffer for redblob.
 * param[in] redblob_sz size of redblob.
 */
struct sk_red_blob_encap {
	void *data;
	uint32_t data_sz;
	void *redblob;
	uint32_t redblob_sz;
};

/* struct sk_red_blob_decap
 * details Structure containing buffer pointers for redblob decapsulation.
 * param[in] redblob Input redblob.
 * param[in] redblob_sz size of redblob.
 * param[out] data output buffer for data.
 * param[in] data_sz size of output data.
 */
struct sk_red_blob_decap {
	void *redblob;
	uint32_t redblob_sz;
	void *data;
	uint32_t data_sz;
};

/* struct sk_req
 * details Structure for securekey request creation.
 * param[in] type operation supported.
 * param[in] req_u union of struct for supported operation.
 * param[out] ret return status of CAAM operation.
 * param[in] mem_pointer memory pointer for allocated kernel memory.
 * param[in] desc_pointer Pointer to securekey descriptor creation structure.
 * param[in] comp struct completion object.
 * param[in] hwdesc contains descriptor instructions.
 */
struct sk_req {
	enum sk_req_type type;
	void *arg;
	union {
		struct sk_red_blob_encap sk_red_blob_encap;
		struct sk_red_blob_decap sk_red_blob_decap;
		struct sk_fetch_rnd_data sk_fetch_rnd_data;
	} req_u;
	int ret;
	void *mem_pointer;
	void *desc_pointer;
	struct completion comp;
	u32 hwdesc[MAX_CAAM_DESCSIZE];
};

int caam_get_random(struct secure_key_payload *p,  enum sk_req_type fetch_rnd,
		    struct device *dev);
int key_blob(struct secure_key_payload *p, enum sk_req_type encap_type,
	     struct device *dev);
int key_deblob(struct secure_key_payload *p, enum sk_req_type decap_type,
	       struct device *dev);

#endif /*_SECUREKEY_DESC_H_*/
