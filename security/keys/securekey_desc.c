// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 NXP
 *
 */

#include <keys/secure-type.h>
#include "securekey_desc.h"

/* key modifier for blob encapsulation & decapsulation descriptor */
u8 key_modifier[] = "SECURE_KEY";
u32 key_modifier_len = 10;

void caam_sk_rng_desc(struct sk_req *skreq, struct sk_desc *skdesc)
{
	struct sk_fetch_rnd_data *fetch_rnd_data = NULL;
	struct random_desc *rnd_desc = NULL;
	size_t len = 0;
	u32 *desc = skreq->hwdesc;

	init_job_desc(desc, 0);

	fetch_rnd_data = &skreq->req_u.sk_fetch_rnd_data;
	rnd_desc = &skdesc->dma_u.random_descp;
	len = fetch_rnd_data->key_len;

	/* command 0x82500000 */
	append_cmd(desc, CMD_OPERATION | OP_TYPE_CLASS1_ALG |
			OP_ALG_ALGSEL_RNG);
	/* command 0x60340000 | len */
	append_cmd(desc, CMD_FIFO_STORE | FIFOST_TYPE_RNGSTORE | len);
	append_ptr(desc, rnd_desc->rnd_data);
}

void caam_sk_redblob_encap_desc(struct sk_req *skreq, struct sk_desc *skdesc)
{
	struct redblob_encap_desc *red_blob_desc =
					&skdesc->dma_u.redblob_encapdesc;
	struct sk_red_blob_encap *red_blob_req =
					&skreq->req_u.sk_red_blob_encap;
	u32 *desc = skreq->hwdesc;

	init_job_desc(desc, 0);

	/* Load class 2 key with key modifier. */
	append_key_as_imm(desc, key_modifier, key_modifier_len,
			  key_modifier_len, CLASS_2 | KEY_DEST_CLASS_REG);

	/* SEQ IN PTR Command. */
	append_seq_in_ptr(desc, red_blob_desc->in_data, red_blob_req->data_sz,
			  0);

	/* SEQ OUT PTR Command. */
	append_seq_out_ptr(desc, red_blob_desc->redblob,
			   red_blob_req->redblob_sz, 0);

	/* RedBlob encapsulation PROTOCOL Command. */
	append_operation(desc, OP_TYPE_ENCAP_PROTOCOL | OP_PCLID_BLOB);
}

/* void caam_sk_redblob_decap_desc(struct sk_req *skreq, struct sk_desc *skdesc)
 * brief CAAM Descriptor creator from redblob to plaindata.
 * param[in] skreq Pointer to secure key request structure
 * param[in] skdesc Pointer to secure key descriptor structure
 */
void caam_sk_redblob_decap_desc(struct sk_req *skreq, struct sk_desc *skdesc)
{
	struct redblob_decap_desc *red_blob_desc =
					&skdesc->dma_u.redblob_decapdesc;
	struct sk_red_blob_decap *red_blob_req =
					&skreq->req_u.sk_red_blob_decap;
	u32 *desc = skreq->hwdesc;

	init_job_desc(desc, 0);

	/* Load class 2 key with key modifier. */
	append_key_as_imm(desc, key_modifier, key_modifier_len,
			  key_modifier_len, CLASS_2 | KEY_DEST_CLASS_REG);

	/* SEQ IN PTR Command. */
	append_seq_in_ptr(desc, red_blob_desc->redblob,
			  red_blob_req->redblob_sz, 0);

	/* SEQ OUT PTR Command. */
	append_seq_out_ptr(desc, red_blob_desc->out_data,
			   red_blob_req->data_sz, 0);

	/* RedBlob decapsulation PROTOCOL Command. */
	append_operation(desc, OP_TYPE_DECAP_PROTOCOL | OP_PCLID_BLOB);
}

/* int caam_sk_get_random_map(struct device *dev, struct sk_req *req,
 *			      struct sk_desc *skdesc)
 * brief DMA map the buffer virtual pointers to physical address.
 * param[in] dev Pointer to job ring device structure
 * param[in] req Pointer to secure key request structure
 * param[in] skdesc Pointer to secure key descriptor structure
 * return 0 on success, error value otherwise.
 */
int caam_sk_get_random_map(struct device *dev, struct sk_req *req,
			   struct sk_desc *skdesc)
{
	struct sk_fetch_rnd_data *fetch_rnd_data;
	struct random_desc *rnd_desc;

	fetch_rnd_data = &req->req_u.sk_fetch_rnd_data;
	rnd_desc = &skdesc->dma_u.random_descp;

	rnd_desc->rnd_data = dma_map_single(dev, fetch_rnd_data->data,
				fetch_rnd_data->key_len, DMA_FROM_DEVICE);

	if (dma_mapping_error(dev, rnd_desc->rnd_data)) {
		dev_err(dev, "Unable to map memory\n");
		goto sk_random_map_fail;
	}
	return 0;

sk_random_map_fail:
	return -ENOMEM;
}

/* int caam_sk_redblob_encap_map(struct device *dev, struct sk_req *req,
 *					struct sk_desc *skdesc)
 * brief DMA map the buffer virtual pointers to physical address.
 * param[in] dev Pointer to job ring device structure
 * param[in] req Pointer to secure key request structure
 * param[in] skdesc Pointer to secure key descriptor structure
 * return 0 on success, error value otherwise.
 */
int caam_sk_redblob_encap_map(struct device *dev, struct sk_req *req,
			      struct sk_desc *skdesc)
{
	struct sk_red_blob_encap *red_blob_encap;
	struct redblob_encap_desc *red_blob_desc;

	red_blob_encap = &req->req_u.sk_red_blob_encap;
	red_blob_desc = &skdesc->dma_u.redblob_encapdesc;

	red_blob_desc->in_data = dma_map_single(dev, red_blob_encap->data,
					red_blob_encap->data_sz, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, red_blob_desc->in_data)) {
		dev_err(dev, "Unable to map memory\n");
		goto sk_data_fail;
	}

	red_blob_desc->redblob = dma_map_single(dev, red_blob_encap->redblob,
				red_blob_encap->redblob_sz, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, red_blob_desc->redblob)) {
		dev_err(dev, "Unable to map memory\n");
		goto sk_redblob_fail;
	}

	return 0;

sk_redblob_fail:
	dma_unmap_single(dev, red_blob_desc->in_data, red_blob_encap->data_sz,
			 DMA_TO_DEVICE);
sk_data_fail:
	return -ENOMEM;
}

/* static int caam_sk_redblob_decap_map(struct device *dev,
 *					    struct sk_req *req,
 *					    struct sk_desc *skdesc)
 * brief DMA map the buffer virtual pointers to physical address.
 * param[in] dev Pointer to job ring device structure
 * param[in] req Pointer to secure key request structure
 * param[in] skdesc Pointer to secure key descriptor structure
 * return 0 on success, error value otherwise.
 */
int caam_sk_redblob_decap_map(struct device *dev, struct sk_req *req,
			      struct sk_desc *skdesc)
{
	struct sk_red_blob_decap *red_blob_decap;
	struct redblob_decap_desc *red_blob_desc;

	red_blob_decap = &req->req_u.sk_red_blob_decap;
	red_blob_desc = &skdesc->dma_u.redblob_decapdesc;

	red_blob_desc->redblob = dma_map_single(dev, red_blob_decap->redblob,
				red_blob_decap->redblob_sz, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, red_blob_desc->redblob)) {
		dev_err(dev, "Unable to map memory\n");
		goto sk_redblob_fail;
	}

	red_blob_desc->out_data = dma_map_single(dev, red_blob_decap->data,
				red_blob_decap->data_sz, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, red_blob_desc->out_data)) {
		dev_err(dev, "Unable to map memory\n");
		goto sk_data_fail;
	}

	return 0;

sk_data_fail:
	dma_unmap_single(dev, red_blob_desc->redblob,
			 red_blob_decap->redblob_sz, DMA_TO_DEVICE);
sk_redblob_fail:
	return -ENOMEM;
}

/* @fn void securekey_unmap(struct device *dev,
 *			    struct sk_desc *skdesc, struct sk_req *req)
 * @brief DMA unmap the buffer pointers.
 * @param[in] dev Pointer to job ring device structure
 * @param[in] skdesc Pointer to secure key descriptor structure
 * @param[in] req Pointer to secure key request structure
 */
void securekey_unmap(struct device *dev,
		     struct sk_desc *skdesc, struct sk_req *req)
{

	switch (req->type) {
	case sk_get_random:
		{
			struct sk_fetch_rnd_data *fetch_rnd_data;
			struct random_desc *rnd_desc;

			fetch_rnd_data = &req->req_u.sk_fetch_rnd_data;
			rnd_desc = &skdesc->dma_u.random_descp;

			/* Unmap Descriptor buffer pointers. */
			dma_unmap_single(dev, rnd_desc->rnd_data,
					 fetch_rnd_data->key_len,
					 DMA_FROM_DEVICE);
			break;
		}
	case sk_red_blob_enc:
		{
			struct sk_red_blob_encap *red_blob_encap;
			struct redblob_encap_desc *red_blob_desc;

			red_blob_encap = &req->req_u.sk_red_blob_encap;
			red_blob_desc = &skdesc->dma_u.redblob_encapdesc;

			/* Unmap Descriptor buffer pointers. */
			dma_unmap_single(dev, red_blob_desc->in_data,
					 red_blob_encap->data_sz,
					 DMA_TO_DEVICE);

			dma_unmap_single(dev, red_blob_desc->redblob,
					 red_blob_encap->redblob_sz,
					 DMA_FROM_DEVICE);

			break;
		}
	case sk_red_blob_dec:
		{
			struct sk_red_blob_decap *red_blob_decap;
			struct redblob_decap_desc *red_blob_desc;

			red_blob_decap = &req->req_u.sk_red_blob_decap;
			red_blob_desc = &skdesc->dma_u.redblob_decapdesc;

			/* Unmap Descriptor buffer pointers. */
			dma_unmap_single(dev, red_blob_desc->redblob,
					 red_blob_decap->redblob_sz,
					 DMA_TO_DEVICE);

			dma_unmap_single(dev, red_blob_desc->out_data,
					 red_blob_decap->data_sz,
					 DMA_FROM_DEVICE);

			break;
		}
	default:
		dev_err(dev, "Unable to find request type\n");
		break;
	}
	kfree(skdesc);
}

/*  int caam_securekey_desc_init(struct device *dev, struct sk_req *req)
 *  brief CAAM Descriptor creator for secure key operations.
 *  param[in] dev Pointer to job ring device structure
 *  param[in] req Pointer to secure key request structure
 *  return 0 on success, error value otherwise.
 */
int caam_securekey_desc_init(struct device *dev, struct sk_req *req)
{
	struct sk_desc *skdesc = NULL;
	int ret = 0;

	switch (req->type) {
	case sk_get_random:
		{
			skdesc = kmalloc(sizeof(*skdesc), GFP_DMA);
			if (!skdesc) {
				ret = -ENOMEM;
				goto out;
			}
			skdesc->req_type = req->type;

			if (caam_sk_get_random_map(dev, req, skdesc)) {
				dev_err(dev, "caam get_random map fail\n");
				ret = -ENOMEM;
				goto out;
			}
			caam_sk_rng_desc(req, skdesc);
			break;
		}
	case sk_red_blob_enc:
		{
			skdesc = kmalloc(sizeof(*skdesc), GFP_DMA);
			if (!skdesc) {
				ret = -ENOMEM;
				goto out;
			}

			skdesc->req_type = req->type;

			if (caam_sk_redblob_encap_map(dev, req, skdesc)) {
				dev_err(dev, "caam redblob_encap map fail\n");
				ret = -ENOMEM;
				goto out;
			}

			/* Descriptor function to create redblob from data. */
			caam_sk_redblob_encap_desc(req, skdesc);
			break;
		}

	case sk_red_blob_dec:
		{
			skdesc = kmalloc(sizeof(*skdesc), GFP_DMA);
			if (!skdesc) {
				ret = -ENOMEM;
				goto out;
			}

			skdesc->req_type = req->type;

			if (caam_sk_redblob_decap_map(dev, req, skdesc)) {
				dev_err(dev, "caam redblob_decap map fail\n");
				ret = -ENOMEM;
				goto out;
			}

			/* Descriptor function to decap data from redblob. */
			caam_sk_redblob_decap_desc(req, skdesc);
			break;
		}
	default:
		pr_debug("Unknown request type\n");
		ret = -EINVAL;
		goto out;
	}

	req->desc_pointer = (void *)skdesc;

out:
	return ret;
}

/* static void caam_op_done (struct device *dev, u32 *desc, u32 ret,
 *			     void *context)
 * brief callback function to be called when descriptor executed.
 * param[in] dev Pointer to device structure
 * param[in] desc descriptor pointer
 * param[in] ret return status of Job submitted
 * param[in] context void pointer
 */
static void caam_op_done(struct device *dev, u32 *desc, u32 ret,
			 void *context)
{
	struct sk_req *req = context;

	if (ret) {
		dev_err(dev, "caam op done err: %x\n", ret);
		/* print the error source name. */
		caam_jr_strstatus(dev, ret);
	}
	/* Call securekey_unmap function for unmapping the buffer pointers. */
	securekey_unmap(dev, req->desc_pointer, req);

	req->ret = ret;
	complete(&req->comp);
}


/*  static int sk_job_submit(struct device *jrdev, struct sk_req *req)
 *  brief Enqueue a Job descriptor to Job ring and wait until SEC returns.
 *  param[in] jrdev Pointer to job ring device structure
 *  param[in] req Pointer to secure key request structure
 *  return 0 on success, error value otherwise.
 */
static int sk_job_submit(struct device *jrdev, struct sk_req *req)
{
	int ret;

	init_completion(&req->comp);

	/* caam_jr_enqueue function for Enqueue a job descriptor */
	ret = caam_jr_enqueue(jrdev, req->hwdesc, caam_op_done, req);
	if (!ret)
		wait_for_completion_interruptible(&req->comp);

	ret = req->ret;
	return ret;
}

/* caam_get_random(struct secure_key_payload *p,  enum sk_req_type fetch_rnd,
 *		   struct device *dev)
 * Create the random number of the specified length using CAAM block
 * param[in]: out pointer to place the random bytes
 * param[in]: length for the random data bytes.
 * param[in]: dev Pointer to job ring device structure
 * If operation is successful return 0, otherwise error.
 */
int caam_get_random(struct secure_key_payload *p,  enum sk_req_type fetch_rnd,
		    struct device *dev)
{
	struct sk_fetch_rnd_data *fetch_rnd_data = NULL;
	struct sk_req *req = NULL;
	int ret = 0;
	void *temp = NULL;

	req = kmalloc(sizeof(struct sk_req), GFP_DMA);
	if (!req) {
		ret = -ENOMEM;
		goto out;
	}

	req->type = fetch_rnd;
	fetch_rnd_data = &(req->req_u.sk_fetch_rnd_data);

	/* initialise with key length */
	fetch_rnd_data->key_len = p->key_len;

	temp = kmalloc(fetch_rnd_data->key_len, GFP_DMA);
	if (!temp) {
		ret = -ENOMEM;
		goto out;
	}
	fetch_rnd_data->data = temp;

	ret = caam_securekey_desc_init(dev, req);

	if (ret) {
		pr_info("caam_securekey_desc_init failed\n");
		goto out;
	}

	ret = sk_job_submit(dev, req);
	if (!ret) {
		/*Copy output to key buffer. */
		memcpy(p->key, fetch_rnd_data->data, p->key_len);
	} else {
		ret = -EINVAL;
	}

out:
	if (req)
		kfree(req);

	if (temp)
		kfree(temp);

	return ret;
}
EXPORT_SYMBOL(caam_get_random);

/* key_deblob(struct secure_key_payload *p, enum sk_req_type decap_type
 *		struct device *dev)
 * Deblobify the blob to get the key data and fill in secure key payload struct
 * param[in] p pointer to the secure key payload
 * param[in] decap_type operation to be done.
 * param[in] dev dev Pointer to job ring device structure
 * If operation is successful return 0, otherwise error.
 */
int key_deblob(struct secure_key_payload *p, enum sk_req_type decap_type,
	       struct device *dev)
{
	unsigned int blob_len;
	struct sk_red_blob_decap *d_blob;
	struct sk_req *req = NULL;
	int total_sz = 0, *temp = NULL, ret = 0;

	req = kmalloc(sizeof(struct sk_req), GFP_DMA);
	if (!req) {
		ret = -ENOMEM;
		goto out;
	}

	d_blob = &(req->req_u.sk_red_blob_decap);
	blob_len = p->blob_len;
	req->type = decap_type;

	/*
	 * Red blob size is the blob_len filled in payload struct
	 * Data_sz i.e. key is the blob_len - blob header size
	 */

	d_blob->redblob_sz = blob_len;
	d_blob->data_sz = blob_len - (SK_BLOB_KEY_SZ + SK_BLOB_MAC_SZ);
	total_sz = d_blob->data_sz + d_blob->redblob_sz;

	temp = kmalloc(total_sz, GFP_DMA);
	if (!temp) {
		ret = -ENOMEM;
		goto out;
	}

	req->mem_pointer = temp;
	d_blob->redblob = temp;
	d_blob->data = d_blob->redblob + d_blob->redblob_sz;
	memcpy(d_blob->redblob, p->blob, blob_len);

	ret = caam_securekey_desc_init(dev, req);

	if (ret) {
		pr_info("caam_securekey_desc_init: Failed\n");
		goto out;
	}

	ret = sk_job_submit(dev, req);
	if (!ret) {
		/*Copy output to key buffer. */
		p->key_len = d_blob->data_sz;
		memcpy(p->key, d_blob->data, p->key_len);
	} else {
		ret = -EINVAL;
	}

out:
	if (temp)
		kfree(temp);
	if (req)
		kfree(req);
	return ret;
}
EXPORT_SYMBOL(key_deblob);

/* key_blob(struct secure_key_payload *p, enum sk_req_type encap_type,
 *		struct device *dev)
 * To blobify the key data to get the blob. This blob can only be seen by
 * userspace.
 * param[in] p pointer to the secure key payload
 * param[in] decap_type operation to be done.
 * param[in] dev dev Pointer to job ring device structure
 * If operation is successful return 0, otherwise error.
 */
int key_blob(struct secure_key_payload *p, enum sk_req_type encap_type,
	     struct device *dev)
{
	unsigned int key_len;
	struct sk_red_blob_encap *k_blob;
	struct sk_req *req = NULL;
	int total_sz = 0, *temp = NULL, ret = 0;

	req = kmalloc(sizeof(struct sk_req), GFP_DMA);
	if (!req) {
		ret = -ENOMEM;
		goto out;
	}

	key_len = p->key_len;

	req->type = encap_type;
	k_blob = &(req->req_u.sk_red_blob_encap);

	/*
	 * Data_sz i.e. key len and the corresponding blob_len is
	 * key_len + BLOB header size.
	 */

	k_blob->data_sz = key_len;
	k_blob->redblob_sz = key_len + SK_BLOB_KEY_SZ + SK_BLOB_MAC_SZ;
	total_sz = k_blob->data_sz + k_blob->redblob_sz;

	temp = kmalloc(total_sz, GFP_DMA);
	if (!temp) {
		ret = -ENOMEM;
		goto out;
	}

	req->mem_pointer = temp;
	k_blob->data = temp;

	k_blob->redblob = k_blob->data + k_blob->data_sz;
	memcpy(k_blob->data, p->key, key_len);

	ret = caam_securekey_desc_init(dev, req);

	if (ret) {
		pr_info("caam_securekey_desc_init failed\n");
		goto out;
	}

	ret = sk_job_submit(dev, req);
	if (!ret) {
		/*Copy output to key buffer. */
		p->blob_len = k_blob->redblob_sz;
		memcpy(p->blob, k_blob->redblob, p->blob_len);
	} else {
		ret = -EINVAL;
	}

out:
	if (temp)
		kfree(req->mem_pointer);
	if (req)
		kfree(req);
	return ret;

}
EXPORT_SYMBOL(key_blob);
