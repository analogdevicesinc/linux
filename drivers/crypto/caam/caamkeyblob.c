// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Black key generation and blob encapsulation/decapsulation for CAAM
 *
 * Copyright 2018-2020 NXP
 */
#include "caamkeyblob.h"
#include "error.h"

/* Black key generation and blob encap/decap job completion handler */
static void caam_key_blob_done(struct device *dev, u32 *desc, u32 err,
			       void *context)
{
	struct jr_job_result *res = context;
	int ecode = 0;

	dev_dbg(dev, "%s %d: err 0x%x\n", __func__, __LINE__, err);

	if (err)
		ecode = caam_jr_strstatus(dev, err);

	/* Save the error for post-processing */
	res->error = ecode;
	/* Mark job as complete */
	complete(&res->completion);
}

/**
 * map_write_data   - Prepare data to be written to CAAM
 *
 * @dev             : struct device of the job ring to be used
 * @data            : The data to be prepared
 * @size            : The size of data to be prepared
 * @dma_addr        : The retrieve DMA address of the input data
 * @allocated_data  : Pointer to a DMA-able address where the input
 *                    data is copied and synchronized
 *
 * Return           : '0' on success, error code otherwise
 */
static int map_write_data(struct device *dev, const u8 *data, size_t size,
			  dma_addr_t *dma_addr, u8 **allocated_data)
{
	int ret = 0;

	/* Allocate memory for data and copy it to DMA zone */
	*allocated_data = kmemdup(data, size, GFP_KERNEL | GFP_DMA);
	if (!*allocated_data) {
		ret = -ENOMEM;
		goto exit;
	}

	*dma_addr = dma_map_single(dev, *allocated_data, size, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *dma_addr)) {
		dev_err(dev, "Unable to map write data\n");
		ret = -ENOMEM;
		goto free_alloc;
	}

	goto exit;

free_alloc:
	kfree(*allocated_data);

exit:
	return ret;
}

/**
 * map_read_data   - Prepare data to be read from CAAM
 *
 * @dev             : struct device of the job ring to be used
 * @size            : The size of data to be prepared
 * @dma_addr        : The retrieve DMA address of the data to be read
 * @allocated_data  : Pointer to a DMA-able address where the data
 *                    to be read will be copied and synchronized
 *
 * Return           : '0' on success, error code otherwise
 */
static int map_read_data(struct device *dev, size_t size, dma_addr_t *dma_addr,
			 u8 **allocated_data)
{
	int ret = 0;

	/* Allocate memory for data compatible with DMA */
	*allocated_data = kmalloc(size, GFP_KERNEL | GFP_DMA);
	if (!*allocated_data) {
		ret = -ENOMEM;
		goto exit;
	}

	*dma_addr = dma_map_single(dev, *allocated_data, size, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, *dma_addr)) {
		dev_err(dev, "Unable to map read data\n");
		ret = -ENOMEM;
		goto free_alloc;
	}

	goto exit;

free_alloc:
	kfree(*allocated_data);

exit:
	return ret;
}

/**
 * read_map_data   - Read the data from CAAM
 *
 * @dev             : struct device of the job ring to be used
 * @data            : The read data from CAAM will be copied here
 * @dma_addr        : The DMA address of the data to be read
 * @allocated_data  : Pointer to a DMA-able address where the data
 *                    to be read is
 * @size            : The size of data to be read
 */
static void read_map_data(struct device *dev, u8 *data, dma_addr_t dma_addr,
			  u8 *allocated_data, size_t size)
{
	/* Synchronize the DMA and copy the data */
	dma_sync_single_for_cpu(dev, dma_addr, size, DMA_FROM_DEVICE);
	memcpy(data, allocated_data, size);
}

/**
 * unmap_read_write_data - Unmap the data needed for or from CAAM
 *
 * @dev             : struct device of the job ring to be used
 * @dma_addr        : The DMA address of the data used for DMA transfer
 * @allocated_data  : The data used for DMA transfer
 * @size            : The size of data
 * @dir             : The DMA_API direction
 */
static void unmap_read_write_data(struct device *dev, dma_addr_t dma_addr,
				  u8 *allocated_data, size_t size,
				  enum dma_data_direction dir)
{
	/* Free the resources and clear the data*/
	dma_unmap_single(dev, dma_addr, size, dir);
	kfree_sensitive(allocated_data);
}

/**
 * get_caam_dma_addr - Get the CAAM DMA address of a physical address.
 *
 * @phy_address     : The physical address
 *
 * Return           : The CAAM DMA address
 */
static dma_addr_t get_caam_dma_addr(const void *phy_address)
{
	uintptr_t ptr_conv;
	dma_addr_t caam_dma_address = 0;

	/* Check if conversion is possible */
	if (sizeof(caam_dma_address) < sizeof(phy_address)) {
		/*
		 * Check that all bits sets in the phy_address
		 * can be stored in caam_dma_address
		 */

		/* Generate a mask of the representable bits */
		u64 mask = GENMASK_ULL(sizeof(caam_dma_address) * 8 - 1, 0);

		/*
		 * Check that the bits not representable of
		 * the physical address are not set
		 */
		if ((uintptr_t)phy_address & ~mask)
			goto exit;
	}

	/* Convert address to caam_dma_address */
	ptr_conv = (uintptr_t)phy_address;
	caam_dma_address = (dma_addr_t)ptr_conv;

exit:
	return caam_dma_address;
}

/**
 * generate_black_key - Generate a black key from a plaintext or random,
 *                      based on the given input: a size for a random black
 *                      key, or a plaintext (input key).
 *
 * If the memory type is Secure Memory, the key to cover is read
 * directly by CAAM from Secure Memory without intermediate copy.
 * The value of the input key (plaintext) must be a physical address
 * in Secure Memory.
 *
 * Notes:
 * Limited to Class 1 keys, at the present time.
 * The input and output data are copied to temporary arrays
 * except for the input key if the memory type is Secure Memory.
 * For now, we have support for Black keys, stored in General Memory.
 *
 * @dev             : struct device of the job ring to be used
 * @info            : keyblob_info structure, will be updated with
 *                    the black key data from CAAM.
 *                    This contains, also, all the data necessary to generate
 *                    a black key from plaintext/random like: key encryption
 *                    key, memory type, input key, etc.
 *
 * Return           : '0' on success, error code otherwise
 */
int generate_black_key(struct device *dev, struct keyblob_info *info)
{
	int ret = 0;
	bool not_random = false;
	u8 trusted_key, key_enc;
	u32 *desc = NULL;
	size_t black_key_length_req = 0;
	dma_addr_t black_key_dma;
	u8 *tmp_black_key = NULL;

	/* Validate device */
	if (!dev)
		return -EINVAL;

	/*
	 * If an input key (plaintext) is given,
	 * generate a black key from it, not from random
	 */
	if (info->key)
		not_random = true;

	/* Get trusted key and key encryption type from type */
	trusted_key = (info->type >> TAG_OBJ_TK_OFFSET) & 0x1;
	key_enc = (info->type >> TAG_OBJ_EKT_OFFSET) & 0x1;

	dev_dbg(dev, "%s input: [key: (%zu) black_key: %p(%zu), key_enc: %x]\n",
		__func__, info->key_len, info->black_key, info->black_key_len,
		key_enc);
	if (not_random)
		print_hex_dump_debug("input key @" __stringify(__LINE__) ": ",
				     DUMP_PREFIX_ADDRESS, 16, 4, info->key,
				     info->key_len, 1);

	/* Validate key type - only JDKEK keys are supported */
	if (!is_key_type(info->type) || is_trusted_type(info->type))
		return -EINVAL;

	/*
	 * Validate key size, expected values are
	 * between 16 and 64 bytes.
	 * See TODO from cnstr_desc_black_key().
	 */
	if (info->key_len < MIN_KEY_SIZE || info->key_len > MAX_KEY_SIZE)
		return -EINVAL;

	/*
	 * Based on key encryption type (ecb or ccm),
	 * compute the black key size
	 */
	if (key_enc == KEY_COVER_ECB)
		/*
		 * ECB-Black Key will be padded with zeros to make it a
		 * multiple of 16 bytes long before it is encrypted,
		 * and the resulting Black Key will be this length.
		 */
		black_key_length_req = ECB_BLACK_KEY_SIZE(info->key_len);
	else if (key_enc == KEY_COVER_CCM)
		/*
		 * CCM-Black Key will always be at least 12 bytes longer,
		 * since the encapsulation uses a 6-byte nonce and adds
		 * a 6-byte ICV. But first, the key is padded as necessary so
		 * that CCM-Black Key is a multiple of 8 bytes long.
		 */
		black_key_length_req = CCM_BLACK_KEY_SIZE(info->key_len);

	/* Check if there is enough space for black key */
	if (info->black_key_len < black_key_length_req) {
		info->black_key_len = black_key_length_req;
		return -EINVAL;
	}

	/* Black key will have at least the same length as the input key */
	info->black_key_len = info->key_len;

	dev_dbg(dev, "%s processing: [key: (%zu) black_key: %p(%zu)",
		__func__, info->key_len, info->black_key, info->black_key_len);
	dev_dbg(dev, "req:%zu, key_enc: 0x%x]\n", black_key_length_req, key_enc);

	/* Map black key, this will be read from CAAM */
	if (map_read_data(dev, black_key_length_req,
			  &black_key_dma, &tmp_black_key)) {
		dev_err(dev, "Unable to map black key\n");
		ret = -ENOMEM;
		goto exit;
	}

	/* Construct descriptor for black key */
	if (not_random)
		ret = cnstr_desc_black_key(&desc, info->key, info->key_len,
					   black_key_dma, info->black_key_len,
					   key_enc, trusted_key);
	else
		ret = cnstr_desc_random_black_key(&desc, info->key_len,
						  black_key_dma,
						  info->black_key_len,
						  key_enc, trusted_key);

	if (ret) {
		dev_err(dev,
			"Failed to construct the descriptor for black key\n");
		goto unmap_black_key;
	}

	/* Execute descriptor and wait for its completion */
	ret = caam_jr_run_and_wait_for_completion(dev, desc,
						  caam_key_blob_done);
	if (ret) {
		dev_err(dev, "Failed to execute black key descriptor\n");
		goto free_desc;
	}

	/* Read black key from CAAM */
	read_map_data(dev, info->black_key, black_key_dma,
		      tmp_black_key, black_key_length_req);

	/* Update black key length with the correct size */
	info->black_key_len = black_key_length_req;

free_desc:
	kfree(desc);

unmap_black_key:
	unmap_read_write_data(dev, black_key_dma, tmp_black_key,
			      black_key_length_req, DMA_FROM_DEVICE);

exit:
	return ret;
}
EXPORT_SYMBOL(generate_black_key);

/**
 * caam_blob_encap - Encapsulate a black key into a blob
 *
 * If the memory type is Secure Memory, the key to encapsulate is read
 * directly by CAAM from Secure Memory without intermediate copy.
 * The value of the key (black key) must be a physical address
 * in Secure Memory.
 *
 * Notes:
 * For now, we have support for Black keys, stored in General Memory and
 * encapsulated into black blobs.
 *
 * @dev             : struct device of the job ring to be used
 * @info            : keyblob_info structure, will be updated with
 *                    the blob data from CAAM.
 *                    This contains, also, all the data necessary to
 *                    encapsulate a black key into a blob: key encryption
 *                    key, memory type, color, etc.
 *
 * Return           : '0' on success, error code otherwise
 */
int caam_blob_encap(struct device *dev, struct keyblob_info *info)
{
	int ret = 0;
	u32 *desc = NULL;
	size_t black_key_real_len = 0;
	size_t blob_req_len = 0;
	u8 mem_type, color, key_enc, trusted_key;
	dma_addr_t black_key_dma, blob_dma;
	unsigned char *blob = info->blob;
	u8 *tmp_black_key = NULL, *tmp_blob = NULL;

	/* Validate device */
	if (!dev)
		return -EINVAL;

	/*
	 * Get memory type, trusted key, key encryption
	 * type and color from type
	 */
	mem_type = (info->type >> TAG_OBJ_MEM_OFFSET) & 0x1;
	color = (info->type >> TAG_OBJ_COLOR_OFFSET) & 0x1;
	key_enc = (info->type >> TAG_OBJ_EKT_OFFSET) & 0x1;
	trusted_key = (info->type >> TAG_OBJ_TK_OFFSET) & 0x1;

	/* Validate input data*/
	if (!info->key_mod || !blob)
		return -EINVAL;

	/* Validate object type - only JDKEK keys are supported */
	if (is_trusted_type(info->type))
		return -EINVAL;

	dev_dbg(dev, "%s input:[black_key: %p (%zu) color: %x, key_enc: %x",
		__func__, info->black_key, info->black_key_len, color, key_enc);
	dev_dbg(dev, ", key_mod: %p (%zu)", info->key_mod, info->key_mod_len);
	dev_dbg(dev, "blob: %p (%zu)]\n", blob, info->blob_len);

	/*
	 * Based on memory type, the key modifier length
	 * can be 8-byte or 16-byte.
	 */
	if (mem_type == DATA_SECMEM)
		info->key_mod_len = KEYMOD_SIZE_SM;
	else
		info->key_mod_len = KEYMOD_SIZE_GM;

	/* Adapt the size of the black key */
	black_key_real_len = info->black_key_len;

	blob_req_len = CCM_BLACK_KEY_SIZE(info->key_len);

	/* Check if the blob can be stored */
	if (info->blob_len < (blob_req_len + BLOB_OVERHEAD))
		return -EINVAL;

	/* Update the blob length */
	info->blob_len = blob_req_len + BLOB_OVERHEAD;

	dev_dbg(dev, "%s processing: [black_key: %p (%zu) cnstr: %zu",
		__func__, info->black_key, info->black_key_len,
		black_key_real_len);
	dev_dbg(dev, " color: %x key_enc: %x, mem_type: %x,",
		color, key_enc, mem_type);
	dev_dbg(dev, ", key_mod: %p (%zu) ", info->key_mod, info->key_mod_len);
	dev_dbg(dev, "blob: %p (%zu)]\n", blob, info->blob_len);

	/* Map black key, this will be transferred to CAAM */
	if (mem_type == DATA_GENMEM) {
		if (map_write_data(dev, info->black_key, info->black_key_len,
				   &black_key_dma, &tmp_black_key)) {
			dev_err(dev, "Unable to map black key for blob\n");
			ret = -ENOMEM;
			goto exit;
		}
	} else {
		black_key_dma = get_caam_dma_addr(info->black_key);
		if (!black_key_dma)
			return -ENOMEM;
	}

	/* Map blob, this will be read to CAAM */
	if (mem_type == DATA_GENMEM) {
		if (map_read_data(dev, info->blob_len, &blob_dma, &tmp_blob)) {
			dev_err(dev, "Unable to map blob\n");
			ret = -ENOMEM;
			goto unmap_black_key;
		}
	} else {
		blob_dma = get_caam_dma_addr(info->blob);
		if (!blob_dma)
			return -ENOMEM;
	}

	/* Construct descriptor for blob encapsulation */
	ret = cnstr_desc_blob_encap(&desc, black_key_dma, info->key_len,
				    color, key_enc, trusted_key, mem_type,
				    info->key_mod, info->key_mod_len,
				    blob_dma, info->blob_len);
	if (ret) {
		dev_err(dev,
			"Failed to construct the descriptor for blob encap\n");
		goto unmap_blob;
	}

	/* Execute descriptor and wait for its completion */
	ret = caam_jr_run_and_wait_for_completion(dev, desc,
						  caam_key_blob_done);
	if (ret) {
		dev_err(dev, "Failed to execute blob encap descriptor\n");
		goto free_desc;
	}

	/* Read blob from CAAM */
	if (mem_type == DATA_GENMEM)
		read_map_data(dev, blob, blob_dma, tmp_blob, info->blob_len);

	print_hex_dump_debug("blob @" __stringify(__LINE__) ": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, blob,
			     info->blob_len, 1);
free_desc:
	kfree(desc);

unmap_blob:
	if (mem_type == DATA_GENMEM)
		unmap_read_write_data(dev, blob_dma, tmp_blob,
				      info->blob_len, DMA_FROM_DEVICE);

unmap_black_key:
	if (mem_type == DATA_GENMEM)
		unmap_read_write_data(dev, black_key_dma, tmp_black_key,
				      info->black_key_len, DMA_TO_DEVICE);

exit:
	return ret;
}
EXPORT_SYMBOL(caam_blob_encap);

/**
 * caam_blob_decap - Decapsulate a black key from a blob
 *
 * Notes:
 * For now, we have support for Black blob, stored in General Memory and
 * can be decapsulated into a black key.
 *
 * @dev             : struct device of the job ring to be used
 * @info            : keyblob_info structure, will be updated with
 *                    the black key decapsulated from the blob.
 *                    This contains, also, all the data necessary to
 *                    encapsulate a black key into a blob: key encryption
 *                    key, memory type, color, etc.
 *
 * Return           : '0' on success, error code otherwise
 */
int caam_blob_decap(struct device *dev, struct keyblob_info *info)
{
	int ret = 0;
	u32 *desc = NULL;
	u8 mem_type, color, key_enc, trusted_key;
	size_t black_key_real_len;
	dma_addr_t black_key_dma, blob_dma;
	unsigned char *blob = info->blob + TAG_OVERHEAD_SIZE;
	u8 *tmp_black_key = NULL, *tmp_blob = NULL;

	/* Validate device */
	if (!dev)
		return -EINVAL;

	/*
	 * Get memory type, trusted key, key encryption
	 * type and color from type
	 */
	mem_type = (info->type >> TAG_OBJ_MEM_OFFSET) & 0x1;
	color = (info->type >> TAG_OBJ_COLOR_OFFSET) & 0x1;
	key_enc = (info->type >> TAG_OBJ_EKT_OFFSET) & 0x1;
	trusted_key = (info->type >> TAG_OBJ_TK_OFFSET) & 0x1;

	/* Validate input data*/
	if (!info->key_mod || !blob)
		return -EINVAL;

	dev_dbg(dev, "%s input: [blob: %p (%zu), mem_type: %x, color: %x",
		__func__, blob, info->blob_len, mem_type, color);
	dev_dbg(dev, " keymod: %p (%zu)", info->key_mod, info->key_mod_len);
	dev_dbg(dev, " secret: %p (%zu) key_enc: %x]\n",
		info->black_key, info->black_key_len, key_enc);

	/* Validate object type - only JDKEK keys are supported */
	if (is_trusted_type(info->type))
		return -EINVAL;

	print_hex_dump_debug("blob @" __stringify(__LINE__) ": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, blob,
			     info->blob_len, 1);

	/*
	 * Based on memory type, the key modifier length
	 * can be 8-byte or 16-byte.
	 */
	if (mem_type == DATA_SECMEM)
		info->key_mod_len = KEYMOD_SIZE_SM;
	else
		info->key_mod_len = KEYMOD_SIZE_GM;

	/* Check if the blob is valid */
	if (info->blob_len <= BLOB_OVERHEAD)
		return -EINVAL;

	/* Initialize black key length */
	black_key_real_len = info->blob_len - BLOB_OVERHEAD;

	/* Check if the black key has enough space to be stored */
	if (info->black_key_len < black_key_real_len)
		return -EINVAL;

	/*
	 * Based on key encryption type (ecb or ccm),
	 * compute the black key size
	 */
	if (key_enc == KEY_COVER_ECB)
		/*
		 * ECB-Black Key will be padded with zeros to make it a
		 * multiple of 16 bytes long before it is encrypted,
		 * and the resulting Black Key will be this length.
		 */
		black_key_real_len = ECB_BLACK_KEY_SIZE(info->key_len);
	else if (key_enc == KEY_COVER_CCM)
		/*
		 * CCM-Black Key will always be at least 12 bytes longer,
		 * since the encapsulation uses a 6-byte nonce and adds
		 * a 6-byte ICV. But first, the key is padded as necessary so
		 * that CCM-Black Key is a multiple of 8 bytes long.
		 */
		black_key_real_len = CCM_BLACK_KEY_SIZE(info->key_len);

	/* Check if there is enough space for black key */
	if (info->black_key_len < black_key_real_len)
		return -EINVAL;

	/* Update black key length with the one computed based on key_enc */
	info->black_key_len = black_key_real_len;

	dev_dbg(dev, "%s processing: [blob: %p (%zu), mem_type: %x, color: %x,",
		__func__, blob, info->blob_len, mem_type, color);
	dev_dbg(dev, " key_mod: %p (%zu), black_key: %p (%zu) real_len: %zu]\n",
		info->key_mod, info->key_mod_len, info->black_key,
		info->black_key_len, black_key_real_len);

	/* Map blob, this will be transferred to CAAM */
	if (mem_type == DATA_GENMEM) {
		if (map_write_data(dev, blob, info->blob_len,
				   &blob_dma, &tmp_blob)) {
			dev_err(dev, "Unable to map blob for decap\n");
			ret = -ENOMEM;
			goto exit;
		}
	} else {
		blob_dma = get_caam_dma_addr(blob);
		if (!blob_dma)
			return -ENOMEM;
	}

	/* Map black key, this will be read from CAAM */
	if (mem_type == DATA_GENMEM) {
		if (map_read_data(dev, info->black_key_len,
				  &black_key_dma, &tmp_black_key)) {
			dev_err(dev, "Unable to map black key for blob decap\n");
			ret = -ENOMEM;
			goto unmap_blob;
		}
	} else {
		black_key_dma = get_caam_dma_addr(info->black_key);
		if (!black_key_dma)
			return -ENOMEM;
	}

	ret = cnstr_desc_blob_decap(&desc, blob_dma, info->blob_len,
				    info->key_mod, info->key_mod_len,
				    black_key_dma, info->key_len,
				    color, key_enc, trusted_key, mem_type);
	if (ret) {
		dev_err(dev,
			"Failed to construct the descriptor for blob decap\n");
		goto unmap_black_key;
	}

	ret = caam_jr_run_and_wait_for_completion(dev, desc,
						  caam_key_blob_done);
	if (ret) {
		dev_err(dev, "Failed to execute blob decap descriptor\n");
		goto free_desc;
	}

	/* Read black key from CAAM */
	if (mem_type == DATA_GENMEM)
		read_map_data(dev, info->black_key, black_key_dma,
			      tmp_black_key, info->black_key_len);

free_desc:
	kfree(desc);

unmap_black_key:
	if (mem_type == DATA_GENMEM)
		unmap_read_write_data(dev, black_key_dma, tmp_black_key,
				      info->black_key_len, DMA_FROM_DEVICE);

unmap_blob:
	if (mem_type == DATA_GENMEM)
		unmap_read_write_data(dev, blob_dma, tmp_blob,
				      info->blob_len, DMA_TO_DEVICE);

exit:
	return ret;
}
EXPORT_SYMBOL(caam_blob_decap);
