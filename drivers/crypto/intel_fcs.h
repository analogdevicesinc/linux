/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020, Intel Corporation
 */

#ifndef __INTEL_FCS_H
#define __INTEL_FCS_H

/**
 * enum fcs_vab_img_type - enumeration of image types
 * @INTEL_FCS_IMAGE_HPS: Image to validate is HPS image
 * @INTEL_FCS_IMAGE_BITSTREAM: Image to validate is bitstream
 */
enum fcs_vab_img_type {
	INTEL_FCS_IMAGE_HPS = 0,
	INTEL_FCS_IMAGE_BITSTREAM = 1
};

/**
 * enum fcs_certificate_test - enumeration of certificate test
 * @INTEL_FCS_NO_TEST: Write to eFuses
 * @INTEL_FCS_TEST: Write to cache, do not write eFuses
 */
enum fcs_certificate_test {
	INTEL_FCS_NO_TEST = 0,
	INTEL_FCS_TEST = 1
};

/**
 * struct intel_fcs_cert_test_word - certificate test word
 * @test_bit: if set, do not write fuses, write to cache only.
 * @rsvd: write as 0
 */
struct intel_fcs_cert_test_word {
	uint32_t	test_bit:1;
	uint32_t	rsvd:31;
};

/**
 * struct fcs_validation_request - validate HPS or bitstream image
 * @so_type: the type of signed object, 0 for HPS and 1 for bitstream
 * @src: the source of signed object,
 *       for HPS, this is the virtual address of the signed source
 *	 for Bitstream, this is path of the signed source, the default
 *       path is /lib/firmware
 * @size: the size of the signed object
 */
struct fcs_validation_request {
	enum fcs_vab_img_type so_type;
	void *src;
	size_t size;
};

/**
 * struct fcs_key_manage_request - Request key management from SDM
 * @addr: the virtual address of the signed object,
 * @size: the size of the signed object
 */
struct fcs_key_manage_request {
	void *addr;
	size_t size;
};

/**
 * struct fcs_certificate_request - Certificate request to SDM
 * @test: test bit (1 if want to write to cache instead of fuses)
 * @addr: the virtual address of the signed object,
 * @size: the size of the signed object
 */
struct fcs_certificate_request {
	struct intel_fcs_cert_test_word test;
	void *addr;
	size_t size;
};

/**
 * struct fcs_data_encryption - aes data encryption command layout
 * @src: the virtual address of the input data
 * @src_size: the size of the unencrypted source
 * @dst: the virtual address of the output data
 * @dst_size: the size of the encrypted result
 */
struct fcs_data_encryption {
	void *src;
	size_t src_size;
	void *dst;
	size_t dst_size;
};

/**
 * struct fcs_data_decryption - aes data decryption command layout
 * @src: the virtual address of the input data
 * @src_size: the size of the encrypted source
 * @dst: the virtual address of the output data
 * @dst_size: the size of the decrypted result
 */
struct fcs_data_decryption {
	void *src;
	size_t src_size;
	void *dst;
	size_t dst_size;
};

/**
 * struct fcs_random_number_gen
 * @rndm: 8 words of random data.
 */
struct fcs_random_number_gen {
	uint32_t rndm[8];
};

/**
 * struct fcs_version
 * @version: version data.
 * @flags: Reserved as 0
 */
struct fcs_version {
	uint32_t version;
	uint32_t flags;
};
#endif

