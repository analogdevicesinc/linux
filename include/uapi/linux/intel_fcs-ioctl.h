/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright (C) 2020, Intel Corporation
 */

#ifndef __INTEL_FCS_IOCTL_H
#define __INTEL_FCS_IOCTL_H

#include <linux/types.h>

/* the value may need be changed when upstream */
#define INTEL_FCS_IOCTL		0xC0

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
	__u32	test_bit:1;
	__u32	rsvd:31;
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
	__u32 size;
};

/**
 * struct fcs_key_manage_request - Request key management from SDM
 * @addr: the virtual address of the signed object,
 * @size: the size of the signed object
 */
struct fcs_key_manage_request {
	void *addr;
	__u32 size;
};

/**
 * struct fcs_certificate_request - Certificate request to SDM
 * @test: test bit (1 if want to write to cache instead of fuses)
 * @addr: the virtual address of the signed object,
 * @size: the size of the signed object
 * @c_status: returned certificate status
 */
struct fcs_certificate_request {
	struct intel_fcs_cert_test_word test;
	void *addr;
	__u32 size;
	__u32 c_status;
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
	__u32 src_size;
	void *dst;
	__u32 dst_size;
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
	__u32 src_size;
	void *dst;
	__u32 dst_size;
};

/**
 * struct fcs_random_number_gen
 * @rndm: 8 words of random data.
 */
struct fcs_random_number_gen {
	__u32 rndm[8];
};

/**
 * struct fcs_version
 * @version: version data.
 * @flags: Reserved as 0
 */
struct fcs_version {
	__u32 version;
	__u32 flags;
};

/**
 * struct intel_fcs_dev_ioct: common structure passed to Linux
 *	kernel driver for all commands.
 * @status: Used for the return code.
 *      -1 -- operation is not started
 *       0 -- operation is successfully completed
 *      non-zero -- operation failed
 * @s_request: Validation of a bitstream.
 * @c_request: Certificate request.
 *      hps_vab: validation of an HPS image
 *	counter set: burn fuses for new counter values
 * @gp_data: view the eFuse provisioning state.
 * @d_encryption: AES encryption (SDOS)
 * @d_decryption: AES decryption (SDOS)
 * @rn_gen: random number generator result
 */
struct intel_fcs_dev_ioctl {
	/* used for return status code */
	int status;

	/* command parameters */
	union {
		struct fcs_validation_request	s_request;
		struct fcs_certificate_request	c_request;
		struct fcs_key_manage_request	gp_data;
		struct fcs_data_encryption	d_encryption;
		struct fcs_data_decryption	d_decryption;
		struct fcs_random_number_gen	rn_gen;
		struct fcs_version		version;
	} com_paras;
};

/**
 * intel_fcs_command_code - support fpga crypto service commands
 *
 * Values are subject to change as a result of upstreaming.
 *
 * @INTEL_FCS_DEV_VERSION_CMD:
 *
 * @INTEL_FCS_DEV_CERTIFICATE_CMD:
 *
 * @INTEL_FCS_DEV_VALIDATE_REQUEST_CMD:
 *
 * @INTEL_FCS_DEV_COUNTER_SET_CMD:
 *
 * @INTEL_FCS_DEV_SVN_COMMIT_CMD:
 *
 * @INTEL_FCS_DEV_DATA_ENCRYPTION_CMD:
 *
 * @INTEL_FCS_DEV_DATA_DECRYPTION_CMD:
 *
 * @INTEL_FCS_DEV_RANDOM_NUMBER_GEN_CMD:
 */
enum intel_fcs_command_code {
	INTEL_FCS_DEV_COMMAND_NONE = 0,
	INTEL_FCS_DEV_VERSION_CMD = 1,
	INTEL_FCS_DEV_CERTIFICATE_CMD = 0xB,
	INTEL_FCS_DEV_VALIDATE_REQUEST_CMD = 0x78,
	INTEL_FCS_DEV_COUNTER_SET_CMD,
	INTEL_FCS_DEV_GET_PROVISION_DATA_CMD = 0x7B,
	INTEL_FCS_DEV_DATA_ENCRYPTION_CMD = 0x7E,
	INTEL_FCS_DEV_DATA_DECRYPTION_CMD,
	INTEL_FCS_DEV_RANDOM_NUMBER_GEN_CMD
};

#define INTEL_FCS_DEV_VERSION_REQUEST \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_VERSION_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_VALIDATION_REQUEST \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_VALIDATE_REQUEST_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_SEND_CERTIFICATE \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CERTIFICATE_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_GET_PROVISION_DATA \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_GET_PROVISION_DATA_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_DATA_ENCRYPTION \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_DATA_ENCRYPTION_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_DATA_DECRYPTION \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_DATA_DECRYPTION_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_RANDOM_NUMBER_GEN \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_RANDOM_NUMBER_GEN_CMD, struct intel_fcs_dev_ioctl)

#endif

