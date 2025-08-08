/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright (C) 2020, Intel Corporation
 */

#ifndef __INTEL_FCS_IOCTL_H
#define __INTEL_FCS_IOCTL_H

#include <linux/types.h>

/* the value may need be changed when upstream */
#define INTEL_FCS_IOCTL		0xC0

/* define macro to be used to fix the size of struct intel_fcs_dev_ioctl */
#define INTEL_FCS_IOCTL_MAX_SZ		256U
/* the header include the 8 bytes stucture padding and 4 bytes status */
#define INTEL_FCS_IOCTL_HEADER_SZ	12U
#define INTEL_FCS_IOCTL_PLACEHOLDER_SZ	(INTEL_FCS_IOCTL_MAX_SZ - \
					 INTEL_FCS_IOCTL_HEADER_SZ) / 4

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
 * struct fcs_mbox_send_cmd - send generic mailbox command
 * @mbox_cmd: mailbox command code
 * @urgent: 0 for CASUAL, 1 for URGENT
 * @cmd_data: virtual address of mailbox command data
 * @cmd_data_sz: size of mailbox command data in bytes
 * @rsp_data: virtual address to store response data
 * @rsp_data_sz: maximun size to store response data in bytes
 */
struct fcs_mbox_send_cmd {
	uint32_t mbox_cmd;
	uint8_t urgent;
	void *cmd_data;
	uint16_t cmd_data_sz;
	void *rsp_data;
	uint16_t rsp_data_sz;
};

/**
 * struct fcs_placeholder - placeholder of ioctl stuct
 * @data: placeholder of iotcl struct
 */
struct fcs_placeholder {
	uint32_t data[INTEL_FCS_IOCTL_PLACEHOLDER_SZ];
};

/**
 * struct intel_fcs_cert_test_word - certificate test word
 * @test_word: if set, do not write fuses, write to cache only.
 */
struct intel_fcs_cert_test_word {
	uint32_t test_word;
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
	uint32_t size;
};

/**
 * struct fcs_key_manage_request - Request key management from SDM
 * @addr: the virtual address of the signed object,
 * @size: the size of the signed object
 */
struct fcs_key_manage_request {
	void *addr;
	uint32_t size;
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
	uint32_t size;
	uint32_t c_status;
};

/**
 * struct fcs_single_certificate_request - Single certificate to SDM
 * @test: test bit (1 if want to write to cache instead of fuses)
 * @counter_type: select the counter type with valid value from 1 to 5
 * @counter_value: counter value
 */
struct fcs_single_certificate_request {
	struct intel_fcs_cert_test_word test;
	uint8_t counter_type;
	uint32_t counter_value;
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
	uint32_t src_size;
	void *dst;
	uint32_t dst_size;
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
	uint32_t src_size;
	void *dst;
	uint32_t dst_size;
};

/**
 * struct fcs_random_number_gen
 * @rndm: 8 words of random data.
 */
struct fcs_random_number_gen {
	uint32_t rndm[8];
};

/**
 * struct fcs_psgsigma_teardown
 * @teardown
 * @sid: the session ID
 */
struct fcs_psgsigma_teardown {
	bool teardown;
	uint32_t sid;
};

/**
 * struct fcs_attestation_chipid
 * @chip_id_low: device chip ID lower 32
 * @chip_id_high: device chip ID high 32
 */
struct fcs_attestation_chipid {
	uint32_t chip_id_low;
	uint32_t chip_id_high;
};

/**
 * struct intel_fcs_attestation_resv_word - attestation reserve word
 * @resv_word: a reserve word required by firmware
 */
struct intel_fcs_attestation_resv_word {
	uint32_t resv_word;
};

/**
 * struct fcs_attestation_subkey
 * @resv: reserve word
 * @cmd_data: command data
 * @cmd_data_sz: command data size
 * @rsp_data: response data
 * @rsp_data_sz: response data size
 */
struct fcs_attestation_subkey {
	struct intel_fcs_attestation_resv_word resv;
	char *cmd_data;
	uint32_t cmd_data_sz;
	char *rsp_data;
	uint32_t rsp_data_sz;
};

/**
 * struct fcs_attestation_measuerments
 * @resv: reserve word
 * @cmd_data: command data
 * @cmd_data_sz: command data size
 * @rsp_data: response data
 * @rsp_data_sz: response data size
 */
struct fcs_attestation_measuerments {
	struct intel_fcs_attestation_resv_word resv;
	char *cmd_data;
	uint32_t cmd_data_sz;
	char *rsp_data;
	uint32_t rsp_data_sz;
};

/**
 * struct fcs_attestation_certificate
 * @c_request: certificate request
 * @rsp_data: response data of the request certificate
 * @rsp_data_sz: size of response data of the request certificate
 */
struct fcs_attestation_certificate {
	int c_request;
	char *rsp_data;
	uint32_t rsp_data_sz;
};

/**
 * fcs_attestation_certificate_reload
 * @c_request: certificate request
 */
struct fcs_attestation_certificate_reload {
	int c_request;
};

/**
 * struct fcs_rom_patch_sha384
 * @checksum: 12 words of checksum calculated from rom patch area
 */
struct fcs_rom_patch_sha384 {
	uint32_t checksum[12];
};

/**
 * struct fcs_crypto_service_session
 * @sid: the crypto service session ID
 */
struct fcs_crypto_service_session {
	uint32_t sid;
};

struct fcs_crypto_key_header {
	uint32_t sid;
	uint32_t res1;
	uint32_t res2;
};

struct fcs_crypto_key_import {
	struct fcs_crypto_key_header hd;
	char *obj_data;
	uint32_t obj_data_sz;
};

struct fcs_crypto_key_object {
	uint32_t sid;
	uint32_t kid;
	char *obj_data;
	uint32_t obj_data_sz;
};

/**
 * struct fcs_acs_crypt_parameter
 * @bmode: block mode
 * @aes_mode: encrypt or decrypt
 * 	0	encrypt
 * 	1	decrypt
 * @resv: reserved
 * @iv: 128-bit IV field
 */
struct fcs_acs_crypt_parameter {
	char bmode;
	char aes_mode;
	char resv[10];
	char iv_field[16];
};

/**
 * struct fcs_aes_crypt
 * @sid: session ID
 * @cid: context ID
 * @kuid: key UID
 * @src: source
 * @src_size: size of source
 * @dst: destination
 * @dst_size: size of destination
 * @cpara: crypto parameter
 */
struct fcs_aes_crypt {
	uint32_t sid;
	uint32_t cid;
	uint32_t kuid;
	void *src;
	uint32_t src_size;
	void *dst;
	uint32_t dst_size;
	int cpara_size;
	struct fcs_acs_crypt_parameter cpara;
};

/**
 * struct fcs_sha2_mac_data
 * @sid: session ID
 * @cid: context ID
 * @kuid: key UID
 * @src: source
 * @src_size: size of source
 * @dst: destination
 * @dst_size: size of destination
 * @sha_op_mode: SHA operating mode
 * @sha_digest_sz: SHA digest size
 */
struct fcs_sha2_mac_data {
	uint32_t sid;
	uint32_t cid;
	uint32_t kuid;
	void *src;
	uint32_t src_size;
	void *dst;
	uint32_t dst_size;
	int sha_op_mode;
	int sha_digest_sz;
	uint32_t userdata_sz;
};

/**
 * struct fcs_ecdsa_data
 * @sid: session ID
 * @cid: context ID
 * @kuid: key UID
 * @src: source
 * @src_size: size of source
 * @dst: destination
 * @dst_size: size of destination
 * @ecc_algorithm: ECC algorithm
 */
struct fcs_ecdsa_data {
	uint32_t sid;
	uint32_t cid;
	uint32_t kuid;
	void *src;
	uint32_t src_size;
	void *dst;
	uint32_t dst_size;
	int ecc_algorithm;
};

/**
 * struct fcs_ecdsa_sha2_data
 * @sid: session ID
 * @cid: context ID
 * @kuid: key UID
 * @src: pointer of source
 * @src_size: size of source
 * @dst: pointer of destination
 * @dst_size: size of destination
 * @ecc_algorithm: ECC algorithm
 * @userdata_sz: size of user data
 */
struct fcs_ecdsa_sha2_data {
       uint32_t sid;
       uint32_t cid;
       uint32_t kuid;
       void *src;
       uint32_t src_size;
       void *dst;
       uint32_t dst_size;
       int ecc_algorithm;
       uint32_t userdata_sz;
};

/**
 * struct fcs_random_number_gen_ext
 * @sid: session ID
 * @cid: context ID
 * @rng_data: random data
 * @rng_sz: size of random data
 */
struct fcs_random_number_gen_ext {
       uint32_t sid;
       uint32_t cid;
       void *rng_data;
       uint32_t rng_sz;
};

/**
 * struct fcs_sdos_data_ext - SDOS encryption/decryption
 * @sid: session ID
 * @cid: context ID
 * @op_mode: SDOS operation mode
 * 	1	encryption
 * 	0	decryption
 * @oid1: owner ID word 1, valid for date decryption only
 * @oid2: owner ID word 2, valid for date decryption only
 * @src: the virtual address of the input data
 * @src_size: the size of the input data
 * @dst: the virtual address of the output data
 * dst_size: the size of the output data
 */
struct fcs_sdos_data_ext {
	uint32_t sid;
	uint32_t cid;
	int op_mode;
	void *src;
	uint32_t src_size;
	void *dst;
	uint32_t dst_size;
};

/**
 * struct intel_fcs_dev_ioctl: common structure passed to Linux
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
 * @sdos_data_ext: SDOS ext data
 */
struct intel_fcs_dev_ioctl {
	/* used for return status code */
	int status;

	/* command parameters */
	union {
		struct fcs_mbox_send_cmd	mbox_send_cmd;
		struct fcs_placeholder		placeholder;
		struct fcs_validation_request	s_request;
		struct fcs_certificate_request	c_request;
		struct fcs_single_certificate_request	i_request;
		struct fcs_key_manage_request	gp_data;
		struct fcs_data_encryption	d_encryption;
		struct fcs_data_decryption	d_decryption;
		struct fcs_random_number_gen	rn_gen;
		struct fcs_psgsigma_teardown	tdown;
		struct fcs_attestation_chipid	c_id;
		struct fcs_attestation_subkey	subkey;
		struct fcs_attestation_measuerments	measurement;
		struct fcs_attestation_certificate	certificate;
		struct fcs_attestation_certificate_reload	c_reload;
		struct fcs_rom_patch_sha384 sha384;
		struct fcs_crypto_service_session	s_session;
		struct fcs_crypto_key_import		k_import;
		struct fcs_crypto_key_object		k_object;
		struct fcs_aes_crypt		a_crypt;
		struct fcs_sha2_mac_data	s_mac_data;
		struct fcs_ecdsa_data		ecdsa_data;
		struct fcs_ecdsa_sha2_data	ecdsa_sha2_data;
		struct fcs_random_number_gen_ext	rn_gen_ext;
		struct fcs_sdos_data_ext	data_sdos_ext;
	} com_paras;
};

/**
 * intel_fcs_command_code - support fpga crypto service commands
 *
 * Values are subject to change as a result of upstreaming.
 *
 * @INTEL_FCS_DEV_VERSION_CMD:
 *
 * @INTEL_FCS_DEV_MBOX_SEND_CMD:
 *
 * @INTEL_FCS_DEV_CERTIFICATE_CMD:
 *
 * @INTEL_FCS_DEV_VALIDATE_REQUEST_CMD:
 *
 * @INTEL_FCS_DEV_COUNTER_SET_CMD:
 *
 * @INTEL_FCS_DEV_COUNTER_SET_PREAUTHORIZED_CMD:
 *
 * @INTEL_FCS_DEV_GET_PROVISION_DATA_CMD:
 *
 * @INTEL_FCS_DEV_DATA_ENCRYPTION_CMD:
 *
 * @INTEL_FCS_DEV_DATA_DECRYPTION_CMD:
 *
 * @INTEL_FCS_DEV_RANDOM_NUMBER_GEN_CMD:
 *
 * @INTEL_FCS_DEV_GET_ROM_PATCH_SHA384_CMD:
 */
enum intel_fcs_command_code {
	INTEL_FCS_DEV_COMMAND_NONE = 0,
	INTEL_FCS_DEV_VERSION_CMD = 1,
	INTEL_FCS_DEV_MBOX_SEND_CMD,
	INTEL_FCS_DEV_CERTIFICATE_CMD = 0xB,
	INTEL_FCS_DEV_VALIDATE_REQUEST_CMD = 0x78,
	INTEL_FCS_DEV_COUNTER_SET_CMD,
	INTEL_FCS_DEV_COUNTER_SET_PREAUTHORIZED_CMD,
	INTEL_FCS_DEV_GET_PROVISION_DATA_CMD,
	INTEL_FCS_DEV_DATA_ENCRYPTION_CMD = 0x7E,
	INTEL_FCS_DEV_DATA_DECRYPTION_CMD,
	INTEL_FCS_DEV_RANDOM_NUMBER_GEN_CMD,
	INTEL_FCS_DEV_PSGSIGMA_TEARDOWN_CMD = 0x88,
	INTEL_FCS_DEV_CHIP_ID_CMD,
	INTEL_FCS_DEV_ATTESTATION_SUBKEY_CMD,
	INTEL_FCS_DEV_ATTESTATION_MEASUREMENT_CMD,
	INTEL_FCS_DEV_ATTESTATION_GET_CERTIFICATE_CMD,
	INTEL_FCS_DEV_ATTESTATION_CERTIFICATE_RELOAD_CMD,
	INTEL_FCS_DEV_GET_ROM_PATCH_SHA384_CMD,
	INTEL_FCS_DEV_CRYPTO_OPEN_SESSION_CMD = 0xA0,
	INTEL_FCS_DEV_CRYPTO_CLOSE_SESSION_CMD,
	INTEL_FCS_DEV_CRYPTO_IMPORT_KEY_CMD,
	INTEL_FCS_DEV_CRYPTO_EXPORT_KEY_CMD,
	INTEL_FCS_DEV_CRYPTO_REMOVE_KEY_CMD,
	INTEL_FCS_DEV_CRYPTO_GET_KEY_INFO_CMD,
	INTEL_FCS_DEV_CRYPTO_AES_CRYPT_CMD,
	INTEL_FCS_DEV_CRYPTO_GET_DIGEST_CMD,
	INTEL_FCS_DEV_CRYPTO_MAC_VERIFY_CMD,
	INTEL_FCS_DEV_CRYPTO_ECDSA_HASH_SIGNING_CMD,
	INTEL_FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING_CMD,
	INTEL_FCS_DEV_CRYPTO_ECDSA_HASH_VERIFY_CMD,
	INTEL_FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_CMD,
	INTEL_FCS_DEV_CRYPTO_ECDSA_GET_PUBLIC_KEY_CMD,
	INTEL_FCS_DEV_CRYPTO_ECDH_REQUEST_CMD,
	INTEL_FCS_DEV_RANDOM_NUMBER_GEN_EXT_CMD,
	INTEL_FCS_DEV_SDOS_DATA_EXT_CMD,
};

#define INTEL_FCS_DEV_VERSION_REQUEST \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_VERSION_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_MBOX_SEND \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_MBOX_SEND_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_VALIDATION_REQUEST \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_VALIDATE_REQUEST_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_SEND_CERTIFICATE \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CERTIFICATE_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_COUNTER_SET_PREAUTHORIZED \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_COUNTER_SET_PREAUTHORIZED_CMD, struct intel_fcs_dev_ioctl)

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

#define INTEL_FCS_DEV_PSGSIGMA_TEARDOWN \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_PSGSIGMA_TEARDOWN_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CHIP_ID \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CHIP_ID_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_ATTESTATION_SUBKEY \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_ATTESTATION_SUBKEY_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_ATTESTATION_MEASUREMENT \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_ATTESTATION_MEASUREMENT_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_ATTESTATION_GET_CERTIFICATE \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_ATTESTATION_GET_CERTIFICATE_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_ATTESTATION_CERTIFICATE_RELOAD \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_ATTESTATION_CERTIFICATE_RELOAD_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_GET_ROM_PATCH_SHA384 \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_GET_ROM_PATCH_SHA384_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_OPEN_SESSION \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_OPEN_SESSION_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_CLOSE_SESSION \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_CLOSE_SESSION_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_IMPORT_KEY \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_IMPORT_KEY_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_EXPORT_KEY \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_EXPORT_KEY_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_REMOVE_KEY \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_REMOVE_KEY_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_GET_KEY_INFO \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_GET_KEY_INFO_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_AES_CRYPT \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_AES_CRYPT_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_GET_DIGEST \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_GET_DIGEST_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_MAC_VERIFY \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_MAC_VERIFY_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_ECDSA_HASH_SIGNING \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_ECDSA_HASH_SIGNING_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_SIGNING_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_ECDSA_HASH_VERIFY \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_ECDSA_HASH_VERIFY_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_ECDSA_SHA2_DATA_VERIFY_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_ECDSA_GET_PUBLIC_KEY \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_ECDSA_GET_PUBLIC_KEY_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_CRYPTO_ECDH_REQUEST \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_CRYPTO_ECDH_REQUEST_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_RANDOM_NUMBER_GEN_EXT \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_RANDOM_NUMBER_GEN_EXT_CMD, struct intel_fcs_dev_ioctl)

#define INTEL_FCS_DEV_SDOS_DATA_EXT \
	_IOWR(INTEL_FCS_IOCTL, \
	      INTEL_FCS_DEV_SDOS_DATA_EXT_CMD, struct intel_fcs_dev_ioctl)

#endif

