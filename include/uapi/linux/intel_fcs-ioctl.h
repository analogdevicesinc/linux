/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020, Intel Corporation
 */

#ifndef __INTEL_FCS_IOCTL_H
#define __INTEL_FCS_IOCTL_H

/* the value may need be changed when upstream */
#define INTEL_FCS_IOCTL		0xC0

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

