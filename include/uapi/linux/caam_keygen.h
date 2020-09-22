/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * NXP CAAM key generation ioctl and kernel interface
 *
 * Copyright 2020 NXP
 *
 * This file is used by the NXP CAAM Crypto driver.
 * It can be included by applications that need to communicate
 * with the driver via the ioctl interface.
 */

#ifndef _UAPI_CAAM_KEYGEN_H
#define _UAPI_CAAM_KEYGEN_H

#include <linux/types.h>

/**
 * struct caam_keygen_cmd - Structure that contains all the necessary
 *                          information to transfer user-space data to
 *                          kernel space. This data is used to generate
 *                          a black key and encapsulate it into a blob
 *
 * @key_enc_len        : Length of the key_enc field
 * @key_mode_len       : Length of the key_mode field
 * @key_value_len      : Length of the key_value field
 * @black_key_len      : Length of the generated black key
 * @key_enc            : Encrypted Key Type
 *                       Can be either ecb or ccm (AES-ECB or AES-CCM)
 * @key_mode           : The key mode used to generate the black key
 *                       Can be either -s (for random black key) or
 *                       -t (for black key generated from a plaintext)
 * @key_value          : Based on the key_mode field, can be either the
 *                       plaintext (for black key generated from a plaintext)
 *                       or size of key in case of black key generated
 *                       from random
 * @black_key          : Black key data obtained from CAAM
 * @blob_len           : Length of the blob that encapsulates the black key
 * @blob               : Blob data obtained from CAAM
 */
struct caam_keygen_cmd {
	__u8 key_enc_len;
	__u8 key_mode_len;
	__u16 key_value_len;

	__u32 black_key_len;

	__u64 key_enc;
	__u64 key_mode;
	__u64 key_value;

	__u64 black_key;

	__u32 blob_len;
	__u64 blob;
};

/* The ioctl type, documented in ioctl-number.txt */
#define CAAM_KEYGEN_IOCTL_TYPE		'K'

/* Create a key */
#define CAAM_KEYGEN_IOCTL_CREATE \
	_IOWR(CAAM_KEYGEN_IOCTL_TYPE, 0, struct caam_keygen_cmd)

/* Import a key from a blob */
#define CAAM_KEYGEN_IOCTL_IMPORT \
	_IOWR(CAAM_KEYGEN_IOCTL_TYPE, 1, struct caam_keygen_cmd)

#endif /* _UAPI_CAAM_KEYGEN_H */
