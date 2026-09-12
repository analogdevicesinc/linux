/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __TRUSTED_TPM_H
#define __TRUSTED_TPM_H

#include <keys/trusted-type.h>

extern struct trusted_key_ops trusted_key_tpm_ops;

struct trusted_key_tpm {
	uint32_t keyhandle;
	unsigned char keyauth[TPM_DIGEST_SIZE];
	uint32_t blobauth_len;
	unsigned char blobauth[TPM_DIGEST_SIZE];
	uint32_t pcrinfo_len;
	unsigned char pcrinfo[MAX_PCRINFO_SIZE];
	int pcrlock;
	uint32_t hash;
	uint32_t policydigest_len;
	unsigned char policydigest[MAX_DIGEST_SIZE];
	uint32_t policyhandle;
};

int tpm2_seal_trusted(struct tpm_chip *chip,
		      struct trusted_key_payload *payload,
		      struct trusted_key_options *options);
int tpm2_unseal_trusted(struct tpm_chip *chip,
			struct trusted_key_payload *payload,
			struct trusted_key_options *options);

#endif
