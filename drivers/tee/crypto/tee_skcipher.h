/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023 NXP
 */

#ifndef __TEE_SKCIPHER_H__
#define __TEE_SKCIPHER_H__

#include <linux/arm-smccc.h>

#define OPTEE_SMC_FAST_CALL_VAL(func_num) \
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_32, \
			   ARM_SMCCC_OWNER_TRUSTED_OS, (func_num))

#define TEE_CRYPTO_CRA_PRIORITY        3000

/* Commands implemented by the PTA */
/*
 * Set AES CBC symmetric key
 *
 * [in]     param[0].memref        Salt used to derive a key
 */
#define PTA_SET_CBC_KEY 0

/*
 * Set AES XTS symmetric keys
 *
 * [in]     param[0].memref        Salt used to derive keys
 */
#define PTA_SET_XTS_KEY 1

/*
 * Remove secrets keys according to key id
 *
 * [in]     param[0].value.a       Key id 1
 * [in]     param[0].value.b       Key id 2
 */
#define PTA_REMOVE_KEY  2

/*
 * Do AES CBC Encryption
 *
 * [in]     param[0].memref        Parameters buffer
 * [in]     param[1].value.a       Key id
 */
#define PTA_ENCRYPT_CBC 3

/*
 * Do AES CBC Decryption
 *
 * [in]     param[0].memref        Parameters buffer
 * [in]     param[1].value.a       Key id
 */
#define PTA_DECRYPT_CBC 4

/*
 * Do AES XTS Encryption
 *
 * [in]     param[0].memref        Parameters buffer
 * [in]     param[1].value.a       Key id 1
 * [in]     param[1].value.b       Key id 2
 */
#define PTA_ENCRYPT_XTS 5

/*
 * Do AES XTS Decryption
 *
 * [in]     param[0].memref        Parameters buffer
 * [in]     param[1].value.a       Key id 1
 * [in]     param[1].value.b       Key id 2
 */
#define PTA_DECRYPT_XTS 6

/*
 * Allocate a static shared memory buffer
 *
 * [in]     param[0].value.a       Buffer size
 * [out]    param[1].value.a       MSB Buffer physical address
 * [out]    param[1].value.b       LSB Buffer physical address
 */
#define PTA_SHM_ALLOCATE 7

/*
 * Free a static shared memory buffer
 *
 * [in]    param[0].value.a       MSB Buffer physical address
 * [in]    param[0].value.b       LSB Buffer physical address
 */
#define PTA_SHM_FREE	 8

/* Commands implemented by the PTA */

/*
 * AES key sizes
 */
#define AES_128 128
#define AES_256 256

#define TA_AES_SIZE_128 (AES_128 >> 3)
#define TA_AES_SIZE_256 (AES_256 >> 3)

/*
 * Do AES CBC Encryption
 *
 * Call register usage:
 * a0	SMC Function ID, IMX_SMC_ENCRYPT_CBC
 * a1	Key ids
 * a2	Initial vector physical address
 * a3	Input buffer physical address
 * a4	Input buffer length
 * a5	Output buffer physical address
 * a6	Output buffer length
 * a7	Not used
 *
 * Normal return register usage:
 * a0	OPTEE_SMC_RETURN_OK
 * a1-3	Not used
 * a4-7	Preserved
 *
 * OPTEE_SMC_RETURN_EBADCMD on Invalid input offset:
 * a0	OPTEE_SMC_RETURN_EBADCMD
 * a1-3	Not used
 * a4-7	Preserved
 */
#define IMX_SMC_FUNCID_ENCRYPT_CBC	20
#define IMX_SMC_ENCRYPT_CBC \
	OPTEE_SMC_FAST_CALL_VAL(IMX_SMC_FUNCID_ENCRYPT_CBC)

/*
 * Do AES CBC Decryption
 *
 * Call register usage:
 * a0	SMC Function ID, IMX_SMC_DECRYPT_CBC
 * a1	Key ids
 * a2	Initial vector physical address
 * a3	Input buffer physical address
 * a4	Input buffer length
 * a5	Output buffer physical address
 * a6	Output buffer length
 * a7	Not used
 *
 * Normal return register usage:
 * a0	OPTEE_SMC_RETURN_OK
 * a1-3	Not used
 * a4-7	Preserved
 *
 * OPTEE_SMC_RETURN_EBADCMD on Invalid input offset:
 * a0	OPTEE_SMC_RETURN_EBADCMD
 * a1-3	Not used
 * a4-7	Preserved
 */
#define IMX_SMC_FUNCID_DECRYPT_CBC	21
#define IMX_SMC_DECRYPT_CBC \
	OPTEE_SMC_FAST_CALL_VAL(IMX_SMC_FUNCID_DECRYPT_CBC)

/*
 * Do AES XTS Encryption
 *
 * Call register usage:
 * a0	SMC Function ID, IMX_SMC_ENCRYPT_XTS
 * a1	Key ids
 * a2	Initial vector physical address
 * a3	Input buffer physical address
 * a4	Input buffer length
 * a5	Output buffer physical address
 * a6	Output buffer length
 * a7	Not used
 *
 * Normal return register usage:
 * a0	OPTEE_SMC_RETURN_OK
 * a1-3	Not used
 * a4-7	Preserved
 *
 * OPTEE_SMC_RETURN_EBADCMD on Invalid input offset:
 * a0	OPTEE_SMC_RETURN_EBADCMD
 * a1-3	Not used
 * a4-7	Preserved
 */
#define IMX_SMC_FUNCID_ENCRYPT_XTS	22
#define IMX_SMC_ENCRYPT_XTS \
	OPTEE_SMC_FAST_CALL_VAL(IMX_SMC_FUNCID_ENCRYPT_XTS)

/*
 * Do AES XTS Decryption
 *
 * Call register usage:
 * a0	SMC Function ID, IMX_SMC_DECRYPT_XTS
 * a1	Key ids
 * a2	Initial vector physical address
 * a3	Input buffer physical address
 * a4	Input buffer length
 * a5	Output buffer physical address
 * a6	Output buffer length
 * a7	Not used
 *
 * Normal return register usage:
 * a0	OPTEE_SMC_RETURN_OK
 * a1-3	Not used
 * a4-7	Preserved
 *
 * OPTEE_SMC_RETURN_EBADCMD on Invalid input offset:
 * a0	OPTEE_SMC_RETURN_EBADCMD
 * a1-3	Not used
 * a4-7	Preserved
 */
#define IMX_SMC_FUNCID_DECRYPT_XTS	23
#define IMX_SMC_DECRYPT_XTS \
	OPTEE_SMC_FAST_CALL_VAL(IMX_SMC_FUNCID_DECRYPT_XTS)

#endif
