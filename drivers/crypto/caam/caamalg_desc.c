// SPDX-License-Identifier: GPL-2.0+
/*
 * Shared descriptors for aead, skcipher algorithms
 *
 * Copyright 2016-2019 NXP
 */

#include "compat.h"
#include "desc_constr.h"
#include "caamalg_desc.h"

/*
 * For aead functions, read payload and write payload,
 * both of which are specified in req->src and req->dst
 */
static inline void aead_append_src_dst(u32 *desc, u32 msg_type)
{
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_MESSAGE_DATA | KEY_VLF);
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_BOTH |
			     KEY_VLF | msg_type | FIFOLD_TYPE_LASTBOTH);
}

/* Set DK bit in class 1 operation if shared */
static inline void append_dec_op1(u32 *desc, u32 type)
{
	u32 *jump_cmd, *uncond_jump_cmd;

	/* DK bit is valid only for AES */
	if ((type & OP_ALG_ALGSEL_MASK) != OP_ALG_ALGSEL_AES) {
		append_operation(desc, type | OP_ALG_AS_INITFINAL |
				 OP_ALG_DECRYPT);
		return;
	}

	jump_cmd = append_jump(desc, JUMP_TEST_ALL | JUMP_COND_SHRD);
	append_operation(desc, type | OP_ALG_AS_INIT | OP_ALG_DECRYPT);
	uncond_jump_cmd = append_jump(desc, JUMP_TEST_ALL);
	set_jump_tgt_here(desc, jump_cmd);
	append_operation(desc, type | OP_ALG_AS_INIT | OP_ALG_DECRYPT |
			 OP_ALG_AAI_DK);
	set_jump_tgt_here(desc, uncond_jump_cmd);
}

/**
 * cnstr_shdsc_aead_null_encap - IPSec ESP encapsulation shared descriptor
 *                               (non-protocol) with no (null) encryption.
 * @desc: pointer to buffer used for descriptor construction
 * @adata: pointer to authentication transform definitions.
 *         A split key is required for SEC Era < 6; the size of the split key
 *         is specified in this case. Valid algorithm values - one of
 *         OP_ALG_ALGSEL_{MD5, SHA1, SHA224, SHA256, SHA384, SHA512} ANDed
 *         with OP_ALG_AAI_HMAC_PRECOMP.
 * @icvsize: integrity check value (ICV) size (truncated or full)
 * @era: SEC Era
 */
void cnstr_shdsc_aead_null_encap(u32 * const desc, struct alginfo *adata,
				 unsigned int icvsize, int era)
{
	u32 *key_jump_cmd, *read_move_cmd, *write_move_cmd;

	init_sh_desc(desc, HDR_SHARE_SERIAL);

	/* Skip if already shared */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);
	if (era < 6) {
		if (adata->key_inline)
			append_key_as_imm(desc, adata->key_virt,
					  adata->keylen_pad, adata->keylen,
					  CLASS_2 | KEY_DEST_MDHA_SPLIT |
					  KEY_ENC);
		else
			append_key(desc, adata->key_dma, adata->keylen,
				   CLASS_2 | KEY_DEST_MDHA_SPLIT | KEY_ENC);
	} else {
		append_proto_dkp(desc, adata);
	}
	set_jump_tgt_here(desc, key_jump_cmd);

	/* assoclen + cryptlen = seqinlen */
	append_math_sub(desc, REG3, SEQINLEN, REG0, CAAM_CMD_SZ);

	/* Prepare to read and write cryptlen + assoclen bytes */
	append_math_add(desc, VARSEQINLEN, ZERO, REG3, CAAM_CMD_SZ);
	append_math_add(desc, VARSEQOUTLEN, ZERO, REG3, CAAM_CMD_SZ);

	/*
	 * MOVE_LEN opcode is not available in all SEC HW revisions,
	 * thus need to do some magic, i.e. self-patch the descriptor
	 * buffer.
	 */
	read_move_cmd = append_move(desc, MOVE_SRC_DESCBUF |
				    MOVE_DEST_MATH3 |
				    (0x6 << MOVE_LEN_SHIFT));
	write_move_cmd = append_move(desc, MOVE_SRC_MATH3 |
				     MOVE_DEST_DESCBUF |
				     MOVE_WAITCOMP |
				     (0x8 << MOVE_LEN_SHIFT));

	/* Class 2 operation */
	append_operation(desc, adata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_ENCRYPT);

	/* Read and write cryptlen bytes */
	aead_append_src_dst(desc, FIFOLD_TYPE_MSG | FIFOLD_TYPE_FLUSH1);

	set_move_tgt_here(desc, read_move_cmd);
	set_move_tgt_here(desc, write_move_cmd);
	append_cmd(desc, CMD_LOAD | DISABLE_AUTO_INFO_FIFO);
	append_move(desc, MOVE_SRC_INFIFO_CL | MOVE_DEST_OUTFIFO |
		    MOVE_AUX_LS);

	/* Write ICV */
	append_seq_store(desc, icvsize, LDST_CLASS_2_CCB |
			 LDST_SRCDST_BYTE_CONTEXT);

	print_hex_dump_debug("aead null enc shdesc@" __stringify(__LINE__)": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc),
			     1);
}
EXPORT_SYMBOL(cnstr_shdsc_aead_null_encap);

/**
 * cnstr_shdsc_aead_null_decap - IPSec ESP decapsulation shared descriptor
 *                               (non-protocol) with no (null) decryption.
 * @desc: pointer to buffer used for descriptor construction
 * @adata: pointer to authentication transform definitions.
 *         A split key is required for SEC Era < 6; the size of the split key
 *         is specified in this case. Valid algorithm values - one of
 *         OP_ALG_ALGSEL_{MD5, SHA1, SHA224, SHA256, SHA384, SHA512} ANDed
 *         with OP_ALG_AAI_HMAC_PRECOMP.
 * @icvsize: integrity check value (ICV) size (truncated or full)
 * @era: SEC Era
 */
void cnstr_shdsc_aead_null_decap(u32 * const desc, struct alginfo *adata,
				 unsigned int icvsize, int era)
{
	u32 *key_jump_cmd, *read_move_cmd, *write_move_cmd, *jump_cmd;

	init_sh_desc(desc, HDR_SHARE_SERIAL);

	/* Skip if already shared */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);
	if (era < 6) {
		if (adata->key_inline)
			append_key_as_imm(desc, adata->key_virt,
					  adata->keylen_pad, adata->keylen,
					  CLASS_2 | KEY_DEST_MDHA_SPLIT |
					  KEY_ENC);
		else
			append_key(desc, adata->key_dma, adata->keylen,
				   CLASS_2 | KEY_DEST_MDHA_SPLIT | KEY_ENC);
	} else {
		append_proto_dkp(desc, adata);
	}
	set_jump_tgt_here(desc, key_jump_cmd);

	/* Class 2 operation */
	append_operation(desc, adata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_DECRYPT | OP_ALG_ICV_ON);

	/* assoclen + cryptlen = seqoutlen */
	append_math_sub(desc, REG2, SEQOUTLEN, REG0, CAAM_CMD_SZ);

	/* Prepare to read and write cryptlen + assoclen bytes */
	append_math_add(desc, VARSEQINLEN, ZERO, REG2, CAAM_CMD_SZ);
	append_math_add(desc, VARSEQOUTLEN, ZERO, REG2, CAAM_CMD_SZ);

	/*
	 * MOVE_LEN opcode is not available in all SEC HW revisions,
	 * thus need to do some magic, i.e. self-patch the descriptor
	 * buffer.
	 */
	read_move_cmd = append_move(desc, MOVE_SRC_DESCBUF |
				    MOVE_DEST_MATH2 |
				    (0x6 << MOVE_LEN_SHIFT));
	write_move_cmd = append_move(desc, MOVE_SRC_MATH2 |
				     MOVE_DEST_DESCBUF |
				     MOVE_WAITCOMP |
				     (0x8 << MOVE_LEN_SHIFT));

	/* Read and write cryptlen bytes */
	aead_append_src_dst(desc, FIFOLD_TYPE_MSG | FIFOLD_TYPE_FLUSH1);

	/*
	 * Insert a NOP here, since we need at least 4 instructions between
	 * code patching the descriptor buffer and the location being patched.
	 */
	jump_cmd = append_jump(desc, JUMP_TEST_ALL);
	set_jump_tgt_here(desc, jump_cmd);

	set_move_tgt_here(desc, read_move_cmd);
	set_move_tgt_here(desc, write_move_cmd);
	append_cmd(desc, CMD_LOAD | DISABLE_AUTO_INFO_FIFO);
	append_move(desc, MOVE_SRC_INFIFO_CL | MOVE_DEST_OUTFIFO |
		    MOVE_AUX_LS);
	append_cmd(desc, CMD_LOAD | ENABLE_AUTO_INFO_FIFO);

	/* Load ICV */
	append_seq_fifo_load(desc, icvsize, FIFOLD_CLASS_CLASS2 |
			     FIFOLD_TYPE_LAST2 | FIFOLD_TYPE_ICV);

	print_hex_dump_debug("aead null dec shdesc@" __stringify(__LINE__)": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc),
			     1);
}
EXPORT_SYMBOL(cnstr_shdsc_aead_null_decap);

static void init_sh_desc_key_aead(u32 * const desc,
				  struct alginfo * const cdata,
				  struct alginfo * const adata,
				  const bool is_rfc3686, u32 *nonce, int era)
{
	u32 *key_jump_cmd;
	unsigned int enckeylen = cdata->keylen;

	/* Note: Context registers are saved. */
	init_sh_desc(desc, HDR_SHARE_SERIAL | HDR_SAVECTX);

	/* Skip if already shared */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);

	/*
	 * RFC3686 specific:
	 *	| key = {AUTH_KEY, ENC_KEY, NONCE}
	 *	| enckeylen = encryption key size + nonce size
	 */
	if (is_rfc3686)
		enckeylen -= CTR_RFC3686_NONCE_SIZE;

	if (era < 6) {
		if (adata->key_inline)
			append_key_as_imm(desc, adata->key_virt,
					  adata->keylen_pad, adata->keylen,
					  CLASS_2 | KEY_DEST_MDHA_SPLIT |
					  KEY_ENC);
		else
			append_key(desc, adata->key_dma, adata->keylen,
				   CLASS_2 | KEY_DEST_MDHA_SPLIT | KEY_ENC);
	} else {
		append_proto_dkp(desc, adata);
	}

	if (cdata->key_inline)
		append_key_as_imm(desc, cdata->key_virt, enckeylen,
				  enckeylen, CLASS_1 | KEY_DEST_CLASS_REG);
	else
		append_key(desc, cdata->key_dma, enckeylen, CLASS_1 |
			   KEY_DEST_CLASS_REG);

	/* Load Counter into CONTEXT1 reg */
	if (is_rfc3686) {
		append_load_as_imm(desc, nonce, CTR_RFC3686_NONCE_SIZE,
				   LDST_CLASS_IND_CCB |
				   LDST_SRCDST_BYTE_OUTFIFO | LDST_IMM);
		append_move(desc,
			    MOVE_SRC_OUTFIFO |
			    MOVE_DEST_CLASS1CTX |
			    (16 << MOVE_OFFSET_SHIFT) |
			    (CTR_RFC3686_NONCE_SIZE << MOVE_LEN_SHIFT));
	}

	set_jump_tgt_here(desc, key_jump_cmd);
}

/**
 * cnstr_shdsc_aead_encap - IPSec ESP encapsulation shared descriptor
 *                          (non-protocol).
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - one of OP_ALG_ALGSEL_{AES, DES, 3DES} ANDed
 *         with OP_ALG_AAI_CBC or OP_ALG_AAI_CTR_MOD128.
 * @adata: pointer to authentication transform definitions.
 *         A split key is required for SEC Era < 6; the size of the split key
 *         is specified in this case. Valid algorithm values - one of
 *         OP_ALG_ALGSEL_{MD5, SHA1, SHA224, SHA256, SHA384, SHA512} ANDed
 *         with OP_ALG_AAI_HMAC_PRECOMP.
 * @ivsize: initialization vector size
 * @icvsize: integrity check value (ICV) size (truncated or full)
 * @is_rfc3686: true when ctr(aes) is wrapped by rfc3686 template
 * @nonce: pointer to rfc3686 nonce
 * @ctx1_iv_off: IV offset in CONTEXT1 register
 * @is_qi: true when called from caam/qi
 * @era: SEC Era
 */
void cnstr_shdsc_aead_encap(u32 * const desc, struct alginfo *cdata,
			    struct alginfo *adata, unsigned int ivsize,
			    unsigned int icvsize, const bool is_rfc3686,
			    u32 *nonce, const u32 ctx1_iv_off, const bool is_qi,
			    int era)
{
	/* Note: Context registers are saved. */
	init_sh_desc_key_aead(desc, cdata, adata, is_rfc3686, nonce, era);

	/* Class 2 operation */
	append_operation(desc, adata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_ENCRYPT);

	if (is_qi) {
		u32 *wait_load_cmd;

		/* REG3 = assoclen */
		append_seq_load(desc, 4, LDST_CLASS_DECO |
				LDST_SRCDST_WORD_DECO_MATH3 |
				(4 << LDST_OFFSET_SHIFT));

		wait_load_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
					    JUMP_COND_CALM | JUMP_COND_NCP |
					    JUMP_COND_NOP | JUMP_COND_NIP |
					    JUMP_COND_NIFP);
		set_jump_tgt_here(desc, wait_load_cmd);

		append_seq_load(desc, ivsize, LDST_CLASS_1_CCB |
				LDST_SRCDST_BYTE_CONTEXT |
				(ctx1_iv_off << LDST_OFFSET_SHIFT));
	}

	/* Read and write assoclen bytes */
	if (is_qi || era < 3) {
		append_math_add(desc, VARSEQINLEN, ZERO, REG3, CAAM_CMD_SZ);
		append_math_add(desc, VARSEQOUTLEN, ZERO, REG3, CAAM_CMD_SZ);
	} else {
		append_math_add(desc, VARSEQINLEN, ZERO, DPOVRD, CAAM_CMD_SZ);
		append_math_add(desc, VARSEQOUTLEN, ZERO, DPOVRD, CAAM_CMD_SZ);
	}

	/* Skip assoc data */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_SKIP | FIFOLDST_VLF);

	/* read assoc before reading payload */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS2 | FIFOLD_TYPE_MSG |
				      FIFOLDST_VLF);

	/* Load Counter into CONTEXT1 reg */
	if (is_rfc3686)
		append_load_imm_be32(desc, 1, LDST_IMM | LDST_CLASS_1_CCB |
				     LDST_SRCDST_BYTE_CONTEXT |
				     ((ctx1_iv_off + CTR_RFC3686_IV_SIZE) <<
				      LDST_OFFSET_SHIFT));

	/* Class 1 operation */
	append_operation(desc, cdata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_ENCRYPT);

	/* Read and write cryptlen bytes */
	append_math_add(desc, VARSEQINLEN, SEQINLEN, REG0, CAAM_CMD_SZ);
	append_math_add(desc, VARSEQOUTLEN, SEQINLEN, REG0, CAAM_CMD_SZ);
	aead_append_src_dst(desc, FIFOLD_TYPE_MSG1OUT2);

	/* Write ICV */
	append_seq_store(desc, icvsize, LDST_CLASS_2_CCB |
			 LDST_SRCDST_BYTE_CONTEXT);

	print_hex_dump_debug("aead enc shdesc@" __stringify(__LINE__)": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc),
			     1);
}
EXPORT_SYMBOL(cnstr_shdsc_aead_encap);

/**
 * cnstr_shdsc_aead_decap - IPSec ESP decapsulation shared descriptor
 *                          (non-protocol).
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - one of OP_ALG_ALGSEL_{AES, DES, 3DES} ANDed
 *         with OP_ALG_AAI_CBC or OP_ALG_AAI_CTR_MOD128.
 * @adata: pointer to authentication transform definitions.
 *         A split key is required for SEC Era < 6; the size of the split key
 *         is specified in this case. Valid algorithm values - one of
 *         OP_ALG_ALGSEL_{MD5, SHA1, SHA224, SHA256, SHA384, SHA512} ANDed
 *         with OP_ALG_AAI_HMAC_PRECOMP.
 * @ivsize: initialization vector size
 * @icvsize: integrity check value (ICV) size (truncated or full)
 * @geniv: whether to generate Encrypted Chain IV
 * @is_rfc3686: true when ctr(aes) is wrapped by rfc3686 template
 * @nonce: pointer to rfc3686 nonce
 * @ctx1_iv_off: IV offset in CONTEXT1 register
 * @is_qi: true when called from caam/qi
 * @era: SEC Era
 */
void cnstr_shdsc_aead_decap(u32 * const desc, struct alginfo *cdata,
			    struct alginfo *adata, unsigned int ivsize,
			    unsigned int icvsize, const bool geniv,
			    const bool is_rfc3686, u32 *nonce,
			    const u32 ctx1_iv_off, const bool is_qi, int era)
{
	/* Note: Context registers are saved. */
	init_sh_desc_key_aead(desc, cdata, adata, is_rfc3686, nonce, era);

	/* Class 2 operation */
	append_operation(desc, adata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_DECRYPT | OP_ALG_ICV_ON);

	if (is_qi) {
		u32 *wait_load_cmd;

		/* REG3 = assoclen */
		append_seq_load(desc, 4, LDST_CLASS_DECO |
				LDST_SRCDST_WORD_DECO_MATH3 |
				(4 << LDST_OFFSET_SHIFT));

		wait_load_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
					    JUMP_COND_CALM | JUMP_COND_NCP |
					    JUMP_COND_NOP | JUMP_COND_NIP |
					    JUMP_COND_NIFP);
		set_jump_tgt_here(desc, wait_load_cmd);

		if (!geniv)
			append_seq_load(desc, ivsize, LDST_CLASS_1_CCB |
					LDST_SRCDST_BYTE_CONTEXT |
					(ctx1_iv_off << LDST_OFFSET_SHIFT));
	}

	/* Read and write assoclen bytes */
	if (is_qi || era < 3) {
		append_math_add(desc, VARSEQINLEN, ZERO, REG3, CAAM_CMD_SZ);
		if (geniv)
			append_math_add_imm_u32(desc, VARSEQOUTLEN, REG3, IMM,
						ivsize);
		else
			append_math_add(desc, VARSEQOUTLEN, ZERO, REG3,
					CAAM_CMD_SZ);
	} else {
		append_math_add(desc, VARSEQINLEN, ZERO, DPOVRD, CAAM_CMD_SZ);
		if (geniv)
			append_math_add_imm_u32(desc, VARSEQOUTLEN, DPOVRD, IMM,
						ivsize);
		else
			append_math_add(desc, VARSEQOUTLEN, ZERO, DPOVRD,
					CAAM_CMD_SZ);
	}

	/* Skip assoc data */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_SKIP | FIFOLDST_VLF);

	/* read assoc before reading payload */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS2 | FIFOLD_TYPE_MSG |
			     KEY_VLF);

	if (geniv) {
		append_seq_load(desc, ivsize, LDST_CLASS_1_CCB |
				LDST_SRCDST_BYTE_CONTEXT |
				(ctx1_iv_off << LDST_OFFSET_SHIFT));
		append_move(desc, MOVE_SRC_CLASS1CTX | MOVE_DEST_CLASS2INFIFO |
			    (ctx1_iv_off << MOVE_OFFSET_SHIFT) | ivsize);
	}

	/* Load Counter into CONTEXT1 reg */
	if (is_rfc3686)
		append_load_imm_be32(desc, 1, LDST_IMM | LDST_CLASS_1_CCB |
				     LDST_SRCDST_BYTE_CONTEXT |
				     ((ctx1_iv_off + CTR_RFC3686_IV_SIZE) <<
				      LDST_OFFSET_SHIFT));

	/* Choose operation */
	if (ctx1_iv_off)
		append_operation(desc, cdata->algtype | OP_ALG_AS_INITFINAL |
				 OP_ALG_DECRYPT);
	else
		append_dec_op1(desc, cdata->algtype);

	/* Read and write cryptlen bytes */
	append_math_add(desc, VARSEQINLEN, SEQOUTLEN, REG0, CAAM_CMD_SZ);
	append_math_add(desc, VARSEQOUTLEN, SEQOUTLEN, REG0, CAAM_CMD_SZ);
	aead_append_src_dst(desc, FIFOLD_TYPE_MSG);

	/* Load ICV */
	append_seq_fifo_load(desc, icvsize, FIFOLD_CLASS_CLASS2 |
			     FIFOLD_TYPE_LAST2 | FIFOLD_TYPE_ICV);

	print_hex_dump_debug("aead dec shdesc@" __stringify(__LINE__)": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc),
			     1);
}
EXPORT_SYMBOL(cnstr_shdsc_aead_decap);

/**
 * cnstr_shdsc_aead_givencap - IPSec ESP encapsulation shared descriptor
 *                             (non-protocol) with HW-generated initialization
 *                             vector.
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - one of OP_ALG_ALGSEL_{AES, DES, 3DES} ANDed
 *         with OP_ALG_AAI_CBC or OP_ALG_AAI_CTR_MOD128.
 * @adata: pointer to authentication transform definitions.
 *         A split key is required for SEC Era < 6; the size of the split key
 *         is specified in this case. Valid algorithm values - one of
 *         OP_ALG_ALGSEL_{MD5, SHA1, SHA224, SHA256, SHA384, SHA512} ANDed
 *         with OP_ALG_AAI_HMAC_PRECOMP.
 * @ivsize: initialization vector size
 * @icvsize: integrity check value (ICV) size (truncated or full)
 * @is_rfc3686: true when ctr(aes) is wrapped by rfc3686 template
 * @nonce: pointer to rfc3686 nonce
 * @ctx1_iv_off: IV offset in CONTEXT1 register
 * @is_qi: true when called from caam/qi
 * @era: SEC Era
 */
void cnstr_shdsc_aead_givencap(u32 * const desc, struct alginfo *cdata,
			       struct alginfo *adata, unsigned int ivsize,
			       unsigned int icvsize, const bool is_rfc3686,
			       u32 *nonce, const u32 ctx1_iv_off,
			       const bool is_qi, int era)
{
	u32 geniv, moveiv;
	u32 *wait_cmd;

	/* Note: Context registers are saved. */
	init_sh_desc_key_aead(desc, cdata, adata, is_rfc3686, nonce, era);

	if (is_qi) {
		u32 *wait_load_cmd;

		/* REG3 = assoclen */
		append_seq_load(desc, 4, LDST_CLASS_DECO |
				LDST_SRCDST_WORD_DECO_MATH3 |
				(4 << LDST_OFFSET_SHIFT));

		wait_load_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
					    JUMP_COND_CALM | JUMP_COND_NCP |
					    JUMP_COND_NOP | JUMP_COND_NIP |
					    JUMP_COND_NIFP);
		set_jump_tgt_here(desc, wait_load_cmd);
	}

	if (is_rfc3686) {
		if (is_qi)
			append_seq_load(desc, ivsize, LDST_CLASS_1_CCB |
					LDST_SRCDST_BYTE_CONTEXT |
					(ctx1_iv_off << LDST_OFFSET_SHIFT));

		goto copy_iv;
	}

	/* Generate IV */
	geniv = NFIFOENTRY_STYPE_PAD | NFIFOENTRY_DEST_DECO |
		NFIFOENTRY_DTYPE_MSG | NFIFOENTRY_LC1 |
		NFIFOENTRY_PTYPE_RND | (ivsize << NFIFOENTRY_DLEN_SHIFT);
	append_load_imm_u32(desc, geniv, LDST_CLASS_IND_CCB |
			    LDST_SRCDST_WORD_INFO_FIFO | LDST_IMM);
	append_cmd(desc, CMD_LOAD | DISABLE_AUTO_INFO_FIFO);
	append_move(desc, MOVE_WAITCOMP |
		    MOVE_SRC_INFIFO | MOVE_DEST_CLASS1CTX |
		    (ctx1_iv_off << MOVE_OFFSET_SHIFT) |
		    (ivsize << MOVE_LEN_SHIFT));
	append_cmd(desc, CMD_LOAD | ENABLE_AUTO_INFO_FIFO);

copy_iv:
	/* Copy IV to class 1 context */
	append_move(desc, MOVE_SRC_CLASS1CTX | MOVE_DEST_OUTFIFO |
		    (ctx1_iv_off << MOVE_OFFSET_SHIFT) |
		    (ivsize << MOVE_LEN_SHIFT));

	/* Return to encryption */
	append_operation(desc, adata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_ENCRYPT);

	/* Read and write assoclen bytes */
	if (is_qi || era < 3) {
		append_math_add(desc, VARSEQINLEN, ZERO, REG3, CAAM_CMD_SZ);
		append_math_add(desc, VARSEQOUTLEN, ZERO, REG3, CAAM_CMD_SZ);
	} else {
		append_math_add(desc, VARSEQINLEN, ZERO, DPOVRD, CAAM_CMD_SZ);
		append_math_add(desc, VARSEQOUTLEN, ZERO, DPOVRD, CAAM_CMD_SZ);
	}

	/* Skip assoc data */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_SKIP | FIFOLDST_VLF);

	/* read assoc before reading payload */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS2 | FIFOLD_TYPE_MSG |
			     KEY_VLF);

	/* Copy iv from outfifo to class 2 fifo */
	moveiv = NFIFOENTRY_STYPE_OFIFO | NFIFOENTRY_DEST_CLASS2 |
		 NFIFOENTRY_DTYPE_MSG | (ivsize << NFIFOENTRY_DLEN_SHIFT);
	append_load_imm_u32(desc, moveiv, LDST_CLASS_IND_CCB |
			    LDST_SRCDST_WORD_INFO_FIFO | LDST_IMM);
	append_load_imm_u32(desc, ivsize, LDST_CLASS_2_CCB |
			    LDST_SRCDST_WORD_DATASZ_REG | LDST_IMM);

	/* Load Counter into CONTEXT1 reg */
	if (is_rfc3686)
		append_load_imm_be32(desc, 1, LDST_IMM | LDST_CLASS_1_CCB |
				     LDST_SRCDST_BYTE_CONTEXT |
				     ((ctx1_iv_off + CTR_RFC3686_IV_SIZE) <<
				      LDST_OFFSET_SHIFT));

	/* Class 1 operation */
	append_operation(desc, cdata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_ENCRYPT);

	/* Will write ivsize + cryptlen */
	append_math_add(desc, VARSEQOUTLEN, SEQINLEN, REG0, CAAM_CMD_SZ);

	/* Not need to reload iv */
	append_seq_fifo_load(desc, ivsize,
			     FIFOLD_CLASS_SKIP);

	/* Will read cryptlen */
	append_math_add(desc, VARSEQINLEN, SEQINLEN, REG0, CAAM_CMD_SZ);

	/*
	 * Wait for IV transfer (ofifo -> class2) to finish before starting
	 * ciphertext transfer (ofifo -> external memory).
	 */
	wait_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL | JUMP_COND_NIFP);
	set_jump_tgt_here(desc, wait_cmd);

	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_BOTH | KEY_VLF |
			     FIFOLD_TYPE_MSG1OUT2 | FIFOLD_TYPE_LASTBOTH);
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_MESSAGE_DATA | KEY_VLF);

	/* Write ICV */
	append_seq_store(desc, icvsize, LDST_CLASS_2_CCB |
			 LDST_SRCDST_BYTE_CONTEXT);

	print_hex_dump_debug("aead givenc shdesc@" __stringify(__LINE__)": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc),
			     1);
}
EXPORT_SYMBOL(cnstr_shdsc_aead_givencap);

/**
 * cnstr_shdsc_tls_encap - tls encapsulation shared descriptor
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - one of OP_ALG_ALGSEL_AES ANDed
 *         with OP_ALG_AAI_CBC
 * @adata: pointer to authentication transform definitions.
 *         A split key is required for SEC Era < 6; the size of the split key
 *         is specified in this case. Valid algorithm values OP_ALG_ALGSEL_SHA1
 *         ANDed with OP_ALG_AAI_HMAC_PRECOMP.
 * @assoclen: associated data length
 * @ivsize: initialization vector size
 * @authsize: authentication data size
 * @blocksize: block cipher size
 * @era: SEC Era
 */
void cnstr_shdsc_tls_encap(u32 * const desc, struct alginfo *cdata,
			   struct alginfo *adata, unsigned int assoclen,
			   unsigned int ivsize, unsigned int authsize,
			   unsigned int blocksize, int era)
{
	u32 *key_jump_cmd, *zero_payload_jump_cmd;
	u32 genpad, idx_ld_datasz, idx_ld_pad, stidx;

	/*
	 * Compute the index (in bytes) for the LOAD with destination of
	 * Class 1 Data Size Register and for the LOAD that generates padding
	 */
	if (adata->key_inline) {
		idx_ld_datasz = DESC_TLS10_ENC_LEN + adata->keylen_pad +
				cdata->keylen - 4 * CAAM_CMD_SZ;
		idx_ld_pad = DESC_TLS10_ENC_LEN + adata->keylen_pad +
			     cdata->keylen - 2 * CAAM_CMD_SZ;
	} else {
		idx_ld_datasz = DESC_TLS10_ENC_LEN + 2 * CAAM_PTR_SZ -
				4 * CAAM_CMD_SZ;
		idx_ld_pad = DESC_TLS10_ENC_LEN + 2 * CAAM_PTR_SZ -
			     2 * CAAM_CMD_SZ;
	}

	stidx = 1 << HDR_START_IDX_SHIFT;
	init_sh_desc(desc, HDR_SHARE_SERIAL | stidx);

	/* skip key loading if they are loaded due to sharing */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);

	if (era < 6) {
		if (adata->key_inline)
			append_key_as_imm(desc, adata->key_virt,
					  adata->keylen_pad, adata->keylen,
					  CLASS_2 | KEY_DEST_MDHA_SPLIT |
					  KEY_ENC);
		else
			append_key(desc, adata->key_dma, adata->keylen,
				   CLASS_2 | KEY_DEST_MDHA_SPLIT | KEY_ENC);
	} else {
		append_proto_dkp(desc, adata);
	}

	if (cdata->key_inline)
		append_key_as_imm(desc, cdata->key_virt, cdata->keylen,
				  cdata->keylen, CLASS_1 | KEY_DEST_CLASS_REG);
	else
		append_key(desc, cdata->key_dma, cdata->keylen, CLASS_1 |
			   KEY_DEST_CLASS_REG);

	set_jump_tgt_here(desc, key_jump_cmd);

	/* class 2 operation */
	append_operation(desc, adata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_ENCRYPT);
	/* class 1 operation */
	append_operation(desc, cdata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_ENCRYPT);

	/* payloadlen = input data length - (assoclen + ivlen) */
	append_math_sub_imm_u32(desc, REG0, SEQINLEN, IMM, assoclen + ivsize);

	/* math1 = payloadlen + icvlen */
	append_math_add_imm_u32(desc, REG1, REG0, IMM, authsize);

	/* padlen = block_size - math1 % block_size */
	append_math_and_imm_u32(desc, REG3, REG1, IMM, blocksize - 1);
	append_math_sub_imm_u32(desc, REG2, IMM, REG3, blocksize);

	/* cryptlen = payloadlen + icvlen + padlen */
	append_math_add(desc, VARSEQOUTLEN, REG1, REG2, 4);

	/*
	 * update immediate data with the padding length value
	 * for the LOAD in the class 1 data size register.
	 */
	append_move(desc, MOVE_SRC_DESCBUF | MOVE_DEST_MATH2 |
			(idx_ld_datasz << MOVE_OFFSET_SHIFT) | 7);
	append_move(desc, MOVE_WAITCOMP | MOVE_SRC_MATH2 | MOVE_DEST_DESCBUF |
			(idx_ld_datasz << MOVE_OFFSET_SHIFT) | 8);

	/* overwrite PL field for the padding iNFO FIFO entry  */
	append_move(desc, MOVE_SRC_DESCBUF | MOVE_DEST_MATH2 |
			(idx_ld_pad << MOVE_OFFSET_SHIFT) | 7);
	append_move(desc, MOVE_WAITCOMP | MOVE_SRC_MATH2 | MOVE_DEST_DESCBUF |
			(idx_ld_pad << MOVE_OFFSET_SHIFT) | 8);

	/* store encrypted payload, icv and padding */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_MESSAGE_DATA | LDST_VLF);

	/* if payload length is zero, jump to zero-payload commands */
	append_math_add(desc, VARSEQINLEN, ZERO, REG0, 4);
	zero_payload_jump_cmd = append_jump(desc, JUMP_TEST_ALL |
					    JUMP_COND_MATH_Z);

	/* load iv in context1 */
	append_cmd(desc, CMD_SEQ_LOAD | LDST_SRCDST_WORD_CLASS_CTX |
		   LDST_CLASS_1_CCB | ivsize);

	/* read assoc for authentication */
	append_seq_fifo_load(desc, assoclen, FIFOLD_CLASS_CLASS2 |
			     FIFOLD_TYPE_MSG);
	/* insnoop payload */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_BOTH | FIFOLD_TYPE_MSG |
			     FIFOLD_TYPE_LAST2 | FIFOLDST_VLF);

	/* jump the zero-payload commands */
	append_jump(desc, JUMP_TEST_ALL | 3);

	/* zero-payload commands */
	set_jump_tgt_here(desc, zero_payload_jump_cmd);

	/* load iv in context1 */
	append_cmd(desc, CMD_SEQ_LOAD | LDST_SRCDST_WORD_CLASS_CTX |
		   LDST_CLASS_1_CCB | ivsize);

	/* assoc data is the only data for authentication */
	append_seq_fifo_load(desc, assoclen, FIFOLD_CLASS_CLASS2 |
			     FIFOLD_TYPE_MSG | FIFOLD_TYPE_LAST2);

	/* send icv to encryption */
	append_move(desc, MOVE_SRC_CLASS2CTX | MOVE_DEST_CLASS1INFIFO |
		    authsize);

	/* update class 1 data size register with padding length */
	append_load_imm_u32(desc, 0, LDST_CLASS_1_CCB |
			    LDST_SRCDST_WORD_DATASZ_REG | LDST_IMM);

	/* generate padding and send it to encryption */
	genpad = NFIFOENTRY_DEST_CLASS1 | NFIFOENTRY_LC1 | NFIFOENTRY_FC1 |
	      NFIFOENTRY_STYPE_PAD | NFIFOENTRY_DTYPE_MSG | NFIFOENTRY_PTYPE_N;
	append_load_imm_u32(desc, genpad, LDST_CLASS_IND_CCB |
			    LDST_SRCDST_WORD_INFO_FIFO | LDST_IMM);

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "tls enc shdesc@" __stringify(__LINE__) ": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, desc,
		       desc_bytes(desc), 1);
#endif
}
EXPORT_SYMBOL(cnstr_shdsc_tls_encap);

/**
 * cnstr_shdsc_tls_decap - tls decapsulation shared descriptor
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - one of OP_ALG_ALGSEL_AES ANDed
 *         with OP_ALG_AAI_CBC
 * @adata: pointer to authentication transform definitions.
 *         A split key is required for SEC Era < 6; the size of the split key
 *         is specified in this case. Valid algorithm values OP_ALG_ALGSEL_SHA1
 *         ANDed with OP_ALG_AAI_HMAC_PRECOMP.
 * @assoclen: associated data length
 * @ivsize: initialization vector size
 * @authsize: authentication data size
 * @blocksize: block cipher size
 * @era: SEC Era
 */
void cnstr_shdsc_tls_decap(u32 * const desc, struct alginfo *cdata,
			   struct alginfo *adata, unsigned int assoclen,
			   unsigned int ivsize, unsigned int authsize,
			   unsigned int blocksize, int era)
{
	u32 stidx, jumpback;
	u32 *key_jump_cmd, *zero_payload_jump_cmd, *skip_zero_jump_cmd;
	/*
	 * Pointer Size bool determines the size of address pointers.
	 * false - Pointers fit in one 32-bit word.
	 * true - Pointers fit in two 32-bit words.
	 */
	bool ps = (CAAM_PTR_SZ != CAAM_CMD_SZ);

	stidx = 1 << HDR_START_IDX_SHIFT;
	init_sh_desc(desc, HDR_SHARE_SERIAL | stidx);

	/* skip key loading if they are loaded due to sharing */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);

	if (era < 6)
		append_key(desc, adata->key_dma, adata->keylen, CLASS_2 |
			   KEY_DEST_MDHA_SPLIT | KEY_ENC);
	else
		append_proto_dkp(desc, adata);

	append_key(desc, cdata->key_dma, cdata->keylen, CLASS_1 |
		   KEY_DEST_CLASS_REG);

	set_jump_tgt_here(desc, key_jump_cmd);

	/* class 2 operation */
	append_operation(desc, adata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_DECRYPT | OP_ALG_ICV_ON);
	/* class 1 operation */
	append_operation(desc, cdata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_DECRYPT);

	/* VSIL = input data length - 2 * block_size */
	append_math_sub_imm_u32(desc, VARSEQINLEN, SEQINLEN, IMM, 2 *
				blocksize);

	/*
	 * payloadlen + icvlen + padlen = input data length - (assoclen +
	 * ivsize)
	 */
	append_math_sub_imm_u32(desc, REG3, SEQINLEN, IMM, assoclen + ivsize);

	/* skip data to the last but one cipher block */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_SKIP | LDST_VLF);

	/* load iv for the last cipher block */
	append_cmd(desc, CMD_SEQ_LOAD | LDST_SRCDST_WORD_CLASS_CTX |
		   LDST_CLASS_1_CCB | ivsize);

	/* read last cipher block */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 | FIFOLD_TYPE_MSG |
			     FIFOLD_TYPE_LAST1 | blocksize);

	/* move decrypted block into math0 and math1 */
	append_move(desc, MOVE_WAITCOMP | MOVE_SRC_OUTFIFO | MOVE_DEST_MATH0 |
		    blocksize);

	/* reset AES CHA */
	append_load_imm_u32(desc, CCTRL_RESET_CHA_AESA, LDST_CLASS_IND_CCB |
			    LDST_SRCDST_WORD_CHACTRL | LDST_IMM);

	/* rewind input sequence */
	append_seq_in_ptr_intlen(desc, 0, 65535, SQIN_RTO);

	/* key1 is in decryption form */
	append_operation(desc, cdata->algtype | OP_ALG_AAI_DK |
			 OP_ALG_AS_INITFINAL | OP_ALG_DECRYPT);

	/* load iv in context1 */
	append_cmd(desc, CMD_SEQ_LOAD | LDST_CLASS_1_CCB |
		   LDST_SRCDST_WORD_CLASS_CTX | ivsize);

	/* read sequence number */
	append_seq_fifo_load(desc, 8, FIFOLD_CLASS_CLASS2 | FIFOLD_TYPE_MSG);
	/* load Type, Version and Len fields in math0 */
	append_cmd(desc, CMD_SEQ_LOAD | LDST_CLASS_DECO |
		   LDST_SRCDST_WORD_DECO_MATH0 | (3 << LDST_OFFSET_SHIFT) | 5);

	/* compute (padlen - 1) */
	append_math_and_imm_u64(desc, REG1, REG1, IMM, 255);

	/* math2 = icvlen + (padlen - 1) + 1 */
	append_math_add_imm_u32(desc, REG2, REG1, IMM, authsize + 1);

	append_jump(desc, JUMP_TEST_ALL | JUMP_COND_CALM | 1);

	/* VSOL = payloadlen + icvlen + padlen */
	append_math_add(desc, VARSEQOUTLEN, ZERO, REG3, 4);

	if (caam_little_end)
		append_moveb(desc, MOVE_WAITCOMP |
			     MOVE_SRC_MATH0 | MOVE_DEST_MATH0 | 8);

	/* update Len field */
	append_math_sub(desc, REG0, REG0, REG2, 8);

	/* store decrypted payload, icv and padding */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_MESSAGE_DATA | LDST_VLF);

	/* VSIL = (payloadlen + icvlen + padlen) - (icvlen + padlen)*/
	append_math_sub(desc, VARSEQINLEN, REG3, REG2, 4);

	zero_payload_jump_cmd = append_jump(desc, JUMP_TEST_ALL |
					    JUMP_COND_MATH_Z);

	/* send Type, Version and Len(pre ICV) fields to authentication */
	append_move(desc, MOVE_WAITCOMP |
		    MOVE_SRC_MATH0 | MOVE_DEST_CLASS2INFIFO |
		    (3 << MOVE_OFFSET_SHIFT) | 5);

	/* outsnooping payload */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_BOTH |
			     FIFOLD_TYPE_MSG1OUT2 | FIFOLD_TYPE_LAST2 |
			     FIFOLDST_VLF);
	skip_zero_jump_cmd = append_jump(desc, JUMP_TEST_ALL | 2);

	set_jump_tgt_here(desc, zero_payload_jump_cmd);
	/* send Type, Version and Len(pre ICV) fields to authentication */
	append_move(desc, MOVE_WAITCOMP | MOVE_AUX_LS |
		    MOVE_SRC_MATH0 | MOVE_DEST_CLASS2INFIFO |
		    (3 << MOVE_OFFSET_SHIFT) | 5);

	set_jump_tgt_here(desc, skip_zero_jump_cmd);
	append_math_add(desc, VARSEQINLEN, ZERO, REG2, 4);

	/* load icvlen and padlen */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 | FIFOLD_TYPE_MSG |
			     FIFOLD_TYPE_LAST1 | FIFOLDST_VLF);

	/* VSIL = (payloadlen + icvlen + padlen) - icvlen + padlen */
	append_math_sub(desc, VARSEQINLEN, REG3, REG2, 4);

	/*
	 * Start a new input sequence using the SEQ OUT PTR command options,
	 * pointer and length used when the current output sequence was defined.
	 */
	if (ps) {
		/*
		 * Move the lower 32 bits of Shared Descriptor address, the
		 * SEQ OUT PTR command, Output Pointer (2 words) and
		 * Output Length into math registers.
		 */
		if (caam_little_end)
			append_move(desc, MOVE_WAITCOMP | MOVE_SRC_DESCBUF |
				    MOVE_DEST_MATH0 |
				    (55 * 4 << MOVE_OFFSET_SHIFT) | 20);
		else
			append_move(desc, MOVE_WAITCOMP | MOVE_SRC_DESCBUF |
				    MOVE_DEST_MATH0 |
				    (54 * 4 << MOVE_OFFSET_SHIFT) | 20);

		/* Transform SEQ OUT PTR command in SEQ IN PTR command */
		append_math_and_imm_u32(desc, REG0, REG0, IMM,
					~(CMD_SEQ_IN_PTR ^ CMD_SEQ_OUT_PTR));
		/* Append a JUMP command after the copied fields */
		jumpback = CMD_JUMP | (char)-9;
		append_load_imm_u32(desc, jumpback, LDST_CLASS_DECO | LDST_IMM |
				    LDST_SRCDST_WORD_DECO_MATH2 |
				    (4 << LDST_OFFSET_SHIFT));
		append_jump(desc, JUMP_TEST_ALL | JUMP_COND_CALM | 1);
		/* Move the updated fields back to the Job Descriptor */
		if (caam_little_end)
			append_move(desc, MOVE_WAITCOMP | MOVE_SRC_MATH0 |
				    MOVE_DEST_DESCBUF |
				    (55 * 4 << MOVE_OFFSET_SHIFT) | 24);
		else
			append_move(desc, MOVE_WAITCOMP | MOVE_SRC_MATH0 |
				    MOVE_DEST_DESCBUF |
				    (54 * 4 << MOVE_OFFSET_SHIFT) | 24);

		/*
		 * Read the new SEQ IN PTR command, Input Pointer, Input Length
		 * and then jump back to the next command from the
		 * Shared Descriptor.
		 */
		append_jump(desc, JUMP_TEST_ALL | JUMP_COND_CALM | 6);
	} else {
		/*
		 * Move the SEQ OUT PTR command, Output Pointer (1 word) and
		 * Output Length into math registers.
		 */
		if (caam_little_end)
			append_move(desc, MOVE_WAITCOMP | MOVE_SRC_DESCBUF |
				    MOVE_DEST_MATH0 |
				    (54 * 4 << MOVE_OFFSET_SHIFT) | 12);
		else
			append_move(desc, MOVE_WAITCOMP | MOVE_SRC_DESCBUF |
				    MOVE_DEST_MATH0 |
				    (53 * 4 << MOVE_OFFSET_SHIFT) | 12);

		/* Transform SEQ OUT PTR command in SEQ IN PTR command */
		append_math_and_imm_u64(desc, REG0, REG0, IMM,
					~(((u64)(CMD_SEQ_IN_PTR ^
						 CMD_SEQ_OUT_PTR)) << 32));
		/* Append a JUMP command after the copied fields */
		jumpback = CMD_JUMP | (char)-7;
		append_load_imm_u32(desc, jumpback, LDST_CLASS_DECO | LDST_IMM |
				    LDST_SRCDST_WORD_DECO_MATH1 |
				    (4 << LDST_OFFSET_SHIFT));
		append_jump(desc, JUMP_TEST_ALL | JUMP_COND_CALM | 1);
		/* Move the updated fields back to the Job Descriptor */
		if (caam_little_end)
			append_move(desc, MOVE_WAITCOMP | MOVE_SRC_MATH0 |
				    MOVE_DEST_DESCBUF |
				    (54 * 4 << MOVE_OFFSET_SHIFT) | 16);
		else
			append_move(desc, MOVE_WAITCOMP | MOVE_SRC_MATH0 |
				    MOVE_DEST_DESCBUF |
				    (53 * 4 << MOVE_OFFSET_SHIFT) | 16);

		/*
		 * Read the new SEQ IN PTR command, Input Pointer, Input Length
		 * and then jump back to the next command from the
		 * Shared Descriptor.
		 */
		 append_jump(desc, JUMP_TEST_ALL | JUMP_COND_CALM | 5);
	}

	/* skip payload */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_SKIP | FIFOLDST_VLF);
	/* check icv */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS2 | FIFOLD_TYPE_ICV |
			     FIFOLD_TYPE_LAST2 | authsize);

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "tls dec shdesc@" __stringify(__LINE__) ": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, desc,
		       desc_bytes(desc), 1);
#endif
}
EXPORT_SYMBOL(cnstr_shdsc_tls_decap);

/**
 * cnstr_shdsc_gcm_encap - gcm encapsulation shared descriptor
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - OP_ALG_ALGSEL_AES ANDed with OP_ALG_AAI_GCM.
 * @ivsize: initialization vector size
 * @icvsize: integrity check value (ICV) size (truncated or full)
 * @is_qi: true when called from caam/qi
 */
void cnstr_shdsc_gcm_encap(u32 * const desc, struct alginfo *cdata,
			   unsigned int ivsize, unsigned int icvsize,
			   const bool is_qi)
{
	u32 *key_jump_cmd, *zero_payload_jump_cmd, *zero_assoc_jump_cmd1,
	    *zero_assoc_jump_cmd2;

	init_sh_desc(desc, HDR_SHARE_SERIAL);

	/* skip key loading if they are loaded due to sharing */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);
	if (cdata->key_inline)
		append_key_as_imm(desc, cdata->key_virt, cdata->keylen,
				  cdata->keylen, CLASS_1 | KEY_DEST_CLASS_REG);
	else
		append_key(desc, cdata->key_dma, cdata->keylen, CLASS_1 |
			   KEY_DEST_CLASS_REG);
	set_jump_tgt_here(desc, key_jump_cmd);

	/* class 1 operation */
	append_operation(desc, cdata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_ENCRYPT);

	if (is_qi) {
		u32 *wait_load_cmd;

		/* REG3 = assoclen */
		append_seq_load(desc, 4, LDST_CLASS_DECO |
				LDST_SRCDST_WORD_DECO_MATH3 |
				(4 << LDST_OFFSET_SHIFT));

		wait_load_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
					    JUMP_COND_CALM | JUMP_COND_NCP |
					    JUMP_COND_NOP | JUMP_COND_NIP |
					    JUMP_COND_NIFP);
		set_jump_tgt_here(desc, wait_load_cmd);

		append_math_sub_imm_u32(desc, VARSEQOUTLEN, SEQINLEN, IMM,
					ivsize);
	} else {
		append_math_sub(desc, VARSEQOUTLEN, SEQINLEN, REG0,
				CAAM_CMD_SZ);
	}

	/* if assoclen + cryptlen is ZERO, skip to ICV write */
	zero_assoc_jump_cmd2 = append_jump(desc, JUMP_TEST_ALL |
						 JUMP_COND_MATH_Z);

	if (is_qi)
		append_seq_fifo_load(desc, ivsize, FIFOLD_CLASS_CLASS1 |
				     FIFOLD_TYPE_IV | FIFOLD_TYPE_FLUSH1);

	/* if assoclen is ZERO, skip reading the assoc data */
	append_math_add(desc, VARSEQINLEN, ZERO, REG3, CAAM_CMD_SZ);
	zero_assoc_jump_cmd1 = append_jump(desc, JUMP_TEST_ALL |
					   JUMP_COND_MATH_Z);

	append_math_add(desc, VARSEQOUTLEN, ZERO, REG3, CAAM_CMD_SZ);

	/* skip assoc data */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_SKIP | FIFOLDST_VLF);

	/* cryptlen = seqinlen - assoclen */
	append_math_sub(desc, VARSEQOUTLEN, SEQINLEN, REG3, CAAM_CMD_SZ);

	/* if cryptlen is ZERO jump to zero-payload commands */
	zero_payload_jump_cmd = append_jump(desc, JUMP_TEST_ALL |
					    JUMP_COND_MATH_Z);

	/* read assoc data */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 | FIFOLDST_VLF |
			     FIFOLD_TYPE_AAD | FIFOLD_TYPE_FLUSH1);
	set_jump_tgt_here(desc, zero_assoc_jump_cmd1);

	append_math_sub(desc, VARSEQINLEN, SEQINLEN, REG0, CAAM_CMD_SZ);

	/* write encrypted data */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_MESSAGE_DATA | FIFOLDST_VLF);

	/* read payload data */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 | FIFOLDST_VLF |
			     FIFOLD_TYPE_MSG | FIFOLD_TYPE_LAST1);

	/* jump to ICV writing */
	if (is_qi)
		append_jump(desc, JUMP_TEST_ALL | 4);
	else
		append_jump(desc, JUMP_TEST_ALL | 2);

	/* zero-payload commands */
	set_jump_tgt_here(desc, zero_payload_jump_cmd);

	/* read assoc data */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 | FIFOLDST_VLF |
			     FIFOLD_TYPE_AAD | FIFOLD_TYPE_LAST1);
	if (is_qi)
		/* jump to ICV writing */
		append_jump(desc, JUMP_TEST_ALL | 2);

	/* There is no input data */
	set_jump_tgt_here(desc, zero_assoc_jump_cmd2);

	if (is_qi)
		append_seq_fifo_load(desc, ivsize, FIFOLD_CLASS_CLASS1 |
				     FIFOLD_TYPE_IV | FIFOLD_TYPE_FLUSH1 |
				     FIFOLD_TYPE_LAST1);

	/* write ICV */
	append_seq_store(desc, icvsize, LDST_CLASS_1_CCB |
			 LDST_SRCDST_BYTE_CONTEXT);

	print_hex_dump_debug("gcm enc shdesc@" __stringify(__LINE__)": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc),
			     1);
}
EXPORT_SYMBOL(cnstr_shdsc_gcm_encap);

/**
 * cnstr_shdsc_gcm_decap - gcm decapsulation shared descriptor
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - OP_ALG_ALGSEL_AES ANDed with OP_ALG_AAI_GCM.
 * @ivsize: initialization vector size
 * @icvsize: integrity check value (ICV) size (truncated or full)
 * @is_qi: true when called from caam/qi
 */
void cnstr_shdsc_gcm_decap(u32 * const desc, struct alginfo *cdata,
			   unsigned int ivsize, unsigned int icvsize,
			   const bool is_qi)
{
	u32 *key_jump_cmd, *zero_payload_jump_cmd, *zero_assoc_jump_cmd1;

	init_sh_desc(desc, HDR_SHARE_SERIAL);

	/* skip key loading if they are loaded due to sharing */
	key_jump_cmd = append_jump(desc, JUMP_JSL |
				   JUMP_TEST_ALL | JUMP_COND_SHRD);
	if (cdata->key_inline)
		append_key_as_imm(desc, cdata->key_virt, cdata->keylen,
				  cdata->keylen, CLASS_1 | KEY_DEST_CLASS_REG);
	else
		append_key(desc, cdata->key_dma, cdata->keylen, CLASS_1 |
			   KEY_DEST_CLASS_REG);
	set_jump_tgt_here(desc, key_jump_cmd);

	/* class 1 operation */
	append_operation(desc, cdata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_DECRYPT | OP_ALG_ICV_ON);

	if (is_qi) {
		u32 *wait_load_cmd;

		/* REG3 = assoclen */
		append_seq_load(desc, 4, LDST_CLASS_DECO |
				LDST_SRCDST_WORD_DECO_MATH3 |
				(4 << LDST_OFFSET_SHIFT));

		wait_load_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
					    JUMP_COND_CALM | JUMP_COND_NCP |
					    JUMP_COND_NOP | JUMP_COND_NIP |
					    JUMP_COND_NIFP);
		set_jump_tgt_here(desc, wait_load_cmd);

		append_seq_fifo_load(desc, ivsize, FIFOLD_CLASS_CLASS1 |
				     FIFOLD_TYPE_IV | FIFOLD_TYPE_FLUSH1);
	}

	/* if assoclen is ZERO, skip reading the assoc data */
	append_math_add(desc, VARSEQINLEN, ZERO, REG3, CAAM_CMD_SZ);
	zero_assoc_jump_cmd1 = append_jump(desc, JUMP_TEST_ALL |
						 JUMP_COND_MATH_Z);

	append_math_add(desc, VARSEQOUTLEN, ZERO, REG3, CAAM_CMD_SZ);

	/* skip assoc data */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_SKIP | FIFOLDST_VLF);

	/* read assoc data */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 | FIFOLDST_VLF |
			     FIFOLD_TYPE_AAD | FIFOLD_TYPE_FLUSH1);

	set_jump_tgt_here(desc, zero_assoc_jump_cmd1);

	/* cryptlen = seqoutlen - assoclen */
	append_math_sub(desc, VARSEQINLEN, SEQOUTLEN, REG0, CAAM_CMD_SZ);

	/* jump to zero-payload command if cryptlen is zero */
	zero_payload_jump_cmd = append_jump(desc, JUMP_TEST_ALL |
					    JUMP_COND_MATH_Z);

	append_math_sub(desc, VARSEQOUTLEN, SEQOUTLEN, REG0, CAAM_CMD_SZ);

	/* store encrypted data */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_MESSAGE_DATA | FIFOLDST_VLF);

	/* read payload data */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 | FIFOLDST_VLF |
			     FIFOLD_TYPE_MSG | FIFOLD_TYPE_FLUSH1);

	/* zero-payload command */
	set_jump_tgt_here(desc, zero_payload_jump_cmd);

	/* read ICV */
	append_seq_fifo_load(desc, icvsize, FIFOLD_CLASS_CLASS1 |
			     FIFOLD_TYPE_ICV | FIFOLD_TYPE_LAST1);

	print_hex_dump_debug("gcm dec shdesc@" __stringify(__LINE__)": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc),
			     1);
}
EXPORT_SYMBOL(cnstr_shdsc_gcm_decap);

/**
 * cnstr_shdsc_rfc4106_encap - IPSec ESP gcm encapsulation shared descriptor
 *                             (non-protocol).
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - OP_ALG_ALGSEL_AES ANDed with OP_ALG_AAI_GCM.
 * @ivsize: initialization vector size
 * @icvsize: integrity check value (ICV) size (truncated or full)
 * @is_qi: true when called from caam/qi
 *
 * Input sequence: AAD | PTXT
 * Output sequence: AAD | CTXT | ICV
 * AAD length (assoclen), which includes the IV length, is available in Math3.
 */
void cnstr_shdsc_rfc4106_encap(u32 * const desc, struct alginfo *cdata,
			       unsigned int ivsize, unsigned int icvsize,
			       const bool is_qi)
{
	u32 *key_jump_cmd, *zero_cryptlen_jump_cmd, *skip_instructions;
	init_sh_desc(desc, HDR_SHARE_SERIAL);

	/* Skip key loading if it is loaded due to sharing */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);
	if (cdata->key_inline)
		append_key_as_imm(desc, cdata->key_virt, cdata->keylen,
				  cdata->keylen, CLASS_1 | KEY_DEST_CLASS_REG);
	else
		append_key(desc, cdata->key_dma, cdata->keylen, CLASS_1 |
			   KEY_DEST_CLASS_REG);
	set_jump_tgt_here(desc, key_jump_cmd);

	/* Class 1 operation */
	append_operation(desc, cdata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_ENCRYPT);

	if (is_qi) {
		u32 *wait_load_cmd;

		/* REG3 = assoclen */
		append_seq_load(desc, 4, LDST_CLASS_DECO |
				LDST_SRCDST_WORD_DECO_MATH3 |
				(4 << LDST_OFFSET_SHIFT));

		wait_load_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
					    JUMP_COND_CALM | JUMP_COND_NCP |
					    JUMP_COND_NOP | JUMP_COND_NIP |
					    JUMP_COND_NIFP);
		set_jump_tgt_here(desc, wait_load_cmd);

		/* Read salt and IV */
		append_fifo_load_as_imm(desc, (void *)(cdata->key_virt +
					cdata->keylen), 4, FIFOLD_CLASS_CLASS1 |
					FIFOLD_TYPE_IV);
		append_seq_fifo_load(desc, ivsize, FIFOLD_CLASS_CLASS1 |
				     FIFOLD_TYPE_IV | FIFOLD_TYPE_FLUSH1);
	}

	append_math_sub_imm_u32(desc, VARSEQINLEN, REG3, IMM, ivsize);
	append_math_add(desc, VARSEQOUTLEN, ZERO, REG3, CAAM_CMD_SZ);

	/* Skip AAD */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_SKIP | FIFOLDST_VLF);

	/* Read cryptlen and set this value into VARSEQOUTLEN */
	append_math_sub(desc, VARSEQOUTLEN, SEQINLEN, REG3, CAAM_CMD_SZ);

	/* If cryptlen is ZERO jump to AAD command */
	zero_cryptlen_jump_cmd = append_jump(desc, JUMP_TEST_ALL |
					    JUMP_COND_MATH_Z);

	/* Read AAD data */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 | FIFOLDST_VLF |
			     FIFOLD_TYPE_AAD | FIFOLD_TYPE_FLUSH1);

	/* Workaround for erratum A-005473 (simultaneous SEQ FIFO skips) */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_MESSAGE_DATA);

	/* Skip IV */
	append_seq_fifo_load(desc, ivsize, FIFOLD_CLASS_SKIP);
	append_math_add(desc, VARSEQINLEN, VARSEQOUTLEN, REG0, CAAM_CMD_SZ);

	/* Write encrypted data */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_MESSAGE_DATA | FIFOLDST_VLF);

	/* Read payload data */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 | FIFOLDST_VLF |
			     FIFOLD_TYPE_MSG | FIFOLD_TYPE_LAST1);

	/* Jump instructions to avoid double reading of AAD */
	skip_instructions = append_jump(desc, JUMP_TEST_ALL);

	/* There is no input data, cryptlen = 0 */
	set_jump_tgt_here(desc, zero_cryptlen_jump_cmd);

	/* Read AAD */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 | FIFOLDST_VLF |
			     FIFOLD_TYPE_AAD | FIFOLD_TYPE_LAST1);

	set_jump_tgt_here(desc, skip_instructions);

	/* Write ICV */
	append_seq_store(desc, icvsize, LDST_CLASS_1_CCB |
			 LDST_SRCDST_BYTE_CONTEXT);

	print_hex_dump_debug("rfc4106 enc shdesc@" __stringify(__LINE__)": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc),
			     1);
}
EXPORT_SYMBOL(cnstr_shdsc_rfc4106_encap);

/**
 * cnstr_shdsc_rfc4106_decap - IPSec ESP gcm decapsulation shared descriptor
 *                             (non-protocol).
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - OP_ALG_ALGSEL_AES ANDed with OP_ALG_AAI_GCM.
 * @ivsize: initialization vector size
 * @icvsize: integrity check value (ICV) size (truncated or full)
 * @is_qi: true when called from caam/qi
 */
void cnstr_shdsc_rfc4106_decap(u32 * const desc, struct alginfo *cdata,
			       unsigned int ivsize, unsigned int icvsize,
			       const bool is_qi)
{
	u32 *key_jump_cmd;

	init_sh_desc(desc, HDR_SHARE_SERIAL);

	/* Skip key loading if it is loaded due to sharing */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);
	if (cdata->key_inline)
		append_key_as_imm(desc, cdata->key_virt, cdata->keylen,
				  cdata->keylen, CLASS_1 |
				  KEY_DEST_CLASS_REG);
	else
		append_key(desc, cdata->key_dma, cdata->keylen, CLASS_1 |
			   KEY_DEST_CLASS_REG);
	set_jump_tgt_here(desc, key_jump_cmd);

	/* Class 1 operation */
	append_operation(desc, cdata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_DECRYPT | OP_ALG_ICV_ON);

	if (is_qi) {
		u32 *wait_load_cmd;

		/* REG3 = assoclen */
		append_seq_load(desc, 4, LDST_CLASS_DECO |
				LDST_SRCDST_WORD_DECO_MATH3 |
				(4 << LDST_OFFSET_SHIFT));

		wait_load_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
					    JUMP_COND_CALM | JUMP_COND_NCP |
					    JUMP_COND_NOP | JUMP_COND_NIP |
					    JUMP_COND_NIFP);
		set_jump_tgt_here(desc, wait_load_cmd);

		/* Read salt and IV */
		append_fifo_load_as_imm(desc, (void *)(cdata->key_virt +
					cdata->keylen), 4, FIFOLD_CLASS_CLASS1 |
					FIFOLD_TYPE_IV);
		append_seq_fifo_load(desc, ivsize, FIFOLD_CLASS_CLASS1 |
				     FIFOLD_TYPE_IV | FIFOLD_TYPE_FLUSH1);
	}

	append_math_sub_imm_u32(desc, VARSEQINLEN, REG3, IMM, ivsize);
	append_math_add(desc, VARSEQOUTLEN, ZERO, REG3, CAAM_CMD_SZ);

	/* Read assoc data */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 | FIFOLDST_VLF |
			     FIFOLD_TYPE_AAD | FIFOLD_TYPE_FLUSH1);

	/* Skip IV */
	append_seq_fifo_load(desc, ivsize, FIFOLD_CLASS_SKIP);

	/* Will read cryptlen bytes */
	append_math_sub(desc, VARSEQINLEN, SEQOUTLEN, REG3, CAAM_CMD_SZ);

	/* Workaround for erratum A-005473 (simultaneous SEQ FIFO skips) */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 | FIFOLD_TYPE_MSG);

	/* Skip assoc data */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_SKIP | FIFOLDST_VLF);

	/* Will write cryptlen bytes */
	append_math_sub(desc, VARSEQOUTLEN, SEQOUTLEN, REG0, CAAM_CMD_SZ);

	/* Store payload data */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_MESSAGE_DATA | FIFOLDST_VLF);

	/* Read encrypted data */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 | FIFOLDST_VLF |
			     FIFOLD_TYPE_MSG | FIFOLD_TYPE_FLUSH1);

	/* Read ICV */
	append_seq_fifo_load(desc, icvsize, FIFOLD_CLASS_CLASS1 |
			     FIFOLD_TYPE_ICV | FIFOLD_TYPE_LAST1);

	print_hex_dump_debug("rfc4106 dec shdesc@" __stringify(__LINE__)": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc),
			     1);
}
EXPORT_SYMBOL(cnstr_shdsc_rfc4106_decap);

/**
 * cnstr_shdsc_rfc4543_encap - IPSec ESP gmac encapsulation shared descriptor
 *                             (non-protocol).
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - OP_ALG_ALGSEL_AES ANDed with OP_ALG_AAI_GCM.
 * @ivsize: initialization vector size
 * @icvsize: integrity check value (ICV) size (truncated or full)
 * @is_qi: true when called from caam/qi
 */
void cnstr_shdsc_rfc4543_encap(u32 * const desc, struct alginfo *cdata,
			       unsigned int ivsize, unsigned int icvsize,
			       const bool is_qi)
{
	u32 *key_jump_cmd, *read_move_cmd, *write_move_cmd;

	init_sh_desc(desc, HDR_SHARE_SERIAL);

	/* Skip key loading if it is loaded due to sharing */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);
	if (cdata->key_inline)
		append_key_as_imm(desc, cdata->key_virt, cdata->keylen,
				  cdata->keylen, CLASS_1 | KEY_DEST_CLASS_REG);
	else
		append_key(desc, cdata->key_dma, cdata->keylen, CLASS_1 |
			   KEY_DEST_CLASS_REG);
	set_jump_tgt_here(desc, key_jump_cmd);

	/* Class 1 operation */
	append_operation(desc, cdata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_ENCRYPT);

	if (is_qi) {
		/* assoclen is not needed, skip it */
		append_seq_fifo_load(desc, 4, FIFOLD_CLASS_SKIP);

		/* Read salt and IV */
		append_fifo_load_as_imm(desc, (void *)(cdata->key_virt +
					cdata->keylen), 4, FIFOLD_CLASS_CLASS1 |
					FIFOLD_TYPE_IV);
		append_seq_fifo_load(desc, ivsize, FIFOLD_CLASS_CLASS1 |
				     FIFOLD_TYPE_IV | FIFOLD_TYPE_FLUSH1);
	}

	/* assoclen + cryptlen = seqinlen */
	append_math_sub(desc, REG3, SEQINLEN, REG0, CAAM_CMD_SZ);

	/*
	 * MOVE_LEN opcode is not available in all SEC HW revisions,
	 * thus need to do some magic, i.e. self-patch the descriptor
	 * buffer.
	 */
	read_move_cmd = append_move(desc, MOVE_SRC_DESCBUF | MOVE_DEST_MATH3 |
				    (0x6 << MOVE_LEN_SHIFT));
	write_move_cmd = append_move(desc, MOVE_SRC_MATH3 | MOVE_DEST_DESCBUF |
				     (0x8 << MOVE_LEN_SHIFT) | MOVE_WAITCOMP);

	/* Will read assoclen + cryptlen bytes */
	append_math_sub(desc, VARSEQINLEN, SEQINLEN, REG0, CAAM_CMD_SZ);

	/* Will write assoclen + cryptlen bytes */
	append_math_sub(desc, VARSEQOUTLEN, SEQINLEN, REG0, CAAM_CMD_SZ);

	/* Read and write assoclen + cryptlen bytes */
	aead_append_src_dst(desc, FIFOLD_TYPE_AAD);

	set_move_tgt_here(desc, read_move_cmd);
	set_move_tgt_here(desc, write_move_cmd);
	append_cmd(desc, CMD_LOAD | DISABLE_AUTO_INFO_FIFO);
	/* Move payload data to OFIFO */
	append_move(desc, MOVE_SRC_INFIFO_CL | MOVE_DEST_OUTFIFO);

	/* Write ICV */
	append_seq_store(desc, icvsize, LDST_CLASS_1_CCB |
			 LDST_SRCDST_BYTE_CONTEXT);

	print_hex_dump_debug("rfc4543 enc shdesc@" __stringify(__LINE__)": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc),
			     1);
}
EXPORT_SYMBOL(cnstr_shdsc_rfc4543_encap);

/**
 * cnstr_shdsc_rfc4543_decap - IPSec ESP gmac decapsulation shared descriptor
 *                             (non-protocol).
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - OP_ALG_ALGSEL_AES ANDed with OP_ALG_AAI_GCM.
 * @ivsize: initialization vector size
 * @icvsize: integrity check value (ICV) size (truncated or full)
 * @is_qi: true when called from caam/qi
 */
void cnstr_shdsc_rfc4543_decap(u32 * const desc, struct alginfo *cdata,
			       unsigned int ivsize, unsigned int icvsize,
			       const bool is_qi)
{
	u32 *key_jump_cmd, *read_move_cmd, *write_move_cmd;

	init_sh_desc(desc, HDR_SHARE_SERIAL);

	/* Skip key loading if it is loaded due to sharing */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);
	if (cdata->key_inline)
		append_key_as_imm(desc, cdata->key_virt, cdata->keylen,
				  cdata->keylen, CLASS_1 | KEY_DEST_CLASS_REG);
	else
		append_key(desc, cdata->key_dma, cdata->keylen, CLASS_1 |
			   KEY_DEST_CLASS_REG);
	set_jump_tgt_here(desc, key_jump_cmd);

	/* Class 1 operation */
	append_operation(desc, cdata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_DECRYPT | OP_ALG_ICV_ON);

	if (is_qi) {
		/* assoclen is not needed, skip it */
		append_seq_fifo_load(desc, 4, FIFOLD_CLASS_SKIP);

		/* Read salt and IV */
		append_fifo_load_as_imm(desc, (void *)(cdata->key_virt +
					cdata->keylen), 4, FIFOLD_CLASS_CLASS1 |
					FIFOLD_TYPE_IV);
		append_seq_fifo_load(desc, ivsize, FIFOLD_CLASS_CLASS1 |
				     FIFOLD_TYPE_IV | FIFOLD_TYPE_FLUSH1);
	}

	/* assoclen + cryptlen = seqoutlen */
	append_math_sub(desc, REG3, SEQOUTLEN, REG0, CAAM_CMD_SZ);

	/*
	 * MOVE_LEN opcode is not available in all SEC HW revisions,
	 * thus need to do some magic, i.e. self-patch the descriptor
	 * buffer.
	 */
	read_move_cmd = append_move(desc, MOVE_SRC_DESCBUF | MOVE_DEST_MATH3 |
				    (0x6 << MOVE_LEN_SHIFT));
	write_move_cmd = append_move(desc, MOVE_SRC_MATH3 | MOVE_DEST_DESCBUF |
				     (0x8 << MOVE_LEN_SHIFT) | MOVE_WAITCOMP);

	/* Will read assoclen + cryptlen bytes */
	append_math_sub(desc, VARSEQINLEN, SEQOUTLEN, REG0, CAAM_CMD_SZ);

	/* Will write assoclen + cryptlen bytes */
	append_math_sub(desc, VARSEQOUTLEN, SEQOUTLEN, REG0, CAAM_CMD_SZ);

	/* Store payload data */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_MESSAGE_DATA | FIFOLDST_VLF);

	/* In-snoop assoclen + cryptlen data */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_BOTH | FIFOLDST_VLF |
			     FIFOLD_TYPE_AAD | FIFOLD_TYPE_LAST2FLUSH1);

	set_move_tgt_here(desc, read_move_cmd);
	set_move_tgt_here(desc, write_move_cmd);
	append_cmd(desc, CMD_LOAD | DISABLE_AUTO_INFO_FIFO);
	/* Move payload data to OFIFO */
	append_move(desc, MOVE_SRC_INFIFO_CL | MOVE_DEST_OUTFIFO);
	append_cmd(desc, CMD_LOAD | ENABLE_AUTO_INFO_FIFO);

	/* Read ICV */
	append_seq_fifo_load(desc, icvsize, FIFOLD_CLASS_CLASS1 |
			     FIFOLD_TYPE_ICV | FIFOLD_TYPE_LAST1);

	print_hex_dump_debug("rfc4543 dec shdesc@" __stringify(__LINE__)": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc),
			     1);
}
EXPORT_SYMBOL(cnstr_shdsc_rfc4543_decap);

/**
 * cnstr_shdsc_chachapoly - Chacha20 + Poly1305 generic AEAD (rfc7539) and
 *                          IPsec ESP (rfc7634, a.k.a. rfc7539esp) shared
 *                          descriptor (non-protocol).
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - OP_ALG_ALGSEL_CHACHA20 ANDed with
 *         OP_ALG_AAI_AEAD.
 * @adata: pointer to authentication transform definitions
 *         Valid algorithm values - OP_ALG_ALGSEL_POLY1305 ANDed with
 *         OP_ALG_AAI_AEAD.
 * @ivsize: initialization vector size
 * @icvsize: integrity check value (ICV) size (truncated or full)
 * @encap: true if encapsulation, false if decapsulation
 * @is_qi: true when called from caam/qi
 */
void cnstr_shdsc_chachapoly(u32 * const desc, struct alginfo *cdata,
			    struct alginfo *adata, unsigned int ivsize,
			    unsigned int icvsize, const bool encap,
			    const bool is_qi)
{
	u32 *key_jump_cmd, *wait_cmd;
	u32 nfifo;
	const bool is_ipsec = (ivsize != CHACHAPOLY_IV_SIZE);

	/* Note: Context registers are saved. */
	init_sh_desc(desc, HDR_SHARE_SERIAL | HDR_SAVECTX);

	/* skip key loading if they are loaded due to sharing */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);

	append_key_as_imm(desc, cdata->key_virt, cdata->keylen, cdata->keylen,
			  CLASS_1 | KEY_DEST_CLASS_REG);

	/* For IPsec load the salt from keymat in the context register */
	if (is_ipsec)
		append_load_as_imm(desc, cdata->key_virt + cdata->keylen, 4,
				   LDST_CLASS_1_CCB | LDST_SRCDST_BYTE_CONTEXT |
				   4 << LDST_OFFSET_SHIFT);

	set_jump_tgt_here(desc, key_jump_cmd);

	/* Class 2 and 1 operations: Poly & ChaCha */
	if (encap) {
		append_operation(desc, adata->algtype | OP_ALG_AS_INITFINAL |
				 OP_ALG_ENCRYPT);
		append_operation(desc, cdata->algtype | OP_ALG_AS_INITFINAL |
				 OP_ALG_ENCRYPT);
	} else {
		append_operation(desc, adata->algtype | OP_ALG_AS_INITFINAL |
				 OP_ALG_DECRYPT | OP_ALG_ICV_ON);
		append_operation(desc, cdata->algtype | OP_ALG_AS_INITFINAL |
				 OP_ALG_DECRYPT);
	}

	if (is_qi) {
		u32 *wait_load_cmd;
		u32 ctx1_iv_off = is_ipsec ? 8 : 4;

		/* REG3 = assoclen */
		append_seq_load(desc, 4, LDST_CLASS_DECO |
				LDST_SRCDST_WORD_DECO_MATH3 |
				4 << LDST_OFFSET_SHIFT);

		wait_load_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
					    JUMP_COND_CALM | JUMP_COND_NCP |
					    JUMP_COND_NOP | JUMP_COND_NIP |
					    JUMP_COND_NIFP);
		set_jump_tgt_here(desc, wait_load_cmd);

		append_seq_load(desc, ivsize, LDST_CLASS_1_CCB |
				LDST_SRCDST_BYTE_CONTEXT |
				ctx1_iv_off << LDST_OFFSET_SHIFT);
	}

	/*
	 * MAGIC with NFIFO
	 * Read associated data from the input and send them to class1 and
	 * class2 alignment blocks. From class1 send data to output fifo and
	 * then write it to memory since we don't need to encrypt AD.
	 */
	nfifo = NFIFOENTRY_DEST_BOTH | NFIFOENTRY_FC1 | NFIFOENTRY_FC2 |
		NFIFOENTRY_DTYPE_POLY | NFIFOENTRY_BND;
	append_load_imm_u32(desc, nfifo, LDST_CLASS_IND_CCB |
			    LDST_SRCDST_WORD_INFO_FIFO_SM | LDLEN_MATH3);

	append_math_add(desc, VARSEQINLEN, ZERO, REG3, CAAM_CMD_SZ);
	append_math_add(desc, VARSEQOUTLEN, ZERO, REG3, CAAM_CMD_SZ);
	append_seq_fifo_load(desc, 0, FIFOLD_TYPE_NOINFOFIFO |
			     FIFOLD_CLASS_CLASS1 | LDST_VLF);
	append_move_len(desc, MOVE_AUX_LS | MOVE_SRC_AUX_ABLK |
			MOVE_DEST_OUTFIFO | MOVELEN_MRSEL_MATH3);
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_MESSAGE_DATA | LDST_VLF);

	/* IPsec - copy IV at the output */
	if (is_ipsec)
		append_seq_fifo_store(desc, ivsize, FIFOST_TYPE_METADATA |
				      0x2 << 25);

	wait_cmd = append_jump(desc, JUMP_JSL | JUMP_TYPE_LOCAL |
			       JUMP_COND_NOP | JUMP_TEST_ALL);
	set_jump_tgt_here(desc, wait_cmd);

	if (encap) {
		/* Read and write cryptlen bytes */
		append_math_add(desc, VARSEQINLEN, SEQINLEN, REG0, CAAM_CMD_SZ);
		append_math_add(desc, VARSEQOUTLEN, SEQINLEN, REG0,
				CAAM_CMD_SZ);
		aead_append_src_dst(desc, FIFOLD_TYPE_MSG1OUT2);

		/* Write ICV */
		append_seq_store(desc, icvsize, LDST_CLASS_2_CCB |
				 LDST_SRCDST_BYTE_CONTEXT);
	} else {
		/* Read and write cryptlen bytes */
		append_math_add(desc, VARSEQINLEN, SEQOUTLEN, REG0,
				CAAM_CMD_SZ);
		append_math_add(desc, VARSEQOUTLEN, SEQOUTLEN, REG0,
				CAAM_CMD_SZ);
		aead_append_src_dst(desc, FIFOLD_TYPE_MSG);

		/* Load ICV for verification */
		append_seq_fifo_load(desc, icvsize, FIFOLD_CLASS_CLASS2 |
				     FIFOLD_TYPE_LAST2 | FIFOLD_TYPE_ICV);
	}

	print_hex_dump_debug("chachapoly shdesc@" __stringify(__LINE__)": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc),
			     1);
}
EXPORT_SYMBOL(cnstr_shdsc_chachapoly);

/* For skcipher encrypt and decrypt, read from req->src and write to req->dst */
static inline void skcipher_append_src_dst(u32 *desc)
{
	append_math_add(desc, VARSEQOUTLEN, SEQINLEN, REG0, CAAM_CMD_SZ);
	append_math_add(desc, VARSEQINLEN, SEQINLEN, REG0, CAAM_CMD_SZ);
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 |
			     KEY_VLF | FIFOLD_TYPE_MSG | FIFOLD_TYPE_LAST1);
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_MESSAGE_DATA | KEY_VLF);
}

/**
 * cnstr_shdsc_skcipher_encap - skcipher encapsulation shared descriptor
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - one of OP_ALG_ALGSEL_{AES, DES, 3DES} ANDed
 *         with OP_ALG_AAI_CBC or OP_ALG_AAI_CTR_MOD128
 *                                - OP_ALG_ALGSEL_CHACHA20
 * @ivsize: initialization vector size
 * @is_rfc3686: true when ctr(aes) is wrapped by rfc3686 template
 * @ctx1_iv_off: IV offset in CONTEXT1 register
 */
void cnstr_shdsc_skcipher_encap(u32 * const desc, struct alginfo *cdata,
				unsigned int ivsize, const bool is_rfc3686,
				const u32 ctx1_iv_off)
{
	u32 *key_jump_cmd;
	u32 options = cdata->algtype | OP_ALG_AS_INIT | OP_ALG_ENCRYPT;
	bool is_chacha20 = ((cdata->algtype & OP_ALG_ALGSEL_MASK) ==
			    OP_ALG_ALGSEL_CHACHA20);

	init_sh_desc(desc, HDR_SHARE_SERIAL | HDR_SAVECTX);
	/* Skip if already shared */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);

	/* Load class1 key only */
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_FSL_CAAM_TK_API) &&
	    cdata->key_cmd_opt)
		/*
		 * Black keys can be loaded using only a KEY command
		 * with ENC=1 and the proper setting of the EKT bit.
		 */
		append_key_as_imm(desc, cdata->key_virt, cdata->keylen,
				  cdata->key_real_len, CLASS_1 |
				  KEY_DEST_CLASS_REG | cdata->key_cmd_opt);
	else
		append_key_as_imm(desc, cdata->key_virt, cdata->keylen,
				  cdata->keylen, CLASS_1 | KEY_DEST_CLASS_REG);

	/* Load nonce into CONTEXT1 reg */
	if (is_rfc3686) {
		const u8 *nonce = cdata->key_virt + cdata->keylen;

		append_load_as_imm(desc, nonce, CTR_RFC3686_NONCE_SIZE,
				   LDST_CLASS_IND_CCB |
				   LDST_SRCDST_BYTE_OUTFIFO | LDST_IMM);
		append_move(desc, MOVE_WAITCOMP | MOVE_SRC_OUTFIFO |
			    MOVE_DEST_CLASS1CTX | (16 << MOVE_OFFSET_SHIFT) |
			    (CTR_RFC3686_NONCE_SIZE << MOVE_LEN_SHIFT));
	}

	set_jump_tgt_here(desc, key_jump_cmd);

	/* Load IV, if there is one */
	if (ivsize)
		append_seq_load(desc, ivsize, LDST_SRCDST_BYTE_CONTEXT |
				LDST_CLASS_1_CCB | (ctx1_iv_off <<
				LDST_OFFSET_SHIFT));

	/* Load counter into CONTEXT1 reg */
	if (is_rfc3686)
		append_load_imm_be32(desc, 1, LDST_IMM | LDST_CLASS_1_CCB |
				     LDST_SRCDST_BYTE_CONTEXT |
				     ((ctx1_iv_off + CTR_RFC3686_IV_SIZE) <<
				      LDST_OFFSET_SHIFT));

	/* Load operation */
	if (is_chacha20)
		options |= OP_ALG_AS_FINALIZE;
	append_operation(desc, options);

	/* Perform operation */
	skcipher_append_src_dst(desc);

	/* Store IV */
	if (!is_chacha20 && ivsize)
		append_seq_store(desc, ivsize, LDST_SRCDST_BYTE_CONTEXT |
				 LDST_CLASS_1_CCB | (ctx1_iv_off <<
				 LDST_OFFSET_SHIFT));

	print_hex_dump_debug("skcipher enc shdesc@" __stringify(__LINE__)": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc),
			     1);
}
EXPORT_SYMBOL(cnstr_shdsc_skcipher_encap);

/**
 * cnstr_shdsc_skcipher_decap - skcipher decapsulation shared descriptor
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - one of OP_ALG_ALGSEL_{AES, DES, 3DES} ANDed
 *         with OP_ALG_AAI_CBC or OP_ALG_AAI_CTR_MOD128
 *                                - OP_ALG_ALGSEL_CHACHA20
 * @ivsize: initialization vector size
 * @is_rfc3686: true when ctr(aes) is wrapped by rfc3686 template
 * @ctx1_iv_off: IV offset in CONTEXT1 register
 */
void cnstr_shdsc_skcipher_decap(u32 * const desc, struct alginfo *cdata,
				unsigned int ivsize, const bool is_rfc3686,
				const u32 ctx1_iv_off)
{
	u32 *key_jump_cmd;
	bool is_chacha20 = ((cdata->algtype & OP_ALG_ALGSEL_MASK) ==
			    OP_ALG_ALGSEL_CHACHA20);

	init_sh_desc(desc, HDR_SHARE_SERIAL | HDR_SAVECTX);
	/* Skip if already shared */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);

	/* Load class1 key only */
	if (IS_ENABLED(CONFIG_CRYPTO_DEV_FSL_CAAM_TK_API) &&
	    cdata->key_cmd_opt)
		/*
		 * Black keys can be loaded using only a KEY command
		 * with ENC=1 and the proper setting of the EKT bit.
		 */
		append_key_as_imm(desc, cdata->key_virt, cdata->keylen,
				  cdata->key_real_len, CLASS_1 |
				  KEY_DEST_CLASS_REG | cdata->key_cmd_opt);
	else
		append_key_as_imm(desc, cdata->key_virt, cdata->keylen,
				  cdata->keylen, CLASS_1 | KEY_DEST_CLASS_REG);

	/* Load nonce into CONTEXT1 reg */
	if (is_rfc3686) {
		const u8 *nonce = cdata->key_virt + cdata->keylen;

		append_load_as_imm(desc, nonce, CTR_RFC3686_NONCE_SIZE,
				   LDST_CLASS_IND_CCB |
				   LDST_SRCDST_BYTE_OUTFIFO | LDST_IMM);
		append_move(desc, MOVE_WAITCOMP | MOVE_SRC_OUTFIFO |
			    MOVE_DEST_CLASS1CTX | (16 << MOVE_OFFSET_SHIFT) |
			    (CTR_RFC3686_NONCE_SIZE << MOVE_LEN_SHIFT));
	}

	set_jump_tgt_here(desc, key_jump_cmd);

	/* Load IV, if there is one */
	if (ivsize)
		append_seq_load(desc, ivsize, LDST_SRCDST_BYTE_CONTEXT |
				LDST_CLASS_1_CCB | (ctx1_iv_off <<
				LDST_OFFSET_SHIFT));

	/* Load counter into CONTEXT1 reg */
	if (is_rfc3686)
		append_load_imm_be32(desc, 1, LDST_IMM | LDST_CLASS_1_CCB |
				     LDST_SRCDST_BYTE_CONTEXT |
				     ((ctx1_iv_off + CTR_RFC3686_IV_SIZE) <<
				      LDST_OFFSET_SHIFT));

	/* Choose operation */
	if (ctx1_iv_off)
		append_operation(desc, cdata->algtype | OP_ALG_AS_INIT |
				 OP_ALG_DECRYPT);
	else
		append_dec_op1(desc, cdata->algtype);

	/* Perform operation */
	skcipher_append_src_dst(desc);

	/* Store IV */
	if (!is_chacha20 && ivsize)
		append_seq_store(desc, ivsize, LDST_SRCDST_BYTE_CONTEXT |
				 LDST_CLASS_1_CCB | (ctx1_iv_off <<
				 LDST_OFFSET_SHIFT));

	print_hex_dump_debug("skcipher dec shdesc@" __stringify(__LINE__)": ",
			     DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc),
			     1);
}
EXPORT_SYMBOL(cnstr_shdsc_skcipher_decap);

/**
 * cnstr_shdsc_xts_skcipher_encap - xts skcipher encapsulation shared descriptor
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - OP_ALG_ALGSEL_AES ANDed with OP_ALG_AAI_XTS.
 */
void cnstr_shdsc_xts_skcipher_encap(u32 * const desc, struct alginfo *cdata)
{
	/*
	 * Set sector size to a big value, practically disabling
	 * sector size segmentation in xts implementation. We cannot
	 * take full advantage of this HW feature with existing
	 * crypto API / dm-crypt SW architecture.
	 */
	__be64 sector_size = cpu_to_be64(BIT(15));
	u32 *key_jump_cmd;

	init_sh_desc(desc, HDR_SHARE_SERIAL | HDR_SAVECTX);
	/* Skip if already shared */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);

	/* Load class1 keys only */
	append_key_as_imm(desc, cdata->key_virt, cdata->keylen,
			  cdata->keylen, CLASS_1 | KEY_DEST_CLASS_REG);

	/* Load sector size with index 40 bytes (0x28) */
	append_load_as_imm(desc, (void *)&sector_size, 8, LDST_CLASS_1_CCB |
			   LDST_SRCDST_BYTE_CONTEXT |
			   (0x28 << LDST_OFFSET_SHIFT));

	set_jump_tgt_here(desc, key_jump_cmd);

	/*
	 * create sequence for loading the sector index / 16B tweak value
	 * Lower 8B of IV - sector index / tweak lower half
	 * Upper 8B of IV - upper half of 16B tweak
	 */
	append_seq_load(desc, 8, LDST_SRCDST_BYTE_CONTEXT | LDST_CLASS_1_CCB |
			(0x20 << LDST_OFFSET_SHIFT));
	append_seq_load(desc, 8, LDST_SRCDST_BYTE_CONTEXT | LDST_CLASS_1_CCB |
			(0x30 << LDST_OFFSET_SHIFT));

	/* Load operation */
	append_operation(desc, cdata->algtype | OP_ALG_AS_INITFINAL |
			 OP_ALG_ENCRYPT);

	/* Perform operation */
	skcipher_append_src_dst(desc);

	/* Store lower 8B and upper 8B of IV */
	append_seq_store(desc, 8, LDST_SRCDST_BYTE_CONTEXT | LDST_CLASS_1_CCB |
			 (0x20 << LDST_OFFSET_SHIFT));
	append_seq_store(desc, 8, LDST_SRCDST_BYTE_CONTEXT | LDST_CLASS_1_CCB |
			 (0x30 << LDST_OFFSET_SHIFT));

	print_hex_dump_debug("xts skcipher enc shdesc@" __stringify(__LINE__)
			     ": ", DUMP_PREFIX_ADDRESS, 16, 4,
			     desc, desc_bytes(desc), 1);
}
EXPORT_SYMBOL(cnstr_shdsc_xts_skcipher_encap);

/**
 * cnstr_shdsc_xts_skcipher_decap - xts skcipher decapsulation shared descriptor
 * @desc: pointer to buffer used for descriptor construction
 * @cdata: pointer to block cipher transform definitions
 *         Valid algorithm values - OP_ALG_ALGSEL_AES ANDed with OP_ALG_AAI_XTS.
 */
void cnstr_shdsc_xts_skcipher_decap(u32 * const desc, struct alginfo *cdata)
{
	/*
	 * Set sector size to a big value, practically disabling
	 * sector size segmentation in xts implementation. We cannot
	 * take full advantage of this HW feature with existing
	 * crypto API / dm-crypt SW architecture.
	 */
	__be64 sector_size = cpu_to_be64(BIT(15));
	u32 *key_jump_cmd;

	init_sh_desc(desc, HDR_SHARE_SERIAL | HDR_SAVECTX);
	/* Skip if already shared */
	key_jump_cmd = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
				   JUMP_COND_SHRD);

	/* Load class1 key only */
	append_key_as_imm(desc, cdata->key_virt, cdata->keylen,
			  cdata->keylen, CLASS_1 | KEY_DEST_CLASS_REG);

	/* Load sector size with index 40 bytes (0x28) */
	append_load_as_imm(desc, (void *)&sector_size, 8, LDST_CLASS_1_CCB |
			   LDST_SRCDST_BYTE_CONTEXT |
			   (0x28 << LDST_OFFSET_SHIFT));

	set_jump_tgt_here(desc, key_jump_cmd);

	/*
	 * create sequence for loading the sector index / 16B tweak value
	 * Lower 8B of IV - sector index / tweak lower half
	 * Upper 8B of IV - upper half of 16B tweak
	 */
	append_seq_load(desc, 8, LDST_SRCDST_BYTE_CONTEXT | LDST_CLASS_1_CCB |
			(0x20 << LDST_OFFSET_SHIFT));
	append_seq_load(desc, 8, LDST_SRCDST_BYTE_CONTEXT | LDST_CLASS_1_CCB |
			(0x30 << LDST_OFFSET_SHIFT));
	/* Load operation */
	append_dec_op1(desc, cdata->algtype);

	/* Perform operation */
	skcipher_append_src_dst(desc);

	/* Store lower 8B and upper 8B of IV */
	append_seq_store(desc, 8, LDST_SRCDST_BYTE_CONTEXT | LDST_CLASS_1_CCB |
			 (0x20 << LDST_OFFSET_SHIFT));
	append_seq_store(desc, 8, LDST_SRCDST_BYTE_CONTEXT | LDST_CLASS_1_CCB |
			 (0x30 << LDST_OFFSET_SHIFT));

	print_hex_dump_debug("xts skcipher dec shdesc@" __stringify(__LINE__)
			     ": ", DUMP_PREFIX_ADDRESS, 16, 4, desc,
			     desc_bytes(desc), 1);
}
EXPORT_SYMBOL(cnstr_shdsc_xts_skcipher_decap);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FSL CAAM descriptor support");
MODULE_AUTHOR("Freescale Semiconductor - NMG/STC");
