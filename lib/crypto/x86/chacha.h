/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ChaCha and HChaCha functions (x86_64 optimized)
 *
 * Copyright (C) 2015 Martin Willi
 */

#include <asm/simd.h>
#include <linux/jump_label.h>
#include <linux/kernel.h>
#include <linux/sizes.h>

asmlinkage void chacha_block_xor_ssse3(const struct chacha_state *state,
				       u8 *dst, const u8 *src, size_t len,
				       int nrounds);
asmlinkage void chacha_4block_xor_ssse3(const struct chacha_state *state,
					u8 *dst, const u8 *src, size_t len,
					int nrounds);
asmlinkage void hchacha_block_ssse3(const struct chacha_state *state,
				    u32 out[HCHACHA_OUT_WORDS], int nrounds);

asmlinkage void chacha_2block_xor_avx2(const struct chacha_state *state,
				       u8 *dst, const u8 *src, size_t len,
				       int nrounds);
asmlinkage void chacha_4block_xor_avx2(const struct chacha_state *state,
				       u8 *dst, const u8 *src, size_t len,
				       int nrounds);
asmlinkage void chacha_8block_xor_avx2(const struct chacha_state *state,
				       u8 *dst, const u8 *src, size_t len,
				       int nrounds);

asmlinkage void chacha_2block_xor_avx512vl(const struct chacha_state *state,
					   u8 *dst, const u8 *src, size_t len,
					   int nrounds);
asmlinkage void chacha_4block_xor_avx512vl(const struct chacha_state *state,
					   u8 *dst, const u8 *src, size_t len,
					   int nrounds);
asmlinkage void chacha_8block_xor_avx512vl(const struct chacha_state *state,
					   u8 *dst, const u8 *src, size_t len,
					   int nrounds);

asmlinkage void chacha_16block_xor_avx512(const struct chacha_state *state,
					  u8 *dst, const u8 *src, size_t len,
					  int nrounds);

static __ro_after_init DEFINE_STATIC_KEY_FALSE(chacha_use_simd);
static __ro_after_init DEFINE_STATIC_KEY_FALSE(chacha_use_avx2);
static __ro_after_init DEFINE_STATIC_KEY_FALSE(chacha_use_avx512vl);
static __ro_after_init DEFINE_STATIC_KEY_FALSE(chacha_use_avx512);

static void chacha_dosimd(struct chacha_state *state, u8 *dst, const u8 *src,
			  size_t bytes, int nrounds)
{
	if (static_branch_likely(&chacha_use_avx512vl)) {
		if (static_branch_likely(&chacha_use_avx512)) {
			while (bytes >= CHACHA_BLOCK_SIZE * 16) {
				chacha_16block_xor_avx512(
					state, dst, src, CHACHA_BLOCK_SIZE * 16,
					nrounds);
				bytes -= CHACHA_BLOCK_SIZE * 16;
				src += CHACHA_BLOCK_SIZE * 16;
				dst += CHACHA_BLOCK_SIZE * 16;
				state->x[12] += 16;
			}
			if (bytes > CHACHA_BLOCK_SIZE * 8) {
				chacha_16block_xor_avx512(state, dst, src,
							  bytes, nrounds);
				state->x[12] +=
					DIV_ROUND_UP(bytes, CHACHA_BLOCK_SIZE);
				return;
			}
		} else {
			while (bytes >= CHACHA_BLOCK_SIZE * 8) {
				chacha_8block_xor_avx512vl(
					state, dst, src, CHACHA_BLOCK_SIZE * 8,
					nrounds);
				bytes -= CHACHA_BLOCK_SIZE * 8;
				src += CHACHA_BLOCK_SIZE * 8;
				dst += CHACHA_BLOCK_SIZE * 8;
				state->x[12] += 8;
			}
		}
		if (bytes > CHACHA_BLOCK_SIZE * 4)
			chacha_8block_xor_avx512vl(state, dst, src, bytes,
						   nrounds);
		else if (bytes > CHACHA_BLOCK_SIZE * 2)
			chacha_4block_xor_avx512vl(state, dst, src, bytes,
						   nrounds);
		else if (bytes)
			chacha_2block_xor_avx512vl(state, dst, src, bytes,
						   nrounds);
		state->x[12] += DIV_ROUND_UP(bytes, CHACHA_BLOCK_SIZE);
	} else if (static_branch_likely(&chacha_use_avx2)) {
		while (bytes >= CHACHA_BLOCK_SIZE * 8) {
			chacha_8block_xor_avx2(state, dst, src,
					       CHACHA_BLOCK_SIZE * 8, nrounds);
			bytes -= CHACHA_BLOCK_SIZE * 8;
			src += CHACHA_BLOCK_SIZE * 8;
			dst += CHACHA_BLOCK_SIZE * 8;
			state->x[12] += 8;
		}
		if (bytes > CHACHA_BLOCK_SIZE * 4)
			chacha_8block_xor_avx2(state, dst, src, bytes, nrounds);
		else if (bytes > CHACHA_BLOCK_SIZE * 2)
			chacha_4block_xor_avx2(state, dst, src, bytes, nrounds);
		else if (bytes > CHACHA_BLOCK_SIZE)
			chacha_2block_xor_avx2(state, dst, src, bytes, nrounds);
		else if (bytes)
			chacha_block_xor_ssse3(state, dst, src, bytes, nrounds);
		state->x[12] += DIV_ROUND_UP(bytes, CHACHA_BLOCK_SIZE);
	} else {
		while (bytes >= CHACHA_BLOCK_SIZE * 4) {
			chacha_4block_xor_ssse3(state, dst, src,
						CHACHA_BLOCK_SIZE * 4, nrounds);
			bytes -= CHACHA_BLOCK_SIZE * 4;
			src += CHACHA_BLOCK_SIZE * 4;
			dst += CHACHA_BLOCK_SIZE * 4;
			state->x[12] += 4;
		}
		if (bytes > CHACHA_BLOCK_SIZE)
			chacha_4block_xor_ssse3(state, dst, src, bytes,
						nrounds);
		else if (bytes)
			chacha_block_xor_ssse3(state, dst, src, bytes, nrounds);
		state->x[12] += DIV_ROUND_UP(bytes, CHACHA_BLOCK_SIZE);
	}
}

static void hchacha_block_arch(const struct chacha_state *state,
			       u32 out[HCHACHA_OUT_WORDS], int nrounds)
{
	if (!static_branch_likely(&chacha_use_simd)) {
		hchacha_block_generic(state, out, nrounds);
	} else {
		kernel_fpu_begin();
		hchacha_block_ssse3(state, out, nrounds);
		kernel_fpu_end();
	}
}

static void chacha_crypt_arch(struct chacha_state *state, u8 *dst,
			      const u8 *src, size_t bytes, int nrounds)
{
	if (!static_branch_likely(&chacha_use_simd) ||
	    bytes <= CHACHA_BLOCK_SIZE)
		return chacha_crypt_generic(state, dst, src, bytes, nrounds);

	do {
		size_t todo = min(bytes, SZ_4K);

		kernel_fpu_begin();
		chacha_dosimd(state, dst, src, todo, nrounds);
		kernel_fpu_end();

		bytes -= todo;
		src += todo;
		dst += todo;
	} while (bytes);
}

#define chacha_mod_init_arch chacha_mod_init_arch
static void chacha_mod_init_arch(void)
{
	if (!boot_cpu_has(X86_FEATURE_SSSE3))
		return;

	static_branch_enable(&chacha_use_simd);

	if (boot_cpu_has(X86_FEATURE_AVX) &&
	    boot_cpu_has(X86_FEATURE_AVX2)) {
		static_branch_enable(&chacha_use_avx2);

		if (boot_cpu_has(X86_FEATURE_AVX512VL) &&
		    boot_cpu_has(X86_FEATURE_AVX512BW) &&
		    boot_cpu_has(X86_FEATURE_BMI2)) {
			static_branch_enable(&chacha_use_avx512vl);

			if (!boot_cpu_has(X86_FEATURE_PREFER_YMM))
				static_branch_enable(&chacha_use_avx512);
		}
	}
}
