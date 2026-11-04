/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <asm/cpufeature.h>

extern const struct raid6_calls raid6_sse2x1;
extern const struct raid6_calls raid6_sse2x2;
extern const struct raid6_calls raid6_sse2x4;
extern const struct raid6_calls raid6_avx2x1;
extern const struct raid6_calls raid6_avx2x2;
extern const struct raid6_calls raid6_avx2x4;
extern const struct raid6_calls raid6_avx512x1;
extern const struct raid6_calls raid6_avx512x2;
extern const struct raid6_calls raid6_avx512x4;

extern const struct raid6_recov_calls raid6_recov_ssse3;
extern const struct raid6_recov_calls raid6_recov_avx2;
extern const struct raid6_recov_calls raid6_recov_avx512;

static __always_inline void __init arch_raid6_init(void)
{
	if (boot_cpu_has(X86_FEATURE_AVX2)) {
		raid6_algo_add(&raid6_avx2x1);
		raid6_algo_add(&raid6_avx2x2);
		if (IS_ENABLED(CONFIG_X86_64))
			raid6_algo_add(&raid6_avx2x4);
		if (boot_cpu_has(X86_FEATURE_AVX512BW) &&
		    boot_cpu_has(X86_FEATURE_AVX512VL) &&
		    boot_cpu_has(X86_FEATURE_AVX512DQ)) {
			raid6_algo_add(&raid6_avx512x1);
			raid6_algo_add(&raid6_avx512x2);
			if (IS_ENABLED(CONFIG_X86_64))
				raid6_algo_add(&raid6_avx512x4);
			raid6_recov_algo_add(&raid6_recov_avx512);
		} else {
			raid6_recov_algo_add(&raid6_recov_avx2);
		}
	} else if (IS_ENABLED(CONFIG_X86_64) ||
		   boot_cpu_has(X86_FEATURE_XMM2)) {
		/* x86_64 can assume SSE2 as baseline */
		raid6_algo_add(&raid6_sse2x1);
		raid6_algo_add(&raid6_sse2x2);
		if (IS_ENABLED(CONFIG_X86_64))
			raid6_algo_add(&raid6_sse2x4);
		if (boot_cpu_has(X86_FEATURE_SSSE3))
			raid6_recov_algo_add(&raid6_recov_ssse3);
	} else {
		raid6_algo_add_default();
	}
}
