/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_ASM_ALPHA_PERF_EVENT_H
#define _UAPI_ASM_ALPHA_PERF_EVENT_H

#include <linux/types.h>

/*
 * PERF_SAMPLE_RAW payload of a ProfileMe sample from the cpu PMU.
 *
 * pmpc, i_stat and pctr_ctl are the values of the PMPC, I_STAT and
 * PCTR_CTL registers when the ProfileMe window closed. i_stat holds only
 * bits 40:30 and pctr_ctl only bits 47:0. See the EV67/EV68 Hardware
 * Reference Manuals for the field layout.
 *
 * In the sample record the payload follows the 32-bit PERF_SAMPLE_RAW
 * size, so it starts 4 bytes past an 8-byte boundary. The 64-bit fields
 * are packed after the 32-bit version so that they are 8-byte aligned in
 * the record and no padding follows the payload.
 *
 * Fields may be added to the end in later versions.
 */
#define ALPHA_PROFILEME_SAMPLE_VERSION	1

struct alpha_profileme_sample {
	__u32 version;
	__u64 pmpc __attribute__((packed));
	__u64 i_stat __attribute__((packed));
	__u64 pctr_ctl __attribute__((packed));
};

#endif /* _UAPI_ASM_ALPHA_PERF_EVENT_H */
