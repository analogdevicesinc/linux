/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#ifndef _ATH12K_PCIC_H
#define _ATH12K_PCIC_H

#include <linux/types.h>

struct ath12k_base;

int ath12k_pcic_start(struct ath12k_base *ab);
void ath12k_pcic_stop(struct ath12k_base *ab);
void ath12k_pcic_write32(struct ath12k_base *ab, u32 offset, u32 value);
u32 ath12k_pcic_read32(struct ath12k_base *ab, u32 offset);
void ath12k_pcic_get_msi_address(struct ath12k_base *ab, u32 *msi_addr_lo,
				 u32 *msi_addr_hi);
int ath12k_pcic_cmem_write32(struct ath12k_base *ab, u32 addr,
			     u32 value);
int ath12k_pcic_cmem_read32(struct ath12k_base *ab, u32 addr, u32 *value);
void ath12k_pcic_config_static_window(struct ath12k_base *ab);
#endif /* _ATH12K_PCIC_H */
