/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Packet engine driver for Analog Devices Incorporated
 *
 * Currently tested on SC598 processor
 *
 * Copyright (c) 2023 - Timesys Corporation
 *   Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 */

#ifndef ADI_PKTE_HASH
#define ADI_PKTE_HASH

#define NUM_HASH_CATEGORIES 2
extern struct adi_algs_info adi_algs_info_adi[NUM_HASH_CATEGORIES];

void adi_write_packet(struct adi_dev *pkte_dev, u32 *source);

#endif
