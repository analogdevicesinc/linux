/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef LNXWRP_SYSFS_FM_H_
#define LNXWRP_SYSFS_FM_H_

#include "lnxwrp_sysfs.h"

int fm_sysfs_create(struct device *dev);
void fm_sysfs_destroy(struct device *dev);
int fm_dump_regs(void *h_dev, char *buf, int nn);
int fm_fpm_dump_regs(void *h_dev, char *buf, int nn);
int fm_kg_dump_regs(void *h_pcd, char *buf, int nn);
int fm_kg_pe_dump_regs(void *h_pcd, char *buf, int nn);
int fm_dump_scheme(void *h_pcd, int scnum, char *buf, int nn);
int fm_dump_tnum_dbg(void *h_fm, int tn_s, int tn_e, char *buf, int nn);
int fm_dump_cls_plan(void *h_pcd, int cpn, char *buf, int nn);
int fm_plcr_dump_regs(void *h_pcd, char *buf, int nn);
int fm_prs_dump_regs(void *h_pcd, char *buf, int nn);
int fm_profile_dump_regs(void *h_pcd, int ppnum, char *buf, int nn);

#define FM_DMP_PGSZ_ERR { \
			snprintf(&buf[PAGE_SIZE - 80], 70, \
			"\n Err: current sysfs buffer reached PAGE_SIZE\n");\
			n = PAGE_SIZE - 2; \
			}

#define FM_DMP_LN(buf, n, ...) \
	do { \
		int k, m = n; \
		m += k = snprintf(&buf[m], PAGE_SIZE - m, __VA_ARGS__); \
		if (k < 0 || m > PAGE_SIZE - 90) \
			FM_DMP_PGSZ_ERR \
		n = m; \
	} while (0)

#define FM_DMP_TITLE(buf, n, addr, ...) \
	do { \
		int k, m = n; \
		m += k = snprintf(&buf[m], PAGE_SIZE - m, "\n"); \
		if (k < 0 || m > PAGE_SIZE - 90) \
			FM_DMP_PGSZ_ERR \
		m += k = snprintf(&buf[m], PAGE_SIZE - m, __VA_ARGS__); \
		if (k < 0 || m > PAGE_SIZE - 90) \
			FM_DMP_PGSZ_ERR \
		if (addr) {                           \
			phys_addr_t pa; \
			pa = virt_to_phys(addr); \
			m += k = \
			snprintf(&buf[m], PAGE_SIZE - m, " (0x%lX)", \
				(long unsigned int)(pa)); \
			if (k < 0 || m > PAGE_SIZE - 90) \
				FM_DMP_PGSZ_ERR \
		} \
		m += k = snprintf(&buf[m], PAGE_SIZE - m, \
			"\n----------------------------------------\n\n"); \
			if (k < 0 || m > PAGE_SIZE - 90) \
				FM_DMP_PGSZ_ERR \
		n = m; \
	} while (0)

#define FM_DMP_SUBTITLE(buf, n, ...) \
	do { \
		int k, m = n; \
		m += k = snprintf(&buf[m], PAGE_SIZE - m, "------- "); \
		if (k < 0 || m > PAGE_SIZE - 90) \
			FM_DMP_PGSZ_ERR \
		m += k = snprintf(&buf[m], PAGE_SIZE - m, __VA_ARGS__); \
		if (k < 0 || m > PAGE_SIZE - 90) \
			FM_DMP_PGSZ_ERR \
		m += k = snprintf(&buf[m], PAGE_SIZE - m, "\n"); \
		if (k < 0 || m > PAGE_SIZE - 90) \
			FM_DMP_PGSZ_ERR \
		n = m; \
	} while (0)

#define FM_DMP_MEM_32(buf, n, addr) \
	{ \
		uint32_t val; \
		phys_addr_t pa; \
		int k, m = n; \
		pa = virt_to_phys(addr); \
		val = ioread32be((addr)); \
		do { \
			m += k = snprintf(&buf[m], \
				PAGE_SIZE - m, "0x%010llX: 0x%08x\n", \
				pa, val); \
			if (k < 0 || m > PAGE_SIZE - 90) \
				FM_DMP_PGSZ_ERR \
			n += k; \
		} while (0) ;\
	}

#define FM_DMP_V32(buf, n, st, phrase) \
	do { \
		int k, m = n; \
		phys_addr_t pa = virt_to_phys(&((st)->phrase)); \
		k = snprintf(&buf[m], PAGE_SIZE - m, \
		"0x%010llX: 0x%08x%8s\t%s\n", (unsigned long long) pa, \
		ioread32be((uint32_t *)&((st)->phrase)), "", #phrase); \
		if (k < 0 || m > PAGE_SIZE - 90) \
			FM_DMP_PGSZ_ERR \
		n += k; \
	} while (0)

#endif /* LNXWRP_SYSFS_FM_H_ */
