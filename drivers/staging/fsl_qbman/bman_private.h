/* Copyright 2008-2012 Freescale Semiconductor, Inc.
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

#include "dpa_sys.h"
#include <linux/fsl_bman.h>

/* Revision info (for errata and feature handling) */
#define BMAN_REV10 0x0100
#define BMAN_REV20 0x0200
#define BMAN_REV21 0x0201
#define QBMAN_ANY_PORTAL_IDX 0xffffffff
extern u16 bman_ip_rev; /* 0 if uninitialised, otherwise QMAN_REVx */

/*
 * Global variables of the max portal/pool number this bman version supported
 */
extern u16 bman_pool_max;

/* used by CCSR and portal interrupt code */
enum bm_isr_reg {
	bm_isr_status = 0,
	bm_isr_enable = 1,
	bm_isr_disable = 2,
	bm_isr_inhibit = 3
};

struct bm_portal_config {
	/* Corenet portal addresses;
	 * [0]==cache-enabled, [1]==cache-inhibited. */
	__iomem void *addr_virt[2];
	struct resource addr_phys[2];
	/* Allow these to be joined in lists */
	struct list_head list;
	/* User-visible portal configuration settings */
	struct bman_portal_config public_cfg;
	/* power management saved data */
	u32 saved_isdr;
};

#ifdef CONFIG_FSL_BMAN_CONFIG
/* Hooks from bman_driver.c to bman_config.c */
int bman_init_ccsr(struct device_node *node, bool *need_cleanup);
#endif

/* Hooks from bman_driver.c in to bman_high.c */
struct bman_portal *bman_create_portal(
				       struct bman_portal *portal,
				       const struct bm_portal_config *config);
struct bman_portal *bman_create_affine_portal(
			const struct bm_portal_config *config);
struct bman_portal *bman_create_affine_slave(struct bman_portal *redirect,
								int cpu);
void bman_destroy_portal(struct bman_portal *bm);

const struct bm_portal_config *bman_destroy_affine_portal(void);

/* Hooks from fsl_usdpaa.c to bman_driver.c */
struct bm_portal_config *bm_get_unused_portal(void);
struct bm_portal_config *bm_get_unused_portal_idx(uint32_t idx);
void bm_put_unused_portal(struct bm_portal_config *pcfg);
void bm_set_liodns(struct bm_portal_config *pcfg);

/* Pool logic in the portal driver, during initialisation, needs to know if
 * there's access to CCSR or not (if not, it'll cripple the pool allocator). */
#ifdef CONFIG_FSL_BMAN_CONFIG
int bman_have_ccsr(void);
#else
#define bman_have_ccsr() 0
#endif

/* Stockpile build constants. The _LOW value: when bman_acquire() is called and
 * the stockpile fill-level is <= _LOW, an acquire is attempted from h/w but it
 * might fail (if the buffer pool is depleted). So this value provides some
 * "stagger" in that the bman_acquire() function will only fail if lots of bufs
 * are requested at once or if h/w has been tested a couple of times without
 * luck. The _HIGH value: when bman_release() is called and the stockpile
 * fill-level is >= _HIGH, a release is attempted to h/w but it might fail (if
 * the release ring is full). So this value provides some "stagger" so that
 * ring-access is retried a couple of times prior to the API returning a
 * failure. The following *must* be true;
 *   BMAN_STOCKPILE_HIGH-BMAN_STOCKPILE_LOW > 8
 *     (to avoid thrashing)
 *   BMAN_STOCKPILE_SZ >= 16
 *     (as the release logic expects to either send 8 buffers to hw prior to
 *     adding the given buffers to the stockpile or add the buffers to the
 *     stockpile before sending 8 to hw, as the API must be an all-or-nothing
 *     success/fail.)
 */
#define BMAN_STOCKPILE_SZ   16u /* number of bufs in per-pool cache */
#define BMAN_STOCKPILE_LOW  2u  /* when fill is <= this, acquire from hw */
#define BMAN_STOCKPILE_HIGH 14u /* when fill is >= this, release to hw */

/*************************************************/
/*   BMan s/w corenet portal, low-level i/face   */
/*************************************************/

/* Used by all portal interrupt registers except 'inhibit'
 * This mask contains all the "irqsource" bits visible to API users
 */
#define BM_PIRQ_VISIBLE	(BM_PIRQ_RCRI | BM_PIRQ_BSCN)

/* These are bm_<reg>_<verb>(). So for example, bm_disable_write() means "write
 * the disable register" rather than "disable the ability to write". */
#define bm_isr_status_read(bm)		__bm_isr_read(bm, bm_isr_status)
#define bm_isr_status_clear(bm, m)	__bm_isr_write(bm, bm_isr_status, m)
#define bm_isr_enable_read(bm)		__bm_isr_read(bm, bm_isr_enable)
#define bm_isr_enable_write(bm, v)	__bm_isr_write(bm, bm_isr_enable, v)
#define bm_isr_disable_read(bm)		__bm_isr_read(bm, bm_isr_disable)
#define bm_isr_disable_write(bm, v)	__bm_isr_write(bm, bm_isr_disable, v)
#define bm_isr_inhibit(bm)		__bm_isr_write(bm, bm_isr_inhibit, 1)
#define bm_isr_uninhibit(bm)		__bm_isr_write(bm, bm_isr_inhibit, 0)

#ifdef CONFIG_FSL_BMAN_CONFIG
/* Set depletion thresholds associated with a buffer pool. Requires that the
 * operating system have access to Bman CCSR (ie. compiled in support and
 * run-time access courtesy of the device-tree). */
int bm_pool_set(u32 bpid, const u32 *thresholds);
#define BM_POOL_THRESH_SW_ENTER 0
#define BM_POOL_THRESH_SW_EXIT  1
#define BM_POOL_THRESH_HW_ENTER 2
#define BM_POOL_THRESH_HW_EXIT  3

/* Read the free buffer count for a given buffer */
u32 bm_pool_free_buffers(u32 bpid);

__init int bman_init(void);
__init int bman_resource_init(void);
__init int bman_init_early(void);

const struct bm_portal_config *bman_get_bm_portal_config(
						struct bman_portal *portal);

/* power management */
#ifdef CONFIG_SUSPEND
void suspend_unused_bportal(void);
void resume_unused_bportal(void);
#endif

#endif /* CONFIG_FSL_BMAN_CONFIG */
