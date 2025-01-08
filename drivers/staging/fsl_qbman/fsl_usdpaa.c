/* Copyright (C) 2008-2012 Freescale Semiconductor, Inc.
 * Copyright 2020,2023-2024 NXP
 * Authors: Andy Fleming <afleming@freescale.com>
 *	    Timur Tabi <timur@freescale.com>
 *	    Geoff Thorpe <Geoff.Thorpe@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/memblock.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mman.h>
#include <linux/of_reserved_mem.h>
#include <linux/eventfd.h>
#include <linux/fdtable.h>

#include "dpa_sys.h"
#include <linux/fsl_usdpaa.h>
#include "bman_low.h"
#include "qman_low.h"
/* Headers requires for
 * Link status support
 */
#include <linux/device.h>
#include <linux/of_mdio.h>
#include "mac.h"
#include "dpaa_eth_common.h"

#define USDPAA_IOCTL_VERSION_NUMBER 2

/* Private data for Proxy Interface */
struct dpa_proxy_priv_s {
	struct mac_device	*mac_dev;
};

struct eventfd_list {
	struct list_head d_list;
	struct list_head ctx_list;
	struct net_device *ndev;
	struct eventfd_ctx *efd_ctx;
};

/* Interface Helpers */
static inline struct device *get_dev_ptr(char *if_name);
static void phy_link_updates(struct net_device *net_dev);
/* IOCTL handlers */
static inline int
ioctl_usdpaa_get_link_status(struct usdpaa_ioctl_link_status_args *input);

struct ctx;
static int ioctl_en_if_link_status(struct usdpaa_ioctl_link_status *args, struct ctx *ctx);
static int ioctl_disable_if_link_status(char *if_name);

/* Physical address range of the memory reservation, exported for mm/mem.c */
static u64 phys_start;
static u64 phys_size;
static u64 arg_phys_size;

/* PFN versions of the above */
static unsigned long pfn_start;
static unsigned long pfn_size;

/* Memory reservations are manipulated under this spinlock (which is why 'refs'
 * isn't atomic_t). */
static DEFINE_SPINLOCK(mem_lock);

/* The range of TLB1 indices */
static unsigned int num_tlb = 1;

/* Memory reservation is represented as a list of 'mem_fragment's, some of which
 * may be mapped. Unmapped fragments are always merged where possible. */
static LIST_HEAD(mem_list);
static LIST_HEAD(eventfd_head);

struct mem_mapping;

/* Memory fragments are in 'mem_list'. */
struct mem_fragment {
	u64 base;
	u64 len;
	unsigned long pfn_base; /* PFN version of 'base' */
	unsigned long pfn_len; /* PFN version of 'len' */
	unsigned int refs; /* zero if unmapped */
	u64 root_len; /* Size of the orignal fragment */
	unsigned long root_pfn; /* PFN of the orignal fragment */
	struct list_head list;
	/* if mapped, flags+name captured at creation time */
	u32 flags;
	char name[USDPAA_DMA_NAME_MAX];
	u64 map_len;
	/* support multi-process locks per-memory-fragment. */
	int has_locking;
	wait_queue_head_t wq;
	struct mem_mapping *owner;
};

/* Mappings of memory fragments in 'struct ctx'. These are created from
 * ioctl(USDPAA_IOCTL_DMA_MAP), though the actual mapping then happens via a
 * mmap(). */
struct mem_mapping {
	struct mem_fragment *root_frag;
	u32 frag_count;
	u64 total_size;
	struct list_head list;
	int refs;
	void *virt_addr;
};

struct portal_mapping {
	struct usdpaa_ioctl_portal_map user;
	union {
		struct qm_portal_config *qportal;
		struct bm_portal_config *bportal;
	};
	/* Declare space for the portals in case the process
	   exits unexpectedly and needs to be cleaned by the kernel */
	union {
		struct qm_portal qman_portal_low;
		struct bm_portal bman_portal_low;
	};
	struct list_head list;
	struct resource *phys;
	struct iommu_domain *iommu_domain;
};

/* Track the DPAA resources the process is using */
struct active_resource {
	struct list_head list;
	u32 id;
	u32 num;
	unsigned int refcount;
};

/* Per-FD state (which should also be per-process but we don't enforce that) */
struct ctx {
	/* Lock to protect the context */
	spinlock_t lock;
	/* Allocated resources get put here for accounting */
	struct list_head resources[usdpaa_id_max];
	/* list of DMA maps */
	struct list_head maps;
	/* list of portal maps */
	struct list_head portals;
	/* list of event fds */
	struct list_head events;
};

/* Different resource classes */
static const struct alloc_backend {
	enum usdpaa_id_type id_type;
	int (*alloc)(u32 *, u32, u32, int);
	void (*release)(u32 base, unsigned int count);
	int (*reserve)(u32 base, unsigned int count);
	const char *acronym;
} alloc_backends[] = {
	{
		.id_type = usdpaa_id_fqid,
		.alloc = qman_alloc_fqid_range,
		.release = qman_release_fqid_range,
		.reserve = qman_reserve_fqid_range,
		.acronym = "FQID"
	},
	{
		.id_type = usdpaa_id_bpid,
		.alloc = bman_alloc_bpid_range,
		.release = bman_release_bpid_range,
		.reserve = bman_reserve_bpid_range,
		.acronym = "BPID"
	},
	{
		.id_type = usdpaa_id_qpool,
		.alloc = qman_alloc_pool_range,
		.release = qman_release_pool_range,
		.reserve = qman_reserve_pool_range,
		.acronym = "QPOOL"
	},
	{
		.id_type = usdpaa_id_cgrid,
		.alloc = qman_alloc_cgrid_range,
		.release = qman_release_cgrid_range,
		.acronym = "CGRID"
	},
	{
		.id_type = usdpaa_id_ceetm0_lfqid,
		.alloc = qman_alloc_ceetm0_lfqid_range,
		.release = qman_release_ceetm0_lfqid_range,
		.acronym = "CEETM0_LFQID"
	},
	{
		.id_type = usdpaa_id_ceetm0_channelid,
		.alloc = qman_alloc_ceetm0_channel_range,
		.release = qman_release_ceetm0_channel_range,
		.acronym = "CEETM0_LFQID"
	},
	{
		.id_type = usdpaa_id_ceetm1_lfqid,
		.alloc = qman_alloc_ceetm1_lfqid_range,
		.release = qman_release_ceetm1_lfqid_range,
		.acronym = "CEETM1_LFQID"
	},
	{
		.id_type = usdpaa_id_ceetm1_channelid,
		.alloc = qman_alloc_ceetm1_channel_range,
		.release = qman_release_ceetm1_channel_range,
		.acronym = "CEETM1_LFQID"
	},
	{
		/* This terminates the array */
		.id_type = usdpaa_id_max
	}
};

/* Determines the largest acceptable page size for a given size
   The sizes are determined by what the TLB1 acceptable page sizes are */
static u32 largest_page_size(u32 size)
{
	int shift = 30; /* Start at 1G size */
	if (size < 4096)
		return 0;
	do {
		if (size >= (1<<shift))
			return 1<<shift;
		shift -= 2;
	} while (shift >= 12); /* Up to 4k */
	return 0;
}

/* Determine if value is power of 4 */
static inline bool is_power_of_4(u64 x)
{
	if (x == 0 || ((x & (x - 1)) != 0))
		return false;
	return !!(x & 0x5555555555555555ull);
}

/* Helper for ioctl_dma_map() when we have a larger fragment than we need. This
 * splits the fragment into 4 and returns the upper-most. (The caller can loop
 * until it has a suitable fragment size.) */
static struct mem_fragment *split_frag(struct mem_fragment *frag)
{
	struct mem_fragment *x[3];

	x[0] = kmalloc(sizeof(struct mem_fragment), GFP_ATOMIC);
	x[1] = kmalloc(sizeof(struct mem_fragment), GFP_ATOMIC);
	x[2] = kmalloc(sizeof(struct mem_fragment), GFP_ATOMIC);
	if (!x[0] || !x[1] || !x[2]) {
		kfree(x[0]);
		kfree(x[1]);
		kfree(x[2]);
		return NULL;
	}
	BUG_ON(frag->refs);
	frag->len >>= 2;
	frag->pfn_len >>= 2;
	x[0]->base = frag->base + frag->len;
	x[1]->base = x[0]->base + frag->len;
	x[2]->base = x[1]->base + frag->len;
	x[0]->len = x[1]->len = x[2]->len = frag->len;
	x[0]->pfn_base = frag->pfn_base + frag->pfn_len;
	x[1]->pfn_base = x[0]->pfn_base + frag->pfn_len;
	x[2]->pfn_base = x[1]->pfn_base + frag->pfn_len;
	x[0]->pfn_len = x[1]->pfn_len = x[2]->pfn_len = frag->pfn_len;
	x[0]->refs = x[1]->refs = x[2]->refs = 0;
	x[0]->root_len = x[1]->root_len = x[2]->root_len = frag->root_len;
	x[0]->root_pfn = x[1]->root_pfn = x[2]->root_pfn = frag->root_pfn;
	x[0]->name[0] = x[1]->name[0] = x[2]->name[0] = 0;
	list_add_tail(&x[0]->list, &frag->list);
	list_add_tail(&x[1]->list, &x[0]->list);
	list_add_tail(&x[2]->list, &x[1]->list);
	return x[2];
}

static __maybe_unused void dump_frags(void)
{
	struct mem_fragment *frag;
	int i = 0;
	list_for_each_entry(frag, &mem_list, list) {
		pr_info("FRAG %d: base 0x%llx pfn_base 0x%lx len 0x%llx root_len 0x%llx root_pfn 0x%lx refs %d name %s\n",
			i, frag->base, frag->pfn_base,
			frag->len, frag->root_len, frag->root_pfn,
			frag->refs, frag->name);
		++i;
	}
}

/* Walk the list of fragments and adjoin neighbouring segments if possible */
static void compress_frags(void)
{
	/* Walk the fragment list and combine fragments */
	struct mem_fragment *frag, *nxtfrag;
	u64 len = 0;

	int i, numfrags;


	frag = list_entry(mem_list.next, struct mem_fragment, list);

	while (&frag->list != &mem_list) {
		/* Must combine consecutive fragemenst with
		   same root_pfn such that they are power of 4 */
		if (frag->refs != 0) {
			frag = list_entry(frag->list.next,
					  struct mem_fragment, list);
			continue; /* Not this window */
		}
		len = frag->len;
		numfrags = 0;
		nxtfrag =  list_entry(frag->list.next,
				      struct mem_fragment, list);
		while (true) {
			if (&nxtfrag->list == &mem_list) {
				numfrags = 0;
				break; /* End of list */
			}
			if (nxtfrag->refs) {
				numfrags = 0;
				break; /* In use still */
			}
			if (nxtfrag->root_pfn != frag->root_pfn) {
				numfrags = 0;
				break; /* Crosses root fragment boundary */
			}
			len += nxtfrag->len;
			numfrags++;
			if (is_power_of_4(len)) {
				/* These fragments can be combined */
				break;
			}
			nxtfrag =  list_entry(nxtfrag->list.next,
					      struct mem_fragment, list);
		}
		if (numfrags == 0) {
			frag = list_entry(frag->list.next,
					  struct mem_fragment, list);
			continue; /* try the next window */
		}
		for (i = 0; i < numfrags; i++) {
			struct mem_fragment *todel =
				list_entry(nxtfrag->list.prev,
					   struct mem_fragment, list);
			nxtfrag->len += todel->len;
			nxtfrag->pfn_len += todel->pfn_len;
			list_del(&todel->list);
		}
		/* Re evaluate the list, things may merge now */
		frag = list_entry(mem_list.next, struct mem_fragment, list);
	}
}

static int usdpaa_open(struct inode *inode, struct file *filp)
{
	const struct alloc_backend *backend = &alloc_backends[0];
	struct ctx *ctx = kmalloc(sizeof(struct ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	filp->private_data = ctx;

	while (backend->id_type != usdpaa_id_max) {
		INIT_LIST_HEAD(&ctx->resources[backend->id_type]);
		backend++;
	}

	INIT_LIST_HEAD(&ctx->maps);
	INIT_LIST_HEAD(&ctx->portals);
	INIT_LIST_HEAD(&ctx->events);
	spin_lock_init(&ctx->lock);

	//filp->f_mapping->backing_dev_info = &directly_mappable_cdev_bdi;

	return 0;
}

#define DQRR_MAXFILL 15


/* Invalidate a portal */
static void dbci_portal(void *addr)
{
	int i;

	for (i = 0; i < 0x4000; i += 64)
		dcbi(addr + i);
}

/* Reset a QMan portal to its default state */
static int init_qm_portal(struct qm_portal_config *config,
			  struct qm_portal *portal)
{
	const struct qm_dqrr_entry *dqrr = NULL;
	int i;

	portal->addr.addr_ce = config->addr_virt[DPA_PORTAL_CE];
	portal->addr.addr_ci = config->addr_virt[DPA_PORTAL_CI];

	/* Make sure interrupts are inhibited */
	qm_out(IIR, 1);

	/*
	 * Invalidate the entire CE portal are to ensure no stale
	 * cachelines are present.  This should be done on all
	 * cores as the portal is mapped as M=0 (non-coherent).
	 */
	on_each_cpu(dbci_portal, portal->addr.addr_ce, 1);

	/* Initialize the DQRR.  This will stop any dequeue
	   commands that are in progress */
	if (qm_dqrr_init(portal, config, qm_dqrr_dpush, qm_dqrr_pvb,
			 qm_dqrr_cdc, DQRR_MAXFILL)) {
		pr_err("qm_dqrr_init() failed when trying to"
		       " recover portal, portal will be leaked\n");
		return 1;
	}

	/* Discard any entries on the DQRR */
	/* If we consume the ring twice something is wrong */
	for (i = 0; i < DQRR_MAXFILL * 2; i++) {
		qm_dqrr_pvb_update(portal);
		dqrr = qm_dqrr_current(portal);
		if (!dqrr)
			break;
		qm_dqrr_cdc_consume_1ptr(portal, dqrr, 0);
		qm_dqrr_pvb_update(portal);
		qm_dqrr_next(portal);
	}
	/* Initialize the EQCR */
	if (qm_eqcr_init(portal, qm_eqcr_pvb,
			qm_eqcr_get_ci_stashing(portal), 1)) {
		pr_err("Qman EQCR initialisation failed\n");
		return 1;
	}
	/* initialize the MR */
	if (qm_mr_init(portal, qm_mr_pvb, qm_mr_cci)) {
		pr_err("Qman MR initialisation failed\n");
		return 1;
	}
	qm_mr_pvb_update(portal);
	while (qm_mr_current(portal)) {
		qm_mr_next(portal);
		qm_mr_cci_consume_to_current(portal);
		qm_mr_pvb_update(portal);
	}

	if (qm_mc_init(portal)) {
		pr_err("Qman MC initialisation failed\n");
		return 1;
	}
	return 0;
}

static int init_bm_portal(struct bm_portal_config *config,
			  struct bm_portal *portal)
{
	portal->addr.addr_ce = config->addr_virt[DPA_PORTAL_CE];
	portal->addr.addr_ci = config->addr_virt[DPA_PORTAL_CI];

	/*
	 * Invalidate the entire CE portal are to ensure no stale
	 * cachelines are present.  This should be done on all
	 * cores as the portal is mapped as M=0 (non-coherent).
	 */
	on_each_cpu(dbci_portal, portal->addr.addr_ce, 1);

	if (bm_rcr_init(portal, bm_rcr_pvb, bm_rcr_cce)) {
		pr_err("Bman RCR initialisation failed\n");
	return 1;
	}
	if (bm_mc_init(portal)) {
		pr_err("Bman MC initialisation failed\n");
		return 1;
	}
	return 0;
}

/* Function that will scan all FQ's in the system.  For each FQ that is not
   OOS it will call the check_channel helper to determine if the FQ should
   be torn down.  If the check_channel helper returns true the FQ will be
   transitioned to the OOS state */
static int qm_check_and_destroy_fqs(struct qm_portal *portal, void *ctx,
				    bool (*check_channel)(void*, u32))
{
	u32 fq_id = 0;
	while (1) {
		struct qm_mc_command *mcc;
		struct qm_mc_result *mcr;
		u8 state;
		u32 channel;

		/* Determine the channel for the FQID */
		mcc = qm_mc_start(portal);
		mcc->queryfq.fqid = fq_id;
		qm_mc_commit(portal, QM_MCC_VERB_QUERYFQ);
		while (!(mcr = qm_mc_result(portal)))
			cpu_relax();
		DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK)
			   == QM_MCR_VERB_QUERYFQ);
		if (mcr->result != QM_MCR_RESULT_OK)
			break; /* End of valid FQIDs */

		channel = mcr->queryfq.fqd.dest.channel;
		/* Determine the state of the FQID */
		mcc = qm_mc_start(portal);
		mcc->queryfq_np.fqid = fq_id;
		qm_mc_commit(portal, QM_MCC_VERB_QUERYFQ_NP);
		while (!(mcr = qm_mc_result(portal)))
			cpu_relax();
		DPA_ASSERT((mcr->verb & QM_MCR_VERB_MASK)
			   == QM_MCR_VERB_QUERYFQ_NP);
		state = mcr->queryfq_np.state & QM_MCR_NP_STATE_MASK;
		if (state == QM_MCR_NP_STATE_OOS)
			/* Already OOS, no need to do anymore checks */
			goto next;

		if (check_channel(ctx, channel))
			qm_shutdown_fq(&portal, 1, fq_id);
 next:
		++fq_id;
	}
	return 0;
}

static bool check_channel_device(void *_ctx, u32 channel)
{
	struct ctx *ctx = _ctx;
	struct portal_mapping *portal, *tmpportal;
	struct active_resource *res;

	/* See if the FQ is destined for one of the portals we're cleaning up */
	list_for_each_entry_safe(portal, tmpportal, &ctx->portals, list) {
		if (portal->user.type == usdpaa_portal_qman) {
			if (portal->qportal->public_cfg.channel == channel) {
				/* This FQs destination is a portal
				   we're cleaning, send a retire */
				return true;
			}
		}
	}

	/* Check the pool channels that will be released as well */
	list_for_each_entry(res, &ctx->resources[usdpaa_id_qpool], list) {
		if ((res->id >= channel) &&
		    ((res->id + res->num - 1) <= channel))
			return true;
	}
	return false;
}

static bool check_portal_channel(void *ctx, u32 channel)
{
	u32 portal_channel = *(u32 *)ctx;
	if (portal_channel == channel) {
		/* This FQs destination is a portal
		   we're cleaning, send a retire */
		return true;
	}
	return false;
}



static int usdpaa_release(struct inode *inode, struct file *filp)
{
	int err = 0;
	struct ctx *ctx = filp->private_data;
	struct mem_mapping *map, *tmpmap;
	struct portal_mapping *portal, *tmpportal;
	const struct alloc_backend *backend = &alloc_backends[0];
	struct active_resource *tmp, *res;
	struct qm_portal *qm_cleanup_portal = NULL;
	struct bm_portal *bm_cleanup_portal = NULL;
	struct qm_portal_config *qm_alloced_portal = NULL;
	struct bm_portal_config *bm_alloced_portal = NULL;
	struct eventfd_list *ev_mem, *tmp_ev_mem;

	struct qm_portal **portal_array;
	int portal_count = 0;

	portal_array = kmalloc_array(qman_portal_max,
				     sizeof(struct qm_portal *), GFP_KERNEL);
	if (!portal_array)
		return -ENOMEM;

	/* Ensure the release operation cannot be migrated to another
	   CPU as CPU specific variables may be needed during cleanup */
#ifdef CONFIG_PREEMPT_RT_FULL
	migrate_disable();
#endif
	/* The following logic is used to recover resources that were not
	   correctly released by the process that is closing the FD.
	   Step 1: syncronize the HW with the qm_portal/bm_portal structures
	   in the kernel
	*/

	list_for_each_entry_safe(portal, tmpportal, &ctx->portals, list) {
		/* Try to recover any portals that weren't shut down */
		if (portal->user.type == usdpaa_portal_qman) {
			portal_array[portal_count] = &portal->qman_portal_low;
			++portal_count;
			init_qm_portal(portal->qportal,
				       &portal->qman_portal_low);
			if (!qm_cleanup_portal) {
				qm_cleanup_portal = &portal->qman_portal_low;
			} else {
				/* Clean FQs on the dedicated channel */
				u32 chan = portal->qportal->public_cfg.channel;
				qm_check_and_destroy_fqs(
					&portal->qman_portal_low, &chan,
					check_portal_channel);
			}
		} else {
			/* BMAN */
			init_bm_portal(portal->bportal,
				       &portal->bman_portal_low);
			if (!bm_cleanup_portal)
				bm_cleanup_portal = &portal->bman_portal_low;
		}
	}
	/* If no portal was found, allocate one for cleanup */
	if (!qm_cleanup_portal) {
		qm_alloced_portal = qm_get_unused_portal();
		if (!qm_alloced_portal) {
			pr_crit("No QMan portal avalaible for cleanup\n");
			err = -1;
			goto done;
		}
		qm_cleanup_portal = kmalloc(sizeof(struct qm_portal),
					    GFP_KERNEL);
		if (!qm_cleanup_portal) {
			err = -ENOMEM;
			goto done;
		}
		init_qm_portal(qm_alloced_portal, qm_cleanup_portal);
		portal_array[portal_count] = qm_cleanup_portal;
		++portal_count;
	}
	if (!bm_cleanup_portal) {
		bm_alloced_portal = bm_get_unused_portal();
		if (!bm_alloced_portal) {
			pr_crit("No BMan portal avalaible for cleanup\n");
			err = -1;
			goto done;
		}
		bm_cleanup_portal = kmalloc(sizeof(struct bm_portal),
					    GFP_KERNEL);
		if (!bm_cleanup_portal) {
			err = -ENOMEM;
			goto done;
		}
		init_bm_portal(bm_alloced_portal, bm_cleanup_portal);
	}

	/* OOS the FQs associated with this process */
	qm_check_and_destroy_fqs(qm_cleanup_portal, ctx, check_channel_device);

	while (backend->id_type != usdpaa_id_max) {
		int res_count = 0;
		list_for_each_entry_safe(res, tmp, &ctx->resources[backend->id_type],
				    list) {
			if (backend->id_type == usdpaa_id_fqid) {
				int i = 0;
				for (; i < res->num; i++) {
					/* Clean FQs with the cleanup portal */
					qm_shutdown_fq(portal_array,
						       portal_count,
						       res->id + i);
				}
			}
			res_count += res->num;

			backend->release(res->id, res->num);
			list_del(&res->list);
			kfree(res);
		}
		if (res_count)
			pr_info("USDPAA release: %d %s%s\n",
				res_count, backend->acronym,
				(res_count > 1) ? "s" : "");
		backend++;
	}
	/* Release any DMA regions */
	spin_lock(&mem_lock);
	list_for_each_entry_safe(map, tmpmap, &ctx->maps, list) {
		struct mem_fragment *current_frag = map->root_frag;
		int i;
		if (map->root_frag->has_locking &&
		    (map->root_frag->owner == map)) {
			map->root_frag->owner = NULL;
			wake_up(&map->root_frag->wq);
		}
		/* Check each fragment and merge if the ref count is 0 */
		for (i = 0; i < map->frag_count; i++) {
			--current_frag->refs;
			current_frag = list_entry(current_frag->list.prev,
						  struct mem_fragment, list);
		}

		compress_frags();
		list_del(&map->list);
		kfree(map);
	}
	spin_unlock(&mem_lock);

	/* free eventfds */
	list_for_each_entry_safe(ev_mem, tmp_ev_mem, &ctx->events, ctx_list) {
		eventfd_ctx_put(ev_mem->efd_ctx);
		list_del(&ev_mem->ctx_list);
		list_del(&ev_mem->d_list);
		kfree(ev_mem);
	}

	/* Return portals */
	list_for_each_entry_safe(portal, tmpportal, &ctx->portals, list) {
		if (portal->user.type == usdpaa_portal_qman) {
			/* Give the portal back to the allocator */
			init_qm_portal(portal->qportal,
				       &portal->qman_portal_low);
			qm_put_unused_portal(portal->qportal);
		} else {
			init_bm_portal(portal->bportal,
				       &portal->bman_portal_low);
			bm_put_unused_portal(portal->bportal);
		}
		list_del(&portal->list);
		kfree(portal);
	}
	if (qm_alloced_portal) {
		qm_put_unused_portal(qm_alloced_portal);
		kfree(qm_cleanup_portal);
	}
	if (bm_alloced_portal) {
		bm_put_unused_portal(bm_alloced_portal);
		kfree(bm_cleanup_portal);
	}

	kfree(ctx);
done:
#ifdef CONFIG_PREEMPT_RT_FULL
	migrate_enable();
#endif
	kfree(portal_array);
	return err;
}

static int check_mmap_dma(struct ctx *ctx, struct vm_area_struct *vma,
			  int *match, unsigned long *pfn)
{
	struct mem_mapping *map;

	list_for_each_entry(map, &ctx->maps, list) {
		int i;
		struct mem_fragment *frag = map->root_frag;

		for (i = 0; i < map->frag_count; i++) {
			if (frag->pfn_base == vma->vm_pgoff) {
				*match = 1;
				*pfn = frag->pfn_base;
				return 0;
			}
			frag = list_entry(frag->list.next, struct mem_fragment,
					  list);
		}
	}
	*match = 0;
	return 0;
}

static int check_mmap_resource(struct resource *res, struct vm_area_struct *vma,
			       int *match, unsigned long *pfn)
{
	*pfn = res->start >> PAGE_SHIFT;
	if (*pfn == vma->vm_pgoff) {
		*match = 1;
		if ((vma->vm_end - vma->vm_start) != resource_size(res))
			return -EINVAL;
	} else
		*match = 0;
	return 0;
}

static int check_mmap_portal(struct ctx *ctx, struct vm_area_struct *vma,
			      int *match, unsigned long *pfn)
{
	struct portal_mapping *portal;
	int ret;

	list_for_each_entry(portal, &ctx->portals, list) {
		ret = check_mmap_resource(&portal->phys[DPA_PORTAL_CE], vma,
					  match, pfn);
		if (*match) {
			vma->vm_page_prot =
#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
				pgprot_cached_ns(vma->vm_page_prot);
#else
				pgprot_cached_noncoherent(vma->vm_page_prot);
#endif
			return ret;
		}
		ret = check_mmap_resource(&portal->phys[DPA_PORTAL_CI], vma,
					  match, pfn);
		if (*match) {
			vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
			return ret;
		}
	}
	*match = 0;
	return 0;
}

static int usdpaa_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct ctx *ctx = filp->private_data;
	unsigned long pfn = 0;
	int match, ret;

	spin_lock(&mem_lock);
	ret = check_mmap_dma(ctx, vma, &match, &pfn);
	if (!match)
		ret = check_mmap_portal(ctx, vma, &match, &pfn);
	spin_unlock(&mem_lock);
	if (!match)
		return -EINVAL;
	if (!ret)
		ret = remap_pfn_range(vma, vma->vm_start, pfn,
				      vma->vm_end - vma->vm_start,
				      vma->vm_page_prot);
	return ret;
}

/* Return the nearest rounded-up address >= 'addr' that is 'sz'-aligned. 'sz'
 * must be a power of 2, but both 'addr' and 'sz' can be expressions. */
#define USDPAA_MEM_ROUNDUP(addr, sz) \
	({ \
		unsigned long foo_align = (sz) - 1; \
		((addr) + foo_align) & ~foo_align; \
	})
/* Searching for a size-aligned virtual address range starting from 'addr' */
static unsigned long usdpaa_get_unmapped_area(struct file *file,
					      unsigned long addr,
					      unsigned long len,
					      unsigned long pgoff,
					      unsigned long flags)
{
	struct vm_area_struct *vma;

	/* Need to align the address to the largest pagesize of the mapping
	 * because the MMU requires the virtual address to have the same
	 * alignment as the physical address
	 */
	unsigned long align_addr = USDPAA_MEM_ROUNDUP(addr,
						      largest_page_size(len));
	VMA_ITERATOR(vmi, current->mm, align_addr);

	if (len % PAGE_SIZE)
		return -EINVAL;
	if (!len)
		return -EINVAL;

	/* Keep searching until we reach the end of currently-used virtual
	 * address-space or we find a big enough gap. */
	for_each_vma(vmi, vma) {
		if ((align_addr + len) < vma->vm_start)
			return align_addr;

		align_addr = USDPAA_MEM_ROUNDUP(vma->vm_end, largest_page_size(len));
	}
	if ((TASK_SIZE - len) < align_addr)
		return -ENOMEM;
	return align_addr;
}

static long ioctl_id_alloc(struct ctx *ctx, void __user *arg)
{
	struct usdpaa_ioctl_id_alloc i = {0};
	const struct alloc_backend *backend;
	struct active_resource *res;
	int ret = copy_from_user(&i, arg, sizeof(i));
	if (ret)
		return ret;
	if ((i.id_type >= usdpaa_id_max) || !i.num)
		return -EINVAL;
	backend = &alloc_backends[i.id_type];
	/* Allocate the required resource type */
	ret = backend->alloc(&i.base, i.num, i.align, i.partial);
	if (ret < 0)
		return ret;
	i.num = ret;
	/* Copy the result to user-space */
	ret = copy_to_user(arg, &i, sizeof(i));
	if (ret) {
		backend->release(i.base, i.num);
		return ret;
	}
	/* Assign the allocated range to the FD accounting */
	res = kmalloc(sizeof(*res), GFP_KERNEL);
	if (!res) {
		backend->release(i.base, i.num);
		return -ENOMEM;
	}
	spin_lock(&ctx->lock);
	res->id = i.base;
	res->num = i.num;
	res->refcount = 1;
	list_add(&res->list, &ctx->resources[i.id_type]);
	spin_unlock(&ctx->lock);
	return 0;
}

static long ioctl_id_release(struct ctx *ctx, void __user *arg)
{
	struct usdpaa_ioctl_id_release i;
	const struct alloc_backend *backend;
	struct active_resource *tmp, *pos;

	int ret = copy_from_user(&i, arg, sizeof(i));
	if (ret)
		return ret;
	if ((i.id_type >= usdpaa_id_max) || !i.num)
		return -EINVAL;
	backend = &alloc_backends[i.id_type];
	/* Pull the range out of the FD accounting - the range is valid iff this
	 * succeeds. */
	spin_lock(&ctx->lock);
	list_for_each_entry_safe(pos, tmp, &ctx->resources[i.id_type], list) {
		if (pos->id == i.base && pos->num == i.num) {
			pos->refcount--;
			if (pos->refcount) {
				spin_unlock(&ctx->lock);
				return 0; /* Still being used */
			}
			list_del(&pos->list);
			kfree(pos);
			spin_unlock(&ctx->lock);
			goto found;
		}
	}
	/* Failed to find the resource */
	spin_unlock(&ctx->lock);
	pr_err("Couldn't find resource type %d base 0x%x num %d\n",
	       i.id_type, i.base, i.num);
	return -EINVAL;
found:
	/* Release the resource to the backend */
	backend->release(i.base, i.num);
	return 0;
}

static long ioctl_id_reserve(struct ctx *ctx, void __user *arg)
{
	struct usdpaa_ioctl_id_reserve i = {0};
	const struct alloc_backend *backend;
	struct active_resource *tmp, *pos;

	int ret = copy_from_user(&i, arg, sizeof(i));
	if (ret)
		return ret;
	if ((i.id_type >= usdpaa_id_max) || !i.num)
		return -EINVAL;
	backend = &alloc_backends[i.id_type];
	if (!backend->reserve)
		return -EINVAL;
	/* Pull the range out of the FD accounting - the range is valid iff this
	 * succeeds. */
	spin_lock(&ctx->lock);
	list_for_each_entry_safe(pos, tmp, &ctx->resources[i.id_type], list) {
		if (pos->id == i.base && pos->num == i.num) {
			pos->refcount++;
			spin_unlock(&ctx->lock);
			return 0;
		}
	}

	/* Failed to find the resource */
	spin_unlock(&ctx->lock);

	/* Reserve the resource in the backend */
	ret = backend->reserve(i.base, i.num);
	if (ret)
		return ret;
	/* Assign the reserved range to the FD accounting */
	pos = kmalloc(sizeof(*pos), GFP_KERNEL);
	if (!pos) {
		backend->release(i.base, i.num);
		return -ENOMEM;
	}
	spin_lock(&ctx->lock);
	pos->id = i.base;
	pos->num = i.num;
	pos->refcount = 1;
	list_add(&pos->list, &ctx->resources[i.id_type]);
	spin_unlock(&ctx->lock);
	return 0;
}

static long ioctl_dma_map(struct file *fp, struct ctx *ctx,
			  struct usdpaa_ioctl_dma_map *i)
{
	struct mem_fragment *frag, *start_frag, *next_frag;
	struct mem_mapping *map, *tmp;
	int ret = 0;
	u32 largest_page, so_far = 0;
	int frag_count = 0;
	unsigned long next_addr = PAGE_SIZE, populate;

	map = kmalloc(sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	spin_lock(&mem_lock);

	/* error checking to ensure values copied from user space are valid */
	if (i->len % PAGE_SIZE) {
		spin_unlock(&mem_lock);
		kfree(map);
		return -EINVAL;
	}

	if (i->flags & USDPAA_DMA_FLAG_SHARE) {
		list_for_each_entry(frag, &mem_list, list) {
			if (frag->refs && (frag->flags &
					   USDPAA_DMA_FLAG_SHARE) &&
					!strncmp(i->name, frag->name,
						 USDPAA_DMA_NAME_MAX)) {
				/* Matching entry */
				if ((i->flags & USDPAA_DMA_FLAG_CREATE) &&
				    !(i->flags & USDPAA_DMA_FLAG_LAZY)) {
					ret = -EBUSY;
					goto out;
				}

				/* Check to ensure size matches record */
				if (i->len != frag->map_len && i->len) {
					pr_err("ioctl_dma_map() Size requested does not match %s and is none zero\n",
					frag->name);
					ret = -EINVAL;
					goto out;
				}

				/* Check if this has already been mapped
				   to this process */
				list_for_each_entry(tmp, &ctx->maps, list)
					if (tmp->root_frag == frag) {
						/* Already mapped, just need to
						   inc ref count */
						tmp->refs++;
						kfree(map);
						i->did_create = 0;
						i->len = tmp->total_size;
						i->phys_addr = frag->base;
						i->ptr = tmp->virt_addr;
						spin_unlock(&mem_lock);
						return 0;
					}
				/* Matching entry - just need to map */
				i->has_locking = frag->has_locking;
				i->did_create = 0;
				i->len = frag->map_len;
				start_frag = frag;
				goto do_map;
			}
		}
		/* No matching entry */
		if (!(i->flags & USDPAA_DMA_FLAG_CREATE)) {
			pr_err("ioctl_dma_map() No matching entry\n");
			ret = -ENOMEM;
			goto out;
		}
	}
	/* New fragment required, size must be provided. */
	if (!i->len) {
		ret = -EINVAL;
		goto out;
	}

	/* Find one of more contiguous fragments that satisfy the total length
	   trying to minimize the number of fragments
	   compute the largest page size that the allocation could use */
	largest_page = largest_page_size(i->len);
	start_frag = NULL;
	while (largest_page &&
	       largest_page <= largest_page_size(phys_size) &&
	       start_frag == NULL) {
		/* Search the list for a frag of that size */
		list_for_each_entry(frag, &mem_list, list) {
			if (!frag->refs && (frag->len == largest_page)) {
				/* See if the next x fragments are free
				   and can accomidate the size */
				u32 found_size = largest_page;
				next_frag = list_entry(frag->list.prev,
						       struct mem_fragment,
						       list);
				/* If the fragement is too small check
				   if the neighbours cab support it */
				while (found_size < i->len) {
					if (&mem_list == &next_frag->list)
						break; /* End of list */
					if (next_frag->refs != 0 ||
					    next_frag->len == 0)
						break; /* not enough space */
					found_size += next_frag->len;
					next_frag = list_entry(
						next_frag->list.prev,
						struct mem_fragment,
						list);
				}
				if (found_size >= i->len) {
					/* Success! there is enough contigous
					   free space */
					start_frag = frag;
					break;
				}
			}
		} /* next frag loop */
		/* Couldn't statisfy the request with this
		   largest page size, try a smaller one */
		largest_page <<= 2;
	}
	if (start_frag == NULL) {
		/* Couldn't find proper amount of space */
		ret = -ENOMEM;
		goto out;
	}
	i->did_create = 1;
do_map:
	/* Verify there is sufficient space to do the mapping */
	down_write(&current->mm->mmap_lock);
	next_addr = usdpaa_get_unmapped_area(fp, next_addr, i->len, 0, 0);
	up_write(&current->mm->mmap_lock);

	if (next_addr & ~PAGE_MASK) {
		ret = -ENOMEM;
		goto out;
	}

	/* We may need to divide the final fragment to accomidate the mapping */
	next_frag = start_frag;
	while (so_far != i->len) {
		BUG_ON(next_frag->len == 0);
		while ((next_frag->len + so_far) > i->len) {
			/* Split frag until they match */
			split_frag(next_frag);
		}
		so_far += next_frag->len;
		next_frag->refs++;
		++frag_count;
		next_frag = list_entry(next_frag->list.prev,
				       struct mem_fragment, list);
	}
	if (i->did_create) {
		size_t name_len = 0;
		start_frag->flags = i->flags;
		strncpy(start_frag->name, i->name, USDPAA_DMA_NAME_MAX);
		start_frag->name[USDPAA_DMA_NAME_MAX - 1] = '\0';
		name_len = strnlen(start_frag->name, USDPAA_DMA_NAME_MAX);
		if (name_len >= USDPAA_DMA_NAME_MAX) {
			ret = -EFAULT;
			goto out;
		}
		start_frag->map_len = i->len;
		start_frag->has_locking = i->has_locking;
		init_waitqueue_head(&start_frag->wq);
		start_frag->owner = NULL;
	}

	/* Setup the map entry */
	map->root_frag = start_frag;
	map->total_size = i->len;
	map->frag_count = frag_count;
	map->refs = 1;
	list_add(&map->list, &ctx->maps);
	i->phys_addr = start_frag->base;
out:
	spin_unlock(&mem_lock);

	if (!ret) {
		unsigned long longret;
		down_write(&current->mm->mmap_lock);
		longret = do_mmap(fp, next_addr, map->total_size,
					PROT_READ |
					(i->flags &
					 USDPAA_DMA_FLAG_RDONLY ? 0
					 : PROT_WRITE),
					MAP_SHARED,
					0,
					start_frag->pfn_base,
					&populate,
					NULL);
		up_write(&current->mm->mmap_lock);
		if (longret & ~PAGE_MASK) {
			ret = (int)longret;
		} else {
			i->ptr = (void *)longret;
			map->virt_addr = i->ptr;
		}
	} else
		kfree(map);
	return ret;
}

static long ioctl_dma_unmap(struct ctx *ctx, void __user *arg)
{
	struct mem_mapping *map;
	struct vm_area_struct *vma;
	int ret, i;
	struct mem_fragment *current_frag;
	size_t sz;
	unsigned long base;

	down_write(&current->mm->mmap_lock);
	vma = find_vma(current->mm, (unsigned long)arg);
	if (!vma || (vma->vm_start > (unsigned long)arg)) {
		up_write(&current->mm->mmap_lock);
		return -EFAULT;
	}
	spin_lock(&mem_lock);
	list_for_each_entry(map, &ctx->maps, list) {
		if (map->root_frag->pfn_base == vma->vm_pgoff) {
			/* Drop the map lock if we hold it */
			if (map->root_frag->has_locking &&
					(map->root_frag->owner == map)) {
				map->root_frag->owner = NULL;
				wake_up(&map->root_frag->wq);
			}
			goto map_match;
		}
	}
	/* Failed to find a matching mapping for this process */
	ret = -EFAULT;
	spin_unlock(&mem_lock);
	goto out;
map_match:
	map->refs--;
	if (map->refs != 0) {
		/* Another call the dma_map is referencing this */
		ret = 0;
		spin_unlock(&mem_lock);
		goto out;
	}

	current_frag = map->root_frag;
	for (i = 0; i < map->frag_count; i++) {
		DPA_ASSERT(current_frag->refs > 0);
		--current_frag->refs;
		current_frag = list_entry(current_frag->list.prev,
					  struct mem_fragment, list);
	}
	map->root_frag->name[0] = 0;
	list_del(&map->list);
	compress_frags();
	spin_unlock(&mem_lock);

	base = vma->vm_start;
	sz = vma->vm_end - vma->vm_start;
	do_munmap(current->mm, base, sz, NULL);
	ret = 0;
 out:
	up_write(&current->mm->mmap_lock);
	return ret;
}

static long ioctl_dma_stats(struct ctx *ctx, void __user *arg)
{
	struct mem_fragment *frag;
	struct usdpaa_ioctl_dma_used result;

	result.free_bytes = 0;
	result.total_bytes = phys_size;

	list_for_each_entry(frag, &mem_list, list) {
		if (frag->refs == 0)
			result.free_bytes += frag->len;
	}

	return copy_to_user(arg, &result, sizeof(result)); }

static int test_lock(struct mem_mapping *map)
{
	int ret = 0;
	spin_lock(&mem_lock);
	if (!map->root_frag->owner) {
		map->root_frag->owner = map;
		ret = 1;
	}
	spin_unlock(&mem_lock);
	return ret;
}

static long ioctl_dma_lock(struct ctx *ctx, void __user *arg)
{
	struct mem_mapping *map;
	struct vm_area_struct *vma;

	spin_lock(&mem_lock);
	down_read(&current->mm->mmap_lock);
	vma = find_vma(current->mm, (unsigned long)arg);
	if (!vma || (vma->vm_start > (unsigned long)arg)) {
		up_read(&current->mm->mmap_lock);
		spin_unlock(&mem_lock);
		return -EFAULT;
	}
	list_for_each_entry(map, &ctx->maps, list) {
		if (map->root_frag->pfn_base == vma->vm_pgoff)
			goto map_match;
	}
	map = NULL;
map_match:
	up_read(&current->mm->mmap_lock);
	spin_unlock(&mem_lock);

	if (!map)
		return -EFAULT;
	if (!map->root_frag->has_locking)
		return -ENODEV;
	return wait_event_interruptible(map->root_frag->wq, test_lock(map));
}

static long ioctl_dma_unlock(struct ctx *ctx, void __user *arg)
{
	struct mem_mapping *map;
	struct vm_area_struct *vma;
	int ret;

	spin_lock(&mem_lock);
	down_read(&current->mm->mmap_lock);
	vma = find_vma(current->mm, (unsigned long)arg);
	if (!vma || (vma->vm_start > (unsigned long)arg))
		ret = -EFAULT;
	else {
		list_for_each_entry(map, &ctx->maps, list) {
			if (map->root_frag->pfn_base == vma->vm_pgoff) {
				if (!map->root_frag->has_locking)
					ret = -ENODEV;
				else if (map->root_frag->owner == map) {
					map->root_frag->owner = NULL;
					wake_up(&map->root_frag->wq);
					ret = 0;
				} else
					ret = -EBUSY;
				goto map_match;
			}
		}
		ret = -EINVAL;
	}
map_match:
	up_read(&current->mm->mmap_lock);
	spin_unlock(&mem_lock);
	return ret;
}

static int portal_mmap(struct file *fp, struct resource *res, void **ptr)
{
	unsigned long longret = 0, populate;
	resource_size_t len;

	down_write(&current->mm->mmap_lock);
	len = resource_size(res);
	if (len != (unsigned long)len)
		return -EINVAL;
	longret = do_mmap(fp, PAGE_SIZE, (unsigned long)len,
				PROT_READ | PROT_WRITE, MAP_SHARED, 0,
				res->start >> PAGE_SHIFT, &populate, NULL);
	up_write(&current->mm->mmap_lock);

	if (longret & ~PAGE_MASK)
		return (int)longret;

	*ptr = (void *) longret;
	return 0;
}

static void portal_munmap(struct resource *res, void  *ptr)
{
	down_write(&current->mm->mmap_lock);
	do_munmap(current->mm, (unsigned long)ptr, resource_size(res), NULL);
	up_write(&current->mm->mmap_lock);
}

static long ioctl_portal_map(struct file *fp, struct ctx *ctx,
			     struct usdpaa_ioctl_portal_map  *arg)
{
	struct portal_mapping *mapping = kmalloc(sizeof(*mapping), GFP_KERNEL);
	int ret;

	if (!mapping)
		return -ENOMEM;

	mapping->user = *arg;
	mapping->iommu_domain = NULL;

	if (mapping->user.type == usdpaa_portal_qman) {
		mapping->qportal =
			qm_get_unused_portal_idx(mapping->user.index);
		if (!mapping->qportal) {
			ret = -ENODEV;
			goto err_get_portal;
		}
		mapping->phys = &mapping->qportal->addr_phys[0];
		mapping->user.channel = mapping->qportal->public_cfg.channel;
		mapping->user.pools = mapping->qportal->public_cfg.pools;
		mapping->user.index = mapping->qportal->public_cfg.index;
	} else if (mapping->user.type == usdpaa_portal_bman) {
		mapping->bportal =
			bm_get_unused_portal_idx(mapping->user.index);
		if (!mapping->bportal) {
			ret = -ENODEV;
			goto err_get_portal;
		}
		mapping->phys = &mapping->bportal->addr_phys[0];
		mapping->user.index = mapping->bportal->public_cfg.index;
	} else {
		ret = -EINVAL;
		goto err_copy_from_user;
	}
	/* Need to put pcfg in ctx's list before the mmaps because the mmap
	 * handlers look it up. */
	spin_lock(&mem_lock);
	list_add(&mapping->list, &ctx->portals);
	spin_unlock(&mem_lock);
	ret = portal_mmap(fp, &mapping->phys[DPA_PORTAL_CE],
			  &mapping->user.addr.cena);
	if (ret)
		goto err_mmap_cena;
	ret = portal_mmap(fp, &mapping->phys[DPA_PORTAL_CI],
			  &mapping->user.addr.cinh);
	if (ret)
		goto err_mmap_cinh;
	*arg = mapping->user;
	return ret;

err_mmap_cinh:
	portal_munmap(&mapping->phys[DPA_PORTAL_CE], mapping->user.addr.cena);
err_mmap_cena:
	if ((mapping->user.type == usdpaa_portal_qman) && mapping->qportal)
		qm_put_unused_portal(mapping->qportal);
	else if ((mapping->user.type == usdpaa_portal_bman) && mapping->bportal)
		bm_put_unused_portal(mapping->bportal);
	spin_lock(&mem_lock);
	list_del(&mapping->list);
	spin_unlock(&mem_lock);
err_get_portal:
err_copy_from_user:
	kfree(mapping);
	return ret;
}

static long ioctl_portal_unmap(struct ctx *ctx, struct usdpaa_portal_map *i)
{
	struct portal_mapping *mapping;
	struct vm_area_struct *vma;
	unsigned long pfn;
	u32 channel;

	/* Get the PFN corresponding to one of the virt addresses */
	down_read(&current->mm->mmap_lock);
	vma = find_vma(current->mm, (unsigned long)i->cinh);
	if (!vma || (vma->vm_start > (unsigned long)i->cinh)) {
		up_read(&current->mm->mmap_lock);
		return -EFAULT;
	}
	pfn = vma->vm_pgoff;
	up_read(&current->mm->mmap_lock);

	/* Find the corresponding portal */
	spin_lock(&mem_lock);
	list_for_each_entry(mapping, &ctx->portals, list) {
		if (pfn == (mapping->phys[DPA_PORTAL_CI].start >> PAGE_SHIFT))
			goto found;
	}
	mapping = NULL;
found:
	if (mapping)
		list_del(&mapping->list);
	spin_unlock(&mem_lock);
	if (!mapping)
		return -ENODEV;
	portal_munmap(&mapping->phys[DPA_PORTAL_CI], mapping->user.addr.cinh);
	portal_munmap(&mapping->phys[DPA_PORTAL_CE], mapping->user.addr.cena);
	if (mapping->user.type == usdpaa_portal_qman) {
		init_qm_portal(mapping->qportal,
				       &mapping->qman_portal_low);

		/* Tear down any FQs this portal is referencing */
		channel = mapping->qportal->public_cfg.channel;
		qm_check_and_destroy_fqs(&mapping->qman_portal_low,
					 &channel,
					 check_portal_channel);
		qm_put_unused_portal(mapping->qportal);
	} else if (mapping->user.type == usdpaa_portal_bman) {
		init_bm_portal(mapping->bportal,
			       &mapping->bman_portal_low);
		bm_put_unused_portal(mapping->bportal);
	}
	kfree(mapping);
	return 0;
}

static void portal_config_pamu(struct qm_portal_config *pcfg, uint8_t sdest,
			       uint32_t cpu, uint32_t cache, uint32_t window)
{
#ifdef CONFIG_FSL_QMAN_CONFIG
	if (qman_set_sdest(pcfg->public_cfg.channel, sdest))
#endif
		pr_warn("Failed to set QMan portal's stash request queue\n");
}

static long ioctl_allocate_raw_portal(struct file *fp, struct ctx *ctx,
				      struct usdpaa_ioctl_raw_portal *arg)
{
	struct portal_mapping *mapping = kmalloc(sizeof(*mapping), GFP_KERNEL);
	int ret;

	if (!mapping)
		return -ENOMEM;

	mapping->user.type = arg->type;
	mapping->iommu_domain = NULL;
	if (arg->type == usdpaa_portal_qman) {
		mapping->qportal = qm_get_unused_portal_idx(arg->index);
		if (!mapping->qportal) {
			ret = -ENODEV;
			goto err;
		}
		mapping->phys = &mapping->qportal->addr_phys[0];
		arg->index = mapping->qportal->public_cfg.index;
		arg->cinh = mapping->qportal->addr_phys[DPA_PORTAL_CI].start;
		arg->cena = mapping->qportal->addr_phys[DPA_PORTAL_CE].start;
		if (arg->enable_stash) {
			/* Setup the PAMU with the supplied parameters */
			portal_config_pamu(mapping->qportal, arg->sdest,
					   arg->cpu, arg->cache, arg->window);
		}
	} else if (mapping->user.type == usdpaa_portal_bman) {
		mapping->bportal =
			bm_get_unused_portal_idx(arg->index);
		if (!mapping->bportal) {
			ret = -ENODEV;
			goto err;
		}
		mapping->phys = &mapping->bportal->addr_phys[0];
		arg->index = mapping->bportal->public_cfg.index;
		arg->cinh = mapping->bportal->addr_phys[DPA_PORTAL_CI].start;
		arg->cena = mapping->bportal->addr_phys[DPA_PORTAL_CE].start;
	} else {
		ret = -EINVAL;
		goto err;
	}
	/* Need to put pcfg in ctx's list before the mmaps because the mmap
	 * handlers look it up. */
	spin_lock(&mem_lock);
	list_add(&mapping->list, &ctx->portals);
	spin_unlock(&mem_lock);
	return 0;
err:
	kfree(mapping);
	return ret;
}

static long ioctl_free_raw_portal(struct file *fp, struct ctx *ctx,
				      struct usdpaa_ioctl_raw_portal *arg)
{
	struct portal_mapping *mapping;
	u32 channel;

	/* Find the corresponding portal */
	spin_lock(&mem_lock);
	list_for_each_entry(mapping, &ctx->portals, list) {
		if (mapping->phys[DPA_PORTAL_CI].start == arg->cinh)
			goto found;
	}
	mapping = NULL;
found:
	if (mapping)
		list_del(&mapping->list);
	spin_unlock(&mem_lock);
	if (!mapping)
		return -ENODEV;
	if (mapping->user.type == usdpaa_portal_qman) {
		init_qm_portal(mapping->qportal,
				       &mapping->qman_portal_low);

		/* Tear down any FQs this portal is referencing */
		channel = mapping->qportal->public_cfg.channel;
		qm_check_and_destroy_fqs(&mapping->qman_portal_low,
					 &channel,
					 check_portal_channel);
		qm_put_unused_portal(mapping->qportal);
	} else if (mapping->user.type == usdpaa_portal_bman) {
		init_bm_portal(mapping->bportal,
			       &mapping->bman_portal_low);
		bm_put_unused_portal(mapping->bportal);
	}
	kfree(mapping);
	return 0;
}


static inline struct device *get_dev_ptr(char *if_name)
{
	struct device *dev;
	char node[NODE_NAME_LEN];

	sprintf(node, "soc:fsl,dpaa:%s",if_name);
	dev = bus_find_device_by_name(&platform_bus_type, NULL, node);
	if (dev == NULL) {
		pr_err(KBUILD_MODNAME "IF %s not found\n", if_name);
		return NULL;
	}
	pr_debug("%s: found dev 0x%lX  for If %s ,dev->platform_data %p\n",
			  __func__, (unsigned long)dev,
			  if_name, dev->platform_data);

	return dev;
}

static int ioctl_set_link_status(struct usdpaa_ioctl_update_link_status *args)
{
	struct device *dev;
	struct mac_device *mac_dev;

	dev = get_dev_ptr(args->if_name);
	if (!dev)
		return -ENODEV;

	if (of_device_is_compatible(dev->of_node, "fsl,dpa-ethernet")) {
		struct net_device *ndev;
		struct dpa_priv_s *npriv = NULL;

		ndev = dev_get_drvdata(dev);
		npriv = netdev_priv(ndev);
		mac_dev =  npriv->mac_dev;
	} else if (of_device_is_compatible(dev->of_node, "fsl,dpa-ethernet-init")) {
		struct proxy_device *proxy_dev;

		proxy_dev = dev_get_drvdata(dev);
		mac_dev =  proxy_dev->mac_dev;
	} else {
		pr_err(KBUILD_MODNAME "Not supported device\n");
		return -ENOMEM;
	}

	if (!mac_dev->phy_dev) {
		pr_err(KBUILD_MODNAME "Device does not have associated phy dev\n");
		return -EINVAL;
	}

	if (args->set_link_status == ETH_LINK_UP)
		phy_resume(mac_dev->phy_dev);
	else if (args->set_link_status == ETH_LINK_DOWN)
		phy_suspend(mac_dev->phy_dev);
	else
		return -EINVAL;
	return 0;
}

static int ioctl_set_link_speed(struct usdpaa_ioctl_update_link_speed *args)
{
	struct device *dev;
	struct mac_device *mac_dev;

	dev = get_dev_ptr(args->if_name);
	if (!dev)
		return -ENODEV;

	if (of_device_is_compatible(dev->of_node, "fsl,dpa-ethernet")) {
		struct net_device *ndev;
		struct dpa_priv_s *npriv = NULL;

		ndev = dev_get_drvdata(dev);
		npriv = netdev_priv(ndev);
		mac_dev =  npriv->mac_dev;
	} else if (of_device_is_compatible(dev->of_node, "fsl,dpa-ethernet-init")) {
		struct proxy_device *proxy_dev;

		proxy_dev = dev_get_drvdata(dev);
		mac_dev =  proxy_dev->mac_dev;
	} else {
		pr_err(KBUILD_MODNAME "Not supported device\n");
		return -ENOMEM;
	}

	if (!mac_dev->phy_dev) {
		pr_err(KBUILD_MODNAME "Device does not have associated phy dev\n");
		return -EINVAL;
	}

	mac_dev->phy_dev->speed = args->link_speed;
	mac_dev->phy_dev->duplex = args->link_duplex;
	mac_dev->phy_dev->autoneg = AUTONEG_DISABLE;

	return phy_start_aneg(mac_dev->phy_dev);
}

static int ioctl_link_restart_autoneg(char *if_name)
{
	struct device *dev;
	struct mac_device *mac_dev;

	dev = get_dev_ptr(if_name);
	if (!dev)
		return -ENODEV;

	if (of_device_is_compatible(dev->of_node, "fsl,dpa-ethernet")) {
		struct net_device *ndev;
		struct dpa_priv_s *npriv = NULL;

		ndev = dev_get_drvdata(dev);
		npriv = netdev_priv(ndev);
		mac_dev =  npriv->mac_dev;
	} else if (of_device_is_compatible(dev->of_node, "fsl,dpa-ethernet-init")) {
		struct proxy_device *proxy_dev;

		proxy_dev = dev_get_drvdata(dev);
		mac_dev =  proxy_dev->mac_dev;
	} else {
		pr_err(KBUILD_MODNAME "Not supported device\n");
		return -ENOMEM;
	}

	if (!mac_dev->phy_dev) {
		pr_err(KBUILD_MODNAME "Device does not have associated phy dev\n");
		return -EINVAL;
	}

	mac_dev->phy_dev->autoneg = AUTONEG_ENABLE;

	return phy_restart_aneg(mac_dev->phy_dev);
}

/* This function will return Current link status, link speed, link duplex and
 * link autoneg status of the device.
 *
 * Input parameter:
 * input->if_name: Interface node name
 *
 */
static inline int
ioctl_usdpaa_get_link_status(struct usdpaa_ioctl_link_status_args *input)
{
	struct net_device *net_dev = NULL;
	struct device *dev;

	dev = get_dev_ptr(input->if_name);
	if (dev == NULL)
		return -ENODEV;

	if (of_device_is_compatible(dev->of_node, "fsl,dpa-ethernet"))
		net_dev = dev_get_drvdata(dev);
	else
		net_dev = dev->platform_data;

	if (net_dev == NULL)
		return -ENODEV;

	if (net_dev->phydev == NULL) { /* Interface is down from kernel */
		input->link_status = ETH_LINK_DOWN;
	} else {
		input->link_status = netif_carrier_ok(net_dev);
		input->link_autoneg = net_dev->phydev->autoneg;
		input->link_duplex = net_dev->phydev->duplex;
	}

	if (input->link_status)
		input->link_speed = net_dev->phydev->speed;
	else
		input->link_speed = 0;

	return 0;
}


/* Link Status Callback Function
 * This function will be resgitered to PHY framework to get
 * Link update notifications and should be responsible for waking up
 * user space task when there is a link update notification.
 */
static void phy_link_updates(struct net_device *net_dev)
{
	struct list_head *position = NULL;
	struct eventfd_list  *ev_mem = NULL;
	struct dpa_priv_s *npriv = NULL;
	struct mac_device *mac_dev;
	struct phy_device *phy_dev;

	if (!net_dev->phydev) {
		npriv = netdev_priv(net_dev);
		mac_dev =  npriv->mac_dev;
		phy_dev = of_phy_find_device(mac_dev->phy_node);
	} else {
		phy_dev = net_dev->phydev;
	}

	list_for_each(position, &eventfd_head) {
		ev_mem = list_entry(position, struct eventfd_list, d_list);
		if (ev_mem->ndev == net_dev) {
			eventfd_signal(ev_mem->efd_ctx);
			pr_debug("%s: Link '%s': Speed '%d-Mbps': Autoneg '%d': Duplex '%d'\n",
				net_dev->name,
				netif_carrier_ok(net_dev) ? "UP" : "DOWN",
				phy_dev->speed,
				phy_dev->autoneg,
				phy_dev->duplex);
			return;
		}
	}

	pr_err(KBUILD_MODNAME "Device not registered for link events\n");
}

static int setup_eventfd(struct task_struct *userspace_task,
			struct usdpaa_ioctl_link_status *args,
			struct net_device *net_dev, struct ctx *ctx)
{
	struct file *efd_file = NULL;
	struct list_head *position = NULL;
	struct eventfd_list *ev_mem;

	rcu_read_lock();
	efd_file = lookup_fdget_rcu(args->efd);
	rcu_read_unlock();

	/* check if device is already registered */
	list_for_each(position, &eventfd_head) {
		ev_mem = list_entry(position, struct eventfd_list, d_list);
		if (ev_mem->ndev == net_dev) {
			ev_mem->efd_ctx = eventfd_ctx_fileget(efd_file);
			if (ev_mem->efd_ctx == NULL) {
				pr_err(KBUILD_MODNAME "get eventfd context failed\n");
				return -EINVAL;
			}
			return 0;
		}
	}

	ev_mem = kmalloc(sizeof(*ev_mem), GFP_KERNEL);
	if (!ev_mem) {
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&ev_mem->d_list);
	INIT_LIST_HEAD(&ev_mem->ctx_list);

	ev_mem->ndev = net_dev;
	ev_mem->efd_ctx = eventfd_ctx_fileget(efd_file);
	if (ev_mem->efd_ctx == NULL) {
	pr_err(KBUILD_MODNAME "get eventfd context failed\n");
		kfree(ev_mem);
		return -EINVAL;
	}
	list_add(&ev_mem->ctx_list, &ctx->events);
	list_add(&ev_mem->d_list, &eventfd_head);

	return 0;
}

/* IOCTL handler for enabling Link status request for a given interface
 * Input parameters:
 * args->if_name:	This the network interface node name as defind in
 *			device tree file. Currently, it has format of
 *			"ethernet@x" type for each interface.
 * args->efd:		The eventfd value which should be waked up when
 *			there is any link update received.
 */
static int ioctl_en_if_link_status(struct usdpaa_ioctl_link_status *args, struct ctx *ctx)
{
	struct net_device *net_dev = NULL;
	struct dpa_proxy_priv_s *priv = NULL;
	struct device *dev;
	struct mac_device *mac_dev;
	struct task_struct *userspace_task = NULL;
	int ret = 0;

	dev = get_dev_ptr(args->if_name);
	if (dev == NULL)
		return -ENODEV;

	if (of_device_is_compatible(dev->of_node, "fsl,dpa-ethernet")) {
		struct dpa_priv_s *npriv = NULL;
		struct phy_device *phy;

		net_dev = dev_get_drvdata(dev);
		npriv = netdev_priv(net_dev);
		mac_dev =  npriv->mac_dev;
		phy = of_phy_find_device(mac_dev->phy_node);
		phy->adjust_link = phy_link_updates;
		/* Get current task context from which IOCTL was called */
		userspace_task = current;

		ret = setup_eventfd(userspace_task, args, net_dev, ctx);
		if (ret)
			return ret;

		/* Since there will be NO PHY update as link is already setup,
		 * wake user context once so that current PHY status can
		 * be fetched.
		 */
		phy_link_updates(net_dev);

	} else if (of_device_is_compatible(dev->of_node, "fsl,dpa-ethernet-init")) {
		struct proxy_device *proxy_dev;

		/* Utilize dev->platform_data to save netdevice
		 * pointer as it will not be registered.
		 */
		if (dev->platform_data) {
			pr_debug("%s: IF %s already initialized\n",
				 __func__, args->if_name);
			/* This will happen when application is not able to initiate
			 * cleanup in last run. We still need to save the new
			 * eventfd context.
			 */
			net_dev = dev->platform_data;
			priv = netdev_priv(net_dev);

			/* Get current task context from which IOCTL was called */
			userspace_task = current;

			ret = setup_eventfd(userspace_task, args, net_dev, ctx);
			if (ret) {
				dev->platform_data = NULL;
				free_netdev(net_dev);
				return ret;
			}
			/* Since there will be NO PHY update as link is already setup,
			 * wake user context once so that current PHY status can
			 * be fetched.
			 */
			phy_link_updates(net_dev);
			return 0;
		}

		proxy_dev = dev_get_drvdata(dev);
		mac_dev =  proxy_dev->mac_dev;

		/* Allocate an dummy net device for proxy interface */
		net_dev = alloc_etherdev(sizeof(*priv));
		if (!net_dev) {
			pr_err(KBUILD_MODNAME "alloc_etherdev failed\n");
			return -ENOMEM;
		}
		SET_NETDEV_DEV(net_dev, dev);
		priv = netdev_priv(net_dev);
		priv->mac_dev = mac_dev;
		/* Get current task context from which IOCTL was called */
		userspace_task = current;

		ret = setup_eventfd(userspace_task, args, net_dev, ctx);
		if (ret) {
			free_netdev(net_dev);
			return ret;
		}
		strncpy(net_dev->name, args->if_name, IF_NAME_MAX_LEN);
		dev->platform_data = net_dev;

		/* FIXME: This is duplicated from memac_init_phy().
		 * The internal connection to the serdes is XGMII, but this isn't
		 * really correct for the phy mode (which is the external connection).
		 * However, this is how all older device trees say that they want
		 * 10GBASE-R (aka XFI), so just convert it for them.
		 */
		if (mac_dev->phy_if == PHY_INTERFACE_MODE_XGMII)
			mac_dev->phy_if = PHY_INTERFACE_MODE_10GBASER;

		pr_debug("%s: mac_dev %p cell_index %d\n",
			 __func__, mac_dev, mac_dev->cell_index);
		mac_dev->phy_dev = of_phy_connect(net_dev, mac_dev->phy_node,
						  phy_link_updates, 0,
						  mac_dev->phy_if);
		if (unlikely(mac_dev->phy_dev == NULL) || IS_ERR(mac_dev->phy_dev)) {
			pr_err("%s: --------Error in PHY Connect\n", __func__);
			/* Free the allocated memory for net device */
			free_netdev(net_dev);
			return -ENODEV;
		}
		pr_debug("%s: --- PHY connected for %s\n", __func__, args->if_name);
		net_dev->phydev = mac_dev->phy_dev;
		mac_dev->start(mac_dev);
	} else {
		pr_err(KBUILD_MODNAME "Not supported device\n");
		return -ENOMEM;
	}

	return 0;
}

/* IOCTL handler for disabling Link status for a given interface
 * Input parameters:
 * if_name:	This the network interface node name as defind in
 *		device tree file. Currently, it has format of
 *		"ethernet@x" type for each interface.
 */
static int ioctl_disable_if_link_status(char *if_name)
{
	struct net_device *net_dev = NULL;
	struct device *dev;
	struct mac_device *mac_dev;
	struct proxy_device *proxy_dev;
	struct list_head *position = NULL;
	struct eventfd_list  *ev_mem = NULL;

	dev = get_dev_ptr(if_name);
	if (dev == NULL)
		return -ENODEV;

	if (of_device_is_compatible(dev->of_node, "fsl,dpa-ethernet")) {
		net_dev = dev_get_drvdata(dev);
		list_for_each(position, &eventfd_head) {
			ev_mem = list_entry(position, struct eventfd_list, d_list);
			if (ev_mem->ndev == net_dev) {
				eventfd_ctx_put(ev_mem->efd_ctx);
				list_del(&ev_mem->ctx_list);
				list_del(position);
				kfree(ev_mem);
				break;
			}
		}

		return 0;
	}

	/* Utilize dev->platform_data to save netdevice
	   pointer as it will not be registered */
	if (!dev->platform_data) {
		pr_debug("%s: IF %s already Disabled for Link status\n",
			__func__, if_name);
		return 0;
	}

	net_dev = dev->platform_data;
	proxy_dev = dev_get_drvdata(dev);
	mac_dev =  proxy_dev->mac_dev;
	mac_dev->stop(mac_dev);

	/* free the memory for eventfd */
	list_for_each(position, &eventfd_head) {
		ev_mem = list_entry(position, struct eventfd_list, d_list);
		if (ev_mem->ndev == net_dev) {
			eventfd_ctx_put(ev_mem->efd_ctx);
			list_del(&ev_mem->ctx_list);
			list_del(&ev_mem->d_list);
			kfree(ev_mem);
			break;
		}
	}
	/* This will also deregister the call back */
	phy_disconnect(mac_dev->phy_dev);
	phy_resume(mac_dev->phy_dev);

	free_netdev(net_dev);
	dev->platform_data = NULL;

	pr_debug("%s: Link status Disabled for %s\n", __func__, if_name);
	return 0;
}

static long usdpaa_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	struct ctx *ctx = fp->private_data;
	void __user *a = (void __user *)arg;
	switch (cmd) {
	case USDPAA_IOCTL_ID_ALLOC:
		return ioctl_id_alloc(ctx, a);
	case USDPAA_IOCTL_ID_RELEASE:
		return ioctl_id_release(ctx, a);
	case USDPAA_IOCTL_ID_RESERVE:
		return ioctl_id_reserve(ctx, a);
	case USDPAA_IOCTL_DMA_MAP:
	{
		struct usdpaa_ioctl_dma_map input = {0};
		int ret;
		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		ret = ioctl_dma_map(fp, ctx, &input);
		if (copy_to_user(a, &input, sizeof(input)))
			return -EFAULT;
		return ret;
	}
	case USDPAA_IOCTL_DMA_UNMAP:
		return ioctl_dma_unmap(ctx, a);
	case USDPAA_IOCTL_DMA_LOCK:
		return ioctl_dma_lock(ctx, a);
	case USDPAA_IOCTL_DMA_UNLOCK:
		return ioctl_dma_unlock(ctx, a);
	case USDPAA_IOCTL_PORTAL_MAP:
	{
		struct usdpaa_ioctl_portal_map input;
		int ret;
		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		ret =  ioctl_portal_map(fp, ctx, &input);
		if (copy_to_user(a, &input, sizeof(input)))
			return -EFAULT;
		return ret;
	}
	case USDPAA_IOCTL_PORTAL_UNMAP:
	{
		struct usdpaa_portal_map input;
		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		return ioctl_portal_unmap(ctx, &input);
	}
	case USDPAA_IOCTL_DMA_USED:
		return ioctl_dma_stats(ctx, a);
	case USDPAA_IOCTL_ALLOC_RAW_PORTAL:
	{
		struct usdpaa_ioctl_raw_portal input;
		int ret;
		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		ret = ioctl_allocate_raw_portal(fp, ctx, &input);
		if (copy_to_user(a, &input, sizeof(input)))
			return -EFAULT;
		return ret;
	}
	case USDPAA_IOCTL_FREE_RAW_PORTAL:
	{
		struct usdpaa_ioctl_raw_portal input;
		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		return ioctl_free_raw_portal(fp, ctx, &input);
	}
	case USDPAA_IOCTL_ENABLE_LINK_STATUS_INTERRUPT:
	{
		struct usdpaa_ioctl_link_status input = {0};
		int ret;

		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		input.if_name[IF_NAME_MAX_LEN - 1] = '\0';

		ret = ioctl_en_if_link_status(&input, ctx);
		if (ret)
			pr_err("Error(%d) enable link interrupt:IF: %s\n",
				ret, input.if_name);
		return ret;
	}
	case USDPAA_IOCTL_DISABLE_LINK_STATUS_INTERRUPT:
	{
		char if_name[IF_NAME_MAX_LEN];
		int ret;

		if (copy_from_user(&if_name, a, sizeof(if_name)))
			return -EFAULT;
		if_name[IF_NAME_MAX_LEN - 1] = '\0';

		ret = ioctl_disable_if_link_status(if_name);
		if (ret)
			pr_err("Error(%d) Disabling link interrupt:IF: %s\n",
				ret, if_name);
		return ret;
	}
	case USDPAA_IOCTL_GET_LINK_STATUS:
	{
		int ret;
		struct usdpaa_ioctl_link_status_args input = {0};

		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		input.if_name[IF_NAME_MAX_LEN - 1] = '\0';

		ret = ioctl_usdpaa_get_link_status(&input);
		if (ret)
			return ret;

		if (copy_to_user(a, &input, sizeof(input)))
			return -EFAULT;

		return ret;
	}
	case USDPAA_IOCTL_UPDATE_LINK_STATUS:
	{
		struct usdpaa_ioctl_update_link_status input = {0};
		int ret;

		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		input.if_name[IF_NAME_MAX_LEN - 1] = '\0';

		ret = ioctl_set_link_status(&input);
		if (ret)
			pr_err("Error(%d) updating link status:IF: %s\n",
			       ret, input.if_name);
		return ret;
	}
	case USDPAA_IOCTL_GET_IOCTL_VERSION:
	{
		int ver_num = USDPAA_IOCTL_VERSION_NUMBER;

		if (copy_to_user(a, &ver_num, sizeof(ver_num)))
			return -EFAULT;

		return 0;
	}
	case USDPAA_IOCTL_UPDATE_LINK_SPEED:
	{
		struct usdpaa_ioctl_update_link_speed input = {0};
		int ret;

		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		input.if_name[IF_NAME_MAX_LEN - 1] = '\0';

		ret = ioctl_set_link_speed(&input);
		if (ret)
			pr_err("Error(%d) updating link speed:IF: %s\n",
			       ret, input.if_name);
		return ret;
	}
	case USDPAA_IOCTL_RESTART_LINK_AUTONEG:
	{
		char if_name[IF_NAME_MAX_LEN];
		int ret;

		if (copy_from_user(&if_name, a, sizeof(if_name)))
			return -EFAULT;
		if_name[IF_NAME_MAX_LEN - 1] = '\0';

		ret = ioctl_link_restart_autoneg(if_name);
		if (ret)
			pr_err("Error(%d) restarting autoneg:IF: %s\n",
			       ret, if_name);
		return ret;
	}
	}
	return -EINVAL;
}

static long usdpaa_ioctl_compat(struct file *fp, unsigned int cmd,
				unsigned long arg)
{
#ifdef CONFIG_COMPAT
	struct ctx *ctx = fp->private_data;
	void __user *a = (void __user *)arg;
#endif
	switch (cmd) {
#ifdef CONFIG_COMPAT
	case USDPAA_IOCTL_DMA_MAP_COMPAT:
	{
		int ret;
		struct usdpaa_ioctl_dma_map_compat input = {0};
		struct usdpaa_ioctl_dma_map converted = {0};

		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;

		converted.ptr = compat_ptr(input.ptr);
		converted.phys_addr = input.phys_addr;
		converted.len = input.len;
		converted.flags = input.flags;
		memset(converted.name, '\0', USDPAA_DMA_NAME_MAX);
		strncpy(converted.name, input.name, USDPAA_DMA_NAME_MAX);
		converted.name[USDPAA_DMA_NAME_MAX - 1] = '\0';
		converted.has_locking = input.has_locking;
		converted.did_create = input.did_create;

		ret = ioctl_dma_map(fp, ctx, &converted);
		input.ptr = ptr_to_compat(converted.ptr);
		input.phys_addr = converted.phys_addr;
		input.len = converted.len;
		input.flags = converted.flags;
		memset(input.name, '\0', USDPAA_DMA_NAME_MAX);
		strncpy(input.name, converted.name, USDPAA_DMA_NAME_MAX);
		input.name[USDPAA_DMA_NAME_MAX - 1] = '\0';
		input.has_locking = converted.has_locking;
		input.did_create = converted.did_create;
		if (copy_to_user(a, &input, sizeof(input)))
			return -EFAULT;
		return ret;
	}
	case USDPAA_IOCTL_PORTAL_MAP_COMPAT:
	{
		int ret;
		struct compat_usdpaa_ioctl_portal_map input;
		struct usdpaa_ioctl_portal_map converted = {0};
		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		converted.type = input.type;
		converted.index = input.index;
		ret = ioctl_portal_map(fp, ctx, &converted);
		input.addr.cinh = ptr_to_compat(converted.addr.cinh);
		input.addr.cena = ptr_to_compat(converted.addr.cena);
		input.channel = converted.channel;
		input.pools = converted.pools;
		input.index = converted.index;
		if (copy_to_user(a, &input, sizeof(input)))
			return -EFAULT;
		return ret;
	}
	case USDPAA_IOCTL_PORTAL_UNMAP_COMPAT:
	{
		struct usdpaa_portal_map_compat input;
		struct usdpaa_portal_map converted = {0};

		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		converted.cinh = compat_ptr(input.cinh);
		converted.cena = compat_ptr(input.cena);
		return ioctl_portal_unmap(ctx, &converted);
	}
	case USDPAA_IOCTL_ALLOC_RAW_PORTAL_COMPAT:
	{
		int ret;
		struct usdpaa_ioctl_raw_portal converted = {0};
		struct compat_ioctl_raw_portal input;
		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		converted.type = input.type;
		converted.index = input.index;
		converted.enable_stash = input.enable_stash;
		converted.cpu = input.cpu;
		converted.cache = input.cache;
		converted.window = input.window;
		converted.sdest = input.sdest;
		ret = ioctl_allocate_raw_portal(fp, ctx, &converted);

		input.cinh = converted.cinh;
		input.cena = converted.cena;
		input.index = converted.index;

		if (copy_to_user(a, &input, sizeof(input)))
			return -EFAULT;
		return ret;
	}
	case USDPAA_IOCTL_FREE_RAW_PORTAL_COMPAT:
	{
		struct usdpaa_ioctl_raw_portal converted = {0};
		struct compat_ioctl_raw_portal input;
		if (copy_from_user(&input, a, sizeof(input)))
			return -EFAULT;
		converted.type = input.type;
		converted.index = input.index;
		converted.cinh = input.cinh;
		converted.cena = input.cena;
		return ioctl_free_raw_portal(fp, ctx, &converted);
	}
#endif
	default:
		return usdpaa_ioctl(fp, cmd, arg);
	}
	return -EINVAL;
}

int usdpaa_get_portal_config(struct file *filp, void *cinh,
			     enum usdpaa_portal_type ptype, unsigned int *irq,
			     void **iir_reg)
{
	/* Walk the list of portals for filp and return the config
	   for the portal that matches the hint */
	struct ctx *context;
	struct portal_mapping *portal;

	/* First sanitize the filp */
	if (filp->f_op->open != usdpaa_open)
		return -ENODEV;
	context = filp->private_data;
	spin_lock(&context->lock);
	list_for_each_entry(portal, &context->portals, list) {
		if (portal->user.type == ptype &&
		    portal->user.addr.cinh == cinh) {
			if (ptype == usdpaa_portal_qman) {
				*irq = portal->qportal->public_cfg.irq;
				*iir_reg = portal->qportal->addr_virt[1] +
					QM_REG_IIR;
			} else {
				*irq = portal->bportal->public_cfg.irq;
				*iir_reg = portal->bportal->addr_virt[1] +
					BM_REG_IIR;
			}
			spin_unlock(&context->lock);
			return 0;
		}
	}
	spin_unlock(&context->lock);
	return -EINVAL;
}

static const struct file_operations usdpaa_fops = {
	.open		   = usdpaa_open,
	.release	   = usdpaa_release,
	.mmap		   = usdpaa_mmap,
	.get_unmapped_area = usdpaa_get_unmapped_area,
	.unlocked_ioctl	   = usdpaa_ioctl,
	.compat_ioctl	   = usdpaa_ioctl_compat
};

static struct miscdevice usdpaa_miscdev = {
	.name = "fsl-usdpaa",
	.fops = &usdpaa_fops,
	.minor = MISC_DYNAMIC_MINOR,
};

/* Early-boot memory allocation. The boot-arg "usdpaa_mem=<x>" is used to
 * indicate how much memory (if any) to allocate during early boot. If the
 * format "usdpaa_mem=<x>,<y>" is used, then <y> will be interpreted as the
 * number of TLB1 entries to reserve (default is 1). If there are more mappings
 * than there are TLB1 entries, fault-handling will occur. */

static __init int usdpaa_mem(char *arg)
{
	pr_warn("uspdaa_mem argument is depracated\n");
	arg_phys_size = memparse(arg, &arg);
	num_tlb = 1;
	if (*arg == ',') {
		unsigned long ul;
		int err = kstrtoul(arg + 1, 0, &ul);
		if (err < 0) {
			num_tlb = 1;
			pr_warn("ERROR, usdpaa_mem arg is invalid\n");
		} else
			num_tlb = (unsigned int)ul;
	}
	return 0;
}
early_param("usdpaa_mem", usdpaa_mem);

static int usdpaa_mem_init(struct reserved_mem *rmem)
{
	phys_start = rmem->base;
	phys_size = rmem->size;

	WARN_ON(!(phys_start && phys_size));

	return 0;
}
RESERVEDMEM_OF_DECLARE(usdpaa_mem_init, "fsl,usdpaa-mem", usdpaa_mem_init);

__init int fsl_usdpaa_init_early(void)
{
	if (!phys_size || !phys_start) {
		pr_info("No USDPAA memory, no 'fsl,usdpaa-mem' in device-tree\n");
		return 0;
	}
	if (phys_size % PAGE_SIZE) {
		pr_err("'fsl,usdpaa-mem' size must be a multiple of page size\n");
		phys_size = 0;
		return 0;
	}
	if (arg_phys_size && phys_size != arg_phys_size) {
		pr_err("'usdpaa_mem argument size (0x%llx) does not match device tree size (0x%llx)\n",
		       arg_phys_size, phys_size);
		phys_size = 0;
		return 0;
	}
	pfn_start = phys_start >> PAGE_SHIFT;
	pfn_size = phys_size >> PAGE_SHIFT;
	pr_info("USDPAA region at %llx:%llx(%lx:%lx), %d TLB1 entries)\n",
		phys_start, phys_size, pfn_start, pfn_size, num_tlb);
	return 0;
}
subsys_initcall(fsl_usdpaa_init_early);


static int __init usdpaa_init(void)
{
	struct mem_fragment *frag;
	int ret;
	u64 tmp_size = phys_size;
	u64 tmp_start = phys_start;
	u64 tmp_pfn_start = pfn_start;

	pr_info("Freescale USDPAA process driver\n");
	if (!phys_start) {
		pr_warn("fsl-usdpaa: no region found\n");
		return 0;
	}

	while (tmp_size != 0) {
		u32 frag_size = largest_page_size(tmp_size);
		frag = kmalloc(sizeof(*frag), GFP_KERNEL);
		if (!frag) {
			pr_err("Failed to setup USDPAA memory accounting\n");
			return -ENOMEM;
		}
		frag->base = tmp_start;
		frag->len = frag->root_len = frag_size;
		frag->root_pfn = tmp_pfn_start;
		frag->pfn_base = tmp_pfn_start;
		frag->pfn_len = frag_size / PAGE_SIZE;
		frag->refs = 0;
		init_waitqueue_head(&frag->wq);
		frag->owner = NULL;
		list_add(&frag->list, &mem_list);

		/* Adjust for this frag */
		tmp_start += frag_size;
		tmp_size -= frag_size;
		tmp_pfn_start += frag_size / PAGE_SIZE;
	}
	ret = misc_register(&usdpaa_miscdev);
	if (ret)
		pr_err("fsl-usdpaa: failed to register misc device\n");
	return ret;
}

static void __exit usdpaa_exit(void)
{
	misc_deregister(&usdpaa_miscdev);
}

module_init(usdpaa_init);
module_exit(usdpaa_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Freescale USDPAA process driver");
