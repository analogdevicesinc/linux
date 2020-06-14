/* Copyright 2011-2012 Freescale Semiconductor, Inc.
 * Copyright 2020 NXP
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef FSL_USDPAA_H
#define FSL_USDPAA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/fsl_qman.h> /* For "enum qm_channel" */
#include <linux/compat.h>

#ifdef CONFIG_FSL_USDPAA

/******************************/
/* Allocation of resource IDs */
/******************************/

/* This enum is used to distinguish between the type of underlying object being
 * manipulated. */
enum usdpaa_id_type {
	usdpaa_id_fqid,
	usdpaa_id_bpid,
	usdpaa_id_qpool,
	usdpaa_id_cgrid,
	usdpaa_id_ceetm0_lfqid,
	usdpaa_id_ceetm0_channelid,
	usdpaa_id_ceetm1_lfqid,
	usdpaa_id_ceetm1_channelid,
	usdpaa_id_max /* <-- not a valid type, represents the number of types */
};
#define USDPAA_IOCTL_MAGIC 'u'
struct usdpaa_ioctl_id_alloc {
	uint32_t base; /* Return value, the start of the allocated range */
	enum usdpaa_id_type id_type; /* what kind of resource(s) to allocate */
	uint32_t num; /* how many IDs to allocate (and return value) */
	uint32_t align; /* must be a power of 2, 0 is treated like 1 */
	int partial; /* whether to allow less than 'num' */
};
struct usdpaa_ioctl_id_release {
	/* Input; */
	enum usdpaa_id_type id_type;
	uint32_t base;
	uint32_t num;
};
struct usdpaa_ioctl_id_reserve {
	enum usdpaa_id_type id_type;
	uint32_t base;
	uint32_t num;
};


/* ioctl() commands */
#define USDPAA_IOCTL_ID_ALLOC \
	_IOWR(USDPAA_IOCTL_MAGIC, 0x01, struct usdpaa_ioctl_id_alloc)
#define USDPAA_IOCTL_ID_RELEASE \
	_IOW(USDPAA_IOCTL_MAGIC, 0x02, struct usdpaa_ioctl_id_release)
#define USDPAA_IOCTL_ID_RESERVE \
	_IOW(USDPAA_IOCTL_MAGIC, 0x0A, struct usdpaa_ioctl_id_reserve)

/**********************/
/* Mapping DMA memory */
/**********************/

/* Maximum length for a map name, including NULL-terminator */
#define USDPAA_DMA_NAME_MAX 16
/* Flags for requesting DMA maps. Maps are private+unnamed or sharable+named.
 * For a sharable and named map, specify _SHARED (whether creating one or
 * binding to an existing one). If _SHARED is specified and _CREATE is not, then
 * the mapping must already exist. If _SHARED and _CREATE are specified and the
 * mapping doesn't already exist, it will be created. If _SHARED and _CREATE are
 * specified and the mapping already exists, the mapping will fail unless _LAZY
 * is specified. When mapping to a pre-existing sharable map, the length must be
 * an exact match. Lengths must be a power-of-4 multiple of page size.
 *
 * Note that this does not actually map the memory to user-space, that is done
 * by a subsequent mmap() using the page offset returned from this ioctl(). The
 * ioctl() is what gives the process permission to do this, and a page-offset
 * with which to do so.
 */
#define USDPAA_DMA_FLAG_SHARE    0x01
#define USDPAA_DMA_FLAG_CREATE   0x02
#define USDPAA_DMA_FLAG_LAZY     0x04
#define USDPAA_DMA_FLAG_RDONLY   0x08
struct usdpaa_ioctl_dma_map {
	/* Output parameters - virtual and physical addresses */
	void *ptr;
	uint64_t phys_addr;
	/* Input parameter, the length of the region to be created (or if
	 * mapping an existing region, this must match it). Must be a power-of-4
	 * multiple of page size. */
	uint64_t len;
	/* Input parameter, the USDPAA_DMA_FLAG_* settings. */
	uint32_t flags;
	/* If _FLAG_SHARE is specified, the name of the region to be created (or
	 * of the existing mapping to use). */
	char name[USDPAA_DMA_NAME_MAX];
	/* If this ioctl() creates the mapping, this is an input parameter
	 * stating whether the region supports locking. If mapping an existing
	 * region, this is a return value indicating the same thing. */
	int has_locking;
	/* In the case of a successful map with _CREATE and _LAZY, this return
	 * value indicates whether we created the mapped region or whether it
	 * already existed. */
	int did_create;
};

#ifdef CONFIG_COMPAT
struct usdpaa_ioctl_dma_map_compat {
	/* Output parameters - virtual and physical addresses */
	compat_uptr_t ptr;
	uint64_t phys_addr;
	/* Input parameter, the length of the region to be created (or if
	 * mapping an existing region, this must match it). Must be a power-of-4
	 * multiple of page size. */
	uint64_t len;
	/* Input parameter, the USDPAA_DMA_FLAG_* settings. */
	uint32_t flags;
	/* If _FLAG_SHARE is specified, the name of the region to be created (or
	 * of the existing mapping to use). */
	char name[USDPAA_DMA_NAME_MAX];
	/* If this ioctl() creates the mapping, this is an input parameter
	 * stating whether the region supports locking. If mapping an existing
	 * region, this is a return value indicating the same thing. */
	int has_locking;
	/* In the case of a successful map with _CREATE and _LAZY, this return
	 * value indicates whether we created the mapped region or whether it
	 * already existed. */
	int did_create;
};

#define USDPAA_IOCTL_DMA_MAP_COMPAT \
	_IOWR(USDPAA_IOCTL_MAGIC, 0x03, struct usdpaa_ioctl_dma_map_compat)
#endif


#define USDPAA_IOCTL_DMA_MAP \
	_IOWR(USDPAA_IOCTL_MAGIC, 0x03, struct usdpaa_ioctl_dma_map)
/* munmap() does not remove the DMA map, just the user-space mapping to it.
 * This ioctl will do both (though you can munmap() before calling the ioctl
 * too). */
#define USDPAA_IOCTL_DMA_UNMAP \
	_IOW(USDPAA_IOCTL_MAGIC, 0x04, unsigned char)
/* We implement a cross-process locking scheme per DMA map. Call this ioctl()
 * with a mmap()'d address, and the process will (interruptible) sleep if the
 * lock is already held by another process. Process destruction will
 * automatically clean up any held locks. */
#define USDPAA_IOCTL_DMA_LOCK \
	_IOW(USDPAA_IOCTL_MAGIC, 0x05, unsigned char)
#define USDPAA_IOCTL_DMA_UNLOCK \
	_IOW(USDPAA_IOCTL_MAGIC, 0x06, unsigned char)

/***************************************/
/* Mapping and using QMan/BMan portals */
/***************************************/
enum usdpaa_portal_type {
	 usdpaa_portal_qman,
	 usdpaa_portal_bman,
};

#define QBMAN_ANY_PORTAL_IDX 0xffffffff

struct usdpaa_ioctl_portal_map {
	/* Input parameter, is a qman or bman portal required. */

	enum usdpaa_portal_type type;
	/* Specifes a specific portal index to map or QBMAN_ANY_PORTAL_IDX
	   for don't care.  The portal index will be populated by the
	   driver when the ioctl() successfully completes */
	uint32_t index;

	/* Return value if the map succeeds, this gives the mapped
	 * cache-inhibited (cinh) and cache-enabled (cena) addresses. */
	struct usdpaa_portal_map {
		void *cinh;
		void *cena;
	} addr;
	/* Qman-specific return values */
	uint16_t channel;
	uint32_t pools;
};

#ifdef CONFIG_COMPAT
struct compat_usdpaa_ioctl_portal_map {
	/* Input parameter, is a qman or bman portal required. */
	enum usdpaa_portal_type type;
	/* Specifes a specific portal index to map or QBMAN_ANY_PORTAL_IDX
	   for don't care.  The portal index will be populated by the
	   driver when the ioctl() successfully completes */
	uint32_t index;
	/* Return value if the map succeeds, this gives the mapped
	 * cache-inhibited (cinh) and cache-enabled (cena) addresses. */
	struct usdpaa_portal_map_compat {
		compat_uptr_t cinh;
		compat_uptr_t cena;
	} addr;
	/* Qman-specific return values */
	uint16_t channel;
	uint32_t pools;
};
#define USDPAA_IOCTL_PORTAL_MAP_COMPAT \
	_IOWR(USDPAA_IOCTL_MAGIC, 0x07, struct compat_usdpaa_ioctl_portal_map)
#define USDPAA_IOCTL_PORTAL_UNMAP_COMPAT \
	_IOW(USDPAA_IOCTL_MAGIC, 0x08, struct usdpaa_portal_map_compat)
#endif

#define USDPAA_IOCTL_PORTAL_MAP \
	_IOWR(USDPAA_IOCTL_MAGIC, 0x07, struct usdpaa_ioctl_portal_map)
#define USDPAA_IOCTL_PORTAL_UNMAP \
	_IOW(USDPAA_IOCTL_MAGIC, 0x08, struct usdpaa_portal_map)

struct usdpaa_ioctl_irq_map {
	enum usdpaa_portal_type type; /* Type of portal to map */
	int fd; /* File descriptor that contains the portal */
	void *portal_cinh; /* Cache inhibited area to identify the portal */
};

#define USDPAA_IOCTL_PORTAL_IRQ_MAP \
	_IOW(USDPAA_IOCTL_MAGIC, 0x09, struct usdpaa_ioctl_irq_map)

#ifdef CONFIG_COMPAT

struct compat_ioctl_irq_map {
	enum usdpaa_portal_type type; /* Type of portal to map */
	compat_int_t fd; /* File descriptor that contains the portal */
	compat_uptr_t portal_cinh; /* Used identify the portal */};

#define USDPAA_IOCTL_PORTAL_IRQ_MAP_COMPAT \
	_IOW(USDPAA_IOCTL_MAGIC, 0x09, struct compat_ioctl_irq_map)
#endif

/* ioctl to query the amount of DMA memory used in the system */
struct usdpaa_ioctl_dma_used {
	uint64_t free_bytes;
	uint64_t total_bytes;
};
#define USDPAA_IOCTL_DMA_USED \
	_IOR(USDPAA_IOCTL_MAGIC, 0x0B, struct usdpaa_ioctl_dma_used)

/* ioctl to allocate a raw portal */
struct usdpaa_ioctl_raw_portal {
	/* inputs */
	enum usdpaa_portal_type type; /* Type of portal to allocate */

	 /* set to non zero to turn on stashing */
	uint8_t enable_stash;
	/* Stashing attributes for the portal */
	uint32_t cpu;
	uint32_t cache;
	uint32_t window;

	/* Specifies the stash request queue this portal should use */
	uint8_t sdest;

	/* Specifes a specific portal index to map or QBMAN_ANY_PORTAL_IDX
	 * for don't care.  The portal index will be populated by the
	 * driver when the ioctl() successfully completes */
	uint32_t index;

	/* outputs */
	uint64_t cinh;
	uint64_t cena;
};

#define USDPAA_IOCTL_ALLOC_RAW_PORTAL \
	_IOWR(USDPAA_IOCTL_MAGIC, 0x0C, struct usdpaa_ioctl_raw_portal)

#define USDPAA_IOCTL_FREE_RAW_PORTAL \
	_IOR(USDPAA_IOCTL_MAGIC, 0x0D, struct usdpaa_ioctl_raw_portal)

#ifdef CONFIG_COMPAT

struct compat_ioctl_raw_portal {
	/* inputs */
	enum usdpaa_portal_type type; /* Type of portal to allocate */

	 /* set to non zero to turn on stashing */
	uint8_t enable_stash;
	/* Stashing attributes for the portal */
	uint32_t cpu;
	uint32_t cache;
	uint32_t window;
	/* Specifies the stash request queue this portal should use */
	uint8_t sdest;

	/* Specifes a specific portal index to map or QBMAN_ANY_PORTAL_IDX
	 * for don't care.  The portal index will be populated by the
	 * driver when the ioctl() successfully completes */
	uint32_t index;

	/* outputs */
	uint64_t cinh;
	uint64_t cena;
};

#define USDPAA_IOCTL_ALLOC_RAW_PORTAL_COMPAT \
	_IOWR(USDPAA_IOCTL_MAGIC, 0x0C, struct compat_ioctl_raw_portal)

#define USDPAA_IOCTL_FREE_RAW_PORTAL_COMPAT \
	_IOR(USDPAA_IOCTL_MAGIC, 0x0D, struct compat_ioctl_raw_portal)

#endif

#ifdef __KERNEL__

/* Early-boot hook */
int __init fsl_usdpaa_init_early(void);

/* Fault-handling in arch/powerpc/mm/mem.c gives USDPAA an opportunity to detect
 * faults within its ranges via this hook. */
int usdpaa_test_fault(unsigned long pfn, u64 *phys_addr, u64 *size);

#endif /* __KERNEL__ */

#endif /* CONFIG_FSL_USDPAA */

#ifdef __KERNEL__
/* This interface is needed in a few places and though it's not specific to
 * USDPAA as such, creating a new header for it doesn't make any sense. The
 * qbman kernel driver implements this interface and uses it as the backend for
 * both the FQID and BPID allocators. The fsl_usdpaa driver also uses this
 * interface for tracking per-process allocations handed out to user-space. */
struct dpa_alloc {
	struct list_head free;
	spinlock_t lock;
	struct list_head used;
};
#define DECLARE_DPA_ALLOC(name) \
	struct dpa_alloc name = { \
		.free = { \
			.prev = &name.free, \
			.next = &name.free \
		}, \
		.lock = __SPIN_LOCK_UNLOCKED(name.lock), \
		.used = { \
			 .prev = &name.used, \
			 .next = &name.used \
		 } \
	}
static inline void dpa_alloc_init(struct dpa_alloc *alloc)
{
	INIT_LIST_HEAD(&alloc->free);
	INIT_LIST_HEAD(&alloc->used);
	spin_lock_init(&alloc->lock);
}
int dpa_alloc_new(struct dpa_alloc *alloc, u32 *result, u32 count, u32 align,
		  int partial);
void dpa_alloc_free(struct dpa_alloc *alloc, u32 base_id, u32 count);
void dpa_alloc_seed(struct dpa_alloc *alloc, u32 fqid, u32 count);

/* Like 'new' but specifies the desired range, returns -ENOMEM if the entire
 * desired range is not available, or 0 for success. */
int dpa_alloc_reserve(struct dpa_alloc *alloc, u32 base_id, u32 count);
/* Pops and returns contiguous ranges from the allocator. Returns -ENOMEM when
 * 'alloc' is empty. */
int dpa_alloc_pop(struct dpa_alloc *alloc, u32 *result, u32 *count);
/* Returns 1 if the specified id is alloced, 0 otherwise */
int dpa_alloc_check(struct dpa_alloc *list, u32 id);
#endif /* __KERNEL__ */


/************************************
 * Link Status support for user space
 * interface
 ************************************/
#define IF_NAME_MAX_LEN 16
#define NODE_NAME_LEN	32
#define ETH_LINK_DOWN 0
#define ETH_LINK_UP 1

struct usdpaa_ioctl_link_status {
	/* network device node name */
	char		if_name[IF_NAME_MAX_LEN];
	/* Eventfd value */
	uint32_t	efd;
};

#define USDPAA_IOCTL_ENABLE_LINK_STATUS_INTERRUPT \
	_IOW(USDPAA_IOCTL_MAGIC, 0x0E, struct usdpaa_ioctl_link_status)

#define USDPAA_IOCTL_DISABLE_LINK_STATUS_INTERRUPT \
	_IOW(USDPAA_IOCTL_MAGIC, 0x0F, char *)

struct usdpaa_ioctl_link_status_args {
	/* network device node name */
	char    if_name[IF_NAME_MAX_LEN];
	/* link status(UP/DOWN) */
	int     link_status;
	int	link_speed;
	int	link_duplex;
	int	link_autoneg;
};

struct usdpaa_ioctl_update_link_status {
	/* network device node name */
	char    if_name[IF_NAME_MAX_LEN];
	/* link status(ETH_LINK_UP/DOWN) */
	int     set_link_status;
};

struct usdpaa_ioctl_update_link_speed {
	/* network device node name*/
	char    if_name[IF_NAME_MAX_LEN];
	int	link_speed;
	int	link_duplex;
};

#define USDPAA_IOCTL_GET_LINK_STATUS \
	_IOWR(USDPAA_IOCTL_MAGIC, 0x10, struct usdpaa_ioctl_link_status_args)

#define USDPAA_IOCTL_UPDATE_LINK_STATUS \
	_IOW(USDPAA_IOCTL_MAGIC, 0x11, struct usdpaa_ioctl_update_link_status)

#define USDPAA_IOCTL_UPDATE_LINK_SPEED \
	_IOW(USDPAA_IOCTL_MAGIC, 0x12, struct usdpaa_ioctl_update_link_speed)

#define USDPAA_IOCTL_GET_IOCTL_VERSION \
	_IOR(USDPAA_IOCTL_MAGIC, 0x14, int)

#define USDPAA_IOCTL_RESTART_LINK_AUTONEG \
	_IOW(USDPAA_IOCTL_MAGIC, 0x13, char *)

#ifdef __cplusplus
}
#endif

#endif /* FSL_USDPAA_H */
