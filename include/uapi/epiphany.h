#ifndef EPIPHANY_H
#define EPIPHANY_H
#include <linux/ioctl.h>

#if !defined(__KERNEL__)
#define __user
#endif

#if 0
#define PACK_ALIGN(x)  __attribute__((packed, aligned(x)))
#else
#define PACK_ALIGN(x) __attribute__((aligned(x)))
#endif

/** Length of the Global shared memory region */
#define GLOBAL_SHM_SIZE               (1<<20)
#define MAX_SHM_REGIONS                64
#define SHM_LOCK_NAME                  "/eshmlock" 

#define SHM_MAGIC                     0xabcdef00

typedef struct _EPIPHANY_ALLOC
{
    unsigned long     size;
    unsigned long     flags;
    unsigned long     bus_addr;    /* out */
    unsigned long     phy_addr;    /* out */
    unsigned long     kvirt_addr;  /* out */
    unsigned long     uvirt_addr;  /* out */
    unsigned long     mmap_handle; /* Handle to use for mmap */
} epiphany_alloc_t;

#pragma pack(push, 1)

/** Shared memory segment */
typedef struct PACK_ALIGN(8) e_shmseg {
    void     *addr;         /* Virtual address */
    char      name[256];    /* Region name */
    size_t    size;         /* Region size in bytes */
    void     *paddr;        /* Physical Address accessible from Epiphany cores */
    off_t     offset;       /* Offset from shm base address */
} e_shmseg_t;

typedef struct PACK_ALIGN(8) e_shmseg_pvt  {
    e_shmseg_t  shm_seg;  /* The shared memory segment */
    unsigned    refcnt;   /* host app reference count */
    unsigned    valid;    /* 1 if the region is in use, 0 otherwise */
} e_shmseg_pvt_t;

typedef struct PACK_ALIGN(8) e_shmtable {
    unsigned int   magic;
    unsigned int   padding;
    e_shmseg_pvt_t regions[MAX_SHM_REGIONS];
    unsigned int   free_space;
    off_t          next_free_offset;
    unsigned long  paddr_epi;   /* Physical address of the shm region as seen by epiphany */
    unsigned long  paddr_cpu;   /* Physical address of the shm region as seen by the host cpu */
    char* __user   heap;
    void* __user   lock;        /* User-space semaphore */
} e_shmtable_t;

#pragma pack(pop)

#define EPIPHANY_IOC_MAGIC  'k'

/**
 * If you add an IOC command, please update the 
 * EPIPHANY_IOC_MAXNR macro
 */

#define EPIPHANY_IOC_GETSHM_CMD   24

#define EPIPHANY_IOC_MAXNR        24
 
#define EPIPHANY_IOC_GETSHM _IOWR(EPIPHANY_IOC_MAGIC, EPIPHANY_IOC_GETSHM_CMD, epiphany_alloc_t *)

#endif
