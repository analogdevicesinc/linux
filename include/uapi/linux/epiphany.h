#ifndef EPIPHANY_H
#define EPIPHANY_H
#include <linux/ioctl.h>

/** Length of the Global shared memory region */
#define GLOBAL_SHM_SIZE               (4<<20)
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

#define EPIPHANY_IOC_MAGIC  'k'

/**
 * If you add an IOC command, please update the 
 * EPIPHANY_IOC_MAXNR macro
 */

#define EPIPHANY_IOC_GETSHM_CMD   24

#define EPIPHANY_IOC_MAXNR        24
 
#define EPIPHANY_IOC_GETSHM _IOWR(EPIPHANY_IOC_MAGIC, EPIPHANY_IOC_GETSHM_CMD, epiphany_alloc_t *)

#endif
