/*
 * Freescale HIFI 4 driver
 *
 * Copyright 2018 NXP
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <uapi/linux/mxc_hifi4.h>
#include "fsl_hifi4_proxy.h"


typedef void (*memcpy_func) (void *dest, const void *src, size_t n);
typedef void (*memset_func) (void *s, int c, size_t n);

/* ...maximal number of IPC clients per proxy */
#define XF_CFG_MAX_IPC_CLIENTS          (1 << 4)


/* ...proxy client data */
struct xf_client {
	/* ...pointer to proxy interface */
	struct xf_proxy     *proxy;

	/* ...allocated proxy client id */
	u32                 id;

	/* ...pending response queue */
	struct xf_msg_queue queue;

	/* ...response waiting queue */
	wait_queue_head_t   wait;

	/* ...virtual memory mapping */
	unsigned long       vm_start;

	/* ...counter of memory mappings (no real use of it yet - tbd) */
	atomic_t            vm_use;

	/* ...global structure pointer */
	void				*global;
};

union xf_client_link {
	/* ...index of next client in free list */
	u32                 next;

	/* ...reference to proxy data for allocated client */
	struct xf_client    *client;
};

struct fsl_hifi4 {
	struct device			*dev;
	const char			*fw_name;
	void __iomem			*regs;
	void __iomem			*mu_base_virtaddr;
	sc_ipc_t			hifi_ipcHandle;
	sc_ipc_t			mu_ipcHandle;
	unsigned int			hifi_mu_id;
	int				hifi_mu_init;
	atomic_long_t			refcnt;
	unsigned long			paddr;
	unsigned long			dram0;
	unsigned long			dram1;
	unsigned long			iram;
	unsigned long			sram;
	void			        *sdram_vir_addr;
	unsigned long			sdram_phys_addr;
	void				*msg_buf_virt;
	dma_addr_t			 msg_buf_phys;
	int				 msg_buf_size;
	void				*scratch_buf_virt;
	dma_addr_t			 scratch_buf_phys;
	int				 scratch_buf_size;
	void				*hifi_config_virt;
	dma_addr_t			 hifi_config_phys;
	int				 hifi_config_size;

	/* ...proxy data structures */
	struct xf_proxy proxy;

	/* ...mutex lock */
	struct mutex hifi4_mutex;

	/* ...global clients pool (item[0] serves as list terminator) */
	union xf_client_link xf_client_map[XF_CFG_MAX_IPC_CLIENTS];
};

#define IRAM_OFFSET		0x10000
#define IRAM_SIZE		2048

#define DRAM0_OFFSET		0x0
#define DRAM0_SIZE		0x8000

#define DRAM1_OFFSET		0x8000
#define DRAM1_SIZE		0x8000

#define SYSRAM_OFFSET		0x18000
#define SYSRAM_SIZE		0x40000

#define SYSROM_OFFSET		0x58000
#define SYSROM_SIZE		0x30000

#define MSG_BUF_SIZE		8192
#define INPUT_BUF_SIZE		4096
#define OUTPUT_BUF_SIZE		16384
#define HIFI_CONFIG_SIZE    4096

/*external buffer
 *  ----------------------------------------------------------------------
 *  |  name                      | size     |   description     |
 * -----------------------------------------------------------------------
 *  |  scratch buffer for malloc | 0xffffff | For MEM_scratch_malloc()
 * ------------------------------------------------------------------------
 *  |  global structure          | 4096     | For store hifi config structure
 * ------------------------------------------------------------------------
 */

#define MEMORY_REMAP_OFFSET	0x39000000

/* reserved memory for hifi4 firmware and core libs to
 * save their instruction/data section in SDRAM, the physical
 * address range is 0x8e000000 ~ 0x8fffffff (32M bytes).
 */
#define SDRAM_BASE_ADDR  0x8e000000
#define SDRAM_BASE_SIZE  0x1ffffff
#define SDRAM_CODEC_LIB_OFFSET 0x1000000
#define SDRAM_SCRATCH_BUF_SIZE 0xffffff

#define SC_C_OFS_SEL    39
#define SC_C_OFS_AUDIO  40
#define SC_C_OFS_PERIPH 41
#define SC_C_OFS_IRQ    42

void *memcpy_hifi(void *dest, const void *src, size_t count);
void *memset_hifi(void *dest, int c, size_t count);
struct xf_client *xf_client_lookup(struct fsl_hifi4 *hifi4_priv, u32 id);
