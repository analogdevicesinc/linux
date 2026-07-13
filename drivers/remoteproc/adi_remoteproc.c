// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Device SHARC Image Loader for SC5XX processors
 *
 * Copyright 2020-2022 Analog Devices
 *
 * @todo:
 * - sharc idle core as default with dts override
 * - timeout as default with dts override
 * - resource table dynamically constructed from dts data or executable file
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dmaengine.h>
#include <linux/firmware.h>
#include <linux/elf.h>
#include <linux/mailbox_client.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_ring.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include <linux/soc/adi/icc.h>
#include <linux/soc/adi/spu.h>
#include "remoteproc_internal.h"

/* location of bootrom that loops idle */
#define SHARC_IDLE_ADDR			(0x00090004)

#define SPU_MDMA0_SRC_ID		88
#define SPU_MDMA0_DST_ID		89

#define CORE_INIT_TIMEOUT_MS (2000)
#define CORE_INIT_TIMEOUT msecs_to_jiffies(CORE_INIT_TIMEOUT_MS)

#define MEMORY_COUNT 2

#define ADI_FW_LDR 0
#define ADI_FW_ELF 1

#define NUM_TABLE_ENTRIES         1
/* Resource table for the given remote */

struct bcode_flag_t {
	uint32_t bCode:4,			/* 0-3 */
			 bFlag_save:1,		/* 4 */
			 bFlag_aux:1,		/* 5 */
			 bReserved:1,		/* 6 */
			 bFlag_forward:1,	/* 7 */
			 bFlag_fill:1,		/* 8 */
			 bFlag_quickboot:1, /* 9 */
			 bFlag_callback:1,	/* 10 */
			 bFlag_init:1,		/* 11 */
			 bFlag_ignore:1,	/* 12 */
			 bFlag_indirect:1,	/* 13 */
			 bFlag_first:1,		/* 14 */
			 bFlag_final:1,		/* 15 */
			 bHdrCHK:8,			/* 16-23 */
			 bHdrSIGN:8;		/* 0xAD, 0xAC or 0xAB */
};

struct ldr_hdr {
	struct bcode_flag_t bcode_flag;
	u32 target_addr;
	u32 byte_count;
	u32 argument;
};

struct sharc_resource_table {
	struct resource_table table_hdr;
	unsigned int offset[NUM_TABLE_ENTRIES];
	struct fw_rsc_hdr rpmsg_vdev_hdr;
	struct fw_rsc_vdev rpmsg_vdev;
	struct fw_rsc_vdev_vring vring[2];
} __packed;

struct adi_sharc_resource_table {
	struct adi_resource_table_hdr adi_table_hdr;
	struct sharc_resource_table rsc_table;
} __packed;

#define VRING_ALIGN 0x1000
#define VRING_DEFAULT_SIZE 0x800

/*
 * In regular case the table comes from a firmware file, since ldr format doesn't have
 * resource_table section we initialize the table here, and let remoteproc_core.c
 * copy the initialized cached_table to reserved memory (adi,rsc-table) shared with remote core.
 * The table must be initialized before core start so the remote core
 * can't initialize the reserved memory either.
 */
static struct adi_sharc_resource_table _rsc_table_template = {
	.adi_table_hdr = {
		.tag = ADI_RESOURCE_TABLE_TAG,
		.version = 1,
		.initialized = 0,
	},
	.rsc_table = {
		.table_hdr = {
			/* resource table header */
			1,					/* version */
			NUM_TABLE_ENTRIES,	/* number of table entries */
			{0, 0,},			/* reserved fields */
		},
		.offset = {offsetof(struct sharc_resource_table, rpmsg_vdev_hdr),
		},
		/* virtio device entry */
		.rpmsg_vdev_hdr = {RSC_VDEV,},	/* virtio dev type */
		.rpmsg_vdev = {
			VIRTIO_ID_RPMSG,	/* it's rpmsg virtio */
			1,					/* kick sharc0 */
			/* 1<<0 is VIRTIO_RPMSG_F_NS bit defined in virtio_rpmsg_bus.c */
			1<<0, 0, 0, 0,		/* dfeatures, gfeatures, config len, status */
			2,					/* num_of_vrings */
			{0, 0,},			/* reserved */
		},
		.vring = {
			 /* da allocated by remoteproc driver */
			{FW_RSC_ADDR_ANY, VRING_ALIGN, VRING_DEFAULT_SIZE, 1, 0},
			 /* da allocated by remoteproc driver */
			{FW_RSC_ADDR_ANY, VRING_ALIGN, VRING_DEFAULT_SIZE, 1, 0},
		},
	},
};

enum adi_rproc_rpmsg_state {
	ADI_RP_RPMSG_SYNCED = 0,
	ADI_RP_RPMSG_WAITING = 1,
	ADI_RP_RPMSG_TIMED_OUT = 2,
};

struct adi_rproc_data {
	struct device *dev;
	struct rproc *rproc;
	struct reset_control *rst_crst;
	struct reset_control *rst_start;
	struct regmap *svect_regmap;
	u32 svect_offset;
	struct mbox_client kick_client;
	struct mbox_chan *kick_chan;
	const char *firmware_name;
	int core_id;
	int icc_irq;
	int icc_irq_flags;
	void *mem_virt;
	dma_addr_t mem_handle;
	size_t fw_size;
	unsigned long ldr_load_addr;
	int firmware_format;
	void __iomem *L1_shared_base;
	void __iomem *L2_shared_base;
	struct workqueue_struct *core_workqueue;
	enum adi_rproc_rpmsg_state rpmsg_state;
	u64 l1_da_range[2];
	u64 l2_da_range[2];
	u32 verify;
	struct adi_sharc_resource_table *adi_rsc_table;
	struct sharc_resource_table *loaded_rsc_table;
};

static int adi_core_set_svect(struct adi_rproc_data *rproc_data,
					unsigned long svect)
{
	return regmap_write(rproc_data->svect_regmap, rproc_data->svect_offset, svect);
}

static irqreturn_t sharc_virtio_irq_threaded_handler(int irq, void *p);

static int adi_core_start(struct adi_rproc_data *rproc_data)
{
	int ret = 0;

	if (rproc_data->adi_rsc_table != NULL) {
		rproc_data->rpmsg_state = ADI_RP_RPMSG_WAITING;
		ret = devm_request_threaded_irq(rproc_data->dev,
						rproc_data->icc_irq, NULL,
						sharc_virtio_irq_threaded_handler,
						rproc_data->icc_irq_flags,
						"ICC virtio IRQ", rproc_data);
	}
	if (ret) {
		dev_err(rproc_data->dev, "Fail to request ICC IRQ\n");
		return -ENOENT;
	}

	return reset_control_deassert(rproc_data->rst_start);
}

static int adi_core_reset(struct adi_rproc_data *rproc_data)
{
	return reset_control_reset(rproc_data->rst_crst);
}

static int adi_core_stop(struct adi_rproc_data *rproc_data)
{
	/* After time out the irq is already released */
	if (rproc_data->adi_rsc_table != NULL) {
		if (rproc_data->rpmsg_state != ADI_RP_RPMSG_TIMED_OUT)
			devm_free_irq(rproc_data->dev, rproc_data->icc_irq, rproc_data);
	}
	return reset_control_assert(rproc_data->rst_start);
}

static int is_final(struct ldr_hdr *hdr)
{
	return hdr->bcode_flag.bFlag_final;
}

static int is_empty(struct ldr_hdr *hdr)
{
	return hdr->bcode_flag.bFlag_ignore || (hdr->byte_count == 0);
}

static void load_callback(void *p)
{
	struct completion *cmp = p;

	complete(cmp);
}

/* @todo this needs to return status */
/* @todo the error paths here leak tremendously, this needs further cleanup */
static void ldr_load(struct adi_rproc_data *rproc_data)
{
	struct ldr_hdr *block_hdr = NULL;
	struct ldr_hdr *next_hdr = NULL;
	u8 *virbuf = (u8 *) rproc_data->mem_virt;
	dma_addr_t phybuf = rproc_data->mem_handle;
	int offset;
// part of verify buffer code
//	int i;
//	uint32_t verfied = 0;
//	uint8_t *pCompareBuffer;
//	uint8_t *pVerifyBuffer;
	struct dma_chan *chan = dma_find_channel(DMA_MEMCPY);
	struct dma_async_tx_descriptor *tx;
	struct completion cmp;

	if (!chan) {
		dev_err(rproc_data->dev, "Could not find dma memcpy channel\n");
		return;
	}

	init_completion(&cmp);

	do {
		/* read the header */
		block_hdr = (struct ldr_hdr *) virbuf;
		offset = sizeof(struct ldr_hdr) + (block_hdr->bcode_flag.bFlag_fill ?
					       0 : block_hdr->byte_count);
		next_hdr = (struct ldr_hdr *) (virbuf + offset);
		tx = NULL;

		/* Overwrite the ldr_load_addr */
		if (block_hdr->bcode_flag.bFlag_first)
			rproc_data->ldr_load_addr = (unsigned long)block_hdr->target_addr;

		if (!is_empty(block_hdr)) {
			if (block_hdr->bcode_flag.bFlag_fill) {
				tx = dmaengine_prep_dma_memset(chan,
							       block_hdr->target_addr,
							       block_hdr->argument,
							       block_hdr->byte_count, 0);
			} else {
				tx = dmaengine_prep_dma_memcpy(chan,
							       block_hdr->target_addr,
							       phybuf + sizeof(struct ldr_hdr),
							       block_hdr->byte_count, 0);

//				if (rproc_data->verify) {
//					@todo implement verification
//					pCompareBuffer = virbuf + sizeof(struct ldr_hdr);
//					pVerifyBuffer = virbuf + rproc_data->fw_size;
//
//					dma_memcpy(phybuf + rproc_data->fw_size,
//							   block_hdr->target_addr,
//							   block_hdr->byte_count);
//
//					/* check the data */
//					for (i = 0; i < block_hdr->byte_count; i++) {
//						if (pCompareBuffer[i] != pVerifyBuffer[i]) {
//							dev_err(rproc_data->dev,
//								    "dirty data, compare[%d]:0x%x,"
//									"verify[%d]:0x%x\n",
//									i, pCompareBuffer[i], i,
//									pVerifyBuffer[i]);
//							verfied++;
//							break;
//						}
//					}
//				}
			}

			if (!tx) {
				dev_err(rproc_data->dev, "Failed to allocate dma transaction\n");
				return;
			}

			if (is_final(block_hdr) || (is_final(next_hdr) && is_empty(next_hdr))) {
				tx->callback = load_callback;
				tx->callback_param = &cmp;
			}
			dmaengine_submit(tx);
			dma_async_issue_pending(chan);
		}

		if (is_final(block_hdr)) {
			wait_for_completion(&cmp);
			break;
		}

		virbuf += offset;
		phybuf += offset;
	} while (1);

//	if (rproc_data->verify) {
//		if (verfied == 0)
//			dev_err(rproc_data->dev, "success to verify all the data\n");
//		else
//			dev_err(rproc_data->dev, "fail to verify all the data %d\n", verfied);
//	}
}

static int adi_valid_firmware(struct rproc *rproc, const struct firmware *fw)
{
	struct ldr_hdr *adi_ldr_hdr = (struct ldr_hdr *)fw->data;

	if (!adi_ldr_hdr->byte_count &&
	    (adi_ldr_hdr->bcode_flag.bHdrSIGN == 0xAD ||
	     adi_ldr_hdr->bcode_flag.bHdrSIGN == 0xAC ||
	     adi_ldr_hdr->bcode_flag.bHdrSIGN == 0xAB))
		return ADI_FW_LDR;

	if (!rproc_elf_sanity_check(rproc, fw)) {
		dev_err(&rproc->dev, "ELF format not supported\n");
		return -EOPNOTSUPP;
	}

	dev_err(&rproc->dev, "## No valid image at address %p\n", fw->data);
	return -EINVAL;
}

static void enable_spu(void)
{
	adi_spu_set_securep(SPU_MDMA0_SRC_ID, true);
	adi_spu_set_securep(SPU_MDMA0_DST_ID, true);
}

static void disable_spu(void)
{
	adi_spu_set_securep(SPU_MDMA0_SRC_ID, false);
	adi_spu_set_securep(SPU_MDMA0_DST_ID, false);
}

static int adi_ldr_load(struct adi_rproc_data *rproc_data,
						const struct firmware *fw)
{
	rproc_data->fw_size = fw->size;
	if (!rproc_data->mem_virt) {
		rproc_data->mem_virt = dma_alloc_coherent(rproc_data->dev,
							  fw->size * MEMORY_COUNT,
							  &rproc_data->mem_handle,
							  GFP_KERNEL);
		if (rproc_data->mem_virt == NULL) {
			dev_err(rproc_data->dev, "Unable to allocate memory\n");
			return -ENOMEM;
		}
	}

	memcpy((char *)rproc_data->mem_virt, fw->data, fw->size);

	enable_spu();
	ldr_load(rproc_data);
	disable_spu();

	return 0;
}

/*
 * adi_rproc_load: parse and load ADI SHARC LDR file into memory
 *
 * This function would be called when user run the start command
 * echo start > /sys/class/remoteproc/remoteprocX/state
 */
static int adi_rproc_load(struct rproc *rproc, const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	int ret;

	switch (rproc_data->firmware_format) {
	case ADI_FW_LDR:
		ret = adi_ldr_load(rproc_data, fw);
		break;
	case ADI_FW_ELF:
		ret = rproc_elf_load_segments(rproc, fw);
		break;
	default:
		WARN(1, "Invalid rproc_data->firmware_format\n");
		return -EINVAL;
	}

	if (ret)
		dev_err(rproc_data->dev, "Failed to load ldr, ret:%d\n", ret);

	return ret;
}

/*
 * adi_rproc_start: to start run the applicaiton which is loaded in memory
 *
 * This function would be called when user run the start command
 * echo start > /sys/class/remoteproc/remoteprocX/state
 */
static int adi_rproc_start(struct rproc *rproc)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	int ret;

	ret = adi_core_set_svect(rproc_data, rproc_data->ldr_load_addr);
	if (ret)
		return ret;

	ret = adi_core_reset(rproc_data);
	if (ret)
		return ret;

	return adi_core_start(rproc_data);
}

/*
 * adi_rproc_stop: to stop the running applicaiton in DSP
 * This would be called when user run the stop command
 * echo stop > /sys/class/remoteproc/remoteprocX/state
 */
static int adi_rproc_stop(struct rproc *rproc)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	int ret;

	ret = adi_core_set_svect(rproc_data, SHARC_IDLE_ADDR);
	if (ret)
		return ret;

	ret = adi_core_stop(rproc_data);
	if (ret)
		return ret;

	ret = adi_core_reset(rproc_data);
	if (ret)
		return ret;

	if (rproc_data->mem_virt) {
		memset(rproc_data->mem_virt, 0, rproc_data->fw_size * MEMORY_COUNT);
		dma_free_coherent(rproc_data->dev, rproc_data->fw_size * MEMORY_COUNT,
				  rproc_data->mem_virt, rproc_data->mem_handle);
		rproc_data->mem_virt = NULL;
		rproc_data->fw_size = 0;
	}

	rproc_data->ldr_load_addr = SHARC_IDLE_ADDR;
	rproc_data->loaded_rsc_table = NULL;
	return ret;
}

static int adi_ldr_load_rsc_table(struct rproc *rproc, const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	size_t size = sizeof(_rsc_table_template.rsc_table);

	if (rproc_data->adi_rsc_table == NULL)
		return -EINVAL;

	/* kfree in remoteproc_core.c */
	rproc->cached_table = kmemdup(&_rsc_table_template.rsc_table, size, GFP_KERNEL);
	if (!rproc->cached_table)
		return -ENOMEM;

	if (rproc_data->core_id == 1) {
		((struct sharc_resource_table *)rproc->cached_table)->rpmsg_vdev.notifyid = 1;
		((struct sharc_resource_table *)rproc->cached_table)->vring[0].notifyid = 1;
		((struct sharc_resource_table *)rproc->cached_table)->vring[1].notifyid = 1;
	} else {
		((struct sharc_resource_table *)rproc->cached_table)->rpmsg_vdev.notifyid = 2;
		((struct sharc_resource_table *)rproc->cached_table)->vring[0].notifyid = 2;
		((struct sharc_resource_table *)rproc->cached_table)->vring[1].notifyid = 2;
	}

	/* Initialize ADI resource table header*/
	rproc_data->adi_rsc_table->adi_table_hdr = _rsc_table_template.adi_table_hdr;

	rproc->table_ptr = rproc->cached_table;
	rproc->table_sz = size;

	return 0;
}

static int adi_rproc_map_carveout(struct rproc *rproc, struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	va = ioremap_wc(mem->dma, mem->len);
	if (!va) {
		dev_err(dev, "Unable to map memory carveout %pa+%zx\n", &mem->dma, mem->len);
		return -ENOMEM;
	}
	mem->va = va;
	return 0;
}

static int adi_rproc_unmap_carveout(struct rproc *rproc, struct rproc_mem_entry *mem)
{
	iounmap(mem->va);
	return 0;
}

static int adi_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct sharc_resource_table *rsc_table;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	phys_addr_t size;
	int ret, i, mem_regions, num;

	if (rproc_data->adi_rsc_table == NULL)
		return 0;

	switch (rproc_data->firmware_format) {
	case ADI_FW_LDR:
		ret = adi_ldr_load_rsc_table(rproc, fw);
		break;
	case ADI_FW_ELF:
		ret = rproc_elf_load_rsc_table(rproc, fw);
		break;
	default:
		WARN(1, "Invalid rproc_data->firmware_format\n");
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	/* Set defaults */
	rsc_table = (struct sharc_resource_table *)rproc->cached_table;
	rsc_table->vring[0].da = FW_RSC_ADDR_ANY;
	rsc_table->vring[0].num = VRING_DEFAULT_SIZE;
	rsc_table->vring[1].da = FW_RSC_ADDR_ANY;
	rsc_table->vring[1].num = VRING_DEFAULT_SIZE;

	/*
	 * Find reserved memory for vrings, if not found uses CMA region
	 * The reserved memory can be in internal SRAM for better access time.
	 */
	mem_regions = of_count_phandle_with_args(np, "vdev-vring", NULL);
	for (i = 0; i < mem_regions; i++) {
		struct device_node *node __free(device_node) = of_parse_phandle(np, "vdev-vring", i);
		rmem = of_reserved_mem_lookup(node);
		if (!rmem) {
			dev_err(dev, "Failed to acquire vdev-vring at idx %d\n", i);
			return -EINVAL;
		}

		/* We need at least 16kB for vdev-vring */
		if (rmem->size < 0x4000) {
			dev_err(dev, "Insufficient space, vdev-vring idx %d, min req 16kB\n", i);
			return -EINVAL;
		}

		/* Split the range for two vrings, vring0 -rx and vring1 - tx*/
		size = rmem->size / 2;

		mem = rproc_mem_entry_init(dev, NULL,
					   (dma_addr_t)rmem->base,
					   size, rmem->base,
					   adi_rproc_map_carveout,
					   adi_rproc_unmap_carveout,
					   "vdev%dvring0", i);
		if (!mem)
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);

		mem = rproc_mem_entry_init(dev, NULL,
					   (dma_addr_t)rmem->base + size,
					   size, rmem->base + size,
					   adi_rproc_map_carveout,
					   adi_rproc_unmap_carveout,
					   "vdev%dvring1", i);
		if (!mem)
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);

		/* Update the resource table before loading*/
		/* TODO add support for multiple vdev devices*/
		if (i > 0) {
			continue;
		} else {
			/* Calc how many buffers we can fit in the vring region,
			 * number of buffers must be power of 2
			 */
			for (num = 2; num < 0x00400000; num <<= 1) {
				if (PAGE_ALIGN(vring_size(num, VRING_ALIGN)) > size) {
					num >>= 1; // It's too much, restore
							   // previous value and break
					break;
				}
			}

			rsc_table->vring[0].da = rmem->base;
			rsc_table->vring[0].num = num;
			rsc_table->vring[1].da = rmem->base + size;
			rsc_table->vring[1].num = num;
		}
	}

	/*
	 * Find reserved memory for vring buffers.
	 * The reserved memory be in internal SRAM
	 * for better access time.  If found sets
	 * DMA API to use the region, if not found
	 * uses CMA region
	 */
	mem_regions = of_count_phandle_with_args(np, "memory-region", NULL);
	for (i = 0; i < mem_regions; i++) {
		struct device_node *node __free(device_node) = of_parse_phandle(np, "memory-region", i);
		rmem = of_reserved_mem_lookup(node);
		mem = rproc_of_resm_mem_entry_init(dev, i, rmem->size,
						   rmem->base, "vdev%dbuffer", i);
		if (!mem)
			return -ENOMEM;
		rproc_add_carveout(rproc, mem);
	}

	return 0;
}

static struct resource_table *adi_ldr_find_loaded_rsc_table(struct rproc *rproc,
							    const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;

	if (rproc_data->adi_rsc_table == NULL)
		return NULL;

	return &rproc_data->adi_rsc_table->rsc_table.table_hdr;
}

static struct resource_table *adi_rproc_find_loaded_rsc_table(struct rproc *rproc,
							      const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	struct resource_table *ret = NULL;

	if (rproc_data->adi_rsc_table == NULL)
		return NULL;

	switch (rproc_data->firmware_format) {
	case ADI_FW_LDR:
		ret = adi_ldr_find_loaded_rsc_table(rproc, fw);
		break;
	case ADI_FW_ELF:
		ret = rproc_elf_find_loaded_rsc_table(rproc, fw);
		break;
	default:
		WARN(1, "Invalid rproc_data->firmware_format\n");
		return NULL;
	}
	rproc_data->loaded_rsc_table = (struct sharc_resource_table *)ret;
	return ret;
}

/* @todo store number of vrings from resource table and use it to dynamically
 * notify the correct number of vrings
 */
static irqreturn_t sharc_virtio_irq_threaded_handler(int irq, void *p)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)p;
	struct sharc_resource_table *table = rproc_data->loaded_rsc_table;

	/* Firmwares witout resource table shouldn't enable the virtio irq */
	if (!table) {
		WARN(1, "Invalid rproc_data->firmware_format\n");
		return -EINVAL;
	}

	rproc_vq_interrupt(rproc_data->rproc, table->vring[0].notifyid);
	rproc_vq_interrupt(rproc_data->rproc, table->vring[1].notifyid);

	return IRQ_HANDLED;
}

/* kick a virtqueue */
static void adi_rproc_kick(struct rproc *rproc, int vqid)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	int wait_time;

	/* On first kick check if remote core has done its initialization */
	if (rproc_data->rpmsg_state == ADI_RP_RPMSG_WAITING) {
		for (wait_time = 0; wait_time < CORE_INIT_TIMEOUT_MS; wait_time += 20) {
			if (rproc_data->adi_rsc_table->adi_table_hdr.initialized ==
			    ADI_RSC_TABLE_INIT_MAGIC) {
				rproc_data->rpmsg_state = ADI_RP_RPMSG_SYNCED;
				break;
			}
			msleep(20);
		}
		if (rproc_data->rpmsg_state != ADI_RP_RPMSG_SYNCED) {
			rproc_data->rpmsg_state = ADI_RP_RPMSG_TIMED_OUT;
			devm_free_irq(rproc_data->dev, rproc_data->icc_irq, rproc_data);
			dev_info(rproc_data->dev,
					 "Core%d rpmsg init timeout, probably not supported.\n",
					 rproc_data->core_id);
		}
	}

	if (rproc_data->rpmsg_state == ADI_RP_RPMSG_SYNCED)
		mbox_send_message(rproc_data->kick_chan, NULL);
}

static int adi_rproc_sanity_check(struct rproc *rproc, const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;

	/* Check if it is a LDR or ELF file */
	rproc_data->firmware_format = adi_valid_firmware(rproc, fw);

	if (rproc_data->firmware_format < 0)
		return rproc_data->firmware_format;
	else
		return 0;
}

static u64 adi_rproc_get_boot_addr(struct rproc *rproc, const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	u64 ret;

	switch (rproc_data->firmware_format) {
	case ADI_FW_LDR:
		ret = 0;
		break;
	case ADI_FW_ELF:
		ret = rproc_elf_get_boot_addr(rproc, fw);
		break;
	default:
		WARN(1, "Invalid rproc_data->firmware_format\n");
		return -EINVAL;
	}
	return ret;
}

static void *adi_rproc_da_to_va(struct rproc *rproc, u64 da, size_t len, bool *unused)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	void __iomem *L1_shared_base = rproc_data->L1_shared_base;
	void __iomem *L2_shared_base = rproc_data->L2_shared_base;
	void *ret = NULL;

	if (len == 0)
		return NULL;

	if (da >= rproc_data->l1_da_range[0] && da < rproc_data->l1_da_range[1])
		ret = L1_shared_base + da;
	else if (da >= rproc_data->l2_da_range[0] && da < rproc_data->l2_da_range[1])
		ret = L2_shared_base + (da - rproc_data->l2_da_range[0]);

	return ret;
}

static const struct rproc_ops adi_rproc_ops = {
	.start = adi_rproc_start,
	.stop = adi_rproc_stop,
	.kick = adi_rproc_kick,
	.load = adi_rproc_load,
	.da_to_va = adi_rproc_da_to_va,
	.parse_fw = adi_rproc_parse_fw,
	.find_loaded_rsc_table = adi_rproc_find_loaded_rsc_table,
	.sanity_check = adi_rproc_sanity_check,
	.get_boot_addr = adi_rproc_get_boot_addr,
};

static int adi_remoteproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adi_rproc_data *rproc_data;
	struct device_node *np = dev->of_node;
	struct device_node *node;
	struct of_phandle_args svect_args;
	struct rproc *rproc;
	struct resource *res;
	struct reserved_mem *rmem;
	u32 addr[2];
	int ret, core_id;
	const char *name;

	ret = of_property_read_string(np, "firmware-name", &name);
	if (ret) {
		dev_err(dev, "Unable to get firmware-name property\n");
		return ret;
	}

	ret = of_property_read_u32(np, "core-id", &core_id);
	if (ret) {
		dev_err(dev, "Unable to get core-id property\n");
		return ret;
	}

	rproc = rproc_alloc(dev, np->name, &adi_rproc_ops,
			    name, sizeof(*rproc_data));
	if (!rproc) {
		dev_err(dev, "Unable to allocate remoteproc\n");
		return -ENOMEM;
	}

	rproc_data = (struct adi_rproc_data *)rproc->priv;
	platform_set_drvdata(pdev, rproc);

	ret = of_parse_phandle_with_fixed_args(np, "adi,svect", 1, 0,
					       &svect_args);
	if (ret) {
		dev_err(dev, "Missing adi,svect property\n");
		goto free_rproc;
	}
	rproc_data->svect_regmap = syscon_node_to_regmap(svect_args.np);
	of_node_put(svect_args.np);
	if (IS_ERR(rproc_data->svect_regmap)) {
		dev_err(dev, "Unable to get SVECT regmap\n");
		ret = PTR_ERR(rproc_data->svect_regmap);
		goto free_rproc;
	}
	rproc_data->svect_offset = svect_args.args[0];

	rproc_data->rst_crst = devm_reset_control_get_exclusive(dev, "crst");
	if (IS_ERR(rproc_data->rst_crst)) {
		dev_err(dev, "Unable to get crst reset control\n");
		ret = PTR_ERR(rproc_data->rst_crst);
		goto free_rproc;
	}

	rproc_data->rst_start = devm_reset_control_get_exclusive(dev, "start");
	if (IS_ERR(rproc_data->rst_start)) {
		dev_err(dev, "Unable to get start reset control\n");
		ret = PTR_ERR(rproc_data->rst_start);
		goto free_rproc;
	}

	ret = reset_control_status(rproc_data->rst_start);
	if (ret < 0) {
		dev_err(dev, "Unable to read core status\n");
		goto free_rproc;
	} else if (ret == 0) {
		dev_err(dev, "Error: Core%d not idle\n", core_id);
		ret = -EBUSY;
		goto free_rproc;
	}

	rproc_data->kick_client.dev = dev;
	rproc_data->kick_client.tx_block = false;

	rproc_data->kick_chan = mbox_request_channel_byname(&rproc_data->kick_client,
							    "kick");
	if (IS_ERR(rproc_data->kick_chan)) {
		ret = PTR_ERR(rproc_data->kick_chan);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Unable to get kick mailbox channel\n");
		goto free_rproc;
	}

	/* for now device addresses are represented as 32 bits and expanded to 64
	 * here in driver code
	 */
	if (of_property_read_u32_array(np, "adi,l1-da", addr, 2)) {
		dev_err(dev, "Missing adi,l1-da with L1 device address range information\n");
		ret = -ENODEV;
		goto free_mbox;
	}
	rproc_data->l1_da_range[0] = addr[0];
	rproc_data->l1_da_range[1] = addr[1];

	if (of_property_read_u32_array(np, "adi,l2-da", addr, 2)) {
		dev_err(dev, "Missing adi,l2-da with L2 device address range information\n");
		ret = -ENODEV;
		goto free_mbox;
	}
	rproc_data->l2_da_range[0] = addr[0];
	rproc_data->l2_da_range[1] = addr[1];

	/* Get ADI resource table address */
	node = of_parse_phandle(np, "adi,rsc-table", 0);
	if (node) {
		dev_info(&pdev->dev, "Resource table set, enable rpmsg\n");
		rmem = of_reserved_mem_lookup(node);
		of_node_put(node);
		if (!rmem) {
			dev_err(&pdev->dev, "Translating adi,rsc-table failed\n");
			ret = -ENOMEM;
			goto free_mbox;
		}

		rproc_data->adi_rsc_table = devm_ioremap_wc(dev,
							    rmem->base,
							    rmem->size);
		if (!rproc_data->adi_rsc_table) {
			dev_err(dev, "Can't map adi,rsc-table\n");
			ret = -ENOMEM;
			goto free_mbox;
		}

		rproc_data->icc_irq = platform_get_irq(pdev, 0);
		if (rproc_data->icc_irq <= 0) {
			dev_err(dev, "No ICC IRQ specified\n");
			ret = -ENOENT;
			goto free_mbox;
		}

		rproc_data->icc_irq_flags = IRQF_PERCPU | IRQF_SHARED | IRQF_ONESHOT;

	} else {
		rproc_data->adi_rsc_table = NULL;
	}

	rproc_data->core_workqueue = alloc_workqueue("Core workqueue",
		WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!rproc_data->core_workqueue) {
		dev_err(dev, "Unable to allocate core workqueue\n");
		ret = -ENOMEM;
		goto free_mbox;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Cannot get L1 base address (reg 0)\n");
		ret = -ENODEV;
		goto free_workqueue;
	}
	rproc_data->L1_shared_base = devm_ioremap_wc(dev,
						     res->start,
						     resource_size(res));
	if (!rproc_data->L1_shared_base) {
		dev_err(dev, "Cannot map L1 shared memory\n");
		ret = -ENOMEM;
		goto free_workqueue;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(dev, "Cannot get L2 base address (reg 1)\n");
		ret = -ENODEV;
		goto free_workqueue;
	}
	rproc_data->L2_shared_base = devm_ioremap_wc(dev,
						     res->start,
						     resource_size(res));
	if (!rproc_data->L2_shared_base) {
		dev_err(dev, "Cannot map L2 shared memory\n");
		ret = -ENOMEM;
		goto free_workqueue;
	}

	rproc_data->verify = 0;
	of_property_read_u32(np, "adi,verify", &rproc_data->verify);
	rproc_data->verify = !!rproc_data->verify;
	if (rproc_data->verify)
		dev_info(dev, "Load verification enabled\n");

	rproc_data->dev = &pdev->dev;
	rproc_data->core_id = core_id;
	rproc_data->rproc = rproc;
	rproc_data->firmware_name = name;
	rproc_data->mem_virt = NULL;
	rproc_data->fw_size = 0;
	rproc_data->ldr_load_addr = SHARC_IDLE_ADDR;
	rproc_data->rpmsg_state = ADI_RP_RPMSG_TIMED_OUT;

	dmaengine_get();

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "Failed to add rproc\n");
		goto put_dmaengine;
	}

	return 0;

put_dmaengine:
	dmaengine_put();

free_workqueue:
	destroy_workqueue(rproc_data->core_workqueue);

free_mbox:
	mbox_free_channel(rproc_data->kick_chan);

free_rproc:
	rproc_free(rproc);

	return ret;
}

static void adi_remoteproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct adi_rproc_data *rproc_data = rproc->priv;

	rproc_del(rproc);
	dmaengine_put();
	destroy_workqueue(rproc_data->core_workqueue);
	mbox_free_channel(rproc_data->kick_chan);
	rproc_free(rproc);
}

static const struct of_device_id adi_rproc_of_match[] = {
	{ .compatible = "adi,remoteproc" },
	{ },
};
MODULE_DEVICE_TABLE(of, adi_rproc_of_match);

static struct platform_driver adi_rproc_driver = {
	.probe = adi_remoteproc_probe,
	.remove = adi_remoteproc_remove,
	.driver = {
		.name = "adi_remoteproc",
		.of_match_table = adi_rproc_of_match,
	},
};
module_platform_driver(adi_rproc_driver);

MODULE_DESCRIPTION("Analog Device sc5xx SHARC Image Loader");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Greg Chen <jian.chen@analog.com>");
MODULE_AUTHOR("Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>");
