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
#include <linux/unaligned.h>

#include <linux/soc/adi/icc.h>
#include <linux/soc/adi/spu.h>
#include "remoteproc_internal.h"
#include "remoteproc_elf_helpers.h"

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

#define SHARCFX_IRAM_ARM_OFFSET 0x07540000
#define SHARCFX_IRAM_START	0x2f800000
#define SHARCFX_IRAM_END	0x2f80ffff

/* SHARC+ L1 multiprocessor window offsets, per core (DS Table 4) */
#define SHARC1_MP_OFFSET	0x28000000
#define SHARC2_MP_OFFSET	0x28800000

/*
 * ".adi.attributes" is the Arm build attributes encoding with "AnonADI" as the
 * vendor name: a format byte, then one vendor subsection made of (u8 tag,
 * le32 length) sub-subsections. A Section sub-subsection lists the section
 * indices it describes as 0-terminated ULEB128s, followed by ULEB128 (tag,
 * value) pairs.
 */
#define SHT_ADI_ATTRIBUTES	(SHT_LOPROC + 2)
#define ADI_ATTR_SECTION_NAME	".adi.attributes"
#define ADI_ATTR_FORMAT_A	'A'
#define ADI_ATTR_VENDOR		"AnonADI"
#define ADI_ATTR_SUB_SECTION	2
#define ADI_ATTR_TAG_PART_NAME	4	/* the one string valued attribute */
#define ADI_ATTR_TAG_WORD_BITS	19	/* bits in one addressable word */

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

enum adi_rproc_variant {
	SC5XX_RPROC_SHARC,
	SC5XX_RPROC_SHARCFX,
};

struct adi_rproc_config {
	unsigned int variant;
};

#define WORD_SCALE_8 1
#define WORD_SCALE_16 2
#define WORD_SCALE_32 4
#define WORD_SCALE_48 6
#define WORD_SCALE_64 8

struct sharcp_space {
	uint32_t start;
	uint32_t end;
	uint32_t byte_base;
	uint8_t l1;
	uint8_t scale;
	uint8_t bits;
};

static const struct sharcp_space sharcp_spaces[] = {
	/* L1 block 0 */
	{ 0x00048000, 0x0004dfff, 0x00240000, 1, WORD_SCALE_64, 64 },
	{ 0x00090000, 0x00097fff, 0x00240000, 1, WORD_SCALE_48, 48 },
	{ 0x00090000, 0x0009bfff, 0x00240000, 1, WORD_SCALE_32, 32 },
	{ 0x00120000, 0x00137fff, 0x00240000, 1, WORD_SCALE_16, 16 },
	{ 0x00240000, 0x0026ffff, 0x00240000, 1, WORD_SCALE_8,  8  },
	/* L1 block 1 */
	{ 0x00058000, 0x0005dfff, 0x002c0000, 1, WORD_SCALE_64, 64 },
	{ 0x000b0000, 0x000b7fff, 0x002c0000, 1, WORD_SCALE_48, 48 },
	{ 0x000b0000, 0x000bbfff, 0x002c0000, 1, WORD_SCALE_32, 32 },
	{ 0x00160000, 0x00177fff, 0x002c0000, 1, WORD_SCALE_16, 16 },
	{ 0x002c0000, 0x002effff, 0x002c0000, 1, WORD_SCALE_8,  8  },
	/* L1 block 2 */
	{ 0x00060000, 0x00063fff, 0x00300000, 1, WORD_SCALE_64, 64 },
	{ 0x000c0000, 0x000c5554, 0x00300000, 1, WORD_SCALE_48, 48 },
	{ 0x000c0000, 0x000c7fff, 0x00300000, 1, WORD_SCALE_32, 32 },
	{ 0x00180000, 0x0018ffff, 0x00300000, 1, WORD_SCALE_16, 16 },
	{ 0x00300000, 0x0031ffff, 0x00300000, 1, WORD_SCALE_8,  8  },
	/* L1 block 3 */
	{ 0x00070000, 0x00073fff, 0x00380000, 1, WORD_SCALE_64, 64 },
	{ 0x000e0000, 0x000e5554, 0x00380000, 1, WORD_SCALE_48, 48 },
	{ 0x000e0000, 0x000e7fff, 0x00380000, 1, WORD_SCALE_32, 32 },
	{ 0x001c0000, 0x001cffff, 0x00380000, 1, WORD_SCALE_16, 16 },
	{ 0x00380000, 0x0039ffff, 0x00380000, 1, WORD_SCALE_8,  8  },
	/* L2, shared between the cores, no multiprocessor offset */
	{ 0x00580000, 0x005d5554, 0x20000000, 0, WORD_SCALE_48, 48 }, // ? verify
	{ 0x08000000, 0x0807ffff, 0x20000000, 0, WORD_SCALE_32, 32 },
	{ 0x00b00000, 0x00bfffff, 0x20000000, 0, WORD_SCALE_16, 16 },
	{ 0x20000000, 0x201fffff, 0x20000000, 0, WORD_SCALE_8,  8  },
};

/* Word width of one allocated ELF section, as .adi.attributes records it */
struct sharcp_section {
	u32 addr;
	u32 bits;
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
	/*
	 * Physical bases and sizes matching L1_shared_base/L2_shared_base. MDMA
	 * works on physical addresses, not the ioremapped ones, so the ELF
	 * loader needs both forms of the same window.
	 */
	phys_addr_t l1_phys_base;
	phys_addr_t l2_phys_base;
	size_t l1_size;
	size_t l2_size;
	struct workqueue_struct *core_workqueue;
	enum adi_rproc_rpmsg_state rpmsg_state;
	u64 l1_da_range[2];
	u64 l2_da_range[2];
	u32 verify;
	struct adi_sharc_resource_table *adi_rsc_table;
	struct sharc_resource_table *loaded_rsc_table;
	/* True when the resource table came from the firmware image itself
	 * rather than from the driver's built-in template.
	 */
	bool rsc_table_from_fw;
	struct adi_rproc_config cfg;
	/*
	 * SHARC+ only: word width of each section of the ELF image loaded,
	 * which its device addresses cannot be translated without. Kept from
	 * load to stop, as the resource table is looked up after the load.
	 */
	struct sharcp_section *sections;
	unsigned int num_sections;
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
		return ADI_FW_ELF;
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

/* Read one ULEB128 from [*p, end), failing rather than truncating to 32 bits */
static bool adi_attr_uleb128(const u8 **p, const u8 *end, u32 *val)
{
	unsigned int shift = 0;
	u32 v = 0;
	u8 byte;

	do {
		if (*p >= end || shift > 28)
			return false;

		byte = *(*p)++;
		if (shift == 28 && (byte & 0x70))
			return false;

		v |= (u32)(byte & 0x7f) << shift;
		shift += 7;
	} while (byte & 0x80);

	*val = v;
	return true;
}

/*
 * adi_attr_parse: record the word width of each section .adi.attributes covers
 *
 * @bits is indexed by ELF section number and has @shnum entries. Only Section
 * sub-subsections are read; skipping the rest also steps over the File
 * sub-subsection and the part name in it.
 */
static int adi_attr_parse(const u8 *attr, size_t size, u32 *bits,
			  unsigned int shnum)
{
	const size_t vendor_len = sizeof(ADI_ATTR_VENDOR);
	const size_t hdr_len = 1 + sizeof(u32);
	const u8 *p, *end;
	u32 len;

	if (size < hdr_len || attr[0] != ADI_ATTR_FORMAT_A)
		return -EINVAL;

	/* The vendor subsection length counts itself but not the format byte */
	len = get_unaligned_le32(attr + 1);
	if (len < sizeof(u32) + vendor_len || len > size - 1)
		return -EINVAL;

	p = attr + hdr_len;
	end = attr + 1 + len;

	if (memcmp(p, ADI_ATTR_VENDOR, vendor_len))
		return -EINVAL;
	p += vendor_len;

	while ((size_t)(end - p) >= hdr_len) {
		u32 sub_len = get_unaligned_le32(p + 1);
		const u8 *q = p + hdr_len, *list, *sub_end;
		u32 idx, tag, val, word_bits = 0;
		u8 sub_tag = p[0];

		if (sub_len < hdr_len || sub_len > (size_t)(end - p))
			return -EINVAL;

		sub_end = p + sub_len;
		p = sub_end;

		if (sub_tag != ADI_ATTR_SUB_SECTION)
			continue;

		/* The 0-terminated list of sections this one describes */
		list = q;
		do {
			if (!adi_attr_uleb128(&q, sub_end, &idx))
				return -EINVAL;
		} while (idx);

		while (q < sub_end) {
			if (!adi_attr_uleb128(&q, sub_end, &tag))
				return -EINVAL;

			if (tag == ADI_ATTR_TAG_PART_NAME) {
				q += strnlen((const char *)q, sub_end - q) + 1;
				continue;
			}

			if (!adi_attr_uleb128(&q, sub_end, &val))
				return -EINVAL;

			if (tag == ADI_ATTR_TAG_WORD_BITS)
				word_bits = val;
		}

		if (!word_bits)
			continue;

		while (adi_attr_uleb128(&list, sub_end, &idx) && idx) {
			if (idx < shnum)
				bits[idx] = word_bits;
		}
	}

	return 0;
}

static void sharcp_free_sections(struct adi_rproc_data *rproc_data)
{
	kfree(rproc_data->sections);
	rproc_data->sections = NULL;
	rproc_data->num_sections = 0;
}

/*
 * sharcp_parse_sections: collect the word width of every allocated section
 *
 * SHARC+ addresses count words, not bytes, and the 48-bit instruction space
 * overlays the 32-bit data space, so nothing in an address says how to scale
 * it: only the image's .adi.attributes section does. The ADI linker emits one
 * section per loadable segment with sh_addr matching p_paddr, so recording
 * the widths by section address lets a segment address be looked up directly.
 */
static int sharcp_parse_sections(struct adi_rproc_data *rproc_data,
				 const struct firmware *fw)
{
	const u8 *elf_data = fw->data;
	u8 class = fw_elf_get_class(fw);
	size_t shdr_size = elf_size_of_shdr(class);
	u64 shoff, strtab_off, strtab_size, attr_off, attr_size;
	const void *shdr, *shstr, *attr = NULL;
	struct sharcp_section *sections;
	unsigned int i, n = 0;
	u16 shnum, shstrndx;
	u32 *bits;
	int ret;

	sharcp_free_sections(rproc_data);

	shoff = elf_hdr_get_e_shoff(class, elf_data);
	shnum = elf_hdr_get_e_shnum(class, elf_data);
	shstrndx = elf_hdr_get_e_shstrndx(class, elf_data);

	if (!shnum || shstrndx >= shnum || shoff > fw->size ||
	    (u64)shnum * shdr_size > fw->size - shoff)
		return -EINVAL;

	shdr = elf_data + shoff;
	shstr = shdr + shstrndx * shdr_size;
	strtab_off = elf_shdr_get_sh_offset(class, shstr);
	strtab_size = elf_shdr_get_sh_size(class, shstr);
	if (strtab_off > fw->size || strtab_size > fw->size - strtab_off)
		return -EINVAL;

	for (i = 0; i < shnum; i++) {
		const void *s = shdr + i * shdr_size;
		u32 name = elf_shdr_get_sh_name(class, s);

		if (elf_shdr_get_sh_type(class, s) != SHT_ADI_ATTRIBUTES ||
		    name >= strtab_size ||
		    strtab_size - name < sizeof(ADI_ATTR_SECTION_NAME))
			continue;

		if (!memcmp(elf_data + strtab_off + name, ADI_ATTR_SECTION_NAME,
			    sizeof(ADI_ATTR_SECTION_NAME))) {
			attr = s;
			break;
		}
	}

	if (!attr)
		return -ENOENT;

	attr_off = elf_shdr_get_sh_offset(class, attr);
	attr_size = elf_shdr_get_sh_size(class, attr);
	if (attr_off > fw->size || attr_size > fw->size - attr_off)
		return -EINVAL;

	bits = kcalloc(shnum, sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	ret = adi_attr_parse(elf_data + attr_off, attr_size, bits, shnum);
	if (ret)
		goto free_bits;

	sections = kcalloc(shnum, sizeof(*sections), GFP_KERNEL);
	if (!sections) {
		ret = -ENOMEM;
		goto free_bits;
	}

	for (i = 0; i < shnum; i++) {
		const void *s = shdr + i * shdr_size;

		if (!bits[i] || !(elf_shdr_get_sh_flags(class, s) & SHF_ALLOC) ||
		    !elf_shdr_get_sh_size(class, s))
			continue;

		sections[n].addr = elf_shdr_get_sh_addr(class, s);
		sections[n].bits = bits[i];
		n++;
	}

	rproc_data->sections = sections;
	rproc_data->num_sections = n;

free_bits:
	kfree(bits);
	return ret;
}

static u32 sharcp_section_bits(struct adi_rproc_data *rproc_data, u64 da)
{
	unsigned int i;

	for (i = 0; i < rproc_data->num_sections; i++) {
		if (rproc_data->sections[i].addr == da)
			return rproc_data->sections[i].bits;
	}

	return 0;
}

/*
 * sharcp_da_to_pa: translate a SHARC+ word address into an Arm byte address
 *
 * Each space in sharcp_spaces[] starts at its block's byte base, scaled down
 * by the space's word size. Core-private L1 is then reached through that
 * core's multiprocessor window; L2 is shared and needs no offset.
 */
static phys_addr_t sharcp_da_to_pa(struct adi_rproc_data *rproc_data, u64 da,
				   u32 bits, unsigned int *word)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sharcp_spaces); i++) {
		const struct sharcp_space *sp = &sharcp_spaces[i];
		phys_addr_t pa;

		if (sp->bits != bits || da < sp->start || da > sp->end)
			continue;

		pa = sp->byte_base + (da - sp->start) * sp->scale;
		if (sp->l1)
			pa += rproc_data->core_id == 2 ? SHARC2_MP_OFFSET :
							 SHARC1_MP_OFFSET;

		*word = sp->scale;
		return pa;
	}

	return 0;
}

/*
 * adi_rproc_da_to_pa: translate a core device address into an Arm physical one
 *
 * @word is set to the size in bytes of one addressable word at @da. Returns 0
 * if @da cannot be translated.
 */
static phys_addr_t adi_rproc_da_to_pa(struct adi_rproc_data *rproc_data,
				      u64 da, unsigned int *word)
{
	u32 bits;

	*word = WORD_SCALE_8;

	switch (rproc_data->cfg.variant) {
	case SC5XX_RPROC_SHARCFX:
		if (da >= rproc_data->l1_da_range[0] && da < rproc_data->l1_da_range[1])
			return rproc_data->l1_phys_base + (da - rproc_data->l1_da_range[0]);
		if (da >= rproc_data->l2_da_range[0] && da < rproc_data->l2_da_range[1])
			return rproc_data->l2_phys_base + (da - rproc_data->l2_da_range[0]);
		return 0;
	case SC5XX_RPROC_SHARC:
		bits = sharcp_section_bits(rproc_data, da);
		if (!bits)
			return 0;
		return sharcp_da_to_pa(rproc_data, da, bits, word);
	}

	return 0;
}

static bool adi_rproc_in_window(phys_addr_t pa, size_t len,
				phys_addr_t base, size_t size)
{
	return pa >= base && len <= size && pa - base <= size - len;
}

/* Map a range inside one of the two DT "reg" windows to its ioremapped VA */
static void __iomem *adi_rproc_pa_to_va(struct adi_rproc_data *rproc_data,
					phys_addr_t pa, size_t len)
{
	if (adi_rproc_in_window(pa, len, rproc_data->l1_phys_base,
				rproc_data->l1_size))
		return rproc_data->L1_shared_base + (pa - rproc_data->l1_phys_base);

	if (adi_rproc_in_window(pa, len, rproc_data->l2_phys_base,
				rproc_data->l2_size))
		return rproc_data->L2_shared_base + (pa - rproc_data->l2_phys_base);

	return NULL;
}

/*
 * sharcp_swap_words: copy @len bytes, byte-reversing each @word byte word
 *
 * The .dxe stores every word of a word-addressed SHARC+ space in the opposite
 * byte order to the one the Arm byte window presents. CCES's elfloader applies
 * that swap when it builds a .ldr, which is why the LDR path can copy blocks
 * verbatim; an ELF has to be swapped here, or the core resets to a valid SVECT,
 * fetches byte-reversed instructions and silently does nothing. A trailing
 * partial word is copied as is.
 */
static void sharcp_swap_words(u8 *dst, const u8 *src, size_t len,
			      unsigned int word)
{
	unsigned int i;
	size_t off;

	for (off = 0; off + word <= len; off += word) {
		for (i = 0; i < word; i++)
			dst[off + word - 1 - i] = src[off + i];
	}

	memcpy(dst + off, src + off, len - off);
}

/*
 * adi_rproc_dma_write: copy a buffer to a SHARC physical address using MDMA
 *
 * The SHARC-FX I-completer rejects 8- and 16-bit accesses to IRAM, so the
 * ARM cannot memcpy() into that window: the optimised memcpy tail emits byte
 * and halfword stores and the fabric answers with an SError. MDMA issues
 * naturally aligned bursts instead, which is also how the LDR path loads
 * every block. @src must be a kernel buffer suitable for streaming DMA.
 *
 * @word is the size in bytes of one word at @dst; wider words are byte
 * swapped on the way, see sharcp_swap_words().
 */
static int adi_rproc_dma_write(struct adi_rproc_data *rproc_data,
			       phys_addr_t dst, const void *src, size_t len,
			       unsigned int word)
{
	struct dma_async_tx_descriptor *tx;
	struct dma_chan *chan;
	struct completion cmp;
	dma_addr_t src_handle;
	dma_cookie_t cookie;
	void *bounce;
	int ret = 0;

	chan = dma_find_channel(DMA_MEMCPY);
	if (!chan) {
		dev_err(rproc_data->dev, "Could not find dma memcpy channel\n");
		return -ENODEV;
	}

	/*
	 * fw->data is vmalloc'ed, which cannot be mapped for streaming DMA,
	 * so stage the segment through a coherent bounce buffer.
	 */
	bounce = dma_alloc_coherent(rproc_data->dev, len, &src_handle, GFP_KERNEL);
	if (!bounce)
		return -ENOMEM;

	if (word > WORD_SCALE_8)
		sharcp_swap_words(bounce, src, len, word);
	else
		memcpy(bounce, src, len);

	init_completion(&cmp);

	tx = dmaengine_prep_dma_memcpy(chan, dst, src_handle, len, 0);
	if (!tx) {
		dev_err(rproc_data->dev, "Failed to allocate dma transaction\n");
		ret = -ENOMEM;
		goto free_bounce;
	}

	tx->callback = load_callback;
	tx->callback_param = &cmp;

	cookie = dmaengine_submit(tx);
	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(rproc_data->dev, "Failed to submit dma transaction\n");
		goto free_bounce;
	}

	dma_async_issue_pending(chan);

	if (!wait_for_completion_timeout(&cmp, CORE_INIT_TIMEOUT)) {
		dev_err(rproc_data->dev, "Timed out waiting for dma to %pa\n", &dst);
		dmaengine_terminate_sync(chan);
		ret = -ETIMEDOUT;
	}

free_bounce:
	dma_free_coherent(rproc_data->dev, len, bounce, src_handle);
	return ret;
}

/*
 * adi_rproc_dma_set: fill a SHARC physical range with a byte value using MDMA
 */
static int adi_rproc_dma_set(struct adi_rproc_data *rproc_data,
			     phys_addr_t dst, int value, size_t len)
{
	struct dma_async_tx_descriptor *tx;
	struct dma_chan *chan;
	struct completion cmp;
	dma_cookie_t cookie;
	int ret;

	chan = dma_find_channel(DMA_MEMCPY);
	if (!chan) {
		dev_err(rproc_data->dev, "Could not find dma memcpy channel\n");
		return -ENODEV;
	}

	init_completion(&cmp);

	tx = dmaengine_prep_dma_memset(chan, dst, value, len, 0);
	if (!tx) {
		dev_err(rproc_data->dev, "Failed to allocate dma transaction\n");
		return -ENOMEM;
	}

	tx->callback = load_callback;
	tx->callback_param = &cmp;

	cookie = dmaengine_submit(tx);
	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(rproc_data->dev, "Failed to submit dma transaction\n");
		return ret;
	}

	dma_async_issue_pending(chan);

	if (!wait_for_completion_timeout(&cmp, CORE_INIT_TIMEOUT)) {
		dev_err(rproc_data->dev, "Timed out waiting for dma to %pa\n", &dst);
		dmaengine_terminate_sync(chan);
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * adi_elf_load_segments: load ELF PT_LOAD segments over MDMA
 *
 * Mirrors rproc_elf_load_segments() but routes every write through MDMA
 * rather than memcpy(), because IRAM cannot take narrow accesses from the
 * ARM. See adi_rproc_dma_write().
 *
 * The SPU is held open for the duration of the load, as the LDR path does:
 * the SHARC-FX bus completer ports reject non-secure accesses with an error
 * response rather than completing them.
 *
 * SHARC+ segment addresses are word addresses whose width only the image's
 * .adi.attributes section records, so that is parsed first; see
 * sharcp_parse_sections().
 */
static int adi_elf_load_segments(struct rproc *rproc, const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	const u8 *elf_data = fw->data;
	u8 class = fw_elf_get_class(fw);
	u32 elf_phdr_get_size = elf_size_of_phdr(class);
	struct device *dev = &rproc->dev;
	const void *ehdr, *phdr;
	int i, ret = 0;
	u16 phnum;

	ehdr = elf_data;
	phnum = elf_hdr_get_e_phnum(class, ehdr);
	phdr = elf_data + elf_hdr_get_e_phoff(class, ehdr);

	if (rproc_data->cfg.variant == SC5XX_RPROC_SHARC) {
		ret = sharcp_parse_sections(rproc_data, fw);
		if (ret) {
			dev_err(dev, "failed to parse ADI ELF attributes: %d\n", ret);
			return ret;
		}
	}

	enable_spu();

	for (i = 0; i < phnum; i++, phdr += elf_phdr_get_size) {
		u64 da = elf_phdr_get_p_paddr(class, phdr);
		u64 memsz = elf_phdr_get_p_memsz(class, phdr);
		u64 filesz = elf_phdr_get_p_filesz(class, phdr);
		u64 offset = elf_phdr_get_p_offset(class, phdr);
		u32 type = elf_phdr_get_p_type(class, phdr);
		unsigned int word;
		phys_addr_t pa;

		if (type != PT_LOAD || !memsz)
			continue;

		dev_dbg(dev, "phdr: type %d da 0x%llx memsz 0x%llx filesz 0x%llx\n",
			type, da, memsz, filesz);

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%llx memsz 0x%llx\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%llx avail 0x%zx\n",
				offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		if (!rproc_u64_fit_in_size_t(memsz)) {
			dev_err(dev, "size (%llx) does not fit in size_t type\n",
				memsz);
			ret = -EOVERFLOW;
			break;
		}

		/* Only DMA into the windows the DT gives this core */
		pa = adi_rproc_da_to_pa(rproc_data, da, &word);
		if (!pa || !adi_rproc_pa_to_va(rproc_data, pa, memsz)) {
			dev_err(dev, "bad phdr da 0x%llx mem 0x%llx\n", da, memsz);
			ret = -EINVAL;
			break;
		}

		if (filesz) {
			ret = adi_rproc_dma_write(rproc_data, pa,
						  elf_data + offset, filesz, word);
			if (ret) {
				dev_err(dev, "dma copy failed for da 0x%llx memsz 0x%llx\n",
					da, memsz);
				break;
			}
		}

		/* Zero the .bss-style tail the image does not carry */
		if (memsz > filesz) {
			ret = adi_rproc_dma_set(rproc_data, pa + filesz, 0,
						memsz - filesz);
			if (ret) {
				dev_err(dev, "dma memset failed for da 0x%llx memsz 0x%llx\n",
					da, memsz);
				break;
			}
		}
	}

	disable_spu();

	return ret;
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
		ret = adi_elf_load_segments(rproc, fw);
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
	unsigned long svect = rproc_data->ldr_load_addr;
	int ret;

	/*
	 * The LDR parser records the first block's target address as it walks
	 * the image, so ldr_load_addr is already correct for that format. An
	 * ELF has no equivalent pass, and without this ldr_load_addr would
	 * still hold SHARC_IDLE_ADDR from probe, leaving the core spinning in
	 * the bootrom idle loop instead of running the loaded firmware. Use
	 * the entry point remoteproc resolved through .get_boot_addr.
	 */
	if (rproc_data->firmware_format == ADI_FW_ELF) {
		if (!rproc->bootaddr) {
			dev_err(rproc_data->dev, "No entry point for ELF firmware\n");
			return -EINVAL;
		}
		svect = (unsigned long)rproc->bootaddr;
	}

	dev_dbg(rproc_data->dev, "starting core%d at 0x%lx\n",
		rproc_data->core_id, svect);

	ret = adi_core_set_svect(rproc_data, svect);
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
	sharcp_free_sections(rproc_data);
	return ret;
}

/*
 * Build the resource table from the driver's built-in template.
 *
 * LDR images have no resource table section at all, and ELF images are not
 * required to carry one either, so in both cases the table is synthesised
 * here. The layout matches struct sharc_resource_table, which the carveout
 * setup in adi_rproc_parse_fw() relies on.
 */
static int adi_rsc_table_from_template(struct rproc *rproc)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	size_t size = sizeof(_rsc_table_template.rsc_table);
	struct sharc_resource_table *table;
	u32 notifyid;

	/* kfree in remoteproc_core.c */
	rproc->cached_table = kmemdup(&_rsc_table_template.rsc_table, size, GFP_KERNEL);
	if (!rproc->cached_table)
		return -ENOMEM;

	/* Notify id is the core the vdev belongs to */
	notifyid = (rproc_data->core_id == 1) ? 1 : 2;

	table = (struct sharc_resource_table *)rproc->cached_table;
	table->rpmsg_vdev.notifyid = notifyid;
	table->vring[0].notifyid = notifyid;
	table->vring[1].notifyid = notifyid;

	/* Initialize ADI resource table header*/
	rproc_data->adi_rsc_table->adi_table_hdr = _rsc_table_template.adi_table_hdr;

	rproc->table_ptr = rproc->cached_table;
	rproc->table_sz = size;
	rproc_data->rsc_table_from_fw = false;

	return 0;
}

static int adi_ldr_load_rsc_table(struct rproc *rproc, const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;

	if (rproc_data->adi_rsc_table == NULL)
		return -EINVAL;

	return adi_rsc_table_from_template(rproc);
}

/*
 * Load the resource table from an ELF image.
 *
 * Firmware built with a .resource_table section is used as-is. Images without
 * one (for example the SHARC-FX CCES default projects) fall back to the
 * built-in template so that vring carveouts and rpmsg still come up exactly
 * as they do for LDR images.
 */
static int adi_elf_load_rsc_table(struct rproc *rproc, const struct firmware *fw)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	int ret;

	if (rproc_data->adi_rsc_table == NULL)
		return -EINVAL;

	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (!ret) {
		rproc_data->rsc_table_from_fw = true;

		/*
		 * The carveout setup writes through struct sharc_resource_table,
		 * so a firmware supplied table has to be at least that large.
		 */
		if (rproc->table_sz < sizeof(struct sharc_resource_table)) {
			dev_err(&rproc->dev,
				"firmware resource table too small (%zu < %zu)\n",
				rproc->table_sz,
				sizeof(struct sharc_resource_table));
			kfree(rproc->cached_table);
			rproc->cached_table = NULL;
			rproc->table_ptr = NULL;
			rproc->table_sz = 0;
			return -EINVAL;
		}

		return 0;
	}

	dev_warn(&rproc->dev,
		 "no resource table in firmware, using built-in template\n");

	return adi_rsc_table_from_template(rproc);
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
		ret = adi_elf_load_rsc_table(rproc, fw);
		break;
	default:
		WARN(1, "Invalid rproc_data->firmware_format\n");
		return -EINVAL;
	}

	if (ret < 0) {
		return ret;
	}

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
			/*
			 * Calc how many buffers we can fit in the vring region,
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

	/*
	 * Follow whichever table was actually loaded rather than the firmware
	 * format: an ELF image without a .resource_table section falls back to
	 * the built-in template, so it has to be looked up the same way an LDR
	 * image is.
	 */
	if (rproc_data->rsc_table_from_fw)
		ret = rproc_elf_find_loaded_rsc_table(rproc, fw);
	else
		ret = adi_ldr_find_loaded_rsc_table(rproc, fw);
	rproc_data->loaded_rsc_table = (struct sharc_resource_table *)ret;
	return ret;
}

/*
 * @todo store number of vrings from resource table and use it to dynamically
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
	unsigned int word;
	phys_addr_t pa;

	if (len == 0)
		return NULL;

	pa = adi_rproc_da_to_pa(rproc_data, da, &word);
	if (!pa)
		return NULL;

	return (void __force *)adi_rproc_pa_to_va(rproc_data, pa, len);
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
	const struct adi_rproc_config *cfg;
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

	cfg = of_device_get_match_data(dev);
	if (!cfg)
		return dev_err_probe(dev, -ENODEV, "No variant configuration\n");

	ret = of_property_read_string(np, "firmware-name", &name);
	if (ret)
		return dev_err_probe(dev, ret, "Unable to get firmware-name property\n");

	ret = of_property_read_u32(np, "core-id", &core_id);
	if (ret)
		return dev_err_probe(dev, ret, "Unable to get core-id property\n");

	rproc = devm_rproc_alloc(dev, np->name, &adi_rproc_ops,
				 name, sizeof(*rproc_data));
	if (!rproc)
		return -ENOMEM;

	rproc_data = (struct adi_rproc_data *)rproc->priv;
	rproc_data->cfg = *cfg;
	platform_set_drvdata(pdev, rproc);

	ret = of_parse_phandle_with_fixed_args(np, "adi,svect", 1, 0,
					       &svect_args);
	if (ret)
		return dev_err_probe(dev, ret, "Missing adi,svect property\n");
	rproc_data->svect_regmap = syscon_node_to_regmap(svect_args.np);
	of_node_put(svect_args.np);
	if (IS_ERR(rproc_data->svect_regmap))
		return dev_err_probe(dev, PTR_ERR(rproc_data->svect_regmap),
				     "Unable to get SVECT regmap\n");

	rproc_data->svect_offset = svect_args.args[0];

	rproc_data->rst_crst = devm_reset_control_get_exclusive(dev, "crst");
	if (IS_ERR(rproc_data->rst_crst))
		return dev_err_probe(dev, PTR_ERR(rproc_data->rst_crst),
				     "Unable to get crst reset control\n");

	rproc_data->rst_start = devm_reset_control_get_exclusive(dev, "start");
	if (IS_ERR(rproc_data->rst_start))
		return dev_err_probe(dev, PTR_ERR(rproc_data->rst_start),
				     "Unable to get start reset control\n");

	ret = reset_control_status(rproc_data->rst_start);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Unable to read core status\n");
	else if (ret == 0)
		return dev_err_probe(dev, -EBUSY,
				     "Error: Core%d not idle\n", core_id);

	rproc_data->kick_client.dev = dev;
	rproc_data->kick_client.tx_block = false;

	rproc_data->kick_chan = mbox_request_channel_byname(&rproc_data->kick_client,
							    "kick");
	if (IS_ERR(rproc_data->kick_chan))
		return dev_err_probe(dev, PTR_ERR(rproc_data->kick_chan),
				     "Unable to get kick mailbox channel\n");

	/*
	 * for now device addresses are represented as 32 bits and expanded to 64
	 * here in driver code
	 */
	if (of_property_read_u32_array(np, "adi,l1-da", addr, 2)) {
		ret = dev_err_probe(dev, -ENODEV,
				    "Missing adi,l1-da with L1 device address range information\n");
		goto free_mbox;
	}
	rproc_data->l1_da_range[0] = addr[0];
	rproc_data->l1_da_range[1] = addr[1];

	if (of_property_read_u32_array(np, "adi,l2-da", addr, 2)) {
		ret = dev_err_probe(dev, -ENODEV,
				    "Missing adi,l2-da with L2 device address range information\n");
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
		if (!rmem)
			goto free_mbox;

		rproc_data->adi_rsc_table = devm_ioremap_wc(dev,
							    rmem->base,
							    rmem->size);
		if (!rproc_data->adi_rsc_table) {
			ret = -ENOMEM;
			goto free_mbox;
		}

		rproc_data->icc_irq = platform_get_irq(pdev, 0);
		if (rproc_data->icc_irq <= 0) {
			ret = dev_err_probe(dev, -ENOENT, "No ICC IRQ specified\n");
			goto free_mbox;
		}

		rproc_data->icc_irq_flags = IRQF_PERCPU | IRQF_SHARED | IRQF_ONESHOT;
	} else {
		rproc_data->adi_rsc_table = NULL;
	}

	rproc_data->core_workqueue = alloc_workqueue("Core workqueue",
		WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!rproc_data->core_workqueue)
		goto free_mbox;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = dev_err_probe(dev, -ENODEV, "Cannot get L1 base address (reg 0)\n");
		goto free_workqueue;
	}

	rproc_data->L1_shared_base = devm_ioremap_wc(dev,
						     res->start,
						     resource_size(res));
	if (!rproc_data->L1_shared_base) {
		ret = -ENOMEM;
		goto free_workqueue;
	}
	rproc_data->l1_phys_base = res->start;
	rproc_data->l1_size = resource_size(res);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		ret = dev_err_probe(dev, -ENODEV, "Cannot get L2 base address (reg 1)\n");
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
	rproc_data->l2_phys_base = res->start;
	rproc_data->l2_size = resource_size(res);

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
		dev_err_probe(dev, ret, "Failed to add rproc\n");
		goto put_dmaengine;
	}

	return 0;

put_dmaengine:
	dmaengine_put();

free_workqueue:
	destroy_workqueue(rproc_data->core_workqueue);

free_mbox:
	mbox_free_channel(rproc_data->kick_chan);

	return ret;
}

static void adi_remoteproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct adi_rproc_data *rproc_data = rproc->priv;

	rproc_del(rproc);
	sharcp_free_sections(rproc_data);
	dmaengine_put();
	destroy_workqueue(rproc_data->core_workqueue);
	mbox_free_channel(rproc_data->kick_chan);
}

static const struct adi_rproc_config sc5xx_rproc_cfg = {
	.variant = SC5XX_RPROC_SHARC,
};

static const struct adi_rproc_config sc8xx_rproc_cfg = {
	.variant = SC5XX_RPROC_SHARCFX,
};

static const struct of_device_id adi_rproc_of_match[] = {
	{ .compatible = "adi,sc5xx-rproc", .data = &sc5xx_rproc_cfg },
	{ .compatible = "adi,sc8xx-rproc", .data = &sc8xx_rproc_cfg },
	{ }
};
MODULE_DEVICE_TABLE(of, adi_rproc_of_match);

static struct platform_driver adi_rproc_driver = {
	.probe = adi_remoteproc_probe,
	.remove = adi_remoteproc_remove,
	.driver = {
		.name = "adi-rproc",
		.of_match_table = adi_rproc_of_match,
	},
};
module_platform_driver(adi_rproc_driver);

MODULE_DESCRIPTION("Analog Device sc5xx SHARC Image Loader");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Greg Chen <jian.chen@analog.com>");
MODULE_AUTHOR("Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>");
