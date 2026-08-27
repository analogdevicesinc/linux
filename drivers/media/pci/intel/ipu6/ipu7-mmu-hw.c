// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Intel Corporation
 */

#include <linux/types.h>
#include <linux/iopoll.h>

#include "ipu6.h"
#include "ipu6-dma.h"
#include "ipu6-mmu.h"

static struct ipu7_mmu_hw ipu7_isys_mmu_hwdata[] = {
	{
		.offset = IPU7_IS_MMU_FW_RD_OFFSET,
		.zlx_offset = IPU7_IS_ZLX_UC_RD_OFFSET,
		.uao_offset = IPU7_IS_UAO_UC_RD_OFFSET,
		.info_bits = 0x20006701,
		.refill = 0x00002726,
		.collapse_en_bitmap = 0x0,
		.l1_block = IPU7_IS_MMU_FW_RD_L1_BLOCKNR_REG,
		.l2_block = IPU7_IS_MMU_FW_RD_L2_BLOCKNR_REG,
		.nr_l1streams = IPU7_IS_MMU_FW_RD_STREAM_NUM,
		.nr_l2streams = IPU7_IS_MMU_FW_RD_STREAM_NUM,
		.l1_block_sz = { 0x0, 0x8, 0xa },
		.l2_block_sz = { 0x0, 0x2, 0x4 },
		.zlx_nr = IPU7_IS_ZLX_UC_RD_NUM,
		.zlx_axi_pool = { 0x00000f30 },
		.zlx_en = { 0, 0, 0, 0 },
		.zlx_conf = { 0x0, 0x0, 0x0, 0x0 },
		.uao_p_num = IPU7_IS_UAO_UC_RD_PLANENUM,
		.uao_p2tlb = { 0x61, 0x64, 0x65 },
	},
	{
		.offset = IPU7_IS_MMU_FW_WR_OFFSET,
		.zlx_offset = IPU7_IS_ZLX_UC_WR_OFFSET,
		.uao_offset = IPU7_IS_UAO_UC_WR_OFFSET,
		.info_bits = 0x20006801,
		.refill = 0x00002524,
		.collapse_en_bitmap = 0x0,
		.l1_block = IPU7_IS_MMU_FW_WR_L1_BLOCKNR_REG,
		.l2_block = IPU7_IS_MMU_FW_WR_L2_BLOCKNR_REG,
		.nr_l1streams = IPU7_IS_MMU_FW_WR_STREAM_NUM,
		.nr_l2streams = IPU7_IS_MMU_FW_WR_STREAM_NUM,
		.l1_block_sz = { 0x0, 0x8, 0xa },
		.l2_block_sz = { 0x0, 0x2, 0x4	},
		.zlx_nr = IPU7_IS_ZLX_UC_WR_NUM,
		.zlx_axi_pool = { 0x00000f20 },
		.zlx_en = { 0, 1, 1, 0 },
		.zlx_conf = { 0x0, 0x00010101, 0x00010101 },
		.uao_p_num = IPU7_IS_UAO_UC_WR_PLANENUM,
		.uao_p2tlb = { 0x61, 0x62, 0x63 },
	},
	{
		.offset = IPU7_IS_MMU_M0_OFFSET,
		.zlx_offset = IPU7_IS_ZLX_M0_OFFSET,
		.uao_offset = IPU7_IS_UAO_M0_WR_OFFSET,
		.info_bits = 0x20006601,
		.refill = 0x00002120,
		.collapse_en_bitmap = 0x0,
		.l1_block = IPU7_IS_MMU_M0_L1_BLOCKNR_REG,
		.l2_block = IPU7_IS_MMU_M0_L2_BLOCKNR_REG,
		.nr_l1streams = IPU7_IS_MMU_M0_STREAM_NUM,
		.nr_l2streams = IPU7_IS_MMU_M0_STREAM_NUM,
		.l1_block_sz = { 0x0, 0x3, 0x6, 0x8, 0xa, 0xc, 0xe, 0x10 },
		.l2_block_sz = { 0x0, 0x2, 0x4, 0x6, 0x8, 0xa, 0xc, 0xe },
		.zlx_nr = IPU7_IS_ZLX_M0_NUM,
		.zlx_axi_pool = { 0x00000f10 },
		.zlx_en = { 1, 1, 1, 1, 1, 1, 1, 1 },
		.zlx_conf = {
			0x00010103,
			0x00010103,
			0x00010101,
			0x00010101,
			0x00010101,
			0x00010101,
			0x00010101,
			0x00010101,
		},
		.uao_p_num = IPU7_IS_UAO_M0_WR_PLANENUM,
		.uao_p2tlb = {
			0x00000049,
			0x0000004a,
			0x0000004b,
			0x0000004c,
			0x0000004d,
			0x0000004e,
			0x0000004f,
			0x00000050,
		},
	},
	{
		.offset = IPU7_IS_MMU_M1_OFFSET,
		.zlx_offset = IPU7_IS_ZLX_M1_OFFSET,
		.uao_offset = IPU7_IS_UAO_M1_WR_OFFSET,
		.info_bits = 0x20006901,
		.refill = 0x00002322,
		.collapse_en_bitmap = 0x0,
		.l1_block = IPU7_IS_MMU_M1_L1_BLOCKNR_REG,
		.l2_block = IPU7_IS_MMU_M1_L2_BLOCKNR_REG,
		.nr_l1streams = IPU7_IS_MMU_M1_STREAM_NUM,
		.nr_l2streams = IPU7_IS_MMU_M1_STREAM_NUM,
		.l1_block_sz = {
			0x0, 0x3, 0x6, 0x9, 0xc,
			0xe, 0x10, 0x12, 0x14, 0x16,
			0x18, 0x1a, 0x1c, 0x1e, 0x20, 0x22,
		},
		.l2_block_sz = {
			0x0, 0x2, 0x4, 0x6, 0x8,
			0xa, 0xc, 0xe, 0x10, 0x12,
			0x14, 0x16, 0x18, 0x1a, 0x1c, 0x1e,
		},
		.zlx_nr = IPU7_IS_ZLX_M1_NUM,
		.zlx_axi_pool = { 0x00000f20 },
		.zlx_en = { 1, 1, 1, 1, 1, 1, 1, 1,
			    1, 1, 1, 1, 1, 1, 1, 1,
		},
		.zlx_conf = {
			0x00010103,
			0x00010103,
			0x00010103,
			0x00010103,
			0x00010103,
			0x00010103,
			0x00010103,
			0x00010103,
			0x00010101,
			0x00010101,
			0x00010101,
			0x00010101,
			0x00010101,
			0x00010101,
			0x00010101,
			0x00010101,
		},
		.uao_p_num = IPU7_IS_UAO_M1_WR_PLANENUM,
		.uao_p2tlb = {
			0x00000051,
			0x00000052,
			0x00000053,
			0x00000054,
			0x00000055,
			0x00000056,
			0x00000057,
			0x00000058,
			0x00000059,
			0x0000005a,
			0x0000005b,
			0x0000005c,
			0x0000005d,
			0x0000005e,
			0x0000005f,
			0x00000060,
		},
	},
};

static struct ipu7_mmu_hw ipu7_psys_mmu_hwdata[] = {
	{
		.name = "PS_FW_RD",
		.offset = IPU7_PS_MMU_FW_RD_OFFSET,
		.zlx_offset = IPU7_PS_ZLX_FW_RD_OFFSET,
		.uao_offset = IPU7_PS_UAO_FW_RD_OFFSET,
		.info_bits = 0x20004801,
		.refill = 0x00002726,
		.collapse_en_bitmap = 0x0,
		.l1_block = IPU7_PS_MMU_FW_RD_L1_BLOCKNR_REG,
		.l2_block = IPU7_PS_MMU_FW_RD_L2_BLOCKNR_REG,
		.nr_l1streams = IPU7_PS_MMU_FW_RD_STREAM_NUM,
		.nr_l2streams = IPU7_PS_MMU_FW_RD_STREAM_NUM,
		.l1_block_sz = {
			0, 0x8, 0xa, 0xc, 0xd,
			0xf, 0x11, 0x12, 0x13, 0x14,
			0x16, 0x18, 0x19, 0x1a, 0x1a,
			0x1a, 0x1a, 0x1a, 0x1a, 0x1a,
		},
		.l2_block_sz = {
			0x0, 0x2, 0x4, 0x6, 0x8,
			0xa, 0xc, 0xe, 0x10, 0x12,
			0x14, 0x16, 0x18, 0x1a, 0x1c,
			0x1e, 0x20, 0x22, 0x24, 0x26,
		},
		.zlx_nr = IPU7_PS_ZLX_FW_RD_NUM,
		.zlx_axi_pool = { 0x00000f30 },
		.zlx_en = {
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
		},
		.zlx_conf = { 0x0 },
		.uao_p_num = IPU7_PS_UAO_FW_RD_PLANENUM,
		.uao_p2tlb = {
			0x00000036,
			0x0000003d,
			0x0000003e,
			0x00000039,
			0x0000003f,
			0x00000040,
			0x00000041,
			0x0000003a,
			0x0000003b,
			0x00000042,
			0x00000043,
			0x00000044,
			0x0000003c,
		},
	},
	{
		.offset = IPU7_PS_MMU_FW_WR_OFFSET,
		.zlx_offset = IPU7_PS_ZLX_FW_WR_OFFSET,
		.uao_offset = IPU7_PS_UAO_FW_WR_OFFSET,
		.info_bits = 0x20004601,
		.refill = 0x00002322,
		.collapse_en_bitmap = 0x0,
		.l1_block = IPU7_PS_MMU_FW_WR_L1_BLOCKNR_REG,
		.l2_block = IPU7_PS_MMU_FW_WR_L2_BLOCKNR_REG,
		.nr_l1streams = IPU7_PS_MMU_FW_WR_STREAM_NUM,
		.nr_l2streams = IPU7_PS_MMU_FW_WR_STREAM_NUM,
		.l1_block_sz = {
			0, 0x8, 0xa, 0xc, 0xd,
			0xe, 0xf, 0x10, 0x10, 0x10,
		},
		.l2_block_sz = {
			0x0, 0x2, 0x4, 0x6, 0x8,
			0xa, 0xc, 0xe, 0x10, 0x12,
		},
		.zlx_nr = IPU7_PS_ZLX_FW_WR_NUM,
		.zlx_axi_pool = { 0x00000f20 },
		.zlx_en = { 0, 1, 1, 0, 0, 0, 0, 0, 0, 0 },
		.zlx_conf = { 0x0, 0x00010101, 0x00010101 },
		.uao_p_num = IPU7_PS_UAO_FW_WR_PLANENUM,
		.uao_p2tlb = { 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c },
	},
	{
		.offset = IPU7_PS_MMU_SRT_RD_OFFSET,
		.zlx_offset = IPU7_PS_ZLX_DATA_RD_OFFSET,
		.uao_offset = IPU7_PS_UAO_SRT_RD_OFFSET,
		.info_bits = 0x20004701,
		.refill = 0x00002120,
		.collapse_en_bitmap = 0x0,
		.l1_block = IPU7_PS_MMU_SRT_RD_L1_BLOCKNR_REG,
		.l2_block = IPU7_PS_MMU_SRT_RD_L2_BLOCKNR_REG,
		.nr_l1streams = IPU7_PS_MMU_SRT_RD_STREAM_NUM,
		.nr_l2streams = IPU7_PS_MMU_SRT_RD_STREAM_NUM,
		.l1_block_sz = {
			0x0, 0x4, 0x6, 0x8, 0xb,
			0xd, 0xf, 0x11, 0x13, 0x15,
			0x17, 0x23, 0x2b, 0x37, 0x3f,
			0x41, 0x43, 0x44, 0x45, 0x46,
			0x47, 0x48, 0x49, 0x4a, 0x4b,
			0x4c, 0x4d, 0x4e, 0x4f, 0x50,
			0x51, 0x52, 0x53, 0x55, 0x57,
			0x59, 0x5b, 0x5d, 0x5f, 0x61,
		},
		.l2_block_sz = {
			0x0, 0x2, 0x4, 0x6, 0x8,
			0xa, 0xc, 0xe, 0x10, 0x12,
			0x14, 0x16, 0x18, 0x1a, 0x1c,
			0x1e, 0x20, 0x22, 0x24, 0x26,
			0x28, 0x2a, 0x2c, 0x2e, 0x30,
			0x32, 0x34, 0x36, 0x38, 0x3a,
			0x3c, 0x3e, 0x40, 0x42, 0x44,
			0x46, 0x48, 0x4a, 0x4c, 0x4e,
		},
		.zlx_nr = IPU7_PS_ZLX_DATA_RD_NUM,
		.zlx_axi_pool = { 0x00000f30 },
		.zlx_en = {
			1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
		},
		.zlx_conf = {
			0x00030303,
			0x00010101,
			0x00010101,
			0x00030202,
			0x00010101,
			0x00010101,
			0x00010101,
			0x00030800,
			0x00030500,
			0x00020101,
			0x00042000,
			0x00031000,
			0x00042000,
			0x00031000,
			0x00020400,
			0x00010101,
		},
		.uao_p_num = IPU7_PS_UAO_SRT_RD_PLANENUM,
		.uao_p2tlb = {
			0x00000022,
			0x00000023,
			0x00000024,
			0x00000025,
			0x00000026,
			0x00000027,
			0x00000028,
			0x00000029,
			0x0000002a,
			0x0000002b,
			0x0000002c,
			0x0000002d,
			0x0000002e,
			0x0000002f,
			0x00000030,
			0x00000031,
			0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
			0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
			0x0000001e,
			0x0000001f,
			0x00000020,
			0x00000021,
			0x00000032,
			0x00000033,
			0x00000034,
			0x00000035,
		},
	},
	{
		.offset = IPU7_PS_MMU_SRT_WR_OFFSET,
		.zlx_offset = IPU7_PS_ZLX_DATA_WR_OFFSET,
		.uao_offset = IPU7_PS_UAO_SRT_WR_OFFSET,
		.info_bits = 0x20004501,
		.refill = 0x00002120,
		.collapse_en_bitmap = 0x0,
		.l1_block = IPU7_PS_MMU_SRT_WR_L1_BLOCKNR_REG,
		.l2_block = IPU7_PS_MMU_SRT_WR_L2_BLOCKNR_REG,
		.nr_l1streams = IPU7_PS_MMU_SRT_WR_STREAM_NUM,
		.nr_l2streams = IPU7_PS_MMU_SRT_WR_STREAM_NUM,
		.l1_block_sz = {
			0x0, 0x2, 0x6, 0xa, 0xc,
			0xe, 0x10, 0x12, 0x14, 0x16,
			0x18, 0x1a, 0x1c, 0x1e, 0x20,
			0x22, 0x24, 0x26, 0x32, 0x3a,
			0x3c, 0x3e, 0x4a, 0x52, 0x58,
			0x64, 0x6c, 0x72, 0x7e, 0x86,
			0x8c, 0x8d, 0x8e, 0x8f, 0x90,
			0x91, 0x92, 0x94, 0x96, 0x98,
		},
		.l2_block_sz = {
			0x0, 0x2, 0x4, 0x6, 0x8,
			0xa, 0xc, 0xe, 0x10, 0x12,
			0x14, 0x16, 0x18, 0x1a, 0x1c,
			0x1e, 0x20, 0x22, 0x24, 0x26,
			0x28, 0x2a, 0x2c, 0x2e, 0x30,
			0x32, 0x34, 0x36, 0x38, 0x3a,
			0x3c, 0x3e, 0x40, 0x42, 0x44,
			0x46, 0x48, 0x4a, 0x4c, 0x4e,
		},
		.zlx_nr = IPU7_PS_ZLX_DATA_WR_NUM,
		.zlx_axi_pool = { 0x00000f50 },
		.zlx_en = {
			1, 1, 1, 1, 1, 1, 1, 1,
			0, 0, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 1, 1,
			1, 1, 1, 1, 1, 1, 0, 0,
		},
		.zlx_conf = {
			0x00010102,
			0x00030103,
			0x00030103,
			0x00010101,
			0x00010101,
			0x00030101,
			0x00010101,
			0x38010101,
			0x0,
			0x0,
			0x38010101,
			0x38010101,
			0x38010101,
			0x38010101,
			0x38010101,
			0x38010101,
			0x00010101,
			0x00042000,
			0x00031000,
			0x00010101,
			0x00010101,
			0x00042000,
			0x00031000,
			0x00031000,
			0x00042000,
			0x00031000,
			0x00031000,
			0x00042000,
			0x00031000,
			0x00031000,
			0x0,
			0x0,
		},
		.uao_p_num = IPU7_PS_UAO_SRT_WR_PLANENUM,
		.uao_p2tlb = {
			0x00000000,
			0x00000001,
			0x00000002,
			0x00000003,
			0x00000004,
			0x00000005,
			0x00000006,
			0x00000007,
			0x00000008,
			0x00000009,
			0x0000000a,
			0x0000000b,
			0x0000000c,
			0x0000000d,
			0x0000000e,
			0x0000000f,
			0x00000010,
			0x00000011,
			0x00000012,
			0x00000013,
			0x00000014,
			0x00000015,
			0x00000016,
			0x00000017,
			0x00000018,
			0x00000019,
			0x0000001a,
			0x0000001b,
			0x0000001c,
			0x0000001d,
			0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
			0x0000001e,
			0x0000001f,
			0x00000020,
			0x00000021,
		},
	},
};

static const struct ipu7_mmu_hwdata ipu7_mmu_hwdata_lookup[IPU_SUBSYS_NUM] = {
	[IPU_PSYS] = {
		.hwdata = ipu7_psys_mmu_hwdata,
		.nr_mmus = ARRAY_SIZE(ipu7_psys_mmu_hwdata),
	},
	[IPU_ISYS] = {
		.hwdata = ipu7_isys_mmu_hwdata,
		.nr_mmus = ARRAY_SIZE(ipu7_isys_mmu_hwdata),
	},
};

static void __ipu7_tlb_invalidate(struct ipu6_mmu *mmu)
{
	struct ipu7_mmu_hw *mmu_hw = mmu->ipu7_mmu_hw;
	unsigned long flags;
	unsigned int i;
	int ret;
	u32 val;

	spin_lock_irqsave(&mmu->ready_lock, flags);
	if (!mmu->ready) {
		spin_unlock_irqrestore(&mmu->ready_lock, flags);
		return;
	}

	for (i = 0; i < mmu->nr_mmus; i++) {
		writel(0xffffffffU, mmu_hw[i].base +
		       IPU7_MMU_REG_INVALIDATE_0);

		/* Need check with HW, use l1streams or l2streams */
		if (mmu_hw[i].nr_l2streams > 32)
			writel(0xffffffffU, mmu_hw[i].base +
			       IPU7_MMU_REG_INVALIDATE_1);

		/*
		 * The TLB invalidation is a "single cycle" (IOMMU clock cycles)
		 * When the actual MMIO write reaches the IPU TLB Invalidate
		 * register, wmb() will force the TLB invalidate out if the CPU
		 * attempts to update the IOMMU page table (or sooner).
		 */
		wmb();

		/* wait invalidation done */
		ret = readl_poll_timeout_atomic(mmu_hw[i].base +
						IPU7_MMU_REG_INVALIDATION_STATUS,
						val, !(val & 0x1U), 500,
						IPU7_MMU_TLB_INVALIDATE_TIMEOUT);
		if (ret)
			dev_err(mmu->dev, "MMU[%u] TLB invalidate failed\n", i);
	}

	spin_unlock_irqrestore(&mmu->ready_lock, flags);
}

static int __ipu7_mmu_hw_init(struct ipu6_mmu *mmu)
{
	struct ipu6_mmu_info *mmu_info;
	struct ipu7_mmu_hw *mmu_hw = mmu->ipu7_mmu_hw;
	unsigned int i, j;

	mmu_info = mmu->dmap->mmu_info;
	for (i = 0; i < mmu->nr_mmus; i++) {
		/* Write page table address per MMU */
		writel((phys_addr_t)mmu_info->l1_pt_dma,
		       mmu_hw[i].base + IPU7_MMU_REG_PAGE_TABLE_BASE_ADDR);

		/* Set info bits and axi_refill per MMU */
		writel(mmu_hw[i].info_bits,
		       mmu_hw[i].base + IPU7_MMU_REG_USER_INFO_BITS);
		writel(mmu_hw[i].refill, mmu_hw[i].base + IPU7_MMU_REG_AXI_REFILL_IF_ID);
		writel(mmu_hw[i].collapse_en_bitmap,
		       mmu_hw[i].base + IPU7_MMU_REG_COLLAPSE_ENABLE_BITMAP);

		if (mmu_hw[i].at_sp_arb_cfg)
			writel(mmu_hw[i].at_sp_arb_cfg,
			       mmu_hw[i].base + IPU7_MMU_REG_AT_SP_ARB_CFG);

		/* Default irq configuration */
		writel(0x3ff, mmu_hw[i].base + IPU7_MMU_REG_IRQ_MASK);
		writel(0x3ff, mmu_hw[i].base + IPU7_MMU_REG_IRQ_ENABLE);

		/* Configure MMU TLB stream configuration for L1/L2 */
		for (j = 0; j < mmu_hw[i].nr_l1streams; j++) {
			writel(mmu_hw[i].l1_block_sz[j], mmu_hw[i].base +
			       mmu_hw[i].l1_block + 4U * j);
		}

		for (j = 0; j < mmu_hw[i].nr_l2streams; j++) {
			writel(mmu_hw[i].l2_block_sz[j], mmu_hw[i].base +
			       mmu_hw[i].l2_block + 4U * j);
		}

		for (j = 0; j < mmu_hw[i].uao_p_num; j++) {
			if (!mmu_hw[i].uao_p2tlb[j])
				continue;
			writel(mmu_hw[i].uao_p2tlb[j], mmu_hw[i].uao_base + 4U * j);
		}
	}

	for (i = 0; i < mmu->nr_mmus; i++) {
		for (j = 0; j < IPU7_ZLX_POOL_NUM; j++) {
			if (!mmu_hw[i].zlx_axi_pool[j])
				continue;
			writel(mmu_hw[i].zlx_axi_pool[j],
			       mmu_hw[i].zlx_base + IPU7_ZLX_REG_AXI_POOL + j * 0x4U);
		}

		for (j = 0; j < mmu_hw[i].zlx_nr; j++) {
			if (!mmu_hw[i].zlx_conf[j])
				continue;

			writel(mmu_hw[i].zlx_conf[j],
			       mmu_hw[i].zlx_base + IPU7_ZLX_REG_CONF + j * 0x8U);
		}

		for (j = 0; j < mmu_hw[i].zlx_nr; j++) {
			if (!mmu_hw[i].zlx_en[j])
				continue;

			writel(mmu_hw[i].zlx_en[j],
			       mmu_hw[i].zlx_base + IPU7_ZLX_REG_EN + j * 0x8U);
		}
	}

	return 0;
}

static int __ipu7_mmu_init_hw_data(struct ipu6_mmu *mmu, struct device *dev,
				   void __iomem *base)
{
	const struct ipu7_mmu_hwdata *lookup = ipu7_mmu_hwdata_lookup;
	struct ipu7_mmu_hw *mmu_hw, *src;
	unsigned int i, nr_mmus;

	if (mmu->mmid >= IPU_SUBSYS_NUM)
		return -EINVAL;

	src = lookup[mmu->mmid].hwdata;
	nr_mmus = lookup[mmu->mmid].nr_mmus;

	mmu_hw = devm_kcalloc(dev, nr_mmus, sizeof(*mmu_hw), GFP_KERNEL);
	if (!mmu_hw)
		return -ENOMEM;

	for (i = 0; i < nr_mmus; i++) {
		if (src[i].nr_l1streams > IPU7_MMU_MAX_TLB_L1_STREAMS ||
		    src[i].nr_l2streams > IPU7_MMU_MAX_TLB_L2_STREAMS)
			return -EINVAL;

		mmu_hw[i] = src[i];
		mmu_hw[i].base = base + src[i].offset;
		mmu_hw[i].zlx_base = base + src[i].zlx_offset;
		mmu_hw[i].uao_base = base + src[i].uao_offset;
	}

	mmu->nr_mmus = nr_mmus;
	mmu->ipu7_mmu_hw = mmu_hw;

	return 0;
}

const struct ipu6_mmu_hw_ops ipu7_mmu_ops = {
	.init_hw_data = __ipu7_mmu_init_hw_data,
	.hw_init = __ipu7_mmu_hw_init,
	.tlb_invalidate = __ipu7_tlb_invalidate,
};
