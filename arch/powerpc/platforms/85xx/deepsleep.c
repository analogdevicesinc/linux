/*
 * Support deep sleep feature for T104x
 *
 * Copyright 2018 NXP
 * Author: Chenhui Zhao <chenhui.zhao@freescale.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of the above-listed copyright holders nor the
 *	 names of any contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <sysdev/fsl_soc.h>
#include <asm/machdep.h>
#include <asm/fsl_pm.h>

#define SIZE_1MB	0x100000
#define SIZE_2MB	0x200000

#define CPC_CPCHDBCR0		0x10f00
#define CPC_CPCHDBCR0_SPEC_DIS	0x08000000

#define CCSR_SCFG_DPSLPCR	0xfc000
#define CCSR_SCFG_DPSLPCR_WDRR_EN	0x1
#define CCSR_SCFG_SPARECR2	0xfc504
#define CCSR_SCFG_SPARECR3	0xfc508

#define CCSR_GPIO1_GPDIR	0x130000
#define CCSR_GPIO1_GPODR	0x130004
#define CCSR_GPIO1_GPDAT	0x130008
#define CCSR_GPIO1_GPDIR_29	0x4

#define RCPM_BLOCK_OFFSET	0x00022000
#define EPU_BLOCK_OFFSET	0x00000000
#define NPC_BLOCK_OFFSET	0x00001000

#define CSTTACR0		0xb00
#define CG1CR0			0x31c

#define CCSR_LAW_BASE		0xC00
#define DCFG_BRR	0xE4	/* boot release register */
#define LCC_BSTRH	0x20	/* Boot space translation register high */
#define LCC_BSTRL	0x24	/* Boot space translation register low */
#define LCC_BSTAR	0x28	/* Boot space translation attribute register */
#define RCPM_PCTBENR	0x1A0	/* Physical Core Timebase Enable Register */
#define RCPM_BASE	0xE2000
#define DCFG_BASE	0xE0000

/* 128 bytes buffer for restoring data broke by DDR training initialization */
#define DDR_BUF_SIZE	128
static u8 ddr_buff[DDR_BUF_SIZE] __aligned(64);

static void *dcsr_base, *ccsr_base, *pld_base;
static int pld_flag;

/* for law */
struct fsl_law {
	u32	lawbarh;	/* LAWn base address high */
	u32	lawbarl;	/* LAWn base address low */
	u32	lawar;		/* LAWn attributes */
	u32	reserved;
};

struct fsl_law *saved_law;
static u32 num_laws;

/* for nonboot cpu */
struct fsl_bstr {
	u32	bstrh;
	u32	bstrl;
	u32	bstar;
	u32 cpu_mask;
};
static struct fsl_bstr saved_bstr;

int fsl_dp_iomap(void)
{
	struct device_node *np;
	int ret = 0;
	phys_addr_t ccsr_phy_addr, dcsr_phy_addr;

	saved_law = NULL;
	ccsr_base = NULL;
	dcsr_base = NULL;
	pld_base = NULL;

	ccsr_phy_addr = get_immrbase();
	if (ccsr_phy_addr == -1) {
		pr_err("%s: Can't get the address of CCSR\n", __func__);
		ret = -EINVAL;
		goto ccsr_err;
	}
	ccsr_base = ioremap(ccsr_phy_addr, SIZE_2MB);
	if (!ccsr_base) {
		ret = -ENOMEM;
		goto ccsr_err;
	}

	dcsr_phy_addr = get_dcsrbase();
	if (dcsr_phy_addr == -1) {
		pr_err("%s: Can't get the address of DCSR\n", __func__);
		ret = -EINVAL;
		goto dcsr_err;
	}
	dcsr_base = ioremap(dcsr_phy_addr, SIZE_1MB);
	if (!dcsr_base) {
		ret = -ENOMEM;
		goto dcsr_err;
	}

	np = of_find_compatible_node(NULL, NULL, "fsl,tetra-fpga");
	if (np) {
		pld_flag = T1040QDS_TETRA_FLAG;
	} else {
		np = of_find_compatible_node(NULL, NULL, "fsl,deepsleep-cpld");
		if (np) {
			pld_flag = T104xRDB_CPLD_FLAG;
		} else {
			pr_err("%s: Can't find the FPGA/CPLD node\n",
					__func__);
			ret = -EINVAL;
			goto pld_err;
		}
	}
	pld_base = of_iomap(np, 0);
	of_node_put(np);

	np = of_find_compatible_node(NULL, NULL, "fsl,corenet-law");
	if (!np) {
		pr_err("%s: Can't find the node of \"law\"\n", __func__);
		ret = -EINVAL;
		goto alloc_err;
	}
	ret = of_property_read_u32(np, "fsl,num-laws", &num_laws);
	if (ret) {
		ret = -EINVAL;
		goto alloc_err;
	}

	saved_law = kzalloc(sizeof(*saved_law) * num_laws, GFP_KERNEL);
	if (!saved_law) {
		ret = -ENOMEM;
		goto alloc_err;
	}
	of_node_put(np);

	return 0;

alloc_err:
	iounmap(pld_base);
	pld_base = NULL;
pld_err:
	iounmap(dcsr_base);
	dcsr_base = NULL;
dcsr_err:
	iounmap(ccsr_base);
	ccsr_base = NULL;
ccsr_err:
	return ret;
}

void fsl_dp_iounmap(void)
{
	if (dcsr_base) {
		iounmap(dcsr_base);
		dcsr_base = NULL;
	}

	if (ccsr_base) {
		iounmap(ccsr_base);
		ccsr_base = NULL;
	}

	if (pld_base) {
		iounmap(pld_base);
		pld_base = NULL;
	}

	kfree(saved_law);
	saved_law = NULL;
}

static void fsl_dp_ddr_save(void *ccsr_base)
{
	u32 ddr_buff_addr;

	/*
	 * DDR training initialization will break 128 bytes at the beginning
	 * of DDR, therefore, save them so that the bootloader will restore
	 * them. Assume that DDR is mapped to the address space started with
	 * CONFIG_PAGE_OFFSET.
	 */
	memcpy(ddr_buff, (void *)CONFIG_PAGE_OFFSET, DDR_BUF_SIZE);

	/* assume ddr_buff is in the physical address space of 4GB */
	ddr_buff_addr = (u32)(__pa(ddr_buff) & 0xffffffff);

	/*
	 * the bootloader will restore the first 128 bytes of DDR from
	 * the location indicated by the register SPARECR3
	 */
	out_be32(ccsr_base + CCSR_SCFG_SPARECR3, ddr_buff_addr);
}

static void fsl_dp_mp_save(void *ccsr)
{
	struct fsl_bstr *dst = &saved_bstr;

	dst->bstrh = in_be32(ccsr + LCC_BSTRH);
	dst->bstrl = in_be32(ccsr + LCC_BSTRL);
	dst->bstar = in_be32(ccsr + LCC_BSTAR);
	dst->cpu_mask = in_be32(ccsr + DCFG_BASE + DCFG_BRR);
}

static void fsl_dp_mp_restore(void *ccsr)
{
	struct fsl_bstr *src = &saved_bstr;

	out_be32(ccsr + LCC_BSTRH, src->bstrh);
	out_be32(ccsr + LCC_BSTRL, src->bstrl);
	out_be32(ccsr + LCC_BSTAR, src->bstar);

	/* release the nonboot cpus */
	out_be32(ccsr + DCFG_BASE + DCFG_BRR, src->cpu_mask);

	/* enable the time base */
	out_be32(ccsr + RCPM_BASE + RCPM_PCTBENR, src->cpu_mask);
	/* read back to sync write */
	in_be32(ccsr + RCPM_BASE + RCPM_PCTBENR);
}

static void fsl_dp_law_save(void *ccsr)
{
	int i;
	struct fsl_law *dst = saved_law;
	struct fsl_law *src = (void *)(ccsr + CCSR_LAW_BASE);

	for (i = 0; i < num_laws; i++) {
		dst->lawbarh = in_be32(&src->lawbarh);
		dst->lawbarl = in_be32(&src->lawbarl);
		dst->lawar = in_be32(&src->lawar);
		dst++;
		src++;
	}
}

static void fsl_dp_law_restore(void *ccsr)
{
	int i;
	struct fsl_law *src = saved_law;
	struct fsl_law *dst = (void *)(ccsr + CCSR_LAW_BASE);

	for (i = 0; i < num_laws - 1; i++) {
		out_be32(&dst->lawar, 0);
		out_be32(&dst->lawbarl, src->lawbarl);
		out_be32(&dst->lawbarh, src->lawbarh);
		out_be32(&dst->lawar, src->lawar);

		 /* Read back so that we sync the writes */
		in_be32(&dst->lawar);
		src++;
		dst++;
	}
}

static void fsl_dp_set_resume_pointer(void *ccsr_base)
{
	u32 resume_addr;

	/* the bootloader will finally jump to this address to return kernel */
#ifdef CONFIG_PPC32
	resume_addr = (u32)(__pa(fsl_booke_deep_sleep_resume));
#else
	resume_addr = (u32)(__pa(*(u64 *)fsl_booke_deep_sleep_resume)
			    & 0xffffffff);
#endif

	/* use the register SPARECR2 to save the resume address */
	out_be32(ccsr_base + CCSR_SCFG_SPARECR2, resume_addr);

}

int fsl_enter_epu_deepsleep(void)
{
	fsl_dp_ddr_save(ccsr_base);

	fsl_dp_set_resume_pointer(ccsr_base);

	fsl_dp_mp_save(ccsr_base);
	fsl_dp_law_save(ccsr_base);
	/*  enable Warm Device Reset request. */
	setbits32(ccsr_base + CCSR_SCFG_DPSLPCR, CCSR_SCFG_DPSLPCR_WDRR_EN);

	/* set GPIO1_29 as an output pin (not open-drain), and output 0 */
	clrbits32(ccsr_base + CCSR_GPIO1_GPDAT, CCSR_GPIO1_GPDIR_29);
	clrbits32(ccsr_base + CCSR_GPIO1_GPODR, CCSR_GPIO1_GPDIR_29);
	setbits32(ccsr_base + CCSR_GPIO1_GPDIR, CCSR_GPIO1_GPDIR_29);

	/*
	 * Disable CPC speculation to avoid deep sleep hang, especially
	 * in secure boot mode. This bit will be cleared automatically
	 * when resuming from deep sleep.
	 */
	setbits32(ccsr_base + CPC_CPCHDBCR0, CPC_CPCHDBCR0_SPEC_DIS);

	fsl_epu_setup_default(dcsr_base + EPU_BLOCK_OFFSET);
	fsl_npc_setup_default(dcsr_base + NPC_BLOCK_OFFSET);
	out_be32(dcsr_base + RCPM_BLOCK_OFFSET + CSTTACR0, 0x00001001);
	out_be32(dcsr_base + RCPM_BLOCK_OFFSET + CG1CR0, 0x00000001);

	fsl_dp_enter_low(ccsr_base, dcsr_base, pld_base, pld_flag);

	fsl_dp_law_restore(ccsr_base);
	fsl_dp_mp_restore(ccsr_base);

	/* disable Warm Device Reset request */
	clrbits32(ccsr_base + CCSR_SCFG_DPSLPCR, CCSR_SCFG_DPSLPCR_WDRR_EN);

	fsl_epu_clean_default(dcsr_base + EPU_BLOCK_OFFSET);

	return 0;
}
