/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/delay.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/genalloc.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/psci.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <asm/cacheflush.h>
#include <asm/fncpy.h>
#include <asm/mach/map.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include <asm/tlb.h>

#include <uapi/linux/psci.h>

#include "common.h"
#include "hardware.h"

#define MU_SR		0x60

#define PMPROT	0x8
#define PMCTRL	0x10
#define PMSTAT	0x18
#define SRS	0x20
#define RPC	0x24
#define SSRS	0x28
#define SRIE	0x2c
#define SRIF	0x30
#define CSRE	0x34
#define MR	0x40

#define PMC_HSRUN		0x4
#define PMC_RUN			0x8
#define PMC_VLPR		0xc
#define PMC_STOP		0x10
#define PMC_VLPS		0x14
#define PMC_LLS			0x18
#define PMC_VLLS		0x1c
#define PMC_STATUS		0x20
#define PMC_CTRL		0x24
#define PMC_SRAMCTRL_0		0x28
#define PMC_SRAMCTRL_1		0x2c
#define PMC_SRAMCTRL_2		0x30

#define BM_PMPROT_AHSRUN	(1 << 7)
#define BM_PMPROT_AVLP		(1 << 5)
#define BM_PMPROT_ALLS		(1 << 3)
#define BM_PMPROT_AVLLS		(1 << 1)

#define BM_PMCTRL_STOPA		(1 << 24)
#define BM_PMCTRL_PSTOPO	(3 << 16)
#define BM_PMCTRL_RUNM		(3 << 8)
#define BM_PMCTRL_STOPM		(7 << 0)

#define BM_VLPS_RBBEN		(1 << 28)

#define BM_CTRL_LDOEN		(1 << 31)
#define BM_CTRL_LDOOKDIS	(1 << 30)

#define BM_VLLS_MON1P2HVDHP	(1 << 5)
#define BM_VLLS_MON1P2LVDHP	(1 << 4)

#define BP_PMCTRL_STOPM		0
#define BP_PMCTRL_PSTOPO	16

#define MX7ULP_MAX_MMDC_IO_NUM		36
#define MX7ULP_MAX_MMDC_NUM		50
#define MX7ULP_MAX_IOMUX_NUM		116
#define MX7ULP_MAX_SELECT_INPUT_NUM	78

#define IOMUX_START		0x0
#define SELECT_INPUT_START	0x200

#define TPM_SC		0x10
#define TPM_MOD		0x18
#define TPM_C0SC	0x20
#define TPM_C0V		0x24

#define PCC2_ENABLE_PCS_FIRC	((1 << 30) | (3 << 24))
#define PCC2_ENABLE		(1 << 30)

#define LPUART_BAUD	0x10
#define LPUART_CTRL	0x18
#define LPUART_FIFO	0x28
#define LPUART_WATER	0x2c

#define GPIO_PDOR	0x0
#define GPIO_PDDR	0x14

#define PTC2_LPUART4_TX_OFFSET	0x8
#define PTC3_LPUART4_RX_OFFSET	0xc
#define PTC2_LPUART4_TX_INPUT_OFFSET	0x248
#define PTC3_LPUART4_RX_INPUT_OFFSET	0x24c
#define LPUART4_MUX_VALUE	(4 << 8)
#define LPUART4_INPUT_VALUE	(1)

#define MU_B_SR_NMIC	(1 << 3)

#define DGO_GPR3	0x60
#define DGO_GPR4	0x64

#define ADDR_1M_MASK	0xFFF00000

static void __iomem *smc1_base;
static void __iomem *pmc0_base;
static void __iomem *pmc1_base;
static void __iomem *tpm5_base;
static void __iomem *lpuart4_base;
static void __iomem *iomuxc1_base;
static void __iomem *pcc2_base;
static void __iomem *pcc3_base;
static void __iomem *mu_base;
static void __iomem *scg1_base;
static void __iomem *gpio_base[4];
static void __iomem *suspend_ocram_base;
static void (*imx7ulp_suspend_in_ocram_fn)(void __iomem *sram_base);

static u32 tpm5_regs[4];
static u32 lpuart4_regs[4];
static u32 pcc2_regs[24][2] = {
	{0x20, 0}, {0x3c, 0}, {0x40, 0}, {0x6c, 0},
	{0x84, 0}, {0x90, 0}, {0x94, 0}, {0x98, 0},
	{0x9c, 0}, {0xa4, 0}, {0xa8, 0}, {0xac, 0},
	{0xb0, 0}, {0xb4, 0}, {0xb8, 0}, {0xc4, 0},
	{0xcc, 0}, {0xd0, 0}, {0xd4, 0}, {0xd8, 0},
	{0xdc, 0}, {0xe0, 0}, {0xf4, 0}, {0x10c, 0},
};

static u32 pcc3_regs[16][2] = {
	{0x84, 0}, {0x88, 0}, {0x90, 0}, {0x94, 0},
	{0x98, 0}, {0x9c, 0}, {0xa0, 0}, {0xa4, 0},
	{0xa8, 0}, {0xac, 0}, {0xb8, 0}, {0xbc, 0},
	{0xc0, 0}, {0xc4, 0}, {0x140, 0}, {0x144, 0},
};

static u32 scg1_offset[17] = {
	0x14, 0x30, 0x40, 0x304,
	0x500, 0x504, 0x508, 0x50c,
	0x510, 0x514, 0x600, 0x604,
	0x608, 0x60c, 0x610, 0x614,
	0x104,
};

extern unsigned long iram_tlb_base_addr;
extern unsigned long iram_tlb_phys_addr;

/*
 * suspend ocram space layout:
 * ======================== high address ======================
 *                              .
 *                              .
 *                              .
 *                              ^
 *                              ^
 *                              ^
 *                      imx7ulp_suspend code
 *              PM_INFO structure(imx7ulp_cpu_pm_info)
 * ======================== low address =======================
 */
struct imx7ulp_pm_socdata {
	u32 ddr_type;
	const char *mmdc_compat;
	const u32 mmdc_io_num;
	const u32 *mmdc_io_offset;
	const u32 mmdc_num;
	const u32 *mmdc_offset;
};

static const u32 imx7ulp_mmdc_io_lpddr3_offset[] __initconst = {
	0x128, 0xf8, 0xd8, 0x108,
	0x104, 0x124, 0x80, 0x84,
	0x88, 0x8c, 0x120, 0x10c,
	0x110, 0x114, 0x118, 0x90,
	0x94, 0x98, 0x9c, 0xe0,
	0xe4,
};

static const u32 imx7ulp_mmdc_lpddr3_offset[] __initconst = {
	0x01c, 0x800, 0x85c, 0x890,
	0x848, 0x850, 0x81c, 0x820,
	0x824, 0x828, 0x82c, 0x830,
	0x834, 0x838, 0x8c0, 0x8b8,
	0x004, 0x00c, 0x010, 0x038,
	0x014, 0x018, 0x02c, 0x030,
	0x040, 0x000, 0x01c, 0x01c,
	0x01c, 0x01c, 0x01c, 0x01c,
	0x01c, 0x01c, 0x01c, 0x01c,
	0x01c, 0x01c, 0x83c, 0x020,
	0x800, 0x004, 0x404, 0x01c,
};

static const u32 imx7ulp_lpddr3_script[] __initconst = {
	0x00008000, 0xA1390003, 0x0D3900A0, 0x00400000,
	0x40404040, 0x40404040, 0x33333333, 0x33333333,
	0x33333333, 0x33333333, 0xf3333333, 0xf3333333,
	0xf3333333, 0xf3333333, 0x24922492, 0x00000800,
	0x00020052, 0x292C42F3, 0x00100A22, 0x00120556,
	0x00C700DB, 0x00211718, 0x0F9F26D2, 0x009F0E10,
	0x0000003F, 0xC3190000, 0x00008050, 0x00008058,
	0x003F8030, 0x003F8038, 0xFF0A8030, 0xFF0A8038,
	0x04028030, 0x04028038, 0x83018030, 0x83018038,
	0x01038030, 0x01038038, 0x20000000, 0x00001800,
	0xA1310000, 0x00020052, 0x00011006, 0x00000000,
};

static const struct imx7ulp_pm_socdata imx7ulp_lpddr3_pm_data __initconst = {
	.mmdc_compat = "fsl,imx7ulp-mmdc",
	.mmdc_io_num = ARRAY_SIZE(imx7ulp_mmdc_io_lpddr3_offset),
	.mmdc_io_offset = imx7ulp_mmdc_io_lpddr3_offset,
	.mmdc_num = ARRAY_SIZE(imx7ulp_mmdc_lpddr3_offset),
	.mmdc_offset = imx7ulp_mmdc_lpddr3_offset,
};

/*
 * This structure is for passing necessary data for low level ocram
 * suspend code(arch/arm/mach-imx/suspend-imx7ulp.S), if this struct
 * definition is changed, the offset definition in
 * arch/arm/mach-imx/suspend-imx7ulp.S must be also changed accordingly,
 * otherwise, the suspend to sram function will be broken!
 */
struct imx7ulp_cpu_pm_info {
	u32 m4_reserve0;
	u32 m4_reserve1;
	u32 m4_reserve2;
	phys_addr_t pbase; /* The physical address of pm_info. */
	phys_addr_t resume_addr; /* The physical resume address for asm code */
	u32 pm_info_size; /* Size of pm_info. */
	void __iomem *sim_base;
	void __iomem *scg1_base;
	void __iomem *mmdc_base;
	void __iomem *mmdc_io_base;
	void __iomem *smc1_base;
	u32 scg1[17];
	u32 ttbr1; /* Store TTBR1 */
	u32 gpio[4][2];
	u32 iomux_num; /* Number of IOs which need saved/restored. */
	u32 iomux_val[MX7ULP_MAX_IOMUX_NUM]; /* To save value */
	u32 select_input_num; /* Number of select input which need saved/restored. */
	u32 select_input_val[MX7ULP_MAX_SELECT_INPUT_NUM]; /* To save value */
	u32 mmdc_io_num; /* Number of MMDC IOs which need saved/restored. */
	u32 mmdc_io_val[MX7ULP_MAX_MMDC_IO_NUM][2]; /* To save offset and value */
	u32 mmdc_num; /* Number of MMDC registers which need saved/restored. */
	u32 mmdc_val[MX7ULP_MAX_MMDC_NUM][2];
} __aligned(8);

static struct imx7ulp_cpu_pm_info *pm_info;
static void __iomem *aips1_base;
static void __iomem *aips2_base;
static void __iomem *aips3_base;
static void __iomem *aips4_base;
static void __iomem *aips5_base;

static const char * const low_power_ocram_match[] __initconst = {
	"fsl,lpm-sram",
	NULL
};

static void imx7ulp_gpio_save(void)
{
	int i;

	for (i = 0; i < 4; i++) {
		pm_info->gpio[i][0] = readl_relaxed(gpio_base[i] + GPIO_PDOR);
		pm_info->gpio[i][1] = readl_relaxed(gpio_base[i] + GPIO_PDDR);
	}
}

static void imx7ulp_scg1_save(void)
{
	int i;

	for (i = 0; i < 17; i++)
		pm_info->scg1[i] = readl_relaxed(scg1_base + scg1_offset[i]);
}

static void imx7ulp_pcc3_save(void)
{
	int i;

	for (i = 0; i < 16; i++)
		pcc3_regs[i][1] = readl_relaxed(pcc3_base + pcc3_regs[i][0]);
}

static void imx7ulp_pcc3_restore(void)
{
	int i;

	for (i = 0; i < 16; i++)
		writel_relaxed(pcc3_regs[i][1], pcc3_base + pcc3_regs[i][0]);
}

static void imx7ulp_pcc2_save(void)
{
	int i;

	for (i = 0; i < 24; i++)
		pcc2_regs[i][1] = readl_relaxed(pcc2_base + pcc2_regs[i][0]);
}

static void imx7ulp_pcc2_restore(void)
{
	int i;

	for (i = 0; i < 24; i++)
		writel_relaxed(pcc2_regs[i][1], pcc2_base + pcc2_regs[i][0]);
}

static inline void imx7ulp_iomuxc_save(void)
{
	int i;

	pm_info->iomux_num = MX7ULP_MAX_IOMUX_NUM;
	pm_info->select_input_num = MX7ULP_MAX_SELECT_INPUT_NUM;

	for (i = 0; i < pm_info->iomux_num; i++)
		pm_info->iomux_val[i] =
			readl_relaxed(iomuxc1_base +
				IOMUX_START + i * 0x4);
	for (i = 0; i < pm_info->select_input_num; i++)
		pm_info->select_input_val[i] =
			readl_relaxed(iomuxc1_base +
				SELECT_INPUT_START + i * 0x4);
}

static void imx7ulp_lpuart_save(void)
{
	lpuart4_regs[0] = readl_relaxed(lpuart4_base + LPUART_BAUD);
	lpuart4_regs[1] = readl_relaxed(lpuart4_base + LPUART_FIFO);
	lpuart4_regs[2] = readl_relaxed(lpuart4_base + LPUART_WATER);
	lpuart4_regs[3] = readl_relaxed(lpuart4_base + LPUART_CTRL);
}

static void imx7ulp_lpuart_restore(void)
{
	writel_relaxed(LPUART4_MUX_VALUE,
		iomuxc1_base + PTC2_LPUART4_TX_OFFSET);
	writel_relaxed(LPUART4_MUX_VALUE,
		iomuxc1_base + PTC3_LPUART4_RX_OFFSET);
	writel_relaxed(LPUART4_INPUT_VALUE,
		iomuxc1_base + PTC2_LPUART4_TX_INPUT_OFFSET);
	writel_relaxed(LPUART4_INPUT_VALUE,
		iomuxc1_base + PTC3_LPUART4_RX_INPUT_OFFSET);

	writel_relaxed(lpuart4_regs[0], lpuart4_base + LPUART_BAUD);
	writel_relaxed(lpuart4_regs[1], lpuart4_base + LPUART_FIFO);
	writel_relaxed(lpuart4_regs[2], lpuart4_base + LPUART_WATER);
	writel_relaxed(lpuart4_regs[3], lpuart4_base + LPUART_CTRL);
}

static void imx7ulp_tpm_save(void)
{
	tpm5_regs[0] = readl_relaxed(tpm5_base + TPM_SC);
	tpm5_regs[1] = readl_relaxed(tpm5_base + TPM_MOD);
	tpm5_regs[2] = readl_relaxed(tpm5_base + TPM_C0SC);
	tpm5_regs[3] = readl_relaxed(tpm5_base + TPM_C0V);
}

static void imx7ulp_tpm_restore(void)
{
	writel_relaxed(tpm5_regs[0], tpm5_base + TPM_SC);
	writel_relaxed(tpm5_regs[1], tpm5_base + TPM_MOD);
	writel_relaxed(tpm5_regs[2], tpm5_base + TPM_C0SC);
	writel_relaxed(tpm5_regs[3], tpm5_base + TPM_C0V);
}

static void imx7ulp_set_dgo(u32 val)
{
	writel_relaxed(val, pm_info->sim_base + DGO_GPR3);
	writel_relaxed(val, pm_info->sim_base + DGO_GPR4);
}

int imx7ulp_set_lpm(enum imx7ulp_cpu_pwr_mode mode)
{
	u32 val1 = BM_PMPROT_AHSRUN | BM_PMPROT_AVLP | BM_PMPROT_AVLLS;
	u32 val2 = readl_relaxed(smc1_base + PMCTRL);
	u32 val3 = readl_relaxed(pmc0_base + PMC_CTRL);

	val2 &= ~(BM_PMCTRL_RUNM |
		BM_PMCTRL_STOPM | BM_PMCTRL_PSTOPO);
	val3 |= BM_CTRL_LDOOKDIS;

	switch (mode) {
	case RUN:
		/* system/bus clock enabled */
		val2 |= 0x3 << BP_PMCTRL_PSTOPO;
		break;
	case WAIT:
		/* system clock disabled, bus clock enabled */
		val2 |= 0x2 << BP_PMCTRL_PSTOPO;
		break;
	case STOP:
		/* system/bus clock disabled */
		val2 |= 0x1 << BP_PMCTRL_PSTOPO;
		break;
	case VLPS:
		val2 |= 0x2 << BP_PMCTRL_STOPM;
		break;
	case VLLS:
		val2 |= 0x4 << BP_PMCTRL_STOPM;
		break;
	default:
		return -EINVAL;
	}

	writel_relaxed(val1, smc1_base + PMPROT);
	writel_relaxed(val2, smc1_base + PMCTRL);
	writel_relaxed(val3, pmc0_base + PMC_CTRL);

	return 0;
}

#define MX7ULP_SUSPEND_POWERDWN_PARAM	\
	((0 << PSCI_0_2_POWER_STATE_ID_SHIFT) | \
	 (1 << PSCI_0_2_POWER_STATE_AFFL_SHIFT) | \
	 (PSCI_POWER_STATE_TYPE_POWER_DOWN << PSCI_0_2_POWER_STATE_TYPE_SHIFT))

#define MX7ULP_SUSPEND_STANDBY_PARAM	\
	((0 << PSCI_0_2_POWER_STATE_ID_SHIFT) | \
	 (1 << PSCI_0_2_POWER_STATE_AFFL_SHIFT) | \
	 (PSCI_POWER_STATE_TYPE_STANDBY << PSCI_0_2_POWER_STATE_TYPE_SHIFT))

static int imx7ulp_suspend_finish(unsigned long val)
{
	u32 state;

	if (val == 0)
		state = MX7ULP_SUSPEND_POWERDWN_PARAM;
	else
		state = MX7ULP_SUSPEND_STANDBY_PARAM;

	if (psci_ops.cpu_suspend) {
		return psci_ops.cpu_suspend(state, __pa(cpu_resume));
	}

	imx7ulp_suspend_in_ocram_fn(suspend_ocram_base);

	return 0;
}

static int imx7ulp_pm_enter(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_STANDBY:
		if (psci_ops.cpu_suspend) {
			/* Zzz ... */
			cpu_suspend(1, imx7ulp_suspend_finish);
		} else {
			imx7ulp_set_lpm(VLPS);
			writel_relaxed(
				readl_relaxed(pmc1_base + PMC_VLPS) | BM_VLPS_RBBEN,
				pmc1_base + PMC_VLPS);

			/* Zzz ... */
			cpu_suspend(0, imx7ulp_suspend_finish);

			writel_relaxed(
				readl_relaxed(pmc1_base + PMC_VLPS) & ~BM_VLPS_RBBEN,
				pmc1_base + PMC_VLPS);
			imx7ulp_set_lpm(RUN);
		}
		break;
	case PM_SUSPEND_MEM:
		if (psci_ops.cpu_suspend) {
			/* Zzz ... */
			cpu_suspend(0, imx7ulp_suspend_finish);
		} else {
			imx7ulp_gpio_save();
			imx7ulp_scg1_save();
			imx7ulp_pcc2_save();
			imx7ulp_pcc3_save();
			imx7ulp_tpm_save();
			if (!console_suspend_enabled)
				imx7ulp_lpuart_save();
			imx7ulp_iomuxc_save();
			imx7ulp_set_lpm(VLLS);

			/* Zzz ... */
			cpu_suspend(0, imx7ulp_suspend_finish);

			imx7ulp_pcc2_restore();
			imx7ulp_pcc3_restore();
			if (!console_suspend_enabled)
				imx7ulp_lpuart_restore();
			imx7ulp_set_dgo(0);
			imx7ulp_tpm_restore();
			imx7ulp_set_lpm(RUN);
	}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int imx7ulp_pm_valid(suspend_state_t state)
{
	return (state == PM_SUSPEND_STANDBY || state == PM_SUSPEND_MEM);
}

static const struct platform_suspend_ops imx7ulp_pm_ops = {
	.enter = imx7ulp_pm_enter,
	.valid = imx7ulp_pm_valid,
};

static int __init imx7ulp_suspend_init(void)
{
	int ret = 0;

	suspend_set_ops(&imx7ulp_pm_ops);

	return ret;
}

static struct map_desc iram_tlb_io_desc __initdata = {
	/* .virtual and .pfn are run-time assigned */
	.length     = SZ_1M,
	.type       = MT_MEMORY_RWX_NONCACHED,
};

static int __init imx7ulp_dt_find_lpsram(unsigned long node, const char *uname,
				      int depth, void *data)
{
	unsigned long lpram_addr;
	const __be32 *prop = of_get_flat_dt_prop(node, "reg", NULL);

	if (of_flat_dt_match(node, low_power_ocram_match)) {
		if (!prop)
			return -EINVAL;

		lpram_addr = be32_to_cpup(prop);

		/* We need to create a 1M page table entry. */
		iram_tlb_io_desc.virtual =
			IMX_IO_P2V(lpram_addr & ADDR_1M_MASK);
		iram_tlb_io_desc.pfn = __phys_to_pfn(lpram_addr & ADDR_1M_MASK);
		iram_tlb_phys_addr = lpram_addr;
		iram_tlb_base_addr = IMX_IO_P2V(lpram_addr);
		iotable_init(&iram_tlb_io_desc, 1);
	}

	return 0;
}

void __init imx7ulp_pm_map_io(void)
{
	if (psci_ops.cpu_suspend) {
		return;
	}
	/*
	 * Get the address of IRAM or OCRAM to be used by the low
	 * power code from the device tree.
	 */
	WARN_ON(of_scan_flat_dt(imx7ulp_dt_find_lpsram, NULL));

	/* Return if no IRAM space is allocated for suspend/resume code. */
	if (!iram_tlb_base_addr) {
		pr_warn("No valid ocram available for suspend/resume!\n");
		return;
	}

	/* Set all entries to 0 except first 3 words reserved for M4. */
	memset((void *)iram_tlb_base_addr, 0, MX7ULP_IRAM_TLB_SIZE);
}

void __init imx7ulp_pm_common_init(const struct imx7ulp_pm_socdata
				*socdata)
{
	struct device_node *np;
	unsigned long sram_paddr = 0;
	const u32 *mmdc_offset_array;
	const u32 *mmdc_io_offset_array;
	unsigned long i, j;
	int ret;

	if (psci_ops.cpu_suspend) {
		aips1_base = ioremap(MX7ULP_AIPS1_BASE_ADDR, SZ_1M);
		aips2_base = ioremap(MX7ULP_AIPS2_BASE_ADDR, SZ_1M);
		aips3_base = ioremap(MX7ULP_AIPS3_BASE_ADDR, SZ_1M);
		aips4_base = ioremap(MX7ULP_AIPS4_BASE_ADDR, SZ_1M);
		aips5_base = ioremap(MX7ULP_AIPS5_BASE_ADDR, SZ_1M);
	} else {
		/*
		 * Make sure the IRAM virtual address has a mapping in the IRAM
		 * page table.
		 *
		 * Only use the top 12 bits [31-20] when storing the physical
		 * address in the page table as only these bits are required
		 * for 1M mapping.
		 */
		j = ((iram_tlb_base_addr >> 20) << 2) / 4;
		*((unsigned long *)iram_tlb_base_addr + j) =
			(iram_tlb_phys_addr & ADDR_1M_MASK) |
			TT_ATTRIB_NON_CACHEABLE_1M;
		/*
		 * Make sure the AIPS1 virtual address has a mapping in the
		 * IRAM page table.
		 */
		aips1_base = ioremap(MX7ULP_AIPS1_BASE_ADDR, SZ_1M);
		j = (((u32)aips1_base >> 20) << 2) / 4;
		*((unsigned long *)iram_tlb_base_addr + j) =
			((MX7ULP_AIPS1_BASE_ADDR) & ADDR_1M_MASK) |
			TT_ATTRIB_NON_CACHEABLE_1M;
		/*
		 * Make sure the AIPS2 virtual address has a mapping in the
		 * IRAM page table.
		 */
		aips2_base = ioremap(MX7ULP_AIPS2_BASE_ADDR, SZ_1M);
		j = (((u32)aips2_base >> 20) << 2) / 4;
		*((unsigned long *)iram_tlb_base_addr + j) =
			((MX7ULP_AIPS2_BASE_ADDR) & ADDR_1M_MASK) |
			TT_ATTRIB_NON_CACHEABLE_1M;
		/*
		 * Make sure the AIPS3 virtual address has a mapping in the
		 * IRAM page table.
		 */
		aips3_base = ioremap(MX7ULP_AIPS3_BASE_ADDR, SZ_1M);
		j = (((u32)aips3_base >> 20) << 2) / 4;
		*((unsigned long *)iram_tlb_base_addr + j) =
			((MX7ULP_AIPS3_BASE_ADDR) & ADDR_1M_MASK) |
			TT_ATTRIB_NON_CACHEABLE_1M;
		/*
		 * Make sure the AIPS4 virtual address has a mapping in the
		 * IRAM page table.
		 */
		aips4_base = ioremap(MX7ULP_AIPS4_BASE_ADDR, SZ_1M);
		j = (((u32)aips4_base >> 20) << 2) / 4;
		*((unsigned long *)iram_tlb_base_addr + j) =
			((MX7ULP_AIPS4_BASE_ADDR) & ADDR_1M_MASK) |
			TT_ATTRIB_NON_CACHEABLE_1M;
		/*
		 * Make sure the AIPS5 virtual address has a mapping in the
		 * IRAM page table.
		 */
		aips5_base = ioremap(MX7ULP_AIPS5_BASE_ADDR, SZ_1M);
		j = (((u32)aips5_base >> 20) << 2) / 4;
		*((unsigned long *)iram_tlb_base_addr + j) =
			((MX7ULP_AIPS5_BASE_ADDR) & ADDR_1M_MASK) |
			TT_ATTRIB_NON_CACHEABLE_1M;
	}

	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-smc1");
	smc1_base = of_iomap(np, 0);
	WARN_ON(!smc1_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-pmc0");
	pmc0_base = of_iomap(np, 0);
	WARN_ON(!pmc0_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-pmc1");
	pmc1_base = of_iomap(np, 0);
	WARN_ON(!pmc1_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-tpm");
	tpm5_base = of_iomap(np, 0);
	WARN_ON(!tpm5_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-lpuart");
	lpuart4_base = of_iomap(np, 0);
	WARN_ON(!lpuart4_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-pcc2");
	pcc2_base = of_iomap(np, 0);
	WARN_ON(!pcc2_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-pcc3");
	pcc3_base = of_iomap(np, 0);
	WARN_ON(!pcc3_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-iomuxc-1");
	iomuxc1_base = of_iomap(np, 0);
	WARN_ON(!iomuxc1_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-scg1");
	scg1_base = of_iomap(np, 0);
	WARN_ON(!scg1_base);

	np = NULL;
	for (i = 0; i < 4; i++) {
		np = of_find_compatible_node(np, NULL, "fsl,vf610-gpio");
		gpio_base[i] = of_iomap(np, 1);
		WARN_ON(!gpio_base[i]);
	}

	if (psci_ops.cpu_suspend) {
		pm_info = kzalloc(SZ_16K, GFP_KERNEL);
		if (!pm_info)
			panic("pm info allocation failed\n");
	} else {
		/*
		 * 16KB is allocated for IRAM TLB, but only up 8k is for kernel TLB,
		 * The lower 8K is not used, so use the lower 8K for IRAM code and
		 * pm_info.
		 *
		 */
		sram_paddr = iram_tlb_phys_addr;

		/* Make sure sram_paddr is 8 byte aligned. */
		if ((uintptr_t)(sram_paddr) & (FNCPY_ALIGN - 1))
			sram_paddr += FNCPY_ALIGN - sram_paddr % (FNCPY_ALIGN);

		/* Get the virtual address of the suspend code. */
		suspend_ocram_base = (void *)IMX_IO_P2V(sram_paddr);

		pm_info = suspend_ocram_base;
	}
	pm_info->pbase = sram_paddr;
	pm_info->resume_addr = virt_to_phys(imx7ulp_cpu_resume);
	pm_info->pm_info_size = sizeof(*pm_info);

	pm_info->scg1_base = aips2_base +
		(MX7ULP_SCG1_BASE_ADDR & ~ADDR_1M_MASK);
	pm_info->smc1_base = aips3_base +
		(MX7ULP_SMC1_BASE_ADDR & ~ADDR_1M_MASK);
	pm_info->mmdc_base = aips4_base +
		(MX7ULP_MMDC_BASE_ADDR & ~ADDR_1M_MASK);
	pm_info->mmdc_io_base = aips4_base +
		(MX7ULP_MMDC_IO_BASE_ADDR & ~ADDR_1M_MASK);
	pm_info->sim_base = aips5_base +
		(MX7ULP_SIM_BASE_ADDR & ~ADDR_1M_MASK);

	pm_info->mmdc_io_num = socdata->mmdc_io_num;
	mmdc_io_offset_array = socdata->mmdc_io_offset;
	pm_info->mmdc_num = socdata->mmdc_num;
	mmdc_offset_array = socdata->mmdc_offset;

	for (i = 0; i < pm_info->mmdc_io_num; i++) {
		pm_info->mmdc_io_val[i][0] =
			mmdc_io_offset_array[i];
		pm_info->mmdc_io_val[i][1] =
			readl_relaxed(pm_info->mmdc_io_base +
			mmdc_io_offset_array[i]);
	}

	/* initialize MMDC settings */
	for (i = 0; i < pm_info->mmdc_num; i++)
		pm_info->mmdc_val[i][0] =
			mmdc_offset_array[i];

	for (i = 0; i < pm_info->mmdc_num; i++)
		pm_info->mmdc_val[i][1] = imx7ulp_lpddr3_script[i];

	if (!psci_ops.cpu_suspend) {
		imx7ulp_suspend_in_ocram_fn = fncpy(
			suspend_ocram_base + sizeof(*pm_info),
			&imx7ulp_suspend,
			MX7ULP_SUSPEND_OCRAM_SIZE - sizeof(*pm_info));
	}

	if (IS_ENABLED(CONFIG_SUSPEND)) {
		ret = imx7ulp_suspend_init();
		if (ret)
			pr_warn("%s: No DDR LPM support with suspend %d!\n",
				__func__, ret);
	}
}

void __init imx7ulp_pm_init(void)
{
	imx7ulp_pm_common_init(&imx7ulp_lpddr3_pm_data);
	imx7ulp_set_lpm(RUN);
}

static irqreturn_t imx7ulp_nmi_isr(int irq, void *param)
{
	writel_relaxed(readl_relaxed(mu_base + MU_SR) | MU_B_SR_NMIC,
		mu_base + MU_SR);
	pm_system_wakeup();

	return IRQ_HANDLED;
}

void imx7ulp_enable_nmi(void)
{
	struct device_node *np;
	int irq, ret;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-nmi");
	mu_base = of_iomap(np, 0);
	WARN_ON(!mu_base);
	irq = of_irq_get(np, 0);
	ret = request_irq(irq, imx7ulp_nmi_isr,
		IRQF_NO_SUSPEND, "imx7ulp-nmi", NULL);
	if (ret) {
		pr_err("%s: register interrupt %d failed, rc %d\n",
			__func__, irq, ret);
		return;
	}
}
