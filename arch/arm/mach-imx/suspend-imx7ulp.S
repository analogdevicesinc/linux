/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/linkage.h>
#include <asm/asm-offsets.h>
#include "hardware.h"

/*
 * ==================== low level suspend ====================
 *
 * Better to follow below rules to use ARM registers:
 * r0: pm_info structure address;
 *
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

/*
 * Below offsets are based on struct imx7ulp_cpu_pm_info
 * which defined in arch/arm/mach-imx/pm-imx7ulp.c, this
 * structure contains necessary pm info for low level
 * suspend related code.
 */
#define PM_INFO_M4_RESERVE0_OFFSET		0x0
#define PM_INFO_M4_RESERVE1_OFFSET		0x4
#define PM_INFO_M4_RESERVE2_OFFSET		0x8
#define PM_INFO_PBASE_OFFSET			0xc
#define PM_INFO_RESUME_ADDR_OFFSET		0x10
#define PM_INFO_PM_INFO_SIZE_OFFSET		0x14
#define PM_INFO_PM_INFO_SIM_PBASE_OFFSET	0x18
#define PM_INFO_PM_INFO_SIM_VBASE_OFFSET	0x1c
#define PM_INFO_PM_INFO_SCG1_PBASE_OFFSET	0x20
#define PM_INFO_PM_INFO_SCG1_VBASE_OFFSET	0x24
#define PM_INFO_PM_INFO_MMDC_PBASE_OFFSET	0x28
#define PM_INFO_PM_INFO_MMDC_VBASE_OFFSET	0x2c
#define PM_INFO_PM_INFO_MMDC_IO_PBASE_OFFSET	0x30
#define PM_INFO_PM_INFO_MMDC_IO_VBASE_OFFSET	0x34
#define PM_INFO_PM_INFO_SCG1_VAL_OFFSET		0x38
#define PM_INFO_MX7ULP_TTBR1_V_OFFSET		0x78
#define PM_INFO_MMDC_IO_NUM_OFFSET		0x7c
#define PM_INFO_MMDC_IO_VAL_OFFSET		0x80
/* below offsets depends on MX7ULP_MAX_MMDC_IO_NUM(36) definition */
#define PM_INFO_MMDC_NUM_OFFSET			0x1a0
#define PM_INFO_MMDC_VAL_OFFSET			0x1a4

#define DGO_GPR3	0x60
#define DGO_GPR4	0x64

#define MX7ULP_MMDC_MISC	0x18
#define MX7ULP_MMDC_MAPSR	0x404
#define MX7ULP_MMDC_MPDGCTRL0	0x83c

#define SCG_RCCR	0x14
#define SCG_DDRCCR	0x30
#define SCG_NICCCR	0x40
#define SCG_FIRCDIV	0x304
#define SCG_APLLCSR	0x500
#define SCG_APLLDIV	0x504
#define SCG_APLLCFG	0x508
#define SCG_APLLPFD	0x50c
#define SCG_APLLNUM	0x510
#define SCG_APLLDENOM	0x514
#define SCG_SPLLCSR	0x600
#define SCG_SPLLDIV	0x604
#define SCG_SPLLCFG	0x608
#define SCG_SPLLPFD	0x60c
#define SCG_SPLLNUM	0x610
#define SCG_SPLLDENOM	0x614
#define SCG_SOSCDIV	0x104

#define PMC1_CTRL	0x24

	.align 3

	.macro store_ttbr1

	/* Store TTBR1 to pm_info->ttbr1 */
	mrc	p15, 0, r7, c2, c0, 1
	str	r7, [r0, #PM_INFO_MX7ULP_TTBR1_V_OFFSET]

	/* Disable Branch Prediction, Z bit in SCTLR. */
	mrc	p15, 0, r6, c1, c0, 0
	bic	r6, r6, #0x800
	mcr	p15, 0, r6, c1, c0, 0

	/* Flush the BTAC. */
	ldr	r6, =0x0
	mcr	p15, 0, r6, c7, c1, 6

	ldr	r6, =iram_tlb_phys_addr
	ldr	r6, [r6]
	dsb
	isb

	/* Store the IRAM table in TTBR1 */
	mcr	p15, 0, r6, c2, c0, 1
	/* Read TTBCR and set PD0=1, N = 1 */
	mrc	p15, 0, r6, c2, c0, 2
	orr	r6, r6, #0x11
	mcr	p15, 0, r6, c2, c0, 2

	dsb
	isb

	/* flush the TLB */
	ldr	r6, =0x0
	mcr	p15, 0, r6, c8, c3, 0

	.endm

	.macro restore_ttbr1

	/* Enable L1 data cache. */
	mrc	p15, 0, r6, c1, c0, 0
	orr	r6, r6, #0x4
	mcr	p15, 0, r6, c1, c0, 0

	dsb
	isb

	/* Restore TTBCR */
	/* Read TTBCR and set PD0=0, N = 0 */
	mrc	p15, 0, r6, c2, c0, 2
	bic	r6, r6, #0x11
	mcr	p15, 0, r6, c2, c0, 2
	dsb
	isb

	/* flush the TLB */
	ldr	r6, =0x0
	mcr	p15, 0, r6, c8, c3, 0

	/* Enable Branch Prediction, Z bit in SCTLR. */
	mrc	p15, 0, r6, c1, c0, 0
	orr	r6, r6, #0x800
	mcr	p15, 0, r6, c1, c0, 0

	/* Flush the Branch Target Address Cache (BTAC) */
	ldr	r6, =0x0
	mcr	p15, 0, r6, c7, c1, 6

	/* Restore TTBR1, get the origin ttbr1 from pm info */
	ldr	r7, [r0, #PM_INFO_MX7ULP_TTBR1_V_OFFSET]
	mcr	p15, 0, r7, c2, c0, 1

	.endm

	.macro	disable_l1_dcache

	/*
	 * Flush all data from the L1 data cache before disabling
	 * SCTLR.C bit.
	 */
	push	{r0 - r10, lr}
	ldr	r7, =v7_flush_dcache_all
	mov	lr, pc
	mov	pc, r7
	pop	{r0 - r10, lr}

	/* disable d-cache */
	mrc	p15, 0, r7, c1, c0, 0
	bic	r7, r7, #(1 << 2)
	mcr	p15, 0, r7, c1, c0, 0
	dsb
	isb

	push	{r0 - r10, lr}
	ldr	r7, =v7_flush_dcache_all
	mov	lr, pc
	mov	pc, r7
	pop	{r0 - r10, lr}

	.endm

	.macro	restore_mmdc_settings

	ldr	r10, [r0, #PM_INFO_PM_INFO_MMDC_IO_PBASE_OFFSET]
	ldr	r11, [r0, #PM_INFO_PM_INFO_MMDC_PBASE_OFFSET]

	/* resume mmdc iomuxc settings */
	ldr	r6, [r0, #PM_INFO_MMDC_IO_NUM_OFFSET]
	ldr	r7, =PM_INFO_MMDC_IO_VAL_OFFSET
	add	r7, r7, r0
11:
	ldr	r8, [r7], #0x4
	ldr	r9, [r7], #0x4
	str	r9, [r10, r8]
	subs	r6, r6, #0x1
	bne	11b

	/* restore MMDC settings */
	ldr	r6, [r0, #PM_INFO_MMDC_NUM_OFFSET]
	ldr	r7, =PM_INFO_MMDC_VAL_OFFSET
	add	r7, r7, r0
1:
	ldr	r8, [r7], #0x4
	ldr	r9, [r7], #0x4
	str	r9, [r11, r8]
	subs	r6, r6, #0x1
	bne	1b

	/* let DDR enter self-refresh */
	ldr	r7, [r11, #MX7ULP_MMDC_MAPSR]
	orr	r7, r7, #(1 << 20)
	str	r7, [r11, #MX7ULP_MMDC_MAPSR]
2:
	ldr	r7, [r11, #MX7ULP_MMDC_MAPSR]
	ands	r7, r7, #(1 << 24)
	beq	2b

	/* let DDR out of self-refresh */
	ldr	r7, [r11, #MX7ULP_MMDC_MAPSR]
	bic	r7, r7, #(1 << 20)
	str	r7, [r11, #MX7ULP_MMDC_MAPSR]
3:
	ldr	r7, [r11, #MX7ULP_MMDC_MAPSR]
	ands	r7, r7, #(1 << 24)
	bne	3b

	/* kick off MMDC */
	ldr	r4, =0x0
	str	r4, [r11, #0x1c]

	/* let DDR out of self-refresh */
	ldr	r7, [r11, #MX7ULP_MMDC_MAPSR]
	bic	r7, r7, #(1 << 20)
	str	r7, [r11, #MX7ULP_MMDC_MAPSR]
4:
	ldr	r7, [r11, #MX7ULP_MMDC_MAPSR]
	ands	r7, r7, #(1 << 24)
	bne	4b

	/* enable DDR auto power saving */
	ldr	r7, [r11, #MX7ULP_MMDC_MAPSR]
	bic	r7, r7, #0x1
	str	r7, [r11, #MX7ULP_MMDC_MAPSR]

	.endm

ENTRY(imx7ulp_suspend)
	push	{r4-r12}

	/*
	 * The value of r0 is mapped the same in origin table and IRAM table,
	 * thus no need to care r0 here.
	 */
	ldr	r1, [r0, #PM_INFO_PBASE_OFFSET]
	ldr	r2, [r0, #PM_INFO_RESUME_ADDR_OFFSET]
	ldr	r3, [r0, #PM_INFO_PM_INFO_SIZE_OFFSET]

	/*
	 * counting the resume address in iram
	 * to set it in SRC register.
	 */
	ldr	r6, =imx7ulp_suspend
	ldr	r7, =resume
	sub	r7, r7, r6
	add	r8, r1, r3
	add	r9, r8, r7

	ldr	r11, [r0, #PM_INFO_PM_INFO_SIM_VBASE_OFFSET]
	/* store physical resume addr and pm_info address. */
	str	r9, [r11, #DGO_GPR3]
	str	r1, [r11, #DGO_GPR4]

	disable_l1_dcache

	store_ttbr1

	ldr	r11, [r0, #PM_INFO_PM_INFO_MMDC_VBASE_OFFSET]

	/*
	 * put DDR explicitly into self-refresh and
	 * disable automatic power savings.
	 */
	ldr	r7, [r11, #MX7ULP_MMDC_MAPSR]
	orr	r7, r7, #0x1
	str	r7, [r11, #MX7ULP_MMDC_MAPSR]

	/* make the DDR explicitly enter self-refresh. */
	ldr	r7, [r11, #MX7ULP_MMDC_MAPSR]
	orr	r7, r7, #(1 << 20)
	str	r7, [r11, #MX7ULP_MMDC_MAPSR]

poll_dvfs_set:
	ldr	r7, [r11, #MX7ULP_MMDC_MAPSR]
	ands	r7, r7, #(1 << 24)
	beq	poll_dvfs_set

	/* Zzz, enter stop mode */
	wfi
	nop
	nop
	nop
	nop

	/* clear core0's entry and parameter */
	ldr	r10, [r0, #PM_INFO_PM_INFO_SIM_VBASE_OFFSET]
	mov	r7, #0x0
	str	r7, [r10, #DGO_GPR3]
	str	r7, [r10, #DGO_GPR4]

	ldr	r10, [r0, #PM_INFO_PM_INFO_SCG1_VBASE_OFFSET]
wait_apll:
	ldr	r7, [r10, #SCG_APLLCSR]
	ands	r7, r7, #(1 << 24)
	beq	wait_apll

	/* let DDR out of self-refresh */
	ldr	r7, [r11, #MX7ULP_MMDC_MAPSR]
	bic	r7, r7, #(1 << 20)
	str	r7, [r11, #MX7ULP_MMDC_MAPSR]
poll_dvfs_clear:
	ldr	r7, [r11, #MX7ULP_MMDC_MAPSR]
	ands	r7, r7, #(1 << 24)
	bne	poll_dvfs_clear

	/* enable DDR auto power saving */
	ldr	r7, [r11, #MX7ULP_MMDC_MAPSR]
	bic	r7, r7, #0x1
	str	r7, [r11, #MX7ULP_MMDC_MAPSR]

	restore_ttbr1
	pop	{r4-r12}
	/* return to suspend finish */
	mov	pc, lr

resume:
	/* invalidate L1 I-cache first */
	mov     r6, #0x0
	mcr     p15, 0, r6, c7, c5, 0
	mcr     p15, 0, r6, c7, c5, 6
	/* enable the Icache and branch prediction */
	mov     r6, #0x1800
	mcr     p15, 0, r6, c1, c0, 0
	isb

	ldr	r6, =MX7ULP_PMC1_BASE_ADDR
	ldr	r7, [r6, #PMC1_CTRL]
	orr	r7, r7, #(1 << 14)
	str	r7, [r6, #PMC1_CTRL]

	ldr	r0, =0x20008000
	/* get physical resume address from pm_info. */
	ldr	lr, [r0, #PM_INFO_RESUME_ADDR_OFFSET]

	ldr	r11, =MX7ULP_SCG1_BASE_ADDR
	/* enable spll and pfd0 */
	ldr	r5, =PM_INFO_PM_INFO_SCG1_VAL_OFFSET
	add	r6, r5, #48
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_SPLLCFG]

	add	r6, r5, #56
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_SPLLNUM]

	add	r6, r5, #60
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_SPLLDENOM]

	add	r6, r5, #40
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_SPLLCSR]
5:
	ldr	r7, [r11, #SCG_SPLLCSR]
	ands	r7, r7, #0x1000000
	beq	5b

	add	r6, r5, #44
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_SPLLDIV]

	add	r6, r5, #52
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_SPLLPFD]

	add	r6, r5, #0
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_RCCR]

	/* enable apll and pfd0 */
	add	r6, r5, #24
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_APLLCFG]

	add	r6, r5, #32
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_APLLNUM]

	add	r6, r5, #36
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_APLLDENOM]

	add	r6, r5, #16
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_APLLCSR]
6:
	ldr	r7, [r11, #SCG_APLLCSR]
	ands	r7, r7, #0x1000000
	beq	6b

	add	r6, r5, #20
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_APLLDIV]

	add	r6, r5, #28
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_APLLPFD]

	/* set ddr ccr */
	add	r6, r5, #4
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_DDRCCR]

	/* set nic sel */
	add	r6, r5, #8
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_NICCCR]

	/* set firc div2 to get 48MHz */
	add	r6, r5, #12
	ldr	r7, [r0, r6]
	str	r7, [r11, #SCG_FIRCDIV]

	/* enable mmdc clock in pcc3 */
	ldr	r11, =MX7ULP_PCC3_BASE_ADDR
	ldr	r7, [r11, #0xac]
	orr	r7, r7, #0x40000000
	str	r7, [r11, #0xac]

	restore_mmdc_settings

	mov	pc, lr
ENDPROC(imx7ulp_suspend)

ENTRY(imx7ulp_cpu_resume)
	bl	v7_invalidate_l1
	b	cpu_resume
ENDPROC(imx7ulp_cpu_resume)
