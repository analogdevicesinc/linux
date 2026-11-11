// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2024 Linus Walleij <linus.walleij@linaro.org>

#include <linux/io.h>
#include <linux/of.h>
#include <linux/smp.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/smp_plat.h>

#ifdef CONFIG_DEBUG_BCMBCA
/* This is needed for LL-debug/earlyprintk/debug-macro.S */
static struct map_desc bcmbca_io_desc[] __initdata = {
	{
		.virtual = CONFIG_DEBUG_UART_VIRT,
		.pfn = __phys_to_pfn(CONFIG_DEBUG_UART_PHYS),
		.length = SZ_4K,
		.type = MT_DEVICE,
	},
};
#endif

static void __init bcmbca_map_io(void)
{
#ifdef CONFIG_DEBUG_BCMBCA
	iotable_init(bcmbca_io_desc, ARRAY_SIZE(bcmbca_io_desc));
#endif
}

#ifdef CONFIG_SMP
static int bcmbca_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	struct device_node *np;
	void __iomem *mailbox;
	u32 release_addr;
	int ret;

	/* CFE has already powered up CPU1 and left it waiting in WFI. */
	if (cpu_logical_map(cpu) != 1)
		return -EINVAL;

	np = of_get_cpu_node(cpu, NULL);
	if (!np)
		return -ENODEV;

	ret = of_property_read_u32(np, "cpu-release-addr", &release_addr);
	of_node_put(np);
	if (ret) {
		pr_err("CPU%u: missing or invalid cpu-release-addr property\n", cpu);
		return ret;
	}

	mailbox = ioremap(release_addr, sizeof(u32));
	if (!mailbox)
		return -ENOMEM;

	/*
	 * The CFE stub reads GPIO GeneralPurpose after an interrupt and
	 * branches to it in ARM state. Clear any Thumb function pointer
	 * tag and complete the MMIO write before sending the wakeup IPI.
	 */
	writel(__pa_symbol(secondary_startup_arm) & ~1, mailbox);
	dsb(sy);
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));
	iounmap(mailbox);

	return 0;
}

static const struct smp_operations bcmbca_smp_ops __initconst = {
	.smp_boot_secondary = bcmbca_boot_secondary,
};

CPU_METHOD_OF_DECLARE(bcmbca_smp, "brcm,bca-smp", &bcmbca_smp_ops);
#endif

static const char * const bcmbca_dt_compat[] = {
	/* TODO: Add other BCMBCA SoCs here to get debug UART support */
	"brcm,bcm6846",
	NULL,
};

DT_MACHINE_START(BCMBCA_DT, "BCMBCA Broadband Access Processors")
	.map_io = bcmbca_map_io,
	.dt_compat = bcmbca_dt_compat,
MACHINE_END
