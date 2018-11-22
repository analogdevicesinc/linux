// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file   busfreq_optee.c
 *
 * @brief  iMX.6 and i.MX7 Bus Frequency change.\n
 *         Call OPTEE busfreq function regardless memory type and device.
 *
 * @ingroup PM
 */
#include <asm/fncpy.h>
#include <linux/busfreq-imx.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#include "hardware.h"
#include "smc_sip.h"


extern unsigned int ddr_normal_rate;
static int curr_ddr_rate;

#ifdef CONFIG_SMP
/*
 * External declaration
 */
extern void imx_smp_wfe_optee(u32 cpuid, u32 status_addr);
extern unsigned long imx_smp_wfe_start asm("imx_smp_wfe_optee");
extern unsigned long imx_smp_wfe_end asm("imx_smp_wfe_optee_end");

extern unsigned long ddr_freq_change_iram_base;


/**
 * @brief  Definition of the synchronization status
 *         structure used to control to CPUs status
 *         and on-going frequency change
 */
struct busfreq_sync {
	uint32_t change_ongoing;
	uint32_t wfe_status[NR_CPUS];
} __aligned(8);

static struct busfreq_sync *pSync;

static void (*wfe_change_freq)(uint32_t *wfe_status, uint32_t *freq_done);

static uint32_t *irqs_for_wfe;
static void __iomem *gic_dist_base;

/**
 * @brief  Switch all active cores, except the one changing the
 *         bus frequency, in WFE mode until completion of the
 *         frequency change
 *
 * @param[in]  irq     Interrupt ID - not used
 * @param[in]  dev_id  Client data - not used
 *
 * @retval IRQ_HANDLED  Interrupt handled
 */
static irqreturn_t wait_in_wfe_irq(int irq, void *dev_id)
{
	uint32_t me;

	me = smp_processor_id();
#ifdef CONFIG_LOCAL_TIMERS
	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_ENTER,
		&me);
#endif

	wfe_change_freq(&pSync->wfe_status[me], &pSync->change_ongoing);

#ifdef CONFIG_LOCAL_TIMERS
	clockevents_notify(CLOCK_EVT_NOTIFY_BROADCAST_EXIT,
		&me);
#endif

	return IRQ_HANDLED;
}
#endif

/**
 * @brief   Request OPTEE OS to change the memory bus frequency
 *          to \a ddr_rate value
 *
 * @param[in]  rate  Bus Frequency
 *
 * @retval 0  Success
 */
int update_freq_optee(int ddr_rate)
{
	struct arm_smccc_res res;

	uint32_t me      = 0;
	uint32_t dll_off = 0;
	int      mode    = get_bus_freq_mode();

#ifdef CONFIG_SMP
	uint32_t reg         = 0;
	uint32_t cpu         = 0;
	uint32_t online_cpus = 0;
	uint32_t all_cpus    = 0;
#endif

	pr_debug("\nBusfreq OPTEE set from %d to %d start...\n",
			curr_ddr_rate, ddr_rate);

	if (ddr_rate == curr_ddr_rate)
		return 0;

	if (cpu_is_imx6()) {
		if ((mode == BUS_FREQ_LOW) || (mode == BUS_FREQ_AUDIO))
			dll_off = 1;
	}

	local_irq_disable();

#ifdef CONFIG_SMP
	me = smp_processor_id();

	/* Make sure all the online cores to be active */
	do {
		all_cpus = 0;

		for_each_online_cpu(cpu)
			all_cpus |= (pSync->wfe_status[cpu] << cpu);
	} while (all_cpus);

	pSync->change_ongoing = 1;
	dsb();

	for_each_online_cpu(cpu) {
		if (cpu != me) {
			online_cpus |= (1 << cpu);
			/* Set the interrupt to be pending in the GIC. */
			reg = 1 << (irqs_for_wfe[cpu] % 32);
			writel_relaxed(reg, gic_dist_base + GIC_DIST_PENDING_SET
				+ (irqs_for_wfe[cpu] / 32) * 4);
		}
	}

	/* Wait for all active CPUs to be in WFE */
	do {
		all_cpus = 0;

		for_each_online_cpu(cpu)
			all_cpus |= (pSync->wfe_status[cpu] << cpu);
	} while (all_cpus != online_cpus);

#endif

	/* Now we can change the DDR frequency. */
	/* Call the TEE SiP */
	arm_smccc_smc(OPTEE_SMC_FAST_CALL_SIP_VAL(IMX_SIP_BUSFREQ_CHANGE),
				ddr_rate, dll_off, 0, 0, 0, 0, 0, &res);

	curr_ddr_rate = ddr_rate;

#ifdef CONFIG_SMP
	/* DDR frequency change is done */
	pSync->change_ongoing = 0;
	dsb();

	/* wake up all the cores. */
	sev();
#endif

	local_irq_enable();

	pr_debug("Busfreq OPTEE set to %d done! cpu=%d\n", ddr_rate, me);

	return 0;
}

#ifdef CONFIG_SMP
static int init_freq_optee_smp(struct platform_device *busfreq_pdev)
{
	struct device_node *node = 0;
	struct device *dev = &busfreq_pdev->dev;
	uint32_t cpu;
	int err;
	int irq;
	struct irq_data *irq_data;
	unsigned long wfe_iram_base;

	if (cpu_is_imx6()) {
		node = of_find_compatible_node(NULL, NULL, "arm,cortex-a9-gic");
		if (!node) {
			if (cpu_is_imx6q())
				pr_debug("failed to find imx6q-a9-gic device tree data!\n");

			return -EINVAL;
		}
	} else {
		node = of_find_compatible_node(NULL, NULL, "arm,cortex-a7-gic");
		if (!node) {
			pr_debug("failed to find imx7d-a7-gic device tree data!\n");
			return -EINVAL;
		}
	}

	gic_dist_base = of_iomap(node, 0);
	WARN(!gic_dist_base, "unable to map gic dist registers\n");

	irqs_for_wfe = devm_kzalloc(dev, sizeof(uint32_t) * num_present_cpus(),
					GFP_KERNEL);

	for_each_online_cpu(cpu) {
		/*
		 * set up a reserved interrupt to get all
		 * the active cores into a WFE state
		 * before changing the DDR frequency.
		 */
		irq = platform_get_irq(busfreq_pdev, cpu);

		if (cpu_is_imx6()) {
			err = request_irq(irq, wait_in_wfe_irq,
				IRQF_PERCPU, "mmdc_1", NULL);
		} else {
			err = request_irq(irq, wait_in_wfe_irq,
				IRQF_PERCPU, "ddrc", NULL);
		}

		if (err) {
			dev_err(dev,
				"Busfreq:request_irq failed %d, err = %d\n",
				irq, err);
			return err;
		}

		err = irq_set_affinity(irq, cpumask_of(cpu));
		if (err) {
			dev_err(dev,
				"Busfreq: Cannot set irq affinity irq=%d,\n",
				irq);
			return err;
		}

		irq_data = irq_get_irq_data(irq);
		irqs_for_wfe[cpu] = irq_data->hwirq + 32;
	}

	/* Store the variable used to communicate between cores */
	pSync = (void *)ddr_freq_change_iram_base;

	memset(pSync, 0, sizeof(*pSync));

	wfe_iram_base = ddr_freq_change_iram_base + sizeof(*pSync);

	if (wfe_iram_base & (FNCPY_ALIGN - 1))
		wfe_iram_base += FNCPY_ALIGN -
				((uintptr_t)wfe_iram_base % (FNCPY_ALIGN));

	wfe_change_freq = (void *)fncpy((void *)wfe_iram_base,
				&imx_smp_wfe_optee,
				((&imx_smp_wfe_end -&imx_smp_wfe_start) *4));

	return 0;

}

int init_freq_optee(struct platform_device *busfreq_pdev)
{
	int err = -EINVAL;
	struct device *dev = &busfreq_pdev->dev;

	if (num_present_cpus() <= 1) {
		wfe_change_freq = NULL;

		/* Allocate the cores synchronization variables (not used) */
		pSync = devm_kzalloc(dev, sizeof(*pSync), GFP_KERNEL);

		if (pSync)
			err = 0;
	} else {
		err = init_freq_optee_smp(busfreq_pdev);
	}

	if (err == 0)
		curr_ddr_rate = ddr_normal_rate;

	return err;
}
#else
int init_freq_optee(struct platform_device *busfreq_pdev)
{
	curr_ddr_rate = ddr_normal_rate;
	return 0;
}
#endif

