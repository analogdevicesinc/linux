/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/* Includes */
#include <linux/arm-smccc.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_fdt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mx8_mu.h>
#include <linux/syscore_ops.h>

#include <soc/imx/fsl_hvc.h>
#include <soc/imx8/sc/svc/irq/api.h>
#include <soc/imx8/sc/ipc.h>
#include <soc/imx8/sc/sci.h>

#include "rpc.h"

/* Local Defines */
#define MU_SIZE 0x10000

/* Local Types */
unsigned int scu_mu_id;
static void __iomem *mu_base_virtaddr;
static struct delayed_work scu_mu_work;
static sc_ipc_t mu_ipcHandle;

/* Local functions */

/* Local variables */
static uint32_t gIPCport;
static bool scu_mu_init;

DEFINE_MUTEX(scu_mu_mutex);

static BLOCKING_NOTIFIER_HEAD(SCU_notifier_chain);

EXPORT_SYMBOL(sc_pm_set_resource_power_mode);
EXPORT_SYMBOL(sc_pm_get_resource_power_mode);
EXPORT_SYMBOL(sc_pm_cpu_start);
EXPORT_SYMBOL(sc_misc_set_control);
EXPORT_SYMBOL(sc_pm_clock_enable);
EXPORT_SYMBOL(sc_pm_set_clock_rate);
/*--------------------------------------------------------------------------*/
/* RPC command/response                                                     */
/*--------------------------------------------------------------------------*/
void sc_call_rpc(sc_ipc_t handle, sc_rpc_msg_t *msg, bool no_resp)
{
	struct arm_smccc_res res;

	if (in_interrupt()) {
		pr_warn("Cannot make SC IPC calls from an interrupt context\n");
		dump_stack();
		return;
	}
	mutex_lock(&scu_mu_mutex);

	if (xen_initial_domain()) {
		arm_smccc_hvc(FSL_HVC_SC, (uint64_t)msg, no_resp, 0, 0, 0, 0,
			      0, &res);
		if (res.a0)
			printk("Error FSL_HVC_SC %ld\n", res.a0);
	} else {
		sc_ipc_write(handle, msg);
		if (!no_resp)
			sc_ipc_read(handle, msg);
	}

	mutex_unlock(&scu_mu_mutex);
}
EXPORT_SYMBOL(sc_call_rpc);

/*--------------------------------------------------------------------------*/
/* Get MU base address for specified IPC channel                            */
/*--------------------------------------------------------------------------*/
static uint32_t *sc_ipc_get_mu_base(uint32_t id)
{
	uint32_t *base;

	/* Check parameters */
	if (id >= SC_NUM_IPC)
		base = NULL;
	else
		base = (uint32_t *) (mu_base_virtaddr + (id * MU_SIZE));

	return base;
}

/*--------------------------------------------------------------------------*/
/* Get the MU ID used by Linux                                              */
/*--------------------------------------------------------------------------*/
int sc_ipc_getMuID(uint32_t *mu_id)
{
	if (scu_mu_init) {
		*mu_id = scu_mu_id;
		return SC_ERR_NONE;
	}
	return SC_ERR_UNAVAILABLE;
}

EXPORT_SYMBOL(sc_ipc_getMuID);

/*--------------------------------------------------------------------------*/
/* Open an IPC channel                                                      */
/*--------------------------------------------------------------------------*/
sc_err_t sc_ipc_requestInt(sc_ipc_t *handle, uint32_t id)
{
	return SC_ERR_NONE;
}

/*--------------------------------------------------------------------------*/
/* Open an IPC channel                                                      */
/*--------------------------------------------------------------------------*/
sc_err_t sc_ipc_open(sc_ipc_t *handle, uint32_t id)
{
	uint32_t *base;

	mutex_lock(&scu_mu_mutex);

	if (!scu_mu_init) {
		mutex_unlock(&scu_mu_mutex);
		return SC_ERR_UNAVAILABLE;
	}
	/* Get MU base associated with IPC channel */
	base = sc_ipc_get_mu_base(id);

	if (base == NULL) {
		mutex_unlock(&scu_mu_mutex);
		return SC_ERR_IPC;
	}

	*handle = (sc_ipc_t) task_pid_vnr(current);

	mutex_unlock(&scu_mu_mutex);

	return SC_ERR_NONE;
}
EXPORT_SYMBOL(sc_ipc_open);
/*--------------------------------------------------------------------------*/
/* Close an IPC channel                                                     */
/*--------------------------------------------------------------------------*/
void sc_ipc_close(sc_ipc_t handle)
{
	uint32_t *base;

	mutex_lock(&scu_mu_mutex);

	if (!scu_mu_init) {
		mutex_unlock(&scu_mu_mutex);
		return;
	}

	/* Get MU base associated with IPC channel */
	base = sc_ipc_get_mu_base(gIPCport);

	/* TBD ***** What needs to be done here? */
	mutex_unlock(&scu_mu_mutex);
}
EXPORT_SYMBOL(sc_ipc_close);

/*!
 * This function reads a message from an IPC channel.
 *
 * @param[in]     ipc         id of channel read from
 * @param[out]    data        pointer to message buffer to read
 *
 * This function will block if no message is available to be read.
 */
void sc_ipc_read(sc_ipc_t handle, void *data)
{
	uint32_t *base;
	uint8_t count = 0;
	sc_rpc_msg_t *msg = (sc_rpc_msg_t *) data;

	/* Get MU base associated with IPC channel */
	base = sc_ipc_get_mu_base(gIPCport);

	if ((base == NULL) || (msg == NULL))
		return;

	/* Read first word */
	MU_ReceiveMsg(base, 0, (uint32_t *) msg);
	count++;

	/* Check size */
	if (msg->size > SC_RPC_MAX_MSG) {
		*((uint32_t *) msg) = 0;
		return;
	}

	/* Read remaining words */
	while (count < msg->size) {
		MU_ReceiveMsg(base, count % MU_RR_COUNT,
				  &(msg->DATA.u32[count - 1]));
		count++;
	}
}

/*!
 * This function writes a message to an IPC channel.
 *
 * @param[in]     ipc         id of channel to write to
 * @param[in]     data        pointer to message buffer to write
 *
 * This function will block if the outgoing buffer is full.
 */
void sc_ipc_write(sc_ipc_t handle, void *data)
{
	uint32_t *base;
	uint8_t count = 0;
	sc_rpc_msg_t *msg = (sc_rpc_msg_t *) data;

	/* Get MU base associated with IPC channel */
	base = sc_ipc_get_mu_base(gIPCport);

	if ((base == NULL) || (msg == NULL))
		return;

	/* Check size */
	if (msg->size > SC_RPC_MAX_MSG)
		return;

	/* Write first word */
	MU_SendMessage(base, 0, *((uint32_t *) msg));
	count++;

	/* Write remaining words */
	while (count < msg->size) {
		MU_SendMessage(base, count % MU_TR_COUNT, msg->DATA.u32[count - 1]);
		count++;
	}
}

int register_scu_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&SCU_notifier_chain, nb);
}

EXPORT_SYMBOL(register_scu_notifier);

int unregister_scu_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&SCU_notifier_chain, nb);
}

EXPORT_SYMBOL(unregister_scu_notifier);

static int SCU_notifier_call_chain(unsigned long status, sc_irq_group_t *group)
{
	return blocking_notifier_call_chain(&SCU_notifier_chain, status,
						(void *)group);
}

static void scu_mu_work_handler(struct work_struct *work)
{
	uint32_t irq_status;
	sc_err_t sciErr;
	sc_irq_group_t i;

	/* Walk all groups interrupt callback, callback will judge if it's
	 * the right group for itself, return directly if not.
	 */
	for (i = 0; i < SC_IRQ_NUM_GROUP; i++) {
		sciErr = sc_irq_status(mu_ipcHandle, SC_R_MU_1A, i,
					&irq_status);
		/* no irq? */
		if (!irq_status)
			continue;

		SCU_notifier_call_chain(irq_status, &i);
	}
}

static irqreturn_t imx8_scu_mu_isr(int irq, void *param)
{
	u32 irqs;

	irqs = (readl_relaxed(mu_base_virtaddr + 0x20) & (0xf << 28));
	if (irqs) {
		/* Clear the General Interrupt */
		writel_relaxed(irqs, mu_base_virtaddr + 0x20);
		/* Setup a bottom-half to handle the irq work. */
		schedule_delayed_work(&scu_mu_work, 0);
	}
	return IRQ_HANDLED;
}

static void imx8_mu_resume(void)
{
	int i;

	MU_Init(mu_base_virtaddr);
	for (i = 0; i < MU_RR_COUNT; i++)
		MU_EnableGeneralInt(mu_base_virtaddr, i);
}

struct syscore_ops imx8_mu_syscore_ops = {
	.resume = imx8_mu_resume,
};

/*Initialization of the MU code. */
int __init imx8_mu_init(void)
{
	struct device_node *np;
	int irq;
	int err;
	sc_err_t sciErr;

	/*
	 * Get the address of MU to be used for communication with the SCU
	 */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx8-mu");
	if (!np) {
		pr_info("Cannot find MU entry in device tree\n");
		return 0;
	}
	mu_base_virtaddr = of_iomap(np, 0);
	WARN_ON(!mu_base_virtaddr);

	err = of_property_read_u32_index(np, "fsl,scu_ap_mu_id", 0, &scu_mu_id);
	if (err)
		pr_info("imx8_mu_init: Cannot get mu_id err = %d\n", err);

	irq = of_irq_get(np, 0);

	if (irq <= 0) {
		/* SCU works just fine without irq */
		pr_warn("imx8_mu_init: no irq: %d\n", irq);
	} else {
		err = request_irq(irq, imx8_scu_mu_isr,
				  IRQF_EARLY_RESUME, "imx8_mu_isr", NULL);
		if (err) {
			pr_err("imx8_mu_init: request_irq %d failed: %d\n",
					irq, err);
			return err;
		}

		err = irq_set_irq_wake(irq, 1);
		if (err)
			pr_err("imx8mu_init: set_irq_wake failed: %d\n", err);
	}

	if (!scu_mu_init) {
		uint32_t i;

		INIT_DELAYED_WORK(&scu_mu_work, scu_mu_work_handler);

		/* Init MU */
		MU_Init(mu_base_virtaddr);

#if 1
		/* Enable all RX interrupts */
		for (i = 0; i < MU_RR_COUNT; i++)
			MU_EnableGeneralInt(mu_base_virtaddr, i);
#endif
		gIPCport = scu_mu_id;
		scu_mu_init = true;
	}

	sciErr = sc_ipc_open(&mu_ipcHandle, scu_mu_id);
	if (sciErr != SC_ERR_NONE) {
		pr_info("Cannot open MU channel to SCU\n");
		return sciErr;
	};

	/* Request for the high temp interrupt. */
	sciErr = sc_irq_enable(mu_ipcHandle, SC_R_MU_1A, SC_IRQ_GROUP_TEMP,
			       SC_IRQ_TEMP_PMIC0_HIGH, true);

	if (sciErr)
		pr_info("Cannot request PMIC0_TEMP interrupt\n");

	/* Request for the high temp interrupt. */
	sciErr = sc_irq_enable(mu_ipcHandle, SC_R_MU_1A, SC_IRQ_GROUP_TEMP,
			       SC_IRQ_TEMP_PMIC1_HIGH, true);

	if (sciErr)
		pr_info("Cannot request PMIC1_TEMP interrupt\n");

	/* Request for the rtc alarm interrupt. */
	sciErr = sc_irq_enable(mu_ipcHandle, SC_R_MU_1A, SC_IRQ_GROUP_RTC,
			       SC_IRQ_RTC, true);

	if (sciErr)
		pr_info("Cannot request ALARM_RTC interrupt\n");

	/* Request for the ON/OFF interrupt. */
	sciErr = sc_irq_enable(mu_ipcHandle, SC_R_MU_1A, SC_IRQ_GROUP_WAKE,
			       SC_IRQ_BUTTON, true);

	if (sciErr)
		pr_info("Cannot request ON/OFF interrupt\n");

	/* Request for the watchdog interrupt. */
	sciErr = sc_irq_enable(mu_ipcHandle, SC_R_MU_1A, SC_IRQ_GROUP_WDOG,
			       SC_IRQ_WDOG, true);

	if (sciErr)
		pr_info("Cannot request WDOG interrupt\n");

	register_syscore_ops(&imx8_mu_syscore_ops);

	pr_info("*****Initialized MU\n");
	return scu_mu_id;
}

early_initcall(imx8_mu_init);
