// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2019-2024 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

/**
 * DOC: Mali arbiter interface APIs to share GPU between Virtual Machines
 */

#include <mali_kbase.h>
#include "mali_kbase_arbif.h"
#include <tl/mali_kbase_tracepoints.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include "linux/mali_arbiter_interface.h"

/* Arbiter interface version against which was implemented this module */
#define MALI_REQUIRED_KBASE_ARBITER_INTERFACE_VERSION 5
#if MALI_REQUIRED_KBASE_ARBITER_INTERFACE_VERSION != MALI_ARBITER_INTERFACE_VERSION
#error "Unsupported Mali Arbiter interface version."
#endif

static void on_max_config(struct device *dev, uint32_t max_l2_slices, uint32_t max_core_mask)
{
	struct kbase_device *kbdev;

	if (!dev) {
		pr_err("%s(): dev is NULL", __func__);
		return;
	}

	kbdev = dev_get_drvdata(dev);
	if (!kbdev) {
		dev_err(dev, "%s(): kbdev is NULL", __func__);
		return;
	}

	if (!max_l2_slices || !max_core_mask) {
		dev_dbg(dev, "%s(): max_config ignored as one of the fields is zero", __func__);
		return;
	}

	/* set the max config info in the kbase device */
	kbase_arbiter_set_max_config(kbdev, max_l2_slices, max_core_mask);
}

/**
 * on_update_freq() - Updates GPU clock frequency
 * @dev: arbiter interface device handle
 * @freq: GPU clock frequency value reported from arbiter
 *
 * call back function to update GPU clock frequency with
 * new value from arbiter
 */
static void on_update_freq(struct device *dev, uint32_t freq)
{
	struct kbase_device *kbdev;

	if (!dev) {
		pr_err("%s(): dev is NULL", __func__);
		return;
	}

	kbdev = dev_get_drvdata(dev);
	if (!kbdev) {
		dev_err(dev, "%s(): kbdev is NULL", __func__);
		return;
	}

	kbase_arbiter_pm_update_gpu_freq(&kbdev->arb.arb_freq, freq);
}

/**
 * on_gpu_stop() - sends KBASE_VM_GPU_STOP_EVT event on VM stop
 * @dev: arbiter interface device handle
 *
 * call back function to signal a GPU STOP event from arbiter interface
 */
static void on_gpu_stop(struct device *dev)
{
	struct kbase_device *kbdev;

	if (!dev) {
		pr_err("%s(): dev is NULL", __func__);
		return;
	}

	kbdev = dev_get_drvdata(dev);
	if (!kbdev) {
		dev_err(dev, "%s(): kbdev is NULL", __func__);
		return;
	}

	KBASE_TLSTREAM_TL_ARBITER_STOP_REQUESTED(kbdev, kbdev);
	KBASE_KTRACE_ADD(kbdev, ARB_GPU_STOP_REQUESTED, NULL, 0);
	kbase_arbiter_pm_vm_event(kbdev, KBASE_VM_GPU_STOP_EVT);
}

/**
 * on_gpu_granted() - sends KBASE_VM_GPU_GRANTED_EVT event on GPU granted
 * @dev: arbiter interface device handle
 *
 * call back function to signal a GPU GRANT event from arbiter interface
 */
static void on_gpu_granted(struct device *dev)
{
	struct kbase_device *kbdev;

	if (!dev) {
		pr_err("%s(): dev is NULL", __func__);
		return;
	}

	kbdev = dev_get_drvdata(dev);
	if (!kbdev) {
		dev_err(dev, "%s(): kbdev is NULL", __func__);
		return;
	}

	KBASE_TLSTREAM_TL_ARBITER_GRANTED(kbdev, kbdev);
	KBASE_KTRACE_ADD(kbdev, ARB_GPU_GRANTED, NULL, 0);
	kbase_arbiter_pm_vm_event(kbdev, KBASE_VM_GPU_GRANTED_EVT);
}

/**
 * on_gpu_lost() - sends KBASE_VM_GPU_LOST_EVT event  on GPU granted
 * @dev: arbiter interface device handle
 *
 * call back function to signal a GPU LOST event from arbiter interface
 */
static void on_gpu_lost(struct device *dev)
{
	struct kbase_device *kbdev;

	if (!dev) {
		pr_err("%s(): dev is NULL", __func__);
		return;
	}

	kbdev = dev_get_drvdata(dev);
	if (!kbdev) {
		dev_err(dev, "%s(): kbdev is NULL", __func__);
		return;
	}
	KBASE_TLSTREAM_TL_ARBITER_LOST(kbdev, kbdev);
	KBASE_KTRACE_ADD(kbdev, ARB_GPU_LOST, NULL, 0);
	kbase_arbiter_pm_vm_event(kbdev, KBASE_VM_GPU_LOST_EVT);
}

static int kbase_arbif_of_init(struct kbase_device *kbdev)
{
	struct arbiter_if_dev *arb_if;
	struct device_node *arbiter_if_node;
	struct platform_device *pdev;

	if (!IS_ENABLED(CONFIG_OF)) {
		/*
		 * Return -ENODEV in the event CONFIG_OF is not available and let the
		 * internal AW check for suitability for arbitration.
		 */
		return -ENODEV;
	}

	arbiter_if_node = of_parse_phandle(kbdev->dev->of_node, "arbiter-if", 0);
	if (!arbiter_if_node)
		arbiter_if_node = of_parse_phandle(kbdev->dev->of_node, "arbiter_if", 0);
	if (!arbiter_if_node) {
		dev_dbg(kbdev->dev, "No arbiter_if in Device Tree");
		/* no arbiter interface defined in device tree */
		kbdev->arb.arb_dev = NULL;
		kbdev->arb.arb_if = NULL;
		return -ENODEV;
	}

	pdev = of_find_device_by_node(arbiter_if_node);
	if (!pdev) {
		dev_err(kbdev->dev, "Failed to find arbiter_if device");
		return -EPROBE_DEFER;
	}

	if (!pdev->dev.driver || !try_module_get(pdev->dev.driver->owner)) {
		dev_err(kbdev->dev, "arbiter_if driver not available");
		put_device(&pdev->dev);
		return -EPROBE_DEFER;
	}
	kbdev->arb.arb_dev = &pdev->dev;
	arb_if = platform_get_drvdata(pdev);
	if (!arb_if) {
		dev_err(kbdev->dev, "arbiter_if driver not ready");
		module_put(pdev->dev.driver->owner);
		put_device(&pdev->dev);
		return -EPROBE_DEFER;
	}

	kbdev->arb.arb_if = arb_if;
	return 0;
}

static void kbase_arbif_of_term(struct kbase_device *kbdev)
{
	if (!IS_ENABLED(CONFIG_OF))
		return;

	if (kbdev->arb.arb_dev) {
		module_put(kbdev->arb.arb_dev->driver->owner);
		put_device(kbdev->arb.arb_dev);
	}
	kbdev->arb.arb_dev = NULL;
}


/**
 * kbase_arbif_init() - Kbase Arbiter interface initialisation.
 * @kbdev: The kbase device structure for the device (must be a valid pointer)
 *
 * Initialise Kbase Arbiter interface and assign callback functions.
 *
 * Return:
 * * 0			- the interface was initialized or was not specified
 * *			in the device tree.
 * * -EFAULT		- the interface was specified but failed to initialize.
 * * -EPROBE_DEFER	- module dependencies are not yet available.
 */
int kbase_arbif_init(struct kbase_device *kbdev)
{
	struct arbiter_if_arb_vm_ops ops;
	struct arbiter_if_dev *arb_if;
	int err = 0;

	/* Tries to init with 'arbiter-if' if present in devicetree */
	err = kbase_arbif_of_init(kbdev);

	if (err == -ENODEV) {
		/* devicetree does not support arbitration */
		return -EPERM;
	}

	if (err)
		return err;

	ops.arb_vm_gpu_stop = on_gpu_stop;
	ops.arb_vm_gpu_granted = on_gpu_granted;
	ops.arb_vm_gpu_lost = on_gpu_lost;
	ops.arb_vm_max_config = on_max_config;
	ops.arb_vm_update_freq = on_update_freq;

	kbdev->arb.arb_freq.arb_freq = 0;
	kbdev->arb.arb_freq.freq_updated = false;
	mutex_init(&kbdev->arb.arb_freq.arb_freq_lock);

	arb_if = kbdev->arb.arb_if;

	if (arb_if == NULL) {
		dev_err(kbdev->dev, "No arbiter interface present");
		goto failure_term;
	}

	if (!arb_if->vm_ops.vm_arb_register_dev) {
		dev_err(kbdev->dev, "arbiter_if registration callback not present");
		goto failure_term;
	}

	/* register kbase arbiter_if callbacks */
	err = arb_if->vm_ops.vm_arb_register_dev(arb_if, kbdev->dev, &ops);
	if (err) {
		dev_err(kbdev->dev, "Failed to register with arbiter. (err = %d)", err);
		goto failure_term;
	}

	return 0;

failure_term:
	{
		kbase_arbif_of_term(kbdev);
	}

	if (err != -EPROBE_DEFER)
		err = -EFAULT;
	return err;
}

/**
 * kbase_arbif_destroy() - De-init Kbase arbiter interface
 * @kbdev: The kbase device structure for the device (must be a valid pointer)
 *
 * De-initialise Kbase arbiter interface
 */
void kbase_arbif_destroy(struct kbase_device *kbdev)
{
	struct arbiter_if_dev *arb_if = kbdev->arb.arb_if;

	if (arb_if && arb_if->vm_ops.vm_arb_unregister_dev)
		arb_if->vm_ops.vm_arb_unregister_dev(kbdev->arb.arb_if);

	{
		kbase_arbif_of_term(kbdev);
	}
	kbdev->arb.arb_if = NULL;
}

/**
 * kbase_arbif_get_max_config() - Request max config info
 * @kbdev: The kbase device structure for the device (must be a valid pointer)
 *
 * call back function from arb interface to arbiter requesting max config info
 */
void kbase_arbif_get_max_config(struct kbase_device *kbdev)
{
	struct arbiter_if_dev *arb_if = kbdev->arb.arb_if;

	if (arb_if && arb_if->vm_ops.vm_arb_get_max_config)
		arb_if->vm_ops.vm_arb_get_max_config(arb_if);
}

/**
 * kbase_arbif_gpu_request() - Request GPU from
 * @kbdev: The kbase device structure for the device (must be a valid pointer)
 *
 * call back function from arb interface to arbiter requesting GPU for VM
 */
void kbase_arbif_gpu_request(struct kbase_device *kbdev)
{
	struct arbiter_if_dev *arb_if = kbdev->arb.arb_if;

	if (arb_if && arb_if->vm_ops.vm_arb_gpu_request) {
		KBASE_TLSTREAM_TL_ARBITER_REQUESTED(kbdev, kbdev);
		KBASE_KTRACE_ADD(kbdev, ARB_GPU_REQUESTED, NULL, 0);
		arb_if->vm_ops.vm_arb_gpu_request(arb_if);
	}
}

/**
 * kbase_arbif_gpu_stopped() - send GPU stopped message to the arbiter
 * @kbdev: The kbase device structure for the device (must be a valid pointer)
 * @gpu_required: GPU request flag
 *
 */
void kbase_arbif_gpu_stopped(struct kbase_device *kbdev, u8 gpu_required)
{
	struct arbiter_if_dev *arb_if = kbdev->arb.arb_if;

	if (arb_if && arb_if->vm_ops.vm_arb_gpu_stopped) {
		KBASE_TLSTREAM_TL_ARBITER_STOPPED(kbdev, kbdev);
		KBASE_KTRACE_ADD(kbdev, ARB_GPU_STOPPED, NULL, 0);
		if (gpu_required) {
			KBASE_TLSTREAM_TL_ARBITER_REQUESTED(kbdev, kbdev);
			KBASE_KTRACE_ADD(kbdev, ARB_GPU_REQUESTED, NULL, 0);
		}
		arb_if->vm_ops.vm_arb_gpu_stopped(arb_if, gpu_required);
	}
}

/**
 * kbase_arbif_gpu_active() - Sends a GPU_ACTIVE message to the Arbiter
 * @kbdev: The kbase device structure for the device (must be a valid pointer)
 *
 * Informs the arbiter VM is active
 */
void kbase_arbif_gpu_active(struct kbase_device *kbdev)
{
	struct arbiter_if_dev *arb_if = kbdev->arb.arb_if;

	if (arb_if && arb_if->vm_ops.vm_arb_gpu_active)
		arb_if->vm_ops.vm_arb_gpu_active(arb_if);
}

/**
 * kbase_arbif_gpu_idle() - Inform the arbiter that the VM has gone idle
 * @kbdev: The kbase device structure for the device (must be a valid pointer)
 *
 * Informs the arbiter VM is idle
 */
void kbase_arbif_gpu_idle(struct kbase_device *kbdev)
{
	struct arbiter_if_dev *arb_if = kbdev->arb.arb_if;

	if (arb_if && arb_if->vm_ops.vm_arb_gpu_idle)
		arb_if->vm_ops.vm_arb_gpu_idle(arb_if);
}
