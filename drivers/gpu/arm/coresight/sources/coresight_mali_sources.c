// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2022-2024 ARM Limited. All rights reserved.
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

#include <linux/atomic.h>
#include <linux/coresight.h>
#include <linux/dma-mapping.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/of_platform.h>

#include <linux/mali_kbase_debug_coresight_csf.h>
#include <coresight-priv.h>
#include "sources/coresight_mali_sources.h"

#define CS_SCS_BASE_ADDR 0xE000E000
#define SCS_DEMCR 0xDFC

static struct kbase_debug_coresight_csf_address_range dwt_common_scs_range[] = {
	{ CS_SCS_BASE_ADDR, CS_SCS_BASE_ADDR + CORESIGHT_DEVTYPE }
};

/* For sources, pre enable and post disable sequences are
 * defined to manipulate DEMCR.TRECNA. Clearing of this register has
 * to be done as the last step of coresight disabling procedure and
 * only if there are no more configurations to disable. Otherwise,
 * if cleared earlier, TMC state machine gets stuck during Flush
 * procedure as clearing DEMCR.TRCENA stops ITM/ETM/ELA clocks.
 * Flush procedure never ends and TMC will stay in STOPPING state.
 */
static struct kbase_debug_coresight_csf_op dwt_common_pre_enable_ops[] = {
	// enable DWT functionality via DEMCR register
	WRITE_IMM_OP(CS_SCS_BASE_ADDR + SCS_DEMCR, 0x01000000),
};

static struct kbase_debug_coresight_csf_op dwt_common_post_disable_ops[] = {
	// disable DWT functionality via DEMCR register
	WRITE_IMM_OP(CS_SCS_BASE_ADDR + SCS_DEMCR, 0x00000000),
};

static int coresight_mali_source_trace_id(struct coresight_device *csdev)
{
	struct coresight_mali_source_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);

	return drvdata->trcid;
}

static int coresight_mali_enable_source(struct coresight_device *csdev, struct perf_event *event,
					u32 mode)
{
	return coresight_mali_enable_component(csdev, mode);
}

static void coresight_mali_disable_source(struct coresight_device *csdev, struct perf_event *event)
{
	coresight_mali_disable_component(csdev);
}

static const struct coresight_ops_source coresight_mali_source_ops = {
#if KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE
	.cpu_id = coresight_mali_source_trace_id,
#else
	.trace_id = coresight_mali_source_trace_id,
#endif
	.enable = coresight_mali_enable_source,
	.disable = coresight_mali_disable_source
};

static const struct coresight_ops mali_cs_ops = {
	.source_ops = &coresight_mali_source_ops,
};

int coresight_mali_sources_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct coresight_platform_data *pdata = NULL;
	struct coresight_mali_source_drvdata *drvdata = NULL;
	struct coresight_desc desc = { 0 };
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *gpu_pdev = NULL;
	struct device_node *gpu_node = NULL;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	dev_set_drvdata(dev, drvdata);
	drvdata->base.dev = dev;

#if KERNEL_VERSION(5, 3, 0) <= LINUX_VERSION_CODE
	pdata = coresight_get_platform_data(dev);
#else
	if (np)
		pdata = of_get_coresight_platform_data(dev, np);
#endif
	if (IS_ERR(pdata)) {
		dev_err(drvdata->base.dev, "Failed to get platform data");
		ret = PTR_ERR(pdata);
		goto devm_kfree_drvdata;
	}

	dev->platform_data = pdata;

	gpu_node = of_parse_phandle(np, "gpu", 0);
	if (!gpu_node) {
		dev_err(drvdata->base.dev, "GPU node not available");
		ret = -EINVAL;
		goto devm_kfree_drvdata;
	}
	gpu_pdev = of_find_device_by_node(gpu_node);
	if (!gpu_pdev) {
		dev_err(drvdata->base.dev, "Couldn't find GPU device from node");
		ret = -ENODEV;
		goto devm_kfree_drvdata;
	}

	drvdata->base.gpu_dev = platform_get_drvdata(gpu_pdev);
	if (!drvdata->base.gpu_dev) {
		dev_err(drvdata->base.dev, "GPU dev not available");
		ret = -ENODEV;
		goto devm_kfree_drvdata;
	}

	drvdata->base.kbase_pre_post_all_client = kbase_debug_coresight_csf_register(
		drvdata->base.gpu_dev, dwt_common_scs_range, ARRAY_SIZE(dwt_common_scs_range));
	if (!drvdata->base.kbase_pre_post_all_client) {
		dev_err(drvdata->base.dev, "Registration with access to SCS failed unexpectedly");
		ret = -EINVAL;
		goto devm_kfree_drvdata;
	}

	drvdata->base.pre_enable_seq.ops = dwt_common_pre_enable_ops;
	drvdata->base.pre_enable_seq.nr_ops = ARRAY_SIZE(dwt_common_pre_enable_ops);

	drvdata->base.post_disable_seq.ops = dwt_common_post_disable_ops;
	drvdata->base.post_disable_seq.nr_ops = ARRAY_SIZE(dwt_common_post_disable_ops);

	drvdata->base.pre_post_all_config = kbase_debug_coresight_csf_config_create(
		drvdata->base.kbase_pre_post_all_client, &drvdata->base.pre_enable_seq,
		&drvdata->base.post_disable_seq, true);
	if (!drvdata->base.pre_post_all_config) {
		dev_err(drvdata->base.dev, "pre_post_all_config create failed unexpectedly");
		ret = -EINVAL;
		goto kbase_pre_post_all_client_unregister;
	}

	ret = coresight_mali_sources_init_drvdata(drvdata);
	if (ret) {
		dev_err(drvdata->base.dev, "Failed to init source driver data");
		goto kbase_pre_post_all_config_unregister;
	}

	desc.type = CORESIGHT_DEV_TYPE_SOURCE;
	desc.subtype.source_subtype = CORESIGHT_DEV_SUBTYPE_SOURCE_SOFTWARE;
	desc.ops = &mali_cs_ops;
	desc.pdata = pdata;
	desc.dev = dev;
	desc.groups = coresight_mali_source_groups_get();

#if KERNEL_VERSION(5, 3, 0) <= LINUX_VERSION_CODE
	desc.name = devm_kasprintf(dev, GFP_KERNEL, "%s", drvdata->type_name);
	if (!desc.name) {
		ret = -ENOMEM;
		goto source_deinit_drvdata;
	}
#endif

	drvdata->base.csdev = coresight_register(&desc);
	if (IS_ERR(drvdata->base.csdev)) {
		dev_err(drvdata->base.dev, "Failed to register coresight device\n");
		ret = PTR_ERR(drvdata->base.csdev);
		goto source_deinit_drvdata;
	}

	return ret;

source_deinit_drvdata:
	coresight_mali_sources_deinit_drvdata(drvdata);

kbase_pre_post_all_config_unregister:
	kbase_debug_coresight_csf_config_free(drvdata->base.pre_post_all_config);

kbase_pre_post_all_client_unregister:
	kbase_debug_coresight_csf_unregister(drvdata->base.kbase_pre_post_all_client);

devm_kfree_drvdata:
	devm_kfree(dev, drvdata);

	return ret;
}

#if (KERNEL_VERSION(6, 11, 0) > LINUX_VERSION_CODE)
int coresight_mali_sources_remove(struct platform_device *pdev)
#else
void coresight_mali_sources_remove(struct platform_device *pdev)
#endif
{
	struct coresight_mali_source_drvdata *drvdata = dev_get_drvdata(&pdev->dev);

	if (drvdata->base.csdev != NULL)
		coresight_unregister(drvdata->base.csdev);

	coresight_mali_sources_deinit_drvdata(drvdata);

	if (drvdata->base.pre_post_all_config != NULL)
		kbase_debug_coresight_csf_config_free(drvdata->base.pre_post_all_config);

	if (drvdata->base.kbase_pre_post_all_client != NULL)
		kbase_debug_coresight_csf_unregister(drvdata->base.kbase_pre_post_all_client);

	devm_kfree(&pdev->dev, drvdata);

#if (KERNEL_VERSION(6, 11, 0) > LINUX_VERSION_CODE)
	return 0;
#endif
}

MODULE_AUTHOR("ARM Ltd.");
MODULE_DESCRIPTION("Arm Coresight Mali source");
MODULE_LICENSE("GPL");
