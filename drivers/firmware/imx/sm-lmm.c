// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 NXP
 */

#include <linux/firmware/imx/sm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/scmi_protocol.h>
#include <linux/scmi_imx_protocol.h>

static const struct scmi_imx_lmm_proto_ops *imx_lmm_ops;
static struct scmi_protocol_handle *ph;

int scmi_imx_lmm_info(u32 lmid, struct scmi_imx_lmm_info *info)
{
	/* Pairs with the smp_store_release() in the probe function. */
	if (!smp_load_acquire(&ph))
		return -EPROBE_DEFER;

	if (!info)
		return -EINVAL;

	return imx_lmm_ops->lmm_info(ph, lmid, info);
};
EXPORT_SYMBOL(scmi_imx_lmm_info);

int scmi_imx_lmm_reset_vector_set(u32 lmid, u32 cpuid, u32 flags, u64 vector)
{
	/* Pairs with the smp_store_release() in the probe function. */
	if (!smp_load_acquire(&ph))
		return -EPROBE_DEFER;

	return imx_lmm_ops->lmm_reset_vector_set(ph, lmid, cpuid, flags, vector);
}
EXPORT_SYMBOL(scmi_imx_lmm_reset_vector_set);

int scmi_imx_lmm_operation(u32 lmid, enum scmi_imx_lmm_op op, u32 flags)
{
	/* Pairs with the smp_store_release() in the probe function. */
	if (!smp_load_acquire(&ph))
		return -EPROBE_DEFER;

	switch (op) {
	case SCMI_IMX_LMM_BOOT:
		return imx_lmm_ops->lmm_power_boot(ph, lmid, true);
	case SCMI_IMX_LMM_POWER_ON:
		return imx_lmm_ops->lmm_power_boot(ph, lmid, false);
	case SCMI_IMX_LMM_SHUTDOWN:
		return imx_lmm_ops->lmm_shutdown(ph, lmid, flags);
	default:
		break;
	}

	return -EINVAL;
}
EXPORT_SYMBOL(scmi_imx_lmm_operation);

static int scmi_imx_lmm_probe(struct scmi_device *sdev)
{
	const struct scmi_handle *handle = sdev->handle;
	const struct scmi_imx_lmm_proto_ops *ops;
	struct scmi_protocol_handle *lmm_ph;

	if (!handle)
		return -ENODEV;

	if (imx_lmm_ops) {
		dev_err(&sdev->dev, "lmm already initialized\n");
		return -EEXIST;
	}

	ops = handle->devm_protocol_get(sdev, SCMI_PROTOCOL_IMX_LMM, &lmm_ph);
	if (IS_ERR(ops))
		return PTR_ERR(ops);

	imx_lmm_ops = ops;
	/*
	 * ph is the gate the exported helpers test. Publish it only after
	 * imx_lmm_ops is set, and pair with the smp_load_acquire() there.
	 */
	smp_store_release(&ph, lmm_ph);

	return 0;
}

static const struct scmi_device_id scmi_id_table[] = {
	{ SCMI_PROTOCOL_IMX_LMM, "imx-lmm" },
	{ },
};
MODULE_DEVICE_TABLE(scmi, scmi_id_table);

static struct scmi_driver scmi_imx_lmm_driver = {
	.name = "scmi-imx-lmm",
	.probe = scmi_imx_lmm_probe,
	.id_table = scmi_id_table,
};
module_scmi_driver(scmi_imx_lmm_driver);

MODULE_AUTHOR("Peng Fan <peng.fan@nxp.com>");
MODULE_DESCRIPTION("IMX SM LMM driver");
MODULE_LICENSE("GPL");
