// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2018-2025, Advanced Micro Devices, Inc. */

#include <linux/kernel.h>
#include <linux/ionic/ionic_hwmon.h>

#include "ionic.h"
#include "ionic_lif.h"
#include "ionic_aux.h"

struct ionic_aux_type_info {
	const char *name;
	bool (*supported)(struct ionic *ionic);
	int (*prepare)(struct ionic *ionic);
};

static bool ionic_rdma_supported(struct ionic *ionic)
{
	return le64_to_cpu(ionic->ident.lif.capabilities) & IONIC_LIF_CAP_RDMA;
}

static bool ionic_hwmon_supported(struct ionic *ionic)
{
	return le64_to_cpu(ionic->ident.dev.capabilities) & IONIC_DEV_CAP_HWMON;
}

static int ionic_hwmon_prepare(struct ionic *ionic)
{
	void *buf;

	if (ionic->hwmon_caps_buf)
		return 0;

	buf = dmam_alloc_coherent(ionic->dev, IONIC_HWMON_CAPS_BUF_SIZE,
				  &ionic->hwmon_caps_pa, GFP_KERNEL);
	if (!buf) {
		dev_warn(ionic->dev, "Failed to allocate hwmon buffer\n");
		return -ENOMEM;
	}

	ionic->hwmon_caps_buf = buf;
	return 0;
}

static const struct ionic_aux_type_info ionic_aux_types[IONIC_AUX_MAX] = {
	[IONIC_AUX_RDMA] = {
		.name = "rdma",
		.supported = ionic_rdma_supported,
	},
	[IONIC_AUX_HWMON] = {
		.name = "hwmon",
		.supported = ionic_hwmon_supported,
		.prepare = ionic_hwmon_prepare,
	},
};

static void ionic_auxbus_release(struct device *dev)
{
	struct ionic_aux_dev *ionic_adev;

	ionic_adev = container_of(dev, struct ionic_aux_dev, adev.dev);
	kfree(ionic_adev);
}

static struct ionic_aux_dev *ionic_auxbus_add(struct ionic_lif *lif,
					      const char *name)
{
	struct ionic_aux_dev *ionic_adev;
	struct auxiliary_device *aux_dev;
	int err;

	ionic_adev = kzalloc_obj(*ionic_adev);
	if (!ionic_adev)
		return ERR_PTR(-ENOMEM);

	aux_dev = &ionic_adev->adev;

	aux_dev->id = lif->ionic->aux_id;
	aux_dev->name = name;
	aux_dev->dev.parent = &lif->ionic->pdev->dev;
	aux_dev->dev.release = ionic_auxbus_release;
	ionic_adev->lif = lif;
	err = auxiliary_device_init(aux_dev);
	if (err) {
		dev_err(lif->ionic->dev, "Failed to initialize %s aux device: %d\n",
			aux_dev->name, err);
		kfree(ionic_adev);
		return ERR_PTR(err);
	}

	err = auxiliary_device_add(aux_dev);
	if (err) {
		dev_err(lif->ionic->dev, "Failed to add %s aux device: %d\n",
			aux_dev->name, err);
		auxiliary_device_uninit(aux_dev);
		return ERR_PTR(err);
	}

	return ionic_adev;
}

static void ionic_auxbus_del(struct ionic_aux_dev *ionic_adev)
{
	auxiliary_device_delete(&ionic_adev->adev);
	auxiliary_device_uninit(&ionic_adev->adev);
}

void ionic_auxbus_register(struct ionic_lif *lif)
{
	const struct ionic_aux_type_info *type;
	struct ionic_aux_dev *ionic_adev;
	int i;

	mutex_lock(&lif->adev_lock);

	for (i = 0; i < IONIC_AUX_MAX; i++) {
		type = &ionic_aux_types[i];

		if (!type->supported(lif->ionic))
			continue;

		if (type->prepare && type->prepare(lif->ionic))
			continue;

		ionic_adev = ionic_auxbus_add(lif, type->name);
		if (!IS_ERR(ionic_adev))
			lif->aux_devs[i] = ionic_adev;
	}

	mutex_unlock(&lif->adev_lock);
}

void ionic_auxbus_unregister(struct ionic_lif *lif)
{
	int i;

	mutex_lock(&lif->adev_lock);

	for (i = IONIC_AUX_MAX - 1; i >= 0; i--) {
		if (!lif->aux_devs[i])
			continue;

		ionic_auxbus_del(lif->aux_devs[i]);
		lif->aux_devs[i] = NULL;
	}

	mutex_unlock(&lif->adev_lock);
}

static struct ionic_lif *ionic_hwmon_lif(struct auxiliary_device *adev)
{
	return container_of(adev, struct ionic_aux_dev, adev)->lif;
}

int ionic_hwmon_get_caps(struct auxiliary_device *adev, u8 max_ver,
			 const void **buf, size_t *len, u8 *ver)
{
	struct ionic_lif *lif = ionic_hwmon_lif(adev);
	struct ionic *ionic = lif->ionic;
	struct ionic_admin_ctx ctx = {
		.work = COMPLETION_INITIALIZER_ONSTACK(ctx.work),
		.comp.comp.status = IONIC_RC_EIO,
		.cmd.hwmon_getcaps = {
			.opcode = IONIC_CMD_HWMON_GETCAPS,
			.ver = max_ver,
			.lif_index = cpu_to_le16(lif->index),
			.resp_buf_len = cpu_to_le32(IONIC_HWMON_CAPS_BUF_SIZE),
			.resp_dma_addr = cpu_to_le64(ionic->hwmon_caps_pa),
		},
	};
	int err;

	if (!ionic->hwmon_caps_buf)
		return -ENODEV;

	memset(ionic->hwmon_caps_buf, 0, IONIC_HWMON_CAPS_BUF_SIZE);

	err = ionic_adminq_post_wait_nomsg(lif, &ctx);
	if (err)
		return err;

	if (ctx.comp.comp.status)
		return ionic_error_to_errno(ctx.comp.comp.status);

	*buf = ionic->hwmon_caps_buf;
	*len = IONIC_HWMON_CAPS_BUF_SIZE;
	*ver = ctx.comp.hwmon_getcaps.ver;

	return 0;
}
EXPORT_SYMBOL_NS_GPL(ionic_hwmon_get_caps, "NET_IONIC");

int ionic_hwmon_get_attr(struct auxiliary_device *adev, u8 type, u8 channel,
			 u8 attr, u32 *val)
{
	struct ionic_lif *lif = ionic_hwmon_lif(adev);
	struct ionic_admin_ctx ctx = {
		.work = COMPLETION_INITIALIZER_ONSTACK(ctx.work),
		.comp.comp.status = IONIC_RC_EIO,
		.cmd.hwmon_getattr = {
			.opcode = IONIC_CMD_HWMON_GETATTR,
			.lif_index = cpu_to_le16(lif->index),
			.sensor_type = type,
			.channel = channel,
			.attr = attr,
		},
	};
	int err;

	err = ionic_adminq_post_wait_nomsg(lif, &ctx);
	if (err)
		return err;

	if (ctx.comp.comp.status)
		return ionic_error_to_errno(ctx.comp.comp.status);

	*val = le32_to_cpu(ctx.comp.hwmon_getattr.attr_value);

	return 0;
}
EXPORT_SYMBOL_NS_GPL(ionic_hwmon_get_attr, "NET_IONIC");

struct device *ionic_hwmon_dev(struct auxiliary_device *adev)
{
	return ionic_hwmon_lif(adev)->ionic->dev;
}
EXPORT_SYMBOL_NS_GPL(ionic_hwmon_dev, "NET_IONIC");

void ionic_request_rdma_reset(struct ionic_lif *lif)
{
	struct ionic *ionic = lif->ionic;
	int err;

	union ionic_dev_cmd cmd = {
		.cmd.opcode = IONIC_CMD_RDMA_RESET_LIF,
		.cmd.lif_index = cpu_to_le16(lif->index),
	};

	mutex_lock(&ionic->dev_cmd_lock);

	ionic_dev_cmd_go(&ionic->idev, &cmd);
	err = ionic_dev_cmd_wait(ionic, DEVCMD_TIMEOUT);

	mutex_unlock(&ionic->dev_cmd_lock);

	if (err)
		pr_warn("%s request_reset: error %d\n", __func__, err);
}
EXPORT_SYMBOL_NS(ionic_request_rdma_reset, "NET_IONIC");
