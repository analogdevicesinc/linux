// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2018-2025, Advanced Micro Devices, Inc. */

#include <linux/kernel.h>
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

static const struct ionic_aux_type_info ionic_aux_types[IONIC_AUX_MAX] = {
	[IONIC_AUX_RDMA] = {
		.name = "rdma",
		.supported = ionic_rdma_supported,
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
