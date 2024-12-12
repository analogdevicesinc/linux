// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 NXP
 *
 * File containing client-side RPC functions for the SECO service. These
 * function are ported to clients that communicate to the SC.
 */

#include <linux/firmware/imx/ipc.h>
#include <linux/firmware/imx/se_api.h>
#include <linux/firmware/imx/svc/rm.h>
#include <linux/nvmem-consumer.h>
#include <linux/io.h>
#include <linux/sys_soc.h>
#include <uapi/linux/se_ioctl.h>

#include "seco_init.h"

static const struct soc_device_attribute soc_info_matches[] = {
	{ .soc_id = "i.MX8DXL",
	  .revision = "1.1",
	  .data = &(struct seco_soc_info) {.soc_id = SOC_ID_OF_IMX8DXL, .soc_rev = SOC_REV_A1,}
	},
	{ .soc_id = "i.MX8DXL",
	  .revision = "1.2",
	  .data = &(struct seco_soc_info) {.soc_id = SOC_ID_OF_IMX8DXL, .soc_rev = SOC_REV_B0,}
	},
	{ .soc_id = "i.MX8QXP",
	  .revision = "1.1",
	  .data = &(struct seco_soc_info) {.soc_id = SOC_ID_OF_IMX8QXP, .soc_rev = SOC_REV_B0,}
	},
	{ .soc_id = "i.MX8QXP",
	  .revision = "1.2",
	  .data = &(struct seco_soc_info) {.soc_id = SOC_ID_OF_IMX8QXP, .soc_rev = SOC_REV_C0,}
	},
	{ .soc_id = "i.MX8QM",
	  .revision = "1.1",
	  .data = &(struct seco_soc_info) {.soc_id = SOC_ID_OF_IMX8QM, .soc_rev = SOC_REV_B0,}
	},
	{ .soc_id = "i.MX8QM",
	  .revision = "1.2",
	  .data = &(struct seco_soc_info) {.soc_id = SOC_ID_OF_IMX8QM, .soc_rev = SOC_REV_C0,}
	},
	{ }
};

static int read_fuse(struct se_if_priv *priv, u32 id, u32 *value, u8 mul)
{
	struct nvmem_device *nvmem;
	u32 size_to_read = mul * sizeof(u32);
	struct device_node *np;
	int ret = 0;

	np = priv->dev->of_node;
	if (!of_get_property(np, "nvmem", NULL))
		return -EPERM;

	nvmem = devm_nvmem_device_get(priv->dev, NULL);
	if (IS_ERR(nvmem)) {
		ret = PTR_ERR(nvmem);

		if (ret != -EPROBE_DEFER)
			dev_err(priv->dev, "Failed to retrieve nvmem node\n");

		return ret;
	}

	ret = nvmem_device_read(nvmem, id, size_to_read, value);
	if (ret < 0) {
		dev_err(priv->dev, "Failed to read fuse %d: %d\n", id, ret);
		return ret;
	}

	if (ret != size_to_read) {
		dev_err(priv->dev, "Read only %d instead of %d\n", ret,
			size_to_read);
		return -ENOMEM;
	}

	dev_dbg(priv->dev, "FUSE value 0x%x 0x%x 0x%x 0x%x\n",
		value[0], value[1],
		value[2], value[3]);

	devm_nvmem_device_put(priv->dev, nvmem);

	return 0;
}

/* function to fetch SoC revision on the SECO platform */
int seco_fetch_soc_info(struct se_if_priv *priv, void *data)
{
	struct seco_soc_info *soc_data = data;
	const struct soc_device_attribute *imx_soc_match;
	u32 fuse_val[4] = {0xFF};
	int err = 0;

	imx_soc_match = soc_device_match(soc_info_matches);
	if (!imx_soc_match || !imx_soc_match->data) {
		err = -EINVAL;
		goto exit;
	}
	err = read_fuse(priv, 8, (u32 *)&fuse_val, 4);
	if (err) {
		dev_err(priv->dev, "Fail to read FIPS fuse\n");
		return err;
	}

	if (fuse_val[2] & SECO_NON_FIPS)
		soc_data->board_type = IMX8DXL_DL1;
	else if (fuse_val[0] & V2X_NON_FIPS)
		soc_data->board_type = IMX8DXL_DL3;
	else if (!fuse_val[0] && !fuse_val[2])
		soc_data->board_type = IMX8DXL_DL2;

	soc_data->soc_id = ((const struct seco_soc_info *)imx_soc_match->data)->soc_id;
	soc_data->soc_rev = ((const struct seco_soc_info *)imx_soc_match->data)->soc_rev;

exit:
	return err;
}

int imx_scu_init_fw(struct se_if_priv *priv)
{
	int ret;

	if (!priv) {
		ret = -EINVAL;
		goto exit;
	}

	ret = imx_scu_get_handle(&priv->ipc_scu);
	if (ret) {
		dev_err(priv->dev, "Fail to retrieve IPC handle\n");
		goto exit;
	}

	ret = imx_sc_rm_get_resource_owner(priv->ipc_scu, IMX_SC_R_SECO, &priv->part_owner);
	if (ret) {
		dev_err(priv->dev, "Fail get owner of SECO resource\n");
		goto exit;
	}

	if (!ret) {
		priv->flags |= SCU_MEM_CFG;

		if (get_se_soc_id(priv) == SOC_ID_OF_IMX8DXL &&
				(priv->if_defs->se_if_type == SE_TYPE_ID_SHE ||
				priv->if_defs->se_if_type == SE_TYPE_ID_HSM))
			priv->flags |= SCU_SIGNED_MSG_CFG;
	}
exit:
	return ret;
}

int imx_scu_sec_mem_cfg(struct file *fp, uint32_t offset, uint32_t size)
{
	struct se_if_device_ctx *dev_ctx = fp->private_data;
	u64 high_boundary;
	int ret = 0;

	high_boundary = offset;
	if (high_boundary > SECURE_RAM_SIZE) {
		dev_err(dev_ctx->priv->dev, "base offset is over secure memory\n");
		return -ENOMEM;
	}

	high_boundary += size;
	if (high_boundary > SECURE_RAM_SIZE) {
		dev_err(dev_ctx->priv->dev, "total memory is over secure memory\n");
		return -ENOMEM;
	}

	dev_ctx->se_shared_mem_mgmt.secure_mem.dma_addr = (dma_addr_t)offset;
	dev_ctx->se_shared_mem_mgmt.secure_mem.size = size;
	dev_ctx->se_shared_mem_mgmt.secure_mem.pos = 0;
	dev_ctx->se_shared_mem_mgmt.secure_mem.ptr = devm_ioremap(dev_ctx->priv->dev,
					      (phys_addr_t)(SECURE_RAM_BASE_ADDRESS +
					      (u64)dev_ctx->se_shared_mem_mgmt.secure_mem.dma_addr),
					      dev_ctx->se_shared_mem_mgmt.secure_mem.size);
	if (!dev_ctx->se_shared_mem_mgmt.secure_mem.ptr) {
		dev_err(dev_ctx->priv->dev, "Failed to map secure memory\n");
		return -ENOMEM;
	}

	return ret;
}

int imx_scu_mem_access(struct se_if_device_ctx *dev_ctx)
{
	struct se_if_priv *priv = dev_ctx->priv;
	u8 mr;
	u64 addr;
	int ret;

	addr = dev_ctx->se_shared_mem_mgmt.non_secure_mem.dma_addr;

	ret = imx_sc_rm_find_memreg(priv->ipc_scu,
				    &mr,
				    addr,
				    addr + MAX_DATA_SIZE_PER_USER);
	if (ret) {
		dev_err(dev_ctx->priv->dev,
			"%s: Fail find memreg\n", dev_ctx->devname);
		return ret;
	}

	ret = imx_sc_rm_set_memreg_permissions(priv->ipc_scu, mr,
					       priv->part_owner,
					       IMX_SC_RM_PERM_FULL);
	if (ret) {
		dev_err(dev_ctx->priv->dev,
			"%s: Fail set permission for resource\n",
				dev_ctx->devname);
		return ret;
	}

	return ret;
}

int imx_scu_signed_msg(struct file *fp,
		       uint8_t *msg,
		       uint32_t size,
		       uint32_t *error)
{
	struct se_if_device_ctx *dev_ctx = fp->private_data;
	struct se_if_priv *priv = dev_ctx->priv;
	struct se_shared_mem *shared_mem
		= &dev_ctx->se_shared_mem_mgmt.non_secure_mem;
	int err;
	u64 addr;
	u32 pos;

	/* Check there is enough space in the shared memory. */
	if (size >= shared_mem->size - shared_mem->pos) {
		dev_err(dev_ctx->priv->dev,
			"Not enough mem: %d left, %d required\n",
			shared_mem->size - shared_mem->pos, size);
		return -ENOMEM;
	}

	/* Allocate space in shared memory. 8 bytes aligned. */
	pos = shared_mem->pos;

	/* get physical address from the pos */
	addr = (u64)shared_mem->dma_addr + pos;

	/* copy signed message from user space to this allocated buffer */
	err = copy_from_user(shared_mem->ptr + pos, msg, size);
	if (err) {
		dev_err(dev_ctx->priv->dev,
			"Failed to signed message from user: %d\n",
			err);
		return -EFAULT;
	}

	*error = imx_sc_seco_sab_msg(priv->ipc_scu, addr);
	err = *error;
	if (err)
		dev_err(dev_ctx->priv->dev, "Failt to send signed message\n");

	return err;
}
