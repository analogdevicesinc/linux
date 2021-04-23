// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 NXP
 * Author: Alice Guo <alice.guo@nxp.com>
 */

#include <linux/dev_printk.h>
#include <linux/errno.h>
#include <linux/firmware/imx/s400-api.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/nvmem-provider.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define LOCK_CFG	0x01
#define ECID		0x02
#define UNIQ_ID		0x07
#define OTFAD_CFG	0x17

struct bank_2_reg {
	unsigned int bank;
	unsigned int reg;
	bool flag;
};

static const struct bank_2_reg fsb_bank_reg[] = {
	{ 3, 0 },
	{ 4, 8 },
	{ 5, 16 },
	{ 6, 24 },
	{ 8, 80, true },
	{ 24, 84, true },
	{ 26, 88, true },
	{ 27, 92, true },
	{ 28, 96 },
	{ 29, 104 },
	{ 30, 112 },
	{ 31, 120 },
	{ 37, 128 },
	{ 38, 136 },
	{ 39, 144 },
	{ 40, 152 },
	{ 41, 160 },
	{ 42, 168 },
	{ 43, 176 },
	{ 44, 184 },
	{ 45, 192 },
	{ 46, 200 },
};

struct imx_fsb_s400_fuse {
	void __iomem *regs;
	struct imx_s400_api *s400_api;
	struct nvmem_config config;
	struct mutex lock;
};

static int read_words_via_s400_api(struct imx_s400_api *s400_api, u32 *buf,
				   unsigned int fuse_base)
{
	unsigned int i;
	int err = 0;

	s400_api->tx_msg.header = 0x17970206;

	for (i = 0; i < 8; i++) {
		s400_api->tx_msg.data[0] = fuse_base + i;
		err = imx_s400_api_call(s400_api, buf + i);
	}

	return err;
}

static int read_words_via_fsb(void __iomem *regs, unsigned int bank, u32 *buf)
{
	unsigned int i;
	unsigned int reg_id = UINT_MAX;
	unsigned int size = ARRAY_SIZE(fsb_bank_reg);

	for (i = 0; i < size; i++) {
		if (fsb_bank_reg[i].bank == bank) {
			reg_id = fsb_bank_reg[i].reg;
			break;
		}
	}

	if (reg_id != UINT_MAX) {
		size = fsb_bank_reg[i].flag ? 4 : 8;

		for (i = 0; i < size; i++) {
			*buf = readl_relaxed(regs + (reg_id + i) * 4);
			buf = buf + 1;
		}
	}

	return 0;
}

static int fsb_s400_fuse_read(void *priv, unsigned int offset, void *val,
			      size_t bytes)
{
	struct imx_fsb_s400_fuse *fuse = priv;
	struct imx_s400_api *s400_api = fuse->s400_api;
	unsigned int num_bytes, bank;
	u32 *buf;
	int err;

	num_bytes = round_up(2048, 4);
	buf = kzalloc(num_bytes, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	err = -EINVAL;

	mutex_lock(&fuse->lock);
	for (bank = 0; bank < 63; bank++) {
		switch (bank) {
		case 0:
			break;
		case LOCK_CFG:
			err = read_words_via_s400_api(s400_api, &buf[8], 8);
			if (err)
				goto ret;
			break;
		case ECID:
			err = read_words_via_s400_api(s400_api, &buf[16], 16);
			if (err)
				goto ret;
			break;
		case UNIQ_ID:
			fuse->s400_api->tx_msg.header = 0x17970206;
			fuse->s400_api->tx_msg.data[0] = 0x1;

			err = imx_s400_api_call(s400_api, &buf[56]);
			if (err)
				goto ret;
			break;
		case OTFAD_CFG:
			fuse->s400_api->tx_msg.header = 0x17970206;
			fuse->s400_api->tx_msg.data[0] = 0x2;

			err = imx_s400_api_call(s400_api, &buf[184]);
			if (err)
				goto ret;
			break;
		default:
			err = read_words_via_fsb(fuse->regs + 0x800, bank, &buf[bank * 8]);
			break;
		}
	}

	memcpy(val, (u8 *)(buf + offset), bytes);

ret:
	kfree(buf);
	mutex_unlock(&fuse->lock);

	return err;
}

static int imx_fsb_s400_fuse_probe(struct platform_device *pdev)
{
	struct imx_fsb_s400_fuse *fuse;
	struct nvmem_device *nvmem;
	int err;

	fuse = devm_kzalloc(&pdev->dev, sizeof(*fuse), GFP_KERNEL);
	if (!fuse)
		return -ENOMEM;

	err = get_imx_s400_api(&fuse->s400_api);
	if (err) {
		dev_err(&pdev->dev, "failed to get imx s400 api: %d\n", err);
		return err;
	}

	fuse->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fuse->regs))
		return PTR_ERR(fuse->regs);

	fuse->config.dev = &pdev->dev;
	fuse->config.name = "fsb_s400_fuse";
	fuse->config.id = NVMEM_DEVID_AUTO;
	fuse->config.owner = THIS_MODULE;
	fuse->config.size = 2048; /* 64 Banks */
	fuse->config.reg_read = fsb_s400_fuse_read;
	fuse->config.priv = fuse;

	nvmem = devm_nvmem_register(&pdev->dev, &fuse->config);
	if (IS_ERR(nvmem)) {
		dev_err(&pdev->dev, "failed to register fuse nvmem device\n");
		return PTR_ERR(nvmem);
	}

	mutex_init(&fuse->lock);

	dev_dbg(&pdev->dev, "fuse nvmem device registered successfully\n");

	return 0;
}

static const struct of_device_id imx_fsb_s400_fuse_match[] = {
	{ .compatible = "fsl,imx8ulp-ocotp", },
	{},
};

static struct platform_driver imx_fsb_s400_fuse_driver = {
	.driver = {
		.name = "fsl-ocotp-fsb-s400",
		.of_match_table = imx_fsb_s400_fuse_match,
	},
	.probe = imx_fsb_s400_fuse_probe,
};
module_platform_driver(imx_fsb_s400_fuse_driver);

MODULE_AUTHOR("Alice Guo <alice.guo@nxp.com>");
MODULE_DESCRIPTION("i.MX FSB/S400-API ocotp fuse box driver");
MODULE_LICENSE("GPL v2");
