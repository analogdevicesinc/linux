// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 NXP
 */

#include <linux/dev_printk.h>
#include <linux/errno.h>
#include <linux/firmware/imx/ele_base_msg.h>
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
#define MAPPING_SIZE	0x20

enum soc_type {
	IMX8ULP,
	IMX93,
};

struct bank_2_reg {
	unsigned int bank;
	unsigned int reg;
	bool flag;
};

struct imx_fsb_s400_hw {
	enum soc_type soc;
	unsigned int fsb_otp_shadow;
	const struct bank_2_reg fsb_bank_reg[MAPPING_SIZE];
};

struct imx_fsb_s400_fuse {
	void __iomem *regs;
	struct nvmem_config config;
	struct mutex lock;
	const struct imx_fsb_s400_hw *hw;
};

static int read_words_via_s400_api(u32 *buf, unsigned int fuse_base, unsigned int num)
{
	unsigned int i;
	int err = 0;

	for (i = 0; i < num; i++)
		err = read_common_fuse(fuse_base + i, buf + i);

	return err;
}

static int read_words_via_fsb(void *priv, unsigned int bank, u32 *buf)
{
	struct imx_fsb_s400_fuse *fuse = priv;
	void __iomem *regs = fuse->regs + fuse->hw->fsb_otp_shadow;
	unsigned int i;
	unsigned int reg_id = UINT_MAX;
	unsigned int size = ARRAY_SIZE(fuse->hw->fsb_bank_reg);

	for (i = 0; i < size; i++) {
		if (fuse->hw->fsb_bank_reg[i].bank == bank) {
			reg_id = fuse->hw->fsb_bank_reg[i].reg;
			break;
		}
	}

	if (reg_id != UINT_MAX) {
		size = fuse->hw->fsb_bank_reg[i].flag ? 4 : 8;

		for (i = 0; i < size; i++) {
			*buf = readl_relaxed(regs + (reg_id + i) * 4);
			buf = buf + 1;
		}
	}

	return 0;
}

static int read_nwords_via_fsb(void __iomem *regs, u32 *buf, u32 fuse_base, u32 num)
{
	unsigned int i;

	for (i = 0; i < num; i++) {
		*buf = readl_relaxed(regs + (fuse_base + i) * 4);
		buf = buf + 1;
	}

	return 0;
}

static int fsb_s400_fuse_read(void *priv, unsigned int offset, void *val,
			      size_t bytes)
{
	struct imx_fsb_s400_fuse *fuse = priv;
	void __iomem *regs = fuse->regs + fuse->hw->fsb_otp_shadow;
	unsigned int num_bytes, bank;
	u32 *buf;
	int err;

	num_bytes = round_up(2048, 4);
	buf = kzalloc(num_bytes, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	err = -EINVAL;

	mutex_lock(&fuse->lock);
	if (fuse->hw->soc == IMX8ULP) {
		for (bank = 0; bank < 63; bank++) {
			switch (bank) {
			case 0:
				break;
			case LOCK_CFG:
				err = read_words_via_s400_api(&buf[8], 8, 8);
				if (err)
					goto ret;
				break;
			case ECID:
				err = read_words_via_s400_api(&buf[16], 16, 8);
				if (err)
					goto ret;
				break;
			case UNIQ_ID:
				err = read_common_fuse(OTP_UNIQ_ID, &buf[56]);
				if (err)
					goto ret;
				break;
			case OTFAD_CFG:
				err = read_common_fuse(OTFAD_CONFIG, &buf[184]);
				if (err)
					goto ret;
				break;
			case 25:
			case 26:
			case 27:
				err = read_words_via_s400_api(&buf[200], 200, 24);
				if (err)
					goto ret;
				break;
			case 32:
			case 33:
			case 34:
			case 35:
			case 36:
				err = read_words_via_s400_api(&buf[256], 256, 40);
				if (err)
					goto ret;
				break;
			case 49:
			case 50:
			case 51:
				err = read_words_via_s400_api(&buf[392], 392, 24);
				if (err)
					goto ret;
				break;
			default:
				err = read_words_via_fsb(priv, bank, &buf[bank * 8]);
				break;
			}
		}
	} else if (fuse->hw->soc == IMX93) {
		for (bank = 0; bank < 6; bank++)
			read_nwords_via_fsb(regs, &buf[bank * 8], bank * 8, 8);

		read_nwords_via_fsb(regs, &buf[48], 48, 4); /* OTP_UNIQ_ID */

		err = read_words_via_s400_api(&buf[63], 63, 1);
		if (err)
			goto ret;

		err = read_words_via_s400_api(&buf[128], 128, 16);
		if (err)
			goto ret;

		err = read_words_via_s400_api(&buf[182], 182, 1);
		if (err)
			goto ret;

		err = read_words_via_s400_api(&buf[188], 188, 1);
		if (err)
			goto ret;

		for (bank = 39; bank < 64; bank++)
			read_nwords_via_fsb(regs, &buf[bank * 8], bank * 8, 8);
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

	fuse = devm_kzalloc(&pdev->dev, sizeof(*fuse), GFP_KERNEL);
	if (!fuse)
		return -ENOMEM;

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
	mutex_init(&fuse->lock);
	fuse->hw = of_device_get_match_data(&pdev->dev);

	nvmem = devm_nvmem_register(&pdev->dev, &fuse->config);
	if (IS_ERR(nvmem)) {
		dev_err(&pdev->dev, "failed to register fuse nvmem device\n");
		return PTR_ERR(nvmem);
	}

	dev_dbg(&pdev->dev, "fuse nvmem device registered successfully\n");

	return 0;
}

static const struct imx_fsb_s400_hw imx8ulp_fsb_s400_hw = {
	.soc = IMX8ULP,
	.fsb_otp_shadow = 0x800,
	.fsb_bank_reg = {
		[0] = { 3, 0 },
		[1] = { 4, 8 },
		[2] = { 5, 16 },
		[3] = { 6, 24 },
		[4] = { 8, 80, true },
		[5] = { 24, 84, true },
		[6] = { 26, 88, true },
		[7] = { 27, 92, true },
		[8] = { 28, 96 },
		[9] = { 29, 104 },
		[10] = { 30, 112 },
		[11] = { 31, 120 },
		[12] = { 37, 128 },
		[13] = { 38, 136 },
		[14] = { 39, 144 },
		[15] = { 40, 152 },
		[16] = { 41, 160 },
		[17] = { 42, 168 },
		[18] = { 43, 176 },
		[19] = { 44, 184 },
		[20] = { 45, 192 },
		[21] = { 46, 200 },
	},
};

static const struct imx_fsb_s400_hw imx93_fsb_s400_hw = {
	.soc = IMX93,
	.fsb_otp_shadow = 0x8000,
};

static const struct of_device_id imx_fsb_s400_fuse_match[] = {
	{ .compatible = "fsl,imx8ulp-ocotp", .data = &imx8ulp_fsb_s400_hw, },
	{ .compatible = "fsl,imx93-ocotp", .data = &imx93_fsb_s400_hw, },
	{},
};

static struct platform_driver imx_fsb_s400_fuse_driver = {
	.driver = {
		.name = "fsl-ocotp-fsb-s400",
		.of_match_table = imx_fsb_s400_fuse_match,
	},
	.probe = imx_fsb_s400_fuse_probe,
};
MODULE_DEVICE_TABLE(of, imx_fsb_s400_fuse_match);
module_platform_driver(imx_fsb_s400_fuse_driver);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("i.MX FSB/S400-API ocotp fuse box driver");
MODULE_LICENSE("GPL v2");
