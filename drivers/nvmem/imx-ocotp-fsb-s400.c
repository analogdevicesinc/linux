// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021, 2024 NXP
 */

#include <linux/of_address.h>
#include <linux/dev_printk.h>
#include <linux/errno.h>
#include <linux/firmware/imx/se_api.h>
#include <linux/io.h>
#include <linux/etherdevice.h>
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
#define FUSE_ACC_DIS	0x28

enum soc_type {
	IMX8ULP,
	IMX93,
	IMX95,
};

struct bank_2_reg {
	unsigned int bank;
	unsigned int reg;
	bool flag;
};

struct imx_fsb_s400_hw {
	enum soc_type soc;
	unsigned int fsb_otp_shadow;
	uint32_t se_soc_id;
	const struct bank_2_reg fsb_bank_reg[MAPPING_SIZE];
	bool oscca_fuse_read;
	bool reverse_mac_address;
	bool increase_mac_address;
	const u8 *pf_mac_offset_list;
};

struct imx_fsb_s400_fuse {
	void __iomem *regs;
	struct nvmem_config config;
	struct mutex lock;
	void *se_data;
	const struct imx_fsb_s400_hw *hw;
	bool fsb_read_dis;
	u8 pfn;
};

static int read_words_via_s400_api(u32 *buf, unsigned int fuse_base,
				   unsigned int num, void *se_data)
{
	unsigned int i;
	int err = 0;

	for (i = 0; i < num; i++)
		err = imx_se_read_fuse(se_data, fuse_base + i, buf + i);

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
	int err, i;

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
				err = read_words_via_s400_api(&buf[8], 8, 8,
							      fuse->se_data);
				if (err)
					goto ret;
				break;
			case ECID:
				err = read_words_via_s400_api(&buf[16], 16, 8,
							      fuse->se_data);
				if (err)
					goto ret;
				break;
			case UNIQ_ID:
				err = imx_se_read_fuse(fuse->se_data,
						       OTP_UNIQ_ID,
						       &buf[56]);
				if (err)
					goto ret;
				break;
			case OTFAD_CFG:
				err = imx_se_read_fuse(fuse->se_data,
						       OTFAD_CONFIG, &buf[184]);
				if (err)
					goto ret;
				break;
			case 25:
			case 26:
			case 27:
				err = read_words_via_s400_api(&buf[200], 200, 24,
							      fuse->se_data);
				if (err)
					goto ret;
				break;
			case 32:
			case 33:
			case 34:
			case 35:
			case 36:
				err = read_words_via_s400_api(&buf[256], 256, 40,
							      fuse->se_data);
				if (err)
					goto ret;
				break;
			case 49:
			case 50:
			case 51:
				err = read_words_via_s400_api(&buf[392], 392, 24,
							      fuse->se_data);
				if (err)
					goto ret;
				break;
			default:
				err = read_words_via_fsb(priv, bank, &buf[bank * 8]);
				break;
			}
		}
	} else if (fuse->hw->soc == IMX93) {
		for (bank = 0; bank < 6; bank++) {
			if (fuse->fsb_read_dis)
				read_words_via_s400_api(&buf[bank * 8], bank * 8,
							8, fuse->se_data);
			else
				read_nwords_via_fsb(regs, &buf[bank * 8], bank * 8, 8);
		}

		if (fuse->fsb_read_dis)
			read_words_via_s400_api(&buf[48], 48, 4, fuse->se_data);
		else
			read_nwords_via_fsb(regs, &buf[48], 48, 4); /* OTP_UNIQ_ID */

		err = read_words_via_s400_api(&buf[63], 63, 1, fuse->se_data);
		if (err)
			goto ret;

		err = read_words_via_s400_api(&buf[128], 128, 16, fuse->se_data);
		if (err)
			goto ret;

		err = read_words_via_s400_api(&buf[182], 182, 1, fuse->se_data);
		if (err)
			goto ret;

		err = read_words_via_s400_api(&buf[188], 188, 1, fuse->se_data);
		if (err)
			goto ret;

		for (bank = 39; bank < 64; bank++) {
			if (fuse->fsb_read_dis)
				read_words_via_s400_api(&buf[bank * 8], bank * 8,
							8, fuse->se_data);
			else
				read_nwords_via_fsb(regs, &buf[bank * 8], bank * 8, 8);
		}
	} else if (fuse->hw->soc == IMX95) {
		buf[0] = readl_relaxed(regs + 0 * 4) & 0xffff;
		buf[7] = readl_relaxed(regs + 7 * 4) & 0xffff;
		buf[9] = readl_relaxed(regs + 9 * 4) & 0xffff;
		buf[10] = readl_relaxed(regs + 10 * 4) & 0xffff;
		buf[11] = readl_relaxed(regs + 11 * 4) & 0xffff;
		for (i = 12; i < 36; i++)
			buf[i] = readl_relaxed(regs + i * 4);
		buf[36] = readl_relaxed(regs + 36 * 4) & 0xffff;
		buf[37] = readl_relaxed(regs + 37 * 4) & 0xffff;
		for (i = 38; i < 52; i++)
			buf[i] = readl_relaxed(regs + i * 4);
		buf[317] = readl_relaxed(regs + 317 * 4) & 0xffff;
		buf[318] = readl_relaxed(regs + 318 * 4) & 0xffff;
		for (i = 320; i < 327; i++)
			buf[i] = readl_relaxed(regs + i * 4);
		for (i = 328; i < 512; i++)
			buf[i] = readl_relaxed(regs + i * 4);

		read_words_via_s400_api(&buf[63], 63, 1, fuse->se_data);
		read_words_via_s400_api(&buf[128], 128, 16, fuse->se_data);
		read_words_via_s400_api(&buf[188], 188, 1, fuse->se_data);

		err = 0;

		fuse->pfn = offset >> 12 & 0xf;
		offset = offset & 0xfff;
	}

	memcpy(val, (u8 *)(buf) + offset, bytes);

ret:
	kfree(buf);
	mutex_unlock(&fuse->lock);

	return err;
}

static int fsb_s400_fuse_post_process(void *priv, const char *id, int index,
				      unsigned int offset, void *data,
				      size_t bytes)
{
	struct imx_fsb_s400_fuse *fuse = priv;
	u8 *buf = data;
	int i;

	if (!fuse)
		return -EINVAL;

	/* Deal with some post processing of nvmem cell data */
	if (id && !strcmp(id, "mac-address")) {
		if (fuse->hw->reverse_mac_address) {
			for (i = 0; i < bytes / 2; i++)
				swap(buf[i], buf[bytes - i - 1]);
		}

		if (fuse->hw->increase_mac_address &&
		    fuse->hw->pf_mac_offset_list) {
			if (fuse->pfn >= sizeof(fuse->hw->pf_mac_offset_list))
				return -EINVAL;
			eth_addr_add(buf,
				     fuse->hw->pf_mac_offset_list[fuse->pfn]);
		}
	}

	return 0;
}

static int fsb_s400_fuse_write(void *priv, unsigned int offset, void *val, size_t bytes)
{
	struct imx_fsb_s400_fuse *fuse = priv;
	u32 *buf = val;
	u32 index;
	int ret;

	/* allow only writing one complete OTP word at a time */
	if (bytes != 4)
		return -EINVAL;

	/* divide the offset by the word size to get the word count */
	index = offset / 4;

	mutex_lock(&fuse->lock);
	ret = imx_se_write_fuse(fuse->se_data, index, *buf, false);
	mutex_unlock(&fuse->lock);

	return ret;
}

struct imx_fsb_s400_fuse *gfuse;
static void imx_fsb_s400_fuse_fixup_cell_info(struct nvmem_device *nvmem,
					      struct nvmem_cell_info *cell)
{
	cell->priv = gfuse;
	cell->read_post_process = fsb_s400_fuse_post_process;
}

static int imx_fsb_s400_fuse_probe(struct platform_device *pdev)
{
	struct imx_fsb_s400_fuse *fuse;
	struct nvmem_device *nvmem;
	struct device_node *np;
	void __iomem *reg;
	u32 v;

	fuse = devm_kzalloc(&pdev->dev, sizeof(*fuse), GFP_KERNEL);
	if (!fuse)
		return -ENOMEM;

	gfuse = fuse;

	fuse->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fuse->regs))
		return PTR_ERR(fuse->regs);

	fuse->config.dev = &pdev->dev;
	fuse->config.name = "fsb_s400_fuse";
	fuse->config.id = NVMEM_DEVID_AUTO;
	fuse->config.owner = THIS_MODULE;
	fuse->config.size = 2048; /* 64 Banks */
	fuse->config.add_legacy_fixed_of_cells = true;
	fuse->config.reg_read = fsb_s400_fuse_read;
	if ((of_device_is_compatible(pdev->dev.of_node, "fsl,imx93-ocotp")) ||
	    (of_device_is_compatible(pdev->dev.of_node, "fsl,imx95-ocotp")))
		fuse->config.reg_write = fsb_s400_fuse_write;
	fuse->config.priv = fuse;
	mutex_init(&fuse->lock);
	fuse->hw = of_device_get_match_data(&pdev->dev);

	if (fuse->hw->reverse_mac_address || fuse->hw->increase_mac_address)
		fuse->config.fixup_dt_cell_info = &imx_fsb_s400_fuse_fixup_cell_info;

	if (fuse->hw->oscca_fuse_read) {
		np = of_find_compatible_node(NULL, NULL, "fsl,imx93-aonmix-ns-syscfg");
		if (!np)
			return -ENODEV;

		reg = of_iomap(np, 0);
		if (!reg)
			return -ENOMEM;

		v = readl_relaxed(reg + FUSE_ACC_DIS);
		if (v & BIT(0))
			fuse->fsb_read_dis = true;
		else
			fuse->fsb_read_dis = false;
	} else {
		fuse->fsb_read_dis = false;
	}

	nvmem = devm_nvmem_register(&pdev->dev, &fuse->config);
	if (IS_ERR(nvmem)) {
		dev_err(&pdev->dev, "failed to register fuse nvmem device\n");
		return PTR_ERR(nvmem);
	}

	fuse->se_data = imx_get_se_data_info(fuse->hw->se_soc_id, 0);
	dev_dbg(&pdev->dev, "fuse nvmem device registered successfully\n");

	return 0;
}

static const u8 imx95_pf_mac_offset_list[] = { 0, 3, 6 };

static const struct imx_fsb_s400_hw imx8ulp_fsb_s400_hw = {
	.soc = IMX8ULP,
	.fsb_otp_shadow = 0x800,
	.fsb_bank_reg = {
		[0] = { 3, 0 },
		[1] = { 4, 8 },
		[2] = { 5, 64 },
		[3] = { 6, 72 },
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
	.oscca_fuse_read = false,
	.reverse_mac_address = false,
	.increase_mac_address = false,
	.pf_mac_offset_list = NULL,
	.se_soc_id = SOC_ID_OF_IMX8ULP,
};

static const struct imx_fsb_s400_hw imx93_fsb_s400_hw = {
	.soc = IMX93,
	.fsb_otp_shadow = 0x8000,
	.oscca_fuse_read = true,
	.reverse_mac_address = true,
	.increase_mac_address = false,
	.pf_mac_offset_list = NULL,
	.se_soc_id = SOC_ID_OF_IMX93,
};

static const struct imx_fsb_s400_hw imx95_fsb_s400_hw = {
	.soc = IMX95,
	.fsb_otp_shadow = 0x8000,
	.oscca_fuse_read = false,
	.reverse_mac_address = false,
	.increase_mac_address = true,
	.pf_mac_offset_list = imx95_pf_mac_offset_list,
	.se_soc_id = SOC_ID_OF_IMX95,
};

static const struct of_device_id imx_fsb_s400_fuse_match[] = {
	{ .compatible = "fsl,imx8ulp-ocotp", .data = &imx8ulp_fsb_s400_hw, },
	{ .compatible = "fsl,imx93-ocotp", .data = &imx93_fsb_s400_hw, },
	{ .compatible = "fsl,imx95-ocotp", .data = &imx95_fsb_s400_hw, },
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
