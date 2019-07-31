// SPDX-License-Identifier: GPL-2.0
/*
 * AXI SYSTEM ID HDL CORE driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/nvmem-provider.h>
#include <linux/fpga/adi-axi-common.h>

#define AXI_SYSID_WORD_SIZE	sizeof(u32)


#define AXI_SYSID_REG_ROM_ADDR_WIDTH	0x40
#define AXI_SYSID_ROM_OFFSET		0x800

struct axi_sysid {
	void __iomem *base;
	const struct axi_sysid_core_info *info;
	u32 size;
	u32 *mem;
};

struct sysid_header_v1 {
	u32 version;
	u32 build_info_offs;
	u32 board_info_offs;
	u32 product_info_offs;
	u32 custom_info_offs;
	u32 pr_custom_info_offs;
	u32 padding[9];
	u32 crc;
} __packed;


struct build_info_header_v1 {
	s8 git_hash[44];
	s8 git_clean_chk[4];
	s8 vadj_chk[4];
	s8 epoch[12];
	u8 padding[4];
	u32 crc;
} __packed;

struct axi_sysid_core_info {
	u32 version;
	u32 header_version;
};

static inline u32 axi_sysid_ioread(const struct axi_sysid *st, const u32 reg)
{
	return ioread32(st->base + reg);
}

static int axi_sysid_read(void *context, unsigned int offset,
			  void *val, size_t bytes)
{
	struct axi_sysid *st = context;
	unsigned int count = bytes >> 2;
	u32 index = offset >> 2;
	u32 *buf = val;
	int i;

	if (count > (st->size - index))
		count = st->size - index;

	for (i = index; i < (index + count); i++)
		*buf++ = axi_sysid_ioread(st, AXI_SYSID_ROM_OFFSET + i *
					  AXI_SYSID_WORD_SIZE);

	return 0;
}

static u8 axi_sysid_checksum(u8 *ptr, size_t s)
{
	u8 sum = 0;

	while (s-- != 0)
		sum -= *ptr++;

	return sum;
}

static char *axi_sysid_get_str(struct axi_sysid *st, int this)
{
	if (this > 0) {
		char *str = (char *) &st->mem[this];

		if (*str)
			return str;
	}

	return NULL;
}

static int axi_sysid_validate(struct platform_device *pdev,
			      struct axi_sysid *st)
{
	struct sysid_header_v1 *header;
	struct build_info_header_v1 *build;
	struct tm tm;
	time64_t t;
	int ret;

	st->mem = devm_kzalloc(&pdev->dev, st->size, GFP_KERNEL);
	if (!st->mem)
		return -ENOMEM;

	axi_sysid_read(st, 0, st->mem, st->size);
	header = (struct sysid_header_v1 *) st->mem;

	if (axi_sysid_checksum((u8 *)st->mem, sizeof(struct sysid_header_v1))) {
		dev_err(&pdev->dev, "verfify header checksum failed\n");
		return -EFAULT;
	}

	if (header->version != st->info->header_version) {
		dev_err(&pdev->dev,
			"system ID header version mismatch. Expected %d, Reported %d\n",
			st->info->header_version, header->version);
		return -EFAULT;
	}

	build = (struct build_info_header_v1 *) &st->mem[
		header->build_info_offs];

	if (axi_sysid_checksum((u8 *)build,
		sizeof(struct build_info_header_v1))) {
		dev_err(&pdev->dev, "verfify build header checksum failed\n");
		return -EFAULT;
	}

	ret = sscanf(build->epoch, "%12lld", &t);
	if (ret != 1)
		return -EINVAL;

	time64_to_tm(t, 0, &tm);

	dev_info(&pdev->dev,
		"[%s] on [%s] git <%.40s> %s [%ld-%02d-%02d %02d:%02d:%02d] UTC\n",
		axi_sysid_get_str(st, header->board_info_offs),
		axi_sysid_get_str(st, header->product_info_offs),
		build->git_hash,
		(build->git_clean_chk[0] == 't') ? "clean" : "dirty",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec);

	return 0;
}

static struct nvmem_config axi_sysid_nvmem_config = {
	.name = "system-id",
	.read_only = true,
	.word_size = AXI_SYSID_WORD_SIZE,
	.stride = AXI_SYSID_WORD_SIZE,
	.owner = THIS_MODULE,
	.reg_read = axi_sysid_read,
};

static const struct axi_sysid_core_info version_1_0_0_info = {
	.version = ADI_AXI_PCORE_VER(1, 0, 'a'),
	.header_version = 1,
};

static const struct of_device_id axi_sysid_of_match[] = {
	{ .compatible = "adi,axi-sysid-1.00.a",
		.data = (void *)&version_1_0_0_info},
	{},
};
MODULE_DEVICE_TABLE(of, axi_sysid_of_match);

static int axi_sysid_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct nvmem_device *nvmem;
	struct axi_sysid *st;
	struct resource *res;
	u32 version;

	id = of_match_node(axi_sysid_of_match, pdev->dev.of_node);
	if (!id)
		return -EINVAL;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(st->base))
		return PTR_ERR(st->base);

	st->info = id->data;

	version = axi_sysid_ioread(st, ADI_AXI_REG_VERSION);
	if (ADI_AXI_PCORE_VER_MAJOR(version) !=
		ADI_AXI_PCORE_VER_MAJOR(st->info->version)) {
		dev_err(&pdev->dev,
			"Major version mismatch. Expected %d.%.2d.%c, Reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(st->info->version),
			ADI_AXI_PCORE_VER_MINOR(st->info->version),
			ADI_AXI_PCORE_VER_PATCH(st->info->version),
			ADI_AXI_PCORE_VER_MAJOR(version),
			ADI_AXI_PCORE_VER_MINOR(version),
			ADI_AXI_PCORE_VER_PATCH(version));
		return -ENODEV;
	}

	st->size = (1 << axi_sysid_ioread(st, AXI_SYSID_REG_ROM_ADDR_WIDTH)) *
		   AXI_SYSID_WORD_SIZE;

	axi_sysid_validate(pdev, st);

	axi_sysid_nvmem_config.size = st->size;
	axi_sysid_nvmem_config.dev = &pdev->dev;
	axi_sysid_nvmem_config.priv = st;

	nvmem = nvmem_register(&axi_sysid_nvmem_config);
	if (IS_ERR(nvmem))
		return PTR_ERR(nvmem);

	platform_set_drvdata(pdev, nvmem);

	return 0;
}

static int axi_sysid_remove(struct platform_device *pdev)
{
	struct nvmem_device *nvmem = platform_get_drvdata(pdev);

	return nvmem_unregister(nvmem);
}

static struct platform_driver axi_sysid_driver = {
	.probe	= axi_sysid_probe,
	.remove	= axi_sysid_remove,
	.driver = {
		.name	= "axi_sysid",
		.of_match_table = axi_sysid_of_match,
	},
};
module_platform_driver(axi_sysid_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AXI SYSTEM ID");
MODULE_LICENSE("GPL");
