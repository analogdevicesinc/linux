// SPDX-License-Identifier: GPL-2.0-only
/*
 * Xilinx Spartan6 and 7 Series SelectMAP interface driver
 *
 * (C) 2024 Charles Perry <charles.perry@savoirfairelinux.com>
 *
 * Manage Xilinx FPGA firmware loaded over the SelectMAP configuration
 * interface.
 */

#include <linux/delay.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "xilinx-core.h"

#define IP_DATA_REG	0x18

/* ADI axi_selmap register offsets for register-based control */
#define ADI_SELMAP_REG_MAGIC		0x00
#define ADI_SELMAP_REG_RESET		0x08
#define ADI_SELMAP_REG_PROGRAM_B	0x0C
#define ADI_SELMAP_REG_DEVICE_READY	0x10
#define ADI_SELMAP_REG_CSI_B		0x14
#define ADI_SELMAP_REG_DONE		0x20

#define ADI_SELMAP_MAGIC		0x73656C6D

enum xilinx_selectmap_device_ids {
	ID_XC7S_SELMAP,
	ID_XC7A_SELMAP,
	ID_XC7K_SELMAP,
	ID_XC7V_SELMAP,
	ID_ADI_8_SELMAP,
	ID_ADI_16_SELMAP,
	ID_ADI_32_SELMAP,
};

struct xilinx_selectmap_conf {
	struct xilinx_fpga_core core;
	void __iomem *base;
	enum xilinx_selectmap_device_ids id;
};

#define to_xilinx_selectmap_conf(obj) \
	container_of(obj, struct xilinx_selectmap_conf, core)

static inline bool is_adi_selmap(enum xilinx_selectmap_device_ids id)
{
	return id == ID_ADI_8_SELMAP ||
	       id == ID_ADI_16_SELMAP ||
	       id == ID_ADI_32_SELMAP;
}

static int xilinx_selectmap_write(struct xilinx_fpga_core *core,
				  const char *buf, size_t count)
{
	struct xilinx_selectmap_conf *conf = to_xilinx_selectmap_conf(core);
	size_t i;
	const u16 *buf16 = (u16 *)buf;
	const u32 *buf32 = (u32 *)buf;

	switch (conf->id) {
	case ID_ADI_8_SELMAP:
		for (i = 0; i < count; ++i)
			writeb(buf[i], conf->base + IP_DATA_REG);
		break;
	case ID_ADI_16_SELMAP:
		for (i = 0; i < (count/2)+1; ++i)
			writew(cpu_to_be16(buf16[i]), conf->base + IP_DATA_REG);
		break;
	case ID_ADI_32_SELMAP:
		for (i = 0; i < (count/4)+1; ++i)
			writel(cpu_to_be32(buf32[i]), conf->base + IP_DATA_REG);
		break;
	default:
		for (i = 0; i < count; ++i)
			writeb(buf[i], conf->base);
		break;
	}

	return 0;
}

/*
 * ADI axi_selmap register-based FPGA manager ops.
 * Used when PROG/INIT/DONE GPIOs are not provided in device tree.
 */

static bool adi_selmap_get_done(struct xilinx_selectmap_conf *conf)
{
	return readl(conf->base + ADI_SELMAP_REG_DONE) & 1;
}

static bool adi_selmap_get_init_b(struct xilinx_selectmap_conf *conf)
{
	return readl(conf->base + ADI_SELMAP_REG_DEVICE_READY) & 1;
}

/*
 * PROG_B is directly driven by the register bit.
 * assert=true drives the pin low to start programming sequence.
 */
static void adi_selmap_set_prog_b(struct xilinx_selectmap_conf *conf, bool assert)
{
	writel(assert ? 0 : 1, conf->base + ADI_SELMAP_REG_PROGRAM_B);
}

/*
 * CSI_B is directly driven by the register bit.
 * assert=true drives the pin low to select the FPGA.
 */
static void adi_selmap_set_csi_b(struct xilinx_selectmap_conf *conf, bool assert)
{
	writel(assert ? 0 : 1, conf->base + ADI_SELMAP_REG_CSI_B);
}

static enum fpga_mgr_states adi_selmap_state(struct fpga_manager *mgr)
{
	struct xilinx_selectmap_conf *conf = mgr->priv;

	if (!adi_selmap_get_done(conf))
		return FPGA_MGR_STATE_RESET;

	return FPGA_MGR_STATE_UNKNOWN;
}

static int adi_selmap_write_init(struct fpga_manager *mgr,
				 struct fpga_image_info *info,
				 const char *buf, size_t count)
{
	struct xilinx_selectmap_conf *conf = mgr->priv;
	unsigned long timeout;

	if (info->flags & FPGA_MGR_PARTIAL_RECONFIG) {
		dev_err(&mgr->dev, "Partial reconfiguration not supported\n");
		return -EINVAL;
	}

	/* Assert PROG_B to start configuration sequence */
	adi_selmap_set_prog_b(conf, true);

	/* Wait for INIT_B to assert (device_ready goes low) */
	timeout = jiffies + msecs_to_jiffies(1000);
	while (time_before(jiffies, timeout)) {
		if (!adi_selmap_get_init_b(conf))
			break;
		usleep_range(100, 400);
	}

	if (adi_selmap_get_init_b(conf)) {
		dev_err(&mgr->dev, "Timeout waiting for INIT_B to assert\n");
		adi_selmap_set_prog_b(conf, false);
		return -ETIMEDOUT;
	}

	/* Deassert PROG_B */
	adi_selmap_set_prog_b(conf, false);

	/* Wait for INIT_B to deassert (device_ready goes high) */
	timeout = jiffies + msecs_to_jiffies(1000);
	while (time_before(jiffies, timeout)) {
		if (adi_selmap_get_init_b(conf))
			break;
		usleep_range(100, 400);
	}

	if (!adi_selmap_get_init_b(conf)) {
		dev_err(&mgr->dev, "Timeout waiting for INIT_B to deassert\n");
		return -ETIMEDOUT;
	}

	if (adi_selmap_get_done(conf)) {
		dev_err(&mgr->dev, "Unexpected DONE pin state\n");
		return -EIO;
	}

	/* Assert CSI_B to select the FPGA */
	adi_selmap_set_csi_b(conf, true);

	/* Program latency before configuration data can be sent */
	usleep_range(7500, 8000);

	return 0;
}

static int adi_selmap_write(struct fpga_manager *mgr, const char *buf,
			    size_t count)
{
	struct xilinx_selectmap_conf *conf = mgr->priv;

	return xilinx_selectmap_write(&conf->core, buf, count);
}

static int adi_selmap_write_complete(struct fpga_manager *mgr,
				     struct fpga_image_info *info)
{
	struct xilinx_selectmap_conf *conf = mgr->priv;
	static const char padding[1] = { 0xff };
	unsigned long timeout;
	bool expired = false;
	bool done;
	int ret;

	timeout = jiffies + usecs_to_jiffies(info->config_complete_timeout_us);

	/*
	 * This loop is carefully written such that if the driver is
	 * scheduled out for more than 'timeout', we still check for DONE
	 * before giving up and we apply extra CCLK cycles via padding.
	 */
	while (!expired) {
		expired = time_after(jiffies, timeout);

		done = adi_selmap_get_done(conf);

		ret = xilinx_selectmap_write(&conf->core, padding, sizeof(padding));
		if (ret) {
			adi_selmap_set_csi_b(conf, false);
			return ret;
		}

		if (done) {
			adi_selmap_set_csi_b(conf, false);
			return 0;
		}
	}

	if (!adi_selmap_get_init_b(conf))
		dev_err(&mgr->dev, "CRC error or invalid device\n");
	else
		dev_err(&mgr->dev, "Timeout after config data transfer\n");

	adi_selmap_set_csi_b(conf, false);
	return -ETIMEDOUT;
}

static const struct fpga_manager_ops adi_selmap_ops = {
	.state = adi_selmap_state,
	.write_init = adi_selmap_write_init,
	.write = adi_selmap_write,
	.write_complete = adi_selmap_write_complete,
};

static int xilinx_selectmap_probe(struct platform_device *pdev)
{
	struct xilinx_selectmap_conf *conf;
	struct device_node *np = pdev->dev.of_node;
	struct gpio_desc *gpio;
	struct fpga_manager *mgr;
	void __iomem *base;
	u32 magic;

	conf = devm_kzalloc(&pdev->dev, sizeof(*conf), GFP_KERNEL);
	if (!conf)
		return -ENOMEM;

	conf->core.dev = &pdev->dev;
	conf->core.write = xilinx_selectmap_write;
	conf->id = (enum xilinx_selectmap_device_ids)device_get_match_data(&pdev->dev);

	base = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(base))
		return dev_err_probe(&pdev->dev, PTR_ERR(base),
				     "ioremap error\n");
	conf->base = base;

	/*
	 * For ADI variants, check if PROG GPIO is provided in device tree.
	 * If not, use register-based control via the axi_selmap IP.
	 */
	if (is_adi_selmap(conf->id) &&
	    (!np || !of_property_present(np, "prog-gpios"))) {
		magic = readl(base + ADI_SELMAP_REG_MAGIC);
		if (magic != ADI_SELMAP_MAGIC)
			return dev_err_probe(&pdev->dev, -ENODEV,
					     "Invalid magic 0x%08x (expected 0x%08x)\n",
					     magic, ADI_SELMAP_MAGIC);

		dev_info(&pdev->dev,
			 "Using register-based FPGA programming control\n");

		/* Reset the core (self-clearing, but ensure write completes) */
		writel(0, base + ADI_SELMAP_REG_RESET);

		/* Initialize to idle state: PROG_B deasserted, CSI_B deasserted */
		adi_selmap_set_prog_b(conf, false);
		adi_selmap_set_csi_b(conf, false);

		mgr = devm_fpga_mgr_register(&pdev->dev,
					     "ADI SelectMAP FPGA Manager",
					     &adi_selmap_ops, conf);
		return PTR_ERR_OR_ZERO(mgr);
	}

	/* CSI_B is active low */
	gpio = devm_gpiod_get_optional(&pdev->dev, "csi", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio))
		return dev_err_probe(&pdev->dev, PTR_ERR(gpio),
				     "Failed to get CSI_B gpio\n");

	/* RDWR_B is active low */
	gpio = devm_gpiod_get_optional(&pdev->dev, "rdwr", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio))
		return dev_err_probe(&pdev->dev, PTR_ERR(gpio),
				     "Failed to get RDWR_B gpio\n");

	return xilinx_core_probe(&conf->core);
}

static const struct of_device_id xlnx_selectmap_of_match[] = {
	{ .compatible = "xlnx,fpga-xc7s-selectmap", .data = (void *)ID_XC7S_SELMAP, }, // Spartan-7
	{ .compatible = "xlnx,fpga-xc7a-selectmap", .data = (void *)ID_XC7A_SELMAP, }, // Artix-7
	{ .compatible = "xlnx,fpga-xc7k-selectmap", .data = (void *)ID_XC7K_SELMAP, }, // Kintex-7
	{ .compatible = "xlnx,fpga-xc7v-selectmap", .data = (void *)ID_XC7V_SELMAP, }, // Virtex-7
	{ .compatible = "adi,fpga-8-selectmap", .data = (void *)ID_ADI_8_SELMAP, }, // ADI 8bit version
	{ .compatible = "adi,fpga-16-selectmap", .data = (void *)ID_ADI_16_SELMAP, }, // ADI 16bit version
	{ .compatible = "adi,fpga-32-selectmap", .data = (void *)ID_ADI_32_SELMAP, }, // ADI 32bit version
	{},
};
MODULE_DEVICE_TABLE(of, xlnx_selectmap_of_match);

static struct platform_driver xilinx_selectmap_driver = {
	.driver = {
		.name = "xilinx-selectmap",
		.of_match_table = xlnx_selectmap_of_match,
	},
	.probe  = xilinx_selectmap_probe,
};

module_platform_driver(xilinx_selectmap_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Charles Perry <charles.perry@savoirfairelinux.com>");
MODULE_DESCRIPTION("Load Xilinx FPGA firmware over SelectMap");
