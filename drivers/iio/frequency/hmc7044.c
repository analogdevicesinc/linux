/*
 * Licensed under the GPL-2.
 *
 * THIS IS NOT A DRIVER. DO NOT USE.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <linux/clk.h>
#include <linux/clk-provider.h>

#define AD_WRITE	(0 << 15)
#define AD_CNT(x)	(((x) - 1) << 13)
#define AD_ADDR(x)	((x) & 0xFFF)
#define DELAY_N		0xFFFF

static const unsigned short hmc7044_regs[][2] = {
	{0x000, 0x00}, // reset
	{0x001, 0x00}, // restart

// 	{0x046, 0x00},
// 	{0x047, 0x00},
// 	{0x048, 0x00},
// 	{0x049, 0x00},
// 	{0x0a0, 0x56},

	{0x003, 0x2f}, // pll1, pll2 sysref - high vco
	{0x00a, 0x15}, // in0 - 30.72M
	{0x00b, 0x00}, // in1 - not used
	{0x00c, 0x00}, // in2 - not used
	{0x00d, 0x00}, // in3 - not used
	{0x00e, 0x15}, // vcxo - 122.88M

	{0x032, 0x01}, // x2 bypass
	{0x033, 0x01}, // r2 div (1)
	{0x034, 0x00}, // r2 div
	{0x035, 0x18}, // n2 div (24) - vco - 2949.12M
	{0x036, 0x00}, // n2 div

	{0x046, 0x00},
	{0x047, 0x00},
	{0x048, 0x00},
	{0x049, 0x00},

	{0x01c, 0x01}, // in0-div (1)
	{0x01d, 0x01}, // in1-div (1)
	{0x01e, 0x01}, // in2-div (1)
	{0x01f, 0x01}, // in3-div (1)
	{0x020, 0x04}, // vcxo-div (4)
	{0x021, 0x01}, // r1-div (1)
	{0x022, 0x00}, // r1-div
	{0x026, 0x04}, // n1-div (4)
	{0x027, 0x00}, // n1-div

	{0x050, 0x1f}, // gpio0 -pll1 lock
	{0x051, 0x2b}, // gpio1 -pll2 lock
	{0x05a, 0x00}, // sysref-pulse
	{0x05c, 0x80}, // sysref-divratio (>1000)
	{0x05d, 0x04}, // 128*3*3 = 1152

#if 1
	{0x096, 0x00},
	{0x097, 0x00},
	{0x098, 0x00},
	{0x099, 0x00},
	{0x09a, 0x00},
	{0x09b, 0xaa},
	{0x09c, 0xaa},
	{0x09d, 0xaa},
	{0x09e, 0xaa},
	{0x09f, 0x55},
	{0x0a0, 0x56},
	{0x0a1, 0x97},
	{0x0a2, 0x02},
	{0x0a3, 0x00},
	{0x0a4, 0x00},
	{0x0a5, 0x0e},
	{0x0a6, 0x1e},
	{0x0a7, 0x00},
	{0x0a8, 0x22},
	{0x0a9, 0x00},
	{0x0ab, 0x00},
	{0x0ac, 0x20},
	{0x0ad, 0x00},
	{0x0ae, 0x08},
	{0x0af, 0x50},
	{0x0b0, 0x09},
	{0x0b1, 0x0d},
	{0x0b2, 0x00},
	{0x0b3, 0x00},
	{0x0b5, 0x00},
	{0x0b6, 0x00},
	{0x0b7, 0x00},
	{0x0b8, 0x00},
#endif
	{0x0a0, 0x56},
	{DELAY_N, 10}, // DELAY
	{0x0c8, 0xc0}, // clk-0 (disabled) - en, perf, async
	{0x0c9, 0x0c}, // clk-0 (disabled) - div (12) ~250M
	{0x0ca, 0x00}, // clk-0 (disabled) - div (12) ~250M
	{0x0cd, 0x01}, // clk-0 (disabled) - slip-cntrl
	{0x0d0, 0x08}, // clk-0 (disabld) - lvpecl
	{0x0d2, 0xc1}, // clk-1 (disabled) - en, perf, async
	{0x0d3, 0x00}, // clk-1 (disabled) - div
	{0x0d4, 0x01}, // clk-1 (disabled) - div
	{0x0d7, 0x01}, // clk-1 (disabled) - slip-cntrl
	{0x0da, 0x08}, // clk-1 (disabled) - lvpecl
	{0x0dc, 0xc1}, // clk-2 (dac) - en, perf, async
	{0x0dd, 0x01}, // clk-2 (dac) - div (1) ~250M
	{0x0de, 0x00}, // clk-2 (dac) - div (1) ~250M
	{0x0e1, 0x01}, // clk-2 (dac) - slip-cntrl
	{0x0e4, 0x08}, // clk-2 (dac) - lvpecl
	{0x0e6, 0xc1}, // clk-3 (dac-sysref) - en, perf, async
	{0x0e7, 0x00}, // clk-3 (dac-sysref) - div
	{0x0e8, 0x08}, // clk-3 (dac-sysref) - div
	{0x0eb, 0x02}, // clk-3 (dac-sysref - slip-cntrl
	{0x0ee, 0x10}, // clk-3 (dac-sysref) - lvds
	{0x0f0, 0xc0}, // clk-4 (disabled) - en, perf, async
	{0x0f1, 0x80}, // clk-4 (disabled) - div (384) ~8M
	{0x0f2, 0x01}, // clk-4 (disabled) - div (384) ~8M
	{0x0f5, 0x01}, // clk-4 (disabled) - slip-cntrl
	{0x0f8, 0x10}, // clk-4 (disabled) - lvds
	{0x0fa, 0xc0}, // clk-5 (disabled) - en, perf, async
	{0x0fb, 0x06}, // clk-5 (disabled) - div (6) ~500M
	{0x0fc, 0x00}, // clk-5 (disabled) - div (6) ~500M
	{0x0ff, 0x01}, // clk-5 (disabled) - slip-cntrl
	{0x102, 0x10}, // clk-5 (disabled) - lvds
	{0x104, 0xc1}, // clk-6 (disabled) - en, perf, async
	{0x105, 0x03}, // clk-6 (disabled) - div (3) ~1G
	{0x106, 0x00}, // clk-6 (disabled) - div (3) ~1G
	{0x109, 0x01}, // clk-6 (disabled) - slip-cntrl
	{0x10c, 0x08}, // clk-6 (disabled) - lvpecl
	{0x10e, 0xc0}, // clk-7 (disabled) - en, perf, async
	{0x10f, 0x80}, // clk-7 (disabled) - div (384) ~8M
	{0x110, 0x01}, // clk-7 (disabled) - div (384) ~8M
	{0x113, 0x01}, // clk-7 (disabled) - slip-cntrl
	{0x116, 0x10}, // clk-7 (disabled) - lvds
	{0x118, 0xc0}, // clk-8 (disabled) - en, perf, async
	{0x119, 0x00}, // clk-8 (disabled) - div
	{0x11a, 0x01}, // clk-8 (disabled) - div
	{0x11d, 0x01}, // clk-8 (disabled) - slip-cntrl
	{0x120, 0x10}, // clk-8 (disabled) - lvds
	{0x122, 0xc0}, // clk-9 (disabled) - en, perf, async
	{0x123, 0x00}, // clk-9 (disabled) - div
	{0x124, 0x01}, // clk-9 (disabled) - div
	{0x127, 0x01}, // clk-9 (disabled) - slip-cntrl
	{0x12a, 0x10}, // clk-9 (disabled) - lvds
	{0x12c, 0xc0}, // clk-10 (disabled) - en, perf, async
	{0x12d, 0x03}, // clk-10 (disabled) - div (3) ~1G
	{0x12e, 0x00}, // clk-10 (disabled) - div (3) ~1G
	{0x131, 0x01}, // clk-10 (disabled) - slip-cntrl
	{0x134, 0x08}, // clk-10 (disabled) - lvpecl
	{0x136, 0xc0}, // clk-11 (disabled) - en, perf, async
	{0x137, 0x80}, // clk-11 (disabled) - div (384) ~8M
	{0x138, 0x01}, // clk-11 (disabled) - div (384) ~8M
	{0x13b, 0x01}, // clk-11 (disabled) - slip-cntrl
	{0x13e, 0x10}, // clk-11 (disabled) - lvds
	{0x140, 0xc1}, // clk-12 (fpga-clock-fmc) - en, perf, async
	{0x141, 0x08}, // clk-12 (fpga-clock-fmc) - div (6) 368.64M
	{0x142, 0x00}, // clk-12 (fpga-clock-fmc) - div (6) 368.64M
	{0x145, 0x01}, // clk-12 (fppga-clock-fmc) - slip-cntrl
	{0x148, 0x10}, // clk-12 (fpga-clock-fmc) - lvds
	{0x14a, 0xc1}, // clk-13 (fpga-sysref) - en, perf, async
	{0x14b, 0x00}, // clk-13 (fpga-sysref) - div (512) ~8M
	{0x14c, 0x02}, // clk-13 (fpga-sysref) - div (512) ~8M
	{0x14f, 0x02}, // clk-13 (fpga-sysref) - slip-cntrl
	{0x152, 0x10}, // clk-13 (fpga-sysref) - lvds
	{0x039, 0x01}, // osc-out path enable
	{0x03a, 0x33}, // osc0-out enable
	{0x03b, 0x33}, // osc1-out enable
	{DELAY_N, 10}, // DELAY
	{0x001, 0x02}, // restart
	{DELAY_N, 1}, // DELAY_N
	{0x001, 0x00}, // restart
	{DELAY_N, 1}, // DELAY
};

#define HMC7044_VCO_RATE (24U * 122880000UL)

struct hmc7044_clocks {
	const char *name;
	unsigned long rate;
};

static const struct hmc7044_clocks hmc7044_clocks[] = {
	{ "clk-0", 0 },
	{ "clk-1", 0 },
	{ "clk-2", HMC7044_VCO_RATE },
	{ "clk-3", HMC7044_VCO_RATE / 512 },
	{ "clk-4", 0 },
	{ "clk-5", 0 },
	{ "clk-6", 0 },
	{ "clk-7", 0 },
	{ "clk-8", 0 },
	{ "clk-9", 0 },
	{ "clk-10", 0 },
	{ "clk-11", 0 },
	{ "clk-12", HMC7044_VCO_RATE / 8 },
	{ "clk-13", HMC7044_VCO_RATE / 512 },
};

static int hmc7044_write(struct spi_device *spi,
			 unsigned reg, unsigned val)
{
	unsigned char buf[3];
	int ret;
	u16 cmd;

	cmd = AD_WRITE | AD_CNT(1) | AD_ADDR(reg);
	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xFF;
	buf[2] = val;

	ret = spi_write(spi, buf, 3);
	if (ret < 0)
		return ret;

	return 0;
}

static int hmc7044_probe(struct spi_device *spi)
{
	struct clk **clk;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(hmc7044_regs); i++) {
		switch (hmc7044_regs[i][0]) {
		case DELAY_N:
			mdelay(hmc7044_regs[i][1]);
			break;
		default:
			ret = hmc7044_write(spi, hmc7044_regs[i][0],
					   hmc7044_regs[i][1]);
			if (ret < 0)
				return ret;
			break;
		}
	}

	clk = devm_kcalloc(&spi->dev, ARRAY_SIZE(hmc7044_clocks),
			   sizeof(struct clk *), GFP_KERNEL);
	if (!clk)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(hmc7044_clocks); i++) {
		clk[i] = clk_register_fixed_rate(&spi->dev,
			hmc7044_clocks[i].name, NULL, 0,
			hmc7044_clocks[i].rate);
	}

	if (spi->dev.of_node) {
		struct clk_onecell_data *of_data;

		of_data = devm_kzalloc(&spi->dev, sizeof(*of_data), GFP_KERNEL);
		if (!of_data)
			return -ENOMEM;

		of_data->clks = clk;
		of_data->clk_num = ARRAY_SIZE(hmc7044_clocks);
		ret = of_clk_add_provider(spi->dev.of_node,
					of_clk_src_onecell_get, of_data);
		if (ret) {
			dev_err(&spi->dev, "failed to register OF clock provider\n");
			return ret;
		}
	}

	spi_set_drvdata(spi, clk);

	return 0;
}

static int hmc7044_remove(struct spi_device *spi)
{
	struct clk **clk = spi_get_drvdata(spi);
	unsigned int i;

	if (spi->dev.of_node)
	    of_clk_del_provider(spi->dev.of_node);

	for (i = 0; i < ARRAY_SIZE(hmc7044_clocks); i++)
		clk_unregister(clk[i]);

	return 0;
}

static const struct spi_device_id hmc7044_id[] = {
	{"hmc7044", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, hmc7044_id);

static struct spi_driver hmc7044_driver = {
	.driver = {
		.name = "hmc7044",
	},
	.probe = hmc7044_probe,
	.remove = hmc7044_remove,
	.id_table = hmc7044_id,
};
module_spi_driver(hmc7044_driver);

MODULE_LICENSE("GPL v2");
