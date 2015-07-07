/*
 * AD9548 SPI Network Clock Generator/Synchronizer
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio/consumer.h>

#include <linux/clk.h>
#include <linux/clk-provider.h>

#include <linux/iio/iio.h>

#define AD_READ		(1 << 15)
#define AD_WRITE	(0 << 15)
#define AD_CNT(x)	(((x) - 1) << 13)
#define AD_ADDR(x)	((x) & 0xFFF)
#define WAIT_B		0xFFFF
#define DELAY_N		0xFFFE
#define CHIPID_AD9548	0x48

enum ad8366_type {
	ID_AD9548,
	ID_HMC7044,
};

static const unsigned short ad9548_regs[][2] = {
	{0x0000, 0x30}, /* Reset */
	{0x0000, 0x10},
	{0x0000, 0x10},
	{0x0000, 0x10},
	{0x0000, 0x10},
	{0x0000, 0x10},
	{0x0100, 0x18}, /* System clock */
	{0x0101, 0x28},
	{0x0102, 0x45},
	{0x0103, 0x43},
	{0x0104, 0xDE},
	{0x0105, 0x13},
	{0x0106, 0x01},
	{0x0107, 0x00},
	{0x0108, 0x00},
	{0x0005, 0x01}, /* I/O Update */
	{0x0A02, 0x01}, /* Calibrate sysem clock */
	{0x0005, 0x01},
	{WAIT_B, 0x00},
	{0x0A02, 0x00},
	{0x0005, 0x01},
	{0x0208, 0x00}, /* IRQ Pin Output Mode */
	{0x0209, 0x00}, /* IRQ Masks */
	{0x020A, 0x00},
	{0x020B, 0x00},
	{0x020C, 0x00},
	{0x020D, 0x00},
	{0x020E, 0x00},
	{0x020F, 0x00},
	{0x0210, 0x00},
	{0x0211, 0x00}, /* Watchdog timer */
	{0x0212, 0x00},
	{0x0213, 0xFF}, /* Auxiliary DAC */
	{0x0214, 0x01},
	{0x0300, 0x29}, /* DPLL */
	{0x0301, 0x5C},
	{0x0302, 0x8F},
	{0x0303, 0xC2},
	{0x0304, 0xF5},
	{0x0305, 0x28},
	{0x0307, 0x00},
	{0x0308, 0x00},
	{0x0309, 0x00},
	{0x030A, 0xFF},
	{0x030B, 0xFF},
	{0x030C, 0xFF},
	{0x030D, 0x00},
	{0x030E, 0x00},
	{0x030F, 0x00},
	{0x0310, 0x00},
	{0x0311, 0x00},
	{0x0312, 0x00},
	{0x0313, 0x00},
	{0x0314, 0xE8},
	{0x0315, 0x03},
	{0x0316, 0x00},
	{0x0317, 0x00},
	{0x0318, 0x30},
	{0x0319, 0x75},
	{0x031A, 0x00},
	{0x031B, 0x00},
	{0x0306, 0x01}, /* Update TW */
	{0x0400, 0x0C}, /* Clock distribution output */
	{0x0401, 0x03},
	{0x0402, 0x00},
	{0x0403, 0x02},
	{0x0404, 0x04},
	{0x0405, 0x08},
	{0x0406, 0x03},
	{0x0407, 0x03},
	{0x0408, 0x03},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x03},
	{0x040D, 0x00},
	{0x040E, 0x00},
	{0x040F, 0x00},
	{0x0410, 0x00},
	{0x0411, 0x00},
	{0x0412, 0x00},
	{0x0413, 0x00},
	{0x0414, 0x00},
	{0x0415, 0x00},
	{0x0416, 0x00},
	{0x0417, 0x00},
	{0x0500, 0xFE}, /* Reference inputs */
	{0x0501, 0x00},
	{0x0502, 0x00},
	{0x0503, 0x08},
	{0x0504, 0x00},
	{0x0505, 0x00},
	{0x0506, 0x00},
	{0x0507, 0x00},
	{0x0600, 0x00}, /* Profiles are 0x0600-0x07FF */
	{0x0601, 0x55}, /* Profile 0 */
	{0x0602, 0xA0}, /* 30MHz input from FPGA, 122.880MHz output clock */
	{0x0603, 0xFC},
	{0x0604, 0x01},
	{0x0605, 0x00},
	{0x0606, 0x00},
	{0x0607, 0x00},
	{0x0608, 0xE8},
	{0x0609, 0x03},
	{0x060A, 0x00},
	{0x060B, 0xE8},
	{0x060C, 0x03},
	{0x060D, 0x00},
	{0x060E, 0x88},
	{0x060F, 0x13},
	{0x0610, 0x88},
	{0x0611, 0x13},
	{0x0612, 0x0E},
	{0x0613, 0xB2},
	{0x0614, 0x08},
	{0x0615, 0x82},
	{0x0616, 0x62},
	{0x0617, 0x42},
	{0x0618, 0xD8},
	{0x0619, 0x47},
	{0x061A, 0x21},
	{0x061B, 0xCB},
	{0x061C, 0xC4},
	{0x061D, 0x05},
	{0x061E, 0x7F},
	{0x061F, 0x00},
	{0x0620, 0x00},
	{0x0621, 0x00},
	{0x0622, 0x0B},
	{0x0623, 0x02},
	{0x0624, 0x00},
	{0x0625, 0x00},
	{0x0626, 0x26},
	{0x0627, 0xB0},
	{0x0628, 0x00},
	{0x0629, 0x10},
	{0x062A, 0x27},
	{0x062B, 0x20},
	{0x062C, 0x44},
	{0x062D, 0xF4},
	{0x062E, 0x01},
	{0x062F, 0x00},
	{0x0630, 0x20},
	{0x0631, 0x44},
	{0x0005, 0x01}, /* I/O Update */
	{0x0A0E, 0x01}, /* Force validation timeout */
	{0x0005, 0x01}, /* I/O Update */
	{0x0A02, 0x02}, /* Sync distribution */
	{0x0005, 0x01},
	{0x0A02, 0x00},
	{0x0005, 0x01},
};

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
	{0x0c8, 0xc1}, // clk-0 (adf4355-2) - en, perf, async
	{0x0c9, 0x0c}, // clk-0 (adf4355-2) - div (12) ~250M
	{0x0ca, 0x00}, // clk-0 (adf4355-2) - div (12) ~250M
	{0x0cd, 0x01}, // clk-0 (adf4355-2) - slip-cntrl
	{0x0d0, 0x08}, // clk-0 (adf4355-2) - lvpecl
	{0x0d2, 0xc0}, // clk-1 (disabled) - en, perf, async
	{0x0d3, 0x00}, // clk-1 (disabled) - div
	{0x0d4, 0x01}, // clk-1 (disabled) - div
	{0x0d7, 0x01}, // clk-1 (disabled) - slip-cntrl
	{0x0da, 0x08}, // clk-1 (disabled) - lvpecl
	{0x0dc, 0xc1}, // clk-2 (adf4355-1) - en, perf, async
	{0x0dd, 0x0c}, // clk-2 (adf4355-1) - div (12) ~250M
	{0x0de, 0x00}, // clk-2 (adf4355-1) - div (12) ~250M
	{0x0e1, 0x01}, // clk-2 (adf4355-1) - slip-cntrl
	{0x0e4, 0x08}, // clk-2 (adf4355-1) - lvpecl
	{0x0e6, 0xc0}, // clk-3 (disabled) - en, perf, async
	{0x0e7, 0x00}, // clk-3 (disabled) - div
	{0x0e8, 0x01}, // clk-3 (disabled) - div
	{0x0eb, 0x01}, // clk-3 (disabled) - slip-cntrl
	{0x0ee, 0x08}, // clk-3 (disabled) - lvpecl
	{0x0f0, 0xc1}, // clk-4 (adc-sysref-fmc) - en, perf, async
	{0x0f1, 0x80}, // clk-4 (adc-sysref-fmc) - div (384) ~8M
	{0x0f2, 0x01}, // clk-4 (adc-sysref-fmc) - div (384) ~8M
	{0x0f5, 0x01}, // clk-4 (adc-sysref-fmc) - slip-cntrl
	{0x0f8, 0x10}, // clk-4 (adc-sysref-fmc) - lvds
	{0x0fa, 0xc1}, // clk-5 (adc-clock-fmc) - en, perf, async
	{0x0fb, 0x06}, // clk-5 (adc-clock-fmc) - div (6) ~500M
	{0x0fc, 0x00}, // clk-5 (adc-clock-fmc) - div (6) ~500M
	{0x0ff, 0x01}, // clk-5 (adc-clock-fmc) - slip-cntrl
	{0x102, 0x10}, // clk-5 (adc-clock-fmc) - lvds
	{0x104, 0xc1}, // clk-6 (adc-clock) - en, perf, async
	{0x105, 0x03}, // clk-6 (adc-clock) - div (3) ~1G
	{0x106, 0x00}, // clk-6 (adc-clock) - div (3) ~1G
	{0x109, 0x01}, // clk-6 (adc-clock) - slip-cntrl
	{0x10c, 0x08}, // clk-6 (adc-clock) - lvpecl
	{0x10e, 0xc1}, // clk-7 (adc-sysref) - en, perf, async
	{0x10f, 0x80}, // clk-7 (adc-sysref) - div (384) ~8M
	{0x110, 0x01}, // clk-7 (adc-sysref) - div (384) ~8M
	{0x113, 0x01}, // clk-7 (adc-sysref) - slip-cntrl
	{0x116, 0x10}, // clk-7 (adc-sysref) - lvds
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
	{0x12c, 0xc1}, // clk-10 (dac-clock) - en, perf, async
	{0x12d, 0x03}, // clk-10 (dac-clock) - div (3) ~1G
	{0x12e, 0x00}, // clk-10 (dac-clock) - div (3) ~1G
	{0x131, 0x01}, // clk-10 (dac-clock) - slip-cntrl
	{0x134, 0x08}, // clk-10 (dac-clock) - lvpecl
	{0x136, 0xc1}, // clk-11 (dac-sysref-fmc) - en, perf, async
	{0x137, 0x80}, // clk-11 (dac-sysref-fmc) - div (384) ~8M
	{0x138, 0x01}, // clk-11 (dac-sysref-fmc) - div (384) ~8M
	{0x13b, 0x01}, // clk-11 (dac-sysref-fmc) - slip-cntrl
	{0x13e, 0x10}, // clk-11 (dac-sysref-fmc) - lvds
	{0x140, 0xc1}, // clk-12 (dac-clock-fmc) - en, perf, async
	{0x141, 0x06}, // clk-12 (dac-clock-fmc) - div (6) ~500M
	{0x142, 0x00}, // clk-12 (dac-clock-fmc) - div (6) ~500M
	{0x145, 0x01}, // clk-12 (dac-clock-fmc) - slip-cntrl
	{0x148, 0x10}, // clk-12 (dac-clock-fmc) - lvds
	{0x14a, 0xc1}, // clk-13 (dac-sysref) - en, perf, async
	{0x14b, 0x80}, // clk-13 (dac-sysref) - div (384) ~8M
	{0x14c, 0x01}, // clk-13 (dac-sysref) - div (384) ~8M
	{0x14f, 0x01}, // clk-13 (dac-sysref) - slip-cntrl
	{0x152, 0x10}, // clk-13 (dac-sysref) - lvds
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

struct hmc7044_clocks
{
	const char *name;
	unsigned long rate;
};

static const struct hmc7044_clocks hmc7044_clocks[] = {
	{"clk-0", HMC7044_VCO_RATE / 12},
	{"clk-1", 0},
	{"clk-2", HMC7044_VCO_RATE / 12},
	{"clk-3", 0},
	{"clk-4", HMC7044_VCO_RATE / 384},
	{"clk-5", HMC7044_VCO_RATE / 6},
	{"clk-6", HMC7044_VCO_RATE / 3},
	{"clk-7", HMC7044_VCO_RATE / 384},
	{"clk-8", 0},
	{"clk-9", 0},
	{"clk-10", HMC7044_VCO_RATE / 3},
	{"clk-11", HMC7044_VCO_RATE / 384},
	{"clk-12", HMC7044_VCO_RATE / 6},
	{"clk-13", HMC7044_VCO_RATE / 384},
};

static int ad9548_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[3];
	int ret;
	u16 cmd;

	cmd = AD_READ | AD_CNT(1) | AD_ADDR(reg);
	buf[0] = cmd >> 8;
	buf[1] = cmd & 0xFF;


	ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 1);
	if (ret < 0)
		return ret;
//	pr_err("%s:%d read 0x%X = 0x%X\n", __func__, __LINE__,reg, buf[2]);

	return buf[2];
}

static int ad9548_write(struct spi_device *spi,
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

//	pr_err("%s:%d 0x%X 0x%X\n", __func__, __LINE__, buf[2], ad9548_read(spi, reg));

	return 0;
}

static int ad9548_check_mask(struct spi_device *spi,
			 unsigned reg, unsigned mask, unsigned val)
{
	int ret, timeout = 100;

	do {
		ret = ad9548_read(spi, reg);
		if (ret < 0)
			return ret;
		if ((ret & mask) == val)
			break;
		mdelay(1);
	} while (timeout--);

	if (timeout <= 0) {
		dev_err(&spi->dev, "ERROR: Timeout!\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int ad9548_probe(struct spi_device *spi)
{
	int i, ret, regs;
	const unsigned short (*tab)[2];
	struct clk **clk = NULL;
	struct gpio_desc	 *enable_gpio;
	struct gpio_desc	 *sync_gpio;

	unsigned id = spi_get_device_id(spi)->driver_data;

	switch (id) {
	case ID_AD9548:
		ret = ad9548_read(spi, 0x3);
		if (ret < 0)
			return ret;
		if (ret != CHIPID_AD9548) {
			dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", ret);
	 		return -ENODEV;
		}

		tab = &ad9548_regs[0];
		regs = ARRAY_SIZE(ad9548_regs);
		break;
	case ID_HMC7044:

		enable_gpio = devm_gpiod_get(&spi->dev, "xo-en");
		if (!IS_ERR(enable_gpio)) {
			ret = gpiod_direction_output(enable_gpio, 1);
			pr_err("%s:%d\n", __func__, __LINE__);
			mdelay(250);
		}

		sync_gpio = devm_gpiod_get(&spi->dev, "sync");
		if (!IS_ERR(sync_gpio)) {
			ret = gpiod_direction_output(sync_gpio, 0);
			pr_err("%s:%d\n", __func__, __LINE__);
			mdelay(50);
		}

		tab = &hmc7044_regs[0];
		regs = ARRAY_SIZE(hmc7044_regs);
		break;
	default:
		return -ENODEV;
	}

	for (i = 0; i < regs; i++)
		switch (tab[i][0]) {
		case WAIT_B:
			ret = ad9548_check_mask(spi, 0xD01, BIT(0), BIT(0));
			if (ret < 0)
				return ret;
			break;
		case DELAY_N:
			mdelay(tab[i][1]);
			break;
		default:
			ret = ad9548_write(spi, tab[i][0],
					   tab[i][1]);
			if (ret < 0)
				return ret;

			break;
		}

	switch (id) {
	case ID_AD9548:
		dev_info(&spi->dev, "Rev. 0x%X probed\n", ad9548_read(spi, 0x2));
		break;
	case ID_HMC7044:

		clk = devm_kzalloc(&spi->dev, sizeof(struct clk *) *
				ARRAY_SIZE(hmc7044_clocks), GFP_KERNEL);
		if (clk == NULL)
			return -ENOMEM;

		ret = ad9548_check_mask(spi, 0x07c, 0x20, 0x20); // pll1 locked
		ret |= ad9548_check_mask(spi, 0x07d, 0x01, 0x01); // pll2 locked (nearly?)
		ret |= ad9548_check_mask(spi, 0x07d, 0x08, 0x08); // pll1 & pll2 locked
		if (ret < 0)
			return ret;

		for (i = 0; i < ARRAY_SIZE(hmc7044_clocks); i++) {
			clk[i] = clk_register_fixed_rate(&spi->dev,
				hmc7044_clocks[i].name, NULL, CLK_IS_ROOT,
				hmc7044_clocks[i].rate);
		}

		if (spi->dev.of_node) {
			struct clk_onecell_data *of_data;

			of_data = devm_kzalloc(&spi->dev,
						sizeof(*of_data), GFP_KERNEL);
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

		dev_info(&spi->dev, ": HMC7044 probed\n");
		break;
	default:
		return -ENODEV;
	}

	spi_set_drvdata(spi, clk);

	return 0;
}

static int ad9548_remove(struct spi_device *spi)
{
	struct clk **clk = spi_get_drvdata(spi);
	int i;

	if (spi_get_device_id(spi)->driver_data == ID_HMC7044)
		for (i = 0; i < ARRAY_SIZE(hmc7044_clocks); i++)
			clk_unregister(clk[i]);

	spi_set_drvdata(spi, NULL);

	return 0;
}

static const struct spi_device_id ad9548_id[] = {
	{"ad9548", ID_AD9548},
	{"hmc7044", ID_HMC7044},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9548_id);

static struct spi_driver ad9548_driver = {
	.driver = {
		.name	= "ad9548",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9548_probe,
	.remove		= ad9548_remove,
	.id_table	= ad9548_id,
};
module_spi_driver(ad9548_driver);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices AD9548");
MODULE_LICENSE("GPL v2");
