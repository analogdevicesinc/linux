#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/delay.h>

#define ADF4360_REG_CTRL	0x00
#define ADF4360_REG_R_COUNTER	0x01
#define ADF4360_REG_B_COUNTER	0x02

#define ADF4360_CTRL_PC_2_5	(0x0 << 2)
#define ADF4360_CTRL_PC_5	(0x1 << 2)
#define ADF4360_CTRL_PC_7_5	(0x2 << 2)
#define ADF4360_CTRL_PC_10	(0x3 << 2)
#define ADF4360_CTRL_COUNTER_RESET		BIT(4)
#define ADF4360_CTRL_MTLD	BIT(11)
#define ADF4360_CTRL_PL_3_5	(0x0 << 12)
#define ADF4360_CTRL_PL_5	(0x1 << 12)
#define ADF4360_CTRL_PL_7_5	(0x2 << 12)
#define ADF4360_CTRL_PL_11	(0x3 << 12)
#define ADF4360_CTRL_CPI1(x)	((x) << 14)
#define ADF4360_CTRL_CPI2(x)	((x) << 17)

#define ADF4360_CPI_0_31	0
#define ADF4360_CPI_0_62	1
#define ADF4360_CPI_0_93	2
#define ADF4360_CPI_1_25	3
#define ADF4360_CPI_1_56	4
#define ADF4360_CPI_1_87	5
#define ADF4360_CPI_2_18	6
#define ADF4360_CPI_2_50	7

#define ADF4360_DIVOUT(x) ((x) << 2)
#define ADF4360_B_COUNTER(x) ((x) << 8)

#define ADF4360_R_COUNTER(x) ((x) << 2)
#define ADF4360_ABP(x) ((x) << 16)
#define ADF4360_BSC(x) ((x) << 20)

struct adf4360 {
	struct spi_device *spi;
	unsigned long r, b, divout;

	struct clk_hw clk_hw;

	u8 spi_data[3] ____cacheline_aligned;
};

static struct adf4360 *clk_hw_to_adf4360(struct clk_hw *clk_hw)
{
	return container_of(clk_hw, struct adf4360, clk_hw);
}

static int adf4360_write_reg(struct adf4360 *adf4360, unsigned int reg,
	unsigned int val)
{
	val |= reg;

	adf4360->spi_data[0] = (val >> 16) & 0xff;
	adf4360->spi_data[1] = (val >> 8) & 0xff;
	adf4360->spi_data[2] = val & 0xff;

	return spi_write(adf4360->spi, adf4360->spi_data, 3);
}

/* fVCO = B * fREFIN / R */

static unsigned long adf4360_recalc_rate(struct clk_hw *clk_hw,
	unsigned long parent_rate)
{
	struct adf4360 *adf4360 = clk_hw_to_adf4360(clk_hw);

	return parent_rate * adf4360->b / (adf4360->divout * adf4360->r);
}

static long adf4360_round_rate(struct clk_hw *clk_hw,
	unsigned long rate, unsigned long *parent_rate)
{
	struct adf4360 *adf4360 = clk_hw_to_adf4360(clk_hw);

	return *parent_rate * adf4360->b / (adf4360->divout * adf4360->r);
}

static int adf4360_set_rate(struct clk_hw *clk_hw,
	unsigned long rate, unsigned long parent_rate)
{
	return 0;
}

static const struct clk_ops adf4360_clk_ops = {
	.recalc_rate = adf4360_recalc_rate,
	.round_rate = adf4360_round_rate,
	.set_rate = adf4360_set_rate,
};

static void adf4360_m2k_setup(struct adf4360 *adf4360)
{
	unsigned int val_r, val_ctrl, val_b;

	adf4360->b = 20;
	adf4360->r = 4;
	adf4360->divout = 1;

	val_ctrl = ADF4360_CTRL_PC_5;
	val_ctrl |= ADF4360_CTRL_CPI1(ADF4360_CPI_2_50);
	val_ctrl |= ADF4360_CTRL_CPI2(ADF4360_CPI_2_50);
	val_ctrl |= ADF4360_CTRL_PL_5;
	val_ctrl |= 5 << 5;
	val_ctrl |= 1 << 8;
//	val_ctrl |= BIT(11);
//	val_ctrl |= BIT(20);

	val_r = ADF4360_R_COUNTER(adf4360->r);
	val_r |= ADF4360_BSC(3); /* Divide by 8 */
	val_b = ADF4360_B_COUNTER(adf4360->b) | (2<<2);

	adf4360_write_reg(adf4360, ADF4360_REG_R_COUNTER, val_r);
	adf4360_write_reg(adf4360, ADF4360_REG_CTRL, val_ctrl);
	msleep(15);
	adf4360_write_reg(adf4360, ADF4360_REG_B_COUNTER, val_b);
}

static int adf4360_probe(struct spi_device *spi)
{
	struct clk_init_data init;
	struct adf4360 *adf4360;
	const char *parent_name;
	const char *clk_name;
	struct clk *clk;

	parent_name = of_clk_get_parent_name(spi->dev.of_node, 0);
	if (!parent_name)
		return -EINVAL;

	adf4360 = devm_kzalloc(&spi->dev, sizeof(*adf4360), GFP_KERNEL);
	if (!adf4360)
		return -ENOMEM;

	adf4360->spi = spi;

	clk_name = spi->dev.of_node->name;
	of_property_read_string(spi->dev.of_node, "clock-output-names",
		&clk_name);

	init.name = clk_name;
	init.ops = &adf4360_clk_ops;
	init.flags = CLK_SET_RATE_GATE;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	adf4360->clk_hw.init = &init;

	adf4360_m2k_setup(adf4360);

	clk = devm_clk_register(&spi->dev, &adf4360->clk_hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	return of_clk_add_provider(spi->dev.of_node, of_clk_src_simple_get, clk);
}

static int adf4360_remove(struct spi_device *spi)
{
	of_clk_del_provider(spi->dev.of_node);

	return 0;
}

static const struct spi_device_id adf4360_id[] = {
	{"adf4360-9", 0},
	{}
};

static struct spi_driver adf4360_driver = {
	.driver = {
		.name	= "adf4360",
		.owner	= THIS_MODULE,
	},
	.probe		= adf4360_probe,
	.remove		= adf4360_remove,
	.id_table	= adf4360_id,
};
module_spi_driver(adf4360_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Driver for the Analog Devices ADF4360 PLL");
