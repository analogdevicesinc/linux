// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2024 NXP.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/nvmem-consumer.h>
#include <linux/thermal.h>

#define CTRL0			0x0

#define STAT0			0x10
#define STAT0_DRDY0_IF_MASK	BIT(16)

#define DATA0			0x20
#define DATA_NEGATIVE_MASK	BIT(15)
#define DATA_INT_MASK		GENMASK(14, 7)
#define DATA_FRAC_MASK		GENMASK(6, 0)

#define THR_CTRL01		0x30
#define THR_CTRL23		0x40

#define CTRL1			0x200
#define CTRL1_SET		0x204
#define CTRL1_CLR		0x208
#define CTRL1_EN		BIT(31)
#define CTRL1_START		BIT(30)
#define CTRL1_STOP		BIT(29)
#define CTRL1_RES_MASK		GENMASK(19, 18)
#define CTRL1_MEAS_MODE_MASK	GENMASK(25, 24)

#define PERIOD_CTRL		0x270
#define MEAS_FREQ_MASK		GENMASK(23, 0)

#define REF_DIV			0x280
#define DIV_EN			BIT(31)
#define DIV_MASK		GENMASK(23, 16)

#define PUD_ST_CTRL		0x2B0
#define PUDL_MASK		GENMASK(23, 16)

#define TRIM1			0x2E0
#define TRIM2			0x2F0

#define TMU_TEMP_LOW_LIMIT	-40000
#define TMU_TEMP_HIGH_LIMIT	125000

#define DEFAULT_TRIM1_CONFIG 0xB561BC2DU
#define DEFAULT_TRIM2_CONFIG 0x65D4U

enum measure_mode {
	SINGLE_ONESHOT_MODE,
	CONTINUOUS_MODE,
	PERIODIC_ONESHOT_MODE,
};

struct tmu_sensor {
	struct imx91_tmu *priv;
	struct thermal_zone_device *tzd;
};

struct imx91_tmu {
	void __iomem *base;
	struct clk *clk;
	struct device *dev;
	struct tmu_sensor sensors;
};

enum tmu_trip {
	TMU_TRIP_PASSIVE,
	TMU_TRIP_CRITICAL,
	TMU_TRIP_NUM,
};

static void imx91_tmu_start(struct imx91_tmu *tmu, bool start)
{
	if (start)
		writel_relaxed(CTRL1_START, tmu->base + CTRL1_SET);
	else
		writel_relaxed(CTRL1_STOP, tmu->base + CTRL1_SET);
}

static void imx91_tmu_enable(struct imx91_tmu *tmu, bool enable)
{
	if (enable)
		writel_relaxed(CTRL1_EN, tmu->base + CTRL1_SET);
	else
		writel_relaxed(CTRL1_EN, tmu->base + CTRL1_CLR);
}

static int imx91_tmu_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct tmu_sensor *sensor = thermal_zone_device_priv(tz);
	struct imx91_tmu *tmu = sensor->priv;
	u32 val;
	int ret;

	ret = readl_relaxed_poll_timeout(tmu->base + STAT0, val,
					 val & STAT0_DRDY0_IF_MASK, 1000,
					 40000);
	if (ret)
		return -EAGAIN;

	val = readl_relaxed(tmu->base + DATA0) & 0xffffU;
	*temp = (int)val * 1000LL / 64LL;
	if (*temp < TMU_TEMP_LOW_LIMIT || *temp > TMU_TEMP_HIGH_LIMIT)
		return -EAGAIN;

	return 0;
}

static struct thermal_zone_device_ops tmu_tz_ops = {
	.get_temp = imx91_tmu_get_temp,
};

static int imx91_init_from_nvmem_cells(struct imx91_tmu *tmu)
{
	struct device *dev = tmu->dev;
	int ret;
	u32 trim1, trim2;

	ret = nvmem_cell_read_u32(dev, "tmu-trim1", &trim1);
	if (ret)
		return ret;

	ret = nvmem_cell_read_u32(dev, "tmu-trim2", &trim2);
	if (ret)
		return ret;

	if (trim1 == 0 || trim2 == 0)
		return -EINVAL;

	writel_relaxed(trim1, tmu->base + TRIM1);
	writel_relaxed(trim2, tmu->base + TRIM2);

	return 0;
}

static int imx91_tmu_probe(struct platform_device *pdev)
{
	struct imx91_tmu *tmu;
	unsigned long rate;
	u32 div;
	int ret;
	int i = 0;

	tmu = devm_kzalloc(&pdev->dev, sizeof(struct imx91_tmu), GFP_KERNEL);
	if (!tmu)
		return -ENOMEM;

	tmu->dev = &pdev->dev;

	tmu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tmu->base))
		return PTR_ERR(tmu->base);

	tmu->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(tmu->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(tmu->clk),
				     "failed to get tmu clock\n");

	tmu->sensors.priv = tmu;
	tmu->sensors.tzd = devm_thermal_of_zone_register(&pdev->dev, i, &tmu->sensors,
								&tmu_tz_ops);
	if (IS_ERR(tmu->sensors.tzd)) {
		ret = PTR_ERR(tmu->sensors.tzd);
		dev_err(&pdev->dev, "failed to register thermal zone sensor: %d\n", ret);
		return ret;
	}
	platform_set_drvdata(pdev, tmu);

	ret = clk_prepare_enable(tmu->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable tmu clock: %d\n", ret);
		goto free_cooling;
	}

	/* disable the monitor during initialization */
	imx91_tmu_enable(tmu, false);
	imx91_tmu_start(tmu, false);

	ret = imx91_init_from_nvmem_cells(tmu);
	if (ret) {
		writel_relaxed(DEFAULT_TRIM1_CONFIG, tmu->base + TRIM1);
		writel_relaxed(DEFAULT_TRIM2_CONFIG, tmu->base + TRIM2);
	}

	/* The typical conv clk is 4MHz, the output freq is 'rate / (div + 1)' */
	rate = clk_get_rate(tmu->clk);
	div = (rate / 4000000) - 1;

	/* Set divider value and enable divider */
	writel_relaxed(DIV_EN | FIELD_PREP(DIV_MASK, div), tmu->base + REF_DIV);

	/* Set max power up delay: 'Tpud(ms) = 0xFF * 1000 / 4000000' */
	writel_relaxed(FIELD_PREP(PUDL_MASK, 100U), tmu->base + PUD_ST_CTRL);

	/*
	 * Set resolution mode
	 * 00b - Conversion time = 0.59325 ms
	 * 01b - Conversion time = 1.10525 ms
	 * 10b - Conversion time = 2.12925 ms
	 * 11b - Conversion time = 4.17725 ms
	 */
	writel_relaxed(FIELD_PREP(CTRL1_RES_MASK, 0x3), tmu->base + CTRL1_CLR);
	writel_relaxed(FIELD_PREP(CTRL1_RES_MASK, 0x1), tmu->base + CTRL1_SET);

	/*
	 * Set measure mode
	 * 00b - Single oneshot measurement
	 * 01b - Continuous measurement
	 * 10b - Periodic oneshot measurement
	 */
	writel_relaxed(FIELD_PREP(CTRL1_MEAS_MODE_MASK, 0x3), tmu->base + CTRL1_CLR);
	writel_relaxed(FIELD_PREP(CTRL1_MEAS_MODE_MASK, 0x1), tmu->base + CTRL1_SET);

	/*
	 * Set Periodic Measurement Frequency to 25Hz:
	 * tMEAS_FREQ = tCONV_CLK * PERIOD_CTRL[MEAS_FREQ]. ->
	 * PERIOD_CTRL(MEAS_FREQ) = (1000 / 25) / (1000 / 4000000);
	 * Where tMEAS_FREQ = Measurement period and tCONV_CLK = 1/fCONV_CLK.
	 * This field should have value greater than count corresponds
	 * to time greater than summation of conversion time, power up
	 * delay, and six times of conversion clock time.
	 * tMEAS_FREQ > (tCONV + tPUD + 6 * tCONV_CLK).
	 * tCONV is conversion time determined by CTRL1[RESOLUTION].
	 */
	writel_relaxed(FIELD_PREP(MEAS_FREQ_MASK, 0x27100), tmu->base + PERIOD_CTRL);

	/* enable the monitor */
	imx91_tmu_enable(tmu, true);
	imx91_tmu_start(tmu, true);

	return 0;

free_cooling:
	devm_thermal_of_zone_unregister(&pdev->dev, tmu->sensors.tzd);
	return ret;
}

static void imx91_tmu_remove(struct platform_device *pdev)
{
	struct imx91_tmu *tmu = platform_get_drvdata(pdev);

	/* disable tmu */
	imx91_tmu_start(tmu, false);
	imx91_tmu_enable(tmu, false);

	clk_disable_unprepare(tmu->clk);
	platform_set_drvdata(pdev, NULL);
}

static int __maybe_unused imx91_tmu_suspend(struct device *dev)
{
	struct imx91_tmu *tmu = dev_get_drvdata(dev);

	/* disable tmu */
	imx91_tmu_start(tmu, false);
	imx91_tmu_enable(tmu, false);

	clk_disable_unprepare(tmu->clk);

	return 0;
}

static int __maybe_unused imx91_tmu_resume(struct device *dev)
{
	struct imx91_tmu *tmu = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(tmu->clk);
	if (ret)
		return ret;

	imx91_tmu_enable(tmu, true);
	imx91_tmu_start(tmu, true);

	return 0;
}

static SIMPLE_DEV_PM_OPS(imx91_tmu_pm_ops,
			 imx91_tmu_suspend, imx91_tmu_resume);

static const struct of_device_id imx91_tmu_table[] = {
	{ .compatible = "fsl,imx91-tmu", },
	{ },
};
MODULE_DEVICE_TABLE(of, imx91_tmu_table);

static struct platform_driver imx91_tmu = {
	.driver = {
		.name	= "i.MX91_thermal",
		.pm	= &imx91_tmu_pm_ops,
		.of_match_table = imx91_tmu_table,
	},
	.probe = imx91_tmu_probe,
	.remove = imx91_tmu_remove,
};
module_platform_driver(imx91_tmu);

MODULE_AUTHOR("Peng Fan <peng.fan@nxp.com>");
MODULE_DESCRIPTION("i.MX91 Thermal Monitor Unit driver");
MODULE_LICENSE("GPL");
