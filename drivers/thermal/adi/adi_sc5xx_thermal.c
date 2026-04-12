// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices Inc. ADSP-SC5xx TMU driver
 *
 * Copyright (C) 2026 Analog Devices Inc.
 * 
 * Supports the following SoCs:
 * ADSP-SC598
 * ADSP-SC589
 * ADSP-SC594
 * ADSP-SC573
 *
 * Author: Qasim Ijaz <qasim.ijaz@analog.com>
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define DRIVER_NAME "ADI_SC5XX_THERMAL"

#define TMU_CTL             0x00 /* TMU Control Register */
#define TMU_TEMP            0x04 /* Temperature Value Register */
#define TMU_AVG             0x08 /* Averaging Register */

#define TMU_FLT_LIM_HI      0x0c /* Fault High Limit Register */
#define TMU_ALRT_LIM_HI     0x10 /* Alert High Limit Register */
#define TMU_FLT_LIM_LO      0x14 /* Fault Low Limit Register */
#define TMU_ALRT_LIM_LO     0x18 /* Alert Low Limit Register */

#define TMU_STAT            0x1c /* Status Register */
#define TMU_IMSK            0x28 /* Interrupt Mask Register */

#define TMU_GAIN            0x24 /* Gain Value Register */
#define TMU_OFFSET          0x2c /* Offset Register */

struct adi_tmu_calibration {
	u16 gain;
	u16 offset;
};

struct adi_sc5xx_tmu {
	struct device *dev;
	void __iomem *base;
	const struct adi_tmu_calibration *calibration;
	int irq_fault;
	int irq_alert;
};

static const struct adi_tmu_calibration sc573_tmu_data = {
	.gain   = 0x03F1,
	.offset = 0x0680,
};

static const struct adi_tmu_calibration sc594_tmu_data = {
	.gain   = 0x0004,
	.offset = 0x7D5A,
};

static const struct adi_tmu_calibration sc598_tmu_data = {
	.gain   = 0x0004,
	.offset = 0x7D40,
};

/* Contact Analog Devices, Inc for the current best values to use. */
static const struct adi_tmu_calibration sc589_tmu_data = {
	.gain   = 0x0,
	.offset = 0x0,
};

static inline u32 adi_tmu_read(struct adi_sc5xx_tmu *tmu, u32 offset) {
	return readl(tmu->base + offset);
}

static inline void adi_tmu_write(struct adi_sc5xx_tmu *tmu, u32 offset, u32 val) {
	writel(val, tmu->base + offset);
}

static int adi_sc5xx_thermal_probe(struct platform_device *pdev) {

	struct device *dev = &pdev->dev;
	struct adi_sc5xx_tmu *tmu;
	const struct adi_tmu_calibration *cal;
	int irq_fault;
	int irq_alert;

	tmu = devm_kzalloc(dev, sizeof(*tmu), GFP_KERNEL);
	if (!tmu)
		return -ENOMEM;

	tmu->dev = dev;

	cal = of_device_get_match_data(dev);
	if (!cal)
		return -ENODEV;
	tmu->calibration = cal;

	tmu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tmu->base))
		return PTR_ERR(tmu->base);
	
	platform_set_drvdata(pdev, tmu);
	
	adi_tmu_write(tmu, TMU_GAIN, tmu->calibration->gain);
	adi_tmu_write(tmu, TMU_OFFSET, tmu->calibration->offset);	

	irq_fault = platform_get_irq_byname(pdev, "tmu0_fault");
	irq_alert = platform_get_irq_byname_optional(pdev, "tmu0_alert");

	return 0;	
}

static void adi_sc5xx_thermal_remove(struct platform_device *pdev) {


}

static const struct of_device_id adi_sc5xx_of_match[] = {
	{ .compatible = "adi,sc598-tmu", .data = &sc598_tmu_data },	
	{ .compatible = "adi,sc589-tmu", .data = &sc589_tmu_data },
	{ .compatible = "adi,sc594-tmu", .data = &sc594_tmu_data },
	{ .compatible = "adi,sc573-tmu", .data = &sc573_tmu_data },
	{},
};
MODULE_DEVICE_TABLE(of, adi_sc5xx_of_match);

static struct platform_driver adi_sc5xx_thermal = {
	.probe = adi_sc5xx_thermal_probe, 
	.remove = adi_sc5xx_thermal_remove,
	.driver = {
		.name = DRIVER_NAME;
		.of_match_table = adi_sc5xx_thermal_of_match,
	}
};
module_platform_driver(adi_sc5xx_thermal);

MODULE_AUTHOR("Qasim Ijaz <qasim.ijaz@analog.com>");
MODULE_DESCRIPTION("Analog Devices Inc ADSP-SC5XX thermal management driver");
MODULE_LICENSE("GPL v2");
