// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices Inc. ADSP-SC5xx SoC TMU driver
 *
 * Copyright (C) 2026 Analog Devices Inc.
 * 
 * Supports the following System-on-Chips:
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

#define TMU_CTL             0x00   /* TMU Control Register */
#define TMU_TEMP            0x04   /* Temperature Value Register */
#define TMU_AVG             0x08   /* Averaging Register */

#define TMU_FLT_LIM_HI      0x0c   /* Fault High Limit Register */
#define TMU_ALRT_LIM_HI     0x10   /* Alert High Limit Register */
#define TMU_FLT_LIM_LO      0x14   /* Fault Low Limit Register */
#define TMU_ALRT_LIM_LO     0x18   /* Alert Low Limit Register */

#define TMU_STAT            0x1c   /* Status Register */
#define TMU_IMSK            0x28   /* Interrupt Mask Register */

#define TMU_GAIN            0x24   /* Gain Value Register */
#define TMU_OFFSET          0x2c   /* Offset Register */

#define TMU_AVG_EN	    BIT(0) /* Enable TMU Averaging */

#define TMU_ALRT_LIM_HI     BIT(0) /* Alert High Limit */


#define TMU_CTL_TMEN	    BIT(3)
#define TMU_CTL_SCLKDIV	    
#define TMU_CTL_TMPU	    BIT(0)

struct adi_tmu_soc_data {
	u16 gain;
	u16 offset;
	bool has_alert_irq;
};

struct adi_sc5xx_tmu {
	struct device *dev;
	void __iomem *base;
	const struct adi_tmu_soc_data *soc_data;
	int irq_fault;
	int irq_alert;
};

static const struct adi_tmu_soc_data sc573_soc_data = {
	.gain   = 0x03F1,
	.offset = 0x0680,
	.has_alert_irq = true,
};

static const struct adi_tmu_soc_data sc594_soc_data = {
	.gain   = 0x0004,
	.offset = 0x7D5A,
	.has_alert_irq = true,
};

static const struct adi_tmu_soc_data sc598_soc_data = {
	.gain   = 0x0004,
	.offset = 0x7D40,
	.has_alert_irq = true,
};

/* Contact Analog Devices, Inc for the current best values to use. */
static const struct adi_tmu_soc_data sc589_soc_data = {
	.gain   = 0x0,
	.offset = 0x0,
	.has_alert_irq = false,
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
	const struct adi_tmu_soc_data *soc_data;
	int irq_fault;
	int irq_alert;
	int ret;

	tmu = devm_kzalloc(dev, sizeof(*tmu), GFP_KERNEL);
	if (!tmu)
		return -ENOMEM;

	tmu->dev = dev;

	soc_data = of_device_get_match_data(dev);
	if (!soc_data)
		return -ENODEV;
	tmu->soc_data = soc_data;

	tmu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tmu->base))
		return PTR_ERR(tmu->base);
	
	platform_set_drvdata(pdev, tmu);

	adi_tmu_write(tmu, TMU_GAIN, tmu->soc_data->gain);
	adi_tmu_write(tmu, TMU_OFFSET, tmu->soc_data->offset);	
	adi_tmu_write(tmu, TMU_AVG, TMU_AVG_EN);

	irq_fault = platform_get_irq_byname(pdev, "tmu0_fault");
	if (irq_fault < 0)
		return dev_err_probe(dev, irq_fault, 
				"failed to get tmu0_fault irq\n");

	tmu->irq_fault = irq_fault;
	ret = devm_request_threaded_irq(dev, tmu->irq_fault, 
		top_half_irq_fault, bottom_half_irq_fault, 
		IRQF_ONESHOT, "adi-sc5xx-tmu-fault", tmu);
	
	if (ret < 0)
		return ret;

	irq_alert = platform_get_irq_byname_optional(pdev, "tmu0_alert");
	if (irq_alert == -EPROBE_DEFER)
		return irq_alert;

	if (irq_alert < 0) {
		if (tmu->soc_data->has_alert_irq)
			return dev_err_probe(dev, irq_alert, 
				"failed to get tmu0_alert irq\n");
	}
	else {
		tmu->irq_alert = irq_alert;
		ret = devm_request_threaded_irq(dev, tmu->irq_alert,
			top_half_irq_alert, bottom_half_irq_alert,
			IRQF_ONESHOT, "adi-sc5xx-tmu-alert", tmu);
		if (ret < 0)
			return ret;
	}

	return 0;	
}

static void adi_sc5xx_thermal_remove(struct platform_device *pdev) {


}

static const struct of_device_id adi_sc5xx_thermal_of_match[] = {
	{ .compatible = "adi,sc598-tmu", .data = &sc598_soc_data },	
	{ .compatible = "adi,sc589-tmu", .data = &sc589_soc_data },
	{ .compatible = "adi,sc594-tmu", .data = &sc594_soc_data },
	{ .compatible = "adi,sc573-tmu", .data = &sc573_soc_data },
	{},
};
MODULE_DEVICE_TABLE(of, adi_sc5xx_thermal_of_match);

static struct platform_driver adi_sc5xx_thermal = {
	.probe = adi_sc5xx_thermal_probe, 
	.remove = adi_sc5xx_thermal_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = adi_sc5xx_thermal_of_match,
	}
};
module_platform_driver(adi_sc5xx_thermal);

MODULE_AUTHOR("Qasim Ijaz <qasim.ijaz@analog.com>");
MODULE_DESCRIPTION("Analog Devices Inc ADSP-SC5XX thermal management driver");
MODULE_LICENSE("GPL v2");
