// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Thermal monitoring unit driver for ADI SC59x series SoCs
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Author: Greg Malysa <greg.malysa@timesys.com>
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 *
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

/* Register offsets */
#define SC59X_TMU_CTL				0x00
#define SC59X_TMU_TEMP				0x04
#define SC59X_TMU_AVG				0x08
#define SC59X_TMU_FLT_LIM_HI		0x0c
#define SC59X_TMU_ALRT_LIM_HI		0x10
#define SC59X_TMU_FLT_LIM_LO		0x14
#define SC59X_TMU_ALRT_LIM_LO		0x18
#define SC59X_TMU_STATUS			0x1c
/* no 0x20 register in map */
#define SC59X_TMU_GAIN				0x24
#define SC59X_TMU_IMSK				0x28
#define SC59X_TMU_OFFSET			0x2c
/* no 0x30 register in map */
#define SC59X_TMU_CNV_BLANK			0x34
#define SC59X_TMU_REFR_CNTR			0x38

/* Register bit definitions */
#define SC59X_TMU_CTL_TMEN_FORCE	BIT(13)
#define SC59X_TMU_CTL_SCLKDIV		GENMASK(11, 4)
#define SC59X_TMU_CTL_TMEN			BIT(3)
#define SC59X_TMU_CTL_TMPU			BIT(0)

#define SC59X_TMU_AVG_ENABLE		BIT(0)

#define SC59X_TMU_STAT_FLTHI		BIT(4)
#define SC59X_TMU_STAT_ALRTHI		BIT(5)

#define SC59X_TMU_IMSK_FLTHI		BIT(0)
#define SC59X_TMU_IMSK_ALRTHI		BIT(1)

/* TMU supports an alert trip and a fault trip with programmable limits */
#define SC59X_THERMAL_TRIPS			2
#define SC59X_THERMAL_TRIP_MASK		0x03
#define SC59X_ALERT_TRIP			0
#define SC59X_FAULT_TRIP			1

/* delay in milliseconds when polling for passive cooling */
#define SC59X_PASSIVE_DELAY			1000

/* Minimum value for triggers */
#define SC59X_LIM_HI_MIN			60

/* Default temperature value (27 Celsius) */
#define SC59X_DEFAULT_TEMP          0xd80

struct sc59x_thermal_data {
	void __iomem *ioaddr;
	struct thermal_zone_device *tzdev;
	struct device *dev;
	int last_temp;
};

static int sc59x_change_mode(struct thermal_zone_device *tzdev,
	enum thermal_device_mode mode)
{
	struct sc59x_thermal_data *data = tzdev->devdata;

	//if (mode == THERMAL_DEVICE_ENABLED)
	//...todo
	//else
	//...todo

	return 0;
}

static int sc59x_get_temp(struct thermal_zone_device *tzdev, int *temp)
{
	struct sc59x_thermal_data *data = tzdev->devdata;
	int tmu_temp;

	/* data is 9.7 fixed point representing temperature in celsius
	 * linux expects integer millicelsius
	 */
	tmu_temp = (int16_t) readl(data->ioaddr + SC59X_TMU_TEMP);

	/* Check if value was read too soon after thermal event interrupt was cleared */
	while (tmu_temp == SC59X_DEFAULT_TEMP) {
		msleep_interruptible(50);
		tmu_temp = (int16_t) readl(data->ioaddr + SC59X_TMU_TEMP);
	}

	*temp = (tmu_temp * 1000) / 0x80;

	return 0;
}

static int sc59x_get_trip_type(struct thermal_zone_device *tzdev, int trip,
	enum thermal_trip_type *type)
{
	struct sc59x_thermal_data *data = tzdev->devdata;

	switch (trip) {
	case SC59X_ALERT_TRIP:
		*type = THERMAL_TRIP_PASSIVE;
		break;
	case SC59X_FAULT_TRIP:
		*type = THERMAL_TRIP_CRITICAL;
		break;
	default:
		dev_err(data->dev, "requested invalid trip type for %d\n", trip);
		return -EINVAL;
	}

	return 0;
}

static int sc59x_get_trip_temp(struct thermal_zone_device *tzdev, int trip, int *temp)
{
	struct sc59x_thermal_data *data = tzdev->devdata;
	int tmu_temp = 0;

	switch (trip) {
	case SC59X_ALERT_TRIP:
		tmu_temp = readl(data->ioaddr + SC59X_TMU_ALRT_LIM_HI);
		break;
	case SC59X_FAULT_TRIP:
		tmu_temp = readl(data->ioaddr + SC59X_TMU_FLT_LIM_HI);
		break;
	default:
		dev_err(data->dev, "requested invalid trip temp for %d\n", trip);
		return -EINVAL;
	}

	/* Convert from 9-bit two's complement to 32-bit --
	 * This shouldn't really matter, as we're only reading back the high limits right now.
	 * If we were to read back the low limits too, then we would need this. The high
	 * limits are capped at >= 60C according to the reference manual, so they're
	 * never negative
	 */
	if (tmu_temp & (1<<8))
		tmu_temp |= 0xFFFFFE00;

	/* temp limits are integer celsius */
	*temp = tmu_temp * 1000;

	return 0;
}

static int sc59x_get_crit_temp(struct thermal_zone_device *tzdev, int *temp)
{
	return sc59x_get_trip_temp(tzdev, SC59X_FAULT_TRIP, temp);
}

static int sc59x_set_trip_temp(struct thermal_zone_device *tzdev, int trip, int temp)
{
	struct sc59x_thermal_data *data = tzdev->devdata;
	int tmu_temp;

	/* temp limits are integer celsius */
	tmu_temp = temp / 1000;

	if (tmu_temp < SC59X_LIM_HI_MIN) {
		dev_err(data->dev, "trip limits req >= 60 C, tried to set %d @ %d\n",
			tmu_temp, trip);
		return -EINVAL;
	}

	switch (trip) {
	case SC59X_ALERT_TRIP:
		writel(tmu_temp, data->ioaddr + SC59X_TMU_ALRT_LIM_HI);
		break;
	case SC59X_FAULT_TRIP:
		writel(tmu_temp, data->ioaddr + SC59X_TMU_FLT_LIM_HI);
		break;
	default:
		dev_err(data->dev, "tried to set trip temp for invalid trip %d\n", trip);
		return -EINVAL;
	}

	dev_info(data->dev, "set trip %d temperature to %d C\n", trip, tmu_temp);

	return 0;
}

static struct thermal_zone_device_ops sc59x_tmu_ops = {
	.get_temp = sc59x_get_temp,
	.change_mode = sc59x_change_mode,
	.get_trip_type = sc59x_get_trip_type,
	.get_trip_temp = sc59x_get_trip_temp,
	.get_crit_temp = sc59x_get_crit_temp,
	.set_trip_temp = sc59x_set_trip_temp,
};

static irqreturn_t sc59x_thermal_irq_alert_thread(int irq, void *irqdata)
{
	struct sc59x_thermal_data *data = irqdata;
	u32 imask;
	int trip_temp;
	int tmu_temp;
	u32 recoveryCount = 0;

	sc59x_get_trip_temp(data->tzdev, SC59X_ALERT_TRIP, &trip_temp);
	sc59x_get_temp(data->tzdev, &tmu_temp);

	/* Wait until we have ten consecutive low readings
	 * (each spaced 1 second apart) -- this is debouncing the interrupts
	 * so that we don't get spammed with interrupts while the temperature
	 * recovers
	 */
	while (recoveryCount < 10) {
		msleep_interruptible(1000);
		sc59x_get_temp(data->tzdev, &tmu_temp);
		if (tmu_temp < trip_temp) {
			recoveryCount++;
		} else {
			thermal_zone_device_update(data->tzdev, THERMAL_EVENT_UNSPECIFIED);
			recoveryCount = 0;
		}
	}

	// clearing interrupts may reset temperature register contents
	writel(SC59X_TMU_STAT_ALRTHI | SC59X_TMU_STAT_FLTHI, data->ioaddr + SC59X_TMU_STATUS);

	/* Unmask the interrupt --
	 * masking/unmasking via IMASK isn't 100% necessary
	 * as the interrupts won't hit again until TMU_STATUS is cleared anyways
	 */
	imask = readl(data->ioaddr + SC59X_TMU_IMSK);
	imask &= ~SC59X_TMU_IMSK_ALRTHI;
	writel(imask, data->ioaddr + SC59X_TMU_IMSK);

	return IRQ_HANDLED;
}

static irqreturn_t sc59x_thermal_irq_alert(int irq, void *irqdata)
{
	struct sc59x_thermal_data *data = irqdata;
	u32 imask;

	/* Mask the interrupt until we drop back below the threshold temperature */
	imask = readl(data->ioaddr + SC59X_TMU_IMSK);
	imask |= SC59X_TMU_IMSK_ALRTHI;
	writel(imask, data->ioaddr + SC59X_TMU_IMSK);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t sc59x_thermal_irq_fault_thread(int irq, void *irqdata)
{
	struct sc59x_thermal_data *data = irqdata;
	u32 imask;
	int trip_temp;
	int tmu_temp;
	u32 recoveryCount = 0;

	sc59x_get_trip_temp(data->tzdev, SC59X_FAULT_TRIP, &trip_temp);
	sc59x_get_temp(data->tzdev, &tmu_temp);

	/* Wait until we have ten consecutive low readings
	 * (each spaced 1 second apart) -- this is debouncing the interrupts
	 * so that we don't get spammed with interrupts while the temperature
	 * recovers
	 */
	while (recoveryCount < 10) {
		msleep_interruptible(1000);
		sc59x_get_temp(data->tzdev, &tmu_temp);
		if (tmu_temp < trip_temp) {
			recoveryCount++;
		} else {
			thermal_zone_device_update(data->tzdev, THERMAL_EVENT_UNSPECIFIED);
			recoveryCount = 0;
		}
	}

	/* clearing interrupts may reset temperature register contents */
	writel(SC59X_TMU_STAT_FLTHI, data->ioaddr + SC59X_TMU_STATUS);

	/* Unmask the interrupt --
	 * masking/unmasking via IMASK isn't 100% necessary
	 * as the interrupts won't hit again until TMU_STATUS is cleared anyways
	 */
	imask = readl(data->ioaddr + SC59X_TMU_IMSK);
	imask &= ~SC59X_TMU_IMSK_FLTHI;
	writel(imask, data->ioaddr + SC59X_TMU_IMSK);

	return IRQ_HANDLED;
}

static irqreturn_t sc59x_thermal_irq_fault(int irq, void *irqdata)
{
	struct sc59x_thermal_data *data = irqdata;
	u32 imask;

	/* Mask the interrupt until we drop back below the threshold temperature */
	imask = readl(data->ioaddr + SC59X_TMU_IMSK);
	imask |= SC59X_TMU_IMSK_FLTHI;
	writel(imask, data->ioaddr + SC59X_TMU_IMSK);

	return IRQ_WAKE_THREAD;
}

static int sc59x_thermal_probe(struct platform_device *pdev)
{
	struct sc59x_thermal_data *data;
	struct device *dev = &pdev->dev;
	int irq;
	int ret;
	u32 gain, offset, fault, alert, blanking;
	u32 imask;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);
	data->dev = dev;

	data->ioaddr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(data->ioaddr)) {
		dev_err(dev, "Could not map thermal monitoring unit: %ld\n", PTR_ERR(data->ioaddr));
		return PTR_ERR(data->ioaddr);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "Failed to find fault IRQ: %d\n", irq);
		return irq;
	}

	ret = devm_request_threaded_irq(dev, irq, sc59x_thermal_irq_fault,
			sc59x_thermal_irq_fault_thread, 0, "sc59x_thermal_fault", data);
	if (ret < 0) {
		dev_err(dev, "Failed to request IRQ: %d\n", ret);
		return ret;
	}

	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		dev_err(dev, "Failed to find alert IRQ: %d\n", irq);
		return irq;
	}

	ret = devm_request_threaded_irq(dev, irq, sc59x_thermal_irq_alert,
			sc59x_thermal_irq_alert_thread, 0, "sc59x_thermal_alert", data);
	if (ret < 0) {
		dev_err(dev, "Failed to request IRQ: %d\n", ret);
		return ret;
	}

	fault = SC59X_LIM_HI_MIN;
	of_property_read_u32(dev->of_node, "adi,trip-fault", &fault);
	if (fault < SC59X_LIM_HI_MIN) {
		dev_info(dev, "Fault high limit clamped to 60 C, %d given\n", fault);
		fault = SC59X_LIM_HI_MIN;
	}
	writel(fault, data->ioaddr + SC59X_TMU_FLT_LIM_HI);

	alert = SC59X_LIM_HI_MIN;
	of_property_read_u32(dev->of_node, "adi,trip-alert", &alert);
	if (alert < SC59X_LIM_HI_MIN) {
		dev_info(dev, "Alert high limit clamped to 60 C, %d given\n", fault);
		alert = SC59X_LIM_HI_MIN;
	}
	writel(alert, data->ioaddr + SC59X_TMU_ALRT_LIM_HI);

	gain = 1;
	of_property_read_u32(dev->of_node, "adi,gain", &gain);
	writel(gain, data->ioaddr + SC59X_TMU_GAIN);

	offset = 0;
	of_property_read_u32(dev->of_node, "adi,offset", &offset);
	writel(offset, data->ioaddr + SC59X_TMU_OFFSET);

	if (of_property_read_bool(dev->of_node, "adi,average")) {
		writel(SC59X_TMU_AVG_ENABLE, data->ioaddr + SC59X_TMU_AVG);
		dev_info(dev, "Averaging enabled\n");
	}

	data->tzdev = thermal_zone_device_register("sc59x_thermal_zone",
		SC59X_THERMAL_TRIPS, SC59X_THERMAL_TRIP_MASK, data,
		&sc59x_tmu_ops, NULL, SC59X_PASSIVE_DELAY, 0);
	if (IS_ERR(data->tzdev)) {
		dev_err(dev, "Failed to register thermal zone device: %ld\n", PTR_ERR(data->tzdev));
		return PTR_ERR(data->tzdev);
	}

	/* Unmask interrupts */
	imask = readl(data->ioaddr + SC59X_TMU_IMSK);
	imask &= ~SC59X_TMU_IMSK_FLTHI & ~SC59X_TMU_IMSK_ALRTHI;
	writel(imask, data->ioaddr + SC59X_TMU_IMSK);

	/* Enable TMU in periodic operation */
	writel(SC59X_TMU_CTL_TMEN | SC59X_TMU_CTL_TMPU, data->ioaddr + SC59X_TMU_CTL);

	/* Read blanking interval from device tree or use minimum as default */
	blanking = 0;
	of_property_read_u32(dev->of_node, "adi,blanking-period", &blanking);
	writel(blanking, data->ioaddr + SC59X_TMU_CNV_BLANK);

	dev_info(dev, "Fault limit: %d C, Alert limit: %d C\n", fault, alert);
	dev_info(dev, "Gain 0x%x, offset %d\n", gain, offset);
	dev_info(dev, "SC59x Thermal Monitoring Unit active\n");
	return 0;
}

static int sc59x_thermal_remove(struct platform_device *pdev)
{
	struct sc59x_thermal_data *data = platform_get_drvdata(pdev);

	thermal_zone_device_unregister(data->tzdev);
	return 0;
}

static const struct of_device_id of_sc59x_tmu_match[] = {
	{ .compatible = "adi,sc59x-thermal" },
	{ }
};

MODULE_DEVICE_TABLE(of, of_sc59x_tmu_match);

static struct platform_driver sc59x_tmu_thermal = {
	.driver = {
		.name = "adi_sc59x_tmu",
		.of_match_table = of_sc59x_tmu_match
	},
	.probe = sc59x_thermal_probe,
	.remove = sc59x_thermal_remove,
};

module_platform_driver(sc59x_tmu_thermal);

MODULE_DESCRIPTION("Thermal driver for ADI SC59X Thermal Monitoring Unit");
MODULE_AUTHOR("Greg Malysa <greg.malysa@timesys.com>");
MODULE_LICENSE("GPL v2");
