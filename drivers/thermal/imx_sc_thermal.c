/*
 * Copyright 2017-2018 NXP.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/device_cooling.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <soc/imx8/sc/sci.h>

#include "thermal_core.h"

#define IMX_SC_TEMP_PASSIVE_COOL_DELTA	10000

struct imx_sc_sensor {
	struct thermal_zone_device *tzd;
	struct thermal_cooling_device *cdev;
	sc_rsrc_t hw_id;
	int temp_passive;
	int temp_critical;
};

struct imx_sc_tsens_device {
	u32 sensor_num;
	struct imx_sc_sensor sensor[0];
};

/* The driver support 1 passive trip point and 1 critical trip point */
enum imx_thermal_trip {
	IMX_TRIP_PASSIVE,
	IMX_TRIP_CRITICAL,
	IMX_TRIP_NUM,
};

static const sc_rsrc_t imx8qm_sensor_hw_id[] = {
	SC_R_A53, SC_R_A72, SC_R_GPU_0_PID0, SC_R_GPU_1_PID0,
	SC_R_DRC_0, SC_R_PMIC_0, SC_R_PMIC_1, SC_R_PMIC_2,
};

static const sc_rsrc_t imx8qxp_sensor_hw_id[] = {
	SC_R_SYSTEM, SC_R_DRC_0, SC_R_PMIC_0,
	SC_R_PMIC_1, SC_R_PMIC_2,
};

const int *sensor_hw_id;
sc_ipc_t tsens_ipcHandle;

static int imx_sc_tsens_get_temp(void *data, int *temp)
{
	struct imx_sc_sensor *sensor = data;
	sc_err_t sciErr;
	int16_t celsius;
	int8_t tenths;

	sciErr = sc_misc_get_temp(tsens_ipcHandle, sensor->hw_id,
			SC_C_TEMP, &celsius, &tenths);
	/*
	 * if the SS power domain is down, read temp will fail, so
	 * we can return the temp of CPU domain instead.
	 */
	if (sciErr != SC_ERR_NONE) {
		sciErr = sc_misc_get_temp(tsens_ipcHandle,
			sensor_hw_id[topology_physical_package_id(smp_processor_id())],
			SC_C_TEMP, &celsius, &tenths);
		if (sciErr != SC_ERR_NONE) {
			pr_err("read temp sensor:%d failed\n", sensor->hw_id);
			return -EINVAL;
		}
	}
	*temp = celsius * 1000 + tenths * 100;

	return 0;
}

static int imx_sc_tsens_get_trend(void *p, int trip, enum thermal_trend *trend)
{
	int trip_temp;
	struct imx_sc_sensor *sensor = p;

	if (!sensor->tzd)
		return 0;

	trip_temp = (trip == IMX_TRIP_PASSIVE) ? sensor->temp_passive :
					     sensor->temp_critical;

	if (sensor->tzd->temperature >=
		(trip_temp - IMX_SC_TEMP_PASSIVE_COOL_DELTA))
		*trend = THERMAL_TREND_RAISE_FULL;
	else
		*trend = THERMAL_TREND_DROP_FULL;

	return 0;
}

static int imx_sc_set_trip_temp(void *p, int trip,
			     int temp)
{
	struct imx_sc_sensor *sensor = p;

	if (trip == IMX_TRIP_CRITICAL)
		sensor->temp_critical = temp;

	if (trip == IMX_TRIP_PASSIVE)
		sensor->temp_passive = temp;

	return 0;
}

static const struct thermal_zone_of_device_ops imx_sc_tsens_ops = {
	.get_temp = imx_sc_tsens_get_temp,
	.get_trend = imx_sc_tsens_get_trend,
	.set_trip_temp = imx_sc_set_trip_temp,
};

static const struct of_device_id imx_sc_tsens_table[] = {
	{ .compatible = "nxp,imx8qm-sc-tsens", .data = &imx8qm_sensor_hw_id, },
	{ .compatible = "nxp,imx8qxp-sc-tsens", .data = &imx8qxp_sensor_hw_id, },
	{},
};

MODULE_DEVICE_TABLE(of, imx_sc_tsens_table);

static int imx_sc_tsens_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct imx_sc_tsens_device *tsens_dev;
	struct imx_sc_sensor *sensor;
	const struct thermal_trip *trip;
	struct thermal_zone_device *tzd;
	sc_err_t sciErr;
	uint32_t mu_id;
	u32 tsens_num;
	int ret, sensor_id;

	sciErr = sc_ipc_getMuID(&mu_id);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "Can not get the mu id: %d\n", sciErr);
		return -ENODEV;
	};

	sciErr = sc_ipc_open(&tsens_ipcHandle, mu_id);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "open mu channel failed: %d\n", sciErr);
		return -EINVAL;
	};
	sensor_hw_id = of_device_get_match_data(&pdev->dev);

	/* get the temp sensor number from device node */
	of_property_read_u32(np, "tsens-num", &tsens_num);
	if (!tsens_num) {
		dev_err(&pdev->dev, "no temp sensor number provided!\n");
		return -EINVAL;
	}

	tsens_dev = devm_kzalloc(&pdev->dev, sizeof(*tsens_dev) +
				 tsens_num * sizeof(*sensor), GFP_KERNEL);
	if (!tsens_dev)
		return -ENOMEM;

	tsens_dev->sensor_num = tsens_num;

	for (sensor_id = 0; sensor_id < tsens_num; sensor_id++) {
		sensor = &tsens_dev->sensor[sensor_id];
		sensor->hw_id = sensor_hw_id[sensor_id];
		tzd = devm_thermal_zone_of_sensor_register(&pdev->dev, sensor_id, sensor,
		 &imx_sc_tsens_ops);
		if (IS_ERR(tzd)) {
			dev_err(&pdev->dev, "failed to register temp sensor: %d\n", sensor_id);
			ret = -EINVAL;
			goto failed;
		}
		sensor->tzd = tzd;
		trip = of_thermal_get_trip_points(sensor->tzd);
		sensor->temp_passive = trip[0].temperature;
		sensor->temp_critical = trip[1].temperature;

		sensor->cdev = devfreq_cooling_register();
		if (IS_ERR(sensor->cdev)) {
			dev_err(&pdev->dev,
				"failed to register devfreq cooling device: %d\n",
				ret);
			goto failed;
		}

		ret = thermal_zone_bind_cooling_device(sensor->tzd,
			IMX_TRIP_PASSIVE,
			sensor->cdev,
			THERMAL_NO_LIMIT,
			THERMAL_NO_LIMIT,
			THERMAL_WEIGHT_DEFAULT);
		if (ret) {
			dev_err(&sensor->tzd->device,
				"binding zone %s with cdev %s failed:%d\n",
				sensor->tzd->type, sensor->cdev->type, ret);
			devfreq_cooling_unregister(sensor->cdev);
			goto failed;
		}
	}

	return 0;
failed:
	return ret;
}

static int imx_sc_tsens_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver imx_sc_tsens_driver = {
		.probe = imx_sc_tsens_probe,
		.remove = imx_sc_tsens_remove,
		.driver = {
			.name = "imx-sc-thermal",
			.of_match_table = imx_sc_tsens_table,
		},
};

module_platform_driver(imx_sc_tsens_driver);

MODULE_AUTHOR("Jacky Bai<ping.bai@nxp.com");
MODULE_DESCRIPTION("Thermal driver for NXP i.MX SoCs with system controller");
MODULE_LICENSE("GPL v2");
