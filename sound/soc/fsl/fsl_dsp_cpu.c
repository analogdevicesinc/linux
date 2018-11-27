// SPDX-License-Identifier: GPL-2.0+
//
// DSP Audio platform driver
//
// Copyright 2018 NXP

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>
#include <sound/core.h>
#include <sound/compress_driver.h>

#include "fsl_dsp_cpu.h"

static int dsp_audio_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *cpu_dai) {
	pm_runtime_get_sync(cpu_dai->dev);
	return 0;
}


static void dsp_audio_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *cpu_dai) {
	pm_runtime_put_sync(cpu_dai->dev);
}

static const struct snd_soc_dai_ops dsp_audio_dai_ops = {
	.startup = dsp_audio_startup,
	.shutdown = dsp_audio_shutdown,
};

static struct snd_soc_dai_driver dsp_audio_dai = {
	.name = "dsp-audio-cpu-dai",
	.compress_new = snd_soc_new_compress,
	.ops = &dsp_audio_dai_ops,
	.playback = {
		.stream_name = "Compress Playback",
		.channels_min = 1,
	},
};

static const struct snd_soc_component_driver audio_dsp_component = {
	.name           = "audio-dsp",
};

static int dsp_audio_probe(struct platform_device *pdev)
{
	struct fsl_dsp_audio *dsp_audio;
	int i, ret;
	char tmp[16];

	dsp_audio = devm_kzalloc(&pdev->dev, sizeof(*dsp_audio), GFP_KERNEL);
	if (dsp_audio == NULL)
		return -ENOMEM;

	dev_dbg(&pdev->dev, "probing DSP device....\n");

	/* intialise sof device */
	dev_set_drvdata(&pdev->dev, dsp_audio);

	/* No error out for old DTB cases but only mark the clock NULL */
	dsp_audio->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (IS_ERR(dsp_audio->bus_clk)) {
		dev_err(&pdev->dev, "failed to get bus clock: %ld\n",
				PTR_ERR(dsp_audio->bus_clk));
		dsp_audio->bus_clk = NULL;
	}

	dsp_audio->m_clk = devm_clk_get(&pdev->dev, "mclk");
	if (IS_ERR(dsp_audio->m_clk)) {
		dev_err(&pdev->dev, "failed to get m clock: %ld\n",
				PTR_ERR(dsp_audio->m_clk));
		dsp_audio->m_clk = NULL;
	}

	dsp_audio->asrc_mem_clk = devm_clk_get(&pdev->dev, "mem");
	if (IS_ERR(dsp_audio->asrc_mem_clk)) {
		dev_err(&pdev->dev, "failed to get mem clock\n");
		dsp_audio->asrc_mem_clk = NULL;
	}

	dsp_audio->asrc_ipg_clk = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(dsp_audio->asrc_ipg_clk)) {
		dev_err(&pdev->dev, "failed to get ipg clock\n");
		dsp_audio->asrc_ipg_clk = NULL;
	}

	for (i = 0; i < ASRC_CLK_MAX_NUM; i++) {
		sprintf(tmp, "asrck_%x", i);
		dsp_audio->asrck_clk[i] = devm_clk_get(&pdev->dev, tmp);
		if (IS_ERR(dsp_audio->asrck_clk[i])) {
			dev_err(&pdev->dev, "failed to get %s clock\n", tmp);
			dsp_audio->asrck_clk[i] = NULL;
		}
	}

	pm_runtime_enable(&pdev->dev);

	/* now register audio DSP platform driver */
	ret = snd_soc_register_component(&pdev->dev, &audio_dsp_component,
			&dsp_audio_dai, 1);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"error: failed to register DSP DAI driver %d\n", ret);
		goto err;
	}

        return 0;

err:
	return ret;
}

static int dsp_audio_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM
static int dsp_runtime_resume(struct device *dev)
{
	struct fsl_dsp_audio *dsp_audio = dev_get_drvdata(dev);
	int i, ret;

	ret = clk_prepare_enable(dsp_audio->bus_clk);
	if (ret) {
		dev_err(dev, "failed to enable bus clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(dsp_audio->m_clk);
	if (ret) {
		dev_err(dev, "failed to enable m clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(dsp_audio->asrc_mem_clk);
	if (ret < 0)
		dev_err(dev, "Failed to enable asrc_mem_clk ret = %d\n", ret);

	ret = clk_prepare_enable(dsp_audio->asrc_ipg_clk);
	if (ret < 0)
		dev_err(dev, "Failed to enable asrc_ipg_clk ret = %d\n", ret);

	for (i = 0; i < ASRC_CLK_MAX_NUM; i++) {
		ret = clk_prepare_enable(dsp_audio->asrck_clk[i]);
		if (ret < 0)
			dev_err(dev, "failed to prepare arc clk %d\n", i);
	}

	return ret;
}

static int dsp_runtime_suspend(struct device *dev)
{
	int i;
	struct fsl_dsp_audio *dsp_audio = dev_get_drvdata(dev);

	for (i = 0; i < ASRC_CLK_MAX_NUM; i++)
		clk_disable_unprepare(dsp_audio->asrck_clk[i]);

	clk_disable_unprepare(dsp_audio->asrc_ipg_clk);
	clk_disable_unprepare(dsp_audio->asrc_mem_clk);

	clk_disable_unprepare(dsp_audio->m_clk);
	clk_disable_unprepare(dsp_audio->bus_clk);

	return 0;
}
#endif

static const struct dev_pm_ops dsp_pm_ops = {
	SET_RUNTIME_PM_OPS(dsp_runtime_suspend,
			   dsp_runtime_resume,
			   NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static const struct of_device_id dsp_audio_ids[] = {
	{ .compatible = "fsl,dsp-audio"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dsp_audio_ids);

static struct platform_driver dsp_audio_driver = {
	.driver = {
		.name = "dsp-audio",
		.of_match_table = dsp_audio_ids,
		.pm = &dsp_pm_ops,
	},
	.probe = dsp_audio_probe,
	.remove = dsp_audio_remove,
};
module_platform_driver(dsp_audio_driver);
