/*
 *  Copyright (C) 2012, Analog Devices Inc.
 *	Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>
#include <linux/dma-mapping.h>

#include <linux/dmaengine.h>
#include <linux/of.h>

static const struct snd_pcm_hardware xlnx_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S16_LE |
		SNDRV_PCM_FMTBIT_S8,
	.channels_min = 1,
	.channels_max = UINT_MAX,
	.period_bytes_min = 1024,
	.period_bytes_max = (1 << 20) - 1,
	.periods_min	  = 2,
	.periods_max	  = UINT_MAX,
	.buffer_bytes_max = ULONG_MAX,
};

static int xlnx_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
}

struct xlnx_pcm_dma_params {
	struct device_node *of_node;
	int chan_id;
};

static bool xlnx_pcm_filter(struct dma_chan *chan, void *param)
{
	struct xlnx_pcm_dma_params *p = param;

	return chan->device->dev->of_node == p->of_node &&
		chan->chan_id == p->chan_id;
}

static int xlnx_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct device *dev = rtd->platform->dev;
	struct xlnx_pcm_dma_params *params = dev_get_drvdata(dev);
	int ret;

	ret = snd_soc_set_runtime_hwparams(substream, &xlnx_pcm_hardware);
	if (ret)
		return ret;

	return snd_dmaengine_pcm_open(substream, xlnx_pcm_filter, params);
}

static int xlnx_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;

	return snd_pcm_lib_preallocate_pages_for_all(rtd->pcm,
		SNDRV_DMA_TYPE_DEV,
		card->dev, 8 * 1024 * 1024,
		xlnx_pcm_hardware.buffer_bytes_max);
}

static int xlnx_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	return dma_mmap_coherent(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
	return remap_pfn_range(vma, vma->vm_start,
			substream->dma_buffer.addr >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static struct snd_pcm_ops xlnx_pcm_ops = {
	.open		= xlnx_pcm_open,
	.close		= snd_dmaengine_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= xlnx_pcm_hw_params,
	.hw_free	= snd_pcm_lib_free_pages,
	.trigger	= snd_dmaengine_pcm_trigger,
	.pointer	= snd_dmaengine_pcm_pointer,
	.mmap		= xlnx_pcm_mmap,
};

static struct snd_soc_platform_driver xlnx_pcm_soc_platform = {
	.pcm_new	= xlnx_pcm_new,
/*	.pcm_free	= snd_pcm_lib_preallocate_free_for_all,*/
	.ops		= &xlnx_pcm_ops,
};

static int __devinit xlnx_pcm_soc_platform_probe(struct platform_device *pdev)
{
	struct xlnx_pcm_dma_params *params;
	struct of_phandle_args dma_spec;
	int ret;

	params = devm_kzalloc(&pdev->dev, sizeof(*params), GFP_KERNEL);
	if (!params)
		return -ENOMEM;

	ret = of_parse_phandle_with_args(pdev->dev.of_node, "dma-request",
		    "#dma-cells", 0, &dma_spec);
	if (ret)
		return ret;

	params->of_node = dma_spec.np;
	params->chan_id = dma_spec.args[0];

	dev_set_drvdata(&pdev->dev, params);

	return snd_soc_register_platform(&pdev->dev, &xlnx_pcm_soc_platform);
}

static int __devexit xlnx_pcm_soc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static const struct of_device_id xilinx_pcm_of_match[] __devinitconst = {
	{ .compatible = "xilinx-pcm-audio", },
	{},
};
MODULE_DEVICE_TABLE(of, xilinx_pcm_of_match);

static struct platform_driver xlnx_pcm_driver = {
	.driver = {
			.name = "xilinx-pcm-audio",
			.owner = THIS_MODULE,
			.of_match_table = xilinx_pcm_of_match,
	},
	.probe = xlnx_pcm_soc_platform_probe,
	.remove = __devexit_p(xlnx_pcm_soc_platform_remove),
};
module_platform_driver(xlnx_pcm_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Xilinx DMA engine based audio DMA driver");
MODULE_LICENSE("GPL");
