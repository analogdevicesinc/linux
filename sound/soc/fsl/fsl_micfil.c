/*
 * Copyright 2018 NXP
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <sound/dmaengine_pcm.h>
#include <sound/soc.h>
#include <sound/pcm.h>

#include "fsl_micfil.h"
#include "imx-pcm.h"

#define FSL_MICFIL_RATES		SNDRV_PCM_RATE_8000_192000
#define FSL_MICFIL_FORMATS		(SNDRV_PCM_FMTBIT_S8 | \
					SNDRV_PCM_FMTBIT_S16_LE | \
					SNDRV_PCM_FMTBIT_S20_3LE | \
					SNDRV_PCM_FMTBIT_S24_LE)

struct fsl_micfil {
	struct platform_device *pdev;
	struct regmap *regmap;
	const struct fsl_micfil_soc_data *soc;
	struct clk *mclk;
	struct snd_dmaengine_dai_dma_data dma_params_rx;
	unsigned int dataline;
	bool slave_mode;
	char name[32];
	unsigned int mclk_streams;
	int quality;	/*QUALITY 2-0 bits */
};

struct fsl_micfil_soc_data {
	unsigned int fifos;
	unsigned int fifo_depth;
	unsigned int dataline;
	bool imx;
};

static struct fsl_micfil_soc_data fsl_micfil_imx8mm = {
	.imx = true,
	.fifos = 8,
	.fifo_depth = 8,
	.dataline =  0xf,
};

static const struct of_device_id fsl_micfil_dt_ids[] = {
	{ .compatible = "fsl,imx8mm-micfil", .data = &fsl_micfil_imx8mm },
	{}
};
MODULE_DEVICE_TABLE(of, fsl_micfil_dt_ids);

/* Table 5. Quality Modes
 * Medium	0 0 0
 * High		0 0 1
 * Very Low 2	1 0 0
 * Very Low 1	1 0 1
 * Very Low 0	1 1 0
 * Low		1 1 1
 */
static const char * const micfil_quality_select_texts[] = {
	"Medium", "High",
	"VLow2", "VLow1",
	"VLow0", "Low",
};

static const struct soc_enum fsl_micfil_enum[] = {
	SOC_ENUM_SINGLE(REG_MICFIL_CTRL2,
			MICFIL_CTRL2_QSEL_SHIFT,
			ARRAY_SIZE(micfil_quality_select_texts),
			micfil_quality_select_texts),
};

static int set_quality(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);
	int val = snd_soc_enum_item_to_val(e, item[0]);
	int ret;

	switch (val) {
	case 0:
	case 1:
		micfil->quality = val;
		break;
	case 2:
	case 3:
	case 4:
	case 5:
		micfil->quality = val + 2;
		break;
	default:
		dev_err(comp->dev, "Undefined value %d\n", val);
		return -EINVAL;
	}

	ret = snd_soc_component_update_bits(comp,
					    REG_MICFIL_CTRL2,
					    MICFIL_CTRL2_QSEL_MASK,
					    micfil->quality << MICFIL_CTRL2_QSEL_SHIFT);
	if (ret)
		return ret;

	return 0;
}

static const struct snd_kcontrol_new fsl_micfil_snd_controls[] = {
	SOC_SINGLE_RANGE("CH1 Gain", REG_MICFIL_OUT_CTRL,
			 MICFIL_OUTGAIN_CHX_SHIFT(0), 0x0, 0xF, 0),
	SOC_SINGLE_RANGE("CH2 Gain", REG_MICFIL_OUT_CTRL,
			 MICFIL_OUTGAIN_CHX_SHIFT(1), 0x0, 0xF, 0),
	SOC_SINGLE_RANGE("CH3 Gain", REG_MICFIL_OUT_CTRL,
			 MICFIL_OUTGAIN_CHX_SHIFT(2), 0x0, 0xF, 0),
	SOC_SINGLE_RANGE("CH4 Gain", REG_MICFIL_OUT_CTRL,
			 MICFIL_OUTGAIN_CHX_SHIFT(3), 0x0, 0xF, 0),
	SOC_SINGLE_RANGE("CH5 Gain", REG_MICFIL_OUT_CTRL,
			 MICFIL_OUTGAIN_CHX_SHIFT(4), 0x0, 0xF, 0),
	SOC_SINGLE_RANGE("CH6 Gain", REG_MICFIL_OUT_CTRL,
			 MICFIL_OUTGAIN_CHX_SHIFT(5), 0x0, 0xF, 0),
	SOC_SINGLE_RANGE("CH7 Gain", REG_MICFIL_OUT_CTRL,
			 MICFIL_OUTGAIN_CHX_SHIFT(6), 0x0, 0xF, 0),
	SOC_SINGLE_RANGE("CH8 Gain", REG_MICFIL_OUT_CTRL,
			 MICFIL_OUTGAIN_CHX_SHIFT(7), 0x0, 0xF, 0),
	SOC_ENUM_EXT("MICFIL Quality Select", fsl_micfil_enum[0],
		     snd_soc_get_enum_double, set_quality),
};

static inline unsigned int get_pdm_clk(struct fsl_micfil *micfil,
				       unsigned int rate);

static inline unsigned int get_clk_div(struct fsl_micfil *micfil,
				       unsigned int rate)
{
	u32 ctrl2_reg;
	unsigned long mclk_rate;
	unsigned int clk_div;
	unsigned int osr;

	regmap_read(micfil->regmap, REG_MICFIL_CTRL2, &ctrl2_reg);
	osr = 16 - ((ctrl2_reg & MICFIL_CTRL2_CICOSR_MASK)
		    >> MICFIL_CTRL2_CICOSR_SHIFT);

	mclk_rate = clk_get_rate(micfil->mclk);

	clk_div = mclk_rate / (get_pdm_clk(micfil, rate) * 2);

	return clk_div;
}

static inline unsigned int get_pdm_clk(struct fsl_micfil *micfil,
				       unsigned int rate)
{
	u32 ctrl2_reg;
	unsigned int qsel, osr;
	unsigned int bclk;

	regmap_read(micfil->regmap, REG_MICFIL_CTRL2, &ctrl2_reg);
	osr = 16 - ((ctrl2_reg & MICFIL_CTRL2_CICOSR_MASK)
		    >> MICFIL_CTRL2_CICOSR_SHIFT);

	regmap_read(micfil->regmap, REG_MICFIL_CTRL2, &ctrl2_reg);
	qsel = ((ctrl2_reg & MICFIL_CTRL2_QSEL_MASK)
		>> MICFIL_CTRL2_QSEL_SHIFT);

	switch (qsel) {
	case MICFIL_HIGH_QUALITY:
		bclk = rate * 8 * osr;
		break;
	case MICFIL_MEDIUM_QUALITY:
	case MICFIL_VLOW0_QUALITY:
		bclk = rate * 4 * osr;
		break;
	case MICFIL_LOW_QUALITY:
	case MICFIL_VLOW1_QUALITY:
		bclk = rate * 2 * osr;
		break;
	case MICFIL_VLOW2_QUALITY:
		bclk = rate * osr;
		break;
	default:
		bclk = 0;
		break;
	}

	return bclk;
}

/* Check if BSY_FIL flag in STAT register is set.
 * Read this flag for max 10 times, sleep 100ms
 * after each read and return error if it's not
 * cleared after 10 retries.
 */
static int fsl_micfil_bsy(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int i;
	int ret;
	u32 stat;

	for (i = 0; i < MICFIL_MAX_RETRY; i++) {
		ret = regmap_read(micfil->regmap, REG_MICFIL_STAT, &stat);
		if (ret) {
			dev_err(dev, "failed to read register %d\n",
				REG_MICFIL_STAT);
			return ret;
		}

		if (stat & MICFIL_STAT_BSY_FIL_MASK)
			usleep_range(MICFIL_SLEEP_MIN,
				     MICFIL_SLEEP_MAX);
		else
			return 0;
	}

	return -EINVAL;
}

/* The SRES is a self-negated bit which provides the CPU with the
 * capability to initialize the PDM Interface module through the
 * slave-bus interface. This bit always reads as zero, and this
 * bit is only effective when MDIS is cleared
 */
static int fsl_micfil_reset(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;

	ret = regmap_update_bits(micfil->regmap,
				 REG_MICFIL_CTRL1,
				 MICFIL_CTRL1_MDIS_MASK,
				 0);
	if (ret) {
		dev_err(dev, "failed to clear MDIS bit %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(micfil->regmap,
				 REG_MICFIL_CTRL1,
				 MICFIL_CTRL1_SRES_MASK,
				 MICFIL_CTRL1_SRES);
	if (ret) {
		dev_err(dev, "failed to reset MICFIL: %d\n", ret);
		return ret;
	}

	return 0;
}

static int fsl_micfil_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);
	struct device *dev = &micfil->pdev->dev;

	if (!micfil) {
		dev_err(dev, "micfil dai priv_data not set\n");
		return -EINVAL;
	}

	return 0;
}

static int fsl_micfil_trigger(struct snd_pcm_substream *substream, int cmd,
			      struct snd_soc_dai *dai)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);
	struct device *dev = &micfil->pdev->dev;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = fsl_micfil_reset(dev);
		if (ret) {
			dev_err(dev, "failed to soft reset\n");
			return ret;
		}

		/* DMA Interrupt Selection - DISEL bits
		 * 00 - DMA and IRQ disabled
		 * 01 - DMA req enabled
		 * 10 - IRQ enabled
		 * 11 - reserved
		 */
		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
					 MICFIL_CTRL1_DISEL_MASK,
					 (1 << MICFIL_CTRL1_DISEL_SHIFT));
		if (ret) {
			dev_err(dev, "failed to update DISEL bits\n");
			return ret;
		}

		/* Enable the module */
		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
					 MICFIL_CTRL1_PDMIEN_MASK,
					 MICFIL_CTRL1_PDMIEN);
		if (ret) {
			dev_err(dev, "failed to enable the module\n");
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* Disable the module */
		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
					 MICFIL_CTRL1_PDMIEN_MASK,
					 0);
		if (ret) {
			dev_err(dev, "failed to enable the module\n");
			return ret;
		}

		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
					 MICFIL_CTRL1_DISEL_MASK,
					 (0 << MICFIL_CTRL1_DISEL_SHIFT));
		if (ret) {
			dev_err(dev, "failed to update DISEL bits\n");
			return ret;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int fsl_micfil_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);
	unsigned int channels = params_channels(params);
	unsigned int rate = params_rate(params);
	struct device *dev = &micfil->pdev->dev;
	unsigned int clk_div;
	int ret;

	/* 1. Disable the module */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
				 MICFIL_CTRL1_PDMIEN_MASK, 0);
	if (ret) {
		dev_err(dev, "failed to disable the module\n");
		return ret;
	}

	/* enable channels */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
				 0xFF, ((1 << channels) - 1));
	if (ret) {
		dev_err(dev, "failed to enable channels %d, reg 0x%X\n", ret,
			REG_MICFIL_CTRL1);
		return ret;
	}

	/* The following bits must not change
	 * when BSY_FIL is asserted
	 */
	if (!fsl_micfil_bsy(dev)) {
		/* set CICOSR */
		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL2,
					 MICFIL_CTRL2_CICOSR_MASK,
					 MICFIL_CTRL2_OSR_DEFAULT);
		if (ret) {
			dev_err(dev, "failed to set CICOSR in reg 0x%X\n",
				REG_MICFIL_CTRL2);
			return ret;
		}

		/* Do not enable the clock if it is already enabled */
		if (!(micfil->mclk_streams & BIT(substream->stream))) {
			ret = clk_prepare_enable(micfil->mclk);
			if (ret)
				return ret;
			micfil->mclk_streams |= BIT(substream->stream);
		}

		/* set CLK_DIV */
		clk_div = get_clk_div(micfil, rate);
		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL2,
					 MICFIL_CTRL2_CLKDIV_MASK, clk_div);
		if (ret) {
			dev_err(dev, "failed to set CLKDIV in reg 0x%X\n",
				REG_MICFIL_CTRL2);
			return ret;
		}
	}

	micfil->dma_params_rx.fifo_num = channels;
	micfil->dma_params_rx.maxburst = channels * MICFIL_DMA_MAXBURST_RX;

	return 0;
}

static int fsl_micfil_hw_free(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);

	if (!micfil->slave_mode &&
	    micfil->mclk_streams & BIT(substream->stream)) {
		clk_disable_unprepare(micfil->mclk);
		micfil->mclk_streams &= ~BIT(substream->stream);
	}

	return 0;
}

static int fsl_micfil_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
				     unsigned int freq, int dir)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);
	struct device *dev = &micfil->pdev->dev;

	int ret;

	if (!freq)
		return 0;

	ret = clk_set_rate(micfil->mclk, freq);
	if (ret < 0)
		dev_err(dev, "failed to set mclk[%lu] to rate %u\n",
			clk_get_rate(micfil->mclk), freq);

	return ret;
}

static int fsl_micfil_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);

	/* DAI MODE */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		return -EINVAL;
	}

	/* DAI CLK INVERSION */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}

	micfil->slave_mode = false;

	return 0;
}

static struct snd_soc_dai_ops fsl_micfil_dai_ops = {
	.startup = fsl_micfil_startup,
	.trigger = fsl_micfil_trigger,
	.hw_params = fsl_micfil_hw_params,
	.hw_free = fsl_micfil_hw_free,
	.set_sysclk = fsl_micfil_set_dai_sysclk,
	.set_fmt = fsl_micfil_set_dai_fmt,
};

static int fsl_micfil_dai_probe(struct snd_soc_dai *cpu_dai)
{
	struct fsl_micfil *micfil = dev_get_drvdata(cpu_dai->dev);
	struct device *dev = cpu_dai->dev;
	unsigned int val;
	int ret;

	/* set qsel to medium */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL2,
				 MICFIL_CTRL2_QSEL_MASK, MICFIL_MEDIUM_QUALITY);
	if (ret) {
		dev_err(dev, "failed to set quality mode bits, reg 0x%X\n",
			REG_MICFIL_CTRL2);
		return ret;
	}

	regmap_write(micfil->regmap, REG_MICFIL_OUT_CTRL, 0x77777777);

	snd_soc_dai_init_dma_data(cpu_dai, NULL,
				  &micfil->dma_params_rx);

	/* FIFO Watermark Control - FIFOWMK*/
	val = MICFIL_FIFO_CTRL_FIFOWMK(micfil->soc->fifo_depth) - 1;
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_FIFO_CTRL,
				 MICFIL_FIFO_CTRL_FIFOWMK_MASK,
				 val);
	if (ret) {
		dev_err(dev, "failed to set FIFOWMK\n");
		return ret;
	}

	snd_soc_dai_set_drvdata(cpu_dai, micfil);

	return 0;
}

static struct snd_soc_dai_driver fsl_micfil_dai = {
	.probe = fsl_micfil_dai_probe,
	.capture = {
		.stream_name = "CPU-Capture",
		.channels_min = 1,
		.channels_max = 8,
		.rates = FSL_MICFIL_RATES,
		.formats = FSL_MICFIL_FORMATS,
	},
	.ops = &fsl_micfil_dai_ops,
};

static const struct snd_soc_component_driver fsl_micfil_component = {
	.name		= "fsl-micfil-dai",
	.controls	= fsl_micfil_snd_controls,
	.num_controls	= ARRAY_SIZE(fsl_micfil_snd_controls),
};

/* REGMAP */
static const struct reg_default fsl_micfil_reg_defaults[] = {
	{REG_MICFIL_CTRL1,		0x00000000},
	{REG_MICFIL_CTRL2,		0x00000000},
	{REG_MICFIL_STAT,		0x00000000},
	{REG_MICFIL_FIFO_CTRL,		0x00000007},
	{REG_MICFIL_FIFO_STAT,		0x00000000},
	{REG_MICFIL_DATACH0,		0x00000000},
	{REG_MICFIL_DATACH1,		0x00000000},
	{REG_MICFIL_DATACH2,		0x00000000},
	{REG_MICFIL_DATACH3,		0x00000000},
	{REG_MICFIL_DATACH4,		0x00000000},
	{REG_MICFIL_DATACH5,		0x00000000},
	{REG_MICFIL_DATACH6,		0x00000000},
	{REG_MICFIL_DATACH7,		0x00000000},
	{REG_MICFIL_DC_CTRL,		0x00000000},
	{REG_MICFIL_OUT_CTRL,		0x00000000},
	{REG_MICFIL_OUT_STAT,		0x00000000},
	{REG_MICFIL_VAD0_CTRL1,		0x00000000},
	{REG_MICFIL_VAD0_CTRL2,		0x000A0000},
	{REG_MICFIL_VAD0_STAT,		0x00000000},
	{REG_MICFIL_VAD0_SCONFIG,	0x00000000},
	{REG_MICFIL_VAD0_NCONFIG,	0x80000000},
	{REG_MICFIL_VAD0_NDATA,		0x00000000},
	{REG_MICFIL_VAD0_ZCD,		0x00000004},
};

static bool fsl_micfil_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_MICFIL_CTRL1:
	case REG_MICFIL_CTRL2:
	case REG_MICFIL_STAT:
	case REG_MICFIL_FIFO_CTRL:
	case REG_MICFIL_FIFO_STAT:
	case REG_MICFIL_DATACH0:
	case REG_MICFIL_DATACH1:
	case REG_MICFIL_DATACH2:
	case REG_MICFIL_DATACH3:
	case REG_MICFIL_DATACH4:
	case REG_MICFIL_DATACH5:
	case REG_MICFIL_DATACH6:
	case REG_MICFIL_DATACH7:
	case REG_MICFIL_DC_CTRL:
	case REG_MICFIL_OUT_CTRL:
	case REG_MICFIL_OUT_STAT:
	case REG_MICFIL_VAD0_CTRL1:
	case REG_MICFIL_VAD0_CTRL2:
	case REG_MICFIL_VAD0_STAT:
	case REG_MICFIL_VAD0_SCONFIG:
	case REG_MICFIL_VAD0_NCONFIG:
	case REG_MICFIL_VAD0_NDATA:
	case REG_MICFIL_VAD0_ZCD:
		return true;
	default:
		return false;
	}
}

static bool fsl_micfil_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_MICFIL_CTRL1:
	case REG_MICFIL_CTRL2:
	case REG_MICFIL_STAT:
	case REG_MICFIL_FIFO_CTRL:
	case REG_MICFIL_FIFO_STAT:	/* Write 1 to Clear */
	case REG_MICFIL_DC_CTRL:
	case REG_MICFIL_OUT_CTRL:
	case REG_MICFIL_OUT_STAT:	/* Write 1 to Clear */
	case REG_MICFIL_VAD0_CTRL1:
	case REG_MICFIL_VAD0_CTRL2:
	case REG_MICFIL_VAD0_STAT:	/* Write 1 to Clear */
	case REG_MICFIL_VAD0_SCONFIG:
	case REG_MICFIL_VAD0_NCONFIG:
	case REG_MICFIL_VAD0_ZCD:
		return true;
	default:
		return false;
	}
}

static bool fsl_micfil_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_MICFIL_CTRL1:
	case REG_MICFIL_STAT:
	case REG_MICFIL_DATACH0:
	case REG_MICFIL_DATACH1:
	case REG_MICFIL_DATACH2:
	case REG_MICFIL_DATACH3:
	case REG_MICFIL_DATACH4:
	case REG_MICFIL_DATACH5:
	case REG_MICFIL_DATACH6:
	case REG_MICFIL_DATACH7:
	case REG_MICFIL_VAD0_NDATA:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config fsl_micfil_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,

	.max_register = REG_MICFIL_VAD0_ZCD,
	.reg_defaults = fsl_micfil_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(fsl_micfil_reg_defaults),
	.readable_reg = fsl_micfil_readable_reg,
	.volatile_reg = fsl_micfil_volatile_reg,
	.writeable_reg = fsl_micfil_writeable_reg,
	.cache_type = REGCACHE_RBTREE,
};

/* END OF REGMAP */

static irqreturn_t micfil_isr(int irq, void *devid)
{
	struct fsl_micfil *micfil = (struct fsl_micfil *)devid;
	struct platform_device *pdev = micfil->pdev;
	u32 stat_reg;
	u32 ctrl1_reg;
	bool dma_enabled;
	int i;

	regmap_read(micfil->regmap, REG_MICFIL_STAT, &stat_reg);
	regmap_read(micfil->regmap, REG_MICFIL_CTRL1, &ctrl1_reg);
	dma_enabled = MICFIL_DMA_ENABLED(ctrl1_reg);

	if (stat_reg & MICFIL_STAT_BSY_FIL_MASK)
		dev_dbg(&pdev->dev, "isr: Decimation Filter is running\n");

	if (stat_reg & MICFIL_STAT_FIR_RDY_MASK)
		dev_dbg(&pdev->dev, "isr: FIR Filter Data ready\n");

	if (stat_reg & MICFIL_STAT_LOWFREQF_MASK) {
		dev_dbg(&pdev->dev, "isr: ipg_clk_app is too low\n");
		regmap_update_bits(micfil->regmap, REG_MICFIL_STAT,
				   MICFIL_STAT_LOWFREQF_MASK, 1);
	}

	/* Channel 0-7 Output Data Flags */
	for (i = 0; i < MICFIL_OUTPUT_CHANNELS; i++) {
		if (stat_reg & MICFIL_STAT_CHXF_MASK(i))
			dev_dbg(&pdev->dev,
				"isr: Data available in Data Channel %d\n", i);
		/* if DMA is not enabled, field must be written with 1
		 * to clear
		 */
		if (!dma_enabled)
			regmap_update_bits(micfil->regmap,
					   REG_MICFIL_STAT,
					   MICFIL_STAT_CHXF_MASK(i),
					   1);
	}

	return IRQ_HANDLED;
}

static int fsl_micfil_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *of_id;
	struct fsl_micfil *micfil;
	struct resource *res;
	void __iomem *regs;
	int irq, ret;
	unsigned long irqflag = 0;

	micfil = devm_kzalloc(&pdev->dev, sizeof(*micfil), GFP_KERNEL);
	if (!micfil)
		return -ENOMEM;

	micfil->pdev = pdev;
	strncpy(micfil->name, np->name, sizeof(micfil->name) - 1);

	of_id = of_match_device(fsl_micfil_dt_ids, &pdev->dev);
	if (!of_id || !of_id->data)
		return -EINVAL;

	micfil->soc = of_id->data;

	/* ipg_clk is used to control the registers
	 * ipg_clk_app is used to operate the filter
	 */
	micfil->mclk = devm_clk_get(&pdev->dev, "ipg_clk_app");
	if (IS_ERR(micfil->mclk)) {
		dev_err(&pdev->dev, "failed to get core clock: %ld\n",
			PTR_ERR(micfil->mclk));
		return PTR_ERR(micfil->mclk);
	}

	/* init regmap */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	micfil->regmap = devm_regmap_init_mmio_clk(&pdev->dev,
						   "ipg_clk",
						   regs,
						   &fsl_micfil_regmap_config);
	if (IS_ERR(micfil->regmap)) {
		dev_err(&pdev->dev, "failed to init MICFIL regmap: %ld\n",
			PTR_ERR(micfil->regmap));
		return PTR_ERR(micfil->regmap);
	}

	/* dataline mask for RX */
	ret = of_property_read_u32_index(np,
					 "fsl,dataline",
					 0,
					 &micfil->dataline);
	if (ret)
		micfil->dataline = 1;

	if (micfil->dataline & (~micfil->soc->dataline)) {
		dev_err(&pdev->dev, "dataline setting error, Mask is 0x%X\n",
			micfil->soc->dataline);
		return -EINVAL;
	}

	/* get IRQs */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq for node %s\n", pdev->name);
		return irq;
	}

	if (of_property_read_bool(np, "shared-interrupt"))
		irqflag = IRQF_SHARED;

	ret = devm_request_irq(&pdev->dev, irq, micfil_isr, irqflag,
			       micfil->name, micfil);
	if (ret) {
		dev_err(&pdev->dev, "failed to claim irq %u\n", irq);
		return ret;
	}

	micfil->slave_mode = false;

	micfil->dma_params_rx.chan_name = "rx";
	micfil->dma_params_rx.addr = res->start + REG_MICFIL_DATACH0;
	micfil->dma_params_rx.maxburst = MICFIL_DMA_MAXBURST_RX;

	platform_set_drvdata(pdev, micfil);

	pm_runtime_enable(&pdev->dev);

	regcache_cache_only(micfil->regmap, true);

	ret = devm_snd_soc_register_component(&pdev->dev, &fsl_micfil_component,
					      &fsl_micfil_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to register component %s\n",
			fsl_micfil_component.name);
		return ret;
	}

	if (micfil->soc->imx)
		ret = imx_pcm_platform_register(&pdev->dev);
	else
		ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);

	if (ret) {
		dev_err(&pdev->dev, "failed to pcm register\n");
		return ret;
	}

	return 0;
}

static int fsl_micfil_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int __maybe_unused fsl_micfil_runtime_suspend(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);

	regcache_cache_only(micfil->regmap, true);

	if (micfil->mclk_streams & BIT(SNDRV_PCM_STREAM_CAPTURE))
		clk_disable_unprepare(micfil->mclk);

	return 0;
}

static int __maybe_unused fsl_micfil_runtime_resume(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;

	/* enable mclk */
	if (micfil->mclk_streams & BIT(SNDRV_PCM_STREAM_CAPTURE)) {
		ret = clk_prepare_enable(micfil->mclk);
		if (ret < 0) {
			dev_err(dev, "failed to enable mclk");
			return ret;
		}
	}

	regcache_cache_only(micfil->regmap, false);
	regcache_mark_dirty(micfil->regmap);
	regcache_sync(micfil->regmap);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops fsl_micfil_pm_ops = {
	SET_RUNTIME_PM_OPS(fsl_micfil_runtime_suspend,
			   fsl_micfil_runtime_resume,
			   NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static struct platform_driver fsl_micfil_driver = {
	.probe = fsl_micfil_probe,
	.remove = fsl_micfil_remove,
	.driver = {
		.name = "fsl-micfil-dai",
		.pm = &fsl_micfil_pm_ops,
		.of_match_table = fsl_micfil_dt_ids,
	},
};
module_platform_driver(fsl_micfil_driver);

MODULE_AUTHOR("Cosmin-Gabriel Samoila <cosmin.samoila@nxp.com>");
MODULE_DESCRIPTION("NXP PDM Microphone Interface (MICFIL) driver");
MODULE_LICENSE("GPL v2");
