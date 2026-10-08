// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devies ADSP SPORT driver
 *
 * Copyright (c) 2026 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define ADSP_SPORT_PREALLOC_SIZE	(128 * 1024)
#define ADSP_SPORT_BUFFER_BYTES_MAX	(512 * 1024)

#define ADSP_SPORT_RATES	(SNDRV_PCM_RATE_8000_192000)
#define ADSP_SPORT_FORMATS	(SNDRV_PCM_FMTBIT_S20_LE | \
				 SNDRV_PCM_FMTBIT_U20_LE | \
				 SNDRV_PCM_FMTBIT_S24_LE | \
				 SNDRV_PCM_FMTBIT_U24_LE | \
				 SNDRV_PCM_FMTBIT_S32_LE | \
				 SNDRV_PCM_FMTBIT_U32_LE)

#define SPORT_B_OFFSET		0x80
#define SPORT_CTL(_half)	(0x00 + (SPORT_B_OFFSET * (_half)))
#define SPORT_DIV(_half)	(0x04 + (SPORT_B_OFFSET * (_half)))
#define SPORT_MCTL(_half)	(0x08 + (SPORT_B_OFFSET * (_half)))
#define SPORT_CS0(_half)	(0x0c + (SPORT_B_OFFSET * (_half)))
#define SPORT_CS1(_half)	(0x10 + (SPORT_B_OFFSET * (_half)))
#define SPORT_CS2(_half)	(0x14 + (SPORT_B_OFFSET * (_half)))
#define SPORT_CS3(_half)	(0x18 + (SPORT_B_OFFSET * (_half)))
#define SPORT_ERR(_half)	(0x20 + (SPORT_B_OFFSET * (_half)))
#define SPORT_MSTAT(_half)	(0x24 + (SPORT_B_OFFSET * (_half)))
#define SPORT_CTL2(_half)	(0x28 + (SPORT_B_OFFSET * (_half)))
#define SPORT_TXPRI(_half)	(0x40 + (SPORT_B_OFFSET * (_half)))
#define SPORT_RXPRI(_half)	(0x44 + (SPORT_B_OFFSET * (_half)))
#define SPORT_TXSEC(_half)	(0x48 + (SPORT_B_OFFSET * (_half)))
#define SPORT_RXSEC(_half)	(0x4c + (SPORT_B_OFFSET * (_half)))

#define SPORT_CTL_SPENPRI	GENMASK(0, 0)
#define SPORT_CTL_DTYPE		GENMASK(2, 1)
#define SPORT_CTL_LSBF		GENMASK(3, 3)
#define SPORT_CTL_SLEN		GENMASK(8, 4)
#define SPORT_CTL_PACK		GENMASK(9, 9)
#define SPORT_CTL_ICLK		GENMASK(10, 10)
#define SPORT_CTL_OPMODE	GENMASK(11, 11)
#define SPORT_CTL_CKRE		GENMASK(12, 12)
#define SPORT_CTL_FSR		GENMASK(13, 13)
#define SPORT_CTL_IFS		GENMASK(14, 14)
#define SPORT_CTL_DIFS		GENMASK(15, 15)
#define SPORT_CTL_LFS		GENMASK(16, 16)
#define SPORT_CTL_LAFS		GENMASK(17, 17)
#define SPORT_CTL_RJUST		GENMASK(18, 18)
#define SPORT_CTL_FSED		GENMASK(19, 19)
#define SPORT_CTL_TFIEN		GENMASK(20, 20)
#define SPORT_CTL_GCLKEN	GENMASK(21, 21)
#define SPORT_CTL_SPENSEC	GENMASK(24, 24)
#define SPORT_CTL_SPTRAN	GENMASK(25, 25)
#define SPORT_CTL_DERRSEC	GENMASK(26, 26)
#define SPORT_CTL_DXSSEC	GENMASK(28, 27)
#define SPORT_CTL_DERRPRI	GENMASK(29, 29)
#define SPORT_CTL_DXSPRI	GENMASK(31, 30)

#define SPORT_DIV_CLKDIV	GENMASK(15, 0)
#define SPORT_DIV_FSDIV		GENMASK(31, 16)

#define SPORT_MCTL_MCE		GENMASK(0, 0)
#define SPORT_MCTL_MCPDE	GENMASK(2, 2)
#define SPORT_MCTL_MFD		GENMASK(7, 4)
#define SPORT_MCTL_WSIZE	GENMASK(14, 8)
#define SPORT_MCTL_WOFFSET	GENMASK(25, 16)

#define SPORT_CS0_VALUE		GENMASK(31, 0)
#define SPORT_CS1_VALUE		GENMASK(31, 0)
#define SPORT_CS2_VALUE		GENMASK(31, 0)
#define SPORT_CS3_VALUE		GENMASK(31, 0)

#define SPORT_ERR_DERRPMSK	GENMASK(0, 0)
#define SPORT_ERR_DERRSMSK	GENMASK(1, 1)
#define SPORT_ERR_FSERRMSK	GENMASK(2, 2)
#define SPORT_ERR_DERRPSTAT	GENMASK(4, 4)
#define SPORT_ERR_DERRSSTAT	GENMASK(5, 5)
#define SPORT_ERR_FSERRSTAT	GENMASK(6, 6)
#define SPORT_ERR_MSK		(SPORT_ERR_DERRPMSK | \
				 SPORT_ERR_DERRSMSK | \
				 SPORT_ERR_FSERRMSK)
#define SPORT_ERR_STAT		(SPORT_ERR_DERRPSTAT | \
				 SPORT_ERR_DERRSSTAT | \
				 SPORT_ERR_FSERRSTAT)

#define SPORT_MSTAT_CURCHAN	GENMASK(9, 0)

#define SPORT_CTL2_FSMUXSEL	GENMASK(0, 0)
#define SPORT_CTL2_CKMUXSEL	GENMASK(1, 1)
#define SPORT_CTL2_RLRE		GENMASK(3, 3)

#define SPORT_TXPRI_VALUE	GENMASK(31, 0)
#define SPORT_RXPRI_VALUE	GENMASK(31, 0)
#define SPORT_TXSEC_VALUE	GENMASK(31, 0)
#define SPORT_RXSEC_VALUE	GENMASK(31, 0)

struct adsp_sport;

struct adsp_sport_half {
	struct adsp_sport *sport;
	unsigned int id;
	unsigned int tdm_slots;
	unsigned int slot_width;
	unsigned int tx_mask;
	unsigned int rx_mask;
	unsigned int use_secondary : 1;
	unsigned int internal_bclk : 1;
	unsigned int  internal_fsync : 1;
	struct dma_chan *dma_chan;
	struct snd_dmaengine_dai_dma_data dma_data[2];
	unsigned int chan_constraint_list[2];
	struct snd_pcm_hw_constraint_list chan_constraint;
};

struct adsp_sport {
	struct device *dev;
	void __iomem *regs;
	phys_addr_t phys_base;
	struct clk *sclk;
	struct adsp_sport_half half[2];
};

static irqreturn_t adsp_sport_err_irq(int irq, void *data)
{
	struct adsp_sport_half *h = data;
	struct adsp_sport *sport = h->sport;
	unsigned int id = h->id;
	u32 err;

	err = readl(sport->regs + SPORT_ERR(id));
	if (!(err & SPORT_ERR_STAT))
		return IRQ_NONE;

	if (err & SPORT_ERR_DERRPSTAT)
		dev_err_ratelimited(sport->dev,
				    "HSPORT-%c: primary data error\n",
				    'A' + id);

	if (err & SPORT_ERR_DERRSSTAT)
		dev_err_ratelimited(sport->dev,
				    "HSPORT-%c: secondary data error\n",
				    'A' + id);

	if (err & SPORT_ERR_FSERRSTAT)
		dev_err_ratelimited(sport->dev, "HSPORT-%c: frame sync error\n",
				    'A' + id);

	/* Clear error bits */
	writel(err, sport->regs + SPORT_ERR(id));

	return IRQ_HANDLED;
}

static int adsp_sport_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct adsp_sport *sport = snd_soc_dai_get_drvdata(dai);
	struct adsp_sport_half *h = &sport->half[dai->id];
	unsigned int ctl;
	bool is_i2s = false;

	ctl = readl(sport->regs + SPORT_CTL(dai->id));

	/* Operating mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		FIELD_MODIFY(SPORT_CTL_OPMODE, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_LAFS, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_RJUST, &ctl, 0);
		is_i2s = true;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		FIELD_MODIFY(SPORT_CTL_OPMODE, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_LAFS, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_RJUST, &ctl, 0);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		FIELD_MODIFY(SPORT_CTL_OPMODE, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_LAFS, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_RJUST, &ctl, 1);
		break;
	case SND_SOC_DAIFMT_DSP_A:
		FIELD_MODIFY(SPORT_CTL_OPMODE, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_LAFS, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_RJUST, &ctl, 0);
		break;
	default:
		return -EINVAL;
	}

	/* Clock polarities */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		FIELD_MODIFY(SPORT_CTL_CKRE, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_LFS, &ctl, 0);
		break;
	case SND_SOC_DAIFMT_NB_IF:
		FIELD_MODIFY(SPORT_CTL_CKRE, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_LFS, &ctl, 1);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		FIELD_MODIFY(SPORT_CTL_CKRE, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_LFS, &ctl, 0);
		break;
	case SND_SOC_DAIFMT_IB_IF:
		FIELD_MODIFY(SPORT_CTL_CKRE, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_LFS, &ctl, 1);
		break;
	default:
		return -EINVAL;
	}

	/* I2S frames start with a falling FSYNC edge, unlike the other modes */
	if (is_i2s)
		ctl ^= SPORT_CTL_LFS;

	/* Always require frame sync */
	FIELD_MODIFY(SPORT_CTL_FSR, &ctl, 1);

	/* Always detect received frame syncs by rising or falling edges */
	FIELD_MODIFY(SPORT_CTL_FSED, &ctl, 1);

	/* Clock consumer/provider roles */
	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BP_FP:
		h->internal_bclk = 1;
		h->internal_fsync = 1;
		break;
	case SND_SOC_DAIFMT_BC_FP:
		h->internal_bclk = 0;
		h->internal_fsync = 1;
		break;
	case SND_SOC_DAIFMT_BP_FC:
		h->internal_bclk = 1;
		h->internal_fsync = 0;
		break;
	case SND_SOC_DAIFMT_BC_FC:
		h->internal_bclk = 0;
		h->internal_fsync = 0;
		break;
	default:
		return -EINVAL;
	}

	FIELD_MODIFY(SPORT_CTL_ICLK, &ctl, h->internal_bclk);
	FIELD_MODIFY(SPORT_CTL_IFS, &ctl, h->internal_fsync);

	writel(ctl, sport->regs + SPORT_CTL(dai->id));

	return 0;
}

static int adsp_sport_dai_set_tdm_slot(struct snd_soc_dai *dai,
				       unsigned int tx_mask,
				       unsigned int rx_mask, int slots,
				       int slot_width)
{
	struct adsp_sport *sport = snd_soc_dai_get_drvdata(dai);
	struct adsp_sport_half *h = &sport->half[dai->id];

	if (slots > 32)
		return -EINVAL;

	if (slot_width < 5 || slot_width > 32)
		return -EINVAL;

	h->tdm_slots = slots;
	h->slot_width = slot_width;
	h->tx_mask = tx_mask;
	h->rx_mask = rx_mask;

	return 0;
}

static unsigned int
adsp_sport_dai_pin_channels(struct snd_pcm_substream *substream,
			    struct snd_soc_dai *dai)
{
	struct adsp_sport *sport = snd_soc_dai_get_drvdata(dai);
	struct adsp_sport_half *h = &sport->half[dai->id];

	/* Stereo mode */
	if (!h->tdm_slots)
		return 2;

	/* TDM mode */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return hweight32(h->tx_mask);
	else
		return hweight32(h->rx_mask);
}

static int adsp_sport_dai_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct adsp_sport *sport = snd_soc_dai_get_drvdata(dai);
	struct adsp_sport_half *h = &sport->half[dai->id];
	unsigned int pin_chans = adsp_sport_dai_pin_channels(substream, dai);
	int ret;

	ret = clk_rate_exclusive_get(sport->sclk);
	if (ret)
		return ret;

	/* Constrain the permitted number of channels based on DAI format */
	h->chan_constraint_list[0] = pin_chans; /* Primary pin only */
	h->chan_constraint_list[1] = 2 * pin_chans; /* Primary + secondary */
	h->chan_constraint.count = 2;
	h->chan_constraint.list = h->chan_constraint_list;

	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_CHANNELS,
					 &h->chan_constraint);
	if (ret)
		return ret;

	/* In TDM mode, constrain permitted formats according to slot with */
	if (h->tdm_slots) {
		int slot_width = h->slot_width;
		snd_pcm_format_t f;
		u64 fmts = 0;

		pcm_for_each_format(f) {
			if (ADSP_SPORT_FORMATS & pcm_format_to_bits(f))
				if (snd_pcm_format_width(f) <= slot_width)
					fmts |= pcm_format_to_bits(f);
		}

		ret = snd_pcm_hw_constraint_mask64(substream->runtime,
						   SNDRV_PCM_HW_PARAM_FORMAT,
						   fmts);
		if (ret)
			return ret;
	}

	return 0;
}

static void adsp_sport_dai_shutdown(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct adsp_sport *sport = snd_soc_dai_get_drvdata(dai);

	clk_rate_exclusive_put(sport->sclk);
}

static int adsp_sport_dai_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct adsp_sport *sport = snd_soc_dai_get_drvdata(dai);
	struct adsp_sport_half *h = &sport->half[dai->id];
	unsigned int ctl;
	unsigned int mctl;
	unsigned int word_len;

	ctl = readl(sport->regs + SPORT_CTL(dai->id));

	/* Serial word length (SLEN = bits - 1) */
	word_len = h->tdm_slots ? h->slot_width : params_width(params);
	FIELD_MODIFY(SPORT_CTL_SLEN, &ctl, word_len - 1);

	/* Transmission direction (TX/RX) */
	FIELD_MODIFY(SPORT_CTL_SPTRAN, &ctl,
		     substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	writel(ctl, sport->regs + SPORT_CTL(dai->id));

	/* Multichannel (TDM) configuration */
	mctl = readl(sport->regs + SPORT_MCTL(dai->id));

	if (h->tdm_slots > 2) {
		FIELD_MODIFY(SPORT_MCTL_MCE, &mctl, 1);
		FIELD_MODIFY(SPORT_MCTL_WSIZE, &mctl, h->tdm_slots - 1);
		FIELD_MODIFY(SPORT_MCTL_MCPDE, &mctl, 1);
		FIELD_MODIFY(SPORT_MCTL_MFD, &mctl, 1);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			writel(h->tx_mask, sport->regs + SPORT_CS0(dai->id));
		else
			writel(h->rx_mask, sport->regs + SPORT_CS0(dai->id));
	} else {
		FIELD_MODIFY(SPORT_MCTL_MCE, &mctl, 0);
	}

	writel(mctl, sport->regs + SPORT_MCTL(dai->id));

	/* Program clock dividers when internally generating BCLK or FSYNC */
	if (h->internal_bclk || h->internal_fsync) {
		unsigned int slots = h->tdm_slots ? h->tdm_slots : 2;
		unsigned int rate = params_rate(params);
		unsigned int bclk = rate * slots * word_len;
		unsigned long sclk = clk_get_rate(sport->sclk);
		unsigned int div = 0;

		/* BCLK = sclk / (CLKDIV + 1) */
		if (h->internal_bclk)
			FIELD_MODIFY(SPORT_DIV_CLKDIV, &div, (sclk / bclk) - 1);

		/* FSYNC = BCLK / (FSDIV + 1) */
		if (ctl & SPORT_CTL_IFS)
			FIELD_MODIFY(SPORT_DIV_FSDIV, &div, (bclk / rate) - 1);

		writel(div, sport->regs + SPORT_DIV(dai->id));
	}

	/* Store whether to use secondary data pin, for use in dai_trigger */
	h->use_secondary = params_channels(params) >
			   adsp_sport_dai_pin_channels(substream, dai);

	return 0;
}

static int adsp_sport_dai_trigger(struct snd_pcm_substream *substream, int cmd,
				  struct snd_soc_dai *dai)
{
	struct adsp_sport *sport = snd_soc_dai_get_drvdata(dai);
	struct adsp_sport_half *h = &sport->half[dai->id];
	unsigned int ctl;

	ctl = readl(sport->regs + SPORT_CTL(dai->id));

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		FIELD_MODIFY(SPORT_CTL_SPENPRI, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_SPENSEC, &ctl, h->use_secondary);

		/* Enable error reporting, clearing any stale errors */
		writel(SPORT_ERR_MSK | SPORT_ERR_STAT,
		       sport->regs + SPORT_ERR(dai->id));

		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		FIELD_MODIFY(SPORT_CTL_SPENPRI, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_SPENSEC, &ctl, 0);

		/* Disable error reporting, clearing any stale errors */
		writel(SPORT_ERR_STAT, sport->regs + SPORT_ERR(dai->id));

		break;
	default:
		return -EINVAL;
	}

	writel(ctl, sport->regs + SPORT_CTL(dai->id));

	return 0;
}

static int adsp_sport_dai_probe(struct snd_soc_dai *dai)
{
	struct adsp_sport *sport = snd_soc_dai_get_drvdata(dai);
	struct adsp_sport_half *h = &sport->half[dai->id];
	int s;

	for_each_pcm_streams(s) {
		struct snd_dmaengine_dai_dma_data *dma_data = &h->dma_data[s];

		if (s == SNDRV_PCM_STREAM_PLAYBACK)
			dma_data->addr = sport->phys_base + SPORT_TXPRI(dai->id);
		else
			dma_data->addr = sport->phys_base + SPORT_RXPRI(dai->id);

		dma_data->addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dma_data->maxburst = 1;
	}

	snd_soc_dai_init_dma_data(dai, &h->dma_data[SNDRV_PCM_STREAM_PLAYBACK],
				  &h->dma_data[SNDRV_PCM_STREAM_CAPTURE]);

	return 0;
}

static const struct snd_soc_dai_ops adsp_sport_dai_ops = {
	.probe		= adsp_sport_dai_probe,
	.set_fmt	= adsp_sport_dai_set_fmt,
	.set_tdm_slot	= adsp_sport_dai_set_tdm_slot,
	.startup	= adsp_sport_dai_startup,
	.shutdown	= adsp_sport_dai_shutdown,
	.hw_params	= adsp_sport_dai_hw_params,
	.trigger	= adsp_sport_dai_trigger,
};

static struct snd_soc_dai_driver adsp_sport_dai_drv[] = {
	[0] = {
		.name = "HSPORT-A",
		.playback = {
			.stream_name = "A-Playback",
			.channels_min = 1,
			.channels_max = 128,
			.rates = ADSP_SPORT_RATES,
			.formats = ADSP_SPORT_FORMATS,
		},
		.capture = {
			.stream_name = "A-Capture",
			.channels_min = 1,
			.channels_max = 128,
			.rates = ADSP_SPORT_RATES,
			.formats = ADSP_SPORT_FORMATS,
		},
		.ops = &adsp_sport_dai_ops,
	},
	[1] = {
		.name = "HSPORT-B",
		.playback = {
			.stream_name = "B-Playback",
			.channels_min = 1,
			.channels_max = 128,
			.rates = ADSP_SPORT_RATES,
			.formats = ADSP_SPORT_FORMATS,
		},
		.capture = {
			.stream_name = "B-Capture",
			.channels_min = 1,
			.channels_max = 128,
			.rates = ADSP_SPORT_RATES,
			.formats = ADSP_SPORT_FORMATS,
		},
		.ops = &adsp_sport_dai_ops,
	},
};

static int adsp_sport_pcm_open(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct adsp_sport *sport = snd_soc_dai_get_drvdata(dai);
	struct adsp_sport_half *h = &sport->half[dai->id];
	struct snd_dmaengine_dai_dma_data *dma_data =
		&h->dma_data[substream->stream];
	struct dma_chan *dma_chan = h->dma_chan;
	struct snd_pcm_hardware hw = {};
	int ret;

	hw.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		  SNDRV_PCM_INFO_INTERLEAVED;
	hw.periods_min = 2;
	hw.periods_max = UINT_MAX;
	hw.period_bytes_min = 256;
	hw.period_bytes_max = dma_get_max_seg_size(h->dma_chan->device->dev);
	hw.buffer_bytes_max = ADSP_SPORT_BUFFER_BYTES_MAX;

	ret = snd_dmaengine_pcm_refine_runtime_hwparams(substream, dma_data,
							&hw, dma_chan);
	if (ret)
		return ret;

	snd_soc_set_runtime_hwparams(substream, &hw);

	return snd_dmaengine_pcm_open(substream, h->dma_chan);
}

static int adsp_sport_pcm_close(struct snd_soc_component *component,
				struct snd_pcm_substream *substream)
{
	return snd_dmaengine_pcm_close(substream);
}

static int adsp_sport_pcm_hw_params(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct adsp_sport *sport = snd_soc_dai_get_drvdata(dai);
	struct adsp_sport_half *h = &sport->half[dai->id];
	struct snd_dmaengine_dai_dma_data *dma_data =
		&h->dma_data[substream->stream];
	struct dma_slave_config config = {};
	int ret;

	ret = snd_hwparams_to_dma_slave_config(substream, params, &config);
	if (ret)
		return ret;

	snd_dmaengine_pcm_set_config_from_dai_data(substream, dma_data,
						   &config);

	ret = dmaengine_slave_config(h->dma_chan, &config);
	if (ret)
		return ret;

	return 0;
}

static int adsp_sport_pcm_trigger(struct snd_soc_component *component,
				  struct snd_pcm_substream *substream, int cmd)
{
	return snd_dmaengine_pcm_trigger(substream, cmd);
}

static snd_pcm_uframes_t
adsp_sport_pcm_pointer(struct snd_soc_component *component,
		       struct snd_pcm_substream *substream)
{
	return snd_dmaengine_pcm_pointer(substream);
}

static int adsp_sport_pcm_sync_stop(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream)
{
	return snd_dmaengine_pcm_sync_stop(substream);
}

static int adsp_sport_pcm_new(struct snd_soc_component *component,
			      struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct adsp_sport *sport = snd_soc_dai_get_drvdata(dai);
	struct adsp_sport_half *h = &sport->half[dai->id];

	rtd->pcm->info_flags |= SNDRV_PCM_INFO_HALF_DUPLEX;

	return snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV_IRAM,
					      h->dma_chan->device->dev,
					      ADSP_SPORT_PREALLOC_SIZE,
					      ADSP_SPORT_BUFFER_BYTES_MAX);
}

static const struct snd_soc_component_driver adsp_sport_component_drv = {
	.name		= "adsp-sport",
	.open		= adsp_sport_pcm_open,
	.close		= adsp_sport_pcm_close,
	.hw_params	= adsp_sport_pcm_hw_params,
	.trigger	= adsp_sport_pcm_trigger,
	.pointer	= adsp_sport_pcm_pointer,
	.pcm_construct	= adsp_sport_pcm_new, /* TODO: .pcm_new after v7.1 */
	.sync_stop	= adsp_sport_pcm_sync_stop,
};

static int adsp_sport_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adsp_sport *sport;
	struct resource *res;
	int ret;

	sport = devm_kzalloc(dev, sizeof(*sport), GFP_KERNEL);
	if (!sport)
		return -ENOMEM;

	sport->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(sport->regs))
		return PTR_ERR(sport->regs);

	sport->phys_base = res->start;
	sport->dev = dev;
	platform_set_drvdata(pdev, sport);

	sport->sclk = devm_clk_get_enabled(dev, "sclk");
	if (IS_ERR(sport->sclk))
		return PTR_ERR(sport->sclk);

	for (int i = 0; i < 2; i++) {
		struct adsp_sport_half *h = &sport->half[i];
		const char *name = i == 0 ? "hsport-a" : "hsport-b";
		int irq;

		h->id = i;
		h->sport = sport;

		h->dma_chan = devm_dma_request_chan(dev, name);
		if (IS_ERR(h->dma_chan))
			return PTR_ERR(h->dma_chan);

		irq = platform_get_irq_byname(pdev, name);
		if (irq < 0)
			return irq;

		ret = devm_request_irq(dev, irq, adsp_sport_err_irq, 0,
				       dev_name(dev), h);
		if (ret)
			return ret;
	}

	return devm_snd_soc_register_component(dev, &adsp_sport_component_drv,
					       adsp_sport_dai_drv,
					       ARRAY_SIZE(adsp_sport_dai_drv));
}

static const struct of_device_id adsp_sport_of_match[] = {
	{ .compatible = "adi,adsp-sc598-sport" },
	{},
};
MODULE_DEVICE_TABLE(of, adsp_sport_of_match);

static struct platform_driver adsp_sport_driver = {
	.probe = adsp_sport_probe,
	.driver = {
		.name = "adsp-sport",
		.of_match_table = adsp_sport_of_match,
	},
};
module_platform_driver(adsp_sport_driver);

MODULE_AUTHOR("Alvin Šipraga <alvin.sipraga@analog.com>");
MODULE_DESCRIPTION("ADI ADSP SPORT ASoC driver");
MODULE_LICENSE("GPL");
