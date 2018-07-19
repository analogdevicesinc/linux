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

#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include "fsl_micfil.h"
#include "imx-pcm.h"

#define FSL_MICFIL_RATES		SNDRV_PCM_RATE_8000_48000
#define FSL_MICFIL_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE)

struct fsl_micfil {
	struct platform_device *pdev;
	struct regmap *regmap;
	const struct fsl_micfil_soc_data *soc;
	struct clk *mclk;
	struct snd_dmaengine_dai_dma_data dma_params_rx;
	struct kobject *hwvad_kobject;
	unsigned int channels;
	unsigned int dataline;
	char name[32];
	unsigned int mclk_streams;
	int quality;	/*QUALITY 2-0 bits */
	bool slave_mode;
	atomic_t state;
	atomic_t voice_detected;
	atomic_t init_hwvad_done;
};

struct fsl_micfil_soc_data {
	unsigned int fifos;
	unsigned int fifo_depth;
	unsigned int dataline;
	bool imx;
};

static char *envp[] = {
		"EVENT=PDM_VOICE_DETECT",
		NULL,
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

static int disable_hwvad(struct device *dev);

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
			usleep_range(MICFIL_SLEEP_MIN, MICFIL_SLEEP_MAX);
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

/* enable/disable hwvad interrupts */
static int configure_hwvad_interrupts(struct device *dev,
				      int enable)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;
	u32 vadie_reg = enable ? MICFIL_VAD0_CTRL1_IE : 0;
	u32 vaderie_reg = enable ? MICFIL_VAD0_CTRL1_ERIE : 0;

	/* Voice Activity Detector Interruption Enable */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_IE_MASK,
				 vadie_reg);
	if (ret) {
		dev_err(dev,
			"Failed to set/clear VADIE in CTRL1_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* Voice Activity Detector Error Interruption Enable */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_ERIE_MASK,
				 vaderie_reg);
	if (ret) {
		dev_err(dev,
			"Failed to set/clear VADERIE in CTRL1_VAD0 [%d]\n",
			ret);
		return ret;
	}

	return 0;
}

static int init_hwvad_internal_filters(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;

	/* Voice Activity Detector Internal Filters Initialization*/
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_ST10_MASK,
				 MICFIL_VAD0_CTRL1_ST10);
	if (ret) {
		dev_err(dev,
			"Failed to set VADST10 in CTRL1_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* sleep for 100ms - it should be enough for bit to stay
	 * pulsed for more than 2 cycles
	 */
	usleep_range(MICFIL_SLEEP_MIN, MICFIL_SLEEP_MAX);

	/* Voice Activity Detector Enabled */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_ST10_MASK,
				 0);
	if (ret) {
		dev_err(dev,
			"Failed to clear VADST10 in CTRL1_VAD0 [%d]\n",
			ret);
		return ret;
	}
	return 0;
}

/* Zero-Crossing Detector Initialization
 * Optionally a Zero-Crossing Detection block (ZCD) could
 * be enabled to avoid low energy voiced speech be missed,
 * improving the voice detection performance.
 * See Section 8.4.3
 */
static int __maybe_unused init_zcd(struct device *dev,
				   int threshold,
				   int mode)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;

	if (mode == MICFIL_HWVAD_ZCD_AUTO) {
		/* enable auto-threshold */
		auto_threshold = MICFIL_VAD0_ZCD_ZCDAUT;

		/* Zero-Crossing Detector Adjustment */
		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_ZCD,
					 MICFIL_VAD0_ZCD_ZCDADJ_MASK,
					 MICFIL_HWVAD_ZCDADJ);
		if (ret) {
			dev_err(dev,
				"Failed to set ZCDADJ in ZCD_VAD0 [%d]\n",
				ret);
			return ret;
		}
	}

	/* Zero-Crossing Detector Threshold */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_ZCD,
				 MICFIL_VAD0_ZCD_ZCDTH_MASK,
				 MICFIL_VAD0_ZCD_ZCDTH(threshold));
	if (ret) {
		dev_err(dev, "Failed to set ZCDTH in ZCD_VAD0 [%d]\n", ret);
		return ret;
	}

	/* Zero-Crossing Detector AND Behavior */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_ZCD,
				 MICFIL_VAD0_ZCD_ZCDAND_MASK,
				 MICFIL_HWVAD_ZCDAND);
	if (ret) {
		dev_err(dev, "Failed to set ZCDAND in ZCD_VAD0 [%d]\n", ret);
		return ret;
	}

	/* Zero-Crossing Detector Automatic Threshold */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_ZCD,
				 MICFIL_VAD0_ZCD_ZCDAUT_MASK,
				 auto_threshold);
	if (ret) {
		dev_err(dev,
			"Failed to set/clear ZCDAUT in ZCD_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* Zero-Crossing Detector Enable */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_ZCD,
				 MICFIL_VAD0_ZCD_ZCDEN_MASK,
				 MICFIL_VAD0_ZCD_ZCDEN);
	if (ret) {
		dev_err(dev, "Failed to set ZCDEN in ZCD_VAD0 [%d]\n", ret);
		return ret;
	}

	return 0;
}

/* Configuration done only in energy-based initialization mode */
static int init_hwvad_energy_mode(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret, i;
	u32 stat;
	u32 flag;

	dev_info(dev, "Energy-based mode initialization\n");

	/* Voice Activity Detector Reset */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_RST_SHIFT,
				 MICFIL_VAD0_CTRL1_RST);
	if (ret) {
		dev_err(dev, "Failed to set VADRST in CTRL1_VAD0 [%d]\n", ret);
		return ret;
	}

	/* Voice Activity Detector Enabled */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_EN_MASK,
				 MICFIL_VAD0_CTRL1_EN);
	if (ret) {
		dev_err(dev, "Failed to set VADEN in CTRL1_VAD0 [%d]\n", ret);
		return ret;
	}

	/* it would be a good idea to wait some time before VADEN
	 * is set
	 */
	usleep_range(5 * MICFIL_SLEEP_MIN, 5 * MICFIL_SLEEP_MAX);

	/* Enable Interrupts */
	ret = configure_hwvad_interrupts(dev, 1);

	/* Initialize Zero Crossing Detector */
	ret = init_zcd(dev);
	if (ret)
		return ret;

	/* Enable MICFIL module */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
				 MICFIL_CTRL1_PDMIEN_MASK,
				 MICFIL_CTRL1_PDMIEN);
	if (ret) {
		dev_err(dev, "failed to enable the module\n");
		return ret;
	}

	/* Wait for INITF to be asserted */
	for (i = 0; i < MICFIL_MAX_RETRY; i++) {
		ret = regmap_read(micfil->regmap, REG_MICFIL_VAD0_STAT, &stat);
		if (ret) {
			dev_err(dev, "failed to read register %d\n",
				REG_MICFIL_VAD0_STAT);
			return ret;
		}

		flag = (stat & MICFIL_VAD0_STAT_INITF_MASK);
		if (flag == 0)
			break;

		usleep_range(MICFIL_SLEEP_MIN, MICFIL_SLEEP_MAX);
	}

	if (i == MICFIL_MAX_RETRY) {
		dev_err(dev, "initf not asserted. Failed to init hwvad\n");
		return -EBUSY;
	}

	/* Initialize Internal Filters */
	ret = init_hwvad_internal_filters(dev);
	if (ret)
		return ret;

	return ret;
}

/* Configuration done only in envelope-based initialization mode */
static int init_hwvad_envelope_mode(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret, i;
	u32 stat;
	u32 flag;

	dev_info(dev, "Envelope-based mode initialization\n");

	/* Frame energy disable */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL2,
				 MICFIL_VAD0_CTRL2_FRENDIS_MASK,
				 MICFIL_VAD0_CTRL2_FRENDIS);
	if (ret) {
		dev_err(dev, "Failed to set FRENDIS in CTRL2_VAD0 [%d]\n", ret);
		return ret;
	}

	/* Enable pre-filter Noise & Signal */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL2,
				 MICFIL_VAD0_CTRL2_PREFEN_MASK,
				 MICFIL_VAD0_CTRL2_PREFEN);
	if (ret) {
		dev_err(dev, "Failed to set PREFEN in CTRL2_VAD0 [%d]\n", ret);
		return ret;
	}

	/* Enable Signal Filter */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_SCONFIG,
				 MICFIL_VAD0_SCONFIG_SFILEN_MASK,
				 MICFIL_VAD0_SCONFIG_SFILEN);
	if (ret) {
		dev_err(dev,
			"Failed to set SFILEN in SCONFIG_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* Signal Maximum Enable */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_SCONFIG,
				 MICFIL_VAD0_SCONFIG_SMAXEN_MASK,
				 MICFIL_VAD0_SCONFIG_SMAXEN);
	if (ret) {
		dev_err(dev,
			"Failed to set SMAXEN in SCONFIG_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* Allways enable noise filter, not based on voice activity
	 * information
	 */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_NCONFIG,
				 MICFIL_VAD0_NCONFIG_NFILAUT_MASK,
				 0);
	if (ret) {
		dev_err(dev,
			"Failed to set NFILAUT in NCONFIG_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* Noise Minimum Enable */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_NCONFIG,
				 MICFIL_VAD0_NCONFIG_NMINEN_MASK,
				 MICFIL_VAD0_NCONFIG_NMINEN);
	if (ret) {
		dev_err(dev,
			"Failed to set NMINEN in NCONFIG_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* Noise Decimation Enable */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_NCONFIG,
				 MICFIL_VAD0_NCONFIG_NDECEN_MASK,
				 MICFIL_VAD0_NCONFIG_NDECEN);
	if (ret) {
		dev_err(dev,
			"Failed to set NDECEN in NCONFIG_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* Voice Activity Detector Reset */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_RST_SHIFT,
				 MICFIL_VAD0_CTRL1_RST);
	if (ret) {
		dev_err(dev, "Failed to set VADRST in CTRL1_VAD0 [%d]\n", ret);
		return ret;
	}

	/* Initialize Zero Crossing Detector */
	ret = init_zcd(dev);
	if (ret)
		return ret;

	/* Voice Activity Detector Enabled */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_EN_MASK,
				 MICFIL_VAD0_CTRL1_EN);
	if (ret) {
		dev_err(dev, "Failed to set VADEN in CTRL1_VAD0 [%d]\n", ret);
		return ret;
	}

	/* Enable MICFIL module */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
				 MICFIL_CTRL1_PDMIEN_MASK,
				 MICFIL_CTRL1_PDMIEN);
	if (ret) {
		dev_err(dev, "failed to enable the module\n");
		return ret;
	}

	/* it would be a good idea to wait some time before VADEN
	 * is set
	 */
	usleep_range(3 * MICFIL_SLEEP_MIN, 3 * MICFIL_SLEEP_MAX);

	/* Wait for INITF to be asserted */
	for (i = 0; i < MICFIL_MAX_RETRY; i++) {
		ret = regmap_read(micfil->regmap, REG_MICFIL_VAD0_STAT, &stat);
		if (ret) {
			dev_err(dev, "failed to read register %d\n",
				REG_MICFIL_VAD0_STAT);
			return ret;
		}

		flag = (stat & MICFIL_VAD0_STAT_INITF_MASK);
		if (flag == 0)
			break;

		usleep_range(MICFIL_SLEEP_MIN, MICFIL_SLEEP_MAX);
	}

	if (i == MICFIL_MAX_RETRY) {
		dev_err(dev, "initf not asserted. Failed to init hwvad\n");
		return -EBUSY;
	}

	/* Initialize Internal Filters */
	ret = init_hwvad_internal_filters(dev);
	if (ret)
		return ret;

	/* Enable interrupts */
	ret = configure_hwvad_interrupts(dev, 1);
	if (ret)
		return ret;

	return ret;
}

/* Hardware Voice Active Detection: The HWVAD takes data from the input
 * of a selected PDM microphone to detect if there is any
 * voice activity. When a voice activity is detected, an interrupt could
 * be delivered to the system. Initialization in section 8.4:
 * Can work in two modes:
 *  -> Eneveope-based mode (section 8.4.1)
 *  -> Energy-based mode (section 8.4.2)
 *
 * It is important to remark that the HWVAD detector could be enabled
 * or reset only when the MICFIL isn't running i.e. when the BSY_FIL
 * bit in STAT register is cleared
 */
static int __maybe_unused init_hwvad(struct device *dev, int mode)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;

	ret = fsl_micfil_bsy(dev);
	if (ret) {
		dev_err(dev,
			"Hardware Voice Active Detection initialization fail. BSY_FIL flag is set [%d]\n",
			ret);
		return ret;
	}

	/* configure CIC OSR in VADCICOSR */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_CICOSR_MASK,
				 MICFIL_CTRL2_OSR_DEFAULT);
	if (ret) {
		dev_err(dev, "Failed to set CICOSR in CTRL1_VAD0i [%d]\n", ret);
		return ret;
	}

	/* configure source channel in VADCHSEL */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_CHSEL_MASK,
				 MICFIL_VAD0_CTRL1_CHSEL(micfil->channels));
	if (ret) {
		dev_err(dev, "Failed to set CHSEL in CTRL1_VAD0 [%d]\n", ret);
		return ret;
	}

	/* configure detector frame time VADFRAMET */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL2,
				 MICFIL_VAD0_CTRL2_FRAMET_MASK,
				 MICFIL_VAD0_CTRL2_FRAMET(MICFIL_HWVAD_FRAMET_DEFAULT));
	if (ret) {
		dev_err(dev, "Failed to set FRAMET in CTRL2_VAD0 [%d]\n", ret);
		return ret;
	}

	/* configure initialization time in VADINITT */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_INITT_MASK,
				 MICFIL_VAD0_CTRL1_INITT(MICFIL_HWVAD_INIT_FRAMES));
	if (ret) {
		dev_err(dev, "Failed to set INITT in CTRL1_VAD0 [%d]\n", ret);
		return ret;
	}

	/* configure input gain in VADINPGAIN */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL2,
				 MICFIL_VAD0_CTRL2_INPGAIN_MASK,
				 MICFIL_VAD0_CTRL2_INPGAIN(MICFIL_HWVAD_INPGAIN));
	if (ret) {
		dev_err(dev, "Failed to set INPGAIN in CTRL2_VAD0 [%d]\n", ret);
		return ret;
	}

	/* configure sound gain in SGAIN */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_SCONFIG,
				 MICFIL_VAD0_SCONFIG_SGAIN_MASK,
				 MICFIL_VAD0_SCONFIG_SGAIN(MICFIL_HWVAD_SGAIN));
	if (ret) {
		dev_err(dev, "Failed to set SGAIN in SCONFIG_VAD0 [%d]\n", ret);
		return ret;
	}

	/* configure sound gain in SGAIN */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_NCONFIG,
				 MICFIL_VAD0_NCONFIG_NGAIN_MASK,
				 MICFIL_VAD0_NCONFIG_NGAIN(MICFIL_HWVAD_NGAIN));
	if (ret) {
		dev_err(dev, "Failed to set NGAIN in NCONFIG_VAD0 [%d]\n", ret);
		return ret;
	}

	/* configure or clear the VADNFILADJ based on mode */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_NCONFIG,
				 MICFIL_VAD0_NCONFIG_NFILADJ_MASK,
				 MICFIL_VAD0_NCONFIG_NFILADJ(MICFIL_HWVAD_NFILADJ));
	if (ret) {
		dev_err(dev,
			"Failed to set VADNFILADJ in NCONFIG_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* enable the high-pass filter in VADHPF */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL2,
				 MICFIL_VAD0_CTRL2_HPF_MASK,
				 MICFIL_VAD0_CTRL2_HPF(MICFIL_HWVAD_HPF_BYPASS));
	if (ret) {
		dev_err(dev, "Failed to set HPF in CTRL2_VAD0 [%d]\n", ret);
		return ret;
	}

	/* envelope-based specific initialization */
	if (mode == MICFIL_HWVAD_ENVELOPE_MODE) {
		ret = init_hwvad_envelope_mode(dev);
		if (ret)
			return ret;
	} else {
		ret = init_hwvad_energy_mode(dev);
		if (ret)
			return ret;
	}

	return 0;
}

static int fsl_micfil_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);
	struct device *dev = &micfil->pdev->dev;
	int state;

	state = atomic_read(&micfil->state);
	if (state != RECORDING_OFF_HWVAD_OFF) {
		dev_err(dev, "Cannot record while recording or hwvad is on\n");
		return -EPERM;
	}

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
	int old_state;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* mark as stream open. Not allowed:
		 *   - two paralel recordings
		 *   - hwvad enabled while recording
		 */
		old_state = atomic_cmpxchg(&micfil->state,
					   RECORDING_OFF_HWVAD_OFF,
					   RECORDING_ON_HWVAD_OFF);
		if (old_state != RECORDING_OFF_HWVAD_OFF) {
			dev_err(dev, "Another record or hwvad is on\n");
			return -EBUSY;
		}

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

		/* clear the stream open flag */
		atomic_set(&micfil->state, RECORDING_OFF_HWVAD_OFF);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int fsl_set_clock_params(struct device *dev, unsigned int rate)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	unsigned int clk_div;
	int ret;

	if (fsl_micfil_bsy(dev))
		return -EBUSY;

	/* set CICOSR */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL2,
				 MICFIL_CTRL2_CICOSR_MASK,
				 MICFIL_CTRL2_OSR_DEFAULT);
	if (ret) {
		dev_err(dev, "failed to set CICOSR in reg 0x%X\n",
			REG_MICFIL_CTRL2);
		return ret;
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

	ret = clk_prepare_enable(micfil->mclk);
	if (ret)
		return ret;

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

	ret = fsl_set_clock_params(dev, rate);
	if (ret)
		return ret;

	micfil->dma_params_rx.fifo_num = channels;
	micfil->dma_params_rx.maxburst = channels * MICFIL_DMA_MAXBURST_RX;
	micfil->channels = channels;

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
	case REG_MICFIL_STAT:		/* Write 1 to Clear */
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
	case REG_MICFIL_CTRL2:
	case REG_MICFIL_STAT:
	case REG_MICFIL_DATACH0:
	case REG_MICFIL_DATACH1:
	case REG_MICFIL_DATACH2:
	case REG_MICFIL_DATACH3:
	case REG_MICFIL_DATACH4:
	case REG_MICFIL_DATACH5:
	case REG_MICFIL_DATACH6:
	case REG_MICFIL_DATACH7:
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

static void voice_detected_irq(struct fsl_micfil *micfil)
{
	struct device *dev = &micfil->pdev->dev;
	int ret;

	/* disable hwvad */
	ret = disable_hwvad(dev);
	if (ret)
		dev_err(dev, "Failed to disable HWVAD module\n");

	/* disable hwvad interrupts */
	ret = configure_hwvad_interrupts(dev, 0);
	if (ret)
		dev_err(dev, "Failed to disable interrupts\n");

	/* notify userspace that voice was detected */
	kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
}

static irqreturn_t hwvad_isr(int irq, void *devid)
{
	struct fsl_micfil *micfil = (struct fsl_micfil *)devid;
	struct device *dev = &micfil->pdev->dev;
	u32 vad0_reg;
	int old_flag;

	regmap_read(micfil->regmap, REG_MICFIL_VAD0_STAT, &vad0_reg);

	old_flag = atomic_cmpxchg(&micfil->voice_detected, 0, 1);
	if ((vad0_reg & MICFIL_VAD0_STAT_IF_MASK) && !old_flag) {
		dev_info(dev, "Detected voice\n");

		/* Write 1 to clear */
		regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_STAT,
				   MICFIL_VAD0_STAT_IF_MASK,
				   MICFIL_VAD0_STAT_IF);

		voice_detected_irq(micfil);
	}

	return IRQ_HANDLED;
}

static irqreturn_t micfil_isr(int irq, void *devid)
{
	struct fsl_micfil *micfil = (struct fsl_micfil *)devid;
	struct platform_device *pdev = micfil->pdev;
	irqreturn_t ret = IRQ_HANDLED;
	u32 stat_reg;
	u32 ctrl1_reg;
	u32 vad0_reg;
	bool dma_enabled;
	int i;

	regmap_read(micfil->regmap, REG_MICFIL_STAT, &stat_reg);
	regmap_read(micfil->regmap, REG_MICFIL_CTRL1, &ctrl1_reg);
	regmap_read(micfil->regmap, REG_MICFIL_VAD0_STAT, &vad0_reg);
	dma_enabled = MICFIL_DMA_ENABLED(ctrl1_reg);

	if (vad0_reg & MICFIL_VAD0_STAT_IF_MASK)
		ret = IRQ_WAKE_THREAD;

	if (vad0_reg & MICFIL_VAD0_STAT_EF_MASK)
		dev_dbg(&pdev->dev, "isr: voice activity event detected\n");

	if (vad0_reg & MICFIL_VAD0_STAT_INSATF_MASK)
		dev_dbg(&pdev->dev, "isr: voice activity input overflow/underflow detected\n");

	if (vad0_reg & MICFIL_VAD0_STAT_INITF_MASK)
		dev_dbg(&pdev->dev, "isr: voice activity dectector is initializing\n");

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

	return ret;
}

static int fsl_set_clock_params(struct device *, unsigned int);

static int enable_hwvad(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;
	int old_state;

	/* go further with enablement only if both recording and
	 * hwvad are not on
	 */
	old_state = atomic_cmpxchg(&micfil->state,
				   RECORDING_OFF_HWVAD_OFF,
				   RECORDING_OFF_HWVAD_ON);
	if (old_state != RECORDING_OFF_HWVAD_OFF) {
		dev_err(dev, "Another record or hwvad is on\n");
		return -EBUSY;
	}

	/* This is required because if an arecord was done,
	 * suspend function will mark regmap as cache only
	 * and reads/writes in volatile regs will fail
	 */
	regcache_mark_dirty(micfil->regmap);
	regcache_sync(micfil->regmap);
	regcache_cache_only(micfil->regmap, false);

	/* clear voice detected flag */
	atomic_set(&micfil->voice_detected, 0);

	ret = fsl_set_clock_params(dev, MICFIL_DEFAULT_RATE);
	if (ret)
		goto enable_err;

	ret = fsl_micfil_reset(dev);
	if (ret)
		return ret;

	/* Initialize Hardware Voice Activity */
	ret = init_hwvad(dev, MICFIL_HWVAD_ENVELOPE_MODE);
	if (ret)
		goto enable_err;

	return 0;

enable_err:
	atomic_cmpxchg(&micfil->state, RECORDING_OFF_HWVAD_ON, RECORDING_OFF_HWVAD_OFF);
	return ret;
}

static int disable_hwvad(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret = 0;
	int old_state;

	old_state = atomic_read(&micfil->state);

	if (old_state == RECORDING_OFF_HWVAD_ON) {
		/* Disable MICFIL module */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_CTRL1,
					  MICFIL_CTRL1_PDMIEN_MASK,
					  0);

		/* Voice Activity Detector Reset */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_CTRL1,
					  MICFIL_VAD0_CTRL1_RST_SHIFT,
					  MICFIL_VAD0_CTRL1_RST);

		/* Disable HWVAD */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_CTRL1,
					  MICFIL_VAD0_CTRL1_EN_MASK,
					  0);

		/* Disable Signal Filter */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_SCONFIG,
					  MICFIL_VAD0_SCONFIG_SFILEN_MASK,
					  0);

		/* Signal Maximum Enable */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_SCONFIG,
					  MICFIL_VAD0_SCONFIG_SMAXEN_MASK,
					  0);

		/* Enable pre-filter Noise & Signal */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_CTRL2,
					  MICFIL_VAD0_CTRL2_PREFEN_MASK,
					  0);

		/* Noise Decimation Enable */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_NCONFIG,
					  MICFIL_VAD0_NCONFIG_NDECEN_MASK,
					  0);

		/* disable the clock */
		clk_disable_unprepare(micfil->mclk);

		atomic_set(&micfil->state, RECORDING_OFF_HWVAD_OFF);
	} else {
		ret = -EPERM;
		dev_err(dev, "HWVAD is not enabled %d\n", ret);
	}

	return ret;
}

static ssize_t micfil_hwvad_handler(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct kobject *nand_kobj = kobj->parent;
	struct device *dev = container_of(nand_kobj, struct device, kobj);
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	unsigned long enabled_channels;
	int ret;

	ret = kstrtoul(buf, 16, &enabled_channels);
	if (ret < 0)
		return -EINVAL;

	if (enabled_channels > 0 && enabled_channels <= 8) {
		dev_info(dev,
			 "enabling hwvad with %lu channels at rate %d\n",
			 enabled_channels, MICFIL_DEFAULT_RATE);
		micfil->channels = enabled_channels;
		ret = enable_hwvad(dev);
	} else if (!enabled_channels) {
		dev_info(dev, "disabling hwvad\n");
		micfil->channels = 0;
		ret = disable_hwvad(dev);
	} else {
		dev_err(dev, "Unsupported number of channels. Try 0,1..8\n");
		ret = -EINVAL;
	}
	if (ret)
		return ret;

	return count;
}

static struct kobj_attribute hwvad_enable_attribute = __ATTR(enable, 0660, NULL, micfil_hwvad_handler);

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

	if (of_property_read_bool(np, "fsl,shared-interrupt"))
		irqflag = IRQF_SHARED;

	ret = devm_request_threaded_irq(&pdev->dev, irq, micfil_isr,
					hwvad_isr, irqflag,
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

	/* create sysfs entry used to enable hwvad from userspace */
	micfil->hwvad_kobject = kobject_create_and_add("hwvad",
						       &pdev->dev.kobj);
	if (!micfil->hwvad_kobject)
		return -ENOMEM;

	ret = sysfs_create_file(micfil->hwvad_kobject,
				&hwvad_enable_attribute.attr);
	if (ret) {
		dev_err(&pdev->dev, "failed to create file for hwvad_enable\n");
		kobject_put(micfil->hwvad_kobject);
		return -ENOMEM;
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
