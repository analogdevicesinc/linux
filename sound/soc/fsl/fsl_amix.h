/*
 * Copyright 2017 NXP Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __FSL_AMIX_H
#define __FSL_AMIX_H

#define FSL_AMIX_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FMTBIT_S32_LE)
/* AMIX Registers */
#define FSL_AMIX_CTR		0x200 /* Control */
#define FSL_AMIX_STR		0x204 /* Status */

#define FSL_AMIX_ATCR0		0x208 /* Attenuation Control */
#define FSL_AMIX_ATIVAL0	0x20c /* Attenuation Initial Value */
#define FSL_AMIX_ATSTPUP0	0x210 /* Attenuation step up factor */
#define FSL_AMIX_ATSTPDN0	0x214 /* Attenuation step down factor */
#define FSL_AMIX_ATSTPTGT0	0x218 /* Attenuation step target */
#define FSL_AMIX_ATTNVAL0	0x21c /* Attenuation Value */
#define FSL_AMIX_ATSTP0		0x220 /* Attenuation step number */

#define FSL_AMIX_ATCR1		0x228 /* Attenuation Control */
#define FSL_AMIX_ATIVAL1	0x22c /* Attenuation Initial Value */
#define FSL_AMIX_ATSTPUP1	0x230 /* Attenuation step up factor */
#define FSL_AMIX_ATSTPDN1	0x234 /* Attenuation step down factor */
#define FSL_AMIX_ATSTPTGT1	0x238 /* Attenuation step target */
#define FSL_AMIX_ATTNVAL1	0x23c /* Attenuation Value */
#define FSL_AMIX_ATSTP1		0x240 /* Attenuation step number */

/* AMIX Control Register */
#define FSL_AMIX_CTR_MIXCLK_SHIFT	0
#define FSL_AMIX_CTR_MIXCLK_MASK	(1 << FSL_AMIX_CTR_MIXCLK_SHIFT)
#define FSL_AMIX_CTR_MIXCLK(i)		(i - 1)
#define FSL_AMIX_CTR_OUTSRC_SHIFT	1
#define FSL_AMIX_CTR_OUTSRC_MASK	(0x3 << FSL_AMIX_CTR_OUTSRC_SHIFT)
#define FSL_AMIX_CTR_OUTSRC(i)		((i  << FSL_AMIX_CTR_OUTSRC_SHIFT) \
					      & FSL_AMIX_CTR_OUTSRC_MASK)
#define FSL_AMIX_CTR_OUTWIDTH_SHIFT	3
#define FSL_AMIX_CTR_OUTWIDTH_MASK	(0x7 << FSL_AMIX_CTR_OUTWIDTH_SHIFT)
#define FSL_AMIX_CTR_OUTWIDTH(i)	((i  << FSL_AMIX_CTR_OUTWIDTH_SHIFT) \
					      & FSL_AMIX_CTR_OUTWIDTH_MASK)
#define FSL_AMIX_CTR_OUTCKPOL_SHIFT	6
#define FSL_AMIX_CTR_OUTCKPOL_MASK	(1 << FSL_AMIX_CTR_OUTCKPOL_SHIFT)
#define FSL_AMIX_CTR_OUTCKPOL(i)	(i << FSL_AMIX_CTR_OUTCKPOL_SHIFT)
#define FSL_AMIX_CTR_MASKRTDF_SHIFT	7
#define FSL_AMIX_CTR_MASKRTDF_MASK	(1 << FSL_AMIX_CTR_MASKRTDF_SHIFT)
#define FSL_AMIX_CTR_MASKRTDF(i)	(i << FSL_AMIX_CTR_MASKRTDF_SHIFT)
#define FSL_AMIX_CTR_MASKCKDF_SHIFT	8
#define FSL_AMIX_CTR_MASKCKDF_MASK	(1 << FSL_AMIX_CTR_MASKCKDF_SHIFT)
#define FSL_AMIX_CTR_MASKCKDF(i)	(i << FSL_AMIX_CTR_MASKCKDF_SHIFT)
#define FSL_AMIX_CTR_SYNCMODE_SHIFT	9
#define FSL_AMIX_CTR_SYNCMODE_MASK	(1 << FSL_AMIX_CTR_SYNCMODE_SHIFT)
#define FSL_AMIX_CTR_SYNCMODE(i)	(i << FSL_AMIX_CTR_SYNCMODE_SHIFT)
#define FSL_AMIX_CTR_SYNCSRC_SHIFT	10
#define FSL_AMIX_CTR_SYNCSRC_MASK	(1 << FSL_AMIX_CTR_SYNCSRC_SHIFT)
#define FSL_AMIX_CTR_SYNCSRC(i)		(i << FSL_AMIX_CTR_SYNCSRC_SHIFT)

/* AMIX Status Register */
#define FSL_AMIX_STR_RATEDIFF		BIT(0)
#define FSL_AMIX_STR_CLKDIFF		BIT(1)
#define FSL_AMIX_STR_MIXSTAT_SHIFT	2
#define FSL_AMIX_STR_MIXSTAT_MASK	(0x3 << FSL_AMIX_STR_MIXSTAT_SHIFT)
#define FSL_AMIX_STR_MIXSTAT(i)		((i & FSL_AMIX_STR_MIXSTAT_MASK) \
					   >> FSL_AMIX_STR_MIXSTAT_SHIFT)
/* AMIX Attenuation Control Register */
#define FSL_AMIX_ATCR_AT_EN		BIT(0)
#define FSL_AMIX_ATCR_AT_UPDN		BIT(1)
#define FSL_AMIX_ATCR_ATSTPDIF_SHIFT	2
#define FSL_AMIX_ATCR_ATSTPDFI_MASK	(0xfff << FSL_AMIX_ATCR_ATSTPDIF_SHIFT)

/* AMIX Attenuation Initial Value Register */
#define FSL_AMIX_ATIVAL_ATINVAL_MASK	0x3FFFF

/* AMIX Attenuation Step Up Factor Register */
#define FSL_AMIX_ATSTPUP_ATSTEPUP_MASK	0x3FFFF

/* AMIX Attenuation Step Down Factor Register */
#define FSL_AMIX_ATSTPDN_ATSTEPDN_MASK	0x3FFFF

/* AMIX Attenuation Step Target Register */
#define FSL_AMIX_ATSTPTGT_ATSTPTG_MASK	0x3FFFF

/* AMIX Attenuation Value Register */
#define FSL_AMIX_ATTNVAL_ATCURVAL_MASK	0x3FFFF

/* AMIX Attenuation Step Number Register */
#define FSL_AMIX_ATSTP_STPCTR_MASK	0x3FFFF

#define FSL_AMIX_MAX_DAIS		2
struct fsl_amix {
	struct platform_device *pdev;
	struct regmap *regmap;
	struct clk *ipg_clk;
};

#endif /* __FSL_AMIX_H */
