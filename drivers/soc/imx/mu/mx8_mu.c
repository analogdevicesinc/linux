/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/mx8_mu.h>
#include <linux/of.h>

static int version;

/*!
 * This function sets the Flag n of the MU.
 */
int32_t MU_SetFn(void __iomem *base, uint32_t Fn)
{
	uint32_t reg, offset;

	reg = Fn & (~MU_CR_Fn_MASK1);
	if (reg > 0)
		return -EINVAL;

	offset = unlikely(version == MU_VER_ID_V10)
			  ? MU_V10_ACR_OFFSET1 : MU_ACR_OFFSET1;

	reg = readl_relaxed(base + offset);
	/*  Clear ABFn. */
	reg &= ~MU_CR_Fn_MASK1;
	reg |= Fn;
	writel_relaxed(reg, base + offset);

	return 0;
}

/*!
 * This function reads the status from status register.
 */
uint32_t MU_ReadStatus(void __iomem *base)
{
	uint32_t reg, offset;

	offset = unlikely(version == MU_VER_ID_V10)
			  ? MU_V10_ASR_OFFSET1 : MU_ASR_OFFSET1;

	reg = readl_relaxed(base + offset);

	return reg;
}

/*!
 * This function enables specific RX full interrupt.
 */
void MU_EnableRxFullInt(void __iomem *base, uint32_t index)
{
	uint32_t reg, offset;

	offset = unlikely(version == MU_VER_ID_V10)
			  ? MU_V10_ACR_OFFSET1 : MU_ACR_OFFSET1;

	reg = readl_relaxed(base + offset);
	reg &= ~(MU_CR_GIRn_MASK1 | MU_CR_NMI_MASK1);
	reg |= MU_CR_RIE0_MASK1 >> index;
	writel_relaxed(reg, base + offset);
}

/*!
 * This function enables specific general purpose interrupt.
 */
void MU_EnableGeneralInt(void __iomem *base, uint32_t index)
{
	uint32_t reg, offset;

	offset = unlikely(version == MU_VER_ID_V10)
			  ? MU_V10_ACR_OFFSET1 : MU_ACR_OFFSET1;

	reg = readl_relaxed(base + offset);
	reg &= ~(MU_CR_GIRn_MASK1 | MU_CR_NMI_MASK1);
	reg |= MU_CR_GIE0_MASK1 >> index;
	writel_relaxed(reg, base + offset);
}

/*
 * Wait and send message to the other core.
 */
void MU_SendMessage(void __iomem *base, uint32_t regIndex, uint32_t msg)
{
	uint32_t mask = MU_SR_TE0_MASK1 >> regIndex;

	if (unlikely(version == MU_VER_ID_V10)) {
		/* Wait TX register to be empty. */
		while (!(readl_relaxed(base + MU_V10_ASR_OFFSET1) & mask))
			;
		writel_relaxed(msg, base + MU_V10_ATR0_OFFSET1
			       + (regIndex * 4));
	} else {
		/* Wait TX register to be empty. */
		while (!(readl_relaxed(base + MU_ASR_OFFSET1) & mask))
			;
		writel_relaxed(msg, base + MU_ATR0_OFFSET1  + (regIndex * 4));
	}
}


/*
 * Wait to receive message from the other core.
 */
void MU_ReceiveMsg(void __iomem *base, uint32_t regIndex, uint32_t *msg)
{
	uint32_t mask = MU_SR_RF0_MASK1 >> regIndex;

	if (unlikely(version == MU_VER_ID_V10)) {
		/* Wait RX register to be full. */
		while (!(readl_relaxed(base + MU_V10_ASR_OFFSET1) & mask))
			;
		*msg = readl_relaxed(base + MU_V10_ARR0_OFFSET1
				     + (regIndex * 4));
	} else {
		/* Wait RX register to be full. */
		while (!(readl_relaxed(base + MU_ASR_OFFSET1) & mask))
			;
		*msg = readl_relaxed(base + MU_ARR0_OFFSET1 + (regIndex * 4));
	}
}



void MU_Init(void __iomem *base)
{
	uint32_t reg, offset;

	version = readl_relaxed(base) >> 16;

	offset = unlikely(version == MU_VER_ID_V10)
			  ? MU_V10_ACR_OFFSET1 : MU_ACR_OFFSET1;

	reg = readl_relaxed(base + offset);
	/* Clear GIEn, TIEn, GIRn and ABFn. */
	reg &= ~(MU_CR_GIEn_MASK1 | MU_CR_TIEn_MASK1
		 | MU_CR_GIRn_MASK1 | MU_CR_NMI_MASK1 | MU_CR_Fn_MASK1);

	/*
	 * i.MX6SX and i.MX7D have multi-core power management which need
	 * to use RIE interrupts.
	 */
	if (!(of_machine_is_compatible("fsl,imx6sx") ||
		of_machine_is_compatible("fsl,imx7d")))
		reg &= ~MU_CR_RIEn_MASK1;

	writel_relaxed(reg, base + offset);
}

/**@}*/

