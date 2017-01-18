/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <linux/err.h>
#include <linux/io.h>
#include "mx8_mu.h"


/*!
 * This function enables specific RX full interrupt.
 */
void MU_EnableRxFullInt(void __iomem *base, uint32_t index)
{
	uint32_t reg = readl_relaxed(base + MU_ACR_OFFSET1);

	reg &= ~(MU_CR_GIRn_MASK1 | MU_CR_NMI_MASK1);
	reg |= ~MU_CR_RIE0_MASK1 >> index;
	writel_relaxed(reg, base + MU_ACR_OFFSET1);
}

/*!
 * This function enables specific general purpose interrupt.
 */
void MU_EnableGeneralInt(void __iomem *base, uint32_t index)
{
	uint32_t reg = readl_relaxed(base + MU_ACR_OFFSET1);

	reg &= ~(MU_CR_GIRn_MASK1 | MU_CR_NMI_MASK1);
	reg |= MU_CR_GIE0_MASK1 >> index;
	writel_relaxed(reg, base + MU_ACR_OFFSET1);
}

/*
 * Wait and send message to the other core.
 */
void MU_SendMessage(void __iomem *base, uint32_t regIndex, uint32_t msg)
{
	uint32_t mask = MU_SR_TE0_MASK1 >> regIndex;

	/* Wait TX register to be empty. */
	while (!(readl_relaxed(base + MU_ASR_OFFSET1) & mask))
		;
	writel_relaxed(msg, base + MU_ATR0_OFFSET1  + (regIndex * 4));
}


/*
 * Wait to receive message from the other core.
 */
void MU_ReceiveMsg(void __iomem *base, uint32_t regIndex, uint32_t *msg)
{
	uint32_t mask = MU_SR_RF0_MASK1 >> regIndex;

	/* Wait RX register to be full. */
	while (!(readl_relaxed(base + MU_ASR_OFFSET1) & mask))
		;
	*msg = readl_relaxed(base + MU_ARR0_OFFSET1 + (regIndex * 4));
}



void MU_Init(void __iomem *base)
{
	uint32_t reg;

	reg = readl_relaxed(base + MU_ACR_OFFSET1);
	/* Clear GIEn, RIEn, TIEn, GIRn and ABFn. */
	reg &= ~(MU_CR_GIEn_MASK1 | MU_CR_RIEn_MASK1 | MU_CR_TIEn_MASK1
			| MU_CR_GIRn_MASK1 | MU_CR_NMI_MASK1 | MU_CR_Fn_MASK1);
	writel_relaxed(reg, base + MU_ACR_OFFSET1);
}

/**@}*/

