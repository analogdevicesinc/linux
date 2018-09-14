/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#define MU_ATR0_OFFSET1		0x0
#define MU_ARR0_OFFSET1		0x10
#define MU_ASR_OFFSET1		0x20
#define MU_ACR_OFFSET1		0x24

/* Registers offsets of the MU Version 1.0 */
#define MU_V10_VER_OFFSET1	0x0
#define MU_V10_ATR0_OFFSET1	0x20
#define MU_V10_ARR0_OFFSET1	0x40
#define MU_V10_ASR_OFFSET1	0x60
#define MU_V10_ACR_OFFSET1	0x64
#define MU_VER_ID_V10		0x0100 /* Version 1.0 */

#define MU_TR_COUNT1		4
#define MU_RR_COUNT1		4

#define MU_CR_GIEn_MASK1	(0xF << 28)
#define MU_CR_RIEn_MASK1	(0xF << 24)
#define MU_CR_TIEn_MASK1	(0xF << 20)
#define MU_CR_GIRn_MASK1	(0xF << 16)
#define MU_CR_NMI_MASK1		(1 << 3)
#define MU_CR_Fn_MASK1		0x7

#define MU_SR_TE0_MASK1		(1 << 23)
#define MU_SR_RF0_MASK1		(1 << 27)
#define MU_CR_RIE0_MASK1	(1 << 27)
#define MU_CR_GIE0_MASK1	(1 << 31)

#define MU_TR_COUNT			4
#define MU_RR_COUNT			4


void MU_Init(void __iomem *base);
void MU_SendMessage(void __iomem *base, uint32_t regIndex, uint32_t msg);
void MU_SendMessageTimeout(void __iomem *base, uint32_t regIndex, uint32_t msg, uint32_t t);
void MU_ReceiveMsg(void __iomem *base, uint32_t regIndex, uint32_t *msg);
void MU_EnableGeneralInt(void __iomem *base, uint32_t index);
void MU_EnableRxFullInt(void __iomem *base, uint32_t index);
uint32_t MU_ReadStatus(void __iomem *base);
int32_t MU_SetFn(void __iomem *base, uint32_t Fn);

