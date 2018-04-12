/*
 * deep sleep FSM (finite-state machine) configuration
 *
 * Copyright 2018 NXP
 *
 * Author: Hongbo Zhang <hongbo.zhang@freescale.com>
 *         Chenhui Zhao <chenhui.zhao@freescale.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of the above-listed copyright holders nor the
 *	 names of any contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/types.h>

#include "sleep_fsm.h"
/*
 * These values are from chip's reference manual. For example,
 * the values for T1040 can be found in "8.4.3.8 Programming
 * supporting deep sleep mode" of Chapter 8 "Run Control and
 * Power Management (RCPM)".
 * The default value can be applied to T104x, LS1021.
 */
struct fsm_reg_vals epu_default_val[] = {
	/* EPGCR (Event Processor Global Control Register) */
	{EPGCR, 0},
	/* EPECR (Event Processor Event Control Registers) */
	{EPECR0 + EPECR_STRIDE * 0, 0},
	{EPECR0 + EPECR_STRIDE * 1, 0},
	{EPECR0 + EPECR_STRIDE * 2, 0xF0004004},
	{EPECR0 + EPECR_STRIDE * 3, 0x80000084},
	{EPECR0 + EPECR_STRIDE * 4, 0x20000084},
	{EPECR0 + EPECR_STRIDE * 5, 0x08000004},
	{EPECR0 + EPECR_STRIDE * 6, 0x80000084},
	{EPECR0 + EPECR_STRIDE * 7, 0x80000084},
	{EPECR0 + EPECR_STRIDE * 8, 0x60000084},
	{EPECR0 + EPECR_STRIDE * 9, 0x08000084},
	{EPECR0 + EPECR_STRIDE * 10, 0x42000084},
	{EPECR0 + EPECR_STRIDE * 11, 0x90000084},
	{EPECR0 + EPECR_STRIDE * 12, 0x80000084},
	{EPECR0 + EPECR_STRIDE * 13, 0x08000084},
	{EPECR0 + EPECR_STRIDE * 14, 0x02000084},
	{EPECR0 + EPECR_STRIDE * 15, 0x00000004},
	/*
	 * EPEVTCR (Event Processor EVT Pin Control Registers)
	 * SCU8 triger EVT2, and SCU11 triger EVT9
	 */
	{EPEVTCR0 + EPEVTCR_STRIDE * 0, 0},
	{EPEVTCR0 + EPEVTCR_STRIDE * 1, 0},
	{EPEVTCR0 + EPEVTCR_STRIDE * 2, 0x80000001},
	{EPEVTCR0 + EPEVTCR_STRIDE * 3, 0},
	{EPEVTCR0 + EPEVTCR_STRIDE * 4, 0},
	{EPEVTCR0 + EPEVTCR_STRIDE * 5, 0},
	{EPEVTCR0 + EPEVTCR_STRIDE * 6, 0},
	{EPEVTCR0 + EPEVTCR_STRIDE * 7, 0},
	{EPEVTCR0 + EPEVTCR_STRIDE * 8, 0},
	{EPEVTCR0 + EPEVTCR_STRIDE * 9, 0xB0000001},
	/* EPCMPR (Event Processor Counter Compare Registers) */
	{EPCMPR0 + EPCMPR_STRIDE * 0, 0},
	{EPCMPR0 + EPCMPR_STRIDE * 1, 0},
	{EPCMPR0 + EPCMPR_STRIDE * 2, 0x000000FF},
	{EPCMPR0 + EPCMPR_STRIDE * 3, 0},
	{EPCMPR0 + EPCMPR_STRIDE * 4, 0x000000FF},
	{EPCMPR0 + EPCMPR_STRIDE * 5, 0x00000020},
	{EPCMPR0 + EPCMPR_STRIDE * 6, 0},
	{EPCMPR0 + EPCMPR_STRIDE * 7, 0},
	{EPCMPR0 + EPCMPR_STRIDE * 8, 0x000000FF},
	{EPCMPR0 + EPCMPR_STRIDE * 9, 0x000000FF},
	{EPCMPR0 + EPCMPR_STRIDE * 10, 0x000000FF},
	{EPCMPR0 + EPCMPR_STRIDE * 11, 0x000000FF},
	{EPCMPR0 + EPCMPR_STRIDE * 12, 0x000000FF},
	{EPCMPR0 + EPCMPR_STRIDE * 13, 0},
	{EPCMPR0 + EPCMPR_STRIDE * 14, 0x000000FF},
	{EPCMPR0 + EPCMPR_STRIDE * 15, 0x000000FF},
	/* EPCCR (Event Processor Counter Control Registers) */
	{EPCCR0 + EPCCR_STRIDE * 0, 0},
	{EPCCR0 + EPCCR_STRIDE * 1, 0},
	{EPCCR0 + EPCCR_STRIDE * 2, 0x92840000},
	{EPCCR0 + EPCCR_STRIDE * 3, 0},
	{EPCCR0 + EPCCR_STRIDE * 4, 0x92840000},
	{EPCCR0 + EPCCR_STRIDE * 5, 0x92840000},
	{EPCCR0 + EPCCR_STRIDE * 6, 0},
	{EPCCR0 + EPCCR_STRIDE * 7, 0},
	{EPCCR0 + EPCCR_STRIDE * 8, 0x92840000},
	{EPCCR0 + EPCCR_STRIDE * 9, 0x92840000},
	{EPCCR0 + EPCCR_STRIDE * 10, 0x92840000},
	{EPCCR0 + EPCCR_STRIDE * 11, 0x92840000},
	{EPCCR0 + EPCCR_STRIDE * 12, 0x92840000},
	{EPCCR0 + EPCCR_STRIDE * 13, 0},
	{EPCCR0 + EPCCR_STRIDE * 14, 0x92840000},
	{EPCCR0 + EPCCR_STRIDE * 15, 0x92840000},
	/* EPSMCR (Event Processor SCU Mux Control Registers) */
	{EPSMCR0 + EPSMCR_STRIDE * 0, 0},
	{EPSMCR0 + EPSMCR_STRIDE * 1, 0},
	{EPSMCR0 + EPSMCR_STRIDE * 2, 0x6C700000},
	{EPSMCR0 + EPSMCR_STRIDE * 3, 0x2F000000},
	{EPSMCR0 + EPSMCR_STRIDE * 4, 0x002F0000},
	{EPSMCR0 + EPSMCR_STRIDE * 5, 0x00002E00},
	{EPSMCR0 + EPSMCR_STRIDE * 6, 0x7C000000},
	{EPSMCR0 + EPSMCR_STRIDE * 7, 0x30000000},
	{EPSMCR0 + EPSMCR_STRIDE * 8, 0x64300000},
	{EPSMCR0 + EPSMCR_STRIDE * 9, 0x00003000},
	{EPSMCR0 + EPSMCR_STRIDE * 10, 0x65000030},
	{EPSMCR0 + EPSMCR_STRIDE * 11, 0x31740000},
	{EPSMCR0 + EPSMCR_STRIDE * 12, 0x7F000000},
	{EPSMCR0 + EPSMCR_STRIDE * 13, 0x00003100},
	{EPSMCR0 + EPSMCR_STRIDE * 14, 0x00000031},
	{EPSMCR0 + EPSMCR_STRIDE * 15, 0x76000000},
	/* EPACR (Event Processor Action Control Registers) */
	{EPACR0 + EPACR_STRIDE * 0, 0},
	{EPACR0 + EPACR_STRIDE * 1, 0},
	{EPACR0 + EPACR_STRIDE * 2, 0},
	{EPACR0 + EPACR_STRIDE * 3, 0x00000080},
	{EPACR0 + EPACR_STRIDE * 4, 0},
	{EPACR0 + EPACR_STRIDE * 5, 0x00000040},
	{EPACR0 + EPACR_STRIDE * 6, 0},
	{EPACR0 + EPACR_STRIDE * 7, 0},
	{EPACR0 + EPACR_STRIDE * 8, 0},
	{EPACR0 + EPACR_STRIDE * 9, 0x0000001C},
	{EPACR0 + EPACR_STRIDE * 10, 0x00000020},
	{EPACR0 + EPACR_STRIDE * 11, 0},
	{EPACR0 + EPACR_STRIDE * 12, 0x00000003},
	{EPACR0 + EPACR_STRIDE * 13, 0x06000000},
	{EPACR0 + EPACR_STRIDE * 14, 0x04000000},
	{EPACR0 + EPACR_STRIDE * 15, 0x02000000},
	/* EPIMCR (Event Processor Input Mux Control Registers) */
	{EPIMCR0 + EPIMCR_STRIDE * 0, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 1, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 2, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 3, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 4, 0x44000000},
	{EPIMCR0 + EPIMCR_STRIDE * 5, 0x40000000},
	{EPIMCR0 + EPIMCR_STRIDE * 6, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 7, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 8, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 9, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 10, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 11, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 12, 0x44000000},
	{EPIMCR0 + EPIMCR_STRIDE * 13, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 14, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 15, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 16, 0x6A000000},
	{EPIMCR0 + EPIMCR_STRIDE * 17, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 18, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 19, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 20, 0x48000000},
	{EPIMCR0 + EPIMCR_STRIDE * 21, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 22, 0x6C000000},
	{EPIMCR0 + EPIMCR_STRIDE * 23, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 24, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 25, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 26, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 27, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 28, 0x76000000},
	{EPIMCR0 + EPIMCR_STRIDE * 29, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 30, 0},
	{EPIMCR0 + EPIMCR_STRIDE * 31, 0x76000000},
	/* EPXTRIGCR (Event Processor Crosstrigger Control Register) */
	{EPXTRIGCR, 0x0000FFDF},
	/* end */
	{FSM_END_FLAG, 0},
};

struct fsm_reg_vals npc_default_val[] = {
	/* NPC triggered Memory-Mapped Access Registers */
	{NCR, 0x80000000},
	{MCCR1, 0},
	{MCSR1, 0},
	{MMAR1LO, 0},
	{MMAR1HI, 0},
	{MMDR1, 0},
	{MCSR2, 0},
	{MMAR2LO, 0},
	{MMAR2HI, 0},
	{MMDR2, 0},
	{MCSR3, 0x80000000},
	{MMAR3LO, 0x000E2130},
	{MMAR3HI, 0x00030000},
	{MMDR3, 0x00020000},
	/* end */
	{FSM_END_FLAG, 0},
};

/**
 * fsl_fsm_setup - Configure EPU's FSM registers
 * @base: the base address of registers
 * @val: Pointer to address-value pairs for FSM registers
 */
void fsl_fsm_setup(void __iomem *base, struct fsm_reg_vals *val)
{
	struct fsm_reg_vals *data = val;

	WARN_ON(!base || !data);
	while (data->offset != FSM_END_FLAG) {
		iowrite32be(data->value, base + data->offset);
		data++;
	}
}

void fsl_epu_setup_default(void __iomem *epu_base)
{
	fsl_fsm_setup(epu_base, epu_default_val);
}

void fsl_npc_setup_default(void __iomem *npc_base)
{
	fsl_fsm_setup(npc_base, npc_default_val);
}

void fsl_epu_clean_default(void __iomem *epu_base)
{
	u32 offset;

	/* follow the exact sequence to clear the registers */
	/* Clear EPACRn */
	for (offset = EPACR0; offset <= EPACR15; offset += EPACR_STRIDE)
		iowrite32be(0, epu_base + offset);

	/* Clear EPEVTCRn */
	for (offset = EPEVTCR0; offset <= EPEVTCR9; offset += EPEVTCR_STRIDE)
		iowrite32be(0, epu_base + offset);

	/* Clear EPGCR */
	iowrite32be(0, epu_base + EPGCR);

	/* Clear EPSMCRn */
	for (offset = EPSMCR0; offset <= EPSMCR15; offset += EPSMCR_STRIDE)
		iowrite32be(0, epu_base + offset);

	/* Clear EPCCRn */
	for (offset = EPCCR0; offset <= EPCCR31; offset += EPCCR_STRIDE)
		iowrite32be(0, epu_base + offset);

	/* Clear EPCMPRn */
	for (offset = EPCMPR0; offset <= EPCMPR31; offset += EPCMPR_STRIDE)
		iowrite32be(0, epu_base + offset);

	/* Clear EPCTRn */
	for (offset = EPCTR0; offset <= EPCTR31; offset += EPCTR_STRIDE)
		iowrite32be(0, epu_base + offset);

	/* Clear EPIMCRn */
	for (offset = EPIMCR0; offset <= EPIMCR31; offset += EPIMCR_STRIDE)
		iowrite32be(0, epu_base + offset);

	/* Clear EPXTRIGCRn */
	iowrite32be(0, epu_base + EPXTRIGCR);

	/* Clear EPECRn */
	for (offset = EPECR0; offset <= EPECR15; offset += EPECR_STRIDE)
		iowrite32be(0, epu_base + offset);
}
