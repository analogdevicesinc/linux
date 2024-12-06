/*
 * deep sleep FSM (finite-state machine) configuration
 *
 * Copyright 2018 NXP
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

#ifndef _FSL_SLEEP_FSM_H
#define _FSL_SLEEP_FSM_H

#define FSL_STRIDE_4B	4
#define FSL_STRIDE_8B	8

/* End flag */
#define FSM_END_FLAG		0xFFFFFFFFUL

/* Block offsets */
#define RCPM_BLOCK_OFFSET	0x00022000
#define EPU_BLOCK_OFFSET	0x00000000
#define NPC_BLOCK_OFFSET	0x00001000

/* EPGCR (Event Processor Global Control Register) */
#define EPGCR		0x000

/* EPEVTCR0-9 (Event Processor EVT Pin Control Registers) */
#define EPEVTCR0	0x050
#define EPEVTCR9	0x074
#define EPEVTCR_STRIDE	FSL_STRIDE_4B

/* EPXTRIGCR (Event Processor Crosstrigger Control Register) */
#define EPXTRIGCR	0x090

/* EPIMCR0-31 (Event Processor Input Mux Control Registers) */
#define EPIMCR0		0x100
#define EPIMCR31	0x17C
#define EPIMCR_STRIDE	FSL_STRIDE_4B

/* EPSMCR0-15 (Event Processor SCU Mux Control Registers) */
#define EPSMCR0		0x200
#define EPSMCR15	0x278
#define EPSMCR_STRIDE	FSL_STRIDE_8B

/* EPECR0-15 (Event Processor Event Control Registers) */
#define EPECR0		0x300
#define EPECR15		0x33C
#define EPECR_STRIDE	FSL_STRIDE_4B

/* EPACR0-15 (Event Processor Action Control Registers) */
#define EPACR0		0x400
#define EPACR15		0x43C
#define EPACR_STRIDE	FSL_STRIDE_4B

/* EPCCRi0-15 (Event Processor Counter Control Registers) */
#define EPCCR0		0x800
#define EPCCR15		0x83C
#define EPCCR31		0x87C
#define EPCCR_STRIDE	FSL_STRIDE_4B

/* EPCMPR0-15 (Event Processor Counter Compare Registers) */
#define EPCMPR0		0x900
#define EPCMPR15	0x93C
#define EPCMPR31	0x97C
#define EPCMPR_STRIDE	FSL_STRIDE_4B

/* EPCTR0-31 (Event Processor Counter Register) */
#define EPCTR0		0xA00
#define EPCTR31		0xA7C
#define EPCTR_STRIDE	FSL_STRIDE_4B

/* NPC triggered Memory-Mapped Access Registers */
#define NCR		0x000
#define MCCR1		0x0CC
#define MCSR1		0x0D0
#define MMAR1LO		0x0D4
#define MMAR1HI		0x0D8
#define MMDR1		0x0DC
#define MCSR2		0x0E0
#define MMAR2LO		0x0E4
#define MMAR2HI		0x0E8
#define MMDR2		0x0EC
#define MCSR3		0x0F0
#define MMAR3LO		0x0F4
#define MMAR3HI		0x0F8
#define MMDR3		0x0FC

/* RCPM Core State Action Control Register 0 */
#define CSTTACR0	0xB00

/* RCPM Core Group 1 Configuration Register 0 */
#define CG1CR0		0x31C

struct fsm_reg_vals {
	u32 offset;
	u32 value;
};

void fsl_fsm_setup(void __iomem *base, struct fsm_reg_vals *val);
void fsl_epu_setup_default(void __iomem *epu_base);
void fsl_npc_setup_default(void __iomem *npc_base);
void fsl_epu_clean_default(void __iomem *epu_base);

#endif /* _FSL_SLEEP_FSM_H */
