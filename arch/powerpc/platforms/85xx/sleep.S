/*
 * Enter and leave deep sleep/sleep state
 *
 * Copyright 2018 NXP
 * Author: Scott Wood <scottwood@freescale.com>
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

#include <asm/page.h>
#include <asm/ppc_asm.h>
#include <asm/reg.h>
#include <asm/asm-offsets.h>
#include <asm/fsl_pm.h>
#include <asm/mmu.h>

/*
 * the number of bytes occupied by one register
 * the value of 8 is compatible with both 32-bit and 64-bit registers
 */
#define STRIDE_SIZE		8

/* GPR0 - GPR31 */
#define BOOKE_GPR0_OFF		0x0000
#define BOOKE_GPR_COUNT		32
/* IVOR0 - IVOR42 */
#define BOOKE_IVOR0_OFF	   (BOOKE_GPR0_OFF + BOOKE_GPR_COUNT * STRIDE_SIZE)
#define BOOKE_IVOR_COUNT	43
/* SPRG0 - SPRG9 */
#define BOOKE_SPRG0_OFF	   (BOOKE_IVOR0_OFF + BOOKE_IVOR_COUNT * STRIDE_SIZE)
#define BOOKE_SPRG_COUNT	10
/* IVPR */
#define BOOKE_IVPR_OFF	   (BOOKE_SPRG0_OFF + BOOKE_SPRG_COUNT * STRIDE_SIZE)

#define BOOKE_LR_OFF		(BOOKE_IVPR_OFF + STRIDE_SIZE)
#define BOOKE_MSR_OFF		(BOOKE_LR_OFF + STRIDE_SIZE)
#define BOOKE_TBU_OFF		(BOOKE_MSR_OFF + STRIDE_SIZE)
#define BOOKE_TBL_OFF		(BOOKE_TBU_OFF + STRIDE_SIZE)
#define BOOKE_EPCR_OFF		(BOOKE_TBL_OFF + STRIDE_SIZE)
#define BOOKE_HID0_OFF		(BOOKE_EPCR_OFF + STRIDE_SIZE)
#define BOOKE_PIR_OFF		(BOOKE_HID0_OFF + STRIDE_SIZE)
#define BOOKE_PID0_OFF		(BOOKE_PIR_OFF + STRIDE_SIZE)
#define BOOKE_BUCSR_OFF		(BOOKE_PID0_OFF + STRIDE_SIZE)

#define BUFFER_SIZE		(BOOKE_BUCSR_OFF + STRIDE_SIZE)

#undef SAVE_GPR
#define SAVE_GPR(gpr, offset) \
	PPC_STL gpr, offset(r10)

#define RESTORE_GPR(gpr, offset) \
	PPC_LL gpr, offset(r10)

#define SAVE_SPR(spr, offset) \
	mfspr	r0, spr ;\
	PPC_STL	r0, offset(r10)

#define RESTORE_SPR(spr, offset) \
	PPC_LL	r0, offset(r10) ;\
	mtspr	spr, r0

#define SAVE_ALL_GPR \
	SAVE_GPR(r1, BOOKE_GPR0_OFF + STRIDE_SIZE * 1) ;\
	SAVE_GPR(r2, BOOKE_GPR0_OFF + STRIDE_SIZE * 2) ;\
	SAVE_GPR(r13, BOOKE_GPR0_OFF + STRIDE_SIZE * 13) ;\
	SAVE_GPR(r14, BOOKE_GPR0_OFF + STRIDE_SIZE * 14) ;\
	SAVE_GPR(r15, BOOKE_GPR0_OFF + STRIDE_SIZE * 15) ;\
	SAVE_GPR(r16, BOOKE_GPR0_OFF + STRIDE_SIZE * 16) ;\
	SAVE_GPR(r17, BOOKE_GPR0_OFF + STRIDE_SIZE * 17) ;\
	SAVE_GPR(r18, BOOKE_GPR0_OFF + STRIDE_SIZE * 18) ;\
	SAVE_GPR(r19, BOOKE_GPR0_OFF + STRIDE_SIZE * 19) ;\
	SAVE_GPR(r20, BOOKE_GPR0_OFF + STRIDE_SIZE * 20) ;\
	SAVE_GPR(r21, BOOKE_GPR0_OFF + STRIDE_SIZE * 21) ;\
	SAVE_GPR(r22, BOOKE_GPR0_OFF + STRIDE_SIZE * 22) ;\
	SAVE_GPR(r23, BOOKE_GPR0_OFF + STRIDE_SIZE * 23) ;\
	SAVE_GPR(r24, BOOKE_GPR0_OFF + STRIDE_SIZE * 24) ;\
	SAVE_GPR(r25, BOOKE_GPR0_OFF + STRIDE_SIZE * 25) ;\
	SAVE_GPR(r26, BOOKE_GPR0_OFF + STRIDE_SIZE * 26) ;\
	SAVE_GPR(r27, BOOKE_GPR0_OFF + STRIDE_SIZE * 27) ;\
	SAVE_GPR(r28, BOOKE_GPR0_OFF + STRIDE_SIZE * 28) ;\
	SAVE_GPR(r29, BOOKE_GPR0_OFF + STRIDE_SIZE * 29) ;\
	SAVE_GPR(r30, BOOKE_GPR0_OFF + STRIDE_SIZE * 30) ;\
	SAVE_GPR(r31, BOOKE_GPR0_OFF + STRIDE_SIZE * 31)

#define RESTORE_ALL_GPR \
	RESTORE_GPR(r1, BOOKE_GPR0_OFF + STRIDE_SIZE * 1) ;\
	RESTORE_GPR(r2, BOOKE_GPR0_OFF + STRIDE_SIZE * 2) ;\
	RESTORE_GPR(r13, BOOKE_GPR0_OFF + STRIDE_SIZE * 13) ;\
	RESTORE_GPR(r14, BOOKE_GPR0_OFF + STRIDE_SIZE * 14) ;\
	RESTORE_GPR(r15, BOOKE_GPR0_OFF + STRIDE_SIZE * 15) ;\
	RESTORE_GPR(r16, BOOKE_GPR0_OFF + STRIDE_SIZE * 16) ;\
	RESTORE_GPR(r17, BOOKE_GPR0_OFF + STRIDE_SIZE * 17) ;\
	RESTORE_GPR(r18, BOOKE_GPR0_OFF + STRIDE_SIZE * 18) ;\
	RESTORE_GPR(r19, BOOKE_GPR0_OFF + STRIDE_SIZE * 19) ;\
	RESTORE_GPR(r20, BOOKE_GPR0_OFF + STRIDE_SIZE * 20) ;\
	RESTORE_GPR(r21, BOOKE_GPR0_OFF + STRIDE_SIZE * 21) ;\
	RESTORE_GPR(r22, BOOKE_GPR0_OFF + STRIDE_SIZE * 22) ;\
	RESTORE_GPR(r23, BOOKE_GPR0_OFF + STRIDE_SIZE * 23) ;\
	RESTORE_GPR(r24, BOOKE_GPR0_OFF + STRIDE_SIZE * 24) ;\
	RESTORE_GPR(r25, BOOKE_GPR0_OFF + STRIDE_SIZE * 25) ;\
	RESTORE_GPR(r26, BOOKE_GPR0_OFF + STRIDE_SIZE * 26) ;\
	RESTORE_GPR(r27, BOOKE_GPR0_OFF + STRIDE_SIZE * 27) ;\
	RESTORE_GPR(r28, BOOKE_GPR0_OFF + STRIDE_SIZE * 28) ;\
	RESTORE_GPR(r29, BOOKE_GPR0_OFF + STRIDE_SIZE * 29) ;\
	RESTORE_GPR(r30, BOOKE_GPR0_OFF + STRIDE_SIZE * 30) ;\
	RESTORE_GPR(r31, BOOKE_GPR0_OFF + STRIDE_SIZE * 31)

#define SAVE_ALL_SPRG \
	SAVE_SPR(SPRN_SPRG0, BOOKE_SPRG0_OFF + STRIDE_SIZE * 0) ;\
	SAVE_SPR(SPRN_SPRG1, BOOKE_SPRG0_OFF + STRIDE_SIZE * 1) ;\
	SAVE_SPR(SPRN_SPRG2, BOOKE_SPRG0_OFF + STRIDE_SIZE * 2) ;\
	SAVE_SPR(SPRN_SPRG3, BOOKE_SPRG0_OFF + STRIDE_SIZE * 3) ;\
	SAVE_SPR(SPRN_SPRG4, BOOKE_SPRG0_OFF + STRIDE_SIZE * 4) ;\
	SAVE_SPR(SPRN_SPRG5, BOOKE_SPRG0_OFF + STRIDE_SIZE * 5) ;\
	SAVE_SPR(SPRN_SPRG6, BOOKE_SPRG0_OFF + STRIDE_SIZE * 6) ;\
	SAVE_SPR(SPRN_SPRG7, BOOKE_SPRG0_OFF + STRIDE_SIZE * 7) ;\
	SAVE_SPR(SPRN_SPRG8, BOOKE_SPRG0_OFF + STRIDE_SIZE * 8) ;\
	SAVE_SPR(SPRN_SPRG9, BOOKE_SPRG0_OFF + STRIDE_SIZE * 9)

#define RESTORE_ALL_SPRG \
	RESTORE_SPR(SPRN_SPRG0, BOOKE_SPRG0_OFF + STRIDE_SIZE * 0) ;\
	RESTORE_SPR(SPRN_SPRG1, BOOKE_SPRG0_OFF + STRIDE_SIZE * 1) ;\
	RESTORE_SPR(SPRN_SPRG2, BOOKE_SPRG0_OFF + STRIDE_SIZE * 2) ;\
	RESTORE_SPR(SPRN_SPRG3, BOOKE_SPRG0_OFF + STRIDE_SIZE * 3) ;\
	RESTORE_SPR(SPRN_SPRG4, BOOKE_SPRG0_OFF + STRIDE_SIZE * 4) ;\
	RESTORE_SPR(SPRN_SPRG5, BOOKE_SPRG0_OFF + STRIDE_SIZE * 5) ;\
	RESTORE_SPR(SPRN_SPRG6, BOOKE_SPRG0_OFF + STRIDE_SIZE * 6) ;\
	RESTORE_SPR(SPRN_SPRG7, BOOKE_SPRG0_OFF + STRIDE_SIZE * 7) ;\
	RESTORE_SPR(SPRN_SPRG8, BOOKE_SPRG0_OFF + STRIDE_SIZE * 8) ;\
	RESTORE_SPR(SPRN_SPRG9, BOOKE_SPRG0_OFF + STRIDE_SIZE * 9)

#define SAVE_ALL_IVOR \
	SAVE_SPR(SPRN_IVOR0, BOOKE_IVOR0_OFF + STRIDE_SIZE * 0) ;\
	SAVE_SPR(SPRN_IVOR1, BOOKE_IVOR0_OFF + STRIDE_SIZE * 1) ;\
	SAVE_SPR(SPRN_IVOR2, BOOKE_IVOR0_OFF + STRIDE_SIZE * 2) ;\
	SAVE_SPR(SPRN_IVOR3, BOOKE_IVOR0_OFF + STRIDE_SIZE * 3) ;\
	SAVE_SPR(SPRN_IVOR4, BOOKE_IVOR0_OFF + STRIDE_SIZE * 4) ;\
	SAVE_SPR(SPRN_IVOR5, BOOKE_IVOR0_OFF + STRIDE_SIZE * 5) ;\
	SAVE_SPR(SPRN_IVOR6, BOOKE_IVOR0_OFF + STRIDE_SIZE * 6) ;\
	SAVE_SPR(SPRN_IVOR7, BOOKE_IVOR0_OFF + STRIDE_SIZE * 7) ;\
	SAVE_SPR(SPRN_IVOR8, BOOKE_IVOR0_OFF + STRIDE_SIZE * 8) ;\
	SAVE_SPR(SPRN_IVOR9, BOOKE_IVOR0_OFF + STRIDE_SIZE * 9) ;\
	SAVE_SPR(SPRN_IVOR10, BOOKE_IVOR0_OFF + STRIDE_SIZE * 10) ;\
	SAVE_SPR(SPRN_IVOR11, BOOKE_IVOR0_OFF + STRIDE_SIZE * 11) ;\
	SAVE_SPR(SPRN_IVOR12, BOOKE_IVOR0_OFF + STRIDE_SIZE * 12) ;\
	SAVE_SPR(SPRN_IVOR13, BOOKE_IVOR0_OFF + STRIDE_SIZE * 13) ;\
	SAVE_SPR(SPRN_IVOR14, BOOKE_IVOR0_OFF + STRIDE_SIZE * 14) ;\
	SAVE_SPR(SPRN_IVOR15, BOOKE_IVOR0_OFF + STRIDE_SIZE * 15) ;\
	SAVE_SPR(SPRN_IVOR35, BOOKE_IVOR0_OFF + STRIDE_SIZE * 35) ;\
	SAVE_SPR(SPRN_IVOR36, BOOKE_IVOR0_OFF + STRIDE_SIZE * 36) ;\
	SAVE_SPR(SPRN_IVOR37, BOOKE_IVOR0_OFF + STRIDE_SIZE * 37) ;\
	SAVE_SPR(SPRN_IVOR38, BOOKE_IVOR0_OFF + STRIDE_SIZE * 38) ;\
	SAVE_SPR(SPRN_IVOR39, BOOKE_IVOR0_OFF + STRIDE_SIZE * 39) ;\
	SAVE_SPR(SPRN_IVOR40, BOOKE_IVOR0_OFF + STRIDE_SIZE * 40) ;\
	SAVE_SPR(SPRN_IVOR41, BOOKE_IVOR0_OFF + STRIDE_SIZE * 41)

#define RESTORE_ALL_IVOR \
	RESTORE_SPR(SPRN_IVOR0, BOOKE_IVOR0_OFF + STRIDE_SIZE * 0) ;\
	RESTORE_SPR(SPRN_IVOR1, BOOKE_IVOR0_OFF + STRIDE_SIZE * 1) ;\
	RESTORE_SPR(SPRN_IVOR2, BOOKE_IVOR0_OFF + STRIDE_SIZE * 2) ;\
	RESTORE_SPR(SPRN_IVOR3, BOOKE_IVOR0_OFF + STRIDE_SIZE * 3) ;\
	RESTORE_SPR(SPRN_IVOR4, BOOKE_IVOR0_OFF + STRIDE_SIZE * 4) ;\
	RESTORE_SPR(SPRN_IVOR5, BOOKE_IVOR0_OFF + STRIDE_SIZE * 5) ;\
	RESTORE_SPR(SPRN_IVOR6, BOOKE_IVOR0_OFF + STRIDE_SIZE * 6) ;\
	RESTORE_SPR(SPRN_IVOR7, BOOKE_IVOR0_OFF + STRIDE_SIZE * 7) ;\
	RESTORE_SPR(SPRN_IVOR8, BOOKE_IVOR0_OFF + STRIDE_SIZE * 8) ;\
	RESTORE_SPR(SPRN_IVOR9, BOOKE_IVOR0_OFF + STRIDE_SIZE * 9) ;\
	RESTORE_SPR(SPRN_IVOR10, BOOKE_IVOR0_OFF + STRIDE_SIZE * 10) ;\
	RESTORE_SPR(SPRN_IVOR11, BOOKE_IVOR0_OFF + STRIDE_SIZE * 11) ;\
	RESTORE_SPR(SPRN_IVOR12, BOOKE_IVOR0_OFF + STRIDE_SIZE * 12) ;\
	RESTORE_SPR(SPRN_IVOR13, BOOKE_IVOR0_OFF + STRIDE_SIZE * 13) ;\
	RESTORE_SPR(SPRN_IVOR14, BOOKE_IVOR0_OFF + STRIDE_SIZE * 14) ;\
	RESTORE_SPR(SPRN_IVOR15, BOOKE_IVOR0_OFF + STRIDE_SIZE * 15) ;\
	RESTORE_SPR(SPRN_IVOR35, BOOKE_IVOR0_OFF + STRIDE_SIZE * 35) ;\
	RESTORE_SPR(SPRN_IVOR36, BOOKE_IVOR0_OFF + STRIDE_SIZE * 36) ;\
	RESTORE_SPR(SPRN_IVOR37, BOOKE_IVOR0_OFF + STRIDE_SIZE * 37) ;\
	RESTORE_SPR(SPRN_IVOR38, BOOKE_IVOR0_OFF + STRIDE_SIZE * 38) ;\
	RESTORE_SPR(SPRN_IVOR39, BOOKE_IVOR0_OFF + STRIDE_SIZE * 39) ;\
	RESTORE_SPR(SPRN_IVOR40, BOOKE_IVOR0_OFF + STRIDE_SIZE * 40) ;\
	RESTORE_SPR(SPRN_IVOR41, BOOKE_IVOR0_OFF + STRIDE_SIZE * 41)

/* reset time base to prevent from overflow */
#define DELAY(count)		\
	li	r3, count;	\
	li	r4, 0;		\
	mtspr	SPRN_TBWL, r4;	\
101:	mfspr	r4, SPRN_TBRL;	\
	cmpw	r4, r3;		\
	blt	101b

#define FSL_DIS_ALL_IRQ		\
	mfmsr	r8;			\
	rlwinm	r8, r8, 0, ~MSR_CE;	\
	rlwinm	r8, r8, 0, ~MSR_ME;	\
	rlwinm	r8, r8, 0, ~MSR_EE;	\
	rlwinm	r8, r8, 0, ~MSR_DE;	\
	mtmsr	r8;			\
	isync

#ifndef CONFIG_PPC_E500MC
#define SS_TB		0x00
#define SS_HID		0x08 /* 2 HIDs */
#define SS_IAC		0x10 /* 2 IACs */
#define SS_DAC		0x18 /* 2 DACs */
#define SS_DBCR		0x20 /* 3 DBCRs */
#define SS_PID		0x2c /* 3 PIDs */
#define SS_SPRG		0x38 /* 8 SPRGs */
#define SS_IVOR		0x58 /* 20 interrupt vectors */
#define SS_TCR		0xa8
#define SS_BUCSR	0xac
#define SS_L1CSR	0xb0 /* 2 L1CSRs */
#define SS_MSR		0xb8
#define SS_USPRG	0xbc
#define SS_GPREG	0xc0 /* r12-r31 */
#define SS_LR		0x110
#define SS_CR		0x114
#define SS_SP		0x118
#define SS_CURRENT	0x11c
#define SS_IVPR		0x120
#define SS_BPTR		0x124


#define STATE_SAVE_SIZE 0x128

	.section .data
	.align	5
mpc85xx_sleep_save_area:
	.space	STATE_SAVE_SIZE
ccsrbase_low:
	.long	0
ccsrbase_high:
	.long	0
powmgtreq:
	.long	0

	.section .text
	.align	12

	/*
	 * r3 = high word of physical address of CCSR
	 * r4 = low word of physical address of CCSR
	 * r5 = JOG or deep sleep request
	 *      JOG-0x00200000, deep sleep-0x00100000
	 */
_GLOBAL(mpc85xx_enter_deep_sleep)
	lis	r6, ccsrbase_low@ha
	stw	r4, ccsrbase_low@l(r6)
	lis	r6, ccsrbase_high@ha
	stw	r3, ccsrbase_high@l(r6)

	lis	r6, powmgtreq@ha
	stw	r5, powmgtreq@l(r6)

	lis	r10, mpc85xx_sleep_save_area@h
	ori	r10, r10, mpc85xx_sleep_save_area@l

	mfspr	r5, SPRN_HID0
	mfspr	r6, SPRN_HID1

	stw	r5, SS_HID+0(r10)
	stw	r6, SS_HID+4(r10)

	mfspr	r4, SPRN_IAC1
	mfspr	r5, SPRN_IAC2
	mfspr	r6, SPRN_DAC1
	mfspr	r7, SPRN_DAC2

	stw	r4, SS_IAC+0(r10)
	stw	r5, SS_IAC+4(r10)
	stw	r6, SS_DAC+0(r10)
	stw	r7, SS_DAC+4(r10)

	mfspr	r4, SPRN_DBCR0
	mfspr	r5, SPRN_DBCR1
	mfspr	r6, SPRN_DBCR2

	stw	r4, SS_DBCR+0(r10)
	stw	r5, SS_DBCR+4(r10)
	stw	r6, SS_DBCR+8(r10)

	mfspr	r4, SPRN_PID0
	mfspr	r5, SPRN_PID1
	mfspr	r6, SPRN_PID2

	stw	r4, SS_PID+0(r10)
	stw	r5, SS_PID+4(r10)
	stw	r6, SS_PID+8(r10)

	mfspr	r4, SPRN_SPRG0
	mfspr	r5, SPRN_SPRG1
	mfspr	r6, SPRN_SPRG2
	mfspr	r7, SPRN_SPRG3

	stw	r4, SS_SPRG+0x00(r10)
	stw	r5, SS_SPRG+0x04(r10)
	stw	r6, SS_SPRG+0x08(r10)
	stw	r7, SS_SPRG+0x0c(r10)

	mfspr	r4, SPRN_SPRG4
	mfspr	r5, SPRN_SPRG5
	mfspr	r6, SPRN_SPRG6
	mfspr	r7, SPRN_SPRG7

	stw	r4, SS_SPRG+0x10(r10)
	stw	r5, SS_SPRG+0x14(r10)
	stw	r6, SS_SPRG+0x18(r10)
	stw	r7, SS_SPRG+0x1c(r10)

	mfspr	r4, SPRN_IVPR
	stw	r4, SS_IVPR(r10)

	mfspr	r4, SPRN_IVOR0
	mfspr	r5, SPRN_IVOR1
	mfspr	r6, SPRN_IVOR2
	mfspr	r7, SPRN_IVOR3

	stw	r4, SS_IVOR+0x00(r10)
	stw	r5, SS_IVOR+0x04(r10)
	stw	r6, SS_IVOR+0x08(r10)
	stw	r7, SS_IVOR+0x0c(r10)

	mfspr	r4, SPRN_IVOR4
	mfspr	r5, SPRN_IVOR5
	mfspr	r6, SPRN_IVOR6
	mfspr	r7, SPRN_IVOR7

	stw	r4, SS_IVOR+0x10(r10)
	stw	r5, SS_IVOR+0x14(r10)
	stw	r6, SS_IVOR+0x18(r10)
	stw	r7, SS_IVOR+0x1c(r10)

	mfspr	r4, SPRN_IVOR8
	mfspr	r5, SPRN_IVOR9
	mfspr	r6, SPRN_IVOR10
	mfspr	r7, SPRN_IVOR11

	stw	r4, SS_IVOR+0x20(r10)
	stw	r5, SS_IVOR+0x24(r10)
	stw	r6, SS_IVOR+0x28(r10)
	stw	r7, SS_IVOR+0x2c(r10)

	mfspr	r4, SPRN_IVOR12
	mfspr	r5, SPRN_IVOR13
	mfspr	r6, SPRN_IVOR14
	mfspr	r7, SPRN_IVOR15

	stw	r4, SS_IVOR+0x30(r10)
	stw	r5, SS_IVOR+0x34(r10)
	stw	r6, SS_IVOR+0x38(r10)
	stw	r7, SS_IVOR+0x3c(r10)

	mfspr	r4, SPRN_IVOR32
	mfspr	r5, SPRN_IVOR33
	mfspr	r6, SPRN_IVOR34
	mfspr	r7, SPRN_IVOR35

	stw	r4, SS_IVOR+0x40(r10)
	stw	r5, SS_IVOR+0x44(r10)
	stw	r6, SS_IVOR+0x48(r10)
	stw	r7, SS_IVOR+0x4c(r10)

	mfspr	r4, SPRN_TCR
	mfspr	r5, SPRN_BUCSR
	mfspr	r6, SPRN_L1CSR0
	mfspr	r7, SPRN_L1CSR1
	mfspr	r8, SPRN_USPRG0

	stw	r4, SS_TCR(r10)
	stw	r5, SS_BUCSR(r10)
	stw	r6, SS_L1CSR+0(r10)
	stw	r7, SS_L1CSR+4(r10)
	stw	r8, SS_USPRG+0(r10)

	stmw	r12, SS_GPREG(r10)

	mfmsr	r4
	mflr	r5
	mfcr	r6

	stw	r4, SS_MSR(r10)
	stw	r5, SS_LR(r10)
	stw	r6, SS_CR(r10)
	stw	r1, SS_SP(r10)
	stw	r2, SS_CURRENT(r10)

1:	mftbu	r4
	mftb	r5
	mftbu	r6
	cmpw	r4, r6
	bne	1b

	stw	r4, SS_TB+0(r10)
	stw	r5, SS_TB+4(r10)

	lis	r5, ccsrbase_low@ha
	lwz	r4, ccsrbase_low@l(r5)
	lis	r5, ccsrbase_high@ha
	lwz	r3, ccsrbase_high@l(r5)

	/* Disable machine checks and critical exceptions */
	mfmsr	r5
	rlwinm	r5, r5, 0, ~MSR_CE
	rlwinm	r5, r5, 0, ~MSR_ME
	mtmsr	r5
	isync

	/* Use TLB1[15] to map the CCSR at 0xf0000000 */
	lis	r5, 0x100f
	mtspr	SPRN_MAS0, r5
	lis	r5, 0xc000
	ori	r5, r5, 0x0500
	mtspr	SPRN_MAS1, r5
	lis	r5, 0xf000
	ori	r5, r5, 0x000a
	mtspr	SPRN_MAS2, r5
	rlwinm	r5, r4, 0, 0xfffff000
	ori	r5, r5, 0x0005
	mtspr	SPRN_MAS3, r5
	mtspr	SPRN_MAS7, r3
	isync
	tlbwe
	isync

	lis	r3, 0xf000
	lwz	r4, 0x20(r3)
	stw	r4, SS_BPTR(r10)

	lis	r3, 0xf002	/* L2 cache controller at CCSR+0x20000 */
	bl	flush_disable_L2
	bl	__flush_disable_L1

	/* Enable I-cache, so as not to upset the bus
	 * with our loop.
	 */

	mfspr	r4, SPRN_L1CSR1
	ori	r4, r4, 1
	mtspr	SPRN_L1CSR1, r4
	isync

	/* Set boot page translation */
	lis	r3, 0xf000
	lis	r4, (mpc85xx_deep_resume - PAGE_OFFSET)@h
	ori	r4, r4, (mpc85xx_deep_resume - PAGE_OFFSET)@l
	rlwinm	r4, r4, 20, 0x000fffff
	oris	r4, r4, 0x8000
	stw	r4, 0x20(r3)
	lwz	r4, 0x20(r3)		/* read-back to flush write */
	twi	0, r4, 0
	isync

	/* Disable the decrementer */
	mfspr	r4, SPRN_TCR
	rlwinm	r4, r4, 0, ~TCR_DIE
	mtspr	SPRN_TCR, r4

	mfspr	r4, SPRN_TSR
	oris	r4, r4, TSR_DIS@h
	mtspr	SPRN_TSR, r4

	/* set PMRCCR[VRCNT] to wait power stable for 40ms */
	lis	r3, 0xf00e
	lwz	r4, 0x84(r3)
	clrlwi	r4, r4, 16
	oris	r4, r4, 0x12a3
	stw	r4, 0x84(r3)
	lwz	r4, 0x84(r3)

	/* set deep sleep bit in POWMGTSCR */
	lis	r3, powmgtreq@ha
	lwz	r8, powmgtreq@l(r3)

	lis	r3, 0xf00e
	lwz	r4, 0x80(r3)
	or	r4, r4, r8
	stw	r4, 0x80(r3)
	lwz	r4, 0x80(r3)		/* read-back to flush write */
	twi	0, r4, 0
	isync

	mftb	r5
1:	/* spin until either we enter deep sleep, or the sleep process is
	 * aborted due to a pending wakeup event.  Wait some time between
	 * accesses, so we don't flood the bus and prevent the pmc from
	 * detecting an idle system.
	 */

	mftb	r4
	subf	r7, r5, r4
	cmpwi	r7, 1000
	blt	1b
	mr	r5, r4

	lwz	r6, 0x80(r3)
	andis.	r6, r6, 0x0010
	bne	1b
	b	2f

2:	mfspr	r4, SPRN_PIR
	andi.	r4, r4, 1
99:	bne	99b

	/* Establish a temporary 64MB 0->0 mapping in TLB1[1]. */
	lis	r4, 0x1001
	mtspr	SPRN_MAS0, r4
	lis	r4, 0xc000
	ori	r4, r4, 0x0800
	mtspr	SPRN_MAS1, r4
	li	r4, 0
	mtspr	SPRN_MAS2, r4
	li	r4, 0x0015
	mtspr	SPRN_MAS3, r4
	li	r4, 0
	mtspr	SPRN_MAS7, r4
	isync
	tlbwe
	isync

	lis	r3, (3f - PAGE_OFFSET)@h
	ori	r3, r3, (3f - PAGE_OFFSET)@l
	mtctr	r3
	bctr

	/* Locate the resume vector in the last word of the current page. */
	. = mpc85xx_enter_deep_sleep + 0xffc
mpc85xx_deep_resume:
	b	2b

3:
	/* Restore the contents of TLB1[0].  It is assumed that it covers
	 * the currently executing code and the sleep save area, and that
	 * it does not alias our temporary mapping (which is at virtual zero).
	 */
	lis	r3, (TLBCAM - PAGE_OFFSET)@h
	ori	r3, r3, (TLBCAM - PAGE_OFFSET)@l

	lwz	r4, 0(r3)
	lwz	r5, 4(r3)
	lwz	r6, 8(r3)
	lwz	r7, 12(r3)
	lwz	r8, 16(r3)

	mtspr	SPRN_MAS0, r4
	mtspr	SPRN_MAS1, r5
	mtspr	SPRN_MAS2, r6
	mtspr	SPRN_MAS3, r7
	mtspr	SPRN_MAS7, r8

	isync
	tlbwe
	isync

	/* Access the ccsrbase address with TLB1[0] */
	lis	r5, ccsrbase_low@ha
	lwz	r4, ccsrbase_low@l(r5)
	lis	r5, ccsrbase_high@ha
	lwz	r3, ccsrbase_high@l(r5)

	/* Use TLB1[15] to map the CCSR at 0xf0000000 */
	lis	r5, 0x100f
	mtspr	SPRN_MAS0, r5
	lis	r5, 0xc000
	ori	r5, r5, 0x0500
	mtspr	SPRN_MAS1, r5
	lis	r5, 0xf000
	ori	r5, r5, 0x000a
	mtspr	SPRN_MAS2, r5
	rlwinm	r5, r4, 0, 0xfffff000
	ori	r5, r5, 0x0005
	mtspr	SPRN_MAS3, r5
	mtspr	SPRN_MAS7, r3
	isync
	tlbwe
	isync

	lis	r3, 0xf002	/* L2 cache controller at CCSR+0x20000 */
	bl	invalidate_enable_L2

	/* Access the MEM(r10) with TLB1[0] */
	lis	r10, mpc85xx_sleep_save_area@h
	ori	r10, r10, mpc85xx_sleep_save_area@l

	lis	r3, 0xf000
	lwz	r4, SS_BPTR(r10)
	stw	r4, 0x20(r3)		/* restore BPTR */

	/* Program shift running space to PAGE_OFFSET */
	mfmsr	r3
	lis	r4, 1f@h
	ori	r4, r4, 1f@l

	mtsrr1	r3
	mtsrr0	r4
	rfi

1:	/* Restore the rest of TLB1, in ascending order so that
	 * the TLB1[1] gets invalidated first.
	 *
	 * XXX: It's better to invalidate the temporary mapping
	 * TLB1[15] for CCSR before restore any TLB1 entry include 0.
	 */
	lis	r4, 0x100f
	mtspr	SPRN_MAS0, r4
	lis	r4, 0
	mtspr	SPRN_MAS1, r4
	isync
	tlbwe
	isync

	lis	r3, (TLBCAM + 5*4 - 4)@h
	ori	r3, r3, (TLBCAM + 5*4 - 4)@l
	li	r4, 15
	mtctr	r4

2:
	lwz	r5, 4(r3)
	lwz	r6, 8(r3)
	lwz	r7, 12(r3)
	lwz	r8, 16(r3)
	lwzu	r9, 20(r3)

	mtspr	SPRN_MAS0, r5
	mtspr	SPRN_MAS1, r6
	mtspr	SPRN_MAS2, r7
	mtspr	SPRN_MAS3, r8
	mtspr	SPRN_MAS7, r9

	isync
	tlbwe
	isync
	bdnz	2b

	lis	r10, mpc85xx_sleep_save_area@h
	ori	r10, r10, mpc85xx_sleep_save_area@l

	lwz	r5, SS_HID+0(r10)
	lwz	r6, SS_HID+4(r10)

	isync
	mtspr	SPRN_HID0, r5
	isync

	msync
	mtspr	SPRN_HID1, r6
	isync

	lwz	r4, SS_IAC+0(r10)
	lwz	r5, SS_IAC+4(r10)
	lwz	r6, SS_DAC+0(r10)
	lwz	r7, SS_DAC+4(r10)

	mtspr	SPRN_IAC1, r4
	mtspr	SPRN_IAC2, r5
	mtspr	SPRN_DAC1, r6
	mtspr	SPRN_DAC2, r7

	lwz	r4, SS_DBCR+0(r10)
	lwz	r5, SS_DBCR+4(r10)
	lwz	r6, SS_DBCR+8(r10)

	mtspr	SPRN_DBCR0, r4
	mtspr	SPRN_DBCR1, r5
	mtspr	SPRN_DBCR2, r6

	lwz	r4, SS_PID+0(r10)
	lwz	r5, SS_PID+4(r10)
	lwz	r6, SS_PID+8(r10)

	mtspr	SPRN_PID0, r4
	mtspr	SPRN_PID1, r5
	mtspr	SPRN_PID2, r6

	lwz	r4, SS_SPRG+0x00(r10)
	lwz	r5, SS_SPRG+0x04(r10)
	lwz	r6, SS_SPRG+0x08(r10)
	lwz	r7, SS_SPRG+0x0c(r10)

	mtspr	SPRN_SPRG0, r4
	mtspr	SPRN_SPRG1, r5
	mtspr	SPRN_SPRG2, r6
	mtspr	SPRN_SPRG3, r7

	lwz	r4, SS_SPRG+0x10(r10)
	lwz	r5, SS_SPRG+0x14(r10)
	lwz	r6, SS_SPRG+0x18(r10)
	lwz	r7, SS_SPRG+0x1c(r10)

	mtspr	SPRN_SPRG4, r4
	mtspr	SPRN_SPRG5, r5
	mtspr	SPRN_SPRG6, r6
	mtspr	SPRN_SPRG7, r7

	lwz	r4, SS_IVPR(r10)
	mtspr	SPRN_IVPR, r4

	lwz	r4, SS_IVOR+0x00(r10)
	lwz	r5, SS_IVOR+0x04(r10)
	lwz	r6, SS_IVOR+0x08(r10)
	lwz	r7, SS_IVOR+0x0c(r10)

	mtspr	SPRN_IVOR0, r4
	mtspr	SPRN_IVOR1, r5
	mtspr	SPRN_IVOR2, r6
	mtspr	SPRN_IVOR3, r7

	lwz	r4, SS_IVOR+0x10(r10)
	lwz	r5, SS_IVOR+0x14(r10)
	lwz	r6, SS_IVOR+0x18(r10)
	lwz	r7, SS_IVOR+0x1c(r10)

	mtspr	SPRN_IVOR4, r4
	mtspr	SPRN_IVOR5, r5
	mtspr	SPRN_IVOR6, r6
	mtspr	SPRN_IVOR7, r7

	lwz	r4, SS_IVOR+0x20(r10)
	lwz	r5, SS_IVOR+0x24(r10)
	lwz	r6, SS_IVOR+0x28(r10)
	lwz	r7, SS_IVOR+0x2c(r10)

	mtspr	SPRN_IVOR8, r4
	mtspr	SPRN_IVOR9, r5
	mtspr	SPRN_IVOR10, r6
	mtspr	SPRN_IVOR11, r7

	lwz	r4, SS_IVOR+0x30(r10)
	lwz	r5, SS_IVOR+0x34(r10)
	lwz	r6, SS_IVOR+0x38(r10)
	lwz	r7, SS_IVOR+0x3c(r10)

	mtspr	SPRN_IVOR12, r4
	mtspr	SPRN_IVOR13, r5
	mtspr	SPRN_IVOR14, r6
	mtspr	SPRN_IVOR15, r7

	lwz	r4, SS_IVOR+0x40(r10)
	lwz	r5, SS_IVOR+0x44(r10)
	lwz	r6, SS_IVOR+0x48(r10)
	lwz	r7, SS_IVOR+0x4c(r10)

	mtspr	SPRN_IVOR32, r4
	mtspr	SPRN_IVOR33, r5
	mtspr	SPRN_IVOR34, r6
	mtspr	SPRN_IVOR35, r7

	lwz	r4, SS_TCR(r10)
	lwz	r5, SS_BUCSR(r10)
	lwz	r6, SS_L1CSR+0(r10)
	lwz	r7, SS_L1CSR+4(r10)
	lwz	r8, SS_USPRG+0(r10)

	mtspr	SPRN_TCR, r4
	mtspr	SPRN_BUCSR, r5

	msync
	isync
	mtspr	SPRN_L1CSR0, r6
	isync

	mtspr	SPRN_L1CSR1, r7
	isync

	mtspr	SPRN_USPRG0, r8

	lmw	r12, SS_GPREG(r10)

	lwz	r1, SS_SP(r10)
	lwz	r2, SS_CURRENT(r10)
	lwz	r4, SS_MSR(r10)
	lwz	r5, SS_LR(r10)
	lwz	r6, SS_CR(r10)

	msync
	mtmsr	r4
	isync

	mtlr	r5
	mtcr	r6

	li	r4, 0
	mtspr	SPRN_TBWL, r4

	lwz	r4, SS_TB+0(r10)
	lwz	r5, SS_TB+4(r10)

	mtspr	SPRN_TBWU, r4
	mtspr	SPRN_TBWL, r5

	lis	r3, 1
	mtdec	r3

	blr

#else /* CONFIG_PPC_E500MC */

	.section .data
	.align	6
regs_buffer:
	.space BUFFER_SIZE

	.section .text
/*
 * Save CPU registers
 * r3 : the base address of the buffer which stores the values of registers
 */
e5500_cpu_state_save:
	/* store the base address to r10 */
	mr	r10, r3

	SAVE_ALL_GPR
	SAVE_ALL_SPRG
	SAVE_ALL_IVOR

	SAVE_SPR(SPRN_IVPR, BOOKE_IVPR_OFF)
	SAVE_SPR(SPRN_PID0, BOOKE_PID0_OFF)
	SAVE_SPR(SPRN_EPCR, BOOKE_EPCR_OFF)
	SAVE_SPR(SPRN_HID0, BOOKE_HID0_OFF)
	SAVE_SPR(SPRN_PIR, BOOKE_PIR_OFF)
	SAVE_SPR(SPRN_BUCSR, BOOKE_BUCSR_OFF)
1:
	mfspr	r5, SPRN_TBRU
	mfspr	r4, SPRN_TBRL
	SAVE_GPR(r5, BOOKE_TBU_OFF)
	SAVE_GPR(r4, BOOKE_TBL_OFF)
	mfspr	r3, SPRN_TBRU
	cmpw	r3, r5
	bne	1b

	blr

/*
 * Restore CPU registers
 * r3 : the base address of the buffer which stores the values of registers
 */
e5500_cpu_state_restore:
	/* store the base address to r10 */
	mr	r10, r3

	RESTORE_ALL_GPR
	RESTORE_ALL_SPRG
	RESTORE_ALL_IVOR

	RESTORE_SPR(SPRN_IVPR, BOOKE_IVPR_OFF)
	RESTORE_SPR(SPRN_PID0, BOOKE_PID0_OFF)
	RESTORE_SPR(SPRN_EPCR, BOOKE_EPCR_OFF)
	RESTORE_SPR(SPRN_HID0, BOOKE_HID0_OFF)
	RESTORE_SPR(SPRN_PIR, BOOKE_PIR_OFF)
	RESTORE_SPR(SPRN_BUCSR, BOOKE_BUCSR_OFF)

	li	r0, 0
	mtspr	SPRN_TBWL, r0
	RESTORE_SPR(SPRN_TBWU, BOOKE_TBU_OFF)
	RESTORE_SPR(SPRN_TBWL, BOOKE_TBL_OFF)

	blr

#define CPC_CPCCSR0		0x0
#define CPC_CPCCSR0_CPCFL	0x800

/*
 * Flush the CPC cache.
 * r3 : the base address of CPC
 */
flush_cpc_cache:
	lwz	r6, CPC_CPCCSR0(r3)
	ori	r6, r6, CPC_CPCCSR0_CPCFL
	stw	r6, CPC_CPCCSR0(r3)
	sync

	/* Wait until completing the flush */
1:	lwz	r6, CPC_CPCCSR0(r3)
	andi.	r6, r6, CPC_CPCCSR0_CPCFL
	bne	1b

	blr

/*
 * the last stage to enter deep sleep
 *
 */
	.align 6
_GLOBAL(fsl_dp_enter_low)
deepsleep_start:
	LOAD_REG_ADDR(r9, buf_tmp)
	/* save the return address and MSR */
	mflr	r8
	PPC_STL r8, 0(r9)
	mfmsr	r8
	PPC_STL r8, 8(r9)
	mfspr	r8, SPRN_TCR
	PPC_STL r8, 16(r9)
	mfcr	r8
	PPC_STL	r8, 24(r9)
	li	r8, 0
	mtspr	SPRN_TCR, r8

	/* save the parameters */
	PPC_STL	r3, 32(r9)
	PPC_STL	r4, 40(r9)
	PPC_STL	r5, 48(r9)
	PPC_STL	r6, 56(r9)

	LOAD_REG_ADDR(r3, regs_buffer)
	bl	e5500_cpu_state_save

	/* restore the parameters */
	LOAD_REG_ADDR(r9, buf_tmp)
	PPC_LL	r31, 32(r9)
	PPC_LL	r30, 40(r9)
	PPC_LL	r29, 48(r9)
	PPC_LL	r28, 56(r9)

	/* flush caches inside CPU */
	LOAD_REG_ADDR(r3, cur_cpu_spec)
	PPC_LL	r3, 0(r3)
	PPC_LL	r3, CPU_FLUSH_CACHES(r3)
	PPC_LCMPI  0, r3, 0
	beq	6f
#ifdef CONFIG_PPC64
	PPC_LL	r3, 0(r3)
#endif
	mtctr	r3
	bctrl
6:
	/* Flush the CPC cache */
#define CPC_OFFSET	0x10000
	mr	r3, r31
	addis	r3, r3, CPC_OFFSET@h
	bl	flush_cpc_cache

	/* prefecth TLB */
#define CCSR_GPIO1_GPDAT	0x130008
#define CCSR_GPIO1_GPDAT_29	0x4
	LOAD_REG_IMMEDIATE(r11, CCSR_GPIO1_GPDAT)
	add	r11, r31, r11
	lwz	r10, 0(r11)

#define CCSR_RCPM_PCPH15SETR	0xe20b4
#define CCSR_RCPM_PCPH15SETR_CORE0	0x1
	LOAD_REG_IMMEDIATE(r12, CCSR_RCPM_PCPH15SETR)
	add	r12, r31, r12
	lwz	r10, 0(r12)

#define CCSR_DDR_SDRAM_CFG_2	0x8114
#define CCSR_DDR_SDRAM_CFG_2_FRC_SR	0x80000000
	LOAD_REG_IMMEDIATE(r13, CCSR_DDR_SDRAM_CFG_2)
	add	r13, r31, r13
	lwz	r10, 0(r13)

#define	DCSR_EPU_EPGCR		0x000
#define DCSR_EPU_EPGCR_GCE	0x80000000
	li	r14, DCSR_EPU_EPGCR
	add	r14, r30, r14
	lwz	r10, 0(r14)

#define	DCSR_EPU_EPECR15	0x33C
#define DCSR_EPU_EPECR15_IC0	0x80000000
	li	r15, DCSR_EPU_EPECR15
	add	r15, r30, r15
	lwz	r10, 0(r15)

#define CCSR_SCFG_QMIFRSTCR		0xfc40c
#define CCSR_SCFG_QMIFRSTCR_QMIFRST	0x80000000
	LOAD_REG_IMMEDIATE(r16, CCSR_SCFG_QMIFRSTCR)
	add	r16, r31, r16
	lwz	r10, 0(r16)

/*
 * There are two kind of register maps, one for T1040QDS and
 * the other for T104xRDB.
 */
#define T104XRDB_CPLD_MISCCSR		0x17
#define T104XRDB_CPLD_MISCCSR_SLEEPEN	0x40
#define T1040QDS_QIXIS_PWR_CTL2		0x21
#define T1040QDS_QIXIS_PWR_CTL2_PCTL	0x2
	li	r3, T1040QDS_QIXIS_PWR_CTL2
	PPC_LCMPI  0, r28, T1040QDS_TETRA_FLAG
	beq	20f
	li	r3, T104XRDB_CPLD_MISCCSR
20:	add	r29, r29, r3
	lbz	r10, 0(r29)
	sync

	LOAD_REG_ADDR(r8, deepsleep_start)
	LOAD_REG_ADDR(r9, deepsleep_end)

	/* prefecth code to cache so that executing code after disable DDR */
1:	icbtls	2, 0, r8
	addi	r8, r8, 64
	cmpw	r8, r9
	blt	1b
	sync

	FSL_DIS_ALL_IRQ

	/*
	 * Place DDR controller in self refresh mode.
	 * From here on, can't access DDR any more.
	 */
	lwz	r10, 0(r13)
	oris	r10, r10, CCSR_DDR_SDRAM_CFG_2_FRC_SR@h
	stw	r10, 0(r13)
	lwz	r10, 0(r13)
	sync

	DELAY(500)

	/*
	 * Enable deep sleep signals by write external CPLD/FPGA register.
	 * The bootloader will disable them when wakeup from deep sleep.
	 */
	lbz	r10, 0(r29)
	li	r3, T1040QDS_QIXIS_PWR_CTL2_PCTL
	PPC_LCMPI  0, r28, T1040QDS_TETRA_FLAG
	beq	22f
	li	r3, T104XRDB_CPLD_MISCCSR_SLEEPEN
22:	or	r10, r10, r3
	stb	r10, 0(r29)
	lbz	r10, 0(r29)
	sync

	/*
	 * Set GPIO1_29 to lock the signal MCKE down during deep sleep.
	 * The bootloader will clear it when wakeup.
	 */
	lwz	r10, 0(r11)
	ori	r10, r10, CCSR_GPIO1_GPDAT_29
	stw	r10, 0(r11)
	lwz	r10, 0(r11)

	DELAY(100)

	/* Reset QMan system bus interface */
	lwz	r10, 0(r16)
	oris	r10, r10, CCSR_SCFG_QMIFRSTCR_QMIFRST@h
	stw	r10, 0(r16)
	lwz	r10, 0(r16)

	/* Enable all EPU Counters */
	li	r10, 0
	oris	r10, r10, DCSR_EPU_EPGCR_GCE@h
	stw	r10, 0(r14)
	lwz	r10, 0(r14)

	/* Enable SCU15 to trigger on RCPM Concentrator 0 */
	lwz	r10, 0(r15)
	oris	r10, r10, DCSR_EPU_EPECR15_IC0@h
	stw	r10, 0(r15)
	lwz	r10, 0(r15)

	/* put Core0 in PH15 mode, trigger EPU FSM */
	lwz	r10, 0(r12)
	ori	r10, r10, CCSR_RCPM_PCPH15SETR_CORE0
	stw	r10, 0(r12)
2:
	b 2b

	/*
	 * Leave some space to prevent prefeching instruction
	 * beyond deepsleep_end. The space also can be used as heap.
	 */
buf_tmp:
	.space 128
	.align 6
deepsleep_end:

	.align 12
#ifdef CONFIG_PPC32
_GLOBAL(fsl_booke_deep_sleep_resume)
	/* disable interrupts */
	FSL_DIS_ALL_IRQ

#define ENTRY_DEEPSLEEP_SETUP
#define ENTRY_MAPPING_BOOT_SETUP
#include <../../kernel/85xx_entry_mapping.S>
#undef ENTRY_DEEPSLEEP_SETUP
#undef ENTRY_MAPPING_BOOT_SETUP

	li	r3, 0
	mfspr   r4, SPRN_PIR
	bl	call_setup_cpu

	/* Load each CAM entry */
	LOAD_REG_ADDR(r3, tlbcam_index)
	lwz	r3, 0(r3)
	mtctr	r3
	li	r9, 0
3:	mr	r3, r9
	bl	loadcam_entry
	addi	r9, r9, 1
	bdnz	3b

	/* restore cpu registers */
	LOAD_REG_ADDR(r3, regs_buffer)
	bl	e5500_cpu_state_restore

	/* restore return address */
	LOAD_REG_ADDR(r3, buf_tmp)
	lwz	r4, 16(r3)
	mtspr	SPRN_TCR, r4
	lwz	r4, 0(r3)
	mtlr	r4
	lwz	r4, 8(r3)
	mtmsr	r4
	lwz	r4, 24(r3)
	mtcr	r4

	blr

#else /* CONFIG_PPC32 */

_GLOBAL(fsl_booke_deep_sleep_resume)
	/* disable interrupts */
	FSL_DIS_ALL_IRQ

	/* switch to 64-bit mode */
	bl	.enable_64b_mode

	/* set TOC pointer */
	bl	.relative_toc

	/* setup initial TLBs, switch to kernel space ... */
	bl	.start_initialization_book3e

	/* address space changed, set TOC pointer again */
	bl	.relative_toc

	/* call a cpu state restore handler */
	LOAD_REG_ADDR(r23, cur_cpu_spec)
	ld	r23,0(r23)
	ld	r23,CPU_SPEC_RESTORE(r23)
	cmpdi	0,r23,0
	beq	1f
	ld	r23,0(r23)
	mtctr	r23
	bctrl
1:
	LOAD_REG_ADDR(r3, regs_buffer)
	bl	e5500_cpu_state_restore

	/* Load each CAM entry */
	LOAD_REG_ADDR(r3, tlbcam_index)
	lwz	r3, 0(r3)
	mtctr	r3
	li	r0, 0
3:	mr	r3, r0
	bl	loadcam_entry
	addi	r0, r0, 1
	bdnz	3b

	/* restore return address */
	LOAD_REG_ADDR(r3, buf_tmp)
	ld	r4, 16(r3)
	mtspr	SPRN_TCR, r4
	ld	r4, 0(r3)
	mtlr	r4
	ld	r4, 8(r3)
	mtmsr	r4
	ld	r4, 24(r3)
	mtcr	r4

	blr

#endif /* CONFIG_PPC32 */

#endif
