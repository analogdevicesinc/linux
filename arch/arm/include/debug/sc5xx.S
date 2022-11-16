/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Assembly Macros for interacting with UART
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */



	.macro	addruart, rp, rv, tmp
	ldr	\rp, = 0x31003000			@ System peripherals (phys address)
	ldr	\rv, = 0xF4003000			@ System peripherals (virt address)
	.endm

	.macro	senduart,rd,rx
	str	\rd, [\rx, #(0x24)]			@ Write to Transmitter Holding Register
	.endm

	.macro	waituarttxrdy,rd,rx
1001:	ldr	\rd, [\rx, #(8)]			@ Read Status Register
	tst	\rd, #(0x20)				@ DBGU_TXRDY = 1 when ready to transmit
	beq	1001b
	.endm


	.macro	busyuart,rd,rx
	.endm

	.macro	add_softbp,tmp
	ldr     \tmp,=0
123:
	tst	\tmp, #(1)
	beq	123b
	.endm
