/* Copyright 2014 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DPA_SYS_PPC32_H
#define DPA_SYS_PPC32_H

/* Implementation of PowerPC 32 bit specific routines */

/* TODO: NB, we currently assume that hwsync() and lwsync() imply compiler
 * barriers and that dcb*() won't fall victim to compiler or execution
 * reordering with respect to other code/instructions that manipulate the same
 * cacheline. */
#define hwsync() __asm__ __volatile__ ("sync" : : : "memory")
#define lwsync() __asm__ __volatile__ (stringify_in_c(LWSYNC) : : : "memory")
#define dcbf(p) __asm__ __volatile__ ("dcbf 0,%0" : : "r" (p) : "memory")
#define dcbt_ro(p) __asm__ __volatile__ ("dcbt 0,%0" : : "r" (p))
#define dcbt_rw(p) __asm__ __volatile__ ("dcbtst 0,%0" : : "r" (p))
#define dcbi(p) dcbf(p)

#define dcbzl(p) __asm__ __volatile__ ("dcbzl 0,%0" : : "r" (p))
#define dcbz_64(p) dcbzl(p)
#define dcbf_64(p) dcbf(p)

/* Commonly used combo */
#define dcbit_ro(p) \
	do { \
		dcbi(p); \
		dcbt_ro(p); \
	} while (0)

static inline u64 mfatb(void)
{
	u32 hi, lo, chk;
	do {
		hi = mfspr(SPRN_ATBU);
		lo = mfspr(SPRN_ATBL);
		chk = mfspr(SPRN_ATBU);
	} while (unlikely(hi != chk));
	return ((u64)hi << 32) | (u64)lo;
}

#endif
