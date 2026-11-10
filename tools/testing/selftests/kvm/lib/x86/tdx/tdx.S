/* SPDX-License-Identifier: GPL-2.0-only */

.globl __tdvmcall
/*
 * u64 __tdvmcall(u64 fn, u64 r12, u64 r13, u64 r14, u64 r15)
 */
__tdvmcall:
	push %r12
	push %r13
	push %r14
	push %r15

	/*
	 * Map C ABI fast arguments to TDX GHCI payload registers
	 * RDI=fn, RSI=r12, RDX=r13, RCX=r14, R8=r15
	 */
	mov %rdi, %r11
	mov %rsi, %r12
	mov %rdx, %r13
	mov %rcx, %r14
	mov %r8,  %r15

	xorl %eax, %eax		/* TDCALL leaf 0: TDG.VP.VMCALL */
	mov $0xFC00, %rcx	/* Expose R10-R15 */
	xorl %r10d, %r10d	/* Standard GHCI call */

	/* TDCALL instruction */
	.byte 0x66, 0x0f, 0x01, 0xcc

	/* Extract status */
	mov %r10, %rax

	pop %r15
	pop %r14
	pop %r13
	pop %r12
	ret

.section .note.GNU-stack,"",%progbits
