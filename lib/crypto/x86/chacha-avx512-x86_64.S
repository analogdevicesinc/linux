/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * ChaCha 256-bit cipher algorithm, x64 AVX-512 functions
 *
 * Copyright (C) 2018 Martin Willi
 */

#include <linux/linkage.h>

.section	.rodata.cst32.CTR2BL, "aM", @progbits, 32
.align 32
CTR2BL:	.octa 0x00000000000000000000000000000000
	.octa 0x00000000000000000000000000000001

.section	.rodata.cst32.CTR4BL, "aM", @progbits, 32
.align 32
CTR4BL:	.octa 0x00000000000000000000000000000002
	.octa 0x00000000000000000000000000000003

.section	.rodata.cst64.CTRBL, "aM", @progbits, 64
.align 64
CTRBL:	.octa 0x00000003000000020000000100000000
	.octa 0x00000007000000060000000500000004
	.octa 0x0000000b0000000a0000000900000008
	.octa 0x0000000f0000000e0000000d0000000c

.text

.macro _define_Vi	i
.if VL == 32
	.set	V\i,		%ymm\i
.elseif VL == 64
	.set	V\i,		%zmm\i
.else
	.error	"Unsupported vector length"
.endif
.endm

.macro _define_aliases
	# alias V0-V31 to the vector registers of the selected width VL
.irp i, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31
	_define_Vi	\i
.endr
.endm

SYM_FUNC_START(chacha_0to2blocks_avx512vl)
	# %rdi: Input state matrix, s
	# %rsi: output data, o
	# %rdx: input data, i
	# %rcx: input/output data length in bytes: 0 to 2 blocks inclusive
	# %r8d: nrounds

	# This function encrypts two ChaCha blocks by loading the state
	# matrix twice across four AVX registers. It performs matrix operations
	# on four words in each matrix in parallel, but requires shuffling to
	# rearrange the words after each round.

	# x0..3[0-2] = s0..3
	vbroadcasti128	0x00(%rdi),%ymm0
	vbroadcasti128	0x10(%rdi),%ymm1
	vbroadcasti128	0x20(%rdi),%ymm2
	vbroadcasti128	0x30(%rdi),%ymm3

	vpaddd		CTR2BL(%rip),%ymm3,%ymm3

	vmovdqa		%ymm0,%ymm8
	vmovdqa		%ymm1,%ymm9
	vmovdqa		%ymm2,%ymm10
	vmovdqa		%ymm3,%ymm11

.Ldoubleround:

	# x0 += x1, x3 = rotl32(x3 ^ x0, 16)
	vpaddd		%ymm1,%ymm0,%ymm0
	vpxord		%ymm0,%ymm3,%ymm3
	vprold		$16,%ymm3,%ymm3

	# x2 += x3, x1 = rotl32(x1 ^ x2, 12)
	vpaddd		%ymm3,%ymm2,%ymm2
	vpxord		%ymm2,%ymm1,%ymm1
	vprold		$12,%ymm1,%ymm1

	# x0 += x1, x3 = rotl32(x3 ^ x0, 8)
	vpaddd		%ymm1,%ymm0,%ymm0
	vpxord		%ymm0,%ymm3,%ymm3
	vprold		$8,%ymm3,%ymm3

	# x2 += x3, x1 = rotl32(x1 ^ x2, 7)
	vpaddd		%ymm3,%ymm2,%ymm2
	vpxord		%ymm2,%ymm1,%ymm1
	vprold		$7,%ymm1,%ymm1

	# x1 = shuffle32(x1, MASK(0, 3, 2, 1))
	vpshufd		$0x39,%ymm1,%ymm1
	# x2 = shuffle32(x2, MASK(1, 0, 3, 2))
	vpshufd		$0x4e,%ymm2,%ymm2
	# x3 = shuffle32(x3, MASK(2, 1, 0, 3))
	vpshufd		$0x93,%ymm3,%ymm3

	# x0 += x1, x3 = rotl32(x3 ^ x0, 16)
	vpaddd		%ymm1,%ymm0,%ymm0
	vpxord		%ymm0,%ymm3,%ymm3
	vprold		$16,%ymm3,%ymm3

	# x2 += x3, x1 = rotl32(x1 ^ x2, 12)
	vpaddd		%ymm3,%ymm2,%ymm2
	vpxord		%ymm2,%ymm1,%ymm1
	vprold		$12,%ymm1,%ymm1

	# x0 += x1, x3 = rotl32(x3 ^ x0, 8)
	vpaddd		%ymm1,%ymm0,%ymm0
	vpxord		%ymm0,%ymm3,%ymm3
	vprold		$8,%ymm3,%ymm3

	# x2 += x3, x1 = rotl32(x1 ^ x2, 7)
	vpaddd		%ymm3,%ymm2,%ymm2
	vpxord		%ymm2,%ymm1,%ymm1
	vprold		$7,%ymm1,%ymm1

	# x1 = shuffle32(x1, MASK(2, 1, 0, 3))
	vpshufd		$0x93,%ymm1,%ymm1
	# x2 = shuffle32(x2, MASK(1, 0, 3, 2))
	vpshufd		$0x4e,%ymm2,%ymm2
	# x3 = shuffle32(x3, MASK(0, 3, 2, 1))
	vpshufd		$0x39,%ymm3,%ymm3

	sub		$2,%r8d
	jnz		.Ldoubleround

	# x0 += s0, x1 += s1, x2 += s2, x3 += s3
	vpaddd		%ymm8,%ymm0,%ymm0
	vpaddd		%ymm9,%ymm1,%ymm1
	vpaddd		%ymm10,%ymm2,%ymm2
	vpaddd		%ymm11,%ymm3,%ymm3

	# XOR up to 2 data blocks with the generated keystream.
	vperm2i128	$0x20, %ymm1, %ymm0, %ymm4
	cmp		$1*32,%rcx
	jl		.Lxorpart2
	vpxor		0*32(%rdx),%ymm4,%ymm4
	vmovdqu		%ymm4,0*32(%rsi)

	vperm2i128	$0x20, %ymm3, %ymm2, %ymm4
	cmp		$2*32,%rcx
	jl		.Lxorpart2
	vpxor		1*32(%rdx),%ymm4,%ymm4
	vmovdqu		%ymm4,1*32(%rsi)

	vperm2i128	$0x31, %ymm1, %ymm0, %ymm4
	cmp		$3*32,%rcx
	jl		.Lxorpart2
	vpxor		2*32(%rdx),%ymm4,%ymm4
	vmovdqu		%ymm4,2*32(%rsi)

	vperm2i128	$0x31, %ymm3, %ymm2, %ymm4
	cmp		$4*32,%rcx
	jl		.Lxorpart2
	vpxor		3*32(%rdx),%ymm4,%ymm4
	vmovdqu		%ymm4,3*32(%rsi)

.Ldone2:
	vzeroupper
	RET

.Lxorpart2:
	# xor remaining bytes from partial register into output
	mov		%rcx,%r9
	and		$~31,%r9
	and		$31,%rcx
	jz		.Ldone2

	mov		$-1,%rax
	bzhi		%rcx,%rax,%rax
	kmovq		%rax,%k1

	vmovdqu8	(%rdx,%r9),%ymm0{%k1}{z}
	vpxor		%ymm4,%ymm0,%ymm0
	vmovdqu8	%ymm0,(%rsi,%r9){%k1}

	jmp		.Ldone2

SYM_FUNC_END(chacha_0to2blocks_avx512vl)

SYM_FUNC_START(chacha_2to4blocks_avx512vl)
	# %rdi: Input state matrix, s
	# %rsi: output data, o
	# %rdx: input data, i
	# %rcx: input/output data length in bytes: 2 to 4 blocks inclusive
	# %r8d: nrounds

	# This function encrypts four ChaCha blocks by loading the state
	# matrix four times across eight AVX registers. It performs matrix
	# operations on four words in two matrices in parallel, sequentially
	# to the operations on the four words of the other two matrices. The
	# required word shuffling has a rather high latency, we can do the
	# arithmetic on two matrix-pairs without much slowdown.

	# x0..3[0-4] = s0..3
	vbroadcasti128	0x00(%rdi),%ymm0
	vbroadcasti128	0x10(%rdi),%ymm1
	vbroadcasti128	0x20(%rdi),%ymm2
	vbroadcasti128	0x30(%rdi),%ymm3

	vmovdqa		%ymm0,%ymm4
	vmovdqa		%ymm1,%ymm5
	vmovdqa		%ymm2,%ymm6
	vmovdqa		%ymm3,%ymm7

	vpaddd		CTR2BL(%rip),%ymm3,%ymm3
	vpaddd		CTR4BL(%rip),%ymm7,%ymm7

	vmovdqa		%ymm0,%ymm11
	vmovdqa		%ymm1,%ymm12
	vmovdqa		%ymm2,%ymm13
	vmovdqa		%ymm3,%ymm14
	vmovdqa		%ymm7,%ymm15

.Ldoubleround4:

	# x0 += x1, x3 = rotl32(x3 ^ x0, 16)
	vpaddd		%ymm1,%ymm0,%ymm0
	vpxord		%ymm0,%ymm3,%ymm3
	vprold		$16,%ymm3,%ymm3

	vpaddd		%ymm5,%ymm4,%ymm4
	vpxord		%ymm4,%ymm7,%ymm7
	vprold		$16,%ymm7,%ymm7

	# x2 += x3, x1 = rotl32(x1 ^ x2, 12)
	vpaddd		%ymm3,%ymm2,%ymm2
	vpxord		%ymm2,%ymm1,%ymm1
	vprold		$12,%ymm1,%ymm1

	vpaddd		%ymm7,%ymm6,%ymm6
	vpxord		%ymm6,%ymm5,%ymm5
	vprold		$12,%ymm5,%ymm5

	# x0 += x1, x3 = rotl32(x3 ^ x0, 8)
	vpaddd		%ymm1,%ymm0,%ymm0
	vpxord		%ymm0,%ymm3,%ymm3
	vprold		$8,%ymm3,%ymm3

	vpaddd		%ymm5,%ymm4,%ymm4
	vpxord		%ymm4,%ymm7,%ymm7
	vprold		$8,%ymm7,%ymm7

	# x2 += x3, x1 = rotl32(x1 ^ x2, 7)
	vpaddd		%ymm3,%ymm2,%ymm2
	vpxord		%ymm2,%ymm1,%ymm1
	vprold		$7,%ymm1,%ymm1

	vpaddd		%ymm7,%ymm6,%ymm6
	vpxord		%ymm6,%ymm5,%ymm5
	vprold		$7,%ymm5,%ymm5

	# x1 = shuffle32(x1, MASK(0, 3, 2, 1))
	vpshufd		$0x39,%ymm1,%ymm1
	vpshufd		$0x39,%ymm5,%ymm5
	# x2 = shuffle32(x2, MASK(1, 0, 3, 2))
	vpshufd		$0x4e,%ymm2,%ymm2
	vpshufd		$0x4e,%ymm6,%ymm6
	# x3 = shuffle32(x3, MASK(2, 1, 0, 3))
	vpshufd		$0x93,%ymm3,%ymm3
	vpshufd		$0x93,%ymm7,%ymm7

	# x0 += x1, x3 = rotl32(x3 ^ x0, 16)
	vpaddd		%ymm1,%ymm0,%ymm0
	vpxord		%ymm0,%ymm3,%ymm3
	vprold		$16,%ymm3,%ymm3

	vpaddd		%ymm5,%ymm4,%ymm4
	vpxord		%ymm4,%ymm7,%ymm7
	vprold		$16,%ymm7,%ymm7

	# x2 += x3, x1 = rotl32(x1 ^ x2, 12)
	vpaddd		%ymm3,%ymm2,%ymm2
	vpxord		%ymm2,%ymm1,%ymm1
	vprold		$12,%ymm1,%ymm1

	vpaddd		%ymm7,%ymm6,%ymm6
	vpxord		%ymm6,%ymm5,%ymm5
	vprold		$12,%ymm5,%ymm5

	# x0 += x1, x3 = rotl32(x3 ^ x0, 8)
	vpaddd		%ymm1,%ymm0,%ymm0
	vpxord		%ymm0,%ymm3,%ymm3
	vprold		$8,%ymm3,%ymm3

	vpaddd		%ymm5,%ymm4,%ymm4
	vpxord		%ymm4,%ymm7,%ymm7
	vprold		$8,%ymm7,%ymm7

	# x2 += x3, x1 = rotl32(x1 ^ x2, 7)
	vpaddd		%ymm3,%ymm2,%ymm2
	vpxord		%ymm2,%ymm1,%ymm1
	vprold		$7,%ymm1,%ymm1

	vpaddd		%ymm7,%ymm6,%ymm6
	vpxord		%ymm6,%ymm5,%ymm5
	vprold		$7,%ymm5,%ymm5

	# x1 = shuffle32(x1, MASK(2, 1, 0, 3))
	vpshufd		$0x93,%ymm1,%ymm1
	vpshufd		$0x93,%ymm5,%ymm5
	# x2 = shuffle32(x2, MASK(1, 0, 3, 2))
	vpshufd		$0x4e,%ymm2,%ymm2
	vpshufd		$0x4e,%ymm6,%ymm6
	# x3 = shuffle32(x3, MASK(0, 3, 2, 1))
	vpshufd		$0x39,%ymm3,%ymm3
	vpshufd		$0x39,%ymm7,%ymm7

	sub		$2,%r8d
	jnz		.Ldoubleround4

	# x0 += s0, x1 += s1, x2 += s2, x3 += s3
	vpaddd		%ymm11,%ymm0,%ymm0
	vpaddd		%ymm12,%ymm1,%ymm1
	vpaddd		%ymm13,%ymm2,%ymm2
	vpaddd		%ymm14,%ymm3,%ymm3
	vpaddd		%ymm11,%ymm4,%ymm4
	vpaddd		%ymm12,%ymm5,%ymm5
	vpaddd		%ymm13,%ymm6,%ymm6
	vpaddd		%ymm15,%ymm7,%ymm7

	# XOR the first 2 data blocks with the generated keystream.
	vperm2i128	$0x20, %ymm1, %ymm0, %ymm8
	vpxor		0*32(%rdx),%ymm8,%ymm8
	vmovdqu		%ymm8,0*32(%rsi)

	vperm2i128	$0x20, %ymm3, %ymm2, %ymm8
	vpxor		1*32(%rdx),%ymm8,%ymm8
	vmovdqu		%ymm8,1*32(%rsi)

	vperm2i128	$0x31, %ymm1, %ymm0, %ymm8
	vpxor		2*32(%rdx),%ymm8,%ymm8
	vmovdqu		%ymm8,2*32(%rsi)

	vperm2i128	$0x31, %ymm3, %ymm2, %ymm8
	vpxor		3*32(%rdx),%ymm8,%ymm8
	vmovdqu		%ymm8,3*32(%rsi)

	# XOR up to 2 additional data blocks with the generated keystream.
	vperm2i128	$0x20, %ymm5, %ymm4, %ymm8
	cmp		$5*32,%rcx
	jl		.Lxorpart4
	vpxor		4*32(%rdx),%ymm8,%ymm8
	vmovdqu		%ymm8,4*32(%rsi)

	vperm2i128	$0x20, %ymm7, %ymm6, %ymm8
	cmp		$6*32,%rcx
	jl		.Lxorpart4
	vpxor		5*32(%rdx),%ymm8,%ymm8
	vmovdqu		%ymm8,5*32(%rsi)

	vperm2i128	$0x31, %ymm5, %ymm4, %ymm8
	cmp		$7*32,%rcx
	jl		.Lxorpart4
	vpxor		6*32(%rdx),%ymm8,%ymm8
	vmovdqu		%ymm8,6*32(%rsi)

	vperm2i128	$0x31, %ymm7, %ymm6, %ymm8
	cmp		$8*32,%rcx
	jl		.Lxorpart4
	vpxor		7*32(%rdx),%ymm8,%ymm8
	vmovdqu		%ymm8,7*32(%rsi)

.Ldone4:
	vzeroupper
	RET

.Lxorpart4:
	# xor remaining bytes from partial register into output
	mov		%rcx,%r9
	and		$~31,%r9
	and		$31,%rcx
	jz		.Ldone4

	mov		$-1,%rax
	bzhi		%rcx,%rax,%rax
	kmovq		%rax,%k1

	vmovdqu8	(%rdx,%r9),%ymm0{%k1}{z}
	vpxor		%ymm8,%ymm0,%ymm0
	vmovdqu8	%ymm0,(%rsi,%r9){%k1}

	jmp		.Ldone4

SYM_FUNC_END(chacha_2to4blocks_avx512vl)

.macro _chacha_words	vl
	.set	VL,		\vl

	_define_aliases

	# %rdi: Input state matrix, s
	# %rsi: output data, o
	# %rdx: input data, i
	# %rcx: input/output data length in bytes: 4 to 8 blocks inclusive for
	#       VL=32, or 8 to 16 blocks inclusive for VL=64.
	# %r8d: nrounds

	# This encrypts VL/4 consecutive ChaCha blocks by loading the state
	# matrix in as many vector registers. Compared to AVX2, this mostly
	# benefits from the new rotate instructions and the additional
	# registers.
	#
	# VL=32 encrypts eight blocks in 256-bit ymm registers.
	# VL=64 encrypts sixteen blocks in 512-bit zmm registers.

	# x0..15[0-VL/4] = s[0..15]
	vpbroadcastd	0x00(%rdi),V0
	vpbroadcastd	0x04(%rdi),V1
	vpbroadcastd	0x08(%rdi),V2
	vpbroadcastd	0x0c(%rdi),V3
	vpbroadcastd	0x10(%rdi),V4
	vpbroadcastd	0x14(%rdi),V5
	vpbroadcastd	0x18(%rdi),V6
	vpbroadcastd	0x1c(%rdi),V7
	vpbroadcastd	0x20(%rdi),V8
	vpbroadcastd	0x24(%rdi),V9
	vpbroadcastd	0x28(%rdi),V10
	vpbroadcastd	0x2c(%rdi),V11
	vpbroadcastd	0x30(%rdi),V12
	vpbroadcastd	0x34(%rdi),V13
	vpbroadcastd	0x38(%rdi),V14
	vpbroadcastd	0x3c(%rdi),V15

	# x12 += counter values, one per block
	vpaddd		CTRBL(%rip),V12,V12

	vmovdqa64	V0,V16
	vmovdqa64	V1,V17
	vmovdqa64	V2,V18
	vmovdqa64	V3,V19
	vmovdqa64	V4,V20
	vmovdqa64	V5,V21
	vmovdqa64	V6,V22
	vmovdqa64	V7,V23
	vmovdqa64	V8,V24
	vmovdqa64	V9,V25
	vmovdqa64	V10,V26
	vmovdqa64	V11,V27
	vmovdqa64	V12,V28
	vmovdqa64	V13,V29
	vmovdqa64	V14,V30
	vmovdqa64	V15,V31

.Ldoubleround\@:
	# x0 += x4, x12 = rotl32(x12 ^ x0, 16)
	vpaddd		V0,V4,V0
	vpxord		V0,V12,V12
	vprold		$16,V12,V12
	# x1 += x5, x13 = rotl32(x13 ^ x1, 16)
	vpaddd		V1,V5,V1
	vpxord		V1,V13,V13
	vprold		$16,V13,V13
	# x2 += x6, x14 = rotl32(x14 ^ x2, 16)
	vpaddd		V2,V6,V2
	vpxord		V2,V14,V14
	vprold		$16,V14,V14
	# x3 += x7, x15 = rotl32(x15 ^ x3, 16)
	vpaddd		V3,V7,V3
	vpxord		V3,V15,V15
	vprold		$16,V15,V15

	# x8 += x12, x4 = rotl32(x4 ^ x8, 12)
	vpaddd		V12,V8,V8
	vpxord		V8,V4,V4
	vprold		$12,V4,V4
	# x9 += x13, x5 = rotl32(x5 ^ x9, 12)
	vpaddd		V13,V9,V9
	vpxord		V9,V5,V5
	vprold		$12,V5,V5
	# x10 += x14, x6 = rotl32(x6 ^ x10, 12)
	vpaddd		V14,V10,V10
	vpxord		V10,V6,V6
	vprold		$12,V6,V6
	# x11 += x15, x7 = rotl32(x7 ^ x11, 12)
	vpaddd		V15,V11,V11
	vpxord		V11,V7,V7
	vprold		$12,V7,V7

	# x0 += x4, x12 = rotl32(x12 ^ x0, 8)
	vpaddd		V0,V4,V0
	vpxord		V0,V12,V12
	vprold		$8,V12,V12
	# x1 += x5, x13 = rotl32(x13 ^ x1, 8)
	vpaddd		V1,V5,V1
	vpxord		V1,V13,V13
	vprold		$8,V13,V13
	# x2 += x6, x14 = rotl32(x14 ^ x2, 8)
	vpaddd		V2,V6,V2
	vpxord		V2,V14,V14
	vprold		$8,V14,V14
	# x3 += x7, x15 = rotl32(x15 ^ x3, 8)
	vpaddd		V3,V7,V3
	vpxord		V3,V15,V15
	vprold		$8,V15,V15

	# x8 += x12, x4 = rotl32(x4 ^ x8, 7)
	vpaddd		V12,V8,V8
	vpxord		V8,V4,V4
	vprold		$7,V4,V4
	# x9 += x13, x5 = rotl32(x5 ^ x9, 7)
	vpaddd		V13,V9,V9
	vpxord		V9,V5,V5
	vprold		$7,V5,V5
	# x10 += x14, x6 = rotl32(x6 ^ x10, 7)
	vpaddd		V14,V10,V10
	vpxord		V10,V6,V6
	vprold		$7,V6,V6
	# x11 += x15, x7 = rotl32(x7 ^ x11, 7)
	vpaddd		V15,V11,V11
	vpxord		V11,V7,V7
	vprold		$7,V7,V7

	# x0 += x5, x15 = rotl32(x15 ^ x0, 16)
	vpaddd		V0,V5,V0
	vpxord		V0,V15,V15
	vprold		$16,V15,V15
	# x1 += x6, x12 = rotl32(x12 ^ x1, 16)
	vpaddd		V1,V6,V1
	vpxord		V1,V12,V12
	vprold		$16,V12,V12
	# x2 += x7, x13 = rotl32(x13 ^ x2, 16)
	vpaddd		V2,V7,V2
	vpxord		V2,V13,V13
	vprold		$16,V13,V13
	# x3 += x4, x14 = rotl32(x14 ^ x3, 16)
	vpaddd		V3,V4,V3
	vpxord		V3,V14,V14
	vprold		$16,V14,V14

	# x10 += x15, x5 = rotl32(x5 ^ x10, 12)
	vpaddd		V15,V10,V10
	vpxord		V10,V5,V5
	vprold		$12,V5,V5
	# x11 += x12, x6 = rotl32(x6 ^ x11, 12)
	vpaddd		V12,V11,V11
	vpxord		V11,V6,V6
	vprold		$12,V6,V6
	# x8 += x13, x7 = rotl32(x7 ^ x8, 12)
	vpaddd		V13,V8,V8
	vpxord		V8,V7,V7
	vprold		$12,V7,V7
	# x9 += x14, x4 = rotl32(x4 ^ x9, 12)
	vpaddd		V14,V9,V9
	vpxord		V9,V4,V4
	vprold		$12,V4,V4

	# x0 += x5, x15 = rotl32(x15 ^ x0, 8)
	vpaddd		V0,V5,V0
	vpxord		V0,V15,V15
	vprold		$8,V15,V15
	# x1 += x6, x12 = rotl32(x12 ^ x1, 8)
	vpaddd		V1,V6,V1
	vpxord		V1,V12,V12
	vprold		$8,V12,V12
	# x2 += x7, x13 = rotl32(x13 ^ x2, 8)
	vpaddd		V2,V7,V2
	vpxord		V2,V13,V13
	vprold		$8,V13,V13
	# x3 += x4, x14 = rotl32(x14 ^ x3, 8)
	vpaddd		V3,V4,V3
	vpxord		V3,V14,V14
	vprold		$8,V14,V14

	# x10 += x15, x5 = rotl32(x5 ^ x10, 7)
	vpaddd		V15,V10,V10
	vpxord		V10,V5,V5
	vprold		$7,V5,V5
	# x11 += x12, x6 = rotl32(x6 ^ x11, 7)
	vpaddd		V12,V11,V11
	vpxord		V11,V6,V6
	vprold		$7,V6,V6
	# x8 += x13, x7 = rotl32(x7 ^ x8, 7)
	vpaddd		V13,V8,V8
	vpxord		V8,V7,V7
	vprold		$7,V7,V7
	# x9 += x14, x4 = rotl32(x4 ^ x9, 7)
	vpaddd		V14,V9,V9
	vpxord		V9,V4,V4
	vprold		$7,V4,V4

	sub		$2,%r8d
	jnz		.Ldoubleround\@

	# x0..15 += s[0..15]
	vpaddd		V16,V0,V0
	vpaddd		V17,V1,V1
	vpaddd		V18,V2,V2
	vpaddd		V19,V3,V3
	vpaddd		V20,V4,V4
	vpaddd		V21,V5,V5
	vpaddd		V22,V6,V6
	vpaddd		V23,V7,V7
	vpaddd		V24,V8,V8
	vpaddd		V25,V9,V9
	vpaddd		V26,V10,V10
	vpaddd		V27,V11,V11
	vpaddd		V28,V12,V12
	vpaddd		V29,V13,V13
	vpaddd		V30,V14,V14
	vpaddd		V31,V15,V15

	# interleave 32-bit words in state n, n+1
	vpunpckldq	V1,V0,V16
	vpunpckhdq	V1,V0,V17
	vpunpckldq	V3,V2,V18
	vpunpckhdq	V3,V2,V19
	vpunpckldq	V5,V4,V20
	vpunpckhdq	V5,V4,V21
	vpunpckldq	V7,V6,V22
	vpunpckhdq	V7,V6,V23
	vpunpckldq	V9,V8,V24
	vpunpckhdq	V9,V8,V25
	vpunpckldq	V11,V10,V26
	vpunpckhdq	V11,V10,V27
	vpunpckldq	V13,V12,V28
	vpunpckhdq	V13,V12,V29
	vpunpckldq	V15,V14,V30
	vpunpckhdq	V15,V14,V31

	# interleave 64-bit words in state n, n+2
.if VL == 32
	vpunpckhqdq	V18,V16,V1
	vpunpcklqdq	V18,V16,V16 # leave V0 free to use as temporary register
.else
	vpunpcklqdq	V18,V16,V0
	vpunpckhqdq	V18,V16,V1
.endif
	vpunpcklqdq	V19,V17,V2
	vpunpckhqdq	V19,V17,V3
	vpunpcklqdq	V22,V20,V4
	vpunpckhqdq	V22,V20,V5
	vpunpcklqdq	V23,V21,V6
	vpunpckhqdq	V23,V21,V7
	vpunpcklqdq	V26,V24,V8
	vpunpckhqdq	V26,V24,V9
	vpunpcklqdq	V27,V25,V10
	vpunpckhqdq	V27,V25,V11
	vpunpcklqdq	V30,V28,V12
	vpunpckhqdq	V30,V28,V13
	vpunpcklqdq	V31,V29,V14
	vpunpckhqdq	V31,V29,V15

.if VL == 32
	# Do the final 128-bit interleave within the XOR sequence.

	# XOR the first 4 data blocks with the generated keystream.
	vshufi64x2	$0x0,V4,V16,V0
	vpxord		0x0000(%rdx),V0,V0
	vmovdqu64	V0,0x0000(%rsi)

	vperm2i128	$0x20,V12,V8,V0
	vpxord		0x0020(%rdx),V0,V0
	vmovdqu64	V0,0x0020(%rsi)

	vperm2i128	$0x20,V5,V1,V0
	vpxord		0x0040(%rdx),V0,V0
	vmovdqu64	V0,0x0040(%rsi)

	vperm2i128	$0x20,V13,V9,V0
	vpxord		0x0060(%rdx),V0,V0
	vmovdqu64	V0,0x0060(%rsi)

	vperm2i128	$0x20,V6,V2,V0
	vpxord		0x0080(%rdx),V0,V0
	vmovdqu64	V0,0x0080(%rsi)

	vperm2i128	$0x20,V14,V10,V0
	vpxord		0x00a0(%rdx),V0,V0
	vmovdqu64	V0,0x00a0(%rsi)

	vperm2i128	$0x20,V7,V3,V0
	vpxord		0x00c0(%rdx),V0,V0
	vmovdqu64	V0,0x00c0(%rsi)

	vperm2i128	$0x20,V15,V11,V0
	vpxord		0x00e0(%rdx),V0,V0
	vmovdqu64	V0,0x00e0(%rsi)

	# XOR up to 4 additional data blocks with the generated keystream.
	vshufi64x2	$0x3,V4,V16,V0
	cmp		$0x0120,%rcx
	jl		.Lxorpart\@
	vpxord		0x0100(%rdx),V0,V0
	vmovdqu64	V0,0x0100(%rsi)

	vperm2i128	$0x31,V12,V8,V0
	cmp		$0x0140,%rcx
	jl		.Lxorpart\@
	vpxord		0x0120(%rdx),V0,V0
	vmovdqu64	V0,0x0120(%rsi)

	vperm2i128	$0x31,V5,V1,V0
	cmp		$0x0160,%rcx
	jl		.Lxorpart\@
	vpxord		0x0140(%rdx),V0,V0
	vmovdqu64	V0,0x0140(%rsi)

	vperm2i128	$0x31,V13,V9,V0
	cmp		$0x0180,%rcx
	jl		.Lxorpart\@
	vpxord		0x0160(%rdx),V0,V0
	vmovdqu64	V0,0x0160(%rsi)

	vperm2i128	$0x31,V6,V2,V0
	cmp		$0x01a0,%rcx
	jl		.Lxorpart\@
	vpxord		0x0180(%rdx),V0,V0
	vmovdqu64	V0,0x0180(%rsi)

	vperm2i128	$0x31,V14,V10,V0
	cmp		$0x01c0,%rcx
	jl		.Lxorpart\@
	vpxord		0x01a0(%rdx),V0,V0
	vmovdqu64	V0,0x01a0(%rsi)

	vperm2i128	$0x31,V7,V3,V0
	cmp		$0x01e0,%rcx
	jl		.Lxorpart\@
	vpxord		0x01c0(%rdx),V0,V0
	vmovdqu64	V0,0x01c0(%rsi)

	vperm2i128	$0x31,V15,V11,V0
	cmp		$0x0200,%rcx
	jl		.Lxorpart\@
	vpxord		0x01e0(%rdx),V0,V0
	vmovdqu64	V0,0x01e0(%rsi)
.else
	# interleave 128-bit lanes in state n, n+4 -> zmm16..31
	vshufi64x2	$0x88,V4,V0,V16
	vshufi64x2	$0x88,V12,V8,V20
	vshufi64x2	$0xdd,V4,V0,V24
	vshufi64x2	$0xdd,V12,V8,V28
	vshufi64x2	$0x88,V5,V1,V17
	vshufi64x2	$0x88,V13,V9,V21
	vshufi64x2	$0xdd,V5,V1,V25
	vshufi64x2	$0xdd,V13,V9,V29
	vshufi64x2	$0x88,V6,V2,V18
	vshufi64x2	$0x88,V14,V10,V22
	vshufi64x2	$0xdd,V6,V2,V26
	vshufi64x2	$0xdd,V14,V10,V30
	vshufi64x2	$0x88,V7,V3,V19
	vshufi64x2	$0x88,V15,V11,V23
	vshufi64x2	$0xdd,V7,V3,V27
	vshufi64x2	$0xdd,V15,V11,V31

	# Do the final 256-bit interleave within the XOR sequence.

	# XOR the first 8 data blocks with the generated keystream.
	vshufi64x2	$0x88,V20,V16,V0
	vpxord		0x0000(%rdx),V0,V0
	vmovdqu64	V0,0x0000(%rsi)

	vshufi64x2	$0x88,V21,V17,V0
	vpxord		0x0040(%rdx),V0,V0
	vmovdqu64	V0,0x0040(%rsi)

	vshufi64x2	$0x88,V22,V18,V0
	vpxord		0x0080(%rdx),V0,V0
	vmovdqu64	V0,0x0080(%rsi)

	vshufi64x2	$0x88,V23,V19,V0
	vpxord		0x00c0(%rdx),V0,V0
	vmovdqu64	V0,0x00c0(%rsi)

	vshufi64x2	$0x88,V28,V24,V0
	vpxord		0x0100(%rdx),V0,V0
	vmovdqu64	V0,0x0100(%rsi)

	vshufi64x2	$0x88,V29,V25,V0
	vpxord		0x0140(%rdx),V0,V0
	vmovdqu64	V0,0x0140(%rsi)

	vshufi64x2	$0x88,V30,V26,V0
	vpxord		0x0180(%rdx),V0,V0
	vmovdqu64	V0,0x0180(%rsi)

	vshufi64x2	$0x88,V31,V27,V0
	vpxord		0x01c0(%rdx),V0,V0
	vmovdqu64	V0,0x01c0(%rsi)

	# XOR up to 8 additional data blocks with the generated keystream.
	vshufi64x2	$0xdd,V20,V16,V0
	cmp		$0x0240,%rcx
	jl		.Lxorpart\@
	vpxord		0x0200(%rdx),V0,V0
	vmovdqu64	V0,0x0200(%rsi)

	vshufi64x2	$0xdd,V21,V17,V0
	cmp		$0x0280,%rcx
	jl		.Lxorpart\@
	vpxord		0x0240(%rdx),V0,V0
	vmovdqu64	V0,0x0240(%rsi)

	vshufi64x2	$0xdd,V22,V18,V0
	cmp		$0x02c0,%rcx
	jl		.Lxorpart\@
	vpxord		0x0280(%rdx),V0,V0
	vmovdqu64	V0,0x0280(%rsi)

	vshufi64x2	$0xdd,V23,V19,V0
	cmp		$0x0300,%rcx
	jl		.Lxorpart\@
	vpxord		0x02c0(%rdx),V0,V0
	vmovdqu64	V0,0x02c0(%rsi)

	vshufi64x2	$0xdd,V28,V24,V0
	cmp		$0x0340,%rcx
	jl		.Lxorpart\@
	vpxord		0x0300(%rdx),V0,V0
	vmovdqu64	V0,0x0300(%rsi)

	vshufi64x2	$0xdd,V29,V25,V0
	cmp		$0x0380,%rcx
	jl		.Lxorpart\@
	vpxord		0x0340(%rdx),V0,V0
	vmovdqu64	V0,0x0340(%rsi)

	vshufi64x2	$0xdd,V30,V26,V0
	cmp		$0x03c0,%rcx
	jl		.Lxorpart\@
	vpxord		0x0380(%rdx),V0,V0
	vmovdqu64	V0,0x0380(%rsi)

	vshufi64x2	$0xdd,V31,V27,V0
	cmp		$0x0400,%rcx
	jl		.Lxorpart\@
	vpxord		0x03c0(%rdx),V0,V0
	vmovdqu64	V0,0x03c0(%rsi)
.endif

.Ldone\@:
	vzeroupper
	RET

.Lxorpart\@:
	# xor remaining bytes from partial register into output
	mov		%rcx,%r9
	and		$~(VL - 1),%r9
	and		$(VL - 1),%rcx
	jz		.Ldone\@

	mov		$-1,%rax
	bzhi		%rcx,%rax,%rax
	kmovq		%rax,%k1

	vmovdqu8	(%rdx,%r9),V1{%k1}{z}
	vpxord		V0,V1,V1
	vmovdqu8	V1,(%rsi,%r9){%k1}

	jmp		.Ldone\@
.endm

SYM_FUNC_START(chacha_4to8blocks_avx512vl)
	_chacha_words	vl=32
SYM_FUNC_END(chacha_4to8blocks_avx512vl)

SYM_FUNC_START(chacha_8to16blocks_avx512)
	_chacha_words	vl=64
SYM_FUNC_END(chacha_8to16blocks_avx512)
