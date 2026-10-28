// SPDX-License-Identifier: GPL-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

//! Codec for NVKV, the binary key-value format of GMCAPI.
//!
//! Essentially, the format encodes a sequence of calls to some function f(key, index, value),
//! where value is a `[u8]`, `u32`, `u64`, `[u32]`, or a `[u64]`. The key is a `u16` and the index
//! is a 12 bit integer. The interpretation of these function calls is per GMCAPI. Generally
//! speaking, the function calls will map to some struct - for example, f(GPU_NAME_STRING_KEY, 0,
//! b"some gpu") naturally maps to storing a &str with the GPU name.

#![expect(unused_imports)]

use kernel::{
    bitfield,
    num::Bounded, //
};

use crate::bounded_enum;

mod encode;
pub(crate) use encode::*;

/// The identifier of an NVKV key.
pub(crate) type KeyId = u16;

/// The index of an NVKV value.
pub(crate) type Index = Bounded<u64, 12>;

bitfield! {
    /// The op word that starts each NVKV operation.
    struct Op(u64) {
        15:0 key => KeyId;
        27:16 index => Index;
        31:28 opcode ?=> Opcode;
        63:32 value => u32;
    }
}

bounded_enum! {
    /// Describes the format of the following NVKV operation.
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    #[repr(u8)]
    enum Opcode with TryFrom<Bounded<u64, 4>> {
        /// A 32-bit value in the op word.
        Imm32 = 0,
        /// 32-bit values for consecutive keys, starting at the op word's key.
        Seq32 = 1,
        /// 64-bit values for consecutive keys, starting at the op word's key.
        Seq64 = 2,
        /// An array of bytes.
        Array8 = 3,
        /// An array of 32-bit elements.
        Array32 = 4,
        /// An array of 64-bit elements.
        Array64 = 5,
    }
}
