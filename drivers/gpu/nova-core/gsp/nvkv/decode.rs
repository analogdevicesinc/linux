// SPDX-License-Identifier: GPL-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

#![cfg_attr(not(CONFIG_KUNIT = "y"), expect(dead_code))]

use kernel::prelude::*;

use crate::{
    gsp::nvkv::{
        Index,
        KeyId,
        Op,
        Opcode, //
    },
    num::IntoSafeCast, //
};

/// A decoded NVKV value.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum DecoderValue<'a> {
    Scalar32(u32),
    Scalar64(u64),
    Array8(&'a [u8]),
    Array32(&'a [u32]),
    Array64(&'a [u64]),
}

/// Implements [`TryFrom`] from the given [`DecoderValue`] variant to the given type.
///
/// [`TryFrom`] is used by the [`Schema`] implementations in this file to convert from the
/// [`DecoderValue`]s into the types to store. Provide the implementations for basic types here.
macro_rules! impl_try_from_decoder_value {
    ($ty:ty, $variant:ident) => {
        impl<'a> TryFrom<DecoderValue<'a>> for $ty {
            type Error = Error;

            fn try_from(value: DecoderValue<'a>) -> Result<Self> {
                if let DecoderValue::$variant(v) = value {
                    Ok(v)
                } else {
                    Err(EINVAL)
                }
            }
        }
    };
}

impl_try_from_decoder_value!(u32, Scalar32);
impl_try_from_decoder_value!(u64, Scalar64);
impl_try_from_decoder_value!(&'a [u8], Array8);
impl_try_from_decoder_value!(&'a [u32], Array32);
impl_try_from_decoder_value!(&'a [u64], Array64);

/// The state of one NVKV decode operation which produces a target value [`Target`](Schema::Target).
///
/// The decoded values are passed to the state through [`Visit`], which is a separate trait so
/// that only [`Visit`] has the lifetime of the stream as a parameter. Structs whose [`Schema`]
/// fields don't borrow from the stream then don't need a lifetime parameter.
pub(crate) trait Schema {
    type Target;

    /// Returns an initializer that creates an empty schema in place.
    ///
    /// Use [`KBox::init`] for the heap or [`stack_pin_init!`](pin_init::stack_pin_init) for the
    /// stack (if sure that the value is small enough to fit).
    fn init() -> impl Init<Self>;

    /// Returns an initializer that makes the decoded [`Target`](Schema::Target).
    ///
    /// After the returned initializer runs successfully, the schema must be empty again. Taking
    /// `self` instead of `&mut self` would avoid this contract, but it forces a copy of the schema
    /// onto the stack.
    fn finish(&mut self) -> impl Init<Self::Target, Error> + '_;
}

/// Trait for types that visit the decoded key, index, and value tuple of an NVKV stream.
///
/// A schema that doesn't need to borrow data from the stream can implement this for all `'data`
/// lifetimes, avoiding having to carry the lifetime parameter. A schema that borrows from the
/// stream directly should implement it for its own lifetime only.
pub(crate) trait Visit<'data> {
    /// Visits one decoded tuple. Returns `Ok(true)` if the schema consumed it.
    fn visit(&mut self, key: KeyId, index: Index, value: DecoderValue<'data>) -> Result<bool>;
}

/// A read position in an NVKV stream.
struct Cursor<'a> {
    data: &'a [u64],
}

impl<'a> Cursor<'a> {
    /// Creates a cursor at the start of `data`.
    fn new(data: &'a [u64]) -> Self {
        Self { data }
    }

    /// Returns `true` if no `u64` values remain.
    fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Takes the next `u64`.
    fn take_u64(&mut self) -> Result<u64> {
        // PANIC: `take_u64s(1)` returns exactly one element on success.
        Ok(self.take_u64s(1)?[0])
    }

    /// Takes `count` bytes. If `count` is not a multiple of 8 (`u64` size), bytes are discarded up
    /// to the next multiple.
    fn take_u8s(&mut self, count: usize) -> Result<&'a [u8]> {
        let values = self.take_u64s(count.div_ceil(size_of::<u64>()))?;
        values.as_bytes().get(..count).ok_or(EINVAL)
    }

    /// Takes `count` 32-bit values. If `count` is not a multiple of 2 (`u32` values per `u64`),
    /// bytes are discarded up to the next multiple.
    fn take_u32s(&mut self, count: usize) -> Result<&'a [u32]> {
        let values = self.take_u64s(count.div_ceil(size_of::<u64>() / size_of::<u32>()))?;
        <[u32]>::ref_from_prefix_with_elems(values.as_bytes(), count)
            .map(|(elems, _)| elems)
            .map_err(|_| EINVAL)
    }

    /// Takes `count` `u64` values, or fails with [`EINVAL`] if fewer remain.
    fn take_u64s(&mut self, count: usize) -> Result<&'a [u64]> {
        let (prefix, suffix) = self.data.split_at_checked(count).ok_or(EINVAL)?;
        self.data = suffix;
        Ok(prefix)
    }
}

/// A decoder for an NVKV stream.
pub(crate) struct Decoder<'a> {
    data: &'a [u64],
    policy: UnknownKeyPolicy,
}

impl<'a> Decoder<'a> {
    /// Creates a decoder for `data` that handles unknown keys per `policy`.
    pub(crate) fn new(data: &'a [u64], policy: UnknownKeyPolicy) -> Self {
        Self { data, policy }
    }

    /// Passes a decoded value to `schema` and applies the [`UnknownKeyPolicy`] if not consumed.
    fn visit<S: Visit<'a>>(
        &self,
        schema: &mut S,
        key: KeyId,
        index: Index,
        value: DecoderValue<'a>,
    ) -> Result {
        let consumed = schema.visit(key, index, value)?;
        if !consumed && self.policy == UnknownKeyPolicy::Error {
            Err(EINVAL)
        } else {
            Ok(())
        }
    }

    /// Computes the key to use with a `SEQ` operation, or errors if it doesn't
    /// make a valid key (overflow).
    fn seq_key(base: KeyId, offset: usize) -> Result<KeyId> {
        base.checked_add(KeyId::try_from(offset)?).ok_or(EINVAL)
    }

    /// Decodes every pair into `schema` and returns the result of [`Schema::finish`].
    pub(crate) fn decode<'s, S: Schema + Visit<'a>>(
        &self,
        schema: &'s mut S,
    ) -> Result<impl Init<S::Target, Error> + 's> {
        let mut cursor = Cursor::new(self.data);
        while !cursor.is_empty() {
            let op: Op = cursor.take_u64()?.into();

            let key = op.key();
            let index = op.index();
            let op_value = op.value();
            match op.opcode()? {
                Opcode::Imm32 => {
                    self.visit(schema, key, index, DecoderValue::Scalar32(op_value))?;
                }
                Opcode::Seq32 => {
                    let values = cursor.take_u32s(op_value.into_safe_cast())?;
                    for (i, &value) in values.iter().enumerate() {
                        let key = Self::seq_key(key, i)?;
                        self.visit(schema, key, index, DecoderValue::Scalar32(value))?;
                    }
                }
                Opcode::Seq64 => {
                    let values = cursor.take_u64s(op_value.into_safe_cast())?;
                    for (i, &value) in values.iter().enumerate() {
                        let key = Self::seq_key(key, i)?;
                        self.visit(schema, key, index, DecoderValue::Scalar64(value))?;
                    }
                }
                Opcode::Array8 => {
                    let value = cursor.take_u8s(op_value.into_safe_cast())?;
                    self.visit(schema, key, index, DecoderValue::Array8(value))?;
                }
                Opcode::Array32 => {
                    let value = cursor.take_u32s(op_value.into_safe_cast())?;
                    self.visit(schema, key, index, DecoderValue::Array32(value))?;
                }
                Opcode::Array64 => {
                    let value = cursor.take_u64s(op_value.into_safe_cast())?;
                    self.visit(schema, key, index, DecoderValue::Array64(value))?;
                }
            };
        }
        Ok(schema.finish())
    }
}

/// This is defined per call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum UnknownKeyPolicy {
    /// Decoding skips values that the schema does not consume.
    Ignore,
    /// Decoding fails with [`EINVAL`] on the first value that the schema does not consume.
    Error,
}

#[kunit_tests(nova_core_nvkv_decode)]
mod tests {
    use super::*;

    use crate::gsp::nvkv::Encoder;

    /// The output type of the hand written [`Schema`].
    ///
    /// In this case, we can have it also implement [`Schema`] on itself rather than having a
    /// separate carrier type, since the [`Schema`] implementation is completely stateless.
    #[derive(Default)]
    struct RawSchema {
        scalar32: u32,
        scalar64: u64,
    }

    impl RawSchema {
        const SCALAR32_KEY: KeyId = 0x1001;
        const SCALAR64_KEY: KeyId = 0x1002;
    }

    impl Schema for RawSchema {
        type Target = Self;

        fn init() -> impl Init<Self> {
            Self::default()
        }

        fn finish(&mut self) -> impl Init<Self::Target, Error> + '_ {
            Ok(core::mem::take(self))
        }
    }

    impl<'d> Visit<'d> for RawSchema {
        fn visit(&mut self, key: KeyId, index: Index, value: DecoderValue<'d>) -> Result<bool> {
            if index != Index::new::<0>() {
                return Err(EINVAL);
            }
            match key {
                Self::SCALAR32_KEY => self.scalar32 = value.try_into()?,
                Self::SCALAR64_KEY => self.scalar64 = value.try_into()?,
                _ => return Ok(false),
            }
            Ok(true)
        }
    }

    // Tests that basic decoding into a manually implemented `Schema` works correctly.
    #[test]
    fn decode_raw_schema() -> Result {
        // Decodes an IMM32 pair and a SEQ64 pair (the encoder emits a u64 as a single-element
        // SEQ64) with a hand written `Schema`. Keys and value constants chosen to distinguish e.g.
        // saving the wrong value to the wrong location.
        const SCALAR32_VALUE: u32 = 0x1111_2222;
        const SCALAR64_VALUE: u64 = 0x3333_4444_5555_6666;

        let mut encoder = Encoder::new();
        encoder.encode_u32(RawSchema::SCALAR32_KEY, Index::new::<0>(), SCALAR32_VALUE)?;
        encoder.encode_u64(RawSchema::SCALAR64_KEY, Index::new::<0>(), SCALAR64_VALUE)?;
        let serialized = encoder.finish();

        let decoder = Decoder::new(&serialized, UnknownKeyPolicy::Error);
        let mut schema = KBox::init(RawSchema::init(), GFP_KERNEL)?;
        let decoded = KBox::try_init(decoder.decode(&mut *schema)?, GFP_KERNEL)?;

        assert_eq!(decoded.scalar32, SCALAR32_VALUE);
        assert_eq!(decoded.scalar64, SCALAR64_VALUE);

        Ok(())
    }

    // Tests that an unknown key fails the decode under `UnknownKeyPolicy::Error` and is skipped
    // under `UnknownKeyPolicy::Ignore`.
    #[test]
    fn decode_unknown_key() -> Result {
        const UNKNOWN_KEY: KeyId = 0x2001;

        let mut encoder = Encoder::new();
        encoder.encode_u32(UNKNOWN_KEY, Index::new::<0>(), 1)?;
        let serialized = encoder.finish();

        let decoder = Decoder::new(&serialized, UnknownKeyPolicy::Error);
        let mut schema = KBox::init(RawSchema::init(), GFP_KERNEL)?;
        assert!(decoder.decode(&mut *schema).is_err());

        let decoder = Decoder::new(&serialized, UnknownKeyPolicy::Ignore);
        let mut schema = KBox::init(RawSchema::init(), GFP_KERNEL)?;
        assert!(decoder.decode(&mut *schema).is_ok());

        Ok(())
    }

    /// Records each visit as `(key, index, value)`, for tests on hand-built streams.
    #[derive(Default)]
    struct Recorder<'d> {
        visits: KVVec<(KeyId, u64, DecoderValue<'d>)>,
    }

    impl<'d> Schema for Recorder<'d> {
        type Target = KVVec<(KeyId, u64, DecoderValue<'d>)>;

        fn init() -> impl Init<Self> {
            Self::default()
        }

        fn finish(&mut self) -> impl Init<Self::Target, Error> + '_ {
            Ok(core::mem::take(&mut self.visits))
        }
    }

    impl<'d> Visit<'d> for Recorder<'d> {
        fn visit(&mut self, key: KeyId, index: Index, value: DecoderValue<'d>) -> Result<bool> {
            self.visits.push((key, index.get(), value), GFP_KERNEL)?;
            Ok(true)
        }
    }

    // Tests the decoder on hand-built `u64` values that the encoder does not produce: SEQ32,
    // multi-value SEQ64, zero counts, a non-zero index and padded arrays.
    #[test]
    fn decode_raw_u64s() -> Result {
        const SEQ32_KEY: KeyId = 0x2000;
        const SEQ64_KEY: KeyId = 0x2010;
        const EMPTY_SEQ64_KEY: KeyId = 0x2020;
        const EMPTY_SEQ32_KEY: KeyId = 0x2021;
        const EMPTY_ARRAY8_KEY: KeyId = 0x2030;
        const EMPTY_ARRAY32_KEY: KeyId = 0x2031;
        const EMPTY_ARRAY64_KEY: KeyId = 0x2032;
        const ARRAY8_KEY: KeyId = 0x2040;
        const ARRAY32_KEY: KeyId = 0x2041;

        let data = [
            // SEQ32 with three values for three consecutive keys, packed two per `u64`.
            Op::zeroed()
                .with_key(SEQ32_KEY)
                .with_opcode(Opcode::Seq32)
                .with_value(3)
                .into_raw(),
            0x0000_0002_0000_0001,
            0x0000_0000_0000_0003,
            // SEQ64 with two values at a non-zero index.
            Op::zeroed()
                .with_key(SEQ64_KEY)
                .with_index(Index::new::<3>())
                .with_opcode(Opcode::Seq64)
                .with_value(2)
                .into_raw(),
            0x1111_1111_1111_1111,
            0x2222_2222_2222_2222,
            // A zero-count sequence has no keys and is skipped, as in NVIDIA's decoder.
            Op::zeroed()
                .with_key(EMPTY_SEQ64_KEY)
                .with_opcode(Opcode::Seq64)
                .with_value(0)
                .into_raw(),
            Op::zeroed()
                .with_key(EMPTY_SEQ32_KEY)
                .with_opcode(Opcode::Seq32)
                .with_value(0)
                .into_raw(),
            // A zero-length array is visited with an empty slice.
            Op::zeroed()
                .with_key(EMPTY_ARRAY8_KEY)
                .with_opcode(Opcode::Array8)
                .with_value(0)
                .into_raw(),
            Op::zeroed()
                .with_key(EMPTY_ARRAY32_KEY)
                .with_opcode(Opcode::Array32)
                .with_value(0)
                .into_raw(),
            Op::zeroed()
                .with_key(EMPTY_ARRAY64_KEY)
                .with_opcode(Opcode::Array64)
                .with_value(0)
                .into_raw(),
            // Three bytes padded to one `u64`, then three 32-bit values padded to two `u64` values.
            Op::zeroed()
                .with_key(ARRAY8_KEY)
                .with_opcode(Opcode::Array8)
                .with_value(3)
                .into_raw(),
            0x0000_0000_00cc_bbaa,
            Op::zeroed()
                .with_key(ARRAY32_KEY)
                .with_opcode(Opcode::Array32)
                .with_value(3)
                .into_raw(),
            0x0000_0002_0000_0001,
            0x0000_0000_0000_0003,
        ];

        let decoder = Decoder::new(&data, UnknownKeyPolicy::Error);
        let mut schema = KBox::init(Recorder::init(), GFP_KERNEL)?;
        let visits = KBox::try_init(decoder.decode(&mut *schema)?, GFP_KERNEL)?;

        assert_eq!(
            visits.as_slice(),
            &[
                (SEQ32_KEY, 0, DecoderValue::Scalar32(1)),
                (SEQ32_KEY + 1, 0, DecoderValue::Scalar32(2)),
                (SEQ32_KEY + 2, 0, DecoderValue::Scalar32(3)),
                (SEQ64_KEY, 3, DecoderValue::Scalar64(0x1111_1111_1111_1111)),
                (
                    SEQ64_KEY + 1,
                    3,
                    DecoderValue::Scalar64(0x2222_2222_2222_2222)
                ),
                (EMPTY_ARRAY8_KEY, 0, DecoderValue::Array8(&[])),
                (EMPTY_ARRAY32_KEY, 0, DecoderValue::Array32(&[])),
                (EMPTY_ARRAY64_KEY, 0, DecoderValue::Array64(&[])),
                (ARRAY8_KEY, 0, DecoderValue::Array8(&[0xaa, 0xbb, 0xcc])),
                (ARRAY32_KEY, 0, DecoderValue::Array32(&[1, 2, 3])),
            ]
        );

        Ok(())
    }

    // Tests that decoding a malformed stream fails instead of reading past the payload.
    #[test]
    fn decode_raw_words_malformed() -> Result {
        const KEY: KeyId = 0x2100;

        // An `Op` with the reserved opcode 6.
        let bad_opcode = Op::zeroed().with_key(KEY).into_raw() | (6u64 << Op::OPCODE_SHIFT);
        let streams: [&[u64]; 6] = [
            // 100 bytes need 13 `u64` values, only one follows.
            &[
                Op::zeroed()
                    .with_key(KEY)
                    .with_opcode(Opcode::Array8)
                    .with_value(100)
                    .into_raw(),
                0,
            ],
            // Two 64-bit values, only one follows.
            &[
                Op::zeroed()
                    .with_key(KEY)
                    .with_opcode(Opcode::Seq64)
                    .with_value(2)
                    .into_raw(),
                0,
            ],
            // Three 32-bit values need two `u64` values, only one follows.
            &[
                Op::zeroed()
                    .with_key(KEY)
                    .with_opcode(Opcode::Array32)
                    .with_value(3)
                    .into_raw(),
                0,
            ],
            // A value count with no payload.
            &[Op::zeroed()
                .with_key(KEY)
                .with_opcode(Opcode::Seq32)
                .with_value(1)
                .into_raw()],
            &[bad_opcode],
            // Consecutive keys that overflow `KeyId`.
            &[
                Op::zeroed()
                    .with_key(KeyId::MAX)
                    .with_opcode(Opcode::Seq32)
                    .with_value(2)
                    .into_raw(),
                0,
            ],
        ];

        for stream in streams {
            let decoder = Decoder::new(stream, UnknownKeyPolicy::Ignore);
            let mut schema = KBox::init(Recorder::init(), GFP_KERNEL)?;
            assert!(decoder.decode(&mut *schema).is_err());
        }

        Ok(())
    }
}
