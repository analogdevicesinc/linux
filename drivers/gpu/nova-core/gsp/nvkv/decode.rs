// SPDX-License-Identifier: GPL-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

#![cfg_attr(not(CONFIG_KUNIT = "y"), expect(dead_code))]

use core::marker::PhantomData;

use kernel::{
    alloc::ArrayVec,
    prelude::*, //
};
use pin_init::{
    init_array_from_fn,
    stack_try_pin_init, //
};

use crate::{
    gsp::nvkv::{
        Array,
        Index,
        Key,
        KeyId,
        Op,
        Opcode, //
    },
    num::IntoSafeCast, //
};

/// Defines a schema struct together with its [`Schema`] and [`Visit`] implementations that decode
/// into `$target`.
///
/// Each member of the struct should implement [`Schema`] and [`Visit`]. For every
/// `(key, index, value)` triple decoded from the NVKV stream, the generated parent [`Visit`]
/// implementation will call each member in declaration order with that triple. If a member consumes
/// that triple, it will stop there. Otherwise it will keep going until all members are tried.
///
/// The schema struct holds the state required by the schema implementation to do the decode. It's
/// recommended to use one of the existing [`Schema`] kinds ([`Required`], [`Optional`],
/// [`Accumulated`], [`Key`], [`Array`], [`TruncatingArray`], [`Indexed`]) for each member.
///
/// Use a lifetime on the schema struct if you want to borrow data directly from the encoded stream,
/// for example, `Key<&'d [u8], KEY>`.
///
/// # Examples
///
/// ```
/// nvkv_decode! {
///     struct RequestSchema => Request {
///         id: Required<Key<u32, 0x0001>>,
///         name: Array<u8, 64, 0x0002>,
///     }
/// }
///
/// nvkv_decode! {
///     struct NameSchema<'d> => Name<'d> {
///         name: Key<&'d [u8], 0x0002>,
///     }
/// }
/// ```
macro_rules! nvkv_decode {
    // A schema which doesn't borrow from the stream. The macro uses a generic lifetime for the
    // [`Visit`] implementation but not for the struct itself. This allows omitting the unused
    // lifetime on the struct.
    (
        $(#[$attr:meta])*
        $vis:vis struct $name:ident => $target:ty { $($fields:tt)* }
    ) => {
        $crate::gsp::nvkv::nvkv_decode!(
            @impl ['data] [] $(#[$attr])* $vis struct $name => $target { $($fields)* }
        );
    };
    // A schema which borrows from the stream for lifetime `$datalt`. The macro connects the
    // [`Visit`] lifetime with the struct lifetime.
    (
        $(#[$attr:meta])*
        $vis:vis struct $name:ident<$datalt:lifetime> => $target:ty { $($fields:tt)* }
    ) => {
        $crate::gsp::nvkv::nvkv_decode!(
            @impl [$datalt] [<$datalt>] $(#[$attr])* $vis struct $name => $target { $($fields)* }
        );
    };
    (@impl [$datalt:lifetime] [$($generics:tt)*]
        $(#[$attr:meta])*
        $vis:vis struct $name:ident => $target:ty {
            $(
                $(#[$field_attr:meta])*
                $field_vis:vis $field:ident : $ty:ty
            ),* $(,)?
        }
    ) => {
        $(#[$attr])*
        $vis struct $name $($generics)* {
            $(
                $(#[$field_attr])*
                $field_vis $field: $ty,
            )*
        }

        impl $($generics)* $crate::gsp::nvkv::Schema for $name $($generics)* {
            type Target = $target;

            fn init() -> impl ::kernel::prelude::Init<Self> {
                ::pin_init::init!(Self {
                    $( $field <- <$ty as $crate::gsp::nvkv::Schema>::init(), )*
                })
            }

            fn finish(
                &mut self,
            ) -> impl ::kernel::prelude::Init<Self::Target, ::kernel::error::Error> + '_ {
                let Self { $($field,)* } = self;
                ::kernel::try_init!(Self::Target {
                    $( $field <- $crate::gsp::nvkv::Schema::finish($field), )*
                }? ::kernel::error::Error)
            }
        }

        impl<$datalt> $crate::gsp::nvkv::Visit<$datalt> for $name $($generics)* {
            fn visit(
                &mut self,
                key: $crate::gsp::nvkv::KeyId,
                index: $crate::gsp::nvkv::Index,
                value: $crate::gsp::nvkv::DecoderValue<$datalt>,
            ) -> ::kernel::error::Result<bool> {
                // TODO: This performs worst-case O(#fields) visit calls. Consider optimising this
                // if it becomes a problem.
                Ok(false
                    $( || $crate::gsp::nvkv::Visit::visit(&mut self.$field, key, index, value)? )*)
            }
        }
    };
}
pub(crate) use nvkv_decode;

impl<T: Default, const KEY_ID: KeyId, As> Schema for Key<T, KEY_ID, As> {
    type Target = T;

    fn init() -> impl Init<Self> {
        Self::default()
    }

    fn finish(&mut self) -> impl Init<Self::Target, Error> + '_ {
        const_assert!(
            size_of::<T>() <= Self::MAX_STACK_SIZE,
            "`Key` here would materialize too large a value on the stack"
        );

        Ok(core::mem::take(&mut self.0))
    }
}

impl<'data, T, const KEY_ID: KeyId, As> Visit<'data> for Key<T, KEY_ID, As>
where
    As: TryFrom<DecoderValue<'data>, Error = Error>,
    T: TryFrom<As>,
    Error: From<<T as TryFrom<As>>::Error>,
{
    fn visit(&mut self, key: KeyId, index: Index, value: DecoderValue<'data>) -> Result<bool> {
        if key != KEY_ID {
            Ok(false)
        } else if index != Index::new::<0>() {
            // Single values being set must be at index 0.
            Err(EINVAL)
        } else {
            // Overwrite and take the latest value here.
            self.0 = T::try_from(As::try_from(value)?)?;
            Ok(true)
        }
    }
}

impl<T: Copy, const N: usize, const KEY_ID: KeyId> Array<T, N, KEY_ID> {
    /// Replaces the elements with `elems`.
    ///
    /// Takes the [`Result`] for `elems` because this may be e.g. for an unrelated key.
    fn set(&mut self, key: KeyId, index: Index, elems: Result<&[T]>) -> Result<bool> {
        if key != KEY_ID {
            return Ok(false);
        }
        // Require to be at index 0
        if index != Index::new::<0>() {
            return Err(EINVAL);
        }
        // Reject oversized and take the latest value.
        self.vec.clear();
        self.vec.extend_from_slice(elems?)?;
        Ok(true)
    }
}

impl<T: Copy, const N: usize, const KEY_ID: KeyId> Schema for Array<T, N, KEY_ID> {
    type Target = ArrayVec<T, N>;

    fn init() -> impl Init<Self> {
        init!(Self {
            vec <- ArrayVec::init_empty(),
        })
    }

    fn finish(&mut self) -> impl Init<Self::Target, Error> + '_ {
        ArrayVec::init_with(move |dst| {
            dst.extend_from_slice(&self.vec)?;
            self.vec.clear();
            Ok(())
        })
    }
}

impl<'data, T: Copy + 'data, const N: usize, const KEY_ID: KeyId> Visit<'data>
    for Array<T, N, KEY_ID>
where
    &'data [T]: TryFrom<DecoderValue<'data>, Error = Error>,
{
    fn visit(&mut self, key: KeyId, index: Index, value: DecoderValue<'data>) -> Result<bool> {
        self.set(key, index, value.try_into())
    }
}

/// A schema field for an array value under the NVKV key `KEY_ID` that keeps at most `N` elements.
///
/// Unlike [`Array`], a longer value is truncated to its first `N` elements instead of failing the
/// decode.
pub(crate) struct TruncatingArray<T, const N: usize, const KEY_ID: KeyId> {
    array: Array<T, N, KEY_ID>,
}

impl<T: Copy, const N: usize, const KEY_ID: KeyId> Schema for TruncatingArray<T, N, KEY_ID> {
    type Target = ArrayVec<T, N>;

    fn init() -> impl Init<Self> {
        init!(Self {
            array <- Array::init(),
        })
    }

    fn finish(&mut self) -> impl Init<Self::Target, Error> + '_ {
        self.array.finish()
    }
}

impl<'data, T: Copy + 'data, const N: usize, const KEY_ID: KeyId> Visit<'data>
    for TruncatingArray<T, N, KEY_ID>
where
    &'data [T]: TryFrom<DecoderValue<'data>, Error = Error>,
{
    fn visit(&mut self, key: KeyId, index: Index, value: DecoderValue<'data>) -> Result<bool> {
        let elems = <&[T]>::try_from(value).map(|elems| elems.get(..N).unwrap_or(elems));
        self.array.set(key, index, elems)
    }
}

/// A schema field that marks a child [`Schema`] as required.
///
/// `finish` fails with [`EINVAL`] if the child consumed no value.
pub(crate) struct Required<S> {
    inner: S,
    parsed: bool,
}

impl<S: Schema> Schema for Required<S> {
    type Target = S::Target;

    fn init() -> impl Init<Self> {
        init!(Self {
            inner <- S::init(),
            parsed: false,
        })
    }

    fn finish(&mut self) -> impl Init<Self::Target, Error> + '_ {
        let parsed = core::mem::take(&mut self.parsed);
        self.inner
            .finish()
            .chain(move |_| if parsed { Ok(()) } else { Err(EINVAL) })
    }
}

impl<'data, S: Visit<'data>> Visit<'data> for Required<S> {
    fn visit(&mut self, key: KeyId, index: Index, value: DecoderValue<'data>) -> Result<bool> {
        let consumed = self.inner.visit(key, index, value)?;
        self.parsed |= consumed;
        Ok(consumed)
    }
}

/// A schema field which marks a child [`Schema`] as optional.
pub(crate) struct Optional<S> {
    inner: S,
    parsed: bool,
}

impl<S> Optional<S> {
    /// The maximum size in bytes we allow to materialize on the stack for the
    /// [`Target`](Schema::Target) of an [`Optional`]. This is checked at compile
    /// time.
    const MAX_STACK_SIZE: usize = 512;
}

impl<S: Schema> Schema for Optional<S>
where
    S::Target: Default + Unpin,
{
    type Target = Option<S::Target>;

    fn init() -> impl Init<Self> {
        init!(Self {
            inner <- S::init(),
            parsed: false,
        })
    }

    fn finish(&mut self) -> impl Init<Self::Target, Error> + '_ {
        const_assert!(
            size_of::<Self::Target>() <= Self::MAX_STACK_SIZE,
            "`Optional` here would materialize too large a value on the stack"
        );

        if !core::mem::take(&mut self.parsed) {
            return Ok(None);
        }
        stack_try_pin_init!(let value: S::Target =? self.inner.finish());
        Ok(Some(core::mem::take(value.get_mut())))
    }
}

impl<'data, S: Visit<'data>> Visit<'data> for Optional<S> {
    fn visit(&mut self, key: KeyId, index: Index, value: DecoderValue<'data>) -> Result<bool> {
        let consumed = self.inner.visit(key, index, value)?;
        self.parsed |= consumed;
        Ok(consumed)
    }
}

/// Expects objects specified sequentially with index starting from zero.
///
/// This stores two [`Schema`]s which are initially empty. A [`Schema`] is expected to be complete
/// when we get the next index.
pub(crate) struct Accumulated<S: Schema> {
    // Tracks the current index (starting from 0). When this advances, the schema `current` must be
    // finished.
    current_index: Index,

    // The current `Schema` being constructed now.
    current: S,

    // Tracks empty schemas (one that has never consumed a key via `visit`), so we don't call
    // `finish` on them.
    current_started: bool,

    // The next `Schema` to be constructed. This is required because we must consume a key intended
    // for this Schema from the next index to know we can try calling `finish` on `current`.
    next: S,

    // The set of results generated by the completed schemas so far.
    accumulated: KVVec<S::Target>,
}

impl<S: Schema> Schema for Accumulated<S> {
    type Target = KVVec<S::Target>;

    fn init() -> impl Init<Self> {
        init!(Self {
            current_index: Index::new::<0>(),
            current <- S::init(),
            current_started: false,
            next <- S::init(),
            accumulated: KVVec::new(),
        })
    }

    fn finish(&mut self) -> impl Init<Self::Target, Error> + '_ {
        if self.current_started {
            self.accumulated
                .try_push_init(self.current.finish(), GFP_KERNEL)?;
            self.current_started = false;
        }
        self.current_index = Index::new::<0>();
        Ok(core::mem::take(&mut self.accumulated))
    }
}

impl<'data, S: Schema + Visit<'data>> Visit<'data> for Accumulated<S> {
    fn visit(&mut self, key: KeyId, index: Index, value: DecoderValue<'data>) -> Result<bool> {
        if index != self.current_index {
            if !self.next.visit(key, Index::new::<0>(), value)? {
                // Unrelated key to us.
                return Ok(false);
            }

            // Require that objects at index k have all their keys sent before the k + 1 th object
            // can be completed. Require that objects are sent contiguously in order from index 0.
            if !self.current_started || index != self.current_index + 1 {
                return Err(EINVAL);
            }

            // The current value must be finished. Push it and swap in `next`.
            self.accumulated
                .try_push_init(self.current.finish(), GFP_KERNEL)?;
            core::mem::swap(&mut self.current, &mut self.next);
            self.current_started = true;
            self.current_index = index;
            Ok(true)
        } else {
            let consumed = self.current.visit(key, Index::new::<0>(), value)?;
            self.current_started |= consumed;
            Ok(consumed)
        }
    }
}

/// A schema field that scatters indexed values into an array of `N` slots.
///
/// Values are decoded from the encoded stream using the type `As`, but then converted into `T` with
/// [`From`], like the `As` parameter of [`Key`].
pub(crate) struct Indexed<T, const N: usize, const KEY_ID: KeyId, As = T> {
    slots: [T; N],
    _as: PhantomData<As>,
}

/// Copies `elems`, converted to `T`, into `slots` at `start`.
///
/// Fails with [`EINVAL`] if the window does not fit in `slots`.
fn scatter_window<T: From<As>, As: Copy>(slots: &mut [T], start: usize, elems: &[As]) -> Result {
    let end = start.checked_add(elems.len()).ok_or(EINVAL)?;
    // Reject indices outside of the declared array size.
    let dst = slots.get_mut(start..end).ok_or(EINVAL)?;
    for (d, &e) in dst.iter_mut().zip(elems) {
        *d = T::from(e);
    }
    Ok(())
}

impl<T: Default, const N: usize, const KEY_ID: KeyId, As> Schema for Indexed<T, N, KEY_ID, As> {
    type Target = [T; N];

    fn init() -> impl Init<Self> {
        init!(Self {
            slots <- init_array_from_fn(|_| T::default()),
            _as: PhantomData,
        })
    }

    fn finish(&mut self) -> impl Init<Self::Target, Error> + '_ {
        init_array_from_fn(|i| Ok::<_, Error>(core::mem::take(&mut self.slots[i])))
    }
}

impl<'data, T, const N: usize, const KEY_ID: KeyId, As> Visit<'data> for Indexed<T, N, KEY_ID, As>
where
    T: From<As>,
    As: Copy + TryFrom<DecoderValue<'data>, Error = Error> + 'data,
    &'data [As]: TryFrom<DecoderValue<'data>, Error = Error>,
{
    fn visit(&mut self, key: KeyId, index: Index, value: DecoderValue<'data>) -> Result<bool> {
        if key != KEY_ID {
            return Ok(false);
        }
        let start = index.cast::<usize>().get();
        // Accept both scalar vs scattered array setting for flexibility.
        match <&[As]>::try_from(value) {
            Ok(elems) => scatter_window(&mut self.slots, start, elems)?,
            Err(_) => scatter_window(&mut self.slots, start, &[As::try_from(value)?])?,
        }
        Ok(true)
    }
}

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

    // Tests that decoding via the `nvkv_decode!` macro works correctly.
    #[test]
    fn decode_typed_struct() -> Result {
        const SCALAR32_KEY: KeyId = 0x1234;
        const SCALAR64_KEY: KeyId = 0x1235;
        const ARRAY8_KEY: KeyId = 0x1236;
        const ARRAY32_KEY: KeyId = 0x1237;
        const ARRAY64_KEY: KeyId = 0x1238;
        const OPT_PRESENT_KEY: KeyId = 0x1239;
        const OPT_ABSENT_KEY: KeyId = 0x123a;
        const OPT_ARRAY_KEY: KeyId = 0x123b;
        const TRUNCATED_KEY: KeyId = 0x123c;
        const CONVERTED_KEY: KeyId = 0x123d;
        const X_KEY: KeyId = 0x0100;
        const Y_KEY: KeyId = 0x0101;
        const SLOT_KEY: KeyId = 0x0200;

        const SCALAR32_VALUE: u32 = 0x89ab_cdef;
        const SCALAR64_VALUE: u64 = 0x0123_4567_89ab_cdef;
        const ARRAY8_VALUE: &[u8] = &[0x12, 0x34, 0x56];
        const ARRAY32_VALUE: &[u32] = &[0x0123_4567, 0x89ab_cdef];
        const ARRAY64_VALUE: &[u64] = &[0x0123_4567_89ab_cdef, 0xfedc_ba98_7654_3210];
        const OPT_PRESENT_VALUE: u32 = 0x55;
        const OPT_ARRAY_VALUE: &[u8] = &[0x77, 0x88];
        const TRUNCATED_VALUE: &[u8] = &[0x9a, 0xbc, 0xde];
        const CONVERTED_VALUE: u32 = 0x66;

        // A value that is decoded from the `u32` it converts from.
        #[derive(Default)]
        struct Converted(u32);

        impl From<u32> for Converted {
            fn from(value: u32) -> Self {
                Self(value)
            }
        }

        nvkv_decode! {
            struct PairSchema => Pair {
                x: Required<Key<u32, X_KEY>>,
                y: Required<Key<u32, Y_KEY>>,
            }
        }

        struct Pair {
            x: u32,
            y: u32,
        }

        nvkv_decode! {
            struct TestSchema => TestDecodable {
                scalar32: Required<Key<u32, SCALAR32_KEY>>,
                scalar64: Required<Key<u64, SCALAR64_KEY>>,
                array8: Array<u8, 64, ARRAY8_KEY>,
                array32: Array<u32, 64, ARRAY32_KEY>,
                array64: Array<u64, 32, ARRAY64_KEY>,
                opt_present: Optional<Key<u32, OPT_PRESENT_KEY>>,
                opt_absent: Optional<Key<u32, OPT_ABSENT_KEY>>,
                opt_array: Optional<Array<u8, 4, OPT_ARRAY_KEY>>,
                truncated: TruncatingArray<u8, 2, TRUNCATED_KEY>,
                converted: Key<Converted, CONVERTED_KEY, u32>,
                pairs: Accumulated<PairSchema>,
                slots: Indexed<u32, 4, SLOT_KEY>,
            }
        }

        struct TestDecodable {
            scalar32: u32,
            scalar64: u64,
            array8: ArrayVec<u8, 64>,
            array32: ArrayVec<u32, 64>,
            array64: ArrayVec<u64, 32>,
            opt_present: Option<u32>,
            opt_absent: Option<u32>,
            opt_array: Option<ArrayVec<u8, 4>>,
            truncated: ArrayVec<u8, 2>,
            converted: Converted,
            pairs: KVVec<Pair>,
            slots: [u32; 4],
        }

        let index0 = Index::new::<0>();
        let index1 = Index::new::<1>();
        let index2 = Index::new::<2>();
        let mut encoder = Encoder::new();
        encoder.encode_u32(SCALAR32_KEY, index0, SCALAR32_VALUE)?;
        encoder.encode_u64(SCALAR64_KEY, index0, SCALAR64_VALUE)?;
        encoder.encode_array8(ARRAY8_KEY, index0, ARRAY8_VALUE)?;
        encoder.encode_array32(ARRAY32_KEY, index0, ARRAY32_VALUE)?;
        encoder.encode_array64(ARRAY64_KEY, index0, ARRAY64_VALUE)?;
        encoder.encode_u32(OPT_PRESENT_KEY, index0, OPT_PRESENT_VALUE)?;
        encoder.encode_array8(OPT_ARRAY_KEY, index0, OPT_ARRAY_VALUE)?;
        encoder.encode_array8(TRUNCATED_KEY, index0, TRUNCATED_VALUE)?;
        encoder.encode_u32(CONVERTED_KEY, index0, CONVERTED_VALUE)?;
        encoder.encode_u32(X_KEY, index0, 1)?;
        encoder.encode_u32(Y_KEY, index0, 2)?;
        encoder.encode_u32(SLOT_KEY, index1, 20)?;
        encoder.encode_u32(X_KEY, index1, 3)?;
        encoder.encode_u32(Y_KEY, index1, 4)?;
        encoder.encode_u32(SLOT_KEY, index0, 10)?;
        encoder.encode_array32(SLOT_KEY, index2, &[30, 40])?;
        let serialized = encoder.finish();

        let decoder = Decoder::new(&serialized, UnknownKeyPolicy::Error);
        let mut schema = KBox::init(TestSchema::init(), GFP_KERNEL)?;
        let decoded = KBox::try_init(decoder.decode(&mut *schema)?, GFP_KERNEL)?;

        assert_eq!(decoded.scalar32, SCALAR32_VALUE);
        assert_eq!(decoded.scalar64, SCALAR64_VALUE);
        assert_eq!(*decoded.array8, *ARRAY8_VALUE);
        assert_eq!(*decoded.array32, *ARRAY32_VALUE);
        assert_eq!(*decoded.array64, *ARRAY64_VALUE);
        assert_eq!(decoded.opt_present, Some(OPT_PRESENT_VALUE));
        assert_eq!(decoded.opt_absent, None);
        assert_eq!(decoded.opt_array.as_deref(), Some(OPT_ARRAY_VALUE));
        assert_eq!(*decoded.truncated, TRUNCATED_VALUE[..2]);
        assert_eq!(decoded.converted.0, CONVERTED_VALUE);
        assert_eq!(decoded.pairs.len(), 2);
        assert_eq!(decoded.pairs[0].x, 1);
        assert_eq!(decoded.pairs[0].y, 2);
        assert_eq!(decoded.pairs[1].x, 3);
        assert_eq!(decoded.pairs[1].y, 4);
        assert_eq!(decoded.slots, [10, 20, 30, 40]);

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

    // Tests the error paths of the schema kinds.
    #[test]
    fn decode_typed_struct_errors() -> Result {
        const VALUE_KEY: KeyId = 0x2200;
        const SLOT_KEY: KeyId = 0x2201;
        const BLOB_KEY: KeyId = 0x2202;
        const X_KEY: KeyId = 0x2203;
        const Y_KEY: KeyId = 0x2204;

        let index0 = Index::new::<0>();
        let index1 = Index::new::<1>();
        let index2 = Index::new::<2>();

        nvkv_decode! {
            struct ValueSchema => Value {
                value: Key<u32, VALUE_KEY>,
            }
        }

        struct Value {
            value: u32,
        }

        // A single value at a non-zero index.
        let mut encoder = Encoder::new();
        encoder.encode_u32(VALUE_KEY, index1, 1)?;
        let serialized = encoder.finish();
        let decoder = Decoder::new(&serialized, UnknownKeyPolicy::Error);
        let mut schema = KBox::init(ValueSchema::init(), GFP_KERNEL)?;
        assert!(decoder.decode(&mut *schema).is_err());

        // A 64-bit value for a 32-bit key.
        let mut encoder = Encoder::new();
        encoder.encode_u64(VALUE_KEY, index0, 1)?;
        let serialized = encoder.finish();
        let decoder = Decoder::new(&serialized, UnknownKeyPolicy::Error);
        let mut schema = KBox::init(ValueSchema::init(), GFP_KERNEL)?;
        assert!(decoder.decode(&mut *schema).is_err());

        nvkv_decode! {
            struct SlotsSchema => Slots {
                slots: Indexed<u32, 2, SLOT_KEY>,
            }
        }

        struct Slots {
            slots: [u32; 2],
        }

        // An index past the declared slots.
        let mut encoder = Encoder::new();
        encoder.encode_u32(SLOT_KEY, index2, 1)?;
        let serialized = encoder.finish();
        let decoder = Decoder::new(&serialized, UnknownKeyPolicy::Error);
        let mut schema = KBox::init(SlotsSchema::init(), GFP_KERNEL)?;
        assert!(decoder.decode(&mut *schema).is_err());

        nvkv_decode! {
            struct BlobSchema => Blob {
                blob: Array<u8, 4, BLOB_KEY>,
            }
        }

        struct Blob {
            blob: ArrayVec<u8, 4>,
        }

        // An array longer than the declared capacity.
        let mut encoder = Encoder::new();
        encoder.encode_array8(BLOB_KEY, index0, &[0; 5])?;
        let serialized = encoder.finish();
        let decoder = Decoder::new(&serialized, UnknownKeyPolicy::Error);
        let mut schema = KBox::init(BlobSchema::init(), GFP_KERNEL)?;
        assert!(decoder.decode(&mut *schema).is_err());

        nvkv_decode! {
            struct PairSchema => Pair {
                x: Required<Key<u32, X_KEY>>,
                y: Required<Key<u32, Y_KEY>>,
            }
        }

        struct Pair {
            x: u32,
            y: u32,
        }

        // Required keys that are missing are an error.
        let mut encoder = Encoder::new();
        encoder.encode_u32(X_KEY, index0, 1)?;
        let serialized = encoder.finish();
        let decoder = Decoder::new(&serialized, UnknownKeyPolicy::Error);
        let mut schema = KBox::init(PairSchema::init(), GFP_KERNEL)?;
        let init = decoder.decode(&mut *schema)?;
        assert!(KBox::try_init(init, GFP_KERNEL).is_err());

        nvkv_decode! {
            struct PairsSchema => Pairs {
                pairs: Accumulated<PairSchema>,
            }
        }

        struct Pairs {
            pairs: KVVec<Pair>,
        }

        // Accumulated objects must start at index 0.
        let mut encoder = Encoder::new();
        encoder.encode_u32(X_KEY, index1, 1)?;
        let serialized = encoder.finish();
        let decoder = Decoder::new(&serialized, UnknownKeyPolicy::Error);
        let mut schema = KBox::init(PairsSchema::init(), GFP_KERNEL)?;
        assert!(decoder.decode(&mut *schema).is_err());

        // Accumulated objects must be complete before the next one starts.
        let mut encoder = Encoder::new();
        encoder.encode_u32(X_KEY, index0, 1)?;
        encoder.encode_u32(X_KEY, index1, 2)?;
        let serialized = encoder.finish();
        let decoder = Decoder::new(&serialized, UnknownKeyPolicy::Error);
        let mut schema = KBox::init(PairsSchema::init(), GFP_KERNEL)?;
        assert!(decoder.decode(&mut *schema).is_err());

        Ok(())
    }
}
