// SPDX-License-Identifier: GPL-2.0

//! Basic utilities for dealing with memory, values, and types.

use crate::prelude::*;

/// Transmute between two types.
///
/// Use this instead of [`core::mem::transmute`] when it is known that sizes are identical but this
/// cannot be proven by the compiler.
///
/// This is equivalent to Rust's `transmute_unchecked` intrinsics.
///
/// # Safety
///
/// All safety requirements of [`core::mem::transmute`] apply, plus that the size `Src` and `Dst`
/// must match.
///
/// # Examples
///
/// This can be used when types are known to have the same size, but only at runtime.
///
/// ```no_run
/// # use core::any::TypeId;
/// fn to_u32<T: 'static>(v: T) -> Option<u32> {
///     if TypeId::of::<T>() != TypeId::of::<u32>() {
///         return None;
///     }
///
///     // `core::mem::transmute` won't work here.
///     // SAFETY: We've checked that `T` is `u32`!
///     Some(unsafe { kernel::mem::transmute_unchecked(v) })
/// }
///
/// to_u32(1u32);
/// ```
#[inline(always)]
pub const unsafe fn transmute_unchecked<Src, Dst>(val: Src) -> Dst {
    // SAFETY: This is identical to `transmute` except that we bypassed the size check; which is
    // true per safety requirement.
    unsafe { core::mem::transmute_copy(&core::mem::ManuallyDrop::new(val)) }
}

/// Version of `transmute` that performs size check at monomorphization-time.
///
/// Use this instead of [`core::mem::transmute`] when it is known that sizes are identical but this
/// cannot be proven by the compiler during type checking and can be proven during monomorphization.
///
/// The signature is equivalent to Rust standard library's unstable `transmute_neo` and that of
/// [RFC 3844](https://github.com/rust-lang/rfcs/pull/3844).
///
/// # Safety
///
/// Same as [`core::mem::transmute`].
///
/// # Examples
///
/// This is typically used in generic code where it's known that type will have the same size, but
/// the compiler cannot prove it generically.
///
/// ```no_run
/// trait IsU32 {}
/// impl IsU32 for u32 {}
///
/// fn to_u32<T: IsU32>(v: T) -> u32 {
///     // `core::mem::transmute` won't work here.
///     // SAFETY: We know that `v` is u32!
///     unsafe { kernel::mem::transmute(v) }
/// }
///
/// to_u32(1u32);
/// ```
#[inline(always)]
pub const unsafe fn transmute<Src, Dst>(val: Src) -> Dst {
    const_assert!(size_of::<Src>() == size_of::<Dst>());

    // SAFETY: Size is checked above. Other safety requirements follow those of the function.
    unsafe { transmute_unchecked(val) }
}

/// Safely transmutes a value of one type to a value of another type of the same size.
///
/// The sizes are checked during monomorphization.
///
/// This can be considered as generic version of [`zerocopy::transmute!`] macro that defers the size
/// check and thus can be used in more cases.
///
/// # Examples
///
/// ```no_run
/// fn to_u32<T: FromBytes + IntoBytes>(v: T) -> u32 {
///     // `zerocopy::transmute!` won't work here.
///     kernel::mem::safe_transmute(v)
/// }
///
/// to_u32(1i32);
/// ```
#[inline(always)]
pub const fn safe_transmute<Src: IntoBytes, Dst: FromBytes>(val: Src) -> Dst {
    // SAFETY: `transmute` is safe with `IntoBytes` and `FromBytes` bounds.
    unsafe { transmute(val) }
}
