// SPDX-License-Identifier: GPL-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

//! Memory management subsystems.

#![expect(dead_code)]

use core::{
    fmt::LowerHex,
    ops, //
};

use kernel::fmt;

/// Physical VRAM address in GPU video memory.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub(crate) struct VramAddress(u64);

impl VramAddress {
    /// Creates an address from a raw value.
    pub(crate) const fn from_raw(addr: u64) -> Self {
        Self(addr)
    }

    /// Returns the address as a raw value.
    pub(crate) const fn into_raw(self) -> u64 {
        self.0
    }

    /// Adds `rhs` to this address, returning [`None`] on overflow.
    pub(crate) const fn checked_add(self, rhs: u64) -> Option<Self> {
        match self.into_raw().checked_add(rhs) {
            Some(addr) => Some(Self::from_raw(addr)),
            None => None,
        }
    }
}

impl LowerHex for VramAddress {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        LowerHex::fmt(&self.into_raw(), f)
    }
}

impl ops::Add<u64> for VramAddress {
    type Output = Self;

    fn add(self, rhs: u64) -> Self::Output {
        Self::from_raw(self.into_raw() + rhs)
    }
}

impl ops::Sub for VramAddress {
    type Output = u64;

    fn sub(self, rhs: Self) -> Self::Output {
        self.into_raw() - rhs.into_raw()
    }
}
