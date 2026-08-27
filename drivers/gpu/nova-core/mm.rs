// SPDX-License-Identifier: GPL-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

//! Memory management subsystems.

#![expect(dead_code)]

use core::{
    fmt::LowerHex,
    ops, //
};

use kernel::{
    fmt,
    prelude::*,
    ptr::{
        Alignable,
        Alignment, //
    },
};

use crate::{
    driver::Bar0,
    gpu::Chipset, //
};

mod hal;
mod pramin;
mod regs;

/// GPU Memory Manager - owns all core MM components.
///
/// Provides centralized ownership of memory management resources:
/// - [`pramin::Pramin`] for direct VRAM access.
pub(crate) struct GpuMm<'gpu> {
    pramin: pramin::Pramin<'gpu>,
}

impl<'gpu> GpuMm<'gpu> {
    /// Creates the GPU memory manager.
    pub(crate) fn new(
        bar: Bar0<'gpu>,
        chipset: Chipset,
        total_fb_end: VramAddress,
    ) -> Result<Self> {
        // PRAMIN covers all physical VRAM (including GSP-reserved areas
        // above the usable region, e.g. the BAR1 page directory).
        let vram_region = VramAddress::ZERO..total_fb_end;

        Ok(Self {
            pramin: pramin::Pramin::new(bar, chipset, vram_region)?,
        })
    }

    /// Access the [`pramin::Pramin`].
    fn pramin_mut(&mut self) -> &mut pramin::Pramin<'gpu> {
        &mut self.pramin
    }
}

/// Physical VRAM address in GPU video memory.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub(crate) struct VramAddress(u64);

impl VramAddress {
    /// The zero address.
    pub(crate) const ZERO: Self = Self::from_raw(0);

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

impl Alignable for VramAddress {
    fn align_down(self, alignment: Alignment) -> Self {
        Self::from_raw(self.into_raw().align_down(alignment))
    }

    fn align_up(self, alignment: Alignment) -> Option<Self> {
        self.into_raw().align_up(alignment).map(Self::from_raw)
    }
}

impl LowerHex for VramAddress {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        LowerHex::fmt(&self.into_raw(), f)
    }
}

impl fmt::Debug for VramAddress {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_fmt(fmt!("{:#x}", self))
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
