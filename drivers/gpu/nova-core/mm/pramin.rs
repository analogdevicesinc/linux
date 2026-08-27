// SPDX-License-Identifier: GPL-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

//! Utilities for accessing VRAM through the PRAMIN window.

use core::ops::Range;

use kernel::{
    io::{
        io_project,
        register,
        register::OffsetLoc,
        Io,
        Mmio, //
    },
    prelude::*,
    ptr::{
        Alignable,
        Alignment, //
    },
    sizes::{
        SZ_1M,
        SZ_64K, //
    },
};

use crate::{
    driver::{
        Bar0,
        NovaRegisters, //
    },
    gpu::Chipset,
    mm::{
        hal::{
            self,
            MmHal, //
        },
        VramAddress, //
    },
    num::IntoSafeCast, //
};

/// Size of the PRAMIN window (1 MiB).
const WINDOW_SIZE: usize = SZ_1M;

/// The PRAMIN window, which is a 1 MiB window into VRAM at a fixed BAR0 offset.
#[derive(FromBytes, IntoBytes)]
struct PraminWindow([u8; WINDOW_SIZE]);

register! {
    base: NovaRegisters;

    /// Location of the window inside BAR0.
    PRAMIN: PraminWindow @ 0x700000;
}

/// Owner of the PRAMIN window state.
///
/// [`Pramin::window_at()`] repositions the window as needed and returns a typed MMIO view into
/// it, holding the manager borrowed for the lifetime of the view.
pub(super) struct Pramin<'gpu> {
    bar: Bar0<'gpu>,
    hal: &'static dyn MmHal,
    /// MMIO view of the PRAMIN window in BAR0.
    window: Mmio<'gpu, PraminWindow>,
    /// VRAM range to keep the PRAMIN window inside.
    vram_range: Range<VramAddress>,
    /// Cached window position.
    window_range: Range<VramAddress>,
}

/// Typed view of VRAM through the PRAMIN window.
///
/// Inserts an ordering point after previous writes through the window on drop. Views returned
/// by [`PraminAccess::view()`] cannot outlive this access, so the ordering point covers every
/// write made through them.
pub(super) struct PraminAccess<'a, T>
where
    T: FromBytes + IntoBytes,
{
    view: Mmio<'a, T>,
}

impl<T> PraminAccess<'_, T>
where
    T: FromBytes + IntoBytes,
{
    /// Returns the MMIO view of the accessed location.
    pub(super) fn view(&self) -> Mmio<'_, T> {
        self.view
    }
}

impl<T> Drop for PraminAccess<'_, T>
where
    T: FromBytes + IntoBytes,
{
    fn drop(&mut self) {
        // Insert an ordering point after previous writes through this window.
        self.view.cast::<u8>().read_val();
    }
}

impl<'gpu> Pramin<'gpu> {
    /// Alignment required by the PRAMIN window.
    const BASE_ALIGN: Alignment = Alignment::new::<SZ_64K>();

    /// Creates the window manager for the given VRAM region.
    pub(super) fn new(
        bar: Bar0<'gpu>,
        chipset: Chipset,
        vram_range: Range<VramAddress>,
    ) -> Result<Self> {
        let hal = hal::mm_hal(chipset);
        let window = io_project!(bar, build: PRAMIN);
        let base = vram_range.start.align_down(Self::BASE_ALIGN);
        let window_range = Self::window_range(base)?;
        hal.write_pramin_window_base(bar, base)?;

        Ok(Self {
            bar,
            hal,
            window,
            vram_range,
            window_range,
        })
    }

    /// Returns the VRAM range a window based at `base` exposes.
    fn window_range(base: VramAddress) -> Result<Range<VramAddress>> {
        let end = base
            .checked_add(WINDOW_SIZE.into_safe_cast())
            .ok_or(EINVAL)?;
        Ok(base..end)
    }

    /// Check the window covers `len` bytes at `addr`, moving it if needed.
    ///
    /// Returns the window offset at which to perform the access.
    fn window_offset(&mut self, addr: VramAddress, len: usize) -> Result<usize> {
        let end = addr.checked_add(len.into_safe_cast()).ok_or(EINVAL)?;

        let inside = |r: &Range<VramAddress>| r.contains(&addr) && end <= r.end;
        if !inside(&self.vram_range) {
            return Err(EINVAL);
        }

        // Reposition the window if the access falls outside it.
        if !inside(&self.window_range) {
            let base = addr.align_down(Self::BASE_ALIGN);
            let window_range = Self::window_range(base)?;
            if !inside(&window_range) {
                return Err(EINVAL);
            }
            self.hal.write_pramin_window_base(self.bar, base)?;
            self.window_range = window_range;
        }

        Ok((addr - self.window_range.start).into_safe_cast())
    }

    /// Return a typed MMIO view of a `T` at `vram_addr`.
    ///
    /// Returns an error if `vram_addr` is not aligned to `T`'s alignment, or if
    /// a `T` at `vram_addr` does not fit within the VRAM region.
    pub(super) fn window_at<'a, T>(
        &'a mut self,
        vram_addr: VramAddress,
    ) -> Result<PraminAccess<'a, T>>
    where
        T: FromBytes + IntoBytes,
    {
        let offset = self.window_offset(vram_addr, size_of::<T>())?;
        let view = io_project!(self.window, try: OffsetLoc::new(offset));

        Ok(PraminAccess { view })
    }
}
