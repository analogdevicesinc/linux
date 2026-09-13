// SPDX-License-Identifier: GPL-2.0
// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

//! The radix3 page table, through which firmware running on the GPU reads a buffer in system
//! memory. LibOS, the operating system of the GSP, defines the format: three levels of
//! [`GSP_PAGE_SIZE`] pages, each entry the little-endian DMA address of one page.
//!
//! ```text
//! Level 0:  one page, one entry  ->  the first level 1 page
//! Level 1:  pages of entries     ->  each entry a level 2 page
//! Level 2:  pages of entries     ->  each entry a page of the buffer
//! ```

use kernel::{
    device,
    dma::{
        Coherent,
        CoherentBox,
        DataDirection,
        DmaAddress, //
    },
    prelude::*,
    scatterlist::{
        Owned,
        SGTable, //
    },
};

use crate::{
    gsp::GSP_PAGE_SIZE,
    num::FromSafeCast, //
};

/// A radix3 page table and the data it maps.
#[pin_data]
pub(crate) struct Radix3<'a> {
    /// The mapped data.
    #[pin]
    data: SGTable<Owned<VVec<u8>>>,
    /// Level 2: one entry per page of `data`.
    #[pin]
    level2: SGTable<Owned<VVec<u8>>>,
    /// Level 1: one entry per page of `level2`.
    #[pin]
    level1: SGTable<Owned<VVec<u8>>>,
    /// Level 0: one page, whose single entry is the DMA address of the first `level1` page.
    level0: Coherent<'a, [u64]>,
    /// Length of `data`, in bytes.
    size: usize,
}

impl<'a> Radix3<'a> {
    /// Builds a radix3 page table over `data`, mapped for `dev` to read. May sleep.
    pub(crate) fn new(
        dev: &'a device::Device<device::Bound>,
        data: VVec<u8>,
    ) -> impl PinInit<Self, Error> + 'a {
        let size = data.len();

        pin_init::pin_init_scope(move || {
            Ok(try_pin_init!(Self {
                data <- SGTable::new(dev, data, DataDirection::ToDevice, GFP_KERNEL),
                level2 <- {
                    // Allocate the level 2 page table, map the firmware onto it, and map it into
                    // the device address space.
                    VVec::<u8>::with_capacity(
                        data.iter().count() * core::mem::size_of::<u64>(),
                        GFP_KERNEL,
                    )
                    .map_err(|_| ENOMEM)
                    .and_then(|level2| map_into_lvl(&data, level2))
                    .map(|level2| SGTable::new(dev, level2, DataDirection::ToDevice, GFP_KERNEL))?
                },
                level1 <- {
                    // Allocate the level 1 page table, map the level 2 page table onto it, and map
                    // it into the device address space.
                    VVec::<u8>::with_capacity(
                        level2.iter().count() * core::mem::size_of::<u64>(),
                        GFP_KERNEL,
                    )
                    .map_err(|_| ENOMEM)
                    .and_then(|level1| map_into_lvl(&level2, level1))
                    .map(|level1| SGTable::new(dev, level1, DataDirection::ToDevice, GFP_KERNEL))?
                },
                level0: {
                    // Allocate the level 0 page table as a device-visible DMA object, and map the
                    // level 1 page table onto it.

                    // Fill level 1 page entry.
                    let level1_entry = level1.iter().next().ok_or(EINVAL)?;
                    let level1_entry_addr = level1_entry.dma_address();

                    // Create level 0 page table data and fill its first entry with the level 1
                    // table.
                    let mut level0 = CoherentBox::<'_, [u64]>::zeroed_slice(
                        dev,
                        GSP_PAGE_SIZE / size_of::<u64>(),
                        GFP_KERNEL
                    )?;
                    level0[0] = level1_entry_addr.to_le();

                    level0.into()
                },
                size,
            }))
        })
    }

    /// Returns the DMA address of the level 0 page, which is the address of the table.
    pub(crate) fn dma_address(&self) -> DmaAddress {
        self.level0.dma_address()
    }

    /// Returns the length of the mapped data, in bytes.
    pub(crate) fn size(&self) -> usize {
        self.size
    }
}

/// Appends one level of the table to `dst`: one entry per [`GSP_PAGE_SIZE`] page of each
/// DMA-mapped region of `sg_table`, in region order.
fn map_into_lvl(sg_table: &SGTable<Owned<VVec<u8>>>, mut dst: VVec<u8>) -> Result<VVec<u8>> {
    for sg_entry in sg_table.iter() {
        let num_pages = usize::from_safe_cast(sg_entry.dma_len()).div_ceil(GSP_PAGE_SIZE);

        for i in 0..num_pages {
            let entry = sg_entry.dma_address()
                + (u64::from_safe_cast(i) * u64::from_safe_cast(GSP_PAGE_SIZE));
            dst.extend_from_slice(&entry.to_le_bytes(), GFP_KERNEL)?;
        }
    }

    Ok(dst)
}
