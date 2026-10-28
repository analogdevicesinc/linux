// SPDX-License-Identifier: GPL-2.0
// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

use core::ops::Range;

use kernel::{
    alloc::ArrayVec,
    bitfield,
    device,
    pci,
    prelude::*,
    transmute::{
        AsBytes,
        FromBytes, //
    }, //
};

use crate::{
    gpu::Chipset,
    gsp::{
        nvkv::{
            nvkv_decode,
            nvkv_encode,
            Accumulated,
            Encodable,
            Encoder,
            Key,
            Required,
            TruncatingArray, //
        },
        GSP_PAGE_SIZE, //
    },
    num::IntoSafeCast,
    vgpu::VgpuState, //
};

use super::bindings;

/// Payload of the `GspSetSystemInfo` command.
#[repr(transparent)]
pub(crate) struct GspSetSystemInfo {
    inner: bindings::GspSystemInfo,
}
static_assert!(size_of::<GspSetSystemInfo>() < GSP_PAGE_SIZE);

impl GspSetSystemInfo {
    /// Builds the VF topology passed to GSP-RM as part of the system information.
    fn build_vf_info(
        dev: &pci::Device<device::Bound>,
        vgpu_state: VgpuState,
    ) -> Result<bindings::GSP_VF_INFO> {
        let VgpuState::Enabled { total_vfs } = vgpu_state else {
            return Ok(Zeroable::zeroed());
        };

        let sriov = dev
            .config_space_extended()?
            .find_ext_capability::<pci::ExtSriovRegs>()?
            .ok_or(ENODEV)?;

        let mut vf_bars = sriov.vf_bars()?;
        let bar0 = vf_bars.next().ok_or(EINVAL)?;
        let bar1 = vf_bars.next().ok_or(EINVAL)?;
        let bar2 = vf_bars.next().ok_or(EINVAL)?;

        Ok(bindings::GSP_VF_INFO {
            totalVFs: u32::from(total_vfs.get()),
            firstVFOffset: u32::from(sriov.first_vf_offset()),
            FirstVFBar0Address: bar0.address,
            FirstVFBar1Address: bar1.address,
            FirstVFBar2Address: bar2.address,
            b64bitBar0: u8::from(bar0.is_64bit),
            b64bitBar1: u8::from(bar1.is_64bit),
            b64bitBar2: u8::from(bar2.is_64bit),
            ..Zeroable::zeroed()
        })
    }

    /// Returns an in-place initializer for the `GspSetSystemInfo` command.
    pub(crate) fn init<'a>(
        dev: &'a pci::Device<device::Bound>,
        chipset: Chipset,
        vgpu_state: VgpuState,
    ) -> impl Init<Self, Error> + 'a {
        type InnerGspSystemInfo = bindings::GspSystemInfo;
        let pci_config_mirror_range = chipset.pci_config_mirror_range();
        let init_inner = try_init!(InnerGspSystemInfo {
            gpuPhysAddr: dev.resource_start(0)?,
            gpuPhysFbAddr: dev.resource_start(1)?,
            gpuPhysInstAddr: dev.resource_start(3)?,
            nvDomainBusDeviceFunc: u64::from(dev.dev_id()),

            // Using TASK_SIZE in r535_gsp_rpc_set_system_info() seems wrong because
            // TASK_SIZE is per-task. That's probably a design issue in GSP-RM though.
            maxUserVa: (1 << 47) - 4096,
            pciConfigMirrorBase: pci_config_mirror_range.start,
            pciConfigMirrorSize: pci_config_mirror_range.end - pci_config_mirror_range.start,

            PCIDeviceID: (u32::from(dev.device_id()) << 16) | u32::from(dev.vendor_id().as_raw()),
            PCISubDeviceID: (u32::from(dev.subsystem_device_id()) << 16)
                | u32::from(dev.subsystem_vendor_id()),
            PCIRevisionID: u32::from(dev.revision_id()),
            bIsPrimary: 0,
            bPreserveVideoMemoryAllocations: 0,
            gspVFInfo: Self::build_vf_info(dev, vgpu_state)?,
            ..Zeroable::init_zeroed()
        });

        try_init!(GspSetSystemInfo {
            inner <- init_inner,
        })
    }
}

// SAFETY: These structs don't meet the no-padding requirements of AsBytes but
//         that is not a problem because they are not used outside the kernel.
unsafe impl AsBytes for GspSetSystemInfo {}

// SAFETY: These structs don't meet the no-padding requirements of FromBytes but
//         that is not a problem because they are not used outside the kernel.
unsafe impl FromBytes for GspSetSystemInfo {}

#[repr(transparent)]
pub(crate) struct PackedRegistryEntry(bindings::PACKED_REGISTRY_ENTRY);

impl PackedRegistryEntry {
    pub(crate) fn new(offset: u32, value: u32) -> Self {
        Self({
            bindings::PACKED_REGISTRY_ENTRY {
                nameOffset: offset,

                // We only support DWORD types for now. Support for other types
                // will come later if required.
                type_: cv!(bindings::REGISTRY_TABLE_ENTRY_TYPE_DWORD),
                __bindgen_padding_0: Default::default(),
                data: value,
                length: 0,
            }
        })
    }
}

// SAFETY: Padding is explicit and will not contain uninitialized data.
unsafe impl AsBytes for PackedRegistryEntry {}

/// Payload of the `SetRegistry` command.
#[repr(transparent)]
pub(crate) struct PackedRegistryTable {
    inner: bindings::PACKED_REGISTRY_TABLE,
}

impl PackedRegistryTable {
    pub(crate) fn init(num_entries: u32, size: u32) -> impl Init<Self> {
        type InnerPackedRegistryTable = bindings::PACKED_REGISTRY_TABLE;
        let init_inner = init!(InnerPackedRegistryTable {
            numEntries: num_entries,
            size,
            entries: Default::default()
        });

        init!(PackedRegistryTable { inner <- init_inner })
    }
}

// SAFETY: Padding is explicit and will not contain uninitialized data.
unsafe impl AsBytes for PackedRegistryTable {}

// SAFETY: This struct only contains integer types for which all bit patterns
// are valid.
unsafe impl FromBytes for PackedRegistryTable {}

/// Payload of the `GetGspStaticInfo` command and message.
#[repr(transparent)]
#[derive(Zeroable)]
pub(crate) struct GspStaticConfigInfo(bindings::GspStaticConfigInfo_t);

impl GspStaticConfigInfo {
    /// Returns a bytes array containing the (hopefully) zero-terminated name of this GPU.
    pub(crate) fn gpu_name_str(&self) -> [u8; 64] {
        self.0.gpuNameString
    }

    /// Returns a bytes array containing the (hopefully) zero-terminated short name of this GPU.
    pub(crate) fn gpu_short_name_str(&self) -> [u8; 64] {
        self.0.gpuShortNameString
    }

    /// Returns the 16-byte SHA-1 GPU identifier supplied by GSP-RM.
    ///
    /// GSP-RM reports the GID in binary SHA-1 form, which occupies the first 16 bytes of the
    /// GID info payload.
    pub(crate) fn gpu_gid(&self) -> [u8; 16] {
        let mut gid = [0u8; 16];
        gid.copy_from_slice(&self.0.gidInfo.data[..16]);
        gid
    }

    /// Returns the BAR1 Page Directory Entry base address.
    ///
    /// This is the root page table address for BAR1 virtual memory,
    /// set up by GSP-RM firmware.
    pub(crate) fn bar1_pde_base(&self) -> u64 {
        self.0.bar1PdeBase
    }

    /// Returns an iterator over valid FB regions from GSP firmware data.
    fn fb_regions(
        &self,
    ) -> impl Iterator<Item = &bindings::NV2080_CTRL_CMD_FB_GET_FB_REGION_FB_REGION_INFO> {
        let fb_info = &self.0.fbRegionInfoParams;
        fb_info
            .fbRegion
            .iter()
            .take(fb_info.numFBRegions.into_safe_cast())
            .filter(|reg| reg.limit >= reg.base)
    }

    /// Iterates over usable FB regions from GSP firmware data.
    ///
    /// Each yielded region is a [`Range<u64>`] suitable for driver memory allocation.
    /// Usable regions are those that satisfy all the following properties:
    /// - Are not reserved for firmware internal use.
    /// - Are not protected (hardware-enforced access restrictions).
    /// - Support compression (can use GPU memory compression for bandwidth).
    /// - Support ISO (isochronous memory for display requiring guaranteed bandwidth).
    pub(crate) fn usable_fb_regions(&self) -> impl Iterator<Item = Range<u64>> + '_ {
        self.fb_regions().filter_map(|reg| {
            // Filter: not reserved, not protected, supports compression and ISO.
            if reg.reserved == 0
                && reg.bProtected == 0
                && reg.supportCompressed != 0
                && reg.supportISO != 0
            {
                reg.limit.checked_add(1).map(|end| reg.base..end)
            } else {
                None
            }
        })
    }

    /// Computes the exclusive end of the FB physical address space.
    pub(crate) fn total_fb_end(&self) -> Option<u64> {
        self.fb_regions().map(|reg| reg.limit).max()?.checked_add(1)
    }
}

// SAFETY: Padding is explicit and will not contain uninitialized data.
unsafe impl AsBytes for GspStaticConfigInfo {}

// SAFETY: This struct only contains integer types for which all bit patterns
// are valid.
unsafe impl FromBytes for GspStaticConfigInfo {}

/// Power level requested to the [`UnloadingGuestDriver`] command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
#[expect(unused)]
pub(crate) enum PowerStateLevel {
    /// Full unload.
    Level0 = bindings::NV2080_CTRL_GPU_SET_POWER_STATE_GPU_LEVEL_0,
    /// S3 (suspend to RAM).
    Level3 = bindings::NV2080_CTRL_GPU_SET_POWER_STATE_GPU_LEVEL_3,
    /// Hibernate (suspend to disk).
    Level7 = bindings::NV2080_CTRL_GPU_SET_POWER_STATE_GPU_LEVEL_7,
}

impl PowerStateLevel {
    /// Returns `true` if this state represents a power management transition, i.e. some GPU state
    /// must survive it (as opposed to a full unload).
    pub(crate) fn is_power_transition(self) -> bool {
        self != PowerStateLevel::Level0
    }
}

/// Payload of the `UnloadingGuestDriver` command and message.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, Zeroable)]
pub(crate) struct UnloadingGuestDriver(bindings::rpc_unloading_guest_driver_v1F_07);

impl UnloadingGuestDriver {
    pub(crate) fn new(level: PowerStateLevel) -> Self {
        Self(bindings::rpc_unloading_guest_driver_v1F_07 {
            bInPMTransition: u8::from(level.is_power_transition()),
            bGc6Entering: 0,
            newLevel: level as u32,
            ..Zeroable::zeroed()
        })
    }
}

// SAFETY: Padding is explicit and will not contain uninitialized data.
unsafe impl AsBytes for UnloadingGuestDriver {}

// SAFETY: This struct only contains integer types for which all bit patterns
// are valid.
unsafe impl FromBytes for UnloadingGuestDriver {}

/// The host CPU architecture.
#[derive(Clone, Copy)]
enum HostArch {
    None = 0,
    X86_64 = 1,
    Ppc64le = 2,
    Arm = 3,
    Aarch64 = 4,
    Riscv64 = 5,
}

// TODO[FPRI]: This is a temporary solution to be replaced with the corresponding derive macros once
// they land.
impl TryFrom<u32> for HostArch {
    type Error = Error;

    fn try_from(value: u32) -> Result<Self> {
        match value {
            0 => Ok(Self::None),
            1 => Ok(Self::X86_64),
            2 => Ok(Self::Ppc64le),
            3 => Ok(Self::Arm),
            4 => Ok(Self::Aarch64),
            5 => Ok(Self::Riscv64),
            _ => Err(EINVAL),
        }
    }
}

impl From<HostArch> for u32 {
    fn from(value: HostArch) -> Self {
        value as u32
    }
}

nvkv_encode! {
    /// A GSP registry entry.
    struct RegKey {
        key_name: Key<&'static [u8], 0x3070>,
        key_value: Key<u32, 0x3071>,
    }
}

impl Encodable for KVVec<RegKey> {
    fn encode(&self, encoder: &mut Encoder) -> Result {
        for regkey in self {
            regkey.encode(encoder)?;
        }
        Ok(())
    }
}

nvkv_encode! {
    /// SR-IOV virtual function information.
    struct VfInfo {
        total_vfs: Key<u32, 0x0080>,
        first_vf_offset: Key<u32, 0x0081>,
        flags: Key<u64, 0x1003>,
        first_bar0_address: Key<u64, 0x1050>,
        first_bar1_address: Key<u64, 0x1051>,
        first_bar2_address: Key<u64, 0x1052>,
    }
}

nvkv_encode! {
    /// Payload of the `GSP_INIT` command.
    // TODO: expect() doesn't work here before rustc 1.97.0
    // https://github.com/rust-lang/rust/pull/154377
    #[allow(dead_code)]
    struct GspInitRequest {
        pci_device_id: Key<u32, 0x0001>,
        pci_sub_device_id: Key<u32, 0x0002>,
        pci_revision_id: Key<u32, 0x0003>,
        pci_config_mirror_base: Key<u32, 0x0010>,
        pci_config_mirror_size: Key<u32, 0x0011>,
        host_arch: Key<HostArch, 0x0070, u32>,
        bus_device_func: Key<u64, 0x1020>,
        regkeys: KVVec<RegKey>,
        vf_info: Option<VfInfo>,
    }
}

// Decode:

// Should decode with UnknownKeyPolicy::Ignore.
nvkv_decode! {
    /// Schema for the `GSP_INIT` response.
    // TODO: expect() doesn't work here before rustc 1.97.0
    // https://github.com/rust-lang/rust/pull/154377
    #[allow(dead_code)]
    struct GspInitResponseSchema => GspInitResponse {
        gpu_name: TruncatingArray<u8, { GspInitResponse::MAX_GPU_NAME_LEN }, 0x2000>,
        fb_regions: Accumulated<FbRegionSchema>,
        bar1_pde_base: Required<Key<u64, 0x1020>>,
        vmmu_segment_size: Key<u64, 0x1050>,
    }
}

/// Payload of the `GSP_INIT` response.
// TODO: expect() doesn't work here before rustc 1.97.0
// https://github.com/rust-lang/rust/pull/154377
#[allow(dead_code)]
struct GspInitResponse {
    gpu_name: ArrayVec<u8, { Self::MAX_GPU_NAME_LEN }>,
    fb_regions: KVVec<FbRegion>,
    bar1_pde_base: u64,
    vmmu_segment_size: u64,
}

impl GspInitResponse {
    const MAX_GPU_NAME_LEN: usize = 64;
}

nvkv_decode! {
    /// Schema for one FB region of the `GSP_INIT` response.
    struct FbRegionSchema => FbRegion {
        base: Required<Key<u64, 0x1011>>,
        limit: Required<Key<u64, 0x1012>>,
        flags: Required<Key<FbRegionFlags, 0x0012, u32>>,
        tag: Required<Key<u32, 0x0013>>,
    }
}

bitfield! {
    /// FB region attribute flags.
    #[derive(Default)]
    struct FbRegionFlags(u32) {
        0:0 support_compressed => bool;
        1:1 support_iso => bool;
        2:2 protected => bool;
    }
}

/// One FB memory region.
struct FbRegion {
    base: u64,
    limit: u64,
    flags: FbRegionFlags,
    tag: u32,
}
