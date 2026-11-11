// SPDX-License-Identifier: GPL-2.0
// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

pub(crate) mod commands;
mod r000_00;
mod r570_144;

// Alias to avoid repeating the version number with every use.
use r570_144 as bindings;

use core::ops::Range;

use kernel::{
    bitfield,
    dma::{
        Coherent,
        CoherentView, //
    },
    io::{
        io_read,
        io_write, //
    },
    prelude::*,
    ptr::{
        Alignable,
        Alignment,
        KnownSize, //
    },
    sizes::SizeConstants,
    transmute::{
        AsBytes,
        FromBytes, //
    },
};

use crate::{
    bounded_enum,
    fb::{
        FbRanges,
        FbSizes, //
    },
    firmware::gsp::GspFirmware,
    gpu::{
        Architecture,
        Chipset, //
    },
    gsp::{
        cmdq::Cmdq, //
        GSP_PAGE_SIZE,
    },
    num::{
        self,
        FromSafeCast, //
    },
};

/// Maximum size of a single GSP message queue element in bytes.
pub(crate) const GSP_MSG_QUEUE_ELEMENT_SIZE_MAX: usize =
    cv!(bindings::GSP_MSG_QUEUE_ELEMENT_SIZE_MAX);

/// Empty type to group methods related to heap parameters for running the GSP firmware.
enum GspFwHeapParams {}

/// Minimum required alignment for the GSP heap.
const GSP_HEAP_ALIGNMENT: Alignment = Alignment::SZ_1M;

impl GspFwHeapParams {
    /// Returns the amount of GSP-RM heap memory used during GSP-RM boot and initialization (up to
    /// and including the first client subdevice allocation).
    fn base_rm_size(chipset: Chipset) -> u64 {
        match chipset.arch() {
            Architecture::Turing | Architecture::Ampere | Architecture::Ada => {
                u64::from(bindings::GSP_FW_HEAP_PARAM_BASE_RM_SIZE_TU10X)
            }
            Architecture::Hopper | Architecture::BlackwellGB10x | Architecture::BlackwellGB20x => {
                u64::from(bindings::GSP_FW_HEAP_PARAM_BASE_RM_SIZE_GH100)
            }
        }
    }

    /// Returns the amount of heap memory required to support a single channel allocation.
    fn client_alloc_size() -> u64 {
        u64::from(bindings::GSP_FW_HEAP_PARAM_CLIENT_ALLOC_SIZE)
            .align_up(GSP_HEAP_ALIGNMENT)
            .unwrap_or(u64::MAX)
    }

    /// Returns the amount of memory to reserve for management purposes for a framebuffer of size
    /// `fb_size`.
    fn management_overhead(fb_size: u64) -> Result<u64> {
        let fb_size_gb = fb_size.div_ceil(u64::SZ_1G);

        u64::from(bindings::GSP_FW_HEAP_PARAM_SIZE_PER_GB_FB)
            .checked_mul(fb_size_gb)
            .ok_or(EINVAL)?
            .align_up(GSP_HEAP_ALIGNMENT)
            .ok_or(EINVAL)
    }
}

/// Heap memory requirements and constraints for a given version of the GSP LIBOS.
pub(crate) struct LibosParams {
    /// The base amount of heap required by the GSP operating system, in bytes.
    carveout_size: u64,
    /// The minimum and maximum sizes allowed for the GSP FW heap, in bytes.
    allowed_heap_size: Range<u64>,
}

impl LibosParams {
    /// Version 2 of the GSP LIBOS (Turing and GA100)
    const LIBOS2: LibosParams = LibosParams {
        carveout_size: cv!(bindings::GSP_FW_HEAP_PARAM_OS_SIZE_LIBOS2),
        allowed_heap_size: cv!(bindings::GSP_FW_HEAP_SIZE_OVERRIDE_LIBOS2_MIN_MB => u64)
            * u64::SZ_1M
            ..cv!(bindings::GSP_FW_HEAP_SIZE_OVERRIDE_LIBOS2_MAX_MB => u64) * u64::SZ_1M,
    };

    /// Version 3 of the GSP LIBOS (GA102+)
    const LIBOS3: LibosParams = LibosParams {
        carveout_size: cv!(bindings::GSP_FW_HEAP_PARAM_OS_SIZE_LIBOS3_BAREMETAL),
        allowed_heap_size: cv!(bindings::GSP_FW_HEAP_SIZE_OVERRIDE_LIBOS3_BAREMETAL_MIN_MB => u64)
            * u64::SZ_1M
            ..cv!(bindings::GSP_FW_HEAP_SIZE_OVERRIDE_LIBOS3_BAREMETAL_MAX_MB => u64) * u64::SZ_1M,
    };

    /// Returns the libos parameters corresponding to `chipset`.
    pub(crate) fn from_chipset(chipset: Chipset) -> &'static LibosParams {
        if chipset < Chipset::GA102 {
            &Self::LIBOS2
        } else {
            &Self::LIBOS3
        }
    }

    /// Returns the WPR heap size to reserve when vGPU is enabled.
    pub(crate) fn vgpu_wpr_heap_size() -> u64 {
        u64::from(bindings::GSP_FW_HEAP_SIZE_VGPU_DEFAULT)
    }

    /// Returns the amount of memory (in bytes) to allocate for the WPR heap for a framebuffer size
    /// of `fb_size` (in bytes) for `chipset`.
    pub(crate) fn wpr_heap_size(&self, chipset: Chipset, fb_size: u64) -> Result<u64> {
        // The WPR heap will contain the following:
        // LIBOS carveout,
        Ok(self
            .carveout_size
            // RM boot working memory,
            .saturating_add(GspFwHeapParams::base_rm_size(chipset))
            // One RM client,
            .saturating_add(GspFwHeapParams::client_alloc_size())
            // Overhead for memory management.
            .saturating_add(GspFwHeapParams::management_overhead(fb_size)?)
            // Clamp to the supported heap sizes.
            .clamp(self.allowed_heap_size.start, self.allowed_heap_size.end - 1))
    }
}

/// Structure passed to the GSP bootloader, containing the framebuffer layout as well as the DMA
/// addresses of the GSP bootloader and firmware.
#[repr(transparent)]
pub(crate) struct GspFwWprMeta {
    inner: bindings::GspFwWprMeta,
}

// SAFETY: Padding is explicit and does not contain uninitialized data.
unsafe impl AsBytes for GspFwWprMeta {}

// SAFETY: This struct only contains integer types for which all bit patterns
// are valid.
unsafe impl FromBytes for GspFwWprMeta {}

type GspFwWprMetaBootResumeInfo = bindings::GspFwWprMeta__bindgen_ty_1;
type GspFwWprMetaBootInfo = bindings::GspFwWprMeta__bindgen_ty_1__bindgen_ty_1;

impl GspFwWprMeta {
    /// Returns an initializer for a `GspFwWprMeta` suitable for booting `gsp_firmware` using the
    /// framebuffer ranges `ranges`.
    pub(crate) fn from_ranges<'a>(
        gsp_firmware: &'a GspFirmware<'_>,
        ranges: &'a FbRanges,
    ) -> impl Init<Self> + 'a {
        let init_inner = init!(bindings::GspFwWprMeta {
            // CAST: we want to store the bits of `GSP_FW_WPR_META_MAGIC` unmodified.
            magic: bindings::GSP_FW_WPR_META_MAGIC as u64,
            revision: u64::from(bindings::GSP_FW_WPR_META_REVISION),
            sysmemAddrOfRadix3Elf: gsp_firmware.radix3_dma_address(),
            sizeOfRadix3Elf: u64::from_safe_cast(gsp_firmware.size()),
            sysmemAddrOfBootloader: gsp_firmware.bootloader.ucode.dma_address(),
            sizeOfBootloader: u64::from_safe_cast(gsp_firmware.bootloader.ucode.size()),
            bootloaderCodeOffset: u64::from(gsp_firmware.bootloader.code_offset),
            bootloaderDataOffset: u64::from(gsp_firmware.bootloader.data_offset),
            bootloaderManifestOffset: u64::from(gsp_firmware.bootloader.manifest_offset),
            __bindgen_anon_1: GspFwWprMetaBootResumeInfo {
                __bindgen_anon_1: GspFwWprMetaBootInfo {
                    sysmemAddrOfSignature: gsp_firmware.signatures.dma_address(),
                    sizeOfSignature: u64::from_safe_cast(gsp_firmware.signatures.size()),
                },
            },
            gspFwRsvdStart: ranges.non_wpr_heap.start,
            nonWprHeapOffset: ranges.non_wpr_heap.start,
            nonWprHeapSize: ranges.non_wpr_heap.len(),
            gspFwWprStart: ranges.wpr2.start,
            gspFwHeapOffset: ranges.wpr2_heap.start,
            gspFwHeapSize: ranges.wpr2_heap.len(),
            gspFwOffset: ranges.fw_image.start,
            bootBinOffset: ranges.boot.start,
            frtsOffset: ranges.frts.start,
            frtsSize: ranges.frts.len(),
            gspFwWprEnd: ranges.vga_workspace.start.align_down(Alignment::SZ_128K),
            gspFwHeapVfPartitionCount: ranges.vf_partition_count,
            fbSize: ranges.fb.len(),
            vgaWorkspaceOffset: ranges.vga_workspace.start,
            vgaWorkspaceSize: ranges.vga_workspace.len(),
            pmuReservedSize: ranges.pmu_reserved_size,
            ..Zeroable::init_zeroed()
        });

        init!(GspFwWprMeta {
            inner <- init_inner,
        })
    }

    /// Returns an initializer for a `GspFwWprMeta` suitable for booting `gsp_firmware` using the
    /// framebuffer region sizes `sizes`.
    ///
    /// The region offsets are left at zero: the ACR ucode computes them when it sets up WPR2.
    pub(crate) fn from_sizes<'a>(
        gsp_firmware: &'a GspFirmware<'_>,
        sizes: &'a FbSizes,
    ) -> impl Init<Self> + 'a {
        /// VGA workspace size to reserve at the end of the framebuffer, in bytes.
        const VGA_WORKSPACE_SIZE: u64 = u64::SZ_128K;

        let init_inner = init!(bindings::GspFwWprMeta {
            // CAST: we want to store the bits of `GSP_FW_WPR_META_MAGIC` unmodified.
            magic: bindings::GSP_FW_WPR_META_MAGIC as u64,
            revision: u64::from(bindings::GSP_FW_WPR_META_REVISION),
            sysmemAddrOfRadix3Elf: gsp_firmware.radix3_dma_address(),
            sizeOfRadix3Elf: u64::from_safe_cast(gsp_firmware.size()),
            sysmemAddrOfBootloader: gsp_firmware.bootloader.ucode.dma_address(),
            sizeOfBootloader: u64::from_safe_cast(gsp_firmware.bootloader.ucode.size()),
            bootloaderCodeOffset: u64::from(gsp_firmware.bootloader.code_offset),
            bootloaderDataOffset: u64::from(gsp_firmware.bootloader.data_offset),
            bootloaderManifestOffset: u64::from(gsp_firmware.bootloader.manifest_offset),
            __bindgen_anon_1: GspFwWprMetaBootResumeInfo {
                __bindgen_anon_1: GspFwWprMetaBootInfo {
                    sysmemAddrOfSignature: gsp_firmware.signatures.dma_address(),
                    sizeOfSignature: u64::from_safe_cast(gsp_firmware.signatures.size()),
                },
            },
            nonWprHeapSize: sizes.non_wpr_heap_size,
            gspFwHeapSize: sizes.wpr2_heap_size,
            frtsSize: sizes.frts_size,
            gspFwHeapVfPartitionCount: sizes.vf_partition_count,
            vgaWorkspaceSize: VGA_WORKSPACE_SIZE,
            pmuReservedSize: sizes.pmu_reserved_size,
            ..Zeroable::init_zeroed()
        });

        init!(GspFwWprMeta {
            inner <- init_inner,
        })
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u32)]
pub(crate) enum MsgFunction {
    // Common function codes
    AllocChannelDma = bindings::NV_VGPU_MSG_FUNCTION_ALLOC_CHANNEL_DMA,
    AllocCtxDma = bindings::NV_VGPU_MSG_FUNCTION_ALLOC_CTX_DMA,
    AllocDevice = bindings::NV_VGPU_MSG_FUNCTION_ALLOC_DEVICE,
    AllocMemory = bindings::NV_VGPU_MSG_FUNCTION_ALLOC_MEMORY,
    AllocObject = bindings::NV_VGPU_MSG_FUNCTION_ALLOC_OBJECT,
    AllocRoot = bindings::NV_VGPU_MSG_FUNCTION_ALLOC_ROOT,
    BindCtxDma = bindings::NV_VGPU_MSG_FUNCTION_BIND_CTX_DMA,
    ContinuationRecord = bindings::NV_VGPU_MSG_FUNCTION_CONTINUATION_RECORD,
    Free = bindings::NV_VGPU_MSG_FUNCTION_FREE,
    GetGspStaticInfo = bindings::NV_VGPU_MSG_FUNCTION_GET_GSP_STATIC_INFO,
    GetStaticInfo = bindings::NV_VGPU_MSG_FUNCTION_GET_STATIC_INFO,
    GspInitPostObjGpu = bindings::NV_VGPU_MSG_FUNCTION_GSP_INIT_POST_OBJGPU,
    GspRmControl = bindings::NV_VGPU_MSG_FUNCTION_GSP_RM_CONTROL,
    GspSetSystemInfo = bindings::NV_VGPU_MSG_FUNCTION_GSP_SET_SYSTEM_INFO,
    Log = bindings::NV_VGPU_MSG_FUNCTION_LOG,
    MapMemory = bindings::NV_VGPU_MSG_FUNCTION_MAP_MEMORY,
    Nop = bindings::NV_VGPU_MSG_FUNCTION_NOP,
    SetGuestSystemInfo = bindings::NV_VGPU_MSG_FUNCTION_SET_GUEST_SYSTEM_INFO,
    SetRegistry = bindings::NV_VGPU_MSG_FUNCTION_SET_REGISTRY,
    UnloadingGuestDriver = bindings::NV_VGPU_MSG_FUNCTION_UNLOADING_GUEST_DRIVER,

    // Event codes
    GspInitDone = bindings::NV_VGPU_MSG_EVENT_GSP_INIT_DONE,
    GspLockdownNotice = bindings::NV_VGPU_MSG_EVENT_GSP_LOCKDOWN_NOTICE,
    GspPostNoCat = bindings::NV_VGPU_MSG_EVENT_GSP_POST_NOCAT_RECORD,
    GspRunCpuSequencer = bindings::NV_VGPU_MSG_EVENT_GSP_RUN_CPU_SEQUENCER,
    MmuFaultQueued = bindings::NV_VGPU_MSG_EVENT_MMU_FAULT_QUEUED,
    OsErrorLog = bindings::NV_VGPU_MSG_EVENT_OS_ERROR_LOG,
    PostEvent = bindings::NV_VGPU_MSG_EVENT_POST_EVENT,
    RcTriggered = bindings::NV_VGPU_MSG_EVENT_RC_TRIGGERED,
    UcodeLibOsPrint = bindings::NV_VGPU_MSG_EVENT_UCODE_LIBOS_PRINT,
}

impl TryFrom<u32> for MsgFunction {
    type Error = kernel::error::Error;

    fn try_from(value: u32) -> Result<MsgFunction> {
        match value {
            // Common function codes
            bindings::NV_VGPU_MSG_FUNCTION_ALLOC_CHANNEL_DMA => Ok(MsgFunction::AllocChannelDma),
            bindings::NV_VGPU_MSG_FUNCTION_ALLOC_CTX_DMA => Ok(MsgFunction::AllocCtxDma),
            bindings::NV_VGPU_MSG_FUNCTION_ALLOC_DEVICE => Ok(MsgFunction::AllocDevice),
            bindings::NV_VGPU_MSG_FUNCTION_ALLOC_MEMORY => Ok(MsgFunction::AllocMemory),
            bindings::NV_VGPU_MSG_FUNCTION_ALLOC_OBJECT => Ok(MsgFunction::AllocObject),
            bindings::NV_VGPU_MSG_FUNCTION_ALLOC_ROOT => Ok(MsgFunction::AllocRoot),
            bindings::NV_VGPU_MSG_FUNCTION_BIND_CTX_DMA => Ok(MsgFunction::BindCtxDma),
            bindings::NV_VGPU_MSG_FUNCTION_CONTINUATION_RECORD => {
                Ok(MsgFunction::ContinuationRecord)
            }
            bindings::NV_VGPU_MSG_FUNCTION_FREE => Ok(MsgFunction::Free),
            bindings::NV_VGPU_MSG_FUNCTION_GET_GSP_STATIC_INFO => Ok(MsgFunction::GetGspStaticInfo),
            bindings::NV_VGPU_MSG_FUNCTION_GET_STATIC_INFO => Ok(MsgFunction::GetStaticInfo),
            bindings::NV_VGPU_MSG_FUNCTION_GSP_INIT_POST_OBJGPU => {
                Ok(MsgFunction::GspInitPostObjGpu)
            }
            bindings::NV_VGPU_MSG_FUNCTION_GSP_RM_CONTROL => Ok(MsgFunction::GspRmControl),
            bindings::NV_VGPU_MSG_FUNCTION_GSP_SET_SYSTEM_INFO => Ok(MsgFunction::GspSetSystemInfo),
            bindings::NV_VGPU_MSG_FUNCTION_LOG => Ok(MsgFunction::Log),
            bindings::NV_VGPU_MSG_FUNCTION_MAP_MEMORY => Ok(MsgFunction::MapMemory),
            bindings::NV_VGPU_MSG_FUNCTION_NOP => Ok(MsgFunction::Nop),
            bindings::NV_VGPU_MSG_FUNCTION_SET_GUEST_SYSTEM_INFO => {
                Ok(MsgFunction::SetGuestSystemInfo)
            }
            bindings::NV_VGPU_MSG_FUNCTION_SET_REGISTRY => Ok(MsgFunction::SetRegistry),
            bindings::NV_VGPU_MSG_FUNCTION_UNLOADING_GUEST_DRIVER => {
                Ok(MsgFunction::UnloadingGuestDriver)
            }

            // Event codes
            bindings::NV_VGPU_MSG_EVENT_GSP_INIT_DONE => Ok(MsgFunction::GspInitDone),
            bindings::NV_VGPU_MSG_EVENT_GSP_LOCKDOWN_NOTICE => Ok(MsgFunction::GspLockdownNotice),
            bindings::NV_VGPU_MSG_EVENT_GSP_POST_NOCAT_RECORD => Ok(MsgFunction::GspPostNoCat),
            bindings::NV_VGPU_MSG_EVENT_GSP_RUN_CPU_SEQUENCER => {
                Ok(MsgFunction::GspRunCpuSequencer)
            }
            bindings::NV_VGPU_MSG_EVENT_MMU_FAULT_QUEUED => Ok(MsgFunction::MmuFaultQueued),
            bindings::NV_VGPU_MSG_EVENT_OS_ERROR_LOG => Ok(MsgFunction::OsErrorLog),
            bindings::NV_VGPU_MSG_EVENT_POST_EVENT => Ok(MsgFunction::PostEvent),
            bindings::NV_VGPU_MSG_EVENT_RC_TRIGGERED => Ok(MsgFunction::RcTriggered),
            bindings::NV_VGPU_MSG_EVENT_UCODE_LIBOS_PRINT => Ok(MsgFunction::UcodeLibOsPrint),
            _ => Err(EINVAL),
        }
    }
}

impl From<MsgFunction> for u32 {
    fn from(value: MsgFunction) -> Self {
        // CAST: `MsgFunction` is `repr(u32)` and can thus be cast losslessly.
        value as u32
    }
}

/// Sequencer buffer opcode for GSP sequencer commands.
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u32)]
pub(crate) enum SeqBufOpcode {
    // Core operation opcodes
    CoreReset = bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_CORE_RESET,
    CoreResume = bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_CORE_RESUME,
    CoreStart = bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_CORE_START,
    CoreWaitForHalt = bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_CORE_WAIT_FOR_HALT,

    // Delay opcode
    DelayUs = bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_DELAY_US,

    // Register operation opcodes
    RegModify = bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_REG_MODIFY,
    RegPoll = bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_REG_POLL,
    RegStore = bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_REG_STORE,
    RegWrite = bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_REG_WRITE,
}

impl TryFrom<u32> for SeqBufOpcode {
    type Error = kernel::error::Error;

    fn try_from(value: u32) -> Result<SeqBufOpcode> {
        match value {
            bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_CORE_RESET => {
                Ok(SeqBufOpcode::CoreReset)
            }
            bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_CORE_RESUME => {
                Ok(SeqBufOpcode::CoreResume)
            }
            bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_CORE_START => {
                Ok(SeqBufOpcode::CoreStart)
            }
            bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_CORE_WAIT_FOR_HALT => {
                Ok(SeqBufOpcode::CoreWaitForHalt)
            }
            bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_DELAY_US => Ok(SeqBufOpcode::DelayUs),
            bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_REG_MODIFY => {
                Ok(SeqBufOpcode::RegModify)
            }
            bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_REG_POLL => Ok(SeqBufOpcode::RegPoll),
            bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_REG_STORE => Ok(SeqBufOpcode::RegStore),
            bindings::GSP_SEQ_BUF_OPCODE_GSP_SEQ_BUF_OPCODE_REG_WRITE => Ok(SeqBufOpcode::RegWrite),
            _ => Err(EINVAL),
        }
    }
}

impl From<SeqBufOpcode> for u32 {
    fn from(value: SeqBufOpcode) -> Self {
        // CAST: `SeqBufOpcode` is `repr(u32)` and can thus be cast losslessly.
        value as u32
    }
}

/// Wrapper for GSP sequencer register write payload.
#[repr(transparent)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct RegWritePayload(bindings::GSP_SEQ_BUF_PAYLOAD_REG_WRITE);

impl RegWritePayload {
    /// Returns the register address.
    pub(crate) fn addr(&self) -> u32 {
        self.0.addr
    }

    /// Returns the value to write.
    pub(crate) fn val(&self) -> u32 {
        self.0.val
    }
}

// SAFETY: This struct only contains integer types for which all bit patterns are valid.
unsafe impl FromBytes for RegWritePayload {}

// SAFETY: Padding is explicit and will not contain uninitialized data.
unsafe impl AsBytes for RegWritePayload {}

/// Wrapper for GSP sequencer register modify payload.
#[repr(transparent)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct RegModifyPayload(bindings::GSP_SEQ_BUF_PAYLOAD_REG_MODIFY);

impl RegModifyPayload {
    /// Returns the register address.
    pub(crate) fn addr(&self) -> u32 {
        self.0.addr
    }

    /// Returns the mask to apply.
    pub(crate) fn mask(&self) -> u32 {
        self.0.mask
    }

    /// Returns the value to write.
    pub(crate) fn val(&self) -> u32 {
        self.0.val
    }
}

// SAFETY: This struct only contains integer types for which all bit patterns are valid.
unsafe impl FromBytes for RegModifyPayload {}

// SAFETY: Padding is explicit and will not contain uninitialized data.
unsafe impl AsBytes for RegModifyPayload {}

/// Wrapper for GSP sequencer register poll payload.
#[repr(transparent)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct RegPollPayload(bindings::GSP_SEQ_BUF_PAYLOAD_REG_POLL);

impl RegPollPayload {
    /// Returns the register address.
    pub(crate) fn addr(&self) -> u32 {
        self.0.addr
    }

    /// Returns the mask to apply.
    pub(crate) fn mask(&self) -> u32 {
        self.0.mask
    }

    /// Returns the expected value.
    pub(crate) fn val(&self) -> u32 {
        self.0.val
    }

    /// Returns the timeout in microseconds.
    pub(crate) fn timeout(&self) -> u32 {
        self.0.timeout
    }
}

// SAFETY: This struct only contains integer types for which all bit patterns are valid.
unsafe impl FromBytes for RegPollPayload {}

// SAFETY: Padding is explicit and will not contain uninitialized data.
unsafe impl AsBytes for RegPollPayload {}

/// Wrapper for GSP sequencer delay payload.
#[repr(transparent)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct DelayUsPayload(bindings::GSP_SEQ_BUF_PAYLOAD_DELAY_US);

impl DelayUsPayload {
    /// Returns the delay value in microseconds.
    pub(crate) fn val(&self) -> u32 {
        self.0.val
    }
}

// SAFETY: This struct only contains integer types for which all bit patterns are valid.
unsafe impl FromBytes for DelayUsPayload {}

// SAFETY: Padding is explicit and will not contain uninitialized data.
unsafe impl AsBytes for DelayUsPayload {}

/// Wrapper for GSP sequencer register store payload.
#[repr(transparent)]
#[derive(Copy, Clone, Debug)]
pub(crate) struct RegStorePayload(bindings::GSP_SEQ_BUF_PAYLOAD_REG_STORE);

impl RegStorePayload {
    /// Returns the register address.
    pub(crate) fn addr(&self) -> u32 {
        self.0.addr
    }

    /// Returns the storage index.
    #[allow(unused)]
    pub(crate) fn index(&self) -> u32 {
        self.0.index
    }
}

// SAFETY: This struct only contains integer types for which all bit patterns are valid.
unsafe impl FromBytes for RegStorePayload {}

// SAFETY: Padding is explicit and will not contain uninitialized data.
unsafe impl AsBytes for RegStorePayload {}

/// Wrapper for GSP sequencer buffer command.
#[repr(transparent)]
pub(crate) struct SequencerBufferCmd(bindings::GSP_SEQUENCER_BUFFER_CMD);

impl SequencerBufferCmd {
    /// Returns the opcode as a `SeqBufOpcode` enum, or error if invalid.
    pub(crate) fn opcode(&self) -> Result<SeqBufOpcode> {
        self.0.opCode.try_into()
    }

    /// Returns the register write payload by value.
    ///
    /// Returns an error if the opcode is not `SeqBufOpcode::RegWrite`.
    pub(crate) fn reg_write_payload(&self) -> Result<RegWritePayload> {
        if self.opcode()? != SeqBufOpcode::RegWrite {
            return Err(EINVAL);
        }
        // SAFETY: Opcode is verified to be `RegWrite`, so union contains valid `RegWritePayload`.
        Ok(RegWritePayload(unsafe { self.0.payload.regWrite }))
    }

    /// Returns the register modify payload by value.
    ///
    /// Returns an error if the opcode is not `SeqBufOpcode::RegModify`.
    pub(crate) fn reg_modify_payload(&self) -> Result<RegModifyPayload> {
        if self.opcode()? != SeqBufOpcode::RegModify {
            return Err(EINVAL);
        }
        // SAFETY: Opcode is verified to be `RegModify`, so union contains valid `RegModifyPayload`.
        Ok(RegModifyPayload(unsafe { self.0.payload.regModify }))
    }

    /// Returns the register poll payload by value.
    ///
    /// Returns an error if the opcode is not `SeqBufOpcode::RegPoll`.
    pub(crate) fn reg_poll_payload(&self) -> Result<RegPollPayload> {
        if self.opcode()? != SeqBufOpcode::RegPoll {
            return Err(EINVAL);
        }
        // SAFETY: Opcode is verified to be `RegPoll`, so union contains valid `RegPollPayload`.
        Ok(RegPollPayload(unsafe { self.0.payload.regPoll }))
    }

    /// Returns the delay payload by value.
    ///
    /// Returns an error if the opcode is not `SeqBufOpcode::DelayUs`.
    pub(crate) fn delay_us_payload(&self) -> Result<DelayUsPayload> {
        if self.opcode()? != SeqBufOpcode::DelayUs {
            return Err(EINVAL);
        }
        // SAFETY: Opcode is verified to be `DelayUs`, so union contains valid `DelayUsPayload`.
        Ok(DelayUsPayload(unsafe { self.0.payload.delayUs }))
    }

    /// Returns the register store payload by value.
    ///
    /// Returns an error if the opcode is not `SeqBufOpcode::RegStore`.
    pub(crate) fn reg_store_payload(&self) -> Result<RegStorePayload> {
        if self.opcode()? != SeqBufOpcode::RegStore {
            return Err(EINVAL);
        }
        // SAFETY: Opcode is verified to be `RegStore`, so union contains valid `RegStorePayload`.
        Ok(RegStorePayload(unsafe { self.0.payload.regStore }))
    }
}

// SAFETY: This struct only contains integer types for which all bit patterns are valid.
unsafe impl FromBytes for SequencerBufferCmd {}

// SAFETY: Padding is explicit and will not contain uninitialized data.
unsafe impl AsBytes for SequencerBufferCmd {}

/// Wrapper for GSP run CPU sequencer RPC.
#[repr(transparent)]
pub(crate) struct RunCpuSequencer(bindings::rpc_run_cpu_sequencer_v17_00);

impl RunCpuSequencer {
    /// Returns the command index.
    pub(crate) fn cmd_index(&self) -> u32 {
        self.0.cmdIndex
    }
}

// SAFETY: This struct only contains integer types for which all bit patterns are valid.
unsafe impl FromBytes for RunCpuSequencer {}

// SAFETY: Padding is explicit and will not contain uninitialized data.
unsafe impl AsBytes for RunCpuSequencer {}

/// Struct containing the arguments required to pass a memory buffer to the GSP
/// for use during initialisation.
///
/// The GSP only understands 4K pages (GSP_PAGE_SIZE), so even if the kernel is
/// configured for a larger page size (e.g. 64K pages), we need to give
/// the GSP an array of 4K pages. Since we only create physically contiguous
/// buffers the math to calculate the addresses is simple.
///
/// The buffers must be a multiple of GSP_PAGE_SIZE.  GSP-RM also currently
/// ignores the @kind field for LOGINIT, LOGINTR, and LOGRM, but expects the
/// buffers to be physically contiguous anyway.
///
/// The memory allocated for the arguments must remain until the GSP sends the
/// init_done RPC.
#[repr(transparent)]
pub(crate) struct LibosMemoryRegionInitArgument {
    inner: bindings::LibosMemoryRegionInitArgument,
}

// SAFETY: Padding is explicit and does not contain uninitialized data.
unsafe impl AsBytes for LibosMemoryRegionInitArgument {}

// SAFETY: This struct only contains integer types for which all bit patterns
// are valid.
unsafe impl FromBytes for LibosMemoryRegionInitArgument {}

impl LibosMemoryRegionInitArgument {
    pub(crate) fn new<'a, A: AsBytes + FromBytes + KnownSize + ?Sized>(
        name: &'static str,
        obj: &'a Coherent<'_, A>,
    ) -> impl Init<Self> + 'a {
        /// Generates the `ID8` identifier required for some GSP objects.
        fn id8(name: &str) -> u64 {
            let mut bytes = [0u8; core::mem::size_of::<u64>()];

            for (c, b) in name.bytes().rev().zip(&mut bytes) {
                *b = c;
            }

            u64::from_ne_bytes(bytes)
        }

        let init_inner = init!(bindings::LibosMemoryRegionInitArgument {
            id8: id8(name),
            pa: obj.dma_address(),
            size: num::usize_as_u64(obj.size()),
            kind: cv!(bindings::LibosMemoryRegionKind_LIBOS_MEMORY_REGION_CONTIGUOUS),
            loc: cv!(bindings::LibosMemoryRegionLoc_LIBOS_MEMORY_REGION_LOC_SYSMEM),
            ..Zeroable::init_zeroed()
        });

        init!(LibosMemoryRegionInitArgument {
            inner <- init_inner,
        })
    }
}

/// TX header for setting up a message queue with the GSP.
#[repr(transparent)]
pub(crate) struct MsgqTxHeader(bindings::msgqTxHeader);

impl MsgqTxHeader {
    /// Create a new TX queue header.
    ///
    /// # Arguments
    ///
    /// * `msgq_size` - Total size of the message queue structure, in bytes.
    /// * `rx_hdr_offset` - Offset, in bytes, of the start of the RX header in the message queue
    ///   structure.
    /// * `msg_count` - Number of messages that can be sent, i.e. the number of memory pages
    ///   allocated for the message queue in the message queue structure.
    pub(crate) fn new(msgq_size: u32, rx_hdr_offset: u32, msg_count: u32) -> Self {
        Self(bindings::msgqTxHeader {
            version: 0,
            size: msgq_size,
            msgSize: cv!(GSP_PAGE_SIZE),
            msgCount: msg_count,
            writePtr: 0,
            flags: 1,
            rxHdrOff: rx_hdr_offset,
            entryOff: cv!(GSP_PAGE_SIZE),
        })
    }

    /// Returns the value of the write pointer for this queue.
    pub(crate) fn write_ptr(this: CoherentView<'_, Self>) -> u32 {
        io_read!(this, .0.writePtr)
    }

    /// Sets the value of the write pointer for this queue.
    pub(crate) fn set_write_ptr(this: CoherentView<'_, Self>, val: u32) {
        io_write!(this, .0.writePtr, val)
    }
}

// SAFETY: Padding is explicit and does not contain uninitialized data.
unsafe impl AsBytes for MsgqTxHeader {}

/// RX header for setting up a message queue with the GSP.
#[repr(transparent)]
pub(crate) struct MsgqRxHeader(bindings::msgqRxHeader);

/// Header for the message RX queue.
impl MsgqRxHeader {
    /// Creates a new RX queue header.
    pub(crate) fn new() -> Self {
        Self(Default::default())
    }

    /// Returns the value of the read pointer for this queue.
    pub(crate) fn read_ptr(this: CoherentView<'_, Self>) -> u32 {
        io_read!(this, .0.readPtr)
    }

    /// Sets the value of the read pointer for this queue.
    pub(crate) fn set_read_ptr(this: CoherentView<'_, Self>, val: u32) {
        io_write!(this, .0.readPtr, val)
    }
}

// SAFETY: Padding is explicit and does not contain uninitialized data.
unsafe impl AsBytes for MsgqRxHeader {}

bitfield! {
    struct MsgHeaderVersion(u32) {
        31:24 major;
        23:16 minor;
    }
}

impl MsgHeaderVersion {
    const MAJOR_TOT: u8 = 3;
    const MINOR_TOT: u8 = 0;

    fn new() -> Self {
        Self::zeroed()
            .with_major(Self::MAJOR_TOT)
            .with_minor(Self::MINOR_TOT)
    }
}

/// GSP Message Element.
///
/// This is essentially a message header expected to be followed by the message data.
#[repr(transparent)]
pub(crate) struct GspMsgElement {
    inner: bindings::GSP_MSG_QUEUE_ELEMENT,
}

impl GspMsgElement {
    /// Creates a new message element.
    ///
    /// The RPC header is left initialized to zero and must be initialized separately using e.g.
    /// [`Self::rpc_header_mut`].
    ///
    /// # Arguments
    ///
    /// * `sequence` - Sequence number of the message.
    /// * `cmd_size` - Size of the command (not including the message element), in bytes.
    pub(crate) fn init(sequence: u32, cmd_size: usize) -> impl Init<Self, Error> {
        type InnerGspMsgElement = bindings::GSP_MSG_QUEUE_ELEMENT;
        let init_inner = try_init!(InnerGspMsgElement {
            seqNum: sequence,
            elemCount: size_of::<Self>()
                .checked_add(cmd_size)
                .ok_or(EOVERFLOW)?
                .div_ceil(GSP_PAGE_SIZE)
                .try_into()
                .map_err(|_| EOVERFLOW)?,
            ..Zeroable::init_zeroed()
        });

        try_init!(GspMsgElement {
            inner <- init_inner,
        })
    }

    /// Sets the checksum of this message.
    ///
    /// Since the header is also part of the checksum, this is usually called after the whole
    /// message has been written to the shared memory area.
    pub(crate) fn set_checksum(&mut self, checksum: u32) {
        self.inner.checkSum = checksum;
    }

    /// Returns a reference to the RPC header within the message element.
    pub(crate) fn rpc_header(&self) -> &RpcMessageHeader {
        // SAFETY: `RpcMessageHeader` is a transparent wrapper for the type of `inner.rpc`.
        unsafe { core::mem::transmute(&self.inner.rpc) }
    }

    /// Returns a mutable reference to the RPC header within the message element.
    pub(crate) fn rpc_header_mut(&mut self) -> &mut RpcMessageHeader {
        // SAFETY: `RpcMessageHeader` is a transparent wrapper for the type of `inner.rpc`.
        unsafe { core::mem::transmute(&mut self.inner.rpc) }
    }

    /// Returns the total length of the message, message and RPC headers included.
    pub(crate) fn msg_length(&self) -> usize {
        // Note: this is transport-layer code accessing message-layer data, which is only ok here
        // because RPC is the only kind of message we handle, and its header can (with a stretch) be
        // considered part of the transport header.
        size_of::<Self>() + self.rpc_header().rpc_length()
    }

    /// Returns the number of elements (i.e. memory pages) used by this message.
    pub(crate) fn element_count(&self) -> u32 {
        self.inner.elemCount
    }
}

// SAFETY: Padding is explicit and does not contain uninitialized data.
unsafe impl AsBytes for GspMsgElement {}

// SAFETY: This struct only contains integer types for which all bit patterns
// are valid.
unsafe impl FromBytes for GspMsgElement {}

/// RPC Message Header.
///
/// The header of a RPC message, wrapped inside a [`GspMsgElement`].
#[repr(transparent)]
pub(crate) struct RpcMessageHeader {
    inner: bindings::rpc_message_header_v,
}

impl RpcMessageHeader {
    /// Creates a new RPC header.
    ///
    /// `cmd_size` is the size in bytes of the payload. `function` is the RPC function of the
    /// message.
    pub(crate) fn init(cmd_size: usize, function: MsgFunction) -> impl Init<Self, Error> {
        let init_inner = try_init!(bindings::rpc_message_header_v {
            header_version: MsgHeaderVersion::new().into(),
            signature: bindings::NV_VGPU_MSG_SIGNATURE_VALID,
            function: function.into(),
            length: size_of::<Self>()
                .checked_add(cmd_size)
                .ok_or(EOVERFLOW)
                .and_then(|v| v.try_into().map_err(|_| EINVAL))?,
            rpc_result: 0xffffffff,
            rpc_result_private: 0xffffffff,
            ..Zeroable::init_zeroed()
        });

        try_init!(RpcMessageHeader {
            inner <- init_inner,
        })
    }

    /// Returns the length of the RPC's payload, not including the message and RPC headers.
    pub(crate) fn rpc_length(&self) -> usize {
        // `length` includes the length of the RPC message header.
        num::u32_as_usize(self.inner.length).saturating_sub(size_of::<Self>())
    }

    /// Returns the sequence number of the message.
    pub(crate) fn sequence(&self) -> u32 {
        self.inner.sequence
    }

    /// Returns the function of the message, if it is valid, or the invalid function number as an
    /// error.
    pub(crate) fn function(&self) -> Result<MsgFunction, u32> {
        self.inner
            .function
            .try_into()
            .map_err(|_| self.inner.function)
    }
}

bounded_enum! {
    /// Commands of the GMC API.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[repr(u32)]
    pub(crate) enum GmcCommandId with TryFrom<Bounded<u32, 24>> {
        GspInit = r000_00::GMCAPI_COMMANDS_GMCAPI_CMD_GSP_INIT,
        ExecGenericBootloader = r000_00::GMCAPI_COMMANDS_GMCAPI_CMD_EXEC_GENERIC_BOOTLOADER,
        ExecHsBinary = r000_00::GMCAPI_COMMANDS_GMCAPI_CMD_EXEC_HS_BINARY,
        GspSuspend = r000_00::GMCAPI_COMMANDS_GMCAPI_CMD_GSP_SUSPEND,
        MmuFaultQueued = r000_00::GMCAPI_COMMANDS_GMCAPI_CMD_MMU_FAULT_QUEUED,
    }
}

bitfield! {
    /// The `command` word of a [`GmcApiHeader`].
    pub(crate) struct GmcCommandWord(u32) {
        /// Command ID.
        23:0 id ?=> GmcCommandId;
        /// Set by GSP-RM on the response to a request. Not set for requests and events.
        24:24 is_response => bool;
    }
}

/// Header of a GMC (GPU Management Controller) API message.
///
/// In a queue element, it follows the queue element header and precedes the payload of the
/// command.
#[repr(transparent)]
pub(crate) struct GmcApiHeader {
    inner: r000_00::GMCAPI_HEADER,
}

// Using `allow` instead of `expect` here due to rust-lang/rust#154377.
#[allow(dead_code)]
impl GmcApiHeader {
    /// Creates the header of a request for `command`, sent with sequence number `sequence` and
    /// followed by `payload_size` bytes of payload.
    ///
    /// `max_reply_size` is the largest reply the sender accepts. It is zero for a request that does
    /// not expect a reply or expects a reply with an empty payload (i.e. just the status value).
    ///
    /// # Errors
    ///
    /// - `EOVERFLOW` if `payload_size` does not fit its 32-bit field.
    pub(crate) fn init(
        command: GmcCommandId,
        sequence: u64,
        payload_size: usize,
        max_reply_size: u32,
    ) -> impl Init<Self, Error> {
        let init_inner = try_init!(r000_00::GMCAPI_HEADER {
            command: GmcCommandWord::zeroed().with_id(command).into(),
            size: payload_size.try_into().map_err(|_| EOVERFLOW)?,
            sequence,
            __bindgen_anon_1: r000_00::GMCAPI_HEADER__bindgen_ty_1 {
                request: r000_00::GMCAPI_HEADER__bindgen_ty_1__bindgen_ty_1 {
                    max_response_size: max_reply_size,
                },
            },
            ..Zeroable::init_zeroed()
        });

        try_init!(GmcApiHeader {
            inner <- init_inner,
        })
    }

    /// Returns the `command` word of the header.
    pub(crate) fn command(&self) -> GmcCommandWord {
        GmcCommandWord::from(self.inner.command)
    }

    /// Returns the sequence number of the message.
    pub(crate) fn sequence(&self) -> u64 {
        self.inner.sequence
    }

    /// Returns the size in bytes of the payload following the header.
    pub(crate) fn payload_size(&self) -> usize {
        num::u32_as_usize(self.inner.size)
    }

    /// Returns the `NV_STATUS` carried by a response to a command.
    ///
    /// Returns `Some` with the `NV_STATUS` code if the `response` flag of [`Self::command`] is set;
    /// otherwise returns `None`.
    pub(crate) fn status(&self) -> Option<u32> {
        if self.command().is_response() {
            // SAFETY: we just checked that the message is a response.
            let status = unsafe { self.inner.__bindgen_anon_1.response.status };

            Some(status)
        } else {
            None
        }
    }
}

// SAFETY: This struct only contains integer types, including inside its union, and a zero-sized
// array. It has no padding.
unsafe impl AsBytes for GmcApiHeader {}

// SAFETY: This struct only contains integer types for which all bit patterns are valid.
unsafe impl FromBytes for GmcApiHeader {}

/// Arguments for GSP startup.
#[repr(transparent)]
#[derive(Zeroable)]
pub(crate) struct GspArgumentsCached {
    inner: bindings::GSP_ARGUMENTS_CACHED,
}

impl GspArgumentsCached {
    /// Creates the arguments for starting the GSP up using `cmdq` as its command queue.
    pub(crate) fn new<'a, 'b>(cmdq: &'a Cmdq<'b>) -> impl Init<Self> + use<'a, 'b> {
        let init_inner = init!(bindings::GSP_ARGUMENTS_CACHED {
            messageQueueInitArguments <- MessageQueueInitArguments::new(cmdq),
            bDmemStack: 1,
            ..Zeroable::init_zeroed()
        });

        init!(GspArgumentsCached {
            inner <- init_inner,
        })
    }
}

// SAFETY: Padding is explicit and will not contain uninitialized data.
unsafe impl AsBytes for GspArgumentsCached {}

/// On Turing and GA100, the entries in the `LibosMemoryRegionInitArgument`
/// must all be a multiple of GSP_PAGE_SIZE in size, so add padding to force it
/// to that size.
#[repr(C)]
#[derive(Zeroable)]
pub(crate) struct GspArgumentsPadded {
    pub(crate) inner: GspArgumentsCached,
    _padding: [u8; GSP_PAGE_SIZE - core::mem::size_of::<bindings::GSP_ARGUMENTS_CACHED>()],
}

impl GspArgumentsPadded {
    pub(crate) fn new<'a, 'b>(cmdq: &'a Cmdq<'b>) -> impl Init<Self> + use<'a, 'b> {
        init!(GspArgumentsPadded {
            inner <- GspArgumentsCached::new(cmdq),
            ..Zeroable::init_zeroed()
        })
    }
}

// SAFETY: Padding is explicit and will not contain uninitialized data.
unsafe impl AsBytes for GspArgumentsPadded {}

// SAFETY: This struct only contains integer types for which all bit patterns
// are valid.
unsafe impl FromBytes for GspArgumentsPadded {}

/// Init arguments for the message queue.
type MessageQueueInitArguments = bindings::MESSAGE_QUEUE_INIT_ARGUMENTS;

impl MessageQueueInitArguments {
    /// Creates a new init arguments structure for `cmdq`.
    fn new<'a, 'b>(cmdq: &'a Cmdq<'b>) -> impl Init<Self> + use<'a, 'b> {
        init!(MessageQueueInitArguments {
            sharedMemPhysAddr: cmdq.dma_addr,
            pageTableEntryCount: cv!(Cmdq::NUM_PTES),
            cmdQueueOffset: u64::from_safe_cast(Cmdq::CMDQ_OFFSET),
            statQueueOffset: u64::from_safe_cast(Cmdq::STATQ_OFFSET),
            ..Zeroable::init_zeroed()
        })
    }
}

#[repr(u32)]
pub(crate) enum GspDmaTarget {
    #[expect(dead_code)]
    LocalFb = bindings::GSP_DMA_TARGET_GSP_DMA_TARGET_LOCAL_FB,
    CoherentSystem = bindings::GSP_DMA_TARGET_GSP_DMA_TARGET_COHERENT_SYSTEM,
    NoncoherentSystem = bindings::GSP_DMA_TARGET_GSP_DMA_TARGET_NONCOHERENT_SYSTEM,
}

type GspAcrBootGspRmParams = bindings::GSP_ACR_BOOT_GSP_RM_PARAMS;

impl GspAcrBootGspRmParams {
    fn new(target: GspDmaTarget, wpr_meta_addr: u64) -> impl Init<Self> {
        let params = init!(Self {
            target: target as u32,
            gspRmDescSize: cv!(size_of::<GspFwWprMeta>()),
            gspRmDescOffset: wpr_meta_addr,
            bIsGspRmBoot: 1,
            wprCarveoutOffset: 0,
            wprCarveoutSize: 0,
            __bindgen_padding_0: Default::default(),
        });

        params
    }
}

type GspRmParams = bindings::GSP_RM_PARAMS;

impl GspRmParams {
    fn new(target: GspDmaTarget, libos_addr: u64) -> impl Init<Self> {
        let params = init!(Self {
            target: target as u32,
            bootArgsOffset: libos_addr,
            __bindgen_padding_0: Default::default(),
        });

        params
    }
}

pub(crate) type GspFmcBootParams = bindings::GSP_FMC_BOOT_PARAMS;

// SAFETY: Padding is explicit and will not contain uninitialized data.
unsafe impl AsBytes for GspFmcBootParams {}
// SAFETY: This struct only contains integer types for which all bit patterns are valid.
unsafe impl FromBytes for GspFmcBootParams {}

impl GspFmcBootParams {
    pub(crate) fn new(wpr_meta_addr: u64, libos_addr: u64) -> impl Init<Self> {
        let init = init!(Self {
            // Blackwell FSP obtains WPR info from other sources, so
            // wprCarveoutOffset and wprCarveoutSize are left zero.
            bootGspRmParams <- GspAcrBootGspRmParams::new(GspDmaTarget::CoherentSystem,
                wpr_meta_addr),
            gspRmParams <- GspRmParams::new(GspDmaTarget::NoncoherentSystem, libos_addr),
            ..Zeroable::init_zeroed()
        });

        init
    }
}
