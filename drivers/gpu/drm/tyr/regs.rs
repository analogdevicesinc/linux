// SPDX-License-Identifier: GPL-2.0 or MIT

//! # Definitions
//!
//! - **CEU**: Command Execution Unit - A hardware component that executes commands (instructions)
//!   from the command stream.
//! - **CS**: Command Stream - A sequence of instructions (commands) used to control a particular
//!   job or sequence of jobs. The instructions exist in one or more command buffers.
//! - **CSF**: Command Stream Frontend - The interface and implementation for job submission
//!   exposed to the host CPU driver. This includes the global interface, as well as CSG and CS
//!   interfaces.
//! - **CSG**: Command Stream Group - A group of related command streams. The CSF manages multiple
//!   CSGs, and each CSG contains multiple CSs.
//! - **CSHW**: Command Stream Hardware - The hardware interpreting command streams, including the
//!   iterator control aspects. Implements the CSF in conjunction with the MCU.
//! - **GLB**: Global - Prefix for global interface registers that control operations common to
//!   all CSs.
//! - **JASID**: Job Address Space ID - Identifies the address space for a job.
//! - **MCU**: Microcontroller Unit - Implements the CSF in conjunction with the command stream
//!   hardware.
//! - **MMU**: Memory Management Unit - Handles address translation and memory access protection.

// We don't expect that all the registers and fields will be used, even in the
// future.
//
// Nevertheless, it is useful to have most of them defined, like the C driver
// does.
#![allow(dead_code)]

use kernel::{
    device::{
        Bound,
        Device, //
    },
    devres::Devres,
    io::Io,
    prelude::*, //
};

use crate::driver::IoMem;

/// Represents a register in the Register Set
///
/// TODO: Replace this with the Nova `register!()` macro when it is available.
/// In particular, this will automatically give us 64bit register reads and
/// writes.
pub(crate) struct Register<const OFFSET: usize>;

impl<const OFFSET: usize> Register<OFFSET> {
    #[inline]
    pub(crate) fn read(&self, dev: &Device<Bound>, iomem: &Devres<IoMem>) -> Result<u32> {
        let value = (*iomem).access(dev)?.read32(OFFSET);
        Ok(value)
    }

    #[inline]
    pub(crate) fn write(&self, dev: &Device<Bound>, iomem: &Devres<IoMem>, value: u32) -> Result {
        (*iomem).access(dev)?.write32(value, OFFSET);
        Ok(())
    }
}

/// Combine two 32-bit values into a single 64-bit value.
pub(crate) fn join_u64(lo: u32, hi: u32) -> u64 {
    (u64::from(lo)) | ((u64::from(hi)) << 32)
}

/// Read a logical 64-bit value from split 32-bit registers without tearing.
pub(crate) fn read_u64_no_tearing(lo_read: impl Fn() -> u32, hi_read: impl Fn() -> u32) -> u64 {
    loop {
        let hi1 = hi_read();
        let lo = lo_read();
        let hi2 = hi_read();

        if hi1 == hi2 {
            return join_u64(lo, hi1);
        }
    }
}

/// These registers correspond to the GPU_CONTROL register page.
/// They are involved in GPU configuration and control.
pub(crate) mod gpu_control {
    use core::convert::TryFrom;
    use kernel::{
        error::{
            code::EINVAL,
            Error, //
        },
        num::Bounded,
        register,
        uapi, //
    };
    use pin_init::Zeroable;

    register! {
        /// GPU identification register.
        pub(crate) GPU_ID(u32) @ 0x0 {
            /// Status of the GPU release.
            3:0     ver_status;
            /// Minor release version number.
            11:4    ver_minor;
            /// Major release version number.
            15:12   ver_major;
            /// Product identifier.
            19:16   prod_major;
            /// Architecture patch revision.
            23:20   arch_rev;
            /// Architecture minor revision.
            27:24   arch_minor;
            /// Architecture major revision.
            31:28   arch_major;
        }

        /// Level 2 cache features register.
        pub(crate) L2_FEATURES(u32) @ 0x4 {
            /// Cache line size.
            7:0     line_size;
            /// Cache associativity.
            15:8    associativity;
            /// Cache slice size.
            23:16   cache_size;
            /// External bus width.
            31:24   bus_width;
        }

        /// Shader core features.
        pub(crate) CORE_FEATURES(u32) @ 0x8 {
            /// Shader core variant.
            7:0     core_variant;
        }

        /// Tiler features.
        pub(crate) TILER_FEATURES(u32) @ 0xc {
            /// Log of the tiler's bin size.
            5:0     bin_size;
            /// Maximum number of active levels.
            11:8    max_levels;
        }

        /// Memory system features.
        pub(crate) MEM_FEATURES(u32) @ 0x10 {
            0:0     coherent_core_group => bool;
            1:1     coherent_super_group => bool;
            11:8    l2_slices;
        }

        /// Memory management unit features.
        pub(crate) MMU_FEATURES(u32) @ 0x14 {
            /// Number of bits supported in virtual addresses.
            7:0     va_bits;
            /// Number of bits supported in physical addresses.
            15:8    pa_bits;
        }

        /// Address spaces present.
        pub(crate) AS_PRESENT(u32) @ 0x18 {
            31:0    present;
        }

        /// CSF version information.
        pub(crate) CSF_ID(u32) @ 0x1c {
            /// MCU revision ID.
            3:0     mcu_rev;
            /// MCU minor revision number.
            9:4     mcu_minor;
            /// MCU major revision number.
            15:10   mcu_major;
            /// CSHW revision ID.
            19:16   cshw_rev;
            /// CSHW minor revision number.
            25:20   cshw_minor;
            /// CSHW major revision number.
            31:26   cshw_major;
        }

        /// IRQ sources raw status.
        /// Writing to this register forces bits on, but does not clear them.
        pub(crate) GPU_IRQ_RAWSTAT(u32) @ 0x20 {
            /// A GPU fault has occurred, a 1-bit boolean flag.
            0:0     gpu_fault => bool;
            /// A GPU fault has occurred, a 1-bit boolean flag.
            1:1     gpu_protected_fault => bool;
            /// Reset has completed, a 1-bit boolean flag.
            8:8     reset_completed => bool;
            /// Set when a single power domain has powered up or down, a 1-bit boolean flag.
            9:9     power_changed_single => bool;
            /// Set when the all pending power domain changes are completed, a 1-bit boolean flag.
            10:10   power_changed_all => bool;
            /// Set when cache cleaning has completed, a 1-bit boolean flag.
            17:17   clean_caches_completed => bool;
            /// Mirrors the doorbell interrupt line to the CPU, a 1-bit boolean flag.
            18:18   doorbell_mirror => bool;
            /// MCU requires attention, a 1-bit boolean flag.
            19:19   mcu_status => bool;
        }

        /// IRQ sources to clear. Write only.
        pub(crate) GPU_IRQ_CLEAR(u32) @ 0x24 {
            /// Clear the GPU_FAULT interrupt, a 1-bit boolean flag.
            0:0     gpu_fault => bool;
            /// Clear the GPU_PROTECTED_FAULT interrupt, a 1-bit boolean flag.
            1:1     gpu_protected_fault => bool;
            /// Clear the RESET_COMPLETED interrupt, a 1-bit boolean flag.
            8:8     reset_completed => bool;
            /// Clear the POWER_CHANGED_SINGLE interrupt, a 1-bit boolean flag.
            9:9     power_changed_single => bool;
            /// Clear the POWER_CHANGED_ALL interrupt, a 1-bit boolean flag.
            10:10   power_changed_all => bool;
            /// Clear the CLEAN_CACHES_COMPLETED interrupt, a 1-bit boolean flag.
            17:17   clean_caches_completed => bool;
            /// Clear the MCU_STATUS interrupt, a 1-bit boolean flag.
            19:19   mcu_status => bool;
        }

        /// IRQ sources enabled.
        pub(crate) GPU_IRQ_MASK(u32) @ 0x28 {
            /// Enable the GPU_FAULT interrupt, a 1-bit boolean flag.
            0:0     gpu_fault => bool;
            /// Enable the GPU_PROTECTED_FAULT interrupt, a 1-bit boolean flag.
            1:1     gpu_protected_fault => bool;
            /// Enable the RESET_COMPLETED interrupt, a 1-bit boolean flag.
            8:8     reset_completed => bool;
            /// Enable the POWER_CHANGED_SINGLE interrupt, a 1-bit boolean flag.
            9:9     power_changed_single => bool;
            /// Enable the POWER_CHANGED_ALL interrupt, a 1-bit boolean flag.
            10:10   power_changed_all => bool;
            /// Enable the CLEAN_CACHES_COMPLETED interrupt, a 1-bit boolean flag.
            17:17   clean_caches_completed => bool;
            /// Enable the DOORBELL_MIRROR interrupt, a 1-bit boolean flag.
            18:18   doorbell_mirror => bool;
            /// Enable the MCU_STATUS interrupt, a 1-bit boolean flag.
            19:19   mcu_status => bool;
        }

        /// IRQ status for enabled sources. Read only.
        pub(crate) GPU_IRQ_STATUS(u32) @ 0x2c {
            /// GPU_FAULT interrupt status, a 1-bit boolean flag.
            0:0     gpu_fault => bool;
            /// GPU_PROTECTED_FAULT interrupt status, a 1-bit boolean flag.
            1:1     gpu_protected_fault => bool;
            /// RESET_COMPLETED interrupt status, a 1-bit boolean flag.
            8:8     reset_completed => bool;
            /// POWER_CHANGED_SINGLE interrupt status, a 1-bit boolean flag.
            9:9     power_changed_single => bool;
            /// POWER_CHANGED_ALL interrupt status, a 1-bit boolean flag.
            10:10   power_changed_all => bool;
            /// CLEAN_CACHES_COMPLETED interrupt status, a 1-bit boolean flag.
            17:17   clean_caches_completed => bool;
            /// DOORBELL_MIRROR interrupt status, a 1-bit boolean flag.
            18:18   doorbell_mirror => bool;
            /// MCU_STATUS interrupt status, a 1-bit boolean flag.
            19:19   mcu_status => bool;
        }
    }

    /// Helpers for GPU_COMMAND Register
    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub(crate) enum GpuCommand {
        /// No operation. This is the default value.
        Nop = 0,
        /// Reset the GPU.
        Reset = 1,
        /// Flush caches.
        FlushCaches = 4,
        /// Clear GPU faults.
        ClearFault = 7,
    }

    impl TryFrom<Bounded<u32, 8>> for GpuCommand {
        type Error = Error;

        fn try_from(val: Bounded<u32, 8>) -> Result<Self, Self::Error> {
            match val.get() {
                0 => Ok(GpuCommand::Nop),
                1 => Ok(GpuCommand::Reset),
                4 => Ok(GpuCommand::FlushCaches),
                7 => Ok(GpuCommand::ClearFault),
                _ => Err(EINVAL),
            }
        }
    }

    impl From<GpuCommand> for Bounded<u32, 8> {
        fn from(cmd: GpuCommand) -> Self {
            (cmd as u8).into()
        }
    }

    /// Reset mode for [`GPU_COMMAND::reset()`].
    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub(crate) enum ResetMode {
        /// Stop all external bus interfaces, then reset the entire GPU.
        SoftReset = 1,
        /// Force a full GPU reset.
        HardReset = 2,
    }

    impl TryFrom<Bounded<u32, 4>> for ResetMode {
        type Error = Error;

        fn try_from(val: Bounded<u32, 4>) -> Result<Self, Self::Error> {
            match val.get() {
                1 => Ok(ResetMode::SoftReset),
                2 => Ok(ResetMode::HardReset),
                _ => Err(EINVAL),
            }
        }
    }

    impl From<ResetMode> for Bounded<u32, 4> {
        fn from(mode: ResetMode) -> Self {
            Bounded::try_new(mode as u32).unwrap()
        }
    }

    /// Cache flush mode for [`GPU_COMMAND::flush_caches()`].
    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub(crate) enum FlushMode {
        /// No flush.
        None = 0,
        /// Clean the caches.
        Clean = 1,
        /// Invalidate the caches.
        Invalidate = 2,
        /// Clean and invalidate the caches.
        CleanInvalidate = 3,
    }

    impl TryFrom<Bounded<u32, 4>> for FlushMode {
        type Error = Error;

        fn try_from(val: Bounded<u32, 4>) -> Result<Self, Self::Error> {
            match val.get() {
                0 => Ok(FlushMode::None),
                1 => Ok(FlushMode::Clean),
                2 => Ok(FlushMode::Invalidate),
                3 => Ok(FlushMode::CleanInvalidate),
                _ => Err(EINVAL),
            }
        }
    }

    impl From<FlushMode> for Bounded<u32, 4> {
        fn from(mode: FlushMode) -> Self {
            Bounded::try_new(mode as u32).unwrap()
        }
    }

    register! {
        /// GPU command register.
        ///
        /// Use the constructor methods to create commands:
        /// - [`GPU_COMMAND::nop()`]
        /// - [`GPU_COMMAND::reset()`]
        /// - [`GPU_COMMAND::flush_caches()`]
        /// - [`GPU_COMMAND::clear_fault()`]
        pub(crate) GPU_COMMAND (u32) @ 0x30 {
            7:0     command ?=> GpuCommand;
        }
        /// Internal alias for GPU_COMMAND in reset mode.
        /// Use [`GPU_COMMAND::reset()`] instead.
        GPU_COMMAND_RESET (u32) => GPU_COMMAND {
            7:0     command ?=> GpuCommand;
            11:8    reset_mode ?=> ResetMode;
        }

        /// Internal alias for GPU_COMMAND in cache flush mode.
        /// Use [`GPU_COMMAND::flush_caches()`] instead.
        GPU_COMMAND_FLUSH (u32) => GPU_COMMAND {
            7:0     command ?=> GpuCommand;
            /// L2 cache flush mode.
            11:8    l2_flush ?=> FlushMode;
            /// Shader core load/store cache flush mode.
            15:12   lsc_flush ?=> FlushMode;
            /// Shader core other caches flush mode.
            19:16   other_flush ?=> FlushMode;
        }
    }

    impl GPU_COMMAND {
        /// Create a NOP command.
        pub(crate) fn nop() -> Self {
            Self::zeroed()
        }

        /// Create a reset command with the specified reset mode.
        pub(crate) fn reset(mode: ResetMode) -> Self {
            Self::from_raw(
                GPU_COMMAND_RESET::zeroed()
                    .with_command(GpuCommand::Reset)
                    .with_reset_mode(mode)
                    .into_raw(),
            )
        }

        /// Create a cache flush command with the specified flush modes.
        pub(crate) fn flush_caches(l2: FlushMode, lsc: FlushMode, other: FlushMode) -> Self {
            Self::from_raw(
                GPU_COMMAND_FLUSH::zeroed()
                    .with_command(GpuCommand::FlushCaches)
                    .with_l2_flush(l2)
                    .with_lsc_flush(lsc)
                    .with_other_flush(other)
                    .into_raw(),
            )
        }

        /// Create a clear fault command.
        pub(crate) fn clear_fault() -> Self {
            Self::zeroed().with_command(GpuCommand::ClearFault)
        }
    }

    register! {
        /// GPU status register. Read only.
        pub(crate) GPU_STATUS(u32) @ 0x34 {
            /// GPU active, a 1-bit boolean flag.
            0:0     gpu_active => bool;
            /// Power manager active, a 1-bit boolean flag
            1:1     pwr_active => bool;
            /// Page fault active, a 1-bit boolean flag.
            4:4     page_fault => bool;
            /// Protected mode active, a 1-bit boolean flag.
            7:7     protected_mode_active => bool;
            /// Debug mode active, a 1-bit boolean flag.
            8:8     gpu_dbg_enabled => bool;
        }
    }

    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub(crate) enum ExceptionType {
        /// Exception type: No error.
        Ok = 0x00,
        /// Exception type: GPU external bus error.
        GpuBusFault = 0x80,
        /// Exception type: GPU shareability error.
        GpuShareabilityFault = 0x88,
        /// Exception type: System shareability error.
        SystemShareabilityFault = 0x89,
        /// Exception type: GPU cacheability error.
        GpuCacheabilityFault = 0x8A,
    }

    impl TryFrom<Bounded<u32, 8>> for ExceptionType {
        type Error = Error;

        fn try_from(val: Bounded<u32, 8>) -> Result<Self, Self::Error> {
            match val.get() {
                0x00 => Ok(ExceptionType::Ok),
                0x80 => Ok(ExceptionType::GpuBusFault),
                0x88 => Ok(ExceptionType::GpuShareabilityFault),
                0x89 => Ok(ExceptionType::SystemShareabilityFault),
                0x8A => Ok(ExceptionType::GpuCacheabilityFault),
                _ => Err(EINVAL),
            }
        }
    }

    impl From<ExceptionType> for Bounded<u32, 8> {
        fn from(exc: ExceptionType) -> Self {
            (exc as u8).into()
        }
    }

    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub(crate) enum AccessType {
        /// Access type: An atomic (read/write) transaction.
        Atomic = 0,
        /// Access type: An execute transaction.
        Execute = 1,
        /// Access type: A read transaction.
        Read = 2,
        /// Access type: A write transaction.
        Write = 3,
    }

    impl From<Bounded<u32, 2>> for AccessType {
        fn from(val: Bounded<u32, 2>) -> Self {
            match val.get() {
                0 => AccessType::Atomic,
                1 => AccessType::Execute,
                2 => AccessType::Read,
                3 => AccessType::Write,
                _ => unreachable!(),
            }
        }
    }

    impl From<AccessType> for Bounded<u32, 2> {
        fn from(access: AccessType) -> Self {
            Bounded::try_new(access as u32).unwrap()
        }
    }

    register! {
        /// GPU fault status register. Read only.
        pub(crate) GPU_FAULTSTATUS(u32) @ 0x3c {
            /// Exception type.
            7:0     exception_type ?=> ExceptionType;
            /// Access type.
            9:8     access_type => AccessType;
            /// The GPU_FAULTADDRESS is valid, a 1-bit boolean flag.
            10:10   address_valid => bool;
            /// The JASID field is valid, a 1-bit boolean flag.
            11:11   jasid_valid => bool;
            /// JASID of the fault, if known.
            15:12   jasid;
            /// ID of the source that triggered the fault.
            31:16   source_id;
        }

        /// GPU fault address. Read only.
        /// Once a fault is reported, it must be manually cleared by issuing a
        /// [`GPU_COMMAND::clear_fault()`] command to the [`GPU_COMMAND`] register. No further GPU
        /// faults will be reported until the previous fault has been cleared.
        pub(crate) GPU_FAULTADDRESS_LO(u32) @ 0x40 {
            31:0    pointer;
        }

        pub(crate) GPU_FAULTADDRESS_HI(u32) @ 0x44 {
            31:0    pointer;
        }

        /// Level 2 cache configuration.
        pub(crate) L2_CONFIG(u32) @ 0x48 {
            /// Requested cache size.
            23:16   cache_size;
            /// Requested hash function index.
            31:24   hash_function;
        }

        /// Global time stamp offset.
        pub(crate) TIMESTAMP_OFFSET_LO(u32) @ 0x88 {
            31:0    offset;
        }

        pub(crate) TIMESTAMP_OFFSET_HI(u32) @ 0x8c {
            31:0    offset;
        }

        /// GPU cycle counter. Read only.
        pub(crate) CYCLE_COUNT_LO(u32) @ 0x90 {
            31:0    count;
        }

        pub(crate) CYCLE_COUNT_HI(u32) @ 0x94 {
            31:0    count;
        }

        /// Global time stamp. Read only.
        pub(crate) TIMESTAMP_LO(u32) @ 0x98 {
            31:0    timestamp;
        }

        pub(crate) TIMESTAMP_HI(u32) @ 0x9c {
            31:0    timestamp;
        }

        /// Maximum number of threads per core. Read only constant.
        pub(crate) THREAD_MAX_THREADS(u32) @ 0xa0 {
            31:0    threads;
        }

        /// Maximum number of threads per workgroup. Read only constant.
        pub(crate) THREAD_MAX_WORKGROUP_SIZE(u32) @ 0xa4 {
            31:0    threads;
        }

        /// Maximum number of threads per barrier. Read only constant.
        pub(crate) THREAD_MAX_BARRIER_SIZE(u32) @ 0xa8 {
            31:0    threads;
        }

        /// Thread features. Read only constant.
        pub(crate) THREAD_FEATURES(u32) @ 0xac {
            /// Total number of registers per core.
            21:0    max_registers;
            /// Implementation technology type.
            23:22   implementation_technology;
            /// Maximum number of compute tasks waiting.
            31:24   max_task_queue;
        }

        /// Support flags for compressed texture formats. Read only constant.
        ///
        /// A bitmap where each bit indicates support for a specific compressed texture format.
        /// The bit position maps to an opaque format ID (`texture_features_key_t` in spec).
        pub(crate) TEXTURE_FEATURES(u32)[4] @ 0xb0 {
            31:0    supported_formats;
        }

        /// Shader core present bitmap. Read only constant.
        pub(crate) SHADER_PRESENT_LO(u32) @ 0x100 {
            31:0    value;
        }

        pub(crate) SHADER_PRESENT_HI(u32) @ 0x104 {
            31:0    value;
        }

        /// Tiler present bitmap. Read only constant.
        pub(crate) TILER_PRESENT_LO(u32) @ 0x110 {
            31:0    present;
        }

        pub(crate) TILER_PRESENT_HI(u32) @ 0x114 {
            31:0    present;
        }

        /// L2 cache present bitmap. Read only constant.
        pub(crate) L2_PRESENT_LO(u32) @ 0x120 {
            31:0    present;
        }

        pub(crate) L2_PRESENT_HI(u32) @ 0x124 {
            31:0    present;
        }

        /// Shader core ready bitmap. Read only.
        pub(crate) SHADER_READY_LO(u32) @ 0x140 {
            31:0    ready;
        }

        pub(crate) SHADER_READY_HI(u32) @ 0x144 {
            31:0    ready;
        }

        /// Tiler ready bitmap. Read only.
        pub(crate) TILER_READY_LO(u32) @ 0x150 {
            31:0    ready;
        }

        pub(crate) TILER_READY_HI(u32) @ 0x154 {
            31:0    ready;
        }

        /// L2 ready bitmap. Read only.
        pub(crate) L2_READY_LO(u32) @ 0x160 {
            31:0    ready;
        }

        pub(crate) L2_READY_HI(u32) @ 0x164 {
            31:0    ready;
        }

        /// Shader core power up bitmap.
        pub(crate) SHADER_PWRON_LO(u32) @ 0x180 {
            31:0    request;
        }

        pub(crate) SHADER_PWRON_HI(u32) @ 0x184 {
            31:0    request;
        }

        /// Tiler power up bitmap.
        pub(crate) TILER_PWRON_LO(u32) @ 0x190 {
            31:0    request;
        }

        pub(crate) TILER_PWRON_HI(u32) @ 0x194 {
            31:0    request;
        }

        /// L2 power up bitmap.
        pub(crate) L2_PWRON_LO(u32) @ 0x1a0 {
            31:0    request;
        }

        pub(crate) L2_PWRON_HI(u32) @ 0x1a4 {
            31:0    request;
        }

        /// Shader core power down bitmap.
        pub(crate) SHADER_PWROFF_LO(u32) @ 0x1c0 {
            31:0    request;
        }

        pub(crate) SHADER_PWROFF_HI(u32) @ 0x1c4 {
            31:0    request;
        }

        /// Tiler power down bitmap.
        pub(crate) TILER_PWROFF_LO(u32) @ 0x1d0 {
            31:0    request;
        }

        pub(crate) TILER_PWROFF_HI(u32) @ 0x1d4 {
            31:0    request;
        }

        /// L2 power down bitmap.
        pub(crate) L2_PWROFF_LO(u32) @ 0x1e0 {
            31:0    request;
        }

        pub(crate) L2_PWROFF_HI(u32) @ 0x1e4 {
            31:0    request;
        }

        /// Shader core power transition bitmap. Read-only.
        pub(crate) SHADER_PWRTRANS_LO(u32) @ 0x200 {
            31:0    changing;
        }

        pub(crate) SHADER_PWRTRANS_HI(u32) @ 0x204 {
            31:0    changing;
        }

        /// Tiler power transition bitmap. Read-only.
        pub(crate) TILER_PWRTRANS_LO(u32) @ 0x210 {
            31:0    changing;
        }

        pub(crate) TILER_PWRTRANS_HI(u32) @ 0x214 {
            31:0    changing;
        }

        /// L2 power transition bitmap. Read-only.
        pub(crate) L2_PWRTRANS_LO(u32) @ 0x220 {
            31:0    changing;
        }

        pub(crate) L2_PWRTRANS_HI(u32) @ 0x224 {
            31:0    changing;
        }

        /// Shader core active bitmap. Read-only.
        pub(crate) SHADER_PWRACTIVE_LO(u32) @ 0x240 {
            31:0    active;
        }

        pub(crate) SHADER_PWRACTIVE_HI(u32) @ 0x244 {
            31:0    active;
        }

        /// Tiler active bitmap. Read-only.
        pub(crate) TILER_PWRACTIVE_LO(u32) @ 0x250 {
            31:0    active;
        }

        pub(crate) TILER_PWRACTIVE_HI(u32) @ 0x254 {
            31:0    active;
        }

        /// L2 active bitmap.  Read-only.
        pub(crate) L2_PWRACTIVE_LO(u32) @ 0x260 {
            31:0    active;
        }

        pub(crate) L2_PWRACTIVE_HI(u32) @ 0x264 {
            31:0    active;
        }

        /// Revision ID. Read only constant.
        pub(crate) REVIDR(u32) @ 0x280 {
            31:0    revision;
        }

        /// Coherency features present. Read only constant.
        /// Supported protocols on the interconnect between the GPU and the
        /// system into which it is integrated.
        pub(crate) COHERENCY_FEATURES(u32) @ 0x300 {
            /// ACE-Lite protocol supported, a 1-bit boolean flag.
            0:0     ace_lite => bool;
            /// ACE protocol supported, a 1-bit boolean flag.
            1:1     ace => bool;
        }
    }

    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub(crate) enum CoherencyMode {
        /// ACE-Lite coherency protocol.
        AceLite = uapi::drm_panthor_gpu_coherency_DRM_PANTHOR_GPU_COHERENCY_ACE_LITE as u8,
        /// ACE coherency protocol.
        Ace = uapi::drm_panthor_gpu_coherency_DRM_PANTHOR_GPU_COHERENCY_ACE as u8,
        /// No coherency protocol.
        None = uapi::drm_panthor_gpu_coherency_DRM_PANTHOR_GPU_COHERENCY_NONE as u8,
    }

    impl TryFrom<Bounded<u32, 32>> for CoherencyMode {
        type Error = Error;

        fn try_from(val: Bounded<u32, 32>) -> Result<Self, Self::Error> {
            match val.get() {
                0 => Ok(CoherencyMode::AceLite),
                1 => Ok(CoherencyMode::Ace),
                31 => Ok(CoherencyMode::None),
                _ => Err(EINVAL),
            }
        }
    }

    impl From<CoherencyMode> for Bounded<u32, 32> {
        fn from(mode: CoherencyMode) -> Self {
            (mode as u8).into()
        }
    }

    register! {
        /// Coherency enable. An index of which coherency protocols should be used.
        /// This register only selects the protocol for coherency messages on the
        /// interconnect. This is not to enable or disable coherency controlled by MMU.
        pub(crate) COHERENCY_ENABLE(u32) @ 0x304 {
            31:0    l2_cache_protocol_select ?=> CoherencyMode;
        }
    }

    /// Helpers for MCU_CONTROL register
    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub(crate) enum McuControlMode {
        /// Disable the MCU.
        Disable = 0,
        /// Enable the MCU.
        Enable = 1,
        /// Enable the MCU to execute and automatically reboot after a fast reset.
        Auto = 2,
    }

    impl TryFrom<Bounded<u32, 2>> for McuControlMode {
        type Error = Error;

        fn try_from(val: Bounded<u32, 2>) -> Result<Self, Self::Error> {
            match val.get() {
                0 => Ok(McuControlMode::Disable),
                1 => Ok(McuControlMode::Enable),
                2 => Ok(McuControlMode::Auto),
                _ => Err(EINVAL),
            }
        }
    }

    impl From<McuControlMode> for Bounded<u32, 2> {
        fn from(mode: McuControlMode) -> Self {
            Bounded::try_new(mode as u32).unwrap()
        }
    }

    register! {
        /// MCU control.
        pub(crate) MCU_CONTROL(u32) @ 0x700 {
            /// Request MCU state change.
            1:0 req ?=> McuControlMode;
        }
    }

    /// Helpers for MCU_STATUS register
    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub(crate) enum McuStatus {
        /// MCU is disabled.
        Disabled = 0,
        /// MCU is enabled.
        Enabled = 1,
        /// The MCU has halted by itself in an orderly manner to enable the core group to be
        /// powered down.
        Halt = 2,
        /// The MCU has encountered an error that prevents it from continuing.
        Fatal = 3,
    }

    impl From<Bounded<u32, 2>> for McuStatus {
        fn from(val: Bounded<u32, 2>) -> Self {
            match val.get() {
                0 => McuStatus::Disabled,
                1 => McuStatus::Enabled,
                2 => McuStatus::Halt,
                3 => McuStatus::Fatal,
                _ => unreachable!(),
            }
        }
    }

    impl From<McuStatus> for Bounded<u32, 2> {
        fn from(status: McuStatus) -> Self {
            Bounded::try_new(status as u32).unwrap()
        }
    }

    register! {
        /// MCU status. Read only.
        pub(crate) MCU_STATUS(u32) @ 0x704 {
            /// Read current state of MCU.
            1:0 value => McuStatus;
        }
    }
}

pub(crate) const MMU_IRQ_RAWSTAT: Register<0x2000> = Register;
pub(crate) const MMU_IRQ_CLEAR: Register<0x2004> = Register;
pub(crate) const MMU_IRQ_MASK: Register<0x2008> = Register;
pub(crate) const MMU_IRQ_STAT: Register<0x200c> = Register;

/// These registers correspond to the JOB_CONTROL register page.
/// They are involved in communication between the firmware running on the MCU and the host.
pub(crate) mod job_control {
    use kernel::register;

    register! {
        /// Raw status of job interrupts.
        ///
        /// Write to this register to trigger these interrupts.
        /// Writing a 1 to a bit forces that bit on.
        pub(crate) JOB_IRQ_RAWSTAT(u32) @ 0x1000 {
            /// CSG request. These bits indicate that CSGn requires attention from the host.
            30:0    csg;
            /// GLB request. Indicates that the GLB interface requires attention from the host.
            31:31   glb => bool;
        }

        /// Clear job interrupts. Write only.
        ///
        /// Write a 1 to a bit to clear the corresponding bit in [`JOB_IRQ_RAWSTAT`].
        pub(crate) JOB_IRQ_CLEAR(u32) @ 0x1004 {
            /// Clear CSG request interrupts.
            30:0    csg;
            /// Clear GLB request interrupt.
            31:31   glb => bool;
        }

        /// Mask for job interrupts.
        ///
        /// Set each bit to 1 to enable the corresponding interrupt source or to 0 to disable it.
        pub(crate) JOB_IRQ_MASK(u32) @ 0x1008 {
            /// Enable CSG request interrupts.
            30:0    csg;
            /// Enable GLB request interrupt.
            31:31   glb => bool;
        }

        /// Active job interrupts. Read only.
        ///
        /// This register contains the result of ANDing together [`JOB_IRQ_RAWSTAT`] and
        /// [`JOB_IRQ_MASK`].
        pub(crate) JOB_IRQ_STATUS(u32) @ 0x100c {
            /// CSG request interrupt status.
            30:0    csg;
            /// GLB request interrupt status.
            31:31   glb => bool;
        }
    }
}
