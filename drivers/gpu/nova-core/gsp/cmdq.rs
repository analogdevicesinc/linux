// SPDX-License-Identifier: GPL-2.0
// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

pub(super) mod rpc;

use core::mem;

use kernel::{
    device,
    dma::{
        Coherent,
        CoherentBox,
        DmaAddress, //
    },
    io::{
        io_project,
        poll::read_poll_timeout,
        Io, //
    },
    new_mutex,
    prelude::*,
    ptr,
    sync::{
        barrier::{
            dma_mb,
            Full,
            Read,
            Write, //
        },
        Mutex, //
    },
    time::{
        Delta,
        Instant,
        Monotonic, //
    },
    transmute::{
        AsBytes,
        FromBytes, //
    },
};

use pin_init::pin_init_scope;

use crate::{
    driver::Bar0,
    gsp::{
        fw::{
            GspMsgElement,
            MsgqRxHeader,
            MsgqTxHeader,
            GSP_MSG_QUEUE_ELEMENT_SIZE_MAX, //
        },
        PteArray,
        GSP_PAGE_SHIFT,
        GSP_PAGE_SIZE, //
    },
    num,
    sbuffer::SBufferIter, //
};

use super::regs;

/// Marker type representing the absence of a reply for a command. Commands using this as their
/// reply type are sent using [`Cmdq::send_rpc_command_no_wait`].
pub(crate) struct NoReply;

/// Trait implemented by types that can be sent as a single command queue element.
///
/// The command queue allocates `size()` bytes after the `GspMsgElement` header and calls `write()`
/// to fill them.
trait CommandElement {
    /// Size in bytes of the element, not including the `GspMsgElement` header.
    fn size(&self) -> usize;

    /// Writes the contents of the command into `dst`. `dev` is the queue's device (to be used for
    /// logging), `seq` is the sequence number of the element.
    fn write(&self, dev: &device::Device, seq: u32, dst: &mut GspCommand<'_>) -> Result;
}

/// Number of GSP pages making the [`Msgq`].
pub(crate) const MSGQ_NUM_PAGES: u32 = 0x3f;

/// Circular buffer of a [`Msgq`].
///
/// This area of memory is to be shared between the driver and the GSP to exchange commands or
/// messages.
#[repr(C, align(0x1000))]
#[derive(Debug)]
struct MsgqData {
    data: [[u8; GSP_PAGE_SIZE]; cv!(MSGQ_NUM_PAGES)],
}

// Annoyingly we are forced to use a literal to specify the alignment of
// `MsgqData`, so check that it corresponds to the actual GSP page size here.
static_assert!(align_of::<MsgqData>() == GSP_PAGE_SIZE);

/// Unidirectional message queue.
///
/// Contains the data for a message queue, that either the driver or GSP writes to.
///
/// Note that while the write pointer of `tx` corresponds to the `msgq` of the same instance, the
/// read pointer of `rx` actually refers to the `Msgq` owned by the other side.
/// This design ensures that only the driver or GSP ever writes to a given instance of this struct.
#[repr(C)]
// There is no struct defined for this in the open-gpu-kernel-source headers.
// Instead it is defined by code in `GspMsgQueuesInit()`.
struct Msgq {
    /// Header for sending messages, including the write pointer.
    tx: MsgqTxHeader,
    /// Header for receiving messages, including the read pointer.
    rx: MsgqRxHeader,
    /// The message queue proper.
    msgq: MsgqData,
}

/// Structure shared between the driver and the GSP and containing the command and message queues.
#[repr(C)]
struct GspMem {
    /// Self-mapping page table entries.
    ptes: PteArray<{ Self::PTE_ARRAY_SIZE }>,
    /// CPU queue: the driver writes commands here, and the GSP reads them. It also contains the
    /// write and read pointers that the CPU updates. This means that the read pointer here is an
    /// index into the GSP queue.
    ///
    /// This member is read-only for the GSP.
    cpuq: Msgq,
    /// GSP queue: the GSP writes messages here, and the driver reads them. It also contains the
    /// write and read pointers that the GSP updates. This means that the read pointer here is an
    /// index into the CPU queue.
    ///
    /// This member is read-only for the driver.
    gspq: Msgq,
}

impl GspMem {
    const PTE_ARRAY_SIZE: usize = GSP_PAGE_SIZE / size_of::<u64>();
}

// SAFETY: These structs don't meet the no-padding requirements of AsBytes but
// that is not a problem because they are not used outside the kernel.
unsafe impl AsBytes for GspMem {}

// SAFETY: These structs don't meet the no-padding requirements of FromBytes but
// that is not a problem because they are not used outside the kernel.
unsafe impl FromBytes for GspMem {}

/// Wrapper around [`GspMem`] to share it with the GPU using a [`Coherent`].
///
/// This provides the low-level functionality to communicate with the GSP, including allocation of
/// queue space to write messages to and management of read/write pointers.
///
/// This is shared with the GSP, with clear ownership rules regarding the command queues:
///
/// * The driver owns (i.e. can write to) the part of the CPU message queue between the CPU write
///   pointer and the GSP read pointer. This region is returned by [`Self::driver_write_area`].
/// * The driver owns (i.e. can read from) the part of the GSP message queue between the CPU read
///   pointer and the GSP write pointer. This region is returned by [`Self::driver_read_area`].
struct DmaGspMem<'a>(Coherent<'a, GspMem>);

impl<'a> DmaGspMem<'a> {
    /// Allocate a new instance and map it for `dev`.
    fn new(dev: &'a device::Device<device::Bound>) -> Result<Self> {
        const MSGQ_SIZE: u32 = cv!(size_of::<Msgq>());
        const RX_HDR_OFF: u32 = cv!(mem::offset_of!(Msgq, rx));

        let mut gsp_mem = CoherentBox::<'_, GspMem>::zeroed(dev, GFP_KERNEL)?;
        gsp_mem.cpuq.tx = MsgqTxHeader::new(MSGQ_SIZE, RX_HDR_OFF, MSGQ_NUM_PAGES);
        gsp_mem.cpuq.rx = MsgqRxHeader::new();

        let gsp_mem: Coherent<'_, _> = gsp_mem.into();
        PteArray::init(io_project!(gsp_mem, .ptes), gsp_mem.dma_address())?;

        Ok(Self(gsp_mem))
    }

    /// Returns the region of the CPU message queue that the driver is currently allowed to write
    /// to.
    ///
    /// As the message queue is a circular buffer, the region may be discontiguous in memory. In
    /// that case the second slice will have a non-zero length.
    fn driver_write_area(&mut self) -> (&mut [[u8; GSP_PAGE_SIZE]], &mut [[u8; GSP_PAGE_SIZE]]) {
        let tx = self.cpu_write_ptr();
        let rx = self.gsp_read_ptr();

        // Pointer to the first entry of the CPU message queue.
        let data = ptr::project!(mut self.0.as_mut_ptr(), .cpuq.msgq.data[build: 0]);

        let (tail_end, wrap_end) = if rx == 0 {
            // The write area is non-wrapping, and stops at the second-to-last entry of the command
            // queue (to leave the last one empty).
            (MSGQ_NUM_PAGES - 1, 0)
        } else if rx <= tx {
            // The write area wraps and continues until `rx - 1`.
            (MSGQ_NUM_PAGES, rx - 1)
        } else {
            // The write area doesn't wrap and stops at `rx - 1`.
            (rx - 1, 0)
        };

        // SAFETY:
        // - `data` was created from a valid pointer, and `rx` and `tx` are in the
        //   `0..MSGQ_NUM_PAGES` range per the invariants of `cpu_write_ptr` and `gsp_read_ptr`,
        //   thus the created slices are valid.
        // - The area starting at `tx` and ending at `rx - 2` modulo `MSGQ_NUM_PAGES`,
        //   inclusive, belongs to the driver for writing and is not accessed concurrently by
        //   the GSP.
        // - The caller holds a reference to `self` for as long as the returned slices are live,
        //   meaning the CPU write pointer cannot be advanced and thus that the returned area
        //   remains exclusive to the CPU for the duration of the slices.
        // - The created slices point to non-overlapping sub-ranges of `data` in all
        //   branches (in the `rx <= tx` case, the second slice ends at `rx - 1` which is strictly
        //   less than `tx` where the first slice starts; in the other cases the second slice is
        //   empty), so creating two `&mut` references from them does not violate aliasing rules.
        unsafe {
            (
                core::slice::from_raw_parts_mut(
                    data.add(num::u32_as_usize(tx)),
                    num::u32_as_usize(tail_end - tx),
                ),
                core::slice::from_raw_parts_mut(data, num::u32_as_usize(wrap_end)),
            )
        }
    }

    /// Returns the size of the region of the CPU message queue that the driver is currently allowed
    /// to write to, in bytes.
    fn driver_write_area_size(&self) -> usize {
        let tx = self.cpu_write_ptr();
        let rx = self.gsp_read_ptr();

        // `rx` and `tx` are both in `0..MSGQ_NUM_PAGES` per the invariants of `gsp_read_ptr` and
        // `cpu_write_ptr`. The minimum value case is where `rx == 0` and `tx == MSGQ_NUM_PAGES -
        // 1`, which gives `0 + MSGQ_NUM_PAGES - (MSGQ_NUM_PAGES - 1) - 1 == 0`.
        let slots = (rx + MSGQ_NUM_PAGES - tx - 1) % MSGQ_NUM_PAGES;
        num::u32_as_usize(slots) * GSP_PAGE_SIZE
    }

    /// Returns the region of the GSP message queue that the driver is currently allowed to read
    /// from.
    ///
    /// As the message queue is a circular buffer, the region may be discontiguous in memory. In
    /// that case the second slice will have a non-zero length.
    fn driver_read_area(&self) -> (&[[u8; GSP_PAGE_SIZE]], &[[u8; GSP_PAGE_SIZE]]) {
        let tx = self.gsp_write_ptr();
        let rx = self.cpu_read_ptr();

        // Pointer to the first entry of the GSP message queue.
        let data = ptr::project!(self.0.as_ptr(), .gspq.msgq.data[build: 0]);

        let (tail_end, wrap_end) = if rx <= tx {
            // Read area is non-wrapping and stops right before `tx`.
            (tx, 0)
        } else {
            // Read area is wrapping and stops right before `tx`.
            (MSGQ_NUM_PAGES, tx)
        };

        // SAFETY:
        // - `data` was created from a valid pointer, and `rx` and `tx` are in the
        //   `0..MSGQ_NUM_PAGES` range per the invariants of `gsp_write_ptr` and `cpu_read_ptr`,
        //   thus the created slices are valid.
        // - The area starting at `rx` and ending at `tx - 1` modulo `MSGQ_NUM_PAGES`,
        //   inclusive, belongs to the driver for reading and is not accessed concurrently by
        //   the GSP.
        // - The caller holds a reference to `self` for as long as the returned slices are live,
        //   meaning the CPU read pointer cannot be advanced and thus that the returned area
        //   remains exclusive to the CPU for the duration of the slices.
        unsafe {
            (
                core::slice::from_raw_parts(
                    data.add(num::u32_as_usize(rx)),
                    num::u32_as_usize(tail_end - rx),
                ),
                core::slice::from_raw_parts(data, num::u32_as_usize(wrap_end)),
            )
        }
    }

    /// Allocates a region on the command queue that is large enough to send a command of `size`
    /// bytes, waiting for space to become available based on the provided timeout.
    ///
    /// This returns a [`GspCommand`] ready to be written to by the caller.
    ///
    /// # Errors
    ///
    /// - `EMSGSIZE` if the command is larger than [`GSP_MSG_QUEUE_ELEMENT_SIZE_MAX`].
    /// - `ETIMEDOUT` if space does not become available within the timeout.
    /// - `EIO` if the command header is not properly aligned.
    fn allocate_command(&mut self, size: usize, timeout: Delta) -> Result<GspCommand<'_>> {
        if size_of::<GspMsgElement>() + size > GSP_MSG_QUEUE_ELEMENT_SIZE_MAX {
            return Err(EMSGSIZE);
        }
        read_poll_timeout(
            || Ok(self.driver_write_area_size()),
            |available_bytes| *available_bytes >= size_of::<GspMsgElement>() + size,
            Delta::from_micros(1),
            timeout,
        )?;

        // Get the current writable area as an array of bytes.
        let (slice_1, slice_2) = {
            let (slice_1, slice_2) = self.driver_write_area();

            (slice_1.as_flattened_mut(), slice_2.as_flattened_mut())
        };

        // Extract area for the `GspMsgElement`.
        let (header, slice_1) = GspMsgElement::from_bytes_mut_prefix(slice_1).ok_or(EIO)?;

        // Create the contents area.
        let (slice_1, slice_2) = if slice_1.len() > size {
            // Contents fits entirely in `slice_1`.
            (&mut slice_1[..size], &mut slice_2[0..0])
        } else {
            // Need all of `slice_1` and some of `slice_2`.
            let slice_2_len = size - slice_1.len();
            (slice_1, &mut slice_2[..slice_2_len])
        };

        Ok(GspCommand {
            header,
            contents: (slice_1, slice_2),
        })
    }

    // Returns the index of the memory page the GSP will write the next message to.
    //
    // # Invariants
    //
    // - The returned value is within `0..MSGQ_NUM_PAGES`.
    fn gsp_write_ptr(&self) -> u32 {
        let ptr = MsgqTxHeader::write_ptr(io_project!(self.0, .gspq.tx)) % MSGQ_NUM_PAGES;

        // ORDERING: LOAD->LOAD ordering needed to order `gsp_write_ptr` read before data read.
        dma_mb(Read);

        ptr
    }

    // Returns the index of the memory page the GSP will read the next command from.
    //
    // # Invariants
    //
    // - The returned value is within `0..MSGQ_NUM_PAGES`.
    fn gsp_read_ptr(&self) -> u32 {
        let ptr = MsgqRxHeader::read_ptr(io_project!(self.0, .gspq.rx)) % MSGQ_NUM_PAGES;

        // ORDERING: LOAD->STORE ordering needed to order `gsp_read_ptr` read before data write.
        dma_mb(Full);

        ptr
    }

    // Returns the index of the memory page the CPU can read the next message from.
    //
    // # Invariants
    //
    // - The returned value is within `0..MSGQ_NUM_PAGES`.
    fn cpu_read_ptr(&self) -> u32 {
        MsgqRxHeader::read_ptr(io_project!(self.0, .cpuq.rx)) % MSGQ_NUM_PAGES
    }

    // Informs the GSP that it can send `elem_count` new pages into the message queue.
    fn advance_cpu_read_ptr(&mut self, elem_count: u32) {
        // ORDERING: LOAD->STORE ordering needed to order `cpu_read_ptr` write after data read.
        dma_mb(Full);

        let rx = io_project!(self.0, .cpuq.rx);
        let rptr = MsgqRxHeader::read_ptr(rx).wrapping_add(elem_count) % MSGQ_NUM_PAGES;
        MsgqRxHeader::set_read_ptr(rx, rptr)
    }

    // Returns the index of the memory page the CPU can write the next command to.
    //
    // # Invariants
    //
    // - The returned value is within `0..MSGQ_NUM_PAGES`.
    fn cpu_write_ptr(&self) -> u32 {
        MsgqTxHeader::write_ptr(io_project!(self.0, .cpuq.tx)) % MSGQ_NUM_PAGES
    }

    // Informs the GSP that it can process `elem_count` new pages from the command queue.
    fn advance_cpu_write_ptr(&mut self, elem_count: u32) {
        // ORDERING: STORE->STORE ordering needed to order `cpu_write_ptr` write after data write.
        dma_mb(Write);

        let tx = io_project!(self.0, .cpuq.tx);
        let wptr = MsgqTxHeader::write_ptr(tx).wrapping_add(elem_count) % MSGQ_NUM_PAGES;
        MsgqTxHeader::set_write_ptr(tx, wptr);
    }
}

/// A command ready to be sent on the command queue.
///
/// This is the type returned by [`DmaGspMem::allocate_command`].
struct GspCommand<'a> {
    // Writable reference to the header of the command.
    header: &'a mut GspMsgElement,
    // Writable slices to the contents of the command. The second slice is zero unless the command
    // loops over the command queue.
    contents: (&'a mut [u8], &'a mut [u8]),
}

/// A message ready to be processed from the message queue.
///
/// This is the type returned by [`CmdqInner::wait_for_msg`].
struct GspMessage<'a> {
    // Reference to the header of the message.
    header: &'a GspMsgElement,
    // Slices to the contents of the message. The second slice is empty unless the message loops
    // over the message queue.
    contents: (&'a [u8], &'a [u8]),
}

/// A message whose protocol layer has been parsed and validated.
enum GspParsedMessage<'a> {
    /// A [RPC message](rpc::RpcMessage).
    Rpc(rpc::RpcMessage<'a>),
}

impl GspParsedMessage<'_> {
    /// Log the message on `dev`.
    fn log(&self, dev: &device::Device) {
        match self {
            GspParsedMessage::Rpc(rpc) => rpc.log(dev),
        }
    }
}

/// GSP command queue.
///
/// Provides the ability to send commands and receive messages from the GSP using a shared memory
/// area.
#[pin_data]
pub(crate) struct Cmdq<'cmdq> {
    /// Inner mutex-protected state.
    #[pin]
    inner: Mutex<CmdqInner<'cmdq>>,
    /// DMA address of the command queue's shared memory region.
    pub(super) dma_addr: DmaAddress,
}

impl<'cmdq> Cmdq<'cmdq> {
    /// Offset of the data after the PTEs.
    const POST_PTE_OFFSET: usize = core::mem::offset_of!(GspMem, cpuq);

    /// Offset of command queue ring buffer.
    pub(crate) const CMDQ_OFFSET: usize = core::mem::offset_of!(GspMem, cpuq)
        + core::mem::offset_of!(Msgq, msgq)
        - Self::POST_PTE_OFFSET;

    /// Offset of message queue ring buffer.
    pub(crate) const STATQ_OFFSET: usize = core::mem::offset_of!(GspMem, gspq)
        + core::mem::offset_of!(Msgq, msgq)
        - Self::POST_PTE_OFFSET;

    /// Number of page table entries for the GSP shared region.
    pub(crate) const NUM_PTES: usize = size_of::<GspMem>() >> GSP_PAGE_SHIFT;

    /// Default timeout for receiving a message from the GSP.
    pub(super) const RECEIVE_TIMEOUT: Delta = Delta::from_secs(5);

    /// Creates a new command queue for `dev`.
    pub(crate) fn new(
        dev: &'cmdq device::Device<device::Bound>,
        bar: Bar0<'cmdq>,
    ) -> impl PinInit<Self, Error> + 'cmdq {
        pin_init_scope(move || {
            let gsp_mem = DmaGspMem::new(dev)?;

            Ok(try_pin_init!(Self {
                dma_addr: gsp_mem.0.dma_address(),
                inner <- new_mutex!(CmdqInner {
                    dev,
                    bar,
                    gsp_mem,
                    seq: 0,
                }),
            }))
        })
    }

    /// Computes the checksum for the message pointed to by `it`.
    ///
    /// A message is made of several parts, so `it` is an iterator over byte slices representing
    /// these parts.
    fn calculate_checksum<T: Iterator<Item = u8>>(it: T) -> u32 {
        let sum64 = it
            .enumerate()
            .map(|(idx, byte)| (((idx % 8) * 8) as u32, byte))
            .fold(0, |acc, (rol, byte)| acc ^ u64::from(byte).rotate_left(rol));

        ((sum64 >> 32) as u32) ^ (sum64 as u32)
    }

    /// Notifies the GSP that we have updated the command queue pointers.
    fn notify_gsp(bar: Bar0<'_>) {
        bar.write_reg(regs::NV_PGSP_QUEUE_HEAD::zeroed().with_address(0u32));
    }

    /// Logs and consumes every message the GSP has already posted, and returns without waiting for
    /// more.
    ///
    /// No caller is waiting for a reply while this holds the queue mutex, so every message is
    /// logged as an event. See "Draining the GSP-to-CPU queue" in
    /// `Documentation/gpu/nova/core/interrupts.rst`.
    ///
    /// # Errors
    ///
    /// `EIO` if a message fails framing or checksum validation.
    pub(crate) fn drain(&self) -> Result {
        self.inner.lock().drain()
    }
}

/// Inner mutex protected state of [`Cmdq`].
struct CmdqInner<'a> {
    /// Device this command queue belongs to.
    dev: &'a device::Device,
    /// MMIO mapping of PCI BAR0, for writing the GSP doorbell.
    bar: Bar0<'a>,
    /// Current command sequence number.
    seq: u32,
    /// Memory area shared with the GSP for communicating commands and messages.
    gsp_mem: DmaGspMem<'a>,
}

impl CmdqInner<'_> {
    /// Timeout for waiting for space on the command queue.
    const ALLOCATE_TIMEOUT: Delta = Delta::from_secs(1);

    /// Allocate enough space on the command queue to store `command`, initialize it with
    /// [`CommandElement::write`], and send the command to the GSP.
    ///
    /// # Errors
    ///
    /// - `EMSGSIZE` if the command exceeds the maximum queue element size.
    /// - `ETIMEDOUT` if space does not become available within the timeout.
    ///
    /// Error codes returned by [`CommandElement::write`] are returned as-is.
    fn send_command_element<E: CommandElement>(&mut self, command: E) -> Result {
        let size_in_bytes = command.size();
        let mut dst = self
            .gsp_mem
            .allocate_command(size_in_bytes, Self::ALLOCATE_TIMEOUT)?;

        // Fill the header.
        let msg_element_init = GspMsgElement::init(self.seq, size_in_bytes);
        // SAFETY: `dst.header` is a valid reference, and is not touched if the initializer fails.
        unsafe {
            pin_init::raw_try_init(core::ptr::from_mut(dst.header), msg_element_init)?;
        }

        // Write the message payload.
        command.write(self.dev, self.seq, &mut dst)?;

        // Compute checksum now that the whole message is ready.
        dst.header
            .set_checksum(Cmdq::calculate_checksum(SBufferIter::new_reader([
                dst.header.as_bytes(),
                dst.contents.0,
                dst.contents.1,
            ])));

        // All set - update the write pointer and inform the GSP of the new command.
        let elem_count = dst.header.element_count();
        self.seq += 1;
        self.gsp_mem.advance_cpu_write_ptr(elem_count);
        Cmdq::notify_gsp(self.bar);

        Ok(())
    }

    /// Wait for a message to become available on the message queue.
    ///
    /// This works purely at the transport layer and does not interpret or validate the message
    /// beyond the advertised length in its [`GspMsgElement`].
    ///
    /// This method returns:
    ///
    /// - A reference to the [`GspMsgElement`] of the message,
    /// - Two byte slices with the contents of the message. The second slice is empty unless the
    ///   message loops across the message queue.
    ///
    /// # Errors
    ///
    /// - `ETIMEDOUT` if `timeout` has elapsed before any message becomes available.
    /// - `EIO` if a message fails framing or checksum validation.
    fn wait_for_msg(&self, timeout: Delta) -> Result<GspMessage<'_>> {
        // Wait for a message to arrive from the GSP.
        let (slice_1, slice_2) = if timeout == Delta::ZERO {
            // If `timeout == Delta::ZERO`, check the message queue once as `read_poll_timeout` can
            // potentially sleep if the clock hasn't moved forward while the condition was being
            // checked.
            let slices = self.gsp_mem.driver_read_area();
            if !slices.0.is_empty() {
                Ok(slices)
            } else {
                Err(ETIMEDOUT)
            }
        } else {
            read_poll_timeout(
                || Ok(self.gsp_mem.driver_read_area()),
                |driver_area| !driver_area.0.is_empty(),
                Delta::from_millis(1),
                timeout,
            )
        }
        .map(|(slice_1, slice_2)| (slice_1.as_flattened(), slice_2.as_flattened()))?;

        // Extract the `GspMsgElement`.
        let (header, slice_1) = GspMsgElement::from_bytes_prefix(slice_1).ok_or(EIO)?;

        // Validate checksum after truncating the message to its exact length.
        if Cmdq::calculate_checksum(
            SBufferIter::new_reader([header.as_bytes(), slice_1, slice_2])
                .take(header.msg_length()),
        ) != 0
        {
            dev_err!(&self.dev, "GSP receive: bad checksum\n");
            return Err(EIO);
        }

        Ok(GspMessage {
            header,
            contents: (slice_1, slice_2),
        })
    }

    /// Wait for the next element on the message queue, pass it to `f`, advance the read pointer
    /// past the message, and return the result of `f`.
    ///
    /// The read pointer advances regardless of whether `f` succeeds or not.
    ///
    /// # Errors
    ///
    /// - `ETIMEDOUT` if `timeout` has elapsed before any message becomes available.
    /// - `EIO` if a message fails framing or checksum validation. In this case, the read pointer
    ///   does not advance and the queue is stalled. This is a fatal error.
    ///
    /// If an error is returned, then `f` has not been called.
    ///
    /// Note that `R` may itself be a [`Result`]. In this case, the above only applies to the outer
    /// [`Result`] returned by this method.
    fn consume_element<R>(
        &mut self,
        timeout: Delta,
        f: impl FnOnce(GspParsedMessage<'_>) -> R,
    ) -> Result<R> {
        let message = self.wait_for_msg(timeout)?;
        let elem_count = u32::try_from(message.header.msg_length().div_ceil(GSP_PAGE_SIZE))
            .map_err(|_| {
                dev_err!(&self.dev, "GSP receive: message length overflow\n");
                EIO
            })?;

        // RPC is the only type of message supported for now.
        let message = GspParsedMessage::Rpc(rpc::RpcMessage::parse(self.dev, message)?);

        let res = f(message);

        // Advance the read pointer past this message.
        self.gsp_mem.advance_cpu_read_ptr(elem_count);

        Ok(res)
    }

    /// Consumes messages from the queue until a message that satisfies `f` is encountered, and
    /// returns the result of `f`.
    ///
    /// The message is waited for up to [`Cmdq::RECEIVE_TIMEOUT`] from the call.
    ///
    /// `f` is expected to return `Ok` with the result to return to the caller, and `Err(ENOMSG)` to
    /// signal that the received message was not the one expected (such messages are dropped). Any
    /// other error returned by `f` is propagated to the caller.
    ///
    /// # Errors
    ///
    /// - `ETIMEDOUT` if no message satisfying `f` arrives before the deadline, however many other
    ///   messages arrive while waiting.
    /// - `EIO` if a message fails framing or checksum validation.
    /// - Any error returned by `f` with the exception of `ENOMSG`.
    fn await_msg_with<R>(&mut self, f: impl Fn(GspParsedMessage<'_>) -> Result<R>) -> Result<R> {
        let deadline = Instant::<Monotonic>::now() + Cmdq::RECEIVE_TIMEOUT;
        loop {
            let remaining = deadline - Instant::<Monotonic>::now();
            if remaining.is_negative() {
                break Err(ETIMEDOUT);
            }
            match self.consume_element(remaining, &f)? {
                Ok(m) => break Ok(m),
                Err(ENOMSG) => continue,
                Err(e) => break Err(e),
            }
        }
    }

    /// Logs and consumes every message the queue holds.
    ///
    /// # Errors
    ///
    /// `EIO` if a message fails framing or checksum validation, or if a message's page count
    /// overflows a `u32`.
    fn drain(&mut self) -> Result {
        loop {
            match self.consume_element(Delta::ZERO, |message| message.log(self.dev)) {
                Ok(()) => (),
                // No more message pending on the queue, drain is complete.
                Err(ETIMEDOUT) => return Ok(()),
                // Other errors are fatal, and stall the queue.
                Err(e) => return Err(e),
            }
        }
    }
}
