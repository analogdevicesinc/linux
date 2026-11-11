// SPDX-License-Identifier: GPL-2.0

use kernel::io::register;

use crate::{
    driver::NovaRegisters,
    regs::NV_PBUS_SW_SCRATCH, //
};

// PGSP

// The GSP has eight numbered queues, each with its own four msgq pointer registers interleaved
// at a stride of 8 bytes. nova-core uses queue 0 only.

register! {
    base: NovaRegisters;

    /// Write pointer of the CPU-to-GSP command queue, which the driver advances. A write to this
    /// register also interrupts the GSP, which makes it the doorbell.
    pub(super) NV_PGSP_QUEUE_HEAD(u32)[8, stride = 8] @ 0x00110c00 {
        31:0    address;
    }

    /// Read pointer of the CPU-to-GSP command queue, which the GSP advances.
    pub(super) NV_PGSP_QUEUE_TAIL(u32)[8, stride = 8] @ 0x00110c04 {
        31:0    address;
    }

    /// Write pointer of the GSP-to-CPU message queue, which the GSP advances.
    pub(super) NV_PGSP_MSGQ_HEAD(u32)[8, stride = 8] @ 0x00110c80 {
        31:0    val;
    }

    /// Read pointer of the GSP-to-CPU message queue, which the driver advances.
    pub(super) NV_PGSP_MSGQ_TAIL(u32)[8, stride = 8] @ 0x00110c84 {
        31:0    val;
    }
}

// PBUS

register! {
    base: NovaRegisters;

    /// Scratch register 0xe used as FRTS firmware error code.
    pub(super) NV_PBUS_SW_SCRATCH_0E_FRTS_ERR(u32) => NV_PBUS_SW_SCRATCH[0xe] {
        31:16   frts_err_code;
    }
}
