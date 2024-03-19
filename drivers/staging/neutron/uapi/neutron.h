/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/*
 * Copyright 2023 NXP
 *
 */

#ifndef NEUTRON_H
#define NEUTRON_H

/****************************************************************************
 * Includes
 ****************************************************************************/

#include <linux/ioctl.h>
#include <linux/types.h>

#ifdef __cplusplus
namespace Neutron {
#endif

/****************************************************************************
 * Defines
 ****************************************************************************/

#define NEUTRON_IOCTL_BASE		0x01
#define NEUTRON_IO(nr)			_IO(NEUTRON_IOCTL_BASE, nr)
#define NEUTRON_IOR(nr, type)		_IOR(NEUTRON_IOCTL_BASE, nr, type)
#define NEUTRON_IOW(nr, type)		_IOW(NEUTRON_IOCTL_BASE, nr, type)
#define NEUTRON_IOWR(nr, type)		_IOWR(NEUTRON_IOCTL_BASE, nr, type)

#define NEUTRON_IOCTL_PING		NEUTRON_IO(0x00)
#define NEUTRON_IOCTL_VERSION_REQ	NEUTRON_IO(0x01)
#define NEUTRON_IOCTL_CLEAR_STATUS	NEUTRON_IO(0x02)

#define NEUTRON_IOCTL_BUFFER_CREATE	NEUTRON_IOW(0x07, \
						   struct neutron_uapi_buffer_create)
#define NEUTRON_IOCTL_INFERENCE_CREATE	NEUTRON_IOW(0x08, \
						   struct neutron_uapi_inference_args)
#define NEUTRON_IOCTL_KERNEL_LOAD	NEUTRON_IOW(0x09, \
						   struct neutron_uapi_inference_args)
#define NEUTRON_IOCTL_INFERENCE_STATE	NEUTRON_IOW(0x0a, \
						   struct neutron_uapi_result_status)

/****************************************************************************
 * Types
 ****************************************************************************/

/**
 * enum neutron_uapi_status - Status
 */
enum neutron_uapi_status {
	NEUTRON_UAPI_STATUS_PENDING,
	NEUTRON_UAPI_STATUS_RUNNING,
	NEUTRON_UAPI_STATUS_DONE,
	NEUTRON_UAPI_STATUS_ERROR,
	NEUTRON_UAPI_STATUS_TIMEOUT,
	NEUTRON_UAPI_STATUS_UNREADY,
};

/**
 * struct neutron_uapi_reg_config - Configure register
 * @offset:       Offset of the register
 * @value:        Value of the register.
 */
struct neutron_uapi_reg_config {
	__u32 offset;
	__u32 value;
};

/**
 * struct neutron_uapi_result_status - Status of inference
 * @status:      Status of inference job.
 * @error_code:  Neutron error code when status is error.
 */
struct neutron_uapi_result_status {
	enum neutron_uapi_status status;
	__u32 error_code;
};

/**
 * struct neutron_uapi_buffer_create - Create buffer request
 * @size:   Capacity of the buffer
 * @addr:   Dma addr of the buffer
 */
struct neutron_uapi_buffer_create {
	__u32 size;
	__u64 addr;
};

/**
 * struct neutron_uapi_inference_args - Job creation struct
 * @args:              Union parameters
 * @tensor_offset:     Tensor array address offset, inputs, outputs, weight, scratch
 * @kernel_offset:     Kernel address offset
 * @microcode_offset:  Microcode address offset
 * @tensor_count:      Valid tensor number.
 * @dram_base:         Physical base address in DDR, used by neutron.
 * @reserve:           Reserve for future.
 */
struct neutron_uapi_inference_args {
	union {
		__u32  args0;
		__u32 tensor_offset;
		__u32 kernel_offset;
	};
	union {
		__u32  args1;
		__u32 microcode_offset;
	};
	union {
		__u32  args2;
		__u32  tensor_count;
	};
	union {
		__u32  args3;
	};
	__u32 dram_base;
	__u32 reserve[2];
};


#ifdef __cplusplus
} /* namespace Neutron */
#endif
#endif /* NEUTRON_H */
