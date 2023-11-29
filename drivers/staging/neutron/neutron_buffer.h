/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2023 NXP
 */

#ifndef NEUTRON_BUFFER_H
#define NEUTRON_BUFFER_H

/****************************************************************************/

#include <linux/kref.h>
#include <linux/types.h>

/****************************************************************************/

struct neutron_device;
struct device;

/**
 * struct neutron_buffer - Buffer
 * @dev:                Device
 * @file:               File
 * @kref:               Reference counting
 * @capacity:           Maximum capacity of the buffer
 * @cpu_addr:           Kernel mapped address
 * @dma_addr:           DMA address
 * @dma_addr_orig:      Original DMA address before range mapping
 */
struct neutron_buffer {
	struct neutron_device *ndev;
	struct file           *file;
	struct kref           kref;
	size_t                size;
	void                  *cpu_addr;
	dma_addr_t            dma_addr;
};

/****************************************************************************/

/**
 * neutron_buffer_create() - Create buffer
 *
 * This function must be called in the context of a user space process.
 *
 * Return: fd on success, else error code.
 */
int neutron_buffer_create(struct neutron_device *ndev,
			  size_t size, __u64 *addr_out);

/**
 * neutron_buffer_get() - Gut buffer
 */
void neutron_buffer_get(struct neutron_buffer *buf);

/**
 * neutron_buffer_put() - Put buffer
 */
void neutron_buffer_put(struct neutron_buffer *buf);

#endif /* NEUTRON_BUFFER_H */
