/* Copyright 2017 NXP Semiconductor, Inc.
 * Copyright 2020 Puresoftware Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of NXP Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NXP Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/acpi.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include "dpaa_sys.h"

/* QMan needs global memory areas initialized at boot time */
static dma_addr_t qman_base_addr;

/*
 * Initialize a devices private memory region
 */
int qbman_init_private_mem(struct device *dev, int idx, const char *compat,
			   dma_addr_t *addr, size_t *size, int dev_id)
{
	struct property_entry properties[2];
	struct device_node *mem_node = NULL;
	struct reserved_mem fw_mem;
	struct reserved_mem *rmem;
	__be32 *res_array;
	u32 qbman_vals[4];
	u32 *pr_value;
	int val_cnt;
	u32 val[2];
	int err;

	if (is_of_node(dev->fwnode)) {
		mem_node = of_parse_phandle(dev->of_node, "memory-region", idx);
		if (!mem_node) {
			mem_node = of_find_compatible_node(NULL, NULL, compat);
			if (!mem_node) {
				dev_err(dev, "No memory-region found for index %d or compatible '%s'\n",
					idx, compat);
				return -ENODEV;
			}
		}

		rmem = of_reserved_mem_lookup(mem_node);
		if (!rmem) {
			dev_err(dev, "of_reserved_mem_lookup() returned NULL\n");
			return -ENODEV;
		}
	} else {
		/*
		 * Fetching reserved memory size from scanning ACPI tables.
		 * As part of DPAA architecture, QMAN & BMAN h/w nodes need
		 * a large contiguous memory allocations to store private
		 * data while the data path is running.
		 * We will have to request CMA for each h/w node so that
		 * drivers can fetch and set up h/w in order while probing.
		 */
		struct page *page = NULL;
		size_t page_sz_count = 0;
		unsigned long pool_size_order = 0;

		switch (dev_id) {
		case DPAA_BMAN_DEV:
			val_cnt = 1;
			break;
		case DPAA_QMAN_DEV:
			val_cnt = 2;
			break;
		default:
			return -ENODEV;
		}

		err = fwnode_property_read_u32_array(dev->fwnode,
						     "size", val,
						     val_cnt);
		if (err < 0)
			return err;

		fw_mem.size = val[idx];

		if (dev_id == DPAA_BMAN_DEV) {
			/* In case of Bman, calculate page count and order.
			 * Try allocating this 16MB chunk in one go.
			 */
			page_sz_count = ((fw_mem.size >> PAGE_SHIFT) +
					((fw_mem.size & 0xFFF) ? 1 : 0));
			pool_size_order = get_order(fw_mem.size);
		} else {
			if (!idx) {
				/* In case of Qman, allocate 48 MB -
				 * (8MB + 8MB + 32MB), ideally we need
				 * (8MB + 32MB). Here extra 8MB is just to set
				 * the correct alignment order.
				 */
				fw_mem.size = ((2 * val[idx]) + val[idx + 1]);
				page_sz_count = ((fw_mem.size >> PAGE_SHIFT) +
					((fw_mem.size & 0xFFF) ? 1 : 0));
				pool_size_order = get_order(fw_mem.size);
				/* Once large chunk(48MB) is available then
				 * reset the actual size 8MB for h/w node on
				 * index 0
				 */
				fw_mem.size = val[idx];
			} else {
				/* From the large chunk of 48MB, slice it
				 * at base_address + 16MB, to get the aligned
				 * 32MB chunk.
				 */
				fw_mem.base =
					(qman_base_addr + (2 * val[idx - 1]));
				fw_mem.size = val[idx];
			}
		}
		if (!qman_base_addr) {
			page = dma_alloc_from_contiguous(dev, page_sz_count,
							 pool_size_order,
							 false);
			if (!page) {
				pr_info("dma_alloc_from_contiguous failed.\n");
				return -ENOMEM;
			}
			fw_mem.base = page_to_phys(page);
			if (dev_id == DPAA_QMAN_DEV)
				qman_base_addr = fw_mem.base;
		}
		/* Set the resource buffer */
		rmem = &fw_mem;

		dev_info(dev, "QBman : dev [%d] index [%d] mem-base [%llx] size [%llx]\n",
			 dev_id, idx, rmem->base, rmem->size);
	}

	*addr = rmem->base;
	*size = rmem->size;

	/*
	 * Check if the reg property exists - if not insert the node
	 * so upon kexec() the same memory region address will be preserved.
	 * This is needed because QBMan HW does not allow the base address/
	 * size to be modified once set.
	 */
	if (is_of_node(dev->fwnode)) {
		if (!of_property_present(mem_node, "reg")) {
			struct property *prop;

			prop = devm_kzalloc(dev, sizeof(*prop), GFP_KERNEL);
			if (!prop)
				return -ENOMEM;
			prop->value = devm_kzalloc(dev, sizeof(__be32) * 4,
						   GFP_KERNEL);
			if (!prop->value)
				return -ENOMEM;
			res_array = prop->value;
			res_array[0] = cpu_to_be32(upper_32_bits(*addr));
			res_array[1] = cpu_to_be32(lower_32_bits(*addr));
			res_array[2] = cpu_to_be32(upper_32_bits(*size));
			res_array[3] = cpu_to_be32(lower_32_bits(*size));
			prop->length = sizeof(__be32) * 4;
			prop->name = devm_kstrdup(dev, "reg", GFP_KERNEL);
			if (!prop->name)
				return -ENOMEM;
			err = of_add_property(mem_node, prop);
			if (err)
				return err;
		}
	} else {
		if (!device_property_present(dev, "reg")) {
			/* Fill properties here */
			pr_value = devm_kzalloc(dev, sizeof(u32) * 4,
						GFP_KERNEL);
			pr_value[0] = upper_32_bits(*addr);
			pr_value[1] = lower_32_bits(*addr);
			pr_value[2] = upper_32_bits(*size);
			pr_value[3] = lower_32_bits(*size);

			qbman_vals[0] = pr_value[0];
			qbman_vals[1] = pr_value[1];
			qbman_vals[2] = pr_value[2];
			qbman_vals[3] = pr_value[3];

			properties[0] =
				PROPERTY_ENTRY_U32_ARRAY("reg", qbman_vals);

			device_create_managed_software_node(dev, properties, NULL);
		}
	}

	return 0;
}
