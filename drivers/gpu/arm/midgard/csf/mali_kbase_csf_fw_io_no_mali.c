// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2024 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include "mali_kbase.h"
#include "mali_kbase_csf_fw_io.h"
#include <mali_kbase_linux.h>

#include <linux/mutex.h>

static inline u32 input_page_read(const u32 *const input, const u32 offset)
{
	WARN_ON(offset % sizeof(u32));

	return input[offset / sizeof(u32)];
}

static inline void input_page_write(u32 *const input, const u32 offset, const u32 value)
{
	WARN_ON(offset % sizeof(u32));

	input[offset / sizeof(u32)] = value;
}

static inline void input_page_partial_write(u32 *const input, const u32 offset, u32 value, u32 mask)
{
	WARN_ON(offset % sizeof(u32));

	input[offset / sizeof(u32)] = (input_page_read(input, offset) & ~mask) | (value & mask);
}

static inline u32 output_page_read(const u32 *const output, const u32 offset)
{
	WARN_ON(offset % sizeof(u32));

	return output[offset / sizeof(u32)];
}

static inline void output_page_write(u32 *const output, const u32 offset, const u32 value)
{
	WARN_ON(offset % sizeof(u32));

	output[offset / sizeof(u32)] = value;
}


void kbase_csf_fw_io_init(struct kbase_csf_fw_io *fw_io, struct kbase_device *kbdev)
{
	spin_lock_init(&fw_io->lock);
	bitmap_zero(fw_io->status, KBASEP_FW_IO_STATUS_NUM_BITS);
	fw_io->kbdev = kbdev;
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_init);

void kbase_csf_fw_io_term(struct kbase_csf_fw_io *fw_io)
{
	/* Nothing to do. */
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_term);

int kbase_csf_fw_io_groups_pages_init(struct kbase_csf_fw_io *fw_io, u32 group_num)
{
	struct kbasep_csf_fw_io_group_pages **groups_pages = &fw_io->pages.groups_pages;

	*groups_pages = kcalloc(group_num, sizeof(**groups_pages), GFP_KERNEL);
	return *groups_pages ? 0 : -ENOMEM;
}

int kbase_csf_fw_io_streams_pages_init(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 stream_num)
{
	struct kbasep_csf_fw_io_stream_pages **streams_pages =
		&fw_io->pages.groups_pages[group_id].streams_pages;

	*streams_pages = kcalloc(stream_num, sizeof(**streams_pages), GFP_KERNEL);
	return *streams_pages ? 0 : -ENOMEM;
}

void kbase_csf_fw_io_set_global_pages(struct kbase_csf_fw_io *fw_io, void *input, void *output)
{
	fw_io->pages.input = input;
	fw_io->pages.output = output;
}

void kbase_csf_fw_io_set_group_pages(struct kbase_csf_fw_io *fw_io, u32 group_id, void *input,
				     void *output)
{
	fw_io->pages.groups_pages[group_id].input = input;
	fw_io->pages.groups_pages[group_id].output = output;
}

void kbase_csf_fw_io_set_stream_pages(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 stream_id,
				      void *input, void *output)
{
	fw_io->pages.groups_pages[group_id].streams_pages[stream_id].input = input;
	fw_io->pages.groups_pages[group_id].streams_pages[stream_id].output = output;
}

void kbase_csf_fw_io_pages_term(struct kbase_csf_fw_io *fw_io, u32 group_num)
{
	struct kbasep_csf_fw_io_pages *fw_io_pages = &fw_io->pages;

	if (fw_io_pages->groups_pages) {
		unsigned int gid;

		for (gid = 0; gid < group_num; ++gid)
			kfree(fw_io_pages->groups_pages[gid].streams_pages);

		kfree(fw_io_pages->groups_pages);
		fw_io_pages->groups_pages = NULL;
	}
}

void kbase_csf_fw_io_global_write(struct kbase_csf_fw_io *fw_io, u32 offset, u32 value)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_pages *pages = &fw_io->pages;

	lockdep_assert_held(&fw_io->lock);

	dev_dbg(kbdev->dev, "glob input w: reg %08x val %08x\n", offset, value);
	input_page_write(pages->input, offset, value);

	if (offset == GLB_REQ) {
		/* NO_MALI: Immediately acknowledge requests - except for PRFCNT_ENABLE
		 * and PRFCNT_SAMPLE. These will be processed along with the
		 * corresponding performance counter registers when the global doorbell
		 * is rung in order to emulate the performance counter sampling behavior
		 * of the real firmware.
		 */
		const u32 ack = output_page_read(pages->output, GLB_ACK);
		const u32 req_mask = ~(GLB_REQ_PRFCNT_ENABLE_MASK | GLB_REQ_PRFCNT_SAMPLE_MASK);
		const u32 toggled = (value ^ ack) & req_mask;

		output_page_write(pages->output, GLB_ACK, ack ^ toggled);
	}
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_global_write);

void kbase_csf_fw_io_global_write_mask(struct kbase_csf_fw_io *fw_io, u32 offset, u32 value,
				       u32 mask)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_pages *pages = &fw_io->pages;

	lockdep_assert_held(&fw_io->lock);

	dev_dbg(kbdev->dev, "glob input w: reg %08x val %08x mask %08x\n", offset, value, mask);

	/* NO_MALI: Go through existing function to capture writes */
	kbase_csf_fw_io_global_write(
		fw_io, offset, (input_page_read(pages->input, offset) & ~mask) | (value & mask));
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_global_write_mask);

u32 kbase_csf_fw_io_global_input_read(struct kbase_csf_fw_io *fw_io, u32 offset)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_pages *pages = &fw_io->pages;
	u32 val;

	val = input_page_read(pages->input, offset);
	dev_dbg(kbdev->dev, "glob input r: reg %08x val %08x\n", offset, val);

	return val;
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_global_input_read);

u32 kbase_csf_fw_io_global_read(struct kbase_csf_fw_io *fw_io, u32 offset)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_pages *pages = &fw_io->pages;
	u32 val;

	val = output_page_read(pages->output, offset);
	dev_dbg(kbdev->dev, "glob output r: reg %08x val %08x\n", offset, val);

	return val;
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_global_read);

void kbase_csf_fw_io_group_write(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 offset, u32 value)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_group_pages *group_pages = &fw_io->pages.groups_pages[group_id];

	lockdep_assert_held(&fw_io->lock);

	dev_dbg(kbdev->dev, "csg input w: reg %08x val %08x csg_id %u\n", offset, value, group_id);
	input_page_write(group_pages->input, offset, value);

	if (offset == CSG_REQ) {
		/* NO_MALI: Immediately acknowledge requests */
		output_page_write(group_pages->output, CSG_ACK, value);
	}
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_group_write);

void kbase_csf_fw_io_group_write_mask(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 offset,
				      u32 value, u32 mask)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_group_pages *group_pages = &fw_io->pages.groups_pages[group_id];

	lockdep_assert_held(&fw_io->lock);

	dev_dbg(kbdev->dev, "csg input w: reg %08x val %08x mask %08x csg_id %u\n", offset, value,
		mask, group_id);

	/* NO_MALI: Go through existing function to capture writes */
	kbase_csf_fw_io_group_write(fw_io, group_id, offset,
				    (input_page_read(group_pages->input, offset) & ~mask) |
					    (value & mask));
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_group_write_mask);

u32 kbase_csf_fw_io_group_input_read(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 offset)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_group_pages *group_pages = &fw_io->pages.groups_pages[group_id];
	u32 val;

	val = input_page_read(group_pages->input, offset);
	dev_dbg(kbdev->dev, "csg input r: reg %08x val %08x csg_id %u\n", offset, val, group_id);

	return val;
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_group_input_read);

u32 kbase_csf_fw_io_group_read(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 offset)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_group_pages *group_pages = &fw_io->pages.groups_pages[group_id];
	u32 val;

	val = output_page_read(group_pages->output, offset);
	dev_dbg(kbdev->dev, "csg output r: reg %08x val %08x csg_id %u\n", offset, val, group_id);

	return val;
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_group_read);


void kbase_csf_fw_io_stream_write(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 stream_id,
				  u32 offset, u32 value)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_stream_pages *stream_pages =
		&fw_io->pages.groups_pages[group_id].streams_pages[stream_id];

	lockdep_assert_held(&fw_io->lock);

	dev_dbg(kbdev->dev, "cs input w: reg %08x val %08x csg_id %u cs_id %u\n", offset, value,
		group_id, stream_id);
	input_page_write(stream_pages->input, offset, value);

	if (offset == CS_REQ) {
		/* NO_MALI: Immediately acknowledge requests */
		output_page_write(stream_pages->output, CS_ACK, value);
	}
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_stream_write);

void kbase_csf_fw_io_stream_write_mask(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 stream_id,
				       u32 offset, u32 value, u32 mask)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_stream_pages *stream_pages =
		&fw_io->pages.groups_pages[group_id].streams_pages[stream_id];

	lockdep_assert_held(&fw_io->lock);

	dev_dbg(kbdev->dev, "cs input w: reg %08x val %08x mask %08x csg_id %u cs_id %u\n", offset,
		value, mask, group_id, stream_id);

	/* NO_MALI: Go through existing function to capture writes */
	kbase_csf_fw_io_stream_write(fw_io, group_id, stream_id, offset,
				     (input_page_read(stream_pages->input, offset) & ~mask) |
					     (value & mask));
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_stream_write_mask);

u32 kbase_csf_fw_io_stream_input_read(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 stream_id,
				      u32 offset)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_stream_pages *stream_pages =
		&fw_io->pages.groups_pages[group_id].streams_pages[stream_id];
	u32 val;

	val = input_page_read(stream_pages->input, offset);
	dev_dbg(kbdev->dev, "cs input r: reg %08x val %08x csg_id %u cs_id %u\n", offset, val,
		group_id, stream_id);

	return val;
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_stream_input_read);

u32 kbase_csf_fw_io_stream_read(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 stream_id,
				u32 offset)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_stream_pages *stream_pages =
		&fw_io->pages.groups_pages[group_id].streams_pages[stream_id];
	u32 val;

	val = output_page_read(stream_pages->output, offset);
	dev_dbg(kbdev->dev, "cs output r: reg %08x val %08x csg_id %u cs_id %u\n", offset, val,
		group_id, stream_id);

	return val;
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_stream_read);

void kbase_csf_fw_io_set_status_gpu_suspended(struct kbase_csf_fw_io *fw_io)
{
	set_bit(KBASEP_FW_IO_STATUS_GPU_SUSPENDED, fw_io->status);
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_set_status_gpu_suspended);

void kbase_csf_fw_io_clear_status_gpu_suspended(struct kbase_csf_fw_io *fw_io)
{
	clear_bit(KBASEP_FW_IO_STATUS_GPU_SUSPENDED, fw_io->status);
}

bool kbase_csf_fw_io_check_status_gpu_suspended(struct kbase_csf_fw_io *fw_io)
{
	return !bitmap_empty(fw_io->status, KBASEP_FW_IO_STATUS_NUM_BITS);
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_check_status_gpu_suspended);

void kbase_csf_fw_io_mock_fw_global_write(struct kbase_csf_fw_io *fw_io, u32 offset, u32 value)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_pages *pages = &fw_io->pages;

	dev_dbg(kbdev->dev, "mock fw glob output w: reg %08x val %08x\n", offset, value);
	output_page_write(pages->output, offset, value);
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_mock_fw_global_write);

void kbase_csf_fw_io_mock_fw_group_write(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 offset,
					 u32 value)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_group_pages *group_pages = &fw_io->pages.groups_pages[group_id];

	dev_dbg(kbdev->dev, "mock fw csg output w: reg %08x val %08x csg_id %u\n", offset, value,
		group_id);
	output_page_write(group_pages->output, offset, value);
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_mock_fw_group_write);

void kbase_csf_fw_io_mock_fw_stream_write(struct kbase_csf_fw_io *fw_io, u32 group_id,
					  u32 stream_id, u32 offset, u32 value)
{
	const struct kbase_device *const kbdev = fw_io->kbdev;
	struct kbasep_csf_fw_io_stream_pages *stream_pages =
		&fw_io->pages.groups_pages[group_id].streams_pages[stream_id];

	dev_dbg(kbdev->dev, "mock fw cs output w: reg %08x val %08x csg_id %u cs_id %u\n", offset,
		value, group_id, stream_id);
	output_page_write(stream_pages->output, offset, value);
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_mock_fw_stream_write);
