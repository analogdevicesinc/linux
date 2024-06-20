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

void kbase_csf_fw_io_init(struct kbase_csf_fw_io *fw_io)
{
	spin_lock_init(&fw_io->lock);
	bitmap_zero(fw_io->status, KBASE_FW_IO_STATUS_NUM_BITS);
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_init);

void kbase_csf_fw_io_term(struct kbase_csf_fw_io *fw_io)
{
	/* Nothing to do. */
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_term);

void kbase_csf_fw_io_global_write(struct kbase_csf_fw_io *fw_io,
				  const struct kbase_csf_global_iface *iface, u32 offset, u32 value)
{
	const struct kbase_device *const kbdev = iface->kbdev;

	lockdep_assert_held(&fw_io->lock);

	dev_dbg(kbdev->dev, "glob input w: reg %08x val %08x\n", offset, value);
	input_page_write(iface->input, offset, value);
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_global_write);

void kbase_csf_fw_io_global_write_mask(struct kbase_csf_fw_io *fw_io,
				       const struct kbase_csf_global_iface *iface, u32 offset,
				       u32 value, u32 mask)
{
	const struct kbase_device *const kbdev = iface->kbdev;

	lockdep_assert_held(&fw_io->lock);

	dev_dbg(kbdev->dev, "glob input w: reg %08x val %08x mask %08x\n", offset, value, mask);
	input_page_partial_write(iface->input, offset, value, mask);
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_global_write_mask);

u32 kbase_csf_fw_io_global_input_read(struct kbase_csf_fw_io *fw_io,
				      const struct kbase_csf_global_iface *iface, u32 offset)
{
	const struct kbase_device *const kbdev = iface->kbdev;
	u32 val;

	lockdep_assert_held(&fw_io->lock);

	val = input_page_read(iface->input, offset);
	dev_dbg(kbdev->dev, "glob input r: reg %08x val %08x\n", offset, val);

	return val;
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_global_input_read);

u32 kbase_csf_fw_io_global_read(struct kbase_csf_fw_io *fw_io,
				const struct kbase_csf_global_iface *iface, u32 offset)
{
	const struct kbase_device *const kbdev = iface->kbdev;
	u32 val;

	lockdep_assert_held(&fw_io->lock);

	val = output_page_read(iface->output, offset);
	dev_dbg(kbdev->dev, "glob output r: reg %08x val %08x\n", offset, val);

	return val;
}

void kbase_csf_fw_io_group_write(struct kbase_csf_fw_io *fw_io,
				 const struct kbase_csf_cmd_stream_group_info *info, u32 offset,
				 u32 value)
{
	const struct kbase_device *const kbdev = info->kbdev;

	lockdep_assert_held(&fw_io->lock);

	dev_dbg(kbdev->dev, "csg input w: reg %08x val %08x\n", offset, value);
	input_page_write(info->input, offset, value);
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_group_write);

void kbase_csf_fw_io_group_write_mask(struct kbase_csf_fw_io *fw_io,
				      const struct kbase_csf_cmd_stream_group_info *info,
				      u32 offset, u32 value, u32 mask)
{
	const struct kbase_device *const kbdev = info->kbdev;

	lockdep_assert_held(&fw_io->lock);

	dev_dbg(kbdev->dev, "csg input w: reg %08x val %08x mask %08x\n", offset, value, mask);
	input_page_partial_write(info->input, offset, value, mask);
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_group_write_mask);

u32 kbase_csf_fw_io_group_input_read(struct kbase_csf_fw_io *fw_io,
				     const struct kbase_csf_cmd_stream_group_info *info, u32 offset)
{
	const struct kbase_device *const kbdev = info->kbdev;
	u32 val;

	lockdep_assert_held(&fw_io->lock);

	val = input_page_read(info->input, offset);
	dev_dbg(kbdev->dev, "csg input r: reg %08x val %08x\n", offset, val);

	return val;
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_group_input_read);

u32 kbase_csf_fw_io_group_read(struct kbase_csf_fw_io *fw_io,
			       const struct kbase_csf_cmd_stream_group_info *info, u32 offset)
{
	const struct kbase_device *const kbdev = info->kbdev;
	u32 val;

	lockdep_assert_held(&fw_io->lock);

	val = output_page_read(info->output, offset);
	dev_dbg(kbdev->dev, "csg output r: reg %08x val %08x\n", offset, val);

	return val;
}

void kbase_csf_fw_io_stream_write(struct kbase_csf_fw_io *fw_io,
				  const struct kbase_csf_cmd_stream_info *info, u32 offset,
				  u32 value)
{
	const struct kbase_device *const kbdev = info->kbdev;

	lockdep_assert_held(&fw_io->lock);

	dev_dbg(kbdev->dev, "cs input w: reg %08x val %08x\n", offset, value);
	input_page_write(info->input, offset, value);
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_stream_write);

void kbase_csf_fw_io_stream_write_mask(struct kbase_csf_fw_io *fw_io,
				       const struct kbase_csf_cmd_stream_info *info, u32 offset,
				       u32 value, u32 mask)
{
	const struct kbase_device *const kbdev = info->kbdev;

	lockdep_assert_held(&fw_io->lock);

	dev_dbg(kbdev->dev, "cs input w: reg %08x val %08x mask %08x\n", offset, value, mask);
	input_page_partial_write(info->input, offset, value, mask);
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_stream_write_mask);

u32 kbase_csf_fw_io_stream_input_read(struct kbase_csf_fw_io *fw_io,
				      const struct kbase_csf_cmd_stream_info *info, u32 offset)
{
	const struct kbase_device *const kbdev = info->kbdev;
	u32 val;

	lockdep_assert_held(&fw_io->lock);

	val = input_page_read(info->input, offset);
	dev_dbg(kbdev->dev, "cs input r: reg %08x val %08x\n", offset, val);

	return val;
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_stream_input_read);

u32 kbase_csf_fw_io_stream_read(struct kbase_csf_fw_io *fw_io,
				const struct kbase_csf_cmd_stream_info *info, u32 offset)
{
	const struct kbase_device *const kbdev = info->kbdev;
	u32 val;

	lockdep_assert_held(&fw_io->lock);

	val = output_page_read(info->output, offset);
	dev_dbg(kbdev->dev, "cs output r: reg %08x val %08x\n", offset, val);

	return val;
}

void kbase_csf_fw_io_set_status(struct kbase_csf_fw_io *fw_io,
				enum kbase_csf_fw_io_status_bits status_bit)
{
	set_bit(status_bit, fw_io->status);
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_set_status);

void kbase_csf_fw_io_clear_status(struct kbase_csf_fw_io *fw_io,
				  enum kbase_csf_fw_io_status_bits status_bit)
{
	clear_bit(status_bit, fw_io->status);
}

bool kbase_csf_fw_io_test_status(struct kbase_csf_fw_io *fw_io,
				 enum kbase_csf_fw_io_status_bits status_bit)
{
	return test_bit(status_bit, fw_io->status);
}
KBASE_EXPORT_TEST_API(kbase_csf_fw_io_test_status);
