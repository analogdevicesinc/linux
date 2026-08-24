// SPDX-License-Identifier: MIT
/*
 * Copyright 2025 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include "ras.h"
#include "ras_mp1.h"
#include "core_status.h"
#include "ras_mp1_v13_0.h"

static int __send_mp1_msg32(struct ras_core_context *ras_core,
		enum ras_mp1_msg_id msg_id, u32 input, u32 *output)
{
	if (!ras_core->ras_mp1.sys_func ||
	    !ras_core->ras_mp1.sys_func->mp1_send_ras_msg)
		return -EOPNOTSUPP;

	return ras_core->ras_mp1.sys_func->mp1_send_ras_msg(ras_core,
				msg_id, &input, 1, output, output ? 1 : 0);
}

static int __dump_mp1_bank_reg64(struct ras_core_context *ras_core,
				     u32 msg, u32 idx, u32 reg_idx, u64 *val)
{
	u32 data[2] = {0, 0};
	u32 param;
	int ret;
	u32 i, offset;

	offset = reg_idx * 8;
	for (i = 0; i < ARRAY_SIZE(data); i++) {
		param = ((idx & 0xffff) << 16) | ((offset + (i << 2)) & 0xfffc);
		ret = __send_mp1_msg32(ras_core, msg, param, &data[i]);
		if (ret) {
			RAS_DEV_ERR(ras_core->dev,
				"ACA failed to read register[%u], offset:0x%x\n",
				reg_idx, offset);
			return ret;
		}
	}

	*val = ((u64)data[1] << 32) | data[0];

	return 0;
}

static int mp1_v13_0_get_bank_count(struct ras_core_context *ras_core,
			    enum ras_err_type type, u32 *count)
{
	u32 bank_count = 0;
	u32 msg;
	int ret;

	if (!count)
		return -EINVAL;

	switch (type) {
	case RAS_ERR_TYPE__UE:
		msg = RAS_MP1_MSG_QueryValidMcaCount;
		break;
	case RAS_ERR_TYPE__CE:
	case RAS_ERR_TYPE__DE:
		msg = RAS_MP1_MSG_QueryValidMcaCeCount;
		break;
	default:
		return -EINVAL;
	}

	ret = __send_mp1_msg32(ras_core, msg, 0, &bank_count);
	if (!ret) {
		if (((type == RAS_ERR_TYPE__UE) && (bank_count >= MAX_UE_BANKS_PER_QUERY)) ||
			((type == RAS_ERR_TYPE__CE || type == RAS_ERR_TYPE__DE) &&
			 (bank_count >= MAX_CE_BANKS_PER_QUERY)))
			return -EINVAL;

		*count = bank_count;
	}

	return ret;
}

static int mp1_v13_0_dump_bank(struct ras_core_context *ras_core,
			enum ras_err_type type, u32 idx, u64 *regs, u32 regs_sz)
{
	int i, ret, reg_cnt;
	u32 msg;

	if (!regs || !regs_sz || (idx > 0xffff))
		return -EINVAL;

	switch (type) {
	case RAS_ERR_TYPE__UE:
		msg = RAS_MP1_MSG_McaBankDumpDW;
		break;
	case RAS_ERR_TYPE__CE:
	case RAS_ERR_TYPE__DE:
		msg = RAS_MP1_MSG_McaBankCeDumpDW;
		break;
	default:
		return -EINVAL;
	}

	reg_cnt = min_t(int, 16, regs_sz);
	for (i = 0; i < reg_cnt; i++) {
		ret = __dump_mp1_bank_reg64(ras_core, msg, idx, i, &regs[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int mp1_v13_0_set_debug_mode(struct ras_core_context *ras_core, bool enable)
{
	struct ras_mp1 *mp1 = &ras_core->ras_mp1;
	const struct ras_mp1_sys_func *sys_func = mp1->sys_func;

	if (!sys_func || !sys_func->mp1_set_debug_mode)
		return -RAS_CORE_NOT_SUPPORTED;

	return sys_func->mp1_set_debug_mode(ras_core, enable);
}


const struct ras_mp1_ip_func mp1_ras_func_v13_0 = {
	.get_valid_bank_count = mp1_v13_0_get_bank_count,
	.dump_valid_bank = mp1_v13_0_dump_bank,
	.set_debug_mode = mp1_v13_0_set_debug_mode,
};
