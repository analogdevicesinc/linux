/* SPDX-License-Identifier: GPL-2.0 OR Linux-OpenIB */
/* Copyright (c) 2024 NVIDIA Corporation & Affiliates */

#ifndef MLX5HWS_INTERNAL_H_
#define MLX5HWS_INTERNAL_H_

#include <linux/mlx5/transobj.h>
#include <linux/mlx5/vport.h>
#include "fs_core.h"
#include "wq.h"
#include "lib/mlx5.h"

#include "mlx5hws_prm.h"
#include "mlx5hws.h"
#include "mlx5hws_pool.h"
#include "mlx5hws_vport.h"
#include "mlx5hws_context.h"
#include "mlx5hws_table.h"
#include "mlx5hws_send.h"
#include "mlx5hws_rule.h"
#include "mlx5hws_cmd.h"
#include "mlx5hws_action.h"
#include "mlx5hws_definer.h"
#include "mlx5hws_matcher.h"
#include "mlx5hws_debug.h"
#include "mlx5hws_pat_arg.h"
#include "mlx5hws_bwc.h"
#include "mlx5hws_bwc_complex.h"

#define W_SIZE		2
#define DW_SIZE		4
#define BITS_IN_BYTE	8
#define BITS_IN_DW	(BITS_IN_BYTE * DW_SIZE)

#define IS_BIT_SET(_value, _bit) ((_value) & (1ULL << (_bit)))

#define mlx5hws_err(ctx, arg...) mlx5_core_err((ctx)->mdev, ##arg)
#define mlx5hws_info(ctx, arg...) mlx5_core_info((ctx)->mdev, ##arg)
#define mlx5hws_dbg(ctx, arg...) mlx5_core_dbg((ctx)->mdev, ##arg)

#define MLX5HWS_TABLE_TYPE_BASE 2
#define MLX5HWS_ACTION_STE_IDX_ANY 0

static inline bool is_mem_zero(const u8 *mem, size_t size)
{
	if (unlikely(!size)) {
		pr_warn("HWS: invalid buffer of size 0 in %s\n", __func__);
		return true;
	}

	return (*mem == 0) && memcmp(mem, mem + 1, size - 1) == 0;
}

static inline unsigned long align(unsigned long val, unsigned long align)
{
	return (val + align - 1) & ~(align - 1);
}

#endif /* MLX5HWS_INTERNAL_H_ */
