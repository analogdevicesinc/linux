// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2026 Meta Platforms, Inc. and affiliates. */
#include <test_progs.h>
#include "aggregate_ret_func.skel.h"
#include "aggregate_ret_kfunc.skel.h"

void test_aggregate_ret(void)
{
	RUN_TESTS(aggregate_ret_func);
	RUN_TESTS(aggregate_ret_kfunc);
}
