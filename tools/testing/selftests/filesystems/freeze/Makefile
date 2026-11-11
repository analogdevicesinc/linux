# SPDX-License-Identifier: GPL-2.0
CFLAGS += -Wall -O2 -g $(KHDR_INCLUDES)
LOCAL_HDRS += loop_util.h

TEST_GEN_PROGS := quotactl_test
TEST_GEN_PROGS += bdev_freeze_test bdev_thaw_test shared_bdev_test

include ../../lib.mk
