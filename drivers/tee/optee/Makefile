# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_OPTEE) += optee.o
optee-objs += core.o
optee-objs += call.o
optee-objs += notif.o
optee-objs += rpc.o
optee-objs += protmem.o
optee-objs += supp.o
optee-objs += device.o
optee-$(CONFIG_HAVE_ARM_SMCCC) += smc_abi.o
optee-$(CONFIG_ARM_FFA_TRANSPORT) += ffa_abi.o

# for tracing framework to find optee_trace.h
CFLAGS_smc_abi.o := -I$(src)
