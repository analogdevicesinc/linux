#
# Makefile for the linux kernel.
#

obj-y					:= socfpga.o
obj-$(CONFIG_SMP)	+= headsmp.o platsmp.o
obj-$(CONFIG_SOCFPGA_SUSPEND)	+= pm.o self-refresh.o
obj-$(CONFIG_HW_PERF_EVENTS) += socfpga_cti.o
