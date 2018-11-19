# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the JESD204 core.
#

obj-$(CONFIG_JESD204) += jesd204.o
jesd204-y := jesd204-core.o

obj-$(CONFIG_AXI_JESD204B) += axi_jesd204b_v51.o axi_jesd204b_gt.o

obj-y += linklayer/
obj-y += transceiver/
