# SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
#
# Makefile for the Analog Devices network device drivers.
#

obj-$(CONFIG_ADIN1110) += adin1110.o

obj-$(CONFIG_ADRV906X_PHY) := adrv906x-phy.o
adrv906x-phy-objs := adrv906x-phy-main.o adrv906x-phy-serdes.o

obj-$(CONFIG_ADRV906X_NET) += adrv906x-eth.o
adrv906x-eth-y := adrv906x-net.o adrv906x-mac.o adrv906x-switch.o adrv906x-ndma.o \
			adrv906x-ethtool.o adrv906x-mdio.o adrv906x-tsu.o adrv906x-cmn.o
adrv906x-eth-$(CONFIG_MACSEC) += adrv906x-macsec-ext.o macsec/cco_macsec.o
