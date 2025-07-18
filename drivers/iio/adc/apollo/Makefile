# SPDX-License-Identifier: GPL-2.0

ad9088_drv-y = ad9088.o
ad9088_drv-y += ad9088_dt.o ad9088_fft.o ad9088_ffh.o
ad9088_drv-y += public/src/adi_apollo_adc.o
ad9088_drv-y += public/src/adi_apollo_dac.o
ad9088_drv-y += public/src/adi_apollo_fddc.o
ad9088_drv-y += public/src/adi_apollo_trigts.o
ad9088_drv-y += public/src/adi_apollo_hal_regio_hsci.o
ad9088_drv-y += public/src/adi_apollo_serdes.o
ad9088_drv-y += public/src/adi_apollo_rx.o
ad9088_drv-y += public/src/adi_apollo_arm.o
ad9088_drv-y += public/src/adi_apollo_jrx.o
ad9088_drv-y += public/src/adi_apollo_txen.o
ad9088_drv-y += public/src/adi_apollo_mailbox.o
ad9088_drv-y += public/src/adi_apollo_gpio.o
ad9088_drv-y += public/src/adi_apollo_cddc.o
ad9088_drv-y += public/src/adi_apollo_fsrc.o
ad9088_drv-y += public/src/adi_apollo_hal_regio_spi.o
ad9088_drv-y += public/src/adi_apollo_cfg.o
ad9088_drv-y += public/src/adi_apollo_dformat.o
ad9088_drv-y += public/src/adi_apollo_jtx.o
ad9088_drv-y += public/src/adi_apollo_hal.o
ad9088_drv-y += public/src/adi_apollo_mailbox_handler.o
ad9088_drv-y += public/src/adi_apollo_clk_mcs.o
ad9088_drv-y += public/src/adi_apollo_cfir.o
ad9088_drv-y += public/src/adi_apollo_fnco.o
ad9088_drv-y += public/src/adi_apollo_rxen.o
ad9088_drv-y += public/src/adi_apollo_bmem.o
ad9088_drv-y += public/src/adi_apollo_txmisc.o
ad9088_drv-y += public/src/adi_apollo_tmode.o
ad9088_drv-y += public/src/adi_apollo_reconfig.o
ad9088_drv-y += public/src/adi_apollo_device.o
ad9088_drv-y += public/src/adi_apollo_cduc.o
ad9088_drv-y += public/src/adi_apollo_txmux.o
ad9088_drv-y += public/src/adi_apollo_rxmux.o
ad9088_drv-y += public/src/adi_apollo_pfilt.o
ad9088_drv-y += public/src/adi_apollo_invsinc.o
ad9088_drv-y += public/src/adi_apollo_rxmisc.o
ad9088_drv-y += public/src/adi_apollo_sniffer.o
ad9088_drv-y += public/src/adi_apollo_sysclk_cond.o
ad9088_drv-y += public/src/adi_apollo_smon.o
ad9088_drv-y += public/src/adi_apollo_fduc.o
ad9088_drv-y += public/src/adi_apollo_tx.o
ad9088_drv-y += public/src/adi_apollo_cnco.o
ad9088_drv-y += private/src/adi_apollo_private_sysclk_cond.o
ad9088_drv-y += public/src/adi_apollo_mcs_cal.o
ad9088_drv-y += adi_utils/src/adi_utils.o
ad9088_drv-y += private/src/adi_apollo_private_bmem.o
ad9088_drv-y += private/src/adi_apollo_private_device.o
ad9088_drv-y += public/src/adi_apollo_bmem_deprecated.o
ad9088_drv-y += public/src/adi_apollo_gpio_hop.o
ad9088_drv-y += public/src/adi_apollo_loopback.o
ad9088_drv-y += public/src/adi_apollo_lb0.o
ad9088_drv-y += public/src/adi_apollo_utils.o
ad9088_drv-y += private/src/adi_apollo_private_blk_sel.o
ad9088_drv-y += public/src/adi_apollo_startup.o

obj-$(CONFIG_AD9088) += ad9088_drv.o

ccflags-y += -I$(srctree)/$(src)/public/inc/ \
	-I$(srctree)/$(src)/private/inc/ \
	-I$(srctree)/$(src)/adi_inc/ \
	-I$(srctree)/$(src)/adi_utils/inc/

ccflags-y += -DB0 -I$(srctree)/$(src)/private/inc/bitfields/b0/
