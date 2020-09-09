#
# Makefile for Freesecale PPFE driver
#

ccflags-y +=  -I $(srctree)/$(src)/include  -I $(srctree)/$(src)

obj-$(CONFIG_FSL_PPFE) += pfe.o

pfe-y += pfe_mod.o \
	pfe_hw.o \
	pfe_firmware.o \
	pfe_ctrl.o \
	pfe_hif.o \
	pfe_hif_lib.o\
	pfe_eth.o \
	pfe_sysfs.o \
	pfe_debugfs.o \
	pfe_ls1012a_platform.o \
	pfe_hal.o \
	pfe_cdev.o
