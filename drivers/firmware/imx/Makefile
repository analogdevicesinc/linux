# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_IMX_DSP)		+= imx-dsp.o
obj-$(CONFIG_IMX_SCU)		+= imx-scu.o misc.o imx-scu-irq.o rm.o imx-scu-soc.o seco.o
obj-${CONFIG_IMX_SCMI_MISC_DRV}	+= sm-misc.o
sec_enclave-objs		= se_ctrl.o ele_common.o ele_base_msg.o ele_fw_api.o
obj-${CONFIG_IMX_SEC_ENCLAVE}	+= sec_enclave.o
sec_enclave-${CONFIG_IMX_ELE_TRNG} += ele_trng.o
