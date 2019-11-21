# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_RPMSG)		+= rpmsg_core.o
obj-$(CONFIG_RPMSG_CHAR)	+= rpmsg_char.o
obj-$(CONFIG_RPMSG_CTRL)	+= rpmsg_ctrl.o
obj-$(CONFIG_RPMSG_NS)		+= rpmsg_ns.o
obj-$(CONFIG_RPMSG_MTK_SCP)	+= mtk_rpmsg.o
CFLAGS_qcom_glink_native.o	:= -I$(src)
qcom_glink-objs			:= qcom_glink_native.o qcom_glink_ssr.o
obj-$(CONFIG_RPMSG_QCOM_GLINK) += qcom_glink.o
obj-$(CONFIG_RPMSG_QCOM_GLINK_RPM) += qcom_glink_rpm.o
obj-$(CONFIG_RPMSG_QCOM_GLINK_SMEM) += qcom_glink_smem.o
obj-$(CONFIG_RPMSG_QCOM_SMD)	+= qcom_smd.o
obj-$(CONFIG_RPMSG_VIRTIO)	+= virtio_rpmsg_bus.o
obj-$(CONFIG_HAVE_IMX_RPMSG)	+= imx_rpmsg.o
obj-$(CONFIG_IMX_RPMSG_PINGPONG)	+= imx_rpmsg_pingpong.o
obj-$(CONFIG_IMX_RPMSG_TTY)	+= imx_rpmsg_tty.o
