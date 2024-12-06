# SPDX-License-Identifier: GPL-2.0
imx-media-common-objs := imx-media-capture.o imx-media-dev-common.o \
	imx-media-of.o imx-media-utils.o

imx6-media-objs := imx-media-dev.o imx-media-internal-sd.o \
	imx-ic-common.o imx-ic-prp.o imx-ic-prpencvf.o imx-media-vdic.o \
	imx-media-csc-scaler.o

imx6-media-csi-objs := imx-media-csi.o imx-media-fim.o

imx8-capture-objs := imx8-isi-core.o

imx8-isi-capture-objs := imx8-isi-fmt.o imx8-isi-cap.o
imx8-isi-mem2mem-objs := imx8-isi-fmt.o imx8-isi-m2m.o
obj-$(CONFIG_VIDEO_IMX_MEDIA) += imx-media-common.o
obj-$(CONFIG_VIDEO_IMX_MEDIA) += imx6-media.o
obj-$(CONFIG_VIDEO_IMX_MEDIA) += imx6-media-csi.o
obj-$(CONFIG_VIDEO_IMX_MEDIA) += imx6-mipi-csi2.o

obj-$(CONFIG_IMX8_MIPI_CSI2_SAM)  += imx8-mipi-csi2-sam.o
obj-y += hdmirx/

obj-$(CONFIG_IMX8_CAPTURE_DRIVER) += imx8-capture.o
obj-$(CONFIG_IMX8_ISI_CORE) += imx8-capture.o
obj-$(CONFIG_IMX8_ISI_CAPTURE) 	  += imx8-isi-capture.o
obj-$(CONFIG_IMX8_ISI_M2M)	+= imx8-isi-mem2mem.o
obj-$(CONFIG_IMX8_ISI_HW) 	  += imx8-isi-hw.o
obj-$(CONFIG_GMSL_MAX9286)	  += gmsl-max9286.o
obj-$(CONFIG_IMX8_MEDIA_DEVICE)   += imx8-media-dev.o
