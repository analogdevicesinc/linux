# SPDX-License-Identifier: GPL-2.0

imxdrm-objs := imx-drm-core.o

obj-$(CONFIG_DRM_IMX) += imxdrm.o

obj-$(CONFIG_DRM_IMX_PARALLEL_DISPLAY) += parallel-display.o
obj-$(CONFIG_DRM_IMX_TVE) += imx-tve.o
obj-$(CONFIG_DRM_IMX_LDB) += imx-ldb.o
obj-$(CONFIG_DRM_IMX8QM_LDB) += imx8qm-ldb.o
obj-$(CONFIG_DRM_IMX8QXP_LDB) += imx8qxp-ldb.o
obj-$(CONFIG_DRM_IMX8MP_LDB) += imx8mp-ldb.o

imx-ipuv3-crtc-objs  := ipuv3-crtc.o ipuv3-plane.o
obj-$(CONFIG_DRM_IMX_IPUV3)	+= imx-ipuv3-crtc.o
obj-$(CONFIG_DRM_IMX_HDMI) += dw_hdmi-imx.o
obj-$(CONFIG_DRM_IMX_DCSS) += dcss/
obj-$(CONFIG_DRM_IMX_LCDC) += lcdc/
