# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_DRM_CDNS_DSI) += cdns-dsi.o
cdns-dsi-y := cdns-dsi-core.o
cdns-dsi-$(CONFIG_DRM_CDNS_DSI_J721E) += cdns-dsi-j721e.o
obj-$(CONFIG_DRM_CDNS_MHDP8546) += cdns-mhdp8546.o
cdns-mhdp8546-y := cdns-mhdp8546-core.o cdns-mhdp8546-hdcp.o
cdns-mhdp8546-$(CONFIG_DRM_CDNS_MHDP8546_J721E) += cdns-mhdp8546-j721e.o

cdns_mhdp_drmcore-y := cdns-mhdp-common.o cdns-mhdp-dp.o cdns-mhdp-hdmi.o

cdns_mhdp_drmcore-$(CONFIG_DRM_CDNS_HDMI) += cdns-hdmi-core.o
cdns_mhdp_drmcore-$(CONFIG_DRM_CDNS_DP) += cdns-dp-core.o
cdns_mhdp_drmcore-$(CONFIG_DRM_CDNS_AUDIO) += cdns-mhdp-audio.o
cdns_mhdp_drmcore-$(CONFIG_DRM_CDNS_HDCP) += cdns-mhdp-hdcp.o cdns-hdcp-common.o
cdns_mhdp_drmcore-$(CONFIG_DRM_CDNS_HDMI_CEC) += cdns-mhdp-cec.o

obj-$(CONFIG_DRM_CDNS_MHDP)		+= cdns_mhdp_drmcore.o
