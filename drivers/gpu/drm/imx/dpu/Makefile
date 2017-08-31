ccflags-y += -Idrivers/gpu/drm/imx

imx-dpu-crtc-objs := dpu-crtc.o dpu-plane.o dpu-kms.o
obj-$(CONFIG_DRM_IMX_DPU) += imx-dpu-crtc.o

imx-dpu-render-objs := dpu-blit.o
obj-$(CONFIG_DRM_IMX_DPU) += imx-dpu-render.o
