ccflags-y += -I $(src)/../

imx-dpu-crtc-objs := dpu-crtc.o dpu-kms.o dpu-plane.o
imx-dpu-crtc-$(CONFIG_DEBUG_FS) := dpu-crc.o
obj-$(CONFIG_DRM_IMX_DPU) += imx-dpu-crtc.o
imx-dpu-render-objs := dpu-blit.o
obj-$(CONFIG_DRM_IMX_DPU) += imx-dpu-render.o
