ifeq ($(CONFIG_VIDEO_MXC_IPU_CAMERA),y)
	obj-$(CONFIG_VIDEO_MXC_CAPTURE) += mxc_v4l2_capture.o
	obj-$(CONFIG_MXC_IPU_PRP_VF_SDC) += ipu_prp_vf_sdc.o ipu_prp_vf_sdc_bg.o
	obj-$(CONFIG_MXC_IPU_DEVICE_QUEUE_SDC) += ipu_fg_overlay_sdc.o ipu_bg_overlay_sdc.o
	obj-$(CONFIG_MXC_IPU_PRP_ENC) += ipu_prp_enc.o ipu_still.o
	obj-$(CONFIG_MXC_IPU_CSI_ENC) += ipu_csi_enc.o ipu_still.o
endif

obj-$(CONFIG_VIDEO_MXC_CSI_CAMERA) += mx6s_capture.o
obj-$(CONFIG_MXC_VADC) += mxc_vadc.o
obj-$(CONFIG_MXC_MIPI_CSI) += mxc_mipi_csi.o

# Used for iMX 6QDL
ov5640_camera_int-objs := ov5640.o
obj-$(CONFIG_MXC_CAMERA_OV5640) += ov5640_camera_int.o

# Used for iMX 6UL/ULL/SX/SL/SLL
ov5640_camera_v2-objs := ov5640_v2.o
obj-$(CONFIG_MXC_CAMERA_OV5640_V2) += ov5640_camera_v2.o

ov5642_camera-objs := ov5642.o
obj-$(CONFIG_MXC_CAMERA_OV5642) += ov5642_camera.o

# Used for iMX 6QDL/DQSCM
ov5640_camera_mipi_int-objs := ov5640_mipi.o
obj-$(CONFIG_MXC_CAMERA_OV5640_MIPI) += ov5640_camera_mipi_int.o

# Used for iMX 7D
ov5640_camera_mipi_v2-objs := ov5640_mipi_v2.o
obj-$(CONFIG_MXC_CAMERA_OV5640_MIPI_V2) += ov5640_camera_mipi_v2.o

ov5647_camera_mipi-objs := ov5647_mipi.o
obj-$(CONFIG_MXC_CAMERA_OV5647_MIPI) += ov5647_camera_mipi.o

adv7180_tvin-objs := adv7180.o
obj-$(CONFIG_MXC_TVIN_ADV7180) += adv7180_tvin.o

obj-$(CONFIG_VIDEO_V4L2_MXC_INT_DEVICE) += v4l2-int-device.o
