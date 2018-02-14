#ifndef _DCSS_CRTC_H
#include <linux/hdmi.h>
#include <video/imx-dcss.h>

void dcss_crtc_setup_opipe(struct drm_crtc *crtc, struct drm_connector *conn,
			   u32 colorimetry, u32 eotf,
			   enum hdmi_quantization_range qr);

int dcss_crtc_get_opipe_cfg(struct drm_crtc *crtc,
			    struct dcss_hdr10_pipe_cfg *opipe_cfg);

#endif
