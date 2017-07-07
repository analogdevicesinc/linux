/*
 * Copyright 2017 NXP
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @file mxc_hifi4.h
 *
 * @ingroup Audio
 */

#ifndef __MXC_HIFI4_UAPI_H__
#define __MXC_HIFI4_UAPI_H__

#define HIFI4_IOC_MAGIC		'H'
#define HIFI4_LOAD_CODEC		_IOWR(HIFI4_IOC_MAGIC, 0, unsigned int)
#define HIFI4_INIT_CODEC		_IOWR(HIFI4_IOC_MAGIC, 1, unsigned int)
#define HIFI4_CODEC_OPEN		_IOWR(HIFI4_IOC_MAGIC, 2, unsigned int)
#define HIFI4_CODEC_CLOSE		_IOWR(HIFI4_IOC_MAGIC, 3, unsigned int)
#define HIFI4_DECODE_ONE_FRAME		_IOW(HIFI4_IOC_MAGIC, 4, unsigned int)
#define HIFI4_UNLOAD_CODEC		_IOW(HIFI4_IOC_MAGIC, 5, unsigned int)
#define HIFI4_GET_PCM_PROP		_IOW(HIFI4_IOC_MAGIC, 6, unsigned int)
#define HIFI4_SET_CONFIG               _IOW(HIFI4_IOC_MAGIC, 7, unsigned int)

#define CODEC_MP3_DEC		1
#define CODEC_AAC_DEC		2
#define CODEC_DAB_DEC		3
#define CODEC_MP2_DEC		4
#define CODEC_BSAC_DEC		5
#define CODEC_DRM_DEC		6
#define CODEC_SBC_DEC		7
#define CODEC_SBC_ENC		8
#define CODEC_DEMO_DEC		9

struct decode_info {
	void *in_buf_addr;
	int   in_buf_size;
	int   in_buf_off;
	void *out_buf_addr;
	int   out_buf_size;
	int   out_buf_off;
	unsigned int cycles;
	unsigned int input_over;
};

struct prop_info {
	int   samplerate;
	int   channels;
	int   bits;
	unsigned int consumed_bytes;

	/* dedicate for drm dec */
	int   frame_size;

	/* dedicate for bsac, aacplus and dabplus dec */
	int   aac_samplerate;
	int   sbr_type;
	int   mpeg_surr_present;
};

struct binary_info {
	int type;
	char *file;
};

struct prop_config {
	int   codec_id;  /* codec id */
	int   cmd;       /* command value */
	int   val;       /* parameter value */
	int   ret;       /* executed status of function */
};

#endif/* __MXC_HIFI4_UAPI_H__ */
