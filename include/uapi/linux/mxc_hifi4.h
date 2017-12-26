/*
 * Copyright 2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#define HIFI4_SET_CONFIG		_IOW(HIFI4_IOC_MAGIC, 7, unsigned int)
#define HIFI4_RESET_CODEC		_IOW(HIFI4_IOC_MAGIC, 8, unsigned int)
#define HIFI4_CLIENT_REGISTER		_IOW(HIFI4_IOC_MAGIC, 9, unsigned int)
#define HIFI4_CLIENT_UNREGISTER		_IOW(HIFI4_IOC_MAGIC, 10, unsigned int)

#define CODEC_MP3_DEC		1
#define CODEC_AAC_DEC		2
#define CODEC_DAB_DEC		3
#define CODEC_MP2_DEC		4
#define CODEC_BSAC_DEC		5
#define CODEC_DRM_DEC		6
#define CODEC_SBC_DEC		7
#define CODEC_SBC_ENC		8
#define CODEC_DEMO_DEC		9

enum HIFI_ERROR_TYPE {
	XA_SUCCESS = 0,

	XA_ERROR_STREAM,
	XA_PARA_ERROR,
	XA_INSUFFICIENT_MEM,
	XA_ERR_UNKNOWN,
	XA_PROFILE_NOT_SUPPORT,
	XA_INIT_ERR,
	XA_NO_OUTPUT,

	XA_NOT_ENOUGH_DATA = 0x100,
	XA_CAPIBILITY_CHANGE = 0x200,
	XA_END_OF_STREAM = 0x300, /* no output */
};

/* Parameter type to Set /Get */
enum HIFI_ParaType {
/* Set parmameters */
/* common  */
	XA_SAMPLERATE = 0,
	XA_CHANNEL,
	XA_FRAMED,        /* one whole frame input */
	XA_DEPTH,
	XA_CODEC_DATA,
	XA_BITRATE,
	XA_DOWNMIX_STEREO,
	XA_STREAM_TYPE,
	XA_CHAN_MAP_TABLE,
	//UNIA_CHANNEL_MASK,
	XA_TO_STEREO,

/* dedicate for mp3 dec */
	XA_MP3_DEC_CRC_CHECK = 0x120,
	XA_MP3_DEC_MCH_ENABLE,
	XA_MP3_DEC_NONSTD_STRM_SUPPORT,

/* dedicate for bsac dec */
	XA_BSAC_DEC_DECODELAYERS = 0x130,

/* dedicate for aacplus dec */
	XA_AACPLUS_DEC_BDOWNSAMPLE = 0x140,
	XA_AACPLUS_DEC_BBITSTREAMDOWNMIX,
	XA_AACPLUS_DEC_CHANROUTING,

/* dedicate for dabplus dec */
	XA_DABPLUS_DEC_BDOWNSAMPLE = 0x150,
	XA_DABPLUS_DEC_BBITSTREAMDOWNMIX,
	XA_DABPLUS_DEC_CHANROUTING,

/* dedicate for sbc enc */
	XA_SBC_ENC_SUBBANDS = 0x160,
	XA_SBC_ENC_BLOCKS,
	XA_SBC_ENC_SNR,
	XA_SBC_ENC_BITPOOL,
	XA_SBC_ENC_CHMODE,

};

#define HIFI_STREAM_DABPLUS_BASE  0x30
enum HIFI_StreamType {
    /* AAC/AACPLUS file format */
	XA_STREAM_UNKNOWN = 0,
	XA_STREAM_ADTS,
	XA_STREAM_ADIF,
	XA_STREAM_RAW,

	XA_STREAM_LATM,
	XA_STREAM_LATM_OUTOFBAND_CONFIG,
	XA_STREAM_LOAS,

    /* DABPLUS file format */
	XA_STREAM_DABPLUS_RAW_SIDEINFO = HIFI_STREAM_DABPLUS_BASE,
	XA_STREAM_DABPLUS,

    /* BSAC file raw format */
	XA_STREAM_BSAC_RAW,

};

/* sbc_enc-specific channel modes */
enum HIFI_SbcEncChmode {
	XA_CHMODE_MONO =   0,
	XA_CHMODE_DUAL =   1,
	XA_CHMODE_STEREO = 2,
	XA_CHMODE_JOINT =  3,
};

enum lib_type {
	HIFI_CODEC_LIB = 1,
	HIFI_CODEC_WRAP_LIB,
};

struct decode_info {
	void *in_buf_addr;
	int   in_buf_size;
	int   in_buf_off;
	void *out_buf_addr;
	int   out_buf_size;
	int   out_buf_off;
	unsigned int input_over;
	unsigned int process_id;
};

struct prop_info {
	int   samplerate;
	int   channels;
	int   bits;
	unsigned int consumed_bytes;
	unsigned int cycles;

	unsigned int process_id;
};

struct binary_info {
	int type;
	char *file;
	unsigned int process_id;
	unsigned int lib_type;
};

struct prop_config {
	int   codec_id;  /* codec id */
	int   cmd;       /* command value */
	int   val;       /* parameter value */
	int   ret;       /* executed status of function */
	unsigned int process_id;
};

#endif/* __MXC_HIFI4_UAPI_H__ */
