/*
 * Copyright 2018 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file insert_startcode.c
 *
 * copyright here may be changed later
 *
 *
 */
#include "insert_startcode.h"
// Global VC1 ID and version
u_int32 uVC1CodecID = 0x10; // Simple = 0x10, Main = 0x11
u_int32 uVC1VersionID = 1;

static int insert_RCV_seqhdr(unsigned char *pHeader, u_int32 *pHeaderLen, unsigned char *src,
		u_int32 nFrameSize, u_int32 nWidth, u_int32 nHeight, int *pNoError)
{
	int nHeaderLen;

	unsigned int nValue;
	unsigned int HdrExtDataLen;
	int i = 0;
	int profile;

	nHeaderLen = RCV_HEADER_LEN;

	//Number of Frames, Header Extension Bit, Codec Version
	nValue = RCV_NUM_FRAMES | RCV_SET_HDR_EXT | RCV_CODEC_VERSION;
	pHeader[i++] = (unsigned char)nValue;
	pHeader[i++] = (unsigned char)(nValue >> 8);
	pHeader[i++] = (unsigned char)(nValue >> 16);
#if 0 //1 ???
	pHeader[i++] = 0xC5;
#else
	pHeader[i++] = (unsigned char)(nValue >> 24);
#endif

	//Header Extension Size
	//ASF Parser gives 5 bytes whereas the VPU expects only 4 bytes, so limiting it
	HdrExtDataLen = 4;
	pHeader[i++] = (unsigned char)HdrExtDataLen;
	pHeader[i++] = (unsigned char)(HdrExtDataLen >> 8);
	pHeader[i++] = (unsigned char)(HdrExtDataLen >> 16);
	pHeader[i++] = (unsigned char)(HdrExtDataLen >> 24);

	profile = (*src)>>4;
	if ((profile != 0) && (profile != 4) && (profile != 12)) {
		//it is reasonable to return error immediately since only one sequence header inserted in whole rcv clip
		*pNoError = 0;
	}
	memcpy(pHeader+i, src, HdrExtDataLen);
	i += HdrExtDataLen;

	//Height
	pHeader[i++] = (unsigned char)nHeight;
	pHeader[i++] = (unsigned char)(((nHeight >> 8) & 0xff));
	pHeader[i++] = (unsigned char)(((nHeight >> 16) & 0xff));
	pHeader[i++] = (unsigned char)(((nHeight >> 24) & 0xff));
	//Width
	pHeader[i++] = (unsigned char)nWidth;
	pHeader[i++] = (unsigned char)(((nWidth >> 8) & 0xff));
	pHeader[i++] = (unsigned char)(((nWidth >> 16) & 0xff));
	pHeader[i++] = (unsigned char)(((nWidth >> 24) & 0xff));

	//Frame Size
	pHeader[i++] = (unsigned char)nFrameSize;
	pHeader[i++] = (unsigned char)(nFrameSize >> 8);
	pHeader[i++] = (unsigned char)(nFrameSize >> 16);
#if 0	//1 ???
	pHeader[i++] = (unsigned char)((nFrameSize >> 24));
#else
	pHeader[i++] = (unsigned char)((nFrameSize >> 24) | 0x80);
#endif

	*pHeaderLen = nHeaderLen;

	return 1;
}

static int insert_RCV_pichdr(unsigned char *pHeader, int *pHeaderLen, unsigned int nInSize)
{
	pHeader[0] = (unsigned char)nInSize;
	pHeader[1] = (unsigned char)(nInSize >> 8);
	pHeader[2] = (unsigned char)(nInSize >> 16);
	pHeader[3] = (unsigned char)(nInSize >> 24);
	*pHeaderLen = 4;

	return 1;
}

/*
 * Byte 0-3: Startcode
 * Byte 4:   Payload length bits[23:16]
 * Byte 5:   Payload length bits[15:8]
 * Byte 6:   0x4e
 * Byte 7:   Payload length bits[7:0]
 * Byte 8:   Codec ID			Non-zero
 * Byte 9:   Codec Version ID		Non-zero
 * Byte 10:  Picture Width bits[15:8]
 * Byte 11:  Picture Width bits[7:0]
 * Byte 12:  0x58
 * Byte 13:  Picture Height bits[15:8]
 * Byte 14:  Picture Height bits[7:0]
 * Byte 15:  0x50
 */

static void insert_payload_header_vc1(u_int8 *dst, u_int32 uScodeType, u_int32 uPayloadSize, u_int32 uWidth, u_int32 uHeight)
{
	// Startcode
	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x01;
	dst[3] = uScodeType;

	// Length
	dst[4] = ((uPayloadSize>>16)&0xff);
	dst[5] = ((uPayloadSize>>8)&0xff);
	dst[6] = 0x4e;
	dst[7] = ((uPayloadSize>>0)&0xff);

	// Codec ID and Version
	dst[8] = uVC1CodecID;
	dst[9] = uVC1VersionID;

	// Width
	dst[10] = ((uWidth>>8)&0xff);
	dst[11] = ((uWidth>>0)&0xff);
	dst[12] = 0x58;

	// Height
	dst[13] = ((uHeight>>8)&0xff);
	dst[14] = ((uHeight>>0)&0xff);
	dst[15] = 0x50;
}

static int VC1CreateNALSeqHeader(unsigned char *pHeader, int *pHeaderLen,
		unsigned char *pCodecPri, int nCodecSize, unsigned int *pData, int nMaxHeader)
{
	int nHeaderLen;
	unsigned char temp[4] = {0x00, 0x00, 0x01, 0x0D};

	nHeaderLen = nCodecSize - 1;
	if ((4+nHeaderLen) > nMaxHeader) {
		nHeaderLen = nMaxHeader - 4;
		vpu_dbg(LVL_ERR, "error: header length %d overrun !!! \r\n", nCodecSize);
	}
	memcpy(pHeader, pCodecPri+1, nHeaderLen);

	if (VC1_IS_NOT_NAL(pData[0])) {
		//insert 0x0000010D at the end of header
		memcpy(pHeader+nHeaderLen, temp, 4);
		nHeaderLen += 4;
	}

	*pHeaderLen = nHeaderLen;

	return 1;
}

static int VC1CreateNalFrameHeader(unsigned char *pHeader, int *pHeaderLen, unsigned int *pInData)
{
	unsigned int VC1Id;

	VC1Id = *pInData;
	if (VC1_IS_NOT_NAL(VC1Id)) {
		//need insert header : special ID
		pHeader[0] = 0x0;
		pHeader[1] = 0x0;
		pHeader[2] = 0x01;
		pHeader[3] = 0x0D;
		*pHeaderLen = 4;
	} else {
		//need not insert header
		//do nothing
		*pHeaderLen = 0;
	}

	return 1;
}

void vp6_scd_sequence_header(unsigned char *buffer, int pic_width, int pic_height)
{
	int Length = 0;

	buffer[0] = 0x00;
	buffer[1] = 0x00;
	buffer[2] = 0x01;
	buffer[3] = 0x31;
	buffer[4] = (Length+12)>>16;
	buffer[5] = (Length+12)>>8;
	buffer[6] = 0x4e;
	buffer[7] = (Length+12);
	buffer[8] = 0x36;
	buffer[9] = 0x1;
	buffer[10] = pic_width>>8;
	buffer[11] = pic_width;
	buffer[12] = 0x58;
	buffer[13] = pic_height>>8;
	buffer[14] = pic_height;
	buffer[15] = 0x50;
}

void vp6_scd_frame_header(unsigned char *buffer, int pic_width, int pic_height, int Length)
{
	buffer[0] = 0x00;
	buffer[1] = 0x00;
	buffer[2] = 0x01;
	buffer[3] = 0x32;
	buffer[4] = (Length+12)>>16;
	buffer[5] = (Length+12)>>8;
	buffer[6] = 0x4e;
	buffer[7] = (Length+12);
	buffer[8] = 0x36;
	buffer[9] = 0x1;
	buffer[10] = pic_width>>8;
	buffer[11] = pic_width;
	buffer[12] = 0x58;
	buffer[13] = pic_height>>8;
	buffer[14] = pic_height;
	buffer[15] = 0x50;
}

void vp8_ivf_sequence_header(unsigned char *buffer, int pic_width, int pic_height)
{
	int Length = 32;

	buffer[0] = 0x44;
	buffer[1] = 0x4b;
	buffer[2] = 0x49;
	buffer[3] = 0x46; //0-3byte signature "DKIF"
	buffer[4] = 0x00;
	buffer[5] = 0x00; //4-5byte version 0
	buffer[6] = Length;
	buffer[7] = Length >> 8; //length of Header
	buffer[8] = 0x56;
	buffer[9] = 0x50;
	buffer[10] = 0x38;
	buffer[11] = 0x30; //VP8 fourcc
	buffer[12] = pic_width;
	buffer[13] = pic_width >> 8;
	buffer[14] = pic_height;
	buffer[15] = pic_height >> 8;
	buffer[16] = 0xe8;
	buffer[17] = 0x03;
	buffer[18] = 0x00;
	buffer[19] = 0x00; //16-19 frame rate
	buffer[20] = 0x01;
	buffer[21] = 0x00;
	buffer[22] = 0x00;
	buffer[23] = 0x00; //20-23 time scale
	buffer[24] = 0xdf;
	buffer[25] = 0xf9;
	buffer[26] = 0x09;
	buffer[27] = 0x00; //24-27 number frames
	//28-31 unused
}

void vp8_ivf_frame_header(unsigned char *buffer, u_int32 FrameSize)
{
	buffer[0] = FrameSize;
	buffer[1] = FrameSize >> 8;
	buffer[2] = FrameSize >> 16;
	buffer[3] = FrameSize >> 24;
	//4-11 timestamp
}

void vp8_scd_sequence_header(unsigned char *buffer, int pic_width, int pic_height)
{
	int Length = 32;

	buffer[0] = 0x00;
	buffer[1] = 0x00;
	buffer[2] = 0x01;
	buffer[3] = 0x31;
	buffer[4] = (Length+12)>>16;
	buffer[5] = (Length+12)>>8;
	buffer[6] = 0x4e;
	buffer[7] = (Length+12);
	buffer[8] = 0x36;
	buffer[9] = 0x1;
	buffer[10] = pic_width>>8;
	buffer[11] = pic_width;
	buffer[12] = 0x58;
	buffer[13] = pic_height>>8;
	buffer[14] = pic_height;
	buffer[15] = 0x50;
}
void vp8_scd_frame_header(unsigned char *buffer, int pic_width, int pic_height, int Length)
{
	buffer[0] = 0x00;
	buffer[1] = 0x00;
	buffer[2] = 0x01;
	buffer[3] = 0x32;
	buffer[4] = (Length+12)>>16;
	buffer[5] = (Length+12)>>8;
	buffer[6] = 0x4e;
	buffer[7] = (Length+12);
	buffer[8] = 0x36;
	buffer[9] = 0x1;
	buffer[10] = pic_width>>8;
	buffer[11] = pic_width;
	buffer[12] = 0x58;
	buffer[13] = pic_height>>8;
	buffer[14] = pic_height;
	buffer[15] = 0x50;
}

static void insert_payload_header_divx(u_int8 *dst, u_int32 uPayloadSize, u_int32 uWidth, u_int32 uHeight)
{
	// Startcode
	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x01;
	dst[3] = 0x32;

	// Length
	dst[4] = ((uPayloadSize>>16)&0xff);
	dst[5] = ((uPayloadSize>>8)&0xff);
	dst[6] = 0x4e;
	dst[7] = ((uPayloadSize>>0)&0xff);

	// Codec ID and Version
	dst[8] = 0x38;
	dst[9] = 0x01;

	// Width
	dst[10] = ((uWidth>>8)&0xff);
	dst[11] = ((uWidth>>0)&0xff);
	dst[12] = 0x58;

	// Height
	dst[13] = ((uHeight>>8)&0xff);
	dst[14] = ((uHeight>>0)&0xff);
	dst[15] = 0x50;
}

static void insert_seq_header_spk(u_int8 *dst, u_int32 uPayloadSize, u_int32 uWidth, u_int32 uHeight)
{
	// Startcode
	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x01;
	dst[3] = 0x31;

	// Length
	dst[4] = ((uPayloadSize>>16)&0xff);
	dst[5] = ((uPayloadSize>>8)&0xff);
	dst[6] = 0x4e;
	dst[7] = ((uPayloadSize>>0)&0xff);

	// Codec ID and Version
	dst[8] = 0x39;
	dst[9] = 0x01;

	// Width
	dst[10] = ((uWidth>>8)&0xff);
	dst[11] = ((uWidth>>0)&0xff);
	dst[12] = 0x58;

	// Height
	dst[13] = ((uHeight>>8)&0xff);
	dst[14] = ((uHeight>>0)&0xff);
	dst[15] = 0x50;
}

static void insert_frame_header_spk(u_int8 *dst, u_int32 uPayloadSize, u_int32 uWidth, u_int32 uHeight)
{
	uPayloadSize = 0;
	// Startcode
	dst[0] = 0x00;
	dst[1] = 0x00;
	dst[2] = 0x01;
	dst[3] = 0x32;

	// Length
	dst[4] = ((uPayloadSize>>16)&0xff);
	dst[5] = ((uPayloadSize>>8)&0xff);
	dst[6] = 0x4e;
	dst[7] = ((uPayloadSize>>0)&0xff);

	// Codec ID and Version
	dst[8] = 0x39;
	dst[9] = 0x01;

	// Width
	dst[10] = ((uWidth>>8)&0xff);
	dst[11] = ((uWidth>>0)&0xff);
	dst[12] = 0x58;

	// Height
	dst[13] = ((uHeight>>8)&0xff);
	dst[14] = ((uHeight>>0)&0xff);
	dst[15] = 0x50;
}

u_int32 insert_scode_4_seq(struct vpu_ctx *ctx, u_int8 *src, u_int8 *dst, u_int32 vdec_std, u_int32 uPayloadSize)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	u_int32 length = 0;

	switch (vdec_std) {
	case VPU_VIDEO_VC1: {
		if (q_data->fourcc == V4L2_PIX_FMT_VC1_ANNEX_G) {
			u_int8 Header[VC1_MAX_SEQ_HEADER_SIZE];
			u_int32 uWidth = q_data->width;
			u_int32 uHeight = q_data->height; //Width & Height in the generic payload header are ignored
			u_int32 FrameSize = 0x60;
			u_int32 HeaderLen, NoError = 1;
			//insert startcode for vc1
			insert_payload_header_vc1(dst, VC1_SCODE_NEW_SEQUENCE, 20, uWidth, uHeight);
			length = 16;
			//insert RCV sequence header for vc1 v1, length=20
			insert_RCV_seqhdr(Header, &HeaderLen, src, FrameSize, uWidth, uHeight, &NoError);
			HeaderLen = RCV_HEADER_LEN - 4;
			memcpy(dst + 16, Header, HeaderLen);
			length += HeaderLen;
		} else {
			u_int8 Header[VC1_MAX_SEQ_HEADER_SIZE];
			u_int32 HeaderLen;

			VC1CreateNALSeqHeader(Header, &HeaderLen, src, uPayloadSize,
					(unsigned int *)src, VC1_MAX_SEQ_HEADER_SIZE);
			if (VC1_IS_NOT_NAL(((unsigned int *)src)[0]))
				HeaderLen -= 4;
			memcpy(dst, Header, HeaderLen);
			length += HeaderLen;
		}
	}

	break;
	case VPU_VIDEO_VP6: {
		vp6_scd_sequence_header(dst, q_data->width, q_data->height);
		length = 16;
	}
	break;
	case VPU_VIDEO_VP8: {
		u_int8 seq_header[32] = {0};
		u_int8 frame_header[8] = {0};

		vp8_scd_sequence_header(dst, q_data->width, q_data->height);
		length = 16;
		vp8_ivf_sequence_header(seq_header, q_data->width, q_data->height);
		memcpy(dst+length, seq_header, 32);
		length += 32;
		vp8_scd_frame_header(dst + length, q_data->width, q_data->height, uPayloadSize + 8);
		length += 16;
		vp8_ivf_frame_header(frame_header, uPayloadSize);
		memcpy(dst+length, frame_header, 8);
		length += 8;
		memcpy(dst+length, src, uPayloadSize);
		length += uPayloadSize;
	}
	break;
	case VPU_VIDEO_ASP: {
		if (q_data->fourcc == VPU_PIX_FMT_DIVX) {
			insert_payload_header_divx(dst, uPayloadSize, q_data->width, q_data->height);
			length = 16;
			memcpy(dst+length, src, uPayloadSize);
			length += uPayloadSize;
		}
	}
	break;
	case VPU_VIDEO_SPK: {
		insert_seq_header_spk(dst, uPayloadSize, q_data->width, q_data->height);
		length = 16;
	}
	break;
	default:
	break;
	}
	return length;
}

u_int32 insert_scode_4_pic(struct vpu_ctx *ctx, u_int8 *dst, u_int8 *src, u_int32 vdec_std, u_int32 uPayloadSize)
{
	struct queue_data *q_data = &ctx->q_data[V4L2_SRC];
	u_int32 length = 0;

	switch (vdec_std) {
	case VPU_VIDEO_VC1: {
		if (q_data->fourcc == V4L2_PIX_FMT_VC1_ANNEX_G) {
			u_int8 Header[VC1_MAX_FRM_HEADER_SIZE];
			u_int32 HeaderLen;
			u_int32 uWidth = q_data->width;
			u_int32 uHeight = q_data->height; //Width & Height in the generic payload header are ignored

			insert_payload_header_vc1(dst, VC1_SCODE_NEW_PICTURE, uPayloadSize + 4, uWidth, uHeight);
			insert_RCV_pichdr(Header, &HeaderLen, uPayloadSize);
			memcpy(dst+16, Header, 4);
			length = 16 + 4;
		} else {
			u_int8 Header[VC1_MAX_FRM_HEADER_SIZE];
			u_int32 HeaderLen;

			VC1CreateNalFrameHeader(Header, (int *)(&HeaderLen), (unsigned int *)(src));
			memcpy(dst, Header, HeaderLen);
			length = HeaderLen;
		}
	}
	break;
	case VPU_VIDEO_VP6: {
		vp6_scd_frame_header(dst, q_data->width, q_data->height, uPayloadSize);
		length = 16;
	}
	break;
	case VPU_VIDEO_VP8: {
		u_int8 frame_header[8];

		vp8_scd_frame_header(dst, q_data->width, q_data->height, uPayloadSize + 8);
		length = 16;
		vp8_ivf_frame_header(frame_header, uPayloadSize);
		memcpy(dst+length, frame_header, 8);
		length += 8;
	}
	break;
	case VPU_VIDEO_ASP: {
		if (q_data->fourcc == VPU_PIX_FMT_DIVX) {
			insert_payload_header_divx(dst, uPayloadSize, q_data->width, q_data->height);
			length = 16;
		}
	}
	break;
	case VPU_VIDEO_SPK: {
		insert_frame_header_spk(dst, uPayloadSize, q_data->width, q_data->height);
		length = 16;
	}
	break;
	default:
	break;
	}
	return length;
}



