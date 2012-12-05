/*
 * Copyright (C) 2012 Avionic Design GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/hdmi.h>
#include <linux/string.h>

static void hdmi_infoframe_checksum(void *buffer, size_t size)
{
	u8 *ptr = buffer;
	u8 csum = 0;
	size_t i;

	/* compute checksum */
	for (i = 0; i < size; i++)
		csum += ptr[i];

	ptr[3] = 256 - csum;
}

/**
 * hdmi_avi_infoframe_init() - initialize an HDMI AVI infoframe
 * @frame: HDMI AVI infoframe
 *
 * Returns 0 on success or a negative error code on failure.
 */
int hdmi_avi_infoframe_init(struct hdmi_avi_infoframe *frame)
{
	if (!frame)
		return -EINVAL;

	memset(frame, 0, sizeof(*frame));

	frame->type = HDMI_INFOFRAME_TYPE_AVI;
	frame->version = 2;
	frame->length = 13;

	return 0;
}
EXPORT_SYMBOL(hdmi_avi_infoframe_init);

/**
 * hdmi_avi_infoframe_pack() - write HDMI AVI infoframe to binary buffer
 * @frame: HDMI AVI infoframe
 * @buffer: destination buffer
 * @size: size of buffer
 *
 * Packs the information contained in the @frame structure into a binary
 * representation that can be written into the corresponding controller
 * registers. Also computes the checksum as required by section 5.3.5 of
 * the HDMI 1.4 specification.
 *
 * Returns the number of bytes packed into the binary buffer or a negative
 * error code on failure.
 */
ssize_t hdmi_avi_infoframe_pack(struct hdmi_avi_infoframe *frame, void *buffer,
				size_t size)
{
	u8 *ptr = buffer;
	size_t length;

	if (!frame || !buffer)
		return -EINVAL;

	length = 4 + frame->length;

	if (size < length)
		return -ENOSPC;

	memset(buffer, 0, length);

	ptr[0] = frame->type;
	ptr[1] = frame->version;
	ptr[2] = frame->length;
	ptr[3] = 0; /* checksum */
	ptr[4] = ((frame->colorspace & 0x3) << 5) | (frame->scan_mode & 0x3);

	if (frame->active_info_valid)
		ptr[4] |= BIT(4);

	if (frame->horizontal_bar_valid)
		ptr[4] |= BIT(3);

	if (frame->vertical_bar_valid)
		ptr[4] |= BIT(2);

	ptr[5] = ((frame->colorimetry & 0x3) << 6) |
		 ((frame->picture_aspect & 0x3) << 4) |
		 (frame->active_aspect & 0xf);

	ptr[6] = ((frame->extended_colorimetry & 0x7) << 4) |
		 ((frame->quantization_range & 0x3) << 2) |
		 (frame->nups & 0x3);

	if (frame->itc)
		ptr[6] |= BIT(7);

	ptr[7] = frame->video_code & 0x7f;

	ptr[8] = ((frame->ycc_quantization_range & 0x3) << 6) |
		 ((frame->content_type & 0x3) << 4) |
		 (frame->pixel_repeat & 0xf);

	ptr[9] = frame->top_bar & 0xff;
	ptr[10] = (frame->top_bar >> 8) & 0xff;
	ptr[11] = frame->bottom_bar & 0xff;
	ptr[12] = (frame->bottom_bar >> 8) & 0xff;
	ptr[13] = frame->left_bar & 0xff;
	ptr[14] = (frame->left_bar >> 8) & 0xff;
	ptr[15] = frame->right_bar & 0xff;
	ptr[16] = (frame->right_bar >> 8) & 0xff;

	hdmi_infoframe_checksum(buffer, length);

	return length;
}
EXPORT_SYMBOL(hdmi_avi_infoframe_pack);

/**
 * hdmi_spd_infoframe_init() - initialize an HDMI SPD infoframe
 * @frame: HDMI SPD infoframe
 * @vendor: vendor string
 * @product: product string
 *
 * Returns 0 on success or a negative error code on failure.
 */
int hdmi_spd_infoframe_init(struct hdmi_spd_infoframe *frame,
			    const char *vendor, const char *product)
{
	if (!frame)
		return -EINVAL;

	memset(frame, 0, sizeof(*frame));

	frame->type = HDMI_INFOFRAME_TYPE_SPD;
	frame->version = 1;
	frame->length = 25;

	strncpy(frame->vendor, vendor, sizeof(frame->vendor));
	strncpy(frame->product, product, sizeof(frame->product));

	return 0;
}
EXPORT_SYMBOL(hdmi_spd_infoframe_init);

/**
 * hdmi_spd_infoframe_pack() - write HDMI SPD infoframe to binary buffer
 * @frame: HDMI SPD infoframe
 * @buffer: destination buffer
 * @size: size of buffer
 *
 * Packs the information contained in the @frame structure into a binary
 * representation that can be written into the corresponding controller
 * registers. Also computes the checksum as required by section 5.3.5 of
 * the HDMI 1.4 specification.
 *
 * Returns the number of bytes packed into the binary buffer or a negative
 * error code on failure.
 */
ssize_t hdmi_spd_infoframe_pack(struct hdmi_spd_infoframe *frame, void *buffer,
				size_t size)
{
	u8 *ptr = buffer;
	size_t length, i;

	if (!frame || !buffer)
		return -EINVAL;

	length = 4 + frame->length;

	if (size < length)
		return -ENOSPC;

	memset(buffer, 0, length);

	ptr[0] = frame->type;
	ptr[1] = frame->version;
	ptr[2] = frame->length;
	ptr[3] = 0; /* checksum */

	for (i = 0; i < sizeof(frame->vendor); i++)
		ptr[4 + i] = frame->vendor[i];

	for (i = 0; i < sizeof(frame->product); i++)
		ptr[12 + i] = frame->product[i];

	ptr[26] = frame->sdi;

	hdmi_infoframe_checksum(buffer, length);

	return length;
}
EXPORT_SYMBOL(hdmi_spd_infoframe_pack);

/**
 * hdmi_audio_infoframe_init() - initialize an HDMI audio infoframe
 * @frame: HDMI audio infoframe
 *
 * Returns 0 on success or a negative error code on failure.
 */
int hdmi_audio_infoframe_init(struct hdmi_audio_infoframe *frame)
{
	if (!frame)
		return -EINVAL;

	memset(frame, 0, sizeof(*frame));

	frame->type = HDMI_INFOFRAME_TYPE_AUDIO;
	frame->version = 1;
	frame->length = 10;

	return 0;
}
EXPORT_SYMBOL(hdmi_audio_infoframe_init);

/**
 * hdmi_audio_infoframe_pack() - write HDMI audio infoframe to binary buffer
 * @frame: HDMI audio infoframe
 * @buffer: destination buffer
 * @size: size of buffer
 *
 * Packs the information contained in the @frame structure into a binary
 * representation that can be written into the corresponding controller
 * registers. Also computes the checksum as required by section 5.3.5 of
 * the HDMI 1.4 specification.
 *
 * Returns the number of bytes packed into the binary buffer or a negative
 * error code on failure.
 */
ssize_t hdmi_audio_infoframe_pack(struct hdmi_audio_infoframe *frame,
				  void *buffer, size_t size)
{
	unsigned char channels;
	u8 *ptr = buffer;
	size_t length;

	if (!frame || !buffer)
		return -EINVAL;

	length = 4 + frame->length;

	if (size < length)
		return -ENOSPC;

	memset(buffer, 0, length);

	if (frame->channels >= 2)
		channels = frame->channels - 1;
	else
		channels = 0;

	ptr[0] = frame->type;
	ptr[1] = frame->version;
	ptr[2] = frame->length;
	ptr[3] = 0; /* checksum */

	ptr[4] = ((frame->coding_type & 0xf) << 4) | (channels & 0x7);
	ptr[5] = ((frame->sample_frequency & 0x7) << 2) |
		 (frame->sample_size & 0x3);
	ptr[6] = 0;
	ptr[7] = frame->channel_allocation;
	ptr[8] = (frame->level_shift_value & 0xf) << 3;

	if (frame->downmix_inhibit)
		ptr[8] |= BIT(7);

	hdmi_infoframe_checksum(buffer, length);

	return length;
}
EXPORT_SYMBOL(hdmi_audio_infoframe_pack);

/**
 * hdmi_vendor_infoframe_pack() - write a HDMI vendor infoframe to binary
 *                                buffer
 * @frame: HDMI vendor infoframe
 * @buffer: destination buffer
 * @size: size of buffer
 *
 * Packs the information contained in the @frame structure into a binary
 * representation that can be written into the corresponding controller
 * registers. Also computes the checksum as required by section 5.3.5 of
 * the HDMI 1.4 specification.
 *
 * Returns the number of bytes packed into the binary buffer or a negative
 * error code on failure.
 */
ssize_t hdmi_vendor_infoframe_pack(struct hdmi_vendor_infoframe *frame,
				   void *buffer, size_t size)
{
	u8 *ptr = buffer;
	size_t length;

	if (!frame || !buffer)
		return -EINVAL;

	length = 4 + frame->length;

	if (size < length)
		return -ENOSPC;

	memset(buffer, 0, length);

	ptr[0] = frame->type;
	ptr[1] = frame->version;
	ptr[2] = frame->length;
	ptr[3] = 0; /* checksum */

	memcpy(&ptr[4], frame->data, frame->length);

	hdmi_infoframe_checksum(buffer, length);

	return length;
}
EXPORT_SYMBOL(hdmi_vendor_infoframe_pack);
