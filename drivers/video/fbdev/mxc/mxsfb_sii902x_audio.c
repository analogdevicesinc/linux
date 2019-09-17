// SPDX-License-Identifier: GPL-2.0
// Copyright 2019 NXP

#include <linux/clk.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/of_device.h>
#include <sound/hdmi-codec.h>
#include <drm/drm_edid.h>

#include "mxsfb_sii902x.h"

static const unsigned int audio_rates[] = {
	32000,
	44100,
	48000,
	88200,
	96000,
	176400,
	192000,
};

/*
 * HDMI audio codec callbacks
 */
static int sii902x_audio_hw_params(struct device *dev, void *data,
				    struct hdmi_codec_daifmt *daifmt,
				    struct hdmi_codec_params *params)
{
	struct sii902x_data *sii902x = dev_get_drvdata(dev);
	unsigned char reg;

	/* sii90sx hdmi audio setup */
	i2c_smbus_write_byte_data(sii902x->client, 0x26, 0x90);
	i2c_smbus_write_byte_data(sii902x->client, 0x20, 0x2d);
	i2c_smbus_write_byte_data(sii902x->client, 0x1f, 0x88);
	i2c_smbus_write_byte_data(sii902x->client, 0x1f, 0x91);
	i2c_smbus_write_byte_data(sii902x->client, 0x1f, 0xa2);
	i2c_smbus_write_byte_data(sii902x->client, 0x1f, 0xb3);
	i2c_smbus_write_byte_data(sii902x->client, 0x27, 0);

	switch (params->sample_rate) {
	case 44100:
		reg = 0;
		break;
	case 48000:
		reg = 0x2;
		break;
	case 32000:
		reg = 0x3;
		break;
	case 88200:
		reg = 0x8;
		break;
	case 96000:
		reg = 0xa;
		break;
	case 176400:
		reg = 0xc;
		break;
	case 192000:
		reg = 0xe;
		break;
	default:
		reg = 0x1;
		break;
	}

	i2c_smbus_write_byte_data(sii902x->client, 0x24, reg);
	i2c_smbus_write_byte_data(sii902x->client, 0x25, 0x0b);
	i2c_smbus_write_byte_data(sii902x->client, 0x26, 0x80);

	return 0;
}

static void sii902x_audio_shutdown(struct device *dev, void *data)
{
	struct sii902x_data *sii902x = dev_get_drvdata(dev);

	i2c_smbus_write_byte_data(sii902x->client, 0x26, 0x10);
}


/* Most of these function is copy from the drivers/gpu/drm/drm_edid.c */
typedef void detailed_cb(struct detailed_timing *timing, void *closure);
static void
cea_for_detailed_block(u8 *ext, detailed_cb *cb, void *closure)
{
	int i, n = 0;
	u8 d = ext[0x02];
	u8 *det_base = ext + d;

	n = (127 - d) / 18;
	for (i = 0; i < n; i++)
		cb((struct detailed_timing *)(det_base + 18 * i), closure);
}

static void
vtb_for_detailed_block(u8 *ext, detailed_cb *cb, void *closure)
{
	unsigned int i, n = min_t(int, ext[0x02], 6);
	u8 *det_base = ext + 5;

	if (ext[0x01] != 1)
		return; /* unknown version */

	for (i = 0; i < n; i++)
		cb((struct detailed_timing *)(det_base + 18 * i), closure);
}

#define EDID_DETAILED_TIMINGS 4
#define AUDIO_BLOCK	0x01
#define VIDEO_BLOCK     0x02
#define VENDOR_BLOCK    0x03
#define SPEAKER_BLOCK	0x04

static void
eld_for_detailed_block(u8 *raw_edid, detailed_cb *cb, void *closure)
{
	int i;
	struct edid *edid = (struct edid *)raw_edid;

	if (edid == NULL)
		return;

	for (i = 0; i < EDID_DETAILED_TIMINGS; i++)
		cb(&(edid->detailed_timings[i]), closure);

	for (i = 1; i <= raw_edid[0x7e]; i++) {
		u8 *ext = raw_edid + (i * EDID_LENGTH);

		switch (*ext) {
		case CEA_EXT:
			cea_for_detailed_block(ext, cb, closure);
			break;
		case VTB_EXT:
			vtb_for_detailed_block(ext, cb, closure);
			break;
		default:
			break;
		}
	}
}

static void
monitor_name(struct detailed_timing *t, void *data)
{
	if (t->data.other_data.type == EDID_DETAIL_MONITOR_NAME)
		*(u8 **)data = t->data.other_data.data.str.str;
}

static int get_monitor_name(struct edid *edid, char name[13])
{
	char *edid_name = NULL;
	int mnl;

	if (!edid || !name)
		return 0;

	eld_for_detailed_block((u8 *)edid, monitor_name, &edid_name);
	for (mnl = 0; edid_name && mnl < 13; mnl++) {
		if (edid_name[mnl] == 0x0a)
			break;

		name[mnl] = edid_name[mnl];
	}

	return mnl;
}

static int
cea_revision(const u8 *cea)
{
	return cea[1];
}

static int
cea_db_offsets(const u8 *cea, int *start, int *end)
{
	/* Data block offset in CEA extension block */
	*start = 4;
	*end = cea[2];
	if (*end == 0)
		*end = 127;
	if (*end < 4 || *end > 127)
		return -ERANGE;
	return 0;
}

static int
cea_db_payload_len(const u8 *db)
{
	return db[0] & 0x1f;
}

static int
cea_db_tag(const u8 *db)
{
	return db[0] >> 5;
}

#define HDMI_IEEE_OUI 0x000c03
static bool cea_db_is_hdmi_vsdb(const u8 *db)
{
	int hdmi_id;

	if (cea_db_tag(db) != VENDOR_BLOCK)
		return false;

	if (cea_db_payload_len(db) < 5)
		return false;

	hdmi_id = db[1] | (db[2] << 8) | (db[3] << 16);

	return hdmi_id == HDMI_IEEE_OUI;
}

#define for_each_cea_db(cea, i, start, end) \
	for ((i) = (start); (i) < (end) && (i) + cea_db_payload_len(&(cea)[(i)]) < (end); (i) += cea_db_payload_len(&(cea)[(i)]) + 1)

static u8 *find_cea_extension(const struct edid *edid, int ext_id)
{
	u8 *edid_ext = NULL;
	int i;

	/* No EDID or EDID extensions */
	if (edid == NULL || edid->extensions == 0)
		return NULL;

	/* Find CEA extension */
	for (i = 0; i < edid->extensions; i++) {
		edid_ext = (u8 *)edid + EDID_LENGTH * (i + 1);
		if (edid_ext[0] == ext_id)
			break;
	}

	if (i == edid->extensions)
		return NULL;

	return edid_ext;
}

static void edid_to_eld(char *eld, struct edid *edid)
{
	u8 *cea;
	u8 *db;
	int total_sad_count = 0;
	int mnl;
	int dbl;

	if (!edid)
		return;

	cea = find_cea_extension(edid, CEA_EXT);
	if (!cea)
		return;

	mnl = get_monitor_name(edid, &eld[DRM_ELD_MONITOR_NAME_STRING]);

	eld[DRM_ELD_CEA_EDID_VER_MNL] = cea[1] << DRM_ELD_CEA_EDID_VER_SHIFT;
	eld[DRM_ELD_CEA_EDID_VER_MNL] |= mnl;

	eld[DRM_ELD_VER] = DRM_ELD_VER_CEA861D;

	eld[DRM_ELD_MANUFACTURER_NAME0] = edid->mfg_id[0];
	eld[DRM_ELD_MANUFACTURER_NAME1] = edid->mfg_id[1];
	eld[DRM_ELD_PRODUCT_CODE0] = edid->prod_code[0];
	eld[DRM_ELD_PRODUCT_CODE1] = edid->prod_code[1];

	if (cea_revision(cea) >= 3) {
		int i, start, end;

		if (cea_db_offsets(cea, &start, &end)) {
			start = 0;
			end = 0;
		}

		for_each_cea_db(cea, i, start, end) {
			db = &cea[i];
			dbl = cea_db_payload_len(db);

			switch (cea_db_tag(db)) {
				int sad_count;

			case AUDIO_BLOCK:
				/* Audio Data Block, contains SADs */
				sad_count = min(dbl / 3, 15 - total_sad_count);
				if (sad_count >= 1)
					memcpy(&eld[DRM_ELD_CEA_SAD(mnl, total_sad_count)],
					       &db[1], sad_count * 3);
				total_sad_count += sad_count;
				break;
			case SPEAKER_BLOCK:
				/* Speaker Allocation Data Block */
				if (dbl >= 1)
					eld[DRM_ELD_SPEAKER] = db[1];
				break;
			case VENDOR_BLOCK:
				/* HDMI Vendor-Specific Data Block */
				if (cea_db_is_hdmi_vsdb(db)) {
					u8 len = cea_db_payload_len(db);

					if (len >= 6 && (db[6] & (1 << 7)))
						eld[DRM_ELD_SAD_COUNT_CONN_TYPE] |= DRM_ELD_SUPPORTS_AI;

				}
				break;
			default:
				break;
			}
		}
	}

	eld[DRM_ELD_SAD_COUNT_CONN_TYPE] |= total_sad_count << DRM_ELD_SAD_COUNT_SHIFT;

	eld[DRM_ELD_SAD_COUNT_CONN_TYPE] |= DRM_ELD_CONN_TYPE_HDMI;

	eld[DRM_ELD_BASELINE_ELD_LEN] =
		DIV_ROUND_UP(drm_eld_calc_baseline_block_size(eld), 4);
}

static int sii902x_audio_get_eld(struct device *dev, void *data, uint8_t *buf, size_t len)
{
	struct sii902x_data *sii902x = dev_get_drvdata(dev);
	uint8_t eld[128];

	memset(eld, 0, 128);

	edid_to_eld(eld, (struct edid *)sii902x->edid);

	memcpy(buf, eld, min(sizeof(eld), len));

	return 0;
}

static const struct hdmi_codec_ops sii902x_audio_codec_ops = {
	.hw_params = sii902x_audio_hw_params,
	.audio_shutdown = sii902x_audio_shutdown,
	.get_eld = sii902x_audio_get_eld,
};

void sii902x_register_audio_driver(struct device *dev)
{
	struct hdmi_codec_pdata codec_data = {
		.ops = &sii902x_audio_codec_ops,
		.max_i2s_channels = 8,
		.i2s = 1,
	};
	struct platform_device *pdev;

	pdev = platform_device_register_data(dev, HDMI_CODEC_DRV_NAME,
					     1, &codec_data,
					     sizeof(codec_data));
	if (IS_ERR(pdev))
		return;
}
