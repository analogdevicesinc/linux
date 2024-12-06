// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "focaltech_core.h"

int fts_check_cid(struct fts_ts_data *ts_data, u8 id_h)
{
	int i = 0;
	struct ft_chip_id_t *cid = &ts_data->ic_info.cid;
	u8 cid_h = 0x0;

	if (cid->type == 0)
		return -ENODATA;

	for (i = 0; i < FTS_MAX_CHIP_IDS; i++) {
		cid_h = ((cid->chip_ids[i] >> 8) & 0x00FF);
		if (cid_h && (id_h == cid_h))
			return 0;
	}

	return -ENODATA;
}

int fts_wait_tp_to_valid(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int cnt = 0;
	u8 idh = 0;
	u8 chip_idh = ts_data->ic_info.ids.chip_idh;

	do {
		ret = fts_read_reg(ts_data, FTS_REG_CHIP_ID, &idh);
		if ((idh == chip_idh) || (fts_check_cid(ts_data, idh) == 0)) {
			dev_info(&ts_data->client->dev, "TP Ready,Device ID:0x%02x", idh);
			return 0;
		}
		dev_dbg(&ts_data->client->dev, "TP Not Ready,ReadData:0x%02x,ret:%d", idh, ret);
		cnt++;
		msleep(INTERVAL_READ_REG_RESUME);
	} while ((cnt * INTERVAL_READ_REG_RESUME) < TIMEOUT_READ_REG);

	return -EIO;
}

void fts_irq_disable(struct fts_ts_data *ts_data)
{
	unsigned long irqflags;

	spin_lock_irqsave(&ts_data->irq_lock, irqflags);

	if (!ts_data->irq_disabled) {
		disable_irq_nosync(ts_data->irq);
		ts_data->irq_disabled = true;
	}

	spin_unlock_irqrestore(&ts_data->irq_lock, irqflags);
}

void fts_irq_enable(struct fts_ts_data *ts_data)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&ts_data->irq_lock, irqflags);

	if (ts_data->irq_disabled) {
		enable_irq(ts_data->irq);
		ts_data->irq_disabled = false;
	}

	spin_unlock_irqrestore(&ts_data->irq_lock, irqflags);
}

void fts_hid2std(struct fts_ts_data *ts_data)
{
	int ret = 0;
	u8 buf[3] = {0xEB, 0xAA, 0x09};

	if (ts_data->bus_type != BUS_TYPE_I2C)
		return;

	ret = fts_write(ts_data, buf, 3);
	if (ret < 0)
		dev_err(&ts_data->client->dev, "hid2std cmd write fail");
	else {
		msleep(FTS_READY_MS);
		buf[0] = buf[1] = buf[2] = 0;
		ret = fts_read(ts_data, NULL, 0, buf, 3);
		if (ret < 0)
			dev_dbg(&ts_data->client->dev, "hid2std cmd read fail");
		else if ((buf[0] == 0xEB) && (buf[1] == 0xAA) && (buf[2] == 0x08))
			dev_dbg(&ts_data->client->dev, "hidi2c change to stdi2c successful");
		else
			dev_dbg(&ts_data->client->dev, "hidi2c change to stdi2c not support or fail");
	}
}

static int fts_get_chip_types(struct fts_ts_data *ts_data, u8 id_h, u8 id_l, bool fw_valid)
{
	u32 i = 0;
	struct ft_chip_t ctype[] = FTS_CHIP_TYPE_MAPPING;
	u32 ctype_entries = sizeof(ctype) / sizeof(struct ft_chip_t);

	if ((id_h == 0x0) || (id_l == 0x0)) {
		dev_err(&ts_data->client->dev, "id_h/id_l is 0");
		return -EINVAL;
	}

	dev_info(&ts_data->client->dev, "verify id:0x%02x%02x", id_h, id_l);
	for (i = 0; i < ctype_entries; i++) {
		if (fw_valid == VALID) {
			if (((id_h == ctype[i].chip_idh) && (id_l == ctype[i].chip_idl)))
				break;
		} else {
			if (((id_h == ctype[i].rom_idh) && (id_l == ctype[i].rom_idl))
				|| ((id_h == ctype[i].pb_idh) && (id_l == ctype[i].pb_idl))
				|| ((id_h == ctype[i].bl_idh) && (id_l == ctype[i].bl_idl))) {
				break;
			}
		}
	}

	if (i >= ctype_entries)
		return -ENODATA;

	ts_data->ic_info.ids = ctype[i];
	return 0;
}

static int fts_read_bootid(struct fts_ts_data *ts_data, u8 *id)
{
	int ret = 0;
	u8 chip_id[2] = { 0 };
	u8 id_cmd[4] = { 0 };
	u32 id_cmd_len = 0;

	id_cmd[0] = FTS_CMD_START1;
	id_cmd[1] = FTS_CMD_START2;
	ret = fts_write(ts_data, id_cmd, 2);
	if (ret < 0) {
		dev_err(&ts_data->client->dev, "start cmd write fail");
		return ret;
	}

	msleep(FTS_CMD_START_DELAY);
	id_cmd[0] = FTS_CMD_READ_ID;
	id_cmd[1] = id_cmd[2] = id_cmd[3] = 0x00;
	id_cmd_len = FTS_CMD_READ_ID_LEN;
	ret = fts_read(ts_data, id_cmd, id_cmd_len, chip_id, 2);
	if ((ret < 0) || (chip_id[0] == 0x0) || (chip_id[1] == 0x0)) {
		dev_err(&ts_data->client->dev, "read boot id fail,read:0x%02x%02x",
				chip_id[0], chip_id[1]);
		return -EIO;
	}

	id[0] = chip_id[0];
	id[1] = chip_id[1];
	return 0;
}

static int fts_get_ic_information(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int cnt = 0;
	u8 chip_id[2] = { 0 };

	ts_data->ic_info.hid_supported = FTS_HID_SUPPORTTED;

	do {
		ret = fts_read_reg(ts_data, FTS_REG_CHIP_ID, &chip_id[0]);
		ret = fts_read_reg(ts_data, FTS_REG_CHIP_ID2, &chip_id[1]);
		if ((ret < 0) || (chip_id[0] == 0x0) || (chip_id[1]) == 0x0) {
			dev_dbg(&ts_data->client->dev, "chip id read invalid, read:0x%02x%02x",
					chip_id[0], chip_id[1]);
		} else {
			ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], VALID);
			if (!ret)
				break;
			dev_dbg(&ts_data->client->dev, "TP not ready, read:0x%02x%02x",
					chip_id[0], chip_id[1]);
		}
		cnt++;
		msleep(INTERVAL_READ_REG);
	} while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

	if ((cnt * INTERVAL_READ_REG) >= TIMEOUT_READ_REG) {
		for (cnt = 0; cnt < 3; cnt++) {
			dev_info(&ts_data->client->dev, "fw is invalid, need read boot id");
			if (ts_data->ic_info.hid_supported)
				fts_hid2std(ts_data);

			ret = fts_read_bootid(ts_data, &chip_id[0]);
			if (ret <  0) {
				dev_err(&ts_data->client->dev, "read boot id fail");
				continue;
			}

			ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], INVALID);
			if (ret < 0) {
				dev_err(&ts_data->client->dev, "can't get ic informaton");
				continue;
			}
			break;
		}
	}

	dev_info(&ts_data->client->dev, "get ic information, chip id = 0x%02x%02x(cid type=0x%x)",
			ts_data->ic_info.ids.chip_idh, ts_data->ic_info.ids.chip_idl,
			ts_data->ic_info.cid.type);

	return ret;
}

static int fts_show_touch_buffer(u8 *data, u32 datalen)
{
	u32 i = 0;
	u32 count = 0;
	char *tmpbuf = NULL;

	tmpbuf = kzalloc(1024, GFP_KERNEL);

	for (i = 0; i < datalen; i++) {
		count += snprintf(tmpbuf + count, 1024 - count, "%02X,", data[i]);
		if (count >= 1024)
			break;
	}
	pr_info("touch_buf:%s", tmpbuf);

	kfree(tmpbuf);
	tmpbuf = NULL;

	return 0;
}

void fts_release_all_finger(struct fts_ts_data *ts_data)
{
	struct input_dev *input_dev = ts_data->input_dev;
	u32 finger_count = 0;
	u32 max_touches = ts_data->pdata->max_touch_number;

	mutex_lock(&ts_data->report_mutex);

	for (finger_count = 0; finger_count < max_touches; finger_count++) {
		input_mt_slot(input_dev, finger_count);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
	}

	input_sync(input_dev);

	ts_data->touch_points = 0;
	mutex_unlock(&ts_data->report_mutex);
}

static int fts_input_report_b(struct fts_ts_data *ts_data, struct ts_event *events)
{
	int i = 0;
	int touch_down_point_cur = 0;
	int touch_point_pre = ts_data->touch_points;
	u32 max_touch_num = ts_data->pdata->max_touch_number;
	bool touch_event_coordinate = false;
	struct input_dev *input_dev = ts_data->input_dev;

	for (i = 0; i < ts_data->touch_event_num; i++) {
		touch_event_coordinate = true;
		if (EVENT_DOWN(events[i].flag)) {
			input_mt_slot(input_dev, events[i].id);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);
			input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, events[i].minor);

			input_report_abs(input_dev, ABS_MT_POSITION_X, events[i].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, events[i].y);

			touch_down_point_cur |= (1 << events[i].id);
			touch_point_pre |= (1 << events[i].id);

			if ((ts_data->log_level >= 2) ||
				((ts_data->log_level == 1) && (events[i].flag == FTS_TOUCH_DOWN))) {
				dev_dbg(&ts_data->client->dev, "[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
						events[i].id, events[i].x, events[i].y,
						events[i].p, events[i].area);
			}
		} else {
			input_mt_slot(input_dev, events[i].id);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
			touch_point_pre &= ~(1 << events[i].id);
			if (ts_data->log_level >= 1)
				dev_dbg(&ts_data->client->dev, "[B]P%d UP!", events[i].id);
		}
	}

	if (unlikely(touch_point_pre ^ touch_down_point_cur)) {
		for (i = 0; i < max_touch_num; i++)  {
			if ((1 << i) & (touch_point_pre ^ touch_down_point_cur)) {
				if (ts_data->log_level >= 1)
					dev_dbg(&ts_data->client->dev, "[B]P%d UP!", i);
				input_mt_slot(input_dev, i);
				input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
			}
		}
	}

	if (touch_down_point_cur)
		input_report_key(input_dev, BTN_TOUCH, 1);
	else if (touch_event_coordinate || ts_data->touch_points) {
		if (ts_data->touch_points && (ts_data->log_level >= 1))
			dev_dbg(&ts_data->client->dev, "[B]Points All Up!");
		input_report_key(input_dev, BTN_TOUCH, 0);
	}

	ts_data->touch_points = touch_down_point_cur;
	input_sync(input_dev);
	return 0;
}

static int fts_read_parse_touchdata(struct fts_ts_data *ts_data, u8 *touch_buf)
{
	int ret = 0;

	memset(touch_buf, 0xFF, FTS_MAX_TOUCH_BUF);
	ts_data->ta_size = ts_data->touch_size;

	/*read touch data*/
	if (ts_data->bus_type == BUS_TYPE_I2C)
		ret = fts_read_touchdata_i2c(ts_data, touch_buf);
	else
		dev_err(&ts_data->client->dev, "unknown bus type:%d", ts_data->bus_type);

	if (ret < 0) {
		dev_err(&ts_data->client->dev, "read touch data fails");
		return TOUCH_ERROR;
	}

	if (ts_data->log_level >= 3)
		fts_show_touch_buffer(touch_buf, ts_data->ta_size);

	if (ret)
		return TOUCH_IGNORE;

	return ((touch_buf[FTS_TOUCH_E_NUM] >> 4) & 0x0F);
}

static int fts_irq_read_report(struct fts_ts_data *ts_data)
{
	int i = 0;
	int max_touch_num = ts_data->pdata->max_touch_number;
	int touch_etype = 0;
	u8 event_num = 0;
	u8 finger_num = 0;
	u8 pointid = 0;
	u8 base = 0;
	u8 *touch_buf = ts_data->touch_buf;
	struct ts_event *events = ts_data->events;

	touch_etype = fts_read_parse_touchdata(ts_data, touch_buf);
	switch (touch_etype) {
	case TOUCH_DEFAULT:
		finger_num = touch_buf[FTS_TOUCH_E_NUM] & 0x0F;
		if (finger_num > max_touch_num) {
			dev_err(&ts_data->client->dev, "invalid point_num(%d)", finger_num);
			return -EIO;
		}

		for (i = 0; i < max_touch_num; i++) {
			base = FTS_ONE_TCH_LEN * i + 2;
			pointid = (touch_buf[FTS_TOUCH_OFF_ID_YH + base]) >> 4;
			if (pointid >= FTS_MAX_ID)
				break;
			else if (pointid >= max_touch_num) {
				dev_err(&ts_data->client->dev,
						"ID(%d) beyond max_touch_number", pointid);
				return -EINVAL;
			}

			events[i].id = pointid;
			events[i].flag = touch_buf[FTS_TOUCH_OFF_E_XH + base] >> 6;
			events[i].x = ((touch_buf[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 8)
						+ (touch_buf[FTS_TOUCH_OFF_XL + base] & 0xFF);
			events[i].y = ((touch_buf[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 8)
						+ (touch_buf[FTS_TOUCH_OFF_YL + base] & 0xFF);
			events[i].p =  touch_buf[FTS_TOUCH_OFF_PRE + base];
			events[i].area = touch_buf[FTS_TOUCH_OFF_AREA + base];
			if (events[i].p <= 0)
				events[i].p = 0x3F;
			if (events[i].area <= 0)
				events[i].area = 0x09;
			events[i].minor = events[i].area;

			event_num++;
			if (EVENT_DOWN(events[i].flag) && (finger_num == 0)) {
				dev_info(&ts_data->client->dev, "abnormal touch data from fw");
				return -EIO;
			}
		}

		if (event_num == 0) {
			dev_info(&ts_data->client->dev,
					"no touch point information(%02x)", touch_buf[2]);
			return -EIO;
		}
		ts_data->touch_event_num = event_num;

		mutex_lock(&ts_data->report_mutex);
		fts_input_report_b(ts_data, events);
		mutex_unlock(&ts_data->report_mutex);
		break;

#if FTS_TOUCH_HIRES_EN
	case TOUCH_DEFAULT_HI_RES:
		finger_num = touch_buf[FTS_TOUCH_E_NUM] & 0x0F;
		if (finger_num > max_touch_num) {
			dev_err(&ts_data->client->dev, "invalid point_num(%d)", finger_num);
			return -EIO;
		}

		for (i = 0; i < max_touch_num; i++) {
			base = FTS_ONE_TCH_LEN * i + 2;
			pointid = (touch_buf[FTS_TOUCH_OFF_ID_YH + base]) >> 4;
			if (pointid >= FTS_MAX_ID)
				break;
			else if (pointid >= max_touch_num) {
				dev_err(&ts_data->client->dev,
						"ID(%d) beyond max_touch_number", pointid);
				return -EINVAL;
			}

			events[i].id = pointid;
			events[i].flag = touch_buf[FTS_TOUCH_OFF_E_XH + base] >> 6;
			events[i].x = ((touch_buf[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 12)
					+ ((touch_buf[FTS_TOUCH_OFF_XL + base] & 0xFF) << 4)
					+ ((touch_buf[FTS_TOUCH_OFF_PRE + base] >> 4) & 0x0F);
			events[i].y = ((touch_buf[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 12)
					+ ((touch_buf[FTS_TOUCH_OFF_YL + base] & 0xFF) << 4)
					+ (touch_buf[FTS_TOUCH_OFF_PRE + base] & 0x0F);
			events[i].x = (events[i].x * FTS_TOUCH_HIRES_X) / FTS_HI_RES_X_MAX;
			events[i].y = (events[i].y * FTS_TOUCH_HIRES_X) / FTS_HI_RES_X_MAX;
			events[i].area = touch_buf[FTS_TOUCH_OFF_AREA + base];
			events[i].p = 0x3F;
			if (events[i].area <= 0)
				events[i].area = 0x09;
			events[i].minor = events[i].area;
			event_num++;
			if (EVENT_DOWN(events[i].flag) && (finger_num == 0)) {
				dev_info(&ts_data->client->dev, "abnormal touch data from fw");
				return -EIO;
			}
		}

		if (event_num == 0) {
			dev_info(&ts_data->client->dev,
					"no touch point information(%02x)", touch_buf[2]);
			return -EIO;
		}
		ts_data->touch_event_num = event_num;

		mutex_lock(&ts_data->report_mutex);
		fts_input_report_b(ts_data, events);
		mutex_unlock(&ts_data->report_mutex);
		break;
#endif

	case TOUCH_PROTOCOL_v2:
		event_num = touch_buf[FTS_TOUCH_E_NUM] & 0x0F;
		if (!event_num || (event_num > max_touch_num)) {
			dev_err(&ts_data->client->dev, "invalid touch event num(%d)", event_num);
			return -EIO;
		}

		ts_data->touch_event_num = event_num;

		for (i = 0; i < event_num; i++) {
			base = FTS_ONE_TCH_LEN_V2 * i + 4;
			pointid = (touch_buf[FTS_TOUCH_OFF_ID_YH + base]) >> 4;
			if (pointid >= max_touch_num) {
				dev_err(&ts_data->client->dev, "touch point ID(%d) beyond max_touch_number(%d)",
						pointid, max_touch_num);
				return -EINVAL;
			}

			events[i].id = pointid;
			events[i].flag = touch_buf[FTS_TOUCH_OFF_E_XH + base] >> 6;

			events[i].x = ((touch_buf[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 12)
					+ ((touch_buf[FTS_TOUCH_OFF_XL + base] & 0xFF) << 4)
					+ ((touch_buf[FTS_TOUCH_OFF_PRE + base] >> 4) & 0x0F);

			events[i].y = ((touch_buf[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 12)
					+ ((touch_buf[FTS_TOUCH_OFF_YL + base] & 0xFF) << 4)
					+ (touch_buf[FTS_TOUCH_OFF_PRE + base] & 0x0F);

			events[i].x = events[i].x  / FTS_HI_RES_X_MAX;
			events[i].y = events[i].y  / FTS_HI_RES_X_MAX;
			events[i].area = touch_buf[FTS_TOUCH_OFF_AREA + base];
			events[i].minor = touch_buf[FTS_TOUCH_OFF_MINOR + base];
			events[i].p = 0x3F;

			if (events[i].area <= 0)
				events[i].area = 0x09;
			if (events[i].minor <= 0)
				events[i].minor = 0x09;

		}

		mutex_lock(&ts_data->report_mutex);
		fts_input_report_b(ts_data, events);
		mutex_unlock(&ts_data->report_mutex);

		break;

	case TOUCH_EXTRA_MSG:
		if (!ts_data->touch_analysis_support) {
			dev_err(&ts_data->client->dev, "touch_analysis is disabled");
			return -EINVAL;
		}

		event_num = touch_buf[FTS_TOUCH_E_NUM] & 0x0F;
		if (!event_num || (event_num > max_touch_num)) {
			dev_err(&ts_data->client->dev, "invalid touch event num(%d)", event_num);
			return -EIO;
		}

		ts_data->touch_event_num = event_num;
		for (i = 0; i < event_num; i++) {
			base = FTS_ONE_TCH_LEN * i + 4;
			pointid = (touch_buf[FTS_TOUCH_OFF_ID_YH + base]) >> 4;
			if (pointid >= max_touch_num) {
				dev_err(&ts_data->client->dev, "touch point ID(%d) beyond max_touch_number(%d)",
						pointid, max_touch_num);
				return -EINVAL;
			}

			events[i].id = pointid;
			events[i].flag = touch_buf[FTS_TOUCH_OFF_E_XH + base] >> 6;
			events[i].x = ((touch_buf[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 8)
						+ (touch_buf[FTS_TOUCH_OFF_XL + base] & 0xFF);
			events[i].y = ((touch_buf[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 8)
						+ (touch_buf[FTS_TOUCH_OFF_YL + base] & 0xFF);
			events[i].p =  touch_buf[FTS_TOUCH_OFF_PRE + base];
			events[i].area = touch_buf[FTS_TOUCH_OFF_AREA + base];
			if (events[i].p <= 0)
				events[i].p = 0x3F;
			if (events[i].area <= 0)
				events[i].area = 0x09;
			events[i].minor = events[i].area;
		}

		mutex_lock(&ts_data->report_mutex);
		fts_input_report_b(ts_data, events);
		mutex_unlock(&ts_data->report_mutex);
		break;

	case TOUCH_IGNORE:
	case TOUCH_ERROR:
		break;

	default:
		dev_info(&ts_data->client->dev, "unknown touch event(%d)", touch_etype);
		break;
	}

	return 0;
}

static irqreturn_t fts_irq_handler(int irq, void *data)
{
	struct fts_ts_data *ts_data = data;
	int ret = 0;

	if ((ts_data->suspended) && (ts_data->pm_suspend)) {
		ret = wait_for_completion_timeout(
				&ts_data->pm_completion,
				msecs_to_jiffies(FTS_TIMEOUT_COMERR_PM));
		if (!ret) {
			dev_err(&ts_data->client->dev, "Bus don't resume from pm(deep),timeout,skip irq");
			return IRQ_HANDLED;
		}
	}

	ts_data->intr_jiffies = jiffies;
	fts_irq_read_report(ts_data);
	if (ts_data->touch_analysis_support && ts_data->ta_flag) {
		ts_data->ta_flag = 0;
		if (ts_data->ta_buf && ts_data->ta_size)
			memcpy(ts_data->ta_buf, ts_data->touch_buf, ts_data->ta_size);
		wake_up_interruptible(&ts_data->ts_waitqueue);
	}

	return IRQ_HANDLED;
}

static int fts_irq_registration(struct fts_ts_data *ts_data)
{
	int ret = 0;
	struct fts_ts_platform_data *pdata = ts_data->pdata;

	ts_data->irq = gpio_to_irq(pdata->irq_gpio);
	pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	dev_info(&ts_data->client->dev, "irq:%d, flag:%x", ts_data->irq, pdata->irq_gpio_flags);
	ret = request_threaded_irq(ts_data->irq, NULL, fts_irq_handler,
							pdata->irq_gpio_flags,
							FTS_DRIVER_NAME, ts_data);
	return ret;
}

static int fts_input_init(struct fts_ts_data *ts_data)
{
	int ret = 0;
	struct fts_ts_platform_data *pdata = ts_data->pdata;
	struct input_dev *input_dev;
	u32 touch_x_max = pdata->x_max;
	u32 touch_y_max = pdata->y_max;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&ts_data->client->dev, "Failed to allocate memory for input device");
		return -ENOMEM;
	}

	/* Init and register Input device */
	input_dev->name = FTS_DRIVER_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = ts_data->dev;

	input_set_drvdata(input_dev, ts_data);

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#if FTS_TOUCH_HIRES_EN
	touch_x_max = (pdata->x_max + 1) * FTS_TOUCH_HIRES_X - 1;
	touch_y_max = (pdata->y_max + 1) * FTS_TOUCH_HIRES_X - 1;
#endif

	input_mt_init_slots(input_dev, pdata->max_touch_number, INPUT_MT_DIRECT);

	input_set_capability(input_dev, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(input_dev, EV_ABS, ABS_MT_POSITION_Y);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, touch_x_max - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, touch_y_max - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 0xFF, 0, 0);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&ts_data->client->dev, "Input device registration failed");
		input_set_drvdata(input_dev, NULL);
		input_free_device(input_dev);
		input_dev = NULL;
		return ret;
	}

	ts_data->input_dev = input_dev;
	return 0;
}

static int fts_buffer_init(struct fts_ts_data *ts_data)
{
	ts_data->touch_buf = kzalloc(FTS_MAX_TOUCH_BUF, GFP_KERNEL);
	if (!ts_data->touch_buf)
		return -ENOMEM;

	if (ts_data->bus_type == BUS_TYPE_I2C)
		ts_data->touch_size = FTS_SIZE_DEFAULT_V2;
	else
		dev_err(&ts_data->client->dev, "unknown bus type:%d", ts_data->bus_type);

	ts_data->touch_analysis_support = 0;
	ts_data->ta_flag = 0;
	ts_data->ta_size = 0;

	return 0;
}

static int fts_gpio_configure(struct fts_ts_data *ts_data)
{
	int ret = 0;

	/* request irq gpio */
	if (gpio_is_valid(ts_data->pdata->irq_gpio)) {
		ret = gpio_request(ts_data->pdata->irq_gpio, "fts_irq_gpio");
		if (ret) {
			dev_err(&ts_data->client->dev, "[GPIO]irq gpio request failed");
			goto err_irq_gpio_req;
		}

		ret = gpio_direction_input(ts_data->pdata->irq_gpio);
		if (ret) {
			dev_err(&ts_data->client->dev, "[GPIO]set_direction for irq gpio failed");
			goto err_irq_gpio_dir;
		}
	}

	return 0;

err_irq_gpio_dir:
	if (gpio_is_valid(ts_data->pdata->irq_gpio))
		gpio_free(ts_data->pdata->irq_gpio);
err_irq_gpio_req:
	return ret;
}

static int fts_bus_init(struct fts_ts_data *ts_data)
{
	ts_data->bus_tx_buf = kzalloc(FTS_MAX_BUS_BUF, GFP_KERNEL);
	if (ts_data->bus_tx_buf == NULL)
		return -ENOMEM;

	ts_data->bus_rx_buf = kzalloc(FTS_MAX_BUS_BUF, GFP_KERNEL);
	if (ts_data->bus_rx_buf == NULL)
		return -ENOMEM;

	return 0;
}

static int fts_parse_dt(struct device *dev, struct fts_ts_platform_data *pdata)
{
	int ret = 0;
	struct device_node *np = dev->of_node;
	u32 temp_val = 0;

	if (!np || !pdata) {
		dev_err(dev, "np/pdata is null");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "touchscreen-size-x", &pdata->x_max);
	if (ret < 0) {
		dev_err(dev, "Unable to read property 'touchscreen-size-x'");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "touchscreen-size-y", &pdata->y_max);
	if (ret < 0) {
		dev_err(dev, "Unable to read property 'touchscreen-size-y'");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "max-touch-number", &temp_val);
	if (ret < 0) {
		dev_err(dev, "Unable to get max-touch-number, use default setting");
		pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
	} else {
		if (temp_val < 2)
			pdata->max_touch_number = 2; /* max_touch_number must >= 2 */
		else if (temp_val > FTS_MAX_POINTS_SUPPORT)
			pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
		else
			pdata->max_touch_number = temp_val;
	}

	pdata->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (!gpio_is_valid(pdata->irq_gpio)) {
		dev_err(dev, "irq-gpios not set\n");
		return -EINVAL;
	}

	pdata->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (!pdata->reset_gpio) {
		dev_info(dev, "reset-gpio not set\n");
	} else {
		ret = PTR_ERR_OR_ZERO(pdata->reset_gpio);
		if (ret) {
			dev_err(dev, "Failed to get reset gpio (%d)\n", ret);
			return ret;
		}
	}

	dev_info(dev, "max touch number:%d, irq gpio:%d",
			pdata->max_touch_number, pdata->irq_gpio);

	return 0;
}

int fts_ts_suspend(struct fts_ts_data *ts_data)
{
	int ret = 0;

	if (ts_data->suspended) {
		dev_info(&ts_data->client->dev, "Already in suspend state");
		return 0;
	}

	fts_irq_disable(ts_data);

	dev_info(&ts_data->client->dev, "make TP enter into sleep mode");
	ret = fts_write_reg(ts_data, FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP);
	if (ret < 0)
		dev_err(&ts_data->client->dev, "set TP to sleep mode fail, ret=%d", ret);

	fts_release_all_finger(ts_data);
	ts_data->suspended = true;
	return 0;
}

static void fts_reset_proc(struct fts_ts_data *ts_data, int hdelayms)
{
	if (ts_data->pdata->reset_gpio) {
		gpiod_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
		msleep(hdelayms);
	}
}

int fts_ts_resume(struct fts_ts_data *ts_data)
{
	if (!ts_data->suspended) {
		dev_dbg(&ts_data->client->dev, "Already in awake state");
		return 0;
	}
	fts_reset_proc(ts_data, FTS_RESET_MS);

	ts_data->suspended = false;
	fts_release_all_finger(ts_data);

	fts_wait_tp_to_valid(ts_data);
	fts_irq_enable(ts_data);

	return 0;
}

static void fts_suspend_work(struct work_struct *work)
{
	struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data, suspend_work);

	fts_ts_suspend(ts_data);
}

static void fts_resume_work(struct work_struct *work)
{
	struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data, resume_work);

	fts_ts_resume(ts_data);
}

static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *v)
{
	struct fts_ts_data *ts_data = container_of(self, struct fts_ts_data, fb_notif);
	int blank_value;

	if (ts_data && v) {
		blank_value = *((int *)(((struct fb_event *)v)->data));
		dev_info(&ts_data->client->dev, "notifier,event:%lu,blank:%d", event, blank_value);
		if ((blank_value == FB_BLANK_UNBLANK) && (event == FB_EVENT_BLANK))
			queue_work(ts_data->ts_workqueue, &ts_data->resume_work);
	} else {
		dev_err(&ts_data->client->dev, "ts_data/v is null");
		return -EINVAL;
	}

	return 0;
}

static int fts_notifier_callback_init(struct fts_ts_data *ts_data)
{
	int ret = 0;

	dev_info(&ts_data->client->dev, "init notifier with fb_register_client");
	ts_data->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts_data->fb_notif);
	if (ret)
		dev_err(&ts_data->client->dev, "[FB]Unable to register fb_notifier: %d", ret);
	return ret;
}

static int fts_notifier_callback_exit(struct fts_ts_data *ts_data)
{
	if (fb_unregister_client(&ts_data->fb_notif))
		dev_err(&ts_data->client->dev, "[FB]Error occurred while unregistering fb_notifier.");
	return 0;
}

int fts_ts_probe_entry(struct fts_ts_data *ts_data)
{
	int ret = 0;

	pr_info("%s", FTS_DRIVER_VERSION);
	ts_data->pdata = kzalloc(sizeof(struct fts_ts_platform_data), GFP_KERNEL);
	if (!ts_data->pdata) {
		dev_err(&ts_data->client->dev, "allocate memory for platform_data fail");
		return -ENOMEM;
	}

	ret = fts_parse_dt(ts_data->dev, ts_data->pdata);
	if (ret) {
		dev_err(&ts_data->client->dev, "device-tree parse fail");
		goto err_parse_dt;
	}

	ts_data->ts_workqueue = create_singlethread_workqueue("fts_wq");
	if (!ts_data->ts_workqueue)
		dev_err(&ts_data->client->dev, "create fts workqueue fail");

	spin_lock_init(&ts_data->irq_lock);
	mutex_init(&ts_data->report_mutex);
	mutex_init(&ts_data->bus_lock);
	init_waitqueue_head(&ts_data->ts_waitqueue);

	ret = fts_bus_init(ts_data);
	if (ret) {
		dev_err(&ts_data->client->dev, "bus initialize fail");
		goto err_bus_init;
	}

	ret = fts_input_init(ts_data);
	if (ret) {
		dev_err(&ts_data->client->dev, "input initialize fail");
		goto err_input_init;
	}

	ret = fts_buffer_init(ts_data);
	if (ret) {
		dev_err(&ts_data->client->dev, "buffer init fail");
		goto err_buffer_init;
	}

	ret = fts_gpio_configure(ts_data);
	if (ret) {
		dev_err(&ts_data->client->dev, "configure the gpios fail");
		goto err_gpio_config;
	}

	fts_reset_proc(ts_data, FTS_RESET_MS);

	ret = fts_get_ic_information(ts_data);
	if (ret) {
		dev_err(&ts_data->client->dev, "not focal IC, unregister driver");
		goto err_irq_req;
	}

	ret = fts_irq_registration(ts_data);
	if (ret) {
		dev_err(&ts_data->client->dev, "request irq failed");
		goto err_irq_req;
	}

	if (ts_data->ts_workqueue) {
		INIT_WORK(&ts_data->resume_work, fts_resume_work);
		INIT_WORK(&ts_data->suspend_work, fts_suspend_work);
	}

	init_completion(&ts_data->pm_completion);
	ts_data->pm_suspend = false;

	ret = fts_notifier_callback_init(ts_data);
	if (ret)
		dev_err(&ts_data->client->dev, "init notifier callback fail");

	return 0;

err_irq_req:
	if (gpio_is_valid(ts_data->pdata->irq_gpio))
		gpio_free(ts_data->pdata->irq_gpio);
err_gpio_config:
	kfree_safe(ts_data->touch_buf);
err_buffer_init:
	input_unregister_device(ts_data->input_dev);
err_input_init:
err_bus_init:
	if (ts_data->ts_workqueue)
		destroy_workqueue(ts_data->ts_workqueue);
	kfree_safe(ts_data->bus_tx_buf);
	kfree_safe(ts_data->bus_rx_buf);
err_parse_dt:
	kfree_safe(ts_data->pdata);

	return ret;
}

int fts_ts_remove_entry(struct fts_ts_data *ts_data)
{
	cancel_work_sync(&ts_data->resume_work);
	cancel_work_sync(&ts_data->suspend_work);

	free_irq(ts_data->irq, ts_data);

	input_unregister_device(ts_data->input_dev);

	if (ts_data->ts_workqueue)
		destroy_workqueue(ts_data->ts_workqueue);

	fts_notifier_callback_exit(ts_data);

	if (gpio_is_valid(ts_data->pdata->irq_gpio))
		gpio_free(ts_data->pdata->irq_gpio);

	kfree_safe(ts_data->touch_buf);
	kfree_safe(ts_data->bus_tx_buf);
	kfree_safe(ts_data->bus_rx_buf);
	kfree_safe(ts_data->pdata);

	return 0;
}
