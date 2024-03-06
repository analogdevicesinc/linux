/* SPDX-License-Identifier: GPL-2.0-only */

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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>

/*register address*/
#define FTS_REG_INT_CNT						0x8F
#define FTS_REG_FLOW_WORK_CNT				0x91
#define FTS_REG_WORKMODE					0x00
#define FTS_REG_WORKMODE_FACTORY_VALUE		0x40
#define FTS_REG_WORKMODE_WORK_VALUE			0x00
#define FTS_REG_ESDCHECK_DISABLE			0x8D
#define FTS_REG_CHIP_ID						0xA3
#define FTS_REG_CHIP_ID2					0x9F
#define FTS_REG_POWER_MODE					0xA5
#define FTS_REG_POWER_MODE_SLEEP			0x03
#define FTS_REG_FW_VER						0xA6
#define FTS_REG_VENDOR_ID					0xA8
#define FTS_REG_LCD_BUSY_NUM				0xAB
#define FTS_REG_FACE_DEC_MODE_EN			0xB0
#define FTS_REG_FACTORY_MODE_DETACH_FLAG	0xB4
#define FTS_REG_FACE_DEC_MODE_STATUS		0x01
#define FTS_REG_IDE_PARA_VER_ID				0xB5
#define FTS_REG_IDE_PARA_STATUS				0xB6
#define FTS_REG_GLOVE_MODE_EN				0xC0
#define FTS_REG_COVER_MODE_EN				0xC1
#define FTS_REG_CHARGER_MODE_EN				0x8B
#define FTS_REG_GESTURE_EN					0xD0
#define FTS_REG_GESTURE_OUTPUT_ADDRESS		0xD3
#define FTS_REG_MODULE_ID					0xE3
#define FTS_REG_LIC_VER						0xE4
#define FTS_REG_ESD_SATURATE				0xED

#define FTS_DRIVER_VERSION					"FocalTech V4.1 20230424"
#define _FT3518								0x35180481
#define FTS_CHIP_TYPE	_FT3518
#define FTS_DRIVER_NAME						"fts_ft3518u"
#define FTS_MAX_POINTS_SUPPORT				10 /* constant value, can't be changed */
#define FTS_MAX_KEYS						4
#define FTS_KEY_DIM							10
#define FTS_COORDS_ARR_SIZE					4
#define FTS_ONE_TCH_LEN						6
#define FTS_ONE_TCH_LEN_V2					8
#define FTS_TOUCH_DATA_LEN_V2	(FTS_MAX_POINTS_SUPPORT * FTS_ONE_TCH_LEN_V2 + 4)
#define FTS_TOUCH_DATA_LEN	(FTS_MAX_POINTS_SUPPORT * FTS_ONE_TCH_LEN + 2)
#define FTS_SIZE_DEFAULT					15
#define FTS_SIZE_DEFAULT_V2					21
#define FTS_MAX_ID							0x0A
#define FTS_TOUCH_OFF_E_XH					0
#define FTS_TOUCH_OFF_XL					1
#define FTS_TOUCH_OFF_ID_YH					2
#define FTS_TOUCH_OFF_YL					3
#define FTS_TOUCH_OFF_PRE					4
#define FTS_TOUCH_OFF_AREA					5
#define FTS_TOUCH_OFF_MINOR					6
#define FTS_TOUCH_E_NUM						1
#define FTS_TOUCH_DOWN						0
#define FTS_TOUCH_UP						1
#define FTS_TOUCH_CONTACT					2
#define EVENT_DOWN(flag)	((flag == FTS_TOUCH_DOWN) || (flag == FTS_TOUCH_CONTACT))
#define EVENT_UP(flag)		(flag == FTS_TOUCH_UP)
#define FTS_MAX_COMPATIBLE_TYPE				8
#define FTS_MAX_COMMMAND_LENGTH				16
#define FTS_MAX_TOUCH_BUF					4096
#define FTS_MAX_BUS_BUF						4096
#define FTS_TIMEOUT_COMERR_PM				700
#define FTS_TOUCH_HIRES_EN					0
#define FTS_TOUCH_HIRES_X					10
#define FTS_HI_RES_X_MAX					16

#define FLAGBIT(x)				(0x00000001 << (x))
#define FLAGBITS(x, y)			((0xFFFFFFFF >> (32 - (y) - 1)) & (0xFFFFFFFF << (x)))
#define FLAG_HID_BIT			10
#define FTS_HID_SUPPORTTED		((FTS_CHIP_TYPE & FLAGBIT(FLAG_HID_BIT))  \
								== FLAGBIT(FLAG_HID_BIT))
#define FTS_MAX_CHIP_IDS		8
#define FTS_CHIP_TYPE_MAPPING	{{0x81, 0x54, 0x52, 0x54, 0x52, 0x00, 0x00, 0x54, 0x5C}}
#define ENABLE								1
#define DISABLE								0
#define VALID								1
#define INVALID								0
#define FTS_CMD_START1						0x55
#define FTS_CMD_START2						0xAA
#define FTS_CMD_START_DELAY					12
#define FTS_CMD_READ_ID						0x90
#define FTS_CMD_READ_ID_LEN					4
#define FTS_CMD_READ_ID_LEN_INCELL			1
#define FTS_CMD_READ_INFO					0x5E

#define INTERVAL_READ_REG			200  /* unit:ms */
#define INTERVAL_READ_REG_RESUME	50  /* unit:ms */
#define TIMEOUT_READ_REG			1000 /* unit:ms */
#define FTS_READY_MS				10
#define FTS_RESET_MS				200
#define kfree_safe(pbuf) do {\
	kfree(pbuf);\
	pbuf = NULL;\
} while (0)

struct ft_chip_t {
	u16 type;
	u8 chip_idh;
	u8 chip_idl;
	u8 rom_idh;
	u8 rom_idl;
	u8 pb_idh;
	u8 pb_idl;
	u8 bl_idh;
	u8 bl_idl;
};

struct ft_chip_id_t {
	u16 type;
	u16 chip_ids[FTS_MAX_CHIP_IDS];
};

struct ts_ic_info {
	bool is_incell;
	bool hid_supported;
	struct ft_chip_t ids;
	struct ft_chip_id_t cid;
};

struct fts_ts_platform_data {
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 key_number;
	u32 keys[FTS_MAX_KEYS];
	u32 key_y_coords[FTS_MAX_KEYS];
	u32 key_x_coords[FTS_MAX_KEYS];
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 max_touch_number;
	struct gpio_desc *reset_gpio;
};

struct ts_event {
	int x;      /*x coordinate */
	int y;      /*y coordinate */
	int p;      /* pressure */
	int flag;   /* touch event flag: 0 -- down; 1-- up; 2 -- contact */
	int id;     /*touch ID */
	int area;
	int minor;
};

struct fts_ts_data {
	struct i2c_client *client;
	struct device *dev;
	struct input_dev *input_dev;
	struct fts_ts_platform_data *pdata;
	struct ts_ic_info ic_info;
	struct workqueue_struct *ts_workqueue;
	struct work_struct resume_work;
	struct work_struct suspend_work;
	wait_queue_head_t ts_waitqueue;
	spinlock_t irq_lock;
	struct mutex report_mutex;
	struct mutex bus_lock;
	unsigned long intr_jiffies;
	int irq;
	int log_level;
	int dummy_byte;
	struct completion pm_completion;
	bool pm_suspend;
	bool suspended;
	bool irq_disabled;
	bool glove_mode;
	bool cover_mode;
	bool charger_mode;
	bool touch_analysis_support;

	struct ts_event events[FTS_MAX_POINTS_SUPPORT];    /* multi-touch */
	u8 touch_addr;
	u32 touch_size;
	u8 *touch_buf;
	int touch_event_num;
	int touch_points;
	int key_state;
	int ta_flag;
	u32 ta_size;
	u8 *ta_buf;

	u8 *bus_tx_buf;
	u8 *bus_rx_buf;
	int bus_type;
	int bus_ver;

	struct notifier_block fb_notif;
};

enum _FTS_BUS_TYPE {
	BUS_TYPE_NONE,
	BUS_TYPE_I2C,
};

enum _FTS_BUS_VER {
	BUS_VER_DEFAULT = 1,
	BUS_VER_V2,
};

enum _FTS_TOUCH_ETYPE {
	TOUCH_DEFAULT = 0x00,
	TOUCH_PROTOCOL_v2 = 0x02,
	TOUCH_EXTRA_MSG = 0x08,
	TOUCH_DEFAULT_HI_RES = 0x82,
	TOUCH_IGNORE = 0xFE,
	TOUCH_ERROR = 0xFF,
};

/* communication interface */
int fts_read(struct fts_ts_data *ts_data, u8 *cmd, u32 cmdlen, u8 *data, u32 datalen);
int fts_read_reg(struct fts_ts_data *ts_data, u8 addr, u8 *value);
int fts_write(struct fts_ts_data *ts_data, u8 *writebuf, u32 writelen);
int fts_write_reg(struct fts_ts_data *ts_data, u8 addr, u8 value);
void fts_hid2std(struct fts_ts_data *ts_data);
int fts_ts_probe_entry(struct fts_ts_data *ts_data);
int fts_ts_remove_entry(struct fts_ts_data *ts_data);
int fts_read_touchdata_i2c(struct fts_ts_data *ts_data, u8 *buf);
/* Other */
int fts_check_cid(struct fts_ts_data *ts_data, u8 id_h);
int fts_wait_tp_to_valid(struct fts_ts_data *ts_data);
void fts_release_all_finger(struct fts_ts_data *ts_data);

void fts_irq_disable(struct fts_ts_data *ts_data);
void fts_irq_enable(struct fts_ts_data *ts_data);
int fts_ts_suspend(struct fts_ts_data *ts_data);
int fts_ts_resume(struct fts_ts_data *ts_data);
