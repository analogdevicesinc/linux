/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MFD_ASUS_TRANSFORMER_EC_H
#define __MFD_ASUS_TRANSFORMER_EC_H

#include <linux/notifier.h>
#include <linux/platform_device.h>

#define ASUSEC_ENTRIES			0x100
#define ASUSEC_ENTRY_SIZE		32
#define ASUSEC_ENTRY_BUFSIZE		(ASUSEC_ENTRY_SIZE + 1)

struct i2c_client;

/**
 * struct asusec_core - public part shared with all cells
 *
 * @model: firmware version running on the EC
 * @name: prefix associated with the EC
 * @dockram: pointer to Dockram's i2c_client
 * @notify_list: notify list used by cells
 * @keyboard_mask: mask of keyboard packets
 * @ext_keys_enabled: true if extension keys driver is active
 * @right_alt_pressed: state of Right Alt (AltGr) key
 * @special_combo_triggered: true if AltGr + extension key combo was triggered
 */
struct asusec_core {
	const char *model;
	const char *name;
	struct i2c_client *dockram;
	struct blocking_notifier_head notify_list;
	u8 keyboard_mask;
	bool ext_keys_enabled;
	bool right_alt_pressed;
	bool special_combo_triggered;
};

/* interrupt sources */
#define ASUSEC_IRQ_STATUS		1
#define ASUSEC_OBF_MASK			BIT(0)
#define ASUSEC_KEY_MASK			BIT(2)
#define ASUSEC_KBC_MASK			BIT(3)
#define ASUSEC_AUX_MASK			BIT(5)
#define ASUSEC_SCI_MASK			BIT(6)
#define ASUSEC_SMI_MASK			BIT(7)

/* SMI notification codes */
#define ASUSEC_SMI_CODE			2
#define ASUSEC_SMI_POWER_NOTIFY		0x31	/* USB cable plug event */
#define ASUSEC_SMI_HANDSHAKE		0x50	/* response to ec_req edge */
#define ASUSEC_SMI_WAKE			0x53
#define ASUSEC_SMI_RESET		0x5f
#define ASUSEC_SMI_ADAPTER_EVENT	0x60	/* charger to dock plug event */
#define ASUSEC_SMI_BACKLIGHT_ON		0x63
#define ASUSEC_SMI_AUDIO_DOCK_IN	0x70

#define ASUSEC_SMI_ACTION(code)		(ASUSEC_SMI_MASK | ASUSEC_OBF_MASK | \
					(ASUSEC_SMI_##code << 8))

/* control register [0x0a] layout */
#define ASUSEC_CTL_SIZE			8

/*
 * EC reports power from 40-pin connector in the LSB of the control
 * register.  The following values have been observed (xor 0x02):
 *
 * PAD-ec no-plug  0x40 / PAD-ec DOCK     0x20 / DOCK-ec no-plug 0x40
 * PAD-ec AC       0x25 / PAD-ec DOCK+AC  0x24 / DOCK-ec AC      0x25
 * PAD-ec USB      0x45 / PAD-ec DOCK+USB 0x24 / DOCK-ec USB     0x41
 */

#define ASUSEC_CTL_DIRECT_POWER_SOURCE	BIT_ULL(0)
#define ASUSEC_STAT_CHARGING		BIT_ULL(2)
#define ASUSEC_CTL_FULL_POWER_SOURCE	BIT_ULL(5)
#define ASUSEC_CTL_SUSB_MODE		BIT_ULL(9)
#define ASUSEC_CMD_SUSPEND_S3		BIT_ULL(33)
#define ASUSEC_CTL_TEST_DISCHARGE	BIT_ULL(35)
#define ASUSEC_CMD_SUSPEND_INHIBIT	BIT_ULL(37)
#define ASUSEC_CTL_FACTORY_MODE		BIT_ULL(38)
#define ASUSEC_CTL_KEEP_AWAKE		BIT_ULL(39)
#define ASUSEC_CTL_USB_CHARGE		BIT_ULL(40)
#define ASUSEC_CTL_LED_BLINK		BIT_ULL(40)
#define ASUSEC_CTL_LED_AMBER		BIT_ULL(41)
#define ASUSEC_CTL_LED_GREEN		BIT_ULL(42)
#define ASUSEC_CMD_SWITCH_HDMI		BIT_ULL(56)
#define ASUSEC_CMD_WIN_SHUTDOWN		BIT_ULL(62)

#define ASUSEC_DOCKRAM_INFO_MODEL	0x01
#define ASUSEC_DOCKRAM_INFO_FW		0x02
#define ASUSEC_DOCKRAM_INFO_CFGFMT	0x03
#define ASUSEC_DOCKRAM_INFO_HW		0x04
#define ASUSEC_DOCKRAM_CONTROL		0x0a
#define ASUSEC_DOCKRAM_BATT_CTL		0x14

#define ASUSEC_WRITE_BUF		0x64
#define ASUSEC_READ_BUF			0x6a

int asus_dockram_access_ctl(struct i2c_client *client,
			    u64 *out, u64 mask, u64 xor);

#endif /* __MFD_ASUS_TRANSFORMER_EC_H */
