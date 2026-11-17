// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/array_size.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/mfd/asus-transformer-ec.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define ASUSEC_EXT_KEY_CODES		0x20

struct asus_ec_keys_data {
	struct notifier_block nb;
	struct asusec_core *ec;
	struct input_dev *xidev;
	unsigned short keymap[ASUSEC_EXT_KEY_CODES * 2];
	bool special_key_mode;
};

static const unsigned short asus_ec_dock_ext_keys[] = {
	/* Function keys [0x00 - 0x19] */
	[0x01] = KEY_DELETE,
	[0x02] = KEY_F1,
	[0x03] = KEY_F2,
	[0x04] = KEY_F3,
	[0x05] = KEY_F4,
	[0x06] = KEY_F5,
	[0x07] = KEY_F6,
	[0x08] = KEY_F7,
	[0x10] = KEY_F8,
	[0x11] = KEY_F9,
	[0x12] = KEY_F10,
	[0x13] = KEY_F11,
	[0x14] = KEY_F12,
	[0x15] = KEY_MUTE,
	[0x16] = KEY_VOLUMEDOWN,
	[0x17] = KEY_VOLUMEUP,
	/* Multimedia keys [0x20 - 0x39] */
	[0x21] = KEY_SCREENLOCK,
	[0x22] = KEY_WLAN,
	[0x23] = KEY_BLUETOOTH,
	[0x24] = KEY_TOUCHPAD_TOGGLE,
	[0x25] = KEY_BRIGHTNESSDOWN,
	[0x26] = KEY_BRIGHTNESSUP,
	[0x27] = KEY_BRIGHTNESS_AUTO,
	[0x28] = KEY_PRINT,
	[0x30] = KEY_WWW,
	[0x31] = KEY_CONFIG,
	[0x32] = KEY_PREVIOUSSONG,
	[0x33] = KEY_PLAYPAUSE,
	[0x34] = KEY_NEXTSONG,
	[0x35] = KEY_MUTE,
	[0x36] = KEY_VOLUMEDOWN,
	[0x37] = KEY_VOLUMEUP,
};

static void asus_ec_keys_report_key(struct input_dev *dev, unsigned int code,
				    unsigned int key, bool value)
{
	input_event(dev, EV_MSC, MSC_SCAN, code);
	input_report_key(dev, key, value);
	input_sync(dev);
}

static int asus_ec_keys_process_key(struct input_dev *dev, u8 code)
{
	struct asus_ec_keys_data *priv = dev_get_drvdata(dev->dev.parent);
	unsigned int key = 0;

	if (code == 0)
		return NOTIFY_DONE;

	if (priv->ec->right_alt_pressed) {
		priv->ec->special_combo_triggered = true;

		/* Flip special key mode state when pressing SCREEN LOCK + R ALT */
		if (code == 1) {
			priv->special_key_mode = !priv->special_key_mode;
			return NOTIFY_OK;
		}
	}

	/*
	 * Relocate code to second "page" if pressed state XOR's mode state
	 * This way special key will invert the current mode
	 */
	if (priv->special_key_mode ^ priv->ec->right_alt_pressed)
		code += ASUSEC_EXT_KEY_CODES;

	if (code < dev->keycodemax) {
		unsigned short *map = dev->keycode;

		key = map[code];
	}

	if (!key)
		key = KEY_UNKNOWN;

	asus_ec_keys_report_key(dev, code, key, 1);
	asus_ec_keys_report_key(dev, code, key, 0);

	return NOTIFY_OK;
}

static int asus_ec_keys_notify(struct notifier_block *nb, unsigned long action, void *data_)
{
	struct asus_ec_keys_data *priv =
		container_of(nb, struct asus_ec_keys_data, nb);
	u8 *data = data_;

	if (action & ASUSEC_SMI_MASK)
		return NOTIFY_DONE;

	if (action & ASUSEC_SCI_MASK)
		return asus_ec_keys_process_key(priv->xidev, data[2]);

	return NOTIFY_DONE;
}

static void asus_ec_keys_setup_keymap(struct asus_ec_keys_data *priv)
{
	struct input_dev *dev = priv->xidev;
	unsigned int i;

	BUILD_BUG_ON(ARRAY_SIZE(priv->keymap) < ARRAY_SIZE(asus_ec_dock_ext_keys));

	dev->keycode = priv->keymap;
	dev->keycodesize = sizeof(*priv->keymap);
	dev->keycodemax = ARRAY_SIZE(priv->keymap);

	input_set_capability(dev, EV_MSC, MSC_SCAN);
	input_set_capability(dev, EV_KEY, KEY_UNKNOWN);

	for (i = 0; i < ARRAY_SIZE(asus_ec_dock_ext_keys); i++) {
		unsigned int code = asus_ec_dock_ext_keys[i];

		__set_bit(code, dev->keybit);
		priv->keymap[i] = code;
	}
}

static int asus_ec_keys_open(struct input_dev *input)
{
	struct asus_ec_keys_data *priv = input_get_drvdata(input);
	struct asusec_core *ec = priv->ec;
	int error;

	priv->nb.notifier_call = asus_ec_keys_notify;
	error = blocking_notifier_chain_register(&ec->notify_list, &priv->nb);
	if (error)
		return error;

	ec->ext_keys_enabled = true;

	return 0;
}

static void asus_ec_keys_close(struct input_dev *input)
{
	struct asus_ec_keys_data *priv = input_get_drvdata(input);
	struct asusec_core *ec = priv->ec;

	ec->ext_keys_enabled = false;
	blocking_notifier_chain_unregister(&ec->notify_list, &priv->nb);
}

static int asus_ec_keys_probe(struct platform_device *pdev)
{
	struct i2c_client *parent = to_i2c_client(pdev->dev.parent);
	struct asusec_core *ec = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct asus_ec_keys_data *priv;
	int error;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->ec = ec;

	priv->xidev = devm_input_allocate_device(dev);
	if (!priv->xidev)
		return -ENOMEM;

	priv->xidev->name = devm_kasprintf(dev, GFP_KERNEL, "%s Keyboard Ext", ec->model);
	priv->xidev->phys = devm_kasprintf(dev, GFP_KERNEL, "i2c-%u-%04x",
					   i2c_adapter_id(parent->adapter), parent->addr);

	if (!priv->xidev->name || !priv->xidev->phys)
		return -ENOMEM;

	priv->xidev->open = asus_ec_keys_open;
	priv->xidev->close = asus_ec_keys_close;
	input_set_drvdata(priv->xidev, priv);

	asus_ec_keys_setup_keymap(priv);

	error = input_register_device(priv->xidev);
	if (error)
		return dev_err_probe(dev, error, "Failed to register extension keys\n");

	return 0;
}

static struct platform_driver asus_ec_keys_driver = {
	.driver.name = "asus-transformer-ec-keys",
	.probe = asus_ec_keys_probe,
};
module_platform_driver(asus_ec_keys_driver);

MODULE_ALIAS("platform:asus-transformer-ec-keys");
MODULE_AUTHOR("Michał Mirosław <mirq-linux@rere.qmqm.pl>");
MODULE_DESCRIPTION("ASUS Transformer's multimedia keys driver");
MODULE_LICENSE("GPL");
