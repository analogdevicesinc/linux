// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/i8042.h>
#include <linux/mfd/asus-transformer-ec.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serio.h>

struct asus_ec_kbc_data {
	struct notifier_block nb;
	struct asusec_core *ec;
	struct i2c_client *parent;
	struct serio *sdev[2];
	bool altgr_delayed;
	bool altgr_suppressed;
};

static bool asus_ec_kbc_altgr_active(struct asus_ec_kbc_data *priv)
{
	return IS_ENABLED(CONFIG_KEYBOARD_ASUS_TRANSFORMER_EC) &&
		(priv->ec->ext_keys_enabled || priv->altgr_delayed ||
		 priv->altgr_suppressed);
}

static void asus_ec_kbc_reset_altgr(struct asus_ec_kbc_data *priv)
{
	if (!IS_ENABLED(CONFIG_KEYBOARD_ASUS_TRANSFORMER_EC))
		return;

	priv->ec->right_alt_pressed = false;
	priv->ec->special_combo_triggered = false;
	priv->altgr_delayed = false;
	priv->altgr_suppressed = false;
}

static void asus_ec_kbc_sync_altgr_combo(struct asus_ec_kbc_data *priv)
{
	if (priv->ec->special_combo_triggered) {
		priv->ec->special_combo_triggered = false;
		if (priv->altgr_delayed) {
			priv->altgr_delayed = false;
			priv->altgr_suppressed = true;
		}
	}
}

static void asus_ec_kbc_flush_altgr(struct asus_ec_kbc_data *priv)
{
	if (!asus_ec_kbc_altgr_active(priv))
		return;

	asus_ec_kbc_sync_altgr_combo(priv);

	if (!priv->altgr_delayed)
		return;

	serio_interrupt(priv->sdev[0], 0xe0, 0);
	serio_interrupt(priv->sdev[0], 0x11, 0);

	priv->altgr_delayed = false;
}

static bool asus_ec_kbc_handle_altgr(struct asus_ec_kbc_data *priv, bool is_release)
{
	bool suppressed;

	if (!asus_ec_kbc_altgr_active(priv)) {
		asus_ec_kbc_reset_altgr(priv);
		return false;
	}

	asus_ec_kbc_sync_altgr_combo(priv);

	if (!is_release) {
		if (!priv->ec->right_alt_pressed) {
			asus_ec_kbc_reset_altgr(priv);
			priv->ec->right_alt_pressed = true;
			priv->altgr_delayed = true;
			return true;
		}

		return priv->altgr_delayed || priv->altgr_suppressed;
	}

	suppressed = priv->altgr_suppressed;
	if (!suppressed)
		asus_ec_kbc_flush_altgr(priv);

	asus_ec_kbc_reset_altgr(priv);
	return suppressed;
}

static int asus_ec_kbc_notify(struct notifier_block *nb,
			      unsigned long action, void *data_)
{
	struct asus_ec_kbc_data *priv = container_of(nb, struct asus_ec_kbc_data, nb);
	unsigned int port_idx, n;
	u8 *data = data_;

	if (action & (ASUSEC_SMI_MASK | ASUSEC_SCI_MASK)) {
		if (action == ASUSEC_SMI_ACTION(HANDSHAKE) ||
		    action == ASUSEC_SMI_ACTION(RESET))
			asus_ec_kbc_reset_altgr(priv);
		return NOTIFY_DONE;
	} else if (action & ASUSEC_AUX_MASK) {
		port_idx = 1;
	} else if (action & priv->ec->keyboard_mask) {
		port_idx = 0;
	} else {
		return NOTIFY_DONE;
	}

	/*
	 * The data[0] is the number of bytes following the count byte: one
	 * status byte plus the payload. The SL101 EC, for example, reports
	 * keyboard reset as 03 09 fa aa (status 09, ACK fa, BAT aa). Skip the
	 * count and status bytes and forward every payload byte to serio.
	 */
	if (data[0] < 2 || data[0] > ASUSEC_ENTRY_SIZE)
		return NOTIFY_BAD;

	n = data[0] - 1;
	data += 2;

	if (port_idx == 0) {
		/*
		 * Remap keyboard key codes to match AT layout:
		 * ALTGR:  RIGHT-ALT  [E0 11] -> delayed / suppressed
		 * SEARCH: RIGHT-META [E0 27] -> LEFT-ALT   [11]
		 * MENU:   COMPOSE    [E0 2F] -> RIGHT-META [E0 27]
		 */
		if ((n == 2 || (n == 3 && data[1] == 0xF0)) && data[0] == 0xE0) {
			u8 *keycode = &data[n - 1];

			switch (*keycode) {
			case 0x11:
				if (asus_ec_kbc_handle_altgr(priv, n == 3))
					return NOTIFY_OK;
				break;
			case 0x27:
				*keycode = 0x11;
				++data;
				--n;
				break;
			case 0x2F:
				*keycode = 0x27;
				break;
			}
		}

		/*
		 * Only flush delayed AltGr on key make events, ignoring
		 * key break codes and PS/2 command responses (ACK, BAT, etc.).
		 */
		if ((n == 2 && data[0] == 0xe0) ||
		    (n == 1 && data[0] > 0 && data[0] < 0x84) || data[0] == 0xe1)
			asus_ec_kbc_flush_altgr(priv);
	}

	while (n--)
		serio_interrupt(priv->sdev[port_idx], *data++, 0);

	return NOTIFY_OK;
}

static int asus_ec_serio_write(struct serio *port, unsigned char data)
{
	struct asus_ec_kbc_data *priv = port->port_data;

	return i2c_smbus_write_word_data(priv->parent, ASUSEC_WRITE_BUF,
					 (data << 8) | port->id.extra);
}

static void asus_ec_serio_remove(void *port)
{
	serio_unregister_port(port);
}

static int asus_ec_register_serio(struct platform_device *pdev, int idx, const char *name, int cmd)
{
	struct asus_ec_kbc_data *priv = platform_get_drvdata(pdev);
	struct i2c_client *parent = priv->parent;
	struct serio *port = kzalloc_obj(*port);

	if (!port)
		return -ENOMEM;

	priv->sdev[idx] = port;
	port->dev.parent = &pdev->dev;
	port->id.type = SERIO_8042;
	port->id.extra = cmd & 0xFF;
	port->write = asus_ec_serio_write;
	port->port_data = (void *)priv;
	snprintf(port->name, sizeof(port->name), "%s %s", priv->ec->model, name);
	snprintf(port->phys, sizeof(port->phys), "i2c-%u-%04x/serio%d",
		 i2c_adapter_id(parent->adapter), parent->addr, idx);

	serio_register_port(port);

	return devm_add_action_or_reset(&pdev->dev, asus_ec_serio_remove, port);
}

static void asus_ec_notifier_chain_unregister(void *data)
{
	struct asus_ec_kbc_data *priv = data;
	struct asusec_core *ec = priv->ec;

	blocking_notifier_chain_unregister(&ec->notify_list, &priv->nb);
}

static int asus_ec_kbc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct asusec_core *ec = dev_get_drvdata(dev->parent);
	struct asus_ec_kbc_data *priv;
	int error;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	priv->ec = ec;
	priv->parent = to_i2c_client(dev->parent);

	priv->nb.notifier_call = asus_ec_kbc_notify;
	error = blocking_notifier_chain_register(&ec->notify_list, &priv->nb);
	if (error)
		return dev_err_probe(dev, error, "Failed to register blocking notifier chain");

	error = devm_add_action_or_reset(dev, asus_ec_notifier_chain_unregister, priv);
	if (error)
		return error;

	error = asus_ec_register_serio(pdev, 0, "Keyboard", 0);
	if (error)
		return error;

	error = asus_ec_register_serio(pdev, 1, "Touchpad", I8042_CMD_AUX_SEND);
	if (error)
		return error;

	return 0;
}

static struct platform_driver asus_ec_kbc_driver = {
	.driver.name = "asus-transformer-ec-kbc",
	.probe = asus_ec_kbc_probe,
};
module_platform_driver(asus_ec_kbc_driver);

MODULE_ALIAS("platform:asus-transformer-ec-kbc");
MODULE_AUTHOR("Michał Mirosław <mirq-linux@rere.qmqm.pl>");
MODULE_DESCRIPTION("ASUS Transformer's Dock keyboard and touchpad driver");
MODULE_LICENSE("GPL");
