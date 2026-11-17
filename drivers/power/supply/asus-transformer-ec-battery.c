// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/array_size.h>
#include <linux/devm-helpers.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/mfd/asus-transformer-ec.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/unaligned.h>

#define ASUSEC_BATTERY_DATA_FRESH_MSEC		5000

#define ASUSEC_BATTERY_DISCHARGING		BIT(6)
#define ASUSEC_BATTERY_FULL_CHARGED		BIT(5)
#define ASUSEC_BATTERY_NOT_CHARGING		BIT(4)

#define TEMP_CELSIUS_OFFSET			2731

struct asus_ec_battery_data {
	struct asusec_core *ec;
	struct power_supply *battery;
	struct power_supply_desc psy_desc;
	struct delayed_work poll_work;
	struct mutex battery_lock; /* for data refresh */
	unsigned long batt_data_ts;
	int last_state;
	u8 batt_data[ASUSEC_ENTRY_BUFSIZE];
};

static int asus_ec_battery_refresh(struct asus_ec_battery_data *priv)
{
	struct i2c_client *client = priv->ec->dockram;
	struct device *dev = &client->dev;
	int ret = 0;

	if (time_before(jiffies, priv->batt_data_ts))
		return ret;

	memset(priv->batt_data, 0, ASUSEC_ENTRY_BUFSIZE);
	ret = i2c_smbus_read_i2c_block_data(client, ASUSEC_DOCKRAM_BATT_CTL,
					    ASUSEC_ENTRY_SIZE, priv->batt_data);
	if (ret < ASUSEC_ENTRY_SIZE)
		return ret < 0 ? ret : -EIO;

	if (priv->batt_data[0] > ASUSEC_ENTRY_SIZE) {
		dev_err(dev, "bad data len; buffer: %*ph; ret: %d\n",
			ASUSEC_ENTRY_BUFSIZE, priv->batt_data, ret);
		return -EPROTO;
	}

	priv->batt_data_ts = jiffies +
		msecs_to_jiffies(ASUSEC_BATTERY_DATA_FRESH_MSEC);

	return ret;
}

static enum power_supply_property asus_ec_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_PRESENT,
};

static const unsigned int asus_ec_battery_prop_offs[] = {
	[POWER_SUPPLY_PROP_STATUS] = 1,
	[POWER_SUPPLY_PROP_VOLTAGE_MAX] = 3,
	[POWER_SUPPLY_PROP_CURRENT_MAX] = 5,
	[POWER_SUPPLY_PROP_TEMP] = 7,
	[POWER_SUPPLY_PROP_VOLTAGE_NOW] = 9,
	[POWER_SUPPLY_PROP_CURRENT_NOW] = 11,
	[POWER_SUPPLY_PROP_CAPACITY] = 13,
	[POWER_SUPPLY_PROP_CHARGE_NOW] = 15,
	[POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW] = 17,
	[POWER_SUPPLY_PROP_TIME_TO_FULL_NOW] = 19,
};

static int asus_ec_battery_get_value(struct asus_ec_battery_data *priv,
				     enum power_supply_property psp)
{
	int ret, offs;

	guard(mutex)(&priv->battery_lock);

	if (psp >= ARRAY_SIZE(asus_ec_battery_prop_offs))
		return -EINVAL;

	offs = asus_ec_battery_prop_offs[psp];
	if (!offs)
		return -EINVAL;

	ret = asus_ec_battery_refresh(priv);
	if (ret < 0)
		return ret;

	if (offs >= priv->batt_data[0])
		return -ENODATA;

	return get_unaligned_le16(priv->batt_data + offs);
}

static int asus_ec_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct asus_ec_battery_data *priv = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;

	default:
		ret = asus_ec_battery_get_value(priv, psp);
		if (ret < 0)
			return ret;

		val->intval = (s16)ret;

		switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			if (ret & ASUSEC_BATTERY_FULL_CHARGED)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else if (ret & ASUSEC_BATTERY_NOT_CHARGING)
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			else if (ret & ASUSEC_BATTERY_DISCHARGING)
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;

		case POWER_SUPPLY_PROP_TEMP:
			val->intval -= TEMP_CELSIUS_OFFSET;
			break;

		case POWER_SUPPLY_PROP_CHARGE_NOW:
		case POWER_SUPPLY_PROP_CURRENT_NOW:
		case POWER_SUPPLY_PROP_CURRENT_MAX:
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
			val->intval *= 1000;
			break;

		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
			val->intval = ret * 60;
			break;

		default:
			break;
		}

		break;
	}

	return 0;
}

static void asus_ec_battery_poll_work(struct work_struct *work)
{
	struct asus_ec_battery_data *priv =
		container_of(work, struct asus_ec_battery_data, poll_work.work);
	int state;

	state = asus_ec_battery_get_value(priv, POWER_SUPPLY_PROP_STATUS);
	if (state < 0)
		goto reschedule;

	if (state & ASUSEC_BATTERY_FULL_CHARGED)
		state = POWER_SUPPLY_STATUS_FULL;
	else if (state & ASUSEC_BATTERY_NOT_CHARGING)
		state = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else if (state & ASUSEC_BATTERY_DISCHARGING)
		state = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		state = POWER_SUPPLY_STATUS_CHARGING;

	if (priv->last_state != state) {
		priv->last_state = state;
		power_supply_changed(priv->battery);
	}

reschedule:
	/* continuously send uevent notification */
	schedule_delayed_work(&priv->poll_work,
			      msecs_to_jiffies(ASUSEC_BATTERY_DATA_FRESH_MSEC));
}

static const struct power_supply_desc asus_ec_battery_desc = {
	.name = "asus-ec-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = asus_ec_battery_properties,
	.num_properties = ARRAY_SIZE(asus_ec_battery_properties),
	.get_property = asus_ec_battery_get_property,
	.external_power_changed = power_supply_changed,
};

static int asus_ec_battery_probe(struct platform_device *pdev)
{
	struct asusec_core *ec = dev_get_drvdata(pdev->dev.parent);
	struct asus_ec_battery_data *priv;
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = { };
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	mutex_init(&priv->battery_lock);

	priv->ec = ec;
	priv->batt_data_ts = jiffies - 1;
	priv->last_state = POWER_SUPPLY_STATUS_UNKNOWN;

	cfg.fwnode = dev_fwnode(dev->parent);
	cfg.drv_data = priv;

	memcpy(&priv->psy_desc, &asus_ec_battery_desc, sizeof(priv->psy_desc));
	priv->psy_desc.name = devm_kasprintf(dev, GFP_KERNEL, "%s-battery",
					     priv->ec->name);
	if (!priv->psy_desc.name)
		return -ENOMEM;

	priv->battery = devm_power_supply_register(dev, &priv->psy_desc, &cfg);
	if (IS_ERR(priv->battery))
		return dev_err_probe(dev, PTR_ERR(priv->battery),
				     "Failed to register power supply\n");

	ret = devm_delayed_work_autocancel(dev, &priv->poll_work,
					   asus_ec_battery_poll_work);
	if (ret)
		return ret;

	schedule_delayed_work(&priv->poll_work,
			      msecs_to_jiffies(ASUSEC_BATTERY_DATA_FRESH_MSEC));

	return 0;
}

static int __maybe_unused asus_ec_battery_suspend(struct device *dev)
{
	struct asus_ec_battery_data *priv = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&priv->poll_work);

	return 0;
}

static int __maybe_unused asus_ec_battery_resume(struct device *dev)
{
	struct asus_ec_battery_data *priv = dev_get_drvdata(dev);

	schedule_delayed_work(&priv->poll_work,
			      msecs_to_jiffies(ASUSEC_BATTERY_DATA_FRESH_MSEC));

	return 0;
}

static SIMPLE_DEV_PM_OPS(asus_ec_battery_pm_ops,
			 asus_ec_battery_suspend, asus_ec_battery_resume);

static struct platform_driver asus_ec_battery_driver = {
	.driver = {
		.name = "asus-transformer-ec-battery",
		.pm = &asus_ec_battery_pm_ops,
	},
	.probe = asus_ec_battery_probe,
};
module_platform_driver(asus_ec_battery_driver);

MODULE_ALIAS("platform:asus-transformer-ec-battery");
MODULE_AUTHOR("Michał Mirosław <mirq-linux@rere.qmqm.pl>");
MODULE_AUTHOR("Svyatoslav Ryhel <clamor95@gmail.com>");
MODULE_DESCRIPTION("ASUS Transformer's battery driver");
MODULE_LICENSE("GPL");
