// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/err.h>
#include <linux/mfd/asus-transformer-ec.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>

struct asus_ec_charger_data {
	struct notifier_block nb;
	struct asusec_core *ec;
	struct power_supply *psy;
	struct power_supply_desc psy_desc;
};

static enum power_supply_property asus_ec_charger_properties[] = {
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int asus_ec_charger_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct asus_ec_charger_data *priv = power_supply_get_drvdata(psy);
	enum power_supply_usb_type psu;
	int ret;
	u64 ctl;

	/* Check if model name is requested first since it needs no hw access */
	if (psp == POWER_SUPPLY_PROP_MODEL_NAME) {
		val->strval = priv->ec->model;
		return 0;
	}

	ret = asus_dockram_access_ctl(priv->ec->dockram, &ctl, 0, 0);
	if (ret)
		return ret;

	switch (ctl & (ASUSEC_CTL_FULL_POWER_SOURCE | ASUSEC_CTL_DIRECT_POWER_SOURCE)) {
	case ASUSEC_CTL_FULL_POWER_SOURCE:
		psu = POWER_SUPPLY_USB_TYPE_CDP;	/* DOCK */
		break;
	case ASUSEC_CTL_DIRECT_POWER_SOURCE:
		psu = POWER_SUPPLY_USB_TYPE_SDP;	/* USB */
		break;
	case 0:
		psu = POWER_SUPPLY_USB_TYPE_UNKNOWN;	/* no power source connected */
		break;
	default:
		psu = POWER_SUPPLY_USB_TYPE_ACA;	/* power adapter */
		break;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = psu != POWER_SUPPLY_USB_TYPE_UNKNOWN;
		return 0;

	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = psu;
		return 0;

	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		if (ctl & ASUSEC_CTL_TEST_DISCHARGE)
			val->intval = POWER_SUPPLY_CHARGE_BEHAVIOUR_FORCE_DISCHARGE;
		else if (ctl & ASUSEC_CTL_USB_CHARGE)
			val->intval = POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
		else
			val->intval = POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;
		return 0;

	default:
		return -EINVAL;
	}
}

static int asus_ec_charger_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct asus_ec_charger_data *priv = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		switch ((enum power_supply_charge_behaviour)val->intval) {
		case POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO:
			return asus_dockram_access_ctl(priv->ec->dockram, NULL,
				ASUSEC_CTL_TEST_DISCHARGE | ASUSEC_CTL_USB_CHARGE,
				ASUSEC_CTL_USB_CHARGE);

		case POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE:
			return asus_dockram_access_ctl(priv->ec->dockram, NULL,
				ASUSEC_CTL_TEST_DISCHARGE | ASUSEC_CTL_USB_CHARGE, 0);

		case POWER_SUPPLY_CHARGE_BEHAVIOUR_FORCE_DISCHARGE:
			return asus_dockram_access_ctl(priv->ec->dockram, NULL,
				ASUSEC_CTL_TEST_DISCHARGE | ASUSEC_CTL_USB_CHARGE,
				ASUSEC_CTL_TEST_DISCHARGE);
		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
}

static int asus_ec_charger_property_is_writeable(struct power_supply *psy,
						 enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		return true;
	default:
		return false;
	}
}

static const struct power_supply_desc asus_ec_charger_desc = {
	.name = "asus-ec-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
			     BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE) |
			     BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_FORCE_DISCHARGE),
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_ACA),
	.properties = asus_ec_charger_properties,
	.num_properties = ARRAY_SIZE(asus_ec_charger_properties),
	.get_property = asus_ec_charger_get_property,
	.set_property = asus_ec_charger_set_property,
	.property_is_writeable = asus_ec_charger_property_is_writeable,
	.no_thermal = true,
};

static int asus_ec_charger_notify(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct asus_ec_charger_data *priv =
		container_of(nb, struct asus_ec_charger_data, nb);

	switch (action) {
	case ASUSEC_SMI_ACTION(POWER_NOTIFY):
	case ASUSEC_SMI_ACTION(ADAPTER_EVENT):
		power_supply_changed(priv->psy);
		break;
	}

	return NOTIFY_DONE;
}

static int asus_ec_charger_probe(struct platform_device *pdev)
{
	struct asusec_core *ec = dev_get_drvdata(pdev->dev.parent);
	struct asus_ec_charger_data *priv;
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = { };

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->ec = ec;

	cfg.fwnode = dev_fwnode(dev->parent);
	cfg.drv_data = priv;

	memcpy(&priv->psy_desc, &asus_ec_charger_desc, sizeof(priv->psy_desc));
	priv->psy_desc.name = devm_kasprintf(dev, GFP_KERNEL, "%s-charger",
					     priv->ec->name);
	if (!priv->psy_desc.name)
		return -ENOMEM;

	priv->psy = devm_power_supply_register(dev, &priv->psy_desc, &cfg);
	if (IS_ERR(priv->psy))
		return dev_err_probe(dev, PTR_ERR(priv->psy),
				     "Failed to register power supply\n");

	priv->nb.notifier_call = asus_ec_charger_notify;

	return blocking_notifier_chain_register(&ec->notify_list, &priv->nb);
}

static void asus_ec_charger_remove(struct platform_device *pdev)
{
	struct asus_ec_charger_data *priv = platform_get_drvdata(pdev);
	struct asusec_core *ec = priv->ec;

	blocking_notifier_chain_unregister(&ec->notify_list, &priv->nb);
}

static struct platform_driver asus_ec_charger_driver = {
	.driver.name = "asus-transformer-ec-charger",
	.probe = asus_ec_charger_probe,
	.remove = asus_ec_charger_remove,
};
module_platform_driver(asus_ec_charger_driver);

MODULE_ALIAS("platform:asus-transformer-ec-charger");
MODULE_AUTHOR("Michał Mirosław <mirq-linux@rere.qmqm.pl>");
MODULE_DESCRIPTION("ASUS Transformer Pad battery charger driver");
MODULE_LICENSE("GPL");
