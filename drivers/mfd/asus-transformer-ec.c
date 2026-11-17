// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/array_size.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/asus-transformer-ec.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define ASUSEC_ACCESS_TIMEOUT		300
#define ASUSEC_DOCKRAM_OFFSET		2
#define ASUSEC_ECREQ_DELAY		50
#define ASUSEC_ECREQ_TIMEOUT		200
#define ASUSEC_RESET			0
#define ASUSEC_RETRY_MAX		3
#define ASUSEC_RSP_BUFFER_SIZE		(ASUSEC_ENTRIES / ASUSEC_ENTRY_SIZE)
#define ASUSEC_SL101_ENTRY_SIZE		8

enum asusec_variant {
	ASUSEC_SL101_DOCK = 1,
	ASUSEC_TF101_DOCK,
	ASUSEC_TF201_PAD,
	ASUSEC_TF600T_PAD,
	ASUSEC_MAX
};

enum asusec_mode {
	ASUSEC_MODE_NONE,
	ASUSEC_MODE_NORMAL,
	ASUSEC_MODE_FACTORY,
	ASUSEC_MODE_MAX
};

/**
 * struct asus_ec_chip_info
 *
 * @name: prefix associated with the EC
 * @variant: id of programming model of EC
 * @fmode: state of Factory Mode bit in EC control register
 * @read_size: size of block to read
 * @keyboard_mask: mask of keyboard packets
 */
struct asus_ec_chip_info {
	const char *name;
	enum asusec_variant variant;
	enum asusec_mode fmode;
	u8 read_size;
	u8 keyboard_mask;
};

/**
 * struct asus_ec_data
 *
 * @ec: public part shared with all cells (must be first)
 * @ecreq_lock: prevents simultaneous access to EC
 * @ecreq_gpio: EC request GPIO
 * @client: pointer to EC's i2c_client
 * @info: pointer to EC's version description
 * @ec_buf: buffer for EC read
 */
struct asus_ec_data {
	struct asusec_core ec;
	struct mutex ecreq_lock;
	struct gpio_desc *ecreq_gpio;
	struct i2c_client *client;
	const struct asus_ec_chip_info *info;
	u8 ec_buf[ASUSEC_ENTRY_BUFSIZE];
};

/**
 * struct dockram_ec_data
 *
 * @ctl_lock: prevent simultaneous access to Dockram
 * @ctl_buf: buffer for Dockram read
 */
struct dockram_ec_data {
	struct mutex ctl_lock;
	u8 ctl_buf[ASUSEC_ENTRY_BUFSIZE];
};

/**
 * asus_dockram_access_ctl - Read from or write to the DockRAM control register.
 * @client: Handle to the DockRAM device.
 * @out: Pointer to a variable where the register value will be stored.
 * @mask: Bitmask of bits to be cleared.
 * @xor: Bitmask of bits to be set (via XOR).
 *
 * This performs a control register read if @out is provided and both @mask
 * and @xor are zero. Otherwise, it performs a control register update if
 * @mask and @xor are provided.
 *
 * Returns a negative errno code else zero on success.
 */
int asus_dockram_access_ctl(struct i2c_client *client, u64 *out, u64 mask, u64 xor)
{
	struct dockram_ec_data *ddata;
	u8 *buf;
	u64 val;
	int ret = 0;

	if (!client)
		return -EINVAL;

	ddata = i2c_get_clientdata(client);
	if (!ddata)
		return -ENODEV;

	buf = ddata->ctl_buf;

	guard(mutex)(&ddata->ctl_lock);

	memset(buf, 0, ASUSEC_ENTRY_BUFSIZE);
	ret = i2c_smbus_read_i2c_block_data(client, ASUSEC_DOCKRAM_CONTROL, ASUSEC_ENTRY_SIZE,
					    buf);
	if (ret < ASUSEC_ENTRY_SIZE) {
		dev_err(&client->dev, "failed to access control buffer: %d\n", ret);
		return ret < 0 ? ret : -EIO;
	}

	if (buf[0] != ASUSEC_CTL_SIZE) {
		dev_err(&client->dev, "buffer size exceeds %d: %d\n", ASUSEC_CTL_SIZE, buf[0]);
		return -EPROTO;
	}

	val = get_unaligned_le64(buf + 1);

	if (out)
		*out = val;

	if (mask || xor) {
		put_unaligned_le64((val & ~mask) ^ xor, buf + 1);
		ret = i2c_smbus_write_i2c_block_data(client, ASUSEC_DOCKRAM_CONTROL,
						     ASUSEC_ENTRY_SIZE, buf);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(asus_dockram_access_ctl);

static int asus_ec_signal_request(struct asus_ec_data *ddata)
{
	guard(mutex)(&ddata->ecreq_lock);

	gpiod_set_value_cansleep(ddata->ecreq_gpio, 1);
	msleep(ASUSEC_ECREQ_DELAY);

	gpiod_set_value_cansleep(ddata->ecreq_gpio, 0);
	msleep(ASUSEC_ECREQ_TIMEOUT);

	return 0;
}

static int asus_ec_log_info(struct asus_ec_data *ddata, unsigned int reg, const char *name)
{
	struct device *dev = &ddata->client->dev;
	u8 buf[ASUSEC_ENTRY_BUFSIZE];
	int ret;

	memset(buf, 0, ASUSEC_ENTRY_BUFSIZE);
	ret = i2c_smbus_read_i2c_block_data(ddata->ec.dockram, reg, ASUSEC_ENTRY_SIZE, buf);
	if (ret < ASUSEC_ENTRY_SIZE)
		return ret < 0 ? ret : -EIO;

	if (buf[0] > ASUSEC_ENTRY_SIZE) {
		dev_err(dev, "bad data len; buffer: %*ph; ret: %d\n", ASUSEC_ENTRY_BUFSIZE, buf,
			ret);
		return -EPROTO;
	}

	dev_info(dev, "%-14s: %.*s\n", name, buf[0], buf + 1);

	if (!ddata->ec.model) {
		ddata->ec.model = devm_kasprintf(dev, GFP_KERNEL, "%.*s", buf[0], buf + 1);
		if (!ddata->ec.model)
			return -ENOMEM;
	}

	return 0;
}

static int asus_ec_detect(struct asus_ec_data *ddata)
{
	int ret;

	ret = asus_ec_log_info(ddata, ASUSEC_DOCKRAM_INFO_MODEL, "Model");
	if (ret)
		return ret;

	ret = asus_ec_log_info(ddata, ASUSEC_DOCKRAM_INFO_FW, "FW version");
	if (ret)
		return ret;

	ret = asus_ec_log_info(ddata, ASUSEC_DOCKRAM_INFO_CFGFMT, "Config format");
	if (ret)
		return ret;

	ret = asus_ec_log_info(ddata, ASUSEC_DOCKRAM_INFO_HW, "HW version");
	if (ret)
		return ret;

	ddata->ec.name = ddata->info->name;
	ddata->ec.keyboard_mask = ddata->info->keyboard_mask;

	return 0;
}

static int asus_ec_reset(struct asus_ec_data *ddata)
{
	int ret;

	guard(mutex)(&ddata->ecreq_lock);

	for (int retry = 0; retry < ASUSEC_RETRY_MAX; retry++) {
		ret = i2c_smbus_write_word_data(ddata->client, ASUSEC_WRITE_BUF, ASUSEC_RESET);
		if (!ret)
			return 0;

		msleep(ASUSEC_ACCESS_TIMEOUT);
	}

	return ret;
}

static void asus_ec_clear_buffer(struct asus_ec_data *ddata)
{
	int ret, retry = ASUSEC_RSP_BUFFER_SIZE;

	/*
	 * Read the buffer till we get valid data by checking ASUSEC_OBF_MASK
	 * of the status byte or till we reach end of the 256 byte buffer.
	 */
	while (retry--) {
		ret = i2c_smbus_read_i2c_block_data(ddata->client, ASUSEC_READ_BUF,
						    ddata->info->read_size, ddata->ec_buf);
		if (ret < ddata->info->read_size)
			continue;

		if (ddata->ec_buf[ASUSEC_IRQ_STATUS] & ASUSEC_OBF_MASK)
			continue;

		break;
	}
}

static int asus_ec_susb_on_status(struct asus_ec_data *ddata)
{
	u64 flag;
	int ret;

	ret = asus_dockram_access_ctl(ddata->ec.dockram, &flag, 0, 0);
	if (ret)
		return ret;

	flag &= ASUSEC_CTL_SUSB_MODE;
	dev_info(&ddata->client->dev, "EC FW behaviour: %s\n",
		 flag ? "susb on when receive ec_req" : "susb on when system wakeup");

	return 0;
}

static int asus_ec_set_factory_mode(struct asus_ec_data *ddata,
				    enum asusec_mode fmode)
{
	dev_info(&ddata->client->dev, "Entering %s mode.\n",
		 fmode == ASUSEC_MODE_FACTORY ? "factory" : "normal");

	return asus_dockram_access_ctl(ddata->ec.dockram, NULL, ASUSEC_CTL_FACTORY_MODE,
				       fmode == ASUSEC_MODE_FACTORY ? ASUSEC_CTL_FACTORY_MODE : 0);
}

static int asus_ec_init(struct asus_ec_data *ddata)
{
	int ret;

	ret = asus_ec_reset(ddata);
	if (ret)
		goto err_exit;

	asus_ec_clear_buffer(ddata);

	/* Check and inform about EC firmware behavior */
	ret = asus_ec_susb_on_status(ddata);
	if (ret)
		goto err_exit;

	/* Some EC require factory mode to be set normal on each request */
	if (ddata->info->fmode)
		ret = asus_ec_set_factory_mode(ddata, ddata->info->fmode);

err_exit:
	if (ret)
		dev_err(&ddata->client->dev, "failed to access EC: %d\n", ret);

	return ret;
}

static void asus_ec_handle_smi(struct asus_ec_data *ddata, unsigned int code)
{
	switch (code) {
	case ASUSEC_SMI_HANDSHAKE:
	case ASUSEC_SMI_RESET:
		asus_ec_init(ddata);
		break;
	}
}

static irqreturn_t asus_ec_interrupt(int irq, void *dev_id)
{
	struct asus_ec_data *ddata = dev_id;
	unsigned long notify_action;
	int ret;

	ret = i2c_smbus_read_i2c_block_data(ddata->client, ASUSEC_READ_BUF,
					    ddata->info->read_size, ddata->ec_buf);
	if (ret < ddata->info->read_size)
		return IRQ_NONE;

	/* Check status byte with ASUSEC_OBF_MASK if data is valid */
	ret = ddata->ec_buf[ASUSEC_IRQ_STATUS] & ASUSEC_OBF_MASK;
	if (!ret)
		return IRQ_NONE;

	notify_action = ddata->ec_buf[ASUSEC_IRQ_STATUS];
	if (notify_action & ASUSEC_SMI_MASK) {
		unsigned int code = ddata->ec_buf[ASUSEC_SMI_CODE];

		asus_ec_handle_smi(ddata, code);

		notify_action |= code << 8;
	}

	blocking_notifier_call_chain(&ddata->ec.notify_list, notify_action, ddata->ec_buf);

	return IRQ_HANDLED;
}

static void asus_ec_release_dockram_dev(void *client)
{
	i2c_unregister_device(client);
}

static struct i2c_client *devm_asus_dockram_get(struct device *dev)
{
	struct i2c_client *parent = to_i2c_client(dev);
	struct i2c_client *dockram;
	struct dockram_ec_data *ddata;
	int ret;

	dockram = i2c_new_ancillary_device(parent, "dockram",
					   parent->addr + ASUSEC_DOCKRAM_OFFSET);
	if (IS_ERR(dockram))
		return dockram;

	ret = devm_add_action_or_reset(dev, asus_ec_release_dockram_dev, dockram);
	if (ret)
		return ERR_PTR(ret);

	ddata = devm_kzalloc(&dockram->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return ERR_PTR(-ENOMEM);

	i2c_set_clientdata(dockram, ddata);
	mutex_init(&ddata->ctl_lock);

	return dockram;
}

static const struct mfd_cell asus_ec_sl101_dock_mfd_devices[] = {
	MFD_CELL_NAME("asus-transformer-ec-kbc"),
};

static const struct mfd_cell asus_ec_tf101_dock_mfd_devices[] = {
	MFD_CELL_NAME("asus-transformer-ec-battery"),
	MFD_CELL_NAME("asus-transformer-ec-charger"),
	MFD_CELL_NAME("asus-transformer-ec-led"),
	MFD_CELL_NAME("asus-transformer-ec-kbc"),
	MFD_CELL_NAME("asus-transformer-ec-keys"),
};

static const struct mfd_cell asus_ec_tf201_pad_mfd_devices[] = {
	MFD_CELL_NAME("asus-transformer-ec-battery"),
	MFD_CELL_NAME("asus-transformer-ec-led"),
};

static const struct mfd_cell asus_ec_tf600t_pad_mfd_devices[] = {
	MFD_CELL_NAME("asus-transformer-ec-battery"),
	MFD_CELL_NAME("asus-transformer-ec-charger"),
	MFD_CELL_NAME("asus-transformer-ec-led"),
};

static int asus_ec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct asus_ec_data *ddata;
	const struct mfd_cell *cells;
	unsigned int num_cells;
	unsigned long irqflags;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return dev_err_probe(dev, -ENXIO,
			"I2C bus is missing required SMBus block mode support\n");

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->info = device_get_match_data(dev);
	if (!ddata->info)
		return -ENODEV;

	switch (ddata->info->variant) {
	case ASUSEC_SL101_DOCK:
		cells = asus_ec_sl101_dock_mfd_devices;
		num_cells = ARRAY_SIZE(asus_ec_sl101_dock_mfd_devices);
		break;
	case ASUSEC_TF101_DOCK:
		cells = asus_ec_tf101_dock_mfd_devices;
		num_cells = ARRAY_SIZE(asus_ec_tf101_dock_mfd_devices);
		break;
	case ASUSEC_TF201_PAD:
		cells = asus_ec_tf201_pad_mfd_devices;
		num_cells = ARRAY_SIZE(asus_ec_tf201_pad_mfd_devices);
		break;
	case ASUSEC_TF600T_PAD:
		cells = asus_ec_tf600t_pad_mfd_devices;
		num_cells = ARRAY_SIZE(asus_ec_tf600t_pad_mfd_devices);
		break;
	default:
		return dev_err_probe(dev, -EINVAL, "Unknown device variant %d\n",
				     ddata->info->variant);
	}

	i2c_set_clientdata(client, ddata);
	ddata->client = client;

	ddata->ec.dockram = devm_asus_dockram_get(dev);
	if (IS_ERR(ddata->ec.dockram))
		return dev_err_probe(dev, PTR_ERR(ddata->ec.dockram), "Failed to get DockRam\n");

	ddata->ecreq_gpio = devm_gpiod_get(dev, "request", GPIOD_OUT_LOW);
	if (IS_ERR(ddata->ecreq_gpio))
		return dev_err_probe(dev, PTR_ERR(ddata->ecreq_gpio),
				     "Failed to get EC request GPIO\n");

	BLOCKING_INIT_NOTIFIER_HEAD(&ddata->ec.notify_list);
	mutex_init(&ddata->ecreq_lock);

	asus_ec_signal_request(ddata);

	ret = asus_ec_detect(ddata);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to detect EC version\n");

	ret = asus_ec_init(ddata);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to init EC\n");

	/*
	 * Systems using device tree should set up interrupt via DTS,
	 * the rest will use the default low interrupt.
	 */
	irqflags = dev->of_node ? 0 : IRQF_TRIGGER_LOW;

	ret = devm_request_threaded_irq(dev, client->irq, NULL, &asus_ec_interrupt,
					IRQF_ONESHOT | irqflags, client->name, ddata);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register IRQ\n");

	/* Parent I2C controller uses DMA, ASUS EC and child devices do not */
	client->dev.coherent_dma_mask = 0;
	client->dev.dma_mask = &client->dev.coherent_dma_mask;

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, cells, num_cells, NULL, 0, NULL);
}

static const struct asus_ec_chip_info asus_ec_sl101_dock_data = {
	.name = "dock",
	.variant = ASUSEC_SL101_DOCK,
	.fmode = ASUSEC_MODE_NONE,
	.read_size = ASUSEC_SL101_ENTRY_SIZE,
	.keyboard_mask = ASUSEC_OBF_MASK,
};

static const struct asus_ec_chip_info asus_ec_tf101_dock_data = {
	.name = "dock",
	.variant = ASUSEC_TF101_DOCK,
	.fmode = ASUSEC_MODE_NONE,
	.read_size = ASUSEC_ENTRY_SIZE,
	.keyboard_mask = ASUSEC_KBC_MASK | ASUSEC_KEY_MASK,
};

static const struct asus_ec_chip_info asus_ec_tf201_pad_data = {
	.name = "pad",
	.variant = ASUSEC_TF201_PAD,
	.fmode = ASUSEC_MODE_NORMAL,
	.read_size = ASUSEC_ENTRY_SIZE,
	.keyboard_mask = ASUSEC_KBC_MASK | ASUSEC_KEY_MASK,
};

static const struct asus_ec_chip_info asus_ec_tf600t_pad_data = {
	.name = "pad",
	.variant = ASUSEC_TF600T_PAD,
	.fmode = ASUSEC_MODE_NORMAL,
	.read_size = ASUSEC_ENTRY_SIZE,
	.keyboard_mask = ASUSEC_KBC_MASK | ASUSEC_KEY_MASK,
};

static const struct of_device_id asus_ec_match[] = {
	{
		.compatible = "asus,sl101-ec-dock",
		.data = &asus_ec_sl101_dock_data,
	}, {
		.compatible = "asus,tf101-ec-dock",
		.data = &asus_ec_tf101_dock_data,
	}, {
		.compatible = "asus,tf201-ec-pad",
		.data = &asus_ec_tf201_pad_data,
	}, {
		.compatible = "asus,tf600t-ec-pad",
		.data = &asus_ec_tf600t_pad_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, asus_ec_match);

static struct i2c_driver asus_ec_driver = {
	.driver	= {
		.name = "asus-transformer-ec",
		.of_match_table = asus_ec_match,
	},
	.probe = asus_ec_probe,
};
module_i2c_driver(asus_ec_driver);

MODULE_AUTHOR("Michał Mirosław <mirq-linux@rere.qmqm.pl>");
MODULE_AUTHOR("Svyatoslav Ryhel <clamor95@gmail.com>");
MODULE_DESCRIPTION("ASUS Transformer's EC driver");
MODULE_LICENSE("GPL");
