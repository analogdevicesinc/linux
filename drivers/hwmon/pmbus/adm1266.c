// SPDX-License-Identifier: GPL-2.0
/*
 * ADM1266 - Cascadable Super Sequencer with Margin
 * Control and Fault Recording
 *
 * Copyright 2020 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/i2c-smbus.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/nvmem-provider.h>
#include <linux/slab.h>

#include "pmbus.h"

#define ADM1266_BLACKBOX_CONFIG	0xD3
#define ADM1266_PDIO_CONFIG	0xD4
#define ADM1266_GO_COMMAND	0xD8
#define ADM1266_READ_STATE	0xD9
#define ADM1266_READ_BLACKBOX	0xDE
#define ADM1266_GPIO_CONFIG	0xE1
#define ADM1266_BLACKBOX_INFO	0xE6
#define ADM1266_PDIO_STATUS	0xE9
#define ADM1266_GPIO_STATUS	0xEA

/* ADM1266 GPIO defines */
#define ADM1266_GPIO_NR			9
#define ADM1266_GPIO_FUNCTIONS(x)	FIELD_GET(BIT(0), x)
#define ADM1266_GPIO_INPUT_EN(x)	FIELD_GET(BIT(2), x)
#define ADM1266_GPIO_OUTPUT_EN(x)	FIELD_GET(BIT(3), x)
#define ADM1266_GPIO_OPEN_DRAIN(x)	FIELD_GET(BIT(4), x)

/* ADM1266 PDIO defines */
#define ADM1266_PDIO_NR			16
#define ADM1266_PDIO_PIN_CFG(x)		FIELD_GET(GENMASK(15, 13), x)
#define ADM1266_PDIO_GLITCH_FILT(x)	FIELD_GET(GENMASK(12, 9), x)
#define ADM1266_PDIO_OUT_CFG(x)		FIELD_GET(GENMASK(2, 0), x)

#define ADM1266_BLACKBOX_OFFSET		0x7F700
#define ADM1266_BLACKBOX_SIZE		64

struct adm1266_data {
	struct pmbus_driver_info info;
	struct gpio_chip gc;
	const char *gpio_names[ADM1266_GPIO_NR + ADM1266_PDIO_NR];
	struct i2c_client *client;
	struct dentry *debugfs_dir;
	struct nvmem_config nvmem_config;
	struct nvmem_device *nvmem;
	u8 *dev_mem;
};

static const struct nvmem_cell_info adm1266_nvmem_cells[] = {
	{
		.name           = "blackbox",
		.offset         = ADM1266_BLACKBOX_OFFSET,
		.bytes          = 2048,
	},
};

#if IS_ENABLED(CONFIG_GPIOLIB)
static const unsigned int adm1266_gpio_mapping[ADM1266_GPIO_NR][2] = {
	{1, 0},
	{2, 1},
	{3, 2},
	{4, 8},
	{5, 9},
	{6, 10},
	{7, 11},
	{8, 6},
	{9, 7},
};

static const char *adm1266_names[ADM1266_GPIO_NR + ADM1266_PDIO_NR] = {
	"GPIO1", "GPIO2", "GPIO3", "GPIO4", "GPIO5", "GPIO6", "GPIO7", "GPIO8",
	"GPIO9", "PDIO1", "PDIO2", "PDIO3", "PDIO4", "PDIO5", "PDIO6",
	"PDIO7", "PDIO8", "PDIO9", "PDIO10", "PDIO11", "PDIO12", "PDIO13",
	"PDIO14", "PDIO15", "PDIO16",
};

static int adm1266_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct adm1266_data *data = gpiochip_get_data(chip);
	u8 read_buf[PMBUS_BLOCK_MAX + 1];
	unsigned long pins_status;
	unsigned int pmbus_cmd;
	int ret;

	if (offset < ADM1266_GPIO_NR)
		pmbus_cmd = ADM1266_GPIO_STATUS;
	else
		pmbus_cmd = ADM1266_PDIO_STATUS;

	ret = i2c_smbus_read_block_data(data->client, pmbus_cmd,
					read_buf);
	if (ret < 0)
		return ret;

	pins_status = read_buf[0] + (read_buf[1] << 8);
	if (offset < ADM1266_GPIO_NR)
		return test_bit(adm1266_gpio_mapping[offset][1], &pins_status);
	else
		return test_bit(offset - ADM1266_GPIO_NR, &pins_status);
}

static int adm1266_gpio_get_multiple(struct gpio_chip *chip,
				     unsigned long *mask,
				     unsigned long *bits)
{
	struct adm1266_data *data = gpiochip_get_data(chip);
	u8 gpio_data[PMBUS_BLOCK_MAX + 1];
	u8 pdio_data[PMBUS_BLOCK_MAX + 1];
	unsigned long gpio_status;
	unsigned long pdio_status;
	unsigned int gpio_nr;
	int ret;

	ret = i2c_smbus_read_block_data(data->client, ADM1266_GPIO_STATUS,
					gpio_data);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_read_block_data(data->client, ADM1266_PDIO_STATUS,
					pdio_data);
	if (ret < 0)
		return ret;

	gpio_status = gpio_data[0] + (gpio_data[1] << 8);
	pdio_status = pdio_data[0] + (pdio_data[1] << 8);
	*bits = 0;
	for_each_set_bit(gpio_nr, mask, ADM1266_GPIO_NR) {
		if (test_bit(adm1266_gpio_mapping[gpio_nr][1], &gpio_status))
			set_bit(gpio_nr, bits);
	}

	for_each_set_bit_from(gpio_nr, mask,
			      ADM1266_GPIO_NR + ADM1266_PDIO_STATUS) {
		if (test_bit(gpio_nr - ADM1266_GPIO_NR, &pdio_status))
			set_bit(gpio_nr, bits);
	}

	return 0;
}

static void adm1266_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct adm1266_data *data = gpiochip_get_data(chip);
	u8 write_buf[PMBUS_BLOCK_MAX + 1];
	u8 read_buf[PMBUS_BLOCK_MAX + 1];
	unsigned long gpio_config;
	unsigned long pdio_config;
	unsigned long pin_cfg;
	int ret;
	int i;

	for (i = 0; i < ADM1266_GPIO_NR; i++) {
		write_buf[0] = adm1266_gpio_mapping[i][1];
		ret = pmbus_block_wr(data->client, ADM1266_GPIO_CONFIG, 1,
				     write_buf, read_buf);
		if (ret < 0)
			dev_err(&data->client->dev, "GPIOs scan failed(%d).\n",
				ret);

		gpio_config = read_buf[0];
		seq_puts(s, adm1266_names[i]);

		seq_puts(s, " ( ");
		if (!ADM1266_GPIO_FUNCTIONS(gpio_config)) {
			seq_puts(s, "high-Z )\n");
			continue;
		}
		if (ADM1266_GPIO_INPUT_EN(gpio_config))
			seq_puts(s, "input ");
		if (ADM1266_GPIO_OUTPUT_EN(gpio_config))
			seq_puts(s, "output ");
		if (ADM1266_GPIO_OPEN_DRAIN(gpio_config))
			seq_puts(s, "open-drain )\n");
		else
			seq_puts(s, "push-pull )\n");
	}

	write_buf[0] = 0xFF;
	ret = pmbus_block_wr(data->client, ADM1266_PDIO_CONFIG, 1, write_buf,
			     read_buf);
	if (ret < 0)
		dev_err(&data->client->dev, "PDIOs scan failed(%d).\n", ret);

	for (i = 0; i < ADM1266_PDIO_NR; i++) {
		seq_puts(s, adm1266_names[ADM1266_GPIO_NR + i]);

		pdio_config = read_buf[2 * i];
		pdio_config += (read_buf[2 * i + 1] << 8);
		pin_cfg = ADM1266_PDIO_PIN_CFG(pdio_config);

		seq_puts(s, " ( ");
		if (!pin_cfg || pin_cfg > 5) {
			seq_puts(s, "high-Z )\n");
			continue;
		}

		if (pin_cfg & BIT(0))
			seq_puts(s, "output ");

		if (pin_cfg & BIT(1))
			seq_puts(s, "input ");

		seq_puts(s, ")\n");
	}
}

static int adm1266_config_gpio(struct adm1266_data *data)
{
	const char *name = dev_name(&data->client->dev);
	char *gpio_name;
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(data->gpio_names); i++) {
		gpio_name = devm_kasprintf(&data->client->dev, GFP_KERNEL,
					   "adm1266-%x-%s", data->client->addr,
					   adm1266_names[i]);
		if (!gpio_name)
			return -ENOMEM;

		data->gpio_names[i] = gpio_name;
	}

	data->gc.label = name;
	data->gc.parent = &data->client->dev;
	data->gc.owner = THIS_MODULE;
	data->gc.base = -1;
	data->gc.names = data->gpio_names;
	data->gc.ngpio = ARRAY_SIZE(data->gpio_names);
	data->gc.get = adm1266_gpio_get;
	data->gc.get_multiple = adm1266_gpio_get_multiple;
	data->gc.dbg_show = adm1266_gpio_dbg_show;

	ret = devm_gpiochip_add_data(&data->client->dev, &data->gc, data);
	if (ret)
		dev_err(&data->client->dev, "GPIO registering failed (%d)\n",
			ret);

	return ret;
}
#else
static inline int adm1266_config_gpio(struct adm1266_data *data)
{
	return 0;
}
#endif

static int adm1266_get_state_op(void *pdata, u64 *state)
{
	struct adm1266_data *data = pdata;
	int ret;

	ret = i2c_smbus_read_word_data(data->client, ADM1266_READ_STATE);
	if (ret < 0)
		return ret;

	*state = ret;

	return 0;
}

static int adm1266_set_go_command_op(void *pdata, u64 val)
{
	struct adm1266_data *data = pdata;
	u8 reg;

	reg = FIELD_GET(GENMASK(4, 0), val);

	return i2c_smbus_write_word_data(data->client, ADM1266_GO_COMMAND, reg);
}

static int adm1266_blackbox_information_read(struct seq_file *s, void *pdata)
{
	struct device *dev = s->private;
	struct i2c_client *client = to_i2c_client(dev);
	u8 read_buf[PMBUS_BLOCK_MAX + 1];
	unsigned int latest_id;
	int ret;

	ret = i2c_smbus_read_block_data(client, ADM1266_BLACKBOX_INFO,
					read_buf);
	if (ret < 0)
		return ret;

	seq_puts(s, "BLACKBOX_INFORMATION:\n");
	latest_id = read_buf[0] + (read_buf[1] << 8);
	seq_printf(s, "Black box ID: %x\n", latest_id);
	seq_printf(s, "Logic index: %x\n", read_buf[2]);
	seq_printf(s, "Record count: %x\n", read_buf[3]);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(go_command_fops, NULL, adm1266_set_go_command_op,
			 "%llu\n");
DEFINE_DEBUGFS_ATTRIBUTE(read_state_fops, adm1266_get_state_op, NULL, "%llu\n");

static void adm1266_debug_init(struct adm1266_data *data)
{
	struct dentry *root;
	char dir_name[30];

	sprintf(dir_name, "adm1266-%x_debugfs", data->client->addr);
	root = debugfs_create_dir(dir_name, NULL);
	data->debugfs_dir = root;
	debugfs_create_file_unsafe("go_command", 0200, root, data,
				   &go_command_fops);
	debugfs_create_file_unsafe("read_state", 0400, root, data,
				   &read_state_fops);
	debugfs_create_devm_seqfile(&data->client->dev, "blackbox_information",
				    root, adm1266_blackbox_information_read);
}

static int adm1266_nvmem_read_blackbox(struct adm1266_data *data, u8 *buf)
{
	u8 write_buf[PMBUS_BLOCK_MAX + 1];
	u8 read_buf[PMBUS_BLOCK_MAX + 1];
	int record_count;
	int ret;
	int i;

	ret = i2c_smbus_read_block_data(data->client, ADM1266_BLACKBOX_INFO,
					read_buf);
	if (ret < 0)
		return ret;

	record_count = read_buf[3];

	for (i = 0; i < record_count; i++) {
		write_buf[0] = i;
		ret = pmbus_block_wr(data->client, ADM1266_READ_BLACKBOX, 1,
				     write_buf, buf);
		if (ret < 0)
			return ret;

		buf += ADM1266_BLACKBOX_SIZE;
	}

	return 0;
}

static bool adm1266_cell_is_accessed(const struct nvmem_cell_info *mem_cell,
				     unsigned int offset, size_t bytes)
{
	unsigned int start_addr = offset;
	unsigned int end_addr = offset + bytes;
	unsigned int cell_start = mem_cell->offset;
	unsigned int cell_end = mem_cell->offset + mem_cell->bytes;

	if (start_addr <= cell_end && cell_start <= end_addr)
		return true;

	return false;
}

static int adm1266_read_mem_cell(struct adm1266_data *data,
				 const struct nvmem_cell_info *mem_cell)
{
	u8 *mem_offset;
	int ret;

	switch (mem_cell->offset) {
	case ADM1266_BLACKBOX_OFFSET:
		mem_offset = data->dev_mem + mem_cell->offset;
		ret = adm1266_nvmem_read_blackbox(data, mem_offset);
		if (ret)
			dev_err(&data->client->dev, "Could not read blackbox!");
		return ret;
	default:
		return -EINVAL;
	}
}

static int adm1266_nvmem_read(void *priv, unsigned int offset, void *val,
			      size_t bytes)
{
	const struct nvmem_cell_info *mem_cell;
	struct adm1266_data *data = priv;
	int ret;
	int i;

	for (i = 0; i < data->nvmem_config.ncells; i++) {
		mem_cell = &adm1266_nvmem_cells[i];
		if (!adm1266_cell_is_accessed(mem_cell, offset, bytes))
			continue;

		ret = adm1266_read_mem_cell(data, mem_cell);
		if (ret < 0)
			return ret;
	}

	memcpy(val, data->dev_mem + offset, bytes);

	return 0;
}

static int adm1266_config_nvmem(struct adm1266_data *data)
{
	data->nvmem_config.name = dev_name(&data->client->dev);
	data->nvmem_config.dev = &data->client->dev;
	data->nvmem_config.root_only = true;
	data->nvmem_config.read_only = true;
	data->nvmem_config.owner = THIS_MODULE;
	data->nvmem_config.reg_read = adm1266_nvmem_read;
	data->nvmem_config.cells = adm1266_nvmem_cells;
	data->nvmem_config.ncells = ARRAY_SIZE(adm1266_nvmem_cells);
	data->nvmem_config.priv = data;
	data->nvmem_config.stride = 1;
	data->nvmem_config.word_size = 1;
	data->nvmem_config.size = 0x80000;

	data->nvmem = nvmem_register(&data->nvmem_config);
	if (IS_ERR(data->nvmem)) {
		dev_err(&data->client->dev, "Could not register nvmem!");
		return PTR_ERR(data->nvmem);
	}

	data->dev_mem = devm_kzalloc(&data->client->dev,
				     data->nvmem_config.size,
				     GFP_KERNEL);
	if (!data->dev_mem)
		return -ENOMEM;

	return 0;
}

static int adm1266_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct pmbus_driver_info *info;
	struct adm1266_data *data;
	u32 funcs;
	int ret;
	int i;

	data = devm_kzalloc(&client->dev, sizeof(struct adm1266_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;

	ret = adm1266_config_gpio(data);
	if (ret < 0)
		return ret;

	ret = adm1266_config_nvmem(data);
	if (ret < 0)
		return ret;

	adm1266_debug_init(data);

	info = &data->info;
	info->pages = 17;
	info->format[PSC_VOLTAGE_OUT] = linear;
	funcs = PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT;
	for (i = 0; i < info->pages; i++)
		info->func[i] = funcs;

	return pmbus_do_probe(client, id, info);
}

static const struct of_device_id adm1266_of_match[] = {
	{ .compatible = "adi,adm1266" },
	{ }
};
MODULE_DEVICE_TABLE(of, adm1266_of_match);

static const struct i2c_device_id adm1266_id[] = {
	{ "adm1266", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adm1266_id);

static struct i2c_driver adm1266_driver = {
	.driver = {
		   .name = "adm1266",
		   .of_match_table = adm1266_of_match,
		  },
	.probe = adm1266_probe,
	.remove = pmbus_do_remove,
	.id_table = adm1266_id,
};

module_i2c_driver(adm1266_driver);

MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_DESCRIPTION("PMBus driver for Analog Devices ADM1266");
MODULE_LICENSE("GPL v2");
