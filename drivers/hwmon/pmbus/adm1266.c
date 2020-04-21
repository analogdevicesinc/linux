// SPDX-License-Identifier: GPL-2.0
/*
 * ADM1266 - Cascadable Super Sequencer with Margin
 * Control and Fault Recording
 *
 * Copyright 2020 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/i2c-smbus.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/nvmem-provider.h>
#include <linux/property.h>
#include <linux/slab.h>

#include "pmbus.h"

#define ADM1266_STORE_USER_ALL	0x15
#define ADM1266_STATUS_MFR	0x80
#define ADM1266_IC_DEVICE_REV	0xAE
#define ADM1266_BLACKBOX_CONFIG	0xD3
#define ADM1266_PDIO_CONFIG	0xD4
#define ADM1266_SEQUENCE_CONFIG	0xD6
#define ADM1266_SYSTEM_CONFIG	0xD7
#define ADM1266_GO_COMMAND	0xD8
#define ADM1266_READ_STATE	0xD9
#define ADM1266_READ_BLACKBOX	0xDE
#define ADM1266_LOGIC_CONFIG	0xE0
#define ADM1266_GPIO_CONFIG	0xE1
#define ADM1266_USER_DATA	0xE3
#define ADM1266_BLACKBOX_INFO	0xE6
#define ADM1266_PDIO_STATUS	0xE9
#define ADM1266_GPIO_STATUS	0xEA
#define ADM1266_MEMORY_CONFIG	0xF8
#define ADM1266_SWITCH_MEMORY	0xFA
#define ADM1266_UPDATE_FW	0xFC
#define ADM1266_FW_PASSWORD	0xFD

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

/* ADM1266 FW_PASSWORD defines*/
#define ADM1266_PASSWD_CMD_LEN	17
#define ADM1266_CHANGE_PASSWORD	1
#define ADM1266_UNLOCK_DEV	2
#define ADM1266_LOCK_DEV	3

/* ADM1266 STATUS_MFR defines */
#define ADM1266_STATUS_PART_LOCKED(x)	FIELD_GET(BIT(2), x)

/* ADM1266 GO_COMMAND defines */
#define ADM1266_GO_COMMAND_STOP		BIT(0)
#define ADM1266_GO_COMMAND_SEQ_RES	BIT(1)
#define ADM1266_GO_COMMAND_HARD_RES	BIT(2)

#define ADM1266_FIRMWARE_OFFSET		0x00000
#define ADM1266_FIRMWARE_SIZE		131072
#define ADM1266_BLACKBOX_OFFSET		0x7F700
#define ADM1266_BLACKBOX_SIZE		64

#define ADM1266_MAX_DEVICES		16

static LIST_HEAD(registered_masters);
static DEFINE_MUTEX(registered_masters_lock);

struct adm1266_data_ref {
	struct adm1266_data *data;
	struct list_head list;
};

struct adm1266_data {
	struct pmbus_driver_info info;
	struct gpio_chip gc;
	struct i2c_client *client;
	struct dentry *debugfs_dir;
	struct nvmem_config nvmem_config;
	struct nvmem_device *nvmem;
	bool master_dev;
	struct list_head cascaded_devices_list;
	struct mutex cascaded_devices_mutex; /* lock cascaded_devices_list */
	u8 nr_devices;
	u8 *dev_mem;
};

static const struct nvmem_cell_info adm1266_nvmem_cells[] = {
	{
		.name           = "blackbox",
		.offset         = ADM1266_BLACKBOX_OFFSET,
		.bytes          = 2048,
	},
	{
		.name           = "firmware",
		.offset         = ADM1266_FIRMWARE_OFFSET,
		.bytes          = ADM1266_FIRMWARE_SIZE,
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
	int ret;

	data->gc.label = name;
	data->gc.parent = &data->client->dev;
	data->gc.owner = THIS_MODULE;
	data->gc.base = -1;
	data->gc.names = adm1266_names;
	data->gc.ngpio = ADM1266_PDIO_NR + ADM1266_GPIO_NR;
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

static int adm1266_group_cmd(struct adm1266_data *data, u8 cmd, u8 *write_data,
			     u8 w_len, bool to_slaves)
{
	struct adm1266_data_ref *slave_ref;
	struct i2c_client *clients[ADM1266_MAX_DEVICES];
	u8 *data_w[ADM1266_MAX_DEVICES];
	u8 w_lens[ADM1266_MAX_DEVICES];
	u8 cmds[ADM1266_MAX_DEVICES];
	int i = 0;

	clients[i] = data->client;
	data_w[i] = write_data;
	i++;

	memset(w_lens, w_len, ADM1266_MAX_DEVICES);
	memset(cmds, cmd, ADM1266_MAX_DEVICES);

	if (!to_slaves)
		return pmbus_group_command(clients, cmds, w_lens, data_w, i);

	list_for_each_entry(slave_ref, &data->cascaded_devices_list,
			    list) {
		clients[i] = slave_ref->data->client;
		data_w[i] = write_data;
		i++;
	}

	return pmbus_group_command(clients, cmds, w_lens, data_w, i);
}

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
	case ADM1266_FIRMWARE_OFFSET:
		/* firmware is write-only */
		return 0;
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

static int adm1266_unlock_device(struct adm1266_data *data)
{
	struct i2c_client *client = data->client;
	u8 passwd_cmd[PMBUS_BLOCK_MAX];
	int reg_val;
	int ret;
	int i;

	memset(passwd_cmd, 0xFF, PMBUS_BLOCK_MAX);
	passwd_cmd[ADM1266_PASSWD_CMD_LEN - 1] = ADM1266_UNLOCK_DEV;

	/* password needs to be written twice correctly*/
	for (i = 0; i < 2; i++) {
		ret = pmbus_block_write(client, ADM1266_FW_PASSWORD,
					ADM1266_PASSWD_CMD_LEN, passwd_cmd);
		if (ret < 0) {
			dev_err(&client->dev, "Could not write password.");
			return ret;
		}

		/* 50 ms delay between subsequent password writes are needed*/
		mdelay(50);
	}

	/* check if device is unlocked */
	reg_val = pmbus_read_byte_data(client, 0, ADM1266_STATUS_MFR);
	if (reg_val < 0) {
		dev_err(&client->dev, "Could not read status.");
		return reg_val;
	}
	if (ADM1266_STATUS_PART_LOCKED(reg_val)) {
		dev_err(&client->dev, "Device locked.");
		return -EBUSY;
	}

	return 0;
}

static int adm1266_unlock_all_dev(struct adm1266_data *data)
{
	struct adm1266_data_ref *slave_ref;
	int ret;

	ret = adm1266_unlock_device(data);
	if (ret < 0) {
		dev_err(&data->client->dev, "Could not unlock master.");
		return ret;
	}

	list_for_each_entry(slave_ref, &data->cascaded_devices_list, list) {
		ret = adm1266_unlock_device(slave_ref->data);
		if (ret < 0) {
			dev_err(&data->client->dev,
				"Could not unlock slave addr: %d.",
				slave_ref->data->client->addr);
			return ret;
		}
	}

	return 0;
}

static const int write_delays[][3] = {
	{ADM1266_SYSTEM_CONFIG, 400, 1},
	{ADM1266_USER_DATA, 100, 1},
	{ADM1266_LOGIC_CONFIG, 200, 1},
	{ADM1266_SEQUENCE_CONFIG, 2500, 1},
	{ADM1266_UPDATE_FW, 2000, 1},
	{ADM1266_MEMORY_CONFIG, 100, 0},
	{ADM1266_STORE_USER_ALL, 300, 0},
};

static int adm1266_write_hex(struct adm1266_data *data,
			     unsigned int offset, unsigned int size)
{
	const u8 *ending_str = ":00000001FF";
	u8 *hex_cmd = data->dev_mem + offset;
	u8 *fw_end = data->dev_mem + offset + size;
	unsigned int write_delay;
	u8 write_buf[PMBUS_BLOCK_MAX + 1];
	u8 first_writes[7];
	u8 byte_count;
	u8 reg_address;
	int ret;
	int i;

	memset(first_writes, 1, 7);

	while (hex_cmd < fw_end) {
		hex_cmd = strnchr(hex_cmd, size, ':');

		if (!hex_cmd || hex_cmd >= fw_end) {
			dev_err(&data->client->dev, "Firmware ending missing.");
			return -EINVAL;
		}

		if (!strncmp(hex_cmd, ending_str, strlen(ending_str)))
			break;

		hex_cmd++;

		ret = hex2bin(&byte_count, hex_cmd, 1);
		if (ret < 0)
			return ret;

		ret = hex2bin(&reg_address, hex_cmd + 4, 1);
		if (ret < 0)
			return ret;

		ret = hex2bin(write_buf, hex_cmd + 8, byte_count);
		if (ret < 0)
			return ret;

		ret = adm1266_group_cmd(data, reg_address, write_buf,
					byte_count, true);
		if (ret < 0) {
			dev_err(&data->client->dev, "Firmware write error: %d.",
				ret);
			return ret;
		}

		/* write to eeprom with specified delays */
		write_delay = 40;
		for (i = 0; i < 7; i++) {
			if (reg_address == write_delays[i][0]) {
				if (write_delays[i][2] && first_writes[i]) {
					first_writes[i] = 0;
					write_delay = write_delays[i][1];
				}

				if (!write_delays[i][2])
					write_delay = write_delays[i][1];
			}
		}
		mdelay(write_delay);
	}

	return 0;
}

static int adm1266_program_firmware(struct adm1266_data *data)
{
	u8 write_data[3];
	int ret;

	write_data[0] = ADM1266_GO_COMMAND_STOP | ADM1266_GO_COMMAND_SEQ_RES;
	write_data[1] = 0x0;
	ret = adm1266_group_cmd(data, ADM1266_GO_COMMAND, write_data, 2, true);
	if (ret < 0) {
		dev_err(&data->client->dev, "Could not stop all devs.");
		return ret;
	}

	/* after issuing a stop command, wait 100 ms */
	mdelay(100);

	ret = adm1266_unlock_all_dev(data);
	if (ret < 0)
		return ret;

	write_data[0] = 0x2;
	write_data[1] = 0x0;
	write_data[2] = 0x0;
	ret = adm1266_group_cmd(data, ADM1266_UPDATE_FW, write_data, 3, true);
	if (ret < 0) {
		dev_err(&data->client->dev, "Could not set bootloader mode.");
		return ret;
	}

	/* wait for adm1266 to enter bootloader mode */
	mdelay(2000);

	ret = adm1266_write_hex(data, ADM1266_FIRMWARE_OFFSET,
				ADM1266_FIRMWARE_SIZE);
	if (ret < 0) {
		dev_err(&data->client->dev, "Could not write hex.");
		return ret;
	}

	write_data[0] = ADM1266_GO_COMMAND_HARD_RES;
	ret = adm1266_group_cmd(data, ADM1266_GO_COMMAND, write_data, 2, true);
	if (ret < 0) {
		dev_err(&data->client->dev, "Could not reset all devs.");
		return ret;
	}

	return 0;
}

/* check if firmware/config write has ended */
static bool adm1266_check_ending(struct adm1266_data *data, unsigned int offset,
				 unsigned int size)
{
	const u8 *ending_str = ":00000001FF";
	u8 *hex_cmd = data->dev_mem + offset;
	u8 *fw_end = data->dev_mem + offset + size;

	hex_cmd = strnchr(hex_cmd, size, ':');
	for (; hex_cmd && hex_cmd < fw_end;
	     hex_cmd = strnchr(hex_cmd, size, ':')) {
		if (!strncmp(hex_cmd, ending_str, strlen(ending_str)))
			return true;

		hex_cmd++;
	}

	return false;
}

static int adm1266_write_mem_cell(struct adm1266_data *data,
				  const struct nvmem_cell_info *mem_cell,
				  unsigned int offset,
				  u8 *val,
				  size_t bytes)
{
	unsigned int cell_end = mem_cell->offset + mem_cell->bytes;
	unsigned int cell_start = mem_cell->offset;
	bool fw_writen;

	switch (mem_cell->offset) {
	case ADM1266_FIRMWARE_OFFSET:
		if (!data->master_dev) {
			dev_err(&data->client->dev,
				"Only master programs the firmware.");
			return -EINVAL;
		}

		if (offset < cell_start || offset + bytes >= cell_end)
			return -EINVAL;

		if (offset == ADM1266_FIRMWARE_OFFSET)
			memset(data->dev_mem, 0, ADM1266_FIRMWARE_SIZE);

		memcpy(data->dev_mem + offset, val, bytes);

		fw_writen = adm1266_check_ending(data, ADM1266_FIRMWARE_OFFSET,
						 ADM1266_FIRMWARE_SIZE);

		if (fw_writen)
			return adm1266_program_firmware(data);

		return 0;
	default:
		return -EINVAL;
	}
}

static int adm1266_nvmem_write(void *priv, unsigned int offset, void *val,
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

		ret = adm1266_write_mem_cell(data, mem_cell, offset,
					     val, bytes);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int adm1266_register_slave(struct adm1266_data *slave,
				  struct adm1266_data *master)
{
	struct adm1266_data_ref *slave_ref;

	slave_ref = devm_kzalloc(&slave->client->dev,
				 sizeof(*slave_ref), GFP_KERNEL);
	if (!slave_ref)
		return -ENOMEM;

	slave_ref->data = slave;
	INIT_LIST_HEAD(&slave_ref->list);

	mutex_lock(&master->cascaded_devices_mutex);
	list_add_tail(&slave_ref->list, &master->cascaded_devices_list);
	mutex_unlock(&master->cascaded_devices_mutex);

	return 0;
}

static int adm1266_register(struct adm1266_data *data)
{
	struct fwnode_reference_args master_fwnode_ref;
	const struct fwnode_handle *fw;
	const struct fwnode_handle *master_fw;
	struct adm1266_data_ref *master_ref;
	int ret;

	fw = dev_fwnode(&data->client->dev);
	INIT_LIST_HEAD(&data->cascaded_devices_list);

	/* master devices do not have this property */
	if (!fwnode_property_present(fw, "adi,master-adm1266")) {
		data->master_dev = true;

		master_ref = devm_kzalloc(&data->client->dev,
					  sizeof(*master_ref), GFP_KERNEL);
		if (!master_ref)
			return -ENOMEM;

		master_ref->data = data;
		INIT_LIST_HEAD(&master_ref->list);

		mutex_lock(&registered_masters_lock);
		list_add(&master_ref->list, &registered_masters);
		mutex_unlock(&registered_masters_lock);
	}

	if (data->master_dev)
		return 0;

	ret = fwnode_property_get_reference_args(fw, "adi,master-adm1266",
						 NULL, 0, 0,
						 &master_fwnode_ref);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"Could not read adi,master-adm1266 property");
		return ret;
	}

	mutex_lock(&registered_masters_lock);

	/* search for the corresponding master of this slave */
	list_for_each_entry(master_ref, &registered_masters, list) {
		master_fw = dev_fwnode(&master_ref->data->client->dev);

		if (master_fw == master_fwnode_ref.fwnode) {
			mutex_unlock(&registered_masters_lock);
			return adm1266_register_slave(data, master_ref->data);
		}
	}

	mutex_unlock(&registered_masters_lock);

	return -EPROBE_DEFER;
}

static int adm1266_config_nvmem(struct adm1266_data *data)
{
	data->nvmem_config.name = dev_name(&data->client->dev);
	data->nvmem_config.dev = &data->client->dev;
	data->nvmem_config.root_only = true;
	data->nvmem_config.owner = THIS_MODULE;
	data->nvmem_config.reg_read = adm1266_nvmem_read;
	data->nvmem_config.reg_write = adm1266_nvmem_write;
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

static int adm1266_firmware_present(struct i2c_client *client)
{
	u8 read_buf[I2C_SMBUS_BLOCK_MAX];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, ADM1266_IC_DEVICE_REV,
					    8, read_buf);
	if (ret < 0) {
		dev_err(&client->dev, "Could not read firmware revision.");
		return ret;
	}

	return !!(read_buf[0] | read_buf[1] | read_buf[2]);
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
	mutex_init(&data->cascaded_devices_mutex);

	ret = adm1266_register(data);
	if (ret < 0)
		return ret;

	adm1266_debug_init(data);

	ret = adm1266_firmware_present(client);
	if (ret < 0)
		return ret;

	if (!ret) {
		dev_notice(&client->dev, "Chip firmware not written.");
		return adm1266_config_nvmem(data);
	}

	ret = adm1266_config_gpio(data);
	if (ret < 0)
		return ret;

	ret = adm1266_config_nvmem(data);
	if (ret < 0)
		return ret;

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
