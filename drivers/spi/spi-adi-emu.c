// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * SPI ADI Emulator
 *
 * Copyright (C) 2022 Analog Devices, Inc.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/spi/spi.h>

#define REG_CHIP_ID		0x00
#define CHIP_ID			0x41

#define REG_SCRATCH_PAD		0x01

#define REG_DEVICE_CONFIG	0x02
#define POWER_DOWN		BIT(5)

#define REG_CNVST		0x03
#define CNVST			BIT(0)

#define REG_CH0_DATA_HIGH	0x04
#define REG_CH0_DATA_LOW	0x05
#define REG_CH1_DATA_HIGH	0x06
#define REG_CH1_DATA_LOW	0x07

#define REG_TEST_MODE		0x08
#define TEST_MODE(x)		(((x) & 0x7) << 1)

#define REGS_NUM		0x09

#define RD_MASK			BIT(7)

const u8 wr_mask[9] = {
	0x0, 0xff, POWER_DOWN, CNVST, 0xf, 0xff, 0xf, 0xff, TEST_MODE(7)
};

enum adi_emu_test_mode {
	TEST_OFF,
	TEST_MIDSCALE,
	TEST_POSITIVE,
	TEST_NEGATIVE,
	TEST_RAMP,
};

struct spi_adi_emu {
	u8			reg[REGS_NUM];
	u8			read_addr;
	bool			power_down;
	enum adi_emu_test_mode	test_mode;
	u16			test_ramp;
	u16			test_off;
};

const u16 test_off_data[128] = {
	0x000, 0x064, 0x0C8, 0x12C, 0x18F, 0x1F1, 0x252, 0x2B1,
	0x30F, 0x36B, 0x3C5, 0x41C, 0x471, 0x4C3, 0x512, 0x55F,
	0x5A7, 0x5ED, 0x62E, 0x66C, 0x6A6, 0x6DC, 0x70D, 0x73A,
	0x763, 0x787, 0x7A7, 0x7C2, 0x7D8, 0x7E9, 0x7F5, 0x7FD,
	0x7FF, 0x7FD, 0x7F5, 0x7E9, 0x7D8, 0x7C2, 0x7A7, 0x787,
	0x763, 0x73A, 0x70D, 0x6DC, 0x6A6, 0x66C, 0x62E, 0x5ED,
	0x5A7, 0x55F, 0x512, 0x4C3, 0x471, 0x41C, 0x3C5, 0x36B,
	0x30F, 0x2B1, 0x252, 0x1F1, 0x18F, 0x12C, 0xC8,  0x64,
	0x000, 0xF9B, 0xF37, 0xED3, 0xE70, 0xE0E, 0xDAD, 0xD4E,
	0xCF0, 0xC94, 0xC3A, 0xBE3, 0xB8E, 0xB3C, 0xAED, 0xAA0,
	0xA58, 0xA12, 0x9D1, 0x993, 0x959, 0x923, 0x8F2, 0x8C5,
	0x89C, 0x878, 0x858, 0x83D, 0x827, 0x816, 0x80A, 0x802,
	0x800, 0x802, 0x80A, 0x816, 0x827, 0x83D, 0x858, 0x878,
	0x89C, 0x8C5, 0x8F2, 0x923, 0x959, 0x993, 0x9D1, 0xA12,
	0xA58, 0xAA0, 0xAED, 0xB3C, 0xB8E, 0xBE3, 0xC3A, 0xC94,
	0xCF0, 0xD4E, 0xDAD, 0xE0E, 0xE70, 0xED3, 0xF37, 0xF9B
};

static int spi_adi_emu_transfer_one(struct spi_master *master,
				struct spi_device *spi,
				struct spi_transfer *transfer)
{
	struct spi_adi_emu *emu = spi_master_get_devdata(master);
	u8 *tx_buf = (u8 *)transfer->tx_buf;
	u8 *rx_buf = (u8 *)transfer->rx_buf;
	u16 ch0_data;
	u16 ch1_data;

	if ((transfer->len == 1) && tx_buf) {
		if (tx_buf[0] & RD_MASK) {
			emu->read_addr = tx_buf[0] & ~RD_MASK;
			goto done;
		} else
			goto error;
	}

	if ((transfer->len == 1) && rx_buf) {
		if (emu->power_down)
			rx_buf[0] = 0;
		else
			rx_buf[0] = emu->reg[emu->read_addr];
		goto done;
	}

	if ((transfer->len == 2) && tx_buf) {
		if (emu->power_down && (tx_buf[0] != REG_DEVICE_CONFIG))
			goto done;
		if ((tx_buf[0] == REG_CNVST) && (tx_buf[1] & CNVST)) {
			switch (emu->test_mode) {
				case TEST_OFF:
					ch0_data = test_off_data[emu->test_off];
					ch1_data = (test_off_data[emu->test_off]
							+ 64) % 128;
					emu->test_off ++;
					emu->test_off %= 128;
					break;
				case TEST_MIDSCALE:
					ch0_data = 0;
					ch1_data = 0;
					break;
				case TEST_POSITIVE:
					ch0_data = 0x7ff;
					ch1_data = 0x7ff;
					break;
				case TEST_NEGATIVE:
					ch0_data = 0x800;
					ch1_data = 0x800;
					break;
				case TEST_RAMP:
					ch0_data = emu->test_ramp;
					ch1_data = (emu->test_ramp
							+ 2048) % 4096;
					emu->test_ramp ++;
					emu->test_ramp %= 4096;
					break;
				default:
					ch0_data = 0;
					ch1_data = 0;
			}
			emu->reg[REG_CH0_DATA_HIGH] = (ch0_data & 0x0f00) >> 8;
			emu->reg[REG_CH0_DATA_LOW] = (ch0_data & 0x00ff) >> 0;
			emu->reg[REG_CH1_DATA_HIGH] = (ch1_data & 0x0f00) >> 8;
			emu->reg[REG_CH1_DATA_LOW] = (ch1_data & 0x00ff) >> 0;
		} else {
			emu->reg[tx_buf[0]] = tx_buf[1] & wr_mask[tx_buf[0]];
			switch (tx_buf[0]) {
				case REG_DEVICE_CONFIG:
					if (emu->reg[tx_buf[0]] & POWER_DOWN)
						emu->power_down = true;
					else
						emu->power_down = false;
					break;
				case REG_TEST_MODE:
					emu->test_mode =
						emu->reg[tx_buf[0]] >> 1;
			}
		}

		goto done;
	}

error:
	return -EINVAL;

done:
	return 0;
}

static int spi_adi_emu_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct spi_adi_emu *emu;

	master = devm_spi_alloc_master(&pdev->dev, sizeof(*emu));
	if (!master)
		return -ENOMEM;

	emu = spi_master_get_devdata(master);

	emu->reg[REG_CHIP_ID] = CHIP_ID;
	emu->reg[REG_DEVICE_CONFIG] = POWER_DOWN;
	emu->power_down = true;
	emu->reg[REG_TEST_MODE] = TEST_MODE(TEST_RAMP);
	emu->test_mode = TEST_RAMP;

	master->dev.of_node = pdev->dev.of_node;
	master->transfer_one = spi_adi_emu_transfer_one;

	return devm_spi_register_master(&pdev->dev, master);
}

static const struct of_device_id spi_adi_emu_of_match[] = {
	{ .compatible = "adi,spi-adi-emu" },
	{ }
};

static struct platform_driver spi_adi_emu_driver = {
	.driver = {
		.name = "spi-adi-emu",
		.of_match_table = spi_adi_emu_of_match,
	},
	.probe = spi_adi_emu_probe,
};
module_platform_driver(spi_adi_emu_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("SPI ADI Emulator");
MODULE_LICENSE("Dual BSD/GPL");