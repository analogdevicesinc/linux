// SPDX-License-Identifier: GPL-2.0
/*
 * Serial port driver for BCM2835AUX UART
 *
 * Copyright (C) 2016 Martin Sperl <kernel@martin.sperl.org>
 *
 * Based on 8250_lpc18xx.c:
 * Copyright (C) 2015 Joachim Eastwood <manabian@gmail.com>
 *
 * The bcm2835aux is capable of RTS auto flow-control, but this driver doesn't
 * take advantage of it yet.  When adding support, be sure not to enable it
 * simultaneously to rs485.
 */

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include "8250.h"

#define BCM2835_AUX_UART_CNTL		8
#define BCM2835_AUX_UART_CNTL_RXEN	0x01 /* Receiver enable */
#define BCM2835_AUX_UART_CNTL_TXEN	0x02 /* Transmitter enable */
#define BCM2835_AUX_UART_CNTL_AUTORTS	0x04 /* RTS set by RX fill level */
#define BCM2835_AUX_UART_CNTL_AUTOCTS	0x08 /* CTS stops transmitter */
#define BCM2835_AUX_UART_CNTL_RTS3	0x00 /* RTS set until 3 chars left */
#define BCM2835_AUX_UART_CNTL_RTS2	0x10 /* RTS set until 2 chars left */
#define BCM2835_AUX_UART_CNTL_RTS1	0x20 /* RTS set until 1 chars left */
#define BCM2835_AUX_UART_CNTL_RTS4	0x30 /* RTS set until 4 chars left */
#define BCM2835_AUX_UART_CNTL_RTSINV	0x40 /* Invert auto RTS polarity */
#define BCM2835_AUX_UART_CNTL_CTSINV	0x80 /* Invert auto CTS polarity */

/**
 * struct bcm2835aux_data - driver private data of BCM2835 auxiliary UART
 * @clk: clock producer of the port's uartclk
 * @line: index of the port's serial8250_ports[] entry
 * @cntl: cached copy of CNTL register
 */
struct bcm2835aux_data {
	struct clk *clk;
	int line;
	u32 cntl;
};

static void bcm2835aux_rs485_start_tx(struct uart_8250_port *up)
{
	if (!(up->port.rs485.flags & SER_RS485_RX_DURING_TX)) {
		struct bcm2835aux_data *data = dev_get_drvdata(up->port.dev);

		data->cntl &= ~BCM2835_AUX_UART_CNTL_RXEN;
		serial_out(up, BCM2835_AUX_UART_CNTL, data->cntl);
	}

	/*
	 * On the bcm2835aux, the MCR register contains no other
	 * flags besides RTS.  So no need for a read-modify-write.
	 */
	if (up->port.rs485.flags & SER_RS485_RTS_ON_SEND)
		serial8250_out_MCR(up, 0);
	else
		serial8250_out_MCR(up, UART_MCR_RTS);
}

static void bcm2835aux_rs485_stop_tx(struct uart_8250_port *up)
{
	if (up->port.rs485.flags & SER_RS485_RTS_AFTER_SEND)
		serial8250_out_MCR(up, 0);
	else
		serial8250_out_MCR(up, UART_MCR_RTS);

	if (!(up->port.rs485.flags & SER_RS485_RX_DURING_TX)) {
		struct bcm2835aux_data *data = dev_get_drvdata(up->port.dev);

		data->cntl |= BCM2835_AUX_UART_CNTL_RXEN;
		serial_out(up, BCM2835_AUX_UART_CNTL, data->cntl);
	}
}

static int bcm2835aux_serial_probe(struct platform_device *pdev)
{
	const struct software_node *bcm2835_swnode;
	struct uart_8250_port up = { };
	struct bcm2835aux_data *data;
	struct resource *res;
	unsigned int uartclk;
	int ret;

	/* allocate the custom structure */
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* initialize data */
	up.capabilities = UART_CAP_FIFO | UART_CAP_MINI;
	up.port.dev = &pdev->dev;
	up.port.type = PORT_16550;
	up.port.flags = UPF_FIXED_PORT | UPF_FIXED_TYPE | UPF_SKIP_TEST | UPF_IOREMAP;
	up.port.rs485_config = serial8250_em485_config;
	up.port.rs485_supported = serial8250_em485_supported;
	up.rs485_start_tx = bcm2835aux_rs485_start_tx;
	up.rs485_stop_tx = bcm2835aux_rs485_stop_tx;

	/* initialize cached copy with power-on reset value */
	data->cntl = BCM2835_AUX_UART_CNTL_RXEN | BCM2835_AUX_UART_CNTL_TXEN;

	platform_set_drvdata(pdev, data);

	/* get the clock - this also enables the HW */
	data->clk = devm_clk_get_optional(&pdev->dev, NULL);
	if (IS_ERR(data->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(data->clk), "could not get clk\n");

	/* map the main registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "memory resource not found");
		return -EINVAL;
	}

	up.port.mapbase = res->start;
	up.port.mapsize = resource_size(res);

	bcm2835_swnode = device_get_match_data(&pdev->dev);
	if (bcm2835_swnode) {
		ret = device_add_software_node(&pdev->dev, bcm2835_swnode);
		if (ret)
			return ret;
	}

	ret = uart_read_port_properties(&up.port);
	if (ret)
		goto rm_swnode;

	up.port.regshift = 2;
	up.port.fifosize = 8;

	/* enable the clock as a last step */
	ret = clk_prepare_enable(data->clk);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "unable to enable uart clock\n");
		goto rm_swnode;
	}

	uartclk = clk_get_rate(data->clk);
	if (uartclk)
		up.port.uartclk = uartclk;

	/* the HW-clock divider for bcm2835aux is 8,
	 * but 8250 expects a divider of 16,
	 * so we have to multiply the actual clock by 2
	 * to get identical baudrates.
	 */
	up.port.uartclk *= 2;

	/* The clock is only queried at probe time, which means we get one shot
	 * at this. A zero clock is never going to work and is almost certainly
	 * due to a parent not being ready, so prefer to defer.
	 */
	if (!up.port.uartclk)
	    return -EPROBE_DEFER;

	/* register the port */
	ret = serial8250_register_8250_port(&up);
	if (ret < 0) {
		dev_err_probe(&pdev->dev, ret, "unable to register 8250 port\n");
		goto dis_clk;
	}
	data->line = ret;

	return 0;

dis_clk:
	clk_disable_unprepare(data->clk);
rm_swnode:
	device_remove_software_node(&pdev->dev);
	return ret;
}

static void bcm2835aux_serial_remove(struct platform_device *pdev)
{
	struct bcm2835aux_data *data = platform_get_drvdata(pdev);

	serial8250_unregister_port(data->line);
	clk_disable_unprepare(data->clk);
	device_remove_software_node(&pdev->dev);
}

/*
 * Some UEFI implementations (e.g. tianocore/edk2 for the Raspberry Pi)
 * describe the miniuart with a base address that encompasses the auxiliary
 * registers shared between the miniuart and spi.
 *
 * This is due to historical reasons, see discussion here:
 * https://edk2.groups.io/g/devel/topic/87501357#84349
 *
 * We need to add the offset between the miniuart and auxiliary registers
 * to get the real miniuart base address.
 */
static const struct property_entry bcm2835_acpi_properties[] = {
	PROPERTY_ENTRY_U32("reg-offset", 0x40),
	{ }
};

static const struct software_node bcm2835_acpi_node = {
	.properties = bcm2835_acpi_properties,
};

static const struct of_device_id bcm2835aux_serial_match[] = {
	{ .compatible = "brcm,bcm2835-aux-uart" },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm2835aux_serial_match);

static const struct acpi_device_id bcm2835aux_serial_acpi_match[] = {
	{ "BCM2836", (kernel_ulong_t)&bcm2835_acpi_node },
	{ }
};
MODULE_DEVICE_TABLE(acpi, bcm2835aux_serial_acpi_match);

static bool bcm2835aux_can_disable_clock(struct device *dev)
{
	struct bcm2835aux_data *data = dev_get_drvdata(dev);
	struct uart_8250_port *up = serial8250_get_port(data->line);

	if (device_may_wakeup(dev))
		return false;

	if (uart_console(&up->port) && !console_suspend_enabled)
		return false;

	return true;
}

static int bcm2835aux_suspend(struct device *dev)
{
	struct bcm2835aux_data *data = dev_get_drvdata(dev);

	serial8250_suspend_port(data->line);

	if (!bcm2835aux_can_disable_clock(dev))
		return 0;

	clk_disable_unprepare(data->clk);
	return 0;
}

static int bcm2835aux_resume(struct device *dev)
{
	struct bcm2835aux_data *data = dev_get_drvdata(dev);
	int ret;

	if (bcm2835aux_can_disable_clock(dev)) {
		ret = clk_prepare_enable(data->clk);
		if (ret)
			return ret;
	}

	serial8250_resume_port(data->line);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(bcm2835aux_dev_pm_ops, bcm2835aux_suspend, bcm2835aux_resume);

static struct platform_driver bcm2835aux_serial_driver = {
	.driver = {
		.name = "bcm2835-aux-uart",
		.of_match_table = bcm2835aux_serial_match,
		.acpi_match_table = bcm2835aux_serial_acpi_match,
		.pm = pm_ptr(&bcm2835aux_dev_pm_ops),
	},
	.probe  = bcm2835aux_serial_probe,
	.remove_new = bcm2835aux_serial_remove,
};
module_platform_driver(bcm2835aux_serial_driver);

#ifdef CONFIG_SERIAL_8250_CONSOLE

static int __init early_bcm2835aux_setup(struct earlycon_device *device,
					const char *options)
{
	if (!device->port.membase)
		return -ENODEV;

	device->port.iotype = UPIO_MEM32;
	device->port.regshift = 2;

	return early_serial8250_setup(device, NULL);
}

OF_EARLYCON_DECLARE(bcm2835aux, "brcm,bcm2835-aux-uart",
		    early_bcm2835aux_setup);
#endif

MODULE_DESCRIPTION("BCM2835 auxiliar UART driver");
MODULE_AUTHOR("Martin Sperl <kernel@martin.sperl.org>");
MODULE_LICENSE("GPL v2");
