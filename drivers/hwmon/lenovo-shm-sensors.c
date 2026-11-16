// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * lenovo-shm-sensors - hwmon driver for Lenovo EC sensor windows
 *
 * Copyright (C) 2026 Jiqi Li <lijq9@lenovo.com>
 *
 * The Nuvoton NCT6801D embedded controller used in Lenovo systems does
 * not expose a hardware-monitor logical device in its Super-I/O
 * configuration space: LDN 0Bh is the ATXPGD/BEEP/DDR4 block, so the
 * eSIO-style register window that nct6683 relies on does not exist on
 * this part.
 *
 * Fan and temperature values are instead published by the EC firmware in
 * one of the shared-memory windows of the SHM logical device (LDN 0Fh),
 * which the ACPI firmware maps as
 *
 *	OperationRegion (SHWM, SystemMemory, <base>, 0xFF)
 *
 * The window base is not hardcoded. It is read back from the shared-memory
 * configuration registers of the EC itself (LDN 0Fh, SHAW1..SHAW5 base
 * address registers at indices F7h-F4h, FBh-F8h, E3h-E0h, E7h-E4h and
 * EBh-E8h for memory access), which is where the firmware that programmed
 * the window keeps it. Several windows are usually enabled, so each
 * candidate is checked for plausible sensor data before it is accepted.
 *
 * The layout inside the window is defined by the platform firmware and
 * not by the controller: the Datasheet specifies the window mechanism but
 * no fan or temperature register block inside it. Different machines can
 * therefore lay the same window out differently, so a layout is tied to a
 * system identified by DMI and never selected by the controller ID. A
 * window whose contents do not match the selected layout is rejected
 * instead of being exposed as a sensor device.
 *
 * Fan readings are big endian, in RPM. Temperature readings use the first
 * byte of each pair and are in degrees Celsius.
 */

#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/hwmon.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define DRVNAME			"lenovo_shm_sensors"

/* Super-I/O configuration ports and registers */
#define SIO_PORT_2E		0x2e
#define SIO_PORT_4E		0x4e
#define SIO_REG_LDSEL		0x07
#define SIO_REG_DEVID		0x20
#define SIO_LD_SHM		0x0f

/* Sensor window geometry */
/*
 * Window span. The firmware maps at least 0x200 bytes from the base, but
 * only the first 0x100 are needed: the highest offset any layout uses is
 * 0x50 + 2 * LENOVO_SHM_MAX_FANS = 0x64.
 */
#define SHM_SIZE		0x100
#define LENOVO_SHM_MAX_FANS		10

/*
 * Plausibility limits used to tell the sensor window from the other
 * shared-memory windows the firmware also enables. A candidate that does
 * not look like sensor data is skipped, so the driver never exposes
 * garbage readings.
 */
#define TEMP_MAX_C		125
#define FAN_MAX_RPM		0x8000

/* SHAW1..SHAW5 base address registers, memory-window form */
static const u8 lenovo_shm_shaw_reg[] = { 0xf4, 0xf8, 0xe0, 0xe4, 0xe8 };

/*
 * Controller identification. The device variant is encoded in the lower
 * nibble of the Super-I/O ID, so the full 16-bit value is compared rather
 * than the SIO_ID_MASK-masked value nct6683 uses: masking would clear the
 * very bits that identify the part.
 */
struct lenovo_shm_chip {
	const char *name;
	u16 sid;
};

static const struct lenovo_shm_chip lenovo_shm_chips[] = {
	{ "NCT6801D", 0x0113 },
};

struct lenovo_shm_temp_ch {
	u8 reg;
	const char *label;
};

/* Window layout of one machine family */
struct lenovo_shm_layout {
	const struct lenovo_shm_temp_ch *temps;
	unsigned int ntemps;
	u8 fan_reg;
	unsigned int nfans;
	const char *const *fan_labels;	/* optional, may be NULL */
};

#define LENOVO_SHM_TEMP_CH(r, l)	{ .reg = (r), .label = (l) }

/*
 * Layout used by the Lenovo P3 TWR G2, taken from the SHWM field
 * definition in that platform's DSDT. This is the only layout known to be
 * correct today; a machine whose firmware lays the window out differently
 * fails the plausibility check below and gets no sensor device, so
 * readings are never taken from the wrong offsets.
 */
static const struct lenovo_shm_temp_ch lenovo_shm_temps_p3twr[] = {
	LENOVO_SHM_TEMP_CH(0x24, "Ambient"),	/* TD0 */
	LENOVO_SHM_TEMP_CH(0x26, "VCC_IA VR"),	/* TD1 */
	LENOVO_SHM_TEMP_CH(0x28, "Diode 2"),	/* TD2 */
	LENOVO_SHM_TEMP_CH(0x30, "CPU"),		/* CPT */
	LENOVO_SHM_TEMP_CH(0x32, "PCH"),		/* PHT */
};

static const char *const lenovo_shm_fans_p3twr[LENOVO_SHM_MAX_FANS] = {
	[0] = "Front",
	[4] = "Side",
	[5] = "CPU",
	[6] = "Rear",
};

enum lenovo_shm_system {
	LENOVO_SHM_P3TWR,
};

static const struct lenovo_shm_layout lenovo_shm_layout_p3twr = {
	.temps = lenovo_shm_temps_p3twr,
	.ntemps = ARRAY_SIZE(lenovo_shm_temps_p3twr),
	.fan_reg = 0x50,
	.nfans = LENOVO_SHM_MAX_FANS,
	.fan_labels = lenovo_shm_fans_p3twr,
};

static const struct lenovo_shm_layout * const lenovo_shm_layouts[] = {
	[LENOVO_SHM_P3TWR] = &lenovo_shm_layout_p3twr,
};

/*
 * The window layout comes from the platform firmware, so the system has
 * to be identified before the offsets can be trusted.
 */
static const struct dmi_system_id lenovo_shm_dmi_table[] = {
	{
		.ident = "LENOVO_P3TWR",
		.driver_data = (void *)(long)LENOVO_SHM_P3TWR,
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "LENOVO"),
			DMI_MATCH(DMI_BOARD_NAME, "106F"),
		},
	},
	{ }
};
MODULE_DEVICE_TABLE(dmi, lenovo_shm_dmi_table);

struct lenovo_shm_data {
	void __iomem *regs;
	const struct lenovo_shm_layout *layout;
};

/* ------------------------------------------------------------------ */
/* Super-I/O access                                                    */

static int lenovo_shm_sio_enter(int port)
{
	if (!request_muxed_region(port, 2, DRVNAME))
		return -EBUSY;

	outb(0x87, port);
	outb(0x87, port);

	return 0;
}

static void lenovo_shm_sio_exit(int port)
{
	outb(0xaa, port);
	release_region(port, 2);
}

static u8 lenovo_shm_sio_inb(int port, u8 reg)
{
	outb(reg, port);
	return inb(port + 1);
}

static void lenovo_shm_sio_select(int port, u8 ld)
{
	outb(SIO_REG_LDSEL, port);
	outb(ld, port + 1);
}

static u32 lenovo_shm_read_shaw(int port, u8 reg)
{
	return lenovo_shm_sio_inb(port, reg) |
	       (lenovo_shm_sio_inb(port, reg + 1) << 8) |
	       (lenovo_shm_sio_inb(port, reg + 2) << 16) |
	       ((u32)lenovo_shm_sio_inb(port, reg + 3) << 24);
}

static const struct lenovo_shm_chip *lenovo_shm_identify(int port)
{
	u16 sid;
	int i;

	sid = (lenovo_shm_sio_inb(port, SIO_REG_DEVID) << 8) |
	      lenovo_shm_sio_inb(port, SIO_REG_DEVID + 1);

	for (i = 0; i < ARRAY_SIZE(lenovo_shm_chips); i++) {
		if (lenovo_shm_chips[i].sid == sid)
			return &lenovo_shm_chips[i];
	}

	return NULL;
}

/* ------------------------------------------------------------------ */
/* Sensor window discovery                                             */

static bool lenovo_shm_window_valid(struct lenovo_shm_data *data)
{
	const struct lenovo_shm_layout *l = data->layout;
	unsigned int i;
	int sane_temps = 0;
	int sane_fans = 0;

	/*
	 * A channel reading 0xff means "not connected" on these ECs, which
	 * is normal early in boot and on SKUs without every sensor fitted,
	 * so require at least one plausible reading rather than all of
	 * them. Fans are counted the same way.
	 */
	for (i = 0; i < l->ntemps; i++) {
		u8 t = readb(data->regs + l->temps[i].reg);

		if (t != 0xff && t <= TEMP_MAX_C)
			sane_temps++;
	}

	for (i = 0; i < l->nfans; i++) {
		u16 rpm = (readb(data->regs + l->fan_reg + i * 2) << 8) |
			  readb(data->regs + l->fan_reg + i * 2 + 1);

		if (rpm > 0 && rpm < FAN_MAX_RPM)
			sane_fans++;
	}

	return sane_temps > 0 && sane_fans > 0;
}

static phys_addr_t lenovo_shm_find_window(const struct lenovo_shm_layout *layout,
					  const struct lenovo_shm_chip **chip)
{
	static const int ports[] = { SIO_PORT_2E, SIO_PORT_4E };
	struct lenovo_shm_data probe = { .layout = layout };
	u32 bases[ARRAY_SIZE(lenovo_shm_shaw_reg)];
	phys_addr_t found = 0;
	int p, i, n;

	for (p = 0; p < ARRAY_SIZE(ports) && !found; p++) {
		const struct lenovo_shm_chip *c;
		int port = ports[p];

		if (lenovo_shm_sio_enter(port))
			continue;

		/*
		 * This is a guard for the bus, not a model match: if the
		 * port carries some other Super-I/O chip, selecting LDN 0Fh
		 * and reading its registers would poke at unrelated
		 * hardware. The model itself was already chosen by DMI.
		 */
		c = lenovo_shm_identify(port);
		if (!c) {
			lenovo_shm_sio_exit(port);
			continue;
		}

		lenovo_shm_sio_select(port, SIO_LD_SHM);
		n = 0;
		for (i = 0; i < ARRAY_SIZE(lenovo_shm_shaw_reg); i++) {
			u32 base = lenovo_shm_read_shaw(port, lenovo_shm_shaw_reg[i]);

			/*
			 * SHAWnBA_3-0 == 0 means the window is disabled
			 * (Datasheet 5.3.3). The Core side WIN_BASEn
			 * register has its own disabled marker, but it is
			 * not reachable from the host, so the only further
			 * check available here is the content check below.
			 */
			if (base && base != 0xffffffff)
				bases[n++] = base;
		}
		lenovo_shm_sio_exit(port);

		/* Validate outside the configuration-region lock. */
		for (i = 0; i < n && !found; i++) {
			/*
			 * Claim the window only for the content check and drop
			 * it again right after; probe takes a devm-managed claim
			 * for the lifetime of the device.
			 */
			if (!request_mem_region(bases[i], SHM_SIZE, DRVNAME))
				continue;
			probe.regs = ioremap(bases[i], SHM_SIZE);
			if (probe.regs && lenovo_shm_window_valid(&probe)) {
				found = bases[i];
				*chip = c;
			}
			if (probe.regs)
				iounmap(probe.regs);
			release_mem_region(bases[i], SHM_SIZE);
		}
	}

	return found;
}

/* ------------------------------------------------------------------ */
/* hwmon interface                                                     */

static int lenovo_shm_read(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, long *val)
{
	struct lenovo_shm_data *data = dev_get_drvdata(dev);
	const struct lenovo_shm_layout *l = data->layout;
	u16 rpm;
	u8 t;

	switch (type) {
	case hwmon_fan:
		if (attr != hwmon_fan_input || channel >= l->nfans)
			return -EOPNOTSUPP;
		rpm = (readb(data->regs + l->fan_reg + channel * 2) << 8) |
		      readb(data->regs + l->fan_reg + channel * 2 + 1);
		if (rpm == 0xffff)
			return -ENODATA;
		*val = rpm;
		return 0;
	case hwmon_temp:
		if (attr != hwmon_temp_input || channel >= l->ntemps)
			return -EOPNOTSUPP;
		t = readb(data->regs + l->temps[channel].reg);
		if (t == 0xff)
			return -ENODATA;
		*val = t * 1000;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int lenovo_shm_read_string(struct device *dev, enum hwmon_sensor_types type,
				  u32 attr, int channel, const char **str)
{
	struct lenovo_shm_data *data = dev_get_drvdata(dev);
	const struct lenovo_shm_layout *l = data->layout;

	switch (type) {
	case hwmon_fan:
		if (attr != hwmon_fan_label || channel >= l->nfans ||
		    !l->fan_labels || !l->fan_labels[channel])
			return -EOPNOTSUPP;
		*str = l->fan_labels[channel];
		return 0;
	case hwmon_temp:
		if (attr != hwmon_temp_label || channel >= l->ntemps ||
		    !l->temps[channel].label)
			return -EOPNOTSUPP;
		*str = l->temps[channel].label;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t lenovo_shm_is_visible(const void *drvdata,
				     enum hwmon_sensor_types type,
				     u32 attr, int channel)
{
	const struct lenovo_shm_data *data = drvdata;

	switch (type) {
	case hwmon_fan:
		if (channel >= data->layout->nfans)
			return 0;
		if (attr == hwmon_fan_label &&
		    (!data->layout->fan_labels ||
		     !data->layout->fan_labels[channel]))
			return 0;
		return 0444;
	case hwmon_temp:
		if (channel >= data->layout->ntemps)
			return 0;
		if (attr == hwmon_temp_label &&
		    !data->layout->temps[channel].label)
			return 0;
		return 0444;
	default:
		return 0;
	}
}

static const struct hwmon_channel_info * const lenovo_shm_info[] = {
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL),
	NULL
};

static const struct hwmon_ops lenovo_shm_ops = {
	.is_visible = lenovo_shm_is_visible,
	.read = lenovo_shm_read,
	.read_string = lenovo_shm_read_string,
};

static const struct hwmon_chip_info lenovo_shm_chip_info = {
	.ops = &lenovo_shm_ops,
	.info = lenovo_shm_info,
};

/* ------------------------------------------------------------------ */
/* Platform glue                                                       */

static int lenovo_shm_probe(struct platform_device *pdev)
{
	const struct dmi_system_id *dmi_id;
	const struct lenovo_shm_layout *layout;
	const struct lenovo_shm_chip *chip = NULL;
	struct device *dev = &pdev->dev;
	struct lenovo_shm_data *data;
	struct device *hwmon;
	phys_addr_t base;
	unsigned long long faas;
	acpi_status status;

	dmi_id = dmi_first_match(lenovo_shm_dmi_table);
	if (!dmi_id)
		return -ENODEV;

	layout = lenovo_shm_layouts[(long)dmi_id->driver_data];
	if (!layout)
		return -ENODEV;

	/*
	 * The DSDT defines WMI1.WQD1..WQD5, which read this window under
	 * the MWMI mutex. They are gated by FAAS bit 3, which the BIOS
	 * controls. If that bit is set ACPI is reading the same bytes, so
	 * stay out; if the flag cannot be read at all, do not assume it is
	 * safe either.
	 */
	faas = 0;
	status = acpi_evaluate_integer(NULL, "\\WMI1.FAAS", NULL, &faas);
	if (ACPI_FAILURE(status) || (faas & BIT(3))) {
		dev_dbg(dev, "ACPI owns the EC sensor interface (FAAS %s%#llx)\n",
			ACPI_FAILURE(status) ? "unreadable " : "", faas);
		return -EBUSY;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->layout = layout;

	base = lenovo_shm_find_window(layout, &chip);
	if (!base) {
		dev_dbg(dev, "no supported EC sensor window found\n");
		return -ENODEV;
	}

	if (!devm_request_mem_region(dev, base, SHM_SIZE, DRVNAME))
		return -EBUSY;

	data->regs = devm_ioremap(dev, base, SHM_SIZE);
	if (!data->regs)
		return -ENOMEM;

	if (!lenovo_shm_window_valid(data)) {
		dev_dbg(dev, "rejecting implausible window at %pa\n", &base);
		return -ENODEV;
	}

	dev_info(dev, "%s EC sensor window at %pa\n",
		 chip ? chip->name : "Nuvoton", &base);
	platform_set_drvdata(pdev, data);

	hwmon = devm_hwmon_device_register_with_info(dev, DRVNAME, data,
						     &lenovo_shm_chip_info, NULL);
	return PTR_ERR_OR_ZERO(hwmon);
}

static struct platform_driver lenovo_shm_driver = {
	.driver = {
		.name = DRVNAME,
	},
	.probe = lenovo_shm_probe,
};

static struct platform_device *lenovo_shm_pdev;

static int __init lenovo_shm_init(void)
{
	/*
	 * Reject unsupported machines before creating the device, and let
	 * platform_create_bundle() run probe so that a failure there
	 * propagates and the module does not stay loaded without a bound
	 * device.
	 */
	if (!dmi_check_system(lenovo_shm_dmi_table))
		return -ENODEV;

	lenovo_shm_pdev = platform_create_bundle(&lenovo_shm_driver,
						 lenovo_shm_probe,
						 NULL, 0, NULL, 0);
	if (IS_ERR(lenovo_shm_pdev))
		return PTR_ERR(lenovo_shm_pdev);

	return 0;
}

static void __exit lenovo_shm_exit(void)
{
	platform_device_unregister(lenovo_shm_pdev);
	platform_driver_unregister(&lenovo_shm_driver);
}

module_init(lenovo_shm_init);
module_exit(lenovo_shm_exit);

MODULE_AUTHOR("Jiqi Li <lijq9@lenovo.com>");
MODULE_DESCRIPTION("Lenovo EC shared-memory sensor driver");
MODULE_LICENSE("GPL");
