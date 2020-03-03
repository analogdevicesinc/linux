/*
 * ADI AXI-FMCADC5-SYNC Module
 *
 * Copyright 2016 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * https://wiki.analog.com/resources/fpga/docs/axi_fmcadc5_sync
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

/* register addresses & data (direct access) */

#define I5G_VERSION_ADDR			(0x0000 << 2)
#define I5G_IDENTIFIER_ADDR			(0x0001 << 2)
#define I5G_SCRATCH_ADDR			(0x0002 << 2)
#define I5G_TIMER_ADDR				(0x0003 << 2)
#define I5G_SPI_REQUEST_ADDR    		(0x0010 << 2)
#define I5G_SPI_GRANT_ADDR 			(0x0011 << 2)
#define I5G_SPI_SELECT_N_ADDR			(0x0012 << 2)
#define I5G_SPI_TRANSMIT_ADDR			(0x0013 << 2)
#define I5G_SPI_RECEIVE_ADDR 			(0x0014 << 2)
#define I5G_SPI_BUSY_ADDR    			(0x0015 << 2)
#define I5G_DELAY_ADDR				(0x0020 << 2)
#define I5G_DELAY_VERIFY_ADDR			(0x0021 << 2)
#define I5G_DELAY_LOCKED_ADDR			(0x0022 << 2)
#define I5G_SYNC_CONTROL_ADDR			(0x0030 << 2)
#define I5G_SYNC_STATUS_ADDR			(0x0031 << 2)
#define I5G_SYSREF_CONTROL_ADDR			(0x0040 << 2)
#define I5G_SYSREF_REQUEST_ADDR			(0x0041 << 2)
#define I5G_VCAL_CNT_ADDR			(0x0050 << 2)
#define I5G_VCAL_ENABLE_ADDR			(0x0051 << 2)
#define I5G_CAL_ENABLE_ADDR			(0x0060 << 2)
#define I5G_CAL_MAX_0_ADDR			(0x0064 << 2)
#define I5G_CAL_MIN_0_ADDR			(0x0065 << 2)
#define I5G_CAL_MAX_1_ADDR			(0x0066 << 2)
#define I5G_CAL_MIN_1_ADDR			(0x0067 << 2)
#define I5G_COR_ENABLE_ADDR			(0x0061 << 2)
#define I5G_COR_SCALE_0_ADDR			(0x0068 << 2)
#define I5G_COR_OFFSET_0_ADDR			(0x0069 << 2)
#define I5G_COR_SCALE_1_ADDR			(0x006a << 2)
#define I5G_COR_OFFSET_1_ADDR			(0x006b << 2)

/* register addresses & data (direct access) */

#define I5G_VERSION      			0x040063	/* version (4.0a) */
#define I5G_SPI_REQUEST_ACCESS  		0x000001 	/* request access */
#define I5G_SPI_REQUEST_RELEASE 		0x000000 	/* release access */
#define I5G_SPI_ACCESS_ENABLED			0x000001 	/* request enabled */
#define I5G_SPI_ACCESS_DISABLED			0x000000 	/* request disabled */
#define I5G_SPI_BUSY         			0x000001 	/* access busy */
#define I5G_DELAY_LOCKED     			0x000001 	/* delay is locked */
#define I5G_SYNC_SET              		0x000007 	/* dual mode and disabled */
#define I5G_SYNC_RELEASE          		0x000004 	/* dual mode and enabled */
#define I5G_SYNC_OOS              		0x000000 	/* out-of-sync */
#define I5G_SYSREF_SET              		0x000021 	/* one-shot and disabled */
#define I5G_SYSREF_RELEASE          		0x000020 	/* one-shot and enabled */
#define I5G_SYSREF_REQUEST          		0x000001 	/* sysref-request */
#define I5G_SYSREF_BUSY				0x000001 	/* sysref-busy */
#define I5G_VCAL_CNT_10M			0x000004	/* 100/((n+1)*2) */
#define I5G_VCAL_ENABLE				0x000001	/* enable */
#define I5G_VCAL_DISABLE			0x000000	/* enable */
#define I5G_CAL_ENABLE				0x000001	/* enable */
#define I5G_CAL_DISABLE				0x000000	/* enable */
#define I5G_COR_ENABLE				0x000001	/* enable */
#define I5G_COR_DISABLE				0x000000	/* enable */

/* register addresses & data (indirect access) */

#define	I5G_AD9625_ID_ADDR 			0x000001	/* identifier address */
#define	I5G_AD9625_ID_DATA 			0x000041	/* refer data sheet for details */
#define	I5G_AD9625_ST_ADDR 			0x000072	/* sysref time-stamp address [7:6] */
#define	I5G_AD9625_ST_DATA 			0x00008b	/* sysref timestamping enabled (2'b10) */
#define I5G_AD9625_SG_ADDR 			0x00013c	/* sysref guard band [7:5] */
#define I5G_AD9625_SS_ADDR 			0x000100	/* sysref status reporting [2] */
#define I5G_AD9625_SS_MASK 			0x000004 	/* setup violations mask */
#define I5G_AD9625_SS_SET  			0x000004 	/* setup violations detected */
#define I5G_AD9625_IO_ADDR 			0x0000ff	/* internal update address [0] */
#define I5G_AD9625_IO_DATA 			0x000001 	/* register update(1), self-clearing */
#define I5G_AD9625_SC_ADDR 			0x00003a	/* sysref control address */

/* [6] run(0)/clear(1), [3] @pos(0)/@neg(1), [2] continous(0)/one-shot(1), [1] disable(0)/enable(0) */

#define I5G_AD9625_SC_ENABLE(sel)  		((sel == 1) ? 0x00000e : 0x000006)
#define I5G_AD9625_SC_RECEIVED(sel)  		((sel == 1) ? 0x00000c : 0x000004)
#define I5G_AD9625_SC_CLEAR(sel)		((sel == 1) ? 0x00004e : 0x000046)

/* default is ms, we need finer delays (10ns) */

#define I5G_TIMER_US(d)				((d*100)-1)

/* state and such */

struct s_i5g {
	struct device		*dev;
	void __iomem		*regs;
	struct device_node 	*node;
	int			ad9625_cs_0;
	int			ad9625_cs_1;
	int			sysref_delay;
};

/* simple read */

static inline int i5g_read(struct s_i5g *st, int reg) {

	return(ioread32(st->regs + reg));
}

/* simple write */

static inline void i5g_write(struct s_i5g *st, int reg, int val) {

	iowrite32(val, st->regs + reg);
	return;
}

/* indirect access, returns within 32 clock cycles as long as a clock is present, no need to timeout */

static inline int i5g_spi_access(struct s_i5g *st, int data) {

	i5g_write(st, I5G_SPI_TRANSMIT_ADDR, data);
	while (i5g_read(st, I5G_SPI_BUSY_ADDR) == I5G_SPI_BUSY) {}
	return(i5g_read(st, I5G_SPI_RECEIVE_ADDR));
}

/* indirect read, straight forward just send 3 bytes and return the last byte */

static inline int i5g_spi_read(struct s_i5g *st, int sel, int reg) {

	int val;

	i5g_write(st, I5G_SPI_SELECT_N_ADDR, ~sel);
	val = i5g_spi_access(st, ((reg>>8) | 0x80));
	val = i5g_spi_access(st, reg);
	val = i5g_spi_access(st, 0);
	i5g_write(st, I5G_SPI_SELECT_N_ADDR, -1);
	return(val);
}

/* simple write, in this case just send 3 bytes */

static inline void i5g_spi_write(struct s_i5g *st, int sel, int reg, int val) {

	i5g_write(st, I5G_SPI_SELECT_N_ADDR, ~sel);
	i5g_spi_access(st, ((reg>>8) | 0x00));
	i5g_spi_access(st, reg);
	i5g_spi_access(st, val);
	i5g_write(st, I5G_SPI_SELECT_N_ADDR, -1);
	return;
}

/* device spi settings - clear & enable violations reporting */

static inline void i5g_ad9625_setup(struct s_i5g *st, int sel, int band) {

	/* clear the sysref violations */

	i5g_spi_write(st, sel, I5G_AD9625_SG_ADDR, (band << 5));
	i5g_spi_write(st, sel, I5G_AD9625_ST_ADDR, I5G_AD9625_ST_DATA);
	i5g_spi_write(st, sel, I5G_AD9625_SC_ADDR, I5G_AD9625_SC_CLEAR(sel));
	i5g_spi_write(st, sel, I5G_AD9625_IO_ADDR, I5G_AD9625_IO_DATA);

	/* enable the sysref violations */

	i5g_spi_write(st, sel, I5G_AD9625_SC_ADDR, I5G_AD9625_SC_ENABLE(sel));
	i5g_spi_write(st, sel, I5G_AD9625_IO_ADDR, I5G_AD9625_IO_DATA);
	return;
}

/* device spi status - sysref received and violations we need */

static inline int i5g_ad9625_status(struct s_i5g *st, int sel, int band, int status) {

	int data;

	/* sysref received */

	data = i5g_spi_read(st, sel, I5G_AD9625_SC_ADDR);
	if (data != I5G_AD9625_SC_RECEIVED(sel)) {
		dev_info(st->dev, "sysref status mismatch (%d, %d, %x)!\n", sel, band, data);
	}

	/* sysref violations */

	data = i5g_spi_read(st, sel, I5G_AD9625_SS_ADDR);
	if ((data & I5G_AD9625_SS_MASK) == I5G_AD9625_SS_SET) {
		return(status | (0x1 << band));
	}

	return(status);
}

/* check violation band, mostly a gimmick function- but helps you understand the logic of synchronization */

static inline int i5g_status_check(int status) {

	if (status == 0x3e) return(0); /* sysref at or within 235ps */
	if (status == 0x3c) return(0); /* sysref at or within 270ps */
	if (status == 0x38) return(0); /* sysref at or within 305ps */

	return(-1);
}

/* the mean thing that brutally overtakes everything else to synchronize the devices for interleaving */
/* if you need to resync, try re-entry to this function */
/* if handling differently by individual components, this is the part you need to copy over */
/* remove probe based return codes, let user space fix the data path and re-enter again */

static int i5g_intlv(struct s_i5g *st) {

	int delay;
	int band;
	int status_1;
	int status_2;
	int data;
	int timeout;

	/* make sure everyone else is in the game and has individually achieved sync */

	data = i5g_read(st, I5G_SYNC_STATUS_ADDR);
	if (data == I5G_SYNC_OOS) {
		dev_info(st->dev, "link out-of-sync (%x), deferring probe!\n", data);
		return(-1);
	}

	/* request indirect access (this overrides the default access) */

	timeout = 100;
	i5g_write(st, I5G_SPI_REQUEST_ADDR, I5G_SPI_REQUEST_ACCESS);
	while (i5g_read(st, I5G_SPI_GRANT_ADDR) == I5G_SPI_ACCESS_DISABLED) {
		timeout = timeout - 1;
		if (timeout == 0) {
			dev_info(st->dev, "request for device access denied!\n");
			return(0);
		}
	}

	/* check we got the right devices */

	data = i5g_spi_read(st, st->ad9625_cs_0, I5G_AD9625_ID_ADDR);
	if (data != I5G_AD9625_ID_DATA) {
		dev_info(st->dev, "unsupported device (%x) at (%d)!\n", data, st->ad9625_cs_0);
		return(0);
	}

	data = i5g_spi_read(st, st->ad9625_cs_1, I5G_AD9625_ID_ADDR);
	if (data != I5G_AD9625_ID_DATA) {
		dev_info(st->dev, "unsupported device (%x) at (%d)!\n", data, st->ad9625_cs_1);
		return(0);
	}

	/* check delay is locked */

	data = i5g_read(st, I5G_DELAY_LOCKED_ADDR);
	if (data != I5G_DELAY_LOCKED) {
		dev_info(st->dev, "sysref delay controller is out-of-lock (%d)!\n", data);
		return(0);
	}

	/* so far so good, let's try synchronization */
	/* set dual mode, this allows both devices to see the same sync (importantly deassertion) */
	/* disable syncs, bring the link down (back to cgs) and initialize delay */

	i5g_write(st, I5G_SYNC_CONTROL_ADDR, I5G_SYNC_SET);
	i5g_write(st, I5G_SYSREF_CONTROL_ADDR, I5G_SYSREF_SET);

	/* quick facts about clocks & sysref and all this mess! */
	/* device clock is 2.5G, fpga gets 625M, recovered clock is 156.25M */
	/* 625M is derived from the device 2.5G clock */
	/* 156.25 is derived from the recoverd (CDR) clock */
	/* sysref is generated by fpga(!) at 156.25M */
	/* we need to make sure that device 1 samples sysref first followed by device 2 */
	/* sysref must meet setup time of device 1 only (device 2 follows) */
	/* ad9625 features a sysref monitoring routine that tells us where it is w.r.t the 2.5G clock */
	/* 2.5G == 400 ps, sysref setup requirement is 200ps (not a good candidate is it?), hold is -100ps */
	/* i.e. actual sysref is 100ps, but has a significant internal delay (guessing ~150ps or so) */
	/* most likely chance to miss the edge, (remember there is still a possibility of device 2 getting it) */
	/* ideally you want the sysref setup time to be less than half the sampling clock in this case */
	/* so we swap the edges and allow us a full 400 ps for showing up before the sampling edge */
	/* ignore hold, sysref is being generated by a 156.25M clock (/16) */
	/* move sysref until we find the ideal spot that hit the devices, at that point get out */

	st->sysref_delay = -1;
	for (delay = 0; delay < 32; delay++) {

		/* change the delay */

		i5g_write(st, I5G_DELAY_ADDR, delay);
		data = i5g_read(st, I5G_DELAY_VERIFY_ADDR);
		if (data != delay) {
			dev_info(st->dev, "delay data mismatch(%d, %d)!\n", delay, data);
		}

		/* change the guard band (does not affect actual timing) */

		status_1 = 0;
		status_2 = 0;
		for (band = 0; band < 6; band++) {

			/* get the devices ready */

			i5g_ad9625_setup(st, st->ad9625_cs_0, band);
			i5g_ad9625_setup(st, st->ad9625_cs_1, band);

			/* send the sysref */

			i5g_write(st, I5G_SYSREF_REQUEST_ADDR, I5G_SYSREF_REQUEST);
			while (i5g_read(st, I5G_SYSREF_REQUEST_ADDR) == I5G_SYSREF_BUSY) {}

			/* check the sysref violations */

			status_1 = i5g_ad9625_status(st, st->ad9625_cs_0, band, status_1);
			status_2 = i5g_ad9625_status(st, st->ad9625_cs_1, band, status_2);
		}

		/* all we need is to keep the sysref edge close to the sampling clock edge */
		/* here we are keeping sysref within 305ps ~ 235ps */
		/* if you are experimenting, walk this through and print the bands */

		if ((i5g_status_check(status_1) == 0) && (i5g_status_check(status_2) == 0)) {
			st->sysref_delay = delay;
			dev_info(st->dev, "sysref synchronization @%d, status(%02x, %02x)!\n", delay, status_1, status_2);
			break;
		}
	}

	/* set delay, enable syncs back and check status */

	i5g_write(st, I5G_SYNC_CONTROL_ADDR, I5G_SYNC_RELEASE);
	i5g_write(st, I5G_SYSREF_CONTROL_ADDR, I5G_SYSREF_RELEASE);

	/* give it some time */

	mdelay(1);

	/* is anything wrong? */

	data = i5g_read(st, I5G_SYNC_STATUS_ADDR);
	if (data == I5G_SYNC_OOS) {
		dev_info(st->dev, "resync failed, may need to reset the transceiver chain (%x)!\n", data);
	}

	/* release indirect access (this overrides the default access) */

	timeout = 100;
	i5g_write(st, I5G_SPI_REQUEST_ADDR, I5G_SPI_REQUEST_RELEASE);
	while (i5g_read(st, I5G_SPI_GRANT_ADDR) == I5G_SPI_ACCESS_ENABLED) {
		timeout = timeout - 1;
		if (timeout == 0) {
			dev_info(st->dev, "request for device release failed!\n");
			return(0);
		}
	}

	/* that's all folks! */

	return(0);
}

/* export interleaving to user space */

static ssize_t i5g_intlv_u(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {

	struct s_i5g *st;

	st = dev_get_drvdata(dev);
	i5g_intlv(st);
	return(count);
}

static DEVICE_ATTR(i5g_intlv, S_IWUSR, NULL, i5g_intlv_u);

/* calibrate the data paths, poor man's version (see hdl for more details) */
/* during boot, the lowest frequency component must be present at full scale */
/* assumes a solid linear frequency response on all the discrete components (500K to 9G) */
/* make this function re-entrant if needs to be called again */
/* remove probe based return codes, let user space fix the data path and re-enter again */

static int i5g_calibrate(struct s_i5g *st) {

	int n;
	int16_t cal_max_0;
	int16_t cal_min_0;
	int16_t cal_max_1;
	int16_t cal_min_1;
	int16_t cal_offset_0;
	int16_t cal_offset_1;
	uint16_t cal_scale_0;
	uint16_t cal_scale_1;

	/* calibrate gain and offset */

	i5g_write(st, I5G_VCAL_ENABLE_ADDR, I5G_VCAL_ENABLE);
	i5g_write(st, I5G_COR_ENABLE_ADDR, I5G_COR_DISABLE);

	/* loop to get an average */

	cal_max_0 = 0;
	cal_min_0 = 0;
	cal_max_1 = 0;
	cal_min_1 = 0;

	for (n = 0; n < 16; n++) {

		i5g_write(st, I5G_CAL_ENABLE_ADDR, I5G_CAL_ENABLE);
		i5g_write(st, I5G_TIMER_ADDR, I5G_TIMER_US(100));
		while (i5g_read(st, I5G_TIMER_ADDR) != 0) {}

		i5g_write(st, I5G_CAL_ENABLE_ADDR, I5G_CAL_DISABLE);
		if (i5g_read(st, I5G_CAL_ENABLE_ADDR) != I5G_CAL_DISABLE) {
			dev_info(st->dev, "calibration failed, expect mismatch!\n");
		}

		cal_max_0 = cal_max_0 + i5g_read(st, I5G_CAL_MAX_0_ADDR);
		cal_min_0 = cal_min_0 + i5g_read(st, I5G_CAL_MIN_0_ADDR);
		cal_max_1 = cal_max_1 + i5g_read(st, I5G_CAL_MAX_1_ADDR);
		cal_min_1 = cal_min_1 + i5g_read(st, I5G_CAL_MIN_1_ADDR);
	}

	/* if you keep dither, need to filter out post adc-core */

	i5g_write(st, I5G_VCAL_ENABLE_ADDR, I5G_VCAL_DISABLE);
	i5g_write(st, I5G_COR_ENABLE_ADDR, I5G_COR_DISABLE);

	/* peaks -divide or shift? */

	cal_max_0 = cal_max_0/16;
	cal_min_0 = cal_min_0/16;
	cal_max_1 = cal_max_1/16;
	cal_min_1 = cal_min_1/16;

	/* offsets -divide or shift? -multiply or subtract? */

	cal_offset_0 = -1*((cal_max_0 + cal_min_0)/2);
	cal_offset_1 = -1*((cal_max_1 + cal_min_1)/2);

	i5g_write(st, I5G_COR_OFFSET_0_ADDR, cal_offset_0);
	i5g_write(st, I5G_COR_OFFSET_1_ADDR, cal_offset_1);
	i5g_write(st, I5G_COR_ENABLE_ADDR, I5G_COR_ENABLE);

	/* scale */

	cal_scale_0 = (uint16_t)(cal_max_0 - cal_min_0);
	cal_scale_1 = (uint16_t)(cal_max_1 - cal_min_1);

	if (cal_scale_1 > cal_scale_0) {
		cal_scale_0 = ((uint32_t)(cal_scale_1*32768))/cal_scale_0;
		cal_scale_1 = 32768;
	} else {
		cal_scale_1 = ((uint32_t)(cal_scale_0*32768))/cal_scale_1;
		cal_scale_0 = 32768;
	}

	i5g_write(st, I5G_COR_SCALE_0_ADDR, cal_scale_0);
	i5g_write(st, I5G_COR_SCALE_1_ADDR, cal_scale_1);
	i5g_write(st, I5G_COR_ENABLE_ADDR, I5G_COR_ENABLE);

	/* fyi */

	dev_info(st->dev, "calibration values (0) (%d, %d)!\n", cal_max_0, cal_min_0);
	dev_info(st->dev, "correction values (0) (%d, %d)!\n", cal_offset_0, cal_scale_0);
	dev_info(st->dev, "calibration values (1) (%d, %d)!\n", cal_max_1, cal_min_1);
	dev_info(st->dev, "correction values (1) (%d, %d)!\n", cal_offset_1, cal_scale_1);

	return(0);
}

/* export calibration to user space */

static ssize_t i5g_calibrate_u(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {

	struct s_i5g *st;

	st = dev_get_drvdata(dev);
	i5g_calibrate(st);
	return(count);
}

static DEVICE_ATTR(i5g_calibrate, S_IWUSR, NULL, i5g_calibrate_u);

/* match table for of_platform binding */

static const struct of_device_id fmcadc5_sync_of_match[] = {
	{ .compatible = "adi,axi-fmcadc5-sync-1.0", .data = (void *) 1},
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, fmcadc5_sync_of_match);

/* probe function */

static int fmcadc5_sync_probe(struct platform_device *pdev) {

	struct device_node *np = pdev->dev.of_node;
	struct s_i5g *st;
	struct resource *mem;
	int data;

	/* get the structure ready */

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st) {
		dev_err(&pdev->dev, "not enough memory for device!\n");
		return(-ENOMEM);
	}

	/* read settings from device tree */

	of_property_read_u32(np, "adi,ad9625-device-select-0", &st->ad9625_cs_0);
	of_property_read_u32(np, "adi,ad9625-device-select-1", &st->ad9625_cs_1);

	/* get the key & open the doors to the hardware */

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs)) {
		return PTR_ERR(st->regs);
	}

	st->dev = &pdev->dev;
	platform_set_drvdata(pdev, st);
	dev_info(&pdev->dev, "HW address 0x%08llX mapped to 0x%p\n", (unsigned long long)mem->start, st->regs);

	/* check version, give up if not an exact match */

	data = i5g_read(st, I5G_VERSION_ADDR);
	if (data != I5G_VERSION) {
		dev_err(&pdev->dev, "unsupported core version (%x)!\n", data);
		return(-ENODEV);
	}

	/* interleave & calibrate */

	if (i5g_intlv(st) != 0) {
		return(-EPROBE_DEFER);
	}

	i5g_calibrate(st);

	device_create_file(&pdev->dev, &dev_attr_i5g_calibrate);
	device_create_file(&pdev->dev, &dev_attr_i5g_intlv);

	/* done (at least all that we could do), goodbye */

	return(0);
}

/* remove and release resources */

static int fmcadc5_sync_remove(struct platform_device *pdev) {

	/* anything to do here - free some stuff? */

	return(0);
}

/* register driver */

static struct platform_driver fmcadc5_sync_of_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = fmcadc5_sync_of_match,
	},
	.probe		= fmcadc5_sync_probe,
	.remove		= fmcadc5_sync_remove,
};
module_platform_driver(fmcadc5_sync_of_driver);

/* cosmetic stuff */

MODULE_AUTHOR("Rejeesh Kutty <rejeesh.kutty@analog.com>");
MODULE_DESCRIPTION("Analog Devices AXI-FMCADC5-SYNC (I5G) Module");
MODULE_LICENSE("GPL v2");

