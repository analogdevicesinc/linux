// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADE9078 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/gpio/consumer.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/spi/spi.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>

#include <linux/module.h>
#include <linux/moduleparam.h>
#include "ade9078.h"

/*
 * struct ade9078_device - ade9078 specific data
 * @lock		mutex for the device
 * @gpio_reset	reset gpio pointer, retrieved from DT
 * @irq0_bits	IRQ0 mask and status bits, are set by the driver and are passed
 * 				to the IC after being set
 * @irq1_bits	IRQ1 mask and status bits, are set by the driver and are passed
 * 				to the IC after being set
 * @rst_done	flag for when reset sequence irq has been received
 * @wf_mode		wave form buffer mode, read datasheet for more details,
 * 				retrieved from DT
 * @wfb_trg_cfg	wave form buffer triger configuration, read datasheet for more
 * 				details, retrieved from DT
 * @spi 		spi device associated to the ade9078
 * @tx			transmit buffer for the spi
 * @rx			receive buffer for the spi
 * @tx_buff		transmit buffer for the iio buffer trough spi, used in iio
 * 				buffer configuration
 * @rx_buff		receive buffer for the iio buffer trough spi, will contain the
 * 				samples from the IC wave form buffer
 * @xfer		transfer setup used in iio buffer configuration
 * @spi_msg		message transfer trough spi, used in iio buffer configuration
 * @regmap		register map pointer
 * @indio_dev:	the IIO device
 * @trig		iio trigger pointer, is connected to IRQ0 and IRQ1
 */
struct ade9078_device {
	struct mutex lock;
	struct gpio_desc *gpio_reset;
	u32 irq0_bits;
	u32 irq1_bits;
	bool rst_done;
	u8 wf_mode;
	u16 wfb_trg_cfg;
	struct spi_device *spi;
	u8 *tx;
	u8 *rx;
	u8 tx_buff[2];
	u8 rx_buff[ADE9078_WFB_FULL_BUFF_SIZE] ____cacheline_aligned;
	struct spi_transfer	xfer[2];
	struct spi_message spi_msg;
	struct regmap *regmap;
	struct iio_dev *indio_dev;
	struct iio_trigger *trig;
};

static const struct iio_event_spec ade9078_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE) |
						BIT(IIO_EV_INFO_VALUE),
	},
};

//IIO channels of the ade9078 for each phase individually
static const struct iio_chan_spec ade9078_a_channels[] = {
		ADE9078_CHANNEL(ADE9078_PHASE_A_NR, ADE9078_PHASE_A_NAME),
};
static const struct iio_chan_spec ade9078_b_channels[] = {
		ADE9078_CHANNEL(ADE9078_PHASE_B_NR, ADE9078_PHASE_B_NAME),
};
static const struct iio_chan_spec ade9078_c_channels[] = {
		ADE9078_CHANNEL(ADE9078_PHASE_C_NR, ADE9078_PHASE_C_NAME),
};

/*
 * ade9078_spi_write_reg() - ade9078 write register over SPI
 * the data format for communicating with the ade9078 over SPI
 * is very specific
 * @context:	void pointer to the SPI device
 * @reg:		address of the of desired register
 * @val:  		value to be written to the ade9078
 */
static int ade9078_spi_write_reg(void *context, unsigned int reg,
		unsigned int val)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct ade9078_device *ade9078_dev = spi_get_drvdata(spi);

	u16 addr;
	int ret = 0;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = ade9078_dev->tx,
			.bits_per_word = 8,
			.len = 6,
		},
	};

	addr = ADE9078_WRITE_REG(reg);

	mutex_lock(&ade9078_dev->lock);
	ade9078_dev->tx[5] = (u8)  val & 0xFF;
	ade9078_dev->tx[4] = (u8) (val >> 8) & 0xFF;
	ade9078_dev->tx[3] = (u8) (val >> 16) & 0xFF;
	ade9078_dev->tx[2] = (u8) (val >> 24) & 0xFF;
	ade9078_dev->tx[1] = (u8) addr;
	ade9078_dev->tx[0] = (u8) (addr >> 8);

	//registers which are 16 bits
	if(reg > 0x480 && reg < 0x4FE)
	{
		ade9078_dev->tx[3] = (u8)  val & 0xFF;
		ade9078_dev->tx[2] = (u8) (val >> 8) & 0xFF;
		xfer[0].len = 4;
	}

	ret = spi_sync_transfer(ade9078_dev->spi, xfer, ARRAY_SIZE(xfer));
	if (ret)
	{
		dev_err(&ade9078_dev->spi->dev, "problem when writing register 0x%x",
				reg);
	}

	mutex_unlock(&ade9078_dev->lock);
	return ret;
}

/*
 * ade9078_spi_write_reg() - ade9078 read register over SPI
 * the data format for communicating with the ade9078 over SPI
 * is very specific
 * @context:	void pointer to the SPI device
 * @reg:		address of the of desired register
 * @val:  		value to be read to the ade9078
 */
static int ade9078_spi_read_reg(void *context, unsigned int reg,
		unsigned int *val)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct ade9078_device *ade9078_dev = spi_get_drvdata(spi);

	u16 addr;
	int ret = 0;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = ade9078_dev->tx,
			.bits_per_word = 8,
			.len = 2,
		},
		{
			.rx_buf = ade9078_dev->rx,
			.bits_per_word = 8,
			.len = 6,
		},
	};

	addr = ADE9078_READ_REG(reg);

	mutex_lock(&ade9078_dev->lock);
	ade9078_dev->tx[1] = (u8) addr;
	ade9078_dev->tx[0] = (u8) (addr >> 8);

	//registers which are 16 bits
	if(reg > 0x480 && reg < 0x4FE)
		xfer[1].len = 4;

	ret = spi_sync_transfer(ade9078_dev->spi, xfer, ARRAY_SIZE(xfer));
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "problem when reading register 0x%x",
				reg);
		goto err_ret;
	}

	//registers which are 16 bits
	if(reg > 0x480 && reg < 0x4FE)
		*val = (ade9078_dev->rx[0] << 8) | ade9078_dev->rx[1];
	else
		*val = (ade9078_dev->rx[0] << 24) | (ade9078_dev->rx[1] << 16) |
		(ade9078_dev->rx[2] << 8) | ade9078_dev->rx[3];

err_ret:
	mutex_unlock(&ade9078_dev->lock);
	return ret;
}

/*
 * ade9078_en_wfb() - enables or disables the WFBuffer in the ADE9078
 * @ade9078_dev:		ade9078 device data
 * @state:				true for enabled; false for disabled
 */
static int ade9078_en_wfb(struct ade9078_device *ade9078_dev, bool state)
{
	if(state)
		return regmap_update_bits(ade9078_dev->regmap, ADDR_WFB_CFG,
				BIT_MASK(4), BIT_MASK(4));
	else
		return regmap_update_bits(ade9078_dev->regmap, ADDR_WFB_CFG,
				BIT_MASK(4), 0);
}

/*
 * ade9078_update_mask0() - updates interrupt mask0 and resets all of the status
 * 							register 0
 * @ade9078_dev:		ade9078 device data
 */
static int ade9078_update_mask0(struct ade9078_device *ade9078_dev)
{
	unsigned int ret;

	ret = regmap_write(ade9078_dev->regmap, ADDR_STATUS0,
			0xFFFFFFFF);
	if (ret)
		return ret;

	ret = regmap_write(ade9078_dev->regmap, ADDR_MASK0, ade9078_dev->irq0_bits);
	if (ret)
		return ret;

	return 0;
}

/*
 * ade9078_update_mask0() - updates interrupt mask1 and resets all of the status
 * 							register 1
 * @ade9078_dev:		ade9078 device data
 */
static int ade9078_update_mask1(struct ade9078_device *ade9078_dev)
{
	unsigned int ret;

	ret = regmap_write(ade9078_dev->regmap, ADDR_STATUS1,
			0xFFFFFFFF);
	if (ret)
		return ret;

	ret = regmap_write(ade9078_dev->regmap, ADDR_MASK1, ade9078_dev->irq1_bits);
	if (ret)
		return ret;

	return 0;
}

/*
 * ade9078_test_bits() - tests the bits of a given register within the IC
 * @map:				regmap device
 * @reg:				register to be tested
 * @bits:				bits to be checked
 *
 * Returns 0 if at least one of the tested bits is not set, 1 if all tested
 * bits are set and a negative error number if the underlying regmap_read()
 * fails.
 */
static int ade9078_test_bits(struct regmap *map, unsigned int reg,
		unsigned int bits)
{
	unsigned int val, ret;

	ret = regmap_read(map, reg, &val);
	if (ret)
		return ret;

	return (val & bits) == bits;
}

/*
 * ade9078_irq0_handler() - handler for IRQ0. A hand-off for the threaded
 * 							handler
 */
static irqreturn_t ade9078_irq0_handler(int irq, void *data)
{
	struct ade9078_device *ade9078_dev = data;

	dev_info(&ade9078_dev->spi->dev, "IRQ0 Interrupted");

	return IRQ_WAKE_THREAD;
}

/*
 * ade9078_irq0_thread() - Thread for IRQ0. It reads Status register 0 and
 * checks for the IRQ activation. This is configured to acquire samples in to
 * the IC buffer and dump it in to the iio_buffer according to Stop When Buffer
 * Is Full Mode, Stop Filling on Trigger and Capture Around Trigger from the
 * ADE9078 Datasheet
 */
static irqreturn_t ade9078_irq0_thread(int irq, void *data)
{
	struct ade9078_device *ade9078_dev = data;
	u32 status;
	u32 handled_irq = 0;

	regmap_read(ade9078_dev->regmap, ADDR_STATUS0, &status);
	dev_info(&ade9078_dev->spi->dev, "IRQ0 status 0x%x", status);

	if(((status & ADE9078_ST0_PAGE_FULL) == ADE9078_ST0_PAGE_FULL) &&
	((ade9078_dev->irq0_bits & ADE9078_ST0_PAGE_FULL) ==
			ADE9078_ST0_PAGE_FULL))
	{
		//Stop Filling on Trigger and Center Capture Around Trigger
		if(ade9078_dev->wf_mode){
			regmap_write(ade9078_dev->regmap, ADDR_WFB_TRG_CFG,
					ade9078_dev->wfb_trg_cfg);
			ade9078_dev->irq0_bits |= ADE9078_ST0_WFB_TRIG_IRQ;
		}
		else{
			//Stop When Buffer Is Full Mode
			ade9078_en_wfb(ade9078_dev, false);
			iio_trigger_poll(ade9078_dev->trig);
		}

		//disable Page full interrupt
		ade9078_dev->irq0_bits &= ~ADE9078_ST0_PAGE_FULL;
		regmap_write(ade9078_dev->regmap, ADDR_MASK0,
				ade9078_dev->irq0_bits);

		dev_info(&ade9078_dev->spi->dev, "IRQ0 ADE9078_ST0_PAGE_FULL");
		handled_irq |= ADE9078_ST0_PAGE_FULL;
	}

	if(((status & ADE9078_ST0_WFB_TRIG_IRQ) == ADE9078_ST0_WFB_TRIG_IRQ) &&
	((ade9078_dev->irq0_bits & ADE9078_ST0_WFB_TRIG_IRQ) ==
			ADE9078_ST0_WFB_TRIG_IRQ))
	{
		//Stop Filling on Trigger and Center Capture Around Trigger
		ade9078_en_wfb(ade9078_dev, false);
		iio_trigger_poll(ade9078_dev->trig);

		handled_irq |= ADE9078_ST0_WFB_TRIG_IRQ;
		dev_info(&ade9078_dev->spi->dev, "IRQ0 ADE9078_ST0_WFB_TRIG_IRQ");
	}

	regmap_write(ade9078_dev->regmap, ADDR_STATUS0, handled_irq);

	dev_info(&ade9078_dev->spi->dev, "IRQ0 thread done");

	return IRQ_HANDLED;
}

/*
 * ade9078_irq1_handler() - handler for IRQ1. A hand-off for the threaded
 * 							handler
 */
static irqreturn_t ade9078_irq1_handler(int irq, void *data)
{
	struct ade9078_device *ade9078_dev = data;

	dev_info(&ade9078_dev->spi->dev, "IRQ1 Interrupted");

	return IRQ_WAKE_THREAD;
}

/*
 * ade9078_irq1_thread() - Thread for IRQ1. It reads Status register 1 and
 * checks for the IRQ activation. This thread handles the reset condition and
 * the zero-crossing conditions for all 3 phases on Voltage and Current
 */
static irqreturn_t ade9078_irq1_thread(int irq, void *data)
{
	struct ade9078_device *ade9078_dev = data;
	struct iio_dev *indio_dev = ade9078_dev->indio_dev;
	int result;
	u32 status;
	s64 timestamp = iio_get_time_ns(indio_dev);

	//reset
	if(ade9078_dev->rst_done == false){
		result = ade9078_test_bits(ade9078_dev->regmap, ADDR_STATUS1,
				ADE9078_ST1_RSTDONE);
		if(result < 0)
			dev_err(&ade9078_dev->spi->dev, "Error testing reset done");
		else if(result == 1)
			ade9078_dev->rst_done = true;
		dev_info(&ade9078_dev->spi->dev, "IRQ1 Reset");
		goto irq_done;
	}

	regmap_read(ade9078_dev->regmap, ADDR_STATUS1, &status);

	//crossings
	if((((status & ADE9078_ST1_ZXVA) == ADE9078_ST1_ZXVA) ||
	   ((status & ADE9078_ST1_ZXTOVA) == ADE9078_ST1_ZXTOVA)) &&
	   ((ade9078_dev->irq1_bits & ADE9078_ST1_ZXVA) == ADE9078_ST1_ZXVA))
	{
		iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							ADE9078_PHASE_A_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_EITHER),
				   timestamp);
		dev_info(&ade9078_dev->spi->dev, "IRQ1 ADE9078_ST1_ZXVA");
	}

	if((((status & ADE9078_ST1_ZXVB) == ADE9078_ST1_ZXVB) ||
	   ((status & ADE9078_ST1_ZXTOVB) == ADE9078_ST1_ZXTOVB)) &&
	   ((ade9078_dev->irq1_bits & ADE9078_ST1_ZXVB) == ADE9078_ST1_ZXVB))
	{
		iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							ADE9078_PHASE_B_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_EITHER),
				   timestamp);
		dev_info(&ade9078_dev->spi->dev, "IRQ1 ADE9078_ST1_ZXVB");
	}

	if((((status & ADE9078_ST1_ZXVC) == ADE9078_ST1_ZXVC) ||
	   ((status & ADE9078_ST1_ZXTOVC) == ADE9078_ST1_ZXTOVC)) &&
	   ((ade9078_dev->irq1_bits & ADE9078_ST1_ZXVC) == ADE9078_ST1_ZXVC))
	{
		iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							ADE9078_PHASE_C_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_EITHER),
				   timestamp);
		dev_info(&ade9078_dev->spi->dev, "IRQ1 ADE9078_ST1_ZXVC");
	}

irq_done:
	dev_info(&ade9078_dev->spi->dev, "IRQ1 thread done");
	return IRQ_HANDLED;
}

/*
 * ade9078_pop_wfb() - parses the SPI receive buffer, rearranges
 * the bits and pushes the data to the IIO buffer
 */
static void ade9078_pop_wfb(struct iio_poll_func *pf)
{
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	u32 i;
	u32 data;

	for(i=0; i <= ADE9078_WFB_FULL_BUFF_NR_SAMPLES; i=i+4)
	{
		data = get_unaligned_be32(&ade9078_dev->rx_buff[i]);
		iio_push_to_buffers(ade9078_dev->indio_dev, &data);
	}

	dev_info(&ade9078_dev->spi->dev, "Pushed to buffer");
}

/*
 * ade9078_trigger_handler() - the bottom half of the pollfunc
 * for the iio trigger buffer. It acquires data trough the SPI
 * and rearranges the data to match BE
 */
static irqreturn_t ade9078_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	int ret;

	dev_info(&ade9078_dev->spi->dev, "Triggered");
	if(bitmap_empty(indio_dev->active_scan_mask, indio_dev->masklength))
	{
		dev_err(&ade9078_dev->spi->dev, "Bitmap empty in trigger handler");
		goto err_out;
	}

	mutex_lock(&ade9078_dev->lock);
	ret = spi_sync(ade9078_dev->spi, &ade9078_dev->spi_msg);
	if(ret)
	{
		mutex_unlock(&ade9078_dev->lock);
		dev_err(&ade9078_dev->spi->dev, "SPI fail in trigger handler");
		goto err_out;
	}

	ade9078_pop_wfb(pf);

	mutex_unlock(&ade9078_dev->lock);

err_out:

	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

/*
 * ade9078_configure_scan() - sets up the transfer parameters
 * as well as the tx and rx buffers
 * @indio_dev:		the IIO device
 */
static int ade9078_configure_scan(struct iio_dev *indio_dev)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	int ret = 0;
	u16 addr;

	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;

	addr = ADE9078_READ_REG(ADDR_WF_BUFF);

	ade9078_dev->tx_buff[1] = (u8) addr;
	ade9078_dev->tx_buff[0] = (u8) (addr >> 8);

	ade9078_dev->xfer[0].tx_buf = &ade9078_dev->tx_buff[0];
	ade9078_dev->xfer[0].bits_per_word = 8;
	ade9078_dev->xfer[0].len = 2;

	ade9078_dev->xfer[1].rx_buf = &ade9078_dev->rx_buff[0];
	ade9078_dev->xfer[1].bits_per_word = 8;
	ade9078_dev->xfer[1].len = ADE9078_WFB_FULL_BUFF_SIZE;

	spi_message_init(&ade9078_dev->spi_msg);
	spi_message_add_tail(&ade9078_dev->xfer[0], &ade9078_dev->spi_msg);
	spi_message_add_tail(&ade9078_dev->xfer[1], &ade9078_dev->spi_msg);

	return ret;
}

/*
 * ade9078_read_raw() - IIO read function
 * @indio_dev:		the IIO device
 * @chan:			channel specs of the ade9078
 * @val:			first half of the read value
 * @val2:			second half of the read value
 * @mask:			info mask of the channel
 */
static int ade9078_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	int ret;
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = regmap_read(ade9078_dev->regmap, chan->address, val);

		iio_device_release_direct_mode(indio_dev);

		return IIO_VAL_INT;
		break;

	case IIO_CHAN_INFO_SCALE:
		switch(chan->type) {
		case IIO_CURRENT:
			if(chan->address >= ADDR_AI_PCF && chan->address <= ADDR_CI_PCF){
				*val = 1;
				*val2 = ADE9078_PCF_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			}
			if(chan->address >= ADDR_AIRMS && chan->address <= ADDR_CIRMS){
				*val = 1;
				*val2 = ADE9078_RMS_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			}
			break;
		case IIO_VOLTAGE:
			if(chan->address >= ADDR_AV_PCF && chan->address <= ADDR_CV_PCF){
				*val = 1;
				*val2 = ADE9078_PCF_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			}
			if(chan->address >= ADDR_AVRMS && chan->address <= ADDR_CVRMS){
				*val = 1;
				*val2 = ADE9078_RMS_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			}
			break;
		case IIO_POWER:
			*val = 1;
			*val2 = ADE9000_WATT_FULL_SCALE_CODES;
			return IIO_VAL_FRACTIONAL;
			break;
		default: break;
		}
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

/*
 * ade9078_write_raw() - IIO write function
 * @indio_dev:		the IIO device
 * @chan:			channel specs of the ade9078
 * @val:			first half of the written value
 * @val2:			second half of the written value
 * @mask:			info mask of the channel
 */
static int ade9078_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
//	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
//		ade9078_en_wfb(ade9078_dev, val);
		return 0;
	}

	return -EINVAL;
}

/*
 * ade9078_reg_acess() - IIO debug register access
 * @indio_dev:		the IIO device
 * @reg:			register to be accessed
 * @tx_val:			value to be transmitted
 * @rx_val:			value to be received
 */
static int ade9078_reg_acess(struct iio_dev *indio_dev,
		unsigned int reg,
		unsigned int tx_val,
		unsigned int *rx_val)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);

	if (rx_val)
		return regmap_read(ade9078_dev->regmap, reg, rx_val);
	else
		return regmap_write(ade9078_dev->regmap, reg, tx_val);
}

/*
 * ade9078_write_event_config() - IIO event configure to enable zero-crossing
 * and zero-crossing timeout on voltage and current for each phases. These
 * events will also influence the trigger conditions for the buffer capture.
 */
static int ade9078_write_event_config(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir,
				int state)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	u32 number;
	u32 status1 = 0;
	int ret;

	dev_info(&ade9078_dev->spi->dev, "Enter event");

	number = chan->channel;
	dev_info(&ade9078_dev->spi->dev, "Event channel %d", number);
	switch(number){
	case ADE9078_PHASE_A_NR:
		if(chan->type == IIO_VOLTAGE){
			if(state){
				ade9078_dev->irq1_bits |= ADE9078_ST1_ZXVA | ADE9078_ST1_ZXTOVA;
				ade9078_dev->wfb_trg_cfg |= BIT(6);
				dev_info(&ade9078_dev->spi->dev, "ZXVA set");
			}
			else{
				ade9078_dev->irq1_bits &= ~ADE9078_ST1_ZXVA &
										  ~ADE9078_ST1_ZXTOVA;
				ade9078_dev->wfb_trg_cfg &= ~BIT(6);
				dev_info(&ade9078_dev->spi->dev, "ZXVA cleared");
			}
		}
		else if(chan->type == IIO_CURRENT){
			if(state){
				ade9078_dev->irq1_bits |= ADE9078_ST1_ZXIA;
				ade9078_dev->wfb_trg_cfg |= BIT(3);
			}
			else {
				ade9078_dev->irq1_bits &= ~ADE9078_ST1_ZXIA;
				ade9078_dev->wfb_trg_cfg &= ~BIT(3);
			}
		}
		break;
	case ADE9078_PHASE_B_NR:
		if(chan->type == IIO_VOLTAGE){
			if(state){
				ade9078_dev->irq1_bits |= ADE9078_ST1_ZXVB | ADE9078_ST1_ZXTOVB;
				ade9078_dev->wfb_trg_cfg |= BIT(7);
				dev_info(&ade9078_dev->spi->dev, "ZXVB set");
			}
			else{
				ade9078_dev->irq1_bits &= ~ADE9078_ST1_ZXVB &
										  ~ADE9078_ST1_ZXTOVB;
				ade9078_dev->wfb_trg_cfg &= ~BIT(7);
				dev_info(&ade9078_dev->spi->dev, "ZXVB cleared");
			}
		}
		else if(chan->type == IIO_CURRENT){
			if(state){
				ade9078_dev->irq1_bits |= ADE9078_ST1_ZXIB;
				ade9078_dev->wfb_trg_cfg |= BIT(4);
			}
			else{
				ade9078_dev->irq1_bits &= ~ADE9078_ST1_ZXIB;
				ade9078_dev->wfb_trg_cfg &= ~BIT(4);
			}
		}
		break;
	case ADE9078_PHASE_C_NR:
		if(chan->type == IIO_VOLTAGE){
			if(state){
				ade9078_dev->irq1_bits |= ADE9078_ST1_ZXVC | ADE9078_ST1_ZXTOVC;
				ade9078_dev->wfb_trg_cfg |= BIT(8);
				dev_info(&ade9078_dev->spi->dev, "ZXVC set");
			}
			else{
				ade9078_dev->irq1_bits &= ~ADE9078_ST1_ZXVC &
										  ~ADE9078_ST1_ZXTOVC;
				ade9078_dev->wfb_trg_cfg &= ~BIT(8);
				dev_info(&ade9078_dev->spi->dev, "ZXVB cleared");
			}
		}
		else if(chan->type == IIO_CURRENT){
			if(state){
				ade9078_dev->irq1_bits |= ADE9078_ST1_ZXIC;
				ade9078_dev->wfb_trg_cfg |= BIT(5);
			}
			else{
				ade9078_dev->irq1_bits &= ~ADE9078_ST1_ZXIC;
				ade9078_dev->wfb_trg_cfg &= ~BIT(5);
			}
		}
		break;
	default:
		break;
	}

	ret = ade9078_update_mask1(ade9078_dev);
	if(ret)
		return ret;

	//clear relevant status
	status1 |= ADE9078_ST1_ZXVA | ADE9078_ST1_ZXTOVA |
			   ADE9078_ST1_ZXVB | ADE9078_ST1_ZXTOVB |
			   ADE9078_ST1_ZXVC | ADE9078_ST1_ZXTOVC |
			   ADE9078_ST1_ZXIA | ADE9078_ST1_ZXIB | ADE9078_ST1_ZXIC;
	regmap_write(ade9078_dev->regmap, ADDR_STATUS1, status1);

	return 0;
}

/*
 * ade9078_read_event_vlaue() - Outputs the result of the zero-crossing for
 * voltage and current for each phase.
 * Result:
 * 0 - if crossing event not set
 * 1 - if crossing event occurred
 * -1 - if crossing timeout (only for Voltages)
 */
static int ade9078_read_event_vlaue(struct iio_dev *indio_dev,
	      const struct iio_chan_spec *chan,
	      enum iio_event_type type,
	      enum iio_event_direction dir,
	      enum iio_event_info info,
	      int *val, int *val2)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	u32 number;
	u32 status1;
	u32 handled_irq1 = 0;
	int ret;

	ret = regmap_read(ade9078_dev->regmap, ADDR_STATUS1, &status1);
	if (ret)
		return ret;

	dev_info(&ade9078_dev->spi->dev, "Read event status1 0x%x", status1);

	*val = 0;

	number = chan->channel;
	switch(number){
	case ADE9078_PHASE_A_NR:
		if(chan->type == IIO_VOLTAGE){
			if((status1 & ADE9078_ST1_ZXVA) == ADE9078_ST1_ZXVA){
				*val = 1;
				handled_irq1 |= ADE9078_ST1_ZXVA;
			}
			else if((status1 & ADE9078_ST1_ZXTOVA) == ADE9078_ST1_ZXTOVA){
				*val = -1;
				handled_irq1 |= ADE9078_ST1_ZXTOVA;
			}
			if((ade9078_dev->irq1_bits & ADE9078_ST1_ZXTOVA) !=
					ADE9078_ST1_ZXTOVA)
				*val = 0;
		}
		else if(chan->type == IIO_CURRENT){
			if((status1 & ADE9078_ST1_ZXIA) == ADE9078_ST1_ZXIA){
				handled_irq1 |= ADE9078_ST1_ZXIA;
			}

		}
		break;
	case ADE9078_PHASE_B_NR:
		if(chan->type == IIO_VOLTAGE){
			if((status1 & ADE9078_ST1_ZXVB) == ADE9078_ST1_ZXVB){
				*val = 1;
				handled_irq1 |= ADE9078_ST1_ZXVB;
			}
			else if((status1 & ADE9078_ST1_ZXTOVB) == ADE9078_ST1_ZXTOVB){
				*val = -1;
				handled_irq1 |= ADE9078_ST1_ZXTOVB;
			}
			if((ade9078_dev->irq1_bits & ADE9078_ST1_ZXTOVB) !=
					ADE9078_ST1_ZXTOVB)
				*val = 0;
		}
		else if(chan->type == IIO_CURRENT){
			if((status1 & ADE9078_ST1_ZXIB) == ADE9078_ST1_ZXIB){
				handled_irq1 |= ADE9078_ST1_ZXIB;
			}

		}
		break;
	case ADE9078_PHASE_C_NR:
		if(chan->type == IIO_VOLTAGE){
			if((status1 & ADE9078_ST1_ZXVC) == ADE9078_ST1_ZXVC){
				*val = 1;
				handled_irq1 |= ADE9078_ST1_ZXVC;
			}
			else if((status1 & ADE9078_ST1_ZXTOVC) == ADE9078_ST1_ZXTOVC){
				*val = -1;
				handled_irq1 |= ADE9078_ST1_ZXTOVC;
			}
			if((ade9078_dev->irq1_bits & ADE9078_ST1_ZXTOVC) !=
					ADE9078_ST1_ZXTOVC)
				*val = 0;
		}
		else if(chan->type == IIO_CURRENT){
			if((status1 & ADE9078_ST1_ZXIC) == ADE9078_ST1_ZXIC){
				handled_irq1 |= ADE9078_ST1_ZXIC;
			}

		}
		break;
	default:
		break;
	}

	regmap_write(ade9078_dev->regmap, ADDR_STATUS1, handled_irq1);
	dev_info(&ade9078_dev->spi->dev, "Read event handled_irq1 0x%x", handled_irq1);
	return IIO_VAL_INT;
}

/*
 * ade9078_config_wfb() - reads the ade9078 node and configures the wave form
 * buffer based on the options set. Additionally is reads the active scan mask
 * in order to set the input data of the buffer. There are only a few available
 * input configurations permitted by the IC, any unpermitted configuration will
 * result in all channels being active.
 * @indio_dev:		the IIO device
 */
static int ade9078_config_wfb(struct iio_dev *indio_dev)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	u32 wfg_cfg_val = 0;
	u32 tmp;
	int ret;

	bitmap_to_arr32(&wfg_cfg_val, indio_dev->active_scan_mask,
			indio_dev->masklength);

	switch(wfg_cfg_val)
	{
	case ADE9078_SCAN_POS_IA | ADE9078_SCAN_POS_VA :
		wfg_cfg_val = 0x1;
		break;
	case ADE9078_SCAN_POS_IB | ADE9078_SCAN_POS_VB :
		wfg_cfg_val = 0x2;
		break;
	case ADE9078_SCAN_POS_IC | ADE9078_SCAN_POS_VC :
		wfg_cfg_val = 0x3;
		break;
	case ADE9078_SCAN_POS_IA :
		wfg_cfg_val = 0x8;
		break;
	case ADE9078_SCAN_POS_VA :
		wfg_cfg_val = 0x9;
		break;
	case ADE9078_SCAN_POS_IB :
		wfg_cfg_val = 0xA;
		break;
	case ADE9078_SCAN_POS_VB :
		wfg_cfg_val = 0xB;
		break;
	case ADE9078_SCAN_POS_IC :
		wfg_cfg_val = 0xC;
		break;
	case ADE9078_SCAN_POS_VC :
		wfg_cfg_val = 0xD;
		break;
	default:
		wfg_cfg_val = 0x0;
		break;
	}

	ret = of_property_read_u32((&ade9078_dev->spi->dev)->of_node,
			"adi,wf-cap-sel", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get wf-cap-sel: %d\n", ret);
		return ret;
	}
	wfg_cfg_val |= tmp << 5;

	ret = of_property_read_u32((&ade9078_dev->spi->dev)->of_node,
			"adi,wf-mode", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get wf-mode: %d\n", ret);
		return ret;
	}
	wfg_cfg_val |= tmp << 6;
	ade9078_dev->wf_mode = tmp;

	ret = of_property_read_u32((&ade9078_dev->spi->dev)->of_node,
			"adi,wf-src", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get wf-src: %d\n", ret);
		return ret;
	}
	wfg_cfg_val |= tmp << 8;

	ret = of_property_read_u32((&ade9078_dev->spi->dev)->of_node,
			"adi,wf-in-en", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get wf-in-en: %d\n", ret);
		return ret;
	}
	wfg_cfg_val |= tmp << 12;

	return regmap_write(ade9078_dev->regmap, ADDR_WFB_CFG, wfg_cfg_val);
}

/*
 * ade9078_wfb_interrupt_setup() - Configures the wave form buffer interrupt
 * according to modes
 * @ade9078_dev:		ade9078 device data
 * @mode:				modes according to datasheet; values [0-2]
 *
 * This sets the interrupt register and other registers related to the
 * interrupts according to mode [0-2] from the datasheet
 */
static int ade9078_wfb_interrupt_setup(struct ade9078_device *ade9078_dev,
		u8 mode)
{
	int ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_WFB_TRG_CFG, 0x0);
	if(ret)
		return ret;

	if(mode == 1 || mode == 0)
	{
		ret = regmap_write(ade9078_dev->regmap, ADDR_WFB_PG_IRQEN, 0x8000);
		if(ret)
			return ret;
	}
	else if(mode == 2)
	{
		ret = regmap_write(ade9078_dev->regmap, ADDR_WFB_PG_IRQEN, 0x80);
		if(ret)
			return ret;

	}

	ade9078_dev->irq0_bits |= ADE9078_ST0_PAGE_FULL;

	return ret;
}

/*
 * ade9078_buffer_preenable() - configures the wave form buffer
 * @indio_dev:		the IIO device
 */
static int ade9078_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	int ret;

	ret = ade9078_config_wfb(indio_dev);
	if (ret)
		return ret;

	switch (ade9078_dev->wf_mode)
	{
	case 0:
		ret = ade9078_wfb_interrupt_setup(ade9078_dev, 0);
		break;
	case 1:
		ret = ade9078_wfb_interrupt_setup(ade9078_dev, 1);
		break;
	case 2:
		ret = ade9078_wfb_interrupt_setup(ade9078_dev, 2);
		break;
	default:
		break;
	}

	return ret;
}

/*
 * ade9078_buffer_postenable() - after the IIO is enabled
 * this will enable the ade9078 internal buffer for acquisition
 * @indio_dev:		the IIO device
 */
static int ade9078_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	int ret;

	ret = ade9078_update_mask0(ade9078_dev);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Post-enable update mask0 fail");
		return ret;
	}
	ret = ade9078_en_wfb(ade9078_dev, true);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Post-enable wfb enable fail");
		return ret;
	}

	return ret;
}

/*
 * ade9078_buffer_postdisable() - after the iio is disable
 * this will disable the ade9078 internal buffer for acquisition
 * @indio_dev:		the IIO device
 */
static int ade9078_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	int ret;

	ret = ade9078_en_wfb(ade9078_dev, false);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Post-disable wfb disable fail");
		return ret;
	}

	regmap_write(ade9078_dev->regmap, ADDR_WFB_TRG_CFG,
			0x0);

	ade9078_dev->irq0_bits &= ~ADE9078_ST0_WFB_TRIG_IRQ &
							  ~ADE9078_ST0_PAGE_FULL;
	ret = ade9078_update_mask0(ade9078_dev);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Post-disable update maks0 fail");
		return ret;
	}

	return ret;
}

/*
 * ade9078_phase_gain_offset_setup() - reads the gain and offset for
 * I, V and P from the device-tree for each phase and sets them in the
 * respective registers
 * @ade9078_dev:		ade9078 device data
 * @phase_node: 		phase node in the device-tree
 * @phase_nr:			the number attributed to each phase, this also
 * 						represents the phase register offset
 */
static int ade9078_phase_gain_offset_setup(struct ade9078_device *ade9078_dev,
		struct device_node *phase_node, u32 phase_nr)
{
	int ret;
	u32 tmp;

	ret = of_property_read_u32(phase_node, "adi,igain", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get igain: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_AIGAIN,phase_nr), tmp);
	if(ret)
		return ret;

	ret = of_property_read_u32(phase_node, "adi,vgain", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get vgain: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_AVGAIN,phase_nr), tmp);
	if(ret)
		return ret;

	ret = of_property_read_u32(phase_node, "adi,irmsos", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get irmsos: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_AIRMSOS,phase_nr), tmp);
	if(ret)
		return ret;

	ret = of_property_read_u32(phase_node, "adi,vrmsos", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get vrmsos: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_AVRMSOS,phase_nr), tmp);
	if(ret)
		return ret;

	ret = of_property_read_u32(phase_node, "adi,pgain", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get pgain: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_APGAIN,phase_nr), tmp);
	if(ret)
		return ret;

	ret = of_property_read_u32(phase_node, "adi,wattos", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get wattos: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_AWATTOS,phase_nr), tmp);
	if(ret)
		return ret;

	ret = of_property_read_u32(phase_node, "adi,varos", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get varos: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_AVAROS,phase_nr), tmp);
	if(ret)
		return ret;

	ret = of_property_read_u32(phase_node, "adi,fvaros", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get fvaros: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_AFVAROS,phase_nr), tmp);
	if(ret)
		return ret;

	return 0;
}

/*
 * ade9078_setup_iio_channels() - parses the phase nodes of the device-tree and
 * creates the iio channels based on the active phases in the DT. Each phase
 * has its own I, V and P channels with individual gain and offset parameters.
 * @ade9078_dev:		ade9078 device data
 */
static int ade9078_setup_iio_channels(struct ade9078_device *ade9078_dev)
{
	struct iio_chan_spec *chan;
	struct device_node *phase_node = NULL;
	u32 phase_nr;
	u32 chan_size = 0;
	int ret = 0;

	chan = devm_kcalloc(&ade9078_dev->spi->dev,
			ADE9078_MAX_PHASE_NR*ARRAY_SIZE(ade9078_a_channels),
			sizeof(*ade9078_a_channels), GFP_KERNEL);
	if(chan == NULL) {
		dev_err(&ade9078_dev->spi->dev,"Unable to allocate ADE9078 channels");
		return -ENOMEM;
	}
	ade9078_dev->indio_dev->num_channels = 0;
	ade9078_dev->indio_dev->channels = chan;

	for_each_available_child_of_node((&ade9078_dev->spi->dev)->of_node, phase_node){
		if (!of_node_name_eq(phase_node, "phase"))
			continue;

		ret = of_property_read_u32(phase_node, "reg", &phase_nr);
		if (ret) {
			dev_err(&ade9078_dev->spi->dev, "Could not read channel reg : %d\n",
					ret);
			goto put_phase_node;
		}

		switch(phase_nr)
		{
		case ADE9078_PHASE_A_NR:
			memcpy(chan, ade9078_a_channels, sizeof(ade9078_a_channels));
			chan_size = ARRAY_SIZE(ade9078_a_channels);
			ade9078_phase_gain_offset_setup(ade9078_dev, phase_node,
					ADE9078_PHASE_A_NR);
			break;
		case ADE9078_PHASE_B_NR:
			memcpy(chan, ade9078_b_channels, sizeof(ade9078_b_channels));
			chan_size = ARRAY_SIZE(ade9078_b_channels);
			ade9078_phase_gain_offset_setup(ade9078_dev, phase_node,
					ADE9078_PHASE_B_NR);
			break;
		case ADE9078_PHASE_C_NR:
			memcpy(chan, ade9078_c_channels, sizeof(ade9078_c_channels));
			chan_size = ARRAY_SIZE(ade9078_c_channels);
			ade9078_phase_gain_offset_setup(ade9078_dev, phase_node,
					ADE9078_PHASE_C_NR);
			break;
		default:
			break;
		}

		chan += chan_size;
		ade9078_dev->indio_dev->num_channels += chan_size;
	}

put_phase_node:
	of_node_put(phase_node);
	return ret;
}

/*
 * ade9078_reset() - Reset sequence for the ADE9078
 * @ade9078_dev:		ade9078 device data
 */
static int ade9078_reset(struct ade9078_device *ade9078_dev)
{
	ade9078_dev->rst_done = false;

	gpiod_set_value_cansleep(ade9078_dev->gpio_reset, 1);
	usleep_range(1, 100);
	gpiod_set_value_cansleep(ade9078_dev->gpio_reset, 0);
	msleep_interruptible(50);

	if(ade9078_dev->rst_done == false)
		return -EPERM;
	else
		return 0;
}

/*
 * ade9078_setup() - initial register setup of the ade9078
 * @ade9078_dev:		ade9078 device data
 */
static int ade9078_setup(struct ade9078_device *ade9078_dev)
{
	int ret = 0;

	dev_info(&ade9078_dev->spi->dev, "Setup started");
	ret = regmap_write(ade9078_dev->regmap, ADDR_PGA_GAIN, ADE9078_PGA_GAIN);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_CONFIG0, ADE9078_CONFIG0);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_CONFIG1, ADE9078_CONFIG1);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_CONFIG2, ADE9078_CONFIG2);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_CONFIG3, ADE9078_CONFIG3);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_ACCMODE, ADE9078_ACCMODE);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_ZX_LP_SEL, ADE9078_ZX_LP_SEL);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_MASK0, ADE9078_MASK0);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_MASK1, ADE9078_MASK1);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_EVENT_MASK, ADE9078_EVENT_MASK);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_WFB_CFG, ADE9078_WFB_CFG);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_VLEVEL, ADE9078_VLEVEL);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_DICOEFF, ADE9078_DICOEFF);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_EGY_TIME, ADE9078_EGY_TIME);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_EP_CFG, ADE9078_EP_CFG);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_RUN, ADE9078_RUN_ON);
	if(ret)
		return ret;

	msleep_interruptible(2);

	ret = regmap_write(ade9078_dev->regmap, ADDR_STATUS0,
				0xFFFFFFFF);
	if (ret)
		return ret;

	ret = regmap_write(ade9078_dev->regmap, ADDR_STATUS1,
				0xFFFFFFFF);
	if (ret)
		return ret;

	dev_info(&ade9078_dev->spi->dev, "Setup finished");

	return ret;
}


static const struct iio_trigger_ops ade9078_trigger_ops = {
		.validate_device = iio_trigger_validate_own_device,
};

static const struct iio_buffer_setup_ops ade9078_buffer_ops = {
	.preenable = &ade9078_buffer_preenable,
	.postenable = &ade9078_buffer_postenable,
	.postdisable = &ade9078_buffer_postdisable,
};

static const struct iio_info ade9078_info = {
	.read_raw = &ade9078_read_raw,
	.write_raw = &ade9078_write_raw,
	.debugfs_reg_access = &ade9078_reg_acess,
	.write_event_config = &ade9078_write_event_config,
	.read_event_value = &ade9078_read_event_vlaue,
};

/*
 * Regmap configuration
 * The register access of the ade9078 requires a 16 bit address
 * with the read flag on bit 3. This is not supported by default
 * regmap functionality, thus reg_read and reg_write have been
 * replaced with custom functions
 */
static const struct regmap_config ade9078_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	//reg_read and write require the use of devm_regmap_init
	//instead of devm_regmap_init_spi
	.reg_read = ade9078_spi_read_reg,
	.reg_write = ade9078_spi_write_reg,
	.zero_flag_mask = true,
};

static int ade9078_probe(struct spi_device *spi)
{
	struct ade9078_device *ade9078_dev;
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct iio_trigger *trig;
	struct gpio_desc *gpio_reset;
	int irq;

	unsigned long irqflags = 0;
	int ret = 0;

	printk(KERN_INFO "Enter ade9078_probe\n");

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*ade9078_dev));
	if(indio_dev == NULL) {
		dev_err(&spi->dev,"Unable to allocate ADE9078 IIO");
		return -ENOMEM;
	}
	ade9078_dev = iio_priv(indio_dev);
	if(ade9078_dev == NULL) {
		dev_err(&spi->dev,"Unable to allocate ADE9078 device structure");
		return -ENOMEM;
	}

	ade9078_dev->rx = devm_kcalloc(&spi->dev, 6, sizeof(*ade9078_dev->rx),
			GFP_KERNEL);
	if(ade9078_dev->rx == NULL)	{
		dev_err(&spi->dev,"Unable to allocate ADE9078 RX Buffer");
		return-ENOMEM;
	}
	ade9078_dev->tx = devm_kcalloc(&spi->dev, 10, sizeof(*ade9078_dev->tx),
			GFP_KERNEL);
	if(ade9078_dev->tx == NULL) {
		dev_err(&spi->dev,"Unable to allocate ADE9078 TX Buffer");
		return -ENOMEM;
	}
	regmap = devm_regmap_init(&spi->dev, NULL, spi, &ade9078_regmap_config);
	if (IS_ERR(regmap))	{
		dev_err(&spi->dev,"Unable to allocate ADE9078 regmap");
		return PTR_ERR(regmap);
	}
	spi_set_drvdata(spi, ade9078_dev);

	ade9078_dev->rst_done = false;

	irq = of_irq_get_byname((&spi->dev)->of_node, "irq0");
	if (irq < 0){
		dev_err(&spi->dev,"Unable to find irq0");
		return -EINVAL;
	}
	irqflags = irq_get_trigger_type(irq);
	ret = devm_request_threaded_irq(&spi->dev, irq, ade9078_irq0_handler,
			ade9078_irq0_thread, irqflags, KBUILD_MODNAME, ade9078_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to request threaded irq: %d\n", ret);
		return ret;
	}

	irq = of_irq_get_byname((&spi->dev)->of_node, "irq1");
	if (irq < 0){
		dev_err(&spi->dev,"Unable to find irq1");
		return -EINVAL;
	}
	irqflags = irq_get_trigger_type(irq);
	ret = devm_request_threaded_irq(&spi->dev, irq, ade9078_irq1_handler,
			ade9078_irq1_thread, irqflags, KBUILD_MODNAME, ade9078_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to request threaded irq: %d\n", ret);
		return ret;
	}

	trig = devm_iio_trigger_alloc(&spi->dev, "%s-dev%d", KBUILD_MODNAME,
			indio_dev->id);
	if (!trig){
		dev_err(&spi->dev,"Unable to allocate ADE9078 trigger");
		return -ENOMEM;
	}
	iio_trigger_set_drvdata(trig, ade9078_dev);

	gpio_reset = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (!gpio_reset){
		dev_err(&spi->dev,"Unable to allocate ADE9078 reset");
		return -ENOMEM;
	}

	ade9078_dev->spi = spi;
	ade9078_dev->spi->mode = SPI_MODE_0;
	spi_setup(ade9078_dev->spi);

	indio_dev->name = KBUILD_MODNAME;
	indio_dev->dev.parent = &ade9078_dev->spi->dev;
	indio_dev->info = &ade9078_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ade9078_dev->regmap = regmap;
	ade9078_dev->indio_dev = indio_dev;
	ade9078_setup_iio_channels(ade9078_dev);

	ade9078_dev->trig = trig;
	ade9078_dev->trig->dev.parent = &ade9078_dev->spi->dev;
	ade9078_dev->trig->ops = &ade9078_trigger_ops;

	ade9078_dev->gpio_reset = gpio_reset;
	ade9078_dev->wfb_trg_cfg = 0;

	mutex_init(&ade9078_dev->lock);

	ade9078_configure_scan(indio_dev);

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
			&iio_pollfunc_store_time, &ade9078_trigger_handler,
			&ade9078_buffer_ops);
	if (ret) {
		dev_err(&spi->dev, "Failed to setup triggered buffer: %d\n", ret);
		return ret;
	}

	ret = iio_trigger_register(trig);
	if (ret)
	{
		dev_err(&spi->dev,"Unable to register ADE9078 trigger");
		return ret;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Unable to register IIO device");
		return ret;
	}

	ret = ade9078_reset(ade9078_dev);
	if(ret){
		dev_err(&spi->dev, "ADE9078 reset failed");
		return ret;
	}
	dev_info(&spi->dev, "Reset done");

	ret = ade9078_setup(ade9078_dev);
	if (ret) {
		dev_err(&spi->dev, "Unable to setup ADE9078");
		return ret;
	}

	return ret;
};

static int ade9078_remove(struct spi_device *spi)
{
	struct ade9078_device *ade9078_dev = spi_get_drvdata(spi);
	struct iio_dev *indio_dev = ade9078_dev->indio_dev;

	printk(KERN_INFO "Exit ade9078_probe\n");
	ade9078_dev->trig->dev.parent = NULL;
	iio_trigger_unregister(ade9078_dev->trig);
	iio_device_unregister(indio_dev);

	return 0;
}

static const struct spi_device_id ade9078_id[] = {
		{"ade9078", 0},
		{}
};
static struct spi_driver ade9078_driver = {
		.driver = {
				.name = "ade9078",
		},
		.probe = ade9078_probe,
		.remove = ade9078_remove,
		.id_table = ade9078_id,
};

module_spi_driver(ade9078_driver);

MODULE_AUTHOR("Ciprian Hegbeli <ciprian.hegbeli@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADE9078 High Performance, Polyphase Energy Metering IC Driver");
MODULE_LICENSE("GPL v2");
