#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>

static int adi_emu_read_raw(struct iio_dev *indio_dev,
                struct iio_chan_spec const *chan,
                int *val,
                int *val2,
                long mask)
{
    switch (mask) {
    case IIO_CHAN_INFO_ENABLE:
        *val = 0;
        return IIO_VAL_INT;
    case IIO_CHAN_INFO_RAW:
        if (chan->channel)
            *val = 1;
        else
            *val = 0;
        return IIO_VAL_INT;
    }
 
    return -EINVAL;
}

static int adi_emu_write_raw(struct iio_dev *indio_dev,
                 struct iio_chan_spec const *chan,
                 int val,
                 int val2,
                 long mask)
{
    switch (mask) {
    case IIO_CHAN_INFO_ENABLE:
        return 0;
    }
 
    return -EINVAL;
}

static const struct iio_info max31827_info = {
    .read_raw = &max31827_read_raw,
    .write_raw = &max31827_write_raw,
};

// check this
static const struct iio_chan_spec max31827_channels[] = {
    {
        .type = IIO_TEMP,
        .info_mask_shared_by_all = 
            BIT(IIO_CHAN_INFO_ENABLE) | BIT(IIO_CHAN_INFO_RAW), /*I might need additional stuff here*/
        .output = 0,
    }, 
};

static int max31827_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
    struct iio_dev *indio_dev;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;
 
    indio_dev = devm_iio_device_alloc(&client->dev, 0);
    if (!indio_dev)
        return -ENOMEM;
 
    indio_dev->name = "iio-max31827";
    indio_dev->info = &max31827_info;

    indio_dev->channels = max31827_channels;
    indio_dev->num_channels = ARRAY_SIZE(max31827_channels);
 
    return iio_device_register(indio_dev);
}

static struct i2c_driver max31827_driver = {
    .driver = {
        .name = "iio-max31827",
    },
    .probe = max31827_probe,
};
module_i2c_driver(max31827_driver);

MODULE_AUTHOR("Daniel Matyas <daniel.matyas@analog.com>");
MODULE_DESCRIPTION("Maxim MAX31827 low-power temperature switch driver");
MODULE_LICENSE("GPL");
