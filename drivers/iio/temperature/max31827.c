#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>

static const struct iio_info max31827_info = {
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
