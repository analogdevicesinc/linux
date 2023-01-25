#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>

#define MAX31827_CONFIGURATION_1SHOT        BIT(0)
#define MAX31827_CONFIGURATION_CNV_RATE     GENMASK(3,1)
#define MAX31827_CONFIGURATION_PEC          BIT(4)
#define MAX31827_CONFIGURATION_TIMEOUT      BIT(5)
#define MAX31827_CONFIGURATION_RESOL        GENMASK(7,6)
#define MAX31827_CONFIGURATION_ALRM_POL     BIT(8)
#define MAX31827_CONFIGURATION_COMP         BIT(9)
#define MAX31827_CONFIGURATION_FLT_Q        GENMASK(11,10)
#define MAX31827_CONFIGURATION_PEC_ERR      BIT(13)
#define MAX31827_CONFIGURATION_U_TEMP_STAT  BIT(14)
#define MAX31827_CONFIGURATION_O_TEMP_STAT  BIT(15)


 
/* The MAX31827 registers */
#define MAX31827_T_MSB                      0x00
#define MAX31827_T_LSB                      0x01
#define MAX31827_CONFIGURATION              0x02
#define MAX31827_TH_MSB                     0x04
#define MAX31827_TH_LSB                     0x05
#define MAX31827_TL_MSB                     0x06
#define MAX31827_TL_LSB                     0x07
#define MAX31827_TH_HYST_MSB                0x08
#define MAX31827_TH_HYST_LSB                0x09
#define MAX31827_TL_HYST_MSB                0x0A
#define MAX31827_TL_HYST_LSB                0x0B

struct max31827_data {
}

static int adi_emu_read_raw(struct iio_dev *indio_dev,
                struct iio_chan_spec const *chan,
                int *val,
                int *val2,
                long mask)
{
    struct max31827_data *data = iio_priv(indio_dev);

    switch (mask) {
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
    struct max31827_data *data = iio_priv(indio_dev);

    switch (mask) {
    case IIO_CHAN_INFO_HYSTERESIS:
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
            BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_HYSTERESIS), 
    }, 
};

static int max31827_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
    struct iio_dev *indio_dev;
    struct max31827_data *data;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;
 
    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;
    
    data = iio_priv(indio_dev);
    data->enable = false;
 
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
