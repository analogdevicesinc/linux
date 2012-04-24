#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>


#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#define SPI_XCOMM_SETTINGS_3WIRE		BIT(6)
#define SPI_XCOMM_SETTINGS_CS_HIGH	BIT(5)
#define SPI_XCOMM_SETTINGS_SAMPLE_END	BIT(4)
#define SPI_XCOMM_SETTINGS_CPHA		BIT(3)
#define SPI_XCOMM_SETTINGS_CPOL		BIT(2)
#define SPI_XCOMM_SETTINGS_CLOCK_DIV_MASK 0x3
#define SPI_XCOMM_SETTINGS_CLOCK_DIV_64	0x2
#define SPI_XCOMM_SETTINGS_CLOCK_DIV_16	0x1
#define SPI_XCOMM_SETTINGS_CLOCK_DIV_4	0x0

#define SPI_XCOMM_CMD_UPDATE_CONFIG	0x03
#define SPI_XCOMM_CMD_WRITE		0x04

unsigned bus_num = 0;

struct spi_xcomm {
	struct spi_bitbang bitbang; /* keep this one here !!! */
	struct i2c_client *i2c;
	uint16_t settings;
	uint16_t chipselect;

	unsigned int clock;
	unsigned int current_speed;
};

static int spi_xcomm_sync_config(struct spi_xcomm *spi_xcomm)
{
	uint8_t buf[5];

	buf[0] = SPI_XCOMM_CMD_UPDATE_CONFIG;
	buf[1] = spi_xcomm->settings >> 8;
	buf[2] = spi_xcomm->settings & 0xff;
	buf[3] = spi_xcomm->chipselect >> 8;
	buf[4] = spi_xcomm->chipselect & 0xff;

	return i2c_master_send(spi_xcomm->i2c, buf, 5);
}



static void spi_xcomm_chipselect(struct spi_device *spi, int is_active)
{
	struct spi_xcomm *spi_xcomm = spi_master_get_devdata(spi->master);
	unsigned long cs = spi->chip_select;
	uint16_t chipselect = spi_xcomm->chipselect;
	uint16_t settings = spi_xcomm->settings;
//	static int cs = 0;

	if (is_active) {
		settings &= ~BIT(5);
		chipselect |= BIT(cs);
	} else {
		settings |= BIT(5);
		chipselect &= ~BIT(cs);
	}

	if (chipselect != spi_xcomm->chipselect || settings != spi_xcomm->settings) {
		spi_xcomm->chipselect = chipselect;
		spi_xcomm->settings = settings;
		spi_xcomm_sync_config(spi_xcomm);
	}

// 	if (!is_active) {
// 		printk("cs: %d\n", cs);
// 		cs = (cs + 1) % 16;
// 	}
}
static int spi_xcomm_setup_transfer(struct spi_device *spi,
				   struct spi_transfer *t)
{
	struct spi_xcomm *spi_xcomm = spi_master_get_devdata(spi->master);
	unsigned int settings = spi_xcomm->settings;
	unsigned int speed;

	if ((t->bits_per_word && t->bits_per_word != 8) || t->len > 62)
		return -EINVAL;

	speed = t->speed_hz ? t->speed_hz : spi->max_speed_hz;

	if (speed != spi_xcomm->current_speed) {
		unsigned int divider = spi_xcomm->clock / speed;
		if (divider >= 64 || 1)
			settings |= SPI_XCOMM_SETTINGS_CLOCK_DIV_64;
		else if (divider >= 16)
			settings |= SPI_XCOMM_SETTINGS_CLOCK_DIV_16;
		else
			settings |= SPI_XCOMM_SETTINGS_CLOCK_DIV_4;

		spi_xcomm->current_speed = speed;
	}

	if (spi->mode & SPI_CPOL)
		settings |= SPI_XCOMM_SETTINGS_CPOL;
	else
		settings &= ~SPI_XCOMM_SETTINGS_CPOL;

	if (!(spi->mode & SPI_CPHA))
		settings |= SPI_XCOMM_SETTINGS_CPHA;
	else
		settings &= ~SPI_XCOMM_SETTINGS_CPHA;

	if (spi->mode & SPI_3WIRE)
		settings |= SPI_XCOMM_SETTINGS_3WIRE;
	else
		settings &= ~SPI_XCOMM_SETTINGS_3WIRE;

	if (settings != spi_xcomm->settings) {
		spi_xcomm->settings = settings;
		spi_xcomm_sync_config(spi_xcomm);
	}

	return 0;
}

static int spi_xcomm_txrx_bufs(struct spi_device *spi, struct spi_transfer *t, bool is_last)
{
	struct spi_xcomm *spi_xcomm = spi_master_get_devdata(spi->master);
	int ret;
	unsigned int settings = spi_xcomm->settings;

	if (is_last)
		settings |= BIT(5);
	else
		settings &= ~BIT(5);

	if (t->tx_buf) {
		uint8_t buf[63];
		if (settings != spi_xcomm->settings) {
			spi_xcomm->settings = settings;
			spi_xcomm_sync_config(spi_xcomm);
		}
		buf[0] = SPI_XCOMM_CMD_WRITE;
		memcpy(buf + 1, t->tx_buf, t->len);

		ret = i2c_master_send(spi_xcomm->i2c, buf, t->len + 1);
		if (ret < 0)
			return ret;
		else if (ret != t->len + 1)
			return -EIO;
	} else if (t->rx_buf) {
		uint8_t rx_buf[t->len + 1];
		spi_xcomm->settings = settings & ~(0x3f << 10);
		spi_xcomm->settings |= /*(t->len == 1 ? 2 :*/ t->len/*)*/ << 10;
		spi_xcomm_sync_config(spi_xcomm);
		spi_xcomm->settings &= ~(0x3f << 10);

		ret = i2c_master_recv(spi_xcomm->i2c, rx_buf, /*t->len == 1 ? 2 : */t->len);
		if (ret < 0)
			return ret;
		else if (ret != /*(t->len == 1 ? 2 : */t->len/*)*/)
			return -EIO;
		memcpy(t->rx_buf, rx_buf, t->len);
	}
//	msleep(3);
	return t->len;
}

static int spi_xcomm_setup(struct spi_device *spi)
{
	if (spi->bits_per_word != 8)
		return -EINVAL;

	return 0;
}

static int __devinit spi_xcomm_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	uint8_t buf[] = {0x2};
	int ret;
	struct spi_master *master;
	struct spi_xcomm *spi_xcomm;

	master = spi_alloc_master(&i2c->dev, sizeof(*spi_xcomm));
	if (!master)
		return -ENOMEM;

	spi_xcomm = spi_master_get_devdata(master);
	i2c_set_clientdata(i2c, spi_xcomm);
	spi_xcomm->i2c = i2c;
	master->dev.of_node = i2c->dev.of_node;

	master->num_chipselect = 16;
	master->setup = spi_xcomm_setup;
	master->cleanup = spi_bitbang_cleanup;

	spi_xcomm->bitbang.master = spi_master_get(master);
	spi_xcomm->bitbang.master->bus_num = bus_num++;
	spi_xcomm->bitbang.chipselect = spi_xcomm_chipselect;

	spi_xcomm->bitbang.setup_transfer = spi_xcomm_setup_transfer;
	spi_xcomm->bitbang.txrx_bufs = spi_xcomm_txrx_bufs;
	master->flags = SPI_MASTER_HALF_DUPLEX;

	master->mode_bits = SPI_CPHA | SPI_CPOL | SPI_3WIRE;

	spi_xcomm->clock = 48000000;

	ret = spi_bitbang_start(&spi_xcomm->bitbang);
	if (ret < 0)
		spi_master_put(master);

	i2c_master_send(i2c, buf, 1);

	return ret;
}

static int __devexit spi_xcomm_remove(struct i2c_client *i2c)
{
	struct spi_xcomm *spi_xcomm = i2c_get_clientdata(i2c);
	int ret;

	ret = spi_bitbang_stop(&spi_xcomm->bitbang);
	spi_master_put(spi_xcomm->bitbang.master);

	return ret;
}

static const struct i2c_device_id spi_xcomm_ids[] = {
	{ "spi-xcomm", 0 },
	{ },
};

static struct i2c_driver spi_xcomm_driver = {
	.driver = {
		.name	= "spi-xcomm",
		.owner	= THIS_MODULE,
	},
	.id_table	= spi_xcomm_ids,
	.probe		= spi_xcomm_probe,
	.remove		= __devexit_p(spi_xcomm_remove),
};
/*module_i2c_driver(spi_xcomm_driver);*/

static int __init spi_xcomm_init(void)
{
	return i2c_add_driver(&spi_xcomm_driver);
}
module_init(spi_xcomm_init);

static void __exit spi_xcomm_exit(void)
{
	i2c_del_driver(&spi_xcomm_driver);
}
module_exit(spi_xcomm_exit);

MODULE_LICENSE("GPL");
