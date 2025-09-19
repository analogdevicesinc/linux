// SPDX-License-Identifier: GPL-2.0
/*
 *
 *
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include <linux/spi/spi.h>

#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/bitfield.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/types.h>
#include <linux/bitops.h>
#include <linux/unaligned.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

//adg
#include <linux/of.h>
#include <linux/of_device.h>

/*ADMT4000 Registers*/
#define ADMT4000_REG_RST                0x00

/*ADMT4000 Agnostic Page Registers*/
#define ADMT4000_AGP_REG_CNVPAGE        0x01
#define ADMT4000_AGP_REG_ABSANGLE       0x03
#define ADMT4000_AGP_REG_DIGIO          0x04
#define ADMT4000_AGP_REG_ANGLE          0x05
#define ADMT4000_AGP_REG_FAULT          0x06
#define ADMT4000_AGP_REG_ANGLESEC       0x08

/*ADMT 0x00 Page Registers*/
#define ADMT4000_RAW_ANGLE_REG(x)       (0x10 + (x))
#define ADMT4000_00_REG_SINE		0x10
#define ADMT4000_00_REG_COSINE		0x11
#define ADMT4000_00_REG_RADIUS          0x18
#define ADMT4000_00_REG_TMP0            0x20
#define ADMT4000_00_REG_TMP1            0x23

/*ADMT 0x02 Page Registers*/
#define ADMT4000_02_REG_GENERAL         0x10 // ECC0
#define ADMT4000_02_REG_DIGIOEN         0x12 // ECC0
#define ADMT4000_02_REG_ANGLECK         0x13 // ECC0
#define ADMT4000_02_REG_CNVCNT          0x14
#define ADMT4000_02_REG_H1MAG           0x15 // ECC0
#define ADMT4000_02_REG_H1PH            0x16 // ECC0
#define ADMT4000_02_REG_H2MAG           0x17 // ECC0
#define ADMT4000_02_REG_H2PH            0x18 // ECC0
#define ADMT4000_02_REG_H3MAG           0x19 // ECC1/0
#define ADMT4000_02_REG_H3PH            0x1A // ECC1
#define ADMT4000_02_REG_H8MAG           0x1B // ECC1
#define ADMT4000_02_REG_H8PH            0x1C // ECC1
#define ADMT4000_02_REG_ECCEDC          0x1D
#define ADMT4000_02_REG_UNIQD0          0x1E
#define ADMT4000_02_REG_UNIQD1          0x1F
#define ADMT4000_02_REG_UNIQD2          0x20
#define ADMT4000_02_REG_UNIQD3          0x21
#define ADMT4000_02_REG_ECCDIS          0x23

/* ADMT4000 Other Macros, Masks etc. */
#define ADMT4000_FRAME_SIZE		4
#define ADMT4000_RW_MASK                GENMASK(5, 0)
#define ADMT4000_WR_EN                  BIT(6)
#define ADMT4000_FAULT_MASK             BIT(7)
#define ADMT4000_LIFE_CTR               GENMASK(6, 5)
#define ADMT4000_RCV_CRC                GENMASK(4, 0)

/* Upper and Lower Byte Masking */
#define ADMT4000_LOW_BYTE               GENMASK(7, 0)
#define ADMT4000_HI_BYTE                GENMASK(15, 8)

/* Register 01 */
#define ADMT4000_CNV_EDGE_MASK          GENMASK(15, 14)
#define ADMT4000_PAGE_MASK              GENMASK(4, 0)
#define ADMT4000_FALLING_EDGE           0x00
#define ADMT4000_RISING_EDGE            0x3

/* Register 03 */
#define ADMT4000_ABS_ANGLE_MASK         GENMASK(15, 0)
#define ADMT4000_TURN_CNT_MASK          GENMASK(15, 10)
#define ADMT4000_ABS_ANGLE_ANGLE_MASK   GENMASK(9, 0)

//#define ADMT4000_INVALID_TURN           0x36
#define ADMT4000_INVALID_TURN           0x53

/* Register 04*/
#define ADMT4000_MAX_GPIO_INDEX         5
#define ADMT4000_GPIO_LOGIC(x)          BIT(x)

/* Register 05 and 08 */
#define ADMT4000_ANGLE_MASK             GENMASK(15, 4)

/* Register 06 */
#define ADMT4000_ALL_FAULTS             GENMASK(15, 0)
#define ADMT4000_AGP_INDIV_FAULT(x)     BIT((x))

/* Register 0x08/10/11/12/13/18, page 0 */
#define ADMT4000_ANGLE_STAT_MASK        BIT(0)

/* Register 0x10/11/12/13, page 0 */
#define ADMT4000_RAW_ANGLE_MASK         GENMASK(15, 2)
#define ADMT4000_RAW_COSINE_MASK        GENMASK(15, 2)
#define ADMT4000_RAW_SINE_MASK		GENMASK(15, 2)

/* Register 18, page 0 */
#define ADMT4000_RADIUS_MASK            GENMASK(15, 1)

/* Register 0x1D, page 0 */
#define ADMT4000_MTDIAG1_MASK           GENMASK(15, 8)
#define ADMT4000_AFEDIAG2_MASK          GENMASK(7, 0)

/* Register 0x1E, page 0 */
#define ADMT4000_AFEDIAG1_MASK          GENMASK(15, 8)
#define ADMT4000_REF_RES(x)             BIT(8 + (x))
#define ADMT4000_AFEDIAG0_MASK          GENMASK(7, 0)

/* Register 0x20, page 0 */
#define ADMT4000_TEMP_MASK              GENMASK(15, 4)

/* Register 0x10, page 2 */
#define ADMT4000_STORAGE_MSB_XTRACT     BIT(7)
#define ADMT4000_STORAGE_BIT6_XTRACT    BIT(6)
#define ADMT4000_STORAGE_MASK1_XTRACT   GENMASK(5, 3)
#define ADMT4000_STORAGE_MASK0_XTRACT   GENMASK(2, 0)
#define ADMT4000_STORAGE_MSB            BIT(15)
#define ADMT4000_STORAGE_BIT6           BIT(11)
#define ADMT4000_STORAGE_MASK1          GENMASK(8, 6)
#define ADMT4000_STORAGE_MASK0          GENMASK(3, 1)

#define ADMT4000_CONV_SYNC_MODE_MASK    GENMASK(14, 13)
#define ADMT4000_ANGL_FILT_MASK         BIT(12)
#define ADMT4000_H8_CTRL_MASK           BIT(10)
#define ADMT4000_CNV_MODE_MASK          BIT(0)

/* Register 0x12, page 2 */
#define ADMT4000_DIG_IO_EN(x)           BIT(8 + (x))
#define ADMT4000_GPIO_FUNC(x)           BIT(x)

/* Register 0x13, page 2 */
#define ADMT4000_ANGL_CHK_MASK          GENMASK(9, 0)

/* Register 0x13, page 2 */
#define ADMT4000_CNV_CTR_MASK           GENMASK(7, 0)

/* Register 0x15 / 0x18, page 2 */
#define ADMT4000_H_11BIT_MAG_MASK       GENMASK(10, 0)
#define ADMT4000_11BIT_MAX              2047
#define ADMT4000_H_12BIT_PHA_MASK       GENMASK(11, 0)
#define ADMT4000_12BIT_MAX              4095

/* Register 0x1A / 0x1B, page 2 */
#define ADMT4000_H_8BIT_MAG_MASK        GENMASK(7, 0)
#define ADMT4000_8BIT_MAX               127

/* Register 0x1D, page 2 */
#define ADMT4000_ECC_CFG1               GENMASK(15, 8)
#define ADMT4000_ECC_CFG0               GENMASK(7, 0)

/* Register 0x1E 0x1F 0x20 0x21 , page 2 */
#define ADMT4000_ID0_MASK               GENMASK(15, 0)
#define ADMT4000_ID_PROD_MASK           GENMASK(10, 8)
#define ADMT4000_ID_SUPPLY_MASK         GENMASK(7, 6)
#define ADMT4000_ID_ASIL_MASK           GENMASK(5, 3)
#define ADMT4000_ID_SIL_REV_MAS         GENMASK(2, 0)

/* Register 0x23, page 2 */
#define ADMT4000_ECC_EN_COMM            0x0000
#define ADMT4000_ECC_DIS_COMM           0x4D54

/* Turn Count Conversion */
#define ADMT4000_TURN_CNT_THRES         0x35

//
#define ADMT_CONV_SYNC_MODE_MAX		4

//
#define ADMT4000_3P3V			3300000
#define ADMT4000_5V			5000000

struct admt4000_chip_info {
	const char			*name;
	const struct iio_chan_spec	*channels;
	unsigned int num_channels;
};

struct admt4000_state {
	struct spi_device		*spi;
	struct regmap			*regmap;
	/* lock to protect against multiple access to the device and shared data */
	struct mutex			lock;
	const struct admt4000_chip_info *chip_info;
	struct gpio_desc *busy_gpio;
	struct gpio_desc *cnv_gpio;
	struct gpio_desc *acalc_gpio;
	struct completion completion;
	struct iio_trigger *trig;
	int ext_vdd;
	/* Boolean indicator if setup in page 0 or not */
	bool is_page_zero;
	/* Tracker for conversion type: one shot or continuous */
	bool is_one_shot;
	int acalc_irq;
	u8 buf[3];
};

enum admt4000_iio_chan_type {
	ADMT4000_TEMP,
	ADMT4000_ANGLE,
	ADMT4000_TURN_COUNT,
	ADMT4000_COSINE,
	ADMT4000_SINE,
};

enum admt4000_conv_sync_mode {
	ADMT4000_SYNC_SEQ_CTRL = 0,
	ADMT4000_SYNC_START_EDGE = 3,
};

static u8 admt4000_ecc_control_registers[] = {
	ADMT4000_02_REG_GENERAL,
	ADMT4000_02_REG_DIGIOEN,
	ADMT4000_02_REG_ANGLECK,
	ADMT4000_02_REG_H1MAG,
	ADMT4000_02_REG_H1PH,
	ADMT4000_02_REG_H2MAG,
	ADMT4000_02_REG_H2PH,
	ADMT4000_02_REG_H3MAG,
	ADMT4000_02_REG_H3PH,
	ADMT4000_02_REG_H8MAG,
	ADMT4000_02_REG_H8PH,
};

static const char * const admt4000_sync_mode_str[] = {
	"seq_ctrl", "invalid0", "invalid1", "start_edge"};

static const char * const admt4000_h8_ctrl_str[] = {
	"factory", "user"};

static int admt4000_compute_crc(u8 reg_addr, u16 reg_data, u8 excess,
				bool is_write, u8 *crc_ret)
{
	int crc[] = {1, 1, 1, 1, 1};
	int poly, i, xor;
	u32 data_in;

	if (reg_addr > ADMT4000_02_REG_ECCDIS)
		return -EINVAL;

	if (is_write) {
		reg_addr = (reg_addr & ADMT4000_RW_MASK) | ADMT4000_WR_EN;
		data_in = ((reg_addr << 16) | reg_data) << 3;
	} else {
		reg_addr = (reg_addr & ADMT4000_RW_MASK);
		data_in = ((reg_addr << 16) | reg_data) << 3 | excess;
	}

	for (i = 25; i >= 0; i--) {
		xor = ((data_in >> i) & 0x1) ^ crc[4];
		poly = crc[1] ^ xor;

		crc[4] = crc[3];
		crc[3] = crc[2];
		crc[2] = poly;
		crc[1] = crc[0];
		crc[0] = xor;
	}

	*crc_ret = 16 * crc[4] + 8 * crc[3] + 4 * crc[2] + 2 * crc[1] + crc[0];
	return 0;
};

static int admt4000_reg_read(void *context, unsigned int reg_addr, unsigned int *reg_data)
{
	struct admt4000_state *st = context;
	int ret;
	u8 temp, crc;
	u8 buf[1];
	u8 rbuf[3] = {0};

	if (reg_addr > ADMT4000_02_REG_ECCDIS)
		return -EINVAL;

	buf[0] = reg_addr & ADMT4000_RW_MASK;

	ret = spi_write_then_read(st->spi, &buf, 1, &rbuf, 3);

	*reg_data = get_unaligned_be16(rbuf);
	temp = FIELD_GET((ADMT4000_LIFE_CTR | ADMT4000_FAULT_MASK), rbuf[2]);

	ret = admt4000_compute_crc(reg_addr, *reg_data, temp, false, &crc);
	if (ret)
		return ret;

	if ((rbuf[2] & ADMT4000_RCV_CRC) != crc)
		return -EIO;
	return 0;
}

static int admt4000_reg_write(void *context, unsigned int reg_addr, unsigned int reg_data)
{
	struct admt4000_state *st = context;
	u8 buf[4] = {0};
	int ret;
	u8 crc;

	if (reg_addr > ADMT4000_02_REG_ECCDIS)
		return -EINVAL;

	ret = admt4000_compute_crc(reg_addr, reg_data, 0, true, &crc);
	if (ret)
		return ret;

	buf[0] = (reg_addr & ADMT4000_RW_MASK) | ADMT4000_WR_EN;
	put_unaligned_be16(reg_data, &buf[1]);
	buf[3] = crc;

	return spi_write_then_read(st->spi, buf, ADMT4000_FRAME_SIZE, NULL, 0);
}

static const struct regmap_config admt4000_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 24,
	.reg_read = admt4000_reg_read,
	.reg_write = admt4000_reg_write,
};

static int admt4000_reg_update(struct admt4000_state *st, u8 reg_addr,
			u16 update_mask, int update_val)
{
	int ret;
	int temp;

	if (reg_addr > ADMT4000_02_REG_ECCDIS)
		return -EINVAL;

	ret = regmap_read(st->regmap, reg_addr, &temp);
	if (ret)
		return ret;

	temp &= ~update_mask;
	temp |= update_val;

	return regmap_write(st->regmap, reg_addr, temp);
}

static int admt4000_hamming_calc(u8 position, u8 code_length, u8 *code)
{
	u8 bit_extract;
	int count = 0, i, j;

	i = position - 1;
	if (code_length > 120)
		return -EINVAL;

	while (i < code_length) {
		for (j = i; j < i + position; j++) {
			bit_extract = (code[j >> 3] >> (j & 0x7)) & 0x1;
			if (bit_extract)
				count++;
		}

		i += (2 * position);
	}

	return !(count & 1);
}

static int admt4000_ecc_encode(u8 *parity_num, u8 *code_length,
			u8 *code, u8 *input, u8 size_code, u8 size_input,
			u8 *ecc)
{
	int i = 0, j = 0, k = 0;
	int value, eff_pos;
	u8 xtract, bit, position;
	*ecc = 0;
	if (size_code > 16 || size_input > 16)
		return -EINVAL;
	//Compute parity bits
	*parity_num = 0;
	while ((size_input * 8) > (1 << i) - (i + 1)) {
		*parity_num += 1;
		i++;
	}
	*code_length = *parity_num + (size_input * 8);

	for (i = 0; i < *code_length; i++) {
		/* Set to 0 */
		if (i == ((1 << k) - 1)) {
			code[(i >> 3)] &= ~BIT((i & 0x7)); //i & 0x7 is equivalent to i % 8

			k++;
		} else {
			code[(i >> 3)] &= ~BIT((i & 0x7));

			xtract = (input[j >> 3] >> (j & 0x7)) & 0x1;
			xtract = xtract << (i & 0x7);

			code[(i >> 3)] |= xtract;

			j++;
		}
	}
    //Assignment of value bits inside the code array
	/* parity_num is in BITS */
	for (i = 0; i < *parity_num; i++) {
		position = 1 << i;

		value = admt4000_hamming_calc(position, *code_length, code);

		eff_pos = position - 1;

		code[(eff_pos >> 3)] &= ~BIT(eff_pos & 0x7); //i & 0x7 is equivalent to i % 8

	if (value)
		code[(eff_pos >> 3)] |= BIT(eff_pos & 0x7);

	*ecc |= (value << i);
	}

	value = 0;
	for (i = 0; i < *code_length; i++) {
		bit = (input[j >> 3] >> (j & 0x7)) & 0x1;
		if (bit)
			value++;
	}

	value &= 0x1;

	*ecc |= (value << *parity_num);

	return 0;
}

static int admt4000_clear_all_faults(struct admt4000_state *st)
{
	return admt4000_reg_update(st, ADMT4000_AGP_REG_FAULT, ADMT4000_ALL_FAULTS,
				   0);
}

static int admt4000_set_page(struct admt4000_state *st, bool is_page_zero)
{
	int ret;

	if (is_page_zero)
		ret = admt4000_reg_update(st, ADMT4000_AGP_REG_CNVPAGE, ADMT4000_PAGE_MASK,
					  FIELD_PREP(ADMT4000_PAGE_MASK, 0x00));
	else
		ret = admt4000_reg_update(st, ADMT4000_AGP_REG_CNVPAGE, ADMT4000_PAGE_MASK,
					  FIELD_PREP(ADMT4000_PAGE_MASK, 0x02));
	if (ret)
		return ret;

	st->is_page_zero = is_page_zero;

	return 0;
}

static int admt4000_get_page(struct admt4000_state *st, bool *is_page_zero)
{
	int ret, temp;

	ret = regmap_read(st->regmap, ADMT4000_AGP_REG_CNVPAGE, &temp);
	if (ret)
		return ret;

	if (!FIELD_GET(ADMT4000_PAGE_MASK, temp))
		*is_page_zero = true;
	else if (FIELD_GET(ADMT4000_PAGE_MASK, temp) == 0x2)
		*is_page_zero = false;
	else
		return -EINVAL;

	return 0;
}

static int admt4000_ecc_config(struct admt4000_state *st, bool is_en)
{
	int ret;

	if (st->is_page_zero) {
		ret = admt4000_set_page(st, false);
		if (ret)
			return ret;
	}

	if (is_en)
		return regmap_write(st->regmap, ADMT4000_02_REG_ECCDIS, ADMT4000_ECC_EN_COMM);

	return regmap_write(st->regmap, ADMT4000_02_REG_ECCDIS, ADMT4000_ECC_DIS_COMM);
}

static int admt4000_update_ecc(struct admt4000_state *st, u16 *ecc_val)
{
	/*
	 * ECC related variables:
	 * for_encode[15] - contains the 15 byte register data to be encoded
	 * ecc[2] - stores the generated ECC for CONFIG0 and CONFIG1
	 * parity_num - stores the number of parity bits generated
	 * code_len - stores the length of the encoded data (in bits)
	 * encoded[16] - stores the encoded data (original register data + parity bits inserted)
	 *
	 * Others:
	 * ret - error code of function/s ran
	 * temp - stores 16 bit data read from each registers
	 */
	int ret;
	u8 for_encode[15] = {0}, ecc[2] = {0};
	u8 parity_num, code_len, encoded[16] = {0};
	int temp;

	/*
	 * ECC1 (needs padding)
	 */
	for (int i = 7; i < 11; i++) {
		ret = regmap_read(st->regmap, admt4000_ecc_control_registers[i], &temp);
		if (ret)
			return ret;

		if (i != 7)
			put_unaligned_le16(temp, for_encode + (2 * (i - 8) + 1));
		else
			for_encode[14] = FIELD_GET(ADMT4000_HI_BYTE, temp);
	}

	ret = admt4000_ecc_encode(&parity_num, &code_len,
				  encoded, for_encode, 16, 15, ecc);
	if (ret)
		return ret;

	/*
	 * ECC0 (no padding)
	 */
	for (int i = 0; i < 8; i++) {
		ret = regmap_read(st->regmap, admt4000_ecc_control_registers[i], &temp);
		if (ret)
			return ret;

		if (i != 7)
			put_unaligned_le16(temp, for_encode + (2 * i));
		else
			for_encode[14] = FIELD_GET(ADMT4000_LOW_BYTE, temp);
	}

	/*
	 * ECC1 (needs padding)
	 */
	ret = admt4000_ecc_encode(&parity_num, &code_len,
				  encoded, for_encode, 16, 15, ecc + 1);
	if (ret)
		return ret;

	/* Format ECC data to write in actual register */
	temp = get_unaligned_be16(ecc);

	/* Store in return variable */
	if (temp)
		*ecc_val = temp;
	else
		return -EINVAL;

	/* All registers of interest are in page 2 */
	ret = admt4000_set_page(st, false);
	if (ret)
		return ret;

	/* Disable ECC functionality */
	ret = admt4000_ecc_config(st, false);
	if (ret)
		return ret;

	/* Update ECC register contents */
	ret = regmap_write(st->regmap, ADMT4000_02_REG_ECCEDC, temp);
	if (ret)
		return ret;

	/* Enable ECC functionality */
	return admt4000_ecc_config(st, true);
}

static int admt4000_io_en(struct admt4000_state *st, u8 gpio, bool is_en)
{
	int ret;
	u16 ecc;

	if (gpio > ADMT4000_MAX_GPIO_INDEX)
		return -EINVAL;

	if (st->is_page_zero) {
		ret = admt4000_set_page(st, false);
		if (ret)
			return ret;
	}

	/* Disable ECC functionality */
	ret = admt4000_ecc_config(st, false);
	if (ret)
		return ret;

	if (is_en)
		ret = admt4000_reg_update(st, ADMT4000_02_REG_DIGIOEN,
					  ADMT4000_DIG_IO_EN(gpio), ADMT4000_DIG_IO_EN(gpio));
	else
		ret = admt4000_reg_update(st, ADMT4000_02_REG_DIGIOEN,
					  ADMT4000_DIG_IO_EN(gpio), 0);
	if (ret)
		return ret;

	/* Update ECC register & re-enable ECC functionality */
	return admt4000_update_ecc(st, &ecc);
}

static int admt4000_gpio_func(struct admt4000_state *st, u8 gpio,
		       bool is_alt_func)
{
	int ret;
	u16 ecc;

	if (gpio > ADMT4000_MAX_GPIO_INDEX)
		return -EINVAL;

	if (st->is_page_zero) {
		ret = admt4000_set_page(st, false);
		if (ret)
			return ret;
	}

	/* Disable ECC functionality */
	ret = admt4000_ecc_config(st, false);
	if (ret)
		return ret;

	if (is_alt_func)
		ret =  admt4000_reg_update(st, ADMT4000_02_REG_DIGIOEN,
					   ADMT4000_GPIO_FUNC(gpio), 0);
	else
		ret =  admt4000_reg_update(st, ADMT4000_02_REG_DIGIOEN,
					   ADMT4000_GPIO_FUNC(gpio), ADMT4000_GPIO_FUNC(gpio));
	if (ret)
		return ret;

	/* Update ECC register & re-enable ECC functionality */
	return admt4000_update_ecc(st, &ecc);
}

static int admt4000_set_conv_sync_mode(struct admt4000_state *st,
				enum admt4000_conv_sync_mode mode)
{
	int ret;
	u16 ecc;

	if (st->is_page_zero) {
		ret = admt4000_set_page(st, false);
		if (ret)
			return ret;
	}

	/* Disable ECC functionality */
	ret = admt4000_ecc_config(st, false);
	if (ret)
		return ret;

	ret = admt4000_reg_update(st, ADMT4000_02_REG_GENERAL,
				  ADMT4000_CONV_SYNC_MODE_MASK,
				  FIELD_PREP(ADMT4000_CONV_SYNC_MODE_MASK, mode));
	if (ret)
		return ret;

	/* Update ECC register & re-enable ECC functionality */
	return admt4000_update_ecc(st, &ecc);
}

static int admt4000_set_angle_filt(struct admt4000_state *st, bool is_filtered)
{
	int ret;
	u16 ecc;
	int temp;

	if (st->is_page_zero) {
		ret = admt4000_set_page(st, false);
		if (ret)
			return ret;
	}

	/* Disable ECC functionality */
	ret = admt4000_ecc_config(st, false);
	if (ret)
		return ret;

	temp = FIELD_PREP(ADMT4000_ANGL_FILT_MASK, (uint8_t)is_filtered);
	ret =  admt4000_reg_update(st, ADMT4000_02_REG_GENERAL,
				   ADMT4000_ANGL_FILT_MASK, temp);

	if (ret)
		return ret;

	/* Update ECC register & re-enable ECC functionality */
	return admt4000_update_ecc(st, &ecc);
}

static int admt4000_get_angle_filt(struct admt4000_state *st, bool *is_filtered)
{
	int ret;
	int temp;

	if (st->is_page_zero) {
		ret = admt4000_set_page(st, false);
		if (ret)
			return ret;
	}

	ret = regmap_read(st->regmap, ADMT4000_02_REG_GENERAL, &temp);
	if (ret)
		return ret;

	*is_filtered = (bool)FIELD_GET(ADMT4000_ANGL_FILT_MASK, temp);

	return 0;
}

static int admt4000_set_conv_mode(struct admt4000_state *st, bool is_one_shot)
{
	int ret;
	u16 ecc;
	int temp;

	if (st->is_page_zero) {
		ret = admt4000_set_page(st, false);
		if (ret)
			return ret;
	}

	/* Disable ECC functionality */
	ret = admt4000_ecc_config(st, false);
	if (ret)
		return ret;

	temp = FIELD_PREP(ADMT4000_CNV_MODE_MASK, (uint8_t)is_one_shot);
	ret = admt4000_reg_update(st, ADMT4000_02_REG_GENERAL,
				  ADMT4000_CNV_MODE_MASK, temp);
	if (ret)
		return ret;

	st->is_one_shot = is_one_shot;

	return admt4000_update_ecc(st, &ecc);
}

static int admt4000_set_h8_ctrl(struct admt4000_state *st,
			 u8 source)
{
	int ret;
	int temp;
	u16 ecc;

	if (st->is_page_zero) {
		ret = admt4000_set_page(st, false);
		if (ret)
			return ret;
	}

	/* Disable ECC functionality */
	ret = admt4000_ecc_config(st, false);
	if (ret)
		return ret;

	temp = FIELD_PREP(ADMT4000_H8_CTRL_MASK, source);
	ret = admt4000_reg_update(st, ADMT4000_02_REG_GENERAL,
				  ADMT4000_H8_CTRL_MASK, temp);

	if (ret)
		return ret;

	/* Update ECC register & re-enable ECC functionality */
	return admt4000_update_ecc(st, &ecc);
}

static int admt4000_get_h8_ctrl(struct admt4000_state *st,
			int *source)
{
	int ret;
	int temp;

	if (st->is_page_zero) {
		ret = admt4000_set_page(st, false);
		if (ret)
			return ret;
	}

	ret = regmap_read(st->regmap, ADMT4000_02_REG_GENERAL, &temp);
	if (ret)
		return ret;

	*source = FIELD_GET(ADMT4000_H8_CTRL_MASK, temp);

	return 0;
}

static int admt4000_get_h8_corr_src(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct admt4000_state *st = iio_priv(indio_dev);
	int ret, val;

	ret = admt4000_get_h8_ctrl(st, &val);
	if (ret)
		return ret;

	return val;
}

static int admt4000_set_h8_corr_src(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int val)
{
	struct admt4000_state *st = iio_priv(indio_dev);

	if (val >= ARRAY_SIZE(admt4000_h8_ctrl_str))
		return -EINVAL;

	return admt4000_set_h8_ctrl(st, val);
}

static int admt4000_set_hmag_config(struct admt4000_state *st, u8 hmag,
			     u16 mag)
{
	int ret;
	u16 ecc;

	if (st->is_page_zero) {
		ret = admt4000_set_page(st, false);
		if (ret)
			return ret;
	}

	/* Disable ECC functionality */
	ret = admt4000_ecc_config(st, false);
	if (ret)
		return ret;

	switch (hmag) {
	case 1:
		if (mag > ADMT4000_11BIT_MAX)
			return -EINVAL;
		return admt4000_reg_update(st, ADMT4000_02_REG_H1MAG,
					   ADMT4000_H_11BIT_MAG_MASK, FIELD_PREP(ADMT4000_H_11BIT_MAG_MASK, mag));
	case 2:
		if (mag > ADMT4000_11BIT_MAX)
			return -EINVAL;
		return admt4000_reg_update(st, ADMT4000_02_REG_H2MAG,
					   ADMT4000_H_11BIT_MAG_MASK, FIELD_PREP(ADMT4000_H_11BIT_MAG_MASK, mag));
	case 3:
		if (mag > ADMT4000_8BIT_MAX)
			return -EINVAL;
		return admt4000_reg_update(st, ADMT4000_02_REG_H3MAG,
					   ADMT4000_H_8BIT_MAG_MASK, FIELD_PREP(ADMT4000_H_8BIT_MAG_MASK, mag));
	case 8:
		if (mag > ADMT4000_8BIT_MAX)
			return -EINVAL;
		return admt4000_reg_update(st, ADMT4000_02_REG_H8MAG,
					   ADMT4000_H_8BIT_MAG_MASK, FIELD_PREP(ADMT4000_H_8BIT_MAG_MASK, mag));
	default:
		return -EINVAL;
	}

	/* Update ECC register & re-enable ECC functionality */
	return admt4000_update_ecc(st, &ecc);
}

static int admt4000_get_hmag_config(struct admt4000_state *st, u8 hmag,
			     u16 *mag)
{
	int ret;
	int temp;

	if (st->is_page_zero) {
		ret = admt4000_set_page(st, false);
		if (ret)
			return ret;
	}

	switch (hmag) {
	case 1:
		ret = regmap_read(st->regmap, ADMT4000_02_REG_H1MAG, &temp);
		break;
	case 2:
		ret = regmap_read(st->regmap, ADMT4000_02_REG_H2MAG, &temp);
		break;
	case 3:
		ret = regmap_read(st->regmap, ADMT4000_02_REG_H3MAG, &temp);
		break;
	case 8:
		ret = regmap_read(st->regmap, ADMT4000_02_REG_H8MAG, &temp);
		break;
	default:
		return -EINVAL;
	}

	if (ret)
		return ret;

	if (hmag < 3)
		*mag = FIELD_GET(ADMT4000_H_11BIT_MAG_MASK, temp);
	else
		*mag = FIELD_GET(ADMT4000_H_8BIT_MAG_MASK, temp);

	return 0;
}

static int admt4000_set_hphase_config(struct admt4000_state *st, u8 hpha,
			       u16 pha)
{
	int ret;
	u16 ecc;

	if (pha > ADMT4000_12BIT_MAX)
		return -EINVAL;

	if (st->is_page_zero) {
		ret = admt4000_set_page(st, false);
		if (ret)
			return ret;
	}

	/* Disable ECC functionality */
	ret = admt4000_ecc_config(st, false);
	if (ret)
		return ret;

	switch (hpha) {
	case 1:
		return admt4000_reg_update(st, ADMT4000_02_REG_H1PH,
					   ADMT4000_H_12BIT_PHA_MASK, FIELD_PREP(ADMT4000_H_12BIT_PHA_MASK, pha));
	case 2:
		return admt4000_reg_update(st, ADMT4000_02_REG_H2PH,
					   ADMT4000_H_12BIT_PHA_MASK, FIELD_PREP(ADMT4000_H_12BIT_PHA_MASK, pha));
	case 3:
		return admt4000_reg_update(st, ADMT4000_02_REG_H3PH,
					   ADMT4000_H_12BIT_PHA_MASK, FIELD_PREP(ADMT4000_H_12BIT_PHA_MASK, pha));
	case 8:
		return admt4000_reg_update(st, ADMT4000_02_REG_H8PH,
					   ADMT4000_H_12BIT_PHA_MASK, FIELD_PREP(ADMT4000_H_12BIT_PHA_MASK, pha));
	default:
		return -EINVAL;
	}

	/* Update ECC register & re-enable ECC functionality */
	return admt4000_update_ecc(st, &ecc);
}

static int admt4000_get_hphase_config(struct admt4000_state *st, u8 hpha,
			       u16 *pha)
{
	int ret;
	int temp;

	if (st->is_page_zero) {
		ret = admt4000_set_page(st, false);
		if (ret)
			return ret;
	}

	switch (hpha) {
	case 1:
		ret = regmap_read(st->regmap, ADMT4000_02_REG_H1PH, &temp);
		break;
	case 2:
		ret = regmap_read(st->regmap, ADMT4000_02_REG_H2PH, &temp);
		break;
	case 3:
		ret = regmap_read(st->regmap, ADMT4000_02_REG_H3PH, &temp);
		break;
	case 8:
		ret = regmap_read(st->regmap, ADMT4000_02_REG_H8PH, &temp);
		break;
	default:
		return -EINVAL;
	}

	if (ret)
		return ret;

	*pha = FIELD_GET(ADMT4000_H_12BIT_PHA_MASK, temp);

	return 0;
}

static int admt4000_get_temp(struct admt4000_state *st,
		      bool is_primary)
{
	int ret;
	int raw_temp;

	if (!st->is_page_zero) {
		ret = admt4000_set_page(st, true);
		if (ret)
			return ret;
	}

	if (is_primary)
		ret = regmap_read(st->regmap, ADMT4000_00_REG_TMP0, &raw_temp);
	else
		ret = regmap_read(st->regmap, ADMT4000_00_REG_TMP1, &raw_temp);
	if (ret)
		return ret;

	return FIELD_GET(ADMT4000_TEMP_MASK, raw_temp);
}

static int admt4000_get_raw_turns_and_angle(struct admt4000_state *st,
				     int *turns, u16 *angle)
{
	int ret;
	int temp, raw_turns;

	ret = regmap_read(st->regmap, ADMT4000_AGP_REG_ABSANGLE, &temp);
	if (ret)
		return ret;
	raw_turns = FIELD_GET(ADMT4000_TURN_CNT_MASK, temp); // 63
	if (raw_turns > ADMT4000_TURN_CNT_THRES)
		*turns = sign_extend64(raw_turns, 5);
	else
		*turns = raw_turns;
	angle[0] = FIELD_GET(ADMT4000_ABS_ANGLE_ANGLE_MASK, temp);

	ret = regmap_read(st->regmap, ADMT4000_AGP_REG_ANGLE, &temp);
	angle[1] = FIELD_GET(ADMT4000_ANGLE_MASK, temp);

	return 0;
}

static int admt4000_toggle_cnv(struct admt4000_state *st)
{
	int ret;
	u32 temp;

	if (st->cnv_gpio) {
		gpiod_set_value_cansleep(st->cnv_gpio, 1);
		udelay(1);
		gpiod_set_value_cansleep(st->cnv_gpio, 0);
	} else {
		temp = FIELD_PREP(ADMT4000_CNV_EDGE_MASK, ADMT4000_RISING_EDGE);
		ret = admt4000_reg_update(st, ADMT4000_AGP_REG_CNVPAGE,
					  ADMT4000_CNV_EDGE_MASK, temp);

		if (ret)
			return ret;

		temp = FIELD_PREP(ADMT4000_CNV_EDGE_MASK, ADMT4000_FALLING_EDGE);
		ret =  admt4000_reg_update(st, ADMT4000_AGP_REG_CNVPAGE,
					   ADMT4000_CNV_EDGE_MASK, temp);

		if (ret)
			return ret;
	}

	return 0;
}

static irqreturn_t admt4000_interrupt(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct admt4000_state *st = iio_priv(indio_dev);

	if (iio_buffer_enabled(indio_dev))
		iio_trigger_poll(st->trig);
	else
		complete(&st->completion);

	return IRQ_HANDLED;
};

static irqreturn_t admt4000_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct admt4000_state *st = iio_priv(indio_dev);
	int ret;
	int turns;
	u16 angle[2];

	mutex_lock(&st->lock);

	ret = admt4000_get_raw_turns_and_angle(st, &turns, angle);
	if (ret)
		goto out_unlock_notify;

	st->buf[0] = turns;
	st->buf[1] = angle[1];
	ret = admt4000_get_temp(st, true);
	if (ret)
		goto out_unlock_notify;

	st->buf[2] = ret;

	iio_push_to_buffers_with_timestamp(indio_dev, &st->buf, pf->timestamp);

out_unlock_notify:
	mutex_unlock(&st->lock);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int admt4000_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct admt4000_state *st = iio_priv(indio_dev);
	int ret;
	int turns;
	u16 angle[2];

	mutex_lock(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->address) {
		case ADMT4000_TEMP:
			*val = admt4000_get_temp(st, true);
			ret = IIO_VAL_INT;
			break;
		case ADMT4000_ANGLE:
		case ADMT4000_TURN_COUNT:
			if (st->is_one_shot) {
				ret = admt4000_toggle_cnv(st);
				if (ret)
					return ret;
			}
			ret = admt4000_get_raw_turns_and_angle(st, &turns, angle);
			if (chan->address == ADMT4000_ANGLE)
				*val = angle[1];
			else
				*val = turns;
			ret = IIO_VAL_INT;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->address) {
		case ADMT4000_TEMP:
			*val = 100000;
			*val2 = 1632;
			if (st->ext_vdd == ADMT4000_3P3V)
				*val2 = 1632;
			else if (st->ext_vdd == ADMT4000_5V)
				*val2 = 1627;
			ret = IIO_VAL_FRACTIONAL;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->address) {
		case ADMT4000_TEMP:
			if (st->ext_vdd == ADMT4000_3P3V)
				*val = -1150;
			else if (st->ext_vdd == ADMT4000_5V)
				*val = -1145;
			ret = IIO_VAL_INT;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_PROCESSED:
		ret = admt4000_get_raw_turns_and_angle(st, &turns, angle);
		*val = 360000 * turns + 351 * angle[0];
		*val2 = 360000;
		ret = IIO_VAL_FRACTIONAL;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);
	return ret;
}

static int admt4000_debug_reg(struct iio_dev *indio_dev,
			    unsigned int reg, unsigned int writeval, unsigned int *readval)
{
	struct admt4000_state *st = iio_priv(indio_dev);

	if (!readval)
		return regmap_write(st->regmap, reg, writeval);

	return regmap_read(st->regmap, reg, readval);
}

enum angl_ext_info {
	ADMT4000_ANGLE_FILT_EN,
	ADMT4000_H1MAG_CORR,
	ADMT4000_H1PH_CORR,
	ADMT4000_H2MAG_CORR,
	ADMT4000_H2PH_CORR,
	ADMT4000_H3MAG_CORR,
	ADMT4000_H3PH_CORR,
	ADMT4000_H8MAG_CORR,
	ADMT4000_H8PH_CORR,
};

static ssize_t admt4000_angl_read(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   char *buf)
{
	struct admt4000_state *st = iio_priv(indio_dev);
	bool is_en;
	u16 temp;
	int ret;

	mutex_lock(&st->lock);
	switch (private) {
	case ADMT4000_ANGLE_FILT_EN:
		ret = admt4000_get_angle_filt(st, &is_en);
		ret = sprintf(buf, "%d\n", is_en);
		break;
	case ADMT4000_H1MAG_CORR:
		ret = admt4000_get_hmag_config(st, 1, &temp);
		ret = sprintf(buf, "%d\n", temp);
		break;
	case ADMT4000_H1PH_CORR:
		ret = admt4000_get_hphase_config(st, 1, &temp);
		ret = sprintf(buf, "%d\n", temp);
		break;
	case ADMT4000_H2MAG_CORR:
		ret = admt4000_get_hmag_config(st, 2, &temp);
		ret = sprintf(buf, "%d\n", temp);
		break;
	case ADMT4000_H2PH_CORR:
		ret = admt4000_get_hphase_config(st, 2, &temp);
		ret = sprintf(buf, "%d\n", temp);
		break;
	case ADMT4000_H3MAG_CORR:
		ret = admt4000_get_hmag_config(st, 3, &temp);
		ret = sprintf(buf, "%d\n", temp);
		break;
	case ADMT4000_H3PH_CORR:
		ret = admt4000_get_hphase_config(st, 3, &temp);
		ret = sprintf(buf, "%d\n", temp);
		break;
	case ADMT4000_H8MAG_CORR:
		ret = admt4000_get_hmag_config(st, 8, &temp);
		ret = sprintf(buf, "%d\n", temp);
		break;
	case ADMT4000_H8PH_CORR:
		ret = admt4000_get_hphase_config(st, 1, &temp);
		ret = sprintf(buf, "%d\n", temp);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);
	return ret;
}

static ssize_t admt4000_angl_write(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    const char *buf, size_t len)
{
	struct admt4000_state *st = iio_priv(indio_dev);
	int ret;
	u32 temp;
	bool is_en;

	mutex_lock(&st->lock);
	switch (private) {
	case ADMT4000_ANGLE_FILT_EN:
		ret = kstrtobool(buf, &is_en);
		ret = admt4000_set_angle_filt(st, (bool)is_en);
		break;
	case ADMT4000_H1MAG_CORR:
		ret = kstrtouint(buf, 10, &temp);
		ret = admt4000_set_hmag_config(st, 1, temp);
		break;
	case ADMT4000_H1PH_CORR:
		ret = kstrtouint(buf, 10, &temp);
		ret = admt4000_set_hphase_config(st, 1, temp);
		break;
	case ADMT4000_H2MAG_CORR:
		ret = kstrtouint(buf, 10, &temp);
		ret = admt4000_set_hmag_config(st, 2, temp);
		break;
	case ADMT4000_H2PH_CORR:
		ret = kstrtouint(buf, 10, &temp);
		ret = admt4000_set_hphase_config(st, 2, temp);
		break;
	case ADMT4000_H3MAG_CORR:
		ret = kstrtouint(buf, 10, &temp);
		ret = admt4000_set_hmag_config(st, 3, temp);
		break;
	case ADMT4000_H3PH_CORR:
		ret = kstrtouint(buf, 10, &temp);
		ret = admt4000_set_hphase_config(st, 3, temp);
		break;
	case ADMT4000_H8MAG_CORR:
		ret = kstrtouint(buf, 10, &temp);
		ret = admt4000_set_hmag_config(st, 8, temp);
		break;
	case ADMT4000_H8PH_CORR:
		ret = kstrtouint(buf, 10, &temp);
		ret = admt4000_set_hphase_config(st, 8, temp);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);
	return ret ? ret : len;
}

static const struct iio_enum h8_corr_src_available = {
	.items = admt4000_h8_ctrl_str,
	.num_items = ARRAY_SIZE(admt4000_h8_ctrl_str),
	.get = admt4000_get_h8_corr_src,
	.set = admt4000_set_h8_corr_src,
};

#define _ADMT4000_ANGL_INFO(_name, _ident) { \
	.name = _name, \
	.read = admt4000_angl_read, \
	.write = admt4000_angl_write, \
	.private = _ident, \
}

static const struct iio_chan_spec_ext_info admt4000_angl_ext_info[] = {
	_ADMT4000_ANGL_INFO("filter_en", ADMT4000_ANGLE_FILT_EN),
	_ADMT4000_ANGL_INFO("h1mag_corr", ADMT4000_H1MAG_CORR),
	_ADMT4000_ANGL_INFO("h1ph_corr", ADMT4000_H1PH_CORR),
	_ADMT4000_ANGL_INFO("h2mag_corr", ADMT4000_H2MAG_CORR),
	_ADMT4000_ANGL_INFO("h2ph_corr", ADMT4000_H2PH_CORR),
	_ADMT4000_ANGL_INFO("h3mag_corr", ADMT4000_H3MAG_CORR),
	_ADMT4000_ANGL_INFO("h3ph_corr", ADMT4000_H3PH_CORR),
	_ADMT4000_ANGL_INFO("h8mag_corr", ADMT4000_H8MAG_CORR),
	_ADMT4000_ANGL_INFO("h8ph_corr", ADMT4000_H8PH_CORR),
	IIO_ENUM_AVAILABLE("h8_corr_src", IIO_SHARED_BY_TYPE, &h8_corr_src_available),
	IIO_ENUM("h8_corr_src", IIO_SHARED_BY_TYPE, &h8_corr_src_available),
	{ },
};

static const struct iio_chan_spec admt4000_channels[] = {
	{
		.type = IIO_TEMP,
		.address = ADMT4000_TEMP,
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
			.shift = 0,
			.endianness = 1
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET)
	},
	{
		.type = IIO_ANGL,
		.address = ADMT4000_ANGLE,
		.scan_type = {
			.sign = 's',
			.realbits = 14,
			.storagebits = 16,
			.shift = 0,
			.endianness = 1
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.ext_info = admt4000_angl_ext_info,
	},
	{
		.type = IIO_COUNT,
		.address = ADMT4000_TURN_COUNT,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
			.endianness = 1
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_PROCESSED)
	},
};

static const struct admt4000_chip_info admt4000_chip = {
	.name = "admt4000",
	.channels = admt4000_channels,
	.num_channels = ARRAY_SIZE(admt4000_channels),
};

static const struct iio_info admt4000_info = {
	.read_raw = admt4000_read_raw,
	.debugfs_reg_access = admt4000_debug_reg,
};

static int admt4000_probe(struct spi_device *spi)
{
	static const char * const regulators[] = { "vdd", "iovdd" };
	struct iio_dev *indio_dev;
	struct device *dev = &spi->dev;
	struct admt4000_state *st;
	int ret;
	bool bool_temp;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->regmap = devm_regmap_init(dev, NULL, st, &admt4000_spi_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "adi,admt4000 Failed to init regmap");

	st->spi = spi;

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	st->chip_info = spi_get_device_match_data(spi);

	if (!st->chip_info)
		return -ENODEV;

	indio_dev->name = st->chip_info->name;
	indio_dev->info = &admt4000_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(regulators),
					     regulators);
	if (ret)
		return dev_err_probe(dev, ret, "adi,admt4000 Failed to enable regulators\n");

	st->ext_vdd = devm_regulator_get_enable_read_voltage(dev, "vdd");
	if (st->ext_vdd != ADMT4000_3P3V && st->ext_vdd != ADMT4000_5V)
		return -EINVAL;

	ret = admt4000_set_page(st, false);
	if (ret)
		return ret;

	st->busy_gpio = devm_gpiod_get_optional(dev, "busy", GPIOD_IN);
	if (IS_ERR(st->busy_gpio))
		return dev_err_probe(dev, PTR_ERR(st->busy_gpio),
				     "Failed to get busy GPIO\n");

	if (st->busy_gpio) {
		ret = admt4000_gpio_func(st, 00, true);
		if (ret)
			return ret;
		ret = admt4000_io_en(st, 0, true);
		if (ret)
			return ret;
	}

	st->cnv_gpio = devm_gpiod_get_optional(dev, "cnv", GPIOD_OUT_LOW);
	if (IS_ERR(st->cnv_gpio))
		return dev_err_probe(dev, PTR_ERR(st->cnv_gpio),
				     "Failed to get cnv GPIO\n");

	if (st->cnv_gpio) {
		ret = admt4000_gpio_func(st, 1, true);
		if (ret)
			return ret;
	}

	st->acalc_gpio = devm_gpiod_get_optional(dev, "acalc", GPIOD_IN);
	if (IS_ERR(st->acalc_gpio))
		return dev_err_probe(dev, PTR_ERR(st->acalc_gpio),
				     "Failed to get acalc GPIO\n");
	if (st->acalc_gpio) {
		ret = admt4000_gpio_func(st, 3, true);
		if (ret)
			return ret;

		ret = admt4000_io_en(st, 3, true);
		if (ret)
			return ret;

		st->acalc_irq = gpiod_to_irq(st->acalc_gpio);
		if (st->acalc_irq < 0)
			return st->acalc_irq;

		ret = devm_request_irq(dev, st->acalc_irq, admt4000_interrupt,
							   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
							   indio_dev->name, indio_dev);
		if (ret)
			return ret;
	}

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &admt4000_trigger_handler, NULL);
	if (ret) {
		dev_err(dev, "iio triggered buffer setup failed\n");
		return ret;
	}

	ret = admt4000_clear_all_faults(st);
	if (ret)
		return dev_err_probe(dev, ret, "adi,admt4000 Failed to clear all faults\n");

	st->is_one_shot = device_property_read_bool(dev, "adi,one-shot-conversion");
	ret = admt4000_set_conv_mode(st, st->is_one_shot);
	if (ret)
		return dev_err_probe(dev, ret, "adi,admt4000 Failed to set conv mode\n");

	ret = admt4000_set_page(st, true);
	if (ret)
		return ret;

	ret = admt4000_get_page(st, &bool_temp);
	if (ret)
		return ret;

	st->is_page_zero = bool_temp;

	ret = device_property_match_property_string(dev,
			"adi,cnv-sync-mode",
			admt4000_sync_mode_str,
			ARRAY_SIZE(admt4000_sync_mode_str));

	if (ret == 0) {
		ret = admt4000_set_conv_sync_mode(st, ADMT4000_SYNC_SEQ_CTRL);
		if (ret)
			return ret;
	} else if (ret == 2) {
		ret = admt4000_set_conv_sync_mode(st, ADMT4000_SYNC_START_EDGE);
		if (ret)
			return ret;
	}

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id admt4000_of_match[] = {
	{ .compatible = "adi,admt4000", .data = &admt4000_chip},
	{ }
};
MODULE_DEVICE_TABLE(of, admt4000_of_match);

static const struct spi_device_id admt4000_id[] = {
	{"admt4000" },
	{ },
};
MODULE_DEVICE_TABLE(spi, admt4000_id);

static struct spi_driver admt4000_driver = {
	.driver = {
		.name = "admt4000",
		.of_match_table = admt4000_of_match,
	},
	.probe = admt4000_probe,
	.id_table = admt4000_id,
};
module_spi_driver(admt4000_driver);

MODULE_AUTHOR("Celine Joy Capua <celinejoy.capua@analog.com>");
MODULE_DESCRIPTION("ADMT4000");
MODULE_LICENSE("GPL");
