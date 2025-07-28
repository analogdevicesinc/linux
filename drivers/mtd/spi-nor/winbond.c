// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 */

#include <linux/mtd/spi-nor.h>

#include "core.h"

#define WINBOND_NOR_OP_RDEAR	0xc8	/* Read Extended Address Register */
#define WINBOND_NOR_OP_WREAR	0xc5	/* Write Extended Address Register */
#define	WINBOND_NOR_NUM_DIE	0x04	/* Number of Die */

#define WINBOND_NOR_WREAR_OP(buf)					\
	SPI_MEM_OP(SPI_MEM_OP_CMD(WINBOND_NOR_OP_WREAR, 0),		\
		   SPI_MEM_OP_NO_ADDR,					\
		   SPI_MEM_OP_NO_DUMMY,					\
		   SPI_MEM_OP_DATA_OUT(1, buf, 0))

static int
w25q128_post_bfpt_fixups(struct spi_nor *nor,
			 const struct sfdp_parameter_header *bfpt_header,
			 const struct sfdp_bfpt *bfpt)
{
	struct spi_nor_flash_parameter *params = spi_nor_get_params(nor, 0);

	/*
	 * Zetta ZD25Q128C is a clone of the Winbond device. But the encoded
	 * size is really wrong. It seems that they confused Mbit with MiB.
	 * Thus the flash is discovered as a 2MiB device.
	 */
	if (bfpt_header->major == SFDP_JESD216_MAJOR &&
	    bfpt_header->minor == SFDP_JESD216_MINOR &&
	    params->size == SZ_2M &&
	    params->erase_map.regions[0].size == SZ_2M) {
		params->size = SZ_16M;
		params->erase_map.regions[0].size = SZ_16M;
	}

	return 0;
}

static const struct spi_nor_fixups w25q128_fixups = {
	.post_bfpt = w25q128_post_bfpt_fixups,
};

static int
w25q256_post_bfpt_fixups(struct spi_nor *nor,
			 const struct sfdp_parameter_header *bfpt_header,
			 const struct sfdp_bfpt *bfpt)
{
	/*
	 * W25Q256JV supports 4B opcodes but W25Q256FV does not.
	 * Unfortunately, Winbond has re-used the same JEDEC ID for both
	 * variants which prevents us from defining a new entry in the parts
	 * table.
	 * To differentiate between W25Q256JV and W25Q256FV check SFDP header
	 * version: only JV has JESD216A compliant structure (version 5).
	 */
	if (bfpt_header->major == SFDP_JESD216_MAJOR &&
	    bfpt_header->minor == SFDP_JESD216A_MINOR)
		nor->flags |= SNOR_F_4B_OPCODES;

	return 0;
}

static const struct spi_nor_fixups w25q256_fixups = {
	.post_bfpt = w25q256_post_bfpt_fixups,
};

static int spi_nor_multi_die_sr_ready(struct spi_nor *nor)
{
	u8 die;
	int ret;

	do
		ret = spi_nor_sr_ready(nor);
	while (!ret);
	if (ret < 0)
		return ret;

	for (die = 0; die < WINBOND_NOR_NUM_DIE; die++) {
		nor->bouncebuf[0] = die;
		if (nor->spimem) {
			struct spi_mem_op op = SPI_NOR_DIESEL_OP(nor->bouncebuf);

			spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);
			ret = spi_mem_exec_op(nor->spimem, &op);
		} else {
			ret = spi_nor_controller_ops_write_reg(nor, SPINOR_OP_DIESEL,
							       nor->bouncebuf, 1);
		}

		if (ret) {
			dev_dbg(nor->dev, "error %d Switching Die\n", ret);
			return ret;
		}

		do
			ret = spi_nor_sr_ready(nor);
		while (!ret);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static void w25q02_default_init_fixups(struct spi_nor *nor)
{
	struct spi_nor_flash_parameter *params = spi_nor_get_params(nor, 0);

	params->ready = spi_nor_multi_die_sr_ready;
}

static const struct spi_nor_fixups w25q02_fixups = {
	.default_init = w25q02_default_init_fixups,
};

static const struct flash_info winbond_nor_parts[] = {
	{
		.id = SNOR_ID(0xef, 0x30, 0x10),
		.name = "w25x05",
		.size = SZ_64K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x30, 0x11),
		.name = "w25x10",
		.size = SZ_128K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x30, 0x12),
		.name = "w25x20",
		.size = SZ_256K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x30, 0x13),
		.name = "w25x40",
		.size = SZ_512K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x30, 0x14),
		.name = "w25x80",
		.size = SZ_1M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x30, 0x15),
		.name = "w25x16",
		.size = SZ_2M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x30, 0x16),
		.name = "w25x32",
		.size = SZ_4M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x30, 0x17),
		.name = "w25x64",
		.size = SZ_8M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x40, 0x12),
		.name = "w25q20cl",
		.size = SZ_256K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x40, 0x14),
		.name = "w25q80bl",
		.size = SZ_1M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x40, 0x16),
		.name = "w25q32",
		.size = SZ_4M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x40, 0x17),
		.name = "w25q64",
		.size = SZ_8M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xef, 0x40, 0x18),
		.name = "w25q128",
		.size = SZ_16M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixups = &w25q128_fixups,
	}, {
		.id = SNOR_ID(0xef, 0x40, 0x19),
		.name = "w25q256",
		.size = SZ_32M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixups = &w25q256_fixups,
	}, {
		.id = SNOR_ID(0xef, 0x40, 0x20),
		.name = "w25q512jvq",
		.size = SZ_64M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xef, 0x50, 0x12),
		.name = "w25q20bw",
		.size = SZ_256K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x50, 0x14),
		.name = "w25q80",
		.size = SZ_1M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x60, 0x12),
		.name = "w25q20ew",
		.size = SZ_256K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x60, 0x15),
		.name = "w25q16dw",
		.size = SZ_2M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xef, 0x60, 0x16),
		.name = "w25q32dw",
		.size = SZ_4M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.otp = SNOR_OTP(256, 3, 0x1000, 0x1000),
	}, {
		.id = SNOR_ID(0xef, 0x60, 0x17),
		.name = "w25q64dw",
		.size = SZ_8M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xef, 0x60, 0x18),
		.name = "w25q128fw",
		.size = SZ_16M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_4BIT_BP | SPI_NOR_BP3_SR_BIT6,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xef, 0x60, 0x19),
		.name = "w25q256jw",
		.size = SZ_32M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xef, 0x60, 0x20),
		.name = "w25q512nwq",
		.otp = SNOR_OTP(256, 3, 0x1000, 0x1000),
	}, {
		.id = SNOR_ID(0xef, 0x70, 0x15),
		.name = "w25q16jv-im/jm",
		.size = SZ_2M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xef, 0x70, 0x16),
		.name = "w25q32jv",
		.size = SZ_4M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xef, 0x70, 0x17),
		.name = "w25q64jvm",
		.size = SZ_8M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xef, 0x70, 0x18),
		.name = "w25q128jv",
		.size = SZ_16M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_BP3_SR_BIT6,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xef, 0x70, 0x19),
		.name = "w25q256jvm",
	}, {
		.id = SNOR_ID(0xef, 0x71, 0x19),
		.name = "w25m512jv",
		.size = SZ_64M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xef, 0x80, 0x16),
		.name = "w25q32jwm",
		.size = SZ_4M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.otp = SNOR_OTP(256, 3, 0x1000, 0x1000),
	}, {
		.id = SNOR_ID(0xef, 0x80, 0x17),
		.name = "w25q64jwm",
		.size = SZ_8M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xef, 0x80, 0x18),
		.name = "w25q128jwm",
		.size = SZ_16M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xef, 0x80, 0x19),
		.name = "w25q256jwm",
		.size = SZ_32M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB |
				SPI_NOR_4BIT_BP | SPI_NOR_TB_SR_BIT6 | SPI_NOR_BP3_SR_BIT5,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xef, 0x80, 0x20),
		.name = "w25q512nwm",
		.otp = SNOR_OTP(256, 3, 0x1000, 0x1000),
	}, {
		.id = SNOR_ID(0xef, 0x80, 0x22),
		.name = "w25q02nw",
		.size = SZ_8M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_TB_SR_BIT6 |
				SPI_NOR_4BIT_BP | SPI_NOR_BP3_SR_BIT5,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixup_flags = SPI_NOR_4B_OPCODES,
		.fixups = &w25q02_fixups,
	}, {
		.id = SNOR_ID(0xef, 0x90, 0x22),
		.name = "w25q02nw",
		.size = SZ_8M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_HAS_TB | SPI_NOR_TB_SR_BIT6 |
				SPI_NOR_4BIT_BP | SPI_NOR_BP3_SR_BIT5,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixup_flags = SPI_NOR_4B_OPCODES,
		.fixups = &w25q02_fixups,
	},
};

/**
 * winbond_nor_write_ear() - Write Extended Address Register.
 * @nor:	pointer to 'struct spi_nor'.
 * @ear:	value to write to the Extended Address Register.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int winbond_nor_write_ear(struct spi_nor *nor, u8 ear)
{
	int ret;

	nor->bouncebuf[0] = ear;

	if (nor->spimem) {
		struct spi_mem_op op = WINBOND_NOR_WREAR_OP(nor->bouncebuf);

		spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);

		ret = spi_mem_exec_op(nor->spimem, &op);
	} else {
		ret = spi_nor_controller_ops_write_reg(nor,
						       WINBOND_NOR_OP_WREAR,
						       nor->bouncebuf, 1);
	}

	if (ret)
		dev_dbg(nor->dev, "error %d writing EAR\n", ret);

	return ret;
}

/**
 * winbond_nor_set_4byte_addr_mode() - Set 4-byte address mode for Winbond
 * flashes.
 * @nor:	pointer to 'struct spi_nor'.
 * @enable:	true to enter the 4-byte address mode, false to exit the 4-byte
 *		address mode.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int winbond_nor_set_4byte_addr_mode(struct spi_nor *nor, bool enable)
{
	int ret;

	ret = spi_nor_set_4byte_addr_mode_en4b_ex4b(nor, enable);
	if (ret || enable)
		return ret;

	/*
	 * On Winbond W25Q256FV, leaving 4byte mode causes the Extended Address
	 * Register to be set to 1, so all 3-byte-address reads come from the
	 * second 16M. We must clear the register to enable normal behavior.
	 */
	ret = spi_nor_write_enable(nor);
	if (ret)
		return ret;

	ret = winbond_nor_write_ear(nor, 0);
	if (ret)
		return ret;

	return spi_nor_write_disable(nor);
}

static const struct spi_nor_otp_ops winbond_nor_otp_ops = {
	.read = spi_nor_otp_read_secr,
	.write = spi_nor_otp_write_secr,
	.erase = spi_nor_otp_erase_secr,
	.lock = spi_nor_otp_lock_sr2,
	.is_locked = spi_nor_otp_is_locked_sr2,
};

static int winbond_nor_late_init(struct spi_nor *nor)
{
	struct spi_nor_flash_parameter *params = spi_nor_get_params(nor, 0);

	if (params->otp.org)
		params->otp.ops = &winbond_nor_otp_ops;

	/*
	 * Winbond seems to require that the Extended Address Register to be set
	 * to zero when exiting the 4-Byte Address Mode, at least for W25Q256FV.
	 * This requirement is not described in the JESD216 SFDP standard, thus
	 * it is Winbond specific. Since we do not know if other Winbond flashes
	 * have the same requirement, play safe and overwrite the method parsed
	 * from BFPT, if any.
	 */
	params->set_4byte_addr_mode = winbond_nor_set_4byte_addr_mode;

	return 0;
}

static const struct spi_nor_fixups winbond_nor_fixups = {
	.late_init = winbond_nor_late_init,
};

const struct spi_nor_manufacturer spi_nor_winbond = {
	.name = "winbond",
	.parts = winbond_nor_parts,
	.nparts = ARRAY_SIZE(winbond_nor_parts),
	.fixups = &winbond_nor_fixups,
};
