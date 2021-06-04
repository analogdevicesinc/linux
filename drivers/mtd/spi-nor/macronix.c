// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 */

#include <linux/mtd/spi-nor.h>

#include "core.h"

#define SPINOR_OP_RD_CR2		0x71		/* Read configuration register 2 */
#define SPINOR_OP_WR_CR2		0x72		/* Write configuration register 2 */
#define SPINOR_REG_MXIC_CR2_MODE	0x00000000	/* For setting octal DTR mode */
#define SPINOR_REG_MXIC_OPI_DTR_EN	0x2		/* Enable Octal DTR */
#define SPINOR_REG_MXIC_SPI_EN		0x0		/* Enable SPI */

static int
mx25l25635_post_bfpt_fixups(struct spi_nor *nor,
			    const struct sfdp_parameter_header *bfpt_header,
			    const struct sfdp_bfpt *bfpt)
{
	/*
	 * MX25L25635F supports 4B opcodes but MX25L25635E does not.
	 * Unfortunately, Macronix has re-used the same JEDEC ID for both
	 * variants which prevents us from defining a new entry in the parts
	 * table.
	 * We need a way to differentiate MX25L25635E and MX25L25635F, and it
	 * seems that the F version advertises support for Fast Read 4-4-4 in
	 * its BFPT table.
	 */
	if (bfpt->dwords[SFDP_DWORD(5)] & BFPT_DWORD5_FAST_READ_4_4_4)
		nor->flags |= SNOR_F_4B_OPCODES;

	return 0;
}

static const struct spi_nor_fixups mx25l25635_fixups = {
	.post_bfpt = mx25l25635_post_bfpt_fixups,
};

/**
 * spi_nor_macronix_set_octal_dtr() - Enable/Disable octal DTR on Macronix flashes.
 * @nor:		pointer to a 'struct spi_nor'
 * @enable:		whether to enable Octal DTR or switch back to SPI
 *
 * Return: 0 on success, -errno otherwise.
 */
static int spi_nor_macronix_set_octal_dtr(struct spi_nor *nor, bool enable)
{
	struct spi_mem_op op;
	u8 *buf = nor->bouncebuf, i;
	int ret;

	/* Set/unset the octal and DTR enable bits. */
	ret = spi_nor_write_enable(nor);
	if (ret)
		return ret;

	if (enable) {
		buf[0] = SPINOR_REG_MXIC_OPI_DTR_EN;
	} else {
		/*
		 * The register is 1-byte wide, but 1-byte transactions are not
		 * allowed in 8D-8D-8D mode. Since there is no register at the
		 * next location, just initialize the value to 0 and let the
		 * transaction go on.
		 */
		buf[0] = SPINOR_REG_MXIC_SPI_EN;
		buf[1] = 0x0;
	}

	op = (struct spi_mem_op)
		SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_WR_CR2, 1),
			   SPI_MEM_OP_ADDR(4, SPINOR_REG_MXIC_CR2_MODE, 1),
			   SPI_MEM_OP_NO_DUMMY,
			   SPI_MEM_OP_DATA_OUT(enable ? 1 : 2, buf, 1));

	if (!enable)
		spi_nor_spimem_setup_op(nor, &op, SNOR_PROTO_8_8_8_DTR);

	ret = spi_mem_exec_op(nor->spimem, &op);
	if (ret)
		return ret;

	/* Read flash ID to make sure the switch was successful. */
	op = (struct spi_mem_op)
		SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_RDID, 1),
			   SPI_MEM_OP_ADDR(enable ? 4 : 0, 0, 1),
			   SPI_MEM_OP_DUMMY(enable ? 4 : 0, 1),
			   SPI_MEM_OP_DATA_IN(SPI_NOR_MAX_ID_LEN, buf, 1));

	if (enable)
		spi_nor_spimem_setup_op(nor, &op, SNOR_PROTO_8_8_8_DTR);

	ret = spi_mem_exec_op(nor->spimem, &op);
	if (ret)
		return ret;

	if (enable) {
		for (i = 0; i < nor->info->id->len; i++)
			if (buf[i * 2] != nor->info->id->bytes[i])
				return -EINVAL;
	} else {
		if (memcmp(buf, nor->info->id->bytes, nor->info->id->len))
			return -EINVAL;
	}

	return 0;
}

static void octaflash_default_init(struct spi_nor *nor)
{
	nor->params->set_octal_dtr = spi_nor_macronix_set_octal_dtr;
}

static struct spi_nor_fixups octaflash_fixups = {
	.default_init = octaflash_default_init,
};

static const struct flash_info macronix_nor_parts[] = {
	{
		.id = SNOR_ID(0xc2, 0x20, 0x10),
		.name = "mx25l512e",
		.size = SZ_64K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x12),
		.name = "mx25l2005a",
		.size = SZ_256K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x13),
		.name = "mx25l4005a",
		.size = SZ_512K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x14),
		.name = "mx25l8005",
		.size = SZ_1M,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x15),
		.name = "mx25l1606e",
		.size = SZ_2M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x16),
		.name = "mx25l3205d",
		.size = SZ_4M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x17),
		.name = "mx25l6405d",
		.size = SZ_8M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x18),
		.name = "mx25l12805d",
		.size = SZ_16M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_4BIT_BP,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x19),
		.name = "mx25l25635e",
		.size = SZ_32M,
		.no_sfdp_flags = SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixups = &mx25l25635_fixups
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x1a),
		.name = "mx66l51235f",
		.size = SZ_64M,
		.no_sfdp_flags = SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixup_flags = SPI_NOR_4B_OPCODES,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x1b),
		.name = "mx66l1g45g",
		.size = SZ_128M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x23, 0x14),
		.name = "mx25v8035f",
		.size = SZ_1M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x32),
		.name = "mx25u2033e",
		.size = SZ_256K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x33),
		.name = "mx25u4035",
		.size = SZ_512K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x34),
		.name = "mx25u8035",
		.size = SZ_1M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x36),
		.name = "mx25u3235f",
		.size = SZ_4M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x37),
		.name = "mx25u6435f",
		.size = SZ_8M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x38),
		.name = "mx25u12835f",
		.size = SZ_16M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x39),
		.name = "mx25u25635f",
		.size = SZ_32M,
		.no_sfdp_flags = SECT_4K,
		.fixup_flags = SPI_NOR_4B_OPCODES,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x3a),
		.name = "mx25u51245g",
		.size = SZ_64M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixup_flags = SPI_NOR_4B_OPCODES,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x3a),
		.name = "mx66u51235f",
		.size = SZ_64M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixup_flags = SPI_NOR_4B_OPCODES,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x3c),
		.name = "mx66u2g45g",
		.size = SZ_256M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixup_flags = SPI_NOR_4B_OPCODES,
	}, {
		.id = SNOR_ID(0xc2, 0x26, 0x18),
		.name = "mx25l12855e",
		.size = SZ_16M,
	}, {
		.id = SNOR_ID(0xc2, 0x26, 0x19),
		.name = "mx25l25655e",
		.size = SZ_32M,
	}, {
		.id = SNOR_ID(0xc2, 0x26, 0x1b),
		.name = "mx66l1g55g",
		.size = SZ_128M,
		.no_sfdp_flags = SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x28, 0x15),
		.name = "mx25r1635f",
		.size = SZ_2M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x28, 0x16),
		.name = "mx25r3235f",
		.size = SZ_4M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x81, 0x3a),
		.name = "mx25uw51245g",
		.n_banks = 4,
		.flags = SPI_NOR_RWW,
	}, {
		.id = SNOR_ID(0xc2, 0x84, 0x3a),
		.name = "mx25uw51345g",
		.size = SZ_64M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_OCTAL_DTR_READ | SPI_NOR_OCTAL_DTR_PP,
		.fixup_flags = SPI_NOR_4B_OPCODES,
		.fixups = &octaflash_fixups,
	}, {
		.id = SNOR_ID(0xc2, 0x9e, 0x16),
		.name = "mx25l3255e",
		.size = SZ_4M,
		.no_sfdp_flags = SECT_4K,
	}
};

static void macronix_nor_default_init(struct spi_nor *nor)
{
	nor->params->quad_enable = spi_nor_sr1_bit6_quad_enable;
}

static int macronix_nor_late_init(struct spi_nor *nor)
{
	if (!nor->params->set_4byte_addr_mode)
		nor->params->set_4byte_addr_mode = spi_nor_set_4byte_addr_mode_en4b_ex4b;

	return 0;
}

static const struct spi_nor_fixups macronix_nor_fixups = {
	.default_init = macronix_nor_default_init,
	.late_init = macronix_nor_late_init,
};

const struct spi_nor_manufacturer spi_nor_macronix = {
	.name = "macronix",
	.parts = macronix_nor_parts,
	.nparts = ARRAY_SIZE(macronix_nor_parts),
	.fixups = &macronix_nor_fixups,
};
