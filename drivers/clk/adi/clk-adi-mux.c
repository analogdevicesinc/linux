// SPDX-License-Identifier: GPL-2.0-only
/*
 * Clock distribution unit (CDU) mux support
 * for ADSP-SC5xx processors
 *
 * Copyright (C) 2026 Analog Devices Inc.
 *
 * Author: Qasim Ijaz <qasim.ijaz@analog.com>
 * Contact: linux@analog.com
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "clk.h"

#define IN0_CLKON       0x0
#define IN1_CLKON       0x1
#define IN2_CLKON       0x2
#define IN3_CLKON       0x3

#define CDU_CLKO0       0x0
#define CDU_CLKO1       0x1
#define CDU_CLKO2       0x2
#define CDU_CLKO3       0x3
#define CDU_CLKO4       0x4
#define CDU_CLKO5       0x5
#define CDU_CLKO6       0x6
#define CDU_CLKO7       0x7
#define CDU_CLKO8       0x8
#define CDU_CLKO9       0x9
#define CDU_CLKO10      0xA
#define CDU_CLKO12      0xC
#define CDU_CLKO13      0xD
#define CDU_CLKO14      0xE

#define CLK_PARENT_DATA(_name)  \
        static const struct clk_parent_data _name[]

#define MUX_TABLE(_name)	\
        static const u32 _name[]

#define SC5XX_CDU_CFG(n)	((n) * sizeof(u32))

#define SC5XX_CDU_STAT		0x40
#define SC5XX_CDU_CLKINSEL	0x44
#define SC5XX_CDU_CLKINSEL_CGU1	0x00
#define SC5XX_CDU_STAT_LWERR	BIT(17)
#define SC5XX_CDU_STAT_ADRERR	BIT(16)

#define SC5XX_CDU_CFG_EN	BIT(0)
#define SC5XX_CDU_CFG_LOCK	BIT(31)
#define SC5XX_CDU_CFG_SEL	GENMASK(2, 1)

#define SC5XX_CDU_REVID		0x48
#define SC5XX_CDU_REVID_MAJOR	GENMASK(7, 4)
#define SC5XX_CDU_REVID_REV	GENMASK(3, 0)

#define SC5XX_CDU_POLL_DELAY	5
#define SC5XX_CDU_POLL_TIMEOUT	50

/**
 * struct sc5xx_cdu_clks - information about cdu clock
 * @parent_data: parent clock descriptions used by CCF
 * @mux_table: input clock SEL table for this clock
 * @clock_name: name of this clock
 * @clock_flags: optional flags for this clock
 * @id: clock provider onecell identifier
 * @cdu_clkol: the CDU CLKO bit
 * @num_parents: number of entries in @parent_data
 */
struct sc5xx_cdu_clks {
	const struct clk_parent_data	*parent_data;
	const u32			*mux_table;
	const char			*clock_name;
	unsigned long			clock_flags;
	unsigned int			id;
	u8				cdu_clkol;
	u8				num_parents;
};

#define CDU(_id, _parents, _mux, _name, _flags, _clkol)	\
	{							\
		.id		= _id,				\
		.parent_data	= _parents,			\
		.mux_table	= _mux,				\
		.clock_name	= _name,			\
		.clock_flags	= _flags,			\
		.cdu_clkol	= _clkol,			\
		.num_parents	= ARRAY_SIZE(_parents),		\
	}
/**
 * struct sc5xx_cdu_match_data - SoC specific CDU clock data
 * @clks: array of CDU clock descriptions for the SoC
 * @soc_name: human readable SoC name
 * @num_clks: number of entries in @clks
 * @clko_max: highest supported CDU output index
 */
struct sc5xx_cdu_match_data {
	const struct sc5xx_cdu_clks	*clks;
	const char			*soc_name;
	unsigned int			num_clks;
	unsigned int			clko_max;
};

/* CDU CLKOn input clock source configurations */
MUX_TABLE(CDU_CLKO_SEL1)	= { IN0_CLKON };
MUX_TABLE(CDU_CLKO_SEL2)	= { IN1_CLKON };
MUX_TABLE(CDU_CLKO_SEL3)	= { IN0_CLKON, IN1_CLKON };
MUX_TABLE(CDU_CLKO_SEL4)	= { IN0_CLKON, IN1_CLKON, IN2_CLKON };
MUX_TABLE(CDU_CLKO_SEL5)	= { IN0_CLKON, IN2_CLKON, IN3_CLKON };
MUX_TABLE(CDU_CLKO_SEL6)	= { IN0_CLKON, IN1_CLKON, IN2_CLKON, IN3_CLKON };

/* ADSP-SC598 and ADSP-SC594 common CDU parents */
CLK_PARENT_DATA(sc59x_cdu_sharc_parents)        = { { .fw_name = "cclk0_0"   }, };
CLK_PARENT_DATA(sc59x_cdu_spdif_parents)        = { { .fw_name = "sclk1_0"   }, };
CLK_PARENT_DATA(sc59x_cdu_trace_parents)        = { { .fw_name = "sclk0_0"   }, };
CLK_PARENT_DATA(sc59x_cdu_ddr_parents)          = { { .fw_name = "dclk_0"    }, { .fw_name = "dclk_1"    }, };
CLK_PARENT_DATA(sc59x_cdu_spi_parents)          = { { .fw_name = "sclk0_0"   }, { .fw_name = "oclk_0"    }, };
CLK_PARENT_DATA(sc59x_cdu_lp_parents)           = { { .fw_name = "oclk_0"    }, { .fw_name = "sclk0_0"   }, { .fw_name = "cclk0_1"  }, };
CLK_PARENT_DATA(sc59x_cdu_lpddr_parents)        = { { .fw_name = "oclk_0"    }, { .fw_name = "dclk_0"    }, { .fw_name = "sysclk_1" }, };
CLK_PARENT_DATA(sc59x_cdu_ospi_refclk_parents)  = { { .fw_name = "sysclk_0"  }, { .fw_name = "sclk0_0"   }, { .fw_name = "sclk1_1"  }, };

/* ADSP-SC589 and ADSP-SC573 common CDU parents */
CLK_PARENT_DATA(sc589_573_cdu_sharc_parents)	= { { .fw_name = "cclk0_0"   }, { .fw_name = "sysclk_0"  }, };
CLK_PARENT_DATA(sc589_573_cdu_arm_parents)	= { { .fw_name = "cclk1_0"   }, { .fw_name = "sysclk_0"  }, };
CLK_PARENT_DATA(sc589_573_cdu_ddr_parents)	= { { .fw_name = "dclk_0"    }, { .fw_name = "dclk_1"    }, };
CLK_PARENT_DATA(sc589_573_cdu_spdif_parents)	= { { .fw_name = "oclk_0"    }, { .fw_name = "oclk_1"    }, { .fw_name = "dclk_1"   }, { .fw_name = "dclk_0"   }, };
CLK_PARENT_DATA(sc589_573_cdu_gige_parents)	= { { .fw_name = "sclk1_0"   }, { .fw_name = "sclk1_1"   }, { .fw_name = "cclk0_1"  }, { .fw_name = "oclk_0"   }, };
CLK_PARENT_DATA(sc589_573_cdu_sdio_parents)	= { { .fw_name = "oclk_0/2"  }, { .fw_name = "cclk1_1/2" }, { .fw_name = "cclk1_1"  }, { .fw_name = "dclk_1"   }, };

/* ADSP-SC598 CDU parents */
CLK_PARENT_DATA(sc598_cdu_can_parents)          = { { .fw_name = "oclk_1"    }, };
CLK_PARENT_DATA(sc598_cdu_emmc_timer_parents)   = { { .fw_name = "sclk1_1/2" }, };
CLK_PARENT_DATA(sc598_cdu_arm_parents)          = { { .fw_name = "cclk2_0"   }, { .fw_name = "cclk2_1"   }, };
CLK_PARENT_DATA(sc598_cdu_gige_parents)         = { { .fw_name = "sclk0_0"   }, { .fw_name = "sclk0_1"   }, { .fw_name = "oclk_0"   }, };
CLK_PARENT_DATA(sc598_cdu_emmc_parents)         = { { .fw_name = "oclk_0"    }, { .fw_name = "sclk0_1"   }, { .fw_name = "dclk_0/2" },
                                                    { .fw_name = "dclk_1/2"  }, };
/* ADSP-SC594 CDU parents */
CLK_PARENT_DATA(sc594_cdu_arm_parents)          = { { .fw_name = "cclk1_0"   }, };
CLK_PARENT_DATA(sc594_cdu_can_parents)          = { { .fw_name = "oclk_0"    }, { .fw_name = "oclk_1"    }, };
CLK_PARENT_DATA(sc594_cdu_gige_parents)         = { { .fw_name = "sclk0_0"   }, { .fw_name = "sclk0_1"   }, };

/* ADSP-SC589 CDU parents */
CLK_PARENT_DATA(sc589_cdu_reserved_parents)     = { { .fw_name = "oclk_0"    }, { .fw_name = "cclk0_1"   }, };
CLK_PARENT_DATA(sc589_cdu_can_parents)          = { { .fw_name = "oclk_0"    }, { .fw_name = "oclk_1"    }, { .fw_name = "dclk_1"   }, };
CLK_PARENT_DATA(sc589_cdu_lp_parents)           = { { .fw_name = "sclk0_0"   }, { .fw_name = "sclk0_1"   }, { .fw_name = "cclk1_1"  }, { .fw_name = "dclk_1"   }, };

/* ADSP-SC573 CDU parents */
CLK_PARENT_DATA(sc573_cdu_can_parents)          = { { .fw_name = "oclk_0"    }, { .fw_name = "oclk_1"    }, { .fw_name = "dclk_1"   }, { .fw_name = "oclk_0/2" }, };

static const struct sc5xx_cdu_clks sc598_mux_clks[] = {
	CDU(ADSP_SC5XX_CLK_CDU_DDR,		sc59x_cdu_ddr_parents,		CDU_CLKO_SEL3,	"cdu_ddr",		0,	CDU_CLKO3),
	CDU(ADSP_SC5XX_CLK_CDU_CAN,		sc598_cdu_can_parents,		CDU_CLKO_SEL2,	"cdu_can",		0,	CDU_CLKO4),
	CDU(ADSP_SC5XX_CLK_CDU_SPDIF,		sc59x_cdu_spdif_parents,	CDU_CLKO_SEL1,	"cdu_spdif",		0,	CDU_CLKO5),
	CDU(ADSP_SC598_CLK_CDU_SPI,		sc59x_cdu_spi_parents,		CDU_CLKO_SEL3,	"cdu_spi",		0,	CDU_CLKO6),
	CDU(ADSP_SC5XX_CLK_CDU_GIGE,		sc598_cdu_gige_parents,		CDU_CLKO_SEL4,	"cdu_gige",		0,	CDU_CLKO7),
	CDU(ADSP_SC598_CLK_CDU_LP,		sc59x_cdu_lp_parents,		CDU_CLKO_SEL4,	"cdu_lp",		0,	CDU_CLKO8),
	CDU(ADSP_SC598_CLK_CDU_LPDDR,		sc59x_cdu_lpddr_parents,	CDU_CLKO_SEL4,	"cdu_lpddr",		0,	CDU_CLKO9),
	CDU(ADSP_SC598_CLK_CDU_OSPI_REFCLK,	sc59x_cdu_ospi_refclk_parents,	CDU_CLKO_SEL4,	"cdu_ospi_refclk",	0,	CDU_CLKO10),
	CDU(ADSP_SC598_CLK_CDU_TRACE,		sc59x_cdu_trace_parents,	CDU_CLKO_SEL1,	"cdu_trace",		0,	CDU_CLKO12),
	CDU(ADSP_SC598_CLK_CDU_EMMC,		sc598_cdu_emmc_parents,		CDU_CLKO_SEL6,	"cdu_emmc",		0,	CDU_CLKO13),
	CDU(ADSP_SC598_CLK_CDU_EMMC_TIMER_QMC,	sc598_cdu_emmc_timer_parents,	CDU_CLKO_SEL2,	"cdu_emmc_timer",	0,	CDU_CLKO14),
	CDU(ADSP_SC5XX_CLK_CDU_SHARC0,		sc59x_cdu_sharc_parents,	CDU_CLKO_SEL1,	"cdu_sharc0",		0,	CDU_CLKO0),
	CDU(ADSP_SC5XX_CLK_CDU_SHARC1,		sc59x_cdu_sharc_parents,	CDU_CLKO_SEL1,	"cdu_sharc1",		0,	CDU_CLKO1),
	CDU(ADSP_SC5XX_CLK_CDU_ARM,		sc598_cdu_arm_parents,		CDU_CLKO_SEL5,	"cdu_arm",		0,	CDU_CLKO2),
};

static const struct sc5xx_cdu_clks sc594_mux_clks[] = {
	CDU(ADSP_SC5XX_CLK_CDU_DDR,		sc59x_cdu_ddr_parents,		CDU_CLKO_SEL3,	"cdu_ddr",		0,	CDU_CLKO3),
	CDU(ADSP_SC5XX_CLK_CDU_CAN,		sc594_cdu_can_parents,		CDU_CLKO_SEL3,	"cdu_can",		0,	CDU_CLKO4),
	CDU(ADSP_SC5XX_CLK_CDU_SPDIF,		sc59x_cdu_spdif_parents,	CDU_CLKO_SEL1,	"cdu_spdif",		0,	CDU_CLKO5),
	CDU(ADSP_SC594_CLK_CDU_SPI,		sc59x_cdu_spi_parents,		CDU_CLKO_SEL3,	"cdu_spi",		0,	CDU_CLKO6),
	CDU(ADSP_SC5XX_CLK_CDU_GIGE,		sc594_cdu_gige_parents,		CDU_CLKO_SEL3,	"cdu_gige",		0,	CDU_CLKO7),
	CDU(ADSP_SC594_CLK_CDU_LP,		sc59x_cdu_lp_parents,		CDU_CLKO_SEL4,	"cdu_lp",		0,	CDU_CLKO8),
	CDU(ADSP_SC594_CLK_CDU_LPDDR,		sc59x_cdu_lpddr_parents,	CDU_CLKO_SEL4,	"cdu_lpddr",		0,	CDU_CLKO9),
	CDU(ADSP_SC594_CLK_CDU_OSPI_REFCLK,	sc59x_cdu_ospi_refclk_parents,	CDU_CLKO_SEL4,	"cdu_ospi_refclk",	0,	CDU_CLKO10),
	CDU(ADSP_SC594_CLK_CDU_TRACE,		sc59x_cdu_trace_parents,	CDU_CLKO_SEL1,	"cdu_trace",		0,	CDU_CLKO12),
	CDU(ADSP_SC5XX_CLK_CDU_SHARC0,		sc59x_cdu_sharc_parents,	CDU_CLKO_SEL1,	"cdu_sharc0",		0,	CDU_CLKO0),
	CDU(ADSP_SC5XX_CLK_CDU_SHARC1,		sc59x_cdu_sharc_parents,	CDU_CLKO_SEL1,	"cdu_sharc1",		0,	CDU_CLKO1),
	CDU(ADSP_SC5XX_CLK_CDU_ARM,		sc594_cdu_arm_parents,		CDU_CLKO_SEL1,	"cdu_arm",		0,	CDU_CLKO2),
};

static const struct sc5xx_cdu_clks sc589_mux_clks[] = {
	CDU(ADSP_SC5XX_CLK_CDU_DDR,		sc589_573_cdu_ddr_parents,	CDU_CLKO_SEL3, 	"cdu_ddr",		0,	CDU_CLKO3),
	CDU(ADSP_SC5XX_CLK_CDU_CAN,		sc589_cdu_can_parents, 		CDU_CLKO_SEL4, 	"cdu_can",		0,	CDU_CLKO4),
	CDU(ADSP_SC5XX_CLK_CDU_SPDIF,		sc589_573_cdu_spdif_parents,	CDU_CLKO_SEL6, 	"cdu_spdif",		0,	CDU_CLKO5),
	CDU(ADSP_SC58X_CLK_CDU_RESERVED,	sc589_cdu_reserved_parents,	CDU_CLKO_SEL3, 	"cdu_reserved",		0,	CDU_CLKO6),
	CDU(ADSP_SC5XX_CLK_CDU_GIGE,		sc589_573_cdu_gige_parents,	CDU_CLKO_SEL6, 	"cdu_gige",		0,	CDU_CLKO7),
	CDU(ADSP_SC58X_CLK_CDU_LP,		sc589_cdu_lp_parents,		CDU_CLKO_SEL6, 	"cdu_lp",		0,	CDU_CLKO8),
	CDU(ADSP_SC58X_CLK_CDU_SDIO,		sc589_573_cdu_sdio_parents,	CDU_CLKO_SEL6, 	"cdu_sdio",		0,	CDU_CLKO9),
	CDU(ADSP_SC5XX_CLK_CDU_SHARC0,		sc589_573_cdu_sharc_parents,	CDU_CLKO_SEL3, 	"cdu_sharc0",		0,	CDU_CLKO0),
	CDU(ADSP_SC5XX_CLK_CDU_SHARC1,		sc589_573_cdu_sharc_parents,	CDU_CLKO_SEL3, 	"cdu_sharc1",		0,	CDU_CLKO1),
	CDU(ADSP_SC5XX_CLK_CDU_ARM,		sc589_573_cdu_arm_parents,	CDU_CLKO_SEL3, 	"cdu_arm",		0,	CDU_CLKO2),
};

static const struct sc5xx_cdu_clks sc573_mux_clks[] = {
	CDU(ADSP_SC5XX_CLK_CDU_DDR,		sc589_573_cdu_ddr_parents,	CDU_CLKO_SEL3,	"cdu_ddr",		0,	CDU_CLKO3),
	CDU(ADSP_SC5XX_CLK_CDU_CAN,		sc573_cdu_can_parents,		CDU_CLKO_SEL6,	"cdu_can",		0,	CDU_CLKO4),
	CDU(ADSP_SC5XX_CLK_CDU_SPDIF,		sc589_573_cdu_spdif_parents,	CDU_CLKO_SEL6,	"cdu_spdif",		0,	CDU_CLKO5),
	CDU(ADSP_SC5XX_CLK_CDU_GIGE,		sc589_573_cdu_gige_parents,	CDU_CLKO_SEL6,	"cdu_gige",		0,	CDU_CLKO7),
	CDU(ADSP_SC57X_CLK_CDU_SDIO,		sc589_573_cdu_sdio_parents,	CDU_CLKO_SEL6,	"cdu_sdio",		0,	CDU_CLKO9),
	CDU(ADSP_SC5XX_CLK_CDU_SHARC0,		sc589_573_cdu_sharc_parents,	CDU_CLKO_SEL3,	"cdu_sharc0",		0,	CDU_CLKO0),
	CDU(ADSP_SC5XX_CLK_CDU_SHARC1,		sc589_573_cdu_sharc_parents,	CDU_CLKO_SEL3,	"cdu_sharc1",		0,	CDU_CLKO1),
	CDU(ADSP_SC5XX_CLK_CDU_ARM,		sc589_573_cdu_arm_parents,	CDU_CLKO_SEL3,	"cdu_arm",		0,	CDU_CLKO2),
};

static const struct sc5xx_cdu_match_data sc598_cdu_info = {
	.clks		= sc598_mux_clks,
	.soc_name	= "ADSP-SC598",
	.num_clks	= ARRAY_SIZE(sc598_mux_clks),
	.clko_max	= CDU_CLKO14,
};

static const struct sc5xx_cdu_match_data sc594_cdu_info = {
	.clks		= sc594_mux_clks,
	.soc_name	= "ADSP-SC594",
	.num_clks	= ARRAY_SIZE(sc594_mux_clks),
	.clko_max	= CDU_CLKO12,
};

static const struct sc5xx_cdu_match_data sc589_cdu_info = {
	.clks		= sc589_mux_clks,
	.soc_name	= "ADSP-SC589",
	.num_clks	= ARRAY_SIZE(sc589_mux_clks),
	.clko_max	= CDU_CLKO9,
};

static const struct sc5xx_cdu_match_data sc573_cdu_info = {
	.clks		= sc573_mux_clks,
	.soc_name	= "ADSP-SC573",
	.num_clks	= ARRAY_SIZE(sc573_mux_clks),
	.clko_max	= CDU_CLKO9,
};

struct sc5xx_cdu {
	u8 cdu_clko;
	spinlock_t *lock;
	void __iomem *base;
	struct clk_hw clk_hw;
	const u32 *mux_table;
};

enum sc5xx_cdu_en_state {
	SC5XX_CDU_EN_DISABLE,
	SC5XX_CDU_EN_ENABLE,
};

static void sc5xx_cdu_print_revision(struct device *dev, const char *soc_name,
				     void __iomem *base)
{
        u32 revid = readl(base + SC5XX_CDU_REVID);
        u32 major = FIELD_GET(SC5XX_CDU_REVID_MAJOR, revid);
        u32 rev = FIELD_GET(SC5XX_CDU_REVID_REV, revid);

        dev_info(dev, "%s CDU revision: major=%u rev=%u (0x%08x)\n",
                 soc_name, major, rev, revid);
}

static inline struct sc5xx_cdu *to_sc5xx_cdu(struct clk_hw *clk_hw)
{
	return container_of(clk_hw, struct sc5xx_cdu, clk_hw);
}

static int sc5xx_cdu_check_unlocked(u32 reg)
{
	if (reg & SC5XX_CDU_CFG_LOCK)
		return -EBUSY;

	return 0;
}

/*
 * CDU_STAT.CLKOn is set while a configuration change for CDU_CFG[n] is in
 * progress. Writes to CDU_CFG[n] must only happen when the corresponding
 * status bit is clear.
 */
static int sc5xx_cdu_wait_ready(struct sc5xx_cdu *cdu_clk)
{
	u32 stat;

	return readl_poll_timeout_atomic(cdu_clk->base + SC5XX_CDU_STAT,
					 stat,
					 !(stat & BIT(cdu_clk->cdu_clko)),
					 SC5XX_CDU_POLL_DELAY,
					 SC5XX_CDU_POLL_TIMEOUT);
}

static u32 sc5xx_cdu_readl(struct sc5xx_cdu *cdu_clk, unsigned int offset)
{
	return readl(cdu_clk->base + offset);
}

static void sc5xx_cdu_writel(struct sc5xx_cdu *cdu_clk,
			    unsigned int offset, u32 val)
{
	writel(val, cdu_clk->base + offset);
}

static unsigned int sc5xx_cdu_cfg(struct sc5xx_cdu *cdu_clk)
{
	return SC5XX_CDU_CFG(cdu_clk->cdu_clko);
}

static int sc5xx_cdu_update_en(struct sc5xx_cdu *cdu_clk,
				enum sc5xx_cdu_en_state state)
{
	u32 reg;
	int ret;

	reg = sc5xx_cdu_readl(cdu_clk, sc5xx_cdu_cfg(cdu_clk));

	ret = sc5xx_cdu_check_unlocked(reg);
	if (ret)
		return ret;

	if (state == SC5XX_CDU_EN_ENABLE)
		reg |= SC5XX_CDU_CFG_EN;
	else
		reg &= ~SC5XX_CDU_CFG_EN;

	sc5xx_cdu_writel(cdu_clk, sc5xx_cdu_cfg(cdu_clk), reg);

	return 0;
}

static int sc5xx_cdu_set_parent(struct clk_hw *clk_hw, u8 index)
{
	struct sc5xx_cdu *cdu_clk = to_sc5xx_cdu(clk_hw);
	unsigned long flags;
	unsigned int input_sel;
	u32 reg, readback;
	int ret;

	input_sel = clk_mux_index_to_val(cdu_clk->mux_table, 0, index);

	spin_lock_irqsave(cdu_clk->lock, flags);

	ret = sc5xx_cdu_wait_ready(cdu_clk);
	if (ret)
		goto out;

	reg = sc5xx_cdu_readl(cdu_clk, sc5xx_cdu_cfg(cdu_clk));

	ret = sc5xx_cdu_check_unlocked(reg);
	if (ret)
		goto out;

	reg &= ~SC5XX_CDU_CFG_SEL;
	reg |= FIELD_PREP(SC5XX_CDU_CFG_SEL, input_sel);

	sc5xx_cdu_writel(cdu_clk, sc5xx_cdu_cfg(cdu_clk), reg);

	ret = sc5xx_cdu_wait_ready(cdu_clk);

	/* Verify the mux update before returning success to CCF. */
	readback = sc5xx_cdu_readl(cdu_clk, sc5xx_cdu_cfg(cdu_clk));
	if (FIELD_GET(SC5XX_CDU_CFG_SEL, readback) != input_sel)
		ret = -EIO;

out:
	spin_unlock_irqrestore(cdu_clk->lock, flags);

	return ret;
}

static u8 sc5xx_cdu_get_parent(struct clk_hw *clk_hw)
{
	struct sc5xx_cdu *cdu_clk = to_sc5xx_cdu(clk_hw);
	unsigned long flags;
	u32 reg, input_sel;
	int parent;

	spin_lock_irqsave(cdu_clk->lock, flags);
	reg = sc5xx_cdu_readl(cdu_clk, sc5xx_cdu_cfg(cdu_clk));
	spin_unlock_irqrestore(cdu_clk->lock, flags);

	input_sel = FIELD_GET(SC5XX_CDU_CFG_SEL, reg);

	parent = clk_mux_val_to_index(clk_hw, cdu_clk->mux_table,
					0, input_sel);
	if (parent < 0)
		return 0;

	return parent;
}

static int sc5xx_cdu_enable(struct clk_hw *clk_hw)
{
	struct sc5xx_cdu *cdu_clk = to_sc5xx_cdu(clk_hw);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(cdu_clk->lock, flags);

	ret = sc5xx_cdu_wait_ready(cdu_clk);
	if (ret)
		goto out;

	ret = sc5xx_cdu_update_en(cdu_clk, SC5XX_CDU_EN_ENABLE);
	if (ret)
		goto out;

	ret = sc5xx_cdu_wait_ready(cdu_clk);

out:
	spin_unlock_irqrestore(cdu_clk->lock, flags);

	return ret;
}

static void sc5xx_cdu_disable(struct clk_hw *clk_hw)
{
	struct sc5xx_cdu *cdu_clk = to_sc5xx_cdu(clk_hw);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(cdu_clk->lock, flags);

	ret = sc5xx_cdu_wait_ready(cdu_clk);
	if (ret)
		goto out;

	ret = sc5xx_cdu_update_en(cdu_clk, SC5XX_CDU_EN_DISABLE);
	if (ret)
		goto out;

	sc5xx_cdu_wait_ready(cdu_clk);

out:
	spin_unlock_irqrestore(cdu_clk->lock, flags);
}

static int sc5xx_cdu_is_enabled(struct clk_hw *clk_hw)
{
	struct sc5xx_cdu *cdu_clk = to_sc5xx_cdu(clk_hw);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(cdu_clk->lock, flags);
	reg = sc5xx_cdu_readl(cdu_clk, sc5xx_cdu_cfg(cdu_clk));
	spin_unlock_irqrestore(cdu_clk->lock, flags);

	return !!(reg & SC5XX_CDU_CFG_EN);
}

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int sc5xx_cdu_debug_show(struct seq_file *s, void *v)
{
	struct clk_hw *clk_hw;
	struct sc5xx_cdu *cdu_clk;
	unsigned long flags;
	u32 cfg_reg, stat_reg;

	clk_hw = s->private;
	cdu_clk = to_sc5xx_cdu(clk_hw);

	spin_lock_irqsave(cdu_clk->lock, flags);
	cfg_reg = sc5xx_cdu_readl(cdu_clk, sc5xx_cdu_cfg(cdu_clk));
	stat_reg = sc5xx_cdu_readl(cdu_clk, SC5XX_CDU_STAT);
	spin_unlock_irqrestore(cdu_clk->lock, flags);

	seq_printf(s, "CDU_CFG[%u]: 0x%08x\n", cdu_clk->cdu_clko, cfg_reg);
	seq_printf(s, "  EN:   %u\n", !!(cfg_reg & SC5XX_CDU_CFG_EN));
	seq_printf(s, "  SEL:  %u\n", FIELD_GET(SC5XX_CDU_CFG_SEL, cfg_reg));
	seq_printf(s, "  LOCK: %u\n", !!(cfg_reg & SC5XX_CDU_CFG_LOCK));

	seq_printf(s, "CDU_STAT: 0x%08x\n", stat_reg);
	seq_printf(s, "  LWERR:  %u\n", !!(stat_reg & SC5XX_CDU_STAT_LWERR));
	seq_printf(s, "  ADRERR: %u\n", !!(stat_reg & SC5XX_CDU_STAT_ADRERR));
	seq_printf(s, "  CLKO%u: %u\n", cdu_clk->cdu_clko,
					!!(stat_reg & BIT(cdu_clk->cdu_clko)));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(sc5xx_cdu_debug);

static void sc5xx_cdu_debug_init(struct clk_hw *clk_hw, struct dentry *dentry)
{
	struct sc5xx_cdu *cdu_clk = to_sc5xx_cdu(clk_hw);
	char debugfs_entry_name[12];

	snprintf(debugfs_entry_name, sizeof(debugfs_entry_name),
			"cdu_cfg[%u]", cdu_clk->cdu_clko);

	debugfs_create_file(debugfs_entry_name, 0444, dentry,
				clk_hw, &sc5xx_cdu_debug_fops);
}
#endif

static const struct clk_ops sc5xx_cdu_ops = {
	.determine_rate = clk_hw_determine_rate_no_reparent,
	.set_parent = sc5xx_cdu_set_parent,
	.get_parent = sc5xx_cdu_get_parent,
	.enable = sc5xx_cdu_enable,
	.disable = sc5xx_cdu_disable,
	.is_enabled = sc5xx_cdu_is_enabled,
#ifdef CONFIG_DEBUG_FS
	.debug_init = sc5xx_cdu_debug_init,
#endif
};


/**
 * sc5xx_cdu_register - Register an ADSP-SC5xx CDU output clock mux.
 * @dev: device pointer
 * @clock_name: Name of the clock to register.
 * @base: Base address of the CDU register block.
 * @cdu_clko: CDU output index controlled by the clock.
 * @parent_data: Parent data for this clock.
 * @mux_table: CDU_CFG[n] clock input mappings.
 * @num_parents: Number of parent clocks.
 * @clock_flags: clock flags.
 * @lock: Lock protecting CDU access.
 *
 * Register a clock for one CDU mux output. CDU_CFG[n]
 * controls the mux selection and output gating for CDU_CLKOn.
 *
 * The mux preserves the bootloader selected input clocks during rate changes.
 * Reparenting must be requested explicitly.
 *
 * Return: Clock specific clk_hw data on success, or an ERR_PTR() on failure.
 */
struct clk_hw *sc5xx_cdu_register(struct device *dev, const char *clock_name,
				  void __iomem *base, u8 cdu_clko, const struct clk_parent_data *parent_data,
				  const u32 *mux_table, u8 num_parents, unsigned long clock_flags,
				  spinlock_t *lock)
{
	struct sc5xx_cdu *cdu_clk;
	struct clk_init_data init = { };
	int ret;

	/* CLKO11 is used internally by the processor and is not user configurable */
	if (cdu_clko > 14 || cdu_clko == 11)
		return ERR_PTR(-EINVAL);

	cdu_clk = devm_kzalloc(dev, sizeof(*cdu_clk), GFP_KERNEL);
	if (!cdu_clk)
		return ERR_PTR(-ENOMEM);

	init.name = clock_name;
	init.ops = &sc5xx_cdu_ops;
	init.parent_data = parent_data;
	init.num_parents = num_parents;
	init.flags = clock_flags;

	cdu_clk->clk_hw.init = &init;
	cdu_clk->base = base;
	cdu_clk->lock = lock;
	cdu_clk->mux_table = mux_table;
	cdu_clk->cdu_clko = cdu_clko;

	ret = devm_clk_hw_register(dev, &cdu_clk->clk_hw);
	if (ret)
		return ERR_PTR(ret);

	return &cdu_clk->clk_hw;
}

static int sc5xx_cdu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_

	struct sc5xx_cdu_match_data cdu_info = of_device_get_match_data(dev);
	if (!cdu_info)
		return dev_err_probe(dev, -EINVAL, "missing CDU data\n");
	
	// REGISTERE CLOCKS BASED ON MATCH DATA
}

static void sc5xx_cdu_remove(struct platform_device *pdev)
{
	
}

static const struct of_device_id sc5xx_cdu_of_match[] = {
	{ .compatible = "adi,sc598-cdu", .data = &sc598_cdu_info },
	{ .compatible = "adi,sc589-cdu", .data = &sc589_cdu_info }, 
	{ .compatible = "adi,sc573-cdu", .data = &sc573_cdu_info }, 
	{ .compatible = "adi,sc594-cdu", .data = &sc594_cdu_info }, 
	{ },
};

static struct platform_driver sc5xx_cdu_driver = {
	.driver	= {
		.name = "clk-adsp-cdu",
		.of_match_table = sc5xx_cdu_of_match,
		.suppress_bind_attrs = true,
	},
	.probe  = sc5xx_cdu_probe,
	.remove = sc5xx_cdu_remove,
};

MODULE_AUTHOR("Qasim Ijaz <qasim.ijaz@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADSP SoC CDU (Clock Distribution Unit) driver");
MODULE_LICENSE("GPL v2");
