/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MACH_IMX_CLK_H
#define __MACH_IMX_CLK_H

#include <linux/spinlock.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <soc/imx/src.h>

extern spinlock_t imx_ccm_lock;

void imx_check_clocks(struct clk *clks[], unsigned int count);
void imx_register_uart_clocks(struct clk ** const clks[]);

extern void imx_cscmr1_fixup(u32 *val);
extern struct imx_sema4_mutex *amp_power_mutex;
extern struct imx_shared_mem *shared_mem;
extern bool uart_from_osc;
extern const struct clk_ops clk_frac_divider_ops;

enum imx_pllv1_type {
	IMX_PLLV1_IMX1,
	IMX_PLLV1_IMX21,
	IMX_PLLV1_IMX25,
	IMX_PLLV1_IMX27,
	IMX_PLLV1_IMX31,
	IMX_PLLV1_IMX35,
};

enum imx_int_pll_type {
	PLL_1416X,
	PLL_1443X,
};

enum imx_sccg_pll_type {
	SCCG_PLL1,
	SCCG_PLL2,
};

/* NOTE: Rate table should be kept sorted in descending order. */
struct imx_int_pll_rate_table {
	unsigned int rate;
	unsigned int pdiv;
	unsigned int mdiv;
	unsigned int sdiv;
	unsigned int kdiv;
};

struct imx_int_pll_clk {
	enum imx_int_pll_type type;
	const struct imx_int_pll_rate_table *rate_table;
	int flags;
};

struct clk *imx_clk_int_pll(const char *name, const char *parent_name, void __iomem *base, const struct imx_int_pll_clk *pll_clk);

struct clk *imx_clk_pllv1(enum imx_pllv1_type type, const char *name,
		const char *parent, void __iomem *base);

struct clk *imx_clk_pllv2(const char *name, const char *parent,
		void __iomem *base);

struct clk *imx_clk_frac_pll(const char *name, const char *parent_name, void __iomem *base);

struct clk *imx_clk_sccg_pll(const char *name, const char *parent_name, void __iomem *base, enum imx_sccg_pll_type pll_type);

enum imx_pllv3_type {
	IMX_PLLV3_GENERIC,
	IMX_PLLV3_SYS,
	IMX_PLLV3_USB,
	IMX_PLLV3_USB_VF610,
	IMX_PLLV3_AV,
	IMX_PLLV3_ENET,
	IMX_PLLV3_ENET_IMX7,
	IMX_PLLV3_SYS_VF610,
	IMX_PLLV3_DDR_IMX7,
	IMX_PLLV3_AV_IMX7,
	IMX_PLLV3_PLL2,
};

/*
 * frac_divider, found on i.MX7ULP PCC module.
 * the output clock of the fractional divider is:
 * Divider output clock = Input clock * (FRAC + 1)
 * / (DIV + 1)
 */
struct clk_frac_divider {
	struct clk_hw	hw;
	void __iomem	*reg;
	u8		mshift;
	u8		mwidth;
	u32		mmask;
	u8		nshift;
	u8		nwidth;
	u32		nmask;
};

#define MAX_SHARED_CLK_NUMBER		100
#define SHARED_MEM_MAGIC_NUMBER		0x12345678
#define MCC_POWER_SHMEM_NUMBER		(6)

struct imx_shared_clk {
	struct clk *self;
	struct clk *parent;
	void *m4_clk;
	void *m4_clk_parent;
	u8 ca9_enabled;
	u8 cm4_enabled;
};

struct imx_shared_mem {
	u32 ca9_valid;
	u32 cm4_valid;
	struct imx_shared_clk imx_clk[MAX_SHARED_CLK_NUMBER];
};

struct clk *imx_clk_pllv3(enum imx_pllv3_type type, const char *name,
		const char *parent_name, void __iomem *base, u32 div_mask);

struct clk *imx_clk_pllv4(const char *name,
			  const char *parent_name, void __iomem *base);
struct clk *imx_clk_pllv5(const char *name, const char *parent_name,
			  void __iomem *base);

struct clk *clk_register_gate2(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg, u8 bit_idx, u8 cgr_val,
		u8 clk_gate_flags, spinlock_t *lock,
		unsigned int *share_count);

struct clk * imx_obtain_fixed_clock(
			const char *name, unsigned long rate);

struct clk *imx_clk_gate_exclusive(const char *name, const char *parent,
	 void __iomem *reg, u8 shift, u32 exclusive_mask);

static inline void imx_clk_prepare_enable(struct clk *clk)
{
	int ret = clk_prepare_enable(clk);

	if (ret)
		pr_err("failed to prepare and enable clk %s: %d\n",
			__clk_get_name(clk), ret);
}

static inline void imx_clk_set_parent(struct clk *clk, struct clk *parent)
{
	int ret = clk_set_parent(clk, parent);

	if (ret)
		pr_err("failed to set parent of clk %s to %s: %d\n",
			__clk_get_name(clk), __clk_get_name(parent), ret);
}

static inline void imx_clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = clk_set_rate(clk, rate);

	if (ret)
		pr_err("failed to set rate of clk %s to %ld: %d\n",
			__clk_get_name(clk), rate, ret);
}

struct clk *imx_clk_pfd(const char *name, const char *parent_name,
		void __iomem *reg, u8 idx);

struct clk *imx_clk_busy_divider(const char *name, const char *parent_name,
				 void __iomem *reg, u8 shift, u8 width,
				 void __iomem *busy_reg, u8 busy_shift);

struct clk *imx_clk_busy_mux(const char *name, void __iomem *reg, u8 shift,
			     u8 width, void __iomem *busy_reg, u8 busy_shift,
			     const char **parent_names, int num_parents);

struct clk *imx_clk_busy_gate(const char *name, const char *parent,
			    void __iomem *reg, u8 shift);

struct clk *imx_clk_fixup_divider(const char *name, const char *parent,
				  void __iomem *reg, u8 shift, u8 width,
				  void (*fixup)(u32 *val));

struct clk *imx_clk_fixup_mux(const char *name, void __iomem *reg,
			      u8 shift, u8 width, const char **parents,
			      int num_parents, void (*fixup)(u32 *val));

static inline struct clk *imx_clk_fixed(const char *name, int rate)
{
	return clk_register_fixed_rate(NULL, name, NULL, 0, rate);
}

static inline struct clk *imx_clk_mux_ldb(const char *name, void __iomem *reg,
		u8 shift, u8 width, const char **parents, int num_parents)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT | CLK_SET_RATE_PARENT, reg,
			shift, width, CLK_MUX_READ_ONLY, &imx_ccm_lock);
}

static inline struct clk *imx_clk_fixed_factor(const char *name,
		const char *parent, unsigned int mult, unsigned int div)
{
	return clk_register_fixed_factor(NULL, name, parent,
			CLK_SET_RATE_PARENT, mult, div);
}

static inline struct clk *imx_clk_divider(const char *name, const char *parent,
		void __iomem *reg, u8 shift, u8 width)
{
	return clk_register_divider(NULL, name, parent,
			CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE,
			reg, shift, width, 0, &imx_ccm_lock);
}

static inline struct clk *imx_clk_divider2(const char *name, const char *parent,
		void __iomem *reg, u8 shift, u8 width)
{
	return clk_register_divider(NULL, name, parent,
		CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE | CLK_OPS_PARENT_ENABLE,
			reg, shift, width, CLK_DIVIDER_ROUND_CLOSEST, &imx_ccm_lock);
}

static inline struct clk *imx_clk_divider_flags(const char *name,
		const char *parent, void __iomem *reg, u8 shift, u8 width,
		unsigned long flags)
{
	return clk_register_divider(NULL, name, parent, flags,
			reg, shift, width, 0, &imx_ccm_lock);
}

static inline struct clk *imx_clk_gate(const char *name, const char *parent,
		void __iomem *reg, u8 shift)
{
	return clk_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
			shift, 0, &imx_ccm_lock);
}

static inline struct clk *imx_clk_gate_dis(const char *name, const char *parent,
		void __iomem *reg, u8 shift)
{
	return clk_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
			shift, CLK_GATE_SET_TO_DISABLE, &imx_ccm_lock);
}

static inline struct clk *imx_clk_gate2(const char *name, const char *parent,
		void __iomem *reg, u8 shift)
{
	return clk_register_gate2(NULL, name, parent,
			CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE, reg,
			shift, 0x3, 0, &imx_ccm_lock, NULL);
}

static inline struct clk *imx_clk_gate2_flags(const char *name, const char *parent,
		void __iomem *reg, u8 shift, unsigned long flags)
{
	return clk_register_gate2(NULL, name, parent, flags, reg,
			shift, 0x3, 0, &imx_ccm_lock, NULL);
}

static inline struct clk *imx_clk_gate2_shared(const char *name,
		const char *parent, void __iomem *reg, u8 shift,
		unsigned int *share_count)
{
	return clk_register_gate2(NULL, name, parent,
			CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE, reg,
			shift, 0x3, 0, &imx_ccm_lock, share_count);
}

static inline struct clk *imx_clk_gate2_shared2(const char *name,
		const char *parent, void __iomem *reg, u8 shift,
		unsigned int *share_count)
{
	return clk_register_gate2(NULL, name, parent, CLK_SET_RATE_PARENT |
			CLK_SET_RATE_GATE | CLK_OPS_PARENT_ENABLE,
			reg, shift, 0x3, 0, &imx_ccm_lock, share_count);
}

static inline struct clk *imx_clk_gate2_cgr(const char *name,
		const char *parent, void __iomem *reg, u8 shift, u8 cgr_val)
{
	return clk_register_gate2(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
			shift, cgr_val, 0, &imx_ccm_lock, NULL);
}

static inline struct clk *imx_clk_gate3(const char *name, const char *parent,
		void __iomem *reg, u8 shift)
{
	/*
	 * per design team's suggestion, clk root is NOT consuming
	 * much power, and clk root enable/disable does NOT have domain
	 * control, so they suggest to leave clk root always on when
	 * M4 is enabled.
	 */
	if (imx_src_is_m4_enabled())
		return clk_register_fixed_factor(NULL, name, parent,
						 CLK_SET_RATE_PARENT, 1, 1);
	else
		return clk_register_gate(NULL, name, parent,
			CLK_SET_RATE_PARENT | CLK_OPS_PARENT_ENABLE,
			reg, shift, 0, &imx_ccm_lock);
}

static inline struct clk *imx_clk_gate4(const char *name, const char *parent,
		void __iomem *reg, u8 shift)
{
	return clk_register_gate2(NULL, name, parent,
		CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE | CLK_OPS_PARENT_ENABLE,
			reg, shift, 0x3, 0, &imx_ccm_lock, NULL);
}

static inline struct clk *imx_clk_mux_bus(const char *name, void __iomem *reg,
		u8 shift, u8 width, const char **parents, int num_parents)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT,
			reg, shift, width, 0, &imx_ccm_lock);
}

static inline struct clk *imx_clk_mux(const char *name, void __iomem *reg,
		u8 shift, u8 width, const char **parents, int num_parents)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT | CLK_SET_PARENT_GATE,
			reg, shift, width, 0, &imx_ccm_lock);
}

static inline struct clk *imx_clk_mux2(const char *name, void __iomem *reg,
		u8 shift, u8 width, const char **parents, int num_parents)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT | CLK_OPS_PARENT_ENABLE,
			reg, shift, width, 0, &imx_ccm_lock);
}

static inline struct clk *imx_clk_mux_flags(const char *name,
		void __iomem *reg, u8 shift, u8 width, const char **parents,
		int num_parents, unsigned long flags)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			flags | CLK_SET_RATE_NO_REPARENT | CLK_SET_PARENT_GATE,
			reg, shift, width, 0, &imx_ccm_lock);
}

static inline struct clk *imx_clk_mux_flags_bus(const char *name,
		void __iomem *reg, u8 shift, u8 width, const char **paretns,
		int num_parents, unsigned long flags)
{
	return clk_register_mux(NULL, name, paretns, num_parents,
			flags | CLK_SET_RATE_NO_REPARENT, reg, shift,
			width, 0, &imx_ccm_lock);
}

static inline struct clk *imx_clk_mux_glitchless(const char *name,
		void __iomem *reg, u8 shift, u8 width, const char **parents,
		int num_parents)
{
	return clk_register_mux(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT, reg, shift,
			width, 0, &imx_ccm_lock);
}

struct clk *imx_clk_cpu(const char *name, const char *parent_name,
		struct clk *div, struct clk *mux, struct clk *pll,
		struct clk *step);

int imx_update_shared_mem(struct clk_hw *hw, bool enable);

static inline int clk_on_imx6sx(void)
{
	return of_machine_is_compatible("fsl,imx6sx");
}

struct clk *imx7ulp_clk_composite(const char *name, const char **parent_name,
			      int num_parents, bool mux_present, bool rate_present,
			      bool gate_present, void __iomem *reg);

struct clk *imx_clk_pfdv2(const char *name, const char *parent_name,
			  void __iomem *reg, u8 idx);

struct clk *imx8m_clk_composite_flags(const char *name, const char **parent_names,
		int num_parents, void __iomem *reg, unsigned long flags);

#define __imx8m_clk_composite(name, parent_names, reg, flags) \
	imx8m_clk_composite_flags(name, parent_names, \
		ARRAY_SIZE(parent_names), reg, \
		flags | CLK_SET_RATE_NO_REPARENT | CLK_OPS_PARENT_ENABLE)

#define imx8m_clk_composite(name, parent_names, reg) \
	__imx8m_clk_composite(name, parent_names, reg, 0)

#define imx8m_clk_composite_critical(name, parent_names, reg) \
	__imx8m_clk_composite(name, parent_names, reg, CLK_IS_CRITICAL)

#endif
