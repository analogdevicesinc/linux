#ifndef __IMX8_CLK_H
#define __IMX8_CLK_H

#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <soc/imx8/sc/sci.h>

extern spinlock_t imx_ccm_lock;
extern sc_ipc_t ccm_ipc_handle;

int imx8_clk_mu_init(void);

struct clk *imx_clk_divider_scu(const char *name,
					sc_rsrc_t rsrc_id, sc_pm_clk_t clk_type);

struct clk *imx_clk_divider2_scu(const char *name, const char *parent_name,
				sc_rsrc_t rsrc_id, sc_pm_clk_t clk_type);

struct clk *imx_clk_divider3_scu(const char *name, const char *parent_name,
			sc_rsrc_t rsrc_id, sc_ctrl_t gpr_id);

struct clk *imx_clk_divider2_scu(const char *name, const char *parent_name,
				sc_rsrc_t rsrc_id, sc_pm_clk_t clk_type);

struct clk *clk_register_gate_scu(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		u8 clk_gate_scu_flags, spinlock_t *lock,
		sc_rsrc_t rsrc_id, sc_pm_clk_t clk_type,
		void __iomem *reg, u8 bit_idx, bool hw_gate);

struct clk *clk_register_gate2_scu(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg, u8 bit_idx,
		u8 clk_gate_flags, spinlock_t *lock, const char *pd_name);

struct clk *clk_register_mux_scu(struct device *dev, const char *name,
		const char **parent_names, u8 num_parents, unsigned long flags,
		void __iomem *reg, u8 shift, u8 width,
		u8 clk_mux_flags, spinlock_t *lock,
		const char *pd_name);

struct clk *clk_register_gate3_scu(struct device *dev, const char *name,
		const char *parent_name, spinlock_t *lock,
		sc_rsrc_t rsrc_id, sc_ctrl_t gpr_id, bool invert_flag);

struct clk *clk_register_mux_gpr_scu(struct device *dev, const char *name,
		const char **parents, int num_parents, spinlock_t *lock,
		sc_rsrc_t rsrc_id, sc_ctrl_t gpr_id);

static inline struct clk *imx_clk_fixed(const char *name, int rate)
{
	return clk_register_fixed_rate(NULL, name, NULL, 0, rate);
}

static inline struct clk *imx_clk_gate_scu(const char *name, const char *parent,
		sc_rsrc_t rsrc_id, sc_pm_clk_t clk_type,
		void __iomem *reg, u8 bit_idx, bool hw_gate)
{
	return clk_register_gate_scu(NULL, name, parent,
			CLK_SET_RATE_PARENT, 0, &imx_ccm_lock, rsrc_id, clk_type,
			reg, bit_idx, hw_gate);
}

static inline struct clk *imx_clk_gate2_scu(const char *name, const char *parent,
		void __iomem *reg, u8 bit_idx, const char *pd_name)
{
	return clk_register_gate2_scu(NULL, name, parent, 0, reg,
			bit_idx, 0, &imx_ccm_lock, pd_name);
}

static inline struct clk *imx_clk_gate3_scu(const char *name, const char *parent,
		sc_rsrc_t rsrc_id, sc_ctrl_t gpr_id, bool invert_flag)
{
	return clk_register_gate3_scu(NULL, name, parent,
			 &imx_ccm_lock, rsrc_id, gpr_id, invert_flag);
}

static inline struct clk *imx_clk_mux_gpr_scu(const char *name, const char **parents,
			int num_parents, sc_rsrc_t rsrc_id, sc_ctrl_t gpr_id)
{
	return clk_register_mux_gpr_scu(NULL, name, parents, num_parents,
			 &imx_ccm_lock, rsrc_id, gpr_id);
}

static inline void imx_clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = clk_set_rate(clk, rate);

	if (ret)
		pr_err("failed to set rate of clk %s to %ld: %d\n",
			__clk_get_name(clk), rate, ret);
}

static inline struct clk *imx_clk_gate(const char *name, const char *parent,
		void __iomem *reg, u8 shift)
{
	return clk_register_gate(NULL, name, parent, CLK_SET_RATE_PARENT, reg,
			shift, 0, &imx_ccm_lock);
}

static inline struct clk *imx_clk_fixed_factor(const char *name,
		const char *parent, unsigned int mult, unsigned int div)
{
	return clk_register_fixed_factor(NULL, name, parent,
			CLK_SET_RATE_PARENT, mult, div);
}

static inline struct clk *imx_clk_mux_scu(const char *name, void __iomem *reg,
		u8 shift, u8 width, const char **parents, int num_parents, const char *pd_name)
{
	return clk_register_mux_scu(NULL, name, parents, num_parents,
			CLK_SET_RATE_NO_REPARENT, reg, shift,
			width, 0, &imx_ccm_lock, pd_name);
}


#endif
