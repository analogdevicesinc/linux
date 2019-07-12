#ifndef CLK_SCLAE_H_
#define CLK_SCLAE_H_

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/math64.h>


struct clock_scale {
	u32 mult;
	u32 div;
};

static inline int of_clk_get_scale(struct device_node *np, const char *name, struct clock_scale *scale)
{
	char property[80];
	u32 vals[2];
	int ret;

	scale->mult = 1;
	scale->div = 1;

	ret = snprintf(property, 80,  "%s%sclock-scales", name ? name : "", name ? "-" : "");
	if (ret < 0)
		return -EINVAL;

	ret = of_property_read_u32_array(np, property, vals, 2);
	if (!ret)  {
		scale->mult = vals[0];
		scale->div = vals[1];
	}

	return ret;
}

static inline unsigned long long clk_get_rate_scaled(struct clk *clk, struct clock_scale *scale)
{
	return div_u64((u64)clk_get_rate(clk) * scale->div, scale->mult);

}

static inline unsigned long long clk_set_rate_scaled(struct clk *clk, unsigned long long rate, struct clock_scale *scale)
{
	return clk_set_rate(clk, div_u64(rate * scale->mult, scale->div));
}

static inline unsigned long to_ccf_scaled(unsigned long long rate, struct clock_scale *scale)
{

	return div_u64(rate * scale->mult, scale->div);
}

static inline unsigned long long from_ccf_scaled(unsigned long long rate, struct clock_scale *scale)
{
	return div_u64(rate * scale->div, scale->mult);
}

#endif /* CLK_SCLAE_H_ */
