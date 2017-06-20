#ifndef __SOC_IMX_SRC_H
#define __SOC_IMX_SRC_H

#ifndef CONFIG_ARM64
bool imx_src_is_m4_enabled(void);
#else
static inline bool imx_src_is_m4_enabled(void)
{
	return 0;
}
#endif

#endif /* __SOC_IMX_SRC_H */
