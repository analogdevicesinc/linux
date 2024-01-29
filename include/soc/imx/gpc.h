#ifndef __SOC_IMX_GPC_H
#define __SOC_IMX_GPC_H

void imx_gpc_hold_m4_in_sleep(void);
void imx_gpc_release_m4_in_sleep(void);
void imx6sx_set_m4_highfreq(bool high_freq);

#endif /* __SOC_IMX_GPC_H */
