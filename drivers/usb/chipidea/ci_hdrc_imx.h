/*
 * Copyright 2012-2015 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __DRIVER_USB_CHIPIDEA_CI_HDRC_IMX_H
#define __DRIVER_USB_CHIPIDEA_CI_HDRC_IMX_H
#include <linux/usb/otg.h>
#include <linux/usb/phy.h>

struct imx_usbmisc_data {
	struct device *dev;
	int index;
	struct regmap *anatop;
	struct usb_phy *usb_phy;

	unsigned int disable_oc:1; /* over current detect disabled */
	unsigned int oc_polarity:1; /* over current polarity if oc enabled */
	unsigned int pwr_polarity:1; /* polarity of enable vbus from pmic */
	unsigned int evdo:1; /* set external vbus divider option */
	unsigned int ulpi:1; /* connected to an ULPI phy */
	unsigned int hsic:1; /* HSIC controlller */
	/*
	 * Specifies the delay between powering up the xtal 24MHz clock
	 * and release the clock to the digital logic inside the analog block
	 */
	unsigned int osc_clkgate_delay;
	enum usb_dr_mode available_role;
	int emp_curr_control;
	int dc_vol_level_adjust;
};

int imx_usbmisc_init(struct imx_usbmisc_data *);
int imx_usbmisc_init_post(struct imx_usbmisc_data *);
int imx_usbmisc_set_wakeup(struct imx_usbmisc_data *, bool);
int imx_usbmisc_charger_detection(struct imx_usbmisc_data *data, bool connect);
int imx_usbmisc_power_lost_check(struct imx_usbmisc_data *);
int imx_usbmisc_hsic_set_connect(struct imx_usbmisc_data *);
int imx_usbmisc_hsic_set_clk(struct imx_usbmisc_data *, bool);
int imx_usbmisc_term_select_override(struct imx_usbmisc_data *data,
						bool enable, int val);

#endif /* __DRIVER_USB_CHIPIDEA_CI_HDRC_IMX_H */
