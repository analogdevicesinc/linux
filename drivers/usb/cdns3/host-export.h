/*
 * host-export.h - Host Export APIs
 *
 * Copyright 2017 NXP
 * Authors: Peter Chen <peter.chen@nxp.com>
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __DRIVERS_USB_CDNS3_HOST_H
#define __DRIVERS_USB_CDNS3_HOST_H

#ifdef CONFIG_USB_CDNS3_HOST

int cdns3_host_init(struct cdns3 *cdns);
void cdns3_host_remove(struct cdns3 *cdns);
void cdns3_host_driver_init(void);

#else

static inline int cdns3_host_init(struct cdns3 *cdns)
{
	return -ENXIO;
}

static inline void cdns3_host_remove(struct cdns3 *cdns)
{

}

static inline void cdns3_host_driver_init(void)
{

}

#endif /* CONFIG_USB_CDNS3_HOST */

#endif /* __DRIVERS_USB_CDNS3_HOST_H */
