/*
 * gadget-export.h - Gadget Export APIs
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

#ifndef __CDNS3_GADGET_EXPORT_H
#define __CDNS3_GADGET_EXPORT_H

#ifdef CONFIG_USB_CDNS3_GADGET

int cdns3_gadget_init(struct cdns3 *cdns);
void cdns3_gadget_remove(struct cdns3 *cdns);
#else

static inline int cdns3_gadget_init(struct cdns3 *cdns)
{
	return -ENXIO;
}

static inline void cdns3_gadget_remove(struct cdns3 *cdns)
{

}

#endif

#endif /* __CDNS3_GADGET_EXPORT_H */
