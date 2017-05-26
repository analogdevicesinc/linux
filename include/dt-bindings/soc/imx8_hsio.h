/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __DT_BINDINGS_IMX8_HSIO_H
#define __DT_BINDINGS_IMX8_HSIO_H

/*
 * imx8qm hsio has pciea, pcieb and sata modules, and hsio
 * can be configured to the following different work modes.
 * 1 - pciea 2 lanes and one sata ahci port.
 * 2 - pciea 1 lane, pcieb 1 lane and one sata ahci port.
 * 3 - pciea 2 lanes, pcieb 1 lane.
 * Choose one mode, refer to the exact hardware board design.
 */
#define		PCIEAX2SATA		1
#define		PCIEAX1PCIEBX1SATA	2
#define		PCIEAX2PCIEBX1		3

#endif /* __DT_BINDINGS_IMX8_HSIO_H */

