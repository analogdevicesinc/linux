/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _PFE_CDEV_H_
#define _PFE_CDEV_H_

#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/poll.h>

#define  PFE_CDEV_NAME "pfe_us_cdev"
#define  PFE_CLASS_NAME  "ppfe_us"

/* Extracted from ls1012a_pfe_platform_data, there are 3 interfaces which are
 * supported by PFE driver. Should be updated if number of eth devices are
 * changed.
 */
#define PFE_CDEV_ETH_COUNT 3

struct pfe_shared_info {
	uint32_t phy_id; /* Link phy ID */
	uint8_t state;  /* Has either 0 or 1 */
};

extern struct pfe_shared_info link_states[PFE_CDEV_ETH_COUNT];

/* IOCTL Commands */
#define PFE_CDEV_ETH0_STATE_GET		_IOR('R', 0, int)
#define PFE_CDEV_ETH1_STATE_GET		_IOR('R', 1, int)
#define PFE_CDEV_HIF_INTR_EN		_IOWR('R', 2, int)

int pfe_cdev_init(void);
void pfe_cdev_exit(void);

#endif /* _PFE_CDEV_H_ */
