/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2023 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#ifndef _MALI_HW_ACCESS_H_
#define _MALI_HW_ACCESS_H_

#include <asm/arch_timer.h>
#include <linux/io.h>


#define mali_readl(addr) readl(addr)
#define mali_readq(addr) readq(addr)
#define mali_writel(val, addr) writel(val, addr)
#define mali_writeq(val, addr) writeq(val, addr)
#define mali_ioremap(addr, size) ioremap(addr, size)
#define mali_iounmap(addr) iounmap(addr)
#define mali_arch_timer_get_cntfrq() arch_timer_get_cntfrq()


#endif /* _MALI_HW_ACCESS_H_ */
