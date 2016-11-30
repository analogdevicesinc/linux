/*
 * FPGA Freeze Bridge Controller
 *
 * Copyright (C) 2016 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _ALT_FRZ_BR_H
#define _ALT_FRZ_BR_H
#include <linux/io.h>

int altera_freeze_br_probe(struct device *dev, void __iomem *reg_base);
int altera_freeze_br_remove(struct device *dev);

#endif /* _ALT_FRZ_BR_H */
