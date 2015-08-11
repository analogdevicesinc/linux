/*
 * Copyright (C) 2015 Altera Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms and conditions of the GNU General Public License,
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

#ifndef MACH_SOCFPGA_NAND_H
#define MACH_SOCFPGA_NAND_H

#ifdef CONFIG_EDAC_ALTERA_NAND_ECC
void socfpga_init_nand_ecc(void);
#else
inline void socfpga_init_nand_ecc(void)
{
}
#endif

#endif /* MACH_SOCFPGA_NAND_H */
