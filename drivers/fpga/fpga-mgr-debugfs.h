/*
 * FPGA Manager DebugFS
 *
 *  Copyright (C) 2016 Intel Corporation.  All rights reserved.
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
#ifndef _LINUX_FPGA_MGR_DEBUGFS_H
#define _LINUX_FPGA_MGR_DEBUGFS_H

#if IS_ENABLED(CONFIG_FPGA_MGR_DEBUG_FS)

void fpga_mgr_debugfs_add(struct fpga_manager *mgr);
void fpga_mgr_debugfs_remove(struct fpga_manager *mgr);
void fpga_mgr_debugfs_init(void);
void fpga_mgr_debugfs_uninit(void);

#else

void fpga_mgr_debugfs_add(struct fpga_manager *mgr) {}
void fpga_mgr_debugfs_remove(struct fpga_manager *mgr) {}
void fpga_mgr_debugfs_init(void) {}
void fpga_mgr_debugfs_uninit(void) {}

#endif /* CONFIG_FPGA_MGR_DEBUG_FS */

#endif /*_LINUX_FPGA_MGR_DEBUGFS_H */
