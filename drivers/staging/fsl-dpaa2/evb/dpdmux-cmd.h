/* Copyright 2013-2016 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the above-listed copyright holders nor the
 * names of any contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _FSL_DPDMUX_CMD_H
#define _FSL_DPDMUX_CMD_H

/* DPDMUX Version */
#define DPDMUX_VER_MAJOR		6
#define DPDMUX_VER_MINOR		1

#define DPDMUX_CMD_BASE_VER		1
#define DPDMUX_CMD_ID_OFFSET		4

#define DPDMUX_CMD(id)	(((id) << DPDMUX_CMD_ID_OFFSET) | DPDMUX_CMD_BASE_VER)

/* Command IDs */
#define DPDMUX_CMDID_CLOSE			DPDMUX_CMD(0x800)
#define DPDMUX_CMDID_OPEN			DPDMUX_CMD(0x806)
#define DPDMUX_CMDID_CREATE			DPDMUX_CMD(0x906)
#define DPDMUX_CMDID_DESTROY			DPDMUX_CMD(0x986)
#define DPDMUX_CMDID_GET_API_VERSION		DPDMUX_CMD(0xa06)

#define DPDMUX_CMDID_ENABLE			DPDMUX_CMD(0x002)
#define DPDMUX_CMDID_DISABLE			DPDMUX_CMD(0x003)
#define DPDMUX_CMDID_GET_ATTR			DPDMUX_CMD(0x004)
#define DPDMUX_CMDID_RESET			DPDMUX_CMD(0x005)
#define DPDMUX_CMDID_IS_ENABLED			DPDMUX_CMD(0x006)

#define DPDMUX_CMDID_SET_IRQ_ENABLE		DPDMUX_CMD(0x012)
#define DPDMUX_CMDID_GET_IRQ_ENABLE		DPDMUX_CMD(0x013)
#define DPDMUX_CMDID_SET_IRQ_MASK		DPDMUX_CMD(0x014)
#define DPDMUX_CMDID_GET_IRQ_MASK		DPDMUX_CMD(0x015)
#define DPDMUX_CMDID_GET_IRQ_STATUS		DPDMUX_CMD(0x016)
#define DPDMUX_CMDID_CLEAR_IRQ_STATUS		DPDMUX_CMD(0x017)

#define DPDMUX_CMDID_SET_MAX_FRAME_LENGTH	DPDMUX_CMD(0x0a1)

#define DPDMUX_CMDID_UL_RESET_COUNTERS		DPDMUX_CMD(0x0a3)

#define DPDMUX_CMDID_IF_SET_ACCEPTED_FRAMES	DPDMUX_CMD(0x0a7)
#define DPDMUX_CMDID_IF_GET_ATTR		DPDMUX_CMD(0x0a8)
#define DPDMUX_CMDID_IF_ENABLE			DPDMUX_CMD(0x0a9)
#define DPDMUX_CMDID_IF_DISABLE			DPDMUX_CMD(0x0aa)

#define DPDMUX_CMDID_IF_ADD_L2_RULE		DPDMUX_CMD(0x0b0)
#define DPDMUX_CMDID_IF_REMOVE_L2_RULE		DPDMUX_CMD(0x0b1)
#define DPDMUX_CMDID_IF_GET_COUNTER		DPDMUX_CMD(0x0b2)
#define DPDMUX_CMDID_IF_SET_LINK_CFG		DPDMUX_CMD(0x0b3)
#define DPDMUX_CMDID_IF_GET_LINK_STATE		DPDMUX_CMD(0x0b4)

#define DPDMUX_CMDID_SET_CUSTOM_KEY		DPDMUX_CMD(0x0b5)
#define DPDMUX_CMDID_ADD_CUSTOM_CLS_ENTRY	DPDMUX_CMD(0x0b6)
#define DPDMUX_CMDID_REMOVE_CUSTOM_CLS_ENTRY	DPDMUX_CMD(0x0b7)

#define DPDMUX_MASK(field)        \
	GENMASK(DPDMUX_##field##_SHIFT + DPDMUX_##field##_SIZE - 1, \
		DPDMUX_##field##_SHIFT)
#define dpdmux_set_field(var, field, val) \
	((var) |= (((val) << DPDMUX_##field##_SHIFT) & DPDMUX_MASK(field)))
#define dpdmux_get_field(var, field)      \
	(((var) & DPDMUX_MASK(field)) >> DPDMUX_##field##_SHIFT)

struct dpdmux_cmd_open {
	u32 dpdmux_id;
};

struct dpdmux_cmd_create {
	u8 method;
	u8 manip;
	u16 num_ifs;
	u32 pad;

	u16 adv_max_dmat_entries;
	u16 adv_max_mc_groups;
	u16 adv_max_vlan_ids;
	u16 pad1;

	u64 options;
};

struct dpdmux_cmd_destroy {
	u32 dpdmux_id;
};

#define DPDMUX_ENABLE_SHIFT	0
#define DPDMUX_ENABLE_SIZE	1

struct dpdmux_rsp_is_enabled {
	u8 en;
};

struct dpdmux_cmd_set_irq_enable {
	u8 enable;
	u8 pad[3];
	u8 irq_index;
};

struct dpdmux_cmd_get_irq_enable {
	u32 pad;
	u8 irq_index;
};

struct dpdmux_rsp_get_irq_enable {
	u8 enable;
};

struct dpdmux_cmd_set_irq_mask {
	u32 mask;
	u8 irq_index;
};

struct dpdmux_cmd_get_irq_mask {
	u32 pad;
	u8 irq_index;
};

struct dpdmux_rsp_get_irq_mask {
	u32 mask;
};

struct dpdmux_cmd_get_irq_status {
	u32 status;
	u8 irq_index;
};

struct dpdmux_rsp_get_irq_status {
	u32 status;
};

struct dpdmux_cmd_clear_irq_status {
	u32 status;
	u8 irq_index;
};

struct dpdmux_rsp_get_attr {
	u8 method;
	u8 manip;
	u16 num_ifs;
	u16 mem_size;
	u16 pad;

	u64 pad1;

	u32 id;
	u32 pad2;

	u64 options;
};

struct dpdmux_cmd_set_max_frame_length {
	u16 max_frame_length;
};

#define DPDMUX_ACCEPTED_FRAMES_TYPE_SHIFT	0
#define DPDMUX_ACCEPTED_FRAMES_TYPE_SIZE	4
#define DPDMUX_UNACCEPTED_FRAMES_ACTION_SHIFT	4
#define DPDMUX_UNACCEPTED_FRAMES_ACTION_SIZE	4

struct dpdmux_cmd_if_set_accepted_frames {
	u16 if_id;
	u8 frames_options;
};

struct dpdmux_cmd_if {
	u16 if_id;
};

struct dpdmux_rsp_if_get_attr {
	u8 pad[3];
	u8 enabled;
	u8 pad1[3];
	u8 accepted_frames_type;
	u32 rate;
};

struct dpdmux_cmd_if_l2_rule {
	u16 if_id;
	u8 mac_addr5;
	u8 mac_addr4;
	u8 mac_addr3;
	u8 mac_addr2;
	u8 mac_addr1;
	u8 mac_addr0;

	u32 pad;
	u16 vlan_id;
};

struct dpdmux_cmd_if_get_counter {
	u16 if_id;
	u8 counter_type;
};

struct dpdmux_rsp_if_get_counter {
	u64 pad;
	u64 counter;
};

struct dpdmux_cmd_if_set_link_cfg {
	u16 if_id;
	u16 pad[3];

	u32 rate;
	u32 pad1;

	u64 options;
};

struct dpdmux_cmd_if_get_link_state {
	u16 if_id;
};

struct dpdmux_rsp_if_get_link_state {
	u32 pad;
	u8 up;
	u8 pad1[3];

	u32 rate;
	u32 pad2;

	u64 options;
};

struct dpdmux_rsp_get_api_version {
	u16 major;
	u16 minor;
};

struct dpdmux_set_custom_key {
	u64 pad[6];
	u64 key_cfg_iova;
};

struct dpdmux_cmd_add_custom_cls_entry {
	u8 pad[3];
	u8 key_size;
	u16 pad1;
	u16 dest_if;
	u64 key_iova;
	u64 mask_iova;
};

struct dpdmux_cmd_remove_custom_cls_entry {
	u8 pad[3];
	u8 key_size;
	u32 pad1;
	u64 key_iova;
	u64 mask_iova;
};

#endif /* _FSL_DPDMUX_CMD_H */
