/* Copyright (c) 2012 Freescale Semiconductor, Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FM_RESS_TEST_H_
#define FM_RESS_TEST_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>

#define _Packed
#define _PackedType __attribute__ ((packed))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define KERN_ALERT ""
#define KERN_INFO ""
#define ASSERT_COND assert
#define printk printf
#define NET_IP_ALIGN 0
#define FM_FIFO_ALLOCATION_OLD_ALG

#if defined(CONFIG_FMAN_DISABLE_OH_AND_DISTRIBUTE_RESOURCES)
#define FM_10G_OPENDMA_MIN_TRESHOLD 8 /* 10g minimum treshold if only HC is enabled and no OH port enabled */
#define FM_OPENDMA_RX_TX_RAPORT 2 /* RX = 2*TX */
#else
#define FM_10G_OPENDMA_MIN_TRESHOLD 7 /* 10g minimum treshold if 7 OH ports are enabled */
#define FM_OPENDMA_RX_TX_RAPORT 1 /* RX = TX */
#endif
#define FM_DEFAULT_TX10G_OPENDMA 8 /* default TX 10g open dmas */
#define FM_DEFAULT_RX10G_OPENDMA 8 /* default RX 10g open dmas */

/* information about all active ports for an FMan.
 * !Some ports may be disabled by u-boot, thus will not be available */
struct fm_active_ports {
    uint32_t num_oh_ports;
    uint32_t num_tx_ports;
    uint32_t num_rx_ports;
    uint32_t num_tx25_ports;
    uint32_t num_rx25_ports;
    uint32_t num_tx10_ports;
    uint32_t num_rx10_ports;
};

/* FMan resources precalculated at fm probe based
 * on available FMan port. */
struct fm_resource_settings {
    /* buffers - fifo sizes */
    uint32_t tx1g_num_buffers;
    uint32_t rx1g_num_buffers;
    uint32_t tx2g5_num_buffers; /* Not supported yet by LLD */
    uint32_t rx2g5_num_buffers; /* Not supported yet by LLD */
    uint32_t tx10g_num_buffers;
    uint32_t rx10g_num_buffers;
    uint32_t oh_num_buffers;
    uint32_t shared_ext_buffers;


    /* open DMAs */
    uint32_t tx_1g_dmas;
    uint32_t rx_1g_dmas;
    uint32_t tx_2g5_dmas; /* Not supported yet by LLD */
    uint32_t rx_2g5_dmas; /* Not supported yet by LLD */
    uint32_t tx_10g_dmas;
    uint32_t rx_10g_dmas;
    uint32_t oh_dmas;
    uint32_t shared_ext_open_dma;

    /* Tnums */
    uint32_t tx_1g_tnums;
    uint32_t rx_1g_tnums;
    uint32_t tx_2g5_tnums; /* Not supported yet by LLD */
    uint32_t rx_2g5_tnums; /* Not supported yet by LLD */
    uint32_t tx_10g_tnums;
    uint32_t rx_10g_tnums;
    uint32_t oh_tnums;
    uint32_t shared_ext_tnums;
};

typedef struct {
	uint8_t                     id;
    struct fm_active_ports      fm_active_ports_info;
    struct fm_resource_settings fm_resource_settings_info;
} t_LnxWrpFmDev;

typedef struct {
	uint8_t                     id;
} t_LnxWrpFmPortDev;

typedef _Packed struct t_FmPrsResult {
	volatile uint8_t     lpid;               /**< Logical port id */
	volatile uint8_t     shimr;              /**< Shim header result  */
	volatile uint16_t    l2r;                /**< Layer 2 result */
	volatile uint16_t    l3r;                /**< Layer 3 result */
	volatile uint8_t     l4r;                /**< Layer 4 result */
	volatile uint8_t     cplan;              /**< Classification plan id */
	volatile uint16_t    nxthdr;             /**< Next Header  */
	volatile uint16_t    cksum;              /**< Checksum */
	volatile uint32_t    lcv;                /**< LCV */
	volatile uint8_t     shim_off[3];        /**< Shim offset */
	volatile uint8_t     eth_off;            /**< ETH offset */
	volatile uint8_t     llc_snap_off;       /**< LLC_SNAP offset */
	volatile uint8_t     vlan_off[2];        /**< VLAN offset */
	volatile uint8_t     etype_off;          /**< ETYPE offset */
	volatile uint8_t     pppoe_off;          /**< PPP offset */
	volatile uint8_t     mpls_off[2];        /**< MPLS offset */
	volatile uint8_t     ip_off[2];          /**< IP offset */
	volatile uint8_t     gre_off;            /**< GRE offset */
	volatile uint8_t     l4_off;             /**< Layer 4 offset */
	volatile uint8_t     nxthdr_off;         /**< Parser end point */
} _PackedType t_FmPrsResult;

#endif
