/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
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

/*
 @File          lnxwrp_resources.h

 @Description   FMD wrapper resource allocation functions.

*/

#ifndef LNXWRP_RESOURCES_H_
#define LNXWRP_RESOURCES_H_

#if !defined(FMAN_RESOURCES_UNIT_TEST)
#include "lnxwrp_fm.h"
#else
#include "lnxwrp_resources_ut.h"
#endif

#define ROUND(X) ((2*(X)+1)/2)
#define CEIL(X) ((X)+1)
/* #define ROUND_DIV(X, Y) (((X)+(Y)/2)/(Y)) */
#define ROUND_DIV(X, Y) ((2*(X)+(Y))/(2*(Y)))
#define CEIL_DIV(X, Y) (((X)+(Y)-1)/(Y))

/* used for resource calculus */
#define DPDE_1G 2	/* DQDP 1g - from LLD:
				DEFAULT_PORT_txFifoDeqPipelineDepth_1G */
#define DPDE_10G 8	/* DQDP 10g - from LLD:
				DEFAULT_PORT_txFifoDeqPipelineDepth_10G */

int fm_set_active_fman_ports(struct platform_device *of_dev,
			  t_LnxWrpFmDev *p_LnxWrpFmDev);

/* Calculate the fifosize based on MURAM allocation, number of ports, dpde
 * value and s/g software support (! Kernel does not suport s/g).
 *
 * Algorithm summary:
 * - Calculate the the minimum fifosize required for every type of port
 * (TX,RX for 1G, 2.5G and 10G).
 * - Set TX the minimum fifosize required.
 * - Distribute the remaining buffers (after all TX were set) to RX ports
 * based on:
 *   1G   RX = Remaining_buffers * 1/(1+2.5+10)
 *   2.5G RX = Remaining_buffers * 2.5/(1+2.5+10)
 *   10G  RX = Remaining_buffers * 10/(1+2.5+10)
 * - if the RX is smaller than the minimum required, then set the minimum
 * required
 * - In the end distribuite the leftovers if there are any (due to
 * unprecise calculus) or if over allocation cat some buffers from all RX
 * ports w/o pass over minimum required treshold, but if there must be
 * pass the treshold in order to cat the over allocation ,then this
 * configuration can not be set - KERN_ALERT.
*/
int fm_precalculate_fifosizes(t_LnxWrpFmDev *p_LnxWrpFmDev,
			   int muram_fifo_size);

#if !defined(FMAN_RESOURCES_UNIT_TEST)
int fm_config_precalculate_fifosize(t_LnxWrpFmPortDev *p_LnxWrpFmPortDev);
#endif

/* Compute FMan open DMA based on total number of open DMAs and
 * number of available fman ports.
 *
 * By default 10g ports are set to input parameters. The other ports
 * tries to keep the proportion rx=2tx open dmas or tresholds.
 *
 * If leftovers, then those will be set as shared.
 *
 * If after computing overflow appears, then it decrements open dma
 * for all ports w/o cross the tresholds. If the tresholds are meet
 * and is still overflow, then it returns error.
*/
int fm_precalculate_open_dma(t_LnxWrpFmDev *p_LnxWrpFmDev,
			  int max_fm_open_dma,
			  int default_tx_10g_dmas,
			  int default_rx_10g_dmas,
			  int min_tx_10g_treshold, int min_rx_10g_treshold);

#if !defined(FMAN_RESOURCES_UNIT_TEST)
int fm_config_precalculate_open_dma(t_LnxWrpFmPortDev *p_LnxWrpFmPortDev);
#endif

/* Compute FMan tnums based on available tnums and number of ports.
 * Set defaults (minim tresholds) and then distribute leftovers.*/
int fm_precalculate_tnums(t_LnxWrpFmDev *p_LnxWrpFmDev, int max_fm_tnums);

#if !defined(FMAN_RESOURCES_UNIT_TEST)
int fm_config_precalculate_tnums(t_LnxWrpFmPortDev *p_LnxWrpFmPortDev);
#endif

#endif /* LNXWRP_RESOURCES_H_ */
