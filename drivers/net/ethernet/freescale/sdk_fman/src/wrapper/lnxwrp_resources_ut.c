/* Copyright (c) 2012 Freescale Semiconductor, Inc.
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

#include "lnxwrp_resources.h"
#include "lnxwrp_resources_ut.h"

#define KILOBYTE 0x400 /* 1024 */

typedef enum e_board_type {
	e_p3041,
	e_p4080,
	e_p5020,
	e_p1023
} e_board_type;

uint8_t board_type;
uint32_t muram_size = 0;
uint32_t dmas_num = 0;
uint32_t task_num = 0;
uint32_t frame_size = 0;
uint32_t oh_num = 0;
uint32_t num_ports_1g = 0;
uint32_t num_ports_10g = 0;
uint32_t num_ports_2g5 = 0;
uint32_t fsl_fman_phy_maxfrm = 0;
uint32_t dpa_rx_extra_headroom = 0;

void show_help(void){
	printf(" help: \n");
	printf(" -b <board_type> -f <max_fram_size(mtu)> -o <num_oh_ports> -g1"
		" <num_1g_ports> -g10 <num_10g_ports> -g25 <num_2g5_ports>\n");
	printf("    Maxim num of DMAS availbale:  P3/P4/P5:32 ,  P1023:16 \n");
	printf("    Maxim num of TNUMs availbale: P3/P4/P5:128,  P1023:32 \n");
	printf("    Muram size:                   P3/P4/P5:160K, P1023:64K \n");
	printf("    Number of ports:\n");
	printf("        P3/P5: 5p 1g, 1p 10g, 7p oh \n");
	printf("        P4   : 4p 1g, 1p 10g, 7p oh \n");
	printf("        P1   : 2p 1g, 0p 10g, 4p oh \n");
	printf("    MTU: Default:1522, Jumbo:9600 \n");
}

int fm_set_param(t_LnxWrpFmDev *p_LnxWrpFmDev) {
	struct fm_active_ports *fm_active_ports_info = NULL;
	fm_active_ports_info = &p_LnxWrpFmDev->fm_active_ports_info;

	switch(board_type){
		case e_p3041:
		case e_p5020:
			muram_size = 160*KILOBYTE;
			dmas_num = 32;
			task_num = 128;
			if ((num_ports_1g+num_ports_2g5) > 5 || num_ports_10g > 1 || oh_num > 7)
				goto err_fm_set_param;
		break;
		case e_p4080:
			muram_size = 160*KILOBYTE;
			dmas_num = 32;
			task_num = 128;
			if ((num_ports_1g+num_ports_2g5) > 4 || num_ports_10g > 1 || oh_num > 7)
				goto err_fm_set_param;
		break;
		case e_p1023:
			muram_size = 64*KILOBYTE;
			dmas_num = 16;
			task_num = 128;
			if ((num_ports_1g+num_ports_2g5) > 2 || oh_num > 4)
				goto err_fm_set_param;
		break;
		default:
			goto err_fm_set_param;
		break;
	}

	p_LnxWrpFmDev->id = 0;
	fsl_fman_phy_maxfrm = frame_size;
	dpa_rx_extra_headroom = 0; /* ATTENTION: can be != 0 */
	fm_active_ports_info->num_oh_ports = oh_num;
	fm_active_ports_info->num_tx_ports = num_ports_1g;
	fm_active_ports_info->num_rx_ports = num_ports_1g;
	fm_active_ports_info->num_tx25_ports = num_ports_2g5;
	fm_active_ports_info->num_rx25_ports = num_ports_2g5;
	fm_active_ports_info->num_tx10_ports = num_ports_10g;
	fm_active_ports_info->num_rx10_ports = num_ports_10g;

	return 0;

err_fm_set_param:
	printf(" ERR: To many ports!!! \n");
	return -1;
}

int main (int argc, char *argv[]){
	t_LnxWrpFmDev LnxWrpFmDev;
	t_LnxWrpFmDev *p_LnxWrpFmDev = &LnxWrpFmDev;
	int tokens_cnt = 1;

	char *token = NULL;

	while(tokens_cnt < argc)
	{
	        token = argv[tokens_cnt++];
		if (strcmp(token, "-b") == 0){
			if(strcmp(argv[tokens_cnt],"p3") == 0)
				board_type = e_p3041;
			else if(strcmp(argv[tokens_cnt],"p4") == 0)
				board_type = e_p4080;
			else if(strcmp(argv[tokens_cnt],"p5") == 0)
				board_type = e_p5020;
			else if(strcmp(argv[tokens_cnt],"p1") == 0)
				board_type = e_p1023;
			else
				show_help();
			tokens_cnt++;
		}
		else if(strcmp(token, "-d") == 0){
			dmas_num = atoi(argv[tokens_cnt++]);
		}
		else if(strcmp(token, "-t") == 0)
			task_num = atoi(argv[tokens_cnt++]);
		else if(strcmp(token, "-f") == 0)
			frame_size = atoi(argv[tokens_cnt++]);
		else if(strcmp(token, "-o") == 0)
			oh_num = atoi(argv[tokens_cnt++]);
		else if(strcmp(token, "-g1") == 0)
			num_ports_1g = atoi(argv[tokens_cnt++]);
		else if(strcmp(token, "-g10") == 0)
			num_ports_10g = atoi(argv[tokens_cnt++]);
		else if(strcmp(token, "-g25") == 0)
			num_ports_2g5 = atoi(argv[tokens_cnt++]);
		else {
			show_help();
			return -1;
		}
	}

	if(fm_set_param(p_LnxWrpFmDev) < 0){
		show_help();
		return -1;
	}

	if(fm_precalculate_fifosizes(
		p_LnxWrpFmDev,
		128*KILOBYTE)
		!= 0)
		return -1;
	if(fm_precalculate_open_dma(
		p_LnxWrpFmDev,
		dmas_num,                   /* max open dmas:dpaa_integration_ext.h */
		FM_DEFAULT_TX10G_OPENDMA,   /* default TX 10g open dmas */
		FM_DEFAULT_RX10G_OPENDMA,   /* default RX 10g open dmas */
		FM_10G_OPENDMA_MIN_TRESHOLD,/* TX 10g minimum treshold */
		FM_10G_OPENDMA_MIN_TRESHOLD)/* RX 10g minimum treshold */
		!= 0)
		return -1;
	if(fm_precalculate_tnums(
		p_LnxWrpFmDev,
		task_num) /* max TNUMS: dpa integration file. */
		!= 0)
		 return -1;

	return 0;
}
