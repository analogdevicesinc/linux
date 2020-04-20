/*
 * Cadence High-Definition Multimedia Interface (HDMI) driver
 *
 * Copyright (C) 2019 NXP Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <drm/drm_of.h>
#include <drm/drm_vblank.h>
#include <drm/drm_print.h>
#include <drm/drm_crtc_helper.h>
#include <linux/io.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_atomic.h>
#include <linux/io.h>

#include <drm/bridge/cdns-mhdp.h>
#include "cdns-mhdp-phy.h"

/* HDMI TX clock control settings */
struct hdmi_ctrl {
	u32 pixel_clk_freq_min;
	u32 pixel_clk_freq_max;
	u32 feedback_factor;
	u32 data_range_kbps_min;
	u32 data_range_kbps_max;
	u32 cmnda_pll0_ip_div;
	u32 cmn_ref_clk_dig_div;
	u32 ref_clk_divider_scaler;
	u32 pll_fb_div_total;
	u32 cmnda_pll0_fb_div_low;
	u32 cmnda_pll0_fb_div_high;
	u32 pixel_div_total;
	u32 cmnda_pll0_pxdiv_low;
	u32 cmnda_pll0_pxdiv_high;
	u32 vco_freq_min;
	u32 vco_freq_max;
	u32 vco_ring_select;
	u32 cmnda_hs_clk_0_sel;
	u32 cmnda_hs_clk_1_sel;
	u32 hsclk_div_at_xcvr;
	u32 hsclk_div_tx_sub_rate;
	u32 cmnda_pll0_hs_sym_div_sel;
	u32 cmnda_pll0_clk_freq_min;
	u32 cmnda_pll0_clk_freq_max;
};

/* HDMI TX clock control settings, pixel clock is output */
static const struct hdmi_ctrl imx8mq_ctrl_table[] = {
/*Minclk  Maxclk Fdbak  DR_min   DR_max  ip_d  dig  DS    Totl */
{ 27000,  27000, 1000,  270000,  270000, 0x03, 0x1, 0x1,  240, 0x0BC, 0x030,  80, 0x026, 0x026, 2160000, 2160000, 0, 2, 2, 2, 4, 0x3,  27000,  27000},
{ 27000,  27000, 1250,  337500,  337500, 0x03, 0x1, 0x1,  300, 0x0EC, 0x03C, 100, 0x030, 0x030, 2700000, 2700000, 0, 2, 2, 2, 4, 0x3,  33750,  33750},
{ 27000,  27000, 1500,  405000,  405000, 0x03, 0x1, 0x1,  360, 0x11C, 0x048, 120, 0x03A, 0x03A, 3240000, 3240000, 0, 2, 2, 2, 4, 0x3,  40500,  40500},
{ 27000,  27000, 2000,  540000,  540000, 0x03, 0x1, 0x1,  240, 0x0BC, 0x030,  80, 0x026, 0x026, 2160000, 2160000, 0, 2, 2, 2, 4, 0x2,  54000,  54000},
{ 54000,  54000, 1000,  540000,  540000, 0x03, 0x1, 0x1,  480, 0x17C, 0x060,  80, 0x026, 0x026, 4320000, 4320000, 1, 2, 2, 2, 4, 0x3,  54000,  54000},
{ 54000,  54000, 1250,  675000,  675000, 0x04, 0x1, 0x1,  400, 0x13C, 0x050,  50, 0x017, 0x017, 2700000, 2700000, 0, 1, 1, 2, 4, 0x2,  67500,  67500},
{ 54000,  54000, 1500,  810000,  810000, 0x04, 0x1, 0x1,  480, 0x17C, 0x060,  60, 0x01C, 0x01C, 3240000, 3240000, 0, 2, 2, 2, 2, 0x2,  81000,  81000},
{ 54000,  54000, 2000, 1080000, 1080000, 0x03, 0x1, 0x1,  240, 0x0BC, 0x030,  40, 0x012, 0x012, 2160000, 2160000, 0, 2, 2, 2, 1, 0x1, 108000, 108000},
{ 74250,  74250, 1000,  742500,  742500, 0x03, 0x1, 0x1,  660, 0x20C, 0x084,  80, 0x026, 0x026, 5940000, 5940000, 1, 2, 2, 2, 4, 0x3,  74250,  74250},
{ 74250,  74250, 1250,  928125,  928125, 0x04, 0x1, 0x1,  550, 0x1B4, 0x06E,  50, 0x017, 0x017, 3712500, 3712500, 1, 1, 1, 2, 4, 0x2,  92812,  92812},
{ 74250,  74250, 1500, 1113750, 1113750, 0x04, 0x1, 0x1,  660, 0x20C, 0x084,  60, 0x01C, 0x01C, 4455000, 4455000, 1, 2, 2, 2, 2, 0x2, 111375, 111375},
{ 74250,  74250, 2000, 1485000, 1485000, 0x03, 0x1, 0x1,  330, 0x104, 0x042,  40, 0x012, 0x012, 2970000, 2970000, 0, 2, 2, 2, 1, 0x1, 148500, 148500},
{ 99000,  99000, 1000,  990000,  990000, 0x03, 0x1, 0x1,  440, 0x15C, 0x058,  40, 0x012, 0x012, 3960000, 3960000, 1, 2, 2, 2, 2, 0x2,  99000,  99000},
{ 99000,  99000, 1250, 1237500, 1237500, 0x03, 0x1, 0x1,  275, 0x0D8, 0x037,  25, 0x00B, 0x00A, 2475000, 2475000, 0, 1, 1, 2, 2, 0x1, 123750, 123750},
{ 99000,  99000, 1500, 1485000, 1485000, 0x03, 0x1, 0x1,  330, 0x104, 0x042,  30, 0x00D, 0x00D, 2970000, 2970000, 0, 2, 2, 2, 1, 0x1, 148500, 148500},
{ 99000,  99000, 2000, 1980000, 1980000, 0x03, 0x1, 0x1,  440, 0x15C, 0x058,  40, 0x012, 0x012, 3960000, 3960000, 1, 2, 2, 2, 1, 0x1, 198000, 198000},
{148500, 148500, 1000, 1485000, 1485000, 0x03, 0x1, 0x1,  660, 0x20C, 0x084,  40, 0x012, 0x012, 5940000, 5940000, 1, 2, 2, 2, 2, 0x2, 148500, 148500},
{148500, 148500, 1250, 1856250, 1856250, 0x04, 0x1, 0x1,  550, 0x1B4, 0x06E,  25, 0x00B, 0x00A, 3712500, 3712500, 1, 1, 1, 2, 2, 0x1, 185625, 185625},
{148500, 148500, 1500, 2227500, 2227500, 0x03, 0x1, 0x1,  495, 0x188, 0x063,  30, 0x00D, 0x00D, 4455000, 4455000, 1, 1, 1, 2, 2, 0x1, 222750, 222750},
{148500, 148500, 2000, 2970000, 2970000, 0x03, 0x1, 0x1,  660, 0x20C, 0x084,  40, 0x012, 0x012, 5940000, 5940000, 1, 2, 2, 2, 1, 0x1, 297000, 297000},
{198000, 198000, 1000, 1980000, 1980000, 0x03, 0x1, 0x1,  220, 0x0AC, 0x02C,  10, 0x003, 0x003, 1980000, 1980000, 0, 1, 1, 2, 1, 0x0, 198000, 198000},
{198000, 198000, 1250, 2475000, 2475000, 0x03, 0x1, 0x1,  550, 0x1B4, 0x06E,  25, 0x00B, 0x00A, 4950000, 4950000, 1, 1, 1, 2, 2, 0x1, 247500, 247500},
{198000, 198000, 1500, 2970000, 2970000, 0x03, 0x1, 0x1,  330, 0x104, 0x042,  15, 0x006, 0x005, 2970000, 2970000, 0, 1, 1, 2, 1, 0x0, 297000, 297000},
{198000, 198000, 2000, 3960000, 3960000, 0x03, 0x1, 0x1,  440, 0x15C, 0x058,  20, 0x008, 0x008, 3960000, 3960000, 1, 1, 1, 2, 1, 0x0, 396000, 396000},
{297000, 297000, 1000, 2970000, 2970000, 0x03, 0x1, 0x1,  330, 0x104, 0x042,  10, 0x003, 0x003, 2970000, 2970000, 0, 1, 1, 2, 1, 0x0, 297000, 297000},
{297000, 297000, 1500, 4455000, 4455000, 0x03, 0x1, 0x1,  495, 0x188, 0x063,  15, 0x006, 0x005, 4455000, 4455000, 1, 1, 1, 2, 1, 0x0, 445500, 445500},
{297000, 297000, 2000, 5940000, 5940000, 0x03, 0x1, 0x1,  660, 0x20C, 0x084,  20, 0x008, 0x008, 5940000, 5940000, 1, 1, 1, 2, 1, 0x0, 594000, 594000},
{594000, 594000, 1000, 5940000, 5940000, 0x03, 0x1, 0x1,  660, 0x20C, 0x084,  10, 0x003, 0x003, 5940000, 5940000, 1, 1, 1, 2, 1, 0x0, 594000, 594000},
{594000, 594000,  750, 4455000, 4455000, 0x03, 0x1, 0x1,  495, 0x188, 0x063,  10, 0x003, 0x003, 4455000, 4455000, 1, 1, 1, 2, 1, 0x0, 445500, 445500},
{594000, 594000,  625, 3712500, 3712500, 0x04, 0x1, 0x1,  550, 0x1B4, 0x06E,  10, 0x003, 0x003, 3712500, 3712500, 1, 1, 1, 2, 1, 0x0, 371250, 371250},
{594000, 594000,  500, 2970000, 2970000, 0x03, 0x1, 0x1,  660, 0x20C, 0x084,  10, 0x003, 0x003, 5940000, 5940000, 1, 1, 1, 2, 2, 0x1, 297000, 297000},
};

/* HDMI TX clock control settings, pixel clock is input */
static const struct hdmi_ctrl imx8qm_ctrl_table[] = {
/*pclk_l  pclk_h  fd    DRR_L    DRR_H   PLLD */
{ 25000,  42500, 1000,  250000,  425000, 0x05, 0x01, 0x01, 400, 0x182, 0x00A, 0, 0, 0, 2000000, 3400000, 0, 2, 2, 2, 4, 0x03,  25000,  42500},
{ 42500,  85000, 1000,  425000,  850000, 0x08, 0x03, 0x01, 320, 0x132, 0x00A, 0, 0, 0, 1700000, 3400000, 0, 1, 1, 2, 4, 0x02,  42500,  85000},
{ 85000, 170000, 1000,  850000, 1700000, 0x11, 0x00, 0x07, 340, 0x146, 0x00A, 0, 0, 0, 1700000, 3400000, 0, 1, 1, 2, 2, 0x01,  85000, 170000},
{170000, 340000, 1000, 1700000, 3400000, 0x22, 0x01, 0x07, 340, 0x146, 0x00A, 0, 0, 0, 1700000, 3400000, 0, 1, 1, 2, 1, 0x00, 170000, 340000},
{340000, 600000, 1000, 3400000, 6000000, 0x3C, 0x03, 0x06, 600, 0x24A, 0x00A, 0, 0, 0, 3400000, 6000000, 1, 1, 1, 2, 1, 0x00, 340000, 600000},
{ 25000,  34000, 1205,  312500,  425000, 0x04, 0x01, 0x01, 400, 0x182, 0x00A, 0, 0, 0, 2500000, 3400000, 0, 2, 2, 2, 4, 0x03,  31250,  42500},
{ 34000,  68000, 1205,  425000,  850000, 0x06, 0x02, 0x01, 300, 0x11E, 0x00A, 0, 0, 0, 1700000, 3400000, 0, 1, 1, 2, 4, 0x02,  42500,  85000},
{ 68000, 136000, 1205,  850000, 1700000, 0x0D, 0x02, 0x02, 325, 0x137, 0x00A, 0, 0, 0, 1700000, 3400000, 0, 1, 1, 2, 2, 0x01,  85000, 170000},
{136000, 272000, 1205, 1700000, 3400000, 0x1A, 0x02, 0x04, 325, 0x137, 0x00A, 0, 0, 0, 1700000, 3400000, 0, 1, 1, 2, 1, 0x00, 170000, 340000},
{272000, 480000, 1205, 3400000, 6000000, 0x30, 0x03, 0x05, 600, 0x24A, 0x00A, 0, 0, 0, 3400000, 6000000, 1, 1, 1, 2, 1, 0x00, 340000, 600000},
{ 25000,  28000, 1500,  375000,  420000, 0x03, 0x01, 0x01, 360, 0x15A, 0x00A, 0, 0, 0, 3000000, 3360000, 0, 2, 2, 2, 4, 0x03,  37500,  42000},
{ 28000,  56000, 1500,  420000,  840000, 0x06, 0x02, 0x01, 360, 0x15A, 0x00A, 0, 0, 0, 1680000, 3360000, 0, 1, 1, 2, 4, 0x02,  42000,  84000},
{ 56000, 113000, 1500,  840000, 1695000, 0x0B, 0x00, 0x05, 330, 0x13C, 0x00A, 0, 0, 0, 1680000, 3390000, 0, 1, 1, 2, 2, 0x01,  84000, 169500},
{113000, 226000, 1500, 1695000, 3390000, 0x16, 0x01, 0x05, 330, 0x13C, 0x00A, 0, 0, 0, 1695000, 3390000, 0, 1, 1, 2, 1, 0x00, 169500, 339000},
{226000, 400000, 1500, 3390000, 6000000, 0x28, 0x03, 0x04, 600, 0x24A, 0x00A, 0, 0, 0, 3390000, 6000000, 1, 1, 1, 2, 1, 0x00, 339000, 600000},
{ 25000,  42500, 2000,  500000,  850000, 0x05, 0x01, 0x01, 400, 0x182, 0x00A, 0, 0, 0, 2000000, 3400000, 0, 1, 1, 2, 4, 0x02,  50000,  85000},
{ 42500,  85000, 2000,  850000, 1700000, 0x08, 0x03, 0x01, 320, 0x132, 0x00A, 0, 0, 0, 1700000, 3400000, 0, 1, 1, 2, 2, 0x01,  85000, 170000},
{ 85000, 170000, 2000, 1700000, 3400000, 0x11, 0x00, 0x07, 340, 0x146, 0x00A, 0, 0, 0, 1700000, 3400000, 0, 1, 1, 2, 1, 0x00, 170000, 340000},
{170000, 300000, 2000, 3400000, 6000000, 0x22, 0x01, 0x06, 680, 0x29A, 0x00A, 0, 0, 0, 3400000, 6000000, 1, 1, 1, 2, 1, 0x00, 340000, 600000},
{594000, 594000, 5000, 2970000, 2970000, 0x3C, 0x03, 0x06, 600, 0x24A, 0x00A, 0, 0, 0, 5940000, 5940000, 1, 1, 1, 2, 2, 0x01, 297000, 297000},
{594000, 594000, 6250, 3712500, 3712500, 0x3C, 0x03, 0x06, 375, 0x169, 0x00A, 0, 0, 0, 3712500, 3712500, 1, 1, 1, 2, 1, 0x00, 371250, 371250},
{594000, 594000, 7500, 4455000, 4455000, 0x3C, 0x03, 0x06, 450, 0x1B4, 0x00A, 0, 0, 0, 4455000, 4455000, 1, 1, 1, 2, 1, 0x00, 445500, 445500},
};

/* HDMI TX PLL tuning settings */
struct hdmi_pll_tuning {
	u32 vco_freq_bin;
	u32 vco_freq_min;
	u32 vco_freq_max;
	u32 volt_to_current_coarse;
	u32 volt_to_current;
	u32 ndac_ctrl;
	u32 pmos_ctrl;
	u32 ptat_ndac_ctrl;
	u32 feedback_div_total;
	u32 charge_pump_gain;
	u32 coarse_code;
	u32 v2i_code;
	u32 vco_cal_code;
};

/* HDMI TX PLL tuning settings, pixel clock is output */
static const struct hdmi_pll_tuning imx8mq_pll_table[] = {
/*    bin VCO_freq min/max  coar  cod NDAC  PMOS PTAT div-T P-Gain Coa V2I CAL */
    {  1, 1980000, 1980000, 0x4, 0x3, 0x0, 0x09, 0x09, 220, 0x42, 160, 5, 183 },
    {  2, 2160000, 2160000, 0x4, 0x3, 0x0, 0x09, 0x09, 240, 0x42, 166, 6, 208 },
    {  3, 2475000, 2475000, 0x5, 0x3, 0x1, 0x00, 0x07, 275, 0x42, 167, 6, 209 },
    {  4, 2700000, 2700000, 0x5, 0x3, 0x1, 0x00, 0x07, 300, 0x42, 188, 6, 230 },
    {  4, 2700000, 2700000, 0x5, 0x3, 0x1, 0x00, 0x07, 400, 0x4C, 188, 6, 230 },
    {  5, 2970000, 2970000, 0x6, 0x3, 0x1, 0x00, 0x07, 330, 0x42, 183, 6, 225 },
    {  6, 3240000, 3240000, 0x6, 0x3, 0x1, 0x00, 0x07, 360, 0x42, 203, 7, 256 },
    {  6, 3240000, 3240000, 0x6, 0x3, 0x1, 0x00, 0x07, 480, 0x4C, 203, 7, 256 },
    {  7, 3712500, 3712500, 0x4, 0x3, 0x0, 0x07, 0x0F, 550, 0x4C, 212, 7, 257 },
    {  8, 3960000, 3960000, 0x5, 0x3, 0x0, 0x07, 0x0F, 440, 0x42, 184, 6, 226 },
    {  9, 4320000, 4320000, 0x5, 0x3, 0x1, 0x07, 0x0F, 480, 0x42, 205, 7, 258 },
    { 10, 4455000, 4455000, 0x5, 0x3, 0x0, 0x07, 0x0F, 495, 0x42, 219, 7, 272 },
    { 10, 4455000, 4455000, 0x5, 0x3, 0x0, 0x07, 0x0F, 660, 0x4C, 219, 7, 272 },
    { 11, 4950000, 4950000, 0x6, 0x3, 0x1, 0x00, 0x07, 550, 0x42, 213, 7, 258 },
    { 12, 5940000, 5940000, 0x7, 0x3, 0x1, 0x00, 0x07, 660, 0x42, 244, 8, 292 },
};

/* HDMI TX PLL tuning settings, pixel clock is input */
static const struct hdmi_pll_tuning imx8qm_pll_table[] = {
/*  bin VCO_freq min/max  coar  cod NDAC  PMOS PTAT div-T P-Gain  pad only */
	{ 0, 1700000, 2000000, 0x3, 0x1, 0x0, 0x8C, 0x2E, 300, 0x08D, 0, 0, 0 },
	{ 0, 1700000, 2000000, 0x3, 0x1, 0x0, 0x8C, 0x2E, 320, 0x08E, 0, 0, 0 },
	{ 0, 1700000, 2000000, 0x3, 0x1, 0x0, 0x8C, 0x2E, 325, 0x08E, 0, 0, 0 },
	{ 0, 1700000, 2000000, 0x3, 0x1, 0x0, 0x8C, 0x2E, 330, 0x08E, 0, 0, 0 },
	{ 0, 1700000, 2000000, 0x3, 0x1, 0x0, 0x8C, 0x2E, 340, 0x08F, 0, 0, 0 },
	{ 0, 1700000, 2000000, 0x3, 0x1, 0x0, 0x8C, 0x2E, 360, 0x0A7, 0, 0, 0 },
	{ 0, 1700000, 2000000, 0x3, 0x1, 0x0, 0x8C, 0x2E, 400, 0x0C5, 0, 0, 0 },
	{ 1, 2000000, 2400000, 0x3, 0x1, 0x0, 0x8C, 0x2E, 300, 0x086, 0, 0, 0 },
	{ 1, 2000000, 2400000, 0x3, 0x1, 0x0, 0x8C, 0x2E, 320, 0x087, 0, 0, 0 },
	{ 1, 2000000, 2400000, 0x3, 0x1, 0x0, 0x8C, 0x2E, 325, 0x087, 0, 0, 0 },
	{ 1, 2000000, 2400000, 0x3, 0x1, 0x0, 0x8C, 0x2E, 330, 0x104, 0, 0, 0 },
	{ 1, 2000000, 2400000, 0x3, 0x1, 0x0, 0x8C, 0x2E, 340, 0x08B, 0, 0, 0 },
	{ 1, 2000000, 2400000, 0x3, 0x1, 0x0, 0x8C, 0x2E, 360, 0x08D, 0, 0, 0 },
	{ 1, 2000000, 2400000, 0x3, 0x1, 0x0, 0x8C, 0x2E, 400, 0x0A6, 0, 0, 0 },
	{ 2, 2400000, 2800000, 0x3, 0x1, 0x0, 0x04, 0x0D, 300, 0x04E, 0, 0, 0 },
	{ 2, 2400000, 2800000, 0x3, 0x1, 0x0, 0x04, 0x0D, 320, 0x04F, 0, 0, 0 },
	{ 2, 2400000, 2800000, 0x3, 0x1, 0x0, 0x04, 0x0D, 325, 0x04F, 0, 0, 0 },
	{ 2, 2400000, 2800000, 0x3, 0x1, 0x0, 0x04, 0x0D, 330, 0x085, 0, 0, 0 },
	{ 2, 2400000, 2800000, 0x3, 0x1, 0x0, 0x04, 0x0D, 340, 0x085, 0, 0, 0 },
	{ 2, 2400000, 2800000, 0x3, 0x1, 0x0, 0x04, 0x0D, 360, 0x086, 0, 0, 0 },
	{ 2, 2400000, 2800000, 0x3, 0x1, 0x0, 0x04, 0x0D, 400, 0x08B, 0, 0, 0 },
	{ 3, 2800000, 3400000, 0x3, 0x1, 0x0, 0x04, 0x0D, 300, 0x047, 0, 0, 0 },
	{ 3, 2800000, 3400000, 0x3, 0x1, 0x0, 0x04, 0x0D, 320, 0x04B, 0, 0, 0 },
	{ 3, 2800000, 3400000, 0x3, 0x1, 0x0, 0x04, 0x0D, 325, 0x04B, 0, 0, 0 },
	{ 3, 2800000, 3400000, 0x3, 0x1, 0x0, 0x04, 0x0D, 330, 0x04B, 0, 0, 0 },
	{ 3, 2800000, 3400000, 0x3, 0x1, 0x0, 0x04, 0x0D, 340, 0x04D, 0, 0, 0 },
	{ 3, 2800000, 3400000, 0x3, 0x1, 0x0, 0x04, 0x0D, 360, 0x04E, 0, 0, 0 },
	{ 3, 2800000, 3400000, 0x3, 0x1, 0x0, 0x04, 0x0D, 400, 0x085, 0, 0, 0 },
	{ 4, 3400000, 3900000, 0x7, 0x1, 0x0, 0x8E, 0x2F, 375, 0x041, 0, 0, 0 },
	{ 4, 3400000, 3900000, 0x7, 0x1, 0x0, 0x8E, 0x2F, 600, 0x08D, 0, 0, 0 },
	{ 4, 3400000, 3900000, 0x7, 0x1, 0x0, 0x8E, 0x2F, 680, 0x0A6, 0, 0, 0 },
	{ 5, 3900000, 4500000, 0x7, 0x1, 0x0, 0x8E, 0x2F, 450, 0x041, 0, 0, 0 },
	{ 5, 3900000, 4500000, 0x7, 0x1, 0x0, 0x8E, 0x2F, 600, 0x087, 0, 0, 0 },
	{ 5, 3900000, 4500000, 0x7, 0x1, 0x0, 0x8E, 0x2F, 680, 0x0A4, 0, 0, 0 },
	{ 6, 4500000, 5200000, 0x7, 0x1, 0x0, 0x04, 0x0D, 600, 0x04F, 0, 0, 0 },
	{ 6, 4500000, 5200000, 0x7, 0x1, 0x0, 0x04, 0x0D, 680, 0x086, 0, 0, 0 },
	{ 7, 5200000, 6000000, 0x7, 0x1, 0x0, 0x04, 0x0D, 600, 0x04D, 0, 0, 0 },
	{ 7, 5200000, 6000000, 0x7, 0x1, 0x0, 0x04, 0x0D, 680, 0x04F, 0, 0, 0 }
};

static void hdmi_arc_config(struct cdns_mhdp_device *mhdp)
{
	u16 txpu_calib_code;
	u16 txpd_calib_code;
	u16 txpu_adj_calib_code;
	u16 txpd_adj_calib_code;
	u16 prev_calib_code;
	u16 new_calib_code;
	u16 rdata;

	/* Power ARC */
	cdns_phy_reg_write(mhdp, TXDA_CYA_AUXDA_CYA, 0x0001);

	prev_calib_code = cdns_phy_reg_read(mhdp, TX_DIG_CTRL_REG_2);
	txpu_calib_code = cdns_phy_reg_read(mhdp, CMN_TXPUCAL_CTRL);
	txpd_calib_code = cdns_phy_reg_read(mhdp, CMN_TXPDCAL_CTRL);
	txpu_adj_calib_code = cdns_phy_reg_read(mhdp, CMN_TXPU_ADJ_CTRL);
	txpd_adj_calib_code = cdns_phy_reg_read(mhdp, CMN_TXPD_ADJ_CTRL);

	new_calib_code = ((txpu_calib_code + txpd_calib_code) / 2)
		+ txpu_adj_calib_code + txpd_adj_calib_code;

	if (new_calib_code != prev_calib_code) {
		rdata = cdns_phy_reg_read(mhdp, TX_ANA_CTRL_REG_1);
		rdata &= 0xDFFF;
		cdns_phy_reg_write(mhdp, TX_ANA_CTRL_REG_1, rdata);
		cdns_phy_reg_write(mhdp, TX_DIG_CTRL_REG_2, new_calib_code);
		mdelay(10);
		rdata |= 0x2000;
		cdns_phy_reg_write(mhdp, TX_ANA_CTRL_REG_1, rdata);
		udelay(150);
	}

	cdns_phy_reg_write(mhdp, TX_ANA_CTRL_REG_2, 0x0100);
	udelay(100);
	cdns_phy_reg_write(mhdp, TX_ANA_CTRL_REG_2, 0x0300);
	udelay(100);
	cdns_phy_reg_write(mhdp, TX_ANA_CTRL_REG_3, 0x0000);
	udelay(100);
	cdns_phy_reg_write(mhdp, TX_ANA_CTRL_REG_1, 0x2008);
	udelay(100);
	cdns_phy_reg_write(mhdp, TX_ANA_CTRL_REG_1, 0x2018);
	udelay(100);
	cdns_phy_reg_write(mhdp, TX_ANA_CTRL_REG_1, 0x2098);
	cdns_phy_reg_write(mhdp, TX_ANA_CTRL_REG_2, 0x030C);
	cdns_phy_reg_write(mhdp, TX_ANA_CTRL_REG_5, 0x0010);
	udelay(100);
	cdns_phy_reg_write(mhdp, TX_ANA_CTRL_REG_4, 0x4001);
	mdelay(5);
	cdns_phy_reg_write(mhdp, TX_ANA_CTRL_REG_1, 0x2198);
	mdelay(5);
	cdns_phy_reg_write(mhdp, TX_ANA_CTRL_REG_2, 0x030D);
	udelay(100);
	cdns_phy_reg_write(mhdp, TX_ANA_CTRL_REG_2, 0x030F);
}

static void hdmi_phy_set_vswing(struct cdns_mhdp_device *mhdp)
{
	const u32 num_lanes = 4;
	u32 k;

	for (k = 0; k < num_lanes; k++) {
		cdns_phy_reg_write(mhdp, (TX_DIAG_TX_DRV | (k << 9)), 0x7c0);
		cdns_phy_reg_write(mhdp, (TX_TXCC_CPOST_MULT_00_0 | (k << 9)), 0x0);
		cdns_phy_reg_write(mhdp, (TX_TXCC_CAL_SCLR_MULT_0 | (k << 9)), 0x120);
	}
}

static int hdmi_feedback_factor(struct cdns_mhdp_device *mhdp)
{
	u32 feedback_factor;

	switch (mhdp->video_info.color_fmt) {
	case YCBCR_4_2_2:
		feedback_factor = 1000;
		break;
	case YCBCR_4_2_0:
		switch (mhdp->video_info.color_depth) {
		case 8:
			feedback_factor = 500;
			break;
		case 10:
			feedback_factor = 625;
			break;
		case 12:
			feedback_factor = 750;
			break;
		case 16:
			feedback_factor = 1000;
			break;
		default:
			DRM_ERROR("Invalid ColorDepth\n");
			return 0;
		}
		break;
	default:
		/* Assume RGB/YUV444 */
		switch (mhdp->video_info.color_depth) {
		case 10:
			feedback_factor = 1250;
			break;
		case 12:
			feedback_factor = 1500;
			break;
		case 16:
			feedback_factor = 2000;
			break;
		default:
			feedback_factor = 1000;
		}
	}
	return feedback_factor;
}

static int hdmi_phy_config(struct cdns_mhdp_device *mhdp,
					const struct hdmi_ctrl *p_ctrl_table,
					const struct hdmi_pll_tuning *p_pll_table,
					char pclk_in)
{
	const u32 num_lanes = 4;
	u32 val, i, k;

	/* enable PHY isolation mode only for CMN */
	cdns_phy_reg_write(mhdp, PHY_PMA_ISOLATION_CTRL, 0xD000);

	/* set cmn_pll0_clk_datart1_div/cmn_pll0_clk_datart0_div dividers */
	val = cdns_phy_reg_read(mhdp, PHY_PMA_ISO_PLL_CTRL1);
	val &= 0xFF00;
	val |= 0x0012;
	cdns_phy_reg_write(mhdp, PHY_PMA_ISO_PLL_CTRL1, val);

	/* assert PHY reset from isolation register */
	cdns_phy_reg_write(mhdp, PHY_ISO_CMN_CTRL, 0x0000);
	/* assert PMA CMN reset */
	cdns_phy_reg_write(mhdp, PHY_PMA_ISO_CMN_CTRL, 0x0000);

	/* register XCVR_DIAG_BIDI_CTRL */
	for (k = 0; k < num_lanes; k++)
		cdns_phy_reg_write(mhdp, XCVR_DIAG_BIDI_CTRL | (k << 9), 0x00FF);

	/* Describing Task phy_cfg_hdp */

	val = cdns_phy_reg_read(mhdp, PHY_PMA_CMN_CTRL1);
	val &= 0xFFF7;
	val |= 0x0008;
	cdns_phy_reg_write(mhdp, PHY_PMA_CMN_CTRL1, val);

	/* PHY Registers */
	val = cdns_phy_reg_read(mhdp, PHY_PMA_CMN_CTRL1);
	val &= 0xCFFF;
	val |= p_ctrl_table->cmn_ref_clk_dig_div << 12;
	cdns_phy_reg_write(mhdp, PHY_PMA_CMN_CTRL1, val);

	val = cdns_phy_reg_read(mhdp, PHY_HDP_CLK_CTL);
	val &= 0x00FF;
	val |= 0x1200;
	cdns_phy_reg_write(mhdp, PHY_HDP_CLK_CTL, val);

	/* Common control module control and diagnostic registers */
	val = cdns_phy_reg_read(mhdp, CMN_CDIAG_REFCLK_CTRL);
	val &= 0x8FFF;
	val |= p_ctrl_table->ref_clk_divider_scaler << 12;
	val |= 0x00C0;
	cdns_phy_reg_write(mhdp, CMN_CDIAG_REFCLK_CTRL, val);

	/* High speed clock used */
	val = cdns_phy_reg_read(mhdp, CMN_DIAG_HSCLK_SEL);
	val &= 0xFF00;
	val |= (p_ctrl_table->cmnda_hs_clk_0_sel >> 1) << 0;
	val |= (p_ctrl_table->cmnda_hs_clk_1_sel >> 1) << 4;
	cdns_phy_reg_write(mhdp, CMN_DIAG_HSCLK_SEL, val);

	for (k = 0; k < num_lanes; k++) {
		val = cdns_phy_reg_read(mhdp, (XCVR_DIAG_HSCLK_SEL | (k << 9)));
		val &= 0xCFFF;
		val |= (p_ctrl_table->cmnda_hs_clk_0_sel >> 1) << 12;
		cdns_phy_reg_write(mhdp, (XCVR_DIAG_HSCLK_SEL | (k << 9)), val);
	}

	/* PLL 0 control state machine registers */
	val = p_ctrl_table->vco_ring_select << 12;
	cdns_phy_reg_write(mhdp, CMN_PLLSM0_USER_DEF_CTRL, val);

	if (pclk_in == true)
		val = 0x30A0;
	else {
		val = cdns_phy_reg_read(mhdp, CMN_PLL0_VCOCAL_START);
		val &= 0xFE00;
		val |= p_pll_table->vco_cal_code;
	}
	cdns_phy_reg_write(mhdp, CMN_PLL0_VCOCAL_START, val);

	cdns_phy_reg_write(mhdp, CMN_PLL0_VCOCAL_INIT_TMR, 0x0064);
	cdns_phy_reg_write(mhdp, CMN_PLL0_VCOCAL_ITER_TMR, 0x000A);

	/* Common functions control and diagnostics registers */
	val = p_ctrl_table->cmnda_pll0_hs_sym_div_sel << 8;
	val |= p_ctrl_table->cmnda_pll0_ip_div;
	cdns_phy_reg_write(mhdp, CMN_DIAG_PLL0_INCLK_CTRL, val);

	cdns_phy_reg_write(mhdp, CMN_DIAG_PLL0_OVRD, 0x0000);

	val = p_ctrl_table->cmnda_pll0_fb_div_high;
	val |= (1 << 15);
	cdns_phy_reg_write(mhdp, CMN_DIAG_PLL0_FBH_OVRD, val);

	val = p_ctrl_table->cmnda_pll0_fb_div_low;
	val |= (1 << 15);
	cdns_phy_reg_write(mhdp, CMN_DIAG_PLL0_FBL_OVRD, val);

	if (pclk_in == false) {
		val = p_ctrl_table->cmnda_pll0_pxdiv_low;
		cdns_phy_reg_write(mhdp, CMN_DIAG_PLL0_PXL_DIVL, val);

		val = p_ctrl_table->cmnda_pll0_pxdiv_high;
		val |= (1 << 15);
		cdns_phy_reg_write(mhdp, CMN_DIAG_PLL0_PXL_DIVH, val);
	}

	val = p_pll_table->volt_to_current_coarse;
	val |= (p_pll_table->volt_to_current) << 4;
	cdns_phy_reg_write(mhdp, CMN_DIAG_PLL0_V2I_TUNE, val);

	val = p_pll_table->charge_pump_gain;
	cdns_phy_reg_write(mhdp, CMN_DIAG_PLL0_CP_TUNE, val);

	cdns_phy_reg_write(mhdp, CMN_DIAG_PLL0_LF_PROG, 0x0008);

	val = p_pll_table->pmos_ctrl;
	val |= (p_pll_table->ndac_ctrl) << 8;
	cdns_phy_reg_write(mhdp, CMN_DIAG_PLL0_PTATIS_TUNE1, val);

	val = p_pll_table->ptat_ndac_ctrl;
	cdns_phy_reg_write(mhdp, CMN_DIAG_PLL0_PTATIS_TUNE2, val);

	if (pclk_in == true)
	cdns_phy_reg_write(mhdp, CMN_DIAG_PLL0_TEST_MODE, 0x0022);
	else
	cdns_phy_reg_write(mhdp, CMN_DIAG_PLL0_TEST_MODE, 0x0020);
	cdns_phy_reg_write(mhdp, CMN_PSM_CLK_CTRL, 0x0016);

	/* Transceiver control and diagnostic registers */
	for (k = 0; k < num_lanes; k++) {
		val = cdns_phy_reg_read(mhdp, (XCVR_DIAG_PLLDRC_CTRL | (k << 9)));
		val &= 0xBFFF;
		cdns_phy_reg_write(mhdp, (XCVR_DIAG_PLLDRC_CTRL | (k << 9)), val);
	}

	for (k = 0; k < num_lanes; k++) {
		val = cdns_phy_reg_read(mhdp, (TX_DIAG_TX_CTRL | (k << 9)));
		val &= 0xFF3F;
		val |= (p_ctrl_table->hsclk_div_tx_sub_rate >> 1) << 6;
		cdns_phy_reg_write(mhdp, (TX_DIAG_TX_CTRL | (k << 9)), val);
	}

	/*
	 * for single ended reference clock val |= 0x0030;
	 * for differential clock  val |= 0x0000;
	 */
	val = cdns_phy_reg_read(mhdp, PHY_PMA_CMN_CTRL1);
	val &= 0xFF8F;
	if (pclk_in == true)
		val |= 0x0030;
	cdns_phy_reg_write(mhdp, PHY_PMA_CMN_CTRL1, val);

	/* for differential clock on the refclk_p and
	 * refclk_m off chip pins: CMN_DIAG_ACYA[8]=1'b1 */
	cdns_phy_reg_write(mhdp, CMN_DIAG_ACYA, 0x0100);

	/* Deassert PHY reset */
	cdns_phy_reg_write(mhdp, PHY_ISO_CMN_CTRL, 0x0001);
	cdns_phy_reg_write(mhdp, PHY_PMA_ISO_CMN_CTRL, 0x0003);

	/* Power state machine registers */
	for (k = 0; k < num_lanes; k++)
		cdns_phy_reg_write(mhdp, XCVR_PSM_RCTRL | (k << 9), 0xFEFC);

	/* Assert cmn_macro_pwr_en */
	cdns_phy_reg_write(mhdp, PHY_PMA_ISO_CMN_CTRL, 0x0013);

	/* wait for cmn_macro_pwr_en_ack */
	for (i = 0; i < 10; i++) {
		val = cdns_phy_reg_read(mhdp, PHY_PMA_ISO_CMN_CTRL);
		if (val & (1 << 5))
			break;
		msleep(20);
	}
	if (i == 10) {
		DRM_ERROR("PMA ouput macro power up failed\n");
		return false;
	}

	/* wait for cmn_ready */
	for (i = 0; i < 10; i++) {
		val = cdns_phy_reg_read(mhdp, PHY_PMA_CMN_CTRL1);
		if (val & (1 << 0))
			break;
		msleep(20);
	}
	if (i == 10) {
		DRM_ERROR("PMA output ready failed\n");
		return false;
	}

	for (k = 0; k < num_lanes; k++) {
		cdns_phy_reg_write(mhdp, TX_PSC_A0 | (k << 9), 0x6791);
		cdns_phy_reg_write(mhdp, TX_PSC_A1 | (k << 9), 0x6790);
		cdns_phy_reg_write(mhdp, TX_PSC_A2 | (k << 9), 0x0090);
		cdns_phy_reg_write(mhdp, TX_PSC_A3 | (k << 9), 0x0090);

		val = cdns_phy_reg_read(mhdp, RX_PSC_CAL | (k << 9));
		val &= 0xFFBB;
		cdns_phy_reg_write(mhdp, RX_PSC_CAL | (k << 9), val);

		val = cdns_phy_reg_read(mhdp, RX_PSC_A0 | (k << 9));
		val &= 0xFFBB;
		cdns_phy_reg_write(mhdp, RX_PSC_A0 | (k << 9), val);
	}
	return true;
}

static int hdmi_phy_cfg_t28hpc(struct cdns_mhdp_device *mhdp,
				struct drm_display_mode *mode)
{
	const struct hdmi_ctrl *p_ctrl_table;
	const struct hdmi_pll_tuning *p_pll_table;
	const u32 refclk_freq_khz = 27000;
	const u8 pclk_in = false;
	u32 pixel_freq = mode->clock;
	u32 vco_freq, char_freq;
	u32 div_total, feedback_factor;
	u32 i, ret;

	feedback_factor = hdmi_feedback_factor(mhdp);

	char_freq = pixel_freq * feedback_factor / 1000;

	DRM_INFO("Pixel clock: %d KHz, character clock: %d, bpc is %0d-bit.\n",
	     pixel_freq, char_freq, mhdp->video_info.color_depth);

	/* Get right row from the ctrl_table table.
	 * Check if 'pixel_freq_khz' value matches the PIXEL_CLK_FREQ column.
	 * Consider only the rows with FEEDBACK_FACTOR column matching feedback_factor. */
	for (i = 0; i < ARRAY_SIZE(imx8mq_ctrl_table); i++) {
		if (feedback_factor == imx8mq_ctrl_table[i].feedback_factor &&
				pixel_freq == imx8mq_ctrl_table[i].pixel_clk_freq_min) {
			p_ctrl_table = &imx8mq_ctrl_table[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(imx8mq_ctrl_table)) {
		DRM_WARN("Pixel clk (%d KHz) not supported, color depth (%0d-bit)\n",
		     pixel_freq, mhdp->video_info.color_depth);
		return 0;
	}

	div_total = p_ctrl_table->pll_fb_div_total;
	vco_freq = refclk_freq_khz * div_total / p_ctrl_table->cmnda_pll0_ip_div;

	/* Get right row from the imx8mq_pll_table table.
	 * Check if vco_freq_khz and feedback_div_total
	 * column matching with imx8mq_pll_table. */
	for (i = 0; i < ARRAY_SIZE(imx8mq_pll_table); i++) {
		if (vco_freq == imx8mq_pll_table[i].vco_freq_min &&
				div_total == imx8mq_pll_table[i].feedback_div_total) {
			p_pll_table = &imx8mq_pll_table[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(imx8mq_pll_table)) {
		DRM_WARN("VCO (%d KHz) not supported\n", vco_freq);
		return 0;
	}
	DRM_INFO("VCO frequency is %d KHz\n", vco_freq);

	ret = hdmi_phy_config(mhdp, p_ctrl_table, p_pll_table, pclk_in);
	if (ret == false)
		return 0;

	return char_freq;
}

static int hdmi_phy_cfg_ss28fdsoi(struct cdns_mhdp_device *mhdp,
				struct drm_display_mode *mode)
{
	const struct hdmi_ctrl *p_ctrl_table;
	const struct hdmi_pll_tuning *p_pll_table;
	const u8 pclk_in = true;
	u32 pixel_freq = mode->clock;
	u32 vco_freq, char_freq;
	u32 div_total, feedback_factor;
	u32 ret, i;

	feedback_factor = hdmi_feedback_factor(mhdp);

	char_freq = pixel_freq * feedback_factor / 1000;

	DRM_INFO("Pixel clock: %d KHz, character clock: %d, bpc is %0d-bit.\n",
	     pixel_freq, char_freq, mhdp->video_info.color_depth);

	/* Get right row from the ctrl_table table.
	 * Check if 'pixel_freq_khz' value matches the PIXEL_CLK_FREQ column.
	 * Consider only the rows with FEEDBACK_FACTOR column matching feedback_factor. */
	for (i = 0; i < ARRAY_SIZE(imx8qm_ctrl_table); i++) {
		if (feedback_factor == imx8qm_ctrl_table[i].feedback_factor &&
				pixel_freq >= imx8qm_ctrl_table[i].pixel_clk_freq_min &&
				pixel_freq <= imx8qm_ctrl_table[i].pixel_clk_freq_max) {
			p_ctrl_table = &imx8qm_ctrl_table[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(imx8qm_ctrl_table)) {
		DRM_WARN("Pixel clk (%d KHz) not supported, color depth (%0d-bit)\n",
		     pixel_freq, mhdp->video_info.color_depth);
		return 0;
	}

	div_total = p_ctrl_table->pll_fb_div_total;
	vco_freq = pixel_freq * div_total / p_ctrl_table->cmnda_pll0_ip_div;

	/* Get right row from the imx8mq_pll_table table.
	 * Check if vco_freq_khz and feedback_div_total
	 * column matching with imx8mq_pll_table. */
	for (i = 0; i < ARRAY_SIZE(imx8qm_pll_table); i++) {
		if (vco_freq >= imx8qm_pll_table[i].vco_freq_min &&
				vco_freq < imx8qm_pll_table[i].vco_freq_max &&
				div_total == imx8qm_pll_table[i].feedback_div_total) {
			p_pll_table = &imx8qm_pll_table[i];
			break;
		}
	}
	if (i == ARRAY_SIZE(imx8qm_pll_table)) {
		DRM_WARN("VCO (%d KHz) not supported\n", vco_freq);
		return 0;
	}
	DRM_INFO("VCO frequency is %d KHz\n", vco_freq);

	ret = hdmi_phy_config(mhdp, p_ctrl_table, p_pll_table, pclk_in);
	if (ret == false)
		return 0;

	return char_freq;
}

static int hdmi_phy_power_up(struct cdns_mhdp_device *mhdp)
{
	u32 val, i;

	/* set Power State to A2 */
	cdns_phy_reg_write(mhdp, PHY_HDP_MODE_CTRL, 0x0004);

	cdns_phy_reg_write(mhdp, TX_DIAG_ACYA_0, 1);
	cdns_phy_reg_write(mhdp, TX_DIAG_ACYA_1, 1);
	cdns_phy_reg_write(mhdp, TX_DIAG_ACYA_2, 1);
	cdns_phy_reg_write(mhdp, TX_DIAG_ACYA_3, 1);

	/* Wait for Power State A2 Ack */
	for (i = 0; i < 10; i++) {
		val = cdns_phy_reg_read(mhdp, PHY_HDP_MODE_CTRL);
		if (val & (1 << 6))
			break;
		msleep(20);
	}
	if (i == 10) {
		dev_err(mhdp->dev, "Wait A2 Ack failed\n");
		return -1;
	}

	/* Power up ARC */
	hdmi_arc_config(mhdp);

	/* Configure PHY in A0 mode (PHY must be in the A0 power
	 * state in order to transmit data)
	 */
	//cdns_phy_reg_write(mhdp, PHY_HDP_MODE_CTRL, 0x0101); //imx8mq
	cdns_phy_reg_write(mhdp, PHY_HDP_MODE_CTRL, 0x0001);

	/* Wait for Power State A0 Ack */
	for (i = 0; i < 10; i++) {
		val = cdns_phy_reg_read(mhdp, PHY_HDP_MODE_CTRL);
		if (val & (1 << 4))
			break;
		msleep(20);
	}
	if (i == 10) {
		dev_err(mhdp->dev, "Wait A0 Ack failed\n");
		return -1;
	}
	return 0;
}

bool cdns_hdmi_phy_video_valid_imx8mq(struct cdns_mhdp_device *mhdp)
{
	u32 rate = mhdp->valid_mode->clock;
	int i;

	for (i = 0; i < ARRAY_SIZE(imx8mq_ctrl_table); i++)
			if(rate == imx8mq_ctrl_table[i].pixel_clk_freq_min)
				return true;
	return false;
}

int cdns_hdmi_phy_set_imx8mq(struct cdns_mhdp_device *mhdp)
{
	struct drm_display_mode *mode = &mhdp->mode;
	int ret;

	/* Check HDMI FW alive before HDMI PHY init */
	ret = cdns_mhdp_check_alive(mhdp);
	if (ret == false) {
		DRM_ERROR("NO HDMI FW running\n");
		return -ENXIO;
	}

	/* Configure PHY */
	mhdp->hdmi.char_rate = hdmi_phy_cfg_t28hpc(mhdp, mode);
	if (mhdp->hdmi.char_rate == 0) {
		DRM_ERROR("failed to set phy pclock\n");
		return -EINVAL;
	}

	ret = hdmi_phy_power_up(mhdp);
	if (ret < 0)
		return ret;

	hdmi_phy_set_vswing(mhdp);

	return true;
}

bool cdns_hdmi_phy_video_valid_imx8qm(struct cdns_mhdp_device *mhdp)
{
	u32 rate = mhdp->valid_mode->clock;
	int i;

	for (i = 0; i < ARRAY_SIZE(imx8qm_ctrl_table); i++)
			if(rate >= imx8qm_ctrl_table[i].pixel_clk_freq_min &&
				rate <= imx8qm_ctrl_table[i].pixel_clk_freq_max)
				return true;
	return false;
}

int cdns_hdmi_phy_set_imx8qm(struct cdns_mhdp_device *mhdp)
{
	struct drm_display_mode *mode = &mhdp->mode;
	int ret;

	/* Check HDMI FW alive before HDMI PHY init */
	ret = cdns_mhdp_check_alive(mhdp);
	if (ret == false) {
		DRM_ERROR("NO HDMI FW running\n");
		return -ENXIO;
	}

	/* Configure PHY */
	mhdp->hdmi.char_rate = hdmi_phy_cfg_ss28fdsoi(mhdp, mode);
	if (mhdp->hdmi.char_rate == 0) {
		DRM_ERROR("failed to set phy pclock\n");
		return -EINVAL;
	}

	ret = hdmi_phy_power_up(mhdp);
	if (ret < 0)
		return ret;

	hdmi_phy_set_vswing(mhdp);

	return true;
}
