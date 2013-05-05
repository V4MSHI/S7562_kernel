/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_NT35510.h"

/* NT35510 config for BOE panel */

static char nt35510_boe_display1[] = {
	0xF0,
	0x55, 0xAA, 0x52, 0x08,
        0x00
};

static char nt35510_boe_display2[] = {
	0xB1,
	0x4C, 0x04
};


static char nt35510_boe_display3[] = {
	0x36,
	0x02
};

static char nt35510_boe_display4[] = {
	0xB6,
	0x0A
};

static char nt35510_boe_display5[] = {
	0xB7,
	0x00, 0x00
};


static char nt35510_boe_display6[] = {
	0xB8,
	0x01, 0x05, 0x05, 0x05
};


static char nt35510_boe_display7[] = {
	0xBA,
	0x01
};

static char nt35510_boe_display8[] = {
	0xBD,
	0x01, 0x84, 0x07, 0x32,
	0x00
};

static char nt35510_boe_display9[] = {
	0xBE,
	0x01, 0x84, 0x07, 0x31,
	0x00
};

static char nt35510_boe_display10[] = {
	0xBF,
	0x01, 0x84, 0x07, 0x31,
	0x00
};

static char nt35510_boe_display11[] = {
	0xCC,
	0x03, 0x00, 0x00
};

static char nt35510_boe_display12[] = {
	0x35,
	0x00
};


/* Power settings sequence */

static char nt35510_boe_power1[] = {
	0xF0,
	0x55, 0xAA, 0x52, 0x08,
	0x01
};

static char nt35510_boe_power2[] = {
	0xB0,
	0x09, 0x09, 0x09
};

static char nt35510_boe_power3[] = {
	0xB6,
	0x34, 0x34, 0x34
};

static char nt35510_boe_power4[] = {
	0xB1,
	0x09, 0x09, 0x09
};

static char nt35510_boe_power5[] = {
	0xB7,
	0x24, 0x24, 0x24
};

static char nt35510_boe_power6[] = {
	0xB3,
	0x05, 0x05, 0x05
};

static char nt35510_boe_power7[] = {
	0xB9,
	0x24, 0x24, 0x24
};

static char nt35510_boe_power8[] = {
	0xBF,
	0x01
};


static char nt35510_boe_power9[] = {
	0xB5,
	0x0B, 0x0B, 0x0B
};

static char nt35510_boe_power10[] = {
	0xBA,
	0x24, 0x24, 0x24
};

static char nt35510_boe_power11[] = {
	0xBC,
	0x00, 0xA3, 0x00
};

static char nt35510_boe_power12[] = {
	0xBD,
	0x00, 0xA3, 0x00
};

static char nt35510_boe_eng1[] = {
	0xFF,
	0xAA, 0x55, 0x25, 0x01
};


static char nt35510_boe_eng2[] = {
	0xFC,
	0x06, 0xA0, 0x24, 0x00,
	0x3C, 0x00, 0x04, 0x80,
	0x10
};


static char nt35510_boe_eng3[] = {
	0xF8,
	0x01, 0x02, 0x00, 0x20, 0x33,
	0x13, 0x00, 0x40, 0x00, 0x00,
	0x23, 0x02, 0x19, 0xC8, 0x00,
	0x00,0x11
};


/* Gamma settings */

static char nt35510_boe_gamma1[] = {
	0xD1,
        0x00, 0x01, 0x00, 0x43, 0x00, 0x6B, 0x00, 0x87,
        0x00, 0xA3, 0x00, 0xCE, 0x00, 0xF1, 0x01, 0x27,
        0x01, 0x53, 0x01, 0x98, 0x01, 0xCE, 0x02, 0x22,
        0x02, 0x83, 0x02, 0x78, 0x02, 0x9E, 0x02, 0xDD,
        0x03, 0x00, 0x03, 0x2E, 0x03, 0x54, 0x03, 0x7F,
        0x03, 0x95, 0x03, 0xB3, 0x03, 0xC2, 0x03, 0xE1,
        0x03, 0xF1, 0x03, 0xFE
};

static char nt35510_boe_gamma2[] = {
	0xD2,
        0x00, 0x01, 0x00, 0x43, 0x00, 0x6B, 0x00, 0x87,
        0x00, 0xA3, 0x00, 0xCE, 0x00, 0xF1, 0x01, 0x27,
        0x01, 0x53, 0x01, 0x98, 0x01, 0xCE, 0x02, 0x22,
        0x02, 0x83, 0x02, 0x78, 0x02, 0x9E, 0x02, 0xDD,
        0x03, 0x00, 0x03, 0x2E, 0x03, 0x54, 0x03, 0x7F,
        0x03, 0x95, 0x03, 0xB3, 0x03, 0xC2, 0x03, 0xE1,
        0x03, 0xF1, 0x03, 0xFE
};

static char nt35510_boe_gamma3[] = {
	0xD3,
        0x00, 0x01, 0x00, 0x43, 0x00, 0x6B, 0x00, 0x87,
        0x00, 0xA3, 0x00, 0xCE, 0x00, 0xF1, 0x01, 0x27,
        0x01, 0x53, 0x01, 0x98, 0x01, 0xCE, 0x02, 0x22,
        0x02, 0x83, 0x02, 0x78, 0x02, 0x9E, 0x02, 0xDD,
        0x03, 0x00, 0x03, 0x2E, 0x03, 0x54, 0x03, 0x7F,
        0x03, 0x95, 0x03, 0xB3, 0x03, 0xC2, 0x03, 0xE1,
        0x03, 0xF1, 0x03, 0xFE
};


static char nt35510_boe_gamma4[] = {
	0xD4,
        0x00, 0x01, 0x00, 0x43, 0x00, 0x6B, 0x00, 0x87,
        0x00, 0xA3, 0x00, 0xCE, 0x00, 0xF1, 0x01, 0x27,
        0x01, 0x53, 0x01, 0x98, 0x01, 0xCE, 0x02, 0x22,
        0x02, 0x43, 0x02, 0x50, 0x02, 0x9E, 0x02, 0xDD,
        0x03, 0x00, 0x03, 0x2E, 0x03, 0x54, 0x03, 0x7F,
        0x03, 0x95, 0x03, 0xB3, 0x03, 0xC2, 0x03, 0xE1,
        0x03, 0xF1, 0x03, 0xFE
};

static char nt35510_boe_gamma5[] = {
	0xD5,
        0x00, 0x01, 0x00, 0x43, 0x00, 0x6B, 0x00, 0x87,
        0x00, 0xA3, 0x00, 0xCE, 0x00, 0xF1, 0x01, 0x27,
        0x01, 0x53, 0x01, 0x98, 0x01, 0xCE, 0x02, 0x22,
        0x02, 0x43, 0x02, 0x50, 0x02, 0x9E, 0x02, 0xDD,
        0x03, 0x00, 0x03, 0x2E, 0x03, 0x54, 0x03, 0x7F,
        0x03, 0x95, 0x03, 0xB3, 0x03, 0xC2, 0x03, 0xE1,
        0x03, 0xF1, 0x03, 0xFE
};


static char nt35510_boe_gamma6[] = {
	0xD6,
        0x00, 0x01, 0x00, 0x43, 0x00, 0x6B, 0x00, 0x87,
        0x00, 0xA3, 0x00, 0xCE, 0x00, 0xF1, 0x01, 0x27,
        0x01, 0x53, 0x01, 0x98, 0x01, 0xCE, 0x02, 0x22,
        0x02, 0x43, 0x02, 0x50, 0x02, 0x9E, 0x02, 0xDD,
        0x03, 0x00, 0x03, 0x2E, 0x03, 0x54, 0x03, 0x7F,
        0x03, 0x95, 0x03, 0xB3, 0x03, 0xC2, 0x03, 0xE1,
        0x03, 0xF1, 0x03, 0xFE
};

static char nt35510_sw_reset[] = { 0x01, 0x00 };

static struct dsi_cmd_desc nt35510_boe_cmd_prepare_panel[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 20,
	sizeof(nt35510_sw_reset), nt35510_sw_reset}
	,
	{DTYPE_DCS_WRITE, 1, 0, 0, NT35510_CMD_DELAY_120MS,
	sizeof(nt35510_exit_sleep), nt35510_exit_sleep}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_eng1), nt35510_boe_eng1}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_eng2), nt35510_boe_eng2}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_eng3), nt35510_boe_eng3}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_power1), nt35510_boe_power1}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_power2), nt35510_boe_power2}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_power3), nt35510_boe_power3}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_power4), nt35510_boe_power4}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_power5), nt35510_boe_power5}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_power6), nt35510_boe_power6}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_power7), nt35510_boe_power7}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_power8), nt35510_boe_power8}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_power9), nt35510_boe_power9}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_power10), nt35510_boe_power10}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_power11), nt35510_boe_power11}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, 20,
	sizeof(nt35510_boe_power12), nt35510_boe_power12}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE,
	sizeof(nt35510_boe_gamma1), nt35510_boe_gamma1}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE,
	sizeof(nt35510_boe_gamma2), nt35510_boe_gamma2}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE,
	sizeof(nt35510_boe_gamma3), nt35510_boe_gamma3}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE,
	sizeof(nt35510_boe_gamma4), nt35510_boe_gamma4}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE,
	sizeof(nt35510_boe_gamma5), nt35510_boe_gamma5}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE,
	sizeof(nt35510_boe_gamma6), nt35510_boe_gamma6}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_display1), nt35510_boe_display1}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_display2), nt35510_boe_display2}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_display3), nt35510_boe_display3}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_display4), nt35510_boe_display4}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_display5), nt35510_boe_display5}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_display6), nt35510_boe_display6}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_display7), nt35510_boe_display7}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_display8), nt35510_boe_display8}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_display9), nt35510_boe_display9}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_display10), nt35510_boe_display10}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE,
	sizeof(nt35510_boe_display11), nt35510_boe_display11}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(nt35510_boe_display12), nt35510_boe_display12}
	,
};


static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl dsi_cmd_mode_phy_db = {
	/* DSI Bit Clock at 500 MHz, 2 lane, RGB888 */
	/* regulator */
	{0x03, 0x01, 0x01, 0x00},
	/* timing   */
	{0xb9, 0x8e, 0x1f, 0x00, 0x98, 0x9c, 0x22, 0x90,
	0x18, 0x03, 0x04},
	/* phy ctrl */
	{0x7f, 0x00, 0x00, 0x00},
	/* strength */
	{0xbb, 0x02, 0x06, 0x00},
	/* pll control */
	{0x01, 0xec, 0x31, 0xd2, 0x00, 0x40, 0x37, 0x62,
	0x01, 0x0f, 0x07,
	0x05, 0x14, 0x03, 0x0, 0x0, 0x0, 0x20, 0x0, 0x02, 0x0},
};

static int mipi_cmd_nt35510_wvga_pt_init(void)
{
	int ret;

	if (msm_fb_detect_client("mipi_cmd_nt35510_boe_wvga"))
		return 0;

	printk(KERN_INFO "NT35510: detected BOE cmd panel.\n");

	pinfo.xres = 480;
	pinfo.yres = 800;
	pinfo.type = MIPI_CMD_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 100;
	pinfo.lcdc.h_front_porch = 100;
	pinfo.lcdc.h_pulse_width = 8;
	pinfo.lcdc.v_back_porch = 20;
	pinfo.lcdc.v_front_porch = 20;
	pinfo.lcdc.v_pulse_width = 1;

	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 100;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;

	pinfo.clk_rate = 499000000;

	pinfo.lcd.vsync_enable = TRUE;
	pinfo.lcd.hw_vsync_mode = TRUE;
	pinfo.lcd.refx100 = 6000; /* adjust refx100 to prevent tearing */
	pinfo.lcd.v_back_porch = 7;
	pinfo.lcd.v_front_porch = 50;
	pinfo.lcd.v_pulse_width = 1;

	pinfo.mipi.mode = DSI_CMD_MODE;
	pinfo.mipi.dst_format = DSI_CMD_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.t_clk_post = 0x20;
	pinfo.mipi.t_clk_pre = 0x2F;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.te_sel = 1; /* TE from vsync gpio */
	pinfo.mipi.interleave_max = 1;
	pinfo.mipi.insert_dcs_cmd = TRUE;
	pinfo.mipi.wr_mem_continue = 0x3c;
	pinfo.mipi.wr_mem_start = 0x2c;
	pinfo.mipi.dsi_phy_db = &dsi_cmd_mode_phy_db;
	pinfo.mipi.tx_eot_append = 0x01;
	pinfo.mipi.rx_eot_ignore = 0x0;
	pinfo.mipi.dlane_swap = 0x01;

	ret = mipi_nt35510_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_WVGA_PT,
                                                nt35510_boe_cmd_prepare_panel,
                                                ARRAY_SIZE(nt35510_boe_cmd_prepare_panel));
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_cmd_nt35510_wvga_pt_init);
