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
 *
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_NT35510.h"


/* NT35510 config for HYDIS panel */
/* Display parameter settings for video mode */
static char video0[] = {
	0xFF,
	0xAA, 0x55, 0x25, 0x01
};

static char video1[] = {
	0xF3,
	0x00, 0x32, 0x00, 0x38,
	0x31, 0x08, 0x11, 0x00
};

static char video2[] = {
	0xF0,
	0x55, 0xAA, 0x52, 0x08,
	0x00
};

static char video3[] = {
	0xB1,
	0x7C, 0x04
};

static char video4[] = {
	0x36,
	0x02
};

static char video5[] = {
	0xB6,
	0x03
};

static char video6[] = {
	0xB7,
	0x70, 0x70
};

static char video7[] = {
	0xB8,
	0x00, 0x06, 0x06, 0x06
};

static char video8[] = {
	0xBC,
	0x00, 0x00, 0x00
};

static char video9[] = {
	0xBD,
	0x01, 0x84, 0x06, 0x50,
	0x00
};

static char video10[] = {
	0xCC,
	0x03, 0x2A, 0x06
};

/* Power settings sequence */

static char video11[] = {
	0xF0,
	0x55, 0xAA, 0x52, 0x08,
	0x01
};

static char video12[] = {
	0xB0,
	0x05, 0x05, 0x05
};

static char video13[] = {
	0xB1,
	0x05, 0x05, 0x05
};

static char video14[] = {
	0xB2,
	0x03, 0x03, 0x03
};

static char video15[] = {
	0xB8,
	0x24, 0x24, 0x24
};

static char video16[] = {
	0xB3,
	0x0A, 0x0A, 0x0A
};

static char video17[] = {
	0xB9,
	0x24, 0x24, 0x24
};

static char video18[] = {
	0xBF,
	0x01
};

static char video19[] = {
	0xB5,
	0x08, 0x08, 0x08
};

static char video20[] = {
	0xB4,
	0x2D, 0x2D, 0x2D
};

static char video21[] = {
	0xBC,
	0x00, 0x50, 0x00
};

static char video22[] = {
	0xBD,
	0x00, 0x60, 0x00
};

static char video23[] = {
	0xCE,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00
};

/* Gamma settings */
static char video24[] = {
	0xD1,
	0x00, 0x37, 0x00, 0x71, 0x00, 0xA9, 0x00, 0xCD,
	0x00, 0xE4, 0x01, 0x16, 0x01, 0x5F, 0x01, 0x98,
	0x01, 0xBA, 0x01, 0xDE, 0x02, 0x0F, 0x02, 0x44,
	0x02, 0x7F, 0x02, 0x81, 0x02, 0xBA, 0x02, 0xE2,
	0x03, 0x0C, 0x03, 0x34, 0x03, 0x4F, 0x03, 0x73,
	0x03, 0x77, 0x03, 0x94, 0x03, 0x9E, 0x03, 0xAC,
	0x03, 0xBD, 0x03, 0xF1
};

static char video25[] = {
	0xD2,
	0x00, 0x37, 0x00, 0x71, 0x00, 0xA9, 0x00, 0xCD,
	0x00, 0xE4, 0x01, 0x16, 0x01, 0x5F, 0x01, 0x98,
	0x01, 0xBA, 0x01, 0xDE, 0x02, 0x0F, 0x02, 0x44,
	0x02, 0x7F, 0x02, 0x81, 0x02, 0xBA, 0x02, 0xE2,
	0x03, 0x0C, 0x03, 0x34, 0x03, 0x4F, 0x03, 0x73,
	0x03, 0x77, 0x03, 0x94, 0x03, 0x9E, 0x03, 0xAC,
	0x03, 0xBD, 0x03, 0xF1
};

static char video26[] = {
	0xD3,
	0x00, 0x37, 0x00, 0x71, 0x00, 0xA9, 0x00, 0xCD,
	0x00, 0xE4, 0x01, 0x16, 0x01, 0x5F, 0x01, 0x98,
	0x01, 0xBA, 0x01, 0xDE, 0x02, 0x0F, 0x02, 0x44,
	0x02, 0x7F, 0x02, 0x81, 0x02, 0xBA, 0x02, 0xE2,
	0x03, 0x0C, 0x03, 0x34, 0x03, 0x4F, 0x03, 0x73,
	0x03, 0x77, 0x03, 0x94, 0x03, 0x9E, 0x03, 0xAC,
	0x03, 0xBD, 0x03, 0xF1
};

static char video27[] = {
	0xD4,
	0x00, 0x37, 0x00, 0x46, 0x00, 0x81, 0x00, 0xA8,
	0x00, 0xCF, 0x01, 0x1A, 0x01, 0x37, 0x01, 0x5F,
	0x01, 0x79, 0x01, 0xBE, 0x01, 0xE2, 0x02, 0x2C,
	0x02, 0x5E, 0x02, 0x60, 0x02, 0x94, 0x02, 0xD0,
	0x02, 0xF2, 0x03, 0x21, 0x03, 0x40, 0x03, 0x6B,
	0x03, 0x77, 0x03, 0x94, 0x03, 0x9E, 0x03, 0xAC,
	0x03, 0xBD, 0x03, 0xF1
};

static char video28[] = {
	0xD5,
	0x00, 0x37, 0x00, 0x46, 0x00, 0x81, 0x00, 0xA8,
	0x00, 0xCF, 0x01, 0x1A, 0x01, 0x37, 0x01, 0x5F,
	0x01, 0x79, 0x01, 0xBE, 0x01, 0xE2, 0x02, 0x2C,
	0x02, 0x5E, 0x02, 0x60, 0x02, 0x94, 0x02, 0xD0,
	0x02, 0xF2, 0x03, 0x21, 0x03, 0x40, 0x03, 0x6B,
	0x03, 0x77, 0x03, 0x94, 0x03, 0x9E, 0x03, 0xAC,
	0x03, 0xBD, 0x03, 0xF1
};

static char video29[] = {
	0xD6,
	0x00, 0x37, 0x00, 0x46, 0x00, 0x81, 0x00, 0xA8,
	0x00, 0xCF, 0x01, 0x1A, 0x01, 0x37, 0x01, 0x5F,
	0x01, 0x79, 0x01, 0xBE, 0x01, 0xE2, 0x02, 0x2C,
	0x02, 0x5E, 0x02, 0x60, 0x02, 0x94, 0x02, 0xD0,
	0x02, 0xF2, 0x03, 0x21, 0x03, 0x40, 0x03, 0x6B,
	0x03, 0x77, 0x03, 0x94, 0x03, 0x9E, 0x03, 0xAC,
	0x03, 0xBD, 0x03, 0xF1
};

static struct dsi_cmd_desc nt35510_hydis_video_prepare_panel[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, NT35510_SLEEP_OUT_DELAY,
	 sizeof(nt35510_exit_sleep), nt35510_exit_sleep}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video0), video0}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video1), video1}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video2), video2}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video3), video3}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video4), video4}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video5), video5}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video6), video6}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video7), video7}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video8), video8}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video9), video9}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video10), video10}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video11), video11}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video12), video12}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video13), video13}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video14), video14}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video15), video15}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video16), video16}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video17), video17}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video18), video18}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video19), video19}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video20), video20}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video21), video21}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video22), video22}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video23), video23}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video24), video24}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video25), video25}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video26), video26}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video27), video27}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video28), video28}
	,
	{DTYPE_GEN_LWRITE, 1, 0, 0, NT35510_CMD_SETTLE, sizeof(video29), video29}
	,
};

static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db = {
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
	{0x00, 0xec, 0x31, 0xd2, 0x00, 0x40, 0x37, 0x62,
	0x01, 0x0f, 0x07,
	0x05, 0x14, 0x03, 0x0, 0x0, 0x0, 0x20, 0x0, 0x02, 0x0},
};

static int mipi_video_nt35510_wvga_pt_init(void)
{
	int ret;

	if (msm_fb_detect_client("mipi_video_nt35510_hydis_wvga"))
		return 0;

	printk(KERN_INFO "NT35510: detected HYDIS video panel.\n");

	pinfo.xres = 480;
	pinfo.yres = 800;
	pinfo.type = MIPI_VIDEO_PANEL;
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
	/* number of dot_clk cycles HSYNC active edge is
	delayed from VSYNC active edge */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.clk_rate = 499000000;
	pinfo.bl_max = 100; /*16; CHECK THIS!!!*/
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	/* send HSA and HE following VS/VE packet */
	pinfo.mipi.pulse_mode_hsa_he = TRUE;
	pinfo.mipi.hfp_power_stop = TRUE; /* LP-11 during the HFP period */
	pinfo.mipi.hbp_power_stop = TRUE; /* LP-11 during the HBP period */
	pinfo.mipi.hsa_power_stop = TRUE; /* LP-11 during the HSA period */
	/* LP-11 or let Command Mode Engine send packets in
	HS or LP mode for the BLLP of the last line of a frame */
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	/* LP-11 or let Command Mode Engine send packets in
	HS or LP mode for packets sent during BLLP period */
	pinfo.mipi.bllp_power_stop = TRUE;

	pinfo.mipi.traffic_mode = DSI_BURST_MODE;
	pinfo.mipi.dst_format =  DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB; /* RGB */
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;

	pinfo.mipi.t_clk_post = 0x20;
	pinfo.mipi.t_clk_pre = 0x2f;

	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_NONE;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60; /* FIXME */

	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db;
	pinfo.mipi.dlane_swap = 0x01;
	/* append EOT at the end of data burst */
	pinfo.mipi.tx_eot_append = 0x01;

	ret = mipi_nt35510_device_register(&pinfo, MIPI_DSI_PRIM,
					   MIPI_DSI_PANEL_WVGA_PT,
					   nt35510_hydis_video_prepare_panel,
					   ARRAY_SIZE(nt35510_hydis_video_prepare_panel));

	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_video_nt35510_wvga_pt_init);
