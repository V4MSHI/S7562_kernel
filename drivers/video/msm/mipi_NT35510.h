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

#ifndef MIPI_NT35510_H
#define MIPI_NT35510_H

#include "mipi_dsi.h"

/* in milliseconds */
#define NT35510_RESET_SLEEP_OUT_DELAY 120
#define NT35510_RESET_SLEEP_IN_DELAY 5
#define NT35510_SLEEP_IN_DELAY 10
#define NT35510_SLEEP_OUT_DELAY 10
#define NT35510_CMD_SETTLE 	0
#define NT35510_SETTING_SETTLE 0
#define NT35510_CMD_DELAY_10MS	10
#define NT35510_CMD_DELAY_120MS	120


/* in microseconds */
#define NT35510_EXIT_DEEP_STANDBY 3001
#define NT35510_HOLD_RESET 11

extern unsigned int board_hw_revision;

int mipi_nt35510_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel,
					struct dsi_cmd_desc *panel_prepare,
					int panel_prepare_length);


/***** common commands ******/
static char nt35510_read_display_power_mode[] = { 0x0A, 0x00 };
static char nt35510_deep_standby_on[] = { 0x4F, 0x01 };

static char nt35510_exit_sleep[] = { 0x11, 0x00 };
static char nt35510_display_on[] = { 0x29, 0x00 };
static char nt35510_display_off[] = { 0x28, 0x00 };
static char nt35510_enter_sleep[] = { 0x10, 0x00 };

static struct dsi_cmd_desc nt35510_deep_standby_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, NT35510_CMD_SETTLE,
	sizeof(nt35510_deep_standby_on), nt35510_deep_standby_on}
};

static struct dsi_cmd_desc nt35510_read_display_power_mode_cmds[] = {
	{DTYPE_DCS_READ, 1, 0, 0, NT35510_CMD_SETTLE,
		sizeof(nt35510_read_display_power_mode), nt35510_read_display_power_mode}
};

static struct dsi_cmd_desc nt35510_display_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, NT35510_CMD_DELAY_10MS,
	sizeof(nt35510_display_on), nt35510_display_on}
	,
};

static struct dsi_cmd_desc nt35510_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, NT35510_CMD_DELAY_10MS,
	sizeof(nt35510_display_off), nt35510_display_off}
	,
	{DTYPE_DCS_WRITE, 1, 0, 0,  NT35510_CMD_DELAY_120MS,
	sizeof(nt35510_enter_sleep), nt35510_enter_sleep}
};

static char nt35510_boe_dummy_data[] = {
	0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,	
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
};

static struct dsi_cmd_desc nt35510_boe_cmd_dummy[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, NT35510_CMD_SETTLE,
	sizeof(nt35510_boe_dummy_data), nt35510_boe_dummy_data}
};

#endif  /* MIPI_NT35510_H */
