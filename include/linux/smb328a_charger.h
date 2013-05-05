/*
 *  Copyright (C) 2009 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SMB328A_CHARGER_H_
#define __SMB328A_CHARGER_H_

/* Charge current */
/* Argument value of "set_charge_current" */
enum smb328a_current_t {
	SMB328A_CURR_500 = 0,
	SMB328A_CURR_600,
	SMB328A_CURR_700,
	SMB328A_CURR_800,
	SMB328A_CURR_900,
	SMB328A_CURR_1000,
	SMB328A_CURR_1100,
	SMB328A_CURR_1200,
};


/* AC input current limit */
enum smb328a_ac_limit_t {
	SMB328A_LIMIT_450 = 0,
	SMB328A_LIMIT_600,
	SMB328A_LIMIT_700,
	SMB328A_LIMIT_800,
	SMB328A_LIMIT_900,
	SMB328A_LIMIT_1000,
	SMB328A_LIMIT_1100,
	SMB328A_LIMIT_1200,
};


/* Charge completion termination current */
enum smb328a_term_curr_t {
	SMB328A_TERM_25,
	SMB328A_TERM_50,
	SMB328A_TERM_75,
	SMB328A_TERM_100,
	SMB328A_TERM_125,
	SMB328A_TERM_150,
	SMB328A_TERM_175,
	SMB328A_TERM_200,
};


struct smb328a_platform_data {
	int (*start_charging)(enum smb328a_current_t curr);
	int (*stop_charging)(void);
	int (*get_vbus_status)(void);
        int chg_curr_ta;
        int chg_curr_ta_event;
        int chg_curr_usb;
        int chg_ac_limit_ta;
        int chg_ac_limit_usb;
        int chg_term_curr;
	/* Tx functions to host */
	void (*tx_charge_done)(void);
};
#endif
