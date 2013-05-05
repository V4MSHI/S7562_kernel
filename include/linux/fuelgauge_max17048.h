/*
 *  Copyright (C) 2009 Samsung Electronics
 *  Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __MAX17048_BATTERY_H_
#define __MAX17048_BATTERY_H_

struct max17048_platform_data {

int (*battery_online) (void);

int (*charger_online) (void);

int (*charger_enable) (void);

int (*low_batt_cb) (void);

int (*check_batt_type) (void);

int (*power_supply_register) (struct device *parent,
struct power_supply *psy);

void (*power_supply_unregister) (struct power_supply *psy);

u16 rcomp_value;

};


#define BATT_TYPE_NORMAL		0 /* 4.2V battery */
#define BATT_TYPE_JAGUAR		1 /* 4.35V, new active battery */
#define BATT_TYPE_D2_HIGH		2 /* 4.35V battery */
#define BATT_TYPE_D2_ACTIVE		3 /* 4.35V, new active battery */
#define BATT_TYPE_AEGIS2		4 /* 4.35V, new active battery */

/* fuelgauge tuning */
/* SOC accuracy depends on RCOMP and Adjusted SOC Method(below values) */
/* you should fix these values for your MODEL */
#if defined(CONFIG_MACH_KYLE)
#define EMPTY_COND_SOC		100
#define EMPTY_SOC		30
#define FULL_SOC_DEFAULT	9900
#define FULL_SOC_LOW		9300
#define FULL_SOC_HIGH		9650
#define FULL_KEEP_SOC		0
#define RCOMP0_TEMP	20 /* 'C */
#else	/*
 */
#define EMPTY_COND_SOC		100
#define EMPTY_SOC		20
#define FULL_SOC_DEFAULT	9400
#define FULL_SOC_LOW		9300
#define FULL_SOC_HIGH		9650
#define FULL_KEEP_SOC		0
#define RCOMP0_TEMP	20 /* 'C */
#endif	/*
 */

#endif	/*
 */
