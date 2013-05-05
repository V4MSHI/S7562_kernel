/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <mach/gpio.h>
#include <mach/pmic.h>
#include <mach/vreg.h>
#include "msm_fb.h"

#ifdef CONFIG_FB_MSM_TRY_MDDI_CATCH_LCDC_PRISM
#include "mddihosti.h"
#endif
#if defined(CONFIG_MACH_AMAZING_CDMA)
#define LCD_WORK_QUEUE
#endif

#include "lcdc_s6d_backlight.h"


#define LCDC_DEBUG

#ifdef LCDC_DEBUG
#define DPRINT(x...)	printk(KERN_ALERT "s6d16a0x_JENA " x)
#else
#define DPRINT(x...)
#endif

/*Header file content from lcdc_s6d05a1x01_new Start*/
struct setting_table {
	unsigned char command;
	unsigned char parameters;
	unsigned char parameter[35]; /* 31 > 90 > 45*/
	long wait;
};

#define PWRCTL          0xF4
#define SLPIN           0x10
#define SLPOUT          0x11
#define DISCTL          0xF2
#define VCMCTL          0xF5
#define SRCCTL          0xF6
#define COLMOD          0x3A
#define WRDISBV         0x51
#define PASSWD1         0xF0
#define PASSWD2         0xF1
#define IFCTL           0xF7
#define PANELCTL        0xF8
#define GAMMASEL        0xF9
#define PGAMMACTL       0xFA
#define NGAMMACTL       0xFB
#define DISPON          0x29
#define DISPOFF         0x28
#define BCMODE          0xC1
#define WRCTRLD         0x53
#define WRBLCTL         0xC3
#define MADCTL          0x36
#define CASET           0x2A
#define PASET           0x2B
#define TEON            0x35
#define MANPWRSEQ       0xF3
#define RAMWR           0x2C

static struct setting_table ifctl[] = {
{ 0xF7, 5, { 0x48, 0x88, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0x30, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0x40, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0x50, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0x60, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0x70, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0x80, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0x90, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0xA0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0xB0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0xC0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0xD0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0xE0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 },
{ 0xF7, 5, { 0x48, 0x88, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0 }

};

static struct setting_table power_on_setting_table_lsi[] = {
	{ PASSWD1,    2,  { 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
	{ PASSWD2,    2,  { 0x5A, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
	{ DISCTL,    19,  { 0x3B, 0x3A, 0x03, 0x04, 0x02, 0x08, 0x08, 0x00, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, 0x54, 0x08, 0x08, 0x08, 0x08, 0x00 },     0 },
	{ PWRCTL,    11,  { 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x6B, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
	{ VCMCTL,     9,  { 0x00, 0x47, 0x75, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
	{ COLMOD,     1,  { 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
	{ MADCTL,     1,  { 0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
	{ CASET,     4,  { 0x00, 0x00, 0x01, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
	{ PASET,     4,  { 0x00, 0x00, 0x01, 0xDF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
	{ SRCCTL,     6,  { 0x03, 0x00, 0x0B, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
	{ IFCTL,      5,  { 0x48, 0x01, 0xF0, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
	{ PANELCTL,   2,  { 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
	{ GAMMASEL,   1,  { 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
	{ PGAMMACTL, 16,  { 0x23, 0x00, 0x09, 0x1D, 0x1D, 0x25, 0x2B, 0x18, 0x21, 0x25, 0x26, 0x1C, 0x1B, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
	{ GAMMASEL,   1,  { 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
	{ PGAMMACTL, 16,  { 0x30, 0x10, 0x08, 0x1D, 0x20, 0x2A, 0x31, 0x10, 0x1B, 0x1A, 0x1F, 0x1C, 0x1B, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
	{ GAMMASEL,   1,  { 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
	{ PGAMMACTL, 16,  { 0x30, 0x10, 0x09, 0x1E, 0x27, 0x32, 0x39, 0x0A, 0x16, 0x17, 0x19, 0x1C, 0x1B, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
	{ SLPOUT,     0,  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  120 },
	{ DISPON,     0,  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },    0 },
	{ WRCTRLD,    1,  { 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
	{ WRBLCTL,    3,  { 0x00, 0x2F, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     0 },
};

static struct setting_table power_off_setting_table[] = {
/*	Display off*/
	{    0x28, 0, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 40 },

	{    0xDE, 1,  { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 120 },
};

static struct setting_table power_on_setting_table_sleep_out[] = {
/*	Sleep out */
   {0x11, 0, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }, 10 },
};

/*End of Header file content from lcdc_s6d05a1x01_new*/

/*#define RDDIDIF			0x04		/* 4 parameters*/
#define RDID1			0xDA		/* 2 parameters */
#define RDID2			0xDB		/* 2 parameters */
#define RDID3			0xDC		/* 2 parameters 8 */

#define POWER_ON_SETTINGS(a)	(int)(sizeof(a)/sizeof(struct setting_table))
#define POWER_OFF_SETTINGS	(int)(sizeof(power_off_setting_table)/sizeof(struct setting_table))

#if  defined(LCD_WORK_QUEUE)
static struct work_struct lcdc_s6d16a0x_work;
struct workqueue_struct *work_queue_s6d16a0x;
#endif

static int spi_cs;
static int spi_sclk;
static int spi_sdi;
static int spi_sdo;


static int lcd_reset;
extern struct class *sec_class;
struct s6d16a0x_state_type {
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
};

static struct s6d16a0x_state_type s6d16a0x_state = { 0 };
static struct msm_panel_common_pdata *lcdc_s6d16a0x_pdata;

static int lcd_prf;

struct device *sec_lcdtype_dev;
extern int board_hw_revision;
unsigned int lcd_Idtype;
EXPORT_SYMBOL(lcd_Idtype);


static DEFINE_SPINLOCK(lcd_ctrl_irq_lock);
static DEFINE_SEMAPHORE(backlight_sem);

#define DEFAULT_USLEEP	1

#if 1
static void read_ldi_register(u8 addr, u8 *buf, int count)
{
	long i, j;

	gpio_set_value(spi_cs, 1);
	udelay(DEFAULT_USLEEP);
	gpio_set_value(spi_sclk, 1);
	udelay(DEFAULT_USLEEP);

	/* Write Command */
	gpio_set_value(spi_cs, 0);
	udelay(DEFAULT_USLEEP);
	gpio_set_value(spi_sclk, 0);
	udelay(DEFAULT_USLEEP);
	gpio_set_value(spi_sdi, 0);
	udelay(DEFAULT_USLEEP);
	gpio_set_value(spi_sclk, 1);
	udelay(DEFAULT_USLEEP);

	for (i = 7; i >= 0; i--) {
		gpio_set_value(spi_sclk, 0);
		udelay(DEFAULT_USLEEP);
		if ((addr >> i) & 0x1)
			gpio_set_value(spi_sdi, 1);
		else
			gpio_set_value(spi_sdi, 0);
		udelay(DEFAULT_USLEEP);
		gpio_set_value(spi_sclk, 1);
		udelay(DEFAULT_USLEEP);
	}

	/*swith input */
	gpio_direction_input(spi_sdo);

	if (count > 1) {
		/*dummy clock cycle */
		gpio_set_value(spi_sclk, 0);
		udelay(DEFAULT_USLEEP);
		gpio_set_value(spi_sclk, 1);
		udelay(DEFAULT_USLEEP);
		}

	/* Read Parameter */
	if (count > 0) {
		for (j = 0; j < count; j++) {

			for (i = 7; i >= 0; i--) {
				gpio_set_value(spi_sclk, 0);
				udelay(DEFAULT_USLEEP);
				/* read bit */
				if (gpio_get_value(spi_sdo))
					buf[j] |= (0x1<<i);
				else
					buf[j] &= ~(0x1<<i);
				gpio_set_value(spi_sclk, 1);
				udelay(DEFAULT_USLEEP);
				}

		}
	}

	gpio_set_value(spi_cs, 1);
	udelay(DEFAULT_USLEEP);
	/*switch output */
	gpio_direction_output(spi_sdi, 0);
}
#endif

static void setting_table_write(struct setting_table *table)
{
	long i, j;
	unsigned long irqflags;
	spin_lock_irqsave(&lcd_ctrl_irq_lock, irqflags);

	gpio_set_value(spi_cs, 1);
	udelay(DEFAULT_USLEEP);
	gpio_set_value(spi_sclk, 1);
	udelay(DEFAULT_USLEEP);

	/* Write Command */
	gpio_set_value(spi_cs, 0);
	udelay(DEFAULT_USLEEP);
	gpio_set_value(spi_sclk, 0);
	udelay(DEFAULT_USLEEP);
	gpio_set_value(spi_sdi, 0);
	udelay(DEFAULT_USLEEP);
	gpio_set_value(spi_sclk, 1);
	udelay(DEFAULT_USLEEP);
	for (i = 7; i >= 0; i--) {
		gpio_set_value(spi_sclk, 0);
		udelay(DEFAULT_USLEEP);
		if ((table->command >> i) & 0x1)
			gpio_set_value(spi_sdi, 1);
		else
			gpio_set_value(spi_sdi, 0);
		udelay(DEFAULT_USLEEP);
		gpio_set_value(spi_sclk, 1);
		udelay(DEFAULT_USLEEP);
		}

	gpio_set_value(spi_cs, 1);
	udelay(DEFAULT_USLEEP);

	/* Write Parameter */
	if ((table->parameters) > 0) {
		for (j = 0; j < table->parameters; j++) {
			gpio_set_value(spi_cs, 0);
			udelay(DEFAULT_USLEEP);
			gpio_set_value(spi_sclk, 0);
			udelay(DEFAULT_USLEEP);
			gpio_set_value(spi_sdi, 1);
			udelay(DEFAULT_USLEEP);
			gpio_set_value(spi_sclk, 1);
			udelay(DEFAULT_USLEEP);
			for (i = 7; i >= 0; i--) {
				gpio_set_value(spi_sclk, 0);
				udelay(DEFAULT_USLEEP);
				if ((table->parameter[j] >> i) & 0x1)
					gpio_set_value(spi_sdi, 1);
				else
					gpio_set_value(spi_sdi, 0);
				udelay(DEFAULT_USLEEP);
				gpio_set_value(spi_sclk, 1);
				udelay(DEFAULT_USLEEP);
				}

				gpio_set_value(spi_cs, 1);
				udelay(DEFAULT_USLEEP);
		}
	}
	spin_unlock_irqrestore(&lcd_ctrl_irq_lock, irqflags);
	mdelay(table->wait);
}

static void spi_init(void)
{
	/* Set the output so that we dont disturb the slave device */
	gpio_set_value(spi_sclk, 0);
	gpio_set_value(spi_sdi, 0);

	/* Set the Chip Select De-asserted */
	gpio_set_value(spi_cs, 0);

}

#define VREG_ENABLE		1
#define VREG_DISABLE	0

static void s6d16a0x_vreg_config(int vreg_en)
{
	int rc;
	struct vreg *vreg_lcd = NULL;

	if (vreg_lcd == NULL) {
		vreg_lcd = vreg_get(NULL, "vlcd");

		if (IS_ERR(vreg_lcd)) {
			printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
				__func__, "vlcd4", PTR_ERR(vreg_lcd));
			return;
		}

		rc = vreg_set_level(vreg_lcd, 3000);
		if (rc) {
			printk(KERN_ERR "%s: LCD set_level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_en) {
		rc = vreg_enable(vreg_lcd);
		if (rc) {
			printk(KERN_ERR "%s: LCD enable failed (%d)\n",
				 __func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_lcd);
		if (rc) {
			printk(KERN_ERR "%s: LCD disable failed (%d)\n",
				 __func__, rc);
		}
	}
}

static void s6d16a0x_disp_powerup(void)
{
	DPRINT("start %s\n", __func__);

	if (!s6d16a0x_state.disp_powered_up && !s6d16a0x_state.display_on) {

		s6d16a0x_vreg_config(VREG_ENABLE);
		msleep(50);
		gpio_tlmm_config(GPIO_CFG(lcd_reset, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL
			, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(lcd_reset, 0);
		msleep(100);
		gpio_set_value(lcd_reset, 1);
		msleep(50);
		s6d16a0x_state.disp_powered_up = TRUE;
	}
}

static void s6d16a0x_disp_powerdown(void)
{
	DPRINT("start %s\n", __func__);

		gpio_tlmm_config(GPIO_CFG(lcd_reset, 0, GPIO_CFG_OUTPUT
					, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(lcd_reset, 0);

	s6d16a0x_vreg_config(VREG_DISABLE);
	msleep(1);

	s6d16a0x_state.disp_powered_up = FALSE;
}

static void read_lcd_id()
{
	unsigned char data[5] = {0, };

	read_ldi_register(RDID1, &data[0], 1);
	read_ldi_register(RDID2, &data[1], 1);
	read_ldi_register(RDID3, &data[2], 1);

	read_ldi_register(0x0B, &data[3], 1);

	lcd_Idtype = data[0] << 2 | data[1] << 1 | data[2] ;
	printk(KERN_ALERT "ldi mtpdata: %x %x %x\n", data[0], data[1], data[2]);
	printk(KERN_ALERT "ldi MADCTL : %x \n", data[3]);

}

static void s6d16a0x_disp_on(void)
{
	int i;
	static int count;


	if (s6d16a0x_state.disp_powered_up && !s6d16a0x_state.display_on) {
		printk(KERN_ALERT "HW rev is %d, apply %d's init sequence\n"
			    , board_hw_revision, board_hw_revision);

		DPRINT("start %s\n", __func__);

		for (i = 0; i < POWER_ON_SETTINGS(power_on_setting_table_lsi); i++)
			setting_table_write(&power_on_setting_table_lsi[i]);

		s6d16a0x_state.display_on = TRUE;

	}
}

#if  defined(LCD_WORK_QUEUE)
static int old_bl_level = 255;

void lcdc_s6d16a0x_workfunc(struct work_struct *work)
{
	lcdc_s6d16a0x_pdata->panel_config_gpio(1);
	s6d16a0x_disp_powerup();
	spi_init();	/* LCD needs SPI */

	setting_table_write(&power_on_setting_table_sleep_out[0]);
	s6d16a0x_disp_on();
	s6d16a0x_state.disp_initialized = TRUE;

	up(&backlight_sem);
	lcdc_s6d_set_brightness_by_ktd259(old_bl_level);
	down(&backlight_sem);
}
#endif
static int lcdc_s6d16a0x_panel_on(struct platform_device *pdev)
{
	DPRINT("start %s\n", __func__);

	if (!s6d16a0x_state.disp_initialized) {
		/* Configure reset GPIO that drives DAC */
		/*Code for work_queue for Amazing TFN*/
	#if  defined(LCD_WORK_QUEUE)
		queue_work(work_queue_s6d16a0x, &lcdc_s6d16a0x_work);
	#else
		lcdc_s6d16a0x_pdata->panel_config_gpio(1);
		s6d16a0x_disp_powerup();
		spi_init();	/* LCD needs SPI */

		setting_table_write(&power_on_setting_table_sleep_out[0]);
		s6d16a0x_disp_on();
		s6d16a0x_state.disp_initialized = TRUE;

	#endif
	}
	return 0;
}

static int lcdc_s6d16a0x_panel_off(struct platform_device *pdev)
{
	int i;

	DPRINT("start %s\n", __func__);

	if (s6d16a0x_state.disp_powered_up && s6d16a0x_state.display_on) {
		#if  defined(LCD_WORK_QUEUE)
		s6d16a0x_state.disp_initialized = FALSE;
		up(&backlight_sem);
		lcdc_s6d_set_brightness_by_ktd259(0);
		down(&backlight_sem);
		/*Code for work_queue for Amazing TFN*/
		cancel_work_sync(&lcdc_s6d16a0x_work);
		#endif
		for (i = 0; i < POWER_OFF_SETTINGS; i++)
			setting_table_write(&power_off_setting_table[i]);
		lcdc_s6d16a0x_pdata->panel_config_gpio(0);
		s6d16a0x_state.display_on = FALSE;
		s6d16a0x_state.disp_initialized = FALSE;
		s6d16a0x_disp_powerdown();
		lcd_prf = 0;
	}

	return 0;
}

static ssize_t lcd_on_off_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct platform_device *pdev;

	if (size < 1)
		return -EINVAL;

	if (strnicmp(buf, "on", 2) == 0 || strnicmp(buf, "1", 1) == 0) {
		lcdc_s6d16a0x_panel_on(pdev);
	} else {
		if (strnicmp(buf, "off", 3) == 0 || strnicmp(buf, "0", 1) == 0) {
			lcdc_s6d16a0x_panel_off(pdev);
		}
	}
	return size;
}

static DEVICE_ATTR(lcd_on_off, 0664, NULL, lcd_on_off_store);

ssize_t lcdtype_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_ALERT "%s \n", __func__);

	if (lcd_Idtype == 0) {
		char *name = "SMD_AMS397GE03";
		return snprintf(buf, 15, "%s\n", name);
		}
	else {
		char *name = "NEW PANEL NAME:TBD";
		return snprintf(buf, 19, "%s\n", name);
		}
}

ssize_t lcdtype_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk(KERN_NOTICE "%s:%s\n", __func__, buf);

	return size;
}
static DEVICE_ATTR(lcdtype, 0644, lcdtype_show, lcdtype_store);
static void lcdc_s6d16a0x_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_value = mfd->bl_level;
	static int lockup_count;

	up(&backlight_sem);
	printk(KERN_ALERT "[BACLKIGHT] : %d\n", bl_value);
	if (!bl_value) {
		/*  Turn off Backlight, don't check disp_initialized value */
		lcd_prf = 1;

	} else {
		if (lcd_prf) {
			down(&backlight_sem);
			return;
		}
		#if defined(LCD_WORK_QUEUE)
			old_bl_level = bl_value;
		#endif
		if (!s6d16a0x_state.disp_initialized) {
			down(&backlight_sem);
			return ;
		}
		while (!s6d16a0x_state.disp_initialized) {
			msleep(100);
			lockup_count++;

			if (lockup_count > 50) {
				printk(KERN_ALERT "Prevent infinite loop(wait for 5000ms)\n");
				printk(KERN_ALERT "LCD can't initialize with in %d ms\n"
					, lockup_count*100);
				lockup_count = 0; /* init lockup_count;*/

				down(&backlight_sem);
				return;
			}
		}
	}

	lcdc_s6d_set_brightness_by_ktd259(bl_value);

	down(&backlight_sem);
}

static int __devinit s6d16a0x_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		s6d16a0x_state.disp_initialized = TRUE;
		s6d16a0x_state.disp_powered_up = TRUE;
		s6d16a0x_state.display_on = TRUE;
		lcdc_s6d16a0x_pdata = pdev->dev.platform_data;
		spi_sclk = *(lcdc_s6d16a0x_pdata->gpio_num);
		spi_cs   = *(lcdc_s6d16a0x_pdata->gpio_num + 1);
		spi_sdi  = *(lcdc_s6d16a0x_pdata->gpio_num + 2);
		lcd_reset = *(lcdc_s6d16a0x_pdata->gpio_num + 3);
		spi_sdo = 23;

		spi_init();
		#if defined(LCD_WORK_QUEUE)
		work_queue_s6d16a0x = create_singlethread_workqueue \
					("s6d16a0x_workqueue");
		if (!work_queue_s6d16a0x) {
			pr_err("%s: count not create workqueue\n", __func__);
			return -ENOMEM;
		}

		INIT_WORK(&lcdc_s6d16a0x_work, lcdc_s6d16a0x_workfunc);
		#endif
		return 0;
	}
	msm_fb_add_device(pdev);
	read_lcd_id();
	if (sec_class == NULL)
		sec_class = class_create(THIS_MODULE, "sec");
	 if (IS_ERR(sec_class))
		pr_err("Failed to create class(sec)!\n");

	 sec_lcdtype_dev = device_create(sec_class, NULL, 0, NULL, "sec_lcd");
	 if (IS_ERR(sec_lcdtype_dev))
		pr_err("Failed to create device(ts)!\n");

	  if (device_create_file(sec_lcdtype_dev, &dev_attr_lcd_on_off) < 0)

		pr_err("Failed to create device file()!\n");

	   if (device_create_file(sec_lcdtype_dev, &dev_attr_lcdtype) < 0)

		pr_err("Failed to create device file()!\n");
	/* lcdc_s6d16a0x_panel_on(pdev); */
	return 0;
}

static void s6d16a0x_shutdown(struct platform_device *pdev)
{
	lcdc_s6d_set_brightness_by_ktd259(0);
	lcdc_s6d16a0x_panel_off(pdev);
}

static struct platform_driver this_driver = {
	.probe  = s6d16a0x_probe,
	.shutdown	= s6d16a0x_shutdown,
	.driver = {
		.name   = "lcdc_s6d16a0x_hvga",
	},
};

static struct msm_fb_panel_data s6d16a0x_panel_data = {
	.on = lcdc_s6d16a0x_panel_on,
	.off = lcdc_s6d16a0x_panel_off,
	.set_backlight = lcdc_s6d16a0x_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_s6d16a0x_hvga",
	.id	= 1,
	.dev	= {
		.platform_data = &s6d16a0x_panel_data,
	}
};

 /*Rookie*/
#define LCDC_FB_XRES    320
#define LCDC_FB_YRES    480

#define LCDC_HBP        15/*16//3//22*/
#define LCDC_HFP        15/*16//3//14*/
#define LCDC_VBP        2/*8*//*change from 8 to 2 //04072012 */
#define LCDC_VPW        2
#define LCDC_VFP        8/*8//2*/
#define LCDC_HPW	5
#define LCDC_BPP	18/*Control the pixel format for panel(RGB666)*/

/*#if 0
#define LCDC_PIXELCLK   { 98 \
		* (LCDC_HFP+LCDC_HPW+LCDC_HBP+LCDC_FB_XRES) \
		* (LCDC_VFP+LCDC_VPW+LCDC_VBP+LCDC_FB_YRES) }
#endif
*/
/*#define LCDC_PCLK  12288000  (16384 * 1000)*/
#define LCDC_PCLK  14336000  /*(16384 * 1000)*/

static int __init lcdc_s6d16a0x_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_TRY_MDDI_CATCH_LCDC_PRISM
	ret = msm_fb_detect_client("lcdc_s6d16a0x_hvga");
	if (ret) {
		printk(KERN_ERR "%s: msm_fb_detect_client failed!\n", __func__);
		return 0;
	}
#endif

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &s6d16a0x_panel_data.panel_info;
	pinfo->xres = LCDC_FB_XRES;
	pinfo->yres = LCDC_FB_YRES;
	MSM_FB_SINGLE_MODE_PANEL(pinfo);
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = LCDC_BPP;
	pinfo->fb_num = 2;
#if 1
	pinfo->clk_rate = LCDC_PCLK;
#else /* rookie */
	pinfo->clk_rate = 17096348;
#endif
	pinfo->bl_max = 255;
	pinfo->bl_min = 1;

	pinfo->lcdc.h_back_porch = LCDC_HBP;
	pinfo->lcdc.h_front_porch = LCDC_HFP;
	pinfo->lcdc.h_pulse_width = LCDC_HPW;
	pinfo->lcdc.v_back_porch = LCDC_VBP;
	pinfo->lcdc.v_front_porch = LCDC_VFP;
	pinfo->lcdc.v_pulse_width = LCDC_VPW;
	pinfo->lcdc.border_clr = 0;     /* blk */
	pinfo->lcdc.underflow_clr = 0xff;       /* blue */
	pinfo->lcdc.hsync_skew = 0;

	ret = platform_device_register(&this_device);
	if (ret) {
		printk(KERN_ERR "%s not able to register the device\n",
			 __func__);
		platform_driver_unregister(&this_driver);
	}
	return ret;
}

static void __exit lcdc_s6d16a0x_panel_exit(void)
{
	platform_device_unregister(&this_device);
	platform_driver_unregister(&this_driver);
	#if  defined(LCD_WORK_QUEUE)
	/*Code for work_queue for Amazing TFN*/
	destroy_workqueue(work_queue_s6d16a0x);
	#endif
}

module_exit(lcdc_s6d16a0x_panel_exit);
module_init(lcdc_s6d16a0x_panel_init);
