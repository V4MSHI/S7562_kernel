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
#include <linux/lcd.h>
#include <mach/gpio.h>
#include <mach/pmic.h>
#include <mach/vreg.h>
#include "msm_fb.h"

#ifdef CONFIG_FB_MSM_TRY_MDDI_CATCH_LCDC_PRISM
#include "mddihosti.h"
#endif

#include "lcdc_backlight_ic.h"

#define LCDC_DEBUG

#ifdef LCDC_DEBUG
#define DPRINT(x...)	printk(KERN_DEBUG "LCD " x)
#else
#define DPRINT(x...)
#endif

#define USE_STANDBY_MODE /*temporary*/

static int spi_cs;
static int spi_sclk;
static int spi_sdi;

static int lcd_reset;

#define ESD_RECOVERY
#ifdef ESD_RECOVERY
static unsigned int lcd_det_irq;
static struct delayed_work lcd_reset_work;
boolean irq_disabled = FALSE;
boolean wa_first_irq = FALSE;
#endif

struct disp_state_type {
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
#ifdef USE_STANDBY_MODE
	boolean standby;
#endif
};

static struct disp_state_type disp_state = { 0 };
static struct msm_panel_common_pdata *lcdc_trebon_pdata;

static int lcd_prf;

extern int board_hw_revision;

static DEFINE_SEMAPHORE(backlight_sem);
static DEFINE_MUTEX(spi_mutex);

#define DEFAULT_USLEEP	1

#define RDID1			0xDA
#define RDID2			0xDB
#define RDID3			0xDC

struct spi_cmd_desc {
	int dlen;
	char *payload;
	int wait;
};

#if (CONFIG_MACH_TREBON_HWREV == 0x0) || (CONFIG_MACH_TREBON_HWREV == 0x1)

static char set_open_password[3] = {
	0xEF,
	0x74, 0x20
};

#if 0 /*jyhong_rev05*/
static char power_setting_seq1[7] = {
	0xF2,
	0x00, 0x00, 0x00, 0x00, 0x0A,
	0x00
};
#endif
static char power_setting_seq2[10] = {
	0xB1,
	0x01, 0x00, 0x22, 0x11, 0x73,
	0x70, 0xEC, 0x15, 0x2C
};

static char power_setting_seq3[9] = {
	0xB2,
	0x66, 0x06, 0xAA, 0x88, 0x88,
	0x08, 0x08, 0x03
};

static char init_seq1[6] = {
	0xB4,
	0x10, 0x00, 0x32, 0x32, 0x32
};

static char init_seq2[9] = {
	0xB6,
	0x74, 0x86, 0x22, 0x00, 0x22,
	0x00, 0x22, 0x00
};

static char init_seq3[4] = {
	0xD5,
	0x02, 0x43, 0x01
};

static char init_seq4[2] = {
	/*
	 * MADCTL
	 * User setting (LDI Upside)
	 */
	0x36,
	0x09
};

static char init_seq5[2] = {
	/*
	 * COLMOD
	 * User setting (16M Color)
	 */
	0x3A,
	0x77
};

static char init_seq6[2] = {
	/*
	 * SETPOL
	 * User setting (DE / DCLK polarity)
	 */
	0xF3,
	0x10
};

/*Gamma Setting 2.6 */
static char gamma_set_seq1[35] = {
	/* Blue gamma */
	0xE0,
	0x4B, 0x49, 0x47, 0x11, 0x0B,
	0x02, 0x3C, 0x36, 0x06, 0x0E,
	0x11, 0x15, 0x19, 0x17, 0x18,
	0x10, 0x17, 0x33, 0x33, 0x33,
	0x11, 0x0C, 0x03, 0x2F, 0x2F,
	0x07, 0x0F, 0x11, 0x16, 0x18,
	0x17, 0x17, 0x0F, 0x16
};

static char gamma_set_seq2[35] = {
	/* Green gamma */
	0xE1,
	0x1D, 0x24, 0x2A, 0x0C, 0x07,
	0x03, 0x31, 0x32, 0x08, 0x0F,
	0x12, 0x17, 0x1B, 0x18, 0x19,
	0x14, 0x19, 0x02, 0x0C, 0x12,
	0x09, 0x09, 0x00, 0x20, 0x2E,
	0x08, 0x0F, 0x12, 0x18, 0x19,
	0x18, 0x18, 0x14, 0x1A
};

static char gamma_set_seq3[35] = {
	/* Red gamma */
	0xE2,
	0x29, 0x2A, 0x2B, 0x0D, 0x0B,
	0x03, 0x2D, 0x34, 0x07, 0x0D,
	0x12, 0x18, 0x1A, 0x18, 0x19,
	0x13, 0x18, 0x0F, 0x12, 0x14,
	0x0A, 0x09, 0x01, 0x1D, 0x2B,
	0x07, 0x0C, 0x12, 0x17, 0x1A,
	0x18, 0x18, 0x13, 0x18
};

static char set_close_password[3] = {
	0xEF,
	0x00, 0x00
};

static char deep_standby_en[2] = {
	0xDE,
	0x01
};

static char sleep_in_seq[1] = { 0x10 };
static char sleep_out_seq[1] = { 0x11 };
static char disp_on_seq[1] = { 0x29 };
static char disp_off_seq[1] = { 0x28 };

static struct spi_cmd_desc display_on_cmds[] = {
	{sizeof(set_open_password), set_open_password, 0},
/*	{sizeof(power_setting_seq1), power_setting_seq1, 0}, *//*jyhong_rev05*/
	{sizeof(power_setting_seq2), power_setting_seq2, 0},
	{sizeof(power_setting_seq3), power_setting_seq3, 0},

	{sizeof(init_seq1), init_seq1, 0},
	{sizeof(init_seq2), init_seq2, 0},
	{sizeof(init_seq3), init_seq3, 0},
	{sizeof(init_seq4), init_seq4, 0},
	{sizeof(init_seq5), init_seq5, 0},
	{sizeof(init_seq6), init_seq6, 0},

	{sizeof(gamma_set_seq1), gamma_set_seq1, 0},
	{sizeof(gamma_set_seq2), gamma_set_seq2, 0},
	{sizeof(gamma_set_seq3), gamma_set_seq3, 0},

	{sizeof(set_close_password), set_close_password, 0},
	{sizeof(disp_on_seq), disp_on_seq, 10},
};

static struct spi_cmd_desc display_off_cmds[] = {
	{sizeof(disp_off_seq), disp_off_seq, 0},
	{sizeof(sleep_in_seq), sleep_in_seq, 120},
};

static struct spi_cmd_desc display_standby_in_cmds[] = {
	{sizeof(disp_off_seq), disp_off_seq, 0},
	{sizeof(deep_standby_en), deep_standby_en, 0},
};

static struct spi_cmd_desc sw_rdy_cmds[] = {
	{sizeof(sleep_out_seq), sleep_out_seq, 0},
};
#else
static char power_setting_seq1[3] = {
	0xC0,
	0x18, 0x19
};

static char power_setting_seq2[2] = {
	0xC1,
	0x04
};

static char power_setting_seq3[2] = {
	0xC2,
	0x33
};

static char power_setting_seq4[3] = {
	0xC5,
	0x00, 0x40
};

static char init_seq1_1[5] = {
	0x2A,
	0x00, 0x00, 0x01, 0x3F
};

static char init_seq1_2[5] = {
	0x28,
	0x00, 0x00, 0x01, 0xDF
};

static char init_seq1_3[2] = {
	0x36,
	0x08
};

static char init_seq1_4[2] = {
	0x3A,
	0x66
};

static char init_seq2_1[3] = {
	0xB1,
	0xA0, 0x11
};

static char init_seq2_2[2] = {
	0xB4,
	0x00
};

static char init_seq2_3[5] = {
	0xB5,
	0x02, 0x02, 0x1C, 0x18
};

static char init_seq2_4[4] = {
	0xB6,
	0x30, 0x42, 0x3B
};

static char init_seq2_5[2] = {
	0xB7,
	0xC7
};

static char init_seq3_1[9] = {
	0xF1,
	0x36, 0x04, 0x00, 0x3C, 0x0F,
	0x0F, 0xA4, 0x02
};

static char init_seq3_2[9] = {
	0xF2,
	0x1A, 0xA3, 0x12, 0x02, 0x82,
	0x02, 0xFF, 0x02
};

static char init_seq3_3[1] = { 0x13 };

static char gamma_set_seq1[16] = {
	0xE0,
	0x1F, 0x26, 0x24, 0x0F, 0x0E,
	0x0D, 0x57, 0x87, 0x49, 0x0B,
	0x17, 0x07, 0x0C, 0x0B, 0x00
};

static char gamma_set_seq2[16] = {
	0xE1,
	0x1F, 0x39, 0x31, 0x09, 0x0A,
	0x01, 0x41, 0x32, 0x33, 0x04,
	0x0D, 0x03, 0x23, 0x1F, 0x00
};

static char gamma_set_seq3[17] = {
	0xE2,
	0x00, 0x09, 0x09, 0x09, 0x0A,
	0x0A, 0x0A, 0x0A, 0x0A, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00
};

static char gamma_set_seq4[65] = {
	0xE3,
	0x00, 0x00, 0x00, 0x00, 0x07,
	0x07, 0x07, 0x07, 0x07, 0x07,
	0x07, 0x07, 0x07, 0x07, 0x07,
	0x07, 0x07, 0x07, 0x07, 0x07,
	0x07, 0x07, 0x07, 0x07, 0x07,
	0x07, 0x07, 0x07, 0x07, 0x07,
	0x07, 0x07, 0x07, 0x07, 0x07,
	0x07, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00
};

static char ifmode_set[2] = {
	/*
	 * DotClock	:Rising Edge
	 * Enable	:High Active
	 * HSYNC	:Low Active
	 * VSYNC	:Low Active
	 * 3-wire mode
	*/
	0xB0,
	0x8C
};
static char ifmode_set2[2] = {
	0xFB,
	0x04
};


static char sleep_in_seq[1] = { 0x10 };
static char sleep_out_seq[1] = { 0x11 };
static char disp_on_seq[1] = { 0x29 };
static char disp_off_seq[1] = { 0x28 };
static char sw_reset_seq[1] = { 0x01 };

static struct spi_cmd_desc display_on_cmds[] = {
	{sizeof(power_setting_seq1), power_setting_seq1, 0},
	{sizeof(power_setting_seq2), power_setting_seq2, 0},
	{sizeof(power_setting_seq3), power_setting_seq3, 0},
	{sizeof(power_setting_seq4), power_setting_seq4, 0},

	{sizeof(init_seq1_1), init_seq1_1, 0},
	{sizeof(init_seq1_2), init_seq1_2, 0},
	{sizeof(init_seq1_3), init_seq1_3, 0},
	{sizeof(init_seq1_4), init_seq1_4, 0},
	{sizeof(init_seq2_1), init_seq2_1, 0},
	{sizeof(init_seq2_2), init_seq2_2, 0},
	{sizeof(init_seq2_3), init_seq2_3, 0},
	{sizeof(init_seq2_4), init_seq2_4, 0},
	{sizeof(init_seq2_5), init_seq2_5, 0},
	{sizeof(init_seq3_1), init_seq3_1, 0},
	{sizeof(init_seq3_2), init_seq3_2, 0},
	{sizeof(init_seq3_3), init_seq3_3, 0},

	{sizeof(gamma_set_seq1), gamma_set_seq1, 0},
	{sizeof(gamma_set_seq2), gamma_set_seq2, 0},
	{sizeof(gamma_set_seq3), gamma_set_seq3, 0},
	{sizeof(gamma_set_seq4), gamma_set_seq4, 0},

	{sizeof(sleep_out_seq), sleep_out_seq, 120},
	{sizeof(disp_on_seq), disp_on_seq, 40},
};

static struct spi_cmd_desc display_off_cmds[] = {
	{sizeof(disp_off_seq), disp_off_seq, 40},
	{sizeof(sleep_in_seq), sleep_in_seq, 120},
};

static struct spi_cmd_desc display_standby_in_cmds[] = {
	{sizeof(disp_off_seq), disp_off_seq, 0},
};

static struct spi_cmd_desc sw_rdy_cmds[] = {
	{sizeof(sw_reset_seq), sw_reset_seq, 10},
	{sizeof(ifmode_set2), ifmode_set2, 0},
	{sizeof(ifmode_set), ifmode_set, 0},
};
#endif

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

	/* swith input */
	gpio_direction_input(spi_sdi);

	if (count > 1) {
		/* dummy clock cycle */
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
				if (gpio_get_value(spi_sdi))
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

	/* switch output */
	gpio_direction_output(spi_sdi, 0);
}

static void spi_cmds_tx(struct spi_cmd_desc *desc, int cnt)
{
	long i, j, p;
	unsigned long irqflags;

	mutex_lock(&spi_mutex);
	for (p = 0; p < cnt; p++) {
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
			if (((char)*(desc+p)->payload >> i) & 0x1)
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
		if ((desc+p)->dlen < 2)
			goto tx_done;

		for (j = 1; j < (desc+p)->dlen; j++) {
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
				if (((char)*((desc+p)->payload+j) >> i) & 0x1)
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
tx_done:
		if ((desc+p)->wait)
			msleep((desc+p)->wait);
	}
	mutex_unlock(&spi_mutex);
}

static void read_lcd_id()
{
	unsigned char data[4] = {0, };

	read_ldi_register(RDID1, &data[0], 1);
	read_ldi_register(RDID2, &data[1], 1);
	read_ldi_register(RDID3, &data[2], 1);

	printk("ldi mtpdata: %x %x %x\n", data[0], data[1], data[2]);
}

static void spi_init(void)
{
	/* Set the output so that we dont disturb the slave device */
	gpio_set_value(spi_sclk, 0);
	gpio_set_value(spi_sdi, 0);

	/* Set the Chip Select De-asserted */
	gpio_set_value(spi_cs, 0);

}

static void spi_standby(void)
{
	/* Set the output so that we dont disturb the slave device */
	gpio_set_value(spi_sclk, 0);
	gpio_set_value(spi_sdi, 0);

	/* Set the Chip Select De-asserted */
	gpio_set_value(spi_cs, 1);

}

#define VREG_ENABLE	1
#define VREG_DISABLE	0

static void trebon_vreg_config(int vreg_en)
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

static void trebon_disp_reset(int normal)
{
	gpio_tlmm_config(GPIO_CFG(lcd_reset, 0, GPIO_CFG_OUTPUT
				, GPIO_CFG_NO_PULL
				, GPIO_CFG_2MA)
				, GPIO_CFG_ENABLE);
#if (CONFIG_MACH_TREBON_HWREV == 0x0) || (CONFIG_MACH_TREBON_HWREV == 0x1)
	if (normal) {
		gpio_set_value(lcd_reset, 0);
		usleep(50);
		gpio_set_value(lcd_reset, 1);
		msleep(80);
	}
	gpio_set_value(lcd_reset, 0);
	msleep(170);
	gpio_set_value(lcd_reset, 1);
/*	msleep(10);*/
	usleep(10000);
#else
	gpio_set_value(lcd_reset, 0);
	usleep(50);
	gpio_set_value(lcd_reset, 1);
	usleep(50);
	gpio_set_value(lcd_reset, 0);
	usleep(50);
	gpio_set_value(lcd_reset, 1);
/*	msleep(10);*/
	usleep(10000);
#endif
}

static void trebon_disp_powerup(void)
{
	DPRINT("start %s\n", __func__);

	if (!disp_state.disp_powered_up && !disp_state.display_on) {

		trebon_vreg_config(VREG_ENABLE);
/*		msleep(10);*/
		usleep(10000);
		trebon_disp_reset(0);

		disp_state.disp_powered_up = TRUE;
	}
}

static void trebon_disp_powerdown(void)
{
	DPRINT("start %s\n", __func__);

	gpio_tlmm_config(GPIO_CFG(lcd_reset, 0, GPIO_CFG_OUTPUT
					, GPIO_CFG_NO_PULL
					, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(lcd_reset, 0);

	trebon_vreg_config(VREG_DISABLE);
	msleep(1);

	disp_state.disp_powered_up = FALSE;
}

static void trebon_disp_on(void)
{
	int i;

	DPRINT("start %s\n", __func__);

	if (disp_state.disp_powered_up && !disp_state.display_on) {
		DPRINT("HW rev is %d, apply %d's init sequence\n" ,
			board_hw_revision, board_hw_revision);

		spi_cmds_tx(display_on_cmds, ARRAY_SIZE(display_on_cmds));
		msleep(40);

		DPRINT("display on cmd : completed\n");
		disp_state.display_on = TRUE;
	}
}

static int lcdc_trebon_panel_standby_in(struct platform_device *pdev)
{
	DPRINT("start %s\n",  __func__);

	if (disp_state.disp_powered_up && disp_state.display_on) {
#ifdef ESD_RECOVERY
		disable_irq_nosync(lcd_det_irq);
		irq_disabled = TRUE;
#endif
		spi_cmds_tx(display_standby_in_cmds,
				ARRAY_SIZE(display_standby_in_cmds));
		msleep(70);

		spi_standby();

		disp_state.display_on = FALSE;
		disp_state.disp_initialized = FALSE;
		lcd_prf = 0;
	}
	return 0;
}

static int lcdc_trebon_panel_standby_out(struct platform_device *pdev)
{
	DPRINT("start %s\n", __func__);

	msleep(50);
	if (!disp_state.disp_initialized) {
#ifdef ESD_RECOVERY
		if (irq_disabled) {
			enable_irq(lcd_det_irq);
			irq_disabled = FALSE;
		}
#endif

	/*signal_timing*/
		spi_standby();
/*		msleep(10);*/
		usleep(10000);

		trebon_disp_reset(0);
/*
		trebon_disp_reset(1);

		spi_init();*/	/* LCD needs SPI */

		spi_cmds_tx(sw_rdy_cmds, ARRAY_SIZE(sw_rdy_cmds));
/*		msleep(10);*/
		usleep(10000);

		trebon_disp_on();

		disp_state.disp_initialized = TRUE;
	}
	return 0;
}

static int lcdc_trebon_panel_on(struct platform_device *pdev)
{
	DPRINT("start %s\n", __func__);

#ifdef USE_STANDBY_MODE
	if (disp_state.standby) {
		lcdc_trebon_panel_standby_out(pdev);
	} else {
#endif
	if (!disp_state.disp_initialized) {
#ifdef ESD_RECOVERY
		if (irq_disabled) {
			enable_irq(lcd_det_irq);
			irq_disabled = FALSE;
		}
#endif
		/* Configure reset GPIO that drives DAC */
		lcdc_trebon_pdata->panel_config_gpio(1);

	/*signal_timing*/
		spi_standby();
/*		msleep(10);*/
		usleep(10000);

		trebon_disp_powerup();
/*
		trebon_disp_powerup();

		spi_init();	*//* LCD needs SPI */

		spi_cmds_tx(sw_rdy_cmds, ARRAY_SIZE(sw_rdy_cmds));
/*		msleep(10);*/
		usleep(10000);

		read_lcd_id();

		trebon_disp_on();

		disp_state.disp_initialized = TRUE;
	}
#ifdef USE_STANDBY_MODE
	}
#endif
	return 0;
}

static int lcdc_trebon_panel_off(struct platform_device *pdev)
{
#ifdef USE_STANDBY_MODE
	lcdc_trebon_panel_standby_in(pdev);
	disp_state.standby = TRUE;
#else
	DPRINT("start %s\n", __func__);

	if (disp_state.disp_powered_up && disp_state.display_on) {
#ifdef ESD_RECOVERY
		disable_irq_nosync(lcd_det_irq);
		irq_disabled = TRUE;
#endif
		spi_cmds_tx(display_off_cmds, ARRAY_SIZE(display_off_cmds));
		lcdc_trebon_pdata->panel_config_gpio(0);
		disp_state.display_on = FALSE;
		disp_state.disp_initialized = FALSE;
		trebon_disp_powerdown();
		lcd_prf = 0;
	}
#endif
	return 0;
}

static void lcdc_trebon_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_value = mfd->bl_level;
	static int lockup_count;

	up(&backlight_sem);
	DPRINT("[BACLKIGHT] : %d\n", bl_value);
	if (!bl_value) {
		/*  Turn off Backlight, don't check disp_initialized value */
		lcd_prf = 1;

	} else {
		if (lcd_prf)
			return;

		while (!disp_state.disp_initialized) {
			msleep(100);
			lockup_count++;

			if (lockup_count > 50) {
				printk(KERN_ERR "Prevent infinite loop(wait for 5s)\n");
				printk(KERN_ERR "LCD can't initialize with in %d ms\n"
					, lockup_count*100);
				lockup_count = 0;

				down(&backlight_sem);
				return;
			}
		}
	}

	backlight_ic_set_brightness(bl_value);

	down(&backlight_sem);
}

#ifdef ESD_RECOVERY
static irqreturn_t trebon_disp_breakdown_det(int irq, void *handle)
{
	if (disp_state.disp_initialized)
		schedule_delayed_work(&lcd_reset_work, 0);

	return IRQ_HANDLED;
}

static void lcdc_dsip_reset_work(struct work_struct *work_ptr)
{
	if (!wa_first_irq) {
		DPRINT("skip lcd reset\n");
		wa_first_irq = TRUE;
		return;
	}

	DPRINT("lcd reset\n");
#if 0

	disp_state.display_on = FALSE;
	disp_state.disp_initialized = FALSE;
	disp_state.disp_powered_up = FALSE;

	trebon_disp_reset(1);

	spi_init();	/* LCD needs SPI */

	spi_cmds_tx(sw_rdy_cmds, ARRAY_SIZE(sw_rdy_cmds));
	msleep(10);

	read_lcd_id();

	trebon_disp_on();

	disp_state.disp_initialized = TRUE;
#else
	lcdc_trebon_panel_off(NULL);
	trebon_disp_reset(0);
	lcdc_trebon_panel_on(NULL);
#endif
}
#endif

static int trebon_disp_set_power(struct lcd_device *dev, int power)
{
	DPRINT("trebon_disp_set_power\n");
	return 0;
}

static int trebon_disp_get_power(struct lcd_device *dev, int power)
{
	DPRINT("trebon_disp_get_power\n");
	return disp_state.disp_initialized;
}

static ssize_t trebon_lcdtype_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char temp[20];
#if (CONFIG_MACH_TREBON_HWREV == 0x0) || (CONFIG_MACH_TREBON_HWREV == 0x1)
/*	sprintf(temp, "SMD_LMS635DF04\n");*/
	DPRINT("SMD_LMS635DF04\n");
#else
/*	sprintf(temp, "SEC_ILI9486\n");*/
	DPRINT("SEC_ILI9486\n");
#endif
/*	strcat(buf, temp);
	return strlen(buf);*/
	strncat(buf, temp, sizeof(temp));
	return strnlen(buf, sizeof(temp));
}

static struct lcd_ops trebon_lcd_props = {
	.get_power = trebon_disp_get_power,
	.set_power = trebon_disp_set_power,
};

static DEVICE_ATTR(lcd_type, S_IRUGO, trebon_lcdtype_show, NULL);

static int __devinit trebon_disp_probe(struct platform_device *pdev)
{
	int ret;
	int i;
	struct lcd_device *lcd_device;
	if (pdev->id == 0) {
		disp_state.disp_initialized = TRUE; /*signal_timing*/
		disp_state.disp_powered_up = TRUE;
		disp_state.display_on = TRUE;
/*
		disp_state.disp_initialized = FALSE;
		disp_state.disp_powered_up = FALSE;
		disp_state.display_on = FALSE;
*/
#ifdef USE_STANDBY_MODE
		disp_state.standby = FALSE;
#endif
		lcdc_trebon_pdata = pdev->dev.platform_data;
		spi_sclk = *(lcdc_trebon_pdata->gpio_num);
		spi_cs   = *(lcdc_trebon_pdata->gpio_num + 1);
		spi_sdi  = *(lcdc_trebon_pdata->gpio_num + 2);
		lcd_reset = *(lcdc_trebon_pdata->gpio_num + 3);

		spi_standby();	/*spi_init();*//*cs: active low*/

#ifdef ESD_RECOVERY
		for (i = 0; i < pdev->num_resources; i++) {
			if (!strncmp(pdev->resource[i].name,
					"lcd_breakdown_det", 17)) {
				lcd_det_irq = pdev->resource[i].start;
				if (!lcd_det_irq)
					DPRINT("LCD_DETECT_IRQ is NULL!\n");
			}
		}
#endif
		return 0;
	}

	msm_fb_add_device(pdev);

	lcd_device = lcd_device_register("display", &pdev->dev, NULL,
					&trebon_lcd_props);

	if (IS_ERR(lcd_device)) {
		ret = PTR_ERR(lcd_device);
		printk(KERN_ERR "lcd : failed to register device\n");
		return ret;
	}

	ret = sysfs_create_file(&lcd_device->dev.kobj, &dev_attr_lcd_type.attr);
	if (ret)
		printk(KERN_ERR "sysfs create fail - %s\n",
		dev_attr_lcd_type.attr.name);
#ifdef ESD_RECOVERY
	INIT_DELAYED_WORK(&lcd_reset_work, lcdc_dsip_reset_work);
	ret = request_irq(lcd_det_irq, trebon_disp_breakdown_det,
	IRQF_TRIGGER_RISING, "lcd_esd_det", NULL);

	if (ret) {
		pr_err("Request_irq failed for TLMM_MSM_SUMMARY_IRQ - %d\n",
			ret);
		return ret;
	}
#endif

	return 0;
}

static void trebon_disp_shutdown(struct platform_device *pdev)
{
	backlight_ic_set_brightness(0);
	lcdc_trebon_panel_off(pdev);
}

static struct platform_driver this_driver = {
	.probe  = trebon_disp_probe,
	.shutdown	= trebon_disp_shutdown,
	.driver = {
		.name   = "lcdc_trebon_hvga",
	},
};

static struct msm_fb_panel_data trebon_panel_data = {
	.on = lcdc_trebon_panel_on,
	.off = lcdc_trebon_panel_off,
	.set_backlight = lcdc_trebon_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_trebon_hvga",
	.id	= 1,
	.dev	= {
		.platform_data = &trebon_panel_data,
	}
};

#define LCDC_FB_XRES	320
#define LCDC_FB_YRES	480

#if (CONFIG_MACH_TREBON_HWREV == 0x0) || (CONFIG_MACH_TREBON_HWREV == 0x1)
 #define LCDC_HBP		32
 #define LCDC_HPW		16
 #define LCDC_HFP		12
 #define LCDC_VBP		7
 #define LCDC_VPW		4
 #define LCDC_VFP		8
 #define LCDC_BPP		24
#else
 #define LCDC_HBP		24
 #define LCDC_HPW		4
 #define LCDC_HFP		28
 #define LCDC_VBP		2
 #define LCDC_VPW		2
 #define LCDC_VFP		2
 #define LCDC_BPP		18
#endif

#if 0
#define LCDC_PCLK	((LCDC_FB_XRES + LCDC_HBP + LCDC_HPW + LCDC_HFP) \
			* (LCDC_FB_YRES + LCDC_VBP + LCDC_VPW + LCDC_VFP) * 60)
#else

#define LCDC_PCLK  (16384000) /*16.384 Mhz*//*jyhong_clk*/
/*
6144 * 1000 Hz
8192 * 1000 Hz
16384 * 1000 Hz
24576 * 1000 Hz
30720 * 1000 Hz
*/
#endif

static int __init lcdc_trebon_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;
#ifdef CONFIG_FB_MSM_TRY_MDDI_CATCH_LCDC_PRISM
	ret = msm_fb_detect_client("lcdc_trebon_hvga");
	if (ret) {
		DPRINT("%s:msm_fb_detect_client failed!\n", __func__);
		return 0;
	}
#endif

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &trebon_panel_data.panel_info;
	pinfo->xres = LCDC_FB_XRES;
	pinfo->yres = LCDC_FB_YRES;
	MSM_FB_SINGLE_MODE_PANEL(pinfo);
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = LCDC_BPP;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 16384000; /*LCDC_PCLK;*/
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

module_init(lcdc_trebon_panel_init);
