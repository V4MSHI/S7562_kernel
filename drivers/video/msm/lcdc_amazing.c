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
#define DPRINT(x...)	printk("AMAZING " x)
#else
#define DPRINT(x...)
#endif

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
};

static struct disp_state_type disp_state = { 0 };
static struct msm_panel_common_pdata *lcdc_amazing_pdata;

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



static char power_setting_seq1[3] = {
	0xC0,
	0x07, 0x07
};

static char power_setting_seq2[2] = {
	0xC1,
	0x45
};

static char power_setting_seq3[2] = {
	0xC2,
	0x22
};

static char init_seq1[5] = {
	0x2A,
	0x00, 0x00, 0x01, 0x3F
};

static char init_seq2[5] = {
	0x2B,
	0x00, 0x00, 0x01, 0xDF
};

static char init_seq3[2] = {
	0xB0,
	0x82
};

static char init_seq4[3] = {
	0xB1,
	0xB0, 0x11
};

static char init_seq5[2] = {
	0xB4,
	0x02
};

static char init_seq6[5] = {
	0xB5,
	0x08, 0x0C, 0x10, 0x0A
};

static char init_seq7[4] = {
	0xB6,
	0x30, 0x42, 0x3B
};

static char init_seq8[2] = {
	0xB7,
	0x07
};

static char init_seq9[6] = {
	0xF4,
	0x00, 0x00, 0x08, 0x91, 0x04
};

static char init_seq10[2] = {
	0x36,
	0x08
};

static char init_seq11[2] = {
	0x3A,
	0x66
};

static char gamma_set_seq1[16] = {
	0xE0,
	0x01, 0x18, 0x15, 0x0A, 0x0D,
	0x08, 0x44, 0x43, 0x35, 0x01,
	0x0C, 0x00, 0x11, 0x0A, 0x0F

};

static char gamma_set_seq2[16] = {
	0xE1,
	0x0F, 0x2A, 0x23, 0x07, 0x0B,
	0x04, 0x42, 0x42, 0x31, 0x04,
	0x0E, 0x02, 0x1F, 0x1C, 0x01

};

static char gamma_set_seq3[17] = {
	0xE2,
	0x19, 0x19, 0x19, 0x19, 0x19,
	0x19, 0x1A, 0x1A, 0x1A, 0x1A,
	0x1A, 0x1A, 0x19, 0x09, 0x09,
	0x09
};

static char gamma_set_seq4[65] = {
	0xE3,
	0x04, 0x04, 0x04, 0x04, 0x04,
	0x04, 0x04, 0x04, 0x04, 0x04,
	0x04, 0x04, 0x04, 0x04, 0x04,
	0x04, 0x04, 0x04, 0x04, 0x04,
	0x04, 0x04, 0x04, 0x05, 0x1D,
	0x2D, 0x2D, 0x2D, 0x2C, 0x4C,
	0x4D, 0x4D, 0x5D, 0x4D, 0x4E,
	0x4D, 0x4D, 0x4D, 0x4D, 0x4D,
	0x3C, 0x3C, 0x3C, 0x4B, 0x4B,
	0x4A, 0x3B, 0x3B, 0x43, 0x33,
	0x33, 0x33, 0xB3, 0xB3, 0xA3,
	0x93, 0x94, 0x84, 0x74, 0x54,
	0x44, 0x33, 0x13, 0x24
};


static char sleep_in_seq[1] = { 0x10 };
static char sleep_out_seq[1] = { 0x11 };
static char disp_on_seq[1] = { 0x29 };
static char disp_off_seq[1] = { 0x28 };

static struct spi_cmd_desc display_on_cmds[] = {

	{sizeof(power_setting_seq1), power_setting_seq1, 0},
	{sizeof(power_setting_seq2), power_setting_seq2, 0},
	{sizeof(power_setting_seq3), power_setting_seq3, 0},

	{sizeof(init_seq1), init_seq1, 0},
	{sizeof(init_seq2), init_seq2, 0},
	{sizeof(init_seq3), init_seq3, 0},
	{sizeof(init_seq4), init_seq4, 0},
	{sizeof(init_seq5), init_seq5, 0},
	{sizeof(init_seq6), init_seq6, 0},
	{sizeof(init_seq7), init_seq7, 0},
	{sizeof(init_seq8), init_seq8, 0},
	{sizeof(init_seq9), init_seq9, 0},
	{sizeof(init_seq10), init_seq10, 0},
	{sizeof(init_seq11), init_seq11, 0},

	{sizeof(gamma_set_seq1), gamma_set_seq1, 0},
	{sizeof(gamma_set_seq2), gamma_set_seq2, 0},
	{sizeof(gamma_set_seq3), gamma_set_seq3, 0},
	{sizeof(gamma_set_seq4), gamma_set_seq4, 0},

	{sizeof(sleep_out_seq), sleep_out_seq, 120},
	{sizeof(disp_on_seq), disp_on_seq, 100},
};

static struct spi_cmd_desc display_off_cmds[] = {
	{sizeof(disp_off_seq), disp_off_seq, 10},
	{sizeof(sleep_in_seq), sleep_in_seq, 120},
};

static struct spi_cmd_desc sw_rdy_cmds[] = {
};



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

	if (!cnt)
		return;

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

#define VREG_ENABLE	1
#define VREG_DISABLE	0

static void amazing_vreg_config(int vreg_en)
{
	DPRINT("start %s\n", __func__);

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
		} else {
			printk(KERN_ERR "%s: LCD enable success (%d)\n",
				 __func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_lcd);
		if (rc) {
			printk(KERN_ERR "%s: LCD disable failed (%d)\n",
				 __func__, rc);
		} else {
			printk(KERN_ERR "%s: LCD disable success (%d)\n",
				 __func__, rc);
		}
	}
}

static void amazing_disp_reset(int normal)
{
	gpio_tlmm_config(GPIO_CFG(lcd_reset, 0, GPIO_CFG_OUTPUT
				, GPIO_CFG_NO_PULL
				, GPIO_CFG_2MA)
				, GPIO_CFG_ENABLE);
	gpio_set_value(lcd_reset, 1);
	msleep(80);
	gpio_set_value(lcd_reset, 0);
	msleep(50);
	gpio_set_value(lcd_reset, 1);
	msleep(120);
}

static void amazing_disp_powerup(void)
{
	DPRINT("start %s\n", __func__);

	if (!disp_state.disp_powered_up && !disp_state.display_on) {

		amazing_vreg_config(VREG_ENABLE);
		/* msleep(10); */
		amazing_disp_reset(0);

		disp_state.disp_powered_up = TRUE;
	}
}

static void amazing_disp_powerdown(void)
{
	DPRINT("start %s\n", __func__);

	gpio_tlmm_config(GPIO_CFG(lcd_reset, 0, GPIO_CFG_OUTPUT
				, GPIO_CFG_NO_PULL
				, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(lcd_reset, 0);

	amazing_vreg_config(VREG_DISABLE);
	msleep(1);

	disp_state.disp_powered_up = FALSE;
}

static void amazing_disp_on(void)
{
	DPRINT("start %s\n", __func__);

	if (disp_state.disp_powered_up && !disp_state.display_on) {
		DPRINT("HW rev is %d, apply %d's init sequence\n"
			    , board_hw_revision, board_hw_revision);

		spi_cmds_tx(display_on_cmds, ARRAY_SIZE(display_on_cmds));

		DPRINT("display on cmd : completed\n");
		disp_state.display_on = TRUE;
	}
}

static int lcdc_amazing_panel_on(struct platform_device *pdev)
{
	DPRINT("start %s\n", __func__);

	if (!disp_state.disp_initialized) {
#ifdef ESD_RECOVERY
		if (irq_disabled) {
			enable_irq(lcd_det_irq);
			irq_disabled = FALSE;
		}
#endif
		/* Configure reset GPIO that drives DAC */
		lcdc_amazing_pdata->panel_config_gpio(1);

		amazing_disp_powerup();

		spi_init();	/* LCD needs SPI */
/*
		spi_cmds_tx(sw_rdy_cmds, ARRAY_SIZE(sw_rdy_cmds));
		msleep(10);
*/
		//read_lcd_id();

		amazing_disp_on();

		if (in_recovery_mode) {
			DPRINT("Backlight control for recovery mode(Max)\n");
			backlight_ic_set_brightness(0xFF);
		}

		disp_state.disp_initialized = TRUE;
	}
	return 0;
}

static int lcdc_amazing_panel_off(struct platform_device *pdev)
{
	DPRINT("start %s\n", __func__);

	if (disp_state.disp_powered_up && disp_state.display_on) {
		if (in_recovery_mode) {
			DPRINT("Backlight control for recovery mode(Dim)\n");
			backlight_ic_set_brightness(0);
		}

#ifdef ESD_RECOVERY
		disable_irq_nosync(lcd_det_irq);
		irq_disabled = TRUE;
#endif
		spi_cmds_tx(display_off_cmds, ARRAY_SIZE(display_off_cmds));
		lcdc_amazing_pdata->panel_config_gpio(0);
		disp_state.display_on = FALSE;
		disp_state.disp_initialized = FALSE;
		amazing_disp_powerdown();
		lcd_prf = 0;
	}
	return 0;
}

static void lcdc_amazing_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_value = mfd->bl_level;
	static int lockup_count;

	lockup_count = 0;
	up(&backlight_sem);
	printk(KERN_INFO "[BACLKIGHT] : %d\n", bl_value);
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
				pr_err("Prevent infinite loop(wait for 5s)\n");
				pr_err("LCD can't initialize with in %d ms\n",
						lockup_count*100);
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
static irqreturn_t amazing_disp_breakdown_det(int irq, void *handle)
{
	if (disp_state.disp_initialized)
		schedule_delayed_work(&lcd_reset_work, 0);

	return IRQ_HANDLED;
}

static void lcdc_dsip_reset_work(struct work_struct *work_ptr)
{
	if (!wa_first_irq) {
		printk(KERN_ERR "skip lcd reset\n");
		wa_first_irq = TRUE;
		return;
	}

	DPRINT("lcd reset\n");

	disp_state.display_on = FALSE;
	disp_state.disp_initialized = FALSE;

	amazing_disp_reset(0);

	spi_init();	/* LCD needs SPI */

	spi_cmds_tx(sw_rdy_cmds, ARRAY_SIZE(sw_rdy_cmds));
	msleep(10);

	read_lcd_id();

	amazing_disp_on();

	disp_state.disp_initialized = TRUE;
}
#endif

static int amazing_disp_set_power(struct lcd_device *dev, int power)
{
	printk(KERN_INFO "amazing_disp_set_power\n");
	return 0;
}

static int amazing_disp_get_power(struct lcd_device *dev, int power)
{
	printk(KERN_INFO "amazing_disp_get_power(%d)\n",
					disp_state.disp_initialized);
	return disp_state.disp_initialized;
}

static ssize_t amazing_lcdtype_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char temp[25];
	size_t maxsize = 25;

	snprintf(temp, maxsize, "BOE_BTO35HVMG003-A301\n");
	strncat(buf, temp, maxsize);
	return strnlen(buf, maxsize);
}

static struct lcd_ops amazing_lcd_props = {
	.get_power = amazing_disp_get_power,
	.set_power = amazing_disp_set_power,
};

static DEVICE_ATTR(lcd_type, S_IRUGO, amazing_lcdtype_show, NULL);

static int __devinit amazing_disp_probe(struct platform_device *pdev)
{
	int ret;
	int i;
	struct lcd_device *lcd_device;

	if (pdev->id == 0) {
		disp_state.disp_initialized = TRUE;
		disp_state.disp_powered_up = TRUE;
		disp_state.display_on = TRUE;

		lcdc_amazing_pdata = pdev->dev.platform_data;
		spi_sclk = *(lcdc_amazing_pdata->gpio_num);
		spi_cs   = *(lcdc_amazing_pdata->gpio_num + 1);
		spi_sdi  = *(lcdc_amazing_pdata->gpio_num + 2);
		lcd_reset = *(lcdc_amazing_pdata->gpio_num + 3);

		DPRINT("%s::spi_sclk = %d\n", __func__, spi_sclk);
		DPRINT("%s::spi_cs = %d\n", __func__, spi_cs);
		DPRINT("%s::spi_sdi = %d\n", __func__, spi_sdi);
		DPRINT("%s::lcd_reset = %d\n", __func__, lcd_reset);

		spi_init();

#ifdef ESD_RECOVERY
		for (i = 0; i < pdev->num_resources; i++) {
			if (!strncmp(pdev->resource[i].name,
					"lcd_breakdown_det", 17)) {
				lcd_det_irq = pdev->resource[i].start;
				if (!lcd_det_irq) {
					printk(KERN_ERR
						"LCD_DETECT_IRQ is NULL!\n");
				}
			}
		}
#endif
		return 0;
	}

	msm_fb_add_device(pdev);

	lcd_device = lcd_device_register("panel", &pdev->dev, NULL,
					&amazing_lcd_props);

	if (IS_ERR(lcd_device)) {
		ret = PTR_ERR(lcd_device);
		printk(KERN_ERR "lcd : failed to register device\n");
		return ret;
	}


	ret = sysfs_create_file(&lcd_device->dev.kobj,
					&dev_attr_lcd_type.attr);
	if (ret) {
		printk(KERN_ERR "sysfs create fail - %s\n",
					dev_attr_lcd_type.attr.name);
	}

#ifdef ESD_RECOVERY
	INIT_DELAYED_WORK(&lcd_reset_work, lcdc_dsip_reset_work);

	ret = request_irq(lcd_det_irq, amazing_disp_breakdown_det,
				IRQF_TRIGGER_RISING, "lcd_esd_det", NULL);
	if (ret) {
		pr_err("Request_irq failed for TLMM_MSM_SUMMARY_IRQ - %d\n",
				ret);
		return ret;
	}
#endif
	/* Avoid unbalanced disables for vreg vlcd at first sleep */
	amazing_vreg_config(VREG_ENABLE);

	return 0;
}

static void amazing_disp_shutdown(struct platform_device *pdev)
{
	backlight_ic_set_brightness(0);
	lcdc_amazing_panel_off(pdev);
}

static struct platform_driver this_driver = {
	.probe  = amazing_disp_probe,
	.shutdown	= amazing_disp_shutdown,
	.driver = {
		.name   = "lcdc_amazing_hvga",
	},
};

static struct msm_fb_panel_data amazing_panel_data = {
	.on = lcdc_amazing_panel_on,
	.off = lcdc_amazing_panel_off,
	.set_backlight = lcdc_amazing_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_amazing_hvga",
	.id	= 1,
	.dev	= {
		.platform_data = &amazing_panel_data,
	}
};

#define LCDC_FB_XRES	320
#define LCDC_FB_YRES	480

#define LCDC_HBP	18
#define LCDC_HPW	2
#define LCDC_HFP	26
#define LCDC_VBP	8
#define LCDC_VPW	2
#define LCDC_VFP	12
#define LCDC_BPP	18/*Control the pixel format for panel(RGB666)*/


/* 16.384Mhz */
#define LCDC_PCLK  12288000/*16384000*/


static int __init lcdc_amazing_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &amazing_panel_data.panel_info;
	pinfo->xres = LCDC_FB_XRES;
	pinfo->yres = LCDC_FB_YRES;
	MSM_FB_SINGLE_MODE_PANEL(pinfo);
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = LCDC_BPP;
	pinfo->fb_num = 2;
	pinfo->clk_rate = LCDC_PCLK;
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

module_init(lcdc_amazing_panel_init);
