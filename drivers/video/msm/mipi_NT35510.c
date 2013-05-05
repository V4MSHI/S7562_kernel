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

#include <linux/fb.h>
#include <linux/lcd.h>

#include <mach/gpio.h>
#include <mach/pmic.h>
#include <mach/vreg.h>

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_NT35510.h"

#include "lcdc_backlight_ic.h"

static struct msm_panel_common_pdata *mipi_nt35510_pdata;
static struct dsi_buf nt35510_tx_buf;
static struct dsi_buf nt35510_rx_buf;

typedef enum
{
	DISP_STATE_UNKNOWN = 0,
	DISP_STATE_ON,
	DISP_STATE_OFF
} disp_state;

/*
 * when we first time enter kernel
 * the LCD is powered on by bootloaer
 *
 * also: initialising makes init_power_state unneeded
 * (caused serious lockups on Kyle rev 0.2 devides)
 */
static disp_state disp_powered_up = DISP_STATE_ON;

/*
 * Commands used for powerin in the panel.
 * They depend on concrete LCD and are passed to
 * initialiser by specific variants of this driver
 */
struct dsi_cmd_desc *concrete_panel_prepare_cmds;
int concrete_panel_prepare_cmds_len;

static int lcd_reset;

#define VREG_ENABLE	TRUE
#define VREG_DISABLE	FALSE

#define LCDC_DEBUG

#ifdef LCDC_DEBUG
#define DPRINT(x...)	printk(KERN_ERR "NT35510 " x)
#else
#define DPRINT(x...)
#endif

static void nt35510_vreg_config(boolean vreg_en)
{
	int rc;
	struct vreg *vreg_lcd = NULL;

#ifdef CONFIG_MACH_KYLE_I
	return;	/* L19(vlcd) use TSP power */
#endif
	DPRINT("start %s\n", __func__);
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

static void nt35510_disp_powerup(void)
{
	DPRINT("start %s\n", __func__);

	gpio_tlmm_config(GPIO_CFG
			 (lcd_reset, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
			  GPIO_CFG_2MA)
			 , GPIO_CFG_ENABLE);

	gpio_set_value(lcd_reset, 0);

/*	nt35510_vreg_config(VREG_ENABLE); */
	mdelay(5);
	gpio_set_value(lcd_reset, 1);
}

static void nt35510_disp_powerdown(void)
{
	DPRINT("start %s\n", __func__);

	/*
	 * nt35510_disp_powerdown called after sleep in
	 * no need to call reset, we called display off, sleep in
	*/
	/*
	gpio_set_value(lcd_reset, 0);
	nt35510_vreg_config(VREG_DISABLE);
	 */
}

/* not used as it causes serious lockups on rev0.2 devices */
#define NT35510_DISPLAY_POWER_MODE_SLEEP_OUT (0x10)
#define NT35510_DISPLAY_POWER_MODE_DISPLAY_ON (0x04)
static void mipi_nt35510_init_power_state(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	DPRINT("start %s\n", __func__);

	return;

	disp_powered_up = DISP_STATE_OFF;

	if (!pdev)
		return;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return;
	if (mfd->key != MFD_KEY)
		return;

	mipi_dsi_cmds_rx(mfd, &nt35510_tx_buf, &nt35510_rx_buf,
			 nt35510_read_display_power_mode_cmds,
			 ARRAY_SIZE(nt35510_read_display_power_mode_cmds));

	if (
		(NT35510_DISPLAY_POWER_MODE_SLEEP_OUT
		 | NT35510_DISPLAY_POWER_MODE_DISPLAY_ON)
		== (nt35510_rx_buf.data[0] &
		  (NT35510_DISPLAY_POWER_MODE_SLEEP_OUT
		   | NT35510_DISPLAY_POWER_MODE_DISPLAY_ON)
	    )
	    ) {
		disp_powered_up = DISP_STATE_ON;
	}
	DPRINT("%s(length %d)[%x] = %d\n",
	       __func__,
	       nt35510_rx_buf.len,
	       nt35510_rx_buf.data[0], disp_powered_up);
}

static int mipi_nt35510_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;

	/*
	 * at this point in initialization
	 * regardless whether display is on or off
	 * tx has to be working properly
	 * this function will be called only once
	 * since after it returns disp_powered up will
	 * have proper value
	 */
	/* CHANGED: never used as it caused lockups */
	if(DISP_STATE_UNKNOWN == disp_powered_up) {
		mipi_nt35510_init_power_state(pdev);
	}

	if (DISP_STATE_ON == disp_powered_up) {
		DPRINT(" %s: already on\n", __func__);
		return 0;
	}

	DPRINT("start %s\n", __func__);

	if (!pdev)
		return -ENODEV;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi = &mfd->panel_info.mipi;

	nt35510_disp_powerup();

	msleep(20);

	mipi_dsi_cmds_tx(mfd, &nt35510_tx_buf,
		concrete_panel_prepare_cmds,
		concrete_panel_prepare_cmds_len);


	mipi_dsi_cmds_tx(mfd, &nt35510_tx_buf,
		nt35510_display_on_cmds,
		ARRAY_SIZE(nt35510_display_on_cmds));


	mdelay(5);	
	mipi_set_tx_power_mode(0);
	mdelay(5);	
	
	mipi_dsi_cmds_tx(mfd, &nt35510_tx_buf,
		nt35510_boe_cmd_dummy,
		ARRAY_SIZE(nt35510_boe_cmd_dummy));
		
	mipi_set_tx_power_mode(1);
	mdelay(1);	
	
	disp_powered_up = DISP_STATE_ON;

	DPRINT("exit %s\n", __func__);
	return 0;
}

static int mipi_nt35510_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	static int b_first_off=1;

	if (DISP_STATE_OFF == disp_powered_up) {
		DPRINT(" %s: already off\n", __func__);
		return 0;
	}

	DPRINT("start %s\n", __func__);

	if (!pdev)
		return -ENODEV;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if(b_first_off)	// workaround.
	{
		nt35510_disp_powerup();

		msleep(20);

		mipi_dsi_cmds_tx(mfd, &nt35510_tx_buf,
			concrete_panel_prepare_cmds,
			concrete_panel_prepare_cmds_len);

		mipi_dsi_cmds_tx(mfd, &nt35510_tx_buf,
			nt35510_display_on_cmds,
			ARRAY_SIZE(nt35510_display_on_cmds));		
		
		msleep(20);
		b_first_off=0;
	}

	mipi_dsi_cmds_tx(mfd, &nt35510_tx_buf, nt35510_display_off_cmds,
			 ARRAY_SIZE(nt35510_display_off_cmds));


	mipi_dsi_cmds_tx(
			mfd,
			&nt35510_tx_buf,
			nt35510_deep_standby_cmds,
			ARRAY_SIZE(nt35510_deep_standby_cmds)
	);


	disp_powered_up = DISP_STATE_OFF;

	DPRINT("exit %s\n", __func__);
	return 0;
}

/** sysfs handling ***/
static ssize_t mipi_nt35510_lcdtype_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
#if defined(CONFIG_MACH_KYLE)
	strcat(buf, "BOE_BTO35HVMG003-A301\n");
#else
	strcat(buf, "UNKNOWN\n");
#endif
	return strlen(buf);
}

static int mipi_nt35510_set_power(struct lcd_device *ldev, int power)
{
	DPRINT("%s[%d] (ignored)\n", __func__, power);
	return 0;
}

static int mipi_nt35510_get_power(struct lcd_device *dev)
{
	DPRINT("%s\n", __func__);
	/* other power states not supported */
	return (DISP_STATE_ON == disp_powered_up)
	    ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;
}

static struct lcd_ops mipi_lcd_props = {
	.get_power = mipi_nt35510_get_power,
	.set_power = mipi_nt35510_set_power,
};

static DEVICE_ATTR(lcd_type, S_IRUGO, mipi_nt35510_lcdtype_show, NULL);

static int __devinit mipi_nt35510_lcd_probe(struct platform_device *pdev)
{
	struct platform_device *msm_fb_pdev;
	struct lcd_device *lcd_device;
	int ret;

	DPRINT("%s\n", __func__);

	if (pdev->id == 0) {
		mipi_nt35510_pdata = pdev->dev.platform_data;
		return 0;
	}

	/*
	 * save returned struct platform_device pointer
	 * as we later need to get msm_fb_data_type
	 */
	msm_fb_pdev = msm_fb_add_device(pdev);

	/* struct lcd_device now has needed platform data */
	lcd_device = lcd_device_register("panel", &pdev->dev,
					 platform_get_drvdata(msm_fb_pdev),
					 &mipi_lcd_props);

	if (IS_ERR(lcd_device)) {
		ret = PTR_ERR(lcd_device);
		printk(KERN_ERR "lcd : failed to register device\n");
		return ret;
	}

	ret = sysfs_create_file(&lcd_device->dev.kobj, &dev_attr_lcd_type.attr);
	if (ret)
		printk(KERN_ERR "sysfs create fail - %s\n",
		       dev_attr_lcd_type.attr.name);

	return 0;
}

static struct platform_driver this_driver = {
	.probe = mipi_nt35510_lcd_probe,
	.driver = {
		   .name = "mipi_NT35510",
		   },
};

static void mipi_nt35510_set_backlight(struct msm_fb_data_type *mfd)
{
	int level = mfd->bl_level;

	/* function will spin lock */
	backlight_ic_set_brightness(level);
}

static struct msm_fb_panel_data nt35510_panel_data = {
	.on = mipi_nt35510_lcd_on,
	.off = mipi_nt35510_lcd_off,
	.set_backlight = mipi_nt35510_set_backlight,
};

static int ch_used[3];

static int mipi_nt35510_lcd_init(void)
{
	DPRINT("start %s\n", __func__);

	lcd_reset = (board_hw_revision >= 2) ? 23 : 22;

	DPRINT("%s : LCD reset %d\n", __func__, lcd_reset);

	mipi_dsi_buf_alloc(&nt35510_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&nt35510_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

int mipi_nt35510_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel,
					struct dsi_cmd_desc *panel_prepare,
					int panel_prepare_length)
{
	struct platform_device *pdev = NULL;
	int ret;

	DPRINT("start %s\n", __func__);
	concrete_panel_prepare_cmds = panel_prepare;
	concrete_panel_prepare_cmds_len = panel_prepare_length;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	ret = mipi_nt35510_lcd_init();
	if (ret) {
		DPRINT("mipi_nt35510_lcd_init() failed with ret %u\n", ret);
		return ret;
	}

	pdev = platform_device_alloc("mipi_NT35510", (panel << 8) | channel);
	if (!pdev)
		return -ENOMEM;

	nt35510_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &nt35510_panel_data,
				       sizeof(nt35510_panel_data));
	if (ret) {
		pr_debug("%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_debug("%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

 err_device_put:
	platform_device_put(pdev);
	return ret;
}
