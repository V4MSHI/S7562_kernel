/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio_event.h>
#include <linux/i2c-gpio.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/usbdiag.h>
#include <mach/usb_gadget_fserial.h>
#include <mach/msm_memtypes.h>
#include <mach/msm_serial_hs.h>
#include <linux/usb/android.h>
#include <linux/platform_device.h>
#include <linux/io.h>
//#include <mach/gpio-v1.h>
#ifdef CONFIG_MACH_JENA
#include <mach/gpio_jena.h>
#elif defined CONFIG_MACH_KYLE
#include <mach/gpio_kyle.h>
#else
#include <mach/gpio_trebon.h>
#endif
#include <mach/pmic.h>
#include <mach/socinfo.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/mmc.h>
#include <linux/i2c.h>
#include <linux/i2c/sx150x.h>
#include <linux/gpio.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <linux/mfd/marimba.h>
#include <mach/vreg.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_battery.h>
#include <linux/smsc911x.h>
#include <linux/atmel_maxtouch.h> //todo*** remove atmel changes
#include "devices.h"
#include "timer.h"
#include "board-msm7x27a-regulator.h"
#include "devices-msm7x2xa.h"
#include "pm.h"
#ifdef CONFIG_SAMSUNG_JACK
#include <linux/sec_jack.h>
#endif
#include <mach/rpc_server_handset.h>
#include <mach/socinfo.h>
#include <linux/fsaxxxx_usbsw.h>
#include "proc_comm.h"
#include "pm-boot.h"
#include "board-msm7627a.h"
#ifdef CONFIG_BATTERY_MAX17040
#include <linux/max17040_battery.h>
#endif
#ifdef CONFIG_PROXIMITY_SENSOR
#include <linux/gp2a.h>
#endif
#ifdef CONFIG_MAX17048_FUELGAUGE
#include <linux/fuelgauge_max17048.h>
#endif
#ifdef CONFIG_KEYBOARD_GPIO
#include <linux/gpio_keys.h>
#endif

#ifndef CONFIG_BT_CSR_7820
#define CONFIG_BT_CSR_7820
#endif

/* GGSM sc47.yun CSR7820 Project 2012.04.23 */
#ifdef CONFIG_BT_CSR_7820
//#include <../../../drivers/bluetooth/bluesleep.c> //SAMSUNG_BT_CONFIG

#define GPIO_WLAN_LEVEL_LOW			0
#define GPIO_WLAN_LEVEL_HIGH			1
#define GPIO_WLAN_LEVEL_NONE			2
#endif
/* GGSM sc47.yun CSR7820 Project end */

#define PMEM_KERNEL_EBI1_SIZE	0x3A000
#define MSM_PMEM_AUDIO_SIZE	0x5B000

int charging_boot;
EXPORT_SYMBOL(charging_boot);
int	fota_boot;
EXPORT_SYMBOL(fota_boot);


#define WLAN_33V_CONTROL_FOR_BT_ANTENNA

#define WLAN_OK (0)
#define WLAN_ERROR (-1)

#ifdef WLAN_33V_CONTROL_FOR_BT_ANTENNA
#define WLAN_33V_WIFI_FLAG (0x01)
#define WLAN_33V_BT_FLAG (0x02)

int wlan_33v_flag;

int wlan_setup_ldo_33v(int input_flag, int on);
#endif
#ifdef CONFIG_SAMSUNG_JACK

#define GPIO_JACK_S_35	48
#define GPIO_SEND_END	92
#define GPIO_RST_BT	77
#define GPIO_BT_EN	82

static struct sec_jack_zone jack_zones[] = {
	[0] = {
		.adc_high	= 3,
		.delay_ms	= 10,
		.check_count	= 5,
		.jack_type	= SEC_HEADSET_3POLE,
	},
	[1] = {
		.adc_high	= 99,
		.delay_ms	= 10,
		.check_count	= 10,
		.jack_type	= SEC_HEADSET_3POLE,
	},
	[2] = {
		.adc_high	= 9999,
		.delay_ms	= 10,
		.check_count	= 5,
		.jack_type	= SEC_HEADSET_4POLE,
	},
};


int get_msm7x27a_det_jack_state(void)
{
	/* Active Low */
	return(gpio_get_value(GPIO_JACK_S_35)) ^ 1;

}
EXPORT_SYMBOL(get_msm7x27a_det_jack_state);

static int get_msm7x27a_send_key_state(void)
{
	/* Active High */
	//return(gpio_get_value(GPIO_SEND_END));
#ifdef CONFIG_MACH_KYLE
	return current_key_state;
#else
	return 0;
#endif
}

#define SMEM_PROC_COMM_MICBIAS_ONOFF		PCOM_OEM_MICBIAS_ONOFF
#define SMEM_PROC_COMM_MICBIAS_ONOFF_REG5	PCOM_OEM_MICBIAS_ONOFF_REG5
#define SMEM_PROC_COMM_GET_ADC				PCOM_OEM_SAMSUNG_GET_ADC

enum {
	SMEM_PROC_COMM_GET_ADC_BATTERY = 0x0,
	SMEM_PROC_COMM_GET_ADC_TEMP,
	SMEM_PROC_COMM_GET_ADC_VF,
	SMEM_PROC_COMM_GET_ADC_ALL, // data1 : VF(MSB 2 bytes) vbatt_adc(LSB 2bytes), data2 : temp_adc
	SMEM_PROC_COMM_GET_ADC_EAR_ADC,		// 3PI_ADC
	SMEM_PROC_COMM_GET_ADC_MAX,
};

enum {
	SMEM_PROC_COMM_MICBIAS_CONTROL_OFF = 0x0,
	SMEM_PROC_COMM_MICBIAS_CONTROL_ON,
	SMEM_PROC_COMM_MICBIAS_CONTROL_MAX
};

static void set_msm7x27a_micbias_state_reg5(bool state)
{
	/* int res = 0;
	 * int data1 = 0;
	 * int data2 = 0;
	 * if (!state)
	 * {
		 * data1 = SMEM_PROC_COMM_MICBIAS_CONTROL_OFF;
		 * res = msm_proc_comm(SMEM_PROC_COMM_MICBIAS_ONOFF_REG5, &data1, &data2);
		 * if(res < 0)
		 * {
			 * pr_err("sec_jack: micbias_reg5 %s  fail \n",state?"on":"off");
		 * }
	 * } */
}

static bool cur_state = false;
static bool proximity_init;

static void set_msm7x27a_micbias_state(bool state)
{

	if(cur_state == state)
	{
		pr_info("sec_jack : earmic_bias same as cur_state\n");
		return;
	}

	if(state)
	{
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_ALWAYS);
		msleep(130);
		cur_state = true;
	}
	else
	{
		pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);
		cur_state = false;
	}

	report_headset_status(state);

	pr_info("sec_jack : earmic_bias %s\n", state?"on":"off");

}

#if defined(JACK_WATERPROOF)
static void update_msm7x27a_earjack_type(bool state)
{
	report_headset_status(state);

	pr_info("sec_jack : update earjack type\n");
}
#endif

#if 1
static int sec_jack_get_adc_value(void)
{
	return current_jack_type;//Kuldeep Commented For porting
}
#endif


void sec_jack_gpio_init(void)
{
	gpio_tlmm_config(GPIO_CFG(GPIO_JACK_S_35, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE); //gpio 48 JACK_INT_N

	if(gpio_request(GPIO_JACK_S_35, "h2w_detect")<0)
		pr_err("sec_jack:gpio_request fail\n");
	if(gpio_direction_input(GPIO_JACK_S_35)<0)
		pr_err("sec_jack:gpio_direction fail\n");
}

static struct sec_jack_platform_data sec_jack_data = {
	.get_det_jack_state	= get_msm7x27a_det_jack_state,
	.get_send_key_state	= get_msm7x27a_send_key_state,
	.set_micbias_state	= set_msm7x27a_micbias_state,
	.set_micbias_state_reg5	= set_msm7x27a_micbias_state_reg5,
#if defined(JACK_WATERPROOF)
	.update_earjack_type	= update_msm7x27a_earjack_type,
#endif
	.get_adc_value	= sec_jack_get_adc_value,
	.zones		= jack_zones,
	.num_zones	= ARRAY_SIZE(jack_zones),
	.det_int	= MSM_GPIO_TO_INT(GPIO_JACK_S_35),
	.send_int	= MSM_GPIO_TO_INT(GPIO_SEND_END),
};

static struct platform_device sec_device_jack = {
	.name           = "sec_jack",
	.id             = -1,
	.dev            = {
		.platform_data  = &sec_jack_data,
	},
};
#endif
#if defined(CONFIG_GPIO_SX150X)
enum {
	SX150X_CORE,
};

static struct sx150x_platform_data sx150x_data[] __initdata = {
	[SX150X_CORE]	= {
		.gpio_base		= GPIO_CORE_EXPANDER_BASE,
		.oscio_is_gpo		= false,
		.io_pullup_ena		= 0,
		.io_pulldn_ena		= 0x02,
		.io_open_drain_ena	= 0xfef8,
		.irq_summary		= -1,
	},
};
#endif

extern unsigned int board_hw_revision;
extern unsigned int kernel_uart_flag;


#ifndef ATH_POLLING
static void (*wlan_status_notify_cb)(int card_present, void *dev_id);
void *wlan_devid;

static int register_wlan_status_notify(void (*callback)(int card_present, void *dev_id), void *dev_id)
{
	printk("%s --enter\n", __func__);

	wlan_status_notify_cb = callback;
	wlan_devid = dev_id;
	return 0;
}

static unsigned int wlan_status(struct device *dev)
{
	int rc;

	printk("%s entered\n", __func__);

	rc = gpio_get_value(GPIO_WLAN_RESET_N/*gpio_wlan_reset_n*/);

	return rc;
}
#endif /* ATH_POLLING */

static struct platform_device msm_wlan_ar6000_pm_device = {
	.name           = "wlan_ar6000_pm_dev",
	.id             = -1,
};

static struct platform_device msm_device_pmic_leds = {
	.name	= "pmic-leds",
	.id		= -1,
};

#if defined(CONFIG_I2C) && defined(CONFIG_GPIO_SX150X)
static struct i2c_board_info core_exp_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sx1509q", 0x3e),
	},
};

static struct platform_device msm_vibrator_device = {
	.name	= "msm_vibrator",
	.id		= -1,
};

#ifdef CONFIG_PROXIMITY_SENSOR
static int gp2a_power(bool on)
{
 #ifdef	CONFIG_MACH_KYLE_I
	int rc;
	struct vreg *l1 = NULL;

	printk(KERN_DEBUG "start %s\n", __func__);
	if (l1 == NULL) {
		l1 = regulator_get(NULL, "ldo1");

		if (IS_ERR(l1)) {
			printk(KERN_DEBUG "%s: vreg_get(%s) failed (%ld)\n"
			, __func__, "ldo1", PTR_ERR(l1));
			return 0;
		}

		rc = regulator_set_voltage(l1, 3000000, 3000000);
		if (rc) {
			printk(KERN_DEBUG "%s: proximity set_level failed (%d)\n"
			, __func__, rc);
		}
	}

	if (on) {
		rc = regulator_enable(l1);
		if (rc) {
			printk(KERN_DEBUG "%s: proximity enable failed (%d)\n"
			, __func__, rc);
		} else {
			printk(KERN_DEBUG "%s: proximity enable success (%d)\n"
			, __func__, rc);
		}
	} else {
		rc = regulator_disable(l1);
		if (rc) {
			printk(KERN_DEBUG "%s: proximity disable failed (%d)\n"
			, __func__, rc);
		} else {
			printk(KERN_DEBUG "%s: proximity disable success (%d)\n"
			, __func__, rc);
		}
	}
	gpio_tlmm_config(
				GPIO_CFG(29 , 0 ,
					GPIO_CFG_INPUT,
					GPIO_CFG_PULL_UP,
					GPIO_CFG_2MA),
				       GPIO_CFG_ENABLE);
#else //CONFIG_MACH_KYLE_I

#ifdef CONFIG_MACH_AMAZING
	gpio_tlmm_config(
				GPIO_CFG(29,0,
					 GPIO_CFG_INPUT,
					 GPIO_CFG_PULL_UP,
					 GPIO_CFG_2MA),
				GPIO_CFG_ENABLE);
#else
	int rc = 0;
    int proximity_init = false;
//	if(board_hw_revision >= 0x06) {
	if(1) {
		if (proximity_init == false) {
			pr_info("[GP2A board hw revision %d\n",
					board_hw_revision);
			struct pm8xxx_gpio_rpc_cfg gpio_cfg = {
				.gpio = PMIC_GPIO_11,
				.mode = OUTPUT_ON,
				.src_pull = PULL_UP_1_5uA,
				.volt_src = PMIC_GPIO_VIN2,
				.buf_config = CONFIG_CMOS,
			};

			rc = pmic_gpio_config(&gpio_cfg);
			if (rc < 0) {
				pr_err("%s pmic gpio config failed %d ",
						__func__,
						rc);
			}
			pmic_gpio_direction_output(PMIC_GPIO_11);
			proximity_init = true;
			gpio_tlmm_config(
					GPIO_CFG(29,0,
						GPIO_CFG_INPUT,
						GPIO_CFG_PULL_UP,
						GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
		}

		 if (on) {
	             pr_err("%s pmic gpio set to 1 ",
			              __func__);
                 rc = pmic_gpio_set_value(PMIC_GPIO_11, 1);
	             if (rc < 0)
	                 pr_err("%s pmic gpio set 1 error ",
	                     __func__);
	         } else {
	             pr_err("%s pmic gpio set to 0 ",
				                 __func__);
	             rc = pmic_gpio_set_value(PMIC_GPIO_11, 0);
	             if (rc < 0)
	                 pr_err("%s pmic gpio set 0 error ",
			                    __func__);
			 }
	}else {
		gpio_tlmm_config(
				GPIO_CFG(29,0,
					 GPIO_CFG_INPUT,
					 GPIO_CFG_PULL_UP,
					 GPIO_CFG_2MA),
				GPIO_CFG_ENABLE);
	}
printk("End of gp2a function");
#endif
#endif//CONFIG_MACH_KYLE_I

		return 0;
}




	static struct gp2a_platform_data gp2a_pdata = {
		.p_out =29,
		.power =gp2a_power,
	};
#endif



static struct i2c_board_info sensor_devices[] = {
#ifdef CONFIG_SENSORS_BMA222
{
	I2C_BOARD_INFO("bma222", 0x08),
},
#endif
#ifdef CONFIG_SENSORS_BMA222E
{
	I2C_BOARD_INFO("bma222e", 0x18),
},
#endif
#ifdef CONFIG_PROXIMITY_SENSOR
{
	I2C_BOARD_INFO("gp2a", 0x44 ),
	.platform_data = &gp2a_pdata,
},
#endif
#ifdef CONFIG_SENSORS_HSCD
{
	I2C_BOARD_INFO("hscd_i2c", 0x0c),
},
#endif
};


#if defined(CONFIG_SENSORS_BMA222E) \
	|| defined(CONFIG_PROXIMITY_SENSOR) || defined(CONFIG_SENSORS_BMA222)
static struct i2c_gpio_platform_data sensor_i2c_gpio_data = {
	.sda_pin =GPIO_SENSOR_SDA,
	.scl_pin =GPIO_SENSOR_SCL,
	.udelay =1,
};

static struct platform_device sensor_i2c_gpio_device = {
	.name ="i2c-gpio",
	.id= 4,
	.dev = {
		.platform_data =&sensor_i2c_gpio_data,
	},
};
#endif




static struct i2c_gpio_platform_data touch_i2c_gpio_data = {
	.sda_pin    = GPIO_TSP_SDA,
	.scl_pin    = GPIO_TSP_SCL,
	.udelay	= 1,
};
static struct platform_device touch_i2c_gpio_device = {
	.name       = "i2c-gpio",
	.id     =  2,
	.dev        = {
		.platform_data  = &touch_i2c_gpio_data,
	},
};
/* I2C 2 */
static struct i2c_board_info touch_i2c_devices[] = {
	{
		I2C_BOARD_INFO("sec_touch", 0x48),
	        .irq = MSM_GPIO_TO_INT( GPIO_TOUCH_IRQ ),
	},
};



static void __init register_i2c_devices(void)
{
	if (machine_is_msm7x27a_surf() || machine_is_msm7625a_surf())
		sx150x_data[SX150X_CORE].io_open_drain_ena = 0xe0f0;

	core_exp_i2c_info[0].platform_data =
			&sx150x_data[SX150X_CORE];

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				core_exp_i2c_info,
				ARRAY_SIZE(core_exp_i2c_info));
#if defined(CONFIG_SENSORS_BMA222E) \
	|| defined(CONFIG_PROXIMITY_SENSOR) || defined(CONFIG_SENSORS_BMA222)
	i2c_register_board_info(4,sensor_devices, ARRAY_SIZE(sensor_devices));
	printk("[registration of devices] done\n");
#endif

}
#endif

struct msm_battery_callback *charger_callbacks;
static enum cable_type_t set_cable_status;
static enum acc_type_t set_acc_status;
static enum ovp_type_t set_ovp_status;

static void msm_battery_register_callback(
		struct msm_battery_callback *ptr)
{
	charger_callbacks = ptr;
	pr_info("[BATT] msm_battery_register_callback start\n");
	if ((set_acc_status != 0) && charger_callbacks
		&& charger_callbacks->set_acc_type)
		charger_callbacks->set_acc_type(charger_callbacks,
		set_acc_status);

	if ((set_cable_status != 0) && charger_callbacks
		&& charger_callbacks->set_cable)
		charger_callbacks->set_cable(charger_callbacks,
		set_cable_status);

	if ((set_ovp_status != 0) && charger_callbacks
		&& charger_callbacks->set_ovp_type)
		charger_callbacks->set_ovp_type(charger_callbacks,
		set_ovp_status);
}

static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_charger_data aries_charger = {
	.register_callbacks	= msm_battery_register_callback,
};

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.charger			= &aries_charger,
	.voltage_min_design		= 2800,
	.voltage_max_design	= 4300,
	.avail_chg_sources		= AC_CHG | USB_CHG ,
	.batt_technology		= POWER_SUPPLY_TECHNOLOGY_LION,
};

static struct platform_device msm_batt_device = {
	.name		= "msm-battery",
	.id		= -1,
	.dev.platform_data	= &msm_psy_batt_data,
};

/*
static u32 msm_calculate_batt_capacity(u32 current_voltage) {
	u32 low_voltage  = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage = msm_psy_batt_data.voltage_max_design;

	return (current_voltage - low_voltage) * 100
			/ (high_voltage - low_voltage);
}*/


int fsa_cable_type = CABLE_TYPE_UNKNOWN;

int fsa880_get_charger_status(void);
int fsa880_get_charger_status(void)
{
	return fsa_cable_type;
}

void trebon_chg_connected(enum chg_type chgtype)
{
	char *chg_types[] = {"STD DOWNSTREAM PORT",
			"CARKIT",
			"DEDICATED CHARGER",
			"INVALID"};
	unsigned *data1 = NULL;
	unsigned *data2 = NULL;
	int ret = 0;

	switch (chgtype) {
	case USB_CHG_TYPE__SDP:
		ret = msm_proc_comm(PCOM_CHG_USB_IS_PC_CONNECTED,
				data1, data2);
		break;
	case USB_CHG_TYPE__WALLCHARGER:
		ret = msm_proc_comm(PCOM_CHG_USB_IS_CHARGER_CONNECTED,
				data1, data2);
		break;
	case USB_CHG_TYPE__INVALID:
		ret = msm_proc_comm(PCOM_CHG_USB_IS_DISCONNECTED,
				data1, data2);
		break;
	default:
		break;
	}

	if (ret < 0)
		pr_err("%s: connection err, ret=%d\n", __func__, ret);

	pr_info("\nCharger Type: %s\n", chg_types[chgtype]);
}

static void jena_usb_cb(u8 attached, struct fsausb_ops *ops)
{
	pr_info("[BATT] [%s] Board file [FSA880]: USB Callback\n", __func__);

	set_acc_status = attached ? ACC_TYPE_USB : ACC_TYPE_NONE;
	if (charger_callbacks && charger_callbacks->set_acc_type)
		charger_callbacks->set_acc_type(charger_callbacks,
		set_acc_status);

	set_cable_status = attached ? CABLE_TYPE_USB : CABLE_TYPE_UNKNOWN;
	if (charger_callbacks && charger_callbacks->set_cable)
		charger_callbacks->set_cable(charger_callbacks,
		set_cable_status);
}

extern  void charger_enable(int enable);
static void jena_charger_cb(u8 attached, struct fsausb_ops *ops)
{
	pr_info("[BATT] Board file [FSA880]: Charger Callback\n");

	set_acc_status = attached ? ACC_TYPE_CHARGER : ACC_TYPE_NONE;
	if (charger_callbacks && charger_callbacks->set_acc_type)
		charger_callbacks->set_acc_type(charger_callbacks,
		set_acc_status);

	set_cable_status = attached ? CABLE_TYPE_TA : CABLE_TYPE_UNKNOWN;
	if (charger_callbacks && charger_callbacks->set_cable)
		charger_callbacks->set_cable(charger_callbacks,
		set_cable_status);

	charger_enable(set_cable_status);
}

static void jena_jig_cb(u8 attached, struct fsausb_ops *ops)
{
	pr_info("[BATT] Board file [FSA880]: Jig Callback\n");

	set_acc_status = attached ? ACC_TYPE_JIG : ACC_TYPE_NONE;
	if (charger_callbacks && charger_callbacks->set_acc_type)
		charger_callbacks->set_acc_type(charger_callbacks,
		set_acc_status);
}

static void jena_ovp_cb(u8 attached, struct fsausb_ops *ops)
{
	pr_info("[BATT] Board file [FSA880]: OVP Callback\n");

	set_ovp_status = attached ? OVP_TYPE_OVP : OVP_TYPE_NONE;
	if (charger_callbacks && charger_callbacks->set_ovp_type)
		charger_callbacks->set_ovp_type(charger_callbacks,
		set_ovp_status);
}
/* check charger cable type for USB phy off */
static int checkChargerType()
{
	return set_cable_status;
}

static void jena_fsa880_reset_cb(void)
{
	pr_info(" [BATT] Board file [FSA880]: Reset Callback\n");
}


/* For uUSB Switch */
static struct fsausb_platform_data jena_fsa880_pdata = {
       .intb_gpio      = MSM_GPIO_TO_INT(GPIO_MUSB_INT),
       .usb_cb         = jena_usb_cb,
       .uart_cb        = NULL,
       .charger_cb     = jena_charger_cb,
       .jig_cb         = jena_jig_cb,
	.ovp_cb		= jena_ovp_cb,
       .reset_cb       = jena_fsa880_reset_cb,
};

/* I2C 3 */
static struct i2c_gpio_platform_data fsa880_i2c_gpio_data = {
	.sda_pin    = GPIO_MUS_SDA,
	.scl_pin    = GPIO_MUS_SCL,
};

static struct platform_device fsa880_i2c_gpio_device = {
	.name       = "i2c-gpio",
	.id     =  3,
	.dev        = {
		.platform_data  = &fsa880_i2c_gpio_data,
	},
};

static struct i2c_board_info fsa880_i2c_devices[] = {
	{
		I2C_BOARD_INFO("FSA9280", 0x4A >> 1),
		.platform_data =  &jena_fsa880_pdata,
		.irq = MSM_GPIO_TO_INT(GPIO_MUSB_INT),
	},
};
#ifdef CONFIG_BQ27425_FUEL_GAUGE


#define FUEL_I2C_SCL 79
#define FUEL_I2C_SDA 78


/* Fuel_gauge */
static struct i2c_gpio_platform_data fuelgauge_i2c_gpio_data = {
	.sda_pin = FUEL_I2C_SDA,
	.scl_pin = FUEL_I2C_SCL,
};

static struct platform_device fuelgauge_i2c_gpio_device = {
	.name	= "i2c-gpio",
	.id	= 6,
	.dev	= {
	.platform_data	= &fuelgauge_i2c_gpio_data,
	},
};
static struct i2c_board_info fg_i2c_devices[] = {
	{
		I2C_BOARD_INFO( "bq27425", 0xAA>>1 ),
	},
};
#endif


#ifdef CONFIG_MAX17048_FUELGAUGE

#define FUEL_I2C_SCL 78
#define FUEL_I2C_SDA 79

static int max17048_low_batt_cb(void)
{
	pr_err("%s: Low battery alert\n", __func__);
	struct power_supply *psy = power_supply_get_by_name("battery");
	union power_supply_propval value;
	if (!psy) {
		pr_err("%s: fail to get battery ps\n", __func__);
		return -ENODEV;
	}
	value.intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	return psy->set_property(psy, POWER_SUPPLY_PROP_CAPACITY_LEVEL, &value);
}
int check_battery_type(void)
{
	return BATT_TYPE_D2_ACTIVE;
}
static int max17048_power_supply_register(struct device *parent,
	struct power_supply *psy)
{
	aries_charger.psy_fuelgauge = psy;
	return 0;
}

static void max17048_power_supply_unregister(struct power_supply *psy)
{
	aries_charger.psy_fuelgauge = NULL;
}

/* Fuel_gauge */
static struct i2c_gpio_platform_data fg_smb_i2c_gpio_data = {
	.sda_pin = FUEL_I2C_SDA,
	.scl_pin = FUEL_I2C_SCL,
};

static struct platform_device fg_smb_i2c_gpio_device = {
	.name	= "i2c-gpio",
	.id	= 6,
	.dev	= {
	.platform_data	= &fg_smb_i2c_gpio_data,
	},
};

static struct max17048_platform_data max17048_pdata = {
	.low_batt_cb = NULL, /* max17048_low_batt_cb, */
	.check_batt_type = check_battery_type,
	.power_supply_register = max17048_power_supply_register,
	.power_supply_unregister = max17048_power_supply_unregister,
	.rcomp_value = 0x501c,
};

static struct i2c_board_info fg_smb_i2c_devices[] = {
    {
		I2C_BOARD_INFO("max17048", 0x6D>>1),
	    .platform_data = &max17048_pdata,
		.irq = NULL, /* MSM_GPIO_TO_INT(GPIO_FUEL_INT), */
    },
#ifdef CONFIG_CHARGER_SMB328A
    {
            I2C_BOARD_INFO("smb328a", (0x69 >> 1)),
	    .platform_data = &smb328a_pdata,
            .irq = MSM_GPIO_TO_INT(18),
    },
#endif

};
#endif
static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(60, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
};

static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(60, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
};

static void gsbi_qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc;

	if (adap_id < 0 || adap_id > 1)
		return;

	/* Each adapter gets 2 lines from the table */
	if (config_type)
		rc = msm_gpios_request_enable(&qup_i2c_gpios_hw[adap_id*2], 2);
	else
		rc = msm_gpios_request_enable(&qup_i2c_gpios_io[adap_id*2], 2);
	if (rc < 0)
		pr_err("QUP GPIO request/enable failed: %d\n", rc);
}

static struct msm_i2c_platform_data msm_gsbi0_qup_i2c_pdata = {
#if defined(CONFIG_MACH_KYLE)
	.clk_freq		= 400000,
#else
	.clk_freq		= 100000,
#endif
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi1_qup_i2c_pdata = {
	.clk_freq		= 100000,
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

#ifdef CONFIG_ARCH_MSM7X27A
#define MSM_PMEM_MDP_SIZE       0x2300000
#if defined(CONFIG_MACH_KYLE)
#define MSM_PMEM_ADSP_SIZE      0x1200000
#else
#define MSM_PMEM_ADSP_SIZE      0x1100000
#endif
#if CONFIG_FB_MSM_TRIPLE_BUFFER
/* prim = 320 x 480 x 4(bpp) x 3(pages) */
#define MSM_FB_SIZE             (800 * 480 * 4 * 3)
#else
/* prim = 320 x 480 x 4(bpp) x 2(pages) */
#define MSM_FB_SIZE             (800 * 480 * 4 * 2)
#endif /* CONFIG_FB_MSM_TRIPLE_BUFFER */
#else
#define MSM_PMEM_MDP_SIZE       0x1DD1000
#define MSM_PMEM_ADSP_SIZE      0x1000000
#define MSM_FB_SIZE             0x195000
#endif /* CONFIG_ARCH_MSM7X27A */




static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};
static int __init boot_mode_boot(char *onoff)
{
	if (strncmp(onoff, "true", 4) == 0) {
		charging_boot = 1;
		fota_boot = 0;
		pr_info("%s[BATT]charging_boot: %d\n",
			__func__, charging_boot);
	} else if (strcmp(onoff, "fota") == 0) {
		fota_boot = 1;
		charging_boot = 0;
	} else {
		charging_boot = 0;
		fota_boot = 0;
	}
	return 1;
}
__setup("androidboot.bootchg=", boot_mode_boot);


#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	int rc = 0;
	unsigned gpio;

	gpio = GPIO_HOST_VBUS_EN;

	rc = gpio_request(gpio, "i2c_host_vbus_en");
	if (rc < 0) {
		pr_err("failed to request %d GPIO\n", gpio);
		return;
	}
	gpio_direction_output(gpio, !!on);
	gpio_set_value_cansleep(gpio, !!on);
	gpio_free(gpio);
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info       = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
};

static void __init msm7x2x_init_host(void)
{
	msm_add_host(0, &msm_usb_host_pdata);
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static struct regulator *reg_hsusb;
static int msm_hsusb_ldo_init(int init)
{
	int rc = 0;

	if (init) {
		reg_hsusb = regulator_get(NULL, "usb");
		if (IS_ERR(reg_hsusb)) {
			rc = PTR_ERR(reg_hsusb);
			pr_err("%s: sandeep could not get regulator: %d\n",
					__func__, rc);
			goto out;
		}

		rc = regulator_set_voltage(reg_hsusb, 3300000, 3300000);
		if (rc) {
			pr_err("%s:sandeep could not set voltage: %d\n",
					__func__, rc);
			goto reg_free;
		}

		return 0;
	}
	/* else fall through */
reg_free:
	regulator_put(reg_hsusb);
out:
	reg_hsusb = NULL;
	return rc;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (IS_ERR_OR_NULL(reg_hsusb))
		return reg_hsusb ? PTR_ERR(reg_hsusb) : -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	return enable ?
		regulator_enable(reg_hsusb) :
		regulator_disable(reg_hsusb);
}

#ifndef CONFIG_USB_EHCI_MSM_72K
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret = 0;

	if (init)
		ret = msm_pm_app_rpc_init(callback);
	else
		msm_pm_app_rpc_deinit(callback);

	return ret;
}
#endif

static struct msm_otg_platform_data msm_otg_pdata = {
#ifndef CONFIG_USB_EHCI_MSM_72K
	.pmic_vbus_notif_init	 = msm_hsusb_pmic_notif_init,
#else
	.vbus_power		 = msm_hsusb_vbus_power,
#endif
	.rpc_connect		 = hsusb_rpc_connect,
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_75_PERCENT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.chg_init		 = hsusb_chg_init,
	/* check charger cable type for USB phy off */
//	.chg_connect_type = checkChargerType,
	/* XXX: block charger current setting */
	.chg_connected		 = hsusb_chg_connected,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
};
#endif

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start = 0x90000300,
		.end   = 0x900003ff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MSM_GPIO_TO_INT(4),
		.end   = MSM_GPIO_TO_INT(4),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

#define WLAN_HOST_WAKE


#ifdef WLAN_HOST_WAKE
struct wlansleep_info {
	unsigned host_wake;
	unsigned host_wake_irq;
	struct wake_lock wake_lock;
};


static struct wlansleep_info *wsi;
static struct tasklet_struct hostwake_task;


static void wlan_hostwake_task(unsigned long data)
{
	printk(KERN_INFO "WLAN: wake lock timeout 0.5 sec...\n");

	wake_lock_timeout(&wsi->wake_lock, HZ / 2);
}


static irqreturn_t wlan_hostwake_isr(int irq, void *dev_id)
{
//please fix    gpio_clear_detect_status(wsi->host_wake_irq);

	/* schedule a tasklet to handle the change in the host wake line */
	tasklet_schedule(&hostwake_task);
	return IRQ_HANDLED;
}


static int wlan_host_wake_init(void)
{
	int ret;

	gpio_tlmm_config(GPIO_CFG(42, 0, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);

	wsi = kzalloc(sizeof(struct wlansleep_info), GFP_KERNEL);
	if (!wsi)
		return -ENOMEM;

	wake_lock_init(&wsi->wake_lock, WAKE_LOCK_SUSPEND, "bluesleep");
	tasklet_init(&hostwake_task, wlan_hostwake_task, 0);

	wsi->host_wake = GPIO_WLAN_HOST_WAKE;
	wsi->host_wake_irq = MSM_GPIO_TO_INT(wsi->host_wake);

//please fix    gpio_configure(wsi->host_wake, GPIOF_INPUT);
	ret = request_irq(wsi->host_wake_irq, wlan_hostwake_isr,
						IRQF_DISABLED | IRQF_TRIGGER_RISING,
						"wlan hostwake", NULL);
	if (ret < 0) {
		printk(KERN_ERR "WLAN: Couldn't acquire WLAN_HOST_WAKE IRQ");
		return -1;
	}

	ret = enable_irq_wake(wsi->host_wake_irq);
	if (ret < 0) {
		printk(KERN_ERR "WLAN: Couldn't enable WLAN_HOST_WAKE as wakeup interrupt");
		free_irq(wsi->host_wake_irq, NULL);
		return -1;
	}

	return 0;
}


static void wlan_host_wake_exit(void)
{
	if (disable_irq_wake(wsi->host_wake_irq))
		printk(KERN_ERR "WLAN: Couldn't disable hostwake IRQ wakeup mode \n");

	free_irq(wsi->host_wake_irq, NULL);

	wake_lock_destroy(&wsi->wake_lock);
	kfree(wsi);
}
#endif /* WLAN_HOST_WAKE */


static int wlan_set_gpio(unsigned gpio, int on)
{
	int rc = 0;
	int gpio_value = 0;

	printk("%s - %d : %s\n", __func__, gpio, on ? "on" : "off");

	// Request
	if (gpio_request(gpio, "wlan_ar6000_pm")) {
		printk(KERN_ERR "%s: gpio_request for %d failed\n",
				__func__, gpio);
		return -1;
	}

	gpio_value = gpio_get_value(gpio);
	printk(KERN_INFO "%s: before (%d) :: gpio_get_value = %d",
			__func__, on, gpio_value);

	// Direction Output On/Off
	rc = gpio_direction_output(gpio, on);
	gpio_free(gpio);

	gpio_value = gpio_get_value(gpio);
	printk(KERN_INFO "%s: after (%d) :: gpio_get_value = %d",
			__func__, on, gpio_value);

	if (rc) {
		printk(KERN_ERR "%s: gpio_direction_output for %d failed\n",
				__func__, gpio);
		return -1;
	}

	return 0;
}


#ifdef WLAN_33V_CONTROL_FOR_BT_ANTENNA
int wlan_setup_ldo_33v(int input_flag, int on)
{
	int skip = 0;
	int temp_flag = wlan_33v_flag;

	printk(KERN_INFO "%s - set by %s : %s\n",
			__func__,
			(input_flag == WLAN_33V_WIFI_FLAG) ? "Wifi" : "BT",
			on ? "on" : "off");
	printk(KERN_INFO "%s - old wlan_33v_flag : %d\n",
			__func__, temp_flag);

	if (on) {
		if (temp_flag)  /* Already On */
			skip = 1;

		temp_flag |= input_flag;
	} else {
		temp_flag &= (~input_flag);

		/* Keep GPIO_WLAN_33V_EN on if either BT or Wifi is turned on*/
		if (temp_flag)
			skip = 1;
	}

	printk(KERN_INFO "%s - new wlan_33v_flag : %d\n",
			__func__, temp_flag);

	if (skip) {
		printk(KERN_INFO "%s - Skip GPIO_WLAN_33V_EN %s\n",
				__func__, on ? "on" : "off");
	} else {
		/* GPIO_WLAN_33V_EN - On / Off */
		if (wlan_set_gpio(GPIO_WLAN_33V_EN, on))
			return WLAN_ERROR;
	}

	wlan_33v_flag = temp_flag;

	return WLAN_OK;
}
#endif

void wlan_setup_power(int on, int detect)
{
	printk("%s %s --enter\n", __func__, on ? "on" : "down");

	if (on) {
#ifdef WLAN_33V_CONTROL_FOR_BT_ANTENNA
		/* GPIO_WLAN_33V_EN - On */
		if (wlan_setup_ldo_33v(WLAN_33V_WIFI_FLAG, 1))
			return;
#endif

		udelay(60);

		// GPIO_WLAN_RESET_N - On
		if (wlan_set_gpio(GPIO_WLAN_RESET_N, 1))
			return;

#ifdef WLAN_HOST_WAKE
		wlan_host_wake_init();
#endif /* WLAN_HOST_WAKE */
	}
	else {
#ifdef WLAN_HOST_WAKE
		wlan_host_wake_exit();
#endif /* WLAN_HOST_WAKE */

		// GPIO_WLAN_RESET_N - Off
		if (wlan_set_gpio(GPIO_WLAN_RESET_N, 0))
			return;

		udelay(60);

#ifdef WLAN_33V_CONTROL_FOR_BT_ANTENNA
		/* GPIO_WLAN_33V_EN - Off */
		if (wlan_setup_ldo_33v(WLAN_33V_WIFI_FLAG, 0))
			return;
#endif
	}

#ifndef ATH_POLLING
	mdelay(100);

	if (detect) {
		/* Detect card */
		if (wlan_status_notify_cb)
			wlan_status_notify_cb(on, wlan_devid);
		else
			printk(KERN_ERR "WLAN: No notify available\n");
	}
#endif /* ATH_POLLING */
}
EXPORT_SYMBOL(wlan_setup_power);
EXPORT_SYMBOL(board_hw_revision);


static int wlan_power_init(void)
{
#ifdef WLAN_33V_CONTROL_FOR_BT_ANTENNA
	wlan_33v_flag = 0;
#endif

	/* Set config - GPIO_WLAN_33V_EN */
	if (gpio_tlmm_config(GPIO_CFG(GPIO_WLAN_33V_EN, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
			GPIO_CFG_2MA), GPIO_CFG_ENABLE)) {
		printk(KERN_ERR "%s: gpio_tlmm_config for %d failed\n",
				__func__, GPIO_WLAN_33V_EN);
		return WLAN_ERROR;
	}

	return WLAN_OK;
}

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

static unsigned long vreg_sts, gpio_sts;
static struct vreg *vreg_mmc;
static struct vreg *vreg_emmc;

struct sdcc_vreg {
	struct vreg *vreg_data;
	unsigned level;
};

static struct sdcc_vreg sdcc_vreg_data[4];

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};

/**
 * Due to insufficient drive strengths for SDC GPIO lines some old versioned
 * SD/MMC cards may cause data CRC errors. Hence, set optimal values
 * for SDC slots based on timing closure and marginality. SDC1 slot
 * require higher value since it should handle bad signal quality due
 * to size of T-flash adapters.
 */
static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(51, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_3"},
	{GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_2"},
	{GPIO_CFG(53, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_1"},
	{GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_0"},
	{GPIO_CFG(55, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_cmd"},
	{GPIO_CFG(56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_14MA),
								"sdc1_clk"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(62, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc2_clk"},
	{GPIO_CFG(63, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_cmd"},
	{GPIO_CFG(64, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_3"},
	{GPIO_CFG(65, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_2"},
	{GPIO_CFG(66, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_1"},
	{GPIO_CFG(67, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_0"},
};

static struct msm_gpio sdc2_sleep_cfg_data[] = {
	{GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
								"sdc2_clk"},
	{GPIO_CFG(63, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_cmd"},
	{GPIO_CFG(64, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_3"},
	{GPIO_CFG(65, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_2"},
	{GPIO_CFG(66, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_1"},
	{GPIO_CFG(67, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_0"},
};
static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(88, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc3_clk"},
	{GPIO_CFG(89, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_cmd"},
	{GPIO_CFG(90, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_3"},
	{GPIO_CFG(91, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_2"},
	{GPIO_CFG(92, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_1"},
	{GPIO_CFG(93, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_0"},
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	{GPIO_CFG(19, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_7"},
	{GPIO_CFG(20, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_6"},
	{GPIO_CFG(21, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_5"},
	{GPIO_CFG(108, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_4"},
#endif
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(19, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_3"},
	{GPIO_CFG(20, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_2"},
	{GPIO_CFG(21, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_1"},
	{GPIO_CFG(106, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_cmd"},
	{GPIO_CFG(108, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_0"},
/*	{GPIO_CFG(109, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_clk"}, */
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
	},
	{
/*		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = sdc2_sleep_cfg_data,
*/
	},
	{
/*		.cfg_data = sdc3_cfg_data,		*/
/*		.size = ARRAY_SIZE(sdc3_cfg_data),	*/
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
	},
};

static int msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];
	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			pr_err("%s: Failed to turn on GPIOs for slot %d\n",
					__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			rc = msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
			return rc;
		}
		msm_gpios_disable_free(curr->cfg_data, curr->size);
	}
	return rc;
}

static int msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_vreg *curr;

	curr = &sdcc_vreg_data[dev_id - 1];

	printk("%s : %d : %d : level : %d\n", __func__, dev_id, enable, curr->level);

	if (!(test_bit(dev_id, &vreg_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &vreg_sts);
		rc = vreg_set_level(curr->vreg_data, curr->level);
		if (rc)
			pr_err("%s: vreg_set_level() = %d\n", __func__, rc);

		rc = vreg_enable(curr->vreg_data);
		if (rc)
			pr_err("%s: vreg_enable() = %d\n", __func__, rc);
	} else {
		clear_bit(dev_id, &vreg_sts);
		rc = vreg_disable(curr->vreg_data);
		if (rc)
			pr_err("%s: vreg_disable() = %d\n", __func__, rc);
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);

	rc = msm_sdcc_setup_gpio(pdev->id, !!vdd);
	if (rc)
		goto out;

	rc = msm_sdcc_setup_vreg(pdev->id, !!vdd);
out:
	return rc;
}

#define GPIO_SDC1_HW_DET 94

#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT) \
	&& defined(CONFIG_MMC_MSM_CARD_HW_DETECTION)
static unsigned int msm7x2xa_sdcc_slot_status(struct device *dev)
{
	int status;

	printk("%s entered\n", __func__);

	status = gpio_tlmm_config(GPIO_CFG(GPIO_SDC1_HW_DET, 2, GPIO_CFG_INPUT,
			GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	if (status)
		pr_err("%s:Failed to configure tlmm for GPIO %d\n", __func__,
				GPIO_SDC1_HW_DET);

	status = gpio_request(GPIO_SDC1_HW_DET, "SD_HW_Detect");
	if (status) {
		pr_err("%s:Failed to request GPIO %d\n", __func__,
				GPIO_SDC1_HW_DET);
	} else {
		status = gpio_direction_input(GPIO_SDC1_HW_DET);
		if (!status)
			status = gpio_get_value(GPIO_SDC1_HW_DET);
		gpio_free(GPIO_SDC1_HW_DET);
	}

	status = status?0:1 ; //PMMC
	printk("<=PMMC=> %s : status : %d \n", __func__, status);
	return status;
}
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data sdc1_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm7x2xa_sdcc_slot_status,
	.status_irq  = MSM_GPIO_TO_INT(GPIO_SDC1_HW_DET),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data sdc2_plat_data = {
	/*
	 * SDC2 supports only 1.8V, claim for 2.85V range is just
	 * for allowing buggy cards who advertise 2.8V even though
	 * they can operate at 1.8V supply.
	 */
	.ocr_mask	= MMC_VDD_28_29 | MMC_VDD_165_195,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#if 0  /* def CONFIG_MMC_MSM_SDIO_SUPPORT */
	.sdiowakeup_irq = MSM_GPIO_TO_INT(66),
#endif
#ifndef ATH_POLLING
	.status = wlan_status,
	.register_status_notify = register_wlan_status_notify,
#endif /* ATH_POLLING */
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000, //24576000, ///*144000,//*/
#ifdef CONFIG_MMC_MSM_SDC2_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data sdc3_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
};
#endif

#if (defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
		&& !defined(CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT))
static struct mmc_platform_data sdc4_plat_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
};
#endif
#endif

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup	= 1,
	.rx_to_inject		= 0xFD,
};
#endif
static struct msm_pm_platform_data msm7x27a_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 16000,
					.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 12000,
					.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 0,
					.suspend_enabled = 1,
					.latency = 2000,
					.residency = 0,
	},
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 2,
					.residency = 0,
	},
};

u32 msm7627a_power_collapse_latency(enum msm_pm_sleep_mode mode)
{
	switch (mode) {
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
		return msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency;
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN:
		return msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency;
	case MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT:
		return msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
		return msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency;
	default:
		return 0;
	}
}

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_RESET_VECTOR_PHYS,
	.p_addr = 0,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_mdp_size", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}

early_param("fb_size", fb_size_setup);

static const char * const msm_fb_lcdc_vreg[] = {
		"gp2",
		"msme1",
};

static const int msm_fb_lcdc_vreg_mV[] = {
	2850,
	1800,
};

#define LCDC_CONFIG_PROC          21
#define LCDC_UN_CONFIG_PROC       22
#define LCDC_API_PROG             0x30000066
#define LCDC_API_VERS             0x00010001

#define	GPIO_SPI_CLK		30
#define	GPIO_SPI_CS		26
#define	GPIO_SPI_SDI		57
#define GPIO_SPI_SDO		23
#define	GPIO_LCD_RESET_N	107
#define	GPIO_LCD_DETECT		38

struct vreg *lcdc_vreg[ARRAY_SIZE(msm_fb_lcdc_vreg)];

#if 0 // toshiba panel
static uint32_t lcdc_gpio_initialized;

static void lcdc_toshiba_gpio_init(void)
{
	int i, rc = 0;
	if (!lcdc_gpio_initialized) {
		if (gpio_request(GPIO_SPI_CLK, "spi_clk")) {
			pr_err("failed to request gpio spi_clk\n");
			return;
		}
		if (gpio_request(GPIO_SPI_CS0_N, "spi_cs")) {
			pr_err("failed to request gpio spi_cs0_N\n");
			goto fail_gpio6;
		}
		if (gpio_request(GPIO_SPI_MOSI, "spi_mosi")) {
			pr_err("failed to request gpio spi_mosi\n");
			goto fail_gpio5;
		}
		if (gpio_request(GPIO_SPI_MISO, "spi_miso")) {
			pr_err("failed to request gpio spi_miso\n");
			goto fail_gpio4;
		}
		if (gpio_request(GPIO_DISPLAY_PWR_EN, "gpio_disp_pwr")) {
			pr_err("failed to request gpio_disp_pwr\n");
			goto fail_gpio3;
		}
		if (gpio_request(GPIO_BACKLIGHT_EN, "gpio_bkl_en")) {
			pr_err("failed to request gpio_bkl_en\n");
			goto fail_gpio2;
		}
		pmapp_disp_backlight_init();

		for (i = 0; i < ARRAY_SIZE(msm_fb_lcdc_vreg); i++) {
			lcdc_vreg[i] = vreg_get(0, msm_fb_lcdc_vreg[i]);

			rc = vreg_set_level(lcdc_vreg[i],
						msm_fb_lcdc_vreg_mV[i]);

			if (rc < 0) {
				pr_err("%s: set regulator level failed "
					"with :(%d)\n", __func__, rc);
				goto fail_gpio1;
			}
		}
		lcdc_gpio_initialized = 1;
	}
	return;

fail_gpio1:
	for (; i > 0; i--)
			vreg_put(lcdc_vreg[i - 1]);

	gpio_free(GPIO_BACKLIGHT_EN);
fail_gpio2:
	gpio_free(GPIO_DISPLAY_PWR_EN);
fail_gpio3:
	gpio_free(GPIO_SPI_MISO);
fail_gpio4:
	gpio_free(GPIO_SPI_MOSI);
fail_gpio5:
	gpio_free(GPIO_SPI_CS0_N);
fail_gpio6:
	gpio_free(GPIO_SPI_CLK);
	lcdc_gpio_initialized = 0;
}

static uint32_t lcdc_gpio_table[] = {
	GPIO_SPI_CLK,
	GPIO_SPI_CS0_N,
	GPIO_SPI_MOSI,
	GPIO_DISPLAY_PWR_EN,
	GPIO_BACKLIGHT_EN,
	GPIO_SPI_MISO,
};

static void config_lcdc_gpio_table(uint32_t *table, int len, unsigned enable)
{
	int n;

	if (lcdc_gpio_initialized) {
		/* All are IO Expander GPIOs */
		for (n = 0; n < (len - 1); n++)
			gpio_direction_output(table[n], 1);
	}
}

static void lcdc_toshiba_config_gpios(int enable)
{
	config_lcdc_gpio_table(lcdc_gpio_table,
		ARRAY_SIZE(lcdc_gpio_table), enable);
}
#endif

static int lcdc_gpio_num[] = {
	GPIO_SPI_CLK,
	GPIO_SPI_CS,
	GPIO_SPI_SDI,
	GPIO_LCD_RESET_N,
	GPIO_LCD_DETECT,
};

static void lcdc_amazing_gpio_init(void)
{
	int rc;

	if (gpio_request(GPIO_SPI_CLK, "spi_clk")) {
		pr_err("failed to request gpio spi_clk\n");
	}
	if (gpio_request(GPIO_SPI_CS, "spi_cs")) {
		pr_err("failed to request gpio spi_cs\n");
	}
	if (gpio_request(GPIO_SPI_SDI, "spi_mosi")) {
		pr_err("failed to request gpio spi_sdi\n");
	}
	if (gpio_request(GPIO_LCD_RESET_N, "gpio_lcd_reset_n")) {
		pr_err("failed to request gpio lcd_reset_n\n");
	}

	/* Change LCD_MCLK Drive Strength */
	rc = gpio_tlmm_config(GPIO_CFG(GPIO_LCD_MCLK, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,
				GPIO_CFG_6MA), GPIO_CFG_ENABLE);

	if (rc) {
		pr_err("%s: gpio_tlmm_config for %d failed\n",
			__func__, GPIO_LCD_MCLK);
		//goto err;
	}

	/* LCD Detect Irq */
	rc = gpio_tlmm_config(GPIO_CFG(GPIO_LCD_DETECT, 0,
				GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: gpio_tlmm_config for %d failed\n",
			__func__, GPIO_LCD_DETECT);
		//goto err;
	}

	rc = gpio_request(GPIO_LCD_DETECT, "gpio_lcd_detect");
	if (rc) {
		pr_err("%s: unable to request gpio %d\n",
			__func__, GPIO_LCD_DETECT);
		//goto err;
	}

	rc = gpio_direction_input(GPIO_LCD_DETECT);
	if (rc < 0) {
		pr_err("%s: unable to set the direction of gpio %d\n",
			__func__, GPIO_LCD_DETECT);
		//goto err;
	}

	return;
}

static uint32_t lcdc_gpio_table[] = {
	GPIO_CFG(GPIO_SPI_CLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_SPI_CS, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_SPI_SDI, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_LCD_RESET_N,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static void config_lcdc_gpio_table(uint32_t *table, int len, unsigned enable)
{
	int n, rc;

	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n],
			enable ? GPIO_CFG_ENABLE : GPIO_CFG_DISABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}
static int msm_fb_lcdc_power_save(int on)
{
#if 0
	/* struct vreg *vreg[ARRAY_SIZE(msm_fb_lcdc_vreg)]; */
	int rc = 0;

	/* Doing the init of the LCDC GPIOs very late as they are from
		an I2C-controlled IO Expander */
	lcdc_toshiba_gpio_init();

	if (lcdc_gpio_initialized) {
		gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN, on);
		gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, on);
	}

	pmapp_disp_backlight_init();
	rc = pmapp_disp_backlight_set_brightness(100);

#endif
    return 0;
}

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_gpio_config = NULL,
	.lcdc_power_save   = msm_fb_lcdc_power_save,
};

static struct resource lcdc_amazing_resources[] = {
	{
		.name = "lcd_breakdown_det",
		.start = MSM_GPIO_TO_INT(GPIO_LCD_DETECT),
		.end = MSM_GPIO_TO_INT(GPIO_LCD_DETECT),
		.flags  = IORESOURCE_IRQ,
	}
};

static void lcdc_amazing_config_gpios(int enable)
{
//KYLE_PATCH
       return;
	config_lcdc_gpio_table(lcdc_gpio_table,
		ARRAY_SIZE(lcdc_gpio_table), enable);
}

static struct msm_panel_common_pdata lcdc_amazing_panel_data = {
	.panel_config_gpio = lcdc_amazing_config_gpios,
	.gpio_num	  = lcdc_gpio_num,

};

static struct platform_device lcdc_amazing_panel_device = {
#if defined(CONFIG_FB_MSM_LCDC_AMAZING_HVGA)
	.name   = "lcdc_amazing_hvga",
	.num_resources  = ARRAY_SIZE(lcdc_amazing_resources),
	.resource       = lcdc_amazing_resources,
#else
	.name   = "lcdc_s6d16a0x_hvga",
#endif
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_amazing_panel_data,
	}
};


static void mipi_kyle_gpio_init(void)
{
	// need to fill
	if (gpio_request(GPIO_LCD_RESET_N, "gpio_lcd_reset_n")) {
		pr_err("failed to request gpio lcd_reset_n\n");
	}
	return;
}

static uint32_t mipi_kyle_gpio_table[] = {
	GPIO_CFG(GPIO_LCD_RESET_N,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static void config_mipi_kyle_gpio_table(uint32_t *table, int len, unsigned enable)
{
	int n, rc;

	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n],
			enable ? GPIO_CFG_ENABLE : GPIO_CFG_DISABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static void mipi_kyle_config_gpios(int enable)
{
	config_mipi_kyle_gpio_table(mipi_kyle_gpio_table,
		ARRAY_SIZE(mipi_kyle_gpio_table), enable);
}

static struct msm_panel_common_pdata mipi_kyle_panel_data = {
	.panel_config_gpio = lcdc_amazing_config_gpios,
	.gpio_num	  = lcdc_gpio_num,

};

static struct platform_device mipi_kyle_panel_device = {

	.name   = "mipi_cmd_nt35510_wvga",
	.id     = 0,
	.dev    = {
		.platform_data = &mipi_kyle_panel_data,
	}
};

#ifdef CONFIG_BATTERY_MAX17040

#define GPIO_FUELGAUGE_I2C_SDA		79
#define GPIO_FUELGAUGE_I2C_SCL		78
#define GPIO_FUEL_INT	23

int check_battery_type(void)
{
	return BATT_TYPE_D2_ACTIVE;
}
void max17040_hw_init(void)
{
	pr_err("%s\n", __func__);
	gpio_tlmm_config(GPIO_CFG(GPIO_FUELGAUGE_I2C_SCL, 0, GPIO_CFG_OUTPUT,
		 GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
	gpio_tlmm_config(GPIO_CFG(GPIO_FUELGAUGE_I2C_SDA,  0, GPIO_CFG_OUTPUT,
		 GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
	gpio_set_value(GPIO_FUELGAUGE_I2C_SCL, 1);
	gpio_set_value(GPIO_FUELGAUGE_I2C_SDA, 1);

//	gpio_tlmm_config(GPIO_CFG(GPIO_FUEL_INT,  0, GPIO_CFG_INPUT,
//	 GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
}

static int max17040_low_batt_cb(void)
{
	pr_err("%s: Low battery alert\n", __func__);

#ifdef CONFIG_BATTERY_SEC
	struct power_supply *psy = power_supply_get_by_name("battery");
	union power_supply_propval value;

	if (!psy) {
		pr_err("%s: fail to get battery ps\n", __func__);
		return -ENODEV;
	}

	value.intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	return psy->set_property(psy, POWER_SUPPLY_PROP_CAPACITY_LEVEL, &value);
#else
	return 0;
#endif
}

static struct max17040_platform_data max17043_pdata = {
	.hw_init = max17040_hw_init,
	.low_batt_cb = max17040_low_batt_cb,
	.check_batt_type = check_battery_type,
	.rcomp_value = 0x701d,
};

static struct i2c_gpio_platform_data fuelgauge_i2c_gpio_data = {
	.sda_pin		= GPIO_FUELGAUGE_I2C_SDA,
	.scl_pin		= GPIO_FUELGAUGE_I2C_SCL,
	.udelay			= 2,
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device fuelgauge_i2c_gpio_device = {
	.name			= "i2c-gpio",
	.id			= MSM_FUELGAUGE_I2C_BUS_ID,
	.dev	= {
	.platform_data	= &fuelgauge_i2c_gpio_data,
	},
};

static struct i2c_board_info fg_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("max17040", 0x6D>>1),
		.platform_data = &max17043_pdata,
		.irq		= MSM_GPIO_TO_INT(GPIO_FUEL_INT),
	}
};
#endif /* CONFIG_BATTERY_MAX17040 */

#if defined(CONFIG_TOUCHSCREEN_ZINITIX_TREBON) || defined(CONFIG_TOUCHSCREEN_ZINITIX_AMAZING)
static void tsp_power_on(void)
{
	int rc = 0;
	printk("[TSP] %s start \n", __func__);

	rc = gpio_request(41, "touch_en");
	if (rc < 0) {
		pr_err("failed to request touch_en\n");
	}

	gpio_tlmm_config(GPIO_CFG(41, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_direction_output(41, 1);
}
#endif

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

static void samsung_sys_class_init(void)
{
	pr_info("samsung sys class init.\n");

	sec_class = class_create(THIS_MODULE, "sec");
	if (IS_ERR(sec_class))
		pr_err("Failed to create class(sec) !\n");

}

#if 0 // toshiba panel
static int lcd_panel_spi_gpio_num[] = {
		GPIO_SPI_MOSI,  /* spi_sdi */
		GPIO_SPI_MISO,  /* spi_sdoi */
		GPIO_SPI_CLK,   /* spi_clk */
		GPIO_SPI_CS0_N, /* spi_cs  */
};

static struct msm_panel_common_pdata lcdc_toshiba_panel_data = {
	.panel_config_gpio = lcdc_toshiba_config_gpios,
	.pmic_backlight = lcdc_toshiba_set_bl,
	.gpio_num	  = lcd_panel_spi_gpio_num,
};

static struct platform_device lcdc_toshiba_panel_device = {
	.name   = "lcdc_toshiba_fwvga_pt",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_toshiba_panel_data,
	}
};
#endif

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

/*** LCD handling for KYLE ***/
static int detected_lcd_id = -1;
static bool lcd_present = false;

/* panel_name={SMD,BOE,FAIL} */
static int __init update_panel_name(char *panel_name)
{
	int i;
	char *src = panel_name;
	char panel[2][4] = {"SMD", "BOE"};
	int panel_id[2] = {1, 2};

		for (i = 0; i < sizeof(panel) / sizeof(panel[0]); i++) {
			if (!strncmp(src, panel[i], sizeof(panel[i])))
			detected_lcd_id = panel_id[i];
		}

	if (detected_lcd_id == -1) {
			printk(KERN_INFO "%s : Fail to get lcd panel name"
				" from bootloader\n", __func__);
		/* LK did not detect LCD.
		 * Probabely LCD is not connected */
		lcd_present = false;
		detected_lcd_id = panel_id[1]; /* default to BOE */
		} else {
			printk(KERN_INFO "%s : Success to get"
				"lcd panel name(%s) from bootloader\n",
			__func__, panel[detected_lcd_id - 1]);
		lcd_present = true;
		}

	return 1;
}
__setup("panel_name=", update_panel_name);

static int msm_fb_detect_panel(const char *name)
{
	int ret = -EPERM;
	char panel[2][24] = {"mipi_cmd_nt35510_hydis_wvga",
            "mipi_cmd_nt35510_boe_wvga"};

	if (!strncmp(name, panel[detected_lcd_id - 1], sizeof(panel[detected_lcd_id - 1])))
		ret = 0;
	else
		ret = -ENODEV;

	return ret;
}

bool msm_fb_is_lcd_present(void)
{
	return lcd_present;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

#ifdef CONFIG_FB_MSM_MIPI_DSI
static int mipi_renesas_set_bl(int level)
{
	int ret;

	ret = pmapp_disp_backlight_set_brightness(level);

	if (ret)
		pr_err("%s: can't set lcd backlight!\n", __func__);

	return ret;
}

static struct msm_panel_common_pdata mipi_renesas_pdata = {
	.pmic_backlight = mipi_renesas_set_bl,
};


static struct platform_device mipi_dsi_renesas_panel_device = {
	.name = "mipi_renesas",
	.id = 0,
	.dev    = {
		.platform_data = &mipi_renesas_pdata,
	}
};
#endif


static void __init msm7x27a_init_mmc(void)
{
        vreg_emmc = vreg_get(NULL,"msme1");
        if (IS_ERR(vreg_emmc)) {
                pr_err("%s: vreg get failed (%ld)\n",
                                __func__, PTR_ERR(vreg_emmc));
                return;
        }

        vreg_mmc = vreg_get(NULL,"mmc");
        if (IS_ERR(vreg_mmc)) {
                pr_err("%s: vreg get failed (%ld)\n",
                                __func__, PTR_ERR(vreg_mmc));
                return;
        }

	/* eMMC slot */
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	sdcc_vreg_data[2].vreg_data = vreg_emmc;
	sdcc_vreg_data[2].level = 1800;
	msm_add_sdcc(3, &sdc3_plat_data);
#endif
	/* Micro-SD slot */
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	sdcc_vreg_data[0].vreg_data = vreg_mmc;
	sdcc_vreg_data[0].level = 2850;
	msm_add_sdcc(1, &sdc1_plat_data);
#endif
	/* SDIO WLAN slot */
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	//sdcc_vreg_data[1].vreg_data = vreg_mmc;
	sdcc_vreg_data[1].vreg_data = vreg_emmc;
	sdcc_vreg_data[1].level = 1800/*2850*/;
	msm_add_sdcc(2, &sdc2_plat_data);
#endif
	/* Not Used */
#if (defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
		&& !defined(CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT))
	sdcc_vreg_data[3].vreg_data = vreg_mmc;
	sdcc_vreg_data[3].level = 2850;
	msm_add_sdcc(4, &sdc4_plat_data);
#endif
}

#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(HANDSET, 0),
	SND(MONO_HEADSET, 2),
	SND(HEADSET, 3),
	SND(SPEAKER, 6),
	SND(TTY_HEADSET, 8),
	SND(TTY_VCO, 9),
	SND(TTY_HCO, 10),
	SND(BT, 12),
	SND(IN_S_SADC_OUT_HANDSET, 16),
	SND(VOICE_RECOGNITION, 24),
	SND(FM_DIGITAL_STEREO_HEADSET, 26),
	SND(FM_DIGITAL_SPEAKER_PHONE, 27),
	SND(FM_DIGITAL_BT_A2DP_HEADSET, 28),
	SND(FM_STEREO_HEADSET, 29),
	SND(FM_SPEAKER_PHONE, 30),
	SND(STEREO_HEADSET_AND_SPEAKER, 31),
	SND(HEADSET_AND_SPEAKER, 32),
	SND(STEREO_HEADSET_3POLE, 34),
	SND(MP3_SPEAKER_PHONE, 35),
	SND(MP3_STEREO_HEADSET, 36),
	SND(BT_NSEC_OFF, 37),
	SND(HANDSET_VOIP, 38),
	SND(STEREO_HEADSET_VOIP, 39),
	SND(SPEAKER_VOIP, 40),
	SND(BT_VOIP, 41),
	SND(HANDSET_VOIP2, 42),
	SND(STEREO_HEADSET_VOIP2, 43),
	SND(SPEAKER_VOIP2, 44),
	SND(BT_VOIP2, 45),
	SND(VOICE_RECORDER_HPH, 46),
	SND(VOICE_RECORDER_SPK, 47),
	SND(FM_ANALOG_STEREO_HEADSET, 50),
	SND(FM_ANALOG_STEREO_HEADSET_CODEC, 51),
	SND(HANDSET_VT, 52),
	SND(STEREO_HEADSET_VT, 53),
	SND(SPEAKER_PHONE_VT, 54),
	SND(BT_VT, 55),
	SND(CURRENT, 0x7FFFFFFE),
};
#undef SND

static struct msm_snd_endpoints msm_device_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device msm_device_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_snd_endpoints
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DMA)), 0,
	0, 0, 0,

	/* Concurrency 1 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	 /* Concurrency 2 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 3 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 4 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 5 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 6 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	0, 0, 0, 0,

	/* Concurrency 7 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};
static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct smsc911x_platform_config smsc911x_config = {
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_16BIT,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.start	= 0x90000000,
		.end	= 0x90007fff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(48),
		.end	= MSM_GPIO_TO_INT(48),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};

static struct platform_device smsc911x_device = {
	.name		= "smsc911x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
	.dev		= {
		.platform_data	= &smsc911x_config,
	},
};

static struct msm_gpio smsc911x_gpios[] = {
	{ GPIO_CFG(48, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
							 "smsc911x_irq"  },
	{ GPIO_CFG(49, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
							 "eth_fifo_sel" },
};

#define ETH_FIFO_SEL_GPIO	49
static void msm7x27a_cfg_smsc911x(void)
{
	int res;

	res = msm_gpios_request_enable(smsc911x_gpios,
				 ARRAY_SIZE(smsc911x_gpios));
	if (res) {
		pr_err("%s: unable to enable gpios for SMSC911x\n", __func__);
		return;
	}

	/* ETH_FIFO_SEL */
	res = gpio_direction_output(ETH_FIFO_SEL_GPIO, 0);
	if (res) {
		pr_err("%s: unable to get direction for gpio %d\n", __func__,
							 ETH_FIFO_SEL_GPIO);
		msm_gpios_disable_free(smsc911x_gpios,
						 ARRAY_SIZE(smsc911x_gpios));
		return;
	}
	gpio_set_value(ETH_FIFO_SEL_GPIO, 0);
}

#if defined(CONFIG_SERIAL_MSM_HSL_CONSOLE) \
		&& defined(CONFIG_MSM_SHARED_GPIO_FOR_UART2DM)
#ifndef CONFIG_MACH_KYLE
static struct msm_gpio uart2dm_gpios[] = {
	{GPIO_CFG(19, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rfr_n" },
	{GPIO_CFG(20, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_cts_n" },
	{GPIO_CFG(21, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rx"    },
	{GPIO_CFG(108, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_tx"    },
};

static void msm7x27a_cfg_uart2dm_serial(void)
{
	int ret;
	ret = msm_gpios_request_enable(uart2dm_gpios,
					ARRAY_SIZE(uart2dm_gpios));
	if (ret)
		pr_err("%s: unable to enable gpios for uart2dm\n", __func__);
}
#endif
#else
static void msm7x27a_cfg_uart2dm_serial(void) { }
#endif

/* GGSM sc47.yun CSR7820 Project 2012.04.23 */
#ifdef CONFIG_BT_CSR_7820
static struct resource bluesleep_resources[] = {
	{
		.name = "gpio_host_wake",
		.start = GPIO_BT_PWR,
		.end = GPIO_BT_PWR,
		.flags = IORESOURCE_IO,
	},
	{
		.name = "host_wake",
		.start = MSM_GPIO_TO_INT(GPIO_BT_PWR),
		.end = MSM_GPIO_TO_INT(GPIO_BT_PWR),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_bt_power_device_csr = {
	.name = "bt_power",
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id = -1,
	.num_resources = ARRAY_SIZE(bluesleep_resources),
	.resource = bluesleep_resources,
};
#endif
/* GGSM sc47.yun CSR7820 Project end */

#ifdef CONFIG_KEYBOARD_GPIO
static struct gpio_keys_button gpio_keys_button[] = {
#if 0
	{
		.code			= KEY_VOLUMEUP,
		.type			= EV_KEY,
		.gpio			= NULL,
		.active_low		= 1,
		.wakeup			= 0,
		.debounce_interval	= 5, /* ms */
		.desc			= "Vol Up",
	},
	{
		.code			= KEY_VOLUMEDOWN,
		.type			= EV_KEY,
		.gpio			= NULL,
		.active_low		= 1,
		.wakeup			= 0,
		.debounce_interval	= 5, /* ms */
		.desc			= "Vol Down",
	},
#endif
	{
		.code			= KEY_HOMEPAGE,
		.type			= EV_KEY,
		.gpio			= 37,
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 5, /* ms */
		.desc			= "Home",
	},
};
static struct gpio_keys_platform_data gpio_keys_platform_data = {
	.buttons	= gpio_keys_button,
	.nbuttons	= ARRAY_SIZE(gpio_keys_button),
	.rep		= 0,
};

static struct platform_device msm7x27a_gpio_keys_device = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_keys_platform_data,
	}
};
#endif

static struct platform_device *rumi_sim_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&smc91x_device,
	&msm_device_uart1,
	&msm_device_nand,
	&msm_device_uart_dm1,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device
};

static struct platform_device *surf_ffa_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&msm_device_uart1,
	&msm_device_uart_dm1,
	//&msm_device_uart_dm2,
	&msm_device_nand,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
	&msm_device_otg,
	&msm_device_gadget_peripheral,
	&android_usb_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_device_snd,
	&msm_device_adspdec,
	&msm_fb_device,
	/*&lcdc_toshiba_panel_device,*/
	/*&lcdc_amazing_panel_device,*/
	&mipi_kyle_panel_device,
	&msm_batt_device,
	/*&smsc911x_device,*/
	&msm_kgsl_3d0,
/*#ifdef CONFIG_BT
	&msm_bt_power_device,*/
/*#if CONFIG_BT_CSR_7820*/
/* GGSM sc47.yun CSR7820 Project 2012.04.23 */
	&msm_bt_power_device_csr,
	&msm_bluesleep_device,
/*#else*/
/* GGSM sc47.yun CSR7820 Project end */
/*#endif*/
	&touch_i2c_gpio_device,
	&fsa880_i2c_gpio_device,
#ifdef CONFIG_BQ27425_FUEL_GAUGE
	&fuelgauge_i2c_gpio_device,
#endif
#ifdef CONFIG_BATTERY_MAX17040
	&fuelgauge_i2c_gpio_device,
#endif
#ifdef CONFIG_MAX17048_FUELGAUGE
	&fg_smb_i2c_gpio_device,
#endif
#if defined(CONFIG_SENSORS_BMA222E) \
	|| defined(CONFIG_PROXIMITY_SENSOR) || defined(CONFIG_SENSORS_BMA222)
	&sensor_i2c_gpio_device,
#endif
#ifdef CONFIG_SAMSUNG_JACK
	&sec_device_jack,
#endif
	&msm_device_pmic_leds,
	&msm_vibrator_device,
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
#ifdef CONFIG_KEYBOARD_GPIO
	&msm7x27a_gpio_keys_device,
#endif
};

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static void __init msm_msm7x2x_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
}

static struct memtype_reserve msm7x27a_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	android_pmem_adsp_pdata.size = pmem_adsp_size;
	android_pmem_pdata.size = pmem_mdp_size;
	android_pmem_audio_pdata.size = pmem_audio_size;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x27a_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	msm7x27a_reserve_table[MEMTYPE_EBI1].size += pmem_kernel_ebi1_size;
#endif
}

static void __init msm7x27a_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7x27a_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info msm7x27a_reserve_info __initdata = {
	.memtype_reserve_table = msm7x27a_reserve_table,
	.calculate_reserve_sizes = msm7x27a_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x27a_paddr_to_memtype,
};

static void __init msm7x27a_reserve(void)
{
	reserve_info = &msm7x27a_reserve_info;
	msm_reserve();
}

static void __init msm_device_i2c_init(void)
{
	msm_gsbi0_qup_i2c_device.dev.platform_data = &msm_gsbi0_qup_i2c_pdata;
	msm_gsbi1_qup_i2c_device.dev.platform_data = &msm_gsbi1_qup_i2c_pdata;

	gpio_set_value(GPIO_TSP_SDA, 1);
	gpio_set_value(GPIO_TSP_SCL, 1);
	gpio_tlmm_config(GPIO_CFG(GPIO_TSP_SDA, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);

	gpio_tlmm_config(GPIO_CFG(GPIO_TSP_SCL, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);

/*	gpio_tlmm_config(GPIO_CFG(27, 0, GPIO_CFG_INPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE); */
#if defined(CONFIG_SENSORS_BMA222E) \
	|| defined(CONFIG_PROXIMITY_SENSOR) || defined(CONFIG_SENSORS_BMA222)
	 gpio_tlmm_config(GPIO_CFG(GPIO_SENSOR_SCL, 0, GPIO_CFG_OUTPUT,
				  GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	 gpio_tlmm_config(GPIO_CFG(GPIO_SENSOR_SDA, 0, GPIO_CFG_OUTPUT,
				 GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
#endif
	printk("[TSP] %s =======gpio_request==test======ln=%d\n",
			__func__, __LINE__);
}

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
	.mdp_rev = MDP_REV_303,
};



#define GPIO_LCDC_BRDG_PD	0xFF
#define GPIO_LCDC_BRDG_RESET_N	0xFF

#define LCDC_RESET_PHYS		0x90008014

static	void __iomem *lcdc_reset_ptr;

static unsigned mipi_dsi_gpio[] = {
	GPIO_CFG(GPIO_LCDC_BRDG_RESET_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_2MA),       /* LCDC_BRDG_RESET_N */
	GPIO_CFG(GPIO_LCDC_BRDG_PD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_2MA),       /* LCDC_BRDG_RESET_N */
};

enum {
	DSI_SINGLE_LANE = 1,
	DSI_TWO_LANES,
};

static int msm_fb_get_lane_config(void)
{
	int rc = DSI_TWO_LANES;

	if (machine_is_msm7625a_surf() || machine_is_msm7625a_ffa()) {
		rc = DSI_SINGLE_LANE;
		pr_info("DSI Single Lane\n");
	} else {
		pr_info("DSI Two Lanes\n");
	}
	return rc;
}

static int msm_fb_dsi_client_reset(void)
{
	int rc = 0;

	return rc;

	rc = gpio_request(GPIO_LCDC_BRDG_RESET_N, "lcdc_brdg_reset_n");
	if (rc < 0) {
		pr_err("failed to request lcd brdg reset_n\n");
		return rc;
	}

	rc = gpio_request(GPIO_LCDC_BRDG_PD, "lcdc_brdg_pd");
	if (rc < 0) {
		pr_err("failed to request lcd brdg pd\n");
		return rc;
	}

	rc = gpio_tlmm_config(mipi_dsi_gpio[0], GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable LCDC Bridge reset enable\n");
		goto gpio_error;
	}

	rc = gpio_tlmm_config(mipi_dsi_gpio[1], GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable LCDC Bridge pd enable\n");
		goto gpio_error2;
	}

	rc = gpio_direction_output(GPIO_LCDC_BRDG_RESET_N, 1);
	rc |= gpio_direction_output(GPIO_LCDC_BRDG_PD, 1);
	gpio_set_value_cansleep(GPIO_LCDC_BRDG_PD, 0);

	if (!rc) {
		if (machine_is_msm7x27a_surf()) {
			lcdc_reset_ptr = ioremap_nocache(LCDC_RESET_PHYS,
				sizeof(uint32_t));

			if (!lcdc_reset_ptr)
				return 0;
		}
		return rc;
	} else {
		goto gpio_error;
	}

gpio_error2:
	pr_err("Failed GPIO bridge pd\n");
	gpio_free(GPIO_LCDC_BRDG_PD);

gpio_error:
	pr_err("Failed GPIO bridge reset\n");
	gpio_free(GPIO_LCDC_BRDG_RESET_N);
	return rc;
}

static const char * const msm_fb_dsi_vreg[] = {
	"gp2",
	"msme1",
	"mddi"
};

static const int msm_fb_dsi_vreg_mV[] = {
	2850,
	1800,
	1200
};

static struct vreg *dsi_vreg[ARRAY_SIZE(msm_fb_dsi_vreg)];
static int dsi_gpio_initialized;

static int mipi_dsi_panel_power(int on)
{
	int i, rc = 0;
	uint32_t lcdc_reset_cfg;

	dsi_gpio_initialized = 1;

	return rc;

	/* I2C-controlled GPIO Expander -init of the GPIOs very late */
	if (!dsi_gpio_initialized) {
		pmapp_disp_backlight_init();

		rc = gpio_request(GPIO_DISPLAY_PWR_EN, "gpio_disp_pwr");
		if (rc < 0) {
			pr_err("failed to request gpio_disp_pwr\n");
			return rc;
		}

		rc = gpio_direction_output(GPIO_DISPLAY_PWR_EN, 1);
		if (rc < 0) {
			pr_err("failed to enable display pwr\n");
			goto fail_gpio1;
		}

		if (machine_is_msm7x27a_surf()) {
			rc = gpio_request(GPIO_BACKLIGHT_EN, "gpio_bkl_en");
			if (rc < 0) {
				pr_err("failed to request gpio_bkl_en\n");
				goto fail_gpio1;
			}

			rc = gpio_direction_output(GPIO_BACKLIGHT_EN, 1);
			if (rc < 0) {
				pr_err("failed to enable backlight\n");
				goto fail_gpio2;
			}
		}

		for (i = 0; i < ARRAY_SIZE(msm_fb_dsi_vreg); i++) {
			dsi_vreg[i] = vreg_get(0, msm_fb_dsi_vreg[i]);

			if (IS_ERR(dsi_vreg[i])) {
				pr_err("%s: vreg get failed with : (%ld)\n",
					__func__, PTR_ERR(dsi_vreg[i]));
				goto fail_gpio2;
			}

			rc = vreg_set_level(dsi_vreg[i],
				msm_fb_dsi_vreg_mV[i]);

			if (rc < 0) {
				pr_err("%s: set regulator level failed "
					"with :(%d)\n",	__func__, rc);
				goto vreg_fail1;
			}
		}
		dsi_gpio_initialized = 1;
	}

		gpio_set_value_cansleep(GPIO_DISPLAY_PWR_EN, on);
		if (machine_is_msm7x27a_surf()) {
			gpio_set_value_cansleep(GPIO_BACKLIGHT_EN, on);
		}

		if (on) {
			gpio_set_value_cansleep(GPIO_LCDC_BRDG_PD, 0);

			if (machine_is_msm7x27a_surf()) {
				lcdc_reset_cfg = readl_relaxed(lcdc_reset_ptr);
				rmb();
				lcdc_reset_cfg &= ~1;

				writel_relaxed(lcdc_reset_cfg, lcdc_reset_ptr);
				msleep(20);
				wmb();
				lcdc_reset_cfg |= 1;
				writel_relaxed(lcdc_reset_cfg, lcdc_reset_ptr);
			} else {
				gpio_set_value_cansleep(GPIO_LCDC_BRDG_RESET_N,
					0);
				msleep(20);
				gpio_set_value_cansleep(GPIO_LCDC_BRDG_RESET_N,
					1);
			}

			if (pmapp_disp_backlight_set_brightness(100))
				pr_err("backlight set brightness failed\n");
		} else {
			gpio_set_value_cansleep(GPIO_LCDC_BRDG_PD, 1);

			if (pmapp_disp_backlight_set_brightness(0))
				pr_err("backlight set brightness failed\n");
		}

		/*Configure vreg lines */
		for (i = 0; i < ARRAY_SIZE(msm_fb_dsi_vreg); i++) {
			if (on) {
				rc = vreg_enable(dsi_vreg[i]);

				if (rc) {
					printk(KERN_ERR "vreg_enable: %s vreg"
						"operation failed\n",
						msm_fb_dsi_vreg[i]);

					goto vreg_fail2;
				}
			} else {
				rc = vreg_disable(dsi_vreg[i]);

				if (rc) {
					printk(KERN_ERR "vreg_disable: %s vreg "
						"operation failed\n",
						msm_fb_dsi_vreg[i]);
					goto vreg_fail2;
				}
			}
		}

	return rc;

vreg_fail2:
	if (on) {
		for (; i > 0; i--)
			vreg_disable(dsi_vreg[i - 1]);
	} else {
		for (; i > 0; i--)
			vreg_enable(dsi_vreg[i - 1]);
	}

	return rc;

vreg_fail1:
	for (; i > 0; i--)
		vreg_put(dsi_vreg[i - 1]);

fail_gpio2:
	gpio_free(GPIO_BACKLIGHT_EN);
fail_gpio1:
	gpio_free(GPIO_DISPLAY_PWR_EN);
	dsi_gpio_initialized = 0;
	return rc;
}

#define MDP_303_VSYNC_GPIO 97

#ifdef CONFIG_FB_MSM_MDP303
static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.vsync_gpio = MDP_303_VSYNC_GPIO,
	.dsi_power_save   = mipi_dsi_panel_power,
	.dsi_client_reset = msm_fb_dsi_client_reset,
	.get_lane_config = msm_fb_get_lane_config,
};
#endif

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	//KYLE_PATCH
//	msm_fb_register_device("lcdc", &lcdc_pdata);
#ifdef CONFIG_FB_MSM_MDP303
	msm_fb_register_device("mipi_dsi", &mipi_dsi_pdata);
#endif
}

#define MSM_EBI2_PHYS			0xa0d00000
#define MSM_EBI2_XMEM_CS2_CFG1		0xa0d10030

static void __init msm7x27a_init_ebi2(void)
{
	uint32_t ebi2_cfg;
	void __iomem *ebi2_cfg_ptr;

	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_PHYS, sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
	if (machine_is_msm7x27a_rumi3() || machine_is_msm7x27a_surf())
		ebi2_cfg |= (1 << 4); /* CS2 */

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);

	/* Enable A/D MUX[bit 31] from EBI2_XMEM_CS2_CFG1 */
	ebi2_cfg_ptr = ioremap_nocache(MSM_EBI2_XMEM_CS2_CFG1,
							 sizeof(uint32_t));
	if (!ebi2_cfg_ptr)
		return;

	ebi2_cfg = readl(ebi2_cfg_ptr);
	if (machine_is_msm7x27a_surf())
		ebi2_cfg |= (1 << 31);

	writel(ebi2_cfg, ebi2_cfg_ptr);
	iounmap(ebi2_cfg_ptr);
}

#define ATMEL_TS_I2C_NAME "maXTouch"

static struct regulator_bulk_data regs_atmel[] = {
	{ .supply = "ldo2",  .min_uV = 2850000, .max_uV = 2850000 },
	{ .supply = "smps3", .min_uV = 1800000, .max_uV = 1800000 },
};

#define ATMEL_TS_GPIO_IRQ 82

static int atmel_ts_power_on(bool on)
{
	int rc = on ?
		regulator_bulk_enable(ARRAY_SIZE(regs_atmel), regs_atmel) :
		regulator_bulk_disable(ARRAY_SIZE(regs_atmel), regs_atmel);

	if (rc)
		pr_err("%s: could not %sable regulators: %d\n",
				__func__, on ? "en" : "dis", rc);
	else
		msleep(50);

	return rc;
}

static int atmel_ts_platform_init(struct i2c_client *client)
{
	int rc;
	struct device *dev = &client->dev;

	rc = regulator_bulk_get(dev, ARRAY_SIZE(regs_atmel), regs_atmel);
	if (rc) {
		dev_err(dev, "%s: could not get regulators: %d\n",
				__func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs_atmel), regs_atmel);
	if (rc) {
		dev_err(dev, "%s: could not set voltages: %d\n",
				__func__, rc);
		goto reg_free;
	}

	rc = gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	if (rc) {
		dev_err(dev, "%s: gpio_tlmm_config for %d failed\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto reg_free;
	}

	/* configure touchscreen interrupt gpio */
	rc = gpio_request(ATMEL_TS_GPIO_IRQ, "atmel_maxtouch_gpio");
	if (rc) {
		dev_err(dev, "%s: unable to request gpio %d\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto ts_gpio_tlmm_unconfig;
	}

	rc = gpio_direction_input(ATMEL_TS_GPIO_IRQ);
	if (rc < 0) {
		dev_err(dev, "%s: unable to set the direction of gpio %d\n",
			__func__, ATMEL_TS_GPIO_IRQ);
		goto free_ts_gpio;
	}
	return 0;

free_ts_gpio:
	gpio_free(ATMEL_TS_GPIO_IRQ);
ts_gpio_tlmm_unconfig:
	gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_DISABLE);
reg_free:
	regulator_bulk_free(ARRAY_SIZE(regs_atmel), regs_atmel);
out:
	return rc;
}

static int atmel_ts_platform_exit(struct i2c_client *client)
{
	gpio_free(ATMEL_TS_GPIO_IRQ);
	gpio_tlmm_config(GPIO_CFG(ATMEL_TS_GPIO_IRQ, 0,
				GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_DISABLE);
	regulator_bulk_free(ARRAY_SIZE(regs_atmel), regs_atmel);
	return 0;
}

static u8 atmel_ts_read_chg(void)
{
	return gpio_get_value(ATMEL_TS_GPIO_IRQ);
}

static u8 atmel_ts_valid_interrupt(void)
{
	return !atmel_ts_read_chg();
}

#define ATMEL_X_OFFSET 13
#define ATMEL_Y_OFFSET 0

static struct maxtouch_platform_data atmel_ts_pdata = {
	.numtouch = 4,
	.init_platform_hw = atmel_ts_platform_init,
	.exit_platform_hw = atmel_ts_platform_exit,
	.power_on = atmel_ts_power_on,
	.display_res_x = 480,
	.display_res_y = 864,
	.min_x = ATMEL_X_OFFSET,
	.max_x = (505 - ATMEL_X_OFFSET),
	.min_y = ATMEL_Y_OFFSET,
	.max_y = (863 - ATMEL_Y_OFFSET),
	.valid_interrupt = atmel_ts_valid_interrupt,
	.read_chg = atmel_ts_read_chg,
};

static struct i2c_board_info atmel_ts_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO(ATMEL_TS_I2C_NAME, 0x4a),
		.platform_data = &atmel_ts_pdata,
		.irq = MSM_GPIO_TO_INT(ATMEL_TS_GPIO_IRQ),
	},
};

static void keypad_gpio_init(void)
{
	int rc = 0;
	printk(KERN_INFO "[KEY] %s start\n", __func__);

	gpio_tlmm_config(GPIO_CFG(36, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, \
						GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(37, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, \
						GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(39, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, \
						GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, \
						GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value_cansleep(31, 0);
}

#define KP_INDEX(row, col) ((row)*ARRAY_SIZE(kp_col_gpios) + (col))

#if (CONFIG_MACH_KYLE_HWREV == 0x0) || (CONFIG_MACH_KYLE_HWREV == 0x1)
	static unsigned int kp_row_gpios[] = {36, 37, 39};
	static unsigned int kp_col_gpios[] = {31};
	static unsigned int kp_wakeup_gpios[] = {37};
	static const unsigned short keymap[ARRAY_SIZE(kp_col_gpios) *
					  ARRAY_SIZE(kp_row_gpios)] = {
		[KP_INDEX(0, 0)] = KEY_VOLUMEDOWN,

		[KP_INDEX(1, 0)] = KEY_HOME,

		[KP_INDEX(2, 0)] = KEY_VOLUMEUP,
	};
#else
	static unsigned int kp_row_gpios[] = {36, 39};
	static unsigned int kp_col_gpios[] = {31};
	static unsigned int kp_wakeup_gpios[] = {37};
	static const unsigned short keymap[ARRAY_SIZE(kp_col_gpios) *
					  ARRAY_SIZE(kp_row_gpios)] = {
		[KP_INDEX(0, 0)] = KEY_VOLUMEDOWN,

		[KP_INDEX(1, 0)] = KEY_VOLUMEUP,
	};
#endif

/* SURF keypad platform device information */
static struct gpio_event_matrix_info kp_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keymap,
	.output_gpios	= kp_col_gpios,
	.input_gpios	= kp_row_gpios,
	.wakeup_gpios	= kp_wakeup_gpios,
	.nwakeups	= ARRAY_SIZE(kp_wakeup_gpios),
	.noutputs	= ARRAY_SIZE(kp_col_gpios),
	.ninputs	= ARRAY_SIZE(kp_row_gpios),
	.settle_time.tv_nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv_nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv_nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS | GPIOKPF_DEBOUNCE,
};

static struct gpio_event_info *kp_info[] = {
	&kp_matrix_info.info
};

static struct gpio_event_platform_data kp_pdata = {
	.name		= "7x27a_kp",
	.info		= kp_info,
	.info_count	= ARRAY_SIZE(kp_info)
};

static struct platform_device kp_pdev = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &kp_pdata,
	},
};

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "sec_jack",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_pdev = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

static struct platform_device msm_proccomm_regulator_dev = {
	.name   = PROCCOMM_REGULATOR_DEV_NAME,
	.id     = -1,
	.dev    = {
		.platform_data = &msm7x27a_proccomm_regulator_data
	}
};

static void __init msm7627a_rumi3_init(void)
{
	msm7x27a_init_ebi2();
	platform_add_devices(rumi_sim_devices,
			ARRAY_SIZE(rumi_sim_devices));
}

#define LED_GPIO_PDM		96
#define UART1DM_RX_GPIO		45

static int __init msm7x27a_init_ar6000pm(void)
{
	return platform_device_register(&msm_wlan_ar6000_pm_device);
}

static void __init msm7x27a_init_regulators(void)
{
	int rc = platform_device_register(&msm_proccomm_regulator_dev);
	if (rc)
		pr_err("%s: could not register regulator device: %d\n",
				__func__, rc);
}

static struct msm7x27a_regulators {
  const char *id;
  unsigned   voltage; // in mv
};

static struct msm7x27a_regulators msm7x27a_reg[] = {
[0] = {
        .id = "smps3",
        .voltage = 1800,
      },
#if 0
[1] = {
        .id = "ldo9",
        .voltage = 1800,
      },
[2] = {
        .id = "ldo13",
        .voltage = 2850,
      },
[3] = {
        .id = "smps1",
        .voltage = 1100,
      },
[4] = {
        .id = "smps2",
        .voltage = 1100,
      },
[5] = {
        .id = "smps4",
        .voltage = 2100,
      },
[4] = {
        .id = "ldo1",
        .voltage = 2100,
      },
[5] = {
        .id = "ldo2",
        .voltage = 2100,
      },
[6] = {
        .id = "ldo3",
        .voltage = 1200,
      },
[7] = {
        .id = "ldo4",
        .voltage = 1100,
      },
[8] = {
        .id = "ldo5",
        .voltage = 0,
      },
[9] = {
        .id = "ldo6",
        .voltage = 1200,
      },
[10] = {
        .id = "ldo7",
        .voltage = 2600,
      },
[11] = {
        .id = "ldo8",
        .voltage = 2800,
      },
[13] = {
        .id = "ldo10",
        .voltage = 2850,
      },
[14] = {
        .id = "ldo11",
        .voltage = 1800,
      },
[15] = {
        .id = "ldo12",
        .voltage = 3300,
      },
[17] = {
        .id = "ldo14",
        .voltage = 3000,
      },
[18] = {
        .id = "ldo15",
        .voltage = 1800,
      },
[19] = {
        .id = "ldo16",
        .voltage = 3000,
      },
[20] = {
        .id = "ldo17",
        .voltage = 2800,
      },
[21] = {
        .id = "ldo18",
        .voltage = 2700,
      },
[22] = {
        .id = "ldo19",
        .voltage = 3000,
      },
#endif
};
/* function to Enable the regulators */
static void msm7x27a_enable_regulators(void)
{
   struct vreg *vreg = NULL;
   struct msm7x27a_regulators *regulators = msm7x27a_reg;
   static int i = 0, ret =0, VRG_SIZE =0;

   /* Get the count of regulators */
   VRG_SIZE= ARRAY_SIZE(msm7x27a_reg);

   for(i=0; i<VRG_SIZE; i++) {

       /* get the regulator descriptor */
       vreg = vreg_get( NULL, regulators[i].id);

       if(IS_ERR(vreg)) {
         pr_err("failed to get the regulator %s\n",regulators[i].id);
         return ;
       }

       /* set the regulator voltage(optimum) */
       ret = vreg_set_level(vreg,regulators[i].voltage);

       if(ret)
       {
         pr_err("failed to set the voltage level for regulator %s\n",regulators[i].id);
         return ;
       }

       /* enable the regulator or ldo */
       if(vreg_enable(vreg))
       {
         pr_err("failed to enable the regulator %s\n",regulators[i].id);
         return ;
       }
    }
   pr_debug("Successfully enabled all regulators\n");
   return;
}


/* GGSM sc47.yun CSR7820 Project 2012.04.23 */
#ifdef CONFIG_BT_CSR_7820
extern int bluesleep_start(void);
extern void bluesleep_stop(void);

#ifdef WLAN_33V_CONTROL_FOR_BT_ANTENNA
extern void bluetooth_setup_ldo_33v(int on)
{
	wlan_setup_ldo_33v(WLAN_33V_BT_FLAG, on);
}
#endif

static uint32_t bt_config_power_on_REV02[] = {
	GPIO_CFG(GPIO_RST_BT, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_EN, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*GPIO_CFG(GPIO_BT_WAKE, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),*/
	GPIO_CFG(GPIO_BT_UART_RTS, 2/*1*/, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_16MA),
	GPIO_CFG(GPIO_BT_UART_CTS, 2/*1*/, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_16MA),
	GPIO_CFG(GPIO_BT_UART_RXD, 2/*1*/, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_16MA),
	GPIO_CFG(GPIO_BT_UART_TXD, 2/*1*/, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_16MA),
	GPIO_CFG(GPIO_BT_PCM_DOUT, 1, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PCM_DIN, 1, GPIO_CFG_INPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PCM_SYNC, 1, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PCM_CLK, 1, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PWR, 0, GPIO_CFG_INPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t bt_config_power_on_REV03[] = {
	GPIO_CFG(GPIO_RST_BT, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_EN, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*GPIO_CFG(GPIO_BT_WAKE, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),*/
	GPIO_CFG(GPIO_BT_UART_RTS, 2/*1*/, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_16MA),
	GPIO_CFG(GPIO_BT_UART_CTS, 2/*1*/, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_16MA),
	GPIO_CFG(GPIO_BT_UART_RXD, 2/*1*/, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_16MA),
	GPIO_CFG(GPIO_BT_UART_TXD, 2/*1*/, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_16MA),
	GPIO_CFG(GPIO_BT_PCM_DOUT, 1, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PCM_DIN, 1, GPIO_CFG_INPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PCM_SYNC, 1, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PCM_CLK, 1, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PWR, 0, GPIO_CFG_INPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t bt_config_power_off_REV02[] = {
	GPIO_CFG(GPIO_RST_BT, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_EN, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*GPIO_CFG(GPIO_BT_WAKE, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),*/
	GPIO_CFG(GPIO_BT_UART_RTS, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_UART_CTS, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_UART_RXD, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_UART_TXD, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PCM_DOUT, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PCM_DIN, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PCM_SYNC, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PCM_CLK, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PWR, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static uint32_t bt_config_power_off_REV03[] = {
	GPIO_CFG(GPIO_RST_BT, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_EN, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*GPIO_CFG(GPIO_BT_WAKE, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),*/
	GPIO_CFG(GPIO_BT_UART_RTS, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_UART_CTS, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_UART_RXD, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_UART_TXD, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PCM_DOUT, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PCM_DIN, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PCM_SYNC, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PCM_CLK, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_BT_PWR, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static int bluetooth_power(int on)
{
	printk(KERN_DEBUG "%s\n", __func__);

	if (on) {

		pr_info("config_gpio_table bt pwr on HW rev = %d\n",
						board_hw_revision);

		if (board_hw_revision <= 2) {
			gpio_set_value(GPIO_BT_EN, 1);
			msleep(100);
			config_gpio_table(bt_config_power_on_REV02,
					ARRAY_SIZE(bt_config_power_on_REV02));
		} else {
			config_gpio_table(bt_config_power_on_REV03,
					ARRAY_SIZE(bt_config_power_on_REV03));
		}

		msleep(500);
		pr_info("bluetooth_power GPIO_RST_BT:%d\n",
				gpio_get_value(GPIO_RST_BT));

		printk(KERN_DEBUG "not use GPIO_BT_WAKE\n");
		gpio_direction_output(GPIO_RST_BT, GPIO_WLAN_LEVEL_HIGH);
		msleep(50);
		gpio_direction_output(GPIO_RST_BT, GPIO_WLAN_LEVEL_LOW);

		msleep(100);
		gpio_direction_output(GPIO_RST_BT, GPIO_WLAN_LEVEL_HIGH);

		bluesleep_start();
	} else {
		bluesleep_stop();
		msleep(10);
		gpio_direction_output(GPIO_RST_BT, GPIO_WLAN_LEVEL_LOW);/* BT_RESET */

		printk(KERN_DEBUG "config_gpio_table bt pwr off\n");

		if (board_hw_revision <= 2) {
			config_gpio_table(bt_config_power_off_REV02,
					ARRAY_SIZE(bt_config_power_off_REV02));
			gpio_set_value(GPIO_BT_EN, 0);
		} else {
			config_gpio_table(bt_config_power_off_REV03,
					ARRAY_SIZE(bt_config_power_off_REV03));
		}
	}
	return 0;
}


static void __init bt_power_init(void)
{
	int rc = 0;

/*
#ifdef CONFIG_SAMSUNG_LPM_MODE
#if defined(CONFIG_TARGET_LOCALE_KOR_SKT) || defined(CONFIG_TARGET_LOCALE_KOR_KT) || defined(CONFIG_TARGET_LOCALE_KOR_LGU) \
	|| defined(CONFIG_TARGET_LOCALE_USA_ATT) || defined (CONFIG_TARGET_LOCALE_JPN_NTT)
	if(charging_mode_from_boot == 1)
		return;
#endif
#endif
*/
	pr_info("bt_power_init \n");

	msm_bt_power_device_csr.dev.platform_data = &bluetooth_power;

	pr_info("bt_gpio_init:low\n");

	if (board_hw_revision <= 2) {
		config_gpio_table(bt_config_power_off_REV02,
			ARRAY_SIZE(bt_config_power_off_REV02));
	} else {
		config_gpio_table(bt_config_power_off_REV03,
			ARRAY_SIZE(bt_config_power_off_REV03));
	}

	rc = gpio_request(GPIO_RST_BT, "BT_RST");
	if (!rc)
		gpio_direction_output(GPIO_RST_BT, 0);
	else {
		pr_err("%s: gpio_direction_output %d = %d\n", __func__,
			GPIO_RST_BT, rc);
		gpio_free(GPIO_RST_BT);
	}

	if (board_hw_revision <= 2) {
		rc = gpio_request(GPIO_BT_EN, "BT_EN");
		if (!rc)
			gpio_direction_output(GPIO_BT_EN, 0);
		else {
			pr_err("%s: gpio_direction_output %d = %d\n", __func__,
				GPIO_BT_EN, rc);
			gpio_free(GPIO_BT_EN);
		}
	}

	/*
	#ifndef CONFIG_TARGET_LOCALE_KOR_SKT
	rc = gpio_request(GPIO_BT_REG_ON, "BT_REGON");
	if (!rc)
		gpio_direction_output(GPIO_BT_REG_ON, 0);
	else {
		pr_err("%s: gpio_direction_output %d = %d\n", __func__,
			GPIO_BT_REG_ON, rc);
		gpio_free(GPIO_BT_REG_ON);
	}

	pr_err("%s: gpio_direction_output success (GPIO: %d , %d)\n", __func__,
			GPIO_RST_BT, GPIO_BT_REG_ON);
	#endif
	*/
}
#endif
/* GGSM sc47.yun CSR7820 Project end */

static void __init msm7x2x_init(void)
{
	msm7x2x_misc_init();
    /* Initialize the regulators */
	msm7x27a_init_regulators();

	/* Enable the Required regulators */
    msm7x27a_enable_regulators();

	/* Common functions for SURF/FFA/RUMI3 */
	msm_device_i2c_init();
	msm7x27a_init_ebi2();

//	msm7x27a_cfg_uart2dm_serial();
#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(UART1DM_RX_GPIO);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
	msm_otg_pdata.swfi_latency =
		msm7x27a_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#endif
	msm_device_gadget_peripheral.dev.platform_data =
		&msm_gadget_pdata;
	msm7x27a_cfg_smsc911x();
#ifdef CONFIG_SAMSUNG_JACK
		sec_jack_gpio_init();
#endif
	platform_add_devices(msm_footswitch_devices,
			msm_num_footswitch_devices);
	platform_add_devices(surf_ffa_devices,
			ARRAY_SIZE(surf_ffa_devices));
		if (!kernel_uart_flag)
		{
			platform_device_register(&msm_device_uart_dm2);
		}
	/* Ensure ar6000pm device is registered before MMC/SDC */
	msm7x27a_init_ar6000pm();
	msm7x27a_init_mmc();

//	lcdc_amazing_gpio_init();
	mipi_kyle_gpio_init();
	msm_fb_add_devices();


#ifdef CONFIG_USB_EHCI_MSM_72K
	msm7x2x_init_host();
#endif

	msm_pm_set_platform_data(msm7x27a_pm_data,
				ARRAY_SIZE(msm7x27a_pm_data));

	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));

	register_i2c_devices();
	wlan_power_init();
/*#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	msm7627a_bt_power_init();*/
/* GGSM sc47.yun CSR7820 Project 2012.04.23 */
/*#elif defined CONFIG_BT_CSR_7820*/
	bt_power_init();
/* GGSM sc47.yun CSR7820 Project end */
/*#endif*/

#if defined(CONFIG_TOUCHSCREEN_ZINITIX_TREBON) || defined(CONFIG_TOUCHSCREEN_ZINITIX_AMAZING)
	tsp_power_on();
#endif

	samsung_sys_class_init();
	i2c_register_board_info( 2, touch_i2c_devices, ARRAY_SIZE(touch_i2c_devices));
	i2c_register_board_info( 3, fsa880_i2c_devices, ARRAY_SIZE(fsa880_i2c_devices));
#ifdef CONFIG_BQ27425_FUEL_GAUGE
	i2c_register_board_info(6, fg_i2c_devices, ARRAY_SIZE(fg_i2c_devices));
#endif
#if defined(CONFIG_BATTERY_MAX17040)
	i2c_register_board_info(MSM_FUELGAUGE_I2C_BUS_ID
	, fg_i2c_board_info, ARRAY_SIZE(fg_i2c_board_info));
#endif
#ifdef CONFIG_MAX17048_FUELGAUGE
	i2c_register_board_info(6, fg_smb_i2c_devices
	, ARRAY_SIZE(fg_smb_i2c_devices));
#endif

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
		atmel_ts_i2c_info,
		ARRAY_SIZE(atmel_ts_i2c_info));

#if defined(CONFIG_MSM_CAMERA)
	msm7627a_camera_init();
#endif

	keypad_gpio_init();
	platform_device_register(&kp_pdev);
	platform_device_register(&hs_pdev);

	/* configure it as a pdm function*/
	if (gpio_tlmm_config(GPIO_CFG(LED_GPIO_PDM, 3,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config for %d failed\n",
			__func__, LED_GPIO_PDM);
	else
		platform_device_register(&led_pdev);

#ifdef CONFIG_MSM_RPC_VIBRATOR
	if (machine_is_msm7x27a_ffa() || machine_is_msm7625a_ffa())
		//msm_init_pmic_vibrator();
#endif
	/*7x25a kgsl initializations*/
	msm7x25a_kgsl_3d0_init();
}

static void __init msm7x2x_init_early(void)
{
	msm_msm7x2x_allocate_memory_regions();
}

MACHINE_START(MSM7X27A_RUMI3, "QCT MSM7x27a RUMI3")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7627a_rumi3_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_SURF, "QCT MSM7x27a SURF")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7X27A_FFA, "QCT MSM7x27a FFA")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7x27a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7625A_SURF, "QCT MSM7625a SURF")
	.boot_params    = PHYS_OFFSET + 0x100,
	.map_io         = msm_common_io_init,
	.reserve        = msm7x27a_reserve,
	.init_irq       = msm_init_irq,
	.init_machine   = msm7x2x_init,
	.timer          = &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7625A_FFA, "QCT MSM7625a FFA")
	.boot_params    = PHYS_OFFSET + 0x100,
	.map_io         = msm_common_io_init,
	.reserve        = msm7x27a_reserve,
	.init_irq       = msm_init_irq,
	.init_machine   = msm7x2x_init,
	.timer          = &msm_timer,
	.init_early     = msm7x2x_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
