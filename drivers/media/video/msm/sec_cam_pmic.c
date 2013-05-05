

/***************************************************************
  CAMERA Power control
 ****************************************************************/


#include "sec_cam_pmic.h"

#include <mach/gpio.h>
#include <asm/gpio.h>

#include <linux/clk.h>
#include <linux/io.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>

#include <linux/regulator/consumer.h>
#include <mach/vreg.h>
#include <mach/camera.h>
#include "sec_cam_pmic.h"
#define CAM_TEST_REV03 //temp, rev03

#ifdef CONFIG_MACH_KYLE_I
struct regulator *l1, *l6, *l15, *l17;
#else
struct regulator *s2, *s3;
struct vreg *l1, *l6, *l15, *l17;
#endif

void cam_ldo_power_on2(void)
{
	int ret;

	printk("#### cam_ldo_power_on ####\n");

}
#if defined(CONFIG_SR300PC20)

/* CAM power
	CAM_SENSOR_A_2.8		:  VREG_L17		: l17
	CAM_SENSOR_IO_1.8	: VREG_L15		: l15
	3M_CORE_1.2			: VREG_L6		: l6
*/
void cam_ldo_power_on(void)
{
	int ret;
	struct vreg *l6, *l15, *l17;
	unsigned int mclk_cfg;
	pr_info("#### cam_ldo_power_on ####\n");

	gpio_tlmm_config(GPIO_CFG(15, 1, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA),
			GPIO_CFG_ENABLE);

	/*l15 = vreg_get(NULL, "vcamio");*/
	l15 = vreg_get(NULL, "ldo15");


	if (!l15)
		pr_err("[SR300PC20]%s: VREG L15 get failed\n", __func__);

	if (vreg_set_level(l15, 1800))
		pr_err("[SR300PC20]%s: vreg_set_level failed\n", __func__);

	if (vreg_enable(l15))
		pr_err("[SR300PC20]%s: reg_enable failed\n", __func__);

	usleep(10);

	/*17 = vreg_get(NULL, "vcama");*/
	l17 = vreg_get(NULL, "ldo17");


	if (!l17)
		pr_err("[SR300PC20]%s: VREG L17 get failed\n", __func__);

	if (vreg_set_level(l17, 2800))
		pr_err("[SR300PC20]%s: vreg_set_level failed\n", __func__);

	if (vreg_enable(l17))
		pr_err("![SR300PC20]%s: reg_enable failed\n", __func__);

	usleep(10);

	/*l6 = vreg_get(NULL, "vcamc");*/
	l6 = vreg_get(NULL, "ldo6");

	if (!l6)
		pr_err("[SR300PC20]%s: VREG L6 get failed\n", __func__);

	if (vreg_set_level(l6, 1200))
		pr_err("[SR300PC20]%s: vreg_set_level failed\n", __func__);

	if (vreg_enable(l6))
		pr_err("!![SR300PC20]%s:   reg_enable failed\n", __func__);

	usleep(10);
}

void cam_ldo_power_off(void)
{
	int ret;
	struct vreg *l6, *l15, *l17;

	pr_info("#### cam_ldo_power_off ####\n");
/*
	l6 = vreg_get(NULL, "vcamc");
	l15 = vreg_get(NULL, "vcamio");
	l17 = vreg_get(NULL, "vcama");
*/

	l6 = vreg_get(NULL, "ldo6");
	l15 = vreg_get(NULL, "ldo15");
	l17 = vreg_get(NULL, "ldo17");

	vreg_disable(l6);
	usleep(10);

	vreg_disable(l17);
	usleep(10);

	vreg_disable(l15);
	usleep(10);

	gpio_set_value_cansleep(CAM_IO_EN, LOW);

}
#else

void cam_ldo_power_on(void)
{
	int ret;
	unsigned int mclk_cfg;
	printk("#### cam_ldo_power_on ####\n");

#ifdef CONFIG_MACH_KYLE_I
	l17 = regulator_get(NULL, "ldo17");
	if (!l17)
		printk(KERN_DEBUG "%s: VREG L17 get failed\n", __func__);

	if (regulator_set_voltage(l17 , 2800000, 2800000))
		printk(KERN_DEBUG "%s: vreg_set_level failed\n", __func__);

	if (regulator_enable(l17))
		printk(KERN_DEBUG "%s: reg_enable failed\n", __func__);

	msleep(20); /*t<=2ms */

	/*VCAM-CORE 1.2V*/
		l6 = regulator_get(NULL, "ldo06");
	if (!l6)
		printk(KERN_DEBUG "%s: VREG l6 get failed\n", __func__);

	if (regulator_set_voltage(l6 , 1200000, 1200000))
		printk(KERN_DEBUG "%s: vreg_set_level failed\n", __func__);

	if (regulator_enable(l6))
		printk(KERN_DEBUG "%s: reg_enable failed\n", __func__);

	msleep(20); /*t<=2ms */

	gpio_set_value_cansleep(CAM_IO_EN, 1);
	msleep(20);
#else
	//Power On V_CAMD_1.8V
	#ifdef CONFIG_MACH_AMAZING_CDMA
	l15 = vreg_get(NULL, "vcamio");
	#else
	l15 = vreg_get(NULL, "ldo15");
	#endif
	if(!l15){
		printk("[S5K5CCAF]%s: VREG L15 get failed\n", __func__);
	}
	if(vreg_set_level(l15, 1800)){
		printk("[S5K5CCAF]%s: vreg_set_level failed\n", __func__);
	}
	if (vreg_enable(l15)) {
		printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![S5K5CCAF]%s: reg_enable failed\n", __func__);
	}
	mdelay(1);//t<=2ms


	//Power On V_CAMA_2.8V
#ifdef CONFIG_MACH_AMAZING_CDMA
	/*l1 = vreg_get(NULL, "ldo1");*/
	l1 = vreg_get(NULL, "vcama");
	if (!l1)
		printk(KERN_DEBUG "%s: VREG L1 get failed\n", __func__);
	if (vreg_set_level(l1 , 2800))
		printk(KERN_DEBUG "%s: vreg_set_level failed\n", __func__);
	if (vreg_enable(l1))
		printk(KERN_DEBUG "%s: reg_enable failed\n", __func__);

	mdelay(1); /*t<=2ms */
	gpio_set_value_cansleep(CAM_C_EN, 1);
	 mdelay(2);

#else
	l17 = vreg_get(NULL, "ldo17");
	if(!l17){
		printk("[S5K5CCAF]%s: VREG L1 get failed\n", __func__);
	}
	if(vreg_set_level(l17, 2800)){
		printk("[S5K5CCAF]%s: vreg_set_level failed\n", __func__);
	}
	if (vreg_enable(l17)) {
		printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![S5K5CCAF]%s: reg_enable failed\n", __func__);
	}

	mdelay(1);//t<=2ms

	//Power On GPIO_107 control (V_CAMC_1.8V)
	//Power On V_CAMA_2.8V
	l6 = vreg_get(NULL, "ldo6");
	if(!l6){
		printk("[S5K5CCAF]%s: VREG L1 get failed\n", __func__);
	}
	if(vreg_set_level(l6, 1200)){
		printk("[S5K5CCAF]%s: vreg_set_level failed\n", __func__);
	}
	if (vreg_enable(l6)) {
		printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![S5K5CCAF]%s: reg_enable failed\n", __func__);
	}
	mdelay(1); /*t<=2ms */
#endif
#endif
}


void cam_ldo_power_off(void)
{
	int ret;

#ifdef CONFIG_MACH_AMAZING_CDMA
	gpio_set_value_cansleep(CAM_C_EN, 0);
	mdelay(1);
#endif

#ifdef CONFIG_MACH_KYLE_I

	gpio_set_value_cansleep(CAM_IO_EN, 0);
	msleep(20);
	if (l6)
		ret = regulator_disable(l6);
			/*ret=vreg_disable(l6);*/
	if (ret)
		printk(KERN_DEBUG "%s: error disabling regulator\n", __func__);
		/*regulator_put(l8);*/

	msleep(20);

	if (l17)
		ret = regulator_disable(l17);
		/*ret = vreg_disable(l17);*/

	if (ret)
		printk(KERN_DEBUG "%s: error disabling regulator\n", __func__);
		/*regulator_put(l8);*/

	msleep(20);
#else


	if (l6) {
		//ret=regulator_disable(l17);
		ret=vreg_disable(l6);
		if (ret) {
			printk("%s: error disabling regulator\n", __func__);
		}
		//regulator_put(l8);
	}
	mdelay(10);
#ifdef CONFIG_MACH_AMAZING_CDMA
	if (l1) {
		ret = vreg_disable(l1);
		if (ret)
			printk(KERN_DEBUG "%s: error disabling regulator\n", __func__);
	}
#else
	if (l17) {
		//ret=regulator_disable(l17);
		ret=vreg_disable(l17);
		if (ret)
			printk(KERN_DEBUG "%s: error disabling regulator\n", __func__);
		//regulator_put(l8);
	}

#endif
	mdelay(10);

	//Power Off V_CAMD_1.8V
	if (l15) {
		ret=vreg_disable(l15);
		if (ret) {
			printk("%s: error disabling regulator\n", __func__);
		}
	}
	mdelay(5);
#endif
}
#endif
