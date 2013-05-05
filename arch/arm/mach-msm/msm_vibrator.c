/* include/asm/mach-msm/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/sched.h>

#include <mach/msm_rpcrouter.h>

//#include <mach/clk.h>
#include <mach/samsung_vibe.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <mach/pmic.h>

struct work_struct	 vibe_work;

struct clk *android_vib_clk; /* gp_clk */

#define GP_CLK_M_DEFAULT			21
#define GP_CLK_N_DEFAULT			18000
#define GP_CLK_D_DEFAULT			9000	/* 50% duty cycle */ 
#define IMM_PWM_MULTIPLIER		    17778	/* Must be integer */

/*
 * ** Global variables for LRA PWM M,N and D values.
 * */
VibeInt32 g_nLRA_GP_CLK_M = GP_CLK_M_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_N = GP_CLK_N_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_D = GP_CLK_N_DEFAULT;
VibeInt32 g_nLRA_GP_CLK_PWM_MUL = IMM_PWM_MULTIPLIER;

static struct hrtimer vibe_timer;
static struct regulator *vreg_msm_vibrator;
static int enabled = 0;

#if defined(CONFIG_MACH_AMAZING_CDMA) || defined(CONFIG_MACH_KYLE_I)
static int is_vibe_on;
#endif

#if defined(CONFIG_MACH_KYLE_I)
#define VIB_ON_GPIO		111
#endif

static int msm_vibrator_suspend(struct platform_device *pdev, pm_message_t state);
static int msm_vibrator_resume(struct platform_device *pdev);
static int msm_vibrator_probe(struct platform_device *pdev);
static int msm_vibrator_exit(struct platform_device *pdev);
static int msm_vibrator_power(int power_mode);


/* Variable for setting PWM in Force Out Set */
VibeInt32 g_nForce_32 = 0;

/*
 * This function is used to set and re-set the GP_CLK M and N counters
 * to output the desired target frequency.
 * 
 */

/* for the suspend/resume VIBRATOR Module */
static struct platform_driver msm_vibrator_platdrv = 
{
	.probe   = msm_vibrator_probe,
	.suspend = msm_vibrator_suspend,
	.resume  = msm_vibrator_resume,
	.remove  = msm_vibrator_exit,
	.driver = 
	{
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int msm_vibrator_suspend(struct platform_device *pdev, pm_message_t state)
{

#if 0
	if(is_vibe_on) {
		clk_disable(android_vib_clk);
		is_vibe_on = 0;
	}
#endif
#if defined(CONFIG_MACH_AMAZING_CDMA) || defined(CONFIG_MACH_KYLE_I)
	msm_vibrator_power(VIBRATION_OFF);
#endif
	printk("[VIB] susepend\n");
	return VIBE_S_SUCCESS;
}

static int msm_vibrator_resume(struct platform_device *pdev)
{

	//	msm_vibrator_power(VIBRATION_ON);

	printk("[VIB] resume\n");
	return VIBE_S_SUCCESS;
}

static int msm_vibrator_exit(struct platform_device *pdev)
{
	printk("[VIB] EXIT\n");
	return 0;
}


static int msm_vibrator_power(int on)
{
	int ret = 0;
#if !defined(CONFIG_MACH_AMAZING_CDMA) && !defined(CONFIG_MACH_KYLE_I)
	if (on) {
#if defined(CONFIG_MACH_KYLE)
		ret = regulator_set_voltage(vreg_msm_vibrator, 3050000, 3050000);
#else
		ret = regulator_set_voltage(vreg_msm_vibrator, 3000000,3000000);
#endif
		if (ret) {
			printk(KERN_ERR "%s: vreg set level failed (%d)\n",
					__func__, ret);
			regulator_put(vreg_msm_vibrator);
			return -EIO;
		}
		if (!enabled) {
			enabled = 1;
			ret = regulator_enable(vreg_msm_vibrator);
		}
		if (ret) {
			printk(KERN_ERR "%s: vreg enable failed (%d)\n",
					__func__, ret);
			return -EIO;
		}
		mdelay(10);
	}
	else {
		if (enabled) {
			enabled = 0;
			ret = regulator_disable(vreg_msm_vibrator);
		}
		if (ret) {
			printk(KERN_ERR "%s: vreg disable failed (%d)\n",
					__func__, ret);
			return -EIO;
		}
	}
#else
	static int nStatus;
	unsigned int vib_voltage = 0;

	printk(KERN_DEBUG "%s on = %d nStatus = %d\n", __func__, on, nStatus);

	if (nStatus == on)	{
		printk(KERN_DEBUG "%s set already!\n", __func__);
		return VIBE_E_FAIL;
	}
	else {
		nStatus = on;
	}

	vib_voltage = 3100;

#if defined(CONFIG_MACH_KYLE_I)
	if (on)
	{
		ret = pmic_vib_mot_set_volt(vib_voltage);
		gpio_set_value_cansleep(VIB_ON_GPIO,1);
	}
	else
	{
		ret = pmic_vib_mot_set_volt(0);
		gpio_set_value_cansleep(VIB_ON_GPIO,0);
	}

#else
	if (on)
		ret = pmic_vib_mot_set_volt(vib_voltage);
	else
		ret = pmic_vib_mot_set_volt(0);
#endif
	printk(KERN_DEBUG "%s status = %d\n", __func__, ret);
#endif
	return VIBE_S_SUCCESS;
}


#if 0
static int vibe_set_pwm_freq(int nForce)
{
#if 1
	/* Put the MND counter in reset mode for programming */
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 0);
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_PRE_DIV_SEL_BMSK, 0 << HWIO_GP_NS_REG_PRE_DIV_SEL_SHFT); /* P: 0 => Freq/1, 1 => Freq/2, 4 => Freq/4 */
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_SRC_SEL_BMSK, 0 << HWIO_GP_NS_REG_SRC_SEL_SHFT); /* S : 0 => TXCO(19.2MHz), 1 => Sleep XTAL(32kHz) */
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_MODE_BMSK, 2 << HWIO_GP_NS_REG_MNCNTR_MODE_SHFT); /* Dual-edge mode */
	HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_M_VAL_BMSK, g_nLRA_GP_CLK_M << HWIO_GP_MD_REG_M_VAL_SHFT);
	g_nForce_32 = ((nForce * g_nLRA_GP_CLK_PWM_MUL) >> 8) + g_nLRA_GP_CLK_D;
	//		printk("%s, g_nForce_32 : %d\n",__FUNCTION__,g_nForce_32);
	HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_D_VAL_BMSK, ( ~((VibeInt16)g_nForce_32 << 1) ) << HWIO_GP_MD_REG_D_VAL_SHFT);
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_GP_N_VAL_BMSK, ~(g_nLRA_GP_CLK_N - g_nLRA_GP_CLK_M) << HWIO_GP_NS_REG_GP_N_VAL_SHFT);
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 1 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT);                    /* Enable M/N counter */
	printk("%x, %x, %x\n",( ~((VibeInt16)g_nForce_32 << 1) ) << HWIO_GP_MD_REG_D_VAL_SHFT,~(g_nLRA_GP_CLK_N - g_nLRA_GP_CLK_M) << HWIO_GP_NS_REG_GP_N_VAL_SHFT,1 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT);
#else
	clk_set_rate(android_vib_clk,32583);
#endif	
	return VIBE_S_SUCCESS;
}


static void set_pmic_vibrator(int on)
{
	//	printk("[VIB] %s, input : %s\n",__func__,on ? "ON":"OFF");
	if (on) {
		clk_enable(android_vib_clk);
		gpio_direction_output(VIB_ON, VIBRATION_ON);
		is_vibe_on = 1;
	} else {
		if(is_vibe_on) {
			gpio_direction_output(VIB_ON, VIBRATION_OFF);
			clk_disable(android_vib_clk);
			is_vibe_on = 0;
		}
	}

}
#endif

#if 0
static void pmic_vibrator_on(struct work_struct *work)
{
	set_pmic_vibrator(VIBRATION_ON);
}

static void pmic_vibrator_off(struct work_struct *work)
{
	set_pmic_vibrator(VIBRATION_OFF);
}

static void timed_vibrator_on(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_on);
}

static void timed_vibrator_off(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_off);
}
//#else
static void pmic_vibrator_on(void)
{
	set_pmic_vibrator(VIBRATION_ON);
	msm_vibrator_power(VIBRATION_ON);
}

static void pmic_vibrator_off(void)
{
	msm_vibrator_power(VIBRATION_OFF);
	set_pmic_vibrator(VIBRATION_OFF);
}

#endif

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	unsigned long flags;

	hrtimer_cancel(&vibe_timer);
	printk(KERN_ERR "[VIB] enable %d\n", value);
	if (value == 0) {
		msm_vibrator_power(VIBRATION_OFF);
	}
	else {
		if(value < 0)
			value = ~value;
		printk(KERN_ERR "[VIB] ON, %d ms\n", value);

		value = (value > 15000 ? 15000 : value);
		msm_vibrator_power(VIBRATION_ON);

		hrtimer_start(&vibe_timer,
				ktime_set(value / 1000, (value % 1000) * 1000000),
				HRTIMER_MODE_REL);
	}
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return (ktime_to_ms(r));
	} else
		return 0;
}

#if defined(CONFIG_MACH_AMAZING_CDMA) || defined(CONFIG_MACH_KYLE) || defined(CONFIG_MACH_KYLE_I)
static int vibrator_timer_do_work(struct work_struct *ignored)
{
	unsigned int remain;

	printk(KERN_DEBUG "[VIB] %s\n", __func__);

	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		remain = r.tv.sec * 1000000 + r.tv.nsec;
		remain = remain / 1000;
		if (r.tv.sec < 0)
			remain = 0;
		printk(KERN_DEBUG "[VIB] hrtimer active, remain:%d\n", remain);

		if (!remain)
			msm_vibrator_power(VIBRATION_OFF);
	}
	else {
		printk(KERN_DEBUG "[VIB] hrtimer not active\n");
		msm_vibrator_power(VIBRATION_OFF);
	}

	return 0;
}


static DECLARE_WORK(vibrator_timer_work, vibrator_timer_do_work);
#endif


static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
#if 0
	timed_vibrator_off(NULL);
	return HRTIMER_NORESTART;
#else
	unsigned int remain;

	printk("[VIB] %s\n",__func__);
#if !defined(CONFIG_MACH_AMAZING_CDMA) && !defined(CONFIG_MACH_KYLE) && !defined(CONFIG_MACH_KYLE_I)
	if(hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		remain=ktime_to_ms(r);
		remain = remain / 1000;
		if(r.tv.sec < 0) {
			remain = 0;
		}
		printk("[VIB] hrtimer active, remain:%d\n",remain);
		if(!remain) {
			msm_vibrator_power(VIBRATION_OFF);
		}

	} else {
		printk("[VIB] hrtimer not active\n");
		msm_vibrator_power(VIBRATION_OFF);
	}
#else
	schedule_work(&vibrator_timer_work);
#endif

	return HRTIMER_NORESTART;
#endif
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

static int msm_vibrator_power_work()
{
	return(msm_vibrator_power(VIBRATION_OFF));
}

static int msm_vibrator_probe(struct platform_device *pdev)
{
	int rc;

#if !defined(CONFIG_MACH_AMAZING_CDMA) && !defined(CONFIG_MACH_KYLE) && !defined(CONFIG_MACH_KYLE_I)
	INIT_WORK(&vibe_work, msm_vibrator_power_work);
#endif
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	rc = timed_output_dev_register(&pmic_vibrator);

	if (rc < 0)
		goto err_read_vib;
#if 0
	msm_vibrator_power(VIBRATION_ON);

	/* Vibrator init sequence
	 * 1. power on ( vreg get )
	 * 2. clock get & enable ( gp_clk )
	 * 3. VIB_EN on
	 */

	android_vib_clk = clk_get(NULL,"gp_clk");

	if(IS_ERR(android_vib_clk)) {
		printk("android vib clk failed!!!\n");
	} else {
		printk("THNAK YOU!!\n");
	}
	vibe_set_pwm_freq(216);
#endif
	return 0;

err_read_vib:
	printk(KERN_ERR "[VIB] timed_output_dev_register fail (rc=%d)\n", rc);
	return rc;
}

static int __init msm_init_pmic_vibrator(void)
{
	int nRet;

	nRet = platform_driver_register(&msm_vibrator_platdrv);

	printk("[VIB] platform driver register result : %d\n",nRet);
	if (nRet)
	{ 
		printk("[VIB] platform_driver_register failed\n");
	}
#if !defined(CONFIG_MACH_AMAZING_CDMA) && !defined(CONFIG_MACH_KYLE_I)
	vreg_msm_vibrator = regulator_get(NULL, "ldo01");

	if (IS_ERR (vreg_msm_vibrator)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
				__func__, PTR_ERR(vreg_msm_vibrator));
		return PTR_ERR(vreg_msm_vibrator);
	}
#endif

	return nRet;

}

void __exit msm_exit_pmic_vibrator(void)
{
	platform_driver_unregister(&msm_vibrator_platdrv);

}

module_init(msm_init_pmic_vibrator);
module_exit(msm_exit_pmic_vibrator);


MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");

