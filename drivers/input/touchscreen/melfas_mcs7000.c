/* drivers/input/touchscreen/melfas_ts_i2c_tsi.c
 *
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/lcd.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/miscdevice.h>
#include <linux/completion.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <mach/vreg.h>

#include "mcs7000_download.h"
#include <linux/i2c/melfas_ts.h>
#include <linux/input/mt.h>


#define INPUT_INFO_REG 0x10
#define IRQ_TOUCH_INT   MSM_GPIO_TO_INT(GPIO_TOUCH_INT)

#if defined(CONFIG_MACH_ROOKIE2) || defined (CONFIG_TOUCHSCREEN_MELFAS_TS)
//#if defined(CONFIG_MACH_ROOKIE2)
#define __TOUCH_KEY__
#define __TOUCH_DEBUG__
#endif


#if 0	// firmware update with schedule
#define FIRMWARE_WORK_CHECK_TIMEOUT (10 * HZ)
static void melfas_firmware_download(void);
static DECLARE_DELAYED_WORK(firmware_work, melfas_firmware_download);
#endif

#define PREVAIL2_SYSFS_ADDED

#define FINGER_NUM	      5 
#define RECOMMEND_FW	 0x02

// TSP Command Test Binary
#if defined (CONFIG_TOUCHSCREEN_MELFAS_TS)
#define MIP_SMD_TX_CH_NUM 0x0B
#define UCKEY 3		/*the number of using touch key*/
#define NUMBER_X_NODE 11
#define NUMBER_Y_NODE 15
#endif

#undef CONFIG_CPU_FREQ
#undef CONFIG_MOUSE_OPTJOY

#ifdef CONFIG_CPU_FREQ
#include <plat/s3c64xx-dvfs.h>
#endif

//#include <linux/i2c/fsa9480.h> 

static uint8_t buf1[6];
static int debug_level = 5;
#define debugprintk(level,x...)  do {if(debug_level>=level) printk(x);}while(0)

extern int mcsdl_download_binary_data(void);
//extern int fsa9480_i2c_read(unsigned char u_addr, unsigned char *pu_data); 
//static u8 fsa9480_device1 = 0x0;

#ifdef CONFIG_MOUSE_OPTJOY
extern int get_sending_oj_event();
#endif

#if defined(CONFIG_MACH_VITAL2) || defined (CONFIG_MACH_ROOKIE2) || defined (CONFIG_TOUCHSCREEN_MELFAS_TS)
#define TSP_TEST_MODE
#define TSP_SDCARD_UPDATE
#ifdef TSP_SDCARD_UPDATE
static bool sdcard_update = false;
#endif
#endif

#if defined(CONFIG_MACH_VITAL2) || defined (CONFIG_TOUCHSCREEN_MELFAS_TS)
#define TSP_TA_NOISE
#endif
extern struct class *sec_class;
#define TS_MAX_TOUCH 5

struct input_info {
	int max_x;
	int max_y;
	int state;
	int x;
	int y;
	int z;
	int width;
	int finger_id;
};



#ifdef PREVAIL2_SYSFS_ADDED

#define TSP_BUF_SIZE 1024
#define FAIL -1
#define TSP_CMD_STR_LEN			32
#define TSP_CMD_RESULT_STR_LEN		512
#define TSP_CMD_PARAM_NUM		8
#define NODE_NUM	117 /* 18x10 */

#define TSP_CMD(name, func) .cmd_name = name, .cmd_func = (void (*)(struct device *, struct device_attribute *, const char *, size_t))func

struct tsp_cmd {
	struct list_head	list;
	const char	*cmd_name;
	void	(*cmd_func)(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t size);
};




static ssize_t touchkey_firm_version_panel_show(struct device *dev, \
		struct device_attribute *attr, char *buf);
static void set_cmd_result(char *buff, int len);
static void not_support_cmd(struct device *dev, struct device_attribute *attr,
						char *buf);
static void set_default_result();
struct tsp_cmd tsp_cmds[] = {
	{TSP_CMD("get_fw_ver_ic", touchkey_firm_version_panel_show),},
	/* ADD NEW (NEEDED) CMDS HERE AS "{TSP_CMD("CMD NAME", CMD_FUNC),}," */
	{TSP_CMD("not_support_cmd", not_support_cmd),},



};

#endif



struct melfas_ts_driver {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int irq;
	int hw_rev;
	int fw_ver;
	int com_type;
	struct input_info info[FINGER_NUM+1];
	int suspended;
	struct early_suspend	early_suspend;
#ifdef TSP_TA_NOISE	
	struct tsp_callbacks callbacks;
	struct melfas_platform_data *pdata;	
	bool charging_status;
	bool tsp_status;
#endif

	#ifdef TSP_TEST_MODE
	struct mutex lock;
	touch_screen_driver_t *melfas_test_mode;
	touch_screen_t *touch_screen;	
	#endif	

#ifdef PREVAIL2_SYSFS_ADDED

	struct list_head			cmd_list_head;
	u8			cmd_state;
	char			cmd[TSP_CMD_STR_LEN];
	int			cmd_param[TSP_CMD_PARAM_NUM];
	char			cmd_result[TSP_CMD_RESULT_STR_LEN];
	struct mutex			cmd_lock;
	bool			cmd_is_running;
	unsigned int reference[NODE_NUM];
	unsigned int raw[NODE_NUM]; /* CM_ABS */
	unsigned int inspection[NODE_NUM];/* CM_DELTA */
	unsigned int intensity[NODE_NUM];
	bool ft_flag;

#endif

};


struct melfas_ts_driver *melfas_ts = NULL;
struct i2c_driver melfas_ts_i2c;
struct workqueue_struct *melfas_ts_wq;

static struct vreg *vreg_touch;
static struct vreg *vreg_touchio;
 
//fot test

/***************************************************************************
* Declare custom type definitions
***************************************************************************/
int melfas_ts_suspend(pm_message_t mesg);
int melfas_ts_resume(void);
void tsp_reset(void);
void melfas_ts_force_touch_up(void);

#ifdef LCD_WAKEUP_PERFORMANCE
static void ts_resume_work_func(struct work_struct *ignored);
static DECLARE_DELAYED_WORK(ts_resume_work, ts_resume_work_func);
static DECLARE_COMPLETION(ts_completion);
#endif

#ifdef TSP_TEST_MODE
static uint16_t tsp_test_temp[11];
static uint16_t tsp_test_reference[15][11];
static uint16_t tsp_test_inspection[15][11];
static uint16_t tsp_test_delta[5];
static bool sleep_state = false;
uint8_t refer_y_channel_num = 1;
uint8_t inspec_y_channel_num = 1;

int touch_screen_ctrl_testmode(int cmd, touch_screen_testmode_info_t *test_info, int test_info_num);
static int ts_melfas_test_mode(int cmd, touch_screen_testmode_info_t *test_info, int test_info_num);

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

static uint16_t melfas_ts_inspection_spec_table[TS_MELFAS_SENSING_CHANNEL_NUM*TS_MELFAS_EXCITING_CHANNEL_NUM*2] = {
	166, 234, 180, 252,	181, 252, 182, 253 , 184, 251, 183, 254, 184, 254, 184,	254, 186, 254, 186,	256, 175, 239,
	183, 236, 198, 255,	198, 255, 200, 253 , 201, 256, 202, 257, 202, 258, 202,	257, 203, 257, 200,	259, 187, 243,
	185, 237, 199, 256,	198, 256, 201, 256 , 201, 255, 202, 258, 202, 258, 203,	257, 203, 258, 201,	260, 188, 244,
	186, 239, 201, 258,	200, 258, 202, 259 , 202, 258, 203, 260, 203, 259, 203,	259, 204, 260, 201,	261, 190, 246,
	188, 241, 202, 261,	202, 261, 204, 260 , 204, 260, 206, 263, 205, 261, 206,	261, 205, 262, 203,	263, 192, 249,
	192, 245, 205, 265,	205, 264, 207, 265 , 208, 264, 206, 266, 209, 265, 207,	265, 207, 267, 205,	266, 199, 266,
	193, 250, 208, 271,	207, 271, 209, 270 , 209, 270, 210, 270, 210, 269, 211,	270, 210, 270, 210,	272, 195, 330,
	201, 259, 213, 280,	213, 278, 215, 276 , 215, 275, 214, 276, 214, 274, 213,	273, 213, 272, 209,	274, 188, 252,
	206, 265, 220, 287,	219, 286, 220, 285 , 221, 284, 220, 284, 220, 284, 220,	282, 220, 282, 215,	284, 193, 261,
	214, 278, 229, 300,	229, 299, 230, 297 , 231, 297, 231, 298, 231, 295, 230,	293, 230, 292, 225,	293, 203, 271,
	227, 292, 242, 316,	242, 315, 243, 312 , 243, 312, 243, 313, 243, 311, 242,	309, 241, 308, 236,	309, 213, 285,
	242, 314, 259, 337,	258, 336, 259, 334 , 259, 333, 258, 334, 258, 334, 256,	331, 256, 331, 251,	331, 226, 306,
	260, 342, 277, 366,	277, 365, 280, 363 , 279, 361, 279, 363, 278, 362, 278,	359, 277, 357, 273,	358, 244, 332,
	286, 380, 304, 401,	305, 399, 305, 398 , 305, 398, 305, 399, 304, 398, 305,	394, 304, 392, 299,	392, 268, 367,
	355, 480, 361, 497,	356, 492, 352, 485 , 348, 477, 343, 463, 347, 476, 352,	483, 353, 486, 355,	490, 336, 473
};
/*ABS VAL*/
#define MAX_RX_	11
#define MAX_TX_	15

static const uint16_t SCR_ABS_UPPER_SPEC[MAX_RX_][MAX_TX_] = {
	{3038, 2923, 2881, 2853, 2825, 2810, 2795, 2795,
		 2781, 2776, 2772, 2772, 2771, 2770, 2750},
	{3098, 2987, 2937, 2906, 2872, 2856, 2836, 2831,
		 2818, 2815, 2811, 2810, 2808, 2805, 2778},
	{3088, 2973, 2916, 2877, 2837, 2812, 2787, 2773,
		 2760, 2752, 2745, 2738, 2736, 2735, 2747},
	{3092, 2988, 2936, 2907, 2872, 2855, 2836, 2827,
		2815, 2811, 2807, 2805, 2803, 2800, 2773},
	{3086, 2987, 2936, 2906, 2873, 2853, 2836, 2826,
		 2813, 2810, 2806, 2803, 2802, 2798, 2772},
	{3072, 2975, 2913, 2877, 2836, 2810, 2786, 2768,
		 2753, 2746, 2738, 2732, 2730, 2727, 2738},
	{3082, 2985, 2935, 2905, 2872, 2851, 2835, 2821,
		 2810, 2805, 2802, 2798, 2797, 2793, 2766},
	{3096, 2996, 2941, 2913, 2878, 2858, 2841, 2827,
		 2815, 2811, 2807, 2803, 2802, 2798, 2771},
	{3091, 2980, 2922, 2883, 2843, 2815, 2793, 2771,
		 2758, 2750, 2741, 2735, 2733, 2731, 2742},
	{3096, 2992, 2933, 2907, 2871, 2852, 2835, 2816,
		2806, 2802, 2797, 2793, 2791, 2788, 2760},
	{3052, 2940, 2892, 2865, 2837, 2816, 2803, 2786,
		 2776, 2772, 2768, 2766, 2765, 2762, 2740},
};


static const uint16_t SCR_ABS_LOWER_SPEC[MAX_RX_][MAX_TX_] = {
	{1824, 1755, 1729, 1713, 1695, 1686, 1677, 1667,
		 1669, 1666, 1664, 1664, 1663, 1662, 1650},
	{1860, 1793, 1763, 1744, 1724, 1714, 1702, 1699,
		 1692, 1689, 1687, 1686, 1686, 1663, 1668},
	{1854, 1785, 1750, 1727, 1703, 1688, 1673, 1665,
		 1656, 1652, 1647, 1644, 1642, 1641, 1649},
	{1856, 1794, 1762, 1745, 1724, 1713, 1702, 1697,
		 1689, 1687, 1685, 1683, 1683, 1680, 1665},
	{1852, 1793, 1762, 1744, 1725, 1713, 1702, 1696,
		1689, 1686, 1684, 1683, 1682, 1680, 1664},
	{1844, 1785, 1749, 1727, 1702, 1686, 1672, 1662,
		1653, 1648, 1644, 1640, 1638, 1637, 1644},
	{1850, 1791, 1671, 1743, 1724, 1711, 1701, 1693,
		 1686, 1683, 1682, 1680, 1679, 1677, 1660},
	{1858, 1798, 1765, 1749, 1728, 1716, 1705, 1697,
		 1689, 1687, 1685, 1683, 1682, 1680, 1663},
	{1855, 1788, 1754, 1731, 1707, 1689, 1677, 1663,
		 1656, 1650, 1645, 1641, 1641, 1639, 1646},
	{1858, 1796, 1761, 1745, 1723, 1712, 1701, 1690,
		 1684, 1682, 1679, 1677, 1675, 1674, 1656},
	{1832, 1764, 1736, 1719, 1703, 1690, 1683, 1672,
		 1666, 1664, 1662, 1660, 1659, 1658, 1644},
};

static const uint16_t SCR_KEY_ABS_LOWER_SPEC[UCKEY] = {1789, 1783, 1786};

static const uint16_t SCR_KEY_ABS_UPPER_SPEC[UCKEY] = {2981, 2971, 2976};

static status_abs ;/*PASS*/
/*ABS VAL*/

/*DELTA_VAL*/

static const uint16_t SCR_DELTA_UPPER_SPEC[MAX_RX_][MAX_TX_] = {
	{832, 717, 673, 646, 618, 602, 587, 587,
		 575, 568, 566, 565, 563, 562, 542},
	{883, 771, 721, 691, 656, 640, 620, 616,
		 603, 598, 596, 593, 592, 590, 562},
	{873, 758, 701, 663, 622, 597, 572, 560,
		 545, 537, 530, 523, 521, 520, 533},
	{878, 775, 722, 693, 658, 641, 622, 615,
		602, 597, 593, 591, 590, 587, 560},
	{872, 773, 723, 693, 660, 641, 623, 613,
		601, 596, 593, 591, 590, 586, 558},
	{862, 765, 703, 667, 626, 600, 576, 558,
		543, 536, 528, 522, 520, 517, 528},
	{873, 776, 725, 695, 662, 641, 625, 612,
		 600, 596, 592, 590, 587, 585, 557},
	{880, 781, 726, 697, 662, 642, 626, 611,
		 600, 596, 591, 588, 586, 583, 556},
	{876, 765, 706, 667, 628, 600, 578, 556,
		 533, 535, 526, 520, 517, 516, 527},
	{887, 783, 725, 698, 662, 643, 626, 607,
		 597, 593, 588, 585, 582, 580, 551},
	{843, 731, 683, 656, 628, 608, 595, 577,
		 568, 563, 560, 557, 556, 553, 531},
};
/* here in upper array at 9th index of 11th row is wrong
	 so for time being i am putting 533 */
static const uint16_t SCR_DELTA_LOWER_SPEC[MAX_RX_][MAX_TX_] = {
	{530, 431, 405, 388, 372, 362, 353, 353,
		 345, 342, 340, 339, 339, 338, 326},
	{531, 463, 433, 415, 394, 384, 372, 370,
		 363, 360, 358, 357, 356, 354, 338},
	{525, 456, 421, 399, 374, 359, 344, 336,
		 327, 323, 318, 315, 313, 312, 321},
	{528, 465, 434, 417, 396, 385, 374, 369,
		 362, 359, 357, 355, 354, 353, 336},
	{524, 465, 435, 417, 396, 385, 375, 369,
		 361, 358, 357, 355, 354, 352, 336},
	{518, 459, 423, 401, 376, 360, 346, 336,
		 327, 322, 318, 314, 312, 311, 318},
	{525, 466, 435, 417, 398, 385, 375, 368,
		 360, 358, 356, 354, 353, 351, 335},
	{528, 469, 436, 419, 398, 386, 376, 367,
		 360, 358, 355, 354, 352, 351, 334},
	{526, 459, 424, 401, 378, 360, 348, 334,
		 327, 321, 316, 312, 311, 310, 317},
	{533, 471, 435, 420, 398, 387, 376, 365,
		 359, 357, 354, 351, 350, 348, 331},
	{507, 439, 411, 394, 378, 366, 357, 347,
		 342, 339, 336, 335, 334, 333, 319},
};
static const uint16_t SCR_KEY_DELTA_LOWER_SPEC[UCKEY] = {460, 457, 456};

static const uint16_t SCR_KEY_DELTA_UPPER_SPEC[UCKEY] = {766, 761, 760};

static status_delta ;/*PASS*/
/*DELTA_VAL*/

/*Intensity_VAL*/
static const uint16_t SCR_KEY_INTENSITY_LOWER_SPEC[UCKEY] = {36, 36, 36};

static const uint16_t SCR_KEY_INTENSITY_UPPER_SPEC[UCKEY] = {68, 68, 66};

static status_intensity ;/*PASS*/
/*Intensity_VAL*/
touch_screen_t touch_screen = {	
	{0},
	1,
	{0}
};

touch_screen_driver_t melfas_test_mode = {
	{
		TS_MELFAS_VENDOR_NAME,
		TS_MELFAS_VENDOR_CHIP_NAME,
		TS_MELFAS_VENDOR_ID, 
		0, 
		0, 
		TS_MELFAS_TESTMODE_MAX_INTENSITY, 
		TS_MELFAS_SENSING_CHANNEL_NUM, TS_MELFAS_EXCITING_CHANNEL_NUM, 
		2, 
		2850, 3750, 
		melfas_ts_inspection_spec_table
	},
	0,
	ts_melfas_test_mode
};
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
void melfas_ts_early_suspend(struct early_suspend *h);
void melfas_ts_late_resume(struct early_suspend *h);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#define TOUCH_HOME	KEY_HOMEPAGE
#define TOUCH_MENU	KEY_MENU
#define TOUCH_BACK	KEY_BACK
#define TOUCH_SEARCH  KEY_SEARCH

int melfas_ts_tk_keycode[] =
{ TOUCH_HOME, TOUCH_MENU, TOUCH_BACK, TOUCH_SEARCH, };

struct device *ts_dev;
struct device *ts_dev1;
struct device *sec_touchkey_dev;

void mcsdl_vdd_on(void)
{
	int ret;

	printk("[TSP] %s 1 \n",__func__);
	 /* VTOUCH_2.8V */
	int rc;
//	struct vreg *vreg_touch =NULL;

//	if (vreg_touch == NULL) {
		vreg_touch = vreg_get(NULL, "vlcd");

		printk("[TSP] %s 2 \n",__func__);
		if (IS_ERR(vreg_touch)) {
			printk("[TSP] %s: vreg_get(%s) failed (%ld)\n",__func__, "vlcd", PTR_ERR(vreg_touch));
			return;
		}

		rc = vreg_set_level(vreg_touch, 3000);
		if (rc) {
			printk("[TSP] %s: TSP set_level failed (%d)\n",__func__, rc);
		}
//	}
	printk("[TSP] %s 3 \n",__func__);

	ret = vreg_enable(vreg_touch);
	if (ret) {
		printk(KERN_ERR "[TSP] %s: vreg_touch enable failed (%d)\n", __func__, ret);
		//return -EIO;
	}
	/* VTOUCHIO_1.8V */
#if 0
	vreg_touchio = vreg_get(NULL, "vreg_msme");
	ret = vreg_set_level(vreg_touchio, 1800);
	if (ret) {
		printk(KERN_ERR "%s: VTOUCHIO_1.8V set level failed (%d)\n", __func__, ret);
	}

	ret = vreg_enable(vreg_touchio);
	if (ret) {
		printk(KERN_ERR "%s: vreg_touchio enable failed (%d)\n", __func__, ret);
		//return -EIO;
	}
	else {
		#ifndef PRODUCT_SHIP
		printk(KERN_INFO "%s: gp13_vreg_touchio enable success!\n", __func__);
		#endif
	}
#endif
  	mdelay(30); //MUST wait for 25ms after vreg_enable()
}

void mcsdl_vdd_off(void)
{
  vreg_disable(vreg_touch);
//  vreg_disable(vreg_touchio);
  mdelay(100); //MUST wait for 100ms before vreg_enable()
}

static int melfas_i2c_write(struct i2c_client* p_client, u8* data, int len)
{
	struct i2c_msg msg;
	int retry_write = 3;
	int ret = 0;
	

	msg.addr = p_client->addr;
	msg.flags = 0; /* I2C_M_WR */
	msg.len = len;
	msg.buf = data ;

	while (retry_write--)
	{
		ret = i2c_transfer(p_client->adapter, &msg, 1);
//		printk("[TSP] %s ret : %d, retry_write : %d \n", __func__, ret, retry_write);
		if (ret == 1)
			break;	// i2c success
	}
//	printk("[TSP] %s ret : %d, retry_write : %d \n", __func__, ret, retry_write);
	if(retry_write == -1)
	{
		printk("[TSP] %s set data pointer fail!\n", __func__);
		return -EIO;
	}

	return 0;
}
static int read_resolution = 0;

static int melfas_i2c_read(struct i2c_client* p_client, u8 reg, u8* data, int len)
{
	int i = 0;
	int retry_write = 3;
	int retry_read = 3;
	int ret = 0;

	struct i2c_msg msg;

	/* set start register for burst read */
	/* send separate i2c msg to give STOP signal after writing. */

	msg.addr = p_client->addr;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = &reg;

	while (retry_write--)
	{
		ret = i2c_transfer(p_client->adapter, &msg, 1);
//		printk("[TSP] %s ret : %d, retry_write : %d \n", __func__, ret, retry_write);
		if (ret == 1)
			break;  // i2c success
	}
//	printk("[TSP] %s ret : %d, retry_write : %d \n", __func__, ret, retry_write);
	if(retry_write == -1)
	{
		printk("[TSP] %s set data pointer fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}
	
	if(read_resolution) // only read resolution
	{
		udelay(100); // delay write register and read data
	}
	else
	{
		udelay(50); // delay write register and read data
	}		
	/* begin to read from the starting address */

	msg.addr = p_client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = data;

	while (retry_read--)
	{
		ret = i2c_transfer(p_client->adapter, &msg, 1);
//		printk("[TSP] %s ret : %d, retry_read : %d \n", __func__, ret, retry_read);
		if (ret == 1)
			break;	// i2c success
	}
//	printk("[TSP] %s ret : %d, retry_read : %d \n", __func__, ret, retry_read);
	if(retry_read == -1)
	{
		printk("[TSP] %s fail! reg(%x)\n", __func__, reg);
		return -EIO;
	}

	return 0;
}

static void melfas_read_version(void)
{
#if defined (CONFIG_TOUCHSCREEN_MELFAS_TS)
//	u8 buf[4] = {0,};
	u8 buf[6] = {0,};
	// HW version (0xF1), Com(0xF2),  Firmware version (0xF4) since Firmware version 0x04
	//if (0 == melfas_i2c_read(melfas_ts->client, MCSTS_MODULE_VER_REG, buf, 4))
	if (0 == melfas_i2c_read(melfas_ts->client, MCSTS_MODULE_VER_REG, buf, 6))
	{
		melfas_ts->hw_rev = buf[1];
		melfas_ts->com_type = buf[2];
		//melfas_ts->fw_ver = buf[3];
		melfas_ts->fw_ver = buf[4];	// change firmware version address  since FW 0x04
		printk("[TSP] %s :HW Ver : 0x%02x, FW Ver : 0x%02x, Com_Type : %d, Com_Type : %c\n", 
			__func__,melfas_ts->hw_rev,melfas_ts->fw_ver, melfas_ts->com_type,(char)melfas_ts->com_type);
	}
	else
	{
		melfas_ts->hw_rev = 0;
		melfas_ts->com_type = 0x00;
		melfas_ts->fw_ver = 0;
		printk("[TSP] %s : Can't find HW Ver, FW ver!\n", __func__);
	}
#else
	u8 buf[2] = {0,};
	
	if (0 == melfas_i2c_read(melfas_ts->client, MCSTS_MODULE_VER_REG, buf, 2))
	{
		melfas_ts->hw_rev = buf[0];
		melfas_ts->fw_ver = buf[1];
		printk("[TSP] %s :HW Ver : 0x%02x, FW Ver : 0x%02x\n", __func__,melfas_ts->hw_rev,melfas_ts->fw_ver);
	}
	else
	{
		melfas_ts->hw_rev = 0;
		melfas_ts->fw_ver = 0;
		printk("[TSP] %s : Can't find HW Ver, FW ver!\n", __func__);
	}
#endif
}

static void melfas_read_resolution(void)
{
	uint16_t max_x=0, max_y=0;
	u8 buf[3] = {0,};
	read_resolution = 1;

	if(0 == melfas_i2c_read(melfas_ts->client, MCSTS_RESOL_HIGH_REG, buf, 3)){
		printk("[TSP] %s, %d :buf[0] : 0x%02x, buf[1] : 0x%02x, buf[2] : 0x%02x\n", __func__,__LINE__,buf[0],buf[1],buf[2]);

		if(buf[0] == 0){
			printk("[TSP] %s, %d :buf[0] : 0x%02x, buf[1] : 0x%02x, buf[2] : 0x%02x\n", __func__,__LINE__,buf[0],buf[1],buf[2]);
			melfas_ts->info[0].max_x = 320;
			melfas_ts->info[0].max_y = 480;
		}
		else{
			printk("[TSP] %s, %d :buf[0] : 0x%02x, buf[1] : 0x%02x, buf[2] : 0x%02x\n", __func__,__LINE__,buf[0],buf[1],buf[2]);
			max_x = buf[1] | ((uint16_t)(buf[0] & 0x0f) << 8);
			max_y = buf[2] | (((uint16_t)(buf[0] & 0xf0) >> 4) << 8);
			melfas_ts->info[0].max_x = max_x;
			melfas_ts->info[0].max_y = max_y;
		}
	}
	else
	{
		printk("[TSP] %s, %d :buf[0] : 0x%02x, buf[1] : 0x%02x, buf[2] : 0x%02x\n", __func__,__LINE__,buf[0],buf[1],buf[2]);
		melfas_ts->info[0].max_x = 320;
		melfas_ts->info[0].max_y = 480;
	}
	read_resolution = 0;
	//printk("[TSP] %s :, Set as max_x: %d, max_y: %d\n", __func__,melfas_ts->info.max_x, melfas_ts->info.max_y);
}

static void melfas_firmware_download(void)
{
	printk("[TSP] %s %d \n", __func__, __LINE__);
	int ret;
	// Disable interrupt and don't use I2C Line
	disable_irq(melfas_ts->client->irq);
	gpio_tlmm_config(GPIO_CFG(GPIO_TSP_SCL,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TSP_SDA,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

#ifdef TSP_SDCARD_UPDATE
	if(sdcard_update)
	ret = mms100_ISC_download_binary_data();
	else
#endif		
	ret = mcsdl_download_binary_data();
	
	if(ret == 0x0 ) {
			enable_irq(melfas_ts->client->irq);		
			melfas_read_version();		
			if (melfas_ts->hw_rev < 0) {
				printk(KERN_ERR "[TSP] i2c_transfer failed\n");
			}

			if (melfas_ts->fw_ver < 0) {
				printk(KERN_ERR "[TSP] i2c_transfer failed\n");
			}

			#ifndef PRODUCT_SHIP
			printk("[TSP] Firmware update success! [Melfas H/W version: 0x%02x., Current F/W version: 0x%02x.]\n", melfas_ts->hw_rev, melfas_ts->fw_ver);
			#endif
	}
	else {
		printk("[TSP] Firmware update failed.. RESET!\n");		
		mcsdl_vdd_off();
		gpio_direction_output(GPIO_TSP_SCL, 0);  // TOUCH SCL DIS
		gpio_direction_output(GPIO_TSP_SDA, 0);  // TOUCH SDA DIS
		
		gpio_direction_output(GPIO_TSP_SCL, 1);  // TOUCH SCL EN
		gpio_direction_output(GPIO_TSP_SDA, 1);  // TOUCH SDA EN	
		gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);			
		mcsdl_vdd_on();
		msleep(300); 

		enable_irq(melfas_ts->client->irq);
	}
#if 0	// firmware update with schedule
//	cancel_delayed_work(&firmware_work);
#endif


#ifdef TSP_SDCARD_UPDATE
	sdcard_update = false;
#endif
}



static ssize_t registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status, mode_ctl, hw_rev, fw_ver;

	status  = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_STATUS_REG);
	if (status < 0) {
		printk(KERN_ERR "[TSP] i2c_smbus_read_byte_data failed\n");;
	}
	mode_ctl = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_MODE_CONTROL_REG);
	if (mode_ctl < 0) {
		printk(KERN_ERR "[TSP] i2c_smbus_read_byte_data failed\n");;
	}
	hw_rev = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_MODULE_VER_REG);
	if (hw_rev < 0) {
		printk(KERN_ERR "[TSP] i2c_smbus_read_byte_data failed\n");;
	}
	fw_ver = i2c_smbus_read_byte_data(melfas_ts->client, MCSTS_FIRMWARE_VER_REG);
	if (fw_ver < 0) {
		printk(KERN_ERR "[TSP] i2c_smbus_read_byte_data failed\n");;
	}

	sprintf(buf, "[TOUCH] Melfas Tsp Register Info.\n");
	sprintf(buf, "%sRegister 0x00 (status)  : 0x%08x\n", buf, status);
	sprintf(buf, "%sRegister 0x01 (mode_ctl): 0x%08x\n", buf, mode_ctl);
	sprintf(buf, "%sRegister 0x30 (hw_rev)  : 0x%08x\n", buf, hw_rev);
	sprintf(buf, "%sRegister 0x31 (fw_ver)  : 0x%08x\n", buf, fw_ver);

	return sprintf(buf, "%s", buf);
}

static ssize_t registers_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret;
	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
		ret = i2c_smbus_write_byte_data(melfas_ts->client, 0x01, 0x01);
		if (ret < 0) {
			printk(KERN_ERR "[TSP] i2c_smbus_write_byte_data failed\n");
		}
		printk("[TSP] software reset.\n");
	}
	return size;
}

static ssize_t gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	sprintf(buf, "[TOUCH] Melfas Tsp Gpio Info.\n");
	sprintf(buf, "%sGPIO TOUCH_INT : %s\n", buf, gpio_get_value(GPIO_TOUCH_INT)? "HIGH":"LOW");
	return sprintf(buf, "%s", buf);
}

static ssize_t gpio_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(strncmp(buf, "ON", 2) == 0 || strncmp(buf, "on", 2) == 0) {
		mcsdl_vdd_on();
		//gpio_set_value(GPIO_TOUCH_EN, GPIO_LEVEL_HIGH);
		printk("[TSP] enable.\n");
		mdelay(200);
	}

	if(strncmp(buf, "OFF", 3) == 0 || strncmp(buf, "off", 3) == 0) {
		mcsdl_vdd_off();
		printk("[TSP] disable.\n");
	}

	if(strncmp(buf, "RESET", 5) == 0 || strncmp(buf, "reset", 5) == 0) {
		mcsdl_vdd_off();
		mdelay(500);
		mcsdl_vdd_on();
		printk("[TSP] reset.\n");
		mdelay(200);
	}
	return size;
}

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	sprintf(buf, "H/W rev. 0x%x F/W ver. 0x%x\n", melfas_ts->hw_rev, melfas_ts->fw_ver);
	return sprintf(buf, "%s", buf);
}

static ssize_t firmware_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(strncmp(buf, "UPDATE", 6) == 0 || strncmp(buf, "update", 6) == 0) {
		melfas_firmware_download();
	}
	else
		printk("\n[TSP] Firmware update error :: Check the your devices version.\n");
#if 0 //def CONFIG_TOUCHSCREEN_MELFAS_FIRMWARE_UPDATE

	if(strncmp(buf, "UPDATE", 6) == 0 || strncmp(buf, "update", 6) == 0) {
		#if defined(CONFIG_MACH_CHIEF)
		if(system_rev >= 8){
			melfas_firmware_download();
			}
		else
			printk("\n[TSP] Firmware update error :: Check the your devices version.\n");
		#else //vital2
			if(system_rev >= 5){
		#ifdef TSP_SDCARD_UPDATE				
			sdcard_update = true;
		#endif
			melfas_firmware_download();
			}
		else
			printk("\n[TSP] Firmware update error :: Check the your devices version.\n");
		#endif
		}

#endif

	return size;
}


static ssize_t debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", debug_level);
}

static ssize_t debug_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if(buf[0]>'0' && buf[0]<='9') {
		debug_level = buf[0] - '0';
	}

	return size;
}

#ifdef TSP_TEST_MODE
void touch_screen_sleep()
{	
	melfas_ts_suspend(PMSG_SUSPEND);
}

void touch_screen_wakeup()
{
	melfas_ts_resume();
}

int touch_screen_get_tsp_info(touch_screen_info_t *tsp_info)
{
		int ret = 0;
			
		/* chipset independent */
		tsp_info->driver = touch_screen.tsp_info.driver;
		tsp_info->reference.bad_point = touch_screen.tsp_info.reference.bad_point;
		tsp_info->reference.table = touch_screen.tsp_info.reference.table;

		/* chipset dependent */
		/* melfas */
		tsp_info->inspection.bad_point = touch_screen.tsp_info.inspection.bad_point;
		tsp_info->inspection.table = touch_screen.tsp_info.inspection.table;	
		return ret;
}

//****************************************************************************
//
// Function Name:   touch_screen_ctrl_testmode
//
// Description:     
//
// Notes: 
//
//****************************************************************************
int touch_screen_ctrl_testmode(int cmd, touch_screen_testmode_info_t *test_info, int test_info_num)
{
	int ret = 0; 
	bool prev_device_state = FALSE;
	touch_screen.driver = &melfas_test_mode;
	touch_screen.tsp_info.driver = &(touch_screen.driver->ts_info);

	if ( touch_screen.device_state == FALSE )
	{
		touch_screen_wakeup();
		touch_screen.device_state = TRUE;
		msleep(100);
		prev_device_state = TRUE;
	}

	if (test_info == NULL)
		return ret;

 	switch (cmd)
 	{
		case TOUCH_SCREEN_TESTMODE_SET_REFERENCE_SPEC_LOW:	
		{
			touch_screen.tsp_info.driver->reference_spec_low = test_info->reference;
			break;
		}
		
		case TOUCH_SCREEN_TESTMODE_SET_REFERENCE_SPEC_HIGH:	
		{
			touch_screen.tsp_info.driver->reference_spec_high = test_info->reference;
			break;
		}

		case TOUCH_SCREEN_TESTMODE_SET_INSPECTION_SPEC_LOW:	
		{
			//touch_screen.tsp_info.driver->inspection_spec_low = test_info->reference;
			break;
		}
		
		case TOUCH_SCREEN_TESTMODE_SET_INSPECTION_SPEC_HIGH:	
		{
			//touch_screen.tsp_info.driver->inspection_spec_high = test_info->reference;
			break;
		}
		
		case TOUCH_SCREEN_TESTMODE_RUN_SELF_TEST:
		{
			printk(KERN_DEBUG "[TSP] STRAT TOUCH_SCREEN_TESTMODE_RUN_SELF_TEST\n") ; 	
			int i;
			uint16_t reference;
			uint16_t inspection, inspection_spec_low, inspection_spec_high;
			uint16_t* inspection_spec_table = NULL;
			uint16_t reference_table_size, inspection_table_size;
			touch_screen_testmode_info_t* reference_table = NULL;
			touch_screen_testmode_info_t* inspection_table = NULL;
			
			reference_table_size = inspection_table_size = touch_screen.tsp_info.driver->x_channel_num * touch_screen.tsp_info.driver->y_channel_num;
			/* chipset independent check item */
			/* reference */
			if ( touch_screen.tsp_info.reference.table == NULL )
			{
				 touch_screen.tsp_info.reference.table = (touch_screen_testmode_info_t*)kzalloc(sizeof(touch_screen_testmode_info_t) * reference_table_size,GFP_KERNEL);
			}
			else
			{	
				/* delete previous reference table */
				kzfree( touch_screen.tsp_info.reference.table);				
				touch_screen.tsp_info.reference.table = NULL;
				touch_screen.tsp_info.reference.table = (touch_screen_testmode_info_t*)kzalloc(sizeof(touch_screen_testmode_info_t) * reference_table_size,GFP_KERNEL);				
			}
			reference_table = touch_screen.tsp_info.reference.table;

			/* init reference info */
			test_info->bad_point = (uint16_t)0xfffe; // good sensor
			memset(reference_table, 0x00, sizeof(touch_screen_testmode_info_t) * reference_table_size);
			if (test_info != NULL)
			{
				touch_screen.driver->test_mode(TOUCH_SCREEN_TESTMODE_GET_REFERENCE, reference_table, reference_table_size);				
				for ( i = 0; i < reference_table_size; i++ )
				{
					reference = ((reference_table+i)->reference >> 8) & 0x00ff;					
					reference |= (reference_table+i)->reference << 8;				
					if (reference < touch_screen.tsp_info.driver->reference_spec_low || reference > touch_screen.tsp_info.driver->reference_spec_high )
					{
						/* bad sensor */
						touch_screen.tsp_info.reference.bad_point = test_info->bad_point = (uint16_t)i;
						printk("[TSP] BAD POINT >> %5d: %5x: %5d:\n",i,((reference_table+i)->reference),reference);
					}
				}
				if ( test_info->bad_point == 0xfffe )
				{		
					touch_screen.tsp_info.reference.bad_point = test_info->bad_point; 
					/* good sensor, we don't need to save reference table */
					//free( touch_screen.tsp_info.reference.table);
					//touch_screen.tsp_info.reference.table = NULL;
				}
			}
			else
			{
				ret = -1;
			}	

			/* chipset dependent check item */
			/* melfas : inspection */
			if ( touch_screen.tsp_info.driver->ven_id == 0x50 && test_info_num > 1 )
			{
				if ( touch_screen.tsp_info.inspection.table == NULL )
				{
					 touch_screen.tsp_info.inspection.table = (touch_screen_testmode_info_t*)kzalloc(sizeof(touch_screen_testmode_info_t) * inspection_table_size,GFP_KERNEL);
				}
				else
				{
					/* delete previous reference table */
					kzfree( touch_screen.tsp_info.inspection.table);				
					touch_screen.tsp_info.inspection.table = NULL;
					touch_screen.tsp_info.inspection.table = (touch_screen_testmode_info_t*)kzalloc(sizeof(touch_screen_testmode_info_t) * inspection_table_size,GFP_KERNEL);
				}

				inspection_table = touch_screen.tsp_info.inspection.table;
				inspection_spec_table =  touch_screen.tsp_info.driver->inspection_spec_table;

				/* init inspection info */
				(test_info+1)->bad_point = (uint16_t)0xfffe; // good sensor
				memset(inspection_table, 0x00, sizeof(touch_screen_testmode_info_t) * inspection_table_size);

				if ( test_info != NULL )
				{
					touch_screen.driver->test_mode(TOUCH_SCREEN_TESTMODE_GET_INSPECTION, inspection_table, inspection_table_size);

					for ( i = 0; i < inspection_table_size; i++ )
					{
						inspection = ((((inspection_table+i)->inspection) & 0x0f) << 8);
						inspection +=((((inspection_table+i)->inspection) >> 8) & 0xff);
						
						inspection_spec_low = *(inspection_spec_table+i*2);
						inspection_spec_high = *(inspection_spec_table+i*2+1);

						if ( inspection < inspection_spec_low  || inspection > inspection_spec_high )							
						{
							/* bad sensor */
							touch_screen.tsp_info.inspection.bad_point = (test_info+1)->bad_point = (uint16_t)i;
							printk("[TSP] BAD POINT >> %3d: %5d [%5d] %5d:\n",i,inspection_spec_low,inspection,inspection_spec_high);
						}
					}

					if ( (test_info+1)->bad_point == 0xfffe )
					{			
						touch_screen.tsp_info.inspection.bad_point = (test_info+1)->bad_point; 
						
						/* good sensor, we don't need to save inspection table */
						//free( touch_screen.tsp_info.inspection.table);
						//touch_screen.tsp_info.inspection.table = NULL;
					}
				}
				else
				{
					ret = -1;
				}
			}
			else
			{
				if ( test_info_num > 1 )
				{
					(test_info+1)->bad_point = 0xfffe;
				}
			}
	
			break;
		}
			
		default:
		{
			if (test_info != NULL)
			{
				printk(KERN_DEBUG "[TSP] DEFAULT\n");
				ret = touch_screen.driver->test_mode(cmd, test_info, test_info_num);
			}	
			else
			{
				ret = -1;
			}
			break;
		}

 	}

	if ( prev_device_state == TRUE )
	{
		touch_screen_sleep();
	}

	return ret;
}

//****************************************************************************
//
// Function Name:   ts_melfas_test_mode
//
// Description:     
//
// Notes: 
//
//****************************************************************************
static int ts_melfas_test_mode(int cmd, touch_screen_testmode_info_t *test_info, int test_info_num)
{
	int i, ret = 0;
	uint8_t buf[TS_MELFAS_SENSING_CHANNEL_NUM*2];

	switch (cmd)
	{
		case TOUCH_SCREEN_TESTMODE_ENTER:
		{	
			//melfas_test_mode.ts_mode = TOUCH_SCREEN_TESTMODE;
			break;
		}
		
		case TOUCH_SCREEN_TESTMODE_EXIT:
		{
			//melfas_test_mode.ts_state = TOUCH_SCREEN_NORMAL;
			break;			
 		}

		case TOUCH_SCREEN_TESTMODE_GET_OP_MODE:
		{
			break;			
		}

		case TOUCH_SCREEN_TESTMODE_GET_THRESHOLD:
		{	
			ret = melfas_i2c_read(melfas_ts->client, TS_MELFAS_TESTMODE_TSP_THRESHOLD_REG, buf, 1); 
			test_info->threshold = buf[0];
			break;
		}

		case TOUCH_SCREEN_TESTMODE_GET_DELTA:
		{
			printk(KERN_DEBUG "[TSP] DELTA\n");
		#if 1	
			buf[0] = TS_MELFAS_TESTMODE_CTRL_REG;
			buf[1] = 0x01;
			ret = melfas_i2c_write(melfas_ts->client, buf, 2); 	
	
			if ( ret > 0 )
			{	
				melfas_test_mode.ts_mode = TOUCH_SCREEN_TESTMODE;				
				mdelay(50);
			}
 			if ( melfas_test_mode.ts_mode == TOUCH_SCREEN_TESTMODE && 
				melfas_test_mode.ts_info.delta_point_num == test_info_num )
 			{
	 			for ( i = 0; i < TS_MELFAS_TESTMODE_MAX_INTENSITY; i++)
	 			{
	 				ret |= melfas_i2c_read(melfas_ts->client, TS_MELFAS_TESTMODE_1ST_INTENSITY_REG+i, buf, 1); 
					test_info[i].delta = buf[0];
					
					mdelay(20); // min 10ms, typical 50ms					
	 			}
 			}
			else
			{
				ret = -3;
			}
			buf[0] = TS_MELFAS_TESTMODE_CTRL_REG;
			buf[1] = 0x00;
			ret = melfas_i2c_write(melfas_ts->client, buf, 2); 	
			if ( ret > 0 )
			{	
				melfas_test_mode.ts_mode = TOUCH_SCREEN_NORMAL;				
			}
		#else
					melfas_test_mode.ts_mode = TOUCH_SCREEN_TESTMODE;	
				
				if ( melfas_test_mode.ts_mode == TOUCH_SCREEN_TESTMODE &&
					(melfas_test_mode.ts_info.x_channel_num * melfas_test_mode.ts_info.y_channel_num) == test_info_num )
				{	
					
					buf[0] = TS_MELFAS_TESTMODE_CTRL_REG;
					buf[1] = 0x02;
					ret = melfas_i2c_write(melfas_ts->client, buf, 2);
					mdelay(500);
					mutex_lock(&melfas_ts->lock);
				
					for ( i = 0; i < TS_MELFAS_EXCITING_CHANNEL_NUM; i++)
					{	
						ret |= melfas_i2c_read(melfas_ts->client,TS_MELFAS_TESTMODE_1ST_INTENSITY_REG, buf, TS_MELFAS_SENSING_CHANNEL_NUM); 
						memcpy(&(test_info[i*TS_MELFAS_SENSING_CHANNEL_NUM].delta), buf, TS_MELFAS_SENSING_CHANNEL_NUM);
				
					}
				
					buf[0] = TS_MELFAS_TESTMODE_CTRL_REG;
					buf[1] = 0x00;
					ret = melfas_i2c_write(melfas_ts->client, buf, 2); 	
					if ( ret == 0 )
					{	
						melfas_test_mode.ts_mode = TOUCH_SCREEN_NORMAL; 			
						mdelay(50);
					}	
				}
				else
				{
					ret = -3;
				}	
				
				mutex_unlock(&melfas_ts->lock);
		#endif
			break;
		}

		case TOUCH_SCREEN_TESTMODE_GET_REFERENCE:
		{				
			printk(KERN_DEBUG "[TSP] REFERENCE\n");
			melfas_test_mode.ts_mode = TOUCH_SCREEN_TESTMODE;	
		
			if ( melfas_test_mode.ts_mode == TOUCH_SCREEN_TESTMODE &&
				(melfas_test_mode.ts_info.x_channel_num * melfas_test_mode.ts_info.y_channel_num) == test_info_num )
			{	
 				buf[0] = TS_MELFAS_TESTMODE_CTRL_REG;
				buf[1] = 0x02;
				ret = melfas_i2c_write(melfas_ts->client, buf, 2);
				mdelay(500);
 				mutex_lock(&melfas_ts->lock);
	 			for ( i = 0; i < TS_MELFAS_EXCITING_CHANNEL_NUM; i++)
	 			{	
					ret |= melfas_i2c_read(melfas_ts->client, TS_MELFAS_TESTMODE_REFERENCE_DATA_START_REG, buf, TS_MELFAS_SENSING_CHANNEL_NUM*2); 
					//printk("[TSP] REFERENCE RAW DATA : [%2x%2x]\n",buf[1],buf[0]);
					memcpy(&(test_info[i*TS_MELFAS_SENSING_CHANNEL_NUM].reference), buf, TS_MELFAS_SENSING_CHANNEL_NUM*2);
 					
	 			}
 				buf[0] = TS_MELFAS_TESTMODE_CTRL_REG;
				buf[1] = 0x00;
				ret = melfas_i2c_write(melfas_ts->client, buf, 2);
 				if ( ret > 0 )
				{	
					melfas_test_mode.ts_mode = TOUCH_SCREEN_NORMAL;				
					mdelay(50);
				}	
			}
			else
			{
				ret = -3;
			}	

			mutex_unlock(&melfas_ts->lock);
 
			break;
		}

		case TOUCH_SCREEN_TESTMODE_GET_INSPECTION:
		{	
			int j;
					
				melfas_test_mode.ts_mode = TOUCH_SCREEN_TESTMODE;	
			printk(KERN_DEBUG "[TSP] INSPECTION\n");
			if ( melfas_test_mode.ts_mode == TOUCH_SCREEN_TESTMODE &&
				(melfas_test_mode.ts_info.x_channel_num * melfas_test_mode.ts_info.y_channel_num) == test_info_num )
			{	
				buf[0] = TS_MELFAS_TESTMODE_INSPECTION_DATA_CTRL_REG;
				buf[1] = 0x1A;
				buf[2] = 0x0;
				buf[3] = 0x0;
				buf[4] = 0x0;
				buf[5] = 0x01;	// start flag
				ret = melfas_i2c_write(melfas_ts->client, buf, 6);
				mdelay(1000);
 				mutex_lock(&melfas_ts->lock);

				ret |= melfas_i2c_read(melfas_ts->client, TS_MELFAS_TESTMODE_INSPECTION_DATA_READ_REG, buf, 2); // dummy read
 	 			for ( j = 0; j < TS_MELFAS_EXCITING_CHANNEL_NUM; j++)
	 			{	
		 			for ( i = 0; i < TS_MELFAS_SENSING_CHANNEL_NUM; i++)
		 			{	
		 				buf[0] = TS_MELFAS_TESTMODE_INSPECTION_DATA_CTRL_REG;
						buf[1] = 0x1A;
						buf[2] = j;		// exciting ch
						buf[3] = i;		// sensing ch
						buf[4] = 0x0;		// reserved
						buf[5] = 0x02;	// start flag, 2: output inspection data, 3: output low data
						ret = melfas_i2c_write(melfas_ts->client, buf, 6);
						ret |= melfas_i2c_read(melfas_ts->client, TS_MELFAS_TESTMODE_INSPECTION_DATA_READ_REG, buf, 2);
						//printk("[TSP] INSPECTION RAW DATA : [%2d,%2d][%02x%02x]\n",buf[2],buf[3],buf[1],buf[0]);
 						memcpy(&(test_info[i+(j*TS_MELFAS_SENSING_CHANNEL_NUM)].inspection), buf, 2);
	 				}
	 			}

				buf[0] = TS_MELFAS_TESTMODE_CTRL_REG;
				buf[1] = 0x00;
				ret = melfas_i2c_write(melfas_ts->client, buf, 2);
 				if ( ret > 0 )
				{	
					melfas_test_mode.ts_mode = TOUCH_SCREEN_NORMAL;				
					mdelay(50);
				}
				/*
				mcsdl_vdd_off();
				mcsdl_vdd_on();
 				mdelay(100);
				
				melfas_test_mode.ts_mode = TOUCH_SCREEN_NORMAL;				
				//mdelay(50);*/
			}
			else
			{
				ret = -3;
			}	
			
			mutex_unlock(&melfas_ts->lock); 			

			break;
		}

		default:
		{
			ret = -2;
			break;
		}
	}

	return ret;
}


// 15 Mode Key test
int menu_pressed = 0;			// check menu key press or not
int back_pressed = 0;			// check back key press or not
int home_pressed = 0;			// check home key press or not
uint8_t menu_sensitivity = 0;	// menu key sensitivity value
uint8_t back_sensitivity = 0;	// back key sensitivity value
uint8_t home_sensitivity = 0;	// home key sensitivity value

/*indiviual touch key senstivity implementation for menu,home and back keys*/
static ssize_t touchkey_menu_sensitivity_show
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t master_write_buf[6] = {0,};
	uint8_t menu_key_sensitivity = 0;	/* menu key sensitivity value*/
	int ret = 0;

	master_write_buf[0] = 0xB0;
	master_write_buf[1] = 0x1A;
	master_write_buf[2] = 0;
	master_write_buf[3] = 0;
	master_write_buf[4] = 0;
	master_write_buf[5] = 0x14;

	ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 6);

	melfas_i2c_read(melfas_ts->client, 0xBF, &menu_key_sensitivity, 1);
	printk(KERN_DEBUG"[TSP] %s, menu : %d\n",
			__func__, menu_key_sensitivity);
	return snprintf(buf, 3, "%d\n", menu_key_sensitivity);
}

static ssize_t touchkey_home_sensitivity_show
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t master_write_buf[6] = {0,};
	uint8_t home_key_sensitivity = 0;	/*home key sensitivity value*/
	int ret = 0;

	master_write_buf[0] = 0xB0;
	master_write_buf[1] = 0x1A;
	master_write_buf[2] = 1;
	master_write_buf[3] = 0;
	master_write_buf[4] = 0;
	master_write_buf[5] = 0x14;
	ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 6);
	melfas_i2c_read(melfas_ts->client, 0xBF, &home_key_sensitivity, 1);
	printk(KERN_DEBUG"[TSP] %s, home : %d\n",
			__func__,  home_key_sensitivity);
	return snprintf(buf, 3, "%d\n", home_key_sensitivity);
}

static ssize_t touchkey_back_sensitivity_show
	(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t master_write_buf[6] = {0,};
	uint8_t back_key_sensitivity = 0;	/* back key sensitivity value*/
	int ret = 0;

	master_write_buf[0] = 0xB0;
	master_write_buf[1] = 0x1A;
	master_write_buf[2] = 2;
	master_write_buf[3] = 0;
	master_write_buf[4] = 0;
	master_write_buf[5] = 0x14;
	ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 6);
	melfas_i2c_read(melfas_ts->client, 0xBF, &back_key_sensitivity, 1);
	printk(KERN_DEBUG"[TSP] %s, Back : %d\n",
			__func__,  back_key_sensitivity);
	return snprintf(buf, 3, "%d\n", back_key_sensitivity);
}

static ssize_t touchkey_sensitivity_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t master_write_buf[6] = {0,};
	int ret = 0;

	// enter touch key sensitivity mode : we can read touchkey sensitivity every times
	master_write_buf[0] = 0xB0;
	master_write_buf[1] = 0x1A;
	master_write_buf[2] = 0;
	master_write_buf[3] = 0;
	master_write_buf[4] = 0;
	master_write_buf[5] = 0x08;

	ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 6);

	printk("[TSP] %s, menu : %d, back : %d, home : %d \n",__func__, menu_sensitivity, back_sensitivity, home_sensitivity);
	return sprintf(buf, "%u %u %u\n", menu_sensitivity, back_sensitivity, home_sensitivity);
}

static ssize_t touchkey_normal_mode(struct device *dev, struct device_attribute *attr, char *buf)
{

	uint8_t master_write_buf[6] = {0,};
	int ret = 0;

	// enter touch key normal mode : we must enter normal mode to finish sensitivity mode
	master_write_buf[0] = 0xB0;
	master_write_buf[1] = 0x1A;
	master_write_buf[2] = 1;
	master_write_buf[3] = 0;
	master_write_buf[4] = 0;
	master_write_buf[5] = 0x08;

	ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 6);
	
	printk("[TSP] %s, menu : %d, back : %d, home : %d \n",__func__, menu_sensitivity, back_sensitivity, home_sensitivity);
	return sprintf(buf, "%u %u %u\n", menu_sensitivity, back_sensitivity, home_sensitivity);
}

static ssize_t touchkey_back_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	printk("[TSP] %s \n",__func__);
	if(back_pressed)
	{
		return sprintf(buf, "%s\n", "PRESS");
	}
	else
	{
		return sprintf(buf, "%s\n", "RELEASE");
	}	
}

static ssize_t touchkey_menu_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	printk("[TSP] %s \n",__func__);
	if(menu_pressed)
	{
		return sprintf(buf, "%s\n", "PRESS");
	}
	else
	{
		return sprintf(buf, "%s\n", "RELEASE");
	}	
}

static ssize_t touchkey_home_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	printk("[TSP] %s \n",__func__);
	if(home_pressed)
	{
		return sprintf(buf, "%s\n", "PRESS");
	}
	else
	{
		return sprintf(buf, "%s\n", "RELEASE");
	}
}

static ssize_t touchkey_firm_version_panel_show(struct device *dev, \
		struct device_attribute *attr, char *buf)
{
	int len;
	printk(KERN_DEBUG"[TSP] %s\n", __func__);
	set_default_result();
	len = snprintf(buf, 5, "0x%x\n", melfas_ts->fw_ver);
	set_cmd_result(buf, strnlen(buf, 100));
	melfas_ts->cmd_state = 2;
	return len;
}


// TSP vesion for *#2663#
static ssize_t tsp_current_firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;
	len = sprintf(buf, "0x%X\n", melfas_ts->fw_ver);
	return len;
}

static ssize_t tsp_current_hardware_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "0x%X\n", melfas_ts->hw_rev);
}

static ssize_t tsp_recommend_firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int recommend_ver = 0;
	recommend_ver = Melfas_recommend_ver();

	return sprintf(buf, "0x%x\n", recommend_ver);
}


static ssize_t tsp_firmware_update_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	sprintf(buf, "H/W rev. 0x%x F/W ver. 0x%x\n", melfas_ts->hw_rev, \
							melfas_ts->fw_ver);
	return sprintf(buf, "%s", buf);
}

static int tsp_update_status = 2;
static ssize_t tsp_firmware_update_store(struct device *dev, \
		struct device_attribute *attr, char *buf, size_t size)
{
	int retry = 4;
	/*tsp_update_status = 0;*/
	int recommend_ver = 0;
	int Compatibility_group = 0 ;
	int do_update = 1;
	// Disable interrupt and don't use I2C Line
	disable_irq(melfas_ts->client->irq);
	gpio_tlmm_config(GPIO_CFG(GPIO_TSP_SCL,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TSP_SDA,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	// if firmware is the latest, do not need to update	
	recommend_ver = Melfas_recommend_ver();
	Compatibility_group = Melfas_Compatibility_group();
	/*melfas_read_version();*/
	printk(KERN_DEBUG "\n tsp_firmware_update_store : \n from chip - [com_type = %c,Firmware ver : 0x%x] \n \
			from Binary file - [com_type = %c, Firmware ver : 0x%x]\n", \
	melfas_ts->com_type, melfas_ts->fw_ver, \
	Compatibility_group, recommend_ver);

	if((melfas_ts->com_type == Compatibility_group) || (melfas_ts->com_type == 0x00))
	{
		if(recommend_ver > melfas_ts->fw_ver)
		{
			while (--retry)
			{
				tsp_update_status = 0;
				if(mms100_ISC_download_binary_data() == 0x0)
				{
				/*	enable_irq(melfas_ts->client->irq);*/
				/*		msleep(300);*/
					melfas_read_version();
					if (melfas_ts->hw_rev < 0) {
						printk(KERN_ERR "[TSP] i2c_transfer failed\n");
					}
			
					if (melfas_ts->fw_ver < 0) {
						printk(KERN_ERR "[TSP] i2c_transfer failed\n");
					}

					printk("[TSP] Firmware update success! [Melfas H/W version: 0x%02x., Current F/W version: 0x%02x.]\n"
						, melfas_ts->hw_rev, melfas_ts->fw_ver);
					tsp_update_status = 1;
					/*enable_irq(melfas_ts->client->irq);*/
					break;
				}
				printk("[TSP] %s TSP_update failed... retry...\n",__func__);
			}
		}
		else
		{
			do_update = 0;
			printk("[TSP] Firmware is the latest. fw_ver:%x, com_type:%c \n",melfas_ts->fw_ver, melfas_ts->com_type);
		}
	}
	else
	{
		do_update = 0;
		printk("[TSP] The pannel is different. fw_ver:%x, com_type:%c \n",melfas_ts->fw_ver, melfas_ts->com_type);
	}

	// don't need to update case 1. tsp pannel is different, case 2. firmware is the latest
	if(!do_update)
	{
		mcsdl_vdd_off();
		gpio_direction_output(GPIO_TSP_SCL, 0);  // TOUCH SCL DIS
		gpio_direction_output(GPIO_TSP_SDA, 0);  // TOUCH SDA DIS
			
		gpio_direction_output(GPIO_TSP_SCL, 1);  // TOUCH SCL EN
		gpio_direction_output(GPIO_TSP_SDA, 1);  // TOUCH SDA EN	
		gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE); 		
		mcsdl_vdd_on();
		msleep(300); 
		/*enable_irq(melfas_ts->client->irq);*/
		tsp_update_status = 2;
	}

	if (retry <= 0)
	{
		printk("[TSP] Firmware update failed.. RESET!\n");		
		mcsdl_vdd_off();
		gpio_direction_output(GPIO_TSP_SCL, 0);  // TOUCH SCL DIS
		gpio_direction_output(GPIO_TSP_SDA, 0);  // TOUCH SDA DIS
			
		gpio_direction_output(GPIO_TSP_SCL, 1);  // TOUCH SCL EN
		gpio_direction_output(GPIO_TSP_SDA, 1);  // TOUCH SDA EN	
		gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE); 		
		mcsdl_vdd_on();
		msleep(300); 
		/*enable_irq(melfas_ts->client->irq);*/
		tsp_update_status = -1;
	}
	enable_irq(melfas_ts->client->irq);
	return sprintf(buf, "%d\n", tsp_update_status);
}

static ssize_t tsp_firmware_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;

	printk(KERN_DEBUG
		   "[TouchKey] touch_update_read: touchkey_update_status %d\n", tsp_update_status);

	if (tsp_update_status == 1) {
		count = sprintf(buf, "PASS\n");
	} else if (tsp_update_status == 0) {
		count = sprintf(buf, "Downloading\n");
	} else if (tsp_update_status == -1) {
		count = sprintf(buf, "Fail\n");
	} else if (tsp_update_status == 2) {
		count = sprintf(buf, "No update\n");
	}

	return count;
}

// TSP Command Test Binary
static ssize_t tsp_xy_node_number_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	printk("[TSP] %s \n",__func__);

	u8 xy_node[2] = {0,};
	int ret = 0;
	int xy_node_addr = 0xed;	// firmware change
		
	ret = melfas_i2c_read(melfas_ts->client, xy_node_addr, xy_node, 2);
	printk("[TSP] %s, x_node : %d, y_node : %d  \n", __func__, xy_node[0], xy_node[1] );

	return sprintf(buf, "%d, %d\n", xy_node[0], xy_node[1]);
	
}

static ssize_t tsp_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Melfas, MMS136\n");
}

static ssize_t tsp_module_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = melfas_ts_resume();
	return ret;
}

static ssize_t tsp_module_off_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = melfas_ts_suspend(PMSG_SUSPEND);
	return ret;
}

static ssize_t tsp_intensity_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, j;
	int16_t cmdata[NUMBER_X_NODE][NUMBER_Y_NODE] = {{0,},};

	uint8_t master_write_buf[6] = {0,};
	uint8_t master_read_buf_array[30] = {0,};
	int ret = 0;
	int addr = 0xBF;
	int written_bytes = 0 ;  /* & error check */
		
	// Intensity Read
	for (i=0; i<NUMBER_X_NODE; i++)
	{
		j = 0;
		{
			master_write_buf[0] = 0xB0;
			master_write_buf[1] = 0x1A;
			master_write_buf[2] = j; //Exciting CH.
			master_write_buf[3] = i; //Sensing CH.
			master_write_buf[4] = 0; //Reserved
			master_write_buf[5] = 0x04;//Flag
			ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 6);

			udelay(500);
			ret = melfas_i2c_read(melfas_ts->client, addr, master_read_buf_array, (15 + 15 % 2));
			for (j = 0; j < NUMBER_Y_NODE; j++)
			{
				cmdata[i][j] = (int8_t)master_read_buf_array[j];
				printk("[TSP] i=%d %5d ", i, cmdata[i][j]);//log_printf(0, "%5d\t", cmdata);
			}
		}
		printk("[TSP] \n");//log_printf(0, "\n");
	}

	master_write_buf[0] = 0xB0;
	master_write_buf[1] = 0x1A;
	master_write_buf[2] = 0; //Exciting CH.
	master_write_buf[3] = 0; //Sensing CH.
	master_write_buf[4] = 0; //Reserved
	master_write_buf[5] = 0x15; //Start Flag
	ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 6);

	for (i = 0; i < NUMBER_X_NODE ; i++)
	{
		for(j = 0 ; j < NUMBER_Y_NODE ; j++)
		{
			written_bytes += sprintf(buf+written_bytes, "%5d", cmdata[i][j]);
		}
		written_bytes += sprintf(buf+written_bytes, "\n");
	}

	if (written_bytes > 0)
		return written_bytes ;

	return sprintf(buf, "-1") ;
}

static ssize_t tsp_key_intensity_read_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	int i, j;
	int16_t cmdata[3] ;
	uint8_t master_write_buf[6] = {0,};
	uint8_t master_read_buf_array[50] = {0,};
	int ret = 0;
	int addr = 0xBF;
	int written_bytes = 0 ;  /* & error check */
	int total_data = 0;
	int count = 0;
	for (i = 0; i < UCKEY; i++) {
		master_write_buf[0] = 0xB0;
		master_write_buf[1] = 0x1A;
		master_write_buf[2] = i;
		master_write_buf[3] = 0; /*Dummy Info*/
		master_write_buf[4] = 0; /*Reserved*/
		master_write_buf[5] = 0x14;
		ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 6);
		ret = melfas_i2c_read(melfas_ts->client,
				 0xBF, master_read_buf_array, 1);
		cmdata[total_data] = master_read_buf_array[0];
		printk(KERN_DEBUG"[TSP] %5d\n", cmdata[total_data]);
		if ((cmdata[total_data] >= SCR_KEY_INTENSITY_LOWER_SPEC[i])
			&& (cmdata[i] <= SCR_KEY_INTENSITY_UPPER_SPEC[i])) {
			count++;
		}
		if (count == 0)
			status_intensity = 1;/*fail*/
		total_data++;
	}
	for (i = 0; i < total_data ; i++)
		written_bytes += sprintf(buf+written_bytes, "%5d", cmdata[i]);

	if (written_bytes > 0)
		return written_bytes ;

	return sprintf(buf, "-1") ;
}

static ssize_t tsp_key_intensity_read_result(struct device *dev,
			struct device_attribute *attr, char *buf, size_t size)
{
	return sprintf(buf, "%u\n", status_intensity);
}
static ssize_t tsp_reference_read_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	int i, j;
	int16_t cmdata[NUMBER_X_NODE][NUMBER_Y_NODE] = {{0,},};
	uint8_t master_write_buf[6] = {0,};
	uint8_t master_read_buf_array[50] = {0,};
	int ret = 0;
	int addr = 0xBF;
	int written_bytes = 0 ;  /* & error check */
		
	for (i=0; i<11; i++)
	{
		j = 0;
		{
			master_write_buf[0] = 0xB0;
			master_write_buf[1] = 0x1A;
			master_write_buf[2] = j; //Exciting CH.
			master_write_buf[3] = i; //Sensing CH.
			master_write_buf[4] = 0; //Reserved
			master_write_buf[5] = 0x06;//Flag

			ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 6);
			udelay(500);
			ret = melfas_i2c_read(melfas_ts->client, addr, master_read_buf_array, (15 * 2));

			for (j = 0; j < 15; j++)
			{
				cmdata[i][j] = (master_read_buf_array[2 * j] | (master_read_buf_array[2 * j + 1] << 8));
				printk("[TSP] i=%d,%5d",i, cmdata[i][j]);//log_printf(0, "%5d\t", cmdata);
			}
		}
		printk("[TSP] \n");//log_printf(0, "\n");
	}

	master_write_buf[0] = 0xB0;
	master_write_buf[1] = 0x1A;
	master_write_buf[2] = 0; //Exciting CH.
	master_write_buf[3] = 0; //Sensing CH.
	master_write_buf[4] = 0; //Reserved
	master_write_buf[5] = 0x15; //Start Flag
	ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 6);

	for (i = 0; i < NUMBER_X_NODE ; i++)
	{
		for(j = 0 ; j < NUMBER_Y_NODE ; j++)
		{
			written_bytes += sprintf(buf+written_bytes, "%5d", (cmdata[i][j])/8);
		}
		written_bytes += sprintf(buf+written_bytes, "\n");
	}

	if (written_bytes > 0)
		return written_bytes ;

	return sprintf(buf, "-1") ;
}
static ssize_t tsp_cmdelta_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t Tx, Rx, KeyNum;
	int r, t, i = 0, total_data = 0;
	status_delta = 0;
	int16_t cmdata[200];
	uint8_t master_write_buf[6] = {0,};
	uint8_t master_read_buf_array[50] = {0,};
	int ret = 0;
	int written_bytes = 0 ;  /* & error check */
	
	disable_irq(melfas_ts->client->irq);

	ret = melfas_i2c_read(melfas_ts->client, MIP_SMD_TX_CH_NUM, master_read_buf_array, 3);
	Tx = master_read_buf_array[0];
	Rx = master_read_buf_array[1];
	KeyNum = master_read_buf_array[2];
	
	printk("[TSP] %s Enter Jig Test Mode!, Tx : %d, Rx : %d\n", __func__, Tx, Rx);
	
	master_write_buf[0] = 0xA0;
	master_write_buf[1] = 0x40;
	ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 2);

	while (gpio_get_value(GPIO_TOUCH_INT))
	{	
		udelay(100);
	}

	ret = melfas_i2c_read(melfas_ts->client, 0xAE, master_read_buf_array, 1);
	
	printk("[TSP] --- CM_DELTA --- \n");
	
	master_write_buf[0] = 0xA0;
	master_write_buf[1] = 0x41;
	ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 2);

	while (gpio_get_value(GPIO_TOUCH_INT))
	{
		udelay(100);
	}

	ret = melfas_i2c_read(melfas_ts->client, 0xAE, master_read_buf_array, 1);
	
	for (r = 0; r < Rx; r++) //Model Dependent
	{
		for (t = 0; t < Tx; t++) //Model Dependent
		{
			master_write_buf[0] = 0xA0;
			master_write_buf[1] = 0x42;
			master_write_buf[2] = t; //Exciting CH.
			master_write_buf[3] = r; //Sensing CH.
			ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 4);

			while (gpio_get_value(GPIO_TOUCH_INT))
			{
				udelay(100);
			}
			
			ret = melfas_i2c_read(melfas_ts->client, 0xAE, master_read_buf_array, 1);

			ret = melfas_i2c_read(melfas_ts->client, 0xAF, master_read_buf_array, master_read_buf_array[0]);

			cmdata[total_data] = (int16_t) (master_read_buf_array[0] | (master_read_buf_array[1] << 8));
			printk("[TSP] %5d\n", cmdata[total_data]);
			/*status_abs will be set to fail
				 if it is not in the limit */
			if ((cmdata[total_data] <= SCR_DELTA_LOWER_SPEC[r][t])
			|| (cmdata[total_data] >= SCR_DELTA_UPPER_SPEC[r][t]))
					status_delta = 1; /* fail */
			total_data++;
		}
		printk(KERN_DEBUG "[TSP]\n");/*log_printf(0, "\n");*/
	}
	
	if (UCKEY)
	{
		for (t = 0; t < UCKEY; t++) //Model Dependent
		{
			master_write_buf[0] = 0xA0;
			master_write_buf[1] = 0x4A;
			master_write_buf[2] = t; //KEY CH.
			master_write_buf[3] = 0; //Dummy Info
			ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 4);

			while (gpio_get_value(GPIO_TOUCH_INT))
			{
				udelay(100);
			}
			ret = melfas_i2c_read(melfas_ts->client, 0xAE, master_read_buf_array, 1);

			ret = melfas_i2c_read(melfas_ts->client, 0xAF, master_read_buf_array, master_read_buf_array[0]);

			cmdata[total_data] = (int16_t) (master_read_buf_array[0] | (master_read_buf_array[1] << 8));
			printk("[TSP] %5d\n", cmdata[total_data]);
			/*status_abs will be set to fail
				 if it is not in the limit */
			if ((cmdata[total_data] <= SCR_KEY_DELTA_LOWER_SPEC[t])
			|| (cmdata[total_data] >= SCR_KEY_DELTA_UPPER_SPEC[t]))
					status_delta = 1; /* fail */
			total_data++;
		}
		printk(KERN_DEBUG"[TSP]\n");/*log_printf(0, "\n");*/
	}

	enable_irq(melfas_ts->client->irq);
	//tsp_reset();
	// exit cmdelta mode
	master_write_buf[0] = 0xA0;
	master_write_buf[1] = 0x4F;
	ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 2);

	for (i = 0; i < total_data ; i++)
	{
		written_bytes += sprintf(buf+written_bytes, "%5d", cmdata[i]);
	}

	if (written_bytes > 0)
		return written_bytes ;

	return sprintf(buf, "-1") ;
}

static ssize_t tsp_cmdelta_read_result(struct device *dev,
		 struct device_attribute *attr, char *buf, size_t size)
{
	return sprintf(buf, "%u\n", status_delta);
}
static ssize_t tsp_cmabs_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t Tx, Rx, keyNum;
	int r, t, i = 0, total_data = 0;
	status_abs = 0;/* initializing everytime */
	int16_t cmdata[200];
	uint8_t master_write_buf[6] = {0,};
	uint8_t master_read_buf_array[50] = {0,};
	int ret = 0;
	int written_bytes = 0 ;  /* & error check */
	disable_irq(melfas_ts->client->irq);
	ret = melfas_i2c_read(melfas_ts->client, MIP_SMD_TX_CH_NUM, master_read_buf_array, 3);

	Tx = master_read_buf_array[0];
	Rx = master_read_buf_array[1];
	keyNum = master_read_buf_array[2];
		
	printk("[TSP] %s Enter Jig Test Mode!\n", __func__);
		
	master_write_buf[0] = 0xA0;
	master_write_buf[1] = 0x40;
	ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 2);
		
	while (gpio_get_value(GPIO_TOUCH_INT))
	{
		udelay(100);
	}
	ret = melfas_i2c_read(melfas_ts->client, 0xAE, master_read_buf_array, 1);

	printk("[TSP] --- CM_ABS --- \n");
	
	master_write_buf[0] = 0xA0;
	master_write_buf[1] = 0x43;
	ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 2);
	
	while (gpio_get_value(GPIO_TOUCH_INT))
	{
		udelay(100);
	}
	ret = melfas_i2c_read(melfas_ts->client, 0xAE, master_read_buf_array, 1);
	
	for (r = 0; r < Rx; r++) //Model Dependent
	{
	
		for (t = 0; t < Tx; t++) //Model Dependent
		{
			master_write_buf[0] = 0xA0;
			master_write_buf[1] = 0x44;
			master_write_buf[2] = t; //Exciting CH.
			master_write_buf[3] = r; //Sensing CH.
			ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 4);

			while (gpio_get_value(GPIO_TOUCH_INT))
			{
				udelay(100);
			}
			ret = melfas_i2c_read(melfas_ts->client, 0xAE, master_read_buf_array, 1);

			ret = melfas_i2c_read(melfas_ts->client, 0xAF, master_read_buf_array, master_read_buf_array[0]);
	
			cmdata[total_data] = (int16_t) (master_read_buf_array[0] | (master_read_buf_array[1] << 8));
			printk("[TSP] %5d\n,", cmdata[total_data]);
			printk("[TSP ABS] %5d %5d\n,", SCR_ABS_LOWER_SPEC[r][t],
						 SCR_ABS_UPPER_SPEC[r][t]);
			/*status_abs will be set to fail
				 if it is not in the limit */
			if ((cmdata[total_data] <= SCR_ABS_LOWER_SPEC[r][t])
			|| (cmdata[total_data] >= SCR_ABS_UPPER_SPEC[r][t])) {
					status_abs = 1; /* fail */
					printk("[TSP] r = %5d t =%5d\n,", r, t);
					printk("[TSP ABS] FAIL\n,");
				}
			total_data++;
		}
		printk(KERN_DEBUG "[TSP]\n");/*log_printf(0, "\n");*/
	}
	
	if (UCKEY)
	{
		for (t = 0; t < UCKEY; t++) //Model Dependent
		{
			master_write_buf[0] = 0xA0;
			master_write_buf[1] = 0x4B;
			master_write_buf[2] = t; //KEY CH.
			master_write_buf[3] = 0; //Dummy Info
			ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 4);

			while (gpio_get_value(GPIO_TOUCH_INT))
			{
				udelay(100);
			}
			ret = melfas_i2c_read(melfas_ts->client, 0xAE, master_read_buf_array, 1);			

			ret = melfas_i2c_read(melfas_ts->client, 0xAF, master_read_buf_array, master_read_buf_array[0]);			

			cmdata[total_data] = (int16_t) (master_read_buf_array[0] | (master_read_buf_array[1] << 8));
			printk("[TSP] %5d,", cmdata[total_data]);
			/*status_abs will be set to fail
				 if it is not in the limit */
			if ((cmdata[total_data] <= SCR_KEY_ABS_LOWER_SPEC[t])
			|| (cmdata[total_data] >= SCR_KEY_ABS_UPPER_SPEC[t]))
					status_abs = 1; /* fail */
			total_data++;
		}
		printk(KERN_DEBUG "[TSP]\n");/*log_printf(0, "\n");*/
	}
	
	enable_irq(melfas_ts->client->irq);
	//tsp_reset();
	// exit cmabs mode
	master_write_buf[0] = 0xA0;
	master_write_buf[1] = 0x4F;
	ret = melfas_i2c_write(melfas_ts->client, master_write_buf, 2);
		
	for (i = 0; i < total_data ; i++)
	{
		written_bytes += sprintf(buf+written_bytes, "%5d\n", cmdata[i]);
	}

	if (written_bytes > 0)
		return written_bytes ;

	return sprintf(buf, "-1") ;
}

static ssize_t tsp_cmabs_read_result(struct device *dev,
		 struct device_attribute *attr, char *buf, size_t size)
{
	return sprintf(buf, "%u\n", status_abs);
}

static ssize_t tsp_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	int i ;
	for (i=0 ; i<11 ; i++){
		printk("[TSP] %d,", tsp_test_temp[i]);
		}
	return sprintf(buf, "%5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d\n", 
		tsp_test_temp[0],tsp_test_temp[1],tsp_test_temp[2],tsp_test_temp[3],
		tsp_test_temp[4],tsp_test_temp[5],tsp_test_temp[6],tsp_test_temp[7],
		tsp_test_temp[8],tsp_test_temp[9],tsp_test_temp[10]);
}

static ssize_t tsp_test_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
	touch_screen.driver = &melfas_test_mode;
	touch_screen.tsp_info.driver = &(touch_screen.driver->ts_info);
	if(strncmp(buf, "self", 4) == 0) {
        /* disable TSP_IRQ */		
		printk(KERN_DEBUG "[TSP] START %s\n", __func__) ;
        disable_irq(melfas_ts->client->irq);
		touch_screen_info_t tsp_info = {0};
		touch_screen_get_tsp_info(&tsp_info);
		
		uint16_t reference_table_size;
		touch_screen_testmode_info_t* reference_table = NULL;
		reference_table_size = tsp_info.driver->x_channel_num * tsp_info.driver->y_channel_num;
		reference_table = (touch_screen_testmode_info_t*)kzalloc(sizeof(touch_screen_testmode_info_t) * reference_table_size,GFP_KERNEL);
		touch_screen_ctrl_testmode(TOUCH_SCREEN_TESTMODE_RUN_SELF_TEST, reference_table, reference_table_size);	

		mcsdl_vdd_off();
		mdelay(500);
		mcsdl_vdd_on();
		printk("[TSP] reset.\n");
		mdelay(200);

        /* enable TSP_IRQ */
		enable_irq(melfas_ts->client->irq);
		}

	else {
		debugprintk(5, "[TSP] TSP Error Unknwon commad!!!\n");
		}

	return size ;
}
static ssize_t tsp_test_reference_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, j ;
	uint8_t k;
	
	for ( j = 0; j < refer_y_channel_num ; j++ ) {
			for (i=0 ; i<11 ; i++) {
				printk("[TSP] %5d ", tsp_test_reference[j][i]);
				}
				printk("\n");
			}
	k = refer_y_channel_num-1;
	return sprintf(buf, "%5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d\n", 
		tsp_test_reference[k][0],tsp_test_reference[k][1],tsp_test_reference[k][2],tsp_test_reference[k][3],
		tsp_test_reference[k][4],tsp_test_reference[k][5],tsp_test_reference[k][6],tsp_test_reference[k][7],
		tsp_test_reference[k][8],tsp_test_reference[k][9],tsp_test_reference[k][10]);

}
static void tsp_test_reference_func(uint8_t y_num)
{
	int i, j;
	touch_screen.driver = &melfas_test_mode;
	touch_screen.tsp_info.driver = &(touch_screen.driver->ts_info);
	refer_y_channel_num = y_num;
			/* disable TSP_IRQ */		
			printk(KERN_DEBUG "[TSP] Reference START %s\n", __func__) ;
			disable_irq(melfas_ts->client->irq);
			touch_screen_info_t tsp_info = {0};
			touch_screen_get_tsp_info(&tsp_info);
			
			uint16_t reference[11];
			uint16_t reference_table_size;
			touch_screen_testmode_info_t* reference_table = NULL;
			reference_table_size = tsp_info.driver->x_channel_num * tsp_info.driver->y_channel_num;
			reference_table = (touch_screen_testmode_info_t*)kzalloc(sizeof(touch_screen_testmode_info_t) * reference_table_size,GFP_KERNEL);
			touch_screen_ctrl_testmode(TOUCH_SCREEN_TESTMODE_GET_REFERENCE, reference_table, reference_table_size); 

			for ( j = 0; j < y_num ; j++ ) {
				for ( i = 0; i < tsp_info.driver->x_channel_num; i++ ) {
					reference[i] = ((reference_table+i+(j*8))->reference >> 8) & 0x00ff;
					reference[i] |= (reference_table+i+(j*8))->reference << 8;
					tsp_test_reference[j][i] = reference[i];
					}
				}
			
			mcsdl_vdd_off();
			mdelay(500);
			mcsdl_vdd_on();
			printk("[TSP] reset.\n");
			mdelay(200);
	
			/* enable TSP_IRQ */
			enable_irq(melfas_ts->client->irq);

}
static ssize_t tsp_test_reference_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
	if(strncmp(buf, "01", 2) == 0) 	
		tsp_test_reference_func(1);			
	else if(strncmp(buf, "02", 2) == 0)		
		tsp_test_reference_func(2);
 	else if(strncmp(buf, "03", 2) == 0)		
		tsp_test_reference_func(3);
 	else if(strncmp(buf, "04", 2) == 0)		
		tsp_test_reference_func(4);
 	else if(strncmp(buf, "05", 2) == 0)		
		tsp_test_reference_func(5);
 	else if(strncmp(buf, "06", 2) == 0)		
		tsp_test_reference_func(6);
	else if(strncmp(buf, "07", 2) == 0)		
		tsp_test_reference_func(7);
	else if(strncmp(buf, "08", 2) == 0)		
		tsp_test_reference_func(8);
	else if(strncmp(buf, "09", 2) == 0)		
		tsp_test_reference_func(9);
	else if(strncmp(buf, "10", 2) == 0)		
		tsp_test_reference_func(10);
	else if(strncmp(buf, "11", 2) == 0)		
		tsp_test_reference_func(11);
	else if(strncmp(buf, "12", 2) == 0)		
		tsp_test_reference_func(12);
	else if(strncmp(buf, "13", 2) == 0)		
		tsp_test_reference_func(13);
	else if(strncmp(buf, "14", 2) == 0)		
		tsp_test_reference_func(14);
	else if(strncmp(buf, "15", 2) == 0)		
		tsp_test_reference_func(15);
	else 
		debugprintk(5, "[TSP] TSP Error Unknwon commad!!!\n");
		
	return size ;
}

static ssize_t tsp_test_inspection_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, j;
	uint8_t k;
	
	for ( j = 0; j < inspec_y_channel_num ; j++ ) {
			for (i=0 ; i < 11 ; i++) {
				printk("[TSP] %5d ", tsp_test_inspection[j][i]);
				}
				printk("\n");
			}
	k = inspec_y_channel_num-1;
	return sprintf(buf, "%5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d\n", 
		tsp_test_inspection[k][0],tsp_test_inspection[k][1],tsp_test_inspection[k][2],tsp_test_inspection[k][3],
		tsp_test_inspection[k][4],tsp_test_inspection[k][5],tsp_test_inspection[k][6],tsp_test_inspection[k][7],
		tsp_test_inspection[k][8],tsp_test_inspection[k][9],tsp_test_inspection[k][10]);
	
}
static void tsp_test_inspection_func(uint8_t y_num)
{
	int i, j;
	touch_screen.driver = &melfas_test_mode;
	touch_screen.tsp_info.driver = &(touch_screen.driver->ts_info);	
	inspec_y_channel_num = y_num;
			/* disable TSP_IRQ */		
			printk(KERN_DEBUG "[TSP] Inspection START %s\n", __func__) ;
			disable_irq(melfas_ts->client->irq);
			touch_screen_info_t tsp_info = {0};
			touch_screen_get_tsp_info(&tsp_info);

			uint16_t inspection[11];
			uint16_t inspection_table_size;
			touch_screen_testmode_info_t* inspection_table = NULL;
									
			inspection_table_size = tsp_info.driver->x_channel_num * tsp_info.driver->y_channel_num;			
			inspection_table = (touch_screen_testmode_info_t*)kzalloc(sizeof(touch_screen_testmode_info_t) * inspection_table_size,GFP_KERNEL);
			touch_screen_ctrl_testmode(TOUCH_SCREEN_TESTMODE_GET_INSPECTION, inspection_table, inspection_table_size); 
			
			for ( j = 0; j < y_num; j++ )
			{
				for ( i = 0; i < tsp_info.driver->x_channel_num; i++ )
				{
					inspection[i] = (((inspection_table+i+(j*8))->inspection) & 0x0f) << 8; 				
					inspection[i] += ((((inspection_table+i+(j*8))->inspection) >> 8) & 0xff) ;
					tsp_test_inspection[j][i] = inspection[i];
				}
			}
			
			mcsdl_vdd_off();
			mdelay(500);
			mcsdl_vdd_on();
			printk("[TSP] reset.\n");
			mdelay(200);
	
			/* enable TSP_IRQ */
			enable_irq(melfas_ts->client->irq);

}
static ssize_t tsp_test_inspection_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
	{
		if(strncmp(buf, "01", 2) == 0)	
			tsp_test_inspection_func(1); 		
		else if(strncmp(buf, "02", 2) == 0)		
			tsp_test_inspection_func(2);
		else if(strncmp(buf, "03", 2) == 0)		
			tsp_test_inspection_func(3);
		else if(strncmp(buf, "04", 2) == 0)		
			tsp_test_inspection_func(4);
		else if(strncmp(buf, "05", 2) == 0)		
			tsp_test_inspection_func(5);
		else if(strncmp(buf, "06", 2) == 0)		
			tsp_test_inspection_func(6);
		else if(strncmp(buf, "07", 2) == 0)		
			tsp_test_inspection_func(7);
		else if(strncmp(buf, "08", 2) == 0)		
			tsp_test_inspection_func(8);
		else if(strncmp(buf, "09", 2) == 0)		
			tsp_test_inspection_func(9);
		else if(strncmp(buf, "10", 2) == 0) 	
			tsp_test_inspection_func(10);
		else if(strncmp(buf, "11", 2) == 0) 	
			tsp_test_inspection_func(11);
		else if(strncmp(buf, "12", 2) == 0) 	
			tsp_test_inspection_func(12);
		else if(strncmp(buf, "13", 2) == 0) 	
			tsp_test_inspection_func(13);
		else if(strncmp(buf, "14", 2) == 0) 	
			tsp_test_inspection_func(14);
		else if(strncmp(buf, "15", 2) == 0) 	
			tsp_test_inspection_func(15);
		else 
			debugprintk(5, "[TSP] TSP Error Unknwon commad!!!\n");
			
		return size ;
	}


static ssize_t tsp_test_delta_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%5d, %5d, %5d, %5d, %5d\n", 
		tsp_test_delta[0],tsp_test_delta[1],tsp_test_delta[2],tsp_test_delta[3],tsp_test_delta[4]);

}
static ssize_t tsp_test_delta_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
	int i ;
	touch_screen.driver = &melfas_test_mode;
	touch_screen.tsp_info.driver = &(touch_screen.driver->ts_info);
	if(strncmp(buf, "delta", 5) == 0) {
			/* disable TSP_IRQ */		
			printk(KERN_DEBUG "[TSP] Delta START %s\n", __func__) ;
			disable_irq(melfas_ts->client->irq);
			touch_screen_info_t tsp_info = {0};
			touch_screen_get_tsp_info(&tsp_info);
			
			touch_screen_testmode_info_t tsp_test_info[5];
			touch_screen_ctrl_testmode(TOUCH_SCREEN_TESTMODE_GET_DELTA, tsp_test_info, 5);

			for (i=0 ; i<5 ; i++){
					tsp_test_delta[i] = tsp_test_info[i].delta;
					}
			
			mcsdl_vdd_off();
			mdelay(500);
			mcsdl_vdd_on();
			printk("[TSP] reset.\n");
			mdelay(200);
	
			/* enable TSP_IRQ */
			enable_irq(melfas_ts->client->irq);
		}
	return size ;
}
static ssize_t config_tsp_version_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{

	/* FW version 07 released on 3rd July*/
	if (0x07 == melfas_ts->fw_ver)
		return sprintf(buf, "0703\n");
	else
		return sprintf(buf, "0000\n");


}

static ssize_t tsp_test_sleep_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(sleep_state)
		sprintf(buf, "sleep\n");
	else
		sprintf(buf, "wakeup\n");

	return sprintf(buf, "%s", buf);
}

static ssize_t tsp_test_sleep_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
	if(strncmp(buf, "sleep", 5) == 0) {
		touch_screen_sleep();
		sleep_state = true;
		}
    else {
		debugprintk(5, "[TSP] TSP Error Unknwon commad!!!\n");
		}

    return size ;
}

static ssize_t tsp_test_wakeup_store(struct device *dev, struct device_attribute *attr, char *buf, size_t size)
{
	if(strncmp(buf, "wakeup", 6) == 0) {
		touch_screen_wakeup();
		sleep_state = false;
		}
    else {
		debugprintk(5, "[TSP] TSP Error Unknwon commad!!!\n");
		}

    return size ;
}

static ssize_t show_threshold(struct device *dev, \
		struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	u8 threshold;
	if (melfas_ts->suspended)
	return 0;
melfas_i2c_read(melfas_ts->client, TS_MELFAS_TESTMODE_TSP_THRESHOLD_REG,\
			&threshold, 1);
	printk(KERN_DEBUG "\n [TSP] Threshold is = 0x%x\n", threshold);
	return sprintf(buf, "%d\n", threshold);

}

#endif

static DEVICE_ATTR(gpio, S_IRUGO | S_IWUSR, gpio_show, gpio_store);
static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR, registers_show, registers_store);
static DEVICE_ATTR(firmware, S_IRUGO | S_IWUSR | S_IWGRP, firmware_show, firmware_store);
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, debug_show, debug_store);

#ifdef TSP_TEST_MODE
// 15 mode KEY test
static DEVICE_ATTR(tkey_enter_sensitivity, S_IRUGO | S_IWUSR | S_IWGRP, touchkey_sensitivity_mode, NULL);
static DEVICE_ATTR(tkey_enter_normal, S_IRUGO | S_IWUSR | S_IWGRP, touchkey_normal_mode, NULL);
static DEVICE_ATTR(touchkey_menu, S_IRUGO | S_IWUSR | S_IWGRP,
		touchkey_menu_sensitivity_show, NULL);
static DEVICE_ATTR(touchkey_home, S_IRUGO | S_IWUSR | S_IWGRP,
		touchkey_home_sensitivity_show, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO | S_IWUSR | S_IWGRP,
		touchkey_back_sensitivity_show, NULL);
static DEVICE_ATTR(tkey_back_press, S_IRUGO | S_IWUSR | S_IWGRP, \
			 touchkey_back_show, NULL);
static DEVICE_ATTR(tkey_menu_press, S_IRUGO | S_IWUSR | S_IWGRP, touchkey_menu_show, NULL);
static DEVICE_ATTR(tkey_home_press, S_IRUGO | S_IWUSR | S_IWGRP, touchkey_home_show, NULL);
static DEVICE_ATTR(touchkey_firm_version_panel, S_IRUGO | S_IWUSR | S_IWGRP, \
			touchkey_firm_version_panel_show, NULL);

// TSP vesion for *#2663#
static DEVICE_ATTR(tsp_firm_version_phone, S_IRUGO | S_IWUSR | S_IWGRP,
		tsp_current_firmware_show, NULL);/*TSP chip F/W version
		shown as Part's TSP FW version ( 2nd option) of *#2663# UI*/
static DEVICE_ATTR(tsp_firm_version_panel, S_IRUGO | S_IWUSR | S_IWGRP,
		tsp_recommend_firmware_show, NULL);/*TSP Main SW verison
		shown as Phones TSP FW version(1st option) of *#2663# UI */
static DEVICE_ATTR(tsp_firm_version_recommend, S_IRUGO | S_IWUSR | S_IWGRP, tsp_recommend_firmware_show, NULL);
static DEVICE_ATTR(tsp_firm_update, 0664, tsp_firmware_update_show, tsp_firmware_update_store);
static DEVICE_ATTR(tsp_firm_update_status, S_IRUGO | S_IWUSR | S_IWGRP, tsp_firmware_status_show, NULL);
static DEVICE_ATTR(tsp_threshold, S_IRUGO | S_IWUSR | S_IWGRP, show_threshold, NULL);

// TSP Command Test Binary
static DEVICE_ATTR(tsp_name, S_IRUGO | S_IWUSR | S_IWGRP, tsp_name_show, NULL);
static DEVICE_ATTR(tsp_xy_node, S_IRUGO | S_IWUSR | S_IWGRP, tsp_xy_node_number_show, NULL);
static DEVICE_ATTR(tsp_module_on, S_IRUGO | S_IWUSR | S_IWGRP, tsp_module_on_show, NULL);
static DEVICE_ATTR(tsp_module_off, S_IRUGO | S_IWUSR | S_IWGRP, tsp_module_off_show, NULL);
static DEVICE_ATTR(tsp_intensity_read, S_IRUGO | S_IWUSR | S_IWGRP, tsp_intensity_read_show, NULL);
static DEVICE_ATTR(tsp_reference_read, S_IRUGO | S_IWUSR | S_IWGRP, tsp_reference_read_show, NULL);
static DEVICE_ATTR(tsp_cmdelta_read, S_IRUGO | S_IWUSR | S_IWGRP, tsp_cmdelta_read_show, NULL);
static DEVICE_ATTR(tsp_cmdelta_read_result, S_IRUGO | S_IWUSR | S_IWGRP,
			 tsp_cmdelta_read_result, NULL);
static DEVICE_ATTR(tsp_cmabs_read, S_IRUGO | S_IWUSR | S_IWGRP, tsp_cmabs_read_show, NULL);
/*here I am just displaying 0/1 if the values
	read through read_show are correct */
static DEVICE_ATTR(tsp_cmabs_read_result, S_IRUGO | S_IWUSR | S_IWGRP,
			 tsp_cmabs_read_result, NULL);
static DEVICE_ATTR(tsp_key_intensity_read, S_IRUGO | S_IWUSR | S_IWGRP,
			 tsp_key_intensity_read_show, NULL);
static DEVICE_ATTR(tsp_key_intensity_read_result, S_IRUGO | S_IWUSR | S_IWGRP,
			 tsp_key_intensity_read_result, NULL);

static DEVICE_ATTR(tsp_test, S_IRUGO | S_IWUSR | S_IWGRP, tsp_test_show, tsp_test_store);
static DEVICE_ATTR(tsp_reference, S_IRUGO | S_IWUSR | S_IWGRP, tsp_test_reference_show, tsp_test_reference_store);
static DEVICE_ATTR(tsp_inspection, S_IRUGO | S_IWUSR | S_IWGRP, tsp_test_inspection_show, tsp_test_inspection_store);
static DEVICE_ATTR(tsp_delta, S_IRUGO | S_IWUSR | S_IWGRP, tsp_test_delta_show, tsp_test_delta_store);
static DEVICE_ATTR(tsp_sleep, S_IRUGO | S_IWUSR | S_IWGRP, tsp_test_sleep_show, tsp_test_sleep_store);
static DEVICE_ATTR(tsp_wakeup, S_IRUGO | S_IWUSR | S_IWGRP, NULL, tsp_test_wakeup_store);
static DEVICE_ATTR(config_tsp_version, S_IRUGO | S_IWUSR | S_IWGRP,
						config_tsp_version_show, NULL);
static DEVICE_ATTR(tsp_firm_version_config, S_IRUGO | S_IWUSR | S_IWGRP,
						config_tsp_version_show, NULL);
#endif

#ifdef PREVAIL2_SYSFS_ADDED

















static ssize_t store_cmd(struct device *dev, struct device_attribute
		*devattr, const char *buf, size_t count)
{

	struct i2c_client *client = melfas_ts->client;

	char *cur, *start, *end;
	char buff[TSP_CMD_STR_LEN] = {0};
	int len, i;
	struct tsp_cmd *tsp_cmd_ptr = NULL;
	char delim = ',';
	bool cmd_found = false;
	int param_cnt = 0;


	if (melfas_ts->cmd_is_running == true) {
		dev_err(&melfas_ts->client->dev, "tsp_cmd: other cmd is running.\n");
		goto err_out;
	}


	/* check lock  */
	mutex_lock(&melfas_ts->cmd_lock);
	melfas_ts->cmd_is_running = true;
	mutex_unlock(&melfas_ts->cmd_lock);
	printk(KERN_DEBUG "\n store_cmd func1\n");
	melfas_ts->cmd_state = 1;

/*	for (i = 0; i < ARRAY_SIZE(melfas_ts->cmd_param); i++)
		melfas_ts->cmd_param[i] = 0;*/

	len = (int)count;
	if (*(buf + len - 1) == '\n')
		len--;
	memset(melfas_ts->cmd, 0x00, ARRAY_SIZE(melfas_ts->cmd));
	memcpy(melfas_ts->cmd, buf, len);

	cur = strchr(buf, (int)delim);
	if (cur)
		memcpy(buff, buf, cur - buf);
	else
		memcpy(buff, buf, len);

	/* find command */
	list_for_each_entry(tsp_cmd_ptr, &melfas_ts->cmd_list_head, list) {
		if (!strcmp(buff, tsp_cmd_ptr->cmd_name)) {
			cmd_found = true;
			break;
		}
	}

	/* set not_support_cmd */
	if (!cmd_found) {
		list_for_each_entry(tsp_cmd_ptr, &melfas_ts->cmd_list_head, list) {
			if (!strcmp("not_support_cmd", tsp_cmd_ptr->cmd_name))
				break;
		}
	}

	/* parsing parameters */
	if (cur && cmd_found) {
		cur++;
		start = cur;
		memset(buff, 0x00, ARRAY_SIZE(buff));
		do {
			if (*cur == delim || cur - buf == len) {
				end = cur;
				memcpy(buff, start, end - start);
				*(buff + strlen(buff)) = '\0';
				printk(KERN_DEBUG "\n buf = %s\n ", buff);
				start = cur + 1;
				param_cnt++;
			}
			cur++;
		} while (cur - buf <= len);
		memcpy(buf, buff, strlen(buff));
	}

	dev_info(&client->dev, "cmd = %s\n", tsp_cmd_ptr->cmd_name);
	for (i = 0; i < param_cnt; i++)
		dev_info(&client->dev, "cmd param %d= %d\n", i,
						melfas_ts->cmd_param[i]);
		tsp_cmd_ptr->cmd_func(dev, devattr, buf, count);

err_out:

	return count;
}



static void set_cmd_result(char *buff, int len)
{
	strncat(melfas_ts->cmd_result, buff, len);
}

static void set_default_result()
{
	char delim = ':';

	memset(melfas_ts->cmd_result, 0x00, ARRAY_SIZE(melfas_ts->cmd_result));
	memcpy(melfas_ts->cmd_result, melfas_ts->cmd, strlen(melfas_ts->cmd));
	strncat(melfas_ts->cmd_result, &delim, 1);
}

static ssize_t show_cmd_status(struct device *dev,
		struct device_attribute *devattr, char *buf)
{

	char buff[16] = {0};

	dev_info(&melfas_ts->client->dev, "tsp cmd: status:%d\n",
			melfas_ts->cmd_state);

	if (melfas_ts->cmd_state == 0)
		snprintf(buff, sizeof(buff), "WAITING");

	else if (melfas_ts->cmd_state == 1)
		snprintf(buff, sizeof(buff), "RUNNING");

	else if (melfas_ts->cmd_state == 2)
		snprintf(buff, sizeof(buff), "OK");

	else if (melfas_ts->cmd_state == 3)
		snprintf(buff, sizeof(buff), "FAIL");

	else if (melfas_ts->cmd_state == 4)
		snprintf(buff, sizeof(buff), "NOT_APPLICABLE");

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", buff);
}

static ssize_t show_cmd_result(struct device *dev, struct device_attribute
		*devattr, char *buf)
{
	int len;

	dev_info(&melfas_ts->client->dev, "tsp cmd: result: %s\n",
			melfas_ts->cmd_result);

	mutex_lock(&melfas_ts->cmd_lock);
	melfas_ts->cmd_is_running = false;
	mutex_unlock(&melfas_ts->cmd_lock);

	melfas_ts->cmd_state = 0;
	len = snprintf(buf, TSP_BUF_SIZE, "%s\n", melfas_ts->cmd_result);
	melfas_ts->cmd_result[0] = '\0';
	return len;
}

static void not_support_cmd(struct device *dev, struct device_attribute *attr,
		char *buf)
{

	char buff[16] = {0};
	set_default_result();
	sprintf(buff, "%s", "NA");
	set_cmd_result(buff, strnlen(buff, 100));
	melfas_ts->cmd_state = 4;
	dev_info(&melfas_ts->client->dev, "%s: \"%s(%d)\"\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
	return;
}









static DEVICE_ATTR(cmd, S_IWUSR | S_IWGRP, NULL, store_cmd);
static DEVICE_ATTR(cmd_status, S_IRUGO, show_cmd_status, NULL);
static DEVICE_ATTR(cmd_result, S_IRUGO, show_cmd_result, NULL);




static struct attribute *sec_touch_facotry_attributes[] = {
	&dev_attr_cmd.attr,
	&dev_attr_cmd_status.attr,
	&dev_attr_cmd_result.attr,
#ifdef ESD_DEBUG
	&dev_attr_intensity_logging_on.attr,
	&dev_attr_intensity_logging_off.attr,
#endif
	NULL,
};

static struct attribute_group sec_touch_factory_attr_group = {
	.attrs = sec_touch_facotry_attributes,
};









#endif







void melfas_read_reg()
{
	int ret;
	int ret1;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf;

	msg[0].addr = melfas_ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;

	start_reg = MCSTS_CABLE_DET_REG;

	msg[1].addr = melfas_ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = &buf;

	ret  = i2c_transfer(melfas_ts->client->adapter, &msg[0], 1);
	ret1 = i2c_transfer(melfas_ts->client->adapter, &msg[1], 1);

	if((ret < 0)||(ret1 < 0)) {
		printk(KERN_ERR "[TSP] ==melfas_ts_work_func: i2c_transfer failed!!== ret:%d ,ret1:%d\n",ret,ret1);
	}
	printk("[TSP] ===melfas_read_reg CABLE_DET_REG VALUE : 0x%02x===\n",buf);
}

void melfas_write_reg(u8 data)
{
	struct i2c_msg msg;
	u8 buf[2];

	buf[0] = MCSTS_CABLE_DET_REG;
	buf[1] = data;

	msg.addr = melfas_ts->client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = buf;

	if (1 != i2c_transfer(melfas_ts->client->adapter, &msg, 1)){
		printk("[TSP] %s fail! data(0x%x)\n", __func__,data);
		}
	#ifndef PRODUCT_SHIP
	printk("[TSP] %s : data(0x%x)\n", __func__,data);
	#endif
}

/*
void set_tsp_noise_filter_reg()
{
	fsa9480_i2c_read(REGISTER_DEVICETYPE1, &fsa9480_device1);
	if ((fsa9480_device1 & CRA_USB)||(fsa9480_device1 & CRA_DEDICATED_CHG)){
		melfas_write_reg(0x01);
		}
}
*/

void tsp_reset(void)
{
	printk(KERN_DEBUG "for esd %s\n", __func__) ;
#ifdef TSP_TA_NOISE	
	char	buf[2];	
#endif
	melfas_ts_force_touch_up();

	mcsdl_vdd_off();
	gpio_direction_output(GPIO_TSP_SCL, 0);  // TOUCH SCL DIS
	gpio_direction_output(GPIO_TSP_SDA, 0);  // TOUCH SDA DIS

    gpio_direction_output(GPIO_TSP_SCL, 1);  // TOUCH SCL EN
    gpio_direction_output(GPIO_TSP_SDA, 1);  // TOUCH SDA EN	
    mcsdl_vdd_on();
    msleep(300); 
#ifdef TSP_TA_NOISE
	buf[0] = 0xAB;
	buf[1] = melfas_ts->charging_status;
	melfas_i2c_write(melfas_ts->client, (char *)buf, 2);
#endif		
}

#ifdef TSP_TA_NOISE
static void inform_charger_connection(struct tsp_callbacks *cb, int mode)
{
	char buf[2];

	buf[0] = 0xAB;
	buf[1] = !!mode;
	melfas_ts->charging_status = !!mode;

	if(melfas_ts->tsp_status){
		pr_info("[TSP] TA is %sconnected\n", !!mode ? "" : "dis");
		melfas_i2c_write(melfas_ts->client, (char *)buf, 2);
	}
}
#endif

#if defined(CONFIG_MACH_ROOKIE2) || defined (CONFIG_TOUCHSCREEN_MELFAS_TS)
static bool touched_src = false;
static int touch_pressed = 0;  // 1:press, 0:release
static void melfas_work_func(void)
{
	int ret = 0, i, j;	
	uint8_t buf[66] = {0};
	int read_num = 0, touchType = 0, touchState = 0, fingerID = 0, keyID = 0;
	unsigned long flags;

	touched_src = false;
	
//	for (i = 0; i < 10; i++)
	for (i = 0; i < 5; i++)		
	{	
		ret = melfas_i2c_read(melfas_ts->client, 0x0F, buf, 1);
//		udelay(50);
		if (ret >= 0)		
		break; // i2c success	
	}	
	//spin_lock_irqsave(&melfas_spin_lock, flags);	

	if (ret < 0)	
	{	
		printk("[TSP]%s i2c failed : %d\n", __func__, ret);	
		tsp_reset();
		return ;	
	}	
	else
	{	
		read_num = buf[0];	
	}

	if ((read_num > 0) && (read_num <= 66)) {
#ifdef __TOUCH_DEBUG__
		printk("[TSP] %s,read_num: %d \n",__func__, read_num);
#endif
//		for (i = 0; i < 10; i++)
		for (i = 0; i < 5; i++)			
		{	
			ret = melfas_i2c_read(melfas_ts->client, 0x10, buf, read_num);	
			if (ret >= 0)			
			break; // i2c success	
		}	
		if (ret < 0)	
		{		
			printk("[TSP]%s i2c failed : %d\n", __func__, ret);
			tsp_reset();
			return ;	
		}		
		else		
		{				
			if (buf[0] == 0x0f)	
			{		
				printk("[TSP]%s ESD defense!!  : %d\n", __func__, fingerID);
				tsp_reset();
				return;	
			}	
			for (i = 0; i < read_num; i = i + 6)	
			{
				touchType = (buf[i] >> 5) & 0x03;	// 6, 7th bits
				touchState = (buf[i] & 0x80);		// 8th bit
				if (touchType == 1)	//Screen	
				{
#ifdef __TOUCH_DEBUG__
//					printk("[TSP] ************ screen touch *********** \n");
#endif
					touched_src = true;			
					fingerID = (buf[i] & 0x0F) - 1;	// 1,2,3,4th bits
					if ((fingerID > TS_MAX_TOUCH - 1) || (fingerID < 0))		
					{	
						printk("[TSP]%s fingerID : %d\n", __func__, fingerID);	
						tsp_reset();
						return ;		
					}	
					melfas_ts->info[fingerID].x = (uint16_t)(buf[i + 1] & 0x0F) << 8 | buf[i + 2]; // total 12bits use
					melfas_ts->info[fingerID].y = (uint16_t)(buf[i + 1] & 0xF0) << 4 | buf[i + 3]; // total 12bits use	
					melfas_ts->info[fingerID].width = buf[i + 4];
					if (touchState)				
						melfas_ts->info[fingerID].z = buf[i + 5];	
					else				
						melfas_ts->info[fingerID].z = 0;
				}
#ifdef __TOUCH_KEY__		
				else if (touchType == 2)	//Key			
				{
#ifdef __TOUCH_DEBUG__
					//printk("\n [TSP] ************ keypad  key *********** id = %d\n",(buf[i]&0x0F));
#endif
					keyID = (buf[i] & 0x0F);		
					if (keyID == 0x1)			
					{			
						input_report_key(melfas_ts->input_dev,KEY_MENU, touchState ? 1 : 0);
#ifdef TSP_TEST_MODE
						menu_sensitivity = buf[i + 5];
						menu_pressed = touchState;
#endif						
					}

					if (keyID == 0x2)			
                    {
						input_report_key(melfas_ts->input_dev,  KEY_HOMEPAGE, touchState ? 1 : 0);
#ifdef TSP_TEST_MODE
                                                home_sensitivity = buf[i + 5];
                                                home_pressed = touchState;
#endif						
					}
					if (keyID == 0x3){
						input_report_key(melfas_ts->input_dev, KEY_BACK, touchState ? 1 : 0);
#ifdef TSP_TEST_MODE
                                                back_sensitivity = buf[i + 5];
                                                back_pressed = touchState;
#endif
					}
					if (keyID == 0x4){
						input_report_key(melfas_ts->input_dev, KEY_SEARCH, touchState ? 1 : 0);
					}
#ifdef __TOUCH_KEYLED__		
					if( !g_check_keyled)			
					{
						b_keyledOn = true;	
					}
#endif

#ifdef __TOUCH_DEBUG__				
					printk("[TSP]%s keyID: %d, State: %d\n", __func__, keyID, touchState);
#endif				
				}
#endif	//  __TOUCH_KEY__	
				}			

				if (touched_src)			
				{	
					for (j = 0; j < TS_MAX_TOUCH; j ++)	
					{
						if (melfas_ts->info[j].z == -1)			
							continue;		
						
					
						if(melfas_ts->info[j].z) {
							input_mt_slot(melfas_ts->input_dev, j);
							input_mt_report_slot_state(melfas_ts->input_dev,MT_TOOL_FINGER, true);
							//input_report_abs(melfas_ts->input_dev, ABS_MT_TRACKING_ID, j);	
							input_report_abs(melfas_ts->input_dev, ABS_MT_WIDTH_MAJOR, melfas_ts->info[j].width);
							input_report_abs(melfas_ts->input_dev, ABS_MT_PRESSURE, melfas_ts->info[j].z);
							input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_X, melfas_ts->info[j].x);		
							input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_Y, melfas_ts->info[j].y);	
							input_report_key(melfas_ts->input_dev, BTN_TOUCH, melfas_ts->info[j].z);							
						}
						else
						{	
							input_mt_slot(melfas_ts->input_dev, j);
							input_mt_report_slot_state(melfas_ts->input_dev,MT_TOOL_FINGER, false);
							//input_report_abs(melfas_ts->input_dev, ABS_MT_TRACKING_ID, j);	
							//input_report_abs(melfas_ts->input_dev, ABS_MT_PRESSURE, melfas_ts->info[j].z);
							
						}						
						//input_mt_sync(melfas_ts->input_dev);

#ifdef __TOUCH_DEBUG__				
					printk("[TSP]%s fingerID: %d, State: %d, x: %d, y: %d, z: %d, w: %d\n",
						__func__, j, (melfas_ts->info[j].z > 0), melfas_ts->info[j].x, melfas_ts->info[j].y, melfas_ts->info[j].z, melfas_ts->info[j].width);
#endif					
						if (melfas_ts->info[j].z == 0)		
							melfas_ts->info[j].z = -1;				
					}
				}	
			input_sync(melfas_ts->input_dev);
			
#ifdef __TOUCH_KEYLED__	
					g_check_action = true;
#endif		
			}
			}
	
	else
			{	
				printk("[TSP]%s : read_num=%d\n",__func__, read_num);
			}

}

#else
static void melfas_work_func(void)
{
	
	int i = 0;
	u8 id = 0;
	int z = 0;
	int width = 0;

	int x = buf1[2] | ((uint16_t)(buf1[1] & 0x0f) << 8);
	int y = buf1[3] | (((uint16_t)(buf1[1] & 0xf0) >> 4) << 8);

#if defined(CONFIG_MACH_CHIEF)
        if(system_rev >= 8){
                z = buf1[5];   //strength
                width = buf1[4]; // width
        }
        else
                z = buf1[4];   //strength
#else //vital2
		if(system_rev >= 5){
                z = buf1[5];   //strength
                width = buf1[4]; // width
        }
        else
                z = buf1[4];   //strength
#endif

        int finger = buf1[0] & 0x0f;  //Touch Point ID
        int touchaction = (int)((buf1[0] >> 4) & 0x3); //Touch action
#ifdef CONFIG_CPU_FREQ
        set_dvfs_perf_level();
#endif

        id = finger;
        //printk("===touchaction : 0x%02x===\n",touchaction);
			switch(touchaction) {
				case 0x0: // Non-touched state
					melfas_ts->info[id].x = -1;
					melfas_ts->info[id].y = -1;
					melfas_ts->info[id].z = -1;
					melfas_ts->info[id].finger_id = finger;
					z = 0;
					break;

				case 0x1: //touched state
					melfas_ts->info[id].x = x;
					melfas_ts->info[id].y = y;
					melfas_ts->info[id].z = z;
					melfas_ts->info[id].finger_id = finger;
					break;

				case 0x2:
					break;

				case 0x3: // Palm Touch
					printk(KERN_DEBUG "[TSP] Palm Touch!\n");
					break;

				case 0x7: // Proximity
					printk(KERN_DEBUG "[TSP] Proximity!\n");
					break;
			}

			melfas_ts->info[id].state = touchaction;
        for ( i= 1; i<FINGER_NUM+1; ++i ) {
#if defined(CONFIG_MACH_CHIEF)
                if(system_rev >= 8) {
                        //debugprintk(5,"[TOUCH_MT] x1: %4d, y1: %4d, z1: %4d, finger: %4d,\n", x, y, z, finger);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_X, melfas_ts->info[i].x);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_Y, melfas_ts->info[i].y);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_TOUCH_MAJOR, melfas_ts->info[i].z);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_WIDTH_MAJOR, melfas_ts->info[i].width);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_TRACKING_ID, melfas_ts->info[i].finger_id-1);
                }
                else {
                        //debugprintk(5,"[TOUCH_MT] x1: %4d, y1: %4d, z1: %4d, finger: %4d,\n", x, y, z, finger);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_X, melfas_ts->info[i].x);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_Y, melfas_ts->info[i].y);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_TOUCH_MAJOR, melfas_ts->info[i].z);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_TRACKING_ID, melfas_ts->info[i].finger_id-1);
                }
#else //vital2
                if(system_rev >= 4) {
                        //debugprintk(5,"[TOUCH_MT] x1: %4d, y1: %4d, z1: %4d, finger: %4d,\n", x, y, z, finger);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_X, melfas_ts->info[i].x);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_Y, melfas_ts->info[i].y);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_TOUCH_MAJOR, melfas_ts->info[i].z);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_WIDTH_MAJOR, melfas_ts->info[i].width);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_TRACKING_ID, melfas_ts->info[i].finger_id-1);
                }
                else {
                        //debugprintk(5,"[TOUCH_MT] x1: %4d, y1: %4d, z1: %4d, finger: %4d,\n", x, y, z, finger);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_X, melfas_ts->info[i].x);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_POSITION_Y, melfas_ts->info[i].y);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_TOUCH_MAJOR, melfas_ts->info[i].z);
                        input_report_abs(melfas_ts->input_dev, ABS_MT_TRACKING_ID, melfas_ts->info[i].finger_id-1);
                }
#endif
                input_mt_sync(melfas_ts->input_dev);
        }

#ifndef PRODUCT_SHIP
#if defined(CONFIG_MACH_CHIEF)
        if(system_rev >= 8)
                debugprintk(5,"[TSP] x: %4d, y: %4d, z: %4d, f: %4d, w: %4d\n", x, y, z, finger, width);
        else
                debugprintk(5,"[TSP] x: %4d, y: %4d, z: %4d, finger: %4d,\n", x, y, z, finger);	
#else
        if(system_rev >= 4)
                debugprintk(5,"[TSP] x: %4d, y: %4d, z: %4d, f: %4d, w: %4d\n", x, y, z, finger, width);
        else
                debugprintk(5,"[TSP] x: %4d, y: %4d, z: %4d, finger: %4d,\n", x, y, z, finger);
#endif
#endif
        input_sync(melfas_ts->input_dev);
}

#endif

irqreturn_t melfas_ts_irq_handler(int irq, void *dev_id)
{
  int ret;
  int ret1;

  struct i2c_msg msg[2];

  uint8_t start_reg;

  msg[0].addr = melfas_ts->client->addr;
  msg[0].flags = 0;
  msg[0].len = 1;
  msg[0].buf = &start_reg;
  start_reg = MCSTS_INPUT_INFO_REG;

  msg[1].addr = melfas_ts->client->addr;
  msg[1].flags = I2C_M_RD;
  msg[1].len = sizeof(buf1);
  msg[1].buf = buf1;

#if defined(CONFIG_MACH_CHIEF)
	if(system_rev >= 8){
		ret = i2c_transfer(melfas_ts->client->adapter, msg, 2);

		if (ret < 0){
			printk(KERN_ERR "[TSP] melfas_ts_work_func: i2c_transfer failed\n");
			}
		else{
			melfas_work_func();
			}
		}
	else{
		ret  = i2c_transfer(melfas_ts->client->adapter, &msg[0], 1);
		ret1 = i2c_transfer(melfas_ts->client->adapter, &msg[1], 1);

		if((ret < 0) ||  (ret1 < 0)){
			printk(KERN_ERR "[TSP] ==melfas_ts_work_func: i2c_transfer failed!!== ret:%d ,ret1:%d\n",ret,ret1);
			}
		else{
			melfas_work_func();
			}
		}
#elif defined (CONFIG_MACH_ROOKIE2) || defined (CONFIG_TOUCHSCREEN_MELFAS_TS)
//#elif defined (CONFIG_MACH_ROOKIE2)
			melfas_work_func();
#else //vital2
	if(system_rev >= 4){
		ret = i2c_transfer(melfas_ts->client->adapter, msg, 2);
		if (ret < 0){
			printk(KERN_ERR "[TSP] melfas_ts_work_func: i2c_transfer failed\n");
			}
		else{
			if(buf1[0] == 0x0F)
			tsp_reset();
			else
			melfas_work_func();
			}
		}
	else{
		ret  = i2c_transfer(melfas_ts->client->adapter, &msg[0], 1);
		udelay(50);	// delay write register and read data
		ret1 = i2c_transfer(melfas_ts->client->adapter, &msg[1], 1);

	  	if((ret < 0) ||  (ret1 < 0)){
	  		printk(KERN_ERR "[TSP] ==melfas_ts_work_func: i2c_transfer failed!!== ret:%d ,ret1:%d\n",ret,ret1);
			}
	  	else{
			if(buf1[0] == 0x0F)
			tsp_reset();
			else
	  		melfas_work_func();
			}
		}
#endif

  return IRQ_HANDLED;
}

#if defined(CONFIG_MACH_VITAL2) || defined (CONFIG_TOUCHSCREEN_MELFAS_TS)
int melfas_ts_probe(struct platform_device *pdev)
{
	int ret = 0;
	uint16_t max_x=0, max_y=0;
	int recommend_ver = 0;
	int Compatibility_group = 0 ;
	static char physical_addr[20];
	int i;
	
#ifndef PRODUCT_SHIP 
	printk("\n====================================================");
	printk("\n=======         [TSP] PROBE       =========");
	printk("\n====================================================\n");
#endif
	if (!i2c_check_functionality(melfas_ts->client->adapter, I2C_FUNC_I2C/*I2C_FUNC_SMBUS_BYTE_DATA*/)) {
		#ifndef PRODUCT_SHIP 
		printk(KERN_ERR "[TSP] melfas_ts_probe: need I2C_FUNC_I2C\n");
		#endif
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
#ifdef TSP_TA_NOISE
	melfas_ts->pdata = pdev->dev.platform_data;

	melfas_ts->callbacks.inform_charger = inform_charger_connection;
	if (melfas_ts->pdata->register_cb)
		melfas_ts->pdata->register_cb(&melfas_ts->callbacks);
	melfas_ts->tsp_status = true;
#endif

#ifdef PREVAIL2_SYSFS_ADDED
	mutex_init(&melfas_ts->cmd_lock);
	melfas_ts->cmd_is_running = false;

	INIT_LIST_HEAD(&melfas_ts->cmd_list_head);
	for (i = 0; i < ARRAY_SIZE(tsp_cmds); i++)
		list_add_tail(&tsp_cmds[i].list, &melfas_ts->cmd_list_head);


#endif

	melfas_read_version();
#ifndef PRODUCT_SHIP 
	printk(KERN_INFO "[TSP] Melfas  H/W version: 0x%02x.\n", melfas_ts->hw_rev);
	printk(KERN_INFO "[TSP] Melfas  H/W version: %c.\n", (char)melfas_ts->com_type);
	printk(KERN_INFO "[TSP] Current F/W version: 0x%02x.\n", melfas_ts->fw_ver);
#endif

// unnecessary power on/off
//	mdelay(100);
//	mcsdl_vdd_off();
//	mdelay(250);
//	mcsdl_vdd_on();
	

	// To check firmware update, get recommend_ver, Compatibility_group
	recommend_ver = Melfas_recommend_ver();
	Compatibility_group = Melfas_Compatibility_group();
	printk(KERN_DEBUG"[TSP] %s recommend_ver:%x, Compatibility_group:%c\n",
			__func__, recommend_ver, Compatibility_group);
	printk(KERN_DEBUG"[TSP] %s melfas_ts->fw_ver:%x, melfas_ts->com_type:%c\n",
			__func__, melfas_ts->fw_ver, melfas_ts->com_type);

#if defined (CONFIG_TOUCHSCREEN_MELFAS_TS)
	// when firmware update fail, add com_type == 0x00 condition to re-try update during booting
	if((melfas_ts->com_type == Compatibility_group) || (melfas_ts->com_type == 0x00))
	{
		if(recommend_ver > melfas_ts->fw_ver)
		{
#if 0	// firmware update with schedule
//	schedule_delayed_work(&firmware_work, FIRMWARE_WORK_CHECK_TIMEOUT);
#endif
			melfas_firmware_download();
		}
		else
		{
			printk(KERN_DEBUG"[TSP] Firmware is the latest. fw_ver:%x, com_type:%c\n",
					melfas_ts->fw_ver, melfas_ts->com_type);
		}
	}
	else
	{
		printk(KERN_DEBUG"[TSP] The pannel is different. fw_ver:%x, com_type:%c\n",
				melfas_ts->fw_ver, melfas_ts->com_type);
	}
#endif

	melfas_read_resolution();
	max_x = 320; //melfas_ts->info[0].max_x ; // x resolution
	max_y = 480; //melfas_ts->info[0].max_y ; //  y resolution

#ifndef PRODUCT_SHIP 
	printk("[TSP] melfas_ts_probe: max_x: %d, max_y: %d\n", max_x, max_y);
#endif

	melfas_ts->input_dev = input_allocate_device();
	if (melfas_ts->input_dev == NULL) {
		ret = -ENOMEM;
		#ifndef PRODUCT_SHIP
		printk(KERN_ERR "[TSP] melfas_ts_probe: Failed to allocate input device\n");
		#endif
		goto err_input_dev_alloc_failed;
	}

	melfas_ts->input_dev->name = "sec_touchscreen";

	snprintf(physical_addr, 10, "input(ts)");
	melfas_ts->input_dev->phys = physical_addr;

	set_bit(EV_SYN, melfas_ts->input_dev->evbit);
	set_bit(EV_KEY, melfas_ts->input_dev->evbit);
	set_bit(MT_TOOL_FINGER, melfas_ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, melfas_ts->input_dev->propbit);
	set_bit(TOUCH_HOME, melfas_ts->input_dev->keybit);
	set_bit(TOUCH_MENU, melfas_ts->input_dev->keybit);
	set_bit(TOUCH_BACK, melfas_ts->input_dev->keybit);
	set_bit(TOUCH_SEARCH, melfas_ts->input_dev->keybit);
	input_mt_init_slots(melfas_ts->input_dev, TS_MAX_TOUCH);
	
	melfas_ts->input_dev->keycode = melfas_ts_tk_keycode;
	/*set_bit(BTN_TOUCH, melfas_ts->input_dev->keybit);*/
	set_bit(EV_ABS, melfas_ts->input_dev->evbit);
	
	input_set_abs_params(melfas_ts->input_dev, ABS_MT_POSITION_X,  0, max_x, 0, 0);
	input_set_abs_params(melfas_ts->input_dev, ABS_MT_POSITION_Y,  0, max_y, 0, 0);
	/*input_set_abs_params(melfas_ts->input_dev,
			 ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);*/
	input_set_abs_params(melfas_ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 30, 0, 0);
	input_set_abs_params(melfas_ts->input_dev, ABS_MT_TRACKING_ID, 0, 4, 0, 0);
	/*input_set_abs_params(melfas_ts->input_dev,
				ABS_TOOL_WIDTH,	0, 255, 0, 0);*/
	input_set_abs_params(melfas_ts->input_dev,
					ABS_MT_PRESSURE, 0, 255, 0, 0);
	ret = input_register_device(melfas_ts->input_dev);
	if (ret) {
		#ifndef PRODUCT_SHIP 
		printk(KERN_ERR "[TSP] melfas_ts_probe: Unable to register %s input device\n", melfas_ts->input_dev->name);
		#endif
		goto err_input_register_device_failed;
	}

	melfas_ts->irq = melfas_ts->client->irq;
	//ret = request_irq(melfas_ts->client->irq, melfas_ts_irq_handler, IRQF_DISABLED, "melfas_ts irq", 0);
	ret = request_threaded_irq(melfas_ts->client->irq, NULL, melfas_ts_irq_handler,IRQF_ONESHOT,"melfas_ts irq", 0);
	if(ret == 0) {
		#ifndef PRODUCT_SHIP 
		printk(KERN_INFO "[TSP] melfas_ts_probe: Start touchscreen %s \n", melfas_ts->input_dev->name);
		#endif
	}
	else {
		printk("[TSP] request_irq failed\n");
	}

	#if 0 //defined(CONFIG_MACH_CHIEF)
		set_tsp_noise_filter_reg();
	#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
	melfas_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	melfas_ts->early_suspend.suspend = melfas_ts_early_suspend;
	melfas_ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&melfas_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#ifdef LCD_WAKEUP_PERFORMANCE
	/* for synchronous ts operations */
	complete(&ts_completion);
#endif

	return 0;

err_input_register_device_failed:
	input_free_device(melfas_ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	kfree(melfas_ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;

}
#else
int melfas_ts_probe(void)
{
	int ret = 0;
	uint16_t max_x=0, max_y=0;
	
#ifndef PRODUCT_SHIP 
	printk("\n===========================================");
	printk("\n=======         [TSP] PROBE       =========");
	printk("\n===========================================\n");
#endif

	if (!i2c_check_functionality(melfas_ts->client->adapter, I2C_FUNC_I2C/*I2C_FUNC_SMBUS_BYTE_DATA*/)) {
		#ifndef PRODUCT_SHIP 
		printk(KERN_ERR "[TSP] melfas_ts_probe: need I2C_FUNC_I2C\n");
		#endif
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	melfas_read_version();
#ifndef PRODUCT_SHIP 
	printk(KERN_INFO "[TSP] Melfas  H/W version: 0x%02x.\n", melfas_ts->hw_rev);
	printk(KERN_INFO "[TSP] Current F/W version: 0x%02x.\n", melfas_ts->fw_ver);
#endif

	mdelay(100);
	mcsdl_vdd_off();
	mdelay(300);
	mcsdl_vdd_on();
	
#if defined(CONFIG_MACH_CHIEF)
	if((system_rev >= 8) && (melfas_ts->fw_ver < 0x21))
		melfas_firmware_download();
#elif  defined(CONFIG_MACH_ROOKIE2)
	if((system_rev >= 5) && (melfas_ts->fw_ver < 0x04))
		melfas_firmware_download(); 
#else //vital2
	if((system_rev >= 5) && (melfas_ts->fw_ver < 0x29))
		melfas_firmware_download(); 
#endif


	melfas_read_resolution();
	max_x = melfas_ts->info[0].max_x ;
	max_y = melfas_ts->info[0].max_y ;

#ifndef PRODUCT_SHIP 
	printk("[TSP] melfas_ts_probe: max_x: %d, max_y: %d\n", max_x, max_y);
#endif

	melfas_ts->input_dev = input_allocate_device();
	if (melfas_ts->input_dev == NULL) {
		ret = -ENOMEM;
		#ifndef PRODUCT_SHIP
		printk(KERN_ERR "[TSP] melfas_ts_probe: Failed to allocate input device\n");
		#endif
		goto err_input_dev_alloc_failed;
	}

	melfas_ts->input_dev->name = "sec_touchscreen";

	snprintf(physical_addr, 10, "input(ts)");
	melfas_ts->input_dev->phys = physical_addr;

	set_bit(EV_SYN, melfas_ts->input_dev->evbit);
	set_bit(EV_KEY, melfas_ts->input_dev->evbit);
	set_bit(TOUCH_HOME, melfas_ts->input_dev->keybit);
	set_bit(TOUCH_MENU, melfas_ts->input_dev->keybit);
	set_bit(TOUCH_BACK, melfas_ts->input_dev->keybit);
	set_bit(TOUCH_SEARCH, melfas_ts->input_dev->keybit);

	melfas_ts->input_dev->keycode = melfas_ts_tk_keycode;
	set_bit(BTN_TOUCH, melfas_ts->input_dev->keybit);
	set_bit(EV_ABS, melfas_ts->input_dev->evbit);

	input_set_abs_params(melfas_ts->input_dev, ABS_MT_POSITION_X,  0, max_x, 0, 0);
	input_set_abs_params(melfas_ts->input_dev, ABS_MT_POSITION_Y,  0, max_y, 0, 0);
	input_set_abs_params(melfas_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(melfas_ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 30, 0, 0);
	input_set_abs_params(melfas_ts->input_dev, ABS_MT_TRACKING_ID, 0, 4, 0, 0);

	ret = input_register_device(melfas_ts->input_dev);
	if (ret) {
		#ifndef PRODUCT_SHIP 
		printk(KERN_ERR "[TSP] melfas_ts_probe: Unable to register %s input device\n", melfas_ts->input_dev->name);
		#endif
		goto err_input_register_device_failed;
	}

	melfas_ts->irq = melfas_ts->client->irq;
	//ret = request_irq(melfas_ts->client->irq, melfas_ts_irq_handler, IRQF_DISABLED, "melfas_ts irq", 0);
	ret = request_threaded_irq(melfas_ts->client->irq, NULL, melfas_ts_irq_handler,IRQF_ONESHOT,"melfas_ts irq", 0);
	if(ret == 0) {
		#ifndef PRODUCT_SHIP 
		printk(KERN_INFO "[TSP] melfas_ts_probe: Start touchscreen %s \n", melfas_ts->input_dev->name);
		#endif
	}
	else {
		printk("[TSP] request_irq failed\n");
	}


	#if defined(CONFIG_MACH_CHIEF)
		set_tsp_noise_filter_reg();
	#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
	melfas_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	melfas_ts->early_suspend.suspend = melfas_ts_early_suspend;
	melfas_ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&melfas_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#ifdef LCD_WAKEUP_PERFORMANCE
	/* for synchronous ts operations */
	complete(&ts_completion);
#endif

	return 0;
err_misc_register_device_failed:
err_input_register_device_failed:
	input_free_device(melfas_ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	kfree(melfas_ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;

}
#endif

int melfas_ts_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&melfas_ts->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
	free_irq(melfas_ts->irq, 0);
	input_unregister_device(melfas_ts->input_dev);
	return 0;
}

int melfas_ts_check_press(void){
	int i = 0;

	touch_pressed = 0; // 1:press, 0:release
	
	for(i = 0; i < TS_MAX_TOUCH; i++)
	{
		if(melfas_ts->info[i].z > 0)
		{	
			printk(KERN_DEBUG "[TSP] %s melfas_ts->info[%d].z : %d\n",
					__func__, i, melfas_ts->info[i].z);
			touch_pressed = 1;
		}
	}

	return touch_pressed;	
}

void melfas_ts_force_touch_up(void)
{
	int i;
	printk("[TSP] %s touch_pressed:%d \n", __func__, touch_pressed);

	for (i = 0; i < TS_MAX_TOUCH; i++) 
	{
		if (melfas_ts->info[i].z > 0) {


			input_mt_slot(melfas_ts->input_dev, i);
			input_mt_report_slot_state(melfas_ts->input_dev,
							 MT_TOOL_FINGER, false);

#ifdef __TOUCH_DEBUG__				
			printk(KERN_DEBUG "[TSP][MMS128][%s] fingerID: %d, State: %d, x: %d, y: %d, z: %d, w: %d\n",
			__func__, i, (melfas_ts->info[i].z > 0), melfas_ts->info[i].x,
			melfas_ts->info[i].y, melfas_ts->info[i].z,
			melfas_ts->info[i].width);
#endif
		}

		melfas_ts->info[i].z = -1;
	}

	input_sync(melfas_ts->input_dev);
	touch_pressed = 0;
}

int melfas_ts_suspend(pm_message_t mesg)
{
#ifdef LCD_WAKEUP_PERFORMANCE
    /* for synchronous ts operations, wait until the ts resume work is complete */
    wait_for_completion(&ts_completion);
#endif

	printk("[TSP] Start %s \n", __func__);

    melfas_ts->suspended = true;

#ifdef TSP_TEST_MODE
    touch_screen.device_state = false; 
#endif
    disable_irq(melfas_ts->irq);
	
	if(melfas_ts_check_press())
	{
		melfas_ts_force_touch_up();
	}
	printk("[TSP] %s touch_pressed:%d \n", __func__, touch_pressed);
	
//    gpio_tlmm_config(GPIO_CFG(GPIO_TSP_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
//    gpio_tlmm_config(GPIO_CFG(GPIO_TSP_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),GPIO_CFG_ENABLE);

	// Requested by HW, 1.8V constant voltage
	gpio_tlmm_config(GPIO_CFG(GPIO_TSP_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TSP_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);

    mcsdl_vdd_off();
    gpio_set_value(GPIO_TSP_SCL, 0);  // TOUCH SCL DIS
    gpio_set_value(GPIO_TSP_SDA, 0);  // TOUCH SDA DIS
#ifdef TSP_TA_NOISE
	melfas_ts->tsp_status = false;
#endif
	printk("[TSP] End %s \n", __func__);

    return 0;
}

#ifdef LCD_WAKEUP_PERFORMANCE
static void ts_resume_work_func(struct work_struct *ignored)
{
#ifdef TSP_TA_NOISE
	char buf[2];
#endif

    melfas_ts->suspended = false;
#ifdef TSP_TA_NOISE
	melfas_ts->tsp_status = true;
	buf[0] = 0xAB;
	buf[1] = melfas_ts->charging_status;
	melfas_i2c_write(melfas_ts->client, (char *)buf, 2);	
    printk("[TSP]ta status %d in resume\n", melfas_ts->charging_status);
#endif	

#ifdef TSP_TEST_MODE
    touch_screen.device_state = true; 
#endif
    enable_irq(melfas_ts->irq);

#if 0 //defined(CONFIG_MACH_CHIEF)
    set_tsp_noise_filter_reg();
#endif

    /* we should always call complete, otherwise the caller will be deadlocked */
    complete(&ts_completion);
}
#endif

int melfas_ts_resume(void)
{
	printk("[TSP] Start %s \n", __func__);

    gpio_tlmm_config(GPIO_CFG(GPIO_TSP_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
    gpio_tlmm_config(GPIO_CFG(GPIO_TSP_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
    gpio_set_value(GPIO_TSP_SCL, 1);  // TOUCH SCL EN
    gpio_set_value(GPIO_TSP_SDA, 1);  // TOUCH SDA EN

    mcsdl_vdd_on();
	mdelay(10);
#ifdef LCD_WAKEUP_PERFORMANCE
    schedule_delayed_work(&ts_resume_work, msecs_to_jiffies(200));
#else
    melfas_ts->suspended = false;

#ifdef TSP_TEST_MODE
    touch_screen.device_state = true; 
#endif
    enable_irq(melfas_ts->irq);

#if 0 //defined(CONFIG_MACH_CHIEF)
    set_tsp_noise_filter_reg();
#endif
#endif
	printk("[TSP] End %s \n", __func__);
    return 0;
}

int tsp_preprocess_suspend(void)
{
#if 0 // blocked for now.. we will gen touch when suspend func is called
  // this function is called before kernel calls suspend functions
  // so we are going suspended if suspended==false
  if(melfas_ts->suspended == false) {
    // fake as suspended
    melfas_ts->suspended = true;

    //generate and report touch event
    melfas_ts_force_touch_up();
  }
#endif
  return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void melfas_ts_early_suspend(struct early_suspend *h)
{
	melfas_ts_suspend(PMSG_SUSPEND);
}

void melfas_ts_late_resume(struct early_suspend *h)
{
	melfas_ts_resume();
}
#endif	/* CONFIG_HAS_EARLYSUSPEND */


int melfas_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	melfas_ts->client = client;
	i2c_set_clientdata(client, melfas_ts);
	return 0;
}

//static int __devexit melfas_i2c_remove(struct i2c_client *client)
static int melfas_i2c_remove(struct i2c_client *client)

{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&melfas_ts->early_suspend);
#endif  /* CONFIG_HAS_EARLYSUSPEND */
	free_irq(melfas_ts->client->irq, 0);
	input_unregister_device(melfas_ts->input_dev);

	melfas_ts = i2c_get_clientdata(client);
	kfree(melfas_ts);
	return 0;
}

struct i2c_device_id melfas_id[] = {
	{ "melfas_ts_i2c", 0 },
//	{ "melfas_ts_i2c", 2 },
	{ }
};

struct i2c_driver melfas_ts_i2c = {
	.driver = {
		.name	= "melfas_ts_i2c",
		.owner	= THIS_MODULE,
	},
	.probe 		= melfas_i2c_probe,
	.remove		= melfas_i2c_remove,
	//.remove		= __devexit_p(melfas_i2c_remove),
	.id_table	= melfas_id,
};


void init_hw_setting(void)
{
	printk("[TSP] %s 1 \n",__func__);
	mcsdl_vdd_on();
	gpio_tlmm_config(GPIO_CFG(GPIO_TSP_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TSP_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_TOUCH_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),GPIO_CFG_ENABLE);

	irq_set_irq_type(IRQ_TOUCH_INT, IRQ_TYPE_LEVEL_LOW); //chief.boot.temp changed from edge low to level low VERIFY!!!
	printk("[TSP] %s 3 \n",__func__);
	mdelay(10);
	printk("[TSP] %s 4 \n",__func__);
}

struct platform_driver melfas_ts_driver =  {
	.probe	= melfas_ts_probe,
	.remove = melfas_ts_remove,
	.driver = {
		.name = "melfas-ts",
		.owner	= THIS_MODULE,
	},
};

int __init melfas_ts_init(void)
{
	int ret, i;
	
#ifndef PRODUCT_SHIP 	
	printk("\n===========================================");
	printk("\n=======         [TSP] INIT        =========");
	printk("\n===========================================\n");
#endif
	init_hw_setting();
	printk("[TSP] %s \n",__func__);
	ts_dev = device_create(sec_class, NULL, 0, NULL, "sec_touchscreen");
	if (IS_ERR(ts_dev))
		pr_err("Failed to create device(sec_touchscreen)!\n");
	sec_touchkey_dev = device_create(sec_class, NULL, 0, NULL, "sec_touchkey");
	if (IS_ERR(sec_touchkey_dev))
		pr_err("Failed to create device(sec_touchkey)!\n");
	if (device_create_file(ts_dev, &dev_attr_gpio) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_gpio.attr.name);
	if (device_create_file(ts_dev, &dev_attr_registers) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_registers.attr.name);
	if (device_create_file(ts_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(ts_dev, &dev_attr_debug) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_debug.attr.name);
	
#ifdef TSP_TEST_MODE
	// 15 mode KEY test
	if (device_create_file(ts_dev, &dev_attr_tkey_enter_sensitivity) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_enter_sensitivity.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tkey_enter_normal) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_enter_normal.attr.name);
	if (device_create_file(sec_touchkey_dev, &dev_attr_touchkey_menu) < 0)
		pr_err("Failed to create device file(%s)!\n",
				dev_attr_touchkey_menu.attr.name);
	if (device_create_file(sec_touchkey_dev, &dev_attr_touchkey_home) < 0)
		pr_err("Failed to create device file(%s)!\n",
				dev_attr_touchkey_home.attr.name);
	if (device_create_file(sec_touchkey_dev, &dev_attr_touchkey_back) < 0)
		pr_err("Failed to create device file(%s)!\n",
				dev_attr_touchkey_back.attr.name);
	if (device_create_file(sec_touchkey_dev, &dev_attr_tkey_back_press) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_back_press.attr.name);
	if (device_create_file(sec_touchkey_dev, &dev_attr_tkey_menu_press) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_menu_press.attr.name);
	if (device_create_file(sec_touchkey_dev, &dev_attr_tkey_home_press) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tkey_home_press.attr.name);
	if (device_create_file(sec_touchkey_dev, \
				&dev_attr_touchkey_firm_version_panel) < 0)
		pr_err("Failed to create device file(%s)!\n", \
				dev_attr_touchkey_firm_version_panel.attr.name);

	/*TSP vesion for *#2663#*/
	if (device_create_file(ts_dev, &dev_attr_tsp_firm_version_phone) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_firm_version_phone.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_firm_version_panel) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_firm_version_panel.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_firm_version_recommend) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_firm_version_recommend.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_firm_update) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_firm_update.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_firm_update_status) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_firm_update_status.attr.name);
	/*tsp threshould*/
	if (device_create_file(ts_dev, &dev_attr_tsp_threshold) < 0)
	pr_err("Failed to create device file(%s)!\n", \
		dev_attr_tsp_threshold.attr.name);

	// TSP Command Test Binary
	if (device_create_file(ts_dev, &dev_attr_tsp_name) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_name.attr.name);	
	if (device_create_file(ts_dev, &dev_attr_tsp_xy_node) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_xy_node.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_module_on) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_module_on.attr.name);	
	if (device_create_file(ts_dev, &dev_attr_tsp_module_off) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_module_off.attr.name);	
	if (device_create_file(ts_dev, &dev_attr_tsp_intensity_read) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_intensity_read.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_key_intensity_read) < 0)
		pr_err("Failed to create device file(%s)!\n",
			 dev_attr_tsp_key_intensity_read.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_key_intensity_read_result) < 0)
		pr_err("Failed to create device file(%s)!\n",
			 dev_attr_tsp_key_intensity_read_result.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_reference_read) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_reference_read.attr.name);	
	if (device_create_file(ts_dev, &dev_attr_tsp_cmdelta_read) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_cmdelta_read.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_cmdelta_read_result) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_tsp_cmdelta_read_result.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_cmabs_read) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_cmabs_read.attr.name);	
	if (device_create_file(ts_dev, &dev_attr_tsp_cmabs_read_result) < 0)
		pr_err("Failed to create device file(%s)!\n",
			 dev_attr_tsp_cmabs_read_result.attr.name);
	
	if (device_create_file(ts_dev, &dev_attr_tsp_test) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_test.attr.name);	
	if (device_create_file(ts_dev, &dev_attr_tsp_reference) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_reference.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_inspection) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_inspection.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_delta) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_delta.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_sleep) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_sleep.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_wakeup) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_wakeup.attr.name);
	if (device_create_file(ts_dev, &dev_attr_config_tsp_version) < 0)
		pr_err("Failed to create device file(%s)!\n",
					dev_attr_config_tsp_version.attr.name);
	if (device_create_file(ts_dev, &dev_attr_tsp_firm_version_config) < 0)
		pr_err("Failed to create device file(%s)!\n",
			 dev_attr_tsp_firm_version_config.attr.name);
#ifdef PREVAIL2_SYSFS_ADDED






		ts_dev1 = device_create(sec_class, NULL, 0, NULL, "tsp");
		if (IS_ERR(ts_dev1))
			pr_err("Failed to create device(tsp)!\n");


		ret = sysfs_create_group(&ts_dev1->kobj,
				       &sec_touch_factory_attr_group);
		if (ret)
			pr_err("Failed to create sysfs group\n");


#endif


#endif
	printk(KERN_DEBUG "[TSP] %s 2\n", __func__);
	melfas_ts = kzalloc(sizeof(struct melfas_ts_driver), GFP_KERNEL);
	if(melfas_ts == NULL) {
		return -ENOMEM;
	}
#ifdef TSP_TEST_MODE
	mutex_init(&melfas_ts->lock);
#endif

	ret = i2c_add_driver(&melfas_ts_i2c);
	if(ret) printk("[TSP] [%s], i2c_add_driver failed...(%d)\n", __func__, ret);

	if(!melfas_ts->client) {
		printk("###################################################\n");
		printk("##                                               ##\n");
		printk("##    WARNING! TOUCHSCREEN DRIVER CAN'T WORK.    ##\n");
		printk("##    PLEASE CHECK YOUR TOUCHSCREEN CONNECTOR!   ##\n");
		printk("##                                               ##\n");
		printk("###################################################\n");
		i2c_del_driver(&melfas_ts_i2c);
		return 0;
	}
	melfas_ts_wq = create_singlethread_workqueue("melfas_ts_wq");
	if (!melfas_ts_wq)
		return -ENOMEM;
	printk("[TSP] %s 3 \n",__func__);

	return platform_driver_register(&melfas_ts_driver);

}

void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_i2c);
	if (melfas_ts_wq)
		destroy_workqueue(melfas_ts_wq);
}
//late_initcall(melfas_ts_init);
module_init(melfas_ts_init);  // for firmware update
module_exit(melfas_ts_exit);

MODULE_DESCRIPTION("Melfas Touchscreen Driver");
MODULE_LICENSE("GPL");

