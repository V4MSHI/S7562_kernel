/* Copyright (c) 2011, Code GEIM Forum. All rights reserved.
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

/* 1/5" sr030pc50*/

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/pmic.h>

#include <mach/camera.h>
#include <mach/vreg.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>

#undef SENSOR_DEBUG
/* #define SENSOR_DEBUG */

#define FACTORY_TEST 1

#include "sr030pc50.h"
#include "s5k4ecgx_kyle.h"

/*#define CONFIG_LOAD_FILE*/

#define PCAM_ENABLE_DEBUG

#ifdef PCAM_ENABLE_DEBUG
#define CAMDRV_DEBUG(fmt, arg...)\
	do {					\
		pr_info("[sr030pc50_DEBUG] %s:%d:" fmt "\n",	\
			__func__, __LINE__, ##arg);		\
	}							\
	while (0)
#else
#define CAMDRV_DEBUG(fmt, arg...)
#endif

#ifdef CONFIG_LOAD_FILE
#define sr030pc50_WRITE_LIST(A)\
	sr030pc50_sensor_write_list(A, (sizeof(A) / sizeof(A[0])), #A);
#define sr030pc50_WRITE_LIST_BURST(A)\
	sr030pc50_sensor_burst_write_list(A, (sizeof(A) / sizeof(A[0])), #A);
#else
#define sr030pc50_WRITE_LIST(A)\
	sr030pc50_sensor_write_list(A, (sizeof(A) / sizeof(A[0])), #A);
#define sr030pc50_WRITE_LIST_BURST(A)\
	sr030pc50_sensor_burst_write_list(A, (sizeof(A) / sizeof(A[0])), #A);
#endif

#define UPTO_MSEC 1000

static char first_start_camera = 1;	/*  1 is not init a sensor */
static char set_init0;

static char mEffect = EXT_CFG_EFFECT_NORMAL;
static char mBrightness = EXT_CFG_BR_STEP_0;
static char mContrast = EXT_CFG_CR_STEP_0;
static char mSaturation = EXT_CFG_SA_STEP_0;
static char mSharpness = EXT_CFG_SP_STEP_0;
static char mWhiteBalance = EXT_CFG_WB_AUTO;
static char mISO = EXT_CFG_ISO_AUTO;
static char mAutoExposure = EXT_CFG_METERING_NORMAL;
static char mScene = EXT_CFG_SCENE_OFF;
static char mCameraMode = EXT_CFG_CAMERA_MODE;
static char mVTCallMode;
static int prev_vtcall_mode = -1;
static char mDTP;
static char mInit;
static int Flipmode;

struct sr030pc50_work {
	struct work_struct work;
};

static struct sr030pc50_work *sr030pc50_sensorw;
static struct i2c_client *sr030pc50_client;

struct sr030pc50_ctrl {
	const struct msm_camera_sensor_info *sensordata;
};

static unsigned int config_csi2;
static struct sr030pc50_ctrl *sr030pc50_ctrl;
static void sr030pc50_set_power(int status);
static DECLARE_WAIT_QUEUE_HEAD(sr030pc50_wait_queue);
DECLARE_MUTEX(sr030pc50_sem);

static void sr030pc50_reset_power(void);
static int sr030pc50_set_preview(void);

#ifdef CONFIG_LOAD_FILE
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>

static char *sr030pc50_regs_table;

static int sr030pc50_regs_table_size;

static int sr030pc50_regs_table_write(char *name);
static int sr030pc50_regs_table_burst_write(char *name);

#define MAX_SETTING_NAME 30
#define TUNNING_FILE_PATH "/mnt/sdcard/sr030pc50.h"

#endif

#define BURST_MODE_BUFFER_MAX_SIZE 255
#define BURST_REG 0x0e
#define DELAY_REG 0xff
unsigned char sr030pc50_buf_for_burstmode[BURST_MODE_BUFFER_MAX_SIZE];

int sr030pc50_camera_antibanding = CAMERA_ANTIBANDING_50HZ; /* default */

int sr030pc50_camera_antibanding_get (void) {
	return sr030pc50_camera_antibanding;
}

ssize_t sr030pc50_camera_antibanding_show (struct device *dev, struct device_attribute *attr, char *buf) {
	int count;

	count = sprintf(buf, "%d", sr030pc50_camera_antibanding);
	pr_err("%s : sr030pc50_camera_antibanding is %d\n",__func__, sr030pc50_camera_antibanding);

	return count;
}

ssize_t sr030pc50_camera_antibanding_store (struct device *dev, struct device_attribute *attr, const char *buf, size_t size) {
	int tmp = 0;

	sscanf(buf, "%d", &tmp);
	if ((CAMERA_ANTIBANDING_50HZ == tmp) || (CAMERA_ANTIBANDING_60HZ == tmp)) {
		sr030pc50_camera_antibanding = tmp;
		pr_err("%s : sr030pc50_camera_antibanding is %d\n",__func__, sr030pc50_camera_antibanding);
	}

	return size;
}

static struct device_attribute sr030pc50_camera_antibanding_attr = {
	.attr = {
		.name = "anti-banding",
		.mode = (S_IRUSR|S_IRGRP | S_IWUSR|S_IWGRP)},
	.show = sr030pc50_camera_antibanding_show,
	.store = sr030pc50_camera_antibanding_store
};

static int sr030pc50_sensor_read(unsigned char subaddr, unsigned char *data)
{
	int ret = 0;
	unsigned char buf[1] = { 0 };
	struct i2c_msg msg = { sr030pc50_client->addr, 0, 1, buf };

#ifdef SENSOR_DEBUG
	pr_err("[ 0x%x ]\n", subaddr);
#endif

	buf[0] = (subaddr & 0xFF);

	ret = i2c_transfer(sr030pc50_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) {
		pr_err("[%s : %d] i2c_transfer fail\n", __func__, __LINE__);
		goto error;
	}

	msg.flags = I2C_M_RD;

	ret = i2c_transfer(sr030pc50_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO) {
		pr_err("[%s : %d] i2c_transfer fail\n", __func__, __LINE__);
		goto error;
	}

	*data = buf[0];		/*check */

error:
	return ret;
}

static int sr030pc50_sensor_write(unsigned char subaddr, unsigned char val)
{
	unsigned char buf[2] = { 0 };
	struct i2c_msg msg = { sr030pc50_client->addr, 0, 2, buf };

#ifdef SENSOR_DEBUG
	pr_err("[ 0x%x %x ]\n", subaddr, buf);
#endif

	buf[0] = (subaddr);
	buf[1] = (val);

	return i2c_transfer(sr030pc50_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
}

/*static int sr030pc50_sensor_write_list
(struct samsung_short_t *list,int size, char *name)*/
static int sr030pc50_sensor_write_list(const u16 *list, int size, char *name)
{
	int ret = 0;
	unsigned char subaddr = 0;
	unsigned char value = 0;
	int i = 0;
	CAMDRV_DEBUG("%s\n", name);

#ifdef CONFIG_LOAD_FILE
	ret = sr030pc50_regs_table_write(name);
#else

	for (i = 0; i < size; i++) {
		subaddr = (list[i] >> 8);	/*address */
		value = (list[i] & 0xFF);	/*value */

		if (subaddr == DELAY_REG) {
			msleep(value * 10);	/*one step is 10ms */
			CAMDRV_DEBUG("[sr030pc50] msleep %d msec\n",
				     value * 10);
		} else {
			if (sr030pc50_sensor_write(subaddr, value) < 0) {
				pr_err
				    ("[%s : %d] sr030pc50_sensor_write fail\n",
				     __func__, __LINE__);
				return -EIO;
			}
		}
	}
#endif
	return ret;
}

static int sr030pc50_sensor_burst_write_list(const u16 *list, int size,
					     char *name)
{
	int err = -EINVAL;
	int i = 0;
	int idx = 0;
	unsigned short subaddr = 0;
	unsigned short value = 0;
	int burst_flag = 0;
	int burst_cnt = 0;
	struct i2c_msg msg = { sr030pc50_client->addr,
		0, 0, sr030pc50_buf_for_burstmode
	};

	CAMDRV_DEBUG("%s, size = %d\n", name, size);

#ifdef CONFIG_LOAD_FILE
	err = sr030pc50_regs_table_burst_write(name);
#else
	for (i = 0; i < size; i++) {
		if (idx > (BURST_MODE_BUFFER_MAX_SIZE - 10)) {
			pr_err("[%s:%d]Burst mode buffer overflow! "
			       "Burst Count %d\n",
			       __func__, __LINE__, burst_cnt);
			pr_err("[%s:%d]count %d, addr %x "
			       "value %x\n", __func__, __LINE__, i,
			       (list[i] >> 8) & 0xff, list[i] & 0xFF);
			pr_err("[%s:%d]addr %x value %x\n",
			       __func__, __LINE__,
			       (list[i - 1] >> 8) & 0xff, list[i - 1] & 0xFF);
			pr_err("[%s:%d]addr %x value %x\n",
			       __func__, __LINE__,
			       (list[i - 2] >> 8) & 0xff, list[i - 2] & 0xFF);
			err = -EIO;
			return err;
		}
		subaddr = (list[i] >> 8);
		value = (list[i] & 0xFF);
		if (burst_flag == 0) {
			switch (subaddr) {
			case BURST_REG:
				if (value != 0x00) {
					burst_flag = 1;
					burst_cnt++;
				}
				break;
			case DELAY_REG:
				msleep(value * 10);	/* a step is 10ms */
				break;
			default:
				idx = 0;
				err = sr030pc50_sensor_write(subaddr, value);
				break;
			}
		} else if (burst_flag == 1) {
			if (subaddr == BURST_REG && value == 0x00) {
				msg.len = idx;
				CAMDRV_DEBUG("burst_cnt %d, idx %d\n",
					     burst_cnt, idx);
				err = i2c_transfer(sr030pc50_client->adapter,
						   &msg, 1);
				if (err < 0) {
					pr_err("[%s:%d]Burst write fail!\n",
					       __func__, __LINE__);
					return err;
				}
				idx = 0;
				burst_flag = 0;
			} else {
				if (idx == 0) {
					sr030pc50_buf_for_burstmode[idx++] =
					    subaddr;
					sr030pc50_buf_for_burstmode[idx++] =
					    value;
				} else {
					sr030pc50_buf_for_burstmode[idx++] =
					    value;
				}
			}
		}
	}
#endif

	if (unlikely(err < 0)) {
		pr_err("[%s:%d] register set failed\n", __func__, __LINE__);
		return err;
	}

	return err;
}

static int sr030pc50_effect_control(char value)
{
	int err = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", value);

	switch (value) {
	case EXT_CFG_EFFECT_NORMAL:{
			err = sr030pc50_WRITE_LIST(sr030pc50_Effect_Normal);
		}
		break;

	case EXT_CFG_EFFECT_NEGATIVE:{
			err = sr030pc50_WRITE_LIST(sr030pc50_Effect_Negative);
		}
		break;

	case EXT_CFG_EFFECT_MONO:{
			err = sr030pc50_WRITE_LIST(sr030pc50_Effect_Gray);
		}
		break;

	case EXT_CFG_EFFECT_SEPIA:{
			err = sr030pc50_WRITE_LIST(sr030pc50_Effect_Sepia);
		}
		break;

	default:{
			pr_warning(" Unexpected Effect mode :" "%d\n", value);
		}
		break;
	}

	if (err != 0)
		pr_err("[%s : %d] %d fail\n", __func__, __LINE__, value);
	else
		CAMDRV_DEBUG("Exit!!\n");

	return err;
}

static int sr030pc50_whitebalance_control(char value)
{
	int err = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", value);

	switch (value) {
	case EXT_CFG_WB_AUTO:{
			err = sr030pc50_WRITE_LIST(sr030pc50_WB_Auto);
		}
		break;

	case EXT_CFG_WB_DAYLIGHT:{
			err = sr030pc50_WRITE_LIST(sr030pc50_WB_Daylight);
		}
		break;

	case EXT_CFG_WB_CLOUDY:{
			err = sr030pc50_WRITE_LIST(sr030pc50_WB_Cloudy);
		}
		break;

	case EXT_CFG_WB_FLUORESCENT:{
			err = sr030pc50_WRITE_LIST(sr030pc50_WB_Fluorescent);
		}
		break;

	case EXT_CFG_WB_INCANDESCENT:{
			err = sr030pc50_WRITE_LIST(sr030pc50_WB_Incandescent);
		}
		break;

	default:{
			pr_warning(" Unexpected WB mode : %d\n", value);
		}
		break;

	}

	if (err != 0)
		pr_err("[%s : %d] %d fail\n", __func__, __LINE__, value);
	else
		CAMDRV_DEBUG("Exit!!\n");

	return err;
}

static int sr030pc50_brightness_control(char vtmode, char value)
{

	int err = 0;

	CAMDRV_DEBUG("Enter [vtmode = %d][value = %d]\n", vtmode, value);
	if (vtmode) {	/* mVTCallMode == 1, VT Mode */
		switch (value) {
		case EXT_CFG_BR_STEP_P_4:{
				err = sr030pc50_WRITE_LIST(
				sr030pc50_VT_brightness_p_4);
			}
			break;

		case EXT_CFG_BR_STEP_P_3:{
				err = sr030pc50_WRITE_LIST(
				sr030pc50_VT_brightness_p_3);
			}
			break;

		case EXT_CFG_BR_STEP_P_2:{
				err = sr030pc50_WRITE_LIST(
				sr030pc50_VT_brightness_p_2);
			}
			break;

		case EXT_CFG_BR_STEP_P_1:{
				err = sr030pc50_WRITE_LIST(
				sr030pc50_VT_brightness_p_1);
			}
			break;

		case EXT_CFG_BR_STEP_0:{
				err = sr030pc50_WRITE_LIST(
				sr030pc50_VT_brightness_0);
			}
			break;

		case EXT_CFG_BR_STEP_M_1:{
				err = sr030pc50_WRITE_LIST(
				sr030pc50_VT_brightness_m_1);
			}
			break;

		case EXT_CFG_BR_STEP_M_2:{
				err = sr030pc50_WRITE_LIST(
				sr030pc50_VT_brightness_m_2);
			}
			break;

		case EXT_CFG_BR_STEP_M_3:{
				err = sr030pc50_WRITE_LIST(
				sr030pc50_VT_brightness_m_3);
			}
			break;

		case EXT_CFG_BR_STEP_M_4:{
				err = sr030pc50_WRITE_LIST(
				sr030pc50_VT_brightness_m_4);
			}
			break;

		default:{
				pr_warning(" Unexpected BR mode : %d\n", value);
			}
			break;

		}
	} else {	/* mVTCallMode == 0, normal Mode */
		switch (value) {
			case EXT_CFG_BR_STEP_P_4:{
					err = sr030pc50_WRITE_LIST(
					sr030pc50_brightness_p_4);
				}
				break;

			case EXT_CFG_BR_STEP_P_3:{
					err = sr030pc50_WRITE_LIST(
					sr030pc50_brightness_p_3);
				}
				break;

			case EXT_CFG_BR_STEP_P_2:{
					err = sr030pc50_WRITE_LIST(
					sr030pc50_brightness_p_2);
				}
				break;

			case EXT_CFG_BR_STEP_P_1:{
					err = sr030pc50_WRITE_LIST(
					sr030pc50_brightness_p_1);
				}
				break;

			case EXT_CFG_BR_STEP_0:{
					err = sr030pc50_WRITE_LIST(
					sr030pc50_brightness_0);
				}
				break;

			case EXT_CFG_BR_STEP_M_1:{
					err = sr030pc50_WRITE_LIST(
					sr030pc50_brightness_m_1);
				}
				break;

			case EXT_CFG_BR_STEP_M_2:{
					err = sr030pc50_WRITE_LIST(
					sr030pc50_brightness_m_2);
				}
				break;

			case EXT_CFG_BR_STEP_M_3:{
					err = sr030pc50_WRITE_LIST(
					sr030pc50_brightness_m_3);
				}
				break;

			case EXT_CFG_BR_STEP_M_4:{
					err = sr030pc50_WRITE_LIST(
					sr030pc50_brightness_m_4);
				}
				break;

			default:{
					pr_warning(" Unexpected BR mode : %d\n",
					value);
				}
				break;
		}
	}

	if (err != 0)
		pr_err("[%s : %d] %d fail\n", __func__, __LINE__, value);
	else
		CAMDRV_DEBUG("Exit!!\n");

	return err;
}

static int sr030pc50_iso_control(char value)
{
	int err = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", value);

	switch (value) {
	case EXT_CFG_ISO_AUTO:{
			err = sr030pc50_WRITE_LIST(sr030pc50_iso_auto);
		}
		break;

	case EXT_CFG_ISO_50:{
			err = sr030pc50_WRITE_LIST(sr030pc50_iso_50);
		}
		break;

	case EXT_CFG_ISO_100:{
			err = sr030pc50_WRITE_LIST(sr030pc50_iso_100);
		}
		break;

	case EXT_CFG_ISO_200:{
			err = sr030pc50_WRITE_LIST(sr030pc50_iso_200);
		}
		break;

	case EXT_CFG_ISO_400:{
			err = sr030pc50_WRITE_LIST(sr030pc50_iso_400);
		}
		break;

	default:{
			pr_warning(" Unexpected ISO mode : %d\n", value);
		}
		break;

	}

	if (err != 0)
		pr_err("[%s : %d] %d fail\n", __func__, __LINE__, value);
	else
		CAMDRV_DEBUG("Exit!!\n");

	return err;
}

static int sr030pc50_metering_control(char value)
{
	int err = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", value);

	return err;
}

static int sr030pc50_scene_control(char value)
{
	int err = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", value);

	return err;
}

static int sr030pc50_contrast_control(char value)
{
	int err = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", value);

	return err;
}

static int sr030pc50_saturation_control(char value)
{
	int err = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", value);

	return err;
}

static int sr030pc50_sharpness_control(char value)
{
	int err = 0;

	CAMDRV_DEBUG("Enter [value = %d]\n", value);

	return err;
}

static int sr030pc50_DTP_control(char value)
{
	int err = 0;

	CAMDRV_DEBUG("[%s]\n", value ? "DTP ON" : "DTP OFF");

	switch (value) {
	case EXT_CFG_DTP_OFF:{
			err = sr030pc50_WRITE_LIST(sr030pc50_dtp_off);
			first_start_camera = 1;
			/*sr030pc50_set_preview(); */
		}
		break;

	case EXT_CFG_DTP_ON:{
			err = sr030pc50_WRITE_LIST(sr030pc50_dtp_on);
		}
		break;

	default:{
			pr_warning(" unexpected DTP control\n");
		}
		break;
	}

	if (err != 0)
		pr_err("[%s : %d] %d fail\n", __func__, __LINE__, value);
	else
		CAMDRV_DEBUG("Exit!!\n");

	return err;
}

static int sr030pc50_set_flipmode(int val)
{
	CAMDRV_DEBUG("Enter [value = %d]", val);
	Flipmode = val;
	return 0;
}

#ifdef FACTORY_TEST
struct device *sr030pc50_dev;

static ssize_t front_camera_type_show(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	char cam_type[] = "SF_SR030PC50_NONE\n";

	return snprintf(buf, sizeof(cam_type), "%s", cam_type);
}

static ssize_t front_camera_firmware_show(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{

	char cam_fw[] = "N\n";

	return snprintf(buf, sizeof(cam_fw), "%s", cam_fw);
}

static DEVICE_ATTR(front_camtype, S_IRUGO, front_camera_type_show, NULL);
static DEVICE_ATTR(front_camfw, 0664, front_camera_firmware_show, NULL);
#endif

static int sr030pc50_sensor_ext_config(void __user *arg)
{
	int err = 0;
	ioctl_pcam_info_8bit ctrl_info;

	if (copy_from_user
	    ((void *)&ctrl_info, (const void *)arg, sizeof(ctrl_info))) {
		pr_err(" %s fail copy_from_user!\n", __func__);
	}

	CAMDRV_DEBUG("%d %d %d %d %d\n",
		     ctrl_info.mode, ctrl_info.address, ctrl_info.value_1,
		     ctrl_info.value_2, ctrl_info.value_3);

	if (mScene != EXT_CFG_SCENE_OFF) {
		if (ctrl_info.mode == EXT_CFG_EFFECT_CONTROL ||
		    ctrl_info.mode == EXT_CFG_WB_CONTROL ||
		    ctrl_info.mode == EXT_CFG_BR_CONTROL ||
		    ctrl_info.mode == EXT_CFG_ISO_CONTROL ||
		    ctrl_info.mode == EXT_CFG_METERING_CONTROL ||
		    ctrl_info.mode == EXT_CFG_CR_CONTROL ||
		    ctrl_info.mode == EXT_CFG_SA_CONTROL ||
		    ctrl_info.mode == EXT_CFG_SP_CONTROL) {
			pr_warn("[%s : %d]It doesn't accept this control "
				"because now scene mode is set!!\n",
				__func__, __LINE__);
			pr_warn("[%s : %d]This mode is %d\n\n",
				__func__, __LINE__, ctrl_info.mode);
			return err;
		}
	}

	switch (ctrl_info.mode) {

	case EXT_CFG_GET_INFO: {
/* EXT_CFG_GET_INFO cmd means driver transfers sensor information*/
/* to upper layer. In case of this exposure time and ISO */
		unsigned short a = 0, b = 0;
		unsigned short c = 0, iso_gain = 0;
		unsigned char data = 0;

		/* page mode 0x20 */
		sr030pc50_sensor_write(0x03, 0x20);

		/* read exposure time */
		sr030pc50_sensor_read(0x80, &data);
		a = (unsigned short)data;
		sr030pc50_sensor_read(0x81, &data);
		b = (unsigned short)data;
		sr030pc50_sensor_read(0x82, &data);
		c = (unsigned short)data;

		/* read ISO gain */
		sr030pc50_sensor_read(0xb0, &data);
		iso_gain = (unsigned short)data;

		ctrl_info.value_1 = a;
		ctrl_info.value_2 = b;
		ctrl_info.value_3 = c;
		ctrl_info.address = iso_gain;

		CAMDRV_DEBUG("exposure %x\n",
			(a << 19) | (b << 11) | (c << 3));
		CAMDRV_DEBUG("ISO gain is %x\n", iso_gain);
		}
		break;
	case EXT_CFG_FRAME_CONTROL:
		switch (ctrl_info.value_1) {

		case EXT_CFG_FRAME_FIX_15:
			if (CAMERA_ANTIBANDING_60HZ ==
				sr030pc50_camera_antibanding_get()) {
					err = sr030pc50_WRITE_LIST
						(sr030pc50_15_fps_60hz);
			} else {
				err = sr030pc50_WRITE_LIST
					(sr030pc50_15_fps);
			}
			break;

		case EXT_CFG_FRAME_FIX_24:
			if (CAMERA_ANTIBANDING_60HZ ==
				sr030pc50_camera_antibanding_get()) {
					err = sr030pc50_WRITE_LIST
						(sr030pc50_20_fps_60hz)
			} else {
				err = sr030pc50_WRITE_LIST
					(sr030pc50_20_fps)
			}
			break;

		case EXT_CFG_FRAME_FIX_30:
			if (CAMERA_ANTIBANDING_60HZ ==
				sr030pc50_camera_antibanding_get()) {
					err = sr030pc50_WRITE_LIST
						(sr030pc50_30_fps_60hz);
			} else {
				err = sr030pc50_WRITE_LIST
					(sr030pc50_30_fps);
			}
			break;

		default:
			pr_warning
			    (" Unexpected"
			     "EXT_CFG_FRAME_CONTROL mode :"
			     "%d\n", ctrl_info.value_1);
			break;

		}
		break;

	case EXT_CFG_EFFECT_CONTROL:
		mEffect = ctrl_info.value_1;
		err = sr030pc50_effect_control(mEffect);
		break;

	case EXT_CFG_WB_CONTROL:
		mWhiteBalance = ctrl_info.value_1;
		err = sr030pc50_whitebalance_control(mWhiteBalance);
		break;

	case EXT_CFG_BR_CONTROL:
		mBrightness = ctrl_info.value_1;
		if (mInit)
			err = sr030pc50_brightness_control(
				mVTCallMode, mBrightness);
		break;

	case EXT_CFG_ISO_CONTROL:
		mISO = ctrl_info.value_1;
		err = sr030pc50_iso_control(mISO);
		break;

	case EXT_CFG_METERING_CONTROL:
		mAutoExposure = ctrl_info.value_1;
		err = sr030pc50_metering_control(mAutoExposure);
		break;

	case EXT_CFG_SCENE_CONTROL:
		mScene = ctrl_info.value_1;
		err = sr030pc50_scene_control(mScene);
		break;

	case EXT_CFG_CR_CONTROL:
		mContrast = ctrl_info.value_1;
		if (mInit)
			err = sr030pc50_contrast_control(mContrast);
		break;

	case EXT_CFG_SA_CONTROL:
		mSaturation = ctrl_info.value_1;
		if (mInit)
			err = sr030pc50_saturation_control(mSaturation);
		break;

	case EXT_CFG_SP_CONTROL:
		mSharpness = ctrl_info.value_1;
		if (mInit)
			err = sr030pc50_sharpness_control(mSharpness);
		break;

	case EXT_CFG_GET_MODULE_STATUS: {
		unsigned short id = 0;	/*CAM FOR FW */
		/*ctrl_info.value_3 = gpio_get_value(0); */

		ctrl_info.value_3 = id;

		CAMDRV_DEBUG
		    (" check current module status : %x\n",
		     ctrl_info.value_3);
		CAMDRV_DEBUG(" PINON/OFF : %d\n", gpio_get_value(0));
		}
		break;

	case EXT_CFG_DTP_CONTROL:
		if (mInit == 0) {
			if (ctrl_info.value_1 == 0)
				ctrl_info.value_3 = 2;

			else if (ctrl_info.value_1 == 1)
				ctrl_info.value_3 = 3;

			mDTP = 1;
		} else {
			err = sr030pc50_DTP_control(ctrl_info.value_1);

			if (ctrl_info.value_1 == 0)
				ctrl_info.value_3 = 2;

			else if (ctrl_info.value_1 == 1)
				ctrl_info.value_3 = 3;

			mDTP = 0;
		}
		break;

	case EXT_CFG_SET_CAM_MODE:
		mCameraMode = ctrl_info.value_1;
		if (mCameraMode == EXT_CFG_CAMCORDER_MODE) {
			pr_err("[%s : %d] Record is starting!!\n",
			       __func__, __LINE__);
		} else {
			pr_err("[%s : %d] Record stopped!!\n",
			       __func__, __LINE__);
		}
		break;

	case EXT_CFG_VT_MODE_CONTROL:
		mVTCallMode = ctrl_info.value_1;
		pr_err("[%s : %d] EXT_CFG_VT_MODE_CONTROL is Setting at [%d] !!\n",
			   __func__, __LINE__, mVTCallMode);
		break;

	case EXT_CFG_SET_FLIP:
		sr030pc50_set_flipmode(ctrl_info.value_1);
		break;

	default:
		pr_warning
		    (" Unexpected mode on sensor_rough_control :"
		     "%d\n", ctrl_info.mode);
		break;
	}

	if (copy_to_user
	    ((void *)arg, (const void *)&ctrl_info, sizeof(ctrl_info))) {
		CAMDRV_DEBUG(" %s fail on copy_to_user!\n", __func__);
	}

	if (err != 0)
		CAMDRV_DEBUG("[%s : %d] fail %d %d %d %d %d\n", __func__,
		       __LINE__, ctrl_info.mode, ctrl_info.address,
		       ctrl_info.value_1, ctrl_info.value_2, ctrl_info.value_3);

	return err;
}

static int sr030pc50_mipi_mode(int mode)
{
	int rc = 0;
	struct msm_camera_csi_params sr030pc50_csi_params;

	CAMDRV_DEBUG("Enter!!\n");

	if (!config_csi2) {
		sr030pc50_csi_params.lane_cnt = 1;
		sr030pc50_csi_params.data_format = CSI_8BIT;
		sr030pc50_csi_params.lane_assign = 0xe4;
		sr030pc50_csi_params.dpcm_scheme = 0;
		sr030pc50_csi_params.settle_cnt = 0x14;	/*lyon.cho 24->0x14 */
		rc = msm_camio_csi_config(&sr030pc50_csi_params);
		if (rc < 0) {
			pr_err("config csi controller failed\n");
			return -EIO;
		}
		config_csi2 = 1;
	}
	CAMDRV_DEBUG("Exit!!\n");
	return rc;
}

static int sr030pc50_set_preview(void)
{
	int err = 0;
	int first_awb_delay = 0;

	CAMDRV_DEBUG("mDTP = %d\n", mDTP);

	err = sr030pc50_mipi_mode(1);
	if (err != 0) {
		CAMDRV_DEBUG("[%s : %d] sr030pc50_mipi_mode fail\n", __func__,
		       __LINE__);
		return -EIO;
	}

	msleep(30);		/*=> Please add some delay*/

	if (prev_vtcall_mode == mVTCallMode) {
		CAMDRV_DEBUG("[%s : %d]Same setting before and now\n",
			__func__, __LINE__);
		sr030pc50_WRITE_LIST(sr030pc50_update_preview_setting);
		return;

	}

	if (mDTP == 1) {
		err = sr030pc50_WRITE_LIST(sr030pc50_dtp_on);
	} else {
		if (mVTCallMode) {	/* mVTCallMode == 1, VT Mode */
			if (CAMERA_ANTIBANDING_60HZ == sr030pc50_camera_antibanding_get()) {
				err = sr030pc50_WRITE_LIST(sr030pc50_VT_init_reg_60hz);
			} else {
			err = sr030pc50_WRITE_LIST(sr030pc50_VT_init_reg);
			}
			if (err != 0) {
				CAMDRV_DEBUG(
					"[%s : %d] sr030pc50_update_preview_setting failed\n",
					 __func__, __LINE__);
				return -EIO;
			}
			CAMDRV_DEBUG(
				"sr030pc50_update_preview_setting is writing done\n");

		} else {	/* mVTCallMode == 0, Normal Mode */
				if (CAMERA_ANTIBANDING_60HZ == sr030pc50_camera_antibanding_get()) {
					err = sr030pc50_WRITE_LIST(sr030pc50_init_reg_60hz);
				} else {
			err = sr030pc50_WRITE_LIST(sr030pc50_init_reg);
				}
			if (err != 0) {
				CAMDRV_DEBUG(
					"[%s : %d] sr030pc50_update_preview_setting failed\n",
					 __func__, __LINE__);
				return -EIO;
			}
			CAMDRV_DEBUG(
				"sr030pc50_update_preview_setting is writing done\n");

		}

		msleep(20);
		first_start_camera = 0;
		mInit = 1;
		first_awb_delay = 1;

		CAMDRV_DEBUG(
			"[%s:%d] Preivew setting is done!!",
			__func__, __LINE__);
	}

	prev_vtcall_mode = mVTCallMode;

	if (err != 0)
		pr_err("[%s : %d] fail\n", __func__, __LINE__);
	else
		CAMDRV_DEBUG("Exit!!\n");

	return err;
}

static int sr030pc50_set_capture(void)
{
	int err = 0;

	CAMDRV_DEBUG("Enter!! Flip mode = %d", Flipmode);

	if (Flipmode)
		err = sr030pc50_WRITE_LIST(sr030pc50_snapshot_X_Flip)
	else
		err = sr030pc50_WRITE_LIST(sr030pc50_snapshot)

	if (err != 0)
		pr_err("[%s : %d] fail\n", __func__, __LINE__);
	else
		CAMDRV_DEBUG("Exit!!\n");

	return err;
}

static long sr030pc50_set_sensor_mode(int mode)
{
	int err = 0;

	CAMDRV_DEBUG("Enter [mode = %d]\n", mode);

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		err = sr030pc50_set_preview();
		break;
	case SENSOR_SNAPSHOT_MODE:
		err = sr030pc50_set_capture();
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		CAMDRV_DEBUG("RAW_SNAPSHOT NOT SUPPORT!!");
		break;
	default:
		return -EINVAL;
	}

	if (err != 0) {
		CAMDRV_DEBUG(
			"[%s : %d] mode is %d fail\n",
			__func__, __LINE__, mode);
	} else
		CAMDRV_DEBUG("Exit!!\n");

	return err;
}

static void sr030pc50_reset_power(void)
{
	CAMDRV_DEBUG("[%s : %d] entered\n", __func__, __LINE__);
	sr030pc50_set_power(0);
	/*add delay */
	usleep(10 * UPTO_MSEC);	/* 10 msec */
	sr030pc50_set_power(1);
	CAMDRV_DEBUG("[%s : %d] exit\n", __func__, __LINE__);
}

void sr030pc50_set_power(int status)
{
	static struct regulator *vreg_ldo17;
	int ret;
	
	CAMDRV_DEBUG("%s\n", status ? "POWER ON" : "POWER OFF");

	if (status == 1) {
		CAMDRV_DEBUG("sr030pc50_set_power ON!!\n");

#ifndef CONFIG_MACH_KYLE
	cam_ldo_power_on();
#endif

	/* 
	* This Code is Power Sequence by Dual Booting 5MP and 1.3MP
	*/

	/********************************************/
	/* * Step1. GPIO, MCLK, LDO Default setting 		*/
	/********************************************/

	gpio_tlmm_config(GPIO_CFG	/*CAM_MEGA_RESET*/
			(CAM_MEGA_RST, 0, GPIO_CFG_OUTPUT,
			  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			 GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG	/*CAM_MEGA_STBY*/
			 (CAM_MEGA_STBY, 0, GPIO_CFG_OUTPUT,
			  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			 GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG	/*CAM_VGA_RESET*/
			(CAM_VGA_RST, 0, GPIO_CFG_OUTPUT,
			  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			 GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG	/*CAM_VGA_STBY*/
			 (CAM_VGA_STBY, 0, GPIO_CFG_OUTPUT,
			  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			 GPIO_CFG_ENABLE);

	gpio_tlmm_config(GPIO_CFG	/*VCAM_IO_1.8v*/
			 (CAM_IO_EN, 0, GPIO_CFG_OUTPUT,
			  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			 GPIO_CFG_ENABLE);

	gpio_tlmm_config(GPIO_CFG	/*VCAM_C_1.2v*/
			 (CAM_CORE_1_2, 0, GPIO_CFG_OUTPUT,
			  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			 GPIO_CFG_ENABLE);

	gpio_set_value(CAM_MEGA_RST, 0);
	gpio_set_value(CAM_MEGA_STBY, 0);
	gpio_set_value(CAM_VGA_RST, 0); 	
	gpio_set_value(CAM_VGA_STBY, 0);		
	gpio_set_value(CAM_IO_EN, 0);
	gpio_set_value(CAM_CORE_1_2, 0);

	gpio_tlmm_config(GPIO_CFG	/*CAM_MCLK*/
			 (15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
			  GPIO_CFG_16MA), GPIO_CFG_DISABLE);
	msm_camio_clk_rate_set(24000000);
	mdelay(2);	/* >10us (temp) */

	/* * Step2. Power Sequence Start. AVDD Power On */
	vreg_ldo17 = regulator_get(NULL, "ldo17");	/*VCAM_A_2_8v*/
	if (IS_ERR(vreg_ldo17)) {
	//if (!vreg_ldo17) {
		CAMDRV_DEBUG("%s: VREG L17 get failed\n",
		__func__);
		}

	ret = regulator_set_voltage(vreg_ldo17,2800000,2800000);
	if (ret) {
	//if (vreg_set_level(vreg_ldo17, 2800)) {
		CAMDRV_DEBUG("%s: vreg_set_level failed\n",
		__func__);
		}
	ret = regulator_enable(vreg_ldo17);
	if (ret) {
		CAMDRV_DEBUG("%s: reg_enable failed\n",
		__func__);
		}
	udelay(10);

	/* * Step3. Power Sequence Start. IO, VT Core Power On	*/

	gpio_set_value_cansleep(CAM_IO_EN, 1);	/* VCAM_IO_1_8v */
	udelay(100);

	/********************************************/
	/* * Step4. Poer Sequence Start. Core Power On	*/
	/********************************************/

	gpio_set_value_cansleep(CAM_CORE_1_2, 1);	/*VCAM_Core_1_2v*/
	mdelay(2);	/* >1ms */

	/********************************************/
	/* * Step5. Poer Sequence Start. Core Power Off	*/
	/********************************************/

	gpio_set_value_cansleep(CAM_CORE_1_2, 0);	/*VCAM_Core_1_2v*/
	mdelay(7);	/* >1ms */

	/* * Step6. CTRL Sequence Start. MCLK Enable */

		msm_camio_clk_rate_set(24000000);	/* 24 Mhz */
		udelay(10);
		msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
		udelay(100);

	/********************************************/
	/* * Step7. CTRL Sequence Start. VGA StanBy High	*/
	/********************************************/

	gpio_set_value_cansleep(CAM_VGA_STBY, 1);	/* VGA_STBY */
	mdelay(3);

	/* * Step8. CTRL Sequence Start. VGA RST High */

	gpio_set_value_cansleep(CAM_VGA_RST, 1);	/* 5M_RST */
	mdelay(50);

	}else {

	CAMDRV_DEBUG("sr030pc50_set_power OFF\n");
	
	/********************************************/
	/* * Step1. Power OFF Sequence	*/
	/********************************************/
	gpio_set_value_cansleep(CAM_VGA_RST, 0);
	udelay(10);

	gpio_set_value_cansleep(CAM_VGA_STBY, 0);
	udelay(50);

	msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
	udelay(10);

	gpio_set_value_cansleep(CAM_IO_EN, 0);
	udelay(10);
	
	regulator_disable(vreg_ldo17);	/*AVDD*/
	//vreg_disable(vreg_get(NULL, "vcama"));
	udelay(1000);

	}

}

static int sr030pc50_check_sensor_id(void)
{
	int err = 0;
	unsigned char id = 0;
	const char page_mode_big = 0x03, page_mode_little = 0x00;
	const char read_addr = 0x04;

	CAMDRV_DEBUG("Enter!!\n");

#ifdef CONFIG_MACH_KYLE
	/* read device id */
	err = sr030pc50_sensor_write(page_mode_big, page_mode_little);
	if (err != 0)
		pr_err("[%s : %d] sr030pc50_sensor_write fail\n", __func__,
		       __LINE__);

	err = sr030pc50_sensor_read(read_addr, &id);	/*address */
	if (err != 0)
		pr_err("[%s : %d] sr030pc50_sensor_read fail\n", __func__,
		       __LINE__);

	if (id != 0xb8) {
		pr_err("[sr030pc50] WRONG SENSOR FW => id 0x%x\n", id);
		err = -1;
	} else {
		CAMDRV_DEBUG("[sr030pc50] CURRENT SENSOR FW => id 0x%x\n", id);
	}
	if (err != 0)
		pr_err("[%s : %d] fail\n", __func__, __LINE__);
	else
		CAMDRV_DEBUG("Exit!!\n");
#endif

	return err;
}

#ifdef CONFIG_LOAD_FILE

static int sr030pc50_regs_table_init(void)
{
	struct file *filp;
	char *dp = NULL;
	long l = 0;
	loff_t pos;
	int ret = -1;
	mm_segment_t fs = get_fs();

	pr_err("[%s : %d] Entered\n", __func__, __LINE__);

	set_fs(get_ds());

	filp = filp_open(TUNNING_FILE_PATH, O_RDONLY, 0);

	if (IS_ERR(filp)) {
		pr_err("[%s : %d]file open error\n", __func__, __LINE__);
		return -ret;
	}
	l = filp->f_path.dentry->d_inode->i_size;
	pr_info("l = %ld\n", l);
	dp = vmalloc(l);
	if (dp == NULL) {
		pr_err("[%s : %d] Out of Memory\n", __func__, __LINE__);
		filp_close(filp, current->files);
		return ret;
	}
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	if (ret != l) {
		pr_err("[%s : %d] Failed to read file ret = %d\n",
		       __func__, __LINE__, ret);
		if (dp != NULL)
			vfree(dp);
		filp_close(filp, current->files);
		ret = -EIO;
		return ret;
	}

	filp_close(filp, current->files);

	set_fs(fs);

	sr030pc50_regs_table = dp;

	sr030pc50_regs_table_size = l;

	*((sr030pc50_regs_table + sr030pc50_regs_table_size) - 1) = '\0';

	pr_err("Exit!!\n");

	ret = 0;
	return ret;
}

void sr030pc50_regs_table_exit(void)
{
	CAMDRV_DEBUG("Enter!!\n");
	if (sr030pc50_regs_table != NULL) {
		vfree(sr030pc50_regs_table);
		sr030pc50_regs_table = NULL;
	}
	CAMDRV_DEBUG("Exit!!\n");
}

static int sr030pc50_is_hexnum(char *num)
{
	int i = 0;
	for (i = 2; num[i] != '\0'; i++) {
		if (!((num[i] >= '0' && num[5] <= '9')
		      || (num[5] >= 'a' && num[5] <= 'f') || (num[5] >= 'A'
							      && num[5] <=
							      'F'))) {
			return 0;
		}
	}
	return 1;
}

static int sr030pc50_regs_table_write(char *name)
{
	char *start = NULL, *end = NULL, *reg = NULL, *temp = NULL;
	unsigned char addr = 0, value = 0;
	unsigned short data = 0;
	char data_buf[7] = { 0 };
	int err = 0;

	CAMDRV_DEBUG("Enter!!\n");

	addr = value = 0;

	*(data_buf + 6) = '\0';

	start = strnstr(sr030pc50_regs_table, name, sr030pc50_regs_table_size);
	if (start == NULL) {
		pr_err("[%s : %d] start is NULL\n", __func__, __LINE__);
		err = -EIO;
		return err;
	}

	end = strnstr(start, "};", sr030pc50_regs_table_size);
	if (end == NULL) {
		pr_err("[%s : %d] end is NULL\n", __func__, __LINE__);
		err = -EIO;
		return err;
	}

	while (1) {
		/* Find Address */
		reg = strnstr(start, "0x", sr030pc50_regs_table_size);
		if (reg)
			start = (reg + 6);

		if ((reg == NULL) || (reg > end)) {
			pr_err("[%s : %d] write end of %s\n",
			       __func__, __LINE__, name);
			break;
		}
		/* Write Value to Address */
		memcpy(data_buf, reg, 6);

		if (sr030pc50_is_hexnum(data_buf) == 0) {
			pr_err("[%s : %d] it's not hex number %s\n",
			       __func__, __LINE__, data_buf);
			continue;
		}

		err = kstrtou16(data_buf, 16, &data);
		if (err < 0) {
			pr_err("[%s : %d] kstrtou16 failed\n",
			       __func__, __LINE__);
		}
		addr = (data >> 8);
		value = (data & 0xff);

		if (addr == 0xff) {
			msleep(value * 10);	/*one step is 10ms */
			CAMDRV_DEBUG("delay %d msec\n", value * 10);
		} else {
			if (sr030pc50_sensor_write(addr, value) < 0) {
				pr_err
				    ("[%s : %d] fail on sensor_write :"
				     "addr[0x%04x], value[0x%04x]\n",
				     __func__, __LINE__, addr, value);
				err = -EIO;
				return err;
			}
			CAMDRV_DEBUG
			    ("success on sensor_write :"
			     "addr[0x%04x], value[0x%04x]\n", addr, value);
		}
	}

	CAMDRV_DEBUG("Exit!!\n");

	return err;
}

static int sr030pc50_regs_table_burst_write(char *name)
{
	char *start = NULL, *end = NULL;
	char *reg = NULL, *temp = NULL;
	unsigned char addr = 0, value = 0;
	unsigned short data = 0;
	char data_buf[7] = { 0 };
	int idx = 0;
	int err = 0;
	int burst_flag = 0;
	int burst_cnt = 0;
	struct i2c_msg msg = { sr030pc50_client->addr,
		0, 0, sr030pc50_buf_for_burstmode
	};

	CAMDRV_DEBUG("Enter!!\n");

	addr = value = 0;

	*(data_buf + 6) = '\0';

	start = strnstr(sr030pc50_regs_table, name, sr030pc50_regs_table_size);
	if (start == NULL) {
		pr_err("[%s : %d] start is NULL\n", __func__, __LINE__);
		err = -EIO;
		return err;
	}

	end = strnstr(start, "};", sr030pc50_regs_table_size);
	if (end == NULL) {
		pr_err("[%s : %d] end is NULL\n", __func__, __LINE__);
		err = -EIO;
		return err;
	}

	while (1) {
		/* Find Address */
		reg = strnstr(start, "0x", sr030pc50_regs_table_size);
		if (reg)
			start = (reg + 6);

		if ((reg == NULL) || (reg > end)) {
			pr_err("[%s : %d] write end of %s\n",
			       __func__, __LINE__, name);
			break;
		}
		/* Write Value to Address */
		memcpy(data_buf, reg, 6);

		if (sr030pc50_is_hexnum(data_buf) == 0) {
			pr_err("[%s : %d] it's not hex number %s\n",
			       __func__, __LINE__, data_buf);
			continue;
		}

		err = kstrtou16(data_buf, 16, &data);
		if (err < 0) {
			pr_err("[%s : %d] kstrtou16 failed\n",
			       __func__, __LINE__);
		}
		addr = (data >> 8);
		value = (data & 0xff);

		if (idx > (BURST_MODE_BUFFER_MAX_SIZE - 10)) {
			pr_err("[%s : %d]Burst mode buffer overflow! "
			       "Burst Count %d\n",
			       __func__, __LINE__, burst_cnt);
			pr_err("[%s : %d] addr %x "
			       "value %x\n", __func__, __LINE__,
			       (data >> 8) & 0xff, data & 0xFF);

			err = -EIO;
			return err;
		}

		if (burst_flag == 0) {
			switch (addr) {
			case BURST_REG:
				if (value != 0x00) {
					burst_flag = 1;
					burst_cnt++;
				}
				break;
			case DELAY_REG:
				msleep(value * 10);	/* a step is 10ms */
				break;
			default:
				idx = 0;
				err = sr030pc50_sensor_write(addr, value);
				break;
			}
		} else if (burst_flag == 1) {
			if (addr == BURST_REG && value == 0x00) {
				msg.len = idx;
				err = i2c_transfer(sr030pc50_client->adapter,
						   &msg, 1) == 1 ? 0 : -EIO;
				idx = 0;
				burst_flag = 0;
			} else {
				if (idx == 0) {
					sr030pc50_buf_for_burstmode[idx++] =
					    addr;
					sr030pc50_buf_for_burstmode[idx++] =
					    value;
				} else
					sr030pc50_buf_for_burstmode[idx++] =
					    value;
			}
		}
	}

	CAMDRV_DEBUG("Exit!!\n");

	return err;
}

#endif

int sr030pc50_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	CAMDRV_DEBUG("Enter!!\n");

	sr030pc50_ctrl = kzalloc(sizeof(struct sr030pc50_ctrl), GFP_KERNEL);
	if (!sr030pc50_ctrl) {
		pr_err("sr030pc50_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		sr030pc50_ctrl->sensordata = data;

	first_start_camera = 1;
	prev_vtcall_mode = -1;
	config_csi2 = 0;
#ifdef CONFIG_LOAD_FILE
	rc = sr030pc50_regs_table_init();
	if (rc < 0) {
		pr_err("[%s : %d] sr030pc50_regs_table_init fail\n",
		       __func__, __LINE__);
		goto init_fail;
	}
#endif
	sr030pc50_set_power(1);
	usleep(1 * UPTO_MSEC);	/* 1msec */

	rc = sr030pc50_check_sensor_id();
	if (rc < 0) {
		pr_err("[%s : %d] sr030pc50_check_sensor_id fail\n",
		       __func__, __LINE__);
		goto init_fail;
	}

init_done:
	CAMDRV_DEBUG("Exit!!\n");
	return rc;

init_fail:
	kfree(sr030pc50_ctrl);
	return rc;
}

static int sr030pc50_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&sr030pc50_wait_queue);
	return 0;
}

int sr030pc50_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long rc = 0;

	if (copy_from_user(&cfg_data,
			   (void *)argp, sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	pr_err("[%s : %d] cfgtype = %d, mode = %d\n",
	       __func__, __LINE__, cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = sr030pc50_set_sensor_mode(cfg_data.mode);
		break;

	case CFG_SET_EFFECT:
		/*rc = sr030pc50_set_effect
		   (cfg_data.mode, cfg_data.cfg.effect); */
		break;

	default:
		rc = -EINVAL;
		pr_err("sr030pc50_sensor_config : Invalid cfgtype ! %d\n",
		       cfg_data.cfgtype);
		break;
	}
	if (rc != 0)
		pr_err("[%s : %d] fail\n", __func__, __LINE__);

	return rc;
}

int sr030pc50_sensor_release(void)
{
	int rc = 0;
	/*int *switch_i2c_addr; //TEMP Dirty Code, Do not use it! */

	first_start_camera = 1;
	set_init0 = 0;

	/*If did not init below that, it can keep the previous status.
	   it depend on concept by PCAM */
	mEffect = EXT_CFG_EFFECT_NORMAL;
	mBrightness = EXT_CFG_BR_STEP_0;
	mContrast = EXT_CFG_CR_STEP_0;
	mSaturation = EXT_CFG_SA_STEP_0;
	mSharpness = EXT_CFG_SP_STEP_0;
	mWhiteBalance = EXT_CFG_WB_AUTO;
	mISO = EXT_CFG_ISO_AUTO;
	mAutoExposure = EXT_CFG_METERING_NORMAL;
	mScene = EXT_CFG_SCENE_OFF;
	mDTP = 0;
	mInit = 0;
	mVTCallMode = 0;
	Flipmode = 0;

	CAMDRV_DEBUG("Enter!!\n");

	kfree(sr030pc50_ctrl);

#ifdef CONFIG_LOAD_FILE
	sr030pc50_regs_table_exit();
#endif

	sr030pc50_set_power(0);

	CAMDRV_DEBUG("Exit!!\n");

	return rc;
}

static int sr030pc50_i2c_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	int rc = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	sr030pc50_sensorw = kzalloc(sizeof(struct sr030pc50_work), GFP_KERNEL);

	if (!sr030pc50_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, sr030pc50_sensorw);
	sr030pc50_init_client(client);
	sr030pc50_client = client;

	pr_err("sr030pc50_probe succeeded!\n");

	return 0;

probe_failure:
	kfree(sr030pc50_sensorw);
	sr030pc50_sensorw = NULL;
	pr_err("sr030pc50_probe failed!\n");
	return rc;
}

static const struct i2c_device_id sr030pc50_i2c_id[] = {
	{"sr030pc50", 0},
	{},
};

static struct i2c_driver sr030pc50_i2c_driver = {
	.id_table = sr030pc50_i2c_id,
	.probe = sr030pc50_i2c_probe,
	.remove = __exit_p(sr030pc50_i2c_remove),
	.driver = {
		   .name = "sr030pc50",
		   },
};

static int sr030pc50_sensor_probe(const struct msm_camera_sensor_info *info,
				  struct msm_sensor_ctrl *s)
{
	int rc = i2c_add_driver(&sr030pc50_i2c_driver);
	if (rc < 0 || sr030pc50_client == NULL) {
		rc = -ENOTSUPP;
		pr_err(
			"sr030pc50_sensor_probe wooks Error. Line : %d\n",
			__LINE__);

		goto probe_done;
	}
	pr_err("sr030pc50_sensor_probe wooks E. Line : %d\n", __LINE__);

	s->s_init = sr030pc50_sensor_init;
	s->s_release = sr030pc50_sensor_release;
	s->s_config = sr030pc50_sensor_config;
	s->s_ext_config = sr030pc50_sensor_ext_config;

	s->s_camera_type = FRONT_CAMERA_2D;
	s->s_mount_angle = 270;

#ifdef FACTORY_TEST
	sr030pc50_dev =	device_create(camera_class, NULL, 0, NULL, "front");
	if (IS_ERR(sr030pc50_dev))
		pr_err("Failed to create device!");

	if (device_create_file(sr030pc50_dev, &dev_attr_front_camtype) < 0) {
		CDBG("failed to create device file, %s\n",
		dev_attr_front_camtype.attr.name);
		}
	if (device_create_file(sr030pc50_dev, &dev_attr_front_camfw) < 0) {
		CDBG("failed to create device file, %s\n",
		dev_attr_front_camfw.attr.name);
		}
	if (device_create_file(sr030pc50_dev, &sr030pc50_camera_antibanding_attr) < 0) {
		CDBG("failed to create device file, %s\n",
		sr030pc50_camera_antibanding_attr.attr.name);
		}
#endif

	pr_err("sr030pc50_sensor_probe wooks X. Line : %d\n", __LINE__);

probe_done:
	pr_info("%s:%d\n", __func__, __LINE__);
	return rc;
}

static int __sr030pc50_probe(struct platform_device *pdev)
{
	pr_err("__sr030pc50_probe wooks E. Line : %d\n", __LINE__);

	return msm_camera_drv_start(pdev, sr030pc50_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __sr030pc50_probe,
	.driver = {
		   .name = "msm_camera_sr030pc50",
		   .owner = THIS_MODULE,
		   },
};

static int __init sr030pc50_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(sr030pc50_init);
