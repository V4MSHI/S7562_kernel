/*
 *  headset/ear-jack device detection driver.
 *d
 *  Copyright (C) 2010 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/sec_jack.h>

#define MODULE_NAME "sec_jack:"
#define MAX_ZONE_LIMIT		10
#define SEND_KEY_CHECK_TIME_MS	60	/* 60ms */
#define DET_CHECK_TIME_MS	200	/* 200ms */
#define WAKE_LOCK_TIME		(HZ * 5)	/* 5 sec */

#define FEATURE_HSSD

#define SUPPORT_PBA
#define SUPPORT_CANSLEEP

#ifdef CONFIG_MACH_CALLISTO
#define SUPPORT_EARADC_CHECK
#endif

#ifdef SUPPORT_CANSLEEP
#define FORCE_SLEEP_COUNT_JACK_CONNECT 10
#endif

#ifdef SUPPORT_PBA
#define JACK_CLASS_NAME "audio"
#define JACK_DEV_NAME "earjack"

static struct class *jack_class;
EXPORT_SYMBOL(jack_class);
/* Sysfs device, this is used for communication with Cal App. */
static struct device *jack_dev;
EXPORT_SYMBOL(jack_dev);
#ifdef JACK_WATERPROOF
static struct device *jack_reselector_fs;
EXPORT_SYMBOL(jack_reselector_fs);
bool recheck_jack = true;
#endif
#endif

/* extern unsigned int on_call; */

struct sec_jack_info {
	struct sec_jack_platform_data *pdata;
	struct delayed_work jack_detect_work;
	struct input_dev *input;
	struct wake_lock det_wake_lock;
	struct sec_jack_zone *zone;

	bool send_key_pressed;
	bool send_key_irq_enabled;
	unsigned int cur_jack_type;

#ifdef SUPPORT_CANSLEEP
	unsigned int pre_connect;
#endif
};

/* sysfs name HeadsetObserver.java looks for to track headset state
 */
struct switch_dev switch_jack_detection = {
	.name = "h2w",
};

/* |+ To support samsung factory test +|
 * struct switch_dev switch_sendend = {
	 * .name = "sec_earbutton",
 * }; */

#ifdef JACK_WATERPROOF
static void handle_jack_not_inserted(struct sec_jack_info *hi);
#endif

static void set_send_key_state(struct sec_jack_info *hi, int state)
{
	input_report_key(hi->input, KEY_MEDIA, state);
	input_sync(hi->input);
	/* switch_set_state(&switch_sendend, state); */
	hi->send_key_pressed = state;
}

static void sec_jack_set_type(struct sec_jack_info *hi, int jack_type)
{
	struct sec_jack_platform_data *pdata = hi->pdata;

	/* this can happen during slow inserts where we think we identified
	 * the type but then we get another interrupt and do it again
	 */
	if (jack_type == hi->cur_jack_type)
		return;

#ifdef SUPPORT_CANSLEEP
	hi->pre_connect = 0;
#endif

#ifdef JACK_WATERPROOF
	if (jack_type == SEC_HEADSET_4POLE || jack_type == 3) {
#else
	if (jack_type == SEC_HEADSET_4POLE) {
#endif

		/* for a 4 pole headset, enable irq
		   for detecting send/end key presses */
		if (!hi->send_key_irq_enabled) {
#ifndef FEATURE_HSSD
			enable_irq(pdata->send_int);
			enable_irq_wake(pdata->send_int);
#endif
			hi->send_key_irq_enabled = 1;
		}
	} else {
		/* for all other jacks, disable send/end irq */
		if (hi->send_key_irq_enabled) {
#ifndef FEATURE_HSSD
			disable_irq(pdata->send_int);
			disable_irq_wake(pdata->send_int);
#endif
			hi->send_key_irq_enabled = 0;
		}
		if (hi->send_key_pressed) {
			set_send_key_state(hi, 0);
			pr_info(MODULE_NAME
				"%s : BTN set released by jack switch to %d\n",
				__func__, jack_type);
		}
	}

#ifdef JACK_WATERPROOF
	if (recheck_jack == true && jack_type == 3) {
		pr_info(MODULE_NAME "%s : JACK_WATERPROOF function work!\n", \
							__func__);
		if (hi->send_key_irq_enabled) {
#ifndef FEATURE_HSSD
			disable_irq(pdata->send_int);
			disable_irq_wake(pdata->send_int);
#endif
			hi->send_key_irq_enabled = 0;
		}
		if (hi->send_key_pressed) {
			set_send_key_state(hi, 0);
			pr_info(MODULE_NAME \
			"%s : BTN set released by jack switch to %d\n", \
			__func__, jack_type);
		}
		handle_jack_not_inserted(hi);

		pr_info(MODULE_NAME "%s : jack_type = %d\n", __func__, \
								jack_type);

		wake_lock_timeout(&hi->det_wake_lock, WAKE_LOCK_TIME);
		switch_set_state(&switch_jack_detection, 0);

		return;
	} else if (recheck_jack == false && jack_type == 3)
		jack_type = 1;
#endif

	/* micbias is left enabled for 4pole and disabled otherwise */
	if (!hi->send_key_irq_enabled)
		pdata->set_micbias_state(false);
	/* because of ESD, micbias always remain high. for only 7x27 device */
	/* pdata->set_micbias_state(true); */

	hi->cur_jack_type = jack_type;
	pr_info(MODULE_NAME "%s : jack_type = %d\n", __func__, jack_type);

	/* prevent suspend to allow user space to respond to switch */
	wake_lock_timeout(&hi->det_wake_lock, WAKE_LOCK_TIME);

	switch_set_state(&switch_jack_detection, jack_type);
}

static void handle_jack_not_inserted(struct sec_jack_info *hi)
{
	sec_jack_set_type(hi, SEC_JACK_NO_DEVICE);
	/* because of ESD, micbias always remain high. for only 7x27 device */

#ifdef SUPPORT_CANSLEEP
	if (hi->pre_connect < FORCE_SLEEP_COUNT_JACK_CONNECT)
		hi->pdata->set_micbias_state(false);

	hi->pre_connect++;
#else
	hi->pdata->set_micbias_state(false);
#endif
}

static void determine_jack_type(struct sec_jack_info *hi)
{
	struct sec_jack_zone *zones = hi->pdata->zones;
	int size = hi->pdata->num_zones;
	int count[MAX_ZONE_LIMIT] = { 0 };
	int adc;
	int i;

	if (hi->pdata->get_det_jack_state()) {
		msleep(50);
		adc = hi->pdata->get_adc_value();
		sec_jack_set_type(hi, adc);
	} else {
		handle_jack_not_inserted(hi);
	}
#ifdef JACK_WATERPROOF
	recheck_jack = false;
#endif
}

/* thread run whenever the headset detect state changes (either insertion
 * or removal).
 */
static irqreturn_t sec_jack_detect_irq_thread(int irq, void *dev_id)
{
	struct sec_jack_info *hi = dev_id;
	struct sec_jack_platform_data *pdata = hi->pdata;
	int time_left_ms = DET_CHECK_TIME_MS;

	wake_lock_timeout(&hi->det_wake_lock, WAKE_LOCK_TIME);

#ifdef SUPPORT_CANSLEEP
	if (hi->pre_connect > (FORCE_SLEEP_COUNT_JACK_CONNECT+2))
		hi->pre_connect = 0;
#endif

	/* debounce headset jack.  don't try to determine the type of
	 * headset until the detect state is true for a while.
	 */
	while (time_left_ms > 0) {
		if (!pdata->get_det_jack_state()) {
			/* jack not detected. */
			handle_jack_not_inserted(hi);
			return IRQ_HANDLED;
		}
		msleep(20);
		time_left_ms -= 20;
	}
	/* set mic bias to enable adc */
	pdata->set_micbias_state(true);
	/* jack presence was detected the whole time, figure out which type */
	determine_jack_type(hi);
	return IRQ_HANDLED;
}

#ifdef SUPPORT_PBA
static ssize_t select_jack_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	pr_info("%s : operate nothing\n", __func__);

	return 0;
}

static ssize_t select_jack_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	struct sec_jack_info *hi = dev_get_drvdata(dev);
	struct sec_jack_platform_data *pdata = hi->pdata;
	int value = 0;

	sscanf(buf, "%d", &value);
	pr_err("%s: User  selection : 0X%x", __func__, value);
#ifdef JACK_WATERPROOF
	if (value == SEC_HEADSET_4POLE || value == 3) {
#else
	if (value == SEC_HEADSET_4POLE) {
#endif
		pdata->set_micbias_state(true);
		msleep(100);
	} else {
		pdata->set_micbias_state(false);
		msleep(100);
	}

	sec_jack_set_type(hi, value);

	return size;
}

static ssize_t earjack_key_state_show(struct device *dev,
		    struct device_attribute *attr, char *buf)
{
	struct sec_jack_info *hi = dev_get_drvdata(dev);
	struct sec_jack_platform_data *pdata = hi->pdata;
	int value = 0;
	value = pdata->get_send_key_state();

	return sprintf(buf, "%d\n", value);
}

static ssize_t earjack_key_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_info("%s : operate nothing\n", __func__);

	return size;
}

static ssize_t earjack_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_jack_info *hi = dev_get_drvdata(dev);
	int value = 0;

	if (hi->cur_jack_type == SEC_HEADSET_4POLE)
		value = 1;
	else
		value = 0;

	return sprintf(buf, "%d\n", value);
}

static ssize_t earjack_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	pr_info("%s : operate nothing\n", __func__);

	return size;
}

static DEVICE_ATTR(key_state, S_IRUGO | S_IWUSR | S_IWGRP,
		earjack_key_state_show, earjack_key_state_store);

static DEVICE_ATTR(state, S_IRUGO | S_IWUSR | S_IWGRP,
		earjack_state_show, earjack_state_store);

static DEVICE_ATTR(select_jack, S_IRUGO | S_IWUSR | S_IWGRP, select_jack_show,
		   select_jack_store);
#ifdef JACK_WATERPROOF
static ssize_t reselect_jack_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	pr_info("%s : operate nothing\n", __func__);

	return 0;
}

static ssize_t reselect_jack_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf,
						size_t size)
{
	struct sec_jack_info *hi = dev_get_drvdata(dev);
	struct sec_jack_platform_data *pdata = hi->pdata;
	int value = 0;

	sscanf(buf, "%d", &value);
	pr_err("%s: User  selection : 0x%x\n", __func__, value);

	if (value == 1) {
		recheck_jack = true;
		pdata->set_micbias_state(true);
		pdata->update_earjack_type(true);
		msleep(100);
		determine_jack_type(hi);
	}
	return size;
}
static DEVICE_ATTR(reselect_jack, S_IRUGO | S_IWUSR | S_IWGRP,
				reselect_jack_show, reselect_jack_store);
#endif
#endif

/* thread run whenever the send/end key state changes. irq thread
 * handles don't need wake locks and since this one reports using
 * input_dev, input_dev guarantees that user space gets event
 * without needing a wake_lock.
 */
static irqreturn_t sec_jack_send_key_irq_thread(int irq, void *dev_id)
{
	struct sec_jack_info *hi = dev_id;
	struct sec_jack_platform_data *pdata = hi->pdata;
	int time_left_ms = SEND_KEY_CHECK_TIME_MS;
	int send_key_state = 0;
#ifdef SUPPORT_EARADC_CHECK
	int adc;
#endif

	/* debounce send/end key */
	while (time_left_ms > 0 && !hi->send_key_pressed) {
		send_key_state = pdata->get_send_key_state();
#ifdef SUPPORT_EARADC_CHECK
		adc = hi->pdata->get_adc_value();
		pr_debug(MODULE_NAME "sendend adc = %d\n", adc);
#endif

		if (!send_key_state || !pdata->get_det_jack_state() ||
		    hi->cur_jack_type != SEC_HEADSET_4POLE) {
			/* button released or jack removed or more
			 * strangely a non-4pole headset
			 */
			pr_info(MODULE_NAME "%s : ignored button (%d %d %d)\n",
				__func__, !send_key_state,
				!pdata->get_det_jack_state(),
				hi->cur_jack_type != SEC_HEADSET_4POLE);
			return IRQ_HANDLED;
		}
#ifdef SUPPORT_EARADC_CHECK
		else {
			if (adc > 120)
				return IRQ_HANDLED;
		}
#endif
		msleep(20);
		time_left_ms -= 20;
	}

	/* report state change of the send_end_key */
	if (hi->send_key_pressed != send_key_state) {
		set_send_key_state(hi, send_key_state);
		pr_info(MODULE_NAME "%s : BTN is %s.\n",
			__func__, send_key_state ? "pressed" : "released");
	}
	return IRQ_HANDLED;
}

static int sec_jack_probe(struct platform_device *pdev)
{
	struct sec_jack_info *hi;
	struct sec_jack_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	pr_info(MODULE_NAME "%s : Registering jack driver\n", __func__);
	if (!pdata) {
		pr_err("%s : pdata is NULL.\n", __func__);
		return -ENODEV;
	}

	if (!pdata->get_adc_value || !pdata->get_det_jack_state ||
	    !pdata->get_send_key_state || !pdata->zones ||
	    !pdata->set_micbias_state || pdata->num_zones > MAX_ZONE_LIMIT) {
		pr_err("%s : need to check pdata\n", __func__);
		return -ENODEV;
	}

	hi = kzalloc(sizeof(struct sec_jack_info), GFP_KERNEL);
	if (hi == NULL) {
		pr_err("%s : Failed to allocate memory.\n", __func__);
		return -ENOMEM;
	}

	hi->pdata = pdata;
#ifndef FEATURE_HSSD
	hi->input = input_allocate_device();
	if (hi->input == NULL) {
		ret = -ENOMEM;
		pr_err("%s : Failed to allocate input device.\n", __func__);
		goto err_request_input_dev;
	}
	hi->input->name = "sec_jack";
	input_set_capability(hi->input, EV_KEY, KEY_MEDIA);
	ret = input_register_device(hi->input);
	if (ret) {
		pr_err("%s : Failed to register driver\n", __func__);
		goto err_register_input_dev;
	}
#endif

	ret = switch_dev_register(&switch_jack_detection);
	if (ret < 0) {
		pr_err("%s : Failed to register switch device\n", __func__);
		goto err_switch_dev_register;
	}
#ifndef FEATURE_HSSD
	ret = switch_dev_register(&switch_sendend);
	if (ret < 0) {
		pr_err("%s : Failed to register switch device\n", __func__);
		goto err_switch_dev_register;
	}
#endif

	wake_lock_init(&hi->det_wake_lock, WAKE_LOCK_SUSPEND, "sec_jack_det");

#ifdef SUPPORT_PBA
	/* Create JACK Device file in Sysfs */
	jack_class = class_create(THIS_MODULE, JACK_CLASS_NAME);
	if (IS_ERR(jack_class))
		printk(KERN_ERR "Failed to create class(sec_jack)\n");

	jack_dev =
	    device_create(jack_class, NULL, 0, hi, JACK_DEV_NAME);
	if (IS_ERR(jack_dev))
		printk(KERN_ERR "Failed to create device(sec_jack)!= %ld\n",
		       IS_ERR(jack_dev));

	if (device_create_file(jack_dev, &dev_attr_select_jack) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n",
				dev_attr_select_jack.attr.name);

	if (device_create_file(jack_dev, &dev_attr_key_state) < 0)
		pr_err("Failed to create device file (%s)!\n",
				dev_attr_key_state.attr.name);

	if (device_create_file(jack_dev, &dev_attr_state) < 0)
		pr_err("Failed to create device file (%s)!\n",
				dev_attr_state.attr.name);
#ifdef JACK_WATERPROOF
	if (device_create_file(jack_dev, &dev_attr_reselect_jack) < 0)
		pr_err("%s : Failed to create device file(%s)!\n", __func__,
		dev_attr_reselect_jack.attr.name);
#endif
#endif
	ret = request_threaded_irq(pdata->det_int, NULL,
				   sec_jack_detect_irq_thread,
				   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				   IRQF_ONESHOT, "sec_headset_detect", hi);
	if (ret) {
		pr_err("%s : Failed to request_irq.\n", __func__);
		goto err_request_detect_irq;
	}

	/* to handle insert/removal when we're sleeping in a call */
	ret = enable_irq_wake(pdata->det_int);
	if (ret) {
		pr_err("%s : Failed to enable_irq_wake.\n", __func__);
		goto err_enable_irq_wake;
	}
#ifndef FEATURE_HSSD
	ret = request_threaded_irq(pdata->send_int, NULL,
				   sec_jack_send_key_irq_thread,
				   IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				   IRQF_ONESHOT, "sec_headset_send_key", hi);
	if (ret) {
		pr_err("%s : Failed to request_irq.\n", __func__);

		goto err_request_send_key_irq;
	}

	/* start with send/end interrupt disable. we only enable it
	 * when we detect a 4 pole headset
	 */
	disable_irq(pdata->send_int);
#endif

#ifdef SUPPORT_CANSLEEP
	hi->pre_connect = 0;
#endif

	dev_set_drvdata(&pdev->dev, hi);
#ifdef JACK_WATERPROOF
	recheck_jack = false;
#endif

	return 0;

err_request_send_key_irq:
	disable_irq_wake(pdata->det_int);
err_enable_irq_wake:
	free_irq(pdata->det_int, hi);
err_request_detect_irq:
	wake_lock_destroy(&hi->det_wake_lock);
	switch_dev_unregister(&switch_jack_detection);
	/* switch_dev_unregister(&switch_sendend); */
err_switch_dev_register:
	input_unregister_device(hi->input);
#ifndef FEATURE_HSSD
err_register_input_dev:
	input_free_device(hi->input);
#endif
err_request_input_dev:
	kfree(hi);

	return ret;
}

static int sec_jack_remove(struct platform_device *pdev)
{

	struct sec_jack_info *hi = dev_get_drvdata(&pdev->dev);

	pr_info(MODULE_NAME "%s :\n", __func__);
	/* rebalance before free */
#ifndef FEATURE_HSSD
	if (hi->send_key_irq_enabled)
		disable_irq_wake(hi->pdata->send_int);
	else
		enable_irq(hi->pdata->send_int);
	free_irq(hi->pdata->send_int, hi);
#endif
	disable_irq_wake(hi->pdata->det_int);
	free_irq(hi->pdata->det_int, hi);
	wake_lock_destroy(&hi->det_wake_lock);
	switch_dev_unregister(&switch_jack_detection);
	/* switch_dev_unregister(&switch_sendend); */
	input_unregister_device(hi->input);
	kfree(hi);

	return 0;
}

static int sec_jack_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct sec_jack_info *hi = dev_get_drvdata(&pdev->dev);
	struct sec_jack_platform_data *pdata = hi->pdata;

	/* if((!(hi->pdata->get_det_jack_state())) && (!on_call))
	 * {
	 * pr_info(MODULE_NAME "%s :micbias_reg5 off\n", __func__);
	 * pdata->set_micbias_state_reg5(false);
	 * }
	 * else
	 * pr_info(MODULE_NAME "%s :\n", __func__);
	 */

	return 0;
}

static struct platform_driver sec_jack_driver = {
	.probe = sec_jack_probe,
	.remove = sec_jack_remove,
	.suspend = sec_jack_suspend,
	.driver = {
		   .name = "sec_jack",
		   .owner = THIS_MODULE,
		   },
};

static int __init sec_jack_init(void)
{
	return platform_driver_register(&sec_jack_driver);
}

static void __exit sec_jack_exit(void)
{
	platform_driver_unregister(&sec_jack_driver);
}

late_initcall(sec_jack_init);
module_exit(sec_jack_exit);

MODULE_AUTHOR("ms17.kim@samsung.com");
MODULE_DESCRIPTION("Samsung Electronics Corp Ear-Jack detection driver");
MODULE_LICENSE("GPL");
