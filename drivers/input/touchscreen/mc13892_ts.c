/*
 * Copyright 2010 Amazon.com, Inc. All Rights Reserved.
 * Manish Lachwani (lachwani@lab126.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/freezer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_adc.h>
#include <linux/kthread.h>

#define MC13892_TS_NAME		"mc13892_ts"

static struct input_dev *mc13892_inputdev;
static struct task_struct *touch_task;
static int calibration[7];
module_param_array(calibration, int, NULL, S_IRUGO | S_IWUSR);

/* Kernel thread for touch input */
static int touch_thread(void *arg)
{
	t_touch_screen touch_sample;
	s32 wait = 0;

	do {
		int x, y;
		static int last_x = -1, last_y = -1, last_press = -1;

		memset(&touch_sample, 0, sizeof(t_touch_screen));
		if (0 != pmic_adc_get_touch_sample(&touch_sample, !wait))
			continue;
		if (!(touch_sample.contact_resistance || wait))
			continue;

		if (touch_sample.x_position == 0 && touch_sample.y_position == 0 &&
			touch_sample.contact_resistance == 0) {
			x = last_x;
			y = last_y;
		} else if (calibration[6] == 0) {
			x = touch_sample.x_position;
			y = touch_sample.y_position;
		} else {
			x = calibration[0] * (int)touch_sample.x_position +
				calibration[1] * (int)touch_sample.y_position +
				calibration[2];
			x /= calibration[6];
			if (x < 0)
				x = 0;
			y = calibration[3] * (int)touch_sample.x_position +
				calibration[4] * (int)touch_sample.y_position +
				calibration[5];
			y /= calibration[6];
			if (y < 0)
				y = 0;
		}

		if (x != last_x) {
			input_report_abs(mc13892_inputdev, ABS_X, x);
			last_x = x;
		}
		if (y != last_y) {
			input_report_abs(mc13892_inputdev, ABS_Y, y);
			last_y = y;
		}

		/* report pressure */
		input_report_abs(mc13892_inputdev, ABS_PRESSURE,
				 touch_sample.contact_resistance);

		if (touch_sample.contact_resistance > 22)
			touch_sample.contact_resistance = 1;
		else
			touch_sample.contact_resistance = 0;

		/* report the BTN_TOUCH */
		if (touch_sample.contact_resistance != last_press)
			input_event(mc13892_inputdev, EV_KEY,
					BTN_TOUCH, touch_sample.contact_resistance);

		input_sync(mc13892_inputdev);
		last_press = touch_sample.contact_resistance;

		wait = touch_sample.contact_resistance;
		msleep(20);

	} while (!kthread_should_stop());

	return 0;
}

static int __init mc13892_ts_init(void)
{
	int retval;

	if (!is_pmic_adc_ready())
		return -ENODEV;

	mc13892_inputdev = input_allocate_device();
	if (!mc13892_inputdev) {
		printk(KERN_ERR
		       "mc13892_ts_init: not enough memory\n");
		return -ENOMEM;
	}

	mc13892_inputdev->name = MC13892_TS_NAME;
	mc13892_inputdev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	mc13892_inputdev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);
	mc13892_inputdev->absbit[0] =
	    BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) | BIT_MASK(ABS_PRESSURE);
	retval = input_register_device(mc13892_inputdev);
	if (retval < 0) {
		input_free_device(mc13892_inputdev);
		return retval;
	}

	touch_task = kthread_run(touch_thread, NULL, "mc13892_ts");
	if (IS_ERR(touch_task)) {
		printk(KERN_ERR
			"mc13892_ts_init: failed to create kthread");
		touch_task = NULL;
		return -1;
	}
	printk(KERN_INFO "Atlas Lite Touchscreen Loaded\n");
	return 0;
}

static void __exit mc13892_ts_exit(void)
{
	if (touch_task)
		kthread_stop(touch_task);

	input_unregister_device(mc13892_inputdev);

	if (mc13892_inputdev) {
		input_free_device(mc13892_inputdev);
		mc13892_inputdev = NULL;
	}
}

late_initcall(mc13892_ts_init);
module_exit(mc13892_ts_exit);

MODULE_DESCRIPTION("MXC touchscreen driver with calibration");
MODULE_AUTHOR("Manish Lachwani <lachwani@lab126.com>");
MODULE_LICENSE("GPL");
