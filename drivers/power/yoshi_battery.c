/*
 * yoshi_battery.c
 * Battery driver for MX50 Yoshi Board
 *
 * Copyright (C) Amazon Technologies Inc. All rights reserved.
 * Manish Lachwani (lachwani@lab126.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/sysdev.h>
#include <linux/delay.h>
#include <mach/boardid.h>
#include <mach/mwan.h>
#include <battery_id.h>

/*
 * I2C registers that need to be read
 */
#define YOSHI_CTRL       			0x00
#define YOSHI_MODE       			0x01
#define YOSHI_AR_L       			0x02
#define YOSHI_AR_H       			0x03
#define YOSHI_TEMP_LOW   			0x06
#define YOSHI_TEMP_HI    			0x07
#define YOSHI_VOLTAGE_LOW			0x08
#define YOSHI_VOLTAGE_HI 			0x09
#define YOSHI_RSOC  				0x0B    /* Relative state of charge */
#define YOSHI_BATTERY_ID 			0x7E
#define YOSHI_AI_LO      			0x14    /* Average Current */
#define YOSHI_AI_HI      			0x15
#define YOSHI_FLAGS      			0x0a
#define YOSHI_CSOC       			0x2c    /* Compensated state of charge */
#define YOSHI_CAC_L      			0x10    /* milliamp-hour */
#define YOSHI_CAC_H      			0x11
#define YOSHI_CYCL_H     			0x29
#define YOSHI_CYCL_L     			0x28
#define YOSHI_LMD_H      			0x0F
#define YOSHI_LMD_L      			0x0E
#define YOSHI_NAC_L      			0x0C
#define YOSHI_NAC_H      			0x0D
#define YOSHI_CYCT_L     			0x2A
#define YOSHI_CYCT_H     			0x2B

#define YOSHI_BATTERY_RESISTANCE		20

#define YOSHI_I2C_ADDRESS			0x55	/* Battery I2C address on the bus */
#define YOSHI_TEMP_LOW_THRESHOLD		37
#define YOSHI_TEMP_HI_THRESHOLD			113
#define YOSHI_TEMP_MID_THRESHOLD		50
#define YOSHI_TEMP_CRIT_THRESHOLD		140
#define YOSHI_VOLT_LOW_THRESHOLD		2500
#define YOSHI_VOLT_HI_THRESHOLD			4350
#define YOSHI_BATTERY_INTERVAL			20000	/* 20 second duration */
#define YOSHI_BATTERY_INTERVAL_EARLY		1000	/* 1 second on probe */
#define YOSHI_BATTERY_INTERVAL_START		5000	/* 5 second timer on startup */
#define YOSHI_BATTERY_INTERVAL_ERROR		10000	/* 10 second timer after an error */
#define YOSHI_BATTERY_INTERVAL_CRITICAL		5000	/* 5 second timer on critical temp */
#define YOSHI_BATTERY_NAC_UPDATE		2000	/* 2 second duration */	
#define BATTERY_SUSPEND_CURRENT_TEQUILA 	44      /* 787uA, NAC compatible=(mAh * 20 / 3.57) */
#define YOSHI_BATTERY_I2C_WR_DELAY      	1       /* ms */
#define YOSHI_TEMP_ERROR_EVENT_THRESHOLD	30	/* Initial value of 10 minutes */
#define YOSHI_CRIT_BATTERY_CURRENT		-93

#define DRIVER_NAME			"Yoshi_Battery"
#define DRIVER_VERSION			"1.0"
#define DRIVER_AUTHOR			"Manish Lachwani"

#define GENERAL_ERROR			0x0001
#define ID_ERROR			0x0002
#define TEMP_RANGE_ERROR		0x0004
#define VOLTAGE_ERROR			0x0008

#define YOSHI_BATTERY_RETRY_THRESHOLD	5	/* Failed retry case - 5 */

#define YOSHI_ERROR_THRESHOLD		4	/* Max of 5 errors at most before sending to userspace */

#define YOSHI_BATTERY_RELAXED_THRESH	1800	/* Every 10 hours or 36000 seconds */
static int yoshi_battery_no_stats = 1;

unsigned int yoshi_battery_error_flags = 0;
EXPORT_SYMBOL(yoshi_battery_error_flags);

int yoshi_reduce_charging = 0;
EXPORT_SYMBOL(yoshi_reduce_charging);

extern int pmic_check_factory_mode(void);

int yoshi_battery_id_valid = 0;

static int yoshi_lmd_counter = YOSHI_BATTERY_RELAXED_THRESH;

int yoshi_battery_id = 0;
int yoshi_voltage = 0;
int yoshi_temperature = 0;
int yoshi_battery_current = 0;
int yoshi_battery_capacity = 0;
int yoshi_battery_mAH = 0;

EXPORT_SYMBOL(yoshi_battery_id);
EXPORT_SYMBOL(yoshi_voltage);
EXPORT_SYMBOL(yoshi_temperature);
EXPORT_SYMBOL(yoshi_battery_current);
EXPORT_SYMBOL(yoshi_battery_capacity);
EXPORT_SYMBOL(yoshi_battery_mAH);

struct yoshi_info {
        struct i2c_client *client;
};

static int yoshi_battery_lmd = 0;
static int yoshi_battery_cycl = 0;
static int yoshi_battery_cyct = 0;

extern int mxc_i2c_suspended;
static int i2c_error_counter = 0;	/* Retry counter */
static int battery_driver_stopped = 0;	/* Battery stop/start flag */
static int last_suspend_current = 0;	/* Last suspend gasguage reading */

static int temp_error_counter = 0;
static int volt_error_counter = 0;

static int battery_curr_diags = 0;
static int battery_suspend_curr_diags = 0;

static int yoshi_battreg = 0;
static struct i2c_client *yoshi_battery_i2c_client;
extern unsigned long last_suspend_time;	 

static int yoshi_battery_poll_threshold = 0;
static int yoshi_temp_error_counter = 0;
static int yoshi_temp_error_threshold = 0;

static void yoshi_battery_send_uevent(void);

static int yoshi_check_battery_id(int id);
extern void pmic_stop_charging(void);

/*
 * Create the sysfs entries
 */

static ssize_t
battery_id_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", yoshi_battery_id);
}
static SYSDEV_ATTR(battery_id, 0644, battery_id_show, NULL);

static ssize_t
battery_id_valid_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", yoshi_battery_id_valid);
}
static SYSDEV_ATTR(battery_id_valid, 0644, battery_id_valid_show, NULL);

static ssize_t
battery_current_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", yoshi_battery_current);
}
static SYSDEV_ATTR(battery_current, 0644, battery_current_show, NULL);

static ssize_t
battery_suspend_current_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", last_suspend_current);
}
static SYSDEV_ATTR(battery_suspend_current, 0644, battery_suspend_current_show, NULL);

static ssize_t
battery_voltage_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", yoshi_voltage);
}
static SYSDEV_ATTR(battery_voltage, 0644, battery_voltage_show, NULL);

static ssize_t
battery_temperature_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", yoshi_temperature);
}
static SYSDEV_ATTR(battery_temperature, 0644, battery_temperature_show, NULL);

static ssize_t
battery_capacity_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d%%\n", yoshi_battery_capacity);
}
static SYSDEV_ATTR(battery_capacity, 0644, battery_capacity_show, NULL);

static ssize_t
battery_mAH_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", yoshi_battery_mAH);
}
static SYSDEV_ATTR(battery_mAH, 0644, battery_mAH_show, NULL);

static ssize_t
battery_error_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", yoshi_battery_error_flags);
}
static SYSDEV_ATTR(battery_error, 0644, battery_error_show, NULL);

static ssize_t
battery_current_diags_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", battery_curr_diags);
}
static SYSDEV_ATTR(battery_current_diags, 0644, battery_current_diags_show, NULL);

static ssize_t
battery_suspend_current_diags_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", battery_suspend_curr_diags);
}
static SYSDEV_ATTR(battery_suspend_current_diags, 0644, battery_suspend_current_diags_show, NULL);

/* This is useful for runtime debugging */

static ssize_t
battery_show_temp_thresholds(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "Hi-%dF, Lo-%dF\n", YOSHI_TEMP_LOW_THRESHOLD, YOSHI_TEMP_HI_THRESHOLD);
}
static SYSDEV_ATTR(battery_temp_thresholds, 0644, battery_show_temp_thresholds, NULL);

static ssize_t
battery_show_voltage_thresholds(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "Hi-%dmV, Lo-%dmV\n", YOSHI_VOLT_LOW_THRESHOLD, YOSHI_VOLT_HI_THRESHOLD);
}
static SYSDEV_ATTR(battery_voltage_thresholds, 0644, battery_show_voltage_thresholds, NULL);

static ssize_t
battery_show_lmd(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", yoshi_battery_lmd);
}
static SYSDEV_ATTR(battery_lmd, 0644, battery_show_lmd, NULL);

static ssize_t
battery_show_cycl(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%x\n", yoshi_battery_cycl);
}
static SYSDEV_ATTR(battery_cycl, 0644, battery_show_cycl, NULL);

static ssize_t
battery_show_cyct(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%x\n", yoshi_battery_cyct);
}
static SYSDEV_ATTR(battery_cyct, 0644, battery_show_cyct, NULL);

static ssize_t
battery_show_polling_intervals(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "Regular-%dms, Bootup-%dms, Error-%dms, Critical-%dms\n", 
			YOSHI_BATTERY_INTERVAL, YOSHI_BATTERY_INTERVAL_START,
			YOSHI_BATTERY_INTERVAL_ERROR, YOSHI_BATTERY_INTERVAL_CRITICAL);
}
static SYSDEV_ATTR(battery_polling_intervals, 0644, battery_show_polling_intervals, NULL);

static ssize_t
battery_show_i2c_address(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "i2c address-0x%x\n", YOSHI_I2C_ADDRESS);
}
static SYSDEV_ATTR(battery_i2c_address, 0644, battery_show_i2c_address, NULL);

static ssize_t
battery_show_resume_stats(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", yoshi_battery_no_stats);
}

static ssize_t
battery_store_resume_stats(struct sys_device *dev, struct sysdev_attribute *attr, const char *buf, size_t count)
{
	int value;

	if ((sscanf(buf, "%d", &value) > 0) &&
		((value == 0) || (value == 1))) {
			yoshi_battery_no_stats = value;
			return strlen(buf);
	}

	return -EINVAL;
}
static SYSDEV_ATTR(resume_stats, S_IRUGO|S_IWUSR, battery_show_resume_stats, battery_store_resume_stats);

static ssize_t
battery_show_battreg(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", yoshi_battreg);
}

static ssize_t
battery_store_battreg(struct sys_device *dev, struct sysdev_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sscanf(buf, "%x", &value) <= 0) {
		printk(KERN_DEBUG "Could not store the battery register value\n");
		return -EINVAL;
	}

	yoshi_battreg = value;
	return count;
}

static SYSDEV_ATTR(battreg, S_IRUGO|S_IWUSR, battery_show_battreg, battery_store_battreg);

static ssize_t
battery_show_battreg_value(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	int value = 0;
	char *curr = buf;

	value = i2c_smbus_read_byte_data(yoshi_battery_i2c_client, yoshi_battreg);
	curr += sprintf(curr, "Battery Register %d\n", yoshi_battreg);
	curr += sprintf(curr, "\n");
	curr += sprintf(curr, " Value: 0x%x\n", value);
	curr += sprintf(curr, "\n");
	
	return curr - buf;
}

static ssize_t
battery_store_battreg_value(struct sys_device *dev, struct sysdev_attribute *attr, const char *buf, size_t count)
{
	int value;
	u8 tmp = 0;

	if (sscanf(buf, "%x", &value) <= 0) {
		printk(KERN_DEBUG "Error setting the value in the register\n");
		return -EINVAL;
	}

	tmp = (u8)value;
	i2c_smbus_write_byte_data(yoshi_battery_i2c_client, yoshi_battreg, tmp);

	return count;
}

static SYSDEV_ATTR(battreg_value, S_IRUGO|S_IWUSR, battery_show_battreg_value, battery_store_battreg_value);

static ssize_t	
battery_show_battery_overheat(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	char *curr = buf;
	int value = 0;

	if ( (yoshi_temperature > YOSHI_TEMP_CRIT_THRESHOLD) &&
		(yoshi_battery_current < YOSHI_CRIT_BATTERY_CURRENT) ) {
		value = 1;
	}

	curr += sprintf(curr, "%d\n", value);
	return curr - buf;
}

static SYSDEV_ATTR(battery_overheat, S_IRUGO|S_IWUSR, battery_show_battery_overheat, NULL);

static ssize_t	
battery_show_battery_temp_errthresh(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	char *curr = buf;

	curr += sprintf(curr, "temp error threshold =  %d\n", yoshi_temp_error_threshold);
	return curr - buf;
}

static ssize_t
battery_store_battery_temp_errthresh(struct sys_device *dev, struct sysdev_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (sscanf(buf, "%d", &value) <= 0) {
		return -EINVAL;
	}

	if (value < 10) {
		printk (KERN_DEBUG "temp error threshold too aggresive\n");
		return -EINVAL;
	}
		
	yoshi_temp_error_threshold = value;
	return count;
}

static SYSDEV_ATTR(battery_temp_errthresh, S_IRUGO|S_IWUSR, battery_show_battery_temp_errthresh, battery_store_battery_temp_errthresh);

static ssize_t
battery_store_battery_send_uevent(struct sys_device *dev, struct sysdev_attribute *attr, const char *buf, size_t count)
{
	yoshi_battery_send_uevent();
	return count;
}

static SYSDEV_ATTR(battery_send_uevent, S_IRUGO|S_IWUSR, NULL, battery_store_battery_send_uevent);

static struct sysdev_class yoshi_battery_sysclass = {
	.name = "yoshi_battery",
};

static struct sys_device yoshi_battery_device = {
	.id = 0,
	.cls = &yoshi_battery_sysclass,
};

static int yoshi_battery_sysdev_ctrl_init(void)
{
	int err = 0;

	err = sysdev_class_register(&yoshi_battery_sysclass);
	if (!err)
		err = sysdev_register(&yoshi_battery_device);
	if (!err) {
		sysdev_create_file(&yoshi_battery_device, &attr_battery_id);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_id_valid);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_current);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_voltage);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_temperature);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_capacity);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_mAH);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_voltage_thresholds);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_polling_intervals);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_temp_thresholds);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_i2c_address);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_error);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_suspend_current);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_current_diags);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_suspend_current_diags);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_cycl);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_lmd);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_cyct);
		sysdev_create_file(&yoshi_battery_device, &attr_resume_stats);
		sysdev_create_file(&yoshi_battery_device, &attr_battreg);
		sysdev_create_file(&yoshi_battery_device, &attr_battreg_value);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_temp_errthresh);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_send_uevent);
		sysdev_create_file(&yoshi_battery_device, &attr_battery_overheat);
	}

	return err;
}

static void yoshi_battery_sysdev_ctrl_exit(void)
{
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_id);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_id_valid);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_current);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_voltage);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_temperature);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_capacity);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_mAH);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_voltage_thresholds);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_polling_intervals);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_temp_thresholds);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_i2c_address);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_error);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_suspend_current);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_current_diags);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_suspend_current_diags);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_cycl);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_lmd);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_cyct);
	sysdev_remove_file(&yoshi_battery_device, &attr_resume_stats);
	sysdev_remove_file(&yoshi_battery_device, &attr_battreg);
	sysdev_remove_file(&yoshi_battery_device, &attr_battreg_value);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_temp_errthresh);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_send_uevent);
	sysdev_remove_file(&yoshi_battery_device, &attr_battery_overheat);

	sysdev_unregister(&yoshi_battery_device);
	sysdev_class_unregister(&yoshi_battery_sysclass);
}

int yoshi_battery_present = 0;
EXPORT_SYMBOL(yoshi_battery_present);

int yoshi_battery_valid(void)
{
    return (yoshi_battery_present && yoshi_battery_id_valid);
}
EXPORT_SYMBOL(yoshi_battery_valid);

static int yoshi_i2c_read(unsigned char reg_num, unsigned char *value)
{
	s32 error;

	error = i2c_smbus_read_byte_data(yoshi_battery_i2c_client, reg_num);
	if (error < 0) {
		printk(KERN_DEBUG "yoshi_battery: i2c read retry\n");
		return -EIO;
	}

	*value = (unsigned char) (error & 0xFF);
	return 0;
}

static int yoshi_i2c_write(unsigned char reg_num, unsigned char value)
{
	s32 error;
	
	error = i2c_smbus_write_byte_data(yoshi_battery_i2c_client, reg_num, value);
	if (error < 0) {
		printk(KERN_DEBUG "yoshi_battery: i2c write retry\n");
		return -EIO;
	}
	return 0;
}

static int yoshi_battery_read_voltage(int *voltage)
{
	unsigned char hi, lo;
	int volts;
	int err1 = 0, err2 = 0;
	int error = 0;
	
	err1 = yoshi_i2c_read(YOSHI_VOLTAGE_LOW, &lo);
	err2 = yoshi_i2c_read(YOSHI_VOLTAGE_HI, &hi);

	error = err1 | err2;

	if (!error) {
		volts = (hi << 8) | lo;
		*voltage = volts;
	}
	
	return error;
}

static int yoshi_battery_read_current(int *curr)
{
	unsigned char hi, lo, flags;
	int c;
	int err1 = 0, err2 = 0, err3 = 0;
	int sign = 1;
	int error = 0;

	err1 = yoshi_i2c_read(YOSHI_AI_LO, &lo);
	err2 = yoshi_i2c_read(YOSHI_AI_HI, &hi);
	err3 = yoshi_i2c_read(YOSHI_FLAGS, &flags);

	error = err1 | err2 | err3;

	if (!error) {
		if (flags & 0x80)
			sign = 1;
		else
			sign = -1;

		battery_curr_diags = sign * (((hi << 8) | lo) * 357);

		if (!battery_suspend_curr_diags)
			battery_suspend_curr_diags = battery_curr_diags;

		c = sign * ((((hi << 8) | lo) * 357) / (100 * YOSHI_BATTERY_RESISTANCE));
		*curr = c;
	}

	return error;
}

static int yoshi_battery_read_capacity(int *capacity)
{
	unsigned char tmp;
	int err = 0;

	err = yoshi_i2c_read(YOSHI_CSOC, &tmp);
	if (!err)
		*capacity = tmp;

	return err;
}
	
static int yoshi_battery_read_mAH(int *mAH)
{
	unsigned char hi, lo;
	int err1 = 0, err2 = 0;
	int error = 0;
	
	err1 = yoshi_i2c_read(YOSHI_CAC_L, &lo);
	err2 = yoshi_i2c_read(YOSHI_CAC_H, &hi);

	error = err1 | err2;

	if (!error)
		*mAH = ((((hi << 8) | lo) * 357) / (100 * YOSHI_BATTERY_RESISTANCE));
	
	return error;
}

/* Read Last Measured Discharge and Learning count */
static void yoshi_battery_read_lmd_cyc(int *lmd, int *cycl, int *cyct)
{
	unsigned char hi, lo;
	int err1 = 0, err2 = 0;
	int error = 0;

	err1 = yoshi_i2c_read(YOSHI_CYCL_L, &lo);
	err2 = yoshi_i2c_read(YOSHI_CYCL_H, &hi);

	error = err1 | err2;

	if (!error)
		*cycl = (hi << 8) | lo;

	/* Clear out hi, lo for next reading */
	hi = lo = 0;

	err1 = yoshi_i2c_read(YOSHI_LMD_L, &lo);
	err2 = yoshi_i2c_read(YOSHI_LMD_H, &hi);

	error = err1 | err2;

	if (!error)
		*lmd = ((((hi << 8) | lo) * 357) / (100 * YOSHI_BATTERY_RESISTANCE));

	/* Clear out hi, lo for next reading */
	hi = lo = 0;

	err1 = yoshi_i2c_read(YOSHI_CYCT_L, &lo);
	err2 = yoshi_i2c_read(YOSHI_CYCT_H, &hi);

	error = err1 | err2;

	if (!error)
		*cyct = (hi << 8) | lo;
}

static int yoshi_battery_read_temperature(int *temperature)
{
	unsigned char temp_hi, temp_lo;
	int err1 = 0, err2 = 0;
	int celsius, fahrenheit;
	int error = 0;
	
	err1 = yoshi_i2c_read(YOSHI_TEMP_LOW, &temp_lo);
	err2 = yoshi_i2c_read(YOSHI_TEMP_HI, &temp_hi);
	
	error = err1 | err2;

	if (!error) {
		celsius = ((((temp_hi << 8) | temp_lo) + 2) / 4) - 273;
		fahrenheit = ((celsius * 9) / 5) + 32;
		*temperature = fahrenheit;
	}

	return error;
}

static int yoshi_check_battery_id(int id)
{
    int i;
    const int *list;
    int num = 0;

    /** Only validiate batteries on non-proto hw */
    if (mx50_board_rev_greater_eq(BOARD_ID_TEQUILA_EVT2)) {
	list = tequila_valid_ids;
	num = sizeof(tequila_valid_ids)/sizeof(tequila_valid_ids[0]);
    } else {
	/* prototype device - always valid */
	return 1;
    }

    /* check list for valid id */
    for (i = 0; i < num; i++) {
	if (list[i] == id) {
	    return 1;
	}
    }

    return 0;
}

static int yoshi_battery_read_id(int *id)
{
	int error = 0;
	unsigned char value = 0xFF;

	error = yoshi_i2c_read(YOSHI_BATTERY_ID, &value);
	
	if (!error) {
		*id = value;
	}

	return error;
}

static int yoshi_battery_read_rsoc(int *rsoc)
{
	int error = 0;
	unsigned char value = 0xFF;
	
	error = yoshi_i2c_read(YOSHI_RSOC, &value);
	if (!error) {
		*rsoc = value;
	}
	
	return error;
}

/* Read Nominal Available Capacity */	
static int yoshi_battery_read_nac(int *nac)
{
	unsigned char hi, lo;
	int err1 = 0, err2 = 0;
	int error = 0;
	
	err1 = yoshi_i2c_read(YOSHI_NAC_L, &lo);
	err2 = yoshi_i2c_read(YOSHI_NAC_H, &hi);
	
	error = err1 | err2;
	
	if (!error)
		*nac = (hi << 8) | lo;
	
	return error;
}

/* Write Nominal Available Capacity */		
static int yoshi_battery_write_nac(int nac)		
{
	int err1 = 0, err2 = 0, err3 = 0, err4 = 0;
	int error = 0;
	
	err1 = yoshi_i2c_write(YOSHI_AR_L, (nac & 0xFF));
	msleep(YOSHI_BATTERY_I2C_WR_DELAY);
	err2 = yoshi_i2c_write(YOSHI_AR_H, ((nac & 0xFF00) >> 8));
	msleep(YOSHI_BATTERY_I2C_WR_DELAY);
	err3 = yoshi_i2c_write(YOSHI_MODE, 0x64);
	msleep(YOSHI_BATTERY_I2C_WR_DELAY);
	err4 = yoshi_i2c_write(YOSHI_CTRL, 0xA9);
	msleep(YOSHI_BATTERY_I2C_WR_DELAY);
	error = err1 | err2 | err3 | err4;
	return error;
}

static void battery_handle_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(battery_work, battery_handle_work);
static void battery_handle_update_nac(struct work_struct *work);				
static DECLARE_DELAYED_WORK(battery_work_nac, battery_handle_update_nac);

static void yoshi_battery_send_uevent(void)
{
	char *envp[] = { "BATTERY=overheat", NULL };
	kobject_uevent_env(&yoshi_battery_i2c_client->dev.kobj, KOBJ_CHANGE, envp);
}

/* Main battery timer task */
static void battery_handle_work(struct work_struct *work)
{
	int err = 0;
	int batt_err_flag = 0;

	/* Is the battery driver stopped? */
	if (battery_driver_stopped)
		return;

	/* Is i2c bus suspended? */
	if (mxc_i2c_suspended)
		goto out_done;

	err = yoshi_battery_read_id(&yoshi_battery_id);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	}
	else
		i2c_error_counter = 0;

	err = yoshi_battery_read_temperature(&yoshi_temperature);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	}
	else
		i2c_error_counter = 0;

	/*
	 * Check for the temperature range violation
	 */
	if ( (yoshi_temperature <= YOSHI_TEMP_LOW_THRESHOLD) ||
		(yoshi_temperature >= YOSHI_TEMP_HI_THRESHOLD) ) {
			temp_error_counter++;
			yoshi_reduce_charging = 0;
	}
	else {
		if (yoshi_temperature < YOSHI_TEMP_MID_THRESHOLD)
			yoshi_reduce_charging = 1;
		else
			yoshi_reduce_charging = 0;

		temp_error_counter = 0;
		yoshi_battery_error_flags &= ~TEMP_RANGE_ERROR;
	}

	/* Over heat condition of battery */
	if ( (yoshi_temperature > YOSHI_TEMP_CRIT_THRESHOLD) &&
		(yoshi_battery_current < YOSHI_CRIT_BATTERY_CURRENT) ) {
			yoshi_temp_error_counter++;
			if (yoshi_temp_error_counter > yoshi_temp_error_threshold) {
			    printk(KERN_CRIT "yoshi_battery: E battery:overheat:temp=%d F current=%d mA:\n", 
				   yoshi_temperature, yoshi_battery_current); 
			    yoshi_battery_send_uevent();
			    yoshi_temp_error_counter = 0;
			}
	}
	else
		yoshi_temp_error_counter = 0;

	if (temp_error_counter > YOSHI_ERROR_THRESHOLD) {
		yoshi_battery_error_flags |= TEMP_RANGE_ERROR;
		printk(KERN_ERR "yoshi_battery: E battery:hitemp:temp=%d F:\n", 
		       yoshi_temperature); 
		temp_error_counter = 0;
	}

	err = yoshi_battery_read_voltage(&yoshi_voltage);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	}
	else
		i2c_error_counter = 0;

	/*
	 * Check for the battery voltage range violation
	 */
	if ( (yoshi_voltage <= YOSHI_VOLT_LOW_THRESHOLD) ||
		(yoshi_voltage >= YOSHI_VOLT_HI_THRESHOLD) ) {
			volt_error_counter++;
	}
	else {
		volt_error_counter = 0;
		yoshi_battery_error_flags &= ~VOLTAGE_ERROR;
	}

	if (volt_error_counter > YOSHI_ERROR_THRESHOLD) {
		yoshi_battery_error_flags |= VOLTAGE_ERROR;
		printk(KERN_ERR "yoshi_battery: E battery:voltage:%d mV:\n", 
		       yoshi_voltage); 
		volt_error_counter = 0;
	}

	/*
	 * Check for the battery current
	 */
	err = yoshi_battery_read_current(&yoshi_battery_current);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;	
		goto out;
	}
	else
		i2c_error_counter = 0;

	/*
	 * Read the gasguage capacity
	 */
	err = yoshi_battery_read_capacity(&yoshi_battery_capacity);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	}
	else
		i2c_error_counter = 0;

	/*
	 * Read the battery mAH
	 */
	err = yoshi_battery_read_mAH(&yoshi_battery_mAH);
	if (err) {
		batt_err_flag |= GENERAL_ERROR;
		goto out;
	}
	else
		i2c_error_counter = 0;

	/* Take these readings every 10 hours */
	if (yoshi_lmd_counter == YOSHI_BATTERY_RELAXED_THRESH) {
		yoshi_lmd_counter = 0;
		yoshi_battery_read_lmd_cyc(&yoshi_battery_lmd, &yoshi_battery_cycl, &yoshi_battery_cyct);
	}
	else {
		yoshi_lmd_counter++;
	}

out:
	if (batt_err_flag) {
		i2c_error_counter++;
		if (i2c_error_counter == YOSHI_BATTERY_RETRY_THRESHOLD) {
			yoshi_battery_error_flags |= GENERAL_ERROR;
			printk(KERN_ERR "Yoshi battery: i2c read error, retry exceeded\n");
			i2c_error_counter = 0;
		}
	}
	else {
		yoshi_battery_error_flags &= ~GENERAL_ERROR;
		i2c_error_counter = 0;
	}

	pr_debug("temp: %d, volt: %d, current: %d, capacity: %d%%, mAH: %d\n",
		yoshi_temperature, yoshi_voltage, yoshi_battery_current,
		yoshi_battery_capacity, yoshi_battery_mAH);
out_done:
	if (batt_err_flag & GENERAL_ERROR)
		yoshi_battery_poll_threshold = YOSHI_BATTERY_INTERVAL_ERROR;
	else {
		if (yoshi_temperature > YOSHI_BATTERY_INTERVAL_CRITICAL)
			yoshi_battery_poll_threshold = YOSHI_BATTERY_INTERVAL_CRITICAL;
		else
			yoshi_battery_poll_threshold = YOSHI_BATTERY_INTERVAL;
	}

	schedule_delayed_work(&battery_work, msecs_to_jiffies(yoshi_battery_poll_threshold));
	return;
}

static void battery_handle_update_nac(struct work_struct *work)
{
	int err = 0, hours = 0;
	int battery_nac = 0, battery_nac_after_resume = 0;
	int battery_nac_cap_drop = 0;
	int battery_suspend_ct = 0;
	int battery_rsoc = 0;

	 /* Is the battery driver stopped? */
	if (battery_driver_stopped)
		return;

	/* Is i2c bus suspended? */
	if (mxc_i2c_suspended)
		return;

	if (mx50_board_is(BOARD_ID_TEQUILA)) {
		battery_suspend_ct = BATTERY_SUSPEND_CURRENT_TEQUILA;
	} 

	if (battery_suspend_ct == 0)
		return;

	err = yoshi_battery_read_rsoc(&battery_rsoc);
	if (err) {
		printk(KERN_DEBUG "Yoshi battery: RSOC - i2c read error \n");
		return;
	}
	
	hours = (last_suspend_time / (60 * 60));
	if (last_suspend_time % (60 * 60))
		hours += 1;
	/* Calculate battery nac drop (mAH) */
	battery_nac_cap_drop = (hours * battery_suspend_ct) / 10;
	if (((hours * battery_suspend_ct) % 10) > 5) 
		battery_nac_cap_drop += 1;
	
	err = yoshi_battery_read_nac(&battery_nac_after_resume);
	if (err) {
		printk(KERN_DEBUG "Yoshi battery: NAC - i2c read error \n");
		return;
	}
	
	if (battery_nac_cap_drop < battery_nac_after_resume)
		battery_nac = battery_nac_after_resume - battery_nac_cap_drop;
	else
		battery_nac = 0;

	printk(KERN_DEBUG "Battery: Resume - TimeSuspended(secs)=%lu, NAC=0x%x, RSOC=%d \n",
				last_suspend_time, battery_nac_after_resume, battery_rsoc);
	
	err = yoshi_battery_write_nac(battery_nac);
	if (err) { 
		printk(KERN_DEBUG "Yoshi battery: NAC - i2c write error \n");
		return;
	}

	err = yoshi_battery_read_rsoc(&battery_rsoc);
	if (err) {
		printk(KERN_DEBUG "Yoshi battery: RSOC - i2c read error \n");
		return;
	}
	
	printk(KERN_DEBUG "Battery: Resume - PostUpdate NAC=0x%x, RSOC=%d \n",
				battery_nac, battery_rsoc);

	return;
}

static const struct i2c_device_id yoshi_id[] =  {
        { "Yoshi_Battery", 0 },
        { },
};
MODULE_DEVICE_TABLE(i2c, yoshi_id);

static int yoshi_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        struct yoshi_info  *info;

        info = kzalloc(sizeof(*info), GFP_KERNEL);
        if (!info) {
                return -ENOMEM;
        }

        client->addr = YOSHI_I2C_ADDRESS;
        i2c_set_clientdata(client, info);
        info->client = client;
        yoshi_battery_i2c_client = info->client;
        yoshi_battery_i2c_client->addr = YOSHI_I2C_ADDRESS;

	if (yoshi_battery_read_id(&yoshi_battery_id) < 0) {
	    printk(KERN_ERR "yoshi_battery: E battery:not detected:\n");

	    /* don't disable charging in factory mode */
	    if (pmic_check_factory_mode()) {
		return 0;
	    } else {
		pmic_stop_charging();
		return -ENODEV;
	    }
	}

	yoshi_battery_id_valid = 0;

	/* For compliance - turn off charging on non-proto devices if 
	   battery is present but id is invalid */
	if (!yoshi_check_battery_id(yoshi_battery_id)) {
	    printk(KERN_ERR "yoshi_battery: E battery:invalid id %d:\n", yoshi_battery_id);
	    pmic_stop_charging();
	} else {
	    yoshi_battery_id_valid = 1;
	}

        if (yoshi_battery_sysdev_ctrl_init() < 0)
                printk(KERN_DEBUG "yoshi battery: could not create sysfs entries\n");

	schedule_delayed_work(&battery_work, msecs_to_jiffies(YOSHI_BATTERY_INTERVAL_EARLY));

	yoshi_battery_present = 1;

	/* Set the poll interval */
	yoshi_battery_poll_threshold = YOSHI_BATTERY_INTERVAL;

	/* Set the error threshold for temperature error counter */
	yoshi_temp_error_threshold = YOSHI_TEMP_ERROR_EVENT_THRESHOLD;

        return 0;
}

static int yoshi_remove(struct i2c_client *client)
{
	struct yoshi_info *info = i2c_get_clientdata(client);
	
	if (yoshi_battery_present) {
		battery_driver_stopped = 1;
		cancel_rearming_delayed_work(&battery_work_nac);
		cancel_rearming_delayed_work(&battery_work);
		yoshi_battery_sysdev_ctrl_exit();
	}

	i2c_set_clientdata(client, info);

	return 0;
}

static unsigned short normal_i2c[] = { YOSHI_I2C_ADDRESS, I2C_CLIENT_END };
I2C_CLIENT_INSMOD;

static int yoshi_battery_suspend(struct i2c_client *client, pm_message_t state)
{
	if (yoshi_battery_present) {
		battery_driver_stopped = 1;
		cancel_rearming_delayed_work(&battery_work_nac);
		cancel_rearming_delayed_work(&battery_work);
		i2c_error_counter = 0;
		yoshi_temp_error_counter = 0;
	}

	return 0;
}
EXPORT_SYMBOL(yoshi_battery_suspend);

static int yoshi_battery_resume(struct i2c_client *client)
{
	battery_driver_stopped = 0;
	schedule_delayed_work(&battery_work_nac, msecs_to_jiffies(YOSHI_BATTERY_NAC_UPDATE));	
	schedule_delayed_work(&battery_work, msecs_to_jiffies(yoshi_battery_poll_threshold));

	return 0;
}

static struct i2c_driver yoshi_i2c_driver = {
	.driver = {
			.name = DRIVER_NAME,
		},
	.probe = yoshi_probe,
	.remove = yoshi_remove,
	.suspend = yoshi_battery_suspend,
	.resume = yoshi_battery_resume,
	.id_table = yoshi_id,
};
	
static int __init yoshi_battery_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&yoshi_i2c_driver);

	if (ret) {
		printk(KERN_DEBUG "Yoshi battery: Could not add driver\n");
		return ret;
	}
	return 0;
}
	

static void __exit yoshi_battery_exit(void)
{
	if (yoshi_battery_present) {
		battery_driver_stopped = 1;
		cancel_rearming_delayed_work(&battery_work);
		yoshi_battery_sysdev_ctrl_exit();
	}
	i2c_del_driver(&yoshi_i2c_driver);
}

module_init(yoshi_battery_init);
module_exit(yoshi_battery_exit);

MODULE_DESCRIPTION("MX50 Yoshi Battery Driver");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
