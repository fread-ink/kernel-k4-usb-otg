/*
 * summit_smb347.c
 * Summit SMB347 Charger detection Driver for MX50 Yoshi Board
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
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/power_supply.h>

#define SUMMIT_SMB347_I2C_ADDRESS	0x06
#define SUMMIT_SMB347_ID		0xE
#define DRIVER_NAME			"summit_smb347"
#define DRIVER_VERSION			"1.0"
#define DRIVER_AUTHOR			"Manish Lachwani"

#define SUMMIT_SMB347_CHARGE_CURRENT	0x0
#define SUMMIT_SMB347_INPUT_CURR_LIMIT	0x1
#define SUMMIT_SMB347_FUNCTIONS		0x2
#define SUMMIT_SMB347_FLOAT_VOLTAGE	0x3
#define SUMMIT_SMB347_CHARGE_CONTROL	0x4
#define SUMMIT_SMB347_STAT_TIMERS	0x5
#define SUMMIT_SMB347_ENABLE_CONTROL	0x6
#define SUMMIT_SMB347_THERMAL_CONTROL	0x7
#define SUMMIT_SMB347_SYSOK_USB30	0x8
#define SUMMIT_SMB347_OTHER_CONTROL_A	0x9
#define SUMMIT_SMB347_OTG_THERM_CONTROL	0xA
#define SUMMIT_SMB347_CELL_TEMP		0xB
#define SUMMIT_SMB347_FAULT_INTERRUPT	0xC
#define SUMMIT_SMB347_INTERRUPT_STAT	0xD
#define SUMMIT_SMB347_SLAVE_ADDR	0xE

#define SUMMIT_SMB347_COMMAND_REG_A	0x30
#define SUMMIT_SMB347_COMMAND_REG_B	0x31
#define SUMMIT_SMB347_COMMAND_REG_C	0x33
#define SUMMIT_SMB347_INTSTAT_REG_A	0x35
#define SUMMIT_SMB347_INTSTAT_REG_B	0x36
#define SUMMIT_SMB347_INTSTAT_REG_C	0x37
#define SUMMIT_SMB347_INTSTAT_REG_D	0x38
#define SUMMIT_SMB347_INTSTAT_REG_E	0x39
#define SUMMIT_SMB347_INTSTAT_REG_F	0x3A
#define SUMMIT_SMB347_STATUS_REG_A	0x3B
#define SUMMIT_SMB347_STATUS_REG_B	0x3C
#define SUMMIT_SMB347_STATUS_REG_C	0x3D
#define SUMMIT_SMB347_STATUS_REG_D	0x3E
#define SUMMIT_SMB347_STATUS_REG_E	0x3F

struct summit_smb347_info {
	struct i2c_client *client;
	struct power_supply charger;
};

/* DEBUG */
#define SUMMIT_SMB347_DEBUG

static int summit_smb347_id_reg = 0;
static int smb347_reg_number = 0;

static struct i2c_client *summit_smb347_i2c_client;

static const struct i2c_device_id summit_smb347_id[] =  {
	{ "summit_smb347", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, summit_smb347_id);

static int summit_smb347_i2c_read(unsigned char reg_num, unsigned char *value);

static void summit_smb347_irq_worker(struct work_struct *unused)
{
	unsigned char value = 0xFF;
	int ret = summit_smb347_i2c_read(SUMMIT_SMB347_STATUS_REG_D, &value);
	printk(KERN_INFO "Status register D: 0x%x\n", value);
	
	ret = summit_smb347_i2c_read(SUMMIT_SMB347_INTSTAT_REG_F, &value);
	printk(KERN_INFO "Interrupt Status register F: 0x%x\n", value);

	ret = summit_smb347_i2c_read(SUMMIT_SMB347_STATUS_REG_C, &value);
	printk(KERN_INFO "Status register C: 0x%x\n", value);

	ret = summit_smb347_i2c_read(SUMMIT_SMB347_STATUS_REG_B, &value);
	printk(KERN_INFO "Status register B: 0x%x\n", value);

	enable_irq(summit_smb347_i2c_client->irq);

	return;
}

DECLARE_WORK(summit_smb347_irq_work, summit_smb347_irq_worker);

static irqreturn_t summit_smb347_irq(int irq, void *data)
{
	disable_irq_nosync(summit_smb347_i2c_client->irq);
	schedule_work(&summit_smb347_irq_work);
	return IRQ_HANDLED;
}

static int summit_smb347_i2c_read(unsigned char reg_num, unsigned char *value)
{
	s32 error;

	error = i2c_smbus_read_byte_data(summit_smb347_i2c_client, reg_num);

	if (error < 0) {
		printk(KERN_INFO "summit_smb347: i2c error\n");
		return error;
	}

	*value = (unsigned char) (error & 0xFF);
	return 0;
}

static int summit_smb347_i2c_write(unsigned char reg_num, unsigned char value)
{
	s32 error;

	error = i2c_smbus_write_byte_data(summit_smb347_i2c_client, reg_num, value);
	if (error < 0) {
		printk(KERN_INFO "summit_smb347: i2c error\n");
	}

	return error;
}

static int summit_smb347_read_id(int *id)
{
	int error = 0;
	unsigned char value = 0xFF;

	error = summit_smb347_i2c_read(SUMMIT_SMB347_ID, &value);

	if (!error) {
		*id = value;
	}

	return error;
}

static void summit_smb347_init_registers(void)
{
	/* Fast charge, pre charge and charge termination */
	summit_smb347_i2c_write(SUMMIT_SMB347_CHARGE_CURRENT, 0x3B);

	/* Input current */
	summit_smb347_i2c_write(SUMMIT_SMB347_INPUT_CURR_LIMIT, 0x66);

	/* Control settings */
	summit_smb347_i2c_write(SUMMIT_SMB347_FUNCTIONS, 0xB6);

	/* Pre charge and fast charge voltage */
	summit_smb347_i2c_write(SUMMIT_SMB347_FLOAT_VOLTAGE, 0xE3);

	/* Current termination and auto recharge */
	summit_smb347_i2c_write(SUMMIT_SMB347_CHARGE_CONTROL, 0x41);

	/* Timers and status */
	summit_smb347_i2c_write(SUMMIT_SMB347_STAT_TIMERS, 0x1F);

	/* PIN and enable control */
	summit_smb347_i2c_write(SUMMIT_SMB347_ENABLE_CONTROL, 0x7B);

	/* Thermal monitor and temp thresholds */
	summit_smb347_i2c_write(SUMMIT_SMB347_THERMAL_CONTROL, 0xAF);

	/* USB 3.0 and SYSOK */
	summit_smb347_i2c_write(SUMMIT_SMB347_SYSOK_USB30, 0x8);

	/* OTG PIN polarity and low battery */
	summit_smb347_i2c_write(SUMMIT_SMB347_OTHER_CONTROL_A, 0x07);

	/* Thermal control */
	summit_smb347_i2c_write(SUMMIT_SMB347_OTG_THERM_CONTROL, 0x10);

	/* Temperature hard/soft limits */
	summit_smb347_i2c_write(SUMMIT_SMB347_CELL_TEMP, 0x18);

	/* Interrupt Control */
	summit_smb347_i2c_write(SUMMIT_SMB347_FAULT_INTERRUPT, 0xCF);
	summit_smb347_i2c_write(SUMMIT_SMB347_INTERRUPT_STAT, 0x4);

	/* Command register A */
	summit_smb347_i2c_write(SUMMIT_SMB347_COMMAND_REG_A, 0xc2);

	/* Command register B for USB */
	summit_smb347_i2c_write(SUMMIT_SMB347_COMMAND_REG_B, 0x03);
}

/*
 * /sys access to the summit registers
 */
static ssize_t smb347_reg_store(struct sys_device *dev, struct sysdev_attribute *attr, const char *buf, size_t size)
{
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0) {
		printk(KERN_ERR "Could not store the SMB347 register value\n");
		return -EINVAL;
	}

	smb347_reg_number = value;
	return size;
}

static ssize_t smb347_reg_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	char *curr = buf;

	curr += sprintf(curr, "SMB347 Register Number: %d\n", smb347_reg_number);
	curr += sprintf(curr, "\n");

	return curr - buf;
}

static SYSDEV_ATTR(smb347_reg, 0644, smb347_reg_show, smb347_reg_store);

static struct sysdev_class smb347_reg_sysclass = {
	.name   = "smb347_reg",
};

static struct sys_device device_smb347_reg = {
	.id     = 0,
	.cls    = &smb347_reg_sysclass,
};

static ssize_t smb347_register_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	unsigned char value = 0xff;
	char *curr = buf;

	summit_smb347_i2c_read(smb347_reg_number, &value);

	curr += sprintf(curr, "SMB347 Register %d\n", smb347_reg_number);
	curr += sprintf(curr, "\n");
	curr += sprintf(curr, " Value: 0x%x\n", value);
	curr += sprintf(curr, "\n");

	return curr - buf;
};

static ssize_t smb347_register_store(struct sys_device *dev, struct sysdev_attribute *attr, const char *buf, size_t size)
{
	int value = 0;

	if (sscanf(buf, "%x", &value) <= 0) {
		printk(KERN_ERR "Error setting the value in the register\n");
		return -EINVAL;
	}

	summit_smb347_i2c_write(smb347_reg_number, value);
	return size;
}

static SYSDEV_ATTR(smb347_register, 0644, smb347_register_show, smb347_register_store);

static struct sysdev_class smb347_register_sysclass = {
	.name   = "smb347_register",
};

static struct sys_device device_smb347_register = {
	.id     = 0,
	.cls    = &smb347_register_sysclass,
};

static enum power_supply_property smb347_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
};

static int smb347_get_property(struct power_supply *ps,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	unsigned char value = 0xff;
	int err = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		err = summit_smb347_i2c_read(SUMMIT_SMB347_STATUS_REG_C, &value);
		if (err < 0)
			val->intval = err;
		else
			val->intval = value & 0x1;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		err = summit_smb347_i2c_read(SUMMIT_SMB347_STATUS_REG_C, &value);
		if (err < 0)
			val->intval = err;
		else
			val->intval =  value & 0x6;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int summit_smb347_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct summit_smb347_info *info;
	int ret = 0;
#ifdef SUMMIT_SMB347_DEBUG
	int i = 0;
#endif
	int error = 0;
	unsigned char value = 0xff;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		return -ENOMEM;
	}

	client->addr = SUMMIT_SMB347_I2C_ADDRESS;

	i2c_set_clientdata(client, info);
	info->client = client;
	info->charger.name = "summit_smb347";
	info->charger.type = POWER_SUPPLY_TYPE_MAINS;
	info->charger.get_property = smb347_get_property;
	info->charger.properties = smb347_charger_props;
	info->charger.num_properties = ARRAY_SIZE(smb347_charger_props);

	ret = power_supply_register(&client->dev, &info->charger);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		i2c_set_clientdata(client, NULL);
		kfree(info);
		return ret;
	}

	summit_smb347_i2c_client = info->client;
	summit_smb347_i2c_client->addr = SUMMIT_SMB347_I2C_ADDRESS;

	if (summit_smb347_read_id(&summit_smb347_id_reg) < 0)
		return -ENODEV;

	printk(KERN_INFO "Summit SMB347 detected, chip_id=0x%x\n", summit_smb347_id_reg);

	ret = request_irq(summit_smb347_i2c_client->irq, summit_smb347_irq,
			IRQ_TYPE_EDGE_RISING | IRQ_TYPE_EDGE_FALLING, "summit_smb347", NULL);

	if (ret != 0) {
		printk(KERN_ERR "Failed to request IRQ %d: %d\n",
				summit_smb347_i2c_client->irq, ret);
	}

	summit_smb347_init_registers();

#ifdef SUMMIT_SMB347_DEBUG
	for (i = 0; i <= 0xE; i++) {
		ret = summit_smb347_i2c_read(i, &value);
		printk(KERN_INFO "summit_smb347: reg=%d, value=0x%x\n", i, value);
	}

	for (i = 0x30; i <= 0x3F; i++) {
		ret = summit_smb347_i2c_read(i, &value);
		printk(KERN_INFO "summit_smb347: reg=%d, value=0x%x\n", i, value);
	}
#endif

	error = sysdev_class_register(&smb347_reg_sysclass);

	if (!error)
		error = sysdev_register(&device_smb347_reg);

	if (!error)
		error = sysdev_create_file(&device_smb347_reg, &attr_smb347_reg);

	error = sysdev_class_register(&smb347_register_sysclass);
	if (!error)
		error = sysdev_register(&device_smb347_register);
	if (!error)
		error = sysdev_create_file(&device_smb347_register, &attr_smb347_register);

	return 0;
}

static int summit_smb347_remove(struct i2c_client *client)
{
	struct summit_smb347_info *info = i2c_get_clientdata(client);

	i2c_set_clientdata(client, info);

	sysdev_remove_file(&device_smb347_reg, &attr_smb347_reg);
	sysdev_remove_file(&device_smb347_register, &attr_smb347_register);

	return 0;
}

static struct i2c_driver summit_smb347_i2c_driver = {
	.driver = {
			.name = DRIVER_NAME,
		},
	.probe = summit_smb347_probe,
	.remove = summit_smb347_remove,
	.id_table = summit_smb347_id,
};

static unsigned short normal_i2c[] = { SUMMIT_SMB347_I2C_ADDRESS, I2C_CLIENT_END };
I2C_CLIENT_INSMOD;


static int __init summit_smb347_init(void)
{
	int ret = 0;

	printk(KERN_INFO "Summit SMB347 Driver\n");

	ret = i2c_add_driver(&summit_smb347_i2c_driver);
	if (ret) {
		printk(KERN_ERR "summit_smb347: Could not add driver\n");
		return ret;
	}

	return 0;
}

static void __exit summit_smb347_exit(void)
{
	i2c_del_driver(&summit_smb347_i2c_driver);
}

module_init(summit_smb347_init);
module_exit(summit_smb347_exit);

MODULE_DESCRIPTION("Summit SMB347Driver");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
