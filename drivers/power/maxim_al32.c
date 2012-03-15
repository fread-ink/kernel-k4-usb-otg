/*
 * maxim_al32.c
 *o Maxim AL32 Charger detection Driver for MX50 Yoshi Board
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

#define MAXIM_AL32_I2C_ADDRESS	0x35
#define MAXIM_AL32_ID		0x0
#define DRIVER_NAME		"maxim_al32"
#define DRIVER_VERSION		"1.0"
#define DRIVER_AUTHOR		"Manish Lachwani"
#define MAXIM_SYSCTRL1		0x07
#define MAXIM_SYSCTRL2		0x08
#define MAXIM_INTMASK1		0x05
#define MAXIM_INTMASK2		0x06
#define MAXIM_INTSTATUS1	0x03
#define MAXIM_INTSTATUS2	0x04
#define MAXIM_INT1		0x01
#define MAXIM_INT2		0x02

/* DEBUG */
#undef MAXIM_AL32_DEBUG

static int maxim_al32_id_reg = 0;

struct maxim_al32_info {
	struct i2c_client *client;
};

static struct i2c_client *maxim_al32_i2c_client;

static const struct i2c_device_id maxim_al32_id[] =  {
	{ "maxim_al32", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, maxim_al32_id);

static int maxim_al32_i2c_read(unsigned char reg_num, unsigned char *value);

static void maxim_al32_irq_worker(struct work_struct *unused)
{
	unsigned char value = 0xFF;
	int ret = maxim_al32_i2c_read(MAXIM_INT2, &value);

	if (value & 0x1)
		printk(KERN_INFO "maxim_al32: Detected a serial connection: 0x%x\n", value);

	ret = maxim_al32_i2c_read(MAXIM_INT1, &value);

	if (value & 0x1) {
		ret = maxim_al32_i2c_read(MAXIM_INTSTATUS1, &value);
		printk(KERN_INFO "maxim_al32: charger_type: 0x%x\n", (value & 0xf));
	}

	enable_irq(maxim_al32_i2c_client->irq);

	return;
}

DECLARE_WORK(maxim_al32_irq_work, maxim_al32_irq_worker);

static irqreturn_t maxim_al32_irq(int irq, void *data)
{
	disable_irq_nosync(maxim_al32_i2c_client->irq);
	schedule_work(&maxim_al32_irq_work);
	return IRQ_HANDLED;
}

static int maxim_al32_i2c_read(unsigned char reg_num, unsigned char *value)
{
	s32 error;

	error = i2c_smbus_read_byte_data(maxim_al32_i2c_client, reg_num);

	if (error < 0) {
		printk(KERN_INFO "maxim_al32: i2c error\n");
	}

	*value = (unsigned char) (error & 0xFF);
	return 0;
}

static int maxim_al32_i2c_write(unsigned char reg_num, unsigned char value)
{
	s32 error;

	error = i2c_smbus_write_byte_data(maxim_al32_i2c_client, reg_num, value);
	if (error < 0) {
		printk(KERN_INFO "maxim_al32: i2c error\n");
	}

	return error;
}

static int maxim_al32_read_id(int *id)
{
	int error = 0;
	unsigned char value = 0xFF;

	error = maxim_al32_i2c_read(MAXIM_AL32_ID, &value);

	if (!error) {
		*id = value;
	}

	return error;
}

static int maxim_al32_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct maxim_al32_info *info;
	int ret = 0;
#ifdef MAXIM_AL32_DEBUG
	int i = 0;
#endif
	unsigned char value = 0xff;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		return -ENOMEM;
	}

	client->addr = MAXIM_AL32_I2C_ADDRESS;

	i2c_set_clientdata(client, info);
	info->client = client;

	maxim_al32_i2c_client = info->client;
	maxim_al32_i2c_client->addr = MAXIM_AL32_I2C_ADDRESS;

	if (maxim_al32_read_id(&maxim_al32_id_reg) < 0)
		return -ENODEV;

	printk(KERN_INFO "Maxim AL32 detected, chip_id=0x%x\n", maxim_al32_id_reg);

	ret = request_irq(maxim_al32_i2c_client->irq, maxim_al32_irq, IRQ_TYPE_EDGE_RISING,
				"maxim_al32", NULL);
	if (ret != 0) {
		printk(KERN_ERR "Failed to request IRQ %d: %d\n",
				maxim_al32_i2c_client->irq, ret);
	}

#ifdef MAXIM_AL32_DEBUG
	for (i=0; i<=9; i++) {
		ret = maxim_al32_i2c_read(i, &value);
		printk(KERN_INFO "maxim_al32: reg=%d, value=0x%x\n", i, value);
	}
#endif
	/* Turn on ADC_EN */
	ret = maxim_al32_i2c_read(MAXIM_SYSCTRL2, &value);
	value |= 0x1;
	ret = maxim_al32_i2c_write(MAXIM_SYSCTRL2, value);
	mdelay(5);
	ret = maxim_al32_i2c_read(MAXIM_SYSCTRL2, &value);

	if (value & 0x1)
		printk(KERN_INFO "maxim_al32: ADC enabled for serial detect\n");

	/* turn on interrupts and low power mode */
	ret = maxim_al32_i2c_read(MAXIM_SYSCTRL1, &value);
	value |= 0xF1;
	ret = maxim_al32_i2c_write(MAXIM_SYSCTRL1, value);

	/* Unmask */
	ret = maxim_al32_i2c_read(MAXIM_INTMASK1, &value);
	value |= 0x3;
	ret = maxim_al32_i2c_write(MAXIM_INTMASK1, value);

	ret = maxim_al32_i2c_read(MAXIM_INTMASK2, &value);
	value |= 0x1;
	ret = maxim_al32_i2c_write(MAXIM_INTMASK2, value);

	/* See if there is a serial connected */
	ret = maxim_al32_i2c_read(MAXIM_INTSTATUS2, &value);
	if (value & 0x1f)
		printk(KERN_INFO "maxim_al32: Detected a serial connection 0x%x\n", value);

#ifdef MAXIM_AL32_DEBUG
	for (i=0; i<=9; i++) {
		ret = maxim_al32_i2c_read(i, &value);
		printk(KERN_INFO "maxim_al32: reg=%d, value=0x%x\n", i, value);
	}
#endif

	return 0;
}

static int maxim_al32_remove(struct i2c_client *client)
{
	struct maxim_al32_info *info = i2c_get_clientdata(client);

	i2c_set_clientdata(client, info);
	return 0;
}

static struct i2c_driver maxim_al32_i2c_driver = {
	.driver = {
			.name = DRIVER_NAME,
		},
	.probe = maxim_al32_probe,
	.remove = maxim_al32_remove,
	.id_table = maxim_al32_id,
};

static unsigned short normal_i2c[] = { MAXIM_AL32_I2C_ADDRESS, I2C_CLIENT_END };
I2C_CLIENT_INSMOD;


static int __init maxim_al32_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&maxim_al32_i2c_driver);
	if (ret) {
		printk(KERN_ERR "maxim_al32: Could not add driver\n");
		return ret;
	}

	return 0;
}

static void __exit maxim_al32_exit(void)
{
	i2c_del_driver(&maxim_al32_i2c_driver);
}

module_init(maxim_al32_init);
module_exit(maxim_al32_exit);

MODULE_DESCRIPTION("Maxim AL32 Driver");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
