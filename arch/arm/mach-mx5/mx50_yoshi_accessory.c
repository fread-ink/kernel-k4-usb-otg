/*
 * mx50_accessory -- MX50 Accessory Port
 *
 * Copyright 2009-2011 Amazon Technologies Inc., All rights reserved
 *
 * Support for the MX50 Accessory Port
 */

#include <linux/types.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/pmic_external.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/sysdev.h>
#include <linux/device.h>
#include <linux/pmic_external.h>
#include <linux/reboot.h>

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/io.h>
#include <asm/irq.h>

/*
 * How this is supposed to work (by Eric)
 *
 * Active/Reading:
 * 
 *	Enable Apollo unless ACC_PG is high and a USB charger is also present
 *	Update charging icon based on ACC_CHRG_B (if desired)
 *
 * Suspend/Screensaver:
 *
 *	Disable Apollo when entering screensaver unless ACC_PG is high and a USB charger is not present
 *	Set MX508 to wake up based on changes in ACC_PG:
 *		If ACC_PG goes high, enable Apollo.
 *		If ACC_PG goes low, disable Apollo
 *
 * Ship Mode:
 *
 *	No special SW logic needed.
 *
 */

extern int gpio_accessory_get_irq(void);
extern int gpio_accessory_charger_detected(void);
extern int pmic_connected_charger(void);

int accessory_in_screensaver = 0;

#define CHGDETS				0x40	/* Charger detect bit */

#define ACC_THRESHOLD			10	/* 10 ms */

#define GPO4_ENABLE			1
#define GPO4_DISABLE			0

#define GPO3_ENABLE			1
#define GPO3_DISABLE			0

#define GPO4_LSH			12
#define GPO3_LSH			10

#define GPO4_MASK			1
#define GPO3_MASK			1

#define VIOHI_EN			1
#define VIOHI_EN_LSH			3
#define VIOHI_EN_MASK			1

static int mx50_accessory_reboot(struct notifier_block *self, unsigned long action, void *cpu);

static struct notifier_block mx50_accessory_reboot_nb =
{
	.notifier_call = mx50_accessory_reboot,
};

/*!
 * This function sets the accessory voltage
 * on Atlas GPO3
 */
static void accessory_regulator_gpo3_voltage(int enable)
{
	int ret = 0;

	pr_debug("%s: %d\n", __func__, enable);

	if (enable) {
		ret = pmic_write_reg(REG_POWER_MISC, (GPO3_DISABLE << GPO3_LSH),
				(GPO3_MASK << GPO3_LSH));
	}
	else {
		ret = pmic_write_reg(REG_POWER_MISC, (GPO3_ENABLE << GPO3_LSH),
				(GPO3_MASK << GPO3_LSH));
	}
}

/*
 * disable/enable are intended for use by the USB driver
 * disable will only disable Apollo if there is an accessory charger connected
 * otherwise the light goes out when you plug in a USB cable (TEQ-7952)
 */
void accessory_charger_disable(void)
{
    pr_debug("%s: %d\n", __func__, gpio_accessory_charger_detected());

    /* Check presence of an accessory charger, otherwise we don't need to disable */
    if (gpio_accessory_charger_detected()) {
	accessory_regulator_gpo3_voltage(0);
    }
}
EXPORT_SYMBOL(accessory_charger_disable);	

void accessory_charger_enable(void)
{
    pr_debug("%s: acc=%d usb=%d ss=%d\n", __func__, gpio_accessory_charger_detected(), pmic_connected_charger(), accessory_in_screensaver);

    /* Don't enable Apollo if USB charger and accessory charger are present */
    if (pmic_connected_charger() && gpio_accessory_charger_detected())
	return;

    /* Don't enable Apollo if in screensaver and we don't have a accessory charger connected */
    if (!gpio_accessory_charger_detected() && accessory_in_screensaver)
	return;
       
    accessory_regulator_gpo3_voltage(1);
}
EXPORT_SYMBOL(accessory_charger_enable);	

static void do_accessory_work(struct work_struct *dummy)
{
    int irq = gpio_accessory_get_irq();
    
    pr_debug("%s: acc=%d\n", __func__, gpio_accessory_charger_detected());

    if (gpio_accessory_charger_detected()) {
	/* Charger was inserted */
	set_irq_type(irq, IRQF_TRIGGER_FALLING);
	accessory_charger_enable();
    } else {
	/* Charger was removed */
	set_irq_type(irq, IRQF_TRIGGER_RISING);
	accessory_charger_disable();
    }
}
static DECLARE_DELAYED_WORK(accessory_work, do_accessory_work);

static irqreturn_t accessory_port_irq(int irq, void *devid)
{
	/* Debounce for 10 ms */
	schedule_delayed_work(&accessory_work, msecs_to_jiffies(ACC_THRESHOLD));

	return IRQ_HANDLED;
}

#define ACCESSORY_DISABLE	0
#define ACCESSORY_ENABLE	1
#define ACCESSORY_SHUTDOWN	2

static ssize_t
mx50_store_accessory_state(struct sys_device *dev, struct sysdev_attribute *attr,
				const char *buf, size_t size)
{
	int value;

	if (sscanf(buf, "%d", &value) > 0) {
	    
	    switch (value) {
	      case ACCESSORY_DISABLE:
		pr_debug("%s: Accessory port suspended\n", __func__);

		/* don't shut off Apollo if a acc charger is connected */
		if (!(gpio_accessory_charger_detected() && !pmic_connected_charger())) {
		    accessory_regulator_gpo3_voltage(0);
		}
		
		accessory_in_screensaver = 1;
		return strlen(buf);

	      case ACCESSORY_ENABLE:
		pr_debug("%s: Accessory port enabled\n", __func__);

		accessory_in_screensaver = 0;
		accessory_charger_enable();

		return strlen(buf);

	      case ACCESSORY_SHUTDOWN:
		pr_debug("%s: Accessory off\n", __func__);
		accessory_regulator_gpo3_voltage(0);
		return strlen(buf);
	    }
	}

	return -EINVAL;
}
static SYSDEV_ATTR(mx50_accessory_state, 0644, NULL, mx50_store_accessory_state);

static struct sysdev_class mx50_accessory_sysclass = {
	.name = "mx50_accessory",
};

static struct sys_device mx50_accessory_device = {
	.id = 0,
	.cls = &mx50_accessory_sysclass,
};

static int mx50_accessory_sysdev_ctrl_init(void)
{
	int err = 0;

	err = sysdev_class_register(&mx50_accessory_sysclass);
	if (!err)
		err = sysdev_register(&mx50_accessory_device);
	if (!err) {
		sysdev_create_file(&mx50_accessory_device, &attr_mx50_accessory_state);
	}

	return err;
}

static void mx50_accessory_sysdev_ctrl_exit(void)
{
	sysdev_remove_file(&mx50_accessory_device, &attr_mx50_accessory_state);
	sysdev_unregister(&mx50_accessory_device);
	sysdev_class_unregister(&mx50_accessory_sysclass);
}

/* Called from pm.c */
int accessory_port_suspend(void)
{ 
    int irq = gpio_accessory_get_irq();

    pr_debug("%s: acc=%d usb=%d\n", __func__, gpio_accessory_charger_detected(), pmic_connected_charger());

    /* Keep Apollo enabled if acc charger and no USB, otherwise disable 
     *   on suspend */
    if (gpio_accessory_charger_detected() && !pmic_connected_charger()) {
	accessory_regulator_gpo3_voltage(1);
    } else {
	accessory_regulator_gpo3_voltage(0);
    }
    
    return 0;
}
EXPORT_SYMBOL(accessory_port_suspend);

/* Called from pm.c */
int accessory_port_resume(void)
{
    int irq = gpio_accessory_get_irq();

    pr_debug("%s: begin\n", __func__);

    accessory_charger_enable();

    return 0;
}
EXPORT_SYMBOL(accessory_port_resume);

static int __init accessory_port_init(void)
{
	int err = 0, irq = gpio_accessory_get_irq();

	printk(KERN_DEBUG "Initializing MX50 Yoshi Accessory Port\n");

	err = request_irq(irq, accessory_port_irq, IRQF_DISABLED, "MX50_Accessory", NULL);
	if (err != 0) {
		printk(KERN_ERR "IRQF_DISABLED: Could not get IRQ %d\n", irq);
		return err;
	}

	disable_irq(irq);

	/* Set the correct IRQ trigger based on accessory charger state */
	if (gpio_accessory_charger_detected())
	    set_irq_type(irq, IRQF_TRIGGER_FALLING);
	else
	    set_irq_type(irq, IRQF_TRIGGER_RISING);
	
	enable_irq(irq);

	/* always enable Apollo while device is running */
	accessory_charger_enable();

	if (mx50_accessory_sysdev_ctrl_init() < 0)
		printk(KERN_ERR "mx50_accessory: Could not create sysfs interface\n");

	register_reboot_notifier(&mx50_accessory_reboot_nb);
	
	pmic_write_reg(REG_MODE_0, VIOHI_EN << VIOHI_EN_LSH, VIOHI_EN_MASK << VIOHI_EN_LSH);
	return err;
}

static void accessory_port_cleanup(void)
{
	accessory_regulator_gpo3_voltage(0);

	free_irq(gpio_accessory_get_irq(), NULL);
	mx50_accessory_sysdev_ctrl_exit();

	pr_debug("%s: end\n", __func__);
}

static int mx50_accessory_reboot(struct notifier_block *self, unsigned long action, void *cpu)
{
	printk(KERN_DEBUG "MX50 Accessory shutdown\n");
	accessory_port_cleanup();
	return 0;
}

static void __exit accessory_port_remove(void)
{
	accessory_port_cleanup();
	unregister_reboot_notifier(&mx50_accessory_reboot_nb);

	pr_debug("%s: end\n", __func__);
}

module_init(accessory_port_init);
module_exit(accessory_port_remove);

MODULE_DESCRIPTION("MX50 Accessory Port");
MODULE_LICENSE("GPL");
