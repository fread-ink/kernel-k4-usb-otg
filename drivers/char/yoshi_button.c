/*
 * Amazon MX50 Yoshi Power Button
 * Copyright (C) 2008 Manish Lachwani <lachwani@lab126.com>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pmic_external.h>
#include <linux/pmic_adc.h>
#include <linux/pmic_light.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <mach/clock.h>

/* There are 3 buttons on the Atlas PMIC and we use ONOFD1 */
#define YOSHI_BUTTON_MINOR	158 /* Major 10, Minor 158, /dev/yoshibutton */

/*
 * Uses Atlas PMIC
 */
#include <linux/pmic_external.h>
#include <mach/pmic_power.h>

static int mb_evt[2] = {KOBJ_OFFLINE, KOBJ_ONLINE};

static struct miscdevice button_misc_device = {
	YOSHI_BUTTON_MINOR,
	"yoshibutton",
	NULL,
};


#define LED_BLINK_THRESHOLD	2000	/* Blink for 2 seconds only */
#define CHGDETS			0x40
#define GREEN_LED_MASK		0x3f
#define GREEN_LED_LSH		15

/*
 * Is a charger connected?
 */
static int charger_connected(void)
{
	int sense_0 = 0;
	int ret = 0; /* Default: no charger */

	pmic_read_reg(REG_INT_SENSE0, &sense_0, 0xffffff);
	if (sense_0 & CHGDETS)
		ret = 1;

	return ret;
}

extern int yoshi_button_green_led;

#ifdef CONFIG_MXC_PMIC_MC13892

static void pmic_enable_green_led(int enable)
{
	if (enable) {
		mc13892_bklit_set_current(LIT_KEY, 0x7);
		mc13892_bklit_set_ramp(LIT_KEY, 0);
		mc13892_bklit_set_dutycycle(LIT_KEY, 0x3f);
	}
	else {
		mc13892_bklit_set_current(LIT_KEY, 0);
		mc13892_bklit_set_dutycycle(LIT_KEY, 0);
	}
}
#endif

#ifdef CONFIG_MXC_PMIC_MC34708

static void pmic_enable_green_led(int enable)
{
	if (enable) {
		mc34708_bklit_set_current(LIT_KEY, 0x7);
		mc34708_bklit_set_ramp(LIT_KEY, 0);
		mc34708_bklit_set_dutycycle(LIT_KEY, 0x3f);
	}
	else {
		mc34708_bklit_set_current(LIT_KEY, 0);
		mc34708_bklit_set_dutycycle(LIT_KEY, 0);
	}
}

#endif

static int bp_pressed = 0;

static void led_timer_handle_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(led_timer_work, led_timer_handle_work);

static void led_timer_handle_work(struct work_struct *work)
{
	unsigned int sense;

	pmic_read_reg(REG_INT_SENSE1, &sense, (1 << 3));

	/* Check if button still pressed after 4 seconds */
	if (bp_pressed) {
		/* If button still pressed, blink LED twice */
		if (!(sense & (1 << 3)) ) {
			pmic_enable_green_led(0);
			mdelay(200);
			pmic_enable_green_led(1);
			mdelay(200);
			pmic_enable_green_led(0);
			mdelay(200);
			pmic_enable_green_led(1);
			mdelay(200);
			pmic_enable_green_led(0);
		}
		else
			pmic_enable_green_led(0);

		/* Clear the state */
		bp_pressed = 0;
		yoshi_button_green_led = 0;

		return;
	}

	if (!(sense & (1 << 3))) {
		/* Still pressed after 2 seconds, reschedule workqueue after 2 seconds */
		bp_pressed = 1;
		schedule_delayed_work(&led_timer_work, msecs_to_jiffies(LED_BLINK_THRESHOLD));
	}
	else {
		pmic_enable_green_led(0);
		yoshi_button_green_led = 0;
	}
}

extern void mxcuart_enable_clock(void);

/*
 * Interrupt triggered when button pressed
 */
static void yoshi_button_handler(void *param)
{
	unsigned int sense;
	unsigned int press_event = 0;
	int i;
	int pb_counter = 100;	/* Monitor 15 second battery cut */

	if (!charger_connected()) {
		/* Set the green LED only if charger not connected */
		pmic_enable_green_led(1);	
		yoshi_button_green_led = 1;
		schedule_delayed_work(&led_timer_work, msecs_to_jiffies(LED_BLINK_THRESHOLD));
	}

	/*
	 * reset button - restart
	 */
	pmic_power_set_auto_reset_en(0);
	pmic_power_set_conf_button(BT_ON1B, 0, 2);	
	
	pmic_read_reg(REG_INT_SENSE1, &sense, (1 << 3));

	if (!(sense & (1 << 3))) {
		/* Delay of about 2sec */
		for (i = 0; i < 100; i++) {
			pmic_read_reg(REG_INT_SENSE1, &sense, (1 << 3));

			if (sense & (1 << 3)) {
				press_event = 1;
				break;
			}

			msleep(35);
		}
	} else {
		pmic_power_set_auto_reset_en(1);
		pmic_power_set_conf_button(BT_ON1B, 1, 2);
		press_event = 1;
	}

	if (kobject_uevent(&button_misc_device.this_device->kobj,
			   mb_evt[press_event]))
		printk(KERN_WARNING "yoshibutton: can't send uevent\n");

	if (!press_event)
		do {
			msleep(50);
			if (--pb_counter == 0)
				printk(KERN_EMERG "bcut: C def:pcut:pending 15 second battery cut:\n");
			pmic_read_reg(REG_INT_SENSE1, &sense, (1 << 3));
		} while (!(sense & (1 << 3)));

	/* Atlas N1B interrupt line debouce is 30 ms */
	msleep(40);

	/* ignore release interrupts */
	pmic_write_reg(REG_INT_STATUS1, (1 << 3), (1 << 3));

	pmic_write_reg(REG_INT_MASK1, 0, (1 << 3));

	mxcuart_enable_clock();
}

static int __init yoshi_power_button_init(void)
{
	printk (KERN_INFO "Amazon MX35 Yoshi Power Button Driver\n");

	if (misc_register (&button_misc_device)) {
		printk (KERN_WARNING "yoshibutton: Couldn't register device 10, "
				"%d.\n", YOSHI_BUTTON_MINOR);
		return -EBUSY;
	}

	if (pmic_power_set_conf_button(BT_ON1B, 0, 2)) {
		printk(KERN_WARNING "yoshibutton: can't configure debounce "
		       "time\n");
		misc_deregister(&button_misc_device);
		return -EIO;
	}

	if (pmic_power_event_sub(PWR_IT_ONOFD1I, yoshi_button_handler)) {
		printk(KERN_WARNING "yoshibutton: can't subscribe to IRQ\n");
		misc_deregister(&button_misc_device);
		return -EIO;
	}

	/* Success */
	return 0;
}

static void __exit yoshi_power_button_exit(void)
{
	pmic_power_event_unsub(PWR_IT_ONOFD1I, yoshi_button_handler);
	misc_deregister (&button_misc_device);
}

MODULE_AUTHOR("Manish Lachwani");
MODULE_LICENSE("GPL");

module_init(yoshi_power_button_init);
module_exit(yoshi_power_button_exit);

