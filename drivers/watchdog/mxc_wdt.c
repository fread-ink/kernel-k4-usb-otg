/*
 * linux/drivers/char/watchdog/mxc_wdt.c
 *
 * Watchdog driver for FSL MXC. It is based on omap1610_wdt.c
 *
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * 2005 (c) MontaVista Software, Inc.  All Rights Reserved.
 *
 * Add support for in-kernel watchdog
 *
 * Copyright 2009-2011 Amazon Technologies, Inc. All Rights Reserved.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 *
 * 20051207: <AKuster@mvista.com>
 *	     	Full rewrite based on
 *		linux-2.6.15-rc5/drivers/char/watchdog/omap_wdt.c
 *	     	Add platform resource support
 *
 */

/*!
 * @defgroup WDOG Watchdog Timer (WDOG) Driver
 */
/*!
 * @file mxc_wdt.c
 *
 * @brief Watchdog timer driver
 *
 * @ingroup WDOG
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/smp_lock.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <linux/clk.h>
#include <linux/timer.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/bitops.h>

#include "mxc_wdt.h"
#define DVR_VER "2.0"

#define WDOG_SEC_TO_COUNT(s)  (((s * 2) + 1) << 8)
#define WDOG_COUNT_TO_SEC(c)  (((c >> 8) - 1) / 2)

#define WATCHDOG_PING_THRESHOLD 	10000	/* 10 second refresh rate */
static void __iomem *wdt_base_reg;
static int mxc_wdt_users;
static struct clk *mxc_wdt_clk;

static unsigned timer_margin = TIMER_MARGIN_DEFAULT;
module_param(timer_margin, uint, 0);
MODULE_PARM_DESC(timer_margin, "initial watchdog timeout (in seconds)");

static unsigned dev_num = 0;

static void do_watchdog_timer(unsigned long unused);
static struct timer_list wdt_timer;

static int mxc_watchdog_stopped = 0;

/*
 * TESTING ONLY
 *
 * Provide a way to increase the timeout when running stress tests
 */

static ssize_t
show_mxc_wdt_timeout(struct device *dev, struct device_attribute *attr, char *buf);

static ssize_t
store_mxc_wdt_timeout(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static DEVICE_ATTR(mxc_wdt_timeout, 0666, show_mxc_wdt_timeout, store_mxc_wdt_timeout);

static ssize_t
store_mxc_wdt_keepalive(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(mxc_wdt_keepalive, 0666, NULL, store_mxc_wdt_keepalive);

static void mxc_wdt_ping(void *base)
{
	/* issue the service sequence instructions */
	__raw_writew(WDT_MAGIC_1, base + MXC_WDT_WSR);
	__raw_writew(WDT_MAGIC_2, base + MXC_WDT_WSR);
}

static void do_watchdog_timer(unsigned long unused)
{
	mxc_wdt_ping(wdt_base_reg);
	if (!mxc_watchdog_stopped)
		mod_timer(&wdt_timer, jiffies + msecs_to_jiffies(WATCHDOG_PING_THRESHOLD));
}

static ssize_t
store_mxc_wdt_keepalive(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	mxc_wdt_ping(wdt_base_reg);
	return count;
}

static void mxc_wdt_config(void *base)
{
	u16 val;

	val = WDOG_SEC_TO_COUNT(timer_margin) |
	      WCR_WOE_BIT | WCR_WDA_BIT | WCR_SRS_BIT |
	      WCR_WDZST_BIT | WCR_WDBG_BIT | WCR_WRE_BIT;
	__raw_writew(val, base + MXC_WDT_WCR);
}

static void mxc_wdt_enable(void *base)
{
	u16 val;

	val = __raw_readw(base + MXC_WDT_WCR);
	val |= WCR_WDE_BIT;
	__raw_writew(val, base + MXC_WDT_WCR);
}

static void mxc_wdt_disable(void *base)
{
	/* disable not supported by this chip */
}

static void mxc_wdt_adjust_timeout(unsigned new_timeout)
{
	if (new_timeout < TIMER_MARGIN_MIN)
		new_timeout = TIMER_MARGIN_DEFAULT;
	if (new_timeout > TIMER_MARGIN_MAX)
		new_timeout = TIMER_MARGIN_MAX;
	timer_margin = new_timeout;
}

static u16 mxc_wdt_get_timeout(void *base)
{
	u16 val;

	val = __raw_readw(base + MXC_WDT_WCR);
	return WDOG_COUNT_TO_SEC(val);
}

static u16 mxc_wdt_get_bootreason(void *base)
{
	u16 val;

	val = __raw_readw(base + MXC_WDT_WRSR);
	return val;
}

static void mxc_wdt_set_timeout(void *base)
{
	u16 val;
	val = __raw_readw(base + MXC_WDT_WCR);
	val = (val & 0x00FF) | WDOG_SEC_TO_COUNT(timer_margin);
	__raw_writew(val, base + MXC_WDT_WCR);
	val = __raw_readw(base + MXC_WDT_WCR);
	timer_margin = WDOG_COUNT_TO_SEC(val);
}

/*
 *	Allow only one task to hold it open
 */

static int mxc_wdt_open(struct inode *inode, struct file *file)
{

	if (test_and_set_bit(1, (unsigned long *)&mxc_wdt_users))
		return -EBUSY;

	mxc_wdt_config(wdt_base_reg);
	mxc_wdt_set_timeout(wdt_base_reg);
	mxc_wdt_enable(wdt_base_reg);
	mxc_wdt_ping(wdt_base_reg);

	return 0;
}

static int mxc_wdt_release(struct inode *inode, struct file *file)
{
	/*
	 *      Shut off the timer unless NOWAYOUT is defined.
	 */
#ifndef CONFIG_WATCHDOG_NOWAYOUT
	mxc_wdt_disable(wdt_base_reg);

#else
	printk(KERN_CRIT "mxc_wdt: Unexpected close, not stopping!\n");
#endif
	mxc_wdt_users = 0;
	return 0;
}

static ssize_t
mxc_wdt_write(struct file *file, const char __user * data,
	      size_t len, loff_t * ppos)
{
	/* Refresh LOAD_TIME. */
	if (len)
		mxc_wdt_ping(wdt_base_reg);
	return len;
}

static int
mxc_wdt_ioctl(struct inode *inode, struct file *file,
	      unsigned int cmd, unsigned long arg)
{
	int new_margin;
	int bootr;

	static struct watchdog_info ident = {
		.identity = "MXC Watchdog",
		.options = WDIOF_SETTIMEOUT,
		.firmware_version = 0,
	};

	switch (cmd) {
	default:
		return -ENOIOCTLCMD;
	case WDIOC_GETSUPPORT:
		return copy_to_user((struct watchdog_info __user *)arg, &ident,
				    sizeof(ident));
	case WDIOC_GETSTATUS:
		return put_user(0, (int __user *)arg);
	case WDIOC_GETBOOTSTATUS:
		bootr = mxc_wdt_get_bootreason(wdt_base_reg);
		return put_user(bootr, (int __user *)arg);
	case WDIOC_KEEPALIVE:
		mxc_wdt_ping(wdt_base_reg);
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, (int __user *)arg))
			return -EFAULT;

		mxc_wdt_adjust_timeout(new_margin);
		mxc_wdt_disable(wdt_base_reg);
		mxc_wdt_set_timeout(wdt_base_reg);
		mxc_wdt_enable(wdt_base_reg);
		mxc_wdt_ping(wdt_base_reg);
		return 0;

	case WDIOC_GETTIMEOUT:
		mxc_wdt_ping(wdt_base_reg);
		new_margin = mxc_wdt_get_timeout(wdt_base_reg);
		return put_user(new_margin, (int __user *)arg);
	}
}

static ssize_t
show_mxc_wdt_timeout(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", timer_margin);
}

static ssize_t
store_mxc_wdt_timeout(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int new_margin;

	if (sscanf(buf, "%d", &new_margin) != 1)
		return -EINVAL;

	if (new_margin > 127)
		new_margin = 127;

	mxc_wdt_adjust_timeout(new_margin);
	mxc_wdt_disable(wdt_base_reg);
	mxc_wdt_set_timeout(wdt_base_reg);
	mxc_wdt_enable(wdt_base_reg);
	mxc_wdt_ping(wdt_base_reg);
	
	return count;
}

static struct file_operations mxc_wdt_fops = {
	.owner = THIS_MODULE,
	.write = mxc_wdt_write,
	.ioctl = mxc_wdt_ioctl,
	.open = mxc_wdt_open,
	.release = mxc_wdt_release,
};

static struct miscdevice mxc_wdt_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name = "watchdog",
	.fops = &mxc_wdt_fops
};

void mxc_wdt_reset(void)
{
        u16 reg;

	clk_enable(mxc_wdt_clk);

        reg = __raw_readw(wdt_base_reg + MXC_WDT_WCR) & ~WCR_SRS_BIT;
        reg |= WCR_WDE_BIT;
        __raw_writew(reg, wdt_base_reg + MXC_WDT_WCR);
}

static ssize_t wdog_rst_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0) {
		printk(KERN_ERR "Error invoking watchdog reset\n");
		return -EINVAL;
	}

	mxc_wdt_reset();
	return size;
}
static DEVICE_ATTR(wdog_rst, 0644, NULL, wdog_rst_store);

static int __init mxc_wdt_probe(struct platform_device *pdev)
{
	struct resource *res, *mem;
	int ret;

	/* reserve static register mappings */
	res = platform_get_resource(pdev, IORESOURCE_MEM, dev_num);
	if (!res)
		return -ENOENT;

	mem = request_mem_region(res->start, res->end - res->start + 1,
				 pdev->name);
	if (mem == NULL)
		return -EBUSY;

	platform_set_drvdata(pdev, mem);

	wdt_base_reg = ioremap(res->start, res->end - res->start + 1);
	mxc_wdt_disable(wdt_base_reg);
	mxc_wdt_adjust_timeout(timer_margin);

	mxc_wdt_users = 0;

	mxc_wdt_miscdev.this_device = &pdev->dev;

	mxc_wdt_clk = clk_get(NULL, "wdog_clk");
	clk_enable(mxc_wdt_clk);

	ret = misc_register(&mxc_wdt_miscdev);
	if (ret)
		goto fail;

	pr_info("MXC Watchdog # %d Timer: initial timeout %d sec\n", dev_num,
		timer_margin);

	mxc_wdt_config(wdt_base_reg);
	mxc_wdt_set_timeout(wdt_base_reg);
	mxc_wdt_enable(wdt_base_reg);
	mxc_wdt_ping(wdt_base_reg);

	init_timer(&wdt_timer);	
	wdt_timer.function = do_watchdog_timer;
	mod_timer(&wdt_timer, jiffies + msecs_to_jiffies(WATCHDOG_PING_THRESHOLD));
	pr_info("MXC Watchdog: Started %d millisecond watchdog refresh\n", WATCHDOG_PING_THRESHOLD);

	if (device_create_file(&pdev->dev, &dev_attr_mxc_wdt_timeout) < 0)
		printk(KERN_ERR "watchdog: error creating mxc_wdt_timeout file\n");

	if (device_create_file(&pdev->dev, &dev_attr_mxc_wdt_keepalive) < 0)
		printk(KERN_ERR "watchdog: error creating mxc_wdt_keepalive file\n");

	if (device_create_file(&pdev->dev, &dev_attr_wdog_rst) < 0)
		printk (KERN_ERR "Error - could not create wdog_rst file\n");

	return 0;

      fail:
	release_resource(mem);
	pr_info("MXC Watchdog Probe failed\n");
	return ret;
}

static void mxc_wdt_shutdown(struct platform_device *pdev)
{
	mxc_wdt_disable(wdt_base_reg);
	pr_info("MXC Watchdog # %d shutdown\n", dev_num);
	del_timer_sync(&wdt_timer);
}

static int __exit mxc_wdt_remove(struct platform_device *pdev)
{
	struct resource *mem = platform_get_drvdata(pdev);
	misc_deregister(&mxc_wdt_miscdev);
	release_resource(mem);
	pr_info("MXC Watchdog # %d removed\n", dev_num);
	device_remove_file(&pdev->dev, &dev_attr_mxc_wdt_timeout);
	device_remove_file(&pdev->dev, &dev_attr_mxc_wdt_keepalive);
	device_remove_file(&pdev->dev, &dev_attr_wdog_rst);
	del_timer_sync(&wdt_timer);
	return 0;
}

#ifdef CONFIG_PM

/*
 * Cannot stop a running watchdog. The watchdog automatically stops counting
 * in state retention mode. We just cancel the timer that pings
 * the watchdog.
 */
static int mxc_wdt_suspend(struct platform_device *pdev, pm_message_t state)
{
	mxc_watchdog_stopped = 1;
	del_timer_sync(&wdt_timer);
	return 0;
}

static int mxc_wdt_resume(struct platform_device *pdev)
{
	mxc_watchdog_stopped = 0;
	mxc_wdt_ping(wdt_base_reg);
	mod_timer(&wdt_timer, jiffies + msecs_to_jiffies(WATCHDOG_PING_THRESHOLD));
	return 0;
}

#else /* CONFIG_PM */

#define	mxc_wdt_suspend	NULL
#define	mxc_wdt_resume	NULL

#endif /* CONFIG_PM */

static struct platform_driver mxc_wdt_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "mxc_wdt",
		   },
	.probe = mxc_wdt_probe,
	.shutdown = mxc_wdt_shutdown,
	.remove = __exit_p(mxc_wdt_remove),
	.suspend = mxc_wdt_suspend,
	.resume = mxc_wdt_resume,
};

static int __init mxc_wdt_init(void)
{
	pr_info("MXC WatchDog Driver %s\n", DVR_VER);

	if ((timer_margin < TIMER_MARGIN_MIN) ||
	    (timer_margin > TIMER_MARGIN_MAX)) {
		pr_info("MXC watchdog error. wrong timer_margin %d\n",
			timer_margin);
		pr_info("    Range: %d to %d seconds\n", TIMER_MARGIN_MIN,
			TIMER_MARGIN_MAX);
		return -EINVAL;
	}

	return platform_driver_register(&mxc_wdt_driver);
}

static void __exit mxc_wdt_exit(void)
{
	platform_driver_unregister(&mxc_wdt_driver);
	pr_info("MXC WatchDog Driver removed\n");
}

module_init(mxc_wdt_init);
module_exit(mxc_wdt_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
