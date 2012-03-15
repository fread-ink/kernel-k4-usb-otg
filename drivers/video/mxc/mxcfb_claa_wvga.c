/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup Framebuffer Framebuffer Driver for SDC and ADC.
 */

/*!
 * @file mxcfb_claa_wvga.c
 *
 * @brief MXC Frame buffer driver for SDC
 *
 * @ingroup Framebuffer
 */

/*!
 * Include files
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mxcfb.h>
#include <linux/regulator/consumer.h>
#include <mach/hardware.h>

static void lcd_poweron(void);
static void lcd_poweroff(void);

static struct platform_device *plcd_dev;
static struct regulator *io_reg;
static struct regulator *core_reg;
static int lcd_on;

static struct fb_videomode video_modes[] = {
	{
	 /* 800x480 @ 55 Hz , pixel clk @ 25MHz */
	 "CLAA-WVGA", 55, 800, 480, 40000, 40, 40, 5, 5, 20, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

static void lcd_init_fb(struct fb_info *info)
{
	struct fb_var_screeninfo var;

	memset(&var, 0, sizeof(var));

	fb_videomode_to_var(&var, &video_modes[0]);

	var.activate = FB_ACTIVATE_ALL;
	var.yres_virtual = var.yres * 3;

	acquire_console_sem();
	info->flags |= FBINFO_MISC_USEREVENT;
	fb_set_var(info, &var);
	info->flags &= ~FBINFO_MISC_USEREVENT;
	release_console_sem();
}

static int lcd_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;

	if (strcmp(event->info->fix.id, "DISP3 BG")) {
		return 0;
	}

	switch (val) {
	case FB_EVENT_FB_REGISTERED:
		lcd_init_fb(event->info);
		lcd_poweron();
		break;
	case FB_EVENT_BLANK:
		if ((event->info->var.xres != 800) ||
		    (event->info->var.yres != 480)) {
			break;
		}
		if (*((int *)event->data) == FB_BLANK_UNBLANK) {
			lcd_poweron();
		} else {
			lcd_poweroff();
		}
		break;
	}
	return 0;
}

static struct notifier_block nb = {
	.notifier_call = lcd_fb_event,
};

/*!
 * This function is called whenever the SPI slave device is detected.
 *
 * @param	spi	the SPI slave device
 *
 * @return 	Returns 0 on SUCCESS and error on FAILURE.
 */
static int __devinit lcd_probe(struct platform_device *pdev)
{
	int i;
	struct mxc_lcd_platform_data *plat = pdev->dev.platform_data;

	if (plat) {
		if (plat->reset)
			plat->reset();

		io_reg = regulator_get(&pdev->dev, plat->io_reg);
		if (IS_ERR(io_reg))
			io_reg = NULL;
		core_reg = regulator_get(&pdev->dev, plat->core_reg);
		if (!IS_ERR(core_reg)) {
			regulator_set_voltage(io_reg, 1800000, 1800000);
		} else {
			core_reg = NULL;
		}
	}

	for (i = 0; i < num_registered_fb; i++) {
		if (strcmp(registered_fb[i]->fix.id, "DISP3 BG") == 0) {
			lcd_init_fb(registered_fb[i]);
			fb_show_logo(registered_fb[i], 0);
			lcd_poweron();
		} else if (strcmp(registered_fb[i]->fix.id, "DISP3 FG") == 0) {
			lcd_init_fb(registered_fb[i]);
		}
	}

	fb_register_client(&nb);

	plcd_dev = pdev;

	return 0;
}

static int __devexit lcd_remove(struct platform_device *pdev)
{
	fb_unregister_client(&nb);
	lcd_poweroff();
	if (io_reg)
		regulator_put(io_reg);
	if (core_reg)
		regulator_put(core_reg);

	return 0;
}

#ifdef CONFIG_PM
static int lcd_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int lcd_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define lcd_suspend NULL
#define lcd_resume NULL
#endif

/*!
 * platform driver structure for CLAA WVGA
 */
static struct platform_driver lcd_driver = {
	.driver = {
		   .name = "lcd_claa"},
	.probe = lcd_probe,
	.remove = __devexit_p(lcd_remove),
	.suspend = lcd_suspend,
	.resume = lcd_resume,
};

/*
 * Send Power On commands to L4F00242T03
 *
 */
static void lcd_poweron(void)
{
	if (lcd_on)
		return;

	dev_dbg(&plcd_dev->dev, "turning on LCD\n");
	if (core_reg)
		regulator_enable(core_reg);
	if (io_reg)
		regulator_enable(io_reg);
	lcd_on = 1;
}

/*
 * Send Power Off commands to L4F00242T03
 *
 */
static void lcd_poweroff(void)
{
	lcd_on = 0;
	dev_dbg(&plcd_dev->dev, "turning off LCD\n");
	if (io_reg)
		regulator_disable(io_reg);
	if (core_reg)
		regulator_disable(core_reg);
}

static int __init claa_lcd_init(void)
{
	return platform_driver_register(&lcd_driver);
}

static void __exit claa_lcd_exit(void)
{
	platform_driver_unregister(&lcd_driver);
}

module_init(claa_lcd_init);
module_exit(claa_lcd_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("CLAA WVGA LCD init driver");
MODULE_LICENSE("GPL");
