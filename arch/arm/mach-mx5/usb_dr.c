/*
 * Copyright (C) 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <mach/arc_otg.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include "usb.h"


static int usbotg_init_ext(struct platform_device *pdev);
static void usbotg_uninit_ext(struct fsl_usb2_platform_data *pdata);
static void usbotg_clock_gate(bool on);

/*
 * platform data structs
 * 	- Which one to use is determined by CONFIG options in usb.h
 * 	- operating_mode plugged at run time
 */
static struct fsl_usb2_platform_data dr_utmi_config = {
	.name              = "DR",
	.platform_init     = usbotg_init_ext,
	.platform_uninit   = usbotg_uninit_ext,
	.phy_mode          = FSL_USB2_PHY_UTMI_WIDE,
	.power_budget      = 500,		/* 500 mA max power */
	.gpio_usb_active   = gpio_usbotg_hs_active,
	.gpio_usb_inactive = gpio_usbotg_hs_inactive,
	.transceiver       = "utmi",
};

/* Notes: configure USB clock*/
static int usbotg_init_ext(struct platform_device *pdev)
{
	struct clk *usb_clk;
	int ret = 0;

	usb_clk = clk_get(NULL, "usb_phy1_clk");
	clk_enable(usb_clk);
	clk_put(usb_clk);

	ret = usbotg_init(pdev);

	return ret;
}

static void usbotg_uninit_ext(struct fsl_usb2_platform_data *pdata)
{
	struct clk *usb_clk;

	usbotg_uninit(pdata);

	usb_clk = clk_get(NULL, "usb_phy1_clk");
	clk_disable(usb_clk);
	clk_put(usb_clk);
}

static void usbotg_clock_gate(bool on)
{
	struct clk *usb_clk;

	if (on) {
		usb_clk = clk_get(NULL, "usb_ahb_clk");
		clk_enable(usb_clk);
		clk_put(usb_clk);

		usb_clk = clk_get(NULL, "usb_phy1_clk");
		clk_enable(usb_clk);
		clk_put(usb_clk);
	} else {
		usb_clk = clk_get(NULL, "usb_phy1_clk");
		clk_disable(usb_clk);
		clk_put(usb_clk);

		usb_clk = clk_get(NULL, "usb_ahb_clk");
		clk_disable(usb_clk);
		clk_put(usb_clk);
	}
}

void mx5_set_otghost_vbus_func(driver_vbus_func driver_vbus)
{
	dr_utmi_config.platform_driver_vbus = driver_vbus;
}

struct usb_dr_device {
	/* Capability register */
	u32 id;
	u32 res1[35];
	u32 sbuscfg;            /* sbuscfg ahb burst */
	u32 res11[27];
	u16 caplength;          /* Capability Register Length */
	u16 hciversion;         /* Host Controller Interface Version */
	u32 hcsparams;          /* Host Controller Structual Parameters */
	u32 hccparams;          /* Host Controller Capability Parameters */
	u32 res2[5];
	u32 dciversion;         /* Device Controller Interface Version */
	u32 dccparams;          /* Device Controller Capability Parameters */
	u32 res3[6];
	/* Operation register */
	u32 usbcmd;             /* USB Command Register */
	u32 usbsts;             /* USB Status Register */
	u32 usbintr;            /* USB Interrupt Enable Register */
	u32 frindex;            /* Frame Index Register */
	u32 res4;
	u32 deviceaddr;         /* Device Address */
	u32 endpointlistaddr;   /* Endpoint List Address Register */
	u32 res5;
	u32 burstsize;          /* Master Interface Data Burst Size Register */
	u32 txttfilltuning;     /* Transmit FIFO Tuning Controls Register */
	u32 res6[6];
	u32 configflag;         /* Configure Flag Register */
	u32 portsc1;            /* Port 1 Status and Control Register */
	u32 res7[7];
	u32 otgsc;              /* On-The-Go Status and Control */
	u32 usbmode;            /* USB Mode Register */
	u32 endptsetupstat;     /* Endpoint Setup Status Register */
	u32 endpointprime;      /* Endpoint Initialization Register */
	u32 endptflush;         /* Endpoint Flush Register */
	u32 endptstatus;        /* Endpoint Status Register */
	u32 endptcomplete;      /* Endpoint Complete Register */
	u32 endptctrl[8 * 2];   /* Endpoint Control Registers */
};

unsigned int usbdr_early_portsc1_read(void)
{
	volatile struct usb_dr_device *dr_regs;
	int start = USB_OTGREGS_BASE;
	int end = USB_OTGREGS_BASE + 0x1ff;
	unsigned int portsc1;
	unsigned int temp;
	unsigned long timeout;

	if (!request_mem_region(start, (end - start + 1), "fsl-usb-dr"))
	        return -1;

	dr_regs = ioremap(start, (end - start + 1));
	if (!dr_regs) {
	        release_mem_region(start, (end - start + 1));
	        return -1;
	}

	/* Stop and reset the usb controller */
	temp = readl(&dr_regs->usbcmd);
	temp &= ~0x1;
	writel(temp, &dr_regs->usbcmd);

	temp = readl(&dr_regs->usbcmd);
	temp |= 0x2;
	writel(temp, &dr_regs->usbcmd);

	/* Wait for reset to complete */
	timeout = jiffies + 1000;
	while (readl(&dr_regs->usbcmd) & 0x2) {
	        if (time_after(jiffies, timeout)) {
	                printk("udc reset timeout! \n");
	                return -ETIMEDOUT;
	        }
	        cpu_relax();
	}

	/* Set the controller as device mode */
	temp = readl(&dr_regs->usbmode);
	temp &= ~0x3;        /* clear mode bits */
	temp |= 0x2;
	/* Disable Setup Lockout */
	temp |= 0x8;
	writel(temp, &dr_regs->usbmode);

	/* Config PHY interface */
	portsc1 = readl(&dr_regs->portsc1);
	portsc1 &= ~(0xc0000000 | 0x10000000 | 0x80) | 0x10000000 | 0x40;
	writel(portsc1, &dr_regs->portsc1);

	/* Set controller to Run */
	temp = readl(&dr_regs->usbcmd);
	temp |= 0x1;
	writel(temp, &dr_regs->usbcmd);

	mdelay(100);

	/* Get the line status bits */
	portsc1 = readl(&dr_regs->portsc1) & 0xc00;

	/* disable all INTR */
	writel(0, &dr_regs->usbintr);

	/* set controller to Stop */
	temp = readl(&dr_regs->usbcmd);
	temp &= ~0x1;
	writel(temp, &dr_regs->usbcmd);

	release_mem_region(start, (end - start + 1));
	return portsc1;
}
EXPORT_SYMBOL(usbdr_early_portsc1_read);

void __init mx5_usb_dr_init(void)
{
#ifdef CONFIG_USB_OTG
	dr_utmi_config.operating_mode = FSL_USB2_DR_OTG;
	platform_device_add_data(&mxc_usbdr_otg_device, &dr_utmi_config, sizeof(dr_utmi_config));
	platform_device_register(&mxc_usbdr_otg_device);
#endif
#ifdef CONFIG_USB_EHCI_ARC_OTG
	dr_utmi_config.operating_mode = DR_HOST_MODE;
	platform_device_add_data(&mxc_usbdr_host_device, &dr_utmi_config, sizeof(dr_utmi_config));
	platform_device_register(&mxc_usbdr_host_device);
#endif
#ifdef CONFIG_USB_GADGET_ARC
	dr_utmi_config.operating_mode = DR_UDC_MODE;
	platform_device_add_data(&mxc_usbdr_udc_device, &dr_utmi_config, sizeof(dr_utmi_config));
	platform_device_register(&mxc_usbdr_udc_device);
#endif
}
