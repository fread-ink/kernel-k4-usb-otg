/*
 * Copyright (C) 2005-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Author: Li Yang <LeoLi@freescale.com>
 *         Jerry Huang <Chang-Ming.Huang@freescale.com>
 *
 * Initialization based on code from Shlomi Gridish.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/proc_fs.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/reboot.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/usb.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/usb/fsl_xcvr.h>
#include <linux/fsl_devices.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/pmic_external.h>
#include <mach/charger.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/byteorder.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>
#include "fsl_otg.h"

void otg_reset_controller(void);

extern int usbotg_get_id_pin(void);
extern int usbotg_low_power_enter(void);
extern int usbotg_low_power_exit(void);
extern void usbotg_uninit_port(struct fsl_usb2_platform_data *pdata);

#define CONFIG_USB_OTG_DEBUG_FILES
#define DRIVER_VERSION "$Revision: 1.55 $"
#define DRIVER_AUTHOR "Jerry Huang/Li Yang"
#define DRIVER_DESC "Freescale USB OTG Driver"
#define DRIVER_INFO DRIVER_VERSION " " DRIVER_DESC


MODULE_DESCRIPTION("Freescale USB OTG Transceiver Driver");

static const char driver_name[] = "fsl-usb2-otg";

const pm_message_t otg_suspend_state = {
	.event = 1,
};

#define HA_DATA_PULSE 1

volatile static struct usb_dr_mmap *usb_dr_regs;
static struct fsl_otg *fsl_otg_dev;
/* FSM timers */
struct fsl_otg_timer *a_wait_vrise_tmr, *a_wait_bcon_tmr, *a_aidl_bdis_tmr,
	*b_ase0_brst_tmr, *b_se0_srp_tmr;

/* Driver specific timers */
struct fsl_otg_timer *b_data_pulse_tmr, *b_vbus_pulse_tmr, *b_srp_fail_tmr,
	*b_srp_wait_tmr, *a_wait_enum_tmr;

static struct fsl_otg_config fsl_otg_initdata = {
	.otg_port = 1,
};

int write_ulpi(u8 addr, u8 data)
{
	u32 temp;
	temp = 0x60000000 | (addr << 16) | data;
	temp = cpu_to_le32(temp);
	usb_dr_regs->ulpiview = temp;
	return 0;
}

/*
 * The ID pin is detected via the mc13892 power management
 * circuit. ID pin detectedion via OTGSC_STS_USB_ID in OTGSC
 * does not seem to work reliably and always reads a '1'
 * when attached to a Mini-A USB OTG cable AND VBus powered on.
 */
static int fsl_otg_read_id_pin(void)
{
	unsigned int reg_sense_0;

	if (pmic_read_reg(REG_INT_SENSE0, &reg_sense_0, (1 << SENSE_IDGNDS) | (1 << SENSE_IDFLOATS))) {
		printk(KERN_ERR "Could not read ID pin. Defaulting to %d.\n", ID_PIN_DEFAULT);

		return ID_PIN_DEFAULT;
	}


	if (!(reg_sense_0 & (1 << SENSE_IDGNDS))) {
		return 0;
	} else if (reg_sense_0 & (1 << SENSE_IDFLOATS)) {
		return 1;
	} else {
		printk(KERN_WARNING "%s: The state of the ID pin is invalid. Defaulting to %d.\n",
				__func__, ID_PIN_DEFAULT);

		return ID_PIN_DEFAULT;
	}
}

/*
 * ID pin detection should work just fine, but for custom
 * cables, provide an option to override the detection results.
 */
static inline int fsl_otg_get_id_pin(struct fsl_otg *otg_dev)
{
	if (otg_dev->id_pin_override >= 0)
		return otg_dev->id_pin_override;
	else
		return fsl_otg_read_id_pin();
}

static void do_statemachine_work(struct work_struct *work)
{
       struct fsl_otg *otg_dev = container_of(work, struct fsl_otg, statemachine_work.work);
       printk(KERN_ERR "I want to sleep\n");
       //msleep(500);
       otg_statemachine(&otg_dev->fsm);
}

/* -------------------------------------------------------------*/
/* Operations that will be called from OTG Finite State Machine */

/* A-device driver vbus, controlled through PP bit in PORTSC */
void fsl_otg_drv_vbus(struct fsl_usb2_platform_data *pdata, int on)
{
	if (pdata->xcvr_ops && pdata->xcvr_ops->set_vbus_power)
		pdata->xcvr_ops->set_vbus_power(pdata->xcvr_ops, pdata, on);
}

int fsl_otg_enable_port (struct otg_fsm *fsm, int on) {
	struct fsl_otg *otg_dev = container_of(fsm, struct fsl_otg, fsm);
	static int port_enabled = 1;
	int ret;

	if (on && !port_enabled) {
		ret = usbotg_low_power_exit();
		if (ret)
			return ret;

		port_enabled = 1;

		fsm->id = fsl_otg_get_id_pin(otg_dev);
		VDBG("Enabled port. ID pin is %d\n", fsm->id);

		return 0;
	} else if (!on && port_enabled) {
		ret = usbotg_low_power_enter();
		if (ret)
			return ret;

		port_enabled = 0;
		return 0;
	} else {
		return 0;
	}

}

/* ------------------------------------------------------*/

/* The timeout callback function to set time out bit */
void set_tmout(unsigned long indicator)
{
	*(int *)indicator = 1;
}


/* Reset controller, not reset the bus */
void otg_reset_controller(void)
{
	u32 command_reg;

	command_reg = readl(&usb_dr_regs->usbcmd);
	command_reg |= USB_CMD_CTRL_RESET;
	writel(command_reg, &usb_dr_regs->usbcmd);

	while (readl(&usb_dr_regs->usbcmd) & USB_CMD_CTRL_RESET)
		msleep(5);
}

/* Call suspend/resume routines in host driver */
int fsl_otg_start_host(struct otg_fsm *fsm, int on)
{
	struct otg_transceiver *xceiv = fsm->transceiver;
	struct device *dev;
	struct fsl_otg *otg_dev = container_of(xceiv, struct fsl_otg, otg);
	struct platform_driver *host_pdrv;
	struct platform_device *host_pdev;
	int retval = 0;

	if (!xceiv->host)
		return -ENODEV;
	dev = xceiv->host->controller;
	host_pdrv = container_of((dev->driver), struct platform_driver, driver);
	host_pdev = to_platform_device(dev);

	DBG("%s: entry. on=%d", __func__, on);

	if (on) {
		/* start fsl usb host controller */
		VDBG("Starting OTG host\n");
		otg_reset_controller();

		if (host_pdrv->resume) {
			otg_dev->otg.host->is_b_host = fsm->id;

			retval = host_pdrv->resume(host_pdev);
			if (!retval && fsm->id) {
				/* default-b */
				fsl_otg_drv_vbus(dev->platform_data, 1);
				/* Workaround: b_host can't driver
				 * vbus, but PP in PORTSC needs to
				 * be 1 for host to work.
				 * So we set drv_vbus bit in
				 * transceiver to 0 thru ULPI. */
#if defined(CONFIG_ISP1504_MXC)
				write_ulpi(0x0c, 0x20);
#endif
			}
		} else {
			printk(KERN_ERR "%s: Host controller has no resume method.\n", __func__);
			retval = -EINVAL;
		}
	} else {
		/* stop fsl usb host controller */
		VDBG("host off......\n");
		if (host_pdrv->suspend) {
			retval = host_pdrv->suspend(host_pdev,
					otg_suspend_state);
			if (!retval && fsm->id)
				/* default-b */
				fsl_otg_drv_vbus(dev->platform_data, 0);
		} else {
			printk(KERN_ERR "%s: Host controller has no suspend method.\n", __func__);
			retval = -EINVAL;
		}
	}

	return retval;
}

/* Call suspend and resume function in udc driver
 * to stop and start udc driver.
 */
int fsl_otg_start_gadget(struct otg_fsm *fsm, int on)
{
	struct otg_transceiver *xceiv = fsm->transceiver;
	struct fsl_otg *otg_dev = container_of(xceiv, struct fsl_otg, otg);
	struct device *dev;
	struct platform_driver *gadget_pdrv;
	struct platform_device *gadget_pdev;

	if (!xceiv->gadget || !xceiv->gadget->dev.parent)
		return -ENODEV;

	VDBG("gadget %s \n", on ? "on" : "off");
	dev = xceiv->gadget->dev.parent;

	gadget_pdrv = container_of((dev->driver),
			struct platform_driver, driver);
	gadget_pdev = to_platform_device(dev);

	otg_dev->otg.gadget->is_a_peripheral = !fsm->id;

	if (on)
		gadget_pdrv->resume(gadget_pdev);
	else
		gadget_pdrv->suspend(gadget_pdev, otg_suspend_state);

	VDBG("Gadget started/stopped");

	return 0;
}

/* Called by initialization code of host driver.  Register host controller
 * to the OTG.  Suspend host for OTG role detection.
 */
static int fsl_otg_set_host(struct otg_transceiver *otg_p, struct usb_bus *host)
{
	struct fsl_otg *otg_dev = container_of(otg_p, struct fsl_otg, otg);

	if (!otg_p || otg_dev != fsl_otg_dev)
		return -ENODEV;

	if (host)
		otg_p->host = host;

	otg_dev->fsm.host_available = host ? 1 : 0;

	if (host) {
		VDBG("Host driver becoming available.\n");

		otg_p->host->otg_port = fsl_otg_initdata.otg_port;
		otg_p->host->is_b_host = otg_dev->fsm.id;
	}

	otg_statemachine(&otg_dev->fsm);

	return 0;
}

/* Called by initialization code of udc.  Register udc to OTG.*/
static int fsl_otg_set_peripheral(struct otg_transceiver *otg_p,
				  struct usb_gadget *gadget)
{
	struct fsl_otg *otg_dev = container_of(otg_p, struct fsl_otg, otg);

	VDBG("otg_dev 0x%x\n", (int)otg_dev);
	VDBG("fsl_otg_dev 0x%x\n", (int)fsl_otg_dev);

	if (!otg_p || otg_dev != fsl_otg_dev)
		return -ENODEV;

	if (gadget)
		otg_p->gadget = gadget;

	otg_dev->fsm.gadget_available = gadget ? 1 : 0;

	if (gadget) {
		otg_dev->fsm.vbus_vld = is_charger_connected();
		otg_p->gadget->is_a_peripheral = !otg_dev->fsm.id;
	}

	otg_statemachine(&otg_dev->fsm);

	return 0;
}

static void callback_connect_event(void *param) {
	struct fsl_otg *otg_dev = (struct fsl_otg *) param;
	struct otg_fsm *fsm = &otg_dev->fsm;

	fsm->vbus_vld = is_charger_connected();
	DBG("Charger connect event. charger connected=%d\n", fsm->vbus_vld);

	if (fsm->vbus_vld)
		otg_statemachine(fsm);
	else
		/* need to leave some time for the USB host to finish
		 * disconnect interrupt processing.
		 */
		schedule_delayed_work(&otg_dev->statemachine_work, msecs_to_jiffies(500));
}

/* Set OTG port power, only for B-device */
static int fsl_otg_set_power(struct otg_transceiver *otg_p, unsigned mA)
{
	if (!fsl_otg_dev)
		return -ENODEV;

	printk(KERN_INFO "FSL OTG:Draw %d mA\n", mA);
	charger_set_current_limit(mA);

	return 0;
}

static struct otg_fsm_ops fsl_otg_ops = {
	.enable_port = fsl_otg_enable_port,

	.start_host = fsl_otg_start_host,
	.start_gadget = fsl_otg_start_gadget,
};

/* Initialize the global variable fsl_otg_dev and request IRQ for OTG */
static int fsl_otg_conf(struct platform_device *pdev)
{
	struct fsl_otg *fsl_otg_tc;

	DBG();

	if (fsl_otg_dev)
		return 0;

	/* allocate space to fsl otg device */
	fsl_otg_tc = kzalloc(sizeof(struct fsl_otg), GFP_KERNEL);
	if (!fsl_otg_tc)
		return -ENODEV;

	mutex_init(&fsl_otg_tc->fsm.state_mutex);
	INIT_DELAYED_WORK(&fsl_otg_tc->statemachine_work, do_statemachine_work);

	/* Set OTG state machine operations */
	fsl_otg_tc->fsm.ops = &fsl_otg_ops;

	/* initialize the otg structure */
	fsl_otg_tc->otg.label = DRIVER_DESC;
	fsl_otg_tc->otg.set_host = fsl_otg_set_host;
	fsl_otg_tc->otg.set_peripheral = fsl_otg_set_peripheral;
	fsl_otg_tc->otg.set_power = fsl_otg_set_power;
	fsl_otg_tc->otg.start_hnp = NULL;
	fsl_otg_tc->otg.start_srp = NULL;

	fsl_otg_dev = fsl_otg_tc;

	platform_set_drvdata(pdev, fsl_otg_tc);

	return 0;
}

extern int otg_set_resources(struct resource *resources);

/* OTG Initialization*/
int usb_otg_start(struct platform_device *pdev)
{
	struct fsl_otg *p_otg = platform_get_drvdata(pdev);
	struct otg_fsm *fsm;
	struct resource *res;
	int status;
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	fsm = &p_otg->fsm;

	/* We don't require predefined MEM/IRQ resource index */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	/* We don't request_mem_region here to enable resource sharing
	 * with host/device */

	usb_dr_regs = ioremap(res->start, sizeof(struct usb_dr_mmap));
	p_otg->dr_mem_map = (struct usb_dr_mmap *)usb_dr_regs;
	pdata->regs = (void *)usb_dr_regs;

	/* TODO: request callbacks from pm */


	if (pdata->platform_init && pdata->platform_init(pdev) != 0)
		return -EINVAL;

	otg_set_resources(pdev->resource);

	/* disable the port for now as there is no host/gadget driver available yet */
	p_otg->id_pin_override = -1;
	fsl_otg_enable_port(&p_otg->fsm, 0);

	/* Initialize the state machine */
	SET_OTG_STATE(&p_otg->otg, OTG_STATE_IDLE);
	fsm->transceiver = &p_otg->otg;

	/* Store the otg transceiver */
	status = otg_set_transceiver(&p_otg->otg);
	if (status) {
		printk(KERN_WARNING ": unable to register OTG transceiver.\n");
		return status;
	}

	charger_event_subscribe(CHARGER_CONNECT_EVENT, &callback_connect_event,
				p_otg);
	fsm->vbus_vld = is_charger_connected();

	return 0;
}

/*-------------------------------------------------------------------------
		PROC File System Support
-------------------------------------------------------------------------*/
#ifdef CONFIG_USB_OTG_DEBUG_FILES

#include <linux/seq_file.h>

static const char proc_filename[] = "driver/isp1504_otg";

static int otg_proc_read(char *page, char **start, off_t off, int count,
			 int *eof, void *_dev)
{
	struct otg_fsm *fsm = &fsl_otg_dev->fsm;
	char *buf = page;
	char *next = buf;
	unsigned size = count;
	int t;
	u32 tmp_reg;

	if (off != 0)
		return 0;

	mutex_lock(&fsm->state_mutex);

	/* ------basic driver infomation ---- */
	t = scnprintf(next, size,
		      DRIVER_DESC "\n" "fsl_usb2_otg version: %s\n\n",
		      DRIVER_VERSION);
	size -= t;
	next += t;

	/* ------ State ----- */
	t = scnprintf(next, size,
			"OTG state: %s\n\n",
			state_string(fsl_otg_dev->otg.state));
	size -= t;
	next += t;

	if (fsl_otg_dev->otg.state != OTG_STATE_IDLE) {

		/* ------ Registers ----- */
		tmp_reg = le32_to_cpu(usb_dr_regs->otgsc);
		t = scnprintf(next, size, "OTGSC reg: %08x\n", tmp_reg);
		size -= t;
		next += t;

		tmp_reg = le32_to_cpu(usb_dr_regs->portsc);
		t = scnprintf(next, size, "PORTSC reg: %08x\n", tmp_reg);
		size -= t;
		next += t;

		tmp_reg = le32_to_cpu(usb_dr_regs->usbmode);
		t = scnprintf(next, size, "USBMODE reg: %08x\n", tmp_reg);
		size -= t;
		next += t;

		tmp_reg = le32_to_cpu(usb_dr_regs->usbcmd);
		t = scnprintf(next, size, "USBCMD reg: %08x\n", tmp_reg);
		size -= t;
		next += t;

		tmp_reg = le32_to_cpu(usb_dr_regs->usbsts);
		t = scnprintf(next, size, "USBSTS reg: %08x\n", tmp_reg);
		size -= t;
		next += t;
	}

#if 1 || defined DEBUG
	/* ------ State Machine Variables ----- */
	t = scnprintf(next, size, "vbus_vld: %d\n", fsm->vbus_vld);
	size -= t;
	next += t;

	t = scnprintf(next, size, "id: %d\n", fsm->id);
	size -= t;
	next += t;
#endif

	mutex_unlock(&fsm->state_mutex);

	*eof = 1;
	return count - size;
}

#define create_proc_file()	create_proc_read_entry(proc_filename, \
				0, NULL, otg_proc_read, NULL)

#define remove_proc_file()	remove_proc_entry(proc_filename, NULL)

#else				/* !CONFIG_USB_OTG_DEBUG_FILES */

#define create_proc_file()	do {} while (0)
#define remove_proc_file()	do {} while (0)

#endif				/*CONFIG_USB_OTG_DEBUG_FILES */

/*----------------------------------------------------------*/
/* Char driver interface to control some OTG input */

/* This function handle some ioctl command,such as get otg
 * status and set host suspend
 */
static int fsl_otg_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	u32 retval = 0;

	switch (cmd) {
	case GET_OTG_STATUS: /* deprecated, legacy only */
		retval = (fsl_otg_dev->otg.state == OTG_STATE_HOST);
		break;

	default:
		break;
	}

	otg_statemachine(&fsl_otg_dev->fsm);

	return retval;
}

static int fsl_otg_open(struct inode *inode, struct file *file)
{

	return 0;
}

static int fsl_otg_release(struct inode *inode, struct file *file)
{

	return 0;
}

static struct file_operations otg_fops = {
	.owner = THIS_MODULE,
	.llseek = NULL,
	.read = NULL,
	.write = NULL,
	.ioctl = fsl_otg_ioctl,
	.open = fsl_otg_open,
	.release = fsl_otg_release,
};

static ssize_t
id_pin_override_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fsl_otg_dev->id_pin_override);
}

static ssize_t
id_pin_override_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	long id_pin_override_long;

	id_pin_override_long = simple_strtol(buf, NULL, 10);
	if (id_pin_override_long < -1 || id_pin_override_long > 1)
		return -EINVAL;

	fsl_otg_dev->id_pin_override = (int) id_pin_override_long;

	return size;
}

static DEVICE_ATTR(id_pin_override, 0644, id_pin_override_show, id_pin_override_store);

static ssize_t
charge_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", charger_get_current_limit());
}

static ssize_t
charge_current_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long charge_current;
	int retval;

	charge_current = simple_strtoul(buf, NULL, 10);
	if (charge_current > ((1 << 31) - 1))
		return -EINVAL;

	retval = charger_set_current_limit(charge_current);
	if (retval)
		return retval;

	return size;
}

static DEVICE_ATTR(charge_current, 0644, charge_current_show, charge_current_store);

static int __init fsl_otg_probe(struct platform_device *pdev)
{
	int status;

	DBG("pdev=0x%p\n", pdev);

	if (!pdev)
		return -ENODEV;

	/* configure the OTG */
	status = fsl_otg_conf(pdev);
	if (status) {
		printk(KERN_INFO "Couldn't init OTG module\n");
		return -status;
	}

	/* start OTG */
	status = usb_otg_start(pdev);

	if (register_chrdev(FSL_OTG_MAJOR, FSL_OTG_NAME, &otg_fops)) {
		printk(KERN_WARNING FSL_OTG_NAME
		       ": unable to register FSL OTG device\n");
		return -EIO;
	}

	if (device_create_file(&pdev->dev, &dev_attr_id_pin_override) < 0)
		dev_err(&pdev->dev, "Could not create id_pin_override device attribute.\n");

	if (device_create_file(&pdev->dev, &dev_attr_charge_current) < 0)
		dev_err(&pdev->dev, "Could not create charge_current device attribute.\n");

	create_proc_file();
	return status;
}

static int fsl_otg_remove(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;
	int retval;

	device_remove_file(&pdev->dev, &dev_attr_id_pin_override);
	device_remove_file(&pdev->dev, &dev_attr_charge_current);

	if ((retval = charger_event_unsubscribe(CHARGER_CONNECT_EVENT, &callback_connect_event)))
			return retval;

	otg_set_transceiver(NULL);
	iounmap((void *)usb_dr_regs);

	kfree(fsl_otg_dev);

	remove_proc_file();

	unregister_chrdev(FSL_OTG_MAJOR, FSL_OTG_NAME);

	if ((retval = fsl_otg_enable_port(&fsl_otg_dev->fsm, 0)))
		return retval;

	usbotg_uninit_port(pdata);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

struct platform_driver fsl_otg_driver = {
	.probe = fsl_otg_probe,
	.remove = fsl_otg_remove,
	.driver = {
		.name = driver_name,
		.owner = THIS_MODULE,
	},
};

/*-------------------------------------------------------------------------*/

static int __init fsl_usb_otg_init(void)
{
	printk(KERN_INFO DRIVER_DESC " loaded, %s\n", DRIVER_VERSION);
	return platform_driver_register(&fsl_otg_driver);
}

static void __exit fsl_usb_otg_exit(void)
{
	platform_driver_unregister(&fsl_otg_driver);
	printk(KERN_INFO DRIVER_DESC " unloaded\n");
}

subsys_initcall(fsl_usb_otg_init);
module_exit(fsl_usb_otg_exit);

MODULE_DESCRIPTION(DRIVER_INFO);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
