/*
 * Copyright (C) 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2009-2011 Amazon Technologies Inc. All Rights Reserved.
 * Manish Lachwani (lachwani@lab126.com).
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#undef DEBUG
#undef VERBOSE

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/dmapool.h>
#include <linux/clk.h>
#include <linux/pmic_external.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>
#include <mach/clock.h>

#include "arcotg_udc.h"
#include <mach/arc_otg.h>
#include <mach/hardware.h>
#include <mach/boardid.h>
#include <mach/charger.h>
#include <linux/pmic_adc.h>
#include <linux/pmic_light.h>

#define	DRIVER_DESC	"ARC USBOTG Device Controller driver"
#define	DRIVER_AUTHOR	"Manish Lachwani"
#define	DRIVER_VERSION	"10/08/2009"

#ifdef CONFIG_PPC_MPC512x
#define BIG_ENDIAN_DESC
#endif

#ifdef BIG_ENDIAN_DESC
#define cpu_to_hc32(x)	(x)
#define hc32_to_cpu(x)	(x)
#else
#define cpu_to_hc32(x)	cpu_to_le32((x))
#define hc32_to_cpu(x)	le32_to_cpu((x))
#endif

#define	DMA_ADDR_INVALID	(~(dma_addr_t)0)

#define CHARGING_THIRD_PARTY   500     /* Third party chargers */
#define CHARGING_HOST          100

static const char driver_name[] = "fsl-usb2-udc";
static const char driver_desc[] = DRIVER_DESC;

volatile static struct usb_dr_device *dr_regs;
volatile static struct usb_sys_interface *usb_sys_regs;

/* it is initialized in probe()  */
static struct fsl_udc *udc_controller;

extern int usbotg_low_power_enter(void);
extern int usbotg_low_power_exit(void);
extern void usbotg_uninit_port(struct fsl_usb2_platform_data *pdata);

extern void accessory_charger_disable(void);
extern void accessory_charger_enable(void);
extern int gpio_accessory_charger_detected(void);

/* SPI Suspended? */
extern int mxc_spi_suspended;

extern void mxc_kernel_uptime(void);

atomic_t charger_atomic_detection = ATOMIC_INIT(0);

static const struct usb_endpoint_descriptor
fsl_ep0_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	0,
	.bmAttributes =		USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize =	USB_MAX_CTRL_PAYLOAD,
};

static const struct usb_endpoint_descriptor
fsl_ep1_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = 	USB_DT_ENDPOINT,
	.bEndpointAddress =	1,
	.bmAttributes = 	USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = 	512,
};

static const size_t g_iram_size = IRAM_TD_PPH_SIZE;

static void resume_ctrl(void);
static int udc_suspend(struct fsl_udc *udc);
static void do_charge_current_work(struct work_struct *work);
static int low_power_enter(struct fsl_udc *udc);
static int low_power_exit(struct fsl_udc *udc);
static int fsl_udc_suspend(struct platform_device *dev, pm_message_t state);
static int fsl_udc_resume(struct platform_device *dev);
static void fsl_ep_fifo_flush(struct usb_ep *_ep);
static int fsl_set_current_limit(struct fsl_udc *udc, unsigned int mA);
static int udc_detection = 0;
static int udc_stopped = 0;

static int current_limit;
static int charging_wall_mA = 0;

/* port change work */
static void port_change_work(struct work_struct *work);
DECLARE_DELAYED_WORK(pc_wq, port_change_work);

static void resume_ctrl(void);
static void suspend_ctrl(void);
static void fsl_udc_redo_enumerate(void);

#ifdef CONFIG_USB_EHCI_ARC_OTG
/* Get platform resource from OTG driver */
extern struct resource *otg_get_resources(void);
#endif

#if defined(CONFIG_PPC32)
#define fsl_readl(addr)		in_le32((addr))
#define fsl_writel(addr, val32) out_le32((val32), (addr))
#else
#define fsl_readl(addr)		readl((addr))
#define fsl_writel(addr, val32) writel((addr), (val32))
#endif

static void fsl_detect_charger_type(void);
static int not_detected = 0;
static int host_detected = 0; 		/* Has a host been detected? */

static void phy_low_power_resume(struct work_struct *work);
DECLARE_WORK(charge_current_work, do_charge_current_work);
static DECLARE_DELAYED_WORK(phy_resume_work, phy_low_power_resume);

static int udc_suspended_mode = 0; /* gadget driver suspended state */

#define VBUSVALIDS		0x08	/* Sense bit for valid VBUS */
#define PC_WORK_THRESHOLD	30000	/* Port change threshold */
#define ARCOTG_IRQ		37
#define CHARGER_STARTUP 	4000	/* Delayed workqueue that runs on startup */
#define CHARGER_TYPE		1000	/* Charger type detection after 1 second */
#define CHARGER_TYPE_SUSPEND	5000	/* Charger detection after a system resume */
#define CHARGER_TYPE_LPM	4000	/* Transition PHY into LPM after 10 seconds */
#define PHY_LPM_TIMER		20000	/* 20 seconds after resume */
#define THIRD_PARTY_DETECT	30000	/* Third Party charger detect */
#define HOST_ENU_DETECT 	100	/* Host enumeration check */
#define CHRG_SUS_LOW_VOLTAGE	3420	/* 3.42V */

#define PMIC_IRQ			108	/* PMIC IRQ */

static atomic_t enumerated = ATOMIC_INIT(0);
static int charger_enumerated = 0;
static int charger_conn_probe = 0;	/* Is charger connected on probe? */

static ssize_t
show_connected(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(connected, 0444, show_connected, NULL);

static ssize_t
show_charging(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(charging, 0444, show_charging, NULL);

static ssize_t
show_lobat_condition(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(lobat_condition, 0444, show_lobat_condition, NULL);

static ssize_t
show_bpsns(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(bpsns, 0444, show_bpsns, NULL);

static ssize_t
show_supply_voltage(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(supply_voltage, 0444, show_supply_voltage, NULL);

static ssize_t
show_chrgraw_voltage(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(chrgraw_voltage, 0444, show_chrgraw_voltage, NULL);

static ssize_t
show_ichrg_setting(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(ichrg_setting, 0444, show_ichrg_setting, NULL);

static ssize_t
show_voltage(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(voltage, 0444, show_voltage, NULL);

static ssize_t
show_battery_current(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(battery_current, 0444, show_battery_current, NULL);

static ssize_t
show_batt_error(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(batt_error, 0444, show_batt_error, NULL);

static ssize_t
show_battery_id(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(battery_id, 0444, show_battery_id, NULL);

static ssize_t
show_third_party(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(third_party, 0444, show_third_party, NULL);

static ssize_t
show_accessory_charger(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(accessory_charger, 0444, show_accessory_charger, NULL);

/********************************************************************
 *	Internal Used Function
********************************************************************/

static void do_charger_detect_work(struct work_struct *work);

#ifdef DUMP_QUEUES
static void dump_ep_queue(struct fsl_ep *ep)
{
	int ep_index;
	struct fsl_req *req;
	struct ep_td_struct *dtd;

	if (list_empty(&ep->queue)) {
		pr_debug("udc: empty\n");
		return;
	}

	ep_index = ep_index(ep) * 2 + ep_is_in(ep);
	pr_debug("udc: ep=0x%p  index=%d\n", ep, ep_index);

	list_for_each_entry(req, &ep->queue, queue) {
		pr_debug("udc: req=0x%p  dTD count=%d\n", req, req->dtd_count);
		pr_debug("udc: dTD head=0x%p  tail=0x%p\n", req->head,
			 req->tail);

		dtd = req->head;

		while (dtd) {
			if (le32_to_cpu(dtd->next_td_ptr) & DTD_NEXT_TERMINATE)
				break;	/* end of dTD list */

			dtd = dtd->next_td_virt;
		}
	}
}
#else
static inline void dump_ep_queue(struct fsl_ep *ep)
{
}
#endif

static void wall_charger_start_charging(void)
{
	printk(KERN_INFO "arcotg_udc: I pmic:chargerWall\n");

	fsl_set_current_limit(udc_controller, charging_wall_mA);
	
	kobject_uevent(&udc_controller->gadget.dev.parent->kobj, KOBJ_ADD);
	if (device_create_file(udc_controller->gadget.dev.parent,
			       &dev_attr_connected) < 0) {
		pr_debug("Charger: Error creating device file\n");
	}
}

static int is_wall_charger_attached(void)
{
	unsigned int portsc1;

	resume_ctrl();
	mdelay(10);

	portsc1 = fsl_readl(&dr_regs->portsc1);

	return (portsc1 & PORTSCX_LINE_STATUS_BITS) == PORTSCX_LINE_STATUS_UNDEF;
}

/*
 * Kicked off only on startup to figure out whether a charger/host is
 * connected or to enter low power idle mode.
 */
static void do_charger_detect_work(struct work_struct *work)
{
	struct fsl_udc *udc = 
		container_of(work, struct fsl_udc, charger_detect_work.work);
	int charger_conn = is_charger_connected();

	pr_debug("%s: begin\n", __func__);

	if (charger_conn) {
		if (!not_detected) {
			pr_debug("%s: not detected prb=%d\n", __func__, charger_conn_probe);

			if (charger_conn_probe == 1) {
				charger_conn_probe = 0;
				if (is_wall_charger_attached()) {
					wall_charger_start_charging();
				} else {
					fsl_detect_charger_type();
				}
			} else {
				fsl_detect_charger_type();
			}
		}
	} else {
		printk(KERN_INFO "No USB/Charger found.\n");
	}
}

/*!
 * Post a low battery or a critical battery event to the userspace 
 */
static void callback_lobat_event(void *param)
{
	if (!charger_have_low_battery())
		return;

	if (charger_have_critical_battery()) {
		char *envp[] = { "BATTERY=critical", NULL };
		printk(KERN_ERR "arcotg_udc: I pmic:critbatEvent::\n");

		/* Relay the lobatl event to userspace */
		kobject_uevent_env(&udc_controller->gadget.dev.parent->kobj, KOBJ_CHANGE, envp);
	} else {
		char *envp[] = { "BATTERY=low", NULL };
		printk(KERN_INFO "arcotg_udc: I pmic:lowbatEvent::\n");

		/* Relay the lobath event to userspace */
		kobject_uevent_env(&udc_controller->gadget.dev.parent->kobj, KOBJ_CHANGE, envp);
	}
}

/*-----------------------------------------------------------------
 * done() - retire a request; caller blocked irqs
 * @status : request status to be set, only works when
 *	request is still in progress.
 *--------------------------------------------------------------*/
static void done(struct fsl_ep *ep, struct fsl_req *req, int status)
{
	struct fsl_udc *udc = NULL;
	unsigned char stopped = ep->stopped;
	struct ep_td_struct *curr_td, *next_td;
	int j;

	udc = (struct fsl_udc *)ep->udc;
	/* Removed the req from fsl_ep->queue */
	list_del_init(&req->queue);

	/* req.status should be set as -EINPROGRESS in ep_queue() */
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

	/* Free dtd for the request */
	next_td = req->head;
	for (j = 0; j < req->dtd_count; j++) {
		curr_td = next_td;
		if (j != req->dtd_count - 1) {
			next_td = curr_td->next_td_virt;
		}

		dma_pool_free(udc->td_pool, curr_td, curr_td->td_dma);
	}

	if (USE_MSC_WR(req->req.length)) {
		req->req.dma -= 1;
		memmove(req->req.buf, req->req.buf + 1, MSC_BULK_CB_WRAP_LEN);
	}

	if (req->mapped) {
		dma_unmap_single(ep->udc->gadget.dev.parent,
			req->req.dma, req->req.length,
			ep_is_in(ep)
				? DMA_TO_DEVICE
				: DMA_FROM_DEVICE);
		req->req.dma = DMA_ADDR_INVALID;
		req->mapped = 0;
	} else
		dma_sync_single_for_cpu(ep->udc->gadget.dev.parent,
			req->req.dma, req->req.length,
			ep_is_in(ep)
				? DMA_TO_DEVICE
				: DMA_FROM_DEVICE);

	if (status && (status != -ESHUTDOWN))
		VDBG("complete %s req %p stat %d len %u/%u",
			ep->ep.name, &req->req, status,
			req->req.actual, req->req.length);

	ep->stopped = 1;

	spin_unlock(&ep->udc->lock);
	/* complete() is from gadget layer,
	 * eg fsg->bulk_in_complete() */
	if (req->req.complete)
		req->req.complete(&ep->ep, &req->req);

	spin_lock(&ep->udc->lock);
	ep->stopped = stopped;
}

/*-----------------------------------------------------------------
 * nuke(): delete all requests related to this ep
 * called with spinlock held
 *--------------------------------------------------------------*/
static void nuke(struct fsl_ep *ep, int status)
{
	ep->stopped = 1;

	/* Flush fifo */
	fsl_ep_fifo_flush(&ep->ep);

	/* Whether this eq has request linked */
	while (!list_empty(&ep->queue)) {
		struct fsl_req *req = NULL;

		req = list_entry(ep->queue.next, struct fsl_req, queue);
		done(ep, req, status);
	}
	dump_ep_queue(ep);
}

/*------------------------------------------------------------------
	Internal Hardware related function
 ------------------------------------------------------------------*/

static int dr_controller_setup(struct fsl_udc *udc)
{
	unsigned int tmp = 0, portctrl = 0;
	unsigned int __attribute((unused)) ctrl = 0;
	unsigned long timeout;
	struct fsl_usb2_platform_data *pdata;

#define FSL_UDC_RESET_TIMEOUT 1000

	/* before here, make sure dr_regs has been initialized */
	if (!udc)
		return -EINVAL;
	pdata = udc->pdata;

	/* Stop and reset the usb controller */
	tmp = fsl_readl(&dr_regs->usbcmd);
	tmp &= ~USB_CMD_RUN_STOP;
	fsl_writel(tmp, &dr_regs->usbcmd);

	tmp = fsl_readl(&dr_regs->usbcmd);
	tmp |= USB_CMD_CTRL_RESET;
	fsl_writel(tmp, &dr_regs->usbcmd);

	/* Wait for reset to complete */
	timeout = jiffies + FSL_UDC_RESET_TIMEOUT;
	while (fsl_readl(&dr_regs->usbcmd) & USB_CMD_CTRL_RESET) {
		if (time_after(jiffies, timeout)) {
			ERR("udc reset timeout! \n");
			return -ETIMEDOUT;
		}
		cpu_relax();
	}

	/* Set the controller as device mode */
	tmp = fsl_readl(&dr_regs->usbmode);
	tmp &= ~USB_MODE_CTRL_MODE_MASK;	/* clear mode bits */
	tmp |= USB_MODE_CTRL_MODE_DEVICE;
	/* Disable Setup Lockout */
	tmp |= USB_MODE_SETUP_LOCK_OFF;
	if (pdata->es)
		tmp |= USB_MODE_ES;
	fsl_writel(tmp, &dr_regs->usbmode);

	fsl_platform_set_device_mode(pdata);

	/* Clear the setup status */
	fsl_writel(0, &dr_regs->usbsts);

	tmp = udc->ep_qh_dma;
	tmp &= USB_EP_LIST_ADDRESS_MASK;
	fsl_writel(tmp, &dr_regs->endpointlistaddr);

	VDBG("vir[qh_base] is %p phy[qh_base] is 0x%8x reg is 0x%8x",
		(int)udc->ep_qh, (int)tmp,
		fsl_readl(&dr_regs->endpointlistaddr));

	/* Config PHY interface */
	portctrl = fsl_readl(&dr_regs->portsc1);
	portctrl &= ~(PORTSCX_PHY_TYPE_SEL | PORTSCX_PORT_WIDTH);
	switch (udc->phy_mode) {
	case FSL_USB2_PHY_ULPI:
		portctrl |= PORTSCX_PTS_ULPI;
		break;
	case FSL_USB2_PHY_UTMI_WIDE:
		portctrl |= PORTSCX_PTW_16BIT;
		/* fall through */
	case FSL_USB2_PHY_UTMI:
		portctrl |= PORTSCX_PTS_UTMI;
		break;
	case FSL_USB2_PHY_SERIAL:
		portctrl |= PORTSCX_PTS_FSLS;
		break;
	default:
		return -EINVAL;
	}
	fsl_writel(portctrl, &dr_regs->portsc1);

	if (pdata->have_sysif_regs) {
		/* Config control enable i/o output, cpu endian register */
		ctrl = __raw_readl(&usb_sys_regs->control);
		ctrl |= USB_CTRL_IOENB;
		__raw_writel(ctrl, &usb_sys_regs->control);
	}

#if defined(CONFIG_PPC32) && !defined(CONFIG_NOT_COHERENT_CACHE)
	/* Turn on cache snooping hardware, since some PowerPC platforms
	 * wholly rely on hardware to deal with cache coherent. */

	if (pdata->have_sysif_regs) {
		/* Setup Snooping for all the 4GB space */
		tmp = SNOOP_SIZE_2GB;	/* starts from 0x0, size 2G */
		__raw_writel(tmp, &usb_sys_regs->snoop1);
		tmp |= 0x80000000;	/* starts from 0x8000000, size 2G */
		__raw_writel(tmp, &usb_sys_regs->snoop2);
	}
#endif

	return 0;
}

/* Enable DR irq and set controller to run state */
static void dr_controller_run(struct fsl_udc *udc)
{
	u32 temp;

	fsl_platform_pullup_enable(udc->pdata);

	/* Enable DR irq reg */
	temp = USB_INTR_INT_EN | USB_INTR_ERR_INT_EN
		| USB_INTR_PTC_DETECT_EN | USB_INTR_RESET_EN
		| USB_INTR_DEVICE_SUSPEND | USB_INTR_SYS_ERR_EN;

	fsl_writel(temp, &dr_regs->usbintr);

	/* Clear stopped bit */
	udc->stopped = 0;

	/* Set the controller as device mode */
	temp = fsl_readl(&dr_regs->usbmode);
	temp |= USB_MODE_CTRL_MODE_DEVICE;
	fsl_writel(temp, &dr_regs->usbmode);

	/* Set controller to Run */
	temp = fsl_readl(&dr_regs->usbcmd);
	temp |= USB_CMD_RUN_STOP;
	fsl_writel(temp, &dr_regs->usbcmd);

	return;
}

static void dr_controller_stop(struct fsl_udc *udc)
{
	unsigned int tmp;

	DBG("%s\n", __func__);

	udc_suspend(udc);

	/* disable IO output */
/*	usb_sys_regs->control = 0; */

	fsl_platform_pullup_disable(udc->pdata);

	/* set controller to Stop */
	tmp = fsl_readl(&dr_regs->usbcmd);
	tmp &= ~USB_CMD_RUN_STOP;
	fsl_writel(tmp, &dr_regs->usbcmd);

	return;
}

void dr_ep_setup(unsigned char ep_num, unsigned char dir, unsigned char ep_type)
{
	unsigned int tmp_epctrl = 0;

	tmp_epctrl = fsl_readl(&dr_regs->endptctrl[ep_num]);
	if (dir) {
		if (ep_num)
			tmp_epctrl |= EPCTRL_TX_DATA_TOGGLE_RST;
		tmp_epctrl |= EPCTRL_TX_ENABLE;
		tmp_epctrl |= ((unsigned int)(ep_type)
				<< EPCTRL_TX_EP_TYPE_SHIFT);
	} else {
		if (ep_num)
			tmp_epctrl |= EPCTRL_RX_DATA_TOGGLE_RST;
		tmp_epctrl |= EPCTRL_RX_ENABLE;
		tmp_epctrl |= ((unsigned int)(ep_type)
				<< EPCTRL_RX_EP_TYPE_SHIFT);
	}

	fsl_writel(tmp_epctrl, &dr_regs->endptctrl[ep_num]);
}

static void
dr_ep_change_stall(unsigned char ep_num, unsigned char dir, int value)
{
	u32 tmp_epctrl = 0;

	tmp_epctrl = fsl_readl(&dr_regs->endptctrl[ep_num]);

	if (value) {
		/* set the stall bit */
		if (dir)
			tmp_epctrl |= EPCTRL_TX_EP_STALL;
		else
			tmp_epctrl |= EPCTRL_RX_EP_STALL;
	} else {
		/* clear the stall bit and reset data toggle */
		if (dir) {
			tmp_epctrl &= ~EPCTRL_TX_EP_STALL;
			tmp_epctrl |= EPCTRL_TX_DATA_TOGGLE_RST;
		} else {
			tmp_epctrl &= ~EPCTRL_RX_EP_STALL;
			tmp_epctrl |= EPCTRL_RX_DATA_TOGGLE_RST;
		}
	}
	fsl_writel(tmp_epctrl, &dr_regs->endptctrl[ep_num]);
}

/* Get stall status of a specific ep
   Return: 0: not stalled; 1:stalled */
static int dr_ep_get_stall(unsigned char ep_num, unsigned char dir)
{
	u32 epctrl;

	epctrl = fsl_readl(&dr_regs->endptctrl[ep_num]);
	if (dir)
		return (epctrl & EPCTRL_TX_EP_STALL) ? 1 : 0;
	else
		return (epctrl & EPCTRL_RX_EP_STALL) ? 1 : 0;
}

/********************************************************************
	Internal Structure Build up functions
********************************************************************/

/*------------------------------------------------------------------
* struct_ep_qh_setup(): set the Endpoint Capabilites field of QH
 * @zlt: Zero Length Termination Select (1: disable; 0: enable)
 * @mult: Mult field
 ------------------------------------------------------------------*/
static void struct_ep_qh_setup(struct fsl_udc *udc, unsigned char ep_num,
		unsigned char dir, unsigned char ep_type,
		unsigned int max_pkt_len,
		unsigned int zlt, unsigned char mult)
{
	struct ep_queue_head *p_QH = &udc->ep_qh[2 * ep_num + dir];
	unsigned int tmp = 0;

	/* set the Endpoint Capabilites in QH */
	switch (ep_type) {
	case USB_ENDPOINT_XFER_CONTROL:
		/* Interrupt On Setup (IOS). for control ep  */
		tmp = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS)
			| EP_QUEUE_HEAD_IOS;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		tmp = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS)
			| (mult << EP_QUEUE_HEAD_MULT_POS);
		break;
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		tmp = max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS;
		break;
	default:
		VDBG("error ep type is %d", ep_type);
		return;
	}
	if (zlt)
		tmp |= EP_QUEUE_HEAD_ZLT_SEL;
	p_QH->max_pkt_length = cpu_to_hc32(tmp);
	p_QH->next_dtd_ptr = 1;
	p_QH->size_ioc_int_sts = 0;

	return;
}

/* Setup qh structure and ep register for ep0. */
static void ep0_setup(struct fsl_udc *udc)
{
	/* the intialization of an ep includes: fields in QH, Regs,
	 * fsl_ep struct */
	struct_ep_qh_setup(udc, 0, USB_RECV, USB_ENDPOINT_XFER_CONTROL,
			USB_MAX_CTRL_PAYLOAD, 0, 0);
	struct_ep_qh_setup(udc, 0, USB_SEND, USB_ENDPOINT_XFER_CONTROL,
			USB_MAX_CTRL_PAYLOAD, 0, 0);
	dr_ep_setup(0, USB_RECV, USB_ENDPOINT_XFER_CONTROL);
	dr_ep_setup(0, USB_SEND, USB_ENDPOINT_XFER_CONTROL);

	return;

}

/***********************************************************************
		Endpoint Management Functions
***********************************************************************/

/*-------------------------------------------------------------------------
 * when configurations are set, or when interface settings change
 * for example the do_set_interface() in gadget layer,
 * the driver will enable or disable the relevant endpoints
 * ep0 doesn't use this routine. It is always enabled.
-------------------------------------------------------------------------*/
static int fsl_ep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct fsl_udc *udc = NULL;
	struct fsl_ep *ep = NULL;
	unsigned short max = 0;
	unsigned char mult = 0, zlt;
	int retval = -EINVAL;
	unsigned long flags = 0;

	ep = container_of(_ep, struct fsl_ep, ep);

	pr_debug("udc: %s ep.name=%s %d\n", __func__, ep->ep.name, desc->bDescriptorType);
	/* catch various bogus parameters */
	if (!_ep || !desc 
			|| (desc->bDescriptorType != USB_DT_ENDPOINT))
		return -EINVAL;

	udc = ep->udc;

	if (!udc->driver || (udc->gadget.speed == USB_SPEED_UNKNOWN))
		return -ESHUTDOWN;

	max = le16_to_cpu(desc->wMaxPacketSize);

	/* Disable automatic zlp generation.  Driver is reponsible to indicate
	 * explicitly through req->req.zero.  This is needed to enable multi-td
	 * request. */
	zlt = 1;

	/* Assume the max packet size from gadget is always correct */
	switch (desc->bmAttributes & 0x03) {
	case USB_ENDPOINT_XFER_CONTROL:
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		/* mult = 0.  Execute N Transactions as demonstrated by
		 * the USB variable length packet protocol where N is
		 * computed using the Maximum Packet Length (dQH) and
		 * the Total Bytes field (dTD) */
		mult = 0;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		/* Calculate transactions needed for high bandwidth iso */
		mult = (unsigned char)(1 + ((max >> 11) & 0x03));
		max = max & 0x8ff;	/* bit 0~10 */
		/* 3 transactions at most */
		if (mult > 3)
			goto en_done;
		break;
	default:
		goto en_done;
	}

	spin_lock_irqsave(&udc->lock, flags);
	ep->ep.maxpacket = max;
	ep->desc = desc;
	ep->stopped = 0;

	/* Controller related setup */
	/* Init EPx Queue Head (Ep Capabilites field in QH
	 * according to max, zlt, mult) */
	struct_ep_qh_setup(udc, (unsigned char) ep_index(ep),
			(unsigned char) ((desc->bEndpointAddress & USB_DIR_IN)
					?  USB_SEND : USB_RECV),
			(unsigned char) (desc->bmAttributes
					& USB_ENDPOINT_XFERTYPE_MASK),
			max, zlt, mult);

	/* Init endpoint ctrl register */
	dr_ep_setup((unsigned char) ep_index(ep),
			(unsigned char) ((desc->bEndpointAddress & USB_DIR_IN)
					? USB_SEND : USB_RECV),
			(unsigned char) (desc->bmAttributes
					& USB_ENDPOINT_XFERTYPE_MASK));

	spin_unlock_irqrestore(&udc->lock, flags);
	retval = 0;

	VDBG("enabled %s (ep%d%s) maxpacket %d", ep->ep.name,
			ep->desc->bEndpointAddress & 0x0f,
			(desc->bEndpointAddress & USB_DIR_IN)
				? "in" : "out", max);
en_done:
	return retval;
}

/*---------------------------------------------------------------------
 * @ep : the ep being unconfigured. May not be ep0
 * Any pending and uncomplete req will complete with status (-ESHUTDOWN)
*---------------------------------------------------------------------*/
static int fsl_ep_disable(struct usb_ep *_ep)
{
	struct fsl_udc *udc = NULL;
	struct fsl_ep *ep = NULL;
	u32 epctrl;
	int ep_num;

	ep = container_of(_ep, struct fsl_ep, ep);
	if (!_ep || !ep->desc) {
		VDBG("%s not enabled", _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	/* disable ep on controller */
	ep_num = ep_index(ep);
	epctrl = fsl_readl(&dr_regs->endptctrl[ep_num]);
	if (ep_is_in(ep))
		epctrl &= ~EPCTRL_TX_ENABLE;
	else
		epctrl &= ~EPCTRL_RX_ENABLE;
	fsl_writel(epctrl, &dr_regs->endptctrl[ep_num]);

	udc = (struct fsl_udc *)ep->udc;

	/* nuke all pending requests (does flush) */
	nuke(ep, -ESHUTDOWN);

	ep->desc = 0;
	ep->stopped = 1;

	VDBG("disabled %s OK", _ep->name);
	return 0;
}

/*---------------------------------------------------------------------
 * allocate a request object used by this endpoint
 * the main operation is to insert the req->queue to the eq->queue
 * Returns the request, or null if one could not be allocated
*---------------------------------------------------------------------*/
static struct usb_request *
fsl_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct fsl_req *req = NULL;

	req = kzalloc(sizeof *req, gfp_flags);
	if (!req)
		return NULL;

	req->req.dma = DMA_ADDR_INVALID;
	pr_debug("udc: req=0x%p   set req.dma=0x%x\n", req, req->req.dma);
	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void fsl_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct fsl_req *req = NULL;

	req = container_of(_req, struct fsl_req, req);

	if (_req)
		kfree(req);
}

static void update_qh(struct fsl_req *req)
{
	struct fsl_ep *ep = req->ep;
	int i = ep_index(ep) * 2 + ep_is_in(ep);
	u32 temp;
	struct ep_queue_head *dQH = &ep->udc->ep_qh[i];

	/* Write dQH next pointer and terminate bit to 0 */
	temp = req->head->td_dma & EP_QUEUE_HEAD_NEXT_POINTER_MASK;
	if (NEED_IRAM(req->ep)) {
		req->cur->next_td_ptr |= cpu_to_hc32(DTD_NEXT_TERMINATE);
		temp = req->cur->td_dma & EP_QUEUE_HEAD_NEXT_POINTER_MASK;
	}
	dQH->next_dtd_ptr = cpu_to_hc32(temp);
	temp = cpu_to_hc32(~(EP_QUEUE_HEAD_STATUS_ACTIVE
				| EP_QUEUE_HEAD_STATUS_HALT));
	dQH->size_ioc_int_sts &= temp;

	temp = ep_is_in(ep)
		? (1 << (ep_index(ep) + 16))
		: (1 << (ep_index(ep)));
	fsl_writel(temp, &dr_regs->endpointprime);
}

/*-------------------------------------------------------------------------*/
static int fsl_queue_td(struct fsl_ep *ep, struct fsl_req *req)
{
	u32 temp, bitmask, tmp_stat;

	/* VDBG("QH addr Register 0x%8x", dr_regs->endpointlistaddr);
	VDBG("ep_qh[%d] addr is 0x%8x", i, (u32)&(ep->udc->ep_qh[i])); */

	bitmask = ep_is_in(ep)
		? (1 << (ep_index(ep) + 16))
		: (1 << (ep_index(ep)));

	/* check if the pipe is empty */
	if (!(list_empty(&ep->queue))) {
		/* Add td to the end */
		struct fsl_req *lastreq;
		lastreq = list_entry(ep->queue.prev, struct fsl_req, queue);
		if (NEED_IRAM(ep)) {
			lastreq->tail->next_td_ptr =
				cpu_to_hc32(req->head->td_dma | DTD_NEXT_TERMINATE);
			goto out;
		} else {
			lastreq->tail->next_td_ptr =
				cpu_to_hc32(req->head->td_dma & DTD_ADDR_MASK);
		}
		/* Read prime bit, if 1 goto done */
		if (fsl_readl(&dr_regs->endpointprime) & bitmask)
			goto out;

		do {
			/* Set ATDTW bit in USBCMD */
			temp = fsl_readl(&dr_regs->usbcmd);
			fsl_writel(temp | USB_CMD_ATDTW, &dr_regs->usbcmd);

			/* Read correct status bit */
			tmp_stat = fsl_readl(&dr_regs->endptstatus) & bitmask;

		} while (!(fsl_readl(&dr_regs->usbcmd) & USB_CMD_ATDTW));

		/* Write ATDTW bit to 0 */
		temp = fsl_readl(&dr_regs->usbcmd);
		fsl_writel(temp & ~USB_CMD_ATDTW, &dr_regs->usbcmd);

		if (tmp_stat)
			goto out;
	}
	update_qh(req);
out:
	return 0;
}

/* Fill in the dTD structure
 * @req: request that the transfer belongs to
 * @length: return actually data length of the dTD
 * @dma: return dma address of the dTD
 * @is_last: return flag if it is the last dTD of the request
 * return: pointer to the built dTD */
static struct ep_td_struct *fsl_build_dtd(struct fsl_req *req, unsigned *length,
		dma_addr_t *dma, int *is_last)
{
	u32 swap_temp;
	struct ep_td_struct *dtd;

	/* how big will this transfer be? */
	*length = min(req->req.length - req->req.actual,
			(unsigned)EP_MAX_LENGTH_TRANSFER);

	if (NEED_IRAM(req->ep))
		*length = min(*length, g_iram_size);
	dtd = dma_pool_alloc(udc_controller->td_pool, GFP_KERNEL, dma);
	if (dtd == NULL)
		return dtd;

	dtd->td_dma = *dma;
	/* Clear reserved field */
	swap_temp = hc32_to_cpu(dtd->size_ioc_sts);
	if (NEED_IRAM(req->ep))
		swap_temp = (u32) (req->req.dma);
	swap_temp &= ~DTD_RESERVED_FIELDS;
	dtd->size_ioc_sts = cpu_to_hc32(swap_temp);

	/* Init all of buffer page pointers */
	swap_temp = (u32) (req->req.dma + req->req.actual);
	dtd->buff_ptr0 = cpu_to_hc32(swap_temp);
	dtd->buff_ptr1 = cpu_to_hc32(swap_temp + 0x1000);
	dtd->buff_ptr2 = cpu_to_hc32(swap_temp + 0x2000);
	dtd->buff_ptr3 = cpu_to_hc32(swap_temp + 0x3000);
	dtd->buff_ptr4 = cpu_to_hc32(swap_temp + 0x4000);

	req->req.actual += *length;

	/* zlp is needed if req->req.zero is set */
	if (req->req.zero) {
		if (*length == 0 || (*length % req->ep->ep.maxpacket) != 0)
			*is_last = 1;
		else
			*is_last = 0;
	} else if (req->req.length == req->req.actual)
		*is_last = 1;
	else
		*is_last = 0;

	if ((*is_last) == 0)
		VDBG("multi-dtd request!\n");
	/* Fill in the transfer size; set active bit */
	swap_temp = ((*length << DTD_LENGTH_BIT_POS) | DTD_STATUS_ACTIVE);

	/* Enable interrupt for the last dtd of a request */
	if (*is_last && !req->req.no_interrupt)
		swap_temp |= DTD_IOC;
	if (NEED_IRAM(req->ep))
		swap_temp |= DTD_IOC;

	dtd->size_ioc_sts = cpu_to_hc32(swap_temp);

	mb();

	VDBG("length = %d address= 0x%x", *length, (int)*dma);

	return dtd;
}

/* Generate dtd chain for a request */
static int fsl_req_to_dtd(struct fsl_req *req)
{
	unsigned	count;
	int		is_last;
	int		is_first = 1;
	struct ep_td_struct	*last_dtd = NULL, *dtd;
	dma_addr_t dma;

	if (NEED_IRAM(req->ep)) {
		req->oridma = req->req.dma;
		if (ep_is_in(req->ep)) {
			if ((list_empty(&req->ep->queue))) {
				memcpy((char *)req->ep->udc->iram_buffer_v[1],
					req->req.buf, min(req->req.length, g_iram_size));
			}
		}
		else {
			req->req.dma = req->ep->udc->iram_buffer[0];
		}
	}

	if (USE_MSC_WR(req->req.length))
		req->req.dma += 1;

	do {
		dtd = fsl_build_dtd(req, &count, &dma, &is_last);
		if (dtd == NULL)
			return -ENOMEM;

		if (is_first) {
			is_first = 0;
			req->head = dtd;
		} else {
			last_dtd->next_td_ptr = cpu_to_hc32(dma);
			last_dtd->next_td_virt = dtd;
		}
		last_dtd = dtd;

		req->dtd_count++;
	} while (!is_last);

	dtd->next_td_ptr = cpu_to_hc32(DTD_NEXT_TERMINATE);

	req->tail = dtd;

	return 0;
}

/* queues (submits) an I/O request to an endpoint */
static int
fsl_ep_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct fsl_ep *ep = container_of(_ep, struct fsl_ep, ep);
	struct fsl_req *req = container_of(_req, struct fsl_req, req);
	struct fsl_udc *udc;
	unsigned long flags;
	int is_iso = 0;

	/* catch various bogus parameters */
	if (!_req || !req->req.complete || !req->req.buf
			|| !list_empty(&req->queue)) {
		VDBG("%s, bad params\n", __func__);
		return -EINVAL;
	}
	if (!_ep || (!ep->desc && ep_index(ep))) {
		VDBG("%s, bad ep\n", __func__);
		return -EINVAL;
	}
	if (ep->desc->bmAttributes == USB_ENDPOINT_XFER_ISOC) {
		if (req->req.length > ep->ep.maxpacket)
			return -EMSGSIZE;
		is_iso = 1;
	}

	udc = ep->udc;
	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	req->ep = ep;

	/* map virtual address to hardware */
	if (req->req.dma == DMA_ADDR_INVALID) {
		req->req.dma = dma_map_single(ep->udc->gadget.dev.parent,
					req->req.buf,
					req->req.length, ep_is_in(ep)
						? DMA_TO_DEVICE
						: DMA_FROM_DEVICE);
		req->mapped = 1;
	} else {
		dma_sync_single_for_device(ep->udc->gadget.dev.parent,
					req->req.dma, req->req.length,
					ep_is_in(ep)
						? DMA_TO_DEVICE
						: DMA_FROM_DEVICE);
		req->mapped = 0;
	}

	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->dtd_count = 0;

	if (NEED_IRAM(ep)) {
		req->last_one = 0;
		req->buffer_offset = 0;
	}
	spin_lock_irqsave(&udc->lock, flags);

	/* build dtds and push them to device queue */
	if (!fsl_req_to_dtd(req)) {
		fsl_queue_td(ep, req);
	} else {
		spin_unlock_irqrestore(&udc->lock, flags);
		return -ENOMEM;
	}

	/* Update ep0 state */
	if ((ep_index(ep) == 0))
		udc->ep0_state = DATA_STATE_XMIT;

	/* irq handler advances the queue */
	if (req != NULL)
		list_add_tail(&req->queue, &ep->queue);
	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

/* dequeues (cancels, unlinks) an I/O request from an endpoint */
static int fsl_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct fsl_ep *ep = container_of(_ep, struct fsl_ep, ep);
	struct fsl_req *req;
	unsigned long flags;
	int ep_num, stopped, ret = 0;
	u32 epctrl;

	if (!_ep || !_req)
		return -EINVAL;

	spin_lock_irqsave(&ep->udc->lock, flags);
	stopped = ep->stopped;

	/* Stop the ep before we deal with the queue */
	ep->stopped = 1;
	ep_num = ep_index(ep);
	epctrl = fsl_readl(&dr_regs->endptctrl[ep_num]);
	if (ep_is_in(ep))
		epctrl &= ~EPCTRL_TX_ENABLE;
	else
		epctrl &= ~EPCTRL_RX_ENABLE;
	fsl_writel(epctrl, &dr_regs->endptctrl[ep_num]);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		ret = -EINVAL;
		goto out;
	}

	/* The request is in progress, or completed but not dequeued */
	if (ep->queue.next == &req->queue) {
		_req->status = -ECONNRESET;
		fsl_ep_fifo_flush(_ep);	/* flush current transfer */

		/* The request isn't the last request in this ep queue */
		if (req->queue.next != &ep->queue) {
			struct ep_queue_head *qh;
			struct fsl_req *next_req;

			qh = ep->qh;
			next_req = list_entry(req->queue.next, struct fsl_req,
					queue);

			/* Point the QH to the first TD of next request */
			fsl_writel((u32) next_req->head, &qh->curr_dtd_ptr);
		}

		/* The request hasn't been processed, patch up the TD chain */
	} else {
		struct fsl_req *prev_req;

		prev_req = list_entry(req->queue.prev, struct fsl_req, queue);
		fsl_writel(fsl_readl(&req->tail->next_td_ptr),
				&prev_req->tail->next_td_ptr);

	}

	done(ep, req, -ECONNRESET);

	/* Enable EP */
out:	epctrl = fsl_readl(&dr_regs->endptctrl[ep_num]);
	if (ep_is_in(ep))
		epctrl |= EPCTRL_TX_ENABLE;
	else
		epctrl |= EPCTRL_RX_ENABLE;
	fsl_writel(epctrl, &dr_regs->endptctrl[ep_num]);
	ep->stopped = stopped;

	spin_unlock_irqrestore(&ep->udc->lock, flags);
	return ret;
}

/*-------------------------------------------------------------------------*/

/*-----------------------------------------------------------------
 * modify the endpoint halt feature
 * @ep: the non-isochronous endpoint being stalled
 * @value: 1--set halt  0--clear halt
 * Returns zero, or a negative error code.
*----------------------------------------------------------------*/
static int fsl_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct fsl_ep *ep = NULL;
	unsigned long flags = 0;
	int status = -EOPNOTSUPP;	/* operation not supported */
	unsigned char ep_dir = 0, ep_num = 0;
	struct fsl_udc *udc = NULL;

	ep = container_of(_ep, struct fsl_ep, ep);
	udc = ep->udc;
	if (!_ep || !ep->desc) {
		status = -EINVAL;
		goto out;
	}

	if (ep->desc->bmAttributes == USB_ENDPOINT_XFER_ISOC) {
		status = -EOPNOTSUPP;
		goto out;
	}

	/* Attempt to halt IN ep will fail if any transfer requests
	 * are still queue */
	if (value && ep_is_in(ep) && !list_empty(&ep->queue)) {
		status = -EAGAIN;
		goto out;
	}

	status = 0;
	ep_dir = ep_is_in(ep) ? USB_SEND : USB_RECV;
	ep_num = (unsigned char)(ep_index(ep));
	spin_lock_irqsave(&ep->udc->lock, flags);
	dr_ep_change_stall(ep_num, ep_dir, value);
	spin_unlock_irqrestore(&ep->udc->lock, flags);

	if (ep_index(ep) == 0) {
		udc->ep0_state = WAIT_FOR_SETUP;
		udc->ep0_dir = 0;
	}
out:
	VDBG(" %s %s halt stat %d", ep->ep.name,
			value ?  "set" : "clear", status);

	return status;
}

static int arcotg_fifo_status(struct usb_ep *_ep)
{
	struct fsl_ep *ep;
	struct fsl_udc *udc;
	int size = 0;
	u32 bitmask;
	struct ep_queue_head *d_qh;

	ep = container_of(_ep, struct fsl_ep, ep);
	if (!_ep || (!ep->desc && ep_index(ep) != 0))
		return -ENODEV;

	udc = (struct fsl_udc *)ep->udc;

	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	d_qh = &ep->udc->ep_qh[ep_index(ep) * 2 + ep_is_in(ep)];

	bitmask = (ep_is_in(ep)) ? (1 << (ep_index(ep) + 16)) :
	    (1 << (ep_index(ep)));

	if (fsl_readl(&dr_regs->endptstatus) & bitmask)
		size = (d_qh->size_ioc_int_sts & DTD_PACKET_SIZE)
		    >> DTD_LENGTH_BIT_POS;

	pr_debug("%s %u\n", __func__, size);
	return size;
}

static void fsl_ep_fifo_flush(struct usb_ep *_ep)
{
	struct fsl_ep *ep;
	int ep_num, ep_dir;
	u32 bits;
	unsigned long timeout;
	int retry = 100;
#define FSL_UDC_FLUSH_TIMEOUT 1000

	if (!_ep) {
		return;
	} else {
		ep = container_of(_ep, struct fsl_ep, ep);
		if (!ep->desc)
			return;
	}
	ep_num = ep_index(ep);
	ep_dir = ep_is_in(ep) ? USB_SEND : USB_RECV;

	if (ep_num == 0)
		bits = (1 << 16) | 1;
	else if (ep_dir == USB_SEND)
		bits = 1 << (16 + ep_num);
	else
		bits = 1 << ep_num;

	timeout = jiffies + FSL_UDC_FLUSH_TIMEOUT;
	do {
		fsl_writel(bits, &dr_regs->endptflush);

		/* Wait until flush complete */
		while (fsl_readl(&dr_regs->endptflush)) {
			if (time_after(jiffies, timeout)) {
				ERR("ep flush timeout\n");
				return;
			}
			cpu_relax();
		}

		/* Check the retry counter */
		retry--;
		if (!retry)
			break;

		/* See if we need to flush again */
	} while (fsl_readl(&dr_regs->endptstatus) & bits);
}

static struct usb_ep_ops fsl_ep_ops = {
	.enable = fsl_ep_enable,
	.disable = fsl_ep_disable,

	.alloc_request = fsl_alloc_request,
	.free_request = fsl_free_request,

	.queue = fsl_ep_queue,
	.dequeue = fsl_ep_dequeue,

	.set_halt = fsl_ep_set_halt,
	.fifo_status = arcotg_fifo_status,
	.fifo_flush = fsl_ep_fifo_flush,	/* flush fifo */
};

/*-------------------------------------------------------------------------
		Gadget Driver Layer Operations
-------------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 * Get the current frame number (from DR frame_index Reg )
 *----------------------------------------------------------------------*/
static int fsl_get_frame(struct usb_gadget *gadget)
{
	return (int)(fsl_readl(&dr_regs->frindex) & USB_FRINDEX_MASKS);
}

/*-----------------------------------------------------------------------
 * Tries to wake up the host connected to this gadget
 -----------------------------------------------------------------------*/
static int fsl_wakeup(struct usb_gadget *gadget)
{
	struct fsl_udc *udc = container_of(gadget, struct fsl_udc, gadget);
	u32 portsc;

	/* Remote wakeup feature not enabled by host */
	if (!udc->remote_wakeup)
		return -ENOTSUPP;

	portsc = fsl_readl(&dr_regs->portsc1);
	/* not suspended? */
	if (!(portsc & PORTSCX_PORT_SUSPEND))
		return 0;
	/* trigger force resume */
	portsc |= PORTSCX_PORT_FORCE_RESUME;
	fsl_writel(portsc, &dr_regs->portsc1);
	return 0;
}

static int can_pullup(struct fsl_udc *udc)
{
	return udc->driver && udc->softconnect && udc->vbus_active;
}

/* Notify controller that VBUS is powered, Called by whatever
   detects VBUS sessions */
static int fsl_vbus_session(struct usb_gadget *gadget, int is_active)
{
	struct fsl_udc	*udc;
	unsigned long	flags;

	udc = container_of(gadget, struct fsl_udc, gadget);
	spin_lock_irqsave(&udc->lock, flags);
	VDBG("VBUS %s\n", is_active ? "on" : "off");
	udc->vbus_active = (is_active != 0);
	if (can_pullup(udc))
		fsl_writel((fsl_readl(&dr_regs->usbcmd) | USB_CMD_RUN_STOP),
				&dr_regs->usbcmd);
	else
		fsl_writel((fsl_readl(&dr_regs->usbcmd) & ~USB_CMD_RUN_STOP),
				&dr_regs->usbcmd);
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

static void do_charge_current_work(struct work_struct *work) {
	fsl_set_current_limit(udc_controller, current_limit);
}

/* constrain controller's VBUS power usage
 * This call is used by gadget drivers during SET_CONFIGURATION calls,
 * reporting how much power the device may consume.  For example, this
 * could affect how quickly batteries are recharged.
 *
 * Returns zero on success, else negative errno.
 */
static int fsl_vbus_draw(struct usb_gadget *gadget, unsigned mA)
{
	atomic_set(&enumerated, 1);
	charger_enumerated = 1;

	/* may be called out of interrupt context, so set the charge current later */
	current_limit = mA;
	schedule_work(&charge_current_work);
	return 0;
}

/* Change Data+ pullup status
 * this func is used by usb_gadget_connect/disconnet
 */
static int fsl_pullup(struct usb_gadget *gadget, int is_on)
{
	struct fsl_udc *udc;

	udc = container_of(gadget, struct fsl_udc, gadget);
	udc->softconnect = (is_on != 0);
	if (can_pullup(udc))
		fsl_writel((fsl_readl(&dr_regs->usbcmd) | USB_CMD_RUN_STOP),
				&dr_regs->usbcmd);
	else
		fsl_writel((fsl_readl(&dr_regs->usbcmd) & ~USB_CMD_RUN_STOP),
				&dr_regs->usbcmd);

	return 0;
}

/* defined in gadget.h */
static struct usb_gadget_ops fsl_gadget_ops = {
	.get_frame = fsl_get_frame,
	.wakeup = fsl_wakeup,
/*	.set_selfpowered = fsl_set_selfpowered,	*/ /* Always selfpowered */
	.vbus_session = fsl_vbus_session,
	.vbus_draw = fsl_vbus_draw,
	.pullup = fsl_pullup,
};

/* Set protocol stall on ep0, protocol stall will automatically be cleared
   on new transaction */
static void ep0stall(struct fsl_udc *udc)
{
	u32 tmp;

	/* must set tx and rx to stall at the same time */
	tmp = fsl_readl(&dr_regs->endptctrl[0]);
	tmp |= EPCTRL_TX_EP_STALL | EPCTRL_RX_EP_STALL;
	fsl_writel(tmp, &dr_regs->endptctrl[0]);
	udc->ep0_state = WAIT_FOR_SETUP;
	udc->ep0_dir = 0;
}

/* Prime a status phase for ep0 */
static int ep0_prime_status(struct fsl_udc *udc, int direction)
{
	struct fsl_req *req = udc->status_req;
	struct fsl_ep *ep;
	int status = 0;

	if (direction == EP_DIR_IN)
		udc->ep0_dir = USB_DIR_IN;
	else
		udc->ep0_dir = USB_DIR_OUT;

	ep = &udc->eps[0];
	udc->ep0_state = WAIT_FOR_OUT_STATUS;

	req->ep = ep;
	req->req.length = 0;
	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->req.complete = NULL;
	req->dtd_count = 0;

	if (fsl_req_to_dtd(req) == 0)
		status = fsl_queue_td(ep, req);
	else
		return -ENOMEM;

	if (status)
		ERR("Can't queue ep0 status request \n");
	list_add_tail(&req->queue, &ep->queue);

	return status;
}

static inline int udc_reset_ep_queue(struct fsl_udc *udc, u8 pipe)
{
	struct fsl_ep *ep = get_ep_by_pipe(udc, pipe);

	if (!ep->name)
		return 0;

	nuke(ep, -ESHUTDOWN);

	return 0;
}

/*
 * ch9 Set address
 */
static void ch9setaddress(struct fsl_udc *udc, u16 value, u16 index, u16 length)
{
	/* Save the new address to device struct */
	udc->device_address = (u8) value;
	/* Update usb state */
	udc->usb_state = USB_STATE_ADDRESS;
	/* Status phase */
	if (ep0_prime_status(udc, EP_DIR_IN))
		ep0stall(udc);
}

/*
 * ch9 Get status
 */
static void ch9getstatus(struct fsl_udc *udc, u8 request_type, u16 value,
		u16 index, u16 length)
{
	u16 tmp = 0;		/* Status, cpu endian */

	struct fsl_req *req;
	struct fsl_ep *ep;
	int status = 0;

	ep = &udc->eps[0];

	if ((request_type & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
		/* Get device status */
		tmp = 1 << USB_DEVICE_SELF_POWERED;
		tmp |= udc->remote_wakeup << USB_DEVICE_REMOTE_WAKEUP;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_INTERFACE) {
		/* Get interface status */
		/* We don't have interface information in udc driver */
		tmp = 0;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) {
		/* Get endpoint status */
		struct fsl_ep *target_ep;

		target_ep = get_ep_by_pipe(udc, get_pipe_by_windex(index));

		/* stall if endpoint doesn't exist */
		if (!target_ep->desc)
			goto stall;
		tmp = dr_ep_get_stall(ep_index(target_ep), ep_is_in(target_ep))
				<< USB_ENDPOINT_HALT;
	}

	udc->ep0_dir = USB_DIR_IN;
	/* Borrow the per device status_req */
	req = udc->status_req;
	/* Fill in the reqest structure */
	*((u16 *) req->req.buf) = cpu_to_le16(tmp);
	req->ep = ep;
	req->req.length = 2;
	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->req.complete = NULL;
	req->dtd_count = 0;

	/* prime the data phase */
	if ((fsl_req_to_dtd(req) == 0))
		status = fsl_queue_td(ep, req);
	else			/* no mem */
		goto stall;

	if (status) {
		ERR("Can't respond to getstatus request \n");
		goto stall;
	}
	list_add_tail(&req->queue, &ep->queue);
	udc->ep0_state = DATA_STATE_XMIT;
	return;
stall:
	ep0stall(udc);
}

/*
 * Wait for the USB to get into test mode
 */
static void usbtest_workqueue_handler(struct work_struct *work)
{
	/*
	 * Rather than using mdelay(), use msleep() since it does not
	 * block the CPU. 
	 */
	msleep(10);
}

static void setup_received_irq(struct fsl_udc *udc,
		struct usb_ctrlrequest *setup)
{
	u16 wValue = le16_to_cpu(setup->wValue);
	u16 wIndex = le16_to_cpu(setup->wIndex);
	u16 wLength = le16_to_cpu(setup->wLength);

	udc_reset_ep_queue(udc, 0);

	/* We process some stardard setup requests here */
	switch (setup->bRequest) {
	case USB_REQ_GET_STATUS:
		/* Data+Status phase from udc */
		if ((setup->bRequestType & (USB_DIR_IN | USB_TYPE_MASK))
					!= (USB_DIR_IN | USB_TYPE_STANDARD))
			break;
		ch9getstatus(udc, setup->bRequestType, wValue, wIndex, wLength);
		return;

	case USB_REQ_SET_ADDRESS:
		/* Status phase from udc */
		if (setup->bRequestType != (USB_DIR_OUT | USB_TYPE_STANDARD
						| USB_RECIP_DEVICE))
			break;
		ch9setaddress(udc, wValue, wIndex, wLength);
		return;

	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
		/* Status phase from udc */
	{
		int rc = -EOPNOTSUPP;
		u16 ptc = 0;

		if ((setup->bRequestType & (USB_RECIP_MASK | USB_TYPE_MASK))
				== (USB_RECIP_ENDPOINT | USB_TYPE_STANDARD)) {
			int pipe = get_pipe_by_windex(wIndex);
			struct fsl_ep *ep;

			if (wValue != 0 || wLength != 0 || pipe > udc->max_ep)
				break;
			ep = get_ep_by_pipe(udc, pipe);

			spin_unlock(&udc->lock);
			rc = fsl_ep_set_halt(&ep->ep,
					(setup->bRequest == USB_REQ_SET_FEATURE)
						? 1 : 0);
			spin_lock(&udc->lock);

		} else if ((setup->bRequestType & (USB_RECIP_MASK
				| USB_TYPE_MASK)) == (USB_RECIP_DEVICE
				| USB_TYPE_STANDARD)) {
			/* Note: The driver has not include OTG support yet.
			 * This will be set when OTG support is added */
			if (setup->wValue == USB_DEVICE_TEST_MODE)
				ptc = setup->wIndex >> 8;
			else if (gadget_is_otg(&udc->gadget)) {
				if (setup->bRequest == USB_DEVICE_B_HNP_ENABLE)
					udc->gadget.b_hnp_enable = 1;
				else if (setup->bRequest == USB_DEVICE_A_HNP_SUPPORT)
					udc->gadget.a_hnp_support = 1;
				else if (setup->bRequest ==
						USB_DEVICE_A_ALT_HNP_SUPPORT)
					udc->gadget.a_alt_hnp_support = 1;
			}
			rc = 0;
		} else
			break;

		if (rc == 0) {
			if (ep0_prime_status(udc, EP_DIR_IN))
				ep0stall(udc);
		}

		if (ptc) {
			u32 tmp;

			if (ep0_prime_status(udc, EP_DIR_IN))
				ep0stall(udc);

			/*
			 *  Bits 16-19 are Port Test Control
			 */
			tmp = fsl_readl(&dr_regs->portsc1) | (ptc << 16);
			fsl_writel(tmp, &dr_regs->portsc1);

			pr_debug(KERN_INFO "arcotg_udc: switch to test mode %d.\n", ptc);
			
			/*
			 * We need a 10ms delay to wait for the USB to enter into
			 * test mode. However, we cannot mdelay() or msleep() here
			 * since this is an IRQ handler. So, we schedule a workqueue
			 */
			schedule_work(&udc->usbtest_work);
		}
			
		return;
	}

	default:
		break;
	}

	/* Requests handled by gadget */
	if (wLength) {
		/* Data phase from gadget, status phase from udc */
		udc->ep0_dir = (setup->bRequestType & USB_DIR_IN)
				?  USB_DIR_IN : USB_DIR_OUT;
		spin_unlock(&udc->lock);
		if (udc->driver->setup(&udc->gadget,
				&udc->local_setup_buff) < 0)
			ep0stall(udc);
		spin_lock(&udc->lock);
		udc->ep0_state = (setup->bRequestType & USB_DIR_IN)
				?  DATA_STATE_XMIT : DATA_STATE_RECV;
	} else {
		/* No data phase, IN status from gadget */
		udc->ep0_dir = USB_DIR_IN;
		spin_unlock(&udc->lock);
		if (udc->driver->setup(&udc->gadget,
				&udc->local_setup_buff) < 0)
			ep0stall(udc);
		spin_lock(&udc->lock);
		udc->ep0_state = WAIT_FOR_OUT_STATUS;
	}
}

/* Process request for Data or Status phase of ep0
 * prime status phase if needed */
static void ep0_req_complete(struct fsl_udc *udc, struct fsl_ep *ep0,
		struct fsl_req *req)
{
	if (udc->usb_state == USB_STATE_ADDRESS) {
		/* Set the new address */
		u32 new_address = (u32) udc->device_address;
		fsl_writel(new_address << USB_DEVICE_ADDRESS_BIT_POS,
				&dr_regs->deviceaddr);
	}

	done(ep0, req, 0);

	switch (udc->ep0_state) {
	case DATA_STATE_XMIT:
		/* receive status phase */
		if (ep0_prime_status(udc, EP_DIR_OUT))
			ep0stall(udc);
		break;
	case DATA_STATE_RECV:
		/* send status phase */
		if (ep0_prime_status(udc, EP_DIR_IN))
			ep0stall(udc);
		break;
	case WAIT_FOR_OUT_STATUS:
		udc->ep0_state = WAIT_FOR_SETUP;
		break;
	case WAIT_FOR_SETUP:
		ERR("Unexpect ep0 packets \n");
		break;
	default:
		ep0stall(udc);
		break;
	}
}

static void iram_process_ep_complete(struct fsl_req *curr_req,
					int cur_transfer)
{
	char *buf;
	u32 len;
	int in = ep_is_in(curr_req->ep);

	if (in)
		buf = (char *)udc_controller->iram_buffer_v[1];
	else
		buf = (char *)udc_controller->iram_buffer_v[0];

	if (curr_req->cur->next_td_ptr == cpu_to_hc32(DTD_NEXT_TERMINATE)
		|| (cur_transfer < g_iram_size)
		|| (curr_req->req.length == curr_req->req.actual))
			curr_req->last_one = 1;

	if (curr_req->last_one) {
		if (!in) {
			memcpy(curr_req->req.buf + curr_req->buffer_offset, buf,
				cur_transfer);
		}
		if (curr_req->tail->next_td_ptr !=
			cpu_to_hc32(DTD_NEXT_TERMINATE)) {
				struct fsl_req *next_req;
				next_req =
					list_entry(curr_req->queue.next,
						struct fsl_req, queue);
				if (in)
					memcpy(buf, next_req->req.buf,
						min(g_iram_size, next_req->req.length));	
				update_qh(next_req);
		}
	}
	else {
		curr_req->buffer_offset += g_iram_size;
		curr_req->cur->next_td_ptr &= ~cpu_to_hc32(DTD_NEXT_TERMINATE);
		curr_req->cur = curr_req->cur->next_td_virt;
		if (in) {
			len =
			min(curr_req->req.length - curr_req->buffer_offset,
				g_iram_size);

			memcpy(buf, curr_req->req.buf + curr_req->buffer_offset,
				len);
		}
		else {
			memcpy(curr_req->req.buf + curr_req->buffer_offset -
				g_iram_size, buf, g_iram_size);
		}
		update_qh(curr_req);
	}
}

/* Tripwire mechanism to ensure a setup packet payload is extracted without
 * being corrupted by another incoming setup packet */
static void tripwire_handler(struct fsl_udc *udc, u8 ep_num, u8 *buffer_ptr)
{
	u32 temp;
	struct ep_queue_head *qh;
	struct fsl_usb2_platform_data *pdata = udc->pdata;

	qh = &udc->ep_qh[ep_num * 2 + EP_DIR_OUT];

	/* Clear bit in ENDPTSETUPSTAT */
	temp = fsl_readl(&dr_regs->endptsetupstat);
	fsl_writel(temp | (1 << ep_num), &dr_regs->endptsetupstat);

	/* while a hazard exists when setup package arrives */
	do {
		/* Set Setup Tripwire */
		temp = fsl_readl(&dr_regs->usbcmd);
		fsl_writel(temp | USB_CMD_SUTW, &dr_regs->usbcmd);

		/* Copy the setup packet to local buffer */
		if (pdata->le_setup_buf) {
			u32 *p = (u32 *)buffer_ptr;
			u32 *s = (u32 *)qh->setup_buffer;

			/* Convert little endian setup buffer to CPU endian */
			*p++ = le32_to_cpu(*s++);
			*p = le32_to_cpu(*s);
		} else {
			memcpy(buffer_ptr, (u8 *) qh->setup_buffer, 8);
		}
	} while (!(fsl_readl(&dr_regs->usbcmd) & USB_CMD_SUTW));

	/* Clear Setup Tripwire */
	temp = fsl_readl(&dr_regs->usbcmd);
	fsl_writel(temp & ~USB_CMD_SUTW, &dr_regs->usbcmd);
}

/* process-ep_req(): free the completed Tds for this req */
static int process_ep_req(struct fsl_udc *udc, int pipe,
		struct fsl_req *curr_req)
{
	struct ep_td_struct *curr_td;
	int	td_complete, actual, remaining_length, j, tmp;
	int	status = 0;
	int	errors = 0;
	struct  ep_queue_head *curr_qh = &udc->ep_qh[pipe];
	int direction = pipe % 2;
	int total = 0, real_len;

	curr_td = curr_req->head;
	td_complete = 0;
	actual = curr_req->req.length;

	for (j = 0; j < curr_req->dtd_count; j++) {
		remaining_length = (hc32_to_cpu(curr_td->size_ioc_sts)
					& DTD_PACKET_SIZE)
				>> DTD_LENGTH_BIT_POS;
		if (NEED_IRAM(curr_req->ep)) {
			if (real_len >= g_iram_size) {
				actual = g_iram_size;
				real_len -= g_iram_size;
			}
			else {
				actual = real_len;
				curr_req->last_one = 1;
			}
		}
		actual -= remaining_length;
		total += actual;

		errors = hc32_to_cpu(curr_td->size_ioc_sts) & DTD_ERROR_MASK;
		if (errors) {
			if (errors & DTD_STATUS_HALTED) {
				ERR("dTD error %08x QH=%d\n", errors, pipe);
				/* Clear the errors and Halt condition */
				tmp = hc32_to_cpu(curr_qh->size_ioc_int_sts);
				tmp &= ~errors;
				curr_qh->size_ioc_int_sts = cpu_to_hc32(tmp);
				status = -EPIPE;
				/* FIXME: continue with next queued TD? */

				break;
			}
			if (errors & DTD_STATUS_DATA_BUFF_ERR) {
				VDBG("Transfer overflow");
				status = -EPROTO;
				break;
			} else if (errors & DTD_STATUS_TRANSACTION_ERR) {
				VDBG("ISO error");
				status = -EILSEQ;
				break;
			} else
				ERR("Unknown error has occured (0x%x)!\r\n",
					errors);

		} else if (hc32_to_cpu(curr_td->size_ioc_sts)
				& DTD_STATUS_ACTIVE) {
			VDBG("Request not complete");
			status = REQ_UNCOMPLETE;
			return status;
		} else if (remaining_length) {
			if (direction) {
				VDBG("Transmit dTD remaining length not zero");
				status = -EPROTO;
				break;
			} else {
				td_complete++;
				break;
			}
		} else {
			td_complete++;
			VDBG("dTD transmitted successful ");
		}

		if (NEED_IRAM(curr_req->ep))
			if (curr_td->
				next_td_ptr & cpu_to_hc32(DTD_NEXT_TERMINATE))
					break;

		if (j != curr_req->dtd_count - 1)
			curr_td = (struct ep_td_struct *)curr_td->next_td_virt;
	}

	if (status)
		return status;

	curr_req->req.actual = total;
	if (NEED_IRAM(curr_req->ep))
		iram_process_ep_complete(curr_req, actual);

	return 0;
}

/* Process a DTD completion interrupt */
static void dtd_complete_irq(struct fsl_udc *udc)
{
	u32 bit_pos;
	int i, ep_num, direction, bit_mask, status;
	struct fsl_ep *curr_ep;
	struct fsl_req *curr_req, *temp_req;

	/* Clear the bits in the register */
	bit_pos = fsl_readl(&dr_regs->endptcomplete);
	fsl_writel(bit_pos, &dr_regs->endptcomplete);

	if (!bit_pos)
		return;

	for (i = 0; i < udc->max_ep * 2; i++) {
		ep_num = i >> 1;
		direction = i % 2;

		bit_mask = 1 << (ep_num + 16 * direction);

		if (!(bit_pos & bit_mask))
			continue;

		curr_ep = get_ep_by_pipe(udc, i);

		/* If the ep is configured */
		if (curr_ep->name == NULL) {
			WARN_ON("Invalid EP?");
			continue;
		}

		/* process the req queue until an uncomplete request */
		list_for_each_entry_safe(curr_req, temp_req, &curr_ep->queue,
				queue) {
			status = process_ep_req(udc, i, curr_req);

			VDBG("status of process_ep_req= %d, ep = %d",
					status, ep_num);
			if (status == REQ_UNCOMPLETE)
				break;
			/* write back status to req */
			curr_req->req.status = status;

			if (ep_num == 0) {
				ep0_req_complete(udc, curr_ep, curr_req);
				break;
			} else {
				if (NEED_IRAM(curr_ep)) {
					if (curr_req->last_one)
						done(curr_ep, curr_req, status);
					break;
				}
				else {
					done(curr_ep, curr_req, status);
				}
			}
		}

		dump_ep_queue(curr_ep);
	}
}

/* Late check to make sure, the device got enumerated */
static void port_change_work(struct work_struct *work)
{
	if (!is_charger_connected())
		return;

	if ((udc_controller->gadget.speed == USB_SPEED_UNKNOWN)
			|| (!udc_controller->gadget.configured) ) {
		fsl_udc_redo_enumerate();
	}


}

/* Process a port change interrupt */
static void port_change_irq(struct fsl_udc *udc)
{
	u32 speed;
	unsigned int portsc1;

	if (udc->bus_reset)
		udc->bus_reset = 0;

	/* Bus resetting is finished */
	if (!(fsl_readl(&dr_regs->portsc1) & PORTSCX_PORT_RESET)) {
		/* Get the speed */
		speed = (fsl_readl(&dr_regs->portsc1)
				& PORTSCX_PORT_SPEED_MASK);
		switch (speed) {
		case PORTSCX_PORT_SPEED_HIGH:
			udc->gadget.speed = USB_SPEED_HIGH;
			break;
		case PORTSCX_PORT_SPEED_FULL:
			udc->gadget.speed = USB_SPEED_FULL;
			break;
		case PORTSCX_PORT_SPEED_LOW:
			udc->gadget.speed = USB_SPEED_LOW;
			break;
		default:
			udc->gadget.speed = USB_SPEED_UNKNOWN;
			break;
		}
	}

	portsc1 = fsl_readl(&dr_regs->portsc1);
	/* Check the High Speed Port settings */
	if ((portsc1 & PORTSCX_CURRENT_CONNECT_STATUS) &&
		(portsc1 & PORTSCX_PORT_HSP)) {
			udc->gadget.speed = USB_SPEED_HIGH;
	}

	/* Update USB state */
	if (!udc->resume_state)
		udc->usb_state = USB_STATE_DEFAULT;

	/* Late workqueue to check if re-enumeration needed */
	schedule_delayed_work(&pc_wq, msecs_to_jiffies(PC_WORK_THRESHOLD));
}

/* Process suspend interrupt */
static void suspend_irq(struct fsl_udc *udc)
{
	pr_debug("%s\n", __func__);

	udc->resume_state = udc->usb_state;
	udc->usb_state = USB_STATE_SUSPENDED;

	/* report suspend to the driver, serial.c does not support this */
	if (udc->driver && udc->driver->suspend)
		udc->driver->suspend(&udc->gadget);
}

static void bus_resume(struct fsl_udc *udc)
{
	udc->usb_state = udc->resume_state;
	udc->resume_state = 0;

	/* report resume to the driver, serial.c does not support this */
	if (udc->driver && udc->driver->resume)
		udc->driver->resume(&udc->gadget);
}

/* Clear up all ep queues */
static int reset_queues(struct fsl_udc *udc)
{
	u8 pipe;

	for (pipe = 0; pipe < udc->max_pipes; pipe++)
		udc_reset_ep_queue(udc, pipe);

	/* report disconnect; the driver is already quiesced */
	if (udc->driver)
		udc->driver->disconnect(&udc->gadget);

	return 0;
}

/* Process reset interrupt */
static void reset_irq(struct fsl_udc *udc)
{
	u32 temp;

	/* Clear the device address */
	temp = fsl_readl(&dr_regs->deviceaddr);
	fsl_writel(temp & ~USB_DEVICE_ADDRESS_MASK, &dr_regs->deviceaddr);

	udc->device_address = 0;

	/* Clear usb state */
	udc->resume_state = 0;
	udc->ep0_dir = 0;
	udc->ep0_state = WAIT_FOR_SETUP;
	udc->remote_wakeup = 0;	/* default to 0 on reset */
	udc->gadget.b_hnp_enable = 0;
	udc->gadget.a_hnp_support = 0;
	udc->gadget.a_alt_hnp_support = 0;

	/* Clear all the setup token semaphores */
	temp = fsl_readl(&dr_regs->endptsetupstat);
	fsl_writel(temp, &dr_regs->endptsetupstat);

	/* Clear all the endpoint complete status bits */
	temp = fsl_readl(&dr_regs->endptcomplete);
	fsl_writel(temp, &dr_regs->endptcomplete);

	/* Write 1s to the flush register */
	fsl_writel(0xffffffff, &dr_regs->endptflush);

	/* Bus is reseting */
	udc->bus_reset = 1;
	/* Reset all the queues, include XD, dTD, EP queue
	 * head and TR Queue */
	reset_queues(udc);
	udc->usb_state = USB_STATE_DEFAULT;
}

/*
 * USB device controller interrupt handler
 */
static irqreturn_t fsl_udc_irq(int irq, void *_udc)
{
	struct fsl_udc *udc = _udc;
	u32 irq_src;
	irqreturn_t status = IRQ_HANDLED;
	unsigned long flags;

	/* Disable ISR for OTG host mode */
	if (udc->stopped)
		return IRQ_NONE;

	spin_lock_irqsave(&udc->lock, flags);
	irq_src = fsl_readl(&dr_regs->usbsts) & fsl_readl(&dr_regs->usbintr);
	/* Clear notification bits */
	fsl_writel(irq_src, &dr_regs->usbsts);

	VDBG("irq_src [0x%8x]", irq_src); 

	if (!udc->driver) {
		spin_unlock_irqrestore(&udc->lock, flags);
		return IRQ_HANDLED;
	}
	
	/* Need to resume? */
	if (udc->usb_state == USB_STATE_SUSPENDED)
		if ((fsl_readl(&dr_regs->portsc1) & PORTSCX_PORT_SUSPEND) == 0)
			bus_resume(udc);

	/* USB Interrupt */
	if (irq_src & USB_STS_INT) {
		VDBG("Packet int");
		/* Setup package, we only support ep0 as control ep */
		if (fsl_readl(&dr_regs->endptsetupstat) & EP_SETUP_STATUS_EP0) {
			if (udc_detection) {
				tripwire_handler(udc, 0,
						(u8 *) (&udc->local_setup_buff));
				setup_received_irq(udc, &udc->local_setup_buff);
			}
			/* Host detection */
			host_detected = 1;
			status = IRQ_HANDLED;
		}

		/* completion of dtd */
		if (fsl_readl(&dr_regs->endptcomplete)) {
			dtd_complete_irq(udc);
			status = IRQ_HANDLED;
		}
	}

	/* SOF (for ISO transfer) */
	if (irq_src & USB_STS_SOF) {
		status = IRQ_HANDLED;
	}

	/* Port Change */
	if (irq_src & USB_STS_PORT_CHANGE) {
		port_change_irq(udc);
		status = IRQ_HANDLED;
	}

	/* Reset Received */
	if (irq_src & USB_STS_RESET) {
		VDBG("reset int");
		reset_irq(udc);
		status = IRQ_HANDLED;
	}

	/* Sleep Enable (Suspend) */
	if (irq_src & USB_STS_SUSPEND) {
		suspend_irq(udc);
		status = IRQ_HANDLED;
	}

	if (irq_src & (USB_STS_ERR | USB_STS_SYS_ERR)) {
		printk(KERN_ERR "Error IRQ %x ", irq_src);
		if (irq_src & USB_STS_SYS_ERR) {
			printk(KERN_ERR "This error can't be recoveried, \
					please reboot your board\n");
			printk(KERN_ERR "If this error happens frequently, \
					please check your dma buffer\n");
		}
	}

	spin_unlock_irqrestore(&udc->lock, flags);
	return status;
}

/*----------------------------------------------------------------*
 * Hook to gadget drivers
 * Called by initialization code of gadget drivers
*----------------------------------------------------------------*/
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	int retval = -ENODEV;
	unsigned long flags = 0;

	if (!udc_controller)
		return -ENODEV;

	if (!driver || (driver->speed != USB_SPEED_FULL
				&& driver->speed != USB_SPEED_HIGH)
			|| !driver->bind || !driver->disconnect
			|| !driver->setup)
		return -EINVAL;

	if (udc_controller->driver)
		return -EBUSY;

	/* lock is needed but whether should use this lock or another */
	spin_lock_irqsave(&udc_controller->lock, flags);

	driver->driver.bus = 0;
	/* hook up the driver */
	udc_controller->driver = driver;
	udc_controller->gadget.dev.driver = &driver->driver;
	spin_unlock_irqrestore(&udc_controller->lock, flags);

	/* bind udc driver to gadget driver */
	retval = driver->bind(&udc_controller->gadget);
	if (retval) {
		VDBG("bind to %s --> %d", driver->driver.name, retval);
		udc_controller->gadget.dev.driver = 0;
		udc_controller->driver = 0;
		goto out;
	}

	if (udc_controller->transceiver) {
		/* connect to bus through transceiver */
		retval = otg_set_peripheral(udc_controller->transceiver,
				&udc_controller->gadget);
		if (retval < 0) {
			ERR("can't bind to transceiver\n");
			driver->unbind(&udc_controller->gadget);
			udc_controller->gadget.dev.driver = 0;
			udc_controller->driver = 0;
			return retval;
		}
	} else {
		/* Enable DR IRQ reg and Set usbcmd reg  Run bit */
		dr_controller_run(udc_controller);
		udc_controller->usb_state = USB_STATE_ATTACHED;
		udc_controller->ep0_state = WAIT_FOR_SETUP;
		udc_controller->ep0_dir = 0;
	}
	pr_debug("%s: bind to driver %s \n",
			udc_controller->gadget.name, driver->driver.name);

out:
	if (retval)
		printk(KERN_DEBUG "retval %d \n", retval);
	return retval;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

/* Disconnect from gadget driver */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct fsl_ep *loop_ep;
	unsigned long flags;
	int ret;

	if (!udc_controller)
		return -ENODEV;

	if (!driver || driver != udc_controller->driver || !driver->unbind)
		return -EINVAL;

	if (udc_controller->transceiver) {
		/* We still need the port to be active for flushing the endpoints
		 * later so we need to avoid that the OTG's call to fsl_udc_suspend
		 * leads to a dead OTG port.
		 * So we have to grab an additional ref to the port here and release
		 * it later when all the endpoints have been flushed.
		 */
		if ((ret = usbotg_low_power_exit())) {
			return ret;
		}

		(void)otg_set_peripheral(udc_controller->transceiver, 0);
	}

	/* in fact, no needed */
	udc_controller->usb_state = USB_STATE_ATTACHED;
	udc_controller->ep0_state = WAIT_FOR_SETUP;
	udc_controller->ep0_dir = 0;

	/* stand operation */
	spin_lock_irqsave(&udc_controller->lock, flags);
	udc_controller->gadget.speed = USB_SPEED_UNKNOWN;
	nuke(&udc_controller->eps[0], -ESHUTDOWN);
	list_for_each_entry(loop_ep, &udc_controller->gadget.ep_list,
			ep.ep_list)
		nuke(loop_ep, -ESHUTDOWN);
	/* stop DR, disable intr */
	dr_controller_stop(udc_controller);
	spin_unlock_irqrestore(&udc_controller->lock, flags);

	if (udc_controller->transceiver) {
		/* release the port handle for the EP flush work */
		if ((ret = usbotg_low_power_enter())) {
			printk(KERN_ERR "%s: ERROR: Could not enter low power mode. Error=%d. Not fatal.",
					__func__, ret);
		}
	}

	if (driver->disconnect)
		driver->disconnect(&udc_controller->gadget);

	/* unbind gadget and unhook driver. */
	driver->unbind(&udc_controller->gadget);
	udc_controller->gadget.dev.driver = 0;
	udc_controller->driver = 0;

	printk(KERN_DEBUG "unregistered gadget driver '%s'\r\n",
	       driver->driver.name);
	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

#ifdef CONFIG_DEBUG_FS

/*-------------------------------------------------------------------------
	Debug filesystem
-------------------------------------------------------------------------*/
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int fsl_udc_show(struct seq_file *s, void *p)
{
	unsigned long flags;
	int t, i;
	u32 tmp_reg;
	struct fsl_ep *ep = NULL;
	struct fsl_req *req;
	struct fsl_usb2_platform_data *pdata;
	struct fsl_udc *udc = udc_controller;

	pdata = udc->pdata;

	spin_lock_irqsave(&udc->lock, flags);

	/* ------basic driver infomation ---- */
	t = seq_printf(s,
			DRIVER_DESC "\n"
			"%s version: %s\n"
			"Driver Author: %s\n"
			"Gadget driver: %s\n\n",
			driver_name, DRIVER_VERSION, DRIVER_AUTHOR,
			udc->driver ? udc->driver->driver.name : "(none)");

	/* ------ DR Registers ----- */
	tmp_reg = fsl_readl(&dr_regs->usbcmd);
	t += seq_printf(s,
			"USBCMD reg: 0x%x\n"
			"SetupTW: %d\n"
			"Run/Stop: %s\n\n",
			tmp_reg,
			(tmp_reg & USB_CMD_SUTW) ? 1 : 0,
			(tmp_reg & USB_CMD_RUN_STOP) ? "Run" : "Stop");

	tmp_reg = fsl_readl(&dr_regs->usbsts);
	t += seq_printf(s,
			"USB Status Reg: 0x%x\n"
			"Dr Suspend: %d" "Reset Received: %d" "System Error: %s"
			"USB Error Interrupt: %s\n\n",
			tmp_reg,
			(tmp_reg & USB_STS_SUSPEND) ? 1 : 0,
			(tmp_reg & USB_STS_RESET) ? 1 : 0,
			(tmp_reg & USB_STS_SYS_ERR) ? "Err" : "Normal",
			(tmp_reg & USB_STS_ERR) ? "Err detected" : "No err");

	tmp_reg = fsl_readl(&dr_regs->usbintr);
	t += seq_printf(s,
			"USB Interrupt Enable Reg: 0x%x\n"
			"Sleep Enable: %d" "SOF Received Enable: %d"
			"Reset Enable: %d\n"
			"System Error Enable: %d"
			"Port Change Dectected Enable: %d\n"
			"USB Error Intr Enable: %d" "USB Intr Enable: %d\n\n",
			tmp_reg,
			(tmp_reg & USB_INTR_DEVICE_SUSPEND) ? 1 : 0,
			(tmp_reg & USB_INTR_SOF_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_RESET_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_SYS_ERR_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_PTC_DETECT_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_ERR_INT_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_INT_EN) ? 1 : 0);

	tmp_reg = fsl_readl(&dr_regs->frindex);
	t += seq_printf(s,
			"USB Frame Index Reg:" "Frame Number is 0x%x\n\n",
			(tmp_reg & USB_FRINDEX_MASKS));

	tmp_reg = fsl_readl(&dr_regs->deviceaddr);
	t += seq_printf(s,
			"USB Device Address Reg:" "Device Addr is 0x%x\n\n",
			(tmp_reg & USB_DEVICE_ADDRESS_MASK));

	tmp_reg = fsl_readl(&dr_regs->endpointlistaddr);
	t += seq_printf(s,
			"USB Endpoint List Address Reg:"
			"Device Addr is 0x%x\n\n",
			(tmp_reg & USB_EP_LIST_ADDRESS_MASK));

	tmp_reg = fsl_readl(&dr_regs->portsc1);
	t += seq_printf(s,
		"USB Port Status&Control Reg: 0x%x\n"
		"Port Transceiver Type : %s" "Port Speed: %s \n"
		"PHY Low Power Suspend: %s" "Port Reset: %s"
		"Port Suspend Mode: %s \n" "Over-current Change: %s"
		"Port Enable/Disable Change: %s\n"
		"Port Enabled/Disabled: %s"
		"Current Connect Status: %s\n\n", tmp_reg, ({
			char *s;
			switch (tmp_reg & PORTSCX_PTS_FSLS) {
			case PORTSCX_PTS_UTMI:
				s = "UTMI"; break;
			case PORTSCX_PTS_ULPI:
				s = "ULPI "; break;
			case PORTSCX_PTS_FSLS:
				s = "FS/LS Serial"; break;
			default:
				s = "None"; break;
			}
			s; }), ({
			char *s;
			switch (tmp_reg & PORTSCX_PORT_SPEED_UNDEF) {
			case PORTSCX_PORT_SPEED_FULL:
				s = "Full Speed"; break;
			case PORTSCX_PORT_SPEED_LOW:
				s = "Low Speed"; break;
			case PORTSCX_PORT_SPEED_HIGH:
				s = "High Speed"; break;
			default:
				s = "Undefined"; break;
			}
			s;
		}),
		(tmp_reg & PORTSCX_PHY_LOW_POWER_SPD) ?
		"Normal PHY mode" : "Low power mode",
		(tmp_reg & PORTSCX_PORT_RESET) ? "In Reset" :
		"Not in Reset",
		(tmp_reg & PORTSCX_PORT_SUSPEND) ? "In " : "Not in",
		(tmp_reg & PORTSCX_OVER_CURRENT_CHG) ? "Dected" :
		"No",
		(tmp_reg & PORTSCX_PORT_EN_DIS_CHANGE) ? "Disable" :
		"Not change",
		(tmp_reg & PORTSCX_PORT_ENABLE) ? "Enable" :
		"Not correct",
		(tmp_reg & PORTSCX_CURRENT_CONNECT_STATUS) ?
		"Attached" : "Not-Att");

	tmp_reg = fsl_readl(&dr_regs->usbmode);
	t += seq_printf(s,
			"USB Mode Reg:" "Controller Mode is : %s\n\n", ({
				char *s;
				switch (tmp_reg & USB_MODE_CTRL_MODE_HOST) {
				case USB_MODE_CTRL_MODE_IDLE:
					s = "Idle"; break;
				case USB_MODE_CTRL_MODE_DEVICE:
					s = "Device Controller"; break;
				case USB_MODE_CTRL_MODE_HOST:
					s = "Host Controller"; break;
				default:
					s = "None"; break;
				}
				s;
			}));

	tmp_reg = fsl_readl(&dr_regs->endptsetupstat);
	t += seq_printf(s,
			"Endpoint Setup Status Reg:" "SETUP on ep 0x%x\n\n",
			(tmp_reg & EP_SETUP_STATUS_MASK));

	for (i = 0; i < udc->max_ep / 2; i++) {
		tmp_reg = fsl_readl(&dr_regs->endptctrl[i]);
		t += seq_printf(s, "EP Ctrl Reg [0x%x]: = [0x%x]\n",
				i, tmp_reg);
	}
	tmp_reg = fsl_readl(&dr_regs->endpointprime);
	t += seq_printf(s, "EP Prime Reg = [0x%x]\n", tmp_reg);

	if (pdata->have_sysif_regs) {
		tmp_reg = usb_sys_regs->snoop1;
		t += seq_printf(s, "\nSnoop1 Reg = [0x%x]\n\n", tmp_reg);

		tmp_reg = usb_sys_regs->control;
		t += seq_printf(s, "General Control Reg = [0x%x]\n\n",
				tmp_reg);
	}

	/* ------fsl_udc, fsl_ep, fsl_request structure information ----- */
	ep = &udc->eps[0];
	t += seq_printf(s, "For %s Maxpkt is 0x%x index is 0x%x\n",
			ep->ep.name, ep_maxpacket(ep), ep_index(ep));

	if (list_empty(&ep->queue)) {
		t += seq_printf(s, "its req queue is empty\n\n");
	} else {
		list_for_each_entry(req, &ep->queue, queue) {
			t += seq_printf(s,
				"req %p actual 0x%x length 0x%x  buf %p\n",
				&req->req, req->req.actual,
				req->req.length, req->req.buf);
		}
	}
	/* other gadget->eplist ep */
	list_for_each_entry(ep, &udc->gadget.ep_list, ep.ep_list) {
		if (ep->desc) {
			t += seq_printf(s,
					"\nFor %s Maxpkt is 0x%x "
					"index is 0x%x\n",
					ep->ep.name, ep_maxpacket(ep),
					ep_index(ep));

			if (list_empty(&ep->queue)) {
				t += seq_printf(s,
						"its req queue is empty\n\n");
			} else {
				list_for_each_entry(req, &ep->queue, queue) {
					t += seq_printf(s,
						"req %p actual 0x%x length"
						"0x%x  buf %p\n",
						&req->req, req->req.actual,
						req->req.length, req->req.buf);
					} /* end for each_entry of ep req */
				}	/* end for else */
			}	/* end for if(ep->queue) */
		}		/* end (ep->desc) */

	spin_unlock_irqrestore(&udc->lock, flags);
	
	return 0;
}

static int fsl_udc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fsl_udc_show, inode->i_private);
}

static const struct file_operations fsl_udc_dbg_fops = {
	.owner		= THIS_MODULE,
	.open		= fsl_udc_open,
	.llseek		= seq_lseek,
	.read		= seq_read,
	.release	= single_release,
};

static void fsl_udc_init_debugfs(struct fsl_udc *udc)
{
	struct dentry *root, *state;

	root = debugfs_create_dir(udc->gadget.name, NULL);
	if (IS_ERR(root) || !root)
		goto err_root;

	state = debugfs_create_file("udcstate", 0400, root, udc,
				&fsl_udc_dbg_fops);
	if (!state)
		goto err_state;
	
	udc->debugfs_root = root;
	udc->debugfs_state = state;
	return;
err_state:
	debugfs_remove(root);
err_root:
	printk(KERN_ERR "udc: debugfs is not available\n");
}

static void fsl_udc_cleanup_debugfs(struct fsl_udc *udc)
{
	debugfs_remove(udc->debugfs_state);
	debugfs_remove(udc->debugfs_root);

	udc->debugfs_state = NULL;
	udc->debugfs_root = NULL;
}

#else

static void fsl_udc_init_debugfs(struct fsl_udc *udc)
{
}

static void fsl_udc_cleanup_debugfs(struct fsl_udc *udc)
{
}

#endif

/*-------------------------------------------------------------------------*/

/* Release udc structures */
static void fsl_udc_release(struct device *dev)
{
	complete(udc_controller->done);
	dma_free_coherent(dev->parent, udc_controller->ep_qh_size,
			udc_controller->ep_qh, udc_controller->ep_qh_dma);
	kfree(udc_controller);
}

/******************************************************************
	Internal structure setup functions
*******************************************************************/
/*------------------------------------------------------------------
 * init resource for globle controller
 * Return the udc handle on success or NULL on failure
 ------------------------------------------------------------------*/
static int __init struct_udc_setup(struct fsl_udc *udc,
		struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata;
	size_t size;

	pdata = pdev->dev.platform_data;
	udc->phy_mode = pdata->phy_mode;

	udc->eps = kzalloc(sizeof(struct fsl_ep) * udc->max_ep, GFP_KERNEL);
	if (!udc->eps) {
		ERR("malloc fsl_ep failed\n");
		return -1;
	}

	/* initialized QHs, take care of alignment */
	size = udc->max_ep * sizeof(struct ep_queue_head);
	if (size < QH_ALIGNMENT)
		size = QH_ALIGNMENT;
	else if ((size % QH_ALIGNMENT) != 0) {
		size += QH_ALIGNMENT + 1;
		size &= ~(QH_ALIGNMENT - 1);
	}
	udc->ep_qh = dma_alloc_coherent(&pdev->dev, size,
					&udc->ep_qh_dma, GFP_KERNEL);
	if (!udc->ep_qh) {
		ERR("malloc QHs for udc failed\n");
		kfree(udc->eps);
		return -1;
	}

	udc->ep_qh_size = size;

	/* Initialize ep0 status request structure */
	/* FIXME: fsl_alloc_request() ignores ep argument */
	udc->status_req = container_of(fsl_alloc_request(NULL, GFP_KERNEL),
			struct fsl_req, req);
	/* allocate a small amount of memory to get valid address */
	udc->status_req->req.buf = kmalloc(8, GFP_KERNEL);
	udc->status_req->req.dma = virt_to_phys(udc->status_req->req.buf);

	udc->resume_state = USB_STATE_NOTATTACHED;
	udc->usb_state = USB_STATE_POWERED;
	udc->ep0_dir = 0;
	udc->remote_wakeup = 0;	/* default to 0 on reset */
	spin_lock_init(&udc->lock);

	return 0;
}

static void suspend_ctrl(void)
{
	unsigned int usbcmd;

	usbcmd = fsl_readl(&dr_regs->usbcmd);
        usbcmd &= ~USB_CMD_RUN_STOP;
        fsl_writel(usbcmd, &dr_regs->usbcmd);
}

static void resume_ctrl(void)
{
	unsigned int usbcmd;
	usbcmd = fsl_readl(&dr_regs->usbcmd);
	usbcmd |= USB_CMD_RUN_STOP;
	fsl_writel(usbcmd, &dr_regs->usbcmd);
}

/* Redo a detach-attach event */
static void fsl_udc_redo_enumerate(void)
{	
	/* Pull D+ low */
	suspend_ctrl();

	/* Settling time */
	mdelay(50);

	/* Attach event */
	resume_ctrl();
}

static ssize_t
show_accessory_charger(struct device *dev, struct device_attribute *attr, char *buf)
{
	return (gpio_accessory_charger_detected());
}

static ssize_t
show_connected(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (is_charger_connected())
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");
}

static ssize_t
show_supply_voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	return -ENOSYS;
}

static ssize_t
show_chrgraw_voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	return -ENOSYS;
}

/*
 * Battery Voltage
 */
static ssize_t
show_voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	return show_supply_voltage(dev, attr, buf);
}

/*
 * BPSNS value
 */
static ssize_t
show_bpsns(struct device *dev, struct device_attribute *attr, char *buf)
{
	return -ENOSYS;
}

/*
 * this is the charging flag that indicates that the device is charging
 */
static ssize_t
show_charging(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (charger_get_charge_current() > 1)
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");
}

/*!
 * Indicates a lobat condition has been hit
 */
static ssize_t
show_lobat_condition(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", charger_have_low_battery());
}

/*
 * Battery current
 */
static ssize_t
show_battery_current(struct device *dev, struct device_attribute *attr, char *buf)
{
	int battery_current_uA;
	int status = charger_get_battery_current(&battery_current_uA);

	if (status < 0)
		return sprintf(buf, "0\n");
	else
		return sprintf(buf, "%d\n", battery_current_uA);
}

/*
 * Fake battery error file till a battery driver is written 
 */
static ssize_t
show_batt_error(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 0);
}

/*!
 * Battery ID
 */
static ssize_t
show_battery_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	return -ENOSYS;
}

/*!
 * Third party charger
 */
static ssize_t
show_third_party(struct device *dev, struct device_attribute *attr, char *buf)
{
	return -ENOSYS;
}

/*
 * Show current ichrg value
 */
static ssize_t
show_ichrg_setting(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", charger_get_charge_current());
}

/* Third party detect timer */
static ssize_t
show_third_party_timer(struct device *_dev, struct device_attribute *attr, char *buf)
{
	return -ENOSYS;
}

static ssize_t
store_third_party_timer(struct device *_dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	return -ENOSYS;
}
static DEVICE_ATTR (third_party_timer, S_IRUGO|S_IWUSR, show_third_party_timer, store_third_party_timer);

/* USB LPM timer */
static ssize_t
show_usb_lpm_timer_threshold(struct device *_dev, struct device_attribute *attr, char *buf)
{
	return -ENOSYS;
}

static ssize_t
store_usb_lpm_timer_threshold(struct device *_dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int value;

	if ( (sscanf(buf, "%d", &value) > 0) ) {
		return -ENOSYS;
	}

	return -EINVAL;
}

static DEVICE_ATTR (usb_lpm_timer_threshold, S_IRUGO|S_IWUSR, show_usb_lpm_timer_threshold, store_usb_lpm_timer_threshold);
	
/* charger enable/disable on the fly */
static ssize_t
show_charger_state(struct device *_dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", charger_get_charge_current());
}

static ssize_t
store_charger_state(struct device *_dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int value;

	if ((sscanf(buf, "%d", &value) > 0) &&
			((value == 0) || (value == 1))) {
		/* Well, we could implement this, but setting the charge current manually seems
		 * like a bad thing to do
		 */
		return -ENOSYS;
	}

	return -EINVAL;
}
static DEVICE_ATTR (charger_state, S_IRUGO|S_IWUSR, show_charger_state, store_charger_state);

static ssize_t
store_chrgled_state(struct device *_dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int value;

	if ((sscanf(buf, "%d", &value) > 0) &&
		((value == 0) || (value == 1))) {
			return -ENOSYS;
	}

	return -EINVAL;
}
static DEVICE_ATTR (chrgled_state, S_IRUGO|S_IWUSR, NULL, store_chrgled_state);

static ssize_t
store_charger_lobath_unsub(struct device *_dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int value;

	if ((sscanf(buf, "%d", &value) > 0) &&
		((value == 0) || (value == 1))) {
			return -ENOSYS;
	}

	return -EINVAL;
}

static DEVICE_ATTR (charger_lobath_unsub, S_IRUGO|S_IWUSR, NULL, store_charger_lobath_unsub);

/*! Failure of enumeration should redo detach/attach event */
static void host_enumeration_detect_fn(struct work_struct *work)
{
	/* If not enumerated and connected to a HOST */
	if (!atomic_read(&enumerated) && is_charger_connected())
		fsl_udc_redo_enumerate();
}

static DECLARE_DELAYED_WORK(host_enumeration_detect, host_enumeration_detect_fn);

static void fsl_detect_charger_type(void)
{
	unsigned int portsc1;
	DBG("%s: enter\n", __func__);

	if (charger_conn_probe) {

		DBG("%s: charger conn\n", __func__);

		resume_ctrl();
		mdelay(10);
		charger_conn_probe = 0;
		if (is_wall_charger_attached()) {
			wall_charger_start_charging();
		}
		return;
	}

	portsc1 = fsl_readl(&dr_regs->portsc1);
	DBG("%s: prtsc1=0x%x", __func__, portsc1);

	if ((portsc1 & PORTSCX_LINE_STATUS_BITS) == PORTSCX_LINE_STATUS_UNDEF) {
		DBG("%s: line undef prtsc1=0x%x", __func__, portsc1);

		wall_charger_start_charging();
		goto out;
	}

	if ((portsc1 & PORTSCX_LINE_STATUS_BITS) == PORTSCX_LINE_STATUS_KSTATE ||
	    (portsc1 & PORTSCX_LINE_STATUS_BITS) == PORTSCX_LINE_STATUS_JSTATE) {
		DBG("Connected to a charger: 0x%x", portsc1);
		fsl_set_current_limit(udc_controller, CHARGING_HOST);
		
		kobject_uevent(&udc_controller->gadget.dev.parent->kobj, KOBJ_ADD);
		if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_connected) < 0)
			printk(KERN_ERR "Host: Error creating device file\n");

		fsl_udc_redo_enumerate();
		goto out;
	}

	if (portsc1 & PORTSCX_CURRENT_CONNECT_STATUS) {
		/* Connect Status */
		if (portsc1 & PORTSCX_PORT_HSP) {
			printk(KERN_INFO "arcotg_udc: I pmic:chargerUsbHigh::\n");
			udc_controller->gadget.speed = USB_SPEED_HIGH;
		}
		else {
			printk(KERN_INFO "arcotg_udc: I pmic:chargerUsbFull::\n");
		}

		if (!atomic_read(&enumerated))
			fsl_set_current_limit(udc_controller, CHARGING_HOST);

		kobject_uevent(&udc_controller->gadget.dev.parent->kobj, KOBJ_ADD);
		if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_connected) < 0)
			printk(KERN_ERR "Host: Error creating device file\n");

		schedule_delayed_work(&host_enumeration_detect, msecs_to_jiffies(HOST_ENU_DETECT));
		goto out;
	}

	if (host_detected) {
		/* A host as already been detected */
		DBG("Connected to a Host: %d", host_detected);
		fsl_set_current_limit(udc_controller, CHARGING_HOST);

		kobject_uevent(&udc_controller->gadget.dev.parent->kobj, KOBJ_ADD);
		if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_connected) < 0)
			printk(KERN_ERR "Host: Error creating device file\n");

		goto out;
	} else {
		pr_debug("Connected to a third-party charger: 0x%x\n", portsc1);
		fsl_set_current_limit(udc_controller, CHARGING_THIRD_PARTY);
		kobject_uevent(&udc_controller->gadget.dev.parent->kobj, KOBJ_ADD);
		if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_connected) < 0)
			printk(KERN_ERR "Charger: Error creating device file\n");
		goto out;
	}

	pr_debug("%s: not detected\n", __func__);

	not_detected = 1;
out:	
	udc_detection = 1;

	pr_debug("%s: cmpl\n", __func__);
}

static int fsl_set_current_limit(struct fsl_udc *udc, unsigned int mA) {
	/*
	 * Set VBus charge current limit. If OTG transceiver is not in use, pass
	 * the current limit directly to the Yoshi battery. Else we have to set the
	 * current limit indirectly via the otg transceiver.
	 */
	if (udc->transceiver) {
		return otg_set_power(udc->transceiver, mA);
	} else {
		return charger_set_current_limit(mA);
	}
}

/*----------------------------------------------------------------
 * Setup the fsl_ep struct for eps
 * Link fsl_ep->ep to gadget->ep_list
 * ep0out is not used so do nothing here
 * ep0in should be taken care
 *--------------------------------------------------------------*/
static int __init struct_ep_setup(struct fsl_udc *udc, unsigned char index,
		char *name, int link)
{
	struct fsl_ep *ep = &udc->eps[index];

	ep->udc = udc;
	strcpy(ep->name, name);
	ep->ep.name = ep->name;

	ep->ep.ops = &fsl_ep_ops;
	ep->stopped = 0;

	/* for ep0: maxP defined in desc
	 * for other eps, maxP is set by epautoconfig() called by gadget layer
	 */
	ep->ep.maxpacket = (unsigned short) ~0;

	/* the queue lists any req for this ep */
	INIT_LIST_HEAD(&ep->queue);

	/* gagdet.ep_list used for ep_autoconfig so no ep0 */
	if (link)
		list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
	ep->gadget = &udc->gadget;
	ep->qh = &udc->ep_qh[index];

	return 0;
}

/* Driver probe function
 * all intialization operations implemented here except enabling usb_intr reg
 * board setup should have been done in the platform code
 */
static int __init fsl_udc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;
	int ret = -ENODEV;
	unsigned int i;
	u32 dccparams;
#ifdef CONFIG_USB_EHCI_ARC_OTG
	unsigned int sense_0;
#endif

	if (strcmp(pdev->name, driver_name)) {
		VDBG("Wrong device\n");
		return -ENODEV;
	}

#ifdef CONFIG_USB_EHCI_ARC_OTG
	pmic_read_reg(REG_INT_SENSE0, &sense_0, 0xffffff);

	if (!(sense_0 & (1 << SENSE_IDFLOATS))) {
		printk(KERN_ERR "Looks like a non-charger connected: 0x%x\n", sense_0);
		return -ENODEV;
	}
#endif

	udc_controller = kzalloc(sizeof(struct fsl_udc), GFP_KERNEL);
	if (udc_controller == NULL) {
		ERR("malloc udc failed\n");
		return -ENOMEM;
	}
	udc_controller->pdata = pdata;

#ifdef CONFIG_USB_EHCI_ARC_OTG
	udc_controller->transceiver = otg_get_transceiver();
	if (!udc_controller->transceiver) {
		printk(KERN_ERR "Can't find OTG driver! (try modprobe \"fsl_otg_arc\" first)\n");
		kfree(udc_controller);
		return -ENODEV;
	}
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENXIO;
		goto err1a;
	}

	if (!udc_controller->transceiver) {
		if (!request_mem_region(res->start, resource_size(res),
				driver_name)) {
			ERR("request mem region for %s failed \n", pdev->name);
			ret = -EBUSY;
			goto err1a;
		}
	}

	dr_regs = ioremap(res->start, resource_size(res));
	if (!dr_regs) {
		ret = -ENOMEM;
		goto err1;
	}
	pdata->regs = (void *)dr_regs;
	/*
	 * do platform specific init: check the clock, grab/config pins, etc.
	 */
	if (pdata->platform_init && pdata->platform_init(pdev)) {
		ret = -ENODEV;
		goto err2a;
	}

	if (pdata->have_sysif_regs)
		usb_sys_regs = (struct usb_sys_interface *)
				((u32)dr_regs + USB_DR_SYS_OFFSET);

	/* Read Device Controller Capability Parameters register */
	dccparams = fsl_readl(&dr_regs->dccparams);
	if (!(dccparams & DCCPARAMS_DC)) {
		ERR("This SOC doesn't support device role\n");
		ret = -ENODEV;
		goto err2;
	}
	/* Get max device endpoints */
	/* DEN is bidirectional ep number, max_ep doubles the number */
	udc_controller->max_ep = (dccparams & DCCPARAMS_DEN_MASK) * 2;

	udc_controller->irq = platform_get_irq(pdev, 0);
	printk(KERN_INFO "IRQ is %u\n", udc_controller->irq);

	if (!udc_controller->irq) {
		ret = -ENODEV;
		goto err2;
	}

	ret = request_irq(udc_controller->irq, fsl_udc_irq, IRQF_DISABLED | IRQF_SHARED,
			driver_name, udc_controller);
	if (ret != 0) {
		ERR("cannot request irq %d err %d \n",
				udc_controller->irq, ret);
		goto err2;
	}

	/* Initialize the udc structure including QH member and other member */
	if (struct_udc_setup(udc_controller, pdev)) {
		ERR("Can't initialize udc data structure\n");
		ret = -ENOMEM;
		goto err3;
	}

	INIT_DELAYED_WORK(&udc_controller->charger_detect_work, do_charger_detect_work);

	charger_conn_probe = is_charger_connected();
	charging_wall_mA = CHARGING_THIRD_PARTY;

	if (!udc_controller->transceiver) {
		/* initialize usb hw reg except for regs for EP,
		 * leave usbintr reg untouched */
		printk(KERN_INFO "No transceiver.\n");
		dr_controller_setup(udc_controller);

		/* Only do charger detection if there is no OTG transceiver.
		 * If there is, it might be that the host is currently using the port.
		 */
		schedule_delayed_work(&udc_controller->charger_detect_work,
						msecs_to_jiffies(CHARGER_STARTUP));
	}

	/* Setup gadget structure */
	udc_controller->gadget.ops = &fsl_gadget_ops;
	udc_controller->gadget.is_dualspeed = 1;
	udc_controller->gadget.ep0 = &udc_controller->eps[0].ep;
	INIT_LIST_HEAD(&udc_controller->gadget.ep_list);
	udc_controller->gadget.speed = USB_SPEED_UNKNOWN;
	udc_controller->gadget.name = driver_name;

	/* Setup gadget.dev and register with kernel */
	dev_set_name(&udc_controller->gadget.dev, "gadget");
	udc_controller->gadget.dev.release = fsl_udc_release;
	udc_controller->gadget.dev.parent = &pdev->dev;
	ret = device_register(&udc_controller->gadget.dev);
	if (ret < 0)
		goto err3;

	if (udc_controller->transceiver)
		udc_controller->gadget.is_otg = 1;

	/* setup QH and epctrl for ep0 */
	ep0_setup(udc_controller);

	/* setup udc->eps[] for ep0 */
	struct_ep_setup(udc_controller, 0, "ep0", 0);
	/* for ep0: the desc defined here;
	 * for other eps, gadget layer called ep_enable with defined desc
	 */
	udc_controller->eps[0].desc = &fsl_ep0_desc;
	udc_controller->eps[0].ep.maxpacket = USB_MAX_CTRL_PAYLOAD;

	/* setup the udc->eps[] for non-control endpoints and link
	 * to gadget.ep_list */
	for (i = 1; i < (int)(udc_controller->max_ep / 2); i++) {
		char name[14];

		sprintf(name, "ep%dout", i);
		struct_ep_setup(udc_controller, i * 2, name, 1);
		udc_controller->eps[i*2].desc = &fsl_ep1_desc;
		sprintf(name, "ep%din", i);
		struct_ep_setup(udc_controller, i * 2 + 1, name, 1);
		udc_controller->eps[i*2+1].desc=&fsl_ep1_desc;
	}

	/* use dma_pool for TD management */
	udc_controller->td_pool = dma_pool_create("udc_td", &pdev->dev,
			sizeof(struct ep_td_struct),
			DTD_ALIGNMENT, UDC_DMA_BOUNDARY);
	if (udc_controller->td_pool == NULL) {
		ret = -ENOMEM;
		goto err4;
	}

	mutex_init(&udc_controller->suspend_lock);

	if (udc_controller->transceiver) {
		/* suspend the PHY until the gadget driver is loaded */
		if ((ret = low_power_enter(udc_controller))) {
			printk(KERN_ERR "%s: ERROR Could not enter low power idle. Exiting.\n", __func__);
			goto err4;
		}
	}

	charger_event_subscribe(CHARGER_LOBAT_EVENT, &callback_lobat_event,
				udc_controller);

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_charging) < 0)
		printk(KERN_ERR "Charger: Error creating device charging file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_lobat_condition) < 0)
		printk(KERN_ERR "Charger: Error creating device lobat_condition file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_supply_voltage) < 0)
		printk(KERN_ERR "Charger: Error creating supply voltage file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_chrgraw_voltage) < 0)
		printk(KERN_ERR "Charger: Error creating chrgraw voltage file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_ichrg_setting) < 0)
		printk(KERN_ERR "Charger: Error creating ichrg_setting file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_voltage) < 0)
		printk(KERN_ERR "Charger: Error creating battery voltage file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_battery_current) < 0)
		printk(KERN_ERR "Charger: Error creating battery current file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_batt_error) < 0)
		printk(KERN_ERR "Charger: Error creating batt_error file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_charger_state) < 0)
		printk(KERN_ERR "Charger: Error creating charger_state file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_chrgled_state) < 0)
		printk(KERN_ERR "Charger: Error creating chrgled_state file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_battery_id) < 0)
		printk(KERN_ERR "Charger: Error creating battery_id file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_third_party) < 0)
		printk(KERN_ERR "Charger: Error creating third_party_charger file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_bpsns) < 0)
		printk(KERN_ERR "Charger: Error creating bpsns file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_charger_lobath_unsub) < 0)
		printk(KERN_ERR "Charger: Error creating charger_lobath_unsub file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_accessory_charger) < 0)
		printk(KERN_ERR "Charger: Error creating accessory_charger file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_third_party_timer) < 0)
		printk(KERN_ERR "Charger: Error creating third party timer file\n");

	if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_usb_lpm_timer_threshold) < 0)
		printk(KERN_ERR "Charger: Error creating usb lpm timer file\n");

	INIT_WORK(&udc_controller->usbtest_work, usbtest_workqueue_handler);

	fsl_udc_init_debugfs(udc_controller);

	printk(KERN_INFO "kernel: I perf:usb:usb_gadget_loaded=");
	mxc_kernel_uptime(); 

	return 0;

err4:
	device_unregister(&udc_controller->gadget.dev);
err3:
	free_irq(udc_controller->irq, udc_controller);
err2:
	if (pdata->platform_uninit)
		pdata->platform_uninit(pdata);
err2a:
	iounmap((u8 __iomem *)dr_regs);
err1:
	if (!udc_controller->transceiver)
		release_mem_region(res->start, resource_size(res));
err1a:
	kfree(udc_controller);
	udc_controller = NULL;
	return ret;
}

/* Driver removal function
 * Free resources and finish pending transactions
 */
static int __exit fsl_udc_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct fsl_usb2_platform_data *pdata = pdev->dev.platform_data;

	DECLARE_COMPLETION(done);

	if (!udc_controller)
		return -ENODEV;

	udc_suspended_mode = 1;
	cancel_rearming_delayed_work(&udc_controller->charger_detect_work);
	cancel_rearming_delayed_work(&pc_wq);
	cancel_rearming_delayed_work(&host_enumeration_detect);
	cancel_rearming_delayed_work(&phy_resume_work);

	udc_controller->done = &done;

	/* DR has been stopped in usb_gadget_unregister_driver() */

	fsl_udc_cleanup_debugfs(udc_controller);

	/* Free allocated memory */
	kfree(udc_controller->status_req->req.buf);
	kfree(udc_controller->status_req);
	kfree(udc_controller->eps);

	dma_pool_destroy(udc_controller->td_pool);

	iounmap((u8 __iomem *)dr_regs);

	kobject_uevent(&udc_controller->gadget.dev.parent->kobj, KOBJ_REMOVE);

	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_connected);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_charging);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_lobat_condition);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_supply_voltage);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_chrgraw_voltage);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_ichrg_setting);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_voltage);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_battery_current);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_batt_error);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_charger_state);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_chrgled_state);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_battery_id);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_third_party);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_bpsns);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_charger_lobath_unsub);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_accessory_charger);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_third_party_timer);
	device_remove_file(udc_controller->gadget.dev.parent, &dev_attr_usb_lpm_timer_threshold);

	if (charger_event_unsubscribe(CHARGER_LOBAT_EVENT, &callback_lobat_event)) {
		printk(KERN_ERR "%s: Could not unsubscribe from lobat event\n", __func__);
	}

	if (!udc_controller->transceiver) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		release_mem_region(res->start, resource_size(res));
	}

	device_unregister(&udc_controller->gadget.dev);
	/* free udc --wait for the release() finished */
	wait_for_completion(&done);

	free_irq(udc_controller->irq, udc_controller);

	/*
	 * do platform specific un-initialization:
	 * release iomux pins, etc.
	 */
	low_power_enter(udc_controller);
	usbotg_uninit_port(pdata);

	return 0;
}

static int udc_suspend(struct fsl_udc *udc)
{
	udc->stopped = 1;
	return 0;
}


static int low_power_enter(struct fsl_udc *udc)
{
	int ret;

	mutex_lock(&udc->suspend_lock);

	if (udc->lpm) {
		mutex_unlock(&udc->suspend_lock);

		return 0;
	}

	printk(KERN_INFO "%s: Going into low power idle\n", __func__);

	dr_controller_stop(udc);

	host_detected = 0;

	if (udc->driver && udc->driver->disconnect)
		udc->driver->disconnect(&udc->gadget);

	device_remove_file(udc->gadget.dev.parent, &dev_attr_connected);
	kobject_uevent_atomic(&udc->gadget.dev.parent->kobj, KOBJ_REMOVE);

	/* Clear out the variables */
	atomic_set(&enumerated, 0);
	charger_conn_probe = 0;
	charger_enumerated = 0;

	if ((ret = usbotg_low_power_enter())) {
		printk(KERN_ERR "%s: ERROR failed during usbotg_low_power_enter(). retval=%d\n",
				__func__, ret);

		return ret;
	}

	udc->lpm = 1;
	mutex_unlock(&udc->suspend_lock);

	return 0;
}

static int low_power_exit(struct fsl_udc *udc)
{
	int ret;

	mutex_lock(&udc->suspend_lock);

	if (!udc->lpm) {
		mutex_unlock(&udc->suspend_lock);

		return 0;
	}

	printk(KERN_INFO "%s: Resuming PHY\n", __func__);


	charger_conn_probe = 0;
	not_detected = 0;

	ret = usbotg_low_power_exit();
	if (ret) {
		printk(KERN_INFO "%s: Could not exit low power idle, retval=%d\n", __func__, ret);
		return ret;
	}

	/* Enable DR irq reg and set controller Run */
	dr_controller_setup(udc);
	dr_controller_run(udc);

	udc->usb_state = USB_STATE_ATTACHED;
	udc->ep0_state = WAIT_FOR_SETUP;
	udc->ep0_dir = 0;

	udc->lpm = 0;
	mutex_unlock(&udc->suspend_lock);

	if (is_charger_connected()) {
		fsl_detect_charger_type();
	}

	return 0;
}

/*-----------------------------------------------------------------
 * Modify Power management attributes
 * Used by OTG statemachine to disable gadget temporarily
 -----------------------------------------------------------------*/
static int fsl_udc_suspend(struct platform_device *dev, pm_message_t state)
{
	return low_power_enter(udc_controller);
}

/*-----------------------------------------------------------------
 * Invoked on USB resume. May be called in_interrupt.
 * Here we start the DR controller and enable the irq
 *-----------------------------------------------------------------*/
static int fsl_udc_resume(struct platform_device *dev)
{
	return low_power_exit(udc_controller);
}

static int arcotg_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO: Return -EBUSY for low battery conditions */

	udc_suspended_mode = 1;

	udc_stopped = 1;
	return 0;
}

/*! Phy low power on resumw */
static void phy_low_power_resume(struct work_struct *work)
{
	if (!is_charger_connected()) {
		udc_stopped = 0;
		pr_debug("arcotg_udc: Putting Phy in low power mode\n");
	}
}

static int arcotg_resume(struct platform_device *pdev)
{
	udc_suspended_mode = 0;
	schedule_delayed_work(&phy_resume_work, msecs_to_jiffies(PHY_LPM_TIMER));
	return 0;
}

/*-------------------------------------------------------------------------
	Register entry point for the peripheral controller driver
--------------------------------------------------------------------------*/

static struct platform_driver udc_driver = {
	.remove  = __exit_p(fsl_udc_remove),
	/* these suspend and resume are not usb suspend and resume */
	.suspend = fsl_udc_suspend,
	.resume  = fsl_udc_resume,
	.probe = fsl_udc_probe,
	.driver  = {
		.name = driver_name,
		.owner = THIS_MODULE,
	},
};

static int __init udc_init(void)
{
	pr_debug("%s (%s)\n", driver_desc, DRIVER_VERSION);
	return platform_driver_register(&udc_driver);
}

module_init(udc_init);

static void __exit udc_exit(void)
{
	platform_driver_unregister(&udc_driver);
	pr_debug("%s unregistered \n", driver_desc);
}

module_exit(udc_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
