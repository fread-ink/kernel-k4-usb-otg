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
#include <linux/pmic_external.h>
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


/*
 *	The spec sheet is not clear about these two bits. The name and description shifted for a row. 
 *	See P143 mc13892 user guild.
 * */
#define CHGFAULTM 0x000300


static const char driver_name[] = "fsl-usb2-udc";
static const char driver_desc[] = DRIVER_DESC;

volatile static struct usb_dr_device *dr_regs;
volatile static struct usb_sys_interface *usb_sys_regs;

/* it is initialized in probe()  */
static struct fsl_udc *udc_controller;

extern int yoshi_voltage;
extern int yoshi_battery_current;
extern int yoshi_battery_id;
extern int yoshi_battery_capacity;
extern int yoshi_reduce_charging;

extern void accessory_charger_disable(void);
extern void accessory_charger_enable(void);
extern int gpio_accessory_charger_detected(void);

/* SPI Suspended? */
extern int mxc_spi_suspended;

/* Battery present and valid? */
extern int yoshi_battery_valid(void);

extern void gpio_chrgled_active(int enable); 
extern int pmic_check_factory_mode(void);

extern void mxc_kernel_uptime(void);

static struct ep_td_struct *last_free_td;

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
static int fsl_udc_suspend(struct device *dev, pm_message_t state);
static int fsl_udc_resume(struct device *dev);
static void fsl_ep_fifo_flush(struct usb_ep *_ep);
static int green_led_set = 0;
static int udc_detection = 0;
static int saved_ichrg_value = 0;
static int udc_stopped = 0;
static int lobat_condition = 0;

static int charger_timer_expiry_threshold = 0;
static int charger_timer_expiry_counter = 0;
static int fsl_vbus_done = 0;

static int crit_boot_flag = 0;
static int charging_wall_mA = 0;

/* Do not suspend the PHY */
static int udc_phy_suspend = 0;

/* port change work */
static void port_change_work(struct work_struct *work);
DECLARE_DELAYED_WORK(pc_wq, port_change_work);

static void resume_ctrl(void);
static void suspend_ctrl(void);
static void fsl_udc_redo_enumerate(void);

#undef ARC_OTG

#ifdef ARC_OTG
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

/* 
 * Charger
 */
static pmic_event_callback_t charger_detect;
static pmic_event_callback_t lobatli_event;
static pmic_event_callback_t lobathi_event;
static pmic_event_callback_t bpon_event;
static pmic_event_callback_t chgfaulti_event;
static pmic_event_callback_t chrgse1b_event;
static pmic_event_callback_t bvalid_detect;

static int pmic_set_chg_current(unsigned short curr);
static void pmic_enable_green_led(int enable);

static struct clk *usb_ahb_clk;
static struct clk *usb_phy1_clk;
static void fsl_idle_low_power_exit(struct fsl_udc *udc);
static void fsl_idle_low_power_enter(struct fsl_udc *udc);
static void fsl_detect_charger_type(void);
static int not_detected = 0;
static int host_detected = 0; 		/* Has a host been detected? */
static void pmic_start_charging(int value);

static void phy_low_power_resume(struct work_struct *work);
static DECLARE_DELAYED_WORK(phy_resume_work, phy_low_power_resume);

static int udc_suspended_mode = 0; /* gadget driver suspended state */

static int third_party_detect_timer = 0;
static int usb_lpm_timer = 0;

#define VBUSVALIDS		0x08	/* Sense bit for valid VBUS */
#define CHGDETS  		0x40	/* Sense bit for CHGDET */
#define BVALIDS			0x10000	/* Sense bit for BVALID */
#define CHGCURRS		0x800	/* Bit #11 */
#define CHGCURRS_THRESHOLD	4100	/* CHGCURRS Threshold */
#define PC_WORK_THRESHOLD	30000	/* Port change threshold */
#define ARCOTG_IRQ		37
#define CHARGER_STARTUP 	4000	/* Delayed workqueue that runs on startup */
#define CHARGER_MISC		10000	/* Charger misc timer */
#define CHARGER_TYPE		1000	/* Charger type detection after 1 second */
#define CHARGER_TYPE_SUSPEND	5000	/* Charger detection after a system resume */
#define CHARGER_TYPE_LPM	4000	/* Transition PHY into LPM after 10 seconds */
#define PHY_LPM_TIMER		20000	/* 20 seconds after resume */
#define THIRD_PARTY_DETECT	30000	/* Third Party charger detect */
#define HOST_ENU_DETECT 	100	/* Host enumeration check */
#define BATT_VOLTAGE_SCALE	4687	/* Scale the values from the ADC based on manual */
#define BATT_CURRENT_SCALE	5865	/* Scale the current read from the ADC */
#define CHARGE_VOLTAGE_SCALE	11700	/* Scale the charger voltage read from A/D */
#define CHARGER_FULL		0x0	/* off - 0mA */
#define CHARGING_HOST		0x1	/* Host charging with PHY active */
#define CHARGING_WALL		0x5	/* Wall charging - */
#define CHARGING_THIRD_PARTY	0x5	/* Third party chargers */
#define BPSNS_VALUE		0x2	/* For LOBATHI and LOBATLI */
#define BPSNS_VALUE_SUSPEND	0x3	/* Suspend - LOBATHI and LOBATLI */
#define BPSNS_MASK		0x3	/* BPSNS Mask in PMIC reg 13 */
#define BPSNS_VOLTAGE_THRESH	3600	/* 3.6V BPSNS threshold */
#define PMIC_REG_PC_0		13	/* PMIC Register 13 */
#define BPSNS_OFFSET		16	/* BPSNS OFFSET in Register 13 */
#define GREEN_LED_THRESHOLD	95	/* Turn Charge LED to Green */
#define BATTCYCL_THRESHOLD	4105	/* Restart charging if not done automatically */
#define LOW_TEMP_CHARGING	0x2	/* 240mA charging if temp < 10C */
#define CHRGLEDEN		18 	/* offset 18 in REG_CHARGE */
#define CHRGCYCLB		22	/* BATTCYCL to restart charging at 95% full */
#define CHRGRESTART		20	/* Restart charging */
#define BPON_THRESHOLD		3800
#define BPON_THRESHOLD_HI	3810
#define CHARGING_BPON		0x3	/* 320mA charging */
#define CHARGER_TURN_ON		3400
#define CHRGRAW_THRESHOLD	6000
#define CRAW_MON_THREHOLD	6 	/* Every 60 seconds or so, monitor the chrgraw */
#define IDGND_SENSE		(1 << 20)
#define CHRG_SUS_LOW_VOLTAGE	3420	/* 3.42V */
#define CHRG_TIMER_THRESHOLD		2
#define CHRG_TIMER_THRESHOLD_TEQ	1

#define BIT_CHG_CURR_LSH	3
#define BIT_CHG_CURR_WID	4

#define BIT_CHG_CURRS_LSH 	11
#define BIT_CHG_CURRS_WID	1

#define BAT_TH_CHECK_DIS_LSH	9
#define BAT_TH_CHECK_DIS_WID	1

#define RESTART_CHG_STAT_LSH	20
#define RESTART_CHG_STAT_WID	1

#define AUTO_CHG_DIS_LSH	21
#define AUTO_CHG_DIS_WID	1

#define VI_PROGRAM_EN_LSH	23
#define VI_PROGRAM_EN_WID	1

/* Battery current amplification from A/D */
#define BAT_CURRENT_UNIT_UA	5870

#define CHARGER_BOOT_THRESHOLD	4

#define FULL_BATTERY_CAPACITY	100
#define FULL_BATTERY_CURR_LIMIT	20

enum chg_setting {
	BAT_TH_CHECK_DIS,
	RESTART_CHG_STAT,
	AUTO_CHG_DIS,
	VI_PROGRAM_EN
};

static int pmic_set_chg_misc(enum chg_setting type, unsigned short flag);

#define LOW_VOLTAGE_THRESHOLD		3400	/* 3.4V */
#define CRITICAL_VOLTAGE_THRESHOLD	3100	/* 3.1V */
#define PMIC_IRQ			108	/* PMIC IRQ */

static int ichrg_value = 0;
static int charger_bpon = 0;
static int full_battery = 0;

static atomic_t enumerated = ATOMIC_INIT(0);
static int wall_charger = 0;
static int third_party_charger = 0;	/* Is it a non-Amazon charger? */
static int charger_enumerated = 0;
static int charger_conn_probe = 0;	/* Is charger connected on probe? */
static int chrgraw_disable = 0;		/* Is charging disabled due to chrgraw? */
static int chrgraw_monitor = 0; 	/* Once every 60 seconds */

static int bpsns_value = BPSNS_VALUE;	/* BPSNS = 0x3 at init */

/* Battery Error Flags */
extern int yoshi_battery_error_flags;

static int read_supply_voltage(void);
static int read_chrgraw_voltage(void);

static void fsl_charger_type_detect(unsigned long data);

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



extern void pmic_vusb2_enable_vote(int enable,int source);
/* Turn VUSB2 on/off */
static void pmic_vusb2_enable(int enable)
{
#define VUSB2_VOTE_SOURCE_OTG  (1U << 1)
	pmic_vusb2_enable_vote(enable,VUSB2_VOTE_SOURCE_OTG);
}

static int read_charger_voltage(void);

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

static int bvalid_connected(void)
{
	int sense_0 = 0;
	int ret = 0; /* Default: no charger */

	pmic_read_reg(REG_INT_SENSE0, &sense_0, 0xffffff);
	if (sense_0 & BVALIDS)
		ret = 1;

	return ret;
}

/*
 * Is a USB charger connected?
 */
static int charger_connected(void)
{
	int sense_0 = 0;
	int ret = 0; /* Default: no charger */
	
	pmic_read_reg(REG_INT_SENSE0, &sense_0, 0xffffff);
	if (sense_0 & CHGDETS)
		ret = 1;

	pr_debug("%s: %d\n", __func__, ret);

	return ret;
}

static void charger_set_config(void)
{
	printk(KERN_INFO "arcotg_udc: I pmic:chargerWall:mV=%d:\n", read_supply_voltage());

	pmic_start_charging(charging_wall_mA);
	
	wall_charger = 1;
	kobject_uevent(&udc_controller->gadget.dev.parent->kobj, KOBJ_ADD);
	if (device_create_file(udc_controller->gadget.dev.parent,
			       &dev_attr_connected) < 0) {
		pr_debug("Charger: Error creating device file\n");
	}
}

static int check_portsc_bits(void)
{
        unsigned int portsc1;

	resume_ctrl();
	mdelay(10);

	portsc1 = fsl_readl(&dr_regs->portsc1);

	if ((portsc1 & PORTSCX_LINE_STATUS_BITS) == PORTSCX_LINE_STATUS_UNDEF) {
		charger_set_config();
		return 1;
        }

	return 0;
}

/*
 * Kicked off only on startup to figure out whether a charger/host is
 * connected or to enter low power idle mode.
 */
static void do_charger_detect_work(struct work_struct *work)
{
	struct fsl_udc *udc = 
		container_of(work, struct fsl_udc, charger_detect_work.work);
	int charger_conn = charger_connected();

	pr_debug("%s: begin\n", __func__);

	if (charger_conn && yoshi_battery_valid()) {
	    if (ichrg_value != 0) {
		pr_debug("%s: detected chg=%d\n", __func__, ichrg_value);

		/* Charger already detected */
		atomic_set(&charger_atomic_detection, 1);

	    } else if (!not_detected) {
		pr_debug("%s: not detected prb=%d\n", __func__, charger_conn_probe);

		atomic_set(&charger_atomic_detection, 1);
		if (charger_conn_probe == 1) {
		    charger_conn_probe = 0;
		    if (!check_portsc_bits()) {
			fsl_detect_charger_type();
		    }
		} else {
		    fsl_detect_charger_type();
		}
	    } 
	} else {
	    pr_debug("No USB/Charger found ... entering low power idle\n");
	    pmic_enable_green_led(0);
	    fsl_idle_low_power_enter(udc);
	}
}

static void pmic_green_led_disable(int flag)
{
	pmic_write_reg(REG_CHARGE, (flag << CHRGLEDEN), (1 << CHRGLEDEN));
}

static void pmic_enable_bpon_charging(void)
{
	/* Don't enable charging if there is no battery */
	if (!yoshi_battery_valid()) {
	    pr_debug("%s: no battery, do not enable charging\n", __func__);
	    return;
	}

	/* Keep auto charging disabled */
	pmic_set_chg_misc(AUTO_CHG_DIS, 1);

        /* Turn on CHRGLED */
	pmic_write_reg(REG_CHARGE, (1 << 18), (1 << 18));

	/* Yellow LED GPIO */
	gpio_chrgled_active(1);

	/* Turn on V & I programming */
	pmic_write_reg(REG_CHARGE, (1 << 23), (1 << 23));

	/* Turn off TRICKLE CHARGE */
	pmic_write_reg(REG_CHARGE, (0 << 7), (1 << 7));

	if (host_detected == 1) {
		if (!atomic_read(&enumerated)) {
			pmic_set_chg_current(CHARGING_HOST);
			ichrg_value = CHARGING_HOST;
		}
		else {
			pmic_set_chg_current(charger_enumerated);
			ichrg_value = charger_enumerated;
		}
	}
	else {
		if (!wall_charger) {
			pmic_set_chg_current(CHARGING_THIRD_PARTY);
			ichrg_value = CHARGING_THIRD_PARTY;
		}
		else {
			pmic_set_chg_current(charging_wall_mA);
			ichrg_value = charging_wall_mA;
		}
	}

	/* PLIM: bits 15 and 16 */
	pmic_write_reg(REG_CHARGE, (0x2 << 15), (0x3 << 15));

	/* PLIMDIS: bit 17 */
	pmic_write_reg(REG_CHARGE, (0 << 17), (1 << 17));

	/* Restart charging */
	pmic_write_reg(REG_CHARGE, (1 << 20), (1 << 20));
}

static void pmic_restart_charging(void)
{
    if (yoshi_battery_valid()) {
	/* Restart the charging process */
	pmic_write_reg(REG_CHARGE, (1 << CHRGRESTART), (1 << CHRGRESTART));
    }
}

/*!
 * Post a low battery or a critical battery event to the userspace 
 */
static void post_lobat_event(int crit_level)
{
	if (!yoshi_battery_valid())
		return;

	if (!crit_level) {
		char *envp[] = { "BATTERY=low", NULL };
		printk(KERN_INFO "arcotg_udc: I pmic:lowbatEvent::\n");

		/* Relay the lobath event to userspace */
		kobject_uevent_env(&udc_controller->gadget.dev.parent->kobj, KOBJ_CHANGE, envp);
	}
	else {
		char *envp[] = { "BATTERY=critical", NULL };
		printk(KERN_ERR "arcotg_udc: I pmic:critbatEvent::\n");

		/* Relay the lobatl event to userspace */
		kobject_uevent_env(&udc_controller->gadget.dev.parent->kobj, KOBJ_CHANGE, envp);
	}

	/* Update the /sys entry */
	lobat_condition = 1;
}

static void do_charger_misc_work(struct work_struct *work);
DECLARE_DELAYED_WORK(charger_misc_work, do_charger_misc_work);

static int reduce_charging = 0;
static int battery_error = 0;


/*!
 * This function checks the charger over-voltage condition
 */
static void fsl_check_overvoltage(int charger_conn)
{
	unsigned int chrgraw = 0;

	/* Clear out the chrgraw monitor */
	chrgraw_monitor = 0;
	chrgraw = read_charger_voltage();

	if (!chrgraw_disable) {
		if (chrgraw >= CHRGRAW_THRESHOLD) {
			pr_debug("overvoltage\n");
			pmic_set_chg_current(0);
			saved_ichrg_value = ichrg_value;
			ichrg_value = 0; /* Userspace can suspend */
			chrgraw_disable = 1;
		}
	}
	else {
		/* Check charger voltage and restart charging */
		if (chrgraw < CHRGRAW_THRESHOLD) {
			if (saved_ichrg_value) {
				ichrg_value = saved_ichrg_value;

				/* Charging was disabled earlier due to chrgraw > 6V */
				pmic_set_chg_current(saved_ichrg_value);
				pmic_restart_charging();
			}
			else
				fsl_detect_charger_type();

			chrgraw_disable = 0;
		}
		else {
			/* Charger disable from callback with an ichrg value */
			if (ichrg_value) {
				pmic_set_chg_current(0);
				saved_ichrg_value = ichrg_value;
				ichrg_value = 0; /* Userspace can suspend */
				chrgraw_disable = 1;
			}
		}
	}
}

static int charger_check_full_battery(void)
{
	if ( (yoshi_battery_capacity == FULL_BATTERY_CAPACITY) &&
		(yoshi_battery_current <= FULL_BATTERY_CURR_LIMIT) )
			return 1;
	
	return 0;
}

/*
 * Do all the miscellaneous activities like lobat checking and led setting
 */
static void do_charger_misc_work(struct work_struct *work)
{
	int batt_voltage = read_supply_voltage();
	int charger_conn = charger_connected();
	int sense_0 = 0;

	pr_debug("%s: begin\n", __func__);

	if (udc_suspended_mode) {
	    pr_debug("%s: suspended\n", __func__);
	    return;
	}

	chrgraw_monitor++;

	if (!yoshi_battery_valid()) {

	    pr_debug("%s: no battery\n", __func__);
	    
	    if (charger_conn && ichrg_value) {
		pmic_set_chg_current(0);
		saved_ichrg_value = 0;
		ichrg_value = 0;
	    }
	    
	    /* Disable all LEDs */
	    pmic_enable_green_led(0);
	    pmic_green_led_disable(0);
	    gpio_chrgled_active(0);
	    
	    return;
	}

	if (!charger_conn && (batt_voltage <= CRITICAL_VOLTAGE_THRESHOLD)) {
	    
		pr_debug("%s: critical battery\n", __func__);

		post_lobat_event(1);
		goto done;
	}

	if (!charger_conn && (batt_voltage <= LOW_VOLTAGE_THRESHOLD)) {

		pr_debug("%s: low battery\n", __func__);

		post_lobat_event(0);
		goto done;
	}

	if (crit_boot_flag  && (yoshi_battery_capacity > CHARGER_BOOT_THRESHOLD)) {
		crit_boot_flag = 0;
		pmic_write_reg(REG_MEM_A, (0 << 4), (1 << 4));
	}

	if (yoshi_battery_error_flags) {

		pr_debug("%s: battery error\n", __func__);

		pmic_set_chg_current(0);
		saved_ichrg_value = ichrg_value;
		ichrg_value = 0;
		battery_error = 1;

		/* Disable all LEDs */
		pmic_enable_green_led(0);
		pmic_green_led_disable(0);
		gpio_chrgled_active(0);
		goto done;
	}

	if (battery_error && !yoshi_battery_error_flags && charger_conn) {
		ichrg_value = saved_ichrg_value;
		battery_error = 0;
		pmic_set_chg_current(ichrg_value);
		pmic_green_led_disable(1);
		gpio_chrgled_active(1);
	}

	if ( (chrgraw_monitor == CRAW_MON_THREHOLD) && charger_conn) {
		pr_debug("%s: chrgraw monitor\n", __func__);
	    
		fsl_check_overvoltage(charger_conn);
	}

	if (chrgraw_disable) {
		pr_debug("%s: chrgraw disabled\n", __func__);

		goto done;
	}

	if (charger_conn && fsl_vbus_done) {
		pr_debug("%s: vbus done\n", __func__);
		fsl_vbus_done = 0;
		pmic_set_chg_current(ichrg_value);
	}

	/* TEQ-7477 - don't reenable charging if the timer expired */
	if (charger_conn &&
	    (charger_timer_expiry_counter > charger_timer_expiry_threshold)) {
		pr_debug ("%s: charge timer expired! do not resume charging\n", __func__);
		goto done;

	}

	if (charger_conn && (batt_voltage <= CHARGER_TURN_ON)) {
		pr_debug("%s: restart charging (voltage)\n", __func__);
		pmic_restart_charging();
	}

	/*
	 * Keep BPSNS to the lowest value "00" for higher charge
	 * current. However, if charger is not connected, then 
	 * we need BPSNS to be "11" for LOBATL and LOBATH
	 */
	if (charger_conn && (batt_voltage > BPSNS_VOLTAGE_THRESH)) {
		/* Try to charge at higher current */
		if (bpsns_value != 0) {
			bpsns_value = 0;
			pmic_write_reg(PMIC_REG_PC_0, bpsns_value << BPSNS_OFFSET,
					BPSNS_MASK << BPSNS_OFFSET);
		}
	}
	else {
		if (bpsns_value != BPSNS_VALUE) {
			bpsns_value = BPSNS_VALUE;
			pmic_write_reg(PMIC_REG_PC_0, bpsns_value << BPSNS_OFFSET,
					BPSNS_MASK << BPSNS_OFFSET);
		}
	}

	if (charger_conn && (batt_voltage <= BPON_THRESHOLD)) {
		if (!charger_bpon) {
			printk(KERN_INFO "arcotg_udc: I pmic:bponLobat:mV=%d:\n", batt_voltage);
			/* BPON charging */
			pmic_enable_bpon_charging();
			charger_bpon = 1;
		}
		goto done;
	}

	if (charger_bpon && (batt_voltage > BPON_THRESHOLD_HI)) {
		printk(KERN_INFO "arcotg_udc: I pmic:bponHi:mV=%d\n", batt_voltage);
		if (host_detected == 0) {
			if (!wall_charger)
				pmic_start_charging(CHARGING_THIRD_PARTY);
			else
				pmic_start_charging(charging_wall_mA);
                }
		else {
			if (!atomic_read(&enumerated))
				pmic_start_charging(CHARGING_HOST);
			else
				pmic_start_charging(charger_enumerated);
		}

		charger_bpon = 0;
	}

	if (charger_conn) {
		if (yoshi_battery_capacity > GREEN_LED_THRESHOLD) {
			if (!green_led_set) {
				green_led_set = 1;

				/* set the green led parameters */
				pmic_enable_green_led(1);

				/* set the chrgleden to 0 */
				pmic_green_led_disable(0);

				/* turn off the charger led */
				gpio_chrgled_active(0);
			}
		}
		else {
			green_led_set = 0;

			/* turn off the green led */
			pmic_enable_green_led(0);

			/* now enable the chrgleden to turn on the yellow LED */
			pmic_green_led_disable(1);

			/* turn back the charger led */
			gpio_chrgled_active(1);
		}
	}

	/* Check for full battery conditions */
	if (!full_battery && charger_conn) {
		pmic_read_reg(REG_INT_SENSE0, &sense_0, 0xffffff);
		if ( (!(sense_0 & CHGCURRS) && (batt_voltage > CHGCURRS_THRESHOLD) ) ||
			charger_check_full_battery() ) {
			full_battery = 1;
			printk(KERN_INFO "arcotg_udc: I pmic:batteryFull::\n");
			saved_ichrg_value = ichrg_value;
			pmic_set_chg_current(CHARGER_FULL);
		}
	}

	if (full_battery && charger_conn) {
		if (batt_voltage < BATTCYCL_THRESHOLD) {
			pmic_set_chg_current(saved_ichrg_value);
			pmic_restart_charging();
			full_battery = 0;
		}
	}

	if (yoshi_reduce_charging && charger_conn) {
		reduce_charging = 1;
		saved_ichrg_value = ichrg_value;
		pmic_set_chg_current(LOW_TEMP_CHARGING);
		ichrg_value = LOW_TEMP_CHARGING;
	}

	if (reduce_charging && !yoshi_reduce_charging && charger_conn) {
		reduce_charging = 0;
		ichrg_value = saved_ichrg_value;
		pmic_set_chg_current(ichrg_value);
	}

done:
	schedule_delayed_work(&charger_misc_work, msecs_to_jiffies(CHARGER_MISC));
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

			 dma_pool_free(udc->td_pool, curr_td, curr_td->td_dma);
                } else {
                        if (last_free_td != NULL)
                                dma_pool_free(udc->td_pool, last_free_td,
                                                last_free_td->td_dma);
                        last_free_td = curr_td;
                }
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

	pr_debug("%s\n", __func__);

	/* if we're in OTG mode, and the Host is currently using the port,
	 * stop now and don't rip the controller out from under the
	 * ehci driver
	 */
	if (udc->gadget.is_otg) {
		if (!(fsl_readl(&dr_regs->otgsc) & OTGSC_STS_USB_ID)) {
			pr_debug("udc: Leaving early\n");
			return;
		}
	}

	last_free_td = NULL;

	/* disable all INTR */
	fsl_writel(0, &dr_regs->usbintr);

	/* Set stopped bit for isr */
	udc->stopped = 1;

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

/* constrain controller's VBUS power usage
 * This call is used by gadget drivers during SET_CONFIGURATION calls,
 * reporting how much power the device may consume.  For example, this
 * could affect how quickly batteries are recharged.
 *
 * Returns zero on success, else negative errno.
 */
static int fsl_vbus_draw(struct usb_gadget *gadget, unsigned mA)
{
	struct fsl_udc *udc;

	udc = container_of(gadget, struct fsl_udc, gadget);
	ichrg_value = mA;
	atomic_set(&enumerated, 1);
	charger_enumerated = mA;
	third_party_charger = 0;
	fsl_vbus_done = 1;
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
	/* If device already enumerated, return. */
	if (charger_connected() && (udc_controller->gadget.speed != USB_SPEED_UNKNOWN)
			&& (udc_controller->gadget.configured) ) {
		return;
	}

	/* No charger, so return */
	if (!charger_connected())
		return;

	/* Re-enumerate */
	fsl_udc_redo_enumerate();
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

	fsl_idle_low_power_exit(udc_controller);
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
		/* Suspend the controller until OTG enable it */
		udc_suspend(udc_controller);
		pr_debug("Suspend udc for OTG auto detect\n");

		/* export udc suspend/resume call to OTG */
		udc_controller->gadget.dev.driver->suspend = fsl_udc_suspend;
		udc_controller->gadget.dev.driver->resume = fsl_udc_resume;

		/* connect to bus through transceiver */
		if (udc_controller->transceiver) {
			retval = otg_set_peripheral(udc_controller->transceiver,
						    &udc_controller->gadget);
			if (retval < 0) {
				ERR("can't bind to transceiver\n");
				driver->unbind(&udc_controller->gadget);
				udc_controller->gadget.dev.driver = 0;
				udc_controller->driver = 0;
				return retval;
			}
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

	if (!udc_controller)
		return -ENODEV;

	if (!driver || driver != udc_controller->driver || !driver->unbind)
		return -EINVAL;

	fsl_idle_low_power_exit(udc_controller);

	if (udc_controller->transceiver)
		(void)otg_set_peripheral(udc_controller->transceiver, 0);

	/* stop DR, disable intr */
	dr_controller_stop(udc_controller);

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
	spin_unlock_irqrestore(&udc_controller->lock, flags);

	/* disconnect gadget before unbinding */
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

static void suspend_port(void)
{
	unsigned int portctrl = 0;

	portctrl = fsl_readl(&dr_regs->portsc1);
        portctrl |= PORTSCX_PHY_LOW_POWER_SPD;
	fsl_writel(portctrl, &dr_regs->portsc1);
}

static void resume_port(void)
{
	unsigned int portctrl = 0;

	if (portctrl & PORTSCX_PORT_SUSPEND) {
		portctrl = fsl_readl(&dr_regs->portsc1);
		portctrl |= PORTSCX_PORT_FORCE_RESUME;
		fsl_writel(portctrl, &dr_regs->portsc1);
	}

	portctrl = fsl_readl(&dr_regs->portsc1);
	portctrl &= ~PORTSCX_PHY_LOW_POWER_SPD;
	fsl_writel(portctrl, &dr_regs->portsc1);

	while ((portctrl & PORTSCX_PORT_SUSPEND)) {
		udelay(100);
		portctrl = fsl_readl(&dr_regs->portsc1);
	}
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
	if (charger_connected())
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");
}

static ssize_t
show_supply_voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	int batt_voltage = read_supply_voltage();

	if (batt_voltage < 0)
		return sprintf(buf, "0\n");
	else
		return sprintf(buf, "%d\n", batt_voltage);
}

static ssize_t
show_chrgraw_voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chrgraw_voltage = read_chrgraw_voltage();
	
	if (chrgraw_voltage < 0)
		return sprintf(buf, "0\n");
	else
		return sprintf(buf, "%d\n", chrgraw_voltage);
}

/*
 * Battery Voltage
 */
static ssize_t
show_voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
	int batt_voltage = read_supply_voltage();

	if (batt_voltage < 0)
		return sprintf(buf, "0\n");
	else
		return sprintf(buf, "%d\n", batt_voltage);
}

/*
 * BPSNS value
 */
static ssize_t
show_bpsns(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", bpsns_value);
}

/*
 * this is the charging flag that indicates that the device is charging
 */
static ssize_t
show_charging(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (charger_connected() && (ichrg_value > 1) )
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");
}

static int pmic_get_batt_current(unsigned short *curr)
{
	t_channel channel;
	unsigned short result[8];

	channel = BATTERY_CURRENT;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	*curr = result[0];

	return 0;
}

/*!
 * Indicates a lobat condition has been hit
 */
static ssize_t
show_lobat_condition(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", lobat_condition);
}

/*
 * Battery current
 */
static ssize_t
show_battery_current(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned short current_raw = 0;
	int current_uA = 0;
	int status = pmic_get_batt_current(&current_raw);

	if (status < 0)
		return sprintf(buf, "0\n");
	else {
		if (current_raw & 0x200)
			current_uA = (0x1FF - (current_raw & 0x1FF)) * BAT_CURRENT_UNIT_UA * (-1);
		else
			current_uA = current_raw & 0x1FF * BAT_CURRENT_UNIT_UA;

		return sprintf(buf, "%d\n", current_uA);
	}
}

/*
 * Fake battery error file till a battery driver is written 
 */
static ssize_t
show_batt_error(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", yoshi_battery_error_flags);
}

/*!
 * Battery ID
 */
static ssize_t
show_battery_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", yoshi_battery_id);
}

/*!
 * Third party charger
 */
static ssize_t
show_third_party(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", third_party_charger);
}

/*
 * Show current ichrg value
 */
static ssize_t
show_ichrg_setting(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ichrg_value);
}

static void pmic_enable_green_led(int enable)
{
	if (enable == 1) {
		mc13892_bklit_set_current(LIT_KEY, 0x7);
		mc13892_bklit_set_ramp(LIT_KEY, 0);
		mc13892_bklit_set_dutycycle(LIT_KEY, 0x3f);
	}
	else {
		mc13892_bklit_set_current(LIT_KEY, 0);
		mc13892_bklit_set_dutycycle(LIT_KEY, 0);
	}
}

static int pmic_set_chg_current(unsigned short curr)
{
	unsigned int mask;
	unsigned int value;

	/* If we have no battery, charging must be shut off unless in factory mode */
	if (!yoshi_battery_valid() && !pmic_check_factory_mode()) {
	    pr_debug("%s: no battery, disable charging\n", __func__);
	    curr = 0;
	}	

	value = BITFVAL(BIT_CHG_CURR, curr);
	mask = BITFMASK(BIT_CHG_CURR);
	CHECK_ERROR(pmic_write_reg(REG_CHARGE, value, mask));

	return 0;
}

static int pmic_set_chg_misc(enum chg_setting type, unsigned short flag)
{
	unsigned int reg_value = 0;
	unsigned int mask = 0;

	switch (type) {
	case BAT_TH_CHECK_DIS:
		reg_value = BITFVAL(BAT_TH_CHECK_DIS, flag);
		mask = BITFMASK(BAT_TH_CHECK_DIS);
		break;
	case RESTART_CHG_STAT:
		reg_value = BITFVAL(RESTART_CHG_STAT, flag);
		mask = BITFMASK(RESTART_CHG_STAT);
		break;
	case AUTO_CHG_DIS:
		reg_value = BITFVAL(AUTO_CHG_DIS, flag);
		mask = BITFMASK(AUTO_CHG_DIS);
		break;
	case VI_PROGRAM_EN:
		reg_value = BITFVAL(VI_PROGRAM_EN, flag);
		mask = BITFMASK(VI_PROGRAM_EN);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(REG_CHARGE, reg_value, mask));
	return 0;
}

static void pmic_start_charging(int value)
{
	/* Don't enable charging if we're in factory mode */
	if (pmic_check_factory_mode() && !yoshi_battery_valid()) {
	    pr_debug("%s: factory mode, do not enable charging\n", __func__);
	    return;
	}

	/* Battery thermistor check disable */
	pmic_set_chg_misc(BAT_TH_CHECK_DIS, 1);
	
	/* Keep auto charging enabled */
	pmic_set_chg_misc(AUTO_CHG_DIS, 0);
	
	/* Allow programming of charge voltage and current */
	pmic_set_chg_misc(VI_PROGRAM_EN, 1);

	/* Set the ichrg */
	pmic_set_chg_current(value);
	ichrg_value = value;

	/* Turn off CYCLB disable */	
	pmic_write_reg(REG_CHARGE, (0 << CHRGCYCLB), (1 << CHRGCYCLB));

	/* PLIM: bits 15 and 16 */
	pmic_write_reg(REG_CHARGE, (0x2 << 15), (0x3 << 15));

	/* PLIMDIS: bit 17 */
	pmic_write_reg(REG_CHARGE, (0 << 17), (1 << 17));

	/* Start the charger state machine */
	pmic_set_chg_misc(RESTART_CHG_STAT, 1);
}

/*
 * Dynamically enable/disable the charger even if the charge
 * cable is connected.
 */
static void fsl_charger_enable(int enable)
{
	if (enable) {
		/*
		 * Check if charger is connected and if ichrg is set to 0
	 	 */
		if (!pmic_check_factory_mode() && charger_connected() && !ichrg_value) {
			pmic_start_charging(charging_wall_mA);
			schedule_delayed_work(&charger_misc_work, msecs_to_jiffies(CHARGER_MISC));
		}
		udc_suspended_mode = 0;
	}
	else {
		/* Fake a suspended mode state so that timer does not run */
		udc_suspended_mode = 1;

		full_battery = 0;

		/* Set the ichrg to 0 */
		pmic_set_chg_current(0);
		ichrg_value = 0;

		/* Clear the charger timer counter */
		charger_timer_expiry_counter = 0;

		/* Shut off the green LED */
		pmic_enable_green_led(0);

		/* Clear out saved ichrg value */
		saved_ichrg_value = 0;

		/* Shut off the CHRGLEDEN bit */
		pmic_green_led_disable(0);

		/* Clear the green LED flag */
		green_led_set = 0;

		atomic_set(&enumerated, 0);

		/* Clear out the variables */
		third_party_charger = 0;
		charger_bpon = 0;
		wall_charger = 0;
		charger_conn_probe = 0;
		charger_enumerated = 0;
	}
}

/* Third party detect timer */
static ssize_t
show_third_party_timer(struct device *_dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", third_party_detect_timer);
}

static ssize_t
store_third_party_timer(struct device *_dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int value;

	if ( (sscanf(buf, "%d", &value) > 0) ) {
		third_party_detect_timer = value;
		return strlen(buf);
	}

	return -EINVAL;
}
static DEVICE_ATTR (third_party_timer, S_IRUGO|S_IWUSR, show_third_party_timer, store_third_party_timer);

/* USB LPM timer */
static ssize_t
show_usb_lpm_timer_threshold(struct device *_dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", usb_lpm_timer);
}

static ssize_t
store_usb_lpm_timer_threshold(struct device *_dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int value;

	if ( (sscanf(buf, "%d", &value) > 0) ) {
		usb_lpm_timer = value;
		return strlen(buf);
	}

	return -EINVAL;
}

static DEVICE_ATTR (usb_lpm_timer_threshold, S_IRUGO|S_IWUSR, show_usb_lpm_timer_threshold, store_usb_lpm_timer_threshold);
	
/* charger enable/disable on the fly */
static ssize_t
show_charger_state(struct device *_dev, struct device_attribute *attr, char *buf)
{
	/* ichrg_value is 0 if not charging */
	return sprintf(buf, "%d\n", ichrg_value);
}

static ssize_t
store_charger_state(struct device *_dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int value;

	if ((sscanf(buf, "%d", &value) > 0) &&
			((value == 0) || (value == 1))) {
		fsl_charger_enable(value);
		return strlen(buf);
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
			gpio_chrgled_active(value);
			return strlen(buf);
	}

	return -EINVAL;
}
static DEVICE_ATTR (chrgled_state, S_IRUGO|S_IWUSR, NULL, store_chrgled_state);

static void fsl_charger_lobath_unsubscribe(int value)
{
	if (!value) {
		pmic_event_subscribe(EVENT_LOBATLI, lobatli_event);
		pmic_event_subscribe(EVENT_LOBATHI, lobathi_event);
	}
	else {
		pmic_event_unsubscribe(EVENT_LOBATLI, lobatli_event);
		pmic_event_unsubscribe(EVENT_LOBATHI, lobathi_event);
	}
}

static ssize_t
store_charger_lobath_unsub(struct device *_dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int value;

	if ((sscanf(buf, "%d", &value) > 0) &&
		((value == 0) || (value == 1))) {
			fsl_charger_lobath_unsubscribe(value);
			return strlen(buf);
	}

	return -EINVAL;
}

static DEVICE_ATTR (charger_lobath_unsub, S_IRUGO|S_IWUSR, NULL, store_charger_lobath_unsub);

/*! Third party charging */
static void third_party_detect(struct work_struct *work)
{
	char *envp[] = { "CHARGER=thirdparty", NULL };

	if (!atomic_read(&enumerated) && charger_connected() && !wall_charger) {
		printk(KERN_INFO "arcotg_udc: I pmic:charger3rdParty:mV=%d:\n", read_supply_voltage());
		pmic_start_charging(CHARGING_THIRD_PARTY);
		third_party_charger = 1;
		/* Relay the event to userspace */
		kobject_uevent_env(&udc_controller->gadget.dev.parent->kobj, KOBJ_CHANGE, envp);
	}
}

static DECLARE_DELAYED_WORK(third_party_work, third_party_detect);

/*! Failure of enumeration should redo detach/attach event */
static void host_enumeration_detect_fn(struct work_struct *work)
{
	/* If not enumerated and connected to a HOST */
	if (!atomic_read(&enumerated) && charger_connected())
		fsl_udc_redo_enumerate();
}

static DECLARE_DELAYED_WORK(host_enumeration_detect, host_enumeration_detect_fn);

static void fsl_detect_charger_type(void)
{
	unsigned int portsc1;

	portsc1 = fsl_readl(&dr_regs->portsc1);

	pr_debug("%s: prtsc1=0x%x\n", __func__, portsc1);

	if (charger_conn_probe) {

	    	pr_debug("%s: charger conn\n", __func__);

		resume_ctrl();
		mdelay(10);
		charger_conn_probe = 0;
		if ((portsc1 & PORTSCX_LINE_STATUS_BITS) == PORTSCX_LINE_STATUS_UNDEF) {
			charger_set_config();
		}
		return;
	}

	portsc1 = fsl_readl(&dr_regs->portsc1);

	if ((portsc1 & PORTSCX_LINE_STATUS_BITS) == PORTSCX_LINE_STATUS_UNDEF) {
	    	pr_debug("%s: line undef prtsc1=0x%x\n", __func__, portsc1);

		charger_set_config();
		goto out;
	}

	if ((portsc1 & PORTSCX_LINE_STATUS_BITS) == PORTSCX_LINE_STATUS_KSTATE ||
	    (portsc1 & PORTSCX_LINE_STATUS_BITS) == PORTSCX_LINE_STATUS_JSTATE) {
		pr_debug("Connected to a charger: 0x%x\n", portsc1);
		pmic_start_charging(CHARGING_HOST);
		
		kobject_uevent(&udc_controller->gadget.dev.parent->kobj, KOBJ_ADD);
		if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_connected) < 0)
			printk(KERN_ERR "Host: Error creating device file\n");

		fsl_udc_redo_enumerate();

		schedule_delayed_work(&third_party_work, msecs_to_jiffies(third_party_detect_timer ));
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
			pmic_start_charging(CHARGING_HOST);

		kobject_uevent(&udc_controller->gadget.dev.parent->kobj, KOBJ_ADD);
		if (device_create_file(udc_controller->gadget.dev.parent,
				&dev_attr_connected) < 0)
			printk(KERN_ERR "Host: Error creating device file\n");

		schedule_delayed_work(&host_enumeration_detect, msecs_to_jiffies(HOST_ENU_DETECT));
		goto out;
	}

	if (host_detected) {
                /* A host as already been detected */
                pr_debug("Connected to a Host: %d\n", host_detected);
                pmic_start_charging(CHARGING_HOST);

                kobject_uevent(&udc_controller->gadget.dev.parent->kobj, KOBJ_ADD);
                if (device_create_file(udc_controller->gadget.dev.parent,
                                &dev_attr_connected) < 0)
                        printk(KERN_ERR "Host: Error creating device file\n");

		goto out;
        } else {
		pr_debug("Connected to a third-party charger: 0x%x\n", portsc1);
		pmic_start_charging(CHARGING_THIRD_PARTY);
		third_party_charger = 1;
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

static void phy_lpm_work(struct work_struct *unused)
{
    pr_debug("%s: begin\n", __func__);

    if (!charger_connected()) {
	fsl_idle_low_power_enter(udc_controller);
    }
}
DECLARE_DELAYED_WORK(phy_lpm_worker, phy_lpm_work);

static void do_bvalid_detect(struct work_struct *work)
{
	if (bvalid_connected()) {
		pr_debug("%s: connected\n", __func__);

		fsl_detect_charger_type();
		if (!green_led_set)
			gpio_chrgled_active(1);
	}
	else {
		host_detected = 0;
		ichrg_value = 0;
		green_led_set = 0;
		pmic_enable_green_led(0);
		gpio_chrgled_active(1);
		pmic_set_chg_current(0);
		
		pr_debug("%s: disconnect\n", __func__);

		/* Clear out saved ichrg value */
		saved_ichrg_value = 0;

		/* Clear out the variables */
		atomic_set(&enumerated, 0);
		third_party_charger = 0;
		charger_bpon = 0;
		wall_charger = 0;
		charger_conn_probe = 0;
		charger_enumerated = 0;
		full_battery = 0;

		/* Clear the charger timer counter */
		charger_timer_expiry_counter = 0;
		pmic_write_reg(REG_MEM_A, (0 << 4), (1 << 4));

		/* Now, schedule a delayed workqueue to put PHY into LPM */
		schedule_delayed_work(&phy_lpm_worker,
				msecs_to_jiffies(usb_lpm_timer));

		chrgraw_monitor = 0;
		chrgraw_disable = 0;
	}
}

DECLARE_WORK(bvalid_detect_work, do_bvalid_detect);

static void fsl_bvalid_type_detect(unsigned long data)
{
	schedule_work(&bvalid_detect_work);
}
static void do_charger_detect(struct work_struct *work)
{
    pr_debug("%s: begin\n", __func__);

	if (charger_connected()) {
	    pr_debug("%s: charger connected!\n", __func__);

	    fsl_detect_charger_type();
	    if (!green_led_set)
		gpio_chrgled_active(1);
	}
	else {

		pr_debug("%s: no charger connected!\n", __func__);

		host_detected = 0;
		ichrg_value = 0;
		green_led_set = 0;
		pmic_enable_green_led(0);
		gpio_chrgled_active(1);
		pmic_set_chg_current(0);

		/* Clear out saved ichrg value */
		saved_ichrg_value = 0;

		/* Clear out the variables */
		atomic_set(&enumerated, 0);
		third_party_charger = 0;
		charger_bpon = 0;
		wall_charger = 0;
		charger_conn_probe = 0;
		charger_enumerated = 0;
		full_battery = 0;

		/* Clear the charger timer counter */
		charger_timer_expiry_counter = 0;
		pmic_write_reg(REG_MEM_A, (0 << 4), (1 << 4));

		/* Now, schedule a delayed workqueue to put PHY into LPM */
		schedule_delayed_work(&phy_lpm_worker,
				msecs_to_jiffies(usb_lpm_timer));

		chrgraw_monitor = 0;
		chrgraw_disable = 0;
	}
}

DECLARE_WORK(charger_detect_work, do_charger_detect);

static void fsl_charger_type_detect(unsigned long data)
{
	schedule_work(&charger_detect_work);
}

extern void mx50_anadig_clr(void);

static void fsl_idle_low_power_exit(struct fsl_udc *udc)
{
	if (!udc->suspended)
		return;

	if (udc_phy_suspend)
		return;

	pmic_vusb2_enable(1);

	mx50_anadig_clr();

	/* First enable the clock */
	clk_enable(usb_ahb_clk);
	clk_enable(usb_phy1_clk);
	udelay(1000); /* Propagation time after clock enable */

	USB_PHY_CTR_FUNC &= ~(1 <<11) ;
	USB_PHY_CTR_FUNC |= (1 << 12);

	/* Next the port */
	resume_port();

	/* Enable DR irq reg and set controller Run */
	if (udc_controller->stopped) {
		dr_controller_setup(udc_controller);
		dr_controller_run(udc_controller);

		udc_controller->stopped = 0;
		udc_controller->usb_state = USB_STATE_ATTACHED;
		udc_controller->ep0_state = WAIT_FOR_SETUP;
		udc_controller->ep0_dir = 0;
	}

	resume_ctrl();

	mx50_anadig_clr();

	udc->suspended = 0;
	enable_irq(ARCOTG_IRQ);
	bus_resume(udc);

	/* Disable accessory charging */
	accessory_charger_disable();
}

static void fsl_idle_low_power_enter(struct fsl_udc *udc)
{
	if (udc->suspended)
		return;

	if (udc_phy_suspend)
		return;

	pr_debug("%s: begin\n", __func__);

	disable_irq(ARCOTG_IRQ);
	udc->suspended = 1;
	fsl_writel(0, &dr_regs->usbintr);

	/* Suspend PORT first */
	suspend_port();

	/* Settling time */
	mdelay(20);

	/* Suspend Controller next */
	suspend_ctrl();

	/* Settling time */
	mdelay(20);

	/* Tell upper stack that PHY suspended */
	suspend_irq(udc);

	USB_PHY_CTR_FUNC &= ~(1 << 12);
        USB_PHY_CTR_FUNC |= (1 <<11) ;

	clk_disable(usb_ahb_clk);
	clk_disable(usb_phy1_clk);
	udc_detection = 0;
	udc->stopped = 1;

	pmic_vusb2_enable(0);

	mx50_anadig_clr();

	if (!charger_connected()) {
		/* Enable accessory charging */
		accessory_charger_enable();
	}
}

static void callback_bvalid_detect(void *param)
{
	struct fsl_udc *udc = (struct fsl_udc *)param;

	if (bvalid_connected()) {
		if (yoshi_battery_error_flags)
			return;

		/* charger already detected */
		if (atomic_read(&charger_atomic_detection) == 1)
			return;

		atomic_set(&charger_atomic_detection, 1);

		fsl_idle_low_power_exit(udc);
	}
	else {
		if (atomic_read(&charger_atomic_detection) == 0)
			return;

		atomic_set(&charger_atomic_detection, 0);

		if (udc->driver && udc->driver->disconnect)
			udc->driver->disconnect(&udc->gadget);

		device_remove_file(udc->gadget.dev.parent, &dev_attr_connected);

		kobject_uevent_atomic(&udc->gadget.dev.parent->kobj, KOBJ_REMOVE);
	}

	if (udc_stopped) {
		udc_stopped = 0;

		mod_timer(&udc->bvalid_detect_timer,
			jiffies + (msecs_to_jiffies(CHARGER_TYPE_SUSPEND)));
	}
	else
		mod_timer(&udc->bvalid_detect_timer,
			jiffies + (msecs_to_jiffies(CHARGER_TYPE)));
}

/*
 * Charger callback from Atlas
 */
static void callback_charger_detect(void *param)
{
	struct fsl_udc *udc = (struct fsl_udc *)param;

	/*
	 * Exit the low power mode before the detection is done
	 */	
	if (charger_connected()) {
		if (yoshi_battery_error_flags)
			return;

		if (atomic_read(&charger_atomic_detection) == 1)
			return;

		atomic_set(&charger_atomic_detection, 1);

		fsl_idle_low_power_exit(udc);
	}
	else {
		if (atomic_read(&charger_atomic_detection) == 0)
			return;

		atomic_set(&charger_atomic_detection, 0);

		if (udc->driver && udc->driver->disconnect)
			udc->driver->disconnect(&udc->gadget);

		device_remove_file(udc->gadget.dev.parent, &dev_attr_connected);

		kobject_uevent_atomic(&udc->gadget.dev.parent->kobj, KOBJ_REMOVE);
	}

	if (udc_stopped) {
		udc_stopped = 0; 
		mod_timer(&udc->detect_timer, 
				jiffies + (msecs_to_jiffies(CHARGER_TYPE_SUSPEND)));
	}
	else
		mod_timer(&udc->detect_timer, jiffies + (msecs_to_jiffies(CHARGER_TYPE)));
}

static void callback_bpon_event(void *param)
{
	printk(KERN_DEBUG "callback_bpon_event\n");
	pmic_enable_bpon_charging();
}

/*
 *! The charger timer is configured for 120 minutes in APLite.
 *  The charger is automatically turned off after 120 minutes of
 *  charging. Once this occurs, the APLite generates a CHGFAULT
 *  with the sense bits correctly set to "10"
 */
static void callback_faulti(void *param)
{
	int sense_0 = 0;
	/* Bits 9 and 8 should be "10" to detect charger timer expiry */
	int charger_timer_mask = 0x200;

	pmic_read_reg(REG_INT_SENSE0, &sense_0, 0xffffff);

	sense_0 &= CHGFAULTM;

	switch(sense_0) /*see mc13892 data sheet*/
	{	case 0x100:	printk(KERN_INFO "arcotg_udc: I pmic:faulti:sense_0=0x%x:charger path over voltage/dissipation in pass device.\n", sense_0);
		break;
		case 0x200: 	printk(KERN_INFO "arcotg_udc: I pmic:faulti:sense_0=0x%x:battery dies/charger times out.\n", sense_0);	
		break;
		case 0x300:	printk(KERN_INFO "arcotg_udc: I pmic:faulti:sense_0=0x%x:battery out of temperature.\n", sense_0);
		break;
		default:
				;
	}
	/* Restart charging if the charger timer has expired */
	if (sense_0 & charger_timer_mask) {

	    /*
	     * According to the IEEE 1725 spec, the max charger timer
	     * should not exceed 6 hours or 360 minutes
	     */
	    if (charger_timer_expiry_counter >= charger_timer_expiry_threshold) {

		pr_debug("%s: charge timer expired!  Do not restart charging\n", __func__);
		charger_timer_expiry_counter++;

	    } else {
	    
		if (charger_connected()) {
		    pmic_restart_charging();
		    charger_timer_expiry_counter++;
		}
	    }
	} 
}

/*
 * CHRGSE1B wall charger detect 
 */
static void callback_se1b(void *param)
{
	int sense_0 = 0;

	pmic_read_reg(REG_PU_MODE_S, &sense_0, 0xffffff);

	/* In the MX508 1.1 silicon, this event significes the presence of a wall charger */
	if (cpu_is_mx50_rev(CHIP_REV_1_1) >= 1) {

		/* if chgdet has not occurred yet, set the ichrg to 480mA and get started */
		if (!ichrg_value)
			pmic_set_chg_current(CHARGING_THIRD_PARTY);
	}
}

/*!
 * The battery driver has already computed the voltage. So, do not redo 
 * this calculation as it unnecessary generates PMIC/SPI traffic every
 * 10 seconds. Reuse the value unless battery error flags are set.
 */
static int read_supply_voltage(void)
{
	unsigned short bp = 0;

	// force read the voltage if it is not initialized.	
	if ((!yoshi_battery_error_flags) && (yoshi_voltage > 0))
		return yoshi_voltage;

	if (pmic_adc_convert(APPLICATION_SUPPLY, &bp)) {
		printk(KERN_ERR "arcotg_udc: E pmic:could not read APPLICATION_SUPPLY from adc:\n");
		return -1;
	}

	/*
	 * MC13892: Multiply the adc value by BATT_VOLTAGE_SCALE to get
	 * the battery value. This is from the Atlas Lite manual
	 */
	return bp*BATT_VOLTAGE_SCALE/1000; /* Return in mV */
}

static int read_chrgraw_voltage(void)
{
	unsigned short bp = 0;

	if (pmic_adc_convert(2, &bp)) {
		printk(KERN_ERR "arcotg_udc: E pmic:could not read CHRGRAW from adc:\n");
		return -1;
	}

	return bp;
}

/*!
 * Read the PMIC A/D for Charger Voltage, i.e. CHGRAW
 */
static int read_charger_voltage(void)
{
	unsigned short bp = 0;

	if (pmic_adc_convert(CHARGE_VOLTAGE, &bp)) {
		printk(KERN_ERR "arcotg_udc: E pmic:could not read CHARGER_VOLTAGE from adc:\n");
		return -1;
	}

	return bp*CHARGE_VOLTAGE_SCALE/1000; /* Return in mV */
}

/*!
 * Read the PMIC A/D for accurate LOBAT readings
 */
static int read_lobat_supply_voltage(void)
{
	unsigned short bp = 0;
	int err = 0;

	err = pmic_adc_convert(APPLICATION_SUPPLY, &bp);

	if (err) {
		printk(KERN_ERR "arcotg_udc: E pmic:could not read APPLICATION_SUPPLY from adc: %d:\n", err);
		return err;
	}

	/*
	 * MC13892: Multiply the adc value by BATT_VOLTAGE_SCALE to get
	 * the battery value. This is from the Atlas Lite manual
	 */
	return bp*BATT_VOLTAGE_SCALE/1000; /* Return in mV */
}

static void lobat_work(void)
{
	int batt_voltage = read_lobat_supply_voltage();

	if (batt_voltage < 0)
		return;

	if (udc_controller->lobathi == 1) {
		udc_controller->lobathi = 0;

		if (batt_voltage > LOW_VOLTAGE_THRESHOLD) {
			if (printk_ratelimit())
				printk(KERN_ERR "arcotg_udc: I pmic:lobat:lobathi=%d:\n", batt_voltage);
			goto out;
		}

		post_lobat_event(0);
	}

	if (udc_controller->lobatli == 1) {
		udc_controller->lobatli = 0;

		if (batt_voltage > CRITICAL_VOLTAGE_THRESHOLD) {
			if (printk_ratelimit())
				printk(KERN_ERR "arcotg_udc: I pmic:lobat:lobatli=%d:\n", batt_voltage);
			goto out;
		}

		post_lobat_event(1);
	}
out:
	return;
		
}
		
/*
 * LOBATHI event callback from Atlas
 */
static void callback_lobathi_event(void *param)
{
	lobat_work();
}

/*
 * LOBATLI event callback from Atlas
 */
static void callback_lobatli_event(void *param)
{
	lobat_work();
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
	unsigned int reg_memory_a;

	if (strcmp(pdev->name, driver_name)) {
		VDBG("Wrong device\n");
		return -ENODEV;
	}

#ifdef ARC_OTG
	pmic_read_reg(REG_INT_SENSE0, &sense_0, 0xffffff);

	if (!(sense_0 & IDGND_SENSE)) {
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

#ifdef ARC_OTG
	/* Memory and interrupt resources will be passed from OTG */
	udc_controller->transceiver = otg_get_transceiver();
	if (!udc_controller->transceiver) {
		printk(KERN_ERR "Can't find OTG driver!\n");
	}

	res = otg_get_resources();
	if (!res) {
		DBG("resource not registered!\n");
		return -ENODEV;
	}
#else
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENXIO;
		goto err1a;
	}

	if (!request_mem_region(res->start, resource_size(res),
				driver_name)) {
		ERR("request mem region for %s failed \n", pdev->name);
		ret = -EBUSY;
		goto err1a;
	}
#endif
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

#ifdef ARC_OTG
	res++;
	udc_controller->irq = res->start;
#else
	udc_controller->irq = platform_get_irq(pdev, 0);
#endif
	if (!udc_controller->irq) {
		ret = -ENODEV;
		goto err2;
	}

	ret = request_irq(udc_controller->irq, fsl_udc_irq, IRQF_DISABLED,
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

	if (!udc_controller->transceiver) {
		/* initialize usb hw reg except for regs for EP,
		 * leave usbintr reg untouched */
		dr_controller_setup(udc_controller);
	}

	INIT_DELAYED_WORK(&udc_controller->charger_detect_work, do_charger_detect_work);
	schedule_delayed_work(&udc_controller->charger_detect_work,
				msecs_to_jiffies(CHARGER_STARTUP));
	usb_ahb_clk = clk_get(NULL, "usb_ahb_clk");
	usb_phy1_clk = clk_get(NULL, "usb_phy1_clk");

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

	/* The charger detect work timer */
	init_timer(&udc_controller->detect_timer);
	udc_controller->detect_timer.data = (unsigned long) udc_controller;
	udc_controller->detect_timer.function = fsl_charger_type_detect;

	/* bvalid detect timer */
	init_timer(&udc_controller->bvalid_detect_timer);
	udc_controller->bvalid_detect_timer.data = (unsigned long) udc_controller;
	udc_controller->bvalid_detect_timer.function = fsl_bvalid_type_detect;

	/*
	 * Charger 
	 */
	charger_detect.param = (void *)udc_controller;
	charger_detect.func = callback_charger_detect;
	CHECK_ERROR(pmic_event_subscribe(EVENT_CHGDETI, charger_detect));

	bvalid_detect.param = (void *)udc_controller;
	bvalid_detect.func = callback_bvalid_detect;
	ret = pmic_event_subscribe(EVENT_BVALIDI, bvalid_detect);

	pr_debug("pmic_event_subscribe: %d\n", ret);

	lobatli_event.param = (void *)udc_controller;
	lobatli_event.func = callback_lobatli_event;	
	CHECK_ERROR(pmic_event_subscribe(EVENT_LOBATLI, lobatli_event));

	lobathi_event.param = (void *)udc_controller;
	lobathi_event.func = callback_lobathi_event;
	CHECK_ERROR(pmic_event_subscribe(EVENT_LOBATHI, lobathi_event));

	bpon_event.param = (void *)udc_controller;
	bpon_event.func = callback_bpon_event;
	CHECK_ERROR(pmic_event_subscribe(EVENT_BPONI, bpon_event));

	chgfaulti_event.param = (void *)udc_controller;
	chgfaulti_event.func = callback_faulti;
	CHECK_ERROR(pmic_event_subscribe(EVENT_CHGFAULTI, chgfaulti_event));

	chrgse1b_event.param = (void *)udc_controller;
	chrgse1b_event.func = callback_se1b;
	CHECK_ERROR(pmic_event_subscribe(EVENT_SE1I, chrgse1b_event));

	/* Set the BPSNS */
	CHECK_ERROR(pmic_write_reg(PMIC_REG_PC_0, BPSNS_VALUE << BPSNS_OFFSET,
				BPSNS_MASK << BPSNS_OFFSET));

	/* If bit is already set, then it is a fall through */
	pmic_read_reg(REG_MEM_A, &reg_memory_a, (0x1f << 0));
	if (reg_memory_a & 0x10)
		printk(KERN_ERR "boot: I def:Booting out of critical battery::\n");
	
	/* Clear out bit #4 in MEMA */
	pmic_write_reg(REG_MEM_A, (0 << 4), (1 << 4));

	/* Turn off the green led */
	pmic_enable_green_led(0);
	pmic_green_led_disable(1);

	if (charger_connected()) {
		charger_conn_probe = 1;
		printk(KERN_INFO "arcotg_udc: I pmic:chargerInit::\n");
		/*
		 * The module is just loaded, check capacity. If <= 4%,
		 * then we are surely in recovery utils
		 */
		if (yoshi_battery_capacity <= CHARGER_BOOT_THRESHOLD) {
			/* Turn on bit #4 */
			pmic_write_reg(REG_MEM_A, (1 << 4), (1 << 4));
			crit_boot_flag = 1;
		}

	}
	/* yellow LED - Atlas Control */
	gpio_chrgled_active(1);

	third_party_detect_timer = THIRD_PARTY_DETECT;
	usb_lpm_timer = CHARGER_TYPE_LPM;

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

	schedule_delayed_work(&charger_misc_work, msecs_to_jiffies(CHARGER_MISC));

	fsl_udc_init_debugfs(udc_controller);

	udc_controller->lobatli = 0;
	udc_controller->lobathi = 0;

	/* Turn on charger detection on IMX508 */

	/* Turn on CHRGDETON */
	USBH1_PHY_CTRL0 |= 0x800000;

	/* Wait 300us */
	udelay(300);

	/* Trun on CHRGDETEN */
	USBH1_PHY_CTRL0 |= 0x1000000;

	/* Set the charger timer expiry threshold and ichrg */
	charging_wall_mA = CHARGING_WALL;

	if (mx50_board_is(BOARD_ID_TEQUILA)) {
	    charger_timer_expiry_threshold = CHRG_TIMER_THRESHOLD_TEQ;
	}

        /* magic number from Data Sheets/Freescale/iMX50_508/MX50_Preliminary_Reference_Manual Rev B.pdf 
         * Page 3039
         * for data integrity*/
        
        /*
         *  -enable high speed driver pre-emphasis
         *  -reduce PLL charge pump current (Icp) by 50%
         *  -change PMOS HS driver timing from 2x to 8x
         *  -increase HS driver amplitude by 2.5% (nominal is 17.78mA)  
         * */
        USB_PHY_CTR_FUNC2 = 0x90545601;

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
	cancel_rearming_delayed_work(&charger_misc_work);
	del_timer_sync(&udc_controller->detect_timer);
	del_timer_sync(&udc_controller->bvalid_detect_timer);
	cancel_rearming_delayed_work(&udc_controller->charger_detect_work);
	cancel_rearming_delayed_work(&pc_wq);
	cancel_rearming_delayed_work(&third_party_work);
	cancel_rearming_delayed_work(&host_enumeration_detect);
	cancel_rearming_delayed_work(&phy_lpm_worker);
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

	/* Charger */
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_CHGDETI, charger_detect));
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_BVALIDI, bvalid_detect));
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_LOBATLI, lobatli_event));
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_LOBATHI, lobathi_event));
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_BPONI, bpon_event));
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_CHGFAULTI, chgfaulti_event));
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_SE1I, chrgse1b_event));

	clk_put(usb_ahb_clk);
	clk_put(usb_phy1_clk);
	udc_controller->lobatli = 0;
	udc_controller->lobathi = 0;

#ifndef ARC_OTG
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));
#endif

	device_unregister(&udc_controller->gadget.dev);
	/* free udc --wait for the release() finished */
	wait_for_completion(&done);

	free_irq(udc_controller->irq, udc_controller);

	/*
	 * do platform specific un-initialization:
	 * release iomux pins, etc.
	 */
	if (pdata->platform_uninit)
		pdata->platform_uninit(pdata);

	return 0;
}

static int udc_suspend(struct fsl_udc *udc)
{
	udc->stopped = 1;
	return 0;
}

/*-----------------------------------------------------------------
 * Modify Power management attributes
 * Used by OTG statemachine to disable gadget temporarily
 -----------------------------------------------------------------*/
static int fsl_udc_suspend(struct device *dev, pm_message_t state)
{
	cancel_rearming_delayed_work(&charger_misc_work);
	cancel_rearming_delayed_work(&third_party_work);
	del_timer_sync(&udc_controller->detect_timer);
	flush_work(&charger_detect_work);
	del_timer_sync(&udc_controller->bvalid_detect_timer);
	flush_work(&bvalid_detect_work);

	return udc_suspend(udc_controller);
}

/*-----------------------------------------------------------------
 * Invoked on USB resume. May be called in_interrupt.
 * Here we start the DR controller and enable the irq
 *-----------------------------------------------------------------*/
static int fsl_udc_resume(struct device *dev)
{
	/* Enable DR irq reg and set controller Run */
	if (udc_controller->stopped) {
		dr_controller_setup(udc_controller);
		dr_controller_run(udc_controller);
	}
	udc_controller->usb_state = USB_STATE_ATTACHED;
	udc_controller->ep0_state = WAIT_FOR_SETUP;
	udc_controller->ep0_dir = 0;
	return 0;
}

static int arcotg_suspend(struct platform_device *pdev, pm_message_t state)
{
	int batt_voltage = 0;

	pmic_write_reg(PMIC_REG_PC_0, BPSNS_VALUE_SUSPEND << BPSNS_OFFSET,
					BPSNS_MASK << BPSNS_OFFSET);

	if (mxc_spi_suspended)
		batt_voltage = read_supply_voltage();
	else
		batt_voltage = read_lobat_supply_voltage();

	if (batt_voltage == -EBUSY && !mxc_spi_suspended) {
		batt_voltage = read_supply_voltage();
	}

	if (yoshi_battery_valid() && batt_voltage && (batt_voltage <= CHRG_SUS_LOW_VOLTAGE))
		return -EBUSY;

	if (yoshi_battery_valid() && batt_voltage && (batt_voltage <= LOW_VOLTAGE_THRESHOLD)) {
		post_lobat_event(0);
		return -EBUSY;
	}

	udc_suspended_mode = 1;
	cancel_rearming_delayed_work(&charger_misc_work);
	cancel_rearming_delayed_work(&third_party_work);
	del_timer_sync(&udc_controller->detect_timer);
	flush_work(&charger_detect_work);
	del_timer_sync(&udc_controller->bvalid_detect_timer);
	flush_work(&bvalid_detect_work);

	udc_stopped = 1;
	gpio_chrgled_active(1);
	return 0;
}

/*! Phy low power on resumw */
static void phy_low_power_resume(struct work_struct *work)
{
	if (!charger_connected()) {
		udc_stopped = 0;
		pr_debug("arcotg_udc: Putting Phy in low power mode\n");
	}
}

static int arcotg_resume(struct platform_device *pdev)
{
	udc_suspended_mode = 0;
	schedule_delayed_work(&charger_misc_work, msecs_to_jiffies(CHARGER_MISC));
	schedule_delayed_work(&phy_resume_work, msecs_to_jiffies(PHY_LPM_TIMER));
	return 0;
}

/*-------------------------------------------------------------------------
	Register entry point for the peripheral controller driver
--------------------------------------------------------------------------*/

static struct platform_driver udc_driver = {
	.remove  = __exit_p(fsl_udc_remove),
	/* these suspend and resume are not usb suspend and resume */
	.suspend = arcotg_suspend,
	.resume  = arcotg_resume,
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
