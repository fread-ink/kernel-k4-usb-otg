/*
 * Copyright (C) 2004-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2010-2011 Amazon.com Inc., All Rights Reserved.
 * Manish Lachwani (lachwani@amazon.com)
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
/*
 * Implementation based on rtc-ds1553.c
 */

/*!
 * @defgroup RTC Real Time Clock (RTC) Driver
 */
/*!
 * @file rtc-mxc_v2.c
 * @brief Real Time Clock interface
 *
 * This file contains Real Time Clock interface for Linux.
 *
 * @ingroup RTC
 */

#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <linux/mxc_srtc.h>
#include <linux/pmic_external.h>
#include <linux/pmic_rtc.h>
#include <linux/kindle_version.h>

#define SRTC_LPSCLR_LLPSC_LSH	17	 /* start bit for LSB time value */

#define SRTC_LPPDR_INIT       0x41736166	/* init for glitch detect */

#define SRTC_LPCR_SWR_LP      (1 << 0)	/* lp software reset */
#define SRTC_LPCR_EN_LP       (1 << 3)	/* lp enable */
#define SRTC_LPCR_WAE         (1 << 4)	/* lp wakeup alarm enable */
#define SRTC_LPCR_SAE         (1 << 5)	/* lp security alarm enable */
#define SRTC_LPCR_SI          (1 << 6)	/* lp security interrupt enable */
#define SRTC_LPCR_ALP         (1 << 7)	/* lp alarm flag */
#define SRTC_LPCR_LTC         (1 << 8)	/* lp lock time counter */
#define SRTC_LPCR_LMC         (1 << 9)	/* lp lock monotonic counter */
#define SRTC_LPCR_SV          (1 << 10)	/* lp security violation */
#define SRTC_LPCR_NSA         (1 << 11)	/* lp non secure access */
#define SRTC_LPCR_NVEIE       (1 << 12)	/* lp non valid state exit int en */
#define SRTC_LPCR_IEIE        (1 << 13)	/* lp init state exit int enable */
#define SRTC_LPCR_NVE         (1 << 14)	/* lp non valid state exit bit */
#define SRTC_LPCR_IE          (1 << 15)	/* lp init state exit bit */

#define SRTC_LPCR_ALL_INT_EN (SRTC_LPCR_WAE | SRTC_LPCR_SAE | \
			      SRTC_LPCR_SI | SRTC_LPCR_ALP | \
			      SRTC_LPCR_NVEIE | SRTC_LPCR_IEIE)

#define SRTC_LPSR_TRI         (1 << 0)	/* lp time read invalidate */
#define SRTC_LPSR_PGD         (1 << 1)	/* lp power supply glitc detected */
#define SRTC_LPSR_CTD         (1 << 2)	/* lp clock tampering detected */
#define SRTC_LPSR_ALP         (1 << 3)	/* lp alarm flag */
#define SRTC_LPSR_MR          (1 << 4)	/* lp monotonic counter rollover */
#define SRTC_LPSR_TR          (1 << 5)	/* lp time rollover */
#define SRTC_LPSR_EAD         (1 << 6)	/* lp external alarm detected */
#define SRTC_LPSR_IT0         (1 << 7)	/* lp IIM throttle */
#define SRTC_LPSR_IT1         (1 << 8)
#define SRTC_LPSR_IT2         (1 << 9)
#define SRTC_LPSR_SM0         (1 << 10)	/* lp security mode */
#define SRTC_LPSR_SM1         (1 << 11)
#define SRTC_LPSR_STATE_LP0   (1 << 12)	/* lp state */
#define SRTC_LPSR_STATE_LP1   (1 << 13)
#define SRTC_LPSR_NVES        (1 << 14)	/* lp non-valid state exit status */
#define SRTC_LPSR_IES         (1 << 15)	/* lp init state exit status */

#define MAX_PIE_NUM     15
#define MAX_PIE_FREQ    32768
#define MIN_PIE_FREQ	1

#define SRTC_PI0         (1 << 0)
#define SRTC_PI1         (1 << 1)
#define SRTC_PI2         (1 << 2)
#define SRTC_PI3         (1 << 3)
#define SRTC_PI4         (1 << 4)
#define SRTC_PI5         (1 << 5)
#define SRTC_PI6         (1 << 6)
#define SRTC_PI7         (1 << 7)
#define SRTC_PI8         (1 << 8)
#define SRTC_PI9         (1 << 9)
#define SRTC_PI10        (1 << 10)
#define SRTC_PI11        (1 << 11)
#define SRTC_PI12        (1 << 12)
#define SRTC_PI13        (1 << 13)
#define SRTC_PI14        (1 << 14)
#define SRTC_PI15        (1 << 15)

#define PIT_ALL_ON      (SRTC_PI1 | SRTC_PI2 | SRTC_PI3 | \
			SRTC_PI4 | SRTC_PI5 | SRTC_PI6 | SRTC_PI7 | \
			SRTC_PI8 | SRTC_PI9 | SRTC_PI10 | SRTC_PI11 | \
			SRTC_PI12 | SRTC_PI13 | SRTC_PI14 | SRTC_PI15)

#define SRTC_SWR_HP      (1 << 0)	/* hp software reset */
#define SRTC_EN_HP       (1 << 3)	/* hp enable */
#define SRTC_TS          (1 << 4)	/* time syncronize hp with lp */

#define SRTC_IE_AHP      (1 << 16)	/* Alarm HP Interrupt Enable bit */
#define SRTC_IE_WDHP     (1 << 18)	/* Write Done HP Interrupt Enable bit */
#define SRTC_IE_WDLP     (1 << 19)	/* Write Done LP Interrupt Enable bit */

#define SRTC_ISR_AHP     (1 << 16)	/* interrupt status: alarm hp */
#define SRTC_ISR_WDHP    (1 << 18)	/* interrupt status: write done hp */
#define SRTC_ISR_WDLP    (1 << 19)	/* interrupt status: write done lp */
#define SRTC_ISR_WPHP    (1 << 20)	/* interrupt status: write pending hp */
#define SRTC_ISR_WPLP    (1 << 21)	/* interrupt status: write pending lp */

#define SRTC_LPSCMR	0x00	/* LP Secure Counter MSB Reg */
#define SRTC_LPSCLR	0x04	/* LP Secure Counter LSB Reg */
#define SRTC_LPSAR	0x08	/* LP Secure Alarm Reg */
#define SRTC_LPSMCR	0x0C	/* LP Secure Monotonic Counter Reg */
#define SRTC_LPCR	0x10	/* LP Control Reg */
#define SRTC_LPSR	0x14	/* LP Status Reg */
#define SRTC_LPPDR	0x18	/* LP Power Supply Glitch Detector Reg */
#define SRTC_LPGR	0x1C	/* LP General Purpose Reg */
#define SRTC_HPCMR	0x20	/* HP Counter MSB Reg */
#define SRTC_HPCLR	0x24	/* HP Counter LSB Reg */
#define SRTC_HPAMR	0x28	/* HP Alarm MSB Reg */
#define SRTC_HPALR	0x2C	/* HP Alarm LSB Reg */
#define SRTC_HPCR	0x30	/* HP Control Reg */
#define SRTC_HPISR	0x34	/* HP Interrupt Status Reg */
#define SRTC_HPIENR	0x38	/* HP Interrupt Enable Reg */

#define SRTC_SECMODE_MASK	0x3	/* the mask of SRTC security mode */
#define SRTC_SECMODE_LOW	0x0	/* Low Security */
#define SRTC_SECMODE_MED	0x1	/* Medium Security */
#define SRTC_SECMODE_HIGH	0x2	/* High Security */
#define SRTC_SECMODE_RESERVED	0x3	/* Reserved */


#define WDOG_LOWER_THRESHOLD	127	/* WDOG timeout is 127 seconds */
#define WDOG_HIGHER_THRESHOLD	187	/* 30 seconds since the last refresh */
					/* First reading after 30 seconds of boot */

struct rtc_drv_data {
	struct rtc_device *rtc;
	void __iomem *ioaddr;
	unsigned long baseaddr;
	int irq;
	struct clk *clk;
	bool irq_enable;
};

static int rtc_suspended = 0;

#define RTC_TIMER_THRESHOLD	30000	/* 30 seconds */
static void do_rtc_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(rtc_work, do_rtc_work);

/* The last good value of the rtc time */
extern u32 saved_last_seconds;

static u32 boot_seconds;

/* completion event for implementing RTC_WAIT_FOR_TIME_SET ioctl */
DECLARE_COMPLETION(srtc_completion);
/* global to save difference of 47-bit counter value */
static int64_t time_diff;

extern int mx50_srsr_wdog(void);

/*!
 * @defgroup RTC Real Time Clock (RTC) Driver
 */
/*!
 * @file rtc-mxc.c
 * @brief Real Time Clock interface
 *
 * This file contains Real Time Clock interface for Linux.
 *
 * @ingroup RTC
 */

static unsigned long rtc_status;

static DEFINE_SPINLOCK(rtc_lock);

static int boot_reason = 0;

static struct device *__dev;

static inline void rtc_write_sync_lp(void __iomem *ioaddr);

static int mxc_rtc_set_time(struct device *dev, struct rtc_time *tm);

static void rtc_last_good_time(void)
{
	unsigned int regA = 0;
	unsigned int regB = 0;

	/* Get the last good saved seconds */
#ifdef CONFIG_MXC_PMIC_MC34708
	pmic_read_reg(REG_MC34708_MEM_A, &regA, (0xff << 16));
	pmic_read_reg(REG_MC34708_MEM_B, &regB, 0x00ffffff);
#else
	pmic_read_reg(REG_MEM_A, &regA, (0xff << 16));
	pmic_read_reg(REG_MEM_B, &regB, 0x00ffffff);
#endif
	saved_last_seconds = (regA << 8)| regB;
}

/*!
 * This function reads the RTC value from some external source.
 *
 * @param  second       pointer to the returned value in second
 *
 * @return 0 if successful; non-zero otherwise
 */
static int get_ext_rtc_time(u32 * second)
{
	int ret = 0;
	struct timeval tmp;
	if (!pmic_rtc_loaded()) {
		printk(KERN_ALERT "rtc cannot get external time, pmic not loaded\n");
		return 1;
	}

	ret = pmic_rtc_get_time(&tmp);

	if (0 == ret)
		*second = tmp.tv_sec;
	else{
		printk(KERN_ALERT "rtc cannot get external time: pmic_rtc_get_time() return 0\n");
		ret = 1;
	}
	return ret;
}

static void mxc_check_boot_reason(void)
{
	unsigned int reg_memory_a;

	if (boot_reason == 1) {
		/* boot reason already found */
#ifdef CONFIG_MXC_PMIC_MC34708
		pmic_write_reg(REG_MC34708_MEM_A, (0 << 0), (0x1f << 0));
#else
		pmic_write_reg(REG_MEM_A, (0 << 0), (0x1f << 0));
#endif
		return;
	}

#ifdef CONFIG_MXC_PMIC_MC34708
	pmic_read_reg(REG_MC34708_MEM_A, &reg_memory_a, (0x1f << 0));
#else
	pmic_read_reg(REG_MEM_A, &reg_memory_a, (0x1f << 0));
#endif

	if (reg_memory_a & 0x2) {
#ifdef CONFIG_MXC_PMIC_MC34708
		pmic_write_reg(REG_MC34708_MEM_A, (0 << 1), (1 << 1));
#else
		pmic_write_reg(REG_MEM_A, (0 << 1), (1 << 1));
#endif
		printk(KERN_ERR "boot: I def:rbt:reset=user_reboot,version=%s:\n", KINDLE_VERSION);
		boot_reason = 1;
		goto out;
	}

	if (reg_memory_a & 0x4) {
#ifdef CONFIG_MXC_PMIC_MC34708
		pmic_write_reg(REG_MC34708_MEM_A, (0 << 2), (1 << 2));
#else
		pmic_write_reg(REG_MEM_A, (0 << 2), (1 << 2));
#endif
		printk(KERN_ERR "boot: W def:hlt:halt=Device,version=%s:\n", KINDLE_VERSION);
		boot_reason = 1;
		goto out;
	}

out:
	if (reg_memory_a & 0x10) {
		/* Clear out bit #4 */
#ifdef CONFIG_MXC_PMIC_MC34708
		pmic_write_reg(REG_MC34708_MEM_A, (0 << 4), (1 << 4));
#else
		pmic_write_reg(REG_MEM_A, (0 << 4), (1 << 4));
#endif
		printk(KERN_ERR "boot: W def:crit:reset=critical_battery,version=%s:\n", KINDLE_VERSION);
		boot_reason = 1;
	}

#ifdef CONFIG_MXC_PMIC_MC34708
	pmic_write_reg(REG_MC34708_MEM_A, (0 << 0), (0x1f << 0));
#else
	pmic_write_reg(REG_MEM_A, (0 << 0), (0x1f << 0));
#endif
}

/*
 * This function sets external RTC
 *
 * @param  second       value in second to be set to external RTC
 *
 * @return 0 if successful; non-zero otherwise
 */
static int set_ext_rtc_time(u32 second)
{
	int ret = 0;
	struct timeval tmp;

	if (!pmic_rtc_loaded()) {
		return 1;
	}

	tmp.tv_sec = second;
	ret = pmic_rtc_set_time(&tmp);
	if (0 != ret)
		ret = 1;

	return ret;
}

/*
 * Both MEMA and MEMB are 24-bit registers. The pmic rtc time is 32-bit.
 * We save the top 8 bits in MEM_A and bottom 24 bits into MEM_B.
 */
static void do_rtc_work(struct work_struct *work)
{
	int ret = 0;
	struct timeval tmp;
	u32 seconds, sec = 0;
	unsigned int regA = 0, regB = 0;
	static int check_boot_runonce = 0;
	struct rtc_time temp_time;

	if (!pmic_rtc_loaded()) {
		goto out;
	}

	/* RTC already suspended */
	if (rtc_suspended)
		return;

	ret = pmic_rtc_get_time(&tmp);

	if (!ret) {
		seconds = tmp.tv_sec;
		rtc_last_good_time();
		if(!check_boot_runonce){
		
			/*********check boot reason************/
			/************from rtc-mxc.c************/
			ret = get_ext_rtc_time(&sec);
			check_boot_runonce = 1;
	
			boot_seconds = sec;
			printk("mxc_rtc: saved=0x%x boot=0x%x\n", saved_last_seconds, boot_seconds);

			if (mx50_srsr_wdog()) {
				printk(KERN_ERR "boot: C def:rst:reset=watchdog,version=%s:\n", KINDLE_VERSION);
				boot_reason = 1;
			}

			mxc_check_boot_reason();

			if (!boot_reason && (saved_last_seconds != 0) &&
				(boot_seconds > saved_last_seconds)) {
				if ( ((boot_seconds - saved_last_seconds) > WDOG_LOWER_THRESHOLD) &&
					((boot_seconds - saved_last_seconds) <= WDOG_HIGHER_THRESHOLD) ) {
					printk(KERN_ERR "boot: C def:rst:reset=watchdog,version=%s:\n", KINDLE_VERSION);
				}
				else {
					printk(KERN_ERR "boot: C def:rst:reset=hard:version=%s\n", KINDLE_VERSION);
				}
			}
			else {
				if (!boot_reason && !saved_last_seconds)
					printk(KERN_ERR "boot: C def:bcut:batterycut=1,version=%s:\n", KINDLE_VERSION);
			}
			/*
		 	* If the current time is less than the saved time, use the saved time
		 	*/
			if (sec < saved_last_seconds) {
				sec = saved_last_seconds;
				set_ext_rtc_time(sec);
			}
			rtc_time_to_tm(sec, &temp_time);
			mxc_rtc_set_time(__dev, &temp_time);
		
			/***********end check reason**********/
	
		}

		if (seconds > saved_last_seconds) {
			/* Mask out 24-bits and save the rtc value */
			regB = seconds & 0x00ffffff;
#ifdef CONFIG_MXC_PMIC_MC34708
			pmic_write_reg(REG_MC34708_MEM_B, regB, 0x00ffffff);
#else
			pmic_write_reg(REG_MEM_B, regB, 0x00ffffff);
#endif

			/* The upper 8-bits are stored in MEM_A begining at location 16 */
			regA = (seconds & 0xff000000) >> 24;
#ifdef CONFIG_MXC_PMIC_MC34708
			pmic_write_reg(REG_MC34708_MEM_A, (regA << 16), (0xff << 16));
#else
			pmic_write_reg(REG_MEM_A, (regA << 16), (0xff << 16));
#endif
		}
	}

out:
	schedule_delayed_work(&rtc_work, msecs_to_jiffies(RTC_TIMER_THRESHOLD));
}

static int show_rtc_pmic_epoch_time(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int regA = 0, regB = 0;
	u32 seconds;

#ifdef CONFIG_MXC_PMIC_MC34708
	pmic_read_reg(REG_MC34708_MEM_A, &regA, (0xff << 16));
	pmic_read_reg(REG_MC34708_MEM_B, &regB, 0x00ffffff);
#else
	pmic_read_reg(REG_MEM_A, &regA, (0xff << 16));
	pmic_read_reg(REG_MEM_B, &regB, 0x00ffffff);
#endif

	regA &= 0x00ff0000; /* Clear out the remaining bits */

	/* Extra shift the MEM_A value */
	seconds = (regA << 8) | regB;
	return sprintf(buf, "%x\n", seconds);
}
static DEVICE_ATTR(rtc_pmic_epoch_time, 0666, show_rtc_pmic_epoch_time, NULL);

static int show_rtc_saved_last_seconds(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", saved_last_seconds);
}
static DEVICE_ATTR(rtc_saved_last_seconds, 0666, show_rtc_saved_last_seconds, NULL);

int wakeup_from_halt = 0;
EXPORT_SYMBOL(wakeup_from_halt);
static ssize_t wakeup_from_halt_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", wakeup_from_halt);
}

static ssize_t wakeup_from_halt_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int value = 0;
	
	if ((sscanf(buf, "%d", &value) > 0) && ((value == 0) || (value == 1))) {
		wakeup_from_halt = value;
		return strlen(buf);
	}

	printk(KERN_ERR "Error setting wakeup_from_halt value \n");
	return -EINVAL;
}
static DEVICE_ATTR(wakeup_from_halt, 0644, wakeup_from_halt_show, wakeup_from_halt_store);

static long wakeup_value = 0;
static ssize_t wakeup_enable_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%ld\n", wakeup_value);
}

static ssize_t wakeup_enable_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int ret;
	struct timeval tmp;
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0) {
		printk(KERN_ERR "Error setting the wakeup value in PMIC RTC\n");
		return -EINVAL;
	}

	if (value != 0) {
		ret = pmic_rtc_get_time(&tmp);
		if (ret < 0)
			printk (KERN_ERR "Failed reading time from RTC\n");

		tmp.tv_sec += value;
		ret = pmic_rtc_set_time_alarm(&tmp);
		if (ret < 0)
			printk (KERN_ERR "Failed setting ALARM for suspend wakeup\n");

		/* Clear TODAI interrupt flag and unmask TODAM */
#ifdef CONFIG_MXC_PMIC_MC34708
		pmic_write_reg(REG_MC34708_INT_STATUS1, 0, 0x2);
		pmic_write_reg(REG_MC34708_INT_MASK1, 0, 0x2);
#else
		pmic_write_reg(REG_INT_STATUS1, 0, 0x2);
		pmic_write_reg(REG_INT_MASK1, 0, 0x2);
#endif
		wakeup_value = value;
	}
	return size;
}
static DEVICE_ATTR(wakeup_enable, 0644, wakeup_enable_show, wakeup_enable_store);

/*!
 * This function does write synchronization for writes to the lp srtc block.
 * To take care of the asynchronous CKIL clock, all writes from the IP domain
 * will be synchronized to the CKIL domain.
 */
static inline void rtc_write_sync_lp(void __iomem *ioaddr)
{
	unsigned int i, count;
	/* Wait for 3 CKIL cycles */
	for (i = 0; i < 3; i++) {
		count = __raw_readl(ioaddr + SRTC_LPSCLR);
		while
			((__raw_readl(ioaddr + SRTC_LPSCLR)) == count);
	}
}

/*!
 * This function updates the RTC alarm registers and then clears all the
 * interrupt status bits.
 *
 * @param  alrm         the new alarm value to be updated in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int rtc_update_alarm(struct device *dev, struct rtc_time *alrm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	struct rtc_time alarm_tm, now_tm;
	unsigned long now, time;
	int ret;

	now = __raw_readl(ioaddr + SRTC_LPSCMR);
	rtc_time_to_tm(now, &now_tm);

	alarm_tm.tm_year = now_tm.tm_year;
	alarm_tm.tm_mon = now_tm.tm_mon;
	alarm_tm.tm_mday = now_tm.tm_mday;

	alarm_tm.tm_hour = alrm->tm_hour;
	alarm_tm.tm_min = alrm->tm_min;
	alarm_tm.tm_sec = alrm->tm_sec;

	rtc_tm_to_time(&now_tm, &now);
	rtc_tm_to_time(&alarm_tm, &time);

	if (time < now) {
		time += 60 * 60 * 24;
		rtc_time_to_tm(time, &alarm_tm);
	}
	ret = rtc_tm_to_time(&alarm_tm, &time);

	__raw_writel(time, ioaddr + SRTC_LPSAR);

	/* clear alarm interrupt status bit */
	__raw_writel(SRTC_LPSR_ALP, ioaddr + SRTC_LPSR);

	return ret;
}

/*!
 * This function is the RTC interrupt service routine.
 *
 * @param  irq          RTC IRQ number
 * @param  dev_id       device ID which is not used
 *
 * @return IRQ_HANDLED as defined in the include/linux/interrupt.h file.
 */
static irqreturn_t mxc_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);
	void __iomem *ioaddr = pdata->ioaddr;
	u32 lp_status, lp_cr;
	u32 events = 0;

	clk_enable(pdata->clk);
	lp_status = __raw_readl(ioaddr + SRTC_LPSR);
	lp_cr = __raw_readl(ioaddr + SRTC_LPCR);

	/* update irq data & counter */
	if (lp_status & SRTC_LPSR_ALP) {
		if (lp_cr & SRTC_LPCR_ALP)
			events |= (RTC_AF | RTC_IRQF);

		/* disable further lp alarm interrupts */
		lp_cr &= ~(SRTC_LPCR_ALP | SRTC_LPCR_WAE);
	}

	/* Update interrupt enables */
	__raw_writel(lp_cr, ioaddr + SRTC_LPCR);

	/* If no interrupts are enabled, turn off interrupts in kernel */
	if (((lp_cr & SRTC_LPCR_ALL_INT_EN) == 0) && (pdata->irq_enable)) {
		disable_irq_nosync(pdata->irq);
		pdata->irq_enable = false;
	}

	/* clear interrupt status */
	__raw_writel(lp_status, ioaddr + SRTC_LPSR);
	clk_disable(pdata->clk);

	rtc_update_irq(pdata->rtc, 1, events);
	return IRQ_HANDLED;
}

/*!
 * This function is used to open the RTC driver.
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_open(struct device *dev)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	clk_enable(pdata->clk);

	if (test_and_set_bit(1, &rtc_status))
		return -EBUSY;
	return 0;
}

/*!
 * clear all interrupts and release the IRQ
 */
static void mxc_rtc_release(struct device *dev)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);

	clk_disable(pdata->clk);

	rtc_status = 0;
}

/*!
 * This function is used to support some ioctl calls directly.
 * Other ioctl calls are supported indirectly through the
 * arm/common/rtctime.c file.
 *
 * @param  cmd          ioctl command as defined in include/linux/rtc.h
 * @param  arg          value for the ioctl command
 *
 * @return  0 if successful or negative value otherwise.
 */
static int mxc_rtc_ioctl(struct device *dev, unsigned int cmd,
			 unsigned long arg)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	unsigned long lock_flags = 0;
	u32 lp_cr;
	u64 time_47bit;
	int retVal;

	switch (cmd) {
	case RTC_AIE_OFF:
		spin_lock_irqsave(&rtc_lock, lock_flags);
		lp_cr = __raw_readl(ioaddr + SRTC_LPCR);
		lp_cr &= ~(SRTC_LPCR_ALP | SRTC_LPCR_WAE);
		if (((lp_cr & SRTC_LPCR_ALL_INT_EN) == 0)
		    && (pdata->irq_enable)) {
			disable_irq(pdata->irq);
			pdata->irq_enable = false;
		}
		__raw_writel(lp_cr, ioaddr + SRTC_LPCR);
		spin_unlock_irqrestore(&rtc_lock, lock_flags);
		return 0;

	case RTC_AIE_ON:
		spin_lock_irqsave(&rtc_lock, lock_flags);
		if (!pdata->irq_enable) {
			enable_irq(pdata->irq);
			pdata->irq_enable = true;
		}
		lp_cr = __raw_readl(ioaddr + SRTC_LPCR);
		lp_cr |= SRTC_LPCR_ALP | SRTC_LPCR_WAE;
		__raw_writel(lp_cr, ioaddr + SRTC_LPCR);
		spin_unlock_irqrestore(&rtc_lock, lock_flags);
		return 0;

	case RTC_READ_TIME_47BIT:
		time_47bit = (((u64) __raw_readl(ioaddr + SRTC_LPSCMR)) << 32 |
			((u64) __raw_readl(ioaddr + SRTC_LPSCLR)));
		time_47bit >>= SRTC_LPSCLR_LLPSC_LSH;

		if (arg && copy_to_user((u64 *) arg, &time_47bit, sizeof(u64)))
			return -EFAULT;

		return 0;

	case RTC_WAIT_TIME_SET:

		/* don't block without releasing mutex first */
		mutex_unlock(&pdata->rtc->ops_lock);

		/* sleep until awakened by SRTC driver when LPSCMR is changed */
		wait_for_completion(&srtc_completion);

		/* relock mutex because rtc_dev_ioctl will unlock again */
		retVal = mutex_lock_interruptible(&pdata->rtc->ops_lock);

		/* copy the new time difference = new time - previous time
		  * to the user param. The difference is a signed value */
		if (arg && copy_to_user((int64_t *) arg, &time_diff,
			sizeof(int64_t)))
			return -EFAULT;

		return retVal;

	}

	return -ENOIOCTLCMD;
}

/*!
 * This function reads the current RTC time into tm in Gregorian date.
 *
 * @param  tm           contains the RTC time value upon return
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;

	rtc_time_to_tm(__raw_readl(ioaddr + SRTC_LPSCMR), tm);
	return 0;
}

/*!
 * This function sets the internal RTC time based on tm in Gregorian date.
 *
 * @param  tm           the time value to be set in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	unsigned long time;
	u64 old_time_47bit, new_time_47bit;
	int ret;
	ret = rtc_tm_to_time(tm, &time);
	if (ret != 0)
		return ret;

	old_time_47bit = (((u64) __raw_readl(ioaddr + SRTC_LPSCMR)) << 32 |
			((u64) __raw_readl(ioaddr + SRTC_LPSCLR)));
	old_time_47bit >>= SRTC_LPSCLR_LLPSC_LSH;

	__raw_writel(time, ioaddr + SRTC_LPSCMR);
	rtc_write_sync_lp(ioaddr);

	new_time_47bit = (((u64) __raw_readl(ioaddr + SRTC_LPSCMR)) << 32 |
			((u64) __raw_readl(ioaddr + SRTC_LPSCLR)));
	new_time_47bit >>= SRTC_LPSCLR_LLPSC_LSH;

	/* update the difference between previous time and new time */
	time_diff = new_time_47bit - old_time_47bit;

	/* signal all waiting threads that time changed */
	complete_all(&srtc_completion);
	/* reinitialize completion variable */
	INIT_COMPLETION(srtc_completion);

	return 0;
}

/*!
 * This function reads the current alarm value into the passed in \b alrm
 * argument. It updates the \b alrm's pending field value based on the whether
 * an alarm interrupt occurs or not.
 *
 * @param  alrm         contains the RTC alarm value upon return
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;

	rtc_time_to_tm(__raw_readl(ioaddr + SRTC_LPSAR), &alrm->time);
	alrm->pending =
	    ((__raw_readl(ioaddr + SRTC_LPSR) & SRTC_LPSR_ALP) != 0) ? 1 : 0;

	return 0;
}

/*!
 * This function sets the RTC alarm based on passed in alrm.
 *
 * @param  alrm         the alarm value to be set in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int mxc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;
	unsigned long lock_flags = 0;
	u32 lp_cr;
	int ret;

	if (rtc_valid_tm(&alrm->time)) {
		if (alrm->time.tm_sec > 59 ||
		    alrm->time.tm_hour > 23 || alrm->time.tm_min > 59) {
			return -EINVAL;
		}
	}

	spin_lock_irqsave(&rtc_lock, lock_flags);
	lp_cr = __raw_readl(ioaddr + SRTC_LPCR);

	ret = rtc_update_alarm(dev, &alrm->time);
	if (ret)
		goto out;

	if (alrm->enabled)
		lp_cr |= (SRTC_LPCR_ALP | SRTC_LPCR_WAE);
	else
		lp_cr &= ~(SRTC_LPCR_ALP | SRTC_LPCR_WAE);

	if (lp_cr & SRTC_LPCR_ALL_INT_EN) {
		if (!pdata->irq_enable) {
			enable_irq(pdata->irq);
			pdata->irq_enable = true;
		}
	} else {
		if (pdata->irq_enable) {
			disable_irq(pdata->irq);
			pdata->irq_enable = false;
		}
	}

	__raw_writel(lp_cr, ioaddr + SRTC_LPCR);

out:
	spin_unlock_irqrestore(&rtc_lock, lock_flags);
	rtc_write_sync_lp(ioaddr);
	return ret;
}

/*!
 * This function is used to provide the content for the /proc/driver/rtc
 * file.
 *
 * @param  seq  buffer to hold the information that the driver wants to write
 *
 * @return  The number of bytes written into the rtc file.
 */
static int mxc_rtc_proc(struct device *dev, struct seq_file *seq)
{
	struct rtc_drv_data *pdata = dev_get_drvdata(dev);
	void __iomem *ioaddr = pdata->ioaddr;

	clk_enable(pdata->clk);
	seq_printf(seq, "alarm_IRQ\t: %s\n",
		   (((__raw_readl(ioaddr + SRTC_LPCR)) & SRTC_LPCR_ALP) !=
		    0) ? "yes" : "no");
	clk_disable(pdata->clk);

	return 0;
}

/*!
 * The RTC driver structure
 */
static struct rtc_class_ops mxc_rtc_ops = {
	.open = mxc_rtc_open,
	.release = mxc_rtc_release,
	.ioctl = mxc_rtc_ioctl,
	.read_time = mxc_rtc_read_time,
	.set_time = mxc_rtc_set_time,
	.read_alarm = mxc_rtc_read_alarm,
	.set_alarm = mxc_rtc_set_alarm,
	.proc = mxc_rtc_proc,
};

/*! MXC RTC Power management control */

static int mxc_rtc_probe(struct platform_device *pdev)
{
	struct clk *clk;
	struct timespec tv;
	struct resource *res;
	struct rtc_device *rtc;
	struct rtc_drv_data *pdata = NULL;
	struct mxc_srtc_platform_data *plat_data = NULL;
	void __iomem *ioaddr;
	int ret = 0;

	__dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->clk = clk_get(&pdev->dev, "rtc_clk");
	clk_enable(pdata->clk);
	pdata->baseaddr = res->start;
	pdata->ioaddr = ioremap(pdata->baseaddr, 0x40);
	ioaddr = pdata->ioaddr;

	/* Configure and enable the RTC */
	pdata->irq = platform_get_irq(pdev, 0);
	if (pdata->irq >= 0) {
		if (request_irq(pdata->irq, mxc_rtc_interrupt, IRQF_SHARED,
				pdev->name, pdev) < 0) {
			dev_warn(&pdev->dev, "interrupt not available.\n");
			pdata->irq = -1;
		} else {
			disable_irq(pdata->irq);
			pdata->irq_enable = false;
		}
	}

	clk = clk_get(NULL, "rtc_clk");
	if (clk_get_rate(clk) != 32768) {
		printk(KERN_ALERT "rtc clock is not valid");
		ret = -EINVAL;
		clk_put(clk);
		goto err_out;
	}
	clk_put(clk);	

	/* initialize glitch detect */
	__raw_writel(SRTC_LPPDR_INIT, ioaddr + SRTC_LPPDR);
	udelay(100);

	/* clear lp interrupt status */
	__raw_writel(0xFFFFFFFF, ioaddr + SRTC_LPSR);
	udelay(100);

	plat_data = (struct mxc_srtc_platform_data *)pdev->dev.platform_data;

	/* move out of init state */
	__raw_writel((SRTC_LPCR_IE | SRTC_LPCR_NSA),
		     ioaddr + SRTC_LPCR);

	udelay(100);

	while ((__raw_readl(ioaddr + SRTC_LPSR) & SRTC_LPSR_IES) == 0)
		;

	/* move out of non-valid state */
	__raw_writel((SRTC_LPCR_IE | SRTC_LPCR_NVE | SRTC_LPCR_NSA |
		      SRTC_LPCR_EN_LP), ioaddr + SRTC_LPCR);

	udelay(100);

	while ((__raw_readl(ioaddr + SRTC_LPSR) & SRTC_LPSR_NVES) == 0)
		;

	__raw_writel(0xFFFFFFFF, ioaddr + SRTC_LPSR);
	udelay(100);

	rtc = rtc_device_register(pdev->name, &pdev->dev,
				  &mxc_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		ret = PTR_ERR(rtc);
		goto err_out;
	}

	pdata->rtc = rtc;
	platform_set_drvdata(pdev, pdata);

	tv.tv_nsec = 0;
	tv.tv_sec = __raw_readl(ioaddr + SRTC_LPSCMR);

	/* By default, devices should wakeup if they can */
	/* So srtc is set as "should wakeup" as it can */
	device_init_wakeup(&pdev->dev, 1);

	clk_disable(pdata->clk);

	printk(KERN_INFO "Probing mxc_rtc done\n");

	if (device_create_file(&pdev->dev, &dev_attr_wakeup_enable) < 0)
		printk (KERN_ERR "Error - could not create RTC sysfs entry\n");

	device_init_wakeup(&pdev->dev, 1);
	schedule_delayed_work(&rtc_work, msecs_to_jiffies(RTC_TIMER_THRESHOLD));

	if (device_create_file(&pdev->dev, &dev_attr_rtc_pmic_epoch_time) < 0)
		printk (KERN_ERR "mxc_rtc: error creating rtc_pmic_epoch_time entry\n");


	if (device_create_file(&pdev->dev, &dev_attr_rtc_saved_last_seconds) < 0)
		printk (KERN_ERR "mxc_rtc: error creating rtc_saved_last_seconds entry\n");

	if (device_create_file(&pdev->dev, &dev_attr_wakeup_from_halt) < 0)
		printk (KERN_ERR "Error - could not create rtc_wakeup_from_halt entry\n");

	return ret;

err_out:
	clk_disable(pdata->clk);
	iounmap(ioaddr);
	if (pdata->irq >= 0)
		free_irq(pdata->irq, pdev);
	kfree(pdata);
	return ret;
}

static int __exit mxc_rtc_remove(struct platform_device *pdev)
{
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);
	rtc_device_unregister(pdata->rtc);
	if (pdata->irq >= 0)
		free_irq(pdata->irq, pdev);

	clk_disable(pdata->clk);
	clk_put(pdata->clk);
	kfree(pdata);

	rtc_suspended = 1;
	cancel_rearming_delayed_work(&rtc_work);

	device_remove_file(&pdev->dev, &dev_attr_wakeup_enable);
	device_remove_file(&pdev->dev, &dev_attr_rtc_pmic_epoch_time);
	device_remove_file(&pdev->dev, &dev_attr_rtc_saved_last_seconds);
	device_remove_file(&pdev->dev, &dev_attr_wakeup_from_halt);

	return 0;
}

unsigned long suspend_time = 0;
unsigned long last_suspend_time = 0;
unsigned long total_suspend_time = 0;

/*!
 * This function is called to save the system time delta relative to
 * the MXC RTC when enterring a low power state. This time delta is
 * then used on resume to adjust the system time to account for time
 * loss while suspended.
 *
 * @param   pdev  not used
 * @param   state Power state to enter.
 *
 * @return  The function always returns 0.
 */
static int mxc_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);
	struct timeval tmp;

	rtc_suspended = 1;
	
	pmic_rtc_get_time(&tmp);
	suspend_time = tmp.tv_sec;

	if (wakeup_value != 0) {
		if (device_may_wakeup(&pdev->dev))
			enable_irq_wake(platform_get_irq(pdev, 0));
		else {
			if (pdata->irq_enable)
				disable_irq(pdata->irq);
		}
	}
	else {
		if (pdata->irq_enable)
			disable_irq(pdata->irq);
	}

	return 0;
}

/*!
 * This function is called to correct the system time based on the
 * current MXC RTC time relative to the time delta saved during
 * suspend.
 *
 * @param   pdev  not used
 *
 * @return  The function always returns 0.
 */
static int mxc_rtc_resume(struct platform_device *pdev)
{
	struct rtc_drv_data *pdata = platform_get_drvdata(pdev);
	unsigned long resume_time = 0;
	struct timeval tmp;

	rtc_suspended = 0;

	if (wakeup_value != 0) {
		wakeup_value = 0;
		if (device_may_wakeup(&pdev->dev))
			disable_irq_wake(platform_get_irq(pdev, 0));
		else {
			if (pdata->irq_enable)
				enable_irq(pdata->irq);
		}
	}
	else {
		if (pdata->irq_enable)
			enable_irq(pdata->irq);
	}

	pmic_rtc_get_time(&tmp);
	resume_time = tmp.tv_sec;

	last_suspend_time = resume_time - suspend_time;
	total_suspend_time += last_suspend_time;
	schedule_delayed_work(&rtc_work, msecs_to_jiffies(RTC_TIMER_THRESHOLD));

	return 0;
}

/*!
 * Contains pointers to the power management callback functions.
 */
static struct platform_driver mxc_rtc_driver = {
	.driver = {
		   .name = "mxc_rtc",
		   },
	.probe = mxc_rtc_probe,
	.remove = __exit_p(mxc_rtc_remove),
	.suspend = mxc_rtc_suspend,
	.resume = mxc_rtc_resume,
};

/*!
 * This function creates the /proc/driver/rtc file and registers the device RTC
 * in the /dev/misc directory. It also reads the RTC value from external source
 * and setup the internal RTC properly.
 *
 * @return  -1 if RTC is failed to initialize; 0 is successful.
 */
static int __init mxc_rtc_init(void)
{
	return platform_driver_register(&mxc_rtc_driver);
}

/*!
 * This function removes the /proc/driver/rtc file and un-registers the
 * device RTC from the /dev/misc directory.
 */
static void __exit mxc_rtc_exit(void)
{
	platform_driver_unregister(&mxc_rtc_driver);

}

module_init(mxc_rtc_init);
module_exit(mxc_rtc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
