/*
 * Copyright 2010-2011 Amazon.com, Inc. All rights reserved.
 * Manish Lachwani (lachwani@lab126.com)
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
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pmic_external.h>
#include <linux/regulator/consumer.h>
#include <linux/sysdev.h>

#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <asm/proc-fns.h>
#include <asm/system.h>
#include "crm_regs.h"

#include <linux/syscalls.h>
#include <linux/pmic_rtc.h>

/*!
 * @defgroup MSL_MX51 i.MX51 Machine Specific Layer (MSL)
 */

/*!
 * @file mach-mx51/system.c
 * @brief This file contains idle and reset functions.
 *
 * @ingroup MSL_MX51
 */

extern int mxc_jtag_enabled;
extern int iram_ready;
extern void __iomem *ccm_base;
extern void __iomem *databahn_base;
extern void (*wait_in_iram)(void *ccm_addr, void *databahn_addr);
extern void mx50_wait(u32 ccm_base, u32 databahn_addr);
extern int low_bus_freq_mode;
extern void *wait_in_iram_base;
extern void __iomem *apll_base;
extern void __iomem *arm_plat_base;

static struct clk *gpc_dvfs_clk;
static struct clk *ddr_clk;
static struct clk *pll1_sw_clk;
static struct clk *osc;
static struct clk *pll1_main_clk;
static struct clk *ddr_clk ;

/* set cpu low power mode before WFI instruction */
void mxc_cpu_lp_set(enum mxc_cpu_pwr_mode mode)
{
	u32 plat_lpc, arm_srpgcr, ccm_clpcr;
	u32 empgc0, empgc1;
	int stop_mode = 0;

	/* always allow platform to issue a deep sleep mode request */
	plat_lpc = __raw_readl(arm_plat_base + MXC_CORTEXA8_PLAT_LPC) &
 	    ~(MXC_CORTEXA8_PLAT_LPC_DSM);
	ccm_clpcr = __raw_readl(MXC_CCM_CLPCR) & ~(MXC_CCM_CLPCR_LPM_MASK);
	arm_srpgcr = __raw_readl(MXC_SRPG_ARM_SRPGCR) & ~(MXC_SRPGCR_PCR);
	empgc0 = __raw_readl(MXC_SRPG_EMPGC0_SRPGCR) & ~(MXC_SRPGCR_PCR);
	empgc1 = __raw_readl(MXC_SRPG_EMPGC1_SRPGCR) & ~(MXC_SRPGCR_PCR);

	switch (mode) {
	case WAIT_CLOCKED:
		break;
	case WAIT_UNCLOCKED:
		ccm_clpcr |= (0x1 << MXC_CCM_CLPCR_LPM_OFFSET);
		break;
	case WAIT_UNCLOCKED_POWER_OFF:
	case STOP_POWER_OFF:
		plat_lpc |= MXC_CORTEXA8_PLAT_LPC_DSM
			    | MXC_CORTEXA8_PLAT_LPC_DBG_DSM;
		if (mode == WAIT_UNCLOCKED_POWER_OFF) {
			ccm_clpcr |= (0x1 << MXC_CCM_CLPCR_LPM_OFFSET);
			ccm_clpcr &= ~MXC_CCM_CLPCR_VSTBY;
			ccm_clpcr &= ~MXC_CCM_CLPCR_SBYOS;
			stop_mode = 0;
		} else {
			ccm_clpcr |= (0x2 << MXC_CCM_CLPCR_LPM_OFFSET);
			ccm_clpcr |= (0x3 << MXC_CCM_CLPCR_STBY_COUNT_OFFSET);
			ccm_clpcr |= MXC_CCM_CLPCR_VSTBY;
			ccm_clpcr |= MXC_CCM_CLPCR_SBYOS;
			stop_mode = 1;
		}

		arm_srpgcr |= MXC_SRPGCR_PCR;
		if (stop_mode) {
			empgc0 |= MXC_SRPGCR_PCR;
			empgc1 |= MXC_SRPGCR_PCR;
		}

		if (tzic_enable_wake(1) != 0)
			return;
		break;
	case STOP_POWER_ON:
		ccm_clpcr |= (0x2 << MXC_CCM_CLPCR_LPM_OFFSET);
		break;
	default:
		printk(KERN_WARNING "UNKNOWN cpu power mode: %d\n", mode);
		return;
	}

	__raw_writel(plat_lpc, arm_plat_base + MXC_CORTEXA8_PLAT_LPC);
	__raw_writel(ccm_clpcr, MXC_CCM_CLPCR);
	if (cpu_is_mx51() || cpu_is_mx50_rev(CHIP_REV_1_1) >= 1)
		__raw_writel(arm_srpgcr, MXC_SRPG_ARM_SRPGCR);
	if (cpu_is_mx50_rev(CHIP_REV_1_1) >= 1)
		__raw_writel(arm_srpgcr, MXC_SRPG_NEON_SRPGCR);
	if (stop_mode) {
		__raw_writel(empgc0, MXC_SRPG_EMPGC0_SRPGCR);
		__raw_writel(empgc1, MXC_SRPG_EMPGC1_SRPGCR);
	}
}

void mxc_pg_enable(struct platform_device *pdev)
{
	if (pdev == NULL)
		return;

	if (strcmp(pdev->name, "mxc_ipu") == 0) {
		__raw_writel(MXC_PGCR_PCR, MXC_PGC_IPU_PGCR);
		__raw_writel(MXC_PGSR_PSR, MXC_PGC_IPU_PGSR);
	} else if (strcmp(pdev->name, "mxc_vpu") == 0) {
		__raw_writel(MXC_PGCR_PCR, MXC_PGC_VPU_PGCR);
		__raw_writel(MXC_PGSR_PSR, MXC_PGC_VPU_PGSR);
	}
}

EXPORT_SYMBOL(mxc_pg_enable);

void mxc_pg_disable(struct platform_device *pdev)
{
	if (pdev == NULL)
		return;

	if (strcmp(pdev->name, "mxc_ipu") == 0) {
		__raw_writel(0x0, MXC_PGC_IPU_PGCR);
		if (__raw_readl(MXC_PGC_IPU_PGSR) & MXC_PGSR_PSR)
			dev_dbg(&pdev->dev, "power gating successful\n");
		__raw_writel(MXC_PGSR_PSR, MXC_PGC_IPU_PGSR);
	} else if (strcmp(pdev->name, "mxc_vpu") == 0) {
		__raw_writel(0x0, MXC_PGC_VPU_PGCR);
		if (__raw_readl(MXC_PGC_VPU_PGSR) & MXC_PGSR_PSR)
			dev_dbg(&pdev->dev, "power gating successful\n");
		__raw_writel(MXC_PGSR_PSR, MXC_PGC_VPU_PGSR);
	}
}

EXPORT_SYMBOL(mxc_pg_disable);

/* To change the idle power mode, need to set arch_idle_mode to a different
 * power mode as in enum mxc_cpu_pwr_mode.
 * May allow dynamically changing the idle mode.
 */
static int arch_idle_mode = WAIT_UNCLOCKED_POWER_OFF;
static int arch_idle_mode_audio = WAIT_UNCLOCKED;

/* Audio playing flag */
int mx50_audio_playing_flag = 0;
EXPORT_SYMBOL(mx50_audio_playing_flag);

/*
 * sysfs entries for the codec
 */
static ssize_t cpu_idle_store(struct sys_device *dev, struct sysdev_attribute *attr, 
				const char *buf, size_t size)
{
	int value = 0;

	if (sscanf(buf, "%d", &value) <= 0) {
		printk(KERN_ERR "Could not create cpu_idle entry\n");
		return -EINVAL;
	}

	mx50_audio_playing_flag = value;
	
	return size;
}

static ssize_t cpu_idle_show(struct sys_device *dev, struct sysdev_attribute *attr, char *buf)
{
	char *curr = buf;

	curr += sprintf(curr, "cpu_idle_flag: %d\n", mx50_audio_playing_flag);
	curr += sprintf(curr, "\n");

	return curr - buf;
}

static SYSDEV_ATTR(cpu_idle, 0644, cpu_idle_show, cpu_idle_store);

static struct sysdev_class cpu_idle_sysclass = {
	.name = "cpu_idle",
};

static struct sys_device device_cpu_idle = {
	.id	= 0,
	.cls	= &cpu_idle_sysclass,
};

/*!
 * This function puts the CPU into idle mode. It is called by default_idle()
 * in process.c file.
 */
void arch_idle(void)
{
	/* gpc clock is needed for SRPG */
	clk_enable(gpc_dvfs_clk);

	if (mx50_audio_playing_flag)
		mxc_cpu_lp_set(arch_idle_mode_audio);
	else
		mxc_cpu_lp_set(arch_idle_mode);

	if (!mx50_audio_playing_flag && cpu_is_mx50() && (clk_get_usecount(ddr_clk) == 0)) {
		memcpy(wait_in_iram_base, mx50_wait, SZ_4K);
		wait_in_iram = (void *)wait_in_iram_base;
                       if (low_bus_freq_mode) {
                               u32 reg, cpu_podf;

                               reg = __raw_readl(apll_base + 0x50);
                               reg = 0x120490;
                               __raw_writel(reg, apll_base + 0x50);
                               reg = __raw_readl(apll_base + 0x80);
                               reg |= 1;
                               __raw_writel(reg, apll_base + 0x80);

                               clk_set_parent(pll1_sw_clk, osc);
                               /* Set the ARM-PODF divider to 1. */
                               cpu_podf = __raw_readl(MXC_CCM_CACRR);
                               __raw_writel(0x01, MXC_CCM_CACRR);

                               wait_in_iram(ccm_base, databahn_base);

                               /* Set the ARM-POD divider back
                                * to the original.
                                */
                               __raw_writel(cpu_podf, MXC_CCM_CACRR);
                               clk_set_parent(pll1_sw_clk, pll1_main_clk);
                       } else
                               wait_in_iram(ccm_base, databahn_base);
	} else
		cpu_do_idle();

	clk_disable(gpc_dvfs_clk);
}

void __init mx50_init_idle_clocks(void)
{
	gpc_dvfs_clk = clk_get(NULL, "gpc_dvfs_clk");
	ddr_clk = clk_get(NULL, "ddr_clk");
	pll1_sw_clk = clk_get(NULL, "pll1_sw_clk");
	osc = clk_get(NULL, "lp_apm");
	pll1_main_clk = clk_get(NULL, "pll1_main_clk");
}


static int __init mxc_cpu_idle_sysfs(void)
{
	int error = 0;

	error = sysdev_class_register(&cpu_idle_sysclass);

	if (!error)
		error = sysdev_register(&device_cpu_idle);

	if (!error)
		error = sysdev_create_file(&device_cpu_idle, &attr_cpu_idle);

	return 0;
}
late_initcall(mxc_cpu_idle_sysfs);

#ifdef CONFIG_MXC_PMIC_MC34708
static void configure_rtc_mc34708(void)
{
	struct timeval pmic_time;

	pmic_write_reg(REG_MC34708_INT_STATUS1, 0, 0x2);
	pmic_write_reg(REG_MC34708_INT_MASK1, 0, 0x2);

	/* Zero out the RTC alarm day */
	pmic_write_reg(REG_MC34708_RTC_ALARM, 0x0, 0xffffffff);
	
	/* Zero out the RTC alarm time */
	pmic_write_reg(REG_MC34708_RTC_DAY_ALARM, 0x0, 0xffffffff);

	pmic_rtc_get_time(&pmic_time);
	pmic_time.tv_sec += 5;
	pmic_rtc_set_time_alarm(&pmic_time);
}
#endif

#ifdef CONFIG_MXC_PMIC_MC13892
static void configure_rtc(void)
{
	struct timeval pmic_time;

	pmic_write_reg(REG_INT_STATUS1, 0, 0x2);
	pmic_write_reg(REG_INT_MASK1, 0, 0x2);
	/* Zero out the RTC alarm day */
        pmic_write_reg(REG_RTC_ALARM, 0x0, 0xffffffff);

        /* Zero out the RTC alarm time */
        pmic_write_reg(REG_RTC_DAY_ALARM, 0x0, 0xffffffff);

	pmic_rtc_get_time(&pmic_time);

	pmic_time.tv_sec += 5;

	pmic_rtc_set_time_alarm(&pmic_time);
}
#endif

extern void gpio_watchdog_reboot(void);

/*
 * This function resets the system. It is called by machine_restart().
 *
 * @param  mode         indicates different kinds of resets
 */
void arch_reset(char mode)
{
	u32 reg;

	/* Turn on MMC clock */
	reg = __raw_readl(MXC_CCM_CCGR3);
	reg |= MXC_CCM_CCGR_CG_MASK << MXC_CCM_CCGR3_CG5_OFFSET;
	__raw_writel(reg, MXC_CCM_CCGR3);

#ifdef CONFIG_MXC_PMIC_MC13892
	configure_rtc();

	//see also mx50_yoshi.c, ../../drivers/rtc-mxc_v2.c, ../../drivers/arcotg_udc.c
	pmic_write_reg(REG_MEM_A, (1 << 1), (1 << 1));	
#endif

#ifdef CONFIG_MXC_PMIC_MC34708
	configure_rtc_mc34708();

	pmic_write_reg(REG_MC34708_MEM_A, (1 << 1), (1 << 1));
#endif
	gpio_watchdog_reboot();
}

void eink_doze_enable(int enable)
{
    return;
}
EXPORT_SYMBOL(eink_doze_enable);

void doze_disable(void)
{
    return;
}
EXPORT_SYMBOL(doze_disable);

void doze_enable(void)
{
    return;
	while (1) {
		/* Do Nothing */
	}
}
EXPORT_SYMBOL(doze_enable);

EXPORT_SYMBOL(sys_open);
EXPORT_SYMBOL(sys_write);
EXPORT_SYMBOL(sys_read);

#include <llog.h>
unsigned long einkfb_logging = _LLOG_LEVEL_DEFAULT;
EXPORT_SYMBOL(einkfb_logging);
