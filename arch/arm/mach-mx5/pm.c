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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/iram_alloc.h>
#include <asm/cacheflush.h>
#include <asm/tlb.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>

#define MXC_SRPG_EMPGC0_SRPGCR	(IO_ADDRESS(GPC_BASE_ADDR) + 0x2C0)
#define MXC_SRPG_EMPGC1_SRPGCR	(IO_ADDRESS(GPC_BASE_ADDR) + 0x2D0)
#define DATABAHN_CTL_REG0			0
#define DATABAHN_CTL_REG19		0x4c
#define DATABAHN_CTL_REG79		0x13c
#define DATABAHN_PHY_REG25		0x264

static struct cpu_wp *cpu_wp_tbl;
static struct clk *cpu_clk;

static int databahn_mode;

void __iomem *pll1_base;
void __iomem *pm_ccm_base;

#if defined(CONFIG_CPU_FREQ)
static int org_freq;
extern int cpufreq_suspended;
extern int set_cpu_freq(int wp);
extern int set_high_bus_freq(int high_bus_speed);

#define MX50_SUSPEND_FREQ		160000000
#endif


static struct device *pm_dev;
struct clk *gpc_dvfs_clk;
extern void cpu_do_suspend_workaround(u32 sdclk_iomux_addr);
extern void mx50_suspend(u32 databahn_addr);
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void __iomem *ccm_base;
extern void __iomem *databahn_base;
extern int accessory_port_suspend(void);
extern int accessory_port_resume(void);

extern int iram_ready;
void *suspend_iram_base;
void (*suspend_in_iram)(void *param1, void *param2, void* param3) = NULL;
void __iomem *suspend_param1;

extern void gpio_ssi_rxc_enable(int enable);

static int mx5_suspend_enter(suspend_state_t state)
{
	/* gpc clock is needed for SRPG */
	clk_enable(gpc_dvfs_clk);
	switch (state) {
	case PM_SUSPEND_MEM:
		mxc_cpu_lp_set(STOP_POWER_OFF);
		break;
	case PM_SUSPEND_STANDBY:
		mxc_cpu_lp_set(WAIT_UNCLOCKED_POWER_OFF);
		break;
	default:
		return -EINVAL;
	}

	if (tzic_enable_wake(0) != 0)
		return -EAGAIN;

	/* Turn off the SSI_RXC */
	gpio_ssi_rxc_enable(0);

	if (state == PM_SUSPEND_MEM) {
		local_flush_tlb_all();
		flush_cache_all();

		/* Store the LPM mode of databanhn */
		databahn_mode = __raw_readl(
			databahn_base + DATABAHN_CTL_REG20);

		suspend_in_iram(databahn_base,
				ccm_base, pll1_base);

		/* Restore the LPM databahn_mode. */
		__raw_writel(databahn_mode,
			databahn_base + DATABAHN_CTL_REG20);
		/*clear the EMPGC0/1 bits */
		__raw_writel(0, MXC_SRPG_EMPGC0_SRPGCR);
		__raw_writel(0, MXC_SRPG_EMPGC1_SRPGCR);

	} else {
			cpu_do_idle();
	}
	clk_disable(gpc_dvfs_clk);

	/* Turn on the SSI_RXC */
	gpio_ssi_rxc_enable(1);
	mdelay(2);

	return 0;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mx5_suspend_prepare(suspend_state_t state)
{
#if defined(CONFIG_CPU_FREQ)
	struct cpufreq_freqs freqs;
	org_freq = clk_get_rate(cpu_clk);
	freqs.old = org_freq / 1000;
	freqs.new = cpu_wp_tbl[0].cpu_rate / 1000;
	freqs.cpu = 0;
	freqs.flags = 0;

	cpufreq_suspended = 1;
	if (clk_get_rate(cpu_clk) != MX50_SUSPEND_FREQ) {
		set_cpu_freq(MX50_SUSPEND_FREQ);
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
#endif

	accessory_port_suspend();

	return 0;
}

/*
 * Called before devices are re-setup.
 */
static void mx5_suspend_finish(void)
{
#if defined(CONFIG_CPU_FREQ)
	struct cpufreq_freqs freqs;

	freqs.old = clk_get_rate(cpu_clk) / 1000;
	freqs.new = org_freq / 1000;
	freqs.cpu = 0;
	freqs.flags = 0;

	cpufreq_suspended = 0;
#endif

	accessory_port_resume();

}

static int mx5_pm_valid(suspend_state_t state)
{
	return (state > PM_SUSPEND_ON && state <= PM_SUSPEND_MAX);
}

struct platform_suspend_ops mx5_suspend_ops = {
	.valid = mx5_pm_valid,
	.begin = mx5_suspend_prepare,
	.enter = mx5_suspend_enter,
	.end = mx5_suspend_finish,
};


static int __devinit mx5_pm_probe(struct platform_device *pdev)
{
	pm_dev = &pdev->dev;
	return 0;
}

static struct platform_driver mx5_pm_driver = {
	.driver = {
		   .name = "mx5_pm",
		   },
	.probe = mx5_pm_probe,
};

static int __init pm_init(void)
{
	int cpu_wp_nr;
	unsigned long iram_paddr;

	pr_info("Static Power Management for Freescale i.MX5\n");
	if (platform_driver_register(&mx5_pm_driver) != 0) {
		printk(KERN_ERR "mx5_pm_driver register failed\n");
		return -ENODEV;
	}

	pll1_base = ioremap(MX53_BASE_ADDR(PLL1_BASE_ADDR), SZ_4K);

	suspend_set_ops(&mx5_suspend_ops);
	/* Move suspend routine into iRAM */
	iram_alloc(SZ_4K, &iram_paddr);
	/* Need to remap the area here since we want the memory region
		 to be executable. */
	suspend_iram_base = __arm_ioremap(iram_paddr, SZ_4K,
					  MT_HIGH_VECTORS);

	if (cpu_is_mx51()) {
		suspend_param1 = IO_ADDRESS(IOMUXC_BASE_ADDR + 0x4b8);
		memcpy(suspend_iram_base, cpu_do_suspend_workaround,
				SZ_4K);
	} else if (cpu_is_mx50()) {
		/*
		 * Need to run the suspend code from IRAM as the DDR needs
		 * to be put into self refresh mode manually.
		 */
		memcpy(suspend_iram_base, mx50_suspend, SZ_4K);

		suspend_param1 = databahn_base;
	}
	suspend_in_iram = (void *)suspend_iram_base;

	cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);

	cpu_clk = clk_get(NULL, "cpu_clk");
	if (IS_ERR(cpu_clk)) {
		printk(KERN_DEBUG "%s: failed to get cpu_clk\n", __func__);
		return PTR_ERR(cpu_clk);
	}
	printk(KERN_INFO "PM driver module loaded\n");

	if (gpc_dvfs_clk == NULL)
		gpc_dvfs_clk = clk_get(NULL, "gpc_dvfs_clk");

	return 0;
}


static void __exit pm_cleanup(void)
{
	/* Unregister the device structure */
	platform_driver_unregister(&mx5_pm_driver);
}

module_init(pm_init);
module_exit(pm_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("PM driver");
MODULE_LICENSE("GPL");
