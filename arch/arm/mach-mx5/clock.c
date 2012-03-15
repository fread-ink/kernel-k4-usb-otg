/*
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/init.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <asm/div64.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/clock.h>
#include <mach/mxc_dvfs.h>
#include <mach/sdram_autogating.h>

#include "crm_regs.h"

static struct clk pll1_main_clk;
static struct clk pll1_sw_clk;
static struct clk pll2_sw_clk;
static struct clk pll3_sw_clk;
static struct clk pll4_sw_clk;
static struct clk lp_apm_clk;
static struct clk tve_clk;
static struct clk emi_fast_clk;
static struct clk emi_slow_clk;
static struct clk emi_intr_clk[];
static struct clk ddr_clk;
static struct clk ipu_clk[];
static struct clk ldb_di_clk[];
static struct clk axi_a_clk;
static struct clk axi_b_clk;
static struct clk ddr_hf_clk;
static struct clk mipi_hsp_clk;
static struct clk gpu3d_clk;
static struct clk gpu2d_clk;
static struct clk vpu_clk[];
static int cpu_curr_wp;
static struct cpu_wp *cpu_wp_tbl;

static void __iomem *pll1_base;
static void __iomem *pll2_base;
static void __iomem *pll3_base;
static void __iomem *pll4_base;

extern int cpu_wp_nr;
extern int lp_high_freq;
extern int lp_med_freq;
int max_axi_a_clk;
int max_axi_b_clk;


#define SPIN_DELAY	1000000 /* in nanoseconds */
#define MAX_AXI_A_CLK_MX51 	166250000
#define MAX_AXI_A_CLK_MX53 	400000000
#define MAX_AXI_B_CLK_MX51 	133000000
#define MAX_AXI_B_CLK_MX53 	200000000
#define MAX_AHB_CLK			133000000
#define MAX_EMI_SLOW_CLK		133000000
#define MAX_DDR_HF_RATE		200000000

extern int mxc_jtag_enabled;
extern int uart_at_24;
extern int cpufreq_trig_needed;
extern int low_bus_freq_mode;

static int cpu_clk_set_wp(int wp);
extern void propagate_rate(struct clk *tclk);
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);

static struct clk esdhc3_clk[];

static void __calc_pre_post_dividers(u32 div, u32 *pre, u32 *post)
{
	u32 min_pre, temp_pre, old_err, err;

	if (div >= 512) {
		*pre = 8;
		*post = 64;
	} else if (div >= 8) {
		min_pre = (div - 1) / 64 + 1;
		old_err = 8;
		for (temp_pre = 8; temp_pre >= min_pre; temp_pre--) {
			err = div % temp_pre;
			if (err == 0) {
				*pre = temp_pre;
				break;
			}
			err = temp_pre - err;
			if (err < old_err) {
				old_err = err;
				*pre = temp_pre;
			}
		}
		*post = (div + *pre - 1) / *pre;
	} else if (div < 8) {
		*pre = div;
		*post = 1;
	}
}

static int _clk_enable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg |= MXC_CCM_CCGR_CG_MASK << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);

	if (clk->flags & AHB_HIGH_SET_POINT)
		lp_high_freq++;
	else if (clk->flags & AHB_MED_SET_POINT)
		lp_med_freq++;

	return 0;
}

static int _clk_enable_inrun(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGR_CG_MASK << clk->enable_shift);
	reg |= 1 << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);
	return 0;
}

static void _clk_disable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGR_CG_MASK << clk->enable_shift);
	__raw_writel(reg, clk->enable_reg);

	if (clk->flags & AHB_HIGH_SET_POINT)
		lp_high_freq--;
	else if (clk->flags & AHB_MED_SET_POINT)
		lp_med_freq--;
}

static void _clk_disable_inwait(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(MXC_CCM_CCGR_CG_MASK << clk->enable_shift);
	reg |= 1 << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);
}

/*
 * For the 4-to-1 muxed input clock
 */
static inline u32 _get_mux(struct clk *parent, struct clk *m0,
			   struct clk *m1, struct clk *m2, struct clk *m3)
{
	if (parent == m0)
		return 0;
	else if (parent == m1)
		return 1;
	else if (parent == m2)
		return 2;
	else if (parent == m3)
		return 3;
	else
		BUG();

	return 0;
}

/*
 * For the ddr muxed input clock
 */
static inline u32 _get_mux_ddr(struct clk *parent, struct clk *m0,
			   struct clk *m1, struct clk *m2, struct clk *m3, struct clk *m4)
{
	if (parent == m0)
		return 0;
	else if (parent == m1)
		return 1;
	else if (parent == m2)
		return 2;
	else if (parent == m3)
		return 3;
	else if (parent == m4)
		return 4;
	else
		BUG();

	return 0;
}

static inline void __iomem *_get_pll_base(struct clk *pll)
{
	if (pll == &pll1_main_clk)
		return pll1_base;
	else if (pll == &pll2_sw_clk)
		return pll2_base;
	else if (pll == &pll3_sw_clk)
		return pll3_base;
	else if (pll == &pll4_sw_clk)
		return pll4_base;
	else
		BUG();

	return NULL;
}

static struct clk ckih_clk = {
	.name = "ckih",
	.flags = RATE_PROPAGATES,
};

static struct clk ckih2_clk = {
	.name = "ckih2",
	.flags = RATE_PROPAGATES,
};

static struct clk osc_clk = {
	.name = "osc",
	.flags = RATE_PROPAGATES,
};

static struct clk ckil_clk = {
	.name = "ckil",
	.flags = RATE_PROPAGATES,
};

static void _fpm_recalc(struct clk *clk)
{
	clk->rate = ckil_clk.rate * 512;
	if ((__raw_readl(MXC_CCM_CCR) & MXC_CCM_CCR_FPM_MULT_MASK) != 0)
		clk->rate *= 2;

}

static int _fpm_enable(struct clk *clk)
{
	u32 reg = __raw_readl(MXC_CCM_CCR);
	reg |= MXC_CCM_CCR_FPM_EN;
	__raw_writel(reg, MXC_CCM_CCR);
	return 0;
}

static void _fpm_disable(struct clk *clk)
{
	u32 reg = __raw_readl(MXC_CCM_CCR);
	reg &= ~MXC_CCM_CCR_FPM_EN;
	__raw_writel(reg, MXC_CCM_CCR);
}

static struct clk fpm_clk = {
	.name = "fpm_clk",
	.parent = &ckil_clk,
	.recalc = _fpm_recalc,
	.enable = _fpm_enable,
	.disable = _fpm_disable,
	.flags = RATE_PROPAGATES,
};

static void _fpm_div2_recalc(struct clk *clk)
{
	clk->rate = clk->parent->rate / 2;
}

static struct clk fpm_div2_clk = {
	.name = "fpm_div2_clk",
	.parent = &fpm_clk,
	.recalc = _fpm_div2_recalc,
	.flags = RATE_PROPAGATES,
};

static void _clk_pll_recalc(struct clk *clk)
{
	long mfi, mfn, mfd, pdf, ref_clk, mfn_abs;
	unsigned long dp_op, dp_mfd, dp_mfn, dp_ctl, pll_hfsm, dbl;
	void __iomem *pllbase;
	s64 temp;

	pllbase = _get_pll_base(clk);

	dp_ctl = __raw_readl(pllbase + MXC_PLL_DP_CTL);
	pll_hfsm = dp_ctl & MXC_PLL_DP_CTL_HFSM;
	dbl = dp_ctl & MXC_PLL_DP_CTL_DPDCK0_2_EN;

	if (pll_hfsm == 0) {
		dp_op = __raw_readl(pllbase + MXC_PLL_DP_OP);
		dp_mfd = __raw_readl(pllbase + MXC_PLL_DP_MFD);
		dp_mfn = __raw_readl(pllbase + MXC_PLL_DP_MFN);
	} else {
		dp_op = __raw_readl(pllbase + MXC_PLL_DP_HFS_OP);
		dp_mfd = __raw_readl(pllbase + MXC_PLL_DP_HFS_MFD);
		dp_mfn = __raw_readl(pllbase + MXC_PLL_DP_HFS_MFN);
	}
	pdf = dp_op & MXC_PLL_DP_OP_PDF_MASK;
	mfi = (dp_op & MXC_PLL_DP_OP_MFI_MASK) >> MXC_PLL_DP_OP_MFI_OFFSET;
	mfi = (mfi <= 5) ? 5 : mfi;
	mfd = dp_mfd & MXC_PLL_DP_MFD_MASK;
	mfn = mfn_abs = dp_mfn & MXC_PLL_DP_MFN_MASK;
	/* Sign extend to 32-bits */
	if (mfn >= 0x04000000) {
		mfn |= 0xFC000000;
		mfn_abs = -mfn;
	}

	ref_clk = 2 * clk->parent->rate;
	if (dbl != 0)
		ref_clk *= 2;

	ref_clk /= (pdf + 1);
	temp = (u64) ref_clk * mfn_abs;
	do_div(temp, mfd + 1);
	if (mfn < 0)
		temp = -temp;
	temp = (ref_clk * mfi) + temp;

	clk->rate = temp;
}

static int _clk_pll_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, reg1;
	void __iomem *pllbase;
	struct timespec nstimeofday;
	struct timespec curtime;

	long mfi, pdf, mfn, mfd = 999999;
	s64 temp64;
	unsigned long quad_parent_rate;
	unsigned long pll_hfsm, dp_ctl;

	pllbase = _get_pll_base(clk);

	quad_parent_rate = 4*clk->parent->rate;
	pdf = mfi = -1;
	while (++pdf < 16 && mfi < 5)
		mfi = rate * (pdf+1) / quad_parent_rate;
	if (mfi > 15)
		return -1;
	pdf--;

	temp64 = rate*(pdf+1) - quad_parent_rate*mfi;
	do_div(temp64, quad_parent_rate/1000000);
	mfn = (long)temp64;

	dp_ctl = __raw_readl(pllbase + MXC_PLL_DP_CTL);
	/* use dpdck0_2 */
	__raw_writel(dp_ctl | 0x1000L, pllbase + MXC_PLL_DP_CTL);
	pll_hfsm = dp_ctl & MXC_PLL_DP_CTL_HFSM;
	if (pll_hfsm == 0) {
		reg = mfi<<4 | pdf;
		__raw_writel(reg, pllbase + MXC_PLL_DP_OP);
		__raw_writel(mfd, pllbase + MXC_PLL_DP_MFD);
		__raw_writel(mfn, pllbase + MXC_PLL_DP_MFN);
	} else {
		reg = mfi<<4 | pdf;
		__raw_writel(reg, pllbase + MXC_PLL_DP_HFS_OP);
		__raw_writel(mfd, pllbase + MXC_PLL_DP_HFS_MFD);
		__raw_writel(mfn, pllbase + MXC_PLL_DP_HFS_MFN);
	}
	/* If auto restart is disabled, restart the PLL and
	  * wait for it to lock.
	  */
	reg = __raw_readl(pllbase + MXC_PLL_DP_CTL);
	if (reg & MXC_PLL_DP_CTL_UPEN) {
		reg = __raw_readl(pllbase + MXC_PLL_DP_CONFIG);
		if (!(reg & MXC_PLL_DP_CONFIG_AREN)) {
			reg1 = __raw_readl(pllbase + MXC_PLL_DP_CTL);
			reg1 |= MXC_PLL_DP_CTL_RST;
			__raw_writel(reg1, pllbase + MXC_PLL_DP_CTL);
		}
		/* Wait for lock */
		getnstimeofday(&nstimeofday);
		while (!(__raw_readl(pllbase + MXC_PLL_DP_CTL)
					& MXC_PLL_DP_CTL_LRF)) {
			getnstimeofday(&curtime);
			if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
				panic("pll_set_rate: pll relock failed\n");
		}
	}
	clk->rate = rate;
	return 0;
}

static int _clk_pll_enable(struct clk *clk)
{
	u32 reg;
	void __iomem *pllbase;
	struct timespec nstimeofday;
	struct timespec curtime;

	pllbase = _get_pll_base(clk);
	reg = __raw_readl(pllbase + MXC_PLL_DP_CTL);

	if (reg & MXC_PLL_DP_CTL_UPEN)
		return 0;

	reg |=  MXC_PLL_DP_CTL_UPEN;
	__raw_writel(reg, pllbase + MXC_PLL_DP_CTL);

	/* Wait for lock */
	getnstimeofday(&nstimeofday);
	while (!(__raw_readl(pllbase + MXC_PLL_DP_CTL) & MXC_PLL_DP_CTL_LRF)) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("pll relock failed\n");
	}
	return 0;
}

static void _clk_pll_disable(struct clk *clk)
{
	u32 reg;
	void __iomem *pllbase;

	pllbase = _get_pll_base(clk);
	reg = __raw_readl(pllbase + MXC_PLL_DP_CTL) & ~MXC_PLL_DP_CTL_UPEN;
	__raw_writel(reg, pllbase + MXC_PLL_DP_CTL);
}

static struct clk pll1_main_clk = {
	.name = "pll1_main_clk",
	.parent = &osc_clk,
	.recalc = _clk_pll_recalc,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.flags = RATE_PROPAGATES,
};

static int _clk_pll1_sw_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CCSR);

	if (parent == &pll1_main_clk) {
		reg &= ~MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
		__raw_writel(reg, MXC_CCM_CCSR);
		/* Set the step_clk parent to be lp_apm, to save power. */
		mux = _get_mux(&lp_apm_clk, &lp_apm_clk, NULL, &pll2_sw_clk,
			       &pll3_sw_clk);
		reg = __raw_readl(MXC_CCM_CCSR);
		reg = (reg & ~MXC_CCM_CCSR_STEP_SEL_MASK) |
		    (mux << MXC_CCM_CCSR_STEP_SEL_OFFSET);
	} else {
		if (parent == &lp_apm_clk) {
			reg |= MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
			reg = __raw_readl(MXC_CCM_CCSR);
			mux = _get_mux(parent, &lp_apm_clk, NULL, &pll2_sw_clk,
				       &pll3_sw_clk);
			reg = (reg & ~MXC_CCM_CCSR_STEP_SEL_MASK) |
			    (mux << MXC_CCM_CCSR_STEP_SEL_OFFSET);
		} else {
			mux = _get_mux(parent, &lp_apm_clk, NULL, &pll2_sw_clk,
				       &pll3_sw_clk);
			reg = (reg & ~MXC_CCM_CCSR_STEP_SEL_MASK) |
			    (mux << MXC_CCM_CCSR_STEP_SEL_OFFSET);
			__raw_writel(reg, MXC_CCM_CCSR);
			reg = __raw_readl(MXC_CCM_CCSR);
			reg |= MXC_CCM_CCSR_PLL1_SW_CLK_SEL;

		}
	}
	__raw_writel(reg, MXC_CCM_CCSR);

	return 0;
}

static void _clk_pll1_sw_recalc(struct clk *clk)
{
	u32 reg, div;
	div = 1;
	reg = __raw_readl(MXC_CCM_CCSR);

	if (clk->parent == &pll2_sw_clk) {
		div = ((reg & MXC_CCM_CCSR_PLL2_PODF_MASK) >>
		       MXC_CCM_CCSR_PLL2_PODF_OFFSET) + 1;
	} else if (clk->parent == &pll3_sw_clk) {
		div = ((reg & MXC_CCM_CCSR_PLL3_PODF_MASK) >>
		       MXC_CCM_CCSR_PLL3_PODF_OFFSET) + 1;
	}
	clk->rate = clk->parent->rate / div;
}

/* pll1 switch clock */
static struct clk pll1_sw_clk = {
	.name = "pll1_sw_clk",
	.parent = &pll1_main_clk,
	.set_parent = _clk_pll1_sw_set_parent,
	.recalc = _clk_pll1_sw_recalc,
	.flags = RATE_PROPAGATES,
};

static int _clk_pll2_sw_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCSR);

	if (parent == &pll2_sw_clk) {
		reg &= ~MXC_CCM_CCSR_PLL2_SW_CLK_SEL;
	} else {
		reg = (reg & ~MXC_CCM_CCSR_PLL2_SW_CLK_SEL);
		reg |= MXC_CCM_CCSR_PLL2_SW_CLK_SEL;
	}
	__raw_writel(reg, MXC_CCM_CCSR);
	return 0;
}

/* same as pll2_main_clk. These two clocks should always be the same */
static struct clk pll2_sw_clk = {
	.name = "pll2",
	.parent = &osc_clk,
	.recalc = _clk_pll_recalc,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.set_rate = _clk_pll_set_rate,
	.set_parent = _clk_pll2_sw_set_parent,
	.flags = RATE_PROPAGATES,
};

/* same as pll3_main_clk. These two clocks should always be the same */
static struct clk pll3_sw_clk = {
	.name = "pll3",
	.parent = &osc_clk,
	.set_rate = _clk_pll_set_rate,
	.recalc = _clk_pll_recalc,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.flags = RATE_PROPAGATES,
};

/* same as pll4_main_clk. These two clocks should always be the same */
static struct clk pll4_sw_clk = {
	.name = "pll4",
	.parent = &osc_clk,
	.set_rate = _clk_pll_set_rate,
	.recalc = _clk_pll_recalc,
	.enable = _clk_pll_enable,
	.disable = _clk_pll_disable,
	.flags = RATE_PROPAGATES,
};

static int _clk_lp_apm_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	if (parent == &osc_clk)
		reg = __raw_readl(MXC_CCM_CCSR) & ~MXC_CCM_CCSR_LP_APM_SEL;
	else if (parent == &fpm_clk)
		reg = __raw_readl(MXC_CCM_CCSR) | MXC_CCM_CCSR_LP_APM_SEL;
	else
		return -EINVAL;

	__raw_writel(reg, MXC_CCM_CCSR);

	return 0;
}

static struct clk lp_apm_clk = {
	.name = "lp_apm",
	.parent = &osc_clk,
	.set_parent = _clk_lp_apm_set_parent,
	.flags = RATE_PROPAGATES,
};

static void _clk_arm_recalc(struct clk *clk)
{
	u32 cacrr, div;

	cacrr = __raw_readl(MXC_CCM_CACRR);
	div = (cacrr & MXC_CCM_CACRR_ARM_PODF_MASK) + 1;
	clk->rate = clk->parent->rate / div;
}

static int _clk_cpu_set_rate(struct clk *clk, unsigned long rate)
{
	u32 i;
	for (i = 0; i < cpu_wp_nr; i++) {
		if (rate == cpu_wp_tbl[i].cpu_rate)
			break;
	}
	if (i >= cpu_wp_nr)
		return -EINVAL;
	cpu_clk_set_wp(i);

	return 0;
}

static unsigned long _clk_cpu_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 i;
	u32 wp;

	for (i = 0; i < cpu_wp_nr; i++) {
		if (rate == cpu_wp_tbl[i].cpu_rate)
			break;
	}

	if (i > cpu_wp_nr)
		wp = 0;

	return cpu_wp_tbl[wp].cpu_rate;
}


static struct clk cpu_clk = {
	.name = "cpu_clk",
	.parent = &pll1_sw_clk,
	.recalc = _clk_arm_recalc,
	.set_rate = _clk_cpu_set_rate,
	.round_rate = _clk_cpu_round_rate,
};

static int _clk_periph_apm_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	struct timespec nstimeofday;
	struct timespec curtime;

	mux = _get_mux(parent, &pll1_sw_clk, &pll3_sw_clk, &lp_apm_clk, NULL);

	reg = __raw_readl(MXC_CCM_CBCMR) & ~MXC_CCM_CBCMR_PERIPH_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CBCMR_PERIPH_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCMR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) &
			MXC_CCM_CDHIPR_PERIPH_CLK_SEL_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("pll _clk_periph_apm_set_parent failed\n");
	}
	return 0;
}

static struct clk periph_apm_clk = {
	.name = "periph_apm_clk",
	.parent = &pll1_sw_clk,
	.set_parent = _clk_periph_apm_set_parent,
	.flags = RATE_PROPAGATES,
};

/* TODO: Need to sync with GPC to determine if DVFS is in place so that
 * the DVFS_PODF divider can be applied in CDCR register.
 */
static void _clk_main_bus_recalc(struct clk *clk)
{
	u32 div = 0;

	if (dvfs_per_divider_active() || low_bus_freq_mode)
		div  = (__raw_readl(MXC_CCM_CDCR) & 0x3);
	clk->rate = clk->parent->rate / (div + 1);
}

static int _clk_main_bus_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);

	if (parent == &pll2_sw_clk) {
		reg = __raw_readl(MXC_CCM_CBCDR) &
		    ~MXC_CCM_CBCDR_PERIPH_CLK_SEL;
	} else if (parent == &periph_apm_clk) {
		reg = __raw_readl(MXC_CCM_CBCDR) | MXC_CCM_CBCDR_PERIPH_CLK_SEL;
	} else {
		return -EINVAL;
	}
	__raw_writel(reg, MXC_CCM_CBCDR);
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);

	return 0;
}

static struct clk main_bus_clk = {
	.name = "main_bus_clk",
	.parent = &pll2_sw_clk,
	.set_parent = _clk_main_bus_set_parent,
	.recalc = _clk_main_bus_recalc,
	.flags = RATE_PROPAGATES,
};

static void _clk_axi_a_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_AXI_A_PODF_MASK) >>
	       MXC_CCM_CBCDR_AXI_A_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static int _clk_axi_a_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_AXI_A_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_AXI_A_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_AXI_A_PODF_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("pll _clk_axi_a_set_rate failed\n");
	}
	clk->rate = rate;
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);

	return 0;
}

static unsigned long _clk_axi_a_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;
	if (clk->parent->rate / div > max_axi_a_clk)
		div++;

	if (div > 8)
		div = 8;

	return clk->parent->rate / div;
}


static struct clk axi_a_clk = {
	.name = "axi_a_clk",
	.parent = &main_bus_clk,
	.recalc = _clk_axi_a_recalc,
	.set_rate = _clk_axi_a_set_rate,
	.round_rate = _clk_axi_a_round_rate,
	.flags = RATE_PROPAGATES,
};

static void _clk_ddr_hf_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_DDR_PODF_MASK) >>
	       MXC_CCM_CBCDR_DDR_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static unsigned long _clk_ddr_hf_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;
	if (clk->parent->rate / div > MAX_DDR_HF_RATE)
		div++;

	if (div > 8)
		div = 8;

	return clk->parent->rate / div;
}

static int _clk_ddr_hf_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_DDR_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_DDR_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_DDR_PODF_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("clk_ddr_hf_set_rate failed\n");
	}
	clk->rate = rate;
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);

	return 0;
}

static struct clk ddr_hf_clk = {
	.name = "ddr_hf_clk",
	.parent = &pll1_sw_clk,
	.recalc = _clk_ddr_hf_recalc,
	.round_rate = _clk_ddr_hf_round_rate,
	.set_rate = _clk_ddr_hf_set_rate,
	.flags = RATE_PROPAGATES,
};

static void _clk_axi_b_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_AXI_B_PODF_MASK) >>
	       MXC_CCM_CBCDR_AXI_B_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static int _clk_axi_b_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;

	emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_AXI_B_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_AXI_B_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_AXI_B_PODF_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("_clk_axi_b_set_rate failed\n");
	}

	clk->rate = rate;
	emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);

	return 0;
}

static unsigned long _clk_axi_b_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;
	if (clk->parent->rate / div > max_axi_b_clk)
		div++;

	if (div > 8)
		div = 8;

	return clk->parent->rate / div;
}


static struct clk axi_b_clk = {
	.name = "axi_b_clk",
	.parent = &main_bus_clk,
	.recalc = _clk_axi_b_recalc,
	.set_rate = _clk_axi_b_set_rate,
	.round_rate = _clk_axi_b_round_rate,
	.flags = RATE_PROPAGATES,
};

static void _clk_ahb_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_AHB_PODF_MASK) >>
	       MXC_CCM_CBCDR_AHB_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}


static int _clk_ahb_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);

	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_AHB_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_AHB_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);

	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_AHB_PODF_BUSY) {
		getnstimeofday(&curtime);
		if (curtime.tv_nsec - nstimeofday.tv_nsec > SPIN_DELAY)
			panic("_clk_ahb_set_rate failed\n");
	}
	clk->rate = rate;
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);

	return 0;
}

static unsigned long _clk_ahb_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;
	if (clk->parent->rate / div > MAX_AHB_CLK)
		div++;

	if (div > 8)
		div = 8;

	return clk->parent->rate / div;
}


static struct clk ahb_clk = {
	.name = "ahb_clk",
	.parent = &main_bus_clk,
	.recalc = _clk_ahb_recalc,
	.set_rate = _clk_ahb_set_rate,
	.round_rate = _clk_ahb_round_rate,
	.flags = RATE_PROPAGATES,
};

static int _clk_max_enable(struct clk *clk)
{
	u32 reg;

	_clk_enable(clk);

	/* Handshake with MAX when LPM is entered. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	if (cpu_is_mx51())
		reg &= ~MXC_CCM_CLPCR_BYPASS_MAX_LPM_HS_MX51;
	else
		reg &= ~MXC_CCM_CLPCR_BYPASS_MAX_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);

	return 0;
}


static void _clk_max_disable(struct clk *clk)
{
	u32 reg;

	_clk_disable_inwait(clk);

	/* No Handshake with MAX when LPM is entered as its disabled. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	if (cpu_is_mx51())
		reg |= MXC_CCM_CLPCR_BYPASS_MAX_LPM_HS_MX51;
	else
		reg |= MXC_CCM_CLPCR_BYPASS_MAX_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}


static struct clk ahb_max_clk = {
	.name = "max_clk",
	.parent = &ahb_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG14_OFFSET,
	.enable = _clk_max_enable,
	.disable = _clk_max_disable,
};

static int _clk_emi_slow_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);

	reg = __raw_readl(MXC_CCM_CBCDR);
	if (parent == &ahb_clk) {
		reg |= MXC_CCM_CBCDR_EMI_CLK_SEL;
	} else if (parent == &main_bus_clk) {
		reg &= ~MXC_CCM_CBCDR_EMI_CLK_SEL;
	} else {
		BUG();
	}
	__raw_writel(reg, MXC_CCM_CBCDR);

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);

	return 0;
}

static void _clk_emi_slow_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_EMI_PODF_MASK) >>
	       MXC_CCM_CBCDR_EMI_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static int _clk_emi_slow_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_EMI_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_EMI_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);
	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) & MXC_CCM_CDHIPR_EMI_PODF_BUSY) {
		getnstimeofday(&curtime);
		if ((curtime.tv_nsec - nstimeofday.tv_nsec) > SPIN_DELAY)
			panic("_clk_emi_slow_set_rate failed\n");
	}
	clk->rate = rate;

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	return 0;
}

static unsigned long _clk_emi_slow_round_rate(struct clk *clk,
					      unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;

	/* Make sure rate is not greater than the maximum value for the clock.
	 * Also prevent a div of 0.
	 */
	if (div == 0)
		div++;
	if (clk->parent->rate / div > MAX_EMI_SLOW_CLK)
		div++;

	if (div > 8)
		div = 8;

	return clk->parent->rate / div;
}


static struct clk emi_slow_clk = {
	.name = "emi_slow_clk",
	.parent = &main_bus_clk,
	.set_parent = _clk_emi_slow_set_parent,
	.recalc = _clk_emi_slow_recalc,
	.set_rate = _clk_emi_slow_set_rate,
	.round_rate = _clk_emi_slow_round_rate,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG8_OFFSET,
	.disable = _clk_disable_inwait,
	.flags = RATE_PROPAGATES,
};

static struct clk ahbmux1_clk = {
	.name = "ahbmux1_clk",
	.id = 0,
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG8_OFFSET,
	.disable = _clk_disable_inwait,
};

static struct clk ahbmux2_clk = {
	.name = "ahbmux2_clk",
	.id = 0,
	.parent = &ahb_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG9_OFFSET,
	.disable = _clk_disable_inwait,
};


static struct clk emi_fast_clk = {
	.name = "emi_fast_clk",
	.parent = &ddr_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG7_OFFSET,
	.disable = _clk_disable_inwait,
};

static struct clk emi_intr_clk[] = {
	{
	.name = "emi_intr_clk",
	.id = 0,
	.parent = &ahb_clk,
	.secondary = &ahbmux2_clk,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG9_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
	},
	{
	/* On MX51 - this clock is name emi_garb_clk, and controls the
	 * access of ARM to GARB.
	 */
	.name = "emi_intr_clk",
	.id = 1,
	.parent = &ahb_clk,
	.secondary = &ahbmux2_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG4_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
	}
};

static void _clk_ipg_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_IPG_PODF_MASK) >>
	       MXC_CCM_CBCDR_IPG_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static struct clk ipg_clk = {
	.name = "ipg_clk",
	.parent = &ahb_clk,
	.recalc = _clk_ipg_recalc,
	.flags = RATE_PROPAGATES,
};

static void _clk_ipg_per_recalc(struct clk *clk)
{
	u32 reg, prediv1, prediv2, podf;

	if (clk->parent == &main_bus_clk || clk->parent == &lp_apm_clk) {
		/* the main_bus_clk is the one before the DVFS engine */
		reg = __raw_readl(MXC_CCM_CBCDR);
		prediv1 = ((reg & MXC_CCM_CBCDR_PERCLK_PRED1_MASK) >>
			   MXC_CCM_CBCDR_PERCLK_PRED1_OFFSET) + 1;
		prediv2 = ((reg & MXC_CCM_CBCDR_PERCLK_PRED2_MASK) >>
			   MXC_CCM_CBCDR_PERCLK_PRED2_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CBCDR_PERCLK_PODF_MASK) >>
			MXC_CCM_CBCDR_PERCLK_PODF_OFFSET) + 1;
		clk->rate = clk->parent->rate / (prediv1 * prediv2 * podf);
	} else if (clk->parent == &ipg_clk) {
		clk->rate = ipg_clk.rate;
	} else {
		BUG();
	}
}

static int _clk_ipg_per_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &main_bus_clk, &lp_apm_clk, &ipg_clk, NULL);
	if (mux == 2) {
		reg |= MXC_CCM_CBCMR_PERCLK_IPG_CLK_SEL;
	} else {
		reg &= ~MXC_CCM_CBCMR_PERCLK_IPG_CLK_SEL;
		if (mux == 0)
			reg &= ~MXC_CCM_CBCMR_PERCLK_LP_APM_CLK_SEL;
		else
			reg |= MXC_CCM_CBCMR_PERCLK_LP_APM_CLK_SEL;
	}
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static struct clk ipg_perclk = {
	.name = "ipg_perclk",
	.parent = &lp_apm_clk,
	.recalc = _clk_ipg_per_recalc,
	.set_parent = _clk_ipg_per_set_parent,
	.flags = RATE_PROPAGATES,
};

static int _clk_ipmux_enable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg |= 1  << clk->enable_shift;
	__raw_writel(reg, clk->enable_reg);

	return 0;
}

static void _clk_ipmux_disable(struct clk *clk)
{
	u32 reg;
	reg = __raw_readl(clk->enable_reg);
	reg &= ~(0x1 << clk->enable_shift);
	__raw_writel(reg, clk->enable_reg);
}

static struct clk ipumux1_clk = {
	.name = "ipumux1",
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG6_1_OFFSET,
	.enable = _clk_ipmux_enable,
	.disable = _clk_ipmux_disable,
};

static struct clk ipumux2_clk = {
	.name = "ipumux2",
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG6_2_OFFSET,
	.enable = _clk_ipmux_enable,
	.disable = _clk_ipmux_disable,
};

static int _clk_ocram_enable(struct clk *clk)
{
	return 0;
}

static void _clk_ocram_disable(struct clk *clk)
{
}

static struct clk ocram_clk = {
	.name = "ocram_clk",
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG1_OFFSET,
	.enable = _clk_ocram_enable,
	.disable = _clk_ocram_disable,
};


static struct clk aips_tz1_clk = {
	.name = "aips_tz1_clk",
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static struct clk aips_tz2_clk = {
	.name = "aips_tz2_clk",
	.parent = &ahb_clk,
	.secondary = &ahb_max_clk,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable_inwait,
};

static struct clk gpc_dvfs_clk = {
	.name = "gpc_dvfs_clk",
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static int _clk_sdma_enable(struct clk *clk)
{
	u32 reg;

	_clk_enable(clk);

	/* Handshake with SDMA when LPM is entered. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	if (cpu_is_mx51())
		reg &= ~MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS_MX51;
	else
		reg &= ~MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);

	return 0;
}

static void _clk_sdma_disable(struct clk *clk)
{
	u32 reg;

	_clk_disable(clk);
	/* No handshake with SDMA as its not enabled. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	if (cpu_is_mx51())
		reg |= MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS_MX51;
	else
		reg |= MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}


static struct clk sdma_clk[] = {
	{
	 .name = "sdma_ahb_clk",
	 .parent = &ahb_clk,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG15_OFFSET,
	 .enable = _clk_sdma_enable,
	 .disable = _clk_sdma_disable,
	 },
	{
	 .name = "sdma_ipg_clk",
	 .parent = &ipg_clk,
#ifdef CONFIG_SDMA_IRAM
	 .secondary = &emi_intr_clk[0],
#endif
	 },
};

static int _clk_ipu_enable(struct clk *clk)
{
	u32 reg;

	_clk_enable(clk);
	/* Handshake with IPU when certain clock rates are changed. */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg &= ~MXC_CCM_CCDR_IPU_HS_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	/* Handshake with IPU when LPM is entered as its enabled. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg &= ~MXC_CCM_CLPCR_BYPASS_IPU_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);

	start_sdram_autogating();

	return 0;
}

static void _clk_ipu_disable(struct clk *clk)
{
	u32 reg;

	if (sdram_autogating_active())
		stop_sdram_autogating();

	_clk_disable(clk);

	/* No handshake with IPU whe dividers are changed
	 * as its not enabled. */
	reg = __raw_readl(MXC_CCM_CCDR);
	if (cpu_is_mx51())
		reg |= MXC_CCM_CCDR_IPU_HS_MASK;
	else
		reg |= MXC_CCM_CCDR_IPU_HS_MX53_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	/* No handshake with IPU when LPM is entered as its not enabled. */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_IPU_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}

static int _clk_ipu_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &ahb_clk,
		       &emi_slow_clk);
	reg = (reg & ~MXC_CCM_CBCMR_IPU_HSP_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CBCMR_IPU_HSP_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}


static struct clk ipu_clk[] = {
	{
	.name = "ipu_clk",
	.parent = &ahb_clk,
	.secondary = &ipu_clk[1],
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG5_OFFSET,
	.enable = _clk_ipu_enable,
	.disable = _clk_ipu_disable,
	.set_parent = _clk_ipu_set_parent,
	 .flags = CPU_FREQ_TRIG_UPDATE | AHB_MED_SET_POINT | RATE_PROPAGATES,
	},
	{
	 .name = "ipu_sec_clk",
	 .parent = &emi_fast_clk,
	 .secondary = &ahbmux1_clk,
	}
};

static int _clk_ipu_di_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	reg &= ~MXC_CCM_CSCMR2_DI_CLK_SEL_MASK(clk->id);
	if (parent == &pll3_sw_clk)
		;
	else if (parent == &osc_clk)
		reg |= 1 << MXC_CCM_CSCMR2_DI_CLK_SEL_OFFSET(clk->id);
	else if (parent == &ckih_clk)
		reg |= 2 << MXC_CCM_CSCMR2_DI_CLK_SEL_OFFSET(clk->id);
	else if ((parent == &pll4_sw_clk) && (clk->id == 0)) {
		if (cpu_is_mx51())
			return -EINVAL;
		reg |= 3 << MXC_CCM_CSCMR2_DI_CLK_SEL_OFFSET(clk->id);
	} else if ((parent == &tve_clk) && (clk->id == 1))
		reg |= 3 << MXC_CCM_CSCMR2_DI_CLK_SEL_OFFSET(clk->id);
	else if ((parent == &ldb_di_clk[clk->id]) && cpu_is_mx53())
		reg |= 5 << MXC_CCM_CSCMR2_DI_CLK_SEL_OFFSET(clk->id);
	else		/* Assume any other clock is external clock pin */
		reg |= 4 << MXC_CCM_CSCMR2_DI_CLK_SEL_OFFSET(clk->id);
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static void _clk_ipu_di_recalc(struct clk *clk)
{
	u32 reg, div, mux;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	mux = (reg & MXC_CCM_CSCMR2_DI_CLK_SEL_MASK(clk->id)) >>
		MXC_CCM_CSCMR2_DI_CLK_SEL_OFFSET(clk->id);
	if (mux == 0) {
		reg = __raw_readl(MXC_CCM_CDCDR) &
		    MXC_CCM_CDCDR_DI1_CLK_PRED_MASK;
		div = (reg >> MXC_CCM_CDCDR_DI1_CLK_PRED_OFFSET) + 1;
		clk->rate = clk->parent->rate / div;
	} else if ((mux == 3) && (clk->id == 1)) {
		clk->rate = clk->parent->rate / 8;
	} else if ((mux == 3) && (clk->id == 0)) {
		reg = __raw_readl(MXC_CCM_CDCDR) &
			MXC_CCM_CDCDR_DI_PLL4_PODF_MASK;
		div = (reg >> MXC_CCM_CDCDR_DI_PLL4_PODF_OFFSET) + 1;
		clk->rate = clk->parent->rate / div;
	} else {
		clk->rate = clk->parent->rate;
	}
}

static int _clk_ipu_di_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;

	if ((clk->parent == &pll4_sw_clk) && (clk->id == 0)) {
		reg = __raw_readl(MXC_CCM_CDCDR);
		reg &= ~MXC_CCM_CDCDR_DI_PLL4_PODF_MASK;
		reg |= (div - 1) << MXC_CCM_CDCDR_DI_PLL4_PODF_OFFSET;
		__raw_writel(reg, MXC_CCM_CDCDR);
	} else if (clk->parent == &pll3_sw_clk) {
		reg = __raw_readl(MXC_CCM_CDCDR);
		reg &= ~MXC_CCM_CDCDR_DI1_CLK_PRED_MASK;
		reg |= (div - 1) << MXC_CCM_CDCDR_DI1_CLK_PRED_OFFSET;
		__raw_writel(reg, MXC_CCM_CDCDR);
	} else if ((clk->parent == &tve_clk) && (clk->id == 1))
		clk->rate = rate; /*the rate decided by tve hw actually*/
	else if ((clk->parent == &ldb_di_clk[clk->id]) && cpu_is_mx53()) {
		clk->rate = clk->parent->rate;
		return 0;
	} else
		return -EINVAL;

	clk->rate = rate;

	return 0;
}

static unsigned long _clk_ipu_di_round_rate(struct clk *clk,
					    unsigned long rate)
{
	u32 div;

	if ((clk->parent == &ldb_di_clk[clk->id]) && cpu_is_mx53())
		return clk->parent->rate;
	else {
		div = clk->parent->rate / rate;
		if (div > 8)
			div = 8;
		else if (div == 0)
			div++;
		return clk->parent->rate / div;
	}
}

static struct clk ipu_di_clk[] = {
	{
	.name = "ipu_di0_clk",
	.id = 0,
	.parent = &pll3_sw_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG5_OFFSET,
	.recalc = _clk_ipu_di_recalc,
	.set_parent = _clk_ipu_di_set_parent,
	.round_rate = _clk_ipu_di_round_rate,
	.set_rate = _clk_ipu_di_set_rate,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.flags = RATE_PROPAGATES,
	},
	{
	.name = "ipu_di1_clk",
	.id = 1,
	.parent = &pll3_sw_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG6_OFFSET,
	.recalc = _clk_ipu_di_recalc,
	.set_parent = _clk_ipu_di_set_parent,
	.round_rate = _clk_ipu_di_round_rate,
	.set_rate = _clk_ipu_di_set_rate,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.flags = RATE_PROPAGATES,
	},
};

static int _clk_ldb_di_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR2);

	if ((parent == &pll3_sw_clk)) {
		if (clk->id == 0)
			reg &= ~(MXC_CCM_CSCMR2_LDB_DI0_CLK_SEL);
		else
			reg &= ~(MXC_CCM_CSCMR2_LDB_DI1_CLK_SEL);
	} else if ((parent == &pll4_sw_clk)) {
		if (clk->id == 0)
			reg |= MXC_CCM_CSCMR2_LDB_DI0_CLK_SEL;
		else
			reg |= MXC_CCM_CSCMR2_LDB_DI1_CLK_SEL;
	} else {
		BUG();
	}

	__raw_writel(reg, MXC_CCM_CSCMR2);
	return 0;
}

static void _clk_ldb_di_recalc(struct clk *clk)
{
	u32 div;

	if (clk->id == 0)
		div = __raw_readl(MXC_CCM_CSCMR2) &
			MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	else
		div = __raw_readl(MXC_CCM_CSCMR2) &
			MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;

	if (div)
		clk->rate = clk->parent->rate / 7;
	else
		clk->rate = 2 * clk->parent->rate / 7;
}

static unsigned long _clk_ldb_di_round_rate(struct clk *clk,
						unsigned long rate)
{
	if (rate * 7 <= clk->parent->rate)
		return clk->parent->rate / 7;
	else
		return 2 * clk->parent->rate / 7;
}

static int _clk_ldb_di_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div = 0;

	if (rate * 7 <=  clk->parent->rate) {
		div = 7;
		rate = clk->parent->rate / 7;
	} else
		rate = 2 * clk->parent->rate / 7;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	if (div == 7)
		reg |= (clk->id ? MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV :
			MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV);
	else
		reg &= ~(clk->id ? MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV :
			MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV);
	__raw_writel(reg, MXC_CCM_CSCMR2);

	clk->rate = rate;
	return 0;
}

static int _clk_ldb_di_enable(struct clk *clk)
{
	_clk_enable(clk);
	ipu_di_clk[clk->id].set_parent(&ipu_di_clk[clk->id], clk);
	ipu_di_clk[clk->id].parent = clk;
	ipu_di_clk[clk->id].rate = clk->rate;
	ipu_di_clk[clk->id].enable(&ipu_di_clk[clk->id]);
	ipu_di_clk[clk->id].usecount++;
	return 0;
}

static void _clk_ldb_di_disable(struct clk *clk)
{
	_clk_disable(clk);
	ipu_di_clk[clk->id].disable(&ipu_di_clk[clk->id]);
	ipu_di_clk[clk->id].usecount--;
}

static struct clk ldb_di_clk[] = {
	{
	.name = "ldb_di0_clk",
	.id = 0,
	.parent = &pll4_sw_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG14_OFFSET,
	.recalc = _clk_ldb_di_recalc,
	.set_parent = _clk_ldb_di_set_parent,
	.round_rate = _clk_ldb_di_round_rate,
	.set_rate = _clk_ldb_di_set_rate,
	.enable = _clk_ldb_di_enable,
	.disable = _clk_ldb_di_disable,
	.flags = RATE_PROPAGATES,
	},
	{
	.name = "ldb_di1_clk",
	.id = 1,
	.parent = &pll4_sw_clk,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG15_OFFSET,
	.recalc = _clk_ldb_di_recalc,
	.set_parent = _clk_ldb_di_set_parent,
	.round_rate = _clk_ldb_di_round_rate,
	.set_rate = _clk_ldb_di_set_rate,
	.enable = _clk_ldb_di_enable,
	.disable = _clk_ldb_di_disable,
	.flags = RATE_PROPAGATES,
	},
};

static int _clk_csi0_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk, NULL);
	reg = (reg & ~MXC_CCM_CSCMR2_CSI_MCLK1_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CSCMR2_CSI_MCLK1_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static void _clk_csi0_recalc(struct clk *clk)
{
	u32 reg, pred, podf;

	reg = __raw_readl(MXC_CCM_CSCDR4);
	pred = ((reg & MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PRED_MASK) >>
			MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PODF_MASK) >>
			MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / (pred * podf);
}

static unsigned long _clk_csi0_round_rate(struct clk *clk, unsigned long rate)
{
	u32 pre, post;
	u32 div = clk->parent->rate / rate;
	if (clk->parent->rate % rate)
		div++;

	__calc_pre_post_dividers(div, &pre, &post);

	return clk->parent->rate / (pre * post);
}

static int _clk_csi0_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;

	div = clk->parent->rate / rate;

	if ((clk->parent->rate / div) != rate)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	/* Set CSI clock divider */
	reg = __raw_readl(MXC_CCM_CSCDR4) &
	    ~(MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PRED_MASK |
		MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR4_CSI_MCLK1_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR4);

	clk->rate = rate;
	return 0;
}

static struct clk csi0_clk = {
	.name = "csi_mclk1",
	.parent = &pll3_sw_clk,
	.set_parent = _clk_csi0_set_parent,
	.recalc = _clk_csi0_recalc,
	.round_rate = _clk_csi0_round_rate,
	.set_rate = _clk_csi0_set_rate,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG2_OFFSET,
	.disable = _clk_disable,
};

static int _clk_csi1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk, NULL);
	reg = (reg & ~MXC_CCM_CSCMR2_CSI_MCLK2_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CSCMR2_CSI_MCLK2_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static void _clk_csi1_recalc(struct clk *clk)
{
	u32 reg, pred, podf;

	reg = __raw_readl(MXC_CCM_CSCDR4);
	pred = ((reg & MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PRED_MASK) >>
			MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PODF_MASK) >>
			MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / (pred * podf);
}

static unsigned long _clk_csi1_round_rate(struct clk *clk, unsigned long rate)
{
	u32 pre, post;
	u32 div = clk->parent->rate / rate;
	if (clk->parent->rate % rate)
		div++;

	__calc_pre_post_dividers(div, &pre, &post);

	return clk->parent->rate / (pre * post);
}

static int _clk_csi1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;

	div = clk->parent->rate / rate;

	if ((clk->parent->rate / div) != rate)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	/* Set CSI clock divider */
	reg = __raw_readl(MXC_CCM_CSCDR4) &
	    ~(MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PRED_MASK |
		MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR4_CSI_MCLK2_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR4);

	clk->rate = rate;
	return 0;
}

static struct clk csi1_clk = {
	.name = "csi_mclk2",
	.parent = &pll3_sw_clk,
	.set_parent = _clk_csi1_set_parent,
	.recalc = _clk_csi1_recalc,
	.round_rate = _clk_csi1_round_rate,
	.set_rate = _clk_csi1_set_rate,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG3_OFFSET,
	.disable = _clk_disable,
};


static int _clk_hsc_enable(struct clk *clk)
{
	u32 reg;

	_clk_enable(clk);
	/* Handshake with IPU when certain clock rates are changed. */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg &= ~MXC_CCM_CCDR_HSC_HS_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	reg = __raw_readl(MXC_CCM_CLPCR);
	reg &= ~MXC_CCM_CLPCR_BYPASS_HSC_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);

	return 0;
}

static void _clk_hsc_disable(struct clk *clk)
{
	u32 reg;

	_clk_disable(clk);
	/* No handshake with HSC as its not enabled. */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_HSC_HS_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_HSC_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
}

static struct clk mipi_esc_clk = {
	.name = "mipi_esc_clk",
	.parent = &pll2_sw_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG5_OFFSET,
};

static struct clk mipi_hsc2_clk = {
	.name = "mipi_hsc2_clk",
	.parent = &pll2_sw_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG4_OFFSET,
	.secondary = &mipi_esc_clk,
};

static struct clk mipi_hsc1_clk = {
	.name = "mipi_hsc1_clk",
	.parent = &pll2_sw_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG3_OFFSET,
	.secondary = &mipi_hsc2_clk,
};

static struct clk mipi_hsp_clk = {
	.name = "mipi_hsp_clk",
	.parent = &ipu_clk[0],
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG6_OFFSET,
	.enable = _clk_hsc_enable,
	.disable = _clk_hsc_disable,
	.secondary = &mipi_hsc1_clk,
};

static int _clk_tve_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);

	if ((parent == &pll3_sw_clk) && cpu_is_mx51()) {
		reg &= ~(MXC_CCM_CSCMR1_TVE_CLK_SEL);
	} else if ((parent == &pll4_sw_clk) && cpu_is_mx53()) {
		reg &= ~(MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL);
	} else if ((parent == &osc_clk) && cpu_is_mx51()) {
		reg |= MXC_CCM_CSCMR1_TVE_CLK_SEL;
		reg &= ~MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL;
	} else if (parent == &ckih_clk) {
		reg |= MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL;
		reg |= MXC_CCM_CSCMR1_TVE_CLK_SEL; /* Reserved on MX53 */
	} else {
		BUG();
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static void _clk_tve_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if ((reg & (MXC_CCM_CSCMR1_TVE_CLK_SEL | MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL)) == 0) {
		reg = __raw_readl(MXC_CCM_CDCDR) &
		    MXC_CCM_CDCDR_TVE_CLK_PRED_MASK;
		div = (reg >> MXC_CCM_CDCDR_TVE_CLK_PRED_OFFSET) + 1;
		clk->rate = clk->parent->rate / div;
	} else {
		clk->rate = clk->parent->rate;
	}
}

static unsigned long _clk_tve_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (cpu_is_mx51() && (reg & MXC_CCM_CSCMR1_TVE_CLK_SEL))
		return -EINVAL;
	if (cpu_is_mx53() && (reg & MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL))
		return -EINVAL;

	div = clk->parent->rate / rate;
	if (div > 8)
		div = 8;
	else if (div == 0)
		div++;
	return clk->parent->rate / div;
}

static int _clk_tve_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (cpu_is_mx51() && (reg & MXC_CCM_CSCMR1_TVE_CLK_SEL))
		return -EINVAL;
	if (cpu_is_mx53() && (reg & MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL))
		return -EINVAL;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;

	div--;
	reg = __raw_readl(MXC_CCM_CDCDR) & ~MXC_CCM_CDCDR_TVE_CLK_PRED_MASK;
	reg |= div << MXC_CCM_CDCDR_TVE_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CDCDR);
	clk->rate = rate;
	return 0;
}

static int _clk_tve_enable(struct clk *clk)
{
	_clk_enable(clk);
	if (clk_get_parent(&ipu_di_clk[1]) != clk) {
		clk_enable(&ipu_di_clk[1]);
		ipu_di_clk[1].set_parent(&ipu_di_clk[1], clk);
		ipu_di_clk[1].parent = clk;
	}
	return 0;
}

static void _clk_tve_disable(struct clk *clk)
{
	_clk_disable(clk);
	if (clk_get_parent(&ipu_di_clk[1]) == clk) {
		ipu_di_clk[1].set_parent(&ipu_di_clk[1], &pll3_sw_clk);
		ipu_di_clk[1].parent = &pll3_sw_clk;
		clk_disable(&ipu_di_clk[1]);
	}
}

static struct clk tve_clk = {
	.name = "tve_clk",
	.parent = &pll3_sw_clk,
	.set_parent = _clk_tve_set_parent,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGR2_CG15_OFFSET,
	.recalc = _clk_tve_recalc,
	.round_rate = _clk_tve_round_rate,
	.set_rate = _clk_tve_set_rate,
	.enable = _clk_tve_enable,
	.disable = _clk_tve_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk spba_clk = {
	.name = "spba_clk",
	.parent = &ipg_clk,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG0_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static void _clk_uart_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_UART_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_UART_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR1_UART_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static int _clk_uart_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_UART_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_UART_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk uart_main_clk = {
	.name = "uart_main_clk",
	.parent = &pll2_sw_clk,
	.recalc = _clk_uart_recalc,
	.set_parent = _clk_uart_set_parent,
	.flags = RATE_PROPAGATES,
};

static struct clk uart1_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 0,
	 .parent = &uart_main_clk,
	 .secondary = &uart1_clk[1],
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG4_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#ifdef UART1_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .name = "uart_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
#ifdef UART1_DMA_ENABLE
	 .secondary = &aips_tz1_clk,
#endif
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG3_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk uart2_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 1,
	 .parent = &uart_main_clk,
	 .secondary = &uart2_clk[1],
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG6_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#ifdef UART2_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .name = "uart_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
#ifdef UART2_DMA_ENABLE
	 .secondary = &aips_tz1_clk,
#endif
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG5_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk uart3_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 2,
	 .parent = &uart_main_clk,
	 .secondary = &uart3_clk[1],
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#ifdef UART3_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .name = "uart_ipg_clk",
	 .id = 2,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG7_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk uart4_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 3,
	 .parent = &uart_main_clk,
	 .secondary = &uart4_clk[1],
	 .enable_reg = MXC_CCM_CCGR7,
	 .enable_shift = MXC_CCM_CCGR7_CG5_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#ifdef UART4_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .name = "uart_ipg_clk",
	 .id = 3,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR7,
	 .enable_shift = MXC_CCM_CCGR7_CG4_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk uart5_clk[] = {
	{
	 .name = "uart_clk",
	 .id = 4,
	 .parent = &uart_main_clk,
	 .secondary = &uart5_clk[1],
	 .enable_reg = MXC_CCM_CCGR7,
	 .enable_shift = MXC_CCM_CCGR7_CG7_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
#ifdef UART5_DMA_ENABLE
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
#endif
	 },
	{
	 .name = "uart_ipg_clk",
	 .id = 4,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR7,
	 .enable_shift = MXC_CCM_CCGR7_CG6_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk gpt_clk[] = {
	{
	 .name = "gpt_clk",
	 .parent = &ipg_perclk,
	 .id = 0,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .secondary = &gpt_clk[1],
	 },
	{
	 .name = "gpt_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "gpt_32k_clk",
	 .id = 0,
	 .parent = &ckil_clk,
	 },
};

static struct clk pwm1_clk[] = {
	{
	 .name = "pwm",
	 .parent = &ipg_perclk,
	 .id = 0,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG6_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .secondary = &pwm1_clk[1],
	 },
	{
	 .name = "pwm_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG5_OFFSET,
	 .enable = _clk_enable_inrun, /*Active only when ARM is running. */
	 .disable = _clk_disable,
	 },
	{
	 .name = "pwm_32k_clk",
	 .id = 0,
	 .parent = &ckil_clk,
	 },
};

static struct clk pwm2_clk[] = {
	{
	 .name = "pwm",
	 .parent = &ipg_perclk,
	 .id = 1,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 .secondary = &pwm2_clk[1],
	 },
	{
	 .name = "pwm_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG7_OFFSET,
	 .enable = _clk_enable_inrun, /*Active only when ARM is running. */
	 .disable = _clk_disable,
	 },
	{
	 .name = "pwm_32k_clk",
	 .id = 1,
	 .parent = &ckil_clk,
	 },
};

static struct clk i2c_clk[] = {
	{
	 .name = "i2c_clk",
	 .id = 0,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "i2c_clk",
	 .id = 1,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "i2c_clk",
	 .id = 2,
	 .parent = &ipg_perclk,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG11_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static void _clk_hsi2c_serial_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR3);
	prediv = ((reg & MXC_CCM_CSCDR3_HSI2C_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR3_HSI2C_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR3_HSI2C_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR3_HSI2C_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static struct clk hsi2c_serial_clk = {
	.name = "hsi2c_serial_clk",
	.id = 0,
	.parent = &pll3_sw_clk,
	.secondary = &spba_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGR1_CG11_OFFSET,
	.recalc = _clk_hsi2c_serial_recalc,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static struct clk hsi2c_clk = {
	.name = "hsi2c_clk",
	.id = 0,
	.parent = &ipg_clk,
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGR1_CG12_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static void _clk_cspi_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR2);
	prediv = ((reg & MXC_CCM_CSCDR2_CSPI_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR2_CSPI_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		BUG();
	podf = ((reg & MXC_CCM_CSCDR2_CSPI_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR2_CSPI_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static int _clk_cspi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_CSPI_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_CSPI_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk cspi_main_clk = {
	.name = "cspi_main_clk",
	.parent = &pll3_sw_clk,
	.recalc = _clk_cspi_recalc,
	.set_parent = _clk_cspi_set_parent,
	.flags = RATE_PROPAGATES,
};

static struct clk cspi1_clk[] = {
	{
	 .name = "cspi_clk",
	 .id = 0,
	 .parent = &cspi_main_clk,
	 .secondary = &cspi1_clk[1],
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "cspi_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG9_OFFSET,
	 .enable = _clk_enable_inrun, /*Active only when ARM is running. */
	 .disable = _clk_disable,
	 },
};

static struct clk cspi2_clk[] = {
	{
	 .name = "cspi_clk",
	 .id = 1,
	 .parent = &cspi_main_clk,
	 .secondary = &cspi2_clk[1],
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG12_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "cspi_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .secondary = &aips_tz2_clk,
	 .enable_reg = MXC_CCM_CCGR4,
	 .enable_shift = MXC_CCM_CCGR4_CG11_OFFSET,
	 .enable = _clk_enable_inrun, /*Active only when ARM is running. */
	 .disable = _clk_disable,
	 },
};

static struct clk cspi3_clk = {
	.name = "cspi_ipg_clk",
	.id = 2,
	.parent = &ipg_clk,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG13_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	.secondary = &aips_tz2_clk,
};

static int _clk_ssi_lp_apm_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &ckih_clk, &lp_apm_clk, &ckih2_clk, NULL);
	reg = __raw_readl(MXC_CCM_CSCMR1) &
	    ~MXC_CCM_CSCMR1_SSI_APM_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_SSI_APM_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk ssi_lp_apm_clk = {
	.name = "ssi_lp_apm_clk",
	.parent = &ckih_clk,
	.set_parent = _clk_ssi_lp_apm_set_parent,
};

static void _clk_ssi1_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CS1CDR);
	prediv = ((reg & MXC_CCM_CS1CDR_SSI1_CLK_PRED_MASK) >>
		  MXC_CCM_CS1CDR_SSI1_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		BUG();
	podf = ((reg & MXC_CCM_CS1CDR_SSI1_CLK_PODF_MASK) >>
		MXC_CCM_CS1CDR_SSI1_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}
static int _clk_ssi1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk,
		       &pll3_sw_clk, &ssi_lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_SSI1_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_SSI1_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk ssi1_clk[] = {
	{
	 .name = "ssi_clk",
	 .id = 0,
	 .parent = &pll3_sw_clk,
	 .set_parent = _clk_ssi1_set_parent,
	 .secondary = &ssi1_clk[1],
	 .recalc = _clk_ssi1_recalc,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "ssi_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &ssi1_clk[2],
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "ssi_dep_clk",
	 .id = 0,
	 .parent = &aips_tz2_clk,
#ifdef CONFIG_SND_MXC_SOC_IRAM
	 .secondary = &emi_intr_clk[0],
#else
	 .secondary = &emi_fast_clk,
#endif
	 },
};

static void _clk_ssi2_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CS2CDR);
	prediv = ((reg & MXC_CCM_CS2CDR_SSI2_CLK_PRED_MASK) >>
		  MXC_CCM_CS2CDR_SSI2_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		BUG();
	podf = ((reg & MXC_CCM_CS2CDR_SSI2_CLK_PODF_MASK) >>
		MXC_CCM_CS2CDR_SSI2_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static int _clk_ssi2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk,
		       &pll3_sw_clk, &ssi_lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_SSI2_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_SSI2_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk ssi2_clk[] = {
	{
	 .name = "ssi_clk",
	 .id = 1,
	 .parent = &pll3_sw_clk,
	 .set_parent = _clk_ssi2_set_parent,
	 .secondary = &ssi2_clk[1],
	 .recalc = _clk_ssi2_recalc,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG11_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "ssi_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .secondary = &ssi2_clk[2],
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG10_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "ssi_dep_clk",
	 .id = 1,
	 .parent = &spba_clk,
#ifdef CONFIG_SND_MXC_SOC_IRAM
	 .secondary = &emi_intr_clk[0],
#else
	 .secondary = &emi_fast_clk,
#endif
	 },
};

static void _clk_ssi_ext1_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	clk->rate = clk->parent->rate;
	reg = __raw_readl(MXC_CCM_CSCMR1);
	if ((reg & MXC_CCM_CSCMR1_SSI_EXT1_COM_CLK_SEL) == 0) {
		reg = __raw_readl(MXC_CCM_CS1CDR);
		prediv = ((reg & MXC_CCM_CS1CDR_SSI_EXT1_CLK_PRED_MASK) >>
			  MXC_CCM_CS1CDR_SSI_EXT1_CLK_PRED_OFFSET) + 1;
		if (prediv == 1)
			BUG();
		podf = ((reg & MXC_CCM_CS1CDR_SSI_EXT1_CLK_PODF_MASK) >>
			MXC_CCM_CS1CDR_SSI_EXT1_CLK_PODF_OFFSET) + 1;
		clk->rate = clk->parent->rate / (prediv * podf);
	}
}

static int _clk_ssi_ext1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div, pre, post;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || div > 512)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	reg = __raw_readl(MXC_CCM_CS1CDR);
	reg &= ~(MXC_CCM_CS1CDR_SSI_EXT1_CLK_PRED_MASK |
		 MXC_CCM_CS1CDR_SSI_EXT1_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CS1CDR_SSI_EXT1_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CS1CDR_SSI_EXT1_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CS1CDR);

	clk->rate = rate;

	return 0;
}

static int _clk_ssi_ext1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &ssi1_clk[0]) {
		reg |= MXC_CCM_CSCMR1_SSI_EXT1_COM_CLK_SEL;
	} else {
		reg &= ~MXC_CCM_CSCMR1_SSI_EXT1_COM_CLK_SEL;
		mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			       &ssi_lp_apm_clk);
		reg = (reg & ~MXC_CCM_CSCMR1_SSI_EXT1_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CSCMR1_SSI_EXT1_CLK_SEL_OFFSET);
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static unsigned long _clk_ssi_ext1_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 pre, post;
	u32 div = clk->parent->rate / rate;

	if (clk->parent->rate % rate)
		div++;

	__calc_pre_post_dividers(div, &pre, &post);

	return clk->parent->rate / (pre * post);
}

static struct clk ssi_ext1_clk = {
	.name = "ssi_ext1_clk",
	.parent = &pll3_sw_clk,
	.set_parent = _clk_ssi_ext1_set_parent,
	.set_rate = _clk_ssi_ext1_set_rate,
	.round_rate = _clk_ssi_ext1_round_rate,
	.recalc = _clk_ssi_ext1_recalc,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGR3_CG14_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static void _clk_ssi_ext2_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	clk->rate = clk->parent->rate;
	reg = __raw_readl(MXC_CCM_CSCMR1);
	if ((reg & MXC_CCM_CSCMR1_SSI_EXT2_COM_CLK_SEL) == 0) {
		reg = __raw_readl(MXC_CCM_CS2CDR);
		prediv = ((reg & MXC_CCM_CS2CDR_SSI_EXT2_CLK_PRED_MASK) >>
			  MXC_CCM_CS2CDR_SSI_EXT2_CLK_PRED_OFFSET) + 1;
		if (prediv == 1)
			BUG();
		podf = ((reg & MXC_CCM_CS2CDR_SSI_EXT2_CLK_PODF_MASK) >>
			MXC_CCM_CS2CDR_SSI_EXT2_CLK_PODF_OFFSET) + 1;
		clk->rate = clk->parent->rate / (prediv * podf);
	}
}

static int _clk_ssi_ext2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &ssi2_clk[0]) {
		reg |= MXC_CCM_CSCMR1_SSI_EXT2_COM_CLK_SEL;
	} else {
		reg &= ~MXC_CCM_CSCMR1_SSI_EXT2_COM_CLK_SEL;
		mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			       &ssi_lp_apm_clk);
		reg = (reg & ~MXC_CCM_CSCMR1_SSI_EXT2_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CSCMR1_SSI_EXT2_CLK_SEL_OFFSET);
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk ssi_ext2_clk = {
	.name = "ssi_ext2_clk",
	.parent = &pll3_sw_clk,
	.set_parent = _clk_ssi_ext2_set_parent,
	.recalc = _clk_ssi_ext2_recalc,
	.enable_reg = MXC_CCM_CCGR3,
	.enable_shift = MXC_CCM_CCGR3_CG15_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
};

static int _clk_esai_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	if (parent ==  &pll1_sw_clk || parent ==  &pll2_sw_clk ||
		    parent ==  &pll3_sw_clk) {
		mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			NULL);
		reg &= ~MXC_CCM_CSCMR2_ESAI_PRE_SEL_MASK;
		reg |= mux << MXC_CCM_CSCMR2_ESAI_PRE_SEL_OFFSET;
		reg &= ~MXC_CCM_CSCMR2_ESAI_POST_SEL_MASK;
		reg |= 0 << MXC_CCM_CSCMR2_ESAI_POST_SEL_OFFSET;
		/* divider setting */
	} else {
		mux = _get_mux(parent, &ssi1_clk[0], &ssi2_clk[0], &ckih_clk,
			&ckih2_clk);
		reg &= ~MXC_CCM_CSCMR2_ESAI_POST_SEL_MASK;
		reg |= (mux + 1) << MXC_CCM_CSCMR2_ESAI_POST_SEL_OFFSET;
		/* divider setting */
	}

	__raw_writel(reg, MXC_CCM_CSCMR2);

	/* set podf = 0 */
	reg = __raw_readl(MXC_CCM_CS1CDR);
	reg &= ~MXC_CCM_CS1CDR_ESAI_CLK_PODF_MASK;
	__raw_writel(reg, MXC_CCM_CS1CDR);

	return 0;
}

static void _clk_esai_recalc(struct clk *clk)
{
	u32 reg, pred, podf;

	reg = __raw_readl(MXC_CCM_CS1CDR);
	if (clk->parent ==  &pll1_sw_clk || clk->parent ==  &pll2_sw_clk ||
		    clk->parent ==  &pll3_sw_clk) {
		pred = ((reg & MXC_CCM_CS1CDR_ESAI_CLK_PRED_MASK) >>
			MXC_CCM_CS1CDR_ESAI_CLK_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CS1CDR_ESAI_CLK_PODF_MASK) >>
			MXC_CCM_CS1CDR_ESAI_CLK_PODF_OFFSET) + 1;

		clk->rate = clk->parent->rate / (pred * podf);
	} else {
		podf = ((reg & MXC_CCM_CS1CDR_ESAI_CLK_PODF_MASK) >>
			MXC_CCM_CS1CDR_ESAI_CLK_PODF_OFFSET) + 1;

		clk->rate = clk->parent->rate / podf;
	}
}

static struct clk esai_clk[] = {
	{
	 .name = "esai_clk",
	 .id = 0,
	 .parent = &pll3_sw_clk,
	 .set_parent = _clk_esai_set_parent,
	 .recalc = _clk_esai_recalc,
	 .secondary = &esai_clk[1],
	 .enable_reg = MXC_CCM_CCGR6,
	 .enable_shift = MXC_CCM_CCGR6_CG9_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
	{
	 .name = "esai_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .enable_reg = MXC_CCM_CCGR6,
	 .enable_shift = MXC_CCM_CCGR6_CG8_OFFSET,
	 .enable = _clk_enable,
	 .disable = _clk_disable,
	 },
};

static struct clk iim_clk = {
	.name = "iim_clk",
	.parent = &ipg_clk,
	.secondary = &aips_tz2_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG15_OFFSET,
	.disable = _clk_disable,
};

static struct clk tmax1_clk = {
	 .name = "tmax1_clk",
	 .id = 0,
	 .parent = &ahb_clk,
	 .secondary = &ahb_max_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG0_OFFSET,
	 .disable = _clk_disable,
	 };

static struct clk tmax2_clk = {
	 .name = "tmax2_clk",
	 .id = 0,
	 .parent = &ahb_clk,
	 .secondary = &ahb_max_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG1_OFFSET,
	 .disable = _clk_disable,
};

static struct clk tmax3_clk = {
	 .name = "tmax3_clk",
	 .id = 0,
	 .parent = &ahb_clk,
	 .secondary = &ahb_max_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR1,
	 .enable_shift = MXC_CCM_CCGR1_CG2_OFFSET,
	 .disable = _clk_disable,
};

static void _clk_usboh3_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_USBOH3_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_USBOH3_CLK_PRED_OFFSET) + 1;
	if (prediv == 1)
		BUG();
	podf = ((reg & MXC_CCM_CSCDR1_USBOH3_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_USBOH3_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static int _clk_usboh3_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_USBOH3_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_USBOH3_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk usboh3_clk[] = {
	{
	 .name = "usboh3_clk",
	 .parent = &pll3_sw_clk,
	 .set_parent = _clk_usboh3_set_parent,
	 .recalc = _clk_usboh3_recalc,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG14_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &usboh3_clk[1],
	 .flags = AHB_MED_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	 },
	{
	 .name = "usb_sec_clk",
	 .parent = &tmax2_clk,
#if defined(CONFIG_USB_STATIC_IRAM) \
    || defined(CONFIG_USB_STATIC_IRAM_PPH)
	 .secondary = &emi_intr_clk[0],
#else
	 .secondary = &emi_fast_clk,
#endif
	 },
};

static struct clk usb_ahb_clk = {
	 .name = "usb_ahb_clk",
	 .parent = &ipg_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR2,
	 .enable_shift = MXC_CCM_CCGR2_CG13_OFFSET,
	 .disable = _clk_disable,
};

static void _clk_usb_phy_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	if (clk->parent == &pll3_sw_clk) {
		reg = __raw_readl(MXC_CCM_CDCDR);
		prediv = ((reg & MXC_CCM_CDCDR_USB_PHY_PRED_MASK) >>
			  MXC_CCM_CDCDR_USB_PHY_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CDCDR_USB_PHY_PODF_MASK) >>
			MXC_CCM_CDCDR_USB_PHY_PODF_OFFSET) + 1;

		clk->rate = clk->parent->rate / (prediv * podf);
	} else
		clk->rate = clk->parent->rate;
}

static int _clk_usb_phy_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CSCMR1);
	if (parent == &osc_clk)
		reg &= ~MXC_CCM_CSCMR1_USB_PHY_CLK_SEL;
	else if (parent == &pll3_sw_clk)
		reg |= MXC_CCM_CSCMR1_USB_PHY_CLK_SEL;
	else
		BUG();

	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static struct clk usb_phy_clk[] = {
	{
	.name = "usb_phy1_clk",
	.id = 0,
	.parent = &pll3_sw_clk,
	.secondary = &tmax3_clk,
	.set_parent = _clk_usb_phy_set_parent,
	.recalc = _clk_usb_phy_recalc,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGR2_CG0_OFFSET,
	.disable = _clk_disable,
	},
	{
	.name = "usb_phy2_clk",
	.id = 1,
	.parent = &pll3_sw_clk,
	.secondary = &tmax3_clk,
	.set_parent = _clk_usb_phy_set_parent,
	.recalc = _clk_usb_phy_recalc,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG6_OFFSET,
	.disable = _clk_disable,
	}
};

static struct clk esdhc_dep_clks = {
	 .name = "sd_dep_clk",
	 .parent = &spba_clk,
	 .secondary = &emi_fast_clk,
};

static void _clk_esdhc1_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static int _clk_esdhc1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
		       &lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CSCMR1) &
	    ~MXC_CCM_CSCMR1_ESDHC1_MSHC2_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_ESDHC1_MSHC2_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}


static int _clk_sdhc1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;

	div = clk->parent->rate / rate;

	if ((clk->parent->rate / div) != rate)
		return -EINVAL;

	 __calc_pre_post_dividers(div, &pre, &post);

	/* Set sdhc1 clock divider */
	reg = __raw_readl(MXC_CCM_CSCDR1) &
		~(MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PRED_MASK |
		MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR1_ESDHC1_MSHC2_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	clk->rate = rate;
	return 0;
}

static struct clk esdhc1_clk[] = {
	{
	 .name = "esdhc_clk",
	 .id = 0,
	 .parent = &pll2_sw_clk,
	 .set_parent = _clk_esdhc1_set_parent,
	 .recalc = _clk_esdhc1_recalc,
	 .set_rate = _clk_sdhc1_set_rate,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG1_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc1_clk[1],
	 },
	{
	 .name = "esdhc_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &esdhc1_clk[2],
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG0_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .name = "esdhc_sec_clk",
	 .id = 0,
	 .parent = &tmax3_clk,
	 .secondary = &esdhc_dep_clks,
	 },

};

static void _clk_esdhc2_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	if (cpu_is_mx51()) {
		reg = __raw_readl(MXC_CCM_CSCDR1);
		prediv = ((reg & MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PRED_MASK) >>
			MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PODF_MASK) >>
			MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PODF_OFFSET) + 1;

		clk->rate = clk->parent->rate / (prediv * podf);
	}
}

static int _clk_esdhc2_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	if (cpu_is_mx51()) {
		mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			       &lp_apm_clk);
		reg = __raw_readl(MXC_CCM_CSCMR1) &
		    ~MXC_CCM_CSCMR1_ESDHC3_MSHC2_CLK_SEL_MASK;
		reg |= mux << MXC_CCM_CSCMR1_ESDHC3_MSHC2_CLK_SEL_OFFSET;
	} else { /* MX53  */
		reg = __raw_readl(MXC_CCM_CSCMR1);
		if (parent == &esdhc1_clk[0])
			reg &= ~MXC_CCM_CSCMR1_ESDHC2_CLK_SEL;
		else if (parent == &esdhc3_clk[0])
			reg |= MXC_CCM_CSCMR1_ESDHC2_CLK_SEL;
		else
			BUG();
	}
	__raw_writel(reg, MXC_CCM_CSCMR1);
	return 0;
}

static int _clk_esdhc2_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;

	if (cpu_is_mx51()) {
		div = clk->parent->rate / rate;

		if ((clk->parent->rate / div) != rate)
			return -EINVAL;

		 __calc_pre_post_dividers(div, &pre, &post);

		/* Set sdhc1 clock divider */
		reg = __raw_readl(MXC_CCM_CSCDR1) &
			~(MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PRED_MASK |
			MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PODF_MASK);
		reg |= (post - 1) <<
				MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PODF_OFFSET;
		reg |= (pre - 1) <<
				MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PRED_OFFSET;
		__raw_writel(reg, MXC_CCM_CSCDR1);

	       clk->rate = rate;
	}
	return 0;
}

static struct clk esdhc2_clk[] = {
	{
	 .name = "esdhc_clk",
	 .id = 1,
	 .parent = &pll3_sw_clk,
	 .set_parent = _clk_esdhc2_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG3_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc2_clk[1],
	 },
	{
	 .name = "esdhc_ipg_clk",
	 .id = 1,
	 .parent = &ipg_clk,
	 .secondary = &esdhc2_clk[2],
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG2_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .name = "esdhc_sec_clk",
	 .id = 0,
	 .parent = &tmax2_clk,
	 .secondary = &esdhc_dep_clks,
	 },
};

static int _clk_esdhc3_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	if (cpu_is_mx51()) {
		reg = __raw_readl(MXC_CCM_CSCMR1);
		if (parent == &esdhc1_clk[0])
			reg &= ~MXC_CCM_CSCMR1_ESDHC3_CLK_SEL_MX51;
		else if (parent == &esdhc2_clk[0])
			reg |= MXC_CCM_CSCMR1_ESDHC3_CLK_SEL_MX51;
		else
			BUG();
	} else { /* MX53 */
		mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			       &lp_apm_clk);
		reg = __raw_readl(MXC_CCM_CSCMR1) &
		    ~MXC_CCM_CSCMR1_ESDHC3_MSHC2_CLK_SEL_MASK;
		reg |= mux << MXC_CCM_CSCMR1_ESDHC3_MSHC2_CLK_SEL_OFFSET;
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static void _clk_esdhc3_recalc(struct clk *clk)
{
	u32 reg, prediv, podf;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	prediv = ((reg & MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PRED_MASK) >>
		  MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PODF_OFFSET) + 1;

	clk->rate = clk->parent->rate / (prediv * podf);
}

static int _clk_sdhc3_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;

	if (cpu_is_mx53()) {
		div = clk->parent->rate / rate;

		if ((clk->parent->rate / div) != rate)
			return -EINVAL;

		__calc_pre_post_dividers(div, &pre, &post);

		/* Set sdhc1 clock divider */
		reg = __raw_readl(MXC_CCM_CSCDR1) &
			~(MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PRED_MASK |
			MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PODF_MASK);
		reg |= (post - 1) << MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PODF_OFFSET;
		reg |= (pre - 1) << MXC_CCM_CSCDR1_ESDHC3_MSHC2_CLK_PRED_OFFSET;
		 __raw_writel(reg, MXC_CCM_CSCDR1);

		clk->rate = rate;
	}
	return 0;
}


static struct clk esdhc3_clk[] = {
	{
	 .name = "esdhc_clk",
	 .id = 2,
	 .parent = &esdhc1_clk[0],
	 .set_parent = _clk_esdhc3_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG5_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc3_clk[1],
	 },
	{
	 .name = "esdhc_ipg_clk",
	 .id = 2,
	 .parent = &ipg_clk,
	 .secondary = &esdhc3_clk[2],
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG4_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .name = "esdhc_sec_clk",
	 .id = 0,
	 .parent = &ahb_max_clk,
	 .secondary = &esdhc_dep_clks,
	 },
};

static int _clk_esdhc4_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg;
	if (cpu_is_mx51()) {
		reg = __raw_readl(MXC_CCM_CSCMR1);
		if (parent == &esdhc1_clk[0])
			reg &= ~MXC_CCM_CSCMR1_ESDHC4_CLK_SEL;
		else if (parent == &esdhc2_clk[0])
			reg |= MXC_CCM_CSCMR1_ESDHC4_CLK_SEL;
		else
			BUG();
	} else {/*MX53 */
		reg = __raw_readl(MXC_CCM_CSCMR1);
		if (parent == &esdhc1_clk[0])
			reg &= ~MXC_CCM_CSCMR1_ESDHC4_CLK_SEL;
		else if (parent == &esdhc3_clk[0])
			reg |= MXC_CCM_CSCMR1_ESDHC4_CLK_SEL;
		else
			BUG();
	}

	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk esdhc4_clk[] = {
	{
	 .name = "esdhc_clk",
	 .id = 3,
	 .parent = &esdhc1_clk[0],
	 .set_parent = _clk_esdhc4_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG7_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &esdhc4_clk[1],
	 },
	{
	 .name = "esdhc_ipg_clk",
	 .id = 3,
	 .parent = &ipg_clk,
	 .secondary = &esdhc4_clk[2],
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR3,
	 .enable_shift = MXC_CCM_CCGR3_CG6_OFFSET,
	 .disable = _clk_disable,
	 },
	{
	 .name = "esdhc_sec_clk",
	 .id = 0,
	 .parent = &tmax3_clk,
	 .secondary = &esdhc_dep_clks,
	 },
};

static struct clk sata_clk = {
	.name = "sata_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG1_OFFSET,
	.disable = _clk_disable,
};

static struct clk ieee_1588_clk = {
	.name = "ieee_1588_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGR7_CG3_OFFSET,
	.disable = _clk_disable,
};

static struct clk mlb_clk[] = {
	{
	.name = "mlb_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR7,
	.enable_shift = MXC_CCM_CCGR7_CG2_OFFSET,
	.disable = _clk_disable,
	.secondary = &mlb_clk[1],
	},
	{
	.name = "mlb_mem_clk",
	.parent = &emi_fast_clk,
	.secondary = &emi_intr_clk[1],
	},
};

static struct clk can1_clk[] = {
	{
	.name = "can_clk",
	.id = 0,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.secondary = &can1_clk[1],
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG10_OFFSET,
	.disable = _clk_disable,
	 },
	{
	.name = "can_cpi_clk",
	.id = 0,
	.parent = &lp_apm_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG11_OFFSET,
	.disable = _clk_disable,
	 },
};

static struct clk can2_clk[] = {
	{
	.name = "can_clk",
	.id = 1,
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.secondary = &can2_clk[1],
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG3_OFFSET,
	.disable = _clk_disable,
	 },
	{
	.name = "can_cpi_clk",
	.id = 1,
	.parent = &lp_apm_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG4_OFFSET,
	.disable = _clk_disable,
	 },
};

static int _clk_sim_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk, NULL);
	reg = __raw_readl(MXC_CCM_CSCMR2) & ~MXC_CCM_CSCMR2_SIM_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR2_SIM_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static void _clk_sim_recalc(struct clk *clk)
{
	u32 reg, pred, podf;

	reg = __raw_readl(MXC_CCM_CSCDR2);
	pred = ((reg & MXC_CCM_CSCDR2_SIM_CLK_PRED_MASK) >>
		MXC_CCM_CSCDR2_SIM_CLK_PRED_OFFSET) + 1;
	podf = ((reg & MXC_CCM_CSCDR2_SIM_CLK_PODF_MASK) >>
		MXC_CCM_CSCDR2_SIM_CLK_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / (pred * podf);
}

static unsigned long _clk_sim_round_rate(struct clk *clk, unsigned long rate)
{
	u32 pre, post;
	u32 div = clk->parent->rate / rate;
	if (clk->parent->rate % rate)
		div++;

	__calc_pre_post_dividers(div, &pre, &post);

	return clk->parent->rate / (pre * post);
}

static int _clk_sim_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg;
	u32 div;
	u32 pre, post;

	div = clk->parent->rate / rate;

	if ((clk->parent->rate / div) != rate)
		return -EINVAL;

	__calc_pre_post_dividers(div, &pre, &post);

	/* Set SIM clock divider */
	reg = __raw_readl(MXC_CCM_CSCDR2) &
	    ~(MXC_CCM_CSCDR2_SIM_CLK_PRED_MASK |
	      MXC_CCM_CSCDR2_SIM_CLK_PODF_MASK);
	reg |= (post - 1) << MXC_CCM_CSCDR2_SIM_CLK_PODF_OFFSET;
	reg |= (pre - 1) << MXC_CCM_CSCDR2_SIM_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR2);

	clk->rate = rate;
	return 0;

}

static struct clk sim_clk[] = {
	{
	.name = "sim_clk",
	.parent = &pll3_sw_clk,
	.set_parent = _clk_sim_set_parent,
	.secondary = &sim_clk[1],
	.recalc = _clk_sim_recalc,
	.round_rate = _clk_sim_round_rate,
	.set_rate = _clk_sim_set_rate,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG2_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	 },
	{
	.name = "sim_ipg_clk",
	.parent = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG1_OFFSET,
	.disable = _clk_disable,
	 },
};

static void _clk_nfc_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CBCDR);
	div = ((reg & MXC_CCM_CBCDR_NFC_PODF_MASK) >>
	       MXC_CCM_CBCDR_NFC_PODF_OFFSET) + 1;
	clk->rate = clk->parent->rate / div;
}

static unsigned long _clk_nfc_round_rate(struct clk *clk,
						unsigned long rate)
{
	u32 div;

	/*
	 * Compute the divider we'd have to use to reach the target rate.
	 */

	div = clk->parent->rate / rate;

	/*
	 * If there's a remainder after the division, then we have to increment
	 * the divider. There are two reasons for this:
	 *
	 * 1) The frequency we round to must be LESS THAN OR EQUAL to the
	 *    target. We aren't allowed to round to a frequency that is higher
	 *    than the target.
	 *
	 * 2) This also catches the case where target rate is less than the
	 *    parent rate, which implies a divider of zero. We can't allow a
	 *    divider of zero.
	 */

	if (clk->parent->rate % rate)
		div++;

	/*
	 * The divider for this clock is 3 bits wide, so we can't possibly
	 * divide the parent by more than eight.
	 */

	if (div > 8)
		return -EINVAL;

	return clk->parent->rate / div;

}

static int _clk_nfc_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;
	struct timespec nstimeofday;
	struct timespec curtime;

	div = clk->parent->rate / rate;
	if (div == 0)
		div++;
	if (((clk->parent->rate / div) != rate) || (div > 8))
		return -EINVAL;

	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.enable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.enable(&emi_slow_clk);


	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_NFC_PODF_MASK;
	reg |= (div - 1) << MXC_CCM_CBCDR_NFC_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CBCDR);
	getnstimeofday(&nstimeofday);
	while (__raw_readl(MXC_CCM_CDHIPR) &
			MXC_CCM_CDHIPR_NFC_IPG_INT_MEM_PODF_BUSY){
		getnstimeofday(&curtime);
		if ((curtime.tv_nsec - nstimeofday.tv_nsec) > SPIN_DELAY)
			panic("_clk_nfc_set_rate failed\n");
	}
	clk->rate = rate;
	if (emi_fast_clk.usecount == 0)
		emi_fast_clk.disable(&emi_fast_clk);
	if (emi_slow_clk.usecount == 0)
		emi_slow_clk.disable(&emi_slow_clk);

	return 0;
}

static struct clk emi_enfc_clk = {
	.name = "nfc_clk",
	.parent = &emi_slow_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG10_OFFSET,
	.disable = _clk_disable_inwait,
	.recalc = _clk_nfc_recalc,
	.round_rate = _clk_nfc_round_rate,
	.set_rate = _clk_nfc_set_rate,
};

static int _clk_spdif_xtal_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	mux = _get_mux(parent, &osc_clk, &ckih_clk, &ckih2_clk, NULL);
	reg = __raw_readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_SPDIF_CLK_SEL_MASK;
	reg |= mux << MXC_CCM_CSCMR1_SPDIF_CLK_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCMR1);

	return 0;
}

static struct clk spdif_xtal_clk = {
	.name = "spdif_xtal_clk",
	.parent = &osc_clk,
	.set_parent = _clk_spdif_xtal_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG15_OFFSET,
	.disable = _clk_disable,
};

static int _clk_spdif0_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	reg |= MXC_CCM_CSCMR2_SPDIF0_COM;
	if (parent != &ssi1_clk[0]) {
		reg &= ~MXC_CCM_CSCMR2_SPDIF0_COM;
		mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			       &spdif_xtal_clk);
		reg = (reg & ~MXC_CCM_CSCMR2_SPDIF0_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CSCMR2_SPDIF0_CLK_SEL_OFFSET);
	}
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static void _clk_spdif0_recalc(struct clk *clk)
{
	u32 reg, pred, podf;

	if (clk->parent == &ssi1_clk[0]) {
		clk->rate = clk->parent->rate;
	} else {
		reg = __raw_readl(MXC_CCM_CDCDR);
		pred = ((reg & MXC_CCM_CDCDR_SPDIF0_CLK_PRED_MASK) >>
			MXC_CCM_CDCDR_SPDIF0_CLK_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CDCDR_SPDIF0_CLK_PODF_MASK) >>
			MXC_CCM_CDCDR_SPDIF0_CLK_PODF_OFFSET) + 1;
		clk->rate = clk->parent->rate / (pred * podf);
	}
}

static struct clk spdif0_clk[] = {
	{
	.name = "spdif_clk",
	.id = 0,
	.parent = &pll3_sw_clk,
	.set_parent = _clk_spdif0_set_parent,
	.recalc = _clk_spdif0_recalc,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG13_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	 .name = "spdif_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGR5_CG15_OFFSET,
	 .disable = _clk_disable,
	 },
};

static int _clk_spdif1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CSCMR2);
	reg |= MXC_CCM_CSCMR2_SPDIF1_COM;
	if (parent != &ssi2_clk[0]) {
		reg &= ~MXC_CCM_CSCMR2_SPDIF1_COM;
		mux = _get_mux(parent, &pll1_sw_clk, &pll2_sw_clk, &pll3_sw_clk,
			       &spdif_xtal_clk);
		reg = (reg & ~MXC_CCM_CSCMR2_SPDIF1_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CSCMR2_SPDIF1_CLK_SEL_OFFSET);
	}
	__raw_writel(reg, MXC_CCM_CSCMR2);

	return 0;
}

static void _clk_spdif1_recalc(struct clk *clk)
{
	u32 reg, pred, podf;

	if (clk->parent == &ssi2_clk[0]) {
		clk->rate = clk->parent->rate;
	} else {
		reg = __raw_readl(MXC_CCM_CDCDR);
		pred = ((reg & MXC_CCM_CDCDR_SPDIF1_CLK_PRED_MASK) >>
			MXC_CCM_CDCDR_SPDIF1_CLK_PRED_OFFSET) + 1;
		podf = ((reg & MXC_CCM_CDCDR_SPDIF1_CLK_PODF_MASK) >>
			MXC_CCM_CDCDR_SPDIF1_CLK_PODF_OFFSET) + 1;
		clk->rate = clk->parent->rate / (pred * podf);
	}
}

static struct clk spdif1_clk[] = {
	{
	.name = "spdif_clk",
	.id = 1,
	.parent = &pll3_sw_clk,
	.set_parent = _clk_spdif1_set_parent,
	.recalc = _clk_spdif1_recalc,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG14_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	 .name = "spdif_ipg_clk",
	 .id = 0,
	 .parent = &ipg_clk,
	 .secondary = &spba_clk,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGR5_CG15_OFFSET,
	 .disable = _clk_disable,
	 },
};

static int _clk_ddr_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, reg2, mux;
	struct timespec nstimeofday;
	struct timespec curtime;

	reg = __raw_readl(MXC_CCM_CBCMR);
	reg2 = __raw_readl(MXC_CCM_CBCDR);
	if (cpu_is_mx51()) {
		clk->parent = &ddr_hf_clk;
		mux = _get_mux_ddr(parent, &axi_a_clk, &axi_b_clk, &emi_slow_clk, &ahb_clk, &ddr_hf_clk);
	} else {
		clk->parent = &axi_a_clk;
		mux = _get_mux_ddr(parent, &axi_a_clk, &axi_b_clk, &emi_slow_clk, &ahb_clk, NULL);
	}
	if (mux < 4) {
		reg = (reg & ~MXC_CCM_CBCMR_DDR_CLK_SEL_MASK) |
		    (mux << MXC_CCM_CBCMR_DDR_CLK_SEL_OFFSET);
		__raw_writel(reg, MXC_CCM_CBCMR);
		if (cpu_is_mx51())
			reg2 = (reg2 & ~MXC_CCM_CBCDR_DDR_HF_SEL);
	} else {
		reg2 = (reg2 & ~MXC_CCM_CBCDR_DDR_HF_SEL) |
			(MXC_CCM_CBCDR_DDR_HF_SEL);
	}
	if (cpu_is_mx51()) {
		__raw_writel(reg2, MXC_CCM_CBCDR);
		getnstimeofday(&nstimeofday);
		while (__raw_readl(MXC_CCM_CDHIPR) &
			MXC_CCM_CDHIPR_DDR_HF_CLK_SEL_BUSY){
			getnstimeofday(&curtime);
			if ((curtime.tv_nsec - nstimeofday.tv_nsec) > SPIN_DELAY)
				panic("_clk_ddr_set_parent failed\n");
		}
	}
	return 0;
}

static struct clk ddr_clk = {
	.name = "ddr_clk",
	.parent = &axi_b_clk,
	.set_parent = _clk_ddr_set_parent,
	.flags = RATE_PROPAGATES,
};

static int _clk_arm_axi_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &emi_slow_clk, &ahb_clk);
	reg = (reg & ~MXC_CCM_CBCMR_ARM_AXI_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CBCMR_ARM_AXI_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static struct clk arm_axi_clk = {
	.name = "arm_axi_clk",
	.parent = &axi_a_clk,
	.set_parent = _clk_arm_axi_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR0,
	.enable_shift = MXC_CCM_CCGR0_CG1_OFFSET,
	.disable = _clk_disable,
};

static int _clk_vpu_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &emi_slow_clk, &ahb_clk);
	reg = (reg & ~MXC_CCM_CBCMR_VPU_AXI_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CBCMR_VPU_AXI_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static int _clk_vpu_enable(struct clk *clk)
{
	/* Set VPU's parent to be axi_a or ahb when its enabled. */
	if (cpu_is_mx51_rev(CHIP_REV_2_0) < 0) {
		clk_set_parent(&vpu_clk[0], &ahb_clk);
		clk_set_parent(&vpu_clk[1], &ahb_clk);
	} else if (cpu_is_mx51()) {
		clk_set_parent(&vpu_clk[0], &axi_a_clk);
		clk_set_parent(&vpu_clk[1], &axi_a_clk);
	}

	return _clk_enable(clk);

}

static void _clk_vpu_disable(struct clk *clk)
{
	_clk_disable(clk);

	/* Set VPU's parent to be axi_b when its disabled. */
	if (cpu_is_mx51()) {
		clk_set_parent(&vpu_clk[0], &axi_b_clk);
		clk_set_parent(&vpu_clk[1], &axi_b_clk);
	}
}

static struct clk vpu_clk[] = {
	{
	 .name = "vpu_clk",
	 .set_parent = _clk_vpu_set_parent,
	 .enable = _clk_enable,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGR5_CG4_OFFSET,
	 .disable = _clk_disable,
	 .secondary = &vpu_clk[1],
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	 },
	{
	 .name = "vpu_core_clk",
	 .set_parent = _clk_vpu_set_parent,
	 .enable = _clk_vpu_enable,
	 .enable_reg = MXC_CCM_CCGR5,
	 .enable_shift = MXC_CCM_CCGR5_CG3_OFFSET,
	 .disable = _clk_vpu_disable,
	 .secondary = &vpu_clk[2],
	 },
	{
	 .name = "vpu_emi_clk",
	 .parent = &emi_fast_clk,
#ifdef CONFIG_MXC_VPU_IRAM
	 .secondary = &emi_intr_clk[0],
#endif
	 }
};

static int _clk_lpsr_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;
	reg = __raw_readl(MXC_CCM_CLPCR);
	mux = _get_mux(parent, &ckil_clk, &osc_clk, NULL, NULL);
	reg = (reg & ~MXC_CCM_CLPCR_LPSR_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CLPCR_LPSR_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CLPCR);

	return 0;
}

static struct clk lpsr_clk = {
	.name = "lpsr_clk",
	.parent = &ckil_clk,
	.set_parent = _clk_lpsr_set_parent,
};

static void _clk_pgc_recalc(struct clk *clk)
{
	u32 reg, div;

	reg = __raw_readl(MXC_CCM_CSCDR1);
	div = (reg & MXC_CCM_CSCDR1_PGC_CLK_PODF_MASK) >>
	    MXC_CCM_CSCDR1_PGC_CLK_PODF_OFFSET;
	div = 1 >> div;
	clk->rate = clk->parent->rate / div;
}

static struct clk pgc_clk = {
	.name = "pgc_clk",
	.parent = &ipg_clk,
	.recalc = _clk_pgc_recalc,
};

/*usb OTG clock */
static struct clk usb_clk = {
	.name = "usb_clk",
	.rate = 60000000,
};

static struct clk usb_utmi_clk = {
	.name = "usb_utmi_clk",
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CSCMR1,
	.enable_shift = MXC_CCM_CSCMR1_USB_PHY_CLK_SEL_OFFSET,
	.disable = _clk_disable,
};

static struct clk rtc_clk = {
	.name = "rtc_clk",
	.parent = &ckil_clk,
	.secondary = &ipg_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG14_OFFSET,
	.disable = _clk_disable,
};

static struct clk ata_clk = {
	.name = "ata_clk",
	.parent = &ipg_clk,
	.secondary = &spba_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG0_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static struct clk owire_clk = {
	.name = "owire_clk",
	.parent = &ipg_perclk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGR2_CG11_OFFSET,
	.disable = _clk_disable,
};


static struct clk fec_clk[] = {
	{
	.name = "fec_clk",
	.parent = &ipg_clk,
	.secondary = &fec_clk[1],
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR2,
	.enable_shift = MXC_CCM_CCGR2_CG12_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	},
	{
	 .name = "fec_sec1_clk",
	 .parent = &tmax2_clk,
	 .secondary = &fec_clk[2],
	},
	{
	 .name = "fec_sec2_clk",
	 .parent = &aips_tz2_clk,
	 .secondary = &emi_fast_clk,
	},
};

static struct clk sahara_clk[] = {
	{
	.name = "sahara_clk",
	.parent = &ahb_clk,
	.secondary = &sahara_clk[1],
	.enable_reg = MXC_CCM_CCGR4,
	.enable_shift = MXC_CCM_CCGR4_CG7_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
	{
	.name = "sahara_sec_clk",
	.parent = &tmax1_clk,
	.secondary = &emi_fast_clk,
	}
};

static struct clk scc_clk[] = {
	{
	.name = "scc_clk",
	.parent = &ahb_clk,
	.secondary = &scc_clk[1],
	.enable_reg = MXC_CCM_CCGR1,
	.enable_shift = MXC_CCM_CCGR1_CG15_OFFSET,
	.enable = _clk_enable,
	.disable = _clk_disable,
	},
	{
	.name = "scc_sec_clk",
	.parent = &tmax1_clk,
	.secondary = &emi_fast_clk,
	}
};


static int _clk_gpu3d_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &emi_slow_clk, &ahb_clk);
	reg = (reg & ~MXC_CCM_CBCMR_GPU_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CBCMR_GPU_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}


static struct clk garb_clk = {
	.name = "garb_clk",
	.parent = &axi_a_clk,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG2_OFFSET,
	.disable = _clk_disable,
};

static struct clk gpu3d_clk = {
	.name = "gpu3d_clk",
	.parent = &axi_a_clk,
	.set_parent = _clk_gpu3d_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR5,
	.enable_shift = MXC_CCM_CCGR5_CG1_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
	.secondary = &garb_clk,
};

static int _clk_gpu2d_set_parent(struct clk *clk, struct clk *parent)
{
	u32 reg, mux;

	reg = __raw_readl(MXC_CCM_CBCMR);
	mux = _get_mux(parent, &axi_a_clk, &axi_b_clk, &emi_slow_clk, &ahb_clk);
	reg = (reg & ~MXC_CCM_CBCMR_GPU2D_CLK_SEL_MASK) |
	    (mux << MXC_CCM_CBCMR_GPU2D_CLK_SEL_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCMR);

	return 0;
}

static struct clk gpu2d_clk = {
	.name = "gpu2d_clk",
	.parent = &axi_a_clk,
	.set_parent = _clk_gpu2d_set_parent,
	.enable = _clk_enable,
	.enable_reg = MXC_CCM_CCGR6,
	.enable_shift = MXC_CCM_CCGR6_CG7_OFFSET,
	.disable = _clk_disable,
	.flags = AHB_HIGH_SET_POINT | CPU_FREQ_TRIG_UPDATE,
};

static void cko1_recalc(struct clk *clk)
{
	unsigned long rate;
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= MXC_CCM_CCOSR_CKOL_DIV_MASK;
	reg = reg >> MXC_CCM_CCOSR_CKOL_DIV_OFFSET;
	rate = clk->parent->rate;
	clk->rate = rate / (reg + 1);
}

static int cko1_enable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg |= MXC_CCM_CCOSR_CKOL_EN;
	__raw_writel(reg, MXC_CCM_CCOSR);
	return 0;
}

static void cko1_disable(struct clk *clk)
{
	u32 reg;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MXC_CCM_CCOSR_CKOL_EN;
	__raw_writel(reg, MXC_CCM_CCOSR);
}

static int cko1_set_rate(struct clk *clk, unsigned long rate)
{
	u32 reg, div;

	div = (clk->parent->rate/rate - 1) & 0x7;
	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MXC_CCM_CCOSR_CKOL_DIV_MASK;
	reg |= div << MXC_CCM_CCOSR_CKOL_DIV_OFFSET;
	__raw_writel(reg, MXC_CCM_CCOSR);
	return 0;
}

static unsigned long cko1_round_rate(struct clk *clk, unsigned long rate)
{
	u32 div;

	div = clk->parent->rate / rate;
	div = div < 1 ? 1 : div;
	div = div > 8 ? 8 : div;
	return clk->parent->rate / div;
}

static int cko1_set_parent(struct clk *clk, struct clk *parent)
{
	u32 sel, reg;

	if (parent == &cpu_clk)
		sel = 0;
	else if (parent == &pll1_sw_clk)
		sel = 1;
	else if (parent == &pll2_sw_clk)
		sel = 2;
	else if (parent == &pll3_sw_clk)
		sel = 3;
	else if (parent == &emi_slow_clk)
		sel = 4;
	else if (parent == &pll4_sw_clk)
		sel = 5;
	else if (parent == &emi_enfc_clk)
		sel = 6;
	else if (parent == &ipu_di_clk[0])
		sel = 8;
	else if (parent == &ahb_clk)
		sel = 11;
	else if (parent == &ipg_clk)
		sel = 12;
	else if (parent == &ipg_perclk)
		sel = 13;
	else if (parent == &ckil_clk)
		sel = 14;
	else
		return -EINVAL;

	reg = __raw_readl(MXC_CCM_CCOSR);
	reg &= ~MXC_CCM_CCOSR_CKOL_SEL_MASK;
	reg |= sel << MXC_CCM_CCOSR_CKOL_SEL_OFFSET;
	__raw_writel(reg, MXC_CCM_CCOSR);
	return 0;
}
static struct clk cko1_clk = {
	.name = "cko1_clk",
	.recalc = cko1_recalc,
	.enable = cko1_enable,
	.disable = cko1_disable,
	.set_rate = cko1_set_rate,
	.round_rate = cko1_round_rate,
	.set_parent = cko1_set_parent,
};

static struct clk *mxc_clks[] = {
	&osc_clk,
	&ckih_clk,
	&ckih2_clk,
	&ckil_clk,
	&pll1_main_clk,
	&pll1_sw_clk,
	&pll2_sw_clk,
	&pll3_sw_clk,
	&ipumux1_clk,
	&ipumux2_clk,
	&gpc_dvfs_clk,
	&lp_apm_clk,
	&cpu_clk,
	&periph_apm_clk,
	&main_bus_clk,
	&axi_a_clk,
	&axi_b_clk,
	&ahb_clk,
	&ahb_max_clk,
	&ipg_clk,
	&ipg_perclk,
	&ahbmux1_clk,
	&ahbmux2_clk,
	&aips_tz1_clk,
	&aips_tz2_clk,
	&sdma_clk[0],
	&sdma_clk[1],
	&ipu_clk[0],
	&ipu_clk[1],
	&ipu_di_clk[0],
	&ipu_di_clk[1],
	&tve_clk,
	&csi0_clk,
	&csi1_clk,
	&uart_main_clk,
	&uart1_clk[0],
	&uart1_clk[1],
	&uart2_clk[0],
	&uart2_clk[1],
	&uart3_clk[0],
	&uart3_clk[1],
	&spba_clk,
	&i2c_clk[0],
	&i2c_clk[1],
	&gpt_clk[0],
	&gpt_clk[1],
	&gpt_clk[2],
	&pwm1_clk[0],
	&pwm1_clk[1],
	&pwm1_clk[2],
	&pwm2_clk[0],
	&pwm2_clk[1],
	&pwm2_clk[2],
	&cspi_main_clk,
	&cspi1_clk[0],
	&cspi1_clk[1],
	&cspi2_clk[0],
	&cspi2_clk[1],
	&cspi3_clk,
	&ssi_lp_apm_clk,
	&ssi1_clk[0],
	&ssi1_clk[1],
	&ssi1_clk[2],
	&ssi2_clk[0],
	&ssi2_clk[1],
	&ssi2_clk[2],
	&ssi_ext1_clk,
	&ssi_ext2_clk,
	&iim_clk,
	&tmax1_clk,
	&tmax2_clk,
	&tmax3_clk,
	&usboh3_clk[0],
	&usboh3_clk[1],
	&usb_ahb_clk,
	&usb_phy_clk[0],
	&usb_utmi_clk,
	&usb_clk,
	&esdhc1_clk[0],
	&esdhc1_clk[1],
	&esdhc2_clk[0],
	&esdhc2_clk[1],
	&esdhc3_clk[0],
	&esdhc3_clk[1],
	&esdhc4_clk[0],
	&esdhc4_clk[1],
	&esdhc_dep_clks,
	&emi_slow_clk,
	&ddr_clk,
	&emi_enfc_clk,
	&emi_fast_clk,
	&emi_intr_clk[0],
	&emi_intr_clk[1],
	&spdif_xtal_clk,
	&spdif0_clk[0],
	&spdif0_clk[1],
	&arm_axi_clk,
	&vpu_clk[0],
	&vpu_clk[1],
	&vpu_clk[2],
	&lpsr_clk,
	&pgc_clk,
	&rtc_clk,
	&ata_clk,
	&owire_clk,
	&fec_clk[0],
	&fec_clk[1],
	&fec_clk[2],
	&sahara_clk[0],
	&sahara_clk[1],
	&gpu3d_clk,
	&garb_clk,
	&gpu2d_clk,
	&scc_clk[0],
	&scc_clk[1],
	&cko1_clk,
};

static void clk_tree_init(void)
{
	u32 reg, dp_ctl;

	ipg_perclk.set_parent(&ipg_perclk, &lp_apm_clk);

	/*
	 *Initialise the IPG PER CLK dividers to 3. IPG_PER_CLK should be at
	 * 8MHz, its derived from lp_apm.
	 */
	reg = __raw_readl(MXC_CCM_CBCDR);
	reg &= ~MXC_CCM_CBCDR_PERCLK_PRED1_MASK;
	reg &= ~MXC_CCM_CBCDR_PERCLK_PRED2_MASK;
	reg &= ~MXC_CCM_CBCDR_PERCLK_PODF_MASK;
	reg |= (2 << MXC_CCM_CBCDR_PERCLK_PRED1_OFFSET);
	__raw_writel(reg, MXC_CCM_CBCDR);

	/* set pll1_main_clk parent */
	pll1_main_clk.parent = &osc_clk;

	/* set pll2_sw_clk parent */
	pll2_sw_clk.parent = &osc_clk;

	/* set pll3_clk parent */
	pll3_sw_clk.parent = &osc_clk;

	if (cpu_is_mx51()) {
		dp_ctl = __raw_readl(pll1_base + MXC_PLL_DP_CTL);
		if ((dp_ctl & MXC_PLL_DP_CTL_REF_CLK_SEL_MASK) == 0)
			pll1_main_clk.parent = &fpm_clk;

		dp_ctl = __raw_readl(pll2_base + MXC_PLL_DP_CTL);
		if ((dp_ctl & MXC_PLL_DP_CTL_REF_CLK_SEL_MASK) == 0)
			pll2_sw_clk.parent = &fpm_clk;

		dp_ctl = __raw_readl(pll3_base + MXC_PLL_DP_CTL);
		if ((dp_ctl & MXC_PLL_DP_CTL_REF_CLK_SEL_MASK) == 0)
			pll3_sw_clk.parent = &fpm_clk;
	} else {
		/* set pll4_clk parent */
		pll4_sw_clk.parent = &osc_clk;
	}

	/* set emi_slow_clk parent */
	emi_slow_clk.parent = &main_bus_clk;
	reg = __raw_readl(MXC_CCM_CBCDR);
	if ((reg & MXC_CCM_CBCDR_EMI_CLK_SEL) != 0)
		emi_slow_clk.parent = &ahb_clk;

	/* set ipg_perclk parent */
	ipg_perclk.parent = &lp_apm_clk;
	reg = __raw_readl(MXC_CCM_CBCMR);
	if ((reg & MXC_CCM_CBCMR_PERCLK_IPG_CLK_SEL) != 0) {
		ipg_perclk.parent = &ipg_clk;
	} else {
		if ((reg & MXC_CCM_CBCMR_PERCLK_LP_APM_CLK_SEL) == 0)
			ipg_perclk.parent = &main_bus_clk;
	}
}


int __init mx51_clocks_init(unsigned long ckil, unsigned long osc, unsigned long ckih1, unsigned long ckih2)
{
	__iomem void *base;
	struct clk **clkp, *tclk;
	int i = 0, j = 0, reg;
	int wp_cnt = 0;

	pll1_base = ioremap(PLL1_BASE_ADDR, SZ_4K);
	pll2_base = ioremap(PLL2_BASE_ADDR, SZ_4K);
	pll3_base = ioremap(PLL3_BASE_ADDR, SZ_4K);

	/* Turn off all possible clocks */
	if (mxc_jtag_enabled) {
		__raw_writel(1 << MXC_CCM_CCGR0_CG0_OFFSET |
			      1 << MXC_CCM_CCGR0_CG1_OFFSET |
			      1 << MXC_CCM_CCGR0_CG2_OFFSET |
			      3 << MXC_CCM_CCGR0_CG3_OFFSET |
			      3 << MXC_CCM_CCGR0_CG4_OFFSET |
			      3 << MXC_CCM_CCGR0_CG8_OFFSET |
			      3 << MXC_CCM_CCGR0_CG9_OFFSET |
			      1 << MXC_CCM_CCGR0_CG12_OFFSET |
			      1 << MXC_CCM_CCGR0_CG13_OFFSET |
			      1 << MXC_CCM_CCGR0_CG14_OFFSET, MXC_CCM_CCGR0);
	} else {
		__raw_writel(1 << MXC_CCM_CCGR0_CG0_OFFSET |
			      1 << MXC_CCM_CCGR0_CG1_OFFSET |
			      1 << MXC_CCM_CCGR0_CG2_OFFSET |
			      3 << MXC_CCM_CCGR0_CG3_OFFSET |
			      3 << MXC_CCM_CCGR0_CG8_OFFSET |
			      3 << MXC_CCM_CCGR0_CG9_OFFSET |
			      1 << MXC_CCM_CCGR0_CG12_OFFSET |
			      1 << MXC_CCM_CCGR0_CG13_OFFSET |
			      3 << MXC_CCM_CCGR0_CG14_OFFSET, MXC_CCM_CCGR0);
	}
	__raw_writel(0, MXC_CCM_CCGR1);
	__raw_writel(0, MXC_CCM_CCGR2);
	__raw_writel(0, MXC_CCM_CCGR3);
	__raw_writel(1 << MXC_CCM_CCGR4_CG8_OFFSET, MXC_CCM_CCGR4);

	__raw_writel(1 << MXC_CCM_CCGR5_CG2_OFFSET |
		     1 << MXC_CCM_CCGR5_CG6_1_OFFSET |
		     1 << MXC_CCM_CCGR5_CG6_2_OFFSET |
		     3 << MXC_CCM_CCGR5_CG7_OFFSET |
		     1 << MXC_CCM_CCGR5_CG8_OFFSET |
		     3 << MXC_CCM_CCGR5_CG9_OFFSET |
		     1 << MXC_CCM_CCGR5_CG10_OFFSET |
		     3 << MXC_CCM_CCGR5_CG11_OFFSET, MXC_CCM_CCGR5);

	__raw_writel(1 << MXC_CCM_CCGR6_CG4_OFFSET, MXC_CCM_CCGR6);

	ckil_clk.rate = ckil;
	osc_clk.rate = osc;
	ckih_clk.rate = ckih1;
	ckih2_clk.rate = ckih2;

	/* Fix up clocks unique to MX51. */
	esdhc2_clk[0].recalc = _clk_esdhc2_recalc;
	esdhc2_clk[0].set_rate = _clk_esdhc2_set_rate;

	emi_intr_clk[1].name = "emi_garb_clk";
	clk_tree_init();

	for (clkp = mxc_clks; clkp < mxc_clks + ARRAY_SIZE(mxc_clks); clkp++)
		clk_register(*clkp);

	clk_register(&fpm_clk);
	clk_register(&fpm_div2_clk);
	clk_register(&hsi2c_clk);
	clk_register(&hsi2c_serial_clk);
	clk_register(&sim_clk[0]);
	clk_register(&sim_clk[1]);
	clk_register(&mipi_hsc1_clk);
	clk_register(&mipi_hsc2_clk);
	clk_register(&mipi_esc_clk);
	clk_register(&mipi_hsp_clk);
	clk_register(&spdif1_clk[0]);
	clk_register(&spdif1_clk[1]);
	clk_register(&ddr_hf_clk);

	max_axi_a_clk = MAX_AXI_A_CLK_MX51;
	max_axi_b_clk = MAX_AXI_B_CLK_MX51;

	/* set DDR clock parent */
	reg = 0;
	if (cpu_is_mx51_rev(CHIP_REV_2_0) >= 1) {
		reg = __raw_readl(MXC_CCM_CBCDR) & MXC_CCM_CBCDR_DDR_HF_SEL;
		reg >>= MXC_CCM_CBCDR_DDR_HF_SEL_OFFSET;

		if (reg)
			tclk = &ddr_hf_clk;
	}
	if (reg == 0) {
		reg = __raw_readl(MXC_CCM_CBCMR) &
					MXC_CCM_CBCMR_DDR_CLK_SEL_MASK;
		reg >>= MXC_CCM_CBCMR_DDR_CLK_SEL_OFFSET;

		if (reg == 0) {
			tclk = &axi_a_clk;
		} else if (reg == 1) {
			tclk = &axi_b_clk;
		} else if (reg == 2) {
			tclk = &emi_slow_clk;
		} else {
			tclk = &ahb_clk;
		}
	}
	clk_set_parent(&ddr_clk, tclk);

	/*Setup the LPM bypass bits */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_HSC_LPM_HS
		| MXC_CCM_CLPCR_BYPASS_IPU_LPM_HS
		| MXC_CCM_CLPCR_BYPASS_RTIC_LPM_HS
		| MXC_CCM_CLPCR_BYPASS_SCC_LPM_HS_MX51
		| MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS_MX51;
	__raw_writel(reg, MXC_CCM_CLPCR);

	/* Disable the handshake with HSC block as its not
	  * initialised right now.
	  */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_HSC_HS_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	/* This will propagate to all children and init all the clock rates */
	propagate_rate(&osc_clk);
	propagate_rate(&ckih_clk);
	propagate_rate(&ckih2_clk);
	propagate_rate(&ckil_clk);
	propagate_rate(&pll1_sw_clk);
	propagate_rate(&pll2_sw_clk);

	clk_enable(&cpu_clk);

	/* Set SDHC parents to be PLL2 */
	clk_set_parent(&esdhc1_clk[0], &pll2_sw_clk);
	clk_set_parent(&esdhc2_clk[0], &pll2_sw_clk);

	/* set SDHC root clock as 166.25MHZ*/
	clk_set_rate(&esdhc1_clk[0], 166250000);
	clk_set_rate(&esdhc2_clk[0], 166250000);

	/* Initialise the parents to be axi_b, parents are set to
	 * axi_a when the clocks are enabled.
	 */
	clk_set_parent(&vpu_clk[0], &axi_b_clk);
	clk_set_parent(&vpu_clk[1], &axi_b_clk);
	clk_set_parent(&gpu3d_clk, &axi_a_clk);
	clk_set_parent(&gpu2d_clk, &axi_a_clk);

	/* move cspi to 24MHz */
	clk_set_parent(&cspi_main_clk, &lp_apm_clk);
	clk_set_rate(&cspi_main_clk, 12000000);
	/*move the spdif0 to spdif_xtal_ckl */
	clk_set_parent(&spdif0_clk[0], &spdif_xtal_clk);
	/*set the SPDIF dividers to 1 */
	reg = __raw_readl(MXC_CCM_CDCDR);
	reg &= ~MXC_CCM_CDCDR_SPDIF0_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CDCDR_SPDIF0_CLK_PRED_MASK;
	__raw_writel(reg, MXC_CCM_CDCDR);

	/* move the spdif1 to 24MHz */
	clk_set_parent(&spdif1_clk[0], &spdif_xtal_clk);
	/* set the spdif1 dividers to 1 */
	reg = __raw_readl(MXC_CCM_CDCDR);
	reg &= ~MXC_CCM_CDCDR_SPDIF1_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CDCDR_SPDIF1_CLK_PRED_MASK;
	__raw_writel(reg, MXC_CCM_CDCDR);

	/* Move SSI clocks to SSI_LP_APM clock */
	clk_set_parent(&ssi_lp_apm_clk, &lp_apm_clk);

	clk_set_parent(&ssi1_clk[0], &ssi_lp_apm_clk);
	/* set the SSI dividers to divide by 2 */
	reg = __raw_readl(MXC_CCM_CS1CDR);
	reg &= ~MXC_CCM_CS1CDR_SSI1_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CS1CDR_SSI1_CLK_PRED_MASK;
	reg |= 1 << MXC_CCM_CS1CDR_SSI1_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CS1CDR);

	clk_set_parent(&ssi2_clk[0], &ssi_lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CS2CDR);
	reg &= ~MXC_CCM_CS2CDR_SSI2_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CS2CDR_SSI2_CLK_PRED_MASK;
	reg |= 1 << MXC_CCM_CS2CDR_SSI2_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CS2CDR);

	/* Change the SSI_EXT1_CLK to be sourced from SSI1_CLK_ROOT */
	clk_set_parent(&ssi_ext1_clk, &ssi1_clk[0]);
	clk_set_parent(&ssi_ext2_clk, &ssi2_clk[0]);

	/* move usb_phy_clk to 24MHz */
	clk_set_parent(&usb_phy_clk[0], &osc_clk);

	/* set usboh3_clk to pll2 */
	clk_set_parent(&usboh3_clk[0], &pll2_sw_clk);
	reg = __raw_readl(MXC_CCM_CSCDR1);
	reg &= ~MXC_CCM_CSCDR1_USBOH3_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CSCDR1_USBOH3_CLK_PRED_MASK;
	reg |= 4 << MXC_CCM_CSCDR1_USBOH3_CLK_PRED_OFFSET;
	reg |= 1 << MXC_CCM_CSCDR1_USBOH3_CLK_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	/* Set the current working point. */
	cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);
	/* Update the cpu working point table based on the PLL1 freq
	 * at boot time
	 */
	if (pll1_main_clk.rate <= cpu_wp_tbl[cpu_wp_nr - 1].cpu_rate)
		wp_cnt = 1;
	else if (pll1_main_clk.rate <= cpu_wp_tbl[1].cpu_rate &&
				pll1_main_clk.rate > cpu_wp_tbl[2].cpu_rate)
		wp_cnt = cpu_wp_nr - 1;
	else
		wp_cnt = cpu_wp_nr;

	cpu_wp_tbl[0].cpu_rate = pll1_main_clk.rate;

	if (wp_cnt == 1) {
		cpu_wp_tbl[0] = cpu_wp_tbl[cpu_wp_nr - 1];
		memset(&cpu_wp_tbl[cpu_wp_nr - 1], 0, sizeof(struct cpu_wp));
		memset(&cpu_wp_tbl[cpu_wp_nr - 2], 0, sizeof(struct cpu_wp));
	} else if (wp_cnt < cpu_wp_nr) {
		for (i = 0; i < wp_cnt; i++)
			cpu_wp_tbl[i] = cpu_wp_tbl[i+1];
		memset(&cpu_wp_tbl[i], 0, sizeof(struct cpu_wp));
	}

	if (wp_cnt < cpu_wp_nr) {
		set_num_cpu_wp(wp_cnt);
		cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);
	}


	for (j = 0; j < cpu_wp_nr; j++) {
		if ((ddr_clk.parent == &ddr_hf_clk)) {
			/* Change the CPU podf divider based on the boot up
			 * pll1 rate.
			 */
			cpu_wp_tbl[j].cpu_podf =
				(pll1_main_clk.rate / cpu_wp_tbl[j].cpu_rate)
				- 1;
			if (pll1_main_clk.rate/(cpu_wp_tbl[j].cpu_podf + 1) >
					cpu_wp_tbl[j].cpu_rate) {
				cpu_wp_tbl[j].cpu_podf++;
				cpu_wp_tbl[j].cpu_rate =
					 pll1_main_clk.rate/
					 (1000 * (cpu_wp_tbl[j].cpu_podf + 1));
				cpu_wp_tbl[j].cpu_rate *= 1000;
			}
			if (pll1_main_clk.rate/(cpu_wp_tbl[j].cpu_podf + 1) <
						cpu_wp_tbl[j].cpu_rate) {
				cpu_wp_tbl[j].cpu_rate = pll1_main_clk.rate;
			}
		}
	cpu_wp_tbl[j].pll_rate = pll1_main_clk.rate;
	}
	/* Set the current working point. */
	for (i = 0; i < cpu_wp_nr; i++) {
		if (clk_get_rate(&cpu_clk) == cpu_wp_tbl[i].cpu_rate) {
			cpu_curr_wp = i;
			break;
		}
	}
	if (i > cpu_wp_nr)
		BUG();

	clk_set_parent(&arm_axi_clk, &axi_a_clk);
	clk_set_parent(&ipu_clk[0], &axi_b_clk);

	if (uart_at_24) {
		/* Move UART to run from lp_apm */
		clk_set_parent(&uart_main_clk, &lp_apm_clk);

		/* Set the UART dividers to divide, so the UART_CLK is 24MHz. */
		reg = __raw_readl(MXC_CCM_CSCDR1);
		reg &= ~MXC_CCM_CSCDR1_UART_CLK_PODF_MASK;
		reg &= ~MXC_CCM_CSCDR1_UART_CLK_PRED_MASK;
		reg |= (0 << MXC_CCM_CSCDR1_UART_CLK_PRED_OFFSET) |
		    (0 << MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET);
		__raw_writel(reg, MXC_CCM_CSCDR1);
	} else {
		/* Move UART to run from PLL1 */
		clk_set_parent(&uart_main_clk, &pll1_sw_clk);

		/* Set the UART dividers to divide,
		 * so the UART_CLK is 66.5MHz.
		 */
		reg = __raw_readl(MXC_CCM_CSCDR1);
		reg &= ~MXC_CCM_CSCDR1_UART_CLK_PODF_MASK;
		reg &= ~MXC_CCM_CSCDR1_UART_CLK_PRED_MASK;
		reg |= (5 << MXC_CCM_CSCDR1_UART_CLK_PRED_OFFSET) |
		    (1 << MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET);
		__raw_writel(reg, MXC_CCM_CSCDR1);
	}

	propagate_rate(&osc_clk);
	propagate_rate(&pll1_sw_clk);
	propagate_rate(&pll2_sw_clk);
	propagate_rate(&pll3_sw_clk);

	clk_set_parent(&emi_slow_clk, &ahb_clk);
	clk_set_rate(&emi_slow_clk, clk_round_rate(&emi_slow_clk, 130000000));

	/* Change the NFC clock rate to be 1:4 ratio with emi clock. */
	clk_set_rate(&emi_enfc_clk, clk_round_rate(&emi_enfc_clk,
			(clk_get_rate(&emi_slow_clk))/4));

	base = ioremap(GPT1_BASE_ADDR, SZ_4K);
	mxc_timer_init(&gpt_clk[0], base, MXC_INT_GPT);
	return 0;
}

int __init mx53_clocks_init(unsigned long ckil, unsigned long osc, unsigned long ckih1, unsigned long ckih2)
{
	__iomem void *base;
	struct clk **clkp, *tclk;
	int i = 0, j = 0, reg;
	int wp_cnt = 0;

	pll1_base = ioremap(MX53_BASE_ADDR(PLL1_BASE_ADDR), SZ_4K);
	pll2_base = ioremap(MX53_BASE_ADDR(PLL2_BASE_ADDR), SZ_4K);
	pll3_base = ioremap(MX53_BASE_ADDR(PLL3_BASE_ADDR), SZ_4K);
	pll4_base = ioremap(MX53_BASE_ADDR(PLL4_BASE_ADDR), SZ_4K);

	/* Turn off all possible clocks */
	if (mxc_jtag_enabled) {
		__raw_writel(1 << MXC_CCM_CCGR0_CG0_OFFSET |
			      1 << MXC_CCM_CCGR0_CG1_OFFSET |
			      1 << MXC_CCM_CCGR0_CG2_OFFSET |
			      3 << MXC_CCM_CCGR0_CG3_OFFSET |
			      3 << MXC_CCM_CCGR0_CG4_OFFSET |
			      3 << MXC_CCM_CCGR0_CG8_OFFSET |
			      3 << MXC_CCM_CCGR0_CG9_OFFSET |
			      1 << MXC_CCM_CCGR0_CG12_OFFSET |
			      1 << MXC_CCM_CCGR0_CG13_OFFSET |
			      1 << MXC_CCM_CCGR0_CG14_OFFSET, MXC_CCM_CCGR0);
	} else {
		__raw_writel(1 << MXC_CCM_CCGR0_CG0_OFFSET |
			      1 << MXC_CCM_CCGR0_CG1_OFFSET |
			      3 << MXC_CCM_CCGR0_CG3_OFFSET |
			      3 << MXC_CCM_CCGR0_CG8_OFFSET |
			      3 << MXC_CCM_CCGR0_CG9_OFFSET |
			      1 << MXC_CCM_CCGR0_CG12_OFFSET |
			      1 << MXC_CCM_CCGR0_CG13_OFFSET |
			      3 << MXC_CCM_CCGR0_CG14_OFFSET, MXC_CCM_CCGR0);
	}

	__raw_writel(0, MXC_CCM_CCGR1);
	__raw_writel(0, MXC_CCM_CCGR2);
	__raw_writel(0, MXC_CCM_CCGR3);
	__raw_writel(1 << MXC_CCM_CCGR4_CG8_OFFSET, MXC_CCM_CCGR4);

	__raw_writel(1 << MXC_CCM_CCGR5_CG2_OFFSET |
		     1 << MXC_CCM_CCGR5_CG6_OFFSET |
		     3 << MXC_CCM_CCGR5_CG7_OFFSET |
		     1 << MXC_CCM_CCGR5_CG8_OFFSET |
		     3 << MXC_CCM_CCGR5_CG9_OFFSET |
		     1 << MXC_CCM_CCGR5_CG10_OFFSET |
		     3 << MXC_CCM_CCGR5_CG11_OFFSET, MXC_CCM_CCGR5);

	__raw_writel(3 << MXC_CCM_CCGR6_CG0_OFFSET |
				3 << MXC_CCM_CCGR6_CG1_OFFSET |
				3 << MXC_CCM_CCGR6_CG4_OFFSET |
				3 << MXC_CCM_CCGR6_CG8_OFFSET |
				3 << MXC_CCM_CCGR6_CG9_OFFSET |
				3 << MXC_CCM_CCGR6_CG12_OFFSET |
				3 << MXC_CCM_CCGR6_CG13_OFFSET , MXC_CCM_CCGR6);

	__raw_writel(0, MXC_CCM_CCGR7);

	ckil_clk.rate = ckil;
	osc_clk.rate = osc;
	ckih_clk.rate = ckih1;
	ckih2_clk.rate = ckih2;

	usb_phy_clk[0].enable_reg = MXC_CCM_CCGR4;
	usb_phy_clk[0].enable_shift = MXC_CCM_CCGR4_CG5_OFFSET;

	ipumux1_clk.enable_reg = MXC_CCM_CCGR5;
	ipumux1_clk.enable_shift = MXC_CCM_CCGR5_CG6_OFFSET;
	ipumux2_clk.enable_reg = MXC_CCM_CCGR6;
	ipumux2_clk.enable_shift = MXC_CCM_CCGR6_CG0_OFFSET;

	esdhc3_clk[0].recalc = _clk_esdhc3_recalc;
	esdhc3_clk[0].set_rate = _clk_sdhc3_set_rate;

#ifdef CONFIG_MXC_VPU_IRAM
	 vpu_clk[2].secondary = &emi_intr_clk[1];
#endif
#if defined(CONFIG_USB_STATIC_IRAM) \
    || defined(CONFIG_USB_STATIC_IRAM_PPH)
	usboh3_clk[1].secondary = &emi_intr_clk[1];
#endif
#ifdef CONFIG_SND_MXC_SOC_IRAM
	ssi2_clk[2].secondary = &emi_intr_clk[1];
	ssi1_clk[2].secondary = &emi_intr_clk[1];
#endif
#ifdef CONFIG_SDMA_IRAM
	sdma_clk[1].secondary = &emi_intr_clk[1];
#endif

	clk_tree_init();

	for (clkp = mxc_clks; clkp < mxc_clks + ARRAY_SIZE(mxc_clks); clkp++)
		clk_register(*clkp);

	clk_register(&pll4_sw_clk);
	clk_register(&uart4_clk[0]);
	clk_register(&uart4_clk[1]);
	clk_register(&uart5_clk[0]);
	clk_register(&uart5_clk[1]);
	clk_register(&i2c_clk[2]);
	clk_register(&usb_phy_clk[1]);
	clk_register(&ocram_clk);
	clk_register(&sata_clk);
	clk_register(&ieee_1588_clk);
	clk_register(&mlb_clk[0]);
	clk_register(&can1_clk[0]);
	clk_register(&can2_clk[0]);
	clk_register(&ldb_di_clk[0]);
	clk_register(&ldb_di_clk[1]);
	/* OSC of 22.5792M or 24.576M for ESAI */
	clk_register(&esai_clk[0]);
	clk_set_parent(&esai_clk[0], &ckih_clk);
	clk_register(&esai_clk[1]);

	ldb_di_clk[0].parent = ldb_di_clk[1].parent =
	tve_clk.parent = &pll4_sw_clk;

	max_axi_a_clk = MAX_AXI_A_CLK_MX53;
	max_axi_b_clk = MAX_AXI_B_CLK_MX53;

	/* set DDR clock parent */
	reg = __raw_readl(MXC_CCM_CBCMR) &
				MXC_CCM_CBCMR_DDR_CLK_SEL_MASK;
	reg >>= MXC_CCM_CBCMR_DDR_CLK_SEL_OFFSET;
	if (reg == 0) {
		tclk = &axi_a_clk;
	} else if (reg == 1) {
		tclk = &axi_b_clk;
	} else if (reg == 2) {
		tclk = &emi_slow_clk;
	} else {
		tclk = &ahb_clk;
	}
	clk_set_parent(&ddr_clk, tclk);

	clk_set_parent(&esdhc1_clk[2], &tmax2_clk);
	clk_set_parent(&esdhc2_clk[0], &esdhc1_clk[0]);
	clk_set_parent(&esdhc3_clk[0], &pll2_sw_clk);

	clk_set_parent(&ipu_di_clk[0], &pll4_sw_clk);

#if 0
	/*Setup the LPM bypass bits */
	reg = __raw_readl(MXC_CCM_CLPCR);
	reg |= MXC_CCM_CLPCR_BYPASS_IPU_LPM_HS
		| MXC_CCM_CLPCR_BYPASS_RTIC_LPM_HS
		| MXC_CCM_CLPCR_BYPASS_SCC_LPM_HS
		| MXC_CCM_CLPCR_BYPASS_SDMA_LPM_HS;
	__raw_writel(reg, MXC_CCM_CLPCR);
#endif

	/* Disable the handshake with HSC block as its not
	  * initialised right now.
	  */
	reg = __raw_readl(MXC_CCM_CCDR);
	reg |= MXC_CCM_CCDR_EMI_HS_MASK;
	__raw_writel(reg, MXC_CCM_CCDR);

	/* This will propagate to all children and init all the clock rates */
	propagate_rate(&osc_clk);
	propagate_rate(&ckih_clk);
	propagate_rate(&ckih2_clk);
	propagate_rate(&ckil_clk);
	propagate_rate(&pll1_sw_clk);
	propagate_rate(&pll2_sw_clk);
	propagate_rate(&pll3_sw_clk);

	clk_enable(&cpu_clk);

	clk_enable(&main_bus_clk);

	/* Set AXI_B_CLK to be 200MHz */
	clk_set_rate(&axi_b_clk, 200000000);

	/* Initialise the parents to be axi_b, parents are set to
	 * axi_a when the clocks are enabled.
	 */

	clk_set_parent(&vpu_clk[0], &axi_b_clk);
	clk_set_parent(&vpu_clk[1], &axi_b_clk);

	/* move cspi to 24MHz */
	clk_set_parent(&cspi_main_clk, &lp_apm_clk);
	clk_set_rate(&cspi_main_clk, 12000000);
	/*move the spdif0 to spdif_xtal_ckl */
	clk_set_parent(&spdif0_clk[0], &spdif_xtal_clk);
	/*set the SPDIF dividers to 1 */
	reg = __raw_readl(MXC_CCM_CDCDR);
	reg &= ~MXC_CCM_CDCDR_SPDIF0_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CDCDR_SPDIF0_CLK_PRED_MASK;
	__raw_writel(reg, MXC_CCM_CDCDR);

	/* Move SSI clocks to SSI_LP_APM clock */
	clk_set_parent(&ssi_lp_apm_clk, &lp_apm_clk);

	clk_set_parent(&ssi1_clk[0], &ssi_lp_apm_clk);
	/* set the SSI dividers to divide by 2 */
	reg = __raw_readl(MXC_CCM_CS1CDR);
	reg &= ~MXC_CCM_CS1CDR_SSI1_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CS1CDR_SSI1_CLK_PRED_MASK;
	reg |= 1 << MXC_CCM_CS1CDR_SSI1_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CS1CDR);

	clk_set_parent(&ssi2_clk[0], &ssi_lp_apm_clk);
	reg = __raw_readl(MXC_CCM_CS2CDR);
	reg &= ~MXC_CCM_CS2CDR_SSI2_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CS2CDR_SSI2_CLK_PRED_MASK;
	reg |= 1 << MXC_CCM_CS2CDR_SSI2_CLK_PRED_OFFSET;
	__raw_writel(reg, MXC_CCM_CS2CDR);

	/* Change the SSI_EXT1_CLK to be sourced from PLL2 for camera */
	clk_disable(&ssi_ext1_clk);
	clk_set_parent(&ssi_ext1_clk, &pll2_sw_clk);
	clk_set_rate(&ssi_ext1_clk, 24000000);
	clk_enable(&ssi_ext1_clk);
	clk_set_parent(&ssi_ext2_clk, &ssi2_clk[0]);

	/* move usb_phy_clk to 24MHz */
	clk_set_parent(&usb_phy_clk[0], &osc_clk);
	clk_set_parent(&usb_phy_clk[1], &osc_clk);

	/* set usboh3_clk to pll2 */
	clk_set_parent(&usboh3_clk[0], &pll2_sw_clk);
	reg = __raw_readl(MXC_CCM_CSCDR1);
	reg &= ~MXC_CCM_CSCDR1_USBOH3_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CSCDR1_USBOH3_CLK_PRED_MASK;
	reg |= 4 << MXC_CCM_CSCDR1_USBOH3_CLK_PRED_OFFSET;
	reg |= 1 << MXC_CCM_CSCDR1_USBOH3_CLK_PODF_OFFSET;
	__raw_writel(reg, MXC_CCM_CSCDR1);

	/* set SDHC root clock as 200MHZ*/
	clk_set_rate(&esdhc1_clk[0], 200000000);
	clk_set_rate(&esdhc3_clk[0], 200000000);

	/* Set the current working point. */
	cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);
	/* Update the cpu working point table based on the PLL1 freq
	 * at boot time
	 */
	if (pll1_main_clk.rate <= cpu_wp_tbl[cpu_wp_nr - 1].cpu_rate)
		wp_cnt = 1;
	else if (pll1_main_clk.rate <= cpu_wp_tbl[1].cpu_rate &&
				pll1_main_clk.rate > cpu_wp_tbl[2].cpu_rate)
		wp_cnt = cpu_wp_nr - 1;
	else
		wp_cnt = cpu_wp_nr;

	cpu_wp_tbl[0].cpu_rate = pll1_main_clk.rate;

	if (wp_cnt == 1) {
		cpu_wp_tbl[0] = cpu_wp_tbl[cpu_wp_nr - 1];
		memset(&cpu_wp_tbl[cpu_wp_nr - 1], 0, sizeof(struct cpu_wp));
		memset(&cpu_wp_tbl[cpu_wp_nr - 2], 0, sizeof(struct cpu_wp));
	} else if (wp_cnt < cpu_wp_nr) {
		for (i = 0; i < wp_cnt; i++)
			cpu_wp_tbl[i] = cpu_wp_tbl[i+1];
		memset(&cpu_wp_tbl[i], 0, sizeof(struct cpu_wp));
	}

	if (wp_cnt < cpu_wp_nr) {
		set_num_cpu_wp(wp_cnt);
		cpu_wp_tbl = get_cpu_wp(&cpu_wp_nr);
	}


	for (j = 0; j < cpu_wp_nr; j++) {
		if ((ddr_clk.parent == &ddr_hf_clk)) {
			/* Change the CPU podf divider based on the boot up
			 * pll1 rate.
			 */
			cpu_wp_tbl[j].cpu_podf =
				(pll1_main_clk.rate / cpu_wp_tbl[j].cpu_rate)
				- 1;
			if (pll1_main_clk.rate/(cpu_wp_tbl[j].cpu_podf + 1) >
					cpu_wp_tbl[j].cpu_rate) {
				cpu_wp_tbl[j].cpu_podf++;
				cpu_wp_tbl[j].cpu_rate =
					 pll1_main_clk.rate/
					 (1000 * (cpu_wp_tbl[j].cpu_podf + 1));
				cpu_wp_tbl[j].cpu_rate *= 1000;
			}
			if (pll1_main_clk.rate/(cpu_wp_tbl[j].cpu_podf + 1) <
						cpu_wp_tbl[j].cpu_rate) {
				cpu_wp_tbl[j].cpu_rate = pll1_main_clk.rate;
			}
		}
	cpu_wp_tbl[j].pll_rate = pll1_main_clk.rate;
	}
	/* Set the current working point. */
	for (i = 0; i < cpu_wp_nr; i++) {
		if (clk_get_rate(&cpu_clk) == cpu_wp_tbl[i].cpu_rate) {
			cpu_curr_wp = i;
			break;
		}
	}
	if (i > cpu_wp_nr)
		BUG();

	propagate_rate(&osc_clk);
	propagate_rate(&pll1_sw_clk);
	propagate_rate(&pll2_sw_clk);
	propagate_rate(&pll3_sw_clk);

	clk_set_parent(&arm_axi_clk, &axi_b_clk);
	clk_set_parent(&ipu_clk[0], &axi_b_clk);
	clk_set_parent(&uart_main_clk, &pll3_sw_clk);
	clk_set_parent(&gpu3d_clk, &axi_b_clk);
	clk_set_parent(&gpu2d_clk, &axi_b_clk);

	clk_set_parent(&emi_slow_clk, &ahb_clk);
	clk_set_rate(&emi_slow_clk, clk_round_rate(&emi_slow_clk, 130000000));

	/* Change the NFC clock rate to be 1:4 ratio with emi clock. */
	clk_set_rate(&emi_enfc_clk, clk_round_rate(&emi_enfc_clk,
			(clk_get_rate(&emi_slow_clk))/4));

	base = ioremap(MX53_BASE_ADDR(GPT1_BASE_ADDR), SZ_4K);
	mxc_timer_init(&gpt_clk[0], base, MXC_INT_GPT);
	return 0;
}

/*!
 * Setup cpu clock based on working point.
 * @param	wp	cpu freq working point
 * @return		0 on success or error code on failure.
 */
static int cpu_clk_set_wp(int wp)
{
	struct cpu_wp *p;
	u32 reg;
	u32 stat;

	if (wp == cpu_curr_wp)
		return 0;

	p = &cpu_wp_tbl[wp];

	/*
	 * If DDR clock is sourced from PLL1, we cannot drop PLL1 freq.
	 * Use the ARM_PODF to change the freq of the core, leave the PLL1
	 * freq unchanged.
	 */
	if (ddr_clk.parent == &ddr_hf_clk) {
		reg = __raw_readl(MXC_CCM_CACRR);
		reg &= ~MXC_CCM_CACRR_ARM_PODF_MASK;
		reg |= cpu_wp_tbl[wp].cpu_podf << MXC_CCM_CACRR_ARM_PODF_OFFSET;
		__raw_writel(reg, MXC_CCM_CACRR);
		cpu_curr_wp = wp;
		cpu_clk.rate = cpu_wp_tbl[wp].cpu_rate;
	} else {
		struct timespec nstimeofday;
		struct timespec curtime;

		/* Change the ARM clock to requested frequency */
		/* First move the ARM clock to step clock which is running
		 * at 24MHz.
		 */

		/* Change the source of pll1_sw_clk to be the step_clk */
		reg = __raw_readl(MXC_CCM_CCSR);
		reg |= MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
		__raw_writel(reg, MXC_CCM_CCSR);

		/* Stop the PLL */
		reg = __raw_readl(pll1_base + MXC_PLL_DP_CTL);
		reg &= ~MXC_PLL_DP_CTL_UPEN;
		__raw_writel(reg, pll1_base + MXC_PLL_DP_CTL);

		/* PDF and MFI */
		reg = p->pdf | p->mfi << MXC_PLL_DP_OP_MFI_OFFSET;
		__raw_writel(reg, pll1_base + MXC_PLL_DP_OP);

		/* MFD */
		__raw_writel(p->mfd, pll1_base + MXC_PLL_DP_MFD);

		/* MFI */
		__raw_writel(p->mfn, pll1_base + MXC_PLL_DP_MFN);

		reg = __raw_readl(pll1_base + MXC_PLL_DP_CTL);
		reg |= MXC_PLL_DP_CTL_UPEN;
		/* Set the UPEN bits */
		__raw_writel(reg, pll1_base + MXC_PLL_DP_CTL);
		/* Forcefully restart the PLL */
		reg |= MXC_PLL_DP_CTL_RST;
		__raw_writel(reg, pll1_base + MXC_PLL_DP_CTL);

		/* Wait for the PLL to lock */
		getnstimeofday(&nstimeofday);
		do {
			getnstimeofday(&curtime);
			if ((curtime.tv_nsec - nstimeofday.tv_nsec) > SPIN_DELAY)
				panic("pll1 relock failed\n");
			stat = __raw_readl(pll1_base + MXC_PLL_DP_CTL) &
			    MXC_PLL_DP_CTL_LRF;
		} while (!stat);

		reg = __raw_readl(MXC_CCM_CCSR);
		/* Move the PLL1 back to the pll1_main_clk */
		reg &= ~MXC_CCM_CCSR_PLL1_SW_CLK_SEL;
		__raw_writel(reg, MXC_CCM_CCSR);

		cpu_curr_wp = wp;

		pll1_sw_clk.rate = cpu_wp_tbl[wp].cpu_rate;
		pll1_main_clk.rate = pll1_sw_clk.rate;
		cpu_clk.rate = pll1_sw_clk.rate;
	}

#if defined(CONFIG_CPU_FREQ)
	cpufreq_trig_needed = 1;
#endif
	return 0;
}
