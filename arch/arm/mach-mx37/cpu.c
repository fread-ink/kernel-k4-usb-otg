/*
 *  Copyright (C) 2001 Deep Blue Solutions Ltd.
 *  Copyright (C) 2004-2010 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/*!
 * @file mach-mx37/cpu.c
 *
 * @brief This file contains the CPU initialization code.
 *
 * @ingroup MSL_MX37
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iram_alloc.h>
#include <mach/hardware.h>
#include <asm/hardware/cache-l2x0.h>

void __iomem *gpc_base;
void __iomem *ccm_base;

/*!
 * CPU initialization. It is called by fixup_mxc_board()
 */
void __init mxc_cpu_init(void)
{
	if (!system_rev) {
		mxc_set_system_rev(0x37, CHIP_REV_1_0);
	}
}

void mx37_vpu_reset(void)
{
	u32 reg;
	void __iomem *src_base;

	src_base = ioremap(SRC_BASE_ADDR, PAGE_SIZE);

	reg = __raw_readl(src_base);
	reg |= 0x02;    /* SW_VPU_RST_BIT */
	 __raw_writel(reg, src_base);
	while (__raw_readl(src_base) & 0x02)
		;

	iounmap(src_base);
}

#define MXC_ARM1176_BASE	IO_ADDRESS(ARM1176_BASE_ADDR)

/*!
 * Post CPU init code
 *
 * @return 0 always
 */
static int __init post_cpu_init(void)
{
	u32 reg;
	void *l2_base;
	volatile unsigned long aips_reg;
	int iram_size = IRAM_SIZE;

#if defined(CONFIG_MXC_SECURITY_SCC) || defined(CONFIG_MXC_SECURITY_SCC_MODULE)
	iram_size -= SCC_RAM_SIZE;
#endif

	iram_init(IRAM_BASE_ADDR, iram_size);

	gpc_base = ioremap(GPC_BASE_ADDR, SZ_4K);
	ccm_base = ioremap(CCM_BASE_ADDR, SZ_4K);

	/* Set ALP bits to 000. Set ALP_EN bit in Arm Memory Controller reg. */
	reg = __raw_readl(MXC_ARM1176_BASE + 0x1C);
	reg = 0x8;
	__raw_writel(reg, MXC_ARM1176_BASE + 0x1C);

	/* Initialize L2 cache */
	l2_base = ioremap(L2CC_BASE_ADDR, SZ_4K);
	if (l2_base) {
		l2x0_init(l2_base, 0x0003001B, 0x00000000);
	}

	__raw_writel(0x0, IO_ADDRESS(AIPS1_BASE_ADDR + 0x40));
	__raw_writel(0x0, IO_ADDRESS(AIPS1_BASE_ADDR + 0x44));
	__raw_writel(0x0, IO_ADDRESS(AIPS1_BASE_ADDR + 0x48));
	__raw_writel(0x0, IO_ADDRESS(AIPS1_BASE_ADDR + 0x4C));
	aips_reg = __raw_readl(IO_ADDRESS(AIPS1_BASE_ADDR + 0x50));
	aips_reg &= 0x00FFFFFF;
	__raw_writel(aips_reg, IO_ADDRESS(AIPS1_BASE_ADDR + 0x50));

	__raw_writel(0x0, IO_ADDRESS(AIPS2_BASE_ADDR + 0x40));
	__raw_writel(0x0, IO_ADDRESS(AIPS2_BASE_ADDR + 0x44));
	__raw_writel(0x0, IO_ADDRESS(AIPS2_BASE_ADDR + 0x48));
	__raw_writel(0x0, IO_ADDRESS(AIPS2_BASE_ADDR + 0x4C));
	aips_reg = __raw_readl(IO_ADDRESS(AIPS2_BASE_ADDR + 0x50));
	aips_reg &= 0x00FFFFFF;
	__raw_writel(aips_reg, IO_ADDRESS(AIPS2_BASE_ADDR + 0x50));

	return 0;
}

postcore_initcall(post_cpu_init);
