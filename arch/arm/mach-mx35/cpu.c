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

/*!
 * @file mach-mx35/cpu.c
 *
 * @brief This file contains the CPU initialization code.
 *
 * @ingroup MSL_MX35
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iram_alloc.h>
#include <mach/hardware.h>
#include <asm/hardware/cache-l2x0.h>

/*!
 * CPU initialization. It is called by fixup_mxc_board()
 */
void __init mxc_cpu_init(void)
{

	/* Setup Peripheral Port Remap register for AVIC */
	asm("ldr r0, =0xC0000015				\n\
	 mcr p15, 0, r0, c15, c2, 4");
	/*TODO:Add code to check chip version */

	if (!system_rev)
		mxc_set_system_rev(0x35, CHIP_REV_1_0);

}

/*!
 * Post CPU init code
 *
 * @return 0 always
 */
static int __init post_cpu_init(void)
{
	void *l2_base;
	unsigned long aips_reg;

	/* Initialize L2 cache */
	l2_base = ioremap(L2CC_BASE_ADDR, SZ_4K);
	if (l2_base)
		l2x0_init(l2_base, 0x00030024, 0x00000000);

	iram_init(MX35_IRAM_BASE_ADDR, MX35_IRAM_SIZE);

	/*
	 * S/W workaround: Clear the off platform peripheral modules
	 * Supervisor Protect bit for SDMA to access them.
	 */
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
