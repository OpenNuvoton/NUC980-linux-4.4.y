/* linux/arch/arm/mach-nuc980/pm.c
 *
 * Copyright (c) 2018 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/cpu_pm.h>
#include <linux/suspend.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <mach/regs-clock.h>
#include <mach/regs-aic.h>
#include <mach/regs-gcr.h>
#include <mach/map.h>

#ifdef CONFIG_PM_SLEEP

static int nuc980_suspend_enter(suspend_state_t state)
{
	u32 upll_div;
	if (state != PM_SUSPEND_MEM)
		return -EINVAL;

	// clear bit 0 so NUC980 enter pd mode instead of idle in next function call
	__raw_writel(__raw_readl(REG_CLK_PMCON) & ~1, REG_CLK_PMCON);
	upll_div=__raw_readl(NUC980_VA_CLK+0x64);
	__raw_writel(0xC0000015,NUC980_VA_CLK+0x64); //Set UPLL to 264Mhz
	udelay(2);
	cpu_do_idle();
	__raw_writel(upll_div,NUC980_VA_CLK+0x64); //Restore UPLL
	udelay(2);
	printk(KERN_INFO "Wake up source: %08x  %08x\n", __raw_readl(REG_WKUPSSR0), __raw_readl(REG_WKUPSSR1));
	// clear wake up source
	__raw_writel(__raw_readl(REG_WKUPSSR0), REG_WKUPSSR0);
	__raw_writel(__raw_readl(REG_WKUPSSR1), REG_WKUPSSR1);

	return 0;
}

static const struct platform_suspend_ops nuc980_suspend_ops = {
	.valid = suspend_valid_only_mem,
	.enter = nuc980_suspend_enter,
};

void __init nuc980_init_suspend(void)
{
	__raw_writel(__raw_readl(REG_CLK_PMCON) & ~0xFF0000, REG_CLK_PMCON);	// reduce wake up delay time waiting for HXT stable
	suspend_set_ops(&nuc980_suspend_ops);
}
#endif
