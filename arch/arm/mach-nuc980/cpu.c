/*
 * linux/arch/arm/mach-nuc980/cpu.c
 *
 * Copyright (c) 2017 Nuvoton Technology Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/serial_core.h>
#include <linux/delay.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>

#include <mach/hardware.h>
#include <mach/regs-gcr.h>
#include <mach/regs-serial.h>
#include <mach/regs-clock.h>

#include "cpu.h"


#ifdef CONFIG_DEBUG_LL
u32 nuc980_uart_init(void){

	__raw_writel(__raw_readl(REG_CLK_PCLKEN0) | 0x10000,REG_CLK_PCLKEN0);
	__raw_writel((__raw_readl(REG_MFP_GPF_H) & 0xfff00fff) | 0x11000,REG_MFP_GPF_H);  // UART0 multi-function
	__raw_writel(__raw_readl(NUC980_VA_UART0+0x0C)|0x7,NUC980_VA_UART0+0x0C);
	__raw_writel(0x30000066,NUC980_VA_UART0+0x24);
	return 0;
}

#endif

void nuc980_restart(enum reboot_mode mode, const char *cmd)
{
	unsigned long flags;

	local_irq_save(flags);
	//UnlockReg
	while (__raw_readl(NUC980_VA_GCR + 0x1fc) != 1) {
		__raw_writel(0x59, NUC980_VA_GCR + 0x1fc);
		__raw_writel(0x16, NUC980_VA_GCR + 0x1fc);
		__raw_writel(0x88, NUC980_VA_GCR + 0x1fc);
	}

	__raw_writel(1, REG_AHBIPRST);	// System reset...
	local_irq_restore(flags);
}
EXPORT_SYMBOL(nuc980_restart);
