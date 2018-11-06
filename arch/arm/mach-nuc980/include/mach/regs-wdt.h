/*
 * arch/arm/mach-nuc980/include/mach/regs-wdt.h
 *
 * Copyright (c) 2018 Nuvoton Technology Corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARCH_REGS_WDT_H
#define __ASM_ARCH_REGS_WDT_H

/* WDT Registers */

#define WDT_BA			NUC980_VA_WDT
#define REG_WDT_CTL		(WDT_BA+0x00)
#define REG_WDT_ALTCTL		(WDT_BA+0x04)
#define REG_WDT_RSTCNT		(WDT_BA+0x08)
#endif /*  __ASM_ARCH_REGS_WDT_H */
