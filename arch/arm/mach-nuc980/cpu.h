/*
 * arch/arm/mach-nuc980/cpu.h
 *
 *
 * Copyright (c) 2017 Nuvoton technology corporation
 * All rights reserved.
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

#include <linux/serial_core.h>

#ifndef __CPU_H__
#define __CPU_H__

#define IODESC_ENT(y)                                  \
{                                                      \
	.virtual = (unsigned long)NUC980_VA_##y,       \
	.pfn     = __phys_to_pfn(NUC980_PA_##y),       \
	.length  = NUC980_SZ_##y,                      \
	.type    = MT_DEVICE,                           \
}

#define NUC980SERIAL_PORT(name)					\
{								\
	.membase	= name##_BA,				\
	.mapbase	= name##_PA,				\
	.irq		= IRQ_##name,				\
	.uartclk	= 12000000,				\
}

#define NUC980PID	NUC980_VA_GCR

extern struct platform_device nuc980_device_sdh;
extern struct platform_device nuc980_device_jpeg;

extern void nuc980_init_irq(void);
extern struct sys_timer nuc980_timer;

extern void nuc980_clock_source(struct device *dev, unsigned char *src);
extern void nuc980_add_clocks(void);
extern void nuc980_init_clocks(void);
extern void nuc980_platform_init(struct platform_device **device, int size);

#endif //__CPU_H__
