/*
 * linux/arch/arm/mach-nuc980/Board-dt-nuc980.c
 *
 * Copyright (C) 2017 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 */

#include <linux/clk.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/pwm.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <mach/map.h>
#include <mach/mfp.h>

#include <mach/irqs.h>
#include <mach/regs-gcr.h>
#include <mach/regs-aic.h>

#include "cpu.h"
#include "pm.h"

extern void nuc980_timer_init(void);
extern void __init nuc980_map_io(void);
extern int __init nuc980_of_init_irq(struct device_node *node, struct device_node *parent);
extern void nuc980_restart(enum reboot_mode mode, const char *cmd);

static void __init nuc980_init_late(void)
{
	nuc980_init_suspend();
}

static void __init nuc980_dt_device_init(void)
{
	printk(KERN_INFO "<DT> %s +\n", __func__);
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	printk(KERN_INFO "<DT> %s -\n", __func__);
}

static const struct of_device_id irq_of_match[] __initconst = {
	{
		.compatible = "nuvoton,nuc980-aic",
		.data = nuc980_of_init_irq
	},
	{ /*sentinel */ }
};

static void __init nuc980_dt_init_irq(void)
{
	of_irq_init(irq_of_match);
}

static const char *nuc980_dt_board_compat[] __initdata = {
	"nuvoton,nuc980",
	"nuvoton,nuc980-dev-v1.0",
	NULL
};

DT_MACHINE_START(nuc980_dt, "Nuvoton NUC980 (Device Tree)")
	.atag_offset = 0x100,
	.init_time = nuc980_timer_init,
	.map_io = nuc980_map_io,
//      .init_early     = nuc980_dt_initialize,
	.init_irq = nuc980_dt_init_irq,
	.init_machine = nuc980_dt_device_init,
	.dt_compat = nuc980_dt_board_compat,
	.init_late = nuc980_init_late,
	.restart = nuc980_restart,
MACHINE_END
