 /*
  * linux/arch/arm/mach-nuc980/mach-nuc980.c
  *
  * Copyright (C) 2017 Nuvoton technology corporation.
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

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/memblock.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <mach/map.h>
#include <mach/mfp.h>

#include <mach/irqs.h>
#include <mach/regs-gcr.h>
#include <mach/regs-aic.h>
#include "cpu.h"
#include "pm.h"

/* Initial IO mappings */
static struct map_desc nuc980_iodesc[] __initdata = {
	IODESC_ENT(IRQ),
	IODESC_ENT(GCR_CLK),
	IODESC_ENT(SDIC),
	IODESC_ENT(EBI),
	IODESC_ENT(GPIO),
	IODESC_ENT(EMAC0),
	IODESC_ENT(EMAC1),
	IODESC_ENT(PDMA0),
	IODESC_ENT(PDMA1),
	IODESC_ENT(EHCI),
	IODESC_ENT(OHCI),
	IODESC_ENT(USBDEV),
	IODESC_ENT(I2S),
	IODESC_ENT(SDH),
	IODESC_ENT(FMI),
	IODESC_ENT(CAP0),
	IODESC_ENT(CAP1),
	IODESC_ENT(CRYPTO),
	IODESC_ENT(WDT_WWDT),
	IODESC_ENT(RTC),
	IODESC_ENT(SC0),
	IODESC_ENT(SC1),
	IODESC_ENT(I2C0),
	IODESC_ENT(I2C1),
	IODESC_ENT(I2C2),
	IODESC_ENT(I2C3),
	IODESC_ENT(UART0),
	IODESC_ENT(UART1),
	IODESC_ENT(UART2),
	IODESC_ENT(UART3),
	IODESC_ENT(UART4),
	IODESC_ENT(UART5),
	IODESC_ENT(UART6),
	IODESC_ENT(UART7),
	IODESC_ENT(UART8),
	IODESC_ENT(UART9),
	IODESC_ENT(SPI0),
	IODESC_ENT(SPI1),
	IODESC_ENT(SPI2),
	IODESC_ENT(TIMER01),
	IODESC_ENT(TIMER23),
	IODESC_ENT(TIMER45),
	IODESC_ENT(PWM0),
	IODESC_ENT(PWM1),
	IODESC_ENT(ADC),
	IODESC_ENT(CAN0),
	IODESC_ENT(CAN1),
	IODESC_ENT(CAN2),
	IODESC_ENT(CAN3),
	IODESC_ENT(SRAM),
};

extern void nuc980_restart(enum reboot_mode mode, const char *cmd);
extern void nuc980_timer_init(void);
static struct platform_device *nuc980_dev[] __initdata = {

};

void __init nuc980_map_io(void)
{
	iotable_init(nuc980_iodesc, ARRAY_SIZE(nuc980_iodesc));
}

static void __init nuc980_init(void)
{
	nuc980_platform_init(nuc980_dev, ARRAY_SIZE(nuc980_dev));
}

static void __init nuc980_init_late(void)
{
	nuc980_init_suspend();
}

static void __init nuc980_reserve_memory(void)
{
	if(memblock_reserve(0, 1024) < 0)
		printk("Failed to reserve memory 0x0~0x400\n");
}

MACHINE_START(NUC980, "NUC980")
	.atag_offset = 0x100,
	.map_io = nuc980_map_io,
	.init_irq = nuc980_init_irq,
	.init_machine = nuc980_init,
	.init_time = nuc980_timer_init,
	.init_late = nuc980_init_late,
	.reserve = nuc980_reserve_memory,
	.restart = nuc980_restart,
MACHINE_END
