/*
 * arch/arm/mach-nuc980/include/mach/regs-serial.h
 *
 * Copyright (c) 2017 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARM_REGS_SERIAL_H
#define __ASM_ARM_REGS_SERIAL_H

#define UART0_BA	(NUC980_VA_UART0)
#define UART1_BA	(NUC980_VA_UART1)
#define UART2_BA	(NUC980_VA_UART2)
#define UART3_BA	(NUC980_VA_UART3)
#define UART4_BA	(NUC980_VA_UART4)
#define UART5_BA	(NUC980_VA_UART5)
#define UART6_BA	(NUC980_VA_UART6)
#define UART7_BA	(NUC980_VA_UART7)
#define UART8_BA	(NUC980_VA_UART8)
#define UART9_BA	(NUC980_VA_UART9)

#define UART0_PA	(NUC980_PA_UART0)
#define UART1_PA	(NUC980_PA_UART1)
#define UART2_PA	(NUC980_PA_UART2)
#define UART3_PA	(NUC980_PA_UART3)
#define UART4_PA	(NUC980_PA_UART4)
#define UART5_PA	(NUC980_PA_UART5)
#define UART6_PA	(NUC980_PA_UART6)
#define UART7_PA	(NUC980_PA_UART7)
#define UART8_PA	(NUC980_PA_UART8)
#define UART9_PA	(NUC980_PA_UART9)

struct uart_port;
struct plat_nuc980serial_port {
	unsigned long iobase;	/* io base address */
	void __iomem *membase;	/* ioremap cookie or NULL */
	resource_size_t mapbase;	/* resource base */
	unsigned int irq;	/* interrupt number */
	unsigned int uartclk;	/* UART clock rate */
	void *private_data;
	unsigned int (*serial_in) (struct uart_port *, int);
	void (*serial_out) (struct uart_port *, int, int);
};

#ifndef __ASSEMBLY__

struct nuc980_uart_clksrc {
	const char *name;
	unsigned int divisor;
	unsigned int min_baud;
	unsigned int max_baud;
};

struct nuc980_uartcfg {
	unsigned char hwport;
	unsigned char unused;
	unsigned short flags;
	unsigned long uart_flags;

	unsigned long ucon;
	unsigned long ulcon;
	unsigned long ufcon;

	struct nuc980_uart_clksrc *clocks;
	unsigned int clocks_size;
};

#endif /* __ASSEMBLY__ */

#endif /* __ASM_ARM_REGS_SERIAL_H */
