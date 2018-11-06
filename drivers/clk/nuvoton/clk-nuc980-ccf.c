/*
 * linux/arch/arm/mach-nuc980/clk-ccf.c
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
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <mach/hardware.h>
#include <mach/regs-clock.h>
#include <linux/spinlock.h>

#include "clk-nuc980.h"

DEFINE_SPINLOCK(nuc980_lock);

static const char *sys_sel_clks[] = { "xin", "dummy", "apll", "upll", };
static const char *audio_sel_clks[] = { "xin", "dummy", "apll", "upll", };
static const char *usb_sel_clks[] = { "dummy", "dummy", "usbphy0_div", "usbphy1_div", };
static const char *qspi0_sel_clks[] = { "xin", "pclk0_div", "apll", "upll", };
static const char *spi0_sel_clks[] = { "xin", "pclk1_div", "apll", "upll", };
static const char *spi1_sel_clks[] = { "xin", "pclk0_div", "apll", "upll", };
static const char *cap0_sel_clks[] = { "xin", "dummy", "cap0_aplldiv", "cap0_uplldiv", };
static const char *cap1_sel_clks[] = { "xin", "dummy", "cap1_aplldiv", "cap1_uplldiv", };
static const char *uart0_sel_clks[] = { "xin", "xin32k", "apll", "upll", };
static const char *uart1_sel_clks[] = { "xin", "xin32k", "apll", "upll", };
static const char *uart2_sel_clks[] = { "xin", "xin32k", "apll", "upll", };
static const char *uart3_sel_clks[] = { "xin", "xin32k", "apll", "upll", };
static const char *uart4_sel_clks[] = { "xin", "xin32k", "apll", "upll", };
static const char *uart5_sel_clks[] = { "xin", "xin32k", "apll", "upll", };
static const char *uart6_sel_clks[] = { "xin", "xin32k", "apll", "upll", };
static const char *uart7_sel_clks[] = { "xin", "xin32k", "apll", "upll", };
static const char *uart8_sel_clks[] = { "xin", "xin32k", "apll", "upll", };
static const char *uart9_sel_clks[] = { "xin", "xin32k", "apll", "upll", };
static const char *adc_sel_clks[] = { "xin", "dummy", "apll", "upll", };
static const char *wwdt_sel_clks[] = { "xin", "xin512_div", "pclk24096_div", "xin32k", };
static const char *timer0_sel_clks[] = { "xin", "pclk0_div", "pclk4096_div", "xin32k", };
static const char *timer1_sel_clks[] = { "xin", "pclk0_div", "pclk4096_div", "xin32k", };
static const char *timer2_sel_clks[] = { "xin", "pclk1_div", "pclk4096_div", "xin32k", };
static const char *timer3_sel_clks[] = { "xin", "pclk1_div", "pclk4096_div", "xin32k", };
static const char *timer4_sel_clks[] = { "xin", "pclk0_div", "pclk4096_div", "xin32k", };
static const char *timer5_sel_clks[] = { "xin", "pclk0_div", "pclk4096_div", "xin32k", };
static const char *sdh0_sel_clks[] = { "xin", "dummy", "apll", "upll", };
static const char *sdh1_sel_clks[] = { "xin", "dummy", "apll", "upll", };
static const char *cko_sel_clks[] = { "xin", "xin32k", "apll", "upll", };
//static const char *gpio_sel_clks[] = { "xin", "xin32k" };	// -- TODO

enum nuc980_clks {
	// source
	xin, xin32k, apll, upll, usbphy0, usbphy1, xin512_div,
	sys_mux, sys_div, ddr_gate, cpu_div, cpu_gate,
	hclk1_div, hclk1_gate, pdma0_gate, pdma1_gate, tic_gate, ebi_gate, gpio_gate,

	hclk_div, hclk_gate, dram_gate, sram_gate,

	emac1_gate, emac1_eclk_div, emac1_eclk_gate, usbh_gate, usbd_gate,
	fmi_gate, nand_gate, crypto_gate, sdh0_gate, cap1_gate,

	emac0_gate, emac0_eclk_div, emac0_eclk_gate, sdh1_gate, audio_gate, cap0_gate, sensor_gate,

	// eclk
	audio_eclk_mux, audio_eclk_div, audio_eclk_gate,
	usb_eclk_mux,/* usb_eclk_div,*/usbphy0_div, usbphy1_div, usb_eclk_gate,
	qspi0_eclk_mux, qspi0_eclk_gate,
	spi0_eclk_mux, spi0_eclk_gate,
	spi1_eclk_mux, spi1_eclk_gate,
	cap1_aplldiv, cap1_uplldiv, cap1_eclk_mux, cap1_eclk_div, cap1_eclk_gate,
	cap0_aplldiv, cap0_uplldiv, cap0_eclk_mux, cap0_eclk_div, cap0_eclk_gate,
	uart0_eclk_mux, uart0_eclk_div, uart0_eclk_gate,
	uart1_eclk_mux, uart1_eclk_div, uart1_eclk_gate,
	uart2_eclk_mux, uart2_eclk_div, uart2_eclk_gate,
	uart3_eclk_mux, uart3_eclk_div, uart3_eclk_gate,
	uart4_eclk_mux, uart4_eclk_div, uart4_eclk_gate,
	uart5_eclk_mux, uart5_eclk_div, uart5_eclk_gate,
	uart6_eclk_mux, uart6_eclk_div, uart6_eclk_gate,
	uart7_eclk_mux, uart7_eclk_div, uart7_eclk_gate,
	uart8_eclk_mux, uart8_eclk_div, uart8_eclk_gate,
	uart9_eclk_mux, uart9_eclk_div, uart9_eclk_gate,
	smc0_eclk_div, smc0_eclk_gate,
	smc1_eclk_div, smc1_eclk_gate,
	adc_eclk_mux, adc_eclk_div, adc_eclk_gate,
	wwdt_eclk_mux, wwdt_eclk_gate,
	wdt_eclk_mux, wdt_eclk_gate,
	timer0_eclk_mux, timer0_eclk_gate,
	timer1_eclk_mux, timer1_eclk_gate,
	timer2_eclk_mux, timer2_eclk_gate,
	timer3_eclk_mux, timer3_eclk_gate,
	timer4_eclk_mux, timer4_eclk_gate,
	timer5_eclk_mux, timer5_eclk_gate,
	sdh0_eclk_mux, sdh0_eclk_div, sdh0_eclk_gate,
	sdh1_eclk_mux, sdh1_eclk_div, sdh1_eclk_gate,
	cko_eclk_mux, cko_eclk_div, cko_eclk_gate,

	// pclk
	pclk0_div, pclk0_gate, pclk4096_div,
	i2c0_gate, i2c2_gate,
	qspi0_gate, spi1_gate,
	timer0_gate, timer1_gate, timer4_gate, timer5_gate,
	uart0_gate, uart2_gate, uart4_gate,	uart6_gate, uart8_gate,

	pclk1_div, pclk1_gate,
	i2c1_gate, i2c3_gate,
	spi0_gate,
	timer2_gate, timer3_gate,
	uart1_gate, uart3_gate, uart5_gate, uart7_gate, uart9_gate,
	adc_gate,

	pclk2_div, pclk2_gate, pclk24096_div,
	rtc_gate,
	wdt_gate,
	wwdt_gate,
	can0_gate, can1_gate, can2_gate, can3_gate,
	smc0_gate, smc1_gate,
	pwm0_gate, pwm1_gate,

	clk_max
};

static struct clk *clk[clk_max];

int __init nuc980_init_clocks(void)
{
	int i;

	// source
	clk[xin] = nuc980_clk_fixed("xin", 12000000);
	clk[xin32k] = nuc980_clk_fixed("xin32k", 32768);
	clk[apll] = nuc980_clk_apll("apll", "xin", REG_CLK_APLLCON);
	clk[upll] = nuc980_clk_upll("upll", "xin", REG_CLK_UPLLCON);
	// for FPGA
	//clk[apll] = nuc980_clk_fixed("apll", 12000000);
	//clk[upll] = nuc980_clk_fixed("upll", 12000000);

	clk[usbphy0] = nuc980_clk_fixed("usbphy0", 480000000);
	clk[usbphy1] = nuc980_clk_fixed("usbphy1", 480000000);

	clk[xin512_div] = nuc980_clk_fixed_factor("xin512_div", "xin", 1, 512);	//  xin/512

	clk[sys_mux] = nuc980_clk_mux("sys_mux", REG_CLK_DIV0, 3, 2, sys_sel_clks, ARRAY_SIZE(sys_sel_clks));
	clk[sys_div] = nuc980_clk_divider("sys_div", "sys_mux", REG_CLK_DIV0, 8, 1);
	clk[ddr_gate] = nuc980_clk_gate("ddr_gate", "sys_div", REG_CLK_HCLKEN, 10);

	// CPU
	clk[cpu_div] = nuc980_clk_divider("cpu_div", "sys_div", REG_CLK_DIV0, 16, 1);
	clk[cpu_gate] = nuc980_clk_gate("cpu_gate", "cpu_div", REG_CLK_HCLKEN, 0);

	// HCLK1 & PCLK1
	clk[hclk1_div]  = nuc980_clk_fixed_factor("hclk1_div", "sys_div", 1, 2); //  /2
	clk[hclk1_gate] = nuc980_clk_gate("hclk1_gate", "hclk1_div", REG_CLK_HCLKEN, 2);
	clk[pdma0_gate] = nuc980_clk_gate("pdma0_hclk_gate", "hclk1_div", REG_CLK_HCLKEN, 12);
	clk[pdma1_gate] = nuc980_clk_gate("pdma1_hclk_gate", "hclk1_div", REG_CLK_HCLKEN, 13);
	clk[tic_gate] = nuc980_clk_gate("tic_hclk_gate", "hclk1_div", REG_CLK_HCLKEN, 7);
	clk[ebi_gate] = nuc980_clk_gate("ebi_hclk_gate", "hclk1_div", REG_CLK_HCLKEN, 9);
	clk[gpio_gate] = nuc980_clk_gate("gpio_hclk_gate", "hclk1_div", REG_CLK_HCLKEN, 11);
	// clk[gpio_eclk_mux] = nuc980_clk_mux("gpio_eclk_mux", REG_CLK_DIV7, 7, 1, gpio_sel_clks, ARRAY_SIZE(gpio_sel_clks));
	// clk[gpio_eclk_div] = nuc980_clk_divider("gpio_eclk_div", "gpio_eclk_mux", REG_CLK_DIV7, 0, 7);
	// clk[gpio_eclk_gate] = nuc980_clk_gate("gpio_eclk_gate", "gpio_eclk_div", REG_CLK_HCLKEN, 11);

	// HCLK & HCLK234
	clk[hclk_div] = nuc980_clk_fixed_factor("hclk_div", "sys_div", 1, 2); //  /2
	clk[hclk_gate] = nuc980_clk_gate("hclk_gate", "hclk_div", REG_CLK_HCLKEN, 1);
	clk[dram_gate] = nuc980_clk_gate("dram_gate", "hclk_div", REG_CLK_HCLKEN, 10);
	clk[sram_gate] = nuc980_clk_gate("sram_gate", "hclk_gate", REG_CLK_HCLKEN, 8);

	// HCLK3
	clk[emac1_gate] = nuc980_clk_gate("emac1_hclk_gate", "hclk_div", REG_CLK_HCLKEN, 17);
	clk[emac1_eclk_div] = nuc980_clk_divider("emac1_eclk_div", "hclk_div", REG_CLK_DIV8, 0, 8);
	clk[emac1_eclk_gate] = nuc980_clk_gate("emac1_eclk_gate", "emac1_eclk_div", REG_CLK_HCLKEN, 17);
	clk[usbh_gate] = nuc980_clk_gate("usbh_hclk_gate", "hclk_div", REG_CLK_HCLKEN, 18);
	clk[usbd_gate] = nuc980_clk_gate("usbd_hclk_gate", "hclk_div", REG_CLK_HCLKEN, 19);
	clk[fmi_gate] = nuc980_clk_gate("fmi_hclk_gate", "hclk_div", REG_CLK_HCLKEN, 20);
	clk[nand_gate] = nuc980_clk_gate("nand_hclk_gate", "hclk_div", REG_CLK_HCLKEN, 21);
	clk[crypto_gate] = nuc980_clk_gate("crypto_hclk_gate", "hclk_div", REG_CLK_HCLKEN, 23);
	clk[sdh0_gate] = nuc980_clk_gate("sdh0_hclk_gate", "hclk_div", REG_CLK_HCLKEN, 22);
	clk[cap1_gate] = nuc980_clk_gate("cap1_hclk_gate", "hclk_div", REG_CLK_HCLKEN, 31);

	// HCLK4
	clk[emac0_gate] = nuc980_clk_gate("emac0_hclk_gate", "hclk_div", REG_CLK_HCLKEN, 16);
	clk[emac0_eclk_div] = nuc980_clk_divider("emac0_eclk_div", "hclk_div", REG_CLK_DIV8, 0, 8);
	clk[emac0_eclk_gate] = nuc980_clk_gate("emac0_eclk_gate", "emac0_eclk_div", REG_CLK_HCLKEN, 16);
	clk[sdh1_gate] = nuc980_clk_gate("sdh1_hclk_gate", "hclk_div", REG_CLK_HCLKEN, 30);
	clk[audio_gate] = nuc980_clk_gate("audio_hclk_gate", "hclk_div", REG_CLK_HCLKEN, 24);
	clk[cap0_gate] = nuc980_clk_gate("cap0_hclk_gate", "hclk_div", REG_CLK_HCLKEN, 26);
	clk[sensor_gate] = nuc980_clk_gate("sensor_hclk_gate", "hclk_div", REG_CLK_HCLKEN, 27); // ?? block diagram

	// ECLK
	// -AUDIO
	clk[audio_eclk_mux] = nuc980_clk_mux("audio_eclk_mux", REG_CLK_DIV1, 19, 2, audio_sel_clks, ARRAY_SIZE(audio_sel_clks));
	clk[audio_eclk_div] = nuc980_clk_divider("audio_eclk_div", "audio_eclk_mux", REG_CLK_DIV1, 24, 8);
	clk[audio_eclk_gate] = nuc980_clk_gate("audio_eclk_gate", "audio_eclk_div", REG_CLK_HCLKEN, 24);

	// -USB
	clk[usb_eclk_mux] = nuc980_clk_mux("usb_eclk_mux", REG_CLK_DIV2, 3, 2, usb_sel_clks, ARRAY_SIZE(usb_sel_clks));
	clk[usbphy0_div] = nuc980_clk_fixed_factor("usbphy0_div", "usbphy0", 1, 10);	//  usbphy0/10 = 48Hz
	clk[usbphy1_div] = nuc980_clk_fixed_factor("usbphy1_div", "usbphy1", 1, 10);	//  usbphy1/10 = 48Hz
	clk[usb_eclk_gate] = nuc980_clk_gate("usb_eclk_gate", "usbphy0_div", REG_CLK_HCLKEN, 18);

	// -QSPI0
	clk[qspi0_eclk_mux] = nuc980_clk_mux("qspi0_eclk_mux", REG_CLK_DIV2, 8, 2, qspi0_sel_clks, ARRAY_SIZE(qspi0_sel_clks));
	clk[qspi0_eclk_gate] = nuc980_clk_gate("qspi0_eclk_gate", "qspi0_eclk_mux", REG_CLK_PCLKEN1, 4);

	// -SPI0
	clk[spi0_eclk_mux] = nuc980_clk_mux("spi0_eclk_mux", REG_CLK_DIV2, 10, 2, spi0_sel_clks, ARRAY_SIZE(spi0_sel_clks));
	clk[spi0_eclk_gate] = nuc980_clk_gate("spi0_eclk_gate", "spi0_eclk_mux", REG_CLK_PCLKEN1, 5);

	// -SPI1
	clk[spi1_eclk_mux] = nuc980_clk_mux("spi1_eclk_mux", REG_CLK_DIV2, 12, 2, spi1_sel_clks, ARRAY_SIZE(spi1_sel_clks));
	clk[spi1_eclk_gate] = nuc980_clk_gate("spi1_eclk_gate", "spi1_eclk_mux", REG_CLK_PCLKEN1, 6);

	// -CAP1
	clk[cap1_aplldiv] = nuc980_clk_divider("cap1_aplldiv", "apll", REG_CLK_DIV2, 16, 3);
	clk[cap1_uplldiv] = nuc980_clk_divider("cap1_uplldiv", "upll", REG_CLK_DIV2, 16, 3);
	clk[cap1_eclk_mux] = nuc980_clk_mux("cap1_eclk_mux", REG_CLK_DIV2, 19, 2, cap1_sel_clks, ARRAY_SIZE(cap1_sel_clks));
	clk[cap1_eclk_div] = nuc980_clk_divider("cap1_eclk_div", "cap1_eclk_mux", REG_CLK_DIV2, 24, 4);
	clk[cap1_eclk_gate] = nuc980_clk_gate("cap1_eclk_gate", "cap1_eclk_div", REG_CLK_HCLKEN, 31);

	// -CAP0
	clk[cap0_aplldiv] = nuc980_clk_divider("cap0_aplldiv", "apll", REG_CLK_DIV3, 16, 3);
	clk[cap0_uplldiv] = nuc980_clk_divider("cap0_uplldiv", "upll", REG_CLK_DIV3, 16, 3);
	clk[cap0_eclk_mux] = nuc980_clk_mux("cap0_eclk_mux", REG_CLK_DIV3, 19, 2, cap0_sel_clks, ARRAY_SIZE(cap0_sel_clks));
	clk[cap0_eclk_div] = nuc980_clk_divider("cap0_eclk_div", "cap0_eclk_mux", REG_CLK_DIV3, 24, 4);
	clk[cap0_eclk_gate] = nuc980_clk_gate("cap0_eclk_gate", "cap0_eclk_div", REG_CLK_HCLKEN, 26);

	// -UART0
	clk[uart0_eclk_mux] = nuc980_clk_mux("uart0_eclk_mux", REG_CLK_DIV4, 3, 2, uart0_sel_clks, ARRAY_SIZE(uart0_sel_clks));
	clk[uart0_eclk_div] = nuc980_clk_divider("uart0_eclk_div", "uart0_eclk_mux", REG_CLK_DIV4, 5, 3);
	clk[uart0_eclk_gate] = nuc980_clk_gate("uart0_eclk_gate", "uart0_eclk_div", REG_CLK_PCLKEN0, 16);

	// -UART1
	clk[uart1_eclk_mux] = nuc980_clk_mux("uart1_eclk_mux", REG_CLK_DIV4, 11, 2, uart1_sel_clks, ARRAY_SIZE(uart1_sel_clks));
	clk[uart1_eclk_div] = nuc980_clk_divider("uart1_eclk_div", "uart1_eclk_mux", REG_CLK_DIV4, 13, 3);
	clk[uart1_eclk_gate] = nuc980_clk_gate("uart1_eclk_gate", "uart1_eclk_div", REG_CLK_PCLKEN0, 17);

	// -UART2
	clk[uart2_eclk_mux] = nuc980_clk_mux("uart2_eclk_mux", REG_CLK_DIV4, 19, 2, uart2_sel_clks, ARRAY_SIZE(uart2_sel_clks));
	clk[uart2_eclk_div] = nuc980_clk_divider("uart2_eclk_div", "uart2_eclk_mux", REG_CLK_DIV4, 21, 3);
	clk[uart2_eclk_gate] = nuc980_clk_gate("uart2_eclk_gate", "uart2_eclk_div", REG_CLK_PCLKEN0, 18);

	// -UART3
	clk[uart3_eclk_mux] = nuc980_clk_mux("uart3_eclk_mux", REG_CLK_DIV4, 27, 2, uart3_sel_clks, ARRAY_SIZE(uart3_sel_clks));
	clk[uart3_eclk_div] = nuc980_clk_divider("uart3_eclk_div", "uart3_eclk_mux", REG_CLK_DIV4, 29, 3);
	clk[uart3_eclk_gate] = nuc980_clk_gate("uart3_eclk_gate", "uart3_eclk_div", REG_CLK_PCLKEN0, 19);

	// -UART4
	clk[uart4_eclk_mux] = nuc980_clk_mux("uart4_eclk_mux", REG_CLK_DIV5, 3, 2, uart4_sel_clks, ARRAY_SIZE(uart4_sel_clks));
	clk[uart4_eclk_div] = nuc980_clk_divider("uart4_eclk_div", "uart4_eclk_mux", REG_CLK_DIV5, 5, 3);
	clk[uart4_eclk_gate] = nuc980_clk_gate("uart4_eclk_gate", "uart4_eclk_div", REG_CLK_PCLKEN0, 20);

	// -UART5
	clk[uart5_eclk_mux] = nuc980_clk_mux("uart5_eclk_mux", REG_CLK_DIV5, 11, 2, uart5_sel_clks, ARRAY_SIZE(uart5_sel_clks));
	clk[uart5_eclk_div] = nuc980_clk_divider("uart5_eclk_div", "uart5_eclk_mux", REG_CLK_DIV5, 13, 3);
	clk[uart5_eclk_gate] = nuc980_clk_gate("uart5_eclk_gate", "uart5_eclk_div", REG_CLK_PCLKEN0, 21);

	// -UART6
	clk[uart6_eclk_mux] = nuc980_clk_mux("uart6_eclk_mux", REG_CLK_DIV5, 19, 2, uart6_sel_clks, ARRAY_SIZE(uart6_sel_clks));
	clk[uart6_eclk_div] = nuc980_clk_divider("uart6_eclk_div", "uart6_eclk_mux", REG_CLK_DIV5, 21, 3);
	clk[uart6_eclk_gate] = nuc980_clk_gate("uart6_eclk_gate", "uart6_eclk_div", REG_CLK_PCLKEN0, 22);

	// -UART7
	clk[uart7_eclk_mux] = nuc980_clk_mux("uart7_eclk_mux", REG_CLK_DIV5, 27, 2, uart7_sel_clks, ARRAY_SIZE(uart7_sel_clks));
	clk[uart7_eclk_div] = nuc980_clk_divider("uart7_eclk_div", "uart7_eclk_mux", REG_CLK_DIV5, 29, 3);
	clk[uart7_eclk_gate] = nuc980_clk_gate("uart7_eclk_gate", "uart7_eclk_div", REG_CLK_PCLKEN0, 23);

	// -UART8
	clk[uart8_eclk_mux] = nuc980_clk_mux("uart8_eclk_mux", REG_CLK_DIV6, 3, 2, uart8_sel_clks, ARRAY_SIZE(uart8_sel_clks));
	clk[uart8_eclk_div] = nuc980_clk_divider("uart8_eclk_div", "uart8_eclk_mux", REG_CLK_DIV6, 5, 3);
	clk[uart8_eclk_gate] = nuc980_clk_gate("uart8_eclk_gate", "uart8_eclk_div", REG_CLK_PCLKEN0, 24);

	// -UART9
	clk[uart9_eclk_mux] = nuc980_clk_mux("uart9_eclk_mux", REG_CLK_DIV6, 11, 2, uart9_sel_clks, ARRAY_SIZE(uart9_sel_clks));
	clk[uart9_eclk_div] = nuc980_clk_divider("uart9_eclk_div", "uart9_eclk_mux", REG_CLK_DIV6, 13, 3);
	clk[uart9_eclk_gate] = nuc980_clk_gate("uart9_eclk_gate", "uart9_eclk_div", REG_CLK_PCLKEN0, 25);

	// -SMARTCARD
	clk[smc0_eclk_div] = nuc980_clk_divider("smc0_eclk_div", "xin", REG_CLK_DIV6, 24, 4);
	clk[smc0_eclk_gate] = nuc980_clk_gate("smc0_eclk_gate", "smc0_eclk_div", REG_CLK_PCLKEN1, 12);

	clk[smc1_eclk_div] = nuc980_clk_divider("smc1_eclk_div", "xin", REG_CLK_DIV6, 28, 4);
	clk[smc1_eclk_gate] = nuc980_clk_gate("smc1_eclk_gate", "smc1_eclk_div", REG_CLK_PCLKEN1, 13);

	// -ADC
	clk[adc_eclk_mux] = nuc980_clk_mux("adc_eclk_mux", REG_CLK_DIV7, 19, 2, adc_sel_clks, ARRAY_SIZE(adc_sel_clks));
	clk[adc_eclk_div] = nuc980_clk_divider("adc_eclk_div", "adc_eclk_mux", REG_CLK_DIV7, 24, 8);
	clk[adc_eclk_gate] = nuc980_clk_gate("adc_eclk_gate", "adc_eclk_div", REG_CLK_PCLKEN1, 24);

	// -WWDT
	clk[wwdt_eclk_mux] = nuc980_clk_mux("wwdt_eclk_mux", REG_CLK_DIV8, 10, 2, wwdt_sel_clks, ARRAY_SIZE(wwdt_sel_clks));
	clk[wwdt_eclk_gate] = nuc980_clk_gate("wwdt_eclk_gate", "wwdt_eclk_mux", REG_CLK_PCLKEN0, 1);

	// -WDT
	clk[wdt_eclk_mux] = nuc980_clk_mux("wdt_eclk_mux", REG_CLK_DIV8, 8, 2, wwdt_sel_clks, ARRAY_SIZE(wwdt_sel_clks));
	clk[wdt_eclk_gate] = nuc980_clk_gate("wdt_eclk_gate", "wdt_eclk_mux", REG_CLK_PCLKEN0, 0);

	// -timer0
	clk[timer0_eclk_mux] = nuc980_clk_mux("timer0_eclk_mux", REG_CLK_DIV8, 16, 2, timer0_sel_clks, ARRAY_SIZE(timer0_sel_clks));
	clk[timer0_eclk_gate] = nuc980_clk_gate("timer0_eclk_gate", "timer0_eclk_mux", REG_CLK_PCLKEN0, 8);

	// -timer1
	clk[timer1_eclk_mux] = nuc980_clk_mux("timer1_eclk_mux", REG_CLK_DIV8, 18, 2, timer1_sel_clks, ARRAY_SIZE(timer1_sel_clks));
	clk[timer1_eclk_gate] = nuc980_clk_gate("timer1_eclk_gate", "timer1_eclk_mux", REG_CLK_PCLKEN0, 9);

	// -timer2
	clk[timer2_eclk_mux] = nuc980_clk_mux("timer2_eclk_mux", REG_CLK_DIV8, 20, 2, timer2_sel_clks, ARRAY_SIZE(timer2_sel_clks));
	clk[timer2_eclk_gate] = nuc980_clk_gate("timer2_eclk_gate", "timer2_eclk_mux", REG_CLK_PCLKEN0, 10);

	// -timer3
	clk[timer3_eclk_mux] = nuc980_clk_mux("timer3_eclk_mux", REG_CLK_DIV8, 22, 2, timer3_sel_clks, ARRAY_SIZE(timer3_sel_clks));
	clk[timer3_eclk_gate] = nuc980_clk_gate("timer3_eclk_gate", "timer3_eclk_mux", REG_CLK_PCLKEN0, 11);

	// -timer4
	clk[timer4_eclk_mux] = nuc980_clk_mux("timer4_eclk_mux", REG_CLK_DIV8, 24, 2, timer4_sel_clks, ARRAY_SIZE(timer4_sel_clks));
	clk[timer4_eclk_gate] = nuc980_clk_gate("timer4_eclk_gate", "timer4_eclk_mux", REG_CLK_PCLKEN0, 12);

	// -timer5
	clk[timer5_eclk_mux] = nuc980_clk_mux("timer5_eclk_mux", REG_CLK_DIV8, 26, 2, timer5_sel_clks, ARRAY_SIZE(timer5_sel_clks));
	clk[timer5_eclk_gate] = nuc980_clk_gate("timer5_eclk_gate", "timer5_eclk_mux", REG_CLK_PCLKEN0, 13);

	// -SDH0
	clk[sdh0_eclk_mux] = nuc980_clk_mux("sdh0_eclk_mux", REG_CLK_DIV3, 3, 2, sdh0_sel_clks, ARRAY_SIZE(sdh0_sel_clks));
	clk[sdh0_eclk_div] = nuc980_clk_divider("sdh0_eclk_div", "sdh0_eclk_mux", REG_CLK_DIV3, 8, 8);
	clk[sdh0_eclk_gate] = nuc980_clk_gate("sdh0_eclk_gate", "sdh0_eclk_div", REG_CLK_HCLKEN, 22);

	// -SDH1
	clk[sdh1_eclk_mux] = nuc980_clk_mux("sdh1_eclk_mux", REG_CLK_DIV9, 3, 2, sdh1_sel_clks, ARRAY_SIZE(sdh1_sel_clks));
	clk[sdh1_eclk_div] = nuc980_clk_divider("sdh1_eclk_div", "sdh1_eclk_mux", REG_CLK_DIV9, 8, 8);
	clk[sdh1_eclk_gate] = nuc980_clk_gate("sdh1_eclk_gate", "sdh1_eclk_div", REG_CLK_HCLKEN, 30);

	clk[cko_eclk_mux] = nuc980_clk_mux("cko_eclk_mux", REG_CLK_DIV9, 19, 2, cko_sel_clks, ARRAY_SIZE(cko_sel_clks));
	clk[cko_eclk_div] = nuc980_clk_divider("cko_eclk_div", "cko_eclk_mux", REG_CLK_DIV9, 24, 8);
	clk[cko_eclk_gate] = nuc980_clk_gate("cko_eclk_gate", "cko_eclk_div", REG_CLK_HCLKEN, 15);

	// -GPIO
	// clk[gpio_eclk_mux] = nuc980_clk_mux("gpio_eclk_mux", REG_CLK_DIV7, 7, 1, gpio_sel_clks, ARRAY_SIZE(gpio_sel_clks));
	// clk[gpio_eclk_div] = nuc980_clk_divider("gpio_eclk_div", "gpio_eclk_mux", REG_CLK_DIV7, 0, 7);
	// clk[gpio_eclk_gate] = nuc980_clk_gate("gpio_eclk_gate", "gpio_eclk_div", REG_CLK_PCLKEN0, 3);

	// PCLK0
	clk[pclk0_div] = nuc980_clk_fixed_factor("pclk0_div", "hclk1_div", 1, 1);
	clk[pclk0_gate] = nuc980_clk_gate("pclk0_gate", "pclk0_div", REG_CLK_HCLKEN, 5);
	clk[pclk4096_div] = nuc980_clk_fixed_factor("pclk4096_div", "hclk1_div", 1, 4096);	//  pclk/4096
	clk[i2c0_gate] = nuc980_clk_gate("i2c0_gate", "pclk0_div", REG_CLK_PCLKEN1, 0);
	clk[i2c2_gate] = nuc980_clk_gate("i2c2_gate", "pclk0_div", REG_CLK_PCLKEN1, 2);
	clk[qspi0_gate] = nuc980_clk_gate("qspi0_gate", "pclk0_div", REG_CLK_PCLKEN1, 4);
	clk[spi1_gate] = nuc980_clk_gate("spi1_gate", "pclk0_div", REG_CLK_PCLKEN1, 6);
	clk[timer0_gate] = nuc980_clk_gate("timer0_gate", "pclk0_div", REG_CLK_PCLKEN0, 8);
	clk[timer1_gate] = nuc980_clk_gate("timer1_gate", "pclk0_div", REG_CLK_PCLKEN0, 9);
	clk[timer4_gate] = nuc980_clk_gate("timer4_gate", "xin", REG_CLK_PCLKEN0, 12);
	clk[timer5_gate] = nuc980_clk_gate("timer5_gate", "xin", REG_CLK_PCLKEN0, 13);
	clk[uart0_gate] = nuc980_clk_gate("uart0_gate", "pclk0_div", REG_CLK_PCLKEN0, 16);
	clk[uart2_gate] = nuc980_clk_gate("uart2_gate", "pclk0_div", REG_CLK_PCLKEN0, 18);
	clk[uart4_gate] = nuc980_clk_gate("uart4_gate", "pclk0_div", REG_CLK_PCLKEN0, 20);
	clk[uart6_gate] = nuc980_clk_gate("uart6_gate", "pclk0_div", REG_CLK_PCLKEN0, 22);
	clk[uart8_gate] = nuc980_clk_gate("uart8_gate", "pclk0_div", REG_CLK_PCLKEN0, 24);

	clk[pclk1_div] = nuc980_clk_fixed_factor("pclk1_div", "hclk1_div", 1, 1);
	clk[pclk1_gate] = nuc980_clk_gate("pclk1_gate", "pclk1_div", REG_CLK_HCLKEN, 6);
	clk[i2c1_gate] = nuc980_clk_gate("i2c1_gate", "pclk1_div", REG_CLK_PCLKEN1, 1);
	clk[i2c3_gate] = nuc980_clk_gate("i2c3_gate", "pclk1_div", REG_CLK_PCLKEN1, 3);
	clk[spi0_gate] = nuc980_clk_gate("spi0_gate", "pclk1_div", REG_CLK_PCLKEN1, 5);
	clk[timer2_gate] = nuc980_clk_gate("timer2_gate", "pclk1_div", REG_CLK_PCLKEN0, 10);
	clk[timer3_gate] = nuc980_clk_gate("timer3_gate", "pclk1_div", REG_CLK_PCLKEN0, 11);
	clk[uart1_gate] = nuc980_clk_gate("uart1_gate", "pclk1_div", REG_CLK_PCLKEN0, 17);
	clk[uart3_gate] = nuc980_clk_gate("uart3_gate", "pclk1_div", REG_CLK_PCLKEN0, 19);
	clk[uart5_gate] = nuc980_clk_gate("uart5_gate", "pclk1_div", REG_CLK_PCLKEN0, 21);
	clk[uart7_gate] = nuc980_clk_gate("uart7_gate", "pclk1_div", REG_CLK_PCLKEN0, 23);
	clk[uart9_gate] = nuc980_clk_gate("uart9_gate", "pclk1_div", REG_CLK_PCLKEN0, 25);
	clk[adc_gate] = nuc980_clk_gate("adc_gate", "pclk1_div", REG_CLK_PCLKEN1, 24);

	// PCLK2
	clk[pclk2_div] = nuc980_clk_fixed_factor("pclk2_div", "hclk1_div", 1, 2);
	clk[pclk2_gate] = nuc980_clk_gate("pclk2_gate", "pclk2_div", REG_CLK_HCLKEN, 14);
	clk[pclk24096_div] = nuc980_clk_fixed_factor("pclk24096_div", "pclk2_div", 1, 4096);	//  pclk2/4096
	clk[rtc_gate] = nuc980_clk_gate("rtc_gate", "pclk2_div", REG_CLK_PCLKEN0, 2);
	clk[wdt_gate] = nuc980_clk_gate("wdt_gate", "pclk2_div", REG_CLK_PCLKEN0, 0);
	clk[wwdt_gate] = nuc980_clk_gate("wwdt_gate", "pclk2_div", REG_CLK_PCLKEN0, 1);
	clk[can0_gate] = nuc980_clk_gate("can0_gate", "pclk2_div", REG_CLK_PCLKEN1, 8);
	clk[can1_gate] = nuc980_clk_gate("can1_gate", "pclk2_div", REG_CLK_PCLKEN1, 9);
	clk[can2_gate] = nuc980_clk_gate("can2_gate", "pclk2_div", REG_CLK_PCLKEN1, 10);
	clk[can3_gate] = nuc980_clk_gate("can3_gate", "pclk2_div", REG_CLK_PCLKEN1, 11);
	clk[smc0_gate] = nuc980_clk_gate("smc0_gate", "pclk2_div", REG_CLK_PCLKEN1, 12);
	clk[smc1_gate] = nuc980_clk_gate("smc1_gate", "pclk2_div", REG_CLK_PCLKEN1, 13);
	clk[pwm0_gate] = nuc980_clk_gate("pwm0_gate", "pclk2_div", REG_CLK_PCLKEN1, 26);
	clk[pwm1_gate] = nuc980_clk_gate("pwm1_gate", "pclk2_div", REG_CLK_PCLKEN1, 27);

	for (i = 0; i < ARRAY_SIZE(clk); i++)
		if (IS_ERR(clk[i]))
			pr_err("nuc980 clk %d: register failed with %ld\n", i, PTR_ERR(clk[i]));

	clk_register_clkdev(clk[xin], "xin", NULL);
	clk_register_clkdev(clk[xin32k], "xin32k", NULL);
	clk_register_clkdev(clk[apll], "apll", NULL);
	clk_register_clkdev(clk[upll], "upll", NULL);

	clk_register_clkdev(clk[usbphy0], "usbphy0", NULL);
	clk_register_clkdev(clk[usbphy1], "usbphy1", NULL);

	clk_register_clkdev(clk[xin512_div], "xin512_div", NULL);

	// SYS
	clk_register_clkdev(clk[sys_mux], "sysmux", NULL);
	clk_register_clkdev(clk[sys_div], "sysdiv", NULL);

	clk_register_clkdev(clk[ddr_gate], "ddr_hclk", NULL);

	// CPU
	clk_register_clkdev(clk[cpu_div], "cpudiv", NULL);
	clk_register_clkdev(clk[cpu_gate], "cpu", NULL);

	// HCLK1
	clk_register_clkdev(clk[hclk1_div], "hclk1div", NULL);
	clk_register_clkdev(clk[hclk1_gate], "hclk1", NULL);
	clk_register_clkdev(clk[pdma0_gate], "pdma0_hclk", NULL);
	clk_register_clkdev(clk[pdma1_gate], "pdma1_hclk", NULL);
	clk_register_clkdev(clk[tic_gate], "tic_hclk", NULL);
	clk_register_clkdev(clk[ebi_gate], "ebi_hclk", NULL);
	clk_register_clkdev(clk[gpio_gate], "gpio_hclk", NULL);

	// HCLK234
	clk_register_clkdev(clk[hclk_div], "hclkdiv", NULL);
	clk_register_clkdev(clk[hclk_gate], "hclk", NULL);
	clk_register_clkdev(clk[dram_gate], "dram", NULL);
	clk_register_clkdev(clk[sram_gate], "sram", NULL);

	//HCLK3
	clk_register_clkdev(clk[emac1_gate], "emac1_hclk", NULL);
	clk_register_clkdev(clk[emac1_eclk_div], "emac1_eclk_div", NULL);
	clk_register_clkdev(clk[emac1_eclk_gate], "emac1_eclk", NULL);
	clk_register_clkdev(clk[usbh_gate], "usbh_hclk", NULL);
	clk_register_clkdev(clk[usbd_gate], "usbd_hclk", NULL);
	clk_register_clkdev(clk[fmi_gate], "fmi_hclk", NULL);
	clk_register_clkdev(clk[nand_gate], "nand_hclk", NULL);
	clk_register_clkdev(clk[sdh0_gate], "sdh0_hclk", NULL);
	clk_register_clkdev(clk[crypto_gate], "crypto_hclk", NULL);
	clk_register_clkdev(clk[cap1_gate], "cap1_hclk", NULL);

	//HCLK4
	clk_register_clkdev(clk[emac0_gate], "emac0_hclk", NULL);
	clk_register_clkdev(clk[emac0_eclk_div], "emac0_eclk_div", NULL);
	clk_register_clkdev(clk[emac0_eclk_gate], "emac0_eclk", NULL);
	clk_register_clkdev(clk[sdh1_gate], "sdh1_hclk", NULL);
	clk_register_clkdev(clk[audio_gate], "audio_hclk", NULL);
	clk_register_clkdev(clk[cap0_gate], "cap0_hclk", NULL);
	clk_register_clkdev(clk[sensor_gate], "sensor_hclk", NULL);

	// ECLK
	clk_register_clkdev(clk[audio_eclk_mux], "audio_eclk_mux", NULL);
	clk_register_clkdev(clk[audio_eclk_div], "audio_eclk_div", NULL);
	clk_register_clkdev(clk[audio_eclk_gate], "audio_eclk", NULL);

	clk_register_clkdev(clk[usb_eclk_mux], "usb_eclk_mux", NULL);
	clk_register_clkdev(clk[usbphy0_div], "usbphy0_div", NULL);
	clk_register_clkdev(clk[usbphy1_div], "usbphy1_div", NULL);
	clk_register_clkdev(clk[usb_eclk_gate], "usb_eclk", NULL);

	clk_register_clkdev(clk[qspi0_eclk_mux], "qspi0_eclk_mux", NULL);
	clk_register_clkdev(clk[qspi0_eclk_gate], "qspi0_eclk", NULL);

	clk_register_clkdev(clk[spi0_eclk_mux], "spi0_eclk_mux", NULL);
	clk_register_clkdev(clk[spi0_eclk_gate], "spi0_eclk", NULL);

	clk_register_clkdev(clk[spi1_eclk_mux], "spi1_eclk_mux", NULL);
	clk_register_clkdev(clk[spi1_eclk_gate], "spi1_eclk", NULL);

	clk_register_clkdev(clk[cap1_aplldiv], "cap1_aplldiv", NULL);
	clk_register_clkdev(clk[cap1_uplldiv], "cap1_uplldiv", NULL);
	clk_register_clkdev(clk[cap1_eclk_mux], "cap1_eclk_mux", NULL);
	clk_register_clkdev(clk[cap1_eclk_div], "cap1_eclk_div", NULL);
	clk_register_clkdev(clk[cap1_eclk_gate], "cap1_eclk", NULL);

	clk_register_clkdev(clk[cap0_aplldiv], "cap0_aplldiv", NULL);
	clk_register_clkdev(clk[cap0_uplldiv], "cap0_uplldiv", NULL);
	clk_register_clkdev(clk[cap0_eclk_mux], "cap0_eclk_mux", NULL);
	clk_register_clkdev(clk[cap0_eclk_div], "cap0_eclk_div", NULL);
	clk_register_clkdev(clk[cap0_eclk_gate], "cap0_eclk", NULL);

	clk_register_clkdev(clk[uart0_eclk_mux], "uart0_eclk_mux", NULL);
	clk_register_clkdev(clk[uart0_eclk_div], "uart0_eclk_div", NULL);
	clk_register_clkdev(clk[uart0_eclk_gate], "uart0_eclk", NULL);

	clk_register_clkdev(clk[uart1_eclk_mux], "uart1_eclk_mux", NULL);
	clk_register_clkdev(clk[uart1_eclk_div], "uart1_eclk_div", NULL);
	clk_register_clkdev(clk[uart1_eclk_gate], "uart1_eclk", NULL);

	clk_register_clkdev(clk[uart2_eclk_mux], "uart2_eclk_mux", NULL);
	clk_register_clkdev(clk[uart2_eclk_div], "uart2_eclk_div", NULL);
	clk_register_clkdev(clk[uart2_eclk_gate], "uart2_eclk", NULL);

	clk_register_clkdev(clk[uart3_eclk_mux], "uart3_eclk_mux", NULL);
	clk_register_clkdev(clk[uart3_eclk_div], "uart3_eclk_div", NULL);
	clk_register_clkdev(clk[uart3_eclk_gate], "uart3_eclk", NULL);

	clk_register_clkdev(clk[uart4_eclk_mux], "uart4_eclk_mux", NULL);
	clk_register_clkdev(clk[uart4_eclk_div], "uart4_eclk_div", NULL);
	clk_register_clkdev(clk[uart4_eclk_gate], "uart4_eclk", NULL);

	clk_register_clkdev(clk[uart5_eclk_mux], "uart5_eclk_mux", NULL);
	clk_register_clkdev(clk[uart5_eclk_div], "uart5_eclk_div", NULL);
	clk_register_clkdev(clk[uart5_eclk_gate], "uart5_eclk", NULL);

	clk_register_clkdev(clk[uart6_eclk_mux], "uart6_eclk_mux", NULL);
	clk_register_clkdev(clk[uart6_eclk_div], "uart6_eclk_div", NULL);
	clk_register_clkdev(clk[uart6_eclk_gate], "uart6_eclk", NULL);

	clk_register_clkdev(clk[uart7_eclk_mux], "uart7_eclk_mux", NULL);
	clk_register_clkdev(clk[uart7_eclk_div], "uart7_eclk_div", NULL);
	clk_register_clkdev(clk[uart7_eclk_gate], "uart7_eclk", NULL);

	clk_register_clkdev(clk[uart8_eclk_mux], "uart8_eclk_mux", NULL);
	clk_register_clkdev(clk[uart8_eclk_div], "uart8_eclk_div", NULL);
	clk_register_clkdev(clk[uart8_eclk_gate], "uart8_eclk", NULL);

	clk_register_clkdev(clk[uart9_eclk_mux], "uart9_eclk_mux", NULL);
	clk_register_clkdev(clk[uart9_eclk_div], "uart9_eclk_div", NULL);
	clk_register_clkdev(clk[uart9_eclk_gate], "uart9_eclk", NULL);

	clk_register_clkdev(clk[smc0_eclk_div], "smc0_eclk_div", NULL);
	clk_register_clkdev(clk[smc0_eclk_gate], "smc0_eclk", NULL);
	clk_register_clkdev(clk[smc1_eclk_div], "smc1_eclk_div", NULL);
	clk_register_clkdev(clk[smc1_eclk_gate], "smc1_eclk", NULL);

	clk_register_clkdev(clk[adc_eclk_mux], "adc_eclk_mux", NULL);
	clk_register_clkdev(clk[adc_eclk_div], "adc_eclk_div", NULL);
	clk_register_clkdev(clk[adc_eclk_gate], "adc_eclk", NULL);

	clk_register_clkdev(clk[wwdt_eclk_mux], "wwdt_eclk_mux", NULL);
	clk_register_clkdev(clk[wwdt_eclk_gate], "wwdt_eclk", NULL);
	clk_register_clkdev(clk[wdt_eclk_mux], "wdt_eclk_mux", NULL);
	clk_register_clkdev(clk[wdt_eclk_gate], "wdt_eclk", NULL);

	clk_register_clkdev(clk[timer0_eclk_mux], "timer0_eclk_mux", NULL);
	clk_register_clkdev(clk[timer0_eclk_gate], "timer0_eclk", NULL);
	clk_register_clkdev(clk[timer1_eclk_mux], "timer1_eclk_mux", NULL);
	clk_register_clkdev(clk[timer1_eclk_gate], "timer1_eclk", NULL);
	clk_register_clkdev(clk[timer2_eclk_mux], "timer2_eclk_mux", NULL);
	clk_register_clkdev(clk[timer2_eclk_gate], "timer2_eclk", NULL);
	clk_register_clkdev(clk[timer3_eclk_mux], "timer3_eclk_mux", NULL);
	clk_register_clkdev(clk[timer3_eclk_gate], "timer3_eclk", NULL);
	clk_register_clkdev(clk[timer4_eclk_mux], "timer4_eclk_mux", NULL);
	clk_register_clkdev(clk[timer4_eclk_gate], "timer4_eclk", NULL);
	clk_register_clkdev(clk[timer5_eclk_mux], "timer5_eclk_mux", NULL);
	clk_register_clkdev(clk[timer5_eclk_gate], "timer5_eclk", NULL);

	clk_register_clkdev(clk[sdh0_eclk_mux], "sdh0_eclk_mux", NULL);
	clk_register_clkdev(clk[sdh0_eclk_div], "sdh0_eclk_div", NULL);
	clk_register_clkdev(clk[sdh0_eclk_gate], "sdh0_eclk", NULL);

	clk_register_clkdev(clk[sdh1_eclk_mux], "sdh1_eclk_mux", NULL);
	clk_register_clkdev(clk[sdh1_eclk_div], "sdh1_eclk_div", NULL);
	clk_register_clkdev(clk[sdh1_eclk_gate], "sdh1_eclk", NULL);

	clk_register_clkdev(clk[cko_eclk_mux], "cko_eclk_mux", NULL);
	clk_register_clkdev(clk[cko_eclk_div], "cko_eclk_div", NULL);
	clk_register_clkdev(clk[cko_eclk_gate], "cko_eclk", NULL);

	//PCLK
	clk_register_clkdev(clk[pclk0_div], "pclk0div", NULL);
	clk_register_clkdev(clk[pclk0_gate], "pclk0", NULL);
	clk_register_clkdev(clk[pclk4096_div], "pclk4096_div", NULL);
	clk_register_clkdev(clk[i2c0_gate], "i2c0", NULL);
	clk_register_clkdev(clk[i2c2_gate], "i2c2", NULL);
	clk_register_clkdev(clk[qspi0_gate], "qspi0", NULL);
	clk_register_clkdev(clk[spi1_gate], "spi1", NULL);
	clk_register_clkdev(clk[timer0_gate], "timer0", NULL);
	clk_register_clkdev(clk[timer1_gate], "timer1", NULL);
	clk_register_clkdev(clk[timer4_gate], "timer4", NULL);
	clk_register_clkdev(clk[timer5_gate], "timer5", NULL);
	clk_register_clkdev(clk[uart0_gate], "uart0", NULL);
	clk_register_clkdev(clk[uart2_gate], "uart2", NULL);
	clk_register_clkdev(clk[uart4_gate], "uart4", NULL);
	clk_register_clkdev(clk[uart6_gate], "uart6", NULL);
	clk_register_clkdev(clk[uart8_gate], "uart8", NULL);

	clk_register_clkdev(clk[pclk1_div], "pclk1div", NULL);
	clk_register_clkdev(clk[pclk1_gate], "pclk1", NULL);
	clk_register_clkdev(clk[i2c1_gate], "i2c1", NULL);
	clk_register_clkdev(clk[i2c3_gate], "i2c3", NULL);
	clk_register_clkdev(clk[spi0_gate], "spi0", NULL);
	clk_register_clkdev(clk[timer2_gate], "timer2", NULL);
	clk_register_clkdev(clk[timer3_gate], "timer3", NULL);
	clk_register_clkdev(clk[uart1_gate], "uart1", NULL);
	clk_register_clkdev(clk[uart3_gate], "uart3", NULL);
	clk_register_clkdev(clk[uart5_gate], "uart5", NULL);
	clk_register_clkdev(clk[uart7_gate], "uart7", NULL);
	clk_register_clkdev(clk[uart9_gate], "uart9", NULL);
	clk_register_clkdev(clk[adc_gate], "adc", NULL);

	clk_register_clkdev(clk[pclk2_div], "pclk2div", NULL);
	clk_register_clkdev(clk[pclk2_gate], "pclk2", NULL);
	clk_register_clkdev(clk[pclk24096_div], "pclk24096_div", NULL);
	clk_register_clkdev(clk[rtc_gate], "rtc", NULL);
	clk_register_clkdev(clk[wdt_gate], "wdt", NULL);
	clk_register_clkdev(clk[wwdt_gate], "wwdt", NULL);
	clk_register_clkdev(clk[can0_gate], "can0", NULL);
	clk_register_clkdev(clk[can1_gate], "can1", NULL);
	clk_register_clkdev(clk[can2_gate], "can2", NULL);
	clk_register_clkdev(clk[can3_gate], "can3", NULL);
	//clk_register_clkdev(clk[gpio_gate], "gpio", NULL);
	clk_register_clkdev(clk[smc0_gate], "smc0", NULL);
	clk_register_clkdev(clk[smc1_gate], "smc1", NULL);
	clk_register_clkdev(clk[pwm0_gate], "pwm0", NULL);
	clk_register_clkdev(clk[pwm1_gate], "pwm1", NULL);

	// enable some important clocks
	clk_prepare(clk_get(NULL, "cpu"));
	clk_enable(clk_get(NULL, "cpu"));

	clk_prepare(clk_get(NULL, "hclk"));
	clk_enable(clk_get(NULL, "hclk"));

	clk_prepare(clk_get(NULL, "sram"));
	clk_enable(clk_get(NULL, "sram"));

	clk_prepare(clk_get(NULL, "dram"));
	clk_enable(clk_get(NULL, "dram"));

	clk_prepare(clk_get(NULL, "ddr_hclk"));
	clk_enable(clk_get(NULL, "ddr_hclk"));

	return 0;
}
