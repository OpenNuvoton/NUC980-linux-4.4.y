/*
 * Copyright (c) 2018 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <mach/map.h>
#include <mach/regs-gcr.h>

// The numbering is not related to actual layout.
const struct pinctrl_pin_desc nuc980_pins[] = {
	PINCTRL_PIN(0x00, "PA0"),
	PINCTRL_PIN(0x01, "PA1"),
	PINCTRL_PIN(0x02, "PA2"),
	PINCTRL_PIN(0x03, "PA3"),
	PINCTRL_PIN(0x04, "PA4"),
	PINCTRL_PIN(0x05, "PA5"),
	PINCTRL_PIN(0x06, "PA6"),
	PINCTRL_PIN(0x07, "PA7"),
	PINCTRL_PIN(0x08, "PA8"),
	PINCTRL_PIN(0x09, "PA9"),
	PINCTRL_PIN(0x0A, "PA10"),
	PINCTRL_PIN(0x0B, "PA11"),
	PINCTRL_PIN(0x0C, "PA12"),
	PINCTRL_PIN(0x0D, "PA13"),
	PINCTRL_PIN(0x0E, "PA14"),
	PINCTRL_PIN(0x0F, "PA15"),
	PINCTRL_PIN(0x10, "PB0"),
	PINCTRL_PIN(0x11, "PB1"),
	PINCTRL_PIN(0x12, "PB2"),
	PINCTRL_PIN(0x13, "PB3"),
	PINCTRL_PIN(0x14, "PB4"),
	PINCTRL_PIN(0x15, "PB5"),
	PINCTRL_PIN(0x16, "PB6"),
	PINCTRL_PIN(0x17, "PB7"),
	PINCTRL_PIN(0x18, "PB8"),
	PINCTRL_PIN(0x19, "PB9"),
	PINCTRL_PIN(0x1A, "PB10"),
	PINCTRL_PIN(0x1B, "PB11"),
	PINCTRL_PIN(0x1C, "PB12"),
	PINCTRL_PIN(0x1D, "PB13"),
	PINCTRL_PIN(0x1E, "PB14"),
	PINCTRL_PIN(0x1F, "PB15"),
	PINCTRL_PIN(0x20, "PC0"),
	PINCTRL_PIN(0x21, "PC1"),
	PINCTRL_PIN(0x22, "PC2"),
	PINCTRL_PIN(0x23, "PC3"),
	PINCTRL_PIN(0x24, "PC4"),
	PINCTRL_PIN(0x25, "PC5"),
	PINCTRL_PIN(0x26, "PC6"),
	PINCTRL_PIN(0x27, "PC7"),
	PINCTRL_PIN(0x28, "PC8"),
	PINCTRL_PIN(0x29, "PC9"),
	PINCTRL_PIN(0x2A, "PC10"),
	PINCTRL_PIN(0x2B, "PC11"),
	PINCTRL_PIN(0x2C, "PC12"),
	PINCTRL_PIN(0x2D, "PC13"),
	PINCTRL_PIN(0x2E, "PC14"),
	PINCTRL_PIN(0x2F, "PC15"),
	PINCTRL_PIN(0x30, "PD0"),
	PINCTRL_PIN(0x31, "PD1"),
	PINCTRL_PIN(0x32, "PD2"),
	PINCTRL_PIN(0x33, "PD3"),
	PINCTRL_PIN(0x34, "PD4"),
	PINCTRL_PIN(0x35, "PD5"),
	PINCTRL_PIN(0x36, "PD6"),
	PINCTRL_PIN(0x37, "PD7"),
	PINCTRL_PIN(0x38, "PD8"),
	PINCTRL_PIN(0x39, "PD9"),
	PINCTRL_PIN(0x3A, "PD10"),
	PINCTRL_PIN(0x3B, "PD11"),
	PINCTRL_PIN(0x3C, "PD12"),
	PINCTRL_PIN(0x3D, "PD13"),
	PINCTRL_PIN(0x3E, "PD14"),
	PINCTRL_PIN(0x3F, "PD15"),
	PINCTRL_PIN(0x40, "PE0"),
	PINCTRL_PIN(0x41, "PE1"),
	PINCTRL_PIN(0x42, "PE2"),
	PINCTRL_PIN(0x43, "PE3"),
	PINCTRL_PIN(0x44, "PE4"),
	PINCTRL_PIN(0x45, "PE5"),
	PINCTRL_PIN(0x46, "PE6"),
	PINCTRL_PIN(0x47, "PE7"),
	PINCTRL_PIN(0x48, "PE8"),
	PINCTRL_PIN(0x49, "PE9"),
	PINCTRL_PIN(0x4A, "PE10"),
	PINCTRL_PIN(0x4B, "PE11"),
	PINCTRL_PIN(0x4C, "PE12"),
	PINCTRL_PIN(0x4D, "PE13"),
	PINCTRL_PIN(0x4E, "PE14"),
	PINCTRL_PIN(0x4F, "PE15"),
	PINCTRL_PIN(0x50, "PF0"),
	PINCTRL_PIN(0x51, "PF1"),
	PINCTRL_PIN(0x52, "PF2"),
	PINCTRL_PIN(0x53, "PF3"),
	PINCTRL_PIN(0x54, "PF4"),
	PINCTRL_PIN(0x55, "PF5"),
	PINCTRL_PIN(0x56, "PF6"),
	PINCTRL_PIN(0x57, "PF7"),
	PINCTRL_PIN(0x58, "PF8"),
	PINCTRL_PIN(0x59, "PF9"),
	PINCTRL_PIN(0x5A, "PF10"),
	PINCTRL_PIN(0x5B, "PF11"),
	PINCTRL_PIN(0x5C, "PF12"),
	PINCTRL_PIN(0x5D, "PF13"),
	PINCTRL_PIN(0x5E, "PF14"),
	PINCTRL_PIN(0x5F, "PF15"),
	PINCTRL_PIN(0x60, "PG0"),
	PINCTRL_PIN(0x61, "PG1"),
	PINCTRL_PIN(0x62, "PG2"),
	PINCTRL_PIN(0x63, "PG3"),
	PINCTRL_PIN(0x64, "PG4"),
	PINCTRL_PIN(0x65, "PG5"),
	PINCTRL_PIN(0x66, "PG6"),
	PINCTRL_PIN(0x67, "PG7"),
	PINCTRL_PIN(0x68, "PG8"),
	PINCTRL_PIN(0x69, "PG9"),
	PINCTRL_PIN(0x6A, "PG10"),
	PINCTRL_PIN(0x6B, "PG11"),
	PINCTRL_PIN(0x6C, "PG12"),
	PINCTRL_PIN(0x6D, "PG13"),
	PINCTRL_PIN(0x6E, "PG14"),
	PINCTRL_PIN(0x6F, "PG15"),
};



struct nuc980_pinctrl_group {
	const char *name;
	const unsigned int *pins;
	const unsigned num_pins;
	const unsigned func;
};

static const unsigned nadc_pins[] = {
#ifdef CONFIG_IIO_NUC980ADC_Ch0
	0x10,
#endif
#ifdef CONFIG_IIO_NUC980ADC_Ch1
	0x11,
#endif
#ifdef CONFIG_IIO_NUC980ADC_Ch2
	0x12,
#endif
#ifdef CONFIG_IIO_NUC980ADC_Ch3
	0x13,
#endif
#ifdef CONFIG_IIO_NUC980ADC_Ch4
	0x14,
#endif
#ifdef CONFIG_IIO_NUC980ADC_Ch5
	0x15,
#endif
#ifdef CONFIG_IIO_NUC980ADC_Ch6
	0x16,
#endif
#ifdef CONFIG_IIO_NUC980ADC_Ch7
	0x17
#endif
}; // Port B
static const unsigned emac0_pins[] = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49}; // Port E
static const unsigned emac1_pins[] = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59}; // Port F
//static const unsigned pps0_pin[] = {0x5E};
//static const unsigned pps1_pin[] = {0x4D};

static const unsigned vcap0_pins[] = {0x23, 0x24, 0x25, 0x26, /*0x27,*/ 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F};
static const unsigned vcap1_pins[] = {0x4C, 0x5A, 0x40, 0x41, /*0x4A,*/ 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49};

static const unsigned sd0_pins[] = {0x2C, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A};
static const unsigned sd1_pins[] = {0x56, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55};

static const unsigned nand_pins[] = {0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F};

static const unsigned usbd_pin[] = {0x4B};  // vbvld

static const unsigned i2c0_0_pins[] = {0x00, 0x01};
static const unsigned i2c0_1_pins[] = {0x0F, 0x6A};
static const unsigned i2c0_2_pins[] = {0x4A, 0x4C};

static const unsigned i2c1_0_pins[] = {0x0D, 0x0E};
static const unsigned i2c1_1_pins[] = {0x16, 0x14};
static const unsigned i2c1_2_pins[] = {0x23, 0x24};

static const unsigned i2c2_0_pins[] = {0x17, 0x15};
static const unsigned i2c2_1_pins[] = {0x18, 0x20};

static const unsigned i2c3_0_pins[] = {0x11, 0x13};
static const unsigned i2c3_1_pins[] = {0x3E, 0x3F};

static const unsigned i2s_0_pins[] = {0x02, 0x03, 0x04, 0x05, 0x06};
static const unsigned i2s_1_pins[] = {0x16, 0x14, 0x17, 0x15, 0x11};

static const unsigned uart0_pins[] = {0x5C, 0x5B}; // tx, rx

static const unsigned uart1_0_pins[] = {0x01, 0x00}; // tx, rx
static const unsigned uart1_1_pins[] = {0x25, 0x26}; // tx, rx
static const unsigned uart1_2_pins[] = {0x27, 0x28}; // rts, cts
static const unsigned uart1_3_pins[] = {0x58, 0x57}; // rts, cts
static const unsigned uart1_4_pins[] = {0x5A, 0x59}; // tx, rx

static const unsigned uart2_0_pins[] = {0x08, 0x07}; // rts, cts
static const unsigned uart2_1_pins[] = {0x0A, 0x09}; // tx, rx
static const unsigned uart2_2_pins[] = {0x63, 0x62}; // rts, cts
static const unsigned uart2_3_pins[] = {0x08, 0x10}; // rts, cts
static const unsigned uart2_4_pins[] = {0x61, 0x60}; // tx, rx
static const unsigned uart2_5_pins[] = {0x36, 0x37}; // tx, rx

static const unsigned uart3_0_pins[] = {0x23, 0x24}; // tx, rx
static const unsigned uart3_1_pins[] = {0x19, 0x1A}; // tx, rx
static const unsigned uart3_2_pins[] = {0x1B, 0x1C}; // rts, cts
static const unsigned uart3_3_pins[] = {0x32, 0x33}; // tx, rx
static const unsigned uart3_4_pins[] = {0x34, 0x35}; // rts, cts
static const unsigned uart3_5_pins[] = {0x55, 0x54}; // rts, cts
static const unsigned uart3_6_pins[] = {0x1D, 0x56}; // tx, rx
static const unsigned uart3_7_pins[] = {0x57, 0x56}; // tx, rx

static const unsigned uart4_0_pins[] = {0x29, 0x2A}; // tx, rx
static const unsigned uart4_1_pins[] = {0x3C, 0x3D}; // tx, rx
static const unsigned uart4_2_pins[] = {0x3E, 0x3F}; // rts, cts
static const unsigned uart4_3_pins[] = {0x41, 0x40}; // rts, cts
static const unsigned uart4_4_pins[] = {0x43, 0x42}; // tx, rx

static const unsigned uart5_0_pins[] = {0x67, 0x66}; // tx, rx
static const unsigned uart5_1_pins[] = {0x65, 0x64}; // rts, cts
static const unsigned uart5_2_pins[] = {0x30, 0x31}; // tx, rx
static const unsigned uart5_3_pins[] = {0x6C, 0x6B}; // rts, cts
static const unsigned uart5_4_pins[] = {0x6E, 0x6D}; // tx, rx

static const unsigned uart6_0_pins[] = {0x03, 0x02}; // rts, cts
static const unsigned uart6_1_pins[] = {0x05, 0x04}; // tx, rx
static const unsigned uart6_2_pins[] = {0x39, 0x38}; // rts, cts
static const unsigned uart6_3_pins[] = {0x3A, 0x3B}; // tx, rx
static const unsigned uart6_4_pins[] = {0x49, 0x48}; // tx, rx

static const unsigned uart7_0_pins[] = {0x0D, 0x0E}; // tx, rx
static const unsigned uart7_1_pins[] = {0x16, 0x14}; // tx, rx
static const unsigned uart7_2_pins[] = {0x15, 0x17}; // rts, cts
static const unsigned uart7_3_pins[] = {0x21, 0x22}; // tx, rx
static const unsigned uart7_4_pins[] = {0x51, 0x50}; // rts, cts
static const unsigned uart7_5_pins[] = {0x53, 0x52}; // tx, rx

static const unsigned uart8_0_pins[] = {0x0C, 0x0B}; // tx, rx
static const unsigned uart8_1_pins[] = {0x68, 0x69}; // rts, cts
static const unsigned uart8_2_pins[] = {0x18, 0x20}; // tx, rx
static const unsigned uart8_3_pins[] = {0x2C, 0x2D}; // tx, rx
static const unsigned uart8_4_pins[] = {0x2E, 0x2F}; // cts, rts

static const unsigned uart9_0_pins[] = {0x11, 0x13}; // tx, rx
//static const unsigned uart9_1_pins[] = {0x12, 0x44}; // rts, cts
static const unsigned uart9_2_pins[] = {0x45, 0x44}; // rts, cts
static const unsigned uart9_3_pins[] = {0x47, 0x46}; // tx, rx
static const unsigned uart9_4_pins[] = {0x4C, 0x4A}; // tx, rx

static const unsigned sc0_0_pins[] = {0x02, 0x03, 0x04, 0x05, 0x06};
static const unsigned sc0_1_pins[] = {0x2B, 0x2C, 0x2D, 0x2E, 0x2F};
static const unsigned sc0_2_pins[] = {0x04, 0x05};  // scuart
static const unsigned sc0_3_pins[] = {0x2C, 0x2D};  // scuart
static const unsigned sc1_0_pins[] = {0x26, 0x27, 0x28, 0x29, 0x2A};
static const unsigned sc1_1_pins[] = {0x50, 0x51, 0x52, 0x53, 0x54};
static const unsigned sc1_2_pins[] = {0x27, 0x28};  // scuart
static const unsigned sc1_3_pins[] = {0x51, 0x52};  // scuart

static const unsigned qspi0_0_pins[] = {0x00, 0x32, 0x33, 0x34, 0x35}; // ss1: PA0
static const unsigned qspi0_1_pins[] = {0x30, 0x32, 0x33, 0x34, 0x35}; // ss1: PD0
static const unsigned qspi0_2_pins[] = {0x32, 0x33, 0x34, 0x35,0x36, 0x37}; // quad
static const unsigned qspi0_3_pins[] = {0x32, 0x33, 0x34, 0x35}; // normal
static const unsigned qspi0_4_pins[] = {0x00, 0x32, 0x33, 0x34, 0x35,0x36, 0x37}; // quad ss1:PA0
static const unsigned qspi0_5_pins[] = {0x30, 0x32, 0x33, 0x34, 0x35,0x36, 0x37}; // quad ss1:PD0

static const unsigned spi0_0_pins[] = {0x38, 0x39, 0x3A, 0x3B};
static const unsigned spi0_1_pins[] = {0x31, 0x38, 0x39, 0x3A, 0x3B}; // ss1: PD1
static const unsigned spi0_2_pins[] = {0x6F, 0x38, 0x39, 0x3A, 0x3B}; // ss1: PG15
static const unsigned spi0_3_pins[] = {0x25, 0x26, 0x27, 0x28};
static const unsigned spi0_4_pins[] = {0x20, 0x25, 0x26, 0x27, 0x28}; // ss1: PC0

static const unsigned spi1_0_pins[] = {0x19, 0x1A, 0x1B, 0x1C};
static const unsigned spi1_1_pins[] = {0x6B, 0x6C, 0x6D, 0x6E};
static const unsigned spi1_2_pins[] = {0x6F, 0x6B, 0x6C, 0x6D, 0x6E}; // ss1: PG15
static const unsigned spi1_3_pins[] = {0x16, 0x14, 0x17, 0x15};
static const unsigned spi1_4_pins[] = {0x11, 0x16, 0x14, 0x17, 0x15}; // ss1: PB1

static const unsigned can0_0_pins[] = {0x23, 0x24};
static const unsigned can0_1_pins[] = {0x36, 0x37};
static const unsigned can0_2_pins[] = {0x6B, 0x6C};
static const unsigned can0_3_pins[] = {0x40, 0x41};
static const unsigned can1_0_pins[] = {0x0D, 0x0E};
static const unsigned can1_1_pins[] = {0x3E, 0x3F};
static const unsigned can1_2_pins[] = {0x6D, 0x6E};
static const unsigned can1_3_pins[] = {0x42, 0x43};
static const unsigned can2_0_pins[] = {0x0F, 0x6A};
static const unsigned can2_1_pins[] = {0x11, 0x13};
static const unsigned can2_2_pins[] = {0x18, 0x20};
static const unsigned can2_3_pins[] = {0x3C, 0x3D};
static const unsigned can2_4_pins[] = {0x44, 0x45};
static const unsigned can3_0_pins[] = {0x00, 0x01};
static const unsigned can3_1_pins[] = {0x46, 0x47};
static const unsigned can3_2_pins[] = {0x4A, 0x4C};

static const unsigned pwm00_0_pin[] = {0x6A};
static const unsigned pwm01_0_pin[] = {0x0F};
static const unsigned pwm02_0_pin[] = {0x0E};
static const unsigned pwm03_0_pin[] = {0x0D};
static const unsigned pwm00_1_pin[] = {0x60};
static const unsigned pwm01_1_pin[] = {0x61};
static const unsigned pwm02_1_pin[] = {0x62};
static const unsigned pwm03_1_pin[] = {0x63};
static const unsigned pwm00_2_pin[] = {0x3C};
static const unsigned pwm01_2_pin[] = {0x3D};
static const unsigned pwm02_2_pin[] = {0x3E};
static const unsigned pwm03_2_pin[] = {0x3F};
static const unsigned pwm00_3_pin[] = {0x55};
static const unsigned pwm01_3_pin[] = {0x56};
static const unsigned pwm02_3_pin[] = {0x57};
static const unsigned pwm03_3_pin[] = {0x58};
static const unsigned pwm02_4_pin[] = {0x1D};

static const unsigned pwm10_0_pin[] = {0x66};
static const unsigned pwm11_0_pin[] = {0x67};
static const unsigned pwm12_0_pin[] = {0x68};
static const unsigned pwm13_0_pin[] = {0x69};
static const unsigned pwm10_1_pin[] = {0x1C};
static const unsigned pwm11_1_pin[] = {0x1B};
static const unsigned pwm12_1_pin[] = {0x1A};
static const unsigned pwm13_1_pin[] = {0x19};
static const unsigned pwm10_2_pin[] = {0x6B};
static const unsigned pwm11_2_pin[] = {0x6C};
static const unsigned pwm12_2_pin[] = {0x6D};
static const unsigned pwm13_2_pin[] = {0x6E};
static const unsigned pwm10_3_pin[] = {0x59};
static const unsigned pwm11_3_pin[] = {0x5A};
static const unsigned pwm12_3_pin[] = {0x4A};
static const unsigned pwm13_3_pin[] = {0x4C};

static const unsigned etimer0_0_pin[] = {0x00}; // ecnt
static const unsigned etimer0_1_pin[] = {0x13}; // tgl
static const unsigned etimer0_2_pin[] = {0x11}; // cap
static const unsigned etimer0_3_pin[] = {0x20}; // tgl
static const unsigned etimer0_4_pin[] = {0x18}; // cap
static const unsigned etimer0_5_pin[] = {0x19}; // tgl
static const unsigned etimer0_6_pin[] = {0x1A}; // cap
static const unsigned etimer0_7_pin[] = {0x36}; // ecnt
static const unsigned etimer0_8_pin[] = {0x50}; // ecnt

static const unsigned etimer1_0_pin[] = {0x01}; // ecnt
static const unsigned etimer1_1_pin[] = {0x0E}; // tgl
static const unsigned etimer1_2_pin[] = {0x0D}; // cap
static const unsigned etimer1_3_pin[] = {0x30}; // tgl
static const unsigned etimer1_4_pin[] = {0x31}; // cap
static const unsigned etimer1_5_pin[] = {0x37}; // ecnt
static const unsigned etimer1_6_pin[] = {0x6B}; // tgl
static const unsigned etimer1_7_pin[] = {0x6C}; // cap
static const unsigned etimer1_8_pin[] = {0x51}; // ecnt
static const unsigned etimer1_9_pin[] = {0x58}; // tgl
static const unsigned etimer1_A_pin[] = {0x59}; // cap

static const unsigned etimer2_0_pin[] = {0x02}; // ecnt
static const unsigned etimer2_1_pin[] = {0x0A}; // tgl
static const unsigned etimer2_2_pin[] = {0x09}; // cap
static const unsigned etimer2_3_pin[] = {0x1C}; // tgl
static const unsigned etimer2_4_pin[] = {0x1B}; // cap
static const unsigned etimer2_5_pin[] = {0x38}; // ecnt
static const unsigned etimer2_6_pin[] = {0x3C}; // tgl
static const unsigned etimer2_7_pin[] = {0x3D}; // cap
static const unsigned etimer2_8_pin[] = {0x52}; // ecnt

static const unsigned etimer3_0_pin[] = {0x03}; // ecnt
static const unsigned etimer3_1_pin[] = {0x08}; // tgl
static const unsigned etimer3_2_pin[] = {0x07}; // cap
static const unsigned etimer3_3_pin[] = {0x39}; // ecnt
static const unsigned etimer3_4_pin[] = {0x3E}; // tgl
static const unsigned etimer3_5_pin[] = {0x3F}; // cap
static const unsigned etimer3_6_pin[] = {0x53}; // ecnt

static const unsigned etimer4_0_pin[] = {0x04}; // ecnt
static const unsigned etimer4_1_pin[] = {0x0C}; // tgl
static const unsigned etimer4_2_pin[] = {0x0B}; // cap
static const unsigned etimer4_3_pin[] = {0x33}; // tgl
static const unsigned etimer4_4_pin[] = {0x32}; // cap
static const unsigned etimer4_5_pin[] = {0x3A}; // ecnt
static const unsigned etimer4_6_pin[] = {0x54}; // ecnt
static const unsigned etimer4_7_pin[] = {0x1D}; // tgl
static const unsigned etimer4_8_pin[] = {0x56}; // cap

static const unsigned etimer5_0_pin[] = {0x05}; // ecnt
static const unsigned etimer5_1_pin[] = {0x6A}; // tgl
static const unsigned etimer5_2_pin[] = {0x0F}; // cap
static const unsigned etimer5_3_pin[] = {0x35}; // tgl
static const unsigned etimer5_4_pin[] = {0x34}; // cap
static const unsigned etimer5_5_pin[] = {0x3B}; // ecnt
static const unsigned etimer5_6_pin[] = {0x55}; // ecnt
static const unsigned etimer5_7_pin[] = {0x5A}; // tgl
static const unsigned etimer5_8_pin[] = {0x57}; // cap

static const unsigned jtag0_pins[] = {0x6B, 0x6C, 0x6D, 0x6E, 0x6F};
static const unsigned jtag1_pins[] = {0x02, 0x03, 0x04, 0x05, 0x06};

static const unsigned eint0_0_pin[] = {0x00};
static const unsigned eint0_1_pin[] = {0x0D};
static const unsigned eint1_0_pin[] = {0x01};
static const unsigned eint1_1_pin[] = {0x0E};
static const unsigned eint2_0_pin[] = {0x30};
static const unsigned eint2_1_pin[] = {0x4A};
static const unsigned eint2_2_pin[] = {0x13};
static const unsigned eint2_3_pin[] = {0x1D};
static const unsigned eint3_0_pin[] = {0x31};
static const unsigned eint3_1_pin[] = {0x4C};
static const unsigned eint3_2_pin[] = {0x6F};

static const unsigned ebi8_0_pin[] =    {0x09, 0x08, 0x07, // nCS0, nRE, nWE,
                                         0x60, 0x61, 0x12, 0x63, 0x66, 0x67, 0x68, 0x69, 0x0C, 0x0B, //address0~9
                                         0x0A, 0x18, 0x10, 0x0D, 0x0E, 0x17, 0x15, 0x11, 0x13, 0x0F, //address10~19
                                         0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27 // data0~7
                                        };
static const unsigned ebi8_1_pin[] =    {0x09, 0x08, 0x07, // nCS0, nRE, nWE,
                                         0x60, 0x61, 0x62, 0x63, 0x66, 0x67, 0x68, 0x69, 0x0C, 0x0B, //address0~9
                                         0x0A, 0x18, 0x65, 0x16, 0x14, 0x17, 0x15, 0x11, 0x64, 0x0F, //address10~19
                                         0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27 // data0~7
                                        };
static const unsigned ebi8_2_pin[] =    {0x06, 0x08, 0x07, // nCS1, nRE, nWE,
                                         0x60, 0x61, 0x12, 0x63, 0x66, 0x67, 0x68, 0x69, 0x0C, 0x0B, //address0~9
                                         0x0A, 0x18, 0x10, 0x0D, 0x0E, 0x17, 0x15, 0x11, 0x13, 0x0F, //address10~19
                                         0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27 // data0~7
                                        };
static const unsigned ebi8_3_pin[] =    {0x06, 0x08, 0x07, // nCS1, nRE, nWE,
                                         0x60, 0x61, 0x62, 0x63, 0x66, 0x67, 0x68, 0x69, 0x0C, 0x0B, //address0~9
                                         0x0A, 0x18, 0x65, 0x16, 0x14, 0x17, 0x15, 0x11, 0x64, 0x0F, //address10~19
                                         0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27 // data0~7
                                        };
static const unsigned ebi8_4_pin[] =    {0x01, 0x08, 0x07, // nCS2, nRE, nWE,
                                         0x60, 0x61, 0x12, 0x63, 0x66, 0x67, 0x68, 0x69, 0x0C, 0x0B, //address0~9
                                         0x0A, 0x18, 0x10, 0x0D, 0x0E, 0x17, 0x15, 0x11, 0x13, 0x0F, //address10~19
                                         0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27 // data0~7
                                        };
static const unsigned ebi8_5_pin[] =    {0x01, 0x08, 0x07, // nCS2, nRE, nWE,
                                         0x60, 0x61, 0x62, 0x63, 0x66, 0x67, 0x68, 0x69, 0x0C, 0x0B, //address0~9
                                         0x0A, 0x18, 0x65, 0x16, 0x14, 0x17, 0x15, 0x11, 0x64, 0x0F, //address10~19
                                         0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27 // data0~7
                                        };

static const unsigned ebi16_0_pin[] =   {0x09, 0x08, 0x07, // nCS0, nRE, nWE,
                                         0x60, 0x61, 0x12, 0x63, 0x66, 0x67, 0x68, 0x69, 0x0C, 0x0B, //address0~9
                                         0x0A, 0x18, 0x10, 0x0D, 0x0E, 0x17, 0x15, 0x11, 0x13, 0x0F, //address10~19
                                         0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, // data0~7
                                         0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F // data8~data15
                                        };
static const unsigned ebi16_1_pin[] =   {0x09, 0x08, 0x07, // nCS0, nRE, nWE,
                                         0x60, 0x61, 0x62, 0x63, 0x66, 0x67, 0x68, 0x69, 0x0C, 0x0B, //address0~9
                                         0x0A, 0x18, 0x65, 0x16, 0x14, 0x17, 0x15, 0x11, 0x64, 0x0F, //address10~19
                                         0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, // data0~7
                                         0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F // data8~data15
                                        };
static const unsigned ebi16_2_pin[] =   {0x06, 0x08, 0x07, // nCS1, nRE, nWE,
                                         0x60, 0x61, 0x12, 0x63, 0x66, 0x67, 0x68, 0x69, 0x0C, 0x0B, //address0~9
                                         0x0A, 0x18, 0x10, 0x0D, 0x0E, 0x17, 0x15, 0x11, 0x13, 0x0F, //address10~19
                                         0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, // data0~7
                                         0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F // data8~data15
                                        };
static const unsigned ebi16_3_pin[] =   {0x06, 0x08, 0x07, // nCS1, nRE, nWE,
                                         0x60, 0x61, 0x62, 0x63, 0x66, 0x67, 0x68, 0x69, 0x0C, 0x0B, //address0~9
                                         0x0A, 0x18, 0x65, 0x16, 0x14, 0x17, 0x15, 0x11, 0x64, 0x0F, //address10~19
                                         0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, // data0~7
                                         0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F // data8~data15
                                        };
static const unsigned ebi16_4_pin[] =   {0x01, 0x08, 0x07, // nCS2, nRE, nWE,
                                         0x60, 0x61, 0x12, 0x63, 0x66, 0x67, 0x68, 0x69, 0x0C, 0x0B, //address0~9
                                         0x0A, 0x18, 0x10, 0x0D, 0x0E, 0x17, 0x15, 0x11, 0x13, 0x0F, //address10~19
                                         0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, // data0~7
                                         0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F // data8~data15
                                        };
static const unsigned ebi16_5_pin[] =   {0x01, 0x08, 0x07, // nCS2, nRE, nWE,
                                         0x60, 0x61, 0x62, 0x63, 0x66, 0x67, 0x68, 0x69, 0x0C, 0x0B, //address0~9
                                         0x0A, 0x18, 0x65, 0x16, 0x14, 0x17, 0x15, 0x11, 0x64, 0x0F, //address10~19
                                         0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, // data0~7
                                         0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F // data8~data15
                                        };

static const unsigned usbh_pwren_ovc_pin[]   = {0x4c, 0x4a};  // USBH_PWREN PE.12 and USB_OVC PE.10
static const unsigned usbh_pwren_pin[]       = {0x4c};        // USBH_PWREN PE.12
static const unsigned usbh_ovc_pin[]         = {0x4a};        // USB_OVC PE.10


static const unsigned usbh_lite0_pb4_pb6[]   = {0x14, 0x16};   // USBH Lite0 D+/D- is PB.4/PB.6
static const unsigned usbh_lite0_pb5_pb7[]   = {0x15, 0x17};   // USBH Lite0 D+/D- is PB.5/PB.7
static const unsigned usbh_lite0_pb10_pb9[]  = {0x1a, 0x19};   // USBH Lite0 D+/D- is PB.10/PB.9
static const unsigned usbh_lite0_pd15_pd14[] = {0x3f, 0x3e};   // USBH Lite0 D+/D- is PD.15/PD.14

static const unsigned usbh_lite1_pe1_pe0[]   = {0x41, 0x40};   // USBH Lite1 D+/D- is PE.1/PE.0
static const unsigned usbh_lite1_pf1_pf0[]   = {0x51, 0x50};   // USBH Lite1 D+/D- is PF.1/PF.0

static const unsigned usbh_lite2_pe3_pe2[]   = {0x43, 0x42};   // USBH Lite2 D+/D- is PE.3/PE.2
static const unsigned usbh_lite2_pf3_pf2[]   = {0x53, 0x52};   // USBH Lite2 D+/D- is PF.3/PF.2

static const unsigned usbh_lite3_pe5_pe4[]   = {0x45, 0x44};   // USBH Lite3 D+/D- is PE.5/PE.4
static const unsigned usbh_lite3_pf5_pf4[]   = {0x55, 0x54};   // USBH Lite3 D+/D- is PF.5/PF.4

static const unsigned usbh_lite4_pe7_pe6[]   = {0x47, 0x46};   // USBH Lite4 D+/D- is PE.7/PE.6
static const unsigned usbh_lite4_pf7_pf6[]   = {0x57, 0x56};   // USBH Lite4 D+/D- is PF.7/PF.6
static const unsigned usbh_lite4_pg10_pa15[] = {0x6a, 0x0f};   // USBH Lite4 D+/D- is PG.10/PA.15
static const unsigned usbh_lite4_pb13_pf6[]  = {0x1d, 0x56};   // USBH Lite4 D+/D- is PB.13/PF.6

static const unsigned usbh_lite5_pe9_pe8[]   = {0x49, 0x48};   // USBH Lite5 D+/D- is PE.9/PE.8
static const unsigned usbh_lite5_pf9_pf8[]   = {0x59, 0x58};   // USBH Lite5 D+/D- is PF.9/PF.8
static const unsigned usbh_lite5_pa14_pa13[] = {0x0e, 0x0d};   // USBH Lite5 D+/D- is PA.14/PA.13
static const unsigned usbh_lite5_pb12_pb11[] = {0x1c, 0x1b};   // USBH Lite5 D+/D- is PB.12/PB.11

// TODO: CKO

static const struct nuc980_pinctrl_group nuc980_pinctrl_groups[] = {
	{
		.name = "nadc_grp",
		.pins = nadc_pins,
		.num_pins = ARRAY_SIZE(nadc_pins),
		.func = 0x8,
	},
	{
		.name = "emac0_grp",
		.pins = emac0_pins,
		.num_pins = ARRAY_SIZE(emac0_pins),
		.func = 0x1,
	},
	{
		.name = "emac1_grp",
		.pins = emac1_pins,
		.num_pins = ARRAY_SIZE(emac1_pins),
		.func = 0x1,
	},
	{
		.name = "vcap0_grp",
		.pins = vcap0_pins,
		.num_pins = ARRAY_SIZE(vcap0_pins),
		.func = 0x2,
	},
	{
		.name = "vcap1_grp",
		.pins = vcap1_pins,
		.num_pins = ARRAY_SIZE(vcap1_pins),
		.func = 0x7,
	},
	{
		.name = "sd0_grp",
		.pins = sd0_pins,
		.num_pins = ARRAY_SIZE(sd0_pins),
		.func = 0x6,
	},
	{
		.name = "sd1_grp",
		.pins = sd1_pins,
		.num_pins = ARRAY_SIZE(sd1_pins),
		.func = 0x2,
	},
	{
		.name = "nand_grp",
		.pins = nand_pins,
		.num_pins = ARRAY_SIZE(nand_pins),
		.func = 0x3,
	},
	{
		.name = "usbd_grp",
		.pins = usbd_pin,
		.num_pins = ARRAY_SIZE(usbd_pin),
		.func = 0x1,
	},
	{
		.name = "i2c0_0_grp",
		.pins = i2c0_0_pins,
		.num_pins = ARRAY_SIZE(i2c0_0_pins),
		.func = 0x3,
	},
	{
		.name = "i2c0_1_grp",
		.pins = i2c0_1_pins,
		.num_pins = ARRAY_SIZE(i2c0_1_pins),
		.func = 0x2,
	},
	{
		.name = "i2c0_2_grp",
		.pins = i2c0_2_pins,
		.num_pins = ARRAY_SIZE(i2c0_2_pins),
		.func = 0x6,
	},
	{
		.name = "i2c1_0_grp",
		.pins = i2c1_0_pins,
		.num_pins = ARRAY_SIZE(i2c1_0_pins),
		.func = 0x2,
	},
	{
		.name = "i2c1_1_grp",
		.pins = i2c1_1_pins,
		.num_pins = ARRAY_SIZE(i2c1_1_pins),
		.func = 0x2,
	},
	{
		.name = "i2c1_2_grp",
		.pins = i2c1_2_pins,
		.num_pins = ARRAY_SIZE(i2c1_2_pins),
		.func = 0x4,
	},
	{
		.name = "i2c2_0_grp",
		.pins = i2c2_0_pins,
		.num_pins = ARRAY_SIZE(i2c2_0_pins),
		.func = 0x2,
	},
	{
		.name = "i2c2_1_grp",
		.pins = i2c2_1_pins,
		.num_pins = ARRAY_SIZE(i2c2_1_pins),
		.func = 0x2,
	},
	{
		.name = "i2c3_0_grp",
		.pins = i2c3_0_pins,
		.num_pins = ARRAY_SIZE(i2c3_0_pins),
		.func = 0x2,
	},
	{
		.name = "i2c3_1_grp",
		.pins = i2c3_1_pins,
		.num_pins = ARRAY_SIZE(i2c3_1_pins),
		.func = 0x3,
	},
	{
		.name = "i2s_0_grp",
		.pins = i2s_0_pins,
		.num_pins = ARRAY_SIZE(i2s_0_pins),
		.func = 0x2,
	},
	{
		.name = "i2s_1_grp",
		.pins = i2s_1_pins,
		.num_pins = ARRAY_SIZE(i2s_1_pins),
		.func = 0x3,
	},
	{
		.name = "uart0_grp",
		.pins = uart0_pins,
		.num_pins = ARRAY_SIZE(uart0_pins),
		.func = 0x1,
	},
	{
		.name = "uart1_0_grp",
		.pins = uart1_0_pins,
		.num_pins = ARRAY_SIZE(uart1_0_pins),
		.func = 0x4,
	},
	{
		.name = "uart1_1_grp",
		.pins = uart1_1_pins,
		.num_pins = ARRAY_SIZE(uart1_1_pins),
		.func = 0x7,
	},
	{
		.name = "uart1_2_grp",
		.pins = uart1_2_pins,
		.num_pins = ARRAY_SIZE(uart1_2_pins),
		.func = 0x7,
	},
	{
		.name = "uart1_3_grp",
		.pins = uart1_3_pins,
		.num_pins = ARRAY_SIZE(uart1_3_pins),
		.func = 0x2,
	},
	{
		.name = "uart1_4_grp",
		.pins = uart1_4_pins,
		.num_pins = ARRAY_SIZE(uart1_4_pins),
		.func = 0x2,
	},
	{
		.name = "uart2_0_grp",
		.pins = uart2_0_pins,
		.num_pins = ARRAY_SIZE(uart2_0_pins),
		.func = 0x2,
	},
	{
		.name = "uart2_1_grp",
		.pins = uart2_1_pins,
		.num_pins = ARRAY_SIZE(uart2_1_pins),
		.func = 0x2,
	},
	{
		.name = "uart2_2_grp",
		.pins = uart2_2_pins,
		.num_pins = ARRAY_SIZE(uart2_2_pins),
		.func = 0x2,
	},
	{
		.name = "uart2_3_grp",
		.pins = uart2_3_pins,
		.num_pins = ARRAY_SIZE(uart2_3_pins),
		.func = 0x2,
	},
	{
		.name = "uart2_4_grp",
		.pins = uart2_4_pins,
		.num_pins = ARRAY_SIZE(uart2_4_pins),
		.func = 0x2,
	},
	{
		.name = "uart2_5_grp",
		.pins = uart2_5_pins,
		.num_pins = ARRAY_SIZE(uart2_5_pins),
		.func = 0x2,
	},
	{
		.name = "uart3_0_grp",
		.pins = uart3_0_pins,
		.num_pins = ARRAY_SIZE(uart3_0_pins),
		.func = 0x5,
	},
	{
		.name = "uart3_1_grp",
		.pins = uart3_1_pins,
		.num_pins = ARRAY_SIZE(uart3_1_pins),
		.func = 0x1,
	},
	{
		.name = "uart3_2_grp",
		.pins = uart3_2_pins,
		.num_pins = ARRAY_SIZE(uart3_2_pins),
		.func = 0x1,
	},
	{
		.name = "uart3_3_grp",
		.pins = uart3_3_pins,
		.num_pins = ARRAY_SIZE(uart3_3_pins),
		.func = 0x2,
	},
	{
		.name = "uart3_4_grp",
		.pins = uart3_4_pins,
		.num_pins = ARRAY_SIZE(uart3_4_pins),
		.func = 0x2,
	},
	{
		.name = "uart3_5_grp",
		.pins = uart3_5_pins,
		.num_pins = ARRAY_SIZE(uart3_5_pins),
		.func = 0x5,
	},
	{
		.name = "uart3_6_grp",
		.pins = uart3_6_pins,
		.num_pins = ARRAY_SIZE(uart3_6_pins),
		.func = 0x5,
	},
	{
		.name = "uart3_7_grp",
		.pins = uart3_7_pins,
		.num_pins = ARRAY_SIZE(uart3_7_pins),
		.func = 0x5,
	},
	{
		.name = "uart4_0_grp",
		.pins = uart4_0_pins,
		.num_pins = ARRAY_SIZE(uart4_0_pins),
		.func = 0x7,
	},
	{
		.name = "uart4_1_grp",
		.pins = uart4_1_pins,
		.num_pins = ARRAY_SIZE(uart4_1_pins),
		.func = 0x1,
	},
	{
		.name = "uart4_2_grp",
		.pins = uart4_2_pins,
		.num_pins = ARRAY_SIZE(uart4_2_pins),
		.func = 0x1,
	},
	{
		.name = "uart4_3_grp",
		.pins = uart4_3_pins,
		.num_pins = ARRAY_SIZE(uart4_3_pins),
		.func = 0x5,
	},
	{
		.name = "uart4_4_grp",
		.pins = uart4_4_pins,
		.num_pins = ARRAY_SIZE(uart4_4_pins),
		.func = 0x5,
	},
	{
		.name = "uart5_0_grp",
		.pins = uart5_0_pins,
		.num_pins = ARRAY_SIZE(uart5_0_pins),
		.func = 0x2,
	},
	{
		.name = "uart5_1_grp",
		.pins = uart5_1_pins,
		.num_pins = ARRAY_SIZE(uart5_1_pins),
		.func = 0x2,
	},
	{
		.name = "uart5_2_grp",
		.pins = uart5_2_pins,
		.num_pins = ARRAY_SIZE(uart5_2_pins),
		.func = 0x2,
	},
	{
		.name = "uart5_3_grp",
		.pins = uart5_3_pins,
		.num_pins = ARRAY_SIZE(uart5_3_pins),
		.func = 0x5,
	},
	{
		.name = "uart5_4_grp",
		.pins = uart5_4_pins,
		.num_pins = ARRAY_SIZE(uart5_4_pins),
		.func = 0x5,
	},
	{
		.name = "uart6_0_grp",
		.pins = uart6_0_pins,
		.num_pins = ARRAY_SIZE(uart6_0_pins),
		.func = 0x1,
	},
	{
		.name = "uart6_1_grp",
		.pins = uart6_1_pins,
		.num_pins = ARRAY_SIZE(uart6_1_pins),
		.func = 0x1,
	},
	{
		.name = "uart6_2_grp",
		.pins = uart6_2_pins,
		.num_pins = ARRAY_SIZE(uart6_2_pins),
		.func = 0x2,
	},
	{
		.name = "uart6_3_grp",
		.pins = uart6_3_pins,
		.num_pins = ARRAY_SIZE(uart6_3_pins),
		.func = 0x2,
	},
	{
		.name = "uart6_4_grp",
		.pins = uart6_4_pins,
		.num_pins = ARRAY_SIZE(uart6_4_pins),
		.func = 0x5,
	},
	{
		.name = "uart7_0_grp",
		.pins = uart7_0_pins,
		.num_pins = ARRAY_SIZE(uart7_0_pins),
		.func = 0x6,
	},
	{
		.name = "uart7_1_grp",
		.pins = uart7_1_pins,
		.num_pins = ARRAY_SIZE(uart7_1_pins),
		.func = 0x5,
	},
	{
		.name = "uart7_2_grp",
		.pins = uart7_2_pins,
		.num_pins = ARRAY_SIZE(uart7_2_pins),
		.func = 0x5,
	},
	{
		.name = "uart7_3_grp",
		.pins = uart7_3_pins,
		.num_pins = ARRAY_SIZE(uart7_3_pins),
		.func = 0x4,
	},
	{
		.name = "uart7_4_grp",
		.pins = uart7_4_pins,
		.num_pins = ARRAY_SIZE(uart7_4_pins),
		.func = 0x5,
	},
	{
		.name = "uart7_5_grp",
		.pins = uart7_5_pins,
		.num_pins = ARRAY_SIZE(uart7_5_pins),
		.func = 0x5,
	},
	{
		.name = "uart8_0_grp",
		.pins = uart8_0_pins,
		.num_pins = ARRAY_SIZE(uart8_0_pins),
		.func = 0x2,
	},
	{
		.name = "uart8_1_grp",
		.pins = uart8_1_pins,
		.num_pins = ARRAY_SIZE(uart8_1_pins),
		.func = 0x2,
	},
	{
		.name = "uart8_2_grp",
		.pins = uart8_2_pins,
		.num_pins = ARRAY_SIZE(uart8_2_pins),
		.func = 0x4,
	},
	{
		.name = "uart8_3_grp",
		.pins = uart8_3_pins,
		.num_pins = ARRAY_SIZE(uart8_3_pins),
		.func = 0x7,
	},
	{
		.name = "uart8_4_grp",
		.pins = uart8_4_pins,
		.num_pins = ARRAY_SIZE(uart8_4_pins),
		.func = 0x7,
	},
	{
		.name = "uart9_0_grp",
		.pins = uart9_0_pins,
		.num_pins = ARRAY_SIZE(uart9_0_pins),
		.func = 0x7,
	},
	{
		.name = "uart9_2_grp",
		.pins = uart9_2_pins,
		.num_pins = ARRAY_SIZE(uart9_2_pins),
		.func = 0x5,
	},
	{
		.name = "uart9_3_grp",
		.pins = uart9_3_pins,
		.num_pins = ARRAY_SIZE(uart9_3_pins),
		.func = 0x5,
	},
	{
		.name = "uart9_4_grp",
		.pins = uart9_4_pins,
		.num_pins = ARRAY_SIZE(uart9_4_pins),
		.func = 0x3,
	},
	{
		.name = "sc0_0_grp",
		.pins = sc0_0_pins,
		.num_pins = ARRAY_SIZE(sc0_0_pins),
		.func = 0x3,
	},
	{
		.name = "sc0_1_grp",
		.pins = sc0_1_pins,
		.num_pins = ARRAY_SIZE(sc0_1_pins),
		.func = 0x4,
	},
	{
		.name = "sc0_2_grp",
		.pins = sc0_2_pins,
		.num_pins = ARRAY_SIZE(sc0_2_pins),
		.func = 0x3,
	},
	{
		.name = "sc0_3_grp",
		.pins = sc0_3_pins,
		.num_pins = ARRAY_SIZE(sc0_3_pins),
		.func = 0x4,
	},
	{
		.name = "sc1_0_grp",
		.pins = sc1_0_pins,
		.num_pins = ARRAY_SIZE(sc1_0_pins),
		.func = 0x4,
	},
	{
		.name = "sc1_1_grp",
		.pins = sc1_1_pins,
		.num_pins = ARRAY_SIZE(sc1_1_pins),
		.func = 0x4,
	},
	{
		.name = "sc1_2_grp",
		.pins = sc1_2_pins,
		.num_pins = ARRAY_SIZE(sc1_2_pins),
		.func = 0x4,
	},
	{
		.name = "sc1_3_grp",
		.pins = sc1_3_pins,
		.num_pins = ARRAY_SIZE(sc1_3_pins),
		.func = 0x4,
	},
	{
		.name = "qspi0_0_grp",
		.pins = qspi0_0_pins,
		.num_pins = ARRAY_SIZE(qspi0_0_pins),
		.func = 0x1,
	},
	{
		.name = "qspi0_1_grp",
		.pins = qspi0_1_pins,
		.num_pins = ARRAY_SIZE(qspi0_1_pins),
		.func = 0x1,
	},
	{
		.name = "qspi0_2_grp",
		.pins = qspi0_2_pins,
		.num_pins = ARRAY_SIZE(qspi0_2_pins),
		.func = 0x1,
	},
	{
		.name = "qspi0_3_grp",
		.pins = qspi0_3_pins,
		.num_pins = ARRAY_SIZE(qspi0_3_pins),
		.func = 0x1,
	},
	{
		.name = "qspi0_4_grp",
		.pins = qspi0_4_pins,
		.num_pins = ARRAY_SIZE(qspi0_4_pins),
		.func = 0x1,
	},
	{
		.name = "qspi0_5_grp",
		.pins = qspi0_5_pins,
		.num_pins = ARRAY_SIZE(qspi0_5_pins),
		.func = 0x1,
	},
	{
		.name = "spi0_0_grp",
		.pins = spi0_0_pins,
		.num_pins = ARRAY_SIZE(spi0_0_pins),
		.func = 0x1,
	},
	{
		.name = "spi0_1_grp",
		.pins = spi0_1_pins,
		.num_pins = ARRAY_SIZE(spi0_1_pins),
		.func = 0x1,
	},
	{
		.name = "spi0_2_grp",
		.pins = spi0_2_pins,
		.num_pins = ARRAY_SIZE(spi0_2_pins),
		.func = 0x1,
	},
	{
		.name = "spi0_3_grp",
		.pins = spi0_3_pins,
		.num_pins = ARRAY_SIZE(spi0_3_pins),
		.func = 0x5,
	},
	{
		.name = "spi0_4_grp",
		.pins = spi0_4_pins,
		.num_pins = ARRAY_SIZE(spi0_4_pins),
		.func = 0x5,
	},
	{
		.name = "spi1_0_grp",
		.pins = spi1_0_pins,
		.num_pins = ARRAY_SIZE(spi1_0_pins),
		.func = 0x5,
	},
	{
		.name = "spi1_1_grp",
		.pins = spi1_1_pins,
		.num_pins = ARRAY_SIZE(spi1_1_pins),
		.func = 0x2,
	},
	{
		.name = "spi1_2_grp",
		.pins = spi1_2_pins,
		.num_pins = ARRAY_SIZE(spi1_2_pins),
		.func = 0x2,
	},
	{
		.name = "spi1_3_grp",
		.pins = spi1_3_pins,
		.num_pins = ARRAY_SIZE(spi1_3_pins),
		.func = 0x6,
	},
	{
		.name = "spi1_4_grp",
		.pins = spi1_4_pins,
		.num_pins = ARRAY_SIZE(spi1_4_pins),
		.func = 0x6,
	},
	{
		.name = "can0_0_grp",
		.pins = can0_0_pins,
		.num_pins = ARRAY_SIZE(can0_0_pins),
		.func = 0x7,
	},
	{
		.name = "can0_1_grp",
		.pins = can0_1_pins,
		.num_pins = ARRAY_SIZE(can0_1_pins),
		.func = 0x4,
	},
	{
		.name = "can0_2_grp",
		.pins = can0_2_pins,
		.num_pins = ARRAY_SIZE(can0_2_pins),
		.func = 0x4,
	},
	{
		.name = "can0_3_grp",
		.pins = can0_3_pins,
		.num_pins = ARRAY_SIZE(can0_3_pins),
		.func = 0x2,
	},
	{
		.name = "can1_0_grp",
		.pins = can1_0_pins,
		.num_pins = ARRAY_SIZE(can1_0_pins),
		.func = 0x5,
	},
	{
		.name = "can1_1_grp",
		.pins = can1_1_pins,
		.num_pins = ARRAY_SIZE(can1_1_pins),
		.func = 0x4,
	},
	{
		.name = "can1_2_grp",
		.pins = can1_2_pins,
		.num_pins = ARRAY_SIZE(can1_2_pins),
		.func = 0x4,
	},
	{
		.name = "can1_3_grp",
		.pins = can1_3_pins,
		.num_pins = ARRAY_SIZE(can1_3_pins),
		.func = 0x2,
	},
	{
		.name = "can2_0_grp",
		.pins = can2_0_pins,
		.num_pins = ARRAY_SIZE(can2_0_pins),
		.func = 0x5,
	},
	{
		.name = "can2_1_grp",
		.pins = can2_1_pins,
		.num_pins = ARRAY_SIZE(can2_1_pins),
		.func = 0x4,
	},
	{
		.name = "can2_2_grp",
		.pins = can2_2_pins,
		.num_pins = ARRAY_SIZE(can2_2_pins),
		.func = 0x3,
	},
	{
		.name = "can2_3_grp",
		.pins = can2_3_pins,
		.num_pins = ARRAY_SIZE(can2_3_pins),
		.func = 0x4,
	},
	{
		.name = "can2_4_grp",
		.pins = can2_4_pins,
		.num_pins = ARRAY_SIZE(can2_4_pins),
		.func = 0x2,
	},
	{
		.name = "can3_0_grp",
		.pins = can3_0_pins,
		.num_pins = ARRAY_SIZE(can3_0_pins),
		.func = 0x7,
	},
	{
		.name = "can3_1_grp",
		.pins = can3_1_pins,
		.num_pins = ARRAY_SIZE(can3_1_pins),
		.func = 0x2,
	},
	{
		.name = "can3_2_grp",
		.pins = can3_2_pins,
		.num_pins = ARRAY_SIZE(can3_2_pins),
		.func = 0x2,
	},
	{
		.name = "pwm00_0_grp",
		.pins = pwm00_0_pin,
		.num_pins = ARRAY_SIZE(pwm00_0_pin),
		.func = 0x7,
	},
	{
		.name = "pwm01_0_grp",
		.pins = pwm01_0_pin,
		.num_pins = ARRAY_SIZE(pwm01_0_pin),
		.func = 0x7,
	},
	{
		.name = "pwm02_0_grp",
		.pins = pwm02_0_pin,
		.num_pins = ARRAY_SIZE(pwm02_0_pin),
		.func = 0x7,
	},
	{
		.name = "pwm03_0_grp",
		.pins = pwm03_0_pin,
		.num_pins = ARRAY_SIZE(pwm03_0_pin),
		.func = 0x7,
	},

	{
		.name = "pwm00_1_grp",
		.pins = pwm00_1_pin,
		.num_pins = ARRAY_SIZE(pwm00_1_pin),
		.func = 0x6,
	},
	{
		.name = "pwm01_1_grp",
		.pins = pwm01_1_pin,
		.num_pins = ARRAY_SIZE(pwm01_1_pin),
		.func = 0x6,
	},
	{
		.name = "pwm02_1_grp",
		.pins = pwm02_1_pin,
		.num_pins = ARRAY_SIZE(pwm02_1_pin),
		.func = 0x6,
	},
	{
		.name = "pwm03_1_grp",
		.pins = pwm03_1_pin,
		.num_pins = ARRAY_SIZE(pwm03_1_pin),
		.func = 0x6,
	},

	{
		.name = "pwm00_2_grp",
		.pins = pwm00_2_pin,
		.num_pins = ARRAY_SIZE(pwm00_2_pin),
		.func = 0x6,
	},
	{
		.name = "pwm01_2_grp",
		.pins = pwm01_2_pin,
		.num_pins = ARRAY_SIZE(pwm01_2_pin),
		.func = 0x6,
	},
	{
		.name = "pwm02_2_grp",
		.pins = pwm02_2_pin,
		.num_pins = ARRAY_SIZE(pwm02_2_pin),
		.func = 0x6,
	},
	{
		.name = "pwm03_2_grp",
		.pins = pwm03_2_pin,
		.num_pins = ARRAY_SIZE(pwm03_2_pin),
		.func = 0x6,
	},

	{
		.name = "pwm00_3_grp",
		.pins = pwm00_3_pin,
		.num_pins = ARRAY_SIZE(pwm00_3_pin),
		.func = 0x4,
	},
	{
		.name = "pwm01_3_grp",
		.pins = pwm01_3_pin,
		.num_pins = ARRAY_SIZE(pwm01_3_pin),
		.func = 0x4,
	},
	{
		.name = "pwm02_3_grp",
		.pins = pwm02_3_pin,
		.num_pins = ARRAY_SIZE(pwm02_3_pin),
		.func = 0x4,
	},
	{
		.name = "pwm03_3_grp",
		.pins = pwm03_3_pin,
		.num_pins = ARRAY_SIZE(pwm03_3_pin),
		.func = 0x4,
	},
	{
		.name = "pwm02_4_grp",
		.pins = pwm02_4_pin,
		.num_pins = ARRAY_SIZE(pwm02_4_pin),
		.func = 0x4,
	},
	{
		.name = "pwm10_0_grp",
		.pins = pwm10_0_pin,
		.num_pins = ARRAY_SIZE(pwm10_0_pin),
		.func = 0x6,
	},
	{
		.name = "pwm11_0_grp",
		.pins = pwm11_0_pin,
		.num_pins = ARRAY_SIZE(pwm11_0_pin),
		.func = 0x6,
	},
	{
		.name = "pwm12_0_grp",
		.pins = pwm12_0_pin,
		.num_pins = ARRAY_SIZE(pwm12_0_pin),
		.func = 0x6,
	},
	{
		.name = "pwm13_0_grp",
		.pins = pwm13_0_pin,
		.num_pins = ARRAY_SIZE(pwm13_0_pin),
		.func = 0x6,
	},

	{
		.name = "pwm10_1_grp",
		.pins = pwm10_1_pin,
		.num_pins = ARRAY_SIZE(pwm10_1_pin),
		.func = 0x2,
	},
	{
		.name = "pwm11_1_grp",
		.pins = pwm11_1_pin,
		.num_pins = ARRAY_SIZE(pwm11_1_pin),
		.func = 0x2,
	},
	{
		.name = "pwm12_1_grp",
		.pins = pwm12_1_pin,
		.num_pins = ARRAY_SIZE(pwm12_1_pin),
		.func = 0x2,
	},
	{
		.name = "pwm13_1_grp",
		.pins = pwm13_1_pin,
		.num_pins = ARRAY_SIZE(pwm13_1_pin),
		.func = 0x2,
	},

	{
		.name = "pwm10_2_grp",
		.pins = pwm10_2_pin,
		.num_pins = ARRAY_SIZE(pwm10_2_pin),
		.func = 0x6,
	},
	{
		.name = "pwm11_2_grp",
		.pins = pwm11_2_pin,
		.num_pins = ARRAY_SIZE(pwm11_2_pin),
		.func = 0x6,
	},
	{
		.name = "pwm12_2_grp",
		.pins = pwm12_2_pin,
		.num_pins = ARRAY_SIZE(pwm12_2_pin),
		.func = 0x6,
	},
	{
		.name = "pwm13_2_grp",
		.pins = pwm13_2_pin,
		.num_pins = ARRAY_SIZE(pwm13_2_pin),
		.func = 0x6,
	},

	{
		.name = "pwm10_3_grp",
		.pins = pwm10_3_pin,
		.num_pins = ARRAY_SIZE(pwm10_3_pin),
		.func = 0x4,
	},
	{
		.name = "pwm11_3_grp",
		.pins = pwm11_3_pin,
		.num_pins = ARRAY_SIZE(pwm11_3_pin),
		.func = 0x4,
	},
	{
		.name = "pwm12_3_grp",
		.pins = pwm12_3_pin,
		.num_pins = ARRAY_SIZE(pwm12_3_pin),
		.func = 0x4,
	},
	{
		.name = "pwm13_3_grp",
		.pins = pwm13_3_pin,
		.num_pins = ARRAY_SIZE(pwm13_3_pin),
		.func = 0x4,
	},
	{
		.name = "etimer0_0_grp",
		.pins = etimer0_0_pin,
		.num_pins = ARRAY_SIZE(etimer0_0_pin),
		.func = 0x6,
	},
	{
		.name = "etimer0_1_grp",
		.pins = etimer0_1_pin,
		.num_pins = ARRAY_SIZE(etimer0_1_pin),
		.func = 0x5,
	},
	{
		.name = "etimer0_2_grp",
		.pins = etimer0_2_pin,
		.num_pins = ARRAY_SIZE(etimer0_2_pin),
		.func = 0x5,
	},
	{
		.name = "etimer0_3_grp",
		.pins = etimer0_3_pin,
		.num_pins = ARRAY_SIZE(etimer0_3_pin),
		.func = 0x7,
	},
	{
		.name = "etimer0_4_grp",
		.pins = etimer0_4_pin,
		.num_pins = ARRAY_SIZE(etimer0_4_pin),
		.func = 0x7,
	},
	{
		.name = "etimer0_5_grp",
		.pins = etimer0_5_pin,
		.num_pins = ARRAY_SIZE(etimer0_5_pin),
		.func = 0x3,
	},
	{
		.name = "etimer0_6_grp",
		.pins = etimer0_6_pin,
		.num_pins = ARRAY_SIZE(etimer0_6_pin),
		.func = 0x3,
	},
	{
		.name = "etimer0_7_grp",
		.pins = etimer0_7_pin,
		.num_pins = ARRAY_SIZE(etimer0_7_pin),
		.func = 0x3,
	},
	{
		.name = "etimer0_8_grp",
		.pins = etimer0_8_pin,
		.num_pins = ARRAY_SIZE(etimer0_8_pin),
		.func = 0x3,
	},
	{
		.name = "etimer1_0_grp",
		.pins = etimer1_0_pin,
		.num_pins = ARRAY_SIZE(etimer1_0_pin),
		.func = 0x6,
	},
	{
		.name = "etimer1_1_grp",
		.pins = etimer1_1_pin,
		.num_pins = ARRAY_SIZE(etimer1_1_pin),
		.func = 0x3,
	},
	{
		.name = "etimer1_2_grp",
		.pins = etimer1_2_pin,
		.num_pins = ARRAY_SIZE(etimer1_2_pin),
		.func = 0x3,
	},
	{
		.name = "etimer1_3_grp",
		.pins = etimer1_3_pin,
		.num_pins = ARRAY_SIZE(etimer1_3_pin),
		.func = 0x3,
	},
	{
		.name = "etimer1_4_grp",
		.pins = etimer1_4_pin,
		.num_pins = ARRAY_SIZE(etimer1_4_pin),
		.func = 0x3,
	},
	{
		.name = "etimer1_5_grp",
		.pins = etimer1_5_pin,
		.num_pins = ARRAY_SIZE(etimer1_5_pin),
		.func = 0x3,
	},
	{
		.name = "etimer1_6_grp",
		.pins = etimer1_6_pin,
		.num_pins = ARRAY_SIZE(etimer1_6_pin),
		.func = 0x3,
	},
	{
		.name = "etimer1_7_grp",
		.pins = etimer1_7_pin,
		.num_pins = ARRAY_SIZE(etimer1_7_pin),
		.func = 0x3,
	},
	{
		.name = "etimer1_8_grp",
		.pins = etimer1_8_pin,
		.num_pins = ARRAY_SIZE(etimer1_8_pin),
		.func = 0x3,
	},
	{
		.name = "etimer1_9_grp",
		.pins = etimer1_9_pin,
		.num_pins = ARRAY_SIZE(etimer1_9_pin),
		.func = 0x3,
	},
	{
		.name = "etimer1_A_grp",
		.pins = etimer1_A_pin,
		.num_pins = ARRAY_SIZE(etimer1_A_pin),
		.func = 0x3,
	},
	{
		.name = "etimer2_0_grp",
		.pins = etimer2_0_pin,
		.num_pins = ARRAY_SIZE(etimer2_0_pin),
		.func = 0x6,
	},
	{
		.name = "etimer2_1_grp",
		.pins = etimer2_1_pin,
		.num_pins = ARRAY_SIZE(etimer2_1_pin),
		.func = 0x3,
	},
	{
		.name = "etimer2_2_grp",
		.pins = etimer2_2_pin,
		.num_pins = ARRAY_SIZE(etimer2_2_pin),
		.func = 0x3,
	},
	{
		.name = "etimer2_3_grp",
		.pins = etimer2_3_pin,
		.num_pins = ARRAY_SIZE(etimer2_3_pin),
		.func = 0x3,
	},
	{
		.name = "etimer2_4_grp",
		.pins = etimer2_4_pin,
		.num_pins = ARRAY_SIZE(etimer2_4_pin),
		.func = 0x3,
	},
	{
		.name = "etimer2_5_grp",
		.pins = etimer2_5_pin,
		.num_pins = ARRAY_SIZE(etimer2_5_pin),
		.func = 0x3,
	},
	{
		.name = "etimer2_6_grp",
		.pins = etimer2_6_pin,
		.num_pins = ARRAY_SIZE(etimer2_6_pin),
		.func = 0x2,
	},
	{
		.name = "etimer2_7_grp",
		.pins = etimer2_7_pin,
		.num_pins = ARRAY_SIZE(etimer2_7_pin),
		.func = 0x2,
	},
	{
		.name = "etimer2_8_grp",
		.pins = etimer2_8_pin,
		.num_pins = ARRAY_SIZE(etimer2_8_pin),
		.func = 0x3,
	},
	{
		.name = "etimer3_0_grp",
		.pins = etimer3_0_pin,
		.num_pins = ARRAY_SIZE(etimer3_0_pin),
		.func = 0x6,
	},
	{
		.name = "etimer3_1_grp",
		.pins = etimer3_1_pin,
		.num_pins = ARRAY_SIZE(etimer3_1_pin),
		.func = 0x3,
	},
	{
		.name = "etimer3_2_grp",
		.pins = etimer3_2_pin,
		.num_pins = ARRAY_SIZE(etimer3_2_pin),
		.func = 0x3,
	},
	{
		.name = "etimer3_3_grp",
		.pins = etimer3_3_pin,
		.num_pins = ARRAY_SIZE(etimer3_3_pin),
		.func = 0x3,
	},
	{
		.name = "etimer3_4_grp",
		.pins = etimer3_4_pin,
		.num_pins = ARRAY_SIZE(etimer3_4_pin),
		.func = 0x2,
	},
	{
		.name = "etimer3_5_grp",
		.pins = etimer3_5_pin,
		.num_pins = ARRAY_SIZE(etimer3_5_pin),
		.func = 0x2,
	},
	{
		.name = "etimer3_6_grp",
		.pins = etimer3_6_pin,
		.num_pins = ARRAY_SIZE(etimer3_6_pin),
		.func = 0x3,
	},
	{
		.name = "etimer4_0_grp",
		.pins = etimer4_0_pin,
		.num_pins = ARRAY_SIZE(etimer4_0_pin),
		.func = 0x6,
	},
	{
		.name = "etimer4_1_grp",
		.pins = etimer4_1_pin,
		.num_pins = ARRAY_SIZE(etimer4_1_pin),
		.func = 0x3,
	},
	{
		.name = "etimer4_2_grp",
		.pins = etimer4_2_pin,
		.num_pins = ARRAY_SIZE(etimer4_2_pin),
		.func = 0x3,
	},
	{
		.name = "etimer4_3_grp",
		.pins = etimer4_3_pin,
		.num_pins = ARRAY_SIZE(etimer4_3_pin),
		.func = 0x3,
	},
	{
		.name = "etimer4_4_grp",
		.pins = etimer4_4_pin,
		.num_pins = ARRAY_SIZE(etimer4_4_pin),
		.func = 0x3,
	},
	{
		.name = "etimer4_5_grp",
		.pins = etimer4_5_pin,
		.num_pins = ARRAY_SIZE(etimer4_5_pin),
		.func = 0x3,
	},
	{
		.name = "etimer4_6_grp",
		.pins = etimer4_6_pin,
		.num_pins = ARRAY_SIZE(etimer4_6_pin),
		.func = 0x3,
	},
	{
		.name = "etimer4_7_grp",
		.pins = etimer4_7_pin,
		.num_pins = ARRAY_SIZE(etimer4_7_pin),
		.func = 0x3,
	},
	{
		.name = "etimer4_8_grp",
		.pins = etimer4_8_pin,
		.num_pins = ARRAY_SIZE(etimer4_8_pin),
		.func = 0x3,
	},
	{
		.name = "etimer5_0_grp",
		.pins = etimer5_0_pin,
		.num_pins = ARRAY_SIZE(etimer5_0_pin),
		.func = 0x6,
	},
	{
		.name = "etimer5_1_grp",
		.pins = etimer5_1_pin,
		.num_pins = ARRAY_SIZE(etimer5_1_pin),
		.func = 0x3,
	},
	{
		.name = "etimer5_2_grp",
		.pins = etimer5_2_pin,
		.num_pins = ARRAY_SIZE(etimer5_2_pin),
		.func = 0x3,
	},
	{
		.name = "etimer5_3_grp",
		.pins = etimer5_3_pin,
		.num_pins = ARRAY_SIZE(etimer5_3_pin),
		.func = 0x3,
	},
	{
		.name = "etimer5_4_grp",
		.pins = etimer5_4_pin,
		.num_pins = ARRAY_SIZE(etimer5_4_pin),
		.func = 0x3,
	},
	{
		.name = "etimer5_5_grp",
		.pins = etimer5_5_pin,
		.num_pins = ARRAY_SIZE(etimer5_5_pin),
		.func = 0x3,
	},
	{
		.name = "etimer5_6_grp",
		.pins = etimer5_6_pin,
		.num_pins = ARRAY_SIZE(etimer5_6_pin),
		.func = 0x3,
	},
	{
		.name = "etimer5_7_grp",
		.pins = etimer5_7_pin,
		.num_pins = ARRAY_SIZE(etimer5_7_pin),
		.func = 0x3,
	},
	{
		.name = "etimer5_8_grp",
		.pins = etimer5_8_pin,
		.num_pins = ARRAY_SIZE(etimer5_8_pin),
		.func = 0x3,
	},
	{
		.name = "jtag0_grp",
		.pins = jtag0_pins,
		.num_pins = ARRAY_SIZE(jtag0_pins),
		.func = 0x7,
	},
	{
		.name = "jtag1_grp",
		.pins = jtag1_pins,
		.num_pins = ARRAY_SIZE(jtag1_pins),
		.func = 0x4,
	},
	{
		.name = "eint0_0_grp",
		.pins = eint0_0_pin,
		.num_pins = ARRAY_SIZE(eint0_0_pin),
		.func = 0x5,
	},
	{
		.name = "eint0_1_grp",
		.pins = eint0_1_pin,
		.num_pins = ARRAY_SIZE(eint0_1_pin),
		.func = 0x8,
	},
	{
		.name = "eint1_0_grp",
		.pins = eint1_0_pin,
		.num_pins = ARRAY_SIZE(eint1_0_pin),
		.func = 0x5,
	},
	{
		.name = "eint1_1_grp",
		.pins = eint1_1_pin,
		.num_pins = ARRAY_SIZE(eint1_1_pin),
		.func = 0x8,
	},
	{
		.name = "eint2_0_grp",
		.pins = eint2_0_pin,
		.num_pins = ARRAY_SIZE(eint2_0_pin),
		.func = 0x4,
	},
	{
		.name = "eint2_1_grp",
		.pins = eint2_1_pin,
		.num_pins = ARRAY_SIZE(eint2_1_pin),
		.func = 0x5,
	},
	{
		.name = "eint2_2_grp",
		.pins = eint2_2_pin,
		.num_pins = ARRAY_SIZE(eint2_2_pin),
		.func = 0x3,
	},
	{
		.name = "eint2_3_grp",
		.pins = eint2_3_pin,
		.num_pins = ARRAY_SIZE(eint2_3_pin),
		.func = 0x2,
	},
	{
		.name = "eint3_0_grp",
		.pins = eint3_0_pin,
		.num_pins = ARRAY_SIZE(eint3_0_pin),
		.func = 0x4,
	},
	{
		.name = "eint3_1_grp",
		.pins = eint3_1_pin,
		.num_pins = ARRAY_SIZE(eint3_1_pin),
		.func = 0x5,
	},
	{
		.name = "eint3_2_grp",
		.pins = eint3_2_pin,
		.num_pins = ARRAY_SIZE(eint3_2_pin),
		.func = 0x4,
	},
	{
		.name = "ebi8_0_grp",
		.pins = ebi8_0_pin,
		.num_pins = ARRAY_SIZE(ebi8_0_pin),
		.func = 0x1,
	},
	{
		.name = "ebi8_1_grp",
		.pins = ebi8_1_pin,
		.num_pins = ARRAY_SIZE(ebi8_1_pin),
		.func = 0x1,
	},
	{
		.name = "ebi8_2_grp",
		.pins = ebi8_2_pin,
		.num_pins = ARRAY_SIZE(ebi8_2_pin),
		.func = 0x1,
	},
	{
		.name = "ebi8_3_grp",
		.pins = ebi8_3_pin,
		.num_pins = ARRAY_SIZE(ebi8_3_pin),
		.func = 0x1,
	},
	{
		.name = "ebi8_4_grp",
		.pins = ebi8_4_pin,
		.num_pins = ARRAY_SIZE(ebi8_4_pin),
		.func = 0x1,
	},
	{
		.name = "ebi8_5_grp",
		.pins = ebi8_5_pin,
		.num_pins = ARRAY_SIZE(ebi8_5_pin),
		.func = 0x1,
	},
	{
		.name = "ebi16_0_grp",
		.pins = ebi16_0_pin,
		.num_pins = ARRAY_SIZE(ebi16_0_pin),
		.func = 0x1,
	},
	{
		.name = "ebi16_1_grp",
		.pins = ebi16_1_pin,
		.num_pins = ARRAY_SIZE(ebi16_1_pin),
		.func = 0x1,
	},
	{
		.name = "ebi16_2_grp",
		.pins = ebi16_2_pin,
		.num_pins = ARRAY_SIZE(ebi16_2_pin),
		.func = 0x1,
	},
	{
		.name = "ebi16_3_grp",
		.pins = ebi16_3_pin,
		.num_pins = ARRAY_SIZE(ebi16_3_pin),
		.func = 0x1,
	},
	{
		.name = "ebi16_4_grp",
		.pins = ebi16_4_pin,
		.num_pins = ARRAY_SIZE(ebi16_4_pin),
		.func = 0x1,
	},
	{
		.name = "ebi16_5_grp",
		.pins = ebi16_5_pin,
		.num_pins = ARRAY_SIZE(ebi16_5_pin),
		.func = 0x1,
	},
	{
		.name = "usbh_power_1_grp",
		.pins = usbh_pwren_ovc_pin,
		.num_pins = ARRAY_SIZE(usbh_pwren_ovc_pin),
		.func = 0x1,
	},
	{
		.name = "usbh_power_2_grp",
		.pins = usbh_pwren_pin,
		.num_pins = ARRAY_SIZE(usbh_pwren_pin),
		.func = 0x1,
	},
	{
		.name = "usbh_power_3_grp",
		.pins = usbh_ovc_pin,
		.num_pins = ARRAY_SIZE(usbh_ovc_pin),
		.func = 0x1,
	},
	{
		.name = "usbh_lite0_1_grp",
		.pins = usbh_lite0_pb4_pb6,
		.num_pins = ARRAY_SIZE(usbh_lite0_pb4_pb6),
		.func = 0x4,
	},
	{
		.name = "usbh_lite0_2_grp",
		.pins = usbh_lite0_pb5_pb7,
		.num_pins = ARRAY_SIZE(usbh_lite0_pb5_pb7),
		.func = 0x4,
	},
	{
		.name = "usbh_lite0_3_grp",
		.pins = usbh_lite0_pb10_pb9,
		.num_pins = ARRAY_SIZE(usbh_lite0_pb10_pb9),
		.func = 0x4,
	},
	{
		.name = "usbh_lite0_4_grp",
		.pins = usbh_lite0_pd15_pd14,
		.num_pins = ARRAY_SIZE(usbh_lite0_pd15_pd14),
		.func = 0x5,
	},
	{
		.name = "usbh_lite1_1_grp",
		.pins = usbh_lite1_pe1_pe0,
		.num_pins = ARRAY_SIZE(usbh_lite1_pe1_pe0),
		.func = 0x6,
	},
	{
		.name = "usbh_lite1_2_grp",
		.pins = usbh_lite1_pf1_pf0,
		.num_pins = ARRAY_SIZE(usbh_lite1_pf1_pf0),
		.func = 0x6,
	},
	{
		.name = "usbh_lite2_1_grp",
		.pins = usbh_lite2_pe3_pe2,
		.num_pins = ARRAY_SIZE(usbh_lite2_pe3_pe2),
		.func = 0x6,
	},
	{
		.name = "usbh_lite2_2_grp",
		.pins = usbh_lite2_pf3_pf2,
		.num_pins = ARRAY_SIZE(usbh_lite2_pf3_pf2),
		.func = 0x6,
	},
	{
		.name = "usbh_lite3_1_grp",
		.pins = usbh_lite3_pe5_pe4,
		.num_pins = ARRAY_SIZE(usbh_lite3_pe5_pe4),
		.func = 0x6,
	},
	{
		.name = "usbh_lite3_2_grp",
		.pins = usbh_lite3_pf5_pf4,
		.num_pins = ARRAY_SIZE(usbh_lite3_pf5_pf4),
		.func = 0x6,
	},
	{
		.name = "usbh_lite4_1_grp",
		.pins = usbh_lite4_pe7_pe6,
		.num_pins = ARRAY_SIZE(usbh_lite4_pe7_pe6),
		.func = 0x6,
	},
	{
		.name = "usbh_lite4_2_grp",
		.pins = usbh_lite4_pf7_pf6,
		.num_pins = ARRAY_SIZE(usbh_lite4_pf7_pf6),
		.func = 0x6,
	},
	{
		.name = "usbh_lite4_3_grp",
		.pins = usbh_lite4_pg10_pa15,
		.num_pins = ARRAY_SIZE(usbh_lite4_pg10_pa15),
		.func = 0x4,
	},
	{
		.name = "usbh_lite4_4_grp",
		.pins = usbh_lite4_pb13_pf6,
		.num_pins = ARRAY_SIZE(usbh_lite4_pb13_pf6),
		.func = 0x6,
	},
	{
		.name = "usbh_lite5_1_grp",
		.pins = usbh_lite5_pe9_pe8,
		.num_pins = ARRAY_SIZE(usbh_lite5_pe9_pe8),
		.func = 0x6,
	},
	{
		.name = "usbh_lite5_2_grp",
		.pins = usbh_lite5_pf9_pf8,
		.num_pins = ARRAY_SIZE(usbh_lite5_pf9_pf8),
		.func = 0x6,
	},
	{
		.name = "usbh_lite5_3_grp",
		.pins = usbh_lite5_pa14_pa13,
		.num_pins = ARRAY_SIZE(usbh_lite5_pa14_pa13),
		.func = 0x4,
	},
	{
		.name = "usbh_lite5_4_grp",
		.pins = usbh_lite5_pb12_pb11,
		.num_pins = ARRAY_SIZE(usbh_lite5_pb12_pb11),
		.func = 0x4,
	},
};

static int nuc980_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(nuc980_pinctrl_groups);
}

static const char *nuc980_get_group_name(struct pinctrl_dev *pctldev,
        unsigned selector)
{
	;
	return nuc980_pinctrl_groups[selector].name;
}

static int nuc980_get_group_pins(struct pinctrl_dev *pctldev, unsigned selector,
                                 const unsigned ** pins,
                                 unsigned * num_pins)
{
	*pins = (unsigned *) nuc980_pinctrl_groups[selector].pins;
	*num_pins = nuc980_pinctrl_groups[selector].num_pins;
	return 0;
}

static struct pinctrl_ops nuc980_pctrl_ops = {
	.get_groups_count = nuc980_get_groups_count,
	.get_group_name = nuc980_get_group_name,
	.get_group_pins = nuc980_get_group_pins,
};

struct nuc980_pmx_func {
	const char *name;
	const char * const *groups;
	const unsigned num_groups;
};

static const char * const nadc_groups[] = {"nadc_grp"};
static const char * const emac0_groups[] = {"emac0_grp"};
static const char * const emac1_groups[] = {"emac1_grp"};
static const char * const vcap0_groups[] = {"vcap0_grp"};
static const char * const vcap1_groups[] = {"vcap1_grp"};
static const char * const sd0_groups[] = {"sd0_grp"};
static const char * const sd1_groups[] = {"sd1_grp"};
static const char * const nand_groups[] = {"nand_grp"};
static const char * const usbd_groups[] = {"usbd_grp"};
static const char * const i2c0_groups[] = {"i2c0_0_grp", "i2c0_1_grp", "i2c0_2_grp"};
static const char * const i2c1_groups[] = {"i2c1_0_grp", "i2c1_1_grp", "i2c1_2_grp"};
static const char * const i2c2_groups[] = {"i2c2_0_grp", "i2c2_1_grp"};
static const char * const i2c3_groups[] = {"i2c3_0_grp", "i2c3_1_grp"};
static const char * const i2s_groups[] = {"i2s_0_grp", "i2s_1_grp"};
static const char * const uart0_groups[] = {"uart0_grp"};
static const char * const uart1_groups[] = {"uart1_0_grp", "uart1_1_grp", "uart1_4_grp"};
static const char * const uart1_fc_groups[] = {"uart1_2_grp", "uart1_3_grp"};
static const char * const uart2_groups[] = {"uart2_1_grp", "uart2_4_grp", "uart2_5_grp"};
static const char * const uart2_fc_groups[] = {"uart2_0_grp", "uart2_2_grp", "uart2_3_grp"};
static const char * const uart3_groups[] = {"uart3_0_grp", "uart3_1_grp", "uart3_3_grp", "uart3_6_grp", "uart3_7_grp"};
static const char * const uart3_fc_groups[] = {"uart3_2_grp", "uart3_4_grp", "uart3_5_grp"};
static const char * const uart4_groups[] = {"uart4_0_grp", "uart4_1_grp", "uart4_4_grp"};
static const char * const uart4_fc_groups[] = {"uart4_2_grp", "uart4_3_grp"};
static const char * const uart5_groups[] = {"uart5_0_grp", "uart5_2_grp", "uart5_4_grp"};
static const char * const uart5_fc_groups[] = {"uart5_1_grp", "uart5_3_grp"};
static const char * const uart6_groups[] = {"uart6_1_grp", "uart6_3_grp", "uart6_4_grp"};
static const char * const uart6_fc_groups[] = {"uart6_0_grp", "uart6_2_grp"};
static const char * const uart7_groups[] = {"uart7_0_grp", "uart7_1_grp", "uart7_3_grp", "uart7_5_grp"};
static const char * const uart7_fc_groups[] = {"uart7_2_grp", "uart7_4_grp"};
static const char * const uart8_groups[] = {"uart8_0_grp", "uart8_2_grp", "uart8_3_grp"};
static const char * const uart8_fc_groups[] = {"uart8_1_grp", "uart8_4_grp"};
static const char * const uart9_groups[] = {"uart9_0_grp", "uart9_3_grp", "uart9_4_grp"};
static const char * const uart9_fc_groups[] = {"uart9_2_grp"};
static const char * const sc0_groups[] = {"sc0_0_grp", "sc0_1_grp"};
static const char * const sc1_groups[] = {"sc1_0_grp", "sc1_1_grp"};
static const char * const scuart0_groups[] = {"sc0_2_grp", "sc0_3_grp"};
static const char * const scuart1_groups[] = {"sc1_2_grp", "sc1_3_grp"};
static const char * const qspi0_groups[] = {"qspi0_3_grp"};
static const char * const qspi0_quad_groups[] = {"qspi0_2_grp"};
static const char * const qspi0_ss1_groups[] = {"qspi0_0_grp", "qspi0_1_grp"};
static const char * const qspi0_quad_ss1_groups[] = {"qspi0_4_grp","qspi0_5_grp"};
static const char * const spi0_groups[] = {"spi0_0_grp","spi0_3_grp"};
static const char * const spi0_ss1_groups[] = {"spi0_1_grp","spi0_2_grp","spi0_4_grp"};
static const char * const spi1_groups[] = {"spi1_0_grp", "spi1_1_grp", "spi1_3_grp"};
static const char * const spi1_ss1_groups[] = {"spi1_2_grp", "spi1_4_grp"};
static const char * const can0_groups[] = {"can0_0_grp", "can0_1_grp", "can0_2_grp", "can0_3_grp"};
static const char * const can1_groups[] = {"can1_0_grp", "can1_1_grp", "can1_2_grp", "can1_3_grp"};
static const char * const can2_groups[] = {"can2_0_grp", "can2_1_grp", "can2_2_grp", "can2_3_grp", "can2_4_grp"};
static const char * const can3_groups[] = {"can3_0_grp", "can3_1_grp", "can3_2_grp"};
static const char * const pwm00_groups[] = {"pwm00_0_grp", "pwm00_1_grp", "pwm00_2_grp", "pwm00_3_grp"};
static const char * const pwm01_groups[] = {"pwm01_0_grp", "pwm01_1_grp", "pwm01_2_grp", "pwm01_3_grp"};
static const char * const pwm02_groups[] = {"pwm02_0_grp", "pwm02_1_grp", "pwm02_2_grp", "pwm02_3_grp", "pwm02_4_grp"};
static const char * const pwm03_groups[] = {"pwm03_0_grp", "pwm03_1_grp", "pwm03_2_grp", "pwm03_3_grp"};
static const char * const pwm10_groups[] = {"pwm10_0_grp", "pwm10_1_grp", "pwm10_2_grp", "pwm10_3_grp"};
static const char * const pwm11_groups[] = {"pwm11_0_grp", "pwm11_1_grp", "pwm11_2_grp", "pwm11_3_grp"};
static const char * const pwm12_groups[] = {"pwm12_0_grp", "pwm12_1_grp", "pwm12_2_grp", "pwm12_3_grp"};
static const char * const pwm13_groups[] = {"pwm13_0_grp", "pwm13_1_grp", "pwm13_2_grp", "pwm13_3_grp"};
static const char * const etimer0_ecnt_groups[] = {"etimer0_0_grp", "etimer0_7_grp", "etimer0_8_grp"};
static const char * const etimer0_tgl_groups[] = {"etimer0_1_grp", "etimer0_3_grp", "etimer0_5_grp"};
static const char * const etimer0_cap_groups[] = {"etimer0_2_grp", "etimer0_4_grp", "etimer0_6_grp"};
static const char * const etimer1_ecnt_groups[] = {"etimer1_0_grp", "etimer1_5_grp","etimer1_8_grp"};
static const char * const etimer1_tgl_groups[] = {"etimer1_1_grp", "etimer1_3_grp","etimer1_6_grp", "etimer1_9_grp"};
static const char * const etimer1_cap_groups[] = {"etimer1_2_grp", "etimer1_4_grp", "etimer1_7_grp", "etimer1_A_grp"};
static const char * const etimer2_ecnt_groups[] = {"etimer2_0_grp", "etimer2_5_grp", "etimer2_8_grp"};
static const char * const etimer2_tgl_groups[] = {"etimer2_1_grp", "etimer2_3_grp", "etimer2_6_grp"};
static const char * const etimer2_cap_groups[] = {"etimer2_2_grp", "etimer2_4_grp", "etimer2_7_grp"};
static const char * const etimer3_ecnt_groups[] = {"etimer3_0_grp", "etimer3_3_grp", "etimer3_6_grp"};
static const char * const etimer3_tgl_groups[] = {"etimer3_1_grp", "etimer3_4_grp"};
static const char * const etimer3_cap_groups[] = {"etimer3_2_grp", "etimer3_5_grp"};
static const char * const etimer4_ecnt_groups[] = {"etimer4_0_grp", "etimer4_5_grp", "etimer4_6_grp"};
static const char * const etimer4_tgl_groups[] = {"etimer4_1_grp", "etimer4_3_grp", "etimer4_7_grp"};
static const char * const etimer4_cap_groups[] = {"etimer4_2_grp", "etimer4_4_grp", "etimer4_8_grp"};
static const char * const etimer5_ecnt_groups[] = {"etimer5_0_grp", "etimer5_5_grp", "etimer5_6_grp"};
static const char * const etimer5_tgl_groups[] = {"etimer5_1_grp", "etimer5_3_grp", "etimer5_7_grp"};
static const char * const etimer5_cap_groups[] = {"etimer5_2_grp", "etimer5_4_grp", "etimer5_8_grp"};
static const char * const jtag0_groups[] = {"jtag0_grp"};
static const char * const jtag1_groups[] = {"jtag1_grp"};
static const char * const eint0_groups[] = {"eint0_0_grp", "eint0_1_grp"};
static const char * const eint1_groups[] = {"eint1_0_grp", "eint1_1_grp"};
static const char * const eint2_groups[] = {"eint2_0_grp", "eint2_1_grp", "eint2_2_grp", "eint2_3_grp"};
static const char * const eint3_groups[] = {"eint3_0_grp", "eint3_1_grp", "eint3_2_grp"};
static const char * const ebi8_0_groups[] = {"ebi8_0_grp"};
static const char * const ebi8_1_groups[] = {"ebi8_1_grp"};
static const char * const ebi8_2_groups[] = {"ebi8_2_grp"};
static const char * const ebi8_3_groups[] = {"ebi8_3_grp"};
static const char * const ebi8_4_groups[] = {"ebi8_4_grp"};
static const char * const ebi8_5_groups[] = {"ebi8_5_grp"};;
static const char * const ebi16_0_groups[] = {"ebi16_0_grp"};
static const char * const ebi16_1_groups[] = {"ebi16_1_grp"};
static const char * const ebi16_2_groups[] = {"ebi16_2_grp"};
static const char * const ebi16_3_groups[] = {"ebi16_3_grp"};
static const char * const ebi16_4_groups[] = {"ebi16_4_grp"};
static const char * const ebi16_5_groups[] = {"ebi16_5_grp"};
static const char * const usbh_power_groups[] = {"usbh_power_1_grp", "usbh_power_2_grp", "usbh_power_3_grp"};
static const char * const usbh_lite0_groups[] = {"usbh_lite0_1_grp", "usbh_lite0_2_grp", "usbh_lite0_3_grp", "usbh_lite0_4_grp"};
static const char * const usbh_lite1_groups[] = {"usbh_lite1_1_grp", "usbh_lite1_2_grp"};
static const char * const usbh_lite2_groups[] = {"usbh_lite2_1_grp", "usbh_lite2_2_grp"};
static const char * const usbh_lite3_groups[] = {"usbh_lite3_1_grp", "usbh_lite3_2_grp"};
static const char * const usbh_lite4_groups[] = {"usbh_lite4_1_grp", "usbh_lite4_2_grp", "usbh_lite4_3_grp", "usbh_lite4_4_grp"};
static const char * const usbh_lite5_groups[] = {"usbh_lite5_1_grp", "usbh_lite5_2_grp", "usbh_lite5_3_grp", "usbh_lite5_4_grp"};

static const struct nuc980_pmx_func nuc980_functions[] = {
	{
		.name = "nadc",
		.groups = nadc_groups,
		.num_groups = ARRAY_SIZE(nadc_groups),
	},
	{
		.name = "emac0",
		.groups = emac0_groups,
		.num_groups = ARRAY_SIZE(emac0_groups),
	},
	{
		.name = "emac1",
		.groups = emac1_groups,
		.num_groups = ARRAY_SIZE(emac1_groups),
	},
	{
		.name = "vcap0",
		.groups = vcap0_groups,
		.num_groups = ARRAY_SIZE(vcap0_groups),
	},
	{
		.name = "vcap1",
		.groups = vcap1_groups,
		.num_groups = ARRAY_SIZE(vcap1_groups),
	},
	{
		.name = "sd0",
		.groups = sd0_groups,
		.num_groups = ARRAY_SIZE(sd0_groups),
	},
	{
		.name = "sd1",
		.groups = sd1_groups,
		.num_groups = ARRAY_SIZE(sd1_groups),
	},
	{
		.name = "nand",
		.groups = nand_groups,
		.num_groups = ARRAY_SIZE(nand_groups),
	},
	{
		.name = "usbd",
		.groups = usbd_groups,
		.num_groups = ARRAY_SIZE(usbd_groups),
	},
	{
		.name = "i2c0",
		.groups = i2c0_groups,
		.num_groups = ARRAY_SIZE(i2c0_groups),
	},
	{
		.name = "i2c1",
		.groups = i2c1_groups,
		.num_groups = ARRAY_SIZE(i2c1_groups),
	},
	{
		.name = "i2c2",
		.groups = i2c2_groups,
		.num_groups = ARRAY_SIZE(i2c2_groups),
	},
	{
		.name = "i2c3",
		.groups = i2c3_groups,
		.num_groups = ARRAY_SIZE(i2c3_groups),
	},
	{
		.name = "i2s",
		.groups = i2s_groups,
		.num_groups = ARRAY_SIZE(i2s_groups),
	},
	{
		.name = "uart0",
		.groups = uart0_groups,
		.num_groups = ARRAY_SIZE(uart0_groups),
	},
	{
		.name = "uart1",
		.groups = uart1_groups,
		.num_groups = ARRAY_SIZE(uart1_groups),
	},
	{
		.name = "uart1_fc",
		.groups = uart1_fc_groups,
		.num_groups = ARRAY_SIZE(uart1_fc_groups),
	},
	{
		.name = "uart2",
		.groups = uart2_groups,
		.num_groups = ARRAY_SIZE(uart2_groups),
	},
	{
		.name = "uart2_fc",
		.groups = uart2_fc_groups,
		.num_groups = ARRAY_SIZE(uart2_fc_groups),
	},
	{
		.name = "uart3",
		.groups = uart3_groups,
		.num_groups = ARRAY_SIZE(uart3_groups),
	},
	{
		.name = "uart3_fc",
		.groups = uart3_fc_groups,
		.num_groups = ARRAY_SIZE(uart3_fc_groups),
	},
	{
		.name = "uart4",
		.groups = uart4_groups,
		.num_groups = ARRAY_SIZE(uart4_groups),
	},
	{
		.name = "uart4_fc",
		.groups = uart4_fc_groups,
		.num_groups = ARRAY_SIZE(uart4_fc_groups),
	},
	{
		.name = "uart5",
		.groups = uart5_groups,
		.num_groups = ARRAY_SIZE(uart5_groups),
	},
	{
		.name = "uart5_fc",
		.groups = uart5_fc_groups,
		.num_groups = ARRAY_SIZE(uart5_fc_groups),
	},
	{
		.name = "uart6",
		.groups = uart6_groups,
		.num_groups = ARRAY_SIZE(uart6_groups),
	},
	{
		.name = "uart6_fc",
		.groups = uart6_fc_groups,
		.num_groups = ARRAY_SIZE(uart6_fc_groups),
	},
	{
		.name = "uart7",
		.groups = uart7_groups,
		.num_groups = ARRAY_SIZE(uart7_groups),
	},
	{
		.name = "uart7_fc",
		.groups = uart7_fc_groups,
		.num_groups = ARRAY_SIZE(uart7_fc_groups),
	},
	{
		.name = "uart8",
		.groups = uart8_groups,
		.num_groups = ARRAY_SIZE(uart8_groups),
	},
	{
		.name = "uart8_fc",
		.groups = uart8_fc_groups,
		.num_groups = ARRAY_SIZE(uart8_fc_groups),
	},
	{
		.name = "uart9",
		.groups = uart9_groups,
		.num_groups = ARRAY_SIZE(uart9_groups),
	},
	{
		.name = "uart9_fc",
		.groups = uart9_fc_groups,
		.num_groups = ARRAY_SIZE(uart9_fc_groups),
	},
	{
		.name = "sc0",
		.groups = sc0_groups,
		.num_groups = ARRAY_SIZE(sc0_groups),
	},
	{
		.name = "sc1",
		.groups = sc1_groups,
		.num_groups = ARRAY_SIZE(sc1_groups),
	},
	{
		.name = "scuart0",
		.groups = scuart0_groups,
		.num_groups = ARRAY_SIZE(scuart0_groups),
	},
	{
		.name = "scuart1",
		.groups = scuart1_groups,
		.num_groups = ARRAY_SIZE(scuart1_groups),
	},
	{
		.name = "qspi0",
		.groups = qspi0_groups,
		.num_groups = ARRAY_SIZE(qspi0_groups),
	},
	{
		.name = "qspi0_quad",
		.groups = qspi0_quad_groups,
		.num_groups = ARRAY_SIZE(qspi0_quad_groups),
	},
	{
		.name = "qspi0_ss1",
		.groups = qspi0_ss1_groups,
		.num_groups = ARRAY_SIZE(qspi0_ss1_groups),
	},
	{
		.name = "qspi0_quad_ss1",
		.groups = qspi0_quad_ss1_groups,
		.num_groups = ARRAY_SIZE(qspi0_quad_ss1_groups),
	},
	{
		.name = "spi0",
		.groups = spi0_groups,
		.num_groups = ARRAY_SIZE(spi0_groups),
	},
	{
		.name = "spi0_ss1",
		.groups = spi0_ss1_groups,
		.num_groups = ARRAY_SIZE(spi0_ss1_groups),
	},
	{
		.name = "spi1",
		.groups = spi1_groups,
		.num_groups = ARRAY_SIZE(spi1_groups),
	},
	{
		.name = "spi1_ss1",
		.groups = spi1_ss1_groups,
		.num_groups = ARRAY_SIZE(spi1_ss1_groups),
	},
	{
		.name = "can0",
		.groups = can0_groups,
		.num_groups = ARRAY_SIZE(can0_groups),
	},
	{
		.name = "can1",
		.groups = can1_groups,
		.num_groups = ARRAY_SIZE(can1_groups),
	},
	{
		.name = "can2",
		.groups = can2_groups,
		.num_groups = ARRAY_SIZE(can2_groups),
	},
	{
		.name = "can3",
		.groups = can3_groups,
		.num_groups = ARRAY_SIZE(can3_groups),
	},
	{
		.name = "pwm00",
		.groups = pwm00_groups,
		.num_groups = ARRAY_SIZE(pwm00_groups),
	},
	{
		.name = "pwm01",
		.groups = pwm01_groups,
		.num_groups = ARRAY_SIZE(pwm01_groups),
	},
	{
		.name = "pwm02",
		.groups = pwm02_groups,
		.num_groups = ARRAY_SIZE(pwm02_groups),
	},
	{
		.name = "pwm03",
		.groups = pwm03_groups,
		.num_groups = ARRAY_SIZE(pwm03_groups),
	},
	{
		.name = "pwm10",
		.groups = pwm10_groups,
		.num_groups = ARRAY_SIZE(pwm10_groups),
	},
	{
		.name = "pwm11",
		.groups = pwm11_groups,
		.num_groups = ARRAY_SIZE(pwm11_groups),
	},
	{
		.name = "pwm12",
		.groups = pwm12_groups,
		.num_groups = ARRAY_SIZE(pwm12_groups),
	},
	{
		.name = "pwm13",
		.groups = pwm13_groups,
		.num_groups = ARRAY_SIZE(pwm13_groups),
	},
	{
		.name = "etimer0_ecnt",
		.groups = etimer0_ecnt_groups,
		.num_groups = ARRAY_SIZE(etimer0_ecnt_groups),
	},
	{
		.name = "etimer0_tgl",
		.groups = etimer0_tgl_groups,
		.num_groups = ARRAY_SIZE(etimer0_tgl_groups),
	},
	{
		.name = "etimer0_cap",
		.groups = etimer0_cap_groups,
		.num_groups = ARRAY_SIZE(etimer0_cap_groups),
	},
	{
		.name = "etimer1_ecnt",
		.groups = etimer1_ecnt_groups,
		.num_groups = ARRAY_SIZE(etimer1_ecnt_groups),
	},
	{
		.name = "etimer1_tgl",
		.groups = etimer1_tgl_groups,
		.num_groups = ARRAY_SIZE(etimer1_tgl_groups),
	},
	{
		.name = "etimer1_cap",
		.groups = etimer1_cap_groups,
		.num_groups = ARRAY_SIZE(etimer1_cap_groups),
	},
	{
		.name = "etimer2_ecnt",
		.groups = etimer2_ecnt_groups,
		.num_groups = ARRAY_SIZE(etimer2_ecnt_groups),
	},
	{
		.name = "etimer2_tgl",
		.groups = etimer2_tgl_groups,
		.num_groups = ARRAY_SIZE(etimer2_tgl_groups),
	},
	{
		.name = "etimer2_cap",
		.groups = etimer2_cap_groups,
		.num_groups = ARRAY_SIZE(etimer2_cap_groups),
	},
	{
		.name = "etimer3_ecnt",
		.groups = etimer3_ecnt_groups,
		.num_groups = ARRAY_SIZE(etimer3_ecnt_groups),
	},
	{
		.name = "etimer3_tgl",
		.groups = etimer3_tgl_groups,
		.num_groups = ARRAY_SIZE(etimer3_tgl_groups),
	},
	{
		.name = "etimer3_cap",
		.groups = etimer3_cap_groups,
		.num_groups = ARRAY_SIZE(etimer3_cap_groups),
	},
	{
		.name = "etimer4_ecnt",
		.groups = etimer4_ecnt_groups,
		.num_groups = ARRAY_SIZE(etimer4_ecnt_groups),
	},
	{
		.name = "etimer4_tgl",
		.groups = etimer4_tgl_groups,
		.num_groups = ARRAY_SIZE(etimer4_tgl_groups),
	},
	{
		.name = "etimer4_cap",
		.groups = etimer4_cap_groups,
		.num_groups = ARRAY_SIZE(etimer4_cap_groups),
	},
	{
		.name = "etimer5_ecnt",
		.groups = etimer5_ecnt_groups,
		.num_groups = ARRAY_SIZE(etimer5_ecnt_groups),
	},
	{
		.name = "etimer5_tgl",
		.groups = etimer5_tgl_groups,
		.num_groups = ARRAY_SIZE(etimer5_tgl_groups),
	},
	{
		.name = "etimer5_cap",
		.groups = etimer5_cap_groups,
		.num_groups = ARRAY_SIZE(etimer5_cap_groups),
	},
	{
		.name = "jtag0",
		.groups = jtag0_groups,
		.num_groups = ARRAY_SIZE(jtag0_groups),
	},
	{
		.name = "jtag1",
		.groups = jtag1_groups,
		.num_groups = ARRAY_SIZE(jtag1_groups),
	},
	{
		.name = "eint0",
		.groups = eint0_groups,
		.num_groups = ARRAY_SIZE(eint0_groups),
	},
	{
		.name = "eint1",
		.groups = eint1_groups,
		.num_groups = ARRAY_SIZE(eint1_groups),
	},
	{
		.name = "eint2",
		.groups = eint2_groups,
		.num_groups = ARRAY_SIZE(eint2_groups),
	},
	{
		.name = "eint3",
		.groups = eint3_groups,
		.num_groups = ARRAY_SIZE(eint3_groups),
	},
	{
		.name = "ebi_8_0",
		.groups = ebi8_0_groups,
		.num_groups = ARRAY_SIZE(ebi8_0_groups),
	},
	{
		.name = "ebi_8_1",
		.groups = ebi8_1_groups,
		.num_groups = ARRAY_SIZE(ebi8_1_groups),
	},
	{
		.name = "ebi_8_2",
		.groups = ebi8_2_groups,
		.num_groups = ARRAY_SIZE(ebi8_2_groups),
	},
	{
		.name = "ebi_8_3",
		.groups = ebi8_3_groups,
		.num_groups = ARRAY_SIZE(ebi8_3_groups),
	},
	{
		.name = "ebi_8_4",
		.groups = ebi8_4_groups,
		.num_groups = ARRAY_SIZE(ebi8_4_groups),
	},
	{
		.name = "ebi_8_5",
		.groups = ebi8_5_groups,
		.num_groups = ARRAY_SIZE(ebi8_5_groups),
	},
	{
		.name = "ebi_16_0",
		.groups = ebi16_0_groups,
		.num_groups = ARRAY_SIZE(ebi16_0_groups),
	},
	{
		.name = "ebi_16_1",
		.groups = ebi16_1_groups,
		.num_groups = ARRAY_SIZE(ebi16_1_groups),
	},
	{
		.name = "ebi_16_2",
		.groups = ebi16_2_groups,
		.num_groups = ARRAY_SIZE(ebi16_2_groups),
	},
	{
		.name = "ebi_16_3",
		.groups = ebi16_3_groups,
		.num_groups = ARRAY_SIZE(ebi16_3_groups),
	},
	{
		.name = "ebi_16_4",
		.groups = ebi16_4_groups,
		.num_groups = ARRAY_SIZE(ebi16_4_groups),
	},
	{
		.name = "ebi_16_5",
		.groups = ebi16_5_groups,
		.num_groups = ARRAY_SIZE(ebi16_5_groups),
	},
	{
		.name = "usbh_pwren_ovc",
		.groups = usbh_power_groups,
		.num_groups = ARRAY_SIZE(usbh_power_groups),
	},
	{
		.name = "usbh_lite0_dp_dm",
		.groups = usbh_lite0_groups,
		.num_groups = ARRAY_SIZE(usbh_lite0_groups),
	},
	{
		.name = "usbh_lite1_dp_dm",
		.groups = usbh_lite1_groups,
		.num_groups = ARRAY_SIZE(usbh_lite1_groups),
	},
	{
		.name = "usbh_lite2_dp_dm",
		.groups = usbh_lite2_groups,
		.num_groups = ARRAY_SIZE(usbh_lite2_groups),
	},
	{
		.name = "usbh_lite3_dp_dm",
		.groups = usbh_lite3_groups,
		.num_groups = ARRAY_SIZE(usbh_lite3_groups),
	},
	{
		.name = "usbh_lite4_dp_dm",
		.groups = usbh_lite4_groups,
		.num_groups = ARRAY_SIZE(usbh_lite4_groups),
	},
	{
		.name = "usbh_lite5_dp_dm",
		.groups = usbh_lite5_groups,
		.num_groups = ARRAY_SIZE(usbh_lite5_groups),
	},
};


int nuc980_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(nuc980_functions);
}

const char *nuc980_get_fname(struct pinctrl_dev *pctldev, unsigned selector)
{
	return nuc980_functions[selector].name;
}

static int nuc980_get_groups(struct pinctrl_dev *pctldev, unsigned selector,
                             const char * const **groups,
                             unsigned * const num_groups)
{
	*groups = nuc980_functions[selector].groups;
	*num_groups = nuc980_functions[selector].num_groups;
	return 0;
}

/*
 * selector = data.nux.func, which is entry number in nuc980_functions,
 * and group = data.mux.group, which is entry number in nuc980_pmx_func
 * group is not used since some function use different setting between
 * different ports. for example UART....
 */
int nuc980_set_mux(struct pinctrl_dev *pctldev, unsigned selector,
                  unsigned group)
{
	unsigned int i, j;
	unsigned int reg, offset;

	//printk("set mux =>%x %x  %s\n", selector, group, nuc980_pinctrl_groups[group].name);
	for(i = 0; i < nuc980_pinctrl_groups[group].num_pins; i++) {
		j = nuc980_pinctrl_groups[group].pins[i];
		offset = (j >> 4) * 8 + ((j & 0x8) ? 4 : 0);

		reg = __raw_readl(REG_MFP_GPA_L + offset);
		reg = (reg & ~(0xF << ((j & 0x7) * 4))) | (nuc980_pinctrl_groups[group].func << ((j & 0x7) * 4));

		__raw_writel(reg, REG_MFP_GPA_L + offset);
	}
	return 0;
}


struct pinmux_ops nuc980_pmxops = {
	.get_functions_count = nuc980_get_functions_count,
	.get_function_name = nuc980_get_fname,
	.get_function_groups = nuc980_get_groups,
	.set_mux = nuc980_set_mux,
};

static struct pinctrl_desc nuc980_pinctrl_desc = {
	.name = "nuc980-pinctrl_desc",
	.pins = nuc980_pins,
	.npins = ARRAY_SIZE(nuc980_pins),
	.pctlops = &nuc980_pctrl_ops,
	.pmxops = &nuc980_pmxops,
	.owner = THIS_MODULE,
};

static const struct pinctrl_map nuc980_pinmap[] = {
	{
		.dev_name = "nuc980-nadc",
		.name = "nadc",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "nadc",
		.data.mux.group = "nadc_grp",
	},
	{
		.dev_name = "nuc980-emac0",
		.name = PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "emac0",
		.data.mux.group = "emac0_grp",
	},
	{
		.dev_name = "nuc980-emac1",
		.name = PINCTRL_STATE_DEFAULT,
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "emac1",
		.data.mux.group = "emac1_grp",
	},
//  {
//      .dev_name = "nuc980-emac0",
//      .name = "pps0",
//      .type = PIN_MAP_TYPE_MUX_GROUP,
//      .ctrl_dev_name = "pinctrl-nuc980",
//      .data.mux.function = "pps0",
//      .data.mux.group = "pps0_grp",
//  },
//  {
//      .dev_name = "nuc980-emac1",
//      .name = "pps1",
//      .type = PIN_MAP_TYPE_MUX_GROUP,
//      .ctrl_dev_name = "pinctrl-nuc980",
//      .data.mux.function = "pps1",
//      .data.mux.group = "pps1_grp",
//  },
	{
		.dev_name = "nuc980-videoin0",
		.name = "vcap0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "vcap0",
		.data.mux.group = "vcap0_grp",
	},
	{
		.dev_name = "nuc980-videoin1",
		.name = "vcap1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "vcap1",
		.data.mux.group = "vcap1_grp",
	},
	{
		.dev_name = "nuc980-fmi",
		.name = "sd0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "sd0",
		.data.mux.group = "sd0_grp",
	},
	{
		.dev_name = "nuc980-sdh",
		.name = "sd1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "sd1",
		.data.mux.group = "sd1_grp",
	},
	{
		.dev_name = "nuc980-fmi",
		.name = "nand",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "nand",
		.data.mux.group = "nand_grp",
	},
	{
		.dev_name = "nuc980-usbdev",
		.name = "usbd-vbusvld",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbd",
		.data.mux.group = "usbd_grp",
	},
	{
		.dev_name = "nuc980-i2c0",
		.name = "i2c0-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "i2c0",
		.data.mux.group = "i2c0_0_grp",
	},
	{
		.dev_name = "nuc980-i2c0",
		.name = "i2c0-PA_PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "i2c0",
		.data.mux.group = "i2c0_1_grp",
	},
	{
		.dev_name = "nuc980-i2c0",
		.name = "i2c0-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "i2c0",
		.data.mux.group = "i2c0_2_grp",
	},
	{
		.dev_name = "nuc980-i2c1",
		.name = "i2c1-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "i2c1",
		.data.mux.group = "i2c1_0_grp",
	},
	{
		.dev_name = "nuc980-i2c1",
		.name = "i2c1-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "i2c1",
		.data.mux.group = "i2c1_1_grp",
	},
	{
		.dev_name = "nuc980-i2c1",
		.name = "i2c1-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "i2c1",
		.data.mux.group = "i2c1_2_grp",
	},
	{
		.dev_name = "nuc980-i2c2",
		.name = "i2c2-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "i2c2",
		.data.mux.group = "i2c2_0_grp",
	},
	{
		.dev_name = "nuc980-i2c2",
		.name = "i2c2-PB_PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "i2c2",
		.data.mux.group = "i2c2_1_grp",
	},
	{
		.dev_name = "nuc980-i2c3",
		.name = "i2c3-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "i2c3",
		.data.mux.group = "i2c3_0_grp",
	},
	{
		.dev_name = "nuc980-i2c3",
		.name = "i2c3-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "i2c3",
		.data.mux.group = "i2c3_1_grp",
	},
	{
		.dev_name = "nuc980-audio-i2s",
		.name = "i2s-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "i2s",
		.data.mux.group = "i2s_0_grp",
	},
	{
		.dev_name = "nuc980-audio-i2s",
		.name = "i2s-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "i2s",
		.data.mux.group = "i2s_1_grp",
	},
	// {
	// .dev_name = "nuc980-uart0",
	// .name = PINCTRL_STATE_DEFAULT,
	// .type = PIN_MAP_TYPE_MUX_GROUP,
	// .ctrl_dev_name = "pinctrl-nuc980",
	// .data.mux.function = "uart0",
	// .data.mux.group = "uart0_grp",
	// },
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("nuc980-uart.0", "uart0_grp", "uart0"),  // hog
	{
		.dev_name = "nuc980-uart.1",
		.name = "uart1-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart1",
		.data.mux.group = "uart1_0_grp",
	},
	{
		.dev_name = "nuc980-uart.1",
		.name = "uart1-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart1",
		.data.mux.group = "uart1_1_grp",
	},
	{
		.dev_name = "nuc980-uart.1",
		.name = "uart1-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart1",
		.data.mux.group = "uart1_4_grp",
	},
	{
		.dev_name = "nuc980-uart.1",
		.name = "uart1-fc-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart1",
		.data.mux.group = "uart1_1_grp",
	},
	{
		.dev_name = "nuc980-uart.1",
		.name = "uart1-fc-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart1_fc",
		.data.mux.group = "uart1_2_grp",
	},
	{
		.dev_name = "nuc980-uart.1",
		.name = "uart1-fc-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart1",
		.data.mux.group = "uart1_4_grp",
	},
	{
		.dev_name = "nuc980-uart.1",
		.name = "uart1-fc-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart1_fc",
		.data.mux.group = "uart1_3_grp",
	},
	{
		.dev_name = "nuc980-uart.2",
		.name = "uart2-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart2",
		.data.mux.group = "uart2_1_grp",
	},
	{
		.dev_name = "nuc980-uart.2",
		.name = "uart2-PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart2",
		.data.mux.group = "uart2_4_grp",
	},
	{
		.dev_name = "nuc980-uart.2",
		.name = "uart2-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart2",
		.data.mux.group = "uart2_5_grp",
	},
	{
		.dev_name = "nuc980-uart.2",
		.name = "uart2-fc-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart2",
		.data.mux.group = "uart2_1_grp",
	},
	{
		.dev_name = "nuc980-uart.2",
		.name = "uart2-fc-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart2_fc",
		.data.mux.group = "uart2_0_grp",
	},
	{
		.dev_name = "nuc980-uart.2",
		.name = "uart2-fc-PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart2",
		.data.mux.group = "uart2_4_grp",
	},
	{
		.dev_name = "nuc980-uart.2",
		.name = "uart2-fc-PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart2_fc",
		.data.mux.group = "uart2_2_grp",
	},
	{
		.dev_name = "nuc980-uart.2",
		.name = "uart2-fc-PA_PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart2",
		.data.mux.group = "uart2_1_grp",
	},
	{
		.dev_name = "nuc980-uart.2",
		.name = "uart2-fc-PA_PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart2_fc",
		.data.mux.group = "uart2_3_grp",
	},
	{
		.dev_name = "nuc980-uart.3",
		.name = "uart3-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart3",
		.data.mux.group = "uart3_0_grp",
	},
	{
		.dev_name = "nuc980-uart.3",
		.name = "uart3-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart3",
		.data.mux.group = "uart3_1_grp",
	},
	{
		.dev_name = "nuc980-uart.3",
		.name = "uart3-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart3",
		.data.mux.group = "uart3_3_grp",
	},
	{
		.dev_name = "nuc980-uart.3",
		.name = "uart3-PB_PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart3",
		.data.mux.group = "uart3_6_grp",
	},
	{
		.dev_name = "nuc980-uart.3",
		.name = "uart3-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart3",
		.data.mux.group = "uart3_7_grp",
	},
	{
		.dev_name = "nuc980-uart.3",
		.name = "uart3-fc-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart3",
		.data.mux.group = "uart3_1_grp",
	},
	{
		.dev_name = "nuc980-uart.3",
		.name = "uart3-fc-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart3_fc",
		.data.mux.group = "uart3_2_grp",
	},
	{
		.dev_name = "nuc980-uart.3",
		.name = "uart3-fc-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart3",
		.data.mux.group = "uart3_3_grp",
	},
	{
		.dev_name = "nuc980-uart.3",
		.name = "uart3-fc-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart3_fc",
		.data.mux.group = "uart3_4_grp",
	},
	{
		.dev_name = "nuc980-uart.3",
		.name = "uart3-fc-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart3",
		.data.mux.group = "uart3_7_grp",
	},
	{
		.dev_name = "nuc980-uart.3",
		.name = "uart3-fc-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart3_fc",
		.data.mux.group = "uart3_5_grp",
	},
	{
		.dev_name = "nuc980-uart.4",
		.name = "uart4-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart4",
		.data.mux.group = "uart4_0_grp",
	},
	{
		.dev_name = "nuc980-uart.4",
		.name = "uart4-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart4",
		.data.mux.group = "uart4_1_grp",
	},
	{
		.dev_name = "nuc980-uart.4",
		.name = "uart4-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart4",
		.data.mux.group = "uart4_4_grp",
	},
	{
		.dev_name = "nuc980-uart.4",
		.name = "uart4-fc-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart4",
		.data.mux.group = "uart4_1_grp",
	},
	{
		.dev_name = "nuc980-uart.4",
		.name = "uart4-fc-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart4_fc",
		.data.mux.group = "uart4_2_grp",
	},
	{
		.dev_name = "nuc980-uart.4",
		.name = "uart4-fc-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart4",
		.data.mux.group = "uart4_4_grp",
	},
	{
		.dev_name = "nuc980-uart.4",
		.name = "uart4-fc-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart4_fc",
		.data.mux.group = "uart4_3_grp",
	},
	{
		.dev_name = "nuc980-uart.5",
		.name = "uart5-PG_0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart5",
		.data.mux.group = "uart5_0_grp",
	},
	{
		.dev_name = "nuc980-uart.5",
		.name = "uart5-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart5",
		.data.mux.group = "uart5_2_grp",
	},
	{
		.dev_name = "nuc980-uart.5",
		.name = "uart5-PG_1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart5",
		.data.mux.group = "uart5_4_grp",
	},
	{
		.dev_name = "nuc980-uart.5",
		.name = "uart5-fc-PG_0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart5",
		.data.mux.group = "uart5_0_grp",
	},
	{
		.dev_name = "nuc980-uart.5",
		.name = "uart5-fc-PG_0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart5_fc",
		.data.mux.group = "uart5_1_grp",
	},
	{
		.dev_name = "nuc980-uart.5",
		.name = "uart5-fc-PG_1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart5",
		.data.mux.group = "uart5_4_grp",
	},
	{
		.dev_name = "nuc980-uart.5",
		.name = "uart5-fc-PG_1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart5_fc",
		.data.mux.group = "uart5_3_grp",
	},
	{
		.dev_name = "nuc980-uart.6",
		.name = "uart6-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart6",
		.data.mux.group = "uart6_1_grp",
	},
	{
		.dev_name = "nuc980-uart.6",
		.name = "uart6-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart6",
		.data.mux.group = "uart6_3_grp",
	},
	{
		.dev_name = "nuc980-uart.6",
		.name = "uart6-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart6",
		.data.mux.group = "uart6_4_grp",
	},
	{
		.dev_name = "nuc980-uart.6",
		.name = "uart6-fc-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart6",
		.data.mux.group = "uart6_1_grp",
	},
	{
		.dev_name = "nuc980-uart.6",
		.name = "uart6-fc-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart6_fc",
		.data.mux.group = "uart6_0_grp",
	},
	{
		.dev_name = "nuc980-uart.6",
		.name = "uart6-fc-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart6",
		.data.mux.group = "uart6_3_grp",
	},
	{
		.dev_name = "nuc980-uart.6",
		.name = "uart6-fc-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart6_fc",
		.data.mux.group = "uart6_2_grp",
	},
	{
		.dev_name = "nuc980-uart.7",
		.name = "uart7-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart7",
		.data.mux.group = "uart7_0_grp",
	},
	{
		.dev_name = "nuc980-uart.7",
		.name = "uart7-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart7",
		.data.mux.group = "uart7_1_grp",
	},
	{
		.dev_name = "nuc980-uart.7",
		.name = "uart7-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart7",
		.data.mux.group = "uart7_3_grp",
	},
	{
		.dev_name = "nuc980-uart.7",
		.name = "uart7-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart7",
		.data.mux.group = "uart7_5_grp",
	},
	{
		.dev_name = "nuc980-uart.7",
		.name = "uart7-fc-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart7",
		.data.mux.group = "uart7_1_grp",
	},
	{
		.dev_name = "nuc980-uart.7",
		.name = "uart7-fc-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart7_fc",
		.data.mux.group = "uart7_2_grp",
	},
	{
		.dev_name = "nuc980-uart.7",
		.name = "uart7-fc-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart7",
		.data.mux.group = "uart7_5_grp",
	},
	{
		.dev_name = "nuc980-uart.7",
		.name = "uart7-fc-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart7_fc",
		.data.mux.group = "uart7_4_grp",
	},
	{
		.dev_name = "nuc980-uart.8",
		.name = "uart8-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart8",
		.data.mux.group = "uart8_0_grp",
	},
	{
		.dev_name = "nuc980-uart.8",
		.name = "uart8-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart8",
		.data.mux.group = "uart8_2_grp",
	},
	{
		.dev_name = "nuc980-uart.8",
		.name = "uart8-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart8",
		.data.mux.group = "uart8_3_grp",
	},
	{
		.dev_name = "nuc980-uart.8",
		.name = "uart8-fc-PA_PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart8",
		.data.mux.group = "uart8_0_grp",
	},
	{
		.dev_name = "nuc980-uart.8",
		.name = "uart8-fc-PA_PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart8_fc",
		.data.mux.group = "uart8_1_grp",
	},
	{
		.dev_name = "nuc980-uart.8",
		.name = "uart8-fc-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart8",
		.data.mux.group = "uart8_3_grp",
	},
	{
		.dev_name = "nuc980-uart.8",
		.name = "uart8-fc-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart8_fc",
		.data.mux.group = "uart8_4_grp",
	},
	{
		.dev_name = "nuc980-uart.9",
		.name = "uart9-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart9",
		.data.mux.group = "uart9_0_grp",
	},
	{
		.dev_name = "nuc980-uart.9",
		.name = "uart9-PE_0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart9",
		.data.mux.group = "uart9_3_grp",
	},
	{
		.dev_name = "nuc980-uart.9",
		.name = "uart9-PE_1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart9",
		.data.mux.group = "uart9_4_grp",
	},
	{
		.dev_name = "nuc980-uart.9",
		.name = "uart9-fc-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart9",
		.data.mux.group = "uart9_3_grp",
	},
	{
		.dev_name = "nuc980-uart.9",
		.name = "uart9-fc-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "uart9_fc",
		.data.mux.group = "uart9_2_grp",
	},
	{
		.dev_name = "nuc980-sc.0",
		.name = "sc0-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "sc0",
		.data.mux.group = "sc0_0_grp",
	},
	{
		.dev_name = "nuc980-sc.0",
		.name = "sc0-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "sc0",
		.data.mux.group = "sc0_1_grp",
	},
	{
		.dev_name = "nuc980-sc.0",
		.name = "scuart0-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "scuart0",
		.data.mux.group = "sc0_2_grp",
	},
	{
		.dev_name = "nuc980-sc.0",
		.name = "scuart0-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "scuart0",
		.data.mux.group = "sc0_3_grp",
	},
	{
		.dev_name = "nuc980-sc.1",
		.name = "sc1-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "sc1",
		.data.mux.group = "sc1_0_grp",
	},
	{
		.dev_name = "nuc980-sc.1",
		.name = "sc1-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "sc1",
		.data.mux.group = "sc1_1_grp",
	},
	{
		.dev_name = "nuc980-sc.1",
		.name = "scuart1-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "scuart1",
		.data.mux.group = "sc1_2_grp",
	},
	{
		.dev_name = "nuc980-sc.1",
		.name = "scuart1-PF",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "scuart1",
		.data.mux.group = "sc1_3_grp",
	},
	{
		.dev_name = "nuc980-qspi0.0",
		.name = "qspi0-normal",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "qspi0",
		.data.mux.group = "qspi0_3_grp",
	},
	{
		.dev_name = "nuc980-qspi0.0",
		.name = "qspi0-quad",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "qspi0_quad",
		.data.mux.group = "qspi0_2_grp",
	},
	{
		.dev_name = "nuc980-qspi0.0",
		.name = "qspi0-ss1-PA0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "qspi0_ss1",
		.data.mux.group = "qspi0_0_grp",
	},
	{
		.dev_name = "nuc980-qspi0.0",
		.name = "qspi0-ss1-PD0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "qspi0_ss1",
		.data.mux.group = "qspi0_1_grp",
	},
	{
		.dev_name = "nuc980-qspi0.0",
		.name = "qspi0-quad-ss1-PA0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "qspi0_quad_ss1",
		.data.mux.group = "qspi0_4_grp",
	},
	{
		.dev_name = "nuc980-qspi0.0",
		.name = "qspi0-quad-ss1-PD0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "qspi0_quad_ss1",
		.data.mux.group = "qspi0_5_grp",
	},
	{
		.dev_name = "nuc980-spi0.0",
		.name = "spi0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "spi0",
		.data.mux.group = "spi0_0_grp",
	},
	{
		.dev_name = "nuc980-spi0.0",
		.name = "spi0-ss1-PD1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "spi0_ss1",
		.data.mux.group = "spi0_1_grp",
	},
	{
		.dev_name = "nuc980-spi0.0",
		.name = "spi0-ss1-PG15",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "spi0_ss1",
		.data.mux.group = "spi0_2_grp",
	},
	{
		.dev_name = "nuc980-spi0.0",
		.name = "spi0-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "spi0",
		.data.mux.group = "spi0_3_grp",
	},
	{
		.dev_name = "nuc980-spi0.0",
		.name = "spi0-ss1-PC0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "spi0_ss1",
		.data.mux.group = "spi0_4_grp",
	},
	{
		.dev_name = "nuc980-spi1.0",
		.name = "spi1-PB9_12",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "spi1",
		.data.mux.group = "spi1_0_grp",
	},
	{
		.dev_name = "nuc980-spi1.0",
		.name = "spi1-PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "spi1",
		.data.mux.group = "spi1_1_grp",
	},
	{
		.dev_name = "nuc980-spi1.0",
		.name = "spi1-PG-ss1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "spi1_ss1",
		.data.mux.group = "spi1_2_grp",
	},
	{
		.dev_name = "nuc980-spi1.0",
		.name = "spi1-PB4_7",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "spi1",
		.data.mux.group = "spi1_3_grp",
	},
	{
		.dev_name = "nuc980-spi1.0",
		.name = "spi1-PB4_7-ss1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "spi1_ss1",
		.data.mux.group = "spi1_4_grp",
	},
	{
		.dev_name = "nuc980-can0",
		.name = "can0-PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can0",
		.data.mux.group = "can0_0_grp",
	},
	{
		.dev_name = "nuc980-can0",
		.name = "can0-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can0",
		.data.mux.group = "can0_1_grp",
	},
	{
		.dev_name = "nuc980-can0",
		.name = "can0-PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can0",
		.data.mux.group = "can0_2_grp",
	},
	{
		.dev_name = "nuc980-can0",
		.name = "can0-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can0",
		.data.mux.group = "can0_3_grp",
	},
	{
		.dev_name = "nuc980-can1",
		.name = "can1-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can1",
		.data.mux.group = "can1_0_grp",
	},
	{
		.dev_name = "nuc980-can1",
		.name = "can1-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can1",
		.data.mux.group = "can1_1_grp",
	},
	{
		.dev_name = "nuc980-can1",
		.name = "can1-PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can1",
		.data.mux.group = "can1_2_grp",
	},
	{
		.dev_name = "nuc980-can1",
		.name = "can1-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can1",
		.data.mux.group = "can1_3_grp",
	},
	{
		.dev_name = "nuc980-can2",
		.name = "can2-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can2",
		.data.mux.group = "can2_0_grp",
	},
	{
		.dev_name = "nuc980-can2",
		.name = "can2-PB",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can2",
		.data.mux.group = "can2_1_grp",
	},
	{
		.dev_name = "nuc980-can2",
		.name = "can2-PB_PC",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can2",
		.data.mux.group = "can2_2_grp",
	},
	{
		.dev_name = "nuc980-can2",
		.name = "can2-PD",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can2",
		.data.mux.group = "can2_3_grp",
	},
	{
		.dev_name = "nuc980-can2",
		.name = "can2-PE",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can2",
		.data.mux.group = "can2_4_grp",
	},
	{
		.dev_name = "nuc980-can3",
		.name = "can3-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can3",
		.data.mux.group = "can3_0_grp",
	},
	{
		.dev_name = "nuc980-can3",
		.name = "can3-PE_0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can3",
		.data.mux.group = "can3_1_grp",
	},
	{
		.dev_name = "nuc980-can3",
		.name = "can3-PE_1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "can3",
		.data.mux.group = "can3_2_grp",
	},
	{
		.dev_name = "nuc980-pwm0.0",
		.name = "pwm00-PG10",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm00",
		.data.mux.group = "pwm00_0_grp",
	},
	{
		.dev_name = "nuc980-pwm0.0",
		.name = "pwm00-PG0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm00",
		.data.mux.group = "pwm00_1_grp",
	},
	{
		.dev_name = "nuc980-pwm0.0",
		.name = "pwm00-PD12",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm00",
		.data.mux.group = "pwm00_2_grp",
	},
	{
		.dev_name = "nuc980-pwm0.0",
		.name = "pwm00-PF5",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm00",
		.data.mux.group = "pwm00_3_grp",
	},
	{
		.dev_name = "nuc980-pwm0.1",
		.name = "pwm01-PA15",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm01",
		.data.mux.group = "pwm01_0_grp",
	},
	{
		.dev_name = "nuc980-pwm0.1",
		.name = "pwm01-PG1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm01",
		.data.mux.group = "pwm01_1_grp",
	},
	{
		.dev_name = "nuc980-pwm0.1",
		.name = "pwm01-PD13",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm01",
		.data.mux.group = "pwm01_2_grp",
	},
	{
		.dev_name = "nuc980-pwm0.1",
		.name = "pwm01-PF6",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm01",
		.data.mux.group = "pwm01_3_grp",
	},
	{
		.dev_name = "nuc980-pwm0.2",
		.name = "pwm02-PA14",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm02",
		.data.mux.group = "pwm02_0_grp",
	},
	{
		.dev_name = "nuc980-pwm0.2",
		.name = "pwm02-PG2",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm02",
		.data.mux.group = "pwm02_1_grp",
	},
	{
		.dev_name = "nuc980-pwm0.2",
		.name = "pwm02-PD14",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm02",
		.data.mux.group = "pwm02_2_grp",
	},
	{
		.dev_name = "nuc980-pwm0.2",
		.name = "pwm02-PF7",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm02",
		.data.mux.group = "pwm02_3_grp",
	},
	{
		.dev_name = "nuc980-pwm0.2",
		.name = "pwm02-PB13",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm02",
		.data.mux.group = "pwm02_4_grp",
	},
	{
		.dev_name = "nuc980-pwm0.3",
		.name = "pwm03-PA13",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm03",
		.data.mux.group = "pwm03_0_grp",
	},
	{
		.dev_name = "nuc980-pwm0.3",
		.name = "pwm03-PG3",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm03",
		.data.mux.group = "pwm03_1_grp",
	},
	{
		.dev_name = "nuc980-pwm0.3",
		.name = "pwm03-PD15",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm03",
		.data.mux.group = "pwm03_2_grp",
	},
	{
		.dev_name = "nuc980-pwm0.3",
		.name = "pwm03-PF8",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm03",
		.data.mux.group = "pwm03_3_grp",
	},
	{
		.dev_name = "nuc980-pwm1.4",
		.name = "pwm10-PG6",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm10",
		.data.mux.group = "pwm10_0_grp",
	},
	{
		.dev_name = "nuc980-pwm1.4",
		.name = "pwm10-PB12",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm10",
		.data.mux.group = "pwm10_1_grp",
	},
	{
		.dev_name = "nuc980-pwm1.4",
		.name = "pwm10-PG11",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm10",
		.data.mux.group = "pwm10_2_grp",
	},
	{
		.dev_name = "nuc980-pwm1.4",
		.name = "pwm10-PF9",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm10",
		.data.mux.group = "pwm10_3_grp",
	},
	{
		.dev_name = "nuc980-pwm1.5",
		.name = "pwm11-PG7",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm11",
		.data.mux.group = "pwm11_0_grp",
	},
	{
		.dev_name = "nuc980-pwm1.5",
		.name = "pwm11-PB11",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm11",
		.data.mux.group = "pwm11_1_grp",
	},
	{
		.dev_name = "nuc980-pwm1.5",
		.name = "pwm11-PG12",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm11",
		.data.mux.group = "pwm11_2_grp",
	},
	{
		.dev_name = "nuc980-pwm1.5",
		.name = "pwm11-PF10",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm11",
		.data.mux.group = "pwm11_3_grp",
	},
	{
		.dev_name = "nuc980-pwm1.6",
		.name = "pwm12-PG8",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm12",
		.data.mux.group = "pwm12_0_grp",
	},
	{
		.dev_name = "nuc980-pwm1.6",
		.name = "pwm12-PB10",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm12",
		.data.mux.group = "pwm12_1_grp",
	},
	{
		.dev_name = "nuc980-pwm1.6",
		.name = "pwm12-PG13",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm12",
		.data.mux.group = "pwm12_2_grp",
	},
	{
		.dev_name = "nuc980-pwm1.6",
		.name = "pwm12-PE10",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm12",
		.data.mux.group = "pwm12_3_grp",
	},
	{
		.dev_name = "nuc980-pwm1.7",
		.name = "pwm13-PG9",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm13",
		.data.mux.group = "pwm13_0_grp",
	},
	{
		.dev_name = "nuc980-pwm1.7",
		.name = "pwm13-PB9",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm13",
		.data.mux.group = "pwm13_1_grp",
	},
	{
		.dev_name = "nuc980-pwm1.3",
		.name = "pwm13-PG14",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm13",
		.data.mux.group = "pwm13_2_grp",
	},
	{
		.dev_name = "nuc980-pwm1.7",
		.name = "pwm13-PE12",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "pwm13",
		.data.mux.group = "pwm13_3_grp",
	},
	{
		.dev_name = "nuc980-timer.0",
		.name = "etimer0-ecnt-PA0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer0_ecnt",
		.data.mux.group = "etimer0_0_grp",
	},
	{
		.dev_name = "nuc980-timer.0",
		.name = "etimer0-tgl-PB3",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer0_tgl",
		.data.mux.group = "etimer0_1_grp",
	},
	{
		.dev_name = "nuc980-timer.0",
		.name = "etimer0-cap-PB1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer0_cap",
		.data.mux.group = "etimer0_2_grp",
	},
	{
		.dev_name = "nuc980-timer.0",
		.name = "etimer0-tgl-PC0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer0_tgl",
		.data.mux.group = "etimer0_3_grp",
	},
	{
		.dev_name = "nuc980-timer.0",
		.name = "etimer0-cap-PB8",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer0_cap",
		.data.mux.group = "etimer0_4_grp",
	},
	{
		.dev_name = "nuc980-timer.0",
		.name = "etimer0-tgl-PB9",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer0_tgl",
		.data.mux.group = "etimer0_5_grp",
	},
	{
		.dev_name = "nuc980-timer.0",
		.name = "etimer0-cap-PB10",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer0_cap",
		.data.mux.group = "etimer0_6_grp",
	},
	{
		.dev_name = "nuc980-timer.0",
		.name = "etimer0-ecnt-PD6",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer0_ecnt",
		.data.mux.group = "etimer0_7_grp",
	},
	{
		.dev_name = "nuc980-timer.0",
		.name = "etimer0-ecnt-PF0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer0_ecnt",
		.data.mux.group = "etimer0_8_grp",
	},
	{
		.dev_name = "nuc980-timer.1",
		.name = "etimer1-ecnt-PA1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer1_ecnt",
		.data.mux.group = "etimer1_0_grp",
	},
	{
		.dev_name = "nuc980-timer.1",
		.name = "etimer1-tgl-PA14",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer1_tgl",
		.data.mux.group = "etimer1_1_grp",
	},
	{
		.dev_name = "nuc980-timer.1",
		.name = "etimer1-cap-PA13",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer1_cap",
		.data.mux.group = "etimer1_2_grp",
	},
	{
		.dev_name = "nuc980-timer.1",
		.name = "etimer1-tgl-PD0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer1_tgl",
		.data.mux.group = "etimer1_3_grp",
	},
	{
		.dev_name = "nuc980-timer.1",
		.name = "etimer1-cap-PD1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer1_cap",
		.data.mux.group = "etimer1_4_grp",
	},
	{
		.dev_name = "nuc980-timer.1",
		.name = "etimer1-ecnt-PD7",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer1_ecnt",
		.data.mux.group = "etimer1_5_grp",
	},
	{
		.dev_name = "nuc980-timer.1",
		.name = "etimer1-tgl-PG11",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer1_tgl",
		.data.mux.group = "etimer1_6_grp",
	},
	{
		.dev_name = "nuc980-timer.1",
		.name = "etimer1-cap-PG12",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer1_cap",
		.data.mux.group = "etimer1_7_grp",
	},
	{
		.dev_name = "nuc980-timer.1",
		.name = "etimer1-ecnt-PF1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer1_ecnt",
		.data.mux.group = "etimer1_8_grp",
	},
	{
		.dev_name = "nuc980-timer.1",
		.name = "etimer1-tgl-PF8",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer1_tgl",
		.data.mux.group = "etimer1_9_grp",
	},
	{
		.dev_name = "nuc980-timer.1",
		.name = "etimer1-cap-PF9",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer1_cap",
		.data.mux.group = "etimer1_A_grp",
	},
	{
		.dev_name = "nuc980-timer.2",
		.name = "etimer2-ecnt-PA2",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer2_ecnt",
		.data.mux.group = "etimer2_0_grp",
	},
	{
		.dev_name = "nuc980-timer.2",
		.name = "etimer2-tgl-PA10",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer2_tgl",
		.data.mux.group = "etimer2_1_grp",
	},
	{
		.dev_name = "nuc980-timer.2",
		.name = "etimer2-cap-PA9",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer2_cap",
		.data.mux.group = "etimer2_2_grp",
	},
	{
		.dev_name = "nuc980-timer.2",
		.name = "etimer2-tgl-PB12",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer2_tgl",
		.data.mux.group = "etimer2_3_grp",
	},
	{
		.dev_name = "nuc980-timer.2",
		.name = "etimer2-cap-PB11",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer2_cap",
		.data.mux.group = "etimer2_4_grp",
	},
	{
		.dev_name = "nuc980-timer.2",
		.name = "etimer2-ecnt-PD8",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer2_ecnt",
		.data.mux.group = "etimer2_5_grp",
	},
	{
		.dev_name = "nuc980-timer.2",
		.name = "etimer2-tgl-PD12",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer2_tgl",
		.data.mux.group = "etimer2_6_grp",
	},
	{
		.dev_name = "nuc980-timer.2",
		.name = "etimer2-cap-PD13",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer2_cap",
		.data.mux.group = "etimer2_7_grp",
	},
	{
		.dev_name = "nuc980-timer.2",
		.name = "etimer2-ecnt-PF2",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer2_ecnt",
		.data.mux.group = "etimer2_8_grp",
	},
	{
		.dev_name = "nuc980-timer.3",
		.name = "etimer3-ecnt-PA3",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer3_ecnt",
		.data.mux.group = "etimer3_0_grp",
	},
	{
		.dev_name = "nuc980-timer.3",
		.name = "etimer3-tgl-PA8",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer3_tgl",
		.data.mux.group = "etimer3_1_grp",
	},
	{
		.dev_name = "nuc980-timer.3",
		.name = "etimer3-cap-PA7",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer3_cap",
		.data.mux.group = "etimer3_2_grp",
	},
	{
		.dev_name = "nuc980-timer.3",
		.name = "etimer3-ecnt-PD9",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer3_ecnt",
		.data.mux.group = "etimer3_3_grp",
	},
	{
		.dev_name = "nuc980-timer.3",
		.name = "etimer3-tgl-PD14",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer3_tgl",
		.data.mux.group = "etimer3_4_grp",
	},
	{
		.dev_name = "nuc980-timer.3",
		.name = "etimer3-cap-PD15",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer3_cap",
		.data.mux.group = "etimer3_5_grp",
	},
	{
		.dev_name = "nuc980-timer.3",
		.name = "etimer3-ecnt-PF3",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer3_ecnt",
		.data.mux.group = "etimer3_6_grp",
	},
	{
		.dev_name = "nuc980-timer.4",
		.name = "etimer4-ecnt-PA4",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer4_ecnt",
		.data.mux.group = "etimer4_0_grp",
	},
	{
		.dev_name = "nuc980-timer.4",
		.name = "etimer4-tgl-PA12",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer4_tgl",
		.data.mux.group = "etimer4_1_grp",
	},
	{
		.dev_name = "nuc980-timer.4",
		.name = "etimer4-cap-PA11",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer4_cap",
		.data.mux.group = "etimer4_2_grp",
	},
	{
		.dev_name = "nuc980-timer.4",
		.name = "etimer4-tgl-PD3",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer4_tgl",
		.data.mux.group = "etimer4_3_grp",
	},
	{
		.dev_name = "nuc980-timer.4",
		.name = "etimer4-cap-PD2",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer4_cap",
		.data.mux.group = "etimer4_4_grp",
	},
	{
		.dev_name = "nuc980-timer.4",
		.name = "etimer4-ecnt-PD10",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer4_ecnt",
		.data.mux.group = "etimer4_5_grp",
	},
	{
		.dev_name = "nuc980-timer.4",
		.name = "etimer4-ecnt-PF4",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer4_ecnt",
		.data.mux.group = "etimer4_6_grp",
	},
	{
		.dev_name = "nuc980-timer.4",
		.name = "etimer4-tgl-PB13",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer4_tgl",
		.data.mux.group = "etimer4_7_grp",
	},
	{
		.dev_name = "nuc980-timer.4",
		.name = "etimer4-cap-PF6",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer4_cap",
		.data.mux.group = "etimer4_8_grp",
	},
	{
		.dev_name = "nuc980-timer.5",
		.name = "etimer5-ecnt-PA5",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer5_ecnt",
		.data.mux.group = "etimer5_0_grp",
	},
	{
		.dev_name = "nuc980-timer.5",
		.name = "etimer5-tgl-PG10",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer5_tgl",
		.data.mux.group = "etimer5_1_grp",
	},
	{
		.dev_name = "nuc980-timer.5",
		.name = "etimer5-cap-PA15",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer5_cap",
		.data.mux.group = "etimer5_2_grp",
	},
	{
		.dev_name = "nuc980-timer.5",
		.name = "etimer5-tgl-PD5",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer5_tgl",
		.data.mux.group = "etimer5_3_grp",
	},
	{
		.dev_name = "nuc980-timer.5",
		.name = "etimer5-cap-PD4",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer5_cap",
		.data.mux.group = "etimer5_4_grp",
	},
	{
		.dev_name = "nuc980-timer.5",
		.name = "etimer5-ecnt-PD11",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer5_ecnt",
		.data.mux.group = "etimer5_5_grp",
	},
	{
		.dev_name = "nuc980-timer.5",
		.name = "etimer5-ecnt-PF5",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer5_ecnt",
		.data.mux.group = "etimer5_6_grp",
	},
	{
		.dev_name = "nuc980-timer.5",
		.name = "etimer5-tgl-PF10",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer5_tgl",
		.data.mux.group = "etimer5_7_grp",
	},
	{
		.dev_name = "nuc980-timer.5",
		.name = "etimer5-cap-PF7",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "etimer5_cap",
		.data.mux.group = "etimer5_8_grp",
	},
	{
		.dev_name = "nuc980-jtag",
		.name = "jtag0-PG",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "jtag0",
		.data.mux.group = "jtag0_grp",
	},
	{
		.dev_name = "nuc980-jtag",
		.name = "jtag1-PA",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "jtag1",
		.data.mux.group = "jtag1_grp",
	},
	{
		.dev_name = "nuc980-gpio.1",
		.name = "eint0-PA0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "eint0",
		.data.mux.group = "eint0_0_grp",
	},
	{
		.dev_name = "nuc980-gpio.1",
		.name = "eint0-PA13",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "eint0",
		.data.mux.group = "eint0_1_grp",
	},
	{
		.dev_name = "nuc980-gpio.2",
		.name = "eint1-PA1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "eint1",
		.data.mux.group = "eint1_0_grp",
	},
	{
		.dev_name = "nuc980-gpio.2",
		.name = "eint1-PA14",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "eint1",
		.data.mux.group = "eint1_1_grp",
	},
	{
		.dev_name = "nuc980-gpio.3",
		.name = "eint2-PD0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "eint2",
		.data.mux.group = "eint2_0_grp",
	},
	{
		.dev_name = "nuc980-gpio.3",
		.name = "eint2-PE10",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "eint2",
		.data.mux.group = "eint2_1_grp",
	},
	{
		.dev_name = "nuc980-gpio.3",
		.name = "eint2-PB3",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "eint2",
		.data.mux.group = "eint2_2_grp",
	},
	{
		.dev_name = "nuc980-gpio.3",
		.name = "eint2-PB13",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "eint2",
		.data.mux.group = "eint2_3_grp",
	},
	{
		.dev_name = "nuc980-gpio.4",
		.name = "eint3-PD1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "eint3",
		.data.mux.group = "eint3_0_grp",
	},
	{
		.dev_name = "nuc980-gpio.4",
		.name = "eint3-PE12",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "eint3",
		.data.mux.group = "eint3_1_grp",
	},
	{
		.dev_name = "nuc980-gpio.4",
		.name = "eint3-PG15",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "eint3",
		.data.mux.group = "eint3_2_grp",
	},
	{
		.dev_name = "nuc980-ebi",
		.name = "ebi-8bit-0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "ebi_8_0",
		.data.mux.group = "ebi8_0_grp",
	},
	{
		.dev_name = "nuc980-ebi",
		.name = "ebi-8bit-1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "ebi_8_1",
		.data.mux.group = "ebi8_1_grp",
	},
	{
		.dev_name = "nuc980-ebi",
		.name = "ebi-8bit-2",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "ebi_8_2",
		.data.mux.group = "ebi8_2_grp",
	},
	{
		.dev_name = "nuc980-ebi",
		.name = "ebi-8bit-3",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "ebi_8_3",
		.data.mux.group = "ebi8_3_grp",
	},
	{
		.dev_name = "nuc980-ebi",
		.name = "ebi-8bit-4",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "ebi_8_4",
		.data.mux.group = "ebi8_4_grp",
	},
	{
		.dev_name = "nuc980-ebi",
		.name = "ebi-8bit-5",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "ebi_8_5",
		.data.mux.group = "ebi8_5_grp",
	},
	{
		.dev_name = "nuc980-ebi",
		.name = "ebi-16bit-0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "ebi_16_0",
		.data.mux.group = "ebi16_0_grp",
	},
	{
		.dev_name = "nuc980-ebi",
		.name = "ebi-16bit-1",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "ebi_16_1",
		.data.mux.group = "ebi16_1_grp",
	},
	{
		.dev_name = "nuc980-ebi",
		.name = "ebi-16bit-2",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "ebi_16_2",
		.data.mux.group = "ebi16_2_grp",
	},
	{
		.dev_name = "nuc980-ebi",
		.name = "ebi-16bit-3",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "ebi_16_3",
		.data.mux.group = "ebi16_3_grp",
	},
	{
		.dev_name = "nuc980-ebi",
		.name = "ebi-16bit-4",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "ebi_16_4",
		.data.mux.group = "ebi16_4_grp",
	},
	{
		.dev_name = "nuc980-ebi",
		.name = "ebi-16bit-5",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "ebi_16_5",
		.data.mux.group = "ebi16_5_grp",
	},
	{
		.dev_name = "nuc980-ehci",
		.name = "usbh_pwren_ovc_on",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_pwren_ovc",
		.data.mux.group = "usbh_power_1_grp",
	},
	{
		.dev_name = "nuc980-ehci",
		.name = "usbh_pwren_on",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_pwren_ovc",
		.data.mux.group = "usbh_power_2_grp",
	},
	{
		.dev_name = "nuc980-ehci",
		.name = "usbh_ovc_on",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_pwren_ovc",
		.data.mux.group = "usbh_power_3_grp",
	},
	{
		.dev_name = "nuc980-ohci.0",
		.name = "usbh_pwren_ovc_on",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_pwren_ovc",
		.data.mux.group = "usbh_power_1_grp",
	},
	{
		.dev_name = "nuc980-ohci.0",
		.name = "usbh_pwren_on",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_pwren_ovc",
		.data.mux.group = "usbh_power_2_grp",
	},
	{
		.dev_name = "nuc980-ohci.0",
		.name = "usbh_ovc_on",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_pwren_ovc",
		.data.mux.group = "usbh_power_3_grp",
	},
	{
		.dev_name = "nuc980-ohci.1",
		.name = "usbh_lite0_pb4_pb6",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite0_dp_dm",
		.data.mux.group = "usbh_lite0_1_grp",
	},
	{
		.dev_name = "nuc980-ohci.1",
		.name = "usbh_lite0_pb5_pb7",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite0_dp_dm",
		.data.mux.group = "usbh_lite0_2_grp",
	},
	{
		.dev_name = "nuc980-ohci.1",
		.name = "usbh_lite0_pb10_pb9",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite0_dp_dm",
		.data.mux.group = "usbh_lite0_3_grp",
	},
	{
		.dev_name = "nuc980-ohci.1",
		.name = "usbh_lite0_pd15_pd14",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite0_dp_dm",
		.data.mux.group = "usbh_lite0_4_grp",
	},
	{
		.dev_name = "nuc980-ohci.2",
		.name = "usbh_lite1_pe1_pe0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite1_dp_dm",
		.data.mux.group = "usbh_lite1_1_grp",
	},
	{
		.dev_name = "nuc980-ohci.2",
		.name = "usbh_lite1_pf1_pf0",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite1_dp_dm",
		.data.mux.group = "usbh_lite1_2_grp",
	},
	{
		.dev_name = "nuc980-ohci.3",
		.name = "usbh_lite2_pe3_pe2",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite2_dp_dm",
		.data.mux.group = "usbh_lite2_1_grp",
	},
	{
		.dev_name = "nuc980-ohci.3",
		.name = "usbh_lite2_pf3_pf2",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite2_dp_dm",
		.data.mux.group = "usbh_lite2_2_grp",
	},
	{
		.dev_name = "nuc980-ohci.4",
		.name = "usbh_lite3_pe5_pe4",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite3_dp_dm",
		.data.mux.group = "usbh_lite3_1_grp",
	},
	{
		.dev_name = "nuc980-ohci.4",
		.name = "usbh_lite3_pf5_pf4",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite3_dp_dm",
		.data.mux.group = "usbh_lite3_2_grp",
	},
	{
		.dev_name = "nuc980-ohci.5",
		.name = "usbh_lite4_pe7_pe6",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite4_dp_dm",
		.data.mux.group = "usbh_lite4_1_grp",
	},
	{
		.dev_name = "nuc980-ohci.5",
		.name = "usbh_lite4_pf7_pf6",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite4_dp_dm",
		.data.mux.group = "usbh_lite4_2_grp",
	},
	{
		.dev_name = "nuc980-ohci.5",
		.name = "usbh_lite4_pg10_pa15",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite4_dp_dm",
		.data.mux.group = "usbh_lite4_3_grp",
	},
	{
		.dev_name = "nuc980-ohci.5",
		.name = "usbh_lite4_pb13_pf6",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite4_dp_dm",
		.data.mux.group = "usbh_lite4_4_grp",
	},
	{
		.dev_name = "nuc980-ohci.6",
		.name = "usbh_lite5_pe9_pe8",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite5_dp_dm",
		.data.mux.group = "usbh_lite5_1_grp",
	},
	{
		.dev_name = "nuc980-ohci.6",
		.name = "usbh_lite5_pf9_pf8",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite5_dp_dm",
		.data.mux.group = "usbh_lite5_2_grp",
	},
	{
		.dev_name = "nuc980-ohci.6",
		.name = "usbh_lite5_pa14_pa13",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite5_dp_dm",
		.data.mux.group = "usbh_lite5_3_grp",
	},
	{
		.dev_name = "nuc980-ohci.6",
		.name = "usbh_lite5_pb12_pb11",
		.type = PIN_MAP_TYPE_MUX_GROUP,
		.ctrl_dev_name = "pinctrl-nuc980",
		.data.mux.function = "usbh_lite5_dp_dm",
		.data.mux.group = "usbh_lite5_4_grp",
	},
};


static int nuc980_pinctrl_probe(struct platform_device *pdev)
{
	struct pinctrl_dev *pctl;

	pctl = pinctrl_register(&nuc980_pinctrl_desc, &pdev->dev, NULL);
	if (IS_ERR(pctl)) {
		pr_err("could not register nuc980 pin driver\n");
	}

	platform_set_drvdata(pdev, pctl);

	return pinctrl_register_mappings(nuc980_pinmap, ARRAY_SIZE(nuc980_pinmap));

}

static int nuc980_pinctrl_remove(struct platform_device *pdev)
{
	struct pinctrl_dev *pctl = platform_get_drvdata(pdev);

	pinctrl_unregister(pctl);

	return 0;
}


static struct platform_driver nuc980_pinctrl_driver = {
	.driver = {
		.name = "pinctrl-nuc980",
		.owner = THIS_MODULE,
	},
	.probe = nuc980_pinctrl_probe,
	.remove = nuc980_pinctrl_remove,
};


static int __init nuc980_pinctrl_init(void)
{
	return platform_driver_register(&nuc980_pinctrl_driver);
}
arch_initcall(nuc980_pinctrl_init);

static void __exit nuc980_pinctrl_exit(void)
{
	platform_driver_unregister(&nuc980_pinctrl_driver);
}

module_exit(nuc980_pinctrl_exit);

MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_LICENSE("GPL");
