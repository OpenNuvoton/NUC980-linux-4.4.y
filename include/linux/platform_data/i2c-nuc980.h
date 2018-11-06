/*
 * include/linux/platform_data/i2c-nuc980.h
 *
 * Copyright (c) 2017 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */
#ifndef __ASM_ARCH_NUC980_I2C_H
#define __ASM_ARCH_NUC980_I2C_H
extern void nuc980_mfp_set_port_g(u32 pin, u32 func);

struct nuc980_platform_i2c {
	int bus_num;
	unsigned long bus_freq;
};
 
#endif	/* __ASM_ARCH_NUC980_I2C_H */

