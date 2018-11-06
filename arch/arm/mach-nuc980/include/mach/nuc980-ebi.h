/* linux/arch/arm/mach-nuc980/include/mach/nuc980-ebi.h
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
#ifndef _NUC980_EBI_H_
#define _NUC980_EBI_H_

#include <linux/types.h>
#include <linux/ioctl.h>

#define EBI_IOC_MAGIC		'e'
#define EBI_IOC_SET			_IOW(EBI_IOC_MAGIC, 0, unsigned int *)

struct nuc980_set_ebi {
	unsigned int bank;
	unsigned int busmode;
	unsigned int CSActiveLevel;
	unsigned int base;
	unsigned int size;
	unsigned int width;
	unsigned int timing;
	//unsigned int tACC;
	//unsigned int tAHD;
	//unsigned int W2X;
	//unsigned int RAHDOFF;
	//unsigned int WAHDOFF;
	//unsigned int R2R;
};

#endif
