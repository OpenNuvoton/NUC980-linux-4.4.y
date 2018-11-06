/* linux/arch/arm/mach-nuc980/include/mach/nuc980-timer.h
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
#ifndef _NUC980_TIMER_H_
#define _NUC980_TIMER_H_

#include <linux/types.h>
#include <linux/ioctl.h>

#define TMR_IOC_MAGIC		'e'
#define TMR_IOC_MAXNR		3

#define TMR_IOC_STOP			_IO(TMR_IOC_MAGIC, 0)
#define TMR_IOC_TOGGLE			_IOW(TMR_IOC_MAGIC, 1, unsigned int *)
#define TMR_IOC_FREE_COUNTING		_IOW(TMR_IOC_MAGIC, 2, unsigned int *)
#define TMR_IOC_TRIGGER_COUNTING	_IOW(TMR_IOC_MAGIC, 3, unsigned int *)
#define TMR_IOC_PERIODIC		_IOW(TMR_IOC_MAGIC, 4, unsigned int *)
#define TMR_IOC_PERIODIC_FOR_WKUP	_IOW(TMR_IOC_MAGIC, 5, unsigned int *)
#define TMR_IOC_CLKLXT			_IOW(TMR_IOC_MAGIC, 6, unsigned int *)
#define TMR_IOC_CLKHXT			_IOW(TMR_IOC_MAGIC, 7, unsigned int *)
#define TMR_IOC_EVENT_COUNTING		_IOW(TMR_IOC_MAGIC, 8, unsigned int *)
// Valid parameters for capture mode ioctls
#define TMR_CAP_EDGE_FF			0x00000
#define TMR_CAP_EDGE_RR			0x40000
#define TMR_CAP_EDGE_FR			0x80000
#define TMR_CAP_EDGE_RF			0xC0000

#define TMR_EXTCNT_EDGE_RF		0x2000
#define TMR_EXTCNT_EDGE_FF		0x0000

#endif
