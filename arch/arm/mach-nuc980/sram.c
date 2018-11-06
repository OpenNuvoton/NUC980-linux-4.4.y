/*
 * mach-nuc980/sram.c - nuc980 simple SRAM allocator
 *
 * Copyright (C) 2018
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/genalloc.h>
#include <mach/map.h>
#include <mach/sram.h>
#include <linux/sizes.h>

/* nuc980 SRAM size */
#define SRAM_SIZE SZ_16K

static struct gen_pool *sram_pool;

struct gen_pool *sram_get_gen_pool(void)
{
	return sram_pool;
}

void *sram_alloc(size_t len, dma_addr_t *dma)
{
	unsigned long vaddr;
	dma_addr_t dma_base = NUC980_PA_SRAM;

	if (dma)
		*dma = 0;
	if (!sram_pool || (dma && !dma_base))
		return NULL;

	vaddr = gen_pool_alloc(sram_pool, len);
	if (!vaddr)
		return NULL;

	if (dma)
		*dma = gen_pool_virt_to_phys(sram_pool, vaddr);
	return (void *)vaddr;

}
EXPORT_SYMBOL(sram_alloc);

void sram_free(void *addr, size_t len)
{
	gen_pool_free(sram_pool, (unsigned long) addr, len);
}
EXPORT_SYMBOL(sram_free);


/*
 * REVISIT This supports CPU and DMA access to/from SRAM, but it
 * doesn't (yet?) support some other notable uses of SRAM:  as TCM
 * for data and/or instructions; and holding code needed to enter
 * and exit suspend states (while DRAM can't be used).
 */
static int __init sram_init(void)
{
	phys_addr_t phys = NUC980_PA_SRAM;
	unsigned len = SRAM_SIZE;
	int status = 0;
	void __iomem *addr;

	if (len) {
		len = min_t(unsigned, len, SRAM_SIZE);
		sram_pool = gen_pool_create(ilog2(SRAM_GRANULARITY), -1);
		if (!sram_pool)
			status = -ENOMEM;
	}

	if (sram_pool) {
		addr = ioremap(phys, len);
		if (!addr)
			return -ENOMEM;
		status = gen_pool_add_virt(sram_pool, (unsigned long) addr,
					   phys, len, -1);
		if (status < 0)
			iounmap(addr);
	}

	WARN_ON(status < 0);
	return status;
}
core_initcall(sram_init);
