/*
 * Copyright (c) 2018 Nuvoton Technology Corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <linux/platform_data/dma-nuc980.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-pdma.h>

#include <linux/kthread.h>

static volatile int m2m_done_state=0;
static DECLARE_WAIT_QUEUE_HEAD(m2m_done);


/* The following data structure represents a single channel of DMA, transmit or receive in the case
 * when using DMA.  It contains all the data to be maintained for the channel.
 */
struct dma_proxy_channel {
	struct device *proxy_device_p;              /* character device support */
	struct device *dma_device_p;
	struct nuc980_mem_alloc src_mem_p;
	struct nuc980_mem_alloc dest_mem_p;
	struct nuc980_mem_dma_param mem_dma_p;
	dev_t dev_node;
	struct cdev cdev;
	struct class *class_p;

	struct dma_chan *channel_p;             /* dma support */
	struct completion cmp;
	dma_cookie_t cookie;
	dma_addr_t dma_handle;
	u32 direction;                      /* DMA_MEM_TO_MEM */
};


/* Allocate the channels for this example statically rather than dynamically for simplicity.
 */
static struct dma_proxy_channel channels;

struct nuc980_dma_done  dma_m2m_done;// = { .wait = &done_wait };


int M2M_Compare(void)
{
	int i;
	struct dma_proxy_channel *pchannel_p=&channels;
	for(i=0; i<pchannel_p->src_mem_p.size; i++) {
		if(*((unsigned char *)(pchannel_p->src_mem_p.vir_addr )+i)!=
		    *((unsigned char *)(pchannel_p->dest_mem_p.vir_addr)+i)) {
			printk("[Compare Error]%d %d %d\n",i,
			       *((unsigned char *)(pchannel_p->src_mem_p.vir_addr )+i),
			       *((unsigned char *)(pchannel_p->dest_mem_p.vir_addr)+i)
			      );
			return 0;
		}
	}
	printk("M2M Compare Data Pass\n");
	return 0;
}

static void nuc980_m2m_dma_callback(void *arg)
{

	struct nuc980_dma_done *done = arg;
	//printk("m2m done->done=%d, done->timeout=%d\n",done->done,done->timeout);
	done->done = true;
	m2m_done_state = 1;
	wake_up_interruptible(&m2m_done);
	return;
}

int M2M_Trigger(void)
{
	int i;
	struct dma_async_tx_descriptor *tx = NULL;
	dma_cookie_t        cookie;
	struct dma_proxy_channel *pchannel_p=&channels;
	struct nuc980_mem_dma_param *dma_param = &pchannel_p->mem_dma_p;
	dma_cap_mask_t mask;
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	for(i=0; i<pchannel_p->src_mem_p.size; i++) {
		*((unsigned char *)pchannel_p->src_mem_p.vir_addr+i)=i;
		*((unsigned char *)pchannel_p->dest_mem_p.vir_addr+i)=0;
	}

	dma_param->src_addr=pchannel_p->src_mem_p.phy_addr;
	dma_param->dst_addr=pchannel_p->dest_mem_p.phy_addr;
	dma_param->size=pchannel_p->dest_mem_p.size;
	tx = pchannel_p->channel_p->device->device_prep_dma_memcpy(pchannel_p->channel_p,
	                dma_param->dst_addr,
	                dma_param->src_addr,
	                dma_param->size,
	                dma_param->cfg);

	dma_m2m_done.done = false;
	tx->callback = nuc980_m2m_dma_callback;
	tx->callback_param = &dma_m2m_done;
	cookie = tx->tx_submit(tx);
	return 0;
}

extern int  emac0_m2m_state;

volatile int m2m_first=1;
static int M2M_Thread_Retrigger(void *data)
{
	if(m2m_first) {
		int i;
		struct dma_proxy_channel *pchannel_p=&channels;
		pchannel_p->src_mem_p.size=1*1024; //set to 1Kbytes
		pchannel_p->src_mem_p.vir_addr = (unsigned int)dma_alloc_writecombine(NULL,
		                                 PAGE_ALIGN(pchannel_p->src_mem_p.size),
		                                 &pchannel_p->src_mem_p.phy_addr,
		                                 GFP_KERNEL);

		for(i=0; i<pchannel_p->src_mem_p.size; i++)
			*((unsigned char *)pchannel_p->src_mem_p.vir_addr+i)=i;

		pchannel_p->dest_mem_p.size=pchannel_p->src_mem_p.size;
		pchannel_p->dest_mem_p.vir_addr = (unsigned int)dma_alloc_writecombine(NULL,
		                                  PAGE_ALIGN(pchannel_p->dest_mem_p.size),
		                                  &pchannel_p->dest_mem_p.phy_addr,
		                                  GFP_KERNEL);
		m2m_first=0;
	}
	while(1) {
		wait_event_interruptible(m2m_done, (m2m_done_state != 0));
		m2m_done_state=0;
		M2M_Compare();
		msleep(100);
		M2M_Trigger();
	}
	return 0;
}

static int __init MyDmaM2M_init(void)
{
	struct dma_proxy_channel *pchannel_p=&channels;
	struct nuc980_mem_dma_param *dma_param = &pchannel_p->mem_dma_p;
	dma_cap_mask_t mask;

	struct dma_async_tx_descriptor *tx = NULL;
	dma_cookie_t        cookie;

	printk("nuc980 dma m2m test...........");
	/* Zero out the capability mask then initialize it for a slave channel that is
	 * private.
	 */
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	/* Request the DMA channel from the DMA engine and then use the device from
	 * the channel for the proxy channel also.
	 */
	pchannel_p->channel_p = dma_request_channel(mask, NULL, NULL);
	if (!pchannel_p->channel_p) {
		dev_err(pchannel_p->dma_device_p, "DMA channel request error\n");
		return -1;
	}
	printk("PDMA M2M channel: %s\n", dma_chan_name(pchannel_p->channel_p));
	kthread_run(M2M_Thread_Retrigger, pchannel_p, "PDMA_m2m_test");
	pchannel_p->channel_p->private=(void *)1;
	pchannel_p->direction = DMA_MEM_TO_MEM;
	while(m2m_first);
	dma_param->src_addr=pchannel_p->src_mem_p.phy_addr;
	dma_param->dst_addr=pchannel_p->dest_mem_p.phy_addr;
	dma_param->size=pchannel_p->dest_mem_p.size;
	tx = pchannel_p->channel_p->device->device_prep_dma_memcpy(pchannel_p->channel_p,
	                dma_param->dst_addr,
	                dma_param->src_addr,
	                dma_param->size,
	                dma_param->cfg);

	dma_m2m_done.done = false;
	tx->callback = nuc980_m2m_dma_callback;
	tx->callback_param = &dma_m2m_done;
	cookie = tx->tx_submit(tx);

	return 0;
}

static void __exit MyDmaM2M_exit(void)
{
	return;
}

module_init(MyDmaM2M_init);
module_exit(MyDmaM2M_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mister X");
