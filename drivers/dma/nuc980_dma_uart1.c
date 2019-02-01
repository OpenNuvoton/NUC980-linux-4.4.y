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
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-serial.h>
#include <mach/regs-pdma.h>
#include <linux/clk.h>
#include <mach/sram.h>
#include <linux/platform_data/dma-nuc980.h>
#include <linux/kthread.h>

static volatile int slave_done_state=0;
static DECLARE_WAIT_QUEUE_HEAD(slave_done);



#define UARTx_BA UART1_BA
#define UARTx_PA UART1_PA
#define PDMA_UARTx_TX PDMA_UART1_TX
#define PDMA_UARTx_RX PDMA_UART1_RX


/* Allocate the channels for this example statically rather than dynamically for simplicity.
 */
static struct nuc980_ip_rx_dma dma_rx;
static struct nuc980_ip_tx_dma dma_tx;
static struct nuc980_mem_alloc src_mem_p;
static struct nuc980_mem_alloc dest_mem_p;
static struct nuc980_dma_done   dma_slave_done;

static void Slave_Compare(void)
{
	int i;
	for(i=0; i<src_mem_p.size; i++) {
		if(*((unsigned char *)(src_mem_p.vir_addr )+i)!=
		    *((unsigned char *)(dest_mem_p.vir_addr)+i)) {
			printk("[Compare Error]%d %d %d\n",i,
			       *((unsigned char *)(src_mem_p.vir_addr )+i),
			       *((unsigned char *)(dest_mem_p.vir_addr)+i)
			      );
			return;
		}
	}
	printk("Slave Compare Pass\n");
}

static void nuc980_slave_dma_callback(void *arg)
{
	struct nuc980_dma_done *done = arg;
	if(done->done)
		printk("transfer1 passed\n");
	if(done->timeout)
		printk("transfer1 timeout\n");

	slave_done_state = 1;
	wake_up_interruptible(&slave_done);
	//printk("nuc980_slave_dma_callback END\n");
	return;
}

static void Slave_Trigger(void)
{
	int i;
	struct nuc980_ip_rx_dma *pdma_rx=&dma_rx;
	struct nuc980_ip_tx_dma *pdma_tx=&dma_tx;
	struct nuc980_dma_config dma_crx,dma_ctx;
	dma_cookie_t        cookie;
	__raw_writel(__raw_readl(UARTx_BA+0x4)&~(0x1<<14),(UARTx_BA+0x4)); //Enable UARTx TX PDMA
	__raw_writel(__raw_readl(UARTx_BA+0x4)&~(0x1<<15),(UARTx_BA+0x4)); //Enable UARTx RX PDMA

	__raw_writel(__raw_readl(UARTx_BA+0x8)|0x6, (UARTx_BA+0x8));  // Reaset TX and RX FIFO

	for(i=0; i<src_mem_p.size; i++) {
		*((unsigned char *)src_mem_p.vir_addr+i)=i;
		*((unsigned char *)dest_mem_p.vir_addr+i)=0;
	}

	/* prepare the RX dma transfer */
	pdma_rx->slave_config.src_addr = UARTx_PA;
	pdma_rx->slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	pdma_rx->slave_config.src_maxburst = 1;
	pdma_rx->slave_config.direction = DMA_DEV_TO_MEM;
	pdma_rx->slave_config.device_fc = false;
	dmaengine_slave_config(pdma_rx->chan_rx,&(pdma_rx->slave_config));

	sg_init_table(pdma_rx->sgrx, 1);
	pdma_rx->sgrx[0].dma_address=dest_mem_p.phy_addr;
	pdma_rx->sgrx[0].length=dest_mem_p.size;
	dma_crx.reqsel = PDMA_UARTx_RX;
	dma_crx.timeout_counter = 0x3FF;
	dma_crx.timeout_prescaler = 7;
	dma_crx.en_sc = 0;
	pdma_rx->rxdesc=pdma_rx->chan_rx->device->device_prep_slave_sg(pdma_rx->chan_rx,
	                pdma_rx->sgrx,
	                1,
	                DMA_FROM_DEVICE,
	                DMA_PREP_INTERRUPT | DMA_CTRL_ACK,
	                (void *)&dma_crx); //PDMA Request Source Select
	if (!pdma_rx->rxdesc) {
		printk("pdma->rxdesc=NULL\n");
		while(1);
	}
	dma_slave_done.done = false;
	pdma_rx->rxdesc->callback = nuc980_slave_dma_callback;
	pdma_rx->rxdesc->callback_param = &dma_slave_done;
	//printk("pdma->rxdesc->callback_param=0x%08x\n",(unsigned int)pdma->rxdesc->callback_param);
	cookie = pdma_rx->rxdesc->tx_submit(pdma_rx->rxdesc);
	if (dma_submit_error(cookie)) {
		printk("rx cookie=%d\n",cookie);
		while(1);
	}

	/* prepare the TX dma transfer */
	pdma_tx->slave_config.dst_addr = UARTx_PA;
	pdma_tx->slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	pdma_tx->slave_config.dst_maxburst = 1;
	pdma_tx->slave_config.direction = DMA_MEM_TO_DEV;
	dmaengine_slave_config(pdma_tx->chan_tx,&(pdma_tx->slave_config));
	sg_init_table(pdma_tx->sgtx, 1);
	pdma_tx->sgtx[0].dma_address=src_mem_p.phy_addr;
	pdma_tx->sgtx[0].length=src_mem_p.size;
	dma_ctx.reqsel = PDMA_UARTx_TX;
	dma_ctx.timeout_counter = 0;
	dma_ctx.timeout_prescaler = 0;
	dma_ctx.en_sc = 0;
	pdma_tx->txdesc=pdma_tx->chan_tx->device->device_prep_slave_sg(pdma_tx->chan_tx,
	                pdma_tx->sgtx,
	                1,
	                DMA_TO_DEVICE,
	                DMA_PREP_INTERRUPT | DMA_CTRL_ACK,
	                (void *)&dma_ctx);
	if (!pdma_tx->txdesc) {
		printk("pdma->txdex=NULL\n");
		while(1);
	}
	pdma_tx->txdesc->callback = NULL;
	pdma_tx->txdesc->callback_param = NULL;
	cookie = pdma_tx->txdesc->tx_submit(pdma_tx->txdesc);
	if (dma_submit_error(cookie)) {
		printk("tx cookie=%d\n",cookie);
		while(1);
	}

	__raw_writel(__raw_readl(UARTx_BA+0x4)|(0x1<<14),UARTx_BA+0x4); //Enable UARTx TX PDMA
	__raw_writel(__raw_readl(UARTx_BA+0x4)|(0x1<<15),UARTx_BA+0x4); //Enable UARTx RX PDMA

	//printk("Slave_Trigger ENDEND\n");
	//return 0;
}

static volatile int slave_first=1;
static int Slave_Thread_Retrigger(void *data)
{
	printk("Slave_Thread_Retrigger...UART1\n");
	if(slave_first) {
		int i;
#if 0
		src_mem_p.size= PDMA_UART_TEST_LEN; //set to 1Kbytes
		src_mem_p.vir_addr = (unsigned int)dma_alloc_writecombine(NULL,
		                     PAGE_ALIGN(src_mem_p.size),
		                     &src_mem_p.phy_addr,
		                     GFP_KERNEL);

		for(i=0; i<src_mem_p.size; i++)
			*((unsigned char *)src_mem_p.vir_addr+i)=i;

		dest_mem_p.size=src_mem_p.size;
		dest_mem_p.vir_addr = (unsigned int)dma_alloc_writecombine(NULL,
		                      PAGE_ALIGN(dest_mem_p.size),
		                      &dest_mem_p.phy_addr,
		                      GFP_KERNEL);
#else
		src_mem_p.size= PDMA_UART_TEST_LEN; //set to 1Kbytes
		src_mem_p.vir_addr = (u32)sram_alloc(1024,&src_mem_p.phy_addr);
		for(i=0; i<src_mem_p.size; i++)
			*((unsigned char *)src_mem_p.vir_addr+i)=i;
		dest_mem_p.size=src_mem_p.size;
		dest_mem_p.vir_addr =(u32)sram_alloc(1024,&dest_mem_p.phy_addr);
#endif
		slave_first=0;
	}
	while(1) {
		if (wait_event_interruptible_timeout(slave_done, (slave_done_state != 0), 2000) == 0) {
			printk("UART1 time-out!!\n");
			while(1);
		} else {
			slave_done_state = 0;
			Slave_Compare();
			dma_slave_done.timeout = 0;
			dma_slave_done.done = 0;
		}
		Slave_Trigger();
	}
	return 0;
}
//drivers/spi/spi-atmel.c

static int __init MyDmaSlave_init(void)
{
	struct nuc980_ip_rx_dma *pdma_rx=&dma_rx;
	struct nuc980_ip_tx_dma *pdma_tx=&dma_tx;
	struct nuc980_dma_config dma_crx,dma_ctx;

	dma_cap_mask_t mask;
	dma_cookie_t        cookie;

	struct clk      *clk;
	clk = clk_get(NULL, "uart1");
	clk_prepare(clk);
	clk_enable(clk);
	clk = clk_get(NULL, "uart1_eclk");
	clk_prepare(clk);
	clk_enable(clk);

	printk("DMA Slave Test...........");
	/* Zero out the capability mask then initialize it for a slave channel that is
	 * private.
	 */
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_PRIVATE, mask);

	kthread_run(Slave_Thread_Retrigger, NULL, "PDMA_slave_test");
	while(slave_first);

	//UARTx initial
	__raw_writel(__raw_readl(UARTx_BA+0x10)|0x10,UARTx_BA+0x10); // internal loopback
	__raw_writel(__raw_readl(UARTx_BA+0x0C)|0x07,UARTx_BA+0x0C);  /* LCR */
	__raw_writel(0x30000066,UARTx_BA+0x24); /*BAUD : 12MHz reference clock input, 115200 */

	/* Request the DMA channel from the DMA engine and then use the device from
	 * the channel for the proxy channel also.
	 */
	pdma_rx->chan_rx = dma_request_channel(mask, NULL, NULL);
	if (!pdma_rx->chan_rx) {
		printk("RX DMA channel request error\n");
		return -1;
	}
	pdma_rx->chan_rx->private=(void *)1;
	//printk("RX %s: %s module removed\n",__func__, dma_chan_name(pdma->chan_rx));

	pdma_tx->chan_tx = dma_request_channel(mask, NULL, NULL);
	if (!pdma_tx->chan_tx) {
		printk("TX DMA channel request error\n");
		return -1;
	}
	pdma_tx->chan_tx->private=(void *)1;
	//printk("TX %s: %s module removed\n",__func__, dma_chan_name(pdma->chan_tx));

	/* prepare the RX dma transfer */
	pdma_rx->slave_config.src_addr = UARTx_PA;
	pdma_rx->slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	pdma_rx->slave_config.src_maxburst = 1;
	pdma_rx->slave_config.direction = DMA_DEV_TO_MEM;
	pdma_rx->slave_config.device_fc = false;
	dmaengine_slave_config(pdma_rx->chan_rx,&(pdma_rx->slave_config));

	sg_init_table(pdma_rx->sgrx, 1);
	pdma_rx->sgrx[0].dma_address=dest_mem_p.phy_addr;
	pdma_rx->sgrx[0].length=dest_mem_p.size;
	dma_crx.reqsel = PDMA_UARTx_RX;
	dma_crx.timeout_counter = 0x3FF;
	dma_crx.timeout_prescaler = 7;
	dma_crx.en_sc = 0;
	pdma_rx->rxdesc=pdma_rx->chan_rx->device->device_prep_slave_sg(pdma_rx->chan_rx,
	                pdma_rx->sgrx,
	                1,
	                DMA_FROM_DEVICE,
	                DMA_PREP_INTERRUPT | DMA_CTRL_ACK,
	                (void *)&dma_crx); //PDMA Request Source Select
	if (!pdma_rx->rxdesc) {
		printk("pdma->rxdes==NULL\n");
		while(1);
	}
	dma_slave_done.done = false;
	pdma_rx->rxdesc->callback = nuc980_slave_dma_callback;
	pdma_rx->rxdesc->callback_param = &dma_slave_done;
	cookie = pdma_rx->rxdesc->tx_submit(pdma_rx->rxdesc);
	if (dma_submit_error(cookie)) {
		printk("dma_submit_error\n");
		while(1);
	}
	/* prepare the TX dma transfer */
	pdma_tx->slave_config.dst_addr = UARTx_PA;
	pdma_tx->slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	pdma_tx->slave_config.dst_maxburst = 1;
	pdma_tx->slave_config.direction = DMA_MEM_TO_DEV;
	dmaengine_slave_config(pdma_tx->chan_tx,&(pdma_tx->slave_config));
	sg_init_table(pdma_tx->sgtx, 1);
	pdma_tx->sgtx[0].dma_address=src_mem_p.phy_addr;
	pdma_tx->sgtx[0].length=src_mem_p.size;
	dma_ctx.reqsel = PDMA_UARTx_TX;
	dma_ctx.timeout_counter = 0;
	dma_ctx.timeout_prescaler = 0;
	dma_ctx.en_sc = 0;
	pdma_tx->txdesc=pdma_tx->chan_tx->device->device_prep_slave_sg(pdma_tx->chan_tx,
	                pdma_tx->sgtx,
	                1,
	                DMA_TO_DEVICE,
	                DMA_PREP_INTERRUPT | DMA_CTRL_ACK,
	                (void *)&dma_ctx);
	if (!pdma_tx->txdesc) {
		printk("pdma->txdes==NULL\n");
		while(1);
	}
	cookie = pdma_tx->txdesc->tx_submit(pdma_tx->txdesc);
	if (dma_submit_error(cookie)) {
		printk("dma_submit_error\n");
		while(1);
	}

	__raw_writel(__raw_readl(UARTx_BA+0x4)|(0x1<<14),UARTx_BA+0x4); //Enable UARTx TX PDMA
	__raw_writel(__raw_readl(UARTx_BA+0x4)|(0x1<<15),UARTx_BA+0x4); //Enable UARTx RX PDMA


	//while(1);
	return 0;
}

static void __exit MyDmaSlave_exit(void)
{
	return;
}

module_init(MyDmaSlave_init);
module_exit(MyDmaSlave_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mister X");
