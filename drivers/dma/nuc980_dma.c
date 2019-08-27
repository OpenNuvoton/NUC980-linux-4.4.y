/*
 * Copyright (c) 2018 Nuvoton Technology Corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/kthread.h>
/*
 * Copyright (c) 2018 Nuvoton Technology Corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */


#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/platform_data/dma-nuc980.h>
#include <asm/irq.h>
#include <mach/map.h>
#include <mach/regs-pdma.h>
#include <mach/regs-clock.h>
#include <linux/platform_device.h>

//-----------------------------------------------------------------------------------
#define PDMA0               ((PDMA_T *)  NUC980_VA_PDMA0)
#define PDMA1               ((PDMA_T *)  NUC980_VA_PDMA1)
//-----------------------------------------------------------------------------------

#include "dmaengine.h"

#ifdef CONFIG_USE_OF
extern struct nuc980_dma_platform_data nuc980_dma_data;
#endif

#if 0
#define ENTRY()                 printk("[%-20s] : Enter...\n", __FUNCTION__)
#define LEAVE()                 printk("[%-20s] : Leave...\n", __FUNCTION__)
#else
#define ENTRY()
#define LEAVE()
#endif

#if 0
#define DMA_DEBUG printk
#define DMA_DEBUG2 printk
#else
#define DMA_DEBUG(fmt,args...)
#define DMA_DEBUG2(fmt,args...)
#endif


#define DMA_MAX_CHAN_DESCRIPTORS    32
#define DMA_MAX_CHAN_BYTES      0x10000

struct nuc980_dma_engine;

/**
 * struct nuc980_dma_desc - NUC980 specific transaction descriptor
 * @src_addr: source address of the transaction
 * @dst_addr: destination address of the transaction
 * @size: size of the transaction (in bytes)
 * @complete: this descriptor is completed
 * @txd: dmaengine API descriptor
 * @tx_list: list of linked descriptors
 * @node: link used for putting this into a channel queue
 */
struct nuc980_dma_desc {
	u32             src_addr;
	u32             dst_addr;
	size_t              size;
	u32                     ctl;
	bool                    complete;
	struct dma_async_tx_descriptor  txd;
	struct list_head        tx_list;
	struct list_head        node;
	struct nuc980_dma_config config;
	DSCT_T dsct[2];
	u32 			dir;
};

/**
 * struct nuc980_dma_chan - an NUC980 DMA M2M channel
 * @chan: dmaengine API channel
 * @edma: pointer to to the engine device
 * @regs: memory mapped registers
 * @irq: interrupt number of the channel
 * @clk: clock used by this channel
 * @tasklet: channel specific tasklet used for callbacks
 * @lock: lock protecting the fields following
 * @flags: flags for the channel
 * @buffer: which buffer to use next (0/1)
 * @active: flattened chain of descriptors currently being processed
 * @queue: pending descriptors which are handled next
 * @free_list: list of free descriptors which can be used
 * @runtime_addr: physical address currently used as dest/src (M2M only). This
 *                is set via %DMA_SLAVE_CONFIG before slave operation is
 *                prepared
 * @runtime_ctrl: M2M runtime values for the control register.
 *
 * As NUC980 DMA controller doesn't support real chained DMA descriptors we
 * will have slightly different scheme here: @active points to a head of
 * flattened DMA descriptor chain.
 *
 * @queue holds pending transactions. These are linked through the first
 * descriptor in the chain. When a descriptor is moved to the @active queue,
 * the first and chained descriptors are flattened into a single list.
 *
 * @chan.private holds pointer to &struct nuc980_dma_data which contains
 * necessary channel configuration information. For memcpy channels this must
 * be %NULL.
 */
struct nuc980_dma_chan {
	struct dma_chan         chan;
	const struct nuc980_dma_engine  *edma;
	void __iomem            *regs;
	int             irq;
	u32             id;
	struct tasklet_struct       tasklet;
	struct tasklet_struct       tasklet_sc;
	/* protects the fields following */
	spinlock_t          lock;
	spinlock_t      wklock;
	unsigned long           flags;
	/* Channel is configured for cyclic transfers */
#define NUC980_DMA_IS_CYCLIC        0

	int             buffer;
	struct list_head        active;
	struct list_head        queue;
	struct list_head        free_list;
	u32             runtime_addr;
	u32             runtime_ctrl;
};

struct mutex pdma0_mutex; /* shared between the threads */
struct mutex pdma1_mutex; /* shared between the threads */

/**
 * struct nuc980_dma_engine - the NUC980 DMA engine instance
 * @dma_dev: holds the dmaengine device
 * @hw_setup: method which sets the channel up for operation
 * @hw_shutdown: shuts the channel down and flushes whatever is left
 * @hw_submit: pushes active descriptor(s) to the hardware
 * @hw_interrupt: handle the interrupt
 * @num_channels: number of channels for this instance
 * @channels: array of channels
 *
 * There is one instance of this struct for the M2M channels.
 * hw_xxx() methods are used to perform operations which are
 * different on M2M and M2P channels. These methods are called with channel
 * lock held and interrupts disabled so they cannot sleep.
 */
struct nuc980_dma_engine {
	struct dma_device   dma_dev;
	bool          m2m;
	int         (*hw_setup)(struct nuc980_dma_chan *);
	void        (*hw_shutdown)(struct nuc980_dma_chan *);
	void        (*hw_submit)(struct nuc980_dma_chan *);
	int         (*hw_interrupt)(struct nuc980_dma_chan *);
#define INTERRUPT_UNKNOWN   0
#define INTERRUPT_DONE      1
#define INTERRUPT_NEXT_BUFFER   2
#define INTERRUPT_TIMEOUT         3

	size_t          num_channels;
	struct nuc980_dma_chan  channels[];
};

static inline struct device *chan2dev(struct nuc980_dma_chan *edmac) {
	ENTRY();
	return &edmac->chan.dev->device;
}

static struct nuc980_dma_chan *to_nuc980_dma_chan(struct dma_chan *chan) {
	ENTRY();
	return container_of(chan, struct nuc980_dma_chan, chan);
}


void nuc980_set_transfer_mode(PDMA_T * pdma,uint32_t u32Ch,uint32_t u32Peripheral)
{

	switch(u32Ch) {
	case 0ul:
		pdma->REQSEL0_3 = (pdma->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC0_Msk) | u32Peripheral;
		break;
	case 1ul:
		pdma->REQSEL0_3 = (pdma->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC1_Msk) | (u32Peripheral << PDMA_REQSEL0_3_REQSRC1_Pos);
		break;
	case 2ul:
		pdma->REQSEL0_3 = (pdma->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC2_Msk) | (u32Peripheral << PDMA_REQSEL0_3_REQSRC2_Pos);
		break;
	case 3ul:
		pdma->REQSEL0_3 = (pdma->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC3_Msk) | (u32Peripheral << PDMA_REQSEL0_3_REQSRC3_Pos);
		break;
	case 4ul:
		pdma->REQSEL4_7 = (pdma->REQSEL4_7 & ~PDMA_REQSEL4_7_REQSRC4_Msk) | u32Peripheral;
		break;
	case 5ul:
		pdma->REQSEL4_7 = (pdma->REQSEL4_7 & ~PDMA_REQSEL4_7_REQSRC5_Msk) | (u32Peripheral << PDMA_REQSEL4_7_REQSRC5_Pos);
		break;
	case 6ul:
		pdma->REQSEL4_7 = (pdma->REQSEL4_7 & ~PDMA_REQSEL4_7_REQSRC6_Msk) | (u32Peripheral << PDMA_REQSEL4_7_REQSRC6_Pos);
		break;
	case 7ul:
		pdma->REQSEL4_7 = (pdma->REQSEL4_7 & ~PDMA_REQSEL4_7_REQSRC7_Msk) | (u32Peripheral << PDMA_REQSEL4_7_REQSRC7_Pos);
		break;
	case 8ul:
		pdma->REQSEL8_11 = (pdma->REQSEL8_11 & ~PDMA_REQSEL8_11_REQSRC8_Msk) | u32Peripheral;
		break;
	case 9ul:
		pdma->REQSEL8_11 = (pdma->REQSEL8_11 & ~PDMA_REQSEL8_11_REQSRC9_Msk) | (u32Peripheral << PDMA_REQSEL8_11_REQSRC9_Pos);
		break;
	case 10ul:
		pdma->REQSEL8_11 = (pdma->REQSEL8_11 & ~PDMA_REQSEL8_11_REQSRC10_Msk) | (u32Peripheral << PDMA_REQSEL8_11_REQSRC10_Pos);
		break;
	case 11ul:
		pdma->REQSEL8_11 = (pdma->REQSEL8_11 & ~PDMA_REQSEL8_11_REQSRC11_Msk) | (u32Peripheral << PDMA_REQSEL8_11_REQSRC11_Pos);
		break;
	case 12ul:
		pdma->REQSEL12_15 = (pdma->REQSEL12_15 & ~PDMA_REQSEL12_15_REQSRC12_Msk) | u32Peripheral;
		break;
	case 13ul:
		pdma->REQSEL12_15 = (pdma->REQSEL12_15 & ~PDMA_REQSEL12_15_REQSRC13_Msk) | (u32Peripheral << PDMA_REQSEL12_15_REQSRC13_Pos);
		break;
	case 14ul:
		pdma->REQSEL12_15 = (pdma->REQSEL12_15 & ~PDMA_REQSEL12_15_REQSRC14_Msk) | (u32Peripheral << PDMA_REQSEL12_15_REQSRC14_Pos);
		break;
	case 15ul:
		pdma->REQSEL12_15 = (pdma->REQSEL12_15 & ~PDMA_REQSEL12_15_REQSRC15_Msk) | (u32Peripheral << PDMA_REQSEL12_15_REQSRC15_Pos);
		break;
	default:
		break;
	}
}

/**
 * nuc980_dma_set_active - set new active descriptor chain
 * @edmac: channel
 * @desc: head of the new active descriptor chain
 *
 * Sets @desc to be the head of the new active descriptor chain. This is the
 * chain which is processed next. The active list must be empty before calling
 * this function.
 *
 * Called with @edmac->lock held and interrupts disabled.
 */
static void nuc980_dma_set_active(struct nuc980_dma_chan *edmac,
                                  struct nuc980_dma_desc *desc)
{
	ENTRY();
	BUG_ON(!list_empty(&edmac->active));

	list_add_tail(&desc->node, &edmac->active);

	/* Flatten the @desc->tx_list chain into @edmac->active list */
	while (!list_empty(&desc->tx_list)) {
		struct nuc980_dma_desc *d = list_first_entry(&desc->tx_list,
		                            struct nuc980_dma_desc, node);

		/*
		 * We copy the callback parameters from the first descriptor
		 * to all the chained descriptors. This way we can call the
		 * callback without having to find out the first descriptor in
		 * the chain. Useful for cyclic transfers.
		 */
		d->txd.callback = desc->txd.callback;
		d->txd.callback_param = desc->txd.callback_param;

		list_move_tail(&d->node, &edmac->active);
	}
	LEAVE();
}

/* Called with @edmac->lock held and interrupts disabled */
static struct nuc980_dma_desc *
nuc980_dma_get_active(struct nuc980_dma_chan *edmac) {
	DMA_DEBUG("NUC980 GDMA %s\n", __FUNCTION__ );
	if (list_empty(&edmac->active))
		return NULL;
	return list_first_entry(&edmac->active, struct nuc980_dma_desc, node);
}

/**
 * nuc980_dma_advance_active - advances to the next active descriptor
 * @edmac: channel
 *
 * Function advances active descriptor to the next in the @edmac->active and
 * returns %true if we still have descriptors in the chain to process.
 * Otherwise returns %false.
 *
 * When the channel is in cyclic mode always returns %true.
 *
 * Called with @edmac->lock held and interrupts disabled.
 */
static bool nuc980_dma_advance_active(struct nuc980_dma_chan *edmac)
{
	struct nuc980_dma_desc *desc;
	DMA_DEBUG("NUC980 GDMA %s\n", __FUNCTION__ );
	list_rotate_left(&edmac->active);

	if (test_bit(NUC980_DMA_IS_CYCLIC, &edmac->flags))
		return true;

	desc = nuc980_dma_get_active(edmac);
	if (!desc)
		return false;

	/*
	 * If txd.cookie is set it means that we are back in the first
	 * descriptor in the chain and hence done with it.
	 */
	return !desc->txd.cookie;
}

/*
 000 = PDMA channel 1 time-out clock source is HCLK/(2^8).
 001 = PDMA channel 1 time-out clock source is HCLK/(2^9).
 010 = PDMA channel 1 time-out clock source is HCLK/(2^10).
 011 = PDMA channel 1 time-out clock source is HCLK/(2^11).
 100 = PDMA channel 1 time-out clock source is HCLK/(2^12).
 101 = PDMA channel 1 time-out clock source is HCLK/(2^13).
 */
void nuc980_dma_SetTimeOut(struct nuc980_dma_chan *edmac,u32 prescaler,u32 counter)
{

	struct nuc980_dma_desc *desc;
	int ch;
	PDMA_T * pdma;
	ENTRY();
	desc = nuc980_dma_get_active(edmac);
	if (!desc) {
		dev_warn(chan2dev(edmac), "PDMA: empty descriptor list\n");
		return;
	}

	if(edmac->irq==IRQ_PDMA0) {
		mutex_lock(&pdma0_mutex);
		pdma=PDMA0;
	} else {
		mutex_lock(&pdma1_mutex);
		pdma=PDMA1;
	}
	ch =edmac->id ;

	if(prescaler ==0 && counter ==0) {
		pdma->TOUTIEN &=~(1 << ch);
		pdma->TOUTEN &=~(1 << ch);  /* Enable time-out funciton */

		if(edmac->irq==IRQ_PDMA0)
			mutex_unlock(&pdma0_mutex);
		else
			mutex_unlock(&pdma1_mutex);

		return;
	}

	if(ch<=7) {
		pdma->TOUTPSC &= ~(0x7 << (PDMA_TOUTPSC_TOUTPSC1_Pos * ch));
		pdma->TOUTPSC |= ((prescaler&0x7) << (PDMA_TOUTPSC_TOUTPSC1_Pos * ch));
	} else {
		pdma->TOUTPSC2 &= ~(0x7 << (PDMA_TOUTPSC_TOUTPSC1_Pos * (ch-8)));
		pdma->TOUTPSC2 |= ((prescaler&0x7) << (PDMA_TOUTPSC_TOUTPSC1_Pos * (ch-8)));
	}

	if(ch==0 || ch==1) {
		pdma->TOC0_1 &= ~( (0xff) << (PDMA_TOC0_1_TOC1_Pos * ch));
		pdma->TOC0_1 |= ((counter&0xff) << (PDMA_TOC0_1_TOC1_Pos * ch));
	} else if(ch==2 || ch==3) {
		pdma->TOC2_3 &= ~( (0xff) << (PDMA_TOC0_1_TOC1_Pos * (ch-2)));
		pdma->TOC2_3 |= ((counter&0xff)<< (PDMA_TOC0_1_TOC1_Pos * (ch-2)));
	} else if(ch==4 || ch==5) {
		pdma->TOC4_5 &= ~( (0xff) << (PDMA_TOC0_1_TOC1_Pos * (ch-4)));
		pdma->TOC4_5 |= ((counter&0xff) << (PDMA_TOC0_1_TOC1_Pos * (ch-4)));
	} else if(ch==6 || ch==7) {
		pdma->TOC6_7 &= ~( (0xff) << (PDMA_TOC0_1_TOC1_Pos * (ch-6)));
		pdma->TOC6_7 |= ((counter&0xff) << (PDMA_TOC0_1_TOC1_Pos * (ch-6)));
	} else if(ch==8 || ch==9) {
		pdma->TOC8_9 &= ~( (0xff) << (PDMA_TOC0_1_TOC1_Pos * (ch-8)));
		pdma->TOC8_9 |= ((counter&0xff) << (PDMA_TOC0_1_TOC1_Pos * (ch-8)));
	}


	pdma->TOUTEN |= (1 << ch);  /* Enable time-out funciton */
	pdma->TOUTIEN |= (1 << ch); /* Enable time-out interrupt */
	if(edmac->irq==IRQ_PDMA0)
		mutex_unlock(&pdma0_mutex);
	else
		mutex_unlock(&pdma1_mutex);

	LEAVE();
}


/*
 * DMA implementation
 */

static int hw_setup(struct nuc980_dma_chan *edmac)
{
#if 0
	const struct nuc980_dma_data *data = edmac->chan.private;
	//u32 control = 0;
	DMA_DEBUG("NUC980 GDMA %s\n", __FUNCTION__ );
	if (!data) {
		/* This is memcpy channel, nothing to configure */
		return 0;
	}

	switch (data->port) {
	case NUC980_DMA_MEM:
		break;

	default:
		return -EINVAL;
	}
#endif
	//writel(control, edmac->regs + M2M_CONTROL);
	return 0;
}

static void hw_shutdown(struct nuc980_dma_chan *edmac)
{
	ENTRY();
	/* Just disable the channel */

	if(edmac->irq==IRQ_PDMA0) {
		PDMA0->CHCTL &= ~(1<<edmac->id);
	} else {
		PDMA1->CHCTL &= ~(1<<edmac->id);
	}
	LEAVE();
}

static void fill_desc(struct nuc980_dma_chan *edmac)
{
	struct nuc980_dma_desc *desc;
	u32 regT;
	PDMA_T * pdma;
	//u32 tcnt,config;

	ENTRY();
	desc = nuc980_dma_get_active(edmac);
	if (!desc) {
		dev_warn(chan2dev(edmac), "PDMA: empty descriptor list\n");
		return;
	}

	DMA_DEBUG("edmac->runtime_ctrl=0x%08x\n",edmac->runtime_ctrl);
	DMA_DEBUG("desc->ctl=0x%08x\n",desc->ctl);
	if(edmac->irq==IRQ_PDMA0) {
		mutex_lock(&pdma0_mutex);
		DMA_DEBUG("PDMA0[%d] CTL=0x%08x\n",edmac->id,PDMA0->DSCT[edmac->id].CTL);
		if((PDMA0->DSCT[edmac->id].CTL & 0x3)!=0) {
			regT=PDMA0->CHCTL;
			PDMA0->DSCT[edmac->id].CTL=0;
			PDMA0->CHRST = (1<<edmac->id);
			PDMA0->CHCTL = (regT | (1<<edmac->id));
		} else {
			PDMA0->DSCT[edmac->id].CTL=0;
			PDMA0->CHCTL |= (1<<edmac->id);
		}
		PDMA0->INTEN |= (1<<edmac->id);
		nuc980_set_transfer_mode(PDMA0,edmac->id,desc->config.reqsel);
		mutex_unlock(&pdma0_mutex);
		pdma = PDMA0;
	} else {
		mutex_lock(&pdma1_mutex);
		DMA_DEBUG("PDMA1[%d] CTL=0x%08x\n",edmac->id,PDMA1->DSCT[edmac->id].CTL);
		if((PDMA1->DSCT[edmac->id].CTL & 0x3)!=0) {
			regT=PDMA1->CHCTL;
			PDMA1->DSCT[edmac->id].CTL=0;
			PDMA1->CHRST = (1<<edmac->id);
			PDMA1->CHCTL = (regT | (1<<edmac->id));
		} else {
			PDMA1->DSCT[edmac->id].CTL=0;
			PDMA1->CHCTL |= (1<<edmac->id);
		}
		PDMA1->INTEN |= (1<<edmac->id);
		nuc980_set_transfer_mode(PDMA1,edmac->id,desc->config.reqsel);
		mutex_unlock(&pdma1_mutex);
		pdma = PDMA1;
	}
	pdma->DSCT[edmac->id].CTL |= (edmac->runtime_ctrl | ((desc->size - 1UL) << PDMA_DSCT_CTL_TXCNT_Pos));
	pdma->DSCT[edmac->id].SA   = desc->src_addr;
	pdma->DSCT[edmac->id].DA   = desc->dst_addr;

	DMA_DEBUG2("===============pdma=============\n");
	DMA_DEBUG2("(0x%08x)pdma->DSCT[%d].CTL=0x%08x\n",&pdma->DSCT[edmac->id].CTL,edmac->id,pdma->DSCT[edmac->id].CTL);
	DMA_DEBUG2("(0x%08x)pdma->DSCT[%d].SA=0x%08x\n",&pdma->DSCT[edmac->id].SA,edmac->id,pdma->DSCT[edmac->id].SA);
	DMA_DEBUG2("(0x%08x)pdma->DSCT[%d].DA=0x%08x\n",&pdma->DSCT[edmac->id].DA,edmac->id,pdma->DSCT[edmac->id].DA);
	DMA_DEBUG2("(0x%08x)pdma->CHCTL=0x%08x\n",&pdma->CHCTL,pdma->CHCTL);
	DMA_DEBUG2("(0x%08x)pdma->INTEN=0x%08x\n",&pdma->INTEN,pdma->INTEN);
	DMA_DEBUG2("(0x%08x)pdma->INTSTS=0x%08x\n",&pdma->INTSTS,pdma->INTSTS);
	DMA_DEBUG2("(0x%08x)pdma->TDSTS=0x%08x\n",&pdma->TDSTS,pdma->TDSTS);
	DMA_DEBUG2("(0x%08x)pdma->REQSEL0_3=0x%08x\n",&pdma->REQSEL0_3,pdma->REQSEL0_3);
	DMA_DEBUG2("===============================\n");
	LEAVE();
}

static void fill_desc_sc(struct nuc980_dma_chan *edmac)
{
	struct nuc980_dma_desc *desc=NULL;
	//DSCT_T *dsct=NULL;
	u32 regT;
	ENTRY();
	desc = nuc980_dma_get_active(edmac);

	if(edmac->irq==IRQ_PDMA0) {
		mutex_lock(&pdma0_mutex);
		DMA_DEBUG("SC PDMA0[%d] CTL=0x%08x\n",edmac->id,PDMA0->DSCT[edmac->id].CTL);
		if((PDMA0->DSCT[edmac->id].CTL & 0x3)!=0) {
			regT=PDMA0->CHCTL;
			PDMA0->DSCT[edmac->id].CTL=0;
			PDMA0->CHRST = (1<<edmac->id);
			PDMA0->CHCTL = (regT | (1<<edmac->id));
		} else {
			PDMA0->DSCT[edmac->id].CTL=0;
			PDMA0->CHCTL |= (1<<edmac->id);
		}
		nuc980_set_transfer_mode(PDMA0,edmac->id,desc->config.reqsel);
		PDMA0->INTEN |= (1<<edmac->id);
		mutex_unlock(&pdma0_mutex);
	} else {
		mutex_lock(&pdma1_mutex);
		DMA_DEBUG("SC PDMA1[%d] CTL=0x%08x\n",edmac->id,PDMA1->DSCT[edmac->id].CTL);
		if((PDMA1->DSCT[edmac->id].CTL & 0x3)!=0) {
			regT=PDMA1->CHCTL;
			PDMA1->DSCT[edmac->id].CTL=0;
			PDMA1->CHRST = (1<<edmac->id);
			PDMA1->CHCTL = (regT | (1<<edmac->id));
		} else {
			PDMA1->DSCT[edmac->id].CTL=0;
			PDMA1->CHCTL |= (1<<edmac->id);
		}
		nuc980_set_transfer_mode(PDMA1,edmac->id,desc->config.reqsel);
		PDMA1->INTEN |= (1<<edmac->id);
		mutex_unlock(&pdma1_mutex);
	}

	if(desc->dir ==DMA_DEV_TO_MEM) {
		desc->dsct[0].CTL =   (edmac->runtime_ctrl | (((desc->size/2) - 1UL) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_OP_SCATTER);
		desc->dsct[0].SA =  desc->src_addr;
		desc->dsct[0].DA =  desc->dst_addr;
		desc->dsct[0].NEXT =virt_to_phys(&desc->dsct[1].CTL);

		desc->dsct[1].CTL =  (edmac->runtime_ctrl | (((desc->size/2) - 1UL) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_OP_SCATTER);
		desc->dsct[1].SA =  desc->src_addr;
		desc->dsct[1].DA =  desc->dst_addr+(desc->size/2);
		desc->dsct[1].NEXT = virt_to_phys(&desc->dsct[0].CTL);
	} else if(desc->dir ==DMA_MEM_TO_DEV) {
		desc->dsct[0].CTL =   (edmac->runtime_ctrl | (((desc->size/2) - 1UL) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_OP_SCATTER);
		desc->dsct[0].SA =  desc->src_addr;
		desc->dsct[0].DA =  desc->dst_addr;
		desc->dsct[0].NEXT =virt_to_phys(&desc->dsct[1].CTL);

		desc->dsct[1].CTL =  (edmac->runtime_ctrl | (((desc->size/2) - 1UL) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_OP_SCATTER);
		desc->dsct[1].SA =  desc->src_addr+(desc->size/2);
		desc->dsct[1].DA =  desc->dst_addr;
		desc->dsct[1].NEXT = virt_to_phys(&desc->dsct[0].CTL);
	}


	//DMA_DEBUG2("===============pdma=============\n");
	//DMA_DEBUG2("(0x%08x)desc->dsct[0].CTL=0x%08x\n",&desc->dsct[0].CTL,desc->dsct[0].CTL);
	//DMA_DEBUG2("(0x%08x)desc->dsct[0].NEXT=0x%08x\n",&desc->dsct[0].NEXT,desc->dsct[0].NEXT);
	//DMA_DEBUG2("(0x%08x)desc->dsct[1].CTL=0x%08x\n",&desc->dsct[1].CTL,desc->dsct[1].CTL);
	//DMA_DEBUG2("(0x%08x)desc->dsct[1].NEXT=0x%08x\n",&desc->dsct[1].NEXT,desc->dsct[1].NEXT);
	//DMA_DEBUG2("===============================\n");

	LEAVE();

}


static void hw_submit(struct nuc980_dma_chan *edmac)
{
	struct nuc980_dma_desc *desc;
	ENTRY();
	desc = nuc980_dma_get_active(edmac);
	/*
	 * Since we allow clients to configure PW (peripheral width) we always
	 * clear PW bits here and then set them according what is given in
	 * the runtime configuration.
	 */
	if(desc->config.en_sc==0) {
		fill_desc(edmac);
		nuc980_dma_SetTimeOut(edmac,desc->config.timeout_prescaler,desc->config.timeout_counter);
		if(edmac->irq==IRQ_PDMA0) {
			PDMA0->DSCT[edmac->id].CTL = (PDMA0->DSCT[edmac->id].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;
			if(desc->config.reqsel==0) {
				PDMA0->SWREQ = 1<<(edmac->id);
			}
		} else {
			PDMA1->DSCT[edmac->id].CTL = (PDMA1->DSCT[edmac->id].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;
			if(desc->config.reqsel==0) {
				PDMA1->SWREQ = 1<<(edmac->id);
			}
		}
	} else {
		fill_desc_sc(edmac);
		nuc980_dma_SetTimeOut(edmac,desc->config.timeout_prescaler,desc->config.timeout_counter);
		if(edmac->irq==IRQ_PDMA0) {
			PDMA0->DSCT[edmac->id].NEXT = virt_to_phys(&desc->dsct[0].CTL);
			PDMA0->DSCT[edmac->id].CTL = PDMA_OP_SCATTER;
			if(desc->config.reqsel==0) {
				PDMA0->SWREQ = 1<<(edmac->id);
			}
		} else {
			PDMA1->DSCT[edmac->id].NEXT =  virt_to_phys(&desc->dsct[0].CTL);
			PDMA1->DSCT[edmac->id].CTL = PDMA_OP_SCATTER;
			if(desc->config.reqsel==0) {
				PDMA1->SWREQ = 1<<(edmac->id);
			}
		}

		{
			PDMA_T * pdma;
			if(edmac->irq==IRQ_PDMA0)
				pdma=PDMA0;
			else
				pdma=PDMA1;

			DMA_DEBUG2("===============pdma=============\n");
			DMA_DEBUG2("(0x%08x)pdma->DSCT[%d].CTL=0x%08x\n",&pdma->DSCT[edmac->id].CTL,edmac->id,pdma->DSCT[edmac->id].CTL);
			DMA_DEBUG2("(0x%08x)pdma->DSCT[%d].SA=0x%08x\n",&pdma->DSCT[edmac->id].SA,edmac->id,pdma->DSCT[edmac->id].SA);
			DMA_DEBUG2("(0x%08x)pdma->DSCT[%d].DA=0x%08x\n",&pdma->DSCT[edmac->id].DA,edmac->id,pdma->DSCT[edmac->id].DA);
			DMA_DEBUG2("(0x%08x)pdma->CHCTL=0x%08x\n",&pdma->CHCTL,pdma->CHCTL);
			DMA_DEBUG2("(0x%08x)pdma->INTEN=0x%08x\n",&pdma->INTEN,pdma->INTEN);
			DMA_DEBUG2("(0x%08x)pdma->INTSTS=0x%08x\n",&pdma->INTSTS,pdma->INTSTS);
			DMA_DEBUG2("(0x%08x)pdma->TDSTS=0x%08x\n",&pdma->TDSTS,pdma->TDSTS);
			DMA_DEBUG2("(0x%08x)pdma->REQSEL0_3=0x%08x\n",&pdma->REQSEL0_3,pdma->REQSEL0_3);
			DMA_DEBUG2("===============================\n");
		}
	}

	LEAVE();
}

/*
 * According to NUC980 User's Guide, we should receive DONE interrupt when all
 * M2M DMA controller transactions complete normally. This is not always the
 * case - sometimes NUC980 M2M DMA asserts DONE interrupt when the DMA channel
 * is still running (channel Buffer FSM in DMA_BUF_ON state, and channel
 * Control FSM in DMA_MEM_RD state, observed at least in IDE-DMA operation).
 * In effect, disabling the channel when only DONE bit is set could stop
 * currently running DMA transfer. To avoid this, we use Buffer FSM and
 * Control FSM to check current state of DMA channel.
 */
static int hw_interrupt(struct nuc980_dma_chan *edmac)
{
	bool last_done;
	struct nuc980_dma_desc *desc;
	ENTRY();
	if(edmac->irq==IRQ_PDMA0) {
		DMA_DEBUG("PDMA0->TDSTS=0x%08x,edmac->id=%d\n",PDMA0->TDSTS,edmac->id);
		PDMA0->TDSTS = (1<<(edmac->id));
	} else {
		DMA_DEBUG("PDMA1->TDSTS=0x%08x,edmac->id=%d\n",PDMA1->TDSTS,edmac->id);
		PDMA1->TDSTS = (1<<(edmac->id));
	}

	/*
	 * Check whether we are done with descriptors or not. This, together
	 * with DMA channel state, determines action to take in interrupt.
	 */
	desc = nuc980_dma_get_active(edmac);
	last_done = !desc; //|| desc->txd.cookie;
	DMA_DEBUG2("last_done=%d,desc=0x%08x,desc->txd.cookie=%d\n",last_done,(unsigned int)desc,desc->txd.cookie);
	if(!last_done) {
		if (nuc980_dma_advance_active(edmac)) {
			DMA_DEBUG2("nuc980_dma_advance_active(edmac)!=NULL\n");
			fill_desc(edmac);
			if(edmac->irq==IRQ_PDMA0) {
				PDMA0->DSCT[edmac->id].CTL = (PDMA0->DSCT[edmac->id].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;
				if(desc->config.reqsel==0) {
					PDMA0->SWREQ = 1<<(edmac->id);
				}
			} else {
				PDMA1->DSCT[edmac->id].CTL = (PDMA1->DSCT[edmac->id].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;
				if(desc->config.reqsel==0) {
					PDMA1->SWREQ = 1<<(edmac->id);
				}
			}
			return INTERRUPT_NEXT_BUFFER;
		} else {
			DMA_DEBUG2("nuc980_dma_advance_active(edmac)=NULL\n");
			last_done = true;
		}
	}

	if(last_done) {
		return INTERRUPT_DONE;
	}
	LEAVE();
	return INTERRUPT_NEXT_BUFFER;
}

/*
 * DMA engine API implementation
 */

static struct nuc980_dma_desc *
nuc980_dma_desc_get(struct nuc980_dma_chan *edmac) {
	struct nuc980_dma_desc *desc, *_desc;
	struct nuc980_dma_desc *ret = NULL;
	ENTRY();
	spin_lock(&edmac->lock);
	list_for_each_entry_safe(desc, _desc, &edmac->free_list, node) {
		if (async_tx_test_ack(&desc->txd)) {
			list_del_init(&desc->node);

			/* Re-initialize the descriptor */
			desc->src_addr = 0;
			desc->dst_addr = 0;
			desc->size = 0;
			desc->complete = false;
			desc->txd.cookie = 0;
			desc->txd.callback = NULL;
			desc->txd.callback_param = NULL;

			ret = desc;
			break;
		}
	}
	spin_unlock(&edmac->lock);
	LEAVE();
	return ret;
}

static void nuc980_dma_desc_put(struct nuc980_dma_chan *edmac,
                                struct nuc980_dma_desc *desc)
{
	//ENTRY();
	if (desc) {
		spin_lock(&edmac->lock);
		list_splice_init(&desc->tx_list, &edmac->free_list);
		list_add(&desc->node, &edmac->free_list);
		spin_unlock(&edmac->lock);
	}
}

/**
 * nuc980_dma_advance_work - start processing the next pending transaction
 * @edmac: channel
 *
 * If we have pending transactions queued and we are currently idling, this
 * function takes the next queued transaction from the @edmac->queue and
 * pushes it to the hardware for execution.
 */
static void nuc980_dma_advance_work(struct nuc980_dma_chan *edmac)
{
	struct nuc980_dma_desc *new;
	ENTRY();
	spin_lock(&edmac->lock);
	if (!list_empty(&edmac->active) || list_empty(&edmac->queue)) {
		spin_unlock(&edmac->lock);
		DMA_DEBUG2("nuc980_dma_advance_work  %d %d\n",list_empty(&edmac->active),list_empty(&edmac->queue));
		return;
	}

	/* Take the next descriptor from the pending queue */
	new = list_first_entry(&edmac->queue, struct nuc980_dma_desc, node);
	list_del_init(&new->node);

	nuc980_dma_set_active(edmac, new);
	DMA_DEBUG2("hw_submit(edmac)\n");
	/* Push it to the hardware */
	edmac->edma->hw_submit(edmac);
	spin_unlock(&edmac->lock);
	LEAVE();
}

#if 0
static void nuc980_dma_unmap_buffers(struct nuc980_dma_desc *desc)
{
	struct device *dev = desc->txd.chan->device->dev;
	ENTRY();
	if (!(desc->txd.flags & DMA_COMPL_SKIP_SRC_UNMAP)) {
		if (desc->txd.flags & DMA_COMPL_SRC_UNMAP_SINGLE)
			dma_unmap_single(dev, desc->src_addr, desc->size,
			                 DMA_TO_DEVICE);
		else
			dma_unmap_page(dev, desc->src_addr, desc->size,
			               DMA_TO_DEVICE);
	}
	if (!(desc->txd.flags & DMA_COMPL_SKIP_DEST_UNMAP)) {
		if (desc->txd.flags & DMA_COMPL_DEST_UNMAP_SINGLE)
			dma_unmap_single(dev, desc->dst_addr, desc->size,
			                 DMA_FROM_DEVICE);
		else
			dma_unmap_page(dev, desc->dst_addr, desc->size,
			               DMA_FROM_DEVICE);
	}
	LEAVE();
}
#endif

static void nuc980_dma_sc_tasklet(unsigned long data)
{
	struct nuc980_dma_chan *edmac = (struct nuc980_dma_chan *)data;
	struct nuc980_dma_desc *desc;
	struct nuc980_dma_done * done=NULL;
	dma_async_tx_callback callback = NULL;
	void *callback_param = NULL;
	ENTRY();
	//spin_lock_irq(&edmac->lock);
	desc = nuc980_dma_get_active(edmac);
	DMA_DEBUG2("*desc=0x%08x\n",*desc);

	desc->complete = true;
	done =(struct nuc980_dma_done *)desc->txd.callback_param;
	if(done!=NULL) {
		done->ch = edmac->id;
		if(desc->config.en_sc==1) {
			if(done->base_addr!=1)
				done->base_addr = 1;
			else
				done->base_addr = 2;
		}
		DMA_DEBUG("(0x%08x)pdma->DSCT[%d].CTL=0x%08x\n",&pdma->DSCT[edmac->id].CTL,edmac->id,pdma->DSCT[edmac->id].CTL);
		DMA_DEBUG("(0x%08x)pdma->DSCT[%d].SA=0x%08x\n",&pdma->DSCT[edmac->id].SA,edmac->id,pdma->DSCT[edmac->id].SA);
		DMA_DEBUG("(0x%08x)pdma->DSCT[%d].DA=0x%08x\n",&pdma->DSCT[edmac->id].DA,edmac->id,pdma->DSCT[edmac->id].DA);
		DMA_DEBUG("(0x%08x)pdma->DSCT[%d].NEXT=0x%08x\n",&pdma->DSCT[edmac->id].NEXT,edmac->id,pdma->DSCT[edmac->id].NEXT);
	}


	if (desc) {
		callback = desc->txd.callback;
		callback_param = desc->txd.callback_param;
	}
	//spin_unlock_irq(&edmac->lock);
	if (callback) {
		callback(callback_param);
	}
	LEAVE();


}

static void nuc980_dma_tasklet(unsigned long data)
{
	struct nuc980_dma_chan *edmac = (struct nuc980_dma_chan *)data;
	struct nuc980_dma_desc *desc, *d;
	struct nuc980_dma_done * done=NULL;
	dma_async_tx_callback callback = NULL;
	void *callback_param = NULL;
	LIST_HEAD(list);
	ENTRY();
	//spin_lock_irq(&edmac->lock);
	/*
	 * If dma_terminate_all() was called before we get to run, the active
	 * list has become empty. If that happens we aren't supposed to do
	 * anything more than call nuc980_dma_advance_work().
	 */
	desc = nuc980_dma_get_active(edmac);
	desc->complete = true;
	done =(struct nuc980_dma_done *)desc->txd.callback_param;
	if(done!=NULL) {
		done->ch = edmac->id;
		if(desc->config.en_sc==1) {
			if(done->base_addr!=1)
				done->base_addr = 1;
			else
				done->base_addr = 2;
		}
		DMA_DEBUG("(0x%08x)pdma->DSCT[%d].CTL=0x%08x\n",&pdma->DSCT[edmac->id].CTL,edmac->id,pdma->DSCT[edmac->id].CTL);
		DMA_DEBUG("(0x%08x)pdma->DSCT[%d].SA=0x%08x\n",&pdma->DSCT[edmac->id].SA,edmac->id,pdma->DSCT[edmac->id].SA);
		DMA_DEBUG("(0x%08x)pdma->DSCT[%d].DA=0x%08x\n",&pdma->DSCT[edmac->id].DA,edmac->id,pdma->DSCT[edmac->id].DA);
		DMA_DEBUG("(0x%08x)pdma->DSCT[%d].NEXT=0x%08x\n",&pdma->DSCT[edmac->id].NEXT,edmac->id,pdma->DSCT[edmac->id].NEXT);
	}

	DMA_DEBUG2("*desc=0x%08x\n",*desc);
	if (desc) {
		DMA_DEBUG2("desc->complete=%d\n",desc->complete);
		if (desc->complete) {
			/* mark descriptor complete for non cyclic case only */
			if (!test_bit(NUC980_DMA_IS_CYCLIC, &edmac->flags))
				dma_cookie_complete(&desc->txd);
			DMA_DEBUG2("nuc980_dma_tasklet : ====>list_splice_init\n");
			list_splice_init(&edmac->active, &list);
		}
		callback = desc->txd.callback;
		callback_param = desc->txd.callback_param;
	}

	//spin_unlock_irq(&edmac->lock);

	/* Pick up the next descriptor from the queue */
	nuc980_dma_advance_work(edmac);

	/* Now we can release all the chained descriptors */
	list_for_each_entry_safe(desc, d, &list, node) {
		/*
		 * For the memcpy channels the API requires us to unmap the
		 * buffers unless requested otherwise.
		 */
		desc->txd.flags = DMA_CTRL_ACK;
		nuc980_dma_desc_put(edmac, desc);
	}
	if (callback) {
		callback(callback_param);
	}
	LEAVE();
}

void nuc980_dma_emac_interrupt(struct nuc980_dma_chan *edmac,int status)
{
	struct nuc980_dma_desc *desc=NULL;
	struct nuc980_dma_done * done=NULL;
	ENTRY();
	//spin_lock(&edmac->lock);
	desc = nuc980_dma_get_active(edmac);
	if (!desc) {
		dev_warn(chan2dev(edmac),
		         "got interrupt while active list is empty\n");
		//spin_unlock(&edmac->lock);
		LEAVE();
		return;
	}
	if(status==INTERRUPT_TIMEOUT) {
		done =(struct nuc980_dma_done *)desc->txd.callback_param;
		if(done!=NULL) {
			done->done = 0;
			done->timeout=1;
			if(edmac->irq==IRQ_PDMA0)
				done->remain = (PDMA0->DSCT[edmac->id].CTL & PDMA_DSCT_CTL_TXCNT_Msk)>>PDMA_DSCT_CTL_TXCNT_Pos;
			else
				done->remain = (PDMA1->DSCT[edmac->id].CTL & PDMA_DSCT_CTL_TXCNT_Msk)>>PDMA_DSCT_CTL_TXCNT_Pos;
		}
		tasklet_schedule(&edmac->tasklet);
		//spin_unlock(&edmac->lock);
		return;
	}


	switch (edmac->edma->hw_interrupt(edmac)) {
	case INTERRUPT_DONE: {
		DMA_DEBUG2("INTERRUPT_DONE\n");
		done =(struct nuc980_dma_done *)desc->txd.callback_param;
		if(done!=NULL) {
			done->done = 1;
			done->timeout=0;
			done->remain = 0;
		}
		if(desc->config.en_sc==0) {
			tasklet_schedule(&edmac->tasklet);
		} else {
			tasklet_schedule(&edmac->tasklet_sc);
		}
	}
	break;

	case INTERRUPT_NEXT_BUFFER:
		DMA_DEBUG2("INTERRUPT_NEXT_BUFFER\n");
		if (test_bit(NUC980_DMA_IS_CYCLIC, &edmac->flags))
			tasklet_schedule(&edmac->tasklet);
		break;

	default:
		dev_warn(chan2dev(edmac), "unknown interrupt!\n");
		break;
	}
	//spin_unlock(&edmac->lock);
	LEAVE();
}
static irqreturn_t nuc980_dma_interrupt(int irq, void *dev_id)
{
	int i;
	struct nuc980_dma_engine *edma= dev_id;
	unsigned int pdma0_int_status = PDMA0->INTSTS;
	unsigned int pdma1_int_status = PDMA1->INTSTS;
	unsigned int pdma0_status = PDMA0->TDSTS;
	unsigned int pdma1_status = PDMA1->TDSTS;

	irqreturn_t ret = IRQ_HANDLED;
	ENTRY();
	DMA_DEBUG2("irqreturn_t\n");
	DMA_DEBUG2("PDMA0->INTSTS=0x%08x,PDMA1->INTSTS=0x%08x\n",PDMA0->INTSTS,PDMA1->INTSTS);
	DMA_DEBUG2("PDMA0->TDSTS=0x%08x,PDMA1->TDSTS=0x%08x\n",PDMA0->TDSTS,PDMA1->TDSTS);

	for(i=(edma->num_channels-1); i>=0; i--) {
		if((edma->channels[i].irq==IRQ_PDMA0) && (irq==IRQ_PDMA0)) {
			if(pdma0_status & (1<<(edma->channels[i].id))) {
				PDMA0->TDSTS = (1<<(edma->channels[i].id));
				nuc980_dma_emac_interrupt(&edma->channels[i],INTERRUPT_DONE);
				//break;
			}

			if(pdma0_int_status & (1<<(edma->channels[i].id+8))) {
				DMA_DEBUG2("PDMA0 INTERRUPT_TIMEOUT id=%d",edma->channels[i].id);
				nuc980_dma_emac_interrupt(&edma->channels[i],INTERRUPT_TIMEOUT);
				PDMA0->TOUTEN &= ~(1<<(edma->channels[i].id));
				//PDMA0->TOUTIEN  &= ~(1<<(edma->channels[i].id));
				PDMA0->INTSTS = (1<<(edma->channels[i].id+8));
				//break;
			}

		} else if((edma->channels[i].irq==IRQ_PDMA1) && (irq==IRQ_PDMA1)) {
			if(pdma1_status & (1<<(edma->channels[i].id))) {
				PDMA1->TDSTS = (1<<(edma->channels[i].id));
				nuc980_dma_emac_interrupt(&edma->channels[i],INTERRUPT_DONE);
				//break;
			}
			if(pdma1_int_status & (1<<(edma->channels[i].id+8))) {
				DMA_DEBUG2("PDMA1 INTERRUPT_TIMEOUT id=%d",edma->channels[i].id);
				nuc980_dma_emac_interrupt(&edma->channels[i],INTERRUPT_TIMEOUT);
				PDMA1->TOUTEN &= ~(1<<(edma->channels[i].id));
				//PDMA1->TOUTIEN  &= ~(1<<(edma->channels[i].id));
				PDMA1->INTSTS = (1<<(edma->channels[i].id+8));
				//break;
			}

		}
	}
	LEAVE();
	return ret;
}

/**
 * nuc980_dma_tx_submit - set the prepared descriptor(s) to be executed
 * @tx: descriptor to be executed
 *
 * Function will execute given descriptor on the hardware or if the hardware
 * is busy, queue the descriptor to be executed later on. Returns cookie which
 * can be used to poll the status of the descriptor.
 */
static dma_cookie_t nuc980_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct nuc980_dma_chan *edmac = to_nuc980_dma_chan(tx->chan);
	struct nuc980_dma_desc *desc;
	dma_cookie_t cookie;
	ENTRY();
	//spin_lock_irqsave(&edmac->lock, flags);
	cookie = dma_cookie_assign(tx);

	desc = container_of(tx, struct nuc980_dma_desc, txd);

	/*
	 * If nothing is currently prosessed, we push this descriptor
	 * directly to the hardware. Otherwise we put the descriptor
	 * to the pending queue.
	 */
	if (list_empty(&edmac->active)) {
		nuc980_dma_set_active(edmac, desc);
		edmac->edma->hw_submit(edmac);
	} else {
		list_add_tail(&desc->node, &edmac->queue);
	}
	//spin_unlock_irqrestore(&edmac->lock, flags);

#if 0
	if(edmac->irq==IRQ_PDMA0) {
		DMA_DEBUG("(0x%08x)PDMA0->DSCT[%d].CTL=0x%08x\n",&PDMA0->DSCT[edmac->id].CTL,edmac->id,PDMA0->DSCT[edmac->id].CTL);
		DMA_DEBUG("(0x%08x)PDMA0->DSCT[%d].SA=0x%08x\n",&PDMA0->DSCT[edmac->id].SA,edmac->id,PDMA0->DSCT[edmac->id].SA);
		DMA_DEBUG("(0x%08x)PDMA0->DSCT[%d].DA=0x%08x\n",&PDMA0->DSCT[edmac->id].DA,edmac->id,PDMA0->DSCT[edmac->id].DA);
		DMA_DEBUG("(0x%08x)PDMA0->CHCTL=0x%08x\n",&PDMA0->CHCTL,PDMA0->CHCTL);
		DMA_DEBUG("(0x%08x)PDMA0->INTEN=0x%08x\n",&PDMA0->INTEN,PDMA0->INTEN);
		DMA_DEBUG("(0x%08x)PDMA0->INTSTS=0x%08x\n",&PDMA0->INTSTS,PDMA0->INTSTS);
		DMA_DEBUG("(0x%08x)PDMA0->TDSTS=0x%08x\n",&PDMA0->TDSTS,PDMA0->TDSTS);
		DMA_DEBUG("(0x%08x)PDMA0->REQSEL0_3=0x%08x\n",&PDMA0->REQSEL0_3,PDMA0->REQSEL0_3);
	} else {
		DMA_DEBUG("(0x%08x)PDMA1->DSCT[%d].CTL=0x%08x\n",&PDMA1->DSCT[edmac->id].CTL,edmac->id,PDMA1->DSCT[edmac->id].CTL);
		DMA_DEBUG("(0x%08x)PDMA1->DSCT[%d].SA=0x%08x\n",&PDMA1->DSCT[edmac->id].SA,edmac->id,PDMA1->DSCT[edmac->id].SA);
		DMA_DEBUG("(0x%08x)PDMA1->DSCT[%d].DA=0x%08x\n",&PDMA1->DSCT[edmac->id].DA,edmac->id,PDMA1->DSCT[edmac->id].DA);
		DMA_DEBUG("(0x%08x)PDMA1->CHCTL=0x%08x\n",&PDMA1->CHCTL,PDMA1->CHCTL);
		DMA_DEBUG("(0x%08x)PDMA1->INTEN=0x%08x\n",&PDMA1->INTEN,PDMA1->INTEN);
		DMA_DEBUG("(0x%08x)PDMA1->INTSTS=0x%08x\n",&PDMA1->INTSTS,PDMA1->INTSTS);
		DMA_DEBUG("(0x%08x)PDMA1->TDSTS=0x%08x\n",&PDMA1->TDSTS,PDMA1->TDSTS);
		DMA_DEBUG("(0x%08x)PDMA1->REQSEL0_3=0x%08x\n",&PDMA1->REQSEL0_3,PDMA1->REQSEL0_3);
	}
#endif

	LEAVE();
	return cookie;
}

/**
 * nuc980_dma_alloc_chan_resources - allocate resources for the channel
 * @chan: channel to allocate resources
 *
 * Function allocates necessary resources for the given DMA channel and
 * returns number of allocated descriptors for the channel. Negative errno
 * is returned in case of failure.
 */
static int nuc980_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct nuc980_dma_chan *edmac = to_nuc980_dma_chan(chan);
	struct nuc980_dma_data *data = chan->private;
	const char *name = dma_chan_name(chan);
	int ret, i;
	ENTRY();
	DMA_DEBUG("name =%s\n", name);
#if 0
	/* Sanity check the channel parameters */
	if (data) {
		switch (data->port) {
		case NUC980_DMA_MEM:
			if (!is_slave_direction(data->direction))
				return -EINVAL;
			break;
		default:
			return -EINVAL;
		}
	}
#endif

	if (data && data->name)
		name = data->name;

	DMA_DEBUG("edmac->irq =%d\n", edmac->irq);

	spin_lock_irq(&edmac->lock);
	dma_cookie_init(&edmac->chan);
	ret = edmac->edma->hw_setup(edmac);
	spin_unlock_irq(&edmac->lock);

	if (ret)
		return ret;

	for (i = 0; i < DMA_MAX_CHAN_DESCRIPTORS; i++) {
		struct nuc980_dma_desc *desc;

		desc = kzalloc(sizeof(*desc), GFP_KERNEL);
		if (!desc) {
			dev_warn(chan2dev(edmac), "not enough descriptors\n");
			break;
		}
		INIT_LIST_HEAD(&desc->tx_list);
		dma_async_tx_descriptor_init(&desc->txd, chan);
		desc->txd.flags = DMA_CTRL_ACK;
		desc->txd.tx_submit = nuc980_dma_tx_submit;
		nuc980_dma_desc_put(edmac, desc);
	}
	DMA_DEBUG("return %d\n",i);
	LEAVE();
	return i;

}

/**
 * nuc980_dma_free_chan_resources - release resources for the channel
 * @chan: channel
 *
 * Function releases all the resources allocated for the given channel.
 * The channel must be idle when this is called.
 */
static void nuc980_dma_free_chan_resources(struct dma_chan *chan)
{
	struct nuc980_dma_chan *edmac = to_nuc980_dma_chan(chan);
	struct nuc980_dma_desc *desc, *d;
	LIST_HEAD(list);
	ENTRY();
	//BUG_ON(!list_empty(&edmac->active));
	BUG_ON(!list_empty(&edmac->queue));
	spin_lock(&edmac->lock);
	edmac->edma->hw_shutdown(edmac);
	edmac->runtime_addr = 0;
	edmac->runtime_ctrl = 0;
	list_splice_init(&edmac->free_list, &list);
	spin_unlock(&edmac->lock);
	list_for_each_entry_safe(desc, d, &list, node)
	kfree(desc);
	LEAVE();
}

/**
 * nuc980_dma_prep_dma_memcpy - prepare a memcpy DMA operation
 * @chan: channel
 * @dest: destination bus address
 * @src: source bus address
 * @len: size of the transaction
 * @flags: flags for the descriptor
 *
 * Returns a valid DMA descriptor or %NULL in case of failure.
 */
static struct dma_async_tx_descriptor *
nuc980_dma_prep_dma_memcpy(struct dma_chan *chan, dma_addr_t dest,
                           dma_addr_t src, size_t len, unsigned long flags) {
	struct nuc980_dma_chan *edmac = to_nuc980_dma_chan(chan);
	struct nuc980_dma_desc *desc, *first;
	size_t bytes, offset;
	ENTRY();
	first = NULL;
	for (offset = 0; offset < len; offset += bytes) {
		desc = nuc980_dma_desc_get(edmac);
		if (!desc) {
			dev_warn(chan2dev(edmac), "couln't get descriptor\n");
			goto fail;
		}

		bytes = min_t(size_t, len - offset, DMA_MAX_CHAN_BYTES);

		desc->src_addr = src + offset;
		desc->dst_addr = dest + offset;
		desc->size = bytes;
		desc->config.reqsel = 0;
		desc->config.timeout_counter=0;
		desc->config.timeout_prescaler=0;
		desc->config.en_sc = 0;
		desc->dir = DMA_MEM_TO_MEM;
		edmac->runtime_ctrl = 0;
		DMA_DEBUG("src_addr=0x%08x\n",desc->src_addr);
		DMA_DEBUG("dst_addr=0x%08x\n",desc->dst_addr);
		DMA_DEBUG("size=0x%08x\n",desc->size);
		DMA_DEBUG("offset=0x%08x\n",offset);
		if (!first)
			first = desc;
		else
			list_add_tail(&desc->node, &first->tx_list);
	}

	first->txd.cookie = -EBUSY;
	first->txd.flags = flags;
	LEAVE();

	return &first->txd;
fail:
	DMA_DEBUG("%s fail =>\n", __FUNCTION__);
	nuc980_dma_desc_put(edmac, first);
	LEAVE();
	return NULL;
}

/**
 * nuc980_dma_prep_slave_sg - prepare a slave DMA operation
 * @chan: channel
 * @sgl: list of buffers to transfer
 * @sg_len: number of entries in @sgl
 * @dir: direction of tha DMA transfer
 * @flags: flags for the descriptor
 * @context: operation context (ignored)
 *
 * Returns a valid DMA descriptor or %NULL in case of failure.
 */
static struct dma_async_tx_descriptor *
nuc980_dma_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
                         unsigned int sg_len, enum dma_transfer_direction dir,
                         unsigned long flags, void *context) {
	struct nuc980_dma_chan *edmac = to_nuc980_dma_chan(chan);
	struct nuc980_dma_desc *desc, *first;
	struct scatterlist *sg;
	struct nuc980_dma_config *config;
	int i;
	ENTRY();

	{
		struct nuc980_dma_desc *d;
		LIST_HEAD(list);
		list_splice_init(&edmac->active, &list);
		/* Now we can release all the chained descriptors */
		list_for_each_entry_safe(desc, d, &list, node) {
			desc->txd.flags = DMA_CTRL_ACK;
			nuc980_dma_desc_put(edmac, desc);
		}
	}
	config =(struct nuc980_dma_config *)context;



	if (test_bit(NUC980_DMA_IS_CYCLIC, &edmac->flags)) {
		dev_warn(chan2dev(edmac),
		         "channel is already used for cyclic transfers\n");
		return NULL;
	}

	first = NULL;
	for_each_sg(sgl, sg, sg_len, i) {
		if (sg_dma_len(sg) > DMA_MAX_CHAN_BYTES) {
			dev_warn(chan2dev(edmac), "too big transfer size %d\n",
			         sg_dma_len(sg));
			goto fail;
		}

		desc = nuc980_dma_desc_get(edmac);
		if (!desc) {
			dev_warn(chan2dev(edmac), "couln't get descriptor\n");
			goto fail;
		}

		if (dir == DMA_MEM_TO_DEV) {
			desc->src_addr = sg_dma_address(sg);
			desc->dst_addr = edmac->runtime_addr;
			desc->dir = DMA_MEM_TO_DEV;
		} else {
			desc->src_addr = edmac->runtime_addr;
			desc->dst_addr = sg_dma_address(sg);
			desc->dir = DMA_DEV_TO_MEM;
		}
		desc->size = sg_dma_len(sg);
		desc->config.reqsel= config->reqsel;
		desc->config.timeout_counter=config->timeout_counter;
		desc->config.timeout_prescaler=config->timeout_prescaler;
		desc->config.en_sc = config->en_sc;
#if 1
		DMA_DEBUG("desc->src_addr=%x\n",desc->src_addr);
		DMA_DEBUG("desc->dst_addr=%x\n",desc->dst_addr);
		DMA_DEBUG("desc->size=%x\n",desc->size);
		DMA_DEBUG("*context=%x\n",*(u32 *)context);
#endif

		if (!first)
			first = desc;
		else
			list_add_tail(&desc->node, &first->tx_list);
	}

	first->txd.cookie = -EBUSY;
	first->txd.flags = flags;
	LEAVE();
	return &first->txd;

fail:
	DMA_DEBUG("%s fail =>\n", __FUNCTION__);
	nuc980_dma_desc_put(edmac, first);
	LEAVE();
	return NULL;
}

/**
 * nuc980_dma_prep_dma_cyclic - prepare a cyclic DMA operation
 * @chan: channel
 * @dma_addr: DMA mapped address of the buffer
 * @buf_len: length of the buffer (in bytes)
 * @period_len: length of a single period
 * @dir: direction of the operation
 * @flags: tx descriptor status flags
 * @context: operation context (ignored)
 *
 * Prepares a descriptor for cyclic DMA operation. This means that once the
 * descriptor is submitted, we will be submitting in a @period_len sized
 * buffers and calling callback once the period has been elapsed. Transfer
 * terminates only when client calls dmaengine_terminate_all() for this
 * channel.
 *
 * Returns a valid DMA descriptor or %NULL in case of failure.
 */
static struct dma_async_tx_descriptor *
nuc980_dma_prep_dma_cyclic(struct dma_chan *chan, dma_addr_t dma_addr,
                           size_t buf_len, size_t period_len,
                           enum dma_transfer_direction dir, unsigned long flags) {
	struct nuc980_dma_chan *edmac = to_nuc980_dma_chan(chan);
	struct nuc980_dma_desc *desc, *first;
	size_t offset = 0;
	ENTRY();
#if 0
	if (dir != nuc980_dma_chan_direction(chan)) {
		dev_warn(chan2dev(edmac),
		         "channel was configured with different direction\n");
		return NULL;
	}
#endif
	if (test_and_set_bit(NUC980_DMA_IS_CYCLIC, &edmac->flags)) {
		dev_warn(chan2dev(edmac),
		         "channel is already used for cyclic transfers\n");
		return NULL;
	}

	if (period_len > DMA_MAX_CHAN_BYTES) {
		dev_warn(chan2dev(edmac), "too big period length %d\n",
		         period_len);
		return NULL;
	}

	/* Split the buffer into period size chunks */
	first = NULL;
	for (offset = 0; offset < buf_len; offset += period_len) {
		desc = nuc980_dma_desc_get(edmac);
		if (!desc) {
			dev_warn(chan2dev(edmac), "couln't get descriptor\n");
			goto fail;
		}

		desc->size = period_len;

		if (!first)
			first = desc;
		else
			list_add_tail(&desc->node, &first->tx_list);
	}

	first->txd.cookie = -EBUSY;
	LEAVE();
	return &first->txd;

fail:
	nuc980_dma_desc_put(edmac, first);
	LEAVE();
	return NULL;
}

static int nuc980_dma_slave_config(struct dma_chan *chan,
                                   struct dma_slave_config *config)
{
	struct nuc980_dma_chan *edmac = to_nuc980_dma_chan(chan);
	enum dma_slave_buswidth width;
	u32 addr, ctrl;

	ENTRY();

	switch (config->direction) {
	case DMA_DEV_TO_MEM:
		ctrl    = PDMA_DSCT_CTL_SAINC_Msk|PDMA_DSCT_CTL_TXTYPE_Msk;
		width = config->src_addr_width;
		addr    = config->src_addr;
		break;

	case DMA_MEM_TO_DEV:
		ctrl    = PDMA_DSCT_CTL_DAINC_Msk|PDMA_DSCT_CTL_TXTYPE_Msk;
		width = config->dst_addr_width;
		addr    = config->dst_addr;
		break;

	default:
		return -EINVAL;
	}


	switch (width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		ctrl |= 0<<PDMA_DSCT_CTL_TXWIDTH_Pos;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		ctrl |= 1<<PDMA_DSCT_CTL_TXWIDTH_Pos;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		ctrl |= 2<<PDMA_DSCT_CTL_TXWIDTH_Pos;
		break;
	default:
		return -EINVAL;
	}

	//spin_lock_irqsave(&edmac->lock, flags);
	edmac->runtime_addr = addr;
	edmac->runtime_ctrl = ctrl;
	//spin_unlock_irqrestore(&edmac->lock, flags);

	LEAVE();
	return 0;
}

/**
 * nuc980_dma_tx_status - check if a transaction is completed
 * @chan: channel
 * @cookie: transaction specific cookie
 * @state: state of the transaction is stored here if given
 *
 * This function can be used to query state of a given transaction.
 */
static enum dma_status nuc980_dma_tx_status(struct dma_chan *chan,
                dma_cookie_t cookie,
                struct dma_tx_state *state)
{
	struct nuc980_dma_chan *edmac = to_nuc980_dma_chan(chan);
	enum dma_status ret;

	spin_lock(&edmac->lock);
	ret = dma_cookie_status(chan, cookie, state);
	spin_unlock(&edmac->lock);
	return ret;
}

/**
 * nuc980_dma_issue_pending - push pending transactions to the hardware
 * @chan: channel
 *
 * When this function is called, all pending transactions are pushed to the
 * hardware and executed.
 */
static void nuc980_dma_issue_pending(struct dma_chan *chan)
{
	ENTRY();
	nuc980_dma_advance_work(to_nuc980_dma_chan(chan));
	LEAVE();
}


static int nuc980_dma_probe(struct platform_device *pdev)
{
	struct nuc980_dma_platform_data *pdata;
	struct nuc980_dma_engine *edma;
	struct dma_device *dma_dev;
	size_t edma_size;
	struct clk *clk;
	int ret, i;
	ENTRY();
#ifdef CONFIG_USE_OF
	platform_device_add_data(pdev, &nuc980_dma_data,sizeof(nuc980_dma_data));
#endif
	printk("%s - pdev = %s\n", __func__, pdev->name);
	pdata = dev_get_platdata(&pdev->dev);
	edma_size = pdata->num_channels * sizeof(struct nuc980_dma_chan);
	edma = kzalloc(sizeof(*edma) + edma_size, GFP_KERNEL);

	if (!edma) {
		DMA_DEBUG("NUC980 GDMA -ENOMEM\n");
		return -ENOMEM;
	}
	DMA_DEBUG("NUC980 GDMA !!!\n");

	/* enable pdma0/pdma1 clock */
	clk = clk_get(NULL, "pdma0_hclk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return -ENOENT;
	}
	dev_dbg(&pdev->dev, "clock source %p\n", clk);
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "pdma1_hclk");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return -ENOENT;
	}
	dev_dbg(&pdev->dev, "clock source %p\n", clk);
	clk_prepare(clk);
	clk_enable(clk);


	dma_dev = &edma->dma_dev;
	edma->num_channels = pdata->num_channels;

	INIT_LIST_HEAD(&dma_dev->channels);
	for (i = 0; i < pdata->num_channels; i++) {
		const struct nuc980_dma_chan_data *cdata = &pdata->channels[i];
		struct nuc980_dma_chan *edmac = &edma->channels[i];
		DMA_DEBUG("ch=%d\n",i);
		edmac->chan.device = dma_dev;
		edmac->chan.private = 0;
		edmac->regs = cdata->base;
		edmac->irq = cdata->irq;
		edmac->edma = edma;
		edmac->id = (((unsigned int)(edmac->regs)&0xF0)>>4);
		spin_lock_init(&edmac->lock);
		spin_lock_init(&edmac->wklock);
		INIT_LIST_HEAD(&edmac->active);
		INIT_LIST_HEAD(&edmac->queue);
		INIT_LIST_HEAD(&edmac->free_list);
		tasklet_init(&edmac->tasklet, nuc980_dma_tasklet,
		             (unsigned long)edmac);
		tasklet_init(&edmac->tasklet_sc, nuc980_dma_sc_tasklet,
		             (unsigned long)edmac);

		list_add_tail(&edmac->chan.device_node,
		              &dma_dev->channels);
	}

	ret = request_irq(IRQ_PDMA0, nuc980_dma_interrupt, IRQF_SHARED, "PDMA0", edma);
	if (ret) {
		printk("request irq(IRQ_PDMA0) fialed\n");
		return ret;
	}
	ret = request_irq(IRQ_PDMA1, nuc980_dma_interrupt, IRQF_SHARED, "PDMA1", edma);
	if (ret) {
		printk("request irq(IRQ_PDMA1) fialed\n");
		return ret;
	}

	dma_cap_zero(dma_dev->cap_mask);
	dma_cap_set(DMA_SLAVE, dma_dev->cap_mask);
	dma_cap_set(DMA_CYCLIC, dma_dev->cap_mask);

	dma_dev->dev = &pdev->dev;
	dma_dev->device_alloc_chan_resources = nuc980_dma_alloc_chan_resources;
	dma_dev->device_free_chan_resources = nuc980_dma_free_chan_resources;
	dma_dev->device_prep_slave_sg = nuc980_dma_prep_slave_sg;
	dma_dev->device_prep_dma_cyclic = nuc980_dma_prep_dma_cyclic;
	dma_dev->device_config = nuc980_dma_slave_config;
	dma_dev->device_issue_pending = nuc980_dma_issue_pending;
	dma_dev->device_tx_status = nuc980_dma_tx_status;

	dma_set_max_seg_size(dma_dev->dev, DMA_MAX_CHAN_BYTES);

	dma_cap_set(DMA_MEMCPY, dma_dev->cap_mask);
	dma_dev->device_prep_dma_memcpy = nuc980_dma_prep_dma_memcpy;

	edma->hw_setup = hw_setup;
	edma->hw_shutdown = hw_shutdown;
	edma->hw_submit = hw_submit;
	edma->hw_interrupt = hw_interrupt;
	dma_cap_set(DMA_PRIVATE, dma_dev->cap_mask);
	mutex_init(&pdma0_mutex);
	mutex_init(&pdma1_mutex);

	platform_set_drvdata(pdev, edma);

	ret = dma_async_device_register(dma_dev);
	if (unlikely(ret)) {
#if 0
		for (i = 0; i < edma->num_channels; i++) {
			struct nuc980_dma_chan *edmac = &edma->channels[i];
			if (!IS_ERR_OR_NULL(edmac->clk))
				clk_put(edmac->clk);
		}
#endif
		kfree(edma);
	} else {
		dev_info(dma_dev->dev, "NUC980 DMA ready\n");
	}
	LEAVE();

	return ret;
}

static int nuc980_dma_suspend(struct platform_device *pdev,pm_message_t state)
{
	struct nuc980_dma_engine *edma = platform_get_drvdata(pdev);
	struct nuc980_dma_chan *edmac = &edma->channels[0];
	ENTRY();
	LEAVE();
	return 0;
}

static int nuc980_dma_resume(struct platform_device *pdev)
{
	struct nuc980_dma_engine *edma = platform_get_drvdata(pdev);
	struct nuc980_dma_chan *edmac = &edma->channels[0];
	ENTRY();
	LEAVE();
	return 0;
}

static struct platform_device_id nuc980_dma_driver_ids[] = {
	{ "nuc980-dma", 0 },
	{ },
};

static const struct of_device_id nuc980_dma_of_match[] = {
	{ .compatible = "nuvoton,nuc980-dma" },
	{},
};

static struct platform_driver nuc980_dma_driver = {
	.driver     = {
		.name   = "nuc980-dma",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(nuc980_dma_of_match),
	},
	.probe      = nuc980_dma_probe,
	.resume     = nuc980_dma_resume,
	.suspend    = nuc980_dma_suspend,
	.id_table   = nuc980_dma_driver_ids,
};
module_platform_driver(nuc980_dma_driver);


MODULE_DESCRIPTION("NUC980 DMA driver");
MODULE_LICENSE("GPL");
