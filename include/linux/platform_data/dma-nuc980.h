#ifndef __ASM_ARCH_DMA_H
#define __ASM_ARCH_DMA_H

#include <linux/types.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>

/**
 * struct nuc980_dma_data - configuration data for the NUC980 dmaengine
 * @port: peripheral which is requesting the channel
 * @direction: TX/RX channel
 * @name: optional name for the channel, this is displayed in /proc/interrupts
 *
 * This information is passed as private channel parameter in a filter
 * function. Note that this is only needed for slave/cyclic channels.  For
 * memcpy channels %NULL data should be passed.
 */
struct nuc980_dma_data {
	int             port;
	enum dma_transfer_direction direction;
	const char          *name;
};

struct nuc980_dma_done {
	int 		    ch;
	u32            base_addr;
	bool            done;
	int             timeout;
	int             remain;
	void            *callback_param;
};

struct nuc980_ip_rx_dma {
	struct dma_chan         *chan_rx;
	struct scatterlist      sgrx[2];
	struct dma_async_tx_descriptor  *rxdesc;
	struct dma_slave_config slave_config;
};

struct nuc980_ip_tx_dma {
	struct dma_chan         *chan_tx;
	struct scatterlist      sgtx[2];
	struct dma_async_tx_descriptor  *txdesc;
	struct dma_slave_config slave_config;
};

struct nuc980_mem_alloc {
	int     size;
	unsigned int    vir_addr;
	unsigned int    phy_addr;
};

struct nuc980_mem_dma_param {
	int     size;
	unsigned int    src_addr;
	unsigned int    dst_addr;
	int     cfg;
};

struct nuc980_dma_config {
	u32 timeout_prescaler;
	u32 timeout_counter;
	u32 en_sc;
	u32 reqsel;
};

/**
 * struct nuc980_dma_chan_data - platform specific data for a DMA channel
 * @name: name of the channel, used for getting the right clock for the channel
 * @base: mapped registers
 * @irq: interrupt number used by this channel
 */
struct nuc980_dma_chan_data {
	const char          *name;
	void __iomem            *base;
	int             irq;
};

/**
 * struct nuc980_dma_platform_data - platform data for the dmaengine driver
 * @channels: array of channels which are passed to the driver
 * @num_channels: number of channels in the array
 *
 * This structure is passed to the DMA engine driver via platform data. For
 * M2P channels, contract is that even channels are for TX and odd for RX.
 * There is no requirement for the M2M channels.
 */
struct nuc980_dma_platform_data {
	struct nuc980_dma_chan_data *channels;
	size_t              num_channels;
};

#endif /* __ASM_ARCH_DMA_H */
