/*
 *  linux/drivers/mmc/host/nuc980_sd.c - Nuvoton NUC980 SD Driver
 *
 * Copyright (c) 2018 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 */


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/atmel_pdc.h>
#include <linux/gfp.h>
#include <linux/freezer.h>
#include <linux/of.h>

#include <linux/mmc/host.h>

#include <asm/io.h>
#include <asm/irq.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gcr.h>
#include <mach/regs-sdh.h>

#if 0
#define ENTRY()                                 printk("[%-20s] : Enter...\n", __FUNCTION__)
#define LEAVE()                                 printk("[%-20s] : Leave...\n", __FUNCTION__)
#else
#define ENTRY()
#define LEAVE()
#endif

//#define nuc980_sd_debug       printk
#define nuc980_sd_debug(...)

#define DRIVER_NAME    "nuc980-sdh"

#define FL_SENT_COMMAND (1 << 0)
#define FL_SENT_STOP    (1 << 1)

#define nuc980_sd_read(reg)          __raw_readl(reg)
#define nuc980_sd_write(reg, val)    __raw_writel((val), (reg))

#define MCI_BLKSIZE         512
#define MCI_MAXBLKSIZE      4095
#define MCI_BLKATONCE       255
#define MCI_BUFSIZE         (MCI_BLKSIZE * MCI_BLKATONCE)

/* Driver thread command */
#define SD_EVENT_NONE       0x00000000
#define SD_EVENT_CMD_OUT    0x00000001
#define SD_EVENT_RSP_IN     0x00000010
#define SD_EVENT_RSP2_IN    0x00000100
#define SD_EVENT_CLK_KEEP0  0x00001000
#define SD_EVENT_CLK_KEEP1  0x00010000

/*
 * Low level type for this driver
 */
struct nuc980_sd_host {
	struct mmc_host *mmc;
	struct mmc_command *cmd;
	struct mmc_request *request;

	void __iomem *sd_base;
	int irq;

	int present;
	struct clk *sd_clk, *upll_clk,*xin_clk,*div_clk,*mux_clk;
	/*
	 * Flag indicating when the command has been sent. This is used to
	 * work out whether or not to send the stop
	 */
	unsigned int flags;
	/* flag for current port */
	u32 bus_mode;
	u32 port;       // SD port 0 / 1

	/* DMA buffer used for transmitting */
	unsigned int* buffer;
	dma_addr_t physical_address;
	unsigned int total_length;

	/* Latest in the scatterlist that has been enabled for transfer, but not freed */
	int in_use_index;

	/* Latest in the scatterlist that has been enabled for transfer */
	int transfer_index;

	/* chekc nuc980 chip version */
	u32 IsVersionC;


	/* Timer for timeouts */
	struct timer_list timer;
};

/* SD Port 0 */
static volatile int sd_event=0, sd_state_xfer=0, sd_ri_timeout=0;
static DECLARE_WAIT_QUEUE_HEAD(sd_wq_xfer);
static int nuc980_sd_event_thread(struct nuc980_sd_host *sd_host);

//--- Define semaphore for SDH controller
struct semaphore sdh_fmi_sem;

#if 0
static void dump_sdh_regs()
{
	printk("    REG_CLK_HCLKEN = 0x%x\n", nuc980_sd_read(REG_CLK_HCLKEN));
	printk("    REG_CLKDIV9 = 0x%x\n", nuc980_sd_read(REG_CLK_DIV9));
	printk("    REG_FMICSR  = 0x%x\n", nuc980_sd_read(REG_FMICSR));
	printk("    REG_DMACCSR = 0x%x\n", nuc980_sd_read(REG_DMACCSR));
	printk("    REG_SDCSR   = 0x%x\n", nuc980_sd_read(REG_SDCSR));
	printk("    REG_SDARG   = 0x%x\n", nuc980_sd_read(REG_SDARG));
	printk("    REG_SDIER   = 0x%x\n", nuc980_sd_read(REG_SDIER));
	printk("    REG_SDISR   = 0x%x\n", nuc980_sd_read(REG_SDISR));
	printk("    REG_SDRSP0  = 0x%x\n", nuc980_sd_read(REG_SDRSP0));
	printk("    REG_SDRSP1  = 0x%x\n", nuc980_sd_read(REG_SDRSP1));
	printk("    REG_SDBLEN  = 0x%x\n", nuc980_sd_read(REG_SDBLEN));
	printk("    REG_SDTMOUT = 0x%x\n", nuc980_sd_read(REG_SDTMOUT));
	printk("    REG_SDECR   = 0x%x\n", nuc980_sd_read(REG_SDECR));
}
#endif

/*
 * Reset the controller and restore most of the state
 */
static void nuc980_sd_reset_host(struct nuc980_sd_host *host)
{
	unsigned long flags;
	ENTRY();
	local_irq_save(flags);

	nuc980_sd_write(REG_DMACCSR, DMACCSR_DMACEN | DMACCSR_SW_RST); //enable DMAC for FMI
	nuc980_sd_write(REG_FMICSR, FMICSR_SW_RST); /* Enable SD functionality of FMI */
	nuc980_sd_write(REG_FMICSR, FMICSR_SD_EN);
	local_irq_restore(flags);
	LEAVE();
}

static void nuc980_sd_timeout_timer(unsigned long data)
{
	struct nuc980_sd_host *host;

	host = (struct nuc980_sd_host *)data;
	ENTRY();
	if (host->request) {
		dev_err(host->mmc->parent, "Timeout waiting end of packet\n");

		if (host->cmd && host->cmd->data) {
			host->cmd->data->error = -ETIMEDOUT;
		} else {
			if (host->cmd)
				host->cmd->error = -ETIMEDOUT;
			else
				host->request->cmd->error = -ETIMEDOUT;
		}
		nuc980_sd_reset_host(host);
		mmc_request_done(host->mmc, host->request);
	}
	LEAVE();
}

/*
 * Copy from sg to a dma block - used for transfers
 */
static inline void nuc980_sd_sg_to_dma(struct nuc980_sd_host *host, struct mmc_data *data)
{
	unsigned int len, i, size;
	unsigned *dmabuf = host->buffer;
	ENTRY();
	size = data->blksz * data->blocks;
	len = data->sg_len;

	/*
	 * Just loop through all entries. Size might not
	 * be the entire list though so make sure that
	 * we do not transfer too much.
	 */
	for (i = 0; i < len; i++) {
		struct scatterlist *sg;
		int amount;
		unsigned int *sgbuffer;

		sg = &data->sg[i];

		sgbuffer = kmap_atomic(sg_page(sg)) + sg->offset;
		amount = min(size, sg->length);
		size -= amount;
		{
			char *tmpv = (char *)dmabuf;
			memcpy(tmpv, sgbuffer, amount);
			tmpv += amount;
			dmabuf = (unsigned *)tmpv;
		}

		kunmap_atomic(sgbuffer);
		data->bytes_xfered += amount;

		if (size == 0)
			break;
	}

	/*
	 * Check that we didn't get a request to transfer
	 * more data than can fit into the SG list.
	 */
	BUG_ON(size != 0);
	LEAVE();
}

/*
 * Handle after a dma read
 */
static void nuc980_sd_post_dma_read(struct nuc980_sd_host *host)
{
	struct mmc_command *cmd;
	struct mmc_data *data;
	unsigned int len, i, size;
	unsigned *dmabuf = host->buffer;
	ENTRY();
	cmd = host->cmd;
	if (!cmd) {
		nuc980_sd_debug("no command\n");
		return;
	}

	data = cmd->data;
	if (!data) {
		nuc980_sd_debug("no data\n");
		return;
	}

	size = data->blksz * data->blocks;
	len = data->sg_len;

	for (i = 0; i < len; i++) {
		struct scatterlist *sg;
		int amount;
		unsigned int *sgbuffer;

		sg = &data->sg[i];

		sgbuffer = kmap_atomic(sg_page(sg)) + sg->offset;
		amount = min(size, sg->length);
		size -= amount;
		{
			char *tmpv = (char *)dmabuf;
			memcpy(sgbuffer, tmpv, amount);
			tmpv += amount;
			dmabuf = (unsigned *)tmpv;
		}
		flush_kernel_dcache_page(sg_page(sg));
		kunmap_atomic(sgbuffer);
		data->bytes_xfered += amount;
		if (size == 0)
			break;
	}
	LEAVE();
}

/*
 * Handle transmitted data
 */
static void nuc980_sd_handle_transmitted(struct nuc980_sd_host *host)
{
	ENTRY();
	if (nuc980_sd_read(REG_SDISR) & SDISR_CRC_IF)
		nuc980_sd_write(REG_SDISR, SDISR_CRC_IF);

	/* check read/busy */
	nuc980_sd_write(REG_SDCSR, nuc980_sd_read(REG_SDCSR) | SDCSR_CLK_KEEP0);

	LEAVE();
}


/*
 * Update bytes tranfered count during a write operation
 */
static void nuc980_sd_update_bytes_xfered(struct nuc980_sd_host *host)
{
	struct mmc_data *data;
	ENTRY();
	/* always deal with the effective request (and not the current cmd) */
	if (host->request->cmd && host->request->cmd->error != 0)
		return;

	if (host->request->data) {
		data = host->request->data;
		if (data->flags & MMC_DATA_WRITE) {
			/* card is in IDLE mode now */
			data->bytes_xfered = data->blksz * data->blocks;
		}
	}
	LEAVE();
}

/*-----------------------------------------------------------------------------
 * Config SIC register to select SD port.
 *---------------------------------------------------------------------------*/
#include <linux/delay.h>

/*
 * Enable the controller
 */
static void nuc980_sd_enable(struct nuc980_sd_host *host)
{
	ENTRY();
	nuc980_sd_write(REG_DMACCSR, DMACCSR_DMACEN);   // enable DMAC for FMI
	nuc980_sd_write(REG_FMICSR, FMICSR_SD_EN);  /* Enable SD functionality of FMI */
	nuc980_sd_write(REG_SDISR, 0xffffffff);
	nuc980_sd_write(REG_SDIER, nuc980_sd_read(REG_SDIER) | SDIER_CD0SRC);   // select GPIO detect
	nuc980_sd_write(REG_SDCSR, (nuc980_sd_read(REG_SDCSR) & 0x9fffffff)); //SD Port 0 is selected
	nuc980_sd_write(REG_SDCSR, (nuc980_sd_read(REG_SDCSR) & ~0xfff0000)|0x09010000);
	LEAVE();
}

/*
 * Disable the controller
 */
static void nuc980_sd_disable(struct nuc980_sd_host *host)
{
	ENTRY();
	nuc980_sd_write(REG_DMACCSR, DMACCSR_DMACEN | DMACCSR_SW_RST); //enable DMAC for FMI
	nuc980_sd_write(REG_FMICSR, FMICSR_SW_RST); /* Enable SD functionality of FMI */
	nuc980_sd_write(REG_SDISR, 0xffffffff);
	nuc980_sd_write(REG_FMICSR, nuc980_sd_read(REG_FMICSR) & ~FMICSR_SD_EN);
	LEAVE();
}

/*
 * Send a command
 */
static void nuc980_sd_send_command(struct nuc980_sd_host *host, struct mmc_command *cmd)
{
	unsigned int volatile csr;
	unsigned int block_length;
	struct mmc_data *data = cmd->data;
	int clock_free_run_status = 0;
	unsigned int blocks;
	ENTRY();
	host->cmd = cmd;

	sd_state_xfer = 0;

	if (nuc980_sd_read(REG_FMICSR) != FMICSR_SD_EN)
		nuc980_sd_write(REG_FMICSR, FMICSR_SD_EN);

	csr = (nuc980_sd_read(REG_SDCSR) & 0xff00c080) | 0x09010000; //schung20170811

	clock_free_run_status = csr | 0x80; /* clock keep */
	csr = csr & ~0x80;
	csr = csr | (cmd->opcode << 8) | SDCSR_CO_EN;   // set command code and enable command out
	sd_event |= SD_EVENT_CMD_OUT;

	if (host->bus_mode == MMC_BUS_WIDTH_4)
		csr |= SDCSR_DBW;

	if (mmc_resp_type(cmd) != MMC_RSP_NONE) {
		/* if a response is expected then allow maximum response latancy */
		/* set 136 bit response for R2, 48 bit response otherwise */
		if (mmc_resp_type(cmd) == MMC_RSP_R2) {
			csr |= SDCSR_R2_EN;
			sd_event |= SD_EVENT_RSP2_IN;
		} else {
			csr |= SDCSR_RI_EN;
			sd_event |= SD_EVENT_RSP_IN;
		}
		nuc980_sd_write(REG_SDISR, SDISR_RITO_IF);
		sd_ri_timeout = 0;
		nuc980_sd_write(REG_SDTMOUT, 0xffff);
	}

	if (data) {
		nuc980_sd_write(REG_SDIER, nuc980_sd_read(REG_SDIER) | SDIER_BLKD_IE);  //Enable SD interrupt & select GPIO detect
		block_length = data->blksz;
		blocks = data->blocks;
		//printk("block_length=%d\n",block_length); //schung
		nuc980_sd_write(REG_SDBLEN, block_length-1);
		if ((block_length > 512) || (blocks >= 256))
			printk("ERROR: don't support read/write 256 blocks in on CMD\n");
		else
			csr = (csr & ~0x00ff0000) | (blocks << 16);
	} else {
		nuc980_sd_write(REG_SDIER, nuc980_sd_read(REG_SDIER) & ~SDIER_BLKD_IE); //disable SD interrupt & select GPIO detect
		block_length = 0;
		blocks = 0;
	}

	/*
	 * Set the arguments and send the command
	 */
	nuc980_sd_debug("Sending command %d as 0x%0X, arg = 0x%08X, blocks = %d, length = %d\n",
	                cmd->opcode, csr, cmd->arg, blocks, block_length);

	if (data) {
		data->bytes_xfered = 0;
		host->transfer_index = 0;
		host->in_use_index = 0;
		if (data->flags & MMC_DATA_READ) {
			/*
			 * Handle a read
			 */
			//down_interruptible(&sdh_dmac_sem); //schung20170814
			nuc980_sd_write(REG_SDTMOUT, 0x3fffff);  //schung20170810
			host->total_length = 0;
			nuc980_sd_write(REG_DMACSAR2, host->physical_address);
			csr = csr | SDCSR_DI_EN;
			nuc980_sd_debug("SDH - Reading %d bytes [phy_addr = 0x%x]\n", block_length * blocks, host->physical_address);
		} else if (data->flags & MMC_DATA_WRITE) {
			//if (down_interruptible(&sdh_dmac_sem))  // get sdio_dmac_sem for data writing.
			//    return;
			//down_interruptible(&sdh_dmac_sem); //schung20170808
			host->total_length = block_length * blocks;
			nuc980_sd_sg_to_dma(host, data);
			nuc980_sd_debug("SDH - Transmitting %d bytes\n", host->total_length);
			nuc980_sd_write(REG_DMACSAR2, host->physical_address);
			csr = csr | SDCSR_DO_EN;
		}
	}
	/*
	 * Send the command and then enable the PDC - not the other way round as
	 * the data sheet says
	 */
	nuc980_sd_write(REG_SDARG, cmd->arg);
	nuc980_sd_write(REG_SDCSR, csr);

	nuc980_sd_event_thread(host);
	if (data) {
		if (data->flags & MMC_DATA_WRITE) {
			while (!(nuc980_sd_read(REG_SDISR) & SDISR_SDDAT0)) {
				nuc980_sd_write(REG_SDCSR, nuc980_sd_read(REG_SDCSR) | 0x40);   /* clk8_oe */
				while (nuc980_sd_read(REG_SDCSR) & 0x40);
			}
			nuc980_sd_update_bytes_xfered(host);
		}
	}
	if (clock_free_run_status) {
		nuc980_sd_write(REG_SDCSR, nuc980_sd_read(REG_SDCSR) | 0x80);   /* clock keep */
	}
	mmc_request_done(host->mmc, host->request);
	LEAVE();
}

static void nuc980_sd_send_stop(struct nuc980_sd_host *host, struct mmc_command *cmd)
{
	unsigned int csr;
	unsigned int block_length;
	unsigned int blocks;
	ENTRY();
	host->cmd = cmd;

	sd_state_xfer = 0;

	if (nuc980_sd_read(REG_FMICSR) != FMICSR_SD_EN)
		nuc980_sd_write(REG_FMICSR, FMICSR_SD_EN);

	csr = (nuc980_sd_read(REG_SDCSR) & 0xff00c080) | 0x09010000; //schung20170811

	csr = csr | (cmd->opcode << 8) | SDCSR_CO_EN;   // set command code and enable command out

	sd_event |= SD_EVENT_CMD_OUT;

	if (host->bus_mode == MMC_BUS_WIDTH_4)
		csr |= SDCSR_DBW;

	if (mmc_resp_type(cmd) != MMC_RSP_NONE) {
		/* if a response is expected then allow maximum response latancy */

		/* set 136 bit response for R2, 48 bit response otherwise */
		if (mmc_resp_type(cmd) == MMC_RSP_R2) {
			csr |= SDCSR_R2_EN;
			sd_event |= SD_EVENT_RSP2_IN;
		} else {
			csr |= SDCSR_RI_EN;
			sd_event |= SD_EVENT_RSP_IN;
		}
		nuc980_sd_write(REG_SDISR, SDISR_RITO_IF);
		sd_ri_timeout = 0;
		nuc980_sd_write(REG_SDTMOUT, 0xffff);
	}

	nuc980_sd_write(REG_SDIER, nuc980_sd_read(REG_SDIER) & ~SDIER_BLKD_IE); //disable SD interrupt & select GPIO detect
	block_length = 0;
	blocks = 0;

	nuc980_sd_write(REG_SDARG, cmd->arg);
	nuc980_sd_write(REG_SDCSR, csr);
	nuc980_sd_event_thread(host);
	mmc_request_done(host->mmc, host->request);
	LEAVE();
}


/*
 * Process the request
 */
static void nuc980_sd_send_request(struct nuc980_sd_host *host)
{

	ENTRY();

	if (!(host->flags & FL_SENT_COMMAND)) {
		host->flags |= FL_SENT_COMMAND;
		nuc980_sd_send_command(host, host->request->cmd);
	} else if ((!(host->flags & FL_SENT_STOP)) && host->request->stop) {
		host->flags |= FL_SENT_STOP;
		nuc980_sd_send_stop(host, host->request->stop);
	} else {
		del_timer(&host->timer);
	}
	LEAVE();
}

/*
 * Handle a command that has been completed
 */
static void nuc980_sd_completed_command(struct nuc980_sd_host *host, unsigned int status)
{
	struct mmc_command *cmd = host->cmd;
	struct mmc_data *data = cmd->data;
	unsigned int i, j, tmp[5], err;
	unsigned char *ptr;
	ENTRY();
	err = nuc980_sd_read(REG_SDISR);
	if ((err & SDISR_RITO_IF) || (cmd->error)) {
		nuc980_sd_write(REG_SDTMOUT, 0x0);
		nuc980_sd_write(REG_SDISR, SDISR_RITO_IF);
		cmd->error = -ETIMEDOUT;
		cmd->resp[0] = cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
	} else {
		if (status & SD_EVENT_RSP_IN) {
			// if not R2
			cmd->resp[0] = (nuc980_sd_read(REG_SDRSP0) << 8)|(nuc980_sd_read(REG_SDRSP1) & 0xff);
			cmd->resp[1] = cmd->resp[2] = cmd->resp[3] = 0;
		} else if (status & SD_EVENT_RSP2_IN) {
			// if R2
			ptr = (unsigned char *)FB0_BASE_ADDR;
			for (i=0, j=0; j<5; i+=4, j++)
				tmp[j] = (*(ptr+i)<<24)|(*(ptr+i+1)<<16)|(*(ptr+i+2)<<8)|(*(ptr+i+3));
			for (i=0; i<4; i++)
				cmd->resp[i] = ((tmp[i] & 0x00ffffff)<<8)|((tmp[i+1] & 0xff000000)>>24);
		}
	}
	//nuc980_sd_debug("Event = 0x%0X [0x%08X 0x%08X] <0x%x>\n", status, cmd->resp[0], cmd->resp[1], err);

	if (!cmd->error) {
		if ((err & SDISR_CRC_7) == 0) {
			if (!(mmc_resp_type(cmd) & MMC_RSP_CRC)) {
				cmd->error = 0;
				nuc980_sd_write(REG_SDISR, SDISR_CRC_IF);
			} else {
				cmd->error = -EIO;
				nuc980_sd_debug("Error detected and set to %d/%d (cmd = %d, retries = %d)\n",
				                cmd->error, data ? data->error : 0, cmd->opcode, cmd->retries);
			}
		} else
			cmd->error = 0;
		if (data) {
			data->bytes_xfered = 0;
			host->transfer_index = 0;
			host->in_use_index = 0;

			if(host->request->cmd->data->flags ==MMC_DATA_WRITE &&
			    host->request->cmd->opcode == 53 &&
			    host->request->cmd->data->blksz  <=64) {
				/* To Avoid CMD53 reading data than less 64 bytes will be worng */
				while((nuc980_sd_read(REG_SDISR)&0x70)!=0x20);
				udelay(2);
				nuc980_sd_write(REG_DMACCSR, nuc980_sd_read(REG_DMACCSR) | DMACCSR_SW_RST);
				while(nuc980_sd_read(REG_DMACCSR)&0x2);
				nuc980_sd_write(REG_SDCSR, nuc980_sd_read(REG_SDCSR) | (1<<14));
				while(nuc980_sd_read(REG_SDCSR)&(1<<14));
			} else {
				if (wait_event_interruptible_timeout(sd_wq_xfer, (sd_state_xfer != 0), 20000) == 0) {
					printk("SD time-out cmd=%d blksz=%d\n",host->request->cmd->opcode,host->request->cmd->data->blksz);
					nuc980_sd_write(REG_DMACCSR, nuc980_sd_read(REG_DMACCSR) | DMACCSR_SW_RST);
					while(nuc980_sd_read(REG_DMACCSR)&0x2);
					nuc980_sd_write(REG_SDCSR, nuc980_sd_read(REG_SDCSR) | (1<<14));
					while(nuc980_sd_read(REG_SDCSR)&(1<<14));
				}
			}
		}
	}

	if (0) { //(cmd->opcode==51)
		//schung
		printk("0x%08x 0x%08x\n",
		       host->buffer[0],
		       host->buffer[1]
		      );

		printk("cmd->resp[0]=0x%08x\n",cmd->resp[0]);
		printk("cmd->resp[1]=0x%08x\n",cmd->resp[1]);
		printk("cmd->resp[2]=0x%08x\n",cmd->resp[2]);
		printk("cmd->resp[3]=0x%08x\n",cmd->resp[3]);
	}

	if ((!(host->flags & FL_SENT_STOP)) && host->request->stop) {
		host->flags |= FL_SENT_STOP;
		nuc980_sd_send_stop(host, host->request->stop);
	}
	LEAVE();
}

/*
 * Handle an MMC request
 */
static int nuc980_sd_card_detect(struct mmc_host *mmc)
{
	struct nuc980_sd_host *host = mmc_priv(mmc);
	int ret;
	ENTRY();
	if (down_interruptible(&sdh_fmi_sem)) return -ERESTARTSYS;
	if (nuc980_sd_read(REG_FMICSR) != FMICSR_SD_EN)
		nuc980_sd_write(REG_FMICSR, FMICSR_SD_EN);

	host->present = nuc980_sd_read(REG_SDISR) & SDISR_CDPS0;
	ret = host->present ? 0 : 1;
	up(&sdh_fmi_sem);
	LEAVE();
	return ret;
}

static void nuc980_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct nuc980_sd_host *host = mmc_priv(mmc);
	int card_present;
	host->request = mrq;
	host->flags = 0;
	ENTRY();
	/* more than 1s timeout needed with slow SD cards */
	//mod_timer(&host->timer, jiffies +  msecs_to_jiffies(2000));
	//return -ENOSYS;
	card_present = nuc980_sd_card_detect(mmc);
	if (down_interruptible(&sdh_fmi_sem)) return;
	if (card_present == 0) {
		nuc980_sd_debug("no medium present\n");
		host->request->cmd->error = -ENOMEDIUM;
		mmc_request_done(host->mmc, host->request);
	} else
		nuc980_sd_send_request(host);
	up(&sdh_fmi_sem);
	LEAVE();
}

/*
 * Set the IOS
 */
extern unsigned long get_cpu_clk(void);
static void nuc980_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct nuc980_sd_host *host = mmc_priv(mmc);
	host->bus_mode = ios->bus_width;
	ENTRY();
	if (down_interruptible(&sdh_fmi_sem)) return;
	/* maybe switch power to the card */
	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		//dump_sdh_regs();
		nuc980_sd_write(REG_FMICSR, 0);
		__raw_writel(0x1, REG_SDECR);
		break;
	case MMC_POWER_UP:
	case MMC_POWER_ON: // enable 74 clocks
		__raw_writel(0, REG_SDECR);
		nuc980_sd_write(REG_FMICSR, FMICSR_SD_EN);

		if (ios->clock == 0) {
			up(&sdh_fmi_sem);
			LEAVE();
			return;
		}
		//printk("ios->clock=%d\n",ios->clock);
		if (ios->clock <= 400000) {
			clk_set_parent(host->mux_clk, host->xin_clk);
			clk_set_rate(host->div_clk, ios->clock);
			//__raw_writel((__raw_readl(REG_CLK_DIV9) & ~0xff00) | ((0xff) << 8),REG_CLK_DIV9);     // SD clock divided by CLKDIV3[SD_N] [15:8]
			nuc980_sd_write(REG_SDCSR, nuc980_sd_read(REG_SDCSR) | SDCSR_CLK74_OE);
			while (nuc980_sd_read(REG_SDCSR) & SDCSR_CLK74_OE);
		} else {
			clk_set_parent(host->mux_clk, host->upll_clk);
			clk_set_rate(host->div_clk, ios->clock);
		}
		break;

	default:
		WARN_ON(1);
	}

	if (ios->bus_width == MMC_BUS_WIDTH_4) {
		nuc980_sd_debug("MMC: Setting controller bus width to 4\n");
		nuc980_sd_write(REG_SDCSR, nuc980_sd_read(REG_SDCSR) | SDCSR_DBW);
	} else {
		//nuc980_sd_debug("MMC: Setting controller bus width to 1\n");
		nuc980_sd_write(REG_SDCSR, nuc980_sd_read(REG_SDCSR) & ~SDCSR_DBW);
	}
	up(&sdh_fmi_sem);
	LEAVE();
}

#define SD_TIMEOUT	10000000
/*
 * Handle CO, RI, and R2 event
 */
static int nuc980_sd_event_thread(struct nuc980_sd_host *sd_host)
{
	int event = 0;
	int completed = 0;
	unsigned int timeout = 0;
	ENTRY();

	completed = 0;
	event = sd_event;
	sd_event = SD_EVENT_NONE;
	if (event & SD_EVENT_CMD_OUT) {
		while (timeout < SD_TIMEOUT)
		{
			if (!(nuc980_sd_read(REG_SDCSR) & SDCSR_CO_EN)) {
				completed = 1;
				break;
			}else{
				ndelay(100);
				timeout++;
			}
		}
	}

	if (event & SD_EVENT_RSP_IN) {
		timeout=0;
		while (timeout < SD_TIMEOUT){
			if (!(nuc980_sd_read(REG_SDCSR) & SDCSR_RI_EN)) {
				completed = 1;
				break;
			}

			if (nuc980_sd_read(REG_SDISR) & SDISR_RITO_IF) {
				nuc980_sd_write(REG_SDTMOUT, 0x0);
				nuc980_sd_write(REG_SDISR, SDISR_RITO_IF);

				completed = 1;
				sd_host->cmd->error = -ETIMEDOUT;
				break;
			}
			timeout++;
			ndelay(100);
		}
	}

	if (event & SD_EVENT_RSP2_IN) {
		timeout = 0;
		while (timeout < SD_TIMEOUT) {
			if (!(nuc980_sd_read(REG_SDCSR) & SDCSR_R2_EN)) {
				completed = 1;
				break;
			}else{
				ndelay(100);
				timeout++;
			}
		}
	}
	if(timeout >= SD_TIMEOUT) {
		completed = 1;
		sd_host->cmd->error = -ETIMEDOUT;
	}
	if (completed) {
		nuc980_sd_completed_command(sd_host, event);
	}

	nuc980_sd_debug("SD0 event quit\n");
	LEAVE();
	return 0;
}

/*
 * Handle an interrupt
 */
static irqreturn_t nuc980_sd_irq(int irq, void *devid)
{
	struct nuc980_sd_host *host = devid;
	unsigned int int_status, present;
	ENTRY();
	int_status = nuc980_sd_read(REG_SDISR);
	//nuc980_sd_debug("FMI irq: status = %08X <0x%x, csr 0x%x>\n", int_status, nuc980_sd_read(REG_MFP_GPD_L), nuc980_sd_read(REG_SDCSR));
	if (int_status & 0x400) { /* sdio 0 interrupt */
		nuc980_sd_write(REG_SDIER, nuc980_sd_read(REG_SDIER) & ~0x400);
		nuc980_sd_write(REG_SDISR, 0x400);
		mmc_signal_sdio_irq(host->mmc);
	}


	if (int_status & SDISR_BLKD_IF) {
		nuc980_sd_debug("SDH xfer done.\n");

		if ((host->cmd == 0) || (host->cmd->data == 0)) {
			//nuc980_sd_debug("SD %d irq: port select = %d, found NULL pointer!!\n", host->port, reg_port_select);
			return IRQ_NONE;
		}

		if (host->cmd->data->flags & MMC_DATA_WRITE) {
			nuc980_sd_handle_transmitted(host);
		} else if (host->cmd->data->flags & MMC_DATA_READ) {
			nuc980_sd_post_dma_read(host);
		}
		nuc980_sd_write(REG_SDISR, SDISR_BLKD_IF);
		sd_state_xfer = 1;
		wake_up_interruptible(&sd_wq_xfer);

	}

	/*
	 * we expect this irq on both insert and remove,
	 * and use a short delay to debounce.
	 */
	/* SD card port 0 detect */
	if (int_status & SDISR_CD0_IF) {
		present = int_status & SDISR_CDPS0;
		host->present = present;
		nuc980_sd_debug("%s: card %s\n", mmc_hostname(host->mmc), present ? "remove" : "insert");
		if (!present) {
			nuc980_sd_debug("****** Resetting SD-card bus width ******\n");
			nuc980_sd_write(REG_SDCSR, nuc980_sd_read(REG_SDCSR) & ~SDCSR_DBW);
		}
		/* 0.5s needed because of early card detect switch firing */
		mmc_detect_change(host->mmc, msecs_to_jiffies(500));
		nuc980_sd_write(REG_SDISR, SDISR_CD0_IF);
	}
	LEAVE();
	return IRQ_HANDLED;
}

static int nuc980_sd_get_ro(struct mmc_host *mmc)
{
	/* TODO: check write protect pin */
	/* if write protect, it should return >0 value */

	/* no write protect */
	return 0;

	/*
	 * Board doesn't support read only detection; let the mmc core
	 * decide what to do.
	 */
	//return -ENOSYS;
}

static void nuc900_sd_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	//struct nuc980_sd_host *host = mmc_priv(mmc);
	ENTRY();
	nuc980_sd_write(REG_DMACIER, DMACIER_TABORT_IE);    //Enable target abort interrupt generation during DMA transfer
	nuc980_sd_write(REG_FMIIER, FMIIER_DTA_IE); //Enable DMAC READ/WRITE target abort interrupt generation

	nuc980_sd_write(REG_SDIER, nuc980_sd_read(REG_SDIER) & ~0x400);
	nuc980_sd_write(REG_SDISR, 0x400);
	if (enable) {
		nuc980_sd_write(REG_SDIER, nuc980_sd_read(REG_SDIER) | 0x4400); /* sdio interrupt */
		nuc980_sd_write(REG_SDCSR, nuc980_sd_read(REG_SDCSR) | 0x80);   /* clock keep */
	} else {
		nuc980_sd_write(REG_SDIER, nuc980_sd_read(REG_SDIER) & ~0x4400);/* sdio interrupt */
		nuc980_sd_write(REG_SDCSR, nuc980_sd_read(REG_SDCSR) & ~0x80);  /* clock keep */
	}
	LEAVE();
}

static const struct mmc_host_ops nuc980_sd_ops = {
	.request    = nuc980_sd_request,
	.set_ios    = nuc980_sd_set_ios,
	.get_ro     = nuc980_sd_get_ro,
	.get_cd     = nuc980_sd_card_detect,
	.enable_sdio_irq = nuc900_sd_enable_sdio_irq,
};

/*
 * Probe for the device
 */
static int nuc980_sd_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc=NULL;
	struct nuc980_sd_host *host=NULL;
	struct resource *res;
	int ret;
	struct clk *clkmux;
#ifndef CONFIG_USE_OF
	struct pinctrl *p;
#endif
	struct clk *sd_clk=NULL,*upll_clk=NULL,*xin_clk=NULL,*div_clk=NULL;
	ENTRY();
	printk("%s - pdev = %s\n", __func__, pdev->name);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;
	if (!request_mem_region(res->start, res->end - res->start + 1, DRIVER_NAME))
		return -EBUSY;

	sema_init(&sdh_fmi_sem, 1);


	/* initial SD1 pin -> PD0~7 */
#ifndef CONFIG_USE_OF
	//__raw_writel((__raw_readl(REG_MFP_GPF_L)&~0x0FFFFFFF) | 0x02222222,REG_MFP_GPF_L);
	p = devm_pinctrl_get_select(&pdev->dev, "sd1");
	if (IS_ERR(p)) {
		dev_err(&pdev->dev, "unable to reserve pin\n");
		ret = PTR_ERR(p);
		return ret;
	}
#endif

	clk_prepare(clk_get(NULL, "sdh1_hclk"));
	clk_enable(clk_get(NULL, "sdh1_hclk"));
	sd_clk = clk_get(NULL, "sdh1_eclk");
	if (IS_ERR(sd_clk)) {
		ret = -ENODEV;
		dev_dbg(&pdev->dev, "no sd_clk?\n");
		goto fail2;
	}
	clk_prepare(sd_clk);
	clk_enable(sd_clk);       /* Enable the peripheral clock */

	clkmux = clk_get(NULL, "sdh1_eclk_mux");
	if (IS_ERR(clkmux)) {
		printk(KERN_ERR "nuc980-sdh:failed to get sdh clock source\n");
		ret = PTR_ERR(clkmux);
		return ret;
	}

	xin_clk = clk_get(NULL, "xin");
	if (IS_ERR(upll_clk)) {
		printk(KERN_ERR "nuc980-sdh:failed to get sdh clock source\n");
		ret = PTR_ERR(upll_clk);
		return ret;
	}

	upll_clk = clk_get(NULL, "upll");
	if (IS_ERR(upll_clk)) {
		printk(KERN_ERR "nuc980-sdh:failed to get sdh clock source\n");
		ret = PTR_ERR(upll_clk);
		return ret;
	}
	clk_set_parent(clkmux, upll_clk);

	div_clk = clk_get(NULL, "sdh1_eclk_div");

	clk_set_rate(div_clk, 10000000);
	//__raw_writel((__raw_readl(REG_CLK_DIV9) & ~0xff00) | ((0xff) << 8),REG_CLK_DIV9);   // SD clock divided by CLKDIV3[SD_N] [15:8]

#if defined (CONFIG_MMC_NUC980_SD)
	/* SD Port 0 initial */
	mmc = mmc_alloc_host(sizeof(struct nuc980_sd_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "couldn't allocate mmc host\n");
		goto fail6;
	}

	mmc->ops = &nuc980_sd_ops;
	mmc->f_min = 300000;
	//mmc->f_max = 50000000;
	mmc->f_max = 25000000; //for SDIO WIFI RTL8821AS
	mmc->ocr_avail = MMC_VDD_27_28|MMC_VDD_28_29|MMC_VDD_29_30|MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps = 0;
	mmc->max_blk_size  = MCI_MAXBLKSIZE;
	mmc->max_blk_count = MCI_BLKATONCE;
	mmc->max_req_size  = MCI_BUFSIZE;
	mmc->max_segs      = MCI_BLKATONCE;
	mmc->max_seg_size  = MCI_BUFSIZE;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->bus_mode = 0;
	mmc->caps |= (MMC_CAP_4_BIT_DATA|MMC_CAP_SDIO_IRQ|MMC_CAP_SD_HIGHSPEED|MMC_CAP_MMC_HIGHSPEED);
	host->port = 0;     // default SD port to check

	host->buffer = dma_alloc_coherent(&pdev->dev, MCI_BUFSIZE, &host->physical_address, GFP_KERNEL);
	if (!host->buffer) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "Can't allocate transmit buffer\n");
		goto fail5;
	}

	host->sd_clk = sd_clk;
	host->upll_clk = upll_clk;
	host->xin_clk= xin_clk;
	host->div_clk = div_clk;
	host->mux_clk = clkmux;

	nuc980_sd_disable(host);
	nuc980_sd_enable(host);

	/*
	 * Allocate the MCI interrupt
	 */
	host->irq = platform_get_irq(pdev, 0);
	ret = request_irq(host->irq, nuc980_sd_irq, IRQF_SHARED, mmc_hostname(mmc), host);
	if (ret) {
		dev_dbg(&pdev->dev, "request MCI interrupt failed\n");
		goto fail0;
	}

	/* add a thread to check CO, RI, and R2 */
	//kernel_thread(sd_event_thread, NULL, 0);
	setup_timer(&host->timer, nuc980_sd_timeout_timer, (unsigned long)host);
	platform_set_drvdata(pdev, mmc);
	/*
	 * Add host to MMC layer
	 */
	host->present = nuc980_sd_read(REG_SDISR) & SDISR_CDPS0;
	nuc980_sd_write(REG_SDIER, nuc980_sd_read(REG_SDIER) | SDIER_CD0_IE | SDIER_CD0SRC);    //Enable SD interrupt & select GPIO detect

	mmc_add_host(mmc);
	nuc980_sd_debug("Added NUC980 SD0 driver\n");
#endif

	LEAVE();
	return 0;

fail0:
	clk_disable(host->sd_clk);
	clk_put(host->sd_clk);
fail2:
	if (host->buffer)
		dma_free_coherent(&pdev->dev, MCI_BUFSIZE, host->buffer, host->physical_address);
fail5:
	mmc_free_host(mmc);
fail6:
	release_mem_region(res->start, res->end - res->start + 1);
	dev_err(&pdev->dev, "probe failed, err %d\n", ret);
	LEAVE();
	return ret;
}

/*
 * Remove a device
 */
static int nuc980_sd_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct nuc980_sd_host *host;
	ENTRY();
	if (!mmc)
		return -1;

	host = mmc_priv(mmc);

	if (host->buffer)
		dma_free_coherent(&pdev->dev, MCI_BUFSIZE, host->buffer, host->physical_address);

	nuc980_sd_disable(host);
	del_timer_sync(&host->timer);
	mmc_remove_host(mmc);
	free_irq(host->irq, host);

	clk_disable(host->sd_clk);            /* Disable the peripheral clock */
	clk_put(host->sd_clk);

	mmc_free_host(mmc);
	platform_set_drvdata(pdev, NULL);
	nuc980_sd_debug("NUC980 SD Removed\n");
	LEAVE();
	return 0;
}

#ifdef CONFIG_PM
static int nuc980_sd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct nuc980_sd_host *host = mmc_priv(mmc);
	int ret = 0;
	ENTRY();
	// For save, wait DMAC to ready
	while ( readl(REG_DMACCSR)      & 0x200 );
	nuc900_sd_enable_sdio_irq(mmc, 0);
	nuc980_sd_disable(host);
	clk_disable(host->sd_clk);
	LEAVE();
	return ret;
}

static int nuc980_sd_resume(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct nuc980_sd_host *host = mmc_priv(mmc);
	int ret = 0;
	ENTRY();
	clk_enable(host->sd_clk);
	nuc980_sd_enable(host);
	nuc900_sd_enable_sdio_irq(mmc, 1);
	LEAVE();
	return ret;
}
#else
#define nuc980_sd_suspend   NULL
#define nuc980_sd_resume    NULL
#endif

static const struct of_device_id nuc980_sdh_of_match[] = {
	{ .compatible = "nuvoton,nuc980-sdh" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_sdh_of_match);

static struct platform_driver nuc980_sd_driver = {
	.probe      = nuc980_sd_probe,
	.remove     = nuc980_sd_remove,
	.suspend    = nuc980_sd_suspend,
	.resume     = nuc980_sd_resume,
	.driver     = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(nuc980_sdh_of_match),
	},
};


module_platform_driver(nuc980_sd_driver);

MODULE_DESCRIPTION("NUC980 dual SD Card Interface driver");
MODULE_AUTHOR("HPChen");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc980_sd");
