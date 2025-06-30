/*
 * Copyright (c) 2018 Nuvoton Technology Corporation.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/gfp.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>
#include <linux/net_tstamp.h>
#include <linux/ptp_clock_kernel.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gcr.h>

#define DRV_MODULE_NAME		"nuc980-emc1"
#define DRV_MODULE_VERSION	"1.0"

/* Ethernet MAC1 Registers */
#define REG_CAMCMR		(void __iomem *)0xF0022000
#define REG_CAMEN		(void __iomem *)0xF0022004
#define REG_CAMM_BASE	(void __iomem *)0xF0022008
#define REG_CAML_BASE	(void __iomem *)0xF002200c
#define REG_TXDLSA		(void __iomem *)0xF0022088
#define REG_RXDLSA		(void __iomem *)0xF002208C
#define REG_MCMDR		(void __iomem *)0xF0022090
#define REG_MIID		(void __iomem *)0xF0022094
#define REG_MIIDA		(void __iomem *)0xF0022098
#define REG_FFTCR		(void __iomem *)0xF002209C
#define REG_TSDR		(void __iomem *)0xF00220a0
#define REG_RSDR		(void __iomem *)0xF00220a4
#define REG_DMARFC		(void __iomem *)0xF00220a8
#define REG_MIEN		(void __iomem *)0xF00220ac
#define REG_MISTA		(void __iomem *)0xF00220b0
#define REG_CTXDSA		(void __iomem *)0xF00220cc
#define REG_CTXBSA		(void __iomem *)0xF00220d0
#define REG_CRXDSA		(void __iomem *)0xF00220d4
#define REG_CRXBSA		(void __iomem *)0xF00220d8
#define REG_TSCTL 		(void __iomem *)0xF0022100
#define REG_TSSEC 		(void __iomem *)0xF0022110 /* [31:0] */
#define REG_TSSUBSEC 	(void __iomem *)0xF0022114 /* [30:0] */
#define REG_TSINC 		(void __iomem *)0xF0022118
#define REG_TSADDEND 	(void __iomem *)0xF002211c
#define REG_UPDSEC 		(void __iomem *)0xF0022120
#define REG_UPDSUBSEC 	(void __iomem *)0xF0022124
#define REG_ALMSEC 		(void __iomem *)0xF0022128
#define REG_ALMSUBSEC 	(void __iomem *)0xF002211c

/* mac controller bit */
#define MCMDR_RXON		0x01
#define MCMDR_ALP		(0x01 << 1)
#define MCMDR_ARP		(0x01 << 2)
#define MCMDR_ACP		(0x01 << 3)
#define MCMDR_SPCRC		(0x01 << 5)
#define MCMDR_MGPWAKE	(0x01 << 6)
#define MCMDR_TXON		(0x01 << 8)
#define MCMDR_FDUP		(0x01 << 18)
#define MCMDR_OPMOD		(0x01 << 20)
#define SWR				(0x01 << 24)

/* cam command register */
#define CAMCMR_AUP		0x01
#define CAMCMR_AMP		(0x01 << 1)
#define CAMCMR_ABP		(0x01 << 2)
#define CAMCMR_CCAM		(0x01 << 3)
#define CAMCMR_ECMP		(0x01 << 4)
#define CAM0EN			0x01

/* mac mii controller bit */
#define MDCON			(0x01 << 19)
//#define PHYAD			(0x01 << 8)
#define PHYWR			(0x01 << 16)
#define PHYBUSY			(0x01 << 17)
#define CAM_ENTRY_SIZE	0x08

/* rx and tx status */
#define TXDS_TXCP		(0x01 << 19)
#define RXDS_CRCE		(0x01 << 17)
#define RXDS_PTLE		(0x01 << 19)
#define RXDS_RXGD		(0x01 << 20)
#define RXDS_ALIE		(0x01 << 21)
#define RXDS_RP			(0x01 << 22)
#define RXDS_RTSAS		(0x01 << 23)

/* mac interrupt status*/
#define MISTA_EXDEF		(0x01 << 19)
#define MISTA_TXBERR	(0x01 << 24)
#define MISTA_TDU		(0x01 << 23)
#define MISTA_RDU		(0x01 << 10)
#define MISTA_RXBERR	(0x01 << 11)
#define MISTA_WOL		(0x01 << 15)
#define MISTA_RXGD		(0x01 << 4)
#define MISTA_TXEMP		(0x01 << 17)
#define MISTA_RXOV		(0x01 << 2)

#define ENSTART			0x01
#define ENRXINTR		0x01
#define ENRXGD			(0x01 << 4)
#define ENRDU			(0x01 << 10)
#define ENRXBERR		(0x01 << 11)
#define ENWOL			(0x01 << 15)
#define ENTXINTR		(0x01 << 16)
#define ENTXCP			(0x01 << 18)
#define ENTXABT			(0x01 << 21)
#define ENTXBERR		(0x01 << 24)
#define PHYBUSY			(0x01 << 17)


/* rx and tx owner bit */
#define RX_OWEN_DMA		(0x01 << 31)
#define RX_OWEN_CPU		(~(0x03 << 30))
#define TX_OWEN_DMA		(0x01 << 31)
#define TX_OWEN_CPU		(~(0x01 << 31))

/* tx frame desc controller bit */
#define TTSEN			0x08
#define MACTXINTEN		0x04
#define CRCMODE			0x02
#define PADDINGMODE		0x01

/* fftcr controller bit */
#define TXTHD 			(0x03 << 8)
#define BLENGTH			(0x01 << 20)

/*Time Stamp Control Register*/
#define TSEN 			0x01
#define TSIEN 			(0x01 << 1)
#define TSMODE 			(0x01 << 2)
#define TSUPDATE 		(0x01 << 3)
#define TSALMEN 		(0x01 << 5)

/* global setting for driver */
#define RX_DESC_SIZE	32
#define TX_DESC_SIZE	32
#define MAX_RBUFF_SZ	0x600
#define MAX_TBUFF_SZ	0x600
#define TX_TIMEOUT		50
#define DELAY			1000
#define CAM0			0x0

#define MII_TIMEOUT		100

#ifdef CONFIG_VLAN_8021Q
#define IS_VLAN 1
#else
#define IS_VLAN 0
#endif

// (ETH_FRAME_LEN + (IS_VLAN * VLAN_HLEN) + ETH_FCS_LEN + Align Size) < 0x600
#define MAX_PACKET_SIZE           1536

#define ETH_TRIGGER_RX	do{__raw_writel(ENSTART, REG_RSDR);}while(0)
#define ETH_TRIGGER_TX	do{__raw_writel(ENSTART, REG_TSDR);}while(0)
#define ETH_ENABLE_TX	do{__raw_writel(__raw_readl( REG_MCMDR) | MCMDR_TXON, REG_MCMDR);}while(0)
#define ETH_ENABLE_RX	do{__raw_writel(__raw_readl( REG_MCMDR) | MCMDR_RXON, REG_MCMDR);}while(0)
#define ETH_DISABLE_TX	do{__raw_writel(__raw_readl( REG_MCMDR) & ~MCMDR_TXON, REG_MCMDR);}while(0)
#define ETH_DISABLE_RX	do{__raw_writel(__raw_readl( REG_MCMDR) & ~MCMDR_RXON, REG_MCMDR);}while(0)

#define TS_ACCURACY_NS  20
#define EMAC_HCLK_FREQ	150000000ULL

struct nuc980_rxbd {
	unsigned int sl;
	unsigned int buffer;
	unsigned int reserved;
	unsigned int next;
};

struct nuc980_txbd {
	unsigned int mode;
	unsigned int buffer;
	unsigned int sl;
	unsigned int next;
};

u8 nuc980_mac1[6] = { 0x08, 0x00, 0x27, 0x00, 0x01, 0x93 };

static struct sk_buff *rx_skb[RX_DESC_SIZE];
static struct sk_buff *tx_skb[TX_DESC_SIZE];
// backup desp starting addr if ts enabled 
static unsigned int rxbd_next[RX_DESC_SIZE];
static unsigned int txbd_next[TX_DESC_SIZE];

struct  nuc980_ether {
	spinlock_t lock;
	struct nuc980_rxbd *rdesc;
	struct nuc980_txbd *tdesc;
	dma_addr_t rdesc_phys;
	dma_addr_t tdesc_phys;
	struct net_device_stats stats;
	struct platform_device *pdev;
	struct net_device *ndev;
	struct resource *res;
	struct clk *clk;
	struct clk *eclk;
	unsigned int msg_enable;
	struct mii_bus *mii_bus;
	struct phy_device *phy_dev;
	struct napi_struct napi;
	int rxirq;
	int txirq;
	unsigned int cur_tx;
	unsigned int cur_rx;
	unsigned int finish_tx;
	unsigned int start_tx_ptr;
	unsigned int start_rx_ptr;
	int link;
	int speed;
	int duplex;
	int wol;
	/* timestamp */
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info ptp_clock_ops;
	unsigned int default_addend;
	int hwts_tx_en;
	int hwts_rx_en;
	spinlock_t ptp_lock;
};

static void nuc980_config_hw_tstamping(u32 data)
{
	__raw_writel(data, REG_TSCTL);
}

static int nuc980_config_addend(u32 addend)
{
	int limit;

	__raw_writel(addend, REG_TSADDEND);
	
	limit = 10;
	while (limit--) {
		if ((__raw_readl(REG_TSADDEND) == addend))
			break;
		mdelay(10);
	}
	
	return 0;
}

static int nuc980_init_systime(struct nuc980_ether *priv, u32 sec, u32 nsec)
{
	int limit;
	u32 value;
	u64 ns;
	
	// Set value to load
	ns = (u64)sec * 1000000000ULL + nsec;
	__raw_writel(ns >> 31, REG_UPDSEC);
	__raw_writel(ns & 0x7FFFFFFF, REG_UPDSUBSEC);
	
	/* issue command to initialize the system time value */
	value = __raw_readl(REG_TSCTL);
	value |= TSIEN;
	__raw_writel(value, REG_TSCTL);

	/* wait for present system time initialize to complete */
	limit = 10;
	while (limit--) {
		if (!(__raw_readl(REG_TSCTL) & TSIEN))
			break;
		mdelay(10);
	}
	__raw_readl(REG_TSSUBSEC); // read subsec to update sec
	
	if (limit <= 0)
		return -EBUSY;

	return 0;
}

static int nuc980_adjust_freq(struct ptp_clock_info *ptp, s32 ppb)
{
	struct nuc980_ether *priv =
	    container_of(ptp, struct nuc980_ether, ptp_clock_ops);
	unsigned long flags;
	u32 diff, addend;
	int neg_adj = 0;
	u64 adj;

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}

	addend = priv->default_addend;
	adj = addend;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);
	addend = neg_adj ? (addend - diff) : (addend + diff);
	
	spin_lock_irqsave(&priv->ptp_lock, flags);
	nuc980_config_addend(addend);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int nuc980_adjust_time(struct ptp_clock_info *ptp, s64 delta)
{
	struct nuc980_ether *priv =
	    container_of(ptp, struct nuc980_ether, ptp_clock_ops);
	unsigned long flags;
	u32 sec, nsec;
	int limit;

	spin_lock_irqsave(&priv->ptp_lock, flags);

	sec = delta >> 31;
	nsec = delta & 0x7FFFFFFF;

	/* issue command to adjust system time value */
	__raw_writel(sec, REG_UPDSEC);
	__raw_writel(nsec, REG_UPDSUBSEC);

	__raw_writel(__raw_readl(REG_TSCTL)|TSUPDATE, REG_TSCTL);

	limit = 10;
	while (limit--) {
		if (!(__raw_readl(REG_TSCTL) & TSUPDATE))
			break;
		mdelay(10);
	}
	__raw_readl(REG_TSSUBSEC); // read subsec to update sec

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int nuc980_get_time(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct nuc980_ether *priv =
	    container_of(ptp, struct nuc980_ether, ptp_clock_ops);
	unsigned long flags;
	u64 ns;
	u32 sec, subsec;

	spin_lock_irqsave(&priv->ptp_lock, flags);

	sec = __raw_readl(REG_TSSEC);
	subsec = __raw_readl(REG_TSSUBSEC);
	ns = ((u64)sec << 31) | (subsec & 0x7FFFFFFF);

	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	*ts = ns_to_timespec64(ns);

	return 0;
}

static int nuc980_set_time(struct ptp_clock_info *ptp,
			   const struct timespec64 *ts)
{
	struct nuc980_ether *priv =
	    container_of(ptp, struct nuc980_ether, ptp_clock_ops);
	unsigned long flags;

	spin_lock_irqsave(&priv->ptp_lock, flags);

	nuc980_init_systime(priv, ts->tv_sec, ts->tv_nsec);

	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return 0;
}

static int nuc980_enable(struct ptp_clock_info *ptp,
			 struct ptp_clock_request *rq, int on)
{
	return -EOPNOTSUPP;
}

static struct ptp_clock_info nuc980_ptp_clock_ops = {
	.owner = THIS_MODULE,
	.name = "nuc980_ptp_clock",
	.max_adj = 62500000,
	.n_alarm = 0,
	.n_ext_ts = 0,
	.n_per_out = 0,
	.pps = 0,
	.adjfreq = nuc980_adjust_freq,
	.adjtime = nuc980_adjust_time,
	.gettime64 = nuc980_get_time,
	.settime64 = nuc980_set_time,
	.enable = nuc980_enable,
};

/*----------------------- ptp helper ------------------------*/
static int nuc980_ptp_register(struct nuc980_ether *priv)
{
	spin_lock_init(&priv->ptp_lock);
	priv->ptp_clock_ops = nuc980_ptp_clock_ops;
#ifdef CONFIG_PTP_1588_CLOCK
	priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_ops,
					     &priv->pdev->dev);
#endif
	if (IS_ERR(priv->ptp_clock)) {
		netdev_err(priv->ndev, "ptp_clock_register failed\n");
		priv->ptp_clock = NULL;
		return -1;
	} 
	else {
		//netdev_info(priv->ndev, "registered PTP clock\n");
		return 0;
	}
}

static void nuc980_ptp_unregister(struct nuc980_ether *priv)
{
	if (priv->ptp_clock) {
#ifdef CONFIG_PTP_1588_CLOCK
		ptp_clock_unregister(priv->ptp_clock);
#endif
		//netdev_info(priv->ndev, "unregistered PTP clock\n");
	}
}

static int nuc980_init_ptp(struct nuc980_ether *priv)
{
	nuc980_config_hw_tstamping(0);//stop HW timestamp
	priv->hwts_tx_en = 0;
	priv->hwts_rx_en = 0;

	return nuc980_ptp_register(priv);
}

static void nuc980_release_ptp(struct nuc980_ether *priv)
{
	nuc980_ptp_unregister(priv);
}

static u64 nuc980_desc_get_timestamp(struct nuc980_ether *priv,void *desc, u32 nTx)
{
	u64 ns;
	u32 tssec, tssubsec;
	
	if(nTx) {
		tssubsec = ((struct nuc980_txbd *)desc)->buffer;
		tssec = ((struct nuc980_txbd *)desc)->next;
	}
	else {
		tssubsec = ((struct nuc980_rxbd *)desc)->buffer;
		tssec = ((struct nuc980_rxbd *)desc)->next;
	}
	
	ns = ((u64)tssec << 31) | (tssubsec & 0x7FFFFFFF);
	
	return ns;
}

static void nuc980_get_tx_hwtstamp(struct nuc980_ether *priv, struct nuc980_txbd *txbd, struct sk_buff *skb)
{
	struct skb_shared_hwtstamps shhwtstamp;
	u64 ns;
	void *desc = NULL;

	/* exit if skb doesn't support hw tstamp */
	if (likely(!(skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS)))
		return;

	desc = (void*)txbd;

	/* get the valid tstamp */
	ns = nuc980_desc_get_timestamp(priv, desc, 1);

	memset(&shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
	shhwtstamp.hwtstamp = ns_to_ktime(ns);
	/* pass tstamp to stack */
	skb_tstamp_tx(skb, &shhwtstamp);

	return;
}

static void nuc980_get_rx_hwtstamp(struct nuc980_ether *priv, struct nuc980_rxbd *rxbd, struct sk_buff *skb)
{
	struct skb_shared_hwtstamps *shhwtstamp = NULL;
	u64 ns;

	/* get valid tstamp */
	ns = nuc980_desc_get_timestamp(priv, (void *)rxbd, 0);
		
	shhwtstamp = skb_hwtstamps(skb);
	memset(shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
	shhwtstamp->hwtstamp = ns_to_ktime(ns);
}

static int nuc980_hwtstamp_ioctl(struct net_device *dev, struct ifreq *ifr)
{
	struct nuc980_ether *priv = netdev_priv(dev);
	struct hwtstamp_config config;
	struct timespec64 now;
	u64 temp = 0;
	u32 value = 0;

	if (copy_from_user(&config, ifr->ifr_data,
			   sizeof(struct hwtstamp_config)))
		return -EFAULT;

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	switch (config.tx_type) {
		case HWTSTAMP_TX_OFF:
			priv->hwts_tx_en = 0;
			break;
		case HWTSTAMP_TX_ON:
			priv->hwts_tx_en = 1;
			break;
		default:
			return -ERANGE;
	}


	switch (config.rx_filter) {
		case HWTSTAMP_FILTER_NONE:
			config.rx_filter = HWTSTAMP_FILTER_NONE;
			break;
		case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
			/* PTP v2, UDP, any kind of event packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
			break;
		case HWTSTAMP_FILTER_PTP_V2_EVENT:
			/* PTP v2/802.AS1 any layer, any kind of event packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
			break;

		case HWTSTAMP_FILTER_PTP_V2_SYNC:
			/* PTP v2/802.AS1, any layer, Sync packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_SYNC;
			break;

		case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
			/* PTP v2/802.AS1, any layer, Delay_req packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_DELAY_REQ;
			break;

		default:
			/* PTP v1, UDP, any kind of event packet */
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
			break;
	}
	
	priv->hwts_rx_en = ((config.rx_filter == HWTSTAMP_FILTER_NONE) ? 0 : 1);

	if (!priv->hwts_tx_en && !priv->hwts_rx_en)
		nuc980_config_hw_tstamping(0);
	else {
		// Target precision is 20ns, means we need 50MHz clk,
		// if HCLK is 150MHz, addend is 2^32 / (150/50)
		value = TS_ACCURACY_NS;
		__raw_writel(value & 0xFF, REG_TSINC);
		temp = div_u64(EMAC_HCLK_FREQ * value, 1000000000UL);
		value = div_u64(1ULL << 32, temp);
		priv->default_addend = value;
		nuc980_config_addend(value);
		nuc980_config_hw_tstamping(TSEN|TSMODE);
		
		/* initialize system time */
		ktime_get_real_ts64(&now);
		
		nuc980_init_systime(priv, now.tv_sec, now.tv_nsec);
	}

	return copy_to_user(ifr->ifr_data, &config,
			    sizeof(struct hwtstamp_config)) ? -EFAULT : 0;
}
/*------------------End of ptp helper -----------------------*/

static __init int setup_macaddr(char *str)
{
	u8 mac[6] = {0, 0, 0, 0, 0, 0};
	char *c = str;
	int i, j;

	if (!str)
		goto err;

	for(i = 0; i < 6; i++) {
		for(j = 0; j < 2; j++) {
			mac[i] <<= 4;
			if(isdigit(*c))
				mac[i] += *c - '0';
			else if(isxdigit(*c))
				mac[i] += toupper(*c) - 'A' + 10;
			else {
				goto err;
			}
			c++;
		}

		if(i != 5)
			if(*c != ':') {
				goto err;
			}

		c++;
	}

	// all good
	for(i = 0; i < 6; i++) {
		nuc980_mac1[i] = mac[i];

	}
	return 0;

err:
	return -EINVAL;
}
early_param("ethaddr1", setup_macaddr);

static void adjust_link(struct net_device *dev)
{
	struct nuc980_ether *ether = netdev_priv(dev);
	struct phy_device *phydev = ether->phy_dev;
	unsigned int val;
	bool status_change = false;
	unsigned long flags;

	spin_lock_irqsave(&ether->lock, flags);

	if (phydev->link) {
		if ((ether->speed != phydev->speed) ||
		    (ether->duplex != phydev->duplex)) {
			ether->speed = phydev->speed;
			ether->duplex = phydev->duplex;
			status_change = true;
		}
	} else {
		// disable tx/rx
		__raw_writel(__raw_readl( REG_MCMDR) & ~(MCMDR_RXON | MCMDR_TXON), REG_MCMDR);
		ether->speed = 0;
		ether->duplex = -1;
	}

	if (phydev->link != ether->link) {

		ether->link = phydev->link;
		if(phydev->link)
			status_change = true;
	}

	spin_unlock_irqrestore(&ether->lock, flags);

	if (status_change) {

		val = __raw_readl( REG_MCMDR) | MCMDR_RXON | MCMDR_TXON;

		if (ether->speed == 100) {
			val |= MCMDR_OPMOD;
		} else {
			val &= ~MCMDR_OPMOD;
		}

		if(ether->duplex == DUPLEX_FULL) {
			val |= MCMDR_FDUP;
		} else {
			val &= ~MCMDR_FDUP;
		}

		__raw_writel(val,  REG_MCMDR);
		ETH_TRIGGER_TX; // in case some packets queued in descriptor
	}
}



static void nuc980_write_cam(struct net_device *dev,
				unsigned int x, unsigned char *pval)
{
	unsigned int msw, lsw;

	msw = (pval[0] << 24) | (pval[1] << 16) | (pval[2] << 8) | pval[3];

	lsw = (pval[4] << 24) | (pval[5] << 16);

	__raw_writel(lsw,  REG_CAML_BASE + x * CAM_ENTRY_SIZE);
	__raw_writel(msw,  REG_CAMM_BASE + x * CAM_ENTRY_SIZE);
}


static struct sk_buff * get_new_skb(struct net_device *dev, u32 i) {
	struct nuc980_ether *ether = netdev_priv(dev);
	struct sk_buff *skb = dev_alloc_skb(MAX_PACKET_SIZE);

	if (skb == NULL)
		return NULL;

	skb_reserve(skb, 2);
	skb->dev = dev;

	(ether->rdesc + i)->buffer = dma_map_single(&dev->dev, skb->data,
							MAX_PACKET_SIZE, DMA_FROM_DEVICE);
	rx_skb[i] = skb;

	return skb;
}

static int nuc980_init_desc(struct net_device *dev)
{
	struct nuc980_ether *ether;
	struct nuc980_txbd  *tdesc;
	struct nuc980_rxbd  *rdesc;
	struct platform_device *pdev;
	unsigned int i;

	ether = netdev_priv(dev);
	pdev = ether->pdev;

	ether->tdesc = (struct nuc980_txbd *)
			dma_alloc_coherent(&pdev->dev, sizeof(struct nuc980_txbd) * TX_DESC_SIZE,
						&ether->tdesc_phys, GFP_KERNEL);

	if (!ether->tdesc) {
		dev_err(&pdev->dev, "Failed to allocate memory for tx desc\n");
		return -ENOMEM;
	}

	ether->rdesc = (struct nuc980_rxbd *)
			dma_alloc_coherent(&pdev->dev, sizeof(struct nuc980_rxbd) * RX_DESC_SIZE,
						&ether->rdesc_phys, GFP_KERNEL);

	if (!ether->rdesc) {
		dev_err(&pdev->dev, "Failed to allocate memory for rx desc\n");
		dma_free_coherent(&pdev->dev, sizeof(struct nuc980_txbd) * TX_DESC_SIZE,
						ether->tdesc, ether->tdesc_phys);
		return -ENOMEM;
	}

	for (i = 0; i < TX_DESC_SIZE; i++) {
		unsigned int offset;

		tx_skb[i] = NULL;
		tdesc = (ether->tdesc + i);

		if (i == TX_DESC_SIZE - 1)
			offset = 0;
		else
			offset = sizeof(struct nuc980_txbd) * (i + 1);

		tdesc->next = ether->tdesc_phys + offset;
		tdesc->buffer = (unsigned int)NULL;
		tdesc->sl = 0;
		tdesc->mode = PADDINGMODE | CRCMODE | MACTXINTEN;
		txbd_next[i]=tdesc->next; // backup
	}

	ether->start_tx_ptr = ether->tdesc_phys;

	for (i = 0; i < RX_DESC_SIZE; i++) {
		unsigned int offset;

		rdesc = (ether->rdesc + i);

		if (i == RX_DESC_SIZE - 1)
			offset = 0;
		else
			offset = sizeof(struct nuc980_rxbd) * (i + 1);

		rdesc->next = ether->rdesc_phys + offset;
		rxbd_next[i] = rdesc->next; // backup
		rdesc->sl = RX_OWEN_DMA;
		if(get_new_skb(dev, i) == NULL) {
			dma_free_coherent(&pdev->dev, sizeof(struct nuc980_txbd) * TX_DESC_SIZE,
						ether->tdesc, ether->tdesc_phys);
			dma_free_coherent(&pdev->dev, sizeof(struct nuc980_rxbd) * RX_DESC_SIZE,
						ether->rdesc, ether->rdesc_phys);

			for(; i != 0; i--) {
				dma_unmap_single(&dev->dev, (dma_addr_t)((ether->rdesc + i)->buffer),
							MAX_PACKET_SIZE, DMA_FROM_DEVICE);
				dev_kfree_skb_any(rx_skb[i]);
			}
			return -ENOMEM;
		}
	}

	ether->start_rx_ptr = ether->rdesc_phys;

	return 0;
}

// This API must call with Tx/Rx stopped
static void nuc980_free_desc(struct net_device *dev)
{
	struct sk_buff *skb;
	u32 i;
	struct nuc980_ether *ether = netdev_priv(dev);
	struct platform_device *pdev = ether->pdev;

	for (i = 0; i < TX_DESC_SIZE; i++) {
		skb = tx_skb[i];
		if(skb != NULL) {
			dma_unmap_single(&dev->dev, (dma_addr_t)((ether->tdesc + i)->buffer), skb->len, DMA_TO_DEVICE);
			dev_kfree_skb_any(skb);
		}
	}

	for (i = 0; i < RX_DESC_SIZE; i++) {
		skb = rx_skb[i];
		if(skb != NULL) {
			dma_unmap_single(&dev->dev, (dma_addr_t)((ether->rdesc + i)->buffer), MAX_PACKET_SIZE, DMA_FROM_DEVICE);
			dev_kfree_skb_any(skb);
		}
	}

	dma_free_coherent(&pdev->dev, sizeof(struct nuc980_txbd) * TX_DESC_SIZE,
				ether->tdesc, ether->tdesc_phys);
	dma_free_coherent(&pdev->dev, sizeof(struct nuc980_rxbd) * RX_DESC_SIZE,
				ether->rdesc, ether->rdesc_phys);

}

static void nuc980_set_fifo_threshold(struct net_device *dev)
{
	unsigned int val;

	val = TXTHD | BLENGTH;
	__raw_writel(val,  REG_FFTCR);
}

static void nuc980_return_default_idle(struct net_device *dev)
{
	unsigned int val;

	val = __raw_readl( REG_MCMDR);
	val |= SWR;
	__raw_writel(val,  REG_MCMDR);
}


static void nuc980_enable_mac_interrupt(struct net_device *dev)
{
	unsigned int val;

	val = ENTXINTR | ENRXINTR | ENRXGD | ENTXCP | ENRDU;
	val |= ENTXBERR | ENRXBERR | ENTXABT | ENWOL;

	__raw_writel(val,  REG_MIEN);
}

static void nuc980_get_and_clear_int(struct net_device *dev,
							unsigned int *val, unsigned int mask)
{
	*val = __raw_readl( REG_MISTA) & mask;
	__raw_writel(*val,  REG_MISTA);
}

static void nuc980_set_global_maccmd(struct net_device *dev)
{
	unsigned int val;

	val = __raw_readl( REG_MCMDR) | MCMDR_SPCRC | MCMDR_ACP;
	if (IS_VLAN)
        {
		val |= MCMDR_ALP;
	}
	__raw_writel(val,  REG_MCMDR);
	__raw_writel(MAX_PACKET_SIZE,  REG_DMARFC);
}

static void nuc980_enable_cam(struct net_device *dev)
{
	unsigned int val;

	nuc980_write_cam(dev, CAM0, dev->dev_addr);

	val = __raw_readl( REG_CAMEN);
	val |= CAM0EN;
	__raw_writel(val,  REG_CAMEN);
}

static void nuc980_enable_cam_command(struct net_device *dev)
{
	unsigned int val;

	val = CAMCMR_ECMP | CAMCMR_ABP | CAMCMR_AMP;
	__raw_writel(val,  REG_CAMCMR);
}


static void nuc980_set_curdest(struct net_device *dev)
{
	struct nuc980_ether *ether = netdev_priv(dev);

	__raw_writel(ether->start_rx_ptr,  REG_RXDLSA);
	__raw_writel(ether->start_tx_ptr,  REG_TXDLSA);
}

static void nuc980_set_alp(struct net_device *dev, int bOn)
{
	unsigned int val;

	val = __raw_readl(REG_MCMDR);
	if (bOn)
	{
		val |= MCMDR_ALP;
	}
	else
	{
		val &= ~(MCMDR_ALP | MCMDR_ARP);
	}
	__raw_writel(val, REG_MCMDR);
}
#if 0
static void nuc980_enable_arp(struct net_device *dev)
{
	unsigned int val;

	val = __raw_readl(REG_MCMDR);
	val |= MCMDR_ARP;
	__raw_writel(val, REG_MCMDR);
}
#endif

static void nuc980_reset_mac(struct net_device *dev)
{
	struct nuc980_ether *ether = netdev_priv(dev);

	ETH_DISABLE_TX;
	ETH_DISABLE_RX;

	nuc980_return_default_idle(dev);
	nuc980_set_fifo_threshold(dev);

	if (!netif_queue_stopped(dev))
		netif_stop_queue(dev);

	nuc980_init_desc(dev);

	ether->cur_tx = 0x0;
	ether->finish_tx = 0x0;
	ether->cur_rx = 0x0;

	nuc980_set_curdest(dev);
	nuc980_enable_cam(dev);
	nuc980_enable_cam_command(dev);
	nuc980_enable_mac_interrupt(dev);

	dev->trans_start = jiffies; /* prevent tx timeout */

	if (netif_queue_stopped(dev))
		netif_wake_queue(dev);
}

static int nuc980_mdio_write(struct mii_bus *bus, int phy_id, int regnum,
		u16 value)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(MII_TIMEOUT * 100);

	__raw_writel(value,  REG_MIID);
	__raw_writel((phy_id << 0x08) | regnum | PHYBUSY | MDCON | PHYWR,  REG_MIIDA);


	/* Wait for completion */
	while (__raw_readl( REG_MIIDA) & PHYBUSY) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		cpu_relax();
	}

	return 0;

}

static int nuc980_mdio_read(struct mii_bus *bus, int phy_id, int regnum)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(MII_TIMEOUT * 100);


	__raw_writel((phy_id << 0x08) | regnum | PHYBUSY | MDCON,  REG_MIIDA);

	/* Wait for completion */
	while (__raw_readl( REG_MIIDA) & PHYBUSY) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		cpu_relax();
	}

	return __raw_readl(REG_MIID);
}

static int nuc980_mdio_reset(struct mii_bus *bus)
{

	// reser EMAC engine??
	return 0;
}

static int nuc980_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *address = addr;

	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, address->sa_data, dev->addr_len);
	nuc980_write_cam(dev, CAM0, dev->dev_addr);

	return 0;
}

static int nuc980_ether_close(struct net_device *dev)
{
	struct nuc980_ether *ether = netdev_priv(dev);
	struct platform_device *pdev;

	pdev = ether->pdev;

	ETH_DISABLE_TX;
	ETH_DISABLE_RX;
	netif_stop_queue(dev);
	napi_disable(&ether->napi);
	free_irq(ether->txirq, dev);
	free_irq(ether->rxirq, dev);

	nuc980_return_default_idle(dev);
	nuc980_free_desc(dev);

	if (ether->phy_dev)
		phy_stop(ether->phy_dev);

	nuc980_release_ptp(ether);

	return 0;
}

static struct net_device_stats *nuc980_ether_stats(struct net_device *dev)
{
	struct nuc980_ether *ether;

	ether = netdev_priv(dev);

	return &ether->stats;
}


static int nuc980_ether_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct nuc980_ether *ether = netdev_priv(dev);
	struct nuc980_txbd *txbd;

	txbd = ether->tdesc + ether->cur_tx;
	if(txbd->mode & TX_OWEN_DMA) {
		netif_stop_queue(dev);
		return NETDEV_TX_BUSY;
	}

	txbd->buffer = dma_map_single(&dev->dev, skb->data,
					skb->len, DMA_TO_DEVICE);

//	tx_skb[ether->cur_tx]  = skb;
	txbd->sl = skb->len;
	wmb();	// This is dummy function for ARM9
	txbd->mode |= TX_OWEN_DMA;
	wmb();	// This is dummy function for ARM9
	tx_skb[ether->cur_tx]  = skb;

	if (unlikely((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
		     ether->hwts_tx_en)) {		     	
		/* declare that device is doing timestamping */
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		txbd->mode |= TTSEN;
	}

	if (!ether->hwts_tx_en)
	{
		skb_tx_timestamp(skb);		
	}

	ETH_TRIGGER_TX;

	if (++ether->cur_tx >= TX_DESC_SIZE)
		ether->cur_tx = 0;
	txbd = ether->tdesc + ether->cur_tx;
	if(txbd->mode & TX_OWEN_DMA) {
		netif_stop_queue(dev);
		//return NETDEV_TX_BUSY;
	}
	return NETDEV_TX_OK;
}

static irqreturn_t nuc980_tx_interrupt(int irq, void *dev_id)
{
	struct nuc980_ether *ether;
	struct platform_device *pdev;
	struct net_device *dev;
	unsigned int status;
	struct sk_buff *s;
	struct nuc980_txbd *txbd;

	dev = dev_id;
	ether = netdev_priv(dev);
	pdev = ether->pdev;

	nuc980_get_and_clear_int(dev, &status, 0xFFFF0000);

	txbd = ether->tdesc + ether->finish_tx;
	while((txbd->mode & TX_OWEN_DMA) != TX_OWEN_DMA) {
		if((s = tx_skb[ether->finish_tx]) != NULL) {
			if(txbd->mode & TTSEN) // get ts & recover next tx addr
			{
				nuc980_get_tx_hwtstamp(ether,txbd,s);
				txbd->next = txbd_next[ether->finish_tx];
			}
			dma_unmap_single(&dev->dev, txbd->buffer, s->len, DMA_TO_DEVICE);
			dev_kfree_skb_irq(s);
			tx_skb[ether->finish_tx] = NULL;
			if (txbd->sl & TXDS_TXCP) {
				ether->stats.tx_packets++;
				ether->stats.tx_bytes += (txbd->sl & 0xFFFF);
			} else {
				ether->stats.tx_errors++;
			}
		} else
			break;
		ether->finish_tx = (ether->finish_tx + 1) % TX_DESC_SIZE;
		txbd = ether->tdesc + ether->finish_tx;
	}

	if (status & MISTA_EXDEF) {
		dev_err(&pdev->dev, "emc defer exceed interrupt\n");
	} else if (status & MISTA_TXBERR) {
		dev_err(&pdev->dev, "emc bus error interrupt\n");
		BUG();
	}

	if (netif_queue_stopped(dev)) {
		netif_wake_queue(dev);
	}

	return IRQ_HANDLED;
}

static int nuc980_poll(struct napi_struct *napi, int budget)
{
	struct nuc980_ether *ether = container_of(napi, struct nuc980_ether, napi);
	struct nuc980_rxbd *rxbd;
	struct net_device *dev = ether->ndev;
	struct sk_buff *skb, *s;
	unsigned int length, status;
	int rx_cnt = 0;
	int complete = 0;

	rxbd = (ether->rdesc + ether->cur_rx);

	while(rx_cnt < budget) {

		if((rxbd->sl & RX_OWEN_DMA) == RX_OWEN_DMA) {
			complete = 1;
			break;
		}

		s = rx_skb[ether->cur_rx];
		status = rxbd->sl;
		length = status & 0xFFFF;

		if (likely((status & RXDS_RXGD) && 
#if (IS_VLAN == 1)
		(length <= MAX_PACKET_SIZE))) {
#else
		(length <= 1514))) {
#endif
			skb = dev_alloc_skb(MAX_PACKET_SIZE);
			if (!skb) {
				struct platform_device *pdev = ether->pdev;
				dev_err(&pdev->dev, "get skb buffer error\n");
				ether->stats.rx_dropped++;
				goto rx_out;
			}
			dma_unmap_single(&dev->dev, (dma_addr_t)rxbd->buffer, MAX_PACKET_SIZE, DMA_FROM_DEVICE);

			skb_put(s, length);
			s->protocol = eth_type_trans(s, dev);

			if (status & RXDS_RTSAS) // get ts & recover next rx addr
			{
				nuc980_get_rx_hwtstamp(ether, rxbd, s);
				rxbd->next = rxbd_next[ether->cur_rx];
			}

			netif_receive_skb(s);
			ether->stats.rx_packets++;
			ether->stats.rx_bytes += length;
			skb_reserve(skb, 2);
			skb->dev = dev;

			rxbd->buffer = dma_map_single(&dev->dev, skb->data,
							MAX_PACKET_SIZE, DMA_FROM_DEVICE);

			rx_skb[ether->cur_rx] = skb;
			rx_cnt++;

		} else {
			ether->stats.rx_errors++;

			if (status & RXDS_RP) {
				ether->stats.rx_length_errors++;
			} else if (status & RXDS_CRCE) {
				ether->stats.rx_crc_errors++;
			} else if (status & RXDS_ALIE) {
				ether->stats.rx_frame_errors++;
			} else if (status & RXDS_PTLE) {
				ether->stats.rx_over_errors++;
			}
		}

		wmb();	// This is dummy function for ARM9
		rxbd->sl = RX_OWEN_DMA;

		if (++ether->cur_rx >= RX_DESC_SIZE)
			ether->cur_rx = 0;

		rxbd = (ether->rdesc + ether->cur_rx);

	}

	if(complete) {
		napi_complete(napi);
		__raw_writel(__raw_readl(REG_MIEN) | ENRXINTR,  REG_MIEN);
	}

rx_out:

	ETH_TRIGGER_RX;
	return(rx_cnt);
}

static irqreturn_t nuc980_rx_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct nuc980_ether *ether = netdev_priv(dev);
	unsigned int status;

	nuc980_get_and_clear_int(dev, &status, 0xFFFF);

	if (unlikely(status & MISTA_RXBERR)) {
		struct platform_device *pdev = ether->pdev;

		dev_err(&pdev->dev, "emc rx bus error\n");
		BUG();

	} else {
		if(status & MISTA_WOL) {

		}

		if(status & MISTA_RXGD) {
			__raw_writel(__raw_readl(REG_MIEN) & ~ENRXINTR,  REG_MIEN);
			napi_schedule(&ether->napi);
		}
	}
	return IRQ_HANDLED;
}


static int nuc980_ether_open(struct net_device *dev)
{
	struct nuc980_ether *ether = netdev_priv(dev);
	struct platform_device *pdev = ether->pdev;

	nuc980_reset_mac(dev);
	nuc980_set_global_maccmd(dev);

	if (request_irq(ether->txirq, nuc980_tx_interrupt,
						0x0, pdev->name, dev)) {
		dev_err(&pdev->dev, "register irq tx failed\n");
		return -EAGAIN;
	}

	if (request_irq(ether->rxirq, nuc980_rx_interrupt,
						IRQF_NO_SUSPEND, pdev->name, dev)) {
		dev_err(&pdev->dev, "register irq rx failed\n");
		free_irq(ether->txirq, dev);
		return -EAGAIN;
	}

	if (nuc980_init_ptp(ether))
		netdev_warn(dev, "PTP init failed\n");

	phy_start(ether->phy_dev);
	netif_start_queue(dev);
	napi_enable(&ether->napi);

	ETH_ENABLE_RX;

	dev_info(&pdev->dev, "%s is OPENED\n", dev->name);

	return 0;
}

static void nuc980_ether_set_multicast_list(struct net_device *dev)
{
	struct nuc980_ether *ether;
	unsigned int rx_mode;

	ether = netdev_priv(dev);

	if (dev->flags & IFF_PROMISC)
		rx_mode = CAMCMR_AUP | CAMCMR_AMP | CAMCMR_ABP | CAMCMR_ECMP;
	else if ((dev->flags & IFF_ALLMULTI) || !netdev_mc_empty(dev))
		rx_mode = CAMCMR_AMP | CAMCMR_ABP | CAMCMR_ECMP;
	else
		rx_mode = CAMCMR_ECMP | CAMCMR_ABP;
	__raw_writel(rx_mode,  REG_CAMCMR);
}



static int nuc980_ether_ioctl(struct net_device *dev,
						struct ifreq *ifr, int cmd)
{
	struct nuc980_ether *ether = netdev_priv(dev);
	struct phy_device *phydev = ether->phy_dev;

	if (!netif_running(dev))
		return -EINVAL;

	if (!phydev)
		return -ENODEV;

	if(cmd==SIOCSHWTSTAMP)
		return nuc980_hwtstamp_ioctl(dev, ifr);

	return phy_mii_ioctl(phydev, ifr, cmd);
}

static void nuc980_get_drvinfo(struct net_device *dev,
					struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_MODULE_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_MODULE_VERSION, sizeof(info->version));
	strlcpy(info->fw_version, "N/A", sizeof(info->fw_version));
	strlcpy(info->bus_info, "N/A", sizeof(info->bus_info));
}

static int nuc980_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct nuc980_ether *ether = netdev_priv(dev);
	struct phy_device *phydev = ether->phy_dev;

	if (NULL == phydev)
		return -ENODEV;

	return phy_ethtool_gset(phydev, cmd);
}

static int nuc980_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct nuc980_ether *ether = netdev_priv(dev);
	struct phy_device *phydev = ether->phy_dev;

	if (NULL == phydev)
		return -ENODEV;

	return phy_ethtool_sset(phydev, cmd);
}

static u32 nuc980_get_msglevel(struct net_device *dev)
{
	struct nuc980_ether *ether = netdev_priv(dev);

	return ether->msg_enable;
}

static void nuc980_set_msglevel(struct net_device *dev, u32 level)
{
	struct nuc980_ether *ether = netdev_priv(dev);

	ether->msg_enable = level;
}

static int nuc980_get_eee(struct net_device *dev, struct ethtool_eee *edata)
{
	return -EOPNOTSUPP;
}

static int nuc980_set_eee(struct net_device *dev, struct ethtool_eee *edata)
{
	return -EOPNOTSUPP;
}

static int nuc980_get_regs_len(struct net_device *dev)
{
	return 76 * sizeof(u32);
}

static void nuc980_get_regs(struct net_device *dev, struct ethtool_regs *regs, void *p)
{

	regs->version = 0;
	memcpy(p, REG_CAMCMR, 76 * sizeof(u32));
}

#ifdef CONFIG_PM
static void nuc980_get_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct nuc980_ether *ether = netdev_priv(dev);

	wol->supported = WAKE_MAGIC;
	wol->wolopts = ether->wol ? WAKE_MAGIC : 0;

}

static int nuc980_set_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct nuc980_ether *ether = netdev_priv(dev);

	if (wol->wolopts & ~WAKE_MAGIC)
		return -EINVAL;

	ether->wol = wol->wolopts & WAKE_MAGIC ? 1 : 0;

	device_set_wakeup_capable(&dev->dev, wol->wolopts & WAKE_MAGIC);
	device_set_wakeup_enable(&dev->dev, wol->wolopts & WAKE_MAGIC);

	return 0;
}
#endif

static int nuc980_get_ts_info(struct net_device *dev, struct ethtool_ts_info *info)
{
	info->so_timestamping = SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE;
	info->phc_index = 0;
	info->tx_types = (1 << HWTSTAMP_TX_OFF) |
			 (1 << HWTSTAMP_TX_ON);
	info->rx_filters = (1 << HWTSTAMP_FILTER_NONE) |
			   (1 << HWTSTAMP_FILTER_ALL);
	return 0;
}

static int nuc980_change_mtu(struct net_device *dev, int new_mtu)
{
	if(new_mtu < 64 || new_mtu > MAX_PACKET_SIZE)
		return -EINVAL;

	dev->mtu = new_mtu;

	if(new_mtu < 1518)
		nuc980_set_alp(dev, false);
	else
		nuc980_set_alp(dev, true);

	return 0;
}

static const struct ethtool_ops nuc980_ether_ethtool_ops = {
	.get_settings	= nuc980_get_settings,
	.set_settings	= nuc980_set_settings,
	.get_drvinfo	= nuc980_get_drvinfo,
	.get_msglevel	= nuc980_get_msglevel,
	.set_msglevel	= nuc980_set_msglevel,
	.get_link 	= ethtool_op_get_link,
	.get_eee	= nuc980_get_eee,
	.set_eee	= nuc980_set_eee,
	.get_regs_len	= nuc980_get_regs_len,
	.get_regs	= nuc980_get_regs,
#ifdef CONFIG_PM
	.get_wol 	= nuc980_get_wol,
	.set_wol 	= nuc980_set_wol,
#endif
	.get_ts_info	= nuc980_get_ts_info,
};

static const struct net_device_ops nuc980_ether_netdev_ops = {
	.ndo_open		= nuc980_ether_open,
	.ndo_stop		= nuc980_ether_close,
	.ndo_start_xmit		= nuc980_ether_start_xmit,
	.ndo_get_stats		= nuc980_ether_stats,
	.ndo_set_rx_mode	= nuc980_ether_set_multicast_list,
	.ndo_set_mac_address	= nuc980_set_mac_address,
	.ndo_do_ioctl		= nuc980_ether_ioctl,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= nuc980_change_mtu,
};

static void __init get_mac_address(struct net_device *dev)
{
	struct nuc980_ether *ether = netdev_priv(dev);
	struct platform_device *pdev;

	pdev = ether->pdev;

	if (is_valid_ether_addr(nuc980_mac1))
		memcpy(dev->dev_addr, &nuc980_mac1[0], 0x06);
	else
		dev_err(&pdev->dev, "invalid mac address\n");
}


static int nuc980_mii_setup(struct net_device *dev)
{
	struct nuc980_ether *ether = netdev_priv(dev);
	struct platform_device *pdev;
	struct phy_device *phydev;
	int i, err = 0;

	pdev = ether->pdev;

	ether->mii_bus = mdiobus_alloc();
	if (!ether->mii_bus) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "mdiobus_alloc() failed\n");
		goto out0;
	}

	ether->mii_bus->name = "nuc980_rmii1";
	ether->mii_bus->read = &nuc980_mdio_read;
	ether->mii_bus->write = &nuc980_mdio_write;
	ether->mii_bus->reset = &nuc980_mdio_reset;
	snprintf(ether->mii_bus->id, MII_BUS_ID_SIZE, "%s-%x",
		 ether->pdev->name, ether->pdev->id);
	ether->mii_bus->priv = ether;
	ether->mii_bus->parent = &ether->pdev->dev;

	ether->mii_bus->irq = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (!ether->mii_bus->irq) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "kmalloc() failed\n");
		goto out1;

	}

	for (i = 0; i < PHY_MAX_ADDR; i++)
		ether->mii_bus->irq[i] = PHY_POLL;
	//ether->mii_bus->irq[1] = ??   write me after the irq number is known

	if (mdiobus_register(ether->mii_bus)) {
		dev_err(&pdev->dev, "mdiobus_register() failed\n");
		goto out2;
	}

	phydev = phy_find_first(ether->mii_bus);
	if(phydev == NULL) {
		err = -ENODEV;
		dev_err(&pdev->dev, "phy_find_first() failed\n");
		goto out3;
	}

	phydev = phy_connect(dev, dev_name(&phydev->dev),
			     &adjust_link,
			     PHY_INTERFACE_MODE_RMII);

	if(IS_ERR(phydev)) {
		err = PTR_ERR(phydev);
		dev_err(&pdev->dev, "phy_connect() failed\n");
		goto out3;
	}

	phydev->supported &= PHY_BASIC_FEATURES;
	phydev->advertising = phydev->supported;
	ether->phy_dev = phydev;
	ether->wol = 0;

	return 0;

out3:
	mdiobus_unregister(ether->mii_bus);
out2:
	kfree(ether->mii_bus->irq);
out1:
	mdiobus_free(ether->mii_bus);
out0:

	return err;
}

static int nuc980_ether_probe(struct platform_device *pdev)
{
	struct nuc980_ether *ether;
	struct net_device *dev;
	int error;
	struct pinctrl *pinctrl;

	dev = alloc_etherdev(sizeof(struct nuc980_ether));
	if (!dev)
		return -ENOMEM;

	ether = netdev_priv(dev);

	ether->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (ether->res == NULL) {
		dev_err(&pdev->dev, "failed to get I/O memory\n");
		error = -ENXIO;
		goto err0;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		return PTR_ERR(pinctrl);
	}

	ether->txirq = platform_get_irq(pdev, 0);
	if (ether->txirq < 0) {
		dev_err(&pdev->dev, "failed to get ether tx irq\n");
		error = -ENXIO;
		goto err0;
	}

	ether->rxirq = platform_get_irq(pdev, 1);
	if (ether->rxirq < 0) {
		dev_err(&pdev->dev, "failed to get ether rx irq\n");
		error = -ENXIO;
		goto err0;
	}

	SET_NETDEV_DEV(dev, &pdev->dev);
	platform_set_drvdata(pdev, dev);
	ether->ndev = dev;

	ether->eclk = clk_get(NULL, "emac1_eclk");
	if (IS_ERR(ether->eclk)) {
		dev_err(&pdev->dev, "failed to get emac1_eclk clock\n");
		error = PTR_ERR(ether->eclk);
		goto err1;
	}

	// Set MDC to 1M
	clk_set_rate(ether->eclk, 1000000);

	clk_prepare(ether->eclk);
	clk_enable(ether->eclk);

	ether->clk = clk_get(NULL, "emac1_hclk");
	if (IS_ERR(ether->clk)) {
		dev_err(&pdev->dev, "failed to get emac1_hclk clock\n");
		error = PTR_ERR(ether->clk);
		goto err1;
	}

	clk_prepare(ether->clk);
	clk_enable(ether->clk);

	ether->pdev = pdev;
	ether->msg_enable = NETIF_MSG_LINK;

	dev->netdev_ops = &nuc980_ether_netdev_ops;
	dev->ethtool_ops = &nuc980_ether_ethtool_ops;

	dev->tx_queue_len = 32;
	dev->dma = 0x0;
	dev->watchdog_timeo = TX_TIMEOUT;

	get_mac_address(dev);

	ether->cur_tx = 0x0;
	ether->cur_rx = 0x0;
	ether->finish_tx = 0x0;
	ether->link = 0;
	ether->speed = 100;
	ether->duplex = DUPLEX_FULL;
	spin_lock_init(&ether->lock);

	netif_napi_add(dev, &ether->napi, nuc980_poll, 32);

	ether_setup(dev);

	if((error = nuc980_mii_setup(dev)) < 0) {
		dev_err(&pdev->dev, "nuc980_mii_setup err\n");
		goto err2;
	}
	netif_carrier_off(dev);
	error = register_netdev(dev);
	if (error != 0) {
		dev_err(&pdev->dev, "register_netdev() failed\n");
		error = -ENODEV;
		goto err2;
	}

	return 0;

err2:
	clk_disable(ether->clk);
	clk_put(ether->clk);
err1:
	platform_set_drvdata(pdev, NULL);
err0:
	free_netdev(dev);

	return error;
}

static int nuc980_ether_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct nuc980_ether *ether = netdev_priv(dev);

	unregister_netdev(dev);

	clk_disable(ether->clk);
	clk_put(ether->clk);

	clk_disable(ether->eclk);
	clk_put(ether->eclk);

	phy_disconnect(ether->phy_dev);

	mdiobus_unregister(ether->mii_bus);
	kfree(ether->mii_bus->irq);
	mdiobus_free(ether->mii_bus);

	platform_set_drvdata(pdev, NULL);

	free_netdev(dev);
	return 0;
}

#ifdef CONFIG_PM
static int nuc980_ether_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct nuc980_ether *ether = netdev_priv(dev);

	netif_device_detach(dev);

	if(netif_running(dev)) {
		ETH_DISABLE_TX;
		ETH_DISABLE_RX;

		napi_disable(&ether->napi);

		if(ether->wol) {  // enable wakeup from magic packet
			__raw_writel(__raw_readl(REG_MCMDR) | MCMDR_MGPWAKE, REG_MCMDR);
			__raw_writel(__raw_readl(REG_WKUPSER1) | (1 << 17), REG_WKUPSER1);
		} else {
			phy_stop(ether->phy_dev);
		}

	}

	return 0;

}

static int nuc980_ether_resume(struct platform_device *pdev)
{

	struct net_device *dev = platform_get_drvdata(pdev);
	struct nuc980_ether *ether = netdev_priv(dev);

	if (netif_running(dev)) {

		if(ether->wol) {  // enable wakeup from magic packet
			__raw_writel(__raw_readl(REG_WKUPSER1) & ~(1 << 17), REG_WKUPSER1);
			__raw_writel(__raw_readl(REG_MCMDR) & ~MCMDR_MGPWAKE, REG_MCMDR);
		} else {

			phy_start(ether->phy_dev);
		}

		napi_enable(&ether->napi);

		ETH_ENABLE_TX;
		ETH_ENABLE_RX;

	}

	netif_device_attach(dev);
	return 0;

}

#else
#define nuc980_ether_suspend NULL
#define nuc980_ether_resume NULL
#endif

static const struct of_device_id nuc980_emac1_of_match[] = {
	{ .compatible = "nuvoton,nuc980-emac1" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_emac1_of_match);

static struct platform_driver nuc980_ether_driver = {
	.probe		= nuc980_ether_probe,
	.remove		= nuc980_ether_remove,
	.suspend 	= nuc980_ether_suspend,
	.resume 	= nuc980_ether_resume,
	.driver		= {
		.name	= "nuc980-emac1",
		.of_match_table = of_match_ptr(nuc980_emac1_of_match),
		.owner	= THIS_MODULE,
	},
};

static int __init nuc980_ether_init(void)
{

	return platform_driver_register(&nuc980_ether_driver);
}

static void __exit nuc980_ether_exit(void)
{
	platform_driver_unregister(&nuc980_ether_driver);
}

module_init(nuc980_ether_init);
module_exit(nuc980_ether_exit);

MODULE_AUTHOR("Nuvoton Technology Corp.");
MODULE_DESCRIPTION("NUC980 MAC1 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc980-emac1");
