/* linux/arch/arm/mach-nuc980/include/mach/regs-usbd.h
 *
 * Copyright (c) 2018 Nuvoton technology corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef __NUC980_USBD_H
#define __NUC980_USBD_H

#define USBD_DMA_LEN		0x10000
#define USB_HIGHSPEED		2
#define USB_FULLSPEED		1

#define DMA_READ		1
#define DMA_WRITE		2

#define REG_USBD_GINTSTS		(0x000)
#define REG_USBD_GINTEN			(0x008)
#define REG_USBD_BUSINTSTS		(0x010)
#define REG_USBD_BUSINTEN		(0x014)
#define REG_USBD_OPER			(0x018)
#define REG_USBD_FRAMECNT		(0x01C)
#define REG_USBD_FADDR			(0x020)
#define REG_USBD_TEST			(0x024)
#define REG_USBD_CEPDAT			(0x028)
#define REG_USBD_CEPCTL			(0x02C)
#define REG_USBD_CEPINTEN		(0x030)
#define REG_USBD_CEPINTSTS		(0x034)
#define REG_USBD_CEPTXCNT		(0x038)
#define REG_USBD_CEPRXCNT		(0x03C)
#define REG_USBD_CEPDATCNT		(0x040)
#define REG_USBD_SETUP1_0		(0x044)
#define REG_USBD_SETUP3_2		(0x048)
#define REG_USBD_SETUP5_4		(0x04C)
#define REG_USBD_SETUP7_6		(0x050)
#define REG_USBD_CEPBUFSTART	(0x054)
#define REG_USBD_CEPBUFEND		(0x058)
#define REG_USBD_DMACTL			(0x05C)
#define REG_USBD_DMACNT			(0x060)

#define REG_USBD_EPA_EPDAT		(0x064)
#define REG_USBD_EPA_EPINTSTS	(0x068)
#define REG_USBD_EPA_EPINTEN	(0x06C)
#define REG_USBD_EPA_EPDATCNT	(0x070)
#define REG_USBD_EPA_EPRSPCTL	(0x074)
#define REG_USBD_EPA_EPMPS		(0x078)
#define REG_USBD_EPA_EPTXCNT	(0x07C)
#define REG_USBD_EPA_EPCFG		(0x080)
#define REG_USBD_EPA_EPBUFSTART	(0x084)
#define REG_USBD_EPA_EPBUFEND	(0x088)

#define REG_USBD_DMAADDR		(0x700)
#define REG_USBD_PHYCTL			(0x704)


#define USBD_GINTSTS_USBIF         (0x1ul << 0)
#define USBD_GINTSTS_CEPIF         (0x1ul << 1)
#define USBD_GINTSTS_EPIF          (0xffful << 2)

#define USBD_BUSINTSTS_SOFIF         (0x1ul << 0)
#define USBD_BUSINTSTS_RSTIF         (0x1ul << 1)
#define USBD_BUSINTSTS_RESUMEIF      (0x1ul << 2)
#define USBD_BUSINTSTS_SUSPENDIF     (0x1ul << 3)
#define USBD_BUSINTSTS_HISPDIF       (0x1ul << 4)
#define USBD_BUSINTSTS_DMADONEIF     (0x1ul << 5)
#define USBD_BUSINTSTS_PHYCLKVLDIF   (0x1ul << 6)
#define USBD_BUSINTSTS_VBUSDETIF     (0x1ul << 8)

#define USBD_BUSINTEN_SOFIEN         (0x1ul << 0)
#define USBD_BUSINTEN_RSTIEN         (0x1ul << 1)
#define USBD_BUSINTEN_RESUMEIEN      (0x1ul << 2)
#define USBD_BUSINTEN_SUSPENDIEN     (0x1ul << 3)
#define USBD_BUSINTEN_HISPDIEN       (0x1ul << 4)
#define USBD_BUSINTEN_DMADONEIEN     (0x1ul << 5)
#define USBD_BUSINTEN_PHYCLKVLDIEN   (0x1ul << 6)
#define USBD_BUSINTEN_VBUSDETIEN     (0x1ul << 8)

#define USBD_CEPINTSTS_SETUPTKIF     (0x1ul << 0)
#define USBD_CEPINTSTS_SETUPPKIF     (0x1ul << 1)
#define USBD_CEPINTSTS_OUTTKIF       (0x1ul << 2)
#define USBD_CEPINTSTS_INTKIF        (0x1ul << 3)
#define USBD_CEPINTSTS_PINGIF        (0x1ul << 4)
#define USBD_CEPINTSTS_TXPKIF        (0x1ul << 5)
#define USBD_CEPINTSTS_RXPKIF        (0x1ul << 6)
#define USBD_CEPINTSTS_NAKIF         (0x1ul << 7)
#define USBD_CEPINTSTS_STALLIF       (0x1ul << 8)
#define USBD_CEPINTSTS_ERRIF         (0x1ul << 9)
#define USBD_CEPINTSTS_STSDONEIF     (0x1ul << 10)
#define USBD_CEPINTSTS_BUFFULLIF     (0x1ul << 11)
#define USBD_CEPINTSTS_BUFEMPTYIF    (0x1ul << 12)

#define USBD_CEPINTEN_SETUPTKIEN     (0x1ul << 0)
#define USBD_CEPINTEN_SETUPPKIEN     (0x1ul << 1)
#define USBD_CEPINTEN_OUTTKIEN       (0x1ul << 2)
#define USBD_CEPINTEN_INTKIEN        (0x1ul << 3)
#define USBD_CEPINTEN_PINGIEN        (0x1ul << 4)
#define USBD_CEPINTEN_TXPKIEN        (0x1ul << 5)
#define USBD_CEPINTEN_RXPKIEN        (0x1ul << 6)
#define USBD_CEPINTEN_NAKIEN         (0x1ul << 7)
#define USBD_CEPINTEN_STALLIEN       (0x1ul << 8)
#define USBD_CEPINTEN_ERRIEN         (0x1ul << 9)
#define USBD_CEPINTEN_STSDONEIEN     (0x1ul << 10)
#define USBD_CEPINTEN_BUFFULLIEN     (0x1ul << 11)
#define USBD_CEPINTEN_BUFEMPTYIEN    (0x1ul << 12)

#define USBD_EPINTSTS_BUFFULLIF      (0x1ul << 0)
#define USBD_EPINTSTS_BUFEMPTYIF     (0x1ul << 1)
#define USBD_EPINTSTS_SHORTTXIF      (0x1ul << 2)
#define USBD_EPINTSTS_TXPKIF         (0x1ul << 3)
#define USBD_EPINTSTS_RXPKIF         (0x1ul << 4)
#define USBD_EPINTSTS_OUTTKIF        (0x1ul << 5)
#define USBD_EPINTSTS_INTKIF         (0x1ul << 6)
#define USBD_EPINTSTS_PINGIF         (0x1ul << 7)
#define USBD_EPINTSTS_NAKIF          (0x1ul << 8)
#define USBD_EPINTSTS_STALLIF        (0x1ul << 9)
#define USBD_EPINTSTS_NYETIF         (0x1ul << 10)
#define USBD_EPINTSTS_ERRIF          (0x1ul << 11)
#define USBD_EPINTSTS_SHORTRXIF      (0x1ul << 12)

#define USBD_PHYCTL_DPPUEN           (0x1ul << 8)
#define USBD_PHYCTL_PHYEN            (0x1ul << 9)
#define USBD_PHYCTL_WKEN             (0x1ul << 24)
#define USBD_PHYCTL_VBUSDET          (0x1ul << 31)


/* Define Endpoint feature */
#define Ep_In				0x01
#define Ep_Out				0x00

/********************* Bit definition of CEPCTL register **********************/
#define USB_CEPCTL_NAKCLR               ((uint32_t)0x00000000)      /*!<NAK clear  \hideinitializer */
#define USB_CEPCTL_STALL                ((uint32_t)0x00000002)      /*!<Stall  \hideinitializer */
#define USB_CEPCTL_ZEROLEN              ((uint32_t)0x00000004)      /*!<Zero length packet  \hideinitializer */
#define USB_CEPCTL_FLUSH                ((uint32_t)0x00000008)      /*!<CEP flush  \hideinitializer */

/********************* Bit definition of EPxRSPCTL register **********************/
#define USB_EP_RSPCTL_FLUSH             ((uint32_t)0x00000001)      /*!<Buffer Flush  \hideinitializer */
#define USB_EP_RSPCTL_MODE_AUTO         ((uint32_t)0x00000000)      /*!<Auto-Validate Mode  \hideinitializer */
#define USB_EP_RSPCTL_MODE_MANUAL       ((uint32_t)0x00000002)      /*!<Manual-Validate Mode  \hideinitializer */
#define USB_EP_RSPCTL_MODE_FLY          ((uint32_t)0x00000004)      /*!<Fly Mode  \hideinitializer */
#define USB_EP_RSPCTL_MODE_MASK         ((uint32_t)0x00000006)      /*!<Mode Mask  \hideinitializer */
#define USB_EP_RSPCTL_TOGGLE            ((uint32_t)0x00000008)      /*!<Clear Toggle bit  \hideinitializer */
#define USB_EP_RSPCTL_HALT              ((uint32_t)0x00000010)      /*!<Endpoint halt  \hideinitializer */
#define USB_EP_RSPCTL_ZEROLEN           ((uint32_t)0x00000020)      /*!<Zero length packet IN  \hideinitializer */
#define USB_EP_RSPCTL_SHORTTXEN         ((uint32_t)0x00000040)      /*!<Packet end  \hideinitializer */
#define USB_EP_RSPCTL_DISBUF            ((uint32_t)0x00000080)      /*!<Disable buffer  \hideinitializer */

/********************* Bit definition of EPxCFG register **********************/
#define USB_EP_CFG_VALID                ((uint32_t)0x00000001)      /*!<Endpoint Valid  \hideinitializer */
#define USB_EP_CFG_TYPE_BULK            ((uint32_t)0x00000002)      /*!<Endpoint type - bulk  \hideinitializer */
#define USB_EP_CFG_TYPE_INT             ((uint32_t)0x00000004)      /*!<Endpoint type - interrupt  \hideinitializer */
#define USB_EP_CFG_TYPE_ISO             ((uint32_t)0x00000006)      /*!<Endpoint type - isochronous  \hideinitializer */
#define USB_EP_CFG_TYPE_MASK            ((uint32_t)0x00000006)      /*!<Endpoint type mask  \hideinitializer */
#define USB_EP_CFG_DIR_OUT              ((uint32_t)0x00000000)      /*!<OUT endpoint  \hideinitializer */
#define USB_EP_CFG_DIR_IN               ((uint32_t)0x00000008)      /*!<IN endpoint  \hideinitializer */


#endif /* __NUC980_USBD_H */
