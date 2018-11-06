/*
 * arch/arm/mach-nuc980/include/mach/regs-cap.h
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
#ifndef __ASM_ARCH_REGS_CAP_H
#define __ASM_ARCH_REGS_CAP_H

#define BIT0    0x00000001
#define BIT1    0x00000002
#define BIT2    0x00000004
#define BIT3    0x00000008
#define BIT4    0x00000010
#define BIT5    0x00000020
#define BIT6    0x00000040
#define BIT7    0x00000080
#define BIT8    0x00000100
#define BIT9    0x00000200
#define BIT10   0x00000400
#define BIT11   0x00000800
#define BIT12   0x00001000
#define BIT13   0x00002000
#define BIT14   0x00004000
#define BIT15   0x00008000
#define BIT16   0x00010000
#define BIT17   0x00020000
#define BIT18   0x00040000
#define BIT19   0x00080000
#define BIT20   0x00100000
#define BIT21   0x00200000
#define BIT22   0x00400000
#define BIT23   0x00800000
#define BIT24   0x01000000
#define BIT25   0x02000000
#define BIT26   0x04000000
#define BIT27   0x08000000
#define BIT28   0x10000000
#define BIT29   0x20000000
#define BIT30   0x40000000
#define BIT31   0x80000000

#include <mach/map.h>

#define CAP0_BA     NUC980_VA_CAP0  /* Videoin Control */
#define CAP1_BA     NUC980_VA_CAP1  /* Videoin Control */
/*
 VideoIn Control Registers
*/
#define REG_CAP0_CTL                (CAP0_BA + 0x00)    /* Image Capture Interface Control Register */
#define REG_CAP0_PAR                (CAP0_BA + 0x04)    /* Image Capture Interface Parameter Register */
#define REG_CAP0_INT                (CAP0_BA + 0x08)    /* Image Capture Interface Interrupt Register */
#define REG_CAP0_POSTERIZE  (CAP0_BA + 0x0C)    /* YUV Component Posterizing Factor Register */
#define REG_CAP0_MD                 (CAP0_BA + 0x10)    /* Motion Detection Register */
#define REG_CAP0_MDADDR         (CAP0_BA + 0x14)    /* Motion Detection Output Address Register */
#define REG_CAP0_MDYADDR        (CAP0_BA + 0x18)    /* Motion Detection Temp Y Output Address Register */
#define REG_CAP0_SEPIA          (CAP0_BA + 0X1C)    /* Sepia Effect Control Register */
#define REG_CAP0_CWSP               (CAP0_BA + 0X20)    /* Cropping Window Starting Address Register */
#define REG_CAP0_CWS                (CAP0_BA + 0X24)    /* Cropping Window Size Register */
#define REG_CAP0_PKTSL          (CAP0_BA + 0X28)    /* Packet Scaling Vertical/Horizontal Factor Register (LSB) */
#define REG_CAP0_PLNSL          (CAP0_BA + 0X2C)    /* Planar Scaling Vertical/Horizontal Factor Register (LSB) */
#define REG_CAP0_FRCTL          (CAP0_BA + 0X30)    /* Scaling Frame Rate Factor Register */
#define REG_CAP0_STRIDE         (CAP0_BA + 0X34)    /* Frame Output Pixel Stride Width Register */
#define REG_CAP0_FIFOTH         (CAP0_BA + 0X3C)    /* FIFO Threshold Register */
#define REG_CAP0_CMPADDR        (CAP0_BA + 0X40)    /* Compare Memory Base Address Register */
#define REG_CAP0_PKTSM          (CAP0_BA + 0X48)    /* Packet Scaling Vertical/Horizontal Factor Register (MSB) */
#define REG_CAP0_PLNSM          (CAP0_BA + 0X4C)    /* Planar Scaling Vertical/Horizontal Factor Register (MSB) */
#define REG_CAP0_CURADDRP       (CAP0_BA + 0X50)    /* Current Packet System Memory Address Register */
#define REG_CAP0_CURADDRY       (CAP0_BA + 0X54)    /* Current Planar Y System Memory Address Register */
#define REG_CAP0_CURADDRU       (CAP0_BA + 0X58)    /* Current Planar U System Memory Address Register */
#define REG_CAP0_CURADDRV       (CAP0_BA + 0X5C)    /* Current Planar V System Memory Address Register */
#define REG_CAP0_PKTBA0         (CAP0_BA + 0X60)    /* System Memory Packet Base Address 0 Register */
#define REG_CAP0_PKTBA1         (CAP0_BA + 0X64)    /* System Memory Packet Base Address 1 Register */
#define REG_CAP0_YBA                (CAP0_BA + 0X80)    /* System Memory Planar Y Base Address Register */
#define REG_CAP0_UBA                (CAP0_BA + 0X84)    /* System Memory Planar U Base Address Register */
#define REG_CAP0_VBA                (CAP0_BA + 0X88)    /* System Memory Planar V Base Address Register */

#define REG_CAP1_CTL                (CAP1_BA + 0x00)    /* Image Capture Interface Control Register */
#define REG_CAP1_PAR                (CAP1_BA + 0x04)    /* Image Capture Interface Parameter Register */
#define REG_CAP1_INT                (CAP1_BA + 0x08)    /* Image Capture Interface Interrupt Register */
#define REG_CAP1_POSTERIZE  (CAP1_BA + 0x0C)    /* YUV Component Posterizing Factor Register */
#define REG_CAP1_MD                 (CAP1_BA + 0x10)    /* Motion Detection Register */
#define REG_CAP1_MDADDR         (CAP1_BA + 0x14)    /* Motion Detection Output Address Register */
#define REG_CAP1_MDYADDR        (CAP1_BA + 0x18)    /* Motion Detection Temp Y Output Address Register */
#define REG_CAP1_SEPIA          (CAP1_BA + 0X1C)    /* Sepia Effect Control Register */
#define REG_CAP1_CWSP               (CAP1_BA + 0X20)    /* Cropping Window Starting Address Register */
#define REG_CAP1_CWS                (CAP1_BA + 0X24)    /* Cropping Window Size Register */
#define REG_CAP1_PKTSL          (CAP1_BA + 0X28)    /* Packet Scaling Vertical/Horizontal Factor Register (LSB) */
#define REG_CAP1_PLNSL          (CAP1_BA + 0X2C)    /* Planar Scaling Vertical/Horizontal Factor Register (LSB) */
#define REG_CAP1_FRCTL          (CAP1_BA + 0X30)    /* Scaling Frame Rate Factor Register */
#define REG_CAP1_STRIDE         (CAP1_BA + 0X34)    /* Frame Output Pixel Stride Width Register */
#define REG_CAP1_FIFOTH         (CAP1_BA + 0X3C)    /* FIFO Threshold Register */
#define REG_CAP1_CMPADDR        (CAP1_BA + 0X40)    /* Compare Memory Base Address Register */
#define REG_CAP1_PKTSM          (CAP1_BA + 0X48)    /* Packet Scaling Vertical/Horizontal Factor Register (MSB) */
#define REG_CAP1_PLNSM          (CAP1_BA + 0X4C)    /* Planar Scaling Vertical/Horizontal Factor Register (MSB) */
#define REG_CAP1_CURADDRP       (CAP1_BA + 0X50)    /* Current Packet System Memory Address Register */
#define REG_CAP1_CURADDRY       (CAP1_BA + 0X54)    /* Current Planar Y System Memory Address Register */
#define REG_CAP1_CURADDRU       (CAP1_BA + 0X58)    /* Current Planar U System Memory Address Register */
#define REG_CAP1_CURADDRV       (CAP1_BA + 0X5C)    /* Current Planar V System Memory Address Register */
#define REG_CAP1_PKTBA0         (CAP1_BA + 0X60)    /* System Memory Packet Base Address 0 Register */
#define REG_CAP1_PKTBA1         (CAP1_BA + 0X64)    /* System Memory Packet Base Address 1 Register */
#define REG_CAP1_YBA                (CAP1_BA + 0X80)    /* System Memory Planar Y Base Address Register */
#define REG_CAP1_UBA                (CAP1_BA + 0X84)    /* System Memory Planar U Base Address Register */
#define REG_CAP1_VBA                (CAP1_BA + 0X88)    /* System Memory Planar V Base Address Register */

#define CAP_CTL_CAPEN           (1 << 0)
#define CAP_CTL_PLNEN           (1 << 5)
#define CAP_CTL_PKTEN           (1 << 6)
#define CAP_CTL_UPDATE      (1 << 20)

#define CAP_CWSP_CWSADDRV   (0x3FF << 16)
#define CAP_CWSP_CWSADDRH   (0x3FF << 0)

#define CAP_PKTSL_PKTSVNL   (0xFF << 24)
#define CAP_PKTSL_PKTSVML   (0xFF << 16)
#define CAP_PKTSL_PKTSHNL   (0xFF << 8)
#define CAP_PKTSL_PKTSHML   (0xFF << 0)

#define CAP_PKTSM_PKTSVNH   (0xFF << 24)
#define CAP_PKTSM_PKTSVMH   (0xFF << 16)
#define CAP_PKTSM_PKTSHNH   (0xFF << 8)
#define CAP_PKTSM_PKTSHMH   (0xFF << 0)

#define CAP_STRIDE_PLNSTRIDE    (0x3FFF << 16)
#define CAP_STRIDE_PKTSTRIDE    (0x3FFF << 0)

#define CAP_PLNSL_PLNSVNL   (0xFF << 24)
#define CAP_PLNSL_PLNSVML   (0xFF << 16)
#define CAP_PLNSL_PLNSHNL   (0xFF << 8)
#define CAP_PLNSL_PLNSHML   (0xFF << 0)
#define CAP_PLNSM_PLNSVNH   (0xFF << 24)
#define CAP_PLNSM_PLNSVMH   (0xFF << 16)
#define CAP_PLNSM_PLNSHNH   (0xFF << 8)
#define CAP_PLNSM_PLNSHMH   (0xFF << 0)

#endif /*  __ASM_ARCH_REGS_CAP_H */
