/*
 * arch/arm/mach-nuc980/include/mach/regs-ebi.h
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

#ifndef __ASM_ARCH_REGS_EBI_H
#define __ASM_ARCH_REGS_EBI_H

#include "mach/nuc980-ebi.h"

#define EBI0			NUC980_VA_EBI
#define EBI1			(NUC980_VA_EBI + 0x10)
#define EBI2			(NUC980_VA_EBI + 0x20)

#define REG_EBI_CTL(x)	(void __iomem *)(x)
#define REG_EBI_TCTL(x)	(void __iomem *)(x + 0x04)


#define EBI_BANK0_BASE_ADDR     0x60000000UL /*!< EBI bank0 base address */
#define EBI_BANK1_BASE_ADDR     0x60100000UL /*!< EBI bank1 base address */
#define EBI_BANK2_BASE_ADDR     0x60200000UL /*!< EBI bank2 base address */

#define EBI_BANK0               0UL    /*!< EBI bank 0 */
#define EBI_BANK1               1UL    /*!< EBI bank 1 */
#define EBI_BANK2               2UL    /*!< EBI bank 2 */

#define EBI_BUSWIDTH_8BIT       8UL   /*!< EBI bus width is 8-bit 	*/
#define EBI_BUSWIDTH_16BIT      16UL  /*!< EBI bus width is 16-bit	 */

#define EBI_CS_ACTIVE_LOW       0UL    /*!< EBI CS active level is low  */
#define EBI_CS_ACTIVE_HIGH      1UL    /*!< EBI CS active level is high */

#define EBI_MCLKDIV_1           0x0UL /*!< EBI output clock(MCLK) is HCLK/1 */
#define EBI_MCLKDIV_2           0x1UL /*!< EBI output clock(MCLK) is HCLK/2 */
#define EBI_MCLKDIV_4           0x2UL /*!< EBI output clock(MCLK) is HCLK/4 */
#define EBI_MCLKDIV_8           0x3UL /*!< EBI output clock(MCLK) is HCLK/8 */
#define EBI_MCLKDIV_16          0x4UL /*!< EBI output clock(MCLK) is HCLK/16 */
#define EBI_MCLKDIV_32          0x5UL /*!< EBI output clock(MCLK) is HCLK/32 */
#define EBI_MCLKDIV_64          0x6UL /*!< EBI output clock(MCLK) is HCLK/64 */
#define EBI_MCLKDIV_128         0x7UL /*!< EBI output clock(MCLK) is HCLK/128 */

#define EBI_TIMING_FASTEST      0x0UL /*!< EBI timing is the fastest */
#define EBI_TIMING_VERYFAST     0x1UL /*!< EBI timing is very fast */
#define EBI_TIMING_FAST         0x2UL /*!< EBI timing is fast */
#define EBI_TIMING_NORMAL       0x3UL /*!< EBI timing is normal */
#define EBI_TIMING_SLOW         0x4UL /*!< EBI timing is slow */
#define EBI_TIMING_VERYSLOW     0x5UL /*!< EBI timing is very slow */
#define EBI_TIMING_SLOWEST      0x6UL /*!< EBI timing is the slowest */

#define EBI_OPMODE_NORMAL       0x0UL                 /*!< EBI bus operate in normal mode */
#define EBI_OPMODE_CACCESS      (EBI_CTL_CACCESS_Msk) /*!< EBI bus operate in Continuous Data Access mode */
#define EBI_OPMODE_ADSEPARATE   (EBI_CTL_ADSEPEN_Msk) /*!< EBI bus operate in AD Separate mode */

#define EBI_CTL_EN_Pos                   (0)                                               /*!< EBI_T::CTL: EN Position                */
#define EBI_CTL_EN_Msk                   (0x1ul << EBI_CTL_EN_Pos)                         /*!< EBI_T::CTL: EN Mask                    */

#define EBI_CTL_DW16_Pos                 (1)                                               /*!< EBI_T::CTL: DW16 Position              */
#define EBI_CTL_DW16_Msk                 (0x1ul << EBI_CTL_DW16_Pos)                       /*!< EBI_T::CTL: DW16 Mask                  */

#define EBI_CTL_CSPOLINV_Pos             (2)                                               /*!< EBI_T::CTL: CSPOLINV Position          */
#define EBI_CTL_CSPOLINV_Msk             (0x1ul << EBI_CTL_CSPOLINV_Pos)                   /*!< EBI_T::CTL: CSPOLINV Mask              */

#define EBI_CTL_ADSEPEN_Pos              (3)                                               /*!< EBI_T::CTL: ADSEPEN Position           */
#define EBI_CTL_ADSEPEN_Msk              (0x1ul << EBI_CTL_ADSEPEN_Pos)                    /*!< EBI_T::CTL: ADSEPEN Mask               */

#define EBI_CTL_CACCESS_Pos              (4)                                               /*!< EBI_T::CTL: CACCESS Position           */
#define EBI_CTL_CACCESS_Msk              (0x1ul << EBI_CTL_CACCESS_Pos)                    /*!< EBI_T::CTL: CACCESS Mask               */

#define EBI_CTL_MCLKDIV_Pos              (8)                                               /*!< EBI_T::CTL: MCLKDIV Position           */
#define EBI_CTL_MCLKDIV_Msk              (0x7ul << EBI_CTL_MCLKDIV_Pos)                    /*!< EBI_T::CTL: MCLKDIV Mask               */

#define EBI_CTL_TALE_Pos                 (16)                                              /*!< EBI_T::CTL: TALE Position              */
#define EBI_CTL_TALE_Msk                 (0x7ul << EBI_CTL_TALE_Pos)                       /*!< EBI_T::CTL: TALE Mask                  */

#define EBI_CTL_WBUFEN_Pos               (24)                                              /*!< EBI_T::CTL: WBUFEN Position            */
#define EBI_CTL_WBUFEN_Msk               (0x1ul << EBI_CTL_WBUFEN_Pos)                     /*!< EBI_T::CTL: WBUFEN Mask                */

#define EBI_TCTL_TACC_Pos                (3)                                               /*!< EBI_T::TCTL: TACC Position             */
#define EBI_TCTL_TACC_Msk                (0x1ful << EBI_TCTL_TACC_Pos)                     /*!< EBI_T::TCTL: TACC Mask                 */

#define EBI_TCTL_TAHD_Pos                (8)                                               /*!< EBI_T::TCTL: TAHD Position             */
#define EBI_TCTL_TAHD_Msk                (0x7ul << EBI_TCTL_TAHD_Pos)                      /*!< EBI_T::TCTL: TAHD Mask                 */

#define EBI_TCTL_W2X_Pos                 (12)                                              /*!< EBI_T::TCTL: W2X Position              */
#define EBI_TCTL_W2X_Msk                 (0xful << EBI_TCTL_W2X_Pos)                       /*!< EBI_T::TCTL: W2X Mask                  */

#define EBI_TCTL_RAHDOFF_Pos             (22)                                              /*!< EBI_T::TCTL: RAHDOFF Position          */
#define EBI_TCTL_RAHDOFF_Msk             (0x1ul << EBI_TCTL_RAHDOFF_Pos)                   /*!< EBI_T::TCTL: RAHDOFF Mask              */

#define EBI_TCTL_WAHDOFF_Pos             (23)                                              /*!< EBI_T::TCTL: WAHDOFF Position          */
#define EBI_TCTL_WAHDOFF_Msk             (0x1ul << EBI_TCTL_WAHDOFF_Pos)                   /*!< EBI_T::TCTL: WAHDOFF Mask              */

#define EBI_TCTL_R2R_Pos                 (24)                                              /*!< EBI_T::TCTL: R2R Position              */
#define EBI_TCTL_R2R_Msk                 (0xful << EBI_TCTL_R2R_Pos)                       /*!< EBI_T::TCTL: R2R Mask                  */

static inline void nuc980_set_ebi_ctl(uint32_t u32Bank, uint32_t u32DataWidth, uint32_t u32TimingClass, uint32_t u32BusMode, uint32_t u32CSActiveLevel)
{
	void __iomem *EBIBaseAddr = EBI0;

	switch (u32Bank) {
	case 0:
		EBIBaseAddr = EBI0;
		break;
	case 1:
		EBIBaseAddr = EBI1;
		break;
	case 2:
		EBIBaseAddr = EBI2;
		break;
	}

	if(u32DataWidth == EBI_BUSWIDTH_8BIT) {
		__raw_writel(__raw_readl(REG_EBI_CTL(EBIBaseAddr)) & ~EBI_CTL_DW16_Msk, REG_EBI_CTL(EBIBaseAddr));
	} else {
		__raw_writel(__raw_readl(REG_EBI_CTL(EBIBaseAddr)) | EBI_CTL_DW16_Msk, REG_EBI_CTL(EBIBaseAddr));
	}

	if(u32CSActiveLevel == EBI_CS_ACTIVE_LOW) {
		__raw_writel(__raw_readl(REG_EBI_CTL(EBIBaseAddr)) & ~EBI_CTL_CSPOLINV_Msk, REG_EBI_CTL(EBIBaseAddr));
	} else {
		__raw_writel(__raw_readl(REG_EBI_CTL(EBIBaseAddr)) | EBI_CTL_CSPOLINV_Msk, REG_EBI_CTL(EBIBaseAddr));
	}

	__raw_writel(__raw_readl(REG_EBI_CTL(EBIBaseAddr)) | u32BusMode, REG_EBI_CTL(EBIBaseAddr));

	switch(u32TimingClass) {
	case EBI_TIMING_FASTEST:
		__raw_writel((__raw_readl(REG_EBI_CTL(EBIBaseAddr)) & ~(EBI_CTL_MCLKDIV_Msk)) |
		             (EBI_MCLKDIV_1 << EBI_CTL_MCLKDIV_Pos) |
		             (u32CSActiveLevel << EBI_CTL_CSPOLINV_Pos) | EBI_CTL_EN_Msk, REG_EBI_CTL(EBIBaseAddr));
		__raw_writel(0x0U, REG_EBI_TCTL(EBIBaseAddr));
		break;

	case EBI_TIMING_VERYFAST:
		__raw_writel((__raw_readl(REG_EBI_CTL(EBIBaseAddr)) & ~(EBI_CTL_MCLKDIV_Msk)) |
		             (EBI_MCLKDIV_1 << EBI_CTL_MCLKDIV_Pos) |
		             (u32CSActiveLevel << EBI_CTL_CSPOLINV_Pos) | EBI_CTL_EN_Msk, REG_EBI_CTL(EBIBaseAddr));
		__raw_writel(0x03003310U, REG_EBI_TCTL(EBIBaseAddr));
		break;

	case EBI_TIMING_FAST:
		__raw_writel((__raw_readl(REG_EBI_CTL(EBIBaseAddr)) & ~(EBI_CTL_MCLKDIV_Msk)) |
		             (EBI_MCLKDIV_2 << EBI_CTL_MCLKDIV_Pos) |
		             (u32CSActiveLevel << EBI_CTL_CSPOLINV_Pos) | EBI_CTL_EN_Msk, REG_EBI_CTL(EBIBaseAddr));
		__raw_writel(0x0U, REG_EBI_TCTL(EBIBaseAddr));
		break;

	case EBI_TIMING_NORMAL:
		__raw_writel((__raw_readl(REG_EBI_CTL(EBIBaseAddr)) & ~(EBI_CTL_MCLKDIV_Msk)) |
		             (EBI_MCLKDIV_2 << EBI_CTL_MCLKDIV_Pos) |
		             (u32CSActiveLevel << EBI_CTL_CSPOLINV_Pos) | EBI_CTL_EN_Msk, REG_EBI_CTL(EBIBaseAddr));
		__raw_writel(0x03003310U, REG_EBI_TCTL(EBIBaseAddr));
		break;

	case EBI_TIMING_SLOW:
		__raw_writel((__raw_readl(REG_EBI_CTL(EBIBaseAddr)) & ~(EBI_CTL_MCLKDIV_Msk)) |
		             (EBI_MCLKDIV_2 << EBI_CTL_MCLKDIV_Pos) |
		             (u32CSActiveLevel << EBI_CTL_CSPOLINV_Pos) | EBI_CTL_EN_Msk, REG_EBI_CTL(EBIBaseAddr));
		__raw_writel(0x03003310U, REG_EBI_TCTL(EBIBaseAddr));
		break;

	case EBI_TIMING_VERYSLOW:
		__raw_writel((__raw_readl(REG_EBI_CTL(EBIBaseAddr)) & ~(EBI_CTL_MCLKDIV_Msk)) |
		             (EBI_MCLKDIV_4 << EBI_CTL_MCLKDIV_Pos) |
		             (u32CSActiveLevel << EBI_CTL_CSPOLINV_Pos) | EBI_CTL_EN_Msk, REG_EBI_CTL(EBIBaseAddr));
		__raw_writel(0x07007730U, REG_EBI_TCTL(EBIBaseAddr));
		break;

	case EBI_TIMING_SLOWEST:
		__raw_writel((__raw_readl(REG_EBI_CTL(EBIBaseAddr)) & ~(EBI_CTL_MCLKDIV_Msk)) |
		             (EBI_MCLKDIV_8 << EBI_CTL_MCLKDIV_Pos) |
		             (u32CSActiveLevel << EBI_CTL_CSPOLINV_Pos) | EBI_CTL_EN_Msk, REG_EBI_CTL(EBIBaseAddr));
		__raw_writel(0x07007730U, REG_EBI_TCTL(EBIBaseAddr));
		break;

	default:
		__raw_writel(__raw_readl(REG_EBI_CTL(EBIBaseAddr)) & ~(EBI_CTL_EN_Msk), REG_EBI_CTL(EBIBaseAddr));
		break;
	}
}

static inline void nuc980_set_ebi_timing(unsigned int bank, unsigned int tACC,
        unsigned int tAHD, unsigned int W2X,unsigned int R2R,
        unsigned int RAHDOFF, unsigned int WAHDOFF)
{
	void __iomem *EBIBaseAddr = EBI0;

	switch (bank) {
	case 0:
		EBIBaseAddr = EBI0;
		break;
	case 1:
		EBIBaseAddr = EBI1;
		break;
	case 2:
		EBIBaseAddr = EBI2;
		break;
	}

	__raw_writel((__raw_readl(REG_EBI_TCTL(EBIBaseAddr)) & ~(0x0FC0F7F0)) |
	             ((tACC&0x1F) << 3) |
	             ((tAHD&0x7) << 8) |
	             ((W2X&0xF) << 12) |
	             ((R2R&0xF) << 24) |
	             ((RAHDOFF&0x1)<<22) |
	             ((WAHDOFF&0x1)<<23)
	             , REG_EBI_TCTL(EBIBaseAddr));
}

#endif /*  __ASM_ARCH_REGS_EBI_H */
