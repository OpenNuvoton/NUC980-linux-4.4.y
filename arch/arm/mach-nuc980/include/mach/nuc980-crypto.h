/*
 * arch/arm/mach-nuc980/include/mach/nuc980-crypto.h
 *
 * Copyright (c) 2018 Nuvoton Technology Corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef _NUC980_CRYPTO_H_
#define _NUC980_CRYPTO_H_

#include <linux/types.h>
#include <linux/ioctl.h>


struct nuc980_crypto_dev
{
    struct device  *dev;
    struct nuc980_crypto_regs  *regs;
    struct mutex   aes_lock;
    struct mutex   sha_lock;

    u32          *aes_inbuf;                /* AES input buffer                          */
    u32          aes_inbuf_size;            /* AES input buffer size                     */
    dma_addr_t   aes_inbuf_dma_addr;        /* physical address of AES input buffer      */
    u32          *aes_outbuf;               /* AES output buffer                         */
    u32          aes_outbuf_size;           /* AES output buffer size                    */
    dma_addr_t   aes_outbuf_dma_addr;       /* physical address of AES output buffer     */

    u32          *hmac_inbuf;               /* SHA/HAMC input buffer                     */
    u32          hmac_inbuf_size;           /* SHA/HMAC input buffer size                */
    dma_addr_t   hmac_inbuf_dma_addr;       /* physical address of SHA/HMAC input buffer */
    u32          sha_len;
};


#define ECC_CURVE_P_192      0x100192
#define ECC_CURVE_P_224      0x100224
#define ECC_CURVE_P_256      0x100256
#define ECC_CURVE_P_384      0x100384
#define ECC_CURVE_P_521      0x100521
#define ECC_CURVE_K_163      0x200163
#define ECC_CURVE_K_233      0x200233
#define ECC_CURVE_K_283      0x200283
#define ECC_CURVE_K_409      0x200409
#define ECC_CURVE_K_571      0x200571
#define ECC_CURVE_B_163      0x300163
#define ECC_CURVE_B_233      0x300233
#define ECC_CURVE_B_283      0x300283
#define ECC_CURVE_B_409      0x300409
#define ECC_CURVE_B_571      0x300571
#define ECC_CURVE_KO_192     0x400192
#define ECC_CURVE_KO_224     0x400224
#define ECC_CURVE_KO_256     0x400256
#define ECC_CURVE_BP_256     0x500256
#define ECC_CURVE_BP_384     0x500384
#define ECC_CURVE_BP_512     0x500512
#define ECC_CURVE_25519      0x025519

enum
{
    CURVE_GF_P,
    CURVE_GF_2M,
};

#define CRYPTO_IOC_MAGIC		'C'

#define AES_IOC_SET_MODE		_IOW(CRYPTO_IOC_MAGIC,  1, unsigned long)
#define AES_IOC_SET_LEN			_IOW(CRYPTO_IOC_MAGIC,  2, unsigned long)
#define AES_IOC_GET_BUFSIZE     _IOW(CRYPTO_IOC_MAGIC,  3, unsigned long *)
#define AES_IOC_SET_IV			_IOW(CRYPTO_IOC_MAGIC,  5, unsigned long *)
#define AES_IOC_SET_KEY			_IOW(CRYPTO_IOC_MAGIC,  6, unsigned long *)
#define AES_IOC_START			_IOW(CRYPTO_IOC_MAGIC,  8, unsigned long)
#define AES_IOC_C_START			_IOW(CRYPTO_IOC_MAGIC,  9, unsigned long)
#define AES_IOC_UPDATE_IV		_IOW(CRYPTO_IOC_MAGIC, 11, unsigned long *)

#define SHA_IOC_INIT	  		_IOW(CRYPTO_IOC_MAGIC, 21, unsigned long)
#define SHA_IOC_FINISH			_IOW(CRYPTO_IOC_MAGIC, 25, unsigned long)

#define ECC_IOC_SEL_CURVE	  	_IOW(CRYPTO_IOC_MAGIC, 51, unsigned long)
#define ECC_IOC_SET_PRI_KEY     _IOW(CRYPTO_IOC_MAGIC, 52, unsigned char *)
#define ECC_IOC_SET_PUB_K1      _IOW(CRYPTO_IOC_MAGIC, 53, unsigned char *)
#define ECC_IOC_SET_PUB_K2      _IOW(CRYPTO_IOC_MAGIC, 54, unsigned char *)
#define ECC_IOC_SET_SCALAR_K    _IOW(CRYPTO_IOC_MAGIC, 55, unsigned char *)
#define ECC_IOC_SET_MSG         _IOW(CRYPTO_IOC_MAGIC, 56, unsigned char *)
#define ECC_IOC_SET_SIG_R       _IOW(CRYPTO_IOC_MAGIC, 57, unsigned char *)
#define ECC_IOC_SET_SIG_S       _IOW(CRYPTO_IOC_MAGIC, 58, unsigned char *)
#define ECC_IOC_GET_PUB_K1      _IOW(CRYPTO_IOC_MAGIC, 61, unsigned char *)
#define ECC_IOC_GET_PUB_K2      _IOW(CRYPTO_IOC_MAGIC, 62, unsigned char *)
#define ECC_IOC_GET_SIG_R       _IOW(CRYPTO_IOC_MAGIC, 63, unsigned char *)
#define ECC_IOC_GET_SIG_S       _IOW(CRYPTO_IOC_MAGIC, 64, unsigned char *)
#define ECC_IOC_GEN_PUB_KEY     _IOW(CRYPTO_IOC_MAGIC, 71, unsigned long)
#define ECC_IOC_ECDSA_SIGN      _IOW(CRYPTO_IOC_MAGIC, 72, unsigned long)
#define ECC_IOC_ECDSA_VERIFY    _IOW(CRYPTO_IOC_MAGIC, 73, unsigned long)
#define ECC_IOC_POINT_MUL       _IOW(CRYPTO_IOC_MAGIC, 81, unsigned long)

#define RSA_IOC_SET_BIT_LEN     _IOW(CRYPTO_IOC_MAGIC, 90, unsigned long)
#define RSA_IOC_SET_N           _IOW(CRYPTO_IOC_MAGIC, 91, unsigned char *)
#define RSA_IOC_SET_D           _IOW(CRYPTO_IOC_MAGIC, 92, unsigned char *)
#define RSA_IOC_SET_E           _IOW(CRYPTO_IOC_MAGIC, 93, unsigned char *)
#define RSA_IOC_SET_C           _IOW(CRYPTO_IOC_MAGIC, 94, unsigned char *)
#define RSA_IOC_SET_MSG         _IOW(CRYPTO_IOC_MAGIC, 95, unsigned char *)
#define RSA_IOC_GET_MSG         _IOW(CRYPTO_IOC_MAGIC, 96, unsigned char *)
#define RSA_IOC_GET_SIG         _IOW(CRYPTO_IOC_MAGIC, 97, unsigned char *)
#define RSA_IOC_SET_SIG         _IOW(CRYPTO_IOC_MAGIC, 98, unsigned char *)
#define RSA_IOC_DO_SIGN         _IOW(CRYPTO_IOC_MAGIC, 101, unsigned long)
#define RSA_IOC_DO_VERIFY       _IOW(CRYPTO_IOC_MAGIC, 102, unsigned long)

#endif
