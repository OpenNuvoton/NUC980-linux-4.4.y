/* linux/driver/crypto/nuc980-crypto-raw.c
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>


#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <mach/map.h>
#include <mach/regs-crypto.h>
#include <mach/nuc980-crypto.h>


extern struct nuc980_crypto_dev  nuc980_crdev;    /* declared in nuc980-crypto.c */


/*-----------------------------------------------------------------------------------------------*/
/*                                                                                               */
/*    AES                                                                                        */
/*                                                                                               */
/*-----------------------------------------------------------------------------------------------*/


// This function does not block, transaction complete in write(), this API is only for user to read back card response.
static ssize_t nvt_aes_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	volatile struct nuc980_crypto_regs  *crpt_regs = nuc980_crdev.regs;
	u32    t0;
	int    ret = 0;

	mutex_lock(&nuc980_crdev.aes_lock);

	t0 = jiffies;
	while (crpt_regs->CRPT_AES_STS & AES_BUSY) {
		if (jiffies - t0 >= 100) { /* 1s time-out */
			ret = -EFAULT;
			goto out;
		}
	}

	if (copy_to_user(buf, (u8 *)nuc980_crdev.aes_outbuf, count)) {
		ret = -EFAULT;
		goto out;
	}

out:
	mutex_unlock(&nuc980_crdev.aes_lock);
	return ret;
}


static ssize_t nvt_aes_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	volatile struct nuc980_crypto_regs  *crpt_regs = nuc980_crdev.regs;
	int   ret = 0;

	mutex_lock(&nuc980_crdev.aes_lock);

	if (copy_from_user((u8 *)nuc980_crdev.aes_inbuf, buf, count)) {
		ret = -EFAULT;
	} else {
		crpt_regs->CRPT_AES_CTL |= AES_START;
	}

	mutex_unlock(&nuc980_crdev.aes_lock);
	return ret;
}

static int nvt_aes_mmap(struct file *filp, struct vm_area_struct * vma)
{
	unsigned long pageFrameNo = 0, size;

	pageFrameNo = __phys_to_pfn(nuc980_crdev.aes_inbuf_dma_addr);

	size = vma->vm_end - vma->vm_start;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= (VM_DONTEXPAND | VM_DONTDUMP);
	if (remap_pfn_range(vma, vma->vm_start, pageFrameNo,size, vma->vm_page_prot)) {
		printk(KERN_INFO "nvt_aes_mmap() : remap_pfn_range() failed !\n");
		return -EINVAL;
	}
	return 0;
}


static long nvt_aes_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	volatile struct nuc980_crypto_regs  *crpt_regs = nuc980_crdev.regs;
	u32   t0, param;

	mutex_lock(&nuc980_crdev.aes_lock);

	switch(cmd) {
	case AES_IOC_SET_MODE:
		//printk("Set AES mode\n");
		crpt_regs->CRPT_AES_CTL = arg | AES_DMAEN;
		crpt_regs->CRPT_AES0_SADDR = nuc980_crdev.aes_inbuf_dma_addr;
		crpt_regs->CRPT_AES0_DADDR = nuc980_crdev.aes_outbuf_dma_addr;
		crpt_regs->CRPT_AES0_CNT = 16;   /* one AES block */
		break;

	case AES_IOC_SET_LEN:
		crpt_regs->CRPT_AES0_CNT = arg;
		break;

	case AES_IOC_SET_IV:
		copy_from_user((void *)&(crpt_regs->CRPT_AES0_IV[0]), (const void *)arg, 16);
		//printk("AES_IOC_SET_IV: 0x%x-0x%x-0x%x-0x%x\n", crpt_regs->CRPT_AES0_IV[0], crpt_regs->CRPT_AES0_IV[1], crpt_regs->CRPT_AES0_IV[2], crpt_regs->CRPT_AES0_IV[3]);
		break;

	case AES_IOC_SET_KEY:
		copy_from_user((void *)&(crpt_regs->CRPT_AES0_KEY[0]), (const void *)arg, 32);
		//printk("AES_IOC_SET_KEY: 0x%x-0x%x-0x%x-0x%x\n", crpt_regs->CRPT_AES0_KEY[0], crpt_regs->CRPT_AES0_KEY[1], crpt_regs->CRPT_AES0_KEY[2], crpt_regs->CRPT_AES0_KEY[3]);
		break;

	case AES_IOC_GET_BUFSIZE:
		param = nuc980_crdev.aes_inbuf_size * 2;    /* input buffer plus output buffer, they are contiguous */
		copy_to_user((u8 *)arg, (u8 *)&param, 4);
		break;

	case AES_IOC_START:
		crpt_regs->CRPT_AES_CTL = (crpt_regs->CRPT_AES_CTL & ~AES_DMACSCAD) | AES_START;
		t0 = jiffies;
		while (crpt_regs->CRPT_AES_STS & AES_BUSY) {
			if (jiffies - t0 >= 100) { /* 1s time-out */
				mutex_unlock(&nuc980_crdev.aes_lock);
				return -EFAULT;
			}
		}
		break;

	case AES_IOC_C_START:
		crpt_regs->CRPT_AES_CTL |= (AES_DMACSCAD | AES_START);
		t0 = jiffies;
		while (crpt_regs->CRPT_AES_STS & AES_BUSY) {
			if (jiffies - t0 >= 100) { /* 1s time-out */
				mutex_unlock(&nuc980_crdev.aes_lock);
				return -EFAULT;
			}
		}
		break;

	case AES_IOC_UPDATE_IV:
		copy_to_user((u8 *)arg, (u8 *)&(crpt_regs->CRPT_AES_FDBCK[0]), 16);
		break;

	default:
		mutex_unlock(&nuc980_crdev.aes_lock);
		return -ENOTTY;
	}
	mutex_unlock(&nuc980_crdev.aes_lock);
	return 0;
}


struct file_operations nvt_aes_fops = {
	.owner		= THIS_MODULE,
	.read		= nvt_aes_read,
	.write		= nvt_aes_write,
	.mmap       = nvt_aes_mmap,
	.unlocked_ioctl	= nvt_aes_ioctl,
};

static struct miscdevice nvt_aes_dev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "nuvoton-aes",
	.fops		= &nvt_aes_fops,
};


/*-----------------------------------------------------------------------------------------------*/
/*                                                                                               */
/*    SHA                                                                                        */
/*                                                                                               */
/*-----------------------------------------------------------------------------------------------*/

#define SHA_BUFF_SIZE       256
static unsigned char  sha_buffer[SHA_BUFF_SIZE];
static int   sha_remaining_cnt = 0;

static ssize_t nvt_sha_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	volatile struct nuc980_crypto_regs  *crpt_regs = nuc980_crdev.regs;
	u32    t0;
	int    ret = 0;

	if (sha_remaining_cnt != 0) {
		printk("SHA_IOC_FINISH missed!\n");
		return -EFAULT;
	}

	mutex_lock(&nuc980_crdev.sha_lock);

	t0 = jiffies;
	while (crpt_regs->CRPT_HMAC_STS & HMAC_BUSY) {
		if (jiffies - t0 >= 100) { /* 1s time-out */
			ret = -EFAULT;
			goto out;
		}
	}

	if (copy_to_user(buf, (u8 *)&crpt_regs->CRPT_HMAC_DGST[0], count)) {
		ret = -EFAULT;
		goto out;
	}

out:
	mutex_unlock(&nuc980_crdev.sha_lock);
	return ret;
}


static ssize_t nvt_sha_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	volatile struct nuc980_crypto_regs  *crpt_regs = nuc980_crdev.regs;
	u32   *data_ptr;
	int   rcnt, ret = 0;

	mutex_lock(&nuc980_crdev.sha_lock);

	while (count > 0) {
		rcnt = SHA_BUFF_SIZE - sha_remaining_cnt;

		if (count < rcnt)
			rcnt = count;

		if (copy_from_user(&sha_buffer[sha_remaining_cnt], buf, rcnt)) {
			ret = -EFAULT;
		}

		buf += rcnt;
		count -= rcnt;
		sha_remaining_cnt += rcnt;

		if ((sha_remaining_cnt == SHA_BUFF_SIZE) && (count > 0)) {
			/*
			 * If SHA buffer full and still have input data, flush the buffer to SHA engine.
			 */
			data_ptr = (u32 *)&sha_buffer[0];
			while (sha_remaining_cnt > 0) {
				if (crpt_regs->CRPT_HMAC_STS & HMAC_DINREQ) {
					crpt_regs->CRPT_HMAC_DATIN = *data_ptr++;
					sha_remaining_cnt -= 4;
				}
			}
			sha_remaining_cnt = 0;
		}
	}
	mutex_unlock(&nuc980_crdev.sha_lock);
	return ret;
}

static long nvt_sha_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	volatile struct nuc980_crypto_regs  *crpt_regs = nuc980_crdev.regs;
	u32   *data_ptr;

	mutex_lock(&nuc980_crdev.sha_lock);

	switch(cmd) {
	case SHA_IOC_INIT:
		sha_remaining_cnt = 0;
		crpt_regs->CRPT_HMAC_CTL = arg | HMAC_START;
		crpt_regs->CRPT_HMAC_DMACNT = 0x10000000;
		break;

	case SHA_IOC_FINISH:
		if (sha_remaining_cnt) {
			crpt_regs->CRPT_HMAC_DMACNT = sha_remaining_cnt;
			data_ptr = (u32 *)&sha_buffer[0];
			while (sha_remaining_cnt > 0) {
				if (crpt_regs->CRPT_HMAC_STS & HMAC_DINREQ) {
					if (sha_remaining_cnt <= 4) {
						crpt_regs->CRPT_HMAC_CTL |= HMAC_START | HMAC_DMALAST;
					}
					crpt_regs->CRPT_HMAC_DATIN = *data_ptr++;
					sha_remaining_cnt -= 4;
				}
			}
			sha_remaining_cnt = 0;
		} else {
			/* SHA was started, but no data pushed! */
			return -EFAULT;
		}
		break;


	default:
		mutex_unlock(&nuc980_crdev.sha_lock);
		return -ENOTTY;
	}

	mutex_unlock(&nuc980_crdev.sha_lock);
	return 0;
}

struct file_operations nvt_sha_fops = {
	.owner		= THIS_MODULE,
	.read		= nvt_sha_read,        /* used to read SHA output digest            */
	.write		= nvt_sha_write,       /* used to push SHA input data               */
	.unlocked_ioctl	= nvt_sha_ioctl,   /* used to start and finish a SHA operation  */
};

static struct miscdevice nvt_sha_dev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "nuvoton-sha",
	.fops		= &nvt_sha_fops,
};


/*-----------------------------------------------------------------------------------------------*/
/*                                                                                               */
/*    ECC                                                                                        */
/*                                                                                               */
/*-----------------------------------------------------------------------------------------------*/

/*
 *  Define elliptic curve (EC)
 */
typedef struct e_curve_t {
	u32   curve_id;
	int   Echar;
	char  Ea[144];
	char  Eb[144];
	char  Px[144];
	char  Py[144];
	int   Epl;
	char  Pp[176];
	int   Eol;
	char  Eorder[176];
	int   key_len;
	int   irreducible_k1;
	int   irreducible_k2;
	int   irreducible_k3;
	int   GF;
}  ECC_CURVE;

static const ECC_CURVE _Curve[] = {
	{
		/* NIST: Curve P-192 : y^2=x^3-ax+b (mod p) */
		ECC_CURVE_P_192,
		48,     /* Echar */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFFFFFFFFFC",   /* "000000000000000000000000000000000000000000000003" */
		"64210519e59c80e70fa7e9ab72243049feb8deecc146b9b1",
		"188da80eb03090f67cbf20eb43a18800f4ff0afd82ff1012",
		"07192b95ffc8da78631011ed6b24cdd573f977a11e794811",
		58,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFFFFFFFFFF",   /* "6277101735386680763835789423207666416083908700390324961279" */
		58,     /* Eol */
		"FFFFFFFFFFFFFFFFFFFFFFFF99DEF836146BC9B1B4D22831",   /* "6277101735386680763835789423176059013767194773182842284081" */
		192,    /* key_len */
		7,
		2,
		1,
		CURVE_GF_P
	},
	{
		/* NIST: Curve P-224 : y^2=x^3-ax+b (mod p) */
		ECC_CURVE_P_224,
		56,     /* Echar */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFFFFFFFFFFFFFFFFFE",  /* "00000000000000000000000000000000000000000000000000000003" */
		"b4050a850c04b3abf54132565044b0b7d7bfd8ba270b39432355ffb4",
		"b70e0cbd6bb4bf7f321390b94a03c1d356c21122343280d6115c1d21",
		"bd376388b5f723fb4c22dfe6cd4375a05a07476444d5819985007e34",
		70,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000001",  /* "0026959946667150639794667015087019630673557916260026308143510066298881" */
		70,     /* Eol */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFF16A2E0B8F03E13DD29455C5C2A3D",  /* "0026959946667150639794667015087019625940457807714424391721682722368061" */
		224,    /* key_len */
		9,
		8,
		3,
		CURVE_GF_P
	},
	{
		/* NIST: Curve P-256 : y^2=x^3-ax+b (mod p) */
		ECC_CURVE_P_256,
		64,     /* Echar */
		"FFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFC",  /* "0000000000000000000000000000000000000000000000000000000000000003" */
		"5ac635d8aa3a93e7b3ebbd55769886bc651d06b0cc53b0f63bce3c3e27d2604b",
		"6b17d1f2e12c4247f8bce6e563a440f277037d812deb33a0f4a13945d898c296",
		"4fe342e2fe1a7f9b8ee7eb4a7c0f9e162bce33576b315ececbb6406837bf51f5",
		78,     /* Epl */
		"FFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF",  /* "115792089210356248762697446949407573530086143415290314195533631308867097853951" */
		78,     /* Eol */
		"FFFFFFFF00000000FFFFFFFFFFFFFFFFBCE6FAADA7179E84F3B9CAC2FC632551",  /* "115792089210356248762697446949407573529996955224135760342422259061068512044369" */
		256,    /* key_len */
		10,
		5,
		2,
		CURVE_GF_P
	},
	{
		/* NIST: Curve P-384 : y^2=x^3-ax+b (mod p) */
		ECC_CURVE_P_384,
		96,     /* Echar */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFF0000000000000000FFFFFFFC",  /* "000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003" */
		"b3312fa7e23ee7e4988e056be3f82d19181d9c6efe8141120314088f5013875ac656398d8a2ed19d2a85c8edd3ec2aef",
		"aa87ca22be8b05378eb1c71ef320ad746e1d3b628ba79b9859f741e082542a385502f25dbf55296c3a545e3872760ab7",
		"3617de4a96262c6f5d9e98bf9292dc29f8f41dbd289a147ce9da3113b5f0b8c00a60b1ce1d7e819d7a431d7c90ea0e5f",
		116,    /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFF0000000000000000FFFFFFFF",  /* "39402006196394479212279040100143613805079739270465446667948293404245721771496870329047266088258938001861606973112319" */
		116,    /* Eol */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC7634D81F4372DDF581A0DB248B0A77AECEC196ACCC52973",  /* "39402006196394479212279040100143613805079739270465446667946905279627659399113263569398956308152294913554433653942643" */
		384,    /* key_len */
		12,
		3,
		2,
		CURVE_GF_P
	},
	{
		/* NIST: Curve P-521 : y^2=x^3-ax+b (mod p)*/
		ECC_CURVE_P_521,
		131,    /* Echar */
		"1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC",  /* "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000003" */
		"051953eb9618e1c9a1f929a21a0b68540eea2da725b99b315f3b8b489918ef109e156193951ec7e937b1652c0bd3bb1bf073573df883d2c34f1ef451fd46b503f00",
		"0c6858e06b70404e9cd9e3ecb662395b4429c648139053fb521f828af606b4d3dbaa14b5e77efe75928fe1dc127a2ffa8de3348b3c1856a429bf97e7e31c2e5bd66",
		"11839296a789a3bc0045c8a5fb42c7d1bd998f54449579b446817afbd17273e662c97ee72995ef42640c550b9013fad0761353c7086a272c24088be94769fd16650",
		157,    /* Epl */
		"1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",  /* "6864797660130609714981900799081393217269435300143305409394463459185543183397656052122559640661454554977296311391480858037121987999716643812574028291115057151" */
		157,    /* Eol */
		"1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFA51868783BF2F966B7FCC0148F709A5D03BB5C9B8899C47AEBB6FB71E91386409",  /* "6864797660130609714981900799081393217269435300143305409394463459185543183397655394245057746333217197532963996371363321113864768612440380340372808892707005449" */
		521,    /* key_len */
		32,
		32,
		32,
		CURVE_GF_P
	},
	{
		/* NIST: Curve B-163 : y^2+xy=x^3+ax^2+b */
		ECC_CURVE_B_163,
		41,     /* Echar */
		"00000000000000000000000000000000000000001",
		"20a601907b8c953ca1481eb10512f78744a3205fd",
		"3f0eba16286a2d57ea0991168d4994637e8343e36",
		"0d51fbc6c71a0094fa2cdd545b11c5c0c797324f1",
		68,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000001",  /* "26959946667150639794667015087019630673557916260026308143510066298881" */
		49,     /* Eol */
		"40000000000000000000292FE77E70C12A4234C33",   /* "5846006549323611672814742442876390689256843201587" */
		163,    /* key_len */
		7,
		6,
		3,
		CURVE_GF_2M
	},
	{
		/* NIST: Curve B-233 : y^2+xy=x^3+ax^2+b */
		ECC_CURVE_B_233,
		59,     /* Echar 59 */
		"00000000000000000000000000000000000000000000000000000000001",
		"066647ede6c332c7f8c0923bb58213b333b20e9ce4281fe115f7d8f90ad",
		"0fac9dfcbac8313bb2139f1bb755fef65bc391f8b36f8f8eb7371fd558b",
		"1006a08a41903350678e58528bebf8a0beff867a7ca36716f7e01f81052",
		68,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000001",  /* "26959946667150639794667015087019630673557916260026308143510066298881" */
		70,     /* Eol */
		"1000000000000000000000000000013E974E72F8A6922031D2603CFE0D7",  /* "6901746346790563787434755862277025555839812737345013555379383634485463" */
		233,    /* key_len */
		74,
		74,
		74,
		CURVE_GF_2M
	},
	{
		/* NIST: Curve B-283 : y^2+xy=x^3+ax^2+b */
		ECC_CURVE_B_283,
		71,     /* Echar */
		"00000000000000000000000000000000000000000000000000000000000000000000001",
		"27b680ac8b8596da5a4af8a19a0303fca97fd7645309fa2a581485af6263e313b79a2f5",
		"5f939258db7dd90e1934f8c70b0dfec2eed25b8557eac9c80e2e198f8cdbecd86b12053",
		"3676854fe24141cb98fe6d4b20d02b4516ff702350eddb0826779c813f0df45be8112f4",
		68,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000001",  /* "26959946667150639794667015087019630673557916260026308143510066298881" */
		85,     /* Eol */
		"3FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEF90399660FC938A90165B042A7CEFADB307",  /* "7770675568902916283677847627294075626569625924376904889109196526770044277787378692871" */
		283,    /* key_len */
		12,
		7,
		5,
		CURVE_GF_2M
	},
	{
		/* NIST: Curve B-409 : y^2+xy=x^3+ax^2+b */
		ECC_CURVE_B_409,
		103,    /* Echar */
		"0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001",
		"021a5c2c8ee9feb5c4b9a753b7b476b7fd6422ef1f3dd674761fa99d6ac27c8a9a197b272822f6cd57a55aa4f50ae317b13545f",
		"15d4860d088ddb3496b0c6064756260441cde4af1771d4db01ffe5b34e59703dc255a868a1180515603aeab60794e54bb7996a7",
		"061b1cfab6be5f32bbfa78324ed106a7636b9c5a7bd198d0158aa4f5488d08f38514f1fdf4b4f40d2181b3681c364ba0273c706",
		68,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000001",  /* "26959946667150639794667015087019630673557916260026308143510066298881" */
		123,    /* Eol */
		"10000000000000000000000000000000000000000000000000001E2AAD6A612F33307BE5FA47C3C9E052F838164CD37D9A21173",  /* "661055968790248598951915308032771039828404682964281219284648798304157774827374805208143723762179110965979867288366567526771" */
		409,    /* key_len */
		87,
		87,
		87,
		CURVE_GF_2M
	},
	{
		/* NIST: Curve B-571 : y^2+xy=x^3+ax^2+b */
		ECC_CURVE_B_571,
		143,    /* Echar */
		"00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001",
		"2f40e7e2221f295de297117b7f3d62f5c6a97ffcb8ceff1cd6ba8ce4a9a18ad84ffabbd8efa59332be7ad6756a66e294afd185a78ff12aa520e4de739baca0c7ffeff7f2955727a",
		"303001d34b856296c16c0d40d3cd7750a93d1d2955fa80aa5f40fc8db7b2abdbde53950f4c0d293cdd711a35b67fb1499ae60038614f1394abfa3b4c850d927e1e7769c8eec2d19",
		"37bf27342da639b6dccfffeb73d69d78c6c27a6009cbbca1980f8533921e8a684423e43bab08a576291af8f461bb2a8b3531d2f0485c19b16e2f1516e23dd3c1a4827af1b8ac15b",
		68,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000001",  /* "26959946667150639794667015087019630673557916260026308143510066298881" */
		172,    /* Eol */
		"3FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE661CE18FF55987308059B186823851EC7DD9CA1161DE93D5174D66E8382E9BB2FE84E47",  /* "3864537523017258344695351890931987344298927329706434998657235251451519142289560424536143999389415773083133881121926944486246872462816813070234528288303332411393191105285703" */
		571,    /* key_len */
		10,
		5,
		2,
		CURVE_GF_2M
	},
	{
		/* NIST: Curve K-163 : y^2+xy=x^3+ax^2+b */
		ECC_CURVE_K_163,
		41,     /* Echar */
		"00000000000000000000000000000000000000001",
		"00000000000000000000000000000000000000001",
		"2fe13c0537bbc11acaa07d793de4e6d5e5c94eee8",
		"289070fb05d38ff58321f2e800536d538ccdaa3d9",
		68,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000001",  /* "26959946667150639794667015087019630673557916260026308143510066298881" */
		49,     /* Eol */
		"4000000000000000000020108A2E0CC0D99F8A5EF",  /* "5846006549323611672814741753598448348329118574063" */
		163,    /* key_len */
		7,
		6,
		3,
		CURVE_GF_2M
	},
	{
		/* NIST: Curve K-233 : y^2+xy=x^3+ax^2+b */
		ECC_CURVE_K_233,
		59,     /* Echar 59 */
		"00000000000000000000000000000000000000000000000000000000000",
		"00000000000000000000000000000000000000000000000000000000001",
		"17232ba853a7e731af129f22ff4149563a419c26bf50a4c9d6eefad6126",
		"1db537dece819b7f70f555a67c427a8cd9bf18aeb9b56e0c11056fae6a3",
		68,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000001",    /* "26959946667150639794667015087019630673557916260026308143510066298881" */
		70,     /* Eol */
		"8000000000000000000000000000069D5BB915BCD46EFB1AD5F173ABDF",  /* "3450873173395281893717377931138512760570940988862252126328087024741343" */
		233,    /* key_len */
		74,
		74,
		74,
		CURVE_GF_2M
	},
	{
		/* NIST: Curve K-283 : y^2+xy=x^3+ax^2+b */
		ECC_CURVE_K_283,
		71,     /* Echar */
		"00000000000000000000000000000000000000000000000000000000000000000000000",
		"00000000000000000000000000000000000000000000000000000000000000000000001",
		"503213f78ca44883f1a3b8162f188e553cd265f23c1567a16876913b0c2ac2458492836",
		"1ccda380f1c9e318d90f95d07e5426fe87e45c0e8184698e45962364e34116177dd2259",
		68,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000001",  /* "26959946667150639794667015087019630673557916260026308143510066298881" */
		85,     /* Eol */
		"1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE9AE2ED07577265DFF7F94451E061E163C61",  /* "3885337784451458141838923813647037813284811733793061324295874997529815829704422603873" */
		283,    /* key_len */
		12,
		7,
		5,
		CURVE_GF_2M
	},
	{
		/* NIST: Curve K-409 : y^2+xy=x^3+ax^2+b */
		ECC_CURVE_K_409,
		103,    /* Echar */
		"0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000",
		"0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001",
		"060f05f658f49c1ad3ab1890f7184210efd0987e307c84c27accfb8f9f67cc2c460189eb5aaaa62ee222eb1b35540cfe9023746",
		"1e369050b7c4e42acba1dacbf04299c3460782f918ea427e6325165e9ea10e3da5f6c42e9c55215aa9ca27a5863ec48d8e0286b",
		68,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000001",  /* "26959946667150639794667015087019630673557916260026308143510066298881" */
		123,    /* Eol */
		"7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFE5F83B2D4EA20400EC4557D5ED3E3E7CA5B4B5C83B8E01E5FCF",  /* "330527984395124299475957654016385519914202341482140609642324395022880711289249191050673258457777458014096366590617731358671" */
		409,    /* key_len */
		87,
		87,
		87,
		CURVE_GF_2M
	},
	{
		/* NIST: Curve K-571 : y^2+xy=x^3+ax^2+b */
		ECC_CURVE_K_571,
		143,    /* Echar */
		"00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000",
		"00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001",
		"26eb7a859923fbc82189631f8103fe4ac9ca2970012d5d46024804801841ca44370958493b205e647da304db4ceb08cbbd1ba39494776fb988b47174dca88c7e2945283a01c8972",
		"349dc807f4fbf374f4aeade3bca95314dd58cec9f307a54ffc61efc006d8a2c9d4979c0ac44aea74fbebbb9f772aedcb620b01a7ba7af1b320430c8591984f601cd4c143ef1c7a3",
		68,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000000000000000000001",  /* "26959946667150639794667015087019630673557916260026308143510066298881" */
		172,    /* Eol */
		"20000000000000000000000000000000000000000000000000000000000000000000000131850E1F19A63E4B391A8DB917F4138B630D84BE5D639381E91DEB45CFE778F637C1001",  /* "1932268761508629172347675945465993672149463664853217499328617625725759571144780212268133978522706711834706712800825351461273674974066617311929682421617092503555733685276673" */
		571,    /* key_len */
		10,
		5,
		2,
		CURVE_GF_2M
	},
	{
		/* Koblitz: Curve secp192k1 : y2 = x3+ax+b over Fp */
		ECC_CURVE_KO_192,
		48,     /* Echar */
		"00000000000000000000000000000000000000000",
		"00000000000000000000000000000000000000003",
		"DB4FF10EC057E9AE26B07D0280B7F4341DA5D1B1EAE06C7D",
		"9B2F2F6D9C5628A7844163D015BE86344082AA88D95E2F9D",
		58,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFEE37",  /* p */
		58,     /* Eol */
		"FFFFFFFFFFFFFFFFFFFFFFFE26F2FC170F69466A74DEFD8D",  /* n */
		192,    /* key_len */
		7,
		2,
		1,
		CURVE_GF_P
	},
	{
		/* Koblitz: Curve secp224k1 : y2 = x3+ax+b over Fp */
		ECC_CURVE_KO_224,
		56,     /* Echar */
		"00000000000000000000000000000000000000000000000000000000",
		"00000000000000000000000000000000000000000000000000000005",
		"A1455B334DF099DF30FC28A169A467E9E47075A90F7E650EB6B7A45C",
		"7E089FED7FBA344282CAFBD6F7E319F7C0B0BD59E2CA4BDB556D61A5",
		70,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFE56D",  /* p */
		70,     /* Eol */
		"0000000000000000000000000001DCE8D2EC6184CAF0A971769FB1F7",  /* n */
		224,    /* key_len */
		7,
		2,
		1,
		CURVE_GF_P
	},
	{
		/* Koblitz: Curve secp256k1 : y2 = x3+ax+b over Fp */
		ECC_CURVE_KO_256,
		64,     /* Echar */
		"0000000000000000000000000000000000000000000000000000000000000000",
		"0000000000000000000000000000000000000000000000000000000000000007",
		"79BE667EF9DCBBAC55A06295CE870B07029BFCDB2DCE28D959F2815B16F81798",
		"483ADA7726A3C4655DA4FBFC0E1108A8FD17B448A68554199C47D08FFB10D4B8",
		78,     /* Epl */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFC2F",  /* p */
		78,     /* Eol */
		"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEBAAEDCE6AF48A03BBFD25E8CD0364141",  /* n */
		256,    /* key_len */
		7,
		2,
		1,
		CURVE_GF_P
	},
	{
		/* Brainpool: Curve brainpoolP256r1 */
		ECC_CURVE_BP_256,
		64,     /* Echar */
		"7D5A0975FC2C3057EEF67530417AFFE7FB8055C126DC5C6CE94A4B44F330B5D9",  /* A */
		"26DC5C6CE94A4B44F330B5D9BBD77CBF958416295CF7E1CE6BCCDC18FF8C07B6",  /* B */
		"8BD2AEB9CB7E57CB2C4B482FFC81B7AFB9DE27E1E3BD23C23A4453BD9ACE3262",  /* x */
		"547EF835C3DAC4FD97F8461A14611DC9C27745132DED8E545C1D54C72F046997",  /* y */
		78,     /* Epl */
		"A9FB57DBA1EEA9BC3E660A909D838D726E3BF623D52620282013481D1F6E5377",  /* p */
		78,     /* Eol */
		"A9FB57DBA1EEA9BC3E660A909D838D718C397AA3B561A6F7901E0E82974856A7",  /* q */
		256,    /* key_len */
		7,
		2,
		1,
		CURVE_GF_P
	},
	{
		/* Brainpool: Curve brainpoolP384r1 */
		ECC_CURVE_BP_384,
		96,     /* Echar */
		"7BC382C63D8C150C3C72080ACE05AFA0C2BEA28E4FB22787139165EFBA91F90F8AA5814A503AD4EB04A8C7DD22CE2826",  /* A */
		"04A8C7DD22CE28268B39B55416F0447C2FB77DE107DCD2A62E880EA53EEB62D57CB4390295DBC9943AB78696FA504C11",  /* B */
		"1D1C64F068CF45FFA2A63A81B7C13F6B8847A3E77EF14FE3DB7FCAFE0CBD10E8E826E03436D646AAEF87B2E247D4AF1E",  /* x */
		"8ABE1D7520F9C2A45CB1EB8E95CFD55262B70B29FEEC5864E19C054FF99129280E4646217791811142820341263C5315",  /* y */
		116,     /* Epl */
		"8CB91E82A3386D280F5D6F7E50E641DF152F7109ED5456B412B1DA197FB71123ACD3A729901D1A71874700133107EC53",  /* p */
		116,     /* Eol */
		"8CB91E82A3386D280F5D6F7E50E641DF152F7109ED5456B31F166E6CAC0425A7CF3AB6AF6B7FC3103B883202E9046565",  /* q */
		384,    /* key_len */
		7,
		2,
		1,
		CURVE_GF_P
	},
	{
		/* Brainpool: Curve brainpoolP512r1 */
		ECC_CURVE_BP_512,
		128,     /* Echar */
		"7830A3318B603B89E2327145AC234CC594CBDD8D3DF91610A83441CAEA9863BC2DED5D5AA8253AA10A2EF1C98B9AC8B57F1117A72BF2C7B9E7C1AC4D77FC94CA",  /* A */
		"3DF91610A83441CAEA9863BC2DED5D5AA8253AA10A2EF1C98B9AC8B57F1117A72BF2C7B9E7C1AC4D77FC94CADC083E67984050B75EBAE5DD2809BD638016F723",  /* B */
		"81AEE4BDD82ED9645A21322E9C4C6A9385ED9F70B5D916C1B43B62EEF4D0098EFF3B1F78E2D0D48D50D1687B93B97D5F7C6D5047406A5E688B352209BCB9F822",  /* x */
		"7DDE385D566332ECC0EABFA9CF7822FDF209F70024A57B1AA000C55B881F8111B2DCDE494A5F485E5BCA4BD88A2763AED1CA2B2FA8F0540678CD1E0F3AD80892",  /* y */
		156,     /* Epl */
		"AADD9DB8DBE9C48B3FD4E6AE33C9FC07CB308DB3B3C9D20ED6639CCA703308717D4D9B009BC66842AECDA12AE6A380E62881FF2F2D82C68528AA6056583A48F3",  /* p */
		156,     /* Eol */
		"AADD9DB8DBE9C48B3FD4E6AE33C9FC07CB308DB3B3C9D20ED6639CCA70330870553E5C414CA92619418661197FAC10471DB1D381085DDADDB58796829CA90069",  /* q */
		512,    /* key_len */
		7,
		2,
		1,
		CURVE_GF_P
	},
	{
		ECC_CURVE_25519,
		64,		// Echar
		"0000000000000000000000000000000000000000000000000000000000076D06",  // "0000000000000000000000000000000000000000000000000000000000000003",
		"0000000000000000000000000000000000000000000000000000000000000001",
		"0000000000000000000000000000000000000000000000000000000000000009",
		"20ae19a1b8a086b4e01edd2c7748d14c923d4d7e6d7c61b229e9c5a27eced3d9",
		78,		// Epl
		"7fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffed",  // "115792089210356248762697446949407573530086143415290314195533631308867097853951",
		78,		// Eol
		"1000000000000000000000000000000014def9dea2f79cd65812631a5cf5d3ed",  // "115792089210356248762697446949407573529996955224135760342422259061068512044369",
		255,	// key_len
		10,
		5,
		2,
		CURVE_GF_P
	},
};

u32 _curve_used = ECC_CURVE_P_256;
static ECC_CURVE  *pCurve = (ECC_CURVE *)&_Curve[0];
static ECC_CURVE  Curve_Copy;
static char  temp_hex_str[160];
static char  ecc_x1[160], ecc_y1[160], ecc_d[160], ecc_k[160], ecc_msg[160], ecc_r[160], ecc_s[160];
static char  hex_char_tbl[] = "0123456789abcdef";

static void dump_ecc_reg(char *str, uint32_t volatile regs[], int32_t count)
{
#if 0
	int32_t  i;

	printk("%s => ", str);
	for (i = 0; i < count; i++) {
		printk("0x%08x ", regs[i]);
	}
	printk("\n");
#endif
}

static char  ch2hex(char ch)
{
	if (ch <= '9') {
		ch = ch - '0';
	} else if ((ch <= 'z') && (ch >= 'a')) {
		ch = ch - 'a' + 10U;
	} else {
		ch = ch - 'A' + 10U;
	}
	return ch;
}

static void Hex2Reg(char input[], u32 volatile reg[])
{
	char      hex;
	int       si, ri;
	u32       i, val32;

	si = (int)strlen(input) - 1;
	ri = 0;

	while (si >= 0) {
		val32 = 0UL;
		for (i = 0UL; (i < 8UL) && (si >= 0); i++) {
			hex = ch2hex(input[si]);
			val32 |= (uint32_t)hex << (i * 4UL);
			si--;
		}
		reg[ri++] = val32;
	}
}

static char get_Nth_nibble_char(u32 val32, u32 idx)
{
	return hex_char_tbl[ (val32 >> (idx * 4U)) & 0xfU ];
}

static void Reg2Hex(int count, u32 volatile reg[], char output[])
{
	int    idx, ri;
	u32    i;

	output[count] = 0U;
	idx = count - 1;

	for (ri = 0; idx >= 0; ri++) {
		for (i = 0UL; (i < 8UL) && (idx >= 0); i++) {
			output[idx] = get_Nth_nibble_char(reg[ri], i);
			idx--;
		}
	}
}

static int ecc_strcmp(char *s1, char *s2)
{
	char  c1, c2;

	while (*s1 == '0') s1++;
	while (*s2 == '0') s2++;

	for ( ; *s1 || *s2; s1++, s2++) {
		if ((*s1 >= 'A') && (*s1 <= 'Z'))
			c1 = *s1 + 32;
		else
			c1 = *s1;

		if ((*s2 >= 'A') && (*s2 <= 'Z'))
			c2 = *s2 + 32;
		else
			c2 = *s2;

		if (c1 != c2)
			return 1;
	}
	return 0;
}

static ECC_CURVE * get_curve(u32 ecc_curve)
{
	u32     i;
	ECC_CURVE  *ret = NULL;

	for (i = 0UL; i < sizeof(_Curve) / sizeof(ECC_CURVE); i++) {
		if (ecc_curve == _Curve[i].curve_id) {
			memcpy((char *)&Curve_Copy, &_Curve[i], sizeof(ECC_CURVE));
			ret = &Curve_Copy;   /* (ECC_CURVE *)&_Curve[i]; */
			break;
		}
	}
	return ret;
}

static int ecc_init_curve(u32 ecc_curve)
{
	volatile struct nuc980_crypto_regs  *crpt_regs = nuc980_crdev.regs;
	int  i;

	pCurve = get_curve(ecc_curve);
	if (pCurve == NULL)
		return -1;

	for (i = 0; i < 18; i++) {
		crpt_regs->ECC_A[i] = 0UL;
		crpt_regs->ECC_B[i] = 0UL;
		crpt_regs->ECC_X1[i] = 0UL;
		crpt_regs->ECC_Y1[i] = 0UL;
		crpt_regs->ECC_N[i] = 0UL;
	}

	Hex2Reg(pCurve->Ea, crpt_regs->ECC_A);
	Hex2Reg(pCurve->Eb, crpt_regs->ECC_B);
	Hex2Reg(pCurve->Px, crpt_regs->ECC_X1);
	Hex2Reg(pCurve->Py, crpt_regs->ECC_Y1);

	//printk("Key length = %d\n", pCurve->key_len);
	dump_ecc_reg("CRPT_ECC_CURVE_A", crpt_regs->ECC_A, 10);
	dump_ecc_reg("CRPT_ECC_CURVE_B", crpt_regs->ECC_B, 10);
	dump_ecc_reg("CRPT_ECC_POINT_X1", crpt_regs->ECC_X1, 10);
	dump_ecc_reg("CRPT_ECC_POINT_Y1", crpt_regs->ECC_Y1, 10);

	if (pCurve->GF == (int)CURVE_GF_2M) {
		crpt_regs->ECC_N[0] = 0x1UL;
		crpt_regs->ECC_N[(pCurve->key_len) / 32] |= (1UL << ((pCurve->key_len) % 32));
		crpt_regs->ECC_N[(pCurve->irreducible_k1) / 32] |= (1UL << ((pCurve->irreducible_k1) % 32));
		crpt_regs->ECC_N[(pCurve->irreducible_k2) / 32] |= (1UL << ((pCurve->irreducible_k2) % 32));
		crpt_regs->ECC_N[(pCurve->irreducible_k3) / 32] |= (1UL << ((pCurve->irreducible_k3) % 32));
	} else {
		Hex2Reg(pCurve->Pp, crpt_regs->ECC_N);
	}
	dump_ecc_reg("CRPT_ECC_CURVE_N", crpt_regs->ECC_N, 10);
	return 0;
}

static int run_ecc_codec(uint32_t mode)
{
	volatile struct nuc980_crypto_regs  *crpt_regs = nuc980_crdev.regs;
	volatile int32_t  t0;

	if ((mode & CRPT_ECC_CTL_ECCOP_Mask) == ECCOP_MODULE) {
		crpt_regs->ECC_CTL = CRPT_ECC_CTL_FSEL_Mask;
	} else {
		if (pCurve->GF == (int)CURVE_GF_2M)
			crpt_regs->ECC_CTL = 0UL;    /* point */
		else
			crpt_regs->ECC_CTL = CRPT_ECC_CTL_FSEL_Mask;  /* CURVE_GF_P */
	}

	t0 = jiffies;
	crpt_regs->ECC_CTL |= ((u32)pCurve->key_len << CRPT_ECC_CTL_CURVE_M_Pos) | mode | CRPT_ECC_CTL_START_Mask;
	while (crpt_regs->ECC_STS & CRPT_ECC_STS_BUSY_Mask) {
		if (jiffies - t0 >= 100) {
			printk("run_ecc_codec time-out!\n");
			return -1;
		}
	}
	return 0;
}

//int32_t  ECC_Mutiply(CRPT_T *crpt, E_ECC_CURVE ecc_curve, char x1[], char y1[], char *k, char x2[], char y2[])
static int ECC_Mutiply(void)
{
	volatile struct nuc980_crypto_regs  *crpt_regs = nuc980_crdev.regs;
	volatile int32_t  i, t0;

	if (ecc_init_curve(_curve_used) != 0)
		return -1;

	for (i = 0; i < 18; i++)
	{
		crpt_regs->ECC_X1[i] = 0UL;
		crpt_regs->ECC_Y1[i] = 0UL;
		crpt_regs->ECC_K[i] = 0UL;
	}
	Hex2Reg(ecc_x1, crpt_regs->ECC_X1);
	Hex2Reg(ecc_y1, crpt_regs->ECC_Y1);
	Hex2Reg(ecc_k, crpt_regs->ECC_K);

	/* set FSEL (Field selection) */
	if (pCurve->GF == (int)CURVE_GF_2M)
		crpt_regs->ECC_CTL = 0UL;
	else
		crpt_regs->ECC_CTL = CRPT_ECC_CTL_FSEL_Mask;  /*  CURVE_GF_P */

	crpt_regs->ECC_CTL |= ((u32)pCurve->key_len << CRPT_ECC_CTL_CURVE_M_Pos) |
					 ECCOP_POINT_MUL | CRPT_ECC_CTL_START_Mask;

	t0 = jiffies;
	while (crpt_regs->ECC_STS & CRPT_ECC_STS_BUSY_Mask) {
		if (jiffies - t0 >= 100) {
			printk("ECC mul time-out!\n");
			return -1;
		}
	}
	Reg2Hex(pCurve->Echar, crpt_regs->ECC_X1, ecc_x1);
	Reg2Hex(pCurve->Echar, crpt_regs->ECC_Y1, ecc_y1);
	return 0;
}

//int32_t  ECC_GeneratePublicKey(CRPT_T *crpt, E_ECC_CURVE ecc_curve, char *private_k, char public_k1[], char public_k2[])
static int  ECC_GeneratePublicKey(void)
{
	volatile struct nuc980_crypto_regs  *crpt_regs = nuc980_crdev.regs;
	volatile int32_t  i, t0;

	if (ecc_init_curve(_curve_used) != 0)
		return -1;

	for (i = 0; i < 18; i++)
		crpt_regs->ECC_K[i] = 0UL;

	Hex2Reg(ecc_d, crpt_regs->ECC_K);

	/* set FSEL (Field selection) */
	if (pCurve->GF == (int)CURVE_GF_2M)
		crpt_regs->ECC_CTL = 0UL;
	else  /*  CURVE_GF_P */
		crpt_regs->ECC_CTL = CRPT_ECC_CTL_FSEL_Mask;

	crpt_regs->ECC_CTL |= ((u32)pCurve->key_len << CRPT_ECC_CTL_CURVE_M_Pos) | ECCOP_POINT_MUL | CRPT_ECC_CTL_START_Mask;

	t0 = jiffies;
	while (crpt_regs->ECC_STS & CRPT_ECC_STS_BUSY_Mask) {
		if (jiffies - t0 >= 100) {
			printk("ECC gen key time-out!\n");
			return -1;
		}
	}

	Reg2Hex(pCurve->Echar, crpt_regs->ECC_X1, ecc_x1);
	Reg2Hex(pCurve->Echar, crpt_regs->ECC_Y1, ecc_y1);
	return 0;
}


//int32_t  ECC_GenerateSignature(CRPT_T *crpt, E_ECC_CURVE ecc_curve, char *message,
//                               char *d, char *k, char *R, char *S)
static int  ECC_GenerateSignature(void)
{
	volatile struct nuc980_crypto_regs  *crpt_regs = nuc980_crdev.regs;
	volatile int32_t  i;
	uint32_t volatile temp_result1[18], temp_result2[18];

	if (ecc_init_curve(_curve_used) != 0)
		return -1;

	/*
	 *   1. Calculate e = HASH(m), where HASH is a cryptographic hashing algorithm, (i.e. SHA-1)
	 *      (1) Use SHA to calculate e
	 */

	/*   2. Select a random integer k form [1, n-1]
	 *      (1) Notice that n is order, not prime modulus or irreducible polynomial function
	 */

	/*
	 *   3. Compute r = x1 (mod n), where (x1, y1) = k * G. If r = 0, go to step 2
	 *      (1) Write the curve parameter A, B, and curve length M to corresponding registers
	 *      (2) Write the prime modulus or irreducible polynomial function to N registers according
	 *      (3) Write the point G(x, y) to X1, Y1 registers
	 *      (4) Write the random integer k to K register
	 *      (5) Set ECCOP(CRPT_ECC_CTL[10:9]) to 00
	 *      (6) Set FSEL(CRPT_ECC_CTL[8]) according to used curve of prime field or binary field
	 *      (7) Set START(CRPT_ECC_CTL[0]) to 1
	 *      (8) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
	 *      (9) Write the curve order and curve length to N ,M registers according
	 *      (10) Write 0x0 to Y1 registers
	 *      (11) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
	 *      (12) Set MOPOP(CRPT_ECC_CTL[12:11]) to 10
	 *      (13) Set START(CRPT_ECC_CTL[0]) to 1         *
	 *      (14) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
	 *      (15) Read X1 registers to get r
	 */

	/* 3-(4) Write the random integer k to K register */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_K[i] = 0UL;

	Hex2Reg(ecc_k, crpt_regs->ECC_K);

	if (run_ecc_codec(ECCOP_POINT_MUL) != 0)
		return -1;

	/*  3-(9) Write the curve order to N registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_N[i] = 0UL;
	Hex2Reg(pCurve->Eorder, crpt_regs->ECC_N);

	/* 3-(10) Write 0x0 to Y1 registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_Y1[i] = 0UL;

	if (run_ecc_codec(ECCOP_MODULE | MODOP_ADD) != 0)
		return -1;

	/* 3-(15) Read X1 registers to get r */
	for (i = 0; i < 18; i++)
		temp_result1[i] = crpt_regs->ECC_X1[i];

	Reg2Hex(pCurve->Echar, temp_result1, ecc_r);

	/*
	 *   4. Compute s = k ? 1 \A1\D1 (e + d \A1\D1 r)(mod n). If s = 0, go to step 2
	 *      (1) Write the curve order to N registers according
	 *      (2) Write 0x1 to Y1 registers
	 *      (3) Write the random integer k to X1 registers according
	 *      (4) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
	 *      (5) Set MOPOP(CRPT_ECC_CTL[12:11]) to 00
	 *      (6) Set START(CRPT_ECC_CTL[0]) to 1
	 *      (7) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
	 *      (8) Read X1 registers to get k^-1
	 *      (9) Write the curve order and curve length to N ,M registers
	 *      (10) Write r, d to X1, Y1 registers
	 *      (11) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
	 *      (12) Set MOPOP(CRPT_ECC_CTL[12:11]) to 01
	 *      (13) Set START(CRPT_ECC_CTL[0]) to 1
	 *      (14) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
	 *      (15) Write the curve order to N registers
	 *      (16) Write e to Y1 registers
	 *      (17) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
	 *      (18) Set MOPOP(CRPT_ECC_CTL[12:11]) to 10
	 *      (19) Set START(CRPT_ECC_CTL[0]) to 1
	 *      (20) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
	 *      (21) Write the curve order and curve length to N ,M registers
	 *      (22) Write k^-1 to Y1 registers
	 *      (23) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
	 *      (24) Set MOPOP(CRPT_ECC_CTL[12:11]) to 01
	 *      (25) Set START(CRPT_ECC_CTL[0]) to 1
	 *      (26) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
	 *      (27) Read X1 registers to get s
	 */

	/* S/W: GFp_add_mod_order(pCurve->key_len+2, 0, x1, a, R); */

	/*  4-(1) Write the curve order to N registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_N[i] = 0UL;
	Hex2Reg(pCurve->Eorder, crpt_regs->ECC_N);

	/*  4-(2) Write 0x1 to Y1 registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_Y1[i] = 0UL;
	crpt_regs->ECC_Y1[0] = 0x1UL;

	/*  4-(3) Write the random integer k to X1 registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_X1[i] = 0UL;
	Hex2Reg(ecc_k, crpt_regs->ECC_X1);

	if (run_ecc_codec(ECCOP_MODULE | MODOP_DIV) != 0)
		return -1;

	/* debug print */
	// Reg2Hex(pCurve->Echar, crpt->ECC_X1, temp_hex_str);
	// CRPT_DBGMSG("(7) output = %s\n", temp_hex_str);

	/*  4-(8) Read X1 registers to get k^-1 */

	for (i = 0; i < 18; i++)
		temp_result2[i] = crpt_regs->ECC_X1[i];

	/* debug print */
	// Reg2Hex(pCurve->Echar, temp_result2, temp_hex_str);
	// CRPT_DBGMSG("k^-1 = %s\n", temp_hex_str);

	/*  4-(9) Write the curve order and curve length to N ,M registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_N[i] = 0UL;
	Hex2Reg(pCurve->Eorder, crpt_regs->ECC_N);

	/*  4-(10) Write r, d to X1, Y1 registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_X1[i] = temp_result1[i];

	for (i = 0; i < 18; i++)
		crpt_regs->ECC_Y1[i] = 0UL;

	Hex2Reg(ecc_d, crpt_regs->ECC_Y1);

	if (run_ecc_codec(ECCOP_MODULE | MODOP_MUL) != 0)
		return -1;

	/* debug print */
	// Reg2Hex(pCurve->Echar, crpt->ECC_X1, temp_hex_str);
	// printk("(14) output = %s\n", temp_hex_str);

	/*  4-(15) Write the curve order to N registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_N[i] = 0UL;

	Hex2Reg(pCurve->Eorder, crpt_regs->ECC_N);

	/*  4-(16) Write e to Y1 registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_Y1[i] = 0UL;

	Hex2Reg(ecc_msg, crpt_regs->ECC_Y1);

	if (run_ecc_codec(ECCOP_MODULE | MODOP_ADD) != 0)
		return -1;

	/* debug print */
	// Reg2Hex(pCurve->Echar, crpt->ECC_X1, temp_hex_str);
	// CRPT_DBGMSG("(20) output = %s\n", temp_hex_str);

	/*  4-(21) Write the curve order and curve length to N ,M registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_N[i] = 0UL;

	Hex2Reg(pCurve->Eorder, crpt_regs->ECC_N);

	/*  4-(22) Write k^-1 to Y1 registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_Y1[i] = temp_result2[i];

	if (run_ecc_codec(ECCOP_MODULE | MODOP_MUL) != 0)
		return -1;

	/*  4-(27) Read X1 registers to get s */
	for (i = 0; i < 18; i++)
		temp_result2[i] = crpt_regs->ECC_X1[i];

	Reg2Hex(pCurve->Echar, temp_result2, ecc_s);
	return 0;
}

//int32_t  ECC_VerifySignature(CRPT_T *crpt, E_ECC_CURVE ecc_curve, char *message,
//                             char *public_k1, char *public_k2, char *R, char *S)
int  ECC_VerifySignature(void)
{
	volatile struct nuc980_crypto_regs  *crpt_regs = nuc980_crdev.regs;
	uint32_t  temp_result1[18], temp_result2[18];
	uint32_t  temp_x[18], temp_y[18];
	int32_t   i;

	/*
	 *   1. Verify that r and s are integers in the interval [1, n-1]. If not, the signature is invalid
	 *   2. Compute e = HASH (m), where HASH is the hashing algorithm in signature generation
	 *      (1) Use SHA to calculate e
	 */

	/*
	 *   3. Compute w = s^-1 (mod n)
	 *      (1) Write the curve order to N registers
	 *      (2) Write 0x1 to Y1 registers
	 *      (3) Write s to X1 registers
	 *      (4) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
	 *      (5) Set MOPOP(CRPT_ECC_CTL[12:11]) to 00
	 *      (6) Set FSEL(CRPT_ECC_CTL[8]) according to used curve of prime field or binary field
	 *      (7) Set START(CRPT_ECC_CTL[0]) to 1
	 *      (8) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
	 *      (9) Read X1 registers to get w
	 */

	if (ecc_init_curve(_curve_used) != 0)
		return -1;

	/*  3-(1) Write the curve order to N registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_N[i] = 0UL;

	Hex2Reg(pCurve->Eorder, crpt_regs->ECC_N);

	/*  3-(2) Write 0x1 to Y1 registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_Y1[i] = 0UL;

	crpt_regs->ECC_Y1[0] = 0x1UL;

	/*  3-(3) Write s to X1 registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_X1[i] = 0UL;

	Hex2Reg(ecc_s, crpt_regs->ECC_X1);

	if (run_ecc_codec(ECCOP_MODULE | MODOP_DIV) != 0)
		return -1;

	/*  3-(9) Read X1 registers to get w */
	for (i = 0; i < 18; i++)
		temp_result2[i] = crpt_regs->ECC_X1[i];

	/* debug print */
	//CRPT_DBGMSG("e = %s\n", message);
	//Reg2Hex(pCurve->Echar, temp_result2, temp_hex_str);
	//CRPT_DBGMSG("w = %s\n", temp_hex_str);
	//CRPT_DBGMSG("o = %s (order)\n", pCurve->Eorder);

	/*
	 *   4. Compute u1 = e \A1\D1 w (mod n) and u2 = r \A1\D1 w (mod n)
	 *      (1) Write the curve order and curve length to N ,M registers
	 *      (2) Write e, w to X1, Y1 registers
	 *      (3) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
	 *      (4) Set MOPOP(CRPT_ECC_CTL[12:11]) to 01
	 *      (5) Set START(CRPT_ECC_CTL[0]) to 1
	 *      (6) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
	 *      (7) Read X1 registers to get u1
	 *      (8) Write the curve order and curve length to N ,M registers
	 *      (9) Write r, w to X1, Y1 registers
	 *      (10) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
	 *      (11) Set MOPOP(CRPT_ECC_CTL[12:11]) to 01
	 *      (12) Set START(CRPT_ECC_CTL[0]) to 1
	 *      (13) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
	 *      (14) Read X1 registers to get u2
	 */

	/*  4-(1) Write the curve order and curve length to N ,M registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_N[i] = 0UL;

	Hex2Reg(pCurve->Eorder, crpt_regs->ECC_N);

	/* 4-(2) Write e, w to X1, Y1 registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_X1[i] = 0UL;

	Hex2Reg(ecc_msg, crpt_regs->ECC_X1);

	for (i = 0; i < 18; i++)
		crpt_regs->ECC_Y1[i] = temp_result2[i];

	if (run_ecc_codec(ECCOP_MODULE | MODOP_MUL) != 0)
		return -1;

	/*  4-(7) Read X1 registers to get u1 */
	for (i = 0; i < 18; i++)
		temp_result1[i] = crpt_regs->ECC_X1[i];

	/* debug print */
	//Reg2Hex(pCurve->Echar, temp_result1, temp_hex_str);
	//CRPT_DBGMSG("u1 = %s\n", temp_hex_str);

	/*  4-(8) Write the curve order and curve length to N ,M registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_N[i] = 0UL;

	Hex2Reg(pCurve->Eorder, crpt_regs->ECC_N);

	/* 4-(9) Write r, w to X1, Y1 registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_X1[i] = 0UL;

	Hex2Reg(ecc_r, crpt_regs->ECC_X1);

	for (i = 0; i < 18; i++)
		crpt_regs->ECC_Y1[i] = temp_result2[i];

	run_ecc_codec(ECCOP_MODULE | MODOP_MUL);

	/*  4-(14) Read X1 registers to get u2 */
	for (i = 0; i < 18; i++)
		temp_result2[i] = crpt_regs->ECC_X1[i];

	/* debug print */
	//Reg2Hex(pCurve->Echar, temp_result2, temp_hex_str);
	//CRPT_DBGMSG("u2 = %s\n", temp_hex_str);

	/*
	 *   5. Compute X\A1\A6 (x1\A1\A6, y1\A1\A6) = u1 * G + u2 * Q
	 *      (1) Write the curve parameter A, B, N, and curve length M to corresponding registers
	 *      (2) Write the point G(x, y) to X1, Y1 registers
	 *      (3) Write u1 to K registers
	 *      (4) Set ECCOP(CRPT_ECC_CTL[10:9]) to 00
	 *      (5) Set START(CRPT_ECC_CTL[0]) to 1
	 *      (6) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
	 *      (7) Read X1, Y1 registers to get u1*G
	 *      (8) Write the curve parameter A, B, N, and curve length M to corresponding registers
	 *      (9) Write the public key Q(x,y) to X1, Y1 registers
	 *      (10) Write u2 to K registers
	 *      (11) Set ECCOP(CRPT_ECC_CTL[10:9]) to 00
	 *      (12) Set START(CRPT_ECC_CTL[0]) to 1
	 *      (13) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
	 *      (14) Write the curve parameter A, B, N, and curve length M to corresponding registers
	 *      (15) Write the result data u1*G to X2, Y2 registers
	 *      (16) Set ECCOP(CRPT_ECC_CTL[10:9]) to 10
	 *      (17) Set START(CRPT_ECC_CTL[0]) to 1
	 *      (18) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
	 *      (19) Read X1, Y1 registers to get X\A1\A6(x1\A1\A6, y1\A1\A6)
	 *      (20) Write the curve order and curve length to N ,M registers
	 *      (21) Write x1\A1\A6 to X1 registers
	 *      (22) Write 0x0 to Y1 registers
	 *      (23) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
	 *      (24) Set MOPOP(CRPT_ECC_CTL[12:11]) to 10
	 *      (25) Set START(CRPT_ECC_CTL[0]) to 1
	 *      (26) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
	 *      (27) Read X1 registers to get x1\A1\A6 (mod n)
	 *
	 *   6. The signature is valid if x1\A1\A6 = r, otherwise it is invalid
	 */

	/*
	 *  (1) Write the curve parameter A, B, N, and curve length M to corresponding registers
	 *  (2) Write the point G(x, y) to X1, Y1 registers
	 */
	ecc_init_curve(_curve_used);

	/* (3) Write u1 to K registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_K[i] = temp_result1[i];

	if (run_ecc_codec(ECCOP_POINT_MUL) != 0)
		return -1;

	/* (7) Read X1, Y1 registers to get u1*G */
	for (i = 0; i < 18; i++) {
		temp_x[i] = crpt_regs->ECC_X1[i];
		temp_y[i] = crpt_regs->ECC_Y1[i];
	}

	/* debug print */
	//Reg2Hex(pCurve->Echar, temp_x, temp_hex_str);
	//CRPT_DBGMSG("5-(7) u1*G, x = %s\n", temp_hex_str);
	//Reg2Hex(pCurve->Echar, temp_y, temp_hex_str);
	//CRPT_DBGMSG("5-(7) u1*G, y = %s\n", temp_hex_str);

	/* (8) Write the curve parameter A, B, N, and curve length M to corresponding registers */
	ecc_init_curve(_curve_used);

	/* (9) Write the public key Q(x,y) to X1, Y1 registers */
	for (i = 0; i < 18; i++) {
		crpt_regs->ECC_X1[i] = 0UL;
		crpt_regs->ECC_Y1[i] = 0UL;
	}

	Hex2Reg(ecc_x1, crpt_regs->ECC_X1);
	Hex2Reg(ecc_y1, crpt_regs->ECC_Y1);

	/* (10) Write u2 to K registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_K[i] = temp_result2[i];

	if (run_ecc_codec(ECCOP_POINT_MUL) != 0)
		return -1;

	for (i = 0; i < 18; i++) {
		temp_result1[i] = crpt_regs->ECC_X1[i];
		temp_result2[i] = crpt_regs->ECC_Y1[i];
	}

	/* debug print */
	//Reg2Hex(pCurve->Echar, temp_result1, temp_hex_str);
	//CRPT_DBGMSG("5-(13) u2*Q, x = %s\n", temp_hex_str);
	//Reg2Hex(pCurve->Echar, temp_result2, temp_hex_str);
	//CRPT_DBGMSG("5-(13) u2*Q, y = %s\n", temp_hex_str);

	/* (14) Write the curve parameter A, B, N, and curve length M to corresponding registers */
	ecc_init_curve(_curve_used);

	/* Write the result data u2*Q to X1, Y1 registers */
	for (i = 0; i < 18; i++) {
		crpt_regs->ECC_X1[i] = temp_result1[i];
		crpt_regs->ECC_Y1[i] = temp_result2[i];
	}

	/* (15) Write the result data u1*G to X2, Y2 registers */
	for (i = 0; i < 18; i++) {
		crpt_regs->ECC_X2[i] = temp_x[i];
		crpt_regs->ECC_Y2[i] = temp_y[i];
	}

	if (run_ecc_codec(ECCOP_POINT_ADD) != 0)
		return -1;

	/* (19) Read X1, Y1 registers to get X\A1\A6(x1\A1\A6, y1\A1\A6) */
	for (i = 0; i < 18; i++) {
		temp_x[i] = crpt_regs->ECC_X1[i];
		temp_y[i] = crpt_regs->ECC_Y1[i];
	}

	/* debug print */
	//Reg2Hex(pCurve->Echar, temp_x, temp_hex_str);
	//CRPT_DBGMSG("5-(19) x' = %s\n", temp_hex_str);
	//Reg2Hex(pCurve->Echar, temp_y, temp_hex_str);
	//CRPT_DBGMSG("5-(19) y' = %s\n", temp_hex_str);

	/*  (20) Write the curve order and curve length to N ,M registers */
	for (i = 0; i < 18; i++)
		crpt_regs->ECC_N[i] = 0UL;

	Hex2Reg(pCurve->Eorder, crpt_regs->ECC_N);

	/*
	 *  (21) Write x1\A1\A6 to X1 registers
	 *  (22) Write 0x0 to Y1 registers
	 */
	for (i = 0; i < 18; i++) {
		crpt_regs->ECC_X1[i] = temp_x[i];
		crpt_regs->ECC_Y1[i] = 0UL;
	}

	/* debug print */
	//Reg2Hex(pCurve->Echar, crpt->ECC_X1, temp_hex_str);
	//CRPT_DBGMSG("5-(21) x' = %s\n", temp_hex_str);
	//Reg2Hex(pCurve->Echar, crpt->ECC_Y1, temp_hex_str);
	//CRPT_DBGMSG("5-(22) y' = %s\n", temp_hex_str);

	if (run_ecc_codec(ECCOP_MODULE | MODOP_ADD) != 0)
		return -1;

	/*  (27) Read X1 registers to get x1\A1\A6 (mod n) */
	Reg2Hex(pCurve->Echar, crpt_regs->ECC_X1, temp_hex_str);
	//printk("5-(27) x1' (mod n) = %s\n", temp_hex_str);

	/* 6. The signature is valid if x1\A1\A6 = r, otherwise it is invalid */

	/* Compare with test pattern to check if r is correct or not */
	if (ecc_strcmp(temp_hex_str, ecc_r) != 0) {
		printk("x1' (mod n) != R Test filed!!\n");
		printk("Signature R [%s] is not matched with expected R [%s]!\n", temp_hex_str, ecc_r);
		return -2;
	}
	return 0;
}

static long nvt_ecc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch(cmd) {
	case ECC_IOC_SEL_CURVE:
		if (get_curve(arg) == NULL)
			return -1;
		_curve_used = arg;
		if (ecc_init_curve(_curve_used) != 0)
			return -1;
		//printk("Select curve 0x%x\n", pCurve->curve_id);
		break;

	case ECC_IOC_SET_PRI_KEY:
		memset(ecc_d, 0, sizeof(ecc_d));
		copy_from_user(ecc_d, (char *)arg, pCurve->Echar);
		break;

	case ECC_IOC_SET_PUB_K1:
		memset(ecc_x1, 0, sizeof(ecc_x1));
		copy_from_user(ecc_x1, (char *)arg, pCurve->Echar);
		break;

	case ECC_IOC_SET_PUB_K2:
		memset(ecc_y1, 0, sizeof(ecc_y1));
		copy_from_user(ecc_y1, (char *)arg, pCurve->Echar);
		break;

	case ECC_IOC_SET_SCALAR_K:
		memset(ecc_k, 0, sizeof(ecc_k));
		copy_from_user(ecc_k, (char *)arg, pCurve->Echar);
		break;

	case ECC_IOC_SET_MSG:
		memset(ecc_msg, 0, sizeof(ecc_msg));
		copy_from_user(ecc_msg, (char *)arg, pCurve->Echar);
		break;

	case ECC_IOC_SET_SIG_R:
		memset(ecc_r, 0, sizeof(ecc_r));
		copy_from_user(ecc_r, (char *)arg, pCurve->Echar);
		break;

	case ECC_IOC_SET_SIG_S:
		memset(ecc_s, 0, sizeof(ecc_s));
		copy_from_user(ecc_s, (char *)arg, pCurve->Echar);
		break;

	case ECC_IOC_GET_PUB_K1:
		copy_to_user((char *)arg, ecc_x1, pCurve->Echar);
		break;

	case ECC_IOC_GET_PUB_K2:
		copy_to_user((char *)arg, ecc_y1, pCurve->Echar);
		break;

	case ECC_IOC_GET_SIG_R:
		copy_to_user((char *)arg, ecc_r, pCurve->Echar);
		break;

	case ECC_IOC_GET_SIG_S:
		copy_to_user((char *)arg, ecc_s, pCurve->Echar);
		break;

	case ECC_IOC_GEN_PUB_KEY:
		return ECC_GeneratePublicKey();

	case ECC_IOC_ECDSA_SIGN:
		return ECC_GenerateSignature();

	case ECC_IOC_ECDSA_VERIFY:
		return ECC_VerifySignature();
		
	case ECC_IOC_POINT_MUL:
		return ECC_Mutiply();

	default:
		return -ENOTTY;
	}
	return 0;
}

struct file_operations nvt_ecc_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= nvt_ecc_ioctl,
};

static struct miscdevice nvt_ecc_dev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "nuvoton-ecc",
	.fops		= &nvt_ecc_fops,
};


/*-----------------------------------------------------------------------------------------------*/
/*                                                                                               */
/*    RSA                                                                                        */
/*                                                                                               */
/*-----------------------------------------------------------------------------------------------*/

#define RSA_MAX_KLEN      (2048)
#define RSA_KBUF_HLEN     (RSA_MAX_KLEN/4 + 8)
#define RSA_KBUF_BLEN     (RSA_MAX_KLEN + 32)

static int   g_rsa_len = RSA_MAX_KLEN;
static char  g_rsa_N[RSA_KBUF_HLEN];
static char  g_rsa_E[RSA_KBUF_HLEN];
static char  g_rsa_d[RSA_KBUF_HLEN];
static char  g_rsa_C[RSA_KBUF_HLEN];
static char  g_rsa_Msg[RSA_KBUF_HLEN];
static char  g_rsa_Sig[RSA_KBUF_HLEN];

/**
  * @brief  RSA digital signature generation.
  * @param[in]  rsa_len     RSA key length
  * @param[in]  n           The modulus for both the public and private keys
  * @param[in]  d           (n,d) is the private key
  * @param[in]  C           The constant value of Montgomery domain.
  * @param[in]  msg         The message to be signed.
  * @param[out] sign        The output signature.
  * @return  0     Success.
  * @return  -1    Error
  */
int  RSA_GenerateSignature(int rsa_len, char *n, char *d, char *C, char *msg, char *sig)
{
	volatile struct nuc980_crypto_regs  *crpt_regs = nuc980_crdev.regs;
	int  i;
	
	for (i = 0; i < 128; i++)
	{
		crpt_regs->RSA_N[i] = 0;
		crpt_regs->RSA_E[i] = 0;
		crpt_regs->RSA_M[i] = 0;
	}
	
	Hex2Reg(n, (uint32_t *)&crpt_regs->RSA_N[0]);
	Hex2Reg(d, (uint32_t *)&crpt_regs->RSA_E[0]);
	Hex2Reg(msg, (uint32_t *)&crpt_regs->RSA_M[0]);
	Hex2Reg(C, (uint32_t *)&crpt_regs->RSA_C[0]);
	
	crpt_regs->RSA_CTL = (rsa_len << CRPT_RSA_CTL_KEYLEN_Pos) | CRPT_RSA_CTL_START_Msk;
	while (crpt_regs->RSA_STS & CRPT_RSA_STS_BUSY_Msk) ;

	Reg2Hex(rsa_len/4, (uint32_t *)crpt_regs->RSA_M, sig);
	return 0;
}

/**
  * @brief  RSA digital signature generation.
  * @param[in]  rsa_len     RSA key length
  * @param[in]  n           The modulus for both the public and private keys
  * @param[in]  e           (n,e) is the public key
  * @param[in]  C           The constant value of Montgomery domain.
  * @param[in]  sign        The signature to be verified.
  * @param[out] msg         The message to be compared.
  * @return  0     Success.
  * @return  -1    Verify failed
  */
int RSA_VerifySignature(int rsa_len, char *n, char *e, char *C, char *sig, char *msg)
{
	volatile struct nuc980_crypto_regs  *crpt_regs = nuc980_crdev.regs;
	char output[RSA_KBUF_HLEN];
	int  i;
	
	for (i = 0; i < 128; i++)
	{
		crpt_regs->RSA_N[i] = 0;
		crpt_regs->RSA_E[i] = 0;
		crpt_regs->RSA_M[i] = 0;
	}
	
	Hex2Reg(n, (uint32_t *)&crpt_regs->RSA_N[0]);
	Hex2Reg(e, (uint32_t *)&crpt_regs->RSA_E[0]);
	Hex2Reg(sig, (uint32_t *)&crpt_regs->RSA_M[0]);
	Hex2Reg(C, (uint32_t *)&crpt_regs->RSA_C[0]);
	
	crpt_regs->RSA_CTL = (rsa_len << CRPT_RSA_CTL_KEYLEN_Pos) | CRPT_RSA_CTL_START_Msk;
	while (crpt_regs->RSA_STS & CRPT_RSA_STS_BUSY_Msk) ;

	Reg2Hex(rsa_len/4, (uint32_t *)crpt_regs->RSA_M, output);
	
	//printk("RSA verify: %s\n", output);
	
	if (ecc_strcmp(output, msg) != 0)
	{
		printk("RSA verify output [%s] is not matched with expected [%s]!\n", output, msg);
		return -1;
	}
	return 0;
}

static long nvt_rsa_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch(cmd) {
	case RSA_IOC_SET_BIT_LEN:
		if (arg & 0xf) return -1;
		if ((arg < 1024) || (arg > 2048)) return -1;
		g_rsa_len = arg;
		break;

	case RSA_IOC_SET_N:
		memset(g_rsa_N, 0, sizeof(g_rsa_N));
		copy_from_user(g_rsa_N, (char *)arg, g_rsa_len/4);
		break;

	case RSA_IOC_SET_E:
		memset(g_rsa_E, 0, sizeof(g_rsa_E));
		copy_from_user(g_rsa_E, (char *)arg, g_rsa_len/4);
		break;

	case RSA_IOC_SET_D:
		memset(g_rsa_d, 0, sizeof(g_rsa_d));
		copy_from_user(g_rsa_d, (char *)arg, g_rsa_len/4);
		break;

	case RSA_IOC_SET_C:
		memset(g_rsa_C, 0, sizeof(g_rsa_C));
		copy_from_user(g_rsa_C, (char *)arg, g_rsa_len/4);
		break;

	case RSA_IOC_SET_MSG:
		memset(g_rsa_Msg, 0, sizeof(g_rsa_Msg));
		copy_from_user(g_rsa_Msg, (char *)arg, g_rsa_len/4);
		break;

	case RSA_IOC_SET_SIG:
		memset(g_rsa_Sig, 0, sizeof(g_rsa_Sig));
		copy_from_user(g_rsa_Sig, (char *)arg, g_rsa_len/4);
		break;

	case RSA_IOC_GET_MSG:
		copy_to_user((char *)arg, g_rsa_Msg, g_rsa_len/4);
		break;

	case RSA_IOC_GET_SIG:
		copy_to_user((char *)arg, g_rsa_Sig, g_rsa_len/4);
		break;
		
	case RSA_IOC_DO_SIGN:
		return RSA_GenerateSignature(g_rsa_len, g_rsa_N, g_rsa_d, g_rsa_C, g_rsa_Msg, g_rsa_Sig);

	case RSA_IOC_DO_VERIFY:
		return RSA_VerifySignature(g_rsa_len, g_rsa_N, g_rsa_E, g_rsa_C, g_rsa_Sig, g_rsa_Msg);

	default:
		return -ENOTTY;
	}
	return 0;
}

struct file_operations nvt_rsa_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= nvt_rsa_ioctl,
};

static struct miscdevice nvt_rsa_dev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "nuvoton-rsa",
	.fops		= &nvt_rsa_fops,
};

static int nuc980_crypto_raw_probe(struct platform_device *pdev)
{
	misc_register(&nvt_aes_dev);
	misc_register(&nvt_sha_dev);
	misc_register(&nvt_ecc_dev);
	misc_register(&nvt_rsa_dev);
	return 0;
}

static int nuc980_crypto_raw_remove(struct platform_device *pdev)
{
	misc_deregister(&nvt_aes_dev);
	misc_deregister(&nvt_sha_dev);
	misc_deregister(&nvt_ecc_dev);
	misc_deregister(&nvt_rsa_dev);
	return 0;
}


static const struct of_device_id nuc980_crypto_raw_of_match[] = {
	{ .compatible = "nuvoton,nuc980-crypto-raw" },
	{},
};
MODULE_DEVICE_TABLE(of, nuc980_crypto_raw_of_match);


static struct platform_driver nuc980_crypto_raw_driver = {
	.probe      = nuc980_crypto_raw_probe,
	.remove     = nuc980_crypto_raw_remove,
	.driver     = {
		.name   = "nuc980-crypto-raw",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(nuc980_crypto_raw_of_match),
	},
};

module_platform_driver(nuc980_crypto_raw_driver);

MODULE_AUTHOR("Nuvoton Technology Corporation");
MODULE_DESCRIPTION("NUC980 Cryptographic Accerlerator Raw");
MODULE_LICENSE("GPL");
