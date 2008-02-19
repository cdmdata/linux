/*
 * This file contains an ECC algorithm from Toshiba that detects and
 * corrects 1 bit errors in a 256 byte block of data.
 *
 * drivers/mtd/nand/nand_ecc.c
 *
 * Copyright (C) 2000-2004 Steven J. Hill (sjhill@realitydiluted.com)
 *                         Toshiba America Electronics Components, Inc.
 *
 * Copyright (C) 2006 Thomas Gleixner <tglx@linutronix.de>
 *
 * This file is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 or (at your option) any
 * later version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this file; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * As a special exception, if other files instantiate templates or use
 * macros or inline functions from these files, or you compile these
 * files and link them with other works to produce a work based on these
 * files, these files do not by themselves cause the resulting work to be
 * covered by the GNU General Public License. However the source code for
 * these files must still be made available in accordance with section (3)
 * of the GNU General Public License.
 *
 * This exception does not invalidate any other reasons why a work based on
 * this file might be covered by the GNU General Public License.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>

//#define VERIFY_NEW_ECC_ALG
#ifdef VERIFY_NEW_ECC_ALG
/*
 * Pre-calculated 256-way 1 byte column parity
 */
static const u_char nand_ecc_precalc_table[] = {
	0x00, 0x55, 0x56, 0x03, 0x59, 0x0c, 0x0f, 0x5a, 0x5a, 0x0f, 0x0c, 0x59, 0x03, 0x56, 0x55, 0x00,
	0x65, 0x30, 0x33, 0x66, 0x3c, 0x69, 0x6a, 0x3f, 0x3f, 0x6a, 0x69, 0x3c, 0x66, 0x33, 0x30, 0x65,
	0x66, 0x33, 0x30, 0x65, 0x3f, 0x6a, 0x69, 0x3c, 0x3c, 0x69, 0x6a, 0x3f, 0x65, 0x30, 0x33, 0x66,
	0x03, 0x56, 0x55, 0x00, 0x5a, 0x0f, 0x0c, 0x59, 0x59, 0x0c, 0x0f, 0x5a, 0x00, 0x55, 0x56, 0x03,
	0x69, 0x3c, 0x3f, 0x6a, 0x30, 0x65, 0x66, 0x33, 0x33, 0x66, 0x65, 0x30, 0x6a, 0x3f, 0x3c, 0x69,
	0x0c, 0x59, 0x5a, 0x0f, 0x55, 0x00, 0x03, 0x56, 0x56, 0x03, 0x00, 0x55, 0x0f, 0x5a, 0x59, 0x0c,
	0x0f, 0x5a, 0x59, 0x0c, 0x56, 0x03, 0x00, 0x55, 0x55, 0x00, 0x03, 0x56, 0x0c, 0x59, 0x5a, 0x0f,
	0x6a, 0x3f, 0x3c, 0x69, 0x33, 0x66, 0x65, 0x30, 0x30, 0x65, 0x66, 0x33, 0x69, 0x3c, 0x3f, 0x6a,
	0x6a, 0x3f, 0x3c, 0x69, 0x33, 0x66, 0x65, 0x30, 0x30, 0x65, 0x66, 0x33, 0x69, 0x3c, 0x3f, 0x6a,
	0x0f, 0x5a, 0x59, 0x0c, 0x56, 0x03, 0x00, 0x55, 0x55, 0x00, 0x03, 0x56, 0x0c, 0x59, 0x5a, 0x0f,
	0x0c, 0x59, 0x5a, 0x0f, 0x55, 0x00, 0x03, 0x56, 0x56, 0x03, 0x00, 0x55, 0x0f, 0x5a, 0x59, 0x0c,
	0x69, 0x3c, 0x3f, 0x6a, 0x30, 0x65, 0x66, 0x33, 0x33, 0x66, 0x65, 0x30, 0x6a, 0x3f, 0x3c, 0x69,
	0x03, 0x56, 0x55, 0x00, 0x5a, 0x0f, 0x0c, 0x59, 0x59, 0x0c, 0x0f, 0x5a, 0x00, 0x55, 0x56, 0x03,
	0x66, 0x33, 0x30, 0x65, 0x3f, 0x6a, 0x69, 0x3c, 0x3c, 0x69, 0x6a, 0x3f, 0x65, 0x30, 0x33, 0x66,
	0x65, 0x30, 0x33, 0x66, 0x3c, 0x69, 0x6a, 0x3f, 0x3f, 0x6a, 0x69, 0x3c, 0x66, 0x33, 0x30, 0x65,
	0x00, 0x55, 0x56, 0x03, 0x59, 0x0c, 0x0f, 0x5a, 0x5a, 0x0f, 0x0c, 0x59, 0x03, 0x56, 0x55, 0x00
};

/**
 * nand_calculate_ecc - [NAND Interface] Calculate 3-byte ECC for 256-byte block
 * @mtd:	MTD block structure
 * @dat:	raw data
 * @ecc_code:	buffer for ECC
 */
int nand_calculate_ecc_old(struct mtd_info *mtd, const u_char *dat,
		       u_char *ecc_code)
{
	uint8_t idx, reg1, reg2, reg3, tmp1, tmp2;
	int i;

	/* Initialize variables */
	reg1 = reg2 = reg3 = 0;

	/* Build up column parity */
	for(i = 0; i < 256; i++) {
		/* Get CP0 - CP5 from table */
		idx = nand_ecc_precalc_table[*dat++];
		reg1 ^= (idx & 0x3f);

		/* All bit XOR = 1 ? */
		if (idx & 0x40) {
			reg3 ^= (uint8_t) i;
			reg2 ^= ~((uint8_t) i);
		}
	}

	/* Create non-inverted ECC code from line parity */
	tmp1  = (reg3 & 0x80) >> 0; /* B7 -> B7 */
	tmp1 |= (reg2 & 0x80) >> 1; /* B7 -> B6 */
	tmp1 |= (reg3 & 0x40) >> 1; /* B6 -> B5 */
	tmp1 |= (reg2 & 0x40) >> 2; /* B6 -> B4 */
	tmp1 |= (reg3 & 0x20) >> 2; /* B5 -> B3 */
	tmp1 |= (reg2 & 0x20) >> 3; /* B5 -> B2 */
	tmp1 |= (reg3 & 0x10) >> 3; /* B4 -> B1 */
	tmp1 |= (reg2 & 0x10) >> 4; /* B4 -> B0 */

	tmp2  = (reg3 & 0x08) << 4; /* B3 -> B7 */
	tmp2 |= (reg2 & 0x08) << 3; /* B3 -> B6 */
	tmp2 |= (reg3 & 0x04) << 3; /* B2 -> B5 */
	tmp2 |= (reg2 & 0x04) << 2; /* B2 -> B4 */
	tmp2 |= (reg3 & 0x02) << 2; /* B1 -> B3 */
	tmp2 |= (reg2 & 0x02) << 1; /* B1 -> B2 */
	tmp2 |= (reg3 & 0x01) << 1; /* B0 -> B1 */
	tmp2 |= (reg2 & 0x01) << 0; /* B7 -> B0 */

	/* Calculate final ECC code */
#ifdef CONFIG_MTD_NAND_ECC_SMC
	ecc_code[0] = ~tmp2;
	ecc_code[1] = ~tmp1;
#else
	ecc_code[0] = ~tmp1;
	ecc_code[1] = ~tmp2;
#endif
	ecc_code[2] = ((~reg1) << 2) | 0x03;

	return 0;
}
#endif

#ifdef CONFIG_MTD_NAND_ECC_SMC
#define LOW_ORDER_INDEX 0
#define HIGH_ORDER_INDEX 1
#else
#define LOW_ORDER_INDEX 1
#define HIGH_ORDER_INDEX 0
#endif

/**
 * nand_calculate_ecc - [NAND Interface] Calculate 3-byte ECC for 256/512 byte block
 * @mtd:	MTD block structure
 * @dat:	raw data
 * @ecc_code:	buffer for ECC
 */
int nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
		       u_char *ecc_code)
{
	uint32_t j = ((struct nand_chip *)mtd->priv)->ecc.size;			/* 256 or 512 bytes/ecc  */
	uint32_t k=0;
	uint32_t xor = 0;
	uint32_t tecc = 0;
	uint32_t ecc = 0;
	uint32_t * p = (uint32_t *)dat;
	uint32_t v;
//#define FORCE_ECC_ERROR
#ifdef FORCE_ECC_ERROR
	uint32_t forceBitNum;
	{
		/* force single bit ecc error to test ecc correction code */
		u_char* p = (u_char*)dat;
		forceBitNum = p[0] | (p[1]<<8);	/* 1st word of block determines which bit is made in error */
		forceBitNum &= 0xfff;
		if ((forceBitNum&0x800)==0) {
			/* limit error to 1st 256 bytes */
			p[forceBitNum>>3] ^= 1<<(forceBitNum&7);
			printk(KERN_INFO "Forcing ecc error, byte:0x%x, bit:%i\n",forceBitNum>>3,forceBitNum&7);
		}
	}
#endif

	do {
		v = *p++;
		xor ^= v;
		v ^= (v>>16);
		v ^= (v>>8);
		v ^= (v>>4);
		v ^= (v>>2);
		v ^= (v>>1);
		if (v&1) tecc ^= k;
		k++;
		j-=4;
	} while (j);
	__cpu_to_le64s(xor);
	v = (xor>>16)^xor;
	v ^= ((v>>8)&(0x00ff00ff&0x00ffffff));
	v ^= ((v>>4)&(0x0f0f0f0f&0x000f0fff));
	v ^= ((v>>2)&(0x33333333&0x0003033f));
	v ^= ((v>>1)&(0x55555555&0x00010117));
	
	/* now duplicate all bits */
	if (tecc&(1<<6)) ecc ^= 3<<22;
	if (tecc&(1<<5)) ecc ^= 3<<20;
	if (tecc&(1<<4)) ecc ^= 3<<18;
	if (tecc&(1<<3)) ecc ^= 3<<16;
	if (tecc&(1<<2)) ecc ^= 3<<14;
	if (tecc&(1<<1)) ecc ^= 3<<12;
	if (tecc&(1<<0)) ecc ^= 3<<10;

	if (v&(1<<16)) ecc ^= 3<<8;
	if (v&(1<<8)) ecc ^= 3<<6;
	if (v&(1<<4)) ecc ^= 3<<4;
	if (v&(1<<2)) ecc ^= 3<<2;
	if (v&(1<<1)) ecc ^= 3<<0;
	if (v&(1<<0)) ecc ^= 0x555555;		/* if parity is odd, low bits are opposite of high bits */

#ifdef FORCE_ECC_ERROR
	if (forceBitNum&0x800) {
		ecc ^= 1<<(forceBitNum&0xf);
		printk(KERN_INFO "Forcing single bit error in ecc itself bit %i\n",forceBitNum&0xf);
	}
#endif
	if (((struct nand_chip *)mtd->priv)->ecc.size==256) ecc <<= 2;
	ecc = ~ecc;


#ifdef VERIFY_NEW_ECC_ALG
	{
		uint32_t s;
		nand_calculate_ecc_old(mtd,dat,ecc_code);
		ecc &= 0xffffff;
		s = (ecc_code[HIGH_ORDER_INDEX]<<16) | (ecc_code[LOW_ORDER_INDEX]<<8) | ecc_code[2];

		if (s != ecc) {
			printk(KERN_ERR "New algorithm is buggy!!!! s=%x, ecc=%x\n",s,ecc);
			return 0;
		}
	}
#endif

	/* Calculate final ECC code */
	ecc_code[HIGH_ORDER_INDEX] = (u_char)(ecc>>16);
	ecc_code[LOW_ORDER_INDEX] = (u_char)(ecc>>8);
	ecc_code[2] = (u_char)ecc;
	return 0;
}

EXPORT_SYMBOL(nand_calculate_ecc);


/**
 * nand_correct_data - [NAND Interface] Detect and correct bit error(s)
 * @mtd:	MTD block structure
 * @dat:	raw data read from the chip
 * @read_ecc:	ECC from the chip
 * @calc_ecc:	the ECC calculated from raw data
 *
 * Detect and correct a 1 bit error for 256/512 byte block
 */
int nand_correct_data(struct mtd_info *mtd, u_char *dat,
		      u_char *read_ecc, u_char *calc_ecc)
{
	uint32_t s,t;
	s = (calc_ecc[HIGH_ORDER_INDEX]<<16) | (calc_ecc[LOW_ORDER_INDEX]<<8) | calc_ecc[2];
	t = (read_ecc[HIGH_ORDER_INDEX]<<16) | (read_ecc[LOW_ORDER_INDEX]<<8) | read_ecc[2];

	s ^= t;
	if (s == 0) {
#ifdef FORCE_ECC_ERROR
		printk(KERN_ERR "Trying to force an error failed\n");
#endif
		return 0;
	}

	if (((struct nand_chip *)mtd->priv)->ecc.size==256) {
		if ((s&3)==0) s ^= (1<<24);
		s >>= 2;
	}

	/* Check for a single bit error */
	if( ((s ^ (s >> 1)) & 0x555555) == 0x555555) {

		uint32_t byteoffs, bitnum;

		byteoffs = (s >> (23-8)) & 0x100;	/* bit 23 used for 512 byte eccsize */
		byteoffs |= (s >> (21-7)) & 0x80;
		byteoffs |= (s >> (19-6)) & 0x40;
		byteoffs |= (s >> (17-5)) & 0x20;
		byteoffs |= (s >> (15-4)) & 0x10;
		byteoffs |= (s >> (13-3)) & 0x08;
		byteoffs |= (s >> (11-2)) & 0x04;
		byteoffs |= (s >> (9-1)) & 0x02;
		byteoffs |= (s >> (7-0)) & 0x01;

		bitnum = (s >> (5-2)) & 0x04;
		bitnum |= (s >> (3-1)) & 0x02;
		bitnum |= (s >> (1-0)) & 0x01;

		dat[byteoffs] ^= (1 << bitnum);
#ifdef FORCE_ECC_ERROR
		printk(KERN_INFO "Correcting FORCED single bit ECC error at offset: 0x%x, bit: %i\n", byteoffs, bitnum);
#endif
		return 1;
	}

	if (((struct nand_chip *)mtd->priv)->ecc.size==256) {
		s &= 0x3fffff;	/* we may have set bit 22 above, make sure it's clear now */
	}
	if ((s & (-s))==s) {
#ifdef FORCE_ECC_ERROR
		printk(KERN_INFO "Detected single bit error in ECC itself 0x%x\n",s);
#endif
		return 1;			/* Single bit ECC error in the ECC itself, nothing to fix */
	}
#ifdef FORCE_ECC_ERROR
	printk(KERN_ERR "unrecoverable error FORCED 0x%x\n",s);
#endif
	return -EBADMSG;
}
EXPORT_SYMBOL(nand_correct_data);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Steven J. Hill <sjhill@realitydiluted.com>");
MODULE_DESCRIPTION("Generic NAND ECC support");
