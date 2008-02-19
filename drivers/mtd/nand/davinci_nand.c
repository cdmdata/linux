/*
 * linux/drivers/mtd/nand/davinci_nand.c
 *
 * NAND Flash Driver
 *
 * Copyright (C) 2006 Texas Instruments.
 *
 * ported to 2.6.23 (C) 2008 by
 * Sander Huijsen <Shuijsen@optelecom-nkf.com>
 * Troy Kisky <troy.kisky@boundarydevices.com>
 * Dirk Behme <Dirk.Behme@gmail.com>
 *
 * --------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * --------------------------------------------------------------------------
 *
 *  Overview:
 *   This is a device driver for the NAND flash device found on the
 *   DaVinci board which utilizes the Samsung k9k2g08 part.
 *
 *  Modifications:
 *  ver. 1.0: Feb 2005, Vinod/Sudhakar
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <mach/hardware.h>
#include <mach/nand.h>
#include <mach/mux.h>

#include <asm/mach/flash.h>

#ifdef CONFIG_NAND_FLASH_HW_ECC
#define DAVINCI_NAND_ECC_MODE NAND_ECC_HW3_512
#else
#define DAVINCI_NAND_ECC_MODE NAND_ECC_SOFT
#endif

#define DRIVER_NAME "davinci_nand"

static struct clk *nand_clock;
static void __iomem *nand_vaddr;

/*
 * MTD structure for DaVinici board
 */
static struct mtd_info *nand_davinci_mtd;
static struct completion cmd_complete;
static int irq_enabled;


#ifdef CONFIG_MTD_PARTITIONS
const char *part_probes[] = { "cmdlinepart", NULL };
#endif

static uint8_t scan_ff_pattern[] = { 0xff, 0xff };

/* BB marker is byte 5 in OOB of page 0 */
static struct nand_bbt_descr davinci_memorybased_small = {
	.options = NAND_BBT_SCAN2NDPAGE,
	.offs = 5,
	.len = 1,
	.pattern = scan_ff_pattern
};

/* BB marker is bytes 0-1 in OOB of page 0 */
static struct nand_bbt_descr davinci_memorybased_large = {
	.options = 0,
	.offs = 0,
	.len = 2,
	.pattern = scan_ff_pattern
};

inline unsigned int davinci_nand_readl(int offset)
{
	return davinci_readl(DAVINCI_ASYNC_EMIF_CNTRL_BASE + offset);
}

inline void davinci_nand_writel(unsigned long value, int offset)
{
	davinci_writel(value, DAVINCI_ASYNC_EMIF_CNTRL_BASE + offset);
}

/*
 * Hardware specific access to control-lines
 */
static void nand_davinci_hwcontrol(struct mtd_info *mtd, int cmd,
				   unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;
	u32 IO_ADDR_W = (u32)chip->IO_ADDR_W;

	/* Did the control lines change? */
	if (ctrl & NAND_CTRL_CHANGE) {
		IO_ADDR_W &= ~(MASK_ALE|MASK_CLE);

		if ((ctrl & NAND_CTRL_CLE) == NAND_CTRL_CLE)
			IO_ADDR_W |= MASK_CLE;
		else if ((ctrl & NAND_CTRL_ALE) == NAND_CTRL_ALE)
			IO_ADDR_W |= MASK_ALE;

		chip->IO_ADDR_W = (void __iomem *)IO_ADDR_W;
	}

	if (cmd != NAND_CMD_NONE)
		writeb(cmd, chip->IO_ADDR_W);
}

static void nand_davinci_select_chip(struct mtd_info *mtd, int chip)
{
	/* do nothing */
}

#ifdef CONFIG_NAND_FLASH_HW_ECC
static void nand_davinci_enable_hwecc(struct mtd_info *mtd, int mode)
{
	u32 retval;

	/* Reset ECC hardware */
	retval = davinci_nand_readl(NANDF1ECC_OFFSET);

	/* Restart ECC hardware */
	retval = davinci_nand_readl(NANDFCR_OFFSET);
	retval |= (1 << 8);
	davinci_nand_writel(retval, NANDFCR_OFFSET);
}

/*
 * Read DaVinci ECC register
 */
static u32 nand_davinci_readecc(struct mtd_info *mtd)
{
	/* Read register ECC and clear it */
	return davinci_nand_readl(NANDF1ECC_OFFSET);
}

/*
 * Read DaVinci ECC registers and rework into MTD format
 */
static int nand_davinci_calculate_ecc(struct mtd_info *mtd,
				      const u_char *dat, u_char *ecc_code)
{
	unsigned int ecc_val = nand_davinci_readecc(mtd);
	/* squeeze 0 middle bits out so that it fits in 3 bytes */
	unsigned int tmp = (ecc_val&0x0fff)|((ecc_val&0x0fff0000)>>4);
	/* invert so that erased block ecc is correct */
	tmp = ~tmp;
	ecc_code[0] = (u_char)(tmp);
	ecc_code[1] = (u_char)(tmp >> 8);
	ecc_code[2] = (u_char)(tmp >> 16);

	return 0;
}

static int nand_davinci_correct_data(struct mtd_info *mtd, u_char *dat,
				     u_char *read_ecc, u_char *calc_ecc)
{
	struct nand_chip *chip = mtd->priv;
	u_int32_t eccNand = read_ecc[0] | (read_ecc[1] << 8) |
					  (read_ecc[2] << 16);
	u_int32_t eccCalc = calc_ecc[0] | (calc_ecc[1] << 8) |
					  (calc_ecc[2] << 16);
	u_int32_t diff = ecc_calc ^ ecc_nand;
	if (!diff)
		return 0;

	if ((((diff>>12)^diff) & 0xfff) == 0xfff) {
		/* Correctable error */
		if ((diff>>(12+3)) < chip->c.ecc.size) {
			dat[diff>>(12+3)] ^= (1 << ((diff>>12)&7));
			return 1;
		}
	} else if (!(diff & (diff-1))) {
		/* Single bit ECC error in the ECC itself,
		   nothing to fix */
		return 1;
	}
	/* Uncorrectable error */
	dev_warn(chip->parent, "uncorrectable ecc error, "
		"read=%x, calc=%x, diff=%x\n", ecc_nand, ecc_calc, diff);
	return -1;

}

#ifdef USE_DMA
static inline int calculate_ecc_dma(struct d_nand_chip *chip,
		u_char *ecc_code)
{
	int i = chip->c.ecc.steps;
	unsigned int *p = chip->bounce_ecc;
	while (i) {
		unsigned int ecc_val = *p++;
		/* squeeze 0 middle bits out so that it fits in 3 bytes */
		unsigned int tmp = (ecc_val&0x0fff)|((ecc_val&0x0fff0000)>>4);
		/* invert so that erased block ecc is correct */
		tmp = ~tmp;
		*ecc_code++ = (u_char)(tmp);
		*ecc_code++ = (u_char)(tmp >> 8);
		*ecc_code++ = (u_char)(tmp >> 16);
		i--;
	}
	return 0;
}
static inline int correct_data_dma(struct mtd_info *mtd,
		struct d_nand_chip *chip,
		u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	int i = chip->c.ecc.steps;
	int cnt = 0;
	int failed = 0;
	int err_mask = 0;
	int mask = 1;
	while (i) {
		int ret;
		ret = nand_davinci_correct_data(mtd, dat, read_ecc, calc_ecc);
		if (ret < 0) {
			err_mask |= mask;
			failed++;
		} else
			cnt += ret;
		mask <<= 1;
		dat += chip->c.ecc.size;
		read_ecc += 3;
		calc_ecc += 3;
		i--;
	}
	if (failed)
		dev_warn(chip->parent, "segment error mask=0x%x, "
				"corrected=%i\n", err_mask, cnt);
	mtd->ecc_stats.failed += failed;
	mtd->ecc_stats.corrected += cnt;
	return (failed)? -1 : cnt;
}

static inline void read_buf_dma(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct d_nand_chip *chip = mtd->priv;
	int edma = chip->edma;
	int bounce = 0;
	dma_addr_t phys;
	/* startup dma */
	INIT_COMPLETION(chip->dma_complete);
	phys = dma_map_single(chip->parent, buf, len, DMA_FROM_DEVICE);
	if (dma_mapping_error(chip->parent, phys)) {
		phys = chip->bounce_phys;
		bounce = 1;
		dev_warn(chip->parent, "bouncing\n");
	}
	chip->parm_edma.dst = phys;
	davinci_set_dma_params(edma, &chip->parm_edma);

	if (nand_is_ready(chip)) {
		chip->dma_active = 1;
		davinci_start_dma(edma);
	} else if (chip->irq < 0) {
		nand_wait_ready(mtd);
		chip->dma_active = 1;
		davinci_start_dma(edma);
	} else {
		chip->start_dma_needed = 1;
		if (!chip->irq_enabled) {
			enable_irq(chip->irq);
			chip->irq_enabled = 1;
		}
		if (nand_is_ready(chip)) {
			rmb();
			if (chip->start_dma_needed) {
				chip->start_dma_needed = 0;
				chip->dma_active = 1;
				davinci_start_dma(edma);
			}
		} else if (!(diff & (diff-1))) {
			/* Single bit ECC error in the ECC itself,
			   nothing to fix */
			return 1;
		} else {
			/* Uncorrectable error */
			return -1;
		}
	}
	wait_for_completion(&chip->dma_complete);
	if (bounce)
		memcpy(buf, chip->bounce_base, len);
	else
		dma_unmap_single(chip->parent, phys, len, DMA_FROM_DEVICE);
}
#if 0
void dump(uint8_t *buf, int len)
{
	int i = 0;
	unsigned int *p = (unsigned int *)buf;
	while (len >= 16) {
		printk(KERN_ERR "%08x %08x %08x %08x  %04x\n",
				p[3], p[2], p[1], p[0], i);
		len -= 16;
		i += 16;
		p += 4;
	}
}
#endif
static int read_page_dma(struct mtd_info *mtd, struct nand_chip *_chip,
	uint8_t *buf)
{
	edmacc_paramentry_regs parm;
	struct d_nand_chip *chip = (struct d_nand_chip *)_chip;
	int i;
	uint8_t *ecc_calc = chip->c.buffers->ecccalc;
	uint8_t *ecc_code = chip->c.buffers->ecccode;
	uint32_t *eccpos = chip->c.ecc.layout->eccpos;

	nand_davinci_enable_hwecc(mtd, NAND_ECC_READ);
	read_buf_dma(mtd, buf, mtd->writesize);
	calculate_ecc_dma(chip, ecc_calc);

	i = chip->c.ecc.total;
	while (i--)
		*ecc_code++ = chip->bounce_oob[*eccpos++];

	correct_data_dma(mtd, chip, buf, chip->c.buffers->ecccode, ecc_calc);
	davinci_get_dma_params(chip->edma_ecc, &parm);
#if 0
	printk(KERN_ERR "%x %x, %x %x, %x %x, %x %x\n",
			chip->parm_ecc.dst, parm.dst,
			chip->parm_ecc.src_dst_cidx, parm.src_dst_cidx,
			chip->parm_ecc.a_b_cnt, parm.a_b_cnt,
			chip->parm_ecc.ccnt, parm.ccnt);
	dump(buf, mtd->writesize);
	dump(chip->bounce_oob, mtd->oobsize);
#endif
	return 0;
}
#ifdef USE_DMA_FOR_WRITE
int get_oob_bytes_before_ecc(struct d_nand_chip *chip)
{
	uint32_t *eccpos = chip->c.ecc.layout->eccpos;
	int oob_before = 63;
	int i;
	for (i = 0; i < chip->c.ecc.total; i++)
		if (oob_before > eccpos[i])
			oob_before = eccpos[i];
	return oob_before & ~0xf;	/* keep multiple of 16 */
}

static inline void write_buf_dma(struct mtd_info *mtd, const uint8_t *buf,
		int len)
{
	struct d_nand_chip *chip = mtd->priv;
	int edma = chip->edma;
	int bounce = 0;
	dma_addr_t phys;
	/* startup dma */
	INIT_COMPLETION(chip->dma_complete);
	phys = dma_map_single(chip->parent, (uint8_t *)buf, len, DMA_TO_DEVICE);
	if (dma_mapping_error(chip->parent, phys)) {
		memcpy(chip->bounce_base, buf, len);
		phys = chip->bounce_phys;
		bounce = 1;
		dev_warn(chip->parent, "bouncing write\n");
	}
	chip->parm_edma_write.src = phys;
	davinci_set_dma_params(edma, &chip->parm_edma_write);

	chip->dma_active = 1;
	davinci_start_dma(edma);
	wait_for_completion(&chip->dma_complete);
	if (bounce == 0)
		dma_unmap_single(chip->parent, phys, len, DMA_TO_DEVICE);
}
/**
 * write_page_dma - hardware ecc based page write function
 */
static void write_page_dma(struct mtd_info *mtd, struct nand_chip *_chip,
	const uint8_t *buf)
{
	struct d_nand_chip *chip = (struct d_nand_chip *)_chip;
	int i;
	int before_ecc = chip->oob_bytes_before_ecc;
	uint8_t *ecc_calc = chip->c.buffers->ecccalc;
	uint32_t *eccpos = chip->c.ecc.layout->eccpos;

	nand_davinci_enable_hwecc(mtd, NAND_ECC_WRITE);
	write_buf_dma(mtd, buf, mtd->writesize);
	calculate_ecc_dma(chip, ecc_calc);

	for (i = 0; i < chip->c.ecc.total; i++)
		chip->c.oob_poi[eccpos[i]] = ecc_calc[i];

	memcpy(chip->bounce_oob, chip->c.oob_poi, before_ecc);
	nand_davinci_write_buf(mtd, chip->c.oob_poi + before_ecc,
			mtd->oobsize - before_ecc);
}
#endif

static void dma_cb(int lch, u16 ch_status, void *data)
{
	struct d_nand_chip *chip = (struct d_nand_chip *)data;
	if (DMA_COMPLETE != ch_status) {
		dev_err(chip->parent, "[DMA FAILED]\n");
	} else if (chip->dma_active) {
		chip->dma_active = 0;
		complete(&chip->dma_complete);
		return;
	} else {
		dev_err(chip->parent, "unexpected dma callback\n");
	}
	davinci_stop_dma(chip->edma);
	davinci_clean_channel(chip->edma);
	davinci_stop_dma(chip->edma_ecc);
	davinci_clean_channel(chip->edma_ecc);
	davinci_set_dma_params(chip->edma_ecc, &chip->parm_ecc);
	chip->dma_active = 0;
	complete(&chip->dma_complete);
}

static int get_dma_channels(struct mtd_info *mtd, struct d_nand_chip *chip)
{
	int edma;
	int tcc = 0;
	int edma_oob;
	int tcc_oob;
	int edma_ecc;
	int tcc_ecc = 0;
	int edma_fcr;
	int tcc_fcr = 0;
	int edma_ecc_parm;
	int tcc_ecc_parm = 0;
	int edma_fcr_parm;
	int tcc_fcr_parm = 0;
	int r;
	edmacc_paramentry_regs parm;
	enum dma_event_q queue_no = EVENTQ_1;
	int eccsize = chip->c.ecc.size;

#ifdef USE_DMA_FOR_WRITE
	int edma_oob_write;
	int tcc_oob_write;
	int before_ecc = get_oob_bytes_before_ecc(chip);
	chip->oob_bytes_before_ecc = before_ecc;
#endif

	}
	return 0;
}
#endif

/*
 * Read OOB data from flash.
 */
static int read_oob_and_check(struct mtd_info *mtd, loff_t offs, uint8_t *buf,
			      struct nand_bbt_descr *bd)
{
	int i, ret;
	int page;
	struct nand_chip *chip = mtd->priv;

	/* Calculate page address from offset */
	page = (int)(offs >> chip->page_shift);
	page &= chip->pagemask;

	/* Read OOB data from flash */
	ret = chip->ecc.read_oob(mtd, chip, page, 1);
	if (ret < 0)
		return ret;

	/* Copy read OOB data to the buffer*/
	memcpy(buf, chip->oob_poi, mtd->oobsize);

	/* Check pattern against BBM in OOB area */
	for (i = 0; i < bd->len; i++) {
		if (buf[bd->offs + i] != bd->pattern[i])
			return 1;
	}
	return 0;
}

/*
 * Fill in the memory based Bad Block Table (BBT).
 */
static int nand_davinci_memory_bbt(struct mtd_info *mtd,
				   struct nand_bbt_descr *bd)
{
	int i, numblocks;
	int startblock = 0;
	loff_t from = 0;
	struct nand_chip *chip = mtd->priv;
	int blocksize = 1 << chip->bbt_erase_shift;
	uint8_t *buf = chip->buffers->databuf;
	int len = bd->options & NAND_BBT_SCAN2NDPAGE ? 2 : 1;

	/* -numblocks- is 2 times the actual number of eraseblocks */
	numblocks = mtd->size >> (chip->bbt_erase_shift - 1);

	/* Now loop through all eraseblocks in the flash */
	for (i = startblock; i < numblocks; i += 2) {
		int j, ret;
		int offs = from;

		/* If NAND_BBT_SCAN2NDPAGE flag is set in bd->options,
		 * also each 2nd page of an eraseblock is checked
		 * for a Bad Block Marker. In that case, len equals 2.
		 */
		for (j = 0; j < len; j++) {
			/* Read OOB data and check pattern */
			ret = read_oob_and_check(mtd, from, buf, bd);
			if (ret < 0)
				return ret;

			/* Check pattern for bad block markers */
			if (ret) {
				/* Mark bad block by writing 0b11 in the
				   table */
				chip->bbt[i >> 3] |= 0x03 << (i & 0x6);

				printk(KERN_WARNING "Bad eraseblock %d at " \
						    "0x%08x\n", i >> 1,
						     (unsigned int)from);

				mtd->ecc_stats.badblocks++;
				break;
			}
			offs += mtd->writesize;
		}

		/* Make -from- point to next eraseblock */
		from += blocksize;
	}

	printk(KERN_NOTICE "Bad block scan: %d out of %d blocks are bad.\n",
			    mtd->ecc_stats.badblocks, numblocks>>1);

	return 0;
}

/*
 * This function creates a memory based bad block table (BBT).
 * It is largely based on the standard BBT function, but all
 * unnecessary junk is thrown out to speed up.
 */
static int nand_davinci_scan_bbt(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_bbt_descr *bd;
	int len, ret = 0;

	chip->bbt_td = NULL;
	chip->bbt_md = NULL;

	/* pagesize determines location of BBM */
	if (mtd->writesize > 512)
		bd = &davinci_memorybased_large;
	else
		bd = &davinci_memorybased_small;

	chip->badblock_pattern = bd;

	/* Use 2 bits per page meaning 4 page markers per byte */
	len = mtd->size >> (chip->bbt_erase_shift + 2);

	/* Allocate memory (2bit per block) and clear the memory bad block
	   table */
	chip->bbt = kzalloc(len, GFP_KERNEL);
	if (!chip->bbt) {
		printk(KERN_ERR "nand_davinci_scan_bbt: Out of memory\n");
		return -ENOMEM;
	}

	/* Now try to fill in the BBT */
	ret = nand_davinci_memory_bbt(mtd, bd);
	if (ret) {
		printk(KERN_ERR "nand_davinci_scan_bbt: "
		       "Can't scan flash and build the RAM-based BBT\n");

		kfree(chip->bbt);
		chip->bbt = NULL;
	}

	return ret;
}

/*
 * Read from memory register: we can read 4 bytes at a time.
 * The hardware takes care of actually reading the NAND flash.
 */
static void nand_davinci_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;
	int num_words = len >> 2;
	u32 *p = (u32 *)buf;
	struct nand_chip *chip = mtd->priv;

	for (i = 0; i < num_words; i++)
		p[i] = readl(chip->IO_ADDR_R);
}

static inline int nand_davinci_get_ready(struct mtd_info *mtd)
{
	return (davinci_nand_readl(NANDFSR_OFFSET) & NAND_BUSY_FLAG);
}
static irqreturn_t nand_isr(int irq, void *mtd_id)
{
	struct mtd_info *mtd = (struct mtd_info *)mtd_id;
	if (nand_davinci_get_ready(mtd)) {
//		printk(KERN_ERR "nand_isr ready\n");
		complete(&cmd_complete);
	} else {
//		printk(KERN_ERR "nand_isr not ready\n");
	}
	return IRQ_HANDLED;
}

/*
 * nand_davinci_waitfunc_with_irq
 * Wait for command done. (erase/program only)
 * allow 400ms for erase
 * allow  20ms for program
 */
static int nand_davinci_waitfunc_with_irq(
		struct mtd_info *mtd, struct nand_chip *chip)
{
	unsigned long timeout = (chip->state == FL_ERASING)?
			((HZ * 400) / 1000) : ((HZ * 20) / 1000);
	__raw_writeb(NAND_CMD_STATUS, (chip->IO_ADDR_R+MASK_CLE));
	cmd_complete.done = 0;
	if (!irq_enabled) {
		enable_irq(IRQ_EMIF_EMWAIT_RISE);
		irq_enabled = 1;
	}
	if (!nand_davinci_get_ready(mtd)) {
		if (wait_for_completion_timeout(&cmd_complete, timeout)==0) {
			printk(KERN_ERR "waitfunc timeout\n");
		}
	}
	return __raw_readb(chip->IO_ADDR_R);
}
static int nand_davinci_dev_ready_with_irq(struct mtd_info *mtd)
{
#if 1
	if (irq_enabled) {
		disable_irq(IRQ_EMIF_EMWAIT_RISE);
		irq_enabled = 0;
	}
	return nand_davinci_get_ready(mtd);
#else
	cmd_complete.done = 0;
	if (!nand_davinci_get_ready(mtd)) {
		if (wait_for_completion_timeout(&cmd_complete, 2)==0) {
			printk(KERN_ERR "dev_ready timeout\n");
		}
		return nand_davinci_get_ready(mtd);
	}
	return 1;
#endif
}
/*
 * Check hardware register for wait status. Returns 1 if device is ready,
 * 0 if it is still busy.
 */
static int nand_davinci_dev_ready(struct mtd_info *mtd)
{
	return (davinci_nand_readl(NANDFSR_OFFSET) & NAND_BUSY_FLAG);
}

static void nand_davinci_set_eccsize(struct nand_chip *chip)
{
	chip->ecc.size = 256;

#ifdef CONFIG_NAND_FLASH_HW_ECC
	switch (chip->ecc.mode) {
	case NAND_ECC_HW12_2048:
		chip->ecc.size = 2048;
		break;

	case NAND_ECC_HW3_512:
	case NAND_ECC_HW6_512:
	case NAND_ECC_HW8_512:
	chip->ecc.size = 512;
		break;

	case NAND_ECC_HW3_256:
	default:
		/* do nothing */
		break;
	}
#endif
}

static void nand_davinci_set_eccbytes(struct nand_chip *chip)
{
	chip->ecc.bytes = 3;

#ifdef CONFIG_NAND_FLASH_HW_ECC
	switch (chip->ecc.mode) {
	case NAND_ECC_HW12_2048:
		chip->ecc.bytes += 4;
	case NAND_ECC_HW8_512:
		chip->ecc.bytes += 2;
	case NAND_ECC_HW6_512:
		chip->ecc.bytes += 3;
	case NAND_ECC_HW3_512:
	case NAND_ECC_HW3_256:
	default:
		/* do nothing */
		break;
	}
#endif
}

static void __devinit nand_davinci_flash_init(void)
{
	u32 regval, tmp;

	/* Check for correct pin mux, reconfigure if necessary */
	tmp = davinci_readl(DAVINCI_SYSTEM_MODULE_BASE + PINMUX0);

	if ((tmp & 0x20020C1F) != 0x00000C1F) {
		/* Disable HPI and ATA mux */
		davinci_mux_peripheral(DAVINCI_MUX_HPIEN, 0);
		davinci_mux_peripheral(DAVINCI_MUX_ATAEN, 0);

		/* Enable VLYNQ and AEAW */
		davinci_mux_peripheral(DAVINCI_MUX_AEAW0, 1);
		davinci_mux_peripheral(DAVINCI_MUX_AEAW1, 1);
		davinci_mux_peripheral(DAVINCI_MUX_AEAW2, 1);
		davinci_mux_peripheral(DAVINCI_MUX_AEAW3, 1);
		davinci_mux_peripheral(DAVINCI_MUX_AEAW4, 1);
		davinci_mux_peripheral(DAVINCI_MUX_VLSCREN, 1);
		davinci_mux_peripheral(DAVINCI_MUX_VLYNQEN, 1);

		regval = davinci_readl(DAVINCI_SYSTEM_MODULE_BASE + PINMUX0);

		printk(KERN_WARNING "Warning: MUX config for NAND: Set " \
		       "PINMUX0 reg to 0x%08x, was 0x%08x, should be done " \
		       "by bootloader.\n", regval, tmp);
	}

	regval = davinci_nand_readl(AWCCR_OFFSET);
	regval |= 0x10000000;
	davinci_nand_writel(regval, AWCCR_OFFSET);

	/*------------------------------------------------------------------*
	 *  NAND FLASH CHIP TIMEOUT @ 459 MHz                               *
	 *                                                                  *
	 *  AEMIF.CLK freq   = PLL1/6 = 459/6 = 76.5 MHz                    *
	 *  AEMIF.CLK period = 1/76.5 MHz = 13.1 ns                         *
	 *                                                                  *
	 *------------------------------------------------------------------*/
	regval = 0
		| (0 << 31)           /* selectStrobe */
		| (0 << 30)           /* extWait */
		| (1 << 26)           /* writeSetup      10 ns */
		| (3 << 20)           /* writeStrobe     40 ns */
		| (1 << 17)           /* writeHold       10 ns */
		| (0 << 13)           /* readSetup       10 ns */
		| (3 << 7)            /* readStrobe      60 ns */
		| (0 << 4)            /* readHold        10 ns */
		| (3 << 2)            /* turnAround      ?? ns */
		| (0 << 0)            /* asyncSize       8-bit bus */
		;
	tmp = davinci_nand_readl(A1CR_OFFSET);
	if (tmp != regval) {
		printk(KERN_WARNING "Warning: NAND config: Set A1CR " \
		       "reg to 0x%08x, was 0x%08x, should be done by " \
		       "bootloader.\n", regval, tmp);
		davinci_nand_writel(regval, A1CR_OFFSET); /* 0x0434018C */
	}

	davinci_nand_writel(0x00000101, NANDFCR_OFFSET);
}

/*
 * Main initialization routine
 */
int __devinit nand_davinci_probe(struct platform_device *pdev)
{
	struct flash_platform_data *pdata = pdev->dev.platform_data;
	struct resource		  *res = pdev->resource;
	struct nand_chip     	  *chip;
	struct device        	  *dev = NULL;
	u32                  	  nand_rev_code;
#ifdef CONFIG_MTD_CMDLINE_PARTS
	const char             	  *master_name;
	int 		     	  mtd_parts_nb = 0;
	struct mtd_partition 	  *mtd_parts = 0;
#endif

	nand_clock = clk_get(dev, "AEMIFCLK");
	if (IS_ERR(nand_clock)) {
		printk(KERN_ERR "Error %ld getting AEMIFCLK clock?\n",
		       PTR_ERR(nand_clock));
		return -1;
	}

	clk_enable(nand_clock);

	/* Allocate memory for MTD device structure and private data */
	nand_davinci_mtd = kmalloc(sizeof(struct mtd_info) +
				   sizeof(struct nand_chip), GFP_KERNEL);

	if (!nand_davinci_mtd) {
		printk(KERN_ERR "Unable to allocate davinci NAND MTD device " \
		       "structure.\n");
		clk_disable(nand_clock);
		return -ENOMEM;
	}

	/* Get pointer to private data */
	chip = (struct nand_chip *) (&nand_davinci_mtd[1]);

	/* Initialize structures */
	memset((char *)nand_davinci_mtd, 0, sizeof(struct mtd_info));
	memset((char *)chip, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	nand_davinci_mtd->priv = chip;

	nand_rev_code = davinci_nand_readl(NRCSR_OFFSET);

	printk("DaVinci NAND Controller rev. %d.%d\n",
	       (nand_rev_code >> 8) & 0xff, nand_rev_code & 0xff);

	nand_vaddr = ioremap(res->start, res->end - res->start);
	if (nand_vaddr == NULL) {
		printk(KERN_ERR "DaVinci NAND: ioremap failed.\n");
		clk_disable(nand_clock);
		kfree(nand_davinci_mtd);
		return -ENOMEM;
	}

	chip->IO_ADDR_R   = (void __iomem *)nand_vaddr;
	chip->IO_ADDR_W   = (void __iomem *)nand_vaddr;
	chip->chip_delay  = 0;
	chip->select_chip = nand_davinci_select_chip;
	chip->options     = 0;
	chip->ecc.mode	  = DAVINCI_NAND_ECC_MODE;

	/* Set ECC size and bytes */
	nand_davinci_set_eccsize(chip);
	nand_davinci_set_eccbytes(chip);

	/* Set address of hardware control function */
	chip->cmd_ctrl  = nand_davinci_hwcontrol;
	chip->dev_ready = nand_davinci_dev_ready;

#ifdef CONFIG_NAND_FLASH_HW_ECC
	chip->ecc.calculate = nand_davinci_calculate_ecc;
	chip->ecc.correct   = nand_davinci_correct_data;
	chip->ecc.hwctl     = nand_davinci_enable_hwecc;
#endif

	/* Speed up the read buffer */
	chip->read_buf      = nand_davinci_read_buf;

	/* Speed up the creation of the bad block table */
	chip->scan_bbt      = nand_davinci_scan_bbt;

	init_completion(&cmd_complete);
	if (request_irq(IRQ_EMIF_EMWAIT_RISE, &nand_isr, 0,
			"davinci_nand", nand_davinci_mtd)) {
		printk(KERN_ERR "nand: request_irq failed, irq:%i\n",
				IRQ_EMIF_EMWAIT_RISE);
	} else {
		chip->dev_ready = nand_davinci_dev_ready_with_irq;
		chip->waitfunc = nand_davinci_waitfunc_with_irq;
		irq_enabled = 1;
	}
	nand_davinci_flash_init();

	nand_davinci_mtd->owner = THIS_MODULE;

	/* Scan to find existence of the device */
	if (nand_scan(nand_davinci_mtd, 1)) {
		printk(KERN_ERR "Chip Select is not set for NAND\n");
		clk_disable(nand_clock);
		kfree(nand_davinci_mtd);
		return -ENXIO;
	}

	/* Register the partitions */
	add_mtd_partitions(nand_davinci_mtd, pdata->parts, pdata->nr_parts);

#ifdef CONFIG_MTD_CMDLINE_PARTS
	/* Set nand_davinci_mtd->name = 0 temporarily */
	master_name = nand_davinci_mtd->name;
	nand_davinci_mtd->name = (char *)0;

	/* nand_davinci_mtd->name == 0, means: don't bother checking
	   <mtd-id> */
	mtd_parts_nb = parse_mtd_partitions(nand_davinci_mtd, part_probes,
					    &mtd_parts, 0);

	/* Restore nand_davinci_mtd->name */
	nand_davinci_mtd->name = master_name;

	add_mtd_partitions(nand_davinci_mtd, mtd_parts, mtd_parts_nb);
#endif

	return 0;
}

/*
 * Clean up routine
 */
static int nand_davinci_remove(struct platform_device *pdev)
{
	clk_disable(nand_clock);

	if (nand_vaddr)
		iounmap(nand_vaddr);

	/* Release resources, unregister device */
	nand_release(nand_davinci_mtd);

	/* Free the MTD device structure */
	kfree(nand_davinci_mtd);

	return 0;
}


static struct platform_driver nand_davinci_driver = {
	.probe		= nand_davinci_probe,
	.remove		= nand_davinci_remove,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static int __init nand_davinci_init(void)
{
	return platform_driver_register(&nand_davinci_driver);
}
module_init(nand_davinci_init);

#ifdef MODULE
static void __exit nand_davinci_exit(void)
{
	platform_driver_unregister(&nand_davinci_driver);
}
module_exit(nand_davinci_exit);
#endif

MODULE_ALIAS(DRIVER_NAME);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Board-specific glue layer for NAND flash on davinci" \
		   "board");
