/*
 * ALSA PCM interface for the TI DAVINCI processor
 *
 * Author:      Vladimir Barinov, <vbarinov@ru.mvista.com>
 * Copyright:   (C) 2007 MontaVista Software, Inc., <source@mvista.com>
 * added IRAM ping/pong (C) 2008 Troy Kisky <troy.kisky@boundarydevices.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <mach/edma.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/dma.h>

#include "davinci-pcm.h"

#define DAVINCI_PCM_DEBUG 0
#if DAVINCI_PCM_DEBUG
#define DPRINTK(format, arg...) printk(KERN_DEBUG format, ## arg)
#else
#define DPRINTK(format, arg...) do {} while (0)
#endif


static struct snd_pcm_hardware davinci_pcm_hardware = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_PAUSE),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
		  SNDRV_PCM_RATE_KNOT),
	.rate_min = 8000,
	.rate_max = 96000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = 7 * 512,	/* This is size of ping + pong buffer*/
	.periods_min = 16,
	.periods_max = 255,
	.fifo_size = 0,
};

struct davinci_runtime_data {
	spinlock_t lock;
	int asp_master_lch;		/* Master DMA channel */
	int asp_link_lch[2];	/* asp parameter link channel */
	int ram_master_lch;
	int ram_link_lch;
	/* Used if playback
	 * After initial copy of 1st iram buffer
	 * this is the link to finish the main buffer
	 */
	int ram_link_lch2;
	edmacc_paramentry_regs asp_params;
	edmacc_paramentry_regs ram_params;
};

#if DAVINCI_PCM_DEBUG
void print_buf_info(int lch, char* name)
{
	edmacc_paramentry_regs p;
	if (lch<0) return;
	davinci_get_dma_params(lch, &p);
	printk(KERN_DEBUG "%s: 0x%x, opt=%x, src=%x, a_b_cnt=%x dst=%x\n",
			name, lch, p.opt, p.src, p.a_b_cnt, p.dst);
	printk(KERN_DEBUG "    src_dst_bidx=%x link_bcntrld=%x src_dst_cidx=%x ccnt=%x\n",
			p.src_dst_bidx, p.link_bcntrld, p.src_dst_cidx, p.ccnt);
}
#endif

static void davinci_pcm_dma_irq(int lch, u16 ch_status, void *data)
{
	struct snd_pcm_substream *substream = data;
#if DAVINCI_PCM_DEBUG
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct davinci_runtime_data *prtd = runtime->private_data;

	print_buf_info(prtd->ram_master_lch, "i ram_master_lch");
#endif
	DPRINTK("lch=%d, status=0x%x\n", lch, ch_status);
	if (unlikely(ch_status != DMA_COMPLETE))
		return;

	if (snd_pcm_running(substream)) {
		snd_pcm_period_elapsed(substream);
	}
}


/*
 * This is called after runtime->dma_addr, period_bytes and data_type are valid
 */
static int davinci_pcm_dma_setup(struct snd_pcm_substream *substream)
{
	unsigned short ram_src_cidx, ram_dst_cidx;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct davinci_runtime_data *prtd = runtime->private_data;
	struct snd_dma_buffer *iram_dma = (struct snd_dma_buffer *)substream->dma_buffer.private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct davinci_pcm_dma_params *dma_data = rtd->dai->cpu_dai->dma_data;
	unsigned int data_type = dma_data->data_type;
	unsigned int ping_size = snd_pcm_lib_period_bytes(substream) >> 1;	/* divide by 2 for ping/pong */
	int lch = prtd->asp_link_lch[1];
	if ((data_type == 0) || (data_type > 4)) {
		printk(KERN_ERR "%s: data_type=%i\n", __func__, data_type);
		return -EINVAL;
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dma_addr_t asp_src_pong = iram_dma->addr + ping_size;
		ram_src_cidx = ping_size;
		ram_dst_cidx = -ping_size;
		davinci_set_dma_src_params(lch, asp_src_pong, INCR, W16BIT);

		lch = prtd->asp_link_lch[0];
		davinci_set_dma_src_index(lch, data_type, 0);
		lch = prtd->asp_link_lch[1];
		davinci_set_dma_src_index(lch, data_type, 0);

		lch = prtd->ram_link_lch;
		davinci_set_dma_src_params(lch, runtime->dma_addr, INCR, W32BIT);
	} else {
		dma_addr_t asp_dst_pong = iram_dma->addr + ping_size;
		ram_src_cidx = -ping_size;
		ram_dst_cidx = ping_size;
		davinci_set_dma_dest_params(lch, asp_dst_pong, INCR, W16BIT);

		lch = prtd->asp_link_lch[0];
		davinci_set_dma_dest_index(lch, data_type, 0);
		lch = prtd->asp_link_lch[1];
		davinci_set_dma_dest_index(lch, data_type, 0);

		lch = prtd->ram_link_lch;
		davinci_set_dma_dest_params(lch, runtime->dma_addr, INCR, W32BIT);
	}

	lch = prtd->asp_link_lch[0];
	davinci_set_dma_transfer_params(lch, data_type, ping_size/data_type, 1, 0, ASYNC);

	lch = prtd->asp_link_lch[1];
	davinci_set_dma_transfer_params(lch, data_type, ping_size/data_type, 1, 0, ASYNC);

	lch = prtd->ram_link_lch;
	davinci_set_dma_src_index(lch, ping_size, ram_src_cidx);
	davinci_set_dma_dest_index(lch, ping_size, ram_dst_cidx);
	davinci_set_dma_transfer_params(lch, ping_size, 2, runtime->periods, 2, ASYNC);

	/* init master params */
	davinci_get_dma_params(prtd->asp_link_lch[0], &prtd->asp_params);
	davinci_get_dma_params(prtd->ram_link_lch, &prtd->ram_params);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		edmacc_paramentry_regs parm;
		/* Copy entire iram buffer before playback started */
		prtd->ram_params.a_b_cnt = (1 << 16) | (ping_size << 1);
		prtd->ram_params.src_dst_bidx = (ping_size << 1);	/* 0 dst_bidx */
		prtd->ram_params.src_dst_cidx = (ping_size << 1);	/* 0 dst_cidx */
		prtd->ram_params.ccnt = 1;

		/* Skip 1st period */
		davinci_get_dma_params(prtd->ram_link_lch, &parm);
		parm.src += (ping_size << 1);
		parm.ccnt -= 1;
		davinci_set_dma_params(prtd->ram_link_lch2, &parm);
	}
	return 0;
}

/* 1 asp tx or rx channel using 2 parameter channels
 * 1 ram to/from iram channel using 1 parameter channel
 * 
 * Playback
 * ram copy channel kicks off first,
 * 1st ram copy of entire iram buffer completion kicks off asp channel
 * asp tcc always kicks off ram copy of 1/2 iram buffer
 * 
 * Record
 * asp channel starts, tcc kicks off ram copy
 */
static int davinci_pcm_dma_request(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dma_buffer *iram_dma = (struct snd_dma_buffer *)substream->dma_buffer.private_data;
	struct davinci_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct davinci_pcm_dma_params *dma_data = rtd->dai->cpu_dai->dma_data;

	dma_addr_t asp_src_ping;
	dma_addr_t asp_dst_ping;

	int tcc = TCC_ANY;
	int ret;
	int lch;
	edmacc_paramentry_regs parm;

	if ((!dma_data)||(!iram_dma))
		return -ENODEV;

	/* Request ram master channel */
	ret = davinci_request_dma(DAVINCI_DMA_CHANNEL_ANY, "audio iram",
				  davinci_pcm_dma_irq, substream,
				  &prtd->ram_master_lch, &tcc, EVENTQ_1);
	if (ret)
		goto exit1;

	/* Request ram link channel */
	tcc = TCC_ANY;
	ret = davinci_request_dma(DAVINCI_EDMA_PARAM_ANY, "audio iram link",
				  NULL, NULL, &prtd->ram_link_lch, &tcc, EVENTQ_1);
	if (ret)
		goto exit2;

	/* Request asp master DMA channel */
	tcc = prtd->ram_master_lch;
	ret = davinci_request_dma(dma_data->channel, dma_data->name,
				  davinci_pcm_dma_irq, substream,
				  &prtd->asp_master_lch, &tcc, EVENTQ_0);
	if (ret)
		goto exit3;
	/* Request asp link channels */
	tcc = prtd->ram_master_lch;
	ret = davinci_request_dma(DAVINCI_EDMA_PARAM_ANY, "audio asp ping",
				  NULL, NULL, &prtd->asp_link_lch[0], &tcc, EVENTQ_0);
	if (ret)
		goto exit4;
	tcc = prtd->ram_master_lch;
	ret = davinci_request_dma(DAVINCI_EDMA_PARAM_ANY, "audio asp pong",
				  NULL, NULL, &prtd->asp_link_lch[1], &tcc, EVENTQ_0);
	if (ret)
		goto exit5;

	prtd->ram_link_lch2 = -1;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		tcc = TCC_ANY;
		ret = davinci_request_dma(DAVINCI_EDMA_PARAM_ANY,
				"audio iram playback link", NULL, NULL,
				&prtd->ram_link_lch2, &tcc, EVENTQ_1);
		if (ret)
			goto exit6;
	}
	
	/* circle ping-pong buffers */
	davinci_dma_link_lch(prtd->asp_link_lch[0], prtd->asp_link_lch[1]);
	davinci_dma_link_lch(prtd->asp_link_lch[1], prtd->asp_link_lch[0]);
	/* circle ram buffers */
	davinci_dma_link_lch(prtd->ram_link_lch, prtd->ram_link_lch);

	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		asp_src_ping = iram_dma->addr;
		asp_dst_ping = dma_data->dma_addr;	/* fifo */
	} else {
		asp_src_ping = dma_data->dma_addr;	/* fifo */
		asp_dst_ping = iram_dma->addr;
	}
	/* ping */
	lch = prtd->asp_link_lch[0];
	davinci_set_dma_src_params(lch, asp_src_ping, INCR, W16BIT);
	davinci_set_dma_dest_params(lch, asp_dst_ping, INCR, W16BIT);
	davinci_set_dma_src_index(lch, 0, 0);
	davinci_set_dma_dest_index(lch, 0, 0);

	davinci_get_dma_params(lch, &parm);
	parm.opt &= ~(TCCMODE | TCC | TCINTEN);
	parm.opt |= TCCHEN | ((prtd->ram_master_lch & 0x3f)<<12);
	davinci_set_dma_params(lch, &parm);

	/* pong */
	lch = prtd->asp_link_lch[1];
	davinci_set_dma_src_params(lch, asp_src_ping, INCR, W16BIT);
	davinci_set_dma_dest_params(lch, asp_dst_ping, INCR, W16BIT);
	davinci_set_dma_src_index(lch, 0, 0);
	davinci_set_dma_dest_index(lch, 0, 0);

	davinci_get_dma_params(lch, &parm);
	parm.opt &= ~(TCCMODE | TCC);
	/* interrupt after every pong completion */
	parm.opt |= TCINTEN | TCCHEN | ((prtd->ram_master_lch & 0x3f)<<12);
	davinci_set_dma_params(lch, &parm);

	/* ram */
	lch = prtd->ram_link_lch;
	davinci_set_dma_src_params(lch, iram_dma->addr, INCR, W32BIT);
	davinci_set_dma_dest_params(lch, iram_dma->addr, INCR, W32BIT);
	return 0;
exit6:
	davinci_free_dma(prtd->ram_link_lch2);
exit5:
	davinci_free_dma(prtd->asp_link_lch[0]);
exit4:
	davinci_free_dma(prtd->asp_master_lch);
exit3:
	davinci_free_dma(prtd->ram_link_lch);
exit2:
	davinci_free_dma(prtd->ram_master_lch);
exit1:
	return ret;
}

static int davinci_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct davinci_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;

	spin_lock(&prtd->lock);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			/* copy 1st iram buffer */
			davinci_start_dma(prtd->ram_master_lch);
		}
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		davinci_start_dma(prtd->asp_master_lch);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		davinci_stop_dma(prtd->ram_master_lch);
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		davinci_stop_dma(prtd->asp_master_lch);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	spin_unlock(&prtd->lock);

	return ret;
}

static int davinci_pcm_prepare(struct snd_pcm_substream *substream)
{
	int ret;
	struct davinci_runtime_data *prtd = substream->runtime->private_data;

	ret = davinci_pcm_dma_setup(substream);
	if (ret<0)
		return ret;
	davinci_set_dma_params(prtd->ram_master_lch, &prtd->ram_params);
	davinci_dma_link_lch(prtd->ram_master_lch, prtd->ram_link_lch2);
	davinci_set_dma_params(prtd->asp_master_lch, &prtd->asp_params);
#if DAVINCI_PCM_DEBUG
	if (1) {
		print_buf_info(prtd->ram_master_lch, "ram_master_lch");
		print_buf_info(prtd->ram_link_lch, "ram_link_lch");
		print_buf_info(prtd->ram_link_lch2, "ram_link_lch2");
		print_buf_info(prtd->asp_master_lch, "asp_master_lch");
		print_buf_info(prtd->asp_link_lch[0], "asp_link_lch[0]");
		print_buf_info(prtd->asp_link_lch[1], "asp_link_lch[1]");
	}
#endif
	return 0;
}

static snd_pcm_uframes_t
davinci_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct davinci_runtime_data *prtd = runtime->private_data;
	unsigned int offset;
	int count_asp, count_ram;
	int mod_ram;
	dma_addr_t ram_src, ram_dst;
	dma_addr_t asp_src, asp_dst;
	unsigned int period_size = snd_pcm_lib_period_bytes(substream);
	
	spin_lock(&prtd->lock);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* reading ram before asp should be safe
		 * as long as the asp transfers less than a ping size
		 * of bytes between the 2 reads
		 */
		davinci_dma_getposition(prtd->ram_master_lch, &ram_src, &ram_dst);
		davinci_dma_getposition(prtd->asp_master_lch, &asp_src, &asp_dst);
		count_asp = asp_src - prtd->asp_params.src;
		count_ram = ram_src - prtd->ram_params.src;
		mod_ram = count_ram % period_size; 
		mod_ram -= count_asp;
		if (mod_ram < 0)
			mod_ram += period_size;
		else if (mod_ram==0) {
			if (snd_pcm_running(substream))
				mod_ram += period_size;
		}
		count_ram -= mod_ram;
		if (count_ram < 0)
			count_ram += period_size * runtime->periods;
	} else {
		davinci_dma_getposition(prtd->ram_master_lch, &ram_src, &ram_dst);
		count_ram = ram_dst - prtd->ram_params.dst;
	}
	spin_unlock(&prtd->lock);

	offset = bytes_to_frames(runtime, count_ram);
	DPRINTK("offset=0x%x\n", offset);
	if (offset >= runtime->buffer_size)
		offset = 0;

	return offset;
}

static int davinci_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct davinci_runtime_data *prtd;
	int ret = 0;

	snd_soc_set_runtime_hwparams(substream, &davinci_pcm_hardware);

	prtd = kzalloc(sizeof(struct davinci_runtime_data), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	spin_lock_init(&prtd->lock);

	runtime->private_data = prtd;

	ret = davinci_pcm_dma_request(substream);
	if (ret) {
		printk(KERN_ERR "davinci_pcm: Failed to get dma channels\n");
		kfree(prtd);
	}

	return ret;
}

static int davinci_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct davinci_runtime_data *prtd = runtime->private_data;

	davinci_dma_unlink_lch(prtd->asp_link_lch[0], prtd->asp_link_lch[0]);
	davinci_dma_unlink_lch(prtd->asp_link_lch[1], prtd->asp_link_lch[1]);
	davinci_dma_unlink_lch(prtd->ram_link_lch, prtd->ram_link_lch);

	davinci_free_dma(prtd->asp_link_lch[0]);
	davinci_free_dma(prtd->asp_link_lch[1]);
	davinci_free_dma(prtd->asp_master_lch);
	davinci_free_dma(prtd->ram_link_lch);
	if (prtd->ram_link_lch2 != -1) {
		prtd->ram_link_lch2 = -1;
		davinci_free_dma(prtd->ram_link_lch2);
	}
	davinci_free_dma(prtd->ram_master_lch);
	kfree(prtd);
	return 0;
}

static int davinci_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int davinci_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int davinci_pcm_mmap(struct snd_pcm_substream *substream,
			    struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
}

struct snd_pcm_ops davinci_pcm_ops = {
	.open = 	davinci_pcm_open,
	.close = 	davinci_pcm_close,
	.ioctl = 	snd_pcm_lib_ioctl,
	.hw_params = 	davinci_pcm_hw_params,
	.hw_free = 	davinci_pcm_hw_free,
	.prepare = 	davinci_pcm_prepare,
	.trigger = 	davinci_pcm_trigger,
	.pointer = 	davinci_pcm_pointer,
	.mmap = 	davinci_pcm_mmap,
};
int davinci_alloc_iram(unsigned size);
void davinci_free_iram(unsigned addr, unsigned size);

static int davinci_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = davinci_pcm_hardware.buffer_bytes_max;
	struct snd_dma_buffer *iram_dma = NULL;
	unsigned iram_phys = 0;
	unsigned int iram_size = davinci_pcm_hardware.period_bytes_max;
	
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->bytes = size;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);

	DPRINTK("preallocate_dma_buffer: area=%p, addr=%p, size=%d\n",
		(void *) buf->area, (void *) buf->addr, size);


	if (!buf->area)
		goto exit1;
	iram_phys = davinci_alloc_iram(iram_size);
	if (((int)iram_phys) <= 0)
		goto exit2;
	iram_dma = kzalloc(sizeof(*iram_dma), GFP_KERNEL);
	if (!iram_dma)
		goto exit3;
	iram_dma->area = (char *)ioremap(iram_phys, iram_size);
	iram_dma->addr = iram_phys;
	memset(iram_dma->area, 0, iram_size);
	buf->private_data = iram_dma ;
	return 0;
	kfree(iram_dma);
exit3:
	davinci_free_iram(iram_phys, iram_size);
exit2:
	dma_free_writecombine(pcm->card->dev, size, buf->area, buf->addr);
	buf->area = NULL;
exit1:
	return -ENOMEM;
}

static void davinci_pcm_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		unsigned int iram_size = davinci_pcm_hardware.period_bytes_max;
		struct snd_dma_buffer *iram_dma;
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
		iram_dma = (struct snd_dma_buffer *)buf->private_data;
		if (iram_dma) {
			davinci_free_iram(iram_dma->addr, iram_size);
			kfree(iram_dma);
		}
	}
}

static u64 davinci_pcm_dmamask = 0xffffffff;

static int davinci_pcm_new(struct snd_card *card,
			   struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	int ret;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &davinci_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (dai->playback.channels_min) {
		ret = davinci_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			return ret;
	}

	if (dai->capture.channels_min) {
		ret = davinci_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			return ret;
	}
	return 0;
}

struct snd_soc_platform davinci_soc_platform = {
	.name = 	"davinci-audio",
	.pcm_ops = 	&davinci_pcm_ops,
	.pcm_new = 	davinci_pcm_new,
	.pcm_free = 	davinci_pcm_free,
};
EXPORT_SYMBOL_GPL(davinci_soc_platform);

MODULE_AUTHOR("Vladimir Barinov");
MODULE_DESCRIPTION("TI DAVINCI PCM DMA module");
MODULE_LICENSE("GPL");
