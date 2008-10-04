/*
 * ALSA SoC I2S (McBSP) Audio Layer for TI DAVINCI processor
 *
 * Author:      Vladimir Barinov, <vbarinov@ru.mvista.com>
 * Copyright:   (C) 2007 MontaVista Software, Inc., <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include "davinci-pcm.h"

#define DAVINCI_MCBSP_DRR_REG	0x00
#define DAVINCI_MCBSP_DXR_REG	0x04
#define DAVINCI_MCBSP_SPCR_REG	0x08
#define DAVINCI_MCBSP_RCR_REG	0x0c
#define DAVINCI_MCBSP_XCR_REG	0x10
#define DAVINCI_MCBSP_SRGR_REG	0x14
#define DAVINCI_MCBSP_PCR_REG	0x24

#define DAVINCI_MCBSP_SPCR_RRST		(1 << 0)
#define DAVINCI_MCBSP_SPCR_RINTM(v)	((v) << 4)
#define DAVINCI_MCBSP_SPCR_XRST		(1 << 16)
#define DAVINCI_MCBSP_SPCR_XINTM(v)	((v) << 20)
#define DAVINCI_MCBSP_SPCR_GRST		(1 << 22)
#define DAVINCI_MCBSP_SPCR_FRST		(1 << 23)
#define DAVINCI_MCBSP_SPCR_FREE		(1 << 25)

#define DAVINCI_MCBSP_RCR_RWDLEN1(v)	((v) << 5)
#define DAVINCI_MCBSP_RCR_RFRLEN1(v)	((v) << 8)
#define DAVINCI_MCBSP_RCR_RDATDLY(v)	((v) << 16)
#define DAVINCI_MCBSP_RCR_RFIG		(1 << 18)
#define DAVINCI_MCBSP_RCR_RWDLEN2(v)	((v) << 21)
#define DAVINCI_MCBSP_RCR_RFRLEN2(v)	((v) << 24)
#define DAVINCI_MCBSP_RCR_RPHASE	(1 << 31)

#define DAVINCI_MCBSP_XCR_XWDLEN1(v)	((v) << 5)
#define DAVINCI_MCBSP_XCR_XFRLEN1(v)	((v) << 8)
#define DAVINCI_MCBSP_XCR_XDATDLY(v)	((v) << 16)
#define DAVINCI_MCBSP_XCR_XFIG		(1 << 18)
#define DAVINCI_MCBSP_XCR_XWDLEN2(v)	((v) << 21)
#define DAVINCI_MCBSP_XCR_XFRLEN2(v)	((v) << 24)
#define DAVINCI_MCBSP_XCR_XPHASE	(1 << 31)

#define DAVINCI_MCBSP_SRGR_FWID(v)	((v) << 8)
#define DAVINCI_MCBSP_SRGR_FPER(v)	((v) << 16)
#define DAVINCI_MCBSP_SRGR_FSGM		(1 << 28)

#define DAVINCI_MCBSP_PCR_CLKRP		(1 << 0)
#define DAVINCI_MCBSP_PCR_CLKXP		(1 << 1)
#define DAVINCI_MCBSP_PCR_FSRP		(1 << 2)
#define DAVINCI_MCBSP_PCR_FSXP		(1 << 3)
#define DAVINCI_MCBSP_PCR_CLKRM		(1 << 8)
#define DAVINCI_MCBSP_PCR_CLKXM		(1 << 9)
#define DAVINCI_MCBSP_PCR_FSRM		(1 << 10)
#define DAVINCI_MCBSP_PCR_FSXM		(1 << 11)


enum {
	DAVINCI_MCBSP_WORD_8 = 0,
	DAVINCI_MCBSP_WORD_12,
	DAVINCI_MCBSP_WORD_16,
	DAVINCI_MCBSP_WORD_20,
	DAVINCI_MCBSP_WORD_24,
	DAVINCI_MCBSP_WORD_32,
};

static struct davinci_pcm_dma_params davinci_i2s_pcm_out = {
	.name = "I2S PCM Stereo out",
};

static struct davinci_pcm_dma_params davinci_i2s_pcm_in = {
	.name = "I2S PCM Stereo in",
};

struct davinci_mcbsp_dev {
	void __iomem			*base;
#define MOD_DSP_A	0
#define MOD_DSP_B	1
	int				mode;
	struct clk			*clk;
	struct davinci_pcm_dma_params	*dma_params[2];
	struct snd_soc_dai *codec_dai;
	int dac_active;
	struct work_struct deferred_mute_work;
};

static inline void davinci_mcbsp_write_reg(struct davinci_mcbsp_dev *dev,
					   int reg, u32 val)
{
	__raw_writel(val, dev->base + reg);
}

static inline u32 davinci_mcbsp_read_reg(struct davinci_mcbsp_dev *dev, int reg)
{
	return __raw_readl(dev->base + reg);
}

static void davinci_mcbsp_start(struct davinci_mcbsp_dev *dev, int playback)
{
	u32 w, pcr;

	pcr = davinci_mcbsp_read_reg(dev, DAVINCI_MCBSP_PCR_REG);
	w = davinci_mcbsp_read_reg(dev, DAVINCI_MCBSP_SPCR_REG);
	if (pcr & (DAVINCI_MCBSP_PCR_FSXM | DAVINCI_MCBSP_PCR_FSRM |
			DAVINCI_MCBSP_PCR_CLKXM | DAVINCI_MCBSP_PCR_CLKRM)) {
		/* Start the sample generator */
		w |= DAVINCI_MCBSP_SPCR_GRST;
		davinci_mcbsp_write_reg(dev, DAVINCI_MCBSP_SPCR_REG, w);
	}
	/* Enable transmitter or receiver */
	w |= (playback) ? DAVINCI_MCBSP_SPCR_XRST : DAVINCI_MCBSP_SPCR_RRST;

	if (pcr & (DAVINCI_MCBSP_PCR_FSXM | DAVINCI_MCBSP_PCR_FSRM)) {
		/* Start frame sync */
		w |= DAVINCI_MCBSP_SPCR_FRST;
	}
	davinci_mcbsp_write_reg(dev, DAVINCI_MCBSP_SPCR_REG, w);
}

static void davinci_mcbsp_stop(struct davinci_mcbsp_dev *dev, int playback)
{
	u32 w;

	/* Reset transmitter/receiver and sample rate/frame sync generators */
	w = davinci_mcbsp_read_reg(dev, DAVINCI_MCBSP_SPCR_REG);
	w &= ~(DAVINCI_MCBSP_SPCR_GRST | DAVINCI_MCBSP_SPCR_FRST);
	w &= (playback) ? ~DAVINCI_MCBSP_SPCR_XRST : ~DAVINCI_MCBSP_SPCR_RRST;
	davinci_mcbsp_write_reg(dev, DAVINCI_MCBSP_SPCR_REG, w);
}


#define DEFAULT_BITPERSAMPLE	16

static int davinci_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,
				   unsigned int fmt)
{
	struct davinci_mcbsp_dev *dev = cpu_dai->private_data;
	unsigned int pcr;
	unsigned int srgr;
	srgr = DAVINCI_MCBSP_SRGR_FSGM |
		DAVINCI_MCBSP_SRGR_FPER(DEFAULT_BITPERSAMPLE * 2 - 1) |
		DAVINCI_MCBSP_SRGR_FWID(DEFAULT_BITPERSAMPLE - 1);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		/* codec is master */
		pcr = 0;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		/* cpu is master */
		pcr = DAVINCI_MCBSP_PCR_FSXM |
			DAVINCI_MCBSP_PCR_FSRM |
			DAVINCI_MCBSP_PCR_CLKXM |
			DAVINCI_MCBSP_PCR_CLKRM;
		break;
	default:
		printk(KERN_ERR "%s:bad master\n", __func__);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_NF:
		/* CLKRP Receive clock polarity,
		 *	1 - sampled on rising edge of CLKR
		 *	valid on rising edge
		 * CLKXP Transmit clock polarity,
		 *	1 - clocked on falling edge of CLKX
		 *	valid on rising edge
		 * FSRP  Receive frame sync pol, 0 - active high
		 * FSXP  Transmit frame sync pol, 0 - active high
		 */
		pcr |= (DAVINCI_MCBSP_PCR_CLKXP | DAVINCI_MCBSP_PCR_CLKRP);
		break;
	case SND_SOC_DAIFMT_NB_IF:
		/* CLKRP Receive clock polarity,
		 *	0 - sampled on falling edge of CLKR
		 *	valid on falling edge
		 * CLKXP Transmit clock polarity,
		 *	0 - clocked on rising edge of CLKX
		 *	valid on falling edge
		 * FSRP  Receive frame sync pol, 1 - active low
		 * FSXP  Transmit frame sync pol, 1 - active low
		 */
		pcr |= (DAVINCI_MCBSP_PCR_FSXP | DAVINCI_MCBSP_PCR_FSRP);
		break;
	case SND_SOC_DAIFMT_IB_IF:
		/* CLKRP Receive clock polarity,
		 *	1 - sampled on rising edge of CLKR
		 *	valid on rising edge
		 * CLKXP Transmit clock polarity,
		 *	1 - clocked on falling edge of CLKX
		 *	valid on rising edge
		 * FSRP  Receive frame sync pol, 1 - active low
		 * FSXP  Transmit frame sync pol, 1 - active low
		 */
		pcr |= (DAVINCI_MCBSP_PCR_CLKXP | DAVINCI_MCBSP_PCR_CLKRP |
			DAVINCI_MCBSP_PCR_FSXP | DAVINCI_MCBSP_PCR_FSRP);
		break;
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		dev->mode = MOD_DSP_A;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		dev->mode = MOD_DSP_B;
		break;
	default:
		printk(KERN_ERR "%s:bad format\n", __func__);
		return -EINVAL;
	}
	davinci_mcbsp_write_reg(dev, DAVINCI_MCBSP_SRGR_REG, srgr);
	davinci_mcbsp_write_reg(dev, DAVINCI_MCBSP_PCR_REG, pcr);
	return 0;
}

static int davinci_i2s_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	struct davinci_mcbsp_dev *dev = cpu_dai->private_data;

	cpu_dai->dma_data = dev->dma_params[substream->stream];

	return 0;
}

static int davinci_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct davinci_pcm_dma_params *dma_params = rtd->dai->cpu_dai->dma_data;
	struct davinci_mcbsp_dev *dev = rtd->dai->cpu_dai->private_data;
	struct snd_soc_dai *codec_dai = dev->codec_dai;
	struct snd_interval *i = NULL;
	int mcbsp_word_length;
	int bits_per_sample;
	int bits_per_frame;
	unsigned int rcr, xcr, srgr;
	int channels;
	int format;
	int element_cnt = 1;
	int right_first = 0;

	i = hw_param_interval(params, SNDRV_PCM_HW_PARAM_SAMPLE_BITS);
	bits_per_sample = snd_interval_value(i);
	i = hw_param_interval(params, SNDRV_PCM_HW_PARAM_FRAME_BITS);
	bits_per_frame = snd_interval_value(i);
	srgr = DAVINCI_MCBSP_SRGR_FSGM |
		DAVINCI_MCBSP_SRGR_FPER(bits_per_frame - 1) |
		DAVINCI_MCBSP_SRGR_FWID(bits_per_sample - 1);

	/* general line settings */
	davinci_mcbsp_write_reg(dev, DAVINCI_MCBSP_SPCR_REG,
				DAVINCI_MCBSP_SPCR_RINTM(3) |
				DAVINCI_MCBSP_SPCR_XINTM(3) |
				DAVINCI_MCBSP_SPCR_FREE);

	rcr = DAVINCI_MCBSP_RCR_RFIG;
	xcr = DAVINCI_MCBSP_XCR_XFIG;
	if (dev->mode == MOD_DSP_A) {
		rcr |= DAVINCI_MCBSP_RCR_RDATDLY(0);
		xcr |= DAVINCI_MCBSP_XCR_XDATDLY(0);
	} else {
		rcr |= DAVINCI_MCBSP_RCR_RDATDLY(1);
		xcr |= DAVINCI_MCBSP_XCR_XDATDLY(1);
	}
	channels = params_channels(params);
	format = params_format(params);
	/* Determine xfer data type */
	if (channels == 2) {
		/* Combining both channels into 1 element will x10 the
		 * amount of time between servicing the dma channel, increase
		 * effiency, and reduce the chance of overrun/underrun. But,
		 * it will result in the left & right channels being swapped.
		 * So, let the codec know to swap them back.
		 *
		 * It is x10 instead of x2 because the clock from the codec
		 * runs at mclk speed, independent of the sample rate.
		 * So, having an entire frame at once means it has to be
		 * serviced at the sample rate instead of the mclk speed.
		 */
		right_first = 1;
		dma_params->convert_mono_stereo = 0;
		switch (format) {
		case SNDRV_PCM_FORMAT_S8:
			dma_params->data_type = 2;	/* 2 byte frame */
			mcbsp_word_length = DAVINCI_MCBSP_WORD_16;
			break;
		case SNDRV_PCM_FORMAT_S16_LE:
			dma_params->data_type = 4;	/* 4 byte frame */
			mcbsp_word_length = DAVINCI_MCBSP_WORD_32;
			break;
		case SNDRV_PCM_FORMAT_S32_LE:
			right_first = 0;
			element_cnt = 2;
			dma_params->data_type = 4;	/* 4 byte element */
			mcbsp_word_length = DAVINCI_MCBSP_WORD_32;
			break;
		default:
			printk(KERN_WARNING
					"davinci-i2s: unsupported PCM format");
			return -EINVAL;
		}
	} else {
		dma_params->convert_mono_stereo = 1;
		/* 1 element in ram becomes 2 for stereo */
		element_cnt = 2;
		switch (format) {
		case SNDRV_PCM_FORMAT_S8:
			/* 1 byte frame in ram */
			dma_params->data_type = 1;
			mcbsp_word_length = DAVINCI_MCBSP_WORD_8;
			break;
		case SNDRV_PCM_FORMAT_S16_LE:
			/* 2 byte frame in ram */
			dma_params->data_type = 2;
			mcbsp_word_length = DAVINCI_MCBSP_WORD_16;
			break;
		case SNDRV_PCM_FORMAT_S32_LE:
			/* 4 byte element */
			dma_params->data_type = 4;
			mcbsp_word_length = DAVINCI_MCBSP_WORD_32;
			break;
		default:
			printk(KERN_WARNING
					"davinci-i2s: unsupported PCM format");
			return -EINVAL;
		}
	}
	if (codec_dai->dai_ops.inform_channel_order)
		codec_dai->dai_ops.inform_channel_order(codec_dai, right_first);
	rcr |=	DAVINCI_MCBSP_RCR_RFRLEN1(element_cnt - 1);
	xcr |=  DAVINCI_MCBSP_XCR_XFRLEN1(element_cnt - 1);

	rcr |=  DAVINCI_MCBSP_RCR_RWDLEN1(mcbsp_word_length) |
		DAVINCI_MCBSP_RCR_RWDLEN2(mcbsp_word_length);
	xcr |=  DAVINCI_MCBSP_XCR_XWDLEN1(mcbsp_word_length) |
		DAVINCI_MCBSP_XCR_XWDLEN2(mcbsp_word_length);
	davinci_mcbsp_write_reg(dev, DAVINCI_MCBSP_SRGR_REG, srgr);
	davinci_mcbsp_write_reg(dev, DAVINCI_MCBSP_RCR_REG, rcr);
	davinci_mcbsp_write_reg(dev, DAVINCI_MCBSP_XCR_REG, xcr);
	return 0;
}

static void codec_mute_deferred(struct work_struct *work)
{
	struct davinci_mcbsp_dev *dev = container_of(work,
			struct davinci_mcbsp_dev, deferred_mute_work);
	struct snd_soc_dai *codec_dai = dev->codec_dai;

	snd_soc_dai_digital_mute(codec_dai, dev->dac_active ^ 1);

	if (!dev->dac_active)
		davinci_mcbsp_stop(dev, 1);
}

static int davinci_i2s_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct davinci_mcbsp_dev *dev = rtd->dai->cpu_dai->private_data;
	int ret = 0;
	int dac_active = 1;
	int playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		davinci_mcbsp_start(dev, playback);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* don't stop driving data lines
		 * until digital_mute done
		 */
		dac_active = 0;
		if (!playback)
			davinci_mcbsp_stop(dev, playback);
		break;
	default:
		ret = -EINVAL;
	}

	if (playback) {
		dev->dac_active = dac_active;
		schedule_work(&dev->deferred_mute_work);
	}
	return ret;
}

static int davinci_i2s_probe(struct platform_device *pdev,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_machine *machine = socdev->machine;
	struct snd_soc_dai *cpu_dai = machine->dai_link[pdev->id].cpu_dai;
	struct snd_soc_dai *codec_dai = machine->dai_link[pdev->id].codec_dai;
	struct davinci_mcbsp_dev *dev;
	struct resource *mem, *ioarea;
	struct evm_snd_platform_data *pdata;
	int ret;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}

	ioarea = request_mem_region(mem->start, (mem->end - mem->start) + 1,
				    pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "McBSP region already claimed\n");
		return -EBUSY;
	}

	dev = kzalloc(sizeof(struct davinci_mcbsp_dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		goto err_release_region;
	}
	INIT_WORK(&dev->deferred_mute_work, codec_mute_deferred);
	dev->codec_dai = codec_dai;

	cpu_dai->private_data = dev;

	dev->clk = clk_get(&pdev->dev, "McBSPCLK");
	if (IS_ERR(dev->clk)) {
		ret = -ENODEV;
		goto err_free_mem;
	}
	clk_enable(dev->clk);

	dev->base = (void __iomem *)IO_ADDRESS(mem->start);
	pdata = pdev->dev.platform_data;

	dev->dma_params[SNDRV_PCM_STREAM_PLAYBACK] = &davinci_i2s_pcm_out;
	dev->dma_params[SNDRV_PCM_STREAM_PLAYBACK]->channel = pdata->tx_dma_ch;
	dev->dma_params[SNDRV_PCM_STREAM_PLAYBACK]->dma_addr =
	    (dma_addr_t)(io_v2p(dev->base) + DAVINCI_MCBSP_DXR_REG);

	dev->dma_params[SNDRV_PCM_STREAM_CAPTURE] = &davinci_i2s_pcm_in;
	dev->dma_params[SNDRV_PCM_STREAM_CAPTURE]->channel = pdata->rx_dma_ch;
	dev->dma_params[SNDRV_PCM_STREAM_CAPTURE]->dma_addr =
	    (dma_addr_t)(io_v2p(dev->base) + DAVINCI_MCBSP_DRR_REG);

	return 0;

err_free_mem:
	kfree(dev);
err_release_region:
	release_mem_region(mem->start, (mem->end - mem->start) + 1);

	return ret;
}

static void davinci_i2s_remove(struct platform_device *pdev,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_machine *machine = socdev->machine;
	struct snd_soc_dai *cpu_dai = machine->dai_link[pdev->id].cpu_dai;
	struct davinci_mcbsp_dev *dev = cpu_dai->private_data;
	struct resource *mem;

	clk_disable(dev->clk);
	clk_put(dev->clk);
	dev->clk = NULL;
	cancel_work_sync(&dev->deferred_mute_work);

	kfree(dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
}
#define DAVINCI_I2S_RATES	(SNDRV_PCM_RATE_8000_96000 |\
	SNDRV_PCM_RATE_5512 | SNDRV_PCM_RATE_KNOT | SNDRV_PCM_RATE_CONTINUOUS)

struct snd_soc_dai davinci_i2s_dai = {
	.name = "davinci-i2s",
	.id = 0,
	.type = SND_SOC_DAI_I2S,
	.probe = davinci_i2s_probe,
	.remove = davinci_i2s_remove,
	.playback = {
		/* fixme, codecs shouldn't need to lie */
		.channels_min = 1, /* lie, I2S dma will convert to stereo*/
		.channels_max = 2,
		.rates = DAVINCI_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		/* fixme, codecs shouldn't need to lie */
		.channels_min = 1, /* lie, I2S dma will convert from stereo */
		.channels_max = 2,
		.rates = DAVINCI_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = {
		.startup = davinci_i2s_startup,
		.trigger = davinci_i2s_trigger,
		.hw_params = davinci_i2s_hw_params,},
	.dai_ops = {
		.set_fmt = davinci_i2s_set_dai_fmt,
	},
};
EXPORT_SYMBOL_GPL(davinci_i2s_dai);

MODULE_AUTHOR("Vladimir Barinov");
MODULE_DESCRIPTION("TI DAVINCI I2S (McBSP) SoC Interface");
MODULE_LICENSE("GPL");
