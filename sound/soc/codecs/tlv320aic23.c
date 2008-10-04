/*
 * Texas Instruments TLV320AIC23 low power audio CODEC
 * ALSA SoC CODEC driver
 * Copyright (C) 2008 Boundary Devices
 * based on	wm8731.c by Richard Purdie
 * Copyright 2005 Openedhand Ltd.
 * which was based on wm8753.c by Liam Girdwood
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "tlv320aic23.h"

/* AIC23 driver private data */
struct aic23 {
	struct snd_soc_codec codec;
	int mclk;
	int master;
	int datfm;
	int requested_adc;
	int requested_dac;
	int right_first;
	u16 reg_cache[AIC23_NUM_CACHE_REGS];	/* shadow registers */
};

/* ---------------------------------------------------------------------
 * Register access routines
 */
static unsigned int aic23_read_cache(struct snd_soc_codec *codec,
					 unsigned int reg)
{
	if (reg >= AIC23_NUM_CACHE_REGS) {
		printk(KERN_ERR "%s: Invalid register %i\n", __func__, reg);
		return -EINVAL;
	}
	return ((u16*)codec->reg_cache)[reg];
}

/* This is a write-only codec */
/*
static unsigned int aic23_read(struct snd_soc_codec *codec,
				   unsigned int reg)
{
	return aic23_read_cache(codec,reg);
}
*/

static int aic23_write(struct snd_soc_codec *codec, unsigned int reg,
			   unsigned int value)
{
	u8 data[2];

	if (reg >= AIC23_NUM_REGS) {
		printk(KERN_ERR "%s: Invalid register %i\n", __func__, reg);
		return -EINVAL;
	}
	value &= 0x1ff;
	if (0) printk(KERN_ERR "%s: reg=%i val=0x%x\n", __func__, reg, value);
	/* update cache */
	if (reg < AIC23_NUM_CACHE_REGS)
		((u16*)codec->reg_cache)[reg] = value;
	/* data is
	 *   D15..D9 WM8731 register offset
	 *   D8...D0 register data
	 */
	value |= (reg<<9);
	/* store high order byte 1st, (big endian mode) */
	data[0] = (u8)(value>>8);
	data[1] = (u8)(value);
	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;
	else
		return -EIO;
}

/*
 * Common Crystals used
 * 11.2896 Mhz /128 = *88.2k  /192 = 58.8k
 * 12.0000 Mhz /125 = *96k    /136 = 88.235K
 * 12.2880 Mhz /128 = *96k    /192 = 64k
 * 16.9344 Mhz /128 = 132.3k /192 = *88.2k
 * 18.4320 Mhz /128 = 144k   /192 = *96k
 */

/*
 * Normal BOSR 0-256/2 = 128, 1-384/2 = 192
 * USB BOSR 0-250/2 = 125, 1-272/2 = 136
 */
static const int bosr_usb_divisor_table[] = {
	128, 125, 192, 136
};
#define LOWER_GROUP (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<6)|(1<<7)
#define UPPER_GROUP (1<<8)|(1<<9)|(1<<10)|(1<<11)     |(1<<15)
static const unsigned short sr_valid_mask[] = {
	LOWER_GROUP|UPPER_GROUP,	/* Normal, bosr - 0*/
	LOWER_GROUP|UPPER_GROUP,	/* Normal, bosr - 1*/
	LOWER_GROUP,			/* Usb, bosr - 0*/
	UPPER_GROUP,			/* Usb, bosr - 1*/
};
/*
 * Every divisor is a factor of 11*12
 */
#define SR_MULT (11*12)
#define A(x) (x)? (SR_MULT/x) : 0
static const unsigned char sr_adc_mult_table[] = {
	A(2), A(2), A(12), A(12),  A(0), A(0), A(3), A(1),
	A(2), A(2), A(11), A(11),  A(0), A(0), A(0), A(1)
};
static const unsigned char sr_dac_mult_table[] = {
	A(2), A(12), A(2), A(12),  A(0), A(0), A(3), A(1),
	A(2), A(11), A(2), A(11),  A(0), A(0), A(0), A(1)
};

unsigned get_score(int adc, int adc_l, int adc_h, int need_adc,
		int dac, int dac_l, int dac_h, int need_dac)
{
	if ((adc >= adc_l) && (adc <= adc_h) &&
			(dac >= dac_l) && (dac <= dac_h)) {
		unsigned diff_adc = need_adc - adc;
		unsigned diff_dac = need_dac - dac;
		unsigned score;
		if (((int)diff_adc) < 0)
			diff_adc = -diff_adc;
		if (((int)diff_dac) < 0)
			diff_dac = -diff_dac;
		score = diff_adc + diff_dac;
		return score;
	}
	return 0xffffffff;
}
int find_rate(int mclk, u32 need_adc, u32 need_dac)
{
	int i,j;
	int best_i = -1;
	int best_j = -1;
	int best_div = 0;
	unsigned best_score = 0xffffffff;
	int adc_l, adc_h, dac_l, dac_h;

	need_adc *= SR_MULT;
	need_dac *= SR_MULT;
//	printk(KERN_ERR "need_adc=%i need_dac=%i\n", need_adc/SR_MULT, need_dac/SR_MULT);
	/*
	 * rates given are +/- 1/32
	 */
	adc_l = need_adc - (need_adc >> 5);
	adc_h = need_adc + (need_adc >> 5);
	dac_l = need_dac - (need_dac >> 5);
	dac_h = need_dac + (need_dac >> 5);
	for (i = 0; i < 4; i++) {
		int base = mclk / bosr_usb_divisor_table[i];
		int mask = sr_valid_mask[i];
		for (j = 0; j < 16; j++, mask >>= 1) {
			int adc; 
			int dac;
			int score;
			if ((mask & 1) == 0)
				continue;
			adc = base * sr_adc_mult_table[j]; 
			dac = base * sr_dac_mult_table[j];
			score = get_score(adc, adc_l, adc_h, need_adc,
					dac, dac_l, dac_h, need_dac);
			if (best_score > score) {
//				printk(KERN_ERR "adc=%i dac=%i\n", adc/SR_MULT, dac/SR_MULT);
//				printk(KERN_ERR "best_score=%u score=%u\n", best_score, score);
				best_score = score;
				best_i = i;
				best_j = j;
				best_div = 0;
			}
			score = get_score((adc >> 1), adc_l, adc_h, need_adc,
					(dac >> 1), dac_l, dac_h, need_dac);
			/* prefer to have a /2 */
			if ((score != 0xffffffff) && (best_score >= score)) {
//				printk(KERN_ERR "adc=%i dac=%i\n", adc/SR_MULT, dac/SR_MULT);
//				printk(KERN_ERR "best_score=%u score=%u\n", best_score, score);
				best_score = score;
				best_i = i;
				best_j = j;
				best_div = 1;
			}
		}
	}
	return (best_j << 2) | best_i | SRC_CLKIN(best_div);
}

static void get_current_sample_rates(struct snd_soc_codec *codec, int mclk,
		u32 *sample_rate_adc, u32 *sample_rate_dac)
{
	int src = aic23_read_cache(codec, AIC23_SAMPLE_RATE_CONTROL);
	int sr = (src >> 2) & 0x0f;
	int val = (mclk / bosr_usb_divisor_table[src & 3]);
	int adc = (val * sr_adc_mult_table[sr]) / SR_MULT;
	int dac = (val * sr_dac_mult_table[sr]) / SR_MULT;
	if (src & SRC_CLKIN_HALF) {
		adc >>= 1;
		dac >>= 1;
	}
	*sample_rate_adc = adc;
	*sample_rate_dac = dac;
}

static int set_sample_rate_control(struct snd_soc_codec *codec, int mclk,
		u32 sample_rate_adc, u32 sample_rate_dac)
{
	/* Search for the right sample rate */
	int data = find_rate(mclk, sample_rate_adc, sample_rate_dac);
	if (data < 0) {
		printk(KERN_ERR "%s:Invalid rate %u,%u requested\n",
				__func__, sample_rate_adc,sample_rate_dac);
		return -EINVAL;
	}
	aic23_write(codec, AIC23_SAMPLE_RATE_CONTROL, data);
	if (1) {
		int adc,dac;
		get_current_sample_rates(codec, mclk, &adc, &dac);
		printk(KERN_ERR "actual samplerate = %u,%u reg=%x\n", 
			adc, dac, data);
	}
	return 0;
}
/* ---------------------------------------------------------------------
 * Digital Audio Interface Operations
 */
static int aic23_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params)
{
	int ret;
	unsigned data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct aic23 *aic23 = codec->private_data;
	u32 sample_rate_adc = aic23->requested_adc;
	u32 sample_rate_dac = aic23->requested_dac;
	u32 sample_rate = params_rate(params);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		aic23->requested_dac = sample_rate_dac = sample_rate; 
		if (!sample_rate_adc)
			sample_rate_adc = sample_rate;
	} else {
		aic23->requested_adc = sample_rate_adc = sample_rate;
		if (!sample_rate_dac)
			sample_rate_dac = sample_rate;
	}
	ret = set_sample_rate_control(codec, aic23->mclk, sample_rate_adc, sample_rate_dac);
	if (ret < 0)
		return ret;
	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S16_BE:
		data = DAF_IWL_16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S20_3BE:
		data = DAF_IWL_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_BE:
		data = DAF_IWL_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
	case SNDRV_PCM_FORMAT_S32_BE:
		data = DAF_IWL_32;
		break;
	default:
		printk(KERN_ERR "%s: bad format\n", __func__);
		return -EINVAL;
	}
	data |= DAF_MASTER(aic23->master) | aic23->datfm;
	if (aic23->right_first)
		data |= DAF_LRSWAP_ON;
	aic23_write(codec, AIC23_DIGITAL_AUDIO_FORMAT, data);
	return 0;
}

/*
 * aic23_mute - Mute control to reduce noise when changing audio format
 */
static int aic23_mute_codec(struct snd_soc_codec *codec, int mute)
{
	u16 reg = aic23_read_cache(codec, AIC23_DIGITAL_AUDIO_CONTROL);
	if (mute)
		reg |= DAC_SOFT_MUTE;
	else
		reg &= ~DAC_SOFT_MUTE;
	aic23_write(codec, AIC23_DIGITAL_AUDIO_CONTROL, reg);
	return 0;
}

static int aic23_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	/* set active */
	aic23_write(codec, AIC23_DIGITAL_INTERFACE_ACT, 0x0001);
	return 0;
}

static void aic23_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	struct aic23 *aic23 = codec->private_data;

	/* deactivate */
	if (!codec->active) {
		udelay(50);
		aic23_write(codec, AIC23_DIGITAL_INTERFACE_ACT, 0x0);
	}
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		aic23->requested_dac = 0;
	else
		aic23->requested_adc = 0;
}

/*
 * aic23_mute - Mute control to reduce noise when changing audio format
 */
static int aic23_mute(struct snd_soc_dai *dai, int mute)
{
	return aic23_mute_codec(dai->codec, mute);
}
static int aic23_inform_channel_order(struct snd_soc_dai *codec_dai, int right_first)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic23 *aic23 = codec->private_data;
	aic23->right_first = right_first;
	return 0;
}

static int aic23_set_sysclk(struct snd_soc_dai *codec_dai,
			    int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic23 *aic23 = codec->private_data;
	aic23->mclk = freq;
	return 0;
}

static int aic23_set_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic23 *aic23 = codec->private_data;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aic23->master = 1;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aic23->master = 0;
		break;
	default:
		printk(KERN_ERR "%s:bad master\n", __func__);
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		aic23->datfm = DAF_FOR_I2S;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		aic23->datfm = DAF_FOR_DSP;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		aic23->datfm = DAF_FOR_DSP | DAF_MSB_ON_2ND_BCLK;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		aic23->datfm = DAF_FOR_RIGHT_ALIGN;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		aic23->datfm = DAF_FOR_LEFT_ALIGN;
		break;
	default:
		printk(KERN_ERR "%s:bad format\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int aic23_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	u16 reg = aic23_read_cache(codec, AIC23_POWER_DOWN_CONTROL) & 0x7f;

	switch (level) {
	case SND_SOC_BIAS_ON:
		/* vref/mid, osc on, dac unmute */
		aic23_write(codec, AIC23_POWER_DOWN_CONTROL, reg);
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		/* power down clock output */
		aic23_write(codec, AIC23_POWER_DOWN_CONTROL,
				reg | (1<<PDC_CLK_BIT));
		break;
	case SND_SOC_BIAS_OFF:
		/* everything off, dac mute, inactive */
		aic23_write(codec, AIC23_DIGITAL_INTERFACE_ACT, 0x0);
		aic23_write(codec, AIC23_POWER_DOWN_CONTROL, 0xffff);
		break;
	}
	codec->bias_level = level;
	return 0;
}

/* ---------------------------------------------------------------------
 * Digital Audio Interface Definition
 */
#define AIC23_RATES	SNDRV_PCM_RATE_8000_96000 |\
	SNDRV_PCM_RATE_5512 | SNDRV_PCM_RATE_KNOT | SNDRV_PCM_RATE_CONTINUOUS

#define AIC23_FORMATS_LE (SNDRV_PCM_FMTBIT_S16_LE |\
			 SNDRV_PCM_FMTBIT_S20_3LE |\
			 SNDRV_PCM_FMTBIT_S24_LE |\
			 SNDRV_PCM_FMTBIT_S32_LE)

#define AIC23_FORMATS_BE (SNDRV_PCM_FMTBIT_S16_BE |\
			 SNDRV_PCM_FMTBIT_S20_3BE |\
			 SNDRV_PCM_FMTBIT_S24_BE |\
			 SNDRV_PCM_FMTBIT_S32_BE)

#define AIC23_FORMATS AIC23_FORMATS_LE | AIC23_FORMATS_BE

struct snd_soc_dai aic23_dai = {
	.name = "tlv320aic23",
	.playback = {
		.stream_name = "HiFi Playback",
		/* fixme, codecs shouldn't need to lie */
		.channels_min = 1, /* lie, I2S dma will convert to stereo*/
		.channels_max = 2,
		.rates = AIC23_RATES,
		.formats = AIC23_FORMATS,
	},
	.capture = {
		.stream_name = "HiFi Capture",
		/* fixme, codecs shouldn't need to lie */
		.channels_min = 1, /* lie, I2S dma will convert from stereo */
		.channels_max = 2,
		.rates = AIC23_RATES,
		.formats = AIC23_FORMATS,
	},
	.ops = {
		.prepare = aic23_pcm_prepare,
		.hw_params = aic23_hw_params,
		.shutdown = aic23_shutdown,
	},
	.dai_ops = {
		.digital_mute = aic23_mute,
		.set_sysclk = aic23_set_sysclk,
		.set_fmt = aic23_set_fmt,
		.inform_channel_order = aic23_inform_channel_order,
	},
};
EXPORT_SYMBOL_GPL(aic23_dai);

/* ---------------------------------------------------------------------
 * ALSA controls
 */
static const char *aic23_capture_src_text[] = {"Line In", "Mic"};
static const char *aic23_deemph[] = {"None", "32Khz", "44.1Khz", "48Khz"};
static const char *aic23_sidetone[] = {"-6db","-9db","-12db","-18db","0db"};

static const struct soc_enum aic23_enum[] = {
	SOC_ENUM_SINGLE(AIC23_ANALOG_AUDIO_CONTROL, AAC_INSEL_BIT, \
			2, aic23_capture_src_text),
	SOC_ENUM_SINGLE(AIC23_ANALOG_AUDIO_CONTROL, AAC_SIDE_TONE_VOL_BIT, \
			5, aic23_sidetone),
	SOC_ENUM_SINGLE(AIC23_DIGITAL_AUDIO_CONTROL, DAC_DEEMP_BIT, \
			4, aic23_deemph),
};

static const struct snd_kcontrol_new aic23_snd_controls[] = {
	SOC_DOUBLE_R("Capture Volume", AIC23_LEFT_INPUT_VOLUME, \
		AIC23_RIGHT_INPUT_VOLUME, 0, 0x1f, 0),
	SOC_DOUBLE_R("Line Capture Switch", AIC23_LEFT_INPUT_VOLUME, \
		AIC23_RIGHT_INPUT_VOLUME, LRIV_MUTE_BIT, 1, 1),
	SOC_DOUBLE_R("Master Playback Volume", AIC23_LEFT_OUTPUT_VOLUME, \
		AIC23_RIGHT_OUTPUT_VOLUME, 0, 0x7f, 0),
	SOC_DOUBLE_R("Master Playback ZC Switch", AIC23_LEFT_OUTPUT_VOLUME, \
		AIC23_RIGHT_INPUT_VOLUME, LROV_ZERO_CROSS_BIT, 1, 0),
	SOC_SINGLE("Mic Boost (+20dB)", AIC23_ANALOG_AUDIO_CONTROL,
		AAC_MIC_BOOST_BIT, 1, 0),
	SOC_SINGLE("Capture Mic Switch", AIC23_ANALOG_AUDIO_CONTROL, \
		AAC_MIC_MUTE_BIT, 1, 1),
	SOC_ENUM("Sidetone Playback Volume", aic23_enum[1]),

	SOC_SINGLE("ADC High Pass Filter Switch", AIC23_DIGITAL_AUDIO_CONTROL, \
		DAC_HIGH_PASS_FILTER_BIT, 1, 1),
	SOC_ENUM("Playback De-emphasis", aic23_enum[2]),
};

/* add non dapm controls */
static int aic23_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(aic23_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				  snd_soc_cnew(&aic23_snd_controls[i],
						codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

/* Output Mixer */
static const struct snd_kcontrol_new aic23_output_mixer_controls[] = {
SOC_DAPM_SINGLE("HiFi Playback Switch", AIC23_ANALOG_AUDIO_CONTROL, \
	AAC_DAC_BIT, 1, 0),
SOC_DAPM_SINGLE("Line Bypass Switch", AIC23_ANALOG_AUDIO_CONTROL, \
	AAC_BYPASS_BIT, 1, 0),
SOC_DAPM_SINGLE("Mic Sidetone Switch", AIC23_ANALOG_AUDIO_CONTROL, \
	AAC_SIDE_TONE_ENABLE_BIT, 1, 0),
};

/* Input mux */
static const struct snd_kcontrol_new aic23_input_mux_controls =
SOC_DAPM_ENUM("Input Select", aic23_enum[0]);

static const struct snd_soc_dapm_widget aic23_dapm_widgets[] = {
SND_SOC_DAPM_DAC("DAC", "HiFi Playback", AIC23_POWER_DOWN_CONTROL, PDC_DAC_BIT, 1),
SND_SOC_DAPM_ADC("ADC", "HiFi Capture", AIC23_POWER_DOWN_CONTROL, PDC_ADC_BIT, 1),
SND_SOC_DAPM_MIXER("Output Mixer", AIC23_POWER_DOWN_CONTROL, PDC_OUT_BIT, 1,
	aic23_output_mixer_controls,
	ARRAY_SIZE(aic23_output_mixer_controls)),
SND_SOC_DAPM_MUX("Capture Source", SND_SOC_NOPM, 0, 0, &aic23_input_mux_controls),
SND_SOC_DAPM_PGA("Line Input", AIC23_POWER_DOWN_CONTROL, PDC_LINE_BIT, 1, NULL, 0),
SND_SOC_DAPM_MICBIAS("Mic Bias", AIC23_POWER_DOWN_CONTROL, PDC_MIC_BIT, 1),
SND_SOC_DAPM_OUTPUT("LHPOUT"),
SND_SOC_DAPM_OUTPUT("RHPOUT"),
SND_SOC_DAPM_OUTPUT("LOUT"),
SND_SOC_DAPM_OUTPUT("ROUT"),
SND_SOC_DAPM_INPUT("LLINEIN"),
SND_SOC_DAPM_INPUT("RLINEIN"),
SND_SOC_DAPM_INPUT("MICIN"),
};

static const struct snd_soc_dapm_route intercon[] = {
	/* output mixer */
	{"Output Mixer", "HiFi Playback Switch", "DAC"},
	{"Output Mixer", "Line Bypass Switch", "Line Input"},
	{"Output Mixer", "Mic Sidetone Switch", "Mic Bias"},

	/* outputs */
	{"LHPOUT", NULL, "Output Mixer"},
	{"RHPOUT", NULL, "Output Mixer"},
	{"LOUT", NULL, "Output Mixer"},
	{"ROUT", NULL, "Output Mixer"},

	/* inputs */
	{"Line Input", NULL, "LLINEIN"},
	{"Line Input", NULL, "RLINEIN"},
	{"Mic Bias", NULL, "MICIN"},

	/* input mux */
	{"Capture Source", "Line In", "Line Input"},
	{"Capture Source", "Mic", "Mic Bias"},
	{"ADC", NULL, "Capture Source"},
};

static int aic23_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, aic23_dapm_widgets,
				  ARRAY_SIZE(aic23_dapm_widgets));
	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));
	snd_soc_dapm_new_widgets(codec);
	return 0;
}

/* ---------------------------------------------------------------------
 */

static int aic23_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	aic23_write(codec, AIC23_DIGITAL_INTERFACE_ACT, 0x0);
	aic23_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int aic23_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	int reg;

	/* Sync reg_cache with the hardware */
	for (reg = 0; reg < AIC23_NUM_CACHE_REGS; reg++) {
		unsigned val = aic23_read_cache(codec, reg);
		aic23_write(codec, reg, val);
	}
	aic23_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	aic23_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}
/* ************************************ */

/*
 * initialise the aic23 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int aic23_init(struct snd_soc_codec *codec, struct aic23 *aic23)
{
	aic23_write(codec, AIC23_RESET, 0);
	/* all off
	 * This addresses the problem of the digital filters
	 * not being initialized when MCLK is removed
	 * slea037.pdf section 2 
	 * "Noise Fixed by Toggling Bit D7 of Power-Down Control"
	 */
	aic23_write(codec, AIC23_POWER_DOWN_CONTROL, 0x01ff);

	/* power on device, only oscillator is on */
	aic23_write(codec, AIC23_POWER_DOWN_CONTROL, 0x00df);
	aic23_write(codec, AIC23_POWER_DOWN_CONTROL, 0x005f);
	aic23_write(codec, AIC23_DIGITAL_INTERFACE_ACT, 0x0000);

	/* set the update bits */
	aic23_write(codec, AIC23_LEFT_INPUT_VOLUME, LRIV_DEFAULT | LRIV_MUTE);
	aic23_write(codec, AIC23_RIGHT_INPUT_VOLUME, LRIV_DEFAULT | LRIV_MUTE);
	aic23_write(codec, AIC23_LEFT_OUTPUT_VOLUME,
			LROV_DEFAULT | LROV_ZERO_CROSS);
	aic23_write(codec, AIC23_RIGHT_OUTPUT_VOLUME,
			LROV_DEFAULT | LROV_ZERO_CROSS);
	aic23_write(codec, AIC23_ANALOG_AUDIO_CONTROL,
			AAC_DAC_ON | AAC_MIC_BOOST_20DB | AAC_INSEL_MIC);
	aic23_write(codec, AIC23_DIGITAL_AUDIO_CONTROL,
			DAC_SOFT_MUTE | DAC_DEEMP_44K);
	aic23_write(codec, AIC23_DIGITAL_AUDIO_FORMAT,
			DAF_IWL_16 | DAF_FOR_DSP | DAF_MASTER_MODE);
	set_sample_rate_control(codec, aic23->mclk, 44100, 44100);
	return 0;
}


static int aic23_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct aic23_setup_data *setup_data = socdev->codec_data;
	struct snd_soc_codec *codec;
	struct aic23 *aic23;
	int ret = 0;

	pr_info("aic23 Audio Codec");
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (!codec) {
		ret = -ENOMEM;
		goto exit1;
	}

	aic23 = kzalloc(sizeof(struct aic23), GFP_KERNEL);
	if (!aic23) {
		ret = -ENOMEM;
		goto exit2;
	}
	aic23->master = 1;
	aic23->datfm = DAF_FOR_DSP;
	aic23->mclk = 12000000;

	codec->name = "aic23";
	codec->owner = THIS_MODULE;
	codec->private_data = aic23;
	codec->reg_cache_size = AIC23_NUM_CACHE_REGS;
	codec->reg_cache = aic23->reg_cache;
	codec->read = aic23_read_cache;
	codec->write = aic23_write;
	codec->set_bias_level = aic23_set_bias_level;
	codec->dai = &aic23_dai;
	codec->num_dai = 1;
	codec->hw_write = setup_data->hw_write;

	socdev->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	ret = aic23_init(codec, aic23);
	if (ret < 0)
		goto exit3;
	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "aic23: failed to create pcms\n");
		goto exit3;
	}
	aic23_add_controls(codec);
	aic23_add_widgets(codec);
	ret = snd_soc_register_card(socdev);
	if (ret < 0) {
		printk(KERN_ERR "aic23: failed to register card\n");
		goto exit4;
	}
	return ret;
exit4:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
exit3:
	codec->private_data = NULL;
	kfree(aic23);
exit2:
	socdev->codec = NULL;
	kfree(codec);
exit1:
	return ret;
}

/* power down chip */
static int aic23_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	aic23_set_bias_level(codec, SND_SOC_BIAS_OFF);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	kfree(codec->private_data);
	kfree(codec);
	return 0;
}

struct snd_soc_codec_device aic23_soc_codec_dev = {
	.probe = 	aic23_probe,
	.remove = 	aic23_remove,
	.suspend = 	aic23_suspend,
	.resume =	aic23_resume,
};
EXPORT_SYMBOL_GPL(aic23_soc_codec_dev);

MODULE_DESCRIPTION("ASoC TLV320AIC23 codec driver");
MODULE_AUTHOR("Troy Kisky <troy.kisky@boundarydevices.com>");
MODULE_LICENSE("GPL");
