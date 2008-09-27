/*
 * linux/sound/oss/davinci-audio-aic23.c
 *
 * Glue audio driver for TI TLV320AIC23 codec
 *
 * Copyright (c) 2000 Nicolas Pitre <nico at cam.org>
 * Copyright (C) 2001, Steve Johnson <stevej at ridgerun.com>
 * Copyright (C) 2004 Texas Instruments, Inc.
 * Copyright (C) 2005 Dirk Behme <dirk.behme at de.bosch.com>
 *	2005-10-18 Rishi Bhattacharya - Support for AIC33 codec and Davinci DM644x Processor
 * Copyright (C) 2007 Boundary Devices
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <sound/davincisound.h>

#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <mach/mcbsp.h>

#include "davinci-audio.h"
#include "davinci-audio-dma-intfc.h"

#define CODEC_NAME				"AIC23"
#define PLATFORM_NAME			"DAVINCI XENON"

/* Define to set the AIC23 as the master w.r.t McBSP */
#define AIC23_MASTER		//commented out means davinci is master
//#define DEBUG
//#define TONE_GEN


// Codec TLV320AIC23
#define LEFT_LINE_VOLUME_ADDR		0x00
#define RIGHT_LINE_VOLUME_ADDR		0x01
#define LEFT_CHANNEL_VOLUME_ADDR	0x02
#define RIGHT_CHANNEL_VOLUME_ADDR	0x03
#define ANALOG_AUDIO_CONTROL_ADDR	0x04
#define DIGITAL_AUDIO_CONTROL_ADDR	0x05
#define POWER_DOWN_CONTROL_ADDR		0x06
#define DIGITAL_AUDIO_FORMAT_ADDR	0x07
#define SAMPLE_RATE_CONTROL_ADDR	0x08
#define DIGITAL_INTERFACE_ACT_ADDR	0x09
#define RESET_CONTROL_ADDR		0x0F

// Left (right) line input volume control register
#define LRS_ENABLED			0x0100
#define LIM_MUTED			0x0080
#define LIV_DEFAULT			0x0017
#define LIV_MAX				0x001f
#define LIV_MIN				0x0000

// Left (right) channel headphone volume control register
#define LZC_ON				0x0080
#define LHV_DEFAULT			0x0079
#define LHV_MAX				0x007f
#define LHV_MIN				0x0000

// Analog audio path control register
#define STE_ENABLED			0x0020
#define DAC_SELECTED			0x0010
#define BYPASS_ON			0x0008
#define INSEL_MIC			0x0004
#define MICM_MUTED			0x0002
#define MICB_20DB			0x0001

// Digital audio path control register
#define DACM_MUTE			0x0008
#define DEEMP_32K			0x0002
#define DEEMP_44K			0x0004
#define DEEMP_48K			0x0006
#define ADCHP_ON			0x0001

// Power control down register
#define DEVICE_POWER_OFF	  	0x0080
#define CLK_OFF				0x0040
#define OSC_OFF				0x0020
#define OUT_OFF				0x0010
#define DAC_OFF				0x0008
#define ADC_OFF				0x0004
#define MIC_OFF				0x0002
#define LINE_OFF			0x0001

// Digital audio interface register
#define MS_MASTER			0x0040
#define LRSWAP_ON			0x0020
#define LRP_ON				0x0010
#define IWL_16				0x0000
#define IWL_20				0x0004
#define IWL_24				0x0008
#define IWL_32				0x000C
#define FOR_I2S				0x0002
#define FOR_DSP				0x0003

// Sample rate control register
#define CLKOUT_HALF			0x0080
#define CLKIN_HALF			0x0040
#define BOSR_384fs			0x0002 // BOSR_272fs when in USB mode
#define USB_CLK_ON			0x0001
#define SR_MASK                         0xf
#define CLKOUT_SHIFT                    7
#define CLKIN_SHIFT                     6
#define SR_SHIFT                        2
#define BOSR_SHIFT                      1

// Digital interface register
#define ACT_ON				0x0001

#define TLV320AIC23ID1                  (0x1a)	// cs low
#define TLV320AIC23ID2                  (0x1b)	// cs high


#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#define PROC_START_FILE "driver/aic23-audio-start"
#define PROC_STOP_FILE  "driver/aic23-audio-stop"
#endif


#ifdef DEBUG
#define DPRINTK(ARGS...)	do { printk("<%s>: ",__FUNCTION__);printk(ARGS); } while (0)
#else
#define DPRINTK( x... )
#endif


/*
 * AUDIO related MACROS
 */
#define DEFAULT_BITPERSAMPLE		16
#define AUDIO_RATE_DEFAULT		44100

/* Select the McBSP For Audio */
#define AUDIO_MCBSP			DAVINCI_MCBSP1

#define REC_MASK 			(SOUND_MASK_LINE | SOUND_MASK_MIC)
#define DEV_MASK 			(REC_MASK | SOUND_MASK_VOLUME)

#define MONO				1
#define STEREO				2

#define SET_VOLUME 			1
#define SET_LINE			2
#define SET_MIC				3
#define SET_RECSRC			4
#define SET_IGAIN			5
#define SET_OGAIN			6
#define SET_BASS			7
#define SET_TREBLE			8
#define SET_MICBIAS			9

#define DEFAULT_OUTPUT_VOLUME	80
#define DEFAULT_INPUT_VOLUME	20	/* 0 ==> mute line in */
#define DEFAULT_INPUT_IGAIN		20
#define DEFAULT_INPUT_OGAIN		100

#define OUTPUT_VOLUME_MIN		LHV_MIN
#define OUTPUT_VOLUME_MAX		LHV_MAX
#define OUTPUT_VOLUME_RANGE		(OUTPUT_VOLUME_MAX - OUTPUT_VOLUME_MIN)
#define OUTPUT_VOLUME_MASK		OUTPUT_VOLUME_MAX

#define INPUT_VOLUME_MIN 		LIV_MIN
#define INPUT_VOLUME_MAX 		LIV_MAX
#define INPUT_VOLUME_RANGE 		(INPUT_VOLUME_MAX - INPUT_VOLUME_MIN)
#define INPUT_VOLUME_MASK 		INPUT_VOLUME_MAX

#define INPUT_GAIN_MIN			LIG_MIN
#define INPUT_GAIN_MAX			LIG_MAX
#define INPUT_GAIN_RANGE		(INPUT_GAIN_MAX - INPUT_GAIN_MIN)

#define OUTPUT_GAIN_MIN			LOG_MIN
#define OUTPUT_GAIN_MAX			LOG_MAX
#define OUTPUT_GAIN_RANGE		(INPUT_GAIN_MAX - INPUT_GAIN_MIN)

#define NUMBER_SAMPLE_RATES_SUPPORTED 10

static audio_stream_t output_stream = {
	.id			= "AIC23 out",
	.dma_dev	= DAVINCI_DMA_MCBSP1_TX,
	.input_or_output = FMODE_WRITE
};

static audio_stream_t input_stream = {
	.id			= "AIC23 in",
	.dma_dev	= DAVINCI_DMA_MCBSP1_RX,
	.input_or_output = FMODE_READ
};

static int audio_dev_id, mixer_dev_id;

static struct aic23_local_info {
	u8	volume;
	u16 volume_reg;
	u8	line;
	u8	mic;
	int recsrc;
	u8 nochan;
	u16 igain;
	u16 ogain;
	u8 micbias;
	u8 bass;
	u8 treble;
	u16 input_volume_reg;
	int mod_cnt;
} aic23_local;

struct sample_rate_reg_info {
	u32 sample_rate;
	u8	control;		/* SR3, SR2, SR1, SR0 and BOSR */
	u8	divider;		/* if 0 CLKIN = MCLK, if 1 CLKIN = MCLK/2 */
};

/* To Store the default sample rate */
static long audio_samplerate = AUDIO_RATE_DEFAULT;

extern struct clk *davinci_mcbsp_get_clock(void);

/* DAC USB-mode sampling rates (MCLK = 12 MHz) */
static const struct sample_rate_reg_info
reg_info[NUMBER_SAMPLE_RATES_SUPPORTED] = {
	{96000, 0x0E, 0},
	{88200, 0x1F, 0},
	{48000, 0x00, 0},
	{44100, 0x11, 0},
	{32000, 0x0C, 0},
	{24000, 0x00, 1},
	{22050, 0x11, 1},
	{16000, 0x0C, 1},
	{ 8000, 0x06, 0},
	{ 4000, 0x06, 1},
};

//.spcr2 = FREE | XINTM(3) | FRST | GRST | XRST,
//.spcr1 = RINTM(3) | RRST,
//	.rcr2  = RWDLEN2(DAVINCI_MCBSP_WORD_16) | RDATDLY(1) | RFRLEN2(0),
//	.rcr1  = RWDLEN1(DAVINCI_MCBSP_WORD_16) | RFRLEN1(1),
//	.xcr2  = XWDLEN2(DAVINCI_MCBSP_WORD_16) | XFIG | XDATDLY(1) | XFRLEN2(0),
//	.xcr1  = XWDLEN1(DAVINCI_MCBSP_WORD_16) | XFRLEN1(1),
//.srgr2 = FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1) | GSYNC | CLKSP,

static struct davinci_mcbsp_reg_cfg initial_config = {
	.spcr2 = FREE | XINTM(3),
	.spcr1 = RINTM(3),
	.rcr2  = RWDLEN2(DAVINCI_MCBSP_WORD_16) | RFIG | RDATDLY(0) | RFRLEN2(0) | RPHASE,
	.rcr1  = RWDLEN1(DAVINCI_MCBSP_WORD_16) | RFRLEN1(0),
	.xcr2  = XWDLEN2(DAVINCI_MCBSP_WORD_16) | XFIG | XDATDLY(0) | XFRLEN2(0) | XPHASE,
	.xcr1  = XWDLEN1(DAVINCI_MCBSP_WORD_16) | XFRLEN1(0),
	.srgr1 = FWID(DEFAULT_BITPERSAMPLE - 1),
	.srgr2 = FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1),
#ifndef AIC23_MASTER
	/* configure McBSP to be the I2S master */
	.pcr0 = FSXM | FSRM | CLKXM | CLKRM | CLKXP | CLKRP,
#else
	/* configure McBSP to be the I2S slave */
	.pcr0 = CLKXP | CLKRP,
#endif /* AIC23_MASTER */
};

#ifdef TONE_GEN
void toneGen(void);
#endif /* TONE_GEN */

static void davinci_aic23_initialize(void *dummy);
static void davinci_aic23_shutdown(void *dummy);
static int  davinci_aic23_ioctl(struct inode *inode, struct file *file,
				uint cmd, ulong arg);
static int  davinci_aic23_probe(void);

#ifdef MODULE
static void davinci_aic23_remove(void);
#endif

static int  davinci_aic23_suspend(void);
static int  davinci_aic23_resume(void);
static inline void aic23_configure(void);
static int  mixer_open(struct inode *inode, struct file *file);
static int  mixer_release(struct inode *inode, struct file *file);
static int  mixer_ioctl(struct inode *inode, struct file *file, uint cmd,
			ulong arg);

#ifdef CONFIG_PROC_FS
static int codec_start(char *buf, char **start, off_t offset, int count,
			int *eof, void *data);
static int codec_stop(char *buf, char **start, off_t offset, int count,
			int *eof, void *data);
#endif


/* File Op structure for mixer */
static struct file_operations davinci_mixer_fops = {
	.open		= mixer_open,
	.release	= mixer_release,
	.ioctl		= mixer_ioctl,
	.owner		= THIS_MODULE
};

/* To store characteristic info regarding the codec for the audio driver */
static audio_state_t aic23_state = {
	.owner 			= THIS_MODULE,
	.output_stream	= &output_stream,
	.input_stream	= &input_stream,
/*	.need_tx_for_rx = 1, //Once the Full Duplex works	*/
	.need_tx_for_rx = 0,
	.hw_init		= davinci_aic23_initialize,
	.hw_shutdown	= davinci_aic23_shutdown,
	.client_ioctl	= davinci_aic23_ioctl,
	.hw_probe		= davinci_aic23_probe,
	.hw_remove		=	__exit_p(davinci_aic23_remove),
	.hw_suspend		= davinci_aic23_suspend,
	.hw_resume		= davinci_aic23_resume,
	.sem = __SEMAPHORE_INITIALIZER(aic23_state.sem,1),
};

/* This will be defined in the audio.h */
static struct file_operations *davinci_audio_fops;

extern int spi_tlv320aic23_write_value(u8 reg, u16 value);
extern int spi_tlv320aic23_init(void);

/* TLV320AIC23 is a write only device */
static __inline__ void audio_aic23_write(u8 address, u16 data)
{
	if (spi_tlv320aic23_write_value(address, data) < 0)
		printk(KERN_INFO "aic23 write failed for reg = %d\n", address);
}

static int aic23_update(int flag, int val)
{
	u16 volume;

	switch (flag) {
	case SET_VOLUME:
		/* Ignore separate left/right channel for now,
			even the codec does support it. */
		val &= 0xff;

		if (val < 0 || val > 100) {
			printk(KERN_ERR "Trying a bad volume value(%d)!\n",val);
			return -EPERM;
		}
		// Convert 0 -> 100 volume to 0x00 (LHV_MIN) -> 0x7f (LHV_MAX) 
		// volume range
		volume = ((val * OUTPUT_VOLUME_RANGE) / 100) + OUTPUT_VOLUME_MIN;
		
		// R/LHV[6:0] 1111111 (+6dB) to 0000000 (-73dB) in 1db steps,
		// default 1111001 (0dB)
		aic23_local.volume_reg &= ~OUTPUT_VOLUME_MASK;
		aic23_local.volume_reg |= volume;
		audio_aic23_write(LEFT_CHANNEL_VOLUME_ADDR, aic23_local.volume_reg);
		audio_aic23_write(RIGHT_CHANNEL_VOLUME_ADDR, aic23_local.volume_reg);
//		audio_aic23_write(LEFT_CHANNEL_VOLUME_ADDR, 0);
//		audio_aic23_write(RIGHT_CHANNEL_VOLUME_ADDR, 0);
		DPRINTK("Set Volume %d, reg=%x\n",val,aic23_local.volume_reg);
		break;

	case SET_LINE:
	case SET_MIC:
		/* Ignore separate left/right channel for now,
			even the codec does support it. */
		val &= 0xff;

		if (val < 0 || val > 100) {
			printk(KERN_ERR "Trying a bad volume value(%d)!\n",val);
			return -EPERM;
		}
		// Convert 0 -> 100 volume to 0x0 (LIV_MIN) -> 0x1f (LIV_MAX) 
		// volume range
		volume = ((val * INPUT_VOLUME_RANGE) / 100) + INPUT_VOLUME_MIN;

		// R/LIV[4:0] 11111 (+12dB) to 00000 (-34.5dB) in 1.5dB steps,
		// default 10111 (0dB)
		aic23_local.input_volume_reg &= ~INPUT_VOLUME_MASK;
		aic23_local.input_volume_reg |= volume;
		audio_aic23_write(LEFT_LINE_VOLUME_ADDR, aic23_local.input_volume_reg);
		audio_aic23_write(RIGHT_LINE_VOLUME_ADDR, aic23_local.input_volume_reg);
		DPRINTK("Set input Volume %d, reg=%x\n",val,aic23_local.input_volume_reg);
		break;
	case SET_RECSRC:
		/* Ignore separate left/right channel for now,
	   	   even the codec does support it. */
		val &= 0xff;

		if (hweight32(val) > 1)
			val &= ~aic23_local.recsrc;

		if (val == SOUND_MASK_MIC) {
			/* enable the mic input*/
			DPRINTK("Enabling mic\n");
			audio_aic23_write(ANALOG_AUDIO_CONTROL_ADDR, DAC_SELECTED | INSEL_MIC);
		}
		else if (val == SOUND_MASK_LINE) {
			/* enable ADC's, enable line iput */
			DPRINTK(" Enabling line in\n");
			audio_aic23_write(ANALOG_AUDIO_CONTROL_ADDR, DAC_SELECTED);
		}
		else {
			/* do nothing */
		}
		aic23_local.recsrc = val;
		break;


	case SET_MICBIAS:
		break;

	case SET_BASS:
		break;

	case SET_TREBLE:
		break;
	}
	return 0;
}

static int mixer_open(struct inode *inode, struct file *file)
{
	/* Any mixer specific initialization */
	return 0;
}

static int mixer_release(struct inode *inode, struct file *file)
{
	/* Any mixer specific Un-initialization */
	return 0;
}

static int
mixer_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
	int val;
	int ret = 0;
	int nr = _IOC_NR(cmd);

	/*
	 * We only accept mixer (type 'M') ioctls.
	 */
	if (_IOC_TYPE(cmd) != 'M')
		return -EINVAL;

	DPRINTK(" 0x%08x\n", cmd);

	if (cmd == SOUND_MIXER_INFO) {
		struct mixer_info mi;

		strncpy(mi.id, "AIC23", sizeof(mi.id));
		strncpy(mi.name, "TI AIC23", sizeof(mi.name));
		mi.modify_counter = aic23_local.mod_cnt;
		return copy_to_user((void *)arg, &mi, sizeof(mi));
	}

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		ret = get_user(val, (int *)arg);
		if (ret)
			goto out;

	
		switch (nr) {
		case SOUND_MIXER_VOLUME:
			aic23_local.mod_cnt++;
			ret = aic23_update(SET_VOLUME, val);
			if (!ret) aic23_local.volume = val;
			break;

		case SOUND_MIXER_LINE:
			aic23_local.mod_cnt++;
			ret = aic23_update(SET_LINE, val);
			if (!ret) aic23_local.line = val;
			break;

		case SOUND_MIXER_MIC:
			aic23_local.mod_cnt++;
			ret = aic23_update(SET_LINE, val);
			if (!ret) aic23_local.mic = val;
			break;

		case SOUND_MIXER_RECSRC:
			if ((val & SOUND_MASK_LINE) ||
			    (val & SOUND_MASK_MIC)) {
				if (aic23_local.recsrc != val) {
					aic23_local.mod_cnt++;
					aic23_update(SET_RECSRC, val);
				}
			}
			else {
				ret = -EINVAL;
			}
			break;

		default:
			ret = -EINVAL;
		}
	}

	if (ret == 0 && _IOC_DIR(cmd) & _IOC_READ) {
		ret = 0;

		switch (nr) {
		case SOUND_MIXER_VOLUME:
			val = aic23_local.volume;
			break;
		case SOUND_MIXER_LINE:
			val = aic23_local.line;
			break;
		case SOUND_MIXER_MIC:
			val = aic23_local.mic;
			break;
		case SOUND_MIXER_RECSRC:
			val = aic23_local.recsrc;
			break;
		case SOUND_MIXER_RECMASK:
			val = REC_MASK;
			break;
		case SOUND_MIXER_DEVMASK:
			val = DEV_MASK;
			break;
		case SOUND_MIXER_CAPS:
			val = 0;
			break;
		case SOUND_MIXER_STEREODEVS:
			val = 0;
			break;
		case SOUND_MIXER_MICBIAS:
			val = aic23_local.micbias;
			break;
		default:
			val = 0;
			ret = -EINVAL;
			break;
		}

		if (ret == 0)
			ret = put_user(val, (int *)arg);
	}
out:
	return ret;

}

int davinci_set_samplerate(long sample_rate)
{
	u8 count = 0;
	u16 data = 0;
	/* wait for any frame to complete */
	udelay(125);

	/* Search for the right sample rate */
	while ((reg_info[count].sample_rate != sample_rate) &&
		(count < NUMBER_SAMPLE_RATES_SUPPORTED)) {
		count++;
	}
	if (count == NUMBER_SAMPLE_RATES_SUPPORTED) {
		printk(KERN_ERR "Invalid Sample Rate %d requested\n",
			(int)sample_rate);
		return -EPERM;
	}

	data = (reg_info[count].divider << CLKIN_SHIFT) | 
		(reg_info[count].control << BOSR_SHIFT) | USB_CLK_ON;

	audio_aic23_write(SAMPLE_RATE_CONTROL_ADDR, data);
	DPRINTK("samplerate = %ld reg=%x\n", sample_rate,data);

	audio_samplerate = sample_rate;

#ifndef AIC23_MASTER
	{
		int clkgdv = 0;
		unsigned long clkval = 0;
		struct clk *mbspclk;
		/* 
			Set Sample Rate at McBSP

			Formula : 
			Codec System Clock = Input clock to McBSP;
			clkgdv = ((Codec System Clock / (SampleRate * BitsPerSample * 2)) - 1);

			FWID = BitsPerSample - 1;
			FPER = (BitsPerSample * 2) - 1;
		*/	
		mbspclk = davinci_mcbsp_get_clock();
		if (mbspclk == NULL) {
			DPRINTK(" Failed to get internal clock to MCBSP");
			return -EPERM;
		}
		clkval = clk_get_rate(mbspclk);
		if (clkval) {
			DPRINTK("mcbsp_clk = %ld\n", clkval);
		} else {
			DPRINTK(" Failed to get the MCBSP clock\n");
			return -EPERM;
		}

		clkgdv = (clkval/(sample_rate * DEFAULT_BITPERSAMPLE * 2)) - 1;
		DPRINTK("clkgdv = %d\n", clkgdv);

		if (clkgdv > 255 || clkgdv < 0) {
			/* For requested sampling rate, the input clock to MCBSP cant be devided
			   down to get the in range clock devider value for 16 bits sample */
			DPRINTK("Invalid Sample Rate %d requested\n",(int)sample_rate);
			return -EPERM;
		}
		initial_config.srgr1 = (FWID(DEFAULT_BITPERSAMPLE - 1) | CLKGDV(clkgdv));
		initial_config.srgr2 = (CLKSM | FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1));
		davinci_mcbsp_stop(AUDIO_MCBSP);
		davinci_mcbsp_config(AUDIO_MCBSP, &initial_config);
	}
#endif /* AIC23_MASTER */

	return 0;
}

static void davinci_aic23_initialize(void *dummy)
{
	DPRINTK("entry\n");

	/* initialize with default sample rate */
	audio_samplerate = AUDIO_RATE_DEFAULT;

	if (davinci_mcbsp_request(AUDIO_MCBSP) < 0) {
		DPRINTK("MCBSP request failed\n");
//		return;
	}

	/* if configured, then stop mcbsp */
	davinci_mcbsp_stop(AUDIO_MCBSP);
	aic23_configure();

	davinci_mcbsp_config(AUDIO_MCBSP, &initial_config);

#ifdef TONE_GEN
	davinci_mcbsp_start(AUDIO_MCBSP);
	toneGen();
#endif /* TONE_GEN */

	DPRINTK("exit\n");
}

static void davinci_aic23_shutdown(void *dummy)
{
	/*
		Turn off codec after it is done.
		Can't do it immediately, since it may still have
		buffered data.

		Wait 20ms (arbitrary value) and then turn it off.
	*/

	DPRINTK("entry\n");
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(2);

	davinci_mcbsp_stop(AUDIO_MCBSP);
	davinci_mcbsp_free(AUDIO_MCBSP);

	audio_aic23_write(RESET_CONTROL_ADDR, 0);
	audio_aic23_write(POWER_DOWN_CONTROL_ADDR, 0xff);
	DPRINTK("exit\n");
}

static inline void aic23_configure()
{
	/* Reset codec */
	audio_aic23_write(RESET_CONTROL_ADDR, 0);

	/* Initialize the AIC23 internal state */

	/* Left/Right line input volume control */
	aic23_local.line = DEFAULT_INPUT_VOLUME;
	aic23_local.mic = DEFAULT_INPUT_VOLUME;
	aic23_update(SET_LINE, DEFAULT_INPUT_VOLUME);

	/* Left/Right headphone channel volume control */
	/* Zero-cross detect on */
	aic23_local.volume_reg = LZC_ON;
	aic23_update(SET_VOLUME, aic23_local.volume);

	/* Analog audio path control, DAC selected, delete INSEL_MIC for line in */
	audio_aic23_write(ANALOG_AUDIO_CONTROL_ADDR, DAC_SELECTED | INSEL_MIC);	// | MICB_20DB | BYPASS_ON

	/* Digital audio path control, de-emphasis control 44.1kHz */
	audio_aic23_write(DIGITAL_AUDIO_CONTROL_ADDR, DEEMP_44K);

	/* Power control, everything is on */
	audio_aic23_write(POWER_DOWN_CONTROL_ADDR, 0);

	/* Digital audio interface, master/slave mode, I2S, 16 bit */
#ifdef AIC23_MASTER
	audio_aic23_write(DIGITAL_AUDIO_FORMAT_ADDR, MS_MASTER | IWL_16 | FOR_DSP);
#else
	audio_aic23_write(DIGITAL_AUDIO_FORMAT_ADDR, IWL_16 | FOR_DSP);
#endif /* AIC23_MASTER */

	/* Enable digital interface */
	audio_aic23_write(DIGITAL_INTERFACE_ACT_ADDR, ACT_ON);

	/* clock configuration */
	davinci_set_samplerate(audio_samplerate);
}

static int
davinci_aic23_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
	long val;
	int ret = 0;

	DPRINTK(" 0x%08x\n", cmd);

	/*
	 * These are platform dependent ioctls which are not handled by the
	 * generic davinci-audio module.
	 */
	switch (cmd) {
	case SNDCTL_DSP_STEREO:
		ret = get_user(val, (int *)arg);
		if (ret)
			return ret;
		/* the AIC23 is stereo only */
		ret = (val == 0) ? -EINVAL : 1;
		return put_user(ret, (int *)arg);

	case SNDCTL_DSP_CHANNELS:
	case SOUND_PCM_READ_CHANNELS:
		/* the AIC23 is stereo only */
		return put_user(2, (long *)arg);

	case SNDCTL_DSP_SPEED:
		ret = get_user(val, (long *)arg);
		if (ret)
			break;
		ret = davinci_set_samplerate(val);
		if (ret)
			break;
		/* fall through */

	case SOUND_PCM_READ_RATE:
		return put_user(audio_samplerate, (long *)arg);

	case SOUND_PCM_READ_BITS:
	case SNDCTL_DSP_SETFMT:
	case SNDCTL_DSP_GETFMTS:
		/* we can do 16-bit only */
		return put_user(AFMT_S16_LE, (long *)arg);

	default:
		/* Maybe this is meant for the mixer (As per OSS Docs) */
		return mixer_ioctl(inode, file, cmd, arg);
	}

	return ret;
}

static int davinci_aic23_probe(void)
{
	/* Get the fops from audio oss driver */
	if (!(davinci_audio_fops = audio_get_fops())) {
		printk(KERN_ERR "Unable to get the file operations for AIC23 OSS driver\n");
		audio_unregister_codec(&aic23_state);
		return -EPERM;
	}

	aic23_local.volume = DEFAULT_OUTPUT_VOLUME;
	aic23_local.recsrc = SOUND_MASK_MIC;	/* either of SOUND_MASK_LINE/SOUND_MASK_MIC */
	aic23_local.micbias = 1;
	aic23_local.mod_cnt = 0;

	/* register devices */
	audio_dev_id = register_sound_dsp(davinci_audio_fops, -1);
	mixer_dev_id = register_sound_mixer(&davinci_mixer_fops, -1);

#ifdef CONFIG_PROC_FS
	create_proc_read_entry(PROC_START_FILE, 0 /* default mode */ ,
				NULL /* parent dir */ ,
				codec_start, NULL /* client data */ );

	create_proc_read_entry(PROC_STOP_FILE, 0 /* default mode */ ,
				NULL /* parent dir */ ,
				codec_stop, NULL /* client data */ );
#endif

	/* Announcement Time */
	printk(KERN_INFO PLATFORM_NAME " " CODEC_NAME
		" audio support initialized\n");
	return 0;
}

#ifdef MODULE
static void __exit davinci_aic23_remove(void)
{
	/* Un-Register the codec with the audio driver */
	unregister_sound_dsp(audio_dev_id);
	unregister_sound_mixer(mixer_dev_id);

#ifdef CONFIG_PROC_FS
	remove_proc_entry(PROC_START_FILE, NULL);
	remove_proc_entry(PROC_STOP_FILE, NULL);
#endif
}
#endif /* MODULE */

static int davinci_aic23_suspend(void)
{
	/* Empty for the moment */
	return 0;
}

static int davinci_aic23_resume(void)
{
	/* Empty for the moment */
	return 0;
}

static int __init audio_aic23_init(void)
{

	int err = 0;

	/* register the codec with the audio driver */
	if ((err = audio_register_codec(&aic23_state))) {
		printk(KERN_ERR "Failed to register AIC23 driver with Audio OSS Driver\n");
	} else {
		spi_tlv320aic23_init();
	}
	return err;
}

static void __exit audio_aic23_exit(void)
{
	void *foo = NULL;
	davinci_aic23_shutdown(foo);
	(void)audio_unregister_codec(&aic23_state);
	return;
}

#ifdef CONFIG_PROC_FS
static int codec_start(char *buf, char **start, off_t offset, int count,
			int *eof, void *data)
{
	void *foo = NULL;
	davinci_aic23_initialize(foo);

	printk("AIC23 codec initialization done.\n");
	return 0;
}
static int codec_stop(char *buf, char **start, off_t offset, int count,
			int *eof, void *data)
{
	void *foo = NULL;
	davinci_aic23_shutdown(foo);
	printk("AIC23 codec shutdown.\n");
	return 0;
}
#endif /* CONFIG_PROC_FS */

module_init(audio_aic23_init);
module_exit(audio_aic23_exit);

MODULE_AUTHOR("Boundary Devices");
MODULE_DESCRIPTION("Glue audio driver for the TI AIC23 codec.");
MODULE_LICENSE("GPL");

#ifdef TONE_GEN
/* Generates a shrill tone */
u16 tone[] = {
	0x0ce4, 0x0ce4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000,
	0x0CE4, 0x0CE4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000,
	0x0CE4, 0x0CE4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000
};

void toneGen(void)
{
	int count = 0;
	int ret = 0;
	printk(KERN_INFO "TONE GEN TEST :");

//#define MAX_CNT 2000
#define MAX_CNT 1
	for (count = 0; count < MAX_CNT; count++) {
		do {
			ret = davinci_mcbsp_xmit_buffer(AUDIO_MCBSP, ((int)(&tone[0])) - 0x20000000,sizeof(tone));
		} while (ret == -EAGAIN);
		if (ret != 0) {
			printk(KERN_INFO "ERROR=%d\n", ret);
			return;
		}
	}
	printk(KERN_INFO "SUCCESS\n");
}
#endif /* TONE_GEN */
