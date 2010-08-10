#include <linux/module.h>
#include <sound/soc.h>
#include <asm/mach-types.h>
#include "../codecs/sgtl5000.h"
#include "pxa2xx-i2s.h"
#include "pxa2xx-pcm.h"

static int hydrogen_fmi_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret;
	int rate = params_rate(params);

//	pr_info("%s\n", __func__);
	/* CPU should be clock master */
	ret = snd_soc_dai_set_fmt(cpu_dai,  SND_SOC_DAIFMT_I2S
				  | SND_SOC_DAIFMT_NB_NF
				  | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;
	/* set the I2S system clock as output */
	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA2XX_I2S_SYSCLK, rate * 256,
	                SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S
				  | SND_SOC_DAIFMT_NB_NF
				  | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;


	ret = snd_soc_dai_set_sysclk(codec_dai, SGTL5000_SYSCLK, rate * 256, 0);
	if (ret < 0)
		return ret;
	ret = snd_soc_dai_set_sysclk(codec_dai, SGTL5000_LRCLK, rate, 0);
	return ret;
}
static int hydrogen_fmi_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static void hydrogen_fmi_shutdown(struct snd_pcm_substream *substream)
{
//	pr_info("%s\n", __func__);
}

static struct snd_soc_ops hydrogen_fmi_ops = {
	.startup = hydrogen_fmi_startup,
	.hw_params = hydrogen_fmi_hw_params,
	.shutdown = hydrogen_fmi_shutdown,
};

static struct snd_soc_dai_link hydrogen_fmi_dai = {
	.name = "sgtl5000",
	.stream_name = "sgtl5000",
	.cpu_dai = &pxa_i2s_dai,
	.codec_dai = &sgtl5000_dai,
	.ops = &hydrogen_fmi_ops,
};

static struct snd_soc_card snd_soc_hydrogen_fmi = {
	.name = "hydrogen_fmi",
	.platform = &pxa2xx_soc_platform,
	.dai_link = &hydrogen_fmi_dai,
	.num_links = 1,
};

static struct snd_soc_device hydrogen_fmi_snd_devdata = {
	.card = &snd_soc_hydrogen_fmi,
	.codec_dev = &soc_codec_dev_sgtl5000,
};

static struct platform_device *hydrogen_fmi_snd_device;

static int __init hydrogen_fmi_asoc_init(void)
{
	int ret;
	pr_info("%s\n", __func__);

	if (0) if (!machine_is_scanpass()) {
		pr_err("Failed machine_is_scanpass\n");
		return -ENODEV;
	}
	hydrogen_fmi_snd_device = platform_device_alloc("soc-audio", -1);
	if (!hydrogen_fmi_snd_device)
		return -ENOMEM;

	platform_set_drvdata(hydrogen_fmi_snd_device, &hydrogen_fmi_snd_devdata);
	hydrogen_fmi_snd_devdata.dev = &hydrogen_fmi_snd_device->dev;
	ret = platform_device_add(hydrogen_fmi_snd_device);
	if (ret)
		platform_device_put(hydrogen_fmi_snd_device);

	return ret;
}
module_init(hydrogen_fmi_asoc_init);

static void __exit hydrogen_fmi_asoc_exit(void)
{
	platform_device_unregister(hydrogen_fmi_snd_device);
}
module_exit(hydrogen_fmi_asoc_exit);

MODULE_AUTHOR("Troy Kisky");
MODULE_DESCRIPTION("ALSA SoC Hydrogen_FMI");
MODULE_LICENSE("GPL");
