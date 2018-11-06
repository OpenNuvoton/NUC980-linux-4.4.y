/*
 * Copyright (c) 2018 Nuvoton technology corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;version 2 of the License.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/soc-dai.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/of.h>

#include <mach/mfp.h>
#include <mach/map.h>
#include <mach/regs-clock.h>

#include "nuc980-audio.h"

static DEFINE_MUTEX(i2s_mutex);
struct nuc980_audio *nuc980_i2s_data;

static int nuc980_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct nuc980_audio *nuc980_audio = nuc980_i2s_data;
	unsigned long val = AUDIO_READ(nuc980_audio->mmio + ACTL_CON);

	switch (params_width(params)) {
		case 8:
			val = (val & ~0x300) | BITS_SELECT_8;
			break;

		case 16:
			val = (val & ~0x300) | BITS_SELECT_16;
			break;

		case 24:
			val = (val & ~0x300) | BITS_SELECT_24;
			break;

		default:
			return -EINVAL;
	}
	AUDIO_WRITE(nuc980_audio->mmio + ACTL_CON, val);
	return 0;
}

static int nuc980_i2s_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct nuc980_audio *nuc980_audio = nuc980_i2s_data;
	unsigned long val = AUDIO_READ(nuc980_audio->mmio + ACTL_I2SCON);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_MSB:
			val |= FORMAT_MSB;
			break;
		case SND_SOC_DAIFMT_I2S:
			val &= ~FORMAT_MSB;
			break;
		default:
			return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBM_CFM:
			val &= ~I2S_SLAVE;
			break;
		case SND_SOC_DAIFMT_CBS_CFS:
			val |= I2S_SLAVE;
			break;
		default:
			return -EINVAL;
	}

	AUDIO_WRITE(nuc980_audio->mmio + ACTL_I2SCON, val);

	return 0;
}

static int nuc980_i2s_set_sysclk(struct snd_soc_dai *cpu_dai, int clk_id, unsigned int freq, int dir)
{
	unsigned int val;
	struct nuc980_audio *nuc980_audio = nuc980_i2s_data;
	struct clk *clkmux, *clkapll, *clkaudio;
	int ret;
	unsigned int mclkdiv, bclkdiv, mclk;

	clkmux = clk_get(NULL, "audio_eclk_mux");
		if (IS_ERR(clkmux)) {
			printk(KERN_ERR "nuc980-audio:failed to get audio clock source\n");
			ret = PTR_ERR(clkmux);
			return ret;
		}
		clkapll = clk_get(NULL, "apll");
		if (IS_ERR(clkapll)) {
			printk(KERN_ERR "nuc980-audio:failed to get audio clock source\n");
			ret = PTR_ERR(clkapll);
			return ret;
		}

		clkaudio = clk_get(NULL, "audio_eclk");
		if (IS_ERR(clkaudio)) {
			printk(KERN_ERR "nuc980-audio:failed to get audio clock source\n");
			ret = PTR_ERR(clkaudio);
			return ret;
		}

	if (clk_id == NUC980_AUDIO_SAMPLECLK) {
		val = AUDIO_READ(nuc980_audio->mmio + ACTL_I2SCON);

		mclk = (freq*256);      // 256fs
		mclkdiv = clk_get_rate(clkaudio) / mclk;
		val &= ~0x000F0000;
		val |= (mclkdiv-1) << 16;

		bclkdiv = mclk / (freq * cpu_dai->sample_bits * cpu_dai->channels);
		bclkdiv = bclkdiv/2 - 1;
		val &= ~0xf0;
		val |= (bclkdiv << 5);

		AUDIO_WRITE(nuc980_audio->mmio + ACTL_I2SCON, val);
	}

	if (clk_id == NUC980_AUDIO_CLKDIV) {
		//use APLL to generate 12.288MHz ,16.934MHz or 11.285Mhz for I2S
		//input source clock is XIN=12Mhz

		clk_set_parent(clkmux, clkapll);

		if ((freq % 8000 == 0) && (freq != 32000)) {
			//12.288MHz ==> APLL=98.4MHz / 8 = 12.3MHz
			clk_set_rate(clkapll, 98400000);
			clk_set_rate(clkaudio, 12300000);
		} else if(freq == 44100) {
			//16.934MHz ==> APLL=169.5MHz / 15 = 11.30MHz
			clk_set_rate(clkapll, 169500000);
			clk_set_rate(clkaudio, 11300000);
		} else {
			//16.934MHz ==> APLL=169.5MHz / 10 = 16.95MHz
			clk_set_rate(clkapll, 169500000);
			clk_set_rate(clkaudio, 16950000);
		}
	}

	return 0;
}

static int nuc980_i2s_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	struct nuc980_audio *nuc980_audio = nuc980_i2s_data;
	int ret = 0;
	unsigned long val, con;

	con = AUDIO_READ(nuc980_audio->mmio + ACTL_CON);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
			val = AUDIO_READ(nuc980_audio->mmio + ACTL_RESET);
			con |= I2S_EN;
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				con |= P_DMA_IRQ_EN;
				AUDIO_WRITE(nuc980_audio->mmio + ACTL_PSR, P_DMA_RIA_IRQ);

				val |= AUDIO_PLAY;
			} else {
				con |= R_DMA_IRQ_EN;
				AUDIO_WRITE(nuc980_audio->mmio + ACTL_RSR, R_DMA_RIA_IRQ);

				val |= AUDIO_RECORD;
			}
			AUDIO_WRITE(nuc980_audio->mmio + ACTL_RESET, val);
			AUDIO_WRITE(nuc980_audio->mmio + ACTL_CON, con);

			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
			val = AUDIO_READ(nuc980_audio->mmio + ACTL_RESET);
			con &= ~I2S_EN;
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				con &= ~P_DMA_IRQ_EN;
				AUDIO_WRITE(nuc980_audio->mmio + ACTL_PSR, RESET_PRSR);
				val &= ~AUDIO_PLAY;
			} else {
				con &= ~R_DMA_IRQ_EN;
				AUDIO_WRITE(nuc980_audio->mmio + ACTL_RSR, RESET_PRSR);
				val &= ~AUDIO_RECORD;
			}

			AUDIO_WRITE(nuc980_audio->mmio + ACTL_RESET, val);
			AUDIO_WRITE(nuc980_audio->mmio + ACTL_CON, con);

			break;
		default:
			ret = -EINVAL;
	}

	return ret;
}

static int nuc980_i2s_probe(struct snd_soc_dai *dai)
{
	struct nuc980_audio *nuc980_audio = nuc980_i2s_data;
	unsigned long val;

	mutex_lock(&i2s_mutex);

	clk_prepare(clk_get(NULL, "audio_hclk"));
	clk_enable(clk_get(NULL, "audio_hclk"));

	/* enable unit clock */
	clk_prepare(nuc980_audio->clk);
	clk_enable(nuc980_audio->clk);

	/* Select I2S pins */
	val = AUDIO_READ(nuc980_audio->mmio + ACTL_CON);
	val = (val & ~0x300) | BITS_SELECT_16;          //set default data bit to 16-bit
	AUDIO_WRITE(nuc980_audio->mmio + ACTL_CON, val);

	mutex_unlock(&i2s_mutex);

	return 0;
}

static int nuc980_i2s_remove(struct snd_soc_dai *dai)
{
	struct nuc980_audio *nuc980_audio = nuc980_i2s_data;
	clk_disable(nuc980_audio->clk);

	return 0;
}

static struct snd_soc_dai_ops nuc980_i2s_dai_ops = {
	.trigger    = nuc980_i2s_trigger,
	.hw_params  = nuc980_i2s_hw_params,
	.set_fmt    = nuc980_i2s_set_fmt,
	.set_sysclk = nuc980_i2s_set_sysclk,
};

struct snd_soc_dai_driver nuc980_i2s_dai = {
	.probe          = nuc980_i2s_probe,
	.remove         = nuc980_i2s_remove,
	.playback = {
		.rates      = SNDRV_PCM_RATE_8000_48000,
		.formats    = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min   = 1,
		.channels_max   = 2,
	},
	.capture = {
		.rates      = SNDRV_PCM_RATE_8000_48000,
		.formats    = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min   = 1,
		.channels_max   = 2,
	},
	.ops = &nuc980_i2s_dai_ops,
};

static const struct snd_soc_component_driver nuc980_i2s_component = {
	.name       = "nuc980-i2s",
};

static int nuc980_i2s_drvprobe(struct platform_device *pdev)
{
	struct nuc980_audio *nuc980_audio;
	struct pinctrl *pinctrl;
	int ret;

	if (nuc980_i2s_data)
		return -EBUSY;

	nuc980_audio = kzalloc(sizeof(struct nuc980_audio), GFP_KERNEL);
	if (!nuc980_audio)
		return -ENOMEM;

	spin_lock_init(&nuc980_audio->lock);
	spin_lock_init(&nuc980_audio->irqlock);

	nuc980_audio->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!nuc980_audio->res) {
		ret = -ENODEV;
		goto out0;
	}

	if (!request_mem_region(nuc980_audio->res->start, resource_size(nuc980_audio->res), pdev->name)) {
		ret = -EBUSY;
		goto out0;
	}

	nuc980_audio->mmio = ioremap(nuc980_audio->res->start, resource_size(nuc980_audio->res));
	if (!nuc980_audio->mmio) {
		ret = -ENOMEM;
		goto out1;
	}

	nuc980_audio->clk = clk_get(NULL, "audio_eclk");
	if (IS_ERR(nuc980_audio->clk)) {
		ret = PTR_ERR(nuc980_audio->clk);
		goto out2;
	}

	nuc980_audio->irq_num = platform_get_irq(pdev, 0);
	if (!nuc980_audio->irq_num) {
		ret = -EBUSY;
		goto out3;
	}

	ret = nuc980_dma_create(nuc980_audio);
	if (ret != 0)
		return ret;

	nuc980_i2s_data = nuc980_audio;

	ret = snd_soc_register_component(&pdev->dev, &nuc980_i2s_component, &nuc980_i2s_dai, 1);
	if (ret)
		goto out3;

#if defined(CONFIG_OF)
	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		return PTR_ERR(pinctrl);
	}
#else

	/* initial I2S pin */
#if defined (CONFIG_NUC980_I2S_PA)

	pinctrl = devm_pinctrl_get_select(&pdev->dev, "i2s-PA");

#elif defined (CONFIG_NUC980_I2S_PB)

	pinctrl = devm_pinctrl_get_select(&pdev->dev, "i2s-PB");

#endif

	if (IS_ERR(pinctrl))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		ret = PTR_ERR(pinctrl);
	}
#endif

	return 0;

out3:
	clk_put(nuc980_audio->clk);
out2:
	iounmap(nuc980_audio->mmio);
out1:
	release_mem_region(nuc980_audio->res->start, resource_size(nuc980_audio->res));
out0:
	kfree(nuc980_audio);

	return ret;
}

#if defined(CONFIG_OF)
static const struct of_device_id nuc980_audio_i2s_of_match[] = {
	{   .compatible = "nuvoton,nuc980-audio-i2s"    },
	{   },
};
MODULE_DEVICE_TABLE(of, nuc980_audio_i2s_of_match);
#endif

static int nuc980_i2s_drvremove(struct platform_device *pdev)
{
	nuc980_dma_destroy(nuc980_i2s_data);
	snd_soc_unregister_component(&pdev->dev);

	clk_put(nuc980_i2s_data->clk);
	iounmap(nuc980_i2s_data->mmio);
	release_mem_region(nuc980_i2s_data->res->start, resource_size(nuc980_i2s_data->res));

	kfree(nuc980_i2s_data);
	nuc980_i2s_data = NULL;

	return 0;
}

static struct platform_driver nuc980_i2s_driver = {
	.driver = {
		.name   = "nuc980-audio-i2s",
		.owner  = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(nuc980_audio_i2s_of_match),
#endif
	},
	.probe      = nuc980_i2s_drvprobe,
	.remove     = nuc980_i2s_drvremove,
};

module_platform_driver(nuc980_i2s_driver);

MODULE_DESCRIPTION("NUC980 IIS SoC driver!");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:nuc980-i2s");
