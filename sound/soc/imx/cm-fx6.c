/*
 * ASoC driver for CM-FX6 module
 *
 * Copyright (C) 2013 - CompuLab, Ltd.
 * Author: Andrey Gelman <andrey.gelman@compulab.co.il>
 *
 * Based on imx-wm8962.c by freescale
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/soc-dapm.h>
#include <mach/audmux.h>

#include "imx-ssi.h"
#include "../codecs/wm8731.h"



static int __devinit imx_wm8731_probe(struct platform_device *pdev);
static int __devinit imx_wm8731_remove(struct platform_device *pdev);
static int imx_wm8731_init(struct snd_soc_pcm_runtime *rtd);
static int imx_hifi_startup_slv_mode(struct snd_pcm_substream *substream);
static int imx_hifi_hw_params_slv_mode(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params);
static int imx_hifi_startup_mst_mode(struct snd_pcm_substream *substream);
static int imx_hifi_hw_params_mst_mode(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params);
static void imx_hifi_shutdown(struct snd_pcm_substream *substream);



static struct platform_device *imx_snd_device;

struct imx_priv {
	struct platform_device *pdev;
};
static struct imx_priv card_priv;

static struct snd_soc_ops imx_hifi_ops = {
	.startup		= imx_hifi_startup_slv_mode,
	.shutdown		= imx_hifi_shutdown,
	.hw_params		= imx_hifi_hw_params_slv_mode,
};

static struct snd_soc_dai_link imx_dai[] = {
	{
		.name		= "HiFi",
		.stream_name	= "HiFi",
		.codec_dai_name	= "wm8731-hifi",
		.codec_name	= "wm8731.2-001a",
		.cpu_dai_name	= "imx-ssi.1",
		.platform_name	= "imx-pcm-audio.1",
		.init		= imx_wm8731_init,
		.ops		= &imx_hifi_ops,
	},
};

static struct snd_soc_card snd_soc_card_imx = {
	.name		= "wm8731-audio",
	.owner		= THIS_MODULE,
	.dai_link	= imx_dai,
	.num_links	= ARRAY_SIZE(imx_dai),
};

static struct platform_driver imx_wm8731_driver = {
	.probe = imx_wm8731_probe,
	.remove = imx_wm8731_remove,
	.driver = {
		.name = "imx-wm8731",
		.owner = THIS_MODULE,
	},
};

/* imx card dapm widgets */
static const struct snd_soc_dapm_widget imx_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack",	NULL),
	SND_SOC_DAPM_SPK("Ext Spk",		NULL),
	SND_SOC_DAPM_LINE("Line Jack",		NULL),
	SND_SOC_DAPM_MIC("Mic Jack",		NULL),
};

/* imx machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	{ "Headphone Jack",	NULL,	"LHPOUT" },
	{ "Headphone Jack",	NULL,	"RHPOUT" },

	{ "Ext Spk",		NULL,	"LOUT" },
	{ "Ext Spk",		NULL,	"ROUT" },

	{ "LLINEIN",		NULL,	"Line Jack" },
	{ "RLINEIN",		NULL,	"Line Jack" },

	{ "MICIN",		NULL,	"Mic Bias" },
	{ "Mic Bias",		NULL,	"Mic Jack"},
};

static int imx_hifi_startup_slv_mode(struct snd_pcm_substream *substream)
{
	/*
	 * As SSI's sys clock rate depends on sampling rate,
	 * the clock enabling code is moved to imx_hifi_hw_params().
	 */

	return 0;
}

static int imx_hifi_hw_params_slv_mode(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct mxc_audio_platform_data *plat = card_priv.pdev->dev.platform_data;
	u32 dai_format;
	unsigned int channels;
	unsigned int tx_mask, rx_mask;
	unsigned int sampling_rate;
	unsigned int div_2, div_psr, div_pm;
	int ret;


	sampling_rate = params_rate(params);
	channels = params_channels(params);
	pr_debug("%s:%s  sampling rate = %u  channels = %u \n", __FUNCTION__,
		 (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "Playback" : "Capture"),
		 sampling_rate, channels);

	/* set CPU DAI configuration */
	switch (sampling_rate)
	{
	case 8000:
	case 32000:
	case 48000:
	case 96000:
		plat->sysclk = 12288000;
		break;

	case 44100:
	case 88200:
		plat->sysclk = 11289600;
		break;

	default:
		return -EINVAL;
	}

	// configure plat->sysclk ... by calling clock_enable()
	plat->clock_enable(1);

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
		SND_SOC_DAIFMT_CBS_CFS;

	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;


	/* set i.MX active slot mask */
	/* S[TR]CCR:DC */
	tx_mask = ~((1 << channels) - 1);
	rx_mask = tx_mask;
	snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask, rx_mask, 2, 32);


	ret = snd_soc_dai_set_sysclk(cpu_dai,
				     IMX_SSP_SYS_CLK,
				     0/*internally ignored*/,
				     SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		pr_err("Failed to set SSI clock: %d \n", ret);
		return ret;
	}


	/*
	 * SSI sysclk divider:
	 * div_2:	/1 or /2
	 * div_psr:	/1 or /8
	 * div_pm:	/1 .. /256
	 */
	div_2	= 0;
	div_psr	= 0;
	switch (sampling_rate)
	{
	case 8000:
		// 1x1x12
		div_pm	= 11;
		break;
	case 32000:
		// 1x1x3
		div_pm	= 2;
		break;
	case 48000:
		// 1x1x2
		div_pm	= 1;
		break;
	case 96000:
		// 1x1x1
		div_pm	= 0;
		break;
	case 44100:
		// 1x1x2
		div_pm	= 1;
		break;
	case 88200:
		// 1x1x1
		div_pm	= 0;
		break;
	default:
		return -EINVAL;
	}

	/* sync mode: a single clock controls both playback and capture */
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_2, (div_2 ? SSI_STCCR_DIV2 : 0));
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_PSR, (div_psr ? SSI_STCCR_PSR : 0));
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_PM, div_pm);

	/* set codec DAI configuration */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBS_CFS;

	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai,
				     WM8731_SYSCLK_MCLK,
				     plat->sysclk,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("Failed to set codec master clock to %u: %d \n",
		       plat->sysclk, ret);
		return ret;
	}

	return 0;
}


static int imx_hifi_startup_mst_mode(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct mxc_audio_platform_data *plat = card_priv.pdev->dev.platform_data;

	if (!codec_dai->active)
		plat->clock_enable(1);

	return 0;
}

static int imx_hifi_hw_params_mst_mode(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct mxc_audio_platform_data *plat = card_priv.pdev->dev.platform_data;
	u32 dai_format;
	unsigned int channels;
	unsigned int tx_mask, rx_mask;
	unsigned int sampling_rate;
	int ret;


	sampling_rate = params_rate(params);
	channels = params_channels(params);
	pr_debug("%s:%s  sampling rate = %u  channels = %u \n", __FUNCTION__,
		 (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "Playback" : "Capture"),
		 sampling_rate, channels);

	/* set cpu DAI configuration */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
		SND_SOC_DAIFMT_CBM_CFM;

	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set i.MX active slot mask */
	/* S[TR]CCR:DC */
	tx_mask = ~((1 << channels) - 1);
	rx_mask = tx_mask;
	snd_soc_dai_set_tdm_slot(cpu_dai, tx_mask, rx_mask, 2, 32);

	ret = snd_soc_dai_set_sysclk(cpu_dai,
				     IMX_SSP_SYS_CLK,
				     0/*internally ignored*/,
				     SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		pr_err("Failed to set SSI clock: %d \n", ret);
		return ret;
	}


	/* set codec DAI configuration */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM;

	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai,
				     WM8731_SYSCLK_MCLK,
				     plat->sysclk,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("Failed to set codec master clock to %u: %d \n",
		       plat->sysclk, ret);
		return ret;
	}

	return 0;
}


static void imx_hifi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct mxc_audio_platform_data *plat = card_priv.pdev->dev.platform_data;

	if (!codec_dai->active)
		plat->clock_enable(0);
}

static int imx_wm8731_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret = 0;
	struct snd_soc_codec *codec = rtd->codec;

	/* Add imx specific widgets */
	ret = snd_soc_dapm_new_controls(&codec->dapm, imx_dapm_widgets,
					ARRAY_SIZE(imx_dapm_widgets));
	if (ret)
		goto out_retcode;

	/* Set up imx specific audio path audio_map */
	ret = snd_soc_dapm_add_routes(&codec->dapm, audio_map, ARRAY_SIZE(audio_map));
	if (ret)
		goto out_retcode;

	ret = snd_soc_dapm_enable_pin(&codec->dapm, "Headphone Jack");
	if (ret)
		goto out_retcode;

	ret = snd_soc_dapm_nc_pin(&codec->dapm, "Ext Spk");
	if (ret)
		goto out_retcode;

out_retcode:

	if (ret)
		pr_err("%s: failed with error code: %d \n", __FUNCTION__, ret);
	else
		pr_info("%s: success \n", __FUNCTION__);

	return ret;
}

/**
 * Configure AUDMUX interconnection between
 * _slave (CPU side) and _master (codec size)
 *
 * When SSI operates in master mode, 5-wire interconnect with
 * audio codec is required:
 * TXC  - BCLK
 * TXD  - DAC data
 * RXD  - ADC data
 * TXFS - {DAC|ADC}LRC, i.e. word clock
 * RXC  - MCLK, i.e. oversampling clock
 * Audmux is operated in asynchronous mode to enable 6-wire
 * interface (as opposed to 4-wire interface in sync mode).
 */
static int imx_audmux_config_slv_mode(int _slave, int _master)
{
	unsigned int ptcr, pdcr;
	int slave = _slave - 1;
	int master = _master - 1;


	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TFSDIR |
		MXC_AUDMUX_V2_PTCR_TFSEL(slave) |
		MXC_AUDMUX_V2_PTCR_RCLKDIR |
		MXC_AUDMUX_V2_PTCR_RCSEL(slave | 0x8) |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(slave);

	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);
	ptcr = ptcr & ~MXC_AUDMUX_V2_PTCR_SYN;
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);



	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_RCLKDIR |
		MXC_AUDMUX_V2_PTCR_RCSEL(master | 0x8) |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(master);

	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);
	ptcr = ptcr & ~MXC_AUDMUX_V2_PTCR_SYN;
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

	return 0;
}

static int imx_audmux_config_mst_mode(int _slave, int _master)
{
	unsigned int ptcr, pdcr;
	int slave = _slave - 1;
	int master = _master - 1;


	ptcr = MXC_AUDMUX_V2_PTCR_SYN;
	ptcr |= MXC_AUDMUX_V2_PTCR_TFSDIR |
		MXC_AUDMUX_V2_PTCR_TFSEL(master) |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(master);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);


	ptcr = MXC_AUDMUX_V2_PTCR_SYN;
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);

	return 0;
}

/*
 * Register the snd_soc_pcm_link drivers
 */
static int __devinit imx_wm8731_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	int ret = 0;

	card_priv.pdev = pdev;

	if (!strncmp("wm8731-mst-mode", plat->codec_name, 15)) {
		/* override default settings */
		imx_audmux_config_mst_mode(plat->src_port, plat->ext_port);
		imx_hifi_ops.startup = imx_hifi_startup_mst_mode;
		imx_hifi_ops.hw_params = imx_hifi_hw_params_mst_mode;
	}
	else {
		imx_audmux_config_slv_mode(plat->src_port, plat->ext_port);
	}

	if (plat->init && plat->init()) {
		return -EINVAL;
	}

	return ret;
}

static int __devinit imx_wm8731_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	plat->clock_enable(0);

	if (plat->finit)
		plat->finit();

	return 0;
}

static int __init imx_asoc_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_wm8731_driver);
	if (ret < 0)
		goto out_retcode;

	imx_snd_device = platform_device_alloc("soc-audio", 6);
	if (!imx_snd_device)
		goto out_dev_alloc_error;

	platform_set_drvdata(imx_snd_device, &snd_soc_card_imx);

	ret = platform_device_add(imx_snd_device);
	if (ret == 0)
		goto out_retcode;

	platform_device_put(imx_snd_device);

out_dev_alloc_error:
	platform_driver_unregister(&imx_wm8731_driver);

out_retcode:
	return ret;
}

static void __exit imx_asoc_exit(void)
{
	platform_driver_unregister(&imx_wm8731_driver);
	platform_device_unregister(imx_snd_device);
}

module_init(imx_asoc_init);
module_exit(imx_asoc_exit);

/* Module information */
MODULE_AUTHOR("Andrey Gelman <andrey.gelman@compulab.co.il>");
MODULE_DESCRIPTION("ALSA SoC imx wm8731");
MODULE_LICENSE("GPL");

