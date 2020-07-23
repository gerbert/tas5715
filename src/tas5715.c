/*
 * ASoC Driver for TAS5715
 *
 * Author:	Andrei Andreyanau <a.andreyanau@sam-solutions.com>
 * Based on ASoC driver for TAS5713 by Sebastian Eickhoff
 *				<basti.eickhoff@googlemail.com>,
 * and TAS5086 ASoC codec driver by Daniel Mack
 *				<zonque@gmail.com>
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include "tas5715.h"

#define TAS5715_DEEMPH_MASK	0x03

static struct i2c_client *i2c;

struct tas5715_priv {
	struct regmap		*regmap;
	struct snd_soc_codec	*codec;
	unsigned int		format;
	int			rate;
	bool			deemph;
	/* GPIO driving Audio Power Down pin, if any */
	int			gpio_audio_pdn;
	/* GPIO driving Audio Reset pin, if any */
	int			gpio_nreset;
};

static int tas5715_register_size(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TAS5715_CLOCK_CTRL ... TAS5715_BKND_ERR:
		return 1;
	case TAS5715_INPUT_MUX:
	case TAS5715_PWM_MUX:
		return 4;
	}

	dev_err(dev, "Unsupported register address: %d\n", reg);
	return 0;
}

static bool tas5715_accessible_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x0b ... 0x0d:
	case 0x15 ... 0x19:
	case 0x1d ... 0x1f:
	case 0x21 ... 0x24:
		return false;
	default:
		return true;
	}
}

static bool tas5715_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TAS5715_DEVICE_ID:
	case TAS5715_ERROR_STATUS:
		return true;
	}

	return false;
}

static bool tas5715_writeable_reg(struct device *dev, unsigned int reg)
{
	return tas5715_accessible_reg(dev, reg) && (reg != TAS5715_DEVICE_ID);
}

static int tas5715_reg_write(void *context, unsigned int reg,
				unsigned int value)
{
	struct i2c_client *client = context;
	unsigned int i, size;
	uint8_t buf[5];
	int ret;

	size = tas5715_register_size(&client->dev, reg);
	if (size == 0)
		return -EINVAL;

	buf[0] = reg;

	for (i = size; i >= 1; --i) {
		buf[i] = value;
		value >>= 8;
	}

	ret = i2c_master_send(client, buf, size + 1);
	if (ret == size + 1)
		return 0;
	else if (ret < 0)
		return ret;
	else
		return -EIO;
}

static int tas5715_reg_read(void *context, unsigned int reg,
			     unsigned int *value)
{
	struct i2c_client *client = context;
	uint8_t send_buf, recv_buf[4];
	struct i2c_msg msgs[2];
	unsigned int size;
	unsigned int i;
	int ret;

	size = tas5715_register_size(&client->dev, reg);
	if (size == 0)
		return -EINVAL;

	send_buf = reg;

	msgs[0].addr = client->addr;
	msgs[0].len = sizeof(send_buf);
	msgs[0].buf = &send_buf;
	msgs[0].flags = 0;

	msgs[1].addr = client->addr;
	msgs[1].len = size;
	msgs[1].buf = recv_buf;
	msgs[1].flags = I2C_M_RD;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	else if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*value = 0;

	for (i = 0; i < size; i++) {
		*value <<= 8;
		*value |= recv_buf[i];
	}

	return 0;
}

static int tas5715_deemph[] = { 0, 32000, 44100, 48000 };

static int tas5715_set_deemph(struct snd_soc_codec *codec)
{
	struct tas5715_priv *priv = snd_soc_codec_get_drvdata(codec);
	int i, val = 0;

	if (priv->deemph)
		for (i = 0; i < ARRAY_SIZE(tas5715_deemph); i++)
			if (tas5715_deemph[i] == priv->rate)
				val = i;

	return regmap_update_bits(priv->regmap, TAS5715_SYSTEM_CTRL1,
				TAS5715_DEEMPH_MASK, val);
}

static int tas5715_get_deemph(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tas5715_priv *priv = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = priv->deemph;

	return 0;
}

static int tas5715_put_deemph(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tas5715_priv *priv = snd_soc_codec_get_drvdata(codec);

	priv->deemph = ucontrol->value.enumerated.item[0];

	return tas5715_set_deemph(codec);
}

static int tas5715_set_dai_fmt(struct snd_soc_dai *codec_dai,
				unsigned int format)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct tas5715_priv *priv = snd_soc_codec_get_drvdata(codec);

	/* The TAS5715 can only be slave to all clocks */
	if ((format & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
		dev_err(codec->dev, "Invalid clocking mode\n");
		return -EINVAL;
	}

	/* We need to refer to the data format from hw_params */
	priv->format = format;

	return 0;
}

static const DECLARE_TLV_DB_SCALE(tas5715_vol_tlv, -10000, 50, 0);

static const struct snd_kcontrol_new tas5715_snd_controls[] = {
	SOC_SINGLE_TLV("Master Playback Volume",
			TAS5715_VOL_MASTER, 0, 248, 1, tas5715_vol_tlv),
	SOC_SINGLE("Master Playback Switch", TAS5715_SYSTEM_CTRL2, 0x06, 0x01, 0),
	SOC_DOUBLE_R_TLV("Channel 1/2 Playback Volume",
			TAS5715_VOL_CH1, TAS5715_VOL_CH2, 0, 248, 1, tas5715_vol_tlv),
	SOC_SINGLE_BOOL_EXT("De-emphasis Switch", 0,
			tas5715_get_deemph, tas5715_put_deemph),
};

static int tas5715_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas5715_priv *priv = snd_soc_codec_get_drvdata(codec);
	int val;
	int ret;

	priv->rate = params_rate(params);

	/*
	 * The chip has a very unituitive register mapping and muxes information
	 * about data format and sample depth into the same register, but not on
	 * a logical bit-boundary. Hence, we have to refer to the format passed
	 * in the set_dai_fmt() callback and set up everything from here.
	 *
	 * First, determine the 'base' value, using the format ...
	 */
	switch (priv->format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
		val = 0x00;
		break;
	case SND_SOC_DAIFMT_I2S:
		val = 0x03;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val = 0x06;
		break;
	default:
		dev_err(codec->dev, "Invalid DAI format\n");
		return -EINVAL;
	}

	/* ... then add the offset for the sample bit depth. */
	switch (params_format(params)) {
        case SNDRV_PCM_FORMAT_S16_LE:
		val += 0;
                break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		val += 1;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		val += 2;
		break;
	default:
		dev_err(codec->dev, "Invalid bit width\n");
		return -EINVAL;
	}

	ret = regmap_write(priv->regmap, TAS5715_SERIAL_DATA_INTERFACE, val);
	if (ret < 0)
		return ret;

	return tas5715_set_deemph(codec);
}

static int tas5715_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas5715_priv *priv = snd_soc_codec_get_drvdata(codec);
	unsigned int val = 0;

	if (mute)
		val = TAS5715_SOFT_MUTE_ALL;

	return regmap_write(priv->regmap, TAS5715_SOFT_MUTE, val);
}

static const struct snd_soc_dai_ops tas5715_dai_ops = {
	.hw_params		= tas5715_hw_params,
	.set_fmt		= tas5715_set_dai_fmt,
	.mute_stream		= tas5715_mute_stream,
};

static struct snd_soc_dai_driver tas5715_dai = {
	.name			= "tas5715-hifi",
	.playback		= {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_48000,
		.formats	= (SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S20_3LE |
				SNDRV_PCM_FMTBIT_S24_LE),
	},
	.ops			= &tas5715_dai_ops,
};

static int tas5715_remove(struct snd_soc_codec *codec)
{
	struct tas5715_priv *priv = snd_soc_codec_get_drvdata(codec);

	if (gpio_is_valid(priv->gpio_nreset))
		/* Set codec to the reset state */
		gpio_set_value(priv->gpio_nreset, 0);

	return 0;
}

static void tas5715_reset(struct tas5715_priv *priv)
{
	/* Perform reset only when reset/power down pins available */
	if (gpio_is_valid(priv->gpio_nreset) &&
		gpio_is_valid(priv->gpio_audio_pdn)) {
		/* Reset codec - minimum assertion time is 100us */
		gpio_direction_output(priv->gpio_nreset, 0);
		gpio_direction_output(priv->gpio_audio_pdn, 1);
		udelay(300);
		gpio_direction_output(priv->gpio_nreset, 1);

		/* Codec needs at least 13.5ms to wake up */
		msleep(15);
	}
}

static int tas5715_probe(struct snd_soc_codec *codec)
{
	struct tas5715_priv *priv = snd_soc_codec_get_drvdata(codec);
	int i, ret;

	i2c = container_of(codec->dev, struct i2c_client, dev);

	codec->control_data = priv->regmap;

	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_REGMAP);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache i/o: %d\n", ret);
		return ret;
	}

	// Reset error
	ret = snd_soc_write(codec, TAS5715_ERROR_STATUS, 0x00);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to reset error status register: %d\n", ret);
		return ret;
	}

	// Trim oscillator
	ret = snd_soc_write(codec, TAS5715_OSC_TRIM, 0x00);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set Oscillator trim register: %d\n", ret);
		return ret;
	}
	msleep(1000);

	// Reset error
	ret = snd_soc_write(codec, TAS5715_ERROR_STATUS, 0x00);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to reset error status register: %d\n", ret);
		return ret;
	}

	// i2s 24bit
	ret = snd_soc_write(codec, TAS5715_SERIAL_DATA_INTERFACE, 0x05);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set Serial interface speed register: %d\n", ret);
		return ret;
	}

	// Unmute
	ret = snd_soc_write(codec, TAS5715_SYSTEM_CTRL2, 0x00);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set System Control Register 2: %d\n", ret);
		return ret;
	}
	ret = snd_soc_write(codec, TAS5715_SOFT_MUTE, 0x00);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set Soft Mute register: %d\n", ret);
		return ret;
	}

	// Set volume to 0dB
	ret = snd_soc_write(codec, TAS5715_VOL_MASTER, 0x30);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set Master Volume: %d\n", ret);
		return ret;
	}

	// Now start programming the default initialization sequence
	for (i = 0; i < ARRAY_SIZE(tas5715_init_sequence); ++i) {
		ret = i2c_master_send(i2c,
				tas5715_init_sequence[i].data,
				tas5715_init_sequence[i].size);
		if (ret < 0)
			printk(KERN_INFO "TAS5715 Codec probe: InitSeq returns: %d\n", ret);
	}

	// Unmute
	ret = snd_soc_write(codec, TAS5715_SYSTEM_CTRL2, 0x00);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set System Control Register 2: %d\n", ret);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_PM
static int tas5715_soc_suspend(struct snd_soc_codec *codec)
{
	struct tas5715_priv *priv = snd_soc_codec_get_drvdata(codec);
	int ret;

	/* Shut down all channels */
	ret = regmap_write(priv->regmap, TAS5715_SYSTEM_CTRL2, 0x40);
	if (ret < 0)
		return ret;

	return 0;
}

static int tas5715_soc_resume(struct snd_soc_codec *codec)
{
	struct tas5715_priv *priv = snd_soc_codec_get_drvdata(codec);
	int ret;

	tas5715_reset(priv);
	regcache_mark_dirty(priv->regmap);

	ret = tas5715_probe(codec);
	if (ret < 0)
		return ret;

	ret = regcache_sync(priv->regmap);
	if (ret < 0)
		return ret;

	return 0;
}
#else
#define tas5715_soc_suspend	NULL
#define tas5715_soc_resume	NULL
#endif /* CONFIG_PM */

static struct snd_soc_codec_driver soc_codec_dev_tas5715 = {
	.probe			= tas5715_probe,
	.remove			= tas5715_remove,
	.suspend		= tas5715_soc_suspend,
	.resume			= tas5715_soc_resume,
	.controls		= tas5715_snd_controls,
	.num_controls		= ARRAY_SIZE(tas5715_snd_controls),
};

static const struct reg_default tas5715_reg_defaults[] = {
	{ 0x07, 0x80 },		// R7	- VOL_MASTER	- -40dB
	{ 0x08, 0x30 },		// R8	- VOL_CH1	- 0dB
	{ 0x09, 0x30 },		// R9	- VOL_CH2	- 0dB
	{ 0x0C, 0x80 },		// R10	- VOL_HEADPHONE	- -40dB
};


static const struct of_device_id tas5715_of_match[] = {
	{ .compatible = "ti,tas5715", },
	{ }
};
MODULE_DEVICE_TABLE(of, tas5715_of_match);

static struct regmap_config tas5715_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 32,
	.max_register		= TAS5715_MAX_REGISTER,
	.reg_defaults		= tas5715_reg_defaults,
	.num_reg_defaults	= ARRAY_SIZE(tas5715_reg_defaults),
	.cache_type		= REGCACHE_RBTREE,
	.volatile_reg		= tas5715_volatile_reg,
	.writeable_reg		= tas5715_writeable_reg,
	.readable_reg		= tas5715_accessible_reg,
	.reg_read		= tas5715_reg_read,
	.reg_write		= tas5715_reg_write,
};

static int tas5715_i2c_probe(struct i2c_client *i2c,
			const struct i2c_device_id *id)
{
	struct tas5715_priv *priv;
	struct device *dev = &i2c->dev;
	int gpio_audio_pdn = -EINVAL;
	int gpio_nreset = -EINVAL;
	int i, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = devm_regmap_init(dev, NULL, i2c, &tas5715_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&i2c->dev, "Failed to create regmap: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(i2c, priv);

	if (of_match_device(of_match_ptr(tas5715_of_match), dev)) {
		struct device_node *of_node = dev->of_node;
		gpio_audio_pdn = of_get_named_gpio(of_node, "audio-power-down", 0);
	}

	if (gpio_is_valid(gpio_audio_pdn))
		if (devm_gpio_request(dev, gpio_audio_pdn, "tas5715-pdn"))
			gpio_audio_pdn = -EINVAL;

	if (of_match_device(of_match_ptr(tas5715_of_match), dev)) {
		struct device_node *of_node = dev->of_node;
		gpio_nreset = of_get_named_gpio(of_node, "audio-reset", 0);
	}

	if (gpio_is_valid(gpio_nreset))
		if (devm_gpio_request(dev, gpio_nreset, "tas5715-reset"))
			gpio_nreset = -EINVAL;


	priv->gpio_audio_pdn = gpio_audio_pdn;
	priv->gpio_nreset = gpio_nreset;
	tas5715_reset(priv);

	/* The TAS5715 always returns 0x44 in its TAS5715_DEVICE_ID register */
	ret = regmap_read(priv->regmap, TAS5715_DEVICE_ID, &i);
	if (ret < 0)
		return ret;

	if (i != 0x44) {
		dev_err(dev,
			"Failed to identify TAS5715 codec (got %02x)\n", i);
		return -ENODEV;
	}

	return snd_soc_register_codec(&i2c->dev, &soc_codec_dev_tas5715,
		&tas5715_dai, 1);
}

static int tas5715_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	i2c_set_clientdata(i2c, NULL);

	return 0;
}

static const struct i2c_device_id tas5715_i2c_id[] = {
	{ "tas5715", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas5715_i2c_id);

static struct i2c_driver tas5715_i2c_driver = {
	.driver = {
		.name		= "tas5715",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(tas5715_of_match),
	},
	.id_table		= tas5715_i2c_id,
	.probe			= tas5715_i2c_probe,
	.remove			= tas5715_i2c_remove,
};

static int __init tas5715_modinit(void)
{
	int ret = 0;

	ret = i2c_add_driver(&tas5715_i2c_driver);
	if (ret)
		printk(KERN_ERR "Failed to register tas5715 I2C driver: %d\n",
			ret);

	return ret;
}
module_init(tas5715_modinit);

static void __exit tas5715_exit(void)
{
	i2c_del_driver(&tas5715_i2c_driver);
}
module_exit(tas5715_exit);

MODULE_AUTHOR("Andrei Andreyanau <a.andreyanau@sam-solutions.com>");
MODULE_DESCRIPTION("ASoC driver for TAS5715");
MODULE_LICENSE("GPL v2");
