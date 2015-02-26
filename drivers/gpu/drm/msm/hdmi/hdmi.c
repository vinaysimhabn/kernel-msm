/*
 * Copyright (c) 2014 The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/of_irq.h>
#include "hdmi.h"
#include <mach/msm_hdmi_audio_codec.h>

enum msm_hdmi_supported_audio_sample_rates {
        AUDIO_SAMPLE_RATE_32KHZ,
        AUDIO_SAMPLE_RATE_44_1KHZ,
        AUDIO_SAMPLE_RATE_48KHZ,
        AUDIO_SAMPLE_RATE_88_2KHZ,
        AUDIO_SAMPLE_RATE_96KHZ,
        AUDIO_SAMPLE_RATE_176_4KHZ,
        AUDIO_SAMPLE_RATE_192KHZ,
        AUDIO_SAMPLE_RATE_MAX
};

/* parameters for clock regeneration */
struct hdmi_tx_audio_acr {
        u32 n;
        u32 cts;
};

struct hdmi_tx_audio_acr_arry {
        u32 pclk;
        struct hdmi_tx_audio_acr lut[AUDIO_SAMPLE_RATE_MAX];
};

/* Audio constants lookup table for hdmi_tx_audio_acr_setup */
/* Valid Pixel-Clock rates: 25.2MHz, 27MHz, 27.03MHz, 74.25MHz, 148.5MHz */
static const struct hdmi_tx_audio_acr_arry hdmi_tx_audio_acr_lut[] = {
        /*  25.200MHz  */
        {25200, {{4096, 25200}, {6272, 28000}, {6144, 25200}, {12544, 28000},
                {12288, 25200}, {25088, 28000}, {24576, 25200} } },
        /*  27.000MHz  */
        {27000, {{4096, 27000}, {6272, 30000}, {6144, 27000}, {12544, 30000},
                {12288, 27000}, {25088, 30000}, {24576, 27000} } },
        /*  27.027MHz */
        {27030, {{4096, 27027}, {6272, 30030}, {6144, 27027}, {12544, 30030},
                {12288, 27027}, {25088, 30030}, {24576, 27027} } },
        /*  74.250MHz */
        {74250, {{4096, 74250}, {6272, 82500}, {6144, 74250}, {12544, 82500},
                {12288, 74250}, {25088, 82500}, {24576, 74250} } },
        /* 148.500MHz */
        {148500, {{4096, 148500}, {6272, 165000}, {6144, 148500},
                {12544, 165000}, {12288, 148500}, {25088, 165000},
                {24576, 148500} } },
        /* 297.000MHz */
        {297000, {{3072, 222750}, {4704, 247500}, {5120, 247500},
                {9408, 247500}, {10240, 247500}, {18816, 247500},
                {20480, 247500} } },
};

void hdmi_set_mode(struct hdmi *hdmi, bool power_on)
{
	uint32_t ctrl = 0;

	if (power_on) {
		ctrl |= HDMI_CTRL_ENABLE;
		if (!hdmi->hdmi_mode) {
			ctrl |= HDMI_CTRL_HDMI;
			hdmi_write(hdmi, REG_HDMI_CTRL, ctrl);
			ctrl &= ~HDMI_CTRL_HDMI;
		} else {
			ctrl |= HDMI_CTRL_HDMI;
		}
	} else {
		ctrl = HDMI_CTRL_HDMI;
	}

	hdmi_write(hdmi, REG_HDMI_CTRL, ctrl);
	DBG("HDMI Core: %s, HDMI_CTRL=0x%08x",
			power_on ? "Enable" : "Disable", ctrl);
}

static irqreturn_t hdmi_irq(int irq, void *dev_id)
{
	struct hdmi *hdmi = dev_id;

	/* Process HPD: */
	hdmi_connector_irq(hdmi->connector);

	/* Process DDC: */
	hdmi_i2c_irq(hdmi->i2c);

	/* TODO audio.. */

	return IRQ_HANDLED;
}

static void hdmi_destroy(struct hdmi *hdmi)
{
	struct hdmi_phy *phy = hdmi->phy;

	if (phy)
		phy->funcs->destroy(phy);

	if (hdmi->i2c)
		hdmi_i2c_destroy(hdmi->i2c);

	platform_set_drvdata(hdmi->pdev, NULL);
}

/* construct hdmi at bind/probe time, grab all the resources.  If
 * we are to EPROBE_DEFER we want to do it here, rather than later
 * at modeset_init() time
 */
static struct hdmi *hdmi_init(struct platform_device *pdev)
{
	struct hdmi_platform_config *config = pdev->dev.platform_data;
	struct hdmi *hdmi = NULL;
	int i, ret;

	hdmi = devm_kzalloc(&pdev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi) {
		ret = -ENOMEM;
		goto fail;
	}

	hdmi->pdev = pdev;
	hdmi->config = config;

	/* not sure about which phy maps to which msm.. probably I miss some */
	if (config->phy_init)
		hdmi->phy = config->phy_init(hdmi);
	else
		hdmi->phy = ERR_PTR(-ENXIO);

	if (IS_ERR(hdmi->phy)) {
		ret = PTR_ERR(hdmi->phy);
		dev_err(&pdev->dev, "failed to load phy: %d\n", ret);
		hdmi->phy = NULL;
		goto fail;
	}

	hdmi->mmio = msm_ioremap(pdev, config->mmio_name, "HDMI");
	if (IS_ERR(hdmi->mmio)) {
		ret = PTR_ERR(hdmi->mmio);
		goto fail;
	}

	hdmi->hpd_regs = devm_kzalloc(&pdev->dev, sizeof(hdmi->hpd_regs[0]) *
			config->hpd_reg_cnt, GFP_KERNEL);
	if (!hdmi->hpd_regs) {
		ret = -ENOMEM;
		goto fail;
	}
	for (i = 0; i < config->hpd_reg_cnt; i++) {
		struct regulator *reg;

		reg = devm_regulator_get(&pdev->dev,
				config->hpd_reg_names[i]);
		if (IS_ERR(reg)) {
			ret = PTR_ERR(reg);
			dev_err(&pdev->dev, "failed to get hpd regulator: %s (%d)\n",
					config->hpd_reg_names[i], ret);
			goto fail;
		}

		hdmi->hpd_regs[i] = reg;
	}

	hdmi->pwr_regs = devm_kzalloc(&pdev->dev, sizeof(hdmi->pwr_regs[0]) *
			config->pwr_reg_cnt, GFP_KERNEL);
	if (!hdmi->pwr_regs) {
		ret = -ENOMEM;
		goto fail;
	}
	for (i = 0; i < config->pwr_reg_cnt; i++) {
		struct regulator *reg;

		reg = devm_regulator_get(&pdev->dev,
				config->pwr_reg_names[i]);
		if (IS_ERR(reg)) {
			ret = PTR_ERR(reg);
			dev_err(&pdev->dev, "failed to get pwr regulator: %s (%d)\n",
					config->pwr_reg_names[i], ret);
			goto fail;
		}

		hdmi->pwr_regs[i] = reg;
	}

	hdmi->hpd_clks = devm_kzalloc(&pdev->dev, sizeof(hdmi->hpd_clks[0]) *
			config->hpd_clk_cnt, GFP_KERNEL);
	if (!hdmi->hpd_clks) {
		ret = -ENOMEM;
		goto fail;
	}
	for (i = 0; i < config->hpd_clk_cnt; i++) {
		struct clk *clk;

		clk = devm_clk_get(&pdev->dev, config->hpd_clk_names[i]);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			dev_err(&pdev->dev, "failed to get hpd clk: %s (%d)\n",
					config->hpd_clk_names[i], ret);
			goto fail;
		}

		hdmi->hpd_clks[i] = clk;
	}

	hdmi->pwr_clks = devm_kzalloc(&pdev->dev, sizeof(hdmi->pwr_clks[0]) *
			config->pwr_clk_cnt, GFP_KERNEL);
	if (!hdmi->pwr_clks) {
		ret = -ENOMEM;
		goto fail;
	}
	for (i = 0; i < config->pwr_clk_cnt; i++) {
		struct clk *clk;

		clk = devm_clk_get(&pdev->dev, config->pwr_clk_names[i]);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			dev_err(&pdev->dev, "failed to get pwr clk: %s (%d)\n",
					config->pwr_clk_names[i], ret);
			goto fail;
		}

		hdmi->pwr_clks[i] = clk;
	}

	hdmi->i2c = hdmi_i2c_init(hdmi);
	if (IS_ERR(hdmi->i2c)) {
		ret = PTR_ERR(hdmi->i2c);
		dev_err(&pdev->dev, "failed to get i2c: %d\n", ret);
		hdmi->i2c = NULL;
		goto fail;
	}

	return hdmi;

fail:
	if (hdmi)
		hdmi_destroy(hdmi);

	return ERR_PTR(ret);
}

/* Second part of initialization, the drm/kms level modeset_init,
 * constructs/initializes mode objects, etc, is called from master
 * driver (not hdmi sub-device's probe/bind!)
 *
 * Any resource (regulator/clk/etc) which could be missing at boot
 * should be handled in hdmi_init() so that failure happens from
 * hdmi sub-device's probe.
 */
int hdmi_modeset_init(struct hdmi *hdmi,
		struct drm_device *dev, struct drm_encoder *encoder)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct platform_device *pdev = hdmi->pdev;
	int ret;

	hdmi->dev = dev;
	hdmi->encoder = encoder;

	hdmi_audio_infoframe_init(&hdmi->audio.infoframe);

	hdmi->bridge = hdmi_bridge_init(hdmi);
	if (IS_ERR(hdmi->bridge)) {
		ret = PTR_ERR(hdmi->bridge);
		dev_err(dev->dev, "failed to create HDMI bridge: %d\n", ret);
		hdmi->bridge = NULL;
		goto fail;
	}

	hdmi->connector = hdmi_connector_init(hdmi);
	if (IS_ERR(hdmi->connector)) {
		ret = PTR_ERR(hdmi->connector);
		dev_err(dev->dev, "failed to create HDMI connector: %d\n", ret);
		hdmi->connector = NULL;
		goto fail;
	}

	hdmi->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (hdmi->irq < 0) {
		ret = hdmi->irq;
		dev_err(dev->dev, "failed to get irq: %d\n", ret);
		goto fail;
	}

	ret = devm_request_irq(&pdev->dev, hdmi->irq,
			hdmi_irq, IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
			"hdmi_isr", hdmi);
	if (ret < 0) {
		dev_err(dev->dev, "failed to request IRQ%u: %d\n",
				hdmi->irq, ret);
		goto fail;
	}

	encoder->bridge = hdmi->bridge;

	priv->bridges[priv->num_bridges++]       = hdmi->bridge;
	priv->connectors[priv->num_connectors++] = hdmi->connector;

	platform_set_drvdata(pdev, hdmi);

	return 0;

fail:
	/* bridge/connector are normally destroyed by drm: */
	if (hdmi->bridge) {
		hdmi->bridge->funcs->destroy(hdmi->bridge);
		hdmi->bridge = NULL;
	}
	if (hdmi->connector) {
		hdmi->connector->funcs->destroy(hdmi->connector);
		hdmi->connector = NULL;
	}

	return ret;
}

/*
 * The hdmi device:
 */

#include <linux/of_gpio.h>

#define HDMI_CFG(item, entry) \
	.item ## _names = item ##_names_ ## entry, \
	.item ## _cnt   = ARRAY_SIZE(item ## _names_ ## entry)

static struct hdmi_platform_config hdmi_tx_8660_config = {
		.phy_init = hdmi_phy_8x60_init,
};

static const char *hpd_reg_names_8960[] = {"core-vdda", "hdmi-mux"};
static const char *hpd_clk_names_8960[] = {"core_clk", "master_iface_clk", "slave_iface_clk"};

static struct hdmi_platform_config hdmi_tx_8960_config = {
		.phy_init = hdmi_phy_8960_init,
		HDMI_CFG(hpd_reg, 8960),
		HDMI_CFG(hpd_clk, 8960),
};

static const char *pwr_reg_names_8x74[] = {"core-vdda", "core-vcc"};
static const char *hpd_reg_names_8x74[] = {"hpd-gdsc", "hpd-5v"};
static const char *pwr_clk_names_8x74[] = {"extp_clk", "alt_iface_clk"};
static const char *hpd_clk_names_8x74[] = {"iface_clk", "core_clk", "mdp_core_clk"};
static unsigned long hpd_clk_freq_8x74[] = {0, 19200000, 0};

static struct hdmi_platform_config hdmi_tx_8074_config = {
		.phy_init = hdmi_phy_8x74_init,
		HDMI_CFG(pwr_reg, 8x74),
		HDMI_CFG(hpd_reg, 8x74),
		HDMI_CFG(pwr_clk, 8x74),
		HDMI_CFG(hpd_clk, 8x74),
		.hpd_freq      = hpd_clk_freq_8x74,
};

static const char *hpd_reg_names_8084[] = {"hpd-gdsc", "hpd-5v", "hpd-5v-en"};

static struct hdmi_platform_config hdmi_tx_8084_config = {
		.phy_init = hdmi_phy_8x74_init,
		HDMI_CFG(pwr_reg, 8x74),
		HDMI_CFG(hpd_reg, 8084),
		HDMI_CFG(pwr_clk, 8x74),
		HDMI_CFG(hpd_clk, 8x74),
		.hpd_freq      = hpd_clk_freq_8x74,
};

static const struct of_device_id dt_match[] = {
	{ .compatible = "qcom,hdmi-tx", .data = &hdmi_tx_8084_config }, /* compat w/ downstream android dtbs */
	{ .compatible = "qcom,hdmi-tx-8084", .data = &hdmi_tx_8084_config },
	{ .compatible = "qcom,hdmi-tx-8074", .data = &hdmi_tx_8074_config },
	{ .compatible = "qcom,hdmi-tx-8960", .data = &hdmi_tx_8960_config },
	{ .compatible = "qcom,hdmi-tx-8660", .data = &hdmi_tx_8660_config },
	{}
};

#ifdef CONFIG_OF
static int get_gpio(struct device *dev, struct device_node *of_node, const char *name)
{
	int gpio = of_get_named_gpio(of_node, name, 0);
	if (gpio < 0) {
		char name2[32];
		snprintf(name2, sizeof(name2), "%s-gpio", name);
		gpio = of_get_named_gpio(of_node, name2, 0);
		if (gpio < 0) {
			dev_err(dev, "failed to get gpio: %s (%d)\n",
					name, gpio);
			gpio = -1;
		}
	}
	return gpio;
}
#endif

static int hdmi_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = dev_get_drvdata(master);
	struct msm_drm_private *priv = drm->dev_private;
	static struct hdmi_platform_config *hdmi_cfg;
	struct hdmi *hdmi;
#ifdef CONFIG_OF
	struct device_node *of_node = dev->of_node;
	const struct of_device_id *match;

	match = of_match_node(dt_match, of_node);
	if (match && match->data) {
		hdmi_cfg = (struct hdmi_platform_config *)match->data;
		DBG("hdmi phy: %s", match->compatible);
	} else {
		dev_err(dev, "unknown phy: %s\n", of_node->name);
		return -ENXIO;
	}

	hdmi_cfg->mmio_name     = "core_physical";
	hdmi_cfg->ddc_clk_gpio  = get_gpio(dev, of_node, "qcom,hdmi-tx-ddc-clk");
	hdmi_cfg->ddc_data_gpio = get_gpio(dev, of_node, "qcom,hdmi-tx-ddc-data");
	hdmi_cfg->hpd_gpio      = get_gpio(dev, of_node, "qcom,hdmi-tx-hpd");
	hdmi_cfg->mux_en_gpio   = get_gpio(dev, of_node, "qcom,hdmi-tx-mux-en");
	hdmi_cfg->mux_sel_gpio  = get_gpio(dev, of_node, "qcom,hdmi-tx-mux-sel");
	hdmi_cfg->mux_lpm_gpio  = get_gpio(dev, of_node, "qcom,hdmi-tx-mux-lpm");

#else
	static struct hdmi_platform_config config = {};
	static const char *hpd_clk_names[] = {
			"core_clk", "master_iface_clk", "slave_iface_clk",
	};
	if (cpu_is_apq8064()) {
		static const char *hpd_reg_names[] = {"8921_hdmi_mvs"};
		config.phy_init      = hdmi_phy_8960_init;
		config.mmio_name     = "hdmi_msm_hdmi_addr";
		config.hpd_reg_names = hpd_reg_names;
		config.hpd_reg_cnt   = ARRAY_SIZE(hpd_reg_names);
		config.hpd_clk_names = hpd_clk_names;
		config.hpd_clk_cnt   = ARRAY_SIZE(hpd_clk_names);
		config.ddc_clk_gpio  = 70;
		config.ddc_data_gpio = 71;
		config.hpd_gpio      = 72;
		config.mux_en_gpio   = -1;
		config.mux_sel_gpio  = -1;
	} else if (cpu_is_msm8960() || cpu_is_msm8960ab()) {
		static const char *hpd_reg_names[] = {"8921_hdmi_mvs"};
		config.phy_init      = hdmi_phy_8960_init;
		config.mmio_name     = "hdmi_msm_hdmi_addr";
		config.hpd_reg_names = hpd_reg_names;
		config.hpd_reg_cnt   = ARRAY_SIZE(hpd_reg_names);
		config.hpd_clk_names = hpd_clk_names;
		config.hpd_clk_cnt   = ARRAY_SIZE(hpd_clk_names);
		config.ddc_clk_gpio  = 100;
		config.ddc_data_gpio = 101;
		config.hpd_gpio      = 102;
		config.mux_en_gpio   = -1;
		config.mux_sel_gpio  = -1;
	} else if (cpu_is_msm8x60()) {
		static const char *hpd_reg_names[] = {
				"8901_hdmi_mvs", "8901_mpp0"
		};
		config.phy_init      = hdmi_phy_8x60_init;
		config.mmio_name     = "hdmi_msm_hdmi_addr";
		config.hpd_reg_names = hpd_reg_names;
		config.hpd_reg_cnt   = ARRAY_SIZE(hpd_reg_names);
		config.hpd_clk_names = hpd_clk_names;
		config.hpd_clk_cnt   = ARRAY_SIZE(hpd_clk_names);
		config.ddc_clk_gpio  = 170;
		config.ddc_data_gpio = 171;
		config.hpd_gpio      = 172;
		config.mux_en_gpio   = -1;
		config.mux_sel_gpio  = -1;
	}
	hdmi_cfg = &config;
#endif
	dev->platform_data = hdmi_cfg;

	hdmi = hdmi_init(to_platform_device(dev));
	if (IS_ERR(hdmi))
		return PTR_ERR(hdmi);
	priv->hdmi = hdmi;

	return 0;
}

static void hdmi_unbind(struct device *dev, struct device *master,
		void *data)
{
	struct drm_device *drm = dev_get_drvdata(master);
	struct msm_drm_private *priv = drm->dev_private;
	if (priv->hdmi) {
		hdmi_destroy(priv->hdmi);
		priv->hdmi = NULL;
	}
}

static const struct component_ops hdmi_ops = {
		.bind   = hdmi_bind,
		.unbind = hdmi_unbind,
};

static int hdmi_dev_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &hdmi_ops);
}

static int hdmi_dev_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &hdmi_ops);
	return 0;
}

static int hdmi_tx_audio_acr_setup(struct hdmi *hdmi,
        bool enabled)
{
        /* Read first before writing */
        u32 acr_pck_ctrl_reg;
        u32 sample_rate;

        if (!hdmi) {
                DBG("%s: Invalid input\n", __func__);
                return -EINVAL;
        }

        sample_rate = hdmi->audio.sample_rate;

        acr_pck_ctrl_reg = hdmi_read(hdmi, REG_HDMI_ACR_PKT_CTRL);

        if (enabled) {
                const struct hdmi_tx_audio_acr_arry *audio_acr =
                        &hdmi_tx_audio_acr_lut[0];
                const int lut_size = sizeof(hdmi_tx_audio_acr_lut)
                        / sizeof(*hdmi_tx_audio_acr_lut);
                u32 i, n, cts, layout, multiplier, aud_pck_ctrl_2_reg;

                for (i = 0; i < lut_size;
                        audio_acr = &hdmi_tx_audio_acr_lut[++i]) {
                        if (audio_acr->pclk == hdmi->pixclock)
                                break;
                }
                if (i >= lut_size) {
                        DBG("%s: pixel clk %lu not supported\n", __func__,
                               hdmi ->pixclock);
                        return -EPERM;
                }

                n = audio_acr->lut[sample_rate].n;
                cts = audio_acr->lut[sample_rate].cts;
                layout = (MSM_HDMI_AUDIO_CHANNEL_2 ==
                        hdmi->audio.channel_num) ? 0 : 1;

                if (
                (AUDIO_SAMPLE_RATE_192KHZ == sample_rate) ||
                (AUDIO_SAMPLE_RATE_176_4KHZ == sample_rate)) {
                        multiplier = 4;
                        n >>= 2; /* divide N by 4 and use multiplier */
                } else if (
                (AUDIO_SAMPLE_RATE_96KHZ == sample_rate) ||
                (AUDIO_SAMPLE_RATE_88_2KHZ == sample_rate)) {
                        multiplier = 2;
                        n >>= 1; /* divide N by 2 and use multiplier */
                } else {
                        multiplier = 1;
                }
                DBG("%s: n=%u, cts=%u, layout=%u\n", __func__, n, cts,
                        layout);
		 /* AUDIO_PRIORITY | SOURCE */
                acr_pck_ctrl_reg |= 0x80000100;

                /* Reset multiplier bits */
                acr_pck_ctrl_reg &= ~(7 << 16);

                /* N_MULTIPLE(multiplier) */
                acr_pck_ctrl_reg |= (multiplier & 7) << 16;

                if ((AUDIO_SAMPLE_RATE_48KHZ == sample_rate) ||
                (AUDIO_SAMPLE_RATE_96KHZ == sample_rate) ||
                (AUDIO_SAMPLE_RATE_192KHZ == sample_rate)) {
                        /* SELECT(3) */
                        acr_pck_ctrl_reg |= 3 << 4;
                        /* CTS_48 */
                        cts <<= 12;

                        /* CTS: need to determine how many fractional bits */
                        hdmi_write(hdmi, HDMI_ACR_48_0, cts);
                        /* N */
                        hdmi_write(hdmi, HDMI_ACR_48_1, n);
                } else if (
                (AUDIO_SAMPLE_RATE_44_1KHZ == sample_rate) ||
                (AUDIO_SAMPLE_RATE_88_2KHZ == sample_rate) ||
                (AUDIO_SAMPLE_RATE_176_4KHZ == sample_rate)) {
                        /* SELECT(2) */
                        acr_pck_ctrl_reg |= 2 << 4;
                        /* CTS_44 */
                        cts <<= 12;

                        /* CTS: need to determine how many fractional bits */
                        hdmi_write(hdmi, HDMI_ACR_44_0, cts);
                        /* N */
                        hdmi_write(hdmi, HDMI_ACR_44_1, n);
                } else {        /* default to 32k */
                       /* SELECT(1) */
                        acr_pck_ctrl_reg |= 1 << 4;
                        /* CTS_32 */
                        cts <<= 12;

                        /* CTS: need to determine how many fractional bits */
                        hdmi_write(hdmi, HDMI_ACR_32_0, cts);
                        /* N */
                        hdmi_write(hdmi, HDMI_ACR_32_1, n);
                }
                /* Payload layout depends on number of audio channels */
                /* LAYOUT_SEL(layout) */
                aud_pck_ctrl_2_reg = 1 | (layout << 1);
                /* override | layout */
                hdmi_write(hdmi, REG_HDMI_AUDIO_PKT_CTRL2, aud_pck_ctrl_2_reg);

                /* SEND | CONT */
                acr_pck_ctrl_reg |= 0x00000003;
        } else {
                /* ~(SEND | CONT) */
                acr_pck_ctrl_reg &= ~0x00000003;
        }
	hdmi_write(hdmi, REG_HDMI_ACR_PKT_CTRL, acr_pck_ctrl_reg);

        return 0;
} /* hdmi_tx_audio_acr_setup */

static int hdmi_tx_audio_iframe_setup(struct hdmi *hdmi,
        bool enabled)
{
        u32 channel_count = 1; /* Def to 2 channels -> Table 17 in CEA-D */
        u32 num_of_channels;
        u32 channel_allocation;
        u32 level_shift;
        u32 down_mix;
        u32 check_sum, audio_info_0_reg, audio_info_1_reg;
        u32 audio_info_ctrl_reg;
        u32 aud_pck_ctrl_2_reg;
        u32 layout;

        if (!hdmi) {
                DBG("%s: invalid input\n", __func__);
                return -EINVAL;
        }

        num_of_channels    = hdmi->audio.channel_num;
        channel_allocation = hdmi->audio.spkr_alloc;
        level_shift        = hdmi->audio.level_shift;
        down_mix           = hdmi->audio.down_mix;

        layout = (MSM_HDMI_AUDIO_CHANNEL_2 == num_of_channels) ? 0 : 1;
        aud_pck_ctrl_2_reg = 1 | (layout << 1);
        hdmi_write(hdmi, HDMI_AUDIO_PKT_CTRL2, aud_pck_ctrl_2_reg);

        /*
         * Please see table 20 Audio InfoFrame in HDMI spec
         * FL  = front left
         * FC  = front Center
         * FR  = front right
         * FLC = front left center
         * FRC = front right center
         * RL  = rear left
         * RC  = rear center
         * RR  = rear right
         * RLC = rear left center
         * RRC = rear right center
         * LFE = low frequency effect
         */

        /* Read first then write because it is bundled with other controls */
        audio_info_ctrl_reg = hdmi_read(hdmi, REG_HDMI_INFOFRAME_CTRL0);

        if (enabled) {
                switch (num_of_channels) {
                case MSM_HDMI_AUDIO_CHANNEL_2:
                        break;
                case MSM_HDMI_AUDIO_CHANNEL_3:
                case MSM_HDMI_AUDIO_CHANNEL_4:
                        channel_count = 3;
                        break;
                case MSM_HDMI_AUDIO_CHANNEL_5:
                case MSM_HDMI_AUDIO_CHANNEL_6:
                        channel_count = 5;
                        break;
                case MSM_HDMI_AUDIO_CHANNEL_7:
                case MSM_HDMI_AUDIO_CHANNEL_8:
                        channel_count = 7;
                        break;
                default:
                        DBG("%s: Unsupported num_of_channels = %u\n",
                                __func__, num_of_channels);
                        return -EINVAL;
                }

                /* Program the Channel-Speaker allocation */
                audio_info_1_reg = 0;
               /* CA(channel_allocation) */
                audio_info_1_reg |= channel_allocation & 0xff;
                /* Program the Level shifter */
                audio_info_1_reg |= (level_shift << 11) & 0x00007800;
                /* Program the Down-mix Inhibit Flag */
                audio_info_1_reg |= (down_mix << 15) & 0x00008000;

                hdmi_write(hdmi, REG_HDMI_AUDIO_INFO1, audio_info_1_reg);

                /*
                 * Calculate CheckSum: Sum of all the bytes in the
                 * Audio Info Packet (See table 8.4 in HDMI spec)
                 */
                check_sum = 0;
                /* HDMI_AUDIO_INFO_FRAME_PACKET_HEADER_TYPE[0x84] */
                check_sum += 0x84;
                /* HDMI_AUDIO_INFO_FRAME_PACKET_HEADER_VERSION[0x01] */
                check_sum += 1;
                /* HDMI_AUDIO_INFO_FRAME_PACKET_LENGTH[0x0A] */
                check_sum += 0x0A;
                check_sum += channel_count;
                check_sum += channel_allocation;
                /* See Table 8.5 in HDMI spec */
                check_sum += (level_shift & 0xF) << 3 | (down_mix & 0x1) << 7;
                check_sum &= 0xFF;
                check_sum = (u8) (256 - check_sum);

                audio_info_0_reg = 0;
                /* CHECKSUM(check_sum) */
                audio_info_0_reg |= check_sum & 0xff;
                /* CC(channel_count) */
                audio_info_0_reg |= (channel_count << 8) & 0x00000700;

                hdmi_write(hdmi, REG_HDMI_AUDIO_INFO0, audio_info_0_reg);
                /*
                 * Set these flags
                 * AUDIO_INFO_UPDATE |
                 * AUDIO_INFO_SOURCE |
                 * AUDIO_INFO_CONT   |
                 * AUDIO_INFO_SEND
                 */
                audio_info_ctrl_reg |= 0x000000F0;
        } else {
                /*Clear these flags
                 * ~(AUDIO_INFO_UPDATE |
                 *   AUDIO_INFO_SOURCE |
                 *   AUDIO_INFO_CONT   |
                 *   AUDIO_INFO_SEND)
                 */
                audio_info_ctrl_reg &= ~0x000000F0;
        }
        hdmi_write(hdmi, REG_HDMI_INFOFRAME_CTRL0, audio_info_ctrl_reg);

        return 0;
} /* hdmi_tx_audio_iframe_setup */

                                            
static int hdmi_tx_audio_setup(struct hdmi *hdmi)
{
        int rc = 0;

        if (!hdmi) {
                DBG("%s: invalid input\n", __func__);
                return -EINVAL;
        }

        rc = hdmi_tx_audio_acr_setup(hdmi, true);
        if (rc) {
                DBG("%s: hdmi_tx_audio_acr_setup failed. rc=%d\n",
                        __func__, rc);
                return rc;
        }

        rc = hdmi_tx_audio_iframe_setup(hdmi, true);
        if (rc) {
                DBG("%s: hdmi_tx_audio_iframe_setup failed. rc=%d\n",
                        __func__, rc);
                return rc;
        }

        DBG("HDMI Audio: Enabled\n");

        return 0;
} /* hdmi_tx_audio_setup */

static int hdmi_tx_audio_info_setup(struct platform_device *pdev,
        u32 sample_rate, u32 num_of_channels, u32 channel_allocation,
        u32 level_shift, bool down_mix)
{
        int rc = 0;
        struct hdmi *hdmi = platform_get_drvdata(pdev);

        if (!hdmi) {
                DBG("%s: invalid input\n", __func__);
                return -ENODEV;
        }

        if (hdmi->hdmi_mode && hdmi->power_on) {

                /* Map given sample rate to Enum */
                if (sample_rate == 32000)
                        sample_rate = AUDIO_SAMPLE_RATE_32KHZ;
                else if (sample_rate == 44100)
                        sample_rate = AUDIO_SAMPLE_RATE_44_1KHZ;
                else if (sample_rate == 48000)
                        sample_rate = AUDIO_SAMPLE_RATE_48KHZ;
                else if (sample_rate == 88200)
                        sample_rate = AUDIO_SAMPLE_RATE_88_2KHZ;
                else if (sample_rate == 96000)
                        sample_rate = AUDIO_SAMPLE_RATE_96KHZ;
                else if (sample_rate == 176400)
                        sample_rate = AUDIO_SAMPLE_RATE_176_4KHZ;
                else if (sample_rate == 192000)
                        sample_rate = AUDIO_SAMPLE_RATE_192KHZ;

                hdmi->audio.sample_rate = sample_rate;
                hdmi->audio.rate = sample_rate;
                hdmi->audio.channel_num = num_of_channels;
                hdmi->audio.spkr_alloc  = channel_allocation;
                hdmi->audio.level_shift = level_shift;
                hdmi->audio.down_mix    = down_mix;

                rc = hdmi_tx_audio_setup(hdmi);
                if (rc)
                        DBG("%s: hdmi_tx_audio_iframe_setup failed.rc=%d\n",
                                __func__, rc);
        } else {
                DBG("%s: Error. panel is not on.\n", __func__);
                rc = -EPERM;
        }

        return rc;
} /* hdmi_tx_audio_info_setup */


int msm_hdmi_register_audio_codec(struct platform_device *pdev,
        struct msm_hdmi_audio_codec_ops *ops)
{
        struct hdmi *hdmi = platform_get_drvdata(pdev);

        if (!hdmi || !ops) {
                DBG("%s: invalid input\n", __func__);
                return -ENODEV;
        }

        ops->audio_info_setup = hdmi_tx_audio_info_setup;
//        ops->get_audio_edid_blk = hdmi_tx_get_audio_edid_blk;
//	ops->hdmi_cable_status = hdmi_tx_get_cable_status;

        return 0;
}
EXPORT_SYMBOL(msm_hdmi_register_audio_codec);

static struct platform_driver hdmi_driver = {
	.probe = hdmi_dev_probe,
	.remove = hdmi_dev_remove,
	.driver = {
		.name = "hdmi_msm",
		.of_match_table = dt_match,
	},
};

void __init hdmi_register(void)
{
	platform_driver_register(&hdmi_driver);
}

void __exit hdmi_unregister(void)
{
	platform_driver_unregister(&hdmi_driver);
}
