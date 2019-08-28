/*
 * Copyright (C) 2017 InforceComputing
 * Author: Vinay Simha BN <vinaysimha@inforcecomputing.com>
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

#include <linux/backlight.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include <video/mipi_display.h>

static const char * const regulator_names[] = {
	"vcc",
};

struct auo_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	struct backlight_device *backlight;
	struct regulator_bulk_data supplies[ARRAY_SIZE(regulator_names)];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bklt_gpio;

	bool prepared;
	bool enabled;

	const struct drm_display_mode *mode;
};

static inline struct auo_panel *to_auo_panel(struct drm_panel *panel)
{
	return container_of(panel, struct auo_panel, base);
}

static int auo_panel_init(struct auo_panel *auo)
{
	struct mipi_dsi_device *dsi = auo->dsi;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	/*auo*/
	ret = mipi_dsi_generic_write(dsi, (u8[]){0x50, 0x77}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xE1, 0x66}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xDC, 0x67}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0x50, 0x00}, 2);
	if (ret < 0)
		return ret;

	/* Setting GOA Timing */
	ret = mipi_dsi_generic_write(dsi, (u8[]){0x57, 0x88}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xBE, 0x1A}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xBF, 0x02}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xC0, 0x82}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xC1, 0x03}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xC3, 0x02}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xDA, 0x1A}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xDB, 0x02}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xDC, 0x82}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xDD, 0x0B}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xDE, 0x08}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xDF, 0x03}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xF2, 0x12}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xF3, 0x14}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xB7, 0x02}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0x57, 0x00}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0x58, 0x33}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0x9E, 0x90}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0x9F, 0x24}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xC5, 0xC2}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xCF, 0xD8}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xD2, 0x76}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0x58, 0x00}, 2);
	if (ret < 0)
		return ret;

	/* Enter Level 2 */
	ret = mipi_dsi_generic_write(dsi, (u8[]){0xF0, 0x5A, 0x5A}, 3);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xF5, 0x81, 0x37, 0x40}, 4);
	if (ret < 0)
		return ret;

	/* Setting GOUT */
	ret = mipi_dsi_generic_write(dsi, (u8[])
			{0xF7, 0x16, 0x16, 0x1A, 0x1A, 0x18, 0x18, 0x19}, 8);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xB0, 0x07}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[])
			{0xF7, 0x19, 0x08, 0x08, 0x0A, 0x0A, 0x0C, 0x0C}, 8);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xB0, 0x0E}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[])
			{0xF7, 0x0E, 0x0E, 0x0F, 0x0F, 0x0D, 0x0D, 0x0B}, 8);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xB0, 0x15}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[])
			{0xF7, 0x0B, 0x09, 0x09, 0x19, 0x19, 0x18, 0x18}, 8);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xB0, 0x1C}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[])
			{0xF7, 0x1A, 0x1A, 0x17, 0x17}, 5);
	if (ret < 0)
		return ret;

	/* Setting Postive Gamma*/
	ret = mipi_dsi_generic_write(dsi, (u8[])
			{0xFA, 0x29, 0x1A, 0x2B, 0x24, 0x29, 0x2C, 0x2A}, 8);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xB0, 0x07}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[])
			{0xFA, 0x27, 0x23, 0x29, 0x36, 0x2A, 0x2E, 0x38}, 8);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xB0, 0x0E}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xFA, 0x35, 0x35, 0x19}, 4);
	if (ret < 0)
		return ret;

	/* Setting Negative Gamma*/
	ret = mipi_dsi_generic_write(dsi, (u8[])
			{0xFB, 0x22, 0x13, 0x24, 0x1B, 0x1E, 0x25, 0x29}, 8);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xB0, 0x07}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[])
			{0xFB, 0x2E, 0x21, 0x31, 0x37, 0x2D, 0x31, 0x2F}, 8);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xB0, 0x0E}, 2);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_generic_write(dsi, (u8[]){0xFB, 0x29, 0x2C, 0x12}, 4);
	if (ret < 0)
		return ret;

	/* BCTRL on */
	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				 (u8[]){ 0x24 }, 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		return ret;

	return 0;
}

static int auo_panel_on(struct auo_panel *auo)
{
	struct mipi_dsi_device *dsi = auo->dsi;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0)
		return ret;

	return 0;
}

static int auo_panel_off(struct auo_panel *auo)
{
	struct mipi_dsi_device *dsi = auo->dsi;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0)
		return ret;

	/* BCTRL off */
	ret = mipi_dsi_dcs_write(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				 (u8[]){ 0x00 }, 1);
	if (ret < 0)
		return ret;

	msleep(300);

	return 0;
}

static int auo_panel_disable(struct drm_panel *panel)
{
	struct auo_panel *auo = to_auo_panel(panel);

	if (!auo->enabled)
		return 0;

	DRM_DEBUG("disable\n");

	if (auo->backlight) {
		auo->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(auo->backlight);
	}

	auo->enabled = false;

	return 0;
}

static int auo_panel_unprepare(struct drm_panel *panel)
{
	struct auo_panel *auo = to_auo_panel(panel);
	struct device *dev = &auo->dsi->dev;
	int ret;

	if (!auo->prepared)
		return 0;

	DRM_DEBUG("unprepare\n");

	ret = auo_panel_off(auo);
	if (ret) {
		dev_err(panel->dev, "failed to set panel off: %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_disable(ARRAY_SIZE(auo->supplies), auo->supplies);
	if (ret < 0)
		dev_err(dev, "regulator disable failed, %d\n", ret);

	gpiod_set_value_cansleep(auo->reset_gpio, 0);

	gpiod_set_value_cansleep(auo->bklt_gpio, 0);

	auo->prepared = false;

	return 0;
}

static int auo_panel_prepare(struct drm_panel *panel)
{
	struct auo_panel *auo = to_auo_panel(panel);
	struct device *dev = &auo->dsi->dev;
	int ret;

	if (auo->prepared)
		return 0;

	DRM_DEBUG("prepare\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(auo->supplies), auo->supplies);
	if (ret < 0) {
		dev_err(dev, "regulator enable failed, %d\n", ret);
		return ret;
	}

	gpiod_set_value_cansleep(auo->reset_gpio, 1);
	mdelay(5);

	gpiod_set_value_cansleep(auo->bklt_gpio, 1);
	mdelay(1);

	ret = auo_panel_init(auo);
	if (ret) {
		dev_err(panel->dev, "failed to init panel: %d\n", ret);
		goto poweroff;
	}

	ret = auo_panel_on(auo);
	if (ret) {
		dev_err(panel->dev, "failed to set panel on: %d\n", ret);
		goto poweroff;
	}

	auo->prepared = true;

	return 0;

poweroff:
	ret = regulator_bulk_disable(ARRAY_SIZE(auo->supplies), auo->supplies);
	if (ret < 0)
		dev_err(dev, "regulator disable failed, %d\n", ret);

	gpiod_set_value_cansleep(auo->reset_gpio, 0);

	gpiod_set_value_cansleep(auo->bklt_gpio, 0);

	return ret;
}

static int auo_panel_enable(struct drm_panel *panel)
{
	struct auo_panel *auo = to_auo_panel(panel);

	if (auo->enabled)
		return 0;

	DRM_DEBUG("enable\n");

	if (auo->backlight) {
		auo->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(auo->backlight);
	}

	auo->enabled = true;

	return 0;
}

static const struct drm_display_mode default_mode = {
	.clock = 66300,
	.hdisplay = 800,
	.hsync_start = 800 + 20,
	.hsync_end = 800 + 20 + 10,
	.htotal = 800 + 20 + 10 + 20,
	.vdisplay = 1280,
	.vsync_start = 1280 + 8,
	.vsync_end = 1280 + 8 + 4,
	.vtotal = 1280 + 8 + 4 + 8,
	.vrefresh = 60,
};

static int auo_panel_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm = 150;
	panel->connector->display_info.height_mm = 94;

	return 1;
}

static const struct drm_panel_funcs auo_panel_funcs = {
		.disable = auo_panel_disable,
		.unprepare = auo_panel_unprepare,
		.prepare = auo_panel_prepare,
		.enable = auo_panel_enable,
		.get_modes = auo_panel_get_modes,
};

static const struct of_device_id auo_of_match[] = {
		{ .compatible = "auo,at-t070d61-39ta-b1", },
		{ }
};
MODULE_DEVICE_TABLE(of, auo_of_match);

static int auo_panel_add(struct auo_panel *auo)
{
	struct device *dev = &auo->dsi->dev;
	struct device_node *np;
	int ret;
	unsigned int i;

	auo->mode = &default_mode;

	for (i = 0; i < ARRAY_SIZE(auo->supplies); i++)
		auo->supplies[i].supply = regulator_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(auo->supplies),
				      auo->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to init regulator, ret=%d\n", ret);
		return ret;
	}

	auo->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(auo->reset_gpio)) {
		ret = PTR_ERR(auo->reset_gpio);
		dev_err(dev, "cannot get reset-gpios %d\n", ret);
		return ret;
	}

	auo->bklt_gpio = devm_gpiod_get(dev, "bklt-pwm", GPIOD_OUT_HIGH);
	if (IS_ERR(auo->bklt_gpio)) {
		ret = PTR_ERR(auo->bklt_gpio);
		dev_err(dev, "cannot get bklt-gpios %d\n", ret);
		return ret;
	}

//#ifdef TEST
	np = of_parse_phandle(dev->of_node, "backlight", 0);
	if (np) {
		auo->backlight = of_find_backlight_by_node(np);
		of_node_put(np);

		if (!auo->backlight)
			return -EPROBE_DEFER;
	}
//#endif
	drm_panel_init(&auo->base);
	auo->base.funcs = &auo_panel_funcs;
	auo->base.dev = &auo->dsi->dev;

	ret = drm_panel_add(&auo->base);
	if (ret < 0)
		goto put_backlight;

	return 0;

put_backlight:
	if (auo->backlight)
		put_device(&auo->backlight->dev);

	return ret;
}

static void auo_panel_del(struct auo_panel *auo)
{
	if (auo->base.dev)
		drm_panel_remove(&auo->base);

	if (auo->backlight)
		put_device(&auo->backlight->dev);
}

static int auo_panel_probe(struct mipi_dsi_device *dsi)
{
	struct auo_panel *auo;
	int ret;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO;

	auo = devm_kzalloc(&dsi->dev, sizeof(*auo), GFP_KERNEL);
	if (!auo)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, auo);

	auo->dsi = dsi;

	ret = auo_panel_add(auo);
	if (ret < 0)
		return ret;

	return mipi_dsi_attach(dsi);
}

static int auo_panel_remove(struct mipi_dsi_device *dsi)
{
	struct auo_panel *auo = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = auo_panel_disable(&auo->base);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to disable panel: %d\n", ret);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_detach(&auo->base);
	auo_panel_del(auo);

	return 0;
}

static void auo_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct auo_panel *auo = mipi_dsi_get_drvdata(dsi);

	auo_panel_disable(&auo->base);
}

static struct mipi_dsi_driver auo_panel_driver = {
	.driver = {
		.name = "panel-auo-auo",
		.of_match_table = auo_of_match,
	},
	.probe = auo_panel_probe,
	.remove = auo_panel_remove,
	.shutdown = auo_panel_shutdown,
};
module_mipi_dsi_driver(auo_panel_driver);

MODULE_AUTHOR("Vinay Simha BN <vinaysimha@inforcecomputing.com>");
MODULE_DESCRIPTION("AUO 800x1280 AT-T070D61-39TA-B1 DSI panel driver");
MODULE_LICENSE("GPL v2");
