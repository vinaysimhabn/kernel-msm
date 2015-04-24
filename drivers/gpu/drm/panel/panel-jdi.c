/*
 * Copyright (C) 2015 InforceComputing
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
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>

static char write_memory_jdi1[2]={0x55, 0x00};
static char write_memory_jdi2[2]={0x53, 0x2c};
static char write_memory_jdi3[2]={0x35, 0x00};
static char write_memory_jdi4[1]={0x29};
static char write_memory_jdi5[1]={0x11};

static char write_memory_jdi6_off[1]={0x28};
static char write_memory_jdi7_off[1]={0x10};

struct panel_jdi {
	struct device *dev;
	struct drm_panel panel;
	struct regulator *reg_l8_avdd;
	struct regulator *reg_s4_iovdd;
	int lcd_reset_gpio;
	int disp_en_gpio;
	int backlight;
	struct videomode vm;
};

static inline struct panel_jdi *to_panel_jdi(struct drm_panel *panel)
{
        return container_of(panel, struct panel_jdi, panel);
}

static void panel_jdi_destroy(struct drm_panel *panel)
{
	struct panel_jdi *panel_jdi = to_panel_jdi(panel);
	kfree(panel_jdi);
}

static int panel_jdi_power_on(struct panel_jdi *panel_jdi)
{
	struct device *dev = panel_jdi->dev;
	int ret = 0;

	ret = regulator_set_optimum_mode(panel_jdi->reg_l8_avdd, 110000);
	if (ret < 0) {
		dev_err(dev, "failed to set l8 mode: %d\n", ret);
		goto fail1;
	}

	ret = regulator_set_optimum_mode(panel_jdi->reg_s4_iovdd, 100000);
	if (ret < 0) {
		dev_err(dev, "failed to set s4 mode: %d\n", ret);
		goto fail2;
	}

	ret = regulator_enable(panel_jdi->reg_l8_avdd);
	if (ret) {
		dev_err(dev, "failed to enable l8: %d\n", ret);
		goto fail1;
	}

	udelay(100);

	ret = regulator_enable(panel_jdi->reg_s4_iovdd);
	if (ret) {
		dev_err(dev, "failed to enable s4: %d\n", ret);
		goto fail2;
	}
	mdelay(2);
	gpio_set_value(panel_jdi->backlight, 1);
	gpio_set_value(panel_jdi->disp_en_gpio, 1);
	mdelay(2);

	gpio_set_value(panel_jdi->lcd_reset_gpio, 1);
        mdelay(1);
        gpio_set_value(panel_jdi->lcd_reset_gpio, 0);
        usleep(50);
        gpio_set_value(panel_jdi->lcd_reset_gpio, 1);

	return 0;

fail2:
	regulator_disable(panel_jdi->reg_s4_iovdd);
fail1:
	regulator_disable(panel_jdi->reg_l8_avdd);
	
	return ret;
}

static int panel_jdi_power_off(struct  panel_jdi *panel_jdi)
{
	struct device *dev = panel_jdi->dev;
	int ret = 0;

	gpio_set_value_cansleep(panel_jdi->lcd_reset_gpio, 0);
	gpio_set_value_cansleep(panel_jdi->disp_en_gpio, 0);
	gpio_set_value_cansleep(panel_jdi->backlight, 0);
	udelay(100);

	ret = regulator_disable(panel_jdi->reg_l8_avdd);
	if (ret)
		dev_err(dev, "failed to disable l8: %d\n", ret);

	udelay(100);

	ret = regulator_disable(panel_jdi->reg_s4_iovdd);
	if (ret)
		dev_err(dev, "failed to disable s4: %d\n", ret);

	return ret;
}

static int jdi_prepare(struct drm_panel *panel)
{
	struct panel_jdi *panel_jdi = to_panel_jdi(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(panel_jdi->dev);
	int ret = 0;
	DRM_DEBUG_KMS("panel on\n");

	ret = panel_jdi_power_on(panel_jdi);
	if (ret)
		return ret;

	mipi_dsi_dcs_write_buffer(dsi,&write_memory_jdi1, sizeof(write_memory_jdi1));
	mipi_dsi_dcs_write_buffer(dsi,&write_memory_jdi2, sizeof(write_memory_jdi2));
	mipi_dsi_dcs_write_buffer(dsi,&write_memory_jdi3, sizeof(write_memory_jdi3));
        mdelay(20);
	mipi_dsi_dcs_write_buffer(dsi,&write_memory_jdi4, sizeof(write_memory_jdi4));
        mdelay(5);
	mipi_dsi_dcs_write_buffer(dsi,&write_memory_jdi5, sizeof(write_memory_jdi5));
	
	return 0;
}

static int jdi_unprepare(struct drm_panel *panel)
{
	struct panel_jdi *panel_jdi = to_panel_jdi(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(panel_jdi->dev);
	int ret;

	DRM_DEBUG_KMS("panel off\n");
	mdelay(5);

	ret = panel_jdi_power_off(panel_jdi);
	if (ret)
		return ret;

	mipi_dsi_dcs_write_buffer(dsi,&write_memory_jdi6_off, sizeof(write_memory_jdi6_off));
	mipi_dsi_dcs_write_buffer(dsi,&write_memory_jdi7_off, sizeof(write_memory_jdi7_off));
	
	return 0;
}

static int jdi_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
        struct panel_jdi *panel_jdi = to_panel_jdi(panel);
	struct drm_display_mode *mode;
	int ret;
	
        mode = drm_mode_create(connector->dev);
        if (!mode) {
                DRM_ERROR("failed to create a new display mode\n");
                return 0;
        }

	ret = of_get_videomode(panel->dev->of_node, &panel_jdi->vm, 0);
        if (ret < 0)
                return ret;
	
	drm_display_mode_from_videomode(&panel_jdi->vm, mode);
        drm_mode_probed_add(connector, mode);

	return 1;
}

static int jdi_enable(struct drm_panel *panel)
{
        return 0;
}

static int jdi_disable(struct drm_panel *panel)
{
        return 0;
}

static const struct drm_panel_funcs jdi_drm_funcs = {
        .disable = jdi_disable,
        .unprepare = jdi_unprepare,
        .prepare = jdi_prepare,
        .enable = jdi_enable,
        .get_modes = jdi_get_modes,
};

static int jdi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
        struct panel_jdi *panel_jdi;
        int ret;

        panel_jdi = devm_kzalloc(dev, sizeof(struct panel_jdi), GFP_KERNEL);
        if (!panel_jdi)
                return -ENOMEM;

        mipi_dsi_set_drvdata(dsi, panel_jdi);

	panel_jdi->dev = dev;

        dsi->lanes = 4;
        dsi->format = MIPI_DSI_FMT_RGB888;
        dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST;

        panel_jdi->disp_en_gpio = 27;
        ret = gpio_request(panel_jdi->disp_en_gpio, "disp_en_gpio");
        if (ret) {
                dev_err(dev, "failed to request disp_en_gpio: %d\n", ret);
                goto fail;
        }
        gpio_export(panel_jdi->disp_en_gpio, true);
        gpio_direction_output(panel_jdi->disp_en_gpio, 1);

        panel_jdi->backlight = 86;
        ret = gpio_request(panel_jdi->backlight, "disp_backlight");
        if (ret) {
                dev_err(dev, "failed to request backlight: %d\n", ret);
                goto fail;
        }
        gpio_export(panel_jdi->backlight, true);
        gpio_direction_output(panel_jdi->backlight, 1);

	panel_jdi->lcd_reset_gpio = 96;
        ret = gpio_request(panel_jdi->lcd_reset_gpio, "disp_rst_n");
        if (ret) {
                dev_err(dev, "failed to request disp_rst_n mpp: %d\n", ret);
                goto fail;
        }
        ret = gpio_export(panel_jdi->lcd_reset_gpio, true);
        if (ret) {
                dev_err(dev, "failed to request gpio export mpp: %d\n", ret);
                goto fail;
        }
        ret = gpio_direction_output(panel_jdi->lcd_reset_gpio, 0);
        if (ret) {
                dev_err(dev, "failed to request gpio direction output mpp: %d\n", ret);
                goto fail;
        }

	panel_jdi->reg_l8_avdd = devm_regulator_get(dev, "8084_l18");
        if (IS_ERR(panel_jdi->reg_l8_avdd)) {
                ret = PTR_ERR(panel_jdi->reg_l8_avdd);
                dev_err(dev, "failed to request 8084_l18 regulator: %d\n", ret);
                goto fail;
        }

        panel_jdi->reg_s4_iovdd = devm_regulator_get(dev, "8084_s4");
        if (IS_ERR(panel_jdi->reg_s4_iovdd)) {
                ret = PTR_ERR(panel_jdi->reg_s4_iovdd);
                dev_err(dev, "failed to request 8084_s4 regulator: %d\n", ret);
                goto fail;
        }

        ret = regulator_set_voltage(panel_jdi->reg_l8_avdd,  2850000, 2850000);
        if (ret) {
                dev_err(dev, "set_voltage l8 failed: %d\n", ret);
                goto fail;
        }

        ret = regulator_set_voltage(panel_jdi->reg_s4_iovdd,  1800000, 1800000);
        if (ret) {
                dev_err(dev, "set_voltage l2 failed: %d\n", ret);
                goto fail;
	}

	drm_panel_init(&panel_jdi->panel);
        panel_jdi->panel.dev = dev;
        panel_jdi->panel.funcs = &jdi_drm_funcs;

        ret = drm_panel_add(&panel_jdi->panel);
        if (ret < 0)
                return ret;

        ret = mipi_dsi_attach(dsi);
        if (ret < 0)
                drm_panel_remove(&panel_jdi->panel);

        return ret;
fail:
        if (panel_jdi)
                panel_jdi_destroy(&panel_jdi->panel);
        return ret;
}

static int jdi_remove(struct mipi_dsi_device *dsi)
{
        struct panel_jdi *panel_jdi = mipi_dsi_get_drvdata(dsi);

        mipi_dsi_detach(dsi);
        drm_panel_remove(&panel_jdi->panel);

        return 0;
}

static struct of_device_id jdi_of_match[] = {
        { .compatible = "truly,jdi" },
        { }
};
MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
        .probe = jdi_probe,
        .remove = jdi_remove,
        .driver = {
                .name = "panel_jdi",
                .of_match_table = jdi_of_match,
        },
};
module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("Vinay Simha BN <vinaysimha@inforcecomputing.com>");
MODULE_DESCRIPTION("MIPI-DSI based JDI 1080x1920 Panel Driver");
MODULE_LICENSE("GPL v2");

