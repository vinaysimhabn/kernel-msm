/*
 * Copyright (C) 2014 InforceComputing 
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

struct panel_otm8018b {
	struct device *dev;
	struct drm_panel panel;
	struct regulator *reg_l8_avdd;
	struct regulator *reg_s4_iovdd;
	int pmic8821_mpp2;
	int backlight;
	struct videomode vm;
};

static inline struct panel_otm8018b *to_panel_otm8018b(struct drm_panel *panel)
{
        return container_of(panel, struct panel_otm8018b, panel);
}

static char enter_sleep[2] = {0x10, 0x00};
static char write_memory105[1]={0x11};
static char write_memory106[1]={0x29};

static void panel_otm8018b_destroy(struct drm_panel *panel)
{
	struct panel_otm8018b *panel_otm8018b = to_panel_otm8018b(panel);
	kfree(panel_otm8018b);
}

static int panel_otm8018b_power_on(struct panel_otm8018b *panel_otm8018b)
{
	struct device *dev = panel_otm8018b->dev;
	int ret = 0;

	ret = regulator_set_optimum_mode(panel_otm8018b->reg_l8_avdd, 110000);
	if (ret < 0) {
		dev_err(dev, "failed to set l8 mode: %d\n", ret);
		goto fail1;
	}

	ret = regulator_set_optimum_mode(panel_otm8018b->reg_s4_iovdd, 100000);
	if (ret < 0) {
		dev_err(dev, "failed to set s4 mode: %d\n", ret);
		goto fail2;
	}

	ret = regulator_enable(panel_otm8018b->reg_l8_avdd);
	if (ret) {
		dev_err(dev, "failed to enable l8: %d\n", ret);
		goto fail1;
	}

	udelay(100);

	ret = regulator_enable(panel_otm8018b->reg_s4_iovdd);
	if (ret) {
		dev_err(dev, "failed to enable s4: %d\n", ret);
		goto fail2;
	}
	mdelay(2);
	gpio_set_value(panel_otm8018b->backlight, 1);
	gpio_set_value(panel_otm8018b->pmic8821_mpp2, 1);
        mdelay(1);
        gpio_set_value(panel_otm8018b->pmic8821_mpp2, 0);
        usleep(50);
        gpio_set_value(panel_otm8018b->pmic8821_mpp2, 1);

	return 0;

fail2:
	regulator_disable(panel_otm8018b->reg_s4_iovdd);
fail1:
	regulator_disable(panel_otm8018b->reg_l8_avdd);
	
	return ret;
}

static int panel_otm8018b_power_off(struct  panel_otm8018b *panel_otm8018b)
{
	//struct device *dev = panel_otm8018b->dev;
	int ret = 0;

	gpio_set_value_cansleep(panel_otm8018b->pmic8821_mpp2, 0);
	gpio_set_value_cansleep(panel_otm8018b->backlight, 0);
	udelay(100);
/*
	ret = regulator_disable(panel_otm8018b->reg_l8_avdd);
	if (ret)
		dev_err(dev, "failed to disable l8: %d\n", ret);

	udelay(100);

	ret = regulator_disable(panel_otm8018b->reg_s4_iovdd);
	if (ret)
		dev_err(dev, "failed to disable s4: %d\n", ret);
*/
	return ret;
}

static int otm8018b_prepare(struct drm_panel *panel)
{
	struct panel_otm8018b *panel_otm8018b = to_panel_otm8018b(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(panel_otm8018b->dev);
	int ret = 0;
	DRM_DEBUG_KMS("panel on\n");

	ret = panel_otm8018b_power_on(panel_otm8018b);
	if (ret)
		return ret;

	mipi_dsi_generic_write(dsi,&write_memory105, 1);
        mdelay(20);
	mipi_dsi_generic_write(dsi,&write_memory106, 1);
        mdelay(5);

	return 0;
}

static int otm8018b_unprepare(struct drm_panel *panel)
{
	struct panel_otm8018b *panel_otm8018b = to_panel_otm8018b(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(panel_otm8018b->dev);
	int ret;

	DRM_DEBUG_KMS("panel off\n");
	mdelay(5);

	ret = panel_otm8018b_power_off(panel_otm8018b);
	if (ret)
		return ret;
	
	mipi_dsi_generic_write(dsi,&enter_sleep, 1);

	return 0;
}

static int otm8018b_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
        //struct panel_otm8018b *panel_otm8018b = to_panel_otm8018b(panel);
	struct drm_display_mode *mode;
	u32 hbp, hfp, vbp, vfp, hspw, vspw;

        mode = drm_mode_create(connector->dev);
        if (!mode) {
                DRM_ERROR("failed to create a new display mode\n");
                return 0;
        }

	snprintf(mode->name, sizeof(mode->name), "480x864");

	mode->clock = 30780;

        hbp = 44;
        hfp = 46;
        vbp = 16;
        vfp = 15;
        hspw = 4;
        vspw = 1;

	mode->hdisplay = 480;
	mode->hsync_start = mode->hdisplay + hfp;
	mode->hsync_end = mode->hsync_start + hspw;
	mode->htotal = mode->hsync_end + hbp;

	mode->vdisplay = 864;
	mode->vsync_start = mode->vdisplay + vfp;
	mode->vsync_end = mode->vsync_start + vspw;
	mode->vtotal = mode->vsync_end + vbp;

	mode->flags = 0;
//	drm_display_mode_from_videomode(&panel_otm8018b->vm, mode);
        drm_mode_probed_add(connector, mode);

	return 1;
}

static int otm8018b_enable(struct drm_panel *panel)
{
        return 0;
}

static int otm8018b_disable(struct drm_panel *panel)
{
        return 0;
}

static const struct drm_panel_funcs otm8018b_drm_funcs = {
        .disable = otm8018b_disable,
        .unprepare = otm8018b_unprepare,
        .prepare = otm8018b_prepare,
        .enable = otm8018b_enable,
        .get_modes = otm8018b_get_modes,
};

static int otm8018b_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
        struct panel_otm8018b *panel_otm8018b;
        int ret;

        panel_otm8018b = devm_kzalloc(dev, sizeof(struct panel_otm8018b), GFP_KERNEL);
        if (!panel_otm8018b)
                return -ENOMEM;

        mipi_dsi_set_drvdata(dsi, panel_otm8018b);

	panel_otm8018b->dev = dev;

        dsi->lanes = 2;
        dsi->format = MIPI_DSI_FMT_RGB888;
        dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST
                | MIPI_DSI_MODE_VIDEO_HFP | MIPI_DSI_MODE_VIDEO_HBP
                | MIPI_DSI_MODE_VIDEO_HSA | MIPI_DSI_MODE_EOT_PACKET
                | MIPI_DSI_MODE_VSYNC_FLUSH | MIPI_DSI_MODE_VIDEO_AUTO_VERT;

        panel_otm8018b->backlight = 86;
        //panel_truly->backlight = config->backlight_gpio;
        ret = gpio_request(panel_otm8018b->backlight, "disp_backlight");
        if (ret) {
                dev_err(dev, "failed to request backlight: %d\n", ret);
                goto fail;
        }
        gpio_export(panel_otm8018b->backlight, true);
        gpio_direction_output(panel_otm8018b->backlight, 1);

	panel_otm8018b->pmic8821_mpp2 = 96;
        ret = gpio_request(panel_otm8018b->pmic8821_mpp2, "disp_rst_n");
        if (ret) {
                dev_err(dev, "failed to request disp_rst_n mpp: %d\n", ret);
                goto fail;
        }
        ret = gpio_export(panel_otm8018b->pmic8821_mpp2, true);
        if (ret) {
                dev_err(dev, "failed to request gpio export mpp: %d\n", ret);
                goto fail;
        }
        ret = gpio_direction_output(panel_otm8018b->pmic8821_mpp2, 0);
        if (ret) {
                dev_err(dev, "failed to request gpio direction output mpp: %d\n", ret);
                goto fail;
        }
        panel_otm8018b->reg_l8_avdd = devm_regulator_get(dev, "dsi1_avdd");
        if (IS_ERR(panel_otm8018b->reg_l8_avdd)) {
                ret = PTR_ERR(panel_otm8018b->reg_l8_avdd);
                dev_err(dev, "failed to request dsi_avdd regulator: %d\n", ret);
                goto fail;
        }

        panel_otm8018b->reg_s4_iovdd = devm_regulator_get(dev, "dsi1_s4_iovdd");
        if (IS_ERR(panel_otm8018b->reg_s4_iovdd)) {
                ret = PTR_ERR(panel_otm8018b->reg_s4_iovdd);
                dev_err(dev, "failed to request dsi_s4_iovdd regulator: %d\n", ret);
                goto fail;
        }

        ret = regulator_set_voltage(panel_otm8018b->reg_l8_avdd,  3300000, 3300000);
        if (ret) {
                dev_err(dev, "set_voltage l8 failed: %d\n", ret);
                goto fail;
        }

        ret = regulator_set_voltage(panel_otm8018b->reg_s4_iovdd,  1800000, 1800000);
        if (ret) {
                dev_err(dev, "set_voltage l2 failed: %d\n", ret);
                goto fail;
        }

	drm_panel_init(&panel_otm8018b->panel);
        panel_otm8018b->panel.dev = dev;
        panel_otm8018b->panel.funcs = &otm8018b_drm_funcs;

        ret = drm_panel_add(&panel_otm8018b->panel);
        if (ret < 0)
                return ret;

        ret = mipi_dsi_attach(dsi);
        if (ret < 0)
                drm_panel_remove(&panel_otm8018b->panel);

        return ret;
fail:
        if (panel_otm8018b)
                panel_otm8018b_destroy(&panel_otm8018b->panel);
        return ret;
}

static struct drm_panel *panel_simple_create(struct device *dev)
{
//	struct device *dev = &dsi->dev;
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(dev);
        struct panel_otm8018b *panel_otm8018b;
        int ret;

        panel_otm8018b = devm_kzalloc(dev, sizeof(*panel_otm8018b), GFP_KERNEL);
        if (!panel_otm8018b)
                return NULL;
	
	panel_otm8018b->dev = dev;

 //       mipi_dsi_set_drvdata(dsi, panel_otm8018b);


        dsi->lanes = 2;
        dsi->format = MIPI_DSI_FMT_RGB888;
        dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST
                | MIPI_DSI_MODE_VIDEO_HFP | MIPI_DSI_MODE_VIDEO_HBP
                | MIPI_DSI_MODE_VIDEO_HSA | MIPI_DSI_MODE_EOT_PACKET
                | MIPI_DSI_MODE_VSYNC_FLUSH | MIPI_DSI_MODE_VIDEO_AUTO_VERT;

        panel_otm8018b->backlight = 86;
        ret = gpio_request(panel_otm8018b->backlight, "disp_backlight");
        if (ret) {
                dev_err(dev, "failed to request backlight: %d\n", ret);
                goto fail;
        }
        gpio_export(panel_otm8018b->backlight, true);
        gpio_direction_output(panel_otm8018b->backlight, 1);

	panel_otm8018b->pmic8821_mpp2 = 96;
        ret = gpio_request(panel_otm8018b->pmic8821_mpp2, "disp_rst_n");
        if (ret) {
                dev_err(dev, "failed to request disp_rst_n mpp: %d\n", ret);
                goto fail;
        }
        ret = gpio_export(panel_otm8018b->pmic8821_mpp2, true);
        if (ret) {
                dev_err(dev, "failed to request gpio export mpp: %d\n", ret);
                goto fail;
        }
        ret = gpio_direction_output(panel_otm8018b->pmic8821_mpp2, 0);
        if (ret) {
                dev_err(dev, "failed to request gpio direction output mpp: %d\n", ret);
                goto fail;
        }
        panel_otm8018b->reg_l8_avdd = devm_regulator_get(dev, "8084_l18");
        if (IS_ERR(panel_otm8018b->reg_l8_avdd)) {
                ret = PTR_ERR(panel_otm8018b->reg_l8_avdd);
                dev_err(dev, "failed to request 8084_l18 regulator: %d\n", ret);
                goto fail;
        }

        panel_otm8018b->reg_s4_iovdd = devm_regulator_get(dev, "8084_s4");
        if (IS_ERR(panel_otm8018b->reg_s4_iovdd)) {
                ret = PTR_ERR(panel_otm8018b->reg_s4_iovdd);
                dev_err(dev, "failed to request 8084_s4 regulator: %d\n", ret);
                goto fail;
        }

        ret = regulator_set_voltage(panel_otm8018b->reg_l8_avdd,  2850000, 2850000);
        if (ret) {
                dev_err(dev, "set_voltage l8 failed: %d\n", ret);
                goto fail;
        }

        ret = regulator_set_voltage(panel_otm8018b->reg_s4_iovdd,  1800000, 1800000);
        if (ret) {
                dev_err(dev, "set_voltage l2 failed: %d\n", ret);
                goto fail;
        }

	drm_panel_init(&panel_otm8018b->panel);
        panel_otm8018b->panel.dev = dev;
        panel_otm8018b->panel.funcs = &otm8018b_drm_funcs;

        ret = drm_panel_add(&panel_otm8018b->panel);
        if (ret < 0)
                return NULL;
/*
        ret = mipi_dsi_attach(dsi);
        if (ret < 0)
                drm_panel_remove(&panel_otm8018b->panel);

        return ret;
*/
        return &panel_otm8018b->panel;
fail:
        if (panel_otm8018b)
                panel_otm8018b_destroy(&panel_otm8018b->panel);
        return &panel_otm8018b->panel;
}

static int otm8018b_remove(struct mipi_dsi_device *dsi)
{
        struct panel_otm8018b *panel_otm8018b = mipi_dsi_get_drvdata(dsi);

        mipi_dsi_detach(dsi);
        drm_panel_remove(&panel_otm8018b->panel);

        return 0;
}


static struct of_device_id otm8018b_of_match[] = {
        { .compatible = "truly,otm8018b" },
        { }
};
MODULE_DEVICE_TABLE(of, otm8018b_of_match);

struct drm_panel *panel_simple_register_truly(struct device *dev, const char *name)
{
        int i;

        for (i = 0; otm8018b_of_match[i].compatible[0]; i++) {
                const struct of_device_id *id = &otm8018b_of_match[i];
                if (!strcmp(id->compatible, name))
                        return panel_simple_create(dev);
        }

        return NULL;
}

static struct mipi_dsi_driver otm8018b_driver = {
        .probe = otm8018b_probe,
        .remove = otm8018b_remove,
        .driver = {
                .name = "panel_otm8018b",
                .of_match_table = otm8018b_of_match,
        },
};
module_mipi_dsi_driver(otm8018b_driver);
MODULE_DESCRIPTION("MIPI-DSI based TRULY 480x864 Panel Driver");
MODULE_LICENSE("GPL v2");

