/*
 * Copyright (C) 2015 InforceComputing
 * Author: Vinay Simha BN <vinaysimha@inforcecomputing.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */
#define DEBUG
#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include "mipi_tc358775_dsi2lvds.h"

#define SWAP_UINT16(x) (((x) >> 8) | ((x) << 8))
#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | \
		 (((x) & 0x0000FF00) << 8) | ((x) << 24))

/* #define LVDS_EDID_PANEL */
/* #define TC358775XBG_DEBUG */
/* #define TC358775XBG_DEBUG */
#define TC358775XBG_USE_I2C
/* #define VESA_DATA_FORMAT */

struct dsi2lvds {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *dsi_sw_select_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *stby_gpio;
	struct i2c_adapter *adapter;
#ifdef TC358775XBG_USE_I2C
	struct i2c_client *i2c_main;
	struct regmap *regmap;
#endif
	bool prepared;
	bool enabled;
	struct backlight_device *backlight;

	const struct drm_display_mode *mode;
};

#ifdef TC358775XBG_USE_I2C
static const struct regmap_config dsi2lvds_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,

	.max_register = 0x05A8,
	.cache_type = REGCACHE_RBTREE,
};
#endif

static inline struct dsi2lvds *to_dsi2lvds_panel(struct drm_panel *panel)
{
	return container_of(panel, struct dsi2lvds, base);
}

#ifndef LVDS_EDID_PANEL
/* TODO read edid and renter proper values */
static const struct drm_display_mode auo_b101xtn01_mode = {
	.clock = 72000,
	.hdisplay = 1366,
	.hsync_start = 1366 + 20,
	.hsync_end = 1366 + 20 + 70,
	.htotal = 1366 + 20 + 70,
	.vdisplay = 768,
	.vsync_start = 768 + 14,
	.vsync_end = 768 + 14 + 42,
	.vtotal = 768 + 14 + 42,
	.vrefresh = 60,
	.flags = DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC,
};
#endif

static int dsi2lvds_init(struct dsi2lvds *dsi2lvds)
{
	struct mipi_dsi_device *dsi = dsi2lvds->dsi;
	int ret = 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret;
}

static int dsi2lvds_on(struct dsi2lvds *dsi2lvds)
{
	struct mipi_dsi_device *dsi = dsi2lvds->dsi;
	int ret = 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret;
}

static int dsi2lvds_off(struct dsi2lvds *dsi2lvds)
{
	struct mipi_dsi_device *dsi = dsi2lvds->dsi;
	int ret = 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	return ret;
}

static int dsi2lvds_disable(struct drm_panel *panel)
{
	struct dsi2lvds *dsi2lvds = to_dsi2lvds_panel(panel);

	if (!dsi2lvds->enabled)
		return 0;

	DRM_DEBUG("disable\n");

	 if (dsi2lvds->backlight) {
		dsi2lvds->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(dsi2lvds->backlight);
	}

	msleep(100);

	dsi2lvds->enabled = false;

	return 0;
}

static int dsi2lvds_unprepare(struct drm_panel *panel)
{
	struct dsi2lvds *dsi2lvds = to_dsi2lvds_panel(panel);
	int ret;

	if (!dsi2lvds->prepared)
		return 0;

	DRM_DEBUG("unprepare\n");

	ret = dsi2lvds_off(dsi2lvds);
	if (ret) {
		dev_err(panel->dev, "failed to set panel off: %d\n", ret);
		return ret;
	}

	dsi2lvds->prepared = false;

	return 0;
}

static int dsi2lvds_prepare(struct drm_panel *panel)
{
	struct dsi2lvds *dsi2lvds = to_dsi2lvds_panel(panel);
	int ret;

	if (dsi2lvds->prepared)
		return 0;

	DRM_DEBUG("prepare\n");

	if (dsi2lvds->stby_gpio) {
		gpiod_set_value(dsi2lvds->stby_gpio, 1);
		mdelay(10);
	}

	if (dsi2lvds->reset_gpio) {
		gpiod_set_value(dsi2lvds->reset_gpio, 1);
		ndelay(50);
	}

	if (dsi2lvds->dsi_sw_select_gpio) {
		gpiod_set_value(dsi2lvds->dsi_sw_select_gpio, 1);
		mdelay(10);
	}

	ret = dsi2lvds_init(dsi2lvds);
	if (ret) {
		dev_err(panel->dev, "failed to init panel: %d\n", ret);
		goto poweroff;
	}

	ret = dsi2lvds_on(dsi2lvds);
	if (ret) {
		dev_err(panel->dev, "failed to set panel on: %d\n", ret);
		goto poweroff;
	}

	dsi2lvds->prepared = true;

	return 0;

poweroff:
	if (dsi2lvds->reset_gpio)
		gpiod_set_value(dsi2lvds->reset_gpio, 0);
	if (dsi2lvds->stby_gpio)
		gpiod_set_value(dsi2lvds->stby_gpio, 0);
	if (dsi2lvds->dsi_sw_select_gpio)
		gpiod_set_value(dsi2lvds->dsi_sw_select_gpio, 0);

	return ret;
}


#ifdef TC358775XBG_DEBUG
static u32 dsi2lvds_read(struct dsi2lvds *dsi2lvds, u16 reg)
{
	int ret;
	u32 data;

	ret = regmap_bulk_read(dsi2lvds->regmap, reg, &data, 1);
	if (ret)
		return ret;

	pr_debug("dsi2lvds: I2C : reg:%04x value:%08x\n",
				reg, SWAP_UINT32(data));

	return data;
}
#endif

static int dsi2lvds_write(struct dsi2lvds *dsi2lvds, u16 reg, u32 data)
{
	int ret;

#ifdef TC358775XBG_DEBUG
	dsi2lvds_read(dsi2lvds, reg);
#endif

	ret = regmap_bulk_write(dsi2lvds->regmap, reg,
			(u32[]) { SWAP_UINT32(data) }, 1);
	if (ret)
		return ret;

#ifdef TC358775XBG_DEBUG
	dsi2lvds_read(dsi2lvds, reg);
#endif
	return ret;
}

#ifdef TC358775XBG_DEBUG
static int dsi2lvds_read_status(struct dsi2lvds *dsi2lvds)
{
	int ret = 0;
	u32 tmp = 0;

	tmp = dsi2lvds_read(dsi2lvds, DSIERRCNT);
	dsi2lvds_write(dsi2lvds, DSIERRCNT, 0xFFFF0000);

	dsi2lvds_read(dsi2lvds, DSI_LANESTATUS0);
	dsi2lvds_read(dsi2lvds, DSI_LANESTATUS1);
	dsi2lvds_read(dsi2lvds, DSI_INTSTATUS);
	dsi2lvds_read(dsi2lvds, SYSSTAT);

	dsi2lvds_write(dsi2lvds, DSIERRCNT, tmp);

	return ret;
}
#endif

static int dsi2lvds_enable(struct drm_panel *panel)
{
	struct dsi2lvds *dsi2lvds = to_dsi2lvds_panel(panel);
	unsigned int val;
	int ret;
	u32 hbpr, hpw, htime1, hfpr, hsize, htime2;
	u32 vbpr, vpw, vtime1, vfpr, vsize, vtime2;

	hbpr = 0;
	hpw  = auo_b101xtn01_mode.hsync_end - auo_b101xtn01_mode.hsync_start;
	vbpr = 0;
	vpw  = auo_b101xtn01_mode.vsync_end - auo_b101xtn01_mode.vsync_start;

	htime1 = (hbpr << 16) + hpw;
	vtime1 = (vbpr << 16) + vpw;

	hfpr = auo_b101xtn01_mode.hsync_start - auo_b101xtn01_mode.hdisplay;
	hsize = auo_b101xtn01_mode.hdisplay;
	vfpr = auo_b101xtn01_mode.vsync_start - auo_b101xtn01_mode.vdisplay;
	vsize = auo_b101xtn01_mode.vdisplay;

	htime2 = (hfpr << 16) + hsize;
	vtime2 = (vfpr << 16) + vsize;

	if (dsi2lvds->enabled)
		return 0;

	DRM_DEBUG("enable\n");

	if (dsi2lvds->backlight) {
		dsi2lvds->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(dsi2lvds->backlight);
	}

	ret = regmap_read(dsi2lvds->regmap, IDREG, &val);
	if (ret)
		return ret;

	pr_debug("dsi2lvsd: IDREG I2C : addr:%04x value:%04x\n",
						IDREG, SWAP_UINT16(val));

	dsi2lvds_write(dsi2lvds, SYSRST, 0x000000FF);
	mdelay(30);

	dsi2lvds_write(dsi2lvds, PPI_TX_RX_TA, 0x00040006);
	dsi2lvds_write(dsi2lvds, PPI_LPTXTIMECNT, 0x00000004);
	dsi2lvds_write(dsi2lvds, PPI_D0S_CLRSIPOCOUNT, 0x00000003);
	dsi2lvds_write(dsi2lvds, PPI_D1S_CLRSIPOCOUNT, 0x00000003);
	dsi2lvds_write(dsi2lvds, PPI_D2S_CLRSIPOCOUNT, 0x00000003);
	dsi2lvds_write(dsi2lvds, PPI_D3S_CLRSIPOCOUNT, 0x00000003);
	dsi2lvds_write(dsi2lvds, PPI_LANEENABLE, 0x0000001F);
	dsi2lvds_write(dsi2lvds, DSI_LANEENABLE, 0x0000001F);
	dsi2lvds_write(dsi2lvds, PPI_STARTPPI, 0x00000001);
	dsi2lvds_write(dsi2lvds, DSI_STARTDSI, 0x00000001);

	/* RGB666 - BIT8(1'b0), Magic square RGB66 18bit ~RGB888 24-bit */
	dsi2lvds_write(dsi2lvds, VPCTRL, 0x01500001);
	dsi2lvds_write(dsi2lvds, HTIM1, htime1);
	dsi2lvds_write(dsi2lvds, VTIM1, vtime1);
	dsi2lvds_write(dsi2lvds, HTIM2, htime2);
	dsi2lvds_write(dsi2lvds, VTIM2, vtime2);

	dsi2lvds_write(dsi2lvds, VFUEN, 0x00000001);
	dsi2lvds_write(dsi2lvds, SYSRST, 0x00000004);

	dsi2lvds_write(dsi2lvds, LVPHY0, 0x00040006);

#ifdef VESA_DATA_FORMAT
	dsi2lvds_write(dsi2lvds, LVMX0003, 0x03020100);
	dsi2lvds_write(dsi2lvds, LVMX0407, 0x08050704);
	dsi2lvds_write(dsi2lvds, LVMX0811, 0x0F0E0A09);
	dsi2lvds_write(dsi2lvds, LVMX1215, 0x100D0C0B);
	dsi2lvds_write(dsi2lvds, LVMX1619, 0x12111716);
	dsi2lvds_write(dsi2lvds, LVMX2023, 0x1B151413);
	dsi2lvds_write(dsi2lvds, LVMX2427, 0x061A1918);
#endif
	/*JEIDA DATA FORMAT default register values*/
	dsi2lvds_write(dsi2lvds, VFUEN, 0x00000001);
	dsi2lvds_write(dsi2lvds, LVCFG, 0x00000031);

#ifdef TC358775XBG_DEBUG
	pr_debug("dsi2lvsd: I2C ------READ STATUS------------\n");
	ret = dsi2lvds_read_status(dsi2lvds);
	if (ret)
		return ret;
	pr_debug("dsi2lvsd: I2C ------READ STATUS------------\n");
#endif

	dsi2lvds->enabled = true;

	return 0;
}

static int dsi2lvds_get_modes(struct drm_panel *panel)
{
#ifdef LVDS_EDID_PANEL
	int num = 0;
	struct dsi2lvds *dsi2lvds = to_dsi2lvds_panel(panel);

	if (dsi2lvds->adapter) {
		struct edid *edid = drm_get_edid(panel->connector,
						dsi2lvds->adapter);
		drm_mode_connector_update_edid_property(panel->connector,
						edid);

		if (edid) {
			num += drm_add_edid_modes(panel->connector, edid);
			kfree(edid);
		}
	}

	return num;
#else
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &auo_b101xtn01_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
		auo_b101xtn01_mode.hdisplay, auo_b101xtn01_mode.vdisplay,
			auo_b101xtn01_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm = 64;
	panel->connector->display_info.height_mm = 114;

	return 1;
#endif
}

static const struct drm_panel_funcs dsi2lvds_funcs = {
		.disable = dsi2lvds_disable,
		.unprepare = dsi2lvds_unprepare,
		.prepare = dsi2lvds_prepare,
		.enable = dsi2lvds_enable,
		.get_modes = dsi2lvds_get_modes,
};

static const struct of_device_id dsi2lvds_of_match[] = {
		{ .compatible = "toshiba, tc358775", },
		{ }
};
MODULE_DEVICE_TABLE(of, dsi2lvds_of_match);

static int dsi2lvds_add(struct dsi2lvds *dsi2lvds)
{
	struct device *dev = &dsi2lvds->dsi->dev;
	int ret;
#ifdef PWM_BACKLIGHT
	struct device_node *np;
#endif

#ifndef LVDS_EDID_PANEL
	dsi2lvds->mode = &auo_b101xtn01_mode;
#endif
	dsi2lvds->dsi_sw_select_gpio = devm_gpiod_get(dev,
				"dsi-sw-select", GPIOD_OUT_LOW);
	if (IS_ERR(dsi2lvds->dsi_sw_select_gpio)) {
		dev_err(dev, "cannot get dsi-sw-select %ld\n",
			PTR_ERR(dsi2lvds->dsi_sw_select_gpio));
		dsi2lvds->dsi_sw_select_gpio = NULL;
	} else {
		gpiod_direction_output(dsi2lvds->dsi_sw_select_gpio, 0);
	}

	dsi2lvds->stby_gpio = devm_gpiod_get(dev, "stby", GPIOD_OUT_LOW);
	if (IS_ERR(dsi2lvds->stby_gpio)) {
		dev_err(dev, "cannot get stby-gpio %ld\n",
			PTR_ERR(dsi2lvds->stby_gpio));
		dsi2lvds->stby_gpio = NULL;
	} else {
		gpiod_direction_output(dsi2lvds->stby_gpio, 0);
	}

	dsi2lvds->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(dsi2lvds->reset_gpio)) {
		dev_err(dev, "cannot get reset-gpios %ld\n",
			PTR_ERR(dsi2lvds->reset_gpio));
		dsi2lvds->reset_gpio = NULL;
	} else {
		gpiod_direction_output(dsi2lvds->reset_gpio, 0);
	}

#ifdef PWM_BACKLIGHT
	np = of_parse_phandle(dev->of_node, "backlight", 0);
	if (np) {
		dsi2lvds->backlight = of_find_backlight_by_node(np);
		of_node_put(np);

		if (!dsi2lvds->backlight)
			return -EPROBE_DEFER;
	}
#endif
	drm_panel_init(&dsi2lvds->base);
	dsi2lvds->base.funcs = &dsi2lvds_funcs;
	dsi2lvds->base.dev = &dsi2lvds->dsi->dev;

	ret = drm_panel_add(&dsi2lvds->base);
	if (ret < 0)
		goto put_backlight;

	return 0;

put_backlight:
	if (dsi2lvds->backlight)
		put_device(&dsi2lvds->backlight->dev);

	return ret;
}

static void dsi2lvds_del(struct dsi2lvds *dsi2lvds)
{
	if (dsi2lvds->base.dev)
		drm_panel_remove(&dsi2lvds->base);

}

static int dsi2lvds_probe(struct mipi_dsi_device *dsi)
{
	struct dsi2lvds *dsi2lvds;
	struct device_node *adapter_node;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO;

	dsi2lvds = devm_kzalloc(&dsi->dev, sizeof(*dsi2lvds), GFP_KERNEL);
	if (!dsi2lvds)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, dsi2lvds);

	dsi2lvds->dsi = dsi;

	adapter_node = of_parse_phandle(dev->of_node, "i2c-bus", 0);
	if (adapter_node) {
		dsi2lvds->adapter =
			of_find_i2c_adapter_by_node(adapter_node);
		if (dsi2lvds->adapter == NULL)
			return -ENODEV;

		snprintf(dsi2lvds->adapter->name,
			sizeof(dsi2lvds->adapter->name), "dsi2lvds i2c3");

#ifdef TC358775XBG_USE_I2C
		dsi2lvds->i2c_main = i2c_new_dummy(dsi2lvds->adapter, 0x0F);

		if (!dsi2lvds->i2c_main)
			return -ENOMEM;

		dsi2lvds->regmap = devm_regmap_init_i2c(
					dsi2lvds->i2c_main,
					&dsi2lvds_regmap_config);

		if (IS_ERR(dsi2lvds->regmap))
			return PTR_ERR(dsi2lvds->regmap);
#endif
	} else
		return -ENODEV;

	ret = dsi2lvds_add(dsi2lvds);
	if (ret < 0)
		return ret;

	return  mipi_dsi_attach(dsi);
}

static int dsi2lvds_remove(struct mipi_dsi_device *dsi)
{
	struct dsi2lvds *dsi2lvds = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = dsi2lvds_disable(&dsi2lvds->base);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to disable panel: %d\n", ret);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_detach(&dsi2lvds->base);
	dsi2lvds_del(dsi2lvds);

	return 0;
}

static void dsi2lvds_shutdown(struct mipi_dsi_device *dsi)
{
	struct dsi2lvds *dsi2lvds = mipi_dsi_get_drvdata(dsi);

	dsi2lvds_disable(&dsi2lvds->base);
}

static struct mipi_dsi_driver dsi2lvds_driver = {
	.driver = {
		.name = "dsi2lvds",
		.of_match_table = dsi2lvds_of_match,
	},
	.probe = dsi2lvds_probe,
	.remove = dsi2lvds_remove,
	.shutdown = dsi2lvds_shutdown,
};
module_mipi_dsi_driver(dsi2lvds_driver);

MODULE_AUTHOR("Vinay Simha BN <vinaysimha@inforcecomputing.com>");
MODULE_DESCRIPTION("Toshiba MIPI-DSI-to-LVDS TC358775 bridge driver");
MODULE_LICENSE("GPL v2");
