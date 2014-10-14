/*
 * Copyright (C) 2014
 * Author: Vinay Simha <vinaysimha@inforcecomputing.com>
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

#include <linux/clk.h>
#include <linux/clk-provider.h>

#include "mdp4_kms.h"

/* *********************************************************************
 * backwards compat:
 */

struct clk_hw {
	uint32_t dummy;
};

/* ********************************************************************* */

struct mdp4_dsi_pll {
	struct clk_hw pll_hw;
	struct drm_device *dev;
	unsigned long pixclk;
};
#define to_mdp4_dsi_pll(x) container_of(x, struct mdp4_dsi_pll, pll_hw)

static struct mdp4_kms *get_kms(struct mdp4_dsi_pll *dsi_pll)
{
	struct msm_drm_private *priv = dsi_pll->dev->dev_private;
	return to_mdp4_kms(to_mdp_kms(priv->kms));
}

struct pll_rate {
	unsigned long rate;
	struct {
		uint32_t val;
		uint32_t reg;
	} conf[45];
};

/* NOTE: keep sorted highest freq to lowest: */
static const struct pll_rate freqtbl[] = {
	{ 343000000, {
		{ 0x03, REG_DSI_8960_PHY_REGULATOR_CTRL_0 },/* regulator */
		{ 0x0a, REG_DSI_8960_PHY_REGULATOR_CTRL_1 },
		{ 0x04, REG_DSI_8960_PHY_REGULATOR_CTRL_2 },
		{ 0x00, REG_DSI_8960_PHY_REGULATOR_CTRL_3 },
		{ 0x67, REG_DSI_8960_PHY_TIMING_CTRL_0 },
		{ 0x16, REG_DSI_8960_PHY_TIMING_CTRL_1 },
		{ 0x0e, REG_DSI_8960_PHY_TIMING_CTRL_2 },
		{ 0x00, REG_DSI_8960_PHY_TIMING_CTRL_3 },
		{ 0x38, REG_DSI_8960_PHY_TIMING_CTRL_4 },
		{ 0x3c, REG_DSI_8960_PHY_TIMING_CTRL_5 },
		{ 0x12, REG_DSI_8960_PHY_TIMING_CTRL_6 },
		{ 0x19, REG_DSI_8960_PHY_TIMING_CTRL_7 },
		{ 0x18, REG_DSI_8960_PHY_TIMING_CTRL_8 },
		{ 0x03, REG_DSI_8960_PHY_TIMING_CTRL_9 },
		{ 0x04, REG_DSI_8960_PHY_TIMING_CTRL_10 },
		{ 0xa0, REG_DSI_8960_PHY_TIMING_CTRL_11 },
		{ 0x5f, REG_DSI_8960_PHY_CTRL_0 }, /* phy ctrl */
		{ 0x00, REG_DSI_8960_PHY_CTRL_1 },
		{ 0x00, REG_DSI_8960_PHY_CTRL_2 },
		{ 0xff, REG_DSI_8960_PHY_STRENGTH_0 }, /* strength*/
		{ 0x00, REG_DSI_8960_PHY_STRENGTH_1 },
		{ 0x06, REG_DSI_8960_PHY_STRENGTH_2 },
		{ 0x00, REG_DSI_PHY_PLL_CTRL_0 }, /*pll control*/
		{ 0x56, REG_DSI_PHY_PLL_CTRL_1 },
		{ 0x31, REG_DSI_PHY_PLL_CTRL_2 },
		{ 0xda, REG_DSI_PHY_PLL_CTRL_3 },
		{ 0x4a, REG_DSI_PHY_PLL_CTRL_4 },
		{ 0x01, REG_DSI_PHY_PLL_CTRL_5 },
		{ 0x19, REG_DSI_PHY_PLL_CTRL_6 },
		{ 0x62, REG_DSI_PHY_PLL_CTRL_7 },
		{ 0x71, REG_DSI_PHY_PLL_CTRL_8 },
		{ 0x0f, REG_DSI_PHY_PLL_CTRL_9 },
		{ 0x07, REG_DSI_PHY_PLL_CTRL_10 },
		{ 0x00, REG_DSI_PHY_PLL_CTRL_11 },
		{ 0x14, REG_DSI_PHY_PLL_CTRL_12 },
		{ 0x03, REG_DSI_PHY_PLL_CTRL_13 },
		{ 0x00, REG_DSI_PHY_PLL_CTRL_14 },
		{ 0x02, REG_DSI_PHY_PLL_CTRL_15 },
		{ 0x00, REG_DSI_PHY_PLL_CTRL_16 },
		{ 0x20, REG_DSI_PHY_PLL_CTRL_17 },
		{ 0x00, REG_DSI_PHY_PLL_CTRL_18 },
		{ 0x01, REG_DSI_PHY_PLL_CTRL_19 },
		{ 0, 0 } }
	},
};

static const struct pll_rate *find_rate(unsigned long rate)
{
	int i;
	for (i = 1; i < ARRAY_SIZE(freqtbl); i++)
		if (rate > freqtbl[i].rate)
			return &freqtbl[i-1];
	return &freqtbl[i-1];
}

int mpd4_dsi_pll_enable(struct clk_hw *hw)
{
	struct mdp4_dsi_pll *dsi_pll = to_mdp4_dsi_pll(hw);
	struct mdp4_kms *mdp4_kms = get_kms(dsi_pll);
	const struct pll_rate *pll_rate = find_rate(dsi_pll->pixclk);
	int i;

	DBG("pixclk=%lu (%lu)", dsi_pll->pixclk, pll_rate->rate);

	if (WARN_ON(!pll_rate))
		return -EINVAL;

	mdp4_write(mdp4_kms, REG_MDP4_LCDC_LVDS_PHY_RESET, 0x33);

	for (i = 0; pll_rate->conf[i].reg; i++)
		mdp4_write(mdp4_kms, pll_rate->conf[i].reg, pll_rate->conf[i].val);

	mdp4_write(mdp4_kms, REG_DSI_8960_PHY_TIMING_CTRL_0, 0x01);

	/* Wait until LVDS PLL is locked and ready */
//	while (!mdp4_read(mdp4_kms, REG_DSI_PHY_PLL_LOCKED))
//		cpu_relax();

	return 0;
}

void mpd4_dsi_pll_disable(struct clk_hw *hw)
{
	struct mdp4_dsi_pll *dsi_pll = to_mdp4_dsi_pll(hw);
	struct mdp4_kms *mdp4_kms = get_kms(dsi_pll);

	DBG("");

	//mdp4_write(mdp4_kms, REG_DSI_PHY_CFG0, 0x0);
	mdp4_write(mdp4_kms, REG_DSI_8960_PHY_TIMING_CTRL_0, 0x0);
}

long mpd4_dsi_pll_round_rate(struct clk_hw *hw, unsigned long rate)
{
	const struct pll_rate *pll_rate = find_rate(rate);
	return pll_rate->rate;
}

int mpd4_dsi_pll_set_rate(struct clk_hw *hw, unsigned long rate)
{
	struct mdp4_dsi_pll *dsi_pll = to_mdp4_dsi_pll(hw);
	dsi_pll->pixclk = rate;
	return 0;
}

struct clk_hw *mpd4_dsi_pll_init(struct drm_device *dev)
{
	struct mdp4_dsi_pll *dsi_pll;
	int ret;

	dsi_pll = devm_kzalloc(dev->dev, sizeof(*dsi_pll), GFP_KERNEL);
	if (!dsi_pll) {
		ret = -ENOMEM;
		goto fail;
	}

	dsi_pll->dev = dev;

	return &dsi_pll->pll_hw;

fail:
	return ERR_PTR(ret);
}
