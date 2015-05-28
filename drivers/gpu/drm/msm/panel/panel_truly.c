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
#include <linux/gpio.h>
#include <linux/regulator/msm-gpio-regulator.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/mfd/pm8xxx/pm8821.h>

#include "panel.h"

#define PM8921_GPIO_BASE                NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)
#define PM8921_MPP_BASE                 (PM8921_GPIO_BASE + PM8921_NR_GPIOS)
#define PM8921_MPP_PM_TO_SYS(pm_mpp)    (pm_mpp - 1 + PM8921_MPP_BASE)
#define PM8921_IRQ_BASE                 (NR_MSM_IRQS + NR_GPIO_IRQS)

#define PM8821_MPP_BASE                 (PM8921_MPP_BASE + PM8921_NR_MPPS)
#define PM8821_MPP_PM_TO_SYS(pm_mpp)    (pm_mpp - 1 + PM8821_MPP_BASE)

struct panel_truly {
	struct panel base;
	struct mipi_adapter *mipi;
	struct regulator *reg_l8_avdd;
	struct regulator *reg_s4_iovdd;
	struct regulator *reg_l16;
	int pmic8821_mpp2;
};
#define to_panel_truly(x) container_of(x, struct panel_truly, base)

static char enter_sleep[2] = {0x10, 0x00};
/*************************** IC for AUO 4' MIPI * 480RGBx854************/
static char write_memory1[4]={0xFF,0x80,0x09,0x01};//Enable EXTC
static char write_memory2[2]={0x00,0x80};//Shift address
static char write_memory3[3]={0xFF,0x80,0x09};	 //Enable Orise mode

static char write_memory4[2]={0x00,0x00};
static char write_memory5[2]={0xD8,0x87};//GVDD	4.8V
static char write_memory6[2]={0x00,0x01};
static char write_memory7[2]={0xD8,0x87};//NGVDD -4.8V
static char write_memory8[2]={0x00,0xB1};
static char write_memory9[2]={0xC5,0xA9};//[0]GVDD output Enable : 0xA9	VDD_18V=LVDSVDD=1.8V
static char write_memory10[2]={0x00,0x91};
static char write_memory11[2]={0xC5,0x79};//[7:4]VGH_level=15V [3:0]VGL_level=-12V
static char write_memory12[2]={0x00,0x00};
static char write_memory13[2]={0xD9,0x45};//VCOM=-1.15V

static char write_memory14[2]={0x00,0x92};
static char write_memory15[2]={0xC5,0x01};//pump45

static char write_memory16[2]={0x00,0xA1};
static char write_memory17[2]={0xC1,0x08};//reg_oscref_rgb_vs_video

static char write_memory18[2]={0x00,0x81};
static char write_memory19[2]={0xC1,0x66};//OSC Adj=65Hz

static char write_memory20[2]={0x00,0xa3};
static char write_memory21[2]={0xc0,0x1B};//source pch

static char write_memory22[2]={0x00,0x82};
static char write_memory23[2]={0xc5,0x83};//REG-Pump23 AVEE VCL

static char write_memory24[2]={0x00,0x81};
static char write_memory25[2]={0xc4,0x83};			 //source bias

static char write_memory26[2]={0x00,0x90};
static char write_memory27[2]={0xB3,0x02};			 //SW_GM 480X854

static char write_memory29[2]={0x00,0x92};
static char write_memory30[2]={0xB3,0x45};			 //Enable SW_GM

static char write_memory31[2]={0x00,0xa0};
static char write_memory32[2]={0xc1,0xea};

static char write_memory33[2]={0x00,0xc0};
static char write_memory34[2]={0xc5,0x00};

static char write_memory35[2]={0x00,0x8b};
static char write_memory36[2]={0xb0,0x40};

static char write_memory37[2]={0x00,0x87};
static char write_memory38[4]={0xC4,0x00,0x80,0x00};

static char write_memory39[2]={0x00,0xB2};
static char write_memory40[5]={0xF5,0x15,0x00,0x15,0x00}; //VRGH Disable

static char write_memory41[2]={0x00,0x93};
static char write_memory42[2]={0xC5,0x03}; //VRGH minimum
///////////////////////////////////////////////////////////////////////////////
static char write_memory43[2]={0x00,0xa7};
static char write_memory44[2]={0xb3,0x01};          //panel_set[0]	= 1

static char write_memory45[2]={0x00,0xa6};
static char write_memory46[2]={0xb3,0x2b};          //reg_panel_zinv,reg_panel_zinv_pixel,reg_panel_zinv_odd,reg_panel_zigzag,reg_panel_zigzag_blue,reg_panel_zigzag_shift_r,reg_panel_zigzag_odd

//C09x : mck_shift1/mck_shift2/mck_shift3
static char write_memory47[2]={0x00,0x90};
static char write_memory48[7]={0xC0,0x00,0x4E,0x00,0x00,0x00,0x03};

//C0Ax : hs_shift/vs_shift

static char write_memory49[2]={0x00,0xa6};
static char write_memory50[4]={0xC1,0x01,0x00,0x00}; // 16'hc1a7 [7:0] : oscref_vedio_hs_shift[7:0]

//Gamma2.2 +/-
static char write_memory51[2]={0x00,0x00};
static char write_memory52[17]={0xE1,0x05,0x0B,0x0F,0x0F,0x08,0x0D,0x0C,0x0B,0x02,0x06,0x16,0x12,0x18,0x24,0x17,0x00};
//V255 V251 V247 V239 V231 V203 V175 V147 V108 V80  V52	 V24  V16  V8   V4   V0

static char write_memory53[2]={0x00,0x00};
static char write_memory54[17]={0xE2,0x05,0x0B,0x0F,0x0F,0x08,0x0D,0x0C,0x0B,0x02,0x06,0x16,0x12,0x18,0x24,0x17,0x00};
//V255 V251 V247 V239 V231 V203 V175 V147 V108 V80  V52	 V24  V16  V8   V4   V0

//--------------------------------------------------------------------------------
//		initial setting 2 < tcon_goa_wave >
//--------------------------------------------------------------------------------
static char write_memory55[2]={0x00,0x91};         //zigzag reverse scan
static char write_memory56[2]={0xB3,0x00};

//CE8x : vst1, vst2, vst3, vst4
static char write_memory57[2]={0x00, 0x80};
static char write_memory58[13]={0xCE,0x85,0x01,0x18,0x84,0x01,0x18,0x00,0x00,0x00,0x00,0x00,0x00};

//CE9x : vend1, vend2, vend3, vend4
static char write_memory59[2]={0x00, 0x90};
static char write_memory60[15]={0xCE,0x13,0x56,0x18,0x13,0x57,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

//CEAx : clka1, clka2
static char write_memory61[2]={0x00, 0xa0};
static char write_memory62[15]={0xCE,0x18,0x0B,0x03,0x5E,0x00,0x18,0x00,0x18,0x0A,0x03,0x5F,0x00,0x18,0x00};

//CEBx : clka3, clka4
static char write_memory63[2]={0x00, 0xb0};
static char write_memory64[15]={0xCE,0x18,0x0D,0x03,0x5C,0x00,0x18,0x00,0x18,0x0C,0x03,0x5D,0x00,0x18,0x00};

//CECx : clkb1, clkb2
static char write_memory65[2]={0x00, 0xc0};
static char write_memory66[15]={0xCE,0x38,0x0D,0x03,0x5E,0x00,0x10,0x07,0x38,0x0C,0x03,0x5F,0x00,0x10,0x07};

//CEDx : clkb3, clkb4
static char write_memory67[2]={0x00, 0xd0};
static char write_memory68[15]={0xCE,0x38,0x09,0x03,0x5A,0x00,0x10,0x07,0x38,0x08,0x03,0x5B,0x00,0x10,0x07};

//CFCx :
static char write_memory69[2]={0x00, 0xC7};
static char write_memory70[2]={0xCF, 0x04};

static char write_memory71[2]={0x00, 0xC9};
static char write_memory72[2]={0xCF, 0x00};

//--------------------------------------------------------------------------------
//		initial setting 3 < Panel setting >
//--------------------------------------------------------------------------------
// cbcx
static char write_memory73[2]={0x00, 0xC0};
static char write_memory74[2]={0xCB, 0x14};

static char write_memory75[2]={0x00, 0xC2};
static char write_memory76[6]={0xCB, 0x14,0x14,0x14,0x14,0x14};

// cbdx
static char write_memory77[2]={0x00, 0xD5};
static char write_memory78[2]={0xCB, 0x14};

static char write_memory79[2]={0x00, 0xD7};
static char write_memory80[6]={0xCB, 0x14,0x14,0x14,0x14,0x14};

// cc8x
static char write_memory81[2]={0x00, 0x80};
static char write_memory82[2]={0xCC, 0x01};

static char write_memory83[2]={0x00, 0x82};
static char write_memory84[6]={0xCC, 0x0F,0x0D,0x0B,0x09,0x05};

// cc9x
static char write_memory85[2]={0x00, 0x9A};
static char write_memory86[2]={0xCC, 0x02};

static char write_memory87[2]={0x00, 0x9C};
static char write_memory88[4]={0xCC, 0x10,0x0E,0x0C};

// ccax
static char write_memory89[2]={0x00, 0xA0};
static char write_memory90[2]={0xCC, 0x0A};

static char write_memory91[2]={0x00, 0xA1};
static char write_memory92[2]={0xCC, 0x06};

// ccbx
static char write_memory93[2]={0x00, 0xB0};
static char write_memory94[2]={0xCC, 0x01};

static char write_memory95[2]={0x00, 0xB2};
static char write_memory96[6]={0xCC, 0x0F,0x0D,0x0B,0x09,0x05};

// cccx
static char write_memory97[2]={0x00, 0xCA};
static char write_memory98[2]={0xCC, 0x02};

static char write_memory99[2]={0x00, 0xCC};
static char write_memory100[4]={0xCC, 0x10,0x0E,0x0C};

// ccdx
static char write_memory101[2]={0x00, 0xD0};
static char write_memory102[2]={0xCC, 0x0A};

static char write_memory103[2]={0x00, 0xD1};
static char write_memory104[2]={0xCC, 0x06};
//-------------------- sleep out --------------------//
static char write_memory105[1]={0x11};
//delay1m{200};

static char write_memory106[1]={0x29};

static void panel_truly_destroy(struct panel *panel)
{
	struct panel_truly *panel_truly = to_panel_truly(panel);
	kfree(panel_truly);
}

static int panel_truly_power_on(struct panel *panel)
{
	struct drm_device *dev = panel->dev;
	struct panel_truly *panel_truly = to_panel_truly(panel);
	int ret = 0;

	ret = regulator_set_optimum_mode(panel_truly->reg_l8_avdd, 110000);
	if (ret < 0) {
		dev_err(dev->dev, "failed to set l8 mode: %d\n", ret);
		goto fail1;
	}

	ret = regulator_set_optimum_mode(panel_truly->reg_s4_iovdd, 100000);
	if (ret < 0) {
		dev_err(dev->dev, "failed to set s4 mode: %d\n", ret);
		goto fail2;
	}

	ret = regulator_enable(panel_truly->reg_l8_avdd);
	if (ret) {
		dev_err(dev->dev, "failed to enable l8: %d\n", ret);
		goto fail1;
	}

	ret = regulator_enable(panel_truly->reg_l16);
        if (ret) {
                dev_err(dev->dev, "failed to enable l8: %d\n", ret);
                goto fail1;
        }

	udelay(100);

	ret = regulator_enable(panel_truly->reg_s4_iovdd);
	if (ret) {
		dev_err(dev->dev, "failed to enable s4: %d\n", ret);
		goto fail2;
	}
	ret = regulator_enable(panel_truly->reg_l16);
	if (ret) {
		dev_err(dev->dev, "failed to enable s4: %d\n", ret);
		goto fail2;
	}
//	mdelay(2);
	gpio_set_value_cansleep(panel_truly->pmic8821_mpp2, 1);
        mdelay(1);/*Reset display 1 ms*/
        gpio_set_value_cansleep(panel_truly->pmic8821_mpp2, 0);
        usleep(50);
        gpio_set_value_cansleep(panel_truly->pmic8821_mpp2, 1);
        mdelay(5);

	return 0;

fail2:
	regulator_disable(panel_truly->reg_s4_iovdd);
fail1:
	regulator_disable(panel_truly->reg_l8_avdd);
	
	return ret;
}

static int panel_truly_power_off(struct panel *panel)
{
	struct drm_device *dev = panel->dev;
	struct panel_truly *panel_truly = to_panel_truly(panel);
	int ret;

	gpio_set_value_cansleep(panel_truly->pmic8821_mpp2, 0);
	udelay(100);

	ret = regulator_disable(panel_truly->reg_l8_avdd);
	if (ret)
		dev_err(dev->dev, "failed to disable l8: %d\n", ret);

	udelay(100);

	ret = regulator_disable(panel_truly->reg_s4_iovdd);
	if (ret)
		dev_err(dev->dev, "failed to disable s4: %d\n", ret);

	return 0;
}

static int panel_truly_on(struct panel *panel)
{
	struct panel_truly *panel_truly = to_panel_truly(panel);
	struct mipi_adapter *mipi = panel_truly->mipi;
	int ret = 0;

	DRM_DEBUG_KMS("panel on\n");

	ret = panel_truly_power_on(panel);
	if (ret)
		return ret;

	mipi_set_panel_config(mipi, &(struct mipi_panel_config){
		.cmd_mode = false,
		.format = DST_FORMAT_RGB888,
		.traffic_mode = NON_BURST_SYNCH_EVENT,
		.bllp_power_stop = true,
		.eof_bllp_power_stop = true,
		.hsa_power_stop = false,
		.hbp_power_stop = true,
		.hfp_power_stop = true,
		.pulse_mode_hsa_he = true,
		.rgb_swap = SWAP_RGB,
		.interleave_max = 0,
		.dma_trigger = TRIGGER_SW,
		.mdp_trigger = TRIGGER_NONE,
		.te = false,
		.dlane_swap = 0,
		.t_clk_pre = 0x1c,
		.t_clk_post = 0x04,
		.rx_eot_ignore = false,
		.tx_eot_append = true,
		.ecc_check = false,
		.crc_check = false,
		.phy = {
			/* regulator */
			{0x03, 0x0a, 0x04, 0x00, 0x20},
		        /* timing   */
		        {0x67, 0x16, 0x0e, 0x00, 0x38, 0x3c, 0x12, 0x19,
			0x18, 0x03, 0x04, 0xa0},
			/* phy ctrl */
		        {0x5f, 0x00, 0x00, 0x10},
		        /* strength */
		        {0xff, 0x00, 0x06, 0x00},
		        /* pll control */
		        /*{0x00, 0x46, 0x30, 0xc4, 0x4a, 0x01, 0x19, 0x62, 0x71, 0x0f, 0x01,
	                0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01},*/
		        {0x00, /*common 8064*/
			 0x56, 0x31, 0xda, /* panel specific */
			 0x4a, 0x01, 0x19, 0x62, /*panel specific */
			 0x71, 0x0f, 0x07, 
			 0x00, 0x14, 0x03, 0x00, 0x02, /*common 8064*/
			 0x00, 0x20, 0x00, 0x01}, /* common 8064*/
		},
	});


	mipi_on(mipi);
	/*Initial setting in LP mode*/
	mipi_set_bus_config(mipi, &(struct mipi_bus_config){
		.low_power = true,
		.lanes = 0x3,
	});

	mipi->wait=5;
	mipi_lwrite(mipi, true, 0, write_memory1);
	mipi->wait=0;
	mipi_gen_write(mipi, true, 0, write_memory2);
	mipi_lwrite(mipi, true, 0, write_memory3);
	mipi_gen_write(mipi, true, 0, write_memory4);
	mipi_gen_write(mipi, true, 0, write_memory5);
	mipi_gen_write(mipi, true, 0, write_memory6);
	mipi_gen_write(mipi, true, 0, write_memory7);
	mipi_gen_write(mipi, true, 0, write_memory8);
	mipi_gen_write(mipi, true, 0, write_memory9);
	mipi_gen_write(mipi, true, 0, write_memory10);
	mipi_gen_write(mipi, true, 0, write_memory11);
	mipi_gen_write(mipi, true, 0, write_memory12);
	mipi_gen_write(mipi, true, 0, write_memory13);
	mipi_gen_write(mipi, true, 0, write_memory14);
	mipi_gen_write(mipi, true, 0, write_memory15);
	mipi_gen_write(mipi, true, 0, write_memory16);
	mipi_gen_write(mipi, true, 0, write_memory17);
	mipi_gen_write(mipi, true, 0, write_memory18);
	mipi_gen_write(mipi, true, 0, write_memory19);
	mipi_gen_write(mipi, true, 0, write_memory20);
	mipi_gen_write(mipi, true, 0, write_memory21);
	mipi_gen_write(mipi, true, 0, write_memory22);
	mipi_gen_write(mipi, true, 0, write_memory23);
	mipi_gen_write(mipi, true, 0, write_memory24);
	mipi_gen_write(mipi, true, 0, write_memory25);
	mipi_gen_write(mipi, true, 0, write_memory26);
	mipi_gen_write(mipi, true, 0, write_memory27);
	mipi_gen_write(mipi, true, 0, write_memory29);
	mipi_gen_write(mipi, true, 0, write_memory30);
	mipi_gen_write(mipi, true, 0, write_memory31);
	mipi_gen_write(mipi, true, 0, write_memory32);
	mipi_gen_write(mipi, true, 0, write_memory33);
	mipi_gen_write(mipi, true, 0, write_memory34);
	mipi_gen_write(mipi, true, 0, write_memory35);
	mipi_gen_write(mipi, true, 0, write_memory36);
	mipi_gen_write(mipi, true, 0, write_memory37);
	mipi_lwrite(mipi, true, 0, write_memory38);
	mipi_gen_write(mipi, true, 0, write_memory39);
	mipi_lwrite(mipi, true, 0, write_memory40);
	mipi_gen_write(mipi, true, 0, write_memory41);
	mipi_gen_write(mipi, true, 0, write_memory42);
	mipi_gen_write(mipi, true, 0, write_memory43);
	mipi_gen_write(mipi, true, 0, write_memory44);
	mipi_gen_write(mipi, true, 0, write_memory45);
	mipi_gen_write(mipi, true, 0, write_memory46);
	mipi_gen_write(mipi, true, 0, write_memory47);
	mipi_lwrite(mipi, true, 0, write_memory48);
	mipi_gen_write(mipi, true, 0, write_memory49);
	mipi_lwrite(mipi, true, 0, write_memory50);
	mipi_gen_write(mipi, true, 0, write_memory51);
	mipi_lwrite(mipi, true, 0, write_memory52);
	mipi_gen_write(mipi, true, 0, write_memory53);
	mipi_lwrite(mipi, true, 0, write_memory54);
	mipi_gen_write(mipi, true, 0, write_memory55);
	mipi_gen_write(mipi, true, 0, write_memory56);
	mipi_gen_write(mipi, true, 0, write_memory57);
	mipi_lwrite(mipi, true, 0, write_memory58);
	mipi_gen_write(mipi, true, 0, write_memory59);
	mipi_lwrite(mipi, true, 0, write_memory60);
	mipi_gen_write(mipi, true, 0, write_memory61);
	mipi_lwrite(mipi, true, 0, write_memory62);
	mipi_gen_write(mipi, true, 0, write_memory63);
	mipi_lwrite(mipi, true, 0, write_memory64);
	mipi_gen_write(mipi, true, 0, write_memory65);
	mipi_lwrite(mipi, true, 0, write_memory66);
	mipi_gen_write(mipi, true, 0, write_memory67);
	mipi_lwrite(mipi, true, 0, write_memory68);
	mipi_gen_write(mipi, true, 0, write_memory69);
	mipi_gen_write(mipi, true, 0, write_memory70);
	mipi_gen_write(mipi, true, 0, write_memory71);
	mipi_gen_write(mipi, true, 0, write_memory72);
	mipi_gen_write(mipi, true, 0, write_memory73);
	mipi_gen_write(mipi, true, 0, write_memory74);
	mipi_gen_write(mipi, true, 0, write_memory75);
	mipi_lwrite(mipi, true, 0, write_memory76);
	mipi_gen_write(mipi, true, 0, write_memory77);
	mipi_gen_write(mipi, true, 0, write_memory78);
	mipi_gen_write(mipi, true, 0, write_memory79);
	mipi_lwrite(mipi, true, 0, write_memory80);
	mipi_gen_write(mipi, true, 0, write_memory81);
	mipi_gen_write(mipi, true, 0, write_memory82);
	mipi_gen_write(mipi, true, 0, write_memory83);
	mipi_lwrite(mipi, true, 0, write_memory84);
	mipi_gen_write(mipi, true, 0, write_memory85);
	mipi_gen_write(mipi, true, 0, write_memory86);
	mipi_gen_write(mipi, true, 0, write_memory87);
	mipi_lwrite(mipi, true, 0, write_memory88);
	mipi_gen_write(mipi, true, 0, write_memory89);
	mipi_gen_write(mipi, true, 0, write_memory90);
	mipi_gen_write(mipi, true, 0, write_memory91);
	mipi_gen_write(mipi, true, 0, write_memory92);
	mipi_gen_write(mipi, true, 0, write_memory93);
	mipi_gen_write(mipi, true, 0, write_memory94);
	mipi_gen_write(mipi, true, 0, write_memory95);
	mipi_lwrite(mipi, true, 0, write_memory96);
	mipi_gen_write(mipi, true, 0, write_memory97);
	mipi_gen_write(mipi, true, 0, write_memory98);
	mipi_gen_write(mipi, true, 0, write_memory99);
	mipi_lwrite(mipi, true, 0, write_memory100);
	mipi_gen_write(mipi, true, 0, write_memory101);
	mipi_gen_write(mipi, true, 0, write_memory102);
	mipi_gen_write(mipi, true, 0, write_memory103);
	mipi_gen_write(mipi, true, 0, write_memory104);
	
	mipi_set_bus_config(mipi, &(struct mipi_bus_config){
		.low_power = false,
		.lanes = 0x3,
	});

	mipi->wait=10;
	mipi_dcs_swrite(mipi, true, 0, false,write_memory105[0]); 
	mipi->wait=200;
        mipi_dcs_swrite(mipi, true, 0, false, write_memory106[0]);
        mdelay(10);

	return 0;
}

static int panel_truly_off(struct panel *panel)
{
	struct panel_truly *panel_truly = to_panel_truly(panel);
	struct mipi_adapter *mipi = panel_truly->mipi;
	int ret;

	DRM_DEBUG_KMS("panel off\n");
	mipi_set_bus_config(mipi, &(struct mipi_bus_config){
		.low_power = true,
		.lanes = 0x3,
	});

	mipi_dcs_swrite(mipi, true, 0, false, enter_sleep[0]);
	mdelay(5);

	mipi_off(mipi);

	ret = panel_truly_power_off(panel);
	if (ret)
		return ret;

	return 0;
}

static struct drm_display_mode *panel_truly_mode(struct panel *panel)
{
	struct drm_display_mode *mode = drm_mode_create(panel->dev);
	u32 hbp, hfp, vbp, vfp, hspw, vspw;

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

	return mode;
}

static const struct panel_funcs panel_truly_funcs = {
		.destroy = panel_truly_destroy,
		.on = panel_truly_on,
		.off = panel_truly_off,
		.mode = panel_truly_mode,
};

struct panel *panel_truly_init(struct drm_device *dev,
		// XXX uggg.. maybe we should just pass in a config structure
		// pre-populated with regulators, gpio's, etc??  the panel
		// needs the drm device, but we need the pdev to lookup the
		// regulators, etc, currently.. and for msm the pdev is different
		// device from the drm device..
		struct platform_device *pdev,
		struct mipi_adapter *mipi)
{
	struct panel_truly *panel_truly;
	struct panel *panel = NULL;
	int ret;

	panel_truly = kzalloc(sizeof(*panel_truly), GFP_KERNEL);
	if (!panel_truly) {
		ret = -ENOMEM;
		goto fail;
	}

	panel_truly->mipi = mipi;

	panel = &panel_truly->base;
	ret = panel_init(dev, panel, &panel_truly_funcs);
	if (ret)
		goto fail;

	/* Maybe GPIO/regulator/etc stuff should come from DT or similar..
	 * but we can sort that out when there is some other device that
	 * uses the same panel.
	 */

	panel_truly->pmic8821_mpp2 = PM8821_MPP_PM_TO_SYS(2);
	ret = gpio_request(panel_truly->pmic8821_mpp2, "disp_rst_n");
	if (ret) {
		dev_err(dev->dev, "failed to request disp_rst_n mpp: %d\n", ret);
		goto fail;
	}
	ret = gpio_export(panel_truly->pmic8821_mpp2, true);
	if (ret) {
		dev_err(dev->dev, "failed to request gpio export mpp: %d\n", ret);
		goto fail;
	}
	ret = gpio_direction_output(panel_truly->pmic8821_mpp2, 0);
	if (ret) {
		dev_err(dev->dev, "failed to request gpio direction output mpp: %d\n", ret);
		goto fail;
	}
	panel_truly->reg_l8_avdd = devm_regulator_get(dev->dev, "dsi1_avdd");
	if (IS_ERR(panel_truly->reg_l8_avdd)) {
		ret = PTR_ERR(panel_truly->reg_l8_avdd);
		dev_err(dev->dev, "failed to request dsi_avdd regulator: %d\n", ret);
		goto fail;
	}

	panel_truly->reg_s4_iovdd = devm_regulator_get(dev->dev, "dsi1_s4_iovdd");
	if (IS_ERR(panel_truly->reg_s4_iovdd)) {
		ret = PTR_ERR(panel_truly->reg_s4_iovdd);
		dev_err(dev->dev, "failed to request dsi_s4_iovdd regulator: %d\n", ret);
		goto fail;
	}

	panel_truly->reg_l16 = devm_regulator_get(dev->dev, "dsi1_l16");
	if (IS_ERR(panel_truly->reg_l16)) {
		ret = PTR_ERR(panel_truly->reg_s4_iovdd);
		dev_err(dev->dev, "failed to request dsi_s4_iovdd regulator: %d\n", ret);
		goto fail;
	}

	ret = regulator_set_voltage(panel_truly->reg_l8_avdd,  3300000, 3300000);
	if (ret) {
		dev_err(dev->dev, "set_voltage l8 failed: %d\n", ret);
		goto fail;
	}

	ret = regulator_set_voltage(panel_truly->reg_s4_iovdd,  1800000, 1800000);
	if (ret) {
		dev_err(dev->dev, "set_voltage l2 failed: %d\n", ret);
		goto fail;
	}


	return panel;
fail:
	if (panel)
		panel_truly_destroy(panel);
	return ERR_PTR(ret);
}
