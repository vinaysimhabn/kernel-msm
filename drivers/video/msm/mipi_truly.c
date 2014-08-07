/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_truly.h"

#include "../board-8064.h"
#include <asm/mach-types.h>
#include <linux/pwm.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/gpio.h>


static struct dsi_buf truly_tx_buf;
static struct dsi_buf truly_rx_buf;
static int mipi_truly_lcd_init(void);

static struct msm_panel_common_pdata *mipi_truly_pdata;

#define TRULY_CMD_DELAY		0
#define TRULY_SLEEP_OFF_DELAY	150
#define TRULY_DISPLAY_ON_DELAY	150
#define GPIO_TRULY_LCD_RESET	129


static struct pwm_device *bl_lpm;
#define PWM_FREQ_HZ 300
#define PWM_PERIOD_USEC (USEC_PER_SEC / PWM_FREQ_HZ)
#define PWM_LEVEL 255
#define PWM_DUTY_LEVEL \
	(PWM_PERIOD_USEC / PWM_LEVEL)

#define gpio_LCD_BL_EN PM8921_GPIO_PM_TO_SYS(26)

static char display_off[2] = {0x28, 0x00};
static char enter_sleep[2] = {0x10, 0x00};

static struct dsi_cmd_desc truly_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(enter_sleep), enter_sleep}
};

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

static struct dsi_cmd_desc truly_display_on_cmds_temp1[] = {

	{DTYPE_GEN_LWRITE, 1, 0, 0, 50, sizeof(write_memory1), write_memory1},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory2), write_memory2},
	//{DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(write_memory3), write_memory3},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory3), write_memory3},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory4), write_memory4},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory5), write_memory5},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory6), write_memory6},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory7), write_memory7},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory8), write_memory8},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory9), write_memory9},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory10), write_memory10},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory11), write_memory11},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory12), write_memory12},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory13), write_memory13},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory14), write_memory14},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory15), write_memory15},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory16), write_memory16},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory17), write_memory17},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory18), write_memory18},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory19), write_memory19},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory20), write_memory20},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory21), write_memory21},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory22), write_memory22},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory23), write_memory23},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory24), write_memory24},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory25), write_memory25},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory26), write_memory26},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory27), write_memory27},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory29), write_memory29},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory30), write_memory30},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory31), write_memory31},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory32), write_memory32},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory33), write_memory33},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory34), write_memory34},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory35), write_memory35},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory36), write_memory36},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory37), write_memory37},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory38), write_memory38},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory39), write_memory39},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory40), write_memory40},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory41), write_memory41},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory42), write_memory42},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory43), write_memory43},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory44), write_memory44},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory45), write_memory45},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory46), write_memory46},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory47), write_memory47},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory48), write_memory48},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory49), write_memory49},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory50), write_memory50},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory51), write_memory51},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory52), write_memory52},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory53), write_memory53},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory54), write_memory54},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory55), write_memory55},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory56), write_memory56},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory57), write_memory57},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory58), write_memory58},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory59), write_memory59},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory60), write_memory60},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory61), write_memory61},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory62), write_memory62},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory63), write_memory63},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory64), write_memory64},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory65), write_memory65},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory66), write_memory66},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory67), write_memory67},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory68), write_memory68},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory69), write_memory69},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory70), write_memory70},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory71), write_memory71},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory72), write_memory72},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory73), write_memory73},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory74), write_memory74},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory75), write_memory75},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory76), write_memory76},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory77), write_memory77},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory78), write_memory78},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory79), write_memory79},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory80), write_memory80},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory81), write_memory81},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory82), write_memory82},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory83), write_memory83},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory84), write_memory84},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory85), write_memory85},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory86), write_memory86},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory87), write_memory87},

	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory88), write_memory88},



	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory89), write_memory89},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory90), write_memory90},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory91), write_memory91},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory92), write_memory92},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory93), write_memory93},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory94), write_memory94},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory95), write_memory95},
	{DTYPE_GEN_LWRITE, 1, 0, 0,0, sizeof(write_memory96), write_memory96},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory97), write_memory97},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory98), write_memory98},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory99), write_memory99},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(write_memory100), write_memory100},

	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory101), write_memory101},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory102), write_memory102},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory103), write_memory103},
	{DTYPE_GEN_WRITE, 1, 0, 0, 0, sizeof(write_memory104), write_memory104},

	{DTYPE_DCS_WRITE, 1, 0, 0, 10, sizeof(write_memory105), write_memory105},
	{DTYPE_DCS_WRITE, 1, 0, 0, 200, sizeof(write_memory106), write_memory106},


};
static struct dsi_cmd_desc truly_display_on_cmds_temp2[] = {

	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(write_memory105), write_memory105},
	{DTYPE_DCS_WRITE, 1, 0, 0, 200, sizeof(write_memory106), write_memory106},
};

static u32 mipi_read_reg(struct msm_fb_data_type *mfd, u16 reg)
{
	u32 data;
	int len = 1;
	struct dsi_cmd_desc cmd_read_reg = {
		//                DTYPE_GEN_READ2, 1, 0, 1, 0, /* cmd 0x24 */
		//                      sizeof(reg), (char *) &reg};

		DTYPE_DCS_READ, 1, 0 ,1, 0, sizeof(reg), (char *) &reg};

mipi_dsi_buf_init(&truly_tx_buf);
mipi_dsi_buf_init(&truly_rx_buf);

/* mutex had been acquired at mipi_dsi_on */
len = mipi_dsi_cmds_rx(mfd, &truly_tx_buf, &truly_rx_buf,
		&cmd_read_reg, len);

data = *(u32 *)truly_rx_buf.data;

if (len != 4)
	pr_err("%s: invalid rlen=%d, expecting 4.\n", __func__, len);

	pr_err("%s: reg=0x%x.data=0x%08x.\n", __func__, reg, data);

	return data;
	}

static int mipi_truly_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	u32 regval;
	//	struct dcs_cmd_req cmdreq;


	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;
	mipi_dsi_buf_init(&truly_tx_buf);
	/*	mipi_dsi_cmds_tx(&truly_tx_buf, truly_display_on_cmds,
		ARRAY_SIZE(truly_display_on_cmds));
	 */

	mipi_dsi_cmds_tx(&truly_tx_buf, truly_display_on_cmds_temp1,
			ARRAY_SIZE(truly_display_on_cmds_temp1));
	if(0) {
		mdelay(250);
		mipi_dsi_buf_init(&truly_tx_buf);
		mipi_dsi_cmds_tx(&truly_tx_buf, truly_display_on_cmds_temp2,
				ARRAY_SIZE(truly_display_on_cmds_temp2));
	}
	if (0)
		regval = mipi_read_reg(mfd, 0xDA);

	gpio_set_value_cansleep(gpio_LCD_BL_EN, 1);


	return 0;
}

static int mipi_truly_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	return 0;
	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;
	mipi_dsi_cmds_tx(&truly_tx_buf, truly_display_off_cmds,
			ARRAY_SIZE(truly_display_off_cmds));
	return 0;
}

#define BL_LEVEL	17
static void mipi_truly_set_backlight(struct msm_fb_data_type *mfd)
{

	int ret;
	static int bl_enable_sleep_control = 0; // sleep only when suspend or resume

	pr_debug("%s: back light level %d\n", __func__, mfd->bl_level);

	if (bl_lpm) {
		if (mfd->bl_level) {
			ret = pwm_config(bl_lpm, PWM_DUTY_LEVEL *
					mfd->bl_level, PWM_PERIOD_USEC);
			if (ret) {
				pr_err("pwm_config on lpm failed %d\n", ret);
				return;
			}
			ret = pwm_enable(bl_lpm);
			if (ret)
				pr_err("pwm enable/disable on lpm failed"
						"for bl %d\n",  mfd->bl_level);
			if(!bl_enable_sleep_control) {
				msleep(10);
				bl_enable_sleep_control = 1;
				printk("%s: pwm enable\n", __func__);
			}
			gpio_set_value_cansleep(gpio_LCD_BL_EN, 1);
		} else {
			gpio_set_value_cansleep(gpio_LCD_BL_EN, 0);
			if(bl_enable_sleep_control) {
				msleep(10);
				bl_enable_sleep_control = 0;
				printk("%s: pwm disable\n", __func__);
			}
			ret = pwm_config(bl_lpm, PWM_DUTY_LEVEL *
					mfd->bl_level, PWM_PERIOD_USEC);
			if (ret) {
				pr_err("pwm_config on lpm failed %d\n", ret);
				return;
			}
			pwm_disable(bl_lpm);
		}
	}


}

static int __devinit mipi_truly_lcd_probe(struct platform_device *pdev)
{

	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct platform_device *current_pdev;
	static struct mipi_dsi_phy_ctrl *phy_settings;
	current_pdev = msm_fb_add_device(pdev);

	if (current_pdev) {
		mfd = platform_get_drvdata(current_pdev);
		if (!mfd)
			return -ENODEV;
		if (mfd->key != MFD_KEY)
			return -EINVAL;
		mipi  = &mfd->panel_info.mipi;

		if (phy_settings != NULL)
			mipi->dsi_phy_db = phy_settings;
	}

	if (pdev->id == 0) {
		mipi_truly_pdata = pdev->dev.platform_data;
		return 0;
	}

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_truly_lcd_probe,
	.driver = {
		.name   = "mipi_truly",
	},
};

static struct msm_fb_panel_data truly_panel_data = {
	.on		= mipi_truly_lcd_on,
	.off		= mipi_truly_lcd_off,
	.set_backlight	= mipi_truly_set_backlight,
};

static int ch_used[3];

int mipi_truly_device_register(struct msm_panel_info *pinfo,
		u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	ret = mipi_truly_lcd_init();
	if (ret) {
		pr_err("mipi_truly_lcd_init() failed with ret %u\n", ret);
		return ret;
	}

	pdev = platform_device_alloc("mipi_truly", (panel << 8)|channel);

	if (!pdev)
		return -ENOMEM;

	truly_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &truly_panel_data,
			sizeof(truly_panel_data));
	if (ret) {
		pr_err("%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);

	if (ret) {
		pr_err("%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_truly_lcd_init(void)
{
	mipi_dsi_buf_alloc(&truly_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&truly_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

