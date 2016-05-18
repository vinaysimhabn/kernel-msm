/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#ifndef MIPI_TC358775_DSI2LVDS_H
#define MIPI_TC358775_DSI2LVDS_H

/* Registers definition */

/* DSI D-PHY Layer Registers */
#define D0W_DPHYCONTTX  0x0004  /* Data Lane 0 DPHY Tx Control */
#define CLW_DPHYCONTRX  0x0020  /* Clock Lane DPHY Rx Control */
#define D0W_DPHYCONTRX  0x0024  /* Data Lane 0 DPHY Rx Control */
#define D1W_DPHYCONTRX  0x0028  /* Data Lane 1 DPHY Rx Control */
#define D2W_DPHYCONTRX  0x002C  /* Data Lane 2 DPHY Rx Control */
#define D3W_DPHYCONTRX  0x0030  /* Data Lane 3 DPHY Rx Control */
#define COM_DPHYCONTRX  0x0038  /* DPHY Rx Common Control */
#define CLW_CNTRL       0x0040  /* Clock Lane Control */
#define D0W_CNTRL       0x0044  /* Data Lane 0 Control */
#define D1W_CNTRL       0x0048  /* Data Lane 1 Control */
#define D2W_CNTRL       0x004C  /* Data Lane 2 Control */
#define D3W_CNTRL       0x0050  /* Data Lane 3 Control */
#define DFTMODE_CNTRL   0x0054  /* DFT Mode Control */

/* DSI PPI Layer Registers */
#define PPI_STARTPPI    0x0104  /* START control bit of PPI-TX function. */
#define PPI_BUSYPPI     0x0108
#define PPI_LINEINITCNT 0x0110  /* Line Initialization Wait Counter  */
#define PPI_LPTXTIMECNT 0x0114
#define PPI_LANEENABLE  0x0134  /* Enables each lane at the PPI layer. */
#define PPI_TX_RX_TA    0x013C  /* DSI Bus Turn Around timing parameters */

/* Analog timer function enable */
#define PPI_CLS_ATMR    0x0140  /* Delay for Clock Lane in LPRX  */
#define PPI_D0S_ATMR    0x0144  /* Delay for Data Lane 0 in LPRX */
#define PPI_D1S_ATMR    0x0148  /* Delay for Data Lane 1 in LPRX */
#define PPI_D2S_ATMR    0x014C  /* Delay for Data Lane 2 in LPRX */
#define PPI_D3S_ATMR    0x0150  /* Delay for Data Lane 3 in LPRX */
#define PPI_D0S_CLRSIPOCOUNT    0x0164

#define PPI_D1S_CLRSIPOCOUNT    0x0168  /* For lane 1 */
#define PPI_D2S_CLRSIPOCOUNT    0x016C  /* For lane 2 */
#define PPI_D3S_CLRSIPOCOUNT    0x0170  /* For lane 3 */

#define CLS_PRE         0x0180  /* Digital Counter inside of PHY IO */
#define D0S_PRE         0x0184  /* Digital Counter inside of PHY IO */
#define D1S_PRE         0x0188  /* Digital Counter inside of PHY IO */
#define D2S_PRE         0x018C  /* Digital Counter inside of PHY IO */
#define D3S_PRE         0x0190  /* Digital Counter inside of PHY IO */
#define CLS_PREP        0x01A0  /* Digital Counter inside of PHY IO */
#define D0S_PREP        0x01A4  /* Digital Counter inside of PHY IO */
#define D1S_PREP        0x01A8  /* Digital Counter inside of PHY IO */
#define D2S_PREP        0x01AC  /* Digital Counter inside of PHY IO */
#define D3S_PREP        0x01B0  /* Digital Counter inside of PHY IO */
#define CLS_ZERO        0x01C0  /* Digital Counter inside of PHY IO */
#define D0S_ZERO        0x01C4  /* Digital Counter inside of PHY IO */
#define D1S_ZERO        0x01C8  /* Digital Counter inside of PHY IO */
#define D2S_ZERO        0x01CC  /* Digital Counter inside of PHY IO */
#define D3S_ZERO        0x01D0  /* Digital Counter inside of PHY IO */

#define PPI_CLRFLG      0x01E0  /* PRE Counters has reached set values */
#define PPI_CLRSIPO     0x01E4  /* Clear SIPO values, Slave mode use only. */
#define HSTIMEOUT       0x01F0  /* HS Rx Time Out Counter */
#define HSTIMEOUTENABLE 0x01F4  /* Enable HS Rx Time Out Counter */
#define DSI_STARTDSI    0x0204  /* START control bit of DSI-TX function */
#define DSI_BUSYDSI     0x0208
#define DSI_LANEENABLE  0x0210  /* Enables each lane at the Protocol layer. */
#define DSI_LANESTATUS0 0x0214  /* Displays lane is in HS RX mode. */
#define DSI_LANESTATUS1 0x0218  /* Displays lane is in ULPS or STOP state */

#define DSI_INTSTATUS   0x0220  /* Interrupt Status */
#define DSI_INTMASK     0x0224  /* Interrupt Mask */
#define DSI_INTCLR      0x0228  /* Interrupt Clear */
#define DSI_LPTXTO      0x0230  /* Low Power Tx Time Out Counter */

#define DSIERRCNT       0x0300  /* DSI Error Count */
#define APLCTRL         0x0400  /* Application Layer Control */
#define RDPKTLN         0x0404  /* Command Read Packet Length */

#define VPCTRL          0x0450  /* Video Path Control */
#define HTIM1           0x0454  /* Horizontal Timing Control 1 */
#define HTIM2           0x0458  /* Horizontal Timing Control 2 */
#define VTIM1           0x045C  /* Vertical Timing Control 1 */
#define VTIM2           0x0460  /* Vertical Timing Control 2 */
#define VFUEN           0x0464  /* Video Frame Timing Update Enable */

/* Mux Input Select for LVDS LINK Input */
#define LVMX0003        0x0480  /* Bit 0 to 3 */
#define LVMX0407        0x0484  /* Bit 4 to 7 */
#define LVMX0811        0x0488  /* Bit 8 to 11 */
#define LVMX1215        0x048C  /* Bit 12 to 15 */
#define LVMX1619        0x0490  /* Bit 16 to 19 */
#define LVMX2023        0x0494  /* Bit 20 to 23 */
#define LVMX2427        0x0498  /* Bit 24 to 27 */

#define LVCFG           0x049C  /* LVDS Configuration  */
#define LVPHY0          0x04A0  /* LVDS PHY 0 */
#define LVPHY1          0x04A4  /* LVDS PHY 1 */
#define SYSSTAT         0x0500  /* System Status  */
#define SYSRST          0x0504  /* System Reset  */
/* GPIO Registers */
#define GPIOC           0x0520  /* GPIO Control  */
#define GPIOO           0x0524  /* GPIO Output  */
#define GPIOI           0x0528  /* GPIO Input  */

/* I2C Registers */
#define I2CTIMCTRL      0x0540  /* I2C IF Timing and Enable Control */
#define I2CMADDR        0x0544  /* I2C Master Addressing */
#define WDATAQ          0x0548  /* Write Data Queue */
#define RDATAQ          0x054C  /* Read Data Queue */

/* Chip ID and Revision ID Register */
#define IDREG           0x0580

#define TC358775XBG_ID  0x00007500

/* Debug Registers */
#define DEBUG00         0x05A0  /* Debug */
#define DEBUG01         0x05A4  /* LVDS Data */


#endif  /* MIPI_TC358775_DSI2LVDS_H */
