/**
  ******************************************************************************
  * @file    ltdc.c
  * @brief   This file provides code for the configuration
  *          of the LTDC instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ltdc.h"

/* USER CODE BEGIN 0 */
const LTDCSYNC_t LTDCSYNC[] = {
 { 252, 2, 2, 8,    640,  480,    24,  40, 128,    9, 3, 28 }, // 0 640x480_72Hz
 { 252, 2, 2, 8,    640,  480,    16,  96,  48,   11, 2, 32 }, // 1 640x480_75Hz
 { 288, 2, 2, 8,    640,  480,    32,  48, 112,    1, 3, 25 }, // 2 640x480_85Hz
 { 295, 2, 2, 8,    800,  600,    40, 128,  88,    1, 4, 23 }, // 3 800x600_60Hz
 { 400, 2, 2, 8,    800,  600,    56, 120,  64,   37, 6, 23 }, // 4 800x600_72Hz
 { 396, 2, 2, 8,    800,  600,    16,  80, 160,    1, 2, 21 }, // 5 800x600_75Hz
 { 450, 2, 2, 8,    800,  600,    32,  64, 152,    1, 3, 27 }, // 6 800x600_85Hz
 { 265, 4, 4, 4,   1024,  768,    24, 136, 160,    3, 6, 29 }, // 7 1024x768_60Hz
    
 { 300, 4, 4, 4,   1024,  768,    24, 136, 144,    3, 6, 29 }, // 8 1024x768_70Hz
    
 { 315, 4, 4, 4,   1024,  768,    16,  96, 176,    1, 3, 28 }, // 9 1024x768_75Hz
 { 432, 4, 4, 4,   1280, 1024,    48, 112, 248,    1, 3, 38 }, //10 1280x1024_60Hz
 { 432, 4, 4, 4,   1920, 1080,   128, 208, 336,    1, 3, 38 }, //11 1920x1080_60Hz
};    
/*

 { 252, 2, 2, 8,    640,  480,    16,  64, 120,    1, 3, 16,    95, 143,  783,  799,   1, 33,  513,  524 }, // 1 640x480_75Hz

  NPIX    NLINE  VsLineStart  VsPixStart  VsLineEnd   VsPixEnd    HsStart     HsEnd   ActiveVideoStart   ActiveVideoEnd DeStart DeEnd
  npix    nline  vsl_s1       vsp_s1      vsl_e1      vsp_e1      hs_e        hs_e    vw_s1              vw_e1          de_s    de_e
 {1688,   1066,  1,           48,         4,          48,         48,         160,    41,                1065,          408,    1688, 0, 0}, // E_REGVFMT_1280x1024p_60Hz 

typedef struct _LTDCSYNC_t {
   uint16_t pll3n, pll3p, pll3q, pll3r;
   uint16_t ahw, avh;
   uint16_t hfp, hsw, hbp;
   uint16_t vfp, vsh, vbp;
} LTDCSYNC_t;
 { 432, 4, 4, 4, 1280, 1024, 48, 112, 248, 1, 3, 38, 111, 359, 1639, 1687, 2, 40, 1064, 1065 }, //10 1280x1024_60Hz

//                        HTOTAL VTOTAL
//1280 × 1024p [1] 60.020 1688 1066 63.98132 108.0004682

  regVal = 0x00; 
  err = setHwRegister(pDis, E_REG_P00_VIDFORMAT_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) format_param[reg_idx].nPix;
  err = setHwRegister(pDis, E_REG_P00_NPIX_LSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) (format_param[reg_idx].nPix >> 8);
  err = setHwRegister(pDis, E_REG_P00_NPIX_MSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) format_param[reg_idx].nLine;
  err = setHwRegister(pDis, E_REG_P00_NLINE_LSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) (format_param[reg_idx].nLine >> 8);
  err = setHwRegister(pDis, E_REG_P00_NLINE_MSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) format_param[reg_idx].VsLineStart;
  err = setHwRegister(pDis, E_REG_P00_VS_LINE_STRT_1_LSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) format_param[reg_idx].VsPixStart;
  err = setHwRegister(pDis, E_REG_P00_VS_PIX_STRT_1_LSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) (format_param[reg_idx].VsPixStart >> 8);
  err = setHwRegister(pDis, E_REG_P00_VS_PIX_STRT_1_MSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) format_param[reg_idx].VsLineEnd;
  err = setHwRegister(pDis, E_REG_P00_VS_LINE_END_1_LSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) format_param[reg_idx].VsPixEnd;
  err = setHwRegister(pDis, E_REG_P00_VS_PIX_END_1_LSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) (format_param[reg_idx].VsPixEnd >> 8);
  err = setHwRegister(pDis, E_REG_P00_VS_PIX_END_1_MSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) format_param[reg_idx].HsStart;
  err = setHwRegister(pDis, E_REG_P00_HS_PIX_START_LSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) (format_param[reg_idx].HsStart >> 8);
  err = setHwRegister(pDis, E_REG_P00_HS_PIX_START_MSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) format_param[reg_idx].HsEnd;
  err = setHwRegister(pDis, E_REG_P00_HS_PIX_STOP_LSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) (format_param[reg_idx].HsEnd >> 8);
  err = setHwRegister(pDis, E_REG_P00_HS_PIX_STOP_MSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) format_param[reg_idx].ActiveVideoStart;
  err = setHwRegister(pDis, E_REG_P00_VWIN_START_1_LSB_W, regVal);
  RETIF_REG_FAIL(err);
  err = setHwRegister(pDis, E_REG_P00_VWIN_START_1_MSB_W, 0);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) format_param[reg_idx].ActiveVideoEnd;
  err = setHwRegister(pDis, E_REG_P00_VWIN_END_1_LSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) (format_param[reg_idx].ActiveVideoEnd >> 8);
  err = setHwRegister(pDis, E_REG_P00_VWIN_END_1_MSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) format_param[reg_idx].DeStart;
  err = setHwRegister(pDis, E_REG_P00_DE_START_LSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) (format_param[reg_idx].DeStart >> 8);
  err = setHwRegister(pDis, E_REG_P00_DE_START_MSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) format_param[reg_idx].DeEnd;
  err = setHwRegister(pDis, E_REG_P00_DE_STOP_LSB_W, regVal);
  RETIF_REG_FAIL(err);
  regVal = (UInt8) (format_param[reg_idx].DeEnd >> 8);
  err = setHwRegister(pDis, E_REG_P00_DE_STOP_MSB_W, regVal);
  RETIF_REG_FAIL(err);
*/

/* USER CODE END 0 */


/*
#ifdef FORMAT_PC
static CONST_DAT tmHdmiTxVidReg_t format_param_PC[HDMITX_VFMT_PC_NUM] =
{
   //  NPIX    NLINE  VsLineStart  VsPixStart  VsLineEnd   VsPixEnd    HsStart     HsEnd   ActiveVideoStart   ActiveVideoEnd DeStart DeEnd
   //  npix    nline  vsl_s1       vsp_s1      vsl_e1      vsp_e1      hs_e        hs_e    vw_s1              vw_e1          de_s    de_e
    {832,   520,      1,          24,          4,           24,         24,     64,     31,         511,    192,    832, 0, 0},/ E_REGVFMT_640x480p_72Hz   /
    {840,   500,      1,          16,          4,           16,         16,     80,     19,         499,    200,    840, 0, 0},/ E_REGVFMT_640x480p_75Hz   /
    {832,   509,      1,          56,          4,           56,         56,     112,    28,         508,    192,    832, 0, 0},/ E_REGVFMT_640x480p_85Hz   /

    {1056,  628,      1,          40,          5,           40,         40,     168,    27,         627,    256,   1056, 0, 0},/ E_REGVFMT_800x600p_60Hz   /

800x600, 60Hz   40.000  800 40  128 88  600 1 4 23

    {1040,  666,      1,          56,          7,           56,         56,     176,    29,         619,    240,   1040, 0, 0},/ E_REGVFMT_800x600p_72Hz   /
    {1056,  625,      1,          16,          4,           16,         16,     96,     24,         624,    256,   1056, 0, 0},/ E_REGVFMT_800x600p_75Hz   /
    {1048,  631,      1,          32,          4,           32,         32,     96,     30,         630,    248,   1048, 0, 0},/ E_REGVFMT_800x600p_85Hz   /
    {1344,  806,      1,          24,          7,           24,         24,     160,    35,         803,    320,   1344, 0, 0},/ E_REGVFMT_1024x768p_60Hz  /
    {1328,  806,      1,          24,          7,           24,         24,     160,    35,         803,    304,   1328, 0, 0},/ E_REGVFMT_1024x768p_70Hz  /
    {1312,  800,      1,          16,          4,           16,         16,     112,    31,         799,    288,   1312, 0, 0},/ E_REGVFMT_1024x768p_75Hz  /
    {1664,  798,      1,          64,          8,           64,         64,     192,    27,         795,    384,   1664, 0, 0},/ E_REGVFMT_1280x768p_60Hz  /
    {1688,  1066,     1,          48,          4,           48,         48,     160,    41,         1065,   408,   1688, 0, 0},/ E_REGVFMT_1280x1024p_60Hz /
    {1792,  795,      1,          64,          7,           64,         64,     176,    24,         792,    432,   1792, 0, 0},/ E_REGVFMT_1360x768p_60Hz  /
    {1864,  1089,     1,          88,          5,           88,         88,     232,    36,         1086,   464,   1864, 0, 0},/ E_REGVFMT_1400x1050p_60Hz /
    {2160,  1250,     1,          64,          4,           64,         64,     256,    49,         1249,   560,   2160, 0, 0},/ E_REGVFMT_1600x1200p_60Hz /
    {1728,  1072,     1,          64,          4,           64,         64,     224,    47,         1071,   448,   1728, 0, 0} / E_REGVFMT_1280x1024p_85Hz /
};
#endif
*/

LTDC_HandleTypeDef hltdc;

/* LTDC init function */
void MX_LTDC_Init(void)
{
  LTDC_LayerCfgTypeDef pLayerCfg;

  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AH;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AH;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AH;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IIPC;

  hltdc.Init.HorizontalSync     = (LTDCSYNC[LTDC_VID_FORMAT].hsw - 1);
  hltdc.Init.VerticalSync       = (LTDCSYNC[LTDC_VID_FORMAT].vsh - 1);
  hltdc.Init.AccumulatedHBP     = (LTDCSYNC[LTDC_VID_FORMAT].hsw + LTDCSYNC[LTDC_VID_FORMAT].hbp - 1);
  hltdc.Init.AccumulatedVBP     = (LTDCSYNC[LTDC_VID_FORMAT].vsh + LTDCSYNC[LTDC_VID_FORMAT].vbp - 1);
  hltdc.Init.AccumulatedActiveW = (LTDCSYNC[LTDC_VID_FORMAT].hsw + LTDCSYNC[LTDC_VID_FORMAT].ahw + LTDCSYNC[LTDC_VID_FORMAT].hbp  - 1);
  hltdc.Init.AccumulatedActiveH = (LTDCSYNC[LTDC_VID_FORMAT].vsh + LTDCSYNC[LTDC_VID_FORMAT].avh + LTDCSYNC[LTDC_VID_FORMAT].vbp  - 1);
  hltdc.Init.TotalWidth         = (LTDCSYNC[LTDC_VID_FORMAT].hsw + LTDCSYNC[LTDC_VID_FORMAT].ahw + LTDCSYNC[LTDC_VID_FORMAT].hbp + LTDCSYNC[LTDC_VID_FORMAT].hfp - 1);
  hltdc.Init.TotalHeigh         = (LTDCSYNC[LTDC_VID_FORMAT].vsh + LTDCSYNC[LTDC_VID_FORMAT].avh + LTDCSYNC[LTDC_VID_FORMAT].vbp + LTDCSYNC[LTDC_VID_FORMAT].vfp - 1);

  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  HAL_LTDC_Init(&hltdc);

  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = LTDCSYNC[LTDC_VID_FORMAT].ahw;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = LTDCSYNC[LTDC_VID_FORMAT].avh;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_L8; //RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 255;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xc0000000;
  pLayerCfg.ImageWidth = LTDCSYNC[LTDC_VID_FORMAT].ahw;
  pLayerCfg.ImageHeight = LTDCSYNC[LTDC_VID_FORMAT].avh;
  pLayerCfg.Backcolor.Blue = 255;
  pLayerCfg.Backcolor.Green = 255;
  pLayerCfg.Backcolor.Red = 255;
  HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0);

}

void HAL_LTDC_MspInit(LTDC_HandleTypeDef* ltdcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(ltdcHandle->Instance==LTDC)
  {
  /* USER CODE BEGIN LTDC_MspInit 0 */

  /* USER CODE END LTDC_MspInit 0 */
    /* LTDC clock enable */
    __HAL_RCC_LTDC_CLK_ENABLE();

    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**LTDC GPIO Configuration
    
    PF10    ------> LTDC_DE
    PA4     ------> LTDC_VSYNC
    PG7     ------> LTDC_CLK
    PC6     ------> LTDC_HSYNC
    
    PB0     ------> LTDC_R3
    PA5     ------> LTDC_R4
    PC0     ------> LTDC_R5
    PB1     ------> LTDC_R6
    PG6     ------> LTDC_R7

    PA6     ------> LTDC_G2
    PC9     ------> LTDC_G3
    PB10    ------> LTDC_G4
    PH4     ------> LTDC_G5
    PI11    ------> LTDC_G6
    PD3     ------> LTDC_G7

    PA8     ------> LTDC_B3
    PG12    ------> LTDC_B4
    PA3     ------> LTDC_B5
    PB8     ------> LTDC_B6
    PB9     ------> LTDC_B7
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    //GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    //GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    //GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_6;
    //GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
    //GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_8|GPIO_PIN_9;
    //GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    //GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    //GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    //GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Alternate = GPIO_AF10_LTDC;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Alternate = GPIO_AF13_LTDC;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* LTDC interrupt Init */
    HAL_NVIC_SetPriority(LTDC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(LTDC_IRQn);
    HAL_NVIC_SetPriority(LTDC_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(LTDC_ER_IRQn);
  /* USER CODE BEGIN LTDC_MspInit 1 */

  /* USER CODE END LTDC_MspInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
