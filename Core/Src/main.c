/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "dma2d.h"
#include "i2c.h"
#include "i2s.h"
#include "ltdc.h"
#include "usb_host.h"
#include "gpio.h"
#include "fmc.h"
#include "rtc.h"
#include "tda998x.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usb_host.h"

#include "usbh_hid.h"
#include "usbh_hid_mouse.h"
#include "usbh_hid_parser.h"

#include "dsp3D.h"
#include "dsp3D_LL.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

//#define POCKETMOD_IMPLEMENTATION
//#include "pocketmod.h"

//pocketmod_context context;

//#include "doyouwantme_mod.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MODELNAME teapotModelData
extern float32_t MODELNAME[];

uint32_t bbuffer = FRAMEBUFFER1_ADDRESS;
uint32_t fbuffer = FRAMEBUFFER0_ADDRESS;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern const uint8_t hdh48_3[];
extern const uint8_t hdh48_3_small[];

extern const LTDCSYNC_t LTDCSYNC[];

static void blitchar(uint32_t xpos, uint32_t ypos, uint8_t c);
static void blitchar2(uint32_t xpos, uint32_t ypos, uint8_t c);

static void blit_l8_glyph(const uint8_t *font,
                          uint32_t glyph_width,
                          uint32_t glyph_storage_height,
                          uint32_t glyph_draw_height,
                          uint32_t xpos,
                          uint32_t ypos,
                          uint8_t c)
{
  const uint32_t first_char = (uint32_t)' ';
  const uint32_t last_char = (uint32_t)'z';
  const uint32_t screen_width = LTDCSYNC[LTDC_VID_FORMAT].ahw;
  const uint32_t screen_height = LTDCSYNC[LTDC_VID_FORMAT].avh;
  const uint32_t dst_x = xpos * glyph_width;
  const uint32_t dst_y = ypos * glyph_draw_height;

  /* Both font arrays contain 91 glyphs: ASCII 0x20 through 0x7A. */
  if (((uint32_t)c < first_char) || ((uint32_t)c > last_char)) {
    c = (uint8_t)'?';
  }

  if ((dst_x >= screen_width) || (dst_y >= screen_height)) {
    return;
  }

  const uint32_t copy_width =
      ((screen_width - dst_x) < glyph_width) ? (screen_width - dst_x) : glyph_width;
  const uint32_t copy_height =
      ((screen_height - dst_y) < glyph_draw_height) ?
          (screen_height - dst_y) : glyph_draw_height;
  const uint32_t glyph_stride = glyph_width * glyph_storage_height;
  const uint8_t *src = font + glyph_stride * ((uint32_t)c - first_char);
  uint8_t *dst = (uint8_t *)(uintptr_t)bbuffer + dst_y * screen_width + dst_x;

  /*
   * LTDC uses an L8 framebuffer: one source byte must become exactly one
   * framebuffer byte. DMA2D has no L8 output format, so an L8-to-RGB565
   * transfer performs color conversion and cannot be used as a byte copy.
   * These glyphs are small; a row-wise copy is deterministic and pixel-exact.
   */
  for (uint32_t row = 0U; row < copy_height; ++row) {
    memcpy(dst, src, copy_width);
    src += glyph_width;
    dst += screen_width;
  }
}

static void blitchar(uint32_t xpos, uint32_t ypos, uint8_t c)
{
  blit_l8_glyph(hdh48_3_small, 16U, 26U, 24U, xpos, ypos, c);
}

static void blitstring(uint32_t xpos, uint32_t ypos, const char *ptr)
{
  while (*ptr != '\0') {
    blitchar(xpos, ypos, (uint8_t)*ptr);
    xpos++;
    ptr++;
  }
}

static void blitstring2(uint32_t xpos, uint32_t ypos, const char *ptr)
{
  while (*ptr != '\0') {
    blitchar2(xpos, ypos, (uint8_t)*ptr);
    xpos++;
    ptr++;
  }
}

static void blitchar2(uint32_t xpos, uint32_t ypos, uint8_t c)
{
  blit_l8_glyph(hdh48_3, 32U, 52U, 48U, xpos, ypos, c);
}

int _write(int file, char *data, int len)
{
  (void)file;

  for (int i = 0; i < len; ++i) {
    const uint8_t c = (uint8_t)data[i];
    if (c == (uint8_t)'\n') {
      continue;
    }
    blitchar((uint32_t)i, 0U, c);
  }

  return len;
}

static volatile uint8_t ltdc_swap_pending = 0U;
static volatile uint8_t ltdc_reload_complete = 0U;
int8_t redraw = 1;

void HAL_LTDC_ReloadEventCallback(LTDC_HandleTypeDef *hltdc)
{
  (void)hltdc;

  /* Ignore unrelated LTDC reloads (for example CLUT configuration). */
  if (ltdc_swap_pending != 0U) {
    __DMB();
    ltdc_reload_complete = 1U;
  }
}

extern HID_MOUSE_Info_TypeDef    mouse_info;
extern USBH_HandleTypeDef hUsbHostFS;
extern ApplicationTypeDef Appli_state;

extern I2S_HandleTypeDef hi2s1;

int8_t MouseX, MouseY;
HID_MOUSE_Info_TypeDef *m_pinfo;

float rx = 0.0, ry = 0.0, rz = 0.0;



void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
  if(USBH_HID_GetDeviceType(phost) == HID_MOUSE)
  {
    HID_MOUSE_Info_TypeDef *Mouse_Info;
    Mouse_Info = USBH_HID_GetMouseInfo(phost);  // Get the info
    MouseX = Mouse_Info->x;  // get the x value
    MouseY = Mouse_Info->y;  // get the y value
    rx -= (float)MouseX / 128.0f;
    ry += (float)MouseY / 128.0f;
    redraw = 1;
  }

}

float clip(float value) {
    value = value < -1.0f ? -1.0f : value;
    value = value > +1.0f ? +1.0f : value;
    return value;
}

float buffer[256][2];
int16_t output[256][2];

  int rendered_bytes;
  int rendered_samples;

uint32_t modticks = 0, modticks_old = 0;

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  //(void)hi2s;
  //modticks++;
  //rendered_bytes = pocketmod_render(&context, buffer, 256);
  //rendered_samples = rendered_bytes / sizeof(float[2]);

  //for (int i = 0; i < rendered_samples; i++) {
  //  output[i][0] = (int16_t) (clip(buffer[i][0]) * 0x7fff);
  //  output[i][1] = (int16_t) (clip(buffer[i][1]) * 0x7fff);
  //}

  //HAL_I2S_Transmit_IT(&hi2s1, (uint16_t *)(void *)output, (uint16_t)(rendered_samples * 2));
}



#define PI 3.14159265358979f
#define TAU (2.0f * PI)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

double temp;
void readtemp(void) {

  uint32_t adc_v;
  HAL_ADC_Start(&hadc3);
  HAL_ADC_PollForConversion(&hadc3, 100);

  adc_v = HAL_ADC_GetValue(&hadc3);
  //adcx = (110.0-30.0)/(*(unsigned short*)(0x1FF1E840) - *(unsigned short*)(0x1FF1E820));
  //temp1 = adcx*(adc_v - *(unsigned short*)(0x1FF1E820))+30;
  temp = __HAL_ADC_CALC_TEMPERATURE(3370, adc_v, ADC_RESOLUTION_16B);

}


static size_t Framebuffer_SizeBytes(void)
{
  return (size_t)LTDCSYNC[LTDC_VID_FORMAT].ahw *
         (size_t)LTDCSYNC[LTDC_VID_FORMAT].avh;
}

static void Framebuffer_Clear(uint32_t address)
{
  const uint32_t width = LTDCSYNC[LTDC_VID_FORMAT].ahw;
  const uint32_t height = LTDCSYNC[LTDC_VID_FORMAT].avh;

  /* The project intentionally uses RGB565 DMA2D output as a two-byte write
   * unit for the L8 framebuffer.  Clearing width/2 RGB565 pixels therefore
   * writes exactly width L8 bytes on every scanline. */
  if ((width & 1U) != 0U) {
    Error_Handler();
  }

  hdma2d.Init.Mode = DMA2D_R2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0U;

  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_DMA2D_Start(&hdma2d, 0U, address, width / 2U, height) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_DMA2D_PollForTransfer(&hdma2d, 1000U) != HAL_OK) {
    Error_Handler();
  }

  /* Restore the mode expected by the existing font blitters. */
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.OutputOffset = 0U;

  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK) {
    Error_Handler();
  }

  /* The framebuffer MPU overlay is Normal, non-cacheable memory. */
  __DSB();
}

static void Framebuffer_QueuePresent(void)
{
  if (ltdc_swap_pending != 0U) {
    return;
  }

  /* Complete all CPU writes before the LTDC shadow register is reloaded.
   * DMA2D transfers used by the text renderer are already polled to completion. */
  __DSB();

  ltdc_reload_complete = 0U;

  if (HAL_LTDC_SetAddress_NoReload(&hltdc, bbuffer, 0U) != HAL_OK) {
    Error_Handler();
  }

  ltdc_swap_pending = 1U;
  __DMB();

  if (HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_VERTICAL_BLANKING) != HAL_OK) {
    ltdc_swap_pending = 0U;
    Error_Handler();
  }
}

static uint8_t Framebuffer_FinishPresent(void)
{
  uint32_t old_front;

  if ((ltdc_swap_pending == 0U) || (ltdc_reload_complete == 0U)) {
    return 0U;
  }

  __DMB();

  /* Only swap software ownership after LTDC confirms the vertical-blank
   * reload.  The old front buffer is now safe to clear and render into. */
  old_front = fbuffer;
  fbuffer = bbuffer;
  bbuffer = old_front;

  ltdc_reload_complete = 0U;
  ltdc_swap_pending = 0U;

  Framebuffer_Clear(bbuffer);
  return 1U;
}

void MPU_Conf(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();

  /* Region 0: the complete 32 MiB SDRAM is write-back cacheable.  CPU-only
   * data, including the depth buffer at 0xC0800000, retains full D-cache
   * performance. */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = SDRAM_BASE_ADDRESS;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.SubRegionDisable = 0x00U;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Region 1 has higher priority and overlays both framebuffer slots.
   * TEX=1,C=0,B=0 selects Normal non-cacheable memory, avoiding stale or
   * dirty cache lines when CPU, DMA2D and LTDC share the same buffers. */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = FRAMEBUFFER0_ADDRESS;
  MPU_InitStruct.Size = MPU_REGION_SIZE_8MB;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

float cz = 0.0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  MPU_Conf();

  //if (!pocketmod_init(&context, &doyouwantme_mod, sizeof(doyouwantme_mod), 48000)) {
    //while(1);
  //}

  /* USER CODE END 1 */
  
  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* High-speed GPIO interfaces such as FMC benefit from the STM32H7 I/O
   * compensation cell.  CSI and SYSCFG clocks are prerequisites. */
  __HAL_RCC_CSI_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  HAL_EnableCompensationCell();

  /* FMC timing at 108 MHz relies on the characterized high-speed I/O
   * conditions.  Do not start FMC traffic until the compensation cell is
   * actually ready. */
  while ((SYSCFG->CCCSR & SYSCFG_CCCSR_READY) == 0U)
  {
  }

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_FMC_Init();
  MX_DMA2D_Init();

  /* Clear both buffers before LTDC starts fetching from external SDRAM. */
  fbuffer = FRAMEBUFFER0_ADDRESS;
  bbuffer = FRAMEBUFFER1_ADDRESS;

  if (Framebuffer_SizeBytes() > FRAMEBUFFER_SLOT_SIZE_BYTES) {
    Error_Handler();
  }

  Framebuffer_Clear(fbuffer);
  Framebuffer_Clear(bbuffer);

  MX_LTDC_Init();
  MX_I2C1_Init();
  MX_I2S1_Init();
  MX_DMA_Init();
  MX_USB_HOST_Init();
  MX_ADC3_Init();
  
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

/**
  * @brief  Load the color lookup table.
  * @param  hltdc     pointer to a LTDC_HandleTypeDef structure that contains
  *                   the configuration information for the LTDC.
  * @param  pCLUT     pointer to the color lookup table address.
  * @param  CLUTSize  the color lookup table size.
  * @param  LayerIdx  LTDC Layer index.
  *                   This parameter can be one of the following values:
  *                   LTDC_LAYER_1 (0) or LTDC_LAYER_2 (1)
  * @retval HAL status
  */

  uint8_t z = 0x00;
  uint32_t pCLUT[256], i;
/*
  for(i=0;i<256;i++){
      r = ((32.0/256.0)*(float)i)*8.0;
      g = ((64.0/256.0)*(float)i)*4.0;
      pCLUT[i] = 0xff000000 |r<<16|g<<8|r;
  }

*/

  for(i=0;i<256;i++){
    pCLUT[i] = 0xff000000 | z<<16 | z<<8 | z;
    z++;
  }

  HAL_LTDC_ConfigCLUT(&hltdc, pCLUT, 256U, 0U);
  HAL_LTDC_EnableCLUT(&hltdc, 0);
  HAL_LTDC_DisableDither(&hltdc);


  tda_init();


  uint32_t tn = 0,to = 0, te = 0, tmax = 0, tmin = 1000;

  char txtbuf[100];

  dsp3D_init();

  /* USER CODE END 2 */


/*
  nsamples = sizeof(signal) / sizeof(signal[0]);
    i = 0;
    while(i < nsamples) {
        double t = ((double)i/2.0)/((double)nsamples);
        signal[i+0] = 32767*sin(100.0 * TAU * t); // left
        signal[i+1] = 32767*sin(100.0 * TAU * t); // right
        i += 2;
    }
*/

  //rendered_bytes = pocketmod_render(&context, buffer, 256);
  //rendered_samples = rendered_bytes / sizeof(float[2]);

  //for (int i = 0; i < rendered_samples; i++) {
  //  output[i][0] = (int16_t) (clip(buffer[i][0]) * 0x7fff);
  //  output[i][1] = (int16_t) (clip(buffer[i][1]) * 0x7fff);
  //}

  //HAL_I2S_Transmit_IT(&hi2s1, (uint16_t *)(void *)output, (uint16_t)(rendered_samples * 2));

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    MX_USB_HOST_Process();

    /* The vertical-blank swap is asynchronous.  Keep servicing USB while
     * waiting, but never render into a buffer still owned by LTDC. */
    if (Framebuffer_FinishPresent() != 0U) {
      redraw = 1;
    }

    if (ltdc_swap_pending != 0U) {
      continue;
    }

    tn = HAL_GetTick();

    if(redraw == 1) {

        redraw = 0;

        //dsp3D_setMeshRotation(rx, ry, rz);
        
        cz += 0.010;
        dsp3D_setMeshRotation(cz, cz, cz);

        const uint32_t fps = (te != 0U) ? (1000U / te) : 0U;
        snprintf(txtbuf, sizeof(txtbuf), "%lums (min %lums/max %lums) %lu FPS",
                 (unsigned long)te, (unsigned long)tmin, (unsigned long)tmax,
                 (unsigned long)fps);
        blitstring2(0,1, txtbuf);

        snprintf(txtbuf, sizeof(txtbuf), "numVert  %lu", (unsigned long)(uint32_t)MODELNAME[0]);
        blitstring2(0,2, txtbuf);

        snprintf(txtbuf, sizeof(txtbuf), "numFaces %lu", (unsigned long)(uint32_t)MODELNAME[1]);
        blitstring2(0,3, txtbuf);

        snprintf(txtbuf, sizeof(txtbuf), "CPU %luMHz",
                 (unsigned long)(HAL_RCC_GetSysClockFreq() / 1000000U));
        blitstring2(0,4, txtbuf);

        snprintf(txtbuf, sizeof(txtbuf), "X=%d, Y=%d %lu %d",
                 (int)MouseX, (int)MouseY, (unsigned long)modticks, rendered_samples);
        blitstring2(0,5, txtbuf);

        modticks_old = modticks;


        RTC_TimeTypeDef sTime;
        RTC_DateTypeDef sDate;
/*  
sDate.Date = 0x20;
sDate.Month = 0x07;
sDate.Year = 0x21;
sDate.WeekDay = RTC_WEEKDAY_SUNDAY;

sTime.Hours = 0x23;
sTime.Minutes = 0x14;
sTime.SecondFraction = 0x00;
HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
*/

        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

        snprintf(txtbuf, sizeof(txtbuf), "%02u:%02u:%02u",
                 (unsigned int)sTime.Hours, (unsigned int)sTime.Minutes,
                 (unsigned int)sTime.Seconds);
        blitstring2(0,6, txtbuf);

        snprintf(txtbuf, sizeof(txtbuf), "%02u.%02u.%02u",
                 (unsigned int)sDate.Date, (unsigned int)sDate.Month,
                 (unsigned int)sDate.Year);
        blitstring2(0,7, txtbuf);

        snprintf(txtbuf, sizeof(txtbuf), "0123456789ABCD");
        blitstring2(0,8, txtbuf);

        dsp3D_setCameraPosition(0.0, 0.0, 20.0);
        dsp3D_setBackFaceCulling(1);

        //dsp3D_setLightPosition(0.0, 0.0, 100.0);

        dsp3D_setPhongMaterial(
          0.08f,   /* ambient */
          0.72f,   /* diffuse */
          0.35f,   /* specular */
          16       /* shininess */
        );

        //dsp3D_renderWireframe(MODELNAME);
        //dsp3D_renderFlat((float *)&MODELNAME);
        //dsp3D_renderPoints((float *)&MODELNAME);
        dsp3D_renderPhong((float *)&MODELNAME);
        //dsp3D_renderGouraud((float *)&MODELNAME);

        dsp3D_present();

        to = tn;
        tn = HAL_GetTick();
        if(tn>to) {
          te = tn-to;
        } else {
          te = to - tn;
        }

        if(tmax<te && to != 0) {
          tmax = te;
        }

        if(tmin>te && to != 0) {
          tmin = te;
        }


        Framebuffer_QueuePresent();
    }

    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_SPI1|RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB|RCC_PERIPHCLK_FMC;

  /* PLL2 serves both ADC and FMC.  With the 8 MHz HSE:
   *   VCO   = 8 / 1 * 54 = 432 MHz
   *   PLL2P = 432 / 6     = 72 MHz  (ADC: unchanged)
   *   PLL2R = 432 / 2     = 216 MHz (FMC kernel)
   *   SDRAM = PLL2R / 2   = 108 MHz
   *
   * 108 MHz keeps STM32H743 Rev V below the characterized 110 MHz
   * FMC_SDCLK limit while retaining substantially more bandwidth than
   * dropping the bus to 100 MHz. */
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 54;
  PeriphClkInitStruct.PLL2.PLL2P = 6;
  PeriphClkInitStruct.PLL2.PLL2Q = 6;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;

  PeriphClkInitStruct.PLL3.PLL3M = 8;
  PeriphClkInitStruct.PLL3.PLL3N = LTDCSYNC[LTDC_VID_FORMAT].pll3n; //295; //294; //296; //294; //252; //320; //252;
  PeriphClkInitStruct.PLL3.PLL3P = LTDCSYNC[LTDC_VID_FORMAT].pll3p; //2;
  PeriphClkInitStruct.PLL3.PLL3Q = LTDCSYNC[LTDC_VID_FORMAT].pll3q; //2;
  PeriphClkInitStruct.PLL3.PLL3R = LTDCSYNC[LTDC_VID_FORMAT].pll3r; //8;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;

  PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable USB Voltage detector
  */
  HAL_PWREx_EnableUSBVoltageDetector();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* PeriphClkInitStruct.PLL3.PLL3N = 260 -> 65MHz at LCD */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
