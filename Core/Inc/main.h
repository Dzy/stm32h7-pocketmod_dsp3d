/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* LTDC timing/pixel-format entry selected from LTDCSYNC[]. */
#define LTDC_VID_FORMAT 8U

/* External SDRAM and framebuffer layout.
 *
 * Two 4 MiB slots are reserved for L8 framebuffers.  The first 8 MiB is
 * overlaid by a non-cacheable MPU region so CPU, DMA2D and LTDC always see
 * the same bytes without per-frame cache maintenance.  The remaining SDRAM
 * stays cacheable; the depth buffer starts immediately after the two slots.
 */
#define SDRAM_BASE_ADDRESS              0xC0000000U
#define SDRAM_SIZE_BYTES                (32U * 1024U * 1024U)
#define FRAMEBUFFER_SLOT_SIZE_BYTES     (4U * 1024U * 1024U)
#define FRAMEBUFFER0_ADDRESS            (SDRAM_BASE_ADDRESS)
#define FRAMEBUFFER1_ADDRESS            (SDRAM_BASE_ADDRESS + FRAMEBUFFER_SLOT_SIZE_BYTES)
#define FRAMEBUFFER_MPU_SIZE_BYTES      (8U * 1024U * 1024U)
#define DEPTH_BUFFER_ADDRESS            (SDRAM_BASE_ADDRESS + FRAMEBUFFER_MPU_SIZE_BYTES)

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USB_FAULT_Pin GPIO_PIN_2
#define USB_FAULT_GPIO_Port GPIOE
#define VBUS_EN_Pin GPIO_PIN_3
#define VBUS_EN_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
