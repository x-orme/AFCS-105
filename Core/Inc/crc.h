/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    crc.h
  * @brief   This file contains all the function prototypes for
  *          the crc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CRC_H__
#define __CRC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CRC_HandleTypeDef hcrc;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CRC_Init(void);

/* USER CODE BEGIN Prototypes */
/**
 * @brief  STM32 하드웨어 CRC 유닛으로 바이트 배열의 CRC-32 계산.
 * @note   STM32F401 CRC 유닛은 32비트 워드만 입력 가능; 바이트를
 *         빅엔디안으로 패킹하고 잔여 바이트는 0 패딩.
 *         다항식: 0x04C11DB7 (CRC-32/MPEG-2, 비반사).
 * @param  data  입력 바이트 배열
 * @param  len   바이트 수
 * @retval 하드웨어 CRC-32 값
 */
uint32_t HW_CRC32(const uint8_t *data, uint32_t len);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CRC_H__ */

