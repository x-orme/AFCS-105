/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    crc.c
  * @brief   This file provides code for the configuration
  *          of the CRC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "crc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CRC_HandleTypeDef hcrc;

/* CRC init function */
void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

void HAL_CRC_MspInit(CRC_HandleTypeDef* crcHandle)
{

  if(crcHandle->Instance==CRC)
  {
  /* USER CODE BEGIN CRC_MspInit 0 */

  /* USER CODE END CRC_MspInit 0 */
    /* CRC clock enable */
    __HAL_RCC_CRC_CLK_ENABLE();
  /* USER CODE BEGIN CRC_MspInit 1 */

  /* USER CODE END CRC_MspInit 1 */
  }
}

void HAL_CRC_MspDeInit(CRC_HandleTypeDef* crcHandle)
{

  if(crcHandle->Instance==CRC)
  {
  /* USER CODE BEGIN CRC_MspDeInit 0 */

  /* USER CODE END CRC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CRC_CLK_DISABLE();
  /* USER CODE BEGIN CRC_MspDeInit 1 */

  /* USER CODE END CRC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
 * @brief  하드웨어 CRC 유닛으로 바이트 배열 CRC-32 계산.
 * @note   F401 CRC 유닛: 32비트 워드 입력만 지원 (바이트 단위 불가).
 *         바이트를 빅엔디안으로 패킹하며 잔여 바이트는 0 패딩.
 *         STM32-Python 간 패킹 규칙 일치 필수.
 */
uint32_t HW_CRC32(const uint8_t *data, uint32_t len)
{
  /* CRC 유닛 리셋 (DR = 0xFFFFFFFF) */
  __HAL_CRC_DR_RESET(&hcrc);

  uint32_t words = len / 4U;
  uint32_t tail  = len % 4U;

  /* 완전한 32비트 워드 — 빅엔디안 패킹 */
  for (uint32_t i = 0; i < words; i++) {
    uint32_t idx = i * 4U;
    uint32_t w = ((uint32_t)data[idx]     << 24) |
                 ((uint32_t)data[idx + 1U] << 16) |
                 ((uint32_t)data[idx + 2U] <<  8) |
                 ((uint32_t)data[idx + 3U]);
    CRC->DR = w;
  }

  /* 잔여 바이트 — 상위 비트 정렬, 나머지 0 패딩 */
  if (tail > 0U) {
    uint32_t w = 0;
    uint32_t base = words * 4U;
    for (uint32_t i = 0; i < tail; i++) {
      w |= ((uint32_t)data[base + i] << (24U - 8U * i));
    }
    CRC->DR = w;
  }

  return CRC->DR;
}
/* USER CODE END 1 */
