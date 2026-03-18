/**
 * @file    flash_ops.h
 * @brief   내부 Flash 포대 좌표 영속 저장 (Sector 7, CRC32).
 */
#ifndef __FLASH_OPS_H
#define __FLASH_OPS_H

#include "main.h"
#include "fcs_common.h" // 공용 타입

// STM32F401RE / F411RE 참조
// 섹터 7: 0x08060000 ~ 0x0807FFFF (128KB) - 512KB 모델 마지막 섹터
#define FLASH_STORAGE_ADDR 0x08060000 
#define FLASH_MAGIC_CODE   0xFCCF0004  // v4: float 통일 (기존 v3 double 레이아웃 무효화)

typedef struct {
  uint32_t magic;
  uint8_t  zone;
  uint8_t  band;
  uint8_t  reserved[2]; // 패딩
  float    easting;
  float    northing;
  float    altitude;
  uint32_t crc32;       // [zone..altitude] 구간 CRC32
} Flash_SaveData_t;

// 함수 선언

/** @brief Sector 7 소거 후 포대 좌표 + CRC32 기록. */
void Flash_Save_BatteryPos(FCS_System_t *sys);

/** @brief Flash에서 포대 좌표 로드; 매직 코드 및 CRC32 검증. */
void Flash_Load_BatteryPos(FCS_System_t *sys);

#endif
