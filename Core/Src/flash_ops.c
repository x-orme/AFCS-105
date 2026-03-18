/**
 * @file    flash_ops.c
 * @brief   내부 Flash 포대 좌표 저장/로드 — Sector 7,
 *          매직 코드 검증, 하드웨어 CRC32 무결성 검사.
 */
#include "flash_ops.h"
#include "crc.h"   // 하드웨어 CRC-32
#include "iwdg.h"  // erase 전 IWDG 리프레시
#include <string.h>
#include <stddef.h>
#include <stdio.h>

// CRC 대상: [zone..altitude] (magic과 crc32 사이)
#define CRC_DATA_OFFSET  offsetof(Flash_SaveData_t, zone)
#define CRC_DATA_SIZE    (offsetof(Flash_SaveData_t, crc32) - CRC_DATA_OFFSET)

// ST-LINK Live Expression 계측용
volatile uint32_t prof_erase_cycles = 0;

/**
 * @brief  Flash에서 포대 좌표 로드 (매직코드/CRC32 검증).
 * @param  sys  시스템 구조체
 */
void Flash_Load_BatteryPos(FCS_System_t *sys) {
  Flash_SaveData_t *data = (Flash_SaveData_t*)FLASH_STORAGE_ADDR;

  // 매직 코드 확인
  if (data->magic != FLASH_MAGIC_CODE) {
    DBG_PRINT("[FLASH] No valid data (magic mismatch). Defaults set.\r\n");
    sys->user_pos.zone = 52;
    sys->user_pos.band = 'S';
    sys->user_pos.easting = 0.0f;
    sys->user_pos.northing = 0.0f;
    sys->user_pos.altitude = 0.0f;
    return;
  }

  // CRC32 검증
  uint32_t calc_crc = HW_CRC32((const uint8_t*)data + CRC_DATA_OFFSET, CRC_DATA_SIZE);
  if (calc_crc != data->crc32) {
    DBG_PRINT("[FLASH] CRC32 mismatch (stored=0x%08lX calc=0x%08lX). Defaults set.\r\n",
           (unsigned long)data->crc32, (unsigned long)calc_crc);
    sys->user_pos.zone = 52;
    sys->user_pos.band = 'S';
    sys->user_pos.easting = 0.0f;
    sys->user_pos.northing = 0.0f;
    sys->user_pos.altitude = 0.0f;
    return;
  }

  // 유효 — 데이터 로드
  sys->user_pos.zone = data->zone;
  sys->user_pos.band = data->band;
  sys->user_pos.easting = data->easting;
  sys->user_pos.northing = data->northing;
  sys->user_pos.altitude = data->altitude;
  DBG_PRINT("[FLASH] Data loaded (CRC OK).\r\n");
}

/**
 * @brief  Flash Sector 7에 포대 좌표 저장 (CRC32 포함).
 * @param  sys  시스템 구조체
 */
void Flash_Save_BatteryPos(FCS_System_t *sys) {
  Flash_SaveData_t data;

  // 데이터 준비
  data.magic = FLASH_MAGIC_CODE;
  data.zone = (uint8_t)sys->user_pos.zone;
  data.band = (uint8_t)sys->user_pos.band;
  data.reserved[0] = 0;
  data.reserved[1] = 0;
  data.easting = sys->user_pos.easting;
  data.northing = sys->user_pos.northing;
  data.altitude = sys->user_pos.altitude;

  // 페이로드 CRC32 계산
  data.crc32 = HW_CRC32((const uint8_t*)&data + CRC_DATA_OFFSET, CRC_DATA_SIZE);

  // dirty-check: 현재 값과 Flash 저장값 비교 (동일하면 erase 스킵 -> 수명 보호)
  if (memcmp(&data, (const void*)FLASH_STORAGE_ADDR, sizeof(Flash_SaveData_t)) == 0) {
    DBG_PRINT("[FLASH] Data unchanged, save skipped.\r\n");
    return;
  }

  // 플래시 잠금 해제
  HAL_FLASH_Unlock();

  // 섹터 7 소거
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError;

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FLASH_SECTOR_7;
  EraseInitStruct.NbSectors = 1;

  // erase 1~2초 소요 — IWDG 타임아웃(~2s) 전에 리프레시
  HAL_IWDG_Refresh(&hiwdg);

  uint32_t t0 = DWT->CYCCNT;
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
    DBG_PRINT("[FLASH] Erase Error (Code: %lu)\r\n", (unsigned long)SectorError);
    HAL_FLASH_Lock();
    return;
  }
  prof_erase_cycles = DWT->CYCCNT - t0;

  // 데이터 쓰기 (바이트 단위)
  uint8_t *ptr = (uint8_t*)&data;
  uint32_t addr = FLASH_STORAGE_ADDR;

  // 부분 쓰기 실패 추적
  uint8_t write_ok = 1;
  for (int i = 0; i < (int)sizeof(Flash_SaveData_t); i++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr + (uint32_t)i, ptr[i]) != HAL_OK) {
      DBG_PRINT("[FLASH] Write Error at offset %d\r\n", i);
      write_ok = 0;
      break;
    }
  }

  HAL_FLASH_Lock();
  if (write_ok) {
    DBG_PRINT("[FLASH] Save complete (CRC=0x%08lX).\r\n", (unsigned long)data.crc32);
  } else {
    DBG_PRINT("[FLASH] Save INCOMPLETE — data may be corrupted.\r\n");
  }
}
