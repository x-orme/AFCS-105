/**
 * @file    bmp280.c
 * @brief   BMP280 I2C 드라이버 — 주소 자동탐지, 보정데이터 읽기,
 *          온도/기압 보상 계산, 기압 고도 산출.
 */
#include "bmp280.h"
#include "fcs_common.h" // DBG_PRINT 사용
#include "i2c.h"
// 고도 미사용 — powf(soft-float) 제거로 코드 크기 절감
// #include <math.h>

// 보정 파라미터
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
// 고도 미사용으로 QNH 기준값도 주석처리
// static float    BMP280_Ref_Pressure = 1013.25f; // 표준 대기압 기본값
static uint16_t bmp280_addr = BMP280_I2C_ADDR_LO; // 초기화 시 결정

/** @brief I2C 읽기 래퍼 — HAL 상태 반환 (에러 감지용). */
static HAL_StatusTypeDef BMP280_ReadShim(uint8_t reg, uint8_t *data, uint8_t len) {
  return HAL_I2C_Mem_Read(&hi2c1, bmp280_addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, BMP280_I2C_TIMEOUT_MS);
}

/** @brief I2C 쓰기 래퍼. */
static void BMP280_WriteShim(uint8_t reg, uint8_t value) {
  HAL_I2C_Mem_Write(&hi2c1, bmp280_addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, BMP280_I2C_TIMEOUT_MS);
}

/**
 * @brief  BMP280 초기화 (주소 탐지 + 설정).
 * @retval 0 성공, 1 실패
 */
uint8_t BMP280_Init(void) {
  uint8_t chipID;

  // I2C 주소 자동 탐지 (0x76 또는 0x77)
  static const uint16_t addrs[] = { BMP280_I2C_ADDR_LO, BMP280_I2C_ADDR_HI };
  uint8_t found = 0;
  for (int i = 0; i < 2; i++) {
    bmp280_addr = addrs[i];
    BMP280_WriteShim(BMP280_REG_RESET, BMP280_SOFT_RESET_CMD);
    HAL_Delay(BMP280_RESET_DELAY_MS);
    chipID = 0;
    HAL_StatusTypeDef rc = BMP280_ReadShim(BMP280_REG_ID, &chipID, 1);
    if ((rc == HAL_OK) && ((chipID == BMP280_CHIP_ID_BMP280) || (chipID == BMP280_CHIP_ID_BME280))) {
      found = 1;
      break;
    }
  }
  if (!found) {
    DBG_PRINT("[BMP] No sensor found\r\n");
    return 1;
  }
  DBG_PRINT("[BMP] Found at 0x%02X (ID=0x%02X)\r\n", bmp280_addr >> 1, chipID);

  // 보정 데이터 읽기
  uint8_t calib[24];
  // calib 읽기 실패 시 보상 공식이 쓰레기값 출력 -> 초기화 실패 처리
  if (BMP280_ReadShim(BMP280_REG_CALIB, calib, 24) != HAL_OK) {
    DBG_PRINT("[BMP] Calib read failed\r\n");
    return 1;
  }

  dig_T1 = (uint16_t)(((uint32_t)calib[1] << 8) | (uint32_t)calib[0]);
  dig_T2 = (int16_t)(((uint32_t)calib[3] << 8) | (uint32_t)calib[2]);
  dig_T3 = (int16_t)(((uint32_t)calib[5] << 8) | (uint32_t)calib[4]);
  dig_P1 = (uint16_t)(((uint32_t)calib[7] << 8) | (uint32_t)calib[6]);
  dig_P2 = (int16_t)(((uint32_t)calib[9] << 8) | (uint32_t)calib[8]);
  dig_P3 = (int16_t)(((uint32_t)calib[11] << 8) | (uint32_t)calib[10]);
  dig_P4 = (int16_t)(((uint32_t)calib[13] << 8) | (uint32_t)calib[12]);
  dig_P5 = (int16_t)(((uint32_t)calib[15] << 8) | (uint32_t)calib[14]);
  dig_P6 = (int16_t)(((uint32_t)calib[17] << 8) | (uint32_t)calib[16]);
  dig_P7 = (int16_t)(((uint32_t)calib[19] << 8) | (uint32_t)calib[18]);
  dig_P8 = (int16_t)(((uint32_t)calib[21] << 8) | (uint32_t)calib[20]);
  dig_P9 = (int16_t)(((uint32_t)calib[23] << 8) | (uint32_t)calib[22]);

  // 센서 설정
  BMP280_WriteShim(BMP280_REG_CONFIG, BMP280_CONFIG_OFF);
  BMP280_WriteShim(BMP280_REG_CTRL_MEAS, BMP280_CTRL_NORMAL_T1_P1);

  return 0; // 초기화 성공
}

/**
 * @brief  온도/기압 센서 읽기.
 * @param  data  센서 출력 (실패 시 기본값)
 * @retval 0=OK, 1=failsafe, 2=I2C 에러
 */
uint8_t BMP280_Read_All(BMP280_Data_t *data) {
  // 원시 데이터 읽기 (기압 MSB...온도 XLSB)
  uint8_t raw[6];
  HAL_StatusTypeDef hal_rc = BMP280_ReadShim(BMP280_REG_PRESS_MSB, raw, 6);

  // I2C 통신 실패 -> 안전값 적용
  if (hal_rc != HAL_OK) {
    data->temperature = BMP280_FAILSAFE_TEMP;
    data->pressure = BMP280_FAILSAFE_PRESS;
    data->altitude = BMP280_FAILSAFE_ALT;
    data->raw_pressure = 0;
    data->raw_temperature = 0;
    return BMP280_I2C_ERROR;
  }

  int32_t adc_P = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | ((uint32_t)raw[2] >> 4));
  int32_t adc_T = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | ((uint32_t)raw[5] >> 4));

  // 원시 데이터 저장 (디버그)
  data->raw_pressure = adc_P;
  data->raw_temperature = adc_T;

  if (adc_P >= BMP280_ADC_SATURATED) {
    data->pressure = BMP280_FAILSAFE_PRESS;
    data->altitude = BMP280_FAILSAFE_ALT;

    float var1, var2, t_fine;
    var1 = (((float)adc_T) / 16384.0f - ((float)dig_T1) / 1024.0f) * ((float)dig_T2);
    var2 = ((((float)adc_T) / 131072.0f - ((float)dig_T1) / 8192.0f) * (((float)adc_T) / 131072.0f - ((float)dig_T1) / 8192.0f)) * ((float)dig_T3);
    t_fine = var1 + var2;
    data->temperature = t_fine / 5120.0f;

    return BMP280_FAILSAFE;
  }

  float var1, var2, t_fine;

  // 온도 계산
  var1 = (((float)adc_T) / 16384.0f - ((float)dig_T1) / 1024.0f) * ((float)dig_T2);
  var2 = ((((float)adc_T) / 131072.0f - ((float)dig_T1) / 8192.0f) * (((float)adc_T) / 131072.0f - ((float)dig_T1) / 8192.0f)) * ((float)dig_T3);
  t_fine = var1 + var2;
  data->temperature = t_fine / 5120.0f;

  // 기압 계산
  var1 = (t_fine / 2.0f) - 64000.0f;
  var2 = var1 * var1 * ((float)dig_P6) / 32768.0f;
  var2 = var2 + var1 * ((float)dig_P5) * 2.0f;
  var2 = (var2 / 4.0f) + (((float)dig_P4) * 65536.0f);
  var1 = (((float)dig_P3) * var1 * var1 / 524288.0f + ((float)dig_P2) * var1) / 524288.0f;
  var1 = (1.0f + var1 / 32768.0f) * ((float)dig_P1);

  if (var1 == 0.0f) {
    data->pressure = 0.0f;
  } else {
    float p = 1048576.0f - (float)adc_P;
    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
    var1 = ((float)dig_P9) * p * p / 2147483648.0f;
    var2 = p * ((float)dig_P8) / 32768.0f;
    p = p + (var1 + var2 + ((float)dig_P7)) / 16.0f;

    data->pressure = p / 100.0f; // Pa -> hPa
  }

  // FCS에서 고도 미사용 -> powf(soft-float) 제거로 코드 크기 절감
  // 원본 고도 계산 (h = 44330 * (1 - (P/P0)^(1/5.255)))
  // if (data->pressure > 0) {
  //   data->altitude = 44330.0f * (1.0f - powf(data->pressure / BMP280_Ref_Pressure, 0.1903f));
  // } else {
  //   data->altitude = 0.0f;
  // }
  data->altitude = 0.0f;

  return BMP280_OK;
}

// 고도 미사용으로 QNH 설정 함수도 주석처리
// /**
//  * @brief  고도 계산용 QNH 기준 기압을 설정한다.
//  * @param  qnh_hpa  기준 기압 (hPa, 유효 범위: 800–1200)
//  */
// void BMP280_SetQNH(float qnh_hpa) {
//   if (qnh_hpa > 800.0f && qnh_hpa < 1200.0f) {
//     BMP280_Ref_Pressure = qnh_hpa;
//   }
// }
