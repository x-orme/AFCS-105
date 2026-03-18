/**
 * @file    bmp280.h
 * @brief   BMP280 기압/온도 센서 드라이버 (I2C).
 */
#ifndef __BMP280_H
#define __BMP280_H

#include "main.h"

/* BMP280 I2C 주소 후보 (SDO=GND: 0x76, SDO=VDD: 0x77) */
#define BMP280_I2C_ADDR_LO  (0x76 << 1)
#define BMP280_I2C_ADDR_HI  (0x77 << 1)

/* 레지스터 */
#define BMP280_REG_TEMP_XLSB   0xFC
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_PRESS_XLSB  0xF9
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_STATUS      0xF3
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_ID          0xD0
#define BMP280_REG_CALIB       0x88

/* 센서 데이터 구조체 */
typedef struct {
  float temperature;
  float pressure;
  float altitude; // 계산된 고도
  int32_t raw_pressure; // 디버그: 원시 ADC 값
  int32_t raw_temperature; // 디버그: 원시 ADC 값
} BMP280_Data_t;

/* BMP280_Read_All 반환 코드 */
#define BMP280_OK           0  // 정상 측정
#define BMP280_FAILSAFE     1  // ADC 포화, 안전값 사용
#define BMP280_I2C_ERROR    2  // I2C 통신 실패

/* 안전값 (표준 대기 기준) */
#define BMP280_FAILSAFE_TEMP   15.0f
#define BMP280_FAILSAFE_PRESS  1013.25f
#define BMP280_FAILSAFE_ALT    113.0f

/* 센서 설정값 */
#define BMP280_SOFT_RESET_CMD      0xB6
#define BMP280_CHIP_ID_BMP280      0x58
#define BMP280_CHIP_ID_BME280      0x60
#define BMP280_CTRL_NORMAL_T1_P1   0x27  /* temp x1, press x1, normal */
#define BMP280_CONFIG_OFF          0x00  /* 필터 꺼짐, 대기 0.5ms */
#define BMP280_ADC_SATURATED       0x80000
#define BMP280_I2C_TIMEOUT_MS      100
#define BMP280_RESET_DELAY_MS      100

/* 함수 */

/** @brief I2C 주소 자동탐지, 보정데이터 읽기, 센서 설정. */
uint8_t BMP280_Init(void);

/** @brief 온도/기압/고도 읽기. @retval BMP280_OK / FAILSAFE / I2C_ERROR */
uint8_t BMP280_Read_All(BMP280_Data_t *data);

// 고도 미사용으로 주석처리
// /** @brief 고도 계산용 QNH 기준 기압 설정. */
// void BMP280_SetQNH(float qnh_hpa);

#endif /* __BMP280_H */
