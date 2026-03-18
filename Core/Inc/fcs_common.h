/**
 * @file    fcs_common.h
 * @brief   FCS 프로젝트 공용 타입 정의, 열거형, 매크로.
 */
#ifndef __FCS_COMMON_H
#define __FCS_COMMON_H

#include <stdint.h>
#include <stdio.h>

// 디버그 출력
#ifdef DEBUG
  #define DBG_PRINT(...) printf(__VA_ARGS__)
#else
  #define DBG_PRINT(...) ((void)0)
#endif

// 1. 공용 열거형
typedef enum {
  UI_BOOT,
  UI_BATTERY_POS,  // 포 위치 설정        [BATTERY POS]
  UI_LAYING_DATA,  // 방열 제원 설정      [LAYING DATA]
  UI_STANDBY,      // 대기 & 환경 센서    [STANDBY]
  UI_TARGET_LOCK,  // 임무 확인           [MSN CHECK]
  UI_FIRE_DATA,    // 사격 제원 표시      [FIRE DATA]
  UI_ADJUST_FIRE   // 수정 사격           [ADJUST FIRE]
} UI_State_t;

typedef enum {
  KEY_NONE = 0,
  KEY_LEFT,   // ADC < 200 (0V)
  KEY_UP,     // ADC 600~700
  KEY_DOWN,   // ADC 1300~1400
  KEY_RIGHT,  // ADC 2000~2100
  KEY_ENTER   // ADC 2900~3000
} KeyState_t;

// 사격 오류 코드
typedef enum {
  FCS_FIRE_OK = 0,
  FCS_FIRE_ERR_RANGE,     // 무기 최대 사거리 초과
  FCS_FIRE_ERR_CHARGE,    // 현재 장약 한계 초과
  FCS_FIRE_ERR_CALC,      // 계산 실패 (사격표 조회 등)
  FCS_FIRE_ERR_TRAVERSE,  // 전이한계 초과 (방열방위각 재설정 필요)
  FCS_FIRE_ERR_ZONE       // 포진지-표적 UTM 존 불일치 (클라이언트 측 재투영 필요)
} FCS_FireError_t;

// 2. 공용 구조체
typedef struct {
  int zone;
  char band;
  float easting;   // double->float (HW FPU, Korea UTM 범위 오차 0)
  float northing;  // double->float (worst-case 재투영 ~1m < 살상반경 30m)
  float altitude;
} UTM_Coord_t;

typedef struct {
  // 최종 사격 제원
  float azimuth;      // 방위각 (mil)
  float elevation;    // 사각 (mil)
  int charge;         // 장약 (1-7호)
  float distance_km;  // 사거리 (km)
  float time_of_flight; // 비행시간 (초)
  int rounds;         // 발사 탄수
  FCS_FireError_t error; // 계산 오류 상태
  uint8_t auto_select;   // 1 = 다음 계산시 장약 자동선택

  // 검증 데이터 (중간 계산값)
  float map_azimuth;  // 도상 방위각 (mil)
  float map_distance; // 도상 사거리 (m)
  float height_diff;  // 수직 간격 (m)
} FireData_t;

typedef struct {
  float air_temp;     // 기온 (C)
  float air_pressure; // 기압 (hPa)
  float wind_speed;   // 풍속 (m/s)
  float wind_dir;     // 풍향 (mil)
  float prop_temp;    // 장약 온도 (C)
  float weight_diff;  // 탄 중량 편차 (kg, 미구현)
} EnvData_t;

typedef struct {
  uint32_t adc_raw[4]; // [0,1,2]=노브, [3]=키
  KeyState_t key_state;
  uint32_t knob_values[3]; // 처리된 노브 값
} InputData_t;

// 진단 카운터 (HAL 오류 추적)
typedef struct {
  uint16_t i2c_errors;  // 누적 I2C 오류 횟수
  uint8_t  sensor_ok;   // 1=정상, 0=오류
} FCS_DiagCounters_t;

typedef struct {
  // 핵심 데이터
  UTM_Coord_t user_pos;
  UTM_Coord_t tgt_pos;

  FireData_t fire;
  EnvData_t env;
  InputData_t input; // 입력 상태

  // 시스템 상태
  UI_State_t state;
  uint8_t cursor_pos; // UI 탐색 커서
  uint16_t mask_angle; // 차폐각
  uint16_t laying_azimuth; // 방열방위각 (mil, 0~6400)

  // 진단
  FCS_DiagCounters_t diag;

  // 수정 사격 데이터
  struct {
    int16_t range_m;  // 사거리 수정 (+/- m)
    int16_t az_mil;   // 편각 수정 (+R/-L mil)
  } adj;

  // 서보 구동 상태 (팬틸트)
  struct {
    uint8_t az_angle;  // 방위각 서보 각도 (0~180)
    uint8_t el_angle;  // 고각 서보 각도 (0~180)
    uint8_t enabled;   // 1=활성, 0=비활성
  } servo;

} FCS_System_t;

#endif // __FCS_COMMON_H
