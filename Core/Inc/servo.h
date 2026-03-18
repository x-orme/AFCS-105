/**
 * @file    servo.h
 * @brief   SG90 서보 모터 PWM 드라이버 (팬틸트 구동부: 방위각/고각).
 */
#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h"
#include "fcs_common.h"

/* SG90 펄스 폭 (100kHz 타이머, 1 카운트 = 10us)
 * 0.5ms = 50 카운트 (0도), 2.4ms = 240 카운트 (180도) */
#define SERVO_PULSE_MIN    50U   /* 0.5ms -> 0도 */
#define SERVO_PULSE_MAX   240U   /* 2.4ms -> 180도 */
#define SERVO_ANGLE_MAX   180U
#define SERVO_AZ_TRIM    (-11)  /* 방위각 중립 보정값 (캘리브레이션 완료) */
#define SERVO_AZ_TRAVERSE_MIL  1400  /* 전이한계: 트림 보정 중앙(79°) 기준 대칭 한계 */
#define SERVO_SLEW_STEP    1U  /* 슬루잉 속도: 1°/tick (50°/s at 50Hz) */
#define SERVO_NEUTRAL_DEG  90U /* 방위각 중립 */
#define SERVO_TRIM_LIMIT   30  /* 트림 클램프 ±30° */
#define SERVO_AZ_MIL_90   1600.0f /* 방위각: 1600mil = 90° */
#define SERVO_MIL_HALF     3200.0f /* 반원 (고각 변환 + 순환 보정) */
#define SERVO_MIL_FULL     6400.0f /* 1회전 */

/**
 * @brief  서보 PWM 출력 초기화 (TIM3 CH1/CH2 시작).
 * @param  htim  TIM 핸들 포인터 (TIM3).
 */
void Servo_Init(TIM_HandleTypeDef *htim);

/**
 * @brief  서보 각도 즉시 설정 (PWM 직접 쓰기).
 * @param  channel  1=방위각, 2=고각
 * @param  angle    도(0-180)
 */
void Servo_SetAngle(uint8_t channel, uint8_t angle);

/**
 * @brief  슬루잉 목표 각도 설정.
 * @param  channel  1=방위각, 2=고각
 * @param  angle    도(0-180)
 */
void Servo_SetTarget(uint8_t channel, uint8_t angle);

/**
 * @brief  서보를 목표 방향으로 한 스텝 전진 (틱마다 호출).
 * @param  az_first  1 = 방위각 우선 (포조준), 0 = 고각 우선 (원점 귀환)
 */
void Servo_SlewUpdate(uint8_t az_first);

/**
 * @brief  방위각 트림 조정 (디버그 전용).
 * @param  delta  트림 스텝 (±1)
 */
#ifdef DEBUG
void Servo_AdjustTrim(int8_t delta);
#endif

/**
 * @brief  현재 방위각 트림 값 반환.
 */
int8_t Servo_GetTrim(void);

/**
 * @brief  사격 제원(방위각/고각)을 서보 각도로 변환.
 * @param  sys  시스템 컨텍스트 (사격 제원 읽기, 서보 각도 쓰기).
 * @note   fire.error == FCS_FIRE_OK일 때만 호출할 것.
 */
void Servo_UpdateFromFireData(FCS_System_t *sys);

#endif /* __SERVO_H */
