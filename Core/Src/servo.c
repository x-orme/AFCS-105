/**
 * @file    servo.c
 * @brief   SG90 서보 모터 PWM 드라이버 구현.
 *
 * TIM3 CH1 (PA6, 방위각) 및 CH2 (PA7, 고각) 사용.
 * 타이머 설정: Prescaler 839 -> 100kHz, Period 1999 -> 50Hz.
 * 펄스 범위: 50~240 카운트 (0.5~2.4ms), 0~180도 대응.
 */
#include "servo.h"

static TIM_HandleTypeDef *servo_htim = NULL;

/* 슬루잉 상태: 현재 위치 / 목표 위치 */
static uint8_t s_cur_az = 0, s_cur_el = 0;
static uint8_t s_tgt_az = 0, s_tgt_el = 0;

/* 트림 상태: 디버그 시리얼(l/r)로 실시간 조정 가능 */
static int8_t az_trim_live = SERVO_AZ_TRIM;

/** @brief  PWM 직접 쓰기 (슬루잉 상태 변경 없음). */
static void servo_write_pwm(uint8_t channel, uint8_t angle)
{
  uint16_t pulse = (uint16_t)(SERVO_PULSE_MIN
    + ((uint32_t)angle * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / SERVO_ANGLE_MAX);
  if (channel == 1U) {
    __HAL_TIM_SET_COMPARE(servo_htim, TIM_CHANNEL_1, pulse);
  } else if (channel == 2U) {
    __HAL_TIM_SET_COMPARE(servo_htim, TIM_CHANNEL_2, pulse);
  } else {
    /* 유효 채널: 1, 2만 지원 */
  }
}

/**
 * @brief  서보 PWM 출력 초기화 (TIM3 CH1/CH2 시작).
 * @param  htim  TIM 핸들 포인터 (TIM3).
 */
void Servo_Init(TIM_HandleTypeDef *htim)
{
  servo_htim = htim;
  uint8_t home_az = (uint8_t)(SERVO_NEUTRAL_DEG + SERVO_AZ_TRIM);
  uint8_t home_el = 0U;
  uint16_t pulse_az = (uint16_t)(SERVO_PULSE_MIN
    + ((uint32_t)home_az * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / SERVO_ANGLE_MAX);
  uint16_t pulse_el = (uint16_t)(SERVO_PULSE_MIN);  // 0° = 최소 펄스
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pulse_az);
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, pulse_el);
  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
  /* 슬루잉 상태를 원점과 동기화 */
  s_cur_az = home_az; s_tgt_az = home_az;
  s_cur_el = home_el; s_tgt_el = home_el;
}

/**
 * @brief  서보 각도 즉시 설정 (PWM 직접 쓰기).
 * @param  channel  1=방위각, 2=고각
 * @param  angle    도(0-180)
 */
void Servo_SetAngle(uint8_t channel, uint8_t angle)
{
  if (servo_htim == NULL) { return; }
  if (angle > SERVO_ANGLE_MAX) { angle = (uint8_t)SERVO_ANGLE_MAX; }
  else { /* 정상 범위 */ }

  /* 0도=50, 180도=240 선형 매핑 */
  uint16_t pulse = (uint16_t)(SERVO_PULSE_MIN
    + ((uint32_t)angle * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / SERVO_ANGLE_MAX);

  if (channel == 1U) {
    __HAL_TIM_SET_COMPARE(servo_htim, TIM_CHANNEL_1, pulse);
    s_cur_az = angle; s_tgt_az = angle; // 슬루잉 상태 동기화
  } else if (channel == 2U) {
    __HAL_TIM_SET_COMPARE(servo_htim, TIM_CHANNEL_2, pulse);
    s_cur_el = angle; s_tgt_el = angle;
  } else {
    /* 유효 채널: 1, 2만 지원 */
  }
}

/**
 * @brief  슬루잉 목표 각도 설정.
 * @param  channel  1=방위각, 2=고각
 * @param  angle    도(0-180)
 */
void Servo_SetTarget(uint8_t channel, uint8_t angle)
{
  if (angle > SERVO_ANGLE_MAX) { angle = (uint8_t)SERVO_ANGLE_MAX; }
  else { /* 정상 범위 */ }
  if (channel == 1U)      { s_tgt_az = angle; }
  else if (channel == 2U) { s_tgt_el = angle; }
  else { /* 유효 채널: 1, 2만 지원 */ }
}

/**
 * @brief  서보를 목표 방향으로 한 스텝 전진 (틱마다 호출).
 * @param  az_first  1 = 방위각 우선 (포조준), 0 = 고각 우선 (원점 귀환)
 */
void Servo_SlewUpdate(uint8_t az_first)
{
  if (servo_htim == NULL) { return; }

  /* 먼저 이동할 축 / 나중에 이동할 축 결정 */
  uint8_t *cur_1st, *cur_2nd;
  const uint8_t *tgt_1st, *tgt_2nd;
  uint8_t ch_1st, ch_2nd;
  if (az_first) {
    cur_1st = &s_cur_az; tgt_1st = &s_tgt_az; ch_1st = 1U;
    cur_2nd = &s_cur_el; tgt_2nd = &s_tgt_el; ch_2nd = 2U;
  } else {
    cur_1st = &s_cur_el; tgt_1st = &s_tgt_el; ch_1st = 2U;
    cur_2nd = &s_cur_az; tgt_2nd = &s_tgt_az; ch_2nd = 1U;
  }

  /* 1차 축 이동 */
  if (*cur_1st != *tgt_1st) {
    uint8_t diff = (*cur_1st < *tgt_1st)
      ? (*tgt_1st - *cur_1st) : (*cur_1st - *tgt_1st);
    uint8_t step = (diff > SERVO_SLEW_STEP) ? SERVO_SLEW_STEP : diff;
    if (*cur_1st < *tgt_1st) { *cur_1st += step; }
    else                     { *cur_1st -= step; }
    servo_write_pwm(ch_1st, *cur_1st);
  } else {
    /* 1차 축 도달 완료 */
  }

  /* 2차 축: 1차 축 도달 후 이동 */
  if ((*cur_1st == *tgt_1st) && (*cur_2nd != *tgt_2nd)) {
    uint8_t diff = (*cur_2nd < *tgt_2nd)
      ? (*tgt_2nd - *cur_2nd) : (*cur_2nd - *tgt_2nd);
    uint8_t step = (diff > SERVO_SLEW_STEP) ? SERVO_SLEW_STEP : diff;
    if (*cur_2nd < *tgt_2nd) { *cur_2nd += step; }
    else                     { *cur_2nd -= step; }
    servo_write_pwm(ch_2nd, *cur_2nd);
  } else {
    /* 양축 도달 또는 1차 축 이동 중 */
  }
}

/**
 * @brief  방위각 트림 조정 (디버그 전용).
 * @param  delta  트림 스텝 (±1)
 */
#ifdef DEBUG
void Servo_AdjustTrim(int8_t delta)
{
  int16_t next = (int16_t)az_trim_live + (int16_t)delta;
  if (next < -SERVO_TRIM_LIMIT) next = -SERVO_TRIM_LIMIT;
  else { /* 하한 정상 */ }
  if (next >  SERVO_TRIM_LIMIT) next =  SERVO_TRIM_LIMIT;
  else { /* 상한 정상 */ }
  az_trim_live = (int8_t)next;
}
#endif

/**
 * @brief  현재 방위각 트림 값 반환.
 * @retval 현재 트림 오프셋 (기본값: SERVO_AZ_TRIM).
 */
int8_t Servo_GetTrim(void)
{
  return az_trim_live;
}

/**
 * @brief  사격 제원을 서보 각도로 변환 (방위각 + 고각).
 * @param  sys  fire.azimuth/elevation/laying_azimuth 읽기,
 *              servo.az_angle/el_angle 쓰기.
 * @note   fire.error == FCS_FIRE_OK일 때만 호출할 것.
 */
void Servo_UpdateFromFireData(FCS_System_t *sys)
{
  float delta_az = sys->fire.azimuth - (float)sys->laying_azimuth;
  if (delta_az > SERVO_MIL_HALF) delta_az -= SERVO_MIL_FULL;
  else { /* 정방향 범위 내 */ }
  if (delta_az < -SERVO_MIL_HALF) delta_az += SERVO_MIL_FULL;
  else { /* 역방향 범위 내 */ }
  float servo_az = ((float)SERVO_NEUTRAL_DEG + (float)az_trim_live)
    - (delta_az * (float)SERVO_NEUTRAL_DEG) / SERVO_AZ_MIL_90;
  if (servo_az < 0.0f) servo_az = 0.0f;
  else { /* 하한 정상 */ }
  if (servo_az > (float)SERVO_ANGLE_MAX) servo_az = (float)SERVO_ANGLE_MAX;
  else { /* 상한 정상 */ }
  sys->servo.az_angle = (uint8_t)servo_az;

  // 클램프 후 캐스팅 (음수 -> unsigned 방지)
  float el_deg = (sys->fire.elevation * (float)SERVO_ANGLE_MAX) / SERVO_MIL_HALF;
  if (el_deg < 0.0f) el_deg = 0.0f;
  else { /* 하한 정상 */ }
  if (el_deg > (float)SERVO_ANGLE_MAX) el_deg = (float)SERVO_ANGLE_MAX;
  else { /* 상한 정상 */ }
  sys->servo.el_angle = (uint8_t)el_deg;
}
