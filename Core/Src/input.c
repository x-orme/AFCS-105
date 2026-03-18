/**
 * @file    input.c
 * @brief   5방향 아날로그 키 스캐너 (불감대 임계값 적용).
 */
#include "input.h"

/**
 * @brief  단일 ADC 값으로 눌린 버튼 판별.
 * @param  adc_value  5방향 키 채널의 12비트 ADC 원시값
 * @retval KeyState_t  KEY_NONE / LEFT / UP / DOWN / RIGHT / ENTER
 */
KeyState_t Input_Scan(uint32_t adc_value) {
  if (adc_value > KEY_THRESH_NONE)                                      return KEY_NONE;
  if (adc_value < KEY_THRESH_LEFT)                                      return KEY_LEFT;
  if ((adc_value > KEY_THRESH_UP_LO)    && (adc_value < KEY_THRESH_UP_HI))    return KEY_UP;
  if ((adc_value > KEY_THRESH_DOWN_LO)  && (adc_value < KEY_THRESH_DOWN_HI))  return KEY_DOWN;
  if ((adc_value > KEY_THRESH_RIGHT_LO) && (adc_value < KEY_THRESH_RIGHT_HI)) return KEY_RIGHT;
  if ((adc_value > KEY_THRESH_ENTER_LO) && (adc_value < KEY_THRESH_ENTER_HI)) return KEY_ENTER;

  return KEY_NONE;
}
