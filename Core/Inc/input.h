/**
 * @file    input.h
 * @brief   5방향 아날로그 키 입력 스캐너.
 */
#ifndef INC_INPUT_H_
#define INC_INPUT_H_

#include "main.h"
#include "fcs_common.h"

/* 5방향 키 ADC 임계값 (12비트, 불감대 포함) */
#define KEY_THRESH_NONE       3800
#define KEY_THRESH_LEFT        200
#define KEY_THRESH_UP_LO       400
#define KEY_THRESH_UP_HI       900
#define KEY_THRESH_DOWN_LO    1100
#define KEY_THRESH_DOWN_HI    1600
#define KEY_THRESH_RIGHT_LO   1800
#define KEY_THRESH_RIGHT_HI   2300
#define KEY_THRESH_ENTER_LO   2700
#define KEY_THRESH_ENTER_HI   3200

/** @brief ADC 값으로 눌린 버튼 판별. */
KeyState_t Input_Scan(uint32_t adc_value);

#endif /* INC_INPUT_H_ */
