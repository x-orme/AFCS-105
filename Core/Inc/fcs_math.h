/**
 * @file    fcs_math.h
 * @brief   탄도 계산 모듈.
 */
#ifndef FCS_MATH_H
#define FCS_MATH_H

#include "fcs_common.h" // UTM_Coord_t, FCS_System_t 사용

/** @brief 사격 제원 산출 (방위각, 고각, 비행시간, 장약). */
void FCS_Calculate_FireData(FCS_System_t *sys);

#endif
