/**
 * @file    fcs_lut.h
 * @brief   Flash LUT 기반 UTM 존 재투영 (하이브리드: LUT trig + float 산술).
 *
 * Q1.30 sin LUT(4KB Flash)로 삼각함수만 대체하고,
 * 나머지 Snyder 급수 산술은 float HW FPU로 처리.
 * 재투영 오버헤드 ~2,000 사이클, 정밀도 ≤1m.
 */
#ifndef FCS_LUT_H
#define FCS_LUT_H

#include <stdint.h>

/* 존 재투영: UTM(zone A) -> LatLon -> UTM(zone B) */
void FCS_Reproject(float src_e, float src_n, int src_zone,
                   int dst_zone, float *out_e, float *out_n);

#endif
