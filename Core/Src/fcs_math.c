/**
 * @file    fcs_math.c
 * @brief   탄도 계산 엔진 — 사격표 조회, 기상 보정, 코리올리 효과.
 */
#include "fcs_math.h"
#include "fcs_lut.h"
#include <math.h>
#include <string.h>

/* 시뮬레이션용 근사 사격표 (105mm 공개 탄도 특성 기반).
 * 실 사격표는 별도 데이터로 교체. */

// 탄도 상수
#define STD_AIR_TEMP_C        15.0f
#define STD_PROP_TEMP_C       21.0f
#define STD_PRESSURE_HPA      1013.25f
#define MIL_PER_CIRCLE        6400.0f
#define CORIOLIS_RANGE_FACTOR 0.5f
#define CORIOLIS_AZ_FACTOR    0.1f
#define MIN_RANGE_M           100.0f

// Live Expression 추적용 (재투영 결과)
volatile float dbg_reproj_e;
volatile float dbg_reproj_n;

/* 시뮬레이션 사격표 — K105A1 (105mm), M1 고폭탄, 장약 1~7호.
 * 실 사격표는 별도 데이터로 교체 가능한 구조. */

typedef struct {
    float range_m;
    float elev_mil;
    float drift_mil; // 우편류
    float c_factor;  // 고저계수 (mil/10m)
} FiringTable_Row_t;

// --- Charge 1 (V0=211m/s, 1.1~3.1km, 5 points) ---
static const FiringTable_Row_t FT_Ch1[] = {
  {  1100.0f,   95.0f,  0.3f,  5.8f },
  {  1600.0f,  175.0f,  1.0f,  4.8f },
  {  2100.0f,  315.0f,  2.0f,  3.8f },
  {  2600.0f,  520.0f,  3.4f,  3.0f },
  {  3100.0f,  800.0f,  5.0f,  2.4f }
};
#define SZ_Ch1 (sizeof(FT_Ch1)/sizeof(FT_Ch1[0]))

// --- Charge 2 (V0=253m/s, 1.4~4.0km, 6 points) ---
static const FiringTable_Row_t FT_Ch2[] = {
  {  1400.0f,   82.0f,  0.4f,  5.5f },
  {  1900.0f,  140.0f,  1.0f,  4.7f },
  {  2400.0f,  220.0f,  1.8f,  4.0f },
  {  2900.0f,  330.0f,  2.8f,  3.3f },
  {  3400.0f,  490.0f,  3.9f,  2.7f },
  {  4000.0f,  780.0f,  5.5f,  2.1f }
};
#define SZ_Ch2 (sizeof(FT_Ch2)/sizeof(FT_Ch2[0]))

// --- Charge 3 (V0=293m/s, 1.8~5.0km, 7 points) ---
static const FiringTable_Row_t FT_Ch3[] = {
  {  1800.0f,   78.0f,  0.5f,  5.2f },
  {  2300.0f,  125.0f,  1.1f,  4.5f },
  {  2800.0f,  185.0f,  1.9f,  3.9f },
  {  3300.0f,  268.0f,  2.8f,  3.3f },
  {  3800.0f,  375.0f,  3.8f,  2.8f },
  {  4400.0f,  530.0f,  5.0f,  2.3f },
  {  5000.0f,  780.0f,  6.0f,  1.9f }
};
#define SZ_Ch3 (sizeof(FT_Ch3)/sizeof(FT_Ch3[0]))

// --- Charge 4 (V0=334m/s, 2.2~6.1km, 8 points) ---
static const FiringTable_Row_t FT_Ch4[] = {
  {  2200.0f,   75.0f,  0.6f,  4.8f },
  {  2800.0f,  118.0f,  1.3f,  4.2f },
  {  3300.0f,  170.0f,  2.1f,  3.6f },
  {  3900.0f,  250.0f,  3.0f,  3.1f },
  {  4400.0f,  338.0f,  4.0f,  2.7f },
  {  5000.0f,  455.0f,  5.2f,  2.3f },
  {  5500.0f,  580.0f,  6.3f,  2.0f },
  {  6100.0f,  790.0f,  7.0f,  1.7f }
};
#define SZ_Ch4 (sizeof(FT_Ch4)/sizeof(FT_Ch4[0]))

// --- Charge 5 (V0=381m/s, 2.7~7.5km, 8 points) ---
static const FiringTable_Row_t FT_Ch5[] = {
  {  2700.0f,   72.0f,  0.7f,  4.5f },
  {  3400.0f,  115.0f,  1.5f,  3.9f },
  {  4000.0f,  165.0f,  2.4f,  3.4f },
  {  4700.0f,  238.0f,  3.4f,  2.9f },
  {  5300.0f,  325.0f,  4.4f,  2.5f },
  {  6000.0f,  438.0f,  5.6f,  2.1f },
  {  6800.0f,  598.0f,  7.0f,  1.8f },
  {  7500.0f,  790.0f,  8.0f,  1.5f }
};
#define SZ_Ch5 (sizeof(FT_Ch5)/sizeof(FT_Ch5[0]))

// --- Charge 6 (V0=434m/s, 3.2~9.1km, 8 points) ---
static const FiringTable_Row_t FT_Ch6[] = {
  {  3200.0f,   68.0f,  0.7f,  4.2f },
  {  4000.0f,  108.0f,  1.5f,  3.7f },
  {  5000.0f,  178.0f,  2.6f,  3.1f },
  {  5800.0f,  255.0f,  3.6f,  2.7f },
  {  6600.0f,  352.0f,  4.8f,  2.3f },
  {  7500.0f,  482.0f,  6.3f,  1.9f },
  {  8300.0f,  628.0f,  8.0f,  1.6f },
  {  9100.0f,  800.0f, 10.0f,  1.3f }
};
#define SZ_Ch6 (sizeof(FT_Ch6)/sizeof(FT_Ch6[0]))

// --- Charge 7 (V0=502m/s, 4.0~11.3km, 8 points) ---
static const FiringTable_Row_t FT_Ch7[] = {
  {  4000.0f,   65.0f,  0.8f,  3.8f },
  {  5000.0f,  105.0f,  1.6f,  3.3f },
  {  6100.0f,  165.0f,  2.7f,  2.8f },
  {  7100.0f,  240.0f,  3.8f,  2.4f },
  {  8200.0f,  338.0f,  5.2f,  2.0f },
  {  9200.0f,  452.0f,  6.8f,  1.7f },
  { 10300.0f,  608.0f,  8.8f,  1.4f },
  { 11300.0f,  800.0f, 12.0f,  1.1f }
};
#define SZ_Ch7 (sizeof(FT_Ch7)/sizeof(FT_Ch7[0]))


// 사격표 레지스트리
// 인덱스 0은 더미, 1-7은 장약 번호에 대응
static const FiringTable_Row_t* FT_DB[8] = {
  NULL, FT_Ch1, FT_Ch2, FT_Ch3, FT_Ch4, FT_Ch5, FT_Ch6, FT_Ch7
};
static const int FT_Sizes[8] = {
  0, SZ_Ch1, SZ_Ch2, SZ_Ch3, SZ_Ch4, SZ_Ch5, SZ_Ch6, SZ_Ch7
};

// 장약별 포구 속도 (m/s)
// 비행시간(TOF) 계산에 사용
static const float MUZZLE_VELOCITY[8] = {
  0.0f,    // 인덱스 0: 미사용
  211.0f,  // 1호 장약
  253.0f,  // 2호 장약
  293.0f,  // 3호 장약
  334.0f,  // 4호 장약
  381.0f,  // 5호 장약
  434.0f,  // 6호 장약
  502.0f   // 7호 장약
};

/**
 * @brief  풍속 기반 사거리/방위각 보정량 계산.
 * @param  wind_speed  풍속 (m/s)
 * @param  wind_dir    풍향 (mil, FROM 기준)
 * @param  fire_az     사격 방위각 (mil)
 * @param  map_dist_m  도상 거리 (m)
 * @param  corr_range  [출력] 사거리 보정량 (m)
 * @param  corr_az     [출력] 방위각 보정량 (mil)
 * @note   wind_speed < 0.1 m/s 시 보정 0 반환.
 */
static void apply_wind_correction(float wind_speed, float wind_dir,
                                  float fire_az, float map_dist_m,
                                  float *corr_range, float *corr_az) {
  *corr_range = 0.0f;
  *corr_az = 0.0f;
  if (wind_speed < 0.1f) return;

  // 상대 풍향각 (풍향 FROM vs 사격 TO)
  float rel_angle = wind_dir - fire_az + (MIL_PER_CIRCLE / 2.0f); // +3200 = 180도 (정풍)
  // fmodf 정규화 (while 루프 대비 반복 횟수 고정)
  rel_angle = fmodf(rel_angle, MIL_PER_CIRCLE);
  if (rel_angle < 0.0f) rel_angle += MIL_PER_CIRCLE;
  else { /* 정상 범위 */ }

  float rel_rad = rel_angle * (3.14159265f / (MIL_PER_CIRCLE / 2.0f));
  float headwind = wind_speed * cosf(rel_rad);  // +: 정풍, -: 배풍
  float crosswind = wind_speed * sinf(rel_rad); // +: 우횡풍, -: 좌횡풍

  // 사거리: 정풍 -> 탄 낙하 단축 -> +보정
  // 근사: 정풍 1m/s -> +0.3% 사거리 보정/km
  *corr_range = headwind * 0.003f * map_dist_m;

  // 방위각: 횡풍 -> 편류
  // 근사: 횡풍 1m/s -> 0.5 mil/km
  *corr_az = -crosswind * 0.5f * (map_dist_m / 1000.0f);
}

/** @brief  선형 보간. */
static float L_Interp(float x1, float y1, float x2, float y2, float x) {
  if (x2 == x1) return y1;
  return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
}


/**
 * @brief  주 탄도 계산 — 표준 포병 사격 제원 산출 절차.
 *
 * 단계: 측지 계산 -> 기상 보정 -> 코리올리 -> 사격표 조회(보간) ->
 * 고저 보정 -> 최종 제원 편성.
 *
 * @param  sys  FCS 전역 구조체 (읽기: user_pos, tgt_pos, env, fire.charge;
 *              쓰기: fire.azimuth, fire.elevation, fire.distance_km,
 *              fire.time_of_flight, fire.error)
 */
void FCS_Calculate_FireData(FCS_System_t *sys) {
  sys->fire.error = FCS_FIRE_OK;

  const UTM_Coord_t *batt = &sys->user_pos;
  const UTM_Coord_t *tgt  = &sys->tgt_pos;
    
  // 도상 데이터 계산
  UTM_Coord_t tgt_proj;

  // 존 불일치 시 LUT 고정소수점 재투영 (Flash 4KB, double 연산 0회)
  if (batt->zone != tgt->zone) {
    FCS_Reproject(tgt->easting, tgt->northing, tgt->zone,
                  batt->zone, &tgt_proj.easting, &tgt_proj.northing);
    tgt_proj.zone = batt->zone;
    tgt_proj.band = tgt->band;
    tgt_proj.altitude = tgt->altitude;
    dbg_reproj_e = tgt_proj.easting;
    dbg_reproj_n = tgt_proj.northing;
    DBG_PRINT("[REPROJ] %d->%d E:%d N:%d\r\n",
              tgt->zone, batt->zone,
              (int)tgt_proj.easting, (int)tgt_proj.northing);
  } else {
    tgt_proj = *tgt;
  }

  // float 연산 (HW FPU, Korea UTM 운용 범위에서 무시 가능한 오차)
  float dx = tgt_proj.easting - batt->easting;
  float dy = tgt_proj.northing - batt->northing;
  float map_dist_m = sqrtf(dx * dx + dy * dy);

  float az_rad = atan2f(dx, dy);
  float map_az_deg = az_rad * (180.0f / 3.14159265f);
  if (map_az_deg < 0.0f) map_az_deg += 360.0f;
  else { /* 정상 범위 */ }

  float map_az_mil = map_az_deg * (MIL_PER_CIRCLE / 360.0f);
  float vi_m = tgt->altitude - batt->altitude; // 수직 간격 (+: 상향, -: 하향)

  // 기상/장약/기압/풍 보정
    
  float corr_dist_m = 0.0f;
    
  // 기온 보정: +10C -> -1% 사거리
  float temp_diff = sys->env.air_temp - STD_AIR_TEMP_C;
  corr_dist_m -= map_dist_m * 0.001f * temp_diff;

  // 장약 온도 보정: +10C -> -2% 사거리
  float prop_diff = sys->env.prop_temp - STD_PROP_TEMP_C;
  corr_dist_m -= map_dist_m * 0.002f * prop_diff;

  // 기압 보정: +10hPa -> +0.5% 사거리
  float press_diff = sys->env.air_pressure - STD_PRESSURE_HPA;
  corr_dist_m += map_dist_m * 0.00005f * press_diff;

  // 풍보정
  float wind_corr_range = 0.0f;
  float wind_corr_az = 0.0f;
  apply_wind_correction(sys->env.wind_speed, sys->env.wind_dir,
                        map_az_mil, map_dist_m,
                        &wind_corr_range, &wind_corr_az);
  corr_dist_m += wind_corr_range;

  // 수정 사격 반영
  float final_lookup_range = map_dist_m + corr_dist_m + (float)sys->adj.range_m;
    
  // 사거리 클램프
  if (final_lookup_range < 0.0f) final_lookup_range = MIN_RANGE_M;
  else { /* 정상 범위 */ }

  // 장약 선택
  int chg_idx;

  if (sys->fire.auto_select) {
    sys->fire.auto_select = 0;

    // 자동선택: 유효한 최대 장약 (7->1)
    // 높은 장약 = 높은 속도 = 낮은 공산오차
    chg_idx = 0;
    for (int c = 7; c >= 1; c--) {
      const FiringTable_Row_t *ft = FT_DB[c];
      int sz = FT_Sizes[c];
      if (ft == NULL || sz < 2) continue;

      // 사거리 유효성 검사
      if ((final_lookup_range < ft[0].range_m) ||
          (final_lookup_range > ft[sz - 1].range_m)) continue;

      // 차폐각 검사를 위한 사각 보간
      int aidx = -1;
      for (int i = 0; i < sz - 1; i++) {
        if ((final_lookup_range >= ft[i].range_m) &&
            (final_lookup_range <= ft[i + 1].range_m)) {
          aidx = i;
          break;
        }
      }
      if (aidx == -1) continue;

      float elev = L_Interp(ft[aidx].range_m, ft[aidx].elev_mil,
                             ft[aidx + 1].range_m, ft[aidx + 1].elev_mil,
                             final_lookup_range);

      // 차폐각 검사 (사각이 차폐각 이상이어야 함)
      if (elev < (float)sys->mask_angle) continue;

      chg_idx = c;
      break;
    }

    // 대체: 유효 장약 없음 -> 7호 사용, 오류 처리에 위임
    if (chg_idx == 0) chg_idx = 7;
    else { /* 유효 장약 선택됨 */ }
    sys->fire.charge = chg_idx;

  } else {
    chg_idx = sys->fire.charge;
    if (chg_idx < 1) chg_idx = 1;
    else { /* 하한 정상 */ }
    if (chg_idx > 7) chg_idx = 7;
    else { /* 상한 정상 */ }
    sys->fire.charge = chg_idx;
  }

  // 코리올리 효과 보정
  // 위도 근사: northing/111000 (double UTM역변환 불필요)
  float lat_deg_approx = tgt->northing / 111000.0f;
  float lat_rad = lat_deg_approx * (3.14159265f / 180.0f);

  float tof = map_dist_m / MUZZLE_VELOCITY[chg_idx];

  // 사거리 보정 (자전 지연)
  float cori_range_m = CORIOLIS_RANGE_FACTOR * (map_dist_m / 1000.0f) * tof * sinf(lat_rad) * sinf(az_rad);
  final_lookup_range += cori_range_m;

  // 방위각 보정 (북반구 우편류 -> 좌 조준)
  float cori_az_mil = CORIOLIS_AZ_FACTOR * tof * sinf(lat_rad);
  float cori_az_corr = -cori_az_mil;

  // 사격표 조회 + 보간
  const FiringTable_Row_t *current_ft = FT_DB[chg_idx];
  int current_size = FT_Sizes[chg_idx];

  // 테이블 유효성 검증
  if ((current_ft == NULL) || (current_size < 2)) {
    sys->fire.error = FCS_FIRE_ERR_CALC;
    return;
  }
  if ((FT_DB[7] == NULL) || (FT_Sizes[7] < 1)) {
    sys->fire.error = FCS_FIRE_ERR_CALC;
    return;
  }

  // 무기 최대 사거리 초과
  float abs_max_range = FT_DB[7][FT_Sizes[7]-1].range_m;
  if (final_lookup_range > abs_max_range) {
    sys->fire.error = FCS_FIRE_ERR_RANGE;
    return;
  }

  // 현재 장약 사거리 범위 검사
  float chg_min = current_ft[0].range_m;
  float chg_max = current_ft[current_size-1].range_m;
    
  if ((final_lookup_range < chg_min) || (final_lookup_range > chg_max)) {
    // 전 장약 최소사거리 미만 -> RANGE, 그 외 -> CHARGE
    sys->fire.error = (final_lookup_range < FT_DB[1][0].range_m)
      ? FCS_FIRE_ERR_RANGE : FCS_FIRE_ERR_CHARGE;
    return;
  }

  float base_elev = 0.0f;
  float drift = 0.0f;
  float c_factor = 0.0f;
    
  // 구간 탐색
  int idx = -1;
  for (int i = 0; i < current_size - 1; i++) {
    if ((final_lookup_range >= current_ft[i].range_m) && (final_lookup_range <= current_ft[i+1].range_m)) {
      idx = i;
      break;
    }
  }
    
  if (idx != -1) {
    // 보간
    float r1 = current_ft[idx].range_m;
    float r2 = current_ft[idx+1].range_m;
        
    base_elev = L_Interp(r1, current_ft[idx].elev_mil, r2, current_ft[idx+1].elev_mil, final_lookup_range);
    drift     = L_Interp(r1, current_ft[idx].drift_mil, r2, current_ft[idx+1].drift_mil, final_lookup_range);
    c_factor  = L_Interp(r1, current_ft[idx].c_factor, r2, current_ft[idx+1].c_factor, final_lookup_range);
  } else {
    // fallthrough 방어
    sys->fire.error = FCS_FIRE_ERR_CALC;
    return;
  }

  // 고저 보정 (C_Factor: 10m당 mil)
  float site_corr = (vi_m / 10.0f) * c_factor;
    
  // 최종 데이터 조립
  sys->fire.time_of_flight = tof;
  sys->fire.distance_km = map_dist_m * 0.001f;
  sys->fire.elevation = base_elev + site_corr;
  // 사각 도메인 클램프: 0~1600mil (0~90°, 물리적 포신 한계)
  if (sys->fire.elevation < 0.0f) sys->fire.elevation = 0.0f;
  else { /* 하한 정상 */ }
  if (sys->fire.elevation > 1600.0f) sys->fire.elevation = 1600.0f;
  else { /* 상한 정상 */ }

  // 수정 사격 방위각 반영
  float gt_dist_km = sys->fire.distance_km; // 포-표적 거리 (km)
  if (gt_dist_km < 0.1f) gt_dist_km = 0.1f; // 0 나누기 방지
  else { /* 정상 범위 */ }

  float correction_mil = (float)sys->adj.az_mil / gt_dist_km;

  sys->fire.azimuth = map_az_mil + drift + cori_az_corr + correction_mil + wind_corr_az;
  // 0~6400mil 정규화 (보정 합산으로 범위 이탈 가능)
  sys->fire.azimuth = fmodf(sys->fire.azimuth, MIL_PER_CIRCLE);
  if (sys->fire.azimuth < 0.0f) sys->fire.azimuth += MIL_PER_CIRCLE;
  else { /* 정상 범위 */ }

  sys->fire.map_azimuth = map_az_mil;
  sys->fire.map_distance = map_dist_m;
  sys->fire.height_diff = vi_m;
}
