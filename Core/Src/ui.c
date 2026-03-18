/**
 * @file    ui.c
 * @brief   OLED 사용자 인터페이스 — 7단계 FSM (BOOT, BATTERY_POS, LAYING_DATA,
 *          STANDBY, TARGET_LOCK, FIRE_DATA, ADJUST_FIRE), 노브/키 입력 처리.
 */
#include "ui.h"
#include "fcs_math.h"  // 탄도 계산 모듈
#include "fcs_core.h"  // BT 연결 상태 조회
#include "flash_ops.h" // Flash 저장/로드
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // abs()
#include <math.h>

// 헬퍼 매크로
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

// UI 상수
#define ADC_MAX           4096U
#define KEY_DEBOUNCE_MS   200U
#define BOOT_DELAY_MS     1000U
#define CHARGE_MAX        7
#define ROUNDS_MIN        1
#define ROUNDS_MAX        10
#define MASK_MAX_MIL      800
#define PT_MIN_C          (-40)
#define PT_MAX_C          60
#define ADJ_RANGE_STEP    100
#define ADJ_AZ_STEP       10
#define LAYING_AZ_MAX     6400

/**
 * @brief  UI 기본값 초기화, 부팅 화면 표시, BATTERY_POS로 전환.
 * @param  sys  user_pos, fire 기본값 및 커서 설정
 */
void UI_Init(FCS_System_t *sys) {
  sys->state = UI_BOOT;

  // 기본값 설정 (한국 기본: Zone 52S)
  sys->user_pos.zone = 52;
  sys->user_pos.band = 'S';
  sys->user_pos.easting = 333712.0f;
  sys->user_pos.northing = 4132894.0f;
  sys->user_pos.altitude = 100.0f;

  // 사격 제원 기본값
  sys->fire.charge = 1;
  sys->fire.rounds = 1;

  // 수정 사격 초기화
  sys->adj.range_m = 0;
  sys->adj.az_mil = 0;

  // 부팅 화면
  ssd1306_Fill(0);
  ssd1306_SetCursor(20, 15);
  ssd1306_WriteString("AFCS-105", Font_11x18, White);
  ssd1306_SetCursor(29, 39);
  ssd1306_WriteString("BOOTING...", Font_7x10, White);
  ssd1306_UpdateScreen();
  HAL_Delay(BOOT_DELAY_MS);

  // 첫 상태 진입
  sys->state = UI_BATTERY_POS;
  sys->cursor_pos = 0;
}

/**
 * @brief  노브 ADC 값 및 키 입력 처리; FSM 상태 전환.
 * @param  sys  입력 읽기, state/fire/env/adj 변경
 */
void UI_Update(FCS_System_t *sys) {

  static KeyState_t last_key = KEY_NONE;
  KeyState_t key = sys->input.key_state;

  // 노브 데이터 처리 (포 제원 설정 전용)

  if (sys->state == UI_LAYING_DATA) {
    // 노브0: 방열방위각 (0~6400mil, 100mil 단위) EMA 평활
    static uint32_t smooth_knob0 = 0;
    if (smooth_knob0 == 0) smooth_knob0 = sys->input.knob_values[0];
    else { /* 초기화 완료 상태 */ }
    smooth_knob0 = (smooth_knob0 * 7U + sys->input.knob_values[0]) / 8U;
    uint16_t lay_step = (uint16_t)((smooth_knob0 * 65U) / ADC_MAX);
    if (lay_step > 64U) lay_step = 64U;
    else { /* 정상 범위 */ }
    sys->laying_azimuth = lay_step * 100U;  /* 0,100,200,...,6400 */

    // 노브1: 차폐각 (0~800mil) EMA 평활
    static uint32_t smooth_knob1 = 0;
    if (smooth_knob1 == 0) smooth_knob1 = sys->input.knob_values[1];
    else { /* 초기화 완료 상태 */ }
    smooth_knob1 = (smooth_knob1 * 3U + sys->input.knob_values[1]) / 4U;
    uint16_t mask_val = (uint16_t)(((smooth_knob1 * 81U) / ADC_MAX) * 10U);
    if (mask_val > MASK_MAX_MIL) mask_val = MASK_MAX_MIL;
    else { /* 정상 범위 */ }
    sys->mask_angle = mask_val;

    // 노브2: 장약 온도 (-40~60°C) EMA 평활
    static uint32_t smooth_knob2 = 0;
    if (smooth_knob2 == 0) smooth_knob2 = sys->input.knob_values[2];
    else { /* 초기화 완료 상태 */ }
    smooth_knob2 = (smooth_knob2 * 3U + sys->input.knob_values[2]) / 4U;
    int pt_raw = (int)((smooth_knob2 * 101U) / ADC_MAX) + PT_MIN_C;
    sys->env.prop_temp = (float)CLAMP(pt_raw, PT_MIN_C, PT_MAX_C);
  }

  // 실시간 재계산
  if (sys->state == UI_FIRE_DATA) {
    // 노브2: 발사 탄수 (1~10) - 사격 제원 전용
    sys->fire.rounds = (int)((sys->input.knob_values[2] * (uint32_t)ROUNDS_MAX) / ADC_MAX) + 1;
    if (sys->fire.rounds > ROUNDS_MAX) sys->fire.rounds = ROUNDS_MAX;
    else { /* 정상 범위 */ }

    FCS_Calculate_FireData(sys);
    FCS_Check_Traverse(sys); // 전이한계 재검사
  }

  // 단계별 버튼 처리
  if ((key != KEY_NONE) && (key != last_key)) {
    static uint32_t last_act_time = 0;
    if (HAL_GetTick() - last_act_time > KEY_DEBOUNCE_MS) {
      last_act_time = HAL_GetTick(); 

      // 자릿수 변경 헬퍼
      static const float pow10[] = { 1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f, 100000.0f, 1000000.0f, 10000000.0f };
      float mod_val = 1.0f;

      // 좌표 포인터
      UTM_Coord_t *curr_coord = (sys->state == UI_BATTERY_POS) ? &sys->user_pos : &sys->tgt_pos;
      
      // 커서 범위: 0-1(존/밴드), 2-7(E), 8-14(N), 15-18(고도)
      int max_cursor = 18;

      // 커서 위치에 따른 자릿수 계산
      if ((sys->cursor_pos >= 2) && (sys->cursor_pos <= 7)) {
        mod_val = pow10[7 - sys->cursor_pos];
      } else if ((sys->cursor_pos >= 8) && (sys->cursor_pos <= 14)) {
        mod_val = pow10[14 - sys->cursor_pos];
      } else if ((sys->cursor_pos >= 15) && (sys->cursor_pos <= 18)) {
        mod_val = pow10[18 - sys->cursor_pos];
      } else {
        /* 존/밴드 위치(0-1): 자릿수 변경 대상 아님 */
      }
            
    switch (sys->state) {
      case UI_BATTERY_POS:
      case UI_TARGET_LOCK:
        // 좌우: 커서 이동
        if (key == KEY_RIGHT) {
          sys->cursor_pos++;
          if (sys->cursor_pos > max_cursor) sys->cursor_pos = 0;
          else { /* 정상 범위 */ }
        } else if (key == KEY_LEFT) {
          if (sys->cursor_pos > 0) sys->cursor_pos--;
          else sys->cursor_pos = max_cursor;
        } else {
          /* 좌우 외 키: 커서 이동 없음 */
        }
            
        // 상하: 값 변경
        if ((key == KEY_UP) || (key == KEY_DOWN)) {
          int dir = (key == KEY_UP) ? 1 : -1;
               
          if (sys->cursor_pos == 0) { // 존
            if (curr_coord->zone == 51) curr_coord->zone = 52;
            else curr_coord->zone = 51;
          }
          else if (sys->cursor_pos == 1) { // 밴드
            if (curr_coord->band == 'S') curr_coord->band = 'T';
            else curr_coord->band = 'S';
          }
          else if (sys->cursor_pos <= 14) { // Easting / Northing 자릿수
            float *target_val_ptr = (sys->cursor_pos <= 7) ? &curr_coord->easting : &curr_coord->northing;
            uint32_t val_int = (uint32_t)(*target_val_ptr);
            uint32_t multiplier = (uint32_t)mod_val;
            int current_digit = (int)((val_int / multiplier) % 10U);
            int new_digit = (current_digit + dir + 10) % 10;
            val_int = val_int - ((uint32_t)current_digit * multiplier) + ((uint32_t)new_digit * multiplier);
            *target_val_ptr = (float)val_int;
          } 
          else { // 고도 자릿수 (공통 로직, cursor_pos >= 15)
            int alt_int = (int)curr_coord->altitude;
            int multiplier = (int)mod_val;
            int current_digit = (alt_int / multiplier) % 10;
            int new_digit = (current_digit + dir + 10) % 10;
            alt_int = alt_int - (current_digit * multiplier) + (new_digit * multiplier);
            curr_coord->altitude = (float)alt_int;
          }
        }
            
        // 상태 전환
        if (sys->state == UI_BATTERY_POS) {
          if (key == KEY_ENTER) {
            Flash_Save_BatteryPos(sys); // 플래시 자동 저장
            sys->state = UI_LAYING_DATA;
          } else {
            /* ENTER 외 키: 상태 유지 */
          }
        } else { // UI_TARGET_LOCK
          if (key == KEY_LEFT) sys->state = UI_STANDBY;
          else if (key == KEY_ENTER) {
            sys->fire.auto_select = 1;
            FCS_Calculate_FireData(sys);
            FCS_Check_Traverse(sys);
            sys->state = UI_FIRE_DATA;
          } else {
            /* 해당 키 아님 */
          }
        }
        break;

      case UI_LAYING_DATA:
        if (key == KEY_LEFT) {
          sys->state = UI_BATTERY_POS;
          sys->cursor_pos = 0;
        }
        else if (key == KEY_ENTER) {
          sys->state = UI_STANDBY;
        } else {
          /* 해당 키 아님 */
        }
        break;

      case UI_STANDBY:
        if (key == KEY_LEFT) {
          sys->state = UI_LAYING_DATA;
        }
        else if (key == KEY_ENTER) {
          // 표적 좌표 초기화
          sys->tgt_pos.zone = 52;
          sys->tgt_pos.band = 'S';
          sys->tgt_pos.easting = 0.0f;
          sys->tgt_pos.northing = 0.0f;
          sys->tgt_pos.altitude = 0.0f;

          sys->state = UI_TARGET_LOCK;
          sys->cursor_pos = 0;
        } else {
          /* 해당 키 아님 */
        }
        break;

      case UI_FIRE_DATA:
        if (key == KEY_UP) {
          if (sys->fire.charge < CHARGE_MAX) sys->fire.charge++;
          else { /* 최대 장약 */ }
        }
        else if (key == KEY_DOWN) {
          if (sys->fire.charge > 1) sys->fire.charge--;
          else { /* 최소 장약 */ }
        }
        else if (key == KEY_LEFT) {
          sys->adj.range_m = 0;
          sys->adj.az_mil = 0;
          sys->state = UI_STANDBY;
        }
        else if (key == KEY_ENTER) sys->state = UI_ADJUST_FIRE;
        else {
          /* 해당 키 아님 */
        }
        break;
            
      case UI_ADJUST_FIRE:
        // 상하: 사거리 수정 (+/- 100m)
        if (key == KEY_UP) sys->adj.range_m += ADJ_RANGE_STEP;
        else if (key == KEY_DOWN) sys->adj.range_m -= ADJ_RANGE_STEP;
        else { /* 상하 외 키: 사거리 변경 없음 */ }
        sys->adj.range_m = (int16_t)CLAMP(sys->adj.range_m, -9900, 9900);

        // 좌우: 편각 수정 (+/- 10mil)
        if (key == KEY_RIGHT) sys->adj.az_mil += ADJ_AZ_STEP;
        else if (key == KEY_LEFT) sys->adj.az_mil -= ADJ_AZ_STEP;
        else { /* 좌우 외 키: 편각 변경 없음 */ }
        sys->adj.az_mil = (int16_t)CLAMP(sys->adj.az_mil, -990, 990);

        // 확인: 재계산 후 사격 제원 복귀
        if (key == KEY_ENTER) {
          FCS_Calculate_FireData(sys);
          FCS_Check_Traverse(sys);
          sys->state = UI_FIRE_DATA;
        } else {
          /* ENTER 외: 재계산 없음 */
        }
        break;

      default: break;
    }
  }
}
  last_key = key;
}

/**
 * @brief  현재 UI 상태 SSD1306 OLED (128x64) 렌더링.
 * @param  sys  state, fire, env, user_pos, tgt_pos, adj 읽기
 */
void UI_Draw(FCS_System_t *sys) {
    char buf[32];
    ssd1306_Fill(0); 

    int cx = 0, cw = 0, cy = 0;
    
    switch (sys->state) {
        case UI_BOOT:
            ssd1306_SetCursor(20, 15);
            ssd1306_WriteString("AFCS-105", Font_11x18, White);
            ssd1306_SetCursor(29, 39);
            ssd1306_WriteString("BOOTING...", Font_7x10, White);
            break;

        case UI_BATTERY_POS:
            // 제목
            ssd1306_SetCursor(25, 0);
            ssd1306_WriteString("[BATTERY POS]", Font_6x8, White);

            // 데이터
            snprintf(buf, sizeof(buf), "%d%c E:%06lu", sys->user_pos.zone, sys->user_pos.band, (uint32_t)sys->user_pos.easting);
            ssd1306_SetCursor(0, 16);
            ssd1306_WriteString(buf, Font_7x10, White);

            snprintf(buf, sizeof(buf), "    N:0%07lu", (uint32_t)sys->user_pos.northing);
            ssd1306_SetCursor(0, 28);
            ssd1306_WriteString(buf, Font_7x10, White);

            // 데이터 (3행: 고도) Y=40 — 콜론 정렬
            snprintf(buf, sizeof(buf), "    A:%04d m", (int)sys->user_pos.altitude);
            ssd1306_SetCursor(0, 40);
            ssd1306_WriteString(buf, Font_7x10, White);

            // 액션 버튼
            ssd1306_SetCursor(80, 54);
            ssd1306_WriteString("[SAVE]", Font_6x8, White);

            // 커서 하이라이트
            if (sys->cursor_pos == 0) { cx=0; cw=14; cy=27; }
            else if (sys->cursor_pos == 1) { cx=14; cw=7; cy=27; }
            else if (sys->cursor_pos <= 7) { cx=42+(sys->cursor_pos-2)*7; cw=7; cy=27; }
            else if (sys->cursor_pos <= 14) { cx=42+7+(sys->cursor_pos-8)*7; cw=7; cy=39; } // '0' 접두어 +7px
            else { cx=42+(sys->cursor_pos-15)*7; cw=7; cy=51; } // 고도 커서 Y=40+11
            
            ssd1306_Line(cx, cy, cx+cw-1, cy, White);
            break;

        case UI_LAYING_DATA:
            ssd1306_SetCursor(25, 0);
            ssd1306_WriteString("[LAYING DATA]", Font_6x8, White);

            // 방열방위각
            snprintf(buf, sizeof(buf), "AOL: %04u mil", sys->laying_azimuth);
            ssd1306_SetCursor(14, 14);
            ssd1306_WriteString(buf, Font_7x10, White);

            // 차폐각
            snprintf(buf, sizeof(buf), "MSK: %03u mil", sys->mask_angle);
            ssd1306_SetCursor(14, 28);
            ssd1306_WriteString(buf, Font_7x10, White);

            // 장약 온도
            snprintf(buf, sizeof(buf), "PT : %3d C", (int)sys->env.prop_temp);
            ssd1306_SetCursor(14, 42);
            ssd1306_WriteString(buf, Font_7x10, White);

            // 확인 버튼
            ssd1306_SetCursor(80, 54);
            ssd1306_WriteString("[SAVE]", Font_6x8, White);
            break;

        case UI_STANDBY:
            ssd1306_SetCursor(37, 0);
            ssd1306_WriteString("[STANDBY]", Font_6x8, White);
            
            // 환경 데이터 (센서 오류시 * 표시)
            // abs()로 C99 음수 나머지 부호 보정, 0~-1구간 부호 보존
            {
              int t_i = (int)sys->env.air_temp;
              int t_d = abs((int)(sys->env.air_temp * 10.0f) % 10);
              const char *t_sign =
                ((sys->env.air_temp < 0.0f) && (t_i == 0)) ? "-" : "";
              if (sys->diag.sensor_ok) {
                snprintf(buf, sizeof(buf), "T:%s%d.%d  P:%d",
                    t_sign, t_i, t_d, (int)sys->env.air_pressure);
              } else {
                snprintf(buf, sizeof(buf), "T:%s%d.%d* P:%d*",
                    t_sign, t_i, t_d, (int)sys->env.air_pressure);
              }
            }
            ssd1306_SetCursor(10, 22);
            ssd1306_WriteString(buf, Font_7x10, White);

            // BT 연결 상태 (Python heartbeat 기반)
            {
              uint32_t elapsed = HAL_GetTick() - FCS_Serial_GetBtLastRxTick();
              if ((FCS_Serial_GetBtLastRxTick() > 0U) && (elapsed < BT_CONN_TIMEOUT_MS)) {
                ssd1306_SetCursor(25, 42);
                ssd1306_WriteString("BT: ONLINE", Font_7x10, White);
              } else {
                ssd1306_SetCursor(22, 42);
                ssd1306_WriteString("BT: OFFLINE", Font_7x10, White);
              }
            }
            break;

        case UI_TARGET_LOCK:
            ssd1306_SetCursor(31, 0);
            ssd1306_WriteString("[TARGET LOCK]", Font_6x8, White);
            
            // 표적 데이터 (포 위치와 동일 레이아웃)
            snprintf(buf, sizeof(buf), "%d%c E:%06lu", sys->tgt_pos.zone, sys->tgt_pos.band, (uint32_t)sys->tgt_pos.easting);
            ssd1306_SetCursor(0, 16);
            ssd1306_WriteString(buf, Font_7x10, White);

            snprintf(buf, sizeof(buf), "    N:0%07lu", (uint32_t)sys->tgt_pos.northing);
            ssd1306_SetCursor(0, 28);
            ssd1306_WriteString(buf, Font_7x10, White);

            // 고도 입력
            snprintf(buf, sizeof(buf), "    A:%04d m", (int)sys->tgt_pos.altitude);
            ssd1306_SetCursor(0, 40); 
            ssd1306_WriteString(buf, Font_7x10, White);

            // 액션 버튼
            ssd1306_SetCursor(80, 54);
            ssd1306_WriteString("[FIRE]", Font_6x8, White);

            // 커서 하이라이트
            if (sys->cursor_pos == 0) { cx=0; cw=14; cy=27; }
            else if (sys->cursor_pos == 1) { cx=14; cw=7; cy=27; }
            else if (sys->cursor_pos <= 7) { cx=42+(sys->cursor_pos-2)*7; cw=7; cy=27; }
            else if (sys->cursor_pos <= 14) { cx=42+7+(sys->cursor_pos-8)*7; cw=7; cy=39; } // '0' 접두어 +7px
            else { cx=42+(sys->cursor_pos-15)*7; cw=7; cy=51; } // 고도 커서
            
            ssd1306_Line(cx, cy, cx+cw-1, cy, White);
            break;

        case UI_FIRE_DATA:
            ssd1306_SetCursor(28, 0);
            ssd1306_WriteString("[FIRE DATA]", Font_6x8, White);
            
            snprintf(buf, sizeof(buf), "CH:%d  AM:HE", sys->fire.charge);
            ssd1306_SetCursor(20, 16);
            ssd1306_WriteString(buf, Font_7x10, White);

            snprintf(buf, sizeof(buf), "FU:Q6 RD:%d", sys->fire.rounds);
            ssd1306_SetCursor(20, 28);
            ssd1306_WriteString(buf, Font_7x10, White);
            
            ssd1306_Line(10, 42, 118, 42, White);
            
            if (sys->fire.error != FCS_FIRE_OK) {
                // 오류 처리
                ssd1306_SetCursor(18, 48);
                if (sys->fire.error == FCS_FIRE_ERR_RANGE) {
                    ssd1306_WriteString("! RANGE ERR !", Font_7x10, White);
                } else if (sys->fire.error == FCS_FIRE_ERR_TRAVERSE) {
                    ssd1306_WriteString("! TRAV  ERR !", Font_7x10, White);
                } else if (sys->fire.error == FCS_FIRE_ERR_ZONE) {
                    ssd1306_WriteString("! ZONE  ERR !", Font_7x10, White);
                } else {
                    ssd1306_WriteString("! CHG   ERR !", Font_7x10, White);
                }
            }
            else if (sys->fire.elevation < (float)sys->mask_angle) {
                ssd1306_SetCursor(18, 48);
                ssd1306_WriteString("! MASK  ERR !", Font_7x10, White); 
            } else {
                snprintf(buf, sizeof(buf), "AZ:%04d QE:%03d", (int)sys->fire.azimuth, (int)sys->fire.elevation);
                ssd1306_SetCursor(15, 48);
                ssd1306_WriteString(buf, Font_7x10, White);
            }
            break;

        case UI_ADJUST_FIRE:
            ssd1306_SetCursor(25, 0);
            ssd1306_WriteString("[ADJUST FIRE]", Font_6x8, White);
            
            // 편각(방위각) 표시
            char dir_c = ' ';
            int abs_mil = sys->adj.az_mil;
            if (abs_mil < 0) { dir_c = 'L'; abs_mil = -abs_mil; }
            else if (abs_mil > 0) { dir_c = 'R'; }
            else { /* 0: 기본값(' ') 유지 */ }
            
            snprintf(buf, sizeof(buf), "DEV : %c  %3d", dir_c, abs_mil);
            ssd1306_SetCursor(10, 20);
            ssd1306_WriteString(buf, Font_7x10, White);

            // 사거리 표시
            char sign_c = '+';
            int abs_rng = sys->adj.range_m;
            if (abs_rng < 0) { sign_c = '-'; abs_rng = -abs_rng; }
            else if (abs_rng == 0) { sign_c = ' '; }
            else { /* 양수: 기본값('+') 유지 */ }
            if (sys->adj.range_m == 0) sign_c = ' ';

            snprintf(buf, sizeof(buf), "RNG : %c %4d", sign_c, abs_rng);
            ssd1306_SetCursor(10, 35);
            ssd1306_WriteString(buf, Font_7x10, White);

            // 동작 버튼
            ssd1306_SetCursor(70, 54);
            ssd1306_WriteString("[UPDATE]", Font_6x8, White);
            break;
    }
    
    ssd1306_UpdateScreen(); 
}
