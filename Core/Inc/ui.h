/**
 * @file    ui.h
 * @brief   OLED 사용자 인터페이스 모듈 (7단계 FSM).
 */
#ifndef INC_UI_H_
#define INC_UI_H_

#include "main.h"
#include "bmp280.h"
#include "input.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "fcs_common.h" // 공용 타입

// 함수 원형

/** @brief UI 기본값 초기화, 부팅 화면 표시, BATTERY_POS 진입. */
void UI_Init(FCS_System_t *sys);

/** @brief 노브/키 입력 처리 및 상태 전환. */
void UI_Update(FCS_System_t *sys);

/** @brief 현재 UI 상태 SSD1306 OLED 렌더링. */
void UI_Draw(FCS_System_t *sys);

#endif /* INC_UI_H_ */
