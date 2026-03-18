/**
 * @file    fcs_core.h
 * @brief   코어 시스템 모듈 — 초기화, I/O 갱신, 직렬 프로토콜.
 */
#ifndef __FCS_CORE_H
#define __FCS_CORE_H

#include "fcs_common.h"

#include "main.h" // HAL 핸들 사용

// 코어 API 함수

/** @brief 시스템 구조체를 기본값으로 초기화. */
void FCS_Init_System(FCS_System_t *sys);

/** @brief 포대(화포) 위치를 UTM 좌표로 설정. */
void FCS_Set_Battery(FCS_System_t *sys, int zone, char band, float e, float n, float alt);

/** @brief 표적 위치를 UTM 좌표로 설정. */
void FCS_Set_Target(FCS_System_t *sys, int zone, char band, float e, float n, float alt);

// 태스크 함수

/** @brief ADC DMA 버퍼를 읽어 입력 상태 갱신 (노브 + 키). */
void FCS_Update_Input(FCS_System_t *sys, const volatile uint16_t *dma_buf);

/** @brief BMP280 읽기 + 범위 클램프 (mtxI2C 구간). */
void FCS_Sensor_Read(void);

/** @brief 센서 캐시를 fcs 구조체에 복사 (mtxFCS 구간). */
void FCS_Sensor_Apply(FCS_System_t *sys);

/** @brief 전이한계 검사 후 초과 시 FCS_FIRE_ERR_TRAVERSE 설정. */
void FCS_Check_Traverse(FCS_System_t *sys);

/** @brief 링 버퍼에서 수신 프레임을 처리 (상태 머신 파서). */
void FCS_Task_Serial(FCS_System_t *sys, UART_HandleTypeDef *huart);

// ISR 인터페이스

/** @brief UART 수신 완료 콜백 — 바이트를 링 버퍼에 삽입. */
void FCS_UART_RxCallback(UART_HandleTypeDef *huart);

/** @brief UART 송신 완료 콜백 — TX busy 플래그 해제, RX IT 재무장. */
void FCS_UART_TxCpltCallback(UART_HandleTypeDef *huart);

/** @brief UART 에러 콜백 — 에러 플래그 정리, 파서 초기화, RX 재무장. */
void FCS_UART_ErrorCallback(UART_HandleTypeDef *huart);

/** @brief UART 수신 인터럽트 체인 시작. */
void FCS_Serial_Start(UART_HandleTypeDef *huart);

/** @brief 링 버퍼 누적 오버플로 횟수 반환. */
uint32_t FCS_Serial_GetOverflowCount(void);

/** @brief BT (USART1) 마지막 바이트 수신 틱 반환 (ms). */
uint32_t FCS_Serial_GetBtLastRxTick(void);

#define BT_CONN_TIMEOUT_MS  5000U  /* 5초 무수신 -> OFFLINE 판정 */

// 버퍼 크기 상수
#define FCS_RESP_BUF_SIZE  96U
#define FCS_TX_BUF_SIZE    128U

// 프로토콜 정의
#define FCS_PROTO_STX  0x02
#define FCS_PROTO_ETX  0x03
#define FCS_PROTO_KEY  0xA5

// 수신 CMD (Python -> STM32)
#define FCS_CMD_TARGET_INPUT 0xA1 // 페이로드: "52,S,E,N,Alt"
#define FCS_CMD_STATUS_REQ   0xC1 // 페이로드: 없음

// 응답 CMD (STM32 -> Python)
#define FCS_CMD_ACK          0xA2 // 정상 응답
#define FCS_CMD_ERR          0xE1 // 오류 응답
#define FCS_CMD_STATUS_ACK   0xC2 // 상태 응답

/**
 * @brief  표적 명령 파싱 및 사격제원 계산.
 * @retval  1 성공, -1 파싱 오류, -2 계산 오류 (RANGE/CHARGE/CALC)
 */
int FCS_Process_Command(FCS_System_t *sys, char *cmd_buffer, char *response_buffer);

#endif
