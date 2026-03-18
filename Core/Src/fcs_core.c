/**
 * @file    fcs_core.c
 * @brief   코어 시스템 모듈 — 초기화, ADC/센서 갱신,
 *          UART 링 버퍼, 암호화 직렬 프로토콜 파서, 명령 분기.
 */
#include "fcs_core.h"
#include "fcs_math.h"
#include "bmp280.h"
#include "input.h"
#include "ui.h"   // UI 상태 열거형
#include "servo.h" // Servo_AdjustTrim, SERVO_AZ_TRAVERSE_MIL 참조
#include "crc.h"  // 하드웨어 CRC-32
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// 직렬 링 버퍼
#define RING_SIZE 128
static uint8_t u_buf[RING_SIZE];
static volatile uint8_t u_head = 0;
static volatile uint8_t u_tail = 0;
static uint8_t rx_byte_u1; // USART1 (BT) ISR 수신 바이트
static uint8_t rx_byte_u2; // USART2 (디버그) ISR 수신 바이트
static volatile uint32_t u_overflow_cnt = 0;
static volatile uint32_t bt_last_rx_tick = 0; /* BT 수신 타임스탬프 */

// 비동기 TX
static uint8_t tx_pkt[FCS_TX_BUF_SIZE]; // IT용 정적 TX 버퍼
static volatile uint8_t tx_busy = 0;     // 전송 진행 플래그

// ST-LINK Live Expression 계측용
volatile uint32_t prof_calc_cycles = 0;

/**
 * @brief  시스템 구조체 초기화 (memset 영점 + 기본값).
 */
void FCS_Init_System(FCS_System_t *sys) {
  memset(sys, 0, sizeof(FCS_System_t));
  sys->state = UI_BOOT;
  sys->env.prop_temp = 21.0f;
  sys->diag.sensor_ok = 1; // 첫 읽기 오류 전까지 정상 가정
  sys->servo.az_angle = (uint8_t)(90 + Servo_GetTrim()); // 방위각 중립+트림
  sys->servo.el_angle = 0U;  // 사각 수평 (팬틸트: 0°=수평)
  sys->servo.enabled = 0U;   // main에서 Init 후 활성화
}

/**
 * @brief  포대(화포) 위치를 UTM 좌표로 설정.
 */
void FCS_Set_Battery(FCS_System_t *sys, int zone, char band, float e, float n, float alt) {
  sys->user_pos.zone = zone;
  sys->user_pos.band = band;
  sys->user_pos.easting = e;
  sys->user_pos.northing = n;
  sys->user_pos.altitude = alt;
}

/**
 * @brief  표적 위치를 UTM 좌표로 설정.
 */
void FCS_Set_Target(FCS_System_t *sys, int zone, char band, float e, float n, float alt) {
  sys->tgt_pos.zone = zone;
  sys->tgt_pos.band = band;
  sys->tgt_pos.easting = e;
  sys->tgt_pos.northing = n;
  sys->tgt_pos.altitude = alt;
}

/**
 * @brief  ADC DMA 버퍼를 시스템 입력 상태에 복사 (논블로킹).
 * @param  sys      FCS 전역 상태
 * @param  dma_buf  4채널 ADC DMA 순환 버퍼
 */
void FCS_Update_Input(FCS_System_t *sys, const volatile uint16_t *dma_buf) {
  for (int i = 0; i < 4; i++) {
    sys->input.adc_raw[i] = (uint32_t)dma_buf[i];
  }

  sys->input.knob_values[0] = sys->input.adc_raw[0];
  sys->input.knob_values[1] = sys->input.adc_raw[1];
  sys->input.knob_values[2] = sys->input.adc_raw[2];

  sys->input.key_state = Input_Scan(sys->input.adc_raw[3]);
}

// 센서 캐시 (Task_Sensor -> fcs 반영 분리)
static BMP280_Data_t sensor_cache;
static uint8_t sensor_cache_rc;

/**
 * @brief  BMP280 읽기 + 범위 클램프 (mtxI2C 구간).
 */
void FCS_Sensor_Read(void) {
  sensor_cache_rc = BMP280_Read_All(&sensor_cache);

  // 센서 오류 시 비정상값 차단 (BMP280 스펙 범위)
  if (sensor_cache.temperature < -40.0f)  sensor_cache.temperature = -40.0f;
  else { /* 하한 정상 */ }
  if (sensor_cache.temperature >  85.0f)  sensor_cache.temperature =  85.0f;
  else { /* 상한 정상 */ }
  if (sensor_cache.pressure < 300.0f)     sensor_cache.pressure = 300.0f;
  else { /* 하한 정상 */ }
  if (sensor_cache.pressure > 1100.0f)    sensor_cache.pressure = 1100.0f;
  else { /* 상한 정상 */ }
}

/**
 * @brief  센서 캐시를 fcs 구조체에 복사 (mtxFCS 구간).
 */
void FCS_Sensor_Apply(FCS_System_t *sys) {
  sys->env.air_temp = sensor_cache.temperature;
  sys->env.air_pressure = sensor_cache.pressure;

  if (sensor_cache_rc == BMP280_OK) {
    sys->diag.sensor_ok = 1;
  } else {
    if (sys->diag.sensor_ok) {
      // 상태 전환시만 출력 (정상 -> 오류)
      DBG_PRINT("[SNS] rc=%u rawT=%ld rawP=%ld\r\n",
                sensor_cache_rc, sensor_cache.raw_temperature,
                sensor_cache.raw_pressure);
    } else {
      /* 이미 오류 상태: 반복 출력 억제 */
    }
    sys->diag.sensor_ok = 0;
    if (sensor_cache_rc == BMP280_I2C_ERROR) {
      sys->diag.i2c_errors++;
    } else {
      /* I2C 외 오류: 별도 카운트 없음 */
    }
  }
}

/**
 * @brief  UART 수신 ISR 콜백 — 바이트를 링 버퍼에 삽입.
 * @note   main.c의 HAL_UART_RxCpltCallback에서 호출 (ISR 컨텍스트).
 */
void FCS_UART_RxCallback(UART_HandleTypeDef *huart) {
    // UART별 수신 바이트 선택
    uint8_t *rx_ptr = (huart->Instance == USART1) ? &rx_byte_u1 : &rx_byte_u2;
    if (huart->Instance == USART1) {
      bt_last_rx_tick = HAL_GetTick(); /* BT 수신 시각 갱신 */
    } else {
      /* USART2: 타임스탬프 불필요 */
    }
    uint8_t next_head = (u_head + 1) % RING_SIZE;
    if (next_head != u_tail) {
        u_buf[u_head] = *rx_ptr;
        u_head = next_head;
    } else {
        u_overflow_cnt++;
    }
    HAL_UART_Receive_IT(huart, rx_ptr, 1);
}

/** @brief UART 수신 인터럽트 체인 시작 (초기화 시 1회 호출). */
void FCS_Serial_Start(UART_HandleTypeDef *huart) {
    uint8_t *rx_ptr = (huart->Instance == USART1) ? &rx_byte_u1 : &rx_byte_u2;
    HAL_UART_Receive_IT(huart, rx_ptr, 1);
}

/** @brief 링 버퍼 누적 오버플로 횟수 반환 (진단용). */
uint32_t FCS_Serial_GetOverflowCount(void) {
    return u_overflow_cnt;
}

/** @brief BT (USART1) 마지막 바이트 수신 틱 반환 (연결 감지용). */
uint32_t FCS_Serial_GetBtLastRxTick(void) {
    return bt_last_rx_tick;
}

// 파서 상태 머신
typedef enum {
  P_IDLE,
  P_CMD,
  P_SALT,
  P_LEN,
  P_PAYLOAD,
  P_CRC,
  P_ETX
} ParserState_t;

static volatile ParserState_t p_state = P_IDLE;
static uint8_t p_cmd = 0;
static uint8_t p_salt = 0;
static uint8_t p_len = 0;
static uint8_t p_idx = 0;
static uint8_t p_payload[128]; // 최대 페이로드
static uint8_t p_crc_buf[4];   // CRC-32 수신 버퍼 (4바이트)
static uint8_t p_crc_idx = 0;  // CRC 바이트 수집 인덱스
static uint32_t p_last_rx_tick = 0;
#define PARSER_TIMEOUT_MS   500U
#define PROTO_MAX_PAYLOAD   120
/* UART_TX_TIMEOUT_MS 삭제 — HAL_UART_Transmit_IT 전환으로 불필요 */

/** @brief 응답 패킷 암호화 + IT 전송. */
static void Send_Response_Packet(UART_HandleTypeDef *huart,
                                 uint8_t resp_cmd, uint8_t salt,
                                 const char *payload)
{
  /* 이전 전송 진행중이면 드롭 */
  if (tx_busy) {
    DBG_PRINT("[TX] BUSY — dropped CMD=0x%02X\r\n", resp_cmd);
    return;
  }

  uint8_t plen = (uint8_t)strlen(payload);
  // tx_pkt[128] 오버플로 방지 (헤더4+CRC4+ETX1=9바이트 오버헤드)
  if (plen > FCS_TX_BUF_SIZE - 9U) {
    plen = FCS_TX_BUF_SIZE - 9U;
  } else {
    /* 버퍼 여유 충분 */
  }
  uint8_t session_key = FCS_PROTO_KEY ^ salt;

  tx_pkt[0] = FCS_PROTO_STX;
  tx_pkt[1] = resp_cmd;
  tx_pkt[2] = salt;
  tx_pkt[3] = plen;

  for (uint8_t i = 0; i < plen; i++) {
    tx_pkt[4U + i] = (uint8_t)payload[i] ^ (uint8_t)(session_key + i);
  }

  uint32_t crc = HW_CRC32(&tx_pkt[1], (uint32_t)(3U + plen));

  uint8_t pos = 4U + plen;
  tx_pkt[pos++] = (uint8_t)(crc >> 24);
  tx_pkt[pos++] = (uint8_t)(crc >> 16);
  tx_pkt[pos++] = (uint8_t)(crc >> 8);
  tx_pkt[pos++] = (uint8_t)(crc);

  tx_pkt[pos++] = FCS_PROTO_ETX;

  tx_busy = 1;
  HAL_StatusTypeDef tx_rc = HAL_UART_Transmit_IT(huart, tx_pkt, (uint16_t)pos);
  if (tx_rc != HAL_OK) {
    tx_busy = 0; // 실패 시 즉시 해제
  } else {
    /* 전송 시작 성공: TxCplt ISR에서 해제 */
  }

  DBG_PRINT("[TX] CMD=0x%02X LEN=%u CRC=0x%08lX %s => %s\r\n",
            resp_cmd, plen, (unsigned long)crc,
            (tx_rc == HAL_OK) ? "OK" : "FAIL", payload);
}

/**
 * @brief  UART 송신 완료 콜백 — TX busy 플래그 해제, RX IT 재무장.
 * @note   main.c의 HAL_UART_TxCpltCallback에서 호출 (ISR 컨텍스트).
 */
void FCS_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  /* BT 응답 전송 완료 시만 해제 (USART2 IT 전환 시 오동작 방지) */
  if (huart->Instance == USART1) {
    tx_busy = 0;
    /* TX 완료 후 RX IT 재무장 (TX 중 HAL Lock으로 실패했을 수 있음) */
    HAL_UART_Receive_IT(huart, &rx_byte_u1, 1);
  } else {
    /* USART2: tx_busy 미사용 */
  }
}

/**
 * @brief  UART 에러 콜백 — 플래그 정리, 파서 초기화, RX 재무장.
 * @note   ORE 시 RX 재무장, FE/NE/PE 시 파서 리셋.
 */
void FCS_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  /* 에러 플래그 정리 — 모든 UART 공통 (미정리 시 ORE 반복 발생) */
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);

  /* USART1(BT) 에러일 때만 파서 리셋 */
  if (huart->Instance == USART1) {
    p_state = P_IDLE;
    if (tx_busy) { tx_busy = 0; }
    else { /* 전송 미진행 */ }
  } else {
    /* USART2: 파서 리셋 불필요 */
  }

  /* RX IT 재무장 — 모든 UART 공통 */
  FCS_Serial_Start(huart);
}

/**
 * @brief  직렬 프로토콜 파서 및 명령 실행.
 * @param  sys    FCS 전역 상태 (명령 분기 시 갱신)
 * @param  huart  응답 전송용 UART 핸들
 */
void FCS_Task_Serial(FCS_System_t *sys, UART_HandleTypeDef *huart) {
  // 미완성 프레임 500ms 이상 정체시 파서 초기화
  if ((p_state != P_IDLE) && ((HAL_GetTick() - p_last_rx_tick) > PARSER_TIMEOUT_MS)) {
    p_state = P_IDLE;
  } else {
    /* 타임아웃 미발생 또는 이미 대기 상태 */
  }

  while (u_head != u_tail) {
    // 바이트 디큐
    uint8_t rx = u_buf[u_tail];
    u_tail = (u_tail + 1) % RING_SIZE;
    p_last_rx_tick = HAL_GetTick();

    // 트림 조정 (PuTTY에서 l/r 키)
    // 포조준 전 상태(BATTERY_POS/LAYING_DATA)에서만 허용 — 사격 중 오조작 방지
#ifdef DEBUG
    if ((p_state == P_IDLE)
        && ((sys->state == UI_BATTERY_POS) || (sys->state == UI_LAYING_DATA))) {
      if ((rx == 'l') || (rx == 'L')) {
        Servo_AdjustTrim(-1);
        sys->servo.az_angle = (uint8_t)(90 + Servo_GetTrim());
        DBG_PRINT("[TRIM] AZ=%d\r\n", Servo_GetTrim());
        continue;
      }
      if ((rx == 'r') || (rx == 'R')) {
        Servo_AdjustTrim(+1);
        sys->servo.az_angle = (uint8_t)(90 + Servo_GetTrim());
        DBG_PRINT("[TRIM] AZ=%d\r\n", Servo_GetTrim());
        continue;
      }
    }
#endif

    // 프로토콜 상태 머신
    switch (p_state) {
      case P_IDLE:
        if (rx == FCS_PROTO_STX) {
          p_state = P_CMD;
        } else {
          /* 비 STX 바이트: 무시 */
        }
        break;
                
      case P_CMD:
        p_cmd = rx;
        p_state = P_SALT;
        break;
                
      case P_SALT:
        p_salt = rx;
        p_state = P_LEN;
        break;
                
      case P_LEN:
        p_len = rx;
        p_idx = 0;
        p_crc_idx = 0; // CRC 수집 인덱스 초기화
        if (p_len > PROTO_MAX_PAYLOAD) p_state = P_IDLE; // 안전 한계
        else if (p_len == 0) p_state = P_CRC; // 빈 페이로드
        else p_state = P_PAYLOAD;
        break;
                
      case P_PAYLOAD:
        p_payload[p_idx++] = rx;
        if (p_idx >= p_len) { p_state = P_CRC; }
        else { /* 수집 진행 중 */ }
        break;
                
      case P_CRC:
        p_crc_buf[p_crc_idx++] = rx;
        if (p_crc_idx >= 4U) { p_state = P_ETX; } // 4바이트 수집 완료
        else { /* CRC 수집 진행 중 */ }
        break;

      case P_ETX:
        if (rx == FCS_PROTO_ETX) {
          // CRC-32 검증
          uint8_t crc_input[3 + PROTO_MAX_PAYLOAD];
          crc_input[0] = p_cmd;
          crc_input[1] = p_salt;
          crc_input[2] = p_len;
          memcpy(&crc_input[3], p_payload, p_len);
          uint32_t calc_crc = HW_CRC32(crc_input, 3U + (uint32_t)p_len);

          // 수신된 CRC-32 복원 (빅엔디안)
          uint32_t recv_crc = ((uint32_t)p_crc_buf[0] << 24) |
                              ((uint32_t)p_crc_buf[1] << 16) |
                              ((uint32_t)p_crc_buf[2] <<  8) |
                              ((uint32_t)p_crc_buf[3]);

          if (calc_crc == recv_crc) {
            // 복호화
            uint8_t session_key = FCS_PROTO_KEY ^ p_salt;
            for (int i = 0; i < (int)p_len; i++) {
              p_payload[i] ^= (uint8_t)(session_key + i);
            }
            p_payload[p_len] = 0; // 널 종단

            // 명령 분기
            char resp[FCS_RESP_BUF_SIZE];
            resp[0] = '\0'; // 미정의 CMD 시 빈 문자열 보장
            int cmd_result = 0;
            if (p_cmd == FCS_CMD_TARGET_INPUT) {
              // 좌표 수신은 WAITING/FIRE_DATA 상태에서만 허용
              if ((sys->state == UI_STANDBY) || (sys->state == UI_FIRE_DATA)) {
                cmd_result = FCS_Process_Command(sys, (char*)p_payload, resp);
              } else {
                snprintf(resp, FCS_RESP_BUF_SIZE, "ERR:NOT READY");
              }
            }
            else if (p_cmd == FCS_CMD_STATUS_REQ) {
              snprintf(resp, FCS_RESP_BUF_SIZE, "STATUS:READY,%d,%c,%ld,%ld,%d",
                sys->user_pos.zone, sys->user_pos.band,
                (long)sys->user_pos.easting, (long)sys->user_pos.northing,
                (int)sys->user_pos.altitude);
              cmd_result = 1;
            }
            else {
              // 미정의 CMD -> 에러 응답
              snprintf(resp, FCS_RESP_BUF_SIZE, "ERR:UNKNOWN CMD 0x%02X", p_cmd);
            }

            // 응답 전송
            if (p_cmd == FCS_CMD_STATUS_REQ) {
              Send_Response_Packet(huart, FCS_CMD_STATUS_ACK, p_salt, resp);
            } else if (cmd_result > 0) {
              Send_Response_Packet(huart, FCS_CMD_ACK, p_salt, resp);
            } else {
              Send_Response_Packet(huart, FCS_CMD_ERR, p_salt, resp);
            }

          } else {
            // CRC 불일치 -> 암호화된 오류 응답
            Send_Response_Packet(huart, FCS_CMD_ERR, p_salt, "CRC Fail");
          }
        } else {
          /* ETX 불일치: 프레임 손상 */
        }
        p_state = P_IDLE;
        break;

      default:
        p_state = P_IDLE;
        break;
      }
    }
}


/**
 * @brief  전이한계 검사 — 방열방위각 대비 방위각 편차가
 *         서보 물리 한계를 초과하면 FCS_FIRE_ERR_TRAVERSE 설정.
 * @param  sys  fire.azimuth, laying_azimuth 참조
 * @note   fire.error == FCS_FIRE_OK일 때만 검사 (기존 에러 보존).
 */
void FCS_Check_Traverse(FCS_System_t *sys) {
  if (sys->fire.error != FCS_FIRE_OK) return; // 기존 에러 보존
  float delta_az = sys->fire.azimuth - (float)sys->laying_azimuth;
  if (delta_az > 3200.0f) delta_az -= 6400.0f;
  else { /* 정방향 범위 내 */ }
  if (delta_az < -3200.0f) delta_az += 6400.0f;
  else { /* 역방향 범위 내 */ }
  if ((delta_az > (float)SERVO_AZ_TRAVERSE_MIL)
   || (delta_az < -(float)SERVO_AZ_TRAVERSE_MIL)) {
    sys->fire.error = FCS_FIRE_ERR_TRAVERSE;
  } else {
    /* 전이한계 이내 */
  }
}

/**
 * @brief  표적 입력 명령 파싱 및 사격제원 계산.
 * @param  sys   FCS 전역 상태 (tgt_pos, fire 갱신)
 * @param  cmd   널 종단 페이로드 문자열 ("52,S,333712,4132894,100")
 * @param  resp  [out] 응답 문자열 버퍼 (크기 >= FCS_RESP_BUF_SIZE)
 * @retval  1 성공, -1 파싱 오류, -2 계산 오류
 */
int FCS_Process_Command(FCS_System_t *sys, char *cmd, char *resp) {

  // 형식: "52,S,333712,4132894,105"
  char *p = cmd;

  char *endp;

  long z_long = strtol(p, &endp, 10);
  if ((endp == p) || (*endp != ',')) {
    snprintf(resp, FCS_RESP_BUF_SIZE, "ERR:PARSE(ZONE)");
    return -1;
  }
  p = endp + 1;

  char b = *p;
  if ((b == '\0') || (*(p + 1) != ',')) {
    snprintf(resp, FCS_RESP_BUF_SIZE, "ERR:PARSE(BAND)");
    return -1;
  }
  p += 2;

  // Easting
  long e_int = strtol(p, &endp, 10);
  if ((endp == p) || (*endp != ',')) {
    snprintf(resp, FCS_RESP_BUF_SIZE, "ERR:PARSE(EAST)");
    return -1;
  }
  p = endp + 1;

  // Northing
  long n_int = strtol(p, &endp, 10);
  if ((endp == p) || (*endp != ',')) {
    snprintf(resp, FCS_RESP_BUF_SIZE, "ERR:PARSE(NORTH)");
    return -1;
  }
  p = endp + 1;

  // Altitude (마지막 필드 — 후행 문자 검증)
  long a_long = strtol(p, &endp, 10);
  if ((endp == p) || ((*endp != '\0') && (*endp != '\r') && (*endp != '\n'))) {
    snprintf(resp, FCS_RESP_BUF_SIZE, "ERR:PARSE(ALT)");
    return -1;
  }

  // 파싱값 물리 범위 검증
  if ((z_long < 1) || (z_long > 60)) {
    snprintf(resp, FCS_RESP_BUF_SIZE, "ERR:RANGE(ZONE %ld)", z_long);
    return -1;
  }
  if ((e_int < 100000) || (e_int > 999999)) {
    snprintf(resp, FCS_RESP_BUF_SIZE, "ERR:RANGE(EAST)");
    return -1;
  }
  if ((n_int < 0) || (n_int > 9999999)) {
    snprintf(resp, FCS_RESP_BUF_SIZE, "ERR:RANGE(NORTH)");
    return -1;
  }
  if ((a_long < -500) || (a_long > 9999)) {
    snprintf(resp, FCS_RESP_BUF_SIZE, "ERR:RANGE(ALT)");
    return -1;
  }

  // 시스템 갱신
  float e = (float)e_int;
  float n = (float)n_int;
  FCS_Set_Target(sys, (int)z_long, b, e, n, (float)a_long);

  // 새 임무 -> 수정사격 보정량 초기화
  sys->adj.range_m = 0;
  sys->adj.az_mil = 0;

  // 탄도 계산 (DWT 프로파일링)
  sys->fire.auto_select = 1;
  uint32_t t0 = DWT->CYCCNT;
  FCS_Calculate_FireData(sys);
  prof_calc_cycles = DWT->CYCCNT - t0;
  DBG_PRINT("[PERF] Ballistic calc: %lu cycles (%lu us)\r\n",
            (unsigned long)prof_calc_cycles,
            (unsigned long)(prof_calc_cycles / 84));

  // 계산 오류 확인
  if (sys->fire.error != FCS_FIRE_OK) {
    const char *err_str;
    if (sys->fire.error == FCS_FIRE_ERR_RANGE)       err_str = "RANGE";
    else if (sys->fire.error == FCS_FIRE_ERR_CHARGE)  err_str = "CHARGE";
    else if (sys->fire.error == FCS_FIRE_ERR_ZONE)    err_str = "ZONE";
    else                                               err_str = "CALC";
    snprintf(resp, FCS_RESP_BUF_SIZE, "ERR:%s", err_str);
    sys->state = UI_FIRE_DATA;
    return -2;
  }

  // 전이한계 검사
  FCS_Check_Traverse(sys);
  if (sys->fire.error == FCS_FIRE_ERR_TRAVERSE) {
    snprintf(resp, FCS_RESP_BUF_SIZE, "ERR:TRAVERSE");
    sys->state = UI_FIRE_DATA;
    return -2;
  }

  // 응답 문자열 구성: AZ EL CHG TOF DIST
  int az_i = (int)sys->fire.azimuth;
  int az_d = (int)((sys->fire.azimuth - (float)az_i) * 10.0f);
  if (az_d < 0) { az_d = -az_d; }
  else { /* 양수 소수부 */ }

  int el_i = (int)sys->fire.elevation;
  int el_d = (int)((sys->fire.elevation - (float)el_i) * 10.0f);
  if (el_d < 0) { el_d = -el_d; }
  else { /* 양수 소수부 */ }

  int tof_i = (int)sys->fire.time_of_flight;
  int tof_d = (int)((sys->fire.time_of_flight - (float)tof_i) * 10.0f);
  if (tof_d < 0) { tof_d = -tof_d; }
  else { /* 양수 소수부 */ }

  int dist_i = (int)sys->fire.distance_km;
  int dist_d = (int)((sys->fire.distance_km - (float)dist_i) * 10.0f);
  if (dist_d < 0) { dist_d = -dist_d; }
  else { /* 양수 소수부 */ }

  snprintf(resp, FCS_RESP_BUF_SIZE,
           "AZ:%d.%d EL:%d.%d CHG:%d TOF:%d.%d DIST:%d.%d",
           az_i, az_d, el_i, el_d, sys->fire.charge,
           tof_i, tof_d, dist_i, dist_d);

  sys->state = UI_FIRE_DATA;
  return 1;
}
