/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fcs_core.h"
#include "fcs_common.h"
#include "ui.h"
#include "servo.h"
#include "iwdg.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEARTBEAT_LED_MS   500   /* LED 토글 주기 (~1Hz) */
#define SENSOR_FAIL_MAX    5     /* BMP280 연속 실패 허용 횟수 */
#define TASK_UI_PERIOD_MS     20
#define TASK_SERVO_PERIOD_MS  20
#define TASK_SENSOR_PERIOD_MS 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern FCS_System_t fcs;
extern volatile uint16_t adc_dma_buf[4];
extern UART_HandleTypeDef huart1;
extern IWDG_HandleTypeDef hiwdg;

// ST-LINK Live Expression 계측용 (Release 빌드 유지, 최적화 방지)
volatile uint32_t prof_stack_ui     = 0;
volatile uint32_t prof_stack_comm   = 0;
volatile uint32_t prof_stack_servo  = 0;
volatile uint32_t prof_stack_sensor = 0;
/* USER CODE END Variables */
/* Definitions for Task_UI */
osThreadId_t Task_UIHandle;
const osThreadAttr_t Task_UI_attributes = {
  .name = "Task_UI",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Comm */
osThreadId_t Task_CommHandle;
const osThreadAttr_t Task_Comm_attributes = {
  .name = "Task_Comm",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_Servo */
osThreadId_t Task_ServoHandle;
const osThreadAttr_t Task_Servo_attributes = {
  .name = "Task_Servo",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_Sensor */
osThreadId_t Task_SensorHandle;
const osThreadAttr_t Task_Sensor_attributes = {
  .name = "Task_Sensor",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for mtxI2C */
osMutexId_t mtxI2CHandle;
const osMutexAttr_t mtxI2C_attributes = {
  .name = "mtxI2C"
};
/* Definitions for mtxFCS */
osMutexId_t mtxFCSHandle;
const osMutexAttr_t mtxFCS_attributes = {
  .name = "mtxFCS"
};
/* Definitions for semUartRx */
osSemaphoreId_t semUartRxHandle;
const osSemaphoreAttr_t semUartRx_attributes = {
  .name = "semUartRx"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/**
 * @brief  I2C1 버스 복구 — SCL 9클럭 토글 후 페리페럴 재초기화.
 * @note   mtxI2C 보호 구간 내에서만 호출할 것.
 */
static void I2C1_BusRecovery(void)
{
  HAL_I2C_DeInit(&hi2c1);

  // SCL(PB8) GPIO 출력으로 전환, 9클럭 토글 (SDA 해제)
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin   = GPIO_PIN_8;
  gpio.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);

  for (uint8_t i = 0; i < 9; i++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    osDelay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    osDelay(1);
  }

  // I2C 재초기화 (MspInit에서 GPIO AF 복원)
  MX_I2C1_Init();
}
/* USER CODE END FunctionPrototypes */

void StartUITask(void *argument);
void StartCommTask(void *argument);
void StartServoTask(void *argument);
void StartSensorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
// DWT 사이클 카운터 활용 (main.c에서 초기화됨)
__weak void configureTimerForRunTimeStats(void)
{
  // DWT->CYCCNT는 main.c에서 이미 활성화
}

__weak unsigned long getRunTimeCounterValue(void)
{
  return (unsigned long)DWT->CYCCNT;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
// 스택 오버플로 감지: LED 빠른 점멸 + 정지
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  __disable_irq();
  for (;;) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    for (volatile uint32_t i = 0; i < 200000; i++) {}
  }
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of mtxI2C */
  mtxI2CHandle = osMutexNew(&mtxI2C_attributes);

  /* creation of mtxFCS */
  mtxFCSHandle = osMutexNew(&mtxFCS_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semUartRx */
  semUartRxHandle = osSemaphoreNew(1, 0, &semUartRx_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_UI */
  Task_UIHandle = osThreadNew(StartUITask, NULL, &Task_UI_attributes);

  /* creation of Task_Comm */
  Task_CommHandle = osThreadNew(StartCommTask, NULL, &Task_Comm_attributes);

  /* creation of Task_Servo */
  Task_ServoHandle = osThreadNew(StartServoTask, NULL, &Task_Servo_attributes);

  /* creation of Task_Sensor */
  Task_SensorHandle = osThreadNew(StartSensorTask, NULL, &Task_Sensor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartUITask */
/**
  * @brief  Task_UI 메인 루프.
  * @param  argument: 미사용
  * @retval None
  */
/* USER CODE END Header_StartUITask */
void StartUITask(void *argument)
{
  /* USER CODE BEGIN StartUITask */
  for(;;)
  {
    // 입력 + UI 로직 (mtxFCS 보호)
    osMutexAcquire(mtxFCSHandle, osWaitForever);
    FCS_Update_Input(&fcs, adc_dma_buf);
    UI_Update(&fcs);
    static FCS_System_t fcs_snap;
    fcs_snap = fcs; // 표시용 스냅샷 (UI_Draw 중 tearing 방지)
    osMutexRelease(mtxFCSHandle);

    // OLED 그리기 (mtxI2C 보호, 스냅샷 사용)
    osMutexAcquire(mtxI2CHandle, osWaitForever);
    vTaskSuspendAll(); // I2C 폴링 중 선점 방지 (ISR는 정상 동작)
    UI_Draw(&fcs_snap);
    xTaskResumeAll();
    // I2C 버스 에러 감지 -> SCL 토글 복구 + OLED 재초기화
    if ((hi2c1.State != HAL_I2C_STATE_READY) ||
        (hi2c1.ErrorCode != HAL_I2C_ERROR_NONE)) {
      I2C1_BusRecovery();
      ssd1306_Init();
    } else {
      /* I2C 정상 */
    }
    osMutexRelease(mtxI2CHandle);

    static uint32_t led_tick = 0;
    if (HAL_GetTick() - led_tick >= HEARTBEAT_LED_MS) {
      led_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    } else {
      /* 토글 주기 미도달 */
    }

    // IWDG 리프레시 (모든 태스크 대표)
    HAL_IWDG_Refresh(&hiwdg);

    // 스택 워터마크 갱신 (ST-LINK Live Expression)
    prof_stack_ui = uxTaskGetStackHighWaterMark(NULL);
    osDelay(TASK_UI_PERIOD_MS);
  }
  /* USER CODE END StartUITask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Task_Comm 메인 루프.
* @param argument: 미사용
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
  for(;;)
  {
    // UART 수신 대기 (ISR에서 세마포어 release, 100ms 타임아웃)
    osSemaphoreAcquire(semUartRxHandle, 100);

    // 링버퍼 드레인 + 명령 처리 (mtxFCS 보호)
    osMutexAcquire(mtxFCSHandle, osWaitForever);
    FCS_Task_Serial(&fcs, &huart1);
    osMutexRelease(mtxFCSHandle);

    prof_stack_comm = uxTaskGetStackHighWaterMark(NULL);
  }
  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
* @brief Task_Servo 메인 루프.
* @param argument: 미사용
* @retval None
*/
/* USER CODE END Header_StartServoTask */
void StartServoTask(void *argument)
{
  /* USER CODE BEGIN StartServoTask */
  for(;;)
  {
    // 사격제원 -> 서보 각도 (mtxFCS 보호)
    osMutexAcquire(mtxFCSHandle, osWaitForever);
    if (fcs.servo.enabled) {
      if ((fcs.state == UI_FIRE_DATA) || (fcs.state == UI_ADJUST_FIRE)) {
        if ((fcs.state == UI_FIRE_DATA) && (fcs.fire.error == FCS_FIRE_OK)) {
          Servo_UpdateFromFireData(&fcs);
        } else {
          /* ADJUST_FIRE 또는 에러: 갱신 생략 */
        }
        if (fcs.fire.error == FCS_FIRE_OK) {
          Servo_SetTarget(1U, fcs.servo.az_angle);
          Servo_SetTarget(2U, fcs.servo.el_angle);
          Servo_SlewUpdate(1U);
        } else {
          Servo_SetTarget(1U, (uint8_t)(SERVO_NEUTRAL_DEG + Servo_GetTrim()));
          Servo_SetTarget(2U, 0U);
          Servo_SlewUpdate(0U);
        }
      } else {
        Servo_SetTarget(1U, (uint8_t)(SERVO_NEUTRAL_DEG + Servo_GetTrim()));
        Servo_SetTarget(2U, 0U);
        Servo_SlewUpdate(0U);
      }
    } else {
      /* 서보 비활성화 */
    }
    osMutexRelease(mtxFCSHandle);

    prof_stack_servo = uxTaskGetStackHighWaterMark(NULL);
    osDelay(TASK_SERVO_PERIOD_MS);
  }
  /* USER CODE END StartServoTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Task_Sensor 메인 루프.
* @param argument: 미사용
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  static uint8_t sensor_fail_cnt = 0;
  for(;;)
  {
    if (sensor_fail_cnt < SENSOR_FAIL_MAX) {
      osMutexAcquire(mtxI2CHandle, osWaitForever);
      FCS_Sensor_Read();
      osMutexRelease(mtxI2CHandle);
    } else {
      /* 연속 실패: I2C 접근 중단 */
    }

    // fcs 갱신 (mtxFCS 보호, 중첩 금지)
    osMutexAcquire(mtxFCSHandle, osWaitForever);
    FCS_Sensor_Apply(&fcs);
    if (fcs.diag.sensor_ok) {
      sensor_fail_cnt = 0;
    } else {
      if (sensor_fail_cnt < SENSOR_FAIL_MAX) { sensor_fail_cnt++; }
      else { /* 최대 카운트 도달 */ }
    }
    osMutexRelease(mtxFCSHandle);

    prof_stack_sensor = uxTaskGetStackHighWaterMark(NULL);
    osDelay(TASK_SENSOR_PERIOD_MS);
  }
  /* USER CODE END StartSensorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

