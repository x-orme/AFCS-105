/* USER CODE BEGIN Header */
/**
  * @file           : main.c
  * @brief          : Main program body
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "bmp280.h"
#include "ui.h"    
#include "input.h" 
#include "flash_ops.h"
#include "fcs_core.h"
#include "servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FCS_System_t fcs; // 시스템 전역 인스턴스
volatile uint16_t adc_dma_buf[4]; // ADC DMA 순환 버퍼 (HALFWORD)

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_adc1;
extern osSemaphoreId_t semUartRxHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  UART 수신 완료 콜백 — FCS 코어 링 버퍼로 전달.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    FCS_UART_RxCallback(huart);
    osSemaphoreRelease(semUartRxHandle);
  }
#ifdef DEBUG
  /* 디버그 전용: USART2 RX -> 링버퍼 (서보 트림 l/r 키 지원) */
  else if (huart->Instance == USART2) {
    FCS_UART_RxCallback(huart);
  }
#endif
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  FCS_UART_TxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  FCS_UART_ErrorCallback(huart);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_TIM3_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  // OLED 초기화
  HAL_Delay(100);
  ssd1306_Init();
  ssd1306_Fill(0);
  HAL_IWDG_Refresh(&hiwdg);

  // I2C 버스 스캔 — 응답하는 장치 주소 출력
  DBG_PRINT("\r\n[I2C] Bus scan...\r\n");
  for (uint8_t addr = 0x03; addr < 0x78; addr++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 1, 10) == HAL_OK) {
      DBG_PRINT("  Found: 0x%02X", addr);
      if (addr == 0x3C || addr == 0x3D) DBG_PRINT(" (OLED)");
      if (addr == 0x76 || addr == 0x77) DBG_PRINT(" (BMP280)");
      DBG_PRINT("\r\n");
    }
  }
  HAL_IWDG_Refresh(&hiwdg);

  // BMP280 초기화
  HAL_Delay(100);
  BMP280_Init();
  HAL_IWDG_Refresh(&hiwdg);
  
  // 시스템 및 UI 초기화
  FCS_Init_System(&fcs);
  UI_Init(&fcs); // UI 상태 초기화
  
  // 저장된 포 위치 로드
  Flash_Load_BatteryPos(&fcs);

  // ADC DMA 연속 변환 시작 (백그라운드 동작)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, 4);
  // 콜백 미사용 -> DMA TC/HT 인터럽트 비활성화
  __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_TC | DMA_IT_HT);

  // 서보 PWM 시작 + 중립 위치
  Servo_Init(&htim3);
  fcs.servo.enabled = 1U;
  DBG_PRINT("\r\n[SERVO] Init OK (TIM3 CH1/CH2)\r\n");

  // UART 수신 시작 (인터럽트 체인 개시)
  FCS_Serial_Start(&huart1);
#ifdef DEBUG
  FCS_Serial_Start(&huart2); // 디버그 전용: 서보 트림 키(l/r) 수신
#endif
  
  // DWT 사이클 카운터 초기화 (성능 프로파일링)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // .isr_vector + .text 무결성 검증 (HW CRC-32)
  {
    extern uint32_t _etext; /* 링커 심볼: .text 섹션 끝 */
    uint32_t fw_end  = (uint32_t)&_etext;
    uint32_t fw_size = fw_end - FLASH_BASE; /* FLASH_BASE = 0x08000000 */
    volatile uint32_t fw_crc = HW_CRC32((const uint8_t*)FLASH_BASE, fw_size);
    (void)fw_crc; /* Release 빌드 unused 경고 억제 (volatile로 계산은 보장) */
    DBG_PRINT("\r\n[FW] CRC=0x%08lX (%lu bytes)\r\n",
              (unsigned long)fw_crc, (unsigned long)fw_size);
  }

  DBG_PRINT("\r\n[FCS] System Ready. Waiting for Commands...\r\n");
  
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // osKernelStart() 이후 도달하지 않음
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  printf 출력 USART2 (디버그 시리얼) 리다이렉트.
 */
// printf -> USART2 리다이렉트
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 1000);
    return len;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
