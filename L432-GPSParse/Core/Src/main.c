/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE 1
#define FALSE 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define GPS_DIAG 			0
#define GPS_RX_BUF_SIZE   	512
#define GPS_PROC_BUF_SIZE 	512
#define NMEA_LINE_MAX     	128
char UsrString[100] = {0};
static uint8_t gpsRxBuf[GPS_RX_BUF_SIZE];

// buffer handed off to main loop
static volatile uint16_t gpsProcLen = 0;
static uint8_t gpsProcBuf[GPS_PROC_BUF_SIZE];
static volatile uint8_t gpsChunkReady = 0;
static volatile uint32_t gps_isr_chunks = 0;

// NMEA line assembly state (used in main loop now)
static uint8_t  nmeaLine[NMEA_LINE_MAX];
static uint16_t nmeaIdx = 0;

// output to user
static volatile uint8_t gps_dt_ready = 0;
static char gps_dt_msg[80];

// Diagnostics
static volatile uint32_t isr_chunks = 0;
static volatile uint32_t isr_drops  = 0;

// RTC variables
static volatile uint8_t rtc_sync_ready = 0;
static volatile int rtc_yy, rtc_mm, rtc_dd, rtc_hh, rtc_mi, rtc_ss;
static uint8_t rtc_has_been_synced = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void PrintSerial(char *format,...);
static int  nmea_checksum_ok(const char *s);
static void process_nmea_line(const char *line);
static int  parse_rmc_datetime_utc(const char *line,
                                  int *yy, int *mm, int *dd,
                                  int *hh, int *mi, int *ss);
static uint8_t rtc_weekday_from_ymd(int y, int m, int d);
static void rtc_set_utc_from_gps(int y, int m, int d, int hh, int mi, int ss);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_IT(&huart1, gpsRxBuf, GPS_RX_BUF_SIZE);

  #if GPS_DIAG
    char msg[64];
	snprintf(msg, sizeof(msg), "Start RxToIdle: %d\r\n", (int)st);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)strlen(msg), 200);
  #endif

  #if GPS_DIAG
    const char *boot = "BOOT: GPS parser start\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)boot, (uint16_t)strlen(boot), 200);
  #endif

  // Start receive-to-idle on USART1
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, gpsRxBuf, GPS_RX_BUF_SIZE);

  // Optional: disable Half Transfer interrupt if DMA is used under the hood (safe to call anyway)
  // __HAL_UART_DISABLE_IT(&huart1, UART_IT_HT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Process any received chunk from ISR
	      if (gpsChunkReady)
	      {
	          // While gpsChunkReady == 1, ISR will NOT overwrite gpsProcBuf
	          uint16_t len = gpsProcLen;
	          if (len > GPS_PROC_BUF_SIZE) len = GPS_PROC_BUF_SIZE;

	          for (uint16_t i = 0; i < len; i++)
	          {
	              uint8_t c = gpsProcBuf[i];

	              if (c == '\r')
	                  continue;

	              if (c == '\n')
	              {
	                  // terminate line
	                  if (nmeaIdx < NMEA_LINE_MAX) nmeaLine[nmeaIdx] = '\0';
	                  else nmeaLine[NMEA_LINE_MAX - 1] = '\0';

	                  if (nmeaIdx > 0)
	                      process_nmea_line((const char*)nmeaLine);

	                  nmeaIdx = 0;
	              }
	              else
	              {
	                  if (nmeaIdx < (NMEA_LINE_MAX - 1))
	                      nmeaLine[nmeaIdx++] = c;
	                  else
	                      nmeaIdx = 0; // overflow: drop this line
	              }
	          }

	          // Now safe to release buffer for ISR to write next chunk
	          gpsChunkReady = 0;
	      }

	      // Transmit parsed datetime when ready (never from ISR)
	      if (gps_dt_ready)
	      {
	          gps_dt_ready = 0;
	          HAL_UART_Transmit(&huart2, (uint8_t*)gps_dt_msg,
	                            (uint16_t)strlen(gps_dt_msg), 200);
	      }

	  #if GPS_DIAG
	      // Heartbeat/counters once per second
	      static uint32_t t0 = 0;
	      if (HAL_GetTick() - t0 >= 1000)
	      {
	          t0 = HAL_GetTick();
	          char msg[96];
	          snprintf(msg, sizeof(msg),
	                   "HB chunks=%lu drops=%lu ready=%u\r\n",
	                   (unsigned long)isr_chunks,
	                   (unsigned long)isr_drops,
	                   (unsigned)gpsChunkReady);
	          HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)strlen(msg), 200);
	      }
	  #endif

	      if (rtc_sync_ready)
	      {
	          rtc_sync_ready = 0;
	          rtc_set_utc_from_gps(rtc_yy, rtc_mm, rtc_dd, rtc_hh, rtc_mi, rtc_ss);

	          // Optional confirmation:
			  #if GPS_DIAG
	            HAL_UART_Transmit(&huart2, (uint8_t*)"RTC synced\r\n", 12, 200);
			  #endif
	      }

	      RTC_TimeTypeDef t;
	      RTC_DateTypeDef d;

	      HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
	      HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN); // must be called after GetTime

	      char msg[80];
	      snprintf(msg, sizeof(msg), "RTC: 20%02u-%02u-%02u %02u:%02u:%02u\r\n",
	               d.Year, d.Month, d.Date, t.Hours, t.Minutes, t.Seconds);
		  #if GPS_DIAG
	        HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)strlen(msg), 200);
          #endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Returns 1..7 = Monday..Sunday (HAL convention: RTC_WEEKDAY_MONDAY etc.)
static uint8_t rtc_weekday_from_ymd(int y, int m, int d)
{
    // Sakamoto algorithm. 0=Sunday..6=Saturday
    static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if (m < 3) y -= 1;
    int w = (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;

    // Convert: 0=Sunday..6=Saturday -> 1=Monday..7=Sunday
    // w=1 means Monday already? Actually w=0 Sunday.
    // Map: Sunday(0)->7, Monday(1)->1, Tuesday(2)->2, ... Saturday(6)->6
    if (w == 0) return 7;
    return (uint8_t)w;
}

static void rtc_set_utc_from_gps(int y, int m, int d, int hh, int mi, int ss)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    // Use BIN format (recommended)
    sTime.Hours = (uint8_t)hh;
    sTime.Minutes = (uint8_t)mi;
    sTime.Seconds = (uint8_t)ss;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    // STM32 HAL RTC year is 0..99
    sDate.Year = (uint8_t)(y % 100);
    sDate.Month = (uint8_t)m;
    sDate.Date = (uint8_t)d;

    // WeekDay: RTC_WEEKDAY_MONDAY (1) .. RTC_WEEKDAY_SUNDAY (7)
    sDate.WeekDay = rtc_weekday_from_ymd(y, m, d);

    // IMPORTANT: per STM32 RTC rules, set TIME then DATE (or follow your reference manual),
    // and use consistent format (RTC_FORMAT_BIN here).
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
        // optional: error handling
        return;
    }

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
        // optional: error handling
        return;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        // Count callbacks (optional; remove if you don't want diagnostics)
        isr_chunks++;

        // If the main loop hasn't consumed the previous chunk yet, drop this one
        if (gpsChunkReady)
        {
            isr_drops++;
        }
        else
        {
            // Bound size to our processing buffer
            if (Size > GPS_PROC_BUF_SIZE) Size = GPS_PROC_BUF_SIZE;

            // Copy received bytes to a buffer the main loop will parse
            memcpy(gpsProcBuf, gpsRxBuf, Size);
            gpsProcLen = Size;

            // Mark ready AFTER copying length+data
            gpsChunkReady = 1;
        }

        // Re-arm reception immediately so we don't miss bytes
        // (This must be called every time you get an event.)
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, gpsRxBuf, GPS_RX_BUF_SIZE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // If any UART error happens on USART1, re-arm reception
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, gpsRxBuf, GPS_RX_BUF_SIZE);
    }
}

// NMEA checksum: sentence "$....*HH"
static int nmea_checksum_ok(const char *s)
{
    if (!s || s[0] != '$') return 0;

    const char *star = strchr(s, '*');
    if (!star) return 0;

    uint8_t cs = 0;
    for (const char *p = s + 1; p < star; p++)
        cs ^= (uint8_t)(*p);

    if (strlen(star) < 3) return 0;

    char hh[3] = { star[1], star[2], 0 };
    uint8_t sent = (uint8_t)strtoul(hh, NULL, 16);

    return (cs == sent);
}

// Parse UTC time+date from $GPRMC / $GNRMC (status must be 'A')
static int parse_rmc_datetime_utc(const char *line,
                                 int *yy, int *mm, int *dd,
                                 int *hh, int *mi, int *ss)
{
    // Accept $GPRMC or $GNRMC
    if (strncmp(line, "$GPRMC", 6) != 0 && strncmp(line, "$GNRMC", 6) != 0)
        return 0;

    // Helper: get pointer+length of NMEA field index (0-based after $GxRMC)
    // Field mapping (RMC):
    // 0: $GxRMC
    // 1: time hhmmss.sss
    // 2: status A/V
    // ...
    // 9: date ddmmyy
    const char *p = line;
    int field = 0;

    const char *f1 = NULL; int f1_len = 0; // time
    const char *f2 = NULL; int f2_len = 0; // status
    const char *f9 = NULL; int f9_len = 0; // date

    // Walk through characters, splitting on ',' but PRESERVING empty fields
    const char *start = p;
    while (*p && *p != '\r' && *p != '\n')
    {
        if (*p == ',' || *p == '*')  // '*' begins checksum, end of data fields
        {
            int len = (int)(p - start);

            if (field == 1) { f1 = start; f1_len = len; }
            if (field == 2) { f2 = start; f2_len = len; }
            if (field == 9) { f9 = start; f9_len = len; }

            if (*p == '*') break;    // stop at checksum marker

            field++;
            p++;
            start = p;
            continue;
        }
        p++;
    }

    // If line ended without ',' processing last field (unlikely for RMC), handle it:
    if (*p == '\0' || *p == '\r' || *p == '\n')
    {
        int len = (int)(p - start);
        if (field == 1) { f1 = start; f1_len = len; }
        if (field == 2) { f2 = start; f2_len = len; }
        if (field == 9) { f9 = start; f9_len = len; }
    }

    // Validate we got needed fields
    if (!f1 || !f2 || !f9) return 0;
    if (f2_len < 1) return 0;
    if (f2[0] != 'A') return 0; // need valid fix

    // Time: need at least hhmmss
    if (f1_len < 6) return 0;
    int HH = (f1[0]-'0')*10 + (f1[1]-'0');
    int MI = (f1[2]-'0')*10 + (f1[3]-'0');
    int SS = (f1[4]-'0')*10 + (f1[5]-'0');

    // Date: ddmmyy
    if (f9_len < 6) return 0;
    int DD = (f9[0]-'0')*10 + (f9[1]-'0');
    int MM = (f9[2]-'0')*10 + (f9[3]-'0');
    int YY = (f9[4]-'0')*10 + (f9[5]-'0');

    *yy = 2000 + YY;
    *mm = MM;
    *dd = DD;
    *hh = HH;
    *mi = MI;
    *ss = SS;
    return 1;
}

static void process_nmea_line(const char *line)
{
    //if (!nmea_checksum_ok(line))
     //   return;

    int yy, mm, dd, hh, mi, ss;
    if (parse_rmc_datetime_utc(line, &yy, &mm, &dd, &hh, &mi, &ss))
    {
        snprintf(gps_dt_msg, sizeof(gps_dt_msg),
                 "GPS UTC: %04d-%02d-%02d %02d:%02d:%02d\r\n",
                 yy, mm, dd, hh, mi, ss);
        gps_dt_ready = 1;
    }

    // Request RTC sync in main loop

    if (!rtc_has_been_synced)
    {
        rtc_yy = yy; rtc_mm = mm; rtc_dd = dd;
        rtc_hh = hh; rtc_mi = mi; rtc_ss = ss;
        rtc_sync_ready = 1;
        rtc_has_been_synced = 1;
    }
}

void PrintSerial(char *format,...)
{

	char str[80];

	/*Extract the the argument list using VA apis */
  	va_list args;
  	va_start(args, format);
  	vsprintf(str, format,args);
  	HAL_UART_Transmit(&huart2,(uint8_t *)str, strlen(str),HAL_MAX_DELAY);
  	va_end(args);

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
