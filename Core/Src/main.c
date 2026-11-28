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
#include "imu.h"
#include "baro.h"
#include "flash.h"
#include "benchmark.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* Button state */
#define BUTTON_RELEASED                    0U
#define BUTTON_PRESSED                     1U
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

__IO uint32_t BspButtonState = BUTTON_RELEASED;
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static imu_data_t imu_data;
static imu_status_t imu_status;

static char uart_buf[200];
static uint32_t imu_error_count = 0;
static baro_data_t baro_data;
static baro_status_t baro_status;
static uint8_t baro_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void uart_print(const char *message)
{
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)message,
                      (uint16_t)strlen(message),
                      100);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* Enable SPI so SCK idles high (SPI mode 3) */
  __HAL_SPI_ENABLE(&hspi1);
  HAL_Delay(10);

  uart_print("\r\nBoot OK\r\n");

  /* ---------- IMU ---------- */
  imu_status = imu_init();
  if (imu_status == IMU_OK)
  {
      uart_print("IMU OK\r\n");
  }
  else
  {
      snprintf(uart_buf, sizeof(uart_buf), "IMU init failed: %d\r\n", (int)imu_status);
      uart_print(uart_buf);
  }

  /* ---------- Barometer ---------- */
  uint8_t baro_id = 0;
  baro_status = baro_read_reg(0x00, &baro_id);
  snprintf(uart_buf, sizeof(uart_buf), "BARO ID: 0x%02X, Status: %d\r\n",
           (unsigned int)baro_id, (int)baro_status);
  uart_print(uart_buf);

  baro_status = baro_init();
  if (baro_status == BARO_OK)
  {
      baro_ready = 1;
      uart_print("BARO OK\r\n");
  }
  else
  {
      snprintf(uart_buf, sizeof(uart_buf), "BARO init failed: %d\r\n", (int)baro_status);
      uart_print(uart_buf);
  }

  /* ---------- Flash: run ONCE at startup ---------- */
  {
      uint8_t flash_id[3] = {0};
      uint32_t test_address = 0xFFF000U;   /* last 4 KB sector */
      uint8_t write_data[8] = { 0x12, 0x34, 0x56, 0x78, 0xAA, 0xBB, 0xCC, 0xDD };
      uint8_t read_data[8]  = {0};
      flash_status_t flash_status;

      flash_status = flash_init();          /* reset + ID check + wait ready */
      flash_read_id(flash_id);
      snprintf(uart_buf, sizeof(uart_buf), "FLASH ID: %02X %02X %02X | Init: %d\r\n",
               (unsigned int)flash_id[0], (unsigned int)flash_id[1],
               (unsigned int)flash_id[2], (int)flash_status);
      uart_print(uart_buf);

      if (flash_status == FLASH_OK)
      {
          flash_status = flash_sector_erase(test_address);
          uart_print(flash_status == FLASH_OK ? "FLASH ERASE OK\r\n" : "FLASH ERASE FAILED\r\n");
      }

      if (flash_status == FLASH_OK)
      {
          flash_status = flash_page_program(test_address, write_data, sizeof(write_data));
          uart_print(flash_status == FLASH_OK ? "FLASH PROGRAM OK\r\n" : "FLASH PROGRAM FAILED\r\n");
      }

      if (flash_status == FLASH_OK)
      {
          flash_status = flash_read(test_address, read_data, sizeof(read_data));
          if (flash_status == FLASH_OK && memcmp(write_data, read_data, sizeof(write_data)) == 0)
          {
              uart_print("FLASH READBACK PASS\r\n");
          }
          else
          {
              uart_print("FLASH READBACK FAILED\r\n");
          }
      }

      snprintf(uart_buf, sizeof(uart_buf), "FLASH FINAL STATUS: %d\r\n", (int)flash_status);
      uart_print(uart_buf);

  }
  benchmark_init();

  const uint32_t iterations = 1000;

  uint32_t min_cycles = UINT32_MAX;
  uint32_t max_cycles = 0;
  uint64_t total_cycles = 0;

  uint32_t failures = 0;

  for (uint32_t i = 0; i < iterations; i++)
  {
      uint32_t start = benchmark_get_cycles();

      imu_status = imu_read(&imu_data);

      uint32_t elapsed =
          benchmark_get_cycles() - start;

      if (imu_status != IMU_OK)
      {
          failures++;
          continue;
      }

      total_cycles += elapsed;

      if (elapsed < min_cycles)
          min_cycles = elapsed;

      if (elapsed > max_cycles)
          max_cycles = elapsed;
  }

  uint32_t successful = iterations - failures;

  if (successful > 0)
  {
      uint32_t avg_cycles =
          (uint32_t)(total_cycles / successful);

      uart_print("\r\n--- IMU POLLING BENCHMARK ---\r\n");

      snprintf(uart_buf, sizeof(uart_buf),
               "Iterations: %lu | Failures: %lu\r\n",
               (unsigned long)iterations,
               (unsigned long)failures);
      uart_print(uart_buf);

      snprintf(uart_buf, sizeof(uart_buf),
               "Average: %lu cycles | %.2f us\r\n",
               (unsigned long)avg_cycles,
               benchmark_cycles_to_us(avg_cycles));
      uart_print(uart_buf);

      snprintf(uart_buf, sizeof(uart_buf),
               "Minimum: %lu cycles | %.2f us\r\n",
               (unsigned long)min_cycles,
               benchmark_cycles_to_us(min_cycles));
      uart_print(uart_buf);

      snprintf(uart_buf, sizeof(uart_buf),
               "Maximum: %lu cycles | %.2f us\r\n",
               (unsigned long)max_cycles,
               benchmark_cycles_to_us(max_cycles));
      uart_print(uart_buf);
  }
  else
  {
      uart_print("BENCHMARK FAILED\r\n");
  }
  /* USER CODE END 2 */

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* USER CODE BEGIN BSP */

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1)
    {
        if (BspButtonState == BUTTON_PRESSED)
        {
            BspButtonState = BUTTON_RELEASED;
            uart_print("Button pressed\r\n");
        }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        /* IMU */
        imu_status = imu_read(&imu_data);
        if (imu_status == IMU_OK)
        {
            snprintf(uart_buf, sizeof(uart_buf),
                     "GX: %.3f GY: %.3f GZ: %.3f | AX: %.3f AY: %.3f AZ: %.3f\r\n",
                     imu_data.gx, imu_data.gy, imu_data.gz,
                     imu_data.ax, imu_data.ay, imu_data.az);
            uart_print(uart_buf);
        }
        else
        {
            imu_error_count++;
            snprintf(uart_buf, sizeof(uart_buf), "IMU read failed: %d | Errors: %lu\r\n",
                     (int)imu_status, (unsigned long)imu_error_count);
            uart_print(uart_buf);
        }

        /* Barometer */
        if (baro_ready)
        {
            baro_status = baro_read(&baro_data);
            if (baro_status == BARO_OK)
            {
                snprintf(uart_buf, sizeof(uart_buf),
                         "Pressure: %.2f Pa | Temp: %.2f C | Altitude: %.2f m\r\n",
                         baro_data.pressure, baro_data.temperature, baro_data.altitude);
                uart_print(uart_buf);
            }
            else if (baro_status != BARO_ERR_NOT_READY)
            {
                snprintf(uart_buf, sizeof(uart_buf), "BARO read failed: %d\r\n", (int)baro_status);
                uart_print(uart_buf);
            }
        }

        HAL_Delay(100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;   /* 84 MHz / 64 = 1.3 MHz */
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BARO_CS_Pin */
  GPIO_InitStruct.Pin = BARO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BARO_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==USER_BUTTON_PIN)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

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
