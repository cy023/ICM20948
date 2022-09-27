/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include "stm32_icm20948.h"
#include "dwt_delay.h"

#define RAD2DEG     57.2957795131
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
STM32ICM20948Settings icmSettings =
{
  .mode = 1,                     // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = true,      // Enables gyroscope output
  .enable_accelerometer = true,  // Enables accelerometer output
  .enable_magnetometer = true,   // Enables magnetometer output
  .enable_quaternion = true,     // Enables quaternion output
  .gyroscope_frequency = 1,      // Max frequency = 225, min frequency = 1
  .accelerometer_frequency = 1,  // Max frequency = 225, min frequency = 1
  .magnetometer_frequency = 1,   // Max frequency = 70, min frequency = 1
  .quaternion_frequency = 50     // Max frequency = 225, min frequency = 50
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void quat2euler(float q0, float q1, float q2, float q3, float *r, float *p, float *y)
{
    // Roll (x-axis rotation)
    float sinr_cosp = +2.0 * (q0 * q1 + q2 * q3);
    float cosr_cosp = +1.0 - 2.0 * (q1 * q1 + q2 * q2);
    *r = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = +2.0 * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1)
        *p = copysign(33.14159265358979323846 / 2, sinp); // use 90 degrees if out of range
    else
        *p = asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = +2.0 * (q0 * q3 + q1 * q2);
    float cosy_cosp = +1.0 - 2.0 * (q2 * q2 + q3 * q3);
    *y = atan2(siny_cosp, cosy_cosp);
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  // uint8_t who;
  // uint8_t buF[11];
  // HAL_I2C_Mem_Read(&ICM20948_I2C, ICM_I2C_ADDR_REVA << 1, 0x00, I2C_MEMADD_SIZE_8BIT, &who, 1, HAL_MAX_DELAY);
  // sprintf(buF, "who 0x%x\r\n", who);
  // HAL_UART_Transmit(&ICM20948_UART, buF, 11, 100);
  // HAL_Delay(10);

  ICM20948_init(icmSettings);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // DWT_Delay(150000);
    // // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // DWT_Delay(623);
    // // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // DWT_Delay(2000);
    // // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // DWT_Delay(23);

    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float mag_x, mag_y, mag_z;
    float quat_w, quat_x, quat_y, quat_z;
    float roll, pitch, yaw;
    char sensor_string_buff[128];

    // Must call this often in main loop -- updates the sensor values
    ICM20948_task();

    if (ICM20948_gyroDataIsReady()) {
      ICM20948_readGyroData(&gyro_x, &gyro_y, &gyro_z);
      sprintf(sensor_string_buff, "Gyro (deg/s): [%f,%f,%f]\r\n", gyro_x, gyro_y, gyro_z);
      HAL_UART_Transmit(&ICM20948_UART, sensor_string_buff, strlen(sensor_string_buff), 100);
    }

    if (ICM20948_accelDataIsReady()) {
      ICM20948_readAccelData(&accel_x, &accel_y, &accel_z);
      sprintf(sensor_string_buff, "Accel (g): [%f,%f,%f]\r\n", accel_x, accel_y, accel_z);
      HAL_UART_Transmit(&ICM20948_UART, sensor_string_buff, strlen(sensor_string_buff), 100);
    }

    if (ICM20948_magDataIsReady()) {
      ICM20948_readMagData(&mag_x, &mag_y, &mag_z);
      sprintf(sensor_string_buff, "Mag (uT): [%f,%f,%f]\r\n", mag_x, mag_y, mag_z);
      HAL_UART_Transmit(&ICM20948_UART, sensor_string_buff, strlen(sensor_string_buff), 100);
    }

    if (ICM20948_quatDataIsReady()) {
      ICM20948_readQuatData(&quat_w, &quat_x, &quat_y, &quat_z);
      // sprintf(sensor_string_buff, "Quat (deg): [%f,%f,%f,%f]\r\n", quat_w, quat_x, quat_y, quat_z);
      // HAL_UART_Transmit(&ICM20948_UART, sensor_string_buff, strlen(sensor_string_buff), 100);

      quat2euler(quat_w, quat_x, quat_y, quat_z, &roll, &pitch, &yaw);
      sprintf(sensor_string_buff, "Attitude (deg): [%f,%f,%f]\r\n", roll*RAD2DEG, pitch*RAD2DEG, yaw*RAD2DEG);
      HAL_UART_Transmit(&ICM20948_UART, sensor_string_buff, strlen(sensor_string_buff), 100);
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
