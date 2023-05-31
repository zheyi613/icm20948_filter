/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "icm20948.h"
#include "math.h"
#include "ahrs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD2DEG  57.295779F
#define DEG2RAD  0.0174533F
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t count = 0;
uint8_t tim_trig = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* X[0] = angle, X[1] = velocity bias for last state */
void kalman(float X[2], float angle, float velocity, float P[2][2], float dt)
{
  float y, S, K[2], P00_tmp, P01_tmp;
  X[1] = velocity - X[0] * dt;
  X[0] += X[1] * dt;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + 0.001);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += 0.003 * dt;

  y = angle - X[0];
  S = P[0][0] + 0.03;
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  X[0] += K[0] * y;
  X[1] += K[1] * y;

  P00_tmp = P[0][0];
  P01_tmp = P[0][1];

  P[0][0] -= K[0] * P00_tmp;
  P[0][1] -= K[0] * P01_tmp;
  P[1][0] -= K[1] * P00_tmp;
  P[1][1] -= K[1] * P01_tmp;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void enu2ned(float *x, float *y, float *z)
{
  float tmp = *x;

  *x = *y;
  *y = tmp;
  *z = -(*z);
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
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  uint8_t msg[200], len = 0;
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  float roll, pitch, yaw;
  int imu_status = 0;
  int mag_status = 0;
  if (icm20948_init(225, GYRO_250_DPS, ACCEL_4G, LP_BW_119HZ)) {
    while (1) {
      HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
      HAL_Delay(1000);
    }
  }
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { /* 50 Hz */
    while (!tim_trig)
      ;
    imu_status = icm20948_read_axis6(&ax, &ay, &az, &gx, &gy, &gz);
    gx *= DEG2RAD;
    gy *= DEG2RAD;
    gz *= DEG2RAD;
    mag_status = icm20948_read_mag(&mx, &my, &mz);

    if (imu_status == 0 && mag_status == 0) {
      enu2ned(&gx, &gy, &gz);
      enu2ned(&ax, &ay, &az);
      enu2ned(&mx, &my, &mz);
      // AHRSupdateIMU(gx, gy, gz, ax, ay, az, 0.02);
      AHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz, 0.02);
      AHRS2euler(&roll, &pitch, &yaw);
      roll *= RAD2DEG;
      pitch *= RAD2DEG;
      yaw *= RAD2DEG;
    } else if (imu_status == 0) {
      AHRSupdateIMU(gx, gy, gz, ax, ay, az, 0.02);
    }

    if (count%5 == 0) {
      len = snprintf((char *)msg, 200, "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,"
                                       "%.2f,%.2f,%.2f\n",
                                       ax, ay, az, gx, gy, gz,
                                       roll, pitch, yaw);
      // len = snprintf((char *)msg, 200, "r: %.2f, p: %.2f, y: %.2f\n\r"
      //                                  "ax: %.2f, ay: %.2f, az: %.2f\n\r"
      //                                  "gx: %.2f, gy: %.2f, gz: %.2f\n\r"
      //                                  "mx: %.2f, my: %.2f, mz: %.2f\n\r",
      //                                  roll, pitch, yaw,
      //                                  ax, ay, az, gx, gy, gz, mx, my, mz);
      CDC_Transmit_FS(msg, len);
    }
    tim_trig = 0;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == htim2.Instance) {
    tim_trig = 1;
    if (count == 50) {
      count = 0;
    }
    else
      count++;
  }
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
