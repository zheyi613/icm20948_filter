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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD2DEGREE  57.29577951
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
extern void AHRSupdate(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float mx, float my, float mz,
                       float dt);
extern void AHRS2euler(float *r, float *p, float *y);
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
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  float ax=0, ay=0, az=0, gx=0, gy=0, gz=0, mx=0, my=0, mz=0;
  float g_square;
  float roll = 0, pitch = 0, yaw = 0;
  float last_roll = 0, last_yaw = 0;
  float x[2] = {0}, y[2] = {0}, z[2] = {0};
  float Px[2][2] = {{1, 0}, {0, 1}};
  float Py[2][2] = {{1, 0}, {0, 1}};
  float Pz[2][2] = {{1, 0}, {0, 1}};
  uint8_t msg[400], len = 0;
  int status = 0;
  status = icm20948_init(225, GYRO_250_DPS, ACCEL_4G);
  if (status) {
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
  {
    while (!tim_trig)
      ;
    // icm20948_read_axis6(&ax, &ay, &az, &gx, &gy, &gz);
    status = icm20948_read_axis9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    if (status == 0) {
      g_square = ax * ax + ay * ay + az * az;
      /* kalman filter */
      // roll = atan2f(ay, az) * RAD2DEGREE;
      // pitch = -atan2f(ax, sqrtf(ay * ay + az * az)) * RAD2DEGREE;
      // yaw = -atan2f(my, mx) * RAD2DEGREE;
      // if (abs(roll - last_roll) > 150) {
      //   x[0] = roll;
      // }
      // if (abs(yaw - last_yaw) > 150) {
      //   z[0] = yaw;
      // }

      // kalman(x, roll, gx, Px, 0.1);
      // kalman(y, pitch, gy, Py, 0.1);
      // kalman(z, yaw, gz, Pz, 0.1);

      // last_roll = roll;
      // last_yaw = yaw;
      /* AHRS */
      // AHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz, 0.02);
      // AHRS2euler(&roll, &pitch, &yaw);
      // len = snprintf((char *)msg, 400, "ax: %.4f, ay: %.4f, az: %.4f, g^2: %.4f\n\r"
      //                                  "gx: %.2f, gy: %.2f, gz: %.2f\n\r"
      //                                  "mx: %.2f, my: %.2f, mz: %.2f\n\r"
      //                                  "r: %.2f, p: %.2f, y: %.2f\n\r",
      //                                  ax, ay, az, g_square,
      //                                  gx, gy, gz,
      //                                  mx, my, mz,
      //                                  roll, pitch, yaw);
      len = snprintf((char *)msg, 400, "%.2f,%.2f,%.2f\n", mx, my, mz);
    } else if (status == 1) {
      // len = snprintf((char *)msg, 400, "accel/gyro failed!%s", "\n\r");
    } else if (status == 2) {
      // len = snprintf((char *)msg, 400, "mag failed!%s", "\n\r");
    }
    if (count%2 == 0) {
      // HAL_UART_Transmit(&huart1, msg, len, 100);
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
