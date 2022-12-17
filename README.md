# MPU9250
MPU9250 (GY-91) driver for STM32 with HAL using SPI (spi1 by default).

## Setup
Define `GY_CS` pin in STM32CubeMX that will be used as Chip Select for the device, or pick the one you need in the `MPU9250_Config.h`.

## How To Use
Use `MPU9250_Init()` to initialize the device.

Then use this to retireve the raw data:
```c
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float AccData[3];
	float MagData[3];
	float GyroData[3];
	float TempData;
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

	HAL_Delay(1000);
	MPU9250_Init();
	MPU9250_SetAccelRange(ACCEL_RANGE_8G);
	MPU9250_SetGyroRange(GYRO_RANGE_500DPS);
	MPU9250_setDlpfBandwidth(DLPF_BANDWIDTH_41HZ);
	MPU9250_setSrd(0);
	MPU9250_calibrateGyro();

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		MPU9250_GetData(AccData, MagData, GyroData, &TempData);
		sprintf(data, "%.2f \t", AccData[0]);
		HAL_UART_Transmit (&huart1, (uint8_t*)data, 8, 10);
		sprintf(data, "%.2f \t", AccData[1]);
		HAL_UART_Transmit (&huart1, (uint8_t*)data, 8, 10);
		sprintf(data, "%.2f \t", AccData[2]);
		HAL_UART_Transmit (&huart1, (uint8_t*)data, 8, 10);
		sprintf(data, "%.2f \t", GyroData[0]);
		HAL_UART_Transmit (&huart1, (uint8_t*)data, 8, 10);
		sprintf(data, "%.2f \t", GyroData[1]);
		HAL_UART_Transmit (&huart1, (uint8_t*)data, 8, 10);
		sprintf(data, "%.2f \t", GyroData[2]);
		HAL_UART_Transmit (&huart1, (uint8_t*)data, 8, 10);
		sprintf(data, "%.2f \t", MagData[0]);
		HAL_UART_Transmit (&huart1, (uint8_t*)data, 8, 10);
		sprintf(data, "%.2f \t", MagData[1]);
		HAL_UART_Transmit (&huart1, (uint8_t*)data, 8, 10);
		sprintf(data, "%.2f \t", MagData[2]);
		HAL_UART_Transmit (&huart1, (uint8_t*)data, 8, 10);
		sprintf(data, "%.2f \t\n", TempData);
		HAL_UART_Transmit (&huart1, (uint8_t*)data,8, 10);
		HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

```
