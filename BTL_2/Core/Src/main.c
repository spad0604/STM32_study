/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdint.h>
#include <math.h>
// #include <kalman.h>  // Tạm thời tắt Kalman để test
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Địa chỉ I2C của MPU6050 (thay đổi nếu dùng sensor khác)
#define MPU6050_ADDR 0xD0
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// extern USBD_HandleTypeDef hUsbDeviceFS;  // Đã tắt USB
// KalmanFilter_t kalman_x, kalman_y, kalman_z;

uint8_t gyro_data[6];
int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
// uint8_t usb_connected = 0;  // Không cần nữa
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Read_Sensor_Data(void);
void MPU6050_Init(void);
void Send_Mouse(int8_t dx, int8_t dy);
void Send_Mouse_UART(int8_t dx, int8_t dy);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Gửi mouse data qua UART (thay vì USB HID)
  * @param  dx: Delta X (-127 to 127)
  * @param  dy: Delta Y (-127 to 127)
  * @retval None
  */
void Send_Mouse_UART(int8_t dx, int8_t dy)
{
    char buffer[50];
    
    // Format: "MOUSE:dx,dy\n" - dễ parse trên PC
    sprintf(buffer, "MOUSE:%d,%d\n", dx, dy);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief  Gửi mouse report qua USB HID
  * @param  dx: Delta X (-127 to 127)
  * @param  dy: Delta Y (-127 to 127)
  * @retval None
  */
void Send_Mouse(int8_t dx, int8_t dy)
{
    // Gửi qua cả USB và UART để test
    uint8_t buffer[4] = {0}; // [buttons, x, y, wheel]
    buffer[1] = dx;
    buffer[2] = dy;
    
    // Chỉ gửi khi có movement để tránh spam
    if(dx != 0 || dy != 0) {
        // USBD_HID_SendReport(&hUsbDeviceFS, buffer, 4);
        Send_Mouse_UART(dx, dy); // Gửi qua UART backup
    }
}

/**
  * @brief  Khởi tạo MPU6050
  * @retval None
  */
void MPU6050_Init(void)
{
    uint8_t data;
    
    // Wake up MPU6050
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);
    
    // Set sample rate to 1kHz
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, &data, 1, HAL_MAX_DELAY);
    
    // Set DLPF to 44Hz
    data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
    
    // Set gyro range to ±2000°/s
    data = 0x18;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
    
    // Set accelerometer range to ±8g
    data = 0x10;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
    
    // Delay để đảm bảo khởi tạo hoàn tất
    HAL_Delay(100);
}

/**
  * @brief  Đọc dữ liệu từ sensor
  * @retval None
  */
void Read_Sensor_Data(void)
{
    // Đọc dữ liệu gyroscope từ MPU6050
    if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 1, gyro_data, 6, HAL_MAX_DELAY) == HAL_OK)
    {
        raw_gyro_x = (int16_t)(gyro_data[0] << 8 | gyro_data[1]);
        raw_gyro_y = (int16_t)(gyro_data[2] << 8 | gyro_data[3]);
        raw_gyro_z = (int16_t)(gyro_data[4] << 8 | gyro_data[5]);

        // Debug: gửi dữ liệu qua UART
        char debug_buffer[50];
        sprintf(debug_buffer, "Gyro: X=%d, Y=%d, Z=%d\n", raw_gyro_x, raw_gyro_y, raw_gyro_z);
        HAL_UART_Transmit(&huart1, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
    }
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  // Gửi thông báo khởi động qua UART
  HAL_UART_Transmit(&huart1, (uint8_t*)"=== STM32 UART Test Started ===\n", 33, HAL_MAX_DELAY);
  
  // Thêm delay dài hơn để power ổn định
  HAL_Delay(2000);
  
  // Khởi tạo MPU6050 trước
  HAL_UART_Transmit(&huart1, (uint8_t*)"Initializing MPU6050...\n", 25, HAL_MAX_DELAY);
  MPU6050_Init();
  HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 initialized!\n", 22, HAL_MAX_DELAY);
  
  // Delay để sensor ổn định
  HAL_Delay(1000);
  
  // Khởi tạo Kalman filters
  // KalmanFilter_Init(&kalman_x, 0.001, 0.003, 0.03);
  // KalmanFilter_Init(&kalman_y, 0.001, 0.003, 0.03);
  // KalmanFilter_Init(&kalman_z, 0.001, 0.003, 0.03);
  
  // USB initialization cuối cùng
  // MX_USB_DEVICE_Init();
  
  // Delay cuối để đảm bảo USB enumeration hoàn tất
  HAL_Delay(1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"Starting main loop...\n", 23, HAL_MAX_DELAY);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Test counter để kiểm tra loop hoạt động
    static uint32_t loop_counter = 0;
    static uint32_t last_print_time = 0;
    
    // In thông báo mỗi giây
    if(HAL_GetTick() - last_print_time > 1000) {
        last_print_time = HAL_GetTick();
        loop_counter++;
        
        char status_msg[50];
        sprintf(status_msg, "Loop #%lu - Tick: %lu\n", loop_counter, HAL_GetTick());
        HAL_UART_Transmit(&huart1, (uint8_t*)status_msg, strlen(status_msg), HAL_MAX_DELAY);
    }
    
    // Test đọc sensor mỗi 2 giây
    static uint32_t last_sensor_time = 0;
    if(HAL_GetTick() - last_sensor_time > 2000) {
        last_sensor_time = HAL_GetTick();
        
        HAL_UART_Transmit(&huart1, (uint8_t*)"Reading sensor...\n", 18, HAL_MAX_DELAY);
        Read_Sensor_Data();
    }
    
    // Delay ngắn
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
