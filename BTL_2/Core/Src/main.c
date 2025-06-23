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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "stm32f4xx_hal.h"  // Bổ sung để tránh lỗi undefined
#include "kalman.h"         // Cho Kalman filtering
#include "usbd_hid.h"       // Cho USB HID mouse
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// MPU6050 I2C Address và Register defines
#define MPU6050_ADDR 0xD0
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_XOUT_H  0x43

// Mouse control parameters - TĂNG SPEED VÀ ĐỘ NHẠY
#define MOUSE_SENSITIVITY_X  5     // Độ nhạy X axis (giảm = nhạy hơn) 
#define MOUSE_SENSITIVITY_Y  5     // Độ nhạy Y axis (giảm = nhạy hơn)
#define GYRO_DEADZONE        3     // Vùng chết để tránh drift (giảm = nhạy hơn)
#define MAX_MOUSE_MOVE       75    // Giới hạn movement tối đa (tăng = nhanh hơn)
#define CALIBRATION_SAMPLES  200   // Số mẫu cho calibration

#define LSB_ACC              16384.0f   // ±2g → 16384 LSB/g
#define LSB_GYRO             131.0f     // ±250 dps → 131 LSB/(°/s)
#define RAD_TO_DEG 57.29577951308232


typedef struct {
    uint8_t buttons;
    int8_t dx;
    int8_t dy;
    uint8_t wheel;
} mouseHID;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// USB HID Device Handle (declared here để tránh undefined reference)
extern USBD_HandleTypeDef hUsbDeviceHS;

// Kalman filters cho gyroscope smoothing
KalmanFilter_t kalman_gyro_x, kalman_gyro_y, kalman_gyro_z;

// Raw gyroscope data
uint8_t gyro_data[6];
int16_t raw_accel_x, raw_accel_y, raw_accel_z;
float accel_x, accel_y, accel_z;

int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
float gyro_x, gyro_y, gyro_z;

float angleX, angleY;
uint32_t timer = 0;


// Mouse control variables
uint8_t mouse_enabled = 1;           // Enable/disable mouse
float gyro_offset_x = 0.0f;         // Calibration offsets  
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;

// Debug and timing
uint32_t debug_counter = 0;
uint32_t mouse_packets_sent = 0;
uint8_t usb_ready = 0;               // USB connection status
uint32_t last_hid_time = 0;          // Timing cho HID reports
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void Gyro_Calibrate(void);
void Process_Gyro_Mouse(void);
void Send_Mouse_HID(int8_t dx, int8_t dy, uint8_t buttons);
void Debug_Print_Status(void);
void Test_Mouse_Movement(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Khởi tạo MPU6050 gyroscope
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
    
    HAL_Delay(100);
}

/**
  * @brief  Calibrate gyroscope để tìm offset
  * @retval None
  */
void Gyro_Calibrate(void)
{
    char buffer[100];
    float sum_x = 0, sum_y = 0, sum_z = 0;
    
    sprintf(buffer, "🔧 Calibrating gyroscope... Keep still!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    for(int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 1, gyro_data, 6, HAL_MAX_DELAY) == HAL_OK)
        {
            int16_t raw_x = (int16_t)(gyro_data[0] << 8 | gyro_data[1]);
            int16_t raw_y = (int16_t)(gyro_data[2] << 8 | gyro_data[3]);
            int16_t raw_z = (int16_t)(gyro_data[4] << 8 | gyro_data[5]);
            
            // ±2000°/s range, scale factor is 16.4 LSB/°/s
            sum_x += (float)raw_x / 16.4f;
            sum_y += (float)raw_y / 16.4f;
            sum_z += (float)raw_z / 16.4f;
        }
        HAL_Delay(5);
    }
    
    gyro_offset_x = sum_x / CALIBRATION_SAMPLES;
    gyro_offset_y = sum_y / CALIBRATION_SAMPLES;
    gyro_offset_z = sum_z / CALIBRATION_SAMPLES;
    
    sprintf(buffer, "✅ Gyro calibrated! Offsets: X=%.1f Y=%.1f Z=%.1f\r\n", 
            gyro_offset_x, gyro_offset_y, gyro_offset_z);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/**
  * @brief  Gửi USB HID mouse report với rate limiting
  * @param  dx: Delta X (-127 to 127)
  * @param  dy: Delta Y (-127 to 127) 
  * @param  buttons: Button state (bit 0: left, bit 1: right)
  * @retval None
  */
void Send_Mouse_HID(int8_t dx, int8_t dy, uint8_t buttons)
{
    static uint32_t busy_count = 0;
    
    // Kiểm tra USB đã sẵn sàng chưa
    if(!usb_ready) {
        return; // Không spam debug message
    }
    
    // Rate limiting: gửi mỗi 8ms (125Hz) - tần số ổn định cho USB HS
    uint32_t current_time = HAL_GetTick();
    if(current_time - last_hid_time < 8) {
        return; // Quá sớm, skip report này
    }
    last_hid_time = current_time;
    mouseHID mousehid = {0};
    
    // Tạo HID report theo standard mouse format
    mousehid.buttons = buttons;
    mousehid.dx = dx;
    mousehid.dy = dy;
    mousehid.wheel = 0;
    
    USBD_StatusTypeDef result = USBD_HID_SendReport(&hUsbDeviceHS, (uint8_t*)&mousehid, sizeof(mousehid));
    
    if(result == USBD_OK) {
        mouse_packets_sent++;
        busy_count = 0; // Reset busy counter khi thành công
        
        // Debug thành công (hiện ít thôi để không spam)
        if(mouse_packets_sent % 200 == 1) {
            char debug_msg[80];
            sprintf(debug_msg, "✅ USB OK: #%lu (dx=%d, dy=%d)\r\n", 
                    mouse_packets_sent, dx, dy);
            HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
        }
    }
    else if(result == 3) { // USBD_BUSY
        busy_count++;
        // Tự động điều chỉnh tần số gửi khi busy
        if(busy_count >= 20) {
            last_hid_time += 10; // Tăng delay để giảm tần số
            if(busy_count >= 50) {
                HAL_UART_Transmit(&huart1, (uint8_t*)"⚠️  USB busy (auto-adjusting rate)\r\n", 37, HAL_MAX_DELAY);
                busy_count = 0; // Reset để không spam
            }
        }
    }
    else {
        // Lỗi khác
        if(debug_counter % 100 == 0) { // Hiện ít thôi
            char debug_msg[60];
            sprintf(debug_msg, "❌ USB Error: %d\r\n", result);
            HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
        }
    }
}

/**
  * @brief  Xử lý gyroscope data và điều khiển chuột
  * @retval None
  */
void Process_Gyro_Mouse(void)
{
    if(!mouse_enabled) return;
    
    // Đọc dữ liệu gyroscope từ MPU6050
    if(HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 1, gyro_data, 6, HAL_MAX_DELAY) == HAL_OK)
    {
        raw_accel_x = (int16_t)(gyro_data[0] << 8 | gyro_data[1]);
        raw_accel_y = (int16_t)(gyro_data[2] << 8 | gyro_data[3]);
        raw_accel_z = (int16_t)(gyro_data[4] << 8 | gyro_data[5]);
        
        // Chuyển đổi sang °/s và trừ offset
        float accel_x_raw = ((float)raw_accel_x / 16.4f) - gyro_offset_x;
        float accel_y_raw = ((float)raw_accel_y / 16.4f) - gyro_offset_y;
        float accel_z_raw = ((float)raw_accel_z / 16.4f) - gyro_offset_z;
        
        raw_gyro_x = (int16_t)(gyro_data[8] << 8 | gyro_data[9]);
        raw_gyro_y = (int16_t)(gyro_data[10] << 8 | gyro_data[11]);
        raw_gyro_z = (int16_t)(gyro_data[12] << 8 | gyro_data[13]);

        float gyro_x_raw = ((float)raw_gyro_x / LSB_GYRO);
        float gyro_y_raw = ((float)raw_gyro_y / LSB_GYRO);
        float gyro_z_raw = ((float)raw_gyro_z / LSB_GYRO);


        // Tính thời gian dt giữa hai lần đo
        double dt = (double)(HAL_GetTick() - timer) / 1000.0;
        timer = HAL_GetTick();

        // Tính góc roll từ gia tốc (gần đúng)
        double roll_sqrt = sqrt(raw_accel_x * raw_accel_x + raw_accel_z * raw_accel_z);
        double roll = (roll_sqrt != 0.0) ? atan(raw_accel_y / roll_sqrt) * RAD_TO_DEG : 0.0;

        // Tính góc pitch từ gia tốc bằng atan2
        double pitch = atan2(-raw_accel_x, raw_accel_z) * RAD_TO_DEG;

        // Bảo vệ bộ lọc Kalman khỏi nhảy đột ngột khi pitch vượt ±90°
        if ((pitch < -90 && angleY > 90) || (pitch > 90 && angleY < -90)) {
        	kalman_gyro_y.angle = pitch;
            angleY = pitch;
        } else {
            angleY = KalmanFilter_Update(&kalman_gyro_y, pitch,gyro_y_raw, dt);
        }

        // Nếu pitch vượt ±90°, đảo chiều trục X
        if (fabs(angleY) > 90)
        	 gyro_x_raw = - gyro_x_raw;

        // Cập nhật roll bằng Kalman filter
        angleX = KalmanFilter_Update(&kalman_gyro_x, roll,gyro_x_raw, dt);


        // Calculate mouse movement
        int8_t mouse_dx = 0, mouse_dy = 0;
        
        // Apply deadzone và calculate movement
        if(abs((int)angleX) > GYRO_DEADZONE)
        {
            mouse_dx = (int8_t)(-angleX / MOUSE_SENSITIVITY_X); // Invert X for natural movement
        }
        
        if(abs((int)angleY) > GYRO_DEADZONE)
        {
            mouse_dy = (int8_t)(angleY / MOUSE_SENSITIVITY_Y);
        }
        
        // Limit movement
        if(mouse_dx > MAX_MOUSE_MOVE) mouse_dx = MAX_MOUSE_MOVE;
        if(mouse_dx < -MAX_MOUSE_MOVE) mouse_dx = -MAX_MOUSE_MOVE;
        if(mouse_dy > MAX_MOUSE_MOVE) mouse_dy = MAX_MOUSE_MOVE;
        if(mouse_dy < -MAX_MOUSE_MOVE) mouse_dy = -MAX_MOUSE_MOVE;
        
        // Đọc trạng thái buttons (nếu có GPIO setup)
        uint8_t buttons = 0;
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) buttons |= 0x01; // Left click (PA2)
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) buttons |= 0x02; // Right click (PA3)
        
        // Gửi mouse movement qua USB HID nếu có movement hoặc button press
        if(mouse_dx != 0 || mouse_dy != 0 || buttons != 0)
        {
            Send_Mouse_HID(mouse_dx, mouse_dy, buttons);
            
            // Debug output cho movement (hiện mọi movement để debug)
            if(abs(mouse_dx) >= 1 || abs(mouse_dy) >= 1 || buttons != 0) {
                char debug_buffer[120];
                sprintf(debug_buffer, "🖱️  Move: dx=%d dy=%d | Gyro: gx=%.1f gy=%.1f | Btn=%d\r\n", 
                        mouse_dx, mouse_dy, angleX, angleY, buttons);
                HAL_UART_Transmit(&huart1, (uint8_t*)debug_buffer, strlen(debug_buffer), HAL_MAX_DELAY);
            }
        }
    }
}

/**
  * @brief  In debug status mỗi 5 giây
  * @retval None
  */
void Debug_Print_Status(void)
{
    char status_buffer[150];
    debug_counter++;
    
    sprintf(status_buffer, "📊 Status #%lu: Mouse %s | Packets: %lu | Offsets: X=%.1f Y=%.1f Z=%.1f\r\n", 
            debug_counter, 
            mouse_enabled ? "ON" : "OFF",
            mouse_packets_sent,
            gyro_offset_x, gyro_offset_y, gyro_offset_z);
    HAL_UART_Transmit(&huart1, (uint8_t*)status_buffer, strlen(status_buffer), HAL_MAX_DELAY);
}

/**
  * @brief  Test mouse movement để verify USB HID
  * @retval None
  */
void Test_Mouse_Movement(void)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)"🧪 Testing USB HID mouse...\r\n", 30, HAL_MAX_DELAY);
    
    // Test với timing tốt hơn cho USB HID
    HAL_Delay(100); // Wait trước khi test
    Send_Mouse_HID(5, 0, 0);   // Right (nhỏ hơn)
    HAL_Delay(50);
    Send_Mouse_HID(0, 5, 0);   // Down
    HAL_Delay(50);
    Send_Mouse_HID(-5, 0, 0);  // Left
    HAL_Delay(50);
    Send_Mouse_HID(0, -5, 0);  // Up
    HAL_Delay(50);
    
    HAL_UART_Transmit(&huart1, (uint8_t*)"✅ Mouse test completed!\r\n", 27, HAL_MAX_DELAY);
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
  /* USER CODE BEGIN 2 */
  
  // Khởi động thông báo
  HAL_UART_Transmit(&huart1, (uint8_t*)"🚀 STM32 Gyroscope Mouse Started\r\n", 35, HAL_MAX_DELAY);
  HAL_Delay(1000);
  
  // Khởi tạo MPU6050
  HAL_UART_Transmit(&huart1, (uint8_t*)"🔧 Initializing MPU6050...\r\n", 29, HAL_MAX_DELAY);
  MPU6050_Init();
  HAL_UART_Transmit(&huart1, (uint8_t*)"✅ MPU6050 initialized!\r\n", 26, HAL_MAX_DELAY);
  HAL_Delay(1000);
  
  // Khởi tạo Kalman filters
  HAL_UART_Transmit(&huart1, (uint8_t*)"🔧 Initializing Kalman filters...\r\n", 36, HAL_MAX_DELAY);
  KalmanFilter_Init(&kalman_gyro_x, 0.001, 0.003, 0.03);
  KalmanFilter_Init(&kalman_gyro_y, 0.001, 0.003, 0.03);
  KalmanFilter_Init(&kalman_gyro_z, 0.001, 0.003, 0.03);
  HAL_UART_Transmit(&huart1, (uint8_t*)"✅ Kalman filters ready!\r\n", 27, HAL_MAX_DELAY);
  
  // Gyroscope calibration
  Gyro_Calibrate();
  
  // Final setup
  HAL_Delay(1000);
  HAL_UART_Transmit(&huart1, (uint8_t*)"🖱️  Gyroscope Mouse READY! Tilt to move cursor.\r\n", 51, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart1, (uint8_t*)"📌 PA2=Left Click, PA3=Right Click\r\n", 37, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart1, (uint8_t*)"═══════════════════════════════════\r\n", 37, HAL_MAX_DELAY);
  
  // Wait for USB HS enumeration (important cho USB HS)
  HAL_UART_Transmit(&huart1, (uint8_t*)"⏳ Waiting for USB HS enumeration...\r\n", 39, HAL_MAX_DELAY);
  HAL_Delay(3000);  // USB HS cần thời gian dài hơn để enumerate
  
  // Enable USB mouse function
  usb_ready = 1;
  HAL_UART_Transmit(&huart1, (uint8_t*)"🔌 USB HID Mouse enabled!\r\n", 28, HAL_MAX_DELAY);
  
  // Test USB HID mouse movement
  Test_Mouse_Movement();
  
  // Hiện thông tin debug đầu tiên
  Debug_Print_Status();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // Đọc trạng thái PA2 và PA3 và in ra UART
    GPIO_PinState pa2_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
    GPIO_PinState pa3_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);

    char pa_status_msg[64];
    sprintf(pa_status_msg, "PA2: %s | PA3: %s\r\n", 
            (pa2_state == GPIO_PIN_SET) ? "HIGH" : "LOW",
            (pa3_state == GPIO_PIN_SET) ? "HIGH" : "LOW");
    HAL_UART_Transmit(&huart1, (uint8_t*)pa_status_msg, strlen(pa_status_msg), HAL_MAX_DELAY);

    Process_Gyro_Mouse();

    //Debug status mỗi 5 giây
   static uint32_t last_debug_time = 0;
   if(HAL_GetTick() - last_debug_time > 5000) {
       last_debug_time = HAL_GetTick();
       Debug_Print_Status();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
