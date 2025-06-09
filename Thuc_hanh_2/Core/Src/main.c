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
#include "i2c_scan.h"
#include "SH1106.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "tm_stm32f4_mfrc522.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_VALID_RFIDS 10
char valid_rfids[MAX_VALID_RFIDS][20];
int num_valid_rfids = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
struct Time
{
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
  uint8_t weekday;
  uint8_t day;
  uint8_t month;
  uint8_t year;
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char buff[100];
char rx_buffer[30]; // Bộ đệm nhận UART
uint8_t rx_index = 0;
uint8_t is_admin_mode = 0; // Chế độ quản trị để thêm thẻ mới
uint8_t button_state = 0; // Trạng thái nút nhấn trước đó
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SetTime()
{
  struct Time Set_time;
  Set_time.sec = 0;
  Set_time.min = 13;
  Set_time.hour = 14;
  Set_time.day = 17;
  Set_time.month = 4;
  Set_time.year = 25;
  Set_time.weekday = 0;
  HAL_I2C_Mem_Write(&hi2c3, 0xD0, 0, 1, (uint8_t *)&Set_time, 7, 1000);
}

uint8_t BCD_To_Dec(uint8_t bcd)
{
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}


struct Time GetRTCTime()
{
  struct Time Get_time_raw;
  struct Time Get_time;

  HAL_I2C_Mem_Read(&hi2c3, 0xD0, 0, 1, (uint8_t *)&Get_time_raw, 7, 1000);

  Get_time.sec     = BCD_To_Dec(Get_time_raw.sec);
  Get_time.min     = BCD_To_Dec(Get_time_raw.min);
  Get_time.hour    = BCD_To_Dec(Get_time_raw.hour);
  Get_time.day     = BCD_To_Dec(Get_time_raw.day);
  Get_time.month   = BCD_To_Dec(Get_time_raw.month);
  Get_time.year    = BCD_To_Dec(Get_time_raw.year);
  Get_time.weekday = BCD_To_Dec(Get_time_raw.weekday);


  return Get_time;
}


void DisplayTimeOnUART(struct Time time)
{
  sprintf(buff, "%02d:%02d:%02d-%02d-%02d/%02d/%02d \n",
          time.hour,
          time.min,
          time.sec,
          time.weekday,
          time.day,
          time.month,
          time.year);
  HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
}

void DisplayTimeOnSH1106(struct Time time)
{
  SH1106_Clear();

  // Display title
  sprintf(buff, "Real Time Clock");
  SH1106_GotoXY(10, 0);
  SH1106_Puts(buff, &Font_11x18, SH1106_COLOR_WHITE);

  // Display time
  sprintf(buff, "%02d:%02d:%02d", time.hour, time.min, time.sec);
  SH1106_GotoXY(25, 20);
  SH1106_Puts(buff, &Font_11x18, SH1106_COLOR_WHITE);

  // Display date
  sprintf(buff, "%02d/%02d/20%02d", time.day, time.month, time.year);
  SH1106_GotoXY(20, 40);
  SH1106_Puts(buff, &Font_11x18, SH1106_COLOR_WHITE);

  SH1106_UpdateScreen();
}

void InitDisplay()
{
  SH1106_Init();
  SH1106_Clear();
  sprintf(buff, "Starting...");
  SH1106_GotoXY(20, 20);
  SH1106_Puts(buff, &Font_11x18, SH1106_COLOR_WHITE);
  SH1106_UpdateScreen();
  HAL_Delay(1000);
}

int isValidCardId(uint8_t *cardId)
{
  char cardIdString[20];
  sprintf(cardIdString, "%02X %02X %02X %02X %02X", cardId[0], cardId[1], cardId[2], cardId[3], cardId[4]);
  
  // In cardID ra UART để debug
  HAL_UART_Transmit(&huart1, (uint8_t *)"Scanned Card: ", 14, 1000);
  HAL_UART_Transmit(&huart1, (uint8_t *)cardIdString, strlen(cardIdString), 1000);
  HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 1000);
  
  // Kiểm tra xem cardID có nằm trong danh sách hợp lệ không
  for (int i = 0; i < num_valid_rfids; i++)
  {
    if (strcmp(cardIdString, valid_rfids[i]) == 0)
    {
      return 1; // Card hợp lệ
    }
  }
  return 0; // Card không hợp lệ
}

// Thêm thẻ RFID mới vào danh sách
void addNewRfid(const char* rfid) {
  if (num_valid_rfids < MAX_VALID_RFIDS) {
    strcpy(valid_rfids[num_valid_rfids], rfid);
    num_valid_rfids++;
    
    sprintf(buff, "Added new RFID: %s\r\n", rfid);
    HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
    
    sprintf(buff, "Total RFIDs: %d\r\n", num_valid_rfids);
    HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
  } else {
    sprintf(buff, "Error: Maximum number of RFIDs reached\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
  }
}

// Xử lý lệnh từ UART
void processCommand(char* cmd) {
  // Loại bỏ ký tự xuống dòng nếu có
  char* newline = strchr(cmd, '\r');
  if (newline) *newline = 0;
  newline = strchr(cmd, '\n');
  if (newline) *newline = 0;
  
  // Kiểm tra lệnh
  if (strncmp(cmd, "admin", 5) == 0) {
    is_admin_mode = 1;
    sprintf(buff, "Admin mode activated\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
    sprintf(buff, "Enter RFID to add (format: XX XX XX XX XX):\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
  }
  else if (strncmp(cmd, "exit", 4) == 0) {
    is_admin_mode = 0;
    sprintf(buff, "Admin mode deactivated\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
  }
  else if (strncmp(cmd, "list", 4) == 0) {
    sprintf(buff, "Valid RFIDs (%d):\r\n", num_valid_rfids);
    HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
    
    for (int i = 0; i < num_valid_rfids; i++) {
      sprintf(buff, "%d: %s\r\n", i+1, valid_rfids[i]);
      HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
    }
  }
  else if (is_admin_mode && strlen(cmd) >= 14) {
    // Kiểm tra định dạng RFID (XX XX XX XX XX)
    int valid_format = 1;
    if (strlen(cmd) >= 14) {
      for (int i = 0; i < 14; i++) {
        if ((i == 2 || i == 5 || i == 8 || i == 11) && cmd[i] != ' ') {
          valid_format = 0;
          break;
        }
        if ((i != 2 && i != 5 && i != 8 && i != 11) && 
            !((cmd[i] >= '0' && cmd[i] <= '9') || 
              (cmd[i] >= 'A' && cmd[i] <= 'F') ||
              (cmd[i] >= 'a' && cmd[i] <= 'f'))) {
          valid_format = 0;
          break;
        }
      }
    } else {
      valid_format = 0;
    }
    
    if (valid_format) {
      // Chuyển đổi chữ thường thành chữ hoa
      for (int i = 0; i < 14; i++) {
        if (cmd[i] >= 'a' && cmd[i] <= 'f') {
          cmd[i] = cmd[i] - 32; // Chuyển thành chữ hoa
        }
      }
      
      addNewRfid(cmd);
    } else {
      sprintf(buff, "Invalid RFID format. Use: XX XX XX XX XX\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
    }
  }
}

// Khởi tạo một số RFID mặc định
void initDefaultRfids() {
  addNewRfid("43 B7 82 34 42");
  addNewRfid("43 E6 15 35 85");
  addNewRfid("B3 DC 9E 19 E8");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  struct Time currentTime;
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
  MX_I2C3_Init();
  MX_SPI4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  sprintf(buff, "Initializing...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);

  // Khởi tạo danh sách RFID mặc định
  initDefaultRfids();
  
  // Hiển thị thông tin hướng dẫn sử dụng
  sprintf(buff, "\r\nCommands:\r\n- admin: Enter admin mode\r\n- exit: Exit admin mode\r\n- list: List all valid RFIDs\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);

  SetTime(); // Set initial time (can be commented out after first run)

  // Initialize OLED display
  sprintf(buff, "Initializing OLED display...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);

  uint8_t oled_status = SH1106_Init();
  if (oled_status)
  {
    sprintf(buff, "OLED display initialized successfully!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
    SH1106_Clear();
    SH1106_GotoXY(0, 0);
    SH1106_Puts("OLED Test OK", &Font_11x18, SH1106_COLOR_WHITE);
    SH1106_UpdateScreen();
  }
  else
  {
    sprintf(buff, "OLED display initialization FAILED!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
  }

  HAL_Delay(1000);

  // Initialize RFID
  TM_MFRC522_Init();

  // Read RFID version register to check if module is responding
  uint8_t version = TM_MFRC522_ReadRegister(MFRC522_REG_VERSION);
  sprintf(buff, "RFID Version: 0x%02X\r\n", version);
  HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);

  if (version == 0x00 || version == 0xFF)
  {
    sprintf(buff, "RFID module not detected!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);

    if (oled_status)
    {
      SH1106_Clear();
      SH1106_GotoXY(0, 0);
      SH1106_Puts("RFID ERROR!", &Font_11x18, SH1106_COLOR_WHITE);
      SH1106_UpdateScreen();
    }
  }
  else
  {
    sprintf(buff, "RFID module OK, scanning...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);

    if (oled_status)
    {
      SH1106_Clear();
      SH1106_GotoXY(0, 0);
      SH1106_Puts("RFID Ready", &Font_11x18, SH1106_COLOR_WHITE);
      SH1106_UpdateScreen();
    }
  }

  FILE*f = open("log.txt", "w");
  if(f == NULL) {
	  sprintf(buff, "Cannot open file");
	  HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t CardId[5];

    // Kiểm tra nút nhấn PA0
    uint8_t current_button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    
    // Phát hiện cạnh xuống (từ 1 -> 0, tức là nút vừa được nhấn)
    if (button_state == 1 && current_button_state == 0) {
      // Nút được nhấn, chuyển sang chế độ admin
      is_admin_mode = !is_admin_mode; // Toggle chế độ admin
      
      if (is_admin_mode) {
        sprintf(buff, "\r\nAdmin mode activated. Enter RFID to add (format: XX XX XX XX XX):\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
        // Xóa màn hình OLED và hiển thị thông báo ở chế độ admin
        SH1106_Clear();
        SH1106_GotoXY(0, 0);
        SH1106_Puts("ADMIN MODE", &Font_11x18, SH1106_COLOR_WHITE);
        SH1106_GotoXY(0, 25);
        SH1106_Puts("Enter ID via", &Font_7x10, SH1106_COLOR_WHITE);
        SH1106_GotoXY(0, 40);
        SH1106_Puts("UART", &Font_7x10, SH1106_COLOR_WHITE);
        SH1106_UpdateScreen();
      } else {
        sprintf(buff, "\r\nAdmin mode deactivated. Returning to normal operation.\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
        // Xóa màn hình OLED và hiển thị thông báo quay lại chế độ kiểm tra
        SH1106_Clear();
        SH1106_GotoXY(0, 0);
        SH1106_Puts("RFID SCANNER", &Font_11x18, SH1106_COLOR_WHITE);
        SH1106_GotoXY(0, 25);
        SH1106_Puts("Waiting for", &Font_7x10, SH1106_COLOR_WHITE);
        SH1106_GotoXY(0, 40);
        SH1106_Puts("card...", &Font_7x10, SH1106_COLOR_WHITE);
        SH1106_UpdateScreen();
      }
      
      // Thêm delay chống dội nút
      HAL_Delay(200);
    }
    
    // Cập nhật trạng thái nút nhấn
    button_state = current_button_state;

    // Nhận dữ liệu từ UART
    uint8_t rx_data;
    if (HAL_UART_Receive(&huart1, &rx_data, 1, 10) == HAL_OK && is_admin_mode) {
      if (rx_data == '\r' || rx_data == '\n') {
        if (rx_index > 0) {
          rx_buffer[rx_index] = 0; // Null terminate
          processCommand(rx_buffer);
          rx_index = 0; // Reset buffer
        }
      } else {
        if (rx_index < sizeof(rx_buffer) - 1) {
          rx_buffer[rx_index++] = rx_data;
        }
      }
    }

    // Chỉ kiểm tra thẻ RFID khi không ở chế độ admin
    if (!is_admin_mode) {
      // Test RFID communication every few seconds
      static uint32_t lastRfidTest = 0;
      if (HAL_GetTick() - lastRfidTest > 3000)
      {
        lastRfidTest = HAL_GetTick();

        // Test reading a register
        uint8_t status = TM_MFRC522_ReadRegister(MFRC522_REG_STATUS1);
        sprintf(buff, "RFID Status: 0x%02X\r\n", status);
        HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
      }

      HAL_Delay(100);

      if (TM_MFRC522_Check(CardId) == MI_OK)
      {
        sprintf(buff, "Card ID: %02X %02X %02X %02X %02X\r\n", CardId[0], CardId[1], CardId[2], CardId[3], CardId[4]);
        HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);

        // Display card ID on OLED
        SH1106_Clear();

        if (isValidCardId(CardId) == 1)
        {
          sprintf(buff, "%s", "Welcome!");
          SH1106_GotoXY(10, 0);
          SH1106_Puts(buff, &Font_7x10, SH1106_COLOR_WHITE);
          HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
          SH1106_GotoXY(10, 20);
          SH1106_Puts(buff, &Font_7x10, SH1106_COLOR_WHITE);

          struct Time time = GetRTCTime();
          FILE*f = open("log.txt", "a");
          if(f == NULL) {
        	  sprintf(buff, "Cannot open file");
        	  HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
          }
          if (f != NULL)
           {
                 fprintf(f, "CardID: %s - Time: %02d:%02d:%02d %02d/%02d/20%02d\n",
                          CardId,
                          time.hour, time.min, time.sec,
                          time.day, time.month, time.year);
                  fclose(f);
              }
        }
        else
        {
          sprintf(buff, "%s", "Rejected!");
          SH1106_GotoXY(10, 0);
          SH1106_Puts(buff, &Font_7x10, SH1106_COLOR_WHITE);
          HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
        }

        //Display each byte of the card ID on a separate line
        sprintf(buff, "ID[0]: %02X", CardId[0]);
        SH1106_GotoXY(10, 12);
        SH1106_Puts(buff, &Font_7x10, SH1106_COLOR_WHITE);

        sprintf(buff, "ID[1]: %02X", CardId[1]);
        SH1106_GotoXY(10, 22);
        SH1106_Puts(buff, &Font_7x10, SH1106_COLOR_WHITE);

        sprintf(buff, "ID[2]: %02X", CardId[2]);
        SH1106_GotoXY(10, 32);
        SH1106_Puts(buff, &Font_7x10, SH1106_COLOR_WHITE);

        sprintf(buff, "ID[3]: %02X", CardId[3]);
        SH1106_GotoXY(10, 42);
        SH1106_Puts(buff, &Font_7x10, SH1106_COLOR_WHITE);

        sprintf(buff, "ID[4]: %02X", CardId[4]);
        SH1106_GotoXY(10, 52);
        SH1106_Puts(buff, &Font_7x10, SH1106_COLOR_WHITE);

        SH1106_UpdateScreen();
        
        // Thêm delay để không đọc lại thẻ liên tục
        HAL_Delay(1000);
      }
      else if (HAL_GetTick() % 2000 < 1000) // Hiện thông báo mỗi 2 giây
      {
        // Hiển thị thông báo chờ thẻ
        SH1106_Clear();
        SH1106_GotoXY(0, 0);
        SH1106_Puts("RFID SCANNER", &Font_11x18, SH1106_COLOR_WHITE);
        SH1106_GotoXY(0, 25);
        SH1106_Puts("Waiting for", &Font_7x10, SH1106_COLOR_WHITE);
        SH1106_GotoXY(0, 40);
        SH1106_Puts("card...", &Font_7x10, SH1106_COLOR_WHITE);
        SH1106_UpdateScreen();
      }
    }
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF15_EVENTOUT;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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
