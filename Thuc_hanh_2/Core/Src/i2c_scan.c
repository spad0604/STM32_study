#include "i2c_scan.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"

char* I2C_Scan(I2C_HandleTypeDef *hi2c)
{
    char uart_buffer[100];
    sprintf(uart_buffer, "Start I2C Scan...\r\n");

    HAL_StatusTypeDef result;
    uint8_t i;
    uint8_t count = 0;

    for (i = 1; i < 128; i++)
    {
        result = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(i << 1), 1, 10);
        if (result == HAL_OK)
        {
            sprintf(uart_buffer, "Found device at address 0x%X\r\n", i);
            count++;
        }
    }

    if (count == 0)
    {
        sprintf(uart_buffer, "No devices found on I2C bus.\r\n");
    }
    else
    {
        sprintf(uart_buffer, "Found %d devices on I2C bus.\r\n", count);
    }
    return uart_buffer;
}
