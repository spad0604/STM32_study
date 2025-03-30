/*
 * irda.c
 *
 *  Created on: Nov 1, 2024
 *      Author: trung
 */
#include "irda.h"
#include "stm32f4xx_hal.h"

extern int tim6_count;

int bit_time[33];
int bit_count;
int command_ok;
unsigned char irda_cmd[4];

void IrGetBitTime()
{
	  if (tim6_count > 100)
	  {
		  bit_count = 0;
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
	  }

	  bit_time[bit_count] = tim6_count;
	  tim6_count = 0;
	  bit_count++;

	  if (bit_count == 33)
	  {
		  bit_count = 0;
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);
		  command_ok = 1;
	  }
}

void IrDecode()
{
  unsigned char code;
  int bit, key, index = 0;
  for (key = 0; key < 4; key++)
  {
	  for (bit = 0; bit < 8; bit++)
	  {
		  code >>= 1;
		  index++;
		  if (bit_time[index] > 20)
			  code |= 0x80;
	  }
	  irda_cmd[key] = code;
  }
}
