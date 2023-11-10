/*
 * printf.c
 *
 *  Created on: Nov 9, 2023
 *      Author: lovro.mileusnic
 */


#include "printf.h"
#include "usart.h"


int usart_printf(const char* msg)
{
	int i = 0;

	do
	{
		USART1_SendChar(msg[i]);
	} while (msg[i++] != '\0');

	return i;
}
