/***************************************************
* Lab_ / usart2.h
*
* Header containing definitions of usart functions
* for USART2
*
*
* Author: Philip Hanhurst
* Date Last Modified: 2/21/2025
*
***************************************************/

// Only import this file once
#pragma once
// #include "stm32f446xx.h"
#include "stm32f4xx.h"
#include "common.h"
#include <stdint.h>

typedef USART_TypeDef* USART;

void USART2_init();
void USART3_init(bool rx, bool tx);
void USART_write(USART u, uint8_t c);
char USART_read(USART u);

bool USART_has_data(USART u);
void USART_wait_for_data(USART u);

void USART_write_string(USART u, char* str);
bool USART_read_string(USART u, char* buffer, int bufferlen);