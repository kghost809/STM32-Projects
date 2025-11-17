// SLAVE
/***************************************************
* <Assignment 8> / <Slave.c>
*
* <This program enables SPI communication enabling itself
*  As the slave and receiving  data from the designated
*  Master microcontroller.>
* 
* Author: , Kevin Lopez,Philip Hanhurst, Brian Lopez, Edward Atristain
* Date Last Modified: <4/15/25>
*
***************************************************/
#include "stm32f446xx.h"
#include "stm32f4xx.h"
#include <stdio.h>

void GPIO_Init(void);
void SPI1_Slave_Init(void);
void init_PWM_PA5(void);

int main(void) {
	GPIO_Init();
	init_PWM_PA5();
	SPI1_Slave_Init();

	TIM2->CCR1 = 100;

	while (1) {
		if (SPI1->SR & 0x01) {
			uint8_t data = SPI1->DR;

			TIM2->CCR1 = data;
		}
	}
}

void init_PWM_PA5() {
	// Enable GPIOA
	RCC->AHB1ENR |= 1;

	// Set PA5 to use TIM2
	GPIOA->AFR[0] &= ~0x00F00000;
	GPIOA->AFR[0] |=  0x00100000;

	// Set PA5 to alternative function mode
	GPIOA->MODER &= ~0x0C00;
	GPIOA->MODER |=  0x0800;

	// TIM2 initialization
	// Enable TIM2 clock
	RCC->APB1ENR |= 1;

	// Set prescaler to 16000, output clock is now 100Hz
	TIM2->PSC = 1599;
	// 100 ticks per period, duty cycle resolution is 1%
	TIM2->ARR = 99;
	// Reset count
	TIM2->CNT = 0;
	// PWM mode 1
	TIM2->CCMR1 = 0x0060;
	// PWM channel 1
	TIM2->CCER = 1;
	// Duty cycle 0%
	TIM2->CCR1 = 0;
	// Enable!
	TIM2->CR1 = 1;
}

// PA5 = LED, PA2 = USART2_TX
void GPIO_Init(void) {
	RCC->AHB1ENR |= 0x01; // 0x01
	RCC->AHB1ENR |= 0x02; // 0x02

	// PA2 = USART2 TX
	GPIOA->MODER &= ~0x30;
	GPIOA->MODER |= 0x20; // Alternate function
	GPIOA->AFR[0] &= ~0x0F00;
	GPIOA->AFR[0] |= 0x0700; // AF7 = USART2
}

void SPI1_Slave_Init(void) {
	RCC->APB2ENR |= 0x1000; // 0x1000

	// SPI pins: PA4 = NSS, PB3 = SCK, PB4 = MISO, PB5 = MOSI
	GPIOA->MODER &= ~0x0300;
	GPIOA->MODER |=  0x0200; // AF for NSS
	GPIOB->MODER &= ~0x00C0;
	GPIOB->MODER |=  0x0080; // AF for SCK
	GPIOB->MODER &= ~0x0300;
	GPIOB->MODER |=  0x0200; // AF for MISO
	GPIOB->MODER &= ~0x0C00;
	GPIOB->MODER |=  0x0800; // AF for MOSI

	GPIOA->AFR[0] |= 0x00050000; // AF5 for PA4
	GPIOB->AFR[0] |= 0x00005000; // AF5 for PB3
	GPIOB->AFR[0] |= 0x00050000; // AF5 for PB4
	GPIOB->AFR[0] |= 0x00500000; // AF5 for PB5

	SPI1->CR1 = 0;        // Clear settings
	SPI1->CR1 &= ~0x04;   // Slave mode
	SPI1->CR1 &= ~0x02;   // CPOL = 0
	SPI1->CR1 &= ~0x01;   // CPHA = 0
	SPI1->CR1 &= ~0x0200; // Use hardware NSS
	SPI1->CR2 = 0;

	SPI1->CR1 |= SPI_CR1_SPE; // (0x40) Enable SPI
}
