/***************************************************
* Final Integrated Project / main.c
*
* Master SPI controller with LCD, PID control, Bluetooth input, and ADC temperature sensing
*
* Author: Kevin Lopez, Philip Hanhurst, Brian Lopez, Edward Atristain
* Date Last Modified: 5/9/2025
*
***************************************************/

#include "stm32f4xx.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "common.h"
#include "lcd.h"
#include "usart.h"
#include "spi.h"

#define TOLERANCE 2
#define PERIOD 5000

volatile int adc_result = 0;
volatile int tim3_flag = 0;

int requested_temp = 60;
float current_temp[3] = {72.0f, 72.0f, 72.0f};

const uint8_t TEMP_CHAR[] = {
  0b00100,
  0b01010,
  0b01110,
  0b01010,
  0b01010,
  0b10101,
  0b10101,
  0b01110
};

double kp = 4.0, ki = 0.2;
//double kp= 4.59, ki=0.54;
// double kp2=8.3, ki=0.72;
// double kp= 5.55, ki = 0.64;
// double kp2= 8.77, ki= 0.8;
double kp2= 8.0, ki2 =0.6;
double error = 0, prev_error = 0, integral = 0;

void deselect_all_slaves(void) {
  GPIOA->BSRR = 1 << 4;
  GPIOA->BSRR |= 1 << 1;
}

void select_slave_A4(void) {
  deselect_all_slaves();
  GPIOA->BSRR = 1 << 20;
}

void select_slave_A1(void) {
  deselect_all_slaves();
  GPIOA->BSRR |= 1 << 17;
}

void update_display(void) {
  LCD_clear();
  char line0[16] = {0};
  char line1[16] = {0};
  float avg = (current_temp[0] + current_temp[1] + current_temp[2]) / 3.0f;

  sprintf(line0, "%2.1fF %2.1fF %2.1fF", current_temp[0], current_temp[1], current_temp[2]);
  sprintf(line1, "A:%2.1fF S:%3dF", avg, requested_temp);

  LCD_write_string(line0, Line0);
  LCD_write_string(line1, Line1);

  char msg[128];
  sprintf(msg, "Avg: %.1fF | Setpoint: %d\n", avg, requested_temp);
  USART_write_string(USART2, msg);
}

static uint8_t SPI1_xfer(uint8_t out) {
  while (!(SPI1->SR & SPI_SR_TXE));
  *(volatile uint8_t *)&SPI1->DR = out;
  while (!(SPI1->SR & SPI_SR_RXNE));
  return *(volatile uint8_t *)&SPI1->DR;
}

void write_temps(void) {
  select_slave_A4();
      delayMs(1);
  SPI1_xfer(0x10);
  SPI1_xfer(requested_temp);
  delayMs(1);
  GPIOA->BSRR = (1 << 4);

  select_slave_A1();
      delayMs(1);
  SPI1_xfer(0x10);
  SPI1_xfer(requested_temp);
  delayMs(1);
  GPIOA->BSRR = ( 1 << 1);
      update_display();
}
void read_temps(void){
                                    select_slave_A4();
                                    delayMs(1);
            SPI1_xfer(0x1F);
                                    delayMs(1);
            uint8_t val = SPI1_xfer(0x00);
                        
            delayMs(1);
            deselect_all_slaves();
            current_temp[1]=val;
            val=0;

                                    select_slave_A1();
                                          delayMs(1);
            SPI1_xfer(0x1F);          // Send command 0x10 to Slave
                                    delayMs(1);
            uint8_t val2 = SPI1_xfer(0x00);
            delayMs(1);
            // Optional: send or receive more bytes here...
            GPIOA->BSRR = (1U << 1);   // Set PA1 HIGH (set bit 1)
            current_temp[2]=val2;
            val2=0;
}



void LCD_set_custom_char(int idx, uint8_t* c) {
  LCD_command(0x40 | (idx << 3));
  for (int i = 0; i < 8; i++) {
    LCD_data(c[i]);
  }
}

void init_adc(void) {
  RCC->AHB1ENR |=  4;               /* enable GPIOC clock */
  GPIOC->MODER |=  3;             /* PC0 analog */
      
      RCC->APB2ENR |= 0x00000200;     /* enable ADC2 clock */
      
            ADC2->CR2=0;
            ADC2->SQR3=10;                                                    /* conversion sequence starts at ch 10 */
            ADC2->SQR1=0;                                                           /* conversion sequence starts at ch 10 */
            ADC2->CR2|=1;                             
            ADC2->SMPR1=3;


 

  RCC->APB2ENR |= (1 << 8); // Enable ADC2 clock
  ADC2->CR2 = 0;
  ADC2->SQR3 = 10; // Channel 10 (PC0)
  ADC2->SQR1 = 0;
  ADC2->SMPR1 = 3;
  ADC2->CR2 |= 1; // Enable ADC2
}

void init_timer(void) {
  RCC->APB1ENR |= (1 << 0); // TIM2
  TIM2->PSC = 160 - 1;
  TIM2->ARR = 1000 - 1;
  TIM2->CCMR1 = 0x0060;
  TIM2->CCER = 1;
  TIM2->CCR1 = 1;
  TIM2->CR1 = 1;

  RCC->APB1ENR |= (1 << 1); // TIM3
  TIM3->PSC = 16000 - 1;
  TIM3->ARR = PERIOD - 1;
  TIM3->DIER |= 1;
  TIM3->CR1 = 1;
      
      RCC-> APB1ENR |=8;      //Timer 5
                  TIM5->PSC = 160-1;            /*divided by 1600*/
                  TIM5->ARR=1000-1;       /*1000 ticks a second*/
                  TIM5->CNT=0;
                  TIM5->CCMR1 = 0x0060;   /* PWM mode 1*/
                  TIM5->CCER =1;                            /* PWM channel 1*/
                  TIM5->CCR1=1-1; /*set 0% duty cycle*/
                  TIM5-> CR1= 1 ;   /* enable timer */
}

void TIM3_IRQHandler(void) {
  TIM3->SR &= ~TIM_SR_UIF;
  ADC2->CR2 |= 0x40000000;
  while (!(ADC2->SR & 2));
  adc_result = ADC2->DR;
  tim3_flag = 1;
}
/* USART2 Interrupt Handler */
void USART2_IRQHandler(void) {
  static char input_buffer[32];
  static int buffer_index = 0;

  if (USART2->SR & 0x0020) {
    char c = USART2->DR;

    if (c == '\n' || c == '\r') {
      input_buffer[buffer_index] = '\0';
      if (strncmp(input_buffer, "temp", 4) == 0) {
        int val = atoi(&input_buffer[4]);
        if (val >= 1 && val <= 100) {
          requested_temp = val;
          write_temps();
        } else {
          USART_write_string(USART2, "Invalid temp range (1-100)\n");
        }
      }
      buffer_index = 0;
    } else if (buffer_index < sizeof(input_buffer) - 1) {
      input_buffer[buffer_index++] = c;
    }
  }
}
void SPI1_init(void) {
    RCC->AHB1ENR |= 3;              /* enable GPIOB & GPIOA clock */
    RCC->APB2ENR |= 0x1000;         /* enable SPI1 clock */

    // Configure PB3, PB4, PB5 for SPI1
    GPIOB->MODER &= ~0x00000FC0;
    GPIOB->MODER |=  0x00000A80;
    GPIOB->AFR[0] &= ~0x00FFF000;
    GPIOB->AFR[0] |=  0x00555000;

    // Configure PA1 and PA4 as outputs for SS
    //GPIOB->MODER &= ~0x000C0000;
   // GPIOB->MODER |=  0x00040000;
    GPIOA->MODER &= ~0x0000030C;
    GPIOA->MODER |=  0x00000104;

    deselect_all_slaves();

    SPI1->CR1 = 0x31C;  // Baud rate, master, CPOL/CPHA = 0, 8-bit data
    SPI1->CR2 = 0;
    SPI1->CR1 |= 0x40;  // Enable SPI1
}
int main(void) {
            __disable_irq();                /* global disable IRQs */
            USART2_init();
            RCC->AHB1ENR |=  1;                 /* enable GPIOA clock */
            GPIOA->AFR[0] |= 0x00100000;        /*PA5 pin for tim2 */
    GPIOA->MODER &= ~0x00000C00;    /* clear pin mode */
    GPIOA->MODER |=  0x00000800;    /* set pin to output mode */
            // Set PA6 to Alternate Function mode
GPIOA->MODER &= ~(3U << (6 * 2));    // Clear mode bits for PA6
GPIOA->MODER |=  (2U << (6 * 2));    // Set to Alternate Function

// Set AF2 (TIM5_CH1) on PA6
GPIOA->AFR[0] &= ~(0xF << (6 * 4));  // Clear AF bits for PA6
GPIOA->AFR[0] |=  (0x2 << (6 * 4));  // Set AF2 (TIM5)
            LCD_init();
            LCD_set_custom_char(0, (uint8_t*)TEMP_CHAR);
            init_adc();
            init_timer();
      
  SPI1_init();
  deselect_all_slaves();
  update_display();

  NVIC_EnableIRQ(TIM3_IRQn);
  NVIC_EnableIRQ(USART2_IRQn);
      
      __enable_irq();                /* global enable IRQs */

  while (1) {
  if (tim3_flag) {
    tim3_flag = 0;

    float analog = (adc_result * 3.3f) / 4095.0f;
            current_temp[0] = analog * 100 * 1.8f + 32;

            error = requested_temp - current_temp[0];
            integral += error * (PERIOD / 1000.0f);
            float output = kp * error + ki * integral;

            // Flip the sign for cooling if needed
            float pwm_val = (output < 0) ? -output : output;

            // Cap output
            if (pwm_val > 1000) pwm_val = 1000;

            if (current_temp[0] < (requested_temp - TOLERANCE)) {
                        TIM5->CCR1 = (int)(pwm_val * 100); // Heating
                        TIM2->CCR1 = 0;
            } else if (current_temp[0] > (requested_temp + TOLERANCE)) {
                        TIM2->CCR1 = (int)(pwm_val * 100); // Cooling
                        TIM5->CCR1 = 0;
            } else {
                        TIM2->CCR1 = 0;
                        TIM5->CCR1 = 0;
            }


                        // Optional: Debug print
                        char c[64];
                        sprintf(c, "Temp: %.2f, Output: %.1f, H:%d, C:%d\r\n", current_temp[0], output, TIM5->CCR1, TIM2->CCR1);
                        USART_write_string(USART2, c);

                        read_temps();
                        update_display();
                  }
            }
                  }