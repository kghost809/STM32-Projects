#include "spi.h"
#include "common.h"
#include "stm32f4xx.h"
#include "usart.h"

void set_afr(GPIO_TypeDef* gpio, uint32_t pin, int value) {
	if (pin >= 16) {
		gpio->AFR[1] = (value << ((pin - 16) * 4));
	} else {
		gpio->AFR[0] = (value << (pin * 4));
	}
}

void SPI_init_slave(SPI_TypeDef* spi, SPIInitInfo config) {
	RCC->APB2ENR |= 0x1000; // 0x1000

	// SPI pins: PA4 = NSS, PB3 = SCK, PB4 = MISO, PB5 = MOSI
	config.nss_gpio->MODER &= ~(0x3 << (config.nss * 2));
	config.nss_gpio->MODER |=  (0x2 << (config.nss * 2)); // AF for NSS
	config.sck_gpio->MODER &= ~(0x3 << (config.sck * 2));
	config.sck_gpio->MODER |=  (0x2 << (config.sck * 2)); // AF for SCK
	config.miso_gpio->MODER &= ~(0x3 << (config.miso * 2));
	config.miso_gpio->MODER |=  (0x2 << (config.miso * 2)); // AF for MISO
	config.mosi_gpio->MODER &= ~(0x3 << (config.mosi * 2));
	config.mosi_gpio->MODER |=  (0x2 << (config.mosi * 2)); // AF for MOSI

	set_afr(config.nss_gpio, config.nss, 0x5); // AF5 for PA4
	set_afr(config.sck_gpio, config.sck, 0x5); // AF5 for PB3
	set_afr(config.miso_gpio, config.miso, 0x5); // AF5 for PB4
	set_afr(config.mosi_gpio, config.mosi, 0x5); // AF5 for PB5

	spi->CR1 = 0;        // Clear settings
	spi->CR1 &= ~0x04;   // Slave mode
	spi->CR1 &= ~0x02;   // CPOL = 0
	spi->CR1 &= ~0x01;   // CPHA = 0
	spi->CR1 &= ~0x0200; // Use hardware NSS
	spi->CR2 = 0;

	spi->CR1 |= SPI_CR1_SPE; // (0x40) Enable SPI
}

/* This function enables slave select, writes one byte to SPI1,
	 wait for transmission complete and deassert slave select. */
void SPI_write(SPI_TypeDef* spi, unsigned char data) {
	while (!(spi->SR & 2)) {}      /* wait until Transfer buffer Empty */
	GPIOA->BSRR = 0x00100000;       /* assert slave select */
	spi->DR = data;                /* write data */
	while (spi->SR & 0x80) {}      /* wait for transmission done */
	GPIOA->BSRR = 0x00000010;       /* deassert slave select */
}

/* This function enables slave select, writes one byte to SPI1,
	 wait for transmission complete and deassert slave select. */
void SPI_respond(SPI_TypeDef* spi, unsigned char data) {
	while (!(spi->SR & 2)) {}      /* wait until Transfer buffer Empty */
	// GPIOA->BSRR = 0x00100000;       /* assert slave select */
	spi->DR = data;                /* write data */
	while (spi->SR & 0x80) {}      /* wait for transmission done */
	// GPIOA->BSRR = 0x00000010;       /* deassert slave select */
}

bool SPI_has_incoming(SPI_TypeDef* spi) {
	return (spi->SR & SPI_SR_RXNE) != 0;
}

void SPI_await_incoming(SPI_TypeDef* spi) {
	USART_write_string(USART2, "Awaiting...\n");
	while (!SPI_has_incoming(spi)) {}
	USART_write_string(USART2, "Done waiting!\n");
}

void SPI_await_outgoing(SPI_TypeDef* spi) {
	if ((spi->SR & SPI_SR_TXE)) {
		USART_write_string(USART2, "Nothing to send...");
		return;
	}
	USART_write_string(USART2, "Awaiting transmission...\n");
	while (!(spi->SR & SPI_SR_TXE)) {}
	USART_write_string(USART2, "Done waiting!\n");
}