    /*
* This file is part of the libopencm3 project.
*
* Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
* Copyright (C) 2015 Jack Ziesing <jziesing@gmail.com>
*
* This library is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this library. If not, see <http://www.gnu.org/licenses/>.
*/
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include "systime.h"

uint32_t encode_byte(uint8_t b);
void write_byte(uint32_t spi, uint8_t b);
void write_rgb(uint32_t spi, uint8_t r, uint8_t g, uint8_t b);

uint32_t encode_byte(uint8_t b) {
	uint32_t result = 0;
	for (int i=0; i<8; i++) {
		result <<= 3;
		if (b & 0x01) {
			result |= 0b110;
		} else {
			result |= 0b100;
		}
	}
	return result;
}

void write_byte(uint32_t spi, uint8_t b) {
	uint32_t data = encode_byte(b);
	spi_send(spi, (data>>16) & 0xFF);
	spi_send(spi, (data>> 8) & 0xFF);
	spi_send(spi, (data>> 0) & 0xFF);
}

void write_rgb(uint32_t spi, uint8_t r, uint8_t g, uint8_t b) {
	for (int i=0; i<20; i++) { spi_send(spi, 0); }
	write_byte(spi, g);
	write_byte(spi, r);
	write_byte(spi, b);
	spi_send(spi, 0);
}

int main(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
    systime_setup(168000);

	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_SPI1);

	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO3|GPIO5);
	gpio_set_af(GPIOB, GPIO_AF5, GPIO3|GPIO5);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO3);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO5);

	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_32, 0, 0, 0, 0);
	spi_set_dff_8bit(SPI1);
	spi_enable(SPI1);

	while (1) {
		write_rgb(SPI1, 255,   0,   0);
		delay_ms(500);
		write_rgb(SPI1, 255, 255,   0);
		delay_ms(500);
		write_rgb(SPI1, 255, 255, 255);
		delay_ms(500);
		write_rgb(SPI1,   0, 255, 255);
		delay_ms(500);
		write_rgb(SPI1,   0, 255,   0);
		delay_ms(500);
		write_rgb(SPI1,   0,   0, 255);
		delay_ms(500);
    }

    return 0;
}
