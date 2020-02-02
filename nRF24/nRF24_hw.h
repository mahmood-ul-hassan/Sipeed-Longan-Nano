#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "gd32vf103.h"
#include "gd32vf103_libopt.h"
#include "systick.h"


#define HIGH	1
#define LOW		0

#define SPI_TX_BUF		32
#define SPI_RX_BUF		32

#define SPI_PRESCALAR    16

#define SPI_PORT		GPIOA
#define SPI_SCK         GPIO_PIN_5
#define SPI_MISO		GPIO_PIN_6
#define SPI_MOSI		GPIO_PIN_7

#define RF24_CSN_PORT	GPIOA
#define RF24_CSN_PIN	GPIO_PIN_3

#define RF24_CE_PORT	GPIOA
#define RF24_CE_PIN		GPIO_PIN_2

/* **** Globals **** */

void nRF24_spi_init(void);

void nRF24_ce(uint8_t level);
void nRF24_csn(uint8_t level);

uint8_t nRF24_spi_transfer(uint8_t data, uint32_t timeout);




