#include "nRF24_hw.h"

/* ************************************************************************** */
void nRF24_csn(uint8_t level){
	(level) ? gpio_bit_set(RF24_CSN_PORT, RF24_CSN_PIN) : gpio_bit_reset(RF24_CSN_PORT, RF24_CSN_PIN);
}

/* ************************************************************************** */
void nRF24_ce(uint8_t level){
	(level) ? gpio_bit_set(RF24_CE_PORT, RF24_CE_PIN) : gpio_bit_reset(RF24_CE_PORT, RF24_CE_PIN);
}

/* ************************************************************************** */
void nRF24_spi_init(void){
	rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_SPI0);

    spi_parameter_struct spi_init_struct;
    /* deinitilize SPI and the parameters */
    spi_i2s_deinit(SPI0);
    spi_struct_para_init(&spi_init_struct);

    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PRESCALAR;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI0, &spi_init_struct);
	spi_crc_polynomial_set(SPI0,7);
	spi_enable(SPI0);
}

/* ************************************************************************** */
void nRF24_gpio_init(void){
    /* SPI0 GPIO config:SCK/PA5, MISO/PA6, MOSI/PA7 */
    gpio_init(SPI_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, SPI_SCK | SPI_MOSI);
    gpio_init(SPI_PORT, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, SPI_MISO);
	
    /* RF24 CE and CSN GPIO config: CE/PA2, CSN/PA3 */
	gpio_init(RF24_CSN_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, RF24_CSN_PIN);
	gpio_init(RF24_CE_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, RF24_CE_PIN);

	nRF24_csn(HIGH);
	nRF24_ce(LOW);	
}


/* ************************************************************************** */
uint8_t nRF24_spi_transfer(uint8_t data, uint32_t timeout){
    while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE));
    spi_i2s_data_transmit(SPI0, data);
    while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE));
    return spi_i2s_data_receive(SPI0);
}
/* ************************************************************************** */

