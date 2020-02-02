#ifndef __nRF24_H__
#define __nRF24_H__

#include "nRF24L01.h"
#include "nRF24_hw.h"

typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e ;
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;

uint8_t nRF24_read_buffer(uint8_t reg, uint8_t* buf, uint8_t len);
uint8_t nRF24_read_register(uint8_t reg);
uint8_t nRF24_write_buffer(uint8_t reg, uint8_t* buf, uint8_t len);
uint8_t nRF24_write_register(uint8_t reg, uint8_t value);
uint8_t nRF24_write_payload(uint8_t * buf, uint8_t data_len, uint8_t writeType);
uint8_t nRF24_read_payload(uint8_t * buf, uint8_t data_len);
uint8_t nRF24_flush_rx(void);
uint8_t nRF24_flush_tx(void);
uint8_t nRF24_get_status(void);
void nRF24_setChannel(uint8_t channel);
uint8_t nRF24_getChannel(void);
void nRF24_setPayloadSize(uint8_t size);
uint8_t nRF24_getPayloadSize(void);
void nRF24_setPALevel(uint8_t level);
uint8_t nRF24_getPALevel(void);
uint8_t nRF24_getARC(void);
bool nRF24_setDataRate(rf24_datarate_e speed);
rf24_datarate_e nRF24_getDataRate(void);
void nRF24_setCRCLength(rf24_crclength_e length);
rf24_crclength_e nRF24_getCRCLength(void);
void nRF24_disableCRC(void);
void nRF24_setRetries(uint8_t delay, uint8_t count);
bool nRF24_testCarrier(void);
bool nRF24_testRPD(void);

void nRF24_openWritingPipe(uint8_t* address);
void nRF24_openWritingPipe_d(uint8_t value);
void nRF24_setAddressWidth(uint8_t a_width);
void nRF24_openReadingPipe(uint8_t child, uint8_t* address);
void nRF24_openReadingPipe_d(uint8_t child, uint8_t address);

void nRF24_startListening(void);
void nRF24_stopListening(void);
void nRF24_closeReadingPipe(uint8_t pipe);

bool nRF24_available(void);
bool nRF24_available_d(uint8_t* pipe_num);

void nRF24_read(uint8_t* buf, uint8_t len);
bool nRF24_write(uint8_t* buf, uint8_t len);
bool nRF24_write_multi(uint8_t* buf, uint8_t len, const bool multicast);
void nRF24_startFastWrite(uint8_t* buf, uint8_t len, bool multicast, bool startTx);

bool nRF24_isChipConnected(void);
void nRF24_powerDown(void);
void nRF24_powerUp(void);
void nRF24_reUseTX(void);
bool nRF24_rxFifoFull(void);

void nRF24_init(void);
uint8_t nRF24_begin(void);

#endif // __RF24_H__
