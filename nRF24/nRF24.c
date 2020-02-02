#include "nRF24.h"


#define RF24_SPI_TIMEOUT 1
#define RF24_NOP 0xFF
#define _BV(x) (1<<(x))

#define delay(x)	delay_1ms(x)
#define delayMicroseconds(x)	delay_1us(x)

#define rf24_max(a,b) (a>b?a:b)
#define rf24_min(a,b) (a<b?a:b)

uint8_t payload_size; /**< Fixed size of payloads */
bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
bool p_variant; /* False for RF24L01 and true for RF24L01P */
uint8_t pipe0_reading_address[5]; /**< Last address set on pipe 0 for reading. */
uint8_t addr_width; /**< The address width to use - 3,4 or 5 bytes. */

uint8_t spi_rxbuff[32+1] ; //SPI receive buffer (payload max 32 bytes)
uint8_t spi_txbuff[32+1] ; //SPI transmit buffer (payload max 32 bytes + 1 byte for the command)

uint32_t txDelay;
uint32_t csDelay;


static const uint8_t child_pipe[] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};
static const uint8_t child_payload_size[] = {RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5};
static const uint8_t child_pipe_enable[] = {ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5};

uint8_t nRF24_read_buffer(uint8_t reg, uint8_t* buf, uint8_t len){
  uint8_t status;

	nRF24_csn(LOW);
	status = nRF24_spi_transfer( R_REGISTER | ( REGISTER_MASK & reg ), RF24_SPI_TIMEOUT);
	while ( len-- ){
		*buf++ = nRF24_spi_transfer(0xff, RF24_SPI_TIMEOUT);
	}
	nRF24_csn(HIGH);
	
  return status;
}

/* ************************************************************************** */
uint8_t nRF24_read_register(uint8_t reg){
	uint8_t result;

	nRF24_csn(LOW);
	nRF24_spi_transfer(R_REGISTER | (REGISTER_MASK & reg), RF24_SPI_TIMEOUT);
	result = nRF24_spi_transfer(0xff, RF24_SPI_TIMEOUT);
	nRF24_csn(HIGH);

	return result;
}

/* ************************************************************************** */
uint8_t nRF24_write_buffer(uint8_t reg, uint8_t* buf, uint8_t len){
	uint8_t status;

	nRF24_csn(LOW);
	status = nRF24_spi_transfer( W_REGISTER | ( REGISTER_MASK & reg ), RF24_SPI_TIMEOUT);
	while ( len-- )
		nRF24_spi_transfer(*buf++, RF24_SPI_TIMEOUT);
	nRF24_csn(HIGH);
	
	return status;
}

/* ************************************************************************** */
uint8_t nRF24_write_register(uint8_t reg, uint8_t value){
    uint8_t status;

	nRF24_csn(LOW);
    status = nRF24_spi_transfer(W_REGISTER | (REGISTER_MASK & reg), RF24_SPI_TIMEOUT);
    nRF24_spi_transfer(value, RF24_SPI_TIMEOUT);
	nRF24_csn(HIGH);

    return status;
}

/* ************************************************************************** */
uint8_t nRF24_write_payload(uint8_t * buf, uint8_t data_len, uint8_t writeType){
    uint8_t status;

	data_len = rf24_min(data_len, payload_size);
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

	nRF24_csn(LOW);
	status = nRF24_spi_transfer( writeType, RF24_SPI_TIMEOUT);
	while ( data_len-- ) {
		nRF24_spi_transfer(*buf++, RF24_SPI_TIMEOUT);
	}
	while ( blank_len-- ) {
		nRF24_spi_transfer(0, RF24_SPI_TIMEOUT);
	}  
	nRF24_csn(HIGH);

    return status;
}

/* ************************************************************************** */
uint8_t nRF24_read_payload(uint8_t * buf, uint8_t data_len){
    uint8_t status;
	
    if (data_len > payload_size) {
        data_len = payload_size;
    }
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

	nRF24_csn(LOW);
	status = nRF24_spi_transfer( R_RX_PAYLOAD, RF24_SPI_TIMEOUT);
	while ( data_len-- ) {
		*buf++ = nRF24_spi_transfer(0xFF, RF24_SPI_TIMEOUT);
	}
	while ( blank_len-- ) {
		nRF24_spi_transfer(0xff, RF24_SPI_TIMEOUT);
	}
  	nRF24_csn(HIGH);

    return status;
}

/* ************************************************************************** */
static uint8_t nRF24_spiTrans(uint8_t cmd){
    uint8_t status;

	nRF24_csn(LOW);
	status = nRF24_spi_transfer(cmd,  RF24_SPI_TIMEOUT);
	nRF24_csn(HIGH);

    return status;
}

/* ************************************************************************** */
uint8_t nRF24_flush_rx(void){
    return nRF24_spiTrans(FLUSH_RX);
}

/****************************************************************************/
uint8_t nRF24_flush_tx(void){
    return nRF24_spiTrans(FLUSH_TX);
}

/* ************************************************************************** */
uint8_t nRF24_get_status(void){
    return nRF24_spiTrans(RF24_NOP);
}

/* ************************************************************************** */
void nRF24_setChannel(uint8_t channel){
    const uint8_t max_channel = 125;
    nRF24_write_register(RF_CH, rf24_min(channel, max_channel));
}

/* ************************************************************************** */
uint8_t nRF24_getChannel(void){

    return nRF24_read_register(RF_CH);
}

/* ************************************************************************** */
void nRF24_setPayloadSize(uint8_t size){
    payload_size = rf24_min(size, 32);
}

/****************************************************************************/
uint8_t nRF24_getPayloadSize(void){
    return payload_size;
}

/* ************************************************************************** */
bool nRF24_isChipConnected(void){
	uint8_t setup = nRF24_read_register(SETUP_AW);
	if (setup >= 1 && setup <= 3) {
			return TRUE;
	}

	return FALSE;
}
/* ************************************************************************** */
void nRF24_powerDown(void){
    nRF24_ce(LOW); // Guarantee CE is low on powerDown
    nRF24_write_register(NRF_CONFIG, nRF24_read_register(NRF_CONFIG) & ~_BV(PWR_UP));
}

/****************************************************************************/
//Power up now. Radio will not power down unless instructed by MCU for config changes etc.
void nRF24_powerUp(void){
    uint8_t cfg = nRF24_read_register(NRF_CONFIG);

    // if not powered up then power up and wait for the radio to initialize
    if (!(cfg & _BV(PWR_UP))) {
        nRF24_write_register(NRF_CONFIG, cfg | _BV(PWR_UP));

        // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
        // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
        // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
        delay(5);
    }
}
/* ************************************************************************** */
bool nRF24_rxFifoFull(void){
    return nRF24_read_register(FIFO_STATUS) & _BV(RX_FULL);
}
/* ************************************************************************** */
void nRF24_reUseTX(void){
	nRF24_write_register(NRF_STATUS, _BV(MAX_RT));              //Clear max retry flag
	nRF24_spiTrans(REUSE_TX_PL);
  	nRF24_ce(LOW);                                          //Re-Transfer packet
	delayMicroseconds(10);
	nRF24_ce(HIGH);
}

/* ************************************************************************** */
void nRF24_setPALevel(uint8_t level){
	uint8_t setup = nRF24_read_register(RF_SETUP) & 0xF8;

	if (level > 3) {                        // If invalid level, go to max PA
			level = (RF24_PA_MAX << 1) + 1;        // +1 to support the SI24R1 chip extra bit
	} else {
			level = (level << 1) + 1;            // Else set level as requested
	}
	nRF24_write_register(RF_SETUP, setup |= level);    // Write it to the chip
}

/****************************************************************************/
uint8_t nRF24_getPALevel(void){
	return (nRF24_read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1;
}

/****************************************************************************/
uint8_t nRF24_getARC(void){
	return nRF24_read_register(OBSERVE_TX) & 0x0F;
}

/****************************************************************************/
bool nRF24_setDataRate(rf24_datarate_e speed){
	bool result = FALSE;
	uint8_t setup = nRF24_read_register(RF_SETUP);

	// HIGH and LOW '00' is 1Mbs - our default
	setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

	txDelay = 250;
	if (speed == RF24_250KBPS) {
		// Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
		// Making it '10'.
		setup |= _BV(RF_DR_LOW);
		txDelay = 450;
	} else {
		// Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
		// Making it '01'
		if (speed == RF24_2MBPS) {
			setup |= _BV(RF_DR_HIGH);
			txDelay = 190;
		}
	}
	nRF24_write_register(RF_SETUP, setup);

	// Verify our result
	if (nRF24_read_register(RF_SETUP) == setup) {
		result = TRUE;
	}
	return result;
}

/****************************************************************************/
rf24_datarate_e nRF24_getDataRate(void){
	rf24_datarate_e result;
	uint8_t dr = nRF24_read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

	// switch uses RAM (evil!)
	// Order matters in our case below
	if (dr == _BV(RF_DR_LOW)) {
		// '10' = 250KBPS
		result = RF24_250KBPS;
	} else if (dr == _BV(RF_DR_HIGH)) {
		// '01' = 2MBPS
		result = RF24_2MBPS;
	} else {
		// '00' = 1MBPS
		result = RF24_1MBPS;
	}
	return result;
}

/****************************************************************************/
void nRF24_setCRCLength(rf24_crclength_e length){
	uint8_t config = nRF24_read_register(NRF_CONFIG) & ~(_BV(CRCO) | _BV(EN_CRC));

	// switch uses RAM (evil!)
	if (length == RF24_CRC_DISABLED) {
		// Do nothing, we turned it off above.
	} else if (length == RF24_CRC_8) {
		config |= _BV(EN_CRC);
	} else {
		config |= _BV(EN_CRC);
		config |= _BV(CRCO);
	}
	nRF24_write_register(NRF_CONFIG, config);
}

/****************************************************************************/
rf24_crclength_e nRF24_getCRCLength(void){
	rf24_crclength_e result = RF24_CRC_DISABLED;

	uint8_t config = nRF24_read_register(NRF_CONFIG) & (_BV(CRCO) | _BV(EN_CRC));
	uint8_t AA = nRF24_read_register(EN_AA);

	if (config & _BV(EN_CRC) || AA) {
		if (config & _BV(CRCO)) {
			result = RF24_CRC_16;
		} else {
			result = RF24_CRC_8;
		}
	}

	return result;
}

/****************************************************************************/
void nRF24_disableCRC(void){
    uint8_t disable = nRF24_read_register(NRF_CONFIG) & ~_BV(EN_CRC);
    nRF24_write_register(NRF_CONFIG, disable);
}

/****************************************************************************/
void nRF24_setRetries(uint8_t delay, uint8_t count){
    nRF24_write_register(SETUP_RETR, (delay & 0xf) << ARD | (count & 0xf) << ARC);
}

/* ************************************************************************** */
bool nRF24_testCarrier(void){
	return (nRF24_read_register(CD) & 1);
}

/****************************************************************************/
bool nRF24_testRPD(void){
	return (nRF24_read_register(RPD) & 1);
}

/* ************************************************************************** */
static void nRF24_toggle_features(void){
  nRF24_csn(LOW);

	nRF24_spi_transfer( ACTIVATE, RF24_SPI_TIMEOUT );
   nRF24_spi_transfer( 0x73, RF24_SPI_TIMEOUT );

  nRF24_csn(HIGH);
}

/* ************************************************************************** */
void nRF24_openWritingPipe_d(uint8_t value){
	// Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
	// expects it LSB first too, so we're good.
	
	uint8_t data[8] = {0};
	data[0] = value;

	nRF24_write_buffer(RX_ADDR_P0, data, addr_width);
	nRF24_write_buffer(TX_ADDR, data, addr_width);

	//const uint8_t max_payload_size = 32;
	//write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
	nRF24_write_register(RX_PW_P0, payload_size);
}

/****************************************************************************/
void nRF24_openWritingPipe(uint8_t* address){
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.
    nRF24_write_buffer(RX_ADDR_P0, address, addr_width);
    nRF24_write_buffer(TX_ADDR, address, addr_width);

    //const uint8_t max_payload_size = 32;
    //write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
    nRF24_write_register(RX_PW_P0, payload_size);
}

/* ************************************************************************** */
void nRF24_setAddressWidth(uint8_t a_width){
	if (a_width -= 2) {
			nRF24_write_register(SETUP_AW, a_width % 4);
			addr_width = (a_width % 4) + 2;
	} else {
			nRF24_write_register(SETUP_AW, 0);
			addr_width = 2;
	}
}

/* ************************************************************************** */
void nRF24_openReadingPipe_d(uint8_t child, uint8_t value){
	uint8_t data[8] = {0};
	data[0] = value;
	// If this is pipe 0, cache the address.  This is needed because
	// openWritingPipe() will overwrite the pipe 0 address, so
	// startListening() will have to restore it.
	if (child == 0) {
			memcpy(pipe0_reading_address, data, addr_width);
	}

	if (child <= 6) {
			// For pipes 2-5, only write the LSB
			if (child < 2) {
					nRF24_write_buffer(child_pipe[child], data, addr_width);
			} else {
					nRF24_write_buffer(child_pipe[child], data, 1);
			}

			nRF24_write_register(child_payload_size[child], payload_size);

			// Note it would be more efficient to set all of the bits for all open
			// pipes at once.  However, I thought it would make the calling code
			// more simple to do it this way.
			nRF24_write_register(EN_RXADDR, nRF24_read_register(EN_RXADDR) | _BV(child_pipe_enable[child]));
	}
}

/* ************************************************************************** */
void nRF24_openReadingPipe(uint8_t child, uint8_t* address){
	// If this is pipe 0, cache the address.  This is needed because
	// openWritingPipe() will overwrite the pipe 0 address, so
	// startListening() will have to restore it.
	if (child == 0) {
		memcpy(pipe0_reading_address, address, addr_width);
	}
	if (child <= 6) {
		// For pipes 2-5, only write the LSB
		if (child < 2) {
				nRF24_write_buffer(child_pipe[child], address, addr_width);
		} else {
				nRF24_write_buffer(child_pipe[child], address, 1);
		}
		nRF24_write_register(child_payload_size[child], payload_size);

		// Note it would be more efficient to set all of the bits for all open
		// pipes at once.  However, I thought it would make the calling code
		// more simple to do it this way.
		nRF24_write_register(EN_RXADDR, nRF24_read_register(EN_RXADDR) | _BV(child_pipe_enable[child]));
	}
}

/* ************************************************************************** */
void nRF24_startListening(void){
	nRF24_powerUp();
	nRF24_write_register(NRF_CONFIG, nRF24_read_register(NRF_CONFIG) | _BV(PRIM_RX));
	nRF24_write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
	nRF24_ce(HIGH);
	// Restore the pipe0 adddress, if exists
	if (pipe0_reading_address[0] > 0) {
		nRF24_write_buffer(RX_ADDR_P0, pipe0_reading_address, addr_width);
	} else {
		nRF24_closeReadingPipe(0);
	}

	// Flush buffers
	//flush_rx();
	if (nRF24_read_register(FEATURE) & _BV(EN_ACK_PAY)) {
		nRF24_flush_tx();
	}
}

/****************************************************************************/
void nRF24_stopListening(void){
	nRF24_ce(LOW);

	delayMicroseconds(txDelay);

	if (nRF24_read_register(FEATURE) & _BV(EN_ACK_PAY)) {
		delayMicroseconds(txDelay); //200
		nRF24_flush_tx();
	}
	nRF24_write_register(NRF_CONFIG, (nRF24_read_register(NRF_CONFIG)) & ~_BV(PRIM_RX));
	nRF24_write_register(EN_RXADDR, nRF24_read_register(EN_RXADDR) | _BV(child_pipe_enable[0])); // Enable RX on pipe0
}

/* ************************************************************************** */
void nRF24_closeReadingPipe(uint8_t pipe){
	nRF24_write_register(EN_RXADDR, nRF24_read_register(EN_RXADDR) & ~_BV(child_pipe_enable[pipe]));
}

/* ************************************************************************** */
bool nRF24_available(void){
    return nRF24_available_d(NULL);
}

/****************************************************************************/
bool nRF24_available_d(uint8_t* pipe_num){
	if (!(nRF24_read_register(FIFO_STATUS) & _BV(RX_EMPTY))) {
		// If the caller wants the pipe number, include that
		if (pipe_num) {
			uint8_t status = nRF24_get_status();
			*pipe_num = (status >> RX_P_NO) & 0x07;
		}
		return 1;
	}
	return 0;
}

/* ************************************************************************** */
void nRF24_startFastWrite(uint8_t* buf, uint8_t len, bool multicast, bool startTx){
	//write_payload( buf,len);
	nRF24_write_payload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
	if (startTx) {
		nRF24_ce(HIGH);
	}
}

/* ************************************************************************** */
void nRF24_read(uint8_t* buf, uint8_t len){
	// Fetch the payload
	nRF24_read_payload(buf, len);

	//Clear the two possible interrupt flags with one command
	nRF24_write_register(NRF_STATUS, _BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS));
}

/* ************************************************************************** */
//Similar to the previous write, clears the interrupt flags
bool nRF24_write_multi(uint8_t* buf, uint8_t len, bool multicast){
	//Start Writing
	nRF24_startFastWrite(buf, len, multicast, TRUE);

	//Wait until complete or failed
	#if defined(FAILURE_HANDLING)
		mxc_delay_start(MXC_DELAY_MSEC(1000));
	#endif // defined(FAILURE_HANDLING)

	while (!(nRF24_get_status() & (_BV(TX_DS) | _BV(MAX_RT)))) {
		#if defined(FAILURE_HANDLING)
			if (mxc_delay_check() != E_BUSY) {
				printf("TIMEOUT! Please check the pin connections detailed in main.c and then reset the board.\n");
				errNotify();
				return 0;
			}
		}
		#endif
	}

	nRF24_ce(LOW);
	uint8_t status = nRF24_write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

	//Max retries exceeded
	if (status & _BV(MAX_RT)) {
		nRF24_flush_tx(); //Only going to be 1 packet int the FIFO at a time using this method, so just flush
		return 0;
	}
	//TX OK 1 or 0
	return 1;
}

/* ************************************************************************** */
bool nRF24_write(uint8_t* buf, uint8_t len){
    return nRF24_write_multi(buf, len, 0);
}

/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */
void nRF24_init(void){
	nRF24_spi_init();
//	nRF24_gpio_init();
	pipe0_reading_address[0] = 0;
	p_variant = FALSE;
	payload_size = 32;
	dynamic_payloads_enabled = FALSE;
	addr_width = 5;
	csDelay = 5;
}

/* ************************************************************************** */
uint8_t nRF24_begin(void){
    uint8_t setup = 0;
	nRF24_init();
    delay(100);

    // Reset NRF_CONFIG and enable 16-bit CRC.
    nRF24_write_register(NRF_CONFIG, 0x0C);

    // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
    // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
    // sizes must never be used. See documentation for a more complete explanation.
    nRF24_setRetries(5, 15);

    // Reset value is MAX
    //setPALevel( RF24_PA_MAX ) ;

    // check for connected module and if this is a p nRF24l01 variant
    //
    if (nRF24_setDataRate(RF24_250KBPS)) {
        p_variant = TRUE;
    }
    setup = nRF24_read_register(RF_SETUP);
    /*if( setup == 0b00001110 )     // register default for nRF24L01P
    {
      p_variant = true ;
    }*/

    // Then set the data rate to the slowest (and most reliable) speed supported by all
    // hardware.
    nRF24_setDataRate(RF24_1MBPS);

    // Initialize CRC and request 2-byte (16bit) CRC
    //setCRCLength( RF24_CRC_16 ) ;

    // Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
    nRF24_toggle_features();
    nRF24_write_register(FEATURE, 0);
    nRF24_write_register(DYNPD, 0);
    dynamic_payloads_enabled = FALSE;

    // Reset current status
    // Notice reset and flush is the last thing we do
    nRF24_write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    // Set up default configuration.  Callers can always change it later.
    // This channel should be universally safe and not bleed over into adjacent
    // spectrum.
    nRF24_setChannel(76);

    // Flush buffers
    nRF24_flush_rx();
    nRF24_flush_tx();

    nRF24_powerUp(); //Power up by default when begin() is called

    // Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from powerUp )
    // PTX should use only 22uA of power
    nRF24_write_register(NRF_CONFIG, (nRF24_read_register(NRF_CONFIG)) & ~_BV(PRIM_RX));

    // if setup is 0 or ff then there was no response from module
    return (setup != 0 && setup != 0xff);
}

/* ************************************************************************** */
