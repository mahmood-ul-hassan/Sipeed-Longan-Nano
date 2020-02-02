/*!
    \file  i2c.c
    \brief I2C configuration file
    
    \version 2019-06-05, V1.0.0, demo for GD32VF103
*/

#include "gd32vf103.h"
#include "ir_sensor/i2c.h"
#include <stdio.h>

/*!
    \brief      configure the GPIO ports
    \param[in]  i2c_periph: I2C0 or I2C1
    \param[out] none
    \retval     none
*/
void gpio_config(uint32_t i2c_periph)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);

    if(i2c_periph == I2C0){
        /* I2C0 GPIO port */
        /* connect PB6 to I2C0_SCL */
        /* connect PB7 to I2C0_SDA */
        gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
    }
    if(i2c_periph == I2C1){
        /* I2C1 GPIO ports */
        /* connect PB10 to I2C1_SCL */
        /* connect PB11 to I2C1_SDA */
        gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_10 | GPIO_PIN_11);
    }
}

/*!
    \brief      configure the I2C0 interfaces
    \param[in]  i2c_periph: I2C0 or I2C1
    \param[out] none
    \retval     none
*/
void i2c_config(uint32_t i2c_periph)
{

    /* configure the I2C GPIO port */
    gpio_config(i2c_periph);
    if(i2c_periph == I2C0){
        /* enable I2C0 clock */
        rcu_periph_clock_enable(RCU_I2C0);
        /* configure I2C0 clock */
        i2c_clock_config(i2c_periph,I2C0_SPEED,I2C_DTCY_2);
        /* configure I2C address */
        i2c_mode_addr_config(i2c_periph,I2C_I2CMODE_ENABLE,I2C_ADDFORMAT_7BITS,I2C0_MASTER_ADDRESS7);
    }
    if(i2c_periph == I2C1){
        /* enable I2C1 clock */
        rcu_periph_clock_enable(RCU_I2C1);
        /* configure I2C1 clock */
        i2c_clock_config(i2c_periph,I2C1_SPEED,I2C_DTCY_2);
        /* configure I2C1 address */
        i2c_mode_addr_config(i2c_periph,I2C_I2CMODE_ENABLE,I2C_ADDFORMAT_7BITS,I2C1_MASTER_ADDRESS7);
    }
    /* enable I2C */
    i2c_enable(i2c_periph);
    /* enable acknowledge */
    i2c_ack_config(i2c_periph,I2C_ACK_ENABLE);
}

/*!
    \brief      I2C timeout
    \param[in]  start_i2c_time: systick timer value
    \param[in]  timeout: msec
    \param[out] none
    \retval     none
*/
uint8_t i2c_timeout(uint64_t start_i2c_time, uint32_t timeout){
    uint64_t delta_mtime;
    delta_mtime = get_timer_value() - start_i2c_time;
    if(delta_mtime <(SystemCoreClock/4000.0 *timeout))
        return 0;
    else
        return 1;
}

/*!
    \brief      I2C Read Buffer
    \param[in]  i2c_periph: I2C0 or I2C1
    \param[in]  slave_address: 7-bit I2C slave address shifted left by 1 bit
    \param[in]  read_address: 8-bit I2C slave register address
    \param[in]  p_buffer: pointer of the buffer to store the read data
    \param[in]  number_of_byte: number of byte needed to read and store in p_buffer
    \param[in]  timeout: msec
    \param[out] none
    \retval     none
*/
uint8_t i2c_read(uint32_t i2c_periph, uint8_t slave_address, uint8_t read_address, uint8_t* p_buffer, uint8_t number_of_byte, uint32_t timeout)
{
    i2c_deinit(i2c_periph);
    i2c_config(i2c_periph);
  
    uint64_t start_mtime;
    	// Don't start measuruing until we see an mtime tick
    uint64_t tmp = get_timer_value();
    do {
    start_mtime = get_timer_value();
    } while (start_mtime == tmp);

    /* wait until I2C bus is idle */
    while(i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY)){
        if(i2c_timeout(start_mtime, timeout))
            return 0;
    }


    if(2 == number_of_byte){
        i2c_ackpos_config(i2c_periph,I2C_ACKPOS_NEXT);
    }
    
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c_periph);
    
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND)){
        if(i2c_timeout(start_mtime, timeout))
            return 0;
    }
    
    /* send slave address to I2C bus */
    i2c_master_addressing(i2c_periph, slave_address, I2C_TRANSMITTER);
    
    /* wait until ADDSEND bit is set */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND)){
        if(i2c_timeout(start_mtime, timeout))
            return 0;
    }
    
    /* clear the ADDSEND bit */
    i2c_flag_clear(i2c_periph,I2C_FLAG_ADDSEND);
    
    /* wait until the transmit data buffer is empty */
    while(SET != i2c_flag_get( i2c_periph , I2C_FLAG_TBE)){
        if(i2c_timeout(start_mtime, timeout))
            return 0;
    }

    /* enable i2c_periph*/
    i2c_enable(i2c_periph);
    
    /* send the EEPROM's internal address to write to */
    i2c_data_transmit(i2c_periph, read_address);  
    
    /* wait until BTC bit is set */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_BTC)){
        if(i2c_timeout(start_mtime, timeout))
            return 0;
    }
    
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c_periph);
    
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND)){
        if(i2c_timeout(start_mtime, timeout))
            return 0;
    }
    
    /* send slave address to I2C bus */
    i2c_master_addressing(i2c_periph, slave_address, I2C_RECEIVER);

    if(number_of_byte < 3){
        /* disable acknowledge */
        i2c_ack_config(i2c_periph,I2C_ACK_DISABLE);
    }
    
    /* wait until ADDSEND bit is set */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND)){
        if(i2c_timeout(start_mtime, timeout))
            return 0;
    }
    
    /* clear the ADDSEND bit */
    i2c_flag_clear(i2c_periph,I2C_FLAG_ADDSEND);
    
    if(1 == number_of_byte){
        /* send a stop condition to I2C bus */
        i2c_stop_on_bus(i2c_periph);
    }
    
    /* while there is data to be read */
    while(number_of_byte){
        if(3 == number_of_byte){
            /* wait until BTC bit is set */
            while(!i2c_flag_get(i2c_periph, I2C_FLAG_BTC)){
              if(i2c_timeout(start_mtime, timeout))
                    return 0;
            }

            /* disable acknowledge */
            i2c_ack_config(i2c_periph,I2C_ACK_DISABLE);
        }
        if(2 == number_of_byte){
            /* wait until BTC bit is set */
            while(!i2c_flag_get(i2c_periph, I2C_FLAG_BTC)){
              if(i2c_timeout(start_mtime, timeout))
                return 0;
            }
            
            /* send a stop condition to I2C bus */
            i2c_stop_on_bus(i2c_periph);
        }
        
        /* wait until the RBNE bit is set and clear it */
        if(i2c_flag_get(i2c_periph, I2C_FLAG_RBNE)){
            /* read a byte from the EEPROM */
            *p_buffer = i2c_data_receive(i2c_periph);
            
            /* point to the next location where the byte read will be saved */
            p_buffer++; 
            
            /* decrement the read bytes counter */
            number_of_byte--;
        } 
    }
    
    /* wait until the stop condition is finished */
    while(I2C_CTL0(i2c_periph)&0x0200){
      if(i2c_timeout(start_mtime, timeout))
        return 0;
    }
    
    /* enable acknowledge */
    i2c_ack_config(i2c_periph,I2C_ACK_ENABLE);
    i2c_ackpos_config(i2c_periph,I2C_ACKPOS_CURRENT);

    return 1;
}

/*!
    \brief      I2C Read Buffer
    \param[in]  i2c_periph: I2C0 or I2C1
    \param[in]  slave_address: 7-bit I2C slave address shifted left by 1 bit
    \param[in]  write_address: 8-bit I2C slave register address
    \param[in]  p_buffer: pointer of the buffer with write data
    \param[in]  number_of_byte: number of byte needed to write from p_buffer
    \param[in]  timeout: msec
    \param[out] none
    \retval     none
*/
uint8_t i2c_write(uint32_t i2c_periph, uint8_t slave_address, uint8_t write_address, uint8_t* p_buffer, uint8_t number_of_byte, uint32_t timeout)
{
    i2c_deinit(i2c_periph);
    i2c_config(i2c_periph);
  
    uint64_t start_mtime;
    	// Don't start measuruing until we see an mtime tick
    uint64_t tmp = get_timer_value();
    do {
    start_mtime = get_timer_value();
    } while (start_mtime == tmp);

    /* wait until I2C bus is idle */
    while(i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY)){
        if(i2c_timeout(start_mtime, timeout))
            return 0;
    }
    
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c_periph);
    
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND)){
        if(i2c_timeout(start_mtime, timeout))
            return 0;
    }
    
    /* send slave address to I2C bus */
    i2c_master_addressing(i2c_periph, slave_address, I2C_TRANSMITTER);
    
    /* wait until ADDSEND bit is set */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND)){
        if(i2c_timeout(start_mtime, timeout))
            return 0;
    }

    
    /* clear the ADDSEND bit */
    i2c_flag_clear(i2c_periph,I2C_FLAG_ADDSEND);
    
    /* wait until the transmit data buffer is empty */
    while( SET != i2c_flag_get(i2c_periph, I2C_FLAG_TBE)){
        if(i2c_timeout(start_mtime, timeout))
            return 0;
    }

    /* send the EEPROM's internal address to write to : only one byte address */
    i2c_data_transmit(i2c_periph, write_address);
    
    /* wait until BTC bit is set */
    while(!i2c_flag_get(i2c_periph, I2C_FLAG_BTC)){
        if(i2c_timeout(start_mtime, timeout))
            return 0;
    }
    
    /* while there is data to be written */
    while(number_of_byte--){  
        i2c_data_transmit(i2c_periph, *p_buffer);
        
        /* point to the next byte to be written */
        p_buffer++; 
        
        /* wait until BTC bit is set */
        while(!i2c_flag_get(i2c_periph, I2C_FLAG_BTC)){
            if(i2c_timeout(start_mtime, timeout))
                return 0;
        }
    }
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(i2c_periph);
    
    /* wait until the stop condition is finished */
    while(I2C_CTL0(i2c_periph)&0x0200){
        if(i2c_timeout(start_mtime, timeout))
            return 0;
    }

    return 1;
}

