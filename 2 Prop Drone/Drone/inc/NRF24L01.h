#ifndef __NRF24L01_H_
#define __NRF24L01_H_

#include "pico/stdlib.h"
#include "pico/stdio.h"

#include "hardware/spi.h"
#include "hardware/gpio.h"

class NRF24
{
private: // Vars.
    spi_inst_t *spi;
    uint8_t csn = 0;
    uint8_t ce = 0;

    uint8_t status = 0;
    uint8_t fifo_status = 0;

public: // Vars.
    uint8_t messageLen = 7;
    uint8_t channel = 2;
    uint16_t packetsLost = 0;
private: // Funcs.
    void csnLow() {
        gpio_put(csn,0);
    }
    void csnHigh() { 
        gpio_put(csn,1);
    }
    void ceLow() { 
        gpio_put(ce,0);
    }
    void ceHigh() { 
        gpio_put(ce,1);
    }

    uint8_t readReg(uint8_t reg);
    
    inline void writeCommand(uint8_t cmd);
    inline void writeReg(uint8_t reg, uint8_t data);
    inline void writeReg(uint8_t reg, uint8_t *data, uint8_t size);

public:
    NRF24(spi_inst_t *spi, uint16_t cns, uint16_t ce);

    void enableAck(uint8_t ack);
    void config(uint8_t *address, uint8_t channel = 2, uint8_t messageLen = 7);

    void modeRX();
    void modeTX();

    uint8_t newMessage();
    void sendMessage(uint8_t *data);
    void getMessage(uint8_t *buffer);

    uint8_t readRegister(uint8_t reg);
};



#endif

