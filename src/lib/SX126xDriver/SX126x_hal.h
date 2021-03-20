#pragma once

/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian

Heavily modified/simplified by Alessandro Carcione 2020 for ELRS project 
*/

#include "SX126x_Regs.h"
#include "SX126x.h"

enum SX126x_InterruptAssignment_
{
    SX126X_INTERRUPT_NONE,
    SX126X_INTERRUPT_RX_DONE,
    SX126X_INTERRUPT_TX_DONE
};

enum SX126x_BusyState_
{
    SX126X_NOT_BUSY = true,
    SX126X_BUSY = false,
};

class SX126xHal
{

private:
    volatile SX126x_InterruptAssignment_ InterruptAssignment = SX126X_INTERRUPT_NONE;

public:
    static SX126xHal *instance;

    SX126xHal();

    void init();
    void end();
    void reset();

    void ICACHE_RAM_ATTR PrintStatus(SX126x_RadioCommands_t command);

    void ICACHE_RAM_ATTR WriteCommand(SX126x_RadioCommands_t command);
    void ICACHE_RAM_ATTR WriteCommand(SX126x_RadioCommands_t opcode, uint8_t *buffer, uint8_t size);
    void ICACHE_RAM_ATTR WriteCommand(SX126x_RadioCommands_t command, uint8_t val);
    void ICACHE_RAM_ATTR WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size);
    void ICACHE_RAM_ATTR WriteRegister(uint16_t address, uint8_t value);

    void ICACHE_RAM_ATTR ReadCommand(SX126x_RadioCommands_t opcode, uint8_t *buffer, uint8_t size);
    void ICACHE_RAM_ATTR ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size);
    uint8_t ICACHE_RAM_ATTR ReadRegister(uint16_t address);

    void ICACHE_RAM_ATTR WriteBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size); // Writes and Reads to FIFO
    void ICACHE_RAM_ATTR ReadBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size);

    static void ICACHE_RAM_ATTR nullCallback(void);
    
    void ICACHE_RAM_ATTR WaitOnBusy();
    static ICACHE_RAM_ATTR void dioISR();
    
    void ICACHE_RAM_ATTR TXenable();
    void ICACHE_RAM_ATTR RXenable();
    void ICACHE_RAM_ATTR TXRXdisable();

    static void (*TXdoneCallback)(); //function pointer for callback
    static void (*RXdoneCallback)(); //function pointer for callback

    void ICACHE_RAM_ATTR SetDio3AsTcxoCtrl(uint8_t tcxoVoltage, uint32_t timeout);
};
