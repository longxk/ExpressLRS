/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy

Modified and adapted by Alessandro Carcione for ELRS project 
*/

#include "SX126x_Regs.h"
#include "SX126x_hal.h"

#include <Arduino.h>
#include <SPI.h>

SX126xHal *SX126xHal::instance = NULL;

void ICACHE_RAM_ATTR SX126xHal::nullCallback(void){};

void (*SX126xHal::TXdoneCallback)() = &nullCallback;
void (*SX126xHal::RXdoneCallback)() = &nullCallback;

SX126xHal::SX126xHal()
{
    instance = this;
}

void SX126xHal::end()
{
    SPI.end();
    detachInterrupt(GPIO_PIN_DIO0);
}

void SX126xHal::init()
{
    Serial.println("Hal Init");
    pinMode(GPIO_PIN_BUSY, INPUT);
    pinMode(GPIO_PIN_DIO1, INPUT);

    pinMode(GPIO_PIN_RST, OUTPUT);
    pinMode(GPIO_PIN_NSS, OUTPUT);

#if defined(GPIO_PIN_RX_ENABLE) || defined(GPIO_PIN_TX_ENABLE)
    Serial.print("This Target uses seperate TX/RX enable pins: ");
#endif

#if defined(GPIO_PIN_TX_ENABLE)
    Serial.print("TX: ");
    Serial.print(GPIO_PIN_TX_ENABLE);
#endif

#if defined(GPIO_PIN_RX_ENABLE)
    Serial.print(" RX: ");
    Serial.println(GPIO_PIN_RX_ENABLE);
#endif

#if defined(GPIO_PIN_RX_ENABLE)
    pinMode(GPIO_PIN_RX_ENABLE, OUTPUT);
    digitalWrite(GPIO_PIN_RX_ENABLE, LOW);
#endif

#if defined(GPIO_PIN_TX_ENABLE)
    pinMode(GPIO_PIN_TX_ENABLE, OUTPUT);
    digitalWrite(GPIO_PIN_TX_ENABLE, LOW);
#endif

#ifdef PLATFORM_ESP32
    SPI.begin(GPIO_PIN_SCK, GPIO_PIN_MISO, GPIO_PIN_MOSI, -1); // sck, miso, mosi, ss (ss can be any GPIO)
    gpio_pullup_en((gpio_num_t)GPIO_PIN_MISO);
    SPI.setFrequency(10000000);
#endif

#ifdef PLATFORM_ESP8266
    Serial.println("PLATFORM_ESP8266");
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setFrequency(10000000);
#endif

#ifdef PLATFORM_STM32
    SPI.setMOSI(GPIO_PIN_MOSI);
    SPI.setMISO(GPIO_PIN_MISO);
    SPI.setSCLK(GPIO_PIN_SCK);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV4); // 72 / 8 = 9 MHz
#endif

#ifdef PLATFORM_ASR6501
    Serial.println("PLATFORM_ASR6501");
    SPI.begin(10000000);
#endif

    //attachInterrupt(digitalPinToInterrupt(GPIO_PIN_BUSY), this->busyISR, CHANGE); //not used atm
    attachInterrupt(GPIO_PIN_DIO1, this->dioISR, RISING);
}

void ICACHE_RAM_ATTR SX126xHal::reset(void)
{
    Serial.println("SX126x Reset");
    delay(50);
    digitalWrite(GPIO_PIN_RST, LOW);
    delay(50);
    digitalWrite(GPIO_PIN_RST, HIGH);

    while (digitalRead(GPIO_PIN_BUSY) == HIGH) // wait for busy
    {
        #ifdef PLATFORM_STM32
        __NOP();
        #elif PLATFORM_ESP32
        _NOP();
        #elif PLATFORM_ESP8266
        _NOP();
        #endif
    }

    //this->BusyState = SX126X_NOT_BUSY;
    Serial.println("SX126x Ready!");
}

void ICACHE_RAM_ATTR SX126xHal::PrintStatus(SX126x_RadioCommands_t command)
{
    if (command == SX126X_RADIO_GET_STATUS || command == SX126X_RADIO_GET_DEVICE_ERROR ||
        command == SX126X_RADIO_CLR_DEVICE_ERROR) {
        return;
    }
    uint8_t status = 0xff;
    ReadCommand(SX126X_RADIO_GET_STATUS, (uint8_t *)&status, 1);
    uint8_t stat1 = (0b01110000 & status) >> 4;
    uint8_t stat2 = (0b00001110 & status) >> 1;
    bool busy = 0b00000001 & status; // reserved, alway 0
    Serial.print("Command: ");
    Serial.print(command, HEX);
    Serial.print(", Status: ");
    Serial.print(stat1, HEX);
    Serial.print(", ");
    Serial.print(stat2, HEX);
    Serial.print(", ");
    Serial.println(busy, HEX);
}

void ICACHE_RAM_ATTR SX126xHal::WriteCommand(SX126x_RadioCommands_t command)
{
    WaitOnBusy();
    digitalWrite(GPIO_PIN_NSS, LOW);
    SPI.transfer((uint8_t)command);
    digitalWrite(GPIO_PIN_NSS, HIGH);

    //PrintStatus(command);
}

void ICACHE_RAM_ATTR SX126xHal::WriteCommand(SX126x_RadioCommands_t command, uint8_t val)
{
    WaitOnBusy();
    digitalWrite(GPIO_PIN_NSS, LOW);
    SPI.transfer((uint8_t)command);
    SPI.transfer(val);
    digitalWrite(GPIO_PIN_NSS, HIGH);

    //PrintStatus(command);
}

void ICACHE_RAM_ATTR SX126xHal::WriteCommand(SX126x_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();
    digitalWrite(GPIO_PIN_NSS, LOW);
    SPI.transfer((uint8_t)command);
    for(uint16_t i = 0; i < size; i++ )
    {
        SPI.transfer(buffer[i]);
    }
    digitalWrite(GPIO_PIN_NSS, HIGH);

    //PrintStatus(command);
}

void ICACHE_RAM_ATTR SX126xHal::ReadCommand(SX126x_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();
    digitalWrite(GPIO_PIN_NSS, LOW);
    SPI.transfer((uint8_t)command);
    if (command == SX126X_RADIO_GET_STATUS) {
        buffer[0] = SPI.transfer(0x00);
    } else {
        SPI.transfer(0x00);
        for(uint16_t i = 0; i < size; i++ )
        {
            buffer[i] = SPI.transfer(0);
        }
    }
    digitalWrite(GPIO_PIN_NSS, HIGH);

    //PrintStatus(command);
}

void ICACHE_RAM_ATTR SX126xHal::WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();
    digitalWrite(GPIO_PIN_NSS, LOW);
    SPI.transfer(SX126X_RADIO_WRITE_REGISTER);
    SPI.transfer((address & 0xFF00) >> 8);
    SPI.transfer(address & 0x00FF);
    for(uint16_t i = 0; i < size; i++ )
    {
        SPI.transfer(buffer[i]);
    }
    digitalWrite(GPIO_PIN_NSS, HIGH);
}

void ICACHE_RAM_ATTR SX126xHal::WriteRegister(uint16_t address, uint8_t value)
{
    WriteRegister(address, &value, 1);
}

void ICACHE_RAM_ATTR SX126xHal::ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();
    digitalWrite(GPIO_PIN_NSS, LOW);

    SPI.transfer(SX126X_RADIO_READ_REGISTER);
    SPI.transfer((address & 0xFF00) >> 8);
    SPI.transfer(address & 0x00FF);
    SPI.transfer(0);
    for(uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SPI.transfer(0);
    }
    digitalWrite(GPIO_PIN_NSS, HIGH);
}

uint8_t ICACHE_RAM_ATTR SX126xHal::ReadRegister(uint16_t address)
{
    uint8_t data;
    ReadRegister(address, &data, 1);
    return data;
}

void ICACHE_RAM_ATTR SX126xHal::WriteBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();
    digitalWrite(GPIO_PIN_NSS, LOW);
    SPI.transfer(SX126X_RADIO_WRITE_BUFFER);
    SPI.transfer(offset);
    for(uint16_t i = 0; i < size; i++ )
    {
        SPI.transfer(buffer[i]);
    }
    digitalWrite(GPIO_PIN_NSS, HIGH);
}

void ICACHE_RAM_ATTR SX126xHal::ReadBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();
    digitalWrite(GPIO_PIN_NSS, LOW);

    SPI.transfer(SX126X_RADIO_READ_BUFFER);
    SPI.transfer(offset);
    SPI.transfer(0x00);
    for(uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SPI.transfer(0);
    }

    digitalWrite(GPIO_PIN_NSS, HIGH);
}

void ICACHE_RAM_ATTR SX126xHal::WaitOnBusy()
{
    #define wtimeoutUS 5000
    uint32_t startTime = micros();

    while (digitalRead(GPIO_PIN_BUSY) == HIGH) // wait untill not busy or until wtimeoutUS
    {
        if (micros() > startTime + wtimeoutUS)
        {
            return;
        }
        else
        {
            #ifdef PLATFORM_STM32
            __NOP();
            #elif PLATFORM_ESP32
            _NOP();
            #elif PLATFORM_ESP8266
            _NOP();
            #endif
        }
    }
}

void ICACHE_RAM_ATTR SX126xHal::dioISR()
{
    if (instance->InterruptAssignment == SX126X_INTERRUPT_RX_DONE)
    {
        RXdoneCallback();
    }
    else if (instance->InterruptAssignment == SX126X_INTERRUPT_TX_DONE)
    {
        TXdoneCallback();
    }
}

void ICACHE_RAM_ATTR SX126xHal::TXenable()
{
    if (instance->InterruptAssignment != SX126X_INTERRUPT_TX_DONE)
    {
        instance->InterruptAssignment = SX126X_INTERRUPT_TX_DONE;
        //Serial.println("TXenb");
    }

    #if defined(GPIO_PIN_RX_ENABLE)
    digitalWrite(GPIO_PIN_RX_ENABLE, LOW);
    #endif

    #if defined(GPIO_PIN_TX_ENABLE)
    digitalWrite(GPIO_PIN_TX_ENABLE, HIGH);
    #endif
}

void ICACHE_RAM_ATTR SX126xHal::RXenable()
{

    if (instance->InterruptAssignment != SX126X_INTERRUPT_RX_DONE)
    {
        instance->InterruptAssignment = SX126X_INTERRUPT_RX_DONE;
        //Serial.println("RXenb");
    }

    #if defined(GPIO_PIN_RX_ENABLE)
    digitalWrite(GPIO_PIN_RX_ENABLE, HIGH);
    #endif

    #if defined(GPIO_PIN_TX_ENABLE)
    digitalWrite(GPIO_PIN_TX_ENABLE, LOW);
    #endif
}

void ICACHE_RAM_ATTR SX126xHal::TXRXdisable()
{
    if (this->InterruptAssignment != SX126X_INTERRUPT_NONE)
    {
        this->InterruptAssignment = SX126X_INTERRUPT_NONE;
    }
    #if defined(GPIO_PIN_RX_ENABLE)
    digitalWrite(GPIO_PIN_RX_ENABLE, LOW);
    #endif

    #if defined(GPIO_PIN_TX_ENABLE)
    digitalWrite(GPIO_PIN_TX_ENABLE, LOW);
    #endif
}

void ICACHE_RAM_ATTR SX126xHal::SetDio3AsTcxoCtrl(uint8_t tcxoVoltage, uint32_t timeout)
{
    uint8_t buf[4];

    buf[0] = tcxoVoltage & 0x07;
    buf[1] = (uint8_t)((timeout >> 16 ) & 0xFF);
    buf[2] = (uint8_t)((timeout >> 8 ) & 0xFF);
    buf[3] = (uint8_t)(timeout & 0xFF);

    WriteCommand(SX126X_RADIO_SET_DIO3_AS_TCXO_CTRL, buf, 4);
}