#include <Arduino.h>

#include "SX126x_Regs.h"
#include "SX126x_hal.h"
#include "SX126x.h"

SX126xHal hal;
SX126xDriver *SX126xDriver::instance = NULL;

/* Steps for startup 

1. If not in STDBY_RC mode, then go to this mode by sending the command:
SetStandby(STDBY_RC)

2. Define the LoRa® packet type by sending the command:
SetPacketType(PACKET_TYPE_LORA)

3. Define the RF frequency by sending the command:
SetRfFrequency(rfFrequency)
The LSB of rfFrequency is equal to the PLL step i.e. 52e6/2^18 Hz. SetRfFrequency() defines the Tx frequency.

4. Indicate the addresses where the packet handler will read (txBaseAddress in Tx) or write (rxBaseAddress in Rx) the first
byte of the data payload by sending the command:
SetBufferBaseAddress(txBaseAddress, rxBaseAddress)
Note:
txBaseAddress and rxBaseAddress are offset relative to the beginning of the data memory map.

5. Define the modulation parameter signal BW SF CR
*/

uint32_t beginTX;
uint32_t endTX;

bool ImageCalibrated = false;

void ICACHE_RAM_ATTR SX126xDriver::nullCallback(void) {return;}

SX126xDriver::SX126xDriver()
{
    instance = this;
}

void SX126xDriver::End()
{
    hal.end();
    instance->TXdoneCallback = &nullCallback; // remove callbacks
    instance->RXdoneCallback = &nullCallback;
}

bool SX126xDriver::Begin()
{
    hal.init();
    hal.TXdoneCallback = &SX126xDriver::TXnbISR;
    hal.RXdoneCallback = &SX126xDriver::RXnbISR;

    hal.reset();
    Serial.println("SX126x Begin");
    hal.SetDio3AsTcxoCtrl(0x02, 5<<6);
    hal.WriteCommand(SX126X_RADIO_CALIBRATE, 0x7f);
    hal.WriteCommand(SX126X_RADIO_SET_DIO2_AS_RF_SWITCH, 1);
    delay(100);

    this->SetMode(SX126X_MODE_STDBY_RC);                                    //step 1 put in STDBY_RC mode
    hal.WriteCommand(SX126X_RADIO_SET_PACKETTYPE, SX126X_PACKET_TYPE_LORA); //Step 2: set packet type to LoRa
    this->ConfigModParams(currBW, currSF, currCR);                          //Step 5: Configure Modulation Params
    hal.WriteRegister(0x0740, 0x24);
    hal.WriteRegister(0x0741, 0x24);
    hal.WriteRegister(0x08ac, 0x96);
    //hal.WriteCommand(SX126X_RADIO_SET_AUTOFS, 0x01);                        //enable auto FS
    //hal.WriteRegister(0x0891, (hal.ReadRegister(0x0891) | 0xC0));           //default is low power mode, switch to high sensitivity instead
    this->SetPacketParams(12, SX126X_LORA_PACKET_IMPLICIT, 8, SX126X_LORA_CRC_OFF, SX126X_LORA_IQ_NORMAL); //default params
    this->SetFIFOaddr(0x00, 0x00);                                          //Step 4: Config FIFO addr
    this->SetDioIrqParams(SX126X_IRQ_RADIO_ALL, SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE, SX126X_IRQ_RADIO_NONE, SX126X_IRQ_RADIO_NONE); //set IRQ to both RXdone/TXdone on DIO1
    return true;
}

void ICACHE_RAM_ATTR SX126xDriver::Config(SX126x_RadioLoRaBandwidths_t bw, SX126x_RadioLoRaSpreadingFactors_t sf, SX126x_RadioLoRaCodingRates_t cr, uint32_t freq, uint8_t PreambleLength)
{
    this->SetMode(SX126X_MODE_STDBY_XOSC);
    hal.WriteCommand(SX126X_RADIO_SET_REGULATORMODE, SX126X_USE_DCDC);
    ConfigModParams(bw, sf, cr);
    SetPacketParams(PreambleLength, SX126X_LORA_PACKET_IMPLICIT, 8, SX126X_LORA_CRC_OFF, SX126X_LORA_IQ_NORMAL); // TODO don't make static etc.
    SetFrequency(freq);
}

void ICACHE_RAM_ATTR SX126xDriver::SetPaConfig(uint8_t paDutyCycle, uint8_t HpMax, uint8_t deviceSel, uint8_t paLUT)
{
    uint8_t buf[4];
    buf[0] = paDutyCycle;
    buf[1] = HpMax;
    buf[2] = deviceSel;
    buf[3] = paLUT;
    hal.WriteCommand(SX126X_RADIO_SET_PACONFIG, buf, 4);
}

void ICACHE_RAM_ATTR SX126xDriver::SetOutputPower(int8_t power)
{
    this->SetPaConfig(0x04, 0x07, 0x00, 0x01);
    if( power > 22 )
    {
        power = 22;
    }
    else if( power < -3 )
    {
        power = -3;
    }
    hal.WriteRegister(0x08E7, 0x38);

    uint8_t buf[2];
    buf[0] = power;
    buf[1] = (uint8_t)SX126X_RADIO_RAMP_10_US;
    hal.WriteCommand(SX126X_RADIO_SET_TXPARAMS, buf, 2);
    Serial.print("SetPower: ");
    Serial.println(buf[0]);
    return;
}

void SX126xDriver::SetPacketParams(uint8_t PreambleLength, SX126x_RadioLoRaPacketLengthsModes_t HeaderType, uint8_t PayloadLength, SX126x_RadioLoRaCrcModes_t crc, SX126x_RadioLoRaIQModes_t InvertIQ)
{
    uint8_t buf[6];

    buf[0] = (PreambleLength >> 8) & 0xFF;
    buf[1] = PreambleLength;
    buf[2] = HeaderType;
    buf[3] = PayloadLength;
    buf[4] = crc;
    buf[5] = InvertIQ;

    hal.WriteCommand(SX126X_RADIO_SET_PACKETPARAMS, buf, sizeof(buf));
}

void SX126xDriver::SetMode(SX126x_RadioOperatingModes_t OPmode)
{

    if (OPmode == currOpmode)
    {
       return;
    }

    WORD_ALIGNED_ATTR uint8_t buf[3];

    switch (OPmode)
    {

    case SX126X_MODE_SLEEP:
        hal.WriteCommand(SX126X_RADIO_SET_SLEEP, 0x01);
        break;

    case SX126X_MODE_CALIBRATION:
        break;

    case SX126X_MODE_STDBY_RC:
        hal.WriteCommand(SX126X_RADIO_SET_STANDBY, SX126X_STDBY_RC);
        break;
        
    case SX126X_MODE_STDBY_XOSC:
        hal.WriteCommand(SX126X_RADIO_SET_STANDBY, SX126X_STDBY_XOSC);
        break;

    case SX126X_MODE_FS:
        hal.WriteCommand(SX126X_RADIO_SET_FS);
        break;

    case SX126X_MODE_RX:
        buf[0] = 0xFF; // periodBase = 1ms, page 71 datasheet, set to FF for cont RX
        buf[1] = 0xFF;
        buf[2] = 0xFF;
        hal.WriteCommand(SX126X_RADIO_SET_RX, buf, sizeof(buf));
        break;

    case SX126X_MODE_TX:
        //uses timeout Time-out duration = periodBase * periodBaseCount
        buf[0] = 0xff; // periodBase = 1ms, page 71 datasheet
        buf[1] = 0xFF; // no timeout set for now
        buf[2] = 0xFF; // TODO dynamic timeout based on expected onairtime
        hal.WriteCommand(SX126X_RADIO_SET_TX, buf, sizeof(buf));
        break;

    case SX126X_MODE_CAD:
        break;

    default:
        break;
    }

    currOpmode = OPmode;
}

void SX126xDriver::ConfigModParams(SX126x_RadioLoRaBandwidths_t bw, SX126x_RadioLoRaSpreadingFactors_t sf, SX126x_RadioLoRaCodingRates_t cr)
{
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used

    WORD_ALIGNED_ATTR uint8_t rfparams[4] = {0};

    rfparams[0] = (uint8_t)sf;
    rfparams[1] = (uint8_t)bw;
    rfparams[2] = (uint8_t)cr;

    hal.WriteCommand(SX126X_RADIO_SET_MODULATIONPARAMS, rfparams, sizeof(rfparams));
}

void SX126xDriver::SetFrequency(uint32_t Reqfreq)
{
    //Serial.print("freq:");
    //Serial.println(Reqfreq);
    WORD_ALIGNED_ATTR uint8_t buf[4] = {0};

    if(ImageCalibrated == false)
    {
        instance->CalibrateImage(Reqfreq);
        ImageCalibrated = true;
    }

    uint32_t freq = (uint32_t)((double)Reqfreq / (double)SX126X_FREQ_STEP);
    buf[0] = (uint8_t)((freq >> 24) & 0xFF);
    buf[1] = (uint8_t)((freq >> 16) & 0xFF);
    buf[2] = (uint8_t)((freq >> 8) & 0xFF);
    buf[3] = (uint8_t)(freq & 0xFF);

    hal.WriteCommand(SX126X_RADIO_SET_RFFREQUENCY, buf, sizeof(buf));
    currFreq = Reqfreq;
}

void SX126xDriver::SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr)
{
    uint8_t buf[2];

    buf[0] = txBaseAddr;
    buf[1] = rxBaseAddr;
    hal.WriteCommand(SX126X_RADIO_SET_BUFFERBASEADDRESS, buf, sizeof(buf));
}

void SX126xDriver::SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);

    hal.WriteCommand(SX126X_RADIO_SET_DIOIRQPARAMS, buf, sizeof(buf));
}

void SX126xDriver::ClearIrqStatus(uint16_t irqMask)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irqMask & 0x00FF);

    hal.WriteCommand(SX126X_RADIO_CLR_IRQSTATUS, buf, sizeof(buf));
}

void SX126xDriver::TXnbISR()
{
    //endTX = micros();
    Serial.println("TXnbISR");
    instance->ClearIrqStatus(SX126X_IRQ_RADIO_ALL);
    instance->currOpmode = SX126X_MODE_FS; // radio goes to FS
    //Serial.print("TOA: ");
    //Serial.println(endTX - beginTX);
    //instance->GetStatus();

    // Serial.println("TXnbISR!");
    //instance->GetStatus();
    
    //instance->GetStatus();
    instance->TXdoneCallback();
}

uint8_t FIFOaddr = 0;

void SX126xDriver::TXnb(volatile uint8_t *data, uint8_t length)
{
    instance->ClearIrqStatus(SX126X_IRQ_RADIO_ALL);
    hal.TXenable(); // do first to allow PA stablise
    hal.WriteBuffer(0x00, data, length); //todo fix offset to equal fifo addr
    instance->SetMode(SX126X_MODE_TX);
    beginTX = micros();
}

void SX126xDriver::RXnbISR()
{
    //Serial.println("RXnbISR");
    instance->currOpmode = SX126X_MODE_FS;
    instance->ClearIrqStatus(SX126X_IRQ_RADIO_ALL);
    uint8_t FIFOaddr = instance->GetRxBufferAddr();
    hal.ReadBuffer(FIFOaddr, instance->RXdataBuffer, TXRXBuffSize);
    instance->GetLastPacketStats();
    instance->RXdoneCallback();
}

void SX126xDriver::RXnb()
{
    hal.RXenable();
    instance->ClearIrqStatus(SX126X_IRQ_RADIO_ALL);
    instance->SetMode(SX126X_MODE_RX);
}

uint8_t ICACHE_RAM_ATTR SX126xDriver::GetRxBufferAddr()
{
    WORD_ALIGNED_ATTR uint8_t status[2] = {0};
    hal.ReadCommand(SX126X_RADIO_GET_RXBUFFERSTATUS, status, 2);
    return status[1];
}

void ICACHE_RAM_ATTR SX126xDriver::GetStatus()
{
    uint8_t status = 0xff;

    uint8_t stat1;
    uint8_t stat2;
    bool busy;

    hal.ReadCommand(SX126X_RADIO_GET_STATUS, (uint8_t *)&status, 1);
    stat1 = (0b01110000 & status) >> 4;
    stat2 = (0b00001110 & status) >> 1;
    busy = 0b00000001 & status; // reserved, alway 0
    Serial.print("Status: ");
    Serial.print(stat1, HEX);
    Serial.print(", ");
    Serial.print(stat2, HEX);
    Serial.print(", ");
    Serial.println(busy, HEX);
}

void ICACHE_RAM_ATTR SX126xDriver::GetLastPacketStats()
{
    uint8_t status[2];

    hal.ReadCommand(SX126X_RADIO_GET_PACKETSTATUS, status, 2);
    instance->LastPacketRSSI = -(int8_t)(status[0] / 2);
    instance->LastPacketSNR = (int8_t)(status[1] / 4);
}

void ICACHE_RAM_ATTR SX126xDriver::CalibrateImage(uint32_t freq)
{
    uint8_t calFreq[2];

    if(freq > 900000000)
    {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    }
    else if(freq > 850000000)
    {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xD8;
    }
    else if(freq > 770000000)
    {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    }
    else if(freq > 460000000)
    {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    }
    else if(freq > 425000000)
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }
    hal.WriteCommand(SX126X_RADIO_CALIBRATE_IMAGE, calFreq, 2);
}
