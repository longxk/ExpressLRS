#pragma once

#define SX126X_XTAL_FREQ 32000000
#define SX126X_FREQ_DIV 33554432 //pow(2.0, 25.0)
#define SX126X_FREQ_STEP 0.95367431640625 //(SX126X_XTAL_FREQ / SX126X_FREQ_DIV)

typedef enum
{
    SX126X_RF_IDLE = 0x00, //!< The radio is idle
    SX126X_RF_RX_RUNNING,  //!< The radio is in reception state
    SX126X_RF_TX_RUNNING,  //!< The radio is in transmission state
    SX126X_RF_CAD,         //!< The radio is doing channel activity detection
} SX126x_RadioStates_t;

/*!
 * \brief Represents the operating mode the radio is actually running
 */
typedef enum
{
    SX126X_MODE_SLEEP = 0x00, //! The radio is in sleep mode
    SX126X_MODE_CALIBRATION,  //! The radio is in calibration mode
    SX126X_MODE_STDBY_RC,     //! The radio is in standby mode with RC oscillator
    SX126X_MODE_STDBY_XOSC,   //! The radio is in standby mode with XOSC oscillator
    SX126X_MODE_FS,           //! The radio is in frequency synthesis mode
    SX126X_MODE_RX,           //! The radio is in receive mode
    SX126X_MODE_TX,           //! The radio is in transmit mode
    SX126X_MODE_CAD           //! The radio is in channel activity detection mode
} SX126x_RadioOperatingModes_t;

/*!
 * \brief Declares the oscillator in use while in standby mode
 *
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications
 */
typedef enum
{
    SX126X_STDBY_RC = 0x00,
    SX126X_STDBY_XOSC = 0x01,
} SX126x_RadioStandbyModes_t;

/*!
 * \brief Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
 */
typedef enum
{
    SX126X_USE_LDO = 0x00,  //! Use LDO (default value)
    SX126X_USE_DCDC = 0x01, //! Use DCDC
} SX126x_RadioRegulatorModes_t;

/*!
 * \brief Represents the possible packet type (i.e. modem) used
 */
typedef enum
{
    SX126X_PACKET_TYPE_GFSK = 0x00,
    SX126X_PACKET_TYPE_LORA = 0x01,
    SX126X_PACKET_TYPE_NONE = 0x0F,
} SX126x_RadioPacketTypes_t;

typedef enum
{
    SX126X_LORA_IQ_NORMAL = 0x00,
    SX126X_LORA_IQ_INVERTED = 0x01,
} SX126x_RadioLoRaIQModes_t;

typedef enum
{
    SX126X_RADIO_CRC_OFF = 0x00, //!< No CRC in use
    SX126X_RADIO_CRC_ON = 0x01,
} SX126x_RadioCrcTypes_t;

/*!
 * \brief Represents the ramping time for power amplifier
 */
typedef enum
{
    SX126X_RADIO_RAMP_10_US = 0x00,
    SX126X_RADIO_RAMP_20_US = 0x01,
    SX126X_RADIO_RAMP_40_US = 0x02,
    SX126X_RADIO_RAMP_80_US = 0x03,
    SX126X_RADIO_RAMP_200_US = 0x04,
    SX126X_RADIO_RAMP_800_US = 0x05,
    SX126X_RADIO_RAMP_1700_US = 0x06,
    SX126X_RADIO_RAMP_3400_US = 0x07,
} SX126x_RadioRampTimes_t;

/*!
 * \brief Represents the number of symbols to be used for channel activity detection operation
 */
typedef enum
{
    SX126X_LORA_CAD_01_SYMBOL = 0x00,
    SX126X_LORA_CAD_02_SYMBOLS = 0x01,
    SX126X_LORA_CAD_04_SYMBOLS = 0x02,
    SX126X_LORA_CAD_08_SYMBOLS = 0x03,
    SX126X_LORA_CAD_16_SYMBOLS = 0x04,
} SX126x_RadioLoRaCadSymbols_t;

/*!
 * \brief Represents the possible spreading factor values in LORA packet types
 */
typedef enum
{
    SX126X_LORA_SF5 = 0x05,
    SX126X_LORA_SF6 = 0x06,
    SX126X_LORA_SF7 = 0x07,
    SX126X_LORA_SF8 = 0x08,
    SX126X_LORA_SF9 = 0x09,
    SX126X_LORA_SF10 = 0x0A,
    SX126X_LORA_SF11 = 0x0B,
    SX126X_LORA_SF12 = 0x0C,
} SX126x_RadioLoRaSpreadingFactors_t;

/*!
 * \brief Represents the bandwidth values for LORA packet type
 */
typedef enum
{
    SX126X_LORA_BW_500 = 6,
    SX126X_LORA_BW_250 = 5,
    SX126X_LORA_BW_125 = 4,
    SX126X_LORA_BW_062 = 3,
    SX126X_LORA_BW_041 = 10,
    SX126X_LORA_BW_031 = 2,
    SX126X_LORA_BW_020 = 9,
    SX126X_LORA_BW_015 = 1,
    SX126X_LORA_BW_010 = 8,
    SX126X_LORA_BW_007 = 0,
} SX126x_RadioLoRaBandwidths_t;

/*!
 * \brief Represents the coding rate values for LORA packet type
 */
typedef enum
{
    SX126X_LORA_CR_4_5 = 0x01,
    SX126X_LORA_CR_4_6 = 0x02,
    SX126X_LORA_CR_4_7 = 0x03,
    SX126X_LORA_CR_4_8 = 0x04,
} SX126x_RadioLoRaCodingRates_t;

typedef enum
{
    SX126X_LORA_PACKET_FIXED_LENGTH = 0x01,    //!< The packet is known on both sides, no header included in the packet
    SX126X_LORA_PACKET_VARIABLE_LENGTH = 0x00, //!< The packet is on variable size, header included
    SX126X_LORA_PACKET_EXPLICIT = SX126X_LORA_PACKET_VARIABLE_LENGTH,
    SX126X_LORA_PACKET_IMPLICIT = SX126X_LORA_PACKET_FIXED_LENGTH,
} SX126x_RadioLoRaPacketLengthsModes_t;

typedef enum
{
    SX126X_LORA_CRC_ON = 0x01,  //!< CRC activated
    SX126X_LORA_CRC_OFF = 0x00, //!< CRC not used
} SX126x_RadioLoRaCrcModes_t;

typedef enum RadioCommands_u
{
    SX126X_RADIO_GET_STATUS = 0xC0,
    SX126X_RADIO_GET_STATS = 0x10,
    SX126X_RADIO_GET_DEVICE_ERROR = 0x17,
    SX126X_RADIO_CLR_DEVICE_ERROR = 0x07,
    SX126X_RADIO_WRITE_REGISTER = 0x0D,
    SX126X_RADIO_READ_REGISTER = 0x1D,
    SX126X_RADIO_WRITE_BUFFER = 0x0E,
    SX126X_RADIO_READ_BUFFER = 0x1E,
    SX126X_RADIO_SET_SLEEP = 0x84,
    SX126X_RADIO_SET_STANDBY = 0x80,
    SX126X_RADIO_SET_FS = 0xC1,
    SX126X_RADIO_SET_TX = 0x83,
    SX126X_RADIO_SET_RX = 0x82,
    SX126X_RADIO_SET_RXDUTYCYCLE = 0x94,
    SX126X_RADIO_SET_CAD = 0xC5,
    SX126X_RADIO_SET_TXCONTINUOUSWAVE = 0xD1,
    SX126X_RADIO_SET_TXCONTINUOUSPREAMBLE = 0xD2,
    SX126X_RADIO_SET_PACKETTYPE = 0x8A,
    SX126X_RADIO_GET_PACKETTYPE = 0x11,
    SX126X_RADIO_SET_RFFREQUENCY = 0x86,
    SX126X_RADIO_SET_TXPARAMS = 0x8E,
    SX126X_RADIO_SET_CADPARAMS = 0x88,
    SX126X_RADIO_SET_BUFFERBASEADDRESS = 0x8F,
    SX126X_RADIO_SET_MODULATIONPARAMS = 0x8B,
    SX126X_RADIO_SET_PACKETPARAMS = 0x8C,
    SX126X_RADIO_GET_RXBUFFERSTATUS = 0x13,
    SX126X_RADIO_GET_PACKETSTATUS = 0x14,
    SX126X_RADIO_GET_RSSIINST = 0x15,
    SX126X_RADIO_SET_DIOIRQPARAMS = 0x08,
    SX126X_RADIO_GET_IRQSTATUS = 0x12,
    SX126X_RADIO_CLR_IRQSTATUS = 0x02,
    SX126X_RADIO_CALIBRATE = 0x89,
    SX126X_RADIO_SET_REGULATORMODE = 0x96,
    SX126X_RADIO_SET_PACONFIG = 0x95,
    SX126X_RADIO_SET_DIO3_AS_TCXO_CTRL = 0x97,
    SX126X_RADIO_CALIBRATE_IMAGE = 0x98,
    SX126X_RADIO_SET_DIO2_AS_RF_SWITCH = 0x9D,
} SX126x_RadioCommands_t;

typedef enum
{
    SX126X_IRQ_RADIO_NONE = 0x0000,
    SX126X_IRQ_TX_DONE = 0x0001,
    SX126X_IRQ_RX_DONE = 0x0002,
    SX126X_IRQ_PREMBLE_DETECTED = 0x0004,
    SX126X_IRQ_SYNCWORD_VALID = 0x0008,
    SX126X_IRQ_HEADER_VALID = 0x0010,
    SX126X_IRQ_HEADER_ERROR = 0x0020,
    SX126X_IRQ_CRC_ERROR = 0x0040,
    SX126X_IRQ_CAD_DONE = 0x0080,
    SX126X_IRQ_CAD_DETECTED = 0x0100,
    SX126X_IRQ_RX_TX_TIMEOUT = 0x0200,
    SX126X_IRQ_RADIO_ALL = 0xFFFF,
} SX126x_RadioIrqMasks_t;