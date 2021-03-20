#pragma once

/// General Features ///
#define FEATURE_OPENTX_SYNC //uncomment to use OpenTX packet sync feature (requires OpenTX 2.4 onwards) - this reduces latency.
#define LED_MAX_BRIGHTNESS 50 //0..255 for max led brightness
/////////////////////////

#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))

#if defined(PLATFORM_STM32) || defined(PLATFORM_ASR6501)
#define ICACHE_RAM_ATTR //nothing//
#else
#ifndef ICACHE_RAM_ATTR //fix to allow both esp32 and esp8266 to use ICACHE_RAM_ATTR for mapping to IRAM
#define ICACHE_RAM_ATTR IRAM_ATTR
#endif
#endif

#ifdef TARGET_TTGO_LORA_V1_AS_TX
#define GPIO_PIN_NSS 18
#define GPIO_PIN_BUSY           -1 // NOT USED ON THIS TARGET 
#define GPIO_PIN_DIO0 26
#define GPIO_PIN_DIO1 -1
#define GPIO_PIN_MOSI 27
#define GPIO_PIN_MISO 19
#define GPIO_PIN_SCK 5
#define GPIO_PIN_RST 14
#define GPIO_PIN_OLED_SDA 4
#define GPIO_PIN_OLED_SCK 15
#define GPIO_PIN_RCSIGNAL_RX 13
#define GPIO_PIN_RCSIGNAL_TX 13
#endif

#ifdef TARGET_TTGO_LORA_V1_AS_RX
#endif

#ifdef TARGET_TTGO_LORA_V2_AS_TX
#define GPIO_PIN_NSS 18
#define GPIO_PIN_BUSY           -1 // NOT USED ON THIS TARGET 
#define GPIO_PIN_DIO0 26
#define GPIO_PIN_DIO1 -1
#define GPIO_PIN_MOSI 27
#define GPIO_PIN_MISO 19
#define GPIO_PIN_SCK 5
#define GPIO_PIN_RST 14
#define GPIO_PIN_OLED_SDA 21
#define GPIO_PIN_OLED_SCK 22
#define GPIO_PIN_RCSIGNAL_RX 13
#define GPIO_PIN_RCSIGNAL_TX 13
#endif

#ifdef TARGET_TTGO_LORA_V2_AS_RX
 // not supported
#endif

#ifdef TARGET_EXPRESSLRS_PCB_TX_V3
#define GPIO_PIN_NSS 5
#define GPIO_PIN_BUSY           -1 // NOT USED ON THIS TARGET 
#define GPIO_PIN_DIO0 26
#define GPIO_PIN_DIO1 25
#define GPIO_PIN_MOSI 23
#define GPIO_PIN_MISO 19
#define GPIO_PIN_SCK 18
#define GPIO_PIN_RST 14
#define GPIO_PIN_RX_ENABLE 13
#define GPIO_PIN_TX_ENABLE 12
#define GPIO_PIN_RCSIGNAL_RX 2
#define GPIO_PIN_RCSIGNAL_TX 2 // so we don't have to solder the extra resistor, we switch rx/tx using gpio mux
#endif

#ifdef TARGET_EXPRESSLRS_PCB_TX_V3_LEGACY
#define GPIO_PIN_BUTTON 36
#define RC_SIGNAL_PULLDOWN 4
#endif

#ifdef TARGET_EXPRESSLRS_PCB_RX_V3
#define GPIO_PIN_NSS 15
#define GPIO_PIN_BUSY           -1 // NOT USED ON THIS TARGET 
#define GPIO_PIN_DIO0 4
#define GPIO_PIN_DIO1 5
#define GPIO_PIN_MOSI 13
#define GPIO_PIN_MISO 12
#define GPIO_PIN_SCK 14
#define GPIO_PIN_RST 2
#define GPIO_PIN_RCSIGNAL_RX -1 //only uses default uart pins so leave as -1 
#define GPIO_PIN_RCSIGNAL_TX -1
#define GPIO_PIN_LED 16
#define GPIO_PIN_LED 16
#define GPIO_PIN_BUTTON 2
#define timerOffset -1
#endif

#ifdef TARGET_R9M_RX
/*
Credit to Jacob Walser (jaxxzer) for the pinout!!!
https://github.com/jaxxzer
*/
#define GPIO_PIN_NSS            PB12
#define GPIO_PIN_BUSY           -1 // NOT USED ON THIS TARGET 
#define GPIO_PIN_DIO0           PA15
#define GPIO_PIN_DIO1           PA1 // NOT CORRECT!!! PIN STILL NEEDS TO BE FOUND BUT IS CURRENTLY UNUSED
#define GPIO_PIN_MOSI           PB15
#define GPIO_PIN_MISO           PB14
#define GPIO_PIN_SCK            PB13
#define GPIO_PIN_RST            PC14
#define GPIO_PIN_SDA            PB7
#define GPIO_PIN_SCL            PB6
#ifdef USE_R9MM_R9MINI_SBUS
    #define GPIO_PIN_RCSIGNAL_RX    PA3
    #define GPIO_PIN_RCSIGNAL_TX    PA2
#else
    #define GPIO_PIN_RCSIGNAL_RX    PA10
    #define GPIO_PIN_RCSIGNAL_TX    PA9
#endif
#ifdef TARGET_R9MX_RX
    #define GPIO_PIN_LED            PB2 // Red
    #define GPIO_PIN_LED_RED        PB2 // Red
    #define GPIO_PIN_LED_GREEN      PB3 // Green 
    #define GPIO_PIN_BUTTON         PB0  // pullup e.g. LOW when pressed
#else
    #define GPIO_PIN_LED            PC1 // Red
    #define GPIO_PIN_LED_RED        PC1 // Red
    #define GPIO_PIN_LED_GREEN      PB3 // Green 
    #define GPIO_PIN_BUTTON         PC13  // pullup e.g. LOW when pressed
#endif
#define timerOffset             2

// External pads
// #define R9m_Ch1    PA8
// #define R9m_Ch2    PA11
// #define R9m_Ch3    PA9
// #define R9m_Ch4    PA10
// #define R9m_sbus   PA2
// #define R9m_sport  PA5
// #define R9m_isport PB11

//method to set HSE and clock speed correctly//
// #if defined(HSE_VALUE)
// /* Redefine the HSE value; it's equal to 8 MHz on the STM32F4-DISCOVERY Kit */
//#undef HSE_VALUE
//#define HSE_VALUE ((uint32_t)16000000).
//#define HSE_VALUE    25000000U
// #endif /* HSE_VALUE */
//#define SYSCLK_FREQ_72MHz
#endif

#ifdef TARGET_R9M_TX

#define GPIO_PIN_RFamp_APC1           PA6  //APC2 is connected through a I2C dac and is handled elsewhere
#define GPIO_PIN_RFswitch_CONTROL     PB3  //HIGH = RX, LOW = TX

#define GPIO_PIN_NSS            PB12
#define GPIO_PIN_BUSY           -1 // NOT USED ON THIS TARGET 
#define GPIO_PIN_DIO0           PA15
#define GPIO_PIN_MOSI           PB15
#define GPIO_PIN_MISO           PB14
#define GPIO_PIN_SCK            PB13
#define GPIO_PIN_RST            PC14
#define GPIO_PIN_RX_ENABLE      GPIO_PIN_RFswitch_CONTROL
#define GPIO_PIN_TX_ENABLE      GPIO_PIN_RFamp_APC1
#define GPIO_PIN_SDA            PB7
#define GPIO_PIN_SCL            PB6
#define GPIO_PIN_RCSIGNAL_RX    PB11 // not yet confirmed
#define GPIO_PIN_RCSIGNAL_TX    PB10 // not yet confirmed
#define GPIO_PIN_LED_RED        PA11 // Red LED
#define GPIO_PIN_LED_GREEN      PA12 // Green LED
#define GPIO_PIN_BUTTON         PA8 // pullup e.g. LOW when pressed
#define GPIO_PIN_BUZZER         PB1
#define GPIO_PIN_DIP1           PA12 // dip switch 1
#define GPIO_PIN_DIP2           PA11 // dip switch 2

#define GPIO_PIN_DEBUG_RX    PA10 // confirmed
#define GPIO_PIN_DEBUG_TX    PA9 // confirmed

#define GPIO_PIN_BUZZER      PB1 // confirmed

#define BUFFER_OE               PA5  //CONFIRMED
#define GPIO_PIN_DIO1           PA1  //Not Needed, HEARTBEAT pin
#endif

#ifdef TARGET_R9M_LITE_TX

#define GPIO_PIN_RFswitch_CONTROL     PC13  // need to confirm  //HIGH = RX, LOW = TX

#define GPIO_PIN_NSS            PB12
#define GPIO_PIN_DIO0           PC15
#define GPIO_PIN_DIO1           -1    //unused for sx1280 
#define GPIO_PIN_BUSY           -1    //unused for sx1280 
#define GPIO_PIN_MOSI           PB15
#define GPIO_PIN_MISO           PB14
#define GPIO_PIN_SCK            PB13
#define GPIO_PIN_RST            PC14
#define GPIO_PIN_RX_ENABLE      PC13 //PB3 // need to confirm 
#define GPIO_PIN_SDA            PB7
#define GPIO_PIN_SCL            PB6
#define GPIO_PIN_RCSIGNAL_RX    PB11 // not yet confirmed
#define GPIO_PIN_RCSIGNAL_TX    PB10 // not yet confirmed
#define GPIO_PIN_LED_RED        PA1 // Red LED // not yet confirmed
#define GPIO_PIN_LED_GREEN      PA4 // Green LED // not yet confirmed

#define GPIO_PIN_DEBUG_RX    PA3 // confirmed
#define GPIO_PIN_DEBUG_TX    PA2 // confirmed

#define BUFFER_OE               PA5  //CONFIRMED

#endif

#ifdef TARGET_RX_ESP8266_SX1280_V1
#define GPIO_PIN_NSS 15
#define GPIO_PIN_BUSY 5
#define GPIO_PIN_DIO0 -1 // does not exist on sx1280
#define GPIO_PIN_DIO1 4
#define GPIO_PIN_MOSI 13
#define GPIO_PIN_MISO 12
#define GPIO_PIN_SCK 14
#define GPIO_PIN_RST 2
#define GPIO_PIN_RCSIGNAL_RX -1 //only uses default uart pins so leave as -1 
#define GPIO_PIN_RCSIGNAL_TX -1
#define GPIO_PIN_LED 16
#define GPIO_PIN_BUTTON 0
#define timerOffset -1
#endif

#ifdef TARGET_TX_ESP32_SX1280_V1
#define GPIO_PIN_NSS 5
#define GPIO_PIN_BUSY 21
#define GPIO_PIN_DIO0 -1 // does not exist on sx1280
#define GPIO_PIN_DIO1 4
#define GPIO_PIN_MOSI 23
#define GPIO_PIN_MISO 19
#define GPIO_PIN_SCK 18
#define GPIO_PIN_RST 14
#define GPIO_PIN_RCSIGNAL_RX 13
#define GPIO_PIN_RCSIGNAL_TX 13
#endif

#if defined(TARGET_TX_ESP32_E28_SX1280_V1) || defined(TARGET_TX_ESP32_LORA1280F27)
#define GPIO_PIN_NSS 5
#define GPIO_PIN_BUSY 21
#define GPIO_PIN_DIO0 -1
#define GPIO_PIN_DIO1 4
#define GPIO_PIN_MOSI 23
#define GPIO_PIN_MISO 19
#define GPIO_PIN_SCK 18
#define GPIO_PIN_RST 14
#define GPIO_PIN_RX_ENABLE 27
#define GPIO_PIN_TX_ENABLE 26
#define GPIO_PIN_OLED_SDA -1
#define GPIO_PIN_OLED_SCK -1
#define GPIO_PIN_RCSIGNAL_RX 13
#define GPIO_PIN_RCSIGNAL_TX 13
#endif

#ifdef TARGET_RX_CUBECELL
#define GPIO_PIN_NSS P4_3
#define GPIO_PIN_BUSY P4_7
#define GPIO_PIN_DIO0 -1 // does not exist on sx1280
#define GPIO_PIN_DIO1 P4_6
#define GPIO_PIN_MOSI P4_0
#define GPIO_PIN_MISO P4_1
#define GPIO_PIN_SCK P4_2
#define GPIO_PIN_RST P5_7
#define GPIO_PIN_RCSIGNAL_RX -1 //only uses default uart pins so leave as -1 
#define GPIO_PIN_RCSIGNAL_TX -1
#define GPIO_PIN_LED -1
#define GPIO_PIN_BUTTON -1
#define timerOffset -1
#endif