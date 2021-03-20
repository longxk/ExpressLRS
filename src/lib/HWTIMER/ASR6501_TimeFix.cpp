#include <Arduino.h>

uint32 micros_asr6501() {
    return CySysTimerGetCount(CY_SYS_TIMER0) * 1000000 / 32768;
}

uint32 millis_asr6501() {
    return CySysTimerGetCount(CY_SYS_TIMER0) * 1000 / 32768;
}