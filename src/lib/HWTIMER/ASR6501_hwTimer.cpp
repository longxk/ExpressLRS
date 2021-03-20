#ifdef PLATFORM_ASR6501
#include "ASR6501_hwTimer.h"
#include "CyLFClk.h"

#define US_COUNTER_VALUE 32768/1000000

void inline hwTimer::nullCallback(void){};

void (*hwTimer::callbackTick)() = &nullCallback;
void (*hwTimer::callbackTock)() = &nullCallback;

volatile uint32_t hwTimer::HWtimerInterval = TimerIntervalUSDefault;
volatile bool hwTimer::TickTock = true;
volatile int16_t hwTimer::PhaseShift = 0;
bool hwTimer::ResetNextLoop = false;
bool hwTimer::running = false;

uint32_t hwTimer::LastCallbackMicrosTick = 0;
uint32_t hwTimer::LastCallbackMicrosTock = 0;

void hwTimer::init()
{
    if (!running)
    {
        CySysClkSetTimerSource(CY_SYS_CLK_TIMER_SRC_WCO);
        CySysClkWcoSetPowerMode(CY_SYS_CLK_WCO_HPM);
        CySysTimerSetMode(CY_SYS_TIMER0, CY_SYS_TIMER_MODE_INT);
        CySysTimerSetInterruptCallback(CY_SYS_TIMER0, callback);
        CySysTimerSetMatch(CY_SYS_TIMER0, (hwTimer::HWtimerInterval >> 1) * US_COUNTER_VALUE);
        CySysTimerEnableIsr(CY_SYS_TIMER0);
        CySysTimerEnable(CY_SYS_TIMER0_MASK);
        
        ResetNextLoop = false;
        TickTock = true;
        running = true;
    }
}

void hwTimer::stop()
{
    if (running)
    {
        CySysTimerDisable(CY_SYS_TIMER0_MASK);
        running = false;
    }
}

void hwTimer::resume()
{
    if (!running)
    {
        init();
        running = true;
    }
}

void hwTimer::updateInterval(uint32_t newTimerInterval)
{
    hwTimer::HWtimerInterval = newTimerInterval;
    CySysTimerSetMatch(CY_SYS_TIMER0, (hwTimer::HWtimerInterval >> 1) * US_COUNTER_VALUE);
}

void ICACHE_RAM_ATTR hwTimer::phaseShift(int32_t newPhaseShift)
{
    return;
    
    int32_t MaxPhaseShift = hwTimer::HWtimerInterval >> 2;

    if (newPhaseShift > MaxPhaseShift)
    {
        hwTimer::PhaseShift = MaxPhaseShift;
    }
    else
    {
        hwTimer::PhaseShift = newPhaseShift;
    }

    if (newPhaseShift < -MaxPhaseShift)
    {
        hwTimer::PhaseShift = -MaxPhaseShift;
    }
    else
    {
        hwTimer::PhaseShift = newPhaseShift;
    }
}

void ICACHE_RAM_ATTR hwTimer::callback()
{
    if (!running) {
        return;
    }
    
    if (hwTimer::TickTock)
    {
        if (hwTimer::ResetNextLoop)
        {
            CySysTimerSetMatch(CY_SYS_TIMER0, (hwTimer::HWtimerInterval >> 1) * US_COUNTER_VALUE);
            hwTimer::ResetNextLoop = false;
        }

        if (hwTimer::PhaseShift > 0 || hwTimer::PhaseShift < 0)
        {
            CySysTimerSetMatch(CY_SYS_TIMER0, ((hwTimer::HWtimerInterval >> 1) + hwTimer::PhaseShift) * US_COUNTER_VALUE);
            hwTimer::ResetNextLoop = true;
            hwTimer::PhaseShift = 0;
        }

        hwTimer::LastCallbackMicrosTick = micros();
        hwTimer::callbackTick();
    }
    else
    {
        hwTimer::LastCallbackMicrosTock = micros();
        hwTimer::callbackTock();
    }
    hwTimer::TickTock = !hwTimer::TickTock;
}
#endif
