#include <system/system.h>
#include <system/timer.h>

#include <FreeRTOS/FreeRTOS.h>
#include <FreeRTOS/timers.h>

#include <stdbool.h>
#include <stddef.h>

#define TMR_CREATE_DEFAULT_PERIOD            (1/*portMAX_DELAY*/)

bool
tmr_init(tmr_t *t, bool autoreload, timer_callback_t callback,
             const char *name)
{
    xTimerHandle handle;

    handle = xTimerCreate((signed char *)name, TMR_CREATE_DEFAULT_PERIOD,
                          autoreload, t, callback);
    if (handle == NULL) {
        return (false);
    }

    *t = (tmr_t)handle;

    return (true);
}

bool
tmr_start(tmr_t t, system_tick_t period)
{

    if (xTimerChangePeriod((xTimerHandle)t, period, SYSTEM_NO_WAIT) != pdPASS) {
        return (false);
    }

    return (xTimerStart((xTimerHandle)t, SYSTEM_NO_WAIT) == pdPASS);
}

bool
tmr_reset(tmr_t t)
{

    return (xTimerReset((xTimerHandle)t, SYSTEM_NO_WAIT) == pdPASS);
}

bool
tmr_stop(tmr_t t)
{

    return (xTimerStop((xTimerHandle)t, SYSTEM_NO_WAIT) == pdPASS);
}

bool
tmr_is_running(tmr_t t)
{

    return (xTimerIsTimerActive((xTimerHandle)t) != pdFALSE);
}
