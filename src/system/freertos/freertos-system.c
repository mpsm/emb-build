#include <system/system.h>

#include <FreeRTOS/FreeRTOS.h>
#include <FreeRTOS/task.h>

#include <stdbool.h>

void
system_init()
{
    __no_operation();
}

bool
system_start(void)
{
	vTaskStartScheduler();
	return true;
}

void
system_delay(system_tick_t ticks)
{
    vTaskDelay(ticks);
}

void
system_delay_ms(system_tick_t ms)
{
#if (configTICK_RATE_HZ == 1000)
    vTaskDelay(ms);
#elif (configTICK_RATE_HZ == 500)
    vTaskDelay(ms >> 1)
#else
    #error "Tick rate not supported!"
#endif
}

system_tick_t
system_get_tick_count(void)
{
    return xTaskGetTickCount();
}

system_tick_t
system_get_tick_ms(void)
{
#if (configTICK_RATE_HZ == 1000)
    return xTaskGetTickCount();
#elif (configTICK_RATE_HZ == 500)
    return (xTaskGetTickCount() << 1);
#else
    #error "Tick rate not supported!"
#endif
}

