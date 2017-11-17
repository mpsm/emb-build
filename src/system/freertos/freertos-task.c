#include <system/system.h>
#include <system/task.h>
#include "freertos-system.h"

#include <FreeRTOS/FreeRTOS.h>
#include <FreeRTOS/task.h>

#include <stdint.h>
#include <stddef.h>
#include <limits.h>
#include <stdbool.h>

bool
task_create(task_t *t, task_routine_t rt, void *arg, const char * const tname,
        unsigned int prio, size_t stack_size)
{
    xTaskHandle handle;
    portBASE_TYPE result;

    /* check stack size before casting */
    if(stack_size > USHRT_MAX) {
        return false;
    }

    result = xTaskCreate((pdTASK_CODE)rt, (signed char *)tname,
        (unsigned short)stack_size, arg, tskIDLE_PRIORITY + prio, &handle);
    *t = (task_t)handle;

    return result == pdTRUE;
}

void
task_destroy(task_t t)
{
#if (INCLUDE_vTaskDelete == 1)
    if(t != NULL) {
        vTaskDelete((xTaskHandle)t);
    }
#endif /* INCLUDE_vTaskDelete == 1 */
}

system_tick_t
task_get_tick_count(void)
{
    return xTaskGetTickCount();
}

void
task_delay_ticks(system_tick_t ticks)
{
    vTaskDelay(ticks);
}

