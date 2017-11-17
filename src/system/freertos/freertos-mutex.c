#include <system/system.h>
#include <system/mutex.h>

#include <FreeRTOS/FreeRTOS.h>
#include <FreeRTOS/task.h>
#include <FreeRTOS/semphr.h>

#include <stdbool.h>

bool
mutex_init(mutex_t *m, const char *name)
{
    xSemaphoreHandle handle;

    handle = xSemaphoreCreateMutex();
    if (name != NULL) {
        vQueueAddToRegistry(handle, (signed char *)name);
    }
    *m = (mutex_t)handle;

    return handle != NULL;
}

bool
mutex_lock(mutex_t m, system_tick_t ticks)
{
    return xSemaphoreTake((xSemaphoreHandle)m, ticks) == pdTRUE;
}

bool
mutex_unlock(mutex_t m)
{
    return xSemaphoreGive((xSemaphoreHandle)m) == pdTRUE;
}
