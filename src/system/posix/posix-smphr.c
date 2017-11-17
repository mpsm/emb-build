#include <system/base.h>
#include <system/smphr.h>

#include "posix-system.h"

#include <semaphore.h>
#include <time.h>

#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <stdio.h>

bool
smphr_init(smphr_t *s, bool taken, const char *name)
{

    *s = malloc(sizeof(sem_t));

    return sem_init((sem_t *)*s, 0, taken ? 0 : 1) == 0;
}

bool
smphr_take(smphr_t s, system_tick_t ticks)
{
    struct timespec wait;

    if(ticks == SYSTEM_NO_WAIT) {
        return sem_trywait((sem_t *)s) == 0;
    }

    if(ticks == SYSTEM_MAX_WAIT) {
        return sem_wait((sem_t *)s) == 0;
    }

    system_delay_to_timespec(ticks, &wait);
    return sem_timedwait((sem_t *)s, &wait) == 0;
}

bool
smphr_give(smphr_t s)
{
    return sem_post((sem_t *)s) == 0;
}

void
smphr_destroy(smphr_t s)
{
    free((sem_t *)s);
}
