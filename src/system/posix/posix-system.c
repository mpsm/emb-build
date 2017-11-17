#include <system/base.h>
#include <system/system.h>
#include <system/task.h>

#include <time.h>
#include <pthread.h>
#include <signal.h>

#include <stdbool.h>
#include <string.h>

#include "posix-system.h"
#include "posix-task.h"
#include "posix-timer.h"

#define NSEC_IN_SEC (1000000000L)

static unsigned int task_count = 0;
static posix_task_t *tasks[SYSTEM_CONFIG_MAX_TASKS];
static void *posix_task_wrapper(void *arg);

#if (SYSTEM_CONFIG_USE_TIMERS == 1)
static task_t timer_task;
#endif

void
system_init()
{
    memset(tasks, 0, sizeof(tasks));
#if (SYSTEM_CONFIG_USE_TIMERS == 1)
    tmr_task_init(&timer_task);
#endif
}

void
system_ticks_to_timespec(system_tick_t ticks, struct timespec *ts)
{
    ts->tv_sec = ticks / SYSTEM_TICK_RATE_MS;
    ticks %= SYSTEM_TICK_RATE_MS;
    ts->tv_nsec = NSEC_IN_SEC / SYSTEM_TICK_RATE_MS * ticks;
}

void
system_delay_to_timespec(system_tick_t delay, struct timespec *ts)
{
    struct timespec ct;

    clock_gettime(CLOCK_REALTIME, &ct);
    system_ticks_to_timespec(delay, ts);

    ts->tv_nsec += ct.tv_nsec;
    ts->tv_sec += ct.tv_sec;
    ts->tv_sec += ts->tv_nsec / NSEC_IN_SEC;
    ts->tv_nsec %= NSEC_IN_SEC;
}

void
system_delay(system_tick_t ticks)
{
    struct timespec delay;

    system_ticks_to_timespec(ticks, &delay);
    nanosleep(&delay, NULL);
}

system_tick_t
system_get_tick_count(void)
{
    struct timespec ct;

    clock_gettime(CLOCK_REALTIME, &ct);
    return (ct.tv_sec * SYSTEM_TICK_RATE_MS)
        + (ct.tv_nsec / (NSEC_IN_SEC / SYSTEM_TICK_RATE_MS));
}

bool
system_task_reg(posix_task_t *t)
{
    if(task_count < SYSTEM_CONFIG_MAX_TASKS) {
        tasks[task_count] = t;
        task_count++;
        return true;
    }

    return false;
}

bool
system_start()
{
    sigset_t sigs;
    unsigned int i = 0;
    int signum;
    bool retval = true;

    /* create full signal set */
    sigfillset(&sigs);
    pthread_sigmask(SIG_BLOCK, &sigs, NULL);

    /* TODO: set thread stack size */
    /* TODO: set thread priority */

    /* spawn threads */
    for(i = 0; i < task_count; ++i) {
        posix_task_t *t = tasks[i];
        if(pthread_create((pthread_t *)t->handle, NULL, posix_task_wrapper, t) != 0) {
            return false;
        } 
    }

    /* wait for a signal */
    if(sigwait(&sigs, &signum) != 0) {
        return false;
    }

    /* cancel all running threads */
    for(i = 0; i < task_count; ++i) {
        retval &= pthread_cancel(*(pthread_t *)tasks[i]->handle) == 0;
        retval &= pthread_join(*(pthread_t *)tasks[i]->handle, NULL);
        task_destroy(tasks[i]);
    }

#if (SYSTEM_CONFIG_USE_TIMERS == 1)
    tmr_destroy_list();
#endif

    return retval;
}

static void *
posix_task_wrapper(void *arg)
{
    posix_task_t *t = (posix_task_t *)arg;
    
    /* call actual task */
    t->rt(t->arg);

    return NULL;
}
