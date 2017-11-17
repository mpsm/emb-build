#include <system/system.h>
#include <system/task.h>
#include <system/mutex.h>
#include <system/smphr.h>

#include "posix-timer.h"

#include <stdbool.h>
#include <stdlib.h>

#define TMR_CREATE_DEFAULT_PERIOD               (1)

static mutex_t tmr_psx_mtx;
static smphr_t tmr_psx_s;

static tmr_t *timer_list;

static void tmr_run(void* arg);
static bool tmr_delete_from_timer_list(tmr_t *timer_list, tmr_t delete_timer);
static bool tmr_add_to_timer_list(tmr_t *timer_list, tmr_t new_timer);

static bool tmr_stop_locked(tmr_t t);
static bool tmr_start_locked(tmr_t t, system_tick_t period);

bool 
tmr_task_init(task_t *t)
{
    posix_timer_t *tmr_list = (posix_timer_t *) malloc(sizeof(posix_timer_t));

    task_create(t, tmr_run, NULL, NULL, 0, 0);
    mutex_init(&tmr_psx_mtx, NULL);
    smphr_init(&tmr_psx_s, true, NULL);

    timer_list = (tmr_t *)tmr_list;
    tmr_list->next_timer = NULL;
    tmr_list->prev_timer = NULL;
    tmr_list->wakeup_time = 0;
    return (true);
}

void
tmr_destroy(tmr_t t)
{

    free(t);
}

void
tmr_destroy_list()
{

    mutex_destroy(tmr_psx_mtx);
    smphr_destroy(tmr_psx_s);
    free(timer_list);
}

void 
tmr_run(void *arg)
{
    tmr_t *current_timer;
    posix_timer_t *current_posix_timer;
    system_tick_t ticks_to_callback, current_ticks;

    while (true) {
        mutex_lock(tmr_psx_mtx, SYSTEM_MAX_WAIT);
        current_timer = ((posix_timer_t *)(timer_list))->next_timer;
        if (current_timer!=NULL) {
            current_posix_timer = ((posix_timer_t *)(current_timer));
            current_ticks = system_get_tick_count();
            if (current_posix_timer->wakeup_time > current_ticks) {    
                ticks_to_callback =
                            current_posix_timer->wakeup_time - current_ticks;
            } else {
                ticks_to_callback = SYSTEM_NO_WAIT;
            }
        } else {
            ticks_to_callback = SYSTEM_MAX_WAIT;
        }
        mutex_unlock(tmr_psx_mtx);

        if (!smphr_take(tmr_psx_s, ticks_to_callback)) {
            mutex_lock(tmr_psx_mtx, SYSTEM_MAX_WAIT);
            if (current_timer != NULL) {
                current_posix_timer->callback(NULL);
                tmr_stop_locked(current_timer);
                if (current_posix_timer->autoreload) {
                    tmr_start_locked(current_timer, current_posix_timer->period);
                }
            }
            mutex_unlock(tmr_psx_mtx);
        }    
    }
}

bool
tmr_init(tmr_t *t, bool autoreload, timer_callback_t callback,
                  const char *name)
{
    posix_timer_t *new_timer = (posix_timer_t *)malloc(sizeof(posix_timer_t));

    if (!new_timer) {
        return (false);
    }
    *t = (tmr_t*)new_timer;
    new_timer->callback = callback;
    new_timer->autoreload = autoreload;
    new_timer->wakeup_time = SYSTEM_MAX_WAIT;
    new_timer->is_running = (false);
    new_timer->next_timer = NULL;
    new_timer->prev_timer = NULL;
    new_timer->period = TMR_CREATE_DEFAULT_PERIOD;
    return (true);                  
}

bool 
tmr_start(tmr_t t, system_tick_t period)
{
    posix_timer_t *current_timer;

    mutex_lock(tmr_psx_mtx, SYSTEM_MAX_WAIT);
    current_timer = (posix_timer_t *)t;
    if (current_timer->is_running == (true)) {
        tmr_stop_locked(t);
    }
    current_timer->period = period;
    current_timer->wakeup_time = current_timer->period + system_get_tick_count();
    current_timer->is_running = (true);

    tmr_add_to_timer_list(timer_list, t);
    smphr_give(tmr_psx_s);
    mutex_unlock(tmr_psx_mtx);
    return (true);
}

bool 
tmr_reset(tmr_t t)
{
    posix_timer_t *current_timer;

    mutex_lock(tmr_psx_mtx, SYSTEM_MAX_WAIT);
    current_timer = (posix_timer_t *)t;
    if (!current_timer->period){
        mutex_unlock(tmr_psx_mtx);
        return (false);
    }
    mutex_unlock(tmr_psx_mtx);
    return tmr_start(t, current_timer->period);
}

bool 
tmr_stop(tmr_t t)
{
    posix_timer_t *current_timer;

    mutex_lock(tmr_psx_mtx, SYSTEM_MAX_WAIT);
    current_timer = (posix_timer_t *)t;
    if (current_timer->is_running) {
        tmr_delete_from_timer_list(timer_list, t);
        smphr_give(tmr_psx_s);
        
        current_timer->wakeup_time = 0;
        current_timer->is_running = (false);
        mutex_unlock(tmr_psx_mtx);
        return (true);
    }
    mutex_unlock(tmr_psx_mtx);
    return (true);
}

bool 
tmr_is_running(tmr_t t)
{
    posix_timer_t *current_timer;
    bool state;

    mutex_lock(tmr_psx_mtx, SYSTEM_MAX_WAIT);
    current_timer = (posix_timer_t *)t;
    state = current_timer->is_running;
    mutex_unlock(tmr_psx_mtx);
    return state;
}

bool
tmr_add_to_timer_list(tmr_t *timer_list, tmr_t new_timer)
{
    tmr_t *pointer, *next;
    posix_timer_t *current, *new;

    pointer = timer_list; 
    new = (posix_timer_t *)(new_timer);
    while (((posix_timer_t *)(pointer))->next_timer != NULL) {
        current = (posix_timer_t *)pointer;
        next = current->next_timer;    
        if (((posix_timer_t *)(next))->wakeup_time > new->wakeup_time) {
            new->prev_timer = pointer;
            new->next_timer = current->next_timer;
            current->next_timer = new_timer;
            ((posix_timer_t *)(next))->prev_timer = new_timer;
            return (true);  
        } 
        pointer = current->next_timer;
    }
    current = (posix_timer_t *)pointer;
    new->prev_timer = pointer;
    new->next_timer = NULL;
    current->next_timer = new_timer;
    return (true);
}

bool
tmr_delete_from_timer_list(tmr_t *timer_list, tmr_t delete_timer)
{
    tmr_t *pointer, *next;
    posix_timer_t *current, *delete;
    pointer = timer_list; 

    delete = (posix_timer_t *)delete_timer;
    current = (posix_timer_t *)pointer;
    while (current->next_timer != NULL) {
        current = (posix_timer_t *)pointer;
        if (current->next_timer == delete_timer) {
            next = delete->next_timer;
            current->next_timer = delete->next_timer;
            if (next != NULL){
                ((posix_timer_t *)(next))->prev_timer = delete->prev_timer;
            }
            return (true);  
        } 
        pointer = current->next_timer;
    }
    return (false);
}

bool 
tmr_start_locked(tmr_t t, system_tick_t period)
{
    posix_timer_t *current_timer;

    current_timer = (posix_timer_t *)t;
    current_timer->period = period;
    current_timer->wakeup_time = current_timer->period + system_get_tick_count();
    current_timer->is_running = (true);
    tmr_add_to_timer_list(timer_list, t);
    smphr_give(tmr_psx_s);
    return (true);
}

bool 
tmr_stop_locked(tmr_t t)
{
    posix_timer_t *current_timer;

    current_timer = (posix_timer_t *)t;
    if (current_timer->is_running) {
        tmr_delete_from_timer_list(timer_list, t);
        smphr_give(tmr_psx_s);
        
        current_timer->wakeup_time = 0;
        current_timer->is_running = (false);
        return (true);
    }
    return (true);
}
