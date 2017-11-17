#ifndef __SYSTEM_POSIX_TIMER_H__
#define __SYSTEM_POSIX_TIMER_H__

#include <system/base.h>
#include <system/task.h>
#include "timer.h"
#include <stdbool.h>

typedef struct {
    tmr_t *next_timer;
    tmr_t *prev_timer;
    
    timer_callback_t callback;
    system_tick_t period;
    system_tick_t wakeup_time;
    bool autoreload;
    bool is_running;
 
} posix_timer_t;

bool tmr_task_init(task_t *t);
void tmr_destroy_list();

#endif /* __SYSTEM_POSIX_TIMER_H__ */
