#ifndef __SYSTEM_TIMER_H__
#define __SYSTEM_TIMER_H__

#include <system/base.h>
#include <stdbool.h>

typedef void* tmr_t;

typedef void (*timer_callback_t)(void *handle);

bool tmr_init(tmr_t *t, bool autoreload, timer_callback_t callback,
                  const char *name);
bool tmr_start(tmr_t t, system_tick_t period);
bool tmr_isr_start(tmr_t t, bool * const woken);
bool tmr_reset(tmr_t t);
bool tmr_isr_reset(tmr_t t, bool * const woken);
bool tmr_stop(tmr_t t);
bool tmr_isr_stop(tmr_t t, bool * const woken);
bool tmr_is_running(tmr_t t);

void tmr_destroy(tmr_t t);

#endif /* __SYSTEM_TIMER_H__ */
