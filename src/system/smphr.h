#ifndef __SYSTEM_SMPHR_H__
#define __SYSTEM_SMPHR_H__

#include <system/base.h>
#include <stdbool.h>

typedef void *smphr_t;

bool smphr_init(smphr_t *s, bool taken, const char *name);
bool smphr_take(smphr_t s, system_tick_t ticks);
bool smphr_give(smphr_t s);
bool smphr_isr_give(smphr_t s, bool * const woken);
void smphr_destroy(smphr_t s);

#endif /* __SYSTEM_SMPHR_H__ */
