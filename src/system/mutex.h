#ifndef __SYSTEM_MUTEX_H__
#define __SYSTEM_MUTEX_H__

#include <system/base.h>
#include <system/smphr.h>

#include <stdbool.h>

typedef void* mutex_t;

bool mutex_init(mutex_t *m, const char *name);
bool mutex_lock(mutex_t m, system_tick_t t);
bool mutex_unlock(mutex_t m);
void mutex_destroy(mutex_t m);

#endif /* __SYSTEM_MUTEX_H__ */
