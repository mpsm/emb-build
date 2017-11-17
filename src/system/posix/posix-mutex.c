#include <system/base.h>
#include <system/smphr.h>
#include <system/mutex.h>

#include <stdbool.h>

bool
mutex_init(mutex_t *m, const char *name)
{
    return smphr_init((smphr_t *)m, false, name);
}

bool
mutex_lock(mutex_t m, system_tick_t t)
{
    return smphr_take((smphr_t)m, t);
}

bool
mutex_unlock(mutex_t m)
{
    return smphr_give((smphr_t)m);
}

void
mutex_destroy(mutex_t m)
{
    return smphr_destroy((smphr_t)m);
}
