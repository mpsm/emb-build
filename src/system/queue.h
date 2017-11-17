#ifndef __SYSTEM_QUEUE_H__
#define __SYSTEM_QUEUE_H__

#include <system/base.h>
#include <system/smphr.h>
#include <system/mutex.h>

#include <stddef.h>
#include <stdbool.h>

typedef void* queue_t;

bool queue_create(queue_t *q, size_t size, size_t elsize, const char *name);
size_t queue_elements_count(queue_t q);
bool queue_push(queue_t q, void *el, system_tick_t ticks);
bool queue_push_to_front(queue_t q, void *el, system_tick_t ticks);
bool queue_isr_push(queue_t q, void *el, bool * const woken);
bool queue_pop(queue_t q, void *el, system_tick_t ticks);
void queue_destroy(queue_t q);

#endif /* __SYSTEM_QUEUE_H__ */
