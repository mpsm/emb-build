#include <system/base.h>
#include <system/queue.h>
#include <system/smphr.h>
#include <system/mutex.h>

#include "posix-queue.h"

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

bool
queue_create(queue_t *queue, size_t size, size_t elsize, const char *name)
{
    posix_queue_t *q = malloc(sizeof(posix_queue_t));
    bool result = false;

    *queue = (queue_t *)q;
    q->size = size;
    q->element_size = elsize;
    q->elements = 0;
    q->index = 0;

    if((q->data = malloc(size * elsize)) == NULL) {
        goto error;
    }
    
    /* TODO: create names for queue mutex and sem */
    if(!smphr_init(&q->sem, true, NULL)) {
        goto error;
    }

    if(!mutex_init(&q->mtx, NULL)) {
        goto error;
    }

    result = true;
    goto ok;

error:
    free(q);
    *queue = NULL;

ok:
    return result;
}

size_t
queue_elements_count(queue_t queue)
{
    posix_queue_t *q = (posix_queue_t *)queue;
    size_t elements;

    mutex_lock(q->mtx, SYSTEM_MAX_WAIT);
    elements = q->elements;
    mutex_unlock(q->mtx);

    return elements;
}

bool
queue_push(queue_t queue, void *el, system_tick_t ticks)
{
    posix_queue_t *q = (posix_queue_t *)queue;
    void *start;

    mutex_lock(q->mtx, SYSTEM_MAX_WAIT);
    if(q->elements == q->size) {
        mutex_unlock(q->mtx);
        return false;
    }

    start = q->data + (q->index + q->elements) % q->size * q->element_size;
    memcpy(start, el, q->element_size);
    q->elements++;
    mutex_unlock(q->mtx);
    smphr_give(q->sem);

    return true;
}

bool
queue_push_to_front(queue_t queue, void *el, system_tick_t ticks)
{
    posix_queue_t *q = (posix_queue_t *)queue;

    mutex_lock(q->mtx, SYSTEM_MAX_WAIT);
    if(q->elements == q->size) {
        mutex_unlock(q->mtx);
        return false;
    }

    memmove(q->data + q->element_size, q->data, (q->index + q->elements) % q->size * q->element_size);
    memcpy(q->data, el, q->element_size);
    q->elements++;

    mutex_unlock(q->mtx);
    smphr_give(q->sem);

    return true;
}

bool
queue_pop(queue_t queue, void *el, system_tick_t ticks)
{
    posix_queue_t *q = (posix_queue_t *)queue;
    void *start;

    if(!smphr_take(q->sem, ticks)) {
        return false;
    }

    mutex_lock(q->mtx, SYSTEM_MAX_WAIT);
    start = q->data + q->index * q->element_size;
    memcpy(el, start, q->element_size);
    q->index = (q->index + 1) % q->size;
    q->elements--;
    mutex_unlock(q->mtx);

    return true;
}

void
queue_destroy(queue_t queue)
{
    posix_queue_t *q = (posix_queue_t *)queue;

    smphr_destroy(q->sem);
    mutex_destroy(q->mtx);
    free(q->data);
    free(q);
}
