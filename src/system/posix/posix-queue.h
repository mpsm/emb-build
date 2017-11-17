#ifndef __SYSTEM_POSIX_QUEUE_H__
#define __SYSTEM_POSIX_QUEUE_H__

#include <system/base.h>
#include <system/smphr.h>
#include <system/mutex.h>

#include <stddef.h>

typedef struct {
    smphr_t sem;
    mutex_t mtx;
    size_t size;
    size_t element_size;
    size_t elements;
    size_t index;
    void *data;
} posix_queue_t;

#endif /* __SYSTEM_POSIX_QUEUE_H__ */
