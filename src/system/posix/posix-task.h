#ifndef __SYSTEM_POSIX_TASK_H__
#define __SYSTEM_POSIX_TASK_H__

#include <system/base.h>
#include <system/task.h>

#include <stddef.h>
#include <pthread.h>

typedef struct {
    task_routine_t rt;
    void *arg;
    const char * name;
    unsigned int prio;
    size_t stack_size;
    pthread_t *handle;
} posix_task_t;

#endif /* __SYSTEM_POSIX_TASK_H__ */
