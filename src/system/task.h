#ifndef __SYSTEM_TASK_H__
#define __SYSTEM_TASK_H__

#include <system/base.h>

#include <stdbool.h>
#include <stddef.h>

/* task descriptor */
typedef void* task_t;

/* task routine */
typedef SYSTEM_TASK_MODIFIER void (*task_routine_t)(void *arg);

bool task_create(task_t *tsk, task_routine_t rt, void *arg,
        const char * const tname, unsigned int prio, size_t stack_size);
void task_destroy(task_t tsk);

#endif /* __SYSTEM_TASK_H__ */
