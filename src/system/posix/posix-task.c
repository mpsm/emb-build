#include <system/base.h>
#include <system/task.h>
#include "posix-system.h"

#include <pthread.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

bool
task_create(task_t *tsk, task_routine_t rt, void *arg, const char * const tname,
        unsigned int prio, size_t stack_size)
{
    posix_task_t *t = malloc(sizeof(posix_task_t));

    *tsk = (task_t)t;
    t->rt = rt;
    t->arg = arg;
    t->name = tname;
    t->prio = prio;
    t->stack_size = stack_size;
    t->handle = malloc(sizeof(pthread_t));

    return system_task_reg(t);
}

void
task_destroy(task_t t)
{
    free(((posix_task_t *)t)->handle);
    free(t);
}
