#ifndef __SYSTEM_BASE_H__
#define __SYSTEM_BASE_H__

#if defined(HAVE_CONFIG_H)
#  include "config.h"
#endif /* defined(HAVE_CONFIG_H) */

#if SYSTEM_CONFIG_TYPE_FREERTOS == 1
#  include <FreeRTOS/FreeRTOS.h>
#  define SYSTEM_NO_WAIT                            ((portTickType)0)
#  define SYSTEM_MAX_WAIT                           (portMAX_DELAY)
#  define SYSTEM_TICK_RATE_MS                       (portTICK_RATE_MS)
#  define SYSTEM_TASK_MODIFIER                      __task
typedef portTickType system_tick_t;
#elif SYSTEM_CONFIG_TYPE_POSIX == 1
#  include <stdint.h>
#  include <stdbool.h>

#  if !defined(SYSTEM_TICK_RATE_MS)
#    define SYSTEM_TICK_RATE_MS                     (1000)
#  endif

#  if !defined(SYSTEM_CONFIG_MAX_TASKS)
#    define SYSTEM_CONFIG_MAX_TASKS                 (10)
#  endif

/* type definitions */
typedef uint32_t system_tick_t;

#  define SYSTEM_NO_WAIT                            ((uint32_t)0)
#  define SYSTEM_MAX_WAIT                           (UINT32_MAX)
#  define SYSTEM_TASK_MODIFIER
#endif

#if !defined(SYSTEM_NO_WAIT) || !defined(SYSTEM_MAX_WAIT)
#  error "Max and no wait times not defined"
#endif

#if !defined(SYSTEM_CONFIG_MAX_TASKS)
#  error "Max task count is not defined"
#endif

#endif /* __SYSTEM_BASE_H__ */
