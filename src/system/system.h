#ifndef __SYSTEM_SYSTEM_H__
#define __SYSTEM_SYSTEM_H__

#include <system/base.h>
#include <stdbool.h>

/* public methods */
void system_init(void);
bool system_start(void);

void system_delay(system_tick_t);
void system_delay_ms(system_tick_t ms);

system_tick_t system_get_tick_count(void);
system_tick_t system_get_tick_ms(void);

#endif /* __SYSTEM_SYSTEM_H__ */
