#pragma once

#include "config.h"

// read text on this square thing on the board
#define CPU_MK64FN1M0VLL12

#define configTICK_RATE_HZ						1000
#define configUSE_TICKLESS_IDLE					0
#define configUSE_PREEMPTION					1
#define configUSE_QUEUE_SETS					0
#define configUSE_IDLE_HOOK						0
#define configUSE_TICK_HOOK						0
// System Clock - 120 MHz po ustawieniu, lub 21 MHz zaraz po starcie
#define configCPU_CLOCK_HZ						DEFAULT_SYSTEM_CLOCK
#define configMAX_PRIORITIES					18
#define configMINIMAL_STACK_SIZE				200
#define configTOTAL_HEAP_SIZE					0x8000
#define configMAX_TASK_NAME_LEN					12
#define configUSE_TRACE_FACILITY				0
#define configUSE_16_BIT_TICKS					0
#define configIDLE_SHOULD_YIELD					1
#define configQUEUE_REGISTRY_SIZE				0
#define configCHECK_FOR_STACK_OVERFLOW			0
#define configUSE_RECURSIVE_MUTEXES				0
#define configUSE_MALLOC_FAILED_HOOK			0
#define configUSE_APPLICATION_TASK_TAG			0
#define configUSE_COUNTING_SEMAPHORES			0
#define configSUPPORT_STATIC_ALLOCATION			0
#define configUSE_TASK_END_SCHEDULER   			0
#define configASSERT(x) if((x)==0) { taskDISABLE_INTERRUPTS(); pulsePanic(RED); }

#define configGENERATE_RUN_TIME_STATS			0
#define configUSE_STATS_FORMATTING_FUNCTIONS	0

#define configUSE_CO_ROUTINES					0
#define configMAX_CO_ROUTINE_PRIORITIES 		2

#define configUSE_TIMERS						1
#define configTIMER_TASK_PRIORITY				(configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH				10
#define configTIMER_TASK_STACK_DEPTH			(configMINIMAL_STACK_SIZE * 2)

#define INCLUDE_vTaskPrioritySet				0
#define INCLUDE_uxTaskPriorityGet				0
#define INCLUDE_vTaskDelete						0
#define INCLUDE_vTaskCleanUpResources			0
#define INCLUDE_vTaskSuspend					1
#define INCLUDE_vTaskDelayUntil					1
#define INCLUDE_vTaskDelay						1
#define INCLUDE_eTaskGetState					0
#define INCLUDE_xTimerPendFunctionCall			1


// documentation:72
#define configPRIO_BITS										4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY				10
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY		1
#define configKERNEL_INTERRUPT_PRIORITY		 				(configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY	 			(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

// generate sources without processor expert
#define configGENERATE_STATIC_SOURCES			1

// GCC
#define configCOMPILER_ARM_GCC					1
#define configCOMPILER							configCOMPILER_ARM_GCC

// ARM
#define configCPU_FAMILY_ARM_M0P             6
#define configCPU_FAMILY_ARM_M4              7
#define configCPU_FAMILY_ARM_M4F             8
#define configCPU_FAMILY_IS_ARM_M4(fam)      (((fam)==configCPU_FAMILY_ARM_M4)  || ((fam)==configCPU_FAMILY_ARM_M4F))
#define configCPU_FAMILY_IS_ARM(fam)         (((fam)==configCPU_FAMILY_ARM_M0P) || configCPU_FAMILY_IS_ARM_M4(fam))
#define configCPU_FAMILY                     configCPU_FAMILY_ARM_M4F

// clock
#define configSYSTICK_USE_CORE_CLOCK              1 
#define configSYSTICK_CLOCK_DIVIDER               1
#define configSYSTICK_CLOCK_HZ                    ((configCPU_CLOCK_HZ)/configSYSTICK_CLOCK_DIVIDER)
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()

// watchdog
#define configWDOG_ENABLE						0

// map interrupts to Port functions
#define vPortPendSVHandler PendSV_Handler
#define vPortSVCHandler SVC_Handler
#define vPortSysTickHandler SysTick_Handler

