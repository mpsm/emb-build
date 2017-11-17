/*
    FreeRTOS V8.2.0 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*-----------------------------------------------------------
 * FreeRTOS for 56800EX port by Richy Ye in Jan. 2013.
 *----------------------------------------------------------*/
/* Scheduler includes. */
#include "FreeRTOSConfig.h"

#include <FreeRTOS/FreeRTOS.h>
#include <FreeRTOS/task.h>

#include "portmacro.h"
#include "portTicks.h" /* for CPU_CORE_CLK_HZ used in configSYSTICK_CLOCK_HZ */
#if configSYSTICK_USE_LOW_POWER_TIMER
  #include "LPTMR_PDD.h" /* PDD interface to low power timer */
  #include "SIM_PDD.h"   /* PDD interface to system integration module */
#endif
/* --------------------------------------------------- */
/* Let the user override the pre-loading of the initial LR with the address of
   prvTaskExitError() in case is messes up unwinding of the stack in the
   debugger. */
#ifdef configTASK_RETURN_ADDRESS
  #define portTASK_RETURN_ADDRESS   configTASK_RETURN_ADDRESS
#else
  #define portTASK_RETURN_ADDRESS   prvTaskExitError
#endif
/* --------------------------------------------------- */
/* macros dealing with tick counter */
#if configSYSTICK_USE_LOW_POWER_TIMER
  #define ENABLE_TICK_COUNTER()       LPTMR_PDD_EnableDevice(LPTMR0_BASE_PTR, PDD_ENABLE); LPTMR_PDD_EnableInterrupt(LPTMR0_BASE_PTR)
  #define DISABLE_TICK_COUNTER()      LPTMR_PDD_EnableDevice(LPTMR0_BASE_PTR, PDD_DISABLE); LPTMR_PDD_DisableInterrupt(LPTMR0_BASE_PTR)
  #define RESET_TICK_COUNTER_VAL()    DISABLE_TICK_COUNTER()  /* CNR is reset when the LPTMR is disabled or counter register overflows */
  #define ACKNOWLEDGE_TICK_ISR()      LPTMR_PDD_ClearInterruptFlag(LPTMR0_BASE_PTR)
#else
  #define ENABLE_TICK_COUNTER()       portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT
  #define DISABLE_TICK_COUNTER()      portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT
  #define RESET_TICK_COUNTER_VAL()    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0 /*portNVIC_SYSTICK_LOAD_REG*/
  #define ACKNOWLEDGE_TICK_ISR()      /* not needed */
#endif

typedef unsigned long TickCounter_t; /* enough for 24 bit Systick */
#if configSYSTICK_USE_LOW_POWER_TIMER
  #define TICK_NOF_BITS               16
  #define COUNTS_UP                   1 /* LPTMR is counting up */
  #define SET_TICK_DURATION(val)      LPTMR_PDD_WriteCompareReg(LPTMR0_BASE_PTR, val)
  #define GET_TICK_DURATION()         LPTMR_PDD_ReadCompareReg(LPTMR0_BASE_PTR)
  #define GET_TICK_CURRENT_VAL(addr)  *(addr)=LPTMR_PDD_ReadCounterReg(LPTMR0_BASE_PTR)
#else
  #define TICK_NOF_BITS               24
  #define COUNTS_UP                   0 /* SysTick is counting down to zero */
  #define SET_TICK_DURATION(val)      portNVIC_SYSTICK_LOAD_REG = val
  #define GET_TICK_DURATION()         portNVIC_SYSTICK_LOAD_REG
  #define GET_TICK_CURRENT_VAL(addr)  *(addr)=portNVIC_SYSTICK_CURRENT_VALUE_REG
#endif

#if configSYSTICK_USE_LOW_POWER_TIMER
  #define TIMER_COUNTS_FOR_ONE_TICK     (configSYSTICK_LOW_POWER_TIMER_CLOCK_HZ/configTICK_RATE_HZ)
#else
  #define TIMER_COUNTS_FOR_ONE_TICK     (configSYSTICK_CLOCK_HZ/configTICK_RATE_HZ)
#endif

#if configUSE_TICKLESS_IDLE == 1
#define UL_TIMER_COUNTS_FOR_ONE_TICK  ((TickCounter_t)(TIMER_COUNTS_FOR_ONE_TICK))

#if configCPU_FAMILY_IS_ARM(configCPU_FAMILY)
  #define TICKLESS_DISABLE_INTERRUPTS()  __asm volatile("cpsid i") /* disable interrupts. Note that the wfi (wait for interrupt) instruction later will still be able to wait for interrupts! */
  #define TICKLESS_ENABLE_INTERRUPTS()   __asm volatile("cpsie i") /* re-enable interrupts. */
#elif (configCPU_FAMILY==configCPU_FAMILY_S08) || (configCPU_FAMILY==configCPU_FAMILY_S12)
  #define TICKLESS_DISABLE_INTERRUPTS()  __asm("sei"); /* disable interrupts */
  #define TICKLESS_ENABLE_INTERRUPTS()   __asm("cli"); /* re-enable interrupts */
#else
  #define TICKLESS_DISABLE_INTERRUPTS()  portDISABLE_INTERRUPTS() /* this disables interrupts! Make sure they are re-enabled in vOnPreSleepProcessing()! */
  #define TICKLESS_ENABLE_INTERRUPTS()   portENABLE_INTERRUPTS()  /* re-enable interrupts */
#endif

  #if 1
    #if configSYSTICK_USE_LOW_POWER_TIMER
      /* using Low Power Timer */
      #define TICK_INTERRUPT_HAS_FIRED()   (LPTMR_PDD_GetInterruptFlag(LPTMR0_BASE_PTR)!=0)  /* returns TRUE if tick interrupt had fired */
      #define TICK_INTERRUPT_FLAG_RESET()  /* not needed */
      #define TICK_INTERRUPT_FLAG_SET()    /* not needed */
    #else
      /* using directly SysTick Timer */
      #define TICK_INTERRUPT_HAS_FIRED()   ((portNVIC_SYSTICK_CTRL_REG&portNVIC_SYSTICK_COUNT_FLAG_BIT)!=0)  /* returns TRUE if tick interrupt had fired */
      #define TICK_INTERRUPT_FLAG_RESET()  /* not needed */
      #define TICK_INTERRUPT_FLAG_SET()    /* not needed */
    #endif
  #else 
    /* using global variable to find out if interrupt has fired */
    volatile uint8_t portTickCntr; /* used to find out if we woke up by the tick interrupt */
    #define TICK_INTERRUPT_HAS_FIRED()   (portTickCntr!=0)  /* returns TRUE if tick interrupt had fired */
    #define TICK_INTERRUPT_FLAG_RESET()  portTickCntr=0
    #define TICK_INTERRUPT_FLAG_SET()    portTickCntr=1
  #endif
#endif /* configUSE_TICKLESS_IDLE == 1 */

/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * resolution of the tick timer.
 */
#if configUSE_TICKLESS_IDLE == 1
  static TickCounter_t xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * Compensate for the CPU cycles that pass while the tick timer is stopped (low
 * power functionality only).
 */
#if configUSE_TICKLESS_IDLE == 1
  static TickCounter_t ulStoppedTimerCompensation = 0; /* number of timer ticks to compensate */
  #define configSTOPPED_TIMER_COMPENSATION    45UL  /* number of CPU cycles to compensate. ulStoppedTimerCompensation will contain the number of timer ticks. */
#endif /* configUSE_TICKLESS_IDLE */

/* Flag indicating that the tick counter interval needs to be restored back to
 * the normal setting. Used when woken up from a low power mode using the LPTMR.
 */
#if (configUSE_TICKLESS_IDLE == 1) && configSYSTICK_USE_LOW_POWER_TIMER
  static uint8_t restoreTickInterval = 0; /* used to flag in tick ISR that compare register needs to be reloaded */
#endif

#if (configCPU_FAMILY==configCPU_FAMILY_CF1) || (configCPU_FAMILY==configCPU_FAMILY_CF2)
  #define portINITIAL_FORMAT_VECTOR           ((portSTACK_TYPE)0x4000)
  #define portINITIAL_STATUS_REGISTER         ((portSTACK_TYPE)0x2000)  /* Supervisor mode set. */
#endif

#if configCPU_FAMILY_IS_ARM(configCPU_FAMILY)
/* Constants required to manipulate the core.
 * SysTick register: http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0662b/CIAGECDD.html
 * Registers first... 
 */

#define portNVIC_SYSTICK_CTRL_REG           (*((volatile unsigned long *)0xe000e010)) /* SYST_CSR, SysTick Control and Status Register */
#define portNVIC_SYSTICK_LOAD_REG           (*((volatile unsigned long *)0xe000e014)) /* SYST_RVR, SysTick reload value register */
#define portNVIC_SYSTICK_CURRENT_VALUE_REG  (*((volatile unsigned long *)0xe000e018)) /* SYST_CVR, SysTick current value register */
#define portNVIC_SYSTICK_CALIB_VALUE_REG    (*((volatile unsigned long *)0xe000e01C)) /* SYST_CALIB, SysTick calibration value register */
/* ...then bits in the registers. */
#define portNVIC_SYSTICK_COUNT_FLAG_BIT     (1UL<<16UL) /* returns 1 if timer counted to 0 since the last read of the register */
#if configSYSTICK_USE_CORE_CLOCK
  #define portNVIC_SYSTICK_CLK_BIT          (1UL<<2UL) /* clock source. 1: core clock, 0: external reference clock */
#else
  #define portNVIC_SYSTICK_CLK_BIT          (0UL<<2UL) /* clock source. 1: core clock, 0: external reference clock */
#endif
#define portNVIC_SYSTICK_INT_BIT            (1UL<<1UL)  /* SysTick interrupt enable bit */
#define portNVIC_SYSTICK_ENABLE_BIT         (1UL<<0UL)  /* SysTick enable bit */

/* Constants required to manipulate the NVIC: */
#define portNVIC_INT_CTRL                   ((volatile unsigned long*)0xe000ed04) /* interrupt control and state register (ICSR) */
#define portNVIC_PENDSVCLEAR_BIT            (1UL<<27UL) /* bit 27 in portNVIC_INT_CTRL (PENDSVCLR) */
#define portNVIC_PEND_SYSTICK_SET_BIT       (1UL<<26UL) /* bit 26 in portNVIC_INT_CTRL (PENDSTSET) */
#define portNVIC_PEND_SYSTICK_CLEAR_BIT     (1UL<<25UL) /* bit 25 in portNVIC_INT_CTRL (PENDSTCLR) */

#define portNVIC_SYSPRI2                    ((volatile unsigned long*)0xe000ed1c) /* system handler priority register 2 (SHPR2), used for SVCall priority, http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0662b/CIAGECDD.html */
#define portNVIC_SVCALL_PRI                 (((unsigned long)configKERNEL_INTERRUPT_PRIORITY)<<24) /* priority of SVCall interrupt (in portNVIC_SYSPRI2) */

#define portNVIC_SYSPRI3                    ((volatile unsigned long*)0xe000ed20) /* system handler priority register 3 (SHPR3), used for SysTick and PendSV priority, http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0662b/CIAGECDD.html */
#define portNVIC_SYSTICK_PRI                (((unsigned long)configKERNEL_INTERRUPT_PRIORITY)<<24) /* priority of SysTick interrupt (in portNVIC_SYSPRI3) */
#define portNVIC_PENDSV_PRI                 (((unsigned long)configKERNEL_INTERRUPT_PRIORITY)<<16) /* priority of PendableService interrupt (in portNVIC_SYSPRI3) */
#define portNVIC_SVC_PRI						( ( ( uint32_t ) configMAX_SYSCALL_INTERRUPT_PRIORITY - 1UL ) << 24UL )

#define portNVIC_SYSPRI7                    ((volatile unsigned long*)0xe000e41c) /* system handler priority register 7, PRI_28 is LPTMR */
#define portNVIC_LP_TIMER_PRI               (((unsigned long)configKERNEL_INTERRUPT_PRIORITY)<<0) /* priority of low power timer interrupt */

#if configSYSTICK_USE_LOW_POWER_TIMER
#define IRQn_Type int
#define __NVIC_PRIO_BITS          configPRIO_BITS
#define     __O     volatile             /*!< Defines 'write only' permissions                */
#define     __IO    volatile             /*!< Defines 'read / write' permissions              */
/** \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
#if (configCPU_FAMILY_IS_ARM_M4(configCPU_FAMILY))
typedef struct
{
  __IO uint32_t ISER[8];                 /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register           */
       uint32_t RESERVED0[24];
  __IO uint32_t ICER[8];                 /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register         */
       uint32_t RSERVED1[24];
  __IO uint32_t ISPR[8];                 /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register          */
       uint32_t RESERVED2[24];
  __IO uint32_t ICPR[8];                 /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register        */
       uint32_t RESERVED3[24];
  __IO uint32_t IABR[8];                 /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register           */
       uint32_t RESERVED4[56];
  __IO uint8_t  IP[240];                 /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
       uint32_t RESERVED5[644];
  __O  uint32_t STIR;                    /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register     */
}  NVIC_Type;
#else /* M0+ */ 
typedef struct
{
  __IO uint32_t ISER[1];                 /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register           */
       uint32_t RESERVED0[31];
  __IO uint32_t ICER[1];                 /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register          */
       uint32_t RSERVED1[31];
  __IO uint32_t ISPR[1];                 /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register           */
       uint32_t RESERVED2[31];
  __IO uint32_t ICPR[1];                 /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register         */
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  __IO uint32_t IP[8];                   /*!< Offset: 0x300 (R/W)  Interrupt Priority Register              */
}  NVIC_Type;
#endif

/* Memory mapping of Cortex-M0+ Hardware */
#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address                 */
#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct          */

/* Interrupt Priorities are WORD accessible only under ARMv6M                   */
/* The following MACROS handle generation of the register offset and byte masks */
#define _BIT_SHIFT(IRQn)         (  (((uint32_t)(IRQn)       )    &  0x03) * 8 )
#define _IP_IDX(IRQn)            (   ((uint32_t)(IRQn)            >>    2)     )

/** \brief  Set Interrupt Priority
    The function sets the priority of an interrupt.
    \note The priority cannot be set for every core interrupt.
    \param [in]      IRQn  Interrupt number.
    \param [in]  priority  Priority to set.
 */
static void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority) {
  IRQn -= 16; /* PEx starts numbers with zero, while system interrupts would be negative */
#if (configCPU_FAMILY_IS_ARM_M4(configCPU_FAMILY))
  NVIC->IP[(uint32_t)(IRQn)] = ((priority << (8 - __NVIC_PRIO_BITS)) & 0xff);   /* set Priority for device specific Interrupts  */
#else /* M0+ */
  NVIC->IP[_IP_IDX(IRQn)] = (NVIC->IP[_IP_IDX(IRQn)] & ~(0xFF << _BIT_SHIFT(IRQn))) |
      (((priority << (8 - __NVIC_PRIO_BITS)) & 0xFF) << _BIT_SHIFT(IRQn)); /* set Priority for device specific Interrupts  */
#endif
}

/** \brief  Enable External Interrupt
    The function enables a device-specific interrupt in the NVIC interrupt controller.
    \param [in]      IRQn  External interrupt number. Value cannot be negative.
 */
static void NVIC_EnableIRQ(IRQn_Type IRQn) {
  IRQn -= 16; /* PEx starts numbers with zero, while system interrupts would be negative */
#if (configCPU_FAMILY_IS_ARM_M4(configCPU_FAMILY))
  NVIC->ISER[(uint32_t)((int32_t)IRQn) >> 5] = (uint32_t)(1 << ((uint32_t)((int32_t)IRQn) & (uint32_t)0x1F)); /* enable interrupt */
#else /* M0+ */
  NVIC->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F)); /* enable interrupt */
#endif
}
#endif /* configSYSTICK_USE_LOW_POWER_TIMER */

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR         (0x01000000)
#define portINITIAL_EXEC_RETURN  (0xfffffffd)
/* Constants required to check the validity of an interrupt priority. */

#define portNVIC_IP_REGISTERS_OFFSET_16 	( 0xE000E3F0 )
#define portAIRCR_REG						( * ( ( volatile uint32_t * ) 0xE000ED0C ) )
#define portMAX_8_BIT_VALUE					( ( uint8_t ) 0xff )
#define portTOP_BIT_OF_BYTE					( ( uint8_t ) 0x80 )
#define portMAX_PRIGROUP_BITS				( ( uint8_t ) 7 )
#define portPRIORITY_GROUP_MASK				( 0x07UL << 8UL )
#define portPRIGROUP_SHIFT					( 8UL )

#if (configCPU_FAMILY==configCPU_FAMILY_ARM_M4F)
  /* Constants required to manipulate the VFP. */
  #define portFPCCR                ((volatile unsigned long *)0xe000ef34) /* Floating point context control register. */
  #define portASPEN_AND_LSPEN_BITS (0x3UL<<30UL)
#endif
#endif
/* Used to keep track of the number of nested calls to taskENTER_CRITICAL().
   This will be set to 0 prior to the first task being started. */
/* Each task maintains its own interrupt status in the critical nesting variable. */
static unsigned portBASE_TYPE uxCriticalNesting = 0xaaaaaaaa;

#if configUSE_TASK_END_SCHEDULER
#include <setjmp.h>
static jmp_buf xJumpBuf; /* Used to restore the original context when the scheduler is ended. */
#endif
/*-----------------------------------------------------------*/
void prvTaskExitError(void) {
  /* A function that implements a task must not exit or attempt to return to
  its caller as there is nothing to return to.  If a task wants to exit it
  should instead call vTaskDelete( NULL ).

  Artificially force an assert() to be triggered if configASSERT() is
  defined, then stop here so application writers can catch the error. */
  configASSERT(uxCriticalNesting == ~0UL);
  portDISABLE_INTERRUPTS();
  for(;;) {
    /* wait here */
  }
}
/*-----------------------------------------------------------*/
#if (configCOMPILER==configCOMPILER_ARM_KEIL) && configCPU_FAMILY_IS_ARM_M4(configCPU_FAMILY)
__asm uint32_t ulPortSetInterruptMask(void) {
  PRESERVE8

  mrs r0, basepri
  mov r1, #configMAX_SYSCALL_INTERRUPT_PRIORITY
  msr basepri, r1
  bx r14
}
#endif /* (configCOMPILER==configCOMPILER_ARM_KEIL) */
/*-----------------------------------------------------------*/
#if (configCOMPILER==configCOMPILER_ARM_KEIL) && configCPU_FAMILY_IS_ARM_M4(configCPU_FAMILY)
__asm void vPortClearInterruptMask(uint32_t ulNewMask) {
  PRESERVE8

  msr basepri, r0
  bx r14
}
#endif /* (configCOMPILER==configCOMPILER_ARM_KEIL) */
/*-----------------------------------------------------------*/
portSTACK_TYPE *pxPortInitialiseStack(portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters ) {
  /* Simulate the stack frame as it would be created by a context switch interrupt. */
  pxTopOfStack--; /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts,
                        and to ensure alignment. */
  *pxTopOfStack = portINITIAL_XPSR;   /* xPSR */
  pxTopOfStack--;
  *pxTopOfStack = (portSTACK_TYPE)pxCode;  /* PC */
  pxTopOfStack--;
  *pxTopOfStack = (portSTACK_TYPE)portTASK_RETURN_ADDRESS;  /* LR */

  /* Save code space by skipping register initialization. */
  pxTopOfStack -= 5;  /* R12, R3, R2 and R1. */
  *pxTopOfStack = (portSTACK_TYPE)pvParameters; /* R0 */

#if configCPU_FAMILY==configCPU_FAMILY_ARM_M4F /* floating point unit */
  /* A save method is being used that requires each task to maintain its
     own exec return value. */
  pxTopOfStack--;
  *pxTopOfStack = portINITIAL_EXEC_RETURN;
#endif
  pxTopOfStack -= 8;  /* R11, R10, R9, R8, R7, R6, R5 and R4. */
 
  return pxTopOfStack;
}

/*-----------------------------------------------------------*/
#if (configCOMPILER==configCOMPILER_S08_FSL) || (configCOMPILER==configCOMPILER_S12_FSL)
#if (configCOMPILER==configCOMPILER_S08_FSL)
  #pragma MESSAGE DISABLE C1404 /* return expected */
  #pragma MESSAGE DISABLE C20000 /* dead code detected */
  #pragma NO_RETURN
  #pragma CODE_SEG __NEAR_SEG NON_BANKED
#elif (configCOMPILER==configCOMPILER_S12_FSL)
  #pragma MESSAGE DISABLE C1404 /* return expected */
  #pragma NO_RETURN
#endif

static portBASE_TYPE xBankedStartScheduler(void) {
  /* Restore the context of the first task. */
  portRESTORE_CONTEXT(); /* Simulate the end of an interrupt to start the scheduler off. */
  /* Should not get here! */
  return pdFALSE;
}

#if (configCOMPILER==configCOMPILER_S08_FSL)
  #pragma CODE_SEG DEFAULT
  #pragma MESSAGE DEFAULT C1404 /* return expected */
  #pragma MESSAGE DEFAULT C20000 /* dead code detected */
#elif (configCOMPILER==configCOMPILER_S12_FSL)
  #pragma MESSAGE DEFAULT C1404 /* return expected */
#endif
#endif
/*-----------------------------------------------------------*/
#if configUSE_TICKLESS_IDLE == 1
#if (configCOMPILER==configCOMPILER_ARM_GCC) || (configCOMPILER==configCOMPILER_ARM_KEIL)
__attribute__((weak))
#endif
void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime) {
  unsigned long ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickIncrements;
  TickCounter_t tmp; /* because of how we get the current tick counter */
  bool tickISRfired;
  uint32_t tickDuration;
  
#if configSYSTICK_USE_LOW_POWER_TIMER
  /* if we wait for the tick interrupt, do not enter low power again below */
  if (restoreTickInterval!=0) {
    /* default wait/sleep code */
    __asm volatile("dsb");
    __asm volatile("wfi");
    __asm volatile("isb");
    return;
  }
#endif

  /* Make sure the tick timer reload value does not overflow the counter. */
  if(xExpectedIdleTime > xMaximumPossibleSuppressedTicks) {
    xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
  }

  /* Stop the tick timer momentarily. The time the counter is stopped for
   * is accounted for as best it can be, but using the tickless mode will
   * inevitably result in some tiny drift of the time maintained by the
   * kernel with respect to calendar time. 
   */
#if configSYSTICK_USE_LOW_POWER_TIMER
  /* disabling the LPTMR does reset the timer register! So I need to get the value first, then disable the timer: */
  GET_TICK_CURRENT_VAL(&tmp);
  DISABLE_TICK_COUNTER();
#else /* using normal timer or SysTick */
  DISABLE_TICK_COUNTER();
  GET_TICK_CURRENT_VAL(&tmp);
#endif
  /* Calculate the reload value required to wait xExpectedIdleTime
   * tick periods. This code will execute part way through one
   * of the tick periods.
   */
  /* -1UL is used because this code will execute part way through one of the tick periods */
#if COUNTS_UP
  ulReloadValue = (UL_TIMER_COUNTS_FOR_ONE_TICK*xExpectedIdleTime);
  #if configSYSTICK_USE_LOW_POWER_TIMER
  if (ulReloadValue > 0) { /* make sure it does not underflow */
    ulReloadValue -= 1UL; /* LPTMR: interrupt will happen at match of compare register && increment, thus minus 1 */
  }
  #endif
  if (tmp!=0 && ulReloadValue>=tmp) { /* make sure it does not underflow */
    ulReloadValue -= tmp; /* take into account what we already executed in the current tick period */
  }
#else
  ulReloadValue = tmp+(UL_TIMER_COUNTS_FOR_ONE_TICK*(xExpectedIdleTime-1UL));
#endif
  if (ulStoppedTimerCompensation!=0 && ulReloadValue>ulStoppedTimerCompensation) {
    ulReloadValue -= ulStoppedTimerCompensation;
  }

  /* Enter a critical section but don't use the taskENTER_CRITICAL()
   * method as that will mask interrupts that should exit sleep mode. 
   */
  TICKLESS_DISABLE_INTERRUPTS();
  
  /* If a context switch is pending or a task is waiting for the scheduler
   * to be unsuspended then abandon the low power entry. 
   */
  if (eTaskConfirmSleepModeStatus()==eAbortSleep) {
     /* Must restore the duration before re-enabling the timers */
#if COUNTS_UP
    #if configSYSTICK_USE_LOW_POWER_TIMER
    tickDuration = UL_TIMER_COUNTS_FOR_ONE_TICK-1UL; /* LPTMR: interrupt will happen at match of compare register && increment, thus minus 1 */
    #else
    tickDuration = UL_TIMER_COUNTS_FOR_ONE_TICK;
    #endif
    if (tmp!=0 && tickDuration >= tmp) { /* make sure it does not underflow */
      tickDuration -= tmp; /* take into account what we already executed in the current tick period */
    }
#else
    tickDuration = tmp;
#endif
    SET_TICK_DURATION(tickDuration);
    ENABLE_TICK_COUNTER(); /* Restart tick timer. */
    TICKLESS_ENABLE_INTERRUPTS();
  } else {
    SET_TICK_DURATION(ulReloadValue); /* Set the new reload value. */
    RESET_TICK_COUNTER_VAL(); /* Reset the counter. */
    ENABLE_TICK_COUNTER(); /* Restart tick timer. */
    TICK_INTERRUPT_FLAG_RESET(); /* reset flag so we know later if it has fired */

    /* Sleep until something happens. configPRE_SLEEP_PROCESSING() can
     * set its parameter to 0 to indicate that its implementation contains
     * its own wait for interrupt or wait for event instruction, and so wfi
     * should not be executed again.  However, the original expected idle
     * time variable must remain unmodified, so a copy is taken.
     */

     /* CPU *HAS TO WAIT* in the sequence below for an interrupt. If vOnPreSleepProcessing() is not used, a default implementation is provided */
    /* default wait/sleep code */
    __asm volatile("dsb");
    __asm volatile("wfi");
    __asm volatile("isb");
    /* ----------------------------------------------------------------------------
     * Here the CPU *HAS TO BE* low power mode, waiting to wake up by an interrupt 
     * ----------------------------------------------------------------------------*/
    /* Stop tick counter. Again, the time the tick counter is stopped for is
     * accounted for as best it can be, but using the tickless mode will
     * inevitably result in some tiny drift of the time maintained by the
     * kernel with respect to calendar time. 
     */
    tickISRfired = (bool)TICK_INTERRUPT_HAS_FIRED(); /* need to check Interrupt flag here, as might be modified below */
#if configSYSTICK_USE_LOW_POWER_TIMER
    /* disabling the LPTMR does reset the timer register! So I need to get the value first, then disable the timer: */
    GET_TICK_CURRENT_VAL(&tmp);
    DISABLE_TICK_COUNTER();
#else
    DISABLE_TICK_COUNTER();
    GET_TICK_CURRENT_VAL(&tmp);
#endif
    TICKLESS_ENABLE_INTERRUPTS();/* Re-enable interrupts */
    if (tickISRfired) {
      /* The tick interrupt has already executed, and the timer
       * count reloaded with the modulo/match value.
       * Reset the counter register with whatever remains of
       * this tick period. 
       */
#if COUNTS_UP
    #if configSYSTICK_USE_LOW_POWER_TIMER
      tickDuration = (UL_TIMER_COUNTS_FOR_ONE_TICK-1UL); /* LPTMR: interrupt will happen at match of compare register && increment, thus minus 1 */
    #else
      tickDuration = UL_TIMER_COUNTS_FOR_ONE_TICK;
    #endif
      if (tickDuration >= tmp) { /* make sure it does not underflow */
        tickDuration -= tmp;
      }
      if (tickDuration > 1) {
        /*! \todo Need to rethink this one! */
        //tickDuration -= 1; /* decrement by one, to compensate for one timer tick, as we are already part way through it */
      } else {
        /* Not enough time to setup for the next tick, so skip it and setup for the
         * next. Make sure to count the tick we skipped.
         */
        tickDuration += (UL_TIMER_COUNTS_FOR_ONE_TICK - 1UL);
        vTaskStepTick(1);
      }
#else
      tickDuration = (UL_TIMER_COUNTS_FOR_ONE_TICK-1UL)-(ulReloadValue-tmp);
#endif
      SET_TICK_DURATION(tickDuration);
      /* The tick interrupt handler will already have pended the tick
       * processing in the kernel.  As the pending tick will be
       * processed as soon as this function exits, the tick value
       * maintained by the tick is stepped forward by one less than the
       * time spent waiting.
       */
      ulCompleteTickPeriods = xExpectedIdleTime-1UL; /* -1 because we already added a completed tick from the tick interrupt */
    } else {
      /* Something other than the tick interrupt ended the sleep.
       * Work out how long the sleep lasted rounded to complete tick
       * periods (not the ulReload value which accounted for part ticks). 
       */
#if COUNTS_UP
      ulCompletedSysTickIncrements = tmp;
      /* How many complete tick periods passed while the processor was waiting? */
      ulCompleteTickPeriods = ulCompletedSysTickIncrements/UL_TIMER_COUNTS_FOR_ONE_TICK;
      /* The reload value is set to whatever fraction of a single tick period remains. */
      tickDuration = (((ulCompleteTickPeriods+1)*UL_TIMER_COUNTS_FOR_ONE_TICK)-1)-ulCompletedSysTickIncrements;
      if (tickDuration > 1) {
        tickDuration -= 1; /* decrement by one, to compensate for one timer tick, as we are already part way through it */
      } else {
         /* Not enough time to setup for the next tick, so skip it and setup for the
          * next. Make sure to count the tick we skipped.
          */
         tickDuration += (UL_TIMER_COUNTS_FOR_ONE_TICK - 1UL);
         if (tickDuration > 1) { /* check for underflow */
           tickDuration -= 1;
         }
         vTaskStepTick(1);
      }
#else
      ulCompletedSysTickIncrements = (xExpectedIdleTime*UL_TIMER_COUNTS_FOR_ONE_TICK)-tmp;
      /* How many complete tick periods passed while the processor was waiting? */
      ulCompleteTickPeriods = ulCompletedSysTickIncrements/UL_TIMER_COUNTS_FOR_ONE_TICK;
      /* The reload value is set to whatever fraction of a single tick period remains. */
      tickDuration = ((ulCompleteTickPeriods+1)*UL_TIMER_COUNTS_FOR_ONE_TICK)-ulCompletedSysTickIncrements;
#endif
      SET_TICK_DURATION(tickDuration);
    }
    /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
       again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
       value.  The critical section is used to ensure the tick interrupt
       can only execute once in the case that the reload register is near
       zero. 
     */
    RESET_TICK_COUNTER_VAL();
    portENTER_CRITICAL();
    {
      ENABLE_TICK_COUNTER();
      vTaskStepTick(ulCompleteTickPeriods);
#if configSYSTICK_USE_LOW_POWER_TIMER
      /* The compare register of the LPTMR should not be modified when the
       * timer is running, so wait for the next tick interrupt to change it.
       */
      if (tickDuration != (UL_TIMER_COUNTS_FOR_ONE_TICK-1UL)) { /* minus one because of LPTMR way to trigger interrupts */
        if (tickISRfired) {
          /* The pending tick interrupt will be immediately processed after
           * exiting this function so we need to delay the change of the tick
           * duration until the one after that.
           */
          restoreTickInterval = 2;
        } else {
          /* Notify the tick interrupt that the tick duration needs to be
           * changed back to the normal setting.
           */
          restoreTickInterval = 1;
        }
      } else {
        /* If the duration is the standard full tick, then there's no reason
         * to stop and restart LPTMR in the tick interrupt.
         */
        restoreTickInterval = 0;
      }
#else
      /* The systick has a load register that will automatically be used
       * when the counter counts down to zero.
       */
      SET_TICK_DURATION(UL_TIMER_COUNTS_FOR_ONE_TICK-1UL);
#endif
    }
    portEXIT_CRITICAL();
  }
}
#endif /* #if configUSE_TICKLESS_IDLE */
/*-----------------------------------------------------------*/
void vPortInitTickTimer(void) {
#if configUSE_TICKLESS_IDLE == 1
{
#if TICK_NOF_BITS==32
  xMaximumPossibleSuppressedTicks = 0xffffffffUL/TIMER_COUNTS_FOR_ONE_TICK; /* 32bit timer register */
#elif TICK_NOF_BITS==24
  xMaximumPossibleSuppressedTicks = 0xffffffUL/TIMER_COUNTS_FOR_ONE_TICK; /* 24bit timer register */
#elif TICK_NOF_BITS==16
  xMaximumPossibleSuppressedTicks = 0xffffUL/TIMER_COUNTS_FOR_ONE_TICK; /* 16bit timer register */
#elif TICK_NOF_BITS==8
  xMaximumPossibleSuppressedTicks = 0xffUL/TIMER_COUNTS_FOR_ONE_TICK; /* 8bit timer register */
#else
  error "unknown configuration!"
#endif
#if configSYSTICK_USE_LOW_POWER_TIMER
  ulStoppedTimerCompensation = configSTOPPED_TIMER_COMPENSATION/(configCPU_CLOCK_HZ/configSYSTICK_LOW_POWER_TIMER_CLOCK_HZ);
#else
  ulStoppedTimerCompensation = configSTOPPED_TIMER_COMPENSATION/(configCPU_CLOCK_HZ/configSYSTICK_CLOCK_HZ);
#endif
}
#endif /* configUSE_TICKLESS_IDLE */

#if configSYSTICK_USE_LOW_POWER_TIMER
  /* SIM_SCGx: enable clock to LPTMR */
  SIM_PDD_SetClockGate(SIM_BASE_PTR, SIM_PDD_CLOCK_GATE_LPTMR0, PDD_ENABLE);

  /* LPTRM0_CSR: clear TCF (Timer compare Flag) with writing a one to it */
  LPTMR_PDD_ClearInterruptFlag(LPTMR0_BASE_PTR);
  
  /* LPTMR_PSR: configure prescaler, bypass and clock source */
  /*           PBYP PCS
   * ERCLK32    1   10
   * LPO_1kHz   1   01
   * ERCLK      0   00
   * IRCLK      1   00
   */
  LPTMR_PDD_SelectPrescalerSource(LPTMR0_BASE_PTR, LPTMR_PDD_SOURCE_LPO1KHZ);
  LPTMR_PDD_EnablePrescalerBypass(LPTMR0_BASE_PTR, LPTMR_PDD_BYPASS_ENABLED);

  /* set timer interrupt priority in IP[] and enable it in ISER[] */
  NVIC_SetPriority(LDD_ivIndex_INT_LPTimer, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(LDD_ivIndex_INT_LPTimer); /* enable IRQ in NVIC_ISER[] */
#else /* use normal SysTick Counter */
  *(portNVIC_SYSPRI3) |= portNVIC_SYSTICK_PRI; /* set priority of SysTick interrupt */
#endif

  /* Configure timer to interrupt at the requested rate. */
  SET_TICK_DURATION(TIMER_COUNTS_FOR_ONE_TICK-1UL);
  RESET_TICK_COUNTER_VAL();
  ENABLE_TICK_COUNTER();
}
/*-----------------------------------------------------------*/
void vPortStartTickTimer(void) {
  ENABLE_TICK_COUNTER();
}
/*-----------------------------------------------------------*/
void vPortStopTickTimer(void) {
  DISABLE_TICK_COUNTER();
}
/*-----------------------------------------------------------*/
#if configCPU_FAMILY==configCPU_FAMILY_ARM_M4F /* floating point unit */
#if (configCOMPILER==configCOMPILER_ARM_GCC)
void vPortEnableVFP(void) {
  /* The FPU enable bits are in the CPACR. */
  __asm volatile (
    "  ldr.w r0, =0xE000ED88  \n" /* CAPCR, 0xE000ED88 */
    "  ldr r1, [r0]           \n" /* read CAPR */
    "  orr r1, r1, #(0xf<<20) \n" /* enable CP10 and CP11 coprocessors */
    "  str r1, [r0]           \n" /* store to new value back */
    : /* no output */
    : /* no input */
    : "r0","r1" /* clobber */
  );
}
#elif (configCOMPILER==configCOMPILER_ARM_KEIL)
__asm void vPortEnableVFP(void) {
    PRESERVE8

    /* The FPU enable bits are in the CPACR. */
    ldr.w r0, =0xE000ED88
    ldr    r1, [r0]
    /* Enable CP10 and CP11 coprocessors, then save back. */
    orr    r1, r1, #(0xf<<20)
    str r1, [r0]
    bx    r14
    nop
}
#endif /* GNU or Keil */
#endif /* configCPU_FAMILY_ARM_M4F */
/*-----------------------------------------------------------*/

#define portFIRST_USER_INTERRUPT_NUMBER		16

/*
 * Used by the portASSERT_IF_INTERRUPT_PRIORITY_INVALID() macro to ensure
 * FreeRTOS API functions are not called from interrupts that have been assigned
 * a priority above configMAX_SYSCALL_INTERRUPT_PRIORITY.
 */
#if ( configASSERT_DEFINED == 1 )
     static uint8_t ucMaxSysCallPriority = 0;
     static uint32_t ulMaxPRIGROUPValue = 0;
     static const volatile uint8_t * const pcInterruptPriorityRegisters = ( const volatile uint8_t * const ) portNVIC_IP_REGISTERS_OFFSET_16;
#endif /* configASSERT_DEFINED */

BaseType_t xPortStartScheduler(void) {
  /* configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to 0.
  See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html */
  configASSERT( configMAX_SYSCALL_INTERRUPT_PRIORITY );

  #if (configASSERT_DEFINED == 1 )
  {
      volatile uint32_t ulOriginalPriority;
      volatile uint8_t * const pucFirstUserPriorityRegister = ( volatile uint8_t * const ) ( portNVIC_IP_REGISTERS_OFFSET_16 + portFIRST_USER_INTERRUPT_NUMBER );
      volatile uint8_t ucMaxPriorityValue;

      /* Determine the maximum priority from which ISR safe FreeRTOS API
      functions can be called.  ISR safe functions are those that end in
      "FromISR".  FreeRTOS maintains separate thread and ISR API functions to
      ensure interrupt entry is as fast and simple as possible.

      Save the interrupt priority value that is about to be clobbered. */
      ulOriginalPriority = *pucFirstUserPriorityRegister;

      /* Determine the number of priority bits available.  First write to all
      possible bits. */
      *pucFirstUserPriorityRegister = portMAX_8_BIT_VALUE;

      /* Read the value back to see how many bits stuck. */
      ucMaxPriorityValue = *pucFirstUserPriorityRegister;

      /* Use the same mask on the maximum system call priority. */
      ucMaxSysCallPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY & ucMaxPriorityValue;

      /* Calculate the maximum acceptable priority group value for the number
      of bits read back. */
      ulMaxPRIGROUPValue = portMAX_PRIGROUP_BITS;
      while( ( ucMaxPriorityValue & portTOP_BIT_OF_BYTE ) == portTOP_BIT_OF_BYTE )
      {
          ulMaxPRIGROUPValue--;
          ucMaxPriorityValue <<= ( uint8_t ) 0x01;
      }

      /* Shift the priority group value back to its position within the AIRCR
      register. */
      ulMaxPRIGROUPValue <<= portPRIGROUP_SHIFT;
      ulMaxPRIGROUPValue &= portPRIORITY_GROUP_MASK;

      /* Restore the clobbered interrupt priority register to its original
      value. */
      *pucFirstUserPriorityRegister = ulOriginalPriority;
  }
  #endif /* conifgASSERT_DEFINED */

  /* Make PendSV, SVCall and SysTick the lowest priority interrupts. SysTick priority will be set in vPortInitTickTimer(). */
  *(portNVIC_SYSPRI3) |= portNVIC_PENDSV_PRI; /* set priority of PendSV interrupt */
  
  
  uxCriticalNesting = 0; /* Initialize the critical nesting count ready for the first task. */
  
  vPortInitTickTimer(); /* initialize tick timer */
  
  vPortStartTickTimer(); /* start tick timer */
  
  vPortEnableVFP(); /* Ensure the VFP is enabled - it should be anyway */
  *(portFPCCR) |= portASPEN_AND_LSPEN_BITS; /* Lazy register save always */
  
#if configUSE_TASK_END_SCHEDULER
    if(setjmp(xJumpBuf) != 0 ) {
      /* here we will get in case of call to vTaskEndScheduler() */
      return pdFALSE;
    }
#endif
  vPortStartFirstTask(); /* Start the first task. */
  /* Should not get here, unless you call vTaskEndScheduler()! */
  return pdFALSE;
}
/*-----------------------------------------------------------*/
void vPortEndScheduler(void) {
  vPortStopTickTimer();
  /* Jump back to the processor state prior to starting the
     scheduler.  This means we are not going to be using a
     task stack frame so the task can be deleted. */
#if configUSE_TASK_END_SCHEDULER
  longjmp(xJumpBuf, 1);
#else
  for(;;){} /* wait here */
#endif
}
/*-----------------------------------------------------------*/
void vPortEnterCritical(void) {
/*
 * Disable interrupts before incrementing the count of critical section nesting.
 * The nesting count is maintained so we know when interrupts should be
 * re-enabled.  Once interrupts are disabled the nesting count can be accessed
 * directly.  Each task maintains its own nesting count.
 */
  portDISABLE_INTERRUPTS();
  portPOST_ENABLE_DISABLE_INTERRUPTS();
  uxCriticalNesting++;
  __asm volatile("dsb");
  __asm volatile("isb");
}
/*-----------------------------------------------------------*/
void vPortExitCritical(void) {
 /* Interrupts are disabled so we can access the nesting count directly.  If the
  * nesting is found to be 0 (no nesting) then we are leaving the critical
  * section and interrupts can be re-enabled.
  */
  uxCriticalNesting--;
  if (uxCriticalNesting == 0)  {
    portENABLE_INTERRUPTS();
    portPOST_ENABLE_DISABLE_INTERRUPTS();
  }
}
/*-----------------------------------------------------------*/
void vPortYieldFromISR(void) {
  /* Set a PendSV to request a context switch. */
  *(portNVIC_INT_CTRL) = portNVIC_PENDSVSET_BIT;
  /* Barriers are normally not required but do ensure the code is completely
  within the specified behavior for the architecture. */
  __asm volatile("dsb");
  __asm volatile("isb");
}
/*-----------------------------------------------------------*/
/* return the tick raw counter value. It is assumed that the counter register has been reset at the last tick time */
portLONG uxGetTickCounterValue(void) {
  portLONG val;
  
  GET_TICK_CURRENT_VAL(&val);
  return val;
}
/*-----------------------------------------------------------*/
#if (configCOMPILER==configCOMPILER_ARM_KEIL)
#if configPEX_KINETIS_SDK /* the SDK expects different interrupt handler names */
void SysTick_Handler(void) {
#else
void vPortTickHandler(void) {
#endif
  /* this is how we get here:
    RTOSTICKLDD1_Interrupt:
    push {r4, lr}
    ...                                       RTOSTICKLDD1_OnCounterRestart
    bl RTOSTICKLDD1_OnCounterRestart     ->   push {r4,lr}
    pop {r4, lr}                              mov r4,r0
                                              bl vPortTickHandler
                                              pop {r4,pc}
  */
#if configUSE_TICKLESS_IDLE == 1
  TICK_INTERRUPT_FLAG_SET();
#endif
  portSET_INTERRUPT_MASK();   /* disable interrupts */
#if (configUSE_TICKLESS_IDLE == 1) && configSYSTICK_USE_LOW_POWER_TIMER
  if (restoreTickInterval > 0) { /* we got interrupted during tickless mode and non-standard compare value: reload normal compare value */
    if (restoreTickInterval == 1) {
      DISABLE_TICK_COUNTER();
      SET_TICK_DURATION(UL_TIMER_COUNTS_FOR_ONE_TICK-1UL);
      ENABLE_TICK_COUNTER();
    }
    restoreTickInterval -= 1;
  }
#endif
  if (xTaskIncrementTick()!=pdFALSE) { /* increment tick count */
    taskYIELD();
  }
  portCLEAR_INTERRUPT_MASK(); /* enable interrupts again */
}
#endif
/*-----------------------------------------------------------*/
#if (configCOMPILER==configCOMPILER_ARM_GCC)
#if configPEX_KINETIS_SDK /* the SDK expects different interrupt handler names */
void SysTick_Handler(void) {
#else
void vPortTickHandler(void) {
#endif
  ACKNOWLEDGE_TICK_ISR();
#if configUSE_TICKLESS_IDLE == 1
  TICK_INTERRUPT_FLAG_SET();
#endif
  portSET_INTERRUPT_MASK();   /* disable interrupts */
#if (configUSE_TICKLESS_IDLE == 1) && configSYSTICK_USE_LOW_POWER_TIMER
  if (restoreTickInterval > 0) { /* we got interrupted during tickless mode and non-standard compare value: reload normal compare value */
    if (restoreTickInterval == 1) {
      DISABLE_TICK_COUNTER();
      SET_TICK_DURATION(UL_TIMER_COUNTS_FOR_ONE_TICK-1UL);
      ENABLE_TICK_COUNTER();
    }
    restoreTickInterval -= 1;
  }
#endif
  if (xTaskIncrementTick()!=pdFALSE) { /* increment tick count */
    taskYIELD();
  }
  portCLEAR_INTERRUPT_MASK(); /* enable interrupts again */
}
#endif

void vPortStartFirstTask(void) {
  __asm volatile (
    " ldr r0, =0xE000ED08 \n" /* Use the NVIC offset register to locate the stack. */
    " ldr r0, [r0]        \n" /* load address of vector table */
    " ldr r0, [r0]        \n" /* load first entry of vector table which is the reset stack pointer */
    " msr msp, r0         \n" /* Set the msp back to the start of the stack. */
    " cpsie i             \n" /* Globally enable interrupts. */
	" cpsie f             \n" // Enable fast interrupts
	" dsb					\n"
	" isb					\n"
    " svc 0               \n" /* System call to start first task. */
    " nop                 \n"
  );
}

/*-----------------------------------------------------------*/
#if (configCOMPILER==configCOMPILER_ARM_KEIL)
#if configCPU_FAMILY_IS_ARM_M4(configCPU_FAMILY) /* Cortex M4 */
__asm void vPortSVCHandler(void) {
  PRESERVE8
  EXTERN pxCurrentTCB

  /* Get the location of the current TCB. */
  ldr r3, =pxCurrentTCB
  ldr r1, [r3]
  ldr r0, [r1]
  /* Pop the core registers. */
#if (configCPU_FAMILY==configCPU_FAMILY_ARM_M4F)
  ldmia r0!, {r4-r11, r14} /* \todo: r14, check http://sourceforge.net/p/freertos/discussion/382005/thread/a9406af1/?limit=25#3bc7 */
#else
  ldmia r0!, {r4-r11}
#endif
  msr psp, r0
  mov r0, #configMAX_SYSCALL_INTERRUPT_PRIORITY
  msr basepri, r0
#if (configCPU_FAMILY==configCPU_FAMILY_ARM_M4F)
#else
  orr r14, r14, #13
#endif
  bx r14
}
/*-----------------------------------------------------------*/
#else /* Cortex M0+ */
#if configPEX_KINETIS_SDK /* the SDK expects different interrupt handler names */
__asm void SVC_Handler(void) {
#else
__asm void vPortSVCHandler(void) {
#endif
  EXTERN pxCurrentTCB

  /* Get the location of the current TCB. */
  ldr r3, =pxCurrentTCB  /* Restore the context. */
  ldr r1, [r3]          /* Use pxCurrentTCBConst to get the pxCurrentTCB address. */
  ldr r0, [r1]          /* The first item in pxCurrentTCB is the task top of stack. */
  adds r0, #16          /* Move to the high registers. */
  ldmia r0!, {r4-r7}    /* Pop the high registers. */
  mov r8, r4 
  mov r9, r5 
  mov r10, r6
  mov r11, r7

  msr psp, r0           /* Remember the new top of stack for the task. */

  subs r0, #32          /* Go back for the low registers that are not automatically restored. */
  ldmia r0!, {r4-r7}    /* Pop low registers.  */
  mov r1, r14           /* OR R14 with 0x0d. */
  movs r0, #0x0d
  orrs r1, r0
  bx r1
  nop
}
#endif
#endif
/*-----------------------------------------------------------*/
#if (configCOMPILER==configCOMPILER_ARM_GCC)
#if configPEX_KINETIS_SDK /* the SDK expects different interrupt handler names */
__attribute__ ((naked)) void SVC_Handler(void) {
#else
__attribute__ ((naked)) void vPortSVCHandler(void) {
#endif
#if configCPU_FAMILY_IS_ARM_M4(configCPU_FAMILY) /* Cortex M4 */
__asm volatile (
    " ldr r3, pxCurrentTCBConst2 \n" /* Restore the context. */
    " ldr r1, [r3]               \n" /* Use pxCurrentTCBConst to get the pxCurrentTCB address. */
    " ldr r0, [r1]               \n" /* The first item in pxCurrentTCB is the task top of stack. */
    /* pop the core registers */
#if (configCPU_FAMILY==configCPU_FAMILY_ARM_M4F)
    " ldmia r0!, {r4-r11, r14}   \n"
#else
    " ldmia r0!, {r4-r11}        \n"
#endif
    " msr psp, r0                \n"
    " mov r0, #0                 \n"
    " msr basepri, r0            \n"
#if (configCPU_FAMILY==configCPU_FAMILY_ARM_M4F)
#else
    " orr r14, r14, #13          \n"
#endif
    " bx r14                     \n"
    "                            \n"
    " .align 2                   \n"
    "pxCurrentTCBConst2: .word pxCurrentTCB \n"
  );
#else /* Cortex M0+ */
  __asm volatile (
    " ldr r3, pxCurrentTCBConst2 \n" /* Restore the context. */
    " ldr r1, [r3]               \n" /* Use pxCurrentTCBConst to get the pxCurrentTCB address. */
    " ldr r0, [r1]               \n" /* The first item in pxCurrentTCB is the task top of stack. */
    " add r0, r0, #16            \n" /* Move to the high registers. */
    " ldmia r0!, {r4-r7}         \n" /* Pop the high registers. */
    " mov r8, r4                 \n"
    " mov r9, r5                 \n"
    " mov r10, r6                \n"
    " mov r11, r7                \n"
    "                            \n"
    " msr psp, r0                \n" /* Remember the new top of stack for the task. */
    "                            \n"
    " sub r0, r0, #32            \n" /* Go back for the low registers that are not automatically restored. */
    " ldmia r0!, {r4-r7}         \n" /* Pop low registers.  */
    " mov r1, r14                \n" /* OR R14 with 0x0d. */
    " movs r0, #0x0d             \n"
    " orr r1, r0                 \n"
    " bx r1                      \n"
    "                            \n"
    ".align 2                    \n"
    "pxCurrentTCBConst2: .word pxCurrentTCB \n"
  );
#endif
}
/*-----------------------------------------------------------*/
#endif

/*-----------------------------------------------------------*/

__attribute__ ((naked)) void vPortPendSVHandler(void) {
	int i;
	(void) i;
  __asm volatile (
    " mrs r0, psp                \n"
    " ldr  r3, pxCurrentTCBConst \n" /* Get the location of the current TCB. */
    " ldr  r2, [r3]              \n"
    " tst r14, #0x10             \n" /* Is the task using the FPU context?  If so, push high vfp registers. */
    " it eq                      \n"
    " vstmdbeq r0!, {s16-s31}    \n"
    " stmdb r0!, {r4-r11, r14}   \n" /* save remaining core registers */
    " str r0, [r2]               \n" /* Save the new top of stack into the first member of the TCB. */
    " stmdb sp!, {r3, r14}       \n"
    " mov r0, %0                 \n"
    " msr basepri, r0            \n"
    " bl vTaskSwitchContext      \n"
    " mov r0, #0                 \n"
    " msr basepri, r0            \n"
    " ldmia sp!, {r3, r14}       \n"
    " ldr r1, [r3]               \n" /* The first item in pxCurrentTCB is the task top of stack. */
    " ldr r0, [r1]               \n"
    " ldmia r0!, {r4-r11, r14}   \n" /* Pop the core registers */
    " tst r14, #0x10             \n" /* Is the task using the FPU context?  If so, pop the high vfp registers too. */
    " it eq                      \n"
    " vldmiaeq r0!, {s16-s31}    \n"
    " msr psp, r0                \n"
    " bx r14                     \n"
    "                            \n"
    " .align 2                   \n"
    "pxCurrentTCBConst: .word pxCurrentTCB  \n"
    ::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY)
  );
}
/*-----------------------------------------------------------*/


    void vPortValidateInterruptPriority( void )
    {
    uint32_t ulCurrentInterrupt;
    uint8_t ucCurrentPriority;

        /* Obtain the number of the currently executing interrupt. */
        __asm volatile( "mrs %0, ipsr" : "=r"( ulCurrentInterrupt ) );

        /* Is the interrupt number a user defined interrupt? */
        if( ulCurrentInterrupt >= portFIRST_USER_INTERRUPT_NUMBER )
        {
            /* Look up the interrupt's priority. */
            ucCurrentPriority = pcInterruptPriorityRegisters[ ulCurrentInterrupt ];

            /* The following assertion will fail if a service routine (ISR) for
            an interrupt that has been assigned a priority above
            configMAX_SYSCALL_INTERRUPT_PRIORITY calls an ISR safe FreeRTOS API
            function.  ISR safe FreeRTOS API functions must *only* be called
            from interrupts that have been assigned a priority at or below
            configMAX_SYSCALL_INTERRUPT_PRIORITY.

            Numerically low interrupt priority numbers represent logically high
            interrupt priorities, therefore the priority of the interrupt must
            be set to a value equal to or numerically *higher* than
            configMAX_SYSCALL_INTERRUPT_PRIORITY.

            Interrupts that use the FreeRTOS API must not be left at their
            default priority of zero as that is the highest possible priority,
            which is guaranteed to be above configMAX_SYSCALL_INTERRUPT_PRIORITY,
            and therefore also guaranteed to be invalid.

            FreeRTOS maintains separate thread and ISR API functions to ensure
            interrupt entry is as fast and simple as possible.

            The following links provide detailed information:
            http://www.freertos.org/RTOS-Cortex-M3-M4.html
            http://www.freertos.org/FAQHelp.html */
            configASSERT( ucCurrentPriority >= ucMaxSysCallPriority );
        }

        /* Priority grouping:  The interrupt controller (NVIC) allows the bits
        that define each interrupt's priority to be split between bits that
        define the interrupt's pre-emption priority bits and bits that define
        the interrupt's sub-priority.  For simplicity all bits must be defined
        to be pre-emption priority bits.  The following assertion will fail if
        this is not the case (if some bits represent a sub-priority).

        If the application only uses CMSIS libraries for interrupt
        configuration then the correct setting can be achieved on all Cortex-M
        devices by calling NVIC_SetPriorityGrouping( 0 ); before starting the
        scheduler.  Note however that some vendor specific peripheral libraries
        assume a non-zero priority group setting, in which cases using a value
        of zero will result in unpredicable behaviour. */
        configASSERT( ( portAIRCR_REG & portPRIORITY_GROUP_MASK ) <= ulMaxPRIGROUPValue );
    }


__attribute__(( naked )) uint32_t ulPortSetInterruptMask( void )
{
    __asm volatile                                                        \
    (                                                                    \
        "    mrs r0, basepri                                            \n" \
        "    mov r1, %0                                                \n"    \
        "    msr basepri, r1                                            \n" \
        "    bx lr                                                    \n" \
        :: "i" ( configMAX_SYSCALL_INTERRUPT_PRIORITY ) : "r0", "r1"    \
    );

    /* This return will not be reached but is necessary to prevent compiler
    warnings. */
    return 0;
}

/*-----------------------------------------------------------*/
__attribute__(( naked )) void vPortClearInterruptMask( uint32_t ulNewMaskValue )
{
    __asm volatile                                                    \
    (                                                                \
        "    msr basepri, r0                                        \n"    \
        "    bx lr                                                \n" \
        :::"r0"                                                        \
    );

    /* Just to avoid compiler warnings. */
    ( void ) ulNewMaskValue;
}

#if (configCPU_FAMILY==configCPU_FAMILY_ARM_M0P) && (configCOMPILER==configCOMPILER_ARM_GCC)
/*-----------------------------------------------------------*/
__attribute__(( naked )) uint32_t ulPortSetInterruptMask( void )
{
    __asm volatile(
                    " mrs r0, PRIMASK    \n"
                    " cpsid i            \n"
                    " bx lr                "
                  );

    /* To avoid compiler warnings.  This line will never be reached. */
    return 0;
}
/*-----------------------------------------------------------*/

__attribute__(( naked )) void vPortClearInterruptMask( uint32_t ulNewMaskValue )
{
    __asm volatile(
                    " msr PRIMASK, r0    \n"
                    " bx lr                "
                  );

    /* Just to avoid compiler warning. */
    ( void ) ulNewMaskValue;
}
#endif /* ARM M0P core && COMPILER_ARM_GCC */

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Scheduler includes. */
#include <FreeRTOS/FreeRTOS.h>
#include <FreeRTOS/queue.h>
#include <FreeRTOS/event_groups.h>
#include <FreeRTOS/mpu_prototypes.h>

#ifndef __VFP_FP__
	#error This port can only be used when the project options are configured to enable hardware floating point support.
#endif

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Constants required to access and manipulate the NVIC. */
#define portNVIC_SYSPRI2_REG					( *	( ( volatile uint32_t * ) 0xe000ed20 ) )
#define portNVIC_SYSPRI1_REG					( * ( ( volatile uint32_t * ) 0xe000ed1c ) )
#define portNVIC_SYS_CTRL_STATE_REG				( * ( ( volatile uint32_t * ) 0xe000ed24 ) )
#define portNVIC_MEM_FAULT_ENABLE				( 1UL << 16UL )

/* Constants required to access and manipulate the MPU. */
#define portMPU_TYPE_REG						( * ( ( volatile uint32_t * ) 0xe000ed90 ) )
#define portMPU_REGION_BASE_ADDRESS_REG			( * ( ( volatile uint32_t * ) 0xe000ed9C ) )
#define portMPU_REGION_ATTRIBUTE_REG			( * ( ( volatile uint32_t * ) 0xe000edA0 ) )
#define portMPU_CTRL_REG						( * ( ( volatile uint32_t * ) 0xe000ed94 ) )
#define portEXPECTED_MPU_TYPE_VALUE				( 8UL << 8UL ) /* 8 regions, unified. */
#define portMPU_ENABLE							( 0x01UL )
#define portMPU_BACKGROUND_ENABLE				( 1UL << 2UL )
#define portPRIVILEGED_EXECUTION_START_ADDRESS	( 0UL )
#define portMPU_REGION_VALID					( 0x10UL )
#define portMPU_REGION_ENABLE					( 0x01UL )
#define portPERIPHERALS_START_ADDRESS			0x40000000UL
#define portPERIPHERALS_END_ADDRESS				0x5FFFFFFFUL

/* Constants required to access and manipulate the SysTick. */
#define portNVIC_SYSTICK_CLK					( 0x00000004UL )
#define portNVIC_SYSTICK_INT					( 0x00000002UL )
#define portNVIC_SYSTICK_ENABLE					( 0x00000001UL )

/* Constants required to set up the initial stack. */
#define portINITIAL_CONTROL_IF_UNPRIVILEGED		( 0x03 )
#define portINITIAL_CONTROL_IF_PRIVILEGED		( 0x02 )



/* Offsets in the stack to the parameters when inside the SVC handler. */
#define portOFFSET_TO_PC						( 6 )

/* For strict compliance with the Cortex-M spec the task start address should
have bit-0 clear, as it is loaded into the PC on exit from an ISR. */
#define portSTART_ADDRESS_MASK				( ( StackType_t ) 0xfffffffeUL )

/*
 * Setup the timer to generate the tick interrupts.
 */
//static void prvSetupTimerInterrupt( void ) PRIVILEGED_FUNCTION;

/*
 * Configure a number of standard MPU regions that are used by all tasks.
 */
//static void prvSetupMPU( void ) PRIVILEGED_FUNCTION;

/*
 * Return the smallest MPU region size that a given number of bytes will fit
 * into.  The region size is returned as the value that should be programmed
 * into the region attribute register for that region.
 */
static uint32_t prvGetMPURegionSizeSetting( uint32_t ulActualSizeInBytes ) PRIVILEGED_FUNCTION;

/*
 * Checks to see if being called from the context of an unprivileged task, and
 * if so raises the privilege level and returns false - otherwise does nothing
 * other than return true.
 */
BaseType_t xPortRaisePrivilege( void ) __attribute__(( naked ));

/*
 * Standard FreeRTOS exception handlers.
 */
void vPortPendSVHandler( void ) __attribute__ (( naked )) PRIVILEGED_FUNCTION;
void vPortSysTickHandler( void )  __attribute__ ((optimize("3"))) PRIVILEGED_FUNCTION;
void vPortSVCHandler( void ) __attribute__ (( naked )) PRIVILEGED_FUNCTION;

/*
 * Starts the scheduler by restoring the context of the first task to run.
 */
//static void prvRestoreContextOfFirstTask( void ) __attribute__(( naked )) PRIVILEGED_FUNCTION;

/*
 * C portion of the SVC handler.  The SVC handler is split between an asm entry
 * and a C wrapper for simplicity of coding and maintenance.
 */
//static void prvSVCHandler( uint32_t *pulRegisters ) __attribute__(( noinline )) PRIVILEGED_FUNCTION;


/*-----------------------------------------------------------*/


// static void prvSVCHandler(	uint32_t *pulParam )
// {
// uint8_t ucSVCNumber;
// 
// 	/* The stack contains: r0, r1, r2, r3, r12, r14, the return address and
// 	xPSR.  The first argument (r0) is pulParam[ 0 ]. */
// 	ucSVCNumber = ( ( uint8_t * ) pulParam[ portOFFSET_TO_PC ] )[ -2 ];
// 	switch( ucSVCNumber )
// 	{
// 		case portSVC_START_SCHEDULER	:	portNVIC_SYSPRI1_REG |= portNVIC_SVC_PRI;
// 											prvRestoreContextOfFirstTask();
// 											break;
// 
// 		case portSVC_YIELD				:	portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
// 											/* Barriers are normally not required
// 											but do ensure the code is completely
// 											within the specified behaviour for the
// 											architecture. */
// 											__asm volatile( "dsb" );
// 											__asm volatile( "isb" );
// 
// 											break;
// 
// 		case portSVC_RAISE_PRIVILEGE	:	__asm volatile
// 											(
// 												"	mrs r1, control		\n" /* Obtain current control value. */
// 												"	bic r1, #1			\n" /* Set privilege bit. */
// 												"	msr control, r1		\n" /* Write back new control value. */
// 												:::"r1"
// 											);
// 											break;
// 
// 		default							:	/* Unknown SVC call. */
// 											break;
// 	}
// }
// /*-----------------------------------------------------------*/

// static void prvRestoreContextOfFirstTask( void )
// {
// 	__asm volatile
// 	(
// 		"	ldr r0, =0xE000ED08				\n" /* Use the NVIC offset register to locate the stack. */
// 		"	ldr r0, [r0]					\n"
// 		"	ldr r0, [r0]					\n"
// 		"	msr msp, r0						\n" /* Set the msp back to the start of the stack. */
// 		"	ldr	r3, pxCurrentTCBConst2		\n" /* Restore the context. */
// 		"	ldr r1, [r3]					\n"
// 		"	ldr r0, [r1]					\n" /* The first item in the TCB is the task top of stack. */
// 		"	add r1, r1, #4					\n" /* Move onto the second item in the TCB... */
// 		"	ldr r2, =0xe000ed9c				\n" /* Region Base Address register. */
// 		"	ldmia r1!, {r4-r11}				\n" /* Read 4 sets of MPU registers. */
// 		"	stmia r2!, {r4-r11}				\n" /* Write 4 sets of MPU registers. */
// 		"	ldmia r0!, {r3-r11, r14}		\n" /* Pop the registers that are not automatically saved on exception entry. */
// 		"	msr control, r3					\n"
// 		"	msr psp, r0						\n" /* Restore the task stack pointer. */
// 		"	mov r0, #0						\n"
// 		"	msr	basepri, r0					\n"
// 		"	bx r14							\n"
// 		"									\n"
// 		"	.align 4						\n"
// 		"pxCurrentTCBConst2: .word pxCurrentTCB	\n"
// 	);
// }
/*-----------------------------------------------------------*/



void vPortSysTickHandler( void )
{
uint32_t ulDummy;

	ulDummy = portSET_INTERRUPT_MASK_FROM_ISR();
	{
		/* Increment the RTOS tick. */
		if( xTaskIncrementTick() != pdFALSE )
		{
			/* Pend a context switch. */
			portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
		}
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR( ulDummy );
}
/*-----------------------------------------------------------*/

// /*
//  * Setup the systick timer to generate the tick interrupts at the required
//  * frequency.
//  */
// static void prvSetupTimerInterrupt( void )
// {
// 	/* Configure SysTick to interrupt at the requested rate. */
// 	portNVIC_SYSTICK_LOAD_REG = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
// 	portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK | portNVIC_SYSTICK_INT | portNVIC_SYSTICK_ENABLE;
// }
// /*-----------------------------------------------------------*/


// static void prvSetupMPU( void )
// {
// extern uint32_t __privileged_functions_end__[];
// extern uint32_t __FLASH_segment_start__[];
// extern uint32_t __FLASH_segment_end__[];
// extern uint32_t __privileged_data_start__[];
// extern uint32_t __privileged_data_end__[];
// 
// 	/* Check the expected MPU is present. */
// 	if( portMPU_TYPE_REG == portEXPECTED_MPU_TYPE_VALUE )
// 	{
// 		/* First setup the entire flash for unprivileged read only access. */
// 		portMPU_REGION_BASE_ADDRESS_REG =	( ( uint32_t ) __FLASH_segment_start__ ) | /* Base address. */
// 											( portMPU_REGION_VALID ) |
// 											( portUNPRIVILEGED_FLASH_REGION );
// 
// 		portMPU_REGION_ATTRIBUTE_REG =	( portMPU_REGION_READ_ONLY ) |
// 										( portMPU_REGION_CACHEABLE_BUFFERABLE ) |
// 										( prvGetMPURegionSizeSetting( ( uint32_t ) __FLASH_segment_end__ - ( uint32_t ) __FLASH_segment_start__ ) ) |
// 										( portMPU_REGION_ENABLE );
// 
// 		/* Setup the first 16K for privileged only access (even though less
// 		than 10K is actually being used).  This is where the kernel code is
// 		placed. */
// 		portMPU_REGION_BASE_ADDRESS_REG =	( ( uint32_t ) __FLASH_segment_start__ ) | /* Base address. */
// 											( portMPU_REGION_VALID ) |
// 											( portPRIVILEGED_FLASH_REGION );
// 
// 		portMPU_REGION_ATTRIBUTE_REG =	( portMPU_REGION_PRIVILEGED_READ_ONLY ) |
// 										( portMPU_REGION_CACHEABLE_BUFFERABLE ) |
// 										( prvGetMPURegionSizeSetting( ( uint32_t ) __privileged_functions_end__ - ( uint32_t ) __FLASH_segment_start__ ) ) |
// 										( portMPU_REGION_ENABLE );
// 
// 		/* Setup the privileged data RAM region.  This is where the kernel data
// 		is placed. */
// 		portMPU_REGION_BASE_ADDRESS_REG =	( ( uint32_t ) __privileged_data_start__ ) | /* Base address. */
// 											( portMPU_REGION_VALID ) |
// 											( portPRIVILEGED_RAM_REGION );
// 
// 		portMPU_REGION_ATTRIBUTE_REG =	( portMPU_REGION_PRIVILEGED_READ_WRITE ) |
// 										( portMPU_REGION_CACHEABLE_BUFFERABLE ) |
// 										prvGetMPURegionSizeSetting( ( uint32_t ) __privileged_data_end__ - ( uint32_t ) __privileged_data_start__ ) |
// 										( portMPU_REGION_ENABLE );
// 
// 		/* By default allow everything to access the general peripherals.  The
// 		system peripherals and registers are protected. */
// 		portMPU_REGION_BASE_ADDRESS_REG =	( portPERIPHERALS_START_ADDRESS ) |
// 											( portMPU_REGION_VALID ) |
// 											( portGENERAL_PERIPHERALS_REGION );
// 
// 		portMPU_REGION_ATTRIBUTE_REG =	( portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER ) |
// 										( prvGetMPURegionSizeSetting( portPERIPHERALS_END_ADDRESS - portPERIPHERALS_START_ADDRESS ) ) |
// 										( portMPU_REGION_ENABLE );
// 
// 		/* Enable the memory fault exception. */
// 		portNVIC_SYS_CTRL_STATE_REG |= portNVIC_MEM_FAULT_ENABLE;
// 
// 		/* Enable the MPU with the background region configured. */
// 		portMPU_CTRL_REG |= ( portMPU_ENABLE | portMPU_BACKGROUND_ENABLE );
// 	}
// }
// /*-----------------------------------------------------------*/

static uint32_t prvGetMPURegionSizeSetting( uint32_t ulActualSizeInBytes )
{
uint32_t ulRegionSize, ulReturnValue = 4;

	/* 32 is the smallest region size, 31 is the largest valid value for
	ulReturnValue. */
	for( ulRegionSize = 32UL; ulReturnValue < 31UL; ( ulRegionSize <<= 1UL ) )
	{
		if( ulActualSizeInBytes <= ulRegionSize )
		{
			break;
		}
		else
		{
			ulReturnValue++;
		}
	}

	/* Shift the code by one before returning so it can be written directly
	into the the correct bit position of the attribute register. */
	return ( ulReturnValue << 1UL );
}
/*-----------------------------------------------------------*/

BaseType_t xPortRaisePrivilege( void )
{
	__asm volatile
	(
		"	mrs r0, control						\n"
		"	tst r0, #1							\n" /* Is the task running privileged? */
		"	itte ne								\n"
		"	movne r0, #0						\n" /* CONTROL[0]!=0, return false. */
		"	svcne %0							\n" /* Switch to privileged. */
		"	moveq r0, #1						\n" /* CONTROL[0]==0, return true. */
		"	bx lr								\n"
		:: "i" (portSVC_RAISE_PRIVILEGE) : "r0"
	);

	return 0;
}
/*-----------------------------------------------------------*/

void vPortStoreTaskMPUSettings( xMPU_SETTINGS *xMPUSettings, const struct xMEMORY_REGION * const xRegions, StackType_t *pxBottomOfStack, uint32_t ulStackDepth )
{
extern uint32_t __SRAM_segment_start__[];
extern uint32_t __SRAM_segment_end__[];
extern uint32_t __privileged_data_start__[];
extern uint32_t __privileged_data_end__[];
int32_t lIndex;
uint32_t ul;

	if( xRegions == NULL )
	{
		/* No MPU regions are specified so allow access to all RAM. */
		xMPUSettings->xRegion[ 0 ].ulRegionBaseAddress =
				( ( uint32_t ) __SRAM_segment_start__ ) | /* Base address. */
				( portMPU_REGION_VALID ) |
				( portSTACK_REGION );

		xMPUSettings->xRegion[ 0 ].ulRegionAttribute =
				( portMPU_REGION_READ_WRITE ) |
				( portMPU_REGION_CACHEABLE_BUFFERABLE ) |
				( prvGetMPURegionSizeSetting( ( uint32_t ) __SRAM_segment_end__ - ( uint32_t ) __SRAM_segment_start__ ) ) |
				( portMPU_REGION_ENABLE );

		/* Re-instate the privileged only RAM region as xRegion[ 0 ] will have
		just removed the privileged only parameters. */
		xMPUSettings->xRegion[ 1 ].ulRegionBaseAddress =
				( ( uint32_t ) __privileged_data_start__ ) | /* Base address. */
				( portMPU_REGION_VALID ) |
				( portSTACK_REGION + 1 );

		xMPUSettings->xRegion[ 1 ].ulRegionAttribute =
				( portMPU_REGION_PRIVILEGED_READ_WRITE ) |
				( portMPU_REGION_CACHEABLE_BUFFERABLE ) |
				prvGetMPURegionSizeSetting( ( uint32_t ) __privileged_data_end__ - ( uint32_t ) __privileged_data_start__ ) |
				( portMPU_REGION_ENABLE );

		/* Invalidate all other regions. */
		for( ul = 2; ul <= portNUM_CONFIGURABLE_REGIONS; ul++ )
		{
			xMPUSettings->xRegion[ ul ].ulRegionBaseAddress = ( portSTACK_REGION + ul ) | portMPU_REGION_VALID;
			xMPUSettings->xRegion[ ul ].ulRegionAttribute = 0UL;
		}
	}
	else
	{
		/* This function is called automatically when the task is created - in
		which case the stack region parameters will be valid.  At all other
		times the stack parameters will not be valid and it is assumed that the
		stack region has already been configured. */
		if( ulStackDepth > 0 )
		{
			/* Define the region that allows access to the stack. */
			xMPUSettings->xRegion[ 0 ].ulRegionBaseAddress =
					( ( uint32_t ) pxBottomOfStack ) |
					( portMPU_REGION_VALID ) |
					( portSTACK_REGION ); /* Region number. */

			xMPUSettings->xRegion[ 0 ].ulRegionAttribute =
					( portMPU_REGION_READ_WRITE ) | /* Read and write. */
					( prvGetMPURegionSizeSetting( ulStackDepth * ( uint32_t ) sizeof( StackType_t ) ) ) |
					( portMPU_REGION_CACHEABLE_BUFFERABLE ) |
					( portMPU_REGION_ENABLE );
		}

		lIndex = 0;

		for( ul = 1; ul <= portNUM_CONFIGURABLE_REGIONS; ul++ )
		{
			if( ( xRegions[ lIndex ] ).ulLengthInBytes > 0UL )
			{
				/* Translate the generic region definition contained in
				xRegions into the CM3 specific MPU settings that are then
				stored in xMPUSettings. */
				xMPUSettings->xRegion[ ul ].ulRegionBaseAddress =
						( ( uint32_t ) xRegions[ lIndex ].pvBaseAddress ) |
						( portMPU_REGION_VALID ) |
						( portSTACK_REGION + ul ); /* Region number. */

				xMPUSettings->xRegion[ ul ].ulRegionAttribute =
						( prvGetMPURegionSizeSetting( xRegions[ lIndex ].ulLengthInBytes ) ) |
						( xRegions[ lIndex ].ulParameters ) |
						( portMPU_REGION_ENABLE );
			}
			else
			{
				/* Invalidate the region. */
				xMPUSettings->xRegion[ ul ].ulRegionBaseAddress = ( portSTACK_REGION + ul ) | portMPU_REGION_VALID;
				xMPUSettings->xRegion[ ul ].ulRegionAttribute = 0UL;
			}

			lIndex++;
		}
	}
}
/*-----------------------------------------------------------*/
