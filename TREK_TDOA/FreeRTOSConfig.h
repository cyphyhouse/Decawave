/*
    FreeRTOS V7.4.1 - Copyright (C) 2013 Real Time Engineers Ltd.

    FEATURES AND PORTS ARE ADDED TO FREERTOS ALL THE TIME.  PLEASE VISIT
    http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.

    >>>>>>NOTE<<<<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
    details. You should have received a copy of the GNU General Public License
    and the FreeRTOS license exception along with FreeRTOS; if not it can be
    viewed here: http://www.freertos.org/a00114.html and also obtained by
    writing to Real Time Engineers Ltd., contact details for whom are available
    on the FreeRTOS WEB site.

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************


    http://www.FreeRTOS.org - Documentation, books, training, latest versions, 
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, and our new
    fully thread aware and reentrant UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High 
    Integrity Systems, who sell the code with commercial support, 
    indemnification and middleware, under the OpenRTOS brand.
    
    http://www.SafeRTOS.com - High Integrity Systems also provide a safety 
    engineered and independently SIL3 certified version for use in safety and 
    mission critical applications that require provable dependability.
*/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Setting configSUPPORT_STATIC_ALLOCATION to 1 allows RTOS objects to be
created using only application supplied memory.  No dynamic memory allocation
will be performed. */
#define configSUPPORT_STATIC_ALLOCATION 1
/* Setting configSUPPORT_DYNAMIC_ALLOCATION to 0 results in all calls to
pvPortMalloc() returning NULL, and all calls to vPortFree() being ignored.
Therefore the application can be built without providing an implementation of
either of these functions (so none of the normal heap_n.c files described on
http://www.freertos.org/a00111.html are required).  Note that
configTOTAL_HEAP_SIZE is not defined. */
#define configSUPPORT_DYNAMIC_ALLOCATION 0

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION		1
#define configUSE_IDLE_HOOK			0
#define configUSE_TICK_HOOK			1
#define configCPU_CLOCK_HZ			( ( unsigned long ) 72000000 )
#define configTICK_RATE_HZ			( 1000 )
#define configMAX_PRIORITIES		( 7 )
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) 512 )
#define configTOTAL_HEAP_SIZE		( ( size_t ) ( 50 * 1024 ) )
#define configMAX_TASK_NAME_LEN		( 16 )
#define configUSE_TRACE_FACILITY	0
#define configUSE_16_BIT_TICKS		0
#define configIDLE_SHOULD_YIELD		1

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		0
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

#define configUSE_MUTEXES				1
#define configUSE_COUNTING_SEMAPHORES 	1
#define configUSE_ALTERNATIVE_API 		0
#define configCHECK_FOR_STACK_OVERFLOW	0
#define configUSE_RECURSIVE_MUTEXES		1
#define configQUEUE_REGISTRY_SIZE		20
#define configGENERATE_RUN_TIME_STATS	0

#define configUSE_APPLICATION_TASK_TAG	1
#define configUSE_QUEUE_SETS			1
#define configUSE_TASK_NOTIFICATIONS	1
#define configUSE_MALLOC_FAILED_HOOK 	0 /* pvPortMalloc() is not used. */

#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()
#define portGET_RUN_TIME_COUNTER_VALUE()

/* Software timer related configuration options. */
#define configUSE_TIMERS                        0
#define configTIMER_TASK_PRIORITY               ( 1 )
#define configTIMER_QUEUE_LENGTH                20
#define configTIMER_TASK_STACK_DEPTH ( configMINIMAL_STACK_SIZE )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet				1
#define INCLUDE_uxTaskPriorityGet				1
#define INCLUDE_vTaskDelete						1
#define INCLUDE_vTaskCleanUpResources			0
#define INCLUDE_vTaskSuspend					1
#define INCLUDE_vTaskDelayUntil					1
#define INCLUDE_vTaskDelay						1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTimerGetTimerDaemonTaskHandle  0
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xSemaphoreGetMutexHolder        1
#define INCLUDE_xTimerPendFunctionCall          0
#define INCLUDE_xTaskAbortDelay 				1

/* This is the raw value as per the Cortex-M3 NVIC.  Values can be 255
(lowest) to 0 (1?) (highest). */
#define configKERNEL_INTERRUPT_PRIORITY 		255
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	191 /* equivalent to 0xb0, or priority 11. */


/* This is the value being used as per the ST library which permits 16
priority values, 0 to 15.  This must correspond to the
configKERNEL_INTERRUPT_PRIORITY setting.  Here 15 corresponds to the lowest
NVIC value of 255. */
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY	15

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names - or at least those used in the unmodified vector table. */
#define vPortSVCHandler 	SVC_Handler
#define xPortPendSVHandler 	PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/* It is a good idea to define configASSERT() while developing.  configASSERT()
uses the same semantics as the standard C assert() macro. */
extern void vAssertCalled( unsigned long ulLine, const char * const pcFileName );
#define configASSERT( x ) if( ( x ) == 0 ) vAssertCalled( __LINE__, __FILE__ )

#endif /* FREERTOS_CONFIG_H */

