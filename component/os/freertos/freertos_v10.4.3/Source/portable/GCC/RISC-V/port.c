/*
 * FreeRTOS Kernel V10.4.3
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 * 1 tab == 4 spaces!
 */

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the RISC-V RV32 port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

/* Standard includes. */
#include "string.h"

#ifdef configCLINT_BASE_ADDRESS
#undef configCLINT_BASE_ADDRESS
#endif

#ifndef configMTIME_BASE_ADDRESS
#warning configMTIME_BASE_ADDRESS must be defined in FreeRTOSConfig.h.  If the target chip includes a memory-mapped mtime register then set configMTIME_BASE_ADDRESS to the mapped address.  Otherwise set configMTIME_BASE_ADDRESS to 0.  See https://www.FreeRTOS.org/Using-FreeRTOS-on-RISC-V.html
#endif

#ifndef configMTIMECMP_BASE_ADDRESS
#warning configMTIMECMP_BASE_ADDRESS must be defined in FreeRTOSConfig.h.  If the target chip includes a memory-mapped mtimecmp register then set configMTIMECMP_BASE_ADDRESS to the mapped address.  Otherwise set configMTIMECMP_BASE_ADDRESS to 0.  See https://www.FreeRTOS.org/Using-FreeRTOS-on-RISC-V.html
#endif

/* Let the user override the pre-loading of the initial LR with the address of
prvTaskExitError() in case it messes up unwinding of the stack in the
debugger. */
#ifdef configTASK_RETURN_ADDRESS
#define portTASK_RETURN_ADDRESS	configTASK_RETURN_ADDRESS
#else
#define portTASK_RETURN_ADDRESS	prvTaskExitError
#endif

/* The stack used by interrupt service routines.  Set configISR_STACK_SIZE_WORDS
to use a statically allocated array as the interrupt stack.  Alternative leave
configISR_STACK_SIZE_WORDS undefined and update the linker script so that a
linker variable names __freertos_irq_stack_top has the same value as the top
of the stack used by main.  Using the linker script method will repurpose the
stack that was used by main before the scheduler was started for use as the
interrupt stack after the scheduler has started. */
#ifdef configISR_STACK_SIZE_WORDS
static __attribute__((aligned(16))) StackType_t xISRStack[ configISR_STACK_SIZE_WORDS ] = { 0 };
const StackType_t xISRStackTop = (StackType_t) & (xISRStack[ configISR_STACK_SIZE_WORDS & ~portBYTE_ALIGNMENT_MASK ]);

/* Don't use 0xa5 as the stack fill bytes as that is used by the kernerl for
the task stacks, and so will legitimately appear in many positions within
the ISR stack. */
#define portISR_STACK_FILL_BYTE	0xee
#else
extern const uint32_t __freertos_irq_stack_top[];
const StackType_t xISRStackTop = (StackType_t) __freertos_irq_stack_top;
#endif

extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */

/*
 * Setup the timer to generate the tick interrupts.  The implementation in this
 * file is weak to allow application writers to change the timer used to
 * generate the tick interrupt.
 */
void vPortSetupTimerInterrupt(void) __attribute__((weak));

/*-----------------------------------------------------------*/

/* Used to program the machine timer compare register. */
uint64_t ullNextTime = 0ULL;
const uint64_t *pullNextTime = &ullNextTime;
size_t uxTimerIncrementsForOneTick;          /* Assumes increment won't go over 32-bits. */
uint32_t const ullMachineTimerCompareRegisterBase = configMTIMECMP_BASE_ADDRESS;
volatile uint64_t *pullMachineTimerCompareRegister = NULL;
#if( configUSE_TICKLESS_IDLE == 1 )
volatile uint64_t *pullMachineTimerCounterRegister = NULL;
#endif

/* Set configCHECK_FOR_STACK_OVERFLOW to 3 to add ISR stack checking to task
stack checking.  A problem in the ISR stack will trigger an assert, not call the
stack overflow hook function (because the stack overflow hook is specific to a
task stack, not the ISR stack). */
#if defined( configISR_STACK_SIZE_WORDS ) && ( configCHECK_FOR_STACK_OVERFLOW > 2 )
#warning This path not tested, or even compiled yet.

static const uint8_t ucExpectedStackBytes[] = {
	portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE,		\
	portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE,		\
	portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE,		\
	portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE,		\
	portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE, portISR_STACK_FILL_BYTE
};
\

#define portCHECK_ISR_STACK() configASSERT( ( memcmp( ( void * ) xISRStack, ( void * ) ucExpectedStackBytes, sizeof( ucExpectedStackBytes ) ) == 0 ) )
#else
/* Define the function away. */
#define portCHECK_ISR_STACK()
#endif /* configCHECK_FOR_STACK_OVERFLOW > 2 */

/*-----------------------------------------------------------*/

#if( configMTIME_BASE_ADDRESS != 0 ) && ( configMTIMECMP_BASE_ADDRESS != 0 )

void vPortSetupTimerInterrupt(void)
{
	uint32_t ulCurrentTimeHigh, ulCurrentTimeLow;
	volatile uint32_t *const pulTimeHigh = (volatile uint32_t *const)((configMTIME_BASE_ADDRESS) +
										   4UL);          /* 8-byte typer so high 32-bit word is 4 bytes up. */
	volatile uint32_t *const pulTimeLow = (volatile uint32_t *const)(configMTIME_BASE_ADDRESS);
	volatile uint32_t ulHartId;

	uxTimerIncrementsForOneTick = (size_t)((configCPU_CLOCK_HZ) / (configTICK_RATE_HZ));

	__asm volatile("csrr %0, mhartid" : "=r"(ulHartId));
	pullMachineTimerCompareRegister  = (volatile uint64_t *)(ullMachineTimerCompareRegisterBase + (ulHartId * sizeof(uint64_t)));
	#if( configUSE_TICKLESS_IDLE == 1 )
		pullMachineTimerCounterRegister = (volatile uint64_t *)(configMTIME_BASE_ADDRESS + (ulHartId * sizeof(uint64_t)));
	#endif
	do {
		ulCurrentTimeHigh = *pulTimeHigh;
		ulCurrentTimeLow = *pulTimeLow;
	} while (ulCurrentTimeHigh != *pulTimeHigh);

	ullNextTime = (uint64_t) ulCurrentTimeHigh;
	ullNextTime <<= 32ULL; /* High 4-byte word is 32-bits up. */
	ullNextTime |= (uint64_t) ulCurrentTimeLow;
	ullNextTime += (uint64_t) uxTimerIncrementsForOneTick;
	*pullMachineTimerCompareRegister = ullNextTime;

	/* Prepare the time to use after the next tick interrupt. */
	ullNextTime += (uint64_t) uxTimerIncrementsForOneTick;
}

#endif /* ( configMTIME_BASE_ADDRESS != 0 ) && ( configMTIME_BASE_ADDRESS != 0 ) */
/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler(void)
{
	extern void xPortStartFirstTask(void);

#if( configASSERT_DEFINED == 1 )
	{
		volatile uint32_t mtvec = 0;

		/* Check the least significant two bits of mtvec are 00 - indicating
		single vector mode. */
		__asm volatile("csrr %0, mtvec" : "=r"(mtvec));
		configASSERT((mtvec & 0x03UL) == 0);

		/* Check alignment of the interrupt stack - which is the same as the
		stack that was being used by main() prior to the scheduler being
		started. */
		configASSERT((xISRStackTop & portBYTE_ALIGNMENT_MASK) == 0);

#ifdef configISR_STACK_SIZE_WORDS
		{
			memset((void *) xISRStack, portISR_STACK_FILL_BYTE, sizeof(xISRStack));
		}
#endif	 /* configISR_STACK_SIZE_WORDS */
	}
#endif /* configASSERT_DEFINED */

	/* If there is a CLINT then it is ok to use the default implementation
	in this file, otherwise vPortSetupTimerInterrupt() must be implemented to
	configure whichever clock is to be used to generate the tick interrupt. */
	vPortSetupTimerInterrupt();

#if( ( configMTIME_BASE_ADDRESS != 0 ) && ( configMTIMECMP_BASE_ADDRESS != 0 ) )
	{
		/* Enable mtime and external interrupts.  1<<7 for timer interrupt, 1<<11
		for external interrupt.  _RB_ What happens here when mtime is not present as
		with pulpino? */
		__asm volatile("csrs mie, %0" :: "r"(0x880));
	}
#else
	{
		/* Enable external interrupts. */
		__asm volatile("csrs mie, %0" :: "r"(0x800));
	}
#endif /* ( configMTIME_BASE_ADDRESS != 0 ) && ( configMTIMECMP_BASE_ADDRESS != 0 ) */

	xPortStartFirstTask();

	/* Should not get here as after calling xPortStartFirstTask() only tasks
	should be executing. */
	return pdFAIL;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler(void)
{
	/* Not implemented. */
	for (;;);
}


#include "diag.h"
void vApplicationTickHook(void)
{
	return;
}

void vApplicationIdleHook(void)
{
	volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();
#if defined (CONFIG_FW_DRIVER_COEXIST) && CONFIG_FW_DRIVER_COEXIST
#if defined (CONFIG_WLAN)
	extern void wlan_driver_check_and_suspend(void);
	wlan_driver_check_and_suspend();
#endif
#endif
#if defined(CONFIG_WIFI_FW_EN) && CONFIG_WIFI_FW_EN
	extern void Wifi_FW_TaskIdle(void);
	Wifi_FW_TaskIdle();
#endif

	if (xFreeStackSpace > 100) {
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}

void vApplicationMallocFailedHook(void)
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	char *pcCurrentTask = "NoTsk";
	if( xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED )
	{
	    pcCurrentTask = pcTaskGetName( NULL );
	}
	DiagPrintf( "[%s]Malloc failed [free heap size: %d]\r\n",  pcCurrentTask, xPortGetFreeHeapSize() );
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

/*-----------------------------------------------------------*/
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	/* To avoid gcc warnings */
	( void ) pxTask;

	/* This function will be called if a task overflows its stack, if
	configCHECK_FOR_STACK_OVERFLOW != 0.  It might be that the function
	parameters have been corrupted, depending on the severity of the stack
	overflow.  When this is the case pxCurrentTCB can be inspected in the
	debugger to find the offending task. */
	DiagPrintf("\n\r[%s] STACK OVERFLOW - TaskName(%s)\n\r", __FUNCTION__, pcTaskName);
	for( ;; );
}

