/**************************************************************************
 *                                                                        *
 *         Copyright (c) 2022 by Generalplus Inc.                         *
 *                                                                        *
 *  This software is copyrighted by and is the property of Generalplus    *
 *  Inc. All rights are reserved by Generalplus Inc.                      *
 *  This software may only be used in accordance with the                 *
 *  corresponding license agreement. Any unauthorized use, duplication,   *
 *  distribution, or disclosure of this software is expressly forbidden.  *
 *                                                                        *
 *  This Copyright notice MUST not be removed or modified without prior   *
 *  written consent of Generalplus Technology Co., Ltd.                   *
 *                                                                        *
 *  Generalplus Inc. reserves the right to modify this software           *
 *  without notice.                                                       *
 *                                                                        *
 *  Generalplus Inc.                                                      *
 *  No.19, Industry E. Rd. IV, Hsinchu Science Park                       *
 *  Hsinchu City 30078, Taiwan, R.O.C.                                    *
 *                                                                        *
 **************************************************************************/
#include "project.h"
#include "drv_l1_gpio.h"
#include "cmsis_os.h"
#include "drv_l1_arm_cp15.h"
#include "drv_l1_clock.h"
#include "board_config.h"
#include "drv_l2.h"
#include "drv_l1_rtc.h"
#include "drv_l2_power_save.h"
#include "gplib.h"
#include "gplib_print_string.h"
#include "gpio_init.h"



#include "console.h"
void CmdTask(void const *param);
xTaskHandle cmdTaskHandle = NULL;
/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define STACKSIZE               32768
#define GPLIB_FILE_SYSTEM_EN    1
#define SPREAD_SPECTRUM     1
#define __SYSTEM_CLOCK    ( 171000000UL)



 /**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
uint32_t SystemCoreClock = __SYSTEM_CLOCK;      // System Core Clock Frequency and SystemCoreClock will be update by SystemInit() in main();
static int cpu_idle=0;
/**************************************************************************
 *               F U N C T I O N    D E C L A R A T I O N S               *
 **************************************************************************/
void CmdTask(void const *param);


/**************************************************************************/
extern void gpio_signal_leds_device_ready(void);
extern void Task_Manager(void);
extern void gpio_ctrl_init(void);

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/

/**************************************************************************/

#ifndef __NO_SYSTEM_INIT
#ifdef __cplusplus
extern "C"
#endif
void SystemInit(void)
{
    SystemCoreClock = drv_l1_clock_get_system_clock_freq();
}
#endif



/*-----------------------------------------------------------*/
void vApplicationTickHook( void )
{

}
/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/
void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
#if WFI_ENA == 1
	drv_l1_cp15_CpuDoIdle();
#endif
	cpu_idle++;
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;
	//DBG_PRINT("task name %s\r\n", pcTaskName);
	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
// For EMC test.
void enableSpreadSpectrum( void )
{
    osDelay(100);
    vTaskSuspendAll();
    portENTER_CRITICAL();
    Iram_copy();
    drv_l2_change_system_speed(162000000, C_CPU_DDR_SYS_642);//Change Clock 162000000 for SystemCoreClock/6 could be 27MHz
    portEXIT_CRITICAL();
    xTaskResumeAll();
}



/**
 * @brief   Initial task
 * @param   none
 * @return 	none
 */
void InitTask()
{
#if (SPREAD_SPECTRUM == 1)
    enableSpreadSpectrum();
#endif

    Task_Manager();

#if (_OPERATING_SYSTEM != _OS_NONE)
    while(1){vTaskDelay(1000);}
#else
    while (1);
#endif

}

/*********************************************************************
 *
 *  main()
 *
 *********************************************************************/
int main(void)
{
    gpio_ctrl_init();

    MMU_PTCreate();
    drv_l1_cp15_mmu_enable();
    drv_l1_cp15_cache_enable();
    SystemInit();

    board_init();

    drv_l1_init();


    DBG_PRINT("SystemCoreClock is %u, CPUCoreClock is %u\r\n", SystemCoreClock, drv_l1_clock_get_cpu_clock_freq());
    drv_l2_init();



    //DBG_PRINT("SystemCoreClock is %u\r\n", SystemCoreClock);
    //drv_l1_clock_print_major_clock_info();

#ifdef NO_FS
#else
#if (GPLIB_FILE_SYSTEM_EN == 1)
    fs_init();              // Initiate file system module
#endif
#endif
#if (_OPERATING_SYSTEM != _OS_NONE)
    osThreadDef(InitTask, osPriorityNormal, 0, STACKSIZE);
#endif

    osThreadDef(CmdTask, osPriorityAboveNormal, 0, STACKSIZE);
  	cmdTaskHandle = osThreadCreate(&os_thread_def_CmdTask, NULL);
#if (_OPERATING_SYSTEM != _OS_NONE)
    osKernelStart(&os_thread_def_InitTask, NULL);
#else
    InitTask();
#endif

    return 0;
}
void CmdTask(void const *param)
{
    DBG_PRINT("%s ... \r\n",__func__);
    Cmd_Task((void *)param);
}
