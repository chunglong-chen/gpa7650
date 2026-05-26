/************************************************************************************
 *  File:     main.c
 *  Purpose:  GP22BTesting main file.
 *            Replace with your code.
 *  Date:
 *  Info:     If __NO_SYSTEM_INIT is defined in the Build options,
 *            the startup code will not branch to SystemInit()
 *            and the function can be removed
 ************************************************************************************/
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

#if defined(GPLIB_CONSOLE_EN) && (GPLIB_CONSOLE_EN == 1)
#include "console.h"
void CmdTask(void const *param);
xTaskHandle cmdTaskHandle = NULL;
#endif

#define BENCHMARK_ALGOS 0 /* If set to 1, will run benchmarks on algorithms instead of booting normally */
#if BENCHMARK_ALGOS == 1
#include "eye_image.h"
#endif

xTaskHandle Task_Manager_Handle;

#ifdef USE_NANDSPI_INSTEAD_NANDFLASH
void spi_nand_custom(void);
#endif

#define STACKSIZE       32768

#define __SYSTEM_CLOCK    ( 171000000UL)
uint32_t SystemCoreClock = __SYSTEM_CLOCK;      // System Core Clock Frequency and SystemCoreClock will be update by SystemInit() in main();

#ifndef __NO_SYSTEM_INIT
#ifdef __cplusplus
extern "C"
#endif
void SystemInit(void)
{
    SystemCoreClock = drv_l1_clock_get_system_clock_freq();
}
#endif

#define SPREAD_SPECTRUM     1

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
static int cpu_idle=0;
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

/*-----------------------------------------------------------*/
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

extern INT32U MMU_PTCreate(void);
extern void Change_System_Pll_Demo(void);
extern void Task_Manager(void);


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

#if (CHANGE_SYSTEM_SPEED == 1)
void Change_System_Pll_Task()
{
    Change_System_Pll_Demo();
}
#endif

/*********************************************************************
 *
 *  main()
 *
 *********************************************************************/
int main()
{
    // t_rtc RTC_TIME;

    // Initiate GPIO as soon as possible after pressing the power button
    gpio_ctrl_init();

    MMU_PTCreate();
    drv_l1_cp15_mmu_enable();
    drv_l1_cp15_cache_enable();

    SystemInit();

    board_init();

    drv_l1_init();
#if defined(USE_NANDSPI_INSTEAD_NANDFLASH)
    spi_nand_custom();
#endif

    DBG_PRINT("SystemCoreClock is %u, CPUCoreClock is %u\r\n", SystemCoreClock, drv_l1_clock_get_cpu_clock_freq());
    drv_l2_init();

#ifdef NO_FS
#else
#if (GPLIB_FILE_SYSTEM_EN == 1)
    fs_init();              // Initiate file system module
#endif
#endif

#if (_OPERATING_SYSTEM != _OS_NONE)
    osThreadDef(InitTask, osPriorityNormal, 0, STACKSIZE);
#endif

#if 0 && (CHANGE_SYSTEM_SPEED == 1) // TEST
    osThreadDef(Change_System_Pll_Task, osPriorityNormal, 0, 4096);
    osThreadCreate(&os_thread_def_Change_System_Pll_Task, NULL);
#endif

    #if BENCHMARK_ALGOS == 1
    #include "GlintDetection.h"
    #include "CommonDefines.h"

    uint32_t clk_cycle_start;
    uint32_t clk_cycle_end;
    static volatile uint32_t for_loop_counter = 0;
    while (1) {
        clk_cycle_start = timerF_counter_count_get();
        for (for_loop_counter = 0; for_loop_counter < 10000000; for_loop_counter++) {
        }
        clk_cycle_end =   timerF_counter_count_get();
        DBG_PRINT("```````````` 10 mil for loop cycle count t1=%u, t2=%u, delta=%u\r\n",clk_cycle_start,clk_cycle_end,(clk_cycle_end-clk_cycle_start)*2/3);

        int32_t roi_x = ROI_X;
        int32_t roi_y = ROI_Y;
        int32_t roi_width = ROI_WIDTH;
        int32_t roi_height = ROI_HEIGHT;
        roi_t roi_settings = {roi_x, roi_y, roi_width, roi_height};
        image_t img_settings = {m_image_eye, 640, 480};

        clk_cycle_start = timerF_counter_count_get();
        glint_coords_t glint_coords = {0};

        int32_t retval = detect_glints(
                    img_settings,
                    roi_settings,
                    &glint_coords);

        clk_cycle_end =   timerF_counter_count_get();
        DBG_PRINT("```````````` glint detection cycle count t1=%u, t2=%u, delta=%u\r\n",clk_cycle_start,clk_cycle_end,(clk_cycle_end-clk_cycle_start)*2/3);

    if (retval >= 0) {
        /* I had trouble printing floating point values, so I'm casting to int for now */
        DBG_PRINT("COORDS,%d, %d, %d ,%d\r\n",
                (int32_t)(glint_coords.coords[0].x - roi_x),
                (int32_t)(glint_coords.coords[0].y - roi_y),
                (int32_t)(glint_coords.coords[1].x - roi_x),
                (int32_t)(glint_coords.coords[1].y - roi_y));
        float center_x = (glint_coords.coords[0].x + glint_coords.coords[1].x) / 2.0f;
        float center_y = (glint_coords.coords[0].y + glint_coords.coords[1].y) / 2.0f;
        float d1 = glint_coords.coords[0].x - glint_coords.coords[1].x;
        float d2 = glint_coords.coords[0].y - glint_coords.coords[1].y;
        float r = sqrtf(d1 * d1 + d2 * d2);

        DBG_PRINT("OUTPUT_IMG,%d,%d,%d\r\n", (int32_t)center_x, (int32_t)center_y, (int32_t)r);
        DBG_PRINT("OUTPUT_ROI,%d,%d,%d\r\n", (int32_t)(center_x - roi_x), (int32_t)(center_y - roi_y), (int32_t)r);
    } else {
        DBG_PRINT("FAILED, retval = %d\r\n",retval);
    }

    }
    #endif // #if BENCHMARK_ALGOS == 1


#if 1
    #if (_OPERATING_SYSTEM != _OS_NONE)
    #if defined(GPLIB_CONSOLE_EN) && (GPLIB_CONSOLE_EN == 1)
	osThreadDef(CmdTask, osPriorityAboveNormal, 0, STACKSIZE);
  	cmdTaskHandle = osThreadCreate(&os_thread_def_CmdTask, NULL);
    #endif
    #endif
#endif
    #if (_OPERATING_SYSTEM != _OS_NONE)
    osKernelStart(&os_thread_def_InitTask, &Task_Manager_Handle);
    #else
    InitTask();
    #endif
    return 0;
}

#if 1 && defined(GPLIB_CONSOLE_EN) && (GPLIB_CONSOLE_EN == 1)

extern void OS_Cmd(void);
extern void Mem_Cmd(void);

extern void FS_Demo(void);
extern void Nand_Demo(void);
extern void Pmic_Cmd(void);

void CmdTask(void const *param)
{
    DBG_PRINT("%s ... \r\n",__func__);

    #if (CMD_OS == 1)
    OS_Cmd();
    #endif

    #if (CMD_MEM == 1)
    Mem_Cmd();
    #endif

    #if FS_DEMO
        #if !(  (defined(NAND1_EN) && (NAND1_EN == 1)) || \
                (defined(NAND2_EN) && (NAND2_EN == 1)) || \
                (defined(NAND3_EN) && (NAND3_EN == 1)) || \
                (defined(NAND_APP_EN) && (NAND_APP_EN == 1)) ||\
                (defined(SD_EN) && (SD_EN == 1)) \
            )
            #error "ERR:FS_DEMO require SD_EN or NAND_EN set to 1"
        #endif
        FS_Demo();
    #endif

    #if NAND_DEMO
        #if (GPLIB_FILE_SYSTEM_EN == 0)
            #error "ERR:NAND_DEMO require GPLIB_FILE_SYSTEM_EN set to 1"
        #endif
        Nand_Demo();
    #endif

    #if (GPY021X_ENABLE_CMD ==1)
        Pmic_Cmd();
    #endif
    Cmd_Task((void *)param);
}
#endif // GPLIB_CONSOLE_EN
