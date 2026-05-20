/** @file blink_task.c
 *  @brief Blink task source file
 *
 *  This task is responsible for blinking LED (IOF8).
 */
#include "blink_task.h"
#include "drv_l1_gpio.h"

/*******************************************************************
 * Constants
 *******************************************************************/
#define BLINK_TASK_TAG              "BLINK_TASK"
#define BLINK_TASK_CYCLE            1000
#define GPA7650_LED                 IO_F8

/*******************************************************************
 * Private Variables
 *******************************************************************/
static osThreadId task_id = NULL;

/*******************************************************************
 * Private Tasks
 *******************************************************************/
/**
  *@brief Blink Task
  *@param None
  *@retval None
  */
void blink_task(void) {

    portTickType xLastWakeTimevBlink;
    DBG_PRINT("blink task start\r\n");

    xLastWakeTimevBlink = xTaskGetTickCount();

    while (1) {
        gpio_write_io(GPA7650_LED, !gpio_read_io(GPA7650_LED));
        vTaskDelayUntil(&xLastWakeTimevBlink, pdMS_TO_TICKS(BLINK_TASK_CYCLE));
    }
}

/*******************************************************************
 * Public Functions
 *******************************************************************/
/**
  *@brief Blink Task Entry
  *@param stack_size: Stack size for task
  *@retval None
  */
void blink_task_entry(INT32U stack_size)
{
    if(task_id == NULL)
    {
        osThreadDef(blink_task, osPriorityNormal, 0, stack_size);
        task_id = osThreadCreate(&os_thread_def_blink_task, NULL);
        if(task_id == NULL)
        {
            DBG_PRINT("[%s][ERROR] Creat Blink Task Failed\r\n", BLINK_TASK_TAG);
        }
    }
}