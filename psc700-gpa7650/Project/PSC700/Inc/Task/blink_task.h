#ifndef __BLINK_TASK_H__
#define __BLINK_TASK_H__

#include "typedef.h"


/*******************************************************************
 * External Functions
 *******************************************************************/
/**
  *@brief Blink Task Entry
  *@param stack_size: Stack size for task
  *@retval None
  */
extern void  blink_task_entry(INT32U stack_size);

#endif //__BLINK_TASK_H__
