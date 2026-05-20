#ifndef __MOTOR_TASK_H__
#define __MOTOR_TASK_H__


/*******************************************************************
 * External Functions
 *******************************************************************/
/**
  *@brief Motor Task Entry
  *@param stack_size: Stack size for task
  *@retval None
  */
extern void motor_task_entry(INT32U stack_size);
/**
  *@brief Start motor moving
  *@param None
  *@retval Result (0:Success Others:Failed)
  */
extern INT8U motor_task_start(void);
/**
  *@brief Get motor's current position
  *@param None
  *@retval Motor's current position
  */
extern INT32U motor_task_get_cur_pos(void);
/**
  *@brief Set motor's new position
  *@param new_pos: New position
  *@retval Result (0:Success Others:Failed)
  */
extern INT8U motor_task_set_pos(INT32U new_pos);
/**
  *@brief Get motor's ready status
  *@param None
  *@retval Status (0:Not Ready 1:Ready)
  */
extern INT8U motor_task_is_ready(void);
/**
  *@brief Start motor moving to home
  *@param None
  *@retval Result (0:Success Others:Failed)
  */
extern INT8U motor_task_home(void);
/**
  *@brief Get motor's busy status
  *@param None
  *@retval Status (0:Not busy 1:Busy)
  */
INT8U motor_task_is_busy(void);
/**
  *@brief Start motor moving to relative position (Asynchronous)
  *@param rel_pos: relative steps to move from current position
  *@retval Result (0:Success Others:Failed)
  */
INT8U motor_task_relative_move(INT32S rel_pos);
/**
  *@brief Start motor moving to absolute position (Asynchronous)
  *@param abs_pos: target absolute position to reach
  *@retval Result (0:Success Others:Failed)
  */
INT8U motor_task_absolute_move(INT32S abs_pos);
#endif //__MOTOR_TASK_H__
