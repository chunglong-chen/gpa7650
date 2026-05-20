#ifndef __AF_TASK_H__
#define __AF_TASK_H__

/*******************************************************************
 * Data Types
 *******************************************************************/
typedef struct{
    INT32U num;
    INT32U *frm_buf;
    INT32U frm_len;
    INT32U *mtr_pos;
}AFInfo_Typedef;

/*******************************************************************
 * External Functions
 *******************************************************************/
/**
  *@brief AF Task Entry
  *@param stack_size: Stack size for task
  *@retval None
  */
extern void af_task_entry(INT32U stack_size);
/**
  *@brief Start AF
  *@param info: Information for AF algorithm
  *@retval Result (0:Success Others:Failed)
  */
extern INT8U af_task_start(AFInfo_Typedef info);
/**
  *@brief Register a callback to get AF completed event
  *@param *callback: To be registered callback
  *@retval None
  */
extern void af_task_register_af_complete(void (*callback)(INT32U pos));
#endif //__AF_TASK_H__
