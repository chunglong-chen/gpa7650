#ifndef __CAM_IMX577_TASK_H__
#define __CAM_IMX577_TASK_H__

/*******************************************************************
 * Data Type
 *******************************************************************/
typedef enum{
    IMX577_MODE_IDLE = 0,
    IMX577_MODE_STREAMING,
    IMX577_MODE_COLLECT,
    IMX577_MODE_COLLECTED,
    IMX577_MODE_MAX,
}IMX577Mode_Typedef;

typedef enum{
    IMX577_RES_VGA = 0,
    IMX577_RES_12M,
    IMX577_RES_MAX,
}IMX577Res_Typedef;

/*******************************************************************
 * External Functions
 *******************************************************************/
/**
  *@brief Camera IMX577 Task Entry
  *@param stack_size: Stack size for task
  *@retval None
  */
extern void cam_imx577_task_entry(INT32U stack_size);
/**
  *@brief IMX577 Control - Drop liveview frame
  *@param num: Number of frame to drop
  *@retval Result (0:Success Othres:Failed)
  */
extern INT8U cam_imx577_task_ctrl_drop_frm(INT32U num);
/**
  *@brief IMX577 Control - Enter idle mode
  *@param None
  *@retval Result (0:Success Othres:Failed)
  */
extern INT8U cam_imx577_task_ctrl_idle(void);
/**
  *@brief IMX577 Control - Enter streaming mode
  *@param None
  *@retval Result (0:Success Othres:Failed)
  */
extern INT8U cam_imx577_task_ctrl_streaming(void);
/**
  *@brief IMX577 Control - Start collecting frame
  *@param num: Number of frames to collect
  *@retval Result (0:Success Othres:Failed)
  */
extern INT8U cam_imx577_task_ctrl_collect_frm(INT32U num);
/**
  *@brief IMX577 Control - Capture preparation
  *@param None
  *@retval None
  */
extern void cam_imx577_task_ctrl_pre_capture(void);
/**
  *@brief IMX577 Control - Capture
  *@param cap_num: Number of captures
  *@param flash_led_en: Enable or disable flash led
  *@param flash_led_delay: Flash LED delay time (Unit: Milliseconds)
  *@param flash_led_on: Flash LED turn-on time (Unit: Milliseconds)
  *@retval Result (0:Success Othres:Failed)
  */
extern INT8U cam_imx577_task_ctrl_capture(INT8U cap_num, INT8U flash_led_en, INT16U flash_led_delay, INT16U flash_led_on);
/**
  *@brief Get captured image
  *@param idx: Captured image index
  *@param *len: Get image length
  *@retval Image buffer address
  */
extern INT32U cam_imx577_task_get_cap(INT8U idx, INT32U *len);
/**
  *@brief Get collection data
  *@param *frm_buf: Get frame buffer addresses
  *@param *frm_len: Get a frame length
  *@param *mtr_pos: Get motor's positions
  *@retval Number of collected data
  */
extern INT32U cam_imx577_task_get_col(INT32U *frm_buf, INT32U *frm_len, INT32U *mtr_pos);
/**
  *@brief Register a callback to get Ready event
  *@param *callback: To be registered callback
  *@retval None
  */
extern void cam_imx577_task_register_get_ready(void (*callback)(void));
/**
  *@brief Register a callback to get liveview image
  *@param *callback: To be registered callback
  *@retval None
  */
extern void cam_imx577_task_register_get_liveview(void (*callback)(INT32U addr, INT32U len));
/**
  *@brief Register a callback to get collection completed event
  *@param *callback: To be registered callback
  *@retval None
  */
extern void cam_imx577_task_register_collect_complete(void (*callback)(void));
/**
  *@brief Register a callback to get capture completed event
  *@param *callback: To be registered callback
  *@retval None
  */
extern void cam_imx577_task_register_capture_complete(void (*callback)(void));
/**
  *@brief Register a callback to get motor's current position
  *@param *callback: To be registered callback
  *@retval None
  */
extern void cam_imx577_task_register_get_motor_pos(INT32U (*callback)(void));
/**
  *@brief Get current collection backup count
  *@param None
  *@retval Number of collected frames already backed up
  */
extern INT32U cam_imx577_task_get_col_backup_cnt(void);
#endif //__CAM_IMX577_TASK_H__
