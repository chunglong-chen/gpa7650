#ifndef __PSC700_CMD_H__
#define __PSC700_CMD_H__

/*******************************************************************
 * Data Types
 *******************************************************************/
typedef struct{
    INT8U cap_num;
    INT8U flash_led_en;
    INT16U flash_led_delay;
    INT16U flash_led_on;
    INT8U flash_led_current;
    INT8U ir_led_on;
    INT8U af_en;
}PSC700UARTCmdCAP_Typedef;
typedef struct{
    INT8U ir_location;
    INT16U ir_level;
}PSC700UARTCmdIR_Typedef;
typedef struct{
    INT16U dac_level;
}PSC700UARTCmdFLV_Typedef;
typedef struct{
    INT8U motor_cmd;
    INT32S position;
}PSC700UARTCmdMTR_Typedef;
/*******************************************************************
 * External Functions
 *******************************************************************/
/**
  *@brief Initialize PSC700 UART Command
  *@param None
  *@retval None
  */
extern void psc700_uart_cmd_init(void);
/**
  *@brief Send PSC700 UART Command Response - Ready
  *@param None
  *@retval None
  */
extern void psc700_uart_cmd_resp_ready(void);
/**
  *@brief Send PSC700 UART Command Response - OK
  *@param None
  *@retval None
  */
extern void psc700_uart_cmd_resp_ok(void);
/**
  *@brief Send PSC700 UART Command Response - Error
  *@param None
  *@retval None
  */
extern void psc700_uart_cmd_resp_err(void);
/**
  *@brief Send PSC700 UART Command Response - Mounted
  *@param None
  *@retval None
  */
extern void psc700_uart_cmd_resp_mounted(void);
#endif //__PSC700_CMD_H__
