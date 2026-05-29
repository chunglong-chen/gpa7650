/**
 * @file    cmd_rx_task.c
 * @brief   UART receive task and command processing module.
 * @ingroup uart_transceiver
 * @details
 * This module handles:
 * 1. UART2 initialization for receiving data
 * 2. Receiving incoming data from UART2
 * 3. Forwarding received data to command parser
 *
 * It continuously polls UART2 and processes incoming data
 * in a periodic FreeRTOS task.
 */
#include "cmd_rx_task.h"
#include "FreeRTOS.h"
#include "cmd_config.h"
#include "console.h"
#include "drv_l1_uart.h"
#include "gpio_init.h"
#include "gplib_print_string.h"
#include "queue.h"
#include "typedef.h"

extern void   drv_l1_uart2_init(void);
extern void   drv_l1_uart2_buad_rate_set(INT32U);
extern void   drv_l1_uart2_fifo_enable(void);
extern void   drv_l1_uart2_tx_enable(void);
extern void   drv_l1_uart2_rx_enable(void);
extern INT32S drv_l1_uart_data_get(INT8U dev_idx, INT8U* data, INT8U wait);

INT8U  m_cmd_rx_buff[BUF_SIZE];
INT32U m_rx_buffer_data_num;

/**
 * @brief   initial uart2
 * @param   none
 * @return  none
 */
void cmd_rx_initial(void) {
    m_rx_buffer_data_num = 0;
    // Steup UART connection to BT module, UART2, baud rate 115200
    drv_l1_uart2_init();
    drv_l1_uart2_buad_rate_set(UART_BAUD_RATE);
    drv_l1_uart2_fifo_enable();
    drv_l1_uart2_tx_enable();
    drv_l1_uart2_rx_enable();
}

/**
 * @brief   Receive data from UART2.
 * @return  Number of bytes received.
 * @details
 * This function continuously reads data from UART2
 * until no more data is available.
 *
 * All received bytes are stored in m_cmd_rx_buff,
 * and the total length is returned.
 */
INT32U cmd_rx_rcv_data(void) {
    INT8U  data;
    INT8U  print_flag = 0;
    INT32U length = 0;

    while (drv_l1_uart_data_get(UART2_DEV, &data, 0) == STATUS_OK) {
        if (!print_flag) {
            print_flag = 1;
             //DBG_PRINT("\r\nRx: ");
        }
         //DBG_PRINT("%02X ", data);

        m_cmd_rx_buff[length] = data;
        length++;
    }
    return length;
}
/**
 * @brief   Process received UART2 data.
 * @details
 * This function:
 * 1. Calls cmd_rx_rcv_data() to receive data
 * 2. Parses received buffer byte-by-byte
 * 3. Sends each byte to command parser (cmdMonitor_UART2)
 *
 * After processing, the receive buffer is cleared.
 */
void cmd_rx_service(void) {
    INT32U ret = 0;
    INT8U* data;
    ret = cmd_rx_rcv_data();

    if (ret) {
        // DBG_PRINT(", %d bytes.\r\n", m_rx_buffer_data_num);
        // DBG_PRINT(", %d bytes.\r\n", ret);

        // DBG_PRINT("%s\r\n", m_cmd_rx_buff);

        data = &m_cmd_rx_buff[0]; // set data pointer to first address
        while (*data != '\0') {
            // DBG_PRINT("%02X", *data);
            cmdMonitor_UART2(*data);
            ++data;
        }
        // DBG_PRINT(", Done\r\n");
        memset(m_cmd_rx_buff, '\0', sizeof(m_cmd_rx_buff)); // clear the m_cmd_rx_buff
    }
}
/**
 * @brief   UART receive task.
 * @details
 * This FreeRTOS task:
 *
 * 1. Initializes UART2
 * 2. Periodically calls cmd_rx_service()
 * 3. Runs at a fixed cycle using vTaskDelayUntil
 *
 * This ensures continuous and stable UART data reception.
 */
void cmd_rx_task(void) {

    portTickType xLastWakeTimevRx;

    cmd_rx_initial();

    xLastWakeTimevRx = xTaskGetTickCount();

    while (1) {

        cmd_rx_service();

        vTaskDelayUntil(&xLastWakeTimevRx, pdMS_TO_TICKS(CMD_RX_TASK_CYCLE));
    }
}
