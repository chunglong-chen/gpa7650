/**
 * @file    cmd_tx_task.c
 * @brief   UART transmit task and response helper module.
 * @ingroup uart_transceiver
 * @details
 * This module handles:
 * 1. UART2 debug message transmission via queue
 * 2. Standard response messages (ack / nak / format error)
 *
 * It uses FreeRTOS queue to buffer outgoing messages and ensures
 * proper command formatting before sending through UART2.
 */
#include "cmd_tx_task.h"
#include "FreeRTOS.h"
#include "cmd_config.h"
#include "drv_l1_uart.h"
#include "gpio_init.h"
#include "gplib_print_string.h"
#include "queue.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define PIC32_UART2_DATA R_UART2_DATA
#define CMD_SIZE         100
#define CMD_AGR_SIZE     20

#define MSG_SUCCESS      "ack\r\n"
#define MSG_FAIL         "nak\r\n"
#define MSG_FORMAT_ERR   "fmt err\r\n"
xQueueHandle queue_cmd_to_microchip = NULL;

extern void drv_l1_uart_data_send(INT8U dev_idx, INT8U data, INT8U wait);

/**
 * @brief   print debug message via uart2
 * @param   *in_fmt, ...:  character string
 * @return  none
 */
void uart2_printf(CHAR* in_fmt, ...) {
    INT8U  buff[PRINT_BUF_SIZE];
    INT32U ret = 0;

    va_list args;
    va_start(args, in_fmt);
    ret = vsnprintf((CHAR*)buff, PRINT_BUF_SIZE, in_fmt, args);

    if (ret < 0) {
        return;
    }
    va_end(args);

    if (queue_cmd_to_microchip) {
        xQueueSendToBack(queue_cmd_to_microchip, buff, pdMS_TO_TICKS(10));
    }
}

void ack(void) {
    uart2_printf(MSG_SUCCESS);
    DBG_PRINT(MSG_SUCCESS);
}

void nak(void) {
    uart2_printf(MSG_FAIL);
    DBG_PRINT(MSG_FAIL);
}

/**
 * @brief   Send acknowledge message with optional information.
 * @param   seq    Sequence string (can be NULL).
 * @param   result Result string (can be NULL).
 * @details
 * Generates formatted acknowledge messages based on input:
 *
 * 1. [seq] ack result
 * 2. [seq] ack
 * 3. ack result
 * 4. ack
 */
void ack_with_info(const char* seq, const char* result) {
    const int has_seq = (seq && *seq);
    const int has_res = (result && *result);

    if (has_seq && has_res) {
        uart2_printf("[%s] ack %s\r\n", seq, result);
        DBG_PRINT   ("[%s] ack %s\r\n", seq, result);
    } else if (has_seq) {
        uart2_printf("[%s] ack\r\n", seq);
        DBG_PRINT   ("[%s] ack\r\n", seq);
    } else if (has_res) {
        uart2_printf("ack %s\r\n", result);
        DBG_PRINT   ("ack %s\r\n", result);
    } else {
        uart2_printf("ack\r\n");
        DBG_PRINT   ("ack\r\n");
    }
}
/**
 * @brief   Send negative acknowledge message with reason.
 * @param   reason Reason string (can be NULL).
 * @details
 * If reason is provided:
 *     [reason] nak
 * Otherwise:
 *     nak
 */
void nak_with_info(const char* reason) {
    if (reason && *reason) {
        uart2_printf("[%s] %s", reason, MSG_FAIL);
        DBG_PRINT("[%s] %s", reason, MSG_FAIL);
    } else {
        uart2_printf("%s", MSG_FAIL);
        DBG_PRINT("%s", MSG_FAIL);
    }
}
void nak_with_detail_info(const char* seq,const char* result) {
    const int has_seq = (seq && *seq);
    const int has_res = (result && *result);

    if (has_seq && has_res) {
        uart2_printf("[%s] nak %s\r\n", seq, result);
        DBG_PRINT   ("[%s] nak %s\r\n", seq, result);
    } else {
        uart2_printf("%s", MSG_FAIL);
        DBG_PRINT("%s", MSG_FAIL);
    }
}
/**
 * @brief   Send format error message with reason.
 * @param   reason Reason string (can be NULL).
 * @details
 * If reason is provided:
 *     [reason] fmt err
 * Otherwise:
 *     fmt err
 */
void fmt_err_with_info(const char* reason) {
    if (reason && *reason) {
        uart2_printf("[%s] %s", reason, MSG_FORMAT_ERR);
        DBG_PRINT("[%s] %s", reason, MSG_FORMAT_ERR);
    } else {
        uart2_printf("%s", MSG_FORMAT_ERR);
        DBG_PRINT("%s", MSG_FORMAT_ERR);
    }
}
/**
 * @brief   Command transmit task for UART2.
 * @details
 * This FreeRTOS task:
 *
 * 1. Creates a transmission queue
 * 2. Waits for command data from the queue
 * 3. Concatenates command fragments until '\n' is received
 * 4. Converts command ending to "\r\n"
 * 5. Sends command byte-by-byte through UART2
 *
 * Notes:
 * - A delay is added between bytes to ensure transmission stability
 * - Buffer is cleared after each complete command
 * - Space is used to concatenate partial command fragments
 */
void cmd_tx_task(void) {

    INT8U  tx_ready = 0;
    INT8U* p_tx_buff;
    INT8U  console_command[BUF_SIZE];
    INT8U  microchip_tx_command[BUF_SIZE];
    INT32U tx_size = 0;

    // initial the console_command
    memset(console_command, '\0', sizeof(console_command));
    // initial the microchip_tx_command
    memset(microchip_tx_command, '\0', sizeof(microchip_tx_command));

    // create command queue, the message in this queue will be sent to Wurth module
    queue_cmd_to_microchip = xQueueCreate(10, sizeof(console_command));

    while (1) {
        /* block forever until a new command is received */
        if (queue_cmd_to_microchip && (xQueueReceive(queue_cmd_to_microchip, console_command, 0) == pdTRUE)) {
             //DBG_PRINT("RECV command from console : %s\r\n", console_command);

            p_tx_buff = console_command;
            while (*p_tx_buff) {
                // DBG_PRINT("%02X ", *p_tx_buff);
                if (*p_tx_buff == '\n') {
                    // if data from queue is '\n' (0x0A), means no data any more
                    tx_ready = 1;
                    break;
                }
                microchip_tx_command[tx_size] = *p_tx_buff;
                tx_size++;
                p_tx_buff++;
            }

            if (tx_ready) {
                // add 0D 0A at the tail
                microchip_tx_command[--tx_size] = '\r'; // replace last index (space) with 0D
                microchip_tx_command[++tx_size] = '\n'; // add 0A to the tail
                // DBG_PRINT("Tx %d bytes : ", (tx_size + 1));
                p_tx_buff = microchip_tx_command;
                // Start to send data
                while (*p_tx_buff) {
                    // DBG_PRINT("%02X ", *p_tx_buff);
                    PIC32_UART2_DATA = *p_tx_buff;
                    vTaskDelay(pdMS_TO_TICKS(1)); // Need delay to make sure all buffer be sent !
                    if (*p_tx_buff == '\n') {
                        break;
                    }
                    p_tx_buff++;
                }
                tx_ready = 0;
                tx_size = 0;
                memset(microchip_tx_command, '\0', sizeof(microchip_tx_command)); // clear the microchip_tx_command
            } else {
                // add space to concatenate string
                microchip_tx_command[tx_size] = ' ';
                tx_size++;
            }
        }
    }
}
