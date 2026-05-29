/**
 * @file    asps_switch_task.c
 * @brief   AS/PS mode switching control task.
 * @ingroup asps_switcher
 * @details
 * This module implements the control logic for switching between
 * AS  and PS  modes using a DC motor and GPIO feedback.
 *
 * It operates as a FreeRTOS task and receives control commands
 * through a message queue. The task handles mode switching,
 * status query, and GPIO state monitoring.
 *
 * Core features:
 * - Motor control for mode switching (forward/reverse)
 * - GPIO feedback monitoring (AS_CHK)
 * - Timeout protection and error handling
 * - Current mode tracking (m_current_mode)
 * - GPIO state tracking (gpio_status)
 *
 * The module processes commands asynchronously and returns
 * results via UART using ACK/NAK responses.
 */
#include "asps_switch_task.h"
#include "a_pe_25hx.h"
#include "cmd_tx_task.h"
#include "gplib_print_string.h"
#include "max14574.h"

extern xQueueHandle queue_cmd_to_ll_task;
extern xQueueHandle queue_cmd_to_new_ll_task;

INT8S volatile m_current_mode = -1;
uint8_t  gpio_status=0;
xQueueHandle queue_cmd_to_asps_switch_task = NULL;

/**
 * @brief   Switch AS/PS mode by controlling motor and GPIO feedback.
 *
 * @details
 * This function controls the DC motor to switch between AS and PS modes.
 * It monitors the AS_CHK GPIO signal to determine movement status and
 * confirms whether the target position is reached.
 *
 * The function includes:
 * - Direction control (forward/reverse)
 * - Movement monitoring using GPIO feedback
 * - Timeout protection (high-level timeout and total timeout)
 * - Stability check when reaching endpoint
 *
 * The current mode is updated only when a valid GPIO transition is detected.
 * If any timeout or abnormal condition occurs, the mode is reset to unknown.
 *
 * @param mode Target mode (AS or PS).
 * @return 0 on success, 1 on failure.
 */
static INT8U asps_switch_mode(INT8U mode) {
    INT8U   ret = 1;
    uint8_t   zero_cnt = 0;
    uint16_t  rising_cnt = 0;
    uint16_t  total_cnt = 0;
    gpio_status=0;
    /* for DQA test
    gpio_set_value(USB_HUB_RST, GPIO_HIGH);
    osDelay(5);
    */

    /* Return success if 'mode' matches 'm_current_mode' without any action. */
    if (mode == m_current_mode) {

        if(gpio_read_io(AS_CHK)==0){
            ret = 0;
            // for DQA test
            //gpio_set_value(USB_HUB_RST, GPIO_LOW);

            return ret;
        }
        else{
            ret = 1;
        }
    }
    uint8_t init_chk;
    init_chk = (uint8_t)gpio_read_io(AS_CHK);
    if (mode == ASPS_TASK_CMD_SET_AS_MODE) {
        DC_MOTOR_REVERSE;
    } else if (mode == ASPS_TASK_CMD_SET_PS_MODE) {
        DC_MOTOR_FORWARD;
    } else{
        m_current_mode=-1;
        return ret;
    }
    /* When the sensor starts to move, the states of AS_CHK will change to 1,
    and they will remain in that state until they return to 0, indicating reaching the end point.*/
    osDelay(100);
    while (1){
        uint8_t chk;
        total_cnt++;
        chk = (uint8_t)gpio_read_io(AS_CHK);
        if (chk) {// GPIO = HIGH
            gpio_status = 1;
            zero_cnt = 0;
            rising_cnt++;
            if (rising_cnt > ASPS_HIGH_TIMEOUT_CNT){
                DC_MOTOR_BRAKE;
                m_current_mode = -1;
                DBG_PRINT("High timeout error!\r\n");
                // for DQA test
                // gpio_set_value(USB_HUB_RST, GPIO_LOW);
                return ret;
            }
        }
        else{ // GPIO = LOW
            zero_cnt++;;
            if (zero_cnt >= ASPS_STABLE_LOW_CNT){
                if(init_chk==1){gpio_status = 1;}
                osDelay(100);
                DC_MOTOR_BRAKE;
                break;
            }
        }
        if (total_cnt > ASPS_TOTAL_TIMEOUT_CNT){
            DC_MOTOR_BRAKE;
            m_current_mode = -1;
            DBG_PRINT("5 sec total timeout error!\r\n");
            // for DQA test
            // gpio_set_value(USB_HUB_RST, GPIO_LOW);
            return ret;
        }
        osDelay(ASPS_LOOP_DELAY_MS);
    }
    if(gpio_status==1){
        m_current_mode = mode;
        ret = 0;
    }else{
        m_current_mode = -1;
        ret = 1;
        DBG_PRINT("GPIO did not change state\r\n");
    }
    // for DQA test
    // gpio_set_value(USB_HUB_RST, GPIO_LOW);
    return ret;
}
/**
 * @brief   Output current AS/PS mode.
 *
 * @details
 * This function prints and transmits the current mode through UART.
 * The mode is determined based on the value of m_current_mode.
 *
 * Possible outputs:
 * - "AS"
 * - "PS"
 * - "UNKNOWN"
 */
static void asps_get_mode(void) {
    switch (m_current_mode) {
    case ASPS_TASK_CMD_SET_AS_MODE:
        DBG_PRINT("AS\r\n");
        uart2_printf("AS\r\n");
        break;
    case ASPS_TASK_CMD_SET_PS_MODE:
        DBG_PRINT("PS\r\n");
        uart2_printf("PS\r\n");
        break;
    default:
        DBG_PRINT("UNKNOWN\r\n");
        uart2_printf("UNKNOWN\r\n");
        break;
    }
}
/**
 * @brief   Initialize ASPS system by performing mode switching.
 *
 * @details
 * This function performs initial calibration by switching
 * from AS mode to PS mode to ensure the system is in a known state.
 *
 * It verifies motor operation and GPIO feedback during initialization.
 */
static void asps_init(void) {
    asps_switch_mode(ASPS_TASK_CMD_SET_AS_MODE);
    osDelay(100);
    asps_switch_mode(ASPS_TASK_CMD_SET_PS_MODE);
}
/**
 * @brief   ASPS switch task handler.
 *
 * @details
 * This task receives ASPS control commands from a message queue
 * and dispatches them to corresponding operations.
 *
 * Supported operations include:
 * - Mode switching (AS / PS)
 * - Current mode query
 * - GPIO state query
 * - Combined status query
 *
 * The task processes commands asynchronously and returns results
 * via UART using ACK/NAK responses.
 *
 * It also maintains the global mode state and GPIO status.
 */
void asps_switch_task(void) {

    asps_task_message_t msg;
    queue_cmd_to_asps_switch_task = xQueueCreate(10, sizeof(asps_task_message_t));
    asps_init();

    while (1) {
        /* block forever until a new command is received */
        if (queue_cmd_to_asps_switch_task && (xQueueReceive(queue_cmd_to_asps_switch_task, &msg, portMAX_DELAY) == pdTRUE)) {
            char char_seq[16];

            snprintf(char_seq, sizeof(char_seq), "%d", msg.index);
            switch (msg.mode) {
            case ASPS_TASK_CMD_SET_AS_MODE:
            case ASPS_TASK_CMD_SET_PS_MODE:
                if (asps_switch_mode(msg.mode)){
                    nak_with_info(char_seq);
                }
                else {
                    ack_with_info(char_seq,NULL);
                }
                break;
            case ASPS_TASK_CMD_GET_MODE:
                if (m_current_mode == ASPS_TASK_CMD_SET_AS_MODE) {
                    ack_with_info(char_seq,"as");
                } else if (m_current_mode == ASPS_TASK_CMD_SET_PS_MODE) {
                    ack_with_info(char_seq,"ps");
                } else{
                    ack_with_info(char_seq,"unkown");
                }
                break;
            case ASPS_TASK_CMD_GET_PI_GPIO:
            {
                char buf[8];
                uint8_t io_e4 = gpio_read_io(AS_CHK);
                snprintf(buf, sizeof(buf), "%d", (unsigned)io_e4);
                ack_with_info(char_seq,buf);
                break;
            }
            case ASPS_TASK_CMD_GET_GPIO_STATUS:
            {
                char buf[8];
                snprintf(buf, sizeof(buf), "%d", (unsigned)gpio_status);
                ack_with_info(char_seq,buf);
                break;
            }
            case ASPS_TASK_CMD_GET_ALL:
            {
                char buf[32];
                const char *mode_str;
                if (m_current_mode == ASPS_TASK_CMD_SET_AS_MODE) {
                    mode_str = "as";
                } else if (m_current_mode == ASPS_TASK_CMD_SET_PS_MODE) {
                    mode_str = "ps";
                } else {
                    mode_str = "unkown";
                }
                uint8_t pi_gpio = (uint8_t)(gpio_read_io(AS_CHK) ? 1 : 0);
                uint8_t status = (uint8_t)(gpio_status ? 1 : 0);
                snprintf(buf, sizeof(buf),"%s %u %u",mode_str,(unsigned)pi_gpio,(unsigned)status);
                ack_with_info(char_seq, buf);
                break;
            }
            default:
                break;
            }
        }
    }
}
