/**
 * @file    a_pe_25hx.c
 * @brief   Liquid Lens (A_PE_25HX) control task and driver implementation.
 * @ingroup liquid_lens_controller
 * @details
 * This module implements the control logic for the A_PE_25HX liquid lens,
 * including I2C communication, command handling, and task-based execution.
 *
 * Responsibilities:
 * - Register read/write via I2C interface
 * - Optical power (diopter) control
 * - Voltage control (VRMS)
 * - OP mode (optical power / voltage mode) switching
 * - Status and version query
 *
 * The module operates as a FreeRTOS task, receiving commands through
 * queue_cmd_to_new_ll_task and dispatching them to corresponding
 * control functions.
 *
 * Command flow:
 *   Console ¡÷ cmd parser ¡÷ queue_cmd_to_new_ll_task ¡÷ this module
 *
 * All responses (ACK / NAK / data) are returned via UART using
 * cmd_tx_task interface.
 */
#include "a_pe_25hx.h"
#include "cmd_tx_task.h"
#include "drv_l1_i2c.h"
#include "gp_stdlib.h"
#include "gpio_init.h"
#include "gplib_print_string.h"
#include <float.h>

#define MIIS_DBG_I2C_ENABLE 0

#if MIIS_DBG_I2C_ENABLE == 1
#define I2C_DBG_PRINT DBG_PRINT
#else
#define I2C_DBG_PRINT(...)
#endif

static drv_l1_i2c_bus_handle_t A_PE_25HX;

xQueueHandle queue_cmd_to_new_ll_task = NULL;
INT8S volatile m_current_diopter = 0;

const lens_data_t AS_MODE_LENS = {-1, -2.5695f, 34.5624};
const lens_data_t PS_MODE_LENS_TABLE[] = {
    {-17, -1.5978f, 37.01},
    {-16, -1.538f, 37.16},
    {-15, -1.4761f, 37.3152},
    {-14, -1.4183f, 37.46},
    {-13, -1.3543f, 37.62},
    {-12, -1.2904f, 37.78},
    {-11, -1.2224f, 37.95},
    {-10, -1.1544f, 38.12},
    {-9, -1.0824f, 38.3},
    {-8, -1.0103f, 38.48},
    {-7, -0.9341f, 38.67},
    {-6, -0.8579f, 38.86},
    {-5, -0.7776f, 39.06},
    {-4, -0.6933f, 39.27},
    {-3, -0.6049f, 39.49},
    {-2, -0.5164f, 39.71},
    {-1, -0.4278f, 39.93},
    {0, -0.3392f, 40.15},
    {1, -0.2586f, 40.35},
    {2, -0.1617f, 40.59},
    {3, -0.0648f, 40.83},
    {4, 0.0402f, 41.09},
    {5, 0.1413f, 41.34},
    {6, 0.2425f, 41.59},
    {7, 0.356f, 41.87},
    {8, 0.4654f, 42.14},
    {9, 0.5791f, 42.42},
    {10, 0.701f, 42.72},
    {11, 0.823f, 43.02},
    {12, 0.9492f, 43.33},
    {13, 1.0796f, 43.65},
    {14, 1.2142f, 43.98},
    {15, 1.3508f, 44.3144},
    {16, 1.5003f, 44.68},
    {17, 1.6477f, 45.04},

};

static INT32S a_pe_25hx_write_reg(INT8U* data, INT8U size) {
#if MIIS_DBG_I2C_ENABLE
    int i;
    for (i = 0; i < size; i++) {
        I2C_DBG_PRINT("txdata[%d] = %#x \r\n", i, data[i]);
    }
#endif
    INT32S ret;
    ret = drv_l1_i2c_bus_write(&A_PE_25HX, data, size);
    if (ret < 0) {
        DBG_PRINT("i2c write error\r\n");
        uart2_printf("i2c write error\r\n");
    }

    return ret;
}

static INT32S a_pe_25hx_read_reg(INT8U* data, INT8U size) {
    INT32S ret;
    ret = drv_l1_i2c_bus_read(&A_PE_25HX, data, size);
#if MIIS_DBG_I2C_ENABLE
    int i;
    for (i = 0; i < size; i++) {
        I2C_DBG_PRINT("rxdata[%d] = %#x \r\n", i, data[i]);
    }
#endif
    if (ret < 0) {
        DBG_PRINT("i2c read error\r\n");
        uart2_printf("i2c read error\r\n");
    }

    return ret;
}

/**
 * @brief   Writes a 32-bit value to a specific register via I2C.
 * @param   reg: The address of the register to be written.
 * @param   data: The 32-bit data to be written to the register.
 * @return  Returns positive on success, or a negative error code on failure.
 */
static INT32S a_pe_25hx_set_register_data(INT16U reg, INT32U data) {
    INT8U i2c_data[6];
    i2c_data[0] = (reg & 0xFF);
    i2c_data[1] = (reg >> 8) & 0xFF;
    i2c_data[2] = (data & 0xFF);
    i2c_data[3] = ((data >> 8) & 0xFF);
    i2c_data[4] = ((data >> 16) & 0xFF);
    i2c_data[5] = ((data >> 24) & 0xFF);

    return a_pe_25hx_write_reg(i2c_data, sizeof(i2c_data));
}

/**
 * @brief   Reads the value from a specific register
 * @param   reg: The address of the register to be read.
 * @param   o_data: Pointer to a variable where the read data (32-bit) will be stored.
 * @return  ret: Returns 0 on success, or a negative error code on failure.
 */
static INT32S a_pe_25hx_get_register_data(INT16U reg, INT32U* o_data) {
    INT32S ret;
    INT8U  i2c_data[2];
    i2c_data[0] = (reg & 0xFF);
    i2c_data[1] = (reg >> 8) & 0xFF;

    ret = a_pe_25hx_write_reg(i2c_data, sizeof(i2c_data));
    if (ret < 0)
        return ret;
    INT8U rx_data[4] = {0};

    ret = a_pe_25hx_read_reg(rx_data, sizeof(rx_data));
    if (ret < 0)
        return ret;

    // Combine the received bytes into a 32-bit unsigned integer
    *o_data = rx_data[0] | (rx_data[1] << 8) | (rx_data[2] << 16) | (rx_data[3] << 24);
    I2C_DBG_PRINT("[Get] data %#x\r\n", *o_data);

    return 0;
}

/* Convert float to INT32U */
static INT32U a_pe_25hx_cal_optical_power(lens_data_t data) {

    INT32U hex; // hex value of optical power
    // convert float to INT32U
    gp_memcpy((INT8S*)&hex, (INT8S*)&data.ll_diopter, sizeof(float));
    return hex;
}

/* Convert INT32U to float */
static float a_pe_25hx_convert_hex_to_float(INT32U optical_power_hex) {
    float optical_power = 0.0f;
    gp_memcpy((INT8S*)&optical_power, (INT8S*)&optical_power_hex, sizeof(float));
    return optical_power;
}


/* Enable or Disable V-temp feature, enable: TRUE / FALSE, RUE(successful) or FALSE(failed) */
static BOOLEAN a_pe_25hx_vtemp_enable(BOOLEAN enable) {
    if (a_pe_25hx_set_register_data(REG_V_TEMP, (INT32U)enable) < 0) {
        DBG_PRINT("%s V-Temp mode error\r\n", enable ? "Enable" : "Disable");
        uart2_printf("%s V-Temp mode error\r\n", enable ? "Enable" : "Disable");
        return FALSE;
    }
    return TRUE;
}

/* Reads the V-Temp register (0x0028) */
static void a_pe_25hx_get_vtemp_status(void) {
    INT32U data;
    if (a_pe_25hx_get_register_data(REG_V_TEMP, &data)) {
        DBG_PRINT("Get v-temp error\r\n");
        uart2_printf("Get v-temp error\r\n");
        return;
    }
    DBG_PRINT("%d\r\n", data);
    uart2_printf("%d\r\n", data);
}

/* Enable or Disable V-speed feature, enable: TRUE / FALSE ,TRUE(successful) or FALSE(failed) */
static BOOLEAN a_pe_25hx_vspeed_enable(BOOLEAN enable) {
    if (a_pe_25hx_set_register_data(REG_V_SPEED, (INT32U)enable) < 0) {
        DBG_PRINT("%s V-Speed mode error\r\n", enable ? "Enable" : "Disable");
        uart2_printf("%s V-Speed mode error\r\n", enable ? "Enable" : "Disable");
        return FALSE;
    }
    return TRUE;
}

/* Reads the V-Speed register (0x0080) */
static void a_pe_25hx_get_vspeed_status(void) {
    INT32U data;
    if (a_pe_25hx_get_register_data(REG_V_SPEED, &data)) {
        DBG_PRINT("Get v-speed error\r\n");
        uart2_printf("Get v-speed error\r\n");
        return;
    }
    DBG_PRINT("%d\r\n", data);
    uart2_printf("%d\r\n", data);
}

/* Move to home(0D) */
static void a_pe_25hx_move_home(void) {
    INT32S      ret;
    lens_data_t target_d = PS_MODE_LENS_TABLE[DIOPTER_HOME + DIOPTER_MAX];

    INT32U op = a_pe_25hx_cal_optical_power(target_d);
    ret = a_pe_25hx_set_register_data(REG_OPWR, op);
    if (ret < 0) {
        DBG_PRINT("move home error\r\n");
        uart2_printf("move home error\r\n");
        return;
    } else
        m_current_diopter = target_d.eye_diopter;

    DBG_PRINT("%dD\r\n", target_d.eye_diopter);
    uart2_printf("%dD\r\n", target_d.eye_diopter);
}

/* Move Liquid Lens to relative diopter */
static void a_pe_25hx_move_relative(INT8S diopter) {
    INT8S abs_diopter = m_current_diopter + diopter;
    if (abs_diopter > DIOPTER_MAX)
        abs_diopter = DIOPTER_MAX;
    else if (abs_diopter < DIOPTER_MIN)
        abs_diopter = DIOPTER_MIN;

    INT32S      ret;
    lens_data_t target_d = PS_MODE_LENS_TABLE[abs_diopter + DIOPTER_MAX];

    INT32U op = a_pe_25hx_cal_optical_power(target_d);
    ret = a_pe_25hx_set_register_data(REG_OPWR, op);
    if (ret < 0) {
        DBG_PRINT("move rel to %d error\r\n", target_d.eye_diopter);
        uart2_printf("move rel to %d error\r\n", target_d.eye_diopter);
        return;
    } else
        m_current_diopter = target_d.eye_diopter;

    DBG_PRINT("%dD\r\n", target_d.eye_diopter);
    uart2_printf("%dD\r\n", target_d.eye_diopter);
}

/* Move to a specific diopter in AS mode */
static void a_pe_25hx_move_as_mode(void) {
    INT32S ret;
    INT32U op = a_pe_25hx_cal_optical_power(AS_MODE_LENS);
    ret = a_pe_25hx_set_register_data(REG_OPWR, op);
    if (ret < 0) {
        DBG_PRINT("AS move Liquid Lens error\r\n");
        uart2_printf("AS move Liquid Lens error\r\n");
        return;
    }
    ack();
}

/* Reads the Temperature register (0x0A) */
static void a_pe_25hx_get_temperature_status(const char* char_seq) {
    INT32U data;
    if (a_pe_25hx_get_register_data(REG_TEMP, &data)) {
        nak_with_info(char_seq);                 /* [seq] nak */
        return;
    }
    float temperature;
    /* Convert INT32U to float */
    gp_memcpy((INT8S*)&temperature, (INT8S*)&data, sizeof(float));
    char result[16];
    snprintf(result, sizeof(result), "%.2f", temperature);

    ack_with_info(char_seq, result);            /* [seq] ack <temperature> */
}

/* Reads the status register (0x0618) */
static void a_pe_25hx_get_status(void) {
    INT32U data;
    if (a_pe_25hx_get_register_data(REG_STATUS, &data)) {
        DBG_PRINT("Get 0x0618 error\r\n");
        uart2_printf("Get 0x0618 error\r\n");
        return;
    }

    DBG_PRINT("Data in %#x is %#x\r\n", REG_STATUS, data);
    uart2_printf("Data in %#x is %#x\r\n", REG_STATUS, data);
}

/* Reads the fail register (0x0619) */
static void a_pe_25hx_get_fail(void) {
    INT32U data;
    if (a_pe_25hx_get_register_data(REG_FAIL, &data)) {
        DBG_PRINT("Get fail error\r\n");
        uart2_printf("Get fail error\r\n");
        return;
    }

    DBG_PRINT("Data in %#x is %#x\r\n", REG_FAIL, data);
    uart2_printf("Data in %#x is %#x\r\n", REG_FAIL, data);
}


/**
 * @brief Read Liquid Lens version and return result.
 *
 * Grammar:
 *   ll info version #[seq]
 *
 * Rules:
 *   - Read failure     -> [seq] nak
 *   - Success          -> [seq] ack "<version>"
 *
 * @details
 * This function reads the REG_VERSION register via I2C.
 * On success, the version value is formatted as a decimal string
 * and returned through ACK response.
 * On failure, a NAK response is returned.
 *
 * @param char_seq Sequence string (e.g., "123").
 */
static void a_pe_25hx_get_version(const char* char_seq) {
    INT32U data;
    if (a_pe_25hx_get_register_data(REG_VERSION, &data)) {
        DBG_PRINT("Get version error\r\n");
        nak_with_info(char_seq);                 /* [seq] nak */
        return;
    }
    char result[8];
    snprintf(result, sizeof(result), "%u", (unsigned)data);
    ack_with_info(char_seq, result);
}
/**
 * @brief   The Liquid lens is controlled using Optical Power or using voltage
 * @param   enable: TRUE (using Optical Power) / FALSE (using voltage)
 * @return  TRUE(successful) or FALSE(failed)
 */
static BOOLEAN a_pe_25hx_op_mode_enable(BOOLEAN enable) {
    if (a_pe_25hx_set_register_data(REG_CONTROL_MODE, (INT32U)enable) < 0) {
        DBG_PRINT("%s OP mode error\r\n", enable ? "Enable" : "Disable");
        uart2_printf("%s OP mode error\r\n", enable ? "Enable" : "Disable");
        return FALSE;
    }
    return TRUE;
}

/* Reads the Control mode register (0x0620) */
/**
 * @brief Read OP mode status from REG_CONTROL_MODE and ACK the result.
 *
 * Grammar:
 *   ll info getmo #[seq]
 * Behavior:
 *   - On read failure               -> [seq] nak
 *   - On success with value {0,1}   -> [seq] ack "<0|1>"
 *
 * @param char_seq Sequence string for ACK/NAK (e.g., "123").
 * @return void (N/A)
 */
static void a_pe_25hx_get_op_mode_status(const char* char_seq) {
    INT32U data = 0;

    /* Read register */
    if (a_pe_25hx_get_register_data(REG_CONTROL_MODE, &data) != 0) {
        DBG_PRINT("Get control mode error\r\n");
        nak_with_info(char_seq);                 /* [seq] nak */
        return;
    }
    /* Format result as string and ACK */
    char result[8];
    snprintf(result, sizeof(result), "%u", (unsigned)data);
    ack_with_info(char_seq, result);             /* [seq] ack "0" or "1" */
}

/**
 * @brief Set Liquid Lens voltage (VRMS).
 *
 * Grammar:
 *    ll setv <float:24.0~70.0> #[seq]
 *
 * Rules:
 *   - Out-of-range voltage -> [seq] fmt err
 *   - Write failure        -> [seq] nak
 *   - Success              -> [seq] ack
 *
 * @param char_seq Sequence string.
 * @param voltage  Target voltage (VRMS).
 */
static void a_pe_25hx_set_voltage(const char* char_seq, double voltage)
{
    if (voltage < MIN_VRMS || voltage > MAX_VRMS) {
        DBG_PRINT("invalid voltage %f\r\n", voltage);
        fmt_err_with_info(char_seq);             /* [seq] format error */
        return;
    }

    INT32U code = VRMS_TO_CODE(voltage);

    if (a_pe_25hx_set_register_data(REG_FOCUS, code) < 0) {
        DBG_PRINT("Set voltage error\r\n");
        nak_with_info(char_seq);                 /* [seq] nak */
        return;
    }

    ack_with_info(char_seq, NULL);               /* [seq] ack */
}

/**
 * @brief   Read the Liquid Lens FOCUS register (0x0000) and reply with ACK/NAK.
 *
 * @details Reads REG_FOCUS via a_pe_25hx_get_register_data(). On success,
 *          formats the value as a decimal string (VRMS) and prints:
 *              [<seq>] ack "<voltage>"
 *          On read failure, prints a debug error and:
 *              [<seq>] nak
 *
 * @param   char_seq  Sequence string to echo in the response (e.g., "123").
 * @note    The register value is treated as unsigned 32-bit and formatted in
 *          decimal. Change formatting if engineering units or hexadecimal
 *          representation is desired.
 * @return  void (N/A)
 */
static void a_pe_25hx_get_voltage(const char* char_seq) {

    INT32U data;

    if (a_pe_25hx_get_register_data(REG_FOCUS, &data)) {
        DBG_PRINT("Get voltage error\r\n");
        nak_with_info(char_seq);                 /* [seq] nak */
        return;
    }
    float voltage = CODE_TO_VRMS(data);
    char result[16];
    snprintf(result, sizeof(result), "%f", voltage);
    ack_with_info(char_seq, result);             /* [seq] ack <voltage> */
}
/**
 * @brief  Read Optical Power register (0x001E).
 * @return Register value on success, UINT32_MAX on failure.
 */
static INT32U a_pe_25hx_get_optical_power(void) {
    INT32U data;
    if (a_pe_25hx_get_register_data(REG_OPWR, &data)) {
        DBG_PRINT("Get optical power error\r\n");
        return UINT32_MAX;
    }

    return data;
}

/**
 * @brief   Move Liquid Lens to an absolute diopter and reply with ACK/NAK.
 *
 * @details Converts target_power to diopters via a_pe_25hx_convert_hex_to_float()
 *          and checks range [A_PE_25HX_DIOPTER_MIN, A_PE_25HX_DIOPTER_MAX].
 *          - Out-of-range: prints debug message and replies with
 *                [<seq>] <format error>
 *            via fmt_err_with_info().
 *          - Writes target to REG_OPWR. On write failure: replies
 *                [<seq>] nak
 *            via nak_with_info().
 *          - On write success: polls a_pe_25hx_get_optical_power() up to a
 *            retry limit (5 ms between attempts) until it matches target.
 *            Read error (UINT32_MAX) is treated as failure.
 *          - Match -> [<seq>] ack ; no match after retries -> [<seq>] nak.
 *
 * @param   char_seq      Sequence string to echo in the response (e.g., "123").
 * @param   target_power  Raw register code for optical power (UINT32).
 * @note    Tune retry count/delay to your device dynamics.
 * @return  void (N/A)
 */
static void a_pe_25hx_move_absolute(const char* char_seq,INT32U target_power) {

    INT8U  retry_cnt = 0; // Retry counter for handling failed attempts
    INT32S ret;
    float  calculated_diopter;
    calculated_diopter = a_pe_25hx_convert_hex_to_float(target_power);

    if (calculated_diopter < A_PE_25HX_DIOPTER_MIN || calculated_diopter > A_PE_25HX_DIOPTER_MAX) {
        DBG_PRINT("Invalid diopter\r\n");
        fmt_err_with_info(char_seq);                 /* [seq] format error */
        return;
    }
    ret = a_pe_25hx_set_register_data(REG_OPWR, target_power);
    if (ret < 0) {
        DBG_PRINT("Write Failed\r\n");
        nak_with_info(char_seq);                 /* [seq] nak */
    } else {
        osDelay(5); // delay 5ms
        INT32U current_power;
        // Start a while loop to check if the read optical power value matches the target optical power
        while ((current_power = a_pe_25hx_get_optical_power()) != target_power) {
            retry_cnt++;
            if (retry_cnt >= 20) {
                DBG_PRINT("Retry limit reached\r\n");
                break;
            }
            // If optical power does not match, retry writing the target power value
            a_pe_25hx_set_register_data(REG_OPWR, target_power);
            osDelay(5);
        }

        // check if the optical power matches the desired value
        if (current_power == target_power) {
            DBG_PRINT("Succeed\r\n");
            ack_with_info(char_seq, NULL);           /* [seq] ack */
        } else {
            DBG_PRINT("Failed\r\n");
            nak_with_info(char_seq);                 /* [seq] nak */
        }
    }
}

/* Reads the Optical Power register (0x001E) and converts it to the corresponding eye diopter. */
static const lens_data_t* a_pe_25hx_get_eye_diopter(void) {
    INT32U power = a_pe_25hx_get_optical_power();
    if (power == UINT32_MAX) {
        return NULL;
    }
    float ll_diopter = a_pe_25hx_convert_hex_to_float(power);
    // find eye_diopter in PS_MODE_LENS_TABLE
    int i;
    for (i = 0; i < (sizeof(PS_MODE_LENS_TABLE) / sizeof(lens_data_t)); i++) {
        if (PS_MODE_LENS_TABLE[i].ll_diopter == ll_diopter) {

            return &PS_MODE_LENS_TABLE[i];
        }
    }
    return NULL;
}
static void a_pe_25hx_init(void) {
    A_PE_25HX.devNumber = I2C_0;
    A_PE_25HX.slaveAddr = A_PE_25HX_SLAVE_ADDR;
    A_PE_25HX.clkRate = A_PE_25HX_I2C_CLK_RATE;
    drv_l1_i2c_init(A_PE_25HX.devNumber);
}

void a_pe_25hx_task_runner(void) {
    a_pe_25hx_task_message_t msg;
    queue_cmd_to_new_ll_task = xQueueCreate(10, sizeof(a_pe_25hx_task_message_t));
    a_pe_25hx_init();

    while (1) {
        /* block forever until a new command is received */
        if (queue_cmd_to_new_ll_task && (xQueueReceive(queue_cmd_to_new_ll_task, &msg, portMAX_DELAY) == pdTRUE)) {
            char char_seq[16];
            snprintf(char_seq, sizeof(char_seq), "%d", msg.index);
            switch (msg.ll_cmd) {
            case A_PE_25HX_TASK_CMD_GET_VERSION:
                a_pe_25hx_get_version(char_seq);
                break;
            case A_PE_25HX_TASK_CMD_SET_VOLTAGE:
                a_pe_25hx_set_voltage(char_seq,msg.voltage);
                break;
            case A_PE_25HX_TASK_CMD_GET_VOLTAGE:
                a_pe_25hx_get_voltage(char_seq);
                break;
            case A_PE_25HX_TASK_CMD_GOTO_ABS:
                a_pe_25hx_move_absolute(char_seq,msg.diopter_hex);
                break;
            case A_PE_25HX_TASK_CMD_GET_OPTICAL_POWER: {
                INT32U current_power = a_pe_25hx_get_optical_power();
                if (current_power == UINT32_MAX) {
                    DBG_PRINT("Failed\r\n");
                    nak_with_info(char_seq);
                } else {
                    char char_current[16];
                    snprintf(char_current, sizeof(char_current), "%x", current_power);
                    ack_with_info(char_seq,char_current);
                }
                break;
            }
            case A_PE_25HX_TASK_CMD_OP_MODE_CTRL:
                if (a_pe_25hx_op_mode_enable(msg.enable)) {
                    ack_with_info(char_seq,NULL);
                }
                else{
                    nak_with_info(char_seq);
                }
                break;
            case A_PE_25HX_TASK_CMD_GET_OP_MODE_STATUS:
                a_pe_25hx_get_op_mode_status(char_seq);
                break;
            /// The following functions are unused, reserved for future use. ///
            case A_PE_25HX_TASK_CMD_HOME:
                a_pe_25hx_move_home();
                break;
            case A_PE_25HX_TASK_CMD_GOTO_REL:
                a_pe_25hx_move_relative(msg.diopter);
                break;
            case A_PE_25HX_TASK_CMD_GET_POSITION: {
                const lens_data_t* ret = a_pe_25hx_get_eye_diopter();
                if (ret == NULL) {
                    DBG_PRINT("get diopter error\r\n");
                    uart2_printf("get diopter error\r\n");
                } else {
                    DBG_PRINT("%dD\r\n", ret->eye_diopter);
                    uart2_printf("%dD\r\n", ret->eye_diopter);
                }
                break;
            }
            case A_PE_25HX_TASK_CMD_V_TEMP_CTRL:
                if (a_pe_25hx_vtemp_enable(msg.enable)) {
                    ack();
                }
                break;
            case A_PE_25HX_TASK_CMD_V_SPEED_CTRL:
                if (a_pe_25hx_vspeed_enable(msg.enable)) {
                    ack();
                }
                break;
            case A_PE_25HX_TASK_CMD_GET_V_TEMP_STATUS:
                a_pe_25hx_get_vtemp_status();
                break;
            case A_PE_25HX_TASK_CMD_GET_V_SPEED_STATUS:
                a_pe_25hx_get_vspeed_status();
                break;
            case A_PE_25HX_TASK_CMD_AS_MODE:
                a_pe_25hx_move_as_mode();
                break;
            case A_PE_25HX_TASK_CMD_GET_TEMPERATURE:
                a_pe_25hx_get_temperature_status(char_seq);
                break;
            case A_PE_25HX_TASK_CMD_GET_STATUS:
                a_pe_25hx_get_status();
                break;
            case A_PE_25HX_TASK_CMD_GET_FAIL:
                a_pe_25hx_get_fail();
                break;
            default:
                break;
            }
        }
    }
}
