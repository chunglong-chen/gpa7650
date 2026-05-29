#include "max14574.h"
#include "cmd_tx_task.h"
#include "drv_l1_i2c.h"
#include "gpio_init.h"
#include "gplib_print_string.h"
#include <math.h>

#define FAIL              -1
#define SUCCESS           0
#define TEMP_READ_FAIL    (float)-50
#define LOOKUP_TABLE_SIZE (DIOPTER_MAX * 2 + 1) // including 0D

static drv_l1_i2c_bus_handle_t MAX14574;

xQueueHandle queue_cmd_to_ll_task = NULL;

INT8S volatile m_diopter = 0;
float volatile eq2_s = 0.0;
float volatile eq2_v0d = 0.0;
// lookup table for +-17D
const INT16U VOLTAGE_LOOKUP_TABLE[LOOKUP_TABLE_SIZE] = {
    406,
    407,
    409,
    411,
    413,
    415,
    417,
    419,
    421,
    424,
    426,
    428,
    431,
    433,
    435,
    438,
    440,
    443,
    446,
    448,
    451,
    454,
    457,
    460,
    463,
    466,
    470,
    473,
    476,
    480,
    484,
    488,
    491,
    495,
    500};

const float AS_MODE_DIOPTER = -2.5664;
const float PS_MODE_DIOPTER_LOOKUP_TABLE[LOOKUP_TABLE_SIZE] = {
    -1.6092, // index 0 -17D
    -1.5483,
    -1.4874, // index 2 -15D
    -1.4188,
    -1.3577,
    -1.2889,
    -1.2201,
    -1.1434,
    -1.0744,
    -0.9976,
    -0.9284,
    -0.8436,
    -0.7665,
    -0.6893,
    -0.6042,
    -0.519,
    -0.4337,
    -0.3405, // index 17 0D
    -0.2471,
    -0.1536,
    -0.0599,
    0.0417,
    0.1436,
    0.2534,
    0.3556,
    0.4658,
    0.5842,
    0.7028,
    0.8216,
    0.9485,
    1.0838,
    1.2113,
    1.3471, // index 32 +15D
    1.4912,
    1.6437, // index 34 +17D
};

/**
 * @brief   Check Liquid Lens is enable or disable
 * @param   none
 * @return  0(Enabled) or -1(Disabled)
 */
static INT8S isLiquidLensEnabled(void) {
    if (!gpio_read_io(LL_EN)) {
        DBG_PRINT("Liquid Lens is not turned On.\r\n");
        uart2_printf("Liquid Lens is not turned On.\r\n");
        return FAIL;
    }

    return SUCCESS;
}

void max14574_error_message(INT8U reg) {
    switch (reg) {
    case MAX14574_REG_STATUS:
        uart2_printf("Reg status(%#x) error.\r\n", reg);
        break;
    case MAX14574_REG_FAIL:
        uart2_printf("Reg fail(%#x) error.\r\n", reg);
        break;
    case MAX14574_REG_TEMPSENSE:
        uart2_printf("Reg tempsensor(%#x) error.\r\n", reg);
        break;
    case MAX14574_REG_USERMODE:
        uart2_printf("Reg usermode(%#x) error.\r\n", reg);
        break;
    case MAX14574_REG_OIS_LSB:
        uart2_printf("Reg OIS_LSB(%#x) error.\r\n", reg);
        break;
    case MAX14574_REG_LLV1:
        uart2_printf("Reg LLV1(%#x) error.\r\n", reg);
        break;
    case MAX14574_REG_COMMAND:
        uart2_printf("Reg command(%#x) error.\r\n", reg);
        break;
    case MAX14574_REG_DRIVERCONF:
        uart2_printf("Reg driverconf(%#x) error.\r\n", reg);
        break;
    default:
        break;
    }
}
/**
 * @brief   Write one register
 * @param   reg: register
 * @param   data: data
 * @return  0(successful) or -1(failed)
 */
INT32S max14574_write_reg(INT8U reg, INT8U data) {
    if (isLiquidLensEnabled())
        return FAIL;
    return drv_l1_reg_1byte_data_1byte_write(&MAX14574, reg, data);
}

/**
 * @brief   Read one register
 * @param   reg: register
 * @param   out_data: output data
 * @return  0(successful) or -1(failed)
 */
INT32S max14574_read_reg(INT8U reg, INT8U* out_data) {
    if (isLiquidLensEnabled())
        return FAIL;
    return drv_l1_reg_1byte_data_1byte_read(&MAX14574, reg, out_data);
}

/**
 * @brief   set voltage to LLV1
 * @param   voltage: 0 to 1023
 * @return  0(successful) or -1(failed)
 */
INT8S max14574_set_voltage(INT16U voltage) {
    INT8U  voltage_LSB;
    INT8U  OIS_LSB_value;
    INT16U volt = voltage;

    /* Check if the value is between 0 and 1023 (10bits) */
    if (volt > MAX14574_MAX_VOLT)
        volt = MAX14574_MAX_VOLT;
    else if (volt < MAX14574_MIN_VOLT)
        volt = MAX14574_MIN_VOLT;

    /* Obtain the two LSBs and the eight MSBs of the 10bits voltage */
    voltage_LSB = (INT8U)(volt & OIS_LSB_MASK);
    OIS_LSB_value = voltage_LSB;
    volt = (INT8U)(volt >> 2);
    /* Write values on registers */
    if (max14574_write_reg(MAX14574_REG_OIS_LSB, OIS_LSB_value) < 0) {
        DBG_PRINT("Writing Error in OIS_LSB Register\r\n");
        max14574_error_message(MAX14574_REG_OIS_LSB);
        return FAIL;
    }

    // We used LLV1 only.
    if (max14574_write_reg(MAX14574_REG_LLV1, volt) < 0) {
        DBG_PRINT("Writing Error in LLV1 Register\r\n");
        max14574_error_message(MAX14574_REG_LLV1);
        return FAIL;
    }

    /* Update the value of output */
    if (max14574_write_reg(MAX14574_REG_COMMAND, COMMAND_UPD_OUT) < 0) {
        DBG_PRINT("Writing Error in COMMAND Register\r\n");
        max14574_error_message(MAX14574_REG_COMMAND);
        return FAIL;
    }
    return SUCCESS;
}

/**
 * @brief   get LLV1 voltage
 * @param   none
 * @return  volt_llv1(successful) or -1(failed)
 */
INT16S max14574_get_voltage(void) {
    INT8U  value_LSB;
    INT8U  volt;
    INT16U volt_llv1;
    if (max14574_read_reg(MAX14574_REG_OIS_LSB, &value_LSB) < 0) {
        DBG_PRINT("Reading Error in OIS_LSB Register\r\n");
        max14574_error_message(MAX14574_REG_OIS_LSB);
        return FAIL;
    }
    if (max14574_read_reg(MAX14574_REG_LLV1, &volt) < 0) {
        DBG_PRINT("Reading Error in LLV1 Register\r\n");
        max14574_error_message(MAX14574_REG_LLV1);
        return FAIL;
    }
    volt_llv1 = (volt << 2) + value_LSB;
    return volt_llv1;
}

/* Get MAX14574 chip version */
void max14574_get_chip_version(void) {
    INT8U version;
    if (max14574_read_reg(MAX14574_REG_STATUS, &version) < 0) {
        DBG_PRINT("Reading Error in STATUS Register\r\n");
        max14574_error_message(MAX14574_REG_STATUS);
        return;
    }
    /* Get the revision value with MAX14574_REG_STATUS[7;5] */
    version = (version & 0b11100000) >> 5;
    DBG_PRINT("MAX14574 Chip revision: %d\r\n", version);
}

/* Read the STATUS register and print the value of this register */
INT8U max14574_get_status(void) {
    INT8U status;
    if (max14574_read_reg(MAX14574_REG_STATUS, &status) < 0) {
        DBG_PRINT("Reading Error in STATUS Register\r\n");
        max14574_error_message(MAX14574_REG_STATUS);
        return FAIL;
    }

    if (status & STATUS_LL_TH)
        DBG_PRINT("Thermal shutdown event has occurred\r\n");

    if (status & STATUS_BST_FAIL)
        DBG_PRINT("B5 bump is not able to reach the Vpeak voltage\r\n");

    /* Keep only bit4 (LL_TH) and bit2 (BST_FAIL), other bits are useless */
    status = status & 0b00010100;
    DBG_PRINT("Status %#x\r\n", status);

    return status;
}

/* Read the FAIL register and print failure */
INT8U max14574_get_fail(void) {
    INT32S ret = 0;
    INT8U  status = 0x01;

    /* Write CHK_FAIL bit for checking status */
    ret = max14574_write_reg(MAX14574_REG_COMMAND, COMMAND_CHK_FAIL);

    if (ret < 0)
        return ret;

    /* Waiting the end of test */
    while ((status & COMMAND_CHK_FAIL) != 0)
        max14574_read_reg(MAX14574_REG_COMMAND, &status);

    /* Get the result of the test by reading FAIL register */
    max14574_read_reg(MAX14574_REG_FAIL, &status);

    if (status & FAIL_OP)
        DBG_PRINT("Open connection detected between the outputs\r\n");

    if (status & FAIL_SH)
        DBG_PRINT("Short-circuit connection detected between the outputs\r\n");

    if (status & FAIL_COM_FAIL)
        DBG_PRINT("Fail connection detected on the C1 bump (COM)\r\n");

    if (status & FAIL_LL4_FAIL)
        DBG_PRINT("Fail connection detected on the C2 bump (LL4)\r\n");

    if (status & FAIL_LL3_FAIL)
        DBG_PRINT("Fail connection detected on the C3 bump (LL3)\r\n");

    if (status & FAIL_LL2_FAIL)
        DBG_PRINT("Fail connection detected on the C4 bump (LL2)\r\n");

    if (status & FAIL_LL1_FAIL)
        DBG_PRINT("Fail connection detected on the C5 bump (LL1)\r\n");

    return status;
}

// T(K)=T(°C)+273.15
float kelvin_to_celsius(float kelvin) {
    return kelvin - 273.15;
}

/**
 * @brief   get temperature
 * @details Read V1/V2 ratio from MAX14574_REG_TEMPSENSE and compute temperature.
 * @param   none
 * @return  ret, ranging from 0x00 to 0xFF.
 *          - 0x00 Measurement not available
 *          - 0x01 1/255xVTM2
 *          - 0x02 2/255xVTM2
 *          - 0xFF VTM2 (1V typ)
 *
 *          Equation:
 *
 *               ln(R1) - ln(R2)                                                      82000
 *          B = -----------------          , B = 4330, R1 = 100K, K1 = 298.15, R2 = ---------  - 82000
 *                 1         1                                                         ret
 *               ------ - ------                                                      -----
 *                 K1       K2                                                         255
 *
 *                                1
 *          ==> K2 = --------------------------------
 *                       1        ln(R1) - ln(R2)
 *                     ------ - -------------------
 *                       K1              B
 */
float max14574_get_temperature(void) {
    INT8U ret = 0;
    if (max14574_read_reg(MAX14574_REG_TEMPSENSE, &ret) < 0) {
        DBG_PRINT("Reading Error in REG_TEMPSENSE Register\r\n");
        max14574_error_message(MAX14574_REG_TEMPSENSE);
        return TEMP_READ_FAIL;
    }

    if (ret) {
        DBG_PRINT("temp reg = %d(%#x)\r\n", ret, ret);
    } else {
        /* 0x00 = Measurement not available. Temp sensor may be broken. */
        DBG_PRINT("Temp sensor not available(may be broken).\r\n");
        uart2_printf("Temp sensor not available(may be broken).\r\n");
        return TEMP_READ_FAIL;
    }

    float d = (float)ret / 255.0;
    float rt = RESISTOR_TEMP(d);
    float lnrt = log(rt);

    float temp_kelvin = 1 / ((1 / TEMPERATRUE_K1) - ((LN_R1 - lnrt) / THERMISTOR_B_CONSTANT));
    float temp_celsius = kelvin_to_celsius(temp_kelvin);
    return temp_celsius;
}

/* Move Liquid Lens to absolute diopter with Temperature calibration */
static void motor_move_with_temp_calibration(INT8S diopter) {
    if (diopter > DIOPTER_MAX || diopter < DIOPTER_MIN) {
        uart2_printf("nak\r\n");
        return;
    }

    float temp = max14574_get_temperature();
    if (temp == TEMP_READ_FAIL) {
        uart2_printf("temp nak\r\n");
        return;
    }

    eq2_s = EQU2_S(temp);
    eq2_v0d = EQU2_V0D(temp);
    DBG_PRINT("Equ2: S(%f) = %f, V0D(%f) = %f\r\n", temp, eq2_s, temp, eq2_v0d);

    float cali_v = (PS_MODE_DIOPTER_LOOKUP_TABLE[(diopter + DIOPTER_MAX)] / eq2_s) + eq2_v0d;
    float vrms = V_TO_VRMS(cali_v);
    DBG_PRINT("%dD, V = %f, Vrms = %f(%d)\r\n", diopter, cali_v, vrms, (INT16U)vrms);
    INT32S ret = max14574_set_voltage((INT16U)vrms);
    if (!ret) {
        m_diopter = diopter;
        DBG_PRINT("move abs to %dD\r\n", m_diopter);
        uart2_printf("%dD\r\n", m_diopter);
    } else
        uart2_printf("nak\r\n");
}

/* Move Liquid Lens to absolute diopter with Temperature calibration */
static void motor_move_as_mode(void) {

    float temp = max14574_get_temperature();
    if (temp == TEMP_READ_FAIL) {
        uart2_printf("temp nak\r\n");
        return;
    }

    eq2_s = EQU2_S(temp);
    eq2_v0d = EQU2_V0D(temp);
    DBG_PRINT("Equ2: S(%f) = %f, V0D(%f) = %f\r\n", temp, eq2_s, temp, eq2_v0d);

    float cali_v = (AS_MODE_DIOPTER / eq2_s) + eq2_v0d;
    float vrms = V_TO_VRMS(cali_v);
    DBG_PRINT("AS, V = %f, Vrms = %f(%d)\r\n", cali_v, vrms, (INT16U)vrms);
    INT32S ret = max14574_set_voltage((INT16U)vrms);
    if (!ret) {
        DBG_PRINT("move to AS mode\r\n");
        uart2_printf("AS\r\n");
    } else
        uart2_printf("nak\r\n");
}

/* Setup(reset) the MAX14574 */
void max14574_setup(void) {

    /* Turn on Driver */
    if (max14574_write_reg(MAX14574_REG_USERMODE, (USERMODE_ACTIVE | USERMODE_SM)) < 0) {
        DBG_PRINT("Writing Error in USERMODE Register\r\n");
        return;
    }

    /* Set output value to zero */
    if (max14574_write_reg(MAX14574_REG_COMMAND, COMMAND_UPD_OUT) < 0) {
        DBG_PRINT("Writing Error in COMMAND Register\r\n");
        return;
    }

    /* Enable external Temperature sensor*/
    if (max14574_write_reg(MAX14574_REG_DRIVERCONF, TS_EXT) < 0) {
        DBG_PRINT("Writing Error in MAX14574_REG_DRIVERCONF Register\r\n");
        return;
    }
}

/* Enable Liquid Lens */
static void motor_enable(void) {
    gpio_set_value(LL_EN, GPIO_HIGH);
    max14574_setup();
    osDelay(10);
    motor_move_with_temp_calibration(DIOPTER_HOME);
}

/* Disable Liquid Lens */
static void motor_disable(void) {
    gpio_set_value(LL_EN, GPIO_LOW);
    DBG_PRINT("disable motor\r\n");
    ack();
}

/* Move Liquid Lens to Home(0D) */
static void motor_move_home(void) {
    motor_move_with_temp_calibration(DIOPTER_HOME);
}

/* Move Liquid Lens to relative diopter */
static void motor_move_relative(INT8S diopter) {
    INT8S abs_diopter = m_diopter + diopter;
    if (abs_diopter > DIOPTER_MAX)
        abs_diopter = DIOPTER_MAX;
    else if (abs_diopter < DIOPTER_MIN)
        abs_diopter = DIOPTER_MIN;
    motor_move_with_temp_calibration(abs_diopter);
}

/**
 * @brief   find the closest Diopter
 * @param   d: liquid lens diopter
 * @return  diopter: eye diopter -15 ~ +15
 */
INT8S findClosestDiopter(float d) {
    INT8U closest_index = 0;
    INT8U i;
    float min_diff = fabs(d - PS_MODE_DIOPTER_LOOKUP_TABLE[0]);

    for (i = 0; i < LOOKUP_TABLE_SIZE; i++) {
        float diff = fabs(d - PS_MODE_DIOPTER_LOOKUP_TABLE[i]);
        if (diff < min_diff) {
            min_diff = diff;
            closest_index = i;
        }
    }

    INT8S diopter = closest_index - DIOPTER_MAX;
    return diopter;
}

/* Retrieve the current voltage of the Liquid Lens and convert it to diopters using the lookup table */
static void motor_get_position(void) {
    INT16S ret_v = max14574_get_voltage();
    if (ret_v < 0) {
        uart2_printf("nak\r\n");
        return;
    }
    float v = VRMS_TO_V(ret_v);
    DBG_PRINT("V %f\r\n", v);

    float d = eq2_s * (v - eq2_v0d);
    DBG_PRINT("curr v %f\r\n", d);

    INT8S diopter = findClosestDiopter(d);
    DBG_PRINT("find diopter %d\r\n", diopter);
    uart2_printf("%dD\r\n", diopter);
}

void max14574_init(void) {
    DBG_PRINT("Liquid Lens init\r\n");
    MAX14574.devNumber = I2C_1;
    MAX14574.slaveAddr = MAX14574_SLAVE_ADDR;
    MAX14574.clkRate = MAX14574_I2C_CLK_RATE;
    drv_l1_i2c_init(MAX14574.devNumber);
}

void max14574_task_runner(void) {
    ll_task_message_t msg;
    queue_cmd_to_ll_task = xQueueCreate(10, sizeof(ll_task_message_t));
    // initial I2C for max14574
    max14574_init();

    while (1) {
        /* block forever until a new command is received */
        if (queue_cmd_to_ll_task && (xQueueReceive(queue_cmd_to_ll_task, &msg, portMAX_DELAY) == pdTRUE)) {
            switch (msg.ll_cmd) {
            case LL_TASK_CMD_SET_VOLTAGE:
                if (max14574_set_voltage(msg.voltage)) {
                    nak();
                } else {
                    ack();
                }
                break;
            case LL_TASK_CMD_GET_VOLTAGE: {
                INT16S ret_v = max14574_get_voltage();
                if (ret_v == FAIL)
                    nak();
                else {
                    DBG_PRINT("%d\r\n", ret_v);
                    uart2_printf("%d\r\n", ret_v);
                }
                break;
            }
            case LL_TASK_CMD_GET_STATUS:
                max14574_get_status();
                break;
            case LL_TASK_CMD_GET_CHIP_VERSION:
                max14574_get_chip_version();
                break;
            case LL_TASK_CMD_GET_FAIL:
                max14574_get_fail();
                break;
            case LL_TASK_CMD_ENABLE:
                motor_enable();
                break;
            case LL_TASK_CMD_DISABLE:
                motor_disable();
                break;
            case LL_TASK_CMD_HOME:
                motor_move_home();
                break;
            case LL_TASK_CMD_GOTO_REL:
                // DBG_PRINT("Current at %dD, move to rel %d\r\n", m_diopter, msg.diopter);
                motor_move_relative(msg.diopter);
                break;
            case LL_TASK_CMD_GET_POSITION:
                motor_get_position();
                break;
            case LL_TASK_CMD_GET_TEMPERATURE: {
                float ret = max14574_get_temperature();
                if (ret == TEMP_READ_FAIL) {
                    uart2_printf("temp nak\r\n");
                } else {
                    DBG_PRINT("%.2f\r\n", ret);
                    uart2_printf("%.2f\r\n", ret);
                }
                break;
            }
            case LL_TASK_CMD_GOTO_ABS:
            case LL_TASK_CMD_GOTO_ABS_WITH_TEMP_CALIBRATION:
                motor_move_with_temp_calibration(msg.diopter);
                break;
            case LL_TASK_CMD_AS_MODE:
                motor_move_as_mode();
                break;
            default:
                break;
            }
        }
    }
}