/**
 * @file    miis_3axis_task.c
 * @brief   MIIS 3-axis motor control task and driver implementation.
 * @ingroup three_axis_controller
 * @details
 * This module implements the control logic for the MIIS 3-axis motor system.
 * It provides I2C-based command communication, motion control, position query,
 * zero-point setting, reset handling, and task-based command processing.
 *
 * The module operates as a FreeRTOS task and receives motion control commands
 * through a message queue. It is responsible for dispatching commands to the
 * corresponding motor control functions and returning results through UART.
 */
#include "miis_3axis_task.h"
#include "cmd_tx_task.h"
#include "drv_l1_i2c.h"
#include "gplib_print_string.h"

#define WAIT_INTERVAL       (50)
#define I2C_ERROR_THRESHOLD 5 // Threshold of consecutive I2C errors before triggering a reset

static drv_l1_i2c_bus_handle_t TCA9535A;

xQueueHandle queue_cmd_to_3axis_task = NULL;

INT8U error_cnt = 0; // Counts consecutive I2C errors, triggers reset when threshold is reached

/**
 * @brief   Display the control and status register values of I2C1.
 * @param   None
 * @return  None
 */
void miis_3axis_show_i2c_reg_info(void) {
    i2cReg_t* pI2CDev = (i2cReg_t*)I2C1_BASE;
    DBG_PRINT("Reg ctrl, status (%#x,%#x)\r\n", (pI2CDev->ICCR), (pI2CDev->ICSR));
}

/**
 * @brief   Perform a software reset on the I2C1 bus and clear the error counter.
 * @param   None
 * @return  None
 */
void miis_3axis_reset_i2c_bus(void) {

    if (drv_l1_i2c_software_reset(TCA9535A.devNumber)) {
        DBG_PRINT("Reset I2C%d failed.\r\n", TCA9535A.devNumber);
        uart2_printf("Reset I2C%d failed.\r\n", TCA9535A.devNumber);
    } else {
        error_cnt = 0;
        DBG_PRINT("Reset I2C%d successful.\r\n", TCA9535A.devNumber);
        uart2_printf("Reset I2C%d successful.\r\n", TCA9535A.devNumber);
    }
}

INT32S miis_3axis_write_reg(INT8U* data, INT8U size) {
    INT32S ret;
    ret = drv_l1_i2c_bus_write(&TCA9535A, data, size);
    if (ret < 0)
        error_cnt++;

    if (error_cnt >= I2C_ERROR_THRESHOLD)
        miis_3axis_reset_i2c_bus();
    return ret;
}

INT32S miis_3axis_read_reg(INT8U* data, INT8U size) {
    INT32S ret;
    ret = drv_l1_i2c_bus_read(&TCA9535A, data, size);
    if (ret < 0)
        error_cnt++;

    if (error_cnt >= I2C_ERROR_THRESHOLD)
        miis_3axis_reset_i2c_bus();
    return ret;
}

/**
 * @brief   Get 3Axis status
 *          -----------------------------------------------
 *          | bit  |      2     |      1     |      0     |
 *          -----------------------------------------------
 *          |state |  far limit | near limit |    busy    |
 *          -----------------------------------------------
 * @param   data.axes: which motor will be inquired
 * @return  miis_3axis_data_t
 */
miis_3axis_data_t miis_3axis_get_status(miis_3axis_data_t data) {
    INT32S            ret;
    INT32U            idx = 0;
    INT8U             i2c_data[1];
    miis_3axis_data_t status;
    if (data.axes == 0 || data.axes > (MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS | MIIS_3X_Z_AXIS)) {
        status.x = -1;
        status.y = -1;
        status.z = -1;
        DBG_PRINT("Invalid axes %#x\r\n", data.axes);
        nak();
        return status;
    }
    status.axes = data.axes;
    i2c_data[0] = (MIIS_3X_STATUS << 4) | data.axes;
    miis_3axis_write_reg(i2c_data, sizeof(i2c_data));
    osDelay(pdMS_TO_TICKS(WAIT_INTERVAL));
    INT8U  status_data[8] = {0};
    INT32U size = ((data.axes >> 0) & 0x1) + ((data.axes >> 1) & 0x1) + ((data.axes >> 2) & 0x1); // How many axes would you like to inquire about

    ret = miis_3axis_read_reg(status_data, size);
    if (ret < 0) {
        status.x = -1;
        status.y = -1;
        status.z = -1;
        DBG_PRINT("3as Status Error\r\n");
        uart2_printf("3as Status Error\r\n");
        miis_3axis_show_i2c_reg_info();
    } else {

        status.x = 0;
        status.y = 0;
        status.z = 0;

        if (data.axes & MIIS_3X_X_AXIS)
            status.x = status_data[idx++] & 0xFF;
        if (data.axes & MIIS_3X_Y_AXIS)
            status.y = status_data[idx++] & 0xFF;
        if (data.axes & MIIS_3X_Z_AXIS)
            status.z = status_data[idx++] & 0xFF;
    }
    if (MIIS_3X_AT_NEAR_LIM(status.x)) {
        DBG_PRINT("X at near limit\r\n");
    }

    if (MIIS_3X_AT_FAR_LIM(status.x)) {
        DBG_PRINT("X at far limit\r\n");
    }
    if (!MIIS_3X_AT_NEAR_LIM(status.x) && !MIIS_3X_AT_FAR_LIM(status.x)) {
        DBG_PRINT("X axis is not at any limit\r\n");
    }
    return status;
}

/**
 * @brief   Move 3Axis to Home, steps:
 *          1.Move Z-axis to absolute position
 *          2.Move X and Y axis to home
 *          3.Move Z to center
 * @param   data.axes: which motor will be moved to home
 * @return  0(successful) or -1(failed)
 */
INT32S miis_3axis_move_home(miis_3axis_data_t data) {
    INT32S ret = -1;
    if (data.axes == 0 || data.axes > (MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS | MIIS_3X_Z_AXIS)) {
        DBG_PRINT("Invalid axes %#x\r\n", data.axes);
        nak();
        return ret;
    }
    INT8U i2c_data[1];

    if (data.z < 0)
        data.z = -(data.z);

    // Move Z back to absolute position data.z
    miis_3axis_message_t msg_z_back = {
        .cmd = MIIS_3X_MOVE_ABS,
        .data = {
            .axes = MIIS_3X_Z_AXIS, // Move Z to absolute.
            .x = 0,
            .y = 0,
            .z = data.z}};
    ret = miis_3axis_move(msg_z_back.cmd, msg_z_back.data);

    if (ret < 0) {
        DBG_PRINT("move z back error\r\n");
        return ret;
    }

    i2c_data[0] = (MIIS_3X_HOME << 4) | MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS; // move x, y to home
    ret = miis_3axis_write_reg(i2c_data, sizeof(i2c_data));
    if (ret < 0) {
        DBG_PRINT("move xy to home error\r\n");
        return ret;
    }

    miis_3axis_data_t tmp;
    tmp.axes = MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS; // query status of x-axis and y-axis
    miis_3axis_wait(tmp);

    i2c_data[0] = (MIIS_3X_HOME << 4) | MIIS_3X_Z_AXIS; // move z to home first
    ret = miis_3axis_write_reg(i2c_data, sizeof(i2c_data));
    if (ret < 0) {
        DBG_PRINT("move z to home error\r\n");
        return ret;
    }
    tmp.axes = MIIS_3X_Z_AXIS;
    miis_3axis_wait(tmp);

    DBG_PRINT("3as Home\r\n");

    return ret;
}
/**
 * @brief   Set selected axes to zero position.
 *
 * @details
 * This function sends the zero-setting command to the 3-axis controller
 * for the specified axes, then polls the axis status until all selected
 * motors are no longer busy.
 *
 * It validates the axis mask before issuing the command and returns
 * an error if the axis selection is invalid or if the operation times out.
 *
 * @param data Axis selection and related motion data.
 * @return Operation result. Returns non-negative on success, or negative on failure.
 */
INT32S miis_3axis_set_zero(miis_3axis_data_t data) {
    INT32S ret = -1;
    if (data.axes == 0 || data.axes > (MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS | MIIS_3X_Z_AXIS)) {
        DBG_PRINT("Invalid axes %#x\r\n", data.axes);
        return ret;
    }
    INT8U i2c_data[1];
    i2c_data[0] = (MIIS_3X_SET_ZERO << 4) | data.axes; // set x/y/z to zero
    ret = miis_3axis_write_reg(i2c_data, sizeof(i2c_data));
    if (ret < 0) {
        DBG_PRINT("set x/y/z to zero error\r\n");
        return ret;
    }
    osDelay(pdMS_TO_TICKS(WAIT_INTERVAL)); // delay to wait status change
    miis_3axis_data_t tmp;
    tmp.axes = MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS | MIIS_3X_Z_AXIS; // query status of z-axis
    miis_3axis_data_t status = miis_3axis_get_status(tmp);
    int tries = 0;
    const int MAX_POLL = 50;
    while (MIIS_3X_IS_BUSY(status.x) || MIIS_3X_IS_BUSY(status.y) || MIIS_3X_IS_BUSY(status.z)) {
        if (++tries >= MAX_POLL) {
            DBG_PRINT("timeout: still busy\r\n");
            return -1;
        }
        osDelay(pdMS_TO_TICKS(WAIT_INTERVAL)); // delay to query status
        status = miis_3axis_get_status(tmp);
    }
    DBG_PRINT("3as set zero\r\n");

    return ret;
}

/**
 * @brief   Get 3Axis module version
 * @param   none
 * @return  0(successful) or -1(failed)
 */
INT32S miis_3axis_get_fw_version(const char* char_seq) {
    INT32S ret;
    INT8U  i2c_data[1];
    i2c_data[0] = (MIIS_3X_FW_VERSION << 4);
    miis_3axis_write_reg(i2c_data, sizeof(i2c_data));
    INT8U version[64] = {0};
    ret = miis_3axis_read_reg(version, sizeof(version));
    if(ret<0){
        nak_with_info(char_seq);
    }else{
        DBG_PRINT("3as ver:%s\r\n", version);
        ack_with_info(char_seq, (const char*)version);
    }
    return ret;
}

/**
 * @brief   Get 3Axis position
 * @param   data.axes: which motor will be inquired
 * @return  0(successful) or -1(failed)
 */
INT32S miis_3axis_get_pos(const char* seq, miis_3axis_data_t data) {
    INT32S ret = -1;
    if (data.axes == 0 || data.axes > (MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS | MIIS_3X_Z_AXIS)) {
        DBG_PRINT("Invalid axes %#x\r\n", data.axes);
        if (seq && *seq) nak_with_info(seq);
        return ret;
    }

    INT8U i2c_data[1];
    i2c_data[0] = (MIIS_3X_POS << 4) | data.axes;
    miis_3axis_write_reg(i2c_data, sizeof(i2c_data));

    INT8U  pos_data[16] = {0};
    INT32U size = ((data.axes) & 1) * 4 + ((data.axes >> 1) & 1) * 4 + ((data.axes >> 2) & 1) * 4;
    ret = miis_3axis_read_reg(pos_data, size);
    if (ret < 0) {
        DBG_PRINT("3as Error\r\n");
        if (seq && *seq) nak_with_info(seq);
        return ret;
    }
    miis_3axis_data_t data_ret = {data.axes, 0, 0, 0};
    INT32U            idx = 0;
    if (data.axes & MIIS_3X_X_AXIS) {
        data_ret.x = (pos_data[idx++] << 24);
        data_ret.x |= (pos_data[idx++] << 16);
        data_ret.x |= (pos_data[idx++] << 8);
        data_ret.x |= pos_data[idx++];
    }
    if (data.axes & MIIS_3X_Y_AXIS) {
        data_ret.y = (pos_data[idx++] << 24);
        data_ret.y |= (pos_data[idx++] << 16);
        data_ret.y |= (pos_data[idx++] << 8);
        data_ret.y |= pos_data[idx++];
    }
    if (data.axes & MIIS_3X_Z_AXIS) {
        data_ret.z = (pos_data[idx++] << 24);
        data_ret.z |= (pos_data[idx++] << 16);
        data_ret.z |= (pos_data[idx++] << 8);
        data_ret.z |= pos_data[idx++];
    }
    DBG_PRINT("3as Pos %+d %+d %+d\r\n", data_ret.x, data_ret.y, data_ret.z);
    if (seq && *seq) {
        char result_buf[48];
        snprintf(result_buf, sizeof(result_buf), "%d %d %d",data_ret.x, data_ret.y, data_ret.z);
        ack_with_info(seq, result_buf);
    }
    return ret;
}

/**
 * @brief   Move 3Axis
 * @param   cmd: MIIS_3X_MOVE_ABS, MIIS_3X_MOVE_REL
 * @param   data.axes: which motor will be inquired
 * @return  0(successful) or -1(failed)
 */
INT32S miis_3axis_move(INT8U cmd, miis_3axis_data_t data) {
    INT32S ret;
    INT8U  i2c_data[16] = {0};
    INT32U i2c_data_idx = 0;
    if (data.axes == 0 || data.axes > (MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS | MIIS_3X_Z_AXIS)) {
        DBG_PRINT("Invalid axes %#x\r\n", data.axes);
        return -1;
    }
    i2c_data[i2c_data_idx] = (cmd << 4) | data.axes;
    if (data.axes & MIIS_3X_X_AXIS) {
        i2c_data[++i2c_data_idx] = (data.x >> 24) & 0xFF;
        i2c_data[++i2c_data_idx] = (data.x >> 16) & 0xFF;
        i2c_data[++i2c_data_idx] = (data.x >> 8) & 0xFF;
        i2c_data[++i2c_data_idx] = (data.x) & 0xFF;
    }
    if (data.axes & MIIS_3X_Y_AXIS) {
        i2c_data[++i2c_data_idx] = (data.y >> 24) & 0xFF;
        i2c_data[++i2c_data_idx] = (data.y >> 16) & 0xFF;
        i2c_data[++i2c_data_idx] = (data.y >> 8) & 0xFF;
        i2c_data[++i2c_data_idx] = (data.y) & 0xFF;
    }
    if (data.axes & MIIS_3X_Z_AXIS) {
        i2c_data[++i2c_data_idx] = (data.z >> 24) & 0xFF;
        i2c_data[++i2c_data_idx] = (data.z >> 16) & 0xFF;
        i2c_data[++i2c_data_idx] = (data.z >> 8) & 0xFF;
        i2c_data[++i2c_data_idx] = (data.z) & 0xFF;
    }

    ret = miis_3axis_write_reg(i2c_data, i2c_data_idx + 1);
    miis_3axis_wait(data);

    if (ret < 0) {
        DBG_PRINT("3as move Error\r\n");
        uart2_printf("3as move Error\r\n");
    }
    return ret;
}

/**
 * @brief Waits for the completion of 3-axis operations by polling their busy status.
 *
 * This function periodically checks the status of the specified axes (X, Y, Z)
 * to determine if all requested axes have completed their operations.
 * The function checks every 100ms and returns once all axes are not busy,
 * or after a maximum of 100 attempts (~10 seconds).
 *
 * @param data A structure containing the axes to monitor and their corresponding status.
 *             The `axes` field should be a bitmask indicating which axes to check.
 *
 * @return 0 if all specified axes complete within the timeout period,
 *         -1 if the timeout is reached and at least one axis is still busy.
 */
INT32S miis_3axis_wait(miis_3axis_data_t data) {
    INT32U i;
    // wait for 1st wait_interval first.
    osDelay(100);

    for (i = 0; i < 100; i++) { // timeout 10000 ms = 10s
        miis_3axis_data_t data_back = miis_3axis_get_status(data);
        // this axis is not busy or this axis is not assigned
        if ((!MIIS_3X_IS_BUSY(data_back.x) || !(data.axes & MIIS_3X_X_AXIS)) &&
            (!MIIS_3X_IS_BUSY(data_back.y) || !(data.axes & MIIS_3X_Y_AXIS)) &&
            (!MIIS_3X_IS_BUSY(data_back.z) || !(data.axes & MIIS_3X_Z_AXIS)))
            return 0;

        osDelay(100);
    }
    DBG_PRINT("3as wait timeout\r\n");
    return -1;
}
/**
 * @brief   Reset the 3-axis controller hardware.
 *
 * @details
 * This function toggles the reset GPIO pin of the M261 controller
 * to perform a hardware reset.
 */
void miis_3axis_reset(void) {
    gpio_set_value(M261_RST,GPIO_LOW);
    osDelay(500);
    gpio_set_value(M261_RST,GPIO_HIGH);
}
/**
 * @brief   Reset the 3-axis controller hardware.
 *
 * @details
 * This function toggles the reset GPIO pin of the M261 controller
 * to perform a hardware reset.
 */
void miis_3axis_init(void) {
    DBG_PRINT("3-Axis init\r\n");
    TCA9535A.devNumber = I2C_1;
    TCA9535A.slaveAddr = TCA9535A_SLAVE_ADDR;
    TCA9535A.clkRate = TCA9535A_I2C_CLK_RATE;
    drv_l1_i2c_init(TCA9535A.devNumber);
}


static INT32S miis_3axis_set_dir_lim(miis_3axis_data_t data, miis_3axis_dir_lim_t dirlim)
{
    INT8U tx_data[2];
    INT8U data_byte = 0;
    INT8U value;
    if (data.axes == 0 || (data.axes & ~(MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS | MIIS_3X_Z_AXIS))) {
        DBG_PRINT("Invalid axes %#x\r\n", data.axes);
        return -1;
    }
    /* X axis -> bits [1:0] */
    if (data.axes & MIIS_3X_X_AXIS) {
        value = (((INT8U)dirlim.x_dir & 0x1) << 1) | ((INT8U)dirlim.x_lim & 0x1);
        data_byte |= (value << 0);
    }

    /* Y axis -> bits [3:2] */
    if (data.axes & MIIS_3X_Y_AXIS) {
        value = (((INT8U)dirlim.y_dir & 0x1) << 1) | ((INT8U)dirlim.y_lim & 0x1);
        data_byte |= (value << 2);
    }

    /* Z axis -> bits [5:4] */
    if (data.axes & MIIS_3X_Z_AXIS) {
        value = (((INT8U)dirlim.z_dir & 0x1) << 1) | ((INT8U)dirlim.z_lim & 0x1);
        data_byte |= (value << 4);
    }

    tx_data[0] = (MIIS_3X_SET_DIR_LIM << 4) | data.axes;
    tx_data[1] = data_byte;

    DBG_PRINT("setdirlim: cmd=0x%02X data=0x%02X\r\n", tx_data[0], tx_data[1]);

    return miis_3axis_write_reg(tx_data, 2);
}
static INT32S miis_3axis_set_dir_lim_default(void)
{
    miis_3axis_data_t data = {0};
    miis_3axis_dir_lim_t dirlim = {0};

    data.axes = MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS | MIIS_3X_Z_AXIS;

    /* - - + - - - */
    dirlim.x_dir = 1;
    dirlim.x_lim = 1;

    dirlim.y_dir = 0;
    dirlim.y_lim = 1;

    dirlim.z_dir = 1;
    dirlim.z_lim = 1;

    return miis_3axis_set_dir_lim(data, dirlim);
}
static INT32S miis_3axis_set_dir_lim_x(void)
{
    miis_3axis_data_t data = {0};
    miis_3axis_dir_lim_t dirlim = {0};

    data.axes = MIIS_3X_X_AXIS | MIIS_3X_Y_AXIS | MIIS_3X_Z_AXIS;

    /* - + + - - - */
    dirlim.x_dir = 1;
    dirlim.x_lim = 0;

    dirlim.y_dir = 0;
    dirlim.y_lim = 1;

    dirlim.z_dir = 1;
    dirlim.z_lim = 1;

    return miis_3axis_set_dir_lim(data, dirlim);
}

static INT32S miis_3axis_probe_lock(void)
{
    INT32S ret;
    miis_3axis_data_t data = {0};
    miis_3axis_data_t status;
    miis_3axis_data_t move_data = {0};
    /* inquire X axis only */
    data.axes = MIIS_3X_X_AXIS;
    status = miis_3axis_get_status(data);
    /* read X axis limit status */
    if (status.x < 0) {
        DBG_PRINT("Get X axis status error\r\n");
        return -1;
    }
    if (!MIIS_3X_AT_NEAR_LIM(status.x) && !MIIS_3X_AT_FAR_LIM(status.x)) {
        DBG_PRINT("X axis is not at any limit\r\n");
        return 0;
    }else {
        DBG_PRINT("X at near limit\r\n");
        move_data.axes = MIIS_3X_X_AXIS;
        move_data.x = 2500;
        move_data.y = 0;
        move_data.z = 0;
        miis_3axis_move(MIIS_3X_MOVE_REL, move_data);
        osDelay(100);
    }
    status = miis_3axis_get_status(data);
    if (!MIIS_3X_AT_NEAR_LIM(status.x) && !MIIS_3X_AT_FAR_LIM(status.x)) {
        DBG_PRINT("Probe has moved away from the home position\r\n");
        return -2;
    }else{
        miis_3axis_set_dir_lim_x();
        osDelay(100);
        move_data.axes = MIIS_3X_X_AXIS;
        move_data.x = -7200;
        move_data.y = 0;
        move_data.z = 0;
        miis_3axis_move(MIIS_3X_MOVE_REL, move_data);
        osDelay(100);
        miis_3axis_set_dir_lim_default();
        osDelay(100);
    }
    status = miis_3axis_get_status(data);
    if (!MIIS_3X_AT_NEAR_LIM(status.x) && !MIIS_3X_AT_FAR_LIM(status.x)){
        DBG_PRINT("Probe unlocked \r\n");
        return 0;
    }else{
        DBG_PRINT("Probe locked\r\n");
        return -3;
    }


}
/**
 * @brief   MIIS 3-axis task entry function.
 *
 * @details
 * This task creates the command queue, initializes the 3-axis controller,
 * and continuously receives motor control messages for processing.
 *
 * Supported operations include firmware version query, homing,
 * absolute and relative movement, busy status query, position query,
 * and zero-point setting.
 *
 * The task dispatches each command to the corresponding control function
 * and returns ACK/NAK responses through UART.
 */
void miis_3axis_task_runner(void) {
    miis_3axis_message_t msg;
    INT32S ret=0;
    queue_cmd_to_3axis_task = xQueueCreate(10, sizeof(miis_3axis_message_t));

    // initial TCA9535A
    miis_3axis_init();

    while (1) {
        /* block forever until a new command is received */
        if (queue_cmd_to_3axis_task && (xQueueReceive(queue_cmd_to_3axis_task, &msg, portMAX_DELAY) == pdTRUE)) {
            char char_seq[16];
            snprintf(char_seq, sizeof(char_seq), "%d", msg.index);
            switch (msg.cmd) {
            case MIIS_3X_FW_VERSION:
                miis_3axis_get_fw_version(char_seq);
                break;
            case MIIS_3X_HOME:
                ret = miis_3axis_move_home(msg.data);
                if(ret<0){nak_with_info(char_seq);}
                else{ack_with_info(char_seq,NULL);}
                break;
            case MIIS_3X_MOVE_ABS:
            case MIIS_3X_MOVE_REL:
                if (miis_3axis_move(msg.cmd, msg.data) > 0) {
                    DBG_PRINT("3as %+d %+d %+d\r\n", msg.data.x, msg.data.y, msg.data.z);
                    char result_buf[64];
                    snprintf(result_buf, sizeof(result_buf), "%+d %+d %+d",msg.data.x, msg.data.y, msg.data.z);
                    ack_with_info(char_seq, result_buf);
                } else {
                    DBG_PRINT("3as move Error\r\n");
                    nak_with_info(char_seq);
                }
                break;
            case SET_TORQUE:
                break;
            case MIIS_3X_STATUS: {
                miis_3axis_data_t status = miis_3axis_get_status(msg.data);
                if (MIIS_3X_IS_BUSY(status.x) || MIIS_3X_IS_BUSY(status.y) || MIIS_3X_IS_BUSY(status.z)) {
                    DBG_PRINT("busy\r\n");
                    //uart2_printf("1\r\n");
                    ack_with_info(char_seq,"1");
                } else {
                    DBG_PRINT("idle\r\n");
                    //uart2_printf("0\r\n");
                    ack_with_info(char_seq,"0");
                }
                break;
            }
            case MIIS_3X_POS:
                miis_3axis_get_pos(char_seq,msg.data);
                break;
            case MIIS_3X_SET_ZERO:
                ret = miis_3axis_set_zero(msg.data);
                if(ret<0){nak_with_info(char_seq);}
                else{ack_with_info(char_seq,NULL);}
                break;
            case MIIS_3X_SET_SPEED:
                break;
            case MIIS_3X_GET_SPEED:
                break;
            case MIIS_3X_SET_DIR_LIM:
                ret = miis_3axis_set_dir_lim(msg.data, msg.dirlim);
                if (ret < 0) {
                    nak_with_info(char_seq);
                } else {
                    ack_with_info(char_seq, NULL);
                }
                break;
            case MIIS_3X_SAVE_SPEED:
                break;
            case MIIS_3X_SAVE_DIR_LIM:
                break;
            case MIIS_3X_SET_NO_TRESPASS:
                break;
            case MIIS_3X_SAVE_NO_TRESPASS:
                break;
            case MIIS_3X_MOVE_CENTER:
                break;
            case MIIS_3X_RESET:
                break;
            case MIIS_3X_LOCKOFF:
                ret=miis_3axis_probe_lock();
                if (ret < 0) {
                    if (ret==-1)
                        nak_with_info(char_seq);
                    else if (ret==-2)
                        nak_with_detail_info(char_seq,"home");
                    else
                        nak_with_detail_info(char_seq,"locked");
                } else {
                    ack_with_info(char_seq, NULL);
                }
                break;

            default:
                break;
            }
        }
    }
}
