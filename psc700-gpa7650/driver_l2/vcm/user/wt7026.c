/** @file wt7026.c
 *  @brief Autofocus motor driver source file
 *
 *  This file contains functions to initialize the SPI_0 interface
 *  and to control the movement of the motor.
 */
#include "wt7026.h"
#include "project.h"
#include "drv_l1_gpio.h"
#include "drv_l1_spi.h"
#include "drv_l1_timer.h"
#include "drv_l2_actuator.h"

/*******************************************************************
 * Constants
 *******************************************************************/
#define MOTOR_DRIVER_TAG                        "WT7026"
#define STEP_DELAY_USEC                         450
// 400 -> 462us 1_000_000 / 462 = 2164 pps
// 450 -> 520us 1_000_000 / 520 = 1923 pps
// 500 -> 576us 1_000_000 / 576 = 1736 pps
//1000 -> 1150us 1_000_000 / 1150 = 869 pps

/*
 * |-------------------------------------------|
 * | BIT4  | BIT3  | BIT2  |   CH1   |   CH2   |
 * |-------+-------+-------+---------+----------
 * |   1   |   1   |   0   | Forward | Forward |
 * |   0   |   1   |   1   |   OFF   | Forward |
 * |   0   |   1   |   0   | Reverse | Forward |
 * |   0   |   0   |   1   | Reverse |   OFF   |
 * |   0   |   0   |   0   | Reverse | Reverse |
 * |   1   |   0   |   1   |   OFF   | Reverse |
 * |   1   |   0   |   0   | Forward | Reverse |
 * |   1   |   1   |   1   | Forward |   OFF   |
 * |-------+-------+-------+---------+---------|
 */
/*
 * 2-2 Phase Full-step
 */
static const INT8U step_sequence_2_2_phase[4] = {
    0x18, // 0001 1000 → Phase 0
    0x08, // 0000 1000 → Phase 2
    0x00, // 0000 0000 → Phase 4
    0x10, // 0001 0000 → Phase 6
};

/*******************************************************************
 * Private Variables
 *******************************************************************/
actuator_s  wt7026_act;

BOOLEAN af_pos_start_flag = FALSE;
INT32U last_af_move_tick = 0;

/*******************************************************************
 * Declare Functions
 *******************************************************************/
/**
  *@brief Initializes the SPI interface for the WT7026 actuator
  *@param None
  *@retval None
  */
static void wt7026_spi_init(void);
/**
  *@brief Sends one byte via native SPI API
  *@param data: Byte to send
  *@retval None
  */
static void wt7026_spi_write_1byte(INT8U data);
/**
  *@brief Packs a 3-bit register address and 5-bit data into one byte
  *@param addr3bit: 3-bit register address
  *@param data5bit: 5-bit data to write
  *@retval Packed byte
  */
static INT8U wt7026_pack_byte(INT8U addr3bit, INT8U data5bit);
/**
  *@brief Writes data to a WT7026 register using wt7026_spi_write_1byte()
  *@param addr: Register address
  *@param data: Data to write
  *@retval None
  */
static void wt7026_write_register(INT8U addr, INT8U data);
/**
  *@brief  Set the WT7026 driver to Active mode (4.8V).
  *@param None
  *@retval None
  */
static void wt7026_active(void);
/**
  *@brief  Set the WT7026 driver to Hold mode (1.8V).
  *@param None
  *@retval None
  */
static void wt7026_hold(void);
/**
  *@brief Moves the actuator one microstep forward (clockwise)
  *@param None
  *@retval None
  */
static void wt7026_forward(void);
/**
  *@brief Moves the actuator one microstep backward (counter-clockwise)
  *@param None
  *@retval None
  */
static void wt7026_backward(void);
/*******************************************************************
 * Private Functions
 *******************************************************************/
/**
  *@brief Initializes the SPI interface for the WT7026 actuator
  *@param None
  *@retval None
  */
static void wt7026_spi_init(void)
{
    drv_l1_spi_init(SPI_CH);
    drv_l1_spi_clk_set(SPI_CH, 5000000);
    drv_l1_spi_pha_pol_set(SPI_CH, PHA0_POL0);

    if(SPI_CH == SPI_0)
        drv_l1_spi_cs_by_gpio_set(SPI_CH, 1, IO_D6, 0);
    else
        drv_l1_spi_cs_by_gpio_set(SPI_CH, 1, IO_D10, 0);

    gpio_init_io(MOTOR_EN_PIN, GPIO_OUTPUT);
    gpio_io_top_prio(MOTOR_EN_PIN, 1);
    gpio_set_port_attribute(MOTOR_EN_PIN, 1);
    gpio_write_io(MOTOR_EN_PIN, 1);

    gpio_init_io(MOTOR_LIMIT_SWITCH_PIN, GPIO_INPUT);
    osDelay(100);
}
/**
  *@brief Sends one byte via native SPI API
  *@param data: Byte to send
  *@retval None
  */
static void wt7026_spi_write_1byte(INT8U data)
{
    drv_l1_spi_transceive_cpu(SPI_CH, &data, 1, NULL, 0);
}
/**
  *@brief Packs a 3-bit register address and 5-bit data into one byte
  *@param addr3bit: 3-bit register address
  *@param data5bit: 5-bit data to write
  *@retval Packed byte
  */
static INT8U wt7026_pack_byte(INT8U addr3bit, INT8U data5bit)
{
    return (INT8U)(((addr3bit & 0x07) << 5) | (data5bit & 0x1F));
}
/**
  *@brief Writes data to a WT7026 register using wt7026_spi_write_1byte()
  *@param addr: Register address
  *@param data: Data to write
  *@retval None
  */
static void wt7026_write_register(INT8U addr, INT8U data)
{
    INT8U send_byte = wt7026_pack_byte(addr, data);
    wt7026_spi_write_1byte(send_byte);
}
/**
  *@brief  Set the WT7026 driver to Active mode (4.8V).
  *@param  None
  *@retval None
  */
static void wt7026_active(void)
{
    // Set voltage to 4.8V
    wt7026_write_register(WT7026_REG_DAC_CH1_2, SET_DAC_VAL(WT7026_CH1_CH2_DAC_RAW_4V8));
}
/**
  *@brief  Set the WT7026 driver to Hold mode (1.8V).
  *@param  None
  *@retval None
  */
static void wt7026_hold(void)
{
    // Reduce voltage to 1.8V to lock motor position
    wt7026_write_register(WT7026_REG_DAC_CH1_2, SET_DAC_VAL(WT7026_CH1_CH2_DAC_RAW_1V8));
}
/*******************************************************************
 * Public Functions
 *******************************************************************/
/**
  *@brief Initializes the WT7026 actuator driver.
  *@param None
  *@retval None
  */
void wt7026_init(void)
{
    DBG_PRINT("[%s][LOG] Start initialization\r\n", MOTOR_DRIVER_TAG);
    wt7026_act.step_phase = 0;
    wt7026_spi_init();

    osDelay(10);
    wt7026_write_register(WT7026_PI_DRV_CTRL, WT7026_REG_PS_PI_RESET_INIT);
    osDelay(1);

    wt7026_write_register(WT7026_REG_DAC_CH1_2, SET_DAC_VAL(WT7026_CH1_CH2_DAC_RAW_1V8));
    osDelay(100);

    wt7026_write_register(WT7026_REG_DISABLE_CH1_2, WT7026_SYS_EN); // enable
    osDelay(100);

    wt7026_home();

    DBG_PRINT("[%s][LOG] initialization complete\r\n", MOTOR_DRIVER_TAG);
}

/**
  *@brief Uninitializes the WT7026 actuator driver.
  *@param None
  *@retval None
  */
void wt7026_uninit(void)
{
    gpio_write_io(MOTOR_EN_PIN, 0);
    drv_l1_spi_uninit(SPI_CH);
    DBG_PRINT("[%s][LOG] uninit\r\n", MOTOR_DRIVER_TAG);
}
/**
  *@brief Moves the actuator one microstep forward (clockwise)
  *@param None
  *@retval None
  */
static void wt7026_forward(void)
{
  for (INT8U i = 0; i < PHASE_COUNT_PER_STEP; i++) {
    wt7026_act.step_phase = (wt7026_act.step_phase - 1 + 4) & 0x03;
    wt7026_write_register(WT7026_REG_CH1_2_SERIAL_CTL, step_sequence_2_2_phase[wt7026_act.step_phase]);
    osDelay(2);
  }
}
/**
  *@brief Moves the actuator one microstep backward (counter-clockwise)
  *@param None
  *@retval None
  */
static void wt7026_backward(void)
{
  for (INT8U i = 0; i < PHASE_COUNT_PER_STEP; i++) {
    wt7026_act.step_phase = (wt7026_act.step_phase + 1) & 0x03;
    wt7026_write_register(WT7026_REG_CH1_2_SERIAL_CTL, step_sequence_2_2_phase[wt7026_act.step_phase]);
    osDelay(2);
  }
}

/**
  *@brief Moves the actuator to the home position by driving it until it touches the limit switch.
  *@param None
  *@retval The total number of steps moved during the homing process.
  */
INT32U wt7026_home(void)
{
    INT32U timeout_cnt = 0;
    INT32U home_step = 0;
    wt7026_active();

    while(gpio_read_io(MOTOR_LIMIT_SWITCH_PIN) && (timeout_cnt < TIMEOUT_STEPS)) {
      wt7026_backward();
      home_step++;
      timeout_cnt++;
    }

    wt7026_act.cur_pos = 0;

    wt7026_hold();
    return home_step;
}

/**
  *@brief Moves the actuator to a specified position
  *@param rel_pos: The relative position to move the actuator to
  *@retval Result: 0 for success, other values indicate failure.
  */
INT8U wt7026_move(INT32S rel_pos)
{
    if (rel_pos == 0)
      return 0;
    if ((rel_pos > 0) && (wt7026_act.cur_pos == MAX_STEPS))
      return 1;
    BOOLEAN dir = (rel_pos > 0);
    INT32S dest_pos = wt7026_act.cur_pos + rel_pos;
    if (dest_pos < 0)
        dest_pos = 0;
    else if (dest_pos > MAX_STEPS)
        dest_pos = MAX_STEPS;

    INT32U steps = abs(rel_pos);
    if (steps > MAX_STEPS)
      steps = TIMEOUT_STEPS;

    wt7026_active();

    while(steps > 0) {
      if (dir) {
        wt7026_forward();
      } else {
        if(gpio_read_io(MOTOR_LIMIT_SWITCH_PIN))
          wt7026_backward();
        else
          break;
      }
      steps--;
    }

    wt7026_act.cur_pos = dest_pos;

    wt7026_hold();

    return 0;
}

/**
  *@brief Moves the actuator to an absolute position by invoking wt7026_move()
  *@param abs_pos: The target absolute position to move the actuator to
  *@retval Result: 0 for success, other values indicate failure.
  */
INT8U wt7026_goto(INT32S abs_pos)
{
  if ((abs_pos > MAX_STEPS))
    return 1;

  INT32S steps = (INT32S)abs_pos- wt7026_act.cur_pos;
  wt7026_move(steps);
  return 0;
}

// Actuator Interface
const drv_l2_actuator_ops_t wt7026_act_ops =
{
    "wt7026_actuator",
    wt7026_init,
    wt7026_uninit,
    wt7026_move
};