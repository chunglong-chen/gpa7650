/**
 * @file    mems.c
 * @brief   MEMS scanning control module.
 * @details This module implements control of MEMS mirror scanning using
 *          DAC-driven voltage waveforms and timer-based state machine.
 *
 *          It provides functionality for:
 *          - MEMS enable/disable control
 *          - Line scan and waveform scan configuration
 *          - Smooth positioning using cosine waveform
 *          - Real-time waveform output via DAC
 *          - Camera trigger synchronization during scanning
 *
 *          The module uses a timer interrupt to drive the MEMS control
 *          state machine, handling movement, scanning execution, and
 *          transition between different scan states.
 * @ingroup spectrometer_mems_controller
 */
#include "mems.h"
#include "user.h"
#include <math.h>
#include <stdio.h>
#include "config/default/peripheral/tcc/plib_tcc1.h"
#include "config/default/peripheral/port/plib_port.h"
#include "spi.h"
#include "alg.h"
#include "motor_ctrl.h"
#include "waveform.h"
#include "lowpass_filter.h"

#define MEMS_WAIT_COUNT 8000 // 8000 * 12.5us = 100ms; wait 100ms for MEMS to stabilize
#define CAMERA_TRIGGER_LOW_COUNT 100 // 100 * 12.5us = 1.25ms; Disable camera 1.25ms after it is triggered high.

#define RADIAL_WAIT_COUNT 770 // Radial scan wait 1 period
#define GCC_WAIT_COUNT 770 // GCC scan wait 1 period
#define GCCHV_WAIT_COUNT 770 // GCCHV scan wait 1 period

WAVEFORM_DATA m_wave_data;
uint32_t trigger_data[MAX_LINESCAN_SIZE];
uint16_t x_data[DATA_ARRAY_SIZE];
uint16_t y_data[DATA_ARRAY_SIZE];
WAVEFORM_MOVE_DATA m_move_data;
uint16_t x_mv_data[COSINEWAVE_SIZE];
uint16_t y_mv_data[COSINEWAVE_SIZE];
WAVEFORM_RADIAL_DATA m_radial_data;
WAVEFORM_OFFSET_DATA m_offset_data; // Store voltage data with offset values
volatile MEMS_SCAN_STATUS mems_scan_status = MEMS_SCAN_NONE;
volatile MEMS_SCAN_TYPE scan_type = SCAN_TYPE_DOT;
MEMS_DATA m_mems_data;
uint32_t current_index = 0; // waveform index
bool before_first_trigger = false; // flag to check if the first trigger has occurred
bool is_camera_enable = false; // flag to check if the camera is currently enabled
uint32_t first_trigger_cnt = 0; // counter for first trigger
uint32_t period_cnt = 0; // counter for period when the first trigger has occurred
uint32_t disable_camera_cnt = 0; // counter for disabling the camera
uint32_t current_scan_count = 0;
uint32_t mems_wait_count = 0;

uint32_t mv_current_x_index = 0; // index of x cosine wave
uint32_t mv_current_y_index = 0; // index of y cosine wave
double mv_target_x;
double mv_target_y;
uint16_t trigger_index = 0; // trigger index
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
/**
 * @brief   MEMS control interrupt handler.
 * @details Executes MEMS control state machine driven by timer interrupt.
 *          Handles movement, waveform scanning, camera triggering, and
 *          transition between scan states.
 *
 * @param[in] status   Interrupt status.
 * @param[in] context  User context.
 */
/* This function is called after period expires */
void TCC1_Callback_InterruptHandler(uint32_t status, uintptr_t context) {
    _12p5Us_tick++;
    switch (mems_scan_status) {
        case MEMS_SCAN_SET_BIAS:
        {
            Toggle_LDAC();
            spi_write2dac(m_mems_data);
            mems_scan_status = MEMS_SCAN_STOP;
            break;
        }
        case MEMS_SCAN_MOVE:
        {
            Toggle_LDAC();

            if (m_move_data.x_length != 0 && mv_current_x_index < m_move_data.x_length) {
                mv_target_x = ((double) m_move_data.ptr_move_x[mv_current_x_index] / (double) UINT16_MAX * (m_move_data.mv_to_x_volt - m_move_data.last_x_volt) + m_move_data.last_x_volt);
                mv_current_x_index++;
            } else if (m_move_data.x_length == 0) {
                mv_target_x = m_move_data.last_x_volt;
            }
            if (m_move_data.y_length != 0 && mv_current_y_index < m_move_data.y_length) {
                mv_target_y = ((double) m_move_data.ptr_move_y[mv_current_y_index] / (double) UINT16_MAX * (m_move_data.mv_to_y_volt - m_move_data.last_y_volt) + m_move_data.last_y_volt);
                mv_current_y_index++;
            } else if (m_move_data.y_length == 0) {
                mv_target_y = m_move_data.last_y_volt;
            }

            update_mems_voltages(&m_mems_data, mv_target_x, mv_target_y);
            spi_write2dac(m_mems_data);

            if (mv_current_x_index == m_move_data.x_length && mv_current_y_index == m_move_data.y_length) {
                mv_current_x_index = 0;
                mv_current_y_index = 0;
                if (scan_type == SCAN_TYPE_DOT)
                    mems_scan_status = MEMS_SCAN_STOP;
                else {
                    mems_scan_status = MEMS_SCAN_WAIT_START;
                    // initialize the parameters
                    mems_wait_count = 0;
                    current_index = 0;
                    current_scan_count = 0;
                    before_first_trigger = true;
                    is_camera_enable = false;
                    period_cnt = 0;
                    first_trigger_cnt = 0;
                    disable_camera_cnt = 0;
                    trigger_index = 0;
                }

                m_move_data.last_x_volt = m_move_data.mv_to_x_volt;
                m_move_data.last_y_volt = m_move_data.mv_to_y_volt;
            }
            break;
        }
        case MEMS_SCAN_WAIT_START:
            if (mems_wait_count == 0) {
                Toggle_LDAC();
                // Pull the camera trigger pin low while waiting for the MEMS to stabilize. 
                // Avoid issuing any scan commands during a scan, as it may cause the camera to remain in a high state, 
                // resulting in the first camera trigger not being effective.
                CAM_TRIG_Clear();
            }

            mems_wait_count++;

            if (mems_wait_count == MEMS_WAIT_COUNT)
                mems_scan_status = MEMS_SCAN_RUN_WAVEFORM;
            break;
        case MEMS_SCAN_RUN_WAVEFORM:
        {
            Toggle_LDAC();

            double target_x = ((double) m_wave_data.ptr_x[current_index] / (double) UINT16_MAX * (m_offset_data.x_stop_volt - m_offset_data.x_start_volt)) + m_offset_data.x_start_volt;
            double target_y = ((double) m_wave_data.ptr_y[current_index] / (double) UINT16_MAX * (m_offset_data.y_stop_volt - m_offset_data.y_start_volt)) + m_offset_data.y_start_volt;

            update_mems_voltages(&m_mems_data, target_x, target_y);
            spi_write2dac(m_mems_data);

            current_index++; // move to the next point in the waveform

            // Use trigger_data if trigger_length exists; otherwise, use trigger_start and trigger_period
            if (m_wave_data.trigger_length != 0) {
                if (current_index == m_wave_data.ptr_trigger[trigger_index]) {
                    trigger_index++;
                    //Pull camera trigger pin high
                    CAM_TRIG_Set();
                    is_camera_enable = true;
                }
            } else {
                if (before_first_trigger) { // if the first trigger has not occurred, increment the first trigger counter
                    if (first_trigger_cnt == m_wave_data.trigger_start) { // if the first trigger counter equals trigger_start, trigger the spectrometer
                        //Pull camera trigger pin high
                        CAM_TRIG_Set();
                        is_camera_enable = true;
                        before_first_trigger = false;
                    }
                    first_trigger_cnt++;
                } else {
                    period_cnt++;
                    if (period_cnt == m_wave_data.trigger_period) { // if the period counter equals trigger_period, trigger the spectrometer
                        CAM_TRIG_Set();
                        is_camera_enable = true;
                        period_cnt = 0;
                    }
                }
            }
            if (is_camera_enable) { // if the camera is enable, increment the disable counter
                disable_camera_cnt++;
                if (disable_camera_cnt == CAMERA_TRIGGER_LOW_COUNT) {
                    //Pull camera trigger pin low
                    CAM_TRIG_Clear();
                    is_camera_enable = false;
                    disable_camera_cnt = 0;
                }
            }

            if (current_index == m_wave_data.length) {
                current_index = 0;
                trigger_index = 0;
                current_scan_count++;
            }

            if ((current_scan_count == m_wave_data.scancnt) && (m_wave_data.scancnt != 0)) {
                // update last volt
                m_move_data.last_x_volt = ((double) m_wave_data.ptr_x[m_wave_data.length - 1] / (double) UINT16_MAX * (m_offset_data.x_stop_volt - m_offset_data.x_start_volt)) + m_offset_data.x_start_volt;
                m_move_data.last_y_volt = ((double) m_wave_data.ptr_y[m_wave_data.length - 1] / (double) UINT16_MAX * (m_offset_data.y_stop_volt - m_offset_data.y_start_volt)) + m_offset_data.y_start_volt;
                mems_scan_status = MEMS_SCAN_STOP_TO_BIAS;
            }
            break;
        }
        case MEMS_SCAN_STOP_TO_BIAS:
        {
            Toggle_LDAC();
            // Pull the camera trigger pin low when the scan stops.
            // Avoid issuing the scanstop command during a scan, as it may cause the camera to remain in a high state, 
            // resulting in the first camera trigger not being effective.
            CAM_TRIG_Clear();
            m_move_data.mv_to_x_volt = 0;
            m_move_data.mv_to_y_volt = 0;

            int interval[2] = {0, 65535};

            // generate cosine waveform 
            cosine_wave(m_move_data.last_x_volt, m_move_data.mv_to_x_volt, interval, x_mv_data, &m_move_data.x_length);
            cosine_wave(m_move_data.last_y_volt, m_move_data.mv_to_y_volt, interval, y_mv_data, &m_move_data.y_length);

            mv_current_x_index = 0;
            mv_current_y_index = 0;
            mems_scan_status = MEMS_SCAN_MOVE;
            scan_type = SCAN_TYPE_DOT;
            break;
        }
        case MEMS_SCAN_STOP:
            Toggle_LDAC(); // Trigger the DAC to load the last data
            mems_scan_status = MEMS_SCAN_NONE;
            break;
        case MEMS_SCAN_NONE:
            break;
        default:
            break;

    }
}

/**
 * @brief Converts a given bias voltage (Vbias) to a corresponding DAC code.
 *
 * This function calculates the DAC code for a specified voltage, ensuring proper handling
 * of both positive and negative voltage values. The voltage is scaled based on the maximum 
 * DAC value (65535) and the maximum voltage (200V). The result is rounded to the nearest 
 * integer to improve accuracy.
 *
 * @param volt: The bias voltage (Vbias) to be converted, can be positive or negative.
 * @return The corresponding DAC code as an integer.
 */
int16_t mems_volt2code(double volt) {
    // Use the round() function to correctly round the result for both positive and negative values.
    return (int16_t) round((volt * DAC_MAX_CODE) / MEMS_MAX_VOLT);
}

/**
 * @brief Updates MEMS voltage channels (X+/X-, Y+/Y-) based on target voltages.
 * 
 * This function calculates and updates the X and Y voltage channels 
 * (X+, X-, Y+, Y-) using the provided target voltages.
 *
 * @param mems_data: Pointer to the MEMS_DATA structure that holds the bias voltage and channel voltages.
 * @param voltx: Target voltage for the X-axis.
 * @param volty: Target voltage for the Y-axis.
 * @return None
 */
void update_mems_voltages(MEMS_DATA *mems_data, double voltx, double volty) {
    // Check for NULL pointer to avoid segmentation fault
    if (mems_data == NULL) {
        return;
    }
    // Update X+ and X- voltages.
    mems_data->xp_volt = mems_data->vbias + (voltx / 2);
    mems_data->xn_volt = mems_data->vbias - (voltx / 2);

    // Update Y+ and Y- voltages.
    mems_data->yp_volt = mems_data->vbias + (volty / 2);
    mems_data->yn_volt = mems_data->vbias - (volty / 2);
}

/* Set the start and stop voltages for the x and y axes in the OFFSET_DATA structure */
void mems_set_offset_voltages(double begin_x, double end_x, double begin_y, double end_y, WAVEFORM_OFFSET_DATA* data) {
    data->x_start_volt = begin_x;
    data->x_stop_volt = end_x;
    data->y_start_volt = begin_y;
    data->y_stop_volt = end_y;
}

/**
 * @brief Set the bias voltage for the MEMS device.
 * @param bias: The bias voltage to be set (must be between 0 and MEMS_MAX_VOLT).
 * @return true if the bias is set successfully, false if the bias is out of range.
 */
bool mems_set_bias(int bias) {
    if (bias > MEMS_MAX_VOLT || bias < 0) {
        return false;
    }

    // Set the bias voltage
    m_mems_data.vbias = bias;

    // Set the maximum voltage difference allowed
    m_mems_data.vdifference_max = bias * 2;

    // Set all channel to bias voltage
    update_mems_voltages(&m_mems_data, 0, 0);

    m_move_data.last_x_volt = 0;
    m_move_data.last_y_volt = 0;
    m_move_data.mv_to_x_volt = 0;
    m_move_data.mv_to_y_volt = 0;

    mems_scan_status = MEMS_SCAN_SET_BIAS;
    return true;
}

/**
 * @brief Moves the MEMS mirror based on the provided voltage differences for X and Y axes.
 * @param voltx: Voltage difference for the X-axis.
 * @param volty: Voltage difference for the Y-axis.
 * @return Returns true if the movement command is valid and initiated, false otherwise.
 */
bool mems_move(int voltx, int volty) {

    if (abs(voltx) > m_mems_data.vdifference_max || abs(volty) > m_mems_data.vdifference_max) {
        printf("\r\nVdifference should not be more than %d\r\n", m_mems_data.vdifference_max);
        return false;
    }

    m_move_data.mv_to_x_volt = voltx;
    m_move_data.mv_to_y_volt = volty;

    int interval[2] = {0, 65535};

    // generate cosine wave
    cosine_wave(m_move_data.last_x_volt, m_move_data.mv_to_x_volt, interval, x_mv_data, &m_move_data.x_length);
    cosine_wave(m_move_data.last_y_volt, m_move_data.mv_to_y_volt, interval, y_mv_data, &m_move_data.y_length);

    scan_type = SCAN_TYPE_DOT;
    mems_scan_status = MEMS_SCAN_MOVE;
    return true;
}
/**
 * @brief   Enable or disable MEMS driver.
 * @details Controls the MEMS enable pin to power on or off the MEMS
 *          actuation circuitry.
 *
 * @param[in] enable  true to enable MEMS, false to disable.
 */
void mems_enable(bool enable) {
    if (enable)
        MEMS_EN_Set();
    else
        MEMS_EN_Clear();
}
/**
 * @brief   Stop MEMS scanning and return to bias position.
 * @details Updates the MEMS scan state to transition from current
 *          scanning operation back to bias (center) position.
 */
void mems_stop_scan() {
    mems_scan_status = MEMS_SCAN_STOP_TO_BIAS;
    scan_type = SCAN_TYPE_DOT;

}
/**
 * @brief   Start MEMS line scan.
 * @details Configures waveform parameters for line scanning, including
 *          start/end positions, scan count, and waveform generation.
 *          Generates triangle waveform for scanning and cosine waveform
 *          for smooth transition to the first scan position.
 *
 * @param[in] begin_x   Starting X voltage.
 * @param[in] end_x     Ending X voltage.
 * @param[in] begin_y   Starting Y voltage.
 * @param[in] end_y     Ending Y voltage.
 * @param[in] scancnt   Number of scan repetitions.
 *
 * @return true  If configuration is successful.
 * @return false If parameters are invalid or exceed limits.
 */
bool mems_start_line_scan(double begin_x, double end_x, double begin_y, double end_y, uint16_t scancnt) {

    if (abs(begin_x) > m_mems_data.vdifference_max || abs(end_x) > m_mems_data.vdifference_max
            || abs(begin_y) > m_mems_data.vdifference_max || abs(end_y) > m_mems_data.vdifference_max) {
        printf("\r\nVdifference should not be more than %d\r\n", m_mems_data.vdifference_max);
        return false;
    }
    m_wave_data.trigger_length = 0;
    m_wave_data.x_start_volt = begin_x;
    m_wave_data.x_stop_volt = end_x;
    m_wave_data.y_start_volt = begin_y;
    m_wave_data.y_stop_volt = end_y;
    mems_set_offset_voltages(begin_x, end_x, begin_y, end_y, &m_offset_data);
    int edges;
    if (scancnt == 0) {
        edges = 2;
        m_wave_data.scancnt = scancnt;
    } else {
        edges = scancnt;
        m_wave_data.scancnt = 1;
    }

    int ramp = 770;
    int interval[2] = {0, 65535};

    m_wave_data.length = edges* ramp;
    if (m_wave_data.length > DATA_ARRAY_SIZE)
        return false;
    m_wave_data.trigger_start = round((770 - 512) / 2) + 324; // Each waveform peak to the next spectrometer trigger is 1.6 ms, PWM 25K use (770-512); 9.6K use 453
    m_wave_data.trigger_period = CURRENT_LINESCAN_ARRAY_SIZE;
    triangle(edges, ramp, interval, x_data, 0);
    triangle(edges, ramp, interval, y_data, 0);

    // Calculate mv_to_x_volt and mv_to_y_volt : Move to the first index in the waveform and wait for MEMS to stabilize
    m_move_data.mv_to_x_volt = ((double) m_wave_data.ptr_x[0] / (double) UINT16_MAX * (m_offset_data.x_stop_volt - m_offset_data.x_start_volt)) + m_offset_data.x_start_volt;
    m_move_data.mv_to_y_volt = ((double) m_wave_data.ptr_y[0] / (double) UINT16_MAX * (m_offset_data.y_stop_volt - m_offset_data.y_start_volt)) + m_offset_data.y_start_volt;
    // Generate cosine wave
    cosine_wave(m_move_data.last_x_volt, m_move_data.mv_to_x_volt, interval, x_mv_data, &m_move_data.x_length);
    cosine_wave(m_move_data.last_y_volt, m_move_data.mv_to_y_volt, interval, y_mv_data, &m_move_data.y_length);
    scan_type = SCAN_TYPE_LINE;
    mv_current_x_index = 0;
    mv_current_y_index = 0;
    mems_scan_status = MEMS_SCAN_MOVE;
    return true;
}

bool mems_start_zz_scan(double begin_x, double end_x, double begin_y, double end_y, uint8_t aslinecnt, uint8_t scancnt, uint8_t fastaxis) {

    if (abs(begin_x) > m_mems_data.vdifference_max || abs(end_x) > m_mems_data.vdifference_max
            || abs(begin_y) > m_mems_data.vdifference_max || abs(end_y) > m_mems_data.vdifference_max) {
        printf("\r\nVdifference should not be more than %d\r\n", m_mems_data.vdifference_max);
        return false;
    }
    m_wave_data.trigger_length = 0;
    m_wave_data.x_start_volt = begin_x;
    m_wave_data.x_stop_volt = end_x;
    m_wave_data.y_start_volt = begin_y;
    m_wave_data.y_stop_volt = end_y;
    m_wave_data.scancnt = scancnt;
    mems_set_offset_voltages(begin_x, end_x, begin_y, end_y, &m_offset_data);
    int edges = aslinecnt;
    int ramp = 770;
    int interval[2] = {0, 65535};

    m_wave_data.length = edges* ramp;
    if (m_wave_data.length > DATA_ARRAY_SIZE)
        return false;
    m_wave_data.trigger_start = round((770 - 512) / 2) + 324; // PWM 25K use (770-512); 9.6K use 453
    m_wave_data.trigger_period = CURRENT_LINESCAN_ARRAY_SIZE;
    // Fastaxis use triangle wave; slowaxis use stair wave
    if (fastaxis == AXIS_X) {
        triangle(edges, ramp, interval, x_data, 0);
        staircase(edges, ramp, interval, y_data);
    } else {
        triangle(edges, ramp, interval, y_data, 0);
        staircase(edges, ramp, interval, x_data);
    }

    // Calculate mv_to_x_volt and mv_to_y_volt : Move to the first index in the waveform and wait for MEMS to stabilize
    m_move_data.mv_to_x_volt = ((double) m_wave_data.ptr_x[0] / (double) UINT16_MAX * (m_offset_data.x_stop_volt - m_offset_data.x_start_volt)) + m_offset_data.x_start_volt;
    m_move_data.mv_to_y_volt = ((double) m_wave_data.ptr_y[0] / (double) UINT16_MAX * (m_offset_data.y_stop_volt - m_offset_data.y_start_volt)) + m_offset_data.y_start_volt;
    // Generate cosine wave
    cosine_wave(m_move_data.last_x_volt, m_move_data.mv_to_x_volt, interval, x_mv_data, &m_move_data.x_length);
    cosine_wave(m_move_data.last_y_volt, m_move_data.mv_to_y_volt, interval, y_mv_data, &m_move_data.y_length);

    scan_type = SCAN_TYPE_ZIGZAG;
    mv_current_x_index = 0;
    mv_current_y_index = 0;
    mems_scan_status = MEMS_SCAN_MOVE;
    return true;
}

bool mems_start_radial_scan(double offsetx, double offsety, double diameter, uint8_t linecnt, uint8_t lineavgcnt, uint8_t scancnt) {

    if (abs(offsetx) > m_mems_data.vdifference_max || abs(offsety) > m_mems_data.vdifference_max
            || abs(diameter) > m_mems_data.vdifference_max) {
        printf("\r\nVdifference should not be more than %d\r\n", m_mems_data.vdifference_max);
        return false;
    }

    m_wave_data.scancnt = scancnt;
    m_wave_data.x_start_volt = offsetx - diameter / 2;
    m_wave_data.x_stop_volt = offsetx + diameter / 2;
    m_wave_data.y_start_volt = offsety - diameter / 2;
    m_wave_data.y_stop_volt = offsety + diameter / 2;
    mems_set_offset_voltages((offsetx - diameter / 2), (offsetx + diameter / 2), (offsety - diameter / 2), (offsety + diameter / 2), &m_offset_data);
    int edges;
    if (lineavgcnt & 1)
        edges = lineavgcnt + 1;
    else
        edges = lineavgcnt;

    int ramp = 770;
    int interval[2] = {0, 65535};

    m_wave_data.length = linecnt * edges * ramp;
    if (m_wave_data.length > DATA_ARRAY_SIZE)
        return false;
    m_wave_data.trigger_start = round((770 - 512) / 2) + 324; // PWM 25K use (770-512); 9.6K use 453
    m_wave_data.trigger_period = CURRENT_LINESCAN_ARRAY_SIZE;
    // Generate trigger point array
    trigger_array(lineavgcnt, linecnt, m_wave_data.trigger_start, m_wave_data.trigger_period, trigger_data, &m_wave_data.trigger_length);

    // Generate x & y wave of Radial scan
    radial_x_wave(edges, ramp, linecnt, interval, x_data);
    radial_y_wave(edges, ramp, linecnt, interval, y_data);

    // Calculate mv_to_x_volt and mv_to_y_volt : Move to the first index in the waveform and wait for MEMS to stabilize
    m_move_data.mv_to_x_volt = ((double) m_wave_data.ptr_x[0] / (double) UINT16_MAX * (m_offset_data.x_stop_volt - m_offset_data.x_start_volt)) + m_offset_data.x_start_volt;
    m_move_data.mv_to_y_volt = ((double) m_wave_data.ptr_y[0] / (double) UINT16_MAX * (m_offset_data.y_stop_volt - m_offset_data.y_start_volt)) + m_offset_data.y_start_volt;
    // Generate cosine wave
    cosine_wave(m_move_data.last_x_volt, m_move_data.mv_to_x_volt, interval, x_mv_data, &m_move_data.x_length);
    cosine_wave(m_move_data.last_y_volt, m_move_data.mv_to_y_volt, interval, y_mv_data, &m_move_data.y_length);

    scan_type = SCAN_TYPE_RADIAL;
    mv_current_x_index = 0;
    mv_current_y_index = 0;
    mems_scan_status = MEMS_SCAN_MOVE;
    return true;
}

/* linecnt is number of vertical line , the total line will be (linecnt+1) */
bool mems_start_gcc_scan(double begin_x, double end_x, double begin_y, double end_y, double hori_begin_x, double hori_end_x, uint8_t lineavgcnt, uint8_t linecnt, uint8_t scancnt) {

    if (abs(begin_x) > m_mems_data.vdifference_max || abs(end_x) > m_mems_data.vdifference_max
            || abs(begin_y) > m_mems_data.vdifference_max || abs(end_y) > m_mems_data.vdifference_max) {
        printf("\r\nVdifference should not be more than %d\r\n", m_mems_data.vdifference_max);
        return false;
    }

    m_wave_data.scancnt = scancnt;
    m_wave_data.x_start_volt = begin_x;
    m_wave_data.x_stop_volt = end_x;
    m_wave_data.y_start_volt = begin_y;
    m_wave_data.y_stop_volt = end_y;
    mems_set_offset_voltages(begin_x, end_x, begin_y, end_y, &m_offset_data);
    int edges;
    if (lineavgcnt & 1)
        edges = lineavgcnt + 1;
    else
        edges = lineavgcnt;
    int ramp = 770;
    int interval[2] = {0, 65535};

    m_wave_data.length = edges * (linecnt + 1) * ramp;
    if (m_wave_data.length > DATA_ARRAY_SIZE)
        return false;
    m_wave_data.trigger_start = round((770 - 512) / 2) + 324; // PWM 25K use (770-512); 9.6K use 453
    m_wave_data.trigger_period = CURRENT_LINESCAN_ARRAY_SIZE;
    // Generate trigger point array
    trigger_array(lineavgcnt, (linecnt + 1), m_wave_data.trigger_start, m_wave_data.trigger_period, trigger_data, &m_wave_data.trigger_length);

    // Generate x & y wave of GCCL scan
    gcc_x_wave(edges, ramp, linecnt, interval, x_data);
    gcc_y_wave(edges, ramp, linecnt, interval, y_data);

    // Calculate mv_to_x_volt and mv_to_y_volt : Move to the first index in the waveform and wait for MEMS to stabilize
    m_move_data.mv_to_x_volt = ((double) m_wave_data.ptr_x[0] / (double) UINT16_MAX * (m_offset_data.x_stop_volt - m_offset_data.x_start_volt)) + m_offset_data.x_start_volt;
    m_move_data.mv_to_y_volt = ((double) m_wave_data.ptr_y[0] / (double) UINT16_MAX * (m_offset_data.y_stop_volt - m_offset_data.y_start_volt)) + m_offset_data.y_start_volt;
    // Generate cosine wave
    cosine_wave(m_move_data.last_x_volt, m_move_data.mv_to_x_volt, interval, x_mv_data, &m_move_data.x_length);
    cosine_wave(m_move_data.last_y_volt, m_move_data.mv_to_y_volt, interval, y_mv_data, &m_move_data.y_length);
    scan_type = SCAN_TYPE_GCC;
    mv_current_x_index = 0;
    mv_current_y_index = 0;
    mems_scan_status = MEMS_SCAN_MOVE;
    return true;
}

bool mems_start_gcchv_scan(double begin_x, double end_x, double begin_y, double end_y, uint8_t lineavgcnt, uint8_t linecnt, uint8_t scancnt, uint8_t fastaxis) {
    if (abs(begin_x) > m_mems_data.vdifference_max || abs(end_x) > m_mems_data.vdifference_max
            || abs(begin_y) > m_mems_data.vdifference_max || abs(end_y) > m_mems_data.vdifference_max) {
        printf("\r\nVdifference should not be more than %d\r\n", m_mems_data.vdifference_max);
        return false;
    }

    m_wave_data.scancnt = scancnt;
    m_wave_data.x_start_volt = begin_x;
    m_wave_data.x_stop_volt = end_x;
    m_wave_data.y_start_volt = begin_y;
    m_wave_data.y_stop_volt = end_y;
    mems_set_offset_voltages(begin_x, end_x, begin_y, end_y, &m_offset_data);
    int edges;
    if (lineavgcnt & 1)
        edges = lineavgcnt + 1;
    else
        edges = lineavgcnt;

    int ramp = 770;
    int interval[2] = {0, 65535};

    m_wave_data.length = edges * linecnt * ramp;
    if (m_wave_data.length > DATA_ARRAY_SIZE)
        return false;
    m_wave_data.trigger_start = round((770 - 512) / 2) + 324; // PWM 25K use (770-512); 9.6K use 453
    m_wave_data.trigger_period = CURRENT_LINESCAN_ARRAY_SIZE;
    // Generate trigger point array
    trigger_array(lineavgcnt, linecnt, m_wave_data.trigger_start, m_wave_data.trigger_period, trigger_data, &m_wave_data.trigger_length);
    // Fastaxis use triangle wave; slowaxis use stair wave
    if (fastaxis == AXIS_X) {
        triangle(edges*linecnt, ramp, interval, x_data, 0);
        staircase(linecnt, ramp*edges, interval, y_data);
    } else {
        staircase(linecnt, ramp*edges, interval, x_data);
        triangle(edges*linecnt, ramp, interval, y_data, 0);
    }

    // Calculate mv_to_x_volt and mv_to_y_volt : Move to the first index in the waveform and wait for MEMS to stabilize
    m_move_data.mv_to_x_volt = ((double) m_wave_data.ptr_x[0] / (double) UINT16_MAX * (m_offset_data.x_stop_volt - m_offset_data.x_start_volt)) + m_offset_data.x_start_volt;
    m_move_data.mv_to_y_volt = ((double) m_wave_data.ptr_y[0] / (double) UINT16_MAX * (m_offset_data.y_stop_volt - m_offset_data.y_start_volt)) + m_offset_data.y_start_volt;
    // Generate cosine wave
    cosine_wave(m_move_data.last_x_volt, m_move_data.mv_to_x_volt, interval, x_mv_data, &m_move_data.x_length);
    cosine_wave(m_move_data.last_y_volt, m_move_data.mv_to_y_volt, interval, y_mv_data, &m_move_data.y_length);
    scan_type = SCAN_TYPE_GCCHV;
    mv_current_x_index = 0;
    mv_current_y_index = 0;
    mems_scan_status = MEMS_SCAN_MOVE;
    return true;
}
/**
 * @brief   Execute MEMS waveform scan with offset.
 * @details Applies offset to pre-generated waveform and moves MEMS
 *          to the starting position using a smooth cosine transition.
 *          Then prepares the system to execute waveform scanning.
 *
 * @param[in] x_coord  X-axis offset.
 * @param[in] y_coord  Y-axis offset.
 *
 * @return true  If waveform execution is ready.
 * @return false If waveform data is not available.
 */
/* As long as length > 0, run the waveform */
bool mems_run_waveform(int x_coord, int y_coord) {

    if (m_wave_data.length == 0) {
        printf("\r\n No waveform data.\r\n");
        return false;
    }

    int interval[2] = {0, 65535};

    mems_set_offset_voltages((m_wave_data.x_start_volt + x_coord), (m_wave_data.x_stop_volt + x_coord), (m_wave_data.y_start_volt + y_coord), (m_wave_data.y_stop_volt + y_coord), &m_offset_data);
    // Calculate mv_to_x_volt and mv_to_y_volt : Move to the first index in the waveform and wait for MEMS to stabilize
    m_move_data.mv_to_x_volt = ((double) m_wave_data.ptr_x[0] / (double) UINT16_MAX * (m_offset_data.x_stop_volt - m_offset_data.x_start_volt)) + m_offset_data.x_start_volt;
    m_move_data.mv_to_y_volt = ((double) m_wave_data.ptr_y[0] / (double) UINT16_MAX * (m_offset_data.y_stop_volt - m_offset_data.y_start_volt)) + m_offset_data.y_start_volt;

    // Generate cosine wave
    cosine_wave(m_move_data.last_x_volt, m_move_data.mv_to_x_volt, interval, x_mv_data, &m_move_data.x_length);
    cosine_wave(m_move_data.last_y_volt, m_move_data.mv_to_y_volt, interval, y_mv_data, &m_move_data.y_length);

    mv_current_x_index = 0;
    mv_current_y_index = 0;
    scan_type = SCAN_TYPE_WAVEFORM;
    mems_scan_status = MEMS_SCAN_MOVE;
    return true;
}

WAVEFORM_DATA_TARGET get_waveform_target(const char* cmd) {
    for (int i = 0; waveform_cmd_list[i].name != NULL; i++) {
        if (strcmp(waveform_cmd_list[i].name, cmd) == 0) {
            return waveform_cmd_list[i].target;
        }
    }
    return WAVEFORM_TARGET_INVALID;
}

void show_valid_waveform_command(void) {
    printf("Valid commands are: ");
    for (int i = 0; waveform_cmd_list[i].name != NULL; i++) {
        if (i > 0) {
            printf(", ");
        }
        printf("%s", waveform_cmd_list[i].name);
    }
    printf("\r\n");
}

void show_waveform_data(void) {
    printf("(x_start, x_stop) (%d, %d)\r\n(y_start, y_stop) (%d, %d) \r\n", m_wave_data.x_start_volt, m_wave_data.x_stop_volt, m_wave_data.y_start_volt, m_wave_data.y_stop_volt);
    printf("length:%d, scancnt:%d, trigger_cnt:%d, trigger_period:%d, trigger_length:%d\r\n",
            m_wave_data.length, m_wave_data.scancnt, m_wave_data.trigger_start, m_wave_data.trigger_period, m_wave_data.trigger_length);
    printf("freq:%d\r\n", m_wave_data.lowpass_filter_freq);
    if (m_wave_data.length == 0)
        return;
    int start = 0;
    int middle = (m_wave_data.length / 2) - 5;
    int end = m_wave_data.length - 10;
    for (int i = start; i < (start + 10); i++) {
        printf("x[%d] %d (%f), y[%d] %d (%f)\r\n",
                i, m_wave_data.ptr_x[i], (double) m_wave_data.ptr_x[i] / UINT16_MAX, i, m_wave_data.ptr_y[i], (double) m_wave_data.ptr_y[i] / UINT16_MAX);
    }
    for (int i = middle; i < (middle + 10); i++) {
        printf("x[%d] %d (%f), y[%d] %d (%f)\r\n",
                i, m_wave_data.ptr_x[i], (double) m_wave_data.ptr_x[i] / UINT16_MAX, i, m_wave_data.ptr_y[i], (double) m_wave_data.ptr_y[i] / UINT16_MAX);
    }

    for (int i = end; i < (end + 10); i++) {
        printf("x[%d] %d (%f), y[%d] %d (%f)\r\n",
                i, m_wave_data.ptr_x[i], (double) m_wave_data.ptr_x[i] / UINT16_MAX, i, m_wave_data.ptr_y[i], (double) m_wave_data.ptr_y[i] / UINT16_MAX);
    }
    for (int i = 0; i < m_wave_data.trigger_length; i++) {
        printf("trigger[%d] %ld\r\n", i, m_wave_data.ptr_trigger[i]);
    }
}

/**
 * @brief   Write data to the specified target.
 * @param  target: Specifies which target will be written to.
 * @param  data_idx: Optional. Index when the target is x or y.
 * @param  value: Value to be written. 
 * @return  none
 */
void write_waveform_data(WAVEFORM_DATA_TARGET target, int data_idx, int value) {
    switch (target) {
        case WAVEFORM_TARGET_LENGTH:
            m_wave_data.length = value;
            break;
        case WAVEFORM_TARGET_SCANCNT:
            m_wave_data.scancnt = value;
            break;
        case WAVEFORM_TARGET_TRIGGER_CNT:
            m_wave_data.trigger_start = value;
            break;
        case WAVEFORM_TARGET_TRIGGER_PERIOD:
            m_wave_data.trigger_period = value;
            break;
        case WAVEFORM_TARGET_FILTER_FREQ:
            m_wave_data.lowpass_filter_freq = value;
            break;
        case WAVEFORM_TARGET_RESET:
            m_wave_data.length = value;
            m_wave_data.scancnt = value;
            m_wave_data.trigger_start = value;
            m_wave_data.trigger_period = value;
            break;
        case WAVEFORM_TARGET_X_DATA:
            if (data_idx >= 0 && data_idx < m_wave_data.length)
                m_wave_data.ptr_x[data_idx] = value;
            break;
        case WAVEFORM_TARGET_Y_DATA:
            if (data_idx >= 0 && data_idx < m_wave_data.length)
                m_wave_data.ptr_y[data_idx] = value;
            break;
        default:
            break;
    }
}

/* Show cuurent status for debug */
void show_information(void) {
    printf("***** Scan Info *****\r\n");
    printf("** scan status %d \r\n", mems_scan_status);
    printf("** scan type %d \r\n", scan_type);
    printf("** scan size %d \r\n", m_wave_data.length);
    printf("** scan idx %ld \r\n", current_index);
    printf("** trig size %d\r\n", m_wave_data.trigger_length);
    printf("** trig idx %d\r\n", trigger_index);
    printf("** mv x len %d, idx %ld \r\n", m_move_data.x_length, mv_current_x_index);
    printf("** mv y len %d, idx %ld \r\n", m_move_data.y_length, mv_current_y_index);
    printf("** mv last (%f, %f) \r\n", m_move_data.last_x_volt, m_move_data.last_y_volt);
    printf("** mv to (%f, %f) \r\n", m_move_data.mv_to_x_volt, m_move_data.mv_to_y_volt);
    printf("***** End *****\r\n");
}
/**
 * @brief   Initialize MEMS control module.
 * @details Registers timer interrupt callback for MEMS control,
 *          initializes hardware control pins, and resets all
 *          waveform, MEMS, and movement-related data structures.
 *
 *          This function prepares the MEMS system for scanning
 *          operations by:
 *          - Configuring TCC1 timer callback
 *          - Resetting trigger and control pins
 *          - Initializing waveform parameters and buffers
 *          - Resetting MEMS voltage and movement state
 *          - Starting the timer for real-time control
 */
void MEMS_Initialize(void) {
    /* Register callback function for TCC1 period interrupt */
    TCC1_TimerCallbackRegister(TCC1_Callback_InterruptHandler, (uintptr_t) NULL);

    CAM_TRIG_Clear();
    MEMS_EN_Clear();
    MEMS_LDAC_Clear();

    m_wave_data.length = 0;
    m_wave_data.scancnt = 0;
    m_wave_data.trigger_start = 0;
    m_wave_data.trigger_period = 0;
    m_wave_data.trigger_length = 0;
    m_wave_data.lowpass_filter_freq = lp_filter_get_freq();
    m_wave_data.ptr_trigger = &trigger_data[0];
    m_wave_data.ptr_x = &x_data[0];
    m_wave_data.ptr_y = &y_data[0];

    m_mems_data.vbias = 0;
    m_mems_data.vdifference_max = 0;

    m_move_data.x_length = 0;
    m_move_data.y_length = 0;
    m_move_data.last_x_volt = 0.0;
    m_move_data.last_y_volt = 0.0;
    m_move_data.mv_to_x_volt = 0.0;
    m_move_data.mv_to_y_volt = 0.0;
    m_move_data.ptr_move_x = &x_mv_data[0];
    m_move_data.ptr_move_y = &y_mv_data[0];
    /* Start the timer channel 1*/
    TCC1_TimerStart();
}

/******************************************************************************
  Function:
    void MEMS_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void MEMS_Tasks(void) {

}


/*******************************************************************************
 End of File
 */
