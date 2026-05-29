/**
 * @file    mems.h
 * @brief   MEMS scanning control interface definitions.
 * @details This file defines data structures, configuration parameters,
 *          and public APIs for MEMS mirror control.
 *
 *          It provides interfaces for:
 *          - Enabling and disabling MEMS operation
 *          - Configuring line scan and waveform scan parameters
 *          - Controlling scan execution and stop behavior
 *          - Managing scan states and waveform data
 * @ingroup spectrometer_mems_controller
 */
#ifndef TIMER_MEMS_H
#define	TIMER_MEMS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "configuration.h"
#include "system/time/sys_time.h"
#include "alg.h"

#ifdef	__cplusplus
extern "C" {
#endif

    /* According to the spec, a 90V bias is used as the baseline to level the MEMS mirror.
     * The X and Y axes voltages are controlled by the differential voltages across four 16-bit DAC channels (X+, X-, Y+, Y-). 
     * Initially, all channels are set to 90V (DAC code 29491) to level the mirror.
     * 
     * Example:
     * To set X to 120V and Y to -120V, adjust the channels as follows: X+ = 150V, X- = 30V, Y+ = 30V, Y- = 150V.
     */

#define MEMS_DEFAULT_VBIAS  90 // Default bias voltage
    /* The MEMS driver has a range from 0V-200V, corresponding to the DAC values of 0-65535 */
#define MEMS_MAX_VOLT       200 // MEMS driver voltage range is 0-200V
#define DAC_MAX_CODE        65535 // DAC8564 is 16-bit

    /* The sampling duration of 360 ms is determined because moving 1V takes 1 ms, and the range from -180V to 180V spans 360V.
     * The sampling rate is 80 kHz, and for a 360 ms duration, the calculated length is 28,800. 
     * Additional space is allocated to ensure sufficient buffer capacity and prevent overflow during processing. 
     * Therefore, the final length used is 30,000.*/
#define COSINEWAVE_SIZE 30000 // 360ms*80kHz = 28800 => 30000

    typedef enum {
        AXIS_X = 1,
        AXIS_Y = 2
    } MEMS_AXIS;

    typedef struct {
        uint8_t vbias; // Bias voltage applied to the MEMS device.
        uint8_t vdifference_max; // Maximum allowed voltage difference. This value should not exceed 2x vbias.
        double xp_volt; // voltage of X+ channel
        double xn_volt; // voltage of X- channel
        double yp_volt; // voltage of Y+ channel
        double yn_volt; // voltage of Y- channel
    } MEMS_DATA;

    typedef enum {
        SCAN_TYPE_DOT = 0,
        SCAN_TYPE_LINE,
        SCAN_TYPE_CROSS,
        SCAN_TYPE_ZIGZAG,
        SCAN_TYPE_RADIAL,
        SCAN_TYPE_CIRCLE,
        SCAN_TYPE_GCC,
        SCAN_TYPE_GCCHV,
        SCAN_TYPE_WAVEFORM
    } MEMS_SCAN_TYPE;

    typedef enum {
        MEMS_SCAN_NONE = 0,
        MEMS_SCAN_SET_BIAS,
        MEMS_SCAN_MOVE,
        MEMS_SCAN_MOVE_TO_START,
        MEMS_SCAN_WAIT_START,
        MEMS_SCAN_RUN_WAVEFORM,
        MEMS_SCAN_STOP_TO_BIAS,
        MEMS_SCAN_STOP
    } MEMS_SCAN_STATUS;

    typedef enum {
        WAVEFORM_TARGET_LENGTH = 0,
        WAVEFORM_TARGET_SCANCNT,
        WAVEFORM_TARGET_TRIGGER_CNT,
        WAVEFORM_TARGET_TRIGGER_PERIOD,
        WAVEFORM_TARGET_FILTER_FREQ,
        WAVEFORM_TARGET_RESET,
        WAVEFORM_TARGET_X_DATA,
        WAVEFORM_TARGET_Y_DATA,
        WAVEFORM_TARGET_INVALID
    } WAVEFORM_DATA_TARGET;

    typedef struct __attribute__((aligned(4))) {
        int length;
        int scancnt;
        int trigger_length;
        int trigger_start;
        int trigger_period;
        int x_start_volt; //x start volt
        int x_stop_volt; //x stop volt
        int y_start_volt; //y start volt
        int y_stop_volt; //y stop volt
        int lowpass_filter_freq; //low-pass filter frequency (Hz)
        uint32_t * ptr_trigger;
        // Keep ptr_x and ptr_y as the last entries.
        uint16_t * ptr_x;
        uint16_t * ptr_y;
    } WAVEFORM_DATA;

    typedef struct __attribute__((aligned(8))) {
        int x_length; // Length of X cosine wave
        int y_length; // Length of Y cosine wave
        double last_x_volt;
        double mv_to_x_volt;
        double last_y_volt;
        double mv_to_y_volt;
        uint16_t * ptr_move_x; // Pointer to X cosine wave
        uint16_t * ptr_move_y; // Pointer to Y cosine wave
    } WAVEFORM_MOVE_DATA;

    typedef struct {
        double center_x_volt;
        double center_y_volt;
        double radius_x_volt;
        double radius_y_volt;
        uint8_t radial_line_cnt; // Number of radial lines
        uint8_t radial_avg_line_cnt; // Number of averages per radial line
        float cosine[12]; // Cosine values for each radial line
        float sine[12]; // Sine values for each radial line
    } WAVEFORM_RADIAL_DATA;

    typedef struct {
        int x_start_volt; // Offset for the x-axis start voltage
        int x_stop_volt; // Offset for the x-axis stop voltage
        int y_start_volt; // Offset for the y-axis start voltage
        int y_stop_volt; // Offset for the y-axis stop voltage
    } WAVEFORM_OFFSET_DATA;

    typedef struct {
        const char* name;
        WAVEFORM_DATA_TARGET target;
    } WAVEFORM_CMD;

    static const WAVEFORM_CMD waveform_cmd_list[] = {
        {"len", WAVEFORM_TARGET_LENGTH},
        {"cnt", WAVEFORM_TARGET_SCANCNT},
        {"start", WAVEFORM_TARGET_TRIGGER_CNT},
        {"period", WAVEFORM_TARGET_TRIGGER_PERIOD},
        {"rst", WAVEFORM_TARGET_RESET},
        // Keep x, y, and NULL as the last entries.
        {"x", WAVEFORM_TARGET_X_DATA},
        {"y", WAVEFORM_TARGET_Y_DATA},
        {NULL, WAVEFORM_TARGET_INVALID} // End marker
    };

    extern WAVEFORM_DATA m_wave_data;
    extern uint32_t trigger_data[MAX_LINESCAN_SIZE];
    extern uint16_t x_data[DATA_ARRAY_SIZE];
    extern uint16_t y_data[DATA_ARRAY_SIZE];
    extern WAVEFORM_MOVE_DATA m_move_data;
    extern uint16_t x_mv_data[COSINEWAVE_SIZE];
    extern uint16_t y_mv_data[COSINEWAVE_SIZE];

    void MEMS_Initialize(void);

    void MEMS_Tasks(void);
    void mems_stop_scan(void);
    bool mems_start_line_scan(double begin_x, double end_x, double begin_y, double end_y, uint16_t scancnt);
    bool mems_start_zz_scan(double begin_x, double end_x, double begin_y, double end_y, uint8_t aslinecnt, uint8_t scancnt, uint8_t fastaxis);
    bool mems_start_radial_scan(double offsetx, double offsety, double diameter, uint8_t linecnt, uint8_t lineavgcnt, uint8_t scancnt);
    bool mems_start_gcc_scan(double begin_x, double end_x, double begin_y, double end_y, double hori_begin_x, double hori_end_x, uint8_t lineavgcnt, uint8_t linecnt, uint8_t scancnt);
    bool mems_start_gcchv_scan(double begin_x, double end_x, double begin_y, double end_y, uint8_t lineavgcnt, uint8_t linecnt, uint8_t scancnt, uint8_t fastaxis);
    bool mems_set_bias(int bias);
    bool mems_move(int voltx, int volty);
    void mems_enable(bool enable);
    int16_t mems_volt2code(double volt);
    void update_mems_voltages(MEMS_DATA *mems_data, double voltx, double volty);
    bool mems_run_waveform(int x_coord, int y_coord);

    WAVEFORM_DATA_TARGET get_waveform_target(const char* cmd);
    void show_valid_waveform_command(void);
    void show_waveform_data(void);
    void write_waveform_data(WAVEFORM_DATA_TARGET target, int data_idx, int value);
    void show_information(void);

#ifdef	__cplusplus
}
#endif

#endif	/* TIMER_MEMS_H */

