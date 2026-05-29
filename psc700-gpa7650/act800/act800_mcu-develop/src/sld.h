/**
 * @file    sld.h
 * @brief   SLD control and monitoring interface definitions.
 * @details This file declares the data types, configuration macros,
 *          runtime status variables, and public APIs for SLD control.
 *          It supports SLD enable control, ADC sample acquisition,
 *          photodiode threshold monitoring, and watchdog-based shutdown.
 * @ingroup sld_controller
 */
#ifndef SLD_H
#define SLD_H

#include <stdint.h>
#include <stdbool.h>
#include "filesys.h"

#ifdef __cplusplus
extern "C" {
#endif
/* ========== User Config ========== */
#define SLD_SAFE_TIME          5000    /* ms: auto-shutdown after enabled this long */
#define SLD_ADC_VREF           3.3f    /* voltage reference for conversion to volts */
 /* Ring buffer: size must be power of two for fast masking */
#define SLD_ADC_BUF_SIZE       1024u
#define SLD_ADC_IDX_MASK       (SLD_ADC_BUF_SIZE - 1u)
typedef struct {
    uint16_t ch0;  /* average raw value of TP74 (ADC CH0) */
    uint16_t ch2;  /* average raw value of TP77 (ADC CH2) */
    uint16_t ch4;  /* average raw value of TP90 (ADC CH4) */
} SLD_ADC_Result;
typedef struct {
    uint32_t tick_ms;   /* sampling timestamp from SYSTICK_GetTickCounter() */
    uint16_t ch0;       /* ADC_CH0 (TP74) */
    uint16_t ch2;       /* ADC_CH2 (TP77) */
    uint16_t ch4;       /* ADC_CH4 (TP90) */
} SLD_ADC_Sample;

/* ========== Module State ========== */
static volatile bool     enable_cover_detection = true;

static volatile uint32_t lasttime_sld_en_tick = 0;

/* ring buffer producer (ISR) / consumer (command) */
static volatile uint32_t rb_w = 0;             /* write index (monotonic, masked on access) */
static volatile uint32_t rb_r = 0;             /* read  index (monotonic, masked on access) */


static volatile bool     rb_capture_enabled = false;

/* gather-per-trigger temps */
static volatile uint32_t ready_mask = 0;       /* accumulate CHRDY flags for one trigger */
static volatile uint16_t tmp_ch0 = 0;
static volatile uint16_t tmp_ch2 = 0;
static volatile uint16_t tmp_ch4 = 0;

static volatile uint16_t sld_pd_threshold = 0;
extern uint16_t sld_flag ;  
void SLD_Initialize(void);
bool sld_enable(bool enable);
bool sld_get_enable(void);
void sld_cover_check_enable(bool enable);
bool sld_adc(SLD_ADC_Result* out);
void sld_result(void);
void SLD_Tasks(void);
void sld_set_pd_threshold(uint16_t thr);
uint16_t sld_get_pd_threshold(void);

#ifdef __cplusplus
}
#endif

#endif /* SLD_H */

