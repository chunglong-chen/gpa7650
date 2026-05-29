/**
 * @file    sld.c
 * @brief   SLD control and monitoring module.
 * @details This module controls the SLD enable signal and monitors its
 *          operating status using ADC sampling. It implements periodic
 *          ADC acquisition triggered by timer events, stores samples in
 *          a ring buffer, and provides safety mechanisms such as
 *          photodiode threshold shutdown and watchdog timeout.
 *
 *          Key features:
 *          - Timer-triggered multi-channel ADC sampling
 *          - Ring buffer storage of ADC samples
 *          - Average value calculation
 *          - Safety shutdown based on photodiode threshold
 *          - Watchdog-based auto shutdown
 * @ingroup sld_controller
 */
#include "sld.h"
#include <stdio.h>
#include "config/default/peripheral/port/plib_port.h"
#include "config/default/peripheral/adc/plib_adc.h"
#include "peripheral/systick/plib_systick.h"
#include "config/default/peripheral/tcc/plib_tcc0.h"

static SLD_ADC_Sample rb[SLD_ADC_BUF_SIZE];
uint16_t sld_flag = 0;  
/* ========== Helpers ========== */
static inline bool rb_empty(void) {
    return (rb_r == rb_w);
}
static inline bool rb_full(void) {
    return (((rb_w + 1u) & SLD_ADC_IDX_MASK) == (rb_r & SLD_ADC_IDX_MASK));
}
static inline void rb_clear(void) {
    rb_r = rb_w = 0;
}

/* Start/stop capture ? called when SLD enable/disable */
static inline void sld_adc_capture_start(void) {
    rb_clear();
    ready_mask = 0;
    rb_capture_enabled = true;
}
static inline void sld_adc_capture_stop(void) {
    rb_capture_enabled = false;
}

/* ========== Interrupt Callbacks ========== */
/**
 * @brief   Timer interrupt callback for ADC trigger.
 * @details Triggered periodically by TCC0 timer to start ADC conversion.
 *
 * @param[in] status   Interrupt status.
 * @param[in] context  User context.
 */
// TCC0 period ISR: one trigger = one "frame" (CH0+CH2+CH4) 
void TCC0_Callback_InterruptHandler(uint32_t status, uintptr_t context) {
    (void)status; (void)context;
    ADC_GlobalEdgeConversionStart();   /* hardware-global edge trigger */
}
/**
 * @brief   ADC interrupt event handler.
 * @details Collects ADC conversion results for multiple channels and
 *          stores them into a ring buffer when all channels are ready.
 *
 * @param[in] status   ADC interrupt status.
 * @param[in] context  User context.
 */
// ADC ISR: collect per-channel results; when 3 channels ready -> push one sample 
void ADC_EventHandler(ADC_CORE_INT status, uintptr_t context) {
    (void)context;

    if (status & ADC_CORE_INT_CHRDY_0) {
        tmp_ch0 = (uint16_t)ADC_ResultGet(ADC_CORE_NUM3, ADC_CH0);
        ready_mask |= ADC_CORE_INT_CHRDY_0;
    }
    if (status & ADC_CORE_INT_CHRDY_2) {
        tmp_ch2 = (uint16_t)ADC_ResultGet(ADC_CORE_NUM3, ADC_CH2);
        ready_mask |= ADC_CORE_INT_CHRDY_2;
    }
    if (status & ADC_CORE_INT_CHRDY_4) {
        tmp_ch4 = (uint16_t)ADC_ResultGet(ADC_CORE_NUM3, ADC_CH4);
        ready_mask |= ADC_CORE_INT_CHRDY_4;
    }

    const uint32_t ALL = (ADC_CORE_INT_CHRDY_0 | ADC_CORE_INT_CHRDY_2 | ADC_CORE_INT_CHRDY_4);

    /* When all three channels are ready -> commit one sample into ring buffer */
    if ((ready_mask & ALL) == ALL) {
        ready_mask = 0;

        if (rb_capture_enabled) {
            uint32_t w = (rb_w & SLD_ADC_IDX_MASK);

            if (rb_full()) {
                /* overwrite oldest: advance consumer to keep data flowing */
                rb_r = (rb_r + 1u) & 0xFFFFFFFFu;
            }

            rb[w].tick_ms = SYSTICK_GetTickCounter();
            rb[w].ch0     = tmp_ch0;
            rb[w].ch2     = tmp_ch2;
            rb[w].ch4     = tmp_ch4;

            rb_w = (rb_w + 1u) & 0xFFFFFFFFu;
        }
    }
}

/**
 * @brief   Initialize SLD module.
 * @details Registers timer and ADC interrupt callbacks required for
 *          periodic sampling and monitoring.
 */
void SLD_Initialize(void) {
    /* Register callbacks (assumes peripheral init is done elsewhere) */
    TCC0_TimerCallbackRegister(TCC0_Callback_InterruptHandler, (uintptr_t)NULL);
    ADC_CORE3CallbackRegister(ADC_EventHandler, 0);
}
/**
 * @brief   Enable or disable SLD.
 * @details Controls SLD enable pin and starts or stops ADC sampling
 *          and timer accordingly.
 *
 * @param[in] enable  true to enable SLD, false to disable.
 *
 * @return Current SLD flag status.
 */
bool sld_enable(bool enable) {
    if(sld_flag!=1){
        if (enable) {
            lasttime_sld_en_tick = SYSTICK_GetTickCounter();
            //sld_adc_capture_start();   /* begin capturing into ring buffer */
        }
        /* Control external SLD enable pin */
        PORT_PinWrite(SLD_EN_PIN, enable);
        if (enable) {
            TCC0_TimerStart();
        } else {
            TCC0_TimerStop();
            sld_adc_capture_stop();    /* stop capturing when turning SLD off */
        }
        return sld_flag;
    }
    else{
        return sld_flag;
    }
}
/**
 * @brief   Get SLD enable status.
 *
 * @return true if SLD is enabled, otherwise false.
 */
bool sld_get_enable(void) {
    return SLD_EN_Get();
}
/**
 * @brief   Enable or disable cover detection.
 *
 * @param[in] enable  true to enable cover detection.
 */
void sld_cover_check_enable(bool enable) {
    enable_cover_detection = enable;
}
/**
 * @brief   Capture ADC samples and calculate average values.
 * @details Captures ADC samples for a fixed duration (~1 second),
 *          stores them in a ring buffer, and calculates average
 *          values for each channel.
 *
 * @param[out] out  Pointer to result structure.
 *
 * @return true  If capture is successful.
 * @return false If no valid data is captured.
 */
//Capture ADC samples for ~1 second and return average raw values of three channels. 
bool sld_adc(SLD_ADC_Result* out){
    if (out == NULL) {
        return false;
    }
    /* Reset ring buffer indices and ready mask for a fresh capture */
    rb_r = 0;
    rb_w = 0;
    ready_mask = 0;

    /* Start capture and TCC0 timer */
    sld_adc_capture_start();
    TCC0_TimerStart();

    uint32_t start_ms = SYSTICK_GetTickCounter();
    while ((uint32_t)(SYSTICK_GetTickCounter() - start_ms) < 1000u) {
        /* Busy-wait for ~1 second.
         * If your system allows, you can replace this with sleep/WFI. */
    }

    /* Stop timer and stop capturing new samples */
    TCC0_TimerStop();
    sld_adc_capture_stop();

    /* Snapshot producer/consumer indices after capture is stopped */
    uint32_t r = rb_r;      // ?????? 0
    uint32_t w = rb_w;
    if (r == w) {           /* No samples captured in this interval */
        return false;
    }
    uint64_t sum_ch0 = 0;
    uint64_t sum_ch2 = 0;
    uint64_t sum_ch4 = 0;
    uint32_t count   = 0;
    while (r != w) {
        uint32_t idx = (r & SLD_ADC_IDX_MASK);
        const SLD_ADC_Sample* s = &rb[idx];
        sum_ch0 += s->ch0;
        sum_ch2 += s->ch2;
        sum_ch4 += s->ch4;
        count++;
        r++;
    }

    if (count == 0u) {
        return false;
    }
    out->ch0 = (uint16_t)(sum_ch0 / count);
    out->ch2 = (uint16_t)(sum_ch2 / count);
    out->ch4 = (uint16_t)(sum_ch4 / count);
    /* NOTE: do NOT modify rb_r here.
     * Let sld_result() consume/dump the samples later.
     */
    return true;
}
/**
 * @brief   Print captured ADC samples.
 * @details Outputs stored ADC samples from the ring buffer,
 *          converting raw values into voltage representation.
 */
void sld_result(void)
{
    /* Pause capture while dumping buffer to avoid races */
    bool was_capturing = rb_capture_enabled;
    sld_adc_capture_stop();

    /* Snapshot producer/consumer indices */
    uint32_t r = rb_r;
    uint32_t w = rb_w;

    if (r == w) {
        printf("SLD ADC buffer is empty.\r\n");
    } else {
        printf("SLD ADC samples (t(ms), raw -> volt):\r\n");
    }

    while (r != w) {
        uint32_t idx = (r & SLD_ADC_IDX_MASK);
        const SLD_ADC_Sample *s = &rb[idx];

        float v0 = (float)s->ch0 * SLD_ADC_VREF / 4095.0f;
        float v2 = (float)s->ch2 * SLD_ADC_VREF / 4095.0f;
        float v4 = (float)s->ch4 * SLD_ADC_VREF / 4095.0f;

        printf("[t=%lu] Limit = %u (%.3f V), Out = %u (%.3f V), Real = %u (%.3f V)\r\n",
               (unsigned long)s->tick_ms,
               (unsigned)s->ch0, v0,
               (unsigned)s->ch2, v2,
               (unsigned)s->ch4, v4);
        r++;
    }

    /* Mark all printed samples as consumed (clear buffer) */
    rb_r = w;

    /* Optionally resume capture if SLD still enabled */
    if (was_capturing && SLD_EN_Get()) {
        sld_adc_capture_start();   /* restart fresh buffer */
    }
}
/**
 * @brief   Set photodiode threshold.
 *
 * @param[in] thr  Threshold value (0~4095).
 */
void sld_set_pd_threshold(uint16_t thr)
{
    sld_pd_threshold = thr;    /* expected range: 0 ~ 4095 */
}
/**
 * @brief   Get photodiode threshold.
 *
 * @return Current threshold value.
 */
uint16_t sld_get_pd_threshold(void)
{
    return sld_pd_threshold;
}
/**
 * @brief   SLD monitoring and safety task. 
 * @details Monitors SLD operating conditions and performs automatic
 *          shutdown when abnormal conditions are detected.
 *
 *          Safety mechanisms:
 *          - Photodiode threshold check
 *          - Watchdog timeout shutdown
 */
// Task: auto-shutdown after SLD_SAFE_TIME; also stop capture 
void SLD_Tasks(void) {
    if (SLD_EN_Get()) {
        /* Within the safe time window, check PD OUT (CH2) against threshold */
        if ( sld_pd_threshold != 0u) {
            uint16_t pd_raw = tmp_ch2;  /* latest CH2 value from ADC ISR */
            if (pd_raw > sld_pd_threshold) {
                sld_flag = 1;
                SST26_WriteSLD("sldflg", sld_flag);
                SLD_EN_Clear();
                sld_adc_capture_stop();
                TCC0_TimerStop();
                float pd_v  = (float)(pd_raw * SLD_ADC_VREF / 4095.0f);
                float thr_v = (float)(sld_pd_threshold * SLD_ADC_VREF / 4095.0f);
                printf("SLD shutdown: PD OUT %u (%.3f V) > threshold %u (%.3f V)\r\n",
                       (unsigned)pd_raw, pd_v,
                       (unsigned)sld_pd_threshold, thr_v);
                return;    /* already shut down, no need to check WDT */
            }
        }
        if (enable_cover_detection) {
            if (SYSTICK_GetTickCounter() - lasttime_sld_en_tick > SLD_SAFE_TIME) {
                SLD_EN_Clear();
                sld_adc_capture_stop();
                TCC0_TimerStop();
                printf("SLD wdt shutdown\r\n");
                return;
            }
        }
    }
}
