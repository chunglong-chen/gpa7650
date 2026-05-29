#include "lowpass_filter.h"
#include <stdio.h>
#include "config/default/peripheral/tcc/plib_tcc3.h"

bool lp_filter_set_freq(int pwm_hz) {
    bool ret = false;

    if (pwm_hz <= 0)
        return ret;

    uint32_t pwm_period = CLOCK_FREQUENCY / pwm_hz - 1;

    TCC3_PWMStop();
    if (!TCC3_PWM32bitPeriodSet(pwm_period)) {
        return ret;
    }
    /* Set duty value of channel 0 & 1 to 50% */
    if (!TCC3_PWM32bitDutySet(TCC3_CHANNEL0, pwm_period / 2)) {
        return ret;
    }
    if (!TCC3_PWM32bitDutySet(TCC3_CHANNEL1, pwm_period / 2)) {
        return ret;
    }
    TCC3_PWMStart();
    ret = true;

    return ret;
}

uint32_t lp_filter_get_freq(void) {
    uint32_t pwm_hz = CLOCK_FREQUENCY / (TCC3_PWM32bitPeriodGet() + 1);
    return pwm_hz;
}