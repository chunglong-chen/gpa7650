/* 
 * File:   lowpass_filter.h
 * Author: A00280
 *
 * Created on 2024?11?14?, ?? 10:38
 */

#ifndef LOWPASS_FILTER_H
#define	LOWPASS_FILTER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define CLOCK_FREQUENCY 100000000 // 100MHz

    bool lp_filter_set_freq(int pwm_hz);
    uint32_t lp_filter_get_freq(void);
#ifdef	__cplusplus
}
#endif

#endif	/* LOWPASS_FILTER_H */

