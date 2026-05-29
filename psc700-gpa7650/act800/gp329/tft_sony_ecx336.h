/**
 * @file    tft_sony_ecx336.h
 * @brief   Sony ECX336 OLED power control interface.
 * @ingroup fixation_display_controller
 * @details
 * This header file provides control interfaces for powering
 * on and off the Sony ECX336 OLED display module.
 *
 * These functions control the display power sequence,
 * including reset and power rail control.
 */
#ifndef __TFT_SONY_ECX336_H__
#define __TFT_SONY_ECX336_H__

void   tft_sony_ecx336_power_off(void);
void   tft_sony_ecx336_power_on(void);

#endif //__TFT_SONY_ECX336_H__
