#include "sys_led_blink.h"
#include "config/default/peripheral/port/plib_port.h"
#include "config/default/peripheral/tcc/plib_tcc0.h"

/* This function is called after period expires 
void TCC0_Callback_InterruptHandler(uint32_t status, uintptr_t context) {
    SYS_LED_Toggle();
}
*/
void sys_led_blink_initialize(void) {
    /* Register callback function for TCC0 period interrupt */
    //TCC0_TimerCallbackRegister(TCC0_Callback_InterruptHandler, (uintptr_t) NULL);
    /* Start the timer channel 0*/
//    TCC0_TimerStart();
}