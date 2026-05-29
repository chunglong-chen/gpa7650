#include "gp329.h"
#include "config/default/peripheral/port/plib_port.h"
#include "peripheral/systick/plib_systick.h"

#define GP329_PWRON_DELAY 500 // 500 ms
#define GP329_PWROFF_DELAY 3000 // 3000 ms
#define GP329_RESET_DELAY 1500 // 1500 ms

volatile bool is_gp329_running = false;

void GP329_Initialize(void) {

    PROBE_VCC_5V_EN_Set();
    GP329_PWRON_Clear();
    GP329_FW_UDATE_Clear();

    gp329_enable(true);
}

bool gp329_enable(bool enable) {

    return enable ? gp329_poweron() : gp329_poweroff();
}

//Simulate pressing the power button to power on GP329

bool gp329_poweron(void) {
    bool ret = false;
    if (!is_gp329_running) {
        GP329_PWRON_Set();
        SYSTICK_DelayMs(GP329_PWRON_DELAY);
        GP329_PWRON_Clear();
        is_gp329_running = true;
        ret = true;
    }
    
    return ret;
        
}

//Simulate long-pressing the power button(3s) to power off GP329

bool gp329_poweroff(void) {
    bool ret = false;
    if (is_gp329_running) {
        GP329_PWRON_Set();
        SYSTICK_DelayMs(GP329_PWROFF_DELAY);
        GP329_PWRON_Clear();
        is_gp329_running = false;
        ret = true;
    }

    return ret;

}

//Put GP329 into programming mode

bool gp329_fw_update(void) {
    if (!is_gp329_running) {
        GP329_FW_UDATE_Set();
        gp329_poweron();
        GP329_FW_UDATE_Clear();
        return true;
    }
    return false;
}

//Reset GP329

void gp329_reset(void) {

    PROBE_VCC_5V_EN_Clear();
    GP329_PWRON_Clear();
    GP329_FW_UDATE_Clear();
    SYSTICK_DelayMs(GP329_RESET_DELAY);
    PROBE_VCC_5V_EN_Set();
    is_gp329_running = false;
}