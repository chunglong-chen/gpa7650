#include "config/default/peripheral/port/plib_port.h"
#include "button.h"

#define BUTTON_DEBOUNCE_MS   30u          // Button debounce time in milliseconds

static bool     btn_raw_last       = false; // Last raw button state (true = pressed)
static bool     btn_stable_state   = false; // Debounced stable button state
static uint32_t btn_last_change_ts = 0u;    // Timestamp when the last raw change occurred

// Read the raw button state; active low ? 0 means pressed
// Assume KEY_EVENT_Get() returns the pin level: 0 = pressed, 1 = released
static inline bool BUTTON_IsPressed_RAW(void)
{
    return (KEY_EVENT_Get() == 0);         // Return true if the pin is low (pressed)
}

void BUTTON_Tasks(void)
{
    bool raw = BUTTON_IsPressed_RAW();      // Read the current raw button state
    uint32_t now = SYSTICK_GetTickCounter();// Read current system tick

    // Step 1: Detect any raw change (press/release/noise)
    if (raw != btn_raw_last) {
        btn_raw_last = raw;                 // Update last raw state
        btn_last_change_ts = now;           // Record the time of change
        return;                             // Wait for debounce, do not process yet
    }

    // Step 2: Check if the raw state has been stable long enough
    if ((now - btn_last_change_ts) < BUTTON_DEBOUNCE_MS)
        return;                             // Still within debounce period, exit

    // Step 3: If debounced state differs, it is a real button event
    if (raw != btn_stable_state) {
        btn_stable_state = raw;             // Update stable state

        if (btn_stable_state)               // If the stable state indicates "pressed"
            printf("[KEY EVENT]\r\n"); // Emit event message
        // If you want to log "released", add printf here
    }
}