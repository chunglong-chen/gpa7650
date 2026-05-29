/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include <stdio.h>
#include "version.h"
#include "build_info.h"

#define SYSTEM_READY_DELAY_MS           1000

uint32_t led_cnt = 0; // Count to toggle System LED
// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );

    printf("ACT 100 peripheral controller ver. %s\r\n", VERSION);
    printf("%s build %s, %s\r\n", DATETIMESTR, FIRMWARE_VERSION, GIT_HASH);
    
    SYSTICK_TimerStart(); // This function resets and starts the System Timer.
    
    WDT_TimeoutPeriodSet(WDT_CONFIG_PER_CYC4096_Val);// WDT timeout period 4 seconds

    WDT_Enable();
    
    /* Start PWM*/
    TCC3_PWMStart();
    
    SYSTICK_DelayMs(SYSTEM_READY_DELAY_MS);
    USB_HUB_RST_Set(); // Pull high
    
    while ( true )
    {
        WDT_Clear();
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );

        /* Blink the green LED in the main loop, not with a timer, so it reflects system activity. */
        led_cnt++;
        if (led_cnt > 25*10000) {
            led_cnt = 0;
            SYS_LED_Toggle();
        }
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

