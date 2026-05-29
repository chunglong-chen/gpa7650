/**
 * @file delay.c
 * @brief Implementation of microseconds delay.
 */
#include <stdio.h>
#include "NuMicro.h"

/**
 * @brief Microsecond delay using NOPs.
 * @param u32Usec Number of microseconds to delay.
 */
void uDelay(uint32_t u32Usec)
{
    for (uint32_t i=0;i<u32Usec;i++)
    {
        for (uint16_t j=0;j<3;j++)
            __NOP();
    }
}

// trigger
// trigger