#include <stdio.h>
#include "NuMicro.h"

void uDelay(uint32_t u32Usec)
{
    for (uint32_t i=0;i<u32Usec;i++)
    {
        for (uint16_t j=0;j<3;j++)
            __NOP();
    }
}