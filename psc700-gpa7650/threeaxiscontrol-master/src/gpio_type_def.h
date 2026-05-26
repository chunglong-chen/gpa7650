#ifndef __GPIO_TYPE_DEF_H__
#define __GPIO_TYPE_DEF_H__
#include "NuMicro.h"

typedef struct{
    GPIO_T *                port;
    uint32_t                pin;
    volatile uint32_t *     datAddr;
}GPIO_TYPEDef;

#endif //__GPIO_TYPE_DEF_H__