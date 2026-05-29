/**
 * @file gpio_type_def.h
 * @brief Defines the GPIO_TYPEDef structure for representing a GPIO pin.
 *
 * @details This structure encapsulates the information of a single GPIO pin, including
 *          its port, pin number, and a direct address to its data register.
 */
#ifndef __GPIO_TYPE_DEF_H__
#define __GPIO_TYPE_DEF_H__
#include "NuMicro.h"

/**
 * @struct GPIO_TYPEDef
 * @brief Represents a GPIO pin with port, pin number, and data address.
 */
typedef struct{
    GPIO_T *                port;       /**< Pointer to the GPIO port (e.g., PA, PB, etc.) */
    uint32_t                pin;        /**< Pin number within the port */
    volatile uint32_t *     datAddr;    /**< Direct address to the data register (e.g., for fast access) */
}GPIO_TYPEDef;

#endif //__GPIO_TYPE_DEF_H__