/* 
 * File:   ina230.h
 * Author: A00361
 *
 * Created on August 19, 2024, 2:46 PM
 */

#ifndef INA230_H
#define	INA230_H
#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#ifdef	__cplusplus
extern "C" {
#endif
#define INA230_BASE    0x40

#define INA230_REGADD_CONFIG   0
#define INA230_REGADD_SHUNT    1
#define INA230_REGADD_BUS      2
#define INA230_REGADD_POWER    3
#define INA230_REGADD_CURRENT  4
#define INA230_REGADD_CALIBRATION  5
#define INA230_REGADD_MASK_EN  6
#define INA230_REGADD_ALERTLimit    7
#define INA230_REGADD_ID       0xFF

#define INA230_POWER_ALERT_OFFSET   11

typedef enum{
    ina230_powerdown = 0,
    ina230_shunt_trig = 1,
    ina230_bus_trig = 2,
    ina230_shunt_bus_trig = 3,
    ina230_shunt_cont = 5,
    ina230_bus_cont = 6,
    ina230_shunt_bus_cont = 7
}INA230_MODE;

typedef enum{
    ina230_avg_1 = 0,
    ina230_avg_4 = 1,
    ina230_avg_16 = 2,
    ina230_avg_64 = 3,
    ina230_avg_128 = 4,
    ina230_avg_256 = 5,
    ina230_avg_512 = 6,
    ina230_avg_1024 = 7
}INA230_AVG_NO;

typedef enum{
    ina230_ct_140us = 0,
    ina230_ct_204us = 1,
    ina230_ct_332us = 2,
    ina230_ct_588us = 3,
    ina230_ct_1100us = 4,
    ina230_ct_2116us = 5,
    ina230_ct_4156us = 6,
    ina230_ct_8244us = 7
}INA230_CONV_TIME;

typedef union{
    uint16_t reg;
    struct{
        unsigned mode : 3;
        unsigned shunt_ct: 3;
        unsigned bus_ct : 3;
        unsigned average : 3;
        unsigned : 3;
        unsigned reset : 1;
    }__attribute__ ((packed));
}INA230_CONFIG_REGISTER;
typedef enum {
        INA230_INIT,
        
    } INA230_STATES;
typedef struct {
        INA230_STATES state;

        uint8_t txBuffer[3];
        
        uint8_t rxBuffer[2];

        volatile bool isTransferDone;

    } INA230_DATA;
void INA230_Initialize(void);
uint16_t ina230_read_id();
uint16_t ina230_read_alert();
uint16_t ina230_read_shunt_volt();
uint16_t ina230_read_volt();
double ina230_read_current();
uint16_t ina230_read_power();
bool ina230_set_power_alert(double watt);
double ina230_get_power_alert();



#ifdef	__cplusplus
}
#endif

#endif	/* INA230_H */

