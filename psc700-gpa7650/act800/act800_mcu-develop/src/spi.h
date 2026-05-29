/* 
 * File:   spi.h
 * Author: A00280
 *
 * Created on June 18, 2024, 2:11 PM
 */

#ifndef SPI_H
#define	SPI_H

#include "config/default/driver/spi/drv_spi.h"
#include "mems.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define MEMS_CHXP   0x00 //DAC Buffer A
#define MEMS_CHXN   0x02 //DAC Buffer B
#define MEMS_CHYN   0x04 //DAC Buffer C
#define MEMS_CHYP   0x06 //DAC Buffer D

    typedef struct {
        uint32_t xp_code;
        uint32_t xn_code;
        uint32_t yp_code;
        uint32_t yn_code;
    } DAC8564_VOLT_DATA;

    void SPI_Initialize(void);

    void SPI_Tasks(void);

    void SPI_Write(uint16_t data);

    void spi_write2dac(MEMS_DATA mems_data);

    void Toggle_LDAC(void);

#ifdef	__cplusplus
}
#endif

#endif	/* SPI_H */

