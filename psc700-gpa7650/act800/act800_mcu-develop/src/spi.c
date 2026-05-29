#include "spi.h"
#include <stdio.h>
#include <string.h>
#include "config/default/peripheral/dma/plib_dma.h"
#include "config/default/peripheral/port/plib_port.h"
#include "peripheral/systick/plib_systick.h"
#include "config/default/peripheral/sercom/spi_master/plib_sercom0_spi_master.h"
#include "user.h"
#include "proc_cmd.h"

DAC8564_VOLT_DATA mems_dac_data;

uint32_t* current_data_ptr;
volatile size_t current_data_index;
size_t total_data_length;

void Toggle_LDAC(void) {
    MEMS_LDAC_Set(); // Pull high
    SYSTICK_DelayUs(0.5);
    MEMS_LDAC_Clear(); // Pull low
}

void SPI_Write(uint16_t data) {

    // This function reverses the order of 4 bytes in a 32-bit integer.
    mems_dac_data.xp_code = ((uint32_t) (data & 0x00FF) << 16) | ((uint32_t) (data & 0xFF00)) | MEMS_CHXP;
    mems_dac_data.xn_code = ((uint32_t) (data & 0x00FF) << 16) | ((uint32_t) (data & 0xFF00)) | MEMS_CHXN;
    mems_dac_data.yp_code = ((uint32_t) (data & 0x00FF) << 16) | ((uint32_t) (data & 0xFF00)) | MEMS_CHYP;
    mems_dac_data.yn_code = ((uint32_t) (data & 0x00FF) << 16) | ((uint32_t) (data & 0xFF00)) | MEMS_CHYN;

    // Clean data to the cache
    DCACHE_CLEAN_BY_ADDR((uint32_t *) & mems_dac_data, sizeof (mems_dac_data));

    current_data_ptr = (uint32_t*) & mems_dac_data;
    current_data_index = 0;
    total_data_length = sizeof (mems_dac_data) / sizeof (uint32_t);

    // Start first DMA transmit
    DMA_ChannelTransfer(DMA_CHANNEL_0, &current_data_ptr[current_data_index], (const void*) &SERCOM0_REGS->SPIM.SERCOM_DATA, sizeof (uint32_t));
    /*
     * DMA_ChannelTransfer() is a software function that takes some time (about 500ns) to transmit data.
     * Therefore, pull the SPI_CS pin low after invoking DMA_ChannelTransfer().
     */
    SPI_CS_Clear();
}

void spi_write2dac(MEMS_DATA mems_data) {
    uint32_t xp_data, xn_data, yp_data, yn_data;
    // Calculate DAC values for X+ (xp_data), X- (xn_data), Y+ (yp_data), and Y- (yn_data) channel.
    xp_data = mems_volt2code(mems_data.xp_volt);
    xn_data = mems_volt2code(mems_data.xn_volt);
    yp_data = mems_volt2code(mems_data.yp_volt);
    yn_data = mems_volt2code(mems_data.yn_volt);
    // This function reverses the order of 4 bytes in a 32-bit integer.
    mems_dac_data.xp_code = ((uint32_t) (xp_data & 0x00FF) << 16) | ((uint32_t) (xp_data & 0xFF00)) | MEMS_CHXP;
    mems_dac_data.xn_code = ((uint32_t) (xn_data & 0x00FF) << 16) | ((uint32_t) (xn_data & 0xFF00)) | MEMS_CHXN;
    mems_dac_data.yp_code = ((uint32_t) (yp_data & 0x00FF) << 16) | ((uint32_t) (yp_data & 0xFF00)) | MEMS_CHYP;
    mems_dac_data.yn_code = ((uint32_t) (yn_data & 0x00FF) << 16) | ((uint32_t) (yn_data & 0xFF00)) | MEMS_CHYN;

    // Clean data to the cache
    DCACHE_CLEAN_BY_ADDR((uint32_t *) & mems_dac_data, sizeof (mems_dac_data));

    current_data_ptr = (uint32_t*) & mems_dac_data;
    current_data_index = 0;
    total_data_length = sizeof (mems_dac_data) / sizeof (uint32_t);
    SPI_CS_Clear(); // Pull SPI_CS pin High
    // Start first DMA transmit
    DMA_ChannelTransfer(DMA_CHANNEL_0, &current_data_ptr[current_data_index], (const void*) &SERCOM0_REGS->SPIM.SERCOM_DATA, sizeof (uint32_t));
}

void DMA_EventHandler(DMA_TRANSFER_EVENT event, uintptr_t contextHandle) {

    SPI_CS_Set(); // Pull SPI_CS pin High
    // Move to the next data block
    current_data_index++;
    if (current_data_index < total_data_length) {
        SPI_CS_Clear(); // Pull SPI_CS pin Low
        // Start the next DMA transfer
        DMA_ChannelTransfer(DMA_CHANNEL_0, &current_data_ptr[current_data_index], (const void*) &SERCOM0_REGS->SPIM.SERCOM_DATA, sizeof (uint32_t));
    }
}

void SPI_Initialize(void) {
    /* Place the App state machine in its initial state. */

    DMA_ChannelCallbackRegister(DMA_CHANNEL_0, DMA_EventHandler, 0);

}

void SPI_Tasks(void) {

}

/*******************************************************************************
 End of File
 */