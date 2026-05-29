/**
 * @file    tft_sony_ecx336.c
 * @brief   Sony ECX336 OLED power control interface.
 * @ingroup fixation_display_controller
 * @details
 * This header file provides control interfaces for powering
 * on and off the Sony ECX336 OLED display module.
 *
 * These functions control the display power sequence,
 * including reset and power rail control.
 */
#include "tft_sony_ecx336.h"
#include "drv_l1_gpio.h"
#include "drv_l1_sfr.h"
#include "drv_l1_spi.h"
#include "drv_l1_tft.h"
#include "drv_l2_display.h"
#include "drv_l2_spi_flash.h"
#include "gpio_init.h"
#include "gplib_print_string.h"

#if (defined SONY_ECX336) && (SONY_ECX336 == 1)

#define SPIOUT_REVERSE 1

#define M_SPI_NUM      SPI_0 // use spi 0
#define M_SPI_CS_HIGH  gpio_write_io(SPI0_CS, GPIO_HIGH);
#define M_SPI_CS_LOW   gpio_write_io(SPI0_CS, GPIO_LOW);
#define SPI0_CS_INIT                                 \
    {                                                \
        gpio_init_io(SPI0_CS, GPIO_OUTPUT);          \
        gpio_set_port_attribute(SPI0_CS, GPIO_HIGH); \
        gpio_write_io(SPI0_CS, GPIO_HIGH);           \
    }
#define RD_ON              0x80           // Register on / off
#define RD_ADDR            0x81           // Register read address setting
#define TFT_TAR_CLK        (27 * 1000000) // 27MHz

#define SONY_H_BACK_PORCH  58
#define SONY_H_SYNC        64
#define SONY_H_ACTIVE      640
#define SONY_H_FRONT_PORCH 96
#define SONY_H_TOTAL       SONY_H_BACK_PORCH + SONY_H_SYNC + SONY_H_ACTIVE + SONY_H_FRONT_PORCH // 858

#define SONY_V_BACK_PORCH  32
#define SONY_V_SYNC        6
#define SONY_V_ACTIVE      400
#define SONY_V_FRONT_PORCH 87
#define SONY_V_TOTAL       SONY_V_BACK_PORCH + SONY_V_SYNC + SONY_V_ACTIVE + SONY_V_FRONT_PORCH // 525

typedef struct
{
    INT8U addr;
    INT8U val;
} sony_regval_t;

const sony_regval_t m_sony_init_regs[] = {
    {0x00, 0x0E},
    {0x01, 0x00},
    {0x02, 0x00},
    {0x03, 0x00},
    {0x04, 0x3F},
    {0x05, 0xC3},
    {0x06, 0x00},
    {0x07, 0x40},
    {0x08, 0x00},
    {0x09, 0x00},
    {0x0A, 0x10},
    {0x0B, 0x00},
    {0x0C, 0x00},
    {0x0D, 0x00},
    {0x0E, 0x00},
    {0x0F, 0x56},
    {0x10, 0x00},
    {0x11, 0x00},
    {0x12, 0x00},
    {0x13, 0x00},
    {0x14, 0x00},
    {0x15, 0x00},
    {0x16, 0x00},
    {0x17, 0x00},
    {0x18, 0x00},
    {0x19, 0x00},
    {0x1A, 0x00},
    {0x1B, 0x00},
    {0x1C, 0x00},
    {0x1D, 0x00},
    {0x1E, 0x00},
    {0x1F, 0x00},
    {0x20, 0x01},
    {0x21, 0x00},
    {0x22, 0x40},
    {0x23, 0x40},
    {0x24, 0x40},
    {0x25, 0xFF},
    {0x26, 0x40},
    {0x27, 0x40},
    {0x28, 0x40},
    {0x29, 0x0B},
    {0x2A, 0xBE},
    {0x2B, 0x3C},
    {0x2C, 0x02},
    {0x2D, 0x7A},
    {0x2E, 0x02},
    {0x2F, 0xFA},
    {0x30, 0x26},
    {0x31, 0x01},
    {0x32, 0xB6},
    {0x33, 0x00},
    {0x34, 0x03},
    {0x35, 0x5A},
    {0x36, 0x00},
    {0x37, 0x76},
    {0x38, 0x02},
    {0x39, 0xFE},
    {0x3A, 0x02},
    {0x3B, 0x0D},
    {0x3C, 0x00},
    {0x3D, 0x1B},
    {0x3E, 0x00},
    {0x3F, 0x1C},
    {0x40, 0x01},
    {0x41, 0xF3},
    {0x42, 0x01},
    {0x43, 0xF4},
    {0x44, 0x80},
    {0x45, 0x00},
    {0x46, 0x00},
    {0x47, 0x41},
    {0x48, 0x08},
    {0x49, 0x02},
    {0x4A, 0xFC},
    {0x4B, 0x08},
    {0x4C, 0x16},
    {0x4D, 0x08},
    {0x4E, 0x00},
    {0x4F, 0x4E},
    {0x50, 0x02},
    {0x51, 0xC2},
    {0x52, 0x01},
    {0x53, 0x2D},
    {0x54, 0x01},
    {0x55, 0x2B},
    {0x56, 0x00},
    {0x57, 0x2B},
    {0x58, 0x23},
    {0x59, 0x02},
    {0x5A, 0x25},
    {0x5B, 0x02},
    {0x5C, 0x25},
    {0x5D, 0x02},
    {0x5E, 0x1D},
    {0x5F, 0x00},
    {0x60, 0x23},
    {0x61, 0x02},
    {0x62, 0x1D},
    {0x63, 0x00},
    {0x64, 0x1A},
    {0x65, 0x03},
    {0x66, 0x0A},
    {0x67, 0xF0},
    {0x68, 0x00},
    {0x69, 0xF0},
    {0x6A, 0x00},
    {0x6B, 0x00},
    {0x6C, 0x00},
    {0x6D, 0xF0},
    {0x6E, 0x00},
    {0x6F, 0x60},
    {0x70, 0x00},
    {0x71, 0x00},
    {0x72, 0x00},
    {0x73, 0x00},
    {0x74, 0x00},
    {0x75, 0x00},
    {0x76, 0x00},
    {0x77, 0x00},
    {0x78, 0x00},
    {0x79, 0x68},
    {0x7A, 0x00},
    {0x7B, 0x00},
    {0x7C, 0x00},
    {0x7D, 0x00},
    {0x7E, 0x00},
    {0x7F, 0x00},
    {RD_ON, 0xFF} //{RD_ON, 0xFF},
    //{0x00, 0x0F},
};

/**
 * @brief   Reverses the bits of a byte.
 *          Example: REVERSE 0x0E(binary: 00001110) results in 0x70 (binary: 01110000).
 * @param   val
 * @return  INT8U
 */
INT8U REVERSE(INT8U val) {
    INT8U ret = 0;

    ret |= (val & 0x01) << 7;
    ret |= (val & 0x02) << 5;
    ret |= (val & 0x04) << 3;
    ret |= (val & 0x08) << 1;
    ret |= (val & 0x10) >> 1;
    ret |= (val & 0x20) >> 3;
    ret |= (val & 0x40) >> 5;
    ret |= (val & 0x80) >> 7;

    return ret;
}

/**
 * @brief   Write data to SPI device
 * @param   addr
 * @param   value
 * @return  STATUS_OK / STATUS_FAIL
 */
INT32S sony_write_reg(INT8U addr, INT8U value) {
    INT32S ret = STATUS_OK;
    INT8U  buf[2];
// DBG_PRINT("write reg %#x, %#x\r\n", addr, value);
#if SPIOUT_REVERSE
    buf[0] = REVERSE(addr);
    buf[1] = REVERSE(value);
#else
    buf[0] = addr;
    buf[1] = value;
#endif
    M_SPI_CS_LOW;
    // ret = drv_l1_spi_transceive(M_SPI_NUM, buf, 2, buf, 0);
    ret = drv_l1_spi_transceive_cpu(M_SPI_NUM, buf, 2, buf, 0);
    M_SPI_CS_HIGH;

    return ret;
}

/**
 * @brief   Power off the Sony ECX336 Oled
 * @details
 * This function powers off the display by entering power-saving mode,
 * disabling reset, and turning off related power rails.
 * @param   none
 * @return  none
 */
void tft_sony_ecx336_power_off(void) {
    sony_write_reg(0x00, 0x0E); // enter power-saving mode
    osDelay(50);
    gpio_set_value(TFT_RST, GPIO_LOW);
    osDelay(50);
    gpio_set_value(M_TFT_EN, GPIO_LOW);
    gpio_set_value(UDIS_EN, GPIO_LOW);
}

/**
 * @brief   Power on the Sony ECX336 Oled
 * @details
 * This function performs the power-on sequence of the display,
 * including power rail enable, reset control, SPI initialization,
 * and register configuration.
 * @param   none
 * @return  none
 */
void tft_sony_ecx336_power_on(void) {
    INT32U i;

    gpio_set_value(TFT_RST, GPIO_LOW);
    gpio_set_value(M_TFT_EN, GPIO_LOW);
    gpio_set_value(UDIS_EN, GPIO_LOW);

    osDelay(100);

    gpio_set_value(M_TFT_EN, GPIO_HIGH); // 1.8V power
    osDelay(50);
    gpio_set_value(TFT_RST, GPIO_HIGH);
    osDelay(50);
    gpio_set_value(UDIS_EN, GPIO_HIGH); // 10V power
    osDelay(50);

    drv_l1_spi_clk_set(M_SPI_NUM, 500000);


    for (i = 0; i < (sizeof(m_sony_init_regs) / sizeof(sony_regval_t)); i++) {
        sony_write_reg(m_sony_init_regs[i].addr, m_sony_init_regs[i].val);
        osDelay(5);
    }
    drv_l1_spi_clk_set(M_SPI_NUM, 1000000); // 1MHz
    osDelay(20);// osDelay(20);
    // Power on Sony Oled
    sony_write_reg(0x00, 0x0F);
    osDelay(20);

}
/**
 * @brief   Initialize TFT controller for Sony ECX336 panel.
 *
 * @details
 * This function configures the TFT controller timing parameters,
 * SPI interface, and display output mode.
 *
 * It sets horizontal and vertical timing, clock configuration,
 * and enables the display output before powering on the panel.
 *
 * @param tft_index TFT device index.
 * @return STATUS_OK on success.
 */
static INT32S tft_sony_ecx336_init(INT32U tft_index) {

    DBG_PRINT("tft_sony_ecx336_spi_init \r\n");
    SPI0_CS_INIT;
    // Enable SPI
    R_SPI0_CTRL |= 0x8000;

    drv_l1_spi_init(M_SPI_NUM);
    drv_l1_spi_clk_set(M_SPI_NUM, 1000000); // 900000 = 1.26MHz
    osDelay(10);

    DBG_PRINT("tft_sony_ecx336_init \r\n");
    // TFT Setting begin
    // Important: The period register should -1 is required.
    R_TFT_V_PERIOD = SONY_V_TOTAL -1;               // V total = 525
    R_TFT_V_START = SONY_V_BACK_PORCH + SONY_V_SYNC; // V Back Porch + V Sync = 32 + 6 = 38
    R_TFT_V_END = SONY_V_TOTAL - SONY_V_FRONT_PORCH; // V total - V Front Porch = 525 - 87 = 438

    R_TFT_H_PERIOD = SONY_H_TOTAL - 1;               // H total = 858
    R_TFT_H_START = SONY_H_BACK_PORCH + SONY_H_SYNC; // H Back Porch + H Sync = 58 + 64 = 122
    R_TFT_H_END = SONY_H_TOTAL - SONY_H_FRONT_PORCH; // H total - H Front Porch = 858-96 = 762

    // reference from tft_auo_a043fl01.c
    drv_l1_disp_dev_rb_switch_set(tft_index, TRUE);
    drv_l1_disp_dev_data_mode_set(tft_index, TFT_DATA_MODE_888);
    drv_l1_disp_dev_dclk_sel_set(tft_index, TFT_DCLK_SEL_90);
    drv_l1_disp_dev_vsync_unit_set(tft_index, TRUE);
    drv_l1_disp_dev_signal_inv_set(tft_index, TFT_VSYNC_INV | TFT_HSYNC_INV, (TFT_ENABLE & TFT_VSYNC_INV) | (TFT_ENABLE & TFT_HSYNC_INV));
    drv_l1_disp_dev_mode_set(tft_index, TFT_MODE_PARALLEL);
    drv_l1_disp_dev_target_clk_set(tft_index, TFT_TAR_CLK);
    drv_l1_disp_dev_en_set(tft_index, TRUE);

    // set Systemclock/6, 162/6 = 27MHz
    R_TFT_CTRL |= (1 << 1);
    R_TFT_CTRL |= (1 << 3);

    tft_sony_ecx336_power_on();
    return STATUS_OK;
}

/**
 * @brief   TFT display parameter configuration structure.
 *
 * @details
 * This structure defines the display resolution, signal configuration,
 * and initialization function for the Sony ECX336 panel.
 *
 * It is used by the display driver to configure and start the panel.
 */
const DispCtrl_t TFT_Param =
    {
        640, // INT16U width;
        400, // INT16U height;
        1,   // INT32U sync_pin_enable;
        1,   // INT32U de_pin_enable;
        0,   // INT32U te_pin_enable;
        tft_sony_ecx336_init};
#endif //(defined SONY_ECX336) && (SONY_ECX336 == 1)
