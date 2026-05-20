#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board_config.h"
#include "drv_l1_clock.h"
#include "drv_l1_csi.h"
#include "drv_l1_sfr.h"
#include "drv_l1_mipi.h"
#include "drv_l1_i2c.h"
#include "drv_l1_front.h"
#include "drv_l2_sensor.h"
#include "drv_l2_sccb.h"
#include "drv_l2_cdsp_config.h"
#include "gplib_print_string.h"
#include "psc700_conf.h"

#if (defined _SENSOR_IMX577_CDSP_MIPI) && (_SENSOR_IMX577_CDSP_MIPI == 1)
#include "drv_l2_user_calibration.h"
#include "drv_l2_user_preference.h"

/**************************************************************************
 *                              M A C R O S                               *
 **************************************************************************/
#define IMX577_A_GAIN_TO_REG(A)    (uint16_t)(1024-(1024/(A)))

/**************************************************************************
 *                           C O N S T A N T S                            *
 **************************************************************************/
#define FIXED_EXP_EN                        1
#define FIXED_WB_EN                         1

#define READ_TABLE_EN                       0
#define MIPI_LANE_NO                        MIPI_2_LANE
#define MIPI_DEV_NO                         MIPI_1 // 0:MIPI_0(IOC8~11) or 1:MIPI_1(IOF6~9)

#define I2C_DEVICE_NUM                      I2C_0
#define FRONT_SENSOR_RESET                  IO_A15

/* Dual Sensor setting */
#define BINMODE_OUT                         0
#define NUM_SENSOR                          SINGLE_SENSOR
#define MIPI_ISR_TEST                       (BINMODE_OUT && (NUM_SENSOR != SINGLE_SENSOR))
#define CDSP_IDX                            ((NUM_SENSOR == REAR_SENSOR) ? 1:0)
#define INTERFACE_FLAG                      (S_MIPI_(MIPI_DEV_NO) | S_CDSP | (BINMODE_OUT*S_BINMODE_(MIPI_DEV_NO)) | S_NUMSENSOR(NUM_SENSOR))

#define IMX577_ID                           0x34

#define IMX577_WIDTH                        640
#define IMX577_HEIGHT                       480

#define IMX577_OUT_WIDTH                    640
#define IMX577_OUT_HEIGHT                   480

#define IMX577_120FPS_50HZ_DAY_EV_IDX       180
#define IMX577_120FPS_50HZ_NIGHT_EV_IDX     220
#define IMX577_120FPS_50HZ_EXP_TIME_TOTAL   261
#define IMX577_120FPS_50HZ_INIT_EV_IDX      ((IMX577_120FPS_50HZ_DAY_EV_IDX + IMX577_120FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define IMX577_120FPS_50HZ_MAX_EV_IDX       (IMX577_120FPS_50HZ_EXP_TIME_TOTAL - 1)

#define IMX577_120FPS_60HZ_DAY_EV_IDX       174
#define IMX577_120FPS_60HZ_NIGHT_EV_IDX     214
#define IMX577_120FPS_60HZ_EXP_TIME_TOTAL   255
#define IMX577_120FPS_60HZ_INIT_EV_IDX      ((IMX577_120FPS_60HZ_DAY_EV_IDX + IMX577_120FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define IMX577_120FPS_60HZ_MAX_EV_IDX       (IMX577_120FPS_60HZ_EXP_TIME_TOTAL - 1)

#define IMX577_25FPS_50HZ_DAY_EV_IDX        171
#define IMX577_25FPS_50HZ_NIGHT_EV_IDX      252
#define IMX577_25FPS_50HZ_EXP_TIME_TOTAL    293
#define IMX577_25FPS_50HZ_INIT_EV_IDX       ((IMX577_25FPS_50HZ_DAY_EV_IDX + IMX577_25FPS_50HZ_NIGHT_EV_IDX) >> 1)
#define IMX577_25FPS_50HZ_MAX_EV_IDX        (IMX577_25FPS_50HZ_EXP_TIME_TOTAL - 1)

#define IMX577_25FPS_60HZ_DAY_EV_IDX        178
#define IMX577_25FPS_60HZ_NIGHT_EV_IDX      259
#define IMX577_25FPS_60HZ_EXP_TIME_TOTAL    300
#define IMX577_25FPS_60HZ_INIT_EV_IDX       ((IMX577_25FPS_60HZ_DAY_EV_IDX + IMX577_25FPS_60HZ_NIGHT_EV_IDX) >> 1)
#define IMX577_25FPS_60HZ_MAX_EV_IDX        (IMX577_25FPS_60HZ_EXP_TIME_TOTAL - 1)

#define SENSOR_I2C_SCL                      IO_B4
#define SENSOR_I2C_SDA                      IO_B5
#define SENSOR_I2C_DRIVING                  IO_DRIVING_lvl0


//IMX577 Register
#define FLASH_STRB_ADJ                      0x0C12
#define FLASH_STRB_START_POINT_H            0x0C14
#define FLASH_STRB_START_POINT_L            0x0C15
#define FLASH_STRB_DLY_RS_CTRL_H            0x0C16
#define FLASH_STRB_DLY_RS_CTRL_L            0x0C17
#define FLASH_STRB_WIDT_H_RS_CTRL_H         0x0C18
#define FLASH_STRB_WIDT_H_RS_CTRL_L         0x0C19
#define FLASH_MD_RS                         0x0C1A
#define FLASH_TRIG_RS                       0x0C1B
#define FLASH_STAT                          0x0C1C

#if FIXED_EXP_EN
#define VGA_120_EXP                         0x0FD8
#define VGA_120_A_GAIN                      0x03C0
#define VGA_120_D_GAIN_GR                   0x0200
#define VGA_120_D_GAIN_R                    0x0200
#define VGA_120_D_GAIN_B                    0x0200
#define VGA_120_D_GAIN_GB                   0x0200

#define VGA_50_EXP                          0x0625
#define VGA_50_A_GAIN                       0x0000
#define VGA_50_D_GAIN_GR                    0x0100
#define VGA_50_D_GAIN_R                     0x0100
#define VGA_50_D_GAIN_B                     0x0100
#define VGA_50_D_GAIN_GB                    0x0100

#define R76_120_EXP                         0x0FD8
#define R76_120_A_GAIN                      IMX577_A_GAIN_TO_REG(16)
#define R76_120_D_GAIN_GR                   0x0200
#define R76_120_D_GAIN_R                    0x0200
#define R76_120_D_GAIN_B                    0x0200
#define R76_120_D_GAIN_GB                   0x0200

#define R76_50_EXP                          0x095D
#define R76_50_A_GAIN                       0x0000
#define R76_50_D_GAIN_GR                    0x0100
#define R76_50_D_GAIN_R                     0x0100
#define R76_50_D_GAIN_B                     0x0100
#define R76_50_D_GAIN_GB                    0x0100

#define FUL_EXP                             0x0FD8
#define FUL_A_GAIN                          0x0200
#define FUL_D_GAIN_GR                       0x0100
#define FUL_D_GAIN_R                        0x0100
#define FUL_D_GAIN_B                        0x0100
#define FUL_D_GAIN_GB                       0x0100
#endif //FIXED_EXP_EN

/**************************************************************************
 *                          D A T A    T Y P E S                          *
 **************************************************************************/
typedef struct regval16_s
{
    INT16U reg_num;
    INT8U value;
} regval16_t;

/**************************************************************************
 *                         G L O B A L    D A T A                         *
 **************************************************************************/
extern void gp_isp_reset_iq_tuning_target(void);
static int *p_expTime_table;
static sensor_exposure_t seInfo;
static int pre_sensor_a_gain, pre_sensor_time;
static INT8U info_index;

static INT32U GainThrTable[6] =
{
    1.90*64, 4.00*64, 6.00*64, 8.00*64, 12.00*64, 16.00*64
};

#if SCCB_MODE == SCCB_GPIO
static void *imx577_handle = NULL;

sccb_config_t imx577_cdsp_mipi_sccb_config =
{
    SENSOR_I2C_SCL,             //scl_port
    SENSOR_I2C_DRIVING,         //scl_drv
    SENSOR_I2C_SDA,             //sda_port
    SENSOR_I2C_DRIVING,         //sda_drv
    IO_F15,                     //pwdn_port
    0,                          //pwdn_drv
    0,                          //have_pwdn
    16,                         //RegBits
    8,                          //DataBits
    IMX577_ID,                  //slaveAddr
    0x20000,                    //timeout
    200                         //clock_rate
};
#elif SCCB_MODE == SCCB_HW_I2C
static drv_l1_i2c_bus_handle_t imx577_handle;
#endif

static mipi_config_t imx577_mipi_cfg =
{
    DISABLE,            /* low power, 0:disable, 1:enable */
    D_PHY_SAMPLE_POS,   /* byte clock edge, 0:posedge, 1:negedge */
#if MIPI_LANE_NO == 1
    MIPI_1_LANE,        /* lane */
    D_PHY_BYTE_CLK,     /* byte clock source */
#elif MIPI_LANE_NO == 2
    MIPI_2_LANE,        /* lane */
    D_PHY_BYTE_CLK,     /* byte clock source */
#elif MIPI_LANE_NO == 4
    MIPI_4_LANE,        /* lane */
    D_PHY_BYTE_CLK,     /* byte clock source */
#endif

    MIPI_AUTO_DETECT,   /* data mode, 0:auto detect, 1:user define */
    MIPI_RAW10,         /* data type, valid when data mode is 1*/
    MIPI_DATA_TO_CDSP,  /* data type, 1:data[7:0]+2':00 to cdsp, 0: 2'00+data[7:0] to csi */
    0,                  /* RSD 2 */

    IMX577_WIDTH,       /* width, 0~0xFFFF */
    IMX577_HEIGHT,      /* height, 0~0xFFFF */
    4,                  /* back porch, 0~0xF */
    4,                  /* front porch, 0~0xF */
    ENABLE,             /* blanking_line_en, 0:disable, 1:enable */
    0,                  /* RSD 3 */

    ENABLE,             /* ecc, 0:disable, 1:enable */
    MIPI_ECC_ORDER3,    /* ecc order */

    150,                /* data mask, unit is ns */
    MIPI_CHECK_HS_SEQ   /* check hs sequence or LP00 for clock lane */
};

#if 0 //no use
static const regval16_t imx577_reset_table[] =
{
    {0x3003, 0x01},
    {0x3003, 0x01},
    {0xFFFF, 0xFF}
};
#endif

// 640x480(V2Binning+2SubSampling_H4binning) RAW10 120fps 600Mbps 2lanes
static const regval16_t imx577_init_table_vga_2lane_120fps[] =
{
    // sensor standby
    {0x0100, 0x00},

    // External Clock Setting
    {0x0136, 0x18},
    {0x0137, 0x00},
    // Register version
    {0x3C7E, 0x01},
    {0x3C7F, 0x02},
    // Global Setting
    {0x38A8, 0x1F},
    {0x38A9, 0xFF},
    {0x38AA, 0x1F},
    {0x38AB, 0xFF},
    {0x55D4, 0x00},
    {0x55D5, 0x00},
    {0x55D6, 0x07},
    {0x55D7, 0xFF},
    {0x55E8, 0x07},
    {0x55E9, 0xFF},
    {0x55EA, 0x00},
    {0x55EB, 0x00},
    {0x575C, 0x07},
    {0x575D, 0xFF},
    {0x575E, 0x00},
    {0x575F, 0x00},
    {0x5764, 0x00},
    {0x5765, 0x00},
    {0x5766, 0x07},
    {0x5767, 0xFF},
    {0x5974, 0x04},
    {0x5975, 0x01},
    {0x5F10, 0x09},
    {0x5F11, 0x92},
    {0x5F12, 0x32},
    {0x5F13, 0x72},
    {0x5F14, 0x16},
    {0x5F15, 0xBA},
    {0x5F17, 0x13},
    {0x5F18, 0x24},
    {0x5F19, 0x60},
    {0x5F1A, 0xE3},
    {0x5F1B, 0xAD},
    {0x5F1C, 0x74},
    {0x5F2D, 0x25},
    {0x5F5C, 0xD0},
    {0x6A22, 0x00},
    {0x6A23, 0x1D},
    {0x7BA8, 0x00},
    {0x7BA9, 0x00},
    {0x886B, 0x00},
    {0x9002, 0x0A},
    {0x9004, 0x1A},
    {0x9214, 0x93},
    {0x9215, 0x69},
    {0x9216, 0x93},
    {0x9217, 0x6B},
    {0x9218, 0x93},
    {0x9219, 0x6D},
    {0x921A, 0x57},
    {0x921B, 0x58},
    {0x921C, 0x57},
    {0x921D, 0x59},
    {0x921E, 0x57},
    {0x921F, 0x5A},
    {0x9220, 0x57},
    {0x9221, 0x5B},
    {0x9222, 0x93},
    {0x9223, 0x02},
    {0x9224, 0x93},
    {0x9225, 0x03},
    {0x9226, 0x93},
    {0x9227, 0x04},
    {0x9228, 0x93},
    {0x9229, 0x05},
    {0x922A, 0x98},
    {0x922B, 0x21},
    {0x922C, 0xB2},
    {0x922D, 0xDB},
    {0x922E, 0xB2},
    {0x922F, 0xDC},
    {0x9230, 0xB2},
    {0x9231, 0xDD},
    {0x9232, 0xB2},
    {0x9233, 0xE1},
    {0x9234, 0xB2},
    {0x9235, 0xE2},
    {0x9236, 0xB2},
    {0x9237, 0xE3},
    {0x9238, 0xB7},
    {0x9239, 0xB9},
    {0x923A, 0xB7},
    {0x923B, 0xBB},
    {0x923C, 0xB7},
    {0x923D, 0xBC},
    {0x923E, 0xB7},
    {0x923F, 0xC5},
    {0x9240, 0xB7},
    {0x9241, 0xC7},
    {0x9242, 0xB7},
    {0x9243, 0xC9},
    {0x9244, 0x98},
    {0x9245, 0x56},
    {0x9246, 0x98},
    {0x9247, 0x55},
    {0x9380, 0x00},
    {0x9381, 0x62},
    {0x9382, 0x00},
    {0x9383, 0x56},
    {0x9384, 0x00},
    {0x9385, 0x52},
    {0x9388, 0x00},
    {0x9389, 0x55},
    {0x938A, 0x00},
    {0x938B, 0x55},
    {0x938C, 0x00},
    {0x938D, 0x41},
    {0x5078, 0x01},

    // MIPI setting
    {0x0112, 0x0A},
    {0x0113, 0x0A},
    {0x0114, 0x01},

    // Frame Horizontal Clock Count
    {0x0342, 0x11},
    {0x0343, 0xA0},

    // Frame Vertical Clock Count
    {0x0340, 0x05},
    {0x0341, 0x31},
    {0x3210, 0x00},

    // Visible Size
    {0x0344, 0x00},
    {0x0345, 0x00},
    {0x0346, 0x00},
    {0x0347, 0x00},
    {0x0348, 0x0F},
    {0x0349, 0xD7},
    {0x034A, 0x0B},
    {0x034B, 0xDF},

    // Mode Setting
    {0x00E3, 0x00},
    {0x00E4, 0x00},
    {0x00E5, 0x01},
    {0x00FC, 0x0A},
    {0x00FD, 0x0A},
    {0x00FE, 0x0A},
    {0x00FF, 0x0A},
    {0xE013, 0x00},
    {0x0220, 0x00},
    {0x0221, 0x11},
    {0x0381, 0x01},
    {0x0383, 0x01},
    {0x0385, 0x01},
    {0x0387, 0x03},
    {0x0900, 0x01},
    {0x0901, 0x42},
    {0x0902, 0x02},
    {0x3140, 0x02},
    {0x3241, 0x11},
    {0x3250, 0x03},
    {0x3E10, 0x00},
    {0x3E11, 0x00},
    {0x3F0D, 0x00},
    {0x3F42, 0x00},
    {0x3F43, 0x00},

    // Digital Crop & Scaling
    {0x0401, 0x00},
    {0x0404, 0x00},
    {0x0405, 0x10},
    {0x0408, 0x00},
    {0x0409, 0xBA},
    {0x040A, 0x00},
    {0x040B, 0x8C},
    {0x040C, 0x02},
    {0x040D, 0x80},
    {0x040E, 0x01},
    {0x040F, 0xE0},

    // Output Crop
    {0x034C, 0x02},
    {0x034D, 0x80},
    {0x034E, 0x01},
    {0x034F, 0xE0},

    // Clock Setting
    {0x0301, 0x05},
    {0x0303, 0x02},
    {0x0305, 0x02},
    {0x0306, 0x00},
    {0x0307, 0x96},
    {0x0309, 0x0A},
    {0x030B, 0x02},
    {0x030D, 0x04},
    {0x030E, 0x00},
    {0x030F, 0xC8},
    {0x0310, 0x01},
    {0x0820, 0x04},
    {0x0821, 0xB0},
    {0x0822, 0x00},
    {0x0823, 0x00},

    // Output Data Select Setting
    {0x3E20, 0x01},
    {0x3E37, 0x00},

    // PowerSave Setting
    {0x3F50, 0x00},
    {0x3F56, 0x00},
    {0x3F57, 0x97},

    // Other Setting
    {0x3C0A, 0x73},
    {0x3C0B, 0x64},
    {0x3C0C, 0x5F},
    {0x3C0D, 0x00},
    {0x3C0E, 0x00},
    {0x3C0F, 0x00},
    {0x3C10, 0xA4},
    {0x3C11, 0x02},
    {0x3C12, 0x00},
    {0x3C13, 0x03},
    {0x3C14, 0x80},
    {0x3C15, 0x04},
    {0x3C16, 0x15},
    {0x3C17, 0x15},
    {0x3C18, 0x15},
    {0x3C19, 0x15},
    {0x3C1A, 0x15},
    {0x3C1B, 0x15},
    {0x3C1C, 0x06},
    {0x3C1D, 0x06},
    {0x3C1E, 0x06},
    {0x3C1F, 0x06},
    {0x3C20, 0x06},
    {0x3C21, 0x06},
    {0x3C22, 0x3F},
    {0x3C23, 0x0A},
    {0x3E35, 0x01},
    {0x3F4A, 0x01},
    {0x3F4B, 0x7F},
    {0x3F26, 0x00},

#if FIXED_EXP_EN
    // Integration Time Setting
    {0x0202, (VGA_120_EXP >> 8) & 0xFF},        //Exposure [15:8]
    {0x0203, (VGA_120_EXP >> 0) & 0xFF},        //Exposure [7:0]

    // Gain Setting
    {0x0204, (VGA_120_A_GAIN >> 8) & 0x03},     //Analog Gain[9:8]
    {0x0205, (VGA_120_A_GAIN >> 0) & 0xFF},     //Analog Gain[7:0]
    {0x020E, (VGA_120_D_GAIN_GR >> 8) & 0xFF},  //Digital Gain GR[15:8]
    {0x020F, (VGA_120_D_GAIN_GR >> 0) & 0xFF},  //Digital Gain GR[7:0]
    {0x0210, (VGA_120_D_GAIN_R >> 8) & 0xFF},   //Digital Gain R[15:8]
    {0x0211, (VGA_120_D_GAIN_R >> 0) & 0xFF},   //Digital Gain R[7:0]
    {0x0212, (VGA_120_D_GAIN_B >> 8) & 0xFF},   //Digital Gain B[15:8]
    {0x0213, (VGA_120_D_GAIN_B >> 0) & 0xFF},   //Digital Gain B[7:0]
    {0x0214, (VGA_120_D_GAIN_GB >> 8) & 0xFF},  //Digital Gain GB[15:8]
    {0x0215, (VGA_120_D_GAIN_GB >> 0) & 0xFF},  //Digital Gain GB[7:0]
#else
    // Integration Time Setting
    {0x0202, 0x05},
    {0x0203, 0x31},
    // 1329, max exp time in imx577_120fps_exp_time_gain_60Hz

    // Gain Setting
    {0x0204, 0x03},
    {0x0205, 0xC0},
    // 960 = 16x, max gain in imx577_120fps_exp_time_gain_60Hz

    {0x020E, 0x01},
    {0x020F, 0x00},
    {0x0210, 0x01},
    {0x0211, 0x00},
    {0x0212, 0x01},
    {0x0213, 0x00},
    {0x0214, 0x01},
    {0x0215, 0x00},
#endif

    // stanby cancel
    {0x0100, 0x01},
    {0xFFFF, 0xFF},
};

// 640x480(V2Binning+2SubSampling_H4binning) RAW10 50fps 600Mbps 2lanes
static const regval16_t imx577_init_table_vga_2lane_50fps[] =
{
    // sensor standby
    {0x0100, 0x00},

    // External Clock Setting
    {0x0136, 0x18},
    {0x0137, 0x00},
    // Register version
    {0x3C7E, 0x01},
    {0x3C7F, 0x02},
    // Global Setting
    {0x38A8, 0x1F},
    {0x38A9, 0xFF},
    {0x38AA, 0x1F},
    {0x38AB, 0xFF},
    {0x55D4, 0x00},
    {0x55D5, 0x00},
    {0x55D6, 0x07},
    {0x55D7, 0xFF},
    {0x55E8, 0x07},
    {0x55E9, 0xFF},
    {0x55EA, 0x00},
    {0x55EB, 0x00},
    {0x575C, 0x07},
    {0x575D, 0xFF},
    {0x575E, 0x00},
    {0x575F, 0x00},
    {0x5764, 0x00},
    {0x5765, 0x00},
    {0x5766, 0x07},
    {0x5767, 0xFF},
    {0x5974, 0x04},
    {0x5975, 0x01},
    {0x5F10, 0x09},
    {0x5F11, 0x92},
    {0x5F12, 0x32},
    {0x5F13, 0x72},
    {0x5F14, 0x16},
    {0x5F15, 0xBA},
    {0x5F17, 0x13},
    {0x5F18, 0x24},
    {0x5F19, 0x60},
    {0x5F1A, 0xE3},
    {0x5F1B, 0xAD},
    {0x5F1C, 0x74},
    {0x5F2D, 0x25},
    {0x5F5C, 0xD0},
    {0x6A22, 0x00},
    {0x6A23, 0x1D},
    {0x7BA8, 0x00},
    {0x7BA9, 0x00},
    {0x886B, 0x00},
    {0x9002, 0x0A},
    {0x9004, 0x1A},
    {0x9214, 0x93},
    {0x9215, 0x69},
    {0x9216, 0x93},
    {0x9217, 0x6B},
    {0x9218, 0x93},
    {0x9219, 0x6D},
    {0x921A, 0x57},
    {0x921B, 0x58},
    {0x921C, 0x57},
    {0x921D, 0x59},
    {0x921E, 0x57},
    {0x921F, 0x5A},
    {0x9220, 0x57},
    {0x9221, 0x5B},
    {0x9222, 0x93},
    {0x9223, 0x02},
    {0x9224, 0x93},
    {0x9225, 0x03},
    {0x9226, 0x93},
    {0x9227, 0x04},
    {0x9228, 0x93},
    {0x9229, 0x05},
    {0x922A, 0x98},
    {0x922B, 0x21},
    {0x922C, 0xB2},
    {0x922D, 0xDB},
    {0x922E, 0xB2},
    {0x922F, 0xDC},
    {0x9230, 0xB2},
    {0x9231, 0xDD},
    {0x9232, 0xB2},
    {0x9233, 0xE1},
    {0x9234, 0xB2},
    {0x9235, 0xE2},
    {0x9236, 0xB2},
    {0x9237, 0xE3},
    {0x9238, 0xB7},
    {0x9239, 0xB9},
    {0x923A, 0xB7},
    {0x923B, 0xBB},
    {0x923C, 0xB7},
    {0x923D, 0xBC},
    {0x923E, 0xB7},
    {0x923F, 0xC5},
    {0x9240, 0xB7},
    {0x9241, 0xC7},
    {0x9242, 0xB7},
    {0x9243, 0xC9},
    {0x9244, 0x98},
    {0x9245, 0x56},
    {0x9246, 0x98},
    {0x9247, 0x55},
    {0x9380, 0x00},
    {0x9381, 0x62},
    {0x9382, 0x00},
    {0x9383, 0x56},
    {0x9384, 0x00},
    {0x9385, 0x52},
    {0x9388, 0x00},
    {0x9389, 0x55},
    {0x938A, 0x00},
    {0x938B, 0x55},
    {0x938C, 0x00},
    {0x938D, 0x41},
    {0x5078, 0x01},

    // MIPI setting
    {0x0112, 0x0A},
    {0x0113, 0x0A},
    {0x0114, 0x01},

    // Frame Horizontal Clock Count
    {0x0342, 0x11},
    {0x0343, 0xA0},

    // Frame Vertical Clock Count
    {0x0340, 0x06},
    {0x0341, 0x3B},
    {0x3210, 0x00},

    // Visible Size
    {0x0344, 0x00},
    {0x0345, 0x00},
    {0x0346, 0x00},
    {0x0347, 0x00},
    {0x0348, 0x0F},
    {0x0349, 0xD7},
    {0x034A, 0x0B},
    {0x034B, 0xDF},

    // Mode Setting
    {0x00E3, 0x00},
    {0x00E4, 0x00},
    {0x00E5, 0x01},
    {0x00FC, 0x0A},
    {0x00FD, 0x0A},
    {0x00FE, 0x0A},
    {0x00FF, 0x0A},
    {0xE013, 0x00},
    {0x0220, 0x00},
    {0x0221, 0x11},
    {0x0381, 0x01},
    {0x0383, 0x01},
    {0x0385, 0x01},
    {0x0387, 0x03},
    {0x0900, 0x01},
    {0x0901, 0x42},
    {0x0902, 0x02},
    {0x3140, 0x02},
    {0x3241, 0x11},
    {0x3250, 0x03},
    {0x3E10, 0x00},
    {0x3E11, 0x00},
    {0x3F0D, 0x00},
    {0x3F42, 0x00},
    {0x3F43, 0x00},

    // Digital Crop & Scaling
    {0x0401, 0x00},
    {0x0404, 0x00},
    {0x0405, 0x10},
    {0x0408, 0x00},
    {0x0409, 0xBA},
    {0x040A, 0x00},
    {0x040B, 0x8C},
    {0x040C, 0x02},
    {0x040D, 0x80},
    {0x040E, 0x01},
    {0x040F, 0xE0},

    // Output Crop
    {0x034C, 0x02},
    {0x034D, 0x80},
    {0x034E, 0x01},
    {0x034F, 0xE0},

    // Clock Setting
    {0x0301, 0x05},
    {0x0303, 0x04},
    {0x0305, 0x04},
    {0x0306, 0x01},
    {0x0307, 0x2C},
    {0x0309, 0x0A},
    {0x030B, 0x02},
    {0x030D, 0x02},
    {0x030E, 0x00},
    {0x030F, 0x64},
    {0x0310, 0x01},
    {0x0820, 0x04},
    {0x0821, 0xB0},
    {0x0822, 0x00},
    {0x0823, 0x00},

    // Output Data Select Setting
    {0x3E20, 0x01},
    {0x3E37, 0x00},

    // PowerSave Setting
    {0x3F50, 0x00},
    {0x3F56, 0x01},
    {0x3F57, 0x2E},

    // Other Setting
    {0x3C0A, 0x73},
    {0x3C0B, 0x64},
    {0x3C0C, 0x5F},
    {0x3C0D, 0x00},
    {0x3C0E, 0x00},
    {0x3C0F, 0x00},
    {0x3C10, 0xA4},
    {0x3C11, 0x02},
    {0x3C12, 0x00},
    {0x3C13, 0x03},
    {0x3C14, 0x80},
    {0x3C15, 0x04},
    {0x3C16, 0x15},
    {0x3C17, 0x15},
    {0x3C18, 0x15},
    {0x3C19, 0x15},
    {0x3C1A, 0x15},
    {0x3C1B, 0x15},
    {0x3C1C, 0x06},
    {0x3C1D, 0x06},
    {0x3C1E, 0x06},
    {0x3C1F, 0x06},
    {0x3C20, 0x06},
    {0x3C21, 0x06},
    {0x3C22, 0x3F},
    {0x3C23, 0x0A},
    {0x3E35, 0x01},
    {0x3F4A, 0x01},
    {0x3F4B, 0x7F},
    {0x3F26, 0x00},

#if FIXED_EXP_EN
    // Integration Time Setting
    {0x0202, (VGA_50_EXP >> 8) & 0xFF},         //Exposure [15:8]
    {0x0203, (VGA_50_EXP >> 0) & 0xFF},         //Exposure [7:0]

    // Gain Setting
    {0x0204, (VGA_50_A_GAIN >> 8) & 0x03},      //Analog Gain[9:8]
    {0x0205, (VGA_50_A_GAIN >> 0) & 0xFF},      //Analog Gain[7:0]
    {0x020E, (VGA_50_D_GAIN_GR >> 8) & 0xFF},   //Digital Gain GR[15:8]
    {0x020F, (VGA_50_D_GAIN_GR >> 0) & 0xFF},   //Digital Gain GR[7:0]
    {0x0210, (VGA_50_D_GAIN_R >> 8) & 0xFF},    //Digital Gain R[15:8]
    {0x0211, (VGA_50_D_GAIN_R >> 0) & 0xFF},    //Digital Gain R[7:0]
    {0x0212, (VGA_50_D_GAIN_B >> 8) & 0xFF},    //Digital Gain B[15:8]
    {0x0213, (VGA_50_D_GAIN_B >> 0) & 0xFF},    //Digital Gain B[7:0]
    {0x0214, (VGA_50_D_GAIN_GB >> 8) & 0xFF},   //Digital Gain GB[15:8]
    {0x0215, (VGA_50_D_GAIN_GB >> 0) & 0xFF},   //Digital Gain GB[7:0]
#else
    // Integration Time Setting
    {0x0202, 0x05},
    {0x0203, 0x1B},

    // Gain Setting
    {0x0204, 0x00},
    {0x0205, 0x00},
    {0x020E, 0x01},
    {0x020F, 0x00},
    {0x0210, 0x01},
    {0x0211, 0x00},
    {0x0212, 0x01},
    {0x0213, 0x00},
    {0x0214, 0x01},
    {0x0215, 0x00},
#endif

    // stanby cancel
    {0x0100, 0x01},
    {0xFFFF, 0xFF},
};

// 1012x760(V2Binning+2SubSampling_H4binning) RAW10 120fps 340Mbps 4lanes
static const regval16_t imx577_init_table_760_4lane_120fps[] =
{
    // sensor standby
    {0x0100, 0x00},

    // External Clock Setting
    {0x0136, 0x18},
    {0x0137, 0x00},
    // Register version
    {0x3C7E, 0x01},
    {0x3C7F, 0x02},
    // Global Setting
    {0x38A8, 0x1F},
    {0x38A9, 0xFF},
    {0x38AA, 0x1F},
    {0x38AB, 0xFF},
    {0x55D4, 0x00},
    {0x55D5, 0x00},
    {0x55D6, 0x07},
    {0x55D7, 0xFF},
    {0x55E8, 0x07},
    {0x55E9, 0xFF},
    {0x55EA, 0x00},
    {0x55EB, 0x00},
    {0x575C, 0x07},
    {0x575D, 0xFF},
    {0x575E, 0x00},
    {0x575F, 0x00},
    {0x5764, 0x00},
    {0x5765, 0x00},
    {0x5766, 0x07},
    {0x5767, 0xFF},
    {0x5974, 0x04},
    {0x5975, 0x01},
    {0x5F10, 0x09},
    {0x5F11, 0x92},
    {0x5F12, 0x32},
    {0x5F13, 0x72},
    {0x5F14, 0x16},
    {0x5F15, 0xBA},
    {0x5F17, 0x13},
    {0x5F18, 0x24},
    {0x5F19, 0x60},
    {0x5F1A, 0xE3},
    {0x5F1B, 0xAD},
    {0x5F1C, 0x74},
    {0x5F2D, 0x25},
    {0x5F5C, 0xD0},
    {0x6A22, 0x00},
    {0x6A23, 0x1D},
    {0x7BA8, 0x00},
    {0x7BA9, 0x00},
    {0x886B, 0x00},
    {0x9002, 0x0A},
    {0x9004, 0x1A},
    {0x9214, 0x93},
    {0x9215, 0x69},
    {0x9216, 0x93},
    {0x9217, 0x6B},
    {0x9218, 0x93},
    {0x9219, 0x6D},
    {0x921A, 0x57},
    {0x921B, 0x58},
    {0x921C, 0x57},
    {0x921D, 0x59},
    {0x921E, 0x57},
    {0x921F, 0x5A},
    {0x9220, 0x57},
    {0x9221, 0x5B},
    {0x9222, 0x93},
    {0x9223, 0x02},
    {0x9224, 0x93},
    {0x9225, 0x03},
    {0x9226, 0x93},
    {0x9227, 0x04},
    {0x9228, 0x93},
    {0x9229, 0x05},
    {0x922A, 0x98},
    {0x922B, 0x21},
    {0x922C, 0xB2},
    {0x922D, 0xDB},
    {0x922E, 0xB2},
    {0x922F, 0xDC},
    {0x9230, 0xB2},
    {0x9231, 0xDD},
    {0x9232, 0xB2},
    {0x9233, 0xE1},
    {0x9234, 0xB2},
    {0x9235, 0xE2},
    {0x9236, 0xB2},
    {0x9237, 0xE3},
    {0x9238, 0xB7},
    {0x9239, 0xB9},
    {0x923A, 0xB7},
    {0x923B, 0xBB},
    {0x923C, 0xB7},
    {0x923D, 0xBC},
    {0x923E, 0xB7},
    {0x923F, 0xC5},
    {0x9240, 0xB7},
    {0x9241, 0xC7},
    {0x9242, 0xB7},
    {0x9243, 0xC9},
    {0x9244, 0x98},
    {0x9245, 0x56},
    {0x9246, 0x98},
    {0x9247, 0x55},
    {0x9380, 0x00},
    {0x9381, 0x62},
    {0x9382, 0x00},
    {0x9383, 0x56},
    {0x9384, 0x00},
    {0x9385, 0x52},
    {0x9388, 0x00},
    {0x9389, 0x55},
    {0x938A, 0x00},
    {0x938B, 0x55},
    {0x938C, 0x00},
    {0x938D, 0x41},
    {0x5078, 0x01},

    // MIPI setting
    {0x0112, 0x0A},
    {0x0113, 0x0A},
    {0x0114, 0x03},

    // Frame Horizontal Clock Count
    {0x0342, 0x0B},
    {0x0343, 0xA0},

    // Frame Vertical Clock Count
    {0x0340, 0x03},
    {0x0341, 0xF0},
    {0x3210, 0x00},

    // Visible Size
    {0x0344, 0x00},
    {0x0345, 0x00},
    {0x0346, 0x00},
    {0x0347, 0x00},
    {0x0348, 0x0F},
    {0x0349, 0xD7},
    {0x034A, 0x0B},
    {0x034B, 0xDF},

    // Mode Setting
    {0x00E3, 0x00},
    {0x00E4, 0x00},
    {0x00E5, 0x01},
    {0x00FC, 0x0A},
    {0x00FD, 0x0A},
    {0x00FE, 0x0A},
    {0x00FF, 0x0A},
    {0xE013, 0x00},
    {0x0220, 0x00},
    {0x0221, 0x11},
    {0x0381, 0x01},
    {0x0383, 0x01},
    {0x0385, 0x01},
    {0x0387, 0x03},
    {0x0900, 0x01},
    {0x0901, 0x42},
    {0x0902, 0x02},
    {0x3140, 0x02},
    {0x3241, 0x11},
    {0x3250, 0x03},
    {0x3E10, 0x00},
    {0x3E11, 0x00},
    {0x3F0D, 0x00},
    {0x3F42, 0x00},
    {0x3F43, 0x00},

    // Digital Crop & Scaling
    {0x0401, 0x00},
    {0x0404, 0x00},
    {0x0405, 0x10},
    {0x0408, 0x00},
    {0x0409, 0x00},
    {0x040A, 0x00},
    {0x040B, 0x00},
    {0x040C, 0x03},
    {0x040D, 0xF4},
    {0x040E, 0x02},
    {0x040F, 0xF8},

    // Output Crop
    {0x034C, 0x03},
    {0x034D, 0xF4},
    {0x034E, 0x02},
    {0x034F, 0xF8},

    // Clock Setting
    {0x0301, 0x05},
    {0x0303, 0x04},
    {0x0305, 0x04},
    {0x0306, 0x01},
    {0x0307, 0x2C},
    {0x0309, 0x0A},
    {0x030B, 0x04},
    {0x030D, 0x02},
    {0x030E, 0x00},
    {0x030F, 0x71},
    {0x0310, 0x01},
    {0x0820, 0x05},
    {0x0821, 0x4C},
    {0x0822, 0x00},
    {0x0823, 0x00},

    // Output Data Select Setting
    {0x3E20, 0x01},
    {0x3E37, 0x00},

    // PowerSave Setting
    {0x3F50, 0x00},
    {0x3F56, 0x00},
    {0x3F57, 0xC7},

    // Other Setting
    {0x3C0A, 0x73},
    {0x3C0B, 0x64},
    {0x3C0C, 0x5F},
    {0x3C0D, 0x00},
    {0x3C0E, 0x00},
    {0x3C0F, 0x00},
    {0x3C10, 0xA4},
    {0x3C11, 0x02},
    {0x3C12, 0x00},
    {0x3C13, 0x03},
    {0x3C14, 0x80},
    {0x3C15, 0x04},
    {0x3C16, 0x15},
    {0x3C17, 0x15},
    {0x3C18, 0x15},
    {0x3C19, 0x15},
    {0x3C1A, 0x15},
    {0x3C1B, 0x15},
    {0x3C1C, 0x06},
    {0x3C1D, 0x06},
    {0x3C1E, 0x06},
    {0x3C1F, 0x06},
    {0x3C20, 0x06},
    {0x3C21, 0x06},
    {0x3C22, 0x3F},
    {0x3C23, 0x0A},
    {0x3E35, 0x01},
    {0x3F4A, 0x01},
    {0x3F4B, 0x7F},
    {0x3F26, 0x00},

#if FIXED_EXP_EN
    // Integration Time Setting
    {0x0202, (R76_120_EXP >> 8) & 0xFF},        //Exposure [15:8]
    {0x0203, (R76_120_EXP >> 0) & 0xFF},        //Exposure [7:0]

    // Gain Setting
    {0x0204, (R76_120_A_GAIN >> 8) & 0x03},     //Analog Gain[9:8]
    {0x0205, (R76_120_A_GAIN >> 0) & 0xFF},     //Analog Gain[7:0]
    {0x020E, (R76_120_D_GAIN_GR >> 8) & 0xFF},  //Digital Gain GR[15:8]
    {0x020F, (R76_120_D_GAIN_GR >> 0) & 0xFF},  //Digital Gain GR[7:0]
    {0x0210, (R76_120_D_GAIN_R >> 8) & 0xFF},   //Digital Gain R[15:8]
    {0x0211, (R76_120_D_GAIN_R >> 0) & 0xFF},   //Digital Gain R[7:0]
    {0x0212, (R76_120_D_GAIN_B >> 8) & 0xFF},   //Digital Gain B[15:8]
    {0x0213, (R76_120_D_GAIN_B >> 0) & 0xFF},   //Digital Gain B[7:0]
    {0x0214, (R76_120_D_GAIN_GB >> 8) & 0xFF},  //Digital Gain GB[15:8]
    {0x0215, (R76_120_D_GAIN_GB >> 0) & 0xFF},  //Digital Gain GB[7:0]
#else
    // Integration Time Setting
    {0x0202, 0x03},
    {0x0203, 0xDA},

    // Gain Setting
    {0x0204, 0x00},
    {0x0205, 0x00},
    {0x020E, 0x01},
    {0x020F, 0x00},
    {0x0210, 0x01},
    {0x0211, 0x00},
    {0x0212, 0x01},
    {0x0213, 0x00},
    {0x0214, 0x01},
    {0x0215, 0x00},
#endif

    // stanby cancel
    {0x0100, 0x01},
    {0xFFFF, 0xFF},
};

// 1012x760(V2Binning+2SubSampling_H4binning) RAW10 50fps 340Mbps 4lanes
static const regval16_t imx577_init_table_760_4lane_50fps[] =
{
    // sensor standby
    {0x0100, 0x00},

    // External Clock Setting
    {0x0136, 0x18},
    {0x0137, 0x00},
    // Register version
    {0x3C7E, 0x01},
    {0x3C7F, 0x02},
    // Global Setting
    {0x38A8, 0x1F},
    {0x38A9, 0xFF},
    {0x38AA, 0x1F},
    {0x38AB, 0xFF},
    {0x55D4, 0x00},
    {0x55D5, 0x00},
    {0x55D6, 0x07},
    {0x55D7, 0xFF},
    {0x55E8, 0x07},
    {0x55E9, 0xFF},
    {0x55EA, 0x00},
    {0x55EB, 0x00},
    {0x575C, 0x07},
    {0x575D, 0xFF},
    {0x575E, 0x00},
    {0x575F, 0x00},
    {0x5764, 0x00},
    {0x5765, 0x00},
    {0x5766, 0x07},
    {0x5767, 0xFF},
    {0x5974, 0x04},
    {0x5975, 0x01},
    {0x5F10, 0x09},
    {0x5F11, 0x92},
    {0x5F12, 0x32},
    {0x5F13, 0x72},
    {0x5F14, 0x16},
    {0x5F15, 0xBA},
    {0x5F17, 0x13},
    {0x5F18, 0x24},
    {0x5F19, 0x60},
    {0x5F1A, 0xE3},
    {0x5F1B, 0xAD},
    {0x5F1C, 0x74},
    {0x5F2D, 0x25},
    {0x5F5C, 0xD0},
    {0x6A22, 0x00},
    {0x6A23, 0x1D},
    {0x7BA8, 0x00},
    {0x7BA9, 0x00},
    {0x886B, 0x00},
    {0x9002, 0x0A},
    {0x9004, 0x1A},
    {0x9214, 0x93},
    {0x9215, 0x69},
    {0x9216, 0x93},
    {0x9217, 0x6B},
    {0x9218, 0x93},
    {0x9219, 0x6D},
    {0x921A, 0x57},
    {0x921B, 0x58},
    {0x921C, 0x57},
    {0x921D, 0x59},
    {0x921E, 0x57},
    {0x921F, 0x5A},
    {0x9220, 0x57},
    {0x9221, 0x5B},
    {0x9222, 0x93},
    {0x9223, 0x02},
    {0x9224, 0x93},
    {0x9225, 0x03},
    {0x9226, 0x93},
    {0x9227, 0x04},
    {0x9228, 0x93},
    {0x9229, 0x05},
    {0x922A, 0x98},
    {0x922B, 0x21},
    {0x922C, 0xB2},
    {0x922D, 0xDB},
    {0x922E, 0xB2},
    {0x922F, 0xDC},
    {0x9230, 0xB2},
    {0x9231, 0xDD},
    {0x9232, 0xB2},
    {0x9233, 0xE1},
    {0x9234, 0xB2},
    {0x9235, 0xE2},
    {0x9236, 0xB2},
    {0x9237, 0xE3},
    {0x9238, 0xB7},
    {0x9239, 0xB9},
    {0x923A, 0xB7},
    {0x923B, 0xBB},
    {0x923C, 0xB7},
    {0x923D, 0xBC},
    {0x923E, 0xB7},
    {0x923F, 0xC5},
    {0x9240, 0xB7},
    {0x9241, 0xC7},
    {0x9242, 0xB7},
    {0x9243, 0xC9},
    {0x9244, 0x98},
    {0x9245, 0x56},
    {0x9246, 0x98},
    {0x9247, 0x55},
    {0x9380, 0x00},
    {0x9381, 0x62},
    {0x9382, 0x00},
    {0x9383, 0x56},
    {0x9384, 0x00},
    {0x9385, 0x52},
    {0x9388, 0x00},
    {0x9389, 0x55},
    {0x938A, 0x00},
    {0x938B, 0x55},
    {0x938C, 0x00},
    {0x938D, 0x41},
    {0x5078, 0x01},

    // MIPI setting
    {0x0112, 0x0A},
    {0x0113, 0x0A},
    {0x0114, 0x03},

    // Frame Horizontal Clock Count
    {0x0342, 0x0B},
    {0x0343, 0xA0},

    // Frame Vertical Clock Count
    {0x0340, 0x09},
    {0x0341, 0x73},
    {0x3210, 0x00},

    // Visible Size
    {0x0344, 0x00},
    {0x0345, 0x00},
    {0x0346, 0x00},
    {0x0347, 0x00},
    {0x0348, 0x0F},
    {0x0349, 0xD7},
    {0x034A, 0x0B},
    {0x034B, 0xDF},

    // Mode Setting
    {0x00E3, 0x00},
    {0x00E4, 0x00},
    {0x00E5, 0x01},
    {0x00FC, 0x0A},
    {0x00FD, 0x0A},
    {0x00FE, 0x0A},
    {0x00FF, 0x0A},
    {0xE013, 0x00},
    {0x0220, 0x00},
    {0x0221, 0x11},
    {0x0381, 0x01},
    {0x0383, 0x01},
    {0x0385, 0x01},
    {0x0387, 0x03},
    {0x0900, 0x01},
    {0x0901, 0x42},
    {0x0902, 0x02},
    {0x3140, 0x02},
    {0x3241, 0x11},
    {0x3250, 0x03},
    {0x3E10, 0x00},
    {0x3E11, 0x00},
    {0x3F0D, 0x00},
    {0x3F42, 0x00},
    {0x3F43, 0x00},

    // Digital Crop & Scaling
    {0x0401, 0x00},
    {0x0404, 0x00},
    {0x0405, 0x10},
    {0x0408, 0x00},
    {0x0409, 0x00},
    {0x040A, 0x00},
    {0x040B, 0x00},
    {0x040C, 0x03},
    {0x040D, 0xF4},
    {0x040E, 0x02},
    {0x040F, 0xF8},

    // Output Crop
    {0x034C, 0x03},
    {0x034D, 0xF4},
    {0x034E, 0x02},
    {0x034F, 0xF8},

    // Clock Setting
    {0x0301, 0x05},
    {0x0303, 0x04},
    {0x0305, 0x04},
    {0x0306, 0x01},
    {0x0307, 0x2C},
    {0x0309, 0x0A},
    {0x030B, 0x04},
    {0x030D, 0x02},
    {0x030E, 0x00},
    {0x030F, 0x71},
    {0x0310, 0x01},
    {0x0820, 0x05},
    {0x0821, 0x4C},
    {0x0822, 0x00},
    {0x0823, 0x00},

    // Output Data Select Setting
    {0x3E20, 0x01},
    {0x3E37, 0x00},

    // PowerSave Setting
    {0x3F50, 0x00},
    {0x3F56, 0x00},
    {0x3F57, 0xC7},

    // Other Setting
    {0x3C0A, 0x73},
    {0x3C0B, 0x64},
    {0x3C0C, 0x5F},
    {0x3C0D, 0x00},
    {0x3C0E, 0x00},
    {0x3C0F, 0x00},
    {0x3C10, 0xA4},
    {0x3C11, 0x02},
    {0x3C12, 0x00},
    {0x3C13, 0x03},
    {0x3C14, 0x80},
    {0x3C15, 0x04},
    {0x3C16, 0x15},
    {0x3C17, 0x15},
    {0x3C18, 0x15},
    {0x3C19, 0x15},
    {0x3C1A, 0x15},
    {0x3C1B, 0x15},
    {0x3C1C, 0x06},
    {0x3C1D, 0x06},
    {0x3C1E, 0x06},
    {0x3C1F, 0x06},
    {0x3C20, 0x06},
    {0x3C21, 0x06},
    {0x3C22, 0x3F},
    {0x3C23, 0x0A},
    {0x3E35, 0x01},
    {0x3F4A, 0x01},
    {0x3F4B, 0x7F},
    {0x3F26, 0x00},

#if FIXED_EXP_EN
    // Integration Time Setting
    {0x0202, (R76_50_EXP >> 8) & 0xFF},        //Exposure [15:8]
    {0x0203, (R76_50_EXP >> 0) & 0xFF},        //Exposure [7:0]

    // Gain Setting
    {0x0204, (R76_50_A_GAIN >> 8) & 0x03},     //Analog Gain[9:8]
    {0x0205, (R76_50_A_GAIN >> 0) & 0xFF},     //Analog Gain[7:0]
    {0x020E, (R76_50_D_GAIN_GR >> 8) & 0xFF},  //Digital Gain GR[15:8]
    {0x020F, (R76_50_D_GAIN_GR >> 0) & 0xFF},  //Digital Gain GR[7:0]
    {0x0210, (R76_50_D_GAIN_R >> 8) & 0xFF},   //Digital Gain R[15:8]
    {0x0211, (R76_50_D_GAIN_R >> 0) & 0xFF},   //Digital Gain R[7:0]
    {0x0212, (R76_50_D_GAIN_B >> 8) & 0xFF},   //Digital Gain B[15:8]
    {0x0213, (R76_50_D_GAIN_B >> 0) & 0xFF},   //Digital Gain B[7:0]
    {0x0214, (R76_50_D_GAIN_GB >> 8) & 0xFF},  //Digital Gain GB[15:8]
    {0x0215, (R76_50_D_GAIN_GB >> 0) & 0xFF},  //Digital Gain GB[7:0]
#else
    // Integration Time Setting
    {0x0202, 0x09},
    {0x0203, 0x5D},

    // Gain Setting
    {0x0204, 0x00},
    {0x0205, 0x00},
    {0x020E, 0x01},
    {0x020F, 0x00},
    {0x0210, 0x01},
    {0x0211, 0x00},
    {0x0212, 0x01},
    {0x0213, 0x00},
    {0x0214, 0x01},
    {0x0215, 0x00},
#endif

    // stanby cancel
    {0x0100, 0x01},
    {0xFFFF, 0xFF},
};

// Full resolution 4056x3040 RAW10 25fps 900Mbps
static const regval16_t imx577_init_table_full_4lane_25fps[] =
{
    // sensor standby
    {0x0100, 0x00},

    // External Clock Setting
    {0x0136, 0x18},
    {0x0137, 0x00},
    // Register version
    {0x3C7E, 0x01},
    {0x3C7F, 0x02},
    // Global Setting
    {0x38A8, 0x1F},
    {0x38A9, 0xFF},
    {0x38AA, 0x1F},
    {0x38AB, 0xFF},
    {0x55D4, 0x00},
    {0x55D5, 0x00},
    {0x55D6, 0x07},
    {0x55D7, 0xFF},
    {0x55E8, 0x07},
    {0x55E9, 0xFF},
    {0x55EA, 0x00},
    {0x55EB, 0x00},
    {0x575C, 0x07},
    {0x575D, 0xFF},
    {0x575E, 0x00},
    {0x575F, 0x00},
    {0x5764, 0x00},
    {0x5765, 0x00},
    {0x5766, 0x07},
    {0x5767, 0xFF},
    {0x5974, 0x04},
    {0x5975, 0x01},
    {0x5F10, 0x09},
    {0x5F11, 0x92},
    {0x5F12, 0x32},
    {0x5F13, 0x72},
    {0x5F14, 0x16},
    {0x5F15, 0xBA},
    {0x5F17, 0x13},
    {0x5F18, 0x24},
    {0x5F19, 0x60},
    {0x5F1A, 0xE3},
    {0x5F1B, 0xAD},
    {0x5F1C, 0x74},
    {0x5F2D, 0x25},
    {0x5F5C, 0xD0},
    {0x6A22, 0x00},
    {0x6A23, 0x1D},
    {0x7BA8, 0x00},
    {0x7BA9, 0x00},
    {0x886B, 0x00},
    {0x9002, 0x0A},
    {0x9004, 0x1A},
    {0x9214, 0x93},
    {0x9215, 0x69},
    {0x9216, 0x93},
    {0x9217, 0x6B},
    {0x9218, 0x93},
    {0x9219, 0x6D},
    {0x921A, 0x57},
    {0x921B, 0x58},
    {0x921C, 0x57},
    {0x921D, 0x59},
    {0x921E, 0x57},
    {0x921F, 0x5A},
    {0x9220, 0x57},
    {0x9221, 0x5B},
    {0x9222, 0x93},
    {0x9223, 0x02},
    {0x9224, 0x93},
    {0x9225, 0x03},
    {0x9226, 0x93},
    {0x9227, 0x04},
    {0x9228, 0x93},
    {0x9229, 0x05},
    {0x922A, 0x98},
    {0x922B, 0x21},
    {0x922C, 0xB2},
    {0x922D, 0xDB},
    {0x922E, 0xB2},
    {0x922F, 0xDC},
    {0x9230, 0xB2},
    {0x9231, 0xDD},
    {0x9232, 0xB2},
    {0x9233, 0xE1},
    {0x9234, 0xB2},
    {0x9235, 0xE2},
    {0x9236, 0xB2},
    {0x9237, 0xE3},
    {0x9238, 0xB7},
    {0x9239, 0xB9},
    {0x923A, 0xB7},
    {0x923B, 0xBB},
    {0x923C, 0xB7},
    {0x923D, 0xBC},
    {0x923E, 0xB7},
    {0x923F, 0xC5},
    {0x9240, 0xB7},
    {0x9241, 0xC7},
    {0x9242, 0xB7},
    {0x9243, 0xC9},
    {0x9244, 0x98},
    {0x9245, 0x56},
    {0x9246, 0x98},
    {0x9247, 0x55},
    {0x9380, 0x00},
    {0x9381, 0x62},
    {0x9382, 0x00},
    {0x9383, 0x56},
    {0x9384, 0x00},
    {0x9385, 0x52},
    {0x9388, 0x00},
    {0x9389, 0x55},
    {0x938A, 0x00},
    {0x938B, 0x55},
    {0x938C, 0x00},
    {0x938D, 0x41},
    {0x5078, 0x01},

    // MIPI setting
    {0x0112, 0x0A},
    {0x0113, 0x0A},
    {0x0114, 0x03},

    // Frame Horizontal Clock Count
    {0x0342, 0x27},
    {0x0343, 0x10},

    // Frame Vertical Clock Count
    {0x0340, 0x0D},
    {0x0341, 0x20},
    {0x3210, 0x00},

    // Visible Size
    {0x0344, 0x00},
    {0x0345, 0x00},
    {0x0346, 0x00},
    {0x0347, 0x00},
    {0x0348, 0x0F},
    {0x0349, 0xD7},
    {0x034A, 0x0B},
    {0x034B, 0xDF},

    // Mode Setting
    {0x00E3, 0x00},
    {0x00E4, 0x00},
    {0x00E5, 0x01},
    {0x00FC, 0x0A},
    {0x00FD, 0x0A},
    {0x00FE, 0x0A},
    {0x00FF, 0x0A},
    {0xE013, 0x00},
    {0x0220, 0x00},
    {0x0221, 0x11},
    {0x0381, 0x01},
    {0x0383, 0x01},
    {0x0385, 0x01},
    {0x0387, 0x01},
    {0x0900, 0x00},
    {0x0901, 0x11},
    {0x0902, 0x00},
    {0x3140, 0x02},
    {0x3241, 0x11},
    {0x3250, 0x03},
    {0x3E10, 0x00},
    {0x3E11, 0x00},
    {0x3F0D, 0x00},
    {0x3F42, 0x00},
    {0x3F43, 0x00},

    // Digital Crop & Scaling
    {0x0401, 0x00},
    {0x0404, 0x00},
    {0x0405, 0x10},
    {0x0408, 0x00},
    {0x0409, 0x00},
    {0x040A, 0x00},
    {0x040B, 0x00},
    {0x040C, 0x0F},
    {0x040D, 0xD8},
    {0x040E, 0x0B},
    {0x040F, 0xE0},

    // Output Crop
    {0x034C, 0x0F},
    {0x034D, 0xD8},
    {0x034E, 0x0B},
    {0x034F, 0xE0},

    // Clock Setting
    {0x0301, 0x05},
    {0x0303, 0x02},
    {0x0305, 0x04},
    {0x0306, 0x01},
    {0x0307, 0x5E},
    {0x0309, 0x0A},
    {0x030B, 0x02},
    {0x030D, 0x04},
    {0x030E, 0x01},
    {0x030F, 0x2C},
    {0x0310, 0x01},
    {0x0820, 0x0E},
    {0x0821, 0x10},
    {0x0822, 0x00},
    {0x0823, 0x00},

    // Output Data Select Setting
    {0x3E20, 0x01},
    {0x3E37, 0x00},

    // PowerSave Setting
    {0x3F50, 0x00},
    {0x3F56, 0x01},
    {0x3F57, 0x1E},

    // Other Setting
    {0x3C0A, 0x5A},
    {0x3C0B, 0x55},
    {0x3C0C, 0x28},
    {0x3C0D, 0x07},
    {0x3C0E, 0xFF},
    {0x3C0F, 0x00},
    {0x3C10, 0x00},
    {0x3C11, 0x02},
    {0x3C12, 0x00},
    {0x3C13, 0x03},
    {0x3C14, 0x00},
    {0x3C15, 0x00},
    {0x3C16, 0x0C},
    {0x3C17, 0x0C},
    {0x3C18, 0x0C},
    {0x3C19, 0x0A},
    {0x3C1A, 0x0A},
    {0x3C1B, 0x0A},
    {0x3C1C, 0x00},
    {0x3C1D, 0x00},
    {0x3C1E, 0x00},
    {0x3C1F, 0x00},
    {0x3C20, 0x00},
    {0x3C21, 0x00},
    {0x3C22, 0x3F},
    {0x3C23, 0x0A},
    {0x3E35, 0x01},
    {0x3F4A, 0x03},
    {0x3F4B, 0xBF},
    {0x3F26, 0x00},

#if FIXED_EXP_EN
    // Integration Time Setting
    {0x0202, (FUL_EXP >> 8) & 0xFF},        //Exposure [15:8]
    {0x0203, (FUL_EXP >> 0) & 0xFF},        //Exposure [7:0]

    // Gain Setting
    {0x0204, (FUL_A_GAIN >> 8) & 0x03},     //Analog Gain[9:8]
    {0x0205, (FUL_A_GAIN >> 0) & 0xFF},     //Analog Gain[7:0]
    {0x020E, (FUL_D_GAIN_GR >> 8) & 0xFF},  //Digital Gain GR[15:8]
    {0x020F, (FUL_D_GAIN_GR >> 0) & 0xFF},  //Digital Gain GR[7:0]
    {0x0210, (FUL_D_GAIN_R >> 8) & 0xFF},   //Digital Gain R[15:8]
    {0x0211, (FUL_D_GAIN_R >> 0) & 0xFF},   //Digital Gain R[7:0]
    {0x0212, (FUL_D_GAIN_B >> 8) & 0xFF},   //Digital Gain B[15:8]
    {0x0213, (FUL_D_GAIN_B >> 0) & 0xFF},   //Digital Gain B[7:0]
    {0x0214, (FUL_D_GAIN_GB >> 8) & 0xFF},  //Digital Gain GB[15:8]
    {0x0215, (FUL_D_GAIN_GB >> 0) & 0xFF},  //Digital Gain GB[7:0]
#else
    // Integration Time Setting
    {0x0202, 0x0D},
    {0x0203, 0x0A},

    // Gain Setting
    {0x0204, 0x00},
    {0x0205, 0x00},
    {0x020E, 0x01},
    {0x020F, 0x00},
    {0x0210, 0x01},
    {0x0211, 0x00},
    {0x0212, 0x01},
    {0x0213, 0x00},
    {0x0214, 0x01},
    {0x0215, 0x00},
#endif

    //stanby cancel
    {0x0100, 0x01},
    {0xFFFF, 0xFF},
};

static const regval16_t imx577_vga_2lane_120fps_partial_setting[] =
{
    // sensor standby
    {0x0100, 0x00},

    // MIPI setting
    {0x0114, 0x01},

    // Frame Horizontal Clock Count
    {0x0342, 0x11},
    {0x0343, 0xA0},

    // Frame Vertical Clock Count
    {0x0340, 0x05},
    {0x0341, 0x31},

    // Mode Setting
    {0x0387, 0x03},
    {0x0900, 0x01},
    {0x0901, 0x42},
    {0x0902, 0x02},

    // Digital Crop & Scaling
    {0x0409, 0xBA},
    {0x040B, 0x8C},
    {0x040C, 0x02},
    {0x040D, 0x80},
    {0x040E, 0x01},
    {0x040F, 0xE0},

    // Output Crop
    {0x034C, 0x02},
    {0x034D, 0x80},
    {0x034E, 0x01},
    {0x034F, 0xE0},

    // Clock Setting
    {0x0303, 0x02},
    {0x0305, 0x02},
    {0x0306, 0x00},
    {0x0307, 0x96},
    {0x030B, 0x02},
    {0x030D, 0x04},
    {0x030E, 0x00},
    {0x030F, 0xC8},
    {0x0310, 0x01},
    {0x0820, 0x04},
    {0x0821, 0xB0},

    // PowerSave Setting
    {0x3F56, 0x00},
    {0x3F57, 0x97},

    // Other Setting
    {0x3C0A, 0x73},
    {0x3C0B, 0x64},
    {0x3C0C, 0x5F},
    {0x3C0D, 0x00},
    {0x3C0E, 0x00},
    {0x3C10, 0xA4},
    {0x3C14, 0x80},
    {0x3C15, 0x04},
    {0x3C16, 0x15},
    {0x3C17, 0x15},
    {0x3C18, 0x15},
    {0x3C19, 0x15},
    {0x3C1A, 0x15},
    {0x3C1B, 0x15},
    {0x3C1C, 0x06},
    {0x3C1D, 0x06},
    {0x3C1E, 0x06},
    {0x3C1F, 0x06},
    {0x3C20, 0x06},
    {0x3C21, 0x06},
    {0x3F4A, 0x01},
    {0x3F4B, 0x7F},

#if FIXED_EXP_EN
    // Integration Time Setting
    {0x0202, (VGA_120_EXP >> 8) & 0xFF},        //Exposure [15:8]
    {0x0203, (VGA_120_EXP >> 0) & 0xFF},        //Exposure [7:0]

    // Gain Setting
    {0x0204, (VGA_120_A_GAIN >> 8) & 0x03},     //Analog Gain[9:8]
    {0x0205, (VGA_120_A_GAIN >> 0) & 0xFF},     //Analog Gain[7:0]
    {0x020E, (VGA_120_D_GAIN_GR >> 8) & 0xFF},  //Digital Gain GR[15:8]
    {0x020F, (VGA_120_D_GAIN_GR >> 0) & 0xFF},  //Digital Gain GR[7:0]
    {0x0210, (VGA_120_D_GAIN_R >> 8) & 0xFF},   //Digital Gain R[15:8]
    {0x0211, (VGA_120_D_GAIN_R >> 0) & 0xFF},   //Digital Gain R[7:0]
    {0x0212, (VGA_120_D_GAIN_B >> 8) & 0xFF},   //Digital Gain B[15:8]
    {0x0213, (VGA_120_D_GAIN_B >> 0) & 0xFF},   //Digital Gain B[7:0]
    {0x0214, (VGA_120_D_GAIN_GB >> 8) & 0xFF},  //Digital Gain GB[15:8]
    {0x0215, (VGA_120_D_GAIN_GB >> 0) & 0xFF},  //Digital Gain GB[7:0]
#endif //FIXED_EXP_EN

    // stanby cancel
    {0x0100, 0x01},
    {0xFFFF, 0xFF},
};

static const regval16_t imx577_vga_2lane_50fps_partial_setting[] =
{
    // sensor standby
    {0x0100, 0x00},

    // MIPI setting
    {0x0114, 0x01},

    // Frame Horizontal Clock Count
    {0x0342, 0x11},
    {0x0343, 0xA0},

    // Frame Vertical Clock Count
    {0x0340, 0x06},
    {0x0341, 0x3B},

    // Mode Setting
    {0x0387, 0x03},
    {0x0900, 0x01},
    {0x0901, 0x42},
    {0x0902, 0x02},

    // Digital Crop & Scaling
    {0x0409, 0xBA},
    {0x040B, 0x8C},
    {0x040C, 0x02},
    {0x040D, 0x80},
    {0x040E, 0x01},
    {0x040F, 0xE0},

    // Output Crop
    {0x034C, 0x02},
    {0x034D, 0x80},
    {0x034E, 0x01},
    {0x034F, 0xE0},

    // Clock Setting
    {0x0303, 0x04},
    {0x0305, 0x04},
    {0x0306, 0x01},
    {0x0307, 0x2C},
    {0x030B, 0x02},
    {0x030D, 0x02},
    {0x030E, 0x00},
    {0x030F, 0x64},
    {0x0310, 0x01},
    {0x0820, 0x04},
    {0x0821, 0xB0},

    // PowerSave Setting
    {0x3F56, 0x01},
    {0x3F57, 0x2E},

    // Other Setting
    {0x3C0A, 0x73},
    {0x3C0B, 0x64},
    {0x3C0C, 0x5F},
    {0x3C0D, 0x00},
    {0x3C0E, 0x00},
    {0x3C10, 0xA4},
    {0x3C14, 0x80},
    {0x3C15, 0x04},
    {0x3C16, 0x15},
    {0x3C17, 0x15},
    {0x3C18, 0x15},
    {0x3C19, 0x15},
    {0x3C1A, 0x15},
    {0x3C1B, 0x15},
    {0x3C1C, 0x06},
    {0x3C1D, 0x06},
    {0x3C1E, 0x06},
    {0x3C1F, 0x06},
    {0x3C20, 0x06},
    {0x3C21, 0x06},
    {0x3F4A, 0x01},
    {0x3F4B, 0x7F},

#if FIXED_EXP_EN
    // Integration Time Setting
    {0x0202, (VGA_50_EXP >> 8) & 0xFF},         //Exposure [15:8]
    {0x0203, (VGA_50_EXP >> 0) & 0xFF},         //Exposure [7:0]

    // Gain Setting
    {0x0204, (VGA_50_A_GAIN >> 8) & 0x03},      //Analog Gain[9:8]
    {0x0205, (VGA_50_A_GAIN >> 0) & 0xFF},      //Analog Gain[7:0]
    {0x020E, (VGA_50_D_GAIN_GR >> 8) & 0xFF},   //Digital Gain GR[15:8]
    {0x020F, (VGA_50_D_GAIN_GR >> 0) & 0xFF},   //Digital Gain GR[7:0]
    {0x0210, (VGA_50_D_GAIN_R >> 8) & 0xFF},    //Digital Gain R[15:8]
    {0x0211, (VGA_50_D_GAIN_R >> 0) & 0xFF},    //Digital Gain R[7:0]
    {0x0212, (VGA_50_D_GAIN_B >> 8) & 0xFF},    //Digital Gain B[15:8]
    {0x0213, (VGA_50_D_GAIN_B >> 0) & 0xFF},    //Digital Gain B[7:0]
    {0x0214, (VGA_50_D_GAIN_GB >> 8) & 0xFF},   //Digital Gain GB[15:8]
    {0x0215, (VGA_50_D_GAIN_GB >> 0) & 0xFF},   //Digital Gain GB[7:0]
#endif //FIXED_EXP_EN

    // stanby cancel
    {0x0100, 0x01},
    {0xFFFF, 0xFF},
};

static const regval16_t imx577_760_4lane_120fps_partial_setting[] =
{
    // sensor standby
    {0x0100, 0x00},

    // MIPI setting
    {0x0114, 0x03},

    // Frame Horizontal Clock Count
    {0x0342, 0x0B},
    {0x0343, 0xA0},

    // Frame Vertical Clock Count
    {0x0340, 0x03},
    {0x0341, 0xF0},

    // Mode Setting
    {0x0387, 0x03},
    {0x0900, 0x01},
    {0x0901, 0x42},
    {0x0902, 0x02},

    // Digital Crop & Scaling
    {0x0409, 0x00},
    {0x040B, 0x00},
    {0x040C, 0x03},
    {0x040D, 0xF4},
    {0x040E, 0x02},
    {0x040F, 0xF8},

    // Output Crop
    {0x034C, 0x03},
    {0x034D, 0xF4},
    {0x034E, 0x02},
    {0x034F, 0xF8},

    // Clock Setting
    {0x0303, 0x04},
    {0x0305, 0x04},
    {0x0306, 0x01},
    {0x0307, 0x2C},
    {0x030B, 0x04},
    {0x030D, 0x02},
    {0x030E, 0x00},
    {0x030F, 0x71},
    {0x0310, 0x01},
    {0x0820, 0x05},
    {0x0821, 0x4C},

    // PowerSave Setting
    {0x3F56, 0x00},
    {0x3F57, 0xC7},

    // Other Setting
    {0x3C0A, 0x73},
    {0x3C0B, 0x64},
    {0x3C0C, 0x5F},
    {0x3C0D, 0x00},
    {0x3C0E, 0x00},
    {0x3C10, 0xA4},
    {0x3C14, 0x80},
    {0x3C15, 0x04},
    {0x3C16, 0x15},
    {0x3C17, 0x15},
    {0x3C18, 0x15},
    {0x3C19, 0x15},
    {0x3C1A, 0x15},
    {0x3C1B, 0x15},
    {0x3C1C, 0x06},
    {0x3C1D, 0x06},
    {0x3C1E, 0x06},
    {0x3C1F, 0x06},
    {0x3C20, 0x06},
    {0x3C21, 0x06},
    {0x3F4A, 0x01},
    {0x3F4B, 0x7F},

#if FIXED_EXP_EN
    // Integration Time Setting
    {0x0202, (R76_120_EXP >> 8) & 0xFF},        //Exposure [15:8]
    {0x0203, (R76_120_EXP >> 0) & 0xFF},        //Exposure [7:0]

    // Gain Setting
    {0x0204, (R76_120_A_GAIN >> 8) & 0x03},     //Analog Gain[9:8]
    {0x0205, (R76_120_A_GAIN >> 0) & 0xFF},     //Analog Gain[7:0]
    {0x020E, (R76_120_D_GAIN_GR >> 8) & 0xFF},  //Digital Gain GR[15:8]
    {0x020F, (R76_120_D_GAIN_GR >> 0) & 0xFF},  //Digital Gain GR[7:0]
    {0x0210, (R76_120_D_GAIN_R >> 8) & 0xFF},   //Digital Gain R[15:8]
    {0x0211, (R76_120_D_GAIN_R >> 0) & 0xFF},   //Digital Gain R[7:0]
    {0x0212, (R76_120_D_GAIN_B >> 8) & 0xFF},   //Digital Gain B[15:8]
    {0x0213, (R76_120_D_GAIN_B >> 0) & 0xFF},   //Digital Gain B[7:0]
    {0x0214, (R76_120_D_GAIN_GB >> 8) & 0xFF},  //Digital Gain GB[15:8]
    {0x0215, (R76_120_D_GAIN_GB >> 0) & 0xFF},  //Digital Gain GB[7:0]
#endif //FIXED_EXP_EN

    // stanby cancel
    {0x0100, 0x01},
    {0xFFFF, 0xFF},
};

static const regval16_t imx577_760_4lane_50fps_partial_setting[] =
{
    // sensor standby
    {0x0100, 0x00},

    // MIPI setting
    {0x0114, 0x03},

    // Frame Horizontal Clock Count
    {0x0342, 0x0B},
    {0x0343, 0xA0},

    // Frame Vertical Clock Count
    {0x0340, 0x09},
    {0x0341, 0x73},

    // Mode Setting
    {0x0387, 0x03},
    {0x0900, 0x01},
    {0x0901, 0x42},
    {0x0902, 0x02},

    // Digital Crop & Scaling
    {0x0409, 0x00},
    {0x040B, 0x00},
    {0x040C, 0x03},
    {0x040D, 0xF4},
    {0x040E, 0x02},
    {0x040F, 0xF8},

    // Output Crop
    {0x034C, 0x03},
    {0x034D, 0xF4},
    {0x034E, 0x02},
    {0x034F, 0xF8},

    // Clock Setting
    {0x0303, 0x04},
    {0x0305, 0x04},
    {0x0306, 0x01},
    {0x0307, 0x2C},
    {0x030B, 0x04},
    {0x030D, 0x02},
    {0x030E, 0x00},
    {0x030F, 0x71},
    {0x0310, 0x01},
    {0x0820, 0x05},
    {0x0821, 0x4C},

    // PowerSave Setting
    {0x3F56, 0x00},
    {0x3F57, 0xC7},

    // Other Setting
    {0x3C0A, 0x73},
    {0x3C0B, 0x64},
    {0x3C0C, 0x5F},
    {0x3C0D, 0x00},
    {0x3C0E, 0x00},
    {0x3C10, 0xA4},
    {0x3C14, 0x80},
    {0x3C15, 0x04},
    {0x3C16, 0x15},
    {0x3C17, 0x15},
    {0x3C18, 0x15},
    {0x3C19, 0x15},
    {0x3C1A, 0x15},
    {0x3C1B, 0x15},
    {0x3C1C, 0x06},
    {0x3C1D, 0x06},
    {0x3C1E, 0x06},
    {0x3C1F, 0x06},
    {0x3C20, 0x06},
    {0x3C21, 0x06},
    {0x3F4A, 0x01},
    {0x3F4B, 0x7F},

#if FIXED_EXP_EN
    // Integration Time Setting
    {0x0202, (R76_50_EXP >> 8) & 0xFF},         //Exposure [15:8]
    {0x0203, (R76_50_EXP >> 0) & 0xFF},         //Exposure [7:0]

    // Gain Setting
    {0x0204, (R76_50_A_GAIN >> 8) & 0x03},      //Analog Gain[9:8]
    {0x0205, (R76_50_A_GAIN >> 0) & 0xFF},      //Analog Gain[7:0]
    {0x020E, (R76_50_D_GAIN_GR >> 8) & 0xFF},   //Digital Gain GR[15:8]
    {0x020F, (R76_50_D_GAIN_GR >> 0) & 0xFF},   //Digital Gain GR[7:0]
    {0x0210, (R76_50_D_GAIN_R >> 8) & 0xFF},    //Digital Gain R[15:8]
    {0x0211, (R76_50_D_GAIN_R >> 0) & 0xFF},    //Digital Gain R[7:0]
    {0x0212, (R76_50_D_GAIN_B >> 8) & 0xFF},    //Digital Gain B[15:8]
    {0x0213, (R76_50_D_GAIN_B >> 0) & 0xFF},    //Digital Gain B[7:0]
    {0x0214, (R76_50_D_GAIN_GB >> 8) & 0xFF},   //Digital Gain GB[15:8]
    {0x0215, (R76_50_D_GAIN_GB >> 0) & 0xFF},   //Digital Gain GB[7:0]
#endif //FIXED_EXP_EN

    // stanby cancel
    {0x0100, 0x01},
    {0xFFFF, 0xFF},
};

static const regval16_t imx577_full_4lane_25fps_partial_setting[] =
{
    // sensor standby
    {0x0100, 0x00},

    // MIPI setting
    {0x0114, 0x03},

    // Frame Horizontal Clock Count
    {0x0342, 0x27},
    {0x0343, 0x10},

    // Frame Vertical Clock Count
    {0x0340, 0x0D},
    {0x0341, 0x20},

    // Mode Setting
    {0x0387, 0x01},
    {0x0900, 0x00},
    {0x0901, 0x11},
    {0x0902, 0x00},

    // Digital Crop & Scaling
    {0x0409, 0x00},
    {0x040B, 0x00},
    {0x040C, 0x0F},
    {0x040D, 0xD8},
    {0x040E, 0x0B},
    {0x040F, 0xE0},

    // Output Crop
    {0x034C, 0x0F},
    {0x034D, 0xD8},
    {0x034E, 0x0B},
    {0x034F, 0xE0},

    // Clock Setting
    {0x0303, 0x02},
    {0x0305, 0x04},
    {0x0306, 0x01},
    {0x0307, 0x5E},
    {0x030B, 0x02},
    {0x030D, 0x04},
    {0x030E, 0x01},
    {0x030F, 0x2C},
    {0x0310, 0x01},
    {0x0820, 0x0E},
    {0x0821, 0x10},

    // PowerSave Setting
    {0x3F56, 0x01},
    {0x3F57, 0x1E},

    // Other Setting
    {0x3C0A, 0x5A},
    {0x3C0B, 0x55},
    {0x3C0C, 0x28},
    {0x3C0D, 0x07},
    {0x3C0E, 0xFF},
    {0x3C10, 0x00},
    {0x3C14, 0x00},
    {0x3C15, 0x00},
    {0x3C16, 0x0C},
    {0x3C17, 0x0C},
    {0x3C18, 0x0C},
    {0x3C19, 0x0A},
    {0x3C1A, 0x0A},
    {0x3C1B, 0x0A},
    {0x3C1C, 0x00},
    {0x3C1D, 0x00},
    {0x3C1E, 0x00},
    {0x3C1F, 0x00},
    {0x3C20, 0x00},
    {0x3C21, 0x00},
    {0x3F4A, 0x03},
    {0x3F4B, 0xBF},

#if FIXED_EXP_EN
    // Integration Time Setting
    {0x0202, (FUL_EXP >> 8) & 0xFF},        //Exposure [15:8]
    {0x0203, (FUL_EXP >> 0) & 0xFF},        //Exposure [7:0]

    // Gain Setting
    {0x0204, (FUL_A_GAIN >> 8) & 0x03},     //Analog Gain[9:8]
    {0x0205, (FUL_A_GAIN >> 0) & 0xFF},     //Analog Gain[7:0]
    {0x020E, (FUL_D_GAIN_GR >> 8) & 0xFF},  //Digital Gain GR[15:8]
    {0x020F, (FUL_D_GAIN_GR >> 0) & 0xFF},  //Digital Gain GR[7:0]
    {0x0210, (FUL_D_GAIN_R >> 8) & 0xFF},   //Digital Gain R[15:8]
    {0x0211, (FUL_D_GAIN_R >> 0) & 0xFF},   //Digital Gain R[7:0]
    {0x0212, (FUL_D_GAIN_B >> 8) & 0xFF},   //Digital Gain B[15:8]
    {0x0213, (FUL_D_GAIN_B >> 0) & 0xFF},   //Digital Gain B[7:0]
    {0x0214, (FUL_D_GAIN_GB >> 8) & 0xFF},  //Digital Gain GB[15:8]
    {0x0215, (FUL_D_GAIN_GB >> 0) & 0xFF},  //Digital Gain GB[7:0]
#endif //FIXED_EXP_EN

    // stanby cancel
    {0x0100, 0x01},
    {0xFFFF, 0xFF},
};

static const int imx577_120fps_exp_time_gain_50Hz[IMX577_120FPS_50HZ_EXP_TIME_TOTAL][3] =
{// {time, analog gain, digital gain}
    {3, (int)(1.00*64), (int)(1.00*64)},
    {3, (int)(1.04*64), (int)(1.00*64)},
    {3, (int)(1.07*64), (int)(1.00*64)},
    {3, (int)(1.11*64), (int)(1.00*64)},
    {3, (int)(1.15*64), (int)(1.00*64)},
    {3, (int)(1.19*64), (int)(1.00*64)},
    {3, (int)(1.23*64), (int)(1.00*64)},
    {3, (int)(1.27*64), (int)(1.00*64)},
    {3, (int)(1.32*64), (int)(1.00*64)},
    {3, (int)(1.37*64), (int)(1.00*64)},
    {3, (int)(1.41*64), (int)(1.00*64)},
    {3, (int)(1.46*64), (int)(1.00*64)},
    {3, (int)(1.52*64), (int)(1.00*64)},
    {3, (int)(1.57*64), (int)(1.00*64)},
    {3, (int)(1.62*64), (int)(1.00*64)},
    {3, (int)(1.68*64), (int)(1.00*64)},
    {3, (int)(1.74*64), (int)(1.00*64)},
    {3, (int)(1.80*64), (int)(1.00*64)},
    {3, (int)(1.87*64), (int)(1.00*64)},
    {3, (int)(1.93*64), (int)(1.00*64)},
    {6, (int)(1.00*64), (int)(1.00*64)},
    {6, (int)(1.04*64), (int)(1.00*64)},
    {6, (int)(1.07*64), (int)(1.00*64)},
    {6, (int)(1.11*64), (int)(1.00*64)},
    {6, (int)(1.15*64), (int)(1.00*64)},
    {6, (int)(1.19*64), (int)(1.00*64)},
    {6, (int)(1.23*64), (int)(1.00*64)},
    {6, (int)(1.27*64), (int)(1.00*64)},
    {6, (int)(1.32*64), (int)(1.00*64)},
    {6, (int)(1.37*64), (int)(1.00*64)},
    {6, (int)(1.41*64), (int)(1.00*64)},
    {6, (int)(1.46*64), (int)(1.00*64)},
    {6, (int)(1.52*64), (int)(1.00*64)},
    {6, (int)(1.57*64), (int)(1.00*64)},
    {6, (int)(1.62*64), (int)(1.00*64)},
    {6, (int)(1.68*64), (int)(1.00*64)},
    {6, (int)(1.74*64), (int)(1.00*64)},
    {6, (int)(1.80*64), (int)(1.00*64)},
    {6, (int)(1.87*64), (int)(1.00*64)},
    {6, (int)(1.93*64), (int)(1.00*64)},
    {13, (int)(1.00*64), (int)(1.00*64)},
    {13, (int)(1.04*64), (int)(1.00*64)},
    {13, (int)(1.07*64), (int)(1.00*64)},
    {13, (int)(1.11*64), (int)(1.00*64)},
    {13, (int)(1.15*64), (int)(1.00*64)},
    {13, (int)(1.19*64), (int)(1.00*64)},
    {13, (int)(1.23*64), (int)(1.00*64)},
    {13, (int)(1.27*64), (int)(1.00*64)},
    {13, (int)(1.32*64), (int)(1.00*64)},
    {13, (int)(1.37*64), (int)(1.00*64)},
    {13, (int)(1.41*64), (int)(1.00*64)},
    {13, (int)(1.46*64), (int)(1.00*64)},
    {13, (int)(1.52*64), (int)(1.00*64)},
    {13, (int)(1.57*64), (int)(1.00*64)},
    {13, (int)(1.62*64), (int)(1.00*64)},
    {13, (int)(1.68*64), (int)(1.00*64)},
    {13, (int)(1.74*64), (int)(1.00*64)},
    {13, (int)(1.80*64), (int)(1.00*64)},
    {13, (int)(1.87*64), (int)(1.00*64)},
    {13, (int)(1.93*64), (int)(1.00*64)},
    {26, (int)(1.00*64), (int)(1.00*64)},
    {26, (int)(1.04*64), (int)(1.00*64)},
    {26, (int)(1.07*64), (int)(1.00*64)},
    {26, (int)(1.11*64), (int)(1.00*64)},
    {26, (int)(1.15*64), (int)(1.00*64)},
    {26, (int)(1.19*64), (int)(1.00*64)},
    {26, (int)(1.23*64), (int)(1.00*64)},
    {26, (int)(1.27*64), (int)(1.00*64)},
    {26, (int)(1.32*64), (int)(1.00*64)},
    {26, (int)(1.37*64), (int)(1.00*64)},
    {26, (int)(1.41*64), (int)(1.00*64)},
    {26, (int)(1.46*64), (int)(1.00*64)},
    {26, (int)(1.52*64), (int)(1.00*64)},
    {26, (int)(1.57*64), (int)(1.00*64)},
    {26, (int)(1.62*64), (int)(1.00*64)},
    {26, (int)(1.68*64), (int)(1.00*64)},
    {26, (int)(1.74*64), (int)(1.00*64)},
    {26, (int)(1.80*64), (int)(1.00*64)},
    {26, (int)(1.87*64), (int)(1.00*64)},
    {26, (int)(1.93*64), (int)(1.00*64)},
    {52, (int)(1.00*64), (int)(1.00*64)},
    {52, (int)(1.00*64), (int)(1.00*64)},
    {53, (int)(1.00*64), (int)(1.00*64)},
    {55, (int)(1.00*64), (int)(1.00*64)},
    {57, (int)(1.00*64), (int)(1.00*64)},
    {59, (int)(1.00*64), (int)(1.00*64)},
    {61, (int)(1.00*64), (int)(1.00*64)},
    {64, (int)(1.00*64), (int)(1.00*64)},
    {66, (int)(1.00*64), (int)(1.00*64)},
    {68, (int)(1.00*64), (int)(1.00*64)},
    {70, (int)(1.00*64), (int)(1.00*64)},
    {73, (int)(1.00*64), (int)(1.00*64)},
    {76, (int)(1.00*64), (int)(1.00*64)},
    {78, (int)(1.00*64), (int)(1.00*64)},
    {81, (int)(1.00*64), (int)(1.00*64)},
    {84, (int)(1.00*64), (int)(1.00*64)},
    {87, (int)(1.00*64), (int)(1.00*64)},
    {90, (int)(1.00*64), (int)(1.00*64)},
    {93, (int)(1.00*64), (int)(1.00*64)},
    {96, (int)(1.00*64), (int)(1.00*64)},
    {100, (int)(1.00*64), (int)(1.00*64)},
    {103, (int)(1.00*64), (int)(1.00*64)},
    {107, (int)(1.00*64), (int)(1.00*64)},
    {111, (int)(1.00*64), (int)(1.00*64)},
    {115, (int)(1.00*64), (int)(1.00*64)},
    {119, (int)(1.00*64), (int)(1.00*64)},
    {123, (int)(1.00*64), (int)(1.00*64)},
    {127, (int)(1.00*64), (int)(1.00*64)},
    {132, (int)(1.00*64), (int)(1.00*64)},
    {136, (int)(1.00*64), (int)(1.00*64)},
    {141, (int)(1.00*64), (int)(1.00*64)},
    {146, (int)(1.00*64), (int)(1.00*64)},
    {151, (int)(1.00*64), (int)(1.00*64)},
    {156, (int)(1.00*64), (int)(1.00*64)},
    {162, (int)(1.00*64), (int)(1.00*64)},
    {168, (int)(1.00*64), (int)(1.00*64)},
    {174, (int)(1.00*64), (int)(1.00*64)},
    {180, (int)(1.00*64), (int)(1.00*64)},
    {186, (int)(1.00*64), (int)(1.00*64)},
    {193, (int)(1.00*64), (int)(1.00*64)},
    {199, (int)(1.00*64), (int)(1.00*64)},
    {206, (int)(1.00*64), (int)(1.00*64)},
    {214, (int)(1.00*64), (int)(1.00*64)},
    {221, (int)(1.00*64), (int)(1.00*64)},
    {229, (int)(1.00*64), (int)(1.00*64)},
    {237, (int)(1.00*64), (int)(1.00*64)},
    {245, (int)(1.00*64), (int)(1.00*64)},
    {254, (int)(1.00*64), (int)(1.00*64)},
    {263, (int)(1.00*64), (int)(1.00*64)},
    {272, (int)(1.00*64), (int)(1.00*64)},
    {282, (int)(1.00*64), (int)(1.00*64)},
    {292, (int)(1.00*64), (int)(1.00*64)},
    {302, (int)(1.00*64), (int)(1.00*64)},
    {313, (int)(1.00*64), (int)(1.00*64)},
    {324, (int)(1.00*64), (int)(1.00*64)},
    {335, (int)(1.00*64), (int)(1.00*64)},
    {347, (int)(1.00*64), (int)(1.00*64)},
    {359, (int)(1.00*64), (int)(1.00*64)},
    {372, (int)(1.00*64), (int)(1.00*64)},
    {385, (int)(1.00*64), (int)(1.00*64)},
    {399, (int)(1.00*64), (int)(1.00*64)},
    {413, (int)(1.00*64), (int)(1.00*64)},
    {427, (int)(1.00*64), (int)(1.00*64)},
    {442, (int)(1.00*64), (int)(1.00*64)},
    {458, (int)(1.00*64), (int)(1.00*64)},
    {474, (int)(1.00*64), (int)(1.00*64)},
    {491, (int)(1.00*64), (int)(1.00*64)},
    {508, (int)(1.00*64), (int)(1.00*64)},
    {526, (int)(1.00*64), (int)(1.00*64)},
    {545, (int)(1.00*64), (int)(1.00*64)},
    {564, (int)(1.00*64), (int)(1.00*64)},
    {584, (int)(1.00*64), (int)(1.00*64)},
    {604, (int)(1.00*64), (int)(1.00*64)},
    {626, (int)(1.00*64), (int)(1.00*64)},
    {648, (int)(1.00*64), (int)(1.00*64)},
    {671, (int)(1.00*64), (int)(1.00*64)},
    {694, (int)(1.00*64), (int)(1.00*64)},
    {719, (int)(1.00*64), (int)(1.00*64)},
    {744, (int)(1.00*64), (int)(1.00*64)},
    {770, (int)(1.00*64), (int)(1.00*64)},
    {798, (int)(1.00*64), (int)(1.00*64)},
    {826, (int)(1.00*64), (int)(1.00*64)},
    {855, (int)(1.00*64), (int)(1.00*64)},
    {885, (int)(1.00*64), (int)(1.00*64)},
    {916, (int)(1.00*64), (int)(1.00*64)},
    {948, (int)(1.00*64), (int)(1.00*64)},
    {982, (int)(1.00*64), (int)(1.00*64)},
    {1016, (int)(1.00*64), (int)(1.00*64)},
    {1052, (int)(1.00*64), (int)(1.00*64)},
    {1089, (int)(1.00*64), (int)(1.00*64)},
    {1128, (int)(1.00*64), (int)(1.00*64)},
    {1168, (int)(1.00*64), (int)(1.00*64)},
    {1209, (int)(1.00*64), (int)(1.00*64)},
    {1251, (int)(1.00*64), (int)(1.00*64)},
    {1296, (int)(1.00*64), (int)(1.00*64)},
    {1341, (int)(1.00*64), (int)(1.00*64)},
    {1389, (int)(1.00*64), (int)(1.00*64)},
    {1437, (int)(1.00*64), (int)(1.00*64)},
    {1488, (int)(1.00*64), (int)(1.00*64)},
    {1541, (int)(1.00*64), (int)(1.00*64)},
    {1595, (int)(1.00*64), (int)(1.00*64)},
    {1595, (int)(1.04*64), (int)(1.00*64)},
    {1595, (int)(1.07*64), (int)(1.00*64)},
    {1595, (int)(1.11*64), (int)(1.00*64)},
    {1595, (int)(1.15*64), (int)(1.00*64)},
    {1595, (int)(1.19*64), (int)(1.00*64)},
    {1595, (int)(1.23*64), (int)(1.00*64)},
    {1595, (int)(1.27*64), (int)(1.00*64)},
    {1595, (int)(1.32*64), (int)(1.00*64)},
    {1595, (int)(1.37*64), (int)(1.00*64)},
    {1595, (int)(1.41*64), (int)(1.00*64)},
    {1595, (int)(1.46*64), (int)(1.00*64)},
    {1595, (int)(1.52*64), (int)(1.00*64)},
    {1595, (int)(1.57*64), (int)(1.00*64)},
    {1595, (int)(1.62*64), (int)(1.00*64)},
    {1595, (int)(1.68*64), (int)(1.00*64)},
    {1595, (int)(1.74*64), (int)(1.00*64)},
    {1595, (int)(1.80*64), (int)(1.00*64)},
    {1595, (int)(1.87*64), (int)(1.00*64)},
    {1595, (int)(1.93*64), (int)(1.00*64)},
    {1595, (int)(2.00*64), (int)(1.00*64)},
    {1595, (int)(2.07*64), (int)(1.00*64)},
    {1595, (int)(2.14*64), (int)(1.00*64)},
    {1595, (int)(2.22*64), (int)(1.00*64)},
    {1595, (int)(2.30*64), (int)(1.00*64)},
    {1595, (int)(2.38*64), (int)(1.00*64)},
    {1595, (int)(2.46*64), (int)(1.00*64)},
    {1595, (int)(2.55*64), (int)(1.00*64)},
    {1595, (int)(2.64*64), (int)(1.00*64)},
    {1595, (int)(2.73*64), (int)(1.00*64)},
    {1595, (int)(2.83*64), (int)(1.00*64)},
    {1595, (int)(2.93*64), (int)(1.00*64)},
    {1595, (int)(3.03*64), (int)(1.00*64)},
    {1595, (int)(3.14*64), (int)(1.00*64)},
    {1595, (int)(3.25*64), (int)(1.00*64)},
    {1595, (int)(3.36*64), (int)(1.00*64)},
    {1595, (int)(3.48*64), (int)(1.00*64)},
    {1595, (int)(3.61*64), (int)(1.00*64)},
    {1595, (int)(3.73*64), (int)(1.00*64)},
    {1595, (int)(3.86*64), (int)(1.00*64)},
    {1595, (int)(4.00*64), (int)(1.00*64)},
    {1595, (int)(4.14*64), (int)(1.00*64)},
    {1595, (int)(4.29*64), (int)(1.00*64)},
    {1595, (int)(4.44*64), (int)(1.00*64)},
    {1595, (int)(4.59*64), (int)(1.00*64)},
    {1595, (int)(4.76*64), (int)(1.00*64)},
    {1595, (int)(4.92*64), (int)(1.00*64)},
    {1595, (int)(5.10*64), (int)(1.00*64)},
    {1595, (int)(5.28*64), (int)(1.00*64)},
    {1595, (int)(5.46*64), (int)(1.00*64)},
    {1595, (int)(5.66*64), (int)(1.00*64)},
    {1595, (int)(5.86*64), (int)(1.00*64)},
    {1595, (int)(6.06*64), (int)(1.00*64)},
    {1595, (int)(6.28*64), (int)(1.00*64)},
    {1595, (int)(6.50*64), (int)(1.00*64)},
    {1595, (int)(6.73*64), (int)(1.00*64)},
    {1595, (int)(6.96*64), (int)(1.00*64)},
    {1595, (int)(7.21*64), (int)(1.00*64)},
    {1595, (int)(7.46*64), (int)(1.00*64)},
    {1595, (int)(7.73*64), (int)(1.00*64)},
    {1595, (int)(8.00*64), (int)(1.00*64)},
    {1595, (int)(8.28*64), (int)(1.00*64)},
    {1595, (int)(8.57*64), (int)(1.00*64)},
    {1595, (int)(8.88*64), (int)(1.00*64)},
    {1595, (int)(9.19*64), (int)(1.00*64)},
    {1595, (int)(9.51*64), (int)(1.00*64)},
    {1595, (int)(9.85*64), (int)(1.00*64)},
    {1595, (int)(10.20*64), (int)(1.00*64)},
    {1595, (int)(10.56*64), (int)(1.00*64)},
    {1595, (int)(10.93*64), (int)(1.00*64)},
    {1595, (int)(11.31*64), (int)(1.00*64)},
    {1595, (int)(11.71*64), (int)(1.00*64)},
    {1595, (int)(12.13*64), (int)(1.00*64)},
    {1595, (int)(12.55*64), (int)(1.00*64)},
    {1595, (int)(13.00*64), (int)(1.00*64)},
    {1595, (int)(13.45*64), (int)(1.00*64)},
    {1595, (int)(13.93*64), (int)(1.00*64)},
    {1595, (int)(14.42*64), (int)(1.00*64)},
    {1595, (int)(14.93*64), (int)(1.00*64)},
    {1595, (int)(15.45*64), (int)(1.00*64)},
    {1595, (int)(16.00*64), (int)(1.00*64)},
}; // 50Hz

static const int imx577_120fps_exp_time_gain_60Hz[IMX577_120FPS_60HZ_EXP_TIME_TOTAL][3] =
{// {time, analog gain, digital gain}
    {3, (int)(1.00*64), (int)(1.00*64)},
    {3, (int)(1.04*64), (int)(1.00*64)},
    {3, (int)(1.07*64), (int)(1.00*64)},
    {3, (int)(1.11*64), (int)(1.00*64)},
    {3, (int)(1.15*64), (int)(1.00*64)},
    {3, (int)(1.19*64), (int)(1.00*64)},
    {3, (int)(1.23*64), (int)(1.00*64)},
    {3, (int)(1.27*64), (int)(1.00*64)},
    {3, (int)(1.32*64), (int)(1.00*64)},
    {3, (int)(1.37*64), (int)(1.00*64)},
    {3, (int)(1.41*64), (int)(1.00*64)},
    {3, (int)(1.46*64), (int)(1.00*64)},
    {3, (int)(1.52*64), (int)(1.00*64)},
    {3, (int)(1.57*64), (int)(1.00*64)},
    {3, (int)(1.62*64), (int)(1.00*64)},
    {5, (int)(1.00*64), (int)(1.00*64)},
    {5, (int)(1.04*64), (int)(1.00*64)},
    {5, (int)(1.07*64), (int)(1.00*64)},
    {5, (int)(1.11*64), (int)(1.00*64)},
    {5, (int)(1.15*64), (int)(1.00*64)},
    {5, (int)(1.19*64), (int)(1.00*64)},
    {5, (int)(1.23*64), (int)(1.00*64)},
    {5, (int)(1.27*64), (int)(1.00*64)},
    {5, (int)(1.32*64), (int)(1.00*64)},
    {5, (int)(1.37*64), (int)(1.00*64)},
    {5, (int)(1.41*64), (int)(1.00*64)},
    {5, (int)(1.46*64), (int)(1.00*64)},
    {5, (int)(1.52*64), (int)(1.00*64)},
    {5, (int)(1.57*64), (int)(1.00*64)},
    {5, (int)(1.62*64), (int)(1.00*64)},
    {5, (int)(1.68*64), (int)(1.00*64)},
    {5, (int)(1.74*64), (int)(1.00*64)},
    {5, (int)(1.80*64), (int)(1.00*64)},
    {5, (int)(1.87*64), (int)(1.00*64)},
    {5, (int)(1.93*64), (int)(1.00*64)},
    {11, (int)(1.00*64), (int)(1.00*64)},
    {11, (int)(1.04*64), (int)(1.00*64)},
    {11, (int)(1.07*64), (int)(1.00*64)},
    {11, (int)(1.11*64), (int)(1.00*64)},
    {11, (int)(1.15*64), (int)(1.00*64)},
    {11, (int)(1.19*64), (int)(1.00*64)},
    {11, (int)(1.23*64), (int)(1.00*64)},
    {11, (int)(1.27*64), (int)(1.00*64)},
    {11, (int)(1.32*64), (int)(1.00*64)},
    {11, (int)(1.37*64), (int)(1.00*64)},
    {11, (int)(1.41*64), (int)(1.00*64)},
    {11, (int)(1.46*64), (int)(1.00*64)},
    {11, (int)(1.52*64), (int)(1.00*64)},
    {11, (int)(1.57*64), (int)(1.00*64)},
    {11, (int)(1.62*64), (int)(1.00*64)},
    {11, (int)(1.68*64), (int)(1.00*64)},
    {11, (int)(1.74*64), (int)(1.00*64)},
    {11, (int)(1.80*64), (int)(1.00*64)},
    {11, (int)(1.87*64), (int)(1.00*64)},
    {21, (int)(1.00*64), (int)(1.00*64)},
    {21, (int)(1.04*64), (int)(1.00*64)},
    {21, (int)(1.07*64), (int)(1.00*64)},
    {21, (int)(1.11*64), (int)(1.00*64)},
    {21, (int)(1.15*64), (int)(1.00*64)},
    {21, (int)(1.19*64), (int)(1.00*64)},
    {21, (int)(1.23*64), (int)(1.00*64)},
    {21, (int)(1.27*64), (int)(1.00*64)},
    {21, (int)(1.32*64), (int)(1.00*64)},
    {21, (int)(1.37*64), (int)(1.00*64)},
    {21, (int)(1.41*64), (int)(1.00*64)},
    {21, (int)(1.46*64), (int)(1.00*64)},
    {21, (int)(1.52*64), (int)(1.00*64)},
    {21, (int)(1.57*64), (int)(1.00*64)},
    {21, (int)(1.62*64), (int)(1.00*64)},
    {21, (int)(1.68*64), (int)(1.00*64)},
    {21, (int)(1.74*64), (int)(1.00*64)},
    {21, (int)(1.80*64), (int)(1.00*64)},
    {21, (int)(1.87*64), (int)(1.00*64)},
    {21, (int)(1.93*64), (int)(1.00*64)},
    {43, (int)(1.00*64), (int)(1.00*64)},
    {43, (int)(1.00*64), (int)(1.00*64)},
    {45, (int)(1.00*64), (int)(1.00*64)},
    {46, (int)(1.00*64), (int)(1.00*64)},
    {48, (int)(1.00*64), (int)(1.00*64)},
    {49, (int)(1.00*64), (int)(1.00*64)},
    {51, (int)(1.00*64), (int)(1.00*64)},
    {53, (int)(1.00*64), (int)(1.00*64)},
    {55, (int)(1.00*64), (int)(1.00*64)},
    {57, (int)(1.00*64), (int)(1.00*64)},
    {59, (int)(1.00*64), (int)(1.00*64)},
    {61, (int)(1.00*64), (int)(1.00*64)},
    {63, (int)(1.00*64), (int)(1.00*64)},
    {65, (int)(1.00*64), (int)(1.00*64)},
    {67, (int)(1.00*64), (int)(1.00*64)},
    {70, (int)(1.00*64), (int)(1.00*64)},
    {72, (int)(1.00*64), (int)(1.00*64)},
    {75, (int)(1.00*64), (int)(1.00*64)},
    {78, (int)(1.00*64), (int)(1.00*64)},
    {80, (int)(1.00*64), (int)(1.00*64)},
    {83, (int)(1.00*64), (int)(1.00*64)},
    {86, (int)(1.00*64), (int)(1.00*64)},
    {89, (int)(1.00*64), (int)(1.00*64)},
    {92, (int)(1.00*64), (int)(1.00*64)},
    {95, (int)(1.00*64), (int)(1.00*64)},
    {99, (int)(1.00*64), (int)(1.00*64)},
    {102, (int)(1.00*64), (int)(1.00*64)},
    {106, (int)(1.00*64), (int)(1.00*64)},
    {110, (int)(1.00*64), (int)(1.00*64)},
    {113, (int)(1.00*64), (int)(1.00*64)},
    {117, (int)(1.00*64), (int)(1.00*64)},
    {122, (int)(1.00*64), (int)(1.00*64)},
    {126, (int)(1.00*64), (int)(1.00*64)},
    {130, (int)(1.00*64), (int)(1.00*64)},
    {135, (int)(1.00*64), (int)(1.00*64)},
    {140, (int)(1.00*64), (int)(1.00*64)},
    {145, (int)(1.00*64), (int)(1.00*64)},
    {150, (int)(1.00*64), (int)(1.00*64)},
    {155, (int)(1.00*64), (int)(1.00*64)},
    {160, (int)(1.00*64), (int)(1.00*64)},
    {166, (int)(1.00*64), (int)(1.00*64)},
    {172, (int)(1.00*64), (int)(1.00*64)},
    {178, (int)(1.00*64), (int)(1.00*64)},
    {184, (int)(1.00*64), (int)(1.00*64)},
    {191, (int)(1.00*64), (int)(1.00*64)},
    {198, (int)(1.00*64), (int)(1.00*64)},
    {205, (int)(1.00*64), (int)(1.00*64)},
    {212, (int)(1.00*64), (int)(1.00*64)},
    {219, (int)(1.00*64), (int)(1.00*64)},
    {227, (int)(1.00*64), (int)(1.00*64)},
    {235, (int)(1.00*64), (int)(1.00*64)},
    {243, (int)(1.00*64), (int)(1.00*64)},
    {252, (int)(1.00*64), (int)(1.00*64)},
    {261, (int)(1.00*64), (int)(1.00*64)},
    {270, (int)(1.00*64), (int)(1.00*64)},
    {279, (int)(1.00*64), (int)(1.00*64)},
    {289, (int)(1.00*64), (int)(1.00*64)},
    {299, (int)(1.00*64), (int)(1.00*64)},
    {310, (int)(1.00*64), (int)(1.00*64)},
    {321, (int)(1.00*64), (int)(1.00*64)},
    {332, (int)(1.00*64), (int)(1.00*64)},
    {344, (int)(1.00*64), (int)(1.00*64)},
    {356, (int)(1.00*64), (int)(1.00*64)},
    {369, (int)(1.00*64), (int)(1.00*64)},
    {382, (int)(1.00*64), (int)(1.00*64)},
    {395, (int)(1.00*64), (int)(1.00*64)},
    {409, (int)(1.00*64), (int)(1.00*64)},
    {423, (int)(1.00*64), (int)(1.00*64)},
    {438, (int)(1.00*64), (int)(1.00*64)},
    {454, (int)(1.00*64), (int)(1.00*64)},
    {470, (int)(1.00*64), (int)(1.00*64)},
    {486, (int)(1.00*64), (int)(1.00*64)},
    {504, (int)(1.00*64), (int)(1.00*64)},
    {521, (int)(1.00*64), (int)(1.00*64)},
    {540, (int)(1.00*64), (int)(1.00*64)},
    {559, (int)(1.00*64), (int)(1.00*64)},
    {578, (int)(1.00*64), (int)(1.00*64)},
    {599, (int)(1.00*64), (int)(1.00*64)},
    {620, (int)(1.00*64), (int)(1.00*64)},
    {642, (int)(1.00*64), (int)(1.00*64)},
    {665, (int)(1.00*64), (int)(1.00*64)},
    {688, (int)(1.00*64), (int)(1.00*64)},
    {712, (int)(1.00*64), (int)(1.00*64)},
    {737, (int)(1.00*64), (int)(1.00*64)},
    {763, (int)(1.00*64), (int)(1.00*64)},
    {790, (int)(1.00*64), (int)(1.00*64)},
    {818, (int)(1.00*64), (int)(1.00*64)},
    {847, (int)(1.00*64), (int)(1.00*64)},
    {877, (int)(1.00*64), (int)(1.00*64)},
    {908, (int)(1.00*64), (int)(1.00*64)},
    {940, (int)(1.00*64), (int)(1.00*64)},
    {973, (int)(1.00*64), (int)(1.00*64)},
    {1007, (int)(1.00*64), (int)(1.00*64)},
    {1043, (int)(1.00*64), (int)(1.00*64)},
    {1079, (int)(1.00*64), (int)(1.00*64)},
    {1118, (int)(1.00*64), (int)(1.00*64)},
    {1157, (int)(1.00*64), (int)(1.00*64)},
    {1198, (int)(1.00*64), (int)(1.00*64)},
    {1240, (int)(1.00*64), (int)(1.00*64)},
    {1284, (int)(1.00*64), (int)(1.00*64)},
    {1329, (int)(1.00*64), (int)(1.00*64)},
    {1329, (int)(1.04*64), (int)(1.00*64)},
    {1329, (int)(1.07*64), (int)(1.00*64)},
    {1329, (int)(1.11*64), (int)(1.00*64)},
    {1329, (int)(1.15*64), (int)(1.00*64)},
    {1329, (int)(1.19*64), (int)(1.00*64)},
    {1329, (int)(1.23*64), (int)(1.00*64)},
    {1329, (int)(1.27*64), (int)(1.00*64)},
    {1329, (int)(1.32*64), (int)(1.00*64)},
    {1329, (int)(1.37*64), (int)(1.00*64)},
    {1329, (int)(1.41*64), (int)(1.00*64)},
    {1329, (int)(1.46*64), (int)(1.00*64)},
    {1329, (int)(1.52*64), (int)(1.00*64)},
    {1329, (int)(1.57*64), (int)(1.00*64)},
    {1329, (int)(1.62*64), (int)(1.00*64)},
    {1329, (int)(1.68*64), (int)(1.00*64)},
    {1329, (int)(1.74*64), (int)(1.00*64)},
    {1329, (int)(1.80*64), (int)(1.00*64)},
    {1329, (int)(1.87*64), (int)(1.00*64)},
    {1329, (int)(1.93*64), (int)(1.00*64)},
    {1329, (int)(2.00*64), (int)(1.00*64)},
    {1329, (int)(2.07*64), (int)(1.00*64)},
    {1329, (int)(2.14*64), (int)(1.00*64)},
    {1329, (int)(2.22*64), (int)(1.00*64)},
    {1329, (int)(2.30*64), (int)(1.00*64)},
    {1329, (int)(2.38*64), (int)(1.00*64)},
    {1329, (int)(2.46*64), (int)(1.00*64)},
    {1329, (int)(2.55*64), (int)(1.00*64)},
    {1329, (int)(2.64*64), (int)(1.00*64)},
    {1329, (int)(2.73*64), (int)(1.00*64)},
    {1329, (int)(2.83*64), (int)(1.00*64)},
    {1329, (int)(2.93*64), (int)(1.00*64)},
    {1329, (int)(3.03*64), (int)(1.00*64)},
    {1329, (int)(3.14*64), (int)(1.00*64)},
    {1329, (int)(3.25*64), (int)(1.00*64)},
    {1329, (int)(3.36*64), (int)(1.00*64)},
    {1329, (int)(3.48*64), (int)(1.00*64)},
    {1329, (int)(3.61*64), (int)(1.00*64)},
    {1329, (int)(3.73*64), (int)(1.00*64)},
    {1329, (int)(3.86*64), (int)(1.00*64)},
    {1329, (int)(4.00*64), (int)(1.00*64)},
    {1329, (int)(4.14*64), (int)(1.00*64)},
    {1329, (int)(4.29*64), (int)(1.00*64)},
    {1329, (int)(4.44*64), (int)(1.00*64)},
    {1329, (int)(4.59*64), (int)(1.00*64)},
    {1329, (int)(4.76*64), (int)(1.00*64)},
    {1329, (int)(4.92*64), (int)(1.00*64)},
    {1329, (int)(5.10*64), (int)(1.00*64)},
    {1329, (int)(5.28*64), (int)(1.00*64)},
    {1329, (int)(5.46*64), (int)(1.00*64)},
    {1329, (int)(5.66*64), (int)(1.00*64)},
    {1329, (int)(5.86*64), (int)(1.00*64)},
    {1329, (int)(6.06*64), (int)(1.00*64)},
    {1329, (int)(6.28*64), (int)(1.00*64)},
    {1329, (int)(6.50*64), (int)(1.00*64)},
    {1329, (int)(6.73*64), (int)(1.00*64)},
    {1329, (int)(6.96*64), (int)(1.00*64)},
    {1329, (int)(7.21*64), (int)(1.00*64)},
    {1329, (int)(7.46*64), (int)(1.00*64)},
    {1329, (int)(7.73*64), (int)(1.00*64)},
    {1329, (int)(8.00*64), (int)(1.00*64)},
    {1329, (int)(8.28*64), (int)(1.00*64)},
    {1329, (int)(8.57*64), (int)(1.00*64)},
    {1329, (int)(8.88*64), (int)(1.00*64)},
    {1329, (int)(9.19*64), (int)(1.00*64)},
    {1329, (int)(9.51*64), (int)(1.00*64)},
    {1329, (int)(9.85*64), (int)(1.00*64)},
    {1329, (int)(10.20*64), (int)(1.00*64)},
    {1329, (int)(10.56*64), (int)(1.00*64)},
    {1329, (int)(10.93*64), (int)(1.00*64)},
    {1329, (int)(11.31*64), (int)(1.00*64)},
    {1329, (int)(11.71*64), (int)(1.00*64)},
    {1329, (int)(12.13*64), (int)(1.00*64)},
    {1329, (int)(12.55*64), (int)(1.00*64)},
    {1329, (int)(13.00*64), (int)(1.00*64)},
    {1329, (int)(13.45*64), (int)(1.00*64)},
    {1329, (int)(13.93*64), (int)(1.00*64)},
    {1329, (int)(14.42*64), (int)(1.00*64)},
    {1329, (int)(14.93*64), (int)(1.00*64)},
    {1329, (int)(15.45*64), (int)(1.00*64)},
    {1329, (int)(16.00*64), (int)(1.00*64)},
}; // 60Hz

static const int imx577_25fps_exp_time_gain_50Hz[IMX577_25FPS_50HZ_EXP_TIME_TOTAL][3] =
{// {time, analog gain, digital gain}
    {2, (int)(1.00*64), (int)(1.00*64)},
    {2, (int)(1.04*64), (int)(1.00*64)},
    {2, (int)(1.07*64), (int)(1.00*64)},
    {2, (int)(1.11*64), (int)(1.00*64)},
    {2, (int)(1.15*64), (int)(1.00*64)},
    {2, (int)(1.19*64), (int)(1.00*64)},
    {2, (int)(1.23*64), (int)(1.00*64)},
    {2, (int)(1.27*64), (int)(1.00*64)},
    {2, (int)(1.32*64), (int)(1.00*64)},
    {2, (int)(1.37*64), (int)(1.00*64)},
    {2, (int)(1.41*64), (int)(1.00*64)},
    {2, (int)(1.46*64), (int)(1.00*64)},
    {3, (int)(1.00*64), (int)(1.00*64)},
    {3, (int)(1.04*64), (int)(1.00*64)},
    {3, (int)(1.07*64), (int)(1.00*64)},
    {3, (int)(1.11*64), (int)(1.00*64)},
    {3, (int)(1.15*64), (int)(1.00*64)},
    {3, (int)(1.19*64), (int)(1.00*64)},
    {3, (int)(1.23*64), (int)(1.00*64)},
    {3, (int)(1.27*64), (int)(1.00*64)},
    {3, (int)(1.32*64), (int)(1.00*64)},
    {3, (int)(1.37*64), (int)(1.00*64)},
    {3, (int)(1.41*64), (int)(1.00*64)},
    {3, (int)(1.46*64), (int)(1.00*64)},
    {3, (int)(1.52*64), (int)(1.00*64)},
    {3, (int)(1.57*64), (int)(1.00*64)},
    {3, (int)(1.62*64), (int)(1.00*64)},
    {3, (int)(1.68*64), (int)(1.00*64)},
    {3, (int)(1.74*64), (int)(1.00*64)},
    {3, (int)(1.80*64), (int)(1.00*64)},
    {3, (int)(1.87*64), (int)(1.00*64)},
    {3, (int)(1.93*64), (int)(1.00*64)},
    {7, (int)(1.00*64), (int)(1.00*64)},
    {7, (int)(1.04*64), (int)(1.00*64)},
    {7, (int)(1.07*64), (int)(1.00*64)},
    {7, (int)(1.11*64), (int)(1.00*64)},
    {7, (int)(1.15*64), (int)(1.00*64)},
    {7, (int)(1.19*64), (int)(1.00*64)},
    {7, (int)(1.23*64), (int)(1.00*64)},
    {7, (int)(1.27*64), (int)(1.00*64)},
    {7, (int)(1.32*64), (int)(1.00*64)},
    {7, (int)(1.37*64), (int)(1.00*64)},
    {7, (int)(1.41*64), (int)(1.00*64)},
    {7, (int)(1.46*64), (int)(1.00*64)},
    {7, (int)(1.52*64), (int)(1.00*64)},
    {7, (int)(1.57*64), (int)(1.00*64)},
    {7, (int)(1.62*64), (int)(1.00*64)},
    {7, (int)(1.68*64), (int)(1.00*64)},
    {7, (int)(1.74*64), (int)(1.00*64)},
    {7, (int)(1.80*64), (int)(1.00*64)},
    {7, (int)(1.87*64), (int)(1.00*64)},
    {7, (int)(1.93*64), (int)(1.00*64)},
    {14, (int)(1.00*64), (int)(1.00*64)},
    {14, (int)(1.04*64), (int)(1.00*64)},
    {14, (int)(1.07*64), (int)(1.00*64)},
    {14, (int)(1.11*64), (int)(1.00*64)},
    {14, (int)(1.15*64), (int)(1.00*64)},
    {14, (int)(1.19*64), (int)(1.00*64)},
    {14, (int)(1.23*64), (int)(1.00*64)},
    {14, (int)(1.27*64), (int)(1.00*64)},
    {14, (int)(1.32*64), (int)(1.00*64)},
    {14, (int)(1.37*64), (int)(1.00*64)},
    {14, (int)(1.41*64), (int)(1.00*64)},
    {14, (int)(1.46*64), (int)(1.00*64)},
    {14, (int)(1.52*64), (int)(1.00*64)},
    {14, (int)(1.57*64), (int)(1.00*64)},
    {14, (int)(1.62*64), (int)(1.00*64)},
    {14, (int)(1.68*64), (int)(1.00*64)},
    {14, (int)(1.74*64), (int)(1.00*64)},
    {14, (int)(1.80*64), (int)(1.00*64)},
    {14, (int)(1.87*64), (int)(1.00*64)},
    {27, (int)(1.00*64), (int)(1.00*64)},
    {27, (int)(1.00*64), (int)(1.00*64)},
    {28, (int)(1.00*64), (int)(1.00*64)},
    {29, (int)(1.00*64), (int)(1.00*64)},
    {30, (int)(1.00*64), (int)(1.00*64)},
    {31, (int)(1.00*64), (int)(1.00*64)},
    {32, (int)(1.00*64), (int)(1.00*64)},
    {33, (int)(1.00*64), (int)(1.00*64)},
    {35, (int)(1.00*64), (int)(1.00*64)},
    {36, (int)(1.00*64), (int)(1.00*64)},
    {37, (int)(1.00*64), (int)(1.00*64)},
    {38, (int)(1.00*64), (int)(1.00*64)},
    {40, (int)(1.00*64), (int)(1.00*64)},
    {41, (int)(1.00*64), (int)(1.00*64)},
    {43, (int)(1.00*64), (int)(1.00*64)},
    {44, (int)(1.00*64), (int)(1.00*64)},
    {46, (int)(1.00*64), (int)(1.00*64)},
    {47, (int)(1.00*64), (int)(1.00*64)},
    {49, (int)(1.00*64), (int)(1.00*64)},
    {51, (int)(1.00*64), (int)(1.00*64)},
    {53, (int)(1.00*64), (int)(1.00*64)},
    {54, (int)(1.00*64), (int)(1.00*64)},
    {56, (int)(1.00*64), (int)(1.00*64)},
    {58, (int)(1.00*64), (int)(1.00*64)},
    {60, (int)(1.00*64), (int)(1.00*64)},
    {62, (int)(1.00*64), (int)(1.00*64)},
    {65, (int)(1.00*64), (int)(1.00*64)},
    {67, (int)(1.00*64), (int)(1.00*64)},
    {69, (int)(1.00*64), (int)(1.00*64)},
    {72, (int)(1.00*64), (int)(1.00*64)},
    {74, (int)(1.00*64), (int)(1.00*64)},
    {77, (int)(1.00*64), (int)(1.00*64)},
    {80, (int)(1.00*64), (int)(1.00*64)},
    {82, (int)(1.00*64), (int)(1.00*64)},
    {85, (int)(1.00*64), (int)(1.00*64)},
    {88, (int)(1.00*64), (int)(1.00*64)},
    {91, (int)(1.00*64), (int)(1.00*64)},
    {95, (int)(1.00*64), (int)(1.00*64)},
    {98, (int)(1.00*64), (int)(1.00*64)},
    {101, (int)(1.00*64), (int)(1.00*64)},
    {105, (int)(1.00*64), (int)(1.00*64)},
    {109, (int)(1.00*64), (int)(1.00*64)},
    {113, (int)(1.00*64), (int)(1.00*64)},
    {117, (int)(1.00*64), (int)(1.00*64)},
    {121, (int)(1.00*64), (int)(1.00*64)},
    {125, (int)(1.00*64), (int)(1.00*64)},
    {129, (int)(1.00*64), (int)(1.00*64)},
    {134, (int)(1.00*64), (int)(1.00*64)},
    {139, (int)(1.00*64), (int)(1.00*64)},
    {143, (int)(1.00*64), (int)(1.00*64)},
    {148, (int)(1.00*64), (int)(1.00*64)},
    {154, (int)(1.00*64), (int)(1.00*64)},
    {159, (int)(1.00*64), (int)(1.00*64)},
    {165, (int)(1.00*64), (int)(1.00*64)},
    {171, (int)(1.00*64), (int)(1.00*64)},
    {177, (int)(1.00*64), (int)(1.00*64)},
    {183, (int)(1.00*64), (int)(1.00*64)},
    {189, (int)(1.00*64), (int)(1.00*64)},
    {196, (int)(1.00*64), (int)(1.00*64)},
    {203, (int)(1.00*64), (int)(1.00*64)},
    {210, (int)(1.00*64), (int)(1.00*64)},
    {217, (int)(1.00*64), (int)(1.00*64)},
    {225, (int)(1.00*64), (int)(1.00*64)},
    {233, (int)(1.00*64), (int)(1.00*64)},
    {241, (int)(1.00*64), (int)(1.00*64)},
    {250, (int)(1.00*64), (int)(1.00*64)},
    {259, (int)(1.00*64), (int)(1.00*64)},
    {268, (int)(1.00*64), (int)(1.00*64)},
    {277, (int)(1.00*64), (int)(1.00*64)},
    {287, (int)(1.00*64), (int)(1.00*64)},
    {297, (int)(1.00*64), (int)(1.00*64)},
    {307, (int)(1.00*64), (int)(1.00*64)},
    {318, (int)(1.00*64), (int)(1.00*64)},
    {330, (int)(1.00*64), (int)(1.00*64)},
    {341, (int)(1.00*64), (int)(1.00*64)},
    {353, (int)(1.00*64), (int)(1.00*64)},
    {366, (int)(1.00*64), (int)(1.00*64)},
    {379, (int)(1.00*64), (int)(1.00*64)},
    {392, (int)(1.00*64), (int)(1.00*64)},
    {406, (int)(1.00*64), (int)(1.00*64)},
    {420, (int)(1.00*64), (int)(1.00*64)},
    {435, (int)(1.00*64), (int)(1.00*64)},
    {450, (int)(1.00*64), (int)(1.00*64)},
    {466, (int)(1.00*64), (int)(1.00*64)},
    {482, (int)(1.00*64), (int)(1.00*64)},
    {499, (int)(1.00*64), (int)(1.00*64)},
    {517, (int)(1.00*64), (int)(1.00*64)},
    {535, (int)(1.00*64), (int)(1.00*64)},
    {554, (int)(1.00*64), (int)(1.00*64)},
    {574, (int)(1.00*64), (int)(1.00*64)},
    {594, (int)(1.00*64), (int)(1.00*64)},
    {615, (int)(1.00*64), (int)(1.00*64)},
    {637, (int)(1.00*64), (int)(1.00*64)},
    {659, (int)(1.00*64), (int)(1.00*64)},
    {682, (int)(1.00*64), (int)(1.00*64)},
    {706, (int)(1.00*64), (int)(1.00*64)},
    {731, (int)(1.00*64), (int)(1.00*64)},
    {757, (int)(1.00*64), (int)(1.00*64)},
    {784, (int)(1.00*64), (int)(1.00*64)},
    {811, (int)(1.00*64), (int)(1.00*64)},
    {840, (int)(1.00*64), (int)(1.00*64)},
    {840, (int)(1.04*64), (int)(1.00*64)},
    {840, (int)(1.07*64), (int)(1.00*64)},
    {840, (int)(1.11*64), (int)(1.00*64)},
    {840, (int)(1.15*64), (int)(1.00*64)},
    {840, (int)(1.19*64), (int)(1.00*64)},
    {840, (int)(1.23*64), (int)(1.00*64)},
    {840, (int)(1.27*64), (int)(1.00*64)},
    {840, (int)(1.32*64), (int)(1.00*64)},
    {840, (int)(1.37*64), (int)(1.00*64)},
    {840, (int)(1.41*64), (int)(1.00*64)},
    {840, (int)(1.46*64), (int)(1.00*64)},
    {840, (int)(1.52*64), (int)(1.00*64)},
    {840, (int)(1.57*64), (int)(1.00*64)},
    {840, (int)(1.62*64), (int)(1.00*64)},
    {840, (int)(1.68*64), (int)(1.00*64)},
    {840, (int)(1.74*64), (int)(1.00*64)},
    {840, (int)(1.80*64), (int)(1.00*64)},
    {840, (int)(1.87*64), (int)(1.00*64)},
    {840, (int)(1.93*64), (int)(1.00*64)},
    {1680, (int)(1.00*64), (int)(1.00*64)},
    {1680, (int)(1.04*64), (int)(1.00*64)},
    {1680, (int)(1.07*64), (int)(1.00*64)},
    {1680, (int)(1.11*64), (int)(1.00*64)},
    {1680, (int)(1.15*64), (int)(1.00*64)},
    {1680, (int)(1.19*64), (int)(1.00*64)},
    {1680, (int)(1.23*64), (int)(1.00*64)},
    {1680, (int)(1.27*64), (int)(1.00*64)},
    {1680, (int)(1.32*64), (int)(1.00*64)},
    {1680, (int)(1.37*64), (int)(1.00*64)},
    {1680, (int)(1.41*64), (int)(1.00*64)},
    {1680, (int)(1.46*64), (int)(1.00*64)},
    {2520, (int)(1.00*64), (int)(1.00*64)},
    {2520, (int)(1.04*64), (int)(1.00*64)},
    {2520, (int)(1.07*64), (int)(1.00*64)},
    {2520, (int)(1.11*64), (int)(1.00*64)},
    {2520, (int)(1.15*64), (int)(1.00*64)},
    {2520, (int)(1.19*64), (int)(1.00*64)},
    {2520, (int)(1.23*64), (int)(1.00*64)},
    {2520, (int)(1.27*64), (int)(1.00*64)},
    {2520, (int)(1.32*64), (int)(1.00*64)},
    {3360, (int)(1.00*64), (int)(1.00*64)},
    {3360, (int)(1.04*64), (int)(1.00*64)},
    {3360, (int)(1.07*64), (int)(1.00*64)},
    {3360, (int)(1.11*64), (int)(1.00*64)},
    {3360, (int)(1.15*64), (int)(1.00*64)},
    {3360, (int)(1.19*64), (int)(1.00*64)},
    {3360, (int)(1.23*64), (int)(1.00*64)},
    {3360, (int)(1.27*64), (int)(1.00*64)},
    {3360, (int)(1.32*64), (int)(1.00*64)},
    {3360, (int)(1.37*64), (int)(1.00*64)},
    {3360, (int)(1.41*64), (int)(1.00*64)},
    {3360, (int)(1.46*64), (int)(1.00*64)},
    {3360, (int)(1.52*64), (int)(1.00*64)},
    {3360, (int)(1.57*64), (int)(1.00*64)},
    {3360, (int)(1.62*64), (int)(1.00*64)},
    {3360, (int)(1.68*64), (int)(1.00*64)},
    {3360, (int)(1.74*64), (int)(1.00*64)},
    {3360, (int)(1.80*64), (int)(1.00*64)},
    {3360, (int)(1.87*64), (int)(1.00*64)},
    {3360, (int)(1.93*64), (int)(1.00*64)},
    {3360, (int)(2.00*64), (int)(1.00*64)},
    {3360, (int)(2.07*64), (int)(1.00*64)},
    {3360, (int)(2.14*64), (int)(1.00*64)},
    {3360, (int)(2.22*64), (int)(1.00*64)},
    {3360, (int)(2.30*64), (int)(1.00*64)},
    {3360, (int)(2.38*64), (int)(1.00*64)},
    {3360, (int)(2.46*64), (int)(1.00*64)},
    {3360, (int)(2.55*64), (int)(1.00*64)},
    {3360, (int)(2.64*64), (int)(1.00*64)},
    {3360, (int)(2.73*64), (int)(1.00*64)},
    {3360, (int)(2.83*64), (int)(1.00*64)},
    {3360, (int)(2.93*64), (int)(1.00*64)},
    {3360, (int)(3.03*64), (int)(1.00*64)},
    {3360, (int)(3.14*64), (int)(1.00*64)},
    {3360, (int)(3.25*64), (int)(1.00*64)},
    {3360, (int)(3.36*64), (int)(1.00*64)},
    {3360, (int)(3.48*64), (int)(1.00*64)},
    {3360, (int)(3.61*64), (int)(1.00*64)},
    {3360, (int)(3.73*64), (int)(1.00*64)},
    {3360, (int)(3.86*64), (int)(1.00*64)},
    {3360, (int)(4.00*64), (int)(1.00*64)},
    {3360, (int)(4.14*64), (int)(1.00*64)},
    {3360, (int)(4.29*64), (int)(1.00*64)},
    {3360, (int)(4.44*64), (int)(1.00*64)},
    {3360, (int)(4.59*64), (int)(1.00*64)},
    {3360, (int)(4.76*64), (int)(1.00*64)},
    {3360, (int)(4.92*64), (int)(1.00*64)},
    {3360, (int)(5.10*64), (int)(1.00*64)},
    {3360, (int)(5.28*64), (int)(1.00*64)},
    {3360, (int)(5.46*64), (int)(1.00*64)},
    {3360, (int)(5.66*64), (int)(1.00*64)},
    {3360, (int)(5.86*64), (int)(1.00*64)},
    {3360, (int)(6.06*64), (int)(1.00*64)},
    {3360, (int)(6.28*64), (int)(1.00*64)},
    {3360, (int)(6.50*64), (int)(1.00*64)},
    {3360, (int)(6.73*64), (int)(1.00*64)},
    {3360, (int)(6.96*64), (int)(1.00*64)},
    {3360, (int)(7.21*64), (int)(1.00*64)},
    {3360, (int)(7.46*64), (int)(1.00*64)},
    {3360, (int)(7.73*64), (int)(1.00*64)},
    {3360, (int)(8.00*64), (int)(1.00*64)},
    {3360, (int)(8.28*64), (int)(1.00*64)},
    {3360, (int)(8.57*64), (int)(1.00*64)},
    {3360, (int)(8.88*64), (int)(1.00*64)},
    {3360, (int)(9.19*64), (int)(1.00*64)},
    {3360, (int)(9.51*64), (int)(1.00*64)},
    {3360, (int)(9.85*64), (int)(1.00*64)},
    {3360, (int)(10.20*64), (int)(1.00*64)},
    {3360, (int)(10.56*64), (int)(1.00*64)},
    {3360, (int)(10.93*64), (int)(1.00*64)},
    {3360, (int)(11.31*64), (int)(1.00*64)},
    {3360, (int)(11.71*64), (int)(1.00*64)},
    {3360, (int)(12.13*64), (int)(1.00*64)},
    {3360, (int)(12.55*64), (int)(1.00*64)},
    {3360, (int)(13.00*64), (int)(1.00*64)},
    {3360, (int)(13.45*64), (int)(1.00*64)},
    {3360, (int)(13.93*64), (int)(1.00*64)},
    {3360, (int)(14.42*64), (int)(1.00*64)},
    {3360, (int)(14.93*64), (int)(1.00*64)},
    {3360, (int)(15.45*64), (int)(1.00*64)},
    {3360, (int)(16.00*64), (int)(1.00*64)},
}; // 50Hz

static const int imx577_25fps_exp_time_gain_60Hz[IMX577_25FPS_60HZ_EXP_TIME_TOTAL][3] =
{// {time, analog gain, digital gain}
    {1, (int)(1.00*64), (int)(1.00*64)},
    {1, (int)(1.04*64), (int)(1.00*64)},
    {1, (int)(1.07*64), (int)(1.00*64)},
    {1, (int)(1.11*64), (int)(1.00*64)},
    {1, (int)(1.15*64), (int)(1.00*64)},
    {1, (int)(1.19*64), (int)(1.00*64)},
    {1, (int)(1.23*64), (int)(1.00*64)},
    {1, (int)(1.27*64), (int)(1.00*64)},
    {1, (int)(1.32*64), (int)(1.00*64)},
    {1, (int)(1.37*64), (int)(1.00*64)},
    {1, (int)(1.41*64), (int)(1.00*64)},
    {1, (int)(1.46*64), (int)(1.00*64)},
    {1, (int)(1.52*64), (int)(1.00*64)},
    {1, (int)(1.57*64), (int)(1.00*64)},
    {1, (int)(1.62*64), (int)(1.00*64)},
    {1, (int)(1.68*64), (int)(1.00*64)},
    {1, (int)(1.74*64), (int)(1.00*64)},
    {1, (int)(1.80*64), (int)(1.00*64)},
    {1, (int)(1.87*64), (int)(1.00*64)},
    {1, (int)(1.93*64), (int)(1.00*64)},
    {3, (int)(1.00*64), (int)(1.00*64)},
    {3, (int)(1.04*64), (int)(1.00*64)},
    {3, (int)(1.07*64), (int)(1.00*64)},
    {3, (int)(1.11*64), (int)(1.00*64)},
    {3, (int)(1.15*64), (int)(1.00*64)},
    {3, (int)(1.19*64), (int)(1.00*64)},
    {3, (int)(1.23*64), (int)(1.00*64)},
    {3, (int)(1.27*64), (int)(1.00*64)},
    {3, (int)(1.32*64), (int)(1.00*64)},
    {3, (int)(1.37*64), (int)(1.00*64)},
    {3, (int)(1.41*64), (int)(1.00*64)},
    {3, (int)(1.46*64), (int)(1.00*64)},
    {3, (int)(1.52*64), (int)(1.00*64)},
    {3, (int)(1.57*64), (int)(1.00*64)},
    {3, (int)(1.62*64), (int)(1.00*64)},
    {3, (int)(1.68*64), (int)(1.00*64)},
    {3, (int)(1.74*64), (int)(1.00*64)},
    {3, (int)(1.80*64), (int)(1.00*64)},
    {3, (int)(1.87*64), (int)(1.00*64)},
    {3, (int)(1.93*64), (int)(1.00*64)},
    {6, (int)(1.00*64), (int)(1.00*64)},
    {6, (int)(1.04*64), (int)(1.00*64)},
    {6, (int)(1.07*64), (int)(1.00*64)},
    {6, (int)(1.11*64), (int)(1.00*64)},
    {6, (int)(1.15*64), (int)(1.00*64)},
    {6, (int)(1.19*64), (int)(1.00*64)},
    {6, (int)(1.23*64), (int)(1.00*64)},
    {6, (int)(1.27*64), (int)(1.00*64)},
    {6, (int)(1.32*64), (int)(1.00*64)},
    {6, (int)(1.37*64), (int)(1.00*64)},
    {6, (int)(1.41*64), (int)(1.00*64)},
    {6, (int)(1.46*64), (int)(1.00*64)},
    {6, (int)(1.52*64), (int)(1.00*64)},
    {6, (int)(1.57*64), (int)(1.00*64)},
    {6, (int)(1.62*64), (int)(1.00*64)},
    {6, (int)(1.68*64), (int)(1.00*64)},
    {6, (int)(1.74*64), (int)(1.00*64)},
    {6, (int)(1.80*64), (int)(1.00*64)},
    {11, (int)(1.00*64), (int)(1.00*64)},
    {11, (int)(1.04*64), (int)(1.00*64)},
    {11, (int)(1.07*64), (int)(1.00*64)},
    {11, (int)(1.11*64), (int)(1.00*64)},
    {11, (int)(1.15*64), (int)(1.00*64)},
    {11, (int)(1.19*64), (int)(1.00*64)},
    {11, (int)(1.23*64), (int)(1.00*64)},
    {11, (int)(1.27*64), (int)(1.00*64)},
    {11, (int)(1.32*64), (int)(1.00*64)},
    {11, (int)(1.37*64), (int)(1.00*64)},
    {11, (int)(1.41*64), (int)(1.00*64)},
    {11, (int)(1.46*64), (int)(1.00*64)},
    {11, (int)(1.52*64), (int)(1.00*64)},
    {11, (int)(1.57*64), (int)(1.00*64)},
    {11, (int)(1.62*64), (int)(1.00*64)},
    {11, (int)(1.68*64), (int)(1.00*64)},
    {11, (int)(1.74*64), (int)(1.00*64)},
    {11, (int)(1.80*64), (int)(1.00*64)},
    {11, (int)(1.87*64), (int)(1.00*64)},
    {11, (int)(1.93*64), (int)(1.00*64)},
    {23, (int)(1.00*64), (int)(1.00*64)},
    {23, (int)(1.00*64), (int)(1.00*64)},
    {23, (int)(1.00*64), (int)(1.00*64)},
    {24, (int)(1.00*64), (int)(1.00*64)},
    {25, (int)(1.00*64), (int)(1.00*64)},
    {26, (int)(1.00*64), (int)(1.00*64)},
    {27, (int)(1.00*64), (int)(1.00*64)},
    {28, (int)(1.00*64), (int)(1.00*64)},
    {29, (int)(1.00*64), (int)(1.00*64)},
    {30, (int)(1.00*64), (int)(1.00*64)},
    {31, (int)(1.00*64), (int)(1.00*64)},
    {32, (int)(1.00*64), (int)(1.00*64)},
    {33, (int)(1.00*64), (int)(1.00*64)},
    {34, (int)(1.00*64), (int)(1.00*64)},
    {36, (int)(1.00*64), (int)(1.00*64)},
    {37, (int)(1.00*64), (int)(1.00*64)},
    {38, (int)(1.00*64), (int)(1.00*64)},
    {39, (int)(1.00*64), (int)(1.00*64)},
    {41, (int)(1.00*64), (int)(1.00*64)},
    {42, (int)(1.00*64), (int)(1.00*64)},
    {44, (int)(1.00*64), (int)(1.00*64)},
    {45, (int)(1.00*64), (int)(1.00*64)},
    {47, (int)(1.00*64), (int)(1.00*64)},
    {49, (int)(1.00*64), (int)(1.00*64)},
    {50, (int)(1.00*64), (int)(1.00*64)},
    {52, (int)(1.00*64), (int)(1.00*64)},
    {54, (int)(1.00*64), (int)(1.00*64)},
    {56, (int)(1.00*64), (int)(1.00*64)},
    {58, (int)(1.00*64), (int)(1.00*64)},
    {60, (int)(1.00*64), (int)(1.00*64)},
    {62, (int)(1.00*64), (int)(1.00*64)},
    {64, (int)(1.00*64), (int)(1.00*64)},
    {66, (int)(1.00*64), (int)(1.00*64)},
    {69, (int)(1.00*64), (int)(1.00*64)},
    {71, (int)(1.00*64), (int)(1.00*64)},
    {74, (int)(1.00*64), (int)(1.00*64)},
    {76, (int)(1.00*64), (int)(1.00*64)},
    {79, (int)(1.00*64), (int)(1.00*64)},
    {82, (int)(1.00*64), (int)(1.00*64)},
    {85, (int)(1.00*64), (int)(1.00*64)},
    {88, (int)(1.00*64), (int)(1.00*64)},
    {91, (int)(1.00*64), (int)(1.00*64)},
    {94, (int)(1.00*64), (int)(1.00*64)},
    {97, (int)(1.00*64), (int)(1.00*64)},
    {101, (int)(1.00*64), (int)(1.00*64)},
    {104, (int)(1.00*64), (int)(1.00*64)},
    {108, (int)(1.00*64), (int)(1.00*64)},
    {112, (int)(1.00*64), (int)(1.00*64)},
    {115, (int)(1.00*64), (int)(1.00*64)},
    {120, (int)(1.00*64), (int)(1.00*64)},
    {124, (int)(1.00*64), (int)(1.00*64)},
    {128, (int)(1.00*64), (int)(1.00*64)},
    {133, (int)(1.00*64), (int)(1.00*64)},
    {137, (int)(1.00*64), (int)(1.00*64)},
    {142, (int)(1.00*64), (int)(1.00*64)},
    {147, (int)(1.00*64), (int)(1.00*64)},
    {152, (int)(1.00*64), (int)(1.00*64)},
    {158, (int)(1.00*64), (int)(1.00*64)},
    {163, (int)(1.00*64), (int)(1.00*64)},
    {169, (int)(1.00*64), (int)(1.00*64)},
    {175, (int)(1.00*64), (int)(1.00*64)},
    {181, (int)(1.00*64), (int)(1.00*64)},
    {188, (int)(1.00*64), (int)(1.00*64)},
    {194, (int)(1.00*64), (int)(1.00*64)},
    {201, (int)(1.00*64), (int)(1.00*64)},
    {208, (int)(1.00*64), (int)(1.00*64)},
    {215, (int)(1.00*64), (int)(1.00*64)},
    {223, (int)(1.00*64), (int)(1.00*64)},
    {231, (int)(1.00*64), (int)(1.00*64)},
    {239, (int)(1.00*64), (int)(1.00*64)},
    {247, (int)(1.00*64), (int)(1.00*64)},
    {256, (int)(1.00*64), (int)(1.00*64)},
    {265, (int)(1.00*64), (int)(1.00*64)},
    {275, (int)(1.00*64), (int)(1.00*64)},
    {284, (int)(1.00*64), (int)(1.00*64)},
    {294, (int)(1.00*64), (int)(1.00*64)},
    {305, (int)(1.00*64), (int)(1.00*64)},
    {315, (int)(1.00*64), (int)(1.00*64)},
    {327, (int)(1.00*64), (int)(1.00*64)},
    {338, (int)(1.00*64), (int)(1.00*64)},
    {350, (int)(1.00*64), (int)(1.00*64)},
    {362, (int)(1.00*64), (int)(1.00*64)},
    {375, (int)(1.00*64), (int)(1.00*64)},
    {388, (int)(1.00*64), (int)(1.00*64)},
    {402, (int)(1.00*64), (int)(1.00*64)},
    {416, (int)(1.00*64), (int)(1.00*64)},
    {431, (int)(1.00*64), (int)(1.00*64)},
    {446, (int)(1.00*64), (int)(1.00*64)},
    {462, (int)(1.00*64), (int)(1.00*64)},
    {478, (int)(1.00*64), (int)(1.00*64)},
    {495, (int)(1.00*64), (int)(1.00*64)},
    {512, (int)(1.00*64), (int)(1.00*64)},
    {531, (int)(1.00*64), (int)(1.00*64)},
    {549, (int)(1.00*64), (int)(1.00*64)},
    {569, (int)(1.00*64), (int)(1.00*64)},
    {589, (int)(1.00*64), (int)(1.00*64)},
    {609, (int)(1.00*64), (int)(1.00*64)},
    {631, (int)(1.00*64), (int)(1.00*64)},
    {653, (int)(1.00*64), (int)(1.00*64)},
    {676, (int)(1.00*64), (int)(1.00*64)},
    {700, (int)(1.00*64), (int)(1.00*64)},
    {700, (int)(1.04*64), (int)(1.00*64)},
    {700, (int)(1.07*64), (int)(1.00*64)},
    {700, (int)(1.11*64), (int)(1.00*64)},
    {700, (int)(1.15*64), (int)(1.00*64)},
    {700, (int)(1.19*64), (int)(1.00*64)},
    {700, (int)(1.23*64), (int)(1.00*64)},
    {700, (int)(1.27*64), (int)(1.00*64)},
    {700, (int)(1.32*64), (int)(1.00*64)},
    {700, (int)(1.37*64), (int)(1.00*64)},
    {700, (int)(1.41*64), (int)(1.00*64)},
    {700, (int)(1.46*64), (int)(1.00*64)},
    {700, (int)(1.52*64), (int)(1.00*64)},
    {700, (int)(1.57*64), (int)(1.00*64)},
    {700, (int)(1.62*64), (int)(1.00*64)},
    {700, (int)(1.68*64), (int)(1.00*64)},
    {700, (int)(1.74*64), (int)(1.00*64)},
    {700, (int)(1.80*64), (int)(1.00*64)},
    {700, (int)(1.87*64), (int)(1.00*64)},
    {700, (int)(1.93*64), (int)(1.00*64)},
    {1400, (int)(1.00*64), (int)(1.00*64)},
    {1400, (int)(1.04*64), (int)(1.00*64)},
    {1400, (int)(1.07*64), (int)(1.00*64)},
    {1400, (int)(1.11*64), (int)(1.00*64)},
    {1400, (int)(1.15*64), (int)(1.00*64)},
    {1400, (int)(1.19*64), (int)(1.00*64)},
    {1400, (int)(1.23*64), (int)(1.00*64)},
    {1400, (int)(1.27*64), (int)(1.00*64)},
    {1400, (int)(1.32*64), (int)(1.00*64)},
    {1400, (int)(1.37*64), (int)(1.00*64)},
    {1400, (int)(1.41*64), (int)(1.00*64)},
    {1400, (int)(1.46*64), (int)(1.00*64)},
    {2100, (int)(1.00*64), (int)(1.00*64)},
    {2100, (int)(1.04*64), (int)(1.00*64)},
    {2100, (int)(1.07*64), (int)(1.00*64)},
    {2100, (int)(1.11*64), (int)(1.00*64)},
    {2100, (int)(1.15*64), (int)(1.00*64)},
    {2100, (int)(1.19*64), (int)(1.00*64)},
    {2100, (int)(1.23*64), (int)(1.00*64)},
    {2100, (int)(1.27*64), (int)(1.00*64)},
    {2100, (int)(1.32*64), (int)(1.00*64)},
    {2800, (int)(1.00*64), (int)(1.00*64)},
    {2800, (int)(1.04*64), (int)(1.00*64)},
    {2800, (int)(1.07*64), (int)(1.00*64)},
    {2800, (int)(1.11*64), (int)(1.00*64)},
    {2800, (int)(1.15*64), (int)(1.00*64)},
    {2800, (int)(1.19*64), (int)(1.00*64)},
    {2800, (int)(1.23*64), (int)(1.00*64)},
    {2800, (int)(1.27*64), (int)(1.00*64)},
    {2800, (int)(1.32*64), (int)(1.00*64)},
    {2800, (int)(1.37*64), (int)(1.00*64)},
    {2800, (int)(1.41*64), (int)(1.00*64)},
    {2800, (int)(1.46*64), (int)(1.00*64)},
    {2800, (int)(1.52*64), (int)(1.00*64)},
    {2800, (int)(1.57*64), (int)(1.00*64)},
    {2800, (int)(1.62*64), (int)(1.00*64)},
    {2800, (int)(1.68*64), (int)(1.00*64)},
    {2800, (int)(1.74*64), (int)(1.00*64)},
    {2800, (int)(1.80*64), (int)(1.00*64)},
    {2800, (int)(1.87*64), (int)(1.00*64)},
    {2800, (int)(1.93*64), (int)(1.00*64)},
    {2800, (int)(2.00*64), (int)(1.00*64)},
    {2800, (int)(2.07*64), (int)(1.00*64)},
    {2800, (int)(2.14*64), (int)(1.00*64)},
    {2800, (int)(2.22*64), (int)(1.00*64)},
    {2800, (int)(2.30*64), (int)(1.00*64)},
    {2800, (int)(2.38*64), (int)(1.00*64)},
    {2800, (int)(2.46*64), (int)(1.00*64)},
    {2800, (int)(2.55*64), (int)(1.00*64)},
    {2800, (int)(2.64*64), (int)(1.00*64)},
    {2800, (int)(2.73*64), (int)(1.00*64)},
    {2800, (int)(2.83*64), (int)(1.00*64)},
    {2800, (int)(2.93*64), (int)(1.00*64)},
    {2800, (int)(3.03*64), (int)(1.00*64)},
    {2800, (int)(3.14*64), (int)(1.00*64)},
    {2800, (int)(3.25*64), (int)(1.00*64)},
    {2800, (int)(3.36*64), (int)(1.00*64)},
    {2800, (int)(3.48*64), (int)(1.00*64)},
    {2800, (int)(3.61*64), (int)(1.00*64)},
    {2800, (int)(3.73*64), (int)(1.00*64)},
    {2800, (int)(3.86*64), (int)(1.00*64)},
    {2800, (int)(4.00*64), (int)(1.00*64)},
    {2800, (int)(4.14*64), (int)(1.00*64)},
    {2800, (int)(4.29*64), (int)(1.00*64)},
    {2800, (int)(4.44*64), (int)(1.00*64)},
    {2800, (int)(4.59*64), (int)(1.00*64)},
    {2800, (int)(4.76*64), (int)(1.00*64)},
    {2800, (int)(4.92*64), (int)(1.00*64)},
    {2800, (int)(5.10*64), (int)(1.00*64)},
    {2800, (int)(5.28*64), (int)(1.00*64)},
    {2800, (int)(5.46*64), (int)(1.00*64)},
    {2800, (int)(5.66*64), (int)(1.00*64)},
    {2800, (int)(5.86*64), (int)(1.00*64)},
    {2800, (int)(6.06*64), (int)(1.00*64)},
    {2800, (int)(6.28*64), (int)(1.00*64)},
    {2800, (int)(6.50*64), (int)(1.00*64)},
    {2800, (int)(6.73*64), (int)(1.00*64)},
    {2800, (int)(6.96*64), (int)(1.00*64)},
    {2800, (int)(7.21*64), (int)(1.00*64)},
    {2800, (int)(7.46*64), (int)(1.00*64)},
    {2800, (int)(7.73*64), (int)(1.00*64)},
    {2800, (int)(8.00*64), (int)(1.00*64)},
    {2800, (int)(8.28*64), (int)(1.00*64)},
    {2800, (int)(8.57*64), (int)(1.00*64)},
    {2800, (int)(8.88*64), (int)(1.00*64)},
    {2800, (int)(9.19*64), (int)(1.00*64)},
    {2800, (int)(9.51*64), (int)(1.00*64)},
    {2800, (int)(9.85*64), (int)(1.00*64)},
    {2800, (int)(10.20*64), (int)(1.00*64)},
    {2800, (int)(10.56*64), (int)(1.00*64)},
    {2800, (int)(10.93*64), (int)(1.00*64)},
    {2800, (int)(11.31*64), (int)(1.00*64)},
    {2800, (int)(11.71*64), (int)(1.00*64)},
    {2800, (int)(12.13*64), (int)(1.00*64)},
    {2800, (int)(12.55*64), (int)(1.00*64)},
    {2800, (int)(13.00*64), (int)(1.00*64)},
    {2800, (int)(13.45*64), (int)(1.00*64)},
    {2800, (int)(13.93*64), (int)(1.00*64)},
    {2800, (int)(14.42*64), (int)(1.00*64)},
    {2800, (int)(14.93*64), (int)(1.00*64)},
    {2800, (int)(15.45*64), (int)(1.00*64)},
    {2800, (int)(16.00*64), (int)(1.00*64)},
}; // 60Hz

////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////////////////////////////
static INT32S imx577_sccb_open(void)
{
#if SCCB_MODE == SCCB_GPIO
    if(imx577_handle == NULL )
        imx577_handle = drv_l2_sccb_open_ext(&imx577_cdsp_mipi_sccb_config);
    if(imx577_handle == 0)
    {
        DBG_PRINT("Sccb open fail.\r\n");
        return STATUS_FAIL;
    }
#elif SCCB_MODE == SCCB_HW_I2C
    imx577_handle.devNumber = I2C_DEVICE_NUM;
    imx577_handle.slaveAddr = IMX577_ID;
    imx577_handle.clkRate = 200;
    drv_l1_i2c_init(imx577_handle.devNumber);
#endif
    return STATUS_OK;
}

static void imx577_sccb_close(void)
{
#if SCCB_MODE == SCCB_GPIO
    if(imx577_handle)
    {
        drv_l2_sccb_close(imx577_handle);
        imx577_handle = NULL;
    }
#elif SCCB_MODE == SCCB_HW_I2C
    drv_l1_i2c_uninit(I2C_DEVICE_NUM);
    imx577_handle.slaveAddr = 0;
    imx577_handle.clkRate = 0;
#endif
}

static INT32S imx577_sccb_write(INT16U reg, INT8U value)
{
#if SCCB_MODE == SCCB_GPIO
    return drv_l2_sccb_write(imx577_handle, reg, value);
#elif SCCB_MODE == SCCB_HW_I2C
    return drv_l1_reg_1byte_data_1byte_write(&imx577_handle, reg, value);
#endif
}

static INT32S imx577_sccb_read(INT16U reg, INT16U *value)
{
#if SCCB_MODE == SCCB_GPIO
    return drv_l2_sccb_read(imx577_handle, reg, value);
#elif SCCB_MODE == SCCB_HW_I2C
    return drv_l1_reg_1byte_data_1byte_write(&imx577_handle, reg, value);
#endif
}

static INT32S imx577_sccb_write_table(regval16_t *pTable)
{
    while(1) {
        if(pTable->reg_num == 0xFFFF && pTable->value == 0xFF) {
            break;
        }

        // DBG_PRINT("0x%04x, 0x%02x\r\n", pTable->reg_num, pTable->value);
        DBG_PRINT("*");
        if(imx577_sccb_write(pTable->reg_num, pTable->value) < 0) {
            DBG_PRINT("sccb write fail: reg = 0x%04x, val = 0x%02x\r\n", pTable->reg_num, pTable->value);
            continue;
        }
        pTable++;
    }
    DBG_PRINT("\r\n\r\n");

    return STATUS_OK;
}

#if READ_TABLE_EN == 1
static INT32S imx577_sccb_read(INT16U reg, INT8U *value)
{
    INT16U data;

    if(drv_l2_sccb_read(imx577_handle, reg, &data) >= 0) {
        *value = (INT8U)data;
        return STATUS_OK;
    } else {
        *value = 0xFF;
        return STATUS_FAIL;
    }
    return STATUS_OK;
}

static INT32S imx577_sccb_read_table(regval16_t *pTable)
{
    INT8U *Rdata;
    while(1) {
        if(pTable->reg_num == 0xFF && pTable->value == 0xFF) {
            break;
        }

        if(imx577_sccb_read(pTable->reg_num, Rdata) < 0) {
            DBG_PRINT("sccb read fail.\r\n");
            continue;
        }
        DBG_PRINT("0x%02x, 0x%02x = 0x%02x\r\n", pTable->reg_num, pTable->value, *Rdata);
        pTable++;
    }
    DBG_PRINT("\r\n\r\n");

    return STATUS_OK;
}
#endif

static int imx577_set_xfps_exposure_time(sensor_exposure_t *si)
{
    unsigned char COARSE_INTEG_TIME_15_8, COARSE_INTEG_TIME_7_0;
    unsigned char ANA_GAIN_GLOBAL_9_8, ANA_GAIN_GLOBAL_7_0;
    unsigned int idx, temp;

    si->sensor_ev_idx += si->ae_ev_step;
    if(si->sensor_ev_idx >= si->max_ev_idx) si->sensor_ev_idx = si->max_ev_idx;
    if(si->sensor_ev_idx < 0) si->sensor_ev_idx = 0;

    idx = si->sensor_ev_idx*3;

    si->time = p_expTime_table[idx];
    si->analog_gain = p_expTime_table[idx+1];
    si->digital_gain = p_expTime_table[idx+2];

    //DBG_PRINT("[EV=%d, offset=%d]: time = %d, analog gain =%d\r\n", si->sensor_ev_idx, si->ae_ev_step, si->time, si->analog_gain);

    // exposure time
    if(si->time != pre_sensor_time)
    {
        pre_sensor_time = si->time;
        temp = si->time;

        COARSE_INTEG_TIME_15_8 = (temp >> 8) & 0xff;
        COARSE_INTEG_TIME_7_0 = (temp & 0xff);

        imx577_sccb_write(0x0202, COARSE_INTEG_TIME_15_8);
        imx577_sccb_write(0x0203, COARSE_INTEG_TIME_7_0);
    }

    // gain
    if(si->analog_gain != pre_sensor_a_gain)
    {
        // gain
        pre_sensor_a_gain = si->analog_gain;

        temp = 1024 - ((1024*64)/si->analog_gain);

        ANA_GAIN_GLOBAL_9_8 = (temp >> 8) & 0x03;
        ANA_GAIN_GLOBAL_7_0 = (temp & 0xff);
        imx577_sccb_write(0x0204, ANA_GAIN_GLOBAL_9_8);
        imx577_sccb_write(0x0205, ANA_GAIN_GLOBAL_7_0);
    }

    return 0;
}

static void imx577_set_ae(int ev_step)
{
#if !FIXED_EXP_EN
    seInfo.ae_ev_step = ev_step;
    imx577_set_xfps_exposure_time(&seInfo);
#endif
}

static void sensor_get_ae_info(sensor_exposure_t *si)
{
    memcpy(si, &seInfo, sizeof(sensor_exposure_t));
}

static void sensor_set_max_lum(int max_lum)
{
    seInfo.max_ev_idx = seInfo.total_ev_idx - (128 - max_lum);
}

#if MIPI_ISR_TEST
static void mipi_imx577_handle(INT32U dev_no, INT32U event)
{
/*
    if(event & MIPI_LANE0_SOT_SYNC_ERR) {
        DBG_PRINT("LANE0_SOT_SYNC_ERR\r\n");
    }

    if(event & MIPI_HD_1BIT_ERR) {
        DBG_PRINT("HD_1BIT_ERR\r\n");
    }

    if(event & MIPI_HD_NBIT_ERR) {
        DBG_PRINT("HD_NBIT_ERR\r\n");
    }

    if(event & MIPI_DATA_CRC_ERR) {
        DBG_PRINT("DATA_CRC_ERR\r\n");
    }

    if(event & MIPI_LANE1_SOT_SYNC_ERR) {
        DBG_PRINT("LANE1_SOT_SYNC_ERR\r\n");
    }

    if(event & MIPI_CCIR_SOF) {
        DBG_PRINT("CCIR_SOF\r\n");
    }

    if(event & MIPI_EOF)
    {
        DBG_PRINT("EOF\r\n");
    }*/

    if(event & MIPI_SOF) {
        drv_l2_cdsp_handle_vd(CDSP_IDX);
        //DBG_PRINT("SOF 1\r\n");
    }
}
#endif

void imx577_mipi_set_exp_freq(INT32U freq)
{
    DBG_PRINT("imx577_mipi_set_exp_freq\r\n");
    if(freq == 50)
    {
        if(imx577_cdsp_mipi_ops.info[info_index].sensor_fps == 120)
        {
            seInfo.sensor_ev_idx = IMX577_120FPS_50HZ_INIT_EV_IDX;
            seInfo.ae_ev_step = 0;
            seInfo.daylight_ev_idx= IMX577_120FPS_50HZ_DAY_EV_IDX;
            seInfo.night_ev_idx= IMX577_120FPS_50HZ_NIGHT_EV_IDX;
            seInfo.max_ev_idx = IMX577_120FPS_50HZ_MAX_EV_IDX;
            seInfo.total_ev_idx = IMX577_120FPS_50HZ_EXP_TIME_TOTAL;

            p_expTime_table = (int *)imx577_120fps_exp_time_gain_50Hz;
            DBG_PRINT(">>Set frequence 120fps 50Hz.\r\n");
        }
        else if(imx577_cdsp_mipi_ops.info[info_index].sensor_fps == 25)
        {
            seInfo.sensor_ev_idx = IMX577_25FPS_50HZ_INIT_EV_IDX;
            seInfo.ae_ev_step = 0;
            seInfo.daylight_ev_idx= IMX577_25FPS_50HZ_DAY_EV_IDX;
            seInfo.night_ev_idx= IMX577_25FPS_50HZ_NIGHT_EV_IDX;
            seInfo.max_ev_idx = IMX577_25FPS_50HZ_MAX_EV_IDX;
            seInfo.total_ev_idx = IMX577_25FPS_50HZ_EXP_TIME_TOTAL;

            p_expTime_table = (int *)imx577_25fps_exp_time_gain_50Hz;
            DBG_PRINT(">>Set frequence 25fps 50Hz.\r\n");
        }
    }
    else if(freq == 60)
    {
        if(imx577_cdsp_mipi_ops.info[info_index].sensor_fps == 120)
        {
            seInfo.sensor_ev_idx = IMX577_120FPS_60HZ_INIT_EV_IDX;
            seInfo.ae_ev_step = 0;
            seInfo.daylight_ev_idx= IMX577_120FPS_60HZ_DAY_EV_IDX;
            seInfo.night_ev_idx= IMX577_120FPS_60HZ_NIGHT_EV_IDX;
            seInfo.max_ev_idx = IMX577_120FPS_60HZ_MAX_EV_IDX;
            seInfo.total_ev_idx = IMX577_120FPS_60HZ_EXP_TIME_TOTAL;

            p_expTime_table = (int *)imx577_120fps_exp_time_gain_60Hz;

            DBG_PRINT(">>Set frequence 120fps 60Hz.\r\n");

        }
        else if(imx577_cdsp_mipi_ops.info[info_index].sensor_fps == 25)
        {
            seInfo.sensor_ev_idx = IMX577_25FPS_60HZ_INIT_EV_IDX;
            seInfo.ae_ev_step = 0;
            seInfo.daylight_ev_idx= IMX577_25FPS_60HZ_DAY_EV_IDX;
            seInfo.night_ev_idx= IMX577_25FPS_60HZ_NIGHT_EV_IDX;
            seInfo.max_ev_idx = IMX577_25FPS_60HZ_MAX_EV_IDX;
            seInfo.total_ev_idx = IMX577_25FPS_60HZ_EXP_TIME_TOTAL;

            p_expTime_table = (int *)imx577_25fps_exp_time_gain_60Hz;
            DBG_PRINT(">>Set frequence 25fps 60Hz.\r\n");
        }
    }
}

static void imx577_seinfo_init(void)
{
    if(imx577_cdsp_mipi_ops.info[info_index].sensor_fps == 120)
    {
        seInfo.sensor_ev_idx = IMX577_120FPS_60HZ_INIT_EV_IDX;
        seInfo.ae_ev_step = 0;
        seInfo.daylight_ev_idx= IMX577_120FPS_60HZ_DAY_EV_IDX;
        seInfo.night_ev_idx= IMX577_120FPS_60HZ_NIGHT_EV_IDX;
        seInfo.max_ev_idx = IMX577_120FPS_60HZ_MAX_EV_IDX;
        seInfo.total_ev_idx = IMX577_120FPS_60HZ_EXP_TIME_TOTAL;

        p_expTime_table = (int *)imx577_120fps_exp_time_gain_60Hz;
    }
    else if(imx577_cdsp_mipi_ops.info[info_index].sensor_fps == 25)
    {
        seInfo.sensor_ev_idx = IMX577_25FPS_60HZ_INIT_EV_IDX;
        seInfo.ae_ev_step = 0;
        seInfo.daylight_ev_idx= IMX577_25FPS_60HZ_DAY_EV_IDX;
        seInfo.night_ev_idx= IMX577_25FPS_60HZ_NIGHT_EV_IDX;
        seInfo.max_ev_idx = IMX577_25FPS_60HZ_MAX_EV_IDX;
        seInfo.total_ev_idx = IMX577_25FPS_60HZ_EXP_TIME_TOTAL;

        p_expTime_table = (int *)imx577_25fps_exp_time_gain_60Hz;
    }

    pre_sensor_time = 1;
    pre_sensor_a_gain = 0x00;

    seInfo.time = 1;
    seInfo.analog_gain = 0x00;
    seInfo.digital_gain = 0x40;
}

/**************************************************************************
 *                       Sensor Initialization Function                   *
 **************************************************************************/
static void imx577_mipi_sensor_init(void)
{
    drv_l2_sensor_set_mclkout(imx577_cdsp_mipi_ops.info[info_index].mclk, 0);

    // reguest sccb
    imx577_sccb_open();

#if 0 //no use
    // reset sensor
    imx577_sccb_write_table((regval16_t *)imx577_reset_table);
    vTaskDelay(20);
#endif

    // init sensor
    if(info_index == 0)
    {
        imx577_sccb_write_table((regval16_t *)imx577_init_table_vga_2lane_120fps);
        DBG_PRINT("Sensor IMX557 VGA 120fps\r\n");
    }
    else if(info_index == 1)
    {
        imx577_sccb_write_table((regval16_t *)imx577_init_table_vga_2lane_50fps);
        DBG_PRINT("Sensor IMX557 VGA 50fps\r\n");
    }
    else if(info_index == 2)
    {
        imx577_sccb_write_table((regval16_t *)imx577_init_table_760_4lane_120fps);
        DBG_PRINT("Sensor IMX557 1012*760 120fps\r\n");
    }
    else if(info_index == 3)
    {
        imx577_sccb_write_table((regval16_t *)imx577_init_table_760_4lane_50fps);
        DBG_PRINT("Sensor IMX557 1012*760 50fps\r\n");
    }
    else if(info_index == 4)
    {
        imx577_sccb_write_table((regval16_t *)imx577_init_table_full_4lane_25fps);
        DBG_PRINT("Sensor IMX557 4056*3040 25fps\r\n");
    }

    DBG_PRINT("Sensor IMX577 initial completed\r\n");
}

/**
 * @brief   initialization function
 * @param   sensor format parameters
 * @return  none
 */
void imx577_cdsp_mipi_init(INT32U index, INT32U bufA, INT32U bufB, INT8U (*init_hook)(void))
{
    INT8U docropflag = 0;
    INT16U target_w, target_h, sensor_w, sensor_h;
    gpCdspFmt_t format;

    // set sensor size
    info_index = index;
    DBG_PRINT("%s = %d\r\n", __func__, index);

    // ae init
    imx577_seinfo_init();

    // Turn on LDO 2.8V for CSI sensor
    //drv_l1_ldo28_sel(LDO_V28, ENABLE);

    // Reset
#ifdef FRONT_SENSOR_RESET
    gpio_set_port_attribute(FRONT_SENSOR_RESET, ATTRIBUTE_HIGH);
    gpio_init_io(FRONT_SENSOR_RESET, GPIO_OUTPUT);
    gpio_write_io(FRONT_SENSOR_RESET, DATA_HIGH);
    osDelay(10);
    gpio_write_io(FRONT_SENSOR_RESET, DATA_LOW);
    osDelay(20);
    gpio_write_io(FRONT_SENSOR_RESET, DATA_HIGH);
    osDelay(10);
#endif

    // cdsp init
    drv_l2_cdsp_open();

    // mipi enable
    drv_l1_mipi_init(MIPI_DEV_NO, ENABLE, 0);

    if(imx577_mipi_cfg.pixel_clk_sel == GENERATE_PIXEL_CLK)
    {
        drv_l1_clock_set_ext_pll_freq(594000000, ENABLE);
        drv_l1_clock_set_peripheral_src_extpll(ENABLE);
        drv_l2_mipi_ctrl_set_clk(ENABLE, 2);//MIPI_CLK_DIV); //Pixel clk=450/4=112.5M,for "GENERATE_PIXEL_CLK"
        osDelay(100);
    }

    // set cdsp format
    #if BINMODE_OUT
    format.image_source = C_CDSP_BINMODE + MIPI_DEV_NO;
    #elif MIPI_DEV_NO == 0
    format.image_source = C_CDSP_MIPI;
    #else
    format.image_source = C_CDSP_MIPI1;
    #endif
    format.input_format =  imx577_cdsp_mipi_ops.info[index].input_format;
    format.output_format = imx577_cdsp_mipi_ops.info[index].output_format;
    target_w = imx577_cdsp_mipi_ops.info[index].target_w;
    target_h = imx577_cdsp_mipi_ops.info[index].target_h;
    format.hpixel = sensor_w = imx577_cdsp_mipi_ops.info[index].sensor_w;
    format.vline = sensor_h = imx577_cdsp_mipi_ops.info[index].sensor_h;
    format.hoffset = imx577_cdsp_mipi_ops.info[index].hoffset;
    format.voffset = imx577_cdsp_mipi_ops.info[index].voffset;
    format.sensor_timing_mode = imx577_cdsp_mipi_ops.info[index].interface_mode;
    format.sensor_hsync_mode = imx577_cdsp_mipi_ops.info[index].hsync_active;
    format.sensor_vsync_mode = imx577_cdsp_mipi_ops.info[index].vsync_active;

    if(drv_l2_cdsp_set_fmt(&format, NUM_SENSOR) < 0)
    {
        DBG_PRINT("cdsp set fmt err!!!\r\n");
    }

    drv_l1_CdspSetDataSource(SDRAM_INPUT);

    // set scale down
    if(((sensor_w > target_w) || (sensor_h > target_h)) && (docropflag == 0))
    {
        drv_l2_cdsp_set_yuv_scale(target_w, target_h);
    }
    else if(((sensor_w > target_w) || (sensor_h > target_h)) && (docropflag == 1))
    {
        INT32U point_x = 16, point_y = 16;
        drv_l1_CdspSetCrop(point_x, point_y, target_w, target_h);
        drv_l1_CdspSetCropEn(docropflag); // 1:Enable, 0:Disable
    }

    if(init_hook) {
        init_hook();
    }

    // cdsp start
    drv_l2_CdspTableRegister((gpCisCali_t*)&g_cali, CDSP_IDX);

    drv_l2_cdsp_stream_on(ENABLE, bufA, bufB, NUM_SENSOR);
    drv_l2_cdsp_analog_gain_thr_init(GainThrTable, CDSP_IDX);
    drv_l2_cdsp_enable((gpUserFav_t *)&g_FavTable, sensor_w, sensor_h, target_w, target_h, CDSP_IDX);
    // Disable auto white balance
    drv_l2_cdsp_awb_gain_auto_update_en(0);
    // Set white balance gain
    drv_l1_CdspSetWbGain(0x20, 0x20, 0x20);
    // Set U/V scale and offset
    drv_l2_cdsp_set_saturation_day(0, 0, 0, 0);

#if FIXED_EXP_EN
    drv_l1_CdspSetAEEn(DISABLE, DISABLE);   //Disable ISP AE
#endif //FIXED_EXP_EN
#if FIXED_WB_EN
    drv_l1_IspSetAwbEn(DISABLE);            //Disable ISP AWB
#endif //FIXED_WB_EN

    if((index == 0) || (index == 1))
    {
        imx577_mipi_cfg.mipi_lane_no = MIPI_2_LANE;
        DBG_PRINT("MIPI 2-Lane\r\n");
    }
    else
    {
        imx577_mipi_cfg.mipi_lane_no = MIPI_4_LANE;
        DBG_PRINT("MIPI 4-Lane\r\n");
    }
    imx577_mipi_cfg.h_size = imx577_cdsp_mipi_ops.info[info_index].sensor_w;
    imx577_mipi_cfg.v_size = imx577_cdsp_mipi_ops.info[info_index].sensor_h;

    // mipi start
    if(drv_l1_mipi_set_parameter(MIPI_DEV_NO, &imx577_mipi_cfg) < 0)
        DBG_PRINT("MIPI%d init fail !!!\r\n", MIPI_DEV_NO);

    #if MIPI_ISR_TEST == 1
    if (MIPI_DEV_NO == MIPI_1)
        drv_l1_mipi_B_isr_register(mipi_imx577_handle);
    else
        drv_l1_mipi_isr_register(mipi_imx577_handle);
    #endif
    drv_l1_mipi_set_irq_enable(MIPI_DEV_NO, ENABLE, MIPI_INT_ALL);

    // Enable MCLK & Init Sensor
    imx577_mipi_sensor_init();

#if !FIXED_EXP_EN
    // reset sensor ev idx
    seInfo.ae_ev_step = 0;
    imx577_set_xfps_exposure_time(&seInfo);
#endif //!FIXED_EXP_EN

    DBG_PRINT("IMX577 cdsp mipi init completed\r\n");
}

/**
 * @brief   un-initialization function
 * @param   sensor format parameters
 * @return  none
 */
static void imx577_cdsp_mipi_uninit(void)
{
    drv_l2_cdsp_stream_off();

    // disable mclk
    drv_l2_sensor_set_mclkout(MCLK_NONE, 0);

    // cdsp disable
    drv_l2_cdsp_close();

    // mipi disable
    drv_l1_mipi_init(MIPI_DEV_NO, DISABLE, 0);

    // release sccb
    imx577_sccb_close();

    /* Turn off LDO 2.8V for CSI sensor */
    //drv_l1_ldo28_sel(LDO_V28, DISABLE);

    drv_l1_CdspReset();
    drv_l1_FrontReset();
    gp_isp_reset_iq_tuning_target();

    DBG_PRINT("IMX577 cdsp mipi uninit\r\n");
}

/**
 * @brief   stream start function
 * @param   info index
 * @return  none
 */
void imx577_cdsp_mipi_stream_on(void)
{
    drv_l1_CdspReset();

    drv_l2_cdsp_clock_enable(ENABLE);
    drv_l1_CdspSetDataSource(BINMODE_OUT ? (BINMODE_INPUT + MIPI_DEV_NO) : MIPI_INPUT);
    DBG_PRINT("IMX577 cdsp mipi stream on\r\n");
}

/**
 * @brief   stream stop function
 * @param   none
 * @return  none
 */
static void imx577_cdsp_mipi_stream_off(void)
{
    drv_l2_cdsp_stream_off();
    DBG_PRINT("IMX577 cdsp mipi stream off\r\n");
}

/**
 * @brief   get info function
 * @param   none
 * @return  pointer to sensor information data
 */
static drv_l2_sensor_info_t* imx577_cdsp_mipi_get_info(INT32U index)
{
    if(index > (MAX_INFO_NUM - 1)) {
        return NULL;
    } else {
        return (drv_l2_sensor_info_t*)&imx577_cdsp_mipi_ops.info[index];
    }
}

/**
 * @brief   get default info function
 * @param   none
 * @return  pointer to default sensor information data
 */
static drv_l2_sensor_info_t* imx577_cdsp_mipi_get_info_by_default(void)
{
    return (drv_l2_sensor_info_t*)&imx577_cdsp_mipi_ops.info[info_index];
}

void imx577_switch_vga_resolution_120fps(void)
{
    // set sensor size
    info_index = 0;

    imx577_mipi_cfg.mipi_lane_no = MIPI_2_LANE;
    imx577_mipi_cfg.h_size = imx577_cdsp_mipi_ops.info[info_index].sensor_w;
    imx577_mipi_cfg.v_size = imx577_cdsp_mipi_ops.info[info_index].sensor_h;

    // mipi start
    if(drv_l1_mipi_set_parameter(MIPI_DEV_NO, &imx577_mipi_cfg) < 0)
        DBG_PRINT("MIPI%d init fail !!!\r\n", MIPI_DEV_NO);

    imx577_sccb_write_table((regval16_t *)imx577_vga_2lane_120fps_partial_setting);
    DBG_PRINT("imx577_vga_2lane_120fps_partial_setting\r\n");

    drv_l1_CdspReset();
    drv_l1_CdspSetDataSource(BINMODE_OUT ? (BINMODE_INPUT + MIPI_DEV_NO) : MIPI_INPUT);
    DBG_PRINT("imx577 vga_2lane_120fps cdsp mipi stream on\r\n");
}

void imx577_switch_vga_resolution_50fps(void)
{
    // set sensor size
    info_index = 1;

    imx577_mipi_cfg.mipi_lane_no = MIPI_2_LANE;
    imx577_mipi_cfg.h_size = imx577_cdsp_mipi_ops.info[info_index].sensor_w;
    imx577_mipi_cfg.v_size = imx577_cdsp_mipi_ops.info[info_index].sensor_h;

    // mipi start
    if(drv_l1_mipi_set_parameter(MIPI_DEV_NO, &imx577_mipi_cfg) < 0)
        DBG_PRINT("MIPI%d init fail !!!\r\n", MIPI_DEV_NO);

    imx577_sccb_write_table((regval16_t *)imx577_vga_2lane_50fps_partial_setting);
    DBG_PRINT("imx577_vga_2lane_50fps_partial_setting\r\n");

    drv_l1_CdspReset();
    drv_l1_CdspSetDataSource(BINMODE_OUT ? (BINMODE_INPUT + MIPI_DEV_NO) : MIPI_INPUT);
    DBG_PRINT("imx577 vga_2lane_50fps cdsp mipi stream on\r\n");
}

void imx577_switch_760_resolution_120fps(void)
{
    // set sensor size
    info_index = 2;

    imx577_mipi_cfg.mipi_lane_no = MIPI_4_LANE;
    imx577_mipi_cfg.h_size = imx577_cdsp_mipi_ops.info[info_index].sensor_w;
    imx577_mipi_cfg.v_size = imx577_cdsp_mipi_ops.info[info_index].sensor_h;

    // mipi start
    if(drv_l1_mipi_set_parameter(MIPI_DEV_NO, &imx577_mipi_cfg) < 0)
        DBG_PRINT("MIPI%d init fail !!!\r\n", MIPI_DEV_NO);

    imx577_sccb_write_table((regval16_t *)imx577_760_4lane_120fps_partial_setting);
    DBG_PRINT("imx577_760_4lane_120fps_partial_setting\r\n");

    drv_l1_CdspReset();
    drv_l1_CdspSetDataSource(BINMODE_OUT ? (BINMODE_INPUT + MIPI_DEV_NO) : MIPI_INPUT);
    DBG_PRINT("imx577 760_4lane_120fps cdsp mipi stream on\r\n");
}

void imx577_switch_760_resolution_50fps(void)
{
    // set sensor size
    info_index = 3;

    imx577_mipi_cfg.mipi_lane_no = MIPI_4_LANE;
    imx577_mipi_cfg.h_size = imx577_cdsp_mipi_ops.info[info_index].sensor_w;
    imx577_mipi_cfg.v_size = imx577_cdsp_mipi_ops.info[info_index].sensor_h;

    // mipi start
    if(drv_l1_mipi_set_parameter(MIPI_DEV_NO, &imx577_mipi_cfg) < 0)
        DBG_PRINT("MIPI%d init fail !!!\r\n", MIPI_DEV_NO);

    imx577_sccb_write_table((regval16_t *)imx577_760_4lane_50fps_partial_setting);
    DBG_PRINT("imx577_760_4lane_50fps_partial_setting\r\n");

    drv_l1_CdspReset();
    drv_l1_CdspSetDataSource(BINMODE_OUT ? (BINMODE_INPUT + MIPI_DEV_NO) : MIPI_INPUT);
    DBG_PRINT("imx577 760_4lane_50fps cdsp mipi stream on\r\n");
}

void imx577_switch_full_resolution_25fps(void)
{
    drv_l1_CdspSetDataSource(SDRAM_INPUT);

    // set sensor size
    info_index = 4;

    imx577_mipi_cfg.mipi_lane_no = MIPI_4_LANE;
    imx577_mipi_cfg.h_size = imx577_cdsp_mipi_ops.info[info_index].sensor_w;
    imx577_mipi_cfg.v_size = imx577_cdsp_mipi_ops.info[info_index].sensor_h;

    // mipi start
    if(drv_l1_mipi_set_parameter(MIPI_DEV_NO, &imx577_mipi_cfg) < 0)
        DBG_PRINT("MIPI%d init fail !!!\r\n", MIPI_DEV_NO);

    imx577_sccb_write_table((regval16_t *)imx577_full_4lane_25fps_partial_setting);
    DBG_PRINT("imx577_full_4lane_25fps_partial_setting\r\n");
}

/**
  *@brief Flash trigger
  *@param None
  *@retval None
  */
void imx577_flash_trigger(void)
{
    INT32S ret;
    INT16U value = 0;

    INT16U start_point = 0;
    INT16U delay = 0;
    INT8U adj = 1;
    INT16U widt_ctrl = 1;

    imx577_sccb_write(FLASH_MD_RS, 0x04);

    imx577_sccb_write(FLASH_STRB_START_POINT_H, ((start_point >> 8) & 0xFF));
    imx577_sccb_write(FLASH_STRB_START_POINT_L, (start_point & 0xFF));

    imx577_sccb_write(FLASH_STRB_DLY_RS_CTRL_H, ((delay >> 8) & 0xFF));
    imx577_sccb_write(FLASH_STRB_DLY_RS_CTRL_L, (delay & 0xFF));

    //Flash Pulse Width (H) (TFLASH_STRB_WIDT_H_RS_CTL/24M) * FLASH_STRB_ADJ
    imx577_sccb_write(FLASH_STRB_ADJ, adj);
    imx577_sccb_write(FLASH_STRB_WIDT_H_RS_CTRL_H, ((widt_ctrl >> 8) & 0xFF));
    imx577_sccb_write(FLASH_STRB_WIDT_H_RS_CTRL_L, (widt_ctrl & 0xFF));

    imx577_sccb_write(FLASH_TRIG_RS, 0x01);
}
/**
  *@brief Read flash trigger status
  *@param None
  *@retval Status (0:Stop Others:Processing)
  */
INT8U imx577_read_flash_trigger(void)
{
    INT32S ret;
    INT16U value = 0;

    ret = imx577_sccb_read(FLASH_TRIG_RS, &value);
    if(value) return 1;
    else return 0;
}
/**
  *@brief Get current sensor resolution
  *@param *width: Get sensor width
  *@param *height: Get sensor hieght
  *@retval None
  */
void imx577_get_cur_res(INT32U *width, INT32U *height)
{
    if((width == NULL) && (height == NULL)) return NULL;
    *width = imx577_cdsp_mipi_ops.info[info_index].sensor_w;
    *height = imx577_cdsp_mipi_ops.info[info_index].sensor_h;
}
/*********************************************
*    sensor ops declaration
*********************************************/
const drv_l2_sensor_ops_t imx577_cdsp_mipi_ops =
{
    SOURCE_CDSP_MIPI,                    /* sensor name */
    imx577_cdsp_mipi_init,
    imx577_cdsp_mipi_uninit,
    imx577_cdsp_mipi_stream_on,
    imx577_cdsp_mipi_stream_off,
    imx577_cdsp_mipi_get_info,
    imx577_set_ae,
    sensor_get_ae_info,
    sensor_set_max_lum,
    {
        /* 1st info */ // VGA
        {
            MCLK_24M,                    /* MCLK */
            V4L2_PIX_FMT_SRGGB10,        /* input format */
            V4L2_PIX_FMT_YUYV,           /* output format */
            120,                         /* FPS in sensor */
            PSC700_LIVEVIEW_WIDTH,       /* target width */
            PSC700_LIVEVIEW_HEIGHT,      /* target height */
            640,                         /* sensor width */
            480,                         /* sensor height */
            0,                           /* sensor h offset */
            0,                           /* sensor v offset */
            MODE_CCIR_601,               /* input interface */
            MODE_NONE_INTERLACE,         /* interlace mode */
            MODE_ACTIVE_HIGH,            /* hsync pin active level */
            MODE_ACTIVE_HIGH,            /* vsync pin active level */
        },
        /* 2nd info */ // VGA@50
        {
            MCLK_24M,                    /* MCLK */
            V4L2_PIX_FMT_SRGGB10,        /* input format */
            V4L2_PIX_FMT_YUYV,           /* output format */
            50,                          /* FPS in sensor */
            PSC700_LIVEVIEW_WIDTH,       /* target width */
            PSC700_LIVEVIEW_HEIGHT,      /* target height */
            640,                         /* sensor width */
            480,                         /* sensor height */
            0,                           /* sensor h offset */
            0,                           /* sensor v offset */
            MODE_CCIR_601,               /* input interface */
            MODE_NONE_INTERLACE,         /* interlace mode */
            MODE_ACTIVE_HIGH,            /* hsync pin active level */
            MODE_ACTIVE_HIGH,            /* vsync pin active level */
        },
        /* 3rd info */ // 760@120
        {
            MCLK_24M,                    /* MCLK */
            V4L2_PIX_FMT_SRGGB10,        /* input format */
            V4L2_PIX_FMT_YUYV,           /* output format */
            120,                         /* FPS in sensor */
            PSC700_LIVEVIEW_WIDTH,       /* target width */
            PSC700_LIVEVIEW_HEIGHT,      /* target height */
            1012,                        /* sensor width */
            760,                         /* sensor height */
            0,                           /* sensor h offset */
            0,                           /* sensor v offset */
            MODE_CCIR_601,               /* input interface */
            MODE_NONE_INTERLACE,         /* interlace mode */
            MODE_ACTIVE_HIGH,            /* hsync pin active level */
            MODE_ACTIVE_HIGH,            /* vsync pin active level */
        },
        /* 4th info */ // 760@50
        {
            MCLK_24M,                    /* MCLK */
            V4L2_PIX_FMT_SRGGB10,        /* input format */
            V4L2_PIX_FMT_YUYV,           /* output format */
            50,                          /* FPS in sensor */
            PSC700_LIVEVIEW_WIDTH,       /* target width */
            PSC700_LIVEVIEW_HEIGHT,      /* target height */
            1012,                        /* sensor width */
            760,                         /* sensor height */
            0,                           /* sensor h offset */
            0,                           /* sensor v offset */
            MODE_CCIR_601,               /* input interface */
            MODE_NONE_INTERLACE,         /* interlace mode */
            MODE_ACTIVE_HIGH,            /* hsync pin active level */
            MODE_ACTIVE_HIGH,            /* vsync pin active level */
        },
        /* 5th info */ // 4056*3040
        {
            MCLK_24M,                    /* MCLK */
            V4L2_PIX_FMT_SRGGB10,        /* input format */
            V4L2_PIX_FMT_YUYV,           /* output format */
            25,                          /* FPS in sensor */
            4056,                        /* target width */
            3040,                        /* target height */
            4056,                        /* sensor width */
            3040,                        /* sensor height */
            0,                           /* sensor h offset */
            0,                           /* sensor v offset */
            MODE_CCIR_601,               /* input interface */
            MODE_NONE_INTERLACE,         /* interlace mode */
            MODE_ACTIVE_HIGH,            /* hsync pin active level */
            MODE_ACTIVE_HIGH,            /* vsync pin active level */
        },
    },
    imx577_cdsp_mipi_get_info_by_default,
    INTERFACE_FLAG,
    imx577_mipi_set_exp_freq,
};

#endif //(defined _SENSOR_IMX577_CDSP_MIPI) && (_SENSOR_IMX577_CDSP_MIPI == 1)
