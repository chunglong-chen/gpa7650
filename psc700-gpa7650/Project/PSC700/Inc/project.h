
// when this on, use spi nand, it replace classic parallel raw nand driver except booting role, old defines
// and apis that use for classic parallel raw nand such as NAND1_EN, NAND2_EN, _DRV_L1_NAND, _DRV_L2_NAND
// NAND_APP_EN, NAND_APP_MSDC_EN, NAND_APP_WRITE_EN, also applied to the spi nand flash driver.
// Also, library have to choose lib_nand_spi_gpm4_g+ide_freertos.a instead of lib_nand_gpm4_g+ide_freertos.a
// Also, set NAND_APP_EN to 1
// Also, if spi nor mounted on same pixmux with spi nand, please remove spi nor
// Also, assign correct cs pin by using spi_nand_assign_cs, see drv_l1_init
//#define USE_NANDSPI_INSTEAD_NANDFLASH

#ifndef DBG_PRINT
#define DBG_PRINT		print_string
#endif

// Global definitions
#define TRUE			1
#define FALSE			0
#define ENABLE					1
#define DISABLE					0
//#define NULL			0
#define STATUS_OK		0
#define STATUS_FAIL		-1

// Operating System definitions and config
#define _OS_NONE		0
#define _OS_UCOS2		1
#define _OS_FREERTOS	2
#define	_OPERATING_SYSTEM       _OS_FREERTOS // _OS_FREERTOS, _OS_NONE

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include "typedef.h"
#include "drv_l1_sfr.h"
#include <stdlib.h>

#define malloc(x)      gp_malloc(x)
//#define DRV_Reg32(addr)		        (*(volatile unsigned *)(addr))

extern INT32U MCLK;
extern INT32U MHZ;

#define SDRAM_START_ADDR            0x00000000
#define SDRAM_END_ADDR              0x01FFFFFF

#define ISRAM_START_ADDR            0xF8000000
#define ISRAM_END_ADDR              0xF8033FFF

#define UART_BAUD_RATE              115200
