#ifndef __FLASH_H__
#define __FLASH_H__

#include <stdio.h>
#include "NuMicro.h"

#define ALIGN_SIZE(SIZE, ALIGN)(((SIZE) + (ALIGN) - 1) & ~((ALIGN) - 1))

#define USERCONFIG_ADDRESS     0x7f000
#define NO_TRESPASS_BORDER_ADDRESS      0x7f800
#define TORQUE_CONFIG_ADDRESS      0x7e800

extern uint32_t userConfigCheckSum;
extern uint32_t noTrespassBorderChecksum;
extern uint32_t torqueConfigChecksum;

char flashErase(uint32_t u32PageAddr);
void flashRead(uint32_t address, uint32_t *dataBuff, uint32_t Len);
void flashWrite(uint32_t address, uint32_t *dataBuff, uint32_t Len);
unsigned int CalcChecksum(unsigned char *src, unsigned short len);

#endif /*__FLASH_H__*/