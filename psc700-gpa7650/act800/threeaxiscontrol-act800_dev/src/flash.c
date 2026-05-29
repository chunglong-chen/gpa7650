/**
 * @file flash.c
 * @brief Implementation of flash read/write/erase functions.
 *
 * @details This file provides functions to read/write/erase flash memory.
 */
#include "flash.h"

uint32_t userConfigCheckSum = 0;
uint32_t noTrespassBorderChecksum=0;
uint32_t torqueConfigChecksum=0; 

/**
 * @brief  Erase one flash memory page.
 *
 * @details This function erases a single flash page. Each page size is 2KB.
 *
 * @param  u32PageAddr The address of the page to erase. Must be aligned to the page size.
 * @retval 0   Erase success.
 * @retval -1  Erase failed.
 */
char flashErase(uint32_t u32PageAddr)
{
	char ret=-1;
	
    SYS_UnlockReg();
    FMC_Open();
	FMC_ENABLE_AP_UPDATE();
	
	ret = FMC_Erase(u32PageAddr);
	
	FMC_DISABLE_AP_UPDATE();
	FMC_Close();
	SYS_LockReg();
	
	return ret;
}

/**
 * @brief  Read flash memory.
 *
 * @details This function reads data from flash memory into the @c dataBuff.
 *			Both the address and data buffer must be 4-byte aligned.
 *
 * @param  		address   Starting address to read from (must be 4-byte aligned).
 * @param[out]  dataBuff  Pointer to the buffer to store read data (must be 4-byte aligned).
 * @param  		Len       Number of 32-bit words to read (i.e., sizeof(dataBuff) / sizeof(uint32_t)).
 */
void flashRead(uint32_t address, uint32_t *dataBuff, uint32_t Len)
{	
	uint32_t i=0;
	
	SYS_UnlockReg();
    FMC_Open();
	FMC_ENABLE_AP_UPDATE();
	
	for (;i<Len;i++)
	{
		dataBuff[i] = FMC_Read((address+(i*4)));
	}
	
	FMC_DISABLE_AP_UPDATE();
	FMC_Close();
	SYS_LockReg();
}

/**
 * @brief  Write data to flash memory.
 *
 * @details This function writes data from the buffer to flash memory.
 *			Both the address and data buffer must be 4-byte aligned.
 *
 * @param  address   Starting address to write to (must be 4-byte aligned).
 * @param  dataBuff  Pointer to the buffer containing data to write (must be 4-byte aligned).
 * @param  Len       Number of 32-bit words to write (i.e., sizeof(dataBuff) / sizeof(uint32_t)).
 */
void flashWrite(uint32_t address, uint32_t *dataBuff, uint32_t Len)
{
	uint32_t i=0;
	
	SYS_UnlockReg();
    FMC_Open();
	FMC_ENABLE_AP_UPDATE();
	
	for (;i<Len;i++)
	{
		FMC_Write((address+(i*4)), dataBuff[i]);
	}
	
	FMC_DISABLE_AP_UPDATE();
	FMC_Close();
	SYS_LockReg();
}

/**
 * @brief  Calculate checksum of a byte array.
 *
 * @details This function calculates a simple checksum by summing all bytes in the array.
 *
 * @param  src  	Pointer to the source data buffer.
 * @param  len  	Length of the buffer in bytes.
 * @return Checksum The computed checksum value (sum of all bytes).
 */
unsigned int CalcChecksum(unsigned char *src, unsigned short len)
{
	unsigned int	Checksum = 0;				// calculated checksum value
	short	Index;						// loop counter
	unsigned char *Data;
			
	/* Loop to calculate checksum */
	Data = src;
	Checksum = 0;

	for (Index = 0; Index < len; Index++)
	{
		Checksum += Data[Index] & 0xff;
	}
	
	//printf("checksum=%d\n",Checksum);
	return(Checksum);
}							

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
