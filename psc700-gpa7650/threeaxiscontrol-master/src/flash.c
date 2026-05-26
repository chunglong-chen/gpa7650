#include "flash.h"

uint32_t userConfigCheckSum = 0;
uint32_t noTrespassBorderChecksum=0;
uint32_t torqueConfigChecksum=0; 

// One page erase
// Page size = 2kB
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

// Must input 4-bytes aligned dataBuff
// Address also need 4-bytes aligned address
// Len = sizeof(dataBuff) / sizeof(int)
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

// Must input 4-bytes aligned dataBuff
// Address also need 4-bytes aligned address
// Len = sizeof(dataBuff) / sizeof(int)
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
