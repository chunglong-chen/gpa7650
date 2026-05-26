/****************************************************************************
 * @file     M261pin.c
 * @version  V1.20
 * @Date     2020/08/19-15:51:59 
 * @brief    NuMicro generated code file
 *
 * Copyright (C) 2013-2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

/********************
MCU:M261SIAAE(LQFP64)
********************/

#include "M261.h"

void M261pin_init_ice(void)
{
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF1MFP_Msk | SYS_GPF_MFPL_PF0MFP_Msk);
    SYS->GPF_MFPL |= (SYS_GPF_MFPL_PF1MFP_ICE_CLK | SYS_GPF_MFPL_PF0MFP_ICE_DAT);

    return;
}

void M261pin_deinit_ice(void)
{
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF1MFP_Msk | SYS_GPF_MFPL_PF0MFP_Msk);

    return;
}

void M261pin_init_uart0(void)
{
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB12MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB13MFP_UART0_TXD | SYS_GPB_MFPH_PB12MFP_UART0_RXD);

    return;
}

void M261pin_deinit_uart0(void)
{
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB13MFP_Msk | SYS_GPB_MFPH_PB12MFP_Msk);

    return;
}

void M261pin_init_uart5(void)
{
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA4MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA5MFP_UART5_TXD | SYS_GPA_MFPL_PA4MFP_UART5_RXD);

    return;
}

void M261pin_deinit_uart5(void)
{
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA4MFP_Msk);

    return;
}

void M261pin_init_i2c0(void)
{
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA4MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA5MFP_I2C0_SCL | SYS_GPA_MFPL_PA4MFP_I2C0_SDA);

    return;
}

void M261pin_deinit_i2c0(void)
{
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA4MFP_Msk);

    return;
}

void M261pin_init()
{
    //SYS->GPA_MFPH = 0x00000000;
    //SYS->GPA_MFPL = 0x00880000;
    //SYS->GPB_MFPH = 0x00660000;
    //SYS->GPB_MFPL = 0x00000000;
    //SYS->GPC_MFPL = 0x00000000;
    //SYS->GPD_MFPL = 0x00000000;
    //SYS->GPF_MFPL = 0x000000EE;

    M261pin_init_ice();
    M261pin_init_uart0();
    M261pin_init_i2c0();

    return;
}

void M261pin_deinit()
{
    M261pin_deinit_ice();
    M261pin_deinit_uart0();
    M261pin_deinit_i2c0();

    return;
}
/*** (C) COPYRIGHT 2013-2020 Nuvoton Technology Corp. ***/
