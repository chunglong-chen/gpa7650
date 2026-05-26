/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M261 MCU.
 *
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "M261pin.h"
#include "uart_cmd.h"
#include "i2c_cmd.h"
#include "ringbuffer.h"
#include "timer.h"
#include "motor_ctrl.h"
#include "main.h"
#include "M261.h"
#include "version.h"
#include "flash.h"
#include "delay.h"
#include "test.h"
#include "user_config.h"

void timer_setup();
void timer_setOneShutPS(TIMER_T *timer, uint32_t period);

static bool burn_in_mode = 0;
static int  burn_in_count = 0;
static bool reduced_torque_test=true;

// key SW2 IRQ
void GPB_IRQHandler(void)
{
    /* To check if PB.2 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT9))
    {
        uDelay(500); //debounce
        if(*(GPIO_PIN_DATA_ADDR(portB, 9)))
        {
            GPIO_CLR_INT_FLAG(PB, BIT9);
            return;
        }
        //in burn-in mode, short press to switch driving current
        if(burn_in_mode)
        {
            reduced_torque_test=!reduced_torque_test;
            printf((reduced_torque_test?"reduced torque selected\n"
                                        :"full torque selected\n"));
        }
        
        //long press to toggle burn-in mode
        //check of press every 1 ms for 1 seconds
        for(uint32_t i=0; i<1000; i++)
        {
            uDelay(1000);
            // if released (high) with in 1 second, clear IRQ flag and return
            if(*(GPIO_PIN_DATA_ADDR(portB, 9)))
            {
                GPIO_CLR_INT_FLAG(PB, BIT9);
                return;
            }
        }
        
        // passed press and hold check, toggle burn-in flag
        burn_in_mode = !burn_in_mode;
        printf("%s burn-in loop\n", (burn_in_mode? "Start" : "Stop"));
        GPIO_CLR_INT_FLAG(PB, BIT9);
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        PB->INTSRC = PB->INTSRC;
        printf("Un-expected interrupts.\n");
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Set SysTick source to HCLK/2 */
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLKSEL_HCLK_DIV2);

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);
    
    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    
    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_HIRC, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HIRC, 0);

    /* Set PA8~PA11 GPIO pull up mode */
    GPIO_SetPullCtl(PA, SW3_PIN_MASK, GPIO_PUSEL_PULL_UP);
    //I2C_UART_SEL = I2C_UART_SW; //I2C UART selection pin: not in use

    /* Enable I2C0 peripheral clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

	//init pin
    M261pin_init();
	
	/* Configure PB.9 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PB, BIT9, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 9, GPIO_INT_FALLING); //active low button, interrupt on pressed (Hi->Lo)
    NVIC_EnableIRQ(GPB_IRQn);
		
    /* Lock protected registers */
    SYS_LockReg();
}

void motor_control_interface_init()
{
    // I2C
    I2C_Open(I2C0, 100000);
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
    /* Slave Address : 0x44 (7-bit)*/
    I2C_SetSlaveAddr(I2C0, 0, 0x44, 0);   
    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

    //UART
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);
    NVIC_EnableIRQ(UART0_IRQn);
}

uint8_t BitToNum(uint32_t src)
{
    uint8_t i=0;
    if (src == 0)
        return 255;
    while(src != 1)
    {
        src = src >> 1;
        i++;
    }
    return i;
}

/*
 * This is a template project for M261 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

    motor_control_interface_init();
    
    printf("\n\r3-axis motion controller %s\n\r", VERSION);
    printf("Medimaging Integrated Solution Inc. All rights reserved.\n\r");
    
    timer_setup();
    //PB10 Red LED
    GPIO_SetMode(PB, BIT10, GPIO_MODE_OUTPUT);
    
    //legacy code: remove once making sure what pin it is
    //GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT);
    
    initUserConfig();
    initNoTrespassBorder();
    initTorqueConfig();
    motor_init();
    
    //
    init_uart_cmd();
    
#if 0
    printf("CPU clock %dHz\n", CLK_GetCPUFreq());
    printf("HCLK clock %dHz\n", CLK_GetHCLKFreq()); 
    printf("PCLK0 clock %dHz\n", CLK_GetPCLK0Freq());   
    printf("PCLK1 clock %dHz\n", CLK_GetPCLK1Freq());
    printf("TIMER0 clock %dHz\n", TIMER_GetModuleClock(TIMER0));
#endif

    motor_enable(0);
    motor_enable(1);
    motor_enable(2);

#if 0
	for (uint8_t i=0;i<3;i++)
	{
		printf("userConfig[%d].highEdgePin.pin = BIT%d\n", i,BitToNum(userConfig[i].highEdgePin.pin));
		printf("userConfig[%d].lowEdgePin.pin  = BIT%d\n", i,BitToNum(userConfig[i].lowEdgePin.pin));
		printf("userConfig[%d].fwdDirState     = %d\n", i,userConfig[i].fwdDirState);
		
		printf("userConfig[%d].enableMototAccelerate = %d\n", i,userConfig[i].enableMototAccelerate);
		printf("userConfig[%d].defaultPPS   = %d\n", i,userConfig[i].defaultPPS);
		printf("userConfig[%d].targetPPS    = %d\n", i,userConfig[i].targetPPS);
		printf("userConfig[%d].acceleration = %d\n", i,userConfig[i].acceleration);
		printf("userConfig[%d].deceleration = %d\n", i,userConfig[i].deceleration);
		
		printf("\n");
	}
    
    for(uint8_t i=0; i<8; i++){
        if(noTrespassBorder[i].enable)
            printf("noTrespassBorder[%d]: (%f)x + (%f)y + (%f)z %s (%f)\n", i,
                    noTrespassBorder[i].a,
                    noTrespassBorder[i].b,
                    noTrespassBorder[i].c,
                   (noTrespassBorder[i].greater_than?">":"<"),
                    noTrespassBorder[i].d);
        else
             printf("noTrespassBorder[%d]: disabled\n", i);
    }
#endif

    /* Got no where to go, just loop forever */
    int loop_cnt=0;
    while(1){
        __NOP();
        if (burn_in_mode)
        {
            burn_in_count=burn_in_loop(burn_in_count,
                (reduced_torque_test?CURRENT_37_PERCENT:CURRENT_100_PERCENT));
            if(burn_in_count==5 && reduced_torque_test)
                burn_in_mode=false;
            if(!burn_in_mode)
                burn_in_count=0;
        }
        else{
            cmd_task();
        }
        if(loop_cnt++==200000){
            PB10=~PB10;
            loop_cnt=0;
        }
        //120kHz
    }
}

void timer_setup(){

    timer_setOneShutPS(TIMER0, 1000); //1us
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    timer_setOneShutPS(TIMER1, 1000); //1us
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);

    timer_setOneShutPS(TIMER2, 1000); //1us
    TIMER_EnableInt(TIMER2);
    NVIC_EnableIRQ(TMR2_IRQn);
    
    /* Open Timer3 in periodic mode
       enable interrupt and 8 interrupt ticks per second */
    //TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 100000000);
    //TIMER_EnableInt(TIMER3);
    //NVIC_EnableIRQ(TMR3_IRQn);
    //TIMER_Start(TIMER3);
}

//period : time in ns
void timer_setOneShutPS(TIMER_T *timer, uint32_t period){
    uint32_t u32Prescale = ((uint64_t)TIMER_GetModuleClock(timer)) * period / 1000000000UL - 1;
    uint32_t u32Mode = TIMER_ONESHOT_MODE;//TIMER_PERIODIC_MODE;
    timer->CTL = u32Mode | u32Prescale;
}

void TMR3_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER3))
    {
        /* Clear Timer3 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER3);
        __NOP();
    }
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
