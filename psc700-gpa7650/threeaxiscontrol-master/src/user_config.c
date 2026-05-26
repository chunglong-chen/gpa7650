#include <stdio.h>
#include "NuMicro.h"
#include "flash.h"
#include "motor_ctrl.h"
#include "user_config.h"

#define PRINT_USER_CONFIG_SUCCESS 0

#pragma pack(4)
USER_CONFIG userConfig[3];
NO_TRESPASS_BORDER noTrespassBorder[8];
TORQUE_CONFIG torqueConfig[3];
#pragma pack()

// Because the structure "motors" mixes state and initial parameters, it is not suitable for
// storing into flash rom.
// So create a structure to store the initial parameters.
void defaultUserConfig(void)
{
    // motor 0
    userConfig[0].highEdgePin.port = PB;
    userConfig[0].highEdgePin.pin  = BIT5;
    userConfig[0].highEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 5);

    userConfig[0].lowEdgePin.port = PB;
    userConfig[0].lowEdgePin.pin  = BIT4;
    userConfig[0].lowEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 4);

    userConfig[0].fwdDirState = TRUE;

    userConfig[0].enableMototAccelerate =  true;
    userConfig[0].defaultPPS = 1000;
    userConfig[0].targetPPS  = 2000;
    userConfig[0].acceleration = 50000;
    userConfig[0].deceleration = 50000;

    // motor 1
    userConfig[1].highEdgePin.port = PB;
    userConfig[1].highEdgePin.pin  = BIT3;
    userConfig[1].highEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 3);

    userConfig[1].lowEdgePin.port = PB;
    userConfig[1].lowEdgePin.pin  = BIT2;
    userConfig[1].lowEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 2);

    userConfig[1].fwdDirState = TRUE;

    userConfig[1].enableMototAccelerate =  true;
    userConfig[1].defaultPPS = 1000;
    userConfig[1].targetPPS  = 2000;
    userConfig[1].acceleration = 50000;
    userConfig[1].deceleration = 50000;

    // motor 2
    userConfig[2].highEdgePin.port = PB;
    userConfig[2].highEdgePin.pin  = BIT1;
    userConfig[2].highEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 1);

    userConfig[2].lowEdgePin.port = PB;
    userConfig[2].lowEdgePin.pin  = BIT0;
    userConfig[2].lowEdgePin.datAddr = GPIO_PIN_DATA_ADDR(portB, 0);

    userConfig[2].fwdDirState = TRUE;

    userConfig[2].enableMototAccelerate =  true;
    userConfig[2].defaultPPS = 1000;
    userConfig[2].targetPPS  = 2000;
    userConfig[2].acceleration = 50000;
    userConfig[2].deceleration = 50000;
}

void initUserConfig(void)
{
    uint32_t length = 0;

    length = (sizeof(USER_CONFIG)*3) / sizeof(int);
    flashRead(USERCONFIG_ADDRESS, (uint32_t *) &userConfig, length);
    flashRead(ALIGN_SIZE(USERCONFIG_ADDRESS+sizeof(USER_CONFIG)*3, 4), &userConfigCheckSum, 1);

    if (CalcChecksum((uint8_t *) &userConfig, (sizeof(USER_CONFIG)*3)) != userConfigCheckSum)
    {
        printf("Failed to load config data. Use default settings.\n");
        defaultUserConfig();
    }
#if PRINT_USER_CONFIG_SUCCESS
    else
        printf("User config init success\n");
#endif
}

void copySpeedToUserConfig(uint8_t motor_id)
{
    userConfig[motor_id].enableMototAccelerate = get_motor_ref(motor_id)->enableMototAccelerate;
    userConfig[motor_id].defaultPPS = get_motor_ref(motor_id)->defaultPPS;
    userConfig[motor_id].targetPPS  = get_motor_ref(motor_id)->targetPPS;
    userConfig[motor_id].acceleration = get_motor_ref(motor_id)->acceleration;
    userConfig[motor_id].deceleration = get_motor_ref(motor_id)->deceleration;
}

void copyPinConfigToUserConfig(uint8_t motor_id)
{
    userConfig[motor_id].highEdgePin.port    = get_motor_ref(motor_id)->highEdgePin.port;
    userConfig[motor_id].highEdgePin.pin     = get_motor_ref(motor_id)->highEdgePin.pin;
    userConfig[motor_id].highEdgePin.datAddr = get_motor_ref(motor_id)->highEdgePin.datAddr;

    userConfig[motor_id].lowEdgePin.port    = get_motor_ref(motor_id)->lowEdgePin.port;
    userConfig[motor_id].lowEdgePin.pin     = get_motor_ref(motor_id)->lowEdgePin.pin;
    userConfig[motor_id].lowEdgePin.datAddr = get_motor_ref(motor_id)->lowEdgePin.datAddr;
}

void copyDirectionConfigToUserConfig(uint8_t motor_id){
    userConfig[motor_id].fwdDirState = get_motor_ref(motor_id)->fwdDirState;
}

bool saveUserConfigToFlash(void)
{
#pragma pack(4)
    USER_CONFIG tempUserConfig[3];
#pragma pack()
    uint32_t tempCheckSum;
    
    // Store userconfig into flash
    userConfigCheckSum = CalcChecksum((uint8_t *) &userConfig, (sizeof(USER_CONFIG)*3));
    
    if (flashErase(USERCONFIG_ADDRESS) != 0)
        return false;
    
    flashWrite(USERCONFIG_ADDRESS, (uint32_t *) &userConfig, ((sizeof(USER_CONFIG)*3) / sizeof(int)));
    flashWrite(ALIGN_SIZE(USERCONFIG_ADDRESS+sizeof(USER_CONFIG)*3, 4), &userConfigCheckSum, 1);

    // Check data
    flashRead(USERCONFIG_ADDRESS, (uint32_t *) &tempUserConfig, ((sizeof(USER_CONFIG)*3) / sizeof(int)));
    flashRead(ALIGN_SIZE(USERCONFIG_ADDRESS+sizeof(USER_CONFIG)*3, 4), &tempCheckSum, 1);
    
    if (CalcChecksum((uint8_t *) &tempUserConfig, (sizeof(USER_CONFIG)*3)) != userConfigCheckSum
    ||  userConfigCheckSum != tempCheckSum)
        return false;
    else
        return true;
}

bool eraseUserConfigFromFlash(void)
{
    return (flashErase(USERCONFIG_ADDRESS)==0 ? true : false);
}

void defaultNoTrespassSetting(void)
{
    for(int i=0; i<8; i++)
    {
        noTrespassBorder[i].enable=false;
        noTrespassBorder[i].greater_than=false;
        noTrespassBorder[i].a=0;
        noTrespassBorder[i].b=0;
        noTrespassBorder[i].c=0;
        noTrespassBorder[i].d=0;
    }
    
}

void setNoTrespassBorder(int i, NO_TRESPASS_BORDER border){
    noTrespassBorder[i]=border;
}

void initNoTrespassBorder(void)
{
    uint32_t length = 0;
    
    flashRead(NO_TRESPASS_BORDER_ADDRESS, (uint32_t *) &noTrespassBorder, (sizeof(noTrespassBorder)/4));
    flashRead(ALIGN_SIZE(NO_TRESPASS_BORDER_ADDRESS+sizeof(noTrespassBorder), 4), &noTrespassBorderChecksum, 1);
    
    if (CalcChecksum((uint8_t *) noTrespassBorder, (sizeof(noTrespassBorder))) != noTrespassBorderChecksum)
    {
        printf("Failed to load border data. Use default settings.\n");
        defaultNoTrespassSetting();
    }
#if PRINT_USER_CONFIG_SUCCESS
    else
        printf("No trespass border init success\n");
#endif
}

bool saveNoTrespassBordersToFlash(void)
{
#pragma pack(4)
    NO_TRESPASS_BORDER tempNoTrespassBorder[8];
#pragma pack()
    uint32_t tempCheckSum;
    
    // Store userconfig into flash
    noTrespassBorderChecksum = CalcChecksum((uint8_t *) noTrespassBorder, (sizeof(noTrespassBorder)));
    
    if (flashErase(NO_TRESPASS_BORDER_ADDRESS) != 0)
        return false;
    
    flashWrite(NO_TRESPASS_BORDER_ADDRESS, (uint32_t *) noTrespassBorder, ((sizeof(noTrespassBorder))/sizeof(int)));
    flashWrite(ALIGN_SIZE(NO_TRESPASS_BORDER_ADDRESS+sizeof(noTrespassBorder), 4), &noTrespassBorderChecksum, 1);

    // Check data
    flashRead(NO_TRESPASS_BORDER_ADDRESS, (uint32_t *) tempNoTrespassBorder, ((sizeof(tempNoTrespassBorder))/sizeof(int)));
    flashRead(ALIGN_SIZE(NO_TRESPASS_BORDER_ADDRESS+sizeof(noTrespassBorder), 4), &tempCheckSum, 1);
    
    if (CalcChecksum((uint8_t *) &tempNoTrespassBorder, (sizeof(noTrespassBorder))) != noTrespassBorderChecksum
    ||  noTrespassBorderChecksum != tempCheckSum)
        return false;
    else
        return true;
}

bool eraseNoTrespassBordersFromFlash(void)
{
    return (flashErase(NO_TRESPASS_BORDER_ADDRESS)==0 ? true : false);
}

/////////////////////////////////////////

void defaultTorqueConfig(void)
{
    for(int i=0; i<3; i++)
    {
        torqueConfig[i].active_torque=CURRENT_100_PERCENT;
        torqueConfig[i].standby_torque=CURRENT_12_PERCENT;
        torqueConfig[i].__reserved_2=0;
        torqueConfig[i].__reserved_3=0;
    }
    
}

void setTorqueConfig(int i, TORQUE_CONFIG config){
    torqueConfig[i]=config;
}

void initTorqueConfig(void)
{
    uint32_t length = 0;
    
    flashRead(TORQUE_CONFIG_ADDRESS, (uint32_t *) torqueConfig, (sizeof(torqueConfig)/4));
    flashRead(ALIGN_SIZE(TORQUE_CONFIG_ADDRESS+sizeof(torqueConfig), 4), &torqueConfigChecksum, 1);
    
    if (CalcChecksum((uint8_t *) torqueConfig, (sizeof(torqueConfig))) != torqueConfigChecksum)
    {
        printf("Failed to load torque data. Use default settings.\n");
        defaultTorqueConfig();
    }
#if PRINT_USER_CONFIG_SUCCESS
    else
        printf("Torque config init success\n");
#endif
}

bool saveTorqueConfigToFlash(void)
{
#pragma pack(4)
    TORQUE_CONFIG tempTorqueConfig[3];
#pragma pack()
    uint32_t tempCheckSum;
    
    // Store userconfig into flash
    torqueConfigChecksum = CalcChecksum((uint8_t*)torqueConfig, (sizeof(torqueConfig)));
    
    if (flashErase(TORQUE_CONFIG_ADDRESS) != 0)
        return false;
    
    flashWrite(TORQUE_CONFIG_ADDRESS, (uint32_t*)torqueConfig, ((sizeof(torqueConfig))/sizeof(int)));
    flashWrite(ALIGN_SIZE(TORQUE_CONFIG_ADDRESS+sizeof(torqueConfig), 4), &torqueConfigChecksum, 1);
    
    // Check data
    flashRead(TORQUE_CONFIG_ADDRESS, (uint32_t*)tempTorqueConfig, ((sizeof(tempTorqueConfig))/sizeof(int)));
    flashRead(ALIGN_SIZE(TORQUE_CONFIG_ADDRESS+sizeof(torqueConfig), 4), &tempCheckSum, 1);
    
    if(CalcChecksum((uint8_t*)tempTorqueConfig, sizeof(torqueConfig))!=torqueConfigChecksum
    || torqueConfigChecksum != tempCheckSum)
        return false;
    else
        return true;
}

bool eraseTorqueConfigFromFlash(void)
{
    return (flashErase(TORQUE_CONFIG_ADDRESS)==0 ? true : false);
}