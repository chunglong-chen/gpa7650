#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#include "NuMicro.h"
#include "stdbool.h"
#include "gpio_type_def.h"

void initUserConfig(void);
void initNoTrespassBorder(void);
void initTorqueConfig(void);

void copySpeedToUserConfig(uint8_t motor_id);
void copyPinConfigToUserConfig(uint8_t motor_id);
void copyDirectionConfigToUserConfig(uint8_t motor_id);

bool saveUserConfigToFlash(void);
bool saveNoTrespassBordersToFlash(void);
bool saveTorqueConfigToFlash(void);

bool eraseUserConfigFromFlash(void);
bool eraseNoTrespassBordersFromFlash(void);
bool eraseTorqueConfigFromFlash(void);

#pragma pack(4)
typedef struct{
	GPIO_TYPEDef    highEdgePin; //left
    GPIO_TYPEDef    lowEdgePin;  //right
	uint32_t        fwdDirState;
	bool            enableMototAccelerate;
	uint32_t        defaultPPS;
	uint32_t        targetPPS;
	uint32_t        acceleration;
	uint32_t        deceleration;
	uint32_t        ratio;
}USER_CONFIG;

typedef struct{
    bool enable;
    bool greater_than;
    float a;
    float b;
    float c;
    float d;
}NO_TRESPASS_BORDER;

typedef struct{
    uint8_t  active_torque;
    uint8_t  standby_torque;
    uint8_t  __reserved_1;
    uint8_t  __reserved_2;
    uint32_t __reserved_3;
}TORQUE_CONFIG;

#pragma pack()

extern USER_CONFIG userConfig[3];
extern NO_TRESPASS_BORDER noTrespassBorder[8];
extern TORQUE_CONFIG torqueConfig[3];

void setNoTrespassBorder(int i, NO_TRESPASS_BORDER border);
void setTorqueConfig(int i, TORQUE_CONFIG config);

#endif //__USER_CONFIG_H__