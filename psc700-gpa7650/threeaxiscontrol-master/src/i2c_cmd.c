#include "i2c_cmd.h"
#define DEBUG_PRINT     0

bool CheckBit(uint8_t __cmd, uint8_t ___motorID);
bool CheckMotorBusy(uint8_t ___motorID);

void I2C_Package(uint8_t *package, uint8_t *index, uint8_t ___motorID, uint8_t packageType);
void I2C_ControlMotor(uint8_t *_SlvRecieveData, uint8_t controlType, uint32_t cmdLength);
void I2C_SetStepSpeed(uint8_t *_SlvRecieveData);
void I2C_SetMotorPinConfig(uint8_t *_SlvRecieveData);
void I2C_SetNoTrespassBorder(uint8_t *_SlvRecieveData);
bool I2C_SaveMotorParameterToFlash(void);
bool I2C_SaveNoTrespassBordersToFlash(void);

uint8_t I2C0SlvRecieveData[32];
uint8_t I2C0SlvTransmitData[32];
uint8_t I2C0SlvRecieveDataIdx = 0;
uint8_t I2C0SlvTransmitDataIdx = 0;

// instantly response once "register address byte" is arrived
bool I2C_ProcessCmd(uint8_t _cmd, uint8_t _motorID, uint8_t *_SlvRecieveData, uint8_t *_SlvTransmitData)
{
    uint8_t dataIdx = 0;
    switch(_cmd){
    case GET_VERSION:
        I2C_Package(_SlvTransmitData, &dataIdx, 0, PACK_VERSION);  // 0 means motor id is unnecessary
        I2C0SlvTransmitDataIdx = dataIdx;
        return ACK;
    case STEP_MOVE_HOME:
    case STEP_MOVE_ABS:
    case STEP_MOVE_REL:
    case RESET_MOTOR_POS:
    case MOTOR_CONFIG_SPEED:
        if((CheckBit(_motorID, MOTORADDR_1) && CheckMotorBusy(MOTORID_1))
        || (CheckBit(_motorID, MOTORADDR_2) && CheckMotorBusy(MOTORID_2))
        || (CheckBit(_motorID, MOTORADDR_3) && CheckMotorBusy(MOTORID_3)))
            return NACK;
        else
            return ACK;
    case GET_STEP_STS:
        if (CheckBit(_motorID, MOTORADDR_1))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_1, PACK_STS_AND_POS);
        if (CheckBit(_motorID, MOTORADDR_2))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_2, PACK_STS_AND_POS);
        if (CheckBit(_motorID, MOTORADDR_3))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_3, PACK_STS_AND_POS);
        I2C0SlvTransmitDataIdx = dataIdx;
        return ACK;
    case GET_STEP_BUSY:
        if (CheckBit(_motorID, MOTORADDR_1))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_1, PACK_BUSY);
        if (CheckBit(_motorID, MOTORADDR_2))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_2, PACK_BUSY);
        if (CheckBit(_motorID, MOTORADDR_3))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_3, PACK_BUSY);
        I2C0SlvTransmitDataIdx = dataIdx;
        return ACK;
    case GET_STEP_POS:
        if (CheckBit(_motorID, MOTORADDR_1))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_1, PACK_POS);
        if (CheckBit(_motorID, MOTORADDR_2))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_2, PACK_POS);
        if (CheckBit(_motorID, MOTORADDR_3))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_3, PACK_POS);
        I2C0SlvTransmitDataIdx = dataIdx;
        return ACK;
    case GET_MOTOR_CONFIG:
        if (CheckBit(_motorID, MOTORADDR_1))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_1, GET_MOTOR_CONFIG);
        if (CheckBit(_motorID, MOTORADDR_2))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_2, GET_MOTOR_CONFIG);
        if (CheckBit(_motorID, MOTORADDR_3))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_3, GET_MOTOR_CONFIG);
        I2C0SlvTransmitDataIdx = dataIdx;
        return ACK;
    case SET_MOTOR_DIR:
        return ACK;
    case RESERVED_0XB:      //deprecated function, keep for compatibility
                            //0xB was store speed (copy speed to user config)
        return ACK;
    case SAVE_MOTOR_PARA:
        return I2C_SaveMotorParameterToFlash();
    case SET_NO_TRESPASS:
        // use 4 bit _motorID as index: maximum of 8 borders
        return _motorID < 8 ? ACK : NACK;
    case SAVE_NO_TRESPASS:
        return I2C_SaveNoTrespassBordersToFlash();
    default:
        return NACK;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                              */
/*  Master Write Package                                                                                   */
/*  0x60 0x80 0x80 ... 0x80 0xA0                                                                           */
/*  Master Read Package                                                                                    */
/*  0xA8 0xB8 0xB8 ... 0xB8 0xC0                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    uint8_t u8Data;
    uint8_t *SlvRecieveData = I2C0SlvRecieveData;
    uint8_t *SlvTransmitData = I2C0SlvTransmitData;
    uint16_t u16Data;
    static uint8_t cmd = 0xFF;
    static uint8_t motorID = 0x00;
    uint8_t idx;

#if DEBUG_PRINT
    printf("u32Status : 0x%x\n", u32Status);
#endif

    switch (u32Status)
    {
        
        case 0x60:    // Write from Master START
            I2C0SlvRecieveDataIdx = 0;
            I2C0SlvTransmitDataIdx = 0;
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        break;

        case 0x80:    // Write from Master DATA
            u8Data = (unsigned char) I2C_GET_DATA(I2C0);
            SlvRecieveData[I2C0SlvRecieveDataIdx] = u8Data;
            if (I2C0SlvRecieveDataIdx == 0)           // cmd always at address 0
            {
                cmd     = SlvRecieveData[I2C0SlvRecieveDataIdx] >>  4;
                motorID = SlvRecieveData[I2C0SlvRecieveDataIdx] & 0xF;
                if (I2C_ProcessCmd(cmd,motorID,SlvRecieveData,SlvTransmitData))
                    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
                else
                    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
            }
            else if (I2C0SlvRecieveDataIdx >= 28)
            {
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
                printf("Command length out of range\n");
            }
            else
            {
                I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
            }
            I2C0SlvRecieveDataIdx++;
        break;

        case 0xA0:    // Write from Master STOP or Repeated START
        #if DEBUG_PRINT
            // For debug to see all the recieved packages
            idx = I2C0SlvRecieveDataIdx;
            for (;I2C0SlvRecieveDataIdx>0;I2C0SlvRecieveDataIdx--)
                printf("data[%d] = 0x%x, %d\n", (idx-I2C0SlvRecieveDataIdx), (SlvRecieveData[idx-I2C0SlvRecieveDataIdx]),(SlvRecieveData[idx-I2C0SlvRecieveDataIdx]));
            I2C0SlvRecieveDataIdx = idx;

            printf("cmd : 0x%x, motorID : 0x%x\n",cmd,motorID);
        #endif
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
            switch(cmd){
                case STEP_MOVE_HOME:
                case STEP_MOVE_ABS:
                case STEP_MOVE_REL:
                case RESET_MOTOR_POS:
                    I2C_ControlMotor(SlvRecieveData, cmd, I2C0SlvRecieveDataIdx);
                    break;
                case MOTOR_CONFIG_SPEED:
                    I2C_SetStepSpeed(SlvRecieveData);
                    break;
                case SET_MOTOR_DIR:
                    I2C_SetMotorPinConfig(SlvRecieveData);
                    break;
                case SET_NO_TRESPASS:
                    I2C_SetNoTrespassBorder(SlvRecieveData);
                    break;
            }
        break;

        case 0x88:    // NACK while Master write data
            I2C0SlvRecieveDataIdx = 0;
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        break;

        case 0xA8:    // Read from Master START
            I2C0SlvTransmitDataIdx = 0;
        case 0xB8:    // Read from Master DATA
            switch (cmd)
            {
                case GET_VERSION:
                case GET_STEP_STS:
                case GET_STEP_BUSY:
                case GET_STEP_POS:
                case GET_MOTOR_CONFIG:
                    I2C_SET_DATA(I2C0, SlvTransmitData[I2C0SlvTransmitDataIdx++]);
                default:
                    break;
            }
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        break;
        
        case 0xC0:    // Read from Master STOP
        case 0xC8:    // NACK while Master read data
            I2C0SlvTransmitDataIdx = 0;
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        break;
        
        default:
        #if DEBUG_PRINT
            printf("Status 0x%x is NOT processed\n", u32Status);
        #endif
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        break;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;
    u32Status = I2C_GET_STATUS(I2C0);
    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
        printf("I2C0_IRQHandler Timeout\n");
    }
    else
        I2C_SlaveTRx(u32Status);
}


/*
 * Ask motor status  (Big-Endian)
 * Recieved:
 * [0x4 | motorID]
 * Transmit:
 * [motor1 sts] [motor1 pos B0] [motor1 pos B1] [motor1 pos B2] [motor1 pos B3]
 * [motor2 sts] [motor2 pos B0] [motor2 pos B1] [motor2 pos B2] [motor2 pos B3]
 * [motor3 sts] [motor3 pos B0] [motor3 pos B1] [motor3 pos B2] [motor3 pos B3]
 *
 * Ask motor busy
 * Recieved:
 * [0x5 | motorID]
 * Transmit:
 * [motor1 sts] [motor2 sts] [motor3 sts]
 *
 * Ask motor position  (Big-Endian)
 * Recieved:
 * [0x6 | motorID]
 * Transmit:
 * [motor1 pos B0] [motor1 pos B1] [motor1 pos B2] [motor1 pos B3]
 * [motor2 pos B0] [motor2 pos B1] [motor2 pos B2] [motor2 pos B3]
 * [motor3 pos B0] [motor3 pos B1] [motor3 pos B2] [motor3 pos B3]
 *
 * Ask motor configs  (Big-Endian)
 * Recieved:
 * [0x9 | motorID]
 * Transmit: (Only one set of motor parameters can be sent out at a time)
 * [EnableAccel] [defpps B0] [defpps B1] [defpps B2] [defpps B3] [targetPPS B0] [targetPPS B1] [targetPPS B2] [targetPPS B3]
 *               [accel B0] [accel B1] [accel B2] [accel B3] [decel B0] [decel B1] [decel B2] [decel B3]
 *
 * Only package the information of the motor inquired
 */
void I2C_Package(uint8_t *package, uint8_t *index, uint8_t ___motorID, uint8_t packageType)
{
    uint8_t u8data;
    MOTOR_DATA * motor = get_motor_ref(___motorID);
    int32_t i32data;
    uint32_t u32data;
    i32data = motor->position;
    uint8_t u8version[] = VERSION;
    uint16_t i;

#if DEBUG_PRINT
    printf("%d %d %d %d\n", 
        motor->position, 
        !*(motor->highEdgePin.datAddr),
        !*(motor->lowEdgePin.datAddr),
        *(motor->busyPin.datAddr)
    );
#endif
    if (packageType == PACK_STS_AND_POS)
    {

        package[(*index)] = ((!*(motor->highEdgePin.datAddr)) << 2) | 
                             ((!*(motor->lowEdgePin.datAddr)) << 1) | 
                             (*(motor->busyPin.datAddr));
        (*index)++;
        package[(*index)] = i32data >> 24;
        (*index)++;
        package[(*index)] = i32data >> 16;
        (*index)++;
        package[(*index)] = i32data >> 8;
        (*index)++;
        package[(*index)] = i32data & 0xFF;
        (*index)++;
    #if DEBUG_PRINT
        printf("packall : [0x%x] [0x%x] [0x%x] [0x%x] [0x%x]  index : %d\n", package[(*index)-5], package[(*index)-4], package[(*index)-3], package[(*index)-2], package[(*index)-1], (*index));
    #endif
    }
    else if (packageType == PACK_BUSY)
    {
        package[(*index)] = ((!*(motor->highEdgePin.datAddr)) << 2) | 
                             ((!*(motor->lowEdgePin.datAddr)) << 1) | 
                             (*(motor->busyPin.datAddr));
        (*index)++;
        
    #if DEBUG_PRINT
        printf("packsts : [0x%x]  index : %d\n", package[(*index)-1], (*index));
    #endif
    }
    else if (packageType == PACK_POS)
    {
        package[(*index)] = i32data >> 24;
        (*index)++;
        
        package[(*index)] = i32data >> 16;
        (*index)++;
        
        package[(*index)] = i32data >> 8;
        (*index)++;

        package[(*index)] = i32data & 0xFF;
        (*index)++;
    #if DEBUG_PRINT
        printf("packpos : [0x%x] [0x%x] [0x%x] [0x%x]  index : %d\n", package[(*index)-4], package[(*index)-3], package[(*index)-2], package[(*index)-1], (*index));
    #endif
    }
    else if (packageType == GET_MOTOR_CONFIG)
    {
        package[(*index)] = motor->enableMototAccelerate;
        (*index)++;
        
        u32data = motor->defaultPPS;
        package[(*index)] = u32data >> 24;
        (*index)++;
        package[(*index)] = u32data >> 16;
        (*index)++;
        package[(*index)] = u32data >> 8;
        (*index)++;
        package[(*index)] = u32data & 0xFF;
        (*index)++;

        u32data = motor->targetPPS;
        package[(*index)] = u32data >> 24;
        (*index)++;
        package[(*index)] = u32data >> 16;
        (*index)++;
        package[(*index)] = u32data >> 8;
        (*index)++;
        package[(*index)] = u32data & 0xFF;
        (*index)++;

        u32data = motor->acceleration;
        package[(*index)] = u32data >> 24;
        (*index)++;
        package[(*index)] = u32data >> 16;
        (*index)++;
        package[(*index)] = u32data >> 8;
        (*index)++;
        package[(*index)] = u32data & 0xFF;
        (*index)++;

        u32data = motor->deceleration;
        package[(*index)] = u32data >> 24;
        (*index)++;
        package[(*index)] = u32data >> 16;
        (*index)++;
        package[(*index)] = u32data >> 8;
        (*index)++;
        package[(*index)] = u32data & 0xFF;
        (*index)++;

    #if DEBUG_PRINT
        printf("packmotor :");
        for (uint8_t i=(*index);i>0;i--)
            printf(" [0x%x] ", package[(*index)-i]);
        printf(" index : %d\n", (*index));
    #endif
    }
    else if (packageType == PACK_VERSION)
    {
        // 20211014 Ting add
        // I know it should use strncpy...
        for (i=0;i<sizeof(u8version);i++)
        {
            package[(*index)] = u8version[i];
            (*index)++;
        }
    #if DEBUG_PRINT
        for (i=0;i<sizeof(u8version);i++)
        {
            printf("%c",package[i]);
        }
        printf("\r\n");
    #endif
    }
    else
        printf("Unknown package type\n");
}


/*
 * Motor back to home
 * [0x1 | motorID] [dummy]
 *
 * Motor go to abs position (Big-Endian)
 * [0x2 | motorID] [pos1 B0] [pos1 B1] [pos1 B2] [pos1 B3] [pos2 B0] [pos2 B1] [pos2 B2] [pos2 B3] [pos3 B0] [pos3 B1] [pos3 B2] [pos3 B3]
 *
 * Motor go to rel position (Big-Endian)
 * [0x3 | motorID] [step1 B0] [step1 B1] [step1 B2] [step1 B3] [step2 B0] [step2 B1] [step2 B2] [step2 B3] [step3 B0] [step3 B1] [step3 B2] [step3 B3]
 */
void I2C_ControlMotor(uint8_t *_SlvRecieveData, uint8_t controlType, uint32_t cmdLength)
{
    uint8_t __motorID = (_SlvRecieveData[0] & 0xF);
    uint8_t SlvRecieveDataIdx = 1;
    bool EnabledMotor[3] = {0};
    uint8_t NumOfMotor = 0;
    int32_t PosReg = 0;
    int32_t MotorPos_1 = 0;
    int32_t MotorPos_2 = 0;
    int32_t MotorPos_3 = 0;
    int32_t BytePerPos = 0;

    // check incoming motorID command
    for (uint8_t i=0;i<3;i++){
        if(CheckBit(__motorID, (MOTORADDR_1 << i))){
            EnabledMotor[i] = true;
            NumOfMotor++;
        }
    }
    
    BytePerPos=(cmdLength-1)/NumOfMotor;
    
#if DEBUG_PRINT
    printf("EnabledMotor[0] = %d, EnabledMotor[1] = %d, EnabledMotor[2] = %d\n",
            EnabledMotor[0],      EnabledMotor[1],      EnabledMotor[2]);
#endif

    // do motor control command without position byte
    if (controlType == STEP_MOVE_HOME)
    {
        if (EnabledMotor[MOTORID_1] && EnabledMotor[MOTORID_2] && EnabledMotor[MOTORID_3])
        {
            motor_home3();
            return;
        }

        for (uint32_t j=0;j<3;j++)
        {
            if (EnabledMotor[j])
                motor_home(j);
        }
        return;
    }
    else if (controlType == RESET_MOTOR_POS)
    {
        for (uint32_t j=0;j<3;j++)
        {
            if (EnabledMotor[j])
                motor_resetPos(j);
        }
        return;
    }

    // parse position byte
    if (EnabledMotor[MOTORID_1])
    {
        for(int i=0; i<BytePerPos; i++)
            MotorPos_1 |= _SlvRecieveData[SlvRecieveDataIdx+i]<<(BytePerPos-i-1)*8;
        SlvRecieveDataIdx += BytePerPos;
    }
    if (EnabledMotor[MOTORID_2])
    {
        for(int i=0; i<BytePerPos; i++)
            MotorPos_2 |= _SlvRecieveData[SlvRecieveDataIdx+i]<<(BytePerPos-i-1)*8;
        SlvRecieveDataIdx += BytePerPos;
    }
    if (EnabledMotor[MOTORID_3])
    {
        for(int i=0; i<BytePerPos; i++)
            MotorPos_3 |= _SlvRecieveData[SlvRecieveDataIdx+i]<<(BytePerPos-i-1)*8;
    }

#if DEBUG_PRINT
    printf("MotorPos_1 = %d, MotorPos_2 = %d, MotorPos_3 = %d\n", 
            MotorPos_1,      MotorPos_2,      MotorPos_3);
#endif

    // do motor control command with position
    switch (controlType)
    {
        case STEP_MOVE_ABS:
            if (EnabledMotor[MOTORID_1] && EnabledMotor[MOTORID_2] && EnabledMotor[MOTORID_3])
                motor_move_abspos3(MotorPos_1, MotorPos_2, MotorPos_3);
            else
            {
                if (EnabledMotor[MOTORID_1])
                    motor_move_abspos(MOTORID_1, MotorPos_1);
                if (EnabledMotor[MOTORID_2])
                    motor_move_abspos(MOTORID_2, MotorPos_2);
                if (EnabledMotor[MOTORID_3])
                    motor_move_abspos(MOTORID_3, MotorPos_3);
            }
        break;

        case STEP_MOVE_REL:    
            if (EnabledMotor[MOTORID_1] && EnabledMotor[MOTORID_2] && EnabledMotor[MOTORID_3])
                motor_move_related3(MotorPos_1, MotorPos_2, MotorPos_3);
            else
            {
                if (EnabledMotor[MOTORID_1])
                    motor_move_related(MOTORID_1, MotorPos_1);
                if (EnabledMotor[MOTORID_2])
                    motor_move_related(MOTORID_2, MotorPos_2);
                if (EnabledMotor[MOTORID_3])
                    motor_move_related(MOTORID_3, MotorPos_3);
            }
        break;

        default:
        break;
    }

}


/*
 * If EnableAccel = 1, recieved packages as below: (Big-Endian)
 * [0x8 | motorID] [EnableAccel] [defpps B0] [defpps B1] [defpps B2] [defpps B3] [targetPPS B0] [targetPPS B1] [targetPPS B2] [targetPPS B3]
 *                               [accel B0] [accel B1] [accel B2] [accel B3] [decel B0] [decel B1] [decel B2] [decel B3]
 *
 * If EnableAccel = 0, recieved packages as below: (Big-Endian)
 * [0x8 | motorID] [EnableAccel] [defpps B0] [defpps B1] [defpps B2] [defpps B3]
 */
void I2C_SetStepSpeed(uint8_t *_SlvRecieveData)
{
    MOTOR_DATA * motor;
    uint8_t __motorID = (_SlvRecieveData[0] & 0xF);
    uint8_t EnableAccel = _SlvRecieveData[1] & 0b1;
    bool EnabledMotor[3] = {0};
    uint32_t defpps, targetPPS, accel, decel;

    // check incoming motorID command
    for (uint8_t i=0;i<3;i++)
        EnabledMotor[i] = CheckBit(__motorID, (MOTORADDR_1 << i));

#if DEBUG_PRINT
    printf("EnabledMotor[0] = %d, EnabledMotor[1] = %d, EnabledMotor[2] = %d\n", EnabledMotor[0], EnabledMotor[1], EnabledMotor[2]);
#endif

    if (EnableAccel == 1)   // Enable acceleration
    {
        // Check speed cmd correctness, cmd should be 18 bytes
        if (I2C0SlvRecieveDataIdx != 18)
            return;

        defpps    = ( _SlvRecieveData[2] << 24) | ( _SlvRecieveData[3] << 16)
                  | ( _SlvRecieveData[4] << 8)  |   _SlvRecieveData[5];
        
        targetPPS = ( _SlvRecieveData[6] << 24) | ( _SlvRecieveData[7] << 16)
                  | ( _SlvRecieveData[8] << 8)  |   _SlvRecieveData[9];
        
        accel     = (_SlvRecieveData[10] << 24) | (_SlvRecieveData[11] << 16)
                  | (_SlvRecieveData[12] << 8)  |  _SlvRecieveData[13];
        
        decel     = (_SlvRecieveData[14] << 24) | (_SlvRecieveData[15] << 16)
                  | (_SlvRecieveData[16] << 8)  |  _SlvRecieveData[17];
                

    #if DEBUG_PRINT
        printf("defpps = %u, targetPPS = %u, accel = %u, decel = %u\n", defpps, targetPPS, accel, decel);
    #endif
        
        if (EnabledMotor[MOTORID_1])
        {
            motor = get_motor_ref(MOTORID_1);
            motor_configSpeed(MOTORID_1, defpps, EnableAccel, targetPPS, accel, decel);
        }
        if (EnabledMotor[MOTORID_2])
        {
            motor = get_motor_ref(MOTORID_2);
            motor_configSpeed(MOTORID_2, defpps, EnableAccel, targetPPS, accel, decel);
        }
        if (EnabledMotor[MOTORID_3])
        {
            motor = get_motor_ref(MOTORID_3);
            motor_configSpeed(MOTORID_3, defpps, EnableAccel, targetPPS, accel, decel);
        }
    }
    else if (EnableAccel == 0) // Disable acceleration
    {
        if (I2C0SlvRecieveDataIdx != 6)  // Check speed cmd correctness, cmd should be 4 bytes
            return;
        
        defpps = (_SlvRecieveData[2] << 24) | (_SlvRecieveData[3] << 16)
               | (_SlvRecieveData[4] << 8)  |  _SlvRecieveData[5];

    #if DEBUG_PRINT
        printf("defpps = %u\n", defpps);
    #endif
        
        if (EnabledMotor[MOTORID_1])
        {
            motor = get_motor_ref(MOTORID_1);
            motor_configSpeed(MOTORID_1, defpps, EnableAccel, motor->targetPPS, motor->acceleration, motor->deceleration);
        }
        if (EnabledMotor[MOTORID_2])
        {
            motor = get_motor_ref(MOTORID_2);
            motor_configSpeed(MOTORID_2, defpps, EnableAccel, motor->targetPPS, motor->acceleration, motor->deceleration);
        }
        if (EnabledMotor[MOTORID_3])
        {
            motor = get_motor_ref(MOTORID_3);
            motor_configSpeed(MOTORID_3, defpps, EnableAccel, motor->targetPPS, motor->acceleration, motor->deceleration);
        }
    }
}

/*
 * Check status bit to decide which pin need to switch
 * Motor go to abs position
 * [0xA | motorID] [0b00ZZYYXX]
 * XX = (motor 1 dir) | (motor 1 pin) , dir is the higher bit
 * YY = (motor 2 dir) | (motor 2 pin) , dir is the higher bit
 * ZZ = (motor 3 dir) | (motor 3 pin) , dir is the higher bit
 */
void I2C_SetMotorPinConfig(uint8_t *_SlvRecieveData)
{
    uint8_t __motorID = (_SlvRecieveData[0] & 0xF);
    uint8_t motorPinConfig = _SlvRecieveData[1];
    for(int i=0; i<3; i++){
        if(CheckBit(__motorID, (MOTORADDR_1 << i))){
            setLimitSwitch(i, motorPinConfig>>  (i*2)&0b1 ? CFG_LIMSW_REVERSE :
                                                            CFG_LIMSW_DEFAULT);
            setDirection(i,   motorPinConfig>>(i*2+1)&0b1 ? CFG_DIR_REVERSE :
                                                            CFG_DIR_DEFAULT);
        }
    }
}

/* 
 * Set no trespass border
 * [cmd | index] [                    flag] [coef. a][][][] [coef. b][][][] [coef. c][][][] [coef. d][][][]
 * [0x8 |   0-8] [greater_than<<1 | enable] {4 bytes float} {4 bytes float} {4 bytes float} {4 bytes float}
 * 8 trespass borders can be set
 * if greater_than is set to 1, any posiiton fulfills ax + by + cz > d is restricted
 * if greater_than is set to 0, any posiiton fulfills ax + by + cz < d is restricted
 */
void I2C_SetNoTrespassBorder(uint8_t *_SlvRecieveData)
{
    // 8 trespass borders are allowed
    uint8_t border_index = (_SlvRecieveData[0] & 0x7);
    uint32_t tmp;
    
    noTrespassBorder[border_index].enable=_SlvRecieveData[1] & 0b1;
    if(noTrespassBorder[border_index].enable)
    {
        noTrespassBorder[border_index].greater_than=(_SlvRecieveData[1]>>1) & 0b1;
        
        tmp=(uint32_t)(_SlvRecieveData[2]<<24
                      |_SlvRecieveData[3]<<16
                      |_SlvRecieveData[4]<<8
                      |_SlvRecieveData[5]);
        noTrespassBorder[border_index].a=*((float*)(&tmp));
        
        tmp=(uint32_t)(_SlvRecieveData[6]<<24
                      |_SlvRecieveData[7]<<16
                      |_SlvRecieveData[8]<<8
                      |_SlvRecieveData[9]);
        noTrespassBorder[border_index].b=*((float*)(&tmp));
        
        tmp=(uint32_t)(_SlvRecieveData[10]<<24
                      |_SlvRecieveData[11]<<16
                      |_SlvRecieveData[12]<<8
                      |_SlvRecieveData[13]);
        noTrespassBorder[border_index].c=*((float*)(&tmp));
        
        tmp=(int)(_SlvRecieveData[14]<<24
                 |_SlvRecieveData[15]<<16
                 |_SlvRecieveData[16]<<8
                 |_SlvRecieveData[17]);
        noTrespassBorder[border_index].d=*((float*)(&tmp));
    }
#if DEBUG_PRINT
    printf("no transpass border[%d]: %fx + %fy + %fz %s %f\n",
            border_index,
            noTrespassBorder[border_index].a,
            noTrespassBorder[border_index].b,
            noTrespassBorder[border_index].c,
            (noTrespassBorder[border_index].greater_than? ">":"<"),
            noTrespassBorder[border_index].d);
#endif
}

// Store user config into flash
// [0xC0]
bool I2C_SaveMotorParameterToFlash(void)
{
    for(int i=0; i<3; i++){
        copySpeedToUserConfig(i);
        copyDirectionConfigToUserConfig(i);
        copyPinConfigToUserConfig(i);
    }
    return saveUserConfigToFlash() ? ACK : NACK;
}

bool I2C_SaveNoTrespassBordersToFlash(void)
{
    return saveNoTrespassBordersToFlash() ? ACK : NACK;
}

bool CheckBit(uint8_t inputMotorID, uint8_t ___motorID)
{
    return (inputMotorID & ___motorID);
}

bool CheckMotorBusy(uint8_t ___motorID)
{
    MOTOR_DATA * motor;
    motor = get_motor_ref(___motorID);
    return *(motor->busyPin.datAddr);
}


