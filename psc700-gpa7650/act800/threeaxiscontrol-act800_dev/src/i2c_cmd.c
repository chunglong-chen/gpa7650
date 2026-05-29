/**
 * @file i2c_cmd.c
 * @brief Implementation of I2C communication handler and command parser.
 * @ingroup i2c_command_parser
 * @details This file provides functions for sending, receiving, and parsing I2C commands.
 */
#include "i2c_cmd.h"
#define DEBUG_PRINT     0

bool CheckBit(uint8_t __cmd, uint8_t ___motorID);
bool CheckMotorBusy(uint8_t ___motorID);

void I2C_Package(uint8_t *package, uint8_t *index, uint8_t ___motorID, uint8_t packageType);
void I2C_ControlMotor(uint8_t *_SlvRecieveData, uint8_t controlType, uint32_t cmdLength);
void I2C_SetTorque(uint8_t *_SlvRecieveData);
bool I2C_SaveTorque();
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
/**
 * @brief Process and respond to I2C commands sent to the motor controller.
 *
 * @details This function interprets the command received via I2C, checks motor states if necessary,
 *          prepares response data, and determines whether to acknowledge (ACK) or not acknowledge (NACK)
 *          the command based on current system status.
 *
 * @param[in]  _cmd              
 * @param[in]  _motorID          
 * @param[in]  _SlvRecieveData   
 *
 * @retval ACK   The command is valid and can be processed.
 * @retval NACK  The command is invalid or cannot be processed at this time (e.g., motor is busy).
 */
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
        || (CheckBit(_motorID, MOTORADDR_3) && CheckMotorBusy(MOTORID_3))
        || (CheckBit(_motorID, MOTORADDR_4) && CheckMotorBusy(MOTORID_4))
        )
            return NACK;
        else
            return ACK;
    case SET_TORQUE:
        return ACK;
    case GET_STEP_BUSY:
        if (CheckBit(_motorID, MOTORADDR_1))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_1, PACK_BUSY);
        if (CheckBit(_motorID, MOTORADDR_2))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_2, PACK_BUSY);
        if (CheckBit(_motorID, MOTORADDR_3))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_3, PACK_BUSY);
        if (CheckBit(_motorID, MOTORADDR_4))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_4, PACK_BUSY);
        I2C0SlvTransmitDataIdx = dataIdx;
        return ACK;
    case GET_STEP_POS:
        if (CheckBit(_motorID, MOTORADDR_1))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_1, PACK_POS);
        if (CheckBit(_motorID, MOTORADDR_2))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_2, PACK_POS);
        if (CheckBit(_motorID, MOTORADDR_3))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_3, PACK_POS);
        if (CheckBit(_motorID, MOTORADDR_4))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_4, PACK_POS);
        I2C0SlvTransmitDataIdx = dataIdx;
        return ACK;
    case GET_MOTOR_CONFIG:
        if (CheckBit(_motorID, MOTORADDR_1))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_1, GET_MOTOR_CONFIG);
        if (CheckBit(_motorID, MOTORADDR_2))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_2, GET_MOTOR_CONFIG);
        if (CheckBit(_motorID, MOTORADDR_3))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_3, GET_MOTOR_CONFIG);
        if (CheckBit(_motorID, MOTORADDR_4))
            I2C_Package(_SlvTransmitData, &dataIdx, MOTORID_4, GET_MOTOR_CONFIG);
        I2C0SlvTransmitDataIdx = dataIdx;
        return ACK;
    case SET_MOTOR_DIR:
        return ACK;
    case SAVE_TORQUE:
        return I2C_SaveTorque();
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
/**
 * @brief I2C slave transmit/receive callback function.
 * Master Write Package: 0x60, 0x80, 0x80, ..., 0xA0
 * Master Read Package:  0xA8, 0xB8, 0xB8, ..., 0xC0
 * @param u32Status I2C status code obtained from hardware (e.g., I2C_GET_STATUS()).
 * @note This function is typically called from the I2C interrupt handler.
 *
 * @see I2C0_IRQHandler()
 * @see I2C_ProcessCmd()
 * @see I2C_ControlMotor()
 * @see I2C_SetStepSpeed()
 * @see I2C_SetMotorPinConfig()
 * @see I2C_SetNoTrespassBorder()
 */
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
                case SET_TORQUE:
                    I2C_SetTorque(SlvRecieveData);
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

/**
 * @brief I2C0 interrupt handler.
 *
 * @details This function handles the I2C0 interrupt. It checks for a timeout condition
 *          and clears the timeout flag if needed. If no timeout occurs, it delegates
 *          further processing to the @c I2C_SlaveTRx() function using the current status.
 *
 *          The handler is responsible for managing I2C slave transmission and reception.
 */
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

/**
 * @brief Packages motor data for I2C transmission based on the requested package type.
 *
 * @details This function handles I2C response packaging for various motor queries.
 *          It supports the following commands:
 *
 *   Motor status and position (Big-Endian) GET_STEP_STS
 *   Received:  `[0x4 | motorID]`
 *   Transmit:
 *   ```
 *   [motor1 sts] [motor1 pos B0] [motor1 pos B1] [motor1 pos B2] [motor1 pos B3]
 *   [motor2 sts] [motor2 pos B0] [motor2 pos B1] [motor2 pos B2] [motor2 pos B3]
 *   [motor3 sts] [motor3 pos B0] [motor3 pos B1] [motor3 pos B2] [motor3 pos B3]
 *   ```
 *
 *  **Motor busy status GET_STEP_BUSY**
 *   Received:  `[0x5 | motorID]` 
 *   Transmit:  `[motor1 sts] [motor2 sts] [motor3 sts]`
 *
 *  **Motor position only (Big-Endian) RESET_MOTOR_POS**
 *   Received:  `[0x6 | motorID]` 
 *   Transmit:
 *   ```
 *   [motor1 pos B0] [motor1 pos B1] [motor1 pos B2] [motor1 pos B3]
 *   [motor2 pos B0] [motor2 pos B1] [motor2 pos B2] [motor2 pos B3]
 *   [motor3 pos B0] [motor3 pos B1] [motor3 pos B2] [motor3 pos B3]
 *   ```
 *
 *  **Motor configuration (Big-Endian) GET_MOTOR_CONFIG**
 *   Received:  `[0x9 | motorID]` 
 *   Transmit (one motor at a time):
 *   ```
 *   [EnableAccel]
 *   [defpps B0] [defpps B1] [defpps B2] [defpps B3]
 *   [targetPPS B0] [targetPPS B1] [targetPPS B2] [targetPPS B3]
 *   [accel B0] [accel B1] [accel B2] [accel B3]
 *   [decel B0] [decel B1] [decel B2] [decel B3]
 *   ```
 *
 * @param package
 *
 * @param index
 *
 * @param ___motorID
 *
 * @param packageType
 *
 * @note Only the data for the inquired motor is packaged. This function is typically called when responding to I2C requests from the master.
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

/**
 * @brief I2C control motor function.
 *
 * @details This function handles motor control commands received via I2C.
 *          Supported commands include:
 *           Motor home: `[0x1 | motorID] [dummy]`
 *           Motor move to absolute position (Big-Endian):
 *              `[0x2 | motorID] [pos1 B0] [pos1 B1] [pos1 B2] [pos1 B3] [pos2 B0] ...`
 *           Motor move to relative position (Big-Endian):
 *              `[0x3 | motorID] [step1 B0] [step1 B1] [step1 B2] [step1 B3] [pos2 B0] ...`
 *
 * @param _SlvRecieveData 
 * @param controlType 
 * @param cmdLength 
 * @note This function is typically called from the @c I2C_SlaveTRx() routine.
 */
void I2C_ControlMotor(uint8_t *_SlvRecieveData, uint8_t controlType, uint32_t cmdLength)
{
    uint8_t __motorID = (_SlvRecieveData[0] & 0xF);
    uint8_t SlvRecieveDataIdx = 1;
    bool EnabledMotor[MOTOR_COUNT] = {0};
    uint8_t NumOfMotor = 0;
    int32_t PosReg = 0;
    int32_t MotorPos_1 = 0;
    int32_t MotorPos_2 = 0;
    int32_t MotorPos_3 = 0;
    int32_t MotorPos_4 = 0;
    int32_t BytePerPos = 0;

    // check incoming motorID command
    for (uint8_t i=0;i<MOTOR_COUNT;i++){
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

        for (uint32_t j=0;j<MOTOR_COUNT;j++)
        {
            if (EnabledMotor[j])
                motor_home(j);
        }
        return;
    }
    else if (controlType == RESET_MOTOR_POS)
    {
        for (uint32_t j=0;j<MOTOR_COUNT;j++)
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
    if (EnabledMotor[MOTORID_4])
    {
        for(int i=0; i<BytePerPos; i++)
            MotorPos_4 |= _SlvRecieveData[SlvRecieveDataIdx+i]<<(BytePerPos-i-1)*8;
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
                if (EnabledMotor[MOTORID_4])
                    motor_move_gate_abspos(MotorPos_4);
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
                if (EnabledMotor[MOTORID_4])
                    motor_move_gate_related(MotorPos_4);
            }
        break;

        default:
        break;
    }

}

/**
 * @brief Parses and sets motor speed parameters based on received I2C data.
 *
 * This function interprets incoming I2C packets (Big-Endian) to configure the step speed
 * and acceleration parameters of a specified motor.
 *
 * Two modes of operation are supported based on the `EnableAccel` flag in the packet:
 *
 *  Acceleration Enabled (`EnableAccel = 1`)
 *
 *   Received packet format:
 *   ```
 *   [0x8 | motorID] [EnableAccel]
 *   [defpps B0] [defpps B1] [defpps B2] [defpps B3]
 *   [targetPPS B0] [targetPPS B1] [targetPPS B2] [targetPPS B3]
 *   [accel B0] [accel B1] [accel B2] [accel B3]
 *   [decel B0] [decel B1] [decel B2] [decel B3]
 *   ```
 *   Where:
 *    `defpps` is the default pulses per second
 *    `targetPPS` is the target pulses per second
 *    `accel` is the acceleration rate
 *    `decel` is the deceleration rate
 *
 *   Acceleration Disabled (`EnableAccel = 0`)
 *
 *   Received packet format:
 *   ```
 *   [0x8 | motorID] [EnableAccel]
 *   [defpps B0] [defpps B1] [defpps B2] [defpps B3]
 *   ```
 *
 * @param[in] _SlvRecieveData Pointer to the received I2C buffer containing the configuration command.
 *
 * @note This function only configures the motor corresponding to the specified `motorID` in the received packet.
 *       The data should be arranged in Big-Endian format.
 */
void I2C_SetStepSpeed(uint8_t *_SlvRecieveData)
{
    MOTOR_DATA * motor;
    uint8_t __motorID = (_SlvRecieveData[0] & 0xF);
    uint8_t EnableAccel = _SlvRecieveData[1] & 0b1;
    bool EnabledMotor[MOTOR_COUNT] = {0};
    uint32_t defpps, targetPPS, accel, decel;

    // check incoming motorID command
    for (uint8_t i=0;i<MOTOR_COUNT;i++)
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
        if (EnabledMotor[MOTORID_4])
        {
            motor = get_motor_ref(MOTORID_4);
            motor_configSpeed(MOTORID_4, defpps, EnableAccel, targetPPS, accel, decel);
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
        if (EnabledMotor[MOTORID_4])
        {
            motor = get_motor_ref(MOTORID_4);
            motor_configSpeed(MOTORID_4, defpps, EnableAccel, motor->targetPPS, motor->acceleration, motor->deceleration);
        }
    }
}

/**
 * @brief Configures the direction and limit switch polarity of motors based on the received I2C command.
 *
 * This function parses a 2-byte I2C packet that contains motor ID and configuration bits for up to 3 motors.
 * Each motor has 2 bits assigned to it in the format `dir|pin`, where:
 * pin: Limit switch polarity configuration
 * dir: Motor direction configuration
 *
 * Received Packet Format @c SET_MOTOR_DIR
 * ```
 * [0xA | motorID] [0b00ZZYYXX]
 * ```
 * `motorID` (lower nibble of first byte): A bitmask indicating which motors to configure.
 *  XX`: Configuration bits for motor 1 (bit 1 = dir, bit 0 = pin)
 *  `YY`: Configuration bits for motor 2
 *  `ZZ`: Configuration bits for motor 3
 *
 * Only the motors with corresponding bits set in `motorID` will be updated.
 *
 * ### Example:
 * ```
 * [0xA | 0x05] [0b00000011]
 *  motorID = 0x05 (binary 00000101): Motor 1 and Motor 3 selected
 *  motor 1 config = 0b11: dir = 1 (reverse), pin = 1 (reverse)
 *  motor 3 config = 0b00: dir = 0 (default), pin = 0 (default)
 * ```
 *
 * @param[in] _SlvRecieveData Pointer to the received 2-byte I2C buffer.
 *
 * @note The actual motor index is determined by shifting `MOTORADDR_1` and checking against `motorID`.
 *       The limit switch and direction are configured using `setLimitSwitch()` and `setDirection()` respectively.
 */
void I2C_SetMotorPinConfig(uint8_t *_SlvRecieveData)
{
    uint8_t __motorID = (_SlvRecieveData[0] & 0xF);
    uint8_t motorPinConfig = _SlvRecieveData[1];
    for(int i=0; i<MOTOR_COUNT; i++){
        if(CheckBit(__motorID, (MOTORADDR_1 << i))){
            setLimitSwitch(i, motorPinConfig>>  (i*2)&0b1 ? CFG_LIMSW_REVERSE :
                                                            CFG_LIMSW_DEFAULT);
            setDirection(i,   motorPinConfig>>(i*2+1)&0b1 ? CFG_DIR_REVERSE :
                                                            CFG_DIR_DEFAULT);
        }
    }
}

/**
 * @brief Set driving torque based on the received I2C command.
 * This function sets driving torque (current) of each axis assigned by motor id.
 * Current can be set to 0-8, representing fraction of 8th of max driving torque.
 *
 * Received Packet Format @c SET_TORQUE
 * [0x4 | motorID] [Torque 0-8] [Torque 0-8] [Torque 0-8] [Torque 0-8]
 * [0x8 | G|Z|Y|X] {  1 byte  } {  1 byte  } {  1 byte  } {  1 byte  }
 *
 * @param[in] _SlvRecieveData Pointer to the received I2C buffer.
 * @return None
 */
void I2C_SetTorque(uint8_t *_SlvRecieveData)
{
    uint8_t __motorID = (_SlvRecieveData[0] & 0xF);
    uint8_t idx=1;
    for(int i=0; i<MOTOR_COUNT; i++){
        if(CheckBit(__motorID, (MOTORADDR_1 << i))){
            uint8_t torque = _SlvRecieveData[idx++];
            if(torque<0 || torque>8) continue;
            select_current(torque, &(get_motor_ref(i)->activeI0State),
                                   &(get_motor_ref(i)->activeI1State));
         }
    }
}

/**
 * @brief  Save driving torque to flash.
 * @param  None
 * @retval true  Save success.
 * @retval false Save failed.
 */
bool I2C_SaveTorque()
{
    for(int id=0; id<MOTOR_COUNT; id++){
    torqueConfig[id].active_torque=parse_current_setting(
                                        get_motor_ref(id)->activeI0State,
                                        get_motor_ref(id)->activeI1State);
    }
    return saveTorqueConfigToFlash() ? ACK : NACK;
}

/**
 * @brief Set no trespass border based on the received I2C command.
 *
 *  Received Packet Format @c SET_NO_TRESPASS
 * [cmd | index] [                    flag] [coef. a][][][] [coef. b][][][] [coef. c][][][] [coef. d][][][]
 *
 * [0x8 |   0-8] [greater_than<<1 | enable] {4 bytes float} {4 bytes float} {4 bytes float} {4 bytes float}
 *
 * 8 trespass borders can be set
 *
 * if greater_than is set to 1, any posiiton fulfills ax + by + cz > d is restricted
 *
 * if greater_than is set to 0, any posiiton fulfills ax + by + cz < d is restricted
 *
 * @param[in] _SlvRecieveData Pointer to the received I2C buffer.
 * @return None
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

/**
 * @brief  Save user config to flash.
 * @param  None
 * @retval true  Save success.
 * @retval false Save failed.
 */
bool I2C_SaveMotorParameterToFlash(void)
{
    for(int i=0; i<MOTOR_COUNT; i++){
        copySpeedToUserConfig(i);
        copyDirectionConfigToUserConfig(i);
        copyPinConfigToUserConfig(i);
    }
    return saveUserConfigToFlash() ? ACK : NACK;
}

/**
 * @brief  Save NoTrespassBorder to flash.
 * @param  None
 * @retval true  Save success.
 * @retval false Save failed.
 */
bool I2C_SaveNoTrespassBordersToFlash(void)
{
    return saveNoTrespassBordersToFlash() ? ACK : NACK;
}

/**
 * @brief  Check if the specified motor ID bit is set.
 *
 * This function checks whether the bit corresponding to a specific motor (`___motorID`)
 * is set in the received I2C motor ID bitmask (`inputMotorID`).
 *
 * @param  inputMotorID Bitmask of motor IDs received via I2C.
 * @param  ___motorID   The bitmask representing a single motor (e.g., MOTORADDR_1 << n).
 *
 * @retval true  The motor bit is set in inputMotorID (i.e., the motor is selected).
 * @retval false The motor bit is not set (i.e., the motor is not selected).
 */
bool CheckBit(uint8_t inputMotorID, uint8_t ___motorID)
{
    return (inputMotorID & ___motorID);
}

/**
 * @brief  Check motor is busy or not.
 * @param  ___motorID The ID (index) of the motor.
 * @retval true  The motor is busy.
 * @retval false The motor is idle.
 */
bool CheckMotorBusy(uint8_t ___motorID)
{
    MOTOR_DATA * motor;
    motor = get_motor_ref(___motorID);
    return *(motor->busyPin.datAddr);
}


