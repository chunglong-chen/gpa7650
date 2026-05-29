#include "ina230.h"
#include "config/default/peripheral/port/plib_port.h"
#include "peripheral/systick/plib_systick.h"
#include "peripheral/sercom/i2c_master/plib_sercom5_i2c_master.h"
#include "peripheral/eic/plib_eic.h"
#include "sld.h"
bool ina230_read_reg(uint8_t reg, uint16_t * value);
bool ina230_write_reg(uint8_t reg, uint16_t value);

#define currentLSB  0.0001  //0.1mA
#define powerLSB    (25 * currentLSB)   //mW
#define shuntR  0.1 //100mR

INA230_DATA ina230_data;
static void EIC_User_Handler(uintptr_t context)
{
    sld_enable(false);
}
void INA230_Initialize(void) {
    INA230_CONFIG_REGISTER config;
    config.mode = ina230_shunt_bus_cont;
    config.average = ina230_avg_4;
    config.bus_ct = ina230_ct_140us;
    config.shunt_ct = ina230_ct_140us;
    config.reset = 0;
    config.reg |= 0x4000; //don't know why, not document
    ina230_write_reg(INA230_REGADD_CONFIG, config.reg);

    /**
     *  Shunt R : 100mR
     *  Current LSB expect to 0.1mA
     *  Cal register = 0.00512/0.1/0.0001 = 1024
     */

    uint16_t cali = 0.00512f / shuntR / currentLSB;
    ina230_write_reg(INA230_REGADD_CALIBRATION, cali);
    
//    if (ina230_read_reg(0x05, &cali)) {
//    printf("Read value: %x\r\n", cali);
//    } else {
//        printf("Read failed!\r\n");
//    }
    ina230_set_power_alert(2.0);
    
    EIC_CallbackRegister(EIC_PIN_2, EIC_User_Handler, 0);
    
}

bool ina230_read_reg(uint8_t reg, uint16_t * value) {
    bool ret = true;

    if (!SERCOM5_I2C_Write(INA230_BASE, &reg, 1)) {
        ret = false;
    }

    while (SERCOM5_I2C_IsBusy());

    if (!SERCOM5_I2C_Read(INA230_BASE, ina230_data.rxBuffer, 2)) {
        ret = false;
    }
    
    while (SERCOM5_I2C_IsBusy());

    /* Check if any error occurred */
    if (SERCOM5_I2C_ErrorGet() == SERCOM_I2C_ERROR_NONE) {
        *value = (ina230_data.rxBuffer[0] << 8) | ina230_data.rxBuffer[1]; // ???????
    } else {
        ret = false;
    }
    
    return ret;
}

bool ina230_write_reg(uint8_t reg, uint16_t value) {
    bool ret = true;
    uint8_t high_byte = value >> 8; // high byte
    uint8_t low_byte = value & 0xFF; // low byte  
    ina230_data.txBuffer[0] = reg;
    ina230_data.txBuffer[1] = high_byte; //high byte
    ina230_data.txBuffer[2] = low_byte; //low byte         
    SERCOM5_I2C_Write(INA230_BASE, &ina230_data.txBuffer[0], 3);

    while (SERCOM5_I2C_IsBusy());

    /* Check if any error occurred */
    if (SERCOM5_I2C_ErrorGet() == SERCOM_I2C_ERROR_NONE) {
        //Transfer is completed successfully.
    } else {
        //Error occurred during transfer.
        ret = false;
    }
    return ret;
}

uint16_t ina230_read_id(){
    uint16_t id;
    ina230_read_reg(INA230_REGADD_ID, &id);
    return id;
}

uint16_t ina230_read_alert(){
    uint16_t alert;
    ina230_read_reg(INA230_REGADD_MASK_EN, &alert);
    return alert;
} 


uint16_t ina230_read_shunt_volt(){
    uint16_t volt;
    ina230_read_reg(INA230_REGADD_SHUNT, &volt);
    return volt * 2.5;  //1 LSB = 2.5uV    
}

//read bus voltage in mv
uint16_t ina230_read_volt(){
    uint16_t volt;
    ina230_read_reg(INA230_REGADD_BUS, &volt);
    
    return volt * 1.25;    //1 LSB = 1.25mV
}

double ina230_read_current(){
    uint16_t current;
    ina230_read_reg(INA230_REGADD_CURRENT, &current);
    return current * currentLSB;
}

uint16_t ina230_read_power(){
    uint16_t power;
    ina230_read_reg(INA230_REGADD_POWER, &power);
    return power * powerLSB * 1000;    //in mW
}

bool ina230_set_power_alert(double watt){

    // TODO : Check how to use the INA230 alert function
    if (ina230_write_reg(INA230_REGADD_MASK_EN, 0)){  //disable alert
        return false;
    }
    //12V, 150mA, power = 1.8W, set 2W
    //power LSB 25mW/bit
    uint16_t pwrlimit = watt/powerLSB;
    if (!ina230_write_reg(INA230_REGADD_ALERTLimit, pwrlimit)){
        return false;
    }
    
    uint16_t alert = 1 << INA230_POWER_ALERT_OFFSET;   //alert power
    //alert |= 1; //alert latch mode
    if (!ina230_write_reg(INA230_REGADD_MASK_EN, alert)){  //enable alert
        return false;
    }
    return true;
}

double ina230_get_power_alert(){
    uint16_t power;
    ina230_read_reg(INA230_REGADD_ALERTLimit, &power);
    return power * powerLSB;
}
