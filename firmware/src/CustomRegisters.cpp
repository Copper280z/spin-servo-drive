
#include "regs/CustomRegisters.h"
#include "motor/FFBLDCMotor.h"
CustomRegisters::CustomRegisters() {};
CustomRegisters::~CustomRegisters() {};

uint8_t CustomRegisters::sizeOfRegister(uint8_t reg) {
    switch (reg) {
        case REG_CURRENT_SP:
            return 4;        
        case REG_VOLTAGE_QFF:
            return 4;        
        case REG_VOLTAGE_DFF:
            return 4;
        default:
            return this->SimpleFOCRegisters::sizeOfRegister(reg);
    }
};


bool CustomRegisters::registerToComms(RegisterIO& comms, uint8_t reg, FOCMotor* motor) {
    switch (reg) {
        case REG_CURRENT_SP:
            comms << motor->current_sp;
            return true;
        case REG_VOLTAGE_QFF:
            comms << ((FFBLDCMotor*) motor)->Uq;
            return true;
        case REG_VOLTAGE_DFF:
            comms << ((FFBLDCMotor*) motor)->Ud;
            return true;
        default:
            return this->SimpleFOCRegisters::registerToComms(comms, reg, motor);
    }
};


bool CustomRegisters::commsToRegister(RegisterIO& comms, uint8_t reg, FOCMotor* motor) {
    switch (reg) {
        case REG_CURRENT_SP:
            float current_sp;
            comms >> current_sp;
            motor->target = current_sp;
            return true;
        case REG_VOLTAGE_QFF:
            return true;
        case REG_VOLTAGE_DFF:
            return true;
        default:
            return this->SimpleFOCRegisters::commsToRegister(comms, reg, motor);
    }
};

