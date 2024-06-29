
#pragma once

#include "comms/SimpleFOCRegisters.h"
#include "common/base_classes/FOCMotor.h"
#include "../motor/FFBLDCMotor.h"

#define REG_CURRENT_SP 0x90
#define REG_VOLTAGE_QFF 0x91
#define REG_VOLTAGE_DFF 0x92


class CustomRegisters : public SimpleFOCRegisters {
public:
    CustomRegisters();
    ~CustomRegisters();

    virtual uint8_t sizeOfRegister(uint8_t reg) override;
    virtual bool registerToComms(RegisterIO& comms, uint8_t reg, FOCMotor* motor) override;
    virtual bool commsToRegister(RegisterIO& comms, uint8_t reg, FOCMotor* motor) override;

    FFBLDCMotor* motor;
};

