#pragma once

#include "SimpleFOC.h"
#include "SST/sst.hpp"

#ifdef __cplusplus
extern "C" {
#endif
void DBC_fault_handler(char const * const module, int const label); 
#ifdef __cplusplus
}
#endif


namespace Tasks {

enum Signals {
    TIMEOUT1_SIG,
    TIMEOUT2_SIG,
    // ...
    MAX_SIG
};

class Commutate : public SST::Task {
public:
    Commutate( FOCMotor &_motor, SST::TCtr _ctr, SST::TCtr _interval);
    void init(SST::Evt const * const ie) override;
    void dispatch(SST::Evt const * const ie) override;
private:
    SST::TimeEvt te1;
    FOCMotor &motor;
    SST::TCtr const ctr;
    SST::TCtr const interval;
};

class Move : public SST::Task {
public:
    Move( FOCMotor &_motor, SST::TCtr _ctr, SST::TCtr _interval);
    void init(SST::Evt const * const ie) override;
    void dispatch(SST::Evt const * const ie) override;
private:
    SST::TimeEvt te1;
    FOCMotor &motor;
    SST::TCtr const ctr;
    SST::TCtr const interval;
};

} // namespace Tasks
