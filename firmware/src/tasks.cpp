#include "Arduino.h"
#include "tasks/tasks.hpp"

namespace Tasks {

Commutate::Commutate( FOCMotor &_motor, SST::TCtr _ctr, SST::TCtr _interval ) 
    : te1(TIMEOUT1_SIG, this),
    motor(_motor),
    ctr(_ctr),
    interval(_interval)
    {}

void Commutate::init(SST::Evt const * const ie) {
    (void)ie;
    te1.arm(ctr, interval);
}

void Commutate::dispatch(SST::Evt const * const ie) {
    (void)ie;
    digitalToggle(PC4);
    motor.loopFOC();
    digitalToggle(PC4);
}

Move::Move( FOCMotor &_motor, SST::TCtr _ctr, SST::TCtr _interval ) 
    : te1(TIMEOUT1_SIG, this),
    motor(_motor),
    ctr(_ctr),
    interval(_interval)
    {}

void Move::init(SST::Evt const * const ie) {
    (void)ie;
    te1.arm(ctr, interval);
}

void Move::dispatch(SST::Evt const * const ie) {
    (void)ie;
    digitalToggle(PA8);
    motor.move();
    digitalToggle(PA8);

}

} // namespace Tasks

void DBC_fault_handler(char const * const module, int const label) {
    //
    // NOTE: add here your application-specific error handling
    // //
    // (void)module;
    // (void)label;

    Serial.print("Error in module: ");
    Serial.println(module);
    Serial.printf("At: %d\n", label);
    Serial.flush();
    // set PRIMASK to disable interrupts and stop SST right here
    __asm volatile ("cpsid i");
    while(true) {__WFI();}
    NVIC_SystemReset();
}