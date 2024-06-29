#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <Wire.h>
#include <settings/stm32/STM32FlashSettingsStorage.h>
#include <encoders/ma730/MagneticSensorMA730SSI.h>
#include <encoders/ma730/MagneticSensorMA730.h>
#include <encoders/hysteresis/HysteresisSensor.h>
#include <comms/telemetry/SimpleTelemetry.h>
#include <comms/streams/BinaryIO.h>
#include <comms/streams/BinaryIO.h>
#include <comms/telemetry/TeleplotTelemetry.h>
#include <comms/streams/PacketCommander.h>
#include <utilities/stm32math/STM32G4CORDICTrigFunctions.h>

#include "SST/sst.hpp"
#include "tasks/tasks.hpp"
#include "SST/dbc_assert.h"
#include "motor/FFBLDCMotor.h"
#include "regs/CustomRegisters.h"

uint32_t tick_count_ovf = 0;
namespace SST {
  void onStart() {
    tick_count_ovf = 14;
    SystemCoreClockUpdate();
    SysTick_Config((SystemCoreClock/(1000U * tick_count_ovf)));
    NVIC_SetPriority(SysTick_IRQn, 0U);
    // NVIC_EnableIRQ(SysTick_IRQn);
  }

  // void onIdle();
}
#ifdef __cplusplus
extern "C" {
#endif

void HAL_IncTick(void)
{
  static uint32_t counter = 1;
  if (counter >= tick_count_ovf) {
    uwTick += uwTickFreq;
    counter=1;
  }
  counter+=1;
}

void osSystickHandler(){
  SST::TimeEvt::tick();
}
void COMP4_IRQHandler();
void COMP1_2_3_IRQHandler();

#ifdef __cplusplus
}
#endif

#define SERIAL_SPEED 115200

#define MUX_SELECT PA6
#define BUTTON PB5
#define BATT_VOLTAGE_SENSE PA4
#define TEMP_SENSE PA5
#define LED PC6

// Define CAN bus pins
#define CAN_TX_PIN PB9
#define CAN_RX_PIN PB8

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(5, 2.0f, 300.0f);
FFBLDCMotor motor = FFBLDCMotor(5, 2.0f, 240.0f, 19e-6f);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA0, PA1, PA2);

//Position Sensor
SPISettings sensorSPISettings(24000000, MA730_BITORDER, SPI_MODE3);
MagneticSensorMA730 sensor = MagneticSensorMA730(PB12, sensorSPISettings);
SPISettings sensorSPISettings(24000000, MA730_BITORDER, SPI_MODE3);
MagneticSensorMA730 sensor = MagneticSensorMA730(PB12, sensorSPISettings);
HysteresisSensor hysteresisSensor = HysteresisSensor(sensor, 0.001f);

LowsideCurrentSense current_sense = LowsideCurrentSense(0.003, 46, PB0, PB1, PA3);

// settings
STM32FlashSettingsStorage settings = STM32FlashSettingsStorage(); // use 1 page at top of flash

// custom register defs
CustomRegisters regs = CustomRegisters();

// instantiate the commander
PacketCommander pcommand = PacketCommander(Serial);
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); };
void doSave(char* cmd) { settings.saveSettings(); };
void doLoad(char* cmd) { settings.loadSettings(); };
void doReinit(char* cmd) { motor.sensor_direction = Direction::UNKNOWN; motor.zero_electric_angle = NOT_SET; motor.initFOC(); };
void doHysteresis(char* cmd) { hysteresisSensor._amount = atof(cmd); Serial.print("Hysteresis: "); Serial.println(hysteresisSensor._amount, 4); };
void doSetSensor(char* cmd) {
  if (cmd[0]=='1') {
    motor.linkSensor(&hysteresisSensor);
    Serial.println("Using hysteresis");
  }
  else {
    motor.linkSensor(&sensor);
    Serial.println("Using MA730");
  }
};

// telemetry
// TextIO IO = TextIO(Serial);
BinaryIO IO = BinaryIO(Serial);
// SimpleTelemetry telemetry = SimpleTelemetry();
// TeleplotTelemetry telemetry = TeleplotTelemetry();
Telemetry telemetry = Telemetry();
void doDownsample(char* cmd) { telemetry.downsample = atoi(cmd); };

Tasks::Commutate commutate_task = Tasks::Commutate(motor, (SST::TCtr) 1U, (SST::TCtr) 1U);
static SST::Evt const *commutateQSto[10];

Tasks::Move move_task = Tasks::Move(motor, (SST::TCtr) 7U, (SST::TCtr) 7U);
static SST::Evt const *moveQSto[10];

#ifdef __cplusplus
extern "C" {
#endif
void COMP4_IRQHandler() {
  commutate_task.activate();
}
void COMP1_2_3_IRQHandler() {
  move_task.activate();
}
#ifdef __cplusplus
}
#endif

void setup() {  
  SimpleFOCRegisters::regs = &regs;
  Serial.begin(SERIAL_SPEED);
  // while (!Serial);
  SimpleFOCDebug::enable(&Serial);

  Serial.print("spin servo - firmware version ");
  Serial.println(SPIN_SERVO_FIRMWARE_VERSION);

  SPI.setMISO(PB14);
  SPI.setMOSI(PB15);
  SPI.setSCLK(PB13);


  SimpleFOC_CORDIC_Config();

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 15;
  driver.voltage_limit = driver.voltage_power_supply*0.95f;
  driver.pwm_frequency	= 20000;

  driver.init();
  sensor.init();
  hysteresisSensor.init();


  FieldStrength fs = sensor.getFieldStrength();
  Serial.print("Field strength: 0x");
  Serial.println(fs, HEX);

  // link driver
  motor.linkDriver(&driver);
  motor.linkSensor(&hysteresisSensor);

  // current sense
  current_sense.linkDriver(&driver);
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  // aligning voltage
  motor.voltage_sensor_align = 0.4;
  motor.current_limit = 3;
  motor.voltage_limit = driver.voltage_limit / 2.0f;
  motor.velocity_limit = 1000.0f; // 1000rad/s = aprox 9550rpm

  float current_bandwidth = 300;
  // motor.PID_current_d.P = motor.phase_inductance*current_bandwidth*_2PI;
  // motor.PID_current_d.I = motor.PID_current_d.P*motor.phase_resistance/motor.phase_inductance;
  // motor.PID_current_q.D = 0;
  motor.PID_current_d.output_ramp = 0;
  motor.LPF_current_d.Tf = 1/(_2PI*current_bandwidth*1.5);

  // motor.PID_current_q.P = motor.phase_inductance*current_bandwidth*_2PI;
  // motor.PID_current_q.I = motor.PID_current_q.P*motor.phase_resistance/motor.phase_inductance;
  // motor.PID_current_q.D = 0;
  motor.PID_current_q.output_ramp = 0;
  motor.LPF_current_q.Tf = 1/(_2PI*current_bandwidth*1.5);

  // some defaults
  motor.PID_velocity.P = 0.05f;
  motor.PID_velocity.I = 0.05f;
  motor.PID_velocity.D = 0.000f;
  motor.PID_velocity.output_ramp = 0;
  motor.LPF_velocity.Tf = 1.0f/(_2PI*100.0f);
  motor.P_angle.P = 1.0f;
  motor.P_angle.I = 0.0f;
  motor.P_angle.D = 0.0f;
  motor.P_angle.output_ramp = 0;
  motor.LPF_angle.Tf = 1.0f/(_2PI*100.0f);

  motor.torque_controller = TorqueControlType::foc_current;
  // motor.controller = MotionControlType::angle;
  motor.controller = static_cast<MotionControlType>(motor.CustomMCType::angle_nocascade);
  motor.motion_downsample = 0;

  // load settings
  settings.addMotor(&motor);
  SimpleFOCRegister registers[] = { REG_SENSOR_DIRECTION, REG_ZERO_ELECTRIC_ANGLE, REG_VEL_LPF_T, REG_VEL_PID_P, REG_VEL_PID_I, REG_VEL_PID_D, REG_VEL_PID_LIM, REG_VEL_PID_RAMP, REG_ANG_LPF_T, REG_ANG_PID_P, REG_ANG_PID_I, REG_ANG_PID_D, REG_ANG_PID_LIM, REG_ANG_PID_RAMP, REG_CURQ_LPF_T, REG_CURQ_PID_P, REG_CURQ_PID_I, REG_CURQ_PID_D, REG_CURQ_PID_LIM, REG_CURQ_PID_RAMP, REG_CURD_LPF_T, REG_CURD_PID_P, REG_CURD_PID_I, REG_CURD_PID_D, REG_CURD_PID_LIM, REG_CURD_PID_RAMP, REG_VOLTAGE_LIMIT, REG_CURRENT_LIMIT, REG_VELOCITY_LIMIT, REG_MOTION_DOWNSAMPLE, REG_TORQUE_MODE, REG_CONTROL_MODE };
  settings.setRegisters(registers, sizeof(registers)/sizeof(SimpleFOCRegister));
  settings.settings_version = 2;
  settings.init();
  //settings.loadSettings();

  // initialize motor
  motor.init();

  // align sensor and start FOC
  Serial.print("Aligning...");
  motor.initFOC();

  // add commands
  command.echo = true;
  command.add('M', doMotor, "motor commands");
  command.add('s', doSave, "save settings");
  command.add('l', doLoad, "load settings");
  command.add('d', doDownsample, "downsample telemetry");
  command.add('r', doReinit, "reinit motor");
  command.add('h', doHysteresis, "hysteresis amount");
  command.add('S', doSetSensor, "set sensor (0=MA730, 1=hysteresis)");
  pcommand.addMotor(&motor);
  pcommand.init(IO);
  // add telemetry
  telemetry.addMotor(&motor);
  telemetry.downsample = 0; // off by default, use register 28 to set
  uint8_t telemetry_registers[] = { REG_TARGET, REG_ANGLE, REG_VELOCITY, REG_CURRENT_Q , REG_CURRENT_SP, REG_VOLTAGE_DFF, REG_VOLTAGE_QFF};
  telemetry.setTelemetryRegisters(sizeof(telemetry_registers)/sizeof(SimpleFOCRegister), telemetry_registers);
  telemetry.init(IO);
  telemetry.init(IO);

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal:"));

  SST::init();
  commutate_task.setIRQ(COMP4_IRQn);
  commutate_task.start(
    2U,
    commutateQSto,
    ARRAY_NELEM(commutateQSto),
    nullptr
  );

  move_task.setIRQ(COMP1_2_3_IRQn);
  move_task.start(
    1U,
    moveQSto,
    ARRAY_NELEM(moveQSto),
    nullptr
  );
  
  SST::start(); // Start SST Scheduler
  SST::onStart(); // configure and start the interrupts
  // Do NOT WFI forever, because arduino expects loop to run serial/usb tasks after loop exits
}

void loop() {
  // uint32_t t_now = micros();
  // if (t_now-time_prev > int(1e6*15)) {
  //   time_prev=t_now;
  //   Serial.printf("\n\n%d\n\n",i);
  //   i=0;
  // }

  // Motion control function
  // motor.move();
  // main FOC algorithm function  
  // motor.loopFOC();
  // motor.move();
  // main FOC algorithm function  
  // motor.loopFOC();
  // user communication
  // command.run();
  pcommand.run();
  // telemetry
  telemetry.run();

}