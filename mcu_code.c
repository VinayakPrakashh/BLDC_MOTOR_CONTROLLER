#include <SimpleFOC.h>
#include "arduino.h"

// These pins are used for the DRV8302 driver
#define IN1_H PA8
#define IN1_L PB13
#define IN2_H PA9
#define IN2_L PB14
#define IN3_H PA10
#define IN3_L PB15
// EN_GATE is the enable pin for the DRV8302 driver
#define EN_GATE PB12
// These pins are used for the Hall sensors
#define HALL_1 PB9
#define HALL_2 PB8
#define HALL_3 PB7
//current sensor pin
#define SO1 PA0
#define SO2 PA1

//fault pins
#define nOCTW PA3
#define nFAULT PA4


HallSensor sensor = HallSensor(HALL_1, HALL_2, HALL_3, 11); // 11 is the number of pole pairs

//  BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
//  - phA_h, phA_l - A phase pwm pin high/low pair 
//  - phB_h, phB_l - B phase pwm pin high/low pair
//  - phB_h, phC_l - C phase pwm pin high/low pair
//  - enable pin    - (optional input)
BLDCDriver6PWM driver = BLDCDriver6PWM(IN1_H,IN1_L, IN2_H,IN2_L,IN3_H,IN3_L, EN_GATE);

// LowsideCurrentSense constructor
LowsideCurrentSense current_sense  = LowsideCurrentSense(0.01, 10, SO1, _NC, SO2); //10mohm shunt, gain 10, SO1 and SO2 are the current sense pins

// interrupt routine initialization
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

// enable hall sensor hardware interrupts
sensor.enableInterrupts(doA, doB, doC)

void setup(){

  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000; //20kHz

  // initialize sensor hardware
  sensor.init();
  Serial.println("Sensor initialized");

  // set the maximum expected velocity (in rad/s)
  sensor.velocity_max = 1000;  // default is 1000 rad/s

  // hardware interrupt enable
  sensor.enableInterrupts(doA, doB, doC);
  
  // link the motor and the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 48; // 48V power supply
  driver.init();
  Serial.print("Driver init ");
// init driver
  if (driver.init())  Serial.println("success!");
  else{
  Serial.println("failed!");
  return; }
  // link the driver to the current sense
  current_sense.linkDriver(&driver);
  // link the motor and the driver
  motor.linkDriver(&driver);
    // motor init
  motor.init();

  // init current sense
  current_sense.init();
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop(){

//Use if need to get angle and velocity 

//   // update the sensor values 
//   sensor.update();
//   // display the angle and the angular velocity to the terminal
//   Serial.print("Angle: ");
//   Serial.print(sensor.getAngle());
//   Serial.print("\t");
//   Serial.print("Velocity: ");
//   Serial.println(sensor.getVelocity());

//for measuring current

    // PhaseCurrent_s currents = current_sense.getPhaseCurrents();

    // Serial.print(currents.a*1000); // milli Amps
    // Serial.print("\t");
    // Serial.print(currents.b*1000); // milli Amps
    // Serial.print("\t");
    // Serial.println(currents.c*1000); // milli Amps

}