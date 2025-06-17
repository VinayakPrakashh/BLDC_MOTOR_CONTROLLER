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
// maximal expected velocity
sensor.velocity_max = 1000; // 1000rad/s by default ~10,000 rpm

// interrupt routine initialization
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

// enable hall sensor hardware interrupts
sensor.enableInterrupts(doA, doB, doC)

void setup(){

  // initialize sensor hardware
  sensor.init();
  // hardware interrupt enable
  sensor.enableInterrupts(doA, doB, doC);
  // link the motor and the sensor
  motor.linkSensor(&sensor);
}