#include <board_vesc_4_12.h>
#include <SimpleFOC.h>
#include <arduino.h>
#define MAX_TARGET 21
#define  BRAKE  PA14
HallSensor sensor = HallSensor(HALL_1, HALL_2, HALL_3, 24);
BLDCMotor motor(24);
BLDCDriver6PWM driver(H1, L1, H2, L2, H3, L3, EN_GATE);//For DRV8302

void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

volatile long servo_duty_period_micros = 0; // typically 800us to 2200us
int blink_counter = 0;
int  blink_delay  = 0;
int current_time;
bool blinkAlternate;
int target_counter = 0;
int target_current;
int target;
int target_delay = 150000;
bool ignition = 0;
int ignitionState;

  //brake delay settings
int brake_counter = 0;
int brake_delay = 50000; //60ms

void setup()
{

  Serial.begin(115200);
  delay(100);
  pinMode(LED_GREEN,OUTPUT);
  pinMode(LED_RED, OUTPUT);

  driver.pwm_frequency = 30000;


  
  sensor.enableInterrupts(doA, doB, doC);
  sensor.init();
  digitalWrite(LED_RED, HIGH);
  delay(100);
  digitalWrite(LED_RED, LOW);
  delay(100);
  digitalWrite(LED_RED, HIGH);
  delay(100);
  digitalWrite(LED_RED, LOW);
  delay(100);
  motor.linkSensor(&sensor);


  driver.voltage_power_supply = 48;
  driver.init();

  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 1;
  motor.voltage_limit = 30;  // [V]
  motor.velocity_limit = 20; // [rad/s]

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 1;
  motor.PID_velocity.output_ramp = 200;
  motor.P_angle.P = 2;

  motor.controller = MotionControlType::velocity; // ControlType::angle;

  motor.init();
  motor.initFOC();
}

void loop()
{ 
  current_time = micros();
 // ignitionState = digitalRead(PB10); // Read the state of the ignition button
  
 // if (ignitionState == HIGH) {
    // Ignition is turned ON
    //Serial.println("Ignition is turned ON");
      //Brake engaged
 if(digitalRead(BRAKE) == HIGH){
      if (current_time - brake_counter > brake_delay) {
      brake_counter = micros();
      target -= 1;
    }
  }
   if(target < 0 ){
   target = 0;
   }

   // current_time = micros();
  target_current = map(analogRead(PA5),257,790, 0, MAX_TARGET * 100) / 100.0; // division by 100 to get some float/decimal point precision
  if(target_current <= target){ // throttle released or not changed 
    target = target_current;
  }
  else if (target_current > target){   // throttle engage
    if (current_time - target_counter > target_delay) {
      target_counter = micros();
      target += 1;
    }
  }

  motor.loopFOC();
  motor.move(-target); // -ve for reverse
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(LED_GREEN, blinkAlternate); //Pin 13 is built in LED
    
    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 500000;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 500000;
      }
  }
  /*
  Serial.print("Target_original: ");
  Serial.print(map(analogRead(PA5), 0, 1023, 0, -MAX_TARGET * 100) / 100.0);
  Serial.print("      Target: ");
  Serial.println(-target);
  */

  Serial.print("Target: ");
  Serial.print(-target);
  Serial.print("     brakeread: ");
  Serial.print(digitalRead(PA14));
  Serial.print("     Current A  : ");
  Serial.print(map(analogRead(BR_SO2), 0,1023,0,100));
  Serial.print("     Current B  : ");
  Serial.print(map(analogRead(BR_SO1), 0,1023,0,100));
  Serial.print("     Temperature  : ");
  Serial.println(map(analogRead(ADC_TEMP),512,1024,0,100));
    // Proceed with the actuation part of the code
    // Add your actuation code here
    
//  } 
  
//  else {
    // Ignition is turned OFF
  //  Serial.println("Ignition is turned OFF");
    // Stay in a loop until ignition is turned ON
  //  while (digitalRead(PA14) != HIGH) {
   //   Serial.println("Ignition is turned OFF");
      //delay(1000); // Delay to avoid flooding the serial monitor
  //  }
 // } 
}