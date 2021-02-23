/*
    Script for the interfacing Teensy3.2 with a Pioneer3AT Motor Control Board

*/
#include "TeensyHW.h"
#include "DCMotor.h"
#include "OpticalEncoder.h"

TeensyHW* teensy;

DCMotor* leftMotor;
DCMotor* rightMotor;

// Values used to test motor directly for development purposes
boolean motor_test = false;
float target_vel = 1.0;

int control_frequency;
float pid_gains[3];
long ticksPerRev = 99650;

IntervalTimer controlTimer;
boolean control_update_flag = false;

IntervalTimer slowTimer;
boolean slow_update_flag = false;
int slow_update_frequency = 1; //1Hz

unsigned long blinkTimer = millis();

void control_update_flag_on() {
  control_update_flag = true;
}

void slow_update_flag_on() {
  slow_update_flag = true;
}

void setup() {
  
  while (!nh.connected()) {
    nh.spinOnce();
  }
  nh.initNode();

  teensy = new TeensyHW();
  
  int control_frequency;
  if (! nh.getParam("control_frequency", &control_frequency)) { 
    //default value
    control_frequency = 10;
  }
  
  if (! nh.getParam("pid_gains", pid_gains, 3)) { 
    //default values
    pid_gains[0]= 0;
    pid_gains[1]= 100;
    pid_gains[2]= 0;
  }
  
  leftMotor = new DCMotor(LDIR, LPWM, LEFT
                           , new OpticalEncoder(LEA, LEB, LEFT, ticksPerRev)
                           , pid_gains[0], pid_gains[1], pid_gains[2]
                           , control_frequency);
                           
  rightMotor = new DCMotor(RDIR, RPWM, RIGHT
                           , new OpticalEncoder(REA, REB, RIGHT, ticksPerRev)
                           , pid_gains[0], pid_gains[1], pid_gains[2]
                           , control_frequency);
                           
  if (motor_test) {
    leftMotor->SetTargetVelocity(target_vel);
    rightMotor->SetTargetVelocity(target_vel); 
  }
  
  int control_update_interval = 1000000/(control_frequency);   // Time between control updates (microseconds)
  controlTimer.begin(control_update_flag_on, control_update_interval);

  int slow_update_interval = 1000000/(slow_update_frequency);
  slowTimer.begin(slow_update_flag_on, slow_update_interval);

}

void loop() {

  nh.spinOnce();

  if (control_update_flag){

    // The following code disables PID if Pioneer is turned off, to prevent PID output balooning.
    float battVoltage = teensy->ReadBattery();
    
    if (battVoltage > 11.0) {
      if (!leftMotor->pidOn) {
        teensy->LEDOn();
        leftMotor->PIDOn();
        rightMotor->PIDOn(); 
      }
    }
    else if (leftMotor->pidOn) {
      leftMotor->PIDOff();
      rightMotor->PIDOff();
    }

    leftMotor->Update();
    rightMotor->Update();

    control_update_flag = false;
  }

  if(slow_update_flag) {
    float battVoltage = teensy->ReadBattery();
    if (battVoltage < 11) {
      teensy->LEDToggle();
    }
    teensy->PublishBatteryVoltage();
    slow_update_flag = false;
  }
}
