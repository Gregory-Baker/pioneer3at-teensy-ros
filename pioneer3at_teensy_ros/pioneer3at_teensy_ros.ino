/*
    Script for the interfacing Teensy3.2 with a Pioneer3AT Motor Control Board

*/
#include "TeensyHW.h"
#include "DCMotor.h"
#include "OpticalEncoder.h"

DCMotor* leftMotor;
DCMotor* rightMotor;

TeensyHW teensy = TeensyHW();

// Values used to test motor directly for development purposes
boolean motor_test = false;
float target_vel = 1.0;

int control_frequency;
float pid_gains[3];
long ticksPerRev = 99650;

IntervalTimer sampleTimer;
boolean sample_flag = false;

void sample_flag_on() {
  sample_flag = true;
}

void setup() {

  while (!nh.connected()) {
    nh.spinOnce();
  }
  nh.initNode();
  
  int control_frequency;
  if (! nh.getParam("/pioneer/control_frequency", &control_frequency)) { 
    //default value
    control_frequency = 10;
  }
  
  if (! nh.getParam("/pioneer/pid_gains", pid_gains, 3)) { 
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
  
  int sample_interval = 1000000/(control_frequency);   // Time between control updates (microseconds)
  sampleTimer.begin(sample_flag_on, sample_interval);

}

void loop() {

  nh.spinOnce();

  if (sample_flag){

    // The following code disables PID if Pioneer is turned off, to prevent PID output balooning.
    boolean pidOn = (teensy.ReadBattery() > 10)? true : false;
  
    if (pidOn && !leftMotor->pidOn) {
      leftMotor->PIDOn();
      rightMotor->PIDOn();
    }
    else if (!pidOn && leftMotor->pidOn) {
      leftMotor->PIDOff();
      rightMotor->PIDOff();
    }

    leftMotor->Update();
    rightMotor->Update();

    sample_flag = false;
  }
}
