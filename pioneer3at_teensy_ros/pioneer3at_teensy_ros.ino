/*
    Script for the interfacing Teensy3.2 with a Pioneer3AT Motor Control Board

*/
#include "pioneer_pinout.h"
#include "DCMotor.h"
#include "OpticalEncoder.h"

DCMotor* leftMotor;
DCMotor* rightMotor; 

IntervalTimer sampleTimer;

float test_vel = 1.0;               // Target motor velocity (Rad/Second)
uint8_t encoder_read_freq = 10;     // Sample time of encoders
double Kp = 0, Ki = 100, Kd = 0;    // PID gains
boolean serial_comms = 1;           // Flag to Print outputs to serial. Note: Serial Monitor and ROS don't work well together.

boolean led_status = 1;
float battery_voltage = 0;
long ticksPerRev = 99650;

double left_vel = 0;
double right_vel = 0;
float left_target_vel = test_vel;
float right_target_vel = test_vel;

unsigned long read_time;
boolean sample_flag = false;
int sample_interval = 1000000/encoder_read_freq;   // Time between control updates (microseconds)

void setup() {

  
  // Initialise pins
  pinMode(LED, OUTPUT);                   // LED
  pinMode(MEN, OUTPUT);                   // Motor enable
  pinMode(VBAT, INPUT);                   // Battery voltage, analog input (10 bit resolution)
  digitalWrite(MEN, LOW);                 // LOW = motors enabled, HIGH = motors disabled

  if (serial_comms) {
    Serial.begin(9600);
    Serial.println("Pioneer PID Test:");
    while(!Serial);                       // TODO Currently only starts when serial plotter/monitor is opened
    Serial.println("left_vel left_target right_vel right_target battery_voltage");
  }
  else {   
    nh.initNode();
  }
  leftMotor = new DCMotor(LDIR, LPWM, LEFT
                           , new OpticalEncoder(LEA, LEB, LEFT, ticksPerRev)
                           , Kp, Ki, Kd
                           , encoder_read_freq);
                           
  rightMotor = new DCMotor(RDIR, RPWM, RIGHT
                           , new OpticalEncoder(REA, REB, RIGHT, ticksPerRev)
                           , Kp, Ki, Kd
                           , encoder_read_freq);
  
  
  delay(2000);
  digitalWrite(LED, led_status);

  read_time = millis();
  sampleTimer.begin(sample_flag_on, sample_interval);
}

void sample_flag_on() {
  sample_flag = true;
}

void print_values () {
  Serial.print(left_vel);
  Serial.print(" ");
  Serial.print(left_target_vel);
  Serial.print(" ");

  Serial.print(right_vel);
  Serial.print(" ");
  Serial.print(right_target_vel);
  Serial.print(" ");

  Serial.println(battery_voltage);
}

// Gradient (m) and intercept (c) values yet to be established 
float calculate_battery_voltage(int vbat_input) {
  float vbat;
  float m = 0.0165;
  float c = -0.386;
  return vbat = m*vbat_input + c;
}

void loop() {

  nh.spinOnce(); 
  
  if (sample_flag) {
    
    battery_voltage = calculate_battery_voltage(analogRead(VBAT));

    boolean pidOn;
    pidOn = (battery_voltage > 10)? true : false;

    leftMotor->SetPIDAuto(pidOn);
    rightMotor->SetPIDAuto(pidOn);

    if (serial_comms){
      left_vel = leftMotor->Update(left_target_vel);
      right_vel = rightMotor->Update(right_target_vel);
      
      print_values();
    }
    sample_flag = false;
    
  }
  
}
