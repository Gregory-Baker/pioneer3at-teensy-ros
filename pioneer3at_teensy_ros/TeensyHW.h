#ifndef TEENSYHW_H
#define TEENSYHW_H

#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

const int LED = 13;           // LED pin

const int LPWM = 20;          // Left motor PWM
const int RPWM = 21;          // Right motor PWM

const int LDIR = 8;           // Left motor direction
const int RDIR = 7;           // Right motor direction

const int LEA = 6;            // Left encoder A
const int REA = 5;            // Right encoder A
const int REB = 4;            // Right encoder B
const int LEB = 3;            // Left encoder B

const int MEN = 21;           // Motor enable
const int ESTOP = 20;         // E-Stop detect input
const int VBAT = A9;          // Battery Voltage

/*  
 *   Note: VBAT pin is connected to Teensy3.2 ADC pin, but also has a 10k resistor to ground 
 *   Internal resistance of MCB is ~6.5k, achieving a voltage divider circuit that converts VBAT 0-5V to 0-3V3 range that the Teensy can read.
 *   The VBAT -> battery voltage conversion for our system has the following gradient and slope.
*/
const float vbat_m = 0.0165;  // Gradient of VBAT read -> battery voltage trendline
const float vbat_c = -0.386;  // y-intercept of VBAT read -> battery voltage trendline

class TeensyHW {
  
  private:
    void initPins();
    boolean ledState;
    std_msgs::Float32 msg;
    ros::Publisher pub;
    double battery_voltage;
    
  public:
    TeensyHW();
    void LEDOn();
    void LEDOff();
    void LEDToggle();
    float ReadBattery();
    void EnableMotors();
    void DisableMotors();
    void PublishBatteryVoltage();
  
};

TeensyHW::TeensyHW()
  : pub("pioneer/battery_voltage", &msg)
{
  initPins();
  EnableMotors();
  LEDOn();
  nh.advertise(pub);
}

void TeensyHW::initPins() {
  pinMode(LED, OUTPUT);                   // LED
  pinMode(MEN, OUTPUT);                   // Motor enable
  pinMode(VBAT, INPUT);                   // Battery voltage, analog input (10 bit resolution)
  pinMode(ESTOP, INPUT);                  // Detect ESTOP input
}

void TeensyHW::LEDOn() {
  ledState = HIGH;
  digitalWrite(LED, HIGH);
}

void TeensyHW::LEDOff() {
  ledState = LOW;
  digitalWrite(LED, HIGH);
}

void TeensyHW::LEDToggle() {
  ledState = !ledState;
  digitalWrite(LED, ledState);
}

float TeensyHW::ReadBattery() {
  return vbat_m*(float)analogRead(VBAT) + vbat_c;
}

void TeensyHW::EnableMotors() {
  digitalWrite(MEN, LOW);
}

void TeensyHW::DisableMotors() {
  digitalWrite(MEN, HIGH);
}

void TeensyHW::PublishBatteryVoltage(){
  battery_voltage = ReadBattery();
  msg.data = battery_voltage;
  pub.publish(&msg);
}

#endif
