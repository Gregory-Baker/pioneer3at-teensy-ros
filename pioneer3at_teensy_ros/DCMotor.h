#pragma once

#include <PID_v2.h>
#include "OpticalEncoder.h"

class DCMotor {

private:

  const int DIR_pin, PWM_pin;
  const boolean wheelLR;
  int k;
  OpticalEncoder* encoder;
  const double Kp, Ki, Kd;
  const int control_rate;
  const uint8_t resolutionBits;
  ros::Subscriber<std_msgs::Float32, DCMotor> sub;
  
  int motor_direction;
  double motor_pwm;
  double motor_velocity;
  double motor_speed;
  double motor_target_speed;
  PID* motorPID;

  void initPins();
  void StopMotor();
  void SetMotorDir(float);
  void UpdateTargetSpeed(float);
  void GetMotorSpeed();
  void SetMotorSpeed();
  void PublishVelocity();
  void motorCb(const std_msgs::Float32&);

public:

  DCMotor(const int, const int, const boolean, OpticalEncoder*, double, double, double, const int);
  double Update(double);
  void SetPIDAuto(boolean);
  
};

DCMotor::DCMotor(const int DIR_pin, const int PWM_pin, const boolean wheelLR, OpticalEncoder* encoder, double Kp, double Ki, double Kd, const int control_rate)
  : DIR_pin(DIR_pin)
  , PWM_pin(PWM_pin)
  , wheelLR(wheelLR)
  , encoder(encoder)
  , Kp(1)
  , Ki(0)
  , Kd(0)
  , control_rate(control_rate)
  , resolutionBits(8)
  , sub(wheelLR == LEFT ? "pioneer/left_wheel_cmd_vel" : "pioneer/right_wheel_cmd_vel", &DCMotor::motorCb, this)
{
  DCMotor::initPins();
  nh.subscribe(sub);
  motorPID = new PID(&motor_speed, &motor_pwm, &motor_target_speed, Kp, Ki, Kd, P_ON_M, DIRECT);
  motorPID->SetSampleTime(1000/control_rate);
  SetPIDAuto(true);
  wheelLR == LEFT? k = -1 : k = 1;
}

void DCMotor::initPins() {
  pinMode(DIR_pin, OUTPUT);
  pinMode(PWM_pin, OUTPUT);

  StopMotor();
}

void DCMotor::StopMotor() {
  motor_target_speed = 0;
}

void DCMotor::UpdateTargetSpeed(float val) {
  motor_target_speed = abs(val);
}

void DCMotor::SetMotorDir(float val) {
  if (val > 0) {
    motor_direction = 1;
    digitalWrite(DIR_pin, wheelLR);
  }
  else {
    motor_direction = -1;
    digitalWrite(DIR_pin, !wheelLR);
  }
}

void DCMotor::GetMotorSpeed(){
  motor_velocity = encoder->GetVelocity();
  motor_speed = abs(motor_velocity);
}

void DCMotor::PublishVelocity(){
  encoder->PublishVelocity();
}

void DCMotor::SetMotorSpeed(){
  analogWrite(PWM_pin, motor_pwm);
}

void DCMotor::SetPIDAuto(boolean pidOn) {
  if (pidOn) {
    motorPID->SetMode(AUTOMATIC);
  }
  else {
    motorPID->SetMode(MANUAL);
  }
}

void DCMotor::motorCb(const std_msgs::Float32& msg) {
  DCMotor::Update((double)msg.data);
  PublishVelocity();
}

double DCMotor::Update(double target_vel) {
  UpdateTargetSpeed(target_vel);
  SetMotorDir(target_vel);
  GetMotorSpeed();
  motorPID->Compute();
  SetMotorSpeed();  
  return motor_velocity;
}

//void DCMotor::Forward(float val) {
//  digitalWrite(DIR_PIN, wheelLR);
//  motor_speed = encoder->GetVelocity();
//  analogWrite(PWM_PIN, DCMotor::mapToInt(val));
//}
//
//void DCMotor::Backward(float val) {
//  digitalWrite(DIR_PIN, !wheelLR);
//  motor_speed = encoder->GetVelocity();
//  analogWrite(PWM_PIN, DCMotor::mapToInt(val));
//}



/**
 * Maps the positive velocity command from
 * [0, 1.0] to [0, 255].
 *
 * @param val The velocity command
 * @return The mapped velocity command
 */
//int32_t DCMotor::mapToInt(float val) {
//  int32_t maxValue = pow(2, this->resolutionBits) - 1; // (2 ^ N)-1
//  return map(static_cast<int32_t>(val), 0, 1, 0, maxValue);
//}
