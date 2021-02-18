#pragma once

#include <PID_v2.h>
#include "OpticalEncoder.h"

class DCMotor {
  
  private:
  
    const int DIR_pin, PWM_pin;
    const boolean wheelLR;
    OpticalEncoder* encoder;
    const double Kp, Ki, Kd;
    const int control_rate;
    const int pwmFrequency;
    ros::Subscriber<std_msgs::Float32, DCMotor> sub;
    
    int motor_direction;
    double motor_pwm;
    double motor_velocity;
    double motor_speed;
    double motor_target_velocity;
    double motor_target_speed;
    PID* motorPID;
  
    void initPins();
    void StopMotor();
    void SetMotorDir(float);
    void GetMotorSpeed();
    void SetMotorSpeed();
    void PublishVelocity();
    void motorCb(const std_msgs::Float32&);
  
  public:
  
    DCMotor(const int, const int, const boolean, OpticalEncoder*, double, double, double, const int);
    double Update();
    void PIDOn();
    void PIDOff();
    boolean pidOn;
    void SetTargetVelocity(float);
  
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
  , pwmFrequency(32000)
  , sub(wheelLR == LEFT ? "pioneer/left_wheel_cmd_vel" : "pioneer/right_wheel_cmd_vel", &DCMotor::motorCb, this)
{
  DCMotor::initPins();
  StopMotor();
  nh.subscribe(sub);
  motorPID = new PID(&motor_speed, &motor_pwm, &motor_target_speed, Kp, Ki, Kd, P_ON_M, DIRECT);
  motorPID->SetSampleTime(1000/control_rate);
  PIDOn();
}

void DCMotor::initPins() {
  pinMode(DIR_pin, OUTPUT);
  pinMode(PWM_pin, OUTPUT);
  analogWriteFrequency (PWM_pin, pwmFrequency);
}

void DCMotor::StopMotor() {
  SetTargetVelocity(0.0);
}

void DCMotor::SetTargetVelocity(float val) {
  motor_target_velocity = val;
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

void DCMotor::PIDOn() {
    motorPID->SetMode(AUTOMATIC);
    pidOn = true;
}

void DCMotor::PIDOff() {
    motorPID->SetMode(MANUAL);
    pidOn = false;
}

void DCMotor::motorCb(const std_msgs::Float32& msg) {
  SetTargetVelocity((double)msg.data);
  //DCMotor::Update();
}

double DCMotor::Update() {
  SetMotorDir(motor_target_velocity);
  GetMotorSpeed();
  motorPID->Compute();
  SetMotorSpeed();  
  PublishVelocity();
  return motor_velocity;
}
