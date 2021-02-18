#ifndef PIONEER_PINOUT_H
#define PIONEER_PINOUT_H

#include <ros.h>

ros::NodeHandle nh;

const int LED = 13;       // LED pin

const int LPWM = 20;      // Left motor PWM
const int RPWM = 21;      // Right motor PWM

const int LDIR = 8;       // Left motor direction
const int RDIR = 7;       // Right motor direction

const int LEA = 6;        // Left encoder A
const int REA = 5;        // Right encoder A
const int REB = 4;        // Right encoder B
const int LEB = 3;        // Left encoder B

const int MEN = 21;       // Motor enable
const int ESTOP = 20;     // E-Stop detect input
const int VBAT = A9;      // Battery Voltage

#endif
