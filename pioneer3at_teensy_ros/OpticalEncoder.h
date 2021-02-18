#pragma once

#include <Encoder.h>
#include <std_msgs/Float32.h>

const boolean LEFT = false;
const boolean RIGHT = true;

class OpticalEncoder {

private:

  Encoder* encoder;
  boolean wheelLR;
  const long ticksPerRev;
  std_msgs::Float32 msg;
  ros::Publisher pub;
  
  long lastRead;
  unsigned long lastReadTime;
  double velocity;

  
public:

  OpticalEncoder(const int, const int, const boolean, const long);
  long GetCount();
  float GetVelocity();
  void PublishVelocity();
  
};

OpticalEncoder::OpticalEncoder(const int A_PIN, const int B_PIN, boolean wheelLR, const long ticksPerRev)
  : encoder(new Encoder(A_PIN, B_PIN))
  , wheelLR(wheelLR)
  , ticksPerRev(ticksPerRev)
  , pub(wheelLR == LEFT? "pioneer/left_wheel_vel" : "pioneer/right_wheel_vel", &msg)
{
  lastRead = encoder->read();
  lastReadTime = millis();
  nh.advertise(pub);
}

float OpticalEncoder::GetVelocity() {
  // Change sign of sensor reading
  int k = 1;
  wheelLR == LEFT? k = -1 : k = 1;

  long ticks = encoder->read();
  long tickChange = (ticks - lastRead);
  float revsTurned = (float)tickChange / (float)ticksPerRev; // Number of revolutions turned since last read
  float time_step = millis() - lastReadTime;
  velocity = k * 2 * PI * revsTurned / (time_step/1000); // velocity of wheel (rad/s)
  lastRead = ticks;
  lastReadTime = millis();
  return velocity;
}

long OpticalEncoder::GetCount() {
  return encoder->read();
}

void OpticalEncoder::PublishVelocity(){
  msg.data = velocity;
  pub.publish(&msg);
}
