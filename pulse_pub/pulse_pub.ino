/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

std_msgs::Float32 pulse_msg;
ros::Publisher chatter("Pulse_signal", &pulse_msg);

void setup()
{
  pinMode(2, INPUT);
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  pulse_msg.data = digitalRead(2);
  chatter.publish( &pulse_msg );
  nh.spinOnce();
  delay(100);
}
