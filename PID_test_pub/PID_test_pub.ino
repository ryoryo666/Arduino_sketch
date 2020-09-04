#include <ros.h>
#include <std_msgs/Float32.h>

#define Kp 30
#define Ki 300
#define Kd 0.1
#define Target 2.5

ros::NodeHandle nh;
std_msgs::Float32 vol;
ros::Publisher chatter("Volume", &vol);

bool LED;
float duty = 0.0;
float dt, preTime;
float P, I, D, preP;

void setup(){
  pinMode(3, OUTPUT);
  
  nh.initNode();
  nh.advertise(chatter);
  
  digitalWrite(13, LED);
  preTime=micros();
}

void loop(){
  analogWrite(3, duty);
  for(int i=0; i<1000; i++){
    vol.data+=analogRead(0);
  }
  vol.data=5.0*(vol.data/1000)/1024;
  
  PID();
  
  chatter.publish(&vol);
  nh.spinOnce();
}

void PID(){
  dt = (micros() - preTime) / 1000000;
  preTime = micros();
  P  = Target - vol.data;
  I += P * dt;
  D  = (P - preP) / dt;
  preP = P;

  duty += Kp * P + Ki * I + Kd * D;
}
