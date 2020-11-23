#include <ros.h>
#include <two_wheel/RPM1_Time.h>

// Gain
#define Kp 30
#define Ki 300
#define Kd 0.1

// Target Value
#define Target 2.5

// ROS
ros::NodeHandle nh;
two_wheel::RPM1_Time vol;
ros::Publisher chatter("Volume", &vol);

float duty = 0.0;
float dt, preTime,startTime;
float P, I, D, preP;

void setup(){
  pinMode(3, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(4, INPUT);
  
  nh.initNode();
  nh.advertise(chatter);
  
  while(digitalRead(4)==LOW){
    digitalWrite(7,HIGH);
  }
  digitalWrite(7,LOW);
  
  preTime=micros();
  startTime=preTime;
}

void loop(){
  analogWrite(3, duty);
  for(int i=0; i<1000; i++){
    vol.data+=analogRead(0);
  }
  vol.data=5.0*(vol.data/1000)/1024;
  vol.time=micros() - startTime;
  chatter.publish(&vol);
  
  PID();

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
