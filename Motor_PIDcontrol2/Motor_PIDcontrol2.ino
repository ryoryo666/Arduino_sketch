#include <ros.h>
#include <two_wheel/PID.h>
#include <TimerOne.h>
#include <std_msgs/Float32.h>

//Pin number
#define encoderA 2
#define encoderB 3
#define start 4
#define motor_pwm 11
#define LED 7
#define m1 13
//Parameter
float Kp=1.0;
float Ki=0.0;
float Kd=0.5;

float Target=30;
float last_data=0.0;
float alpha=0.01;

float duty = 0.0;
float dt, preTime,startTime;
float P, I, D, preP=0;
int i=0;
volatile int encoderCnt=0;

ros::NodeHandle nh;
//  Subscriber setting
void messageCb(const std_msgs::Float32& new_target){
  Target=new_target.data;
}
ros::Subscriber<std_msgs::Float32> sub("target_update", &messageCb);

//  Publisher setting
two_wheel::PID msg;
ros::Publisher chatter("rpm_data", &msg);

void setup(){
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  digitalWrite(encoderA, HIGH);
  digitalWrite(encoderB, HIGH);
  pinMode(start, INPUT);
  digitalWrite(m1, LOW);  
  attachInterrupt(0, A, CHANGE);
  
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  
  while(digitalRead(start)==LOW){
    digitalWrite(LED,HIGH);
  }
  digitalWrite(LED,LOW);
  Timer1.initialize();
  Timer1.attachInterrupt(wakeup,10000);
  
  preTime=micros();
  startTime=preTime;
}

void loop(){
  nh.spinOnce();
}

void wakeup(){
  msg.data=(float)encoderCnt/1296*6000; //  [r/0.01s] * [6000]  =  [rpm]
  msg.data=alpha*msg.data+(1-alpha)*last_data;
  encoderCnt=0;
  if(i==10){
    msg.time=micros()-startTime;
    chatter.publish(&msg);
    i=0;
  }
  PID();
  if(duty>250){
    duty=250;
  }
  analogWrite(motor_pwm, abs(duty));
  last_data=msg.data;
  i++;
}

void PID(){
  dt = (micros() - preTime) / 1000000;
  preTime = micros();
  P  = Target - msg.data;
  I += P * dt;
  D  = (P - preP) / dt;
  preP = P;

  duty += Kp * P + Ki * I + Kd * D;
}

void A(){
  if(digitalRead(encoderA)==digitalRead(encoderB)){
    encoderCnt++;
  }else{
    encoderCnt++;
  }
}
