#include <ros.h>
#include <two_wheel/PID.h>

//Pin number
#define encoderA 2
#define encoderB 3
#define start 4
#define motor_pwm 11
#define LED 7
#define m1 13
//Parameter
#define Kp 1.5
#define Ki 0
#define Kd 0.1
#define Target 20

float duty = 0.0;
float dt, preTime;
float P, I, D, preP;
volatile int encoderCnt=0;

ros::NodeHandle nh;
two_wheel::PID msg;
ros::Publisher chatter("rpm_data", &msg);

void setup(){
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  digitalWrite(encoderA, HIGH);
  digitalWrite(encoderB, HIGH);
  pinMode(start, INPUT);
  pinMode(motor_pwm, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(m1, OUTPUT);
  digitalWrite(m1, HIGH);
  attachInterrupt(0, A, CHANGE);
  
  nh.initNode();
  nh.advertise(chatter);
  
  while(digitalRead(start)==LOW){
    digitalWrite(LED,HIGH);
  }
  digitalWrite(LED,LOW);
  
  preTime=micros();
}

void loop(){
  static float startTime=preTime;
  analogWrite(motor_pwm, duty);
  msg.data=(float)(cntA+cntB)/1296*600;
  msg.time=micros()-startTime;
  cntA=0.0;
  cntB=0.0;
  chatter.publish(&msg);
  
  PID();

  nh.spinOnce();
  delay(100);
}

void A(){
  if(digitalRead(encoderA)==digitalRead(encoderB)){
    encoderCnt++;
  }else{
    encoderCnt--;
  }
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
