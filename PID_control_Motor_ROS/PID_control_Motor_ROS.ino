#include <ros.h>
#include <two_wheel/RPM1_Time.h>
#include <std_msgs/Float32.h>

//Pin number
#define encoderA 3
#define encoderB 5
#define start 8
#define motor_pwm 6
#define LED 7
#define IN1 12
#define IN2 13

//Parameter
float Kp=2.0;
float Ki=0.0;
float Kd=1.1;

float Target=20.0;
float last_data=0.0;
float alpha=0.01;

float duty = 0.0;
float dt, preTime;
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
two_wheel::RPM1_Time msg;
ros::Publisher chatter("rpm_data", &msg);

void setup(){
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  digitalWrite(encoderA, HIGH);
  digitalWrite(encoderB, HIGH);
  pinMode(start, INPUT);
  digitalWrite(IN1, HIGH);  
  digitalWrite(IN2, LOW); 
  attachInterrupt(0, A, CHANGE);
  
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  
  while(digitalRead(start)==LOW){
    digitalWrite(LED,HIGH);
  }
  digitalWrite(LED,LOW);
 
  preTime=micros();
  //startTime=preTime;
}

void loop(){
  static float startTime=micros();
  msg.data=(float)encoderCnt/1296*6000; //  [r/0.01s] * [6000]  =  [rpm]
  msg.data=alpha*msg.data+(1-alpha)*last_data;
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
  encoderCnt=0;
  i++;
  nh.spinOnce();
}

void PID(){
  dt = (micros() - preTime) / 1000000;
  preTime = micros();
  P  = Target - msg.data;
  I += (P + preP)* dt;
  D  = (P - preP) / dt;
  preP = P;

  duty += Kp * P + Ki * I + Kd * D;
  if(duty<0){
    duty=0;
  }
}

void A(){
  if(digitalRead(encoderA)==digitalRead(encoderB)){
    encoderCnt++;
  }else{
    encoderCnt--;
  }
}
