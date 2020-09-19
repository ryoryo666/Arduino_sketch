#include <ros.h>
#include <two_wheel/PID.h>
#include <TimerOne.h>
#include <std_msgs/Float32.h>

//Pin number
#define R_encoderA 2
#define R_encoderB 4
#define L_encoderA 3
#define L_encoderB 5
#define start 
#define r_motor_pwm 
#define l_motor_pwm 
#define LED 
#define m1 
#define m2

//Parameter
float Kp=2.0;
//float Ki=0.0;
float Kd=1.1;

float r_Target = 60.0;
float l_Target = 60.0;
float r_last_data = 0.0;
float l_last_data = 0.0;
float alpha = 0.01;

float r_duty = 0.0;
float l_duty = 0.0;
float r_dt, r_preTime;
float l_dt, l_preTime;
float r_P, r_I, r_D, r_preP = 0;
float l_P, r_I, l_D, l_preP = 0;
int r_i=0, l_i=0;
int flag=0;
volatile int r_encoderCnt=0;
volatile int l_encoderCnt=0;

ros::NodeHandle nh;
//  Subscriber setting
void messageCb(const std_msgs::Float32& new_target){
  r_Target=new_target.data;
  flag=1;
}
ros::Subscriber<std_msgs::Float32> sub("target_update", &messageCb);

//  Publisher setting
two_wheel::PID msg;
ros::Publisher chatter("rpm_data", &msg);

void setup(){
  pinMode(R_encoderA, INPUT);
  pinMode(R_encoderB, INPUT);
  digitalWrite(R_encoderA, HIGH);
  digitalWrite(R_encoderB, HIGH);
  pinMode(L_encoderA, INPUT);
  pinMode(L_encoerrB, INPUT);
  digitalWrite(L_encoderA, HIGH);
  digitalWrite(L_encoderB, HIGH);
  
  pinMode(start, INPUT);
  digitalWrite(m1, HIGH);  
  digitalWrite(m2, LOW);
  attachInterrupt(0, Right_Motor, CHANGE);
  attachInterrupt(1, Left_Motor, CHANGE);
  
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  
  while(digitalRead(start)==LOW){
    digitalWrite(LED,HIGH);
  }
  digitalWrite(LED,LOW);
  msg.data=0.0;
  msg.time=0.0;
  chatter.publish(&msg);
  Timer1.initialize();
  Timer1.attachInterrupt(wakeup,10000);
  
  r_preTime=micros();
  //startTime=r_preTime;
}

void loop(){
  nh.spinOnce();
}

void wakeup(){
  static float startTime=micros();
  msg.data=(float)r_encoderCnt/1296*6000; //  [r/0.01s] * [6000]  =  [rpm]
  msg.data=alpha*msg.data+(1-alpha)*r_last_data;
  r_encoderCnt=0;
  if(r_i==10){
    msg.time=micros()-startTime;
    //msg.time=r_duty*1000000;
    chatter.publish(&msg);
    r_i=0;
  }
  R_PID();
  if(r_duty>250){
    r_duty=250;
  }
  analogWrite(motor_pwm, abs(r_duty));
  r_last_data=msg.data;
  r_i++;
}

void R_PID(){
  r_dt = (micros() - r_preTime) / 1000000;
  r_preTime = micros();
  r_P  = r_Target - msg.data;
  //r_I += (r_P + r_preP)* r_dt;
  if(r_P < -15){
    r_D  = (r_P - r_preP) ;
  }else{
    r_D  = (r_P - r_preP) / r_dt;
  }
  r_preP = r_P;

  r_duty += Kp * r_P +/* Ki * I +*/ Kd * r_D;
  if(r_duty<0){
    r_duty=0;
  }
}


void Right_Motor(){
  if(digitalRead(R_encoderA)==digitalRead(R_encoderB)){
    r_encoderCnt++;
  }else{
    r_encoderCnt++;
  }
}

void Left_Motor(){
  if(digitalRead(L_encoderA)==digitalRead(L_encoderB)){
    l_encoderCnt++;
  }else{
    l_encoderCnt++;
  }
}
