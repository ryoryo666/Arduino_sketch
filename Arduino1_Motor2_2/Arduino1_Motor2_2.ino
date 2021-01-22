#include <ros.h>
#include <two_wheel/RL_RPM.h>
#include <two_wheel/RightLeft_cmd_value.h>

//Pin number
#define R_encoderA 2
#define R_encoderB 4
#define L_encoderA 3
#define L_encoderB 7
#define r_motor_pwm 5
#define l_motor_pwm 6
#define IN1 12
#define IN2 13


//Parameter
float Kp=1.0;
//float Ki=0.0;
float Kd=0.75;

float r_Target = 0.0;
float l_Target = 0.0;
float r_data;
float l_data;
float r_last_data = 0.0;
float l_last_data = 0.0;

float r_duty = 0.0;
float l_duty = 0.0;
float r_dt, r_preTime;
float l_dt, l_preTime;
float r_P, r_I, r_D, r_preP = 0;
float l_P, l_I, l_D, l_preP = 0;
volatile int r_encoderCnt=0;
volatile int l_encoderCnt=0;

ros::NodeHandle nh;
//  Subscriber setting
void messageCb(const two_wheel::RightLeft_cmd_value& new_target){
  r_Target=new_target.r_ref;
  l_Target=new_target.l_ref;
  if(r_Target < 0 && l_Target < 0){
    CCW();
  }else if(r_Target > 0 && l_Target > 0){
    CW();
  }else if(r_Target == 0 || l_Target == 0){
    STOP();
  }
}
ros::Subscriber<two_wheel::RightLeft_cmd_value> sub("New_cmd", messageCb);


//  Publisher setting
two_wheel::RL_RPM msg;
ros::Publisher chatter("Encoder_data", &msg);

void setup(){
  pinMode(R_encoderA, INPUT);
  pinMode(R_encoderB, INPUT);
  digitalWrite(R_encoderA, HIGH);
  digitalWrite(R_encoderB, HIGH);
  pinMode(L_encoderA, INPUT);
  pinMode(L_encoderB, INPUT);
  digitalWrite(L_encoderA, HIGH);
  digitalWrite(L_encoderB, HIGH);

  digitalWrite(IN1, LOW);  
  digitalWrite(IN2, LOW);
  attachInterrupt(0, Right_Motor, CHANGE);
  attachInterrupt(1, Left_Motor, CHANGE);

  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);

  r_preTime=micros();
  l_preTime=r_preTime;
}

void loop(){
  static int i=0;
  static float startTime=micros();
  r_data=(float)r_encoderCnt*((2*3.1415)/1500)*200;  // [rad/s]
  l_data=(float)l_encoderCnt*((2*3.1415)/1500)*200;  // [rad/s]
  r_data=0.01*r_data+(1-0.01)*r_last_data;
  l_data=0.01*l_data+(1-0.01)*l_last_data;  

  R_PID();
  L_PID();

  analogWrite(r_motor_pwm, abs(r_duty));
  analogWrite(l_motor_pwm, abs(l_duty));

  i++;
  if(i==10){
    if(r_encoderCnt==0){
      msg.r_data=0.0;
    }else{
      msg.r_data=r_data;
    }
    
    if(l_encoderCnt==0){
      msg.l_data=0.0;
    }else{
      msg.l_data=l_data;
    }
    chatter.publish(&msg);  // 20Hz Publish
    i=0;
    }
    
  r_encoderCnt=0;
  l_encoderCnt=0;
  r_last_data=r_data;
  l_last_data=l_data;
 
  nh.spinOnce();
  delay(5);
}


void R_PID(){
  r_dt = (micros() - r_preTime) / 1000000;
  r_preTime = micros();
  r_P  = r_Target - abs(r_data);
  //r_I += (r_P + r_preP)* r_dt;
  r_D  = (r_P - r_preP) / r_dt;
  r_preP = r_P;

  r_duty += Kp * r_P /*+ Ki * r_I */+ Kd * r_D;
  if(r_duty<0){
    r_duty=0;
  }else if(r_duty>250){
    r_duty=250;
  }
}

void L_PID(){
  l_dt = (micros() - l_preTime) / 1000000;
  l_preTime = micros();
  l_P  = l_Target - abs(l_data);
  //l_I += (r_P + r_preP)* r_dt;
  l_D  = (l_P - l_preP) / l_dt;
  l_preP = l_P;
 
  l_duty += Kp * l_P /*+ Ki * l_I */+ Kd * l_D;
  if(l_duty<0){
    l_duty=0;
  }else if(l_duty>250){
    l_duty=250;
  }
}


void Right_Motor(){
  if(digitalRead(R_encoderA)==digitalRead(R_encoderB)){
    r_encoderCnt++;
  }
  else{
    r_encoderCnt--;
  }
}

void Left_Motor(){
  if(digitalRead(L_encoderA)==digitalRead(L_encoderB)){
    l_encoderCnt--;
  }
  else{
    l_encoderCnt++;
  }
}

void CW(){
  STOP();
  digitalWrite(IN1, LOW);  
  digitalWrite(IN2, LOW);
}

void CCW(){
  STOP();
  digitalWrite(IN1, HIGH);  
  digitalWrite(IN2, HIGH);
  r_Target = r_Target * -1;
  l_Target = l_Target * -1;
}

void STOP(){
  digitalWrite(9,HIGH);
  digitalWrite(8,HIGH);
}
