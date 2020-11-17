#include <TimerOne.h>

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

float duty = 0.0, data;
float dt, preTime;
float P, I, D, preP=0;
int i=0;
volatile int encoderCnt=0;


void setup(){
  pinMode(start, INPUT);
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  digitalWrite(encoderA, HIGH);
  digitalWrite(encoderB, HIGH);
  digitalWrite(IN1, LOW);  
  digitalWrite(IN2, HIGH); 
  attachInterrupt(0, A, CHANGE);
  
  while(digitalRead(start)==LOW){
    digitalWrite(LED,LOW);
  }
  digitalWrite(LED,HIGH);
  Timer1.initialize();
  Timer1.attachInterrupt(wakeup,10000);
  Serial.begin(9600);
  
  preTime=micros();
  //startTime=preTime;
}

void loop(){
  Serial.println(data);
  delay(100);
}

void wakeup(){
  static float startTime=micros();
  data=(float)encoderCnt/648*6000; //  [r/0.01s] * [6000]  =  [rpm]
  data=alpha*data+(1-alpha)*last_data;
  encoderCnt=0;
  PID();
  if(duty>250){
    duty=250;
  }
  analogWrite(motor_pwm,duty);
  last_data=data;
  i++;
}

void PID(){
  dt = (micros() - preTime) / 1000000;
  preTime = micros();
  P  = Target - data;
  I += (P + preP)* dt;
  if(P < -15){
    D  = (P - preP) ;
  }else{
    D  = (P - preP) / dt;
  }
  preP = P;

  duty += Kp * P +/* Ki * I +*/ Kd * D;
  if(duty<0){
    duty=0;
  }
}

void A(){
  if(digitalRead(encoderA)==digitalRead(encoderB)){
    encoderCnt++;
  }else{
    encoderCnt++;
  }
}
