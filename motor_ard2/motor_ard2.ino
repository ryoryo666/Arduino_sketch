#include<TimerOne.h>

int encoder0PinA=2;
int encoder0PinB=4;
int IN=12;
int M_PWM=3;

int flag=0;
volatile unsigned long s=0;
volatile unsigned long ss=0;
int mode=0;
int Output;
double kp;
double ki;
double dT=0.01;
double t=0.0;
double th;
double dth;
double rpm;
double ref_rpm;
double alpha=0.01;
double err;
volatile double errSum=0.0;
volatile double last_th=0.0;
volatile double last_dth=0.0;
volatile unsigned long encoder0Pos;

int RPM=0;
int RPM_high=0;
int RPM_low=0;
int RSTime=0;
int RSTime_high=0;
int RSTime_low=0;

void setup(){
  pinMode(encoder0PinA,INPUT);
  pinMode(encoder0PinB,INPUT);
  digitalWrite(encoder0PinA,HIGH);
  digitalWrite(encoder0PinB,HIGH);
  attachInterrupt(0,doEncoder,CHANGE);

  kp=2.0;
  ki=4.0;
  encoder0Pos=0;
  digitalWrite(IN,0);
  Serial.begin(9600);
  Timer1.initialize();
  Timer1.attachInterrupt(wakeup,10000);
}

void wakeup(){
 if(flag==1){


    if(mode==0){
      mode=1;
    }
         
    if(s<=RSTime){    
      th=(double)(encoder0Pos*1.666);
      dth=(th-last_th)/dT;
      dth=alpha*dth+(1-alpha)*last_dth;

      ref_rpm=(double)(RPM)*0.5*(1.0-cos(3.141592*t/10.0));
      if(t<=10.0){
        t+=dT;
      }

      err=(double)ref_rpm*6.0-dth;
      errSum+=(err*dT);

      Output=(int)(kp*err+ki*errSum);
      if(Output>250){
        Output=250;
      }

      if(Output>0){
        analogWrite(M_PWM,Output);
      }else{
        Output=0;
        analogWrite(M_PWM,Output);
      }

      last_th=th;
      last_dth=dth;
      rpm=dth/6.0;
      ss++;
      s=ss/100;

      Serial.write(int(rpm)>>8);
      Serial.write(int(rpm)&255);
      Serial.write(int(s)>>8);
      Serial.write(int(s)&255);
      Serial.write(mode);
    }else{
      mode++;
      rpm=0;
      s=0;
      ss=0;
      flag=0;
      ResetState();
      Serial.write(int(rpm)>>8);
      Serial.write(int(rpm)&255);
      Serial.write(int(s)>>8);
      Serial.write(int(s)&255);
      Serial.write(mode);
    }   
  }  
}

void loop(){  
 
  
  if(Serial.available()>3){
    flag=1;
    RPM_high=Serial.read();
    RPM_low=Serial.read();
    RPM=(RPM_high<<8)+RPM_low;
    RSTime_high=Serial.read();
    RSTime_low=Serial.read();
    RSTime=(RSTime_high<<8)+RSTime_low;

    if(RPM==0&&RSTime==0){
      flag=0;
      rpm=0;
      s=0;
      ss=0;
      mode=0;
      ResetState();
      Serial.write(int(rpm)>>8);
      Serial.write(int(rpm)&255);
      Serial.write(int(s)>>8);
      Serial.write(int(s)&255);
      Serial.write(mode);      
    }
  }
}

void doEncoder(){
  if(digitalRead(encoder0PinA)==digitalRead(encoder0PinB)){
    encoder0Pos++;
  }else{
    encoder0Pos--;
  }
}

void ResetState(){
  t=0;
  th=0;
  dth=0;
  rpm=0;
  encoder0Pos=0;
  errSum=0.0;
  last_th=0.0;
  last_dth=0.0;
  analogWrite(M_PWM,0);
}

