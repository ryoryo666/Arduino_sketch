//Pin number
#define R_encoderA 2
#define R_encoderB 4
#define L_encoderA 3
#define L_encoderB 5
#define r_motor_pwm 11
#define l_motor_pwm 6
#define IN1 12
#define IN2 13


//Parameter
float Kp=10.0;
//float Ki=0.0;
float Kd=5.0;

float r_Target = 10.0;
float l_Target = 10.0;
float r_data;
float l_data;
float r_last_data = 0.0;
float l_last_data = 0.0;

float r_duty = 0.0;
float l_duty = 0.0;
float r_dt, r_preTime;
float l_dt, l_preTime;
float last_Time;
float r_P, r_I, r_D, r_preP = 0;
float l_P, l_I, l_D, l_preP = 0;
volatile int r_encoderCnt=0;
volatile int l_encoderCnt=0;

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

  Serial.begin(115200);

  r_preTime=micros();
  l_preTime=r_preTime;
  last_Time=micros();
}

void loop(){
  static int i=0;
  static float startTime=micros();
  r_data=(float)r_encoderCnt/(6*250)*100*60;
  l_data=(float)l_encoderCnt/(6*250)*100*60;
  r_data=0.01*r_data+(1-0.01)*r_last_data;
  l_data=0.01*l_data+(1-0.01)*l_last_data;  

  R_PID();
  L_PID();

  analogWrite(r_motor_pwm, abs(r_duty));
  analogWrite(l_motor_pwm, abs(l_duty));
  Serial.print(l_data);
  Serial.print("\t");
  Serial.println(r_data);
  Serial.print("\n");
    
  r_encoderCnt=0;
  l_encoderCnt=0;
  r_last_data=r_data;
  l_last_data=l_data;
 
  last_Time=micros();
  delay(5);
}


void R_PID(){
  r_dt = (micros() - r_preTime) / 1000000.0;
  r_preTime = micros();
  r_P  = r_Target - abs(r_data);
  //r_I += (r_P + r_preP)* r_dt;
  r_D  = (r_P - r_preP) / r_dt;
  r_preP = r_P;

  r_duty += Kp * r_P +/* Ki * I +*/ Kd * r_D;
  if(r_duty<0){
    r_duty=0;
  }else if(r_duty>250){
    r_duty=250;
  }
}

void L_PID(){
  l_dt = (micros() - l_preTime) / 1000000.0;
  l_preTime = micros();
  l_P  = l_Target - abs(l_data);
  //r_I += (r_P + r_preP)* r_dt;
  l_D  = (l_P - l_preP) / l_dt;
  l_preP = l_P;
 
  l_duty += Kp * l_P +/* Ki * I +*/ Kd * l_D;
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
  digitalWrite(IN1, LOW);  
  digitalWrite(IN2, LOW);
}

void CCW(){
  digitalWrite(IN1, HIGH);  
  digitalWrite(IN2, HIGH);
  r_Target = r_Target * -1;
  l_Target = l_Target * -1;
}
