//Pin number
#define R_encoderA 2
#define R_encoderB 4
#define L_encoderA 3
#define L_encoderB 5
#define r_motor_pwm 11
#define l_motor_pwm 6
#define LED 7
#define start 8
#define IN1 12
#define IN2 13

int gear=250;
int duty=10;
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
  Serial.begin(9600);

}

void loop(){
  static int i=0;
  static float data_r=0.0;
  static float data_l=0.0;
  static float last_data_r=0.0,last_data_l=0.0;
  data_r=(float)r_encoderCnt/(12*gear)*100*60;
  data_l=(float)l_encoderCnt/(12*gear)*100*60;
  data_r=0.01*data_r+(1-0.01)*last_data_r;
  data_l=0.01*data_l+(1-0.01)*last_data_l;

  if(i==10){
    Serial.print(l_encoderCnt);
    Serial.print(" \t");
    Serial.print(r_encoderCnt);
    Serial.println(" ");
    i=0;
  }

  r_encoderCnt=0;
  l_encoderCnt=0;

  analogWrite(r_motor_pwm, duty);
  analogWrite(l_motor_pwm, duty);
  last_data_r=data_r;
  last_data_l=data_l;
  i++;

  delay(10);
  
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
