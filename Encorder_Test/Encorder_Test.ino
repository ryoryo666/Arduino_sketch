#define R_encoderA 2
#define R_encoderB 4

volatile int r_encoderCnt=0;

void setup() {
  // put your setup code here, to run once:
  pinMode(R_encoderA, INPUT);
  pinMode(R_encoderB, INPUT);
  digitalWrite(R_encoderA, HIGH);
  digitalWrite(R_encoderB, HIGH);

  attachInterrupt(0, Right_Motor, CHANGE);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(r_encoderCnt);
  delay(100);

}

void Right_Motor(){
  if(digitalRead(R_encoderA)==digitalRead(R_encoderB)){
    r_encoderCnt++;
  }
  else{
    r_encoderCnt--;
  }
}
