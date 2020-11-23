volatile int cntA=0, cntB=0.0;
double speed=0.0;

void setup(){
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  Serial.begin(19200);
  attachInterrupt(0, A, CHANGE);
  attachInterrupt(1, B, CHANGE);
}

void loop(){
  speed=(double)(cntA+cntB)/2592*600;
  cntA=0.0;
  cntB=0.0;
  Serial.print(speed);
  Serial.println(" rpm");
  delay(100);
}

void A(){
  cntA++;
}

void B(){
  cntB++;
}

