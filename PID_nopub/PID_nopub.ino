#define Kp 30
#define Ki 300
#define Kd 0.1
#define Target 3

bool LED;
float duty = 0.0;
float vol = 0.0;
float dt, preTime;
float P, I, D, preP;

void setup(){
  pinMode(3, OUTPUT);
  
  digitalWrite(13, LED);
  preTime=micros();
  Serial.begin(19200);
}

void loop(){
  analogWrite(3, duty);
  for(int i=0; i<1000; i++){
    vol+=analogRead(0);
  }
  vol=5.0*(vol/1000)/1024;
  
  PID();
  
  Serial.print("Time:");Serial.print(preTime);Serial.print(" ");
  Serial.print("Input:");Serial.print(duty);Serial.print(" ");
  Serial.print("Mesure:");Serial.println(vol);
}

void PID(){
  dt = (micros() - preTime) / 1000000;
  preTime = micros();
  P  = Target - vol;
  I += P * dt;
  D  = (P - preP) / dt;
  preP = P;

  duty += Kp * P + Ki * I + Kd * D;
}
