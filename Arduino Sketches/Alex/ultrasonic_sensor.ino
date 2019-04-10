int trigPin = 13;    // Trigger
int echoPinC = 12;    // Echo
int echoPinL = 8;
int echoPinR = 9;
long duration, cm;
 
void setupUSensor() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPinL, INPUT);
 pinMode(echoPinC, INPUT);
 pinMode(echoPinR, INPUT);
}
 
long loopUSensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  if(dir == FORWARD || dir == STOP || dir == BACKWARD)
  {
  duration = pulseIn(echoPinC, HIGH);
  cm = (duration/2) / 29.1;
  ultra_dist_C = cm;
  }
  else if(dir == RIGHT)
  {
  duration = pulseIn(echoPinR, HIGH);
  cm = (duration/2) / 29.1;
  ultra_dist_R = cm;
  }
  else if(dir == LEFT)
  {
  duration = pulseIn(echoPinL, HIGH);
  cm = (duration/2) / 29.1;
  ultra_dist_L = cm;
  }

}
