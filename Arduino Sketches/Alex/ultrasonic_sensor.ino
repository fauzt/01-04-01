int trigPin = 13;    // Trigger
int echoPinL = 12;    // Echo
long durationL, cmL;
 
void setupUSensor() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPinL, INPUT);
//  pinMode(echoPinC, INPUT);
//  pinMode(echoPinR, INPUT);
}
 
long loopUSensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  durationL = pulseIn(echoPinL, HIGH);
  cmL = (durationL/2) / 29.1;
  return cmL;
}
