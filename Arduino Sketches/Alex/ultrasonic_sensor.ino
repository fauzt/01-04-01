int trigPin = 13;    // Trigger
int echoPinC = 12;    // Echo
long duration, cm;

/*
  file contains ultrasonic function
*/
void setupUSensor() {
  pinMode(trigPin, OUTPUT);
 pinMode(echoPinC, INPUT);
}
 
long loopUSensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPinC, HIGH);
  cm = (duration/2) / 29.1;
  ultra_dist_C = cm;
}
