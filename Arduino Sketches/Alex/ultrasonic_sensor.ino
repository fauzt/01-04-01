int trigPin = 13;    // Trigger
int echoPinL = 12;    // Echo
//int echoPinC = 9;    // Echo
//int echoPinR = 8;    // Echo
long durationL, cmL;
//long durationC,durationR, cmC, cmR;
//int count = 0;
 
void setupUSensor() {
  //Serial Port begin
  Serial.begin (9600);
//Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPinL, INPUT);
//  pinMode(echoPinC, INPUT);
//  pinMode(echoPinR, INPUT);
}
 
long loopUSensor() { //165ms delay
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  /*
  pinMode(echoPinL, INPUT);
  pinMode(echoPinC, INPUT);
  pinMode(echoPinR, INPUT);
  durationL = pulseIn(echoPinL, HIGH);
  durationC = pulseIn(echoPinC, HIGH);
  durationR = pulseIn(echoPinR, HIGH);*/

  //if (count==0)
  //{
    durationL = pulseIn(echoPinL, HIGH);
    //count=1;
    cmL = (durationL/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
    Serial.print("Left distance: ");
    Serial.print(cmL);
    Serial.print("cm ");
  //}
  /*else if (count==1)
  {
    durationC = pulseIn(echoPinC, HIGH);
    count=2;
    cmC = (durationC/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
    Serial.print("Centre distance: ");
    Serial.print(cmC);
    Serial.print("cm ");
  }
  else if (count==2)
  {
    pinMode(echoPinR, INPUT);
    durationR = pulseIn(echoPinR, HIGH);
    count=0;
    cmR = (durationR/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
    Serial.print("Right distance: ");
    Serial.print(cmR);
    Serial.print("cm ");
    Serial.println();
  }
  */
  delay(150);

  return cmL;
}
