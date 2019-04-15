// TCS230 or TCS3200 pins wiring to Arduino
#define S0 A0 //pc0
#define S1 A1
#define S2 A2
#define S3 A3 //pc3
#define sensorOut 8 //pd7

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
//int blueFrequency = 0;
int clearFrequency = 0;

// Stores the red. green and blue colors
int redColor = 0;
int greenColor = 0;
//int blueColor = 0;
int clearColor = 0;

void setupColour() {
  
  // Setting the outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
  // setting DDRC and DDRD, s0-s3 output and sensorOut input
  //DDRC = B00001111;
  //DDRD = B00000000;
  
  // Setting the sensorOut as an input
  pinMode(sensorOut, INPUT);
  
  // Setting frequency scaling to 20%
  //PORTC |= B00000001;//turn s0 high
  //PORTC &= B11111101;//turn s1 low
  
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
}

void loopColour() {
  // Setting RED (R) filtered photodiodes to be read
  //PORTC &= B11111011;//turn s2 low
  //PORTC &= B11110111;//turn s3 low
  
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  
  // Reading the output frequency
  redFrequency = pulseIn(sensorOut, LOW);
  // Remaping the value of the RED (R) frequency from 0 to 255
  // You must replace with your own values. Here's an example: 
  // redColor = map(redFrequency, 70, 120, 255,0);
  redColor = map(redFrequency, 41, 46, 255,0);
  
  // Setting GREEN (G) filtered photodiodes to be read
  //PORTC |= B00000100;//turn s2 high
  //PORTC |= B00001000;//turn s3 high
  
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  greenFrequency = pulseIn(sensorOut, LOW);
  greenColor = map(greenFrequency, 27, 40, 255, 0);
 /*
  // Setting BLUE (B) filtered photodiodes to be read
  PORTC &= B11111011;//turn s2 low
  PORTC |= B00001000;//turn s3 high
  
  //digitalWrite(S2,LOW);
  //digitalWrite(S3,HIGH);
  // Reading the output frequency
  blueFrequency = pulseIn(sensorOut, LOW);
  blueColor = map(blueFrequency, 50, 115, 255, 0);
  
  // Printing the BLUE (B) value 
  Serial.print(" Blue = ");
  Serial.print(blueColor);
  delay(100);*/
  
  //Setting clear (c) filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,LOW);
  
  // Reading the output frequency
  clearFrequency = pulseIn(sensorOut, LOW);
  clearColor = map(clearFrequency, 16, 24, 255, 0);
  
  // Checks the current detected color and prints
  // a message in the serial monitor
  /*if(redColor > greenColor && redColor > blueColor && clearColor<=-250){
      Serial.println(" - RED detected!");
  }
  if(greenColor > redColor && greenColor > blueColor && clearColor<=-250){
      Serial.println(" - GREEN detected!");
  }*/
  //magic number of 8 on ultrasonic
  if(redFrequency >= 40 && clearColor<=100){
      obj_color = RED;
  }
  else if(greenFrequency <= 35 && clearColor<=100){
    obj_color = GREEN;
  }
  else if (clearColor > 100){
    obj_color = WHITE;
  }
//  if(redFrequency < greenFrequency && redFrequency < clearFrequency)
//    obj_color = RED;
//  else if(greenFrequency < redFrequency && greenFrequency < clearFrequency)
//    obj_color = GREEN;
//  else if(clearFrequency < redFrequency && clearFrequency < greenFrequency)
//    obj_color = WHITE;
  else
    obj_color = UNCLEAR;
}
