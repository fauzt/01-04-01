/*
// TCS230 color recognition sensor 
// Sensor connection pins to Arduino are shown in comments

Color Sensor      Arduino
-----------      --------
 VCC               5V
 GND               GND
 s0                A0
 s1                A1
 s2                A2
 s3                A3
 OUT               8
 OE                GND
*/
const int s0 = A0;  
const int s1 = A1;  
const int s2 = A2;  
const int s3 = A3;  
const int out = 8;   
// Variables  
int red = 0;  
int green = 0;  
int blue = 0;  
    
void setupColour()   
{  
  pinMode(s0, OUTPUT);  
  pinMode(s1, OUTPUT);  
  pinMode(s2, OUTPUT);  
  pinMode(s3, OUTPUT);  
  pinMode(out, INPUT); 
  digitalWrite(s0, HIGH);  
  digitalWrite(s1, HIGH);  
}  
    
void loopColour() 
{  
  color(); 
  /*
  Serial.print("R Intensity:");  
  Serial.print(red, DEC);  
  Serial.print(" G Intensity: ");  
  Serial.print(green, DEC);  
  Serial.print(" B Intensity : ");  
  Serial.print(blue, DEC);  
  //Serial.println();  */

  if (red < blue && red < green && red < 35)
  {  
   //Serial.println(" - (Red Color)");
   obj_color = RED;
  }  

  else if (blue < red && blue < green)   
  {  
   //Serial.println(" - (Blue Color)"); 
   obj_color = WHITE;
  }  

  else if (green < red && green < blue)  
  {  
   //Serial.println(" - (Green Color)");
   obj_color = GREEN;
  }  
  else{
   obj_color = UNCLEAR;
  //Serial.println();  
  }
  delay(300);
 }  
    
void color()  
{    
  digitalWrite(s2, LOW);  
  digitalWrite(s3, LOW);  
  //count OUT, pRed, RED  
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
  digitalWrite(s3, HIGH);  
  //count OUT, pBLUE, BLUE  
  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
  digitalWrite(s2, HIGH);  
  //count OUT, pGreen, GREEN  
  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
}
