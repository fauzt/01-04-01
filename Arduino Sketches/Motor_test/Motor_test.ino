#include "Arduino.h"
#include <avr/io.h>
#include <avr/interrupt.h>

int R_motor_f = 10;
int L_motor_f = 6;
int R_motor_r = 11;
int L_motor_r = 5;


#define srt 170
#define fin 0
static volatile char whichdirection = 0;
char dataRecv = 's', dataSend;

ISR(USART_RX_vect)
{
  // Write received data to dataRecv
  dataRecv = UDR0;
}

char recvData()
{
  return dataRecv;
}

void all_stop()
{
    analogWrite(R_motor_f, fin);
    analogWrite(L_motor_f, fin);
    analogWrite(R_motor_r, fin);
    analogWrite(L_motor_r, fin);
}

void forward()
{
    analogWrite(R_motor_f, srt);
    analogWrite(L_motor_f, srt);
    analogWrite(R_motor_r, fin);
    analogWrite(L_motor_r, fin);
}

void back()
{
    analogWrite(R_motor_f, fin);
    analogWrite(L_motor_f, fin);
    analogWrite(R_motor_r, srt);
    analogWrite(L_motor_r, srt);
}

void right_turn()
{
    analogWrite(R_motor_f, fin);
    analogWrite(L_motor_f, srt);
    analogWrite(R_motor_r, srt);
    analogWrite(L_motor_r, fin);
}

void left_turn()
{
    analogWrite(R_motor_f, srt);
    analogWrite(L_motor_f, fin);
    analogWrite(R_motor_r, fin);
    analogWrite(L_motor_r, srt);
}
void setup() {
  // put your setup code here, to run once:
  cli();
  UCSR0C = 0b00000110;
  UBRR0L = 16; //round(16000000/(16*baudrate)) - 1
  UBRR0H = 0;
  UCSR0A = 0;
  UCSR0B = 0b10010000;
  sei();
}

void loop() {
  // put your main code here, to run repeatedly:
  whichdirection = recvData();

  if (whichdirection == 's') //all stop
  {
    all_stop();
  }
  else if(whichdirection == 'f') //forward
  {
    forward();
//    delay(250);
//    all_stop();
  }
  else if(whichdirection == 'b') //backward
  {
    back();
//    delay(250);
//    all_stop();
  }
  else if(whichdirection == 'r') //turn right
  {
    right_turn();
//    delay(250);
//    all_stop();
  }
  else if(whichdirection == 'l') //turn right
  {
    left_turn();
//    delay(250);
//    all_stop();
  }
}
