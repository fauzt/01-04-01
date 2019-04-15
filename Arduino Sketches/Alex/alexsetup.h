#ifndef __ALEXSETUP_H__
#define __ALEXSETUP_H__

void enablePullups()
{
	// Use bare-metal to enable the pull-up resistors on pins
	// 2 and 3. These are pins PD2 and PD3 respectively.
	// We set bits 2 and 3 in DDRD to 0 to make them inputs.

	DDRD &= 0b11110011;
	PORTD |= 0b00001100;
}

void setupEINT()
{
	// Use bare-metal to configure pins 2 and 3 to be
	// falling edge triggered. Remember to enable
	// the INT0 and INT1 interrupts.

	EICRA = 0b00001010;
	EIMSK = 0b00000011;
	
}

void setupSerial()
{
	// To replace later with bare-metal.
//	Serial.begin(57600);
	 UCSR0C = 0b00000110;
	 UBRR0L = 16; //round(16000000/(16*baudrate)) - 1
	 UBRR0H = 0;
	 UCSR0A = 0;
}

void startSerial()
{
	// Empty for now. To be replaced with bare-metal code
	// later on.
	 UCSR0B = 0b10001000;
}
void setupMotors()
{
//	DDRB |= 0b00001100;
//	DDRD |= 0b01100000;
}
void startMotors()
{
//	TCNT0 = 0;
//  TCNT1 = 0;
//  TCNT2 = 0;
//	OCR0A = 0;
//	OCR0B = 0;
//	OCR1B = 0;
//	OCR2A = 0;
//	TCCR0A = 0b10100001;
//	TCCR1A = 0b10100001;
//	TCCR2A = 0b10100001;
//	TIMSK0 = 0b110;
//	TCCR0B = 0b00000001;
//	TCCR1B = 0b00000001;
//	TCCR2B = 0b00000001;
}
#endif
