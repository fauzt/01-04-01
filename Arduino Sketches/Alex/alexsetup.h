#ifndef __ALEXSETUP_H_
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
	Serial.begin(57600);
}

void startSerial()
{
	// Empty for now. To be replaced with bare-metal code
	// later on.
}
void setupMotors()
{
	/* Our motor set up is:
        A1IN - Pin 5, PD5, OC0B
        A2IN - Pin 6, PD6, OC0A
        B1IN - Pin 10, PB2, OC1B
        B2In - pIN 11, PB3, OC2A
  */
}
void startMotors()
{
}
#endif