#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include "serial.h"

#define PORT_NAME	"/dev/ttyACM1" //change portname here to Arduino port 
int main()
{
	char buffer[MAX_BUFFER_LEN];

	memset(buffer, 0, MAX_BUFFER_LEN);

	startSerial(PORT_NAME, B57600, 8, 'N', 1, 5); //serial settings

	if(fork() != 0)
	{
		while(1)
		{
			// Parent will read keyboard
			char ch = getchar();
			if(ch == 's' || ch == 'f' || ch == 'b' || ch == 'r' || ch == 'l') //only push if it reads these commands
				serialWrite(&ch, sizeof(char));
		}

	} 
	else
	{
		while(1)
		{
			int n;
			n=serialRead(buffer);
			buffer[n]=0;
			printf("Received %d bytes: %s\n", n, buffer);
		}
	}
}


