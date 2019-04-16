#ifndef __RESPONSES_H__
#define __RESPONSES_H__
#include "constants.h"
#include "packet.h"

/*
	File contains response functions
*/
int readSerial(char *buffer)
{

  int count = 0;

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

void sendMessage(const char *message)
{
	// Sends text messages back to the Pi. Useful
	// for debugging.

	TPacket messagePacket;
	messagePacket.packetType = PACKET_TYPE_MESSAGE;
	strncpy(messagePacket.data, message, MAX_STR_LEN);
	sendResponse(&messagePacket);
}

void sendBadPacket()
{
	// Tell the Pi that it sent us a packet with a bad
	// magic number.

	TPacket badPacket;
	badPacket.packetType = PACKET_TYPE_ERROR;
	badPacket.command = RESP_BAD_PACKET;
	sendResponse(&badPacket);
}

void sendBadChecksum()
{
	// Tell the Pi that it sent us a packet with a bad
	// checksum.

	TPacket badChecksum;
	badChecksum.packetType = PACKET_TYPE_ERROR;
	badChecksum.command = RESP_BAD_CHECKSUM;
	sendResponse(&badChecksum);
}

void sendBadCommand()
{
	// Tell the Pi that we don't understand its
	// command sent to us.

	TPacket badCommand;
	badCommand.packetType = PACKET_TYPE_ERROR;
	badCommand.command = RESP_BAD_COMMAND;
	sendResponse(&badCommand);
}

void sendBadResponse()
{
	TPacket badResponse;
	badResponse.packetType = PACKET_TYPE_ERROR;
	badResponse.command = RESP_BAD_RESPONSE;
	sendResponse(&badResponse);
}




#endif
