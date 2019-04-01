#include "serialize.h"
#include <arduino.h>
#include "packet.h"
#include "constants.h"
#include "alexsetup.h"
#include "responses.h"

volatile TDirection dir = STOP;
//stores forward/reverse distance travelled
volatile unsigned long forwarddist;
volatile unsigned long reversedist;
volatile unsigned long rightangdist;
volatile unsigned long leftangdist;
//stores tick counts assume both encoders are accurate
volatile unsigned long forwardticks;
volatile unsigned long reverseticks;
volatile unsigned long leftturnticks;
volatile unsigned long rightturnticks;

/*
send ok function with status
*/
void sendOK()
{
	TPacket okPacket;
	okPacket.packetType = PACKET_TYPE_RESPONSE;
	okPacket.command = RESP_OK;
	okPacket.params[0] = forwardticks;
  okPacket.params[1] = reverseticks;
  okPacket.params[2] = leftturnticks;
  okPacket.params[3] = rightturnticks;
  okPacket.params[4] = forwarddist;
  okPacket.params[5] = reversedist;
  okPacket.params[6] = leftangdist;
  okPacket.params[7] = rightangdist;
	sendResponse(&okPacket);
}

void sendStatus()
{
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = forwardticks;
  statusPacket.params[1] = reverseticks;
  statusPacket.params[2] = leftturnticks;
  statusPacket.params[3] = rightturnticks;
  statusPacket.params[4] = forwarddist;
  statusPacket.params[5] = reversedist;
  statusPacket.params[6] = leftangdist;
  statusPacket.params[7] = rightangdist;

  sendResponse(&statusPacket);
}

/*
  ISR functions
*/
void rightISR()
{
  if (dir == FORWARD)
  {
    forwardticks++;
    forwarddist = (forwardticks/COUNTS_PER_REV)*20.0;
  }
  else if (dir == BACKWARD)
  {
    reverseticks++;
    reversedist = (reverseticks/COUNTS_PER_REV)*20.0;
  }

  else if (dir == LEFT)
  {
    leftturnticks++;
    leftangdist = (leftturnticks/COUNTS_PER_REV)*20.0;
  }
  else if (dir == RIGHT)
  {
    rightturnticks++;
    rightangdist = (rightturnticks/COUNTS_PER_REV)*20.0;
  }
}

ISR(INT0_vect)
{
//  leftISR();
}
ISR(INT1_vect)
{
  rightISR();
}

void clearCounters()
{
  forwardticks = 0;
  reversedist = 0;
  forwardticks = 0;
  reverseticks = 0;
  leftturnticks = 0;
  rightturnticks = 0;
}

/*
  Movement functions
*/


void stop()
{
  dir = STOP;
  OCR0A = 0;
  OCR1B = 0;
  OCR2A = 0;
  OCR0B = 0;
}

bool forward(float dist)
{
  dir = FORWARD;
  long targetdist = forwarddist + dist;
  long dist_now = forwarddist;
  while (forwarddist <= targetdist)
  {
    //if(ultrasonic >= FAILSAFE)
    //{
    int val = map(forwarddist, dist_now, targetdist, (long)255 * (MAX_POWER / 100.0), 0);
    OCR0A = val;
    OCR1B = val;
    //}
    // else
    // {
    // 	stop();
    // 	return false;
    // }
  }
  stop();
  return true;
}

bool reverse(float dist)
{
  dir = FORWARD;
  long targetdist = reversedist + dist;
  long dist_now = reversedist;
  while (reversedist <= targetdist)
  {
    //if(ultrasonic >= FAILSAFE)
    //{
    int val = map(forwarddist, dist_now, targetdist, (long)255 * (MAX_POWER / 100.0), 0);
    OCR2A = val;
    OCR0B = val;
    //}
    // else
    // {
    // 	stop();
    // 	return false;
    // }
  }
  stop();
  return true;
}

bool right(float ang)
{
  dir = RIGHT;
  long targetang = rightangdist + ang;
  long dist_now = rightangdist;
  while (rightangdist <= targetang)
  {
    //if(ultrasonic >= FAILSAFE)
    //{
    int val = map(rightangdist, dist_now, targetang, (long)255 * (MAX_POWER / 100.0), 0);
    OCR0A = val;
    OCR0B = val;
    //}
    // else
    // {
    // 	stop();
    // 	return false;
    // }
  }
  stop();
  return true;
}

bool left(float ang)
{
  dir = LEFT;
  long targetang = leftangdist + ang;
  long dist_now = leftangdist;
  while (leftangdist <= targetang)
  {
    //if(ultrasonic >= FAILSAFE)
    //{
    int val = map(leftangdist, dist_now, targetang, (long)255 * (MAX_POWER / 100.0), 0);
    OCR1B = val;
    OCR2A = val;
    //}
    // else
    // {
    // 	stop();
    // 	return false;
    // }
  }
  stop();
  return true;
}

/*
Packet Handlers
*/
TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;
  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);
}

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch (command->command)
  {
  // For movement commands, param[0] = distance, param[1] = speed.
  case COMMAND_FORWARD:
    forward((float)command->params[0]);
    sendOK();
    break;

  case COMMAND_REVERSE:
    reverse((float)command->params[0]);
    sendOK();
    break;

  case COMMAND_TURN_LEFT:

    left((float)command->params[0]);
    sendOK();
    break;

  case COMMAND_TURN_RIGHT:

    right((float)command->params[0]);
    sendOK();
    break;

  case COMMAND_STOP:
    stop();
    sendOK();
    break;

  case COMMAND_GET_STATS:
    sendStatus();
    break;

  case COMMAND_CLEAR_STATS:
    //      clearOneCounter(command->params[0]);
    sendOK();
    break;

  default:
    sendBadCommand();
  }
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
  case PACKET_TYPE_COMMAND:
    handleCommand(packet);
    break;
  case PACKET_TYPE_RESPONSE:
    break;
  case PACKET_TYPE_ERROR:
    break;
  case PACKET_TYPE_MESSAGE:
    break;
  case PACKET_TYPE_HELLO:
    break;
  }
}

void setup()
{
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
}

void loop()
{
  TPacket recvPacket;
  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  }
}
