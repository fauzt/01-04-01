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
  ISR functions
*/
void updatecounters()
{
  
  
  
  
}
void rightISR()
{
  if (dir == FORWARD)
  {
    forwardticks++;
    forwarddist = (forwardticks/COUNTS_PER_REV)*20.0;
  }
  else if (dir == BACKWARD)
  {
    reverseTicks++;
    reversedist = (reverseticks/COUNTS_PER_REV)*20.0;
  }

  else if (dir == LEFT)
  {
    leftturnticks++;
    leftangdist = (leftangticks/COUNTS_PER_REV)*20.0;
  }
  else if (dir == RIGHT)
  {
    rightturnticks++;
    rightangdist = (rightangticks/COUNTS_PER_REV)*20.0;
  }
  updatecounters();
}

ISR(INT0_vect)
{
  leftISR();
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
void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;

  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;

  //  statusPacket.params[0] = leftForwardTicks;
  //  statusPacket.params[1] = rightForwardTicks;
  //  statusPacket.params[2] = leftReverseTicks;
  //  statusPacket.params[3] = rightReverseTicks;
  //  statusPacket.params[4] = leftForwardTicksTurns;
  //  statusPacket.params[5] = rightForwardTicksTurns;
  //  statusPacket.params[6] = leftReverseTicksTurns;
  //  statusPacket.params[7] = rightReverseTicksTurns;
  //  statusPacket.params[8] = forwardDist;
  //  statusPacket.params[9] = reverseDist;

  sendResponse(&statusPacket);
}

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
