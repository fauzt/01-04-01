#include "serialize.h"
#include <arduino.h>
#include "packet.h"
#include "constants.h"
#include "alexsetup.h"

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

void clearCounters()
{
  forwardticks = 0;
  reversedist = 0;
  forwardticks = 0;
  reverseticks = 0;
  leftturnticks = 0;
  rightturnticks = 0;
}

void stop()
{
  dir = STOP;
  analogWrite(LEFT_FOR, 0);
  analogWrite(RIGHT_FOR, 0);
  analogWrite(LEFT_RVR, 0);
  analogWrite(RIGHT_RVR, 0);
}

bool forward(float dist)
{
  dir = FORWARD;
  long targetdist = forwarddist + dist;
  long dist_now = forwarddist;
  while (forwarddist <= targetdist)
  {
    //if(ultrasonic <= FAILSAFE)
    //{
    int val = map(forwarddist, dist_now, targetdist, 0, (long)255 * (MAX_POWER / 100.0));
    analogWrite(LEFT_FOR, val);
    analogWrite(RIGHT_FOR, val);
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
    //if(ultrasonic <= FAILSAFE)
    //{
    int val = map(forwarddist, dist_now, targetdist, 0, (long)255 * (MAX_POWER / 100.0));
    analogWrite(LEFT_RVR, val);
    analogWrite(RIGHT_RVR, val);
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
    //if(ultrasonic <= FAILSAFE)
    //{
    int val = map(rightangdist, dist_now, targetang, 0, (long)255 * (MAX_POWER / 100.0));
    analogWrite(LEFT_FOR, val);
    analogWrite(RIGHT_RVR, val);
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
    //if(ultrasonic <= FAILSAFE)
    //{
    int val = map(leftangdist, dist_now, targetang, 0, (long)255 * (MAX_POWER / 100.0));
    analogWrite(RIGHT_FOR, val);
    analogWrite(LEFT_RVR, val);
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

void handleCommand(TPacket *command)
{
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float)command->params[0]);
      break;

    case COMMAND_REVERSE:
      sendOK();
      reverse((float)command->params[0]);
      break;

    case COMMAND_TURN_LEFT:
      sendOK();
      left((float)command->params[0]);
      break;

    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float)command->params[0]);
      break;

    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    case COMMAND_GET_STATS:
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      clearOneCounter(command->params[0]);
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
