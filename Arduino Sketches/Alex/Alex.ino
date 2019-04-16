#include "serialize.h"
#include "packet.h"
#include "constants.h"
#include "alexsetup.h"
#include "responses.h"

volatile TDirection dir = STOP;
#define LF                  10   // Left forward pin
#define LR                  11   // Left reverse pin
#define RF                  5  // Right forward pin
#define RR                  6

/*
      Alex's State Variables
*/
// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long forwarddist;
volatile unsigned long reversedist;
volatile unsigned long leftangle;
volatile unsigned long rightangle;
volatile unsigned long rightticks;
volatile unsigned long leftticks;
//stores tick counts assume both encoders are accurate
volatile unsigned long forwardticks;
volatile unsigned long reverseticks;
volatile unsigned long ultra_dist_C;
volatile unsigned long angle;
volatile unsigned long obj_color;
volatile unsigned long over_ride;



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

/*
  Function to construct and send a command OK
  response to PI. 
  @param[in] failsafe tells if failsafe was activated during execution
*/
void sendOK(bool failsafe)
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  okPacket.params[0] = forwardticks;
  okPacket.params[1] = reverseticks;
  okPacket.params[2] = leftticks;
  okPacket.params[3] = rightticks;
  okPacket.params[4] = forwarddist;
  okPacket.params[5] = reversedist;
  okPacket.params[6] = obj_color;
  okPacket.params[7] = ultra_dist_C;
  okPacket.params[11] = leftangle;
  okPacket.params[12] = rightangle;
  okPacket.params[13] = over_ride;
  if (!failsafe)
  {
    okPacket.command = RESP_FAILSAFE;
  }
  sendResponse(&okPacket);
}

/*
  Function to construct and send a status
  response to PI. 
*/
void sendStatus()
{
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;

  statusPacket.params[0] = forwardticks;
  statusPacket.params[1] = reverseticks;
  statusPacket.params[2] = leftticks;
  statusPacket.params[3] = rightticks;
  statusPacket.params[4] = forwarddist;
  statusPacket.params[5] = reversedist;
  statusPacket.params[6] = obj_color;
  statusPacket.params[7] = ultra_dist_C;
  statusPacket.params[11] = leftangle;
  statusPacket.params[12] = rightangle;
  statusPacket.params[13] = over_ride;

  sendResponse(&statusPacket);
}

/*
  Function to increment ticks and calculate
  distance travelled since last counter-clear
  called by INT0
*/
void leftISR()
{
  if (dir == FORWARD) {
    forwardticks++;
    forwarddist = ((float)forwardticks / COUNTS_PER_REV) * WHEEL_CIRC;
  }
  else if (dir == BACKWARD) {
    reverseticks++;
    reversedist = ((float)reverseticks / COUNTS_PER_REV) * WHEEL_CIRC;
  }
  else if (dir == LEFT)
  {
    leftticks++;
    leftangle = ((((float)leftticks / COUNTS_PER_REV) * WHEEL_CIRC) / 31.4) * 360;
  }
  else if (dir == RIGHT)
  {
    rightticks++;
    rightangle = ((((float)rightticks / COUNTS_PER_REV) * WHEEL_CIRC) / 31.4) * 360;
  }
}

/*
  ISR for left motor encoder
*/
ISR(INT0_vect)
{
  leftISR();
}

/*
  function to execute "FORWARD" command
  called by handleCommand function
*/
bool forward(float dist)
{
  dir = FORWARD;
  long targetdist = forwarddist + dist;
  long dist_now = forwarddist;
  loopUSensor();
  long counter = 0;
  long old_ticks = reverseticks;
  while (forwarddist <= targetdist)
  {
    if (ultra_dist_C >= FAILSAFE || over_ride == OVER_ON)
    {
      float val = map(forwarddist, dist_now, targetdist, MAX_POWER, MIN_POWER);
      analogWrite(LF, min(255, max(MIN_POWER, val)));
      analogWrite(RF, min(255, max(MIN_POWER, val + BIAS)));
      analogWrite(LR, 0);
      analogWrite(RR, 0);
      loopUSensor();
      if (forwardticks == old_ticks)
      {
        counter++;
      }
      else
      {
        old_ticks = forwardticks;
        counter = 0;
      }
      if (counter == 1000)
      {
        stop();
        return false;
      }
    }
    else
    {
      stop();
      return false;
    }
  }
  stop();
  return true;
}

/*
  function to execute "REVERSE" command
  called by handleCommand function
*/
bool reverse(float dist)
{
  dir = BACKWARD;
  long targetdist = reversedist + dist;
  long dist_now = reversedist;
  long counter = 0;
  long old_ticks = reverseticks;
  while (reversedist < targetdist )
  {
    float val = map(reversedist, dist_now, targetdist, MAX_POWER, MIN_POWER);
    analogWrite(RR, min(255, max(MIN_POWER, val + BIAS)));
    analogWrite(LR, min(255, max(MIN_POWER, val)));
    analogWrite(LF, 0);
    analogWrite(RF, 0);
    if (reverseticks == old_ticks)
    {
      counter++;
    }
    else
    {
      old_ticks = reverseticks;
      counter = 0;
    }
    if (counter == 1000)
    {
      stop();
      return false;
    }
  }
  stop();
  return true;
}

/*
  function to execute "LEFT" command
  called by handleCommand function
*/
bool left(float ang)
{
  dir = LEFT;
  long angle_now = leftangle;
  long target_ang = leftangle + ang;
  long counter = 0;
  long old_ticks = leftticks;
  while (leftangle < target_ang || over_ride == OVER_ON)
  {
    float val = map(leftangle, angle_now, target_ang, MAX_POWER, MIN_POWER);
    analogWrite(RR, 0);
    analogWrite(LR, min(255, max(MIN_POWER, val)));
    analogWrite(LF, 0);
    analogWrite(RF, min(255, max(MIN_POWER, val + BIAS)));
    if (leftticks == old_ticks)
    {
      counter++;
    }
    else
    {
      old_ticks = leftticks;
      counter = 0;
    }
    if (counter == 1000)
    {
      stop();
      return false;
    }
  }
  stop();
  return true;
}

/*
  function to execute "RIGHT" command
  called by handleCommand function
*/
bool right(float ang)
{
  dir = RIGHT;
  long angle_now = rightangle;
  long target_ang = rightangle + ang;
  long counter = 0;
  long old_ticks = rightticks;
  while (rightangle < target_ang || over_ride == OVER_ON)
  {
    float val = map(rightangle, angle_now, target_ang, MAX_POWER, MIN_POWER);
    analogWrite(RR, min(255, max(MIN_POWER, val + BIAS)));
    analogWrite(LR, 0);
    analogWrite(LF, min(255, max(MIN_POWER, val)));
    analogWrite(RF, 0);
    if (rightticks == old_ticks)
    {
      counter++;
    }
    else
    {
      old_ticks = rightticks;
      counter = 0;
    }
    if (counter == 1000)
    {
      stop();
      return false;
    }
  }
  stop();
  return true;
}

/*
function to execute "STOP" command
called by handleCommand or movement functions
*/
void stop()
{
  dir = STOP;
    analogWrite(LF, 0);
    analogWrite(LR, 0);
    delay(100);
    analogWrite(RF, 0);
    analogWrite(RR, 0);
  loopColour();
  loopUSensor();
}


// Clears all our counters
void clearCounters()
{
  forwarddist = 0;
  reversedist = 0;
  forwardticks = 0;
  reverseticks = 0;
  leftticks = 0;
  rightticks = 0;
  leftangle = 0;
  rightangle = 0;
  over_ride = OVER_OFF;
}

void clearOneCounter(int which)
{
  clearCounters();
}

void initializeState()
{
  clearCounters();
}

/*
  Calls functions as needed according to command
  packet received
*/
void handleCommand(TPacket *command)
{
  bool all_good = true;
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:

      all_good = forward((float)command->params[0]);
      sendOK(all_good);
      break;

    case COMMAND_REVERSE:

      all_good = reverse((float)command->params[0]);
      sendOK(all_good);
      break;

    case COMMAND_TURN_LEFT:
      all_good = left((float) command->params[0]);
      sendOK(all_good);
      break;

    case COMMAND_TURN_RIGHT:
      all_good = right((float) command->params[0]);
      sendOK(all_good);
      break;

    case COMMAND_STOP:
      sendOK(all_good);
      stop();
      break;

    case COMMAND_GET_STATS:
      loopColour();
      loopUSensor();
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      clearOneCounter(command->params[0]);
      sendOK(all_good);
      break;

    case COMMAND_FSO:
      if (over_ride == OVER_ON)
      {
        over_ride = OVER_OFF;
      }
      else
      {
        over_ride = OVER_ON;
      }
      sendOK(all_good);
      break;

    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK(true);
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:
  stop();
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  setupUSensor();
  setupColour();
  sei();
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

void loop() {
  TPacket recvPacket; // This holds commands from the Pi

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
