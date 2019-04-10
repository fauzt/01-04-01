#include "serialize.h"
#include "packet.h"
#include "constants.h"
#include "alexsetup.h"
#include "responses.h"

volatile TDirection dir = STOP;
#define LF                  10   // Left forward pin  //HOTFIX: hardware configured incorrectly
#define LR                  11   // Left reverse pin  //HOTFIX
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
volatile unsigned long ultra_dist_L;
volatile unsigned long ultra_dist_C;
volatile unsigned long ultra_dist_R;
volatile unsigned long angle;
volatile unsigned long obj_color;
volatile unsigned long over_ride;

/*-----------------------------GYRO Definitions & variables-------------------------------*/

// Uncomment the following line to use a MinIMU-9 v5 or AltIMU-10 v5. Leave commented for older IMUs (up through v4).
#define IMU_V5
int SENSOR_SIGN[9] = {1, 1, 1, -1, -1, -1, 1, 1, 1};
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

#define M_X_MIN -1000
#define M_Y_MIN -1000
#define M_Z_MIN -1000
#define M_X_MAX +1000
#define M_Y_MAX +1000
#define M_Z_MAX +1000

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

/* ----------------------------------End of GYRO Definitions-------------------------------------------- */

float G_Dt = 0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer = 0;   //general purpuse timer
long timer_old;
long timer24 = 0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6] = {0, 0, 0, 0, 0, 0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3] = {0, 0, 0}; //Store the acceleration in a vector
float Gyro_Vector[3] = {0, 0, 0}; //Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; //Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0}; //Omega Proportional correction
float Omega_I[3] = {0, 0, 0}; //Omega Integrator
float Omega[3] = {0, 0, 0};

// Euler angles
float roll;
float pitch;
float yaw; //ranges from -180 to 180

float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};

unsigned int counter = 0;
byte gyro_sat = 0;

float DCM_Matrix[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};
float Update_Matrix[3][3] = {
  {0, 1, 2},
  {3, 4, 5},
  {6, 7, 8}
}; //Gyros here

float Temporary_Matrix[3][3] = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};

/*-------------------------------End of GYRO Definitions & variables---------------------------------------*/




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
  okPacket.params[8] = angle;
  okPacket.params[9] = ultra_dist_L;
  okPacket.params[9] = ultra_dist_R;

  if (!failsafe)
  {
    okPacket.command = RESP_FAILSAFE;
  }
  sendResponse(&okPacket);
}

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
  statusPacket.params[8] = angle;
  statusPacket.params[9] = ultra_dist_L;
  statusPacket.params[9] = ultra_dist_R;

  sendResponse(&statusPacket);
}



// Functions to be called by INT0 and INT1 ISRs.
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
}

void rightISR()
{
  if (dir == LEFT)
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

ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}

bool forward(float dist, float val)
{
  dir = FORWARD;
  long targetdist = forwarddist + dist;
  long dist_now = forwarddist;
  loopUSensor();
  while (forwarddist <= targetdist)
  {
    if (ultra_dist_C >= FAILSAFE || over_ride == OVER_ON)
    {
      //        int val = map(forwarddist, dist_now, targetdist, (long)255 * (MAX_POWER / 100.0), 0);
      analogWrite(LF, min(255, max(MIN_POWER,val)));
      analogWrite(RF, min(255, max(MIN_POWER,val * 1.275)));
      analogWrite(LR, 0);
      analogWrite(RR, 0);
      loopUSensor();
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

bool reverse(float dist, float val)
{
  dir = BACKWARD;
  long targetdist = reversedist + dist;
  long dist_now = reversedist;
  loopUSensor();
  while (reversedist < targetdist )
  {
    //    if (ultra_dist_C >= FAILSAFE)
    //    {
    //int val = map(reversedist, dist_now, targetdist, 200, 0);
    analogWrite(RR, min(255, max(MIN_POWER,val * 1.25)));
    analogWrite(LR, min(255, max(MIN_POWER,val)));
    analogWrite(LF, 0);
    analogWrite(RF, 0);
    loopUSensor();
    //    }
    //    else
    //    {
    //      stop();
    //      return false;
    //    }
  }
  stop();
  return true;
}
bool left(float ang, float val)
{
  dir = LEFT;
  long target_ang = leftangle + ang;
  while (leftangle < target_ang || over_ride == OVER_ON)
  {
    analogWrite(RR, 0);
    analogWrite(LR, min(255, max(MIN_POWER,val)));
    analogWrite(LF, 0);
    analogWrite(RF, min(255, max(MIN_POWER,val)));
  }
  stop();
  return true;
}

bool right(float ang, float val)
{
  dir = RIGHT;
  long target_ang = rightangle + ang;
  while (rightangle < target_ang || over_ride == OVER_ON)
  {
    analogWrite(RR, min(255, max(MIN_POWER,val)));
    analogWrite(LR, 0);
    analogWrite(LF, min(255, max(MIN_POWER,val)));
    analogWrite(RF, 0);
  }
  stop();
  return true;
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  analogWrite(LF, 0);
  analogWrite(RF, 0);
  analogWrite(LR, 0);
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

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  bool all_good = true;
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:

      all_good = forward((float)command->params[0], (float)command->params[1]);
      sendOK(all_good);
      break;

    case COMMAND_REVERSE:

      all_good = reverse((float)command->params[0], (float)command->params[1]);
      sendOK(all_good);
      break;

    case COMMAND_TURN_LEFT:
      all_good = left((float) command->params[0], (float)command->params[1]);
      sendOK(all_good);
      break;

    case COMMAND_TURN_RIGHT:
      all_good = right((float) command->params[0], (float)command->params[1]);
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
      over_ride = command->params[0];
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
//  waitForHello();
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
