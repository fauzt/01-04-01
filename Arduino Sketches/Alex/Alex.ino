#include "serialize.h"
#include <arduino.h>
#include "packet.h"
#include "constants.h"
#include "alexsetup.h"
#include "responses.h"

#include <Wire.h>

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
volatile unsigned long obj_color; // 0 for red, 1 for green, 2 for confused

volatile unsigned long ultra_dist;

/*-----------------------------GYRO Definitions & variables-------------------------------*/

// Uncomment the following line to use a MinIMU-9 v5 or AltIMU-10 v5. Leave commented for older IMUs (up through v4).
#define IMU_V5

// Uncomment the below line to use this axis definition:
// X axis pointing forward
// Y axis pointing to the right
// and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1, 1, 1, -1, -1, -1, 1, 1, 1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition:
// X axis pointing forward
// Y axis pointing to the left
// and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
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

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

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
  {0, 0, 1}};
float Update_Matrix[3][3] = {
  {0, 1, 2},
  {3, 4, 5},
  {6, 7, 8}}; //Gyros here

float Temporary_Matrix[3][3] = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}};

/*-------------------------------End of GYRO Definitions & variables---------------------------------------*/

/*----------------------------------COLOUR Definitions & variables-----------------------------------------*/

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

// Stores the red. green and blue colors
int redColor = 0;
int greenColor = 0;
int blueColor = 0;

/*----------------------------------------------------------------------------------------------------------*/


/*
  send ok function with status
*/
void sendOK(bool failsafe)
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  okPacket.params[0] = forwardticks;
  okPacket.params[1] = reverseticks;
  // okPacket.params[2] = leftturnticks;
  // okPacket.params[3] = rightturnticks;
  okPacket.params[4] = forwarddist;
  okPacket.params[5] = reversedist;
  // okPacket.params[6] = leftangdist;
  // okPacket.params[7] = rightangdist;
  okPacket.params[8] = (unsigned long)ToDeg(yaw + 180);

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
  // statusPacket.params[2] = leftturnticks;
  // statusPacket.params[3] = rightturnticks;
  statusPacket.params[4] = forwarddist;
  statusPacket.params[5] = reversedist;
  // statusPacket.params[6] = leftangdist;
  // statusPacket.params[7] = rightangdist;
  statusPacket.params[8] = (unsigned long)ToDeg(yaw + 180);
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
    forwarddist = (forwardticks / COUNTS_PER_REV) * 20.0;
  }
  else if (dir == BACKWARD)
  {
    reverseticks++;
    reversedist = (reverseticks / COUNTS_PER_REV) * 20.0;
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
  all movement functions will return true if
  successful movemnet and return false if
  failsafe has been triggered
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
    if (ultra_dist >= FAILSAFE)
    {
      int val = map(forwarddist, dist_now, targetdist, (long)255 * (MAX_POWER / 100.0), 0);
      OCR0A = val;
      OCR1B = val;
      ultra_dist = loopUSensor();
    }
    else
    {
      stop();
      return false;
    }
  }
  stop();
  scanColour();
  return true;
}

bool reverse(float dist)
{
  dir = BACKWARD;
  long targetdist = reversedist + dist;
  long dist_now = reversedist;
  while (reversedist <= targetdist)
  {
    if (ultra_dist >= FAILSAFE)
    {
      int val = map(reversedist, dist_now, targetdist, (long)255 * (MAX_POWER / 100.0), 0);
      OCR2A = val;
      OCR0B = val;
      ultra_dist = loopUSensor();
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

bool right(float ang)
{
  dir = RIGHT;

  long initial_ang = (long) ToDeg(yaw + 180); //angular reference from gyro

  long targetang = initial_ang + ang;
  if (targetang > 360) {
    targetang = targetang - 360;
  }

  while (ToDeg(yaw + 180) <= targetang)
  {
    if (ultra_dist >= FAILSAFE)
    {
      int val;
      if (targetang < initial_ang) {

        long mapped_ang = ToDeg(yaw + 180) - initial_ang;
        if (mapped_ang < 0)
          mapped_ang = mapped_ang + 360;

        val = map( mapped_ang , 0, ang, (long)255 * (MAX_POWER / 100.0), (long)255 * (10.0 / 100.0));
              //330 -> 0 -> 30
      }
      else {
        val = map((long)ToDeg(yaw + 180), initial_ang, targetang, (long)255 * (MAX_POWER / 100.0), (long)255 * (10.0 / 100.0));
      }

      OCR0A = val;
      OCR0B = val;
      loopGyro();
      ultra_dist = loopUSensor();
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

bool left(float ang)
{
  dir = LEFT;
  /*TODO:
    map current angle values for corner case [ example: turn 60deg from 30deg to 330deg anticlockwise ]
  */

  long initial_ang = (long) ToDeg(yaw + 180); //angular reference from gyro

  long targetang = initial_ang - ang;
  if (targetang < 0) {
    targetang = targetang + 360;
  }

  while (ToDeg(yaw) <= targetang)
  {
    if (ultra_dist >= FAILSAFE)
    {
      int val;
      if (targetang > initial_ang) {
        long mapped_ang = initial_ang - ToDeg(yaw + 180);
        if (mapped_ang < 0)
          mapped_ang = mapped_ang + 360;

        val = map( mapped_ang , 0, ang, (long)255 * (MAX_POWER / 100.0), (long)255 * (10.0 / 100.0));
              //30 -> 0 -> 330
      }
      else {
        val = map((long)ToDeg(yaw + 180), initial_ang, targetang, (long)255 * (MAX_POWER / 100.0), (long)255 * (10.0 / 100.0)); // (value to map, minrange, maxrange, max_power_range, min_power_range)
      }

      OCR1B = val;
      OCR2A = val;
      loopGyro();
      ultra_dist = loopUSensor();
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

/* Activate colour sensor */
void scanColour() {
  loopColour();
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
  bool all_good = true;
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      all_good = forward((float)command->params[0]);
      /*TODO:
        Check front facing ultrasonic and check for color
        if and only if something is in front of ALEX
      */
      sendOK(all_good);
      break;

    case COMMAND_REVERSE:
      all_good = reverse((float)command->params[0]);
      /*TODO:
        Check front facing ultrasonic and check for color
        if and only if something is in front of ALEX
      */
      sendOK(all_good);
      break;

    case COMMAND_TURN_LEFT:

      all_good = left((float)command->params[0]);
      sendOK(all_good);
      break;

    case COMMAND_TURN_RIGHT:

      all_good = right((float)command->params[0]);
      sendOK(all_good);
      break;

    case COMMAND_STOP:
      stop();
      sendOK(all_good);
      break;

    case COMMAND_GET_STATS:
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      //      clearOneCounter(command->params[0]);
      sendOK(all_good);
      break;


    //TODO: Insert scan colour command
    // all_good = scanColour();
    // send RGB value (redColour, greenColour, blueColour

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
  setupGyro();
  setupUSensor();
  setupColour();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
}

void loop()
{
  loopGyro();

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
