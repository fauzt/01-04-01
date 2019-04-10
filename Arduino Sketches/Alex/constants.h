#ifndef __CONSTANTS_INC__
#define __CONSTANTS_INC__

/* 
 *  This file containts all the packet types, commands
 *  and status constants
 *  
 */

// Packet types
typedef enum
{
  PACKET_TYPE_COMMAND = 0,
  PACKET_TYPE_RESPONSE = 1,
  PACKET_TYPE_ERROR = 2,
  PACKET_TYPE_MESSAGE = 3,
  PACKET_TYPE_HELLO = 4
} TPacketType;

// Response types. This goes into the command field
typedef enum
{
  RESP_OK = 0,
  RESP_STATUS = 1,
  RESP_BAD_PACKET = 2,
  RESP_BAD_CHECKSUM = 3,
  RESP_BAD_COMMAND = 4,
  RESP_BAD_RESPONSE = 5,
  RESP_FAILSAFE = 6
} TResponseType;

// Commands
// For direction commands, param[0] = distance in cm to move
// param[1] = speed
typedef enum
{
  COMMAND_FORWARD = 0,
  COMMAND_REVERSE = 1,
  COMMAND_TURN_LEFT = 2,
  COMMAND_TURN_RIGHT = 3,
  COMMAND_STOP = 4,
  COMMAND_GET_STATS = 5,
  COMMAND_CLEAR_STATS = 6,
  COMMAND_FSO = 7
} TCommandType;

/*
  Limits for motors/controls
*/
typedef enum
{
  MAX_POWER = 100,
  MAX_ROTATE = 100,
  MIN_POWER = 170,
  FAILSAFE = 7,
}Limits;

/*
  Definitions for motors/encoders
*/
typedef enum
{
  COUNTS_PER_REV = 195,
  WHEEL_CIRC = 20
}Motors;

typedef enum
{
  OVER_ON = 1,
  OVER_OFF = 0
}Override_status;

typedef enum
{
  RED = 1,
  WHITE = 3,
  GREEN = 2,
  UNCLEAR = 4
}Colors;

/*
  Direction definition
*/
typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

#endif
