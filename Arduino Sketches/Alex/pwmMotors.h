void setupMotors() {

  DDRB |= 0b00001100;
  DDRD |= 0b01100000;

}

void startMotors() {
  TCNT = 0;
  OCR0A = 0;
  OCR0B = 0;
  OCR1B = 0;
  OCR2A = 0;
  TCCR0A = 0b10100001;
  TCCR1A = 0b10100001;
  TCCR2A = 0b10100001;
  TIMSK0 = 0b110;
  TCCR0B = 0b00000001;
  TCCR1B = 0b00000001;
  TCCR2B = 0b00000001;
}

int pwmVal(float speed)
{
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

void forward(float dist, float speed)
{
  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;

  newDist = forwardDist + deltaDist;

  dir = FORWARD;

  int val = pwmVal(speed);

  OCR0A = val;
  OCR1B = val;
  OCR2A = 0;
  OCR0B = 0;
}

void reverse(float dist, float speed)
{
  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;

  newDist = reverseDist + deltaDist;

  dir = BACKWARD;

  int val = pwmVal(speed);

  OCR0A = 0;
  OCR1B = 0;
  OCR2A = val;
  OCR0B = val;
}

void left(float ang, float speed)
{
  dir = LEFT;

  int val = pwmVal(speed);

  OCR0A = 0;
  OCR1B = val;
  OCR2A = val;
  OCR0B = 0;
}

void right(float ang, float speed)
{
  dir = RIGHT;

  int val = pwmVal(speed);

  OCR0A = val;
  OCR1B = 0;
  OCR2A = 0;
  OCR0B = val;
}

void stop() {
  dir = STOP;

  OCR0A = 0;
  OCR1B = 0;
  OCR2A = 0;
  OCR0B = 0;
}

void handleCommand(TPacket *command)
{
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
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

void moveAlex() {
  if (deltaDist > 0)
  {
    if (dir == FORWARD)
    {
      if (forwardDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD)
    {
      if (reverseDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == STOP)
    {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }
}
