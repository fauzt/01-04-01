#include <LSM6.h>
#include <LIS3MDL.h>
#include <Wire.h>
LSM6 gyro_acc;
LIS3MDL mag;


void I2C_Init()
{
  Wire.begin();
}

void Gyro_Init()
{
  // Accel_Init() should have already called gyro_acc.init() and enableDefault()
  gyro_acc.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale
}

void Read_Gyro()
{
  gyro_acc.readGyro();

  AN[0] = gyro_acc.g.x;
  AN[1] = gyro_acc.g.y;
  AN[2] = gyro_acc.g.z;

  gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
  gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
  gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
}

void Accel_Init()
{
  gyro_acc.init();
  gyro_acc.enableDefault();
  gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale
}

// Reads x,y and z accelerometer registers
void Read_Accel()
{

  gyro_acc.readAcc();

  AN[3] = gyro_acc.a.x >> 4; // shift right 4 bits to use 12-bit representation (1 g = 256)
  AN[4] = gyro_acc.a.y >> 4;
  AN[5] = gyro_acc.a.z >> 4;

  accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
  accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
  accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
}

void Compass_Init()
{

  mag.init();
  mag.enableDefault();

}

void Read_Compass()
{
  mag.read();

  magnetom_x = SENSOR_SIGN[6] * mag.m.x;
  magnetom_y = SENSOR_SIGN[7] * mag.m.y;
  magnetom_z = SENSOR_SIGN[8] * mag.m.z;
}
