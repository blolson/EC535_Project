#include <stdio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <sys/time.h>
#include "i2c.h"
#include "sensors.h"

#define LSM9DS0_ADDRESS_1_ACCELMAG_READ           0x3b //(00111011) - 29 - Read
#define LSM9DS0_REGISTER_OUT_X_L_M              0x08
#define LSM9DS0_REGISTER_OUT_X_H_M              0x09
#define LSM9DS0_REGISTER_OUT_Y_L_M              0x0A
#define LSM9DS0_REGISTER_OUT_Y_H_M              0x0B
#define LSM9DS0_REGISTER_OUT_Z_L_M              0x0C
#define LSM9DS0_REGISTER_OUT_Z_H_M              0x0D
#define LSM9DS0_REGISTER_OUT_X_L_G              0x28
#define LSM9DS0_REGISTER_OUT_X_H_G              0x29
#define LSM9DS0_REGISTER_OUT_Y_L_G              0x2A
#define LSM9DS0_REGISTER_OUT_Y_H_G              0x2B
#define LSM9DS0_REGISTER_OUT_Z_L_G              0x2C
#define LSM9DS0_REGISTER_OUT_Z_H_G              0x2D
#define LSM9DS0_REGISTER_OUT_X_L_A              0x28
#define LSM9DS0_REGISTER_OUT_X_H_A              0x29
#define LSM9DS0_REGISTER_OUT_Y_L_A              0x2A
#define LSM9DS0_REGISTER_OUT_Y_H_A              0x2B
#define LSM9DS0_REGISTER_OUT_Z_L_A              0x2C
#define LSM9DS0_REGISTER_OUT_Z_H_A              0x2D
#define LSM9DS0_ADDRESS_1_GYRO_READ               0xd7 //(11010111) - 107 - Read
#define LSM9DS0_ADDRESS_1_GYRO_WRITE              0xd6 //(11010110) - Write
#define LSM9DS0_ADDRESS_1_ACCELMAG_READ           0x3b //(00111011) - 29 - Read
#define LSM9DS0_ADDRESS_1_ACCELMAG_WRITE          0x3a //(00111010) - Write
#define LSM9DS0_REGISTER_CTRL_REG1_G            0x20
#define LSM9DS0_REGISTER_CTRL_REG1_XM           0x20
#define LSM9DS0_REGISTER_CTRL_REG7_XM           0x26


coords getMag(int i2c_file)
{
  unsigned char gyro_X_L,gyro_X_H,gyro_Y_L,gyro_Y_H,gyro_Z_L,gyro_Z_H;
  int gyro_X,gyro_Y,gyro_Z;
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_X_L_M, &gyro_X_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_X_H_M, &gyro_X_H);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Y_L_M, &gyro_Y_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Y_H_M, &gyro_Y_H);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Z_L_M, &gyro_Z_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Z_H_M, &gyro_Z_H);
  gyro_X = gyro_X_L + (gyro_X_H << 8);
  gyro_Y = gyro_Y_L + (gyro_Y_H << 8);
  gyro_Z = gyro_Z_L + (gyro_Z_H << 8);
  coords xyz = {MAG, gyro_X, gyro_Y, gyro_Z};
  return xyz;
}

coords getGyro(int i2c_file)
{
  unsigned char gyro_X_L,gyro_X_H,gyro_Y_L,gyro_Y_H,gyro_Z_L,gyro_Z_H;
  int gyro_X,gyro_Y,gyro_Z;
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ >> 1, LSM9DS0_REGISTER_OUT_X_L_G, &gyro_X_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ >> 1, LSM9DS0_REGISTER_OUT_X_H_G, &gyro_X_H);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ >> 1, LSM9DS0_REGISTER_OUT_Y_L_G, &gyro_Y_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ >> 1, LSM9DS0_REGISTER_OUT_Y_H_G, &gyro_Y_H);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ >> 1, LSM9DS0_REGISTER_OUT_Z_L_G, &gyro_Z_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_GYRO_READ >> 1, LSM9DS0_REGISTER_OUT_Z_H_G, &gyro_Z_H);
  gyro_X = gyro_X_L + (gyro_X_H << 8);
  gyro_Y = gyro_Y_L + (gyro_Y_H << 8);
  gyro_Z = gyro_Z_L + (gyro_Z_H << 8);
  coords xyz = {GYRO, gyro_X, gyro_Y, gyro_Z};
  return xyz;
}

coords getAccel(int i2c_file)
{
  unsigned char accel_X_L,accel_X_H,accel_Y_L,accel_Y_H,accel_Z_L,accel_Z_H;
  int accel_X,accel_Y,accel_Z;
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_X_L_A, &accel_X_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_X_H_A, &accel_X_H);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Y_L_A, &accel_Y_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Y_H_A, &accel_Y_H);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Z_L_A, &accel_Z_L);
  get_i2c_register(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_READ >> 1, LSM9DS0_REGISTER_OUT_Z_H_A, &accel_Z_H);
  accel_X = accel_X_L + (accel_X_H << 8);
  accel_Y = accel_Y_L + (accel_Y_H << 8);
  accel_Z = accel_Z_L + (accel_Z_H << 8);
  coords xyz = {ACCEL, accel_X, accel_Y, accel_Z};
  return xyz;
}

void setupGyro(int i2c_file, int address)
{
  //enable gyro continuous update
  set_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_CTRL_REG1_G, 15); //0x0F
}

void setupAccelMag(int i2c_file, int address)
{

  //enable accel to update @ 100Hz
  set_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_CTRL_REG1_XM, 103); //0x67

  //enable mag continuous
  set_i2c_register(i2c_file, address >> 1, LSM9DS0_REGISTER_CTRL_REG7_XM, 0);
}

void setup_all_sensors(int i2c_file)
{
  setupAccelMag(i2c_file, LSM9DS0_ADDRESS_1_ACCELMAG_WRITE);
  setupGyro(i2c_file, LSM9DS0_ADDRESS_1_GYRO_WRITE);
}


void print(coords c) {
  char *sensor = (c.type == ACCEL) ? "ACC" : (c.type == MAG) ? "MAG" : "GYR";
  printf("%s %d %d %d \n", sensor, c.X, c.Y, c.Z);
}

